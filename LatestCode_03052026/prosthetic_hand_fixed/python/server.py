"""server.py - aiohttp UI server for the MARS hand.

What this is
------------
Long-running laptop process that:
  * opens both serial ports (XIAO sensors + OpenRB motors/EMG/button),
  * enables the firmware's # EMG sample stream,
  * mirrors firmware mode from # MODE events,
  * on # EMG_FLEX in OBJ_RECOG: runs the existing record_grasp ->
    extract_features -> model.predict pipeline and broadcasts the
    classification result,
  * hosts a tiny HTTP server on localhost:8080 that serves the UI
    HTML and a WebSocket for real-time updates.

The UI is purely a display + serial input client. The hand's modes
and motors are driven by the OpenRB firmware (physical button, EMG)
regardless of whether the UI is connected.

Run
---
    python python/server.py
    # then open http://localhost:8080 in Chrome/Edge.

This is a drop-in replacement for daemon.py when you want the UI.
daemon.py is still useful for headless tests.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from collections import deque
from typing import Optional, Set

import joblib
from aiohttp import web, WSMsgType

from hand import constants as C
from hand import protocol as P
from hand.controller import HandController, detect_boards
from hand.features import extract_features
from hand.grasp import record_grasp
from hand.motor_field import MotorFieldProfile

# UI calibration refresh path (added 2026-05-22). Reuses the EXACT
# same function `python python/run.py` uses to build calibration --
# no duplicated sweep / build / threshold logic.
from calibrate import run_calibration


# ----------------------------------------------------------------
# Paths
# ----------------------------------------------------------------
HERE     = os.path.dirname(os.path.abspath(__file__))
UI_DIR   = os.path.join(HERE, 'ui')
DATA_DIR = os.path.join(HERE, 'data')

PORT = 8080
HOST = 'localhost'

# ----------------------------------------------------------------
# Mode mirror
# ----------------------------------------------------------------
MODE_POWER, MODE_TRIPOD, MODE_OBJ_RECOG, MODE_SCREWING = 0, 1, 2, 3

MODE_FIRMWARE_NAMES = {
    MODE_POWER:     'POWER',
    MODE_TRIPOD:    'TRIPOD',
    MODE_OBJ_RECOG: 'OBJ_RECOG',
    MODE_SCREWING:  'SCREWING',
}

# UI shows lowercase short labels in the vertical mode column.
MODE_UI_NAMES = {
    MODE_POWER:     'power',
    MODE_TRIPOD:    'tripod',
    MODE_OBJ_RECOG: 'object',
    MODE_SCREWING:  'rotate',
}

# ----------------------------------------------------------------
# Class -> UI image id mapping
# ----------------------------------------------------------------
# UI SVG ids use "<softness>_<shape>" while our model classes use
# "<shape>_<softness>". Map between them. unknown_object and no_object
# both render as the "no_object" placeholder per spec.
REAL_OBJECT_CLASSES = {'cube_soft', 'cube_stiff', 'cylinder_soft', 'cylinder_stiff'}

CLASS_TO_UI = {
    'cube_soft':      'soft_cube',
    'cube_stiff':     'stiff_cube',
    'cylinder_soft':  'soft_cylinder',
    'cylinder_stiff': 'stiff_cylinder',
    'no_object':      'no_object',
    'unknown_object': 'no_object',
}

# Human-readable labels.
CLASS_TO_LABEL = {
    'cube_soft':      'soft cube',
    'cube_stiff':     'stiff cube',
    'cylinder_soft':  'soft cylinder',
    'cylinder_stiff': 'stiff cylinder',
}

# ----------------------------------------------------------------
# Serial-monitor filter
# ----------------------------------------------------------------
# Events that earn a line in the monitor panel. Everything else
# (M-samples, sensor S-samples, EMG sample stream, pong, STREAM_*,
# per-motor enum noise) is hidden -- it's either visualised
# elsewhere or just noise.
MONITOR_EVENT_NAMES = {
    'MODE', 'EMG_FLEX', 'EMG_RELEASE',
    'GRASP_START', 'T1_STALL', 'STABLE_DETECTED', 'T2_CAPTURE',
    'GRASP_COMPLETE', 'GRASP_TIMEOUT',
    'CALIB_START', 'CALIB_COMPLETE', 'CALIB_ABORTED',
    'BUTTON_SHORT', 'BUTTON_LONG',
    'M2_AT', 'M2_STALL', 'WRIST_AT',
    'SCREW_STEP', 'SCREW_AUTO_RETURN', 'HOMED', 'ABORTED',
    'ERROR', 'WARN',
    'EMG_STATUS', 'EMG_STREAM_ON', 'EMG_STREAM_OFF',
    # Grip-motor force fail-safe (added 2026-05-20):
    'FORCE_STOP',     # motor frozen because load exceeded LOAD_LIMIT_GRASP
    'PEAK_LOAD',      # diagnostic: peak load/current observed this grasp
    'DIAG',           # '?' command response (load_limit + peaks)
    # Motor recovery + direction commands (added 2026-05-20):
    'REBOOTING',         # dxl.reboot() in progress
    'REBOOTED',          # motor came back online
    'REBOOT_FAIL',       # motor didn't come back after reboot
    'DRIVE_MODE',        # 'I' command response
    'DRIVE_MODE_RESET',  # setup() forced DRIVE_MODE to default
    # EMG input-source toggle + manual trigger (added 2026-05-22):
    'EMG_INPUT',         # E0/E1/E? response or boot announcement
    'EMG_EDGE_IGNORED',  # EMG rising edge while EMG-triggered actuation
                         # is disabled -- diagnostic only, no action
    'BUTTON_DOUBLE',     # two short presses within BUTTON_DOUBLE_WINDOW_MS;
                         # firmware routes through the EMG-flex path
    'SCREWING_ENABLED',  # firmware feature toggle for SCREWING mode
}


# ================================================================
# Multiplexing the OpenRB event stream
# ================================================================
# record_grasp() consumes from ctrl.openrb_evts (it pops + discards
# non-matching events while waiting for GRASP_START / T1_STALL / etc).
# Our pump also wants to broadcast every event. To avoid stealing
# events from each other we install a BroadcastDeque in place of
# ctrl.openrb_evts: it behaves like the original deque but every
# append() also fans the event into a thread-safe queue that the
# async pump drains.
# ----------------------------------------------------------------
import queue as _queue
broadcast_q: _queue.Queue = _queue.Queue(maxsize=20000)


class BroadcastDeque(deque):
    """Deque that also enqueues new items into broadcast_q.

    The reader thread (in hand/controller.py) calls .append() to add
    events. record_grasp() calls .popleft() to consume them. Our
    pump reads from broadcast_q in parallel -- so both consumers see
    every event without contention.
    """
    def append(self, item):
        super().append(item)
        try:
            broadcast_q.put_nowait(item)
        except _queue.Full:
            pass  # pump too slow; drop oldest implicitly by overflowing


def _install_broadcast_deque(ctrl: HandController):
    """Replace ctrl.openrb_evts in-place with a BroadcastDeque.

    Uses ctrl's existing maxlen and migrates any items already
    captured during boot. Also updates the reader thread's private
    reference to the deque.
    """
    new_dq = BroadcastDeque(maxlen=ctrl.openrb_evts.maxlen or 200)
    for item in list(ctrl.openrb_evts):
        new_dq.append(item)
    ctrl.openrb_evts = new_dq
    # The reader thread was started in __post_init__ with a reference
    # to the old deque. Update its private attribute so new appends
    # go through our subclass. This is the only place we reach into
    # HandController internals; everything else uses the public API.
    ctrl._openrb_reader._event_dq = new_dq


# ================================================================
# Application state
# ================================================================
class AppState:
    def __init__(self):
        self.ctrl: Optional[HandController] = None
        self.profile: Optional[MotorFieldProfile] = None
        self.model = None
        self.model_classes: list = []

        self.clients: Set[web.WebSocketResponse] = set()

        self.mode: int = MODE_POWER
        self.in_grasp: bool = False
        self.grasp_count: int = 0

        # EMG input-source state mirrored from firmware. Firmware boots
        # into E0 (manual-trigger mode) and emits "# EMG_INPUT 0" at
        # the end of setup(); we capture that and broadcast to clients.
        # See motor_control.ino "EMG input-source toggle" section.
        self.emg_enabled: bool = False

        # True while a calibration sweep is running (server-side
        # invocation of calibrate.run_calibration). Used to:
        #   * disable EMG_FLEX -> classifier dispatch in OBJ_RECOG
        #     (otherwise a stray flex would try to grasp during the
        #      sweep);
        #   * prevent overlapping calibration sweeps;
        #   * drive the UI's REFRESH CALIB pill state.
        self.in_calibration: bool = False

        # SCREWING feature toggle, mirrored from firmware's
        # # SCREWING_ENABLED boot event. Default True for safety in
        # case firmware is older and doesn't emit the event.
        self.screwing_enabled: bool = True

        self.sensors_present: int = 0
        self.sensors_total:   int = C.NUM_SENSORS   # 4

        self.xiao_port: str = ''
        self.openrb_port: str = ''
        self.boot_t: float = time.monotonic()

    def status_snapshot(self) -> dict:
        """Initial snapshot sent to a new WebSocket client."""
        return {
            'type': 'status',
            'xiao_port':       self.xiao_port,
            'openrb_port':     self.openrb_port,
            'xiao_ok':         self.ctrl is not None,
            'openrb_ok':       self.ctrl is not None,
            'sensors_present': self.sensors_present,
            'sensors_total':   self.sensors_total,
            'mode_n':          self.mode,
            'mode_name':       MODE_FIRMWARE_NAMES.get(self.mode, '?'),
            'mode_ui':         MODE_UI_NAMES.get(self.mode, 'power'),
            'model_ready':     self.model is not None and self.profile is not None,
            'emg_enabled':     self.emg_enabled,
            'in_calibration':  self.in_calibration,
            'screwing_enabled': self.screwing_enabled,
            'uptime_s':        int(time.monotonic() - self.boot_t),
        }


# ================================================================
# Broadcast helpers
# ================================================================
async def _broadcast(state: AppState, msg: dict):
    """Send `msg` (any JSON-serialisable dict) to every connected client."""
    if not state.clients:
        return
    text = json.dumps(msg)
    dead = []
    for ws in state.clients:
        if ws.closed:
            dead.append(ws)
            continue
        try:
            await ws.send_str(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        state.clients.discard(ws)


# ================================================================
# Event handling
# ================================================================
async def _handle_event(state: AppState, evt: P.Event):
    name = evt.name

    # EMG sample stream -> waveform update only (no monitor entry).
    if name == 'EMG' and len(evt.args) >= 3:
        try:
            smoothed  = int(evt.args[1])
            threshold = int(evt.args[2])
        except ValueError:
            return
        await _broadcast(state, {
            'type':      'emg',
            'smoothed':  smoothed,
            'threshold': threshold,
            'above':     1 if smoothed > threshold else 0,
        })
        return

    # MODE change -> mirror + broadcast + monitor entry.
    if name == 'MODE' and evt.args:
        try:
            state.mode = int(evt.args[0])
        except ValueError:
            pass
        await _broadcast(state, {
            'type':    'mode',
            'n':       state.mode,
            'name':    MODE_FIRMWARE_NAMES.get(state.mode, '?'),
            'ui_name': MODE_UI_NAMES.get(state.mode, 'power'),
        })
        await _monitor_line(state, evt)
        return

    # SCREWING_ENABLED -> mirror firmware feature toggle; UI uses this
    # to gray out (or not) the SCREWING entry in the mode column.
    if name == 'SCREWING_ENABLED' and evt.args:
        try:
            state.screwing_enabled = (int(evt.args[0]) != 0)
        except ValueError:
            pass
        await _broadcast(state, {
            'type':    'screwing_enabled',
            'enabled': state.screwing_enabled,
        })
        await _monitor_line(state, evt)
        return

    # EMG_INPUT -> mirror firmware state + broadcast so the UI pill
    # reflects the current value. Firmware is the source of truth; we
    # never optimistically update on click, we wait for this echo.
    if name == 'EMG_INPUT' and evt.args:
        try:
            state.emg_enabled = (int(evt.args[0]) != 0)
        except ValueError:
            pass
        await _broadcast(state, {
            'type':    'emg_input',
            'enabled': state.emg_enabled,
        })
        await _monitor_line(state, evt)
        return

    # EMG_FLEX -> in OBJ_RECOG, kick off classifier (unless we're in
    # the middle of a calibration sweep, in which case a stray flex
    # would race for the serial port -- ignore it).
    if name == 'EMG_FLEX':
        await _monitor_line(state, evt)
        if (state.mode == MODE_OBJ_RECOG
                and not state.in_grasp
                and not state.in_calibration):
            asyncio.create_task(_run_classifier(state))
        return

    # GRASP_START -> tell UI to switch to "classifying" state.
    if name == 'GRASP_START':
        await _monitor_line(state, evt)
        await _broadcast(state, {'type': 'classifying'})
        return

    # Everything else: monitor line if whitelisted.
    if name in MONITOR_EVENT_NAMES:
        await _monitor_line(state, evt)


async def _monitor_line(state: AppState, evt: P.Event):
    await _broadcast(state, {
        'type': 'event',
        'name': evt.name,
        'raw':  evt.raw,
        'args': evt.args,
    })


# ================================================================
# Classifier dispatch
# ================================================================
def _run_grasp_blocking(state: AppState) -> dict:
    """Synchronous grasp + predict. Called from an executor thread."""
    rec = record_grasp(state.ctrl, state.profile,
                       grasp_index=state.grasp_count)

    if not rec.success:
        return {
            'ok':     False,
            'reason': rec.failure_reason or 'no t1/t2',
        }

    feats = extract_features(rec)
    if feats is None:
        return {'ok': False, 'reason': 'feature extraction returned None'}

    try:
        pred = state.model.predict([feats])[0]
    except Exception as e:
        return {'ok': False, 'reason': f'predict error: {e}'}

    conf = 0.0
    proba_map = {}
    try:
        proba = state.model.predict_proba([feats])[0]
        classes_list = list(state.model.classes_)
        conf = float(proba[classes_list.index(pred)])
        proba_map = {cls: float(p) for cls, p in zip(classes_list, proba)}
    except Exception:
        pass

    return {
        'ok':                True,
        'class':             pred,
        'confidence':        conf,
        'probabilities':     proba_map,
        't1_source':         rec.t1_source,
        'time_to_stall_ms':  rec.time_to_stall_ms,
    }


async def _run_classifier(state: AppState):
    state.in_grasp    = True
    state.grasp_count += 1
    try:
        if state.profile is None or state.model is None:
            await _broadcast(state, {
                'type':       'result',
                'ui_id':      'no_object',
                'label':      'no object detected',
                'confidence': 0.0,
                'note':       'model or calibration missing',
            })
            return

        loop = asyncio.get_event_loop()
        out = await loop.run_in_executor(None, _run_grasp_blocking, state)

        if not out['ok']:
            await _broadcast(state, {
                'type':       'result',
                'ui_id':      'no_object',
                'label':      'no object detected',
                'confidence': 0.0,
                'note':       out['reason'],
            })
            return

        cls = out['class']
        # Convert probabilities to UI labels for easy charting.
        probs_pct = {
            CLASS_TO_LABEL.get(k, k): round(v * 100, 1)
            for k, v in out.get('probabilities', {}).items()
        }
        if cls in REAL_OBJECT_CLASSES:
            await _broadcast(state, {
                'type':             'result',
                'ui_id':            CLASS_TO_UI[cls],
                'label':            CLASS_TO_LABEL[cls],
                'confidence':       round(out['confidence'] * 100, 1),
                'class':            cls,
                'probabilities':    probs_pct,
                't1_source':        out['t1_source'],
                'time_to_stall_ms': out['time_to_stall_ms'],
            })
        else:
            # no_object class -- treat as "No Object Detected".
            await _broadcast(state, {
                'type':             'result',
                'ui_id':            'no_object',
                'label':            'no object detected',
                'confidence':       round(out['confidence'] * 100, 1),
                'class':            cls,
                'probabilities':    probs_pct,
                't1_source':        out['t1_source'],
                'time_to_stall_ms': out['time_to_stall_ms'],
            })
    finally:
        state.in_grasp = False


# ================================================================
# Calibration refresh
# ================================================================
# Reuses calibrate.run_calibration -- the same function `python python/
# run.py` calls before its inference loop. The blocking sweep runs in
# an executor thread (it does ctrl.wait_for_event under the hood, plus
# np / numpy work) so the asyncio pump stays responsive.
#
# Why this exists: without a calibration refresh in the UI, server.py
# uses whatever data/calibration.npz is on disk indefinitely. The model
# pipeline is otherwise byte-identical to run.py infer (same record_grasp
# -> extract_features -> model.predict), so a stale profile is the only
# remaining reason UI OBJ_RECOG accuracy would drift below run.py infer.
# See CLAUDE.md "Open items" for prior context.
# ----------------------------------------------------------------
def _run_calibration_blocking(state: AppState) -> dict:
    """Synchronous calibration. Runs in an executor thread."""
    profile_path = os.path.join(DATA_DIR, 'calibration.npz')
    try:
        ok = run_calibration(state.ctrl, profile_path, interactive=False)
    except Exception as e:
        return {'ok': False, 'reason': f'calibration error: {e}'}
    if not ok:
        return {'ok': False, 'reason': 'calibration failed (see server log / monitor)'}
    try:
        profile = MotorFieldProfile.load(profile_path)
    except Exception as e:
        return {'ok': False, 'reason': f'profile reload failed: {e}'}
    # Hand the new profile back to the caller so the state mutation
    # happens in the asyncio thread, not here.
    return {'ok': True, 'profile': profile, 'path': profile_path}


async def _run_calibration(state: AppState):
    """Orchestrate a UI-driven calibration refresh.

    Broadcasts calib_status updates so the UI pill shows progress, runs
    the blocking sweep in an executor, swaps in the fresh profile on
    success, and re-enables motor + EMG streaming afterwards (the
    sweep itself ends with streams_off, which would otherwise leave
    the UI's EMG canvas frozen and motor data stalled).
    """
    if state.ctrl is None:
        await _broadcast(state, {
            'type':  'calib_status',
            'state': 'fail',
            'note':  'no serial connection',
        })
        return
    if state.in_calibration:
        # Another sweep is already in flight; ignore the click.
        return
    if state.in_grasp:
        await _broadcast(state, {
            'type':  'calib_status',
            'state': 'fail',
            'note':  'grasp in progress; try again in a moment',
        })
        return

    state.in_calibration = True
    await _broadcast(state, {
        'type':  'calib_status',
        'state': 'running',
        'note':  'sweeping motor 1 (slow open->close); hand must be EMPTY',
    })
    try:
        loop = asyncio.get_event_loop()
        out = await loop.run_in_executor(None, _run_calibration_blocking, state)

        if out.get('ok'):
            state.profile = out['profile']
            await _broadcast(state, {
                'type':  'calib_status',
                'state': 'success',
                'note':  f'profile reloaded from {os.path.basename(out["path"])}',
            })
        else:
            await _broadcast(state, {
                'type':  'calib_status',
                'state': 'fail',
                'note':  out.get('reason', 'unknown'),
            })
    finally:
        # run_calibration ends with streams_off (sensor + motor 's'/'x'
        # streams). The EMG sample stream ('j'/'k') is NOT touched by
        # streams_off, so we don't need to re-send 'j'. Re-enable the
        # 's' streams so motor data flows again (record_grasp will
        # also re-enable as needed, but bringing them up now matches
        # the original startup state).
        try:
            state.ctrl.streams_on()
        except Exception:
            pass
        state.in_calibration = False


# ================================================================
# Pump task: drains broadcast_q (OpenRB events) + xiao_evts
# ================================================================
async def pump_events(state: AppState):
    loop = asyncio.get_event_loop()

    # Initial uptime ticker (every second) so the status bar updates
    # even when the hand is idle.
    last_uptime_push = 0.0

    while True:
        # Drain OpenRB event broadcast queue (cross-thread safe).
        drained = 0
        while drained < 200:    # cap per tick to keep loop responsive
            try:
                evt = broadcast_q.get_nowait()
            except _queue.Empty:
                break
            await _handle_event(state, evt)
            drained += 1

        # Drain XIAO events (rare).
        if state.ctrl is not None:
            while state.ctrl.xiao_evts:
                evt = state.ctrl.xiao_evts.popleft()
                # Sensor enumeration updates.
                if evt.name == 'ENUM' and evt.args:
                    try:
                        state.sensors_present = int(evt.args[0])
                    except ValueError:
                        pass
                    await _broadcast(state, {
                        'type':    'sensors',
                        'present': state.sensors_present,
                        'total':   state.sensors_total,
                    })
                elif evt.name == 'ENUM_READY':
                    state.sensors_present = state.sensors_total
                    await _broadcast(state, {
                        'type':    'sensors',
                        'present': state.sensors_present,
                        'total':   state.sensors_total,
                    })
                elif evt.name == 'ENUM_FAIL':
                    # args are missing addresses; present = total - missing.
                    state.sensors_present = max(0, state.sensors_total - len(evt.args))
                    await _broadcast(state, {
                        'type':    'sensors',
                        'present': state.sensors_present,
                        'total':   state.sensors_total,
                    })
                if evt.name in MONITOR_EVENT_NAMES or evt.name.startswith('ENUM'):
                    await _monitor_line(state, evt)

        now = time.monotonic()
        if now - last_uptime_push >= 1.0:
            last_uptime_push = now
            await _broadcast(state, {
                'type':     'uptime',
                'uptime_s': int(now - state.boot_t),
            })

        await asyncio.sleep(0.02)   # 50 Hz pump


# ================================================================
# HTTP / WebSocket handlers
# ================================================================
async def handle_index(request):
    return web.FileResponse(os.path.join(UI_DIR, 'index.html'))


async def handle_ws(request):
    state: AppState = request.app['state']
    ws = web.WebSocketResponse(heartbeat=20.0)
    await ws.prepare(request)
    state.clients.add(ws)

    # Send initial state to new client.
    await ws.send_str(json.dumps(state.status_snapshot()))

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except Exception:
                continue
            if data.get('type') == 'serial':
                cmd = (data.get('cmd') or '').strip()
                if not cmd or state.ctrl is None:
                    continue
                try:
                    state.ctrl.send_openrb(cmd)
                except Exception as e:
                    await ws.send_str(json.dumps({
                        'type': 'event',
                        'name': 'ERROR',
                        'raw':  f'# ERROR send failed: {e}',
                        'args': [],
                    }))
                    continue
                # Echo to monitor so user sees what they typed.
                await _broadcast(state, {
                    'type': 'command_echo',
                    'cmd':  cmd,
                })
            elif data.get('type') == 'emg_input':
                # UI pill toggled. Forward to firmware as E0 / E1. We
                # do NOT update state.emg_enabled here; we wait for the
                # firmware's # EMG_INPUT echo so the UI always reflects
                # the actual firmware state.
                if state.ctrl is None:
                    continue
                enable = bool(data.get('enabled'))
                cmd = 'E1' if enable else 'E0'
                try:
                    state.ctrl.send_openrb(cmd)
                except Exception as e:
                    await ws.send_str(json.dumps({
                        'type': 'event',
                        'name': 'ERROR',
                        'raw':  f'# ERROR send failed: {e}',
                        'args': [],
                    }))
                    continue
                await _broadcast(state, {
                    'type': 'command_echo',
                    'cmd':  cmd,
                })
            elif data.get('type') == 'calibrate':
                # UI REFRESH CALIB pill clicked. Spawn the orchestrator
                # as a task so this WS read loop stays responsive while
                # the ~10 s sweep runs. _run_calibration handles the
                # in_calibration flag, the in_grasp guard, the streams
                # restore, and the broadcast updates.
                asyncio.create_task(_run_calibration(state))
    finally:
        state.clients.discard(ws)
    return ws


# ================================================================
# Startup / shutdown
# ================================================================
async def on_startup(app):
    state: AppState = app['state']

    # Load model + calibration (optional -- UI still runs without).
    profile_path = os.path.join(DATA_DIR, 'calibration.npz')
    model_path   = os.path.join(DATA_DIR, 'model.pkl')
    if os.path.exists(profile_path):
        state.profile = MotorFieldProfile.load(profile_path)
        print(f'[server] loaded calibration: {profile_path}', file=sys.stderr)
    else:
        print(f'[server] WARN calibration not found: {profile_path}', file=sys.stderr)
    if os.path.exists(model_path):
        blob = joblib.load(model_path)
        state.model         = blob['model']
        state.model_classes = list(blob.get('classes', []))
        print(f'[server] loaded model: {model_path}  classes={state.model_classes}',
              file=sys.stderr)
    else:
        print(f'[server] WARN model not found: {model_path}', file=sys.stderr)

    # Open serial.
    xiao_port, openrb_port = detect_boards()
    if not (xiao_port and openrb_port):
        print(f'[server] FAIL detect_boards xiao={xiao_port} openrb={openrb_port}',
              file=sys.stderr)
        print( '[server] starting UI anyway; the page will show "DISCONNECTED".',
              file=sys.stderr)
    else:
        state.xiao_port   = xiao_port
        state.openrb_port = openrb_port
        try:
            state.ctrl = HandController(xiao_port=xiao_port, openrb_port=openrb_port)
        except Exception as e:
            print(f'[server] FAIL opening controller: {e}', file=sys.stderr)
            state.ctrl = None

        if state.ctrl is not None:
            xok, ook = state.ctrl.ping_both()
            if not (xok and ook):
                print(f'[server] WARN ping xiao={xok} openrb={ook}', file=sys.stderr)
            _install_broadcast_deque(state.ctrl)
            state.ctrl.streams_on()
            state.ctrl.send_openrb('j')   # turn on EMG sample stream

            # Query XIAO sensor count. Run in executor since
            # wait_for_event is sync + blocking.
            loop = asyncio.get_event_loop()
            state.ctrl.xiao_evts.clear()
            state.ctrl.send_xiao('e')
            evt = await loop.run_in_executor(
                None, state.ctrl.wait_for_event, 'xiao', 'ENUM', 2.0)
            if evt is not None and evt.args:
                try:
                    state.sensors_present = int(evt.args[0])
                except ValueError:
                    state.sensors_present = 0

            # Catch the boot-time MODE event from OpenRB.
            evt = await loop.run_in_executor(
                None, state.ctrl.wait_for_event, 'openrb', 'MODE', 1.0)
            if evt is not None and evt.args:
                try:
                    state.mode = int(evt.args[0])
                except ValueError:
                    pass

            # Catch the boot-time EMG_INPUT event from OpenRB. Firmware
            # boots into manual-trigger mode (E0) and announces it at
            # the end of setup(). If we miss the boot echo (e.g. opened
            # serial late) we fall back to the safe default already set
            # in AppState.__init__ (emg_enabled = False) and explicitly
            # send E? to query firmware state.
            evt = await loop.run_in_executor(
                None, state.ctrl.wait_for_event, 'openrb', 'EMG_INPUT', 1.0)
            if evt is not None and evt.args:
                try:
                    state.emg_enabled = (int(evt.args[0]) != 0)
                except ValueError:
                    pass
            else:
                # Boot echo missed; query explicitly. The response will
                # be processed by _handle_event in the pump loop.
                try:
                    state.ctrl.send_openrb('E?')
                except Exception:
                    pass

    app['pump_task'] = asyncio.create_task(pump_events(state))
    print(f'[server] http://{HOST}:{PORT}', file=sys.stderr)


async def on_cleanup(app):
    state: AppState = app['state']
    task = app.get('pump_task')
    if task:
        task.cancel()
    if state.ctrl is not None:
        try:
            state.ctrl.send_openrb('k')    # stop EMG sample stream
        except Exception:
            pass
        try:
            state.ctrl.close()
        except Exception:
            pass


# ================================================================
# Entry
# ================================================================
def main():
    app = web.Application()
    app['state'] = AppState()

    app.router.add_get('/',   handle_index)
    app.router.add_get('/ws', handle_ws)
    app.router.add_static('/static', UI_DIR)

    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    web.run_app(app, host=HOST, port=PORT, print=None,
                handle_signals=True)


if __name__ == '__main__':
    main()

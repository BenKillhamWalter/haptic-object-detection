"""daemon.py - long-running laptop process for the stage-2 hand.

Role
----
The OpenRB-150 owns all hardware behaviour (EMG, button, LED, modes,
motors). The daemon's job is the parts that have to live on the laptop:

  * Mirror the firmware's current mode (parsed from `# MODE` events).
  * In OBJ_RECOG mode, react to `# EMG_FLEX` by driving the existing
    object-recognition pipeline:
        send 'g'  ->  record_grasp()  ->  extract_features()
                  ->  model.predict() ->  print "RESULT: <class>".
    The UI (future) will consume those RESULT lines and display the
    matching per-class image.
  * Forward observed events to stdout so the user can see the hand
    transition through modes / EMG edges in real time.

Modes other than OBJ_RECOG run entirely on the firmware -- the daemon
just logs them.

Usage
-----
    python python/daemon.py
    python python/daemon.py --profile data/calibration.npz \
                            --model   data/model.pkl
    python python/daemon.py --quiet         # only print RESULT lines

The daemon does not collect data, does not calibrate, does not train.
It assumes calibration.npz + model.pkl already exist (run run.py for
those workflows). For OBJ_RECOG to work end-to-end, both must be
present on disk; for the other three modes only the serial link is
required.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

import joblib

from hand import constants as C
from hand import protocol as P
from hand.controller import HandController, detect_boards
from hand.features import extract_features
from hand.grasp import record_grasp
from hand.motor_field import MotorFieldProfile


# Mirror of the firmware's Mode enum (motor_control.ino).
MODE_POWER     = 0
MODE_TRIPOD    = 1
MODE_OBJ_RECOG = 2
MODE_SCREWING  = 3
MODE_NAMES = {
    MODE_POWER:     'POWER',
    MODE_TRIPOD:    'TRIPOD',
    MODE_OBJ_RECOG: 'OBJ_RECOG',
    MODE_SCREWING:  'SCREWING',
}


def _data_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'data')


def _log(msg: str, quiet: bool = False):
    """Daemon log line. Goes to stderr so stdout stays clean for the
    eventual UI to consume RESULT: lines via line-buffered piping."""
    if not quiet:
        print(msg, file=sys.stderr, flush=True)


def _emit_result(payload: str):
    """The one thing the UI cares about. Always to stdout, always
    flushed, line-prefixed so it's trivially greppable."""
    print(f'RESULT: {payload}', flush=True)


def _run_classifier(ctrl: HandController,
                    profile: MotorFieldProfile,
                    model,
                    grasp_index: int,
                    quiet: bool) -> None:
    """Drive one grasp + predict, emit a RESULT line.

    Reuses the working `record_grasp()` end-to-end -- which sends 'g'
    on the OpenRB and follows GRASP_START / T1 / STABLE / T2 events.
    The firmware MUST be in MODE_OBJ_RECOG (or any mode that leaves
    'g' as a manual override; cmdGrasp is unconditional in firmware).
    """
    _log(f'[daemon] EMG flex -> grasp #{grasp_index} starting...', quiet)
    rec = record_grasp(ctrl, profile, grasp_index=grasp_index)

    if not rec.success:
        reason = rec.failure_reason or 'no t1/t2'
        _emit_result(f'{C.UNKNOWN_CLASS}   (reason: {reason})')
        return

    feats = extract_features(rec)
    if feats is None:
        _emit_result(f'{C.UNKNOWN_CLASS}   (feature extraction returned None)')
        return

    try:
        pred = model.predict([feats])[0]
    except Exception as e:
        _emit_result(f'{C.UNKNOWN_CLASS}   (predict error: {e})')
        return

    # Confidence is best-effort; if predict_proba is missing or the
    # class lookup fails, fall back to the bare prediction.
    conf_str = ''
    try:
        proba = model.predict_proba([feats])[0]
        conf  = float(proba[list(model.classes_).index(pred)])
        conf_str = f'   confidence: {conf * 100:.1f}%'
    except Exception:
        pass

    _emit_result(f'{pred}{conf_str}   t1_source={rec.t1_source}   '
                 f'time_to_stall_ms={rec.time_to_stall_ms}')


def _drain_xiao_events(ctrl: HandController, quiet: bool):
    """Print XIAO-side events (ENUM_*, STREAM_*, ERROR) for visibility."""
    while ctrl.xiao_evts:
        evt = ctrl.xiao_evts.popleft()
        # Filter the firehose: pong is noise, and ENUM is a one-shot
        # boot announcement we'd rather not spam if the XIAO reboots.
        if evt.name in ('pong',):
            continue
        _log(f'[xiao] {evt.raw}', quiet)


def _drain_openrb_events(ctrl: HandController,
                         state: dict,
                         profile: Optional[MotorFieldProfile],
                         model,
                         quiet: bool):
    """Parse OpenRB events. Returns nothing; updates `state` in place.

    `state` keys:
        mode        : int (current mirrored mode)
        grasp_count : int (monotonic counter for RESULT lines)
        in_grasp    : bool (True between GRASP_START and GRASP_COMPLETE;
                            suppresses EMG_FLEX -> classify re-entry).
    """
    while ctrl.openrb_evts:
        evt = ctrl.openrb_evts.popleft()
        name = evt.name

        if name == 'MODE':
            # "# MODE <n> <name>"
            try:
                new_mode = int(evt.args[0])
            except (IndexError, ValueError):
                continue
            old_mode = state.get('mode')
            state['mode'] = new_mode
            label = MODE_NAMES.get(new_mode, '?')
            if old_mode != new_mode:
                _log(f'[daemon] mode -> {new_mode} ({label})', quiet)

        elif name == 'EMG_FLEX':
            # "# EMG_FLEX <ms> <smoothed> <threshold>"
            if not quiet:
                _log(f'[openrb] {evt.raw}', quiet)
            if state.get('mode') == MODE_OBJ_RECOG and not state.get('in_grasp'):
                if profile is None or model is None:
                    _emit_result(f'{C.UNKNOWN_CLASS}   '
                                 f'(OBJ_RECOG flex but model/profile missing)')
                    continue
                state['grasp_count'] += 1
                state['in_grasp'] = True
                try:
                    _run_classifier(ctrl, profile, model,
                                    state['grasp_count'], quiet)
                finally:
                    state['in_grasp'] = False

        elif name == 'EMG_RELEASE':
            if not quiet:
                _log(f'[openrb] {evt.raw}', quiet)

        elif name in ('EMG_INPUT', 'EMG_EDGE_IGNORED', 'BUTTON_DOUBLE'):
            # EMG input-source toggle + manual-trigger diagnostics
            # (added 2026-05-22). Firmware does the gating; daemon
            # only needs to surface these for visibility.
            _log(f'[openrb] {evt.raw}', quiet)

        elif name in ('GRASP_START', 'T1_STALL', 'STABLE_DETECTED',
                      'T2_CAPTURE', 'GRASP_COMPLETE', 'GRASP_TIMEOUT'):
            # record_grasp() consumes these directly when it's the
            # caller. When the firmware fires a grasp we didn't drive
            # (e.g. a different daemon test), these may appear here.
            # Surface them for visibility but don't act.
            _log(f'[openrb] {evt.raw}', quiet)

        elif name in ('SCREW_STEP', 'WRIST_AT', 'M2_AT', 'M2_STALL',
                      'HOMED', 'BUTTON_SHORT', 'BUTTON_LONG', 'ABORTED'):
            _log(f'[openrb] {evt.raw}', quiet)

        elif name == 'ERROR' or name == 'WARN':
            _log(f'[openrb] {evt.raw}', quiet)

        elif name in ('pong', 'STREAM_ON', 'STREAM_OFF', 'MOTOR'):
            # Pure handshake noise; ignore.
            pass

        else:
            # Future-proofing: surface anything we don't recognise.
            _log(f'[openrb] {evt.raw}', quiet)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default=os.path.join(_data_dir(), 'calibration.npz'),
                    help='motor-field calibration .npz (required for OBJ_RECOG mode)')
    ap.add_argument('--model',   default=os.path.join(_data_dir(), 'model.pkl'),
                    help='trained sklearn model .pkl (required for OBJ_RECOG mode)')
    ap.add_argument('--quiet', action='store_true',
                    help='suppress everything except RESULT: lines on stdout')
    args = ap.parse_args()

    # Load profile + model up-front so the OBJ_RECOG path is fast.
    # Both are optional at startup -- the daemon still runs without
    # them; OBJ_RECOG flexes will just emit an UNKNOWN result.
    profile: Optional[MotorFieldProfile] = None
    model = None
    if os.path.exists(args.profile):
        profile = MotorFieldProfile.load(args.profile)
        _log(f'[daemon] loaded profile: {args.profile}', args.quiet)
    else:
        _log(f'[daemon] WARN profile not found: {args.profile}', args.quiet)
        _log( '[daemon] WARN OBJ_RECOG mode will return unknown_object until '
              'you run `python run.py` to calibrate.', args.quiet)
    if os.path.exists(args.model):
        blob    = joblib.load(args.model)
        model   = blob['model']
        classes = blob['classes']
        _log(f'[daemon] loaded model: {args.model}  classes={classes}', args.quiet)
    else:
        _log(f'[daemon] WARN model not found: {args.model}', args.quiet)
        _log( '[daemon] WARN OBJ_RECOG mode will return unknown_object until '
              'you run `python run.py train`.', args.quiet)

    xiao_port, openrb_port = detect_boards()
    if not (xiao_port and openrb_port):
        _log(f'[daemon] FAIL: could not detect boards. '
             f'xiao={xiao_port}  openrb={openrb_port}', args.quiet)
        return 1
    _log(f'[daemon] xiao={xiao_port}  openrb={openrb_port}', args.quiet)

    with HandController(xiao_port=xiao_port, openrb_port=openrb_port) as ctrl:
        xiao_ok, openrb_ok = ctrl.ping_both()
        if not (xiao_ok and openrb_ok):
            _log(f'[daemon] FAIL: ping. xiao={xiao_ok}  openrb={openrb_ok}',
                 args.quiet)
            return 1

        # Streams stay on for the lifetime of the daemon. record_grasp()
        # toggles them as needed, but having them on by default means
        # the user gets live motor data in the log even when idle.
        ctrl.streams_on()

        # Ask the OpenRB for its current mode so we sync at startup.
        # The firmware emits a # MODE line on boot already, but if the
        # daemon connected late we might have missed it. Sending a
        # harmless command that triggers no behaviour is awkward, so
        # just wait briefly for the boot-time MODE event; if it doesn't
        # arrive, default to POWER (which matches firmware default).
        state = {
            'mode':        MODE_POWER,
            'grasp_count': 0,
            'in_grasp':    False,
        }
        evt = ctrl.wait_for_event('openrb', 'MODE', 1.0)
        if evt is not None:
            try:
                state['mode'] = int(evt.args[0])
            except (IndexError, ValueError):
                pass
        _log(f'[daemon] starting in mode {state["mode"]} '
             f'({MODE_NAMES.get(state["mode"], "?")})', args.quiet)
        _log( '[daemon] running. Press Ctrl-C to stop.', args.quiet)

        try:
            while True:
                _drain_xiao_events(ctrl, args.quiet)
                _drain_openrb_events(ctrl, state, profile, model, args.quiet)
                time.sleep(0.01)
        except KeyboardInterrupt:
            _log('\n[daemon] interrupted, shutting down.', args.quiet)

    return 0


if __name__ == '__main__':
    sys.exit(main())

"""record_grasp: orchestrate one complete grasp end-to-end.

Sends 'g' to the OpenRB, accumulates sensor/motor samples and event
markers as they arrive, applies motor-field subtraction to the sensor
stream in real time, runs magnetic-threshold t1 detection on the
subtracted samples, and waits for the OpenRB's STABLE_DETECTED /
T2_CAPTURE / GRASP_COMPLETE event markers.

Returns a GraspRecord with everything features.py needs.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from . import constants as C
from . import protocol as P
from .controller import HandController
from .motor_field import MotorFieldProfile


@dataclass
class GraspRecord:
    """All raw streams + detection metadata for one grasp."""
    label:           Optional[str] = None
    session_id:      Optional[int] = None
    grasp_index:     Optional[int] = None

    # Raw streams during the grasp window.
    sensor_samples:  List[P.SensorSample]      = field(default_factory=list)
    motor_samples:   List[P.MotorSample]       = field(default_factory=list)

    # Subtracted sensor values aligned 1:1 with sensor_samples.
    # Shape: (N, NUM_SENSORS, NUM_AXES)
    subtracted:      Optional[np.ndarray] = None

    # Event timing in laptop wall-clock (time.monotonic) seconds.
    t_start:         Optional[float] = None
    t1_time:         Optional[float] = None
    t1_source:       Optional[str]   = None  # 'magnetic' or 'encoder_stall' or None
    stable_time:     Optional[float] = None
    t2_time:         Optional[float] = None
    t_complete:      Optional[float] = None

    # Indices into sensor_samples for t1 / t2.
    t1_idx:          Optional[int]   = None
    stable_idx:      Optional[int]   = None
    t2_idx:          Optional[int]   = None

    # Encoder / load / current snapshots at t1 / t2.
    enc_t1:          Optional[float] = None
    enc_t2:          Optional[float] = None
    load_t2:         Optional[int]   = None
    current_t2:      Optional[int]   = None

    # Time from t_start to STABLE_DETECTED (the canonical 'time_to_stall_ms').
    time_to_stall_ms: Optional[int]  = None

    success:         bool = False         # both t1 and t2 captured
    failure_reason:  Optional[str] = None


# ----------------------------------------------------------------
# Real-time magnetic-threshold t1 detector
# ----------------------------------------------------------------
class _MagneticT1Detector:
    """Tracks consecutive over-threshold samples per axis.

    The first axis on the first sensor to reach CONTACT_CONFIRM_SAMPLES
    fires t1. We require the elapsed time since grasp start to exceed
    CONTACT_IGNORE_MS so early transients don't trigger.
    """
    def __init__(self, thresholds: np.ndarray, t_start: float):
        self.thr = np.asarray(thresholds, dtype=np.float32)  # (4,3)
        self.t_start = t_start
        self.streak = np.zeros((C.NUM_SENSORS, C.NUM_AXES), dtype=np.int32)

    def update(self, subtracted_4x3: np.ndarray, recv_t: float):
        """Return True iff t1 should fire this sample."""
        if (recv_t - self.t_start) * 1000.0 < C.CONTACT_IGNORE_MS:
            return False
        over = np.abs(subtracted_4x3) > self.thr
        self.streak = np.where(over, self.streak + 1, 0)
        return bool((self.streak >= C.CONTACT_CONFIRM_SAMPLES).any())


# ----------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------
def _nearest_motor_enc(motor_samples: List[P.MotorSample], recv_t: float) -> float:
    if not motor_samples:
        return float(C.OPEN_POS_DEG)
    # Linear scan is fine for one sample at a time.
    best = motor_samples[0]
    best_dt = abs(motor_samples[0].recv_t - recv_t)
    for m in motor_samples[1:]:
        dt = abs(m.recv_t - recv_t)
        if dt < best_dt:
            best, best_dt = m, dt
    return best.enc_deg


def _index_at_or_after(samples, recv_t: float) -> Optional[int]:
    for i, s in enumerate(samples):
        if s.recv_t >= recv_t:
            return i
    return None


def _index_nearest(samples, recv_t: float) -> Optional[int]:
    """Return the index of the sample closest to recv_t, in either direction.

    Used as a fallback when _index_at_or_after returns None or when we
    care about the closest temporal match more than the directional
    semantics (e.g., for t1 alignment).
    """
    if not samples:
        return None
    best_i, best_dt = 0, abs(samples[0].recv_t - recv_t)
    for i in range(1, len(samples)):
        dt = abs(samples[i].recv_t - recv_t)
        if dt < best_dt:
            best_i, best_dt = i, dt
    return best_i


# ----------------------------------------------------------------
# Main orchestration
# ----------------------------------------------------------------
def record_grasp(
    ctrl:    HandController,
    profile: MotorFieldProfile,
    label:   Optional[str] = None,
    session_id: Optional[int] = None,
    grasp_index: Optional[int] = None,
    timeout_s: float = 12.0,
) -> GraspRecord:
    """Drive one grasp end-to-end. Returns a populated GraspRecord."""

    rec = GraspRecord(label=label, session_id=session_id, grasp_index=grasp_index)
    thresholds = profile.thresholds

    # Drain any stale buffer state.
    ctrl.clear_buffers()
    ctrl.streams_on()
    time.sleep(0.05)
    ctrl.clear_buffers()

    # Trigger the grasp.
    ctrl.send_openrb('g')

    # Wait for GRASP_START.
    start_evt = ctrl.wait_for_event('openrb', 'GRASP_START', 3.0)
    if start_evt is None:
        rec.failure_reason = 'no GRASP_START from OpenRB'
        return rec
    rec.t_start = start_evt.recv_t
    t1_detector = _MagneticT1Detector(thresholds, rec.t_start)

    # Walk the streams until GRASP_COMPLETE arrives (or timeout).
    deadline = rec.t_start + timeout_s
    seen_sensor = 0
    seen_motor  = 0
    while True:
        # Drain motor samples first so subtraction sees fresh encoder data.
        while seen_motor < len(ctrl.motor_dq):
            try:
                rec.motor_samples.append(ctrl.motor_dq[seen_motor])
            except IndexError:
                break
            seen_motor += 1

        # Drain new sensor samples, subtract, run t1 detector.
        while seen_sensor < len(ctrl.sensor_dq):
            try:
                s = ctrl.sensor_dq[seen_sensor]
            except IndexError:
                break
            rec.sensor_samples.append(s)
            enc = _nearest_motor_enc(rec.motor_samples, s.recv_t)
            sub = profile.subtract(s.values, enc)  # (4,3)
            if rec.subtracted is None:
                rec.subtracted = sub[None, :, :].copy()
            else:
                rec.subtracted = np.concatenate(
                    [rec.subtracted, sub[None, :, :]], axis=0)
            if rec.t1_time is None:
                if t1_detector.update(sub, s.recv_t):
                    rec.t1_time   = s.recv_t
                    rec.t1_source = 'magnetic'
                    rec.t1_idx    = seen_sensor
                    rec.enc_t1    = enc
            seen_sensor += 1

        # Drain event markers.
        while ctrl.openrb_evts:
            evt = ctrl.openrb_evts.popleft()
            if evt.name == 'T1_STALL':
                if rec.t1_time is None:
                    rec.t1_time   = evt.recv_t
                    rec.t1_source = 'encoder_stall'
                    rec.t1_idx    = _index_at_or_after(rec.sensor_samples, evt.recv_t)
                    if rec.t1_idx is not None and rec.motor_samples:
                        rec.enc_t1 = _nearest_motor_enc(rec.motor_samples, evt.recv_t)
            elif evt.name == 'STABLE_DETECTED':
                rec.stable_time = evt.recv_t
                rec.stable_idx  = _index_at_or_after(rec.sensor_samples, evt.recv_t)
                rec.time_to_stall_ms = int(round((evt.recv_t - rec.t_start) * 1000.0))
            elif evt.name == 'T2_CAPTURE':
                rec.t2_time   = evt.recv_t
                rec.t2_idx    = _index_at_or_after(rec.sensor_samples, evt.recv_t)
                if len(evt.args) >= 3:
                    try:
                        rec.enc_t2     = float(evt.args[1])
                        rec.load_t2    = int(evt.args[2])
                        rec.current_t2 = int(evt.args[3]) if len(evt.args) >= 4 else None
                    except (ValueError, IndexError):
                        pass
            elif evt.name == 'GRASP_COMPLETE':
                rec.t_complete = evt.recv_t
            elif evt.name == 'GRASP_TIMEOUT':
                rec.failure_reason = 'OpenRB GRASP_TIMEOUT'

        if rec.t_complete is not None:
            break
        if time.monotonic() > deadline:
            rec.failure_reason = rec.failure_reason or 'orchestrator timeout'
            break
        time.sleep(0.005)

    # Final indices fallback. By this point all sensor samples for the
    # grasp window are in rec.sensor_samples, so a lookup that failed
    # in-loop (because the next sensor sample hadn't arrived yet at the
    # time the event was processed) succeeds here.
    #
    # t1_idx historically had no fallback here, which caused failures
    # for fast events at slow sensor rates: T1_STALL fires ~340 ms in,
    # but at 7 Hz the first sensor sample at-or-after 340 ms might
    # not arrive until ~490 ms -- well after we'd already processed
    # the event. Now both at_or_after and nearest are tried so the
    # encoder-stall t1 always maps to some sample.
    if rec.t1_idx is None and rec.t1_time is not None:
        rec.t1_idx = _index_at_or_after(rec.sensor_samples, rec.t1_time)
        if rec.t1_idx is None:
            rec.t1_idx = _index_nearest(rec.sensor_samples, rec.t1_time)
        # Also re-resolve enc_t1 if we missed it in-loop.
        if rec.t1_idx is not None and rec.enc_t1 is None and rec.motor_samples:
            rec.enc_t1 = _nearest_motor_enc(rec.motor_samples, rec.t1_time)
    if rec.t2_idx is None and rec.t2_time is not None:
        rec.t2_idx = _index_at_or_after(rec.sensor_samples, rec.t2_time)
        if rec.t2_idx is None:
            rec.t2_idx = _index_nearest(rec.sensor_samples, rec.t2_time)
    if rec.stable_idx is None and rec.stable_time is not None:
        rec.stable_idx = _index_at_or_after(rec.sensor_samples, rec.stable_time)
        if rec.stable_idx is None:
            rec.stable_idx = _index_nearest(rec.sensor_samples, rec.stable_time)

    rec.success = (rec.t1_time is not None and rec.t2_time is not None
                   and rec.t2_idx is not None and rec.t2_idx < len(rec.sensor_samples))

    if not rec.success and rec.failure_reason is None:
        if rec.t1_time is None:
            rec.failure_reason = 'no t1 detected'
        elif rec.t2_time is None:
            rec.failure_reason = 'no t2 captured'
        else:
            rec.failure_reason = 't2 index out of range'

    return rec

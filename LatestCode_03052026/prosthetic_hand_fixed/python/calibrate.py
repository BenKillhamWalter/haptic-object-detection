"""calibrate.py — run a motor-field calibration sweep and save the profile.

Usage:
    python calibrate.py [--out data/calibration.npz]

Procedure:
  1. Open both serial ports, ping, confirm 4 sensors enumerate.
  2. Prompt the user to confirm the hand is empty.
  3. Send 's' to both boards to start streaming.
  4. Send 'r' to OpenRB which runs the slow sweep and emits
     # CALIB_START / # CALIB_COMPLETE markers.
  5. Build motor_field_profile from sensor samples + motor samples
     received in that window (matched by laptop wall-clock time).
  6. Auto-derive contact thresholds from residual noise.
  7. Save profile + thresholds to .npz.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

from hand import constants as C
from hand.controller import HandController, detect_boards
from hand.motor_field import (
    MotorFieldProfile,
    build_profile_from_sweep,
    derive_thresholds,
)


def _print_thresholds(thr):
    print('[calibrate] auto-derived contact thresholds:')
    for s in range(C.NUM_SENSORS):
        print(f'              s{s}  X={thr[s][0]:5.1f}  Y={thr[s][1]:5.1f}  Z={thr[s][2]:5.1f}')


def run_calibration(ctrl: HandController, out_path: str, interactive: bool = True) -> bool:
    if interactive:
        print('[calibrate] Hand must be EMPTY and motionless. Press ENTER to start sweep.')
        try:
            input('  > ')
        except (EOFError, KeyboardInterrupt):
            print('[calibrate] aborted')
            return False

    ctrl.clear_buffers()
    ctrl.streams_on()
    time.sleep(0.2)
    ctrl.clear_buffers()

    ctrl.send_openrb('r')

    start = ctrl.wait_for_event('openrb', 'CALIB_START', 5.0)
    if start is None:
        print('[calibrate] FAIL: no CALIB_START from OpenRB')
        return False
    print('[calibrate] sweep in progress...')

    complete = ctrl.wait_for_event('openrb', 'CALIB_COMPLETE', 60.0)
    if complete is None:
        print('[calibrate] FAIL: no CALIB_COMPLETE from OpenRB')
        return False
    print('[calibrate] sweep complete.')

    # Take a snapshot of the buffers BEFORE turning streaming off
    # (turning off can race with the last few samples).
    sensor_samples = [s for s in ctrl.sensor_dq if start.recv_t <= s.recv_t <= complete.recv_t]
    motor_samples  = [m for m in ctrl.motor_dq  if start.recv_t <= m.recv_t <= complete.recv_t]
    ctrl.streams_off()

    # Minimum sample requirements account for the achievable XIAO rate
    # (~7-10 Hz at gain=0, osr=2, df=5) and a slow calibration sweep
    # (CALIB_SPEED_RPM = 1.0 → ~7 s sweep + ~2 s reopen = ~9 s window).
    # Expected: ~50-80 sensor samples, ~400-450 motor samples.
    MIN_SENSOR_SAMPLES = 30
    MIN_MOTOR_SAMPLES  = 50

    window_s = max(0.001, complete.recv_t - start.recv_t)
    sensor_hz = len(sensor_samples) / window_s
    motor_hz  = len(motor_samples)  / window_s
    print(f'[calibrate] captured {len(sensor_samples)} sensor ({sensor_hz:.1f} Hz) '
          f'/ {len(motor_samples)} motor ({motor_hz:.1f} Hz) over {window_s:.1f} s')

    if len(sensor_samples) < MIN_SENSOR_SAMPLES or len(motor_samples) < MIN_MOTOR_SAMPLES:
        print(f'[calibrate] FAIL: insufficient samples '
              f'(need >= {MIN_SENSOR_SAMPLES} sensor, >= {MIN_MOTOR_SAMPLES} motor)')
        if sensor_hz < 3.0:
            print('[calibrate]   sensor rate is suspiciously low; check XIAO USB '
                  'and that streaming actually started.')
        return False

    profile = build_profile_from_sweep(sensor_samples, motor_samples)
    populated = int((profile.counts > 0).sum())
    # Need at least ~1/3 of bins covered for nearest-neighbour fill to give
    # sensible interpolation. With 45 bins this is 15.
    MIN_POPULATED_BINS = C.PROFILE_NUM_BINS // 3
    if populated < MIN_POPULATED_BINS:
        print(f'[calibrate] FAIL: only {populated}/{C.PROFILE_NUM_BINS} bins '
              f'populated (need >= {MIN_POPULATED_BINS})')
        return False
    print(f'[calibrate] profile built ({populated}/{C.PROFILE_NUM_BINS} bins '
          f'populated, remaining filled by nearest-neighbour)')

    thr = derive_thresholds(profile, sensor_samples, motor_samples)
    _print_thresholds(thr)

    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    profile.save(out_path)
    print(f'[calibrate] saved -> {out_path}')
    return True


def default_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'data', 'calibration.npz')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=default_path(), help='output .npz path')
    ap.add_argument('--no-interactive', action='store_true', help='skip the ENTER prompt')
    args = ap.parse_args()

    xiao_port, openrb_port = detect_boards()
    if not (xiao_port and openrb_port):
        print('[calibrate] FAIL: could not detect one or both boards')
        return 1

    with HandController(xiao_port=xiao_port, openrb_port=openrb_port) as ctrl:
        ok = run_calibration(ctrl, args.out, interactive=not args.no_interactive)
        return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

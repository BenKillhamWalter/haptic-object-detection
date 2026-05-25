"""collect.py — interactive data collection loop.

Usage:
    python collect.py [--profile data/calibration.npz]
                      [--session N]   (defaults to next free index)

For each grasp the user types a class number (1..5), the script
triggers a grasp on the OpenRB, applies motor-field subtraction in
real time, extracts the 57-feature vector, and appends one row to
the session CSV. Sessions are saved as
    data/grasps_session<N>.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from typing import Optional

import numpy as np

from hand import constants as C
from hand.controller import HandController, detect_boards
from hand.features import extract_features, feature_names
from hand.grasp import record_grasp
from hand.motor_field import MotorFieldProfile


def _data_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'data')


def _next_session_id() -> int:
    """Return the next free session number based on existing CSV files."""
    d = _data_dir()
    if not os.path.isdir(d):
        return 1
    n = 1
    for name in os.listdir(d):
        if name.startswith('grasps_session') and name.endswith('.csv'):
            try:
                k = int(name[len('grasps_session'):-len('.csv')])
                if k >= n: n = k + 1
            except ValueError:
                pass
    return n


def _csv_path(session_id: int) -> str:
    return os.path.join(_data_dir(), f'grasps_session{session_id}.csv')


def _csv_header():
    return ['session_id', 'grasp_index', 'label', 't1_source',
            'time_to_stall_ms', 'enc_t1', 'enc_t2', 'load_t2', 'current_t2',
            'success', 'failure_reason'] + feature_names()


def _prompt_label() -> Optional[str]:
    print('\n[collect] Place object then type:')
    for i, name in enumerate(C.CLASS_NAMES, start=1):
        print(f'           {i}  {name}')
    print('           c  re-calibrate (sweep again)')
    print('           q  quit and save')
    try:
        choice = input('  > ').strip().lower()
    except (EOFError, KeyboardInterrupt):
        return None
    return choice


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default=os.path.join(_data_dir(), 'calibration.npz'))
    ap.add_argument('--session', type=int, default=None)
    args = ap.parse_args()

    if not os.path.exists(args.profile):
        print(f'[collect] FAIL: profile not found at {args.profile}')
        print('         Run calibrate.py first.')
        return 1
    profile = MotorFieldProfile.load(args.profile)
    print(f'[collect] loaded profile from {args.profile}')

    session_id = args.session if args.session is not None else _next_session_id()
    csv_path = _csv_path(session_id)
    print(f'[collect] session_id = {session_id}')
    print(f'[collect] writing to {csv_path}')

    xiao_port, openrb_port = detect_boards()
    if not (xiao_port and openrb_port):
        print('[collect] FAIL: could not detect one or both boards')
        return 1

    grasp_index = 0
    write_header = not os.path.exists(csv_path)

    with HandController(xiao_port=xiao_port, openrb_port=openrb_port) as ctrl, \
         open(csv_path, 'a', newline='') as fh:
        writer = csv.writer(fh)
        if write_header:
            writer.writerow(_csv_header())

        # Sanity-check the hardware before letting anyone press buttons.
        xiao_ok, openrb_ok = ctrl.ping_both()
        if not (xiao_ok and openrb_ok):
            print('[collect] FAIL: ping failed (xiao_ok=%s openrb_ok=%s)' % (xiao_ok, openrb_ok))
            return 1

        while True:
            choice = _prompt_label()
            if choice is None or choice == 'q':
                print('[collect] quitting')
                break
            if choice == 'c':
                # Inline recalibration without leaving the loop.
                from calibrate import run_calibration
                if run_calibration(ctrl, args.profile, interactive=True):
                    profile = MotorFieldProfile.load(args.profile)
                continue
            if choice not in C.LABEL_TO_KEY:
                print('[collect] not a valid label; try again')
                continue
            label = C.LABEL_TO_KEY[choice]

            grasp_index += 1
            print(f'[collect] grasp {grasp_index}: {label} -- recording...')
            rec = record_grasp(
                ctrl, profile,
                label=label,
                session_id=session_id,
                grasp_index=grasp_index,
            )
            if rec.success:
                feats = extract_features(rec)
                if feats is None:
                    print('[collect]   feature extraction failed (saving as failure)')
                    rec.success = False
                    rec.failure_reason = rec.failure_reason or 'feature extract returned None'

            print(f'[collect]   t1_source={rec.t1_source}  time_to_stall_ms={rec.time_to_stall_ms}  '
                  f'success={rec.success}  reason={rec.failure_reason or "-"}')

            row = [
                rec.session_id, rec.grasp_index, rec.label, rec.t1_source,
                rec.time_to_stall_ms, rec.enc_t1, rec.enc_t2,
                rec.load_t2, rec.current_t2,
                int(bool(rec.success)), rec.failure_reason or '',
            ]
            if rec.success and feats is not None:
                row += [float(v) for v in feats]
            else:
                row += [''] * C.NUM_FEATURES
            writer.writerow(row)
            fh.flush()

    print(f'[collect] saved {grasp_index} grasps to {csv_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

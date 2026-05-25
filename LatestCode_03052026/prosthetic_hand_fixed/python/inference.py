"""inference.py — live object-recognition loop.

Usage:
    python inference.py [--profile data/calibration.npz]
                        [--model data/model.pkl]

Workflow:
  1. Load motor-field profile + trained RandomForest.
  2. Open both serial ports, ping, confirm sensors.
  3. Loop: place object, press ENTER -> grasp -> features -> predict.
     If the grasp fails (no t1, no t2, or feature-extract returns
     None), classify as 'unknown_object'.
"""

from __future__ import annotations

import argparse
import os
import sys

import joblib

from hand import constants as C
from hand.controller import HandController, detect_boards
from hand.features import extract_features
from hand.grasp import record_grasp
from hand.motor_field import MotorFieldProfile


def _data_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'data')


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default=os.path.join(_data_dir(), 'calibration.npz'))
    ap.add_argument('--model',   default=os.path.join(_data_dir(), 'model.pkl'))
    args = ap.parse_args()

    if not os.path.exists(args.profile):
        print(f'[inference] FAIL: calibration profile not found: {args.profile}')
        return 1
    if not os.path.exists(args.model):
        print(f'[inference] FAIL: model not found: {args.model}')
        return 1

    profile = MotorFieldProfile.load(args.profile)
    blob    = joblib.load(args.model)
    model   = blob['model']
    classes = blob['classes']
    print(f'[inference] loaded profile + model. classes: {classes}')

    xiao_port, openrb_port = detect_boards()
    if not (xiao_port and openrb_port):
        print('[inference] FAIL: could not detect one or both boards')
        return 1

    grasp_index = 0
    with HandController(xiao_port=xiao_port, openrb_port=openrb_port) as ctrl:
        xiao_ok, openrb_ok = ctrl.ping_both()
        if not (xiao_ok and openrb_ok):
            print('[inference] FAIL: boards did not ping')
            return 1
        print('[inference] ready. Place an object then press ENTER. Type q + ENTER to quit.')
        while True:
            try:
                line = input('  > ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if line == 'q':
                break

            grasp_index += 1
            print(f'[inference] grasp {grasp_index}: running...')
            rec = record_grasp(ctrl, profile, grasp_index=grasp_index)

            if not rec.success:
                print(f'[inference]   RESULT: {C.UNKNOWN_CLASS}   '
                      f'(reason: {rec.failure_reason or "no t1/t2"})')
                continue

            feats = extract_features(rec)
            if feats is None:
                print(f'[inference]   RESULT: {C.UNKNOWN_CLASS}   '
                      f'(feature extraction returned None)')
                continue

            pred = model.predict([feats])[0]
            try:
                proba = model.predict_proba([feats])[0]
                conf = float(proba[list(model.classes_).index(pred)])
                print(f'[inference]   RESULT: {pred}   confidence: {conf * 100:.1f}%   '
                      f't1_source={rec.t1_source}  time_to_stall_ms={rec.time_to_stall_ms}')
            except Exception:
                print(f'[inference]   RESULT: {pred}   t1_source={rec.t1_source}')

    return 0


if __name__ == '__main__':
    sys.exit(main())

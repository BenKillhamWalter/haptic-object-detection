"""run.py — single-entry orchestrator.

    python run.py                 # setup -> calibrate -> collect (default)
    python run.py infer           # setup -> calibrate -> inference
    python run.py train           # just trains on existing CSVs

Walks the user through the standard data-collection day in one go:
detect boards, recover sensors if needed, run motor-field calibration
with auto-threshold derivation, then drop into the interactive
collector. The user can quit any time with 'q' or Ctrl-C; data is
flushed to CSV after every grasp.
"""

from __future__ import annotations

import os
import sys
import time

from hand import constants as C
from hand.controller import HandController, detect_boards


def _data_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'data')


def _heading(text: str):
    print()
    print('=' * 60)
    print(text)
    print('=' * 60)


def _do_setup_check(ctrl: HandController) -> bool:
    """Returns True iff both boards alive and 4 sensors present."""
    xiao_ok, openrb_ok = ctrl.ping_both()
    print(f'[setup] XIAO ping:   {"OK" if xiao_ok else "FAIL"}')
    print(f'[setup] OpenRB ping: {"OK" if openrb_ok else "FAIL"}')
    if not (xiao_ok and openrb_ok):
        return False

    enum = ctrl.enum_xiao()
    if enum is None or not enum.args:
        print('[setup] FAIL: XIAO did not report enum')
        return False
    n_found = int(enum.args[0])
    addrs   = ' '.join(enum.args[1:]) if len(enum.args) > 1 else 'none'
    print(f'[setup] XIAO enum: {n_found}/4 found ({addrs})')

    if n_found < C.NUM_SENSORS:
        print('[setup] not all sensors present, running recovery...')
        ctrl.send_xiao('r')
        ready = ctrl.wait_for_event('xiao', 'ENUM_READY', 5.0)
        if ready is None:
            print('[setup] FAIL: XIAO recovery exhausted')
            return False
        enum2 = ctrl.enum_xiao()
        if enum2 is None or int(enum2.args[0]) < C.NUM_SENSORS:
            print('[setup] FAIL: still missing sensors after recovery')
            return False
        print('[setup] recovery OK: all 4 sensors present')

    motor = ctrl.enum_openrb()
    if motor is None:
        print('[setup] FAIL: OpenRB motor not responding')
        return False
    print(f'[setup] OpenRB motor: {" ".join(motor.args)}')
    return True


def _flow_collect(ctrl: HandController):
    """Calibrate then drop into the interactive collector."""
    from calibrate import run_calibration
    from collect import (
        _csv_path, _csv_header, _next_session_id, _prompt_label
    )
    from hand.features import extract_features
    from hand.grasp import record_grasp
    from hand.motor_field import MotorFieldProfile
    import csv

    _heading('STEP 2 / 3 — Motor-field calibration')
    profile_path = os.path.join(_data_dir(), 'calibration.npz')
    ok = run_calibration(ctrl, profile_path, interactive=True)
    if not ok:
        print('[run] calibration failed; aborting')
        return 1
    profile = MotorFieldProfile.load(profile_path)

    _heading('STEP 3 / 3 — Data collection')
    session_id  = _next_session_id()
    csv_path    = _csv_path(session_id)
    write_header = not os.path.exists(csv_path)
    grasp_index = 0
    print(f'[collect] session_id = {session_id}')
    print(f'[collect] writing to {csv_path}')

    with open(csv_path, 'a', newline='') as fh:
        writer = csv.writer(fh)
        if write_header:
            writer.writerow(_csv_header())
        while True:
            choice = _prompt_label()
            if choice is None or choice == 'q':
                print('[collect] quitting')
                break
            if choice == 'c':
                if run_calibration(ctrl, profile_path, interactive=True):
                    profile = MotorFieldProfile.load(profile_path)
                continue
            if choice not in C.LABEL_TO_KEY:
                print('[collect] not a valid label')
                continue
            label = C.LABEL_TO_KEY[choice]
            grasp_index += 1
            print(f'[collect] grasp {grasp_index}: {label} -- recording...')
            rec = record_grasp(ctrl, profile,
                               label=label, session_id=session_id,
                               grasp_index=grasp_index)
            feats = extract_features(rec) if rec.success else None
            if rec.success and feats is None:
                rec.success = False
                rec.failure_reason = rec.failure_reason or 'feature extract None'
            print(f'[collect]   t1_source={rec.t1_source}  '
                  f'time_to_stall_ms={rec.time_to_stall_ms}  '
                  f'success={rec.success}  '
                  f'reason={rec.failure_reason or "-"}')
            row = [rec.session_id, rec.grasp_index, rec.label, rec.t1_source,
                   rec.time_to_stall_ms, rec.enc_t1, rec.enc_t2,
                   rec.load_t2, rec.current_t2,
                   int(bool(rec.success)), rec.failure_reason or '']
            row += [float(v) for v in feats] if (rec.success and feats is not None) else [''] * C.NUM_FEATURES
            writer.writerow(row)
            fh.flush()

    print(f'[run] saved {grasp_index} grasps to {csv_path}')
    return 0


def _flow_infer(ctrl: HandController):
    """Calibrate then run live inference."""
    from calibrate import run_calibration
    from hand.features import extract_features
    from hand.grasp import record_grasp
    from hand.motor_field import MotorFieldProfile
    import joblib

    model_path = os.path.join(_data_dir(), 'model.pkl')
    if not os.path.exists(model_path):
        print(f'[run] FAIL: no trained model at {model_path}. Run "python run.py train" first.')
        return 1

    _heading('STEP 2 / 3 — Motor-field calibration')
    profile_path = os.path.join(_data_dir(), 'calibration.npz')
    ok = run_calibration(ctrl, profile_path, interactive=True)
    if not ok:
        return 1
    profile = MotorFieldProfile.load(profile_path)

    _heading('STEP 3 / 3 — Live inference')
    blob    = joblib.load(model_path)
    model   = blob['model']
    classes = blob['classes']
    print(f'[infer] classes: {classes}')
    print('[infer] place object, press ENTER. q + ENTER to quit.')

    grasp_index = 0
    while True:
        try:
            line = input('  > ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if line == 'q':
            break
        grasp_index += 1
        rec = record_grasp(ctrl, profile, grasp_index=grasp_index)
        if not rec.success:
            print(f'[infer]   RESULT: {C.UNKNOWN_CLASS}   reason: {rec.failure_reason or "-"}')
            continue
        feats = extract_features(rec)
        if feats is None:
            print(f'[infer]   RESULT: {C.UNKNOWN_CLASS}   (feature extract None)')
            continue
        pred = model.predict([feats])[0]
        try:
            proba = model.predict_proba([feats])[0]
            conf = float(proba[list(model.classes_).index(pred)])
            print(f'[infer]   RESULT: {pred}   confidence: {conf * 100:.1f}%')
        except Exception:
            print(f'[infer]   RESULT: {pred}')
    return 0


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) > 1 else 'collect'
    if mode == 'train':
        # No hardware needed for training.
        from train import main as train_main
        return train_main()

    if mode not in ('collect', 'infer'):
        print(f'[run] unknown mode "{mode}". Valid: collect | infer | train')
        return 2

    _heading('STEP 1 / 3 — Hardware setup')
    print('[run] detecting boards...')
    xiao_port, openrb_port = detect_boards()
    if not (xiao_port and openrb_port):
        print(f'[run] FAIL: detection -- xiao={xiao_port} openrb={openrb_port}')
        return 1
    print(f'[run] XIAO on {xiao_port},  OpenRB on {openrb_port}')

    with HandController(xiao_port=xiao_port, openrb_port=openrb_port) as ctrl:
        if not _do_setup_check(ctrl):
            print('[run] hardware setup failed; aborting')
            return 1
        if mode == 'collect':
            return _flow_collect(ctrl)
        else:
            return _flow_infer(ctrl)


if __name__ == '__main__':
    sys.exit(main())

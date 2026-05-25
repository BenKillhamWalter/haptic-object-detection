"""57-feature extraction.  SINGLE SOURCE OF TRUTH used by both
collect.py (writing features to CSV alongside raw streams) and
inference.py (computing features for live model.predict()).

Layout (matches CONTEXT.md section 5, ported to the new pipeline):

    indices  0..11   peak_abs of 12 magnetic channels over the grasp
    indices 12..23   mean_abs of 12 channels
    indices 24..35   RMS      of 12 channels
    indices 36..47   raw subtracted deltas at t2 (s0_dx ... s3_dz)
    indices 48..51   per-sensor magnitude at t2 (sqrt(dx^2+dy^2+dz^2))
    index   52       enc_t2 (encoder angle at t2)
    index   53       load_t2
    index   54       current_t2
    index   55       time_to_stall_ms (stable-detection time, NOT t1)
    index   56       enc_t1 (encoder angle when t1 fired)

Channel order within the 12-channel blocks:
    s0_dx, s0_dy, s0_dz, s1_dx, ..., s3_dz
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from . import constants as C
from .grasp import GraspRecord


def extract_features(rec: GraspRecord) -> Optional[np.ndarray]:
    """Compute the 57-feature vector for one grasp.

    Returns None if the record is too incomplete to feature-extract
    (caller should treat that as 'unknown_object' or skip the row).
    """
    if not rec.success or rec.subtracted is None:
        return None
    if rec.t1_idx is None or rec.t2_idx is None:
        return None
    if rec.t2_idx <= rec.t1_idx:
        return None

    # Slice grasp window: from t1 to t2 inclusive.
    window = rec.subtracted[rec.t1_idx:rec.t2_idx + 1]   # (N, 4, 3)
    flat   = window.reshape(window.shape[0], C.NUM_CHANNELS)  # (N, 12)
    if flat.shape[0] < 1:
        return None

    peak_abs = np.max(np.abs(flat), axis=0)              # (12,)
    mean_abs = np.mean(np.abs(flat), axis=0)             # (12,)
    rms      = np.sqrt(np.mean(flat ** 2, axis=0))       # (12,)

    # Raw deltas at t2 (flattened 4x3 -> 12).
    deltas_t2 = rec.subtracted[rec.t2_idx].reshape(C.NUM_CHANNELS)  # (12,)

    # Per-sensor magnitudes at t2.
    mag_t2 = np.zeros(C.NUM_SENSORS, dtype=np.float32)
    sub_t2_sa = rec.subtracted[rec.t2_idx]  # (4, 3)
    for si in range(C.NUM_SENSORS):
        mag_t2[si] = float(np.sqrt(np.sum(sub_t2_sa[si] ** 2)))

    # Scalar features.
    enc_t2     = float(rec.enc_t2) if rec.enc_t2 is not None else 0.0
    load_t2    = float(rec.load_t2) if rec.load_t2 is not None else 0.0
    current_t2 = float(rec.current_t2) if rec.current_t2 is not None else 0.0
    tts_ms     = float(rec.time_to_stall_ms) if rec.time_to_stall_ms is not None else 0.0
    enc_t1     = float(rec.enc_t1) if rec.enc_t1 is not None else 0.0

    feats = np.concatenate([
        peak_abs.astype(np.float32),
        mean_abs.astype(np.float32),
        rms.astype(np.float32),
        deltas_t2.astype(np.float32),
        mag_t2.astype(np.float32),
        np.array([enc_t2, load_t2, current_t2, tts_ms, enc_t1], dtype=np.float32),
    ])

    if feats.shape[0] != C.NUM_FEATURES:
        raise AssertionError(
            f'feature vector has {feats.shape[0]} elements, expected {C.NUM_FEATURES}')
    return feats


# Human-readable names for CSV headers and diagnostics.
def feature_names() -> list:
    names = []
    chan_order = []
    for s in range(C.NUM_SENSORS):
        for a in ('x', 'y', 'z'):
            chan_order.append(f's{s}_d{a}')
    for prefix in ('peak_abs', 'mean_abs', 'rms'):
        for c in chan_order:
            names.append(f'{prefix}__{c}')
    for c in chan_order:
        names.append(f't2__{c}')
    for s in range(C.NUM_SENSORS):
        names.append(f't2__s{s}_mag')
    names += ['enc_t2', 'load_t2', 'current_t2', 'time_to_stall_ms', 'enc_t1']
    assert len(names) == C.NUM_FEATURES, (len(names), C.NUM_FEATURES)
    return names

"""Motor-field profile + automatic threshold derivation.

The Dynamixel rotor's magnetic field reaches the MLX90393 sensors and
varies by tens of microtesla as the motor moves 66 -> 110 deg. We
profile that empty-hand field per encoder-degree bin and subtract it
at runtime; what's left is (mostly) object-contact signal + noise.

After the profile is built, we also derive per-channel contact
thresholds from the residual noise so the magnetic-t1 detector is
tuned to the current rig instead of using hard-coded defaults.

This file owns:
    MotorFieldProfile           - bins, build, save, load, subtract
    derive_thresholds           - residual-noise -> per-channel threshold
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List

import numpy as np

from . import constants as C
from . import protocol as P


# ----------------------------------------------------------------
# Profile container
# ----------------------------------------------------------------
@dataclass
class MotorFieldProfile:
    """Empty-hand magnetic field, binned per encoder degree.

    Layout:
        profile : (PROFILE_NUM_BINS, NUM_SENSORS, NUM_AXES) float32
        counts  : (PROFILE_NUM_BINS,) int   - samples per bin
        thresholds : (NUM_SENSORS, NUM_AXES) float32  - magnetic-t1 thresholds
    """
    profile:    np.ndarray = field(default_factory=lambda: np.zeros(
        (C.PROFILE_NUM_BINS, C.NUM_SENSORS, C.NUM_AXES), dtype=np.float32))
    counts:     np.ndarray = field(default_factory=lambda: np.zeros(
        (C.PROFILE_NUM_BINS,), dtype=np.int32))
    thresholds: np.ndarray = field(default_factory=lambda: np.array(
        C.DEFAULT_CONTACT_THRESHOLDS, dtype=np.float32))

    @property
    def calibrated(self) -> bool:
        return int((self.counts > 0).sum()) >= (C.PROFILE_NUM_BINS // 2)

    # ---- I/O ----
    def save(self, path: str):
        np.savez(path,
                 profile=self.profile,
                 counts=self.counts,
                 thresholds=self.thresholds)

    @classmethod
    def load(cls, path: str) -> 'MotorFieldProfile':
        z = np.load(path)
        p = cls()
        p.profile    = z['profile'].astype(np.float32)
        p.counts     = z['counts'].astype(np.int32)
        p.thresholds = z['thresholds'].astype(np.float32)
        return p

    # ---- runtime use ----
    def _bin_for(self, enc_deg: float) -> int:
        idx = int(round(enc_deg)) - C.PROFILE_BIN_DEG_MIN
        if idx < 0: idx = 0
        if idx >= C.PROFILE_NUM_BINS: idx = C.PROFILE_NUM_BINS - 1
        return idx

    def baseline_for(self, enc_deg: float) -> np.ndarray:
        """Return (NUM_SENSORS, NUM_AXES) baseline for the given encoder position."""
        return self.profile[self._bin_for(enc_deg)]

    def subtract(self, raw_12: Iterable[float], enc_deg: float) -> np.ndarray:
        """raw_12 is the 12-value flat sample (s0x,s0y,s0z,s1x,...).

        Returns a (NUM_SENSORS, NUM_AXES) array of motor-field-subtracted values.
        """
        raw = np.asarray(raw_12, dtype=np.float32).reshape(C.NUM_SENSORS, C.NUM_AXES)
        return raw - self.baseline_for(enc_deg)


# ----------------------------------------------------------------
# Build from a calibration sweep
# ----------------------------------------------------------------
def build_profile_from_sweep(
    sensor_samples: List[P.SensorSample],
    motor_samples:  List[P.MotorSample],
) -> 'MotorFieldProfile':
    """Build a motor-field profile from a slow open->close sweep.

    Each sensor sample is paired with the nearest motor sample by
    receive timestamp (we operate in laptop wall-clock time to avoid
    cross-device clock-offset issues). Sensor values are accumulated
    into a running mean per encoder-degree bin.
    """
    prof = MotorFieldProfile()
    if not sensor_samples or not motor_samples:
        return prof

    motor_t   = np.array([m.recv_t   for m in motor_samples])
    motor_enc = np.array([m.enc_deg  for m in motor_samples])

    sum_xyz = np.zeros((C.PROFILE_NUM_BINS, C.NUM_SENSORS, C.NUM_AXES), dtype=np.float64)
    counts  = np.zeros((C.PROFILE_NUM_BINS,), dtype=np.int64)

    for s in sensor_samples:
        # Nearest motor sample by receive time.
        idx = int(np.searchsorted(motor_t, s.recv_t))
        if idx == 0:
            enc = motor_enc[0]
        elif idx >= len(motor_t):
            enc = motor_enc[-1]
        else:
            enc = motor_enc[idx - 1] if (s.recv_t - motor_t[idx - 1]) < (motor_t[idx] - s.recv_t) else motor_enc[idx]

        # Only count samples inside the active sweep range.
        if enc < C.PROFILE_BIN_DEG_MIN - 0.5 or enc > C.PROFILE_BIN_DEG_MAX + 0.5:
            continue

        bin_idx = int(round(enc)) - C.PROFILE_BIN_DEG_MIN
        if bin_idx < 0 or bin_idx >= C.PROFILE_NUM_BINS:
            continue

        vals = np.asarray(s.values, dtype=np.float64).reshape(C.NUM_SENSORS, C.NUM_AXES)
        sum_xyz[bin_idx] += vals
        counts[bin_idx]  += 1

    # Compute means; nearest-neighbour fill any empty bins.
    mean = np.zeros_like(sum_xyz)
    populated = counts > 0
    mean[populated] = sum_xyz[populated] / counts[populated][:, None, None]
    if not populated.all():
        good_bins = np.where(populated)[0]
        for b in range(C.PROFILE_NUM_BINS):
            if populated[b]:
                continue
            nearest = good_bins[np.argmin(np.abs(good_bins - b))]
            mean[b] = mean[nearest]

    prof.profile = mean.astype(np.float32)
    prof.counts  = counts.astype(np.int32)
    return prof


# ----------------------------------------------------------------
# Automatic threshold derivation from sweep residuals
# ----------------------------------------------------------------
def derive_thresholds(
    profile: MotorFieldProfile,
    sensor_samples: List[P.SensorSample],
    motor_samples:  List[P.MotorSample],
) -> np.ndarray:
    """Set profile.thresholds from the noise floor of the empty-hand sweep.

    For each (sensor, axis), collects |raw - bin_mean| residuals across
    the whole sweep, takes the p99, and sets
        threshold = max(THRESHOLD_FLOOR, p99 * THRESHOLD_SAFETY_FACTOR).
    Returns the new thresholds array (also stored on `profile`).
    """
    if not sensor_samples or not motor_samples:
        return profile.thresholds

    motor_t   = np.array([m.recv_t   for m in motor_samples])
    motor_enc = np.array([m.enc_deg  for m in motor_samples])

    residuals = [[[ ] for _ in range(C.NUM_AXES)] for _ in range(C.NUM_SENSORS)]

    for s in sensor_samples:
        idx = int(np.searchsorted(motor_t, s.recv_t))
        if idx == 0:
            enc = motor_enc[0]
        elif idx >= len(motor_t):
            enc = motor_enc[-1]
        else:
            enc = motor_enc[idx - 1] if (s.recv_t - motor_t[idx - 1]) < (motor_t[idx] - s.recv_t) else motor_enc[idx]
        if enc < C.PROFILE_BIN_DEG_MIN - 0.5 or enc > C.PROFILE_BIN_DEG_MAX + 0.5:
            continue
        sub = profile.subtract(s.values, enc)  # (4,3)
        for si in range(C.NUM_SENSORS):
            for ai in range(C.NUM_AXES):
                residuals[si][ai].append(abs(float(sub[si, ai])))

    new_thr = np.zeros((C.NUM_SENSORS, C.NUM_AXES), dtype=np.float32)
    for si in range(C.NUM_SENSORS):
        for ai in range(C.NUM_AXES):
            if not residuals[si][ai]:
                new_thr[si, ai] = C.DEFAULT_CONTACT_THRESHOLDS[si][ai]
                continue
            p99 = float(np.percentile(residuals[si][ai], 99))
            new_thr[si, ai] = max(C.THRESHOLD_FLOOR, p99 * C.THRESHOLD_SAFETY_FACTOR)

    profile.thresholds = new_thr
    return new_thr

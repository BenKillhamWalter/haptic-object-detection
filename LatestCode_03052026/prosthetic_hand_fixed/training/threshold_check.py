"""
threshold_check.py - Verify CONTACT_THRESHOLDS sizing and CONTACT_ENABLED mask.
==============================================================================

Reads grasp CSVs in ./grasp_data and reports per-axis statistics so you can
size CONTACT_THRESHOLDS and CONTACT_ENABLED (in data_collection.ino) correctly.

What it tells you, per axis (6 sensors x 3 axes = 18 channels):
  - Whether the axis looks DEAD (constant or zero values - sensor not reading)
  - Noise floor:  max(|delta|) seen during pre-t1 region of no_object grasps
                  (or whole grasp if t1 fired immediately)
  - Signal peak:  95th-percentile of |delta| during object grasps loading phase
  - Current threshold value (parsed from data_collection.ino if findable)
  - Mask state:   ON / OFF from CONTACT_ENABLED
  - Recommended:  ~3 x noise floor, rounded up
  - Status flag:  OK / WARNING / DEAD

Plus, across the whole dataset:
  - t1_source distribution per class (magnetic vs encoder_stall vs both)
  - First-trigger sensor frequency for object grasps

Usage:
    python threshold_check.py
    python threshold_check.py --data ./grasp_data
    python threshold_check.py --ino ../data_collection/data_collection.ino
    python threshold_check.py --plot          # optional: matplotlib plots
    python threshold_check.py --grasp 12      # plot just one grasp by id

Dependencies:
    pandas, numpy
    matplotlib (only required if --plot is used)
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

NUM_SENSORS = 4   # Index ID1, Index ID2, Thumb ID3, Thumb ID4
SENSOR_NAMES = ["s0_Idx1", "s1_Idx2", "s2_Thb3", "s3_Thb4"]
AXES = ["dx", "dy", "dz"]
SENSOR_COLS = [f"s{i}_d{a}" for i in range(NUM_SENSORS) for a in "xyz"]

# Fallback values used if data_collection.ino can't be parsed.
DEFAULT_THRESHOLDS = [
    [45.0, 45.0, 160.0],
    [30.0, 40.0, 125.0],
    [45.0, 40.0,  80.0],
    [55.0, 65.0,  75.0],
]
DEFAULT_ENABLED = [
    [False, True,  True],
    [False, False, True],
    [True,  False, False],
    [True,  True,  True],
]


# ----------------------------------------------------------------------
# Parsing CONTACT_THRESHOLDS / CONTACT_ENABLED out of data_collection.ino
# ----------------------------------------------------------------------
def _parse_array_block(source: str, name: str) -> Optional[List[List[str]]]:
    """Find `const ... NAME[NUM_SENSORS][3] = { ... };` and return the NxN raw token grid."""
    pat = re.compile(
        rf"{name}\s*\[\s*{NUM_SENSORS}\s*\]\s*\[\s*3\s*\]\s*=\s*\{{(.*?)\}}\s*;",
        re.DOTALL,
    )
    m = pat.search(source)
    if not m:
        return None
    body = m.group(1)
    # Each row is wrapped in {}.
    rows = re.findall(r"\{([^{}]*)\}", body)
    if len(rows) != NUM_SENSORS:
        return None
    grid = []
    for r in rows:
        toks = [t.strip() for t in r.split(",") if t.strip()]
        if len(toks) != 3:
            return None
        grid.append(toks)
    return grid


def parse_thresholds_from_ino(ino_path: str) -> Tuple[List[List[float]], List[List[bool]]]:
    if not os.path.exists(ino_path):
        print(f"  (data_collection.ino not found at {ino_path}; using defaults)")
        return DEFAULT_THRESHOLDS, DEFAULT_ENABLED

    with open(ino_path, "r", encoding="utf-8", errors="replace") as f:
        src = f.read()

    thr_grid = _parse_array_block(src, "CONTACT_THRESHOLDS")
    en_grid = _parse_array_block(src, "CONTACT_ENABLED")

    if thr_grid is None or en_grid is None:
        print("  (could not parse CONTACT_THRESHOLDS / CONTACT_ENABLED; using defaults)")
        return DEFAULT_THRESHOLDS, DEFAULT_ENABLED

    thr = [[float(t.rstrip("fF")) for t in row] for row in thr_grid]
    en = [[t.lower() == "true" for t in row] for row in en_grid]
    print(f"  parsed thresholds + mask from {ino_path}")
    return thr, en


# ----------------------------------------------------------------------
# Loading CSVs
# ----------------------------------------------------------------------
def load_csvs(data_dir: str) -> pd.DataFrame:
    paths = sorted(glob.glob(os.path.join(data_dir, "*.csv")))
    if not paths:
        print(f"ERROR: no CSV files found in {data_dir}")
        sys.exit(1)

    frames = []
    for p in paths:
        try:
            df = pd.read_csv(p, comment="#")
            df.columns = df.columns.str.strip()
            df["source_file"] = os.path.basename(p)
            df["file_grasp_id"] = (
                df["source_file"] + "::" + df["grasp_id"].astype(int).astype(str)
            )
            # tolerate optional columns
            for c in ["t1_source"]:
                if c not in df.columns:
                    df[c] = 0
            frames.append(df)
            print(f"  loaded {os.path.basename(p):30s} {len(df):5d} rows")
        except Exception as e:
            print(f"  WARNING: failed to read {p}: {e}")

    if not frames:
        sys.exit(1)
    return pd.concat(frames, ignore_index=True)


# ----------------------------------------------------------------------
# Per-axis statistics
# ----------------------------------------------------------------------
def per_axis_stats(df: pd.DataFrame) -> Dict[str, Dict]:
    """
    For each of the 18 channel columns, compute:
        dead:        True if the column has near-zero variance across ALL data
        noise_p99:   99th percentile of |delta| across no_object pre-t1 samples
        noise_max:   max of |delta| across no_object pre-t1 samples
        signal_p95:  95th percentile of |delta| across object loading samples
        signal_max:  max of |delta| across object loading samples
    """
    result: Dict[str, Dict] = {}

    is_no_obj = df["label"].astype(str).str.strip() == "no_object"
    df_no = df[is_no_obj]
    df_obj = df[~is_no_obj]

    # Dead-channel detection: per source_file, check std and unique count
    dead_per_file: Dict[Tuple[str, str], bool] = {}
    for col in SENSOR_COLS:
        for src in df["source_file"].unique():
            vals = df.loc[df["source_file"] == src, col].astype(float)
            if len(vals) == 0:
                continue
            std = float(vals.std())
            uniq = vals.nunique()
            # Heuristic: dead = std < 0.5 AND fewer than 4 unique values
            dead_per_file[(col, src)] = (std < 0.5 and uniq < 4)

    for col in SENSOR_COLS:
        # A channel is "DEAD" if every source file shows constant data for it.
        # If even one source has live data, the channel is reading SOMETIMES.
        per_file_dead = [v for (c, _), v in dead_per_file.items() if c == col]
        all_dead = bool(per_file_dead) and all(per_file_dead)
        any_dead = bool(per_file_dead) and any(per_file_dead)

        # Noise floor: pre-t1 region of no_object grasps.
        noise_vals = []
        for gid, g in df_no.groupby("file_grasp_id", sort=False):
            t1_idx = g.index[g["t1_flag"].astype(int) == 1]
            if len(t1_idx) > 0:
                pre = g.loc[: t1_idx[0] - 1] if t1_idx[0] > g.index[0] else g
            else:
                pre = g
            noise_vals.extend(pre[col].abs().tolist())

        # Signal peak: object grasps, loading phase between t1 and t2 (or whole grasp).
        signal_vals = []
        for gid, g in df_obj.groupby("file_grasp_id", sort=False):
            t1_idx = g.index[g["t1_flag"].astype(int) == 1]
            t2_idx = g.index[g["t2_flag"].astype(int) == 1]
            start = t1_idx[0] if len(t1_idx) else g.index[0]
            end = t2_idx[-1] if len(t2_idx) else g.index[-1]
            window = g.loc[start:end]
            signal_vals.extend(window[col].abs().tolist())

        noise_arr = np.array(noise_vals, dtype=float)
        signal_arr = np.array(signal_vals, dtype=float)

        result[col] = {
            "dead": all_dead,
            "any_dead": any_dead,
            "noise_p99": float(np.percentile(noise_arr, 99)) if len(noise_arr) else float("nan"),
            "noise_max": float(noise_arr.max()) if len(noise_arr) else float("nan"),
            "signal_p95": float(np.percentile(signal_arr, 95)) if len(signal_arr) else float("nan"),
            "signal_max": float(signal_arr.max()) if len(signal_arr) else float("nan"),
            "n_noise_samples": len(noise_arr),
            "n_signal_samples": len(signal_arr),
        }
    return result


# ----------------------------------------------------------------------
# Reporting
# ----------------------------------------------------------------------
def print_axis_table(stats: Dict[str, Dict],
                     thresholds: List[List[float]],
                     enabled: List[List[bool]]) -> None:
    print()
    print("=" * 113)
    print(f"{'sensor':10s} {'axis':4s} {'mask':5s} {'cur_thr':>8s} "
          f"{'noise_p99':>10s} {'noise_max':>10s} {'sig_p95':>10s} {'sig_max':>10s} "
          f"{'rec_thr':>8s}  status")
    print("-" * 113)

    for i in range(NUM_SENSORS):
        for j, axis in enumerate(AXES):
            col = f"s{i}_{axis}"
            s = stats[col]
            mask = "ON" if enabled[i][j] else "OFF"
            cur = thresholds[i][j]
            recommended = float("nan")
            status = "OK"

            if s["dead"]:
                status = "DEAD - sensor not reading"
            elif s["any_dead"]:
                status = "PARTIAL DEAD - some files have no signal"
            elif np.isnan(s["noise_p99"]):
                status = "NO no_object DATA"
            else:
                # Recommended threshold = 3 x p99 noise (rounded up).
                recommended = max(1.0, np.ceil(3.0 * s["noise_p99"]))
                if not enabled[i][j]:
                    status = "masked off (no thr enforcement)"
                elif cur < 1.5 * s["noise_p99"]:
                    status = f"THR TOO LOW (false-trigger risk; raise to >= {recommended:.0f})"
                elif cur > 10 * s["noise_p99"] and not np.isnan(s["signal_p95"]) and cur > s["signal_p95"]:
                    status = f"THR TOO HIGH (signal_p95 = {s['signal_p95']:.1f} below thr; lower to ~{recommended:.0f})"
                elif cur > 5 * s["noise_p99"]:
                    status = "OK (conservative)"
                else:
                    status = "OK"

            rec_str = f"{recommended:.0f}" if not np.isnan(recommended) else "n/a"
            print(f"{SENSOR_NAMES[i]:10s} {axis:4s} {mask:5s} {cur:8.1f} "
                  f"{s['noise_p99']:10.2f} {s['noise_max']:10.2f} "
                  f"{s['signal_p95']:10.2f} {s['signal_max']:10.2f} "
                  f"{rec_str:>8s}  {status}")
    print("=" * 113)


def print_t1_source_distribution(df: pd.DataFrame) -> None:
    print("\n=== t1_source distribution per class ===")
    print(f"  source codes: 0=none   1=magnetic   2=encoder_stall   3=both")
    classes = sorted(df["label"].astype(str).str.strip().unique())
    print(f"\n  {'class':18s}  {'#grasps':>7s}  {'src=0':>6s} {'src=1':>6s} {'src=2':>6s} {'src=3':>6s}  {'pct magnetic':>13s}")
    print("  " + "-" * 75)
    for cls in classes:
        sub = df[df["label"].astype(str).str.strip() == cls]
        # one row per grasp = the row with t1_flag=1
        per_grasp = []
        for gid, g in sub.groupby("file_grasp_id", sort=False):
            t1_rows = g[g["t1_flag"].astype(int) == 1]
            if len(t1_rows):
                per_grasp.append(int(t1_rows.iloc[0]["t1_source"]))
            else:
                per_grasp.append(0)
        n = len(per_grasp)
        c0 = per_grasp.count(0)
        c1 = per_grasp.count(1)
        c2 = per_grasp.count(2)
        c3 = per_grasp.count(3)
        pct_mag = 100.0 * (c1 + c3) / n if n else 0.0
        print(f"  {cls:18s}  {n:7d}  {c0:6d} {c1:6d} {c2:6d} {c3:6d}  {pct_mag:12.1f}%")


def print_first_trigger_sensor(df: pd.DataFrame) -> None:
    """Across object grasps, which sensor was first to cross the magnetic threshold?
    Approximated as: the sensor with maximum |delta| / threshold ratio at the t1 row."""
    print("\n=== First-trigger sensor (object grasps) ===")
    print("  Approximated from per-grasp t1 row: sensor with largest |delta| at t1.")
    obj = df[df["label"].astype(str).str.strip() != "no_object"]
    counts: Dict[int, int] = {i: 0 for i in range(NUM_SENSORS)}
    for gid, g in obj.groupby("file_grasp_id", sort=False):
        t1_rows = g[g["t1_flag"].astype(int) == 1]
        if not len(t1_rows):
            continue
        row = t1_rows.iloc[0]
        sens_max = -1.0
        sens_idx = -1
        for i in range(NUM_SENSORS):
            mag = max(abs(row[f"s{i}_dx"]), abs(row[f"s{i}_dy"]), abs(row[f"s{i}_dz"]))
            if mag > sens_max:
                sens_max = mag
                sens_idx = i
        if sens_idx >= 0:
            counts[sens_idx] += 1

    total = sum(counts.values())
    if total == 0:
        print("  (no object grasps with t1 detected)")
        return
    for i in range(NUM_SENSORS):
        bar = "#" * int(40 * counts[i] / total) if total else ""
        print(f"  {SENSOR_NAMES[i]:10s}  {counts[i]:4d}  {100.0 * counts[i] / total:5.1f}%  {bar}")


# ----------------------------------------------------------------------
# Plotting (optional)
# ----------------------------------------------------------------------
def plot_grasp(df: pd.DataFrame, gid: str,
               thresholds: List[List[float]], enabled: List[List[bool]]) -> None:
    import matplotlib.pyplot as plt  # local import so non-plot runs don't need it

    g = df[df["file_grasp_id"] == gid].reset_index(drop=True)
    if len(g) == 0:
        print(f"  grasp {gid} not found")
        return
    label = str(g["label"].iloc[0]).strip()
    t1_idx = g.index[g["t1_flag"].astype(int) == 1]
    t2_idx = g.index[g["t2_flag"].astype(int) == 1]
    t1_ms = float(g.iloc[t1_idx[0]]["timestamp_ms"]) if len(t1_idx) else None
    t2_ms = float(g.iloc[t2_idx[-1]]["timestamp_ms"]) if len(t2_idx) else None

    fig, axes = plt.subplots(NUM_SENSORS, 3, figsize=(13, 9), sharex=True)
    fig.suptitle(f"Grasp {gid}  label={label}  t1={t1_ms} ms  t2={t2_ms} ms")
    t = g["timestamp_ms"].astype(float)
    for i in range(NUM_SENSORS):
        for j, axis in enumerate(AXES):
            ax = axes[i][j]
            col = f"s{i}_{axis}"
            ax.plot(t, g[col].abs(), linewidth=0.9)
            thr = thresholds[i][j]
            on = enabled[i][j]
            ax.axhline(thr, color="r", linestyle="--", linewidth=0.7,
                       label=f"thr={thr:.0f} {'ON' if on else 'OFF'}")
            if t1_ms is not None:
                ax.axvline(t1_ms, color="g", linestyle=":", linewidth=0.7)
            if t2_ms is not None:
                ax.axvline(t2_ms, color="b", linestyle=":", linewidth=0.7)
            ax.set_title(f"{SENSOR_NAMES[i]} |{axis}|", fontsize=8)
            ax.legend(loc="upper right", fontsize=6)
            ax.tick_params(labelsize=7)
    fig.text(0.5, 0.02, "timestamp_ms", ha="center")
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    plt.show()


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main() -> None:
    # Resolve defaults relative to THIS script's location, not the user's cwd.
    # That way you can run threshold_check.py from anywhere and it always finds
    # the right grasp_data and data_collection.ino in this same project tree.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data = os.path.join(script_dir, "grasp_data")
    default_ino  = os.path.normpath(os.path.join(script_dir, "..", "data_collection", "data_collection.ino"))

    p = argparse.ArgumentParser(description="Verify CONTACT_THRESHOLDS / CONTACT_ENABLED from grasp CSVs")
    p.add_argument("--data", default=default_data, help="folder containing grasp CSVs")
    p.add_argument("--ino", default=default_ino,
                   help="path to data_collection.ino (for parsing thresholds)")
    p.add_argument("--plot", action="store_true", help="plot |delta| vs time per channel")
    p.add_argument("--grasp", default=None, help="plot a single grasp by file_grasp_id")
    args = p.parse_args()

    print(f"Reading thresholds + mask from: {args.ino}")
    thresholds, enabled = parse_thresholds_from_ino(args.ino)

    print(f"\nLoading grasp CSVs from: {args.data}")
    df = load_csvs(args.data)

    print(f"\nTotal rows: {len(df)}   grasps: {df['file_grasp_id'].nunique()}")
    print(f"Classes: {sorted(df['label'].astype(str).str.strip().unique().tolist())}")

    # Dead channel quick check up front - useful when middle finger is silent.
    print("\n=== Dead-channel quick check (per source file) ===")
    print("  A channel is flagged DEAD if std < 0.5 AND fewer than 4 unique values.")
    print(f"  {'channel':10s}", end="")
    for src in df["source_file"].unique():
        print(f"  {src[:18]:18s}", end="")
    print()
    for col in SENSOR_COLS:
        print(f"  {col:10s}", end="")
        for src in df["source_file"].unique():
            vals = df.loc[df["source_file"] == src, col].astype(float)
            std = float(vals.std()) if len(vals) else float("nan")
            uniq = vals.nunique()
            tag = "DEAD" if (std < 0.5 and uniq < 4) else f"std={std:.1f}"
            print(f"  {tag:18s}", end="")
        print()

    stats = per_axis_stats(df)
    print_axis_table(stats, thresholds, enabled)
    print_t1_source_distribution(df)
    print_first_trigger_sensor(df)

    if args.plot or args.grasp:
        # If a specific grasp was named, plot only that one. Otherwise plot
        # one example grasp per class (the first one in CSV order).
        if args.grasp:
            plot_grasp(df, args.grasp, thresholds, enabled)
        else:
            for cls in sorted(df["label"].astype(str).str.strip().unique()):
                first_gid = (
                    df[df["label"].astype(str).str.strip() == cls]
                    ["file_grasp_id"].iloc[0]
                )
                print(f"\nplotting first grasp of class '{cls}' -> {first_gid}")
                plot_grasp(df, first_gid, thresholds, enabled)


if __name__ == "__main__":
    main()

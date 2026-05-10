"""
motor_field_check.py - Verify the motor-rotor-field hypothesis.
==================================================================

The Dynamixel XM430's rotor magnet produces a position-dependent magnetic
field at the MLX90393 sensors. Hypothesis: this rotor field is the dominant
component of what we currently call "sensor delta", swamping the actual
tactile contact signal. If true, a per-encoder-position baseline subtraction
will let weaker contact signals trigger magnetic detection reliably.

This script tests that hypothesis on existing CSVs WITHOUT touching firmware.

What it computes
----------------
1. Pearson correlation between `enc_deg` and each of the 12 sensor channels,
   evaluated across no_object grasps only.
   - |r| > 0.7  -> sensor reading mostly tracks motor position (motor-field
     dominant); subtraction will help significantly.
   - |r| < 0.3  -> reading is independent of motor; subtraction won't help.

2. Per-encoder-bin repeatability of the no_object signal: for each 1-degree
   bin, compute std of sensor value across grasps.
   - Low std -> motor field at that position is highly repeatable; the
     subtraction baseline will be accurate.
   - High std -> noisy / unrepeatable; subtraction is shaky.

3. Estimated contact signal per object class, after motor-field subtraction:
   for each class, mean sensor value at encoder bin minus no_object mean at
   the same bin. The remainder is what the model would see if motor field
   were already removed.

If (1) is high, (2) is low, and (3) is meaningfully nonzero for object
classes, the motor-field-subtraction approach is correct. Proceed to firmware.

Usage
-----
    python motor_field_check.py
    python motor_field_check.py --plot
    python motor_field_check.py --data ./grasp_data
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd

NUM_SENSORS = 4
SENSOR_COLS = [f"s{i}_d{a}" for i in range(NUM_SENSORS) for a in "xyz"]
SENSOR_NAMES = ["Idx1", "Idx2", "Thb3", "Thb4"]


# ----------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------
def load_csvs(data_dir: str) -> pd.DataFrame:
    paths = sorted(glob.glob(os.path.join(data_dir, "*.csv")))
    if not paths:
        print(f"ERROR: no CSV files in {data_dir}")
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
            frames.append(df)
            print(f"  loaded {os.path.basename(p):30s} {len(df):5d} rows")
        except Exception as e:
            print(f"  WARNING: could not read {p}: {e}")

    if not frames:
        sys.exit(1)
    df = pd.concat(frames, ignore_index=True)
    df["label"] = df["label"].astype(str).str.strip()

    # Make sure required columns exist and are numeric.
    needed = ["enc_deg"] + SENSOR_COLS
    for col in needed:
        if col not in df.columns:
            print(f"ERROR: required column '{col}' missing from CSVs")
            sys.exit(1)
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=needed)
    return df


# ----------------------------------------------------------------------
# Hypothesis test 1: correlation between enc_deg and each channel
# ----------------------------------------------------------------------
def correlation_table(df_no_obj: pd.DataFrame) -> None:
    print("\n" + "=" * 78)
    print("1. Pearson correlation: enc_deg vs each sensor channel")
    print("   (no_object grasps only - pure motor-field, no contact)")
    print("=" * 78)
    print(f"{'channel':10s}  {'r':>8s}  {'|r|':>6s}  verdict")
    print("-" * 78)

    enc = df_no_obj["enc_deg"].to_numpy()
    summary = {"strong": 0, "moderate": 0, "weak": 0}

    for col in SENSOR_COLS:
        x = df_no_obj[col].to_numpy()
        if x.std() < 1e-6 or enc.std() < 1e-6:
            r = float("nan")
        else:
            r = float(np.corrcoef(enc, x)[0, 1])
        absr = abs(r) if not np.isnan(r) else 0.0
        if absr >= 0.7:
            verdict = "STRONG  (motor-field dominated)"
            summary["strong"] += 1
        elif absr >= 0.3:
            verdict = "moderate (mixed)"
            summary["moderate"] += 1
        else:
            verdict = "weak    (motor field not dominant)"
            summary["weak"] += 1
        print(f"{col:10s}  {r:+8.3f}  {absr:6.3f}  {verdict}")

    print()
    print(f"  Summary across 12 channels: "
          f"{summary['strong']} strong, "
          f"{summary['moderate']} moderate, "
          f"{summary['weak']} weak.")
    if summary["strong"] >= 6:
        print("  -> Motor-field-subtraction will help substantially.")
    elif summary["strong"] + summary["moderate"] >= 8:
        print("  -> Motor-field-subtraction will help on at least the affected channels.")
    else:
        print("  -> Motor-field-subtraction is unlikely to help much; investigate other causes.")


# ----------------------------------------------------------------------
# Hypothesis test 2: per-encoder-bin repeatability of no_object signal
# ----------------------------------------------------------------------
def build_motor_field_profile(df_no_obj: pd.DataFrame,
                              bin_size_deg: float = 1.0
                              ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Build a per-encoder-bin profile from no_object grasps.

    Returns
    -------
    bin_centers : (Nbins,) encoder positions
    mean_profile : (Nbins, 12) mean sensor value at each bin
    std_profile  : (Nbins, 12) std across grasps at each bin
    counts       : (Nbins,)    samples per bin
    """
    enc = df_no_obj["enc_deg"].to_numpy()
    enc_min = float(np.floor(enc.min()))
    enc_max = float(np.ceil(enc.max()))
    edges = np.arange(enc_min, enc_max + bin_size_deg, bin_size_deg)
    centers = 0.5 * (edges[:-1] + edges[1:])
    nbins = len(centers)

    mean_profile = np.zeros((nbins, 12))
    std_profile = np.full((nbins, 12), np.nan)
    counts = np.zeros(nbins, dtype=int)

    df_no_obj = df_no_obj.copy()
    df_no_obj["enc_bin"] = np.digitize(enc, edges) - 1
    df_no_obj["enc_bin"] = df_no_obj["enc_bin"].clip(0, nbins - 1)

    for b in range(nbins):
        sub = df_no_obj[df_no_obj["enc_bin"] == b]
        if len(sub) == 0:
            continue
        counts[b] = len(sub)
        for k, col in enumerate(SENSOR_COLS):
            vals = sub[col].to_numpy()
            mean_profile[b, k] = vals.mean()
            if len(vals) >= 2:
                std_profile[b, k] = vals.std(ddof=1)

    return centers, mean_profile, std_profile, counts


def repeatability_summary(centers: np.ndarray,
                          mean_profile: np.ndarray,
                          std_profile: np.ndarray,
                          counts: np.ndarray) -> None:
    print("\n" + "=" * 78)
    print("2. Motor-field profile repeatability (no_object only)")
    print(f"   {len(centers)} encoder bins of 1 deg, populated bins: "
          f"{int((counts > 0).sum())}")
    print("=" * 78)
    print(f"{'channel':10s}  {'span(mean)':>12s}  {'avg_std':>10s}  "
          f"{'std/span':>10s}  verdict")
    print("-" * 78)

    populated = counts > 0
    if populated.sum() < 5:
        print("  WARNING: very few populated bins; need more no_object grasps.")
        return

    for k, col in enumerate(SENSOR_COLS):
        means = mean_profile[populated, k]
        stds = std_profile[populated, k]
        stds = stds[~np.isnan(stds)]
        span = float(means.max() - means.min())
        avg_std = float(stds.mean()) if len(stds) else float("nan")

        # Relative noise: how big is the per-bin std vs the across-bins span?
        # < 0.2 means the curve is much bigger than the noise = subtraction works well.
        rel = avg_std / span if span > 1e-6 and not np.isnan(avg_std) else float("nan")

        if np.isnan(rel):
            verdict = "no data"
        elif rel < 0.20:
            verdict = "GOOD  (profile is much bigger than per-bin noise)"
        elif rel < 0.50:
            verdict = "ok    (subtraction will help, residual noise nontrivial)"
        else:
            verdict = "POOR  (profile is comparable to per-bin noise)"
        print(f"{col:10s}  {span:12.2f}  {avg_std:10.2f}  {rel:10.3f}  {verdict}")


# ----------------------------------------------------------------------
# Hypothesis test 3: estimated post-subtraction contact signal per class
# ----------------------------------------------------------------------
def estimated_contact_signal(df: pd.DataFrame,
                             centers: np.ndarray,
                             no_obj_mean_profile: np.ndarray,
                             bin_size_deg: float = 1.0) -> None:
    print("\n" + "=" * 78)
    print("3. Estimated contact signal per class AFTER subtracting motor field")
    print("   (mean delta at each enc bin minus no_object mean at same bin)")
    print("=" * 78)

    classes = sorted(df["label"].unique().tolist())
    classes = [c for c in classes if c != "no_object"]
    if not classes:
        print("  No object-class grasps to compare. Need cube_soft / etc data.")
        return

    enc_min = float(np.floor(df["enc_deg"].min()))
    edges = np.arange(enc_min, centers[-1] + bin_size_deg, bin_size_deg)
    nbins = len(centers)

    print(f"\n{'class':18s}  {'channel':10s}  {'pre-sub max':>11s}  {'post-sub max':>12s}  "
          f"{'gain factor':>11s}")
    print("-" * 78)

    for cls in classes:
        sub = df[df["label"] == cls].copy()
        sub_enc = sub["enc_deg"].to_numpy()
        sub["enc_bin"] = np.digitize(sub_enc, edges) - 1
        sub["enc_bin"] = sub["enc_bin"].clip(0, nbins - 1)

        for k, col in enumerate(SENSOR_COLS):
            # Original peak
            pre_max = float(sub[col].abs().max())

            # After subtraction: residual at each bin = current - no_object_baseline
            grouped = sub.groupby("enc_bin")[col].mean()
            residuals = []
            for b, m in grouped.items():
                if 0 <= b < nbins:
                    residuals.append(m - no_obj_mean_profile[b, k])
            if not residuals:
                continue
            post_max = float(np.max(np.abs(residuals)))

            gain = post_max / pre_max if pre_max > 1e-6 else float("nan")

            # Only print thumb axes (most likely contact) and a couple high-yield index ones
            # to keep output skim-able. Otherwise it's 4 classes x 12 channels = 48 lines.
            interesting = col in ("s2_dx", "s2_dy", "s2_dz",
                                  "s3_dx", "s3_dy", "s3_dz",
                                  "s0_dz", "s1_dz")
            if not interesting:
                continue
            note = ""
            if post_max > 30 and gain > 1.5:
                note = "  <-- contact signal exposed by subtraction"
            elif post_max < 10:
                note = "  (faint after subtraction)"
            print(f"{cls:18s}  {col:10s}  {pre_max:11.2f}  {post_max:12.2f}  "
                  f"{gain:11.2f}{note}")
        print()


# ----------------------------------------------------------------------
# Optional plotting
# ----------------------------------------------------------------------
def plot_traces(df: pd.DataFrame, centers: np.ndarray,
                mean_profile: np.ndarray) -> None:
    import matplotlib.pyplot as plt

    classes = sorted(df["label"].unique().tolist())
    fig, axes = plt.subplots(NUM_SENSORS, 3, figsize=(13, 9), sharex=True)
    fig.suptitle("Sensor reading vs encoder position. "
                 "Bold black = no_object MEAN (the proposed motor-field profile). "
                 "Colored = per-grasp traces by class.")

    colors = {
        "no_object": "0.6",
        "cube_soft": "tab:blue",
        "cube_stiff": "tab:cyan",
        "cylinder_soft": "tab:orange",
        "cylinder_stiff": "tab:red",
    }

    for k, col in enumerate(SENSOR_COLS):
        i = k // 3
        j = k % 3
        ax = axes[i][j]
        for cls in classes:
            sub = df[df["label"] == cls]
            for gid, g in sub.groupby("file_grasp_id"):
                ax.plot(g["enc_deg"], g[col], color=colors.get(cls, "tab:gray"),
                        alpha=0.25, linewidth=0.6)
        ax.plot(centers, mean_profile[:, k], color="black", linewidth=1.5)
        ax.set_title(col, fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)

    # One shared legend
    handles = [plt.Line2D([0], [0], color=c, label=lbl) for lbl, c in colors.items()]
    handles.append(plt.Line2D([0], [0], color="black", linewidth=1.5,
                              label="no_object MEAN"))
    fig.legend(handles=handles, loc="lower center", ncol=6, fontsize=8)
    fig.text(0.5, 0.04, "encoder degrees", ha="center")
    plt.tight_layout(rect=[0, 0.07, 1, 0.95])
    plt.show()


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main() -> None:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data = os.path.join(script_dir, "grasp_data")

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", default=default_data, help="folder of grasp CSVs")
    p.add_argument("--plot", action="store_true",
                   help="also draw 12 sensor-vs-encoder plots")
    p.add_argument("--bin", type=float, default=1.0,
                   help="encoder bin size in degrees (default 1)")
    args = p.parse_args()

    print(f"Reading CSVs from: {args.data}")
    df = load_csvs(args.data)

    no_obj = df[df["label"] == "no_object"]
    print(f"\nTotal rows: {len(df)}   no_object rows: {len(no_obj)}   "
          f"grasps: {df['file_grasp_id'].nunique()}")
    print(f"Classes: {sorted(df['label'].unique().tolist())}")
    print(f"Encoder range: {df['enc_deg'].min():.1f} to {df['enc_deg'].max():.1f} deg")

    if len(no_obj) < 30:
        print("\nWARNING: only "
              f"{len(no_obj)} no_object samples - results may be noisy. "
              "Collect more no_object grasps for a cleaner motor-field profile.")

    correlation_table(no_obj)

    centers, mean_prof, std_prof, counts = build_motor_field_profile(
        no_obj, bin_size_deg=args.bin)
    repeatability_summary(centers, mean_prof, std_prof, counts)

    estimated_contact_signal(df, centers, mean_prof, bin_size_deg=args.bin)

    if args.plot:
        plot_traces(df, centers, mean_prof)


if __name__ == "__main__":
    main()

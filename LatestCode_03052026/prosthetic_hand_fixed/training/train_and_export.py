"""
train_and_export.py - Random Forest training pipeline
=====================================================

Loads grasp CSV data, extracts a robust feature vector per grasp,
trains a Random Forest classifier, evaluates it using cross-validation,
and exports C++ files for OpenRB-150 inference.

Classes:
    cube_soft       - soft cube
    cube_stiff      - stiff cube
    cylinder_soft   - soft cylinder
    cylinder_stiff  - stiff cylinder
    no_object       - empty / failed grasp (mechanical end-stop closes hand)
    (additional classes can be added by extending VALID_LABELS)

The harness exposes 'unknown_object' at INFERENCE TIME ONLY when the
predicted top-1 class confidence is below a threshold. 'no_object' is a
trained class; 'unknown_object' is a runtime/post-processing decision.

Feature vector (NUM_FEATURES = 57):
-----------------------------------
4 sensors (Index ID1, Index ID2, Thumb ID3, Thumb ID4) x 3 axes = 12 channels.

A. Time-series statistics over the whole grasp (36)
   - peak_abs (12), mean_abs (12), RMS (12) for each of the 12 magnetic deltas
B. Final state at t2 (16)
   - 12 raw deltas at t2
   - 4 magnitudes at t2
C. Final motor/encoder state (3)
   - enc_t2 (encoder angle at t2)
   - load_t2
   - current_t2
D. Contact timing (2)
   - time_to_stall_ms
   - enc_t1

t1_source (0=none, 1=magnetic, 2=encoder_stall, 3=both) is read from CSV
and printed for debugging but is NOT a training feature.

Usage:
    pip install pandas scikit-learn matplotlib micromlgen

    python train_and_export.py
    python train_and_export.py --trees 25 --depth 8
    python train_and_export.py --data ./grasp_data --out ../3_inference
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from dataclasses import dataclass
from typing import Iterable, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import ConfusionMatrixDisplay, classification_report, confusion_matrix
from sklearn.model_selection import StratifiedKFold, cross_val_predict, cross_val_score
from sklearn.preprocessing import LabelEncoder

try:
    from micromlgen import port
except ImportError:
    print("ERROR: micromlgen is not installed. Run: pip install micromlgen")
    sys.exit(1)

VALID_LABELS = {
    "cube_soft",
    "cube_stiff",
    "cylinder_soft",
    "cylinder_stiff",
    "no_object",
}

# LabelEncoder sorts classes. We use that same sorted order everywhere and
# export it to class_names.h so inference.ino cannot drift from training.
CLASS_NAMES = sorted(VALID_LABELS)

NUM_SENSORS = 4   # Index ID1, Index ID2, Thumb ID3, Thumb ID4 (middle finger removed)
SENSOR_COLS = [
    "s0_dx", "s0_dy", "s0_dz",
    "s1_dx", "s1_dy", "s1_dz",
    "s2_dx", "s2_dy", "s2_dz",
    "s3_dx", "s3_dy", "s3_dz",
]

# Required columns for a CSV to be loadable.
BASE_REQUIRED = [
    "grasp_id", "label", "timestamp_ms", "enc_deg",
    *SENSOR_COLS,
    "t1_flag", "t2_flag", "t1_ms", "t2_ms",
]

# Optional columns. If absent, default to 0 / 0.0.
OPTIONAL_COLUMNS = [
    "load_t1", "load_t2", "current_t1", "current_t2",
    "enc_t1", "time_to_stall_ms",
    "t1_source",   # DEBUG ONLY - not used as feature
]


@dataclass
class Dataset:
    X: np.ndarray
    y: np.ndarray
    feature_names: List[str]
    skipped: List[Tuple[str, str, str]]
    t1_source_counts: dict


def magnitude_xyz(dx: float, dy: float, dz: float) -> float:
    return float(np.sqrt(dx * dx + dy * dy + dz * dz))


def extract_features_from_grasp(df_grasp: pd.DataFrame) -> Tuple[np.ndarray | None, List[str] | str, int]:
    """
    Extract one robust feature vector from one grasp.

    Returns (features, feature_names_or_error, t1_source_for_grasp).

    Falls back to first sample as t1 and last sample as t2 when the firmware
    failed to flag them (e.g. legacy CSV files or pure no_object grasps with
    very early end-stop). The fallback is acceptable for no_object because
    'no contact' is itself a feature; for object classes it lets the model
    use whatever signal it can find.
    """
    if len(df_grasp) < 3:
        return None, "grasp has fewer than 3 samples", 0

    sensor_arr = df_grasp[SENSOR_COLS].to_numpy(dtype=np.float32)  # (N, 12)

    # ------------------------------------------------------------------
    # Pick t1 / t2 rows.
    # ------------------------------------------------------------------
    t1_rows = df_grasp[df_grasp["t1_flag"].astype(int) == 1]
    t2_rows = df_grasp[df_grasp["t2_flag"].astype(int) == 1]

    if not t1_rows.empty:
        row_t1 = t1_rows.iloc[0]
    else:
        row_t1 = df_grasp.iloc[0]   # fallback: first sample

    if not t2_rows.empty:
        row_t2 = t2_rows.iloc[-1]
    else:
        row_t2 = df_grasp.iloc[-1]  # fallback: last sample

    # ------------------------------------------------------------------
    # Feature blocks.
    # ------------------------------------------------------------------
    features: List[float] = []
    feature_names: List[str] = []

    # Block A: Time-series statistics across whole grasp (12 channels x 3 stats)
    abs_arr = np.abs(sensor_arr)
    peak_abs = abs_arr.max(axis=0)        # (12,)
    mean_abs = abs_arr.mean(axis=0)       # (12,)
    rms      = np.sqrt((sensor_arr ** 2).mean(axis=0))  # (12,)

    for i, col in enumerate(SENSOR_COLS):
        features.append(float(peak_abs[i]))
        feature_names.append(f"peak_{col}")
    for i, col in enumerate(SENSOR_COLS):
        features.append(float(mean_abs[i]))
        feature_names.append(f"meanabs_{col}")
    for i, col in enumerate(SENSOR_COLS):
        features.append(float(rms[i]))
        feature_names.append(f"rms_{col}")

    # Block B: Final state at t2 (12 deltas + 4 magnitudes)
    for col in SENSOR_COLS:
        features.append(float(row_t2[col]))
        feature_names.append(f"t2_{col}")

    for i in range(NUM_SENSORS):
        m = magnitude_xyz(float(row_t2[f"s{i}_dx"]),
                          float(row_t2[f"s{i}_dy"]),
                          float(row_t2[f"s{i}_dz"]))
        features.append(m)
        feature_names.append(f"t2_mag_s{i}")

    # Block C: Final motor / encoder state
    features.append(float(row_t2["enc_deg"]))
    feature_names.append("enc_t2")

    features.append(float(row_t2.get("load_t2", 0.0) or 0.0))
    feature_names.append("load_t2")

    features.append(float(row_t2.get("current_t2", 0.0) or 0.0))
    feature_names.append("current_t2")

    # Block D: Contact timing
    # time_to_stall_ms: the firmware logs this once t1 fires; if missing,
    # fall back to t1_ms or grasp duration.
    tts = float(row_t2.get("time_to_stall_ms", 0.0) or 0.0)
    if tts == 0.0:
        if not t1_rows.empty:
            tts = float(t1_rows.iloc[0]["timestamp_ms"])
        else:
            tts = float(df_grasp.iloc[-1]["timestamp_ms"])
    features.append(tts)
    feature_names.append("time_to_stall_ms")

    enc_t1_val = float(row_t1.get("enc_t1", 0.0) or 0.0)
    if enc_t1_val == 0.0:
        enc_t1_val = float(row_t1["enc_deg"])
    features.append(enc_t1_val)
    feature_names.append("enc_t1")

    # ------------------------------------------------------------------
    # t1_source for diagnostics (NOT a feature).
    # ------------------------------------------------------------------
    if "t1_source" in df_grasp.columns:
        sources = df_grasp.loc[df_grasp["t1_flag"].astype(int) == 1, "t1_source"]
        t1_source_for_grasp = int(sources.iloc[0]) if not sources.empty else 0
    else:
        t1_source_for_grasp = 0

    return np.array(features, dtype=np.float32), feature_names, t1_source_for_grasp


def _read_one_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#")
    df.columns = df.columns.str.strip()

    missing = [col for col in BASE_REQUIRED if col not in df.columns]
    if missing:
        raise ValueError(f"missing required columns: {missing}")

    # Add optional columns if absent (legacy CSVs).
    for col in OPTIONAL_COLUMNS:
        if col not in df.columns:
            df[col] = 0

    keep = BASE_REQUIRED + OPTIONAL_COLUMNS
    df = df[keep].copy()
    df["label"] = df["label"].astype(str).str.strip()
    df["source_file"] = os.path.basename(path)

    numeric_cols = [c for c in keep if c != "label"]
    for col in numeric_cols:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    before = len(df)
    df = df.dropna(subset=numeric_cols)
    dropped = before - len(df)
    if dropped:
        print(f"  WARNING: dropped {dropped} row(s) with non-numeric data in {os.path.basename(path)}")

    return df


def load_data(data_dir: str) -> pd.DataFrame:
    csv_files = sorted(glob.glob(os.path.join(data_dir, "*.csv")))
    if not csv_files:
        print(f"ERROR: No CSV files found in '{data_dir}'")
        sys.exit(1)

    print(f"\nLoading {len(csv_files)} CSV file(s) from '{data_dir}'...")
    frames: List[pd.DataFrame] = []

    for path in csv_files:
        try:
            df = _read_one_csv(path)
            frames.append(df)
            print(f"  {os.path.basename(path):45s} {len(df):5d} rows")
        except Exception as exc:
            print(f"  WARNING: Could not load {path}: {exc}")

    if not frames:
        print("ERROR: No valid CSV data loaded.")
        sys.exit(1)

    combined = pd.concat(frames, ignore_index=True)

    unknown_labels = sorted(set(combined["label"].unique()) - VALID_LABELS)
    if unknown_labels:
        print("\nERROR: Unknown label(s) found in CSV data:")
        for label in unknown_labels:
            print(f"  {label}")
        print(f"Expected labels: {CLASS_NAMES}")
        sys.exit(1)

    combined["file_grasp_id"] = (
        combined["source_file"].astype(str) + "::" + combined["grasp_id"].astype(int).astype(str)
    )

    print(f"\nTotal rows: {len(combined)}")
    print(f"Labels found: {sorted(combined['label'].unique().tolist())}")
    return combined


def build_dataset(df: pd.DataFrame) -> Dataset:
    X_list: List[np.ndarray] = []
    y_list: List[str] = []
    skipped: List[Tuple[str, str, str]] = []
    feature_names: List[str] | None = None
    t1_source_counts = {0: 0, 1: 0, 2: 0, 3: 0}

    for gid, group in df.groupby("file_grasp_id", sort=False):
        label = str(group["label"].iloc[0])
        feat, names_or_error, src = extract_features_from_grasp(group)
        if feat is None:
            skipped.append((str(gid), label, str(names_or_error)))
            continue
        X_list.append(feat)
        y_list.append(label)
        if feature_names is None:
            feature_names = list(names_or_error)  # type: ignore[arg-type]
        t1_source_counts[src] = t1_source_counts.get(src, 0) + 1

    print(f"\nGrasps used:    {len(X_list)}")
    print(f"Grasps skipped: {len(skipped)}")
    for gid, label, reason in skipped[:30]:
        print(f"  {gid} ({label}): {reason}")
    if len(skipped) > 30:
        print(f"  ... {len(skipped) - 30} more skipped grasps")

    print("\nt1_source distribution (debug only, not a feature):")
    print(f"  0 (none / fallback to first sample) : {t1_source_counts.get(0, 0)}")
    print(f"  1 (magnetic)                        : {t1_source_counts.get(1, 0)}")
    print(f"  2 (encoder_stall)                   : {t1_source_counts.get(2, 0)}")
    print(f"  3 (magnetic + encoder_stall)        : {t1_source_counts.get(3, 0)}")

    if len(X_list) < 8:
        print("ERROR: Not enough valid grasps to train. Collect more data.")
        sys.exit(1)

    assert feature_names is not None
    return Dataset(
        X=np.array(X_list, dtype=np.float32),
        y=np.array(y_list),
        feature_names=feature_names,
        skipped=skipped,
        t1_source_counts=t1_source_counts,
    )


def make_classifier(trees: int, depth: int) -> RandomForestClassifier:
    return RandomForestClassifier(
        n_estimators=trees,
        max_depth=depth,
        # min_samples_leaf=2: a leaf must have >= 2 samples. Reduces overfitting
        # on small datasets where depth=1 leaves can memorise individual grasps.
        min_samples_leaf=2,
        # class_weight='balanced': automatically scales loss by inverse class
        # frequency. Helps when classes are unequal (e.g. 40/40/45/30/15).
        class_weight="balanced",
        random_state=42,
        n_jobs=-1,
    )


def write_feature_names(path: str, feature_names: Iterable[str]) -> None:
    names = list(feature_names)
    with open(path, "w", encoding="utf-8") as file:
        file.write(f"# Feature vector - {len(names)} values, indices 0-{len(names) - 1}\n")
        file.write("# Order MUST match extractFeatures() in inference.ino\n\n")
        for i, name in enumerate(names):
            file.write(f"[{i:03d}] {name}\n")


def write_class_names_header(path: str, class_names: Iterable[str]) -> None:
    names = list(class_names)
    with open(path, "w", encoding="utf-8") as file:
        file.write("#pragma once\n")
        file.write("// Auto-generated by train_and_export.py. Do not edit manually.\n")
        file.write(f"const int NUM_CLASSES = {len(names)};\n")
        file.write("const char* const CLASS_NAMES[NUM_CLASSES] = {\n")
        for name in names:
            file.write(f'  "{name}",\n')
        file.write("};\n")


def write_model_config_header(path: str, c_code: str, num_features: int) -> None:
    """
    micromlgen usually exports Eloquent::ML::Port::RandomForest. We inspect the
    generated model.h to wire up either a class-based or global predict() call,
    and we also write the feature count so inference.ino can verify it.
    """
    class_match = re.search(r"class\s+([A-Za-z_][A-Za-z0-9_]*)", c_code)
    has_global_predict = bool(re.search(r"\bpredict\s*\(\s*float\s*\*", c_code)) and not class_match

    with open(path, "w", encoding="utf-8") as file:
        file.write("#pragma once\n")
        file.write("// Auto-generated by train_and_export.py. Do not edit manually.\n")
        file.write(f"#define MODEL_NUM_FEATURES {num_features}\n")
        if class_match:
            class_name = class_match.group(1)
            if "namespace Eloquent" in c_code and "namespace ML" in c_code and "namespace Port" in c_code:
                file.write(f"#define MODEL_CLASS Eloquent::ML::Port::{class_name}\n")
            else:
                file.write(f"#define MODEL_CLASS {class_name}\n")
        elif has_global_predict:
            file.write("#define MODEL_HAS_GLOBAL_PREDICT 1\n")
        else:
            file.write("#define MODEL_CLASS Eloquent::ML::Port::RandomForest\n")


def write_model_info(path: str, args: argparse.Namespace, dataset: Dataset, class_names: List[str], cv_mean: float, cv_std: float, n_folds: int) -> None:
    with open(path, "w", encoding="utf-8") as file:
        file.write("Prosthetic Hand Object Recognition Model Info\n")
        file.write("================================================\n\n")
        file.write(f"Samples used: {dataset.X.shape[0]}\n")
        file.write(f"Features: {dataset.X.shape[1]}\n")
        file.write(f"Skipped grasps: {len(dataset.skipped)}\n")
        file.write(f"Trees: {args.trees}\n")
        file.write(f"Max depth: {args.depth}\n")
        file.write(f"CV folds: {n_folds}\n")
        file.write(f"CV accuracy: {cv_mean * 100:.2f}% +/- {cv_std * 100:.2f}%\n")
        file.write(f"Class order: {class_names}\n")
        file.write("\nt1_source distribution (debug):\n")
        file.write(f"  0 (none / fallback)        : {dataset.t1_source_counts.get(0, 0)}\n")
        file.write(f"  1 (magnetic)               : {dataset.t1_source_counts.get(1, 0)}\n")
        file.write(f"  2 (encoder_stall)          : {dataset.t1_source_counts.get(2, 0)}\n")
        file.write(f"  3 (magnetic+encoder_stall) : {dataset.t1_source_counts.get(3, 0)}\n")


def main() -> None:
    # Resolve defaults relative to THIS script's location, not the user's cwd.
    # Lets `python training\train_and_export.py` work from any directory.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data = os.path.join(script_dir, "grasp_data")
    default_out  = os.path.normpath(os.path.join(script_dir, "..", "inference"))

    parser = argparse.ArgumentParser(description="Train RF classifier for prosthetic hand object recognition")
    parser.add_argument("--data", default=default_data, help="Folder containing CSV files")
    parser.add_argument("--trees", type=int, default=25, help="Number of RF trees")
    parser.add_argument("--depth", type=int, default=8, help="Max tree depth")
    parser.add_argument("--out", default=default_out, help="Output folder for model/header files")
    args = parser.parse_args()

    if args.trees < 1:
        print("ERROR: --trees must be at least 1")
        sys.exit(1)
    if args.depth < 1:
        print("ERROR: --depth must be at least 1")
        sys.exit(1)

    df = load_data(args.data)
    dataset = build_dataset(df)

    print("\nClass distribution:")
    for cls in CLASS_NAMES:
        n = int(np.sum(dataset.y == cls))
        print(f"  {cls:20s}: {n} grasps")

    label_encoder = LabelEncoder()
    label_encoder.fit(CLASS_NAMES)
    class_names = label_encoder.classes_.tolist()
    y_enc = label_encoder.transform(dataset.y)

    print(f"\nClass order exported to inference: {class_names}")
    print(f"Feature vector size: {dataset.X.shape[1]}")

    min_class_count = int(np.min(np.bincount(y_enc, minlength=len(class_names))))
    n_folds = min(5, min_class_count) if min_class_count > 0 else 0
    clf = make_classifier(args.trees, args.depth)

    if n_folds < 2:
        print("\nWARNING: Fewer than 2 samples in at least one class. Skipping cross-validation.")
        cv_mean = float("nan")
        cv_std = float("nan")
        y_cv_pred = np.full_like(y_enc, fill_value=-1)
    else:
        cv = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=42)
        print(f"\nEvaluating Random Forest with {n_folds}-fold stratified cross-validation...")
        scores = cross_val_score(clf, dataset.X, y_enc, cv=cv, scoring="accuracy")
        y_cv_pred = cross_val_predict(clf, dataset.X, y_enc, cv=cv)
        cv_mean = float(scores.mean())
        cv_std = float(scores.std())
        print(f"Cross-val accuracy: {cv_mean * 100:.1f}% +/- {cv_std * 100:.1f}%")
        print("\nClassification report from cross-validated predictions:")
        print(classification_report(y_enc, y_cv_pred, target_names=class_names, zero_division=0))

    print(f"\nTraining final Random Forest on all valid grasps ({args.trees} trees, max_depth={args.depth})...")
    final_clf = make_classifier(args.trees, args.depth)
    final_clf.fit(dataset.X, y_enc)

    print("\nTop 20 feature importances from final model:")
    importances = final_clf.feature_importances_
    indices = np.argsort(importances)[::-1]
    for rank, idx in enumerate(indices[:20], start=1):
        print(f"  {rank:2d}. [{idx:03d}] {dataset.feature_names[idx]:30s} {importances[idx]:.4f}")

    os.makedirs(args.out, exist_ok=True)

    cm_path = os.path.join(args.data, "confusion_matrix_cv.png")
    if n_folds >= 2:
        # Row-normalised confusion matrix in percent: cm[i, j] = P(predicted=j | true=i) * 100.
        # Each row sums to 100. Diagonal cell = recall % for that true class.
        cm_pct = confusion_matrix(
            y_enc, y_cv_pred,
            labels=np.arange(len(class_names)),
            normalize="true",
        ) * 100.0

        fig, ax = plt.subplots(figsize=(7.0, 6.0))
        disp = ConfusionMatrixDisplay(cm_pct, display_labels=class_names)
        disp.plot(
            ax=ax,
            cmap="Blues",
            values_format=".2f",   # e.g. "92.31"
            xticks_rotation=30,
            colorbar=False,         # match the screenshot style: no side bar
        )
        ax.set_xlabel("Predicted")
        ax.set_ylabel("True")
        ax.set_title(
            f"RF {args.trees} trees | {n_folds}-fold CV {cv_mean * 100:.1f}%  "
            f"(values are % per true-class row)"
        )
        plt.tight_layout()
        plt.savefig(cm_path, dpi=150)
        plt.close(fig)
        print(f"\nCross-validated confusion matrix (percent) saved to: {cm_path}")
    else:
        print("\nConfusion matrix not saved because cross-validation was skipped.")

    model_path = os.path.join(args.out, "model.h")
    c_code = port(final_clf)
    with open(model_path, "w", encoding="utf-8") as file:
        file.write(c_code)
    print(f"Model exported to: {model_path}")

    class_header_path = os.path.join(args.out, "class_names.h")
    write_class_names_header(class_header_path, class_names)
    print(f"Class mapping exported to: {class_header_path}")

    model_config_path = os.path.join(args.out, "model_config.h")
    write_model_config_header(model_config_path, c_code, dataset.X.shape[1])
    print(f"Model call configuration exported to: {model_config_path}")

    feature_names_path = os.path.join(args.out, "feature_names.txt")
    write_feature_names(feature_names_path, dataset.feature_names)
    print(f"Feature list saved to: {feature_names_path}")

    info_path = os.path.join(args.out, "model_info.txt")
    write_model_info(info_path, args, dataset, class_names, cv_mean, cv_std, n_folds)
    print(f"Model info saved to: {info_path}")

    print("\n" + "=" * 60)
    print("DONE")
    print(f"  CV accuracy : {cv_mean * 100:.1f}% +/- {cv_std * 100:.1f}%" if n_folds >= 2 else "  CV accuracy : skipped")
    print(f"  Trees       : {args.trees}")
    print(f"  Max depth   : {args.depth}")
    print(f"  Features    : {dataset.X.shape[1]}")
    print(f"  Classes     : {class_names}")
    print(f"  Samples     : {dataset.X.shape[0]}")
    print("Next step:")
    print("  Flash inference/inference.ino. Verify it prints MODEL_NUM_FEATURES at startup")
    print(f"  and the value matches {dataset.X.shape[1]}.")
    print("=" * 60)


if __name__ == "__main__":
    main()

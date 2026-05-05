"""
train_and_export.py - Random Forest training pipeline
=====================================================

Loads grasp CSV data, extracts the 56-feature single-grasp vector,
trains a Random Forest classifier, evaluates it using cross-validation,
and exports C++ files for OpenRB-150 inference.

Usage:
    pip install pandas scikit-learn matplotlib micromlgen

    python train_and_export.py
    python train_and_export.py --data ./grasp_data
    python train_and_export.py --trees 20 --depth 8

Outputs by default to ../3_inference:
    model.h            generated C++ Random Forest model
    model_config.h     tells inference.ino how to call the generated model
    class_names.h      class-name order used by the model
    feature_names.txt  56 feature names in exact order
    model_info.txt     training/evaluation summary

Feature vector: 56 values
-------------------------
Group A - t1 = first meaningful contact:
  0-17    s0_dx,s0_dy,s0_dz, ..., s5_dx,s5_dy,s5_dz
  18-23   |s0|, |s1|, ..., |s5|
  24      encoder position at t1

Group B - t2 = final stable grasp / steady hold point:
  25-42   s0_dx,s0_dy,s0_dz, ..., s5_dx,s5_dy,s5_dz
  43-48   |s0|, |s1|, ..., |s5|
  49      encoder position at t2

Group C - stiffness proxy:
  50-55   |s0|_t2 - |s0|_t1, ..., |s5|_t2 - |s5|_t1
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
    "cylinder_stiff",
    "cylinder_soft",
    "cube_stiff",
    "cube_soft",
}

# LabelEncoder sorts classes. We use that same sorted order everywhere and
# export it to class_names.h so inference.ino cannot drift from training.
CLASS_NAMES = sorted(VALID_LABELS)

SENSOR_COLS = [
    "s0_dx", "s0_dy", "s0_dz",
    "s1_dx", "s1_dy", "s1_dz",
    "s2_dx", "s2_dy", "s2_dz",
    "s3_dx", "s3_dy", "s3_dz",
    "s4_dx", "s4_dy", "s4_dz",
    "s5_dx", "s5_dy", "s5_dz",
]

REQUIRED_COLUMNS = [
    "grasp_id", "label", "timestamp_ms", "enc_deg",
    *SENSOR_COLS,
    "t1_flag", "t2_flag", "t1_ms", "t2_ms",
]


@dataclass
class Dataset:
    X: np.ndarray
    y: np.ndarray
    feature_names: List[str]
    skipped: List[Tuple[str, str, str]]


def magnitude(row: pd.Series, sensor_idx: int) -> float:
    """3D vector magnitude for one sensor from a DataFrame row."""
    dx = float(row[f"s{sensor_idx}_dx"])
    dy = float(row[f"s{sensor_idx}_dy"])
    dz = float(row[f"s{sensor_idx}_dz"])
    return float(np.sqrt(dx * dx + dy * dy + dz * dz))


def extract_features_from_grasp(df_grasp: pd.DataFrame) -> Tuple[np.ndarray | None, List[str] | str]:
    """
    Extract one 56-value feature vector from one grasp.

    The firmware flags exactly one t1 sample and one final t2 sample.
    This function still uses first t1 and last t2 to remain compatible
    with older CSV files where t2_flag may have stayed high during hold.
    """
    t1_rows = df_grasp[df_grasp["t1_flag"].astype(int) == 1]
    t2_rows = df_grasp[df_grasp["t2_flag"].astype(int) == 1]

    if t1_rows.empty:
        return None, "t1 not detected"
    if t2_rows.empty:
        return None, "t2 not detected"

    row_t1 = t1_rows.iloc[0]
    row_t2 = t2_rows.iloc[-1]

    features: List[float] = []
    feature_names: List[str] = []

    # Group A: t1 readings
    for col in SENSOR_COLS:
        features.append(float(row_t1[col]))
        feature_names.append(f"t1_{col}")

    mags_t1: List[float] = []
    for i in range(6):
        m = magnitude(row_t1, i)
        mags_t1.append(m)
        features.append(m)
        feature_names.append(f"t1_mag_s{i}")

    features.append(float(row_t1["enc_deg"]))
    feature_names.append("t1_enc_deg")

    # Group B: t2 readings
    for col in SENSOR_COLS:
        features.append(float(row_t2[col]))
        feature_names.append(f"t2_{col}")

    mags_t2: List[float] = []
    for i in range(6):
        m = magnitude(row_t2, i)
        mags_t2.append(m)
        features.append(m)
        feature_names.append(f"t2_mag_s{i}")

    features.append(float(row_t2["enc_deg"]))
    feature_names.append("t2_enc_deg")

    # Group C: stiffness proxy
    for i in range(6):
        delta = mags_t2[i] - mags_t1[i]
        features.append(delta)
        feature_names.append(f"dmag_s{i}")

    if len(features) != 56:
        return None, f"wrong feature count: {len(features)}"

    return np.array(features, dtype=np.float32), feature_names


def _read_one_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#")
    df.columns = df.columns.str.strip()

    missing = [col for col in REQUIRED_COLUMNS if col not in df.columns]
    if missing:
        raise ValueError(f"missing columns: {missing}")

    df = df[REQUIRED_COLUMNS].copy()
    df["label"] = df["label"].astype(str).str.strip()
    df["source_file"] = os.path.basename(path)

    numeric_cols = [col for col in REQUIRED_COLUMNS if col != "label"]
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

    # Make a unique grasp key across multiple CSV files where grasp_id may restart.
    combined["file_grasp_id"] = combined["source_file"].astype(str) + "::" + combined["grasp_id"].astype(int).astype(str)

    print(f"\nTotal rows: {len(combined)}")
    print(f"Labels found: {sorted(combined['label'].unique().tolist())}")
    return combined


def build_dataset(df: pd.DataFrame) -> Dataset:
    X_list: List[np.ndarray] = []
    y_list: List[str] = []
    skipped: List[Tuple[str, str, str]] = []
    feature_names: List[str] | None = None

    for gid, group in df.groupby("file_grasp_id", sort=False):
        label = str(group["label"].iloc[0])
        feat, names_or_error = extract_features_from_grasp(group)
        if feat is None:
            skipped.append((str(gid), label, str(names_or_error)))
            continue
        X_list.append(feat)
        y_list.append(label)
        if feature_names is None:
            feature_names = list(names_or_error)  # type: ignore[arg-type]

    print(f"\nGrasps used:    {len(X_list)}")
    print(f"Grasps skipped: {len(skipped)}")
    for gid, label, reason in skipped[:30]:
        print(f"  {gid} ({label}): {reason}")
    if len(skipped) > 30:
        print(f"  ... {len(skipped) - 30} more skipped grasps")

    if len(X_list) < 8:
        print("ERROR: Not enough valid grasps to train. Collect more data.")
        sys.exit(1)

    assert feature_names is not None
    return Dataset(
        X=np.array(X_list, dtype=np.float32),
        y=np.array(y_list),
        feature_names=feature_names,
        skipped=skipped,
    )


def make_classifier(trees: int, depth: int) -> RandomForestClassifier:
    return RandomForestClassifier(
        n_estimators=trees,
        max_depth=depth,
        random_state=42,
        n_jobs=-1,
    )


def write_feature_names(path: str, feature_names: Iterable[str]) -> None:
    with open(path, "w", encoding="utf-8") as file:
        file.write("# Feature vector - 56 values, indices 0-55\n")
        file.write("# Order MUST match extractFeatures() in inference.ino\n\n")
        for i, name in enumerate(feature_names):
            file.write(f"[{i:02d}] {name}\n")


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


def write_model_config_header(path: str, c_code: str) -> None:
    """
    micromlgen usually exports Eloquent::ML::Port::RandomForest, but this
    file is generated by inspecting model.h so inference.ino does not need
    to hard-code the class name.
    """
    class_match = re.search(r"class\s+([A-Za-z_][A-Za-z0-9_]*)", c_code)
    has_global_predict = bool(re.search(r"\bpredict\s*\(\s*float\s*\*", c_code)) and not class_match

    with open(path, "w", encoding="utf-8") as file:
        file.write("#pragma once\n")
        file.write("// Auto-generated by train_and_export.py. Do not edit manually.\n")
        if class_match:
            class_name = class_match.group(1)
            if "namespace Eloquent" in c_code and "namespace ML" in c_code and "namespace Port" in c_code:
                file.write(f"#define MODEL_CLASS Eloquent::ML::Port::{class_name}\n")
            else:
                file.write(f"#define MODEL_CLASS {class_name}\n")
        elif has_global_predict:
            file.write("#define MODEL_HAS_GLOBAL_PREDICT 1\n")
        else:
            # Default for standard micromlgen RandomForest output.
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Train RF classifier for prosthetic hand object recognition")
    parser.add_argument("--data", default="grasp_data", help="Folder containing CSV files")
    parser.add_argument("--trees", type=int, default=20, help="Number of RF trees")
    parser.add_argument("--depth", type=int, default=8, help="Max tree depth")
    parser.add_argument("--out", default="../3_inference", help="Output folder for model/header files")
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
        print(f"  {cls:20s}: {int(np.sum(dataset.y == cls))} grasps")

    label_encoder = LabelEncoder()
    label_encoder.fit(CLASS_NAMES)
    class_names = label_encoder.classes_.tolist()
    y_enc = label_encoder.transform(dataset.y)

    print(f"\nClass order exported to inference: {class_names}")

    min_class_count = int(np.min(np.bincount(y_enc)))
    n_folds = min(5, min_class_count)
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

    # Final model is trained on all valid grasps for deployment.
    print(f"\nTraining final Random Forest on all valid grasps ({args.trees} trees, max_depth={args.depth})...")
    final_clf = make_classifier(args.trees, args.depth)
    final_clf.fit(dataset.X, y_enc)

    print("\nTop 15 feature importances from final model:")
    importances = final_clf.feature_importances_
    indices = np.argsort(importances)[::-1]
    for rank, idx in enumerate(indices[:15], start=1):
        print(f"  {rank:2d}. [{idx:02d}] {dataset.feature_names[idx]:25s} {importances[idx]:.4f}")

    os.makedirs(args.out, exist_ok=True)

    # Honest confusion matrix from CV predictions where possible.
    cm_path = os.path.join(args.data, "confusion_matrix_cv.png")
    if n_folds >= 2:
        cm = confusion_matrix(y_enc, y_cv_pred, labels=np.arange(len(class_names)))
        fig, ax = plt.subplots(figsize=(6.5, 5.5))
        ConfusionMatrixDisplay(cm, display_labels=class_names).plot(ax=ax, xticks_rotation=30)
        ax.set_title(f"RF {args.trees} trees | {n_folds}-fold CV {cv_mean * 100:.1f}%")
        plt.tight_layout()
        plt.savefig(cm_path, dpi=150)
        plt.close(fig)
        print(f"\nCross-validated confusion matrix saved to: {cm_path}")
    else:
        print("\nConfusion matrix not saved because cross-validation was skipped.")

    # Export C++ model. Do not use classmap here: keeping integer class
    # labels makes inference.ino simple and class_names.h provides mapping.
    model_path = os.path.join(args.out, "model.h")
    c_code = port(final_clf)
    with open(model_path, "w", encoding="utf-8") as file:
        file.write(c_code)
    print(f"Model exported to: {model_path}")

    class_header_path = os.path.join(args.out, "class_names.h")
    write_class_names_header(class_header_path, class_names)
    print(f"Class mapping exported to: {class_header_path}")

    model_config_path = os.path.join(args.out, "model_config.h")
    write_model_config_header(model_config_path, c_code)
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
    print(f"  Samples     : {dataset.X.shape[0]}")
    print("Next step:")
    print("  Flash 3_inference/inference.ino after model.h, model_config.h and class_names.h are generated.")
    print("=" * 60)


if __name__ == "__main__":
    main()

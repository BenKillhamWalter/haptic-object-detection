"""
train_and_export.py  —  Random Forest training pipeline
========================================================
Loads grasp CSV data, extracts features at t1 and t2, trains a
Random Forest classifier, and exports a C++ model header (model.h)
for deployment on the OpenRB-150.

Usage:
    pip install pandas scikit-learn matplotlib micromlgen

    python train_and_export.py                         # uses ./grasp_data/*.csv
    python train_and_export.py --data ./grasp_data     # explicit data folder
    python train_and_export.py --trees 20 --depth 8    # tune RF size

Outputs:
    model.h           → copy into 3_inference/ folder, then flash inference.ino
    feature_names.txt → documents which features the model expects (in order)
    confusion_matrix.png

═══════════════════════════════════════════════════════════════════
FEATURE VECTOR (56 features, same order in training AND inference)
═══════════════════════════════════════════════════════════════════
Group A — readings at t1 (first contact):
   0–17   s0_dx,s0_dy,s0_dz, ..., s5_dx,s5_dy,s5_dz   (18 values)
  18–23   |s0|, |s1|, ..., |s5|  per-sensor magnitude   ( 6 values)
  24      encoder position at t1                         ( 1 value )

Group B — readings at t2 (steady state / grip):
  25–42   s0_dx,s0_dy,s0_dz, ..., s5_dx,s5_dy,s5_dz   (18 values)
  43–48   |s0|, |s1|, ..., |s5|  per-sensor magnitude   ( 6 values)
  49      encoder position at t2                         ( 1 value )

Group C — stiffness proxy (change between t1 and t2):
  50–55   |s0|_t2 - |s0|_t1, ..., |s5|_t2 - |s5|_t1  ( 6 values)
          Soft objects deform more → larger delta magnitude

Total: 56 features
"""

import os
import sys
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.metrics import classification_report, confusion_matrix, ConfusionMatrixDisplay
from sklearn.preprocessing import LabelEncoder

try:
    from micromlgen import port
except ImportError:
    print("ERROR: micromlgen not installed.  Run:  pip install micromlgen")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────
#  Configuration
# ─────────────────────────────────────────────────────────────────

SENSOR_COLS = [
    "s0_dx","s0_dy","s0_dz",
    "s1_dx","s1_dy","s1_dz",
    "s2_dx","s2_dy","s2_dz",
    "s3_dx","s3_dy","s3_dz",
    "s4_dx","s4_dy","s4_dz",
    "s5_dx","s5_dy","s5_dz",
]

CLASS_NAMES = ["cylinder_soft", "cylinder_stiff", "cube_soft", "cube_stiff"]

# ─────────────────────────────────────────────────────────────────
#  Feature extraction
# ─────────────────────────────────────────────────────────────────

def magnitude(row, sensor_idx):
    """3D vector magnitude for one sensor from a DataFrame row."""
    dx = row[f"s{sensor_idx}_dx"]
    dy = row[f"s{sensor_idx}_dy"]
    dz = row[f"s{sensor_idx}_dz"]
    return float(np.sqrt(dx**2 + dy**2 + dz**2))


def extract_features_from_grasp(df_grasp):
    """
    Given a DataFrame of one grasp (all rows for one grasp_id),
    extract the 56-element feature vector.
    Returns None if t1 or t2 were not detected in this grasp.
    """
    # Find t1 and t2 rows (use first row where flag first becomes 1)
    t1_rows = df_grasp[df_grasp["t1_flag"] == 1]
    t2_rows = df_grasp[df_grasp["t2_flag"] == 1]

    if t1_rows.empty:
        return None, "t1 not detected"
    if t2_rows.empty:
        return None, "t2 not detected"

    row_t1 = t1_rows.iloc[0]   # First row where t1 was active
    row_t2 = t2_rows.iloc[-1]  # Last row (most stable point in hold)

    features = []
    feature_names = []

    # ── Group A: readings at t1 ──────────────────────────────────
    for col in SENSOR_COLS:
        features.append(float(row_t1[col]))
        feature_names.append(f"t1_{col}")

    mags_t1 = []
    for i in range(6):
        m = magnitude(row_t1, i)
        mags_t1.append(m)
        features.append(m)
        feature_names.append(f"t1_mag_s{i}")

    features.append(float(row_t1["enc_deg"]))
    feature_names.append("t1_enc_deg")

    # ── Group B: readings at t2 ──────────────────────────────────
    for col in SENSOR_COLS:
        features.append(float(row_t2[col]))
        feature_names.append(f"t2_{col}")

    mags_t2 = []
    for i in range(6):
        m = magnitude(row_t2, i)
        mags_t2.append(m)
        features.append(m)
        feature_names.append(f"t2_mag_s{i}")

    features.append(float(row_t2["enc_deg"]))
    feature_names.append("t2_enc_deg")

    # ── Group C: stiffness proxy (delta magnitude t2 - t1) ───────
    for i in range(6):
        delta = mags_t2[i] - mags_t1[i]
        features.append(delta)
        feature_names.append(f"dmag_s{i}")

    return np.array(features, dtype=np.float32), feature_names


# ─────────────────────────────────────────────────────────────────
#  Data loading
# ─────────────────────────────────────────────────────────────────

def load_data(data_dir):
    csv_files = glob.glob(os.path.join(data_dir, "*.csv"))
    if not csv_files:
        print(f"ERROR: No CSV files found in '{data_dir}'")
        sys.exit(1)

    print(f"\nLoading {len(csv_files)} CSV file(s) from '{data_dir}' ...")
    all_dfs = []
    for f in sorted(csv_files):
        try:
            df = pd.read_csv(f, comment="#")
            # Strip whitespace from column names and string columns
            df.columns = df.columns.str.strip()
            df["label"] = df["label"].str.strip()
            all_dfs.append(df)
            print(f"  {os.path.basename(f):45s}  {len(df):5d} rows")
        except Exception as e:
            print(f"  WARNING: Could not load {f}: {e}")

    if not all_dfs:
        print("ERROR: No valid data loaded.")
        sys.exit(1)

    combined = pd.concat(all_dfs, ignore_index=True)

    # Re-assign unique grasp_id across files to avoid collisions
    combined["file_grasp_id"] = (
        combined.groupby(combined["grasp_id"].ne(combined["grasp_id"].shift()).cumsum()).ngroup()
    )

    print(f"\nTotal rows: {len(combined)}")
    print(f"Labels found: {combined['label'].unique().tolist()}")
    return combined


# ─────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Train RF classifier for prosthetic hand")
    ap.add_argument("--data",   default="grasp_data",  help="Folder containing grasp CSV files")
    ap.add_argument("--trees",  type=int, default=20,  help="Number of RF trees (default 20, max ~30 for OpenRB-150)")
    ap.add_argument("--depth",  type=int, default=8,   help="Max tree depth (default 8)")
    ap.add_argument("--out",    default="../3_inference", help="Output folder for model.h")
    args = ap.parse_args()

    # ── Load data ────────────────────────────────────────────────
    df = load_data(args.data)

    # ── Extract per-grasp feature vectors ────────────────────────
    X_list, y_list, skipped = [], [], []
    feature_names = None

    grasp_groups = df.groupby("file_grasp_id")
    for gid, g in grasp_groups:
        label = g["label"].iloc[0]
        feat, names_or_err = extract_features_from_grasp(g)
        if feat is None:
            skipped.append((gid, label, names_or_err))
            continue
        X_list.append(feat)
        y_list.append(label)
        if feature_names is None:
            feature_names = names_or_err

    print(f"\nGrasps used:    {len(X_list)}")
    print(f"Grasps skipped: {len(skipped)}")
    for s in skipped:
        print(f"  grasp {s[0]} ({s[1]}): {s[2]}")

    if len(X_list) < 8:
        print("ERROR: Not enough valid grasps to train. Collect more data.")
        sys.exit(1)

    X = np.array(X_list)
    y = np.array(y_list)

    # ── Label distribution ────────────────────────────────────────
    print("\nClass distribution:")
    for cls in np.unique(y):
        print(f"  {cls:20s}: {np.sum(y == cls)} grasps")

    # ── Encode labels ─────────────────────────────────────────────
    le = LabelEncoder()
    le.fit(CLASS_NAMES)  # Fixed order — must match inference.ino CLASS_NAMES
    try:
        y_enc = le.transform(y)
    except ValueError as e:
        print(f"\nERROR: Unknown label in data: {e}")
        print(f"Expected one of: {CLASS_NAMES}")
        sys.exit(1)

    # ── Train Random Forest ───────────────────────────────────────
    print(f"\nTraining Random Forest ({args.trees} trees, max_depth={args.depth}) ...")
    clf = RandomForestClassifier(
        n_estimators=args.trees,
        max_depth=args.depth,
        random_state=42,
        n_jobs=-1
    )

    # Cross-validation (stratified, k=5 or fewer if limited data)
    n_folds = min(5, min(np.bincount(y_enc)))
    if n_folds < 2:
        print("WARNING: Very few samples per class — skipping cross-validation.")
        clf.fit(X, y_enc)
        cv_mean, cv_std = float("nan"), float("nan")
    else:
        cv = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=42)
        scores = cross_val_score(clf, X, y_enc, cv=cv, scoring="accuracy")
        cv_mean, cv_std = scores.mean(), scores.std()
        print(f"Cross-val accuracy: {cv_mean*100:.1f}% ± {cv_std*100:.1f}%")
        clf.fit(X, y_enc)

    # Full-data predictions for confusion matrix
    y_pred = clf.predict(X)

    print("\nClassification report (full training set):")
    print(classification_report(y_enc, y_pred, target_names=le.classes_))

    # ── Feature importance ────────────────────────────────────────
    importances = clf.feature_importances_
    indices = np.argsort(importances)[::-1]
    print("Top 15 most important features:")
    for rank, idx in enumerate(indices[:15]):
        print(f"  {rank+1:2d}. [{idx:2d}] {feature_names[idx]:25s}  {importances[idx]:.4f}")

    # ── Confusion matrix plot ─────────────────────────────────────
    cm = confusion_matrix(y_enc, y_pred)
    fig, ax = plt.subplots(figsize=(6, 5))
    ConfusionMatrixDisplay(cm, display_labels=le.classes_).plot(ax=ax)
    ax.set_title(f"RF {args.trees} trees  |  CV {cv_mean*100:.1f}%")
    plt.tight_layout()
    cm_path = os.path.join(args.data, "confusion_matrix.png")
    plt.savefig(cm_path, dpi=120)
    print(f"\nConfusion matrix saved to: {cm_path}")

    # ── Export model.h via micromlgen ─────────────────────────────
    os.makedirs(args.out, exist_ok=True)
    model_path = os.path.join(args.out, "model.h")

    # micromlgen needs integer class labels; pass class names via classmap
    classmap = {i: name for i, name in enumerate(le.classes_)}
    c_code = port(clf, classmap=classmap)

    with open(model_path, "w") as f:
        f.write(c_code)
    print(f"\nModel exported to: {model_path}")

    # ── Save feature names (must match inference.ino) ─────────────
    names_path = os.path.join(args.out, "feature_names.txt")
    with open(names_path, "w") as f:
        f.write("# Feature vector — 56 values, indices 0-55\n")
        f.write("# Order MUST match extractFeatures() in inference.ino\n\n")
        for i, name in enumerate(feature_names):
            f.write(f"  [{i:2d}]  {name}\n")
    print(f"Feature list saved to: {names_path}")

    # ── Summary ───────────────────────────────────────────────────
    print("\n" + "═"*55)
    print("DONE")
    print(f"  CV accuracy : {cv_mean*100:.1f}% ± {cv_std*100:.1f}%")
    print(f"  Trees       : {args.trees}")
    print(f"  Max depth   : {args.depth}")
    print(f"  Features    : {X.shape[1]}")
    print(f"  Samples     : {X.shape[0]}")
    print("─"*55)
    print("Next step:")
    print("  1. Copy 3_inference/model.h into the inference sketch folder")
    print("  2. Flash 3_inference/inference.ino to the OpenRB-150")
    print("═"*55)


if __name__ == "__main__":
    main()

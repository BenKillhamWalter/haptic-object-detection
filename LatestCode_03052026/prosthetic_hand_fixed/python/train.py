"""train.py — load all session CSVs, train a Random Forest, save model.pkl.

Usage:
    python train.py [--trees 25] [--depth 8] [--out data/model.pkl]

Reads every data/grasps_session*.csv, drops failed rows, optionally
recomputes features from raw streams (not needed since collector
writes features), then runs a GroupKFold CV by session_id and trains
the final model on all data.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import glob
from typing import List

import joblib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import GroupKFold, cross_val_predict
from sklearn.metrics import classification_report, confusion_matrix

from hand import constants as C
from hand.features import feature_names


def _data_dir() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'data')


def _load_all_sessions() -> pd.DataFrame:
    files = sorted(glob.glob(os.path.join(_data_dir(), 'grasps_session*.csv')))
    if not files:
        raise FileNotFoundError(f'no grasps_session*.csv in {_data_dir()}')
    frames = []
    for f in files:
        try:
            df = pd.read_csv(f)
            frames.append(df)
            print(f'[train] loaded {os.path.basename(f)} ({len(df)} rows)')
        except pd.errors.EmptyDataError:
            print(f'[train]   skipping empty {f}')
    return pd.concat(frames, ignore_index=True)


def _plot_confusion(cm: np.ndarray, classes: List[str], out_png: str):
    norm = cm.astype(np.float64)
    row_sums = norm.sum(axis=1, keepdims=True)
    row_sums[row_sums == 0] = 1
    norm = 100.0 * norm / row_sums

    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.imshow(norm, cmap='Blues', vmin=0, vmax=100)
    ax.set_xticks(range(len(classes)))
    ax.set_yticks(range(len(classes)))
    ax.set_xticklabels(classes, rotation=45, ha='right')
    ax.set_yticklabels(classes)
    ax.set_xlabel('predicted')
    ax.set_ylabel('true')
    ax.set_title('CV confusion matrix (row-normalised %)')
    for i in range(len(classes)):
        for j in range(len(classes)):
            ax.text(j, i, f'{norm[i, j]:.2f}', ha='center', va='center',
                    color='white' if norm[i, j] > 50 else 'black', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--trees', type=int, default=300)
    ap.add_argument('--depth', type=int, default=8)
    ap.add_argument('--out',   default=os.path.join(_data_dir(), 'model.pkl'))
    args = ap.parse_args()

    df = _load_all_sessions()
    df = df[df['success'].astype(int) == 1].reset_index(drop=True)
    if df.empty:
        print('[train] FAIL: no successful grasps to train on')
        return 1

    feat_cols = feature_names()
    missing = [c for c in feat_cols if c not in df.columns]
    if missing:
        print(f'[train] FAIL: CSV missing feature columns: {missing[:5]} ...')
        return 1
    X = df[feat_cols].astype(np.float32).values
    y = df['label'].astype(str).values
    groups = df['session_id'].astype(int).values

    classes = list(C.CLASS_NAMES)
    print(f'[train] training on {len(X)} grasps across {len(set(groups))} sessions')
    print(f'[train] class balance:')
    for c in classes:
        print(f'           {c:18s} {int((y == c).sum())}')

    clf_kwargs = dict(
        n_estimators=args.trees,
        max_depth=args.depth,
        min_samples_leaf=2,
        class_weight='balanced',
        random_state=42,
        n_jobs=-1,
    )

    n_groups = len(set(groups))
    if n_groups >= 2:
        n_splits = min(5, n_groups)
        cv = GroupKFold(n_splits=n_splits)
        print(f'[train] running GroupKFold CV ({n_splits} splits, session-aware)')
        preds = cross_val_predict(
            RandomForestClassifier(**clf_kwargs), X, y, groups=groups, cv=cv, n_jobs=-1)
        acc = float((preds == y).mean())
        print(f'[train] CV accuracy: {acc * 100:.2f}%')
        print('[train] classification report:')
        print(classification_report(y, preds, labels=classes, zero_division=0))
        cm = confusion_matrix(y, preds, labels=classes)
        png = os.path.join(_data_dir(), 'confusion_matrix_cv.png')
        _plot_confusion(cm, classes, png)
        print(f'[train] saved {png}')
    else:
        print('[train] WARNING: only one session present; skipping CV')

    # Final model trained on all data.
    final = RandomForestClassifier(**clf_kwargs)
    final.fit(X, y)
    joblib.dump({'model': final, 'classes': classes, 'feature_names': feat_cols}, args.out)
    print(f'[train] saved -> {args.out}')

    classes_json = os.path.join(_data_dir(), 'class_names.json')
    with open(classes_json, 'w') as fh:
        json.dump(classes, fh, indent=2)
    print(f'[train] saved -> {classes_json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

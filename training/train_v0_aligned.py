#!/usr/bin/env python3
"""T1b aligned v0：按 session 留出评估，并保存全量 RF 试模型。"""

import json
import pickle
from pathlib import Path

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, f1_score, confusion_matrix

from features_t1b_aligned import build, CLASS_NAMES, FEAT_NAMES


OUT_DIR = Path(__file__).resolve().parent / "models"


def model():
    return RandomForestClassifier(
        n_estimators=120, max_depth=8, class_weight="balanced",
        random_state=42, n_jobs=-1,
    )


def score(y_true, pred):
    return {
        "accuracy": float(accuracy_score(y_true, pred)),
        "macro_f1": float(f1_score(y_true, pred, average="macro", zero_division=0)),
    }


def main():
    X, y, groups, meta = build()
    unique = list(dict.fromkeys(groups.tolist()))
    fold_results = []
    all_true, all_pred = [], []
    for held in unique:
        te = groups == held
        tr = ~te
        clf = model()
        clf.fit(X[tr], y[tr])
        pred = clf.predict(X[te])
        s = score(y[te], pred)
        s.update({"held_out_session": held, "n_test": int(te.sum())})
        fold_results.append(s)
        all_true.extend(y[te].tolist()); all_pred.extend(pred.tolist())

    overall = score(np.asarray(all_true), np.asarray(all_pred))
    cm = confusion_matrix(all_true, all_pred, labels=range(5)).tolist()

    # 规则训练 -> 自由测试，作为当前 Teacher 标签与协议标签之间的压力测试。
    free_ids = {"20260813_200450", "20260822_190703", "20260822_192412"}
    rule_mask = np.array([g not in free_ids for g in groups])
    free_mask = ~rule_mask
    cross = None
    if rule_mask.any() and free_mask.any():
        clf = model(); clf.fit(X[rule_mask], y[rule_mask])
        cross = score(y[free_mask], clf.predict(X[free_mask]))

    final = model(); final.fit(X, y)
    model_path = OUT_DIR / "tinyml_v0_aligned_rf.pkl"
    with open(model_path, "wb") as f:
        pickle.dump(final, f)

    report = {
        "dataset": meta,
        "evaluation": {
            "session_loso": fold_results,
            "session_loso_overall": overall,
            "session_loso_confusion_matrix": cm,
            "rule_to_free": cross,
        },
        "model": {
            "type": "RandomForestClassifier",
            "n_estimators": 120, "max_depth": 8,
            "feature_names": FEAT_NAMES,
            "model_file": model_path.name,
        },
    }
    report_path = OUT_DIR / "tinyml_v0_aligned_report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print("=" * 64)
    print("T1b aligned v0")
    print(f"samples={len(y)}, features={X.shape[1]}, sessions={len(unique)}")
    print(f"LOSO overall: acc={overall['accuracy']:.3f}, macro-F1={overall['macro_f1']:.3f}")
    if cross:
        print(f"rule -> free: acc={cross['accuracy']:.3f}, macro-F1={cross['macro_f1']:.3f}")
    print("per-session:")
    for r in fold_results:
        print(f"  {r['held_out_session']}: acc={r['accuracy']:.3f}, "
              f"F1={r['macro_f1']:.3f}, n={r['n_test']}")
    print(f"saved: {model_path}")
    print(f"report: {report_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""HappyMac v0 模型阶梯：比较可部署的小树/小森林。"""

import json
import pickle
from pathlib import Path

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, f1_score
from sklearn.tree import DecisionTreeClassifier


ROOT = Path(__file__).resolve().parent
MODEL_DIR = ROOT / "models"
X = np.load(MODEL_DIR / "t1b_aligned_X.npy")
y = np.load(MODEL_DIR / "t1b_aligned_y.npy")
groups = np.load(MODEL_DIR / "t1b_aligned_groups.npy")
FREE_IDS = {"20260813_200450", "20260822_190703", "20260822_192412"}


def candidates():
    for depth in (3, 4, 5):
        yield f"tree_d{depth}", lambda depth=depth: DecisionTreeClassifier(
            max_depth=depth, class_weight="balanced", random_state=42)
    for trees in (10, 20, 120):
        yield f"rf_{trees}t", lambda trees=trees: RandomForestClassifier(
            n_estimators=trees, max_depth=8, class_weight="balanced",
            random_state=42, n_jobs=-1)


def metrics(y_true, pred):
    return {
        "accuracy": float(accuracy_score(y_true, pred)),
        "macro_f1": float(f1_score(y_true, pred, average="macro", zero_division=0)),
    }


def evaluate(make_model):
    held_results = []
    y_all, p_all = [], []
    for held in dict.fromkeys(groups.tolist()):
        te = groups == held
        tr = ~te
        m = make_model(); m.fit(X[tr], y[tr])
        p = m.predict(X[te])
        s = metrics(y[te], p)
        s.update({"held_out": held, "n_test": int(te.sum())})
        held_results.append(s)
        y_all.extend(y[te]); p_all.extend(p)
    overall = metrics(np.asarray(y_all), np.asarray(p_all))

    # 只用规则训练、自由测试，观察跨标签来源的迁移能力。
    rule = np.array([g not in FREE_IDS for g in groups])
    free = ~rule
    m = make_model(); m.fit(X[rule], y[rule])
    transfer = metrics(y[free], m.predict(X[free]))
    return overall, transfer, held_results


def main():
    results = {}
    for name, make_model in candidates():
        overall, transfer, held = evaluate(make_model)
        final = make_model(); final.fit(X, y)
        path = MODEL_DIR / f"tinyml_v0_{name}.pkl"
        with open(path, "wb") as f:
            pickle.dump(final, f)
        size = path.stat().st_size
        n_nodes = int(sum(est.tree_.node_count for est in final.estimators_)) \
            if hasattr(final, "estimators_") else int(final.tree_.node_count)
        results[name] = {
            "loso_overall": overall,
            "rule_to_free": transfer,
            "per_session": held,
            "pickle_bytes": size,
            "tree_nodes": n_nodes,
            "model_file": path.name,
        }
        print(f"{name:10s} LOSO acc={overall['accuracy']:.3f} "
              f"F1={overall['macro_f1']:.3f} | rule->free "
              f"acc={transfer['accuracy']:.3f} F1={transfer['macro_f1']:.3f} | "
              f"nodes={n_nodes} pickle={size/1024:.1f}KB")

    report = {
        "source": "t1b_aligned_X/y/groups.npy",
        "n_samples": int(len(y)),
        "n_features": int(X.shape[1]),
        "results": results,
    }
    path = MODEL_DIR / "tinyml_v0_model_ladder_report.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"report: {path}")


if __name__ == "__main__":
    main()

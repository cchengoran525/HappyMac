#!/usr/bin/env python3
"""决策树导出工具：pkl → C 头文件 + JS 预测函数。

默认训练一个 "LATERAL 保守" 的 depth-4 树（min_samples_leaf 正则化），
目标是压低 typing/head_only 微动被误判成 LATERAL（静坐误笑的主因，
见 simulator/eval_replay.js 评测），同时保持整体 LOSO 指标不明显回退。

用法：
  python3 export_tree.py            # 训练 + LOSO 对比 + 导出到 models/exported/
"""
import pickle
from pathlib import Path

import numpy as np
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import LeaveOneGroupOut

ROOT = Path(__file__).resolve().parent
MODEL_DIR = ROOT / "models"
OUT_DIR = MODEL_DIR / "exported"
OUT_DIR.mkdir(exist_ok=True)
FREE_IDS = {"20260813_200450", "20260822_190703", "20260822_192412"}
CLASS_NAMES = ["ABSENT", "STILL", "LATERAL", "APPROACH", "RETREAT"]

# 特征顺序（与固件 features 数组一致）
JS_FEATURE = ["f[0]", "f[1]", "f[2]", "f[3]", "f[4]", "f[5]", "f[6]", "f[7]", "f[8]"]
C_FEATURE = ["f[0]", "f[1]", "f[2]", "f[3]", "f[4]", "f[5]", "f[6]", "f[7]", "f[8]"]


def make_calm_tree():
    return DecisionTreeClassifier(max_depth=4, min_samples_leaf=40,
                                  class_weight="balanced", random_state=42)


def emit_c(tree, feature_names):
    t = tree.tree_
    lines = []

    def walk(node, indent):
        pad = "  " * indent
        if t.children_left[node] == -1:  # leaf
            label = int(np.argmax(t.value[node]))
            lines.append(f"{pad}return {label};")
            return
        feat = JS_FEATURE and C_FEATURE[t.feature[node]]
        thr = f"{t.threshold[node]:.8f}"
        lines.append(f"{pad}if ({feat} <= {thr}) {{")
        walk(t.children_left[node], indent + 1)
        lines.append(f"{pad}}}")
        walk(t.children_right[node], indent)

    lines.append("static inline uint8_t tinymlV0Predict(const float f[9]) {")
    walk(0, 1)
    lines.append("}")
    return "\n".join(lines)


def emit_js(tree):
    t = tree.tree_
    lines = []

    def walk(node, indent):
        pad = "  " * indent
        if t.children_left[node] == -1:
            lines.append(f"{pad}return {int(np.argmax(t.value[node]))};")
            return
        feat = C_FEATURE[t.feature[node]]
        thr = f"{t.threshold[node]:.8f}"
        lines.append(f"{pad}if ({feat} <= {thr}) {{")
        walk(t.children_left[node], indent + 1)
        lines.append(f"{pad}}}")
        walk(t.children_right[node], indent)

    lines.append("function tinymlV0Predict(f) {")
    walk(0, 1)
    lines.append("}")
    return "\n".join(lines)


def still_precision_stats(y_true, y_pred):
    """静坐相关指标：STILL 类被误判成 LATERAL 的比例（误笑驱动）。"""
    still = y_true == 1
    total = int(still.sum())
    bad = int((y_pred[still] == 2).sum())
    return bad, total


def main():
    X = np.load(MODEL_DIR / "t1b_aligned_X.npy")
    y = np.load(MODEL_DIR / "t1b_aligned_y.npy")
    groups = np.load(MODEL_DIR / "t1b_aligned_groups.npy")

    old = pickle.load(open(MODEL_DIR / "tinyml_v0_tree_d4.pkl", "rb"))
    new = make_calm_tree()

    # ── LOSO 对比（关注 STILL→LATERAL 误判，即误笑源头）──
    logo = LeaveOneGroupOut()
    res = {"old": np.empty_like(y), "new": np.empty_like(y)}
    for tr, te in logo.split(X, y, groups):
        o = DecisionTreeClassifier(max_depth=4, class_weight="balanced", random_state=42)
        o.fit(X[tr], y[tr]); res["old"][te] = o.predict(X[te])
        n = make_calm_tree(); n.fit(X[tr], y[tr]); res["new"][te] = n.predict(X[te])

    for name in ("old", "new"):
        yp = res[name]
        acc = (yp == y).mean() * 100
        bad, total = still_precision_stats(y, yp)
        print(f"{name}: LOSO acc={acc:.1f}%  STILL→LATERAL 误判 {bad}/{total} = {bad/total*100:.1f}%")

    # ── 全量训练保守树并导出 ──
    new.fit(X, y)
    header = f"""#pragma once

#include <stdint.h>

// T1b aligned v0 (calm), sklearn DecisionTreeClassifier(max_depth=4, min_samples_leaf=40).
// 目标：压低 typing/head_only 微动被误判为 LATERAL（静坐误笑主因）。
// Feature order:
// |slope_X|, slope_Y, std_X, std_Y, mean_Es, std_Es,
// mean_abs_V, max_abs_V, Es_edge
// Labels: 0 ABSENT, 1 STILL, 2 LATERAL, 3 APPROACH, 4 RETREAT.
// 由 training/export_tree.py 生成，勿手改。
{emit_c(new, CLASS_NAMES)}

static inline const char* tinymlV0LabelName(uint8_t label) {{
  switch (label) {{
    case 0: return "ABSENT";
    case 1: return "STILL";
    case 2: return "LATERAL";
    case 3: return "APPROACH";
    case 4: return "RETREAT";
  }}
  return "UNKNOWN";
}}
"""
    (OUT_DIR / "tinyml_v0_tree_d4_calm.h").write_text(header)
    (OUT_DIR / "tinyml_v0_tree_d4_calm.js").write_text(emit_js(new) + "\n")
    with open(OUT_DIR / "tinyml_v0_tree_d4_calm.pkl", "wb") as f:
        pickle.dump(new, f)
    print(f"\n导出: {OUT_DIR}/tinyml_v0_tree_d4_calm.h / .js / .pkl")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""可分性实验：转头（head_only）能否用现有 9 特征从 typing/still 中分出来？

背景：现版 T1b 训练把 head_only 标成 STILL（PROTOCOL_MAP head_only:1），
模型从未被要求学转头。本实验把 head_only 单独成类，用与固件完全一致的
9 特征 + 2s 窗口，做留一会话交叉验证（LeaveOneGroupOut），
评估"押注 TinyML 重训"这条路在现有特征下的天花板。

用法：python3 exp_head_separability.py
"""
from pathlib import Path

import numpy as np
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import LeaveOneGroupOut

from features_t1b_aligned import SESSION_DIR, FEAT_NAMES, extract

# 三类：0=STILL(静坐) 1=TYPING(打字微动) 2=HEAD(转头)
LABEL_MAP = {"still_30s": 0, "natural_typing": 1, "head_only": 2}
CLASS_NAMES = ["STILL", "TYPING", "HEAD"]
SESSIONS = [
    "20260813_160414", "20260813_165801", "20260813_170609",
    "20260822_183302", "20260822_191628",
]
ACTIONS = ["still_30s", "natural_typing", "head_only"]


def session_features(sid):
    """返回 (X, y, action_of_window)：与固件特征完全一致的滑窗特征。"""
    import csv
    rr = list(csv.DictReader(open(SESSION_DIR / f"session_{sid}_aligned.csv")))
    Xs, ys, acts = [], [], []
    for action in ACTIONS:
        part = [r for r in rr if r["alignment_valid"] == "1"
                and r["aligned_action"] == action]
        if not part:
            continue
        t = np.array([float(r["t_global"]) for r in part])
        x = np.array([int(r["x"]) for r in part])
        y = np.array([int(r["y"]) for r in part])
        es = np.array([int(r["es"]) for r in part])
        v = np.array([int(r["v"]) for r in part])
        labels = np.full(len(part), LABEL_MAP[action], dtype=int)
        X, yy = extract(t, x, y, es, v, labels)
        if len(yy):
            Xs.append(X); ys.append(yy); acts.extend([action] * len(yy))
    if not Xs:
        return np.empty((0, 9)), np.empty(0, int), []
    return np.vstack(Xs), np.concatenate(ys), acts


def confusion(y_true, y_pred):
    m = np.zeros((3, 3), dtype=int)
    for t, p in zip(y_true, y_pred):
        m[t, p] += 1
    return m


def show_confusion(m, title):
    print(f"\n{title}")
    header = "真实\\预测"
    print(f"{header:<10}" + "".join(f"{c:>9}" for c in CLASS_NAMES) + f"{'召回':>8}")
    for i, row in enumerate(m):
        recall = row[i] / max(row.sum(), 1) * 100
        print(f"{CLASS_NAMES[i]:<10}" + "".join(f"{c:>9}" for c in row) + f"{recall:>7.1f}%")
    print(f"{'准确率':<10}{np.trace(m) / max(m.sum(), 1) * 100:>8.1f}%")


def main():
    Xs, ys, groups = [], [], []
    for sid in SESSIONS:
        X, y, _ = session_features(sid)
        if len(y) == 0:
            print(f"[warn] {sid} 无样本")
            continue
        Xs.append(X); ys.append(y); groups.extend([sid] * len(y))
        print(f"{sid}: {len(y)} 窗  " +
              "  ".join(f"{CLASS_NAMES[i]}={int((y == i).sum())}" for i in range(3)))
    X = np.vstack(Xs); y = np.concatenate(ys); groups = np.asarray(groups)
    print(f"\n合计 {len(y)} 窗，特征 = {FEAT_NAMES}")

    # ── 留一会话交叉验证 ──
    logo = LeaveOneGroupOut()
    y_pred = np.empty_like(y)
    print("\n留一会话交叉验证（每折用 4 个会话训练、1 个会话测试）:")
    for tr, te in logo.split(X, y, groups):
        clf = DecisionTreeClassifier(max_depth=4, class_weight="balanced", random_state=0)
        clf.fit(X[tr], y[tr])
        y_pred[te] = clf.predict(X[te])
        acc = (y_pred[te] == y[te]).mean() * 100
        head_recall = ((y_pred[te] == 2) & (y[te] == 2)).sum() / max((y[te] == 2).sum(), 1) * 100
        print(f"  测试 {groups[te][0]}: acc={acc:5.1f}%  HEAD召回={head_recall:5.1f}%")

    m = confusion(y, y_pred)
    show_confusion(m, "LOSO 混淆矩阵（depth=4, balanced）")

    # ── 关键判据：TYPING→HEAD 误报（打字被当成转头 = 平白做反应动画）──
    type_total = m[1].sum()
    type_to_head = m[1, 2]
    head_recall = m[2, 2] / max(m[2].sum(), 1)
    print(f"\n关键指标:")
    print(f"  HEAD 召回（转头被认出来）: {head_recall * 100:.1f}%")
    print(f"  TYPING→HEAD 误报（打字被当成转头）: {type_to_head}/{type_total} = {type_to_head / max(type_total, 1) * 100:.1f}%")
    print(f"  STILL→HEAD 误报（静坐被当成转头）: {m[0, 2]}/{m[0].sum()} = {m[0, 2] / max(m[0].sum(), 1) * 100:.1f}%")

    # ── 深度敏感性 ──
    print("\n深度敏感性（LOSO 准确率 / HEAD召回 / TYPING误报为HEAD）:")
    for depth in (3, 4, 5, 6):
        yp = np.empty_like(y)
        for tr, te in logo.split(X, y, groups):
            clf = DecisionTreeClassifier(max_depth=depth, class_weight="balanced", random_state=0)
            clf.fit(X[tr], y[tr])
            yp[te] = clf.predict(X[te])
        mm = confusion(y, yp)
        print(f"  depth={depth}: acc={np.trace(mm)/mm.sum()*100:5.1f}%  "
              f"HEAD召回={mm[2,2]/max(mm[2].sum(),1)*100:5.1f}%  "
              f"TYPING→HEAD={mm[1,2]/max(mm[1].sum(),1)*100:4.1f}%")


if __name__ == "__main__":
    main()

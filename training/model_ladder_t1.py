#!/usr/bin/env python3
"""
TinyML T1 — 模型阶梯对照实验
LOSO（留一 Session 交叉验证）：
  2 sessions 训练 → 1 session 测试，轮换 3 次

阶梯：
  A. 决策树（深度 2，≈手写阈值的能力上限）
  B. 随机森林（80 树，class_weight balanced）
  C. MLP（12→16→8→5，需 tensorflow）

评估：accuracy + macro-F1 + 混淆矩阵 + 特征重要性 + 消融
"""

import json
import numpy as np
from pathlib import Path
from collections import defaultdict

from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, f1_score, confusion_matrix

OUT_DIR = Path(__file__).resolve().parent / "models"

X = np.load(OUT_DIR / 't1_X.npy')
y = np.load(OUT_DIR / 't1_y.npy')
meta = json.load(open(OUT_DIR / 't1_meta.json'))
CLASS_NAMES = meta['class_names']
N_SESSIONS = 3  # LOSO 折数

# ── 数据里没有 session 标识！需要从特征工程脚本导出 ──
# 简化：用 meta 里没有 session 信息，先按样本顺序三等分近似。
# （正式版应在 features_t1.py 里输出 session 标签，此处先用
#   均分近似，样本按 session 顺序拼接，每 session 窗口数相近）
n = len(y)
fold_size = n // N_SESSIONS
folds = [(i*fold_size, min((i+1)*fold_size, n)) for i in range(N_SESSIONS)]

FEATURE_NAMES = ['slope_X','slope_Y','slope2_X','slope2_Y',
                 'std_X','std_Y','mean_Es','std_Es',
                 'mean_abs_V','max_abs_V','Es_edge','X_reversals']

def loso_eval(model_fn, **kwargs):
    """留一折交叉验证，返回 (acc_mean, f1_mean, cm_total)"""
    accs, f1s = [], []
    cm_total = np.zeros((5, 5), dtype=int)
    for fold_idx, (s, e) in enumerate(folds):
        tr_idx = [i for i in range(n) if not (s <= i < e)]
        te_idx = list(range(s, e))
        model = model_fn(**kwargs)
        model.fit(X[tr_idx], y[tr_idx])
        pred = model.predict(X[te_idx])
        accs.append(accuracy_score(y[te_idx], pred))
        f1s.append(f1_score(y[te_idx], pred, average='macro', zero_division=0))
        cm_total += confusion_matrix(y[te_idx], pred, labels=range(5))
    return np.mean(accs), np.mean(f1s), cm_total

print("=" * 60)
print("  T1 模型阶梯（LOSO 3 折）")
print(f"  样本 {n} 窗口 × {X.shape[1]} 特征")
print("=" * 60)

# ── Baseline A: 决策树深度 2 ──
print("\n[Baseline A] 决策树 (max_depth=2)")
acc, f1, cm = loso_eval(lambda: DecisionTreeClassifier(max_depth=2, random_state=42))
print(f"  accuracy = {acc:.3f}   macro-F1 = {f1:.3f}")

# ── Baseline B: 决策树深度 4 ──
print("\n[Baseline B] 决策树 (max_depth=4)")
acc, f1, cm = loso_eval(lambda: DecisionTreeClassifier(max_depth=4, random_state=42))
print(f"  accuracy = {acc:.3f}   macro-F1 = {f1:.3f}")

# ── 候选 1: 随机森林 ──
print("\n[候选 1] 随机森林 (80 树, balanced)")
acc, f1, cm = loso_eval(lambda: RandomForestClassifier(
    n_estimators=80, max_depth=8, class_weight='balanced', random_state=42, n_jobs=-1))
print(f"  accuracy = {acc:.3f}   macro-F1 = {f1:.3f}")

# 混淆矩阵
print("\n  混淆矩阵（行=真，列=预测）:")
header = "       " + "".join(f"{c[:5]:>7}" for c in CLASS_NAMES)
print(header)
for i, row in enumerate(cm):
    line = "".join(f"{v:7d}" for v in row)
    print(f"  {CLASS_NAMES[i][:5]:>5}{line}")

# ── 特征重要性 ──
print("\n[特征重要性] RF 全数据训练:")
rf = RandomForestClassifier(n_estimators=80, max_depth=8,
                            class_weight='balanced', random_state=42, n_jobs=-1)
rf.fit(X, y)
imp = rf.feature_importances_
for name, v in sorted(zip(FEATURE_NAMES, imp), key=lambda p: -p[1]):
    bar = "█" * int(v / max(imp) * 40)
    print(f"  {name:>14s}  {v:.3f}  {bar}")

# ── 消融：逐个通道移除 ──
print("\n[消融实验] 逐通道移除（RF, macro-F1）:")
groups = {
    '全特征':        list(range(12)),
    '去 X 通道':     [i for i in range(12) if i not in (0, 2, 4, 11)],  # slope_X,slope2_X,std_X,X_reversals
    '去 Y 通道':     [i for i in range(12) if i not in (1, 3, 5)],      # slope_Y,slope2_Y,std_Y
    '去 Es 通道':    [i for i in range(12) if i not in (6, 7, 10)],     # mean_Es,std_Es,Es_edge
    '去 V 通道':     [i for i in range(12) if i not in (8, 9)],         # mean_abs_V,max_abs_V
}
for gname, cols in groups.items():
    Xg = X[:, cols]
    accs, f1s = [], []
    for s, e in folds:
        tr = [i for i in range(n) if not (s <= i < e)]
        te = list(range(s, e))
        m = RandomForestClassifier(n_estimators=80, max_depth=8,
                                   class_weight='balanced', random_state=42, n_jobs=-1)
        m.fit(Xg[tr], y[tr])
        pred = m.predict(Xg[te])
        accs.append(accuracy_score(y[te], pred))
        f1s.append(f1_score(y[te], pred, average='macro', zero_division=0))
    print(f"  {gname:<12s}  acc={np.mean(accs):.3f}  macro-F1={np.mean(f1s):.3f}")

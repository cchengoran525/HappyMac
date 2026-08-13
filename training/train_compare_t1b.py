#!/usr/bin/env python3
"""T1b 训练对比：proto / free / fused / 迁移学习"""
import json
import numpy as np
from pathlib import Path
from sklearn.ensemble import RandomForestClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import accuracy_score, f1_score, confusion_matrix

OUT = Path(__file__).resolve().parent / "models"
NAMES = ['ABSENT', 'STILL', 'LATERAL', 'APPROACH', 'RETREAT']

def rf():
    return RandomForestClassifier(n_estimators=100, max_depth=8,
                                  class_weight='balanced', random_state=42, n_jobs=-1)

def report(title, y_true, y_pred):
    acc = accuracy_score(y_true, y_pred)
    f1 = f1_score(y_true, y_pred, average='macro', zero_division=0)
    print(f"\n{title}")
    print(f"  accuracy={acc:.3f}  macro-F1={f1:.3f}")
    return acc, f1

def show_cm(y_true, y_pred):
    cm = confusion_matrix(y_true, y_pred, labels=range(5))
    print("  混淆矩阵:")
    print("        " + "".join(f"{n[:5]:>6}" for n in NAMES))
    for i, row in enumerate(cm):
        print(f"  {NAMES[i][:5]:>4} " + "".join(f"{v:6d}" for v in row))

# ── 加载 ──
Xp = np.load(OUT/'t1b_proto_X.npy'); yp = np.load(OUT/'t1b_proto_y.npy')
Xf = np.load(OUT/'t1b_free_X.npy');  yf = np.load(OUT/'t1b_free_y.npy')
Xg = np.load(OUT/'t1b_fused_X.npy'); yg = np.load(OUT/'t1b_fused_y.npy')

print("=" * 60)
print("  T1b 数据对比实验（修正后 9 特征）")
print("=" * 60)

# ── 1. 仅协议数据（3 折分层 CV，近似 LOSO）──
print("\n[1] 仅协议数据 (n=1842, 无 ABSENT)")
skf = StratifiedKFold(3, shuffle=True, random_state=42)
accs, f1s = [], []
for tr, te in skf.split(Xp, yp):
    m = rf(); m.fit(Xp[tr], yp[tr])
    pr = m.predict(Xp[te])
    accs.append(accuracy_score(yp[te], pr))
    f1s.append(f1_score(yp[te], pr, average='macro', zero_division=0))
print(f"  3折CV: acc={np.mean(accs):.3f}±{np.std(accs):.3f}  F1={np.mean(f1s):.3f}±{np.std(f1s):.3f}")

# ── 2. 仅自由数据（3 折分层 CV）──
print("\n[2] 仅自由数据 (n=806, 5 类均衡)")
accs, f1s = [], []
for tr, te in skf.split(Xf, yf):
    m = rf(); m.fit(Xf[tr], yf[tr])
    pr = m.predict(Xf[te])
    accs.append(accuracy_score(yf[te], pr))
    f1s.append(f1_score(yf[te], pr, average='macro', zero_division=0))
print(f"  3折CV: acc={np.mean(accs):.3f}±{np.std(accs):.3f}  F1={np.mean(f1s):.3f}±{np.std(f1s):.3f}")

# ── 3. 融合数据（3 折）──
print("\n[3] 融合数据 (n=2648, 5 类)")
accs, f1s = [], []
for tr, te in skf.split(Xg, yg):
    m = rf(); m.fit(Xg[tr], yg[tr])
    pr = m.predict(Xg[te])
    accs.append(accuracy_score(yg[te], pr))
    f1s.append(f1_score(yg[te], pr, average='macro', zero_division=0))
print(f"  3折CV: acc={np.mean(accs):.3f}±{np.std(accs):.3f}  F1={np.mean(f1s):.3f}±{np.std(f1s):.3f}")

# ── 4. 迁移：协议训练 → 自由测试 ──
print("\n[4] 迁移测试（协议训练 → 自由测试）★最严格")
m = rf(); m.fit(Xp, yp)
pr = m.predict(Xf)
report("  协议模型在自由数据上的表现", yf, pr)
show_cm(yf, pr)

# ── 5. 融合训练 → 自由测试 ──
print("\n[5] 融合训练 → 自由测试")
m = rf(); m.fit(Xg, yg)
pr = m.predict(Xf)
report("  融合模型在自由数据上的表现", yf, pr)
show_cm(yf, pr)

# ── 6. 融合数据上的混淆矩阵（整体）──
print("\n[6] 融合模型 3 折总混淆矩阵")
cm_total = np.zeros((5,5), dtype=int)
for tr, te in skf.split(Xg, yg):
    m = rf(); m.fit(Xg[tr], yg[tr])
    cm_total += confusion_matrix(yg[te], m.predict(Xg[te]), labels=range(5))
print("        " + "".join(f"{n[:5]:>6}" for n in NAMES))
for i, row in enumerate(cm_total):
    print(f"  {NAMES[i][:5]:>4} " + "".join(f"{v:6d}" for v in row))

# ── 特征重要性 ──
print("\n[7] 特征重要性（融合模型）")
m = rf(); m.fit(Xg, yg)
FEATS = ['|slope_X|','slope_Y','std_X','std_Y','mean_Es','std_Es',
         'mean_abs_V','max_abs_V','Es_edge']
for name, v in sorted(zip(FEATS, m.feature_importances_), key=lambda p: -p[1]):
    bar = "█" * int(v / max(m.feature_importances_) * 40)
    print(f"  {name:>12s} {v:.3f} {bar}")

#!/usr/bin/env python3
"""多 session 汇总分析：遍历所有 session，输出跨 session 平均统计"""
import csv, json
from pathlib import Path
from collections import defaultdict
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

SESSION_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR = Path(__file__).resolve().parent / "analysis"
OUT_DIR.mkdir(parents=True, exist_ok=True)

EXCLUDE_ACTIONS = {'free_5min'}  # 自由段常被截断

# ── 收集 session：白名单优先，否则全部 ──
# 正式报告只应包含"正确雷达方向 + 预热"的 session。
# 用法: python analyze_all.py 20260813_160414 20260814_xxxxxx ...
import sys
if len(sys.argv) > 1:
    SESSION_WHITELIST = sys.argv[1:]
    print(f"白名单: {SESSION_WHITELIST}")
    summaries = []
    for sid in SESSION_WHITELIST:
        sp = SESSION_DIR / f"session_{sid}_summary.json"
        if sp.exists():
            summaries.append(json.load(open(sp)))
        else:
            print(f"⚠️ 找不到 {sp.name}")
else:
    SESSION_WHITELIST = None
    summaries = []
    for sp in sorted(SESSION_DIR.glob("session_*_summary.json")):
        try:
            summaries.append(json.load(open(sp)))
        except Exception as e:
            print(f"跳过 {sp.name}: {e}")
print(f"Session 数: {len(summaries)}")

if len(summaries) == 0:
    raise SystemExit("没有 session")

# ── 每个 session 读雷达 CSV 计算逐段统计 ──
# summary 里已有 per-action stats (x_mean/x_std/y_mean/y_std/em_pct/es_pct)
# 按 action 汇总成 (session, action) → stats
action_keys = []
for s in summaries:
    for a in s.get('actions', []):
        if a['action'] not in action_keys and a['action'] not in EXCLUDE_ACTIONS:
            action_keys.append(a['action'])

# 保持出现顺序稳定
print(f"动作: {action_keys}")

# ── 表 1: 跨 session 平均 ──
rows_out = []
for ak in action_keys:
    entry = {'action': ak}
    vals = defaultdict(list)
    for s in summaries:
        for a in s.get('actions', []):
            if a['action'] == ak:
                vals['x_mean'].append(a.get('x_mean', 0))
                vals['x_std'].append(a.get('x_std', 0))
                vals['y_mean'].append(a.get('y_mean', 0))
                vals['y_std'].append(a.get('y_std', 0))
                vals['em_pct'].append(a.get('em_pct', 0))
                vals['es_pct'].append(a.get('es_pct', 0))
                break
    if not vals['x_std']: continue
    entry['n_sessions'] = len(vals['x_std'])
    for k, v in vals.items():
        entry[k + '_avg'] = float(np.mean(v))
        entry[k + '_sd'] = float(np.std(v)) if len(v) > 1 else 0.0
    rows_out.append(entry)

print(f"\n{'Action':<18} {'N':>2} {'X_mean':>8} {'X_std':>6} {'Y_mean':>7} {'Y_std':>6} {'Em%':>5} {'Es%':>5}")
print("-" * 70)
for r in rows_out:
    print(f"{r['action']:<18} {r['n_sessions']:>2} "
          f"{r['x_mean_avg']:+8.0f} {r['x_std_avg']:6.0f} "
          f"{r['y_mean_avg']:7.0f} {r['y_std_avg']:6.0f} "
          f"{r['em_pct_avg']:5.0f} {r['es_pct_avg']:5.0f}")

# ── 图: X_std / Y_std 跨 session 对比 ──
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
names = [r['action'] for r in rows_out]
x_stds = [r['x_std_avg'] for r in rows_out]
x_errs = [r['x_std_sd'] for r in rows_out]
y_stds = [r['y_std_avg'] for r in rows_out]
y_errs = [r['y_std_sd'] for r in rows_out]

axes[0].bar(range(len(names)), x_stds, yerr=x_errs, capsize=3, color='steelblue')
axes[0].set_xticks(range(len(names))); axes[0].set_xticklabels(names, rotation=45, fontsize=7, ha='right')
axes[0].set_ylabel('X σ (mm)'); axes[0].set_title(f'X Variability ({len(summaries)} sessions)')

axes[1].bar(range(len(names)), y_stds, yerr=y_errs, capsize=3, color='coral')
axes[1].set_xticks(range(len(names))); axes[1].set_xticklabels(names, rotation=45, fontsize=7, ha='right')
axes[1].set_ylabel('Y σ (mm)'); axes[1].set_title('Y Variability')

plt.tight_layout()
fig.savefig(OUT_DIR / '11_multi_session_stats.png', dpi=150)
plt.close()
print(f"\n图: {OUT_DIR / '11_multi_session_stats.png'}")

# 保存汇总 JSON 供报告使用
report = {'n_sessions': len(summaries), 'actions': rows_out}
out_json = OUT_DIR / 'multi_session_summary.json'
with open(out_json, 'w') as f:
    json.dump(report, f, indent=2, default=str)
print(f"汇总: {out_json}")

# ── 附录输出：每个 session 每轮独立数据 ──
appendix = {}
for s in summaries:
    ts = s.get('timestamp', 'unknown')
    if 'timestamp' not in s:
        # 从文件名推断
        pass
    appendix[ts] = []
    for a in s.get('actions', []):
        if a['action'] in EXCLUDE_ACTIONS: continue
        appendix[ts].append({
            'action': a['action'],
            'n': a.get('n_radar', a.get('n', 0)),
            'x_mean': a.get('x_mean', 0),
            'x_std': a.get('x_std', 0),
            'y_mean': a.get('y_mean', 0),
            'y_std': a.get('y_std', 0),
            'em_pct': a.get('em_pct', 0),
            'es_pct': a.get('es_pct', 0),
        })
appendix_json = OUT_DIR / 'appendix_per_session.json'
with open(appendix_json, 'w') as f:
    json.dump(appendix, f, indent=2, default=str)
print(f"附录数据: {appendix_json}")

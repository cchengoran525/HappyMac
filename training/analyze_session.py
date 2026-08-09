#!/usr/bin/env python3
"""Session 数据分析 + 全套可视化"""
import csv, json
from pathlib import Path
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SESSION_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR = Path(__file__).resolve().parent / "analysis"
SESSION_ID = "20260809_172535"
csv_path = SESSION_DIR / f"session_{SESSION_ID}.csv"
sum_path = SESSION_DIR / f"session_{SESSION_ID}_summary.json"
OUT_DIR.mkdir(parents=True, exist_ok=True)

rows = list(csv.DictReader(open(csv_path)))
summary = json.load(open(sum_path))
by_action = defaultdict(list)
for r in rows:
    by_action[r['action']].append({'t':float(r['t_global']),'x':int(r['x']),'y':int(r['y']),'v':int(r['v']),'em':int(r['em']),'es':int(r['es'])})
action_order = [a['action'] for a in summary['actions']]
action_names = {a['action']:a['name'] for a in summary['actions']}
colors = plt.cm.tab10(np.linspace(0,1,len(action_order)))
print(f"数据: {len(rows)}帧, {len(action_order)}阶段")

# === FIG 1: Full session X/Y timeseries ===
print("[1/6] 时间序列...")
fig,axes=plt.subplots(2,1,figsize=(18,8),sharex=True)
all_acts=[]; global_t0=None
for key in action_order:
    data=by_action[key]
    if not data: continue
    if global_t0 is None: global_t0=data[0]['t']
    t=[d['t']-data[0]['t'] for d in data]
    t_abs=[d['t']-global_t0 for d in data]
    all_acts.append({'key':key,'t_abs':t_abs,'t_rel':t,'x':[d['x'] for d in data],'y':[d['y'] for d in data]})
for i,act in enumerate(all_acts):
    axes[0].plot(act['t_abs'],act['x'],color=colors[i],alpha=0.7,linewidth=0.6)
    axes[1].plot(act['t_abs'],act['y'],color=colors[i],alpha=0.7,linewidth=0.6)
    mid=(act['t_abs'][0]+act['t_abs'][-1])/2
    axes[0].text(mid,axes[0].get_ylim()[1]*0.9,action_names[action_order[i]][:12],fontsize=5,ha='center',color=colors[i],rotation=90,alpha=0.6)
axes[0].set_ylabel('X (mm)'); axes[0].axhline(y=0,color='gray',ls='--',alpha=0.3)
axes[0].set_title('LD2450 X Coordinate — Full Session')
axes[1].set_ylabel('Y (mm)'); axes[1].set_xlabel('Time (s)')
plt.tight_layout(); fig.savefig(OUT_DIR/'01_timeseries.png',dpi=120); plt.close()

# === FIG 2: X distribution per action ===
print("[2/6] X分布...")
n=len(action_order)
fig,axes=plt.subplots(n,1,figsize=(10,2.2*n))
for i,key in enumerate(action_order):
    data=by_action[key]
    if not data: continue
    xs=[d['x'] for d in data]
    ax=axes[i] if n>1 else axes
    ax.hist(xs,bins=40,color=colors[i],alpha=0.7,edgecolor='white')
    ax.axvline(x=np.mean(xs),color='red',ls='--',lw=1)
    ax.axvline(x=0,color='gray',ls='-',alpha=0.5)
    ax.set_title(f"{action_names[key]} (n={len(xs)}, mu={np.mean(xs):+.0f}, sig={np.std(xs):.0f})",fontsize=9)
    ax.set_xlim(-600,800)
plt.tight_layout(); fig.savefig(OUT_DIR/'02_x_distribution.png',dpi=120); plt.close()

# === FIG 3: Still vs Moving ===
print("[3/6] 静止vs移动...")
still_keys=['still_30s']; move_keys=['slow_sweep','quick_points','ellipse','fwd_back','head_only']
still_xs=[]; move_xs=[]
for k in still_keys:
    if k in by_action: still_xs.extend([d['x'] for d in by_action[k]])
for k in move_keys:
    if k in by_action: move_xs.extend([d['x'] for d in by_action[k]])
fig,axes=plt.subplots(1,2,figsize=(12,5))
axes[0].hist(still_xs,bins=50,color='steelblue',alpha=0.7,label=f'Still (n={len(still_xs)}, sig={np.std(still_xs):.0f})')
axes[0].hist(move_xs,bins=50,color='coral',alpha=0.5,label=f'Moving (n={len(move_xs)}, sig={np.std(move_xs):.0f})')
axes[0].axvline(x=0,color='gray',ls='--'); axes[0].legend(); axes[0].set_title('X: Still vs Moving'); axes[0].set_xlabel('X (mm)')
x_stds=[np.std([d['x'] for d in by_action[k]]) if k in by_action else 0 for k in action_order]
names_short=[action_names[k][:20] for k in action_order]
axes[1].barh(range(len(names_short)),x_stds,color=[colors[i] for i in range(len(names_short))])
axes[1].set_yticks(range(len(names_short))); axes[1].set_yticklabels(names_short,fontsize=8)
axes[1].set_xlabel('X std (mm)'); axes[1].set_title('X Variability per Action')
for i,(v,n) in enumerate(zip(x_stds,names_short)): axes[1].text(v+2,i,f'{v:.0f}',va='center',fontsize=7)
plt.tight_layout(); fig.savefig(OUT_DIR/'03_still_vs_move.png',dpi=120); plt.close()

# === FIG 4: Drift analysis ===
print("[4/6] 慢漂分析...")
fig,axes=plt.subplots(2,2,figsize=(14,8))
drift_keys=['still_30s','natural_typing']
for idx,key in enumerate(drift_keys[:4]):
    ax=axes[idx//2][idx%2]; data=by_action.get(key,[])
    if not data: continue
    xs=[d['x'] for d in data]; t=[d['t']-data[0]['t'] for d in data]
    ax.plot(t,xs,color='steelblue',lw=0.5,alpha=0.8)
    window=min(50,len(xs))
    if window>1:
        roll=np.convolve(xs,np.ones(window)/window,mode='valid')
        ax.plot(t[window-1:],roll,color='red',lw=2,label=f'Roll50 sig={np.std(roll):.0f}')
    ax.axhline(y=np.mean(xs),color='green',ls='--',alpha=0.5)
    ax.set_title(f"{action_names.get(key,key)} drift={xs[-1]-xs[0]:+.0f}mm in {t[-1]:.0f}s")
    ax.set_ylabel('X (mm)'); ax.set_xlabel('Time(s)'); ax.legend(fontsize=7)
plt.tight_layout(); fig.savefig(OUT_DIR/'04_drift_analysis.png',dpi=120); plt.close()

# === FIG 5: X vs Em (scatter) ===
print("[5/6] X vs LD2410C...")
still=by_action.get('still_30s',[])
sweep=by_action.get('slow_sweep',[])
fig,axes=plt.subplots(1,2,figsize=(12,5))
if still:
    xs=[d['x'] for d in still]; es=[d['es'] for d in still]
    axes[0].scatter(xs,es,c=np.arange(len(xs)),cmap='viridis',s=3,alpha=0.6)
    axes[0].set_xlabel('X (mm)'); axes[0].set_ylabel('Es (stationary energy)')
    axes[0].set_title(f'STILL: X vs Es (n={len(xs)})')
if sweep:
    xs=[d['x'] for d in sweep]; ems=[d['em'] for d in sweep]
    axes[1].scatter(xs,ems,c=np.arange(len(xs)),cmap='plasma',s=3,alpha=0.6)
    axes[1].set_xlabel('X (mm)'); axes[1].set_ylabel('Em (moving energy)')
    axes[1].set_title(f'SWEEP: X vs Em (n={len(xs)})')
plt.tight_layout(); fig.savefig(OUT_DIR/'05_x_vs_ld2410c.png',dpi=120); plt.close()

# === FIG 6: Research panel ===
print("[6/6] 科研面板...")
fig=plt.figure(figsize=(16,10))
gs=fig.add_gridspec(3,3,hspace=0.35,wspace=0.3)
# X mean per action
ax=fig.add_subplot(gs[0,0])
x_means=[np.mean([d['x'] for d in by_action[k]]) if k in by_action else 0 for k in action_order]
ax.barh(range(len(names_short)),x_means,color=[colors[i] for i in range(len(names_short))])
ax.set_yticks(range(len(names_short))); ax.set_yticklabels([n[:15] for n in names_short],fontsize=7)
ax.axvline(x=0,color='gray',ls='--'); ax.set_title('X Mean per Action',fontsize=10)
# Xσ vs Yσ
ax=fig.add_subplot(gs[0,1])
for i,key in enumerate(action_order):
    if key not in by_action: continue
    sx=np.std([d['x'] for d in by_action[key]]); sy=np.std([d['y'] for d in by_action[key]])
    ax.scatter(sx,sy,color=colors[i],s=80,edgecolors='white')
    ax.annotate(action_names[key][:10],(sx,sy),fontsize=6,ha='center')
ax.set_xlabel('X sig (mm)'); ax.set_ylabel('Y sig (mm)'); ax.set_title('X vs Y Variability',fontsize=10)
mx=max([np.std([d['x'] for d in by_action[k]]) for k in action_order if k in by_action]+[1])
ax.plot([0,mx],[0,mx],'gray',ls='--',alpha=0.3)
# Em/Es
ax=fig.add_subplot(gs[0,2])
em_pcts=[summary['actions'][i]['em_pct'] for i in range(len(action_order))]
es_pcts=[summary['actions'][i]['es_pct'] for i in range(len(action_order))]
ax.bar(range(len(names_short)),em_pcts,color='coral',alpha=0.7,label='Em%')
ax.bar(range(len(names_short)),es_pcts,bottom=em_pcts,color='steelblue',alpha=0.7,label='Es%')
ax.set_xticks(range(len(names_short))); ax.set_xticklabels([n[:8] for n in names_short],fontsize=6,rotation=45)
ax.set_ylabel('%'); ax.set_title('LD2410C Em/Es',fontsize=10); ax.legend(fontsize=7)
# STILL zoom
ax=fig.add_subplot(gs[1,:2])
if still:
    t_s=[d['t']-still[0]['t'] for d in still]; x_s=[d['x'] for d in still]; y_s=[d['y'] for d in still]
    ax.plot(t_s,x_s,color='steelblue',lw=0.5,alpha=0.8,label='X')
    ax.axhline(y=x_s[0],color='green',ls='--',alpha=0.4,label=f'Start={x_s[0]:+d}')
    ax.axhline(y=x_s[-1],color='red',ls='--',alpha=0.4,label=f'End={x_s[-1]:+d}')
    ax.set_ylabel('X (mm)'); ax.set_title(f'STILL 30s: drift={x_s[-1]-x_s[0]:+.0f}mm sig={np.std(x_s):.0f}mm',fontsize=10)
    ax2=ax.twinx(); ax2.plot(t_s,y_s,color='orange',lw=0.5,alpha=0.5,label='Y'); ax2.set_ylabel('Y (mm)')
    l1,b1=ax.get_legend_handles_labels(); l2,b2=ax2.get_legend_handles_labels()
    ax.legend(l1+l2,b1+b2,fontsize=7)
# HEAD zoom
ax=fig.add_subplot(gs[1,2])
head=by_action.get('head_only',[])
if head:
    t_h=[d['t']-head[0]['t'] for d in head]; x_h=[d['x'] for d in head]
    ax.plot(t_h,x_h,color='steelblue',lw=0.8); ax.set_ylabel('X (mm)'); ax.set_xlabel('Time(s)')
    ax.set_title(f'HEAD ONLY: sig={np.std(x_h):.0f} span={max(x_h)-min(x_h)}mm',fontsize=10)
    ax.axhline(y=0,color='gray',ls='--',alpha=0.3)
# SLOW vs QUICK
ax=fig.add_subplot(gs[2,:])
for key,label,ls in [('slow_sweep','Slow Sweep','-'),('quick_points','Quick Points','--')]:
    data=by_action.get(key,[])
    if not data: continue
    t_s=[d['t']-data[0]['t'] for d in data]; x_s=[d['x'] for d in data]
    ax.plot(t_s,x_s,lw=0.8,ls=ls,alpha=0.8,label=f'{label} (sig={np.std(x_s):.0f})')
ax.axhline(y=0,color='gray',ls='--',alpha=0.3)
ax.set_ylabel('X (mm)'); ax.set_xlabel('Time(s)'); ax.set_title('Lateral Movement Comparison',fontsize=10); ax.legend()
plt.tight_layout(); fig.savefig(OUT_DIR/'06_research_panel.png',dpi=150); plt.close()

# ── Summary ──
print(f"\n✅ 6 张图 -> {OUT_DIR}/")
print(f"  01_timeseries.png     全阶段X/Y时间序列")
print(f"  02_x_distribution.png  每阶段X分布直方图")
print(f"  03_still_vs_move.png   静止vs移动对比")
print(f"  04_drift_analysis.png  静止阶段慢漂分析")
print(f"  05_x_vs_ld2410c.png    X vs LD2410C能量散点")
print(f"  06_research_panel.png  综合科研面板")

still=by_action.get('still_30s',[])
head=by_action.get('head_only',[])
sweep=by_action.get('slow_sweep',[])
print(f"\n=== 关键数据 ===")
if still:
    xs=[d['x'] for d in still]
    print(f"STILL: drift={xs[-1]-xs[0]:+.0f}mm/30s sig={np.std(xs):.0f}mm")
if head and still:
    print(f"HEAD sig: {np.std([d['x'] for d in head]):.0f}mm")
    print(f"HEAD/STILL: {np.std([d['x'] for d in head])/max(np.std([d['x'] for d in still]),1):.1f}x")
if sweep and still:
    print(f"SWEEP sig: {np.std([d['x'] for d in sweep]):.0f}mm")
    print(f"SWEEP/STILL: {np.std([d['x'] for d in sweep])/max(np.std([d['x'] for d in still]),1):.1f}x")
ems=[int(r['em']) for r in rows]; ess=[int(r['es']) for r in rows]
print(f"Em avg: {np.mean(ems):.0f}  Es avg: {np.mean(ess):.0f}")
print(f"Es>0 pct: {sum(1 for e in ess if e>0)/len(ess)*100:.0f}%")

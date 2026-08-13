#!/usr/bin/env python3
"""
TinyML T1 — 标签生成 + 特征工程

输入: 3 个正式 session 的雷达 CSV（含协议阶段标签）
输出: training/models/t1_X.npy, t1_y.npy, t1_meta.json

标签体系（躯干质心物理签名，5 类）:
  0 ABSENT    雷达无目标
  1 STILL     躯干静止（含打字/拿东西/发呆）
  2 LATERAL   躯干横向持续位移
  3 APPROACH  躯干向雷达持续位移（Y 减速下降）
  4 RETREAT   躯干远离雷达持续位移（Y 加速上升）

特征（2s 窗口，~20 帧，步长 0.5s）:
  slope_X, slope_Y         一阶趋势
  slope2_X, slope2_Y       二阶趋势（加减速）
  std_X, std_Y             幅度
  mean_Es, std_Es          姿态
  mean_abs_V, max_abs_V    运动强度
  Es_edge                  存在性边沿
  X_reversals              方向切换次数
"""

import csv, json
import numpy as np
from pathlib import Path
from collections import defaultdict

SESSION_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR = Path(__file__).resolve().parent / "models"
OUT_DIR.mkdir(parents=True, exist_ok=True)

SESSIONS = ['20260813_160414', '20260813_165801', '20260813_170609']

# ── 协议标签 → 5 类映射 ──
PROTOCOL_MAP = {
    'still_30s':      1,  # STILL
    'natural_typing': 1,  # STILL（躯干静止）
    'natural_reach':  1,  # STILL（瞬时扰动，T1 归静止）
    'slow_sweep':     2,  # LATERAL
    'quick_points':   2,  # LATERAL
    'ellipse':        2,  # LATERAL
    'fwd_back':      -1,  # 需按 Y 斜率切段（APPROACH/RETREAT）
    'stand_sit':     -1,  # 站起坐下含两类位移，按斜率切
    'head_only':      1,  # STILL（躯干不动）
    'free_5min':     -2,  # 排除
}
CLASS_NAMES = ['ABSENT', 'STILL', 'LATERAL', 'APPROACH', 'RETREAT']

# ── 窗口参数 ──
WINDOW_SEC = 2.0
STRIDE_SEC = 0.5

def load_session(sid):
    """加载雷达 CSV，返回 (t, x, y, es, v, proto_label)"""
    rows = list(csv.DictReader(open(SESSION_DIR / f"session_{sid}.csv")))
    t = np.array([float(r['t_global']) for r in rows])
    x = np.array([int(r['x']) for r in rows])
    y = np.array([int(r['y']) for r in rows])
    es = np.array([int(r['es']) for r in rows])
    v = np.array([int(r['v']) for r in rows])
    proto = [r['action'] for r in rows]
    return t, x, y, es, v, proto

def segment_fwd_back(y, dt_samples, window_n, min_dur_s=0.8):
    """
    用 Y 斜率自动切段 APPROACH/RETREAT。
    返回每帧的段标签：3=APPROACH, 4=RETREAT, -1=过渡（丢弃）
    规则：rolling slope 持续同号 >= min_dur_s
    """
    # 平滑 Y 后算 rolling slope（每 window_n 帧拟合一次）
    labels = np.full(len(y), -1, dtype=int)
    smooth = np.convolve(y, np.ones(10)/10, mode='same')
    min_frames = int(min_dur_s / (dt_samples or 0.1))

    i = 0
    n = len(y)
    while i < n - window_n:
        seg = smooth[i:i+window_n]
        slope = np.polyfit(np.arange(window_n), seg, 1)[0]
        # 用斜率符号 + 幅度判断（8mm/帧 ≈ 80mm/s 人体躯干移动门槛）
        if slope < -8:  # Y 持续下降 = 凑近
            j = i
            while j < n and j < i + window_n:
                labels[j] = 3
                j += 1
            # 延伸：只要后续窗口仍为负斜率就继续标
            k = i + window_n
            while k + window_n < n:
                nxt = smooth[k:k+window_n]
                sl2 = np.polyfit(np.arange(window_n), nxt, 1)[0]
                if sl2 < -8:
                    labels[k:k+window_n] = 3
                    k += window_n // 2
                else:
                    break
            i = k
        elif slope > 8:  # Y 持续上升 = 远离
            j = i
            while j < n and j < i + window_n:
                labels[j] = 4
                j += 1
            k = i + window_n
            while k + window_n < n:
                nxt = smooth[k:k+window_n]
                sl2 = np.polyfit(np.arange(window_n), nxt, 1)[0]
                if sl2 > 8:
                    labels[k:k+window_n] = 4
                    k += window_n // 2
                else:
                    break
            i = k
        else:
            i += window_n // 2  # 平段跳半步
    return labels

def extract_features(t, x, y, es, v, start, end):
    """从窗口 [start, end) 提取 12 特征"""
    xs = x[start:end]; ys = y[start:end]
    ess = es[start:end]; vs = v[start:end]
    n = len(xs)
    if n < 5:
        return None
    k = np.arange(n)

    feats = []
    # 一阶趋势
    feats.append(np.polyfit(k, xs, 1)[0] if n > 2 else 0)   # slope_X
    feats.append(np.polyfit(k, ys, 1)[0] if n > 2 else 0)   # slope_Y
    # 二阶趋势
    feats.append(np.polyfit(k, xs, 2)[0] if n > 4 else 0)   # slope2_X
    feats.append(np.polyfit(k, ys, 2)[0] if n > 4 else 0)   # slope2_Y
    # 幅度
    feats.append(np.std(xs))                                # std_X
    feats.append(np.std(ys))                                # std_Y
    # 姿态（Es 数值——实验发现是有效弱特征）
    feats.append(np.mean(ess))                              # mean_Es
    feats.append(np.std(ess))                               # std_Es
    # 运动强度
    feats.append(np.mean(np.abs(vs)))                       # mean_abs_V
    feats.append(np.max(np.abs(vs)))                        # max_abs_V
    # 存在性边沿（Es 0→非0 跳变次数）
    edge = sum(1 for i in range(1, n) if ess[i] > 0 and ess[i-1] == 0)
    feats.append(edge)                                      # Es_edge
    # X 方向切换次数（>20mm 计一次）
    dx = np.diff(xs)
    reversals = sum(1 for i in range(1, len(dx)) if dx[i]*dx[i-1] < 0 and abs(dx[i]) > 20)
    feats.append(reversals)                                 # X_reversals
    return np.array(feats, dtype=np.float32)

def main():
    all_X, all_y = [], []
    stats = defaultdict(int)

    for sid in SESSIONS:
        t, x, y, es, v, proto = load_session(sid)
        n = len(t)
        if n < 100:
            print(f"跳过 {sid}: 数据太少")
            continue

        # 帧级标签
        frame_labels = np.full(n, -2, dtype=int)
        for i, p in enumerate(proto):
            frame_labels[i] = PROTOCOL_MAP.get(p, -2)

        # fwd_back / stand_sit 按 Y 斜率切段
        for target_proto in ['fwd_back', 'stand_sit']:
            mask = np.array([p == target_proto for p in proto])
            if not mask.any():
                continue
            idx = np.where(mask)[0]
            # 只对连续段做（假设单段）
            seg_labels = segment_fwd_back(y[idx], np.median(np.diff(t[idx])) if len(idx)>1 else 0.1, 20)
            frame_labels[idx] = seg_labels

        # 滑窗
        window_n = int(WINDOW_SEC / max(np.median(np.diff(t)), 0.02))
        stride_n = int(STRIDE_SEC / max(np.median(np.diff(t)), 0.02))
        window_n = max(window_n, 10)
        stride_n = max(stride_n, 5)

        for start in range(0, n - window_n, stride_n):
            end = start + window_n
            # 窗口标签：众数，全 -2（排除）或 -1（过渡）则跳过
            wl = frame_labels[start:end]
            valid = wl[(wl >= 0)]
            if len(valid) < window_n * 0.6:  # 至少 60% 有效
                continue
            label = int(np.bincount(valid).argmax())
            feats = extract_features(t, x, y, es, v, start, end)
            if feats is None:
                continue
            all_X.append(feats)
            all_y.append(label)
            stats[CLASS_NAMES[label]] += 1

    X = np.stack(all_X)
    y = np.array(all_y)
    np.save(OUT_DIR / 't1_X.npy', X)
    np.save(OUT_DIR / 't1_y.npy', y)

    meta = {
        'sessions': SESSIONS,
        'window_sec': WINDOW_SEC,
        'stride_sec': STRIDE_SEC,
        'n_samples': len(y),
        'n_features': X.shape[1],
        'class_distribution': dict(stats),
        'class_names': CLASS_NAMES,
    }
    with open(OUT_DIR / 't1_meta.json', 'w') as f:
        json.dump(meta, f, indent=2)

    print(f"✅ 特征工程完成")
    print(f"  样本: {len(y)} 窗口")
    print(f"  特征: {X.shape[1]} 维")
    print(f"  类别分布: {dict(stats)}")
    print(f"  输出: {OUT_DIR / 't1_X.npy'} / t1_y.npy")

if __name__ == '__main__':
    main()

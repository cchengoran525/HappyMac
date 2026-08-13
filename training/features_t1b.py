#!/usr/bin/env python3
"""
TinyML T1b — 特征工程（支持协议 session + 自由 session 融合）

自由 session 的标签是视觉 Teacher 自动打的（0-4 已有），
需平滑：2 秒滑窗内多数投票吸收自动标签抖动。

输出三种数据集的 X/y：
  t1b_free_X/y      — 仅自由 session
  t1b_proto_X/y     — 仅 3 个协议 session
  t1b_fused_X/y     — 两者融合
"""

import csv, json
import numpy as np
from pathlib import Path
from collections import defaultdict

SESSION_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR = Path(__file__).resolve().parent / "models"
OUT_DIR.mkdir(parents=True, exist_ok=True)

PROTO_SESSIONS = ['20260813_160414', '20260813_165801', '20260813_170609']
FREE_SESSION = '20260813_200450_free'

PROTOCOL_MAP = {
    'still_30s': 1, 'natural_typing': 1, 'natural_reach': 1, 'head_only': 1,
    'slow_sweep': 2, 'quick_points': 2, 'ellipse': 2,
    'fwd_back': -1, 'stand_sit': -1, 'free_5min': -2,
}
CLASS_NAMES = ['ABSENT', 'STILL', 'LATERAL', 'APPROACH', 'RETREAT']
WINDOW_SEC, STRIDE_SEC = 2.0, 0.5

FEAT_NAMES = ['|slope_X|','slope_Y','std_X','std_Y','mean_Es','std_Es',
              'mean_abs_V','max_abs_V','Es_edge']  # 9 特征（修掉废特征）

def load_proto(sid):
    """协议 session：原始列 + 协议映射标签"""
    rows = list(csv.DictReader(open(SESSION_DIR / f"session_{sid}.csv")))
    t = np.array([float(r['t_global']) for r in rows])
    x = np.array([int(r['x']) for r in rows]); y = np.array([int(r['y']) for r in rows])
    es = np.array([int(r['es']) for r in rows]); v = np.array([int(r['v']) for r in rows])
    proto = [r['action'] for r in rows]
    return t, x, y, es, v, proto

def load_free():
    """自由 session：自带视觉标签"""
    rows = list(csv.DictReader(open(SESSION_DIR / f"session_{FREE_SESSION}.csv")))
    t = np.array([float(r['t_global']) for r in rows])
    x = np.array([int(r['x']) for r in rows]); y = np.array([int(r['y']) for r in rows])
    es = np.array([int(r['es']) for r in rows]); v = np.array([int(r['v']) for r in rows])
    lab = np.array([int(r['label']) for r in rows])
    return t, x, y, es, v, lab

def segment_fwd_back(y, window_n):
    """Y 斜率自动切段（同 features_t1.py）"""
    labels = np.full(len(y), -1, dtype=int)
    smooth = np.convolve(y, np.ones(10)/10, mode='same')
    i, n = 0, len(y)
    while i < n - window_n:
        slope = np.polyfit(np.arange(window_n), smooth[i:i+window_n], 1)[0]
        if slope < -8:
            k = i
            while k + window_n < n and np.polyfit(np.arange(window_n), smooth[k:k+window_n], 1)[0] < -8:
                labels[k:k+window_n//2] = 3
                k += window_n // 2
            labels[i:min(k+window_n, n)] = 3
            i = k
        elif slope > 8:
            k = i
            while k + window_n < n and np.polyfit(np.arange(window_n), smooth[k:k+window_n], 1)[0] > 8:
                labels[k:k+window_n//2] = 4
                k += window_n // 2
            labels[i:min(k+window_n, n)] = 4
            i = k
        else:
            i += window_n // 2
    return labels

def extract(t, x, y, es, v, frame_labels, window_n, stride_n):
    """滑窗提取特征（9 维）"""
    X_list, y_list = [], []
    n = len(t)
    for start in range(0, n - window_n, stride_n):
        end = start + window_n
        wl = frame_labels[start:end]
        valid = wl[wl >= 0]
        if len(valid) < window_n * 0.6:
            continue
        label = int(np.bincount(valid).argmax())
        xs, ys = x[start:end], y[start:end]
        ess, vs = es[start:end], v[start:end]
        k = np.arange(len(xs))
        feats = [
            abs(np.polyfit(k, xs, 1)[0]),   # |slope_X|
            np.polyfit(k, ys, 1)[0],        # slope_Y
            np.std(xs),                     # std_X
            np.std(ys),                     # std_Y
            np.mean(ess),                   # mean_Es
            np.std(ess),                    # std_Es
            np.mean(np.abs(vs)),            # mean_abs_V
            np.max(np.abs(vs)),             # max_abs_V
            sum(1 for i in range(1, len(ess)) if ess[i] > 0 and ess[i-1] == 0),  # Es_edge
        ]
        X_list.append(feats); y_list.append(label)
    return np.array(X_list, dtype=np.float32), np.array(y_list, dtype=int)

def build_proto():
    all_X, all_y = [], []
    for sid in PROTO_SESSIONS:
        t, x, y, es, v, proto = load_proto(sid)
        frame_labels = np.array([PROTOCOL_MAP.get(p, -2) for p in proto])
        dt = np.median(np.diff(t)) or 0.06
        wn, sn = max(int(WINDOW_SEC/dt), 10), max(int(STRIDE_SEC/dt), 5)
        for tp in ['fwd_back', 'stand_sit']:
            mask = np.array([p == tp for p in proto])
            if mask.any():
                idx = np.where(mask)[0]
                frame_labels[idx] = segment_fwd_back(y[idx], 20)
        X, yy = extract(t, x, y, es, v, frame_labels, wn, sn)
        all_X.append(X); all_y.append(yy)
    return np.vstack(all_X), np.concatenate(all_y)

def build_free():
    t, x, y, es, v, lab = load_free()
    # 平滑：20 帧多数投票（吸收自动标签抖动）
    smooth_lab = lab.copy()
    for i in range(len(lab)):
        s = max(0, i-10); e = min(len(lab), i+11)
        smooth_lab[i] = np.bincount(lab[s:e]).argmax()
    dt = np.median(np.diff(t)) or 0.06
    wn, sn = max(int(WINDOW_SEC/dt), 10), max(int(STRIDE_SEC/dt), 5)
    return extract(t, x, y, es, v, smooth_lab, wn, sn)

if __name__ == '__main__':
    Xp, yp = build_proto()
    Xf, yf = build_free()
    X_fused = np.vstack([Xp, Xf]); y_fused = np.concatenate([yp, yf])

    np.save(OUT_DIR/'t1b_proto_X.npy', Xp); np.save(OUT_DIR/'t1b_proto_y.npy', yp)
    np.save(OUT_DIR/'t1b_free_X.npy', Xf);  np.save(OUT_DIR/'t1b_free_y.npy', yf)
    np.save(OUT_DIR/'t1b_fused_X.npy', X_fused); np.save(OUT_DIR/'t1b_fused_y.npy', y_fused)

    for name, yy in [('proto', yp), ('free', yf), ('fused', y_fused)]:
        dist = {CLASS_NAMES[c]: int((yy == c).sum()) for c in range(5)}
        print(f"{name:>6s}: {len(yy)} 窗口  {dist}")
    print(f"\n✅ 保存到 {OUT_DIR}/t1b_*.npy")

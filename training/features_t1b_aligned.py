#!/usr/bin/env python3
"""从 5 次规则 + 3 次自由的对齐文件生成 T1b v0 特征。

规则数据只使用 alignment_valid=1；自由数据只使用 radar_valid=1。
规则动作标签沿用原 T1b 试验映射，fwd_back/stand_sit 再用 Y 斜率
粗分 APPROACH/RETREAT。每个规则 action 单独切窗，避免跨倒计时空档。
"""

import csv
import json
from pathlib import Path

import numpy as np


SESSION_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR = Path(__file__).resolve().parent / "models"
OUT_DIR.mkdir(parents=True, exist_ok=True)

RULE_SESSIONS = [
    "20260813_160414", "20260813_165801", "20260813_170609",
    "20260822_183302", "20260822_191628",
]
FREE_SESSIONS = ["20260813_200450", "20260822_190703", "20260822_192412"]

PROTOCOL_MAP = {
    "still_30s": 1, "natural_typing": 1, "natural_reach": 1,
    "head_only": 1,
    "slow_sweep": 2, "quick_points": 2, "ellipse": 2,
}
CLASS_NAMES = ["ABSENT", "STILL", "LATERAL", "APPROACH", "RETREAT"]
FEAT_NAMES = ["|slope_X|", "slope_Y", "std_X", "std_Y", "mean_Es",
              "std_Es", "mean_abs_V", "max_abs_V", "Es_edge"]
WINDOW_SEC, STRIDE_SEC = 2.0, 0.5


def rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def segment_fwd_back(y, window_n=20):
    labels = np.full(len(y), -1, dtype=int)
    if len(y) < window_n:
        return labels
    smooth = np.convolve(y, np.ones(10) / 10, mode="same")
    i, n = 0, len(y)
    while i < n - window_n:
        slope = np.polyfit(np.arange(window_n), smooth[i:i + window_n], 1)[0]
        if slope < -8 or slope > 8:
            label = 3 if slope < -8 else 4
            k = i
            while k + window_n < n:
                s = np.polyfit(np.arange(window_n), smooth[k:k + window_n], 1)[0]
                if (label == 3 and s >= -8) or (label == 4 and s <= 8):
                    break
                labels[k:k + window_n // 2] = label
                k += window_n // 2
            labels[i:min(k + window_n, n)] = label
            i = k
        else:
            i += window_n // 2
    return labels


def extract(t, x, y, es, v, frame_labels):
    if len(t) < 12:
        return np.empty((0, len(FEAT_NAMES)), dtype=np.float32), np.empty(0, dtype=int)
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        dt = 0.06
    wn = max(int(WINDOW_SEC / dt), 10)
    sn = max(int(STRIDE_SEC / dt), 5)
    X_list, y_list = [], []
    for start in range(0, len(t) - wn + 1, sn):
        end = start + wn
        valid = frame_labels[start:end]
        valid = valid[valid >= 0]
        if len(valid) < wn * 0.6:
            continue
        label = int(np.bincount(valid, minlength=5).argmax())
        xs, ys, ess, vs = x[start:end], y[start:end], es[start:end], v[start:end]
        k = np.arange(len(xs))
        X_list.append([
            abs(np.polyfit(k, xs, 1)[0]),
            np.polyfit(k, ys, 1)[0],
            np.std(xs), np.std(ys), np.mean(ess), np.std(ess),
            np.mean(np.abs(vs)), np.max(np.abs(vs)),
            sum(ess[i] > 0 and ess[i - 1] == 0 for i in range(1, len(ess))),
        ])
        y_list.append(label)
    return np.asarray(X_list, dtype=np.float32), np.asarray(y_list, dtype=int)


def rule_session(sid):
    rr = rows(SESSION_DIR / f"session_{sid}_aligned.csv")
    all_x, all_y, all_g = [], [], []
    action_counts = {}
    for action in ["still_30s", "slow_sweep", "quick_points", "ellipse",
                   "fwd_back", "head_only", "natural_typing", "natural_reach",
                   "stand_sit"]:
        part = [r for r in rr if r["alignment_valid"] == "1"
                and r["aligned_action"] == action]
        if not part:
            continue
        t = np.array([float(r["t_global"]) for r in part])
        x = np.array([int(r["x"]) for r in part])
        y = np.array([int(r["y"]) for r in part])
        es = np.array([int(r["es"]) for r in part])
        v = np.array([int(r["v"]) for r in part])
        labels = np.full(len(part), PROTOCOL_MAP.get(action, -1), dtype=int)
        if action in ("fwd_back", "stand_sit"):
            labels = segment_fwd_back(y)
        X, yy = extract(t, x, y, es, v, labels)
        if len(yy):
            all_x.append(X); all_y.append(yy); all_g.extend([sid] * len(yy))
        action_counts[action] = len(yy)
    if not all_x:
        return np.empty((0, len(FEAT_NAMES)), dtype=np.float32), np.empty(0, dtype=int), [], action_counts
    return np.vstack(all_x), np.concatenate(all_y), all_g, action_counts


def free_session(sid):
    rr = rows(SESSION_DIR / f"session_{sid}_free_aligned.csv")
    t = np.array([float(r["t_global"]) for r in rr])
    x = np.array([int(r["x"]) for r in rr])
    y = np.array([int(r["y"]) for r in rr])
    es = np.array([int(r["es"]) for r in rr])
    v = np.array([int(r["v"]) for r in rr])
    labels = np.array([int(r["label"]) if r["radar_valid"] == "1" else -1
                       for r in rr])
    X, yy = extract(t, x, y, es, v, labels)
    return X, yy, [sid] * len(yy), {"free": len(yy)}


def build():
    Xs, ys, groups, detail = [], [], [], {}
    for sid in RULE_SESSIONS:
        X, y, g, d = rule_session(sid)
        Xs.append(X); ys.append(y); groups.extend(g); detail[sid] = d
    for sid in FREE_SESSIONS:
        X, y, g, d = free_session(sid)
        Xs.append(X); ys.append(y); groups.extend(g); detail[sid] = d
    X = np.vstack(Xs); y = np.concatenate(ys); groups = np.asarray(groups)
    np.save(OUT_DIR / "t1b_aligned_X.npy", X)
    np.save(OUT_DIR / "t1b_aligned_y.npy", y)
    np.save(OUT_DIR / "t1b_aligned_groups.npy", groups)
    meta = {
        "source": "5 rule aligned + 3 free aligned sessions",
        "feature_names": FEAT_NAMES, "class_names": CLASS_NAMES,
        "window_sec": WINDOW_SEC, "stride_sec": STRIDE_SEC,
        "n_samples": len(y), "detail": detail,
        "counts": {CLASS_NAMES[i]: int(np.sum(y == i)) for i in range(5)},
    }
    (OUT_DIR / "t1b_aligned_meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
    return X, y, groups, meta


if __name__ == "__main__":
    X, y, groups, meta = build()
    print(f"aligned v0: X={X.shape}, groups={len(set(groups))}")
    print(meta["counts"])

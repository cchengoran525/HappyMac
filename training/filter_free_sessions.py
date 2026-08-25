#!/usr/bin/env python3
"""为自由 session 生成非破坏性的过滤版雷达标签和雷达有效性标记。

原始 free CSV 的 label 是雷达队列被排空时写入的当前 Teacher label，
因此会受串口队列延迟影响。这里改用 *_free_labels.csv 的时间序列作为
标签来源，先用首尾时间做 session 级时间归一，再对雷达窗口做时间多数投票。

原始文件不会被覆盖，输出：
  session_<id>_free_filtered.csv
  session_<id>_free_filtered_meta.json
  session_<id>_free_clean.csv
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


SESSION_DIR = Path(__file__).resolve().parent / "sessions"
LABEL_NAMES = ["ABSENT", "STILL", "LATERAL", "APPROACH", "RETREAT"]


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def make_runs(labels, times):
    runs = []
    start = 0
    for i in range(1, len(labels) + 1):
        if i == len(labels) or labels[i] != labels[start]:
            runs.append((start, i, int(labels[start])))
            start = i
    return runs


def majority_filter(labels, times, window_sec=2.0, min_run_sec=0.6):
    """时间窗口多数投票，再去掉很短的孤立脉冲。"""
    labels = np.asarray(labels, dtype=int)
    times = np.asarray(times, dtype=float)
    out = np.empty_like(labels)

    for i, t in enumerate(times):
        lo = np.searchsorted(times, t - window_sec / 2.0, side="left")
        hi = np.searchsorted(times, t + window_sec / 2.0, side="right")
        out[i] = np.bincount(labels[lo:hi], minlength=len(LABEL_NAMES)).argmax()

    # 最多两轮：短促的自动标签闪现，回填为相邻较长状态。
    for _ in range(2):
        runs = make_runs(out, times)
        for k, (a, b, label) in enumerate(runs):
            duration = times[b - 1] - times[a]
            if duration >= min_run_sec or k == 0 or k == len(runs) - 1:
                continue
            left_a, left_b, left_label = runs[k - 1]
            right_a, right_b, right_label = runs[k + 1]
            left_duration = times[left_b - 1] - times[left_a]
            right_duration = times[right_b - 1] - times[right_a]
            out[a:b] = left_label if left_duration >= right_duration else right_label
    return out


def filter_session(session_id, window_sec=2.0, min_run_sec=0.6):
    prefix = SESSION_DIR / f"session_{session_id}_free"
    radar_path = prefix.with_suffix(".csv")
    labels_path = Path(str(prefix) + "_labels.csv")
    if not radar_path.exists() or not labels_path.exists():
        raise FileNotFoundError(f"缺少 {radar_path.name} 或 {labels_path.name}")

    radar = read_csv(radar_path)
    teacher = read_csv(labels_path)
    if not radar or not teacher:
        raise ValueError(f"空文件：{session_id}")

    radar_t = np.array([float(r["t_global"]) for r in radar], dtype=float)
    teacher_t = np.array([float(r["t_global"]) for r in teacher], dtype=float)
    teacher_y = np.array([int(r["label"]) for r in teacher], dtype=int)

    # sync_ts 与 free 的 t0 没有同时写入旧文件；用首尾把两条时间轴
    # 对齐。对 2 秒滑窗 v0 训练足够，且不改写原始数据。
    if radar_t[-1] <= radar_t[0] or teacher_t[-1] <= teacher_t[0]:
        raise ValueError(f"时间轴无效：{session_id}")
    radar_on_teacher_time = (
        (radar_t - radar_t[0]) / (radar_t[-1] - radar_t[0])
        * (teacher_t[-1] - teacher_t[0]) + teacher_t[0]
    )
    idx = np.searchsorted(teacher_t, radar_on_teacher_time, side="right") - 1
    idx = np.clip(idx, 0, len(teacher_y) - 1)
    relabeled = teacher_y[idx]
    filtered = majority_filter(relabeled, radar_on_teacher_time,
                               window_sec=window_sec,
                               min_run_sec=min_run_sec)

    output_path = Path(str(prefix) + "_filtered.csv")
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_global", "x", "y", "v", "em", "es", "label"])
        for row, label in zip(radar, filtered):
            writer.writerow([row["t_global"], row["x"], row["y"], row["v"],
                             row["em"], row["es"], int(label)])

    # 只屏蔽“视觉确认有人，但 LD2450 同时跑到明显不可信区域”的行。
    # x 有正负方向，所以使用 abs(x)。原始 x/y 保留，避免不可逆覆盖。
    x = np.array([float(r["x"]) for r in radar], dtype=float)
    y = np.array([float(r["y"]) for r in radar], dtype=float)
    radar_outlier = (np.abs(x) > 1000.0) & (y > 1000.0) & (filtered != 0)
    radar_valid = (~radar_outlier).astype(int)

    clean_path = Path(str(prefix) + "_clean.csv")
    with open(clean_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_global", "x", "y", "v", "em", "es",
                         "label", "radar_valid"])
        for row, label, valid in zip(radar, filtered, radar_valid):
            writer.writerow([row["t_global"], row["x"], row["y"], row["v"],
                             row["em"], row["es"], int(label), int(valid)])

    raw_label = np.array([int(r["label"]) for r in radar], dtype=int)
    meta = {
        "session_id": session_id,
        "source_radar": radar_path.name,
        "source_teacher_labels": labels_path.name,
        "output": output_path.name,
        "clean_output": clean_path.name,
        "method": "teacher_timeline_endpoint_affine + time_majority",
        "window_sec": window_sec,
        "min_run_sec": min_run_sec,
        "n_radar": len(radar),
        "changed_from_raw_radar_label_fraction": float(np.mean(filtered != raw_label)),
        "changed_by_majority_fraction": float(np.mean(filtered != relabeled)),
        "radar_outlier_rule": "abs(x)>1000 and y>1000 and filtered_label!=ABSENT",
        "radar_invalid_rows": int(np.sum(radar_outlier)),
        "raw_radar_counts": {str(i): int(np.sum(raw_label == i)) for i in range(5)},
        "filtered_counts": {str(i): int(np.sum(filtered == i)) for i in range(5)},
    }
    meta_path = Path(str(prefix) + "_filtered_meta.json")
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    return meta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", action="append", dest="sessions",
                        help="session id without _free; repeatable")
    parser.add_argument("--window", type=float, default=2.0)
    parser.add_argument("--min-run", type=float, default=0.6)
    args = parser.parse_args()

    sessions = args.sessions
    if not sessions:
        sessions = [p.name[len("session_"):-len("_free.csv")]
                    for p in sorted(SESSION_DIR.glob("session_*_free.csv"))]

    for sid in sessions:
        meta = filter_session(sid, args.window, args.min_run)
        print(f"{sid}: label_changed={meta['changed_from_raw_radar_label_fraction']:.1%}, "
              f"radar_invalid={meta['radar_invalid_rows']}, "
              f"filtered={meta['filtered_counts']}")


if __name__ == "__main__":
    main()

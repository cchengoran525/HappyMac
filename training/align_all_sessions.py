#!/usr/bin/env python3
"""为已有的 5 次规则 + 3 次自由 session 生成非破坏性对齐文件。

规则 session:
  - 以视频 vidts 的每个 action 首尾时间作为真实阶段窗口；
  - 用规则阶段起点和雷达原始 action 起点估计 session 级时间偏移；
  - 阶段间的倒计时/串口队列污染行保留，但标为 alignment_valid=0；
  - 阶段内标签以视频 action 为准，不再使用 collector 写入的 action。

自由 session:
  - 读取 filter_free_sessions.py 生成的 *_free_clean.csv；
  - 将雷达时间映射到 Teacher label 时间轴，保留 radar_valid；
  - 输出统一字段，方便后续特征提取器批量读取。

原始 CSV 和已有 clean/filtered 文件都不会被覆盖。
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


SESSION_DIR = Path(__file__).resolve().parent / "sessions"
RULE_SESSIONS = [
    "20260813_160414",
    "20260813_165801",
    "20260813_170609",
    "20260822_183302",
    "20260822_191628",
]
FREE_SESSIONS = [
    "20260813_200450",
    "20260822_190703",
    "20260822_192412",
]
RULE_ACTIONS = [
    "still_30s", "slow_sweep", "quick_points", "ellipse", "fwd_back",
    "head_only", "natural_typing", "natural_reach", "stand_sit",
]


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, fieldnames, rows):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def action_ranges(vidts):
    grouped = defaultdict(list)
    for row in vidts:
        grouped[row["action"]].append(float(row["t_wallclock"]))
    order = []
    for row in vidts:
        action = row["action"]
        if action not in order:
            order.append(action)
    return [(action, min(grouped[action]), max(grouped[action]),
             len(grouped[action])) for action in order]


def radar_ranges(radar):
    grouped = defaultdict(list)
    for row in radar:
        grouped[row.get("action", "")].append(float(row["t_global"]))
    order = []
    for row in radar:
        action = row.get("action", "")
        if action not in order:
            order.append(action)
    return [(action, min(grouped[action]), max(grouped[action]),
             len(grouped[action])) for action in order if action]


def align_rule(session_id):
    radar_path = SESSION_DIR / f"session_{session_id}.csv"
    vidts_path = SESSION_DIR / f"session_{session_id}_vidts.csv"
    if not radar_path.exists() or not vidts_path.exists():
        raise FileNotFoundError(f"缺少规则文件：{radar_path.name} 或 {vidts_path.name}")

    radar = read_csv(radar_path)
    vidts = read_csv(vidts_path)
    # 旧的三次规则文件后面还带有 free_5min；5+3 汇总中的规则部分
    # 只取九个正式阶段，避免把自由尾段混进规则训练集。
    vidts = [row for row in vidts if row["action"] in RULE_ACTIONS]
    radar = [row for row in radar if row.get("action", "") in RULE_ACTIONS]
    vranges = action_ranges(vidts)
    rranges = radar_ranges(radar)
    vmap = {a: (s, e, n) for a, s, e, n in vranges}
    rmap = {a: (s, e, n) for a, s, e, n in rranges}
    video_base = vranges[0][1]

    # 第一阶段会把初始化/首个倒计时的积压雷达混进来，故只用后续
    # 阶段估计偏移。其余阶段的偏移在现有文件中很稳定（约 182 秒）。
    offsets = [rmap[a][0] - (vmap[a][0] - video_base)
               for a, _, _, _ in vranges[1:]
               if a in rmap]
    if len(offsets) < 3:
        raise ValueError(f"可用于估计偏移的阶段太少：{session_id}")
    offset = float(np.median(offsets))

    fields = ["t_global", "t_video_rel", "x", "y", "v", "em", "es",
              "raw_action", "aligned_action", "alignment_valid"]
    aligned_rows = []
    valid = 0
    for row in radar:
        t_global = float(row["t_global"])
        t_rel = t_global - offset
        aligned_action = ""
        for action, start, end, _ in vranges:
            # 视频日志的首尾是可观测阶段窗口；窗口之间的倒计时不进训练。
            if start - video_base <= t_rel <= end - video_base:
                aligned_action = action
                break
        is_valid = int(bool(aligned_action))
        valid += is_valid
        aligned_rows.append({
            "t_global": row["t_global"],
            "t_video_rel": f"{t_rel:.3f}",
            "x": row["x"], "y": row["y"], "v": row["v"],
            "em": row["em"], "es": row["es"],
            "raw_action": row.get("action", ""),
            "aligned_action": aligned_action,
            "alignment_valid": str(is_valid),
        })

    output = SESSION_DIR / f"session_{session_id}_aligned.csv"
    write_csv(output, fields, aligned_rows)
    expected = {a: n for a, _, _, n in vranges}
    aligned_counts = {a: sum(r["aligned_action"] == a for r in aligned_rows)
                      for a in expected}
    raw_counts = {a: sum(r.get("action", "") == a for r in radar)
                  for a in expected}
    regular_counts = [n for a, n in raw_counts.items()
                      if a not in ("still_30s", "natural_typing")]
    radar_baseline = float(np.median(regular_counts))
    meta = {
        "session_id": session_id,
        "kind": "rule",
        "source_radar": radar_path.name,
        "source_video_timestamps": vidts_path.name,
        "output": output.name,
        "method": "video_action_intervals + median_offset_after_first_action",
        "radar_to_video_offset_sec": offset,
        "offset_samples_sec": offsets,
        "n_radar_raw": len(radar),
        "n_alignment_valid": valid,
        "alignment_valid_fraction": valid / len(radar),
        "video_action_ranges_rel": {
            a: {"start": s - video_base, "end": e - video_base,
                "n_video": n} for a, s, e, n in vranges
        },
        "aligned_radar_counts": aligned_counts,
        "raw_action_counts": raw_counts,
        "expected_radar_baseline_for_20s_action": radar_baseline,
        "incomplete_actions": [a for a, n in raw_counts.items()
                                if n < radar_baseline * 0.90],
    }
    meta_path = SESSION_DIR / f"session_{session_id}_aligned_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    return meta


def align_free(session_id):
    clean_path = SESSION_DIR / f"session_{session_id}_free_clean.csv"
    labels_path = SESSION_DIR / f"session_{session_id}_free_labels.csv"
    if not clean_path.exists() or not labels_path.exists():
        raise FileNotFoundError(f"缺少自由 clean/labels 文件：{session_id}")
    clean = read_csv(clean_path)
    teacher = read_csv(labels_path)
    radar_t = np.array([float(r["t_global"]) for r in clean])
    teacher_t = np.array([float(r["t_global"]) for r in teacher])
    if radar_t[-1] <= radar_t[0] or teacher_t[-1] <= teacher_t[0]:
        raise ValueError(f"自由 session 时间轴无效：{session_id}")
    t_teacher = ((radar_t - radar_t[0]) / (radar_t[-1] - radar_t[0])
                 * (teacher_t[-1] - teacher_t[0]) + teacher_t[0])

    fields = ["t_global", "t_teacher", "x", "y", "v", "em", "es",
              "label", "radar_valid", "alignment_valid"]
    rows = []
    for row, t in zip(clean, t_teacher):
        rows.append({
            "t_global": row["t_global"],
            "t_teacher": f"{t:.3f}",
            "x": row["x"], "y": row["y"], "v": row["v"],
            "em": row["em"], "es": row["es"],
            "label": row["label"],
            "radar_valid": row["radar_valid"],
            "alignment_valid": "1",
        })
    output = SESSION_DIR / f"session_{session_id}_free_aligned.csv"
    write_csv(output, fields, rows)
    meta = {
        "session_id": session_id,
        "kind": "free",
        "source_clean": clean_path.name,
        "source_teacher_labels": labels_path.name,
        "output": output.name,
        "method": "teacher_timeline_endpoint_affine_from_clean",
        "n_radar": len(clean),
        "radar_invalid_rows": sum(r["radar_valid"] == "0" for r in rows),
        "label_counts": {str(i): sum(r["label"] == str(i) for r in rows)
                         for i in range(5)},
    }
    meta_path = SESSION_DIR / f"session_{session_id}_free_aligned_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    return meta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rule", action="append", dest="rules")
    parser.add_argument("--free", action="append", dest="free")
    args = parser.parse_args()
    rules = args.rules or RULE_SESSIONS
    free = args.free or FREE_SESSIONS
    for sid in rules:
        meta = align_rule(sid)
        print(f"rule {sid}: valid={meta['n_alignment_valid']}/{meta['n_radar_raw']} "
              f"({meta['alignment_valid_fraction']:.1%}), offset="
              f"{meta['radar_to_video_offset_sec']:.3f}s, "
              f"incomplete={meta['incomplete_actions']}")
    for sid in free:
        meta = align_free(sid)
        print(f"free {sid}: radar={meta['n_radar']}, "
              f"invalid={meta['radar_invalid_rows']}, labels={meta['label_counts']}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""对齐 sessions_v2：使用同一台主机的 host_ts，不再用首尾缩放偷对齐。

V2 采集器为每个雷达样本保存 host_ts，为每个视频帧保存 t_wallclock。
两者在同一主机时钟上，故规则动作直接按时间区间标注；C3 c3_ms 保留
用于检查重启、回绕和串口时钟漂移，不作为唯一对齐轴。
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

from v2_io import quality_gate, quality_summary


SESSION_DIR = Path(__file__).resolve().parent / "sessions_v2"
V2_ACTIONS = [
    "empty_table", "still_30s", "natural_typing", "slow_sweep",
    "quick_points", "ellipse", "fwd_back", "head_only", "turn_toward",
    "big_sway", "still_near", "still_far", "natural_reach", "stand_sit",
]


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, fields, rows):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def ranges(vidts):
    grouped = defaultdict(list)
    order = []
    for row in vidts:
        action = row.get("action", "")
        if action not in order:
            order.append(action)
        try:
            grouped[action].append(float(row["t_wallclock"]))
        except (KeyError, ValueError):
            pass
    return [(a, min(grouped[a]), max(grouped[a]), len(grouped[a]))
            for a in order if grouped[a]]


def align_session(session_id: str):
    radar_path = SESSION_DIR / f"session_{session_id}.csv"
    vidts_path = SESSION_DIR / f"session_{session_id}_vidts.csv"
    summary_path = SESSION_DIR / f"session_{session_id}_summary.json"
    if not radar_path.exists() or not vidts_path.exists():
        raise FileNotFoundError(f"缺少 {radar_path.name} 或 {vidts_path.name}")

    radar = read_csv(radar_path)
    vidts = read_csv(vidts_path)
    vranges = ranges(vidts)
    if not vranges:
        raise ValueError(f"没有有效视频时间区间：{session_id}")

    fields = [
        "t_global", "host_ts", "c3_ms",
        "x", "y", "v", "x2", "y2", "v2", "x3", "y3", "v3",
        "em", "es", "d2410", "pres", "ir",
        "raw_action", "aligned_action", "alignment_valid", "radar_valid",
    ]
    rows = []
    for row in radar:
        try:
            host_ts = float(row["host_ts"])
        except (KeyError, ValueError):
            continue
        action = ""
        for candidate, start, end, _ in vranges:
            if start <= host_ts <= end:
                action = candidate
                break
        valid = int(bool(action))
        out = {key: row.get(key, "0") for key in fields}
        out.update({
            "raw_action": row.get("action", ""),
            "aligned_action": action,
            "alignment_valid": str(valid),
            # 保留原始雷达，V2 初版只在明显断线/非法行时屏蔽。
            "radar_valid": str(valid),
        })
        rows.append(out)

    output = SESSION_DIR / f"session_{session_id}_aligned.csv"
    write_csv(output, fields, rows)
    quality_rows = [r for r in radar if r.get("host_ts")]
    quality = quality_summary(quality_rows)
    meta = {
        "session_id": session_id,
        "kind": "v2",
        "source_radar": radar_path.name,
        "source_video_timestamps": vidts_path.name,
        "output": output.name,
        "method": "same-host host_ts interval labeling",
        "n_radar_raw": len(radar),
        "n_aligned_rows": len(rows),
        "alignment_valid_fraction": (
            sum(r["alignment_valid"] == "1" for r in rows) / len(rows)
            if rows else 0.0
        ),
        "action_counts": {
            a: sum(r["aligned_action"] == a for r in rows)
            for a, *_ in vranges
        },
        "video_action_ranges": {
            a: {"start": s, "end": e, "n_video": n}
            for a, s, e, n in vranges
        },
        "quality": quality,
        "quality_gate": quality_gate(quality),
    }
    if summary_path.exists():
        source_meta = json.loads(summary_path.read_text())
        meta["fw_version"] = source_meta.get("fw_version")
        meta["orientation"] = source_meta.get("orientation")
        meta["geometry"] = source_meta.get("geometry")
    meta_path = SESSION_DIR / f"session_{session_id}_aligned_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    return meta


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", action="append", dest="sessions",
                        help="session id，不带 session_；可重复")
    args = parser.parse_args()
    sessions = args.sessions
    if not sessions:
        sessions = [p.name[len("session_"):-len(".csv")]
                    for p in sorted(SESSION_DIR.glob("session_*.csv"))
                    if not p.name.endswith("_aligned.csv")]
    for sid in sessions:
        meta = align_session(sid)
        print(f"{sid}: aligned={meta['n_aligned_rows']}, "
              f"valid={meta['alignment_valid_fraction']:.1%}, "
              f"gate={'PASS' if meta['quality_gate']['pass'] else 'FAIL'}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""V1/V2 C3 串口格式与采集质量工具。

V2 固件每行包含 LD2450 的 3 个目标；本模块负责：
  - 解析 V1/V2 RADAR 行，避免字段位置错读；
  - 保留 C3 millis 与主机接收时间；
  - 对一场采集做最小质量门检查。

原始数据永远保留，质量门只生成报告，不删除数据。
"""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, Optional


V2_RADAR_FIELDS = [
    "ms", "x", "y", "v", "x2", "y2", "v2", "x3", "y3", "v3",
    "em", "es", "d2410", "pres", "ir",
]

V1_RADAR_FIELDS = ["ms", "x", "y", "v", "em", "es", "d2410", "pres", "ir"]


def _ints(values: Iterable[str]) -> list[int]:
    return [int(float(v)) for v in values]


def parse_radar_line(line: str, host_ts: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """解析一行 RADAR。

    返回统一字典：V1 没有的 x2/y2/v2、x3/y3/v3 置零；
    ``radar_format`` 明确记录原始格式。
    """
    if not line.startswith("RADAR,"):
        return None
    parts = line.strip().split(",")
    values = parts[1:]
    if len(values) >= len(V2_RADAR_FIELDS):
        raw = _ints(values[:len(V2_RADAR_FIELDS)])
        data = dict(zip(V2_RADAR_FIELDS, raw))
        data["radar_format"] = "v2"
    elif len(values) >= len(V1_RADAR_FIELDS):
        raw = _ints(values[:len(V1_RADAR_FIELDS)])
        data = dict(zip(V1_RADAR_FIELDS, raw))
        for key in ("x2", "y2", "v2", "x3", "y3", "v3"):
            data[key] = 0
        data["radar_format"] = "v1"
    else:
        return None

    data["c3_ms"] = data.pop("ms")
    data["host_ts"] = float(host_ts) if host_ts is not None else None
    return data


def parse_control_line(line: str) -> Optional[Dict[str, Any]]:
    """解析 !FW_VERSION / !SYNC 等控制行。"""
    line = line.strip()
    if line.startswith("!FW_VERSION,"):
        return {"type": "fw_version", "version": line.split(",", 1)[1]}
    if line.startswith("!SYNC,"):
        try:
            return {"type": "sync", "c3_ms": int(line.split(",", 1)[1])}
        except ValueError:
            return None
    if line.startswith("!START,"):
        try:
            return {"type": "start", "c3_ms": int(line.split(",", 1)[1])}
        except ValueError:
            return None
    return None


def _float(row: Dict[str, Any], key: str, default: float = 0.0) -> float:
    try:
        value = float(row.get(key, default))
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def quality_summary(rows: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """计算不依赖标签的 V2 快速质量指标。"""
    rows = list(rows)
    times = sorted(
        _float(r, "host_ts") for r in rows if r.get("host_ts") is not None
    )
    diffs = [b - a for a, b in zip(times, times[1:]) if b > a]
    duration = times[-1] - times[0] if len(times) >= 2 else 0.0
    em_active = sum(_float(r, "em") > 0 for r in rows)
    target2 = sum(abs(_float(r, "x2")) > 0 or _float(r, "y2") > 0 for r in rows)
    target3 = sum(abs(_float(r, "x3")) > 0 or _float(r, "y3") > 0 for r in rows)
    return {
        "n_rows": len(rows),
        "duration_sec": duration,
        "median_dt_sec": float(sorted(diffs)[len(diffs) // 2]) if diffs else None,
        "frame_rate_hz": (len(diffs) / duration) if duration > 0 else 0.0,
        "max_gap_sec": max(diffs) if diffs else None,
        "em_active_fraction": em_active / len(rows) if rows else 0.0,
        "target2_nonzero_fraction": target2 / len(rows) if rows else 0.0,
        "target3_nonzero_fraction": target3 / len(rows) if rows else 0.0,
    }


def quality_gate(summary: Dict[str, Any], *, min_hz: float = 14.0,
                 max_gap_sec: float = 1.0,
                 min_em_fraction: float = 0.10) -> Dict[str, Any]:
    """返回 PASS/FAIL 与逐项原因；不会修改任何采集文件。"""
    checks = {
        "frame_rate": summary.get("frame_rate_hz", 0.0) >= min_hz,
        "max_gap": (summary.get("max_gap_sec") is not None and
                     summary["max_gap_sec"] <= max_gap_sec),
        "em_activity": summary.get("em_active_fraction", 0.0) >= min_em_fraction,
    }
    return {"pass": all(checks.values()), "checks": checks}

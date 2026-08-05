#!/usr/bin/env python3
"""
HappyMac — 特征工程 + 标签生成

从 collect.py 产出的原始 CSV 中：
  1. 读取原始数据
  2. 滑动窗口切分
  3. 每个窗口提取 18 个特征（config.FEATURE_NAMES）
  4. 生成模型 A 标签（6 类）和模型 B 标签（8 类）
  5. 输出 X_train.npy / y_a_train.npy / y_b_train.npy

用法：
  python preprocess.py                          # 处理所有 sessions
  python preprocess.py --session 2026-08-05_14-32.csv  # 单文件
  python preprocess.py --vis                    # 可视化特征分布
"""

import argparse
import csv
import re
from collections import deque
from pathlib import Path

import numpy as np

from config import *


# ============================================================
#  数据加载
# ============================================================

def load_session(path: Path) -> dict:
    """加载一个采集会话的 CSV，返回 numpy 数组字典"""
    data = {
        "ts": [], "x": [], "y": [], "v": [], "em": [], "es": [],
        "d2410": [], "pres": [], "ir": [],
        "head_x": [], "head_y": [], "head_z": [],
        "head_yaw": [], "head_pitch": [], "head_roll": [],
        "left_eye": [], "right_eye": [], "mouth": [],
    }
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                data["ts"].append(float(row.get("ts_video", 0)))
                data["x"].append(float(row["x"]))
                data["y"].append(float(row["y"]))
                data["v"].append(float(row["v"]))
                data["em"].append(float(row["em"]))
                data["es"].append(float(row["es"]))
                data["d2410"].append(float(row["d2410"]))
                data["pres"].append(float(row["pres"]))
                data["ir"].append(float(row["ir"]))
                data["head_x"].append(float(row["head_x"]))
                data["head_y"].append(float(row["head_y"]))
                data["head_z"].append(float(row["head_z"]))
                data["head_yaw"].append(float(row["head_yaw"]))
                data["head_pitch"].append(float(row["head_pitch"]))
                data["head_roll"].append(float(row["head_roll"]))
                data["left_eye"].append(float(row["left_eye_open"]))
                data["right_eye"].append(float(row["right_eye_open"]))
                data["mouth"].append(float(row["mouth_open"]))
            except (ValueError, KeyError):
                continue
    return {k: np.array(v) for k, v in data.items()}


# ============================================================
#  特征提取（单窗口）
# ============================================================

def extract_features(x_arr, y_arr, v_arr, em_arr, es_arr, ir_arr) -> np.ndarray:
    """
    从一个窗口的雷达数据中提取 18 个特征。
    返回 shape=(len(FEATURE_NAMES),) 的 float32 数组。
    """
    n = len(x_arr)
    if n < 3:
        return np.full(len(FEATURE_NAMES), np.nan, dtype=np.float32)

    feats = []

    # —— LD2450 位置 ——
    feats.append(np.mean(x_arr))                          # x_mean
    feats.append(np.std(x_arr))                           # x_std
    feats.append(np.polyfit(np.arange(n), x_arr, 1)[0]    # x_slope
                 if n >= 2 else 0)
    feats.append(np.mean(y_arr))                          # y_mean
    feats.append(np.std(y_arr))                           # y_std
    feats.append(np.polyfit(np.arange(n), y_arr, 1)[0]    # y_slope
                 if n >= 2 else 0)

    # —— LD2450 速度 ——
    feats.append(np.mean(np.abs(v_arr)))                  # v_mean_abs
    feats.append(np.max(np.abs(v_arr)))                   # v_max_abs
    feats.append(np.mean(np.abs(v_arr) > 10))             # v_frac_gt_10

    # —— LD2410C 能量 ——
    feats.append(np.mean(em_arr))                         # em_mean
    feats.append(np.std(em_arr))                          # em_std
    feats.append(np.max(em_arr))                          # em_max
    feats.append(np.mean(es_arr))                         # es_mean

    # —— 组合特征 ——
    # XY 相关性
    if n >= 3 and np.std(x_arr) > 0 and np.std(y_arr) > 0:
        feats.append(np.corrcoef(x_arr, y_arr)[0, 1])
    else:
        feats.append(0.0)                                 # xy_corr

    # X 趋势 × 能量（能量确认的移动）
    x_slope = feats[2]
    feats.append(x_slope * np.mean(em_arr))               # x_slope_x_em

    # 单位距离反射能量（朝向 proxy）
    y_mean = feats[3]
    feats.append(np.mean(es_arr) / max(y_mean, 1.0))      # es_div_y_mean

    # 能量尖峰
    if n >= 3:
        em_sorted = np.sort(em_arr)[::-1]
        baseline = np.median(em_arr)
        feats.append(em_sorted[0] - baseline)             # em_spike_amplitude
        # 尖峰持续时间：连续 > baseline*2 的帧数
        spike_mask = em_arr > baseline * 2
        feats.append(np.sum(spike_mask))                  # em_spike_duration
    else:
        feats.append(0.0)
        feats.append(0.0)

    # 最大 Y 帧间跳变
    if n >= 2:
        feats.append(np.max(np.abs(np.diff(y_arr))))      # delta_y_max_abs
    else:
        feats.append(0.0)

    # 红外
    feats.append(np.mean(ir_arr))                         # ir_present_mean

    return np.array(feats, dtype=np.float32)


# ============================================================
#  标签生成
# ============================================================

def label_a(head_data: dict, start: int, end: int) -> int:
    """
    模型 A：位置动作分类（6 类）
    取窗口内 head 数据的 中位数 判定。
    """
    hx = np.median(head_data["head_x"][start:end])
    hz = np.median(head_data["head_z"][start:end])
    # hy = np.median(head_data["head_y"][start:end])
    pres = np.mean(head_data["head_z"][start:end] > 0.01)  # 有检测的比例

    if pres < 0.3:
        return 0  # ABSENT

    if hz < 0.6:
        return 3  # LEAN_FWD

    if hz > 1.2:
        return 4  # LEAN_BACK

    if hx < -0.25:
        return 1  # LEFT
    elif hx > 0.25:
        return 5  # RIGHT
    else:
        return 2  # CENTER


def label_b(head_data, start, end) -> int:
    """
    模型 B：状态分类（8 类）
    5 秒窗口，统计 + 规则判定。
    """
    hx = head_data["head_x"][start:end]
    hy = head_data["head_y"][start:end]
    hz = head_data["head_z"][start:end]
    yaw = head_data["head_yaw"][start:end]

    n = len(hx)
    if n < 3:
        return 0  # ABSENT

    pres = np.mean(hz > 0.01)
    if pres < 0.3:
        # 看一下前后有没有数据（离场检测）
        return 0   # ABSENT

    # 头部移动量
    head_motion = np.std(hx) + np.std(hy)

    # 注视屏幕的近似：|yaw| < 15deg 且 pitch 不大
    gaze_on_screen = np.mean(np.abs(yaw) < 15)

    # 头部 z 变化趋势（前 3 帧 vs 后 3 帧）
    if n >= 6:
        dz = np.median(hz[-3:]) - np.median(hz[:3])
    else:
        dz = 0

    # 判定

    # 到场：头部从无到有 + z 大幅下降
    if dz < -0.4:
        return 1  # ARRIVING

    # 离场：头部 z 大幅上升
    if dz > 0.4:
        return 2  # LEAVING

    # 突然看它：yaw 在窗口内出现 >15° 的快速变化
    if n >= 5:
        yaw_diff = np.abs(np.diff(yaw))
        if np.max(yaw_diff) > 15:
            # 判断方向
            # yaw 从负变正或从正变负 → 转头动作
            yaw_start = np.median(yaw[:max(1, n//3)])
            yaw_end = np.median(yaw[-(n//3):])
            if abs(yaw_end - yaw_start) > 15:
                if abs(yaw_end) < abs(yaw_start):
                    return 6  # LOOK_AT_IT（转头朝向正面=朝向HappyMac所在位置）
                else:
                    return 7  # LOOK_AWAY

    # 专注：静止 + 注视屏幕
    if head_motion < 3 and gaze_on_screen > 0.8:
        return 3  # FOCUSED

    # 活跃
    if head_motion > 8:
        return 4  # ACTIVE

    # 其余 = 发呆/空闲
    if head_motion < 3 and gaze_on_screen < 0.5:
        return 5  # IDLE

    # 默认
    return 4  # ACTIVE（若上面都没命中）


# ============================================================
#  主逻辑
# ============================================================

def process_session(path: Path) -> tuple:
    """处理一个 session，返回 (X, y_a, y_b)"""
    data = load_session(path)
    n_total = len(data["ts"])
    if n_total < 50:
        print(f"  ⚠️  数据太少 ({n_total} 行)，跳过")
        return None, None, None

    window_a_frames = int(WINDOW_A_SEC * RADAR_HZ)  # 20
    window_b_frames = int(WINDOW_B_SEC * RADAR_HZ)  # 50
    stride_frames   = int(STRIDE_SEC * RADAR_HZ)    # 2

    X_list, ya_list, yb_list = [], [], []

    for start in range(0, n_total - window_b_frames, stride_frames):
        end_a = start + window_a_frames
        end_b = start + window_b_frames

        # —— 模型 A（2s 窗口）——
        feats_a = extract_features(
            data["x"][start:end_a],
            data["y"][start:end_a],
            data["v"][start:end_a],
            data["em"][start:end_a],
            data["es"][start:end_a],
            data["ir"][start:end_a],
        )
        if not np.any(np.isnan(feats_a)):
            # 特征用于模型 A
            pass  # 和 B 共用特征，但窗口不同。这里统一用 B 窗口特征 + A 特征拼接
            # 简化处理：模型 A 也用 B 窗口的特征（B 窗口更长，包含更多上下文）

        # —— 模型 B（5s 窗口）——
        feats_b = extract_features(
            data["x"][start:end_b],
            data["y"][start:end_b],
            data["v"][start:end_b],
            data["em"][start:end_b],
            data["es"][start:end_b],
            data["ir"][start:end_b],
        )
        if np.any(np.isnan(feats_b)):
            continue

        # —— 特征拼接 ——
        # 模型 A 用 2s 窗口特征，模型 B 用 5s 窗口特征
        # 统一输出：A 特征 + B 特征 → 同一个样本
        feats_a = extract_features(
            data["x"][start:end_a],
            data["y"][start:end_a],
            data["v"][start:end_a],
            data["em"][start:end_a],
            data["es"][start:end_a],
            data["ir"][start:end_a],
        )
        if np.any(np.isnan(feats_a)):
            continue

        # 拼接：先 A 特征，再 B 特征
        all_feats = np.concatenate([feats_a, feats_b])

        # 标签
        head_data = {
            "head_x": data["head_x"], "head_y": data["head_y"],
            "head_z": data["head_z"],
            "head_yaw": data["head_yaw"],
            "head_pitch": data["head_pitch"],
            "head_roll": data["head_roll"],
        }
        la = label_a(head_data, start, end_a)
        lb = label_b(head_data, start, end_b)

        X_list.append(all_feats)
        ya_list.append(la)
        yb_list.append(lb)

    if not X_list:
        return None, None, None

    X   = np.stack(X_list)
    y_a = np.array(ya_list)
    y_b = np.array(yb_list)

    return X, y_a, y_b


def main():
    parser = argparse.ArgumentParser(description="HappyMac 特征工程")
    parser.add_argument("--session", type=str, default=None,
                        help="处理单个 session 文件")
    parser.add_argument("--vis", action="store_true",
                        help="可视化特征分布")
    args = parser.parse_args()

    if args.session:
        paths = [Path(args.session)]
    else:
        paths = sorted(DATA_DIR.glob("*.csv"))

    if not paths:
        print("❌ 没有找到 session 文件。请先运行 collect.py")
        return

    all_X, all_ya, all_yb = [], [], []

    for p in paths:
        print(f"[preprocess] {p.name}")
        X, ya, yb = process_session(p)
        if X is not None:
            all_X.append(X)
            all_ya.append(ya)
            all_yb.append(yb)
            print(f"  → {len(X)} 个窗口, "
                  f"A: {np.bincount(ya, minlength=6)}, "
                  f"B: {np.bincount(yb, minlength=8)}")

    if not all_X:
        print("❌ 没有有效数据")
        return

    X_all   = np.concatenate(all_X)
    ya_all  = np.concatenate(all_ya)
    yb_all  = np.concatenate(all_yb)

    MODEL_DIR.mkdir(parents=True, exist_ok=True)

    # 模型 A（只用前 18 个特征 = 2s 窗口特征）
    np.save(MODEL_DIR / "X_a.npy", X_all[:, :len(FEATURE_NAMES)])
    np.save(MODEL_DIR / "y_a.npy", ya_all)

    # 模型 B（用全部 36 个特征 = 2s + 5s 特征拼接）
    np.save(MODEL_DIR / "X_b.npy", X_all)
    np.save(MODEL_DIR / "y_b.npy", yb_all)

    # 特征名文件
    feat_names_a = FEATURE_NAMES
    feat_names_b = [f"a_{n}" for n in FEATURE_NAMES] + \
                   [f"b_{n}" for n in FEATURE_NAMES]
    with open(MODEL_DIR / "feature_names.txt", "w") as f:
        f.write("Model A (18 features, {} windows, {} samples):\n".format(
            WINDOW_A_SEC, len(ya_all)))
        f.write("  " + ", ".join(feat_names_a) + "\n\n")
        f.write("Model B (36 features, {} windows, {} samples):\n".format(
            WINDOW_B_SEC, len(yb_all)))
        f.write("  " + ", ".join(feat_names_b) + "\n")

    print(f"\n✅ 完成: {len(ya_all)} 个样本")
    print(f"   模型 A（{len(LABEL_A_CLASSES)} 类）: "
          f"{dict(zip(LABEL_A_CLASSES, np.bincount(ya_all, minlength=6)))}")
    print(f"   模型 B（{len(LABEL_B_CLASSES)} 类）: "
          f"{dict(zip(LABEL_B_CLASSES, np.bincount(yb_all, minlength=8)))}")
    print(f"   输出: {MODEL_DIR}/")

    if args.vis:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 6, figsize=(18, 10))
        for i, ax in enumerate(axes.flat):
            if i < len(FEATURE_NAMES):
                ax.hist(X_all[:, i], bins=30, alpha=0.7)
                ax.set_title(FEATURE_NAMES[i], fontsize=8)
            else:
                ax.axis("off")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()

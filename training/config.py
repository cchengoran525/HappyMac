"""
HappyMac TinyML — 全局配置

修改此文件即可调整整个训练管线的参数，
无需改 collect.py / preprocess.py / train.py。
"""

from pathlib import Path

# ─── 路径 ───────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR     = PROJECT_ROOT / "training" / "sessions"
MODEL_DIR    = PROJECT_ROOT / "training" / "models"

# ─── 雷达串口 ───────────────────────────────────────
RADAR_PORT    = "/dev/cu.usbmodem2101"
RADAR_BAUD    = 115200
RADAR_CSV_FMT = "RADAR,{ms},{x},{y},{v},{em},{es},{d},{pres},{ir}"

# ─── S3 摄像头 ──────────────────────────────────────
# ESP32-CAM 默认 MJPEG 推流地址
CAMERA_URL = "http://192.168.4.1:81/stream"
# 如果 MJPEG 不稳定可改用单帧模式:
# CAMERA_URL = "http://192.168.4.1/capture"

# ─── 时间对齐 ───────────────────────────────────────
# 视频帧与最近雷达帧的最大允许时间差（秒）
MAX_TIME_DELTA = 0.075  # 75ms

# ─── MediaPipe ──────────────────────────────────────
# Face Landmarker 模型（自动下载到本目录）
MP_MODEL_PATH = str(PROJECT_ROOT / "training" / "face_landmarker_v2_with_blendshapes.task")
# 检测置信度
MP_FACE_DETECT_CONFIDENCE = 0.5
MP_FACE_TRACK_CONFIDENCE  = 0.5

# ─── 标签定义 ───────────────────────────────────────
# 模型 A：位置动作（2 秒窗口，取中位数判定）
LABEL_A_DEFS = {
    "ABSENT":      "head_z 无检测",
    "LEFT":        "head_x < -0.25 (头偏左 >25cm)",
    "CENTER":      "abs(head_x) < 0.25",
    "RIGHT":       "head_x > 0.25",
    "LEAN_FWD":    "head_z < 0.6 (头凑近 <60cm)",
    "LEAN_BACK":   "head_z > 1.2 (头后靠 >1.2m)",
}
LABEL_A_CLASSES = list(LABEL_A_DEFS.keys())  # 6 类

# 模型 B：用户状态（5 秒窗口，统计判定）
LABEL_B_DEFS = {
    "ABSENT":      "无头部检测",
    "ARRIVING":    "head_z 5秒内下降 >40cm",
    "LEAVING":     "head_z 5秒内上升 >40cm 或 无检测",
    "FOCUSED":     "5秒窗口头部移动 std <3cm，且注视屏幕 >80%时间",
    "ACTIVE":      "5秒窗口头部移动 std >8cm",
    "IDLE":        "5秒窗口头部移动 std <3cm，但注视屏幕 <50%时间",
    "LOOK_AT_IT":  "头部突然转 >15° 朝向 HappyMac（yaw 变化 >15° + 300-800ms内完成）",
    "LOOK_AWAY":   "头部转离 HappyMac（yaw 变化反向 >15°）",
}
LABEL_B_CLASSES = list(LABEL_B_DEFS.keys())  # 8 类

# ─── 窗口参数 ───────────────────────────────────────
WINDOW_A_SEC = 2.0    # 模型 A 窗口
WINDOW_B_SEC = 5.0    # 模型 B 窗口
STRIDE_SEC   = 0.2    # 滑动步长
RADAR_HZ     = 10     # 雷达采样率（用于窗口帧数计算）

# ─── 特征列表 ───────────────────────────────────────
# 这些特征从每个窗口提取，顺序固定（模型依赖此顺序）
FEATURE_NAMES = [
    # LD2450 位置（6）———————————————————————————————
    "x_mean", "x_std", "x_slope",
    "y_mean", "y_std", "y_slope",
    # LD2450 速度（3）———————————————————————————————
    "v_mean_abs", "v_max_abs", "v_frac_gt_10",
    # LD2410C 能量（4）———————————————————————————————
    "em_mean", "em_std", "em_max",
    "es_mean",
    # 组合特征（6）————————————————————————————————————
    "xy_corr",
    "x_slope_x_em",         # X趋势 × 能量（能量确认的移动）
    "es_div_y_mean",         # 单位距离反射能量（朝向 proxy）
    "em_spike_amplitude",    # 窗口内能量尖峰幅度
    "em_spike_duration",     # 能量尖峰持续帧数
    "delta_y_max_abs",       # 最大帧间 Y 跳变（到场/离场 proxy）
    # 红外（1）———————————————————————————————————————
    "ir_present_mean",
]

# ─── 训练参数 ───────────────────────────────────────
RF_N_ESTIMATORS  = 80
RF_MAX_DEPTH     = 10
MLP_HIDDEN_1     = 32
MLP_HIDDEN_2     = 16
MLP_DROPOUT      = 0.2
TRAIN_EPOCHS     = 100
BATCH_SIZE       = 32
LEARNING_RATE    = 0.001
VAL_SPLIT        = 0.15
TEST_SPLIT       = 0.15

# ─── 量化 ──────────────────────────────────────────
QUANT_SAMPLES = 500  # 代表性数据集的样本数（用于 INT8 校准）

# ─── 数据增强（训练时）───────────────────────────────
AUGMENT_NOISE_STD = 0.02   # 给特征加高斯噪声（std=2%）
AUGMENT_SHIFT_MAX = 0.05   # 给特征加随机偏移（max=5%）

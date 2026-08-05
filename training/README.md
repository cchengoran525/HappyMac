# HappyMac — TinyML 训练管线

## 问题定义

**跨模态蒸馏（Cross-modal Teacher-Student Distillation）**

- **Teacher（训练期）**：S3 摄像头 + MediaPipe Face Mesh + MediaPipe Pose → 视觉关键点 → 注意力状态标签
- **Student（部署期）**：LD2450 + LD2410C 双 24GHz 雷达 → 18 维时序统计特征 → MLP → 相同语义的行为状态
- **目标硬件**：ESP32-C3 SuperMini（RISC-V 160MHz, 400KB SRAM, 4MB Flash）

训练完成后，摄像头被永久移除。C3 仅凭两个雷达信号进行推理。

## 学术背景

### 与现有工作的关系

| 工作 | 传感器 | 模型 | 与我们的关系 |
|------|--------|------|-------------|
| [dronefreak/human-action-classification](https://github.com/dronefreak/human-action-classification) | 摄像头 + MediaPipe Pose (33 关键点) | Keypoint MLP / CNN / 3D CNN | **借用**：关键点归一化方法、两层标签架构、Keypoint MLP 模式。**不借用**：UCF-101 分类、ResNet50、3D CNN |
| CVPR 2024, *Bootstrapping Radars* | 77GHz 4D 车载雷达 + 摄像头 | CNN + 对比学习 | 跨模态蒸馏范式的参考，但硬件层级和输入维度完全不同 |
| DOAJ 2024, *mmWave-triggered face verification* | 24GHz 雷达 + ESP32-S3 | INT8 MobileFaceNet | 硬件栈最接近的工作（ESP32 + 24GHz + INT8），但做的是人脸识别而非行为分类 |
| ESP-NN 官方基准 | ESP32-C3/S3 | MobileNetV1 | C3 推理性能参考：MLP <2K 参数 <10ms |

### 本项目的学术空白

**24GHz 极低成本双雷达（<50 元）+ TinyML 跨模态蒸馏（视觉→雷达）+ 桌面注意力行为分类**——此组合在文献和开源社区中均无直接对标的先例。

- LD2410/LD2450 DIY 社区项目均使用纯阈值规则，无 ML
- 60GHz+ 雷达的 ML 研究使用高维热力图输入（CNN/Transformer），不与本项目的 18 维标量统计特征可比
- 桌面注意力行为分类（专注/发呆/注意力转移）本身是 HCI 领域关注的问题，但现有方案均依赖摄像头或可穿戴设备

### 已知限制（已写入主 README）

- 双雷达 24GHz 同频干扰：无休眠指令，软件分时不可行，需硬件 MOSFET 门控
- LD2450 X 轴精度正比于距离（<0.5m 基本为噪声）
- Teacher 标签来自 MediaPipe 几何推导，非人工标注，存在系统性偏差
- ESP32-C3 无 SIMD 向量扩展（相比 S3 的 ESP-NN 加速 42x），仅能运行小型 MLP

## 架构

### 数据流

```
[采集]  collect.py
  S3 MJPEG 视频流 ──→ MediaPipe Face Mesh (468关键点) ──→ 头部位姿、眼部开合、注视方向
         │            MediaPipe Pose     (33关键点) ──→ 手腕位置、躯干角度、手部速度
         │
  C3 USB 串口     ──→ LD2450 (X/Y/V) + LD2410C (Em/Es) + SR602 ──→ 时间对齐 → CSV

[处理]  preprocess.py
  原始 CSV → 滑动窗口 (模型A: 2s, 模型B: 5s) → 特征提取 → 标签生成 → .npy

[训练]  train.py
  RF 快速验证 (准确率 ≥85%? → 特征有效性闸门) → MLP (32→16→softmax) → INT8 量化 → model_*.h
```

### 两层标签架构（借自 human-action-classification）

```
PositionClassifier（粗粒度，纯关键点几何规则）
  ├── ABSENT       (无检测)
  ├── LEFT         (head_x < -0.25)
  ├── CENTER       (|head_x| < 0.25)
  ├── RIGHT        (head_x > 0.25)
  ├── LEAN_FWD     (head_z < 0.6)
  └── LEAN_BACK    (head_z > 1.2)

StateClassifier（细粒度，关键点时序特征 × 规则混合）
  ├── ABSENT       (无检测)
  ├── ARRIVING     (到场：head_z 骤降 + 手部出现)
  ├── LEAVING      (离场：head_z 骤升 + 手部消失)
  ├── FOCUSED      (专注：头部微动低 + 手部活跃 + 注视屏幕)
  ├── ACTIVE       (活跃：头部微动高)
  ├── IDLE         (发呆：头部微动低 + 手部停止 + 注视漂移)
  ├── ATTN_SHIFT   (注意力转移：状态短时切换，非持续状态)
  └── LOOK_AWAY    (注意力回到屏幕)
```

### 特征设计（18 维）

详见 `config.py`。核心设计原则：

```
LD2450 位置 (6):  mean/std/slope of X/Y          ← 空间位置 + 趋势
LD2450 速度 (3):  mean/max/fraction of |V|       ← 运动强度
LD2410C 能量 (4): mean/std/max of Em, mean of Es ← 微动 + 反射截面
组合特征 (6):     xy_corr, es/y_ratio, 能量尖峰 ← 传感器融合 + 距离归一化
红外 (1):         ir_present_mean                ← 二值存在性确认
```

关键创新特征：`es_div_y_mean`（单位距离反射能量）——理论依据为雷达方程 Pr ∝ σ/R⁴，除以 Y 后近似消去距离衰减，仅留目标雷达截面 σ 的贡献。可用于间接感知目标朝向变化（正脸 vs 侧脸 RCS 差 3-6dB）。**此特征在 24GHz 雷达上的有效性尚未被实验验证。**（关口 2, 预估成功率 ~45%）

### 状态检测的生理依据（注意力转移）

不直接测量"头转了多少度"，而是检测注意力转移的全身生理伴随反应：

```
注意力转移过程中：
  1. 手部微动停止     → Em 在 0.5-1s 内骤降（打字/鼠标停止）
  2. 身体姿态微调     → X/Y 微动 + Em 小尖峰（身体往目标方向偏转）
  3. 心率变异性变化   → Es 的微振动频率偏移（副交感→交感切换）
                        ⚠️ 24GHz 雷达能否感知此信号未经验证
  4. 呼吸模式变化     → Es 在 3-5s 窗口上的低频成分变化
```

这些信号在 18 维特征空间中形成了一个与其他状态不重叠的 cluster——这是 ML 能够分类的前提，也是本项目最核心的待验证假设。


## 文件说明

```
training/
├── README.md           ← 本文件
├── config.py           全局配置（修改参数无需改其他文件）
├── collect.py          数据采集主脚本
├── preprocess.py       特征工程 + 标签生成
├── train.py            RF 验证 → MLP 训练 → INT8 量化 → C 头文件
├── requirements.txt    Python 依赖
├── .gitignore          排除 __pycache__、模型文件
├── sessions/           原始采集 CSV（.gitignore）
├── models/             训练产物（.npy / .h5 / .tflite / .h）
└── face_landmarker_v2_with_blendshapes.task  # 首次运行时自动下载
```


## 用法

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 采集数据（需要 S3 摄像头 + C3 雷达同时连接）
python collect.py --duration 600          # 采集 10 分钟
python collect.py --duration 1800 --no-display  # 30 分钟，无预览窗

# 3. 特征工程
python preprocess.py                      # 处理所有 sessions
python preprocess.py --session sessions/2026-08-05_14-32.csv  # 单文件
python preprocess.py --vis                # 可视化特征分布

# 4. 训练
python train.py --rf-only                 # 仅 RF 验证（先跑这个！）
python train.py                           # 完整训练（RF+MLP+量化）
python train.py --model a                 # 仅训练模型 A
python train.py --export                  # 导出 C 头文件

# 5. 部署到 C3
# 将 models/model_a.h 和 model_b.h 复制到固件目录
# 在 new_radar.ino 中 #include 并使用 TensorFlow Lite Micro 推理
```


## 关键验证关口

| # | 关口 | 预估成功率 | 依赖 |
|---|------|:---------:|------|
| 1 | LD2450 X 轴可区分左中右 (≥1m) | 85% | 已部分验证 |
| 2 | LD2410C Es 可感知注意力转移 | ~45% | **核心风险** |
| 3 | 双雷达干扰在 ML 中可容忍 | 60% | 需一致性 |
| 4 | MediaPipe 产出可靠标签 | 75% | 窗口设计兜底 |
| 5 | RF >85% → 特征有效 | 50% | 依赖 2, 3, 4 |
| 6 | C3 MLP 推理达标 | 95% | ESP-NN 基准 |
| 7 | 最终产品可用 | 55% | 综合 |


## 参考

- [dronefreak/human-action-classification](https://github.com/dronefreak/human-action-classification) — 两层标签架构 + 关键点归一化 + Keypoint MLP 参考实现
- [HLK-LD2450 协议手册](http://r0.hlktech.com/download/HLK-LD2450-24G/) — X/Y 符号编码为 bit15=1→正（非标准补码）
- [ESP-NN](https://github.com/espressif/esp-nn) — ESP32-S3/C3 推理性能基准
- Hao et al., "Bootstrapping Autonomous Driving Radars with Self-Supervised Learning", CVPR 2024 — 跨模态雷达-视觉蒸馏参考范式

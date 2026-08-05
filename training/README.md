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


## 研究方向与开放问题

以下问题横跨三个子领域（传感系统 / 跨模态蒸馏 / 交互设计），按可操作性和发表潜力排列。标注了每个问题的研究方法建议和预期产出类型。

---

### 方向一：低成本雷达信号表征（Sensing Systems）

> 顶会：SenSys, IPSN, Ubicomp (IMWUT Note)

**RQ1.1 — LD2450 的 X 轴精度-距离函数是什么样的？**

这是最基础的问题。在不同目标距离（0.3 / 0.5 / 1.0 / 1.5 / 2.0 m）下，记录静止人体目标的 X 坐标标准差（σ）。预期得到一个非线性曲线：σ ∝ 1/distance²（来自相位干涉的几何稀释）。

**方法：** 多距离定点实验，每距离 60 秒 × 3 次重复。对照组：单雷达 vs 双雷达同时开（量化干扰对 X 精度的影响）。

**产出：** 4 页 note。这是文献中首次对 LD2450 X 轴的系统表征。阴性结果（"X 精度在任何距离都不够用"）和阳性结果（"≥1m 后精度进入可用区间"）同样成立。

---

**RQ1.2 — LD2410C 的 stationary energy (Es) 对目标朝向敏感吗？**

用一个可旋转的平板/人体模型或真实受试者在固定距离（1m）处，从正对（0°）旋转到侧对（90°），记录 Es 值的变化曲线。理论预测：Es ∝ RCS（雷达截面），RCS 随朝向在 3-6 dB 范围内变化。

**方法：** 10°步进旋转实验，每角度 20 秒。同样需要双雷达/单雷达对照组，排除干扰导致 Es 波动。如果用真人受试者，用 MediaPipe head_yaw 做朝向的 Ground Truth。

**产出：** 如果 Es 对朝向有明显响应（>2dB 动态范围），这是这个方向最关键的阳性发现——"24GHz 静止能量可以感知目标朝向"是一句此前没人用实验确认过的陈述。如果没有响应，这是同等重要的阴性发现，定义了 LD2410C 的物理极限。

---

**RQ1.3 — 事件相关雷达响应（Event-Related Radar Response, ERRR）**

注意力转移事件（手机响、有人进房间、HappyMac 突然动作）前后的雷达信号在全维度（X/Y/V/Em/Es）上是否表现出统计显著的变化？

**方法：** 重复诱发注意力转移 30+ 次，画事件相关的时间序列图（±5s 窗口，类似 EEG 的 ERP 分析）。配对 t 检验比较事件前 2s 基线 vs 事件后 2s 响应窗。

**产出：** 如果有显著响应——这是最强的论文实验结果，直接证明了雷达能"感知"一个认知事件。如果不显著——同样是有价值的结论：注意力转移可能不在 24GHz 雷达的可感知物理特征范围内。"ERRR" 这个缩写没人用过，你可以命名。

---

**RQ1.4 — 双 24GHz FMCW 雷达同频干扰的量化表征**

两个雷达同时开启时，(a) LD2410C 的底噪、(b) LD2450 的 X/Y 精度劣化、(c) 幽灵目标的出现频率——各是多少？

**方法：** 单雷达 vs 双雷达对照实验（拔掉 2410 VCC 作为单雷达基线）。三个因变量：(a) LD2410C 的 Em/Es 在"空房间"条件下的值（底噪），(b) LD2450 X 坐标静止目标的 σ，(c) Target 槽位 T1/T2 的假目标活跃率。两雷达在 3 种相对距离（10cm / 30cm / 50cm 天线间距）下分别测试。

**产出：** 这是 DIY 社区里被反复讨论但没人系统测过的问题。数据本身就是贡献。

---

### 方向二：跨模态蒸馏与 TinyML 部署（Embedded ML / Cross-modal Sensing）

> 顶会：Ubicomp (IMWUT), PerCom, AAAI tiny paper track

**RQ2.1 — 视觉 Teacher 能多准地识别桌面注意力状态？（天花板实验）**

纯用视觉关键点（Face Mesh 468 点 + Pose 33 点），训练一个 keypoint MLP 做 6-8 类桌面注意力状态分类。准确率 = Teacher 能力天花板 = Student 的理论上界。

**方法：** 借鉴 human-action-classification 的 keypoint 管线（50→128→64→n MLP），用规则标签做 baseline，与 ML 训练标签做对比。分析 misclassification 的来源（哪些状态之间容易混淆？人是如此还是模型也是如此？）。

**产出：** Teacher 准确率如果 >90%——标签体系可靠，天花板足够高，Student 有追赶空间。Teacher 准确率如果 <75%——说明"桌面注意力状态"即使摄像头也分辨不好，雷达不是问题，标签体系需要重新设计。两种结果都导向可发表的讨论。

---

**RQ2.2 — 从视觉 Teacher 到雷达 Student 的跨模态蒸馏中，多少信息损失来自模态 gap，多少来自模型压缩？**

分解信息损失路径：视觉 Teacher（FP32 MLP）→ 雷达 Student（FP32 MLP）→ 量化（INT8 MLP）。每一步的精度损失各是多少？

**方法：** 三步对比实验：(a) 视觉 keypoint → 同类标签的 MLP 准确率，(b) 雷达 18 维特征 → 同类标签的 MLP 准确率，(c) 同 (b) 但 INT8 量化后。Δ1 = (a)-(b) = 模态 gap；Δ2 = (b)-(c) = 量化损失。

**产出：** 大部分跨模态蒸馏论文只报告了总精度差。你把这个差分解成两个可独立优化的部分——这是一个方法贡献。

---

**RQ2.3 — 18 个雷达特征中，哪些真正有贡献？（消融实验）**

逐个移除特征/特征组，观察精度变化。同时报告 Random Forest 的特征重要性排序。

**方法：** Ablation study:(i) 只用 LD2450 所有特征，(ii) 只用 LD2410C 所有特征，(iii) 移除所有组合特征只留基础统计特征，(iv) 移除 es_div_y_mean 孤立测试其单独贡献。每组跑 5 折交叉验证。

**产出：** 特征重要性排序 + 最小特征子集（可能 top-10 就够）。如果 es_div_y_mean 是 top-3 重要特征——验证了我们在雷达方程上的理论推导。如果它在 bottom-3——理论推导有问题，值得一篇短论文讨论为什么。无论正负，回答了一项可验证的科学问题。

---

**RQ2.4 — MLP 在 ESP32-C3 上的推理延迟和能耗**

实测 INT8 量化后的 MLP（32→16→softmax）在 C3 上的单次推理时间、内存占用、功耗。

**方法：** `micros()` 计时 + 示波器测 GPIO 翻转功耗脉冲。和 ESP-NN 基准数据对比，验证 <10ms 的预期是否成立。

**产出：** 基准数据本身不是主要贡献，但在论文里是必须有的 Engineering Validation 段落。

---

### 方向三：不完美感知的交互设计（HCI / Design）

> 顶会：CHI, DIS, TEI

**RQ3.1 — "Attention-Oriented Sensing"：传感器不需要测到眼睛方向就能感知注意力转移——这个设计原则如何系统化？**

将你在 HappyMac 上形成的方法论抽象为一个可推广的设计框架：(1) 拒绝"精确测量"的传感器选择标准，选"对状态切换最敏感"的传感器；(2) 用传感器融合（双雷达）弥补单传感器的物理盲区；(3) 用概率状态空间替代确定性阈值。

**方法：** 除了 HappyMac 之外，找一个或两个其他场景（比如：教室座位上的注意力监测、会议室里的参与度感知），用同样原则做纸上设计推演，展示框架的可迁移性。

**产出：** 设计方法论文。CHI/DIS 的 full paper。这是三个方向里最不需要技术完成度的——你主要卖的是一个原创的设计概念，原型是证据但不是全部。

---

**RQ3.2 — 概率驱动的表情状态机 vs 二值检测的表情状态机：用户体验有区别吗？**

同一个人面对两种模式：(a) 确定性模式——"人在左边 → 脸看向左"，(b) 概率模式——"左 60% / 中 30% / 右 10% → 脸往左偏但有点犹豫"。用户感受到的"生命感"有差异吗？

**方法：** 小规模用户研究（3-5 人即可做 pilot）。在限定场景下做 within-subject A/B 对比，用 Likert 量表 + 开放式访谈收数据。这不是找"显著性"——pilot study 在 HCI 里够发 workshop 或作为 full paper 的 preliminary evidence。

**产出：** 如果概率模式被评价为"更像活的"——你不是在报告一个工程优化结果，你是在报告一个交互设计原则。这条原则比 HappyMac 这个产品活得久。

---

**RQ3.3 — "不完美感知"如何影响用户对设备的信任感和陪伴感？**

让受试者使用 HappyMac 一段时间（哪怕 10 分钟），然后告诉他们"它偶尔会看错方向，因为它没有摄像头，只有一个很便宜的雷达芯片"。他们说"它更像宠物了"还是"它不好用"？

**方法：** 准结构化访谈。不给受试者预设"你觉得它像宠物吗"——让他们自己找词描述。你只做转录和聚类分析。

**产出：** 定性的用户反馈。如果"雷达感知的不完美→更像宠物"这个猜测被受试者自发表达出来——你在 CHI 的 contribution 就成立了。因为"让传感器故意保持不精确"这个主张在此之前没有实验证据。

---

**RQ3.4 —（如果你做多传感器对比）— 用户在不知道传感器类型的情况下，对摄像头、雷达、ToF 驱动的同一交互的感受有何不同？**

三种传感器实现同一个"桌面宠物"交互（摄像头版/雷达版/ToF版），用户盲测，记录他们对"监视感""生命感""舒适度"的描述。

**方法：** 需要做三个原型变体。工作量大，但如果做出来——CHI 级别的 full paper。

**产出：** 实验证据支持"传感器选择是设计决策，不是纯粹的技术决策"。这是 HCI 社区喜欢的 argument。

---

### 路线建议

```
现在 → MVP（数据到手）：
  RQ1.1 + RQ1.2 + RQ1.3 的数据一锅采集，一套实验回答三个问题
  → 决定哪些 RQ 有阳性信号

MVP 之后 → Demo 之前：
  RQ2.1（天花板实验）决定是否值得往下做
  RQ2.3（消融实验）指导特征精简化

Demo 之后 → 论文：
  选一条主线路：
    - 信号表征主线 (Paper 1): RQ1.1 + RQ1.2 + RQ1.4
    - 跨模态蒸馏主线 (Paper 2): RQ2.1 + RQ2.2 + RQ2.3
    - 交互设计主线 (Paper 3): RQ3.1 + RQ3.2 + RQ3.3

  每篇论文只需回答 2-3 个 RQ 即可成立
  不必全部做完——选实验结果最好的那条线往里深挖
```

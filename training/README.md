# HappyMac — TinyML 训练管线

> 📋 **下一轮正式数据采集（装壳后）的流程/内容/质量门规范见 [DATA_COLLECTION_V2.md](DATA_COLLECTION_V2.md)**——开工前必读。V1 数据（`sessions/` 2026-08 八会话）体检结论与问题追踪也在这份文档里。

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


## T1 TinyML 首次实验（2026-08-13）

### 结论：可行性绿灯 + 两个关键教训

| 实验 | 结果 | 含义 |
|------|------|------|
| 协议数据 RF（3折） | acc=0.778, F1=0.708 | 信号存在，特征修复后显著提升 |
| 自由数据 RF（3折） | acc=0.681, F1=0.625 | 自然数据更难（标签噪声+行为混合） |
| 融合数据 RF（3折） | acc=0.683, F1=0.626 | — |
| **迁移：协议→自由** | **acc=0.323** | **表演数据训练的模型在真实场景崩溃** |
| **融合→自由** | **acc=0.811, F1=0.765** | **加自然数据后翻倍** |

### 关键发现

1. **Es 数值是头号特征**（重要性 0.346，超 slope_Y 三倍）——"LD2410C 静止能量数值是姿态弱特征"的假设被训练直接验证
2. **X 特征修复**：删除 slope_X（方向抵消）、slope2_X（零方差）、X_reversals（阈值失效），改用 |slope_X| → 协议数据 F1 从 0.537 提升至 0.708
3. **协议数据 ≠ 自然行为**：只用脚本表演数据训练，迁移到自由行为时崩溃（acc 0.323）。自然数据必须进入训练集
4. **视觉 Teacher 自动打标签跑通**：MediaPipe（存在+横向）+ MiDaS（纵深）实时标注，3408 次自动标签切换（0.1s/次，需平滑）
5. **剩余硬骨头**：STILL↔LATERAL 混淆（互混 300+ 窗口），APPROACH 精度弱（74/170）

### 数据管线

```
collect_session.py   协议数据采集（9 阶段 × 3 session）
collect_free.py      自由数据采集（视觉 Teacher 自动打标签）
features_t1b.py      特征工程（9 特征，支持协议+自由融合）
train_compare_t1b.py 训练对比（proto/free/fused/迁移）
```

特征集（9 维）：|slope_X|, slope_Y, std_X, std_Y, mean_Es, std_Es, mean_abs_V, max_abs_V, Es_edge

### 下一步

- [ ] 再采 2-3 次自由数据（每次 5 分钟，提升自然场景精度）→ 已并入 [DATA_COLLECTION_V2.md](DATA_COLLECTION_V2.md) V2 轮次计划
- [ ] 攻 STILL↔LATERAL 混淆（窗口长度调优 / 深度特征增强）
- [ ] RF → MLP 蒸馏 → INT8 量化 → C3 部署
- [x] ~~补 APPROACH 数据~~（2026-08-30 定案：APPROACH/RETREAT 由 Y 趋势规则承接，ML 不做这两类，见「分类 → 检测通道 → 动画」）

> **2026-08-30 评估修正**：本节的 3 折随机切分指标系统性高估（同 session 窗口互相泄漏），
> 引用请以下节 LOSO 诚实值为准；转头类实验结论见下节。

---

## T1b 对齐数据集与诚实评估（2026-08-30）

### 数据集与评估方法

`t1b_aligned_X/y/groups.npy`（`align_all_sessions.py` 产出）：5 个规则协议 session +
3 个自由 session，视频对齐标签（`aligned_action`），4744 窗 / 9 特征 / 2s 窗 0.5s 步进。
类别分布：STILL 2340 / LATERAL 945 / ABSENT 542 / RETREAT 523 / APPROACH 394。

评估一律用**留一会话交叉验证（LOSO）**：每次留出整个 session 做测试。
随机切分（本文件 T1 节的 0.811/0.765、`train_compare_t1b.py` 旧报告）因同 session
窗口互相泄漏而系统性高估，**引用一律以 LOSO 为准**。

### LOSO 主结果（RF 120 树，tinyml_v0_aligned_report.json）

| 指标 | 数值 |
|---|---|
| session LOSO acc / macro-F1 | **0.563 / 0.462** |
| 规则数据训练 → 自由数据 | 0.408 / 0.245 |

最难折都是自由 session（acc 0.46-0.50）——自然行为混合 + Teacher 标签噪声是主要误差源。

### 模型阶梯（tinyml_v0_model_ladder_report.json）

树 d3/d4/d5、RF 10/20/120 树的 LOSO 全部挤在 acc 0.49-0.56 / macro-F1 0.42-0.46：
**模型容量不是瓶颈，9 特征的信息上限就在这里**——提升精度只能靠特征/窗口工程，
不能靠换大模型。上机选 d4 决策树（25 节点 / 3.8KB，`tinyml_v0_tree_d4.h` 已进 v0 固件）：
RF 精度无优势且进不了 C3。

### 逐类可靠度（LOSO 混淆矩阵，RF 120 树）

| 类 | 召回 | 主要混淆去向 | 结论 |
|---|---|---|---|
| STILL | 68% | →RETREAT 261 / →APPROACH 151 | ML 可用 |
| LATERAL | 60% | →STILL 223 | ML 可用，配规则积分器或门兜底 |
| ABSENT | 58% | →LATERAL 102（无人判在动） | ML 不可用，存在性走 SR602/规则 |
| RETREAT | 25% | →STILL 196 | ML 不可用，Y 趋势规则顶 |
| APPROACH | 18% | →STILL 146 | ML 不可用，Y 趋势规则顶 |

### 转头可分性实验（exp_head_separability.py，2026-08-30）

head_only 单独成类（STILL/TYPING/HEAD 三分类）× 现有 9 特征 × 2s 窗 × LOSO（5 会话 1185 窗）：

```
HEAD 召回 54.0%；TYPING→HEAD 误报 28.0%；STILL→HEAD 误报 9.1%
树深度 3-6 扫描：acc 52-54%，无改善
```

**结论：转头在当前 9 特征下不可分。** 此结果是 V2 验收门
（[DATA_COLLECTION_V2.md](DATA_COLLECTION_V2.md) §7②：召回 >75% 且打字误报 <10%）的
V1 基线；V2 的调敏 em + 3 目标 + turn_toward 数据是翻案机会，不达标则维持
"小幅动作归 STILL"。翻案方向是**新特征**（Y 微动谱 / Es 频域分量 / 更长窗口），
不是更多同类数据。

### 专注 vs 发呆（窗口级不可分，降级）

still/typing 窗口级互混 ~50%（STILL→TYPING 170 窗 / TYPING→STILL 85 窗）。
Es 组间差（typing 72.8 vs still 63.9，见 `docs/REPORT_radar_characterization.md`）
只在分钟级平均下成立。**定案：不作为分类目标；用 30-60s 长窗 mean_Es 驱动
眨眼频率 / 呼吸节奏 / 亮度等"氛围"参数（慢通道，不追求实时）。**

---

## 分类 → 检测通道 → 动画（定案）

原则：每条动画只押在**对该信号最可靠的检测通道**上；ML 只做混淆矩阵证明它做得到的事。
LOSO 0.563 意味着误分类是常态——「ML 输出只是期望、动画层负责容错」从设计哲学升级为工程必需。

| 人体活动 | 检测通道（定死） | 动画 |
|---|---|---|
| 离场 / 回来 | SR602 + 雷达目标存在，500ms 防抖、8s 进睡眠（纯规则） | GOODBYE → SLEEP；WAKING 渐睁眼 |
| 静止在场（含打字） | ML STILL，2s 窗 + ~2000ms 确认 | IDLE：慢眨眼、正常跟随、静止注意力漂移 |
| 横移 / 大动作 | ML LATERAL + 规则运动积分器**或门**；方向由 slope_X 符号给出，不进 ML | ACTIVE：睁大眼 + 微笑（600ms 进入延迟 + 1.5s 最短保持） |
| 凑近 / 远离 | LD2450 Y 趋势规则（15mm/140ms + 重武装滞回），ML 仅放行 | APPROACH / RETREAT 短事件动画，播完回 IDLE 不锁表情 |
| 左中右位置 | X 相对校准基线 + 双阈值滞回（快通道，不进 ML） | 连续 face_look ±24px + 几何视差（鼻 7 / 嘴 4 / 眼距收缩 14px） |

两条**连续调制通道**（不占状态）：

1. **mean_Es 长窗（30-60s）** → 眨眼频率 / 呼吸节奏 / 亮度微调（氛围）
2. **mean_abs_V（8mm/s 噪声底以上）** → 跟随增益 / 眼高，ACTIVE↔IDLE 连续过渡

**表情相邻原则**：互相混淆的类，动画必须表情相邻——误分类在用户眼里是性格化的犹豫，不是错误。

- STILL↔LATERAL 互混最重 → IDLE/ACTIVE 只差眼高/嘴型（600ms 延迟 + 1.5s 保持已对）
- APPROACH 混入 STILL → APPROACH 是短事件、播完回 IDLE，永不锁表情
- ABSENT 幻觉 19% → SLEEP 由 8s 持续无目标的时间常数吸收
- LATERAL 方向错误 → face_look 限速缓动天然容错（慢慢转过头再修正）

**明确不做**：HEAD 类（V2 验收前）、窗口级 FOCUSED/IDLE、Em 参与任何判定、
stand_sit 独立类（语义已被 presence 边沿的 GOODBYE/WAKING 覆盖）。
零 ML 新动画候选：**「被抓到偷看」**（face_look 已偏侧 + 人回中边沿触发一次性微表情）、
深夜困意（时钟驱动，不占雷达分类）。

---

## 双模型架构：位置（模型 A）+ 状态（模型 B）

### 设计目标

T1b 的 5 类模型回答"人在干嘛"（状态），驱动表情状态机。还需补模型 A 回答"人在哪边"（位置），驱动脸朝向。**两个模型共享雷达数据流，不同时间尺度，独立输出。**

### 模型 A：位置分类（快通道，0.5-1s 窗口）

```
输入特征（3 维，全部是"相对量"）：
  X_rel    = mean_X(1s) − median_X(10s 滚动基线)   ← 漂移免疫
  |slope_X| (1s 窗口)                               ← 是否在横移
  std_X    (1s 窗口)                                ← 横移幅度

输出：LEFT / CENTER / RIGHT（3 类软输出，非硬分类）
方向：slope_X 符号直接给出，不需要 ML 学
```

**漂移与几何偏移的解法**：
- X 绝对漂移（128mm/30s）→ 用滚动基线归一化，只学相对偏移
- 摄像头在雷达右侧的几何偏移 → 视觉 Teacher 的 L/C/R 标签相对"校准参考点"定义，不相对画面中心

**视觉 Teacher 校准协议**（collect_free.py 将加入）：
```
采集开始：用户坐雷达正前方 10 秒
→ 记录脸中心 x 的中位数 = 图像坐标系 CENTER 参考点
→ 后续标签：脸 x 相对参考点 ± 阈值 → L/C/R
→ 每 session 重新校准，吸收坐姿微变和几何偏移
```

### 模型 B：状态分类（慢通道，2s 窗口）

T1b 已实现：ABSENT / STILL / LATERAL / APPROACH / RETREAT，融合训练 F1=0.765。

### 仲裁层与动画层（解耦设计）

```
雷达 → 特征 → 模型A [P(L),P(C),P(R)] + 模型B [5类概率]
              ↓                          ↓
        target_angle = P(R)×30° − P(L)×30°   （期望朝向角，可模糊）
              ↓
        动画层（死区 + 平滑限速 + 几何视差）：
        face_target += (desired − target) × 0.08
        face += clamp((target − face) × 0.06, ±0.55px)
        整脸共同平移 + 眼距收缩 + 鼻/嘴相对偏移
              ↓
        OLED 20Hz 重绘
```

**核心原则**：
1. **ML 输出是"期望方向"**，允许模糊、允许概率——动画层负责"怎么转过去"，一定平滑有惯性
2. **不写死边界**：位置是连续姿态空间，脸经过软边界时"犹豫"地停在两区之间，不跳变
3. **缺陷变性格**：雷达的噪声/漂移/延迟经低通滤波后成为脸的"注意力飘忽"和"想了想才转头"——这正是 Paper 3 不完美感知设计的具体数学实现
4. **仲裁优先级**：ABSENT > 状态转换 > 位置。人从右侧走过 = 模型B 说 LATERAL + 模型A 说 RIGHT + slope 符号说往左 → 脸先转向右，再跟着左移（伪3D视差跟随）

### 模型 A 实施状态

- [x] C3 上先接入 Model A v0：LD2450 X 相对校准中心的 LEFT/CENTER/RIGHT 分区
- [x] 使用进入/退出双阈值滞回，避免边界附近左右抖动；动画仍使用连续 look 值
- [ ] collect_free.py 加校准段（10 秒记录 ref_x）+ 输出 L/C/R 标签
- [ ] 特征脚本加 3 维位置特征（X_rel, |slope_X|, std_X）
- [ ] RF 训练 3 类（预期高于 5 类状态模型）
- [x] C3 动画层实现（死区 + 限速跟随 + 几何视差 + 12.5Hz OLED）

## 关键验证关口

| # | 关口 | 预估成功率 | 状态 |
|---|------|:---------:|------|
| 1 | LD2450 X 轴可区分左中右 (≥1m) | 85% | ✅ 已验证（趋势可用，绝对值漂移） |
| 2 | LD2410C Es 可感知注意力转移 | ~45% | ❌ 证伪（朝向无信号）；✅ Es 数值是姿态特征 |
| 3 | 双雷达干扰在 ML 中可容忍 | 60% | ⏳ 待 AB 对照（V2 以 es 基线登记控制，不消除） |
| 4 | MediaPipe 产出可靠标签 | 75% | ✅ 跑通；Teacher 标签噪声是自由 session 难折的主因之一 |
| 5 | RF 特征可分性 | 50% | ⚠️ 随机切分 F1=0.765 → **LOSO 诚实值 0.462**；瓶颈在特征不在模型 |
| 6 | C3 MLP 推理达标 | 95% | ⏳ d4 决策树已上机；MLP/INT8 蒸馏待做 |
| 7 | 最终产品可用 | 55% | ⏳ MVP 故事线 81% 成立；误笑率 16.6% → 目标 <10% |
| 8 | 转头（head_only）可分 | — | ❌ V1 基线证伪（召回 54% / 打字误报 28%）；V2 验收门 >75% / <10% |


## 参考

- [dronefreak/human-action-classification](https://github.com/dronefreak/human-action-classification) — 两层标签架构 + 关键点归一化 + Keypoint MLP 参考实现
- [HLK-LD2450 协议手册](http://r0.hlktech.com/download/HLK-LD2450-24G/) — X/Y 符号编码为 bit15=1→正（非标准补码）
- [ESP-NN](https://github.com/espressif/esp-nn) — ESP32-S3/C3 推理性能基准
- Hao et al., "Bootstrapping Autonomous Driving Radars with Self-Supervised Learning", CVPR 2024 — 跨模态雷达-视觉蒸馏参考范式


## 研究方向与开放问题

以下问题横跨三个子领域（传感系统 / 跨模态蒸馏 / 交互设计），按可操作性和发表潜力排列。标注了每个问题的研究方法建议和预期产出类型。

### 进度账本（2026-08-30 校准）

总判断：**工程完成度 >> 科学完成度**。管线 / 固件 / 模拟器 / 回放评测全通，
但三个方向各自缺的受控实验一个都还没正式跑。论文引用指标一律以 LOSO 为准。

| RQ | 状态 | 缺什么 |
|---|---|---|
| 1.1 X-距离函数 | ⏳ 半闭环 | 5 距离 × 3 重复扫描；已有单距离漂移定量化（σ=128mm/30s @0.7m）；V2 的 still_near/far 是起点 |
| 1.2 Es-朝向 | ✅ 闭环 | 无——朝向证伪（\|r\|<0.12）+ Es 姿态特征阳性副产品，可直接成文 |
| 1.3 ERRR | ❌ 未动 | 30+ 次诱发事件采集 |
| 1.4 干扰 AB | ❌ 未动 | 单/双雷达 × 3 天线间距对照（V1 两雷达同开，无基线） |
| 2.1 教师天花板 | ❌ 未动 | **可离线**：8 个 V1 会话 mp4 已存 + `t_video_rel` 可重对齐，重跑 MediaPipe + keypoint MLP |
| 2.2 损失分解 | ❌ 未动 | 教师半边可离线；学生半边已有（LOSO 0.563）；量化半边待 INT8 |
| 2.3 消融 | ❌ 未动 | **可离线**：`exp_head_separability.py` 即模板 |
| 2.4 C3 延迟能耗 | ❌ 未动 | 待 MLP 蒸馏 + INT8 上机 |
| 3.1-3.4 | ❌ 未动 | 最便宜是 3.2 pilot：`DETERMINISTIC_MODE` 编译开关 + 3-5 人 Likert |

最快成文路径 = **Paper 1**（只缺受控数据，不缺分析与写作框架，已有
[docs/REPORT_radar_characterization.md](../docs/REPORT_radar_characterization.md) 作素材）；
产品优先则先做窗口扫描 + eval_replay 指标闭环。

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

### 下一步行动清单（2026-08-30 排定）

**A. 离线可做（无硬件，随时）**
1. RQ2.3 特征消融：特征组（纯 LD2450 / 纯 LD2410C / 去 Es_edge）× LOSO，模板 = `exp_head_separability.py`
2. 窗口长度扫描：window ∈ {1, 2, 3, 5}s × LOSO，专看 STILL↔LATERAL 互混变化
3. RQ2.1 教师天花板（离线）：对 8 个 V1 会话 mp4 重跑 MediaPipe，用 `t_video_rel` 对齐 `aligned_action`，训 keypoint MLP（>90% = 标签可靠有追头空间；<75% = 标签体系需重设计——两种结果都可发表）
4. RQ2.2 量化半边：RF → MLP 蒸馏 + INT8 仿真，Python 侧先量出 Δ2

**B. 需要硬件（与 V2 采集同场搭车）**
5. Paper 1 补实验日：距离扫描（0.3/0.5/1.0/1.5/2.0m × 静止/慢扫 × 3 重复）+ 干扰 AB（拔 2410 供电基线 × 3 天线间距），分析脚本现成
6. 补自由数据已并入 V2 轮次计划（[DATA_COLLECTION_V2.md](DATA_COLLECTION_V2.md)）

**C. 需要真人（原型稳定后）**
7. RQ3.2 pilot：固件加 `DETERMINISTIC_MODE` 编译开关（硬边界取概率路径），3-5 人 within-subject Likert + 开放式访谈

优先级：目标最快成文 → A1/A2 → B5（Paper 1 投入产出比最高，RQ1.2 已闭环可先写）；
目标 MVP → A2 + eval_replay 指标闭环 → V2 重训。A 组与产品线完全共用，无选边成本。

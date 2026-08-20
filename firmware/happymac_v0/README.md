# HappyMac v0 动画固件

这是第一版产品固件，目标是先完成一个可以装入 3D 打印外壳、持续运行并展示完整生命感的原型。

这一版的重点不是证明 TinyML 精度，而是完成：

- OLED 像素脸；
- 睡眠、醒来、待机和动作反馈；
- 左右跟随和伪 3D 视差；
- 雷达、动画和外壳之间的完整闭环；
- 为之后接入 Model A / Model B 保留稳定接口。

产品固件和研究采集固件分开。v0 可以完全不使用摄像头、不使用 ML，只用双雷达和 SR602 的规则判断驱动 OLED 动画。

## 一、整体架构

最终采用三层结构：

```text
传感器 / TinyML 模型 / 规则系统
              ↓
       PerceptionOutput
              ↓
        StateMachine
              ↓
       AnimationLayer
              ↓
             OLED
```

核心原则：

1. 位置模型只回答“人在哪里、脸应该看向哪里”。
2. 状态模型只回答“当前是什么状态、应该用什么情绪表现”。
3. 动画层负责平滑、惯性、视差、亮度和一次性事件动画。
4. 不让两个模型直接竞争一个最终状态。

## 二、当前状态

| 状态 | 进入条件 | 动画表现 |
|---|---|---|
| `SLEEP` | 连续 8 秒没有目标 | 闭眼、低亮度呼吸，不跟随 |
| `GOODBYE` | 目标刚刚消失 | 低落、目送，短暂保留最后方向 |
| `WAKING` | 睡眠后重新检测到目标 | 眼睛逐渐睁开，跟随强度逐步恢复 |
| `IDLE` | 有人且运动较小 | 普通待机、慢眨眼、正常跟随 |
| `ACTIVE` | 速度或位置变化明显 | 睁大眼、开心表情、加强跟随 |
| `APPROACH` | Y 窗口趋势变小，目标靠近 | 惊讶、大眼、短暂事件动画 |
| `RETREAT` | Y 窗口趋势变大，目标远离 | 目送、眼睛变小，随后回到待机 |

左右跟随不是独立状态，而是连续的 `face_look` 参数。这样不会因为 LD2450 X 轴漂移而在 `LEFT` / `CENTER` / `RIGHT` 之间频繁跳变。

`APPROACH`、`RETREAT`、`WAKING` 和 `GOODBYE` 既是状态，也是事件入口。事件动画播放一小段时间后，应回到 `IDLE` 或 `ACTIVE`，不能永久锁住表情。

## 三、Model A 和 Model B 的职责

文档中原来的编号是：

- Model A：位置模型；
- Model B：状态模型。

后续实现应优先按功能称呼“位置模型”和“状态模型”，避免编号混淆。

### 位置模型

输入 LD2450 的 X/Y、趋势、方差等短窗口特征，输出：

```text
p_left
p_center
p_right
```

它只驱动：

- 脸的左右目标位置；
- 眼球方向；
- 鼻子、眼睛、嘴巴的视差偏移。

它不直接改变睡眠、活跃或惊讶状态。

### 状态模型

输入双雷达的时序特征，输出例如：

```text
p_absent
p_idle
p_active
p_approach
p_retreat
```

后续还可以扩展：

```text
p_focused
p_distracted
p_look_at_it
```

它只驱动：

- 眼睛开合；
- 瞳孔大小；
- 嘴巴形状；
- 眨眼频率；
- OLED 亮度；
- 呼吸速度；
- 一次性表情事件。

## 四、融合方式

### 4.1 位置概率转换为连续方向

不使用位置模型的硬分类结果，而使用概率差：

```cpp
direction = p_right - p_left;
look_target = direction * 8.0f;
```

当模型不确定时，保持原方向，不突然跳转：

```cpp
confidence = max(p_left, max(p_center, p_right));

if (confidence < 0.55f) {
    look_target *= 0.95f;
}
```

### 4.2 状态控制跟随强度

不同状态使用不同的跟随增益：

```text
SLEEP       0.00
WAKING      0.35
IDLE        1.00
ACTIVE      1.20
APPROACH    0.60
RETREAT     0.50
GOODBYE     0.00
```

最终方向：

```cpp
final_look = position_look * look_gain;
```

含义是：睡着时不追踪，醒来时逐渐恢复，普通待机正常跟随，惊讶或告别时优先表现情绪。

### 4.3 动画层负责平滑

脸不能直接跳到模型输出的位置，使用缓动：

```cpp
face_look += (final_look - face_look) * 0.15f;
```

视差层级：

```text
鼻子：face_look × 1.0
眼睛：face_look × 0.7
嘴巴：face_look × 0.4
```

这会形成“鼻子先动、眼睛跟随、嘴巴略滞后”的伪 3D 效果。

## 五、状态切换和防抖

模型输出要先经过概率平滑：

```text
新状态概率 > 0.60，连续两个窗口出现
→ 才切换到新状态

当前状态概率 < 0.40
→ 才允许离开当前状态
```

这样可以避免：

```text
IDLE → ACTIVE → IDLE → ACTIVE
```

状态模型的时间窗口可以较慢，位置模型的窗口可以较快：

```text
位置模型：0.5–1 秒窗口，快速跟随
状态模型：2–5 秒窗口，稳定判断情绪
动画层：20Hz 重绘，持续缓动
```

## 六、典型场景

### 人在右侧并且正在活动

```text
位置模型：RIGHT 0.82
状态模型：ACTIVE 0.76
```

表现：睁大眼、开心表情，脸平滑转向右侧，鼻子位移最大。

### 人突然靠近

```text
位置模型：CENTER 0.60 / RIGHT 0.30
状态模型：APPROACH 0.78
```

表现：播放约 500ms 的惊讶动画，眼睛睁大、嘴巴变成 O，暂时降低左右跟随，随后回到 `ACTIVE` 或 `IDLE`。

### 人离开

```text
状态模型：ABSENT 0.85
```

表现：停止跟随，脸慢慢回到中心，亮度降低，进入呼吸睡眠。

## 七、统一模型接口

未来规则系统、Random Forest、MLP 和 INT8 模型都应该转换成同一个结构：

```cpp
struct PerceptionOutput {
  float p_left;
  float p_center;
  float p_right;

  float p_absent;
  float p_idle;
  float p_active;
  float p_approach;
  float p_retreat;

  bool valid;
};
```

动画层不关心输入来自哪里。当前 v0 使用规则系统生成相同语义的结果；以后只替换感知输入层，不重写表情和状态机。

## 八、当前固件状态

当前 `happymac_v0.ino` 已经实现：

- 7 个基础状态；
- 规则版状态切换；
- 睡眠呼吸；
- 醒来、活跃、惊讶、目送表情；
- 左右连续跟随；
- OLED 20Hz 重绘；
- `STATE`、`RADAR`、`ANIM` 串口日志；
- `!CAL` 重新校准；
- `!STATE` 查询状态。

当前还没有实现：

- 真正的 Model A 位置推理；
- 真正的 Model B 状态推理；
- INT8 模型加载；
- 模型概率输出接入；
- 装入最终外壳后的阈值校准。

## 九、研究模式和产品模式

- 研究采集：`new_radar/new_radar.ino`
- 产品动画：`firmware/happymac_v0/happymac_v0.ino`

两个文件不要混用。研究固件输出采集阶段信息，产品固件输出 `STATE`、`RADAR` 和 `ANIM` 日志。后续正式实验应基于已经确认的外壳、雷达角度和元件位置重新采集。

## 十、当前已知限制

- `Em` 不作为主要判据；桌面距离下它不稳定。
- `APPROACH` / `RETREAT` 依赖 LD2450 Y 趋势，装入外壳后需要重新调阈值。
- LD2450 X 存在慢漂，因此位置输出必须使用相对基线、窗口趋势和概率平滑。
- 这一版只是规则版动画，不代表最终 TinyML 分类结果。
- S3 摄像头仍只作为训练期 Teacher，不放进无摄像头产品形态。

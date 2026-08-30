# HappyMac OLED 模拟器

无硬件时的开发验证环境：在浏览器里 1:1 复现 `firmware/happymac_v0` 的完整逻辑链路
（状态机 / 运动证据积分 / Y 趋势武装滞回 / TinyML 决策树 / 动画层），并以
128×64 等比例、整数倍放大渲染 1.3″ SH1106 OLED 画面（旁边附真实物理尺寸预览）。

## 打开方式

直接双击 `happymac_sim.html`（单文件、无依赖、file:// 可用），或：

```bash
open happymac_sim.html
```

## 能干什么

| 用途 | 做法 |
|---|---|
| 调动画/阈值参数 | 改文件顶部 `FW` 常量区（与固件 `#define` 一一对应），确认效果后再手动同步回 `.ino` |
| 验证防抖修复 | 勾选「旧版单帧判定」复现修复前的笑容横跳，关掉即当前固件行为 |
| 回放真实数据 | 「CSV 回放」选 `training/sessions/*.csv`（`t_global,x,y,v,em,es` 格式），真实雷达数据驱动同一套动画 |
| 快速过全状态 | 「场景剧本」：静坐 / 凑近 / 左右探头 / 离开回来（含睡前链）/ 深夜静止 / 贴脸强噪声 |
| 看睡前过渡链 | 「离开→回来」剧本：无人 2.5s 后打哈欠→揉眼→渐暗→入睡，人回来随时打断；串口面板有 `EV,` 日志 |
| 与真机对拍 | 串口面板输出与固件相同的 `STATE/ML/POS/RADAR/ANIM` 日志格式 |

## 噪声模型（按实测标定）

- LD2450 实际输出 ≈15Hz（采集 CSV 中位间隔 67ms；旧固件因 OLED 重绘拖慢，处理速率仅 ≈7.6Hz）；
- 静态噪声为慢漂移：边际 σ≈100mm@0.7m，帧间相关 ρ≈0.997，
  对应静坐时滤波步进（140ms）中位数 ≈5.5mm（2026-08-29 实测）；
- 速度 v 静坐时中位数为 0，仅 0.7~3.1% 的行出现 8 的倍数尖峰（跟踪器毛刺）。

## 测试

```bash
node test_sim.js          # 场景断言（固定种子，可复现）
node eval_replay.js       # 真实数据评测：回放全部采集 CSV，对比固件状态 vs 真实动作标签
node eval_replay.js --deglitch   # 同上，但先剔除 v 毛刺帧（对照实验）
node eval_replay.js path/to/session.csv  # 只评单个会话
```

评测口径：still/typing/head_only 期望 IDLE（期间 ACTIVE = 误笑）；
运动类期望 ACTIVE（动作间停顿 IDLE 也算对）；每段动作切换后 2.5s 过渡宽限不计分；
`free_5min` 无期望状态不计分；8 月 13 日后会话优先使用视频对齐后的 `aligned_action`。

从 HTML 中抽出核心逻辑在 Node 里无头运行，覆盖：凑近触发 APPROACH、
静坐 90s 强噪声不横跳（新旧判定对照）、离场 GOODBYE→SLEEP、
TinyML 决策树输出对拍、OLED 帧缓冲冒烟。

页面控制台另有 `__sim.state()` 调试钩子可查运行时状态。

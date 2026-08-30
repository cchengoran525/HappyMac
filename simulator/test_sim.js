// HappyMac 模拟器无头测试：从 HTML 中抽出核心逻辑，在 Node 里跑场景断言。
// 用法：node test_sim.js
const fs = require("fs");
const path = require("path");
const os = require("os");

const html = fs.readFileSync(path.join(__dirname, "happymac_sim.html"), "utf8");
const m = html.match(/<script>([\s\S]*)<\/script>/);
if (!m) { console.error("FAIL: 未找到 <script> 块"); process.exit(1); }
const tmp = path.join(os.tmpdir(), "happymac_sim_core.js");
fs.writeFileSync(tmp, m[1]);

// 固定种子伪随机数：噪声场景结果可复现，避免偶发波动造成假失败
let seed = 42;
Math.random = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296; };

const { createSimCore, createRadarModel, createFB, renderFace, FW, STATE_NAMES } = require(tmp);

let failed = 0;
function check(name, cond, detail) {
  console.log(`${cond ? "PASS" : "FAIL"}  ${name}${detail ? "  → " + detail : ""}`);
  if (!cond) failed++;
}

// 通用驱动：按 10ms 步进推进 dt 毫秒，每步喂雷达模型
function drive(core, radar, dt, input, noise = false) {
  const steps = Math.round(dt / 10);
  for (let i = 0; i < steps; i++) {
    const p = input ? input(i * 10) : undefined;
    radar.tick(10, noise, p);
    core.step(10);
  }
}

// ── 场景 1：凑近 → APPROACH 触发，停下后回到 IDLE ──
(function () {
  const core = createSimCore();
  const radar = createRadarModel(core);
  const seen = new Set();
  core.onLog = (tag, msg) => { if (tag === "st") seen.add(msg.split(",")[2]); };
  // 2s 远处静止（雷达开机稳定）
  drive(core, radar, 2000, () => ({ x: 0, y: 2000 }));
  // 2.5s 内从 2m 走近到 0.55m（v≈-580mm/s）
  drive(core, radar, 2500, (dt) => {
    const p = Math.min(1, dt / 2500);
    return { x: 0, y: 2000 - 1450 * p };
  });
  // 4s 停在近处
  drive(core, radar, 4000, () => ({ x: 0, y: 550 }));
  check("靠近触发 APPROACH", seen.has("APPROACH"), `seen=[${[...seen]}]`);
  check("停下后回到 IDLE/ACTIVE", seen.has("IDLE") || seen.has("ACTIVE"));
})();

// ── 场景 2：静坐 + 近距强噪声 90s，新判定不横跳，旧判定必横跳 ──
// 对照组还原修复前的历史条件：旧固件处理速率 ≈7.6Hz → 2s 窗口凑不满 20 帧，
// ML 永远不确认（真机上 ml_stable 一直是 255），规则路径的单帧硬阈值直接暴露在噪声下。
function flickerTest(legacy) {
  const core = createSimCore();
  const radar = createRadarModel(core, legacy ? 132 : 67);
  core.legacy = legacy;
  let activeEntries = 0, approachEvents = 0, prevState = null;
  core.onLog = (tag, msg) => {
    if (tag !== "st") return;
    const st = msg.split(",")[2];
    if (st === "ACTIVE" && prevState !== "ACTIVE") activeEntries++;
    if (st === "APPROACH") approachEvents++;
    prevState = st;
  };
  drive(core, radar, 1500, () => ({ x: 0, y: 700 }));
  drive(core, radar, 90000, () => ({ x: 0, y: 700 }), true);   // 喂真实位置=静止，噪声由雷达模型注入
  return { activeEntries, approachEvents };
}
(function () {
  const fixed = flickerTest(false);
  // 断言放宽到 3：噪声下 ML 偶发确认 LATERAL 也会进 ACTIVE（真实固件行为，
  // 真实数据 v 尖峰占 0.7-3.1%，900ms 确认会漏过极少量）；重点是远低于对照组。
  check("静坐 90s + σ≈100mm 噪声：新判定几乎不进 ACTIVE", fixed.activeEntries <= 3,
    `ACTIVE 进入 ${fixed.activeEntries} 次, APPROACH ${fixed.approachEvents} 次`);
  const legacy = flickerTest(true);
  check("同条件旧版单帧判定确实横跳（对照）", legacy.activeEntries >= 3,
    `ACTIVE 进入 ${legacy.activeEntries} 次`);
})();

// ── 场景 3：离开 → GOODBYE → 睡前过渡链（哈欠→揉眼→渐暗）→ 入睡 ──
(function () {
  const core = createSimCore();
  const radar = createRadarModel(core);
  const events = [], evs = [];
  core.onLog = (tag, msg) => {
    if (tag === "st") events.push([core.t, msg.split(",")[2]]);
    if (tag === "ev") evs.push([core.t, msg.split(",")[2]]);
  };
  drive(core, radar, 3000, () => ({ x: 0, y: 900 }));
  drive(core, radar, 8600, () => ({ x: 0, y: 900, present: false }));
  const goodbye = events.find(e => e[1] === "GOODBYE");
  const yawn = evs.find(e => e[1] === "YAWN");
  const rub = evs.find(e => e[1] === "RUB");
  const dim = evs.find(e => e[1] === "DIM");
  const sleep = events.find(e => e[1] === "SLEEP");
  check("离场后进入 GOODBYE", !!goodbye, goodbye ? `@${goodbye[0]}ms` : "");
  check("无人 2.5s 后开始打哈欠", !!yawn && yawn[0] >= 5200 && yawn[0] <= 6300,
    yawn ? `@${yawn[0]}ms` : "");
  check("随后揉眼", !!rub && rub[0] >= 7200 && rub[0] <= 8300, rub ? `@${rub[0]}ms` : "");
  check("随后渐暗", !!dim && dim[0] >= 8700 && dim[0] <= 9800, dim ? `@${dim[0]}ms` : "");
  check("渐暗结束转入 SLEEP（链时序 ≈ last_seen+7.2s）",
    !!sleep && sleep[0] >= 9800 && sleep[0] <= 10800, sleep ? `@${sleep[0]}ms` : "");
})();

// ── 场景 3b：睡前链被打断 —— 哈欠打到一半人回来 → 事件清空 → WAKING ──
(function () {
  const core = createSimCore();
  const radar = createRadarModel(core);
  drive(core, radar, 3000, () => ({ x: 0, y: 900 }));
  drive(core, radar, 3600, () => ({ x: 0, y: 900, present: false })); // 哈欠已开始
  const evDuring = core.evName;
  drive(core, radar, 800, () => ({ x: 0, y: 900 }));                  // 人回来
  check("哈欠期间检测到事件", evDuring === "YAWN", `ev=${evDuring}`);
  check("人回来后事件立刻清空", core.ev === 0, `ev=${core.evName}`);
  check("打断后进入 WAKING", core.state === core.HM ? false : core.state === 2,
    `state=${core.STATE_NAMES[core.state]}`);
})();

// ── 场景 4：TinyML 决策树移植对拍 ──
(function () {
  const core = createSimCore();
  const radar = createRadarModel(core);
  drive(core, radar, 5000, () => ({ x: 0, y: 900 })); // es≈70, v≈0 → STILL（2s 窗口 + 900ms 确认）
  check("TinyML: 静坐 → STILL", core.mlStable === 1, `mlStable=${core.mlStable}`);

  const core2 = createSimCore();
  const radar2 = createRadarModel(core2);
  drive(core2, radar2, 5000, (dt) => ({ x: Math.sin(dt / 250 * Math.PI * 2) * 60, y: 900, v: 20 }));
  // es 派生：移动时 em>0、es≈70；v=20 → mean_abs_v>2.5 且 max≤35.5 → LATERAL
  check("TinyML: 摆动 → LATERAL(2)", core2.mlStable === 2, `mlStable=${core2.mlStable}`);
})();

// ── 场景 5：渲染冒烟测试 ──
(function () {
  const core = createSimCore();
  const radar = createRadarModel(core);
  drive(core, radar, 2000, () => ({ x: 300, y: 800 }));
  const fb = createFB();
  renderFace(fb, core, true);
  let lit = 0;
  for (let i = 0; i < fb.fb.length; i++) if (fb.fb[i]) lit++;
  check("OLED 帧缓冲有内容", lit > 200 && lit < 6000, `lit=${lit}/8192`);
  check("眼睛画在合理区域", fb.fb[23 * 128 + 38] === 1 || fb.fb[23 * 128 + 90] === 1);
})();

console.log(failed === 0 ? "\n全部通过 ✅" : `\n${failed} 项失败 ❌`);
process.exit(failed === 0 ? 0 : 1);

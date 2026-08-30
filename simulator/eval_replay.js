// HappyMac 评测：把真实采集 CSV 按原始时间轴回放进固件逻辑，
// 逐帧对比"固件状态 vs 采集时真实动作标签"。
// 用法：node eval_replay.js [会话CSV路径 ...]（缺省自动扫描 training/sessions）
const fs = require("fs");
const path = require("path");
const os = require("os");

// ── 固定种子，保证可复现 ──
let seed = 20260829;
Math.random = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296; };

const html = fs.readFileSync(path.join(__dirname, "happymac_sim.html"), "utf8");
const tmp = path.join(os.tmpdir(), "happymac_eval_core.js");
fs.writeFileSync(tmp, html.match(/<script>([\s\S]*)<\/script>/)[1]);
const { FW, createSimCore, STATE_NAMES } = require(tmp);

// ── 动作 → 期望固件状态 ──
// 静止类：只允许 IDLE（固件此时笑 = 误笑，重点指标）
// 运动类：允许 ACTIVE/IDLE（动作间停顿），fwd_back/stand_sit 额外允许 APPROACH/RETREAT
const STILL_ACTIONS = new Set(["still_30s", "natural_typing", "head_only"]);
const MAY_TRANSIENT = new Set(["fwd_back", "stand_sit"]);
function expectedSet(action) {
  if (STILL_ACTIONS.has(action)) return new Set(["IDLE"]);
  const s = new Set(["ACTIVE", "IDLE"]);
  if (MAY_TRANSIENT.has(action)) { s.add("APPROACH"); s.add("RETREAT"); }
  return s;
}

// ── 载入会话（优先 aligned_action 且 alignment_valid，其次 action）──
function loadSession(file) {
  const lines = fs.readFileSync(file, "utf8").split(/\r?\n/);
  const header = lines[0].split(",");
  const hasAligned = header.includes("aligned_action") && header.includes("alignment_valid");
  const rows = [];
  for (let i = 1; i < lines.length; i++) {
    const c = lines[i].split(",");
    if (c.length < 7) continue;
    const t = parseFloat(c[0]);
    if (!isFinite(t)) continue;
    let label = c[6];
    if (hasAligned) {
      const valid = c[header.indexOf("alignment_valid")];
      const aligned = c[header.indexOf("aligned_action")];
      if (valid === "True" && aligned && aligned !== "") label = aligned;
    }
    rows.push({
      t: t * 1000,
      x: parseFloat(c[1]), y: parseFloat(c[2]), v: parseFloat(c[3]),
      em: parseFloat(c[4]) || 0, es: parseFloat(c[5]) || 0,
      label,
    });
  }
  rows.sort((a, b) => a.t - b.t);
  return rows;
}

// ── 回放单个会话 ──
const SETTLE_MS = 2500;     // 动作段切换后的过渡宽限（WAKING/保持时间/ML确认）
const SKIP_START_MS = 3000; // 会话开头的 WAKING 阶段不计分
function replay(rows, legacy) {
  const core = createSimCore();
  core.legacy = legacy;
  const samples = [];
  let prevLabel = null, labelChangedAt = -1e9;
  let prevT = 0;
  for (const row of rows) {
    // 推进到本行时间；空窗(>400ms 无行)期间喂空帧（目标丢失）
    while (core.t < row.t - 5) {
      if (row.t - prevT > 400) core.ingest2450Empty();
      core.step(10);
    }
    core.ingest2450(Math.round(row.x), Math.round(row.y), Math.round(row.v));
    core.set2410(true, row.em, row.es);
    core.setIR(row.y < 1000);
    core.step(10); // 让 updateState 处理完这一帧
    prevT = core.t;
    if (row.label !== prevLabel) { prevLabel = row.label; labelChangedAt = core.t; }
    if (core.t < SKIP_START_MS) continue;
    if (core.t - labelChangedAt < SETTLE_MS) continue; // 过渡宽限期不计分
    const st = STATE_NAMES[core.state];
    if (st === "WAKING" || st === "GOODBYE" || st === "SLEEP") continue; // 存在性过渡不计分
    samples.push({ t: core.t, action: row.label, state: st, ml: core.mlStable });
  }
  return samples;
}

// ── 汇总 ──
function score(samples) {
  let total = 0, ok = 0, stillFrames = 0, stillBad = 0, stillBadViaML = 0;
  const perAction = {};
  for (const s of samples) {
    if (s.action === "free_5min") continue; // 自由段无期望状态，不计分
    total++;
    const exp = expectedSet(s.action);
    const hit = exp.has(s.state);
    if (hit) ok++;
    const pa = perAction[s.action] || (perAction[s.action] = { n: 0, ok: 0, states: {} });
    pa.n++; if (hit) pa.ok++;
    pa.states[s.state] = (pa.states[s.state] || 0) + 1;
    if (STILL_ACTIONS.has(s.action)) {
      stillFrames++;
      if (s.state !== "IDLE") {
        stillBad++;
        if (s.ml === 2) stillBadViaML++; // ML 确认 LATERAL 把状态拽进 ACTIVE
      }
    }
  }
  return { total, ok, stillFrames, stillBad, stillBadViaML, perAction,
           acc: total ? ok / total : 0, falseSmile: stillFrames ? stillBad / stillFrames : 0 };
}

// ── 嘴部笑容建模：与固件动画层同公式（进入延迟 + 最短保持），算"嘴真的在笑"的帧 ──
function smileStats(samples, enterMs = FW.SMILE_ENTER_MS || 0, minMs = FW.SMILE_MIN_MS || 2200) {
  const STILL = new Set(["still_30s", "natural_typing", "head_only"]);
  const MOVING = new Set(["slow_sweep", "quick_points", "ellipse", "natural_reach", "stand_sit", "fwd_back"]);
  let stillSmile = 0, stillN = 0, movN = 0, movReact = 0;
  let smileUntil = 0, activeSince = null;
  for (const s of samples) {
    if (s.action === "free_5min") continue;
    let smiling = false;
    if (s.state === "ACTIVE") {
      if (activeSince === null) activeSince = s.t;
      smiling = s.t - activeSince >= enterMs;
    } else if (s.state === "IDLE" || s.state === "APPROACH" || s.state === "RETREAT") {
      activeSince = null;
      smiling = s.t < smileUntil;
    } else { activeSince = null; smileUntil = 0; }
    if (smiling) smileUntil = s.t + minMs;
    if (STILL.has(s.action)) { stillN++; if (smiling) stillSmile++; }
    if (MOVING.has(s.action)) { movN++; if (s.state === "ACTIVE" || s.state === "APPROACH" || s.state === "RETREAT") movReact++; }
  }
  return { falseSmile: stillN ? stillSmile / stillN : 0, stillSmile, stillN,
           moveReact: movN ? movReact / movN : 0 };
}

// ── 参数扫描：小幅动作归静（压误笑），同时不许牺牲运动响应 ──
function sweep() {
  const MOV = new Set(["slow_sweep", "quick_points", "ellipse", "natural_reach", "stand_sit", "fwd_back"]);
  const sessions = files.slice(0, 5);
  console.log(`参数扫描（${sessions.length} 会话）：误笑率越低越好，运动响应率不能明显掉`);
  console.log("confirm  vTH  dTH  |  误笑率  运动响应  一致率");
  const rows = [];
  for (const confirm of [900, 1500, 2000]) {
    for (const vth of [15, 20]) {
      for (const dth of [130, 160]) {
        FW.ML_CONFIRM_MS = confirm; FW.MOTION_SPEED_TH = vth; FW.MOTION_DELTA_MM = dth;
        let smile = 0, stillN = 0, mov = 0, movN = 0, tot = 0, ok = 0;
        for (const f of sessions) {
          const s = replay(loadSession(f), false);
          const ss = smileStats(s, 600);
          smile += ss.stillSmile; stillN += ss.stillN;
          const sc = score(s); tot += sc.total; ok += sc.ok;
          for (const x of s) if (MOV.has(x.action)) {
            movN++;
            if (x.state === "ACTIVE" || x.state === "APPROACH" || x.state === "RETREAT") mov++;
          }
        }
        const r = { confirm, vth, dth, smile: smile / stillN, move: mov / movN, acc: ok / tot };
        rows.push(r);
        console.log(`${String(confirm).padStart(7)} ${String(vth).padStart(4)} ${String(dth).padStart(4)}  |  ` +
          `${(r.smile * 100).toFixed(1).padStart(5)}%  ${(r.move * 100).toFixed(1).padStart(7)}%  ${(r.acc * 100).toFixed(1).padStart(5)}%`);
      }
    }
  }
  const base = rows.find(r => r.confirm === 900 && r.vth === 15 && r.dth === 130);
  const candidates = rows.filter(r => r.move >= base.move - 0.05).sort((a, b) => a.smile - b.smile);
  console.log("\n候选（运动响应 ≥ 基线-5pp，按误笑率升序，前 5）:");
  for (const r of candidates.slice(0, 5))
    console.log(`  confirm=${r.confirm} vTH=${r.vth} dTH=${r.dth} → 误笑 ${(r.smile * 100).toFixed(1)}%，响应 ${(r.move * 100).toFixed(1)}%，一致率 ${(r.acc * 100).toFixed(1)}%`);
}

// ── 主流程 ──
// --deglitch 实验：把 v 里 8 的倍数尖峰（≥16，跟踪器毛刺）清零后重放，
// 用来验证"毛刺 v 喂进决策树导致误 LATERAL"的假设。
const DEGLITCH = process.argv.includes("--deglitch");
const SWEEP = process.argv.includes("--sweep");
// 实验旋钮：--floor N（v 噪声底） --confirm N（ML 确认时长）
function applyKnobs() {
  const gi = (name, dflt) => {
    const i = process.argv.indexOf(name);
    return i >= 0 ? parseFloat(process.argv[i + 1]) : dflt;
  };
  FW.V_NOISE_FLOOR = gi("--floor", FW.V_NOISE_FLOOR);
  FW.ML_CONFIRM_MS = gi("--confirm", FW.ML_CONFIRM_MS);
}
const dir = "/Users/chenmingyuan/Desktop/HappyMac/training/sessions";
const files = process.argv.slice(2).filter(a => a.endsWith(".csv"))
  .map(a => a.startsWith("/") ? a : path.join(dir, a));
if (files.length === 0) {
  for (const f of fs.readdirSync(dir))
    if (/^session_\d+_\d+\.csv$/.test(f)) files.push(path.join(dir, f));
}

applyKnobs();
if (SWEEP) { sweep(); process.exit(0); }

const agg = { cur: { total: 0, ok: 0, stillFrames: 0, stillBad: 0, stillBadViaML: 0 },
              old: { total: 0, ok: 0, stillFrames: 0, stillBad: 0, stillBadViaML: 0 } };
const perSession = [];

for (const f of files) {
  let rows = loadSession(f);
  if (!rows.length) continue;
  if (DEGLITCH) {
    let n = 0;
    for (const r of rows) {
      if (Math.abs(r.v) >= 16 && Math.abs(r.v) % 8 === 0) { r.v = 0; n++; }
    }
    console.log(`[deglitch] ${path.basename(f)}: 清零 ${n} 个 v 毛刺帧`);
  }
  const cur = score(replay(rows, false));
  const old = score(replay(rows, true));
  for (const k of ["total", "ok", "stillFrames", "stillBad", "stillBadViaML"]) { agg.cur[k] += cur[k]; agg.old[k] += old[k]; }
  perSession.push({ name: path.basename(f, ".csv"), rows: rows.length,
                    curAcc: cur.acc, oldAcc: old.acc,
                    curSmile: cur.falseSmile, oldSmile: old.falseSmile });
}

console.log("=== 会话级结果（固件当前逻辑 vs 旧版单帧判定）===");
console.log("会话                        帧数   一致率(新/旧)        静坐误笑率(新/旧)");
for (const s of perSession) {
  console.log(
    s.name.padEnd(26) + String(s.rows).padStart(6) +
    `   ${(s.curAcc * 100).toFixed(1).padStart(5)}% / ${(s.oldAcc * 100).toFixed(1).padStart(5)}%` +
    `   ${(s.curSmile * 100).toFixed(1).padStart(5)}% / ${(s.oldSmile * 100).toFixed(1).padStart(5)}%`);
}

console.log("\n=== 合计 ===");
console.log(`计分帧: ${agg.cur.total}`);
console.log(`总体一致率: ${(agg.cur.ok / agg.cur.total * 100).toFixed(1)}%（新） vs ${(agg.old.ok / agg.old.total * 100).toFixed(1)}%（旧）`);
console.log(`静坐误笑率（still/typing/head_only 中非 IDLE 帧）:`);
console.log(`  当前固件: ${(agg.cur.stillBad / agg.cur.stillFrames * 100).toFixed(2)}%  (${agg.cur.stillBad}/${agg.cur.stillFrames} 帧)`);
console.log(`    其中 ML 确认 LATERAL 驱动: ${agg.cur.stillBadViaML} 帧 (${(agg.cur.stillBadViaML / agg.cur.stillBad * 100).toFixed(0)}%)，规则路径驱动: ${agg.cur.stillBad - agg.cur.stillBadViaML} 帧`);
console.log(`  旧判定:   ${(agg.old.stillBad / agg.old.stillFrames * 100).toFixed(2)}%  (${agg.old.stillBad}/${agg.old.stillFrames} 帧)`);

// ── 分动作明细（当前固件，聚合所有会话）──
console.log("\n=== 分动作明细（当前固件，全部会话聚合）===");
const merged = {};
for (const f of files) {
  const rows = loadSession(f);
  for (const [action, pa] of Object.entries(score(replay(rows, false)).perAction)) {
    const m = merged[action] || (merged[action] = { n: 0, ok: 0, states: {} });
    m.n += pa.n; m.ok += pa.ok;
    for (const [st, c] of Object.entries(pa.states)) m.states[st] = (m.states[st] || 0) + c;
  }
}
console.log("动作            帧数    一致率   固件状态分布（前3）");
for (const [action, m] of Object.entries(merged).sort((a, b) => b[1].n - a[1].n)) {
  const top = Object.entries(m.states).sort((a, b) => b[1] - a[1]).slice(0, 3)
    .map(([st, c]) => `${st} ${(c / m.n * 100).toFixed(0)}%`).join(", ");
  console.log(action.padEnd(15) + String(m.n).padStart(6) + `   ${(m.ok / m.n * 100).toFixed(1).padStart(5)}%   ${top}`);
}

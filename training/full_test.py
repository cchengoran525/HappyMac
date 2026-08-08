#!/usr/bin/env python3
"""
HappyMac — 完整雷达基准测试（纯 ANSI）
- 上半屏：实时数据
- 下半屏：实验阶段 + 倒计时
- 自动导出 JSON
"""

import json
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
import numpy as np

RADAR_PORT = "/dev/cu.usbmodem2101"
RADAR_BAUD = 115200
OUT_DIR = Path(__file__).resolve().parent / "bench_results"
OUT_DIR.mkdir(parents=True, exist_ok=True)

PHASES = [
    {"name": "静止基线",     "key": "still",      "sec": 10,
     "desc": "坐定，正常呼吸，尽量不动"},
    {"name": "大幅左右移动", "key": "left_right",  "sec": 12,
     "desc": "左移(3s)→回中(3s)→右移(3s)→回中(3s)"},
    {"name": "前倾后靠",     "key": "fwd_back",    "sec": 12,
     "desc": "前倾凑近(3s)→正常(3s)→后靠(3s)→正常(3s)"},
    {"name": "转头测试",     "key": "head_turn",   "sec": 12,
     "desc": "正视(2s)→右转(3s)→正视(3s)→左转(3s)→正视。只转头，身不动！"},
    {"name": "自然使用",     "key": "natural",     "sec": 30,
     "desc": "随意用电脑。最后 5 秒站起来走开"},
]

BUF_SIZE = 60  # 显示缓冲


def radar_reader(port, baud, data_q, stop_evt):
    try:
        ser = serial.Serial(port, baud, timeout=0.3)
    except Exception as e:
        data_q.put({"error": str(e)})
        return
    while not stop_evt.is_set():
        try:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith("RADAR,"):
                p = line.split(",")
                if len(p) >= 9:
                    data_q.put({"x": int(p[2]), "y": int(p[3]), "v": int(p[4]),
                                "em": int(p[5]), "es": int(p[6])})
        except Exception:
            time.sleep(0.01)
    ser.close()


def clear():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def draw(buf_x, buf_y, buf_v, buf_em, buf_es, phase_idx, phase, status, color):
    """绘制 ANSI TUI"""
    # 颜色：G=绿 Y=黄 R=红 C=青 M=品红 W=白
    c = {"G": "\033[32m", "Y": "\033[33m", "R": "\033[31m",
         "C": "\033[36m", "M": "\033[35m", "W": "\033[37m", "X": "\033[0m"}

    lines = []
    lines.append(f"{c['C']}╔{'═'*58}╗{c['X']}")
    lines.append(f"{c['C']}║{c['X']} {c['C']}HappyMac 雷达基准测试{c['X']}" + " " * 37 + f"{c['C']}║{c['X']}")
    lines.append(f"{c['C']}╠{'═'*58}╣{c['X']}")

    # 数据行
    if buf_x:
        n_show = min(len(buf_x), 6)
        xs = buf_x[-n_show:]; ys = buf_y[-n_show:]
        vs = buf_v[-n_show:]; ems = buf_em[-n_show:]; ess = buf_es[-n_show:]
        lines.append(f"{c['C']}║{c['X']} {'X':>6} {'Y':>6} {'V':>5} {'Em':>5} {'Es':>5}  (最近{n_show}帧)" + " "*19 + f"{c['C']}║{c['X']}")
        for i in range(n_show-1, -1, -1):
            lines.append(f"{c['C']}║{c['X']} {xs[i]:+6d} {ys[i]:6d} {vs[i]:+5d} {ems[i]:5d} {ess[i]:5d}" + " "*22 + f"{c['C']}║{c['X']}")

        all_xs = buf_x[-40:] if len(buf_x) >= 40 else buf_x
        all_ys = buf_y[-40:] if len(buf_y) >= 40 else buf_y
        sx, sy = np.std(all_xs), np.std(all_ys)
        lines.append(f"{c['C']}║{c['X']} X: μ={np.mean(all_xs):+.0f} σ={sx:.0f} [{np.min(all_xs):+d}..{np.max(all_xs):+d}]" + " "*12 + f"{c['C']}║{c['X']}")
        lines.append(f"{c['C']}║{c['X']} Y: μ={np.mean(all_ys):.0f} σ={sy:.0f} [{np.min(all_ys)}..{np.max(all_ys)}]" + " "*16 + f"{c['C']}║{c['X']}")
    else:
        lines.append(f"{c['C']}║{c['X']} {c['Y']}(等待雷达数据...){c['X']}" + " " * 35 + f"{c['C']}║{c['X']}")

    lines.append(f"{c['C']}╠{'═'*58}╣{c['X']}")

    # 阶段指示
    total = len(PHASES)
    done = phase_idx
    bar_w = 46
    filled = int(bar_w * done / total) if total else 0
    bar = "█" * filled + "░" * (bar_w - filled)
    lines.append(f"{c['C']}║{c['X']} 阶段 {done}/{total} [{bar}]" + " "*4 + f"{c['C']}║{c['X']}")
    lines.append(f"{c['C']}║{c['X']} {c['M']}▸ {phase['name']}{c['X']}" + " "*(50-len(phase['name'])) + f"{c['C']}║{c['X']}")
    lines.append(f"{c['C']}║{c['X']}   {phase['desc']}" + " "*(48-len(phase['desc'])) + f"{c['C']}║{c['X']}")
    lines.append(f"{c['C']}║{c['X']}   时长: {phase['sec']}s" + " "*44 + f"{c['C']}║{c['X']}")
    lines.append(f"{c['C']}║{c['X']}" + " "*58 + f"{c['C']}║{c['X']}")

    # 状态大字
    status_color = c['G'] if color == 'G' else c['Y'] if color == 'Y' else c['R']
    pad = (58 - len(status)) // 2
    lines.append(f"{c['C']}║{c['X']}" + " "*pad + f"{status_color}{status}{c['X']}" + " "*(58-pad-len(status)) + f"{c['C']}║{c['X']}")

    lines.append(f"{c['C']}╚{'═'*58}╝{c['X']}")

    clear()
    print("\n".join(lines))
    sys.stdout.flush()


def countdown(sec, buf_x, buf_y, buf_v, buf_em, buf_es, phase_idx, phase):
    for i in range(sec, 0, -1):
        draw(buf_x, buf_y, buf_v, buf_em, buf_es, phase_idx, phase,
             f"  {i}  ", 'Y')
        time.sleep(1)


def run():
    data_q = queue.Queue(maxsize=200)
    stop_evt = threading.Event()
    reader = threading.Thread(target=radar_reader,
                              args=(RADAR_PORT, RADAR_BAUD, data_q, stop_evt), daemon=True)
    reader.start()
    time.sleep(1)

    # 检查串口错误
    try:
        err = data_q.get_nowait()
        if "error" in err:
            print(f"\033[31m串口错误: {err['error']}\033[0m")
            return
    except queue.Empty:
        pass

    buf_x, buf_y, buf_v, buf_em, buf_es = [], [], [], [], []
    all_sessions = {}

    for phase_idx, phase in enumerate(PHASES):
        # 清显示缓冲
        buf_x, buf_y, buf_v, buf_em, buf_es = [], [], [], [], []
        phase_data = []

        # 倒计时
        countdown(3, buf_x, buf_y, buf_v, buf_em, buf_es, phase_idx, phase)

        # 开始
        t_start = time.time()
        t_end = t_start + phase["sec"]

        while time.time() < t_end:
            # 读数据
            while True:
                try:
                    d = data_q.get_nowait()
                    if "error" in d: continue
                    buf_x.append(d["x"]); buf_y.append(d["y"])
                    buf_v.append(d["v"]); buf_em.append(d["em"])
                    buf_es.append(d["es"])
                    phase_data.append(d)
                    if len(buf_x) > BUF_SIZE:
                        buf_x = buf_x[-BUF_SIZE:]; buf_y = buf_y[-BUF_SIZE:]
                        buf_v = buf_v[-BUF_SIZE:]; buf_em = buf_em[-BUF_SIZE:]
                        buf_es = buf_es[-BUF_SIZE:]
                except queue.Empty:
                    break

            remaining = max(0, int(t_end - time.time()))
            draw(buf_x, buf_y, buf_v, buf_em, buf_es, phase_idx, phase,
                 f"▶ 进行中  剩余 {remaining}s", 'G')
            time.sleep(0.1)

        # 完成
        all_sessions[phase["key"]] = phase_data
        draw(buf_x, buf_y, buf_v, buf_em, buf_es, phase_idx, phase,
             "✓ 本阶段完成！", 'G')
        time.sleep(1.5)

        # 阶段间休息
        if phase_idx < len(PHASES) - 1:
            nx = PHASES[phase_idx + 1]
            draw(buf_x, buf_y, buf_v, buf_em, buf_es, phase_idx + 1, nx,
                 "准备下一阶段...", 'Y')
            time.sleep(2)

    stop_evt.set()
    reader.join(timeout=1)

    # ── 导出 ──
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    export = {"_meta": {"timestamp": ts, "phases": []}}

    for pk, data in all_sessions.items():
        if not data: continue
        xs = [d["x"] for d in data]; ys = [d["y"] for d in data]
        vs = [d["v"] for d in data]
        ems = [d["em"] for d in data]; ess = [d["es"] for d in data]
        export[pk] = {
            "n": len(data),
            "x": {"mean": float(np.mean(xs)), "std": float(np.std(xs)),
                  "min": int(np.min(xs)), "max": int(np.max(xs)),
                  "span": int(np.max(xs)-np.min(xs))},
            "y": {"mean": float(np.mean(ys)), "std": float(np.std(ys)),
                  "min": int(np.min(ys)), "max": int(np.max(ys)),
                  "span": int(np.max(ys)-np.min(ys))},
            "v": {"mean_abs": float(np.mean(np.abs(vs)))},
            "em": {"mean": float(np.mean(ems)), "max": int(np.max(ems))},
            "es": {"mean": float(np.mean(ess)), "max": int(np.max(ess))},
        }
        export["_meta"]["phases"].append({
            "key": pk, "n": len(data),
            "x_span": export[pk]["x"]["span"],
            "x_std": round(export[pk]["x"]["std"], 1),
            "y_span": export[pk]["y"]["span"],
        })

    out_path = OUT_DIR / f"fulltest_{ts}.json"
    with open(out_path, "w") as f:
        json.dump(export, f, indent=2)

    # ── 总结 ──
    clear()
    print(f"\033[36m{'='*60}\033[0m")
    print(f"\033[36m  测试完成！\033[0m")
    print(f"\033[36m{'='*60}\033[0m")
    print(f"\n{'阶段':<18} {'帧':>5} {'Xσ':>7} {'Xspan':>7} {'Yσ':>7} {'Yspan':>7} {'Em':>7} {'Es':>7}")
    print("-"*75)
    for ph in export["_meta"]["phases"]:
        k = ph["key"]; d = export.get(k, {})
        if not d: continue
        print(f"{k:<18} {d['n']:>5} {d['x']['std']:7.1f} {d['x']['span']:+7.0f} "
              f"{d['y']['std']:7.1f} {d['y']['span']:7.0f} "
              f"{d['em']['mean']:7.1f} {d['es']['mean']:7.1f}")

    # 判断
    still = export.get("still", {})
    lr = export.get("left_right", {})
    fb = export.get("fwd_back", {})
    ht = export.get("head_turn", {})

    print(f"\n\033[36m{'='*60}\033[0m")
    print(f"\033[36m  关键指标\033[0m")
    print(f"\033[36m{'='*60}\033[0m")

    if still and lr:
        r = lr["x"]["span"] / max(still["x"]["span"], 1)
        print(f"\n  X 左右区分度: {r:.1f}x {'✅ 明显' if r>3 else '⚠️ 偏弱' if r>1.5 else '❌ 分不出'}")
        print(f"    静止X跨度={still['x']['span']}mm  vs  左右移动X跨度={lr['x']['span']}mm")
    if still and fb:
        r = fb["y"]["span"] / max(still["y"]["span"], 1)
        print(f"\n  Y 前后区分度: {r:.1f}x {'✅ 明显' if r>3 else '⚠️ 偏弱' if r>1.5 else '❌ 分不出'}")
    if still and ht:
        r = ht["es"]["max"] / max(still["es"]["max"], 1)
        print(f"\n  Es 转头响应: {r:.1f}x {'✅ 有变化' if r>1.5 else '⚠️ 无变化'}")

    print(f"\n\033[32m数据已保存: {out_path}\033[0m")


if __name__ == "__main__":
    run()

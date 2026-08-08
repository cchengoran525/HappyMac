#!/usr/bin/env python3
"""
HappyMac — 雷达 + 摄像头同步测试
- OLED 显示英文阶段提示 + 倒计时
- 电脑摄像头实时人脸检测
- 雷达数据同步采集 + JSON 导出
"""

import json, queue, threading, time, cv2, serial
import numpy as np
from pathlib import Path
from datetime import datetime

RADAR_PORT = "/dev/cu.usbmodem2101"
OUT_DIR = Path(__file__).resolve().parent / "bench_results"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ─── English phases (OLED font = ASCII only) ────────
PHASES = [
    ("still",      "STILL BASELINE",   10, "Sit still, breathe normally"),
    ("left_right", "LEFT <> RIGHT",    12, "Lean LEFT 3s -> center -> RIGHT 3s -> center"),
    ("fwd_back",   "FWD <> BACK",      12, "Lean FWD 3s -> normal -> BACK 3s -> normal"),
    ("head_turn",  "HEAD TURN",        12, "Face fwd(2s)->turn RIGHT(3s)->fwd(3s)->LEFT(3s)"),
    ("natural",    "NATURAL USE",      30, "Use computer normally. Stand up & leave last 5s"),
]

# ─── Simple body detection (no external models) ──────
# 直接用摄像头画面，用光流或帧差法感知运动
# 不做人脸检测——太慢且 API 不兼容
# 重点是看: 你动的时候雷达 X/Y 怎么变

def main():
    # ── Connect radar ──
    ser = serial.Serial(RADAR_PORT, 115200, timeout=0.3)
    time.sleep(1)
    data_q = queue.Queue(); stop = threading.Event()
    def reader():
        while not stop.is_set():
            try:
                l = ser.readline().decode(errors='replace').strip()
                if l.startswith('RADAR,'):
                    p = l.split(',')
                    if len(p) >= 9:
                        data_q.put({'x':int(p[2]),'y':int(p[3]),'v':int(p[4]),
                                     'em':int(p[5]),'es':int(p[6])})
            except: time.sleep(0.01)
    threading.Thread(target=reader, daemon=True).start()

    def oled(cmd):
        ser.write((cmd+'\n').encode()); time.sleep(0.05)

    # ── Camera ──
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("[cam] Webcam ready")

    all_data = {}
    oled("!CLEAR"); time.sleep(0.5)

    for pi, (key, name, dur, desc) in enumerate(PHASES):
        print(f"\n{'='*50}")
        print(f"[{pi+1}/{len(PHASES)}] {name}: {desc}")
        print(f"{'='*50}")

        # Countdown on OLED
        for cd in [3, 2, 1]:
            oled(f"!PHASE:{name},{cd}")
            print(f"  {cd}...")
            time.sleep(1)
        oled("!GO")
        print("  ▶ GO!")

        phase_data = []
        t0 = time.time()

        while time.time() - t0 < dur:
            # Camera
            ret, frame = cap.read()

            # Radar

            # Radar
            try:
                d = data_q.get(timeout=0.02)
                phase_data.append(d)
            except queue.Empty:
                pass

            # Display
            if ret:
                # Status bar
                remaining = max(0, int(dur - (time.time() - t0)))
                cv2.putText(frame, f"{name} {remaining}s",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                if phase_data:
                    d = phase_data[-1]
                    cv2.putText(frame, f"X:{d['x']:+d} Y:{d['y']}",
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)
                cv2.imshow("HappyMac Test (q=quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        oled("!DONE")

        if phase_data:
            xs=[d['x'] for d in phase_data]; ys=[d['y'] for d in phase_data]
            all_data[key] = {
                'name':name, 'n':len(phase_data),
                'x_mean':float(np.mean(xs)), 'x_std':float(np.std(xs)),
                'x_span':int(max(xs)-min(xs)),
                'y_mean':float(np.mean(ys)), 'y_std':float(np.std(ys)),
                'y_span':int(max(ys)-min(ys)),
                'em_mean':float(np.mean([d['em'] for d in phase_data])),
                'es_mean':float(np.mean([d['es'] for d in phase_data])),
            }
            print(f"  {len(phase_data)} frames  Xσ={np.std(xs):.0f}  Yσ={np.std(ys):.0f}")
        else:
            print(f"  NO DATA!")

        time.sleep(1.5)

    oled("!CLEAR")
    stop.set(); ser.close()
    cap.release(); cv2.destroyAllWindows()

    # ── Export ──
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = OUT_DIR / f"live_{ts}.json"
    with open(out_path, 'w') as f:
        json.dump({'_meta':{'ts':ts,'n_phases':len(all_data)}, **all_data},
                  f, indent=2, default=str)

    # ── Summary ──
    print(f"\n{'='*55}")
    print(f"  SUMMARY")
    print(f"{'='*55}")
    print(f"{'Phase':<16} {'Frames':>6} {'Xσ':>6} {'Xspan':>7} {'Yσ':>6} {'Yspan':>7}")
    print("-"*60)
    for k,d in all_data.items():
        print(f"{d['name']:<16} {d['n']:>6} {d['x_std']:6.1f} {d['x_span']:+7d} {d['y_std']:6.1f} {d['y_span']:7d}")
    print(f"\nSaved: {out_path}")

    still = all_data.get('still',{})
    lr = all_data.get('left_right',{})
    if still and lr:
        print(f"X L/R ratio: {lr['x_span']/max(still['x_span'],1):.1f}x")


if __name__ == '__main__':
    main()

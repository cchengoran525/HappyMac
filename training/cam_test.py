#!/usr/bin/env python3
"""雷达+摄像头同步录制测试。产出: video.mp4 + radar.csv"""

import queue, threading, time, cv2, serial, csv
import numpy as np
from pathlib import Path
from datetime import datetime

RADAR_PORT = "/dev/cu.usbmodem2101"
OUT_DIR = Path(__file__).resolve().parent / "bench_results"
OUT_DIR.mkdir(parents=True, exist_ok=True)

PHASES = [
    ("still",      "STILL",      8, "Sit still"),
    ("left_right", "LEFT<>RIGHT",12, "Lean LEFT 3s -> center -> RIGHT 3s"),
    ("fwd_back",   "FWD<>BACK",  12, "Lean FWD 3s -> normal -> BACK 3s"),
]

def main():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_path = OUT_DIR / f"cam_{ts}.mp4"
    radar_path = OUT_DIR / f"radar_{ts}.csv"

    # ── Radar ──
    ser = serial.Serial(RADAR_PORT, 115200, timeout=0.3)
    time.sleep(1)
    radar_q = queue.Queue(); stop = threading.Event()
    def rdr():
        while not stop.is_set():
            try:
                l = ser.readline().decode(errors='replace').strip()
                if l.startswith('RADAR,'):
                    p = l.split(',')
                    if len(p) >= 9:
                        radar_q.put({'t':time.time(),'x':int(p[2]),'y':int(p[3]),
                                      'v':int(p[4]),'em':int(p[5]),'es':int(p[6])})
            except: time.sleep(0.01)
    threading.Thread(target=rdr, daemon=True).start()

    # ── Camera ──
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 15
    writer = cv2.VideoWriter(str(video_path), fourcc, fps, (640, 480))

    # ── Radar CSV ──
    csv_f = open(radar_path, 'w', newline='')
    csv_w = csv.writer(csv_f)
    csv_w.writerow(['t_sec','x','y','v','em','es','phase'])

    def oled(cmd):
        ser.write((cmd+'\n').encode()); time.sleep(0.05)

    print("=== 摄像头+雷达同步录制 ===")
    print(f"视频: {video_path}")
    print(f"雷达: {radar_path}")
    print()

    all_radar = []
    t_global_start = time.time()

    for pi, (key, name, dur, desc) in enumerate(PHASES):
        print(f"[{pi+1}/{len(PHASES)}] {name}: {desc}")

        # OLED countdown
        for cd in [3, 2, 1]:
            oled(f"!PHASE:{name},{cd}")
            time.sleep(1)
        oled("!GO")

        t_phase_start = time.time()

        while time.time() - t_phase_start < dur:
            # Camera
            ret, frame = cap.read()
            if ret:
                t_rel = time.time() - t_phase_start
                # Overlay
                cv2.putText(frame, f"{name} {max(0,int(dur-t_rel))}s",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                # Latest radar
                try:
                    latest = None
                    while True:
                        latest = radar_q.get_nowait()
                        latest['phase'] = key
                        latest['t_global'] = time.time()
                        all_radar.append(latest)
                except queue.Empty:
                    pass
                if all_radar:
                    r = all_radar[-1]
                    cv2.putText(frame, f"X:{r['x']:+d} Y:{r['y']} V:{r['v']:+d}",
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)
                    cv2.putText(frame, f"Em:{r['em']} Es:{r['es']}",
                               (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)
                writer.write(frame)
                cv2.imshow("Radar+Camera (q=quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.05)

        oled("!DONE")
        print(f"  done")
        time.sleep(1)

    oled("!CLEAR")

    # ── Cleanup ──
    stop.set(); ser.close()
    cap.release(); writer.release(); csv_f.close()
    cv2.destroyAllWindows()

    # ── Write radar CSV ──
    with open(radar_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_global','x','y','v','em','es','phase'])
        for r in all_radar:
            w.writerow([f"{r['t_global']:.3f}", r['x'], r['y'], r['v'],
                        r['em'], r['es'], r['phase']])

    # ── Summary ──
    print(f"\n=== 结果 ===")
    print(f"视频: {video_path}  ({Path(video_path).stat().st_size//1024} KB)")
    print(f"雷达: {radar_path}  ({len(all_radar)} 帧)")

    # Print stats per phase
    for key, name, _, _ in PHASES:
        data = [r for r in all_radar if r['phase'] == key]
        if not data: continue
        xs = [r['x'] for r in data]; ys = [r['y'] for r in data]
        print(f"  {name:<14} {len(data):>4}f  X:{np.mean(xs):+.0f}+-{np.std(xs):.0f}  Y:{np.mean(ys):.0f}+-{np.std(ys):.0f}")

    print(f"\n回放: open {video_path}")
    print(f"分析: 看视频里你动的时候，画面上的 X/Y 数字怎么变的")

if __name__ == '__main__':
    main()

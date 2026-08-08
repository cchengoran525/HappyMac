#!/usr/bin/env python3
"""S3摄像头 + C3雷达 同步采集。电脑需连 HappyMac-S3 热点。离线可跑。"""

import queue, threading, time, cv2, serial, csv, urllib.request
import numpy as np
from pathlib import Path
from datetime import datetime

S3_URL = "http://192.168.4.1/stream"
RADAR_PORT = "/dev/cu.usbmodem2101"
OUT_DIR = Path(__file__).resolve().parent / "bench_results"
OUT_DIR.mkdir(parents=True, exist_ok=True)

PHASES = [
    ("still",      "STILL",      10, "Sit still, breathe normally"),
    ("left_right", "LEFT<>RIGHT",12, "Lean LEFT 3s->center->RIGHT 3s->center"),
    ("fwd_back",   "FWD<>BACK",  12, "Lean FWD 3s->normal->BACK 3s->normal"),
    ("natural",    "NATURAL",    30, "Use computer normally. Stand & leave last 5s"),
]

def main():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

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

    def oled(cmd):
        ser.write((cmd+'\n').encode()); time.sleep(0.05)

    # ── S3 MJPEG stream ──
    print(f"[S3] 连接 {S3_URL} ...")
    stream = urllib.request.urlopen(S3_URL, timeout=5)
    buf = b""
    print("[S3] 流已连接")

    # ── Radar CSV ──
    radar_path = OUT_DIR / f"s3radar_{ts}.csv"

    # ── Video ──
    video_path = OUT_DIR / f"s3cam_{ts}.mp4"
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(str(video_path), fourcc, 10, (640, 480))

    all_radar = []

    for pi, (key, name, dur, desc) in enumerate(PHASES):
        print(f"\n[{pi+1}/{len(PHASES)}] {name}: {desc}")

        for cd in [3, 2, 1]:
            oled(f"!PHASE:{name},{cd}")
            print(f"  {cd}...")
            time.sleep(1)
        oled("!GO")
        print("  ▶ GO!")

        t0 = time.time()
        while time.time() - t0 < dur:
            # Read MJPEG frame
            buf += stream.read(4096)
            a = buf.find(b'\xff\xd8')
            b_end = buf.find(b'\xff\xd9')
            frame = None
            if a != -1 and b_end != -1 and b_end > a:
                jpg = buf[a:b_end+2]
                buf = buf[b_end+2:]
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)

            # Drain radar
            try:
                while True:
                    r = radar_q.get_nowait()
                    r['phase'] = key
                    all_radar.append(r)
            except queue.Empty:
                pass

            # Display
            if frame is not None:
                h, w = frame.shape[:2]
                remaining = max(0, int(dur - (time.time() - t0)))
                cv2.putText(frame, f"{name} {remaining}s", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                if all_radar:
                    r = all_radar[-1]
                    cv2.putText(frame, f"X:{r['x']:+d} Y:{r['y']} V:{r['v']:+d}",
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    cv2.putText(frame, f"Em:{r['em']} Es:{r['es']}",
                               (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                writer.write(cv2.resize(frame, (640, 480)))
                cv2.imshow("S3+Radar (q=quit)", cv2.resize(frame, (640, 480)))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        oled("!DONE")

    oled("!CLEAR")
    stop.set(); ser.close(); writer.release()
    cv2.destroyAllWindows()

    # Write radar CSV
    with open(radar_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_global','x','y','v','em','es','phase'])
        for r in all_radar:
            w.writerow([f"{r['t']:.3f}", r['x'], r['y'], r['v'], r['em'], r['es'], r['phase']])

    print(f"\n✅ 完成")
    print(f"视频: {video_path}")
    print(f"雷达: {radar_path} ({len(all_radar)} 帧)")

    for key, name, _, _ in PHASES:
        data = [r for r in all_radar if r['phase'] == key]
        if not data: continue
        xs = [r['x'] for r in data]; ys = [r['y'] for r in data]
        print(f"  {name:<14} {len(data):>4}f  X:{np.mean(xs):+.0f}+-{np.std(xs):.0f}  Y:{np.mean(ys):.0f}+-{np.std(ys):.0f}")

if __name__ == '__main__':
    main()

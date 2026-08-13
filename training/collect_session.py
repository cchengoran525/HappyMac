#!/usr/bin/env python3
"""
HappyMac — 完整数据采集 session
S3 摄像头 + C3 双雷达同步录制
多种动作类型 · 自动标注 · 后期分析

用法：电脑连 HappyMac-S3 热点，C3 插 USB
     python collect_session.py
"""

import csv, json, queue, sys, threading, time, cv2, serial, urllib.request
import numpy as np
from pathlib import Path
from datetime import datetime

S3_URL    = "http://192.168.4.1/stream"
RADAR_PORT = "/dev/cu.usbmodem2101"
OUT_DIR   = Path(__file__).resolve().parent / "sessions"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ═══════════════════════════════════════════════════════
#  动作序列（仿雷达测试标准范式）
# ═══════════════════════════════════════════════════════
ACTIONS = [
    # 基线
    ("still_30s",       "STILL 30s",        30,
     "Sit perfectly still. Hands on lap. Breathe normally."),

    # 左右移动
    ("slow_sweep",      "SLOW SWEEP L<>R",  20,
     "SLOW continuous left-right sweep. ~5s per side. Smooth."),
    ("quick_points",    "QUICK L . R . L",  20,
     "Quick: LEFT(3s)→STOP(2s)→RIGHT(3s)→STOP(2s)→CENTER"),
    ("ellipse",         "ELLIPSE MOTION",   20,
     "Draw an ellipse with your upper body. Clockwise then CCW."),

    # 前后
    ("fwd_back",        "FWD <> BACK",      20,
     "Lean forward(3s)→normal(3s)→lean back(3s)→normal"),

    # 转头（身体不动）
    ("head_only",       "HEAD ONLY TURN",   20,
     "Body STILL. Turn head: R(3s)→fwd(3s)→L(3s)→fwd. Repeat."),

    # 自然使用
    ("natural_typing",  "TYPE NATURALLY",   45,
     "Type/code naturally. Look at screen, not at radar."),
    ("natural_reach",   "REACH & GRAB",     20,
     "Reach for phone/water. Grab things on desk. Lean around."),
    ("stand_sit",       "STAND <> SIT",     20,
     "Stand up(3s)→sit(3s)→stand→sit. Repeat."),

    # 自由
    ("free_5min",       "FREE 5 MIN",      300,
     "Do whatever you want. Use computer, stand, walk, leave, return."),
]

# ═══════════════════════════════════════════════════════

class SessionCollector:
    def __init__(self, ts):
        self.ts = ts
        self.radar_q = queue.Queue()
        self.stop = threading.Event()
        self.all_radar = []
        self.action_log = []

        # Radar serial — 禁用 DTR/RTS 防止误复位 C3
        self.ser = serial.Serial(RADAR_PORT, 115200, timeout=0.3)
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        time.sleep(1)
        self.sync_ts = time.time()
        self.ser.write(b'!SYNC\n')
        t_wait = time.time()
        while time.time() - t_wait < 3:
            l = self.ser.readline().decode(errors='replace').strip()
            if l.startswith('!SYNC,'):
                self.sync_ms = int(l.split(',')[1])
                print(f"[SYNC] C3 replies: ms={self.sync_ms}")
                break
        threading.Thread(target=self._radar_thread, daemon=True).start()

        # ── 雷达预热 3 分钟（冷启动漂移期，已实测确认）──
        print("\n[WARMUP] 雷达预热 180 秒（冷启动噪声期，请静置）")
        t_warm = time.time()
        while time.time() - t_warm < 180:
            remaining = 180 - int(time.time() - t_warm)
            print(f"\r[WARMUP] 剩余 {remaining:3d}s    ", end="", flush=True)
            time.sleep(1)
        print("\n[WARMUP] 完成，开始正式采集\n")

        # 清空预热期积压的数据
        while True:
            try: self.radar_q.get_nowait()
            except queue.Empty: break

        # S3 stream
        print(f"[S3] connecting {S3_URL}...")
        self.stream = urllib.request.urlopen(S3_URL, timeout=5)
        self.stream_buf = b""
        print("[S3] connected")

        # Video writer
        self.video_path = OUT_DIR / f"session_{ts}.mp4"
        self.writer = cv2.VideoWriter(
            str(self.video_path), cv2.VideoWriter_fourcc(*'mp4v'), 10, (640, 480))

        # Radar CSV
        self.csv_path = OUT_DIR / f"session_{ts}.csv"
        self.csv_f = open(self.csv_path, 'w', newline='')
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow(['t_global','x','y','v','em','es','action'])

        # Video timestamp log（每帧的墙钟时间戳）
        self.vid_ts_path = OUT_DIR / f"session_{ts}_vidts.csv"
        self.vid_ts_f = open(self.vid_ts_path, 'w')
        self.vid_ts_f.write("t_wallclock,action\n")

        self.t_start = time.time()

    def _radar_thread(self):
        while not self.stop.is_set():
            try:
                l = self.ser.readline().decode(errors='replace').strip()
                if l.startswith('RADAR,'):
                    p = l.split(',')
                    if len(p) >= 9:
                        self.radar_q.put({
                            't': time.time(),
                            'x': int(p[2]), 'y': int(p[3]), 'v': int(p[4]),
                            'em': int(p[5]), 'es': int(p[6])
                        })
            except: time.sleep(0.01)

    def oled(self, cmd):
        self.ser.write((cmd + '\n').encode())
        time.sleep(0.04)

    def _read_mjpeg(self):
        self.stream_buf += self.stream.read(4096)
        a = self.stream_buf.find(b'\xff\xd8')
        b = self.stream_buf.find(b'\xff\xd9')
        if a != -1 and b != -1 and b > a:
            jpg = self.stream_buf[a:b+2]
            self.stream_buf = self.stream_buf[b+2:]
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            return img, time.time()  # 墙钟时间戳！
        return None, None

    def run_action(self, key, name, duration, desc):
        print(f"\n{'='*55}")
        print(f"  {name}  ({duration}s)")
        print(f"  {desc}")
        print(f"{'='*55}")

        # Countdown on OLED
        for cd in [3, 2, 1]:
            self.oled(f"!PHASE:{name[:16]},{cd}")
            time.sleep(1)
        self.oled("!GO")

        t0 = time.time()
        action_data = []
        frame_count = 0

        while time.time() - t0 < duration:
            # MJPEG frame（带墙钟时间戳）
            frame, t_vid = self._read_mjpeg()

            # Drain radar（保留雷达自己的到达时间戳）
            radar_latest = None
            while True:
                try:
                    r = self.radar_q.get_nowait()
                    r['action'] = key
                    self.all_radar.append(r)
                    action_data.append(r)
                    radar_latest = r
                except queue.Empty:
                    break

            # Write radar CSV（用雷达到达时间，不是视频时间！）
            if radar_latest:
                self.csv_w.writerow([
                    f"{radar_latest['t'] - (self.sync_ts or self.t_start):.3f}",
                    radar_latest['x'], radar_latest['y'],
                    radar_latest['v'], radar_latest['em'], radar_latest['es'],
                    key
                ])

            # Write video timestamp log（墙钟时间）
            if frame is not None:
                self.vid_ts_f.write(f"{t_vid:.6f},{key}\n")

            # Display
            if frame is not None:
                elapsed = time.time() - t0
                remaining = max(0, int(duration - elapsed))
                h, w = frame.shape[:2]

                # Info bar
                cv2.rectangle(frame, (0, 0), (w, 55), (0, 0, 0), -1)
                cv2.putText(frame, f"{name} {remaining}s", (5, 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(frame, desc[:80], (5, 35),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
                if radar_latest:
                    cv2.putText(frame,
                        f"X:{radar_latest['x']:+d} Y:{radar_latest['y']} "
                        f"V:{radar_latest['v']:+d} Em:{radar_latest['em']} Es:{radar_latest['es']}",
                        (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                # X-bar
                if radar_latest:
                    bx = int(np.clip((radar_latest['x'] + 500) / 1000 * w, 10, w - 10))
                    cv2.rectangle(frame, (10, h - 40), (w - 10, h - 10), (40, 40, 40), -1)
                    cv2.rectangle(frame, (10, h - 40), (bx, h - 10), (0, 200, 0), -1)
                    cv2.line(frame, (w // 2, h - 40), (w // 2, h - 10), (255, 255, 255), 1)

                frame = cv2.resize(frame, (640, 480))
                self.writer.write(frame)
                cv2.imshow("HappyMac Session (q=quit)", frame)
                frame_count += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[user quit]")
                break

        self.oled("!DONE")

        # Stats
        if action_data:
            xs = [d['x'] for d in action_data]
            ys = [d['y'] for d in action_data]
            ems = [d['em'] for d in action_data]
            ess = [d['es'] for d in action_data]
            summary = {
                'action': key, 'name': name, 'duration': duration,
                'n_radar': len(action_data), 'n_video': frame_count,
                'x_mean': float(np.mean(xs)), 'x_std': float(np.std(xs)),
                'x_min': int(np.min(xs)), 'x_max': int(np.max(xs)),
                'y_mean': float(np.mean(ys)), 'y_std': float(np.std(ys)),
                'em_pct': sum(1 for e in ems if e > 0) / len(ems) * 100,
                'es_pct': sum(1 for e in ess if e > 0) / len(ess) * 100,
            }
            self.action_log.append(summary)
            print(f"  {len(action_data)} radar frames  "
                  f"X:{np.mean(xs):+.0f}±{np.std(xs):.0f}  "
                  f"Y:{np.mean(ys):.0f}±{np.std(ys):.0f}")
        else:
            print(f"  NO RADAR DATA!")

        time.sleep(2)

    def finish(self):
        self.oled("!CLEAR")
        self.stop.set()

        # Save all radar data（用雷达线程记录的到达时间戳）
        self.csv_f.close()
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_global', 'x', 'y', 'v', 'em', 'es', 'action'])
            t_ref = self.sync_ts or self.t_start
            for r in self.all_radar:
                w.writerow([
                    f"{r['t'] - t_ref:.3f}",
                    r['x'], r['y'], r['v'], r['em'], r['es'], r.get('action', '')
                ])

        self.vid_ts_f.close()
        self.writer.release()
        self.ser.close()
        cv2.destroyAllWindows()

        # Save summary
        summary_path = OUT_DIR / f"session_{self.ts}_summary.json"
        with open(summary_path, 'w') as f:
            json.dump({
                'timestamp': self.ts,
                's3_url': S3_URL,
                'radar_port': RADAR_PORT,
                'n_total_radar': len(self.all_radar),
                'actions': self.action_log
            }, f, indent=2)

        print(f"\n{'='*55}")
        print(f"  SESSION COMPLETE")
        print(f"{'='*55}")
        print(f"  视频: {self.video_path}")
        print(f"  雷达: {self.csv_path}")
        print(f"  摘要: {summary_path}")
        print(f"  总雷达帧: {len(self.all_radar)}")

        # Print action summary table
        print(f"\n{'Action':<20} {'Radar':>6} {'X_mean':>7} {'X_std':>6} {'Y_mean':>7} {'Y_std':>6} {'Em%':>5} {'Es%':>5}")
        print("-" * 75)
        for a in self.action_log:
            print(f"{a['name']:<20} {a['n_radar']:>6} {a['x_mean']:+7.0f} {a['x_std']:6.0f} "
                  f"{a['y_mean']:7.0f} {a['y_std']:6.0f} {a['em_pct']:5.0f} {a['es_pct']:5.0f}")


def main():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"\n{'='*55}")
    print(f"  HappyMac 完整采集 Session")
    print(f"  {ts}")
    print(f"{'='*55}")
    print(f"  S3:  {S3_URL}")
    print(f"  C3:  {RADAR_PORT}")
    print(f"  动作: {len(ACTIONS)} 个")
    total_sec = sum(a[2] for a in ACTIONS)
    print(f"  总时长: ~{total_sec//60}min {total_sec%60}s")
    print(f"\n  准备: WiFi 连 HappyMac-S3, C3 插 USB")
    print(f"  按 Enter 开始...")
    input()

    collector = SessionCollector(ts)

    try:
        for key, name, dur, desc in ACTIONS:
            collector.run_action(key, name, dur, desc)
    except KeyboardInterrupt:
        print("\n[interrupted]")
    finally:
        collector.finish()


if __name__ == '__main__':
    main()

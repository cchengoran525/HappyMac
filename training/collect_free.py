#!/usr/bin/env python3
"""
HappyMac — 自由数据采集（视觉 Teacher 自动打标签）

MediaPipe 人脸关键点自动标注，按键仅作手动修正：
  - 无人脸          → ABSENT (0)
  - 脸中心横移持续   → LATERAL (2)
  - 脸框持续变大     → APPROACH (3)
  - 脸框持续变小     → RETREAT (4)
  - 其他             → STILL (1)

手动覆盖键: 0=ABSENT 1=STILL 2=LATERAL 3=APPROACH 4=RETREAT
用法：电脑连 HappyMac-S3 热点，C3 插 USB
     python collect_free.py
"""

import csv, queue, threading, time, cv2, serial, urllib.request
import numpy as np
import mediapipe as mp
from pathlib import Path
from datetime import datetime

S3_URL = "http://192.168.4.1/stream"
RADAR_PORT = "/dev/cu.usbmodem2101"
OUT_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR.mkdir(parents=True, exist_ok=True)
DURATION_SEC = 300  # 5 分钟
WARMUP_SEC = 180    # 3 分钟预热

LABELS = {ord('0'): 'ABSENT', ord('1'): 'STILL', ord('2'): 'LATERAL',
          ord('3'): 'APPROACH', ord('4'): 'RETREAT'}
LABEL_MAP = {'ABSENT': 0, 'STILL': 1, 'LATERAL': 2, 'APPROACH': 3, 'RETREAT': 4}

ts = datetime.now().strftime("%Y%m%d_%H%M%S")

# ── MediaPipe Teacher（存在性 + 横向）──
model_path = str(OUT_DIR.parent / "face_landmarker_v2_with_blendshapes.task")
BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
landmarker = FaceLandmarker.create_from_options(FaceLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.VIDEO,
    num_faces=1, min_face_detection_confidence=0.5,
    min_tracking_confidence=0.5, output_face_blendshapes=False))

# ── MiDaS Teacher（纵深）──
import torch
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
midas.eval()
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
midas_transform = midas_transforms.small_transform
midas_last = 0  # 上次 MiDaS 推理时间

# ── 自动标签状态机 ──
auto_label = 'STILL'
manual_override = None  # 手动覆盖后 3 秒恢复自动
manual_until = 0
fcx_hist = []   # 脸中心 x 历史（MediaPipe）
depth_hist = [] # 脸部深度历史（MiDaS，越大越近）

def auto_classify(fcx=None, depth=None):
    """混合分类：MediaPipe 管存在+横向，MiDaS 管纵深"""
    global auto_label
    if fcx is None:
        auto_label = 'ABSENT'  # 无人脸
        fcx_hist.clear()
        return auto_label
    if depth is not None:
        fcx_hist.append(fcx); depth_hist.append(depth)
        if len(fcx_hist) > 30:
            fcx_hist.pop(0); depth_hist.pop(0)
    else:
        fcx_hist.append(fcx)
        if len(fcx_hist) > 30:
            fcx_hist.pop(0)
    if len(fcx_hist) < 8:
        return auto_label

    # 横向：近 1 秒脸中心位移
    dx = fcx_hist[-1] - fcx_hist[-8]
    # 纵深：近 1.5 秒 MiDaS 深度趋势（越大=越近）
    if len(depth_hist) >= 8:
        dd = depth_hist[-1] - depth_hist[-8]
        # MiDaS 相对深度归一化：用历史标准差做阈值
        dstd = np.std(depth_hist) + 1e-9
        dd_norm = dd / dstd
        if dd_norm > 0.6:
            auto_label = 'APPROACH'
        elif dd_norm < -0.6:
            auto_label = 'RETREAT'
        elif abs(dx) > 0.03:
            auto_label = 'LATERAL'
        else:
            auto_label = 'STILL'
    else:
        if abs(dx) > 0.03:
            auto_label = 'LATERAL'
        else:
            auto_label = 'STILL'
    return auto_label

# ── Radar ──
ser = serial.Serial(RADAR_PORT, 115200, timeout=0.3)
ser.setDTR(False); ser.setRTS(False)
time.sleep(1)
ser.write(b'!SYNC\n')
sync_ts = time.time()
t_wait = time.time()
while time.time() - t_wait < 3:
    l = ser.readline().decode(errors='replace').strip()
    if l.startswith('!SYNC,'):
        print(f"[SYNC] C3 ms={l.split(',')[1]}")
        break

radar_q = queue.Queue(); stop = threading.Event()
def rdr():
    while not stop.is_set():
        try:
            l = ser.readline().decode(errors='replace').strip()
            if l.startswith('RADAR,'):
                p = l.split(',')
                if len(p) >= 9:
                    radar_q.put({'t': time.time(), 'x': int(p[2]), 'y': int(p[3]),
                                 'v': int(p[4]), 'em': int(p[5]), 'es': int(p[6])})
        except: time.sleep(0.01)
threading.Thread(target=rdr, daemon=True).start()

# ── 预热 ──
print(f"[WARMUP] 雷达预热 {WARMUP_SEC}s（可以先去倒水）")
for i in range(WARMUP_SEC, 0, -1):
    print(f"\r[WARMUP] 剩余 {i:3d}s  ", end="", flush=True)
    time.sleep(1)
print("\n[WARMUP] 完成\n")
while True:
    try: radar_q.get_nowait()
    except queue.Empty: break

# ── S3 ──
print(f"[S3] connecting {S3_URL}...")
stream = urllib.request.urlopen(S3_URL, timeout=5)
stream_buf = b""
print("[S3] connected")

# ── 输出文件 ──
csv_path = OUT_DIR / f"session_{ts}_free.csv"
vid_ts_path = OUT_DIR / f"session_{ts}_free_vidts.csv"
video_path = OUT_DIR / f"session_{ts}_free.mp4"
label_path = OUT_DIR / f"session_{ts}_free_labels.csv"

csv_f = open(csv_path, 'w', newline=''); csv_w = csv.writer(csv_f)
csv_w.writerow(['t_global', 'x', 'y', 'v', 'em', 'es', 'label'])
vid_f = open(vid_ts_path, 'w'); vid_f.write("t_wallclock,label\n")
writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*'mp4v'), 10, (640, 480))
lab_f = open(label_path, 'w', newline=''); lab_w = csv.writer(lab_f)
lab_w.writerow(['t_global', 'label', 'source'])  # source: auto/manual

# ── 主循环 ──
current_label = 'STILL'
current_label_id = 1
lab_w.writerow([0.0, current_label_id, 'auto'])
print(f"\n{'='*55}")
print(f"  自由采集开始（{DURATION_SEC}s）——视觉自动打标签")
print(f"  手动覆盖: 0=离开 1=静止 2=左右晃 3=凑近 4=远离")
print(f"{'='*55}\n")

t0 = time.time()
all_radar = []
label_changes = 0
mp_frame_idx = 0

while time.time() - t0 < DURATION_SEC:
    # Radar
    while True:
        try:
            r = radar_q.get_nowait()
            r['label'] = current_label_id
            all_radar.append(r)
            csv_w.writerow([f"{r['t']-sync_ts:.3f}", r['x'], r['y'], r['v'],
                            r['em'], r['es'], current_label_id])
        except queue.Empty:
            break

    # MJPEG + 视觉 Teacher
    stream_buf += stream.read(4096)
    a = stream_buf.find(b'\xff\xd8'); b = stream_buf.find(b'\xff\xd9')
    frame = None
    if a != -1 and b != -1 and b > a:
        jpg = stream_buf[a:b+2]
        stream_buf = stream_buf[b+2:]
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            frame = cv2.flip(img, -1)  # 摄像头倒装，翻转 180°
            frame = cv2.resize(frame, (640, 480))

            # ── Teacher 自动标签 ──
            if manual_override is None or time.time() > manual_until:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                res = landmarker.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), mp_frame_idx)
                mp_frame_idx += 1
                if res.face_landmarks and len(res.face_landmarks) > 0:
                    lm = res.face_landmarks[0]
                    xs = [lm[i].x for i in range(len(lm))]
                    ys = [lm[i].y for i in range(len(lm))]
                    fcx = (min(xs)+max(xs))/2
                    # MiDaS 每 0.5s 跑一次（纵深基准）
                    depth = None
                    if time.time() - midas_last > 0.5:
                        with torch.no_grad():
                            ib = midas_transform(rgb)
                            pred = midas(ib)
                            pred = torch.nn.functional.interpolate(
                                pred.unsqueeze(1), size=rgb.shape[:2],
                                mode="bicubic", align_corners=False).squeeze()
                        dm = pred.cpu().numpy()
                        h, w = dm.shape
                        px = int(np.clip(fcx * w, 0, w-1))
                        py = int(np.clip(((min(ys)+max(ys))/2) * h, 0, h-1))
                        patch = dm[max(0,py-5):py+6, max(0,px-5):px+6]
                        depth = float(np.median(patch))
                        midas_last = time.time()
                    current_label = auto_classify(fcx, depth)
                else:
                    current_label = auto_classify(fcx=None)
                current_label_id = LABEL_MAP[current_label]
                lab_w.writerow([f"{time.time()-t0:.3f}", current_label_id, 'auto'])

            vid_f.write(f"{time.time():.6f},{current_label_id}\n")
            writer.write(frame)

    # 按键（手动覆盖）
    if frame is not None:
        remaining = max(0, int(DURATION_SEC - (time.time() - t0)))
        cv2.rectangle(frame, (0, 0), (640, 90), (0, 0, 0), -1)
        tag = "AUTO" if manual_override is None else "MANUAL"
        cv2.putText(frame, f"{current_label} ({tag})  {remaining}s", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        cv2.putText(frame, "0=ABSENT 1=STILL 2=LATERAL 3=APPROACH 4=RETREAT", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.imshow("Free Session (auto-label)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in LABELS:
            current_label = LABELS[key]
            current_label_id = LABEL_MAP[current_label]
            manual_override = key
            manual_until = time.time() + 3  # 3 秒后恢复自动
            label_changes += 1
            lab_w.writerow([f"{time.time()-t0:.3f}", current_label_id, 'manual'])
            print(f"  [{time.time()-t0:5.1f}s] MANUAL → {current_label}")
        elif key == ord('q'):
            print("\n[quit]")
            break

stop.set(); ser.close(); stream.close()
csv_f.close(); vid_f.close(); writer.release(); lab_f.close()
cv2.destroyAllWindows()

print(f"\n{'='*55}")
print(f"  完成")
print(f"{'='*55}")
print(f"  雷达:  {csv_path}  ({len(all_radar)} 帧)")
print(f"  视频:  {video_path}")
print(f"  标签:  {label_path}  ({label_changes} 次手动修正)")

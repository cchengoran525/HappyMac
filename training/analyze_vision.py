#!/usr/bin/env python3
"""摄像头+雷达联合分析：MediaPipe头部位姿 × 雷达X/Y/Es → 三张散点图"""
import csv, json, cv2, time
from pathlib import Path
from collections import defaultdict
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mediapipe as mp

SESSION_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR = Path(__file__).resolve().parent / "analysis"
import sys
SESSION_ID = sys.argv[1] if len(sys.argv) > 1 else "20260813_160414"

video_path = SESSION_DIR / f"session_{SESSION_ID}.mp4"
csv_path   = SESSION_DIR / f"session_{SESSION_ID}.csv"
sum_path   = SESSION_DIR / f"session_{SESSION_ID}_summary.json"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Load radar CSV ──
radar_rows = list(csv.DictReader(open(csv_path)))
radar_by_time = []
for r in radar_rows:
    radar_by_time.append({
        't': float(r['t_global']),
        'x': int(r['x']), 'y': int(r['y']),
        'v': int(r['v']), 'em': int(r['em']), 'es': int(r['es']),
    })
summary = json.load(open(sum_path))
action_names = {a['action']: a['name'] for a in summary['actions']}

print(f"雷达: {len(radar_by_time)} 帧")
print(f"视频: {video_path}")

# ── MediaPipe Face Landmarker ──
BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# Download model if needed
model_path = str(SESSION_DIR.parent / "face_landmarker_v2_with_blendshapes.task")
import urllib.request, os
if not os.path.exists(model_path):
    print("[mp] 下载模型...")
    url = "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/latest/face_landmarker.task"
    urllib.request.urlretrieve(url, model_path)

options = FaceLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.VIDEO,
    num_faces=1, min_face_detection_confidence=0.5,
    min_tracking_confidence=0.5, output_face_blendshapes=False)
landmarker = FaceLandmarker.create_from_options(options)

# ── Load video wall-clock timestamps ──
vid_ts_path = SESSION_DIR / f"session_{SESSION_ID}_vidts.csv"
if vid_ts_path.exists():
    vid_rows = list(csv.DictReader(open(vid_ts_path)))
    vid_ts = [float(r['t_wallclock']) for r in vid_rows]
    print(f"视频时间戳: {len(vid_ts)} 帧 (墙钟)")
else:
    vid_ts = None
    print("⚠️ 无 vidts 文件，退化为帧号/fps")

# ── Process video ──
cap = cv2.VideoCapture(str(video_path))
fps = cap.get(cv2.CAP_PROP_FPS)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print(f"视频: {total_frames}帧 @ {fps:.0f}fps")

face_data = []  # {t, head_x, head_y, head_z, head_yaw, face_cx}
frame_idx = 0

while True:
    ret, frame = cap.read()
    if not ret: break
    # Flip vertically (camera was upside down)
    frame = cv2.flip(frame, -1)

    # 用墙钟时间戳（优先）或帧号/fps（退化）
    if vid_ts is not None and frame_idx < len(vid_ts):
        t_elapsed = vid_ts[frame_idx] - vid_ts[0] if vid_ts else frame_idx / max(fps, 1)
    else:
        t_elapsed = frame_idx / max(fps, 1)

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    result = landmarker.detect_for_video(mp_image, frame_idx)
    frame_idx += 1

    if result.face_landmarks and len(result.face_landmarks) > 0:
        lm = result.face_landmarks[0]
        h, w = frame.shape[:2]
        # Face center (normalized 0-1)
        xs_face = [lm[i].x for i in range(len(lm))]
        ys_face = [lm[i].y for i in range(len(lm))]
        zs_face = [lm[i].z for i in range(len(lm))]
        fc_x = (min(xs_face) + max(xs_face)) / 2
        fc_y = (min(ys_face) + max(ys_face)) / 2
        fc_z = np.mean(zs_face)

        # Head yaw from nose-chin line
        nose = np.array([lm[1].x, lm[1].y])
        chin = np.array([lm[152].x, lm[152].y])
        left_eye = np.array([lm[33].x, lm[33].y])
        right_eye = np.array([lm[263].x, lm[263].y])
        eye_center = (left_eye + right_eye) / 2
        face_vec = nose - eye_center
        yaw = np.degrees(np.arctan2(face_vec[0], abs(face_vec[1]) + 0.001))

        # Head x: face center relative to image center (-1 to 1)
        head_x = (fc_x - 0.5) * 2.0
        head_z = fc_z  # depth proxy

        face_data.append({
            't': t_elapsed, 'frame': frame_idx,
            'head_x': head_x, 'head_z': head_z,
            'head_yaw': yaw, 'face_cx': fc_x, 'face_cy': fc_y,
        })

    if frame_idx % 200 == 0:
        print(f"  {frame_idx}/{total_frames} ({frame_idx/max(total_frames,1)*100:.0f}%) {len(face_data)} faces")

cap.release()
landmarker.close()
print(f"人脸检测: {len(face_data)}/{total_frames} ({len(face_data)/max(total_frames,1)*100:.0f}%)")

if len(face_data) < 10:
    print("❌ 人脸检测太少，检查视频方向/光照")
    raise SystemExit

# ── Align radar to video by timestamp ──
t_vid_start = face_data[0]['t']
t_vid_end = face_data[-1]['t']
t_radar_start = radar_by_time[0]['t']

# For each video frame, find closest radar reading within 200ms
aligned = []
for f in face_data:
    t_vid = f['t']
    # Find radar readings near this time
    t_radar_rel = t_vid - t_vid_start  # approximate radar time offset
    best = None; best_dt = 999
    for r in radar_by_time:
        dt = abs(r['t'] - t_radar_rel - t_radar_start)
        if dt < best_dt:
            best_dt = dt; best = r
        if dt < 0.01: break  # close enough
    if best and best_dt < 0.2:  # within 200ms
        aligned.append({**f, **best})

print(f"对齐: {len(aligned)} 对 (视频{len(face_data)} × 雷达{len(radar_by_time)})")

if len(aligned) < 50:
    print("❌ 时间对齐失败，数据太少")
    raise SystemExit

# ═══════════════════════════════════════════════════════
#  FIGURE 1: Y 轴验证 — head_z vs radar Y
# ═══════════════════════════════════════════════════════
print("绘图...")
fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))

# 1: Y validation
hz = [a['head_z'] for a in aligned]
ry = [a['y'] for a in aligned]
axes[0].scatter(hz, ry, c=np.arange(len(aligned)), cmap='viridis', s=2, alpha=0.5)
axes[0].set_xlabel('MediaPipe head_z (depth proxy)')
axes[0].set_ylabel('LD2450 Y (mm)')
# R²
r = np.corrcoef(hz, ry)[0, 1] if len(hz) > 2 else 0
axes[0].set_title(f'Y-axis Validation (r={r:.3f}, n={len(aligned)}) {"✅" if abs(r)>0.3 else "⚠️ weak"}')

# 2: X validation
hx = [a['head_x'] for a in aligned]
rx = [a['x'] for a in aligned]
axes[1].scatter(hx, rx, c=np.arange(len(aligned)), cmap='plasma', s=2, alpha=0.5)
axes[1].set_xlabel('MediaPipe head_x (normalized)')
axes[1].set_ylabel('LD2450 X (mm)')
r2 = np.corrcoef(hx, rx)[0, 1] if len(hx) > 2 else 0
axes[1].set_title(f'X-axis Validation (r={r2:.3f}) {"✅" if abs(r2)>0.2 else "⚠️ weak"}')

# 3: Es vs head_yaw
hy = [a['head_yaw'] for a in aligned]
es = [a['es'] for a in aligned]
axes[2].scatter(hy, es, c=np.arange(len(aligned)), cmap='coolwarm', s=3, alpha=0.5)
axes[2].set_xlabel('MediaPipe head_yaw (deg)')
axes[2].set_ylabel('LD2410C Es (stationary energy)')
r3 = np.corrcoef(hy, es)[0, 1] if len(hy) > 2 else 0
axes[2].set_title(f'Es vs Head Yaw (r={r3:.3f}) {"✅ orientation signal" if abs(r3)>0.15 else "⚠️ no signal"}')

plt.tight_layout()
fig.savefig(OUT_DIR / f'07_vision_vs_radar_{SESSION_ID}.png', dpi=150)
plt.close()

# ═══════════════════════════════════════════════════════
#  FIGURE 2: 时序对比 — 视频 head_x vs 雷达 X
# ═══════════════════════════════════════════════════════
fig, axes = plt.subplots(2, 1, figsize=(16, 7), sharex=True)

t_aligned = [a['t'] - aligned[0]['t'] for a in aligned]
axes[0].plot(t_aligned, [a['head_x'] for a in aligned], color='#FF6B6B', lw=0.8, alpha=0.8, label='Head X (camera)')
axes[0].set_ylabel('Head X (normalized)'); axes[0].set_title('Camera vs Radar: Lateral Position')
axes[0].legend(); axes[0].axhline(y=0, color='gray', ls='--', alpha=0.3)

axes[1].plot(t_aligned, [a['x'] for a in aligned], color='#4ECDC4', lw=0.8, alpha=0.8, label='Radar X (mm)')
axes[1].set_ylabel('Radar X (mm)'); axes[1].set_xlabel('Time (s)')
axes[1].legend(); axes[1].axhline(y=0, color='gray', ls='--', alpha=0.3)
# Correlation by 5-second windows
for ax in axes:
    ax.fill_between(t_aligned, ax.get_ylim()[0], ax.get_ylim()[1],
                     step='mid', alpha=0.03)

plt.tight_layout()
fig.savefig(OUT_DIR / f'08_camera_vs_radar_timeseries_{SESSION_ID}.png', dpi=150)
plt.close()

# ═══════════════════════════════════════════════════════
#  FIGURE 3: Head yaw over time vs Es
# ═══════════════════════════════════════════════════════
fig, ax1 = plt.subplots(figsize=(16, 5))
ax1.plot(t_aligned, [a['head_yaw'] for a in aligned], color='#FF6B6B', lw=0.8, alpha=0.8, label='Head Yaw (deg)')
ax1.set_ylabel('Head Yaw (deg)', color='#FF6B6B'); ax1.set_xlabel('Time (s)')
ax2 = ax1.twinx()
ax2.plot(t_aligned, [a['es'] for a in aligned], color='#4ECDC4', lw=0.6, alpha=0.5, label='Es (radar)')
ax2.set_ylabel('Es (stationary energy)', color='#4ECDC4')
ax1.set_title('Head Yaw (camera) vs Es (radar) — "Does turning head change radar reflection?"')
l1, b1 = ax1.get_legend_handles_labels(); l2, b2 = ax2.get_legend_handles_labels()
ax1.legend(l1 + l2, b1 + b2, fontsize=8)
plt.tight_layout()
fig.savefig(OUT_DIR / f'09_yaw_vs_es_{SESSION_ID}.png', dpi=150)
plt.close()

# ── Summary ──
print(f"\n✅ 3 张视觉对比图 -> {OUT_DIR}/")
print(f"  07_vision_vs_radar.png          三张散点图 (Y验证/X验证/Es朝向)")
print(f"  08_camera_vs_radar_timeseries.png 摄像头head_x vs 雷达X 时间序列")
print(f"  09_yaw_vs_es.png                转头角度 vs 雷达静止能量")
print(f"\n=== 关键相关性 ===")
print(f"  Y验证: r={r:.3f}  {'✅ 雷达Y能跟踪距离' if abs(r)>0.3 else '⚠️ 弱相关'}")
print(f"  X验证: r={r2:.3f} {'✅ 雷达X能跟踪横向位置' if abs(r2)>0.2 else '⚠️ 弱相关 — X轴不准是实锤'}")
print(f"  Es朝向: r={r3:.3f} {'✅ 静止能量能感知头部朝向！' if abs(r3)>0.15 else '⚠️ 无朝向信号 — 雷达看不到你转头'}")

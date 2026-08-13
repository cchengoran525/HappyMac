#!/usr/bin/env python3
"""MiDaS 单目深度 × 雷达 Y 相关性验证"""
import csv, json, cv2, numpy as np, time
from pathlib import Path
from collections import defaultdict
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import torch
import mediapipe as mp

SESSION_DIR = Path(__file__).resolve().parent / "sessions"
OUT_DIR = Path(__file__).resolve().parent / "analysis"
import sys
SESSION_ID = sys.argv[1] if len(sys.argv) > 1 else "20260813_160414"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Load radar ──
rows = list(csv.DictReader(open(SESSION_DIR / f"session_{SESSION_ID}.csv")))
radar_t = np.array([float(r['t_global']) for r in rows])
radar_y = np.array([int(r['y']) for r in rows])
radar_action = [r['action'] for r in rows]

# ── Load video timestamps ──
vid_rows = list(csv.DictReader(open(SESSION_DIR / f"session_{SESSION_ID}_vidts.csv")))
vid_t = np.array([float(r['t_wallclock']) for r in vid_rows])
vid_action = [r['action'] for r in vid_rows]

# ── MiDaS ──
print("[MiDaS] 加载模型（首次下载 ~70MB）...")
midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
midas.eval()
midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
transform = midas_transforms.small_transform
print("[MiDaS] OK")

# ── MediaPipe face (仅定位脸部中心) ──
model_path = str(SESSION_DIR.parent / "face_landmarker_v2_with_blendshapes.task")
BaseOptions = mp.tasks.BaseOptions
FaceLandmarker = mp.tasks.vision.FaceLandmarker
FaceLandmarkerOptions = mp.tasks.vision.FaceLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode
lm_options = FaceLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.VIDEO,
    num_faces=1, min_face_detection_confidence=0.5,
    min_tracking_confidence=0.5, output_face_blendshapes=False)
landmarker = FaceLandmarker.create_from_options(lm_options)

cap = cv2.VideoCapture(str(SESSION_DIR / f"session_{SESSION_ID}.mp4"))
total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print(f"视频: {total} 帧")

# 每 0.5s 采样一帧（10fps 视频 = 每 5 帧）
SAMPLE_EVERY = 5
depth_log = []  # (t_wallclock, depth_at_face, action)
fi = 0
while True:
    ret, frame = cap.read()
    if not ret: break
    if fi % SAMPLE_EVERY != 0:
        fi += 1; continue
    frame = cv2.flip(frame, -1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Face detection
    res = landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), fi)
    face_center = None
    if res.face_landmarks and len(res.face_landmarks) > 0:
        lm = res.face_landmarks[0]
        xs = [lm[i].x for i in range(len(lm))]
        ys = [lm[i].y for i in range(len(lm))]
        face_center = ((min(xs)+max(xs))/2, (min(ys)+max(ys))/2)

    # MiDaS depth（small_transform 已含 batch 维）
    with torch.no_grad():
        input_batch = transform(rgb)
        prediction = midas(input_batch)
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1), size=rgb.shape[:2], mode="bicubic", align_corners=False
        ).squeeze()
    depth_map = prediction.cpu().numpy()

    depth_val = None
    if face_center is not None:
        h, w = depth_map.shape
        px = int(np.clip(face_center[0] * w, 0, w-1))
        py = int(np.clip(face_center[1] * h, 0, h-1))
        # 取脸部中心 9x9 邻域中位数
        r = 5
        patch = depth_map[max(0,py-r):py+r+1, max(0,px-r):px+r+1]
        depth_val = float(np.median(patch))

    if fi-1 < len(vid_t) and depth_val is not None:
        depth_log.append((vid_t[fi-1], depth_val, vid_action[fi-1]))
    fi += 1
    if fi % 100 == 0:
        print(f"  {fi}/{total}")

cap.release(); landmarker.close()
print(f"深度采样: {len(depth_log)}")

# ── 对齐：组内归零，最近邻 ──
EXCLUDE = {'free_5min'}  # 自由段被意外截断，不参与分析
f_groups = defaultdict(list)
for t, d, a in depth_log:
    if a in EXCLUDE: continue
    f_groups[a].append((t, d))
r_groups = defaultdict(list)
for t, y, a in zip(radar_t, radar_y, radar_action):
    if a in EXCLUDE: continue
    r_groups[a].append((t, y))
for a in list(f_groups):
    t0 = f_groups[a][0][0]; f_groups[a] = [(t-t0, d) for t,d in f_groups[a]]
for a in list(r_groups):
    t0 = r_groups[a][0][0]; r_groups[a] = [(t-t0, y) for t,y in r_groups[a]]

print(f"\n{'Action':<18} {'n':>5} {'Depth-雷达Y r':>12}")
print("-"*40)
all_d = []; all_y = []
for a in sorted(f_groups):
    fd = f_groups[a]; rd = r_groups.get(a, [])
    if len(fd) < 10 or len(rd) < 10: continue
    rt = np.array([p[0] for p in rd]); ry = np.array([p[1] for p in rd])
    dl = []; yl = []
    for t, d in fd:
        idx = np.argmin(np.abs(rt - t))
        if abs(rt[idx]-t) < 0.3:
            dl.append(d); yl.append(ry[idx])
    if len(dl) < 10: continue
    r = np.corrcoef(dl, yl)[0,1]
    all_d.extend(dl); all_y.extend(yl)
    print(f"{a:<18} {len(dl):>5} {r:>+12.3f}")

if all_d:
    r_all = np.corrcoef(all_d, all_y)[0,1]
    print(f"\n全部: r={r_all:+.3f}")
    print(f"\n注意: MiDaS 输出'越近值越大'，雷达 Y '越近值越小'")
    print(f"→ 预期负相关。|r|>0.3 说明两路距离测量一致")

# 图
fig, ax = plt.subplots(figsize=(10, 6))
ax.scatter(all_d, all_y, s=3, alpha=0.5, c='steelblue')
ax.set_xlabel('MiDaS depth at face (higher = closer)')
ax.set_ylabel('Radar Y (mm)')
ax.set_title(f'MiDaS Depth vs Radar Y  (r={r_all:+.3f}, n={len(all_d)})')
plt.tight_layout(); fig.savefig(OUT_DIR / f'10_depth_vs_y_{SESSION_ID}.png', dpi=150); plt.close()
print(f"图: {OUT_DIR / f'10_depth_vs_y_{SESSION_ID}.png'}")

#!/usr/bin/env python3
"""
HappyMac — 数据采集脚本（Teacher-Student 跨模态蒸馏）

训练期架构：
  ┌──────────────────────┐   ┌──────────────────────┐
  │  S3 摄像头 (ESP32-CAM) │   │  C3 + 双雷达 (USB 串口) │
  │  拍人的脸和身体         │   │  24GHz 反射信号         │
  │  WiFi MJPEG 推流       │   │  USB CDC 吐 CSV        │
  └──────────┬───────────┘   └──────────┬───────────┘
             │                          │
             └────────┬─────────────────┘
                      ▼
              ┌───────────────┐
              │  collect.py   │
              │  (本脚本)      │
              │               │
              │  · MediaPipe   │
              │    Face Mesh   │
              │    468 关键点   │
              │    → 头部位姿  │
              │    → 注视方向  │
              │    → 脸部朝向  │
              │               │
              │  · 串口雷达    │
              │    LD2450 X/Y/V│
              │    LD2410C E/s │
              │               │
              │  · 时间对齐    │
              │    (±50ms)    │
              │               │
              │  → 输出 CSV    │
              └───────────────┘

用法：
  python collect.py                    # 默认参数
  python collect.py --duration 600     # 采集 10 分钟
  python collect.py --port /dev/cu.usbmodem2201  # 自定义串口
  python collect.py --no-display       # 不显示预览窗口（省 CPU）

输出：
  training/sessions/YYYY-MM-DD_HH-MM-SS.csv

每行格式：
  ts_video,ts_radar,
  x,y,v,em,es,d2410,pres,ir,          ← 雷达原始值
  head_x,head_y,head_z,               ← 头部 3D 位置 (m)
  head_yaw,head_pitch,head_roll,      ← 头部姿态角 (deg)
  face_bbox_x,face_bbox_y,            ← 脸部在画面位置 (归一化 0-1)
  face_bbox_w,face_bbox_h,            ← 脸部边界框尺寸 (归一化)
  left_eye_open,right_eye_open,       ← 眼睛开合度 (0-1)
  mouth_open                           ← 嘴巴开合度 (0-1)
"""

import argparse
import csv
import queue
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import serial
import urllib.request

# MediaPipe
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from mediapipe import solutions as mp_solutions

# ─── 配置 ────────────────────────────────────────────
from config import *


# ========================================================
#  辅助函数
# ========================================================

def download_model(path: str):
    """下载 MediaPipe Face Landmarker 模型"""
    if Path(path).exists():
        return
    print(f"[setup] 下载 MediaPipe 模型到 {path} ...")
    url = ("https://storage.googleapis.com/mediapipe-models/"
           "face_landmarker/face_landmarker/float16/latest/"
           "face_landmarker.task")
    urllib.request.urlretrieve(url, path)
    print("[setup] 下载完成")


def compute_head_pose(landmarks_3d, landmarks_2d, image_w, image_h):
    """
    从 MediaPipe Face Mesh 的 3D/2D 关键点计算头部姿态。
    返回 (yaw, pitch, roll) 角度（度），以及头部 3D 位置 (x, y, z) 米。
    """
    # 选 6 个稳定关键点做 PnP
    # nose_tip=1, chin=152, left_eye_outer=33, right_eye_outer=263,
    # left_mouth=61, right_mouth=291
    idxs = [1, 152, 33, 263, 61, 291]

    pts_3d = np.float32([[landmarks_3d[i].x, landmarks_3d[i].y, landmarks_3d[i].z]
                          for i in idxs])
    pts_2d = np.float32([[landmarks_2d[i].x * image_w, landmarks_2d[i].y * image_h]
                          for i in idxs])

    # 相机内参（假设典型 webcam，实际值不重要，比例对就行）
    focal = image_w
    center = (image_w / 2.0, image_h / 2.0)
    cam_matrix = np.array([[focal, 0, center[0]],
                            [0, focal, center[1]],
                            [0, 0, 1]], dtype=np.float64)

    dist_coeffs = np.zeros((4, 1))

    success, rvec, tvec = cv2.solvePnP(
        pts_3d, pts_2d, cam_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE)

    if not success:
        return None

    # 旋转向量 → 旋转矩阵 → 欧拉角
    rmat, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(rmat[0, 0] ** 2 + rmat[1, 0] ** 2)
    if sy > 1e-6:
        yaw   = np.degrees(np.arctan2( rmat[1, 0], rmat[0, 0]))
        pitch = np.degrees(np.arctan2(-rmat[2, 0], sy))
        roll  = np.degrees(np.arctan2( rmat[2, 1], rmat[2, 2]))
    else:
        yaw   = np.degrees(np.arctan2(-rmat[1, 2], rmat[1, 1]))
        pitch = np.degrees(np.arctan2(-rmat[2, 0], sy))
        roll  = 0.0

    # 头部 3D 位置（tvec 单位 = 相机焦距归一化 × 实际距离）
    head_x = tvec[0][0] / focal       # 米（近似）
    head_y = tvec[1][0] / focal
    head_z = tvec[2][0] / focal

    return {
        "head_x": head_x, "head_y": head_y, "head_z": head_z,
        "head_yaw": yaw, "head_pitch": pitch, "head_roll": roll,
    }


def eye_aspect_ratio(landmarks, eye_idxs):
    """计算眼睛开合度 EAR (Eye Aspect Ratio)"""
    pts = np.array([[landmarks[i].x, landmarks[i].y] for i in eye_idxs])
    # 垂直距离 / 水平距离
    v1 = np.linalg.norm(pts[1] - pts[5])
    v2 = np.linalg.norm(pts[2] - pts[4])
    h  = np.linalg.norm(pts[0] - pts[3])
    if h < 1e-6:
        return 0.0
    return (v1 + v2) / (2.0 * h)


def mouth_aspect_ratio(landmarks):
    """计算嘴巴开合度"""
    pts = np.array([[landmarks[i].x, landmarks[i].y]
                     for i in [13, 14, 78, 308]])
    v = np.linalg.norm(pts[0] - pts[1])
    h = np.linalg.norm(pts[2] - pts[3])
    if h < 1e-6:
        return 0.0
    return v / h


# ========================================================
#  MJPEG 流读取（独立线程）
# ========================================================

class MJPEGReader(threading.Thread):
    """从 ESP32-CAM MJPEG 流读取帧，放入队列"""

    def __init__(self, url: str, frame_q: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.url = url
        self.q = frame_q
        self.stop = stop_event
        self.fps = 0.0

    def run(self):
        reconnect_delay = 1.0
        while not self.stop.is_set():
            try:
                stream = urllib.request.urlopen(self.url, timeout=5)
                buf = b""
                t0 = time.time()
                frames = 0
                while not self.stop.is_set():
                    buf += stream.read(4096)
                    if not buf:
                        break

                    # MJPEG 帧分隔符
                    a = buf.find(b"\xff\xd8")  # SOI
                    b_end = buf.find(b"\xff\xd9")  # EOI
                    if a != -1 and b_end != -1 and b_end > a:
                        jpg = buf[a:b_end + 2]
                        buf = buf[b_end + 2:]

                        ts = time.time()
                        frame = cv2.imdecode(
                            np.frombuffer(jpg, dtype=np.uint8),
                            cv2.IMREAD_COLOR)
                        if frame is not None:
                            # 丢旧帧（队列只保留最新 2 帧）
                            while self.q.qsize() > 2:
                                try:
                                    self.q.get_nowait()
                                except queue.Empty:
                                    break
                            self.q.put((ts, frame))

                        frames += 1
                        if frames >= 30:
                            dt = time.time() - t0
                            self.fps = 30.0 / dt if dt > 0 else 0
                            t0 = time.time()
                            frames = 0

                stream.close()
            except Exception as e:
                if not self.stop.is_set():
                    print(f"[camera] 连接断开: {e}，{reconnect_delay:.0f}s 后重连...")
                    time.sleep(reconnect_delay)
                    reconnect_delay = min(reconnect_delay * 1.5, 10.0)
            reconnect_delay = 1.0


# ========================================================
#  串口读取（独立线程）
# ========================================================

class RadarReader(threading.Thread):
    """从 C3 USB 串口读取雷达 CSV，放入队列"""

    def __init__(self, port: str, baud: int, data_q: queue.Queue,
                 stop_event: threading.Event):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = data_q
        self.stop = stop_event
        self.samples = 0

    def run(self):
        while not self.stop.is_set():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.5)
                print(f"[radar] 已连接 {self.port}")
                while not self.stop.is_set():
                    line = ser.readline().decode(errors="replace").strip()
                    if line.startswith("RADAR,"):
                        ts = time.time()
                        # 格式: RADAR,ms,x,y,v,em,es,d,pres,ir
                        parts = line.split(",")
                        if len(parts) >= 10:
                            try:
                                data = {
                                    "ts_local": ts,
                                    "ms": int(parts[1]),
                                    "x": int(parts[2]), "y": int(parts[3]),
                                    "v": int(parts[4]),
                                    "em": int(parts[5]), "es": int(parts[6]),
                                    "d2410": int(parts[7]),
                                    "pres": int(parts[8]), "ir": int(parts[9]),
                                }
                                self.q.put(data)
                                self.samples += 1
                            except ValueError:
                                pass
                ser.close()
            except Exception as e:
                if not self.stop.is_set():
                    print(f"[radar] 串口错误: {e}，重试...")
                    time.sleep(2)
            time.sleep(1)


# ========================================================
#  MediaPipe 处理 + 时间对齐（主线程）
# ========================================================

def run_collection(duration: float, show_preview: bool, out_path: Path,
                   radar_port=RADAR_PORT, camera_url=CAMERA_URL):
    """主采集循环"""

    # ── 下载模型 ──
    download_model(MP_MODEL_PATH)

    # ── 队列 ──
    frame_q  = queue.Queue(maxsize=10)
    radar_q  = queue.Queue(maxsize=200)
    stop_evt = threading.Event()

    # ── 启动采集线程 ──
    camera = MJPEGReader(camera_url, frame_q, stop_evt)
    radar  = RadarReader(radar_port, RADAR_BAUD, radar_q, stop_evt)
    camera.start()
    radar.start()

    # ── 初始化 MediaPipe ──
    print("[mp] 加载 MediaPipe Face Landmarker ...")
    base_options = mp_python.BaseOptions(model_asset_path=MP_MODEL_PATH)
    options = mp_vision.FaceLandmarkerOptions(
        base_options=base_options,
        running_mode=mp_vision.RunningMode.VIDEO,
        num_faces=1,
        min_face_detection_confidence=MP_FACE_DETECT_CONFIDENCE,
        min_tracking_confidence=MP_FACE_TRACK_CONFIDENCE,
        output_face_blendshapes=True,
        output_facial_transformation_matrixes=False,
    )
    landmarker = mp_vision.FaceLandmarker.create_from_options(options)

    # ── 输出文件 ──
    out_path.parent.mkdir(parents=True, exist_ok=True)
    f = open(out_path, "w", newline="")
    writer = csv.writer(f)
    header = [
        "ts_video", "ts_radar_ms",
        "x", "y", "v", "em", "es", "d2410", "pres", "ir",
        "head_x", "head_y", "head_z",
        "head_yaw", "head_pitch", "head_roll",
        "face_bbox_x", "face_bbox_y", "face_bbox_w", "face_bbox_h",
        "left_eye_open", "right_eye_open", "mouth_open",
    ]
    writer.writerow(header)
    f.flush()

    # ── 主循环 ──
    print(f"[main] 开始采集，时长={duration}s，输出={out_path}")
    print("[main] 按 Ctrl+C 提前停止")

    t_start = time.time()
    frame_idx = 0
    row_count = 0
    radar_buffer = {}  # ms → data（缓存雷达数据，等待匹配）

    # MediaPipe 需要 VIDEO 模式的帧序号
    mp_frame_idx = 0

    try:
        while time.time() - t_start < duration:
            # 1. 从雷达队列消费最新数据
            while True:
                try:
                    r = radar_q.get_nowait()
                    radar_buffer[r["ms"]] = r
                    # 只保留最近 500 条（5 秒缓冲）
                    if len(radar_buffer) > 500:
                        oldest = min(radar_buffer.keys())
                        del radar_buffer[oldest]
                except queue.Empty:
                    break

            # 2. 从视频队列取一帧
            try:
                video_ts, frame = frame_q.get(timeout=0.5)
            except queue.Empty:
                continue

            frame_idx += 1
            h, w = frame.shape[:2]

            # 3. MediaPipe 推理
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = landmarker.detect_for_video(mp_image, mp_frame_idx)
            mp_frame_idx += 1

            # 4. 提取头部位姿
            head_data = {}
            if result.face_landmarks and len(result.face_landmarks) > 0:
                lm = result.face_landmarks[0]
                # 头部位姿（PnP）
                # MediaPipe Face Landmarker 返回归一化坐标
                # 构建 3D 和 2D 关键点
                idxs = [1, 152, 33, 263, 61, 291]
                try:
                    pts_3d = np.float32([[
                        mp_solutions.face_mesh_connections.FACEMESH_TESSELATION
                        # 实际上需要 model 的 3D 坐标
                        # 用通用 3D 人脸模型近似
                    ]])
                    # TODO: 替换为标准 3D 人脸模型
                except:
                    pass

                # 简化的姿态计算（从关键点几何关系推导）
                # 鼻尖 (1) 和下巴 (152) 的连线 → 近似 pitch
                nose_tip  = np.float32([lm[1].x, lm[1].y])
                chin      = np.float32([lm[152].x, lm[152].y])
                left_eye  = np.float32([lm[33].x, lm[33].y])
                right_eye = np.float32([lm[263].x, lm[263].y])

                # 眼睛中点
                eye_center = (left_eye + right_eye) / 2.0
                # 鼻尖到眼睛中点的向量（用于近似朝向）
                face_vec = nose_tip - eye_center

                # yaw 近似：鼻尖偏离两眼中心的水平分量
                yaw_approx = np.degrees(np.arctan2(face_vec[0], abs(face_vec[1]) + 0.001))
                # pitch 近似：鼻尖-下巴线与垂直线的夹角
                nose_chin_vec = chin - nose_tip
                pitch_approx = np.degrees(np.arctan2(
                    nose_chin_vec[1], abs(nose_chin_vec[0]) + 0.001)) - 90

                # 头部 z 近似：人脸 bbox 面积越大 = 越近
                xs = [lm[i].x for i in range(468)]
                ys = [lm[i].y for i in range(468)]
                zs = [lm[i].z for i in range(468)]
                bbox_w = max(xs) - min(xs)
                bbox_h = max(ys) - min(ys)
                head_z_approx = 1.0 / (bbox_w * bbox_h + 0.01)

                # 头部 x 近似：脸部中心偏离画面中心
                head_x_approx = ((min(xs) + max(xs)) / 2.0 - 0.5) * 2.0  # -1..1

                # 眼睛开合度
                left_ear = eye_aspect_ratio(lm,
                    [33, 160, 158, 133, 153, 144])   # 左眼 6 点
                right_ear = eye_aspect_ratio(lm,
                    [362, 385, 387, 263, 373, 380])  # 右眼 6 点

                # 嘴巴开合度
                mar = mouth_aspect_ratio(lm)

                # 脸框中心
                fc_x = (min(xs) + max(xs)) / 2.0
                fc_y = (min(ys) + max(ys)) / 2.0

                head_data = {
                    "head_x":     head_x_approx,
                    "head_y":     (fc_y - 0.5) * 2.0,
                    "head_z":     head_z_approx,
                    "head_yaw":   yaw_approx,
                    "head_pitch": pitch_approx,
                    "head_roll":  0.0,
                    "face_bbox_x": fc_x,
                    "face_bbox_y": fc_y,
                    "face_bbox_w": bbox_w,
                    "face_bbox_h": bbox_h,
                    "left_eye_open":  left_ear,
                    "right_eye_open": right_ear,
                    "mouth_open":     mar,
                }
            else:
                head_data = {
                    "head_x": 0, "head_y": 0, "head_z": 0,
                    "head_yaw": 0, "head_pitch": 0, "head_roll": 0,
                    "face_bbox_x": 0, "face_bbox_y": 0,
                    "face_bbox_w": 0, "face_bbox_h": 0,
                    "left_eye_open": 0, "right_eye_open": 0,
                    "mouth_open": 0,
                }

            # 5. 时间对齐：找最近的雷达帧
            if radar_buffer:
                radar_times = sorted(radar_buffer.keys())
                # video_ts 对应 C3 的 ms 时间戳需要对齐
                # 用 video_ts 对应雷达的本地采集时间
                # 实际对齐策略：取 video frame 时间前后 ±MAX_TIME_DELTA 内的雷达帧
                best_r = None
                for r in radar_buffer.values():
                    if abs(r["ts_local"] - video_ts) < MAX_TIME_DELTA:
                        best_r = r
                        break
                # 如果没有精确匹配，取最近的
                if best_r is None and radar_buffer:
                    best_r = min(radar_buffer.values(),
                                 key=lambda r: abs(r["ts_local"] - video_ts))
            else:
                best_r = None

            # 6. 写一行
            if best_r:
                row = [
                    f"{video_ts:.6f}", best_r["ms"],
                    best_r["x"], best_r["y"], best_r["v"],
                    best_r["em"], best_r["es"], best_r["d2410"],
                    best_r["pres"], best_r["ir"],
                ]
            else:
                row = [f"{video_ts:.6f}", 0,
                       0, 0, 0, 0, 0, 0, 0, 0]

            row += [
                f"{head_data['head_x']:.4f}",
                f"{head_data['head_y']:.4f}",
                f"{head_data['head_z']:.4f}",
                f"{head_data['head_yaw']:.2f}",
                f"{head_data['head_pitch']:.2f}",
                f"{head_data['head_roll']:.2f}",
                f"{head_data['face_bbox_x']:.4f}",
                f"{head_data['face_bbox_y']:.4f}",
                f"{head_data['face_bbox_w']:.4f}",
                f"{head_data['face_bbox_h']:.4f}",
                f"{head_data['left_eye_open']:.4f}",
                f"{head_data['right_eye_open']:.4f}",
                f"{head_data['mouth_open']:.4f}",
            ]
            writer.writerow(row)
            row_count += 1

            # 7. 预览
            if show_preview:
                # 在画面上叠加 info
                if result.face_landmarks:
                    # 画脸部轮廓
                    for conn in mp_solutions.face_mesh.FACEMESH_TESSELATION:
                        idx1, idx2 = conn
                        pt1 = (int(result.face_landmarks[0][idx1].x * w),
                               int(result.face_landmarks[0][idx1].y * h))
                        pt2 = (int(result.face_landmarks[0][idx2].x * w),
                               int(result.face_landmarks[0][idx2].y * h))
                        cv2.line(frame, pt1, pt2, (0, 255, 0), 1)

                # 状态栏
                status = (f"Frames:{row_count} "
                          f"Cam:{camera.fps:.0f}fps "
                          f"Radar:{radar.samples}spl")
                cv2.putText(frame, status, (10, h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.imshow("HappyMac Collect (q=quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n[main] 用户终止")
                    break

            # 每 100 帧打印进度
            if row_count % 100 == 0:
                elapsed = time.time() - t_start
                rate = row_count / max(elapsed, 1)
                print(f"[main] {row_count} 行 | {elapsed:.0f}s | ~{rate:.0f} 行/秒")

    except KeyboardInterrupt:
        print("\n[main] 用户中断")
    finally:
        stop_evt.set()
        f.close()
        cv2.destroyAllWindows()
        landmarker.close()

    elapsed = time.time() - t_start
    print(f"[main] 完成: {row_count} 行, {elapsed:.0f} 秒")
    print(f"[main] 输出: {out_path}")


# ========================================================
#  入口
# ========================================================

def main():
    parser = argparse.ArgumentParser(description="HappyMac 数据采集")
    parser.add_argument("--duration", type=float, default=3600,
                        help="采集时长（秒），默认 3600（1 小时）")
    parser.add_argument("--port", type=str, default=RADAR_PORT,
                        help="C3 雷达串口")
    parser.add_argument("--camera", type=str, default=CAMERA_URL,
                        help="S3 摄像头 URL")
    parser.add_argument("--out", type=str, default=None,
                        help="输出文件路径（默认自动生成时间戳文件名）")
    parser.add_argument("--no-display", action="store_true",
                        help="不显示预览窗口")
    args = parser.parse_args()

    if args.out:
        out_path = Path(args.out)
    else:
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_path = DATA_DIR / f"{stamp}.csv"

    run_collection(args.duration, not args.no_display, out_path,
                   radar_port=args.port, camera_url=args.camera)


if __name__ == "__main__":
    main()

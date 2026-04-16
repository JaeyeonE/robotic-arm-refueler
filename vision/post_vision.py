import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import requests
from datetime import datetime
import math
from ultralytics import YOLO

# 모델 다른 것으로 수정 필요
model = YOLO('refueling.pt')

# ─── Flask API 설정 ───
API_URL = "http://localhost:5000"
STATION_ID = 1          # 이 카메라가 연결된 스테이션 ID
POST_INTERVAL = 1.0     # 초 단위 전송 주기
last_post = 0

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

# ─── intrinsics 가져오기 ───
frames = pipeline.wait_for_frames()
aligned = align.process(frames)
depth_frame = aligned.get_depth_frame()
intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

# ─── 카메라 → 로봇 베이스 변환 ───
R_cam2robot = np.array([[-1,  0,  0],
                        [ 0,  0,  -1],
                        [ 0,  -1, 0]], dtype=np.float64)

t_cam2robot = np.array([400.0, 238.0, 110.0])  # mm

CONF_THRESHOLD = 0.70
SAVE_DIR = "captures"
os.makedirs(SAVE_DIR, exist_ok=True)

def get_handle_angle(roi):
    """ROI 내 검은 막대의 PCA 방향 → 가로 기�� 각도 (-90~90도)"""
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < 50:
        return None
    data = cnt.reshape(-1, 2).astype(np.float32)
    mean, eigenvectors, _ = cv2.PCACompute2(data, mean=None)
    vx, vy = eigenvectors[0]
    angle = math.degrees(math.atan2(vy, vx))
    if angle > 90:
        angle -= 180
    elif angle < -90:
        angle += 180
    return round(angle, 1)


def pixel_to_robot(u, v, depth_m):
    point_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m)
    point_cam_mm = np.array(point_cam) * 1000.0
    point_robot = R_cam2robot @ point_cam_mm + t_cam2robot
    return point_robot

last_print = time.time()

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        display_image = color_image.copy()
        results = model(color_image, verbose=False)

        now = time.time()
        should_print = (now - last_print) >= 1.0

        detected_objects = []

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRESHOLD:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = model.names[cls]

                u, v = (x1 + x2) // 2, (y1 + y2) // 2
                depth_val = depth_frame.get_distance(u, v)
                if depth_val <= 0.0:
                    continue
                robot_xyz = pixel_to_robot(u, v, depth_val)
                rx, ry, rz = robot_xyz

                # cap_handle → 회전 각도 계산
                handle_angle = None
                if label == "cap_handle":
                    pad = 8
                    rx1 = max(0, x1 - pad)
                    ry1 = max(0, y1 - pad)
                    rx2 = min(color_image.shape[1], x2 + pad)
                    ry2 = min(color_image.shape[0], y2 + pad)
                    roi = color_image[ry1:ry2, rx1:rx2]
                    if roi.size:
                        handle_angle = get_handle_angle(roi)

                detected_objects.append({
                    'label': label,
                    'conf': conf,
                    'bbox': (x1, y1, x2, y2),
                    'pixel': (u, v),
                    'depth': depth_val,
                    'robot_xyz': (rx, ry, rz),
                    'handle_angle': handle_angle,
                })

                # 바운딩 박스 + 정보 표시
                cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{label} {conf:.2f} | d={depth_val:.2f}m"
                cv2.putText(display_image, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                robot_text = f"R({rx:.1f}, {ry:.1f}, {rz:.1f})mm"
                cv2.putText(display_image, robot_text, (x1, y2 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

                if handle_angle is not None:
                    cv2.putText(display_image, f"angle={handle_angle:.1f}deg",
                                (x1, y2 + 35), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 255), 2)

                if should_print:
                    print(f"[{label}] conf={conf:.2f} | "
                          f"pixel=({u},{v}) | depth={depth_val:.3f}m | "
                          f"robot=({rx:.1f}, {ry:.1f}, {rz:.1f}) mm")

        if should_print:
            last_print = now

        # ─── Flask API POST ───
        if detected_objects and (now - last_post) >= POST_INTERVAL:
            for obj in detected_objects:
                try:
                    requests.post(f"{API_URL}/api/vision/detect", json={
                        'station_id': STATION_ID,
                        'label':      obj['label'],
                        'confidence': obj['conf'],
                        'bbox_x1':    obj['bbox'][0],
                        'bbox_y1':    obj['bbox'][1],
                        'bbox_x2':    obj['bbox'][2],
                        'bbox_y2':    obj['bbox'][3],
                        'center_u':   obj['pixel'][0],
                        'center_v':   obj['pixel'][1],
                        'depth':      obj['depth'],
                        'robot_x':    obj['robot_xyz'][0],
                        'robot_y':    obj['robot_xyz'][1],
                        'robot_z':    obj['robot_xyz'][2],
                        'handle_angle': obj['handle_angle'],
                    }, timeout=0.3)
                except Exception:
                    pass
            last_post = now

        # ─── 프레임 → Flask POST ───
        _, jpeg = cv2.imencode('.jpg', display_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        try:
            requests.post(f"{API_URL}/api/vision/frame/{STATION_ID}",
                          data=jpeg.tobytes(),
                          headers={'Content-Type': 'image/jpeg'},
                          timeout=0.3)
        except Exception:
            pass

        time.sleep(0.03)  # ~30fps 유지

finally:
    pipeline.stop()

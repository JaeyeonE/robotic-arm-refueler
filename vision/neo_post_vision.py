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


def pixel_to_robot(u, v, depth_m):
    point_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], depth_m)
    point_cam_mm = np.array(point_cam) * 1000.0
    point_robot = R_cam2robot @ point_cam_mm + t_cam2robot
    return point_robot


# ─── 영상 처리 함수 (업데이트) ───

def extract_handle_face_center(roi):
    """
    roi 안에서 cap_handle의 정면 긴 면(검은 막대)만 추출
    반환:
        center_2d : (cx, cy)  # roi 내부 좌표
        mask      : binary mask
        box       : cv2.boxPoints(rect)
    """
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None, None

    best_cnt = None
    best_score = -1

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 50:
            continue

        rect = cv2.minAreaRect(cnt)
        (_, _), (w, h), _ = rect

        short_side = min(w, h)
        long_side = max(w, h)

        if short_side < 3:
            continue

        aspect_ratio = long_side / short_side
        score = area * aspect_ratio

        if score > best_score:
            best_score = score
            best_cnt = cnt

    if best_cnt is None:
        return None, None, None

    rect = cv2.minAreaRect(best_cnt)
    (cx, cy), _, _ = rect
    box = cv2.boxPoints(rect)
    box = np.int32(box)

    mask = np.zeros(binary.shape, dtype=np.uint8)
    cv2.drawContours(mask, [best_cnt], -1, 255, -1)

    return (cx, cy), mask, box


def get_handle_angle_horizontal(roi):
    """
    roi 안에서 검은 막대 방향을 PCA로 계산
    가로 방향 = 0도
    결과 범위 = [-90, 90]
    """
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None

    cnt = max(contours, key=cv2.contourArea)

    if cv2.contourArea(cnt) < 50:
        return None, None

    data = cnt.reshape(-1, 2).astype(np.float32)
    mean, eigenvectors, _ = cv2.PCACompute2(data, mean=None)

    cx, cy = mean[0]
    vx, vy = eigenvectors[0]

    angle_deg = math.degrees(math.atan2(vy, vx))

    if angle_deg > 90:
        angle_deg -= 180
    elif angle_deg < -90:
        angle_deg += 180

    length = 50
    x1 = int(cx - vx * length)
    y1 = int(cy - vy * length)
    x2 = int(cx + vx * length)
    y2 = int(cy + vy * length)

    return angle_deg, ((x1, y1), (x2, y2))


def get_mask_3d_center(mask, x_offset, y_offset, depth_frame):
    """
    mask 내부의 유효 depth를 전부 3D로 변환해서 평균 중심 계산
    반환:
        center_robot : [x, y, z] mm
        center_pixel : (u_mean, v_mean)
        valid_count  : 유효 depth 점 개수
    """
    ys, xs = np.where(mask > 0)

    if len(xs) == 0:
        return None, None, 0

    robot_points = []
    pixels = []

    for x, y in zip(xs, ys):
        u = int(x + x_offset)
        v = int(y + y_offset)

        depth_m = depth_frame.get_distance(u, v)

        if depth_m <= 0:
            continue

        robot_xyz = pixel_to_robot(u, v, depth_m)
        robot_points.append(robot_xyz)
        pixels.append([u, v])

    if len(robot_points) == 0:
        return None, None, 0

    robot_points = np.array(robot_points, dtype=np.float64)
    pixels = np.array(pixels, dtype=np.float64)

    center_robot = np.mean(robot_points, axis=0)
    center_pixel = np.mean(pixels, axis=0)

    return center_robot, center_pixel, len(robot_points)


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

                # 바운딩 박스 + 정보 표시
                cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{label} {conf:.2f} | d={depth_val:.2f}m"
                cv2.putText(display_image, text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                robot_text = f"R({rx:.1f}, {ry:.1f}, {rz:.1f})mm"
                cv2.putText(display_image, robot_text, (x1, y2 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

                # cap_handle → 정면 면 추출 + 각도 + 3D 중심
                handle_angle = None
                face_robot_xyz = None
                if label == "cap_handle":
                    pad = 8
                    x1p = max(0, x1 - pad)
                    y1p = max(0, y1 - pad)
                    x2p = min(color_image.shape[1], x2 + pad)
                    y2p = min(color_image.shape[0], y2 + pad)
                    roi = color_image[y1p:y2p, x1p:x2p]

                    if roi.size:
                        # 1) 각도 계산
                        angle_result, line_pts = get_handle_angle_horizontal(roi)
                        if angle_result is not None:
                            handle_angle = angle_result
                            cv2.putText(display_image,
                                        f"angle={handle_angle:.1f}deg",
                                        (x1, y2 + 35),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                            if line_pts is not None:
                                (lx1, ly1), (lx2, ly2) = line_pts
                                cv2.line(display_image,
                                         (lx1 + x1p, ly1 + y1p),
                                         (lx2 + x1p, ly2 + y1p),
                                         (255, 0, 0), 2)

                        # 2) 정면 긴 면 추출 + 3D 중심
                        face_center_roi, face_mask, face_box = extract_handle_face_center(roi)

                        if face_center_roi is not None and face_mask is not None:
                            cx_roi, cy_roi = face_center_roi
                            cx_img = int(cx_roi + x1p)
                            cy_img = int(cy_roi + y1p)

                            cv2.circle(display_image, (cx_img, cy_img), 5, (0, 255, 255), -1)

                            if face_box is not None:
                                face_box_img = face_box + np.array([x1p, y1p])
                                cv2.drawContours(display_image, [face_box_img], 0, (255, 255, 0), 2)

                            cv2.putText(display_image,
                                        f"face2d=({cx_img},{cy_img})",
                                        (x1, y2 + 55),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                            # mask 내부 depth 평균으로 3D 중심
                            center_robot, center_pixel, valid_count = get_mask_3d_center(
                                face_mask, x1p, y1p, depth_frame
                            )

                            if center_robot is not None:
                                face_robot_xyz = tuple(center_robot)
                                frx, fry, frz = face_robot_xyz

                                cv2.putText(display_image,
                                            f"F({frx:.1f},{fry:.1f},{frz:.1f})",
                                            (x1, y2 + 75),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                                if should_print:
                                    print(f"[cap_handle_face_3d] "
                                          f"2d=({cx_img},{cy_img}) | "
                                          f"robot=({frx:.1f}, {fry:.1f}, {frz:.1f}) mm | "
                                          f"valid_depth={valid_count}")

                detected_objects.append({
                    'label': label,
                    'conf': conf,
                    'bbox': (x1, y1, x2, y2),
                    'pixel': (u, v),
                    'depth': depth_val,
                    'robot_xyz': (rx, ry, rz),
                    'handle_angle': handle_angle,
                    'face_robot_xyz': face_robot_xyz,
                })

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
                    payload = {
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
                    }
                    # face 3D 중심이 있으면 추가
                    if obj['face_robot_xyz'] is not None:
                        payload['face_robot_x'] = obj['face_robot_xyz'][0]
                        payload['face_robot_y'] = obj['face_robot_xyz'][1]
                        payload['face_robot_z'] = obj['face_robot_xyz'][2]

                    requests.post(f"{API_URL}/api/vision/detect", json=payload,
                                  timeout=0.3)
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

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
from datetime import datetime
from ultralytics import YOLO
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


class FuelPortPerceptionNode(Node):
    def __init__(self):
        super().__init__('fuel_port_perception_node')

        # -----------------------------
        # ROS parameters
        # -----------------------------
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('target_label', 'cap_handle')
        self.declare_parameter('conf_threshold', 0.70)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_label = self.get_parameter('target_label').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value

        # 고정 자세값
        self.fixed_rx = -90.0
        self.fixed_ry = 0.0
        self.fixed_rz = -90.0

        # -----------------------------
        # YOLO
        # -----------------------------
        self.model = YOLO(model_path)

        # -----------------------------
        # RealSense
        # -----------------------------
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        self.intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        # -----------------------------
        # Camera -> Robot transform
        # -----------------------------
        self.R_cam2robot = np.array(
            [
                [-1, 0, 0],
                [0, 0, -1],
                [0, -1, 0],
            ],
            dtype=np.float64,
        )

        self.t_cam2robot = np.array([420.0, 65.0, 80.0])  # mm

        self.save_dir = "captures"
        os.makedirs(self.save_dir, exist_ok=True)

        self.last_print = time.time()
        self.latest_best_target = None
        self.request_pending = False

        # -----------------------------
        # ROS comm
        # -----------------------------
        self.request_sub = self.create_subscription(
            Bool, '/fueling/request_detection', self.request_callback, 10
        )
        self.pose_pub = self.create_publisher(
            Float64MultiArray, '/fueling/fuel_port_pose', 10
        )
        self.status_pub = self.create_publisher(
            String, '/fueling/status', 10
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('fuel_port_perception_node ready.')

    def request_callback(self, msg: Bool):
        if msg.data:
            self.request_pending = True
            self.publish_status('detection_requested')

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def pixel_to_robot(self, u, v, depth_m):
        point_cam = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
        point_cam_mm = np.array(point_cam) * 1000.0
        point_robot = self.R_cam2robot @ point_cam_mm + self.t_cam2robot
        return point_robot

    def get_handle_angle_horizontal(self, roi):
        """
        roi 안에서 검은 막대 방향을 PCA로 계산
        가로 방향 = 0도
        결과 범위 = [-90, 90]
        """
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        _, binary = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY_INV)

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

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        display_image = color_image.copy()
        results = self.model(color_image, verbose=False)

        now = time.time()
        should_print = (now - self.last_print) >= 1.0

        detected_objects = []
        best_target = None

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                u, v = (x1 + x2) // 2, (y1 + y2) // 2
                depth_val = depth_frame.get_distance(u, v)
                if depth_val <= 0.0:
                    continue

                robot_xyz = self.pixel_to_robot(u, v, depth_val)
                x_robot, y_robot, z_robot = robot_xyz

                detected_objects.append(
                    {
                        "label": label,
                        "conf": conf,
                        "pixel": (u, v),
                        "depth": depth_val,
                        "robot_xyz": (x_robot, y_robot, z_robot),
                    }
                )

                cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                text = f"{label} {conf:.2f} | d={depth_val:.2f}m"
                cv2.putText(
                    display_image,
                    text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

                robot_text = f"R({x_robot:.1f}, {y_robot:.1f}, {z_robot:.1f})mm"
                cv2.putText(
                    display_image,
                    robot_text,
                    (x1, y2 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 200, 255),
                    1,
                )

                angle = 0.0
                if label == self.target_label:
                    pad = 8
                    x1p = max(0, x1 - pad)
                    y1p = max(0, y1 - pad)
                    x2p = min(color_image.shape[1] - 1, x2 + pad)
                    y2p = min(color_image.shape[0] - 1, y2 + pad)

                    roi = color_image[y1p:y2p, x1p:x2p]

                    if roi.size != 0:
                        angle, line_pts = self.get_handle_angle_horizontal(roi)

                        if angle is not None:
                            cv2.putText(
                                display_image,
                                f"angle={angle:.1f} deg",
                                (x1, y2 + 35),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 0, 255),
                                2,
                            )

                            if line_pts is not None:
                                (lx1, ly1), (lx2, ly2) = line_pts
                                cv2.line(
                                    display_image,
                                    (lx1 + x1p, ly1 + y1p),
                                    (lx2 + x1p, ly2 + y1p),
                                    (255, 0, 0),
                                    2,
                                )

                    candidate = {
                        'x': float(x_robot),
                        'y': float(y_robot),
                        'z': float(z_robot),
                        'rx': self.fixed_rx,
                        'ry': self.fixed_ry,
                        'rz': self.fixed_rz,
                        'angle': 0.0,
                        'conf': float(conf),
                    }

                    if best_target is None or conf > best_target['conf']:
                        best_target = candidate

                    if should_print:
                        print(
                            f"[{label}] conf={conf:.2f} | "
                            f"pixel=({u},{v}) | depth={depth_val:.3f}m | "
                            f"robot=({x_robot:.1f}, {y_robot:.1f}, {z_robot:.1f}) mm | "
                            f"fixed_rpy=({self.fixed_rx}, {self.fixed_ry}, {self.fixed_rz})"
                        )

        self.latest_best_target = best_target
        if self.latest_best_target is not None:
            cv2.putText(
                display_image,
                f"BEST: ({self.latest_best_target['x']:.1f}, {self.latest_best_target['y']:.1f}, {self.latest_best_target['z']:.1f})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
            )

        if should_print:
            self.last_print = now

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET,
        )

        images = np.hstack((display_image, depth_colormap))
        cv2.imshow("YOLOv8 + RealSense | RGB + Depth", images)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            rclpy.shutdown()
            return

        elif key == ord("c"):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            img_path = os.path.join(self.save_dir, f"{timestamp}.jpg")
            txt_path = os.path.join(self.save_dir, f"{timestamp}.txt")

            cv2.imwrite(img_path, display_image)

            with open(txt_path, "w") as f:
                f.write("label | conf | pixel(u,v) | depth(m) | robot_X(mm) | robot_Y(mm) | robot_Z(mm)\n")
                f.write("-" * 80 + "\n")
                for obj in detected_objects:
                    f.write(
                        f"{obj['label']} | {obj['conf']:.2f} | "
                        f"({obj['pixel'][0]},{obj['pixel'][1]}) | "
                        f"{obj['depth']:.3f} | "
                        f"{obj['robot_xyz'][0]:.1f} | "
                        f"{obj['robot_xyz'][1]:.1f} | "
                        f"{obj['robot_xyz'][2]:.1f}\n"
                    )

            print(f"\n>>> 캡쳐 저장 완료: {img_path}")
            print(f">>> 좌표 저장 완료: {txt_path} ({len(detected_objects)}개 객체)\n")

        if self.request_pending:
            if self.latest_best_target is None:
                self.publish_status('detection_failed')
                self.request_pending = False
                return

            msg = Float64MultiArray()
            msg.data = [
                self.latest_best_target['x'],
                self.latest_best_target['y'],
                self.latest_best_target['z'],
                self.latest_best_target['rx'],
                self.latest_best_target['ry'],
                self.latest_best_target['rz'],
                self.latest_best_target['angle'],
                self.latest_best_target['conf'],
            ]
            self.pose_pub.publish(msg)
            self.publish_status('fuel_port_pose_published')
            self.get_logger().info(f'published pose: {msg.data}')
            self.request_pending = False

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FuelPortPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
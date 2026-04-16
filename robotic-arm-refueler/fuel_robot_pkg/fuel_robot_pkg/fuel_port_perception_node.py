import math
from typing import Optional

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


class FuelPortPerceptionNode(Node):
    def __init__(self):
        super().__init__('fuel_port_perception_node')

        self.declare_parameter('model_path', 'yolov12.pt')
        self.declare_parameter('target_label', 'cap_handle')
        self.declare_parameter('conf_threshold', 0.80)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_label = self.get_parameter('target_label').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value

        self.model = YOLO(model_path)

        # 원본 코드 설정 유지
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

        # 원본 코드 설정 유지
        self.R_cam2robot = np.array([
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, -1.0, 0.0]
        ], dtype=np.float64)

        self.t_cam2robot = np.array([420.0, 65.0, 80.0], dtype=np.float64)

        self.tool_rx = 180.0
        self.tool_ry = 0.0
        self.tool_rz_base = 90.0

        self.request_pending = False
        self.latest_best_target: Optional[dict] = None

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

    def pixel_to_robot(self, u: int, v: int, depth_m: float) -> np.ndarray:
        point_cam = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth_m)
        point_cam_mm = np.array(point_cam, dtype=np.float64) * 1000.0
        point_robot = self.R_cam2robot @ point_cam_mm + self.t_cam2robot
        return point_robot

    def get_handle_angle_horizontal(self, roi):
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
                rx_pos, ry_pos, rz_pos = robot_xyz

                angle = 0.0
                if label == self.target_label:
                    pad = 8
                    x1p = max(0, x1 - pad)
                    y1p = max(0, y1 - pad)
                    x2p = min(color_image.shape[1] - 1, x2 + pad)
                    y2p = min(color_image.shape[0] - 1, y2 + pad)

                    roi = color_image[y1p:y2p, x1p:x2p]
                    if roi.size != 0:
                        angle_result, line_pts = self.get_handle_angle_horizontal(roi)
                        if angle_result is not None:
                            angle = angle_result
                            cv2.putText(
                                display_image,
                                f'angle={angle:.1f}',
                                (x1, y2 + 35),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.6,
                                (0, 0, 255),
                                2
                            )
                            if line_pts is not None:
                                (lx1, ly1), (lx2, ly2) = line_pts
                                cv2.line(
                                    display_image,
                                    (lx1 + x1p, ly1 + y1p),
                                    (lx2 + x1p, ly2 + y1p),
                                    (255, 0, 0),
                                    2
                                )

                cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    display_image,
                    f'{label} {conf:.2f}',
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
                cv2.putText(
                    display_image,
                    f'R({rx_pos:.1f}, {ry_pos:.1f}, {rz_pos:.1f})',
                    (x1, y2 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 200, 255),
                    1
                )

                if label == self.target_label:
                    rz_tool = self.tool_rz_base + angle
                    candidate = {
                        'x': float(rx_pos),
                        'y': float(ry_pos),
                        'z': float(rz_pos),
                        'rx': float(self.tool_rx),
                        'ry': float(self.tool_ry),
                        'rz': float(rz_tool),
                        'angle': float(angle),
                        'conf': float(conf),
                    }

                    if best_target is None or conf > best_target['conf']:
                        best_target = candidate

        self.latest_best_target = best_target

        if best_target is not None:
            cv2.putText(
                display_image,
                f"BEST: ({best_target['x']:.1f}, {best_target['y']:.1f}, {best_target['z']:.1f})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2
            )

        cv2.imshow('fuel_port_perception_node', display_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('q pressed. shutting down perception node.')
            rclpy.shutdown()
            return

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
            ]
            self.xyz_pub.publish(msg)
            self.publish_status('fuel_port_xyz_published')
            self.get_logger().info(f'published xyz: {msg.data}')
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
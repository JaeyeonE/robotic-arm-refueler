import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import requests as http_requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

# ROS2 status → 웹 step 매핑
STATUS_TO_STEP = {
    'start_received':           {'current_step': 1, 'step_progress': 0.0},
    'target_pose_ready':        {'current_step': 2, 'step_progress': 0.3},
    'robot_move_command_sent':  {'current_step': 2, 'step_progress': 0.7},
    'move_completed':           {'current_step': 4, 'step_progress': 1.0},
    'go_home_success':          {'current_step': 4, 'step_progress': 1.0},
    'move_to_fuel_port_success': {'current_step': 3, 'step_progress': 0.0},
}


class UiGatewayNode(Node):
    def __init__(self):
        super().__init__('ui_gateway_node')

        self.declare_parameter('http_port', 6000)
        self.declare_parameter('flask_url', 'http://localhost:5000')
        self.declare_parameter('station_id', 1)

        self.http_port = self.get_parameter('http_port').get_parameter_value().integer_value
        self.flask_url = self.get_parameter('flask_url').get_parameter_value().string_value
        self.station_id = self.get_parameter('station_id').get_parameter_value().integer_value

        self.start_pub = self.create_publisher(Bool, '/fueling/start', 10)
        self.xyz_pub = self.create_publisher(
            Float64MultiArray, '/fueling/fuel_port_xyz', 10
        )
        self.robot_cmd_pub = self.create_publisher(
            String, '/fueling/robot_cmd', 10
        )

        self.status_sub = self.create_subscription(
            String, '/fueling/status', self.status_callback, 10
        )
        self.done_sub = self.create_subscription(
            Bool, '/fueling/done', self.done_callback, 10
        )

        self.latest_status = ''
        self.latest_done = None
        self.current_task_id = None

        self._start_http_server()
        self.get_logger().info(
            f'UI gateway ready. HTTP :{self.http_port}, Flask: {self.flask_url}'
        )

    # ──────────────────────────────────────
    #  Flask 전송 헬퍼
    # ──────────────────────────────────────
    def _patch_task(self, **fields):
        if not self.current_task_id:
            return
        try:
            http_requests.patch(
                f'{self.flask_url}/api/task/{self.current_task_id}',
                json=fields, timeout=0.5,
            )
        except Exception as e:
            self.get_logger().warn(f'Flask PATCH failed: {e}')

    def _post_log(self, level, message, source='control'):
        try:
            http_requests.post(
                f'{self.flask_url}/api/logs',
                json={
                    'level': level,
                    'message': message,
                    'source': source,
                    'station_id': self.station_id,
                    'task_id': self.current_task_id,
                }, timeout=0.5,
            )
        except Exception:
            pass

    # ──────────────────────────────────────
    #  HTTP 서버 (Flask → ROS2 방향)
    # ──────────────────────────────────────
    def _start_http_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(length) if length else b'{}'
                try:
                    data = json.loads(body)
                except json.JSONDecodeError:
                    data = {}

                if self.path == '/fueling/start':
                    self._handle_start(data)
                elif self.path == '/fueling/estop':
                    self._handle_estop(data)
                elif self.path == '/fueling/go_home':
                    self._handle_go_home(data)
                else:
                    self._respond(404, {'error': 'not found'})

            def do_GET(self):
                if self.path == '/fueling/status':
                    self._respond(200, {
                        'status': node.latest_status,
                        'done': node.latest_done,
                    })
                else:
                    self._respond(404, {'error': 'not found'})

            def _handle_start(self, data):
                x = data.get('robot_x')
                y = data.get('robot_y')
                z = data.get('robot_z')

                if x is None or y is None or z is None:
                    self._respond(400, {'error': 'robot_x, robot_y, robot_z required'})
                    return

                # task_id 저장 (Flask가 전달)
                node.current_task_id = data.get('task_id')

                start_msg = Bool()
                start_msg.data = True
                node.start_pub.publish(start_msg)
                node.get_logger().info('published /fueling/start')

                xyz_msg = Float64MultiArray()
                xyz_msg.data = [float(x), float(y), float(z)]
                node.xyz_pub.publish(xyz_msg)
                node.get_logger().info(
                    f'published xyz: [{x:.1f}, {y:.1f}, {z:.1f}]'
                )

                node.latest_done = None
                self._respond(200, {'ok': True})

            def _handle_estop(self, data):
                node.get_logger().warn('E-STOP received from web')

                cmd_msg = String()
                cmd_msg.data = 'estop'
                node.robot_cmd_pub.publish(cmd_msg)

                node.publish_status('estop_from_web')
                node.latest_done = False

                self._respond(200, {'ok': True})

            def _handle_go_home(self, data):
                node.get_logger().info('Go home received from web')
                cmd_msg = String()
                cmd_msg.data = 'go_home'
                node.robot_cmd_pub.publish(cmd_msg)
                node.publish_status('go_home_from_web')
                self._respond(200, {'ok': True})

            def _respond(self, code, obj):
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(obj).encode())

            def log_message(self, format, *args):
                node.get_logger().debug(f'HTTP: {format % args}')

        server = HTTPServer(('0.0.0.0', self.http_port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

    # ──────────────────────────────────────
    #  ROS2 콜백 (ROS2 → Flask 방향)
    # ──────────────────────────────────────
    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub = self.create_publisher(String, '/fueling/status', 10)
        self.status_pub.publish(msg)

    def status_callback(self, msg: String):
        self.latest_status = msg.data
        self.get_logger().info(f'[UI STATUS] {msg.data}')

        # step 매핑 → Flask PATCH
        step_info = STATUS_TO_STEP.get(msg.data)
        if step_info:
            self._patch_task(**step_info)

        # 모든 상태 변경을 로그로 전송
        self._post_log('INFO', f'[ROS2] {msg.data}')

    def done_callback(self, msg: Bool):
        self.latest_done = msg.data
        if msg.data:
            self.get_logger().info('[UI STATUS] Fueling task completed.')
            self._patch_task(status='success', current_step=4, step_progress=1.0)
            self._post_log('OK', '로봇 작업 완료')
        else:
            self.get_logger().warn('[UI STATUS] Fueling task failed.')
            self._post_log('ERROR', '로봇 작업 실패')

        self.current_task_id = None


def main(args=None):
    rclpy.init(args=args)
    node = UiGatewayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

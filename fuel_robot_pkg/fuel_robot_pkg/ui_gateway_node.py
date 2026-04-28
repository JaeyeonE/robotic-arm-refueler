import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import requests as http_requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


# ROS2 status → 웹 step 매핑
STATUS_TO_STEP = {
    'start_received':            {'current_step': 1, 'step_progress': 0.0},
    'target_pose_ready':         {'current_step': 2, 'step_progress': 0.3},

    # Action result 기반 최종 완료
    'move_completed':            {'current_step': 4, 'step_progress': 1.0},
    'go_home_success':           {'current_step': 4, 'step_progress': 1.0},

    # 시퀀스 상태
    'fueling_started':           {'current_step': 1, 'step_progress': 0.0},
    'fueling_complete':          {'current_step': 4, 'step_progress': 1.0, 'status': 'success'},
    'fueling_error':             {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},
    'target_out_of_safe_zone':   {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},

    # E-STOP / ERROR
    'estop_cancel_requested':    {'current_step': 0, 'step_progress': 0.0, 'status': 'estop'},
    'task_canceled_by_estop':    {'current_step': 0, 'step_progress': 0.0, 'status': 'estop'},
    'estop_requested_no_active_goal': {'current_step': 0, 'step_progress': 0.0, 'status': 'estop'},
    'estop_cancel_failed':       {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},
    'go_home_failed':            {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},
    'action_server_not_available': {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},
    'goal_rejected':             {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},
    'goal_request_failed':       {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},
    'action_result_failed':      {'current_step': 0, 'step_progress': 0.0, 'status': 'fail'},
}

# Action step → kiosk 4-step 매핑 경계
# Phase 1: action 1~12  → step 1
# Phase 2: action 13~19 → step 2
# Phase 3: action 20~26 → step 3
# Phase 4: action 27~   → step 4
PHASE_BOUNDARIES = [12, 19, 26]


class UiGatewayNode(Node):
    def __init__(self):
        super().__init__('ui_gateway_node')

        self.declare_parameter('http_port', 6000)
        self.declare_parameter('flask_url', 'http://localhost:5000')
        self.declare_parameter('station_id', 1)

        self.http_port = self.get_parameter('http_port').value
        self.flask_url = self.get_parameter('flask_url').value
        self.station_id = self.get_parameter('station_id').value

        # ──────────────────────────────────────
        # Flask → ROS2
        # ──────────────────────────────────────
        self.start_pub = self.create_publisher(
            Bool,
            '/fueling/start',
            10
        )

        self.xyz_pub = self.create_publisher(
            Float64MultiArray,
            '/fueling/fuel_port_xyz',
            10
        )

        self.go_home_pub = self.create_publisher(
            Bool,
            '/fueling/go_home',
            10
        )

        # 기존 /fueling/robot_cmd 제거
        # E-STOP은 TaskManager가 구독하는 Bool topic으로 분리
        self.estop_request_pub = self.create_publisher(
            Bool,
            '/fueling/estop_request',
            10
        )

        # ──────────────────────────────────────
        # ROS2 → Flask
        # ──────────────────────────────────────
        # UI Gateway는 /fueling/status를 직접 publish하지 않는다.
        # status는 TaskManager 중심으로 발행하고, 여기서는 구독만 한다.
        self.status_sub = self.create_subscription(
            String,
            '/fueling/status',
            self.status_callback,
            10
        )

        self.done_sub = self.create_subscription(
            Bool,
            '/fueling/done',
            self.done_callback,
            10
        )

        self.safety_warning_sub = self.create_subscription(
            String,
            '/fueling/safety_warning',
            self.safety_warning_callback,
            10
        )

        # safety_monitor_node → UI
        self.tcp_sub = self.create_subscription(
            Float64MultiArray,
            '/fueling/current_tcp_pose',
            self._tcp_callback,
            10
        )

        self.joint_pos_sub = self.create_subscription(
            Float64MultiArray,
            '/fueling/current_joint_pos',
            self._joint_pos_callback,
            10
        )

        self.joint_torque_sub = self.create_subscription(
            Float64MultiArray,
            '/fueling/current_joint_torque',
            self._joint_torque_callback,
            10
        )

        self.latest_status = ''
        self.latest_done = None
        self.latest_safety_warning = ''
        self.current_task_id = None

        # 최신 로봇 상태 버퍼
        self.latest_tcp = None
        self.latest_joints = None
        self.latest_torques = None
        self.robot_mode = 'IDLE'
        self.dart_connected = False
        self._last_snapshot_payload = None

        # 1Hz로 Flask에 snapshot POST
        self.snapshot_timer = self.create_timer(1.0, self._post_snapshot)

        self._start_http_server()
        self.get_logger().info(
            f'UI gateway ready. HTTP :{self.http_port}, Flask: {self.flask_url}'
        )

    # ──────────────────────────────────────
    # Flask 전송 헬퍼
    # ──────────────────────────────────────
    def _patch_task(self, **fields):
        if not self.current_task_id:
            return

        try:
            http_requests.patch(
                f'{self.flask_url}/api/task/{self.current_task_id}',
                json=fields,
                timeout=0.5,
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
                },
                timeout=0.5,
            )
        except Exception:
            pass

    # ──────────────────────────────────────
    # HTTP 서버: Flask/app.py → ROS2
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
                        'safety_warning': node.latest_safety_warning,
                        'robot_mode': node.robot_mode,
                    })
                else:
                    self._respond(404, {'error': 'not found'})

            def _handle_start(self, data):
                x = data.get('robot_x')
                y = data.get('robot_y')
                z = data.get('robot_z')
                angle = data.get('handle_angle')

                if x is None or y is None or z is None:
                    self._respond(400, {'error': 'robot_x, robot_y, robot_z required'})
                    return

                node.current_task_id = data.get('task_id')

                # 1. start 요청
                start_msg = Bool()
                start_msg.data = True
                node.start_pub.publish(start_msg)
                node.get_logger().info('published /fueling/start')

                # 2. xyz 또는 xyz+handle_angle 전달
                xyz_msg = Float64MultiArray()

                if angle is not None:
                    xyz_msg.data = [
                        float(x),
                        float(y),
                        float(z),
                        float(angle),
                    ]
                    node.get_logger().info(
                        'published /fueling/fuel_port_xyz: '
                        f'[{float(x):.1f}, {float(y):.1f}, {float(z):.1f}, {float(angle):.1f}]'
                    )
                else:
                    xyz_msg.data = [
                        float(x),
                        float(y),
                        float(z),
                    ]
                    node.get_logger().info(
                        'published /fueling/fuel_port_xyz: '
                        f'[{float(x):.1f}, {float(y):.1f}, {float(z):.1f}]'
                    )

                node.xyz_pub.publish(xyz_msg)

                node.latest_done = None
                node.latest_safety_warning = ''
                self._respond(200, {'ok': True})

            def _handle_estop(self, data):
                node.get_logger().warn('E-STOP received from web')

                msg = Bool()
                msg.data = True
                node.estop_request_pub.publish(msg)

                # 상태는 직접 publish하지 않고, TaskManager가 /fueling/status로 발행하게 둔다.
                node.latest_done = False
                node.robot_mode = 'ESTOP'

                node._post_log('ERROR', 'E-STOP request published to ROS2', source='control')

                self._respond(200, {'ok': True})

            def _handle_go_home(self, data):
                node.get_logger().info('Go home received from web')

                msg = Bool()
                msg.data = True
                node.go_home_pub.publish(msg)

                # 상태는 직접 publish하지 않고, TaskManager가 go_home_received 등을 발행하게 둔다.
                node._post_log('INFO', 'Go home request published to ROS2', source='control')

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
    # ROS2 상태 처리: ROS2 → Flask
    # ──────────────────────────────────────
    def status_callback(self, msg: String):
        self.latest_status = msg.data
        self.get_logger().info(f'[UI STATUS] {msg.data}')

        # robot_mode 추적
        if (
            msg.data in ('start_received', 'target_pose_ready', 'fueling_started')
            or msg.data.startswith('fueling_progress:')
        ):
            self.robot_mode = 'FUELING'

        elif msg.data in ('fueling_complete', 'go_home_success', 'move_completed'):
            self.robot_mode = 'IDLE'

        elif (
            msg.data in (
                'estop_cancel_requested',
                'task_canceled_by_estop',
                'estop_requested_no_active_goal',
                'estop_executed',
            )
            or msg.data.startswith('safety_warning:')
        ):
            self.robot_mode = 'ESTOP'

        elif msg.data in (
            'fueling_error',
            'go_home_failed',
            'target_out_of_safe_zone',
            'action_server_not_available',
            'goal_rejected',
            'goal_request_failed',
            'action_result_failed',
            'estop_cancel_failed',
        ):
            self.robot_mode = 'ERROR'

        # fueling_progress:N/T 포맷 파싱
        if msg.data.startswith('fueling_progress:'):
            self._handle_fueling_progress(msg.data)
            return

        # go_home_progress:N/T는 필요 시 로그만 남김
        if msg.data.startswith('go_home_progress:'):
            self._post_log('INFO', f'[홈 복귀 진행] {msg.data}')
            return

        # 고정 step 매핑 → Flask PATCH
        step_info = STATUS_TO_STEP.get(msg.data)
        if step_info:
            self._patch_task(**step_info)

        self._post_log('INFO', f'[ROS2] {msg.data}')

    def safety_warning_callback(self, msg: String):
        if not msg.data:
            return

        self.latest_safety_warning = msg.data
        self.robot_mode = 'ESTOP'

        self.get_logger().warn(f'[SAFETY WARNING] {msg.data}')
        self._post_log('ERROR', f'[안전 경고] {msg.data}', source='safety')

    def _handle_fueling_progress(self, status_str: str):
        try:
            parts = status_str.split(':')[1]
            completed, total = map(int, parts.split('/'))
        except (IndexError, ValueError):
            return

        overall = completed / total if total > 0 else 0.0

        if completed <= PHASE_BOUNDARIES[0]:
            current_step = 1
        elif completed <= PHASE_BOUNDARIES[1]:
            current_step = 2
        elif completed <= PHASE_BOUNDARIES[2]:
            current_step = 3
        else:
            current_step = 4

        self._patch_task(
            current_step=current_step,
            step_progress=round(overall, 3),
        )

        # 진행 로그는 5 action마다 전송
        if completed % 5 == 0 or completed == total:
            pct = int(overall * 100)
            self._post_log('INFO', f'[주유 진행] {pct}% ({completed}/{total})')

    def done_callback(self, msg: Bool):
        self.latest_done = msg.data

        if msg.data:
            self.get_logger().info('[UI STATUS] task completed.')
            self._patch_task(status='success', current_step=4, step_progress=1.0)
            self._post_log('OK', '로봇 작업 완료')
        else:
            self.get_logger().warn('[UI STATUS] task failed or canceled.')

            # E-STOP 관련 status가 마지막이었다면 estop으로 유지
            if self.latest_status in (
                'estop_cancel_requested',
                'task_canceled_by_estop',
                'estop_requested_no_active_goal',
            ) or self.robot_mode == 'ESTOP':
                self._patch_task(status='estop')
                self._post_log('ERROR', '로봇 작업 E-STOP으로 중단')
            else:
                self._patch_task(status='fail')
                self._post_log('ERROR', '로봇 작업 실패')

        self.current_task_id = None

    # ──────────────────────────────────────
    # 로봇 상태 snapshot: safety_monitor_node → Flask
    # ──────────────────────────────────────
    def _tcp_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.latest_tcp = list(msg.data[:6])
            self.dart_connected = True

    def _joint_pos_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.latest_joints = list(msg.data[:6])
            self.dart_connected = True

    def _joint_torque_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self.latest_torques = list(msg.data[:6])

    def _post_snapshot(self):
        if self.latest_tcp is None and self.latest_joints is None:
            return

        payload = {
            'station_id': self.station_id,
            'task_id': self.current_task_id,
            'robot_mode': self.robot_mode,
            'dart_connected': 1 if self.dart_connected else 0,
        }

        if self.latest_tcp:
            payload.update({
                'tcp_x': self.latest_tcp[0],
                'tcp_y': self.latest_tcp[1],
                'tcp_z': self.latest_tcp[2],
                'tcp_a': self.latest_tcp[3],
                'tcp_b': self.latest_tcp[4],
                'tcp_c': self.latest_tcp[5],
            })

        if self.latest_joints:
            for i in range(6):
                payload[f'j{i + 1}_angle'] = self.latest_joints[i]

        if self.latest_torques:
            for i in range(6):
                payload[f'j{i + 1}_torque'] = self.latest_torques[i]

        if payload == self._last_snapshot_payload:
            return

        self._last_snapshot_payload = payload

        try:
            http_requests.post(
                f'{self.flask_url}/api/robot/snapshot',
                json=payload,
                timeout=0.5,
            )
        except Exception as e:
            self.get_logger().debug(f'snapshot POST failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UiGatewayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down UI gateway.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
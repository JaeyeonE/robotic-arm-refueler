import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, Float64MultiArray, String

from fuel_robot_interfaces.action import FuelTask


class FuelingTaskManagerNode(Node):
    """
    Fueling Task Manager

    역할:
    - UI Gateway로부터 /fueling/start, /fueling/fuel_port_xyz, /fueling/go_home을 받는다.
    - 좌표 안전구역을 검사한다.
    - Doosan Commander Action Server(/dsr01/fuel_task)로 goal을 보낸다.
    - Action feedback을 /fueling/status로 변환한다.
    - Action result를 /fueling/done으로 변환한다.

    제거한 구조:
    - /fueling/cmd_done 구독 제거
    - /fueling/fuel_port_pose 발행 제거
    - /fueling/robot_cmd 발행 제거
    """

    STATE_IDLE = 'IDLE'
    STATE_WAITING_XYZ = 'WAITING_XYZ'
    STATE_SENDING_GOAL = 'SENDING_GOAL'
    STATE_RUNNING = 'RUNNING'
    STATE_CANCELING = 'CANCELING'
    STATE_ESTOP = 'ESTOP'
    STATE_ERROR = 'ERROR'

    def __init__(self):
        super().__init__('fueling_task_manager_node')

        # ──────────────────────────────────────
        # Parameters: fixed TCP orientation
        # ──────────────────────────────────────
        self.declare_parameter('fixed_rx', 90.0)
        self.declare_parameter('fixed_ry', -90.0)
        self.declare_parameter('fixed_rz', -90.0)

        self.fixed_rx = float(self.get_parameter('fixed_rx').value)
        self.fixed_ry = float(self.get_parameter('fixed_ry').value)
        self.fixed_rz = float(self.get_parameter('fixed_rz').value)

        # ──────────────────────────────────────
        # Parameters: target safe zone
        # /fueling/fuel_port_xyz로 들어온 목표 좌표를 사전 검사
        # ──────────────────────────────────────
        self.declare_parameter('min_x', 360.0)
        self.declare_parameter('max_x', 600.0)
        self.declare_parameter('min_y', -525.0)
        self.declare_parameter('max_y', -220.0)
        self.declare_parameter('min_z', 160.0)
        self.declare_parameter('max_z', 370.0)

        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)

        # ──────────────────────────────────────
        # Action Client
        # DoosanCommanderNode(namespace=dsr01)의 ActionServer 이름과 맞춤
        # ──────────────────────────────────────
        self._action_client = ActionClient(
            self,
            FuelTask,
            '/dsr01/fuel_task'
        )

        self._goal_handle = None
        self.current_command = None

        # ──────────────────────────────────────
        # Subscriptions
        # ──────────────────────────────────────
        self.start_sub = self.create_subscription(
            Bool,
            '/fueling/start',
            self.start_callback,
            10
        )

        self.xyz_sub = self.create_subscription(
            Float64MultiArray,
            '/fueling/fuel_port_xyz',
            self.xyz_callback,
            10
        )

        self.go_home_sub = self.create_subscription(
            Bool,
            '/fueling/go_home',
            self.go_home_callback,
            10
        )

        # 9단계에서 UI Gateway, SafetyMonitor가 이 topic으로 E-STOP 요청을 보낼 예정
        # 지금 넣어도 문제 없음. publisher가 없으면 그냥 대기 상태.
        self.estop_sub = self.create_subscription(
            Bool,
            '/fueling/estop_request',
            self.estop_callback,
            10
        )

        # 선택 사항: safety warning을 status로도 남기고 싶을 때 사용
        self.safety_warning_sub = self.create_subscription(
            String,
            '/fueling/safety_warning',
            self.safety_warning_callback,
            10
        )

        # ──────────────────────────────────────
        # Publishers
        # ──────────────────────────────────────
        self.status_pub = self.create_publisher(
            String,
            '/fueling/status',
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            '/fueling/done',
            10
        )

        # ──────────────────────────────────────
        # State variables
        # ──────────────────────────────────────
        self.state = self.STATE_IDLE

        self.latest_xyz = None
        self.latest_handle_angle = 0.0
        self.use_handle_angle = False

        self.get_logger().info(
            'FuelingTaskManagerNode ready. Action Client -> /dsr01/fuel_task'
        )

    # ──────────────────────────────────────
    # Utility
    # ──────────────────────────────────────
    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'[STATUS] {text}')

    def publish_done(self, value: bool):
        msg = Bool()
        msg.data = value
        self.done_pub.publish(msg)
        self.get_logger().info(f'[DONE] {value}')

    def set_state(self, new_state: str):
        if self.state == new_state:
            return

        self.get_logger().info(f'[STATE] {self.state} -> {new_state}')
        self.state = new_state

    def reset_state(self):
        self._goal_handle = None
        self.current_command = None
        self.latest_xyz = None
        self.latest_handle_angle = 0.0
        self.use_handle_angle = False
        self.set_state(self.STATE_IDLE)

    def clear_goal_only(self):
        self._goal_handle = None
        self.current_command = None
        self.latest_xyz = None
        self.latest_handle_angle = 0.0
        self.use_handle_angle = False

    def is_busy(self) -> bool:
        return self.state in (
            self.STATE_WAITING_XYZ,
            self.STATE_SENDING_GOAL,
            self.STATE_RUNNING,
            self.STATE_CANCELING,
        )

    def is_in_safe_zone(self, x: float, y: float, z: float) -> bool:
        return (
            self.min_x <= x <= self.max_x and
            self.min_y <= y <= self.max_y and
            self.min_z <= z <= self.max_z
        )

    # ──────────────────────────────────────
    # /fueling/start
    # ──────────────────────────────────────
    def start_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.is_busy():
            self.get_logger().warn(f'start rejected: current state={self.state}')
            self.publish_status('task_already_running')
            return

        if self.state in (self.STATE_ESTOP, self.STATE_ERROR):
            # 필요하면 여기서 start를 차단하고 reset 버튼을 별도로 둘 수도 있음.
            # 지금은 재시작을 허용하기 위해 IDLE처럼 처리.
            self.get_logger().warn(f'start requested from {self.state}; resetting to WAITING_XYZ')

        self.current_command = 'execute_fueling'
        self.set_state(self.STATE_WAITING_XYZ)

        self.publish_status('start_received')
        self.get_logger().info('Start received. Waiting for /fueling/fuel_port_xyz.')

    # ──────────────────────────────────────
    # /fueling/fuel_port_xyz
    # data: [x, y, z] 또는 [x, y, z, handle_angle]
    # ──────────────────────────────────────
    def xyz_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            self.get_logger().error('fuel_port_xyz must contain at least x, y, z')
            self.publish_status('invalid_xyz_message')

            if self.state == self.STATE_WAITING_XYZ:
                self.publish_done(False)
                self.set_state(self.STATE_ERROR)
                self.clear_goal_only()
            return

        # 최신 좌표는 상태와 상관없이 저장해둠
        # start와 xyz 순서가 살짝 엇갈려도 좌표를 잃지 않기 위함
        self.latest_xyz = [float(v) for v in msg.data]

        if self.state != self.STATE_WAITING_XYZ:
            self.get_logger().info(
                f'fuel_port_xyz cached but ignored for now. state={self.state}, data={self.latest_xyz}'
            )
            return

        x, y, z = self.latest_xyz[:3]

        if len(self.latest_xyz) >= 4:
            self.latest_handle_angle = float(self.latest_xyz[3])
            self.use_handle_angle = True
            self.get_logger().info(
                f'detection result -> x={x:.1f}, y={y:.1f}, z={z:.1f}, '
                f'handle_angle={self.latest_handle_angle:.1f}'
            )
        else:
            self.latest_handle_angle = 0.0
            self.use_handle_angle = False
            self.get_logger().info(
                f'detection result -> x={x:.1f}, y={y:.1f}, z={z:.1f}, '
                'handle_angle=None'
            )

        if not self.is_in_safe_zone(x, y, z):
            self.get_logger().error(
                f'out of safe zone: x={x:.1f}, y={y:.1f}, z={z:.1f}, '
                f'allowed_x=[{self.min_x:.1f},{self.max_x:.1f}], '
                f'allowed_y=[{self.min_y:.1f},{self.max_y:.1f}], '
                f'allowed_z=[{self.min_z:.1f},{self.max_z:.1f}]'
            )

            self.publish_status('target_out_of_safe_zone')
            self.publish_done(False)
            self.set_state(self.STATE_ERROR)
            self.clear_goal_only()
            return

        target_pose = [
            float(x),
            float(y),
            float(z),
            float(self.fixed_rx),
            float(self.fixed_ry),
            float(self.fixed_rz),
        ]

        self.publish_status('target_pose_ready')
        self.set_state(self.STATE_SENDING_GOAL)

        self.send_fuel_task_goal(
            command='execute_fueling',
            target_pose=target_pose,
            handle_angle=self.latest_handle_angle,
            use_handle_angle=self.use_handle_angle,
        )

    # ──────────────────────────────────────
    # /fueling/go_home
    # ──────────────────────────────────────
    def go_home_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.state in (
            self.STATE_WAITING_XYZ,
            self.STATE_SENDING_GOAL,
            self.STATE_RUNNING,
            self.STATE_CANCELING,
        ):
            self.get_logger().warn(f'go_home rejected: current state={self.state}')
            self.publish_status(f'go_home_rejected:{self.state}')
            return

        self.current_command = 'go_home'
        self.set_state(self.STATE_SENDING_GOAL)

        self.publish_status('go_home_received')
        self.get_logger().info('Go home received. Sending Action goal.')

        self.send_fuel_task_goal(
            command='go_home',
            target_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            handle_angle=0.0,
            use_handle_angle=False,
        )

    # ──────────────────────────────────────
    # /fueling/estop_request
    # ──────────────────────────────────────
    def estop_callback(self, msg: Bool):
        if not msg.data:
            return

        self.handle_estop_request(source='estop_request')

    def handle_estop_request(self, source: str = 'unknown'):
        self.get_logger().warn(f'E-STOP requested from {source}. state={self.state}')

        # 이미 cancel 중이면 중복 처리 방지
        if self.state == self.STATE_CANCELING:
            self.get_logger().warn('E-STOP ignored: already canceling')
            return

        # 실행 중인 goal이 있으면 cancel 요청
        if self._goal_handle is not None:
            self.set_state(self.STATE_CANCELING)
            self.publish_status('estop_cancel_requested')

            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            return

        # goal handle이 없으면 Action cancel은 못 보내지만 상태는 ESTOP으로 전환
        self.publish_status('estop_requested_no_active_goal')
        self.publish_done(False)
        self.clear_goal_only()
        self.set_state(self.STATE_ESTOP)

    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            self.get_logger().warn(f'Cancel response: {cancel_response}')
        except Exception as e:
            self.get_logger().error(f'Cancel request failed: {e}')
            self.publish_status('estop_cancel_failed')
            self.publish_done(False)
            self.clear_goal_only()
            self.set_state(self.STATE_ERROR)
            return

        self.publish_status('task_canceled_by_estop')
        self.publish_done(False)
        self.clear_goal_only()
        self.set_state(self.STATE_ESTOP)

    # ──────────────────────────────────────
    # /fueling/safety_warning
    # ──────────────────────────────────────
    def safety_warning_callback(self, msg: String):
        # SafetyMonitor가 경고 이유를 보냈을 때 UI에서도 볼 수 있게 status로 남김
        if not msg.data:
            return

        self.get_logger().warn(f'Safety warning received: {msg.data}')
        self.publish_status(f'safety_warning:{msg.data}')

    # ──────────────────────────────────────
    # Action Goal send
    # ──────────────────────────────────────
    def send_fuel_task_goal(
        self,
        command: str,
        target_pose,
        handle_angle: float = 0.0,
        use_handle_angle: bool = False,
    ):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FuelTask Action Server not available: /dsr01/fuel_task')
            self.publish_status('action_server_not_available')
            self.publish_done(False)
            self.clear_goal_only()
            self.set_state(self.STATE_ERROR)
            return

        goal_msg = FuelTask.Goal()
        goal_msg.command = str(command)
        goal_msg.target_pose = [float(v) for v in target_pose]
        goal_msg.handle_angle = float(handle_angle)
        goal_msg.use_handle_angle = bool(use_handle_angle)

        self.current_command = command

        self.get_logger().info(
            f'Sending Action goal: command={goal_msg.command}, '
            f'target_pose={goal_msg.target_pose}, '
            f'handle_angle={goal_msg.handle_angle:.1f}, '
            f'use_handle_angle={goal_msg.use_handle_angle}'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Goal request failed: {e}')
            self.publish_status('goal_request_failed')
            self.publish_done(False)
            self.clear_goal_only()
            self.set_state(self.STATE_ERROR)
            return

        if not goal_handle.accepted:
            self.get_logger().error('Action goal rejected')
            self.publish_status('goal_rejected')
            self.publish_done(False)
            self.clear_goal_only()
            self.set_state(self.STATE_ERROR)
            return

        self.get_logger().info('Action goal accepted')
        self._goal_handle = goal_handle
        self.set_state(self.STATE_RUNNING)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback

        self.get_logger().info(
            f'[Feedback] {fb.current_step}/{fb.total_steps} - {fb.status_desc}'
        )

        if self.current_command == 'go_home':
            self.publish_status(f'go_home_progress:{fb.current_step}/{fb.total_steps}')
        else:
            self.publish_status(f'fueling_progress:{fb.current_step}/{fb.total_steps}')

    def result_callback(self, future):
        # cancel 처리 중이면 cancel_done_callback이 최종 상태를 정리하도록 둔다.
        # 단, 일부 환경에서는 cancel 이후에도 result_callback이 먼저 올 수 있어 방어 로직을 둔다.
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            status = wrapped_result.status
        except Exception as e:
            self.get_logger().error(f'Action result failed: {e}')
            self.publish_status('action_result_failed')
            self.publish_done(False)
            self.clear_goal_only()
            self.set_state(self.STATE_ERROR)
            return

        command = self.current_command

        self.get_logger().info(
            f'Action result received: command={command}, '
            f'success={result.success}, message={result.message}, status={status}'
        )

        if self.state == self.STATE_CANCELING:
            self.get_logger().warn('Action result received while canceling. Finalizing as ESTOP.')
            self.publish_status('task_canceled_by_estop')
            self.publish_done(False)
            self.clear_goal_only()
            self.set_state(self.STATE_ESTOP)
            return

        if result.success:
            if command == 'go_home':
                self.publish_status('go_home_success')
            else:
                self.publish_status('move_completed')

            self.publish_done(True)
            self.clear_goal_only()
            self.set_state(self.STATE_IDLE)
            return

        # 실패 처리
        if command == 'go_home':
            self.publish_status('go_home_failed')
        else:
            self.publish_status('fueling_error')

        self.publish_done(False)
        self.clear_goal_only()
        self.set_state(self.STATE_ERROR)


def main(args=None):
    rclpy.init(args=args)

    node = FuelingTaskManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down task manager.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
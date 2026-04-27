import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, Float64MultiArray, String

from fuel_robot_interfaces.action import FuelTask


class FuelingTaskManagerNode(Node):
    def __init__(self):
        super().__init__('fueling_task_manager_node')

        # ── 고정 자세값 ──
        self.declare_parameter('fixed_rx', 90.0)
        self.declare_parameter('fixed_ry', -90.0)
        self.declare_parameter('fixed_rz', -90.0)

        self.fixed_rx = self.get_parameter('fixed_rx').value
        self.fixed_ry = self.get_parameter('fixed_ry').value
        self.fixed_rz = self.get_parameter('fixed_rz').value

        # ── 안전구역 ──
        self.declare_parameter('min_x', 360.0)
        self.declare_parameter('max_x', 600.0)
        self.declare_parameter('min_y', -525.0)
        self.declare_parameter('max_y', -220.0)
        self.declare_parameter('min_z', 160.0)
        self.declare_parameter('max_z', 370.0)

        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value

        # ── Action Client ──
        self._action_client = ActionClient(
            self,
            FuelTask,
            '/dsr01/fuel_task'
        )
        self._goal_handle = None
        self.current_command = None

        # ── Subscriptions ──
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

        self.robot_cmd_sub = self.create_subscription(
            String,
            '/fueling/robot_cmd',
            self.robot_cmd_callback,
            10
        )

        # ── Publishers ──
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

        # ── 상태 변수 ──
        self.busy = False
        self.waiting_for_xyz = False

        self.latest_handle_angle = 0.0
        self.use_handle_angle = False

        self.get_logger().info('FuelingTaskManagerNode Action Client ready.')

    # ──────────────────────────────────────
    #  유틸리티
    # ──────────────────────────────────────
    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_done(self, value: bool):
        msg = Bool()
        msg.data = value
        self.done_pub.publish(msg)

    def reset_state(self):
        self.busy = False
        self.waiting_for_xyz = False
        self._goal_handle = None
        self.current_command = None
        self.latest_handle_angle = 0.0
        self.use_handle_angle = False

    def is_in_safe_zone(self, x: float, y: float, z: float) -> bool:
        return (
            self.min_x <= x <= self.max_x and
            self.min_y <= y <= self.max_y and
            self.min_z <= z <= self.max_z
        )

    # ──────────────────────────────────────
    #  Start → Vision xyz 대기
    # ──────────────────────────────────────
    def start_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.busy:
            self.get_logger().warn('task already running')
            self.publish_status('task_already_running')
            return

        self.busy = True
        self.waiting_for_xyz = True
        self.current_command = 'execute_fueling'

        self.publish_status('start_received')
        self.get_logger().info('Start received. Waiting for xyz from vision.')

    def xyz_callback(self, msg: Float64MultiArray):
        if not self.busy or not self.waiting_for_xyz:
            return

        if len(msg.data) < 3:
            self.get_logger().error('fuel_port_xyz must contain at least x, y, z')
            self.publish_status('invalid_xyz_message')
            self.publish_done(False)
            self.reset_state()
            return

        x, y, z = msg.data[:3]

        if len(msg.data) >= 4:
            self.latest_handle_angle = float(msg.data[3])
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
                f'out of safe zone: x={x:.1f}, y={y:.1f}, z={z:.1f}'
            )
            self.publish_status('target_out_of_safe_zone')
            self.publish_done(False)
            self.reset_state()
            return

        target_pose = [
            float(x),
            float(y),
            float(z),
            float(self.fixed_rx),
            float(self.fixed_ry),
            float(self.fixed_rz),
        ]

        self.waiting_for_xyz = False
        self.publish_status('target_pose_ready')

        self.send_fuel_task_goal(
            command='execute_fueling',
            target_pose=target_pose,
            handle_angle=self.latest_handle_angle,
            use_handle_angle=self.use_handle_angle,
        )

    # ──────────────────────────────────────
    #  Go Home Topic → Action Goal
    # ──────────────────────────────────────
    def go_home_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.busy:
            self.get_logger().warn('cannot go home: task already running')
            self.publish_status('task_already_running')
            return

        self.busy = True
        self.waiting_for_xyz = False
        self.current_command = 'go_home'

        self.publish_status('go_home_received')
        self.get_logger().info('Go home received. Sending Action goal.')

        self.send_fuel_task_goal(
            command='go_home',
            target_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            handle_angle=0.0,
            use_handle_angle=False,
        )

    # ──────────────────────────────────────
    #  Action Goal 송신
    # ──────────────────────────────────────
    def send_fuel_task_goal(
        self,
        command: str,
        target_pose,
        handle_angle: float = 0.0,
        use_handle_angle: bool = False,
    ):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FuelTask Action Server not available')
            self.publish_status('action_server_not_available')
            self.publish_done(False)
            self.reset_state()
            return

        goal_msg = FuelTask.Goal()
        goal_msg.command = command
        goal_msg.target_pose = [float(v) for v in target_pose]
        goal_msg.handle_angle = float(handle_angle)
        goal_msg.use_handle_angle = bool(use_handle_angle)

        self.get_logger().info(
            f'Sending Action goal: command={command}, '
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
            self.reset_state()
            return

        if not goal_handle.accepted:
            self.get_logger().error('Action goal rejected')
            self.publish_status('goal_rejected')
            self.publish_done(False)
            self.reset_state()
            return

        self.get_logger().info('Action goal accepted')
        self._goal_handle = goal_handle

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
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'Action result failed: {e}')
            self.publish_status('action_result_failed')
            self.publish_done(False)
            self.reset_state()
            return

        command = self.current_command

        if result.success:
            self.get_logger().info(f'Action succeeded: {result.message}')

            if command == 'go_home':
                self.publish_status('go_home_success')
            else:
                self.publish_status('move_completed')

            self.publish_done(True)
        else:
            self.get_logger().error(f'Action failed: {result.message}')

            if command == 'go_home':
                self.publish_status('go_home_failed')
            else:
                self.publish_status('fueling_error')

            self.publish_done(False)

        self.reset_state()

    # ──────────────────────────────────────
    #  E-STOP
    # ──────────────────────────────────────
    def robot_cmd_callback(self, msg: String):
        if msg.data != 'estop':
            return

        self.get_logger().warn('E-STOP received. Canceling active Action goal.')

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        self.publish_status('estop_from_task_manager')
        self.publish_done(False)
        self.reset_state()


def main(args=None):
    rclpy.init(args=args)

    node = FuelingTaskManagerNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
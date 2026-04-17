import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


class FuelingTaskManagerNode(Node):
    def __init__(self):
        super().__init__('fueling_task_manager_node')

        # 고정 자세값
        self.declare_parameter('fixed_rx', 90.0)
        self.declare_parameter('fixed_ry', -90.0)
        self.declare_parameter('fixed_rz', -90.0)

        self.fixed_rx = self.get_parameter('fixed_rx').value
        self.fixed_ry = self.get_parameter('fixed_ry').value
        self.fixed_rz = self.get_parameter('fixed_rz').value

        # 안전구역
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

        self.start_sub = self.create_subscription(
            Bool, '/fueling/start', self.start_callback, 10
        )
        self.xyz_sub = self.create_subscription(
            Float64MultiArray, '/fueling/fuel_port_xyz', self.xyz_callback, 10
        )
        self.cmd_done_sub = self.create_subscription(
            String, '/fueling/cmd_done', self.cmd_done_callback, 10
        )
        self.robot_cmd_sub = self.create_subscription(
            String, '/fueling/robot_cmd', self.robot_cmd_callback, 10
        )

        self.target_pose_pub = self.create_publisher(
            Float64MultiArray, '/fueling/fuel_port_pose', 10
        )
        self.robot_cmd_pub = self.create_publisher(
            String, '/fueling/robot_cmd', 10
        )
        self.status_pub = self.create_publisher(
            String, '/fueling/status', 10
        )
        self.done_pub = self.create_publisher(
            Bool, '/fueling/done', 10
        )

        self.busy = False
        self.waiting_for_xyz = False
        self.waiting_for_robot = False
        self.move_timer = None

        self.get_logger().info('fueling_task_manager_node ready.')

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
        self.waiting_for_robot = False

        if self.move_timer is not None:
            self.move_timer.cancel()
            self.move_timer = None

    # 안전구역 검사 함수
    def is_in_safe_zone(self, x: float, y: float, z: float) -> bool:
        return (
            self.min_x <= x <= self.max_x and
            self.min_y <= y <= self.max_y and
            self.min_z <= z <= self.max_z
        )

    def start_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.busy:
            self.get_logger().warn('task already running')
            self.publish_status('task_already_running')
            return

        self.busy = True
        self.waiting_for_xyz = True
        self.waiting_for_robot = False

        self.publish_status('start_received')
        self.get_logger().info('waiting for xyz from gateway')

    def xyz_callback(self, msg: Float64MultiArray):
        if not self.busy or not self.waiting_for_xyz:
            return

        if len(msg.data) < 3:
            self.get_logger().error('fuel_port_xyz must contain at least 3 values: x, y, z')
            self.publish_status('invalid_xyz_message')
            self.publish_done(False)
            self.reset_state()
            return

        x, y, z = msg.data[:3]

        self.get_logger().info(
            f'detection result -> x={x:.1f}, y={y:.1f}, z={z:.1f}'
        )
        # 안전구역 검사
        if not self.is_in_safe_zone(x, y, z):
            self.get_logger().error(
                f'out of safe zone: x={x:.1f}, y={y:.1f}, z={z:.1f}'
            )
            self.publish_status('target_out_of_safe_zone')
            self.publish_done(False)
            self.reset_state()
            return

        target_msg = Float64MultiArray()
        target_msg.data = [x, y, z, self.fixed_rx, self.fixed_ry, self.fixed_rz]
        self.target_pose_pub.publish(target_msg)

        self.publish_status('target_pose_ready')

        self.waiting_for_xyz = False
        self.waiting_for_robot = True

        # pose가 commander에 먼저 반영된 뒤 명령이 가도록 약간 지연
        if self.move_timer is not None:
            self.move_timer.cancel()

        self.move_timer = self.create_timer(0.1, self.send_move_command_once)

    def send_move_command_once(self):
        if self.move_timer is not None:
            self.move_timer.cancel()
            self.move_timer = None

        cmd_msg = String()
        cmd_msg.data = 'execute_fueling'
        self.robot_cmd_pub.publish(cmd_msg)

        self.get_logger().info('published robot command: execute_fueling')
        self.publish_status('robot_move_command_sent')

    def robot_cmd_callback(self, msg: String):
        if msg.data == 'estop':
            self.get_logger().warn('E-STOP received, resetting FSM')
            self.publish_done(False)
            self.reset_state()

    def cmd_done_callback(self, msg: String):
        if not self.busy or not self.waiting_for_robot:
            return

        if msg.data not in ('move_to_fuel_port', 'execute_fueling'):
            return

        self.get_logger().info(f'robot command completed: {msg.data}')
        self.publish_status('move_completed')
        self.publish_done(True)
        self.reset_state()


def main(args=None):
    rclpy.init(args=args)
    node = FuelingTaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
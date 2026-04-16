import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from dsr_msgs2.srv import MoveJoint, MoveLine, SetRobotMode, SetToolDigitalOutput
from functools import partial


class DoosanCommanderNode(Node):
    def __init__(self):
        super().__init__('doosan_commander_node', namespace='dsr01')

        self.latest_fuel_port_pose = None

        self.cmd_sub = self.create_subscription(
            String, '/fueling/robot_cmd', self.cmd_callback, 10
        )
        self.pose_sub = self.create_subscription(
            Float64MultiArray, '/fueling/fuel_port_pose', self.pose_callback, 10
        )

        self.done_pub = self.create_publisher(
            String, '/fueling/cmd_done', 10
        )
        self.status_pub = self.create_publisher(
            String, '/fueling/status', 10
        )

        self.set_mode_cli = self.create_client(SetRobotMode, 'system/set_robot_mode')
        self.move_joint_cli = self.create_client(MoveJoint, 'motion/move_joint')
        self.move_line_cli = self.create_client(MoveLine, 'motion/move_line')
        self.tool_do_cli = self.create_client(SetToolDigitalOutput, 'io/set_tool_digital_output')

        self.wait_for_services()
        self.set_autonomous_mode()

        self.home_j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        self.get_logger().info('Doosan commander ready.')

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def wait_for_services(self):
        while not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for system/set_robot_mode...')
        while not self.move_joint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for motion/move_joint...')
        while not self.move_line_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for motion/move_line...')
        while not self.tool_do_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for io/set_tool_digital_output...')

    def set_autonomous_mode(self):
        req = SetRobotMode.Request()
        req.robot_mode = 1
        future = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Robot mode set request sent.')

    def pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            self.get_logger().warn('fuel_port_pose must contain exactly 6 values')
            return

        self.latest_fuel_port_pose = list(msg.data)

        x, y, z, rx, ry, rz = self.latest_fuel_port_pose
        self.get_logger().info(
            f'updated target pose -> x={x:.1f}, y={y:.1f}, z={z:.1f}, '
            f'rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}'
        )

    def cmd_callback(self, msg: String):
        cmd = msg.data

        if cmd == 'go_home':
            self.call_move_joint(self.home_j, cmd)

        elif cmd == 'move_to_fuel_port':
            if self.latest_fuel_port_pose is None:
                self.get_logger().error('target pose not received yet')
                self.publish_status('target_pose_not_ready')
                return

            self.call_move_line(self.latest_fuel_port_pose, cmd)

        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def call_move_joint(self, joints, done_cmd):
        req = MoveJoint.Request()
        req.pos = joints
        req.vel = 20.0
        req.acc = 20.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        future = self.move_joint_cli.call_async(req)
        future.add_done_callback(partial(self._service_done_callback, done_cmd=done_cmd))

    def call_move_line(self, pose, done_cmd):
        req = MoveLine.Request()
        req.pos = pose
        req.vel = [30.0, 20.0]
        req.acc = [60.0, 40.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        future = self.move_line_cli.call_async(req)
        future.add_done_callback(partial(self._service_done_callback, done_cmd=done_cmd))

    def _service_done_callback(self, future, done_cmd):
        try:
            response = future.result()
            self.get_logger().info(f'Command finished: {done_cmd}, success={response.success}')

            msg = String()
            msg.data = done_cmd
            self.done_pub.publish(msg)

            if response.success:
                self.publish_status(f'{done_cmd}_success')
            else:
                self.publish_status(f'{done_cmd}_failed')

        except Exception as e:
            self.get_logger().error(f'Command failed: {done_cmd}, error={e}')
            self.publish_status(f'{done_cmd}_exception')


def main(args=None):
    rclpy.init(args=args)
    node = DoosanCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
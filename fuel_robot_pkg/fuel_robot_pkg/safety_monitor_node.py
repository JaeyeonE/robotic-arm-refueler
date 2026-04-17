import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from dsr_msgs2.srv import GetCurrentPosj, GetCurrentPosx, GetRobotState


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node', namespace='dsr01')

        # 안전 범위
        self.declare_parameter('max_x', 700.0)
        self.declare_parameter('min_x', -700.0)
        self.declare_parameter('max_y', 700.0)
        self.declare_parameter('min_y', -700.0)
        self.declare_parameter('max_z', 700.0)
        self.declare_parameter('min_z', 0.0)

        self.max_x = float(self.get_parameter('max_x').value)
        self.min_x = float(self.get_parameter('min_x').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.min_z = float(self.get_parameter('min_z').value)

        self.estop_sent = False
        self.current_tcp = None
        self.current_joint = None
        self.robot_state = None
        self.waiting_response = False

        self.robot_cmd_pub = self.create_publisher(
            String, '/fueling/robot_cmd', 10
        )
        self.status_pub = self.create_publisher(
            String, '/fueling/status', 10
        )
        self.current_tcp_pub = self.create_publisher(
            Float64MultiArray, '/fueling/current_tcp_pose', 10
        )
        self.current_joint_pub = self.create_publisher(
            Float64MultiArray, '/fueling/current_joint_pos', 10
        )

        # 공식 서비스
        self.get_posj_cli = self.create_client(
            GetCurrentPosj, 'aux_control/get_current_posj'
        )
        self.get_posx_cli = self.create_client(
            GetCurrentPosx, 'aux_control/get_current_posx'
        )
        self.get_robot_state_cli = self.create_client(
            GetRobotState, 'system/get_robot_state'
        )

        self.wait_for_services()

        # 1Hz 감시
        self.timer = self.create_timer(1, self.monitor_once)

        self.get_logger().info('safety_monitor_node ready')

    def wait_for_services(self):
        while not self.get_posj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for aux_control/get_current_posj...')
        while not self.get_posx_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for aux_control/get_current_posx...')
        while not self.get_robot_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for system/get_robot_state...')

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_estop(self, reason: str):
        if self.estop_sent:
            return

        cmd = String()
        cmd.data = 'estop'
        self.robot_cmd_pub.publish(cmd)

        self.publish_status(reason)
        self.get_logger().error(reason)
        self.estop_sent = True

    def monitor_once(self):
        if self.waiting_response:
            return

        self.waiting_response = True

        # 1) robot state 요청
        state_req = GetRobotState.Request()
        state_future = self.get_robot_state_cli.call_async(state_req)
        state_future.add_done_callback(self._robot_state_cb)

        # 2) joint 요청
        posj_req = GetCurrentPosj.Request()
        posj_future = self.get_posj_cli.call_async(posj_req)
        posj_future.add_done_callback(self._posj_cb)

        # 3) tcp 요청
        posx_req = GetCurrentPosx.Request()
        posx_req.ref = 0   # DR_BASE
        posx_future = self.get_posx_cli.call_async(posx_req)
        posx_future.add_done_callback(self._posx_cb)

    def _robot_state_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.robot_state = int(resp.robot_state)
        except Exception as e:
            self.get_logger().error(f'get_robot_state failed: {e}')

    def _posj_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.current_joint = list(resp.pos)

                msg = Float64MultiArray()
                msg.data = self.current_joint
                self.current_joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'get_current_posj failed: {e}')

    def _posx_cb(self, future):
        try:
            resp = future.result()
            if resp.success and len(resp.task_pos_info) > 0:
                # 공식 문서: task_pos_info[0][0:5]가 pose, [6]이 solution space
                pose = list(resp.task_pos_info[0].data[:6])
                self.current_tcp = pose

                msg = Float64MultiArray()
                msg.data = pose
                self.current_tcp_pub.publish(msg)

                self.check_workspace_limit()
        except Exception as e:
            self.get_logger().error(f'get_current_posx failed: {e}')
        finally:
            self.waiting_response = False

    def check_workspace_limit(self):
        if self.current_tcp is None:
            return

        x, y, z, rx, ry, rz = self.current_tcp

        if not (self.min_x <= x <= self.max_x):
            self.publish_estop(
                f'safety_limit_exceeded:x={x:.1f}, allowed=[{self.min_x:.1f},{self.max_x:.1f}]'
            )
            return

        if not (self.min_y <= y <= self.max_y):
            self.publish_estop(
                f'safety_limit_exceeded:y={y:.1f}, allowed=[{self.min_y:.1f},{self.max_y:.1f}]'
            )
            return

        if not (self.min_z <= z <= self.max_z):
            self.publish_estop(
                f'safety_limit_exceeded:z={z:.1f}, allowed=[{self.min_z:.1f},{self.max_z:.1f}]'
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
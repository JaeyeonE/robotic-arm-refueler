import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray

from dsr_msgs2.srv import (
    GetCurrentPosj,
    GetCurrentPosx,
    GetRobotState,
    GetJointTorque,
)


class SafetyMonitorNode(Node):
    """
    SafetyMonitorNode

    역할:
    - Doosan 서비스로 현재 TCP, 조인트, 토크, robot_state를 1Hz로 읽음
    - /fueling/current_tcp_pose 발행
    - /fueling/current_joint_pos 발행
    - /fueling/current_joint_torque 발행
    - 작업 중에만 안전구역 검사 수행
    - 안전구역 이탈 시 /fueling/safety_warning 발행
    - 안전구역 이탈 시 /fueling/estop_request true 발행

    제거한 구조:
    - /fueling/robot_cmd 발행 제거
    - /fueling/status 직접 발행 제거
    """

    def __init__(self):
        super().__init__('safety_monitor_node', namespace='dsr01')

        # ─────────────────────────────────────
        # 안전 범위
        # 단위: Doosan TCP pose 기준 mm
        # ─────────────────────────────────────
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

        # ─────────────────────────────────────
        # 상태 변수
        # ─────────────────────────────────────
        self.monitor_enabled = False
        self.estop_sent = False

        self.current_tcp = None
        self.current_joint = None
        self.current_torque = None
        self.robot_state = None

        self.waiting_response = False

        # ─────────────────────────────────────
        # Subscribers
        # ─────────────────────────────────────
        self.status_sub = self.create_subscription(
            String,
            '/fueling/status',
            self.status_callback,
            10,
        )

        # ─────────────────────────────────────
        # Publishers
        # ─────────────────────────────────────
        self.safety_warning_pub = self.create_publisher(
            String,
            '/fueling/safety_warning',
            10,
        )

        self.estop_request_pub = self.create_publisher(
            Bool,
            '/fueling/estop_request',
            10,
        )

        self.current_tcp_pub = self.create_publisher(
            Float64MultiArray,
            '/fueling/current_tcp_pose',
            10,
        )

        self.current_joint_pub = self.create_publisher(
            Float64MultiArray,
            '/fueling/current_joint_pos',
            10,
        )

        self.current_torque_pub = self.create_publisher(
            Float64MultiArray,
            '/fueling/current_joint_torque',
            10,
        )

        # ─────────────────────────────────────
        # Doosan service clients
        # namespace='dsr01'이므로 실제 서비스는 /dsr01/...
        # ─────────────────────────────────────
        self.get_posj_cli = self.create_client(
            GetCurrentPosj,
            'aux_control/get_current_posj',
        )

        self.get_posx_cli = self.create_client(
            GetCurrentPosx,
            'aux_control/get_current_posx',
        )

        self.get_robot_state_cli = self.create_client(
            GetRobotState,
            'system/get_robot_state',
        )

        self.get_joint_torque_cli = self.create_client(
            GetJointTorque,
            'aux_control/get_joint_torque',
        )

        self.wait_for_services()

        # 1Hz 감시
        self.timer = self.create_timer(1.0, self.monitor_once)

        self.get_logger().info(
            'safety_monitor_node ready. '
            'robot_cmd removed, using /fueling/safety_warning and /fueling/estop_request.'
        )

    # ─────────────────────────────────────
    # Service wait
    # ─────────────────────────────────────
    def wait_for_services(self):
        while rclpy.ok() and not self.get_posj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for aux_control/get_current_posj...')

        while rclpy.ok() and not self.get_posx_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for aux_control/get_current_posx...')

        while rclpy.ok() and not self.get_robot_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for system/get_robot_state...')

        while rclpy.ok() and not self.get_joint_torque_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for aux_control/get_joint_torque...')

    # ─────────────────────────────────────
    # /fueling/status callback
    # ─────────────────────────────────────
    def status_callback(self, msg: String):
        status = msg.data

        # 작업 시작 또는 작업 진행 중에만 안전 감시 활성화
        if (
            status in ('fueling_started', 'target_pose_ready')
            or status.startswith('fueling_progress:')
        ):
            if not self.monitor_enabled:
                self.get_logger().info(f'Safety monitoring enabled by status: {status}')

            self.monitor_enabled = True

            # 새로운 작업이 시작되면 이전 estop latch 해제
            self.estop_sent = False
            return

        # 작업이 끝났거나 중단되면 안전 감시 비활성화
        if (
            status in (
                'move_completed',
                'fueling_complete',
                'go_home_success',
                'fueling_error',
                'go_home_failed',
                'target_out_of_safe_zone',
                'task_canceled_by_estop',
                'estop_requested_no_active_goal',
                'estop_executed',
                'action_server_not_available',
                'goal_rejected',
                'goal_request_failed',
                'action_result_failed',
                'estop_cancel_failed',
            )
            or status.startswith('go_home_progress:')
            or status.startswith('safety_warning:')
        ):
            if self.monitor_enabled:
                self.get_logger().info(f'Safety monitoring disabled by status: {status}')

            self.monitor_enabled = False
            return

    # ─────────────────────────────────────
    # Main monitor loop
    # ─────────────────────────────────────
    def monitor_once(self):
        if self.waiting_response:
            return

        self.waiting_response = True

        # 1) robot state 요청
        state_req = GetRobotState.Request()
        state_future = self.get_robot_state_cli.call_async(state_req)
        state_future.add_done_callback(self._robot_state_cb)

        # 2) joint position 요청
        posj_req = GetCurrentPosj.Request()
        posj_future = self.get_posj_cli.call_async(posj_req)
        posj_future.add_done_callback(self._posj_cb)

        # 3) tcp pose 요청
        posx_req = GetCurrentPosx.Request()
        posx_req.ref = 0  # DR_BASE
        posx_future = self.get_posx_cli.call_async(posx_req)
        posx_future.add_done_callback(self._posx_cb)

        # 4) joint torque 요청
        torque_req = GetJointTorque.Request()
        torque_future = self.get_joint_torque_cli.call_async(torque_req)
        torque_future.add_done_callback(self._torque_cb)

    # ─────────────────────────────────────
    # Service callbacks
    # ─────────────────────────────────────
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

            if not resp.success:
                return

            self.current_joint = [float(v) for v in resp.pos[:6]]

            if len(self.current_joint) < 6:
                return

            if any(math.isnan(v) for v in self.current_joint):
                return

            msg = Float64MultiArray()
            msg.data = self.current_joint
            self.current_joint_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'get_current_posj failed: {e}')

    def _torque_cb(self, future):
        try:
            resp = future.result()

            if not resp.success or len(resp.jts) < 6:
                return

            torque = [float(v) for v in resp.jts[:6]]

            if any(math.isnan(v) for v in torque):
                return

            self.current_torque = torque

            msg = Float64MultiArray()
            msg.data = torque
            self.current_torque_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'get_joint_torque failed: {e}')

    def _posx_cb(self, future):
        try:
            resp = future.result()

            if resp.success and len(resp.task_pos_info) > 0:
                pose = list(resp.task_pos_info[0].data[:6])

                if len(pose) < 6:
                    return

                pose = [float(v) for v in pose]

                if any(math.isnan(v) for v in pose):
                    return

                self.current_tcp = pose

                msg = Float64MultiArray()
                msg.data = pose
                self.current_tcp_pub.publish(msg)

                self.check_workspace_limit()

        except Exception as e:
            self.get_logger().error(f'get_current_posx failed: {e}')

        finally:
            self.waiting_response = False

    # ─────────────────────────────────────
    # Safety check
    # ─────────────────────────────────────
    def check_workspace_limit(self):
        if self.current_tcp is None:
            return

        # 핵심: 작업 중이 아닐 때는 안전구역 검사로 estop을 보내지 않음
        if not self.monitor_enabled:
            return

        x, y, z, rx, ry, rz = self.current_tcp

        if not (self.min_x <= x <= self.max_x):
            self.publish_safety_estop(
                f'safety_limit_exceeded:x={x:.1f}, allowed=[{self.min_x:.1f},{self.max_x:.1f}]'
            )
            return

        if not (self.min_y <= y <= self.max_y):
            self.publish_safety_estop(
                f'safety_limit_exceeded:y={y:.1f}, allowed=[{self.min_y:.1f},{self.max_y:.1f}]'
            )
            return

        if not (self.min_z <= z <= self.max_z):
            self.publish_safety_estop(
                f'safety_limit_exceeded:z={z:.1f}, allowed=[{self.min_z:.1f},{self.max_z:.1f}]'
            )
            return

    def publish_safety_estop(self, reason: str):
        """
        안전구역 이탈 시:
        1. /fueling/safety_warning에 원인 문자열 발행
        2. /fueling/estop_request에 true 발행

        실제 action cancel과 move_stop은:
        SafetyMonitor → TaskManager → Commander Action cancel 흐름에서 처리
        """
        if self.estop_sent:
            return

        warning_msg = String()
        warning_msg.data = reason
        self.safety_warning_pub.publish(warning_msg)

        estop_msg = Bool()
        estop_msg.data = True
        self.estop_request_pub.publish(estop_msg)

        self.get_logger().error(f'{reason} -> published /fueling/estop_request')

        self.estop_sent = True

        # 한 번 E-STOP 요청을 보낸 뒤에는 추가 반복 발행을 막기 위해 감시 비활성화
        self.monitor_enabled = False


def main(args=None):
    rclpy.init(args=args)

    node = SafetyMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down safety monitor.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
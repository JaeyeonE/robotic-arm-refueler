import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from dsr_msgs2.srv import (
    MoveJoint, MoveLine, MoveStop, SetRobotMode, DrlStart,
)
from functools import partial
from collections import deque

from fuel_robot_pkg.gripper_drl_controller import DRL_BASE_CODE

# ── 그리퍼 stroke 값 ──
GRIP_OPEN = 0
GRIP_CAP = 800
GRIP_GUN = 700
GRIP_READY = 350

class DoosanCommanderNode(Node):
    def __init__(self):
        super().__init__('doosan_commander_node', namespace='dsr01')

        self.latest_fuel_port_pose = None
        self.latest_fuel_port_angle = None

        # ── 시퀀스 FSM 상태 ──
        self.step_queue = deque()
        self.total_steps = 0
        self.completed_steps = 0
        self.is_running = False
        self._wait_timer = None

        # ── Subscriptions ──
        self.cmd_sub = self.create_subscription(
            String, '/fueling/robot_cmd', self.cmd_callback, 10
        )
        self.pose_sub = self.create_subscription(
            Float64MultiArray, '/fueling/fuel_port_pose', self.pose_callback, 10
        )
        self.angle_sub = self.create_subscription(
            Float64MultiArray, '/fueling/fuel_port_angle', self.angle_callback, 10
        )

        # ── Publishers ──
        self.done_pub = self.create_publisher(String, '/fueling/cmd_done', 10)
        self.status_pub = self.create_publisher(String, '/fueling/status', 10)

        # ── Service Clients ──
        self.set_mode_cli = self.create_client(SetRobotMode, 'system/set_robot_mode')
        self.move_joint_cli = self.create_client(MoveJoint, 'motion/move_joint')
        self.move_line_cli = self.create_client(MoveLine, 'motion/move_line')
        self.move_stop_cli = self.create_client(MoveStop, 'motion/move_stop')
        self.drl_cli = self.create_client(DrlStart, 'drl/drl_start')

        self.wait_for_services()
        self.set_autonomous_mode()

        self.home_j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        self.get_logger().info('Doosan commander ready.')

    # ──────────────────────────────────────
    #  유틸리티
    # ──────────────────────────────────────
    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def wait_for_services(self):
        clients = [
            ('system/set_robot_mode', self.set_mode_cli),
            ('motion/move_joint',     self.move_joint_cli),
            ('motion/move_line',      self.move_line_cli),
            ('motion/move_stop',      self.move_stop_cli),
            ('drl/drl_start',         self.drl_cli),
        ]
        for name, cli in clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'waiting for {name}...')

    def set_autonomous_mode(self):
        req = SetRobotMode.Request()
        req.robot_mode = 1
        future = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Robot mode set request sent.')

    # ──────────────────────────────────────
    #  ROS2 콜백
    # ──────────────────────────────────────
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
    
    def angle_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 1:
            self.get_logger().warn('fuel_port_angle must contain exactly 1 value')
            return
        self.latest_fuel_port_angle = float(msg.data[0])
        self.get_logger().info(
            f'updated target angle -> angle={self.latest_fuel_port_angle:.1f}'
        )

    def cmd_callback(self, msg: String):
        cmd = msg.data

        if cmd == 'estop':
            self.execute_estop()

        elif cmd == 'go_home':
            self.execute_go_home()

        elif cmd == 'execute_fueling':
            self.execute_fueling()

        elif cmd == 'move_to_fuel_port':
            if self.latest_fuel_port_pose is None:
                self.get_logger().error('target pose not received yet')
                self.publish_status('target_pose_not_ready')
                return
            self._call_move_line(self.latest_fuel_port_pose, cmd, mode=0,
                                 callback=self._standalone_done_cb)

        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    # ──────────────────────────────────────
    #  E-STOP
    # ──────────────────────────────────────
    def execute_estop(self):
        self.get_logger().warn('E-STOP: clearing sequence and stopping')
        self.step_queue.clear()
        self.is_running = False
        self._cancel_wait_timer()

        req = MoveStop.Request()
        req.stop_mode = 0  # DR_QSTOP_STO
        future = self.move_stop_cli.call_async(req)
        future.add_done_callback(self._estop_done_cb)
        self.publish_status('estop_executed')

    def _cancel_wait_timer(self):
        if self._wait_timer is None:
            return
        try:
            self._wait_timer.cancel()
            self.destroy_timer(self._wait_timer)
        except Exception as e:
            self.get_logger().warn(f'wait timer cancel failed: {e}')
        finally:
            self._wait_timer = None

    def _estop_done_cb(self, future):
        try:
            resp = future.result()
            self.get_logger().warn(f'E-STOP result: success={resp.success}')
        except Exception as e:
            self.get_logger().error(f'E-STOP failed: {e}')

    # ──────────────────────────────────────
    #  Go Home (단독 명령)
    # ──────────────────────────────────────
    def execute_go_home(self):
        self.get_logger().info('Go home: enabling servo then move_joint')
        req = SetRobotMode.Request()
        req.robot_mode = 1
        future = self.set_mode_cli.call_async(req)
        future.add_done_callback(self._servo_on_then_home)

    def _servo_on_then_home(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'Servo ON result: success={resp.success}')
        except Exception as e:
            self.get_logger().error(f'Servo ON failed: {e}')
            return
        self._call_move_joint(self.home_j, 'go_home', mode=0,
                              callback=self._standalone_done_cb)

    def _standalone_done_cb(self, future, done_cmd='unknown'):
        """단독 명령(go_home, move_to_fuel_port 등)의 완료 콜백"""
        try:
            resp = future.result()
            self.get_logger().info(f'{done_cmd} finished, success={resp.success}')
            msg = String()
            msg.data = done_cmd
            self.done_pub.publish(msg)
            if resp.success:
                self.publish_status(f'{done_cmd}_success')
            else:
                self.publish_status(f'{done_cmd}_failed')
        except Exception as e:
            self.get_logger().error(f'{done_cmd} exception: {e}')
            self.publish_status(f'{done_cmd}_exception')

    # ──────────────────────────────────────
    #  모션 프리미티브 (공용)
    # ──────────────────────────────────────
    def _call_move_joint(self, joints, done_cmd, mode=0, callback=None):
        req = MoveJoint.Request()
        req.pos = joints
        req.vel = 30.0
        req.acc = 30.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = mode       # 0=절대, 1=상대
        req.blend_type = 0
        req.sync_type = 0
        future = self.move_joint_cli.call_async(req)
        cb = callback or self._step_done_cb
        future.add_done_callback(partial(cb, done_cmd=done_cmd))

    def _call_move_line(self, pose, done_cmd, mode=0, callback=None):
        req = MoveLine.Request()
        req.pos = pose
        req.vel = [30.0, 30.0]
        req.acc = [30.0, 30.0]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0
        req.mode = mode       # 0=절대, 1=상대
        req.blend_type = 0
        req.sync_type = 0
        future = self.move_line_cli.call_async(req)
        cb = callback or self._step_done_cb
        future.add_done_callback(partial(cb, done_cmd=done_cmd))

    def _call_gripper(self, stroke, done_cmd):
        script = DRL_BASE_CODE + f'\ngripper_move({stroke})\n'
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = script
        future = self.drl_cli.call_async(req)
        future.add_done_callback(partial(self._step_done_cb, done_cmd=done_cmd))

    def _call_wait(self, seconds, done_cmd):
        """지정된 시간(초) 동안 대기하는 타이머를 생성합니다."""
        self.get_logger().info(f'[SEQ] Waiting for {seconds}s... ({done_cmd})')
        self._cancel_wait_timer()
        self._wait_timer = self.create_timer(
            float(seconds),
            partial(self._wait_timer_cb, done_cmd=done_cmd)
        )

    def _wait_timer_cb(self, done_cmd):
        """타이머 알람이 울리면 실행되는 함수입니다."""
        self._cancel_wait_timer()
        # E-STOP이 wait 중에 들어왔을 경우 시퀀스 재개 차단
        if not self.is_running:
            self.get_logger().warn(f'[SEQ] {done_cmd} wait timer fired after abort — ignored')
            return
        self._step_done_cb(future=None, done_cmd=done_cmd)

    # ──────────────────────────────────────
    #  시퀀스 실행 엔진
    # ──────────────────────────────────────
    def _step_done_cb(self, future, done_cmd='step'):
        """시퀀스 스텝 완료 콜백 → 다음 스텝 실행"""
        # E-STOP이 이미 시퀀스를 중단시켰으면 콜백 무시
        if not self.is_running:
            self.get_logger().warn(f'[SEQ] {done_cmd} callback after abort — ignored')
            return
        try:
            if future is not None:
                resp = future.result()
                ok = resp.success if resp else False
            else:
                ok = True

            self.get_logger().info(f'[SEQ] {done_cmd} done, success={ok}')

            if not ok:
                self.get_logger().error(f'[SEQ] {done_cmd} FAILED — aborting sequence')
                self.step_queue.clear()
                self.is_running = False
                self.publish_status('fueling_error')
                return
        except Exception as e:
            self.get_logger().error(f'[SEQ] {done_cmd} exception: {e}')
            self.step_queue.clear()
            self.is_running = False
            self.publish_status('fueling_error')
            return

        self.completed_steps += 1
        # 진행도 publish: "fueling_progress:<완료>/<전체>"
        self.publish_status(
            f'fueling_progress:{self.completed_steps}/{self.total_steps}'
        )
        self._run_next_step()

    def _run_next_step(self):
        if not self.step_queue:
            self.get_logger().info('[SEQ] === Fueling sequence COMPLETE ===')
            self.is_running = False
            self.publish_status('fueling_complete')
            msg = String()
            msg.data = 'execute_fueling'
            self.done_pub.publish(msg)
            return

        step = self.step_queue.popleft()
        stype = step['type']
        desc = step.get('desc', f'step_{self.completed_steps + 1}')

        self.get_logger().info(
            f'[SEQ] ({self.completed_steps + 1}/{self.total_steps}) {desc}'
        )

        if stype == 'move_j':
            self._call_move_joint(step['pos'], desc, mode=step.get('mode', 0))
        elif stype == 'move_l':
            self._call_move_line(step['pos'], desc, mode=step.get('mode', 0))
        elif stype == 'gripper':
            self._call_gripper(step['stroke'], desc)
        elif stype == 'wait':
            self._call_wait(step['seconds'], desc)
        else:
            self.get_logger().error(f'Unknown step type: {stype}')
            self._run_next_step()

    # ──────────────────────────────────────
    #  주유 시퀀스 빌드 + 실행
    # ──────────────────────────────────────
    def execute_fueling(self):
        if self.is_running:
            self.get_logger().warn('Fueling sequence already running')
            return
        if self.latest_fuel_port_pose is None:
            self.get_logger().error('No fuel port pose — cannot start fueling')
            self.publish_status('target_pose_not_ready')
            return

        self.is_running = True
        self.completed_steps = 0

        pose = self.latest_fuel_port_pose  # [x, y, z, rx, ry, rz]

        # 동적 좌표 계산
        approach_pose = pose.copy()
        approach_pose[0] += 10
        approach_pose[1] += 85.0           # Step 2: y + 80mm + 5mm
        approach_pose[2] -= 10

        fuel_insert_pose = pose.copy()
        fuel_insert_pose[1] += 300.0       # Step 15: y + 300mm

        # handle_angle이 없으면 기존 하드코딩 기본값(38.56) 유지
        if self.latest_fuel_port_angle is None:
            final_angle = 38.56
            self.get_logger().warn(
                '[SEQ] handle_angle not received — using default J6=38.56'
            )
        elif self.latest_fuel_port_angle >= 0:
            
            final_angle = self.latest_fuel_port_angle - 90
        else:
            
            final_angle = self.latest_fuel_port_angle + 90

        self.step_queue = deque([
            # ═══ Phase 1: 캡 제거 (Step 1~8) ═══
            {'type': 'gripper', 'stroke': GRIP_OPEN,
             'desc': '01_gripper_open'},
            {'type': 'move_j',  'pos': self.home_j,
             'desc': '01_home_position'},
            {'type': 'move_j',  'pos': [16.73, 33.71, 38.04, 96.83, -85.52, -9.42],
             'desc': '01-1_avoid_waypoint_1'},
            {'type': 'move_j',  'pos': [-4.34, 37.27, 91.2, 87.3, -86.61, 38.56],
             'desc': '01-1_avoid_waypoint_2'},
            {'type': 'gripper', 'stroke': GRIP_READY,
             'desc': '01_gripper_ready'},

            {'type': 'move_l',  'pos': approach_pose,
             'desc': '02_approach_fuel_port_y+80'},
            {'type': 'move_j',  'pos': [0.0, 0.0, 0.0, 0.0, 0.0, final_angle], 'mode': 1,
             'desc': '02-1_rotate_gripper'}, 
            {'type': 'move_l',  'pos': [0.0, -80.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '03_insert_y-80'},
            {'type': 'wait', 'seconds': 0.5, 'desc': 'wait01'},

            {'type': 'gripper', 'stroke': GRIP_CAP,
             'desc': '04_grip_gas_cap'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait01_1'},
            {'type': 'move_j',  'pos': [0.0, 0.0, 0.0, 0.0, 0.0, -90.0], 'mode': 1,
             'desc': '05_unscrew_j6-90'},
            {'type': 'move_l',  'pos': [0.0, 80.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '06_retreat_y+80'},

            {'type': 'move_j',  'pos': [-11.28, 29.99, 58.06, 180.0, -91.94, -11.29],
             'desc': '07_x_axis_align'},
            {'type': 'move_l',  'pos': [0.0, 0.0, -217.78, 0.0, 0.0, 0.0], 'mode' : 1,
             'desc': '07-1_place_cap_down'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait02'},
            {'type': 'gripper', 'stroke': GRIP_READY,
             'desc': '08_release_cap'},
             {'type': 'wait', 'seconds': 1.0, 'desc': 'wait02_1'},

            # ═══ Phase 2: 주유건 픽업 (Step 9~13) ═══
            {'type': 'move_l',  'pos': [0.0, 0.0, 300.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '09_up_z+300'},
            {'type': 'gripper', 'stroke': GRIP_OPEN,
             'desc': '09_1_open_more'},
            {'type': 'move_j',  'pos': [8.17, 40.77, 21.86, 96.84, -96.83, -9.42],
             'desc': '10_gun_waypoint_1'},
            {'type': 'move_j',  'pos': [-38.45, -0.81, 120.63, 57.03, 45.0, 0.0],
             'desc': '10_gun_waypoint_2'},
            {'type': 'move_j',  'pos': [-28.36, 63.83, 54.62, 74.22, 116.64, -32.22],
             'desc': '10_gun_waypoint_3'},
            {'type': 'move_l',  'pos': [0.0, 100.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '11_gun_approach_y+100'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait04'},
            {'type': 'gripper', 'stroke': GRIP_GUN,
             'desc': '12_grip_fuel_gun'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait04_1'},
            {'type': 'move_l',  'pos': [0.0, -40.0, 70.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '13_lift_gun_y-40_z+70'},

            # ═══ Phase 3: 주유건 삽입 + 주유 (Step 14~15) ═══
            {'type': 'move_j',  'pos': [-17.65, 31.58, 65.69, 85.49, 106.69, -9.58],
             'desc': '14_to_port_wp1'},
            {'type': 'move_j',  'pos': [52.38, 10.46, 77.5, 100.99, -64.29, -9.6],
             'desc': '14_to_port_wp2'},
            {'type': 'move_j',  'pos': [16.53, 37.18, 95.25, 101.33, -102.13, 43.64],
             'desc': '14_to_port_wp3'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait05'},

            {'type': 'move_l',  'pos': fuel_insert_pose,
             'desc': '15_fuel_approach_y+300'},
            {'type': 'move_l',  'pos': [0.0, 0.0, -20.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '15_insert_gun_z-35'},
            {'type': 'move_l',  'pos': [0.0, -150.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '16_insert_gun_y-150'},             
            {'type': 'wait', 'seconds': 4.0, 'desc': 'wait05-1'},

            # ═══ Phase 4: 주유 후 주유건 재거치 (Step 16~19) ═══
            {'type': 'move_l',  'pos': [0.0, 150.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '16_retreat_y+150'},

            {'type': 'move_j',  'pos': [37.37, -21.49, 133.03, 78.64, -69.04, 32.08],
             'desc': '17_remount_wp1'},
            {'type': 'move_j',  'pos': [-24.63, 34.65, 57.44, 85.63, 112.46, -4.84],
             'desc': '17_remount_wp2'},
            {'type': 'move_j',  'pos': [-20.98, 55.77, 69.95, 75.92, 106.53, -44.1],
             'desc': '17_remount_wp3'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait06'},

            {'type': 'gripper', 'stroke': GRIP_OPEN,
             'desc': '18_release_fuel_gun'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait06_1'},
            {'type': 'move_l',  'pos': [0.0, -40.0, 70.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '19_retreat_z+100'},
            {'type': 'gripper', 'stroke': GRIP_READY,
             'desc': '02_gripper_ready'},

            # ═══ Phase 5: 캡 재장착 (Step 20~28) ═══
            {'type': 'move_j',  'pos': [-11.28, 29.99, 58.06, 180.0, -91.94, -11.29],
             'desc': '20_x_axis_align'},
            {'type': 'move_l',  'pos': [0.0, 0.0, -217.78, 0.0, 0.0, 0.0], 'mode' : 1,
             'desc': '07-3_place_cap_down'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait07'},
            {'type': 'gripper', 'stroke': GRIP_CAP,
             'desc': '22_grip_cap'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait07_1'}, 

            {'type': 'move_l',  'pos': [0.0, 0.0, 100.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '23_lift_z+150'},
            {'type': 'move_j',  'pos': [-4.34, 37.27, 91.2, 87.3, -86.61, -51.44],
             'desc': '24_cap_align'},
            {'type': 'move_l',  'pos': approach_pose,
             'desc': '24-1_cap_absolute_align'},
            {'type': 'move_l',  'pos': [0.0, -80.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '25_cap_approach_y-80'},
            {'type': 'move_j',  'pos': [0.0, 0.0, 0.0, 0.0, 0.0, 90.0], 'mode': 1,
             'desc': '26_screw_j6+90'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait08'},

            {'type': 'gripper', 'stroke': GRIP_READY,
             'desc': '27_release_cap'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait08_1'}, 
            {'type': 'move_l',  'pos': [0.0, 100.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1,
             'desc': '28_retreat_y+100'},

            # ═══ Phase 6: 복귀 (Step 29) ═══
            {'type': 'move_j',  'pos': self.home_j,
             'desc': '29_go_home'},
        ])

        self.total_steps = len(self.step_queue)
        self.get_logger().info(
            f'[SEQ] === Starting fueling sequence: {self.total_steps} actions ==='
        )
        self.publish_status('fueling_started')
        self._run_next_step()


def main(args=None):
    rclpy.init(args=args)
    node = DoosanCommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

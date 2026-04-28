import asyncio
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

from dsr_msgs2.srv import (
    MoveJoint,
    MoveLine,
    MoveStop,
    SetRobotMode,
    DrlStart,
)

from fuel_robot_interfaces.action import FuelTask
from fuel_robot_pkg.gripper_drl_controller import DRL_BASE_CODE


# ──────────────────────────────────────
#  Gripper stroke values
# ──────────────────────────────────────
GRIP_OPEN = 0
GRIP_CAP = 800
GRIP_GUN = 700
GRIP_READY = 350


class DoosanCommanderNode(Node):
    """
    Doosan E0509 commander node.

    Action 전환 원칙:
    - 로봇 실행 명령은 /dsr01/fuel_task Action으로만 받는다.
    - /fueling/robot_cmd, /fueling/fuel_port_pose, /fueling/fuel_port_angle은 구독하지 않는다.
    - TaskManager가 Action Client로 goal을 보내고, 이 노드는 Action Server로 실제 로봇 동작만 수행한다.
    """

    def __init__(self):
        super().__init__('doosan_commander_node', namespace='dsr01')

        # ── Robot default pose ──
        self.home_j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        # ── Runtime state ──
        self.is_running = False
        self.estop_requested = False
        self.current_command: Optional[str] = None

        # Commander는 status의 주인이 아니므로 최소한의 상태만 발행한다.
        # 실제 UI용 progress/status는 TaskManager가 action feedback/result를 받아서 발행하는 구조를 권장한다.
        self.status_pub = self.create_publisher(String, '/fueling/status', 10)

        # ── Doosan service clients ──
        self.set_mode_cli = self.create_client(SetRobotMode, 'system/set_robot_mode')
        self.move_joint_cli = self.create_client(MoveJoint, 'motion/move_joint')
        self.move_line_cli = self.create_client(MoveLine, 'motion/move_line')
        self.move_stop_cli = self.create_client(MoveStop, 'motion/move_stop')
        self.drl_cli = self.create_client(DrlStart, 'drl/drl_start')

        self.wait_for_services()

        # ── Action Server ──
        # namespace='dsr01' 이므로 실제 Action 이름은 /dsr01/fuel_task
        self._action_server = ActionServer(
            self,
            FuelTask,
            'fuel_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Doosan Commander Action Server ready: /dsr01/fuel_task')

    # ──────────────────────────────────────
    #  Utility
    # ──────────────────────────────────────
    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def wait_for_services(self):
        clients = [
            ('system/set_robot_mode', self.set_mode_cli),
            ('motion/move_joint', self.move_joint_cli),
            ('motion/move_line', self.move_line_cli),
            ('motion/move_stop', self.move_stop_cli),
            ('drl/drl_start', self.drl_cli),
        ]

        for name, cli in clients:
            while rclpy.ok() and not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'waiting for {name}...')

    async def set_autonomous_mode(self) -> bool:
        """
        goal 실행 직전에 autonomous mode 요청.
        __init__에서 한 번만 호출하는 것보다 goal 시작 전에 확인하는 편이 안전하다.
        """
        try:
            req = SetRobotMode.Request()
            req.robot_mode = 1

            future = self.set_mode_cli.call_async(req)
            response = await future

            success = bool(getattr(response, 'success', True))
            if success:
                self.get_logger().info('Robot mode set to autonomous.')
            else:
                self.get_logger().error(f'Failed to set robot mode: {response}')

            return success

        except Exception as e:
            self.get_logger().error(f'set_autonomous_mode exception: {e}')
            return False

    # ──────────────────────────────────────
    #  Action callbacks
    # ──────────────────────────────────────
    def goal_callback(self, goal_request):
        command = goal_request.command.strip()

        if self.is_running:
            self.get_logger().warn(
                f'Reject goal: sequence already running. requested={command}'
            )
            return GoalResponse.REJECT

        allowed_commands = {
            'go_home',
            'execute_fueling',

            # 작업별 단독 테스트용 command
            'test_gripper_open',
            'test_gripper_ready',
            'test_gripper_cap',
            'test_gripper_gun',
            'test_approach_fuel_port',
        }

        if command not in allowed_commands:
            self.get_logger().warn(f'Reject goal: unknown command={command}')
            return GoalResponse.REJECT

        self.get_logger().info(f'Accept goal: {command}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        TaskManager가 /fueling/estop_request를 받으면 cancel_goal_async()를 호출할 예정.
        cancel 요청이 들어오면 즉시 MoveStop을 보낸다.
        """
        self.get_logger().warn('Cancel requested -> execute E-STOP')
        self.execute_estop()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        command = goal_handle.request.command.strip()
        self.current_command = command
        self.is_running = True
        self.estop_requested = False

        try:
            steps = self.build_steps_from_goal(goal_handle)

            if not steps:
                self.is_running = False
                goal_handle.abort()
                return FuelTask.Result(
                    success=False,
                    message=f'empty_or_invalid_steps:{command}',
                )

            mode_ok = await self.set_autonomous_mode()
            if not mode_ok:
                self.is_running = False
                goal_handle.abort()
                return FuelTask.Result(
                    success=False,
                    message='failed_to_set_autonomous_mode',
                )

            total_steps = len(steps)
            feedback_msg = FuelTask.Feedback()

            self.get_logger().info(
                f'[ACTION] Start command={command}, total_steps={total_steps}'
            )

            for idx, step in enumerate(steps, start=1):
                # cancel은 step 시작 전에도 확인
                if goal_handle.is_cancel_requested or self.estop_requested:
                    self.get_logger().warn('[ACTION] Goal canceled before step execution')
                    self.execute_estop()
                    self.is_running = False
                    goal_handle.canceled()
                    return FuelTask.Result(success=False, message='task_canceled')

                desc = step.get('desc', f'step_{idx}')

                feedback_msg.current_step = idx
                feedback_msg.total_steps = total_steps
                feedback_msg.status_desc = desc
                goal_handle.publish_feedback(feedback_msg)

                self.get_logger().info(f'[SEQ] ({idx}/{total_steps}) {desc}')

                success = await self.run_step(step)

                # cancel은 step이 진행 중일 때 들어올 수 있으므로 step 이후에도 다시 확인
                if goal_handle.is_cancel_requested or self.estop_requested:
                    self.get_logger().warn('[ACTION] Goal canceled during step execution')
                    self.execute_estop()
                    self.is_running = False
                    goal_handle.canceled()
                    return FuelTask.Result(success=False, message='task_canceled')

                if not success:
                    self.is_running = False
                    goal_handle.abort()

                    error_msg = f'failed_at:{idx}/{total_steps}:{desc}'
                    self.get_logger().error(f'[ACTION] {error_msg}')
                    return FuelTask.Result(success=False, message=error_msg)

            self.is_running = False
            goal_handle.succeed()

            self.get_logger().info(f'[ACTION] Command complete: {command}')
            return FuelTask.Result(success=True, message=f'{command}_completed')

        except Exception as e:
            self.is_running = False
            self.get_logger().error(f'execute_callback exception: {e}')
            goal_handle.abort()
            return FuelTask.Result(success=False, message=f'exception:{type(e).__name__}:{e}')

        finally:
            self.current_command = None

    # ──────────────────────────────────────
    #  Build step list
    # ──────────────────────────────────────
    def build_steps_from_goal(self, goal_handle) -> List[Dict[str, Any]]:
        command = goal_handle.request.command.strip()

        if command == 'go_home':
            return self.build_go_home_sequence()

        if command == 'execute_fueling':
            target_pose = list(goal_handle.request.target_pose)

            if len(target_pose) != 6:
                self.get_logger().error(
                    f'execute_fueling requires target_pose length 6, got {len(target_pose)}'
                )
                return []

            handle_angle = None
            if goal_handle.request.use_handle_angle:
                handle_angle = float(goal_handle.request.handle_angle)

            return self.build_fueling_sequence(target_pose, handle_angle)

        # ── 작업별 테스트 command ──
        if command == 'test_gripper_open':
            return [{'type': 'gripper', 'stroke': GRIP_OPEN, 'desc': 'test_gripper_open'}]

        if command == 'test_gripper_ready':
            return [{'type': 'gripper', 'stroke': GRIP_READY, 'desc': 'test_gripper_ready'}]

        if command == 'test_gripper_cap':
            return [{'type': 'gripper', 'stroke': GRIP_CAP, 'desc': 'test_gripper_cap'}]

        if command == 'test_gripper_gun':
            return [{'type': 'gripper', 'stroke': GRIP_GUN, 'desc': 'test_gripper_gun'}]

        if command == 'test_approach_fuel_port':
            target_pose = list(goal_handle.request.target_pose)

            if len(target_pose) != 6:
                self.get_logger().error(
                    f'test_approach_fuel_port requires target_pose length 6, got {len(target_pose)}'
                )
                return []

            approach_pose = target_pose.copy()
            approach_pose[0] += 10.0
            approach_pose[1] += 85.0
            approach_pose[2] -= 10.0

            return [
                {'type': 'move_j', 'pos': self.home_j, 'desc': 'test_home_before_approach'},
                {'type': 'move_l', 'pos': approach_pose, 'desc': 'test_approach_fuel_port'},
            ]

        return []

    def build_go_home_sequence(self) -> List[Dict[str, Any]]:
        return [
            {'type': 'move_j', 'pos': self.home_j, 'desc': 'go_home'}
        ]

    def build_fueling_sequence(
        self,
        pose: List[float],
        handle_angle: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        pose: [x, y, z, rx, ry, rz]
        단위는 기존 Doosan commander와 동일하게 mm, deg 기준으로 사용.
        """

        approach_pose = pose.copy()
        approach_pose[0] += 10.0
        approach_pose[1] += 85.0
        approach_pose[2] -= 10.0

        fuel_insert_pose = pose.copy()
        fuel_insert_pose[1] += 300.0

        if handle_angle is None:
            final_angle = 38.56
            self.get_logger().warn(
                '[SEQ] handle_angle not received — using default J6=38.56'
            )
        elif handle_angle >= 0:
            final_angle = handle_angle - 90.0
        else:
            final_angle = handle_angle + 90.0

        return [
            # ═══ Phase 1: 캡 제거 ═══
            {'type': 'gripper', 'stroke': GRIP_OPEN, 'desc': '01_gripper_open'},
            {'type': 'move_j', 'pos': self.home_j, 'desc': '01_home_position'},
            {'type': 'move_j', 'pos': [16.73, 33.71, 38.04, 96.83, -85.52, -9.42],
             'desc': '01-1_avoid_waypoint_1'},
            {'type': 'move_j', 'pos': [-4.34, 37.27, 91.2, 87.3, -86.61, 38.56],
             'desc': '01-1_avoid_waypoint_2'},
            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '01_gripper_ready'},

            {'type': 'move_l', 'pos': approach_pose, 'desc': '02_approach_fuel_port_y+80'},
            {'type': 'move_j', 'pos': [0.0, 0.0, 0.0, 0.0, 0.0, final_angle],
             'mode': 1, 'desc': '02-1_rotate_gripper'},
            {'type': 'move_l', 'pos': [0.0, -80.0, 0.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '03_insert_y-80'},
            {'type': 'wait', 'seconds': 0.5, 'desc': 'wait01'},

            {'type': 'gripper', 'stroke': GRIP_CAP, 'desc': '04_grip_gas_cap'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait01_1'},
            {'type': 'move_j', 'pos': [0.0, 0.0, 0.0, 0.0, 0.0, -90.0],
             'mode': 1, 'desc': '05_unscrew_j6-90'},
            {'type': 'move_l', 'pos': [0.0, 80.0, 0.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '06_retreat_y+80'},

            {'type': 'move_j', 'pos': [-11.28, 29.99, 58.06, 180.0, -91.94, -11.29],
             'desc': '07_x_axis_align'},
            {'type': 'move_l', 'pos': [0.0, 0.0, -217.78, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '07-1_place_cap_down'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait02'},
            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '08_release_cap'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait02_1'},

            # ═══ Phase 2: 주유건 픽업 ═══
            {'type': 'move_l', 'pos': [0.0, 0.0, 300.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '09_up_z+300'},
            {'type': 'gripper', 'stroke': GRIP_OPEN, 'desc': '09_1_open_more'},
            {'type': 'move_j', 'pos': [8.17, 40.77, 21.86, 96.84, -96.83, -9.42],
             'desc': '10_gun_waypoint_1'},
            {'type': 'move_j', 'pos': [-38.45, -0.81, 120.63, 57.03, 45.0, 0.0],
             'desc': '10_gun_waypoint_2'},
            {'type': 'move_j', 'pos': [-28.36, 63.83, 54.62, 74.22, 116.64, -32.22],
             'desc': '10_gun_waypoint_3'},
            {'type': 'move_l', 'pos': [0.0, 100.0, 0.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '11_gun_approach_y+100'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait04'},
            {'type': 'gripper', 'stroke': GRIP_GUN, 'desc': '12_grip_fuel_gun'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait04_1'},
            {'type': 'move_l', 'pos': [0.0, -40.0, 70.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '13_lift_gun_y-40_z+70'},

            # ═══ Phase 3: 주유건 삽입 ═══
            {'type': 'move_j', 'pos': [-17.65, 31.58, 65.69, 85.49, 106.69, -9.58],
             'desc': '14_to_port_wp1'},
            {'type': 'move_j', 'pos': [52.38, 10.46, 77.5, 100.99, -64.29, -9.6],
             'desc': '14_to_port_wp2'},
            {'type': 'move_j', 'pos': [16.53, 37.18, 95.25, 101.33, -102.13, 43.64],
             'desc': '14_to_port_wp3'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait05'},

            {'type': 'move_l', 'pos': fuel_insert_pose, 'desc': '15_fuel_approach_y+300'},
            {'type': 'move_l', 'pos': [0.0, 0.0, -20.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '15_insert_gun_z-35'},
            {'type': 'move_l', 'pos': [0.0, -150.0, 0.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '16_insert_gun_y-150'},
            {'type': 'wait', 'seconds': 4.0, 'desc': 'wait05-1'},

            # ═══ Phase 4: 주유 후 주유건 재거치 ═══
            {'type': 'move_l', 'pos': [0.0, 150.0, 0.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '16_retreat_y+150'},
            {'type': 'move_j', 'pos': [37.37, -21.49, 133.03, 78.64, -69.04, 32.08],
             'desc': '17_remount_wp1'},
            {'type': 'move_j', 'pos': [-24.63, 34.65, 57.44, 85.63, 112.46, -4.84],
             'desc': '17_remount_wp2'},
            {'type': 'move_j', 'pos': [-20.98, 55.77, 69.95, 75.92, 106.53, -44.1],
             'desc': '17_remount_wp3'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait06'},

            {'type': 'gripper', 'stroke': GRIP_OPEN, 'desc': '18_release_fuel_gun'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait06_1'},
            {'type': 'move_l', 'pos': [0.0, -40.0, 70.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '19_retreat_z+100'},
            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '19_gripper_ready_after_gun'},

            # ═══ Phase 5: 캡 재장착 ═══
            {'type': 'move_j', 'pos': [-11.28, 29.99, 58.06, 180.0, -91.94, -11.29],
             'desc': '20_x_axis_align'},
            {'type': 'move_l', 'pos': [0.0, 0.0, -217.78, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '21_place_cap_down_again'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait07'},
            {'type': 'gripper', 'stroke': GRIP_CAP, 'desc': '22_grip_cap'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait07_1'},

            {'type': 'move_l', 'pos': [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '23_lift_z+100'},
            {'type': 'move_j', 'pos': [-4.34, 37.27, 91.2, 87.3, -86.61, -51.44],
             'desc': '24_cap_align'},
            {'type': 'move_l', 'pos': approach_pose, 'desc': '24-1_cap_absolute_align'},
            {'type': 'move_l', 'pos': [0.0, -80.0, 0.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '25_cap_approach_y-80'},
            {'type': 'move_j', 'pos': [0.0, 0.0, 0.0, 0.0, 0.0, 90.0],
             'mode': 1, 'desc': '26_screw_j6+90'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait08'},

            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '27_release_cap'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait08_1'},
            {'type': 'move_l', 'pos': [0.0, 100.0, 0.0, 0.0, 0.0, 0.0],
             'mode': 1, 'desc': '28_retreat_y+100'},

            # ═══ Phase 6: 홈 복귀 ═══
            {'type': 'move_j', 'pos': self.home_j, 'desc': '29_go_home'},
        ]

    # ──────────────────────────────────────
    #  Run one step
    # ──────────────────────────────────────
    async def run_step(self, step: Dict[str, Any]) -> bool:
        stype = step.get('type')
        desc = step.get('desc', 'unknown_step')

        try:
            if stype == 'move_j':
                return await self._run_move_joint(step, desc)

            if stype == 'move_l':
                return await self._run_move_line(step, desc)

            if stype == 'gripper':
                return await self._run_gripper(step, desc)

            if stype == 'wait':
                seconds = float(step.get('seconds', 0.0))
                return self._run_wait(seconds, desc)

            self.get_logger().error(f'Unknown step type: {stype}, desc={desc}')
            return False

        except Exception as e:
            self.get_logger().error(
                f'run_step exception: type={stype}, desc={desc}, error={e}'
            )
            return False
        
    def _run_wait(self, seconds: float, desc: str) -> bool:
        """
        rclpy ActionServer 내부에서는 asyncio.sleep 대신 time.sleep 기반으로 대기한다.
        짧게 쪼개서 대기하면 E-STOP cancel 요청도 중간에 반응할 수 있다.
        """
        import time

        self.get_logger().info(f'[WAIT] {desc}: {seconds:.2f}s')

        start_time = time.time()
        check_interval = 0.05

        while time.time() - start_time < seconds:
            if self.estop_requested or not self.is_running:
                self.get_logger().warn(f'[WAIT] {desc} aborted during wait')
                return False

            time.sleep(check_interval)

        return True

    async def _run_move_joint(self, step: Dict[str, Any], desc: str) -> bool:
        req = MoveJoint.Request()
        req.pos = [float(v) for v in step['pos']]
        req.vel = float(step.get('vel', 30.0))
        req.acc = float(step.get('acc', 30.0))
        req.time = float(step.get('time', 0.0))
        req.radius = float(step.get('radius', 0.0))
        req.mode = int(step.get('mode', 0))
        req.blend_type = int(step.get('blend_type', 0))
        req.sync_type = int(step.get('sync_type', 0))

        future = self.move_joint_cli.call_async(req)
        response = await future

        success = bool(getattr(response, 'success', False))
        if not success:
            self.get_logger().error(f'[MOVE_J FAILED] desc={desc}, response={response}')
        return success

    async def _run_move_line(self, step: Dict[str, Any], desc: str) -> bool:
        req = MoveLine.Request()
        req.pos = [float(v) for v in step['pos']]
        req.vel = [float(v) for v in step.get('vel', [30.0, 30.0])]
        req.acc = [float(v) for v in step.get('acc', [30.0, 30.0])]
        req.time = float(step.get('time', 0.0))
        req.radius = float(step.get('radius', 0.0))
        req.ref = int(step.get('ref', 0))
        req.mode = int(step.get('mode', 0))
        req.blend_type = int(step.get('blend_type', 0))
        req.sync_type = int(step.get('sync_type', 0))

        future = self.move_line_cli.call_async(req)
        response = await future

        success = bool(getattr(response, 'success', False))
        if not success:
            self.get_logger().error(f'[MOVE_L FAILED] desc={desc}, response={response}')
        return success

    async def _run_gripper(self, step: Dict[str, Any], desc: str) -> bool:
        stroke = int(step['stroke'])
        script = DRL_BASE_CODE + f'\ngripper_move({stroke})\n'

        req = DrlStart.Request()
        req.robot_system = 0
        req.code = script

        future = self.drl_cli.call_async(req)
        response = await future

        success = bool(getattr(response, 'success', False))
        if not success:
            self.get_logger().error(
                f'[GRIPPER FAILED] desc={desc}, stroke={stroke}, response={response}'
            )
        return success

    # ──────────────────────────────────────
    #  E-STOP
    # ──────────────────────────────────────
    def execute_estop(self):
        if self.estop_requested:
            self.get_logger().warn('E-STOP already requested. Skip duplicate stop.')
            return

        self.estop_requested = True
        self.is_running = False

        req = MoveStop.Request()
        req.stop_mode = 0

        future = self.move_stop_cli.call_async(req)
        future.add_done_callback(self._move_stop_done_callback)

        self.publish_status('estop_executed')
        self.get_logger().warn('E-STOP command sent.')

    def _move_stop_done_callback(self, future):
        try:
            response = future.result()
            success = bool(getattr(response, 'success', False))
            self.get_logger().warn(f'E-STOP response: success={success}, response={response}')
        except Exception as e:
            self.get_logger().error(f'E-STOP service exception: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = DoosanCommanderNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt. Shutting down commander.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
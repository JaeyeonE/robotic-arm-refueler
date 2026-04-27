import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from dsr_msgs2.srv import (
    MoveJoint, MoveLine, MoveStop, SetRobotMode, DrlStart,
)

from fuel_robot_interfaces.action import FuelTask
from fuel_robot_pkg.gripper_drl_controller import DRL_BASE_CODE


GRIP_OPEN = 0
GRIP_CAP = 800
GRIP_GUN = 700
GRIP_READY = 350


class DoosanCommanderNode(Node):
    def __init__(self):
        super().__init__('doosan_commander_node', namespace='dsr01')

        self.home_j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.is_running = False

        self.status_pub = self.create_publisher(String, '/fueling/status', 10)

        self.set_mode_cli = self.create_client(SetRobotMode, 'system/set_robot_mode')
        self.move_joint_cli = self.create_client(MoveJoint, 'motion/move_joint')
        self.move_line_cli = self.create_client(MoveLine, 'motion/move_line')
        self.move_stop_cli = self.create_client(MoveStop, 'motion/move_stop')
        self.drl_cli = self.create_client(DrlStart, 'drl/drl_start')

        self.wait_for_services()
        self.set_autonomous_mode()

        self._action_server = ActionServer(
            self,
            FuelTask,
            'fuel_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Doosan Commander Action Server ready.')

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
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'waiting for {name}...')

    def set_autonomous_mode(self):
        req = SetRobotMode.Request()
        req.robot_mode = 1
        self.set_mode_cli.call_async(req)
        self.get_logger().info('Robot mode set request sent.')

    def goal_callback(self, goal_request):
        if self.is_running:
            self.get_logger().warn('Reject goal: sequence already running')
            return GoalResponse.REJECT

        self.get_logger().info(f'Received goal: {goal_request.command}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Cancel requested -> E-STOP')
        self.execute_estop()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        command = goal_handle.request.command

        if command == 'go_home':
            steps = [
                {'type': 'move_j', 'pos': self.home_j, 'desc': 'go_home'}
            ]

        elif command == 'execute_fueling':
            target_pose = list(goal_handle.request.target_pose)

            if len(target_pose) != 6:
                goal_handle.abort()
                return FuelTask.Result(success=False, message='invalid_target_pose')

            handle_angle = None
            if goal_handle.request.use_handle_angle:
                handle_angle = float(goal_handle.request.handle_angle)

            steps = self.build_fueling_sequence(target_pose, handle_angle)

        else:
            goal_handle.abort()
            return FuelTask.Result(success=False, message=f'unknown_command:{command}')

        self.is_running = True
        total_steps = len(steps)
        feedback_msg = FuelTask.Feedback()

        self.publish_status('fueling_started')
        self.get_logger().info(f'[SEQ] Starting {command}: {total_steps} steps')

        for idx, step in enumerate(steps, start=1):
            if goal_handle.is_cancel_requested:
                self.execute_estop()
                self.is_running = False
                goal_handle.canceled()
                return FuelTask.Result(success=False, message='task_canceled')

            desc = step.get('desc', f'step_{idx}')

            feedback_msg.current_step = idx
            feedback_msg.total_steps = total_steps
            feedback_msg.status_desc = desc
            goal_handle.publish_feedback(feedback_msg)

            if command == 'go_home':
                self.publish_status(f'go_home_progress:{idx}/{total_steps}')
            else:
                self.publish_status(f'fueling_progress:{idx}/{total_steps}')

            self.get_logger().info(f'[SEQ] ({idx}/{total_steps}) {desc}')

            success = await self.run_step(step)

            if not success:
                self.is_running = False
                goal_handle.abort()
                self.publish_status('fueling_error')
                return FuelTask.Result(success=False, message=f'failed_at:{desc}')

        self.is_running = False
        goal_handle.succeed()

        if command == 'go_home':
            self.publish_status('go_home_success')
        else:
            self.publish_status('fueling_complete')

        self.get_logger().info('[SEQ] === Sequence COMPLETE ===')
        return FuelTask.Result(success=True, message='all_steps_completed')

    async def run_step(self, step: dict) -> bool:
        stype = step['type']

        try:
            if stype == 'move_j':
                req = MoveJoint.Request()
                req.pos = step['pos']
                req.vel = 30.0
                req.acc = 30.0
                req.time = 0.0
                req.radius = 0.0
                req.mode = step.get('mode', 0)
                req.blend_type = 0
                req.sync_type = 0

                future = self.move_joint_cli.call_async(req)
                response = await future
                return response.success

            if stype == 'move_l':
                req = MoveLine.Request()
                req.pos = step['pos']
                req.vel = [30.0, 30.0]
                req.acc = [30.0, 30.0]
                req.time = 0.0
                req.radius = 0.0
                req.ref = 0
                req.mode = step.get('mode', 0)
                req.blend_type = 0
                req.sync_type = 0

                future = self.move_line_cli.call_async(req)
                response = await future
                return response.success

            if stype == 'gripper':
                script = DRL_BASE_CODE + f'\ngripper_move({step["stroke"]})\n'

                req = DrlStart.Request()
                req.robot_system = 0
                req.code = script

                future = self.drl_cli.call_async(req)
                response = await future
                return response.success

            if stype == 'wait':
                await asyncio.sleep(float(step['seconds']))
                return True

            self.get_logger().error(f'Unknown step type: {stype}')
            return False

        except Exception as e:
            self.get_logger().error(f'run_step exception [{stype}]: {e}')
            return False

    def execute_estop(self):
        req = MoveStop.Request()
        req.stop_mode = 0
        self.move_stop_cli.call_async(req)

        self.is_running = False
        self.publish_status('estop_executed')
        self.get_logger().warn('E-STOP executed')

    def build_fueling_sequence(self, pose, handle_angle=None):
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
            {'type': 'gripper', 'stroke': GRIP_OPEN, 'desc': '01_gripper_open'},
            {'type': 'move_j', 'pos': self.home_j, 'desc': '01_home_position'},
            {'type': 'move_j', 'pos': [16.73, 33.71, 38.04, 96.83, -85.52, -9.42], 'desc': '01-1_avoid_waypoint_1'},
            {'type': 'move_j', 'pos': [-4.34, 37.27, 91.2, 87.3, -86.61, 38.56], 'desc': '01-1_avoid_waypoint_2'},
            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '01_gripper_ready'},

            {'type': 'move_l', 'pos': approach_pose, 'desc': '02_approach_fuel_port_y+80'},
            {'type': 'move_j', 'pos': [0.0, 0.0, 0.0, 0.0, 0.0, final_angle], 'mode': 1, 'desc': '02-1_rotate_gripper'},
            {'type': 'move_l', 'pos': [0.0, -80.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '03_insert_y-80'},
            {'type': 'wait', 'seconds': 0.5, 'desc': 'wait01'},

            {'type': 'gripper', 'stroke': GRIP_CAP, 'desc': '04_grip_gas_cap'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait01_1'},
            {'type': 'move_j', 'pos': [0.0, 0.0, 0.0, 0.0, 0.0, -90.0], 'mode': 1, 'desc': '05_unscrew_j6-90'},
            {'type': 'move_l', 'pos': [0.0, 80.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '06_retreat_y+80'},

            {'type': 'move_j', 'pos': [-11.28, 29.99, 58.06, 180.0, -91.94, -11.29], 'desc': '07_x_axis_align'},
            {'type': 'move_l', 'pos': [0.0, 0.0, -217.78, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '07-1_place_cap_down'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait02'},
            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '08_release_cap'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait02_1'},

            {'type': 'move_l', 'pos': [0.0, 0.0, 300.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '09_up_z+300'},
            {'type': 'gripper', 'stroke': GRIP_OPEN, 'desc': '09_1_open_more'},
            {'type': 'move_j', 'pos': [8.17, 40.77, 21.86, 96.84, -96.83, -9.42], 'desc': '10_gun_waypoint_1'},
            {'type': 'move_j', 'pos': [-38.45, -0.81, 120.63, 57.03, 45.0, 0.0], 'desc': '10_gun_waypoint_2'},
            {'type': 'move_j', 'pos': [-28.36, 63.83, 54.62, 74.22, 116.64, -32.22], 'desc': '10_gun_waypoint_3'},
            {'type': 'move_l', 'pos': [0.0, 100.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '11_gun_approach_y+100'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait04'},
            {'type': 'gripper', 'stroke': GRIP_GUN, 'desc': '12_grip_fuel_gun'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait04_1'},
            {'type': 'move_l', 'pos': [0.0, -40.0, 70.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '13_lift_gun_y-40_z+70'},

            {'type': 'move_j', 'pos': [-17.65, 31.58, 65.69, 85.49, 106.69, -9.58], 'desc': '14_to_port_wp1'},
            {'type': 'move_j', 'pos': [52.38, 10.46, 77.5, 100.99, -64.29, -9.6], 'desc': '14_to_port_wp2'},
            {'type': 'move_j', 'pos': [16.53, 37.18, 95.25, 101.33, -102.13, 43.64], 'desc': '14_to_port_wp3'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait05'},

            {'type': 'move_l', 'pos': fuel_insert_pose, 'desc': '15_fuel_approach_y+300'},
            {'type': 'move_l', 'pos': [0.0, 0.0, -20.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '15_insert_gun_z-35'},
            {'type': 'move_l', 'pos': [0.0, -150.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '16_insert_gun_y-150'},
            {'type': 'wait', 'seconds': 4.0, 'desc': 'wait05-1'},

            {'type': 'move_l', 'pos': [0.0, 150.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '16_retreat_y+150'},
            {'type': 'move_j', 'pos': [37.37, -21.49, 133.03, 78.64, -69.04, 32.08], 'desc': '17_remount_wp1'},
            {'type': 'move_j', 'pos': [-24.63, 34.65, 57.44, 85.63, 112.46, -4.84], 'desc': '17_remount_wp2'},
            {'type': 'move_j', 'pos': [-20.98, 55.77, 69.95, 75.92, 106.53, -44.1], 'desc': '17_remount_wp3'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait06'},

            {'type': 'gripper', 'stroke': GRIP_OPEN, 'desc': '18_release_fuel_gun'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait06_1'},
            {'type': 'move_l', 'pos': [0.0, -40.0, 70.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '19_retreat_z+100'},
            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '02_gripper_ready'},

            {'type': 'move_j', 'pos': [-11.28, 29.99, 58.06, 180.0, -91.94, -11.29], 'desc': '20_x_axis_align'},
            {'type': 'move_l', 'pos': [0.0, 0.0, -217.78, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '07-3_place_cap_down'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait07'},
            {'type': 'gripper', 'stroke': GRIP_CAP, 'desc': '22_grip_cap'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait07_1'},

            {'type': 'move_l', 'pos': [0.0, 0.0, 100.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '23_lift_z+150'},
            {'type': 'move_j', 'pos': [-4.34, 37.27, 91.2, 87.3, -86.61, -51.44], 'desc': '24_cap_align'},
            {'type': 'move_l', 'pos': approach_pose, 'desc': '24-1_cap_absolute_align'},
            {'type': 'move_l', 'pos': [0.0, -80.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '25_cap_approach_y-80'},
            {'type': 'move_j', 'pos': [0.0, 0.0, 0.0, 0.0, 0.0, 90.0], 'mode': 1, 'desc': '26_screw_j6+90'},
            {'type': 'wait', 'seconds': 1.0, 'desc': 'wait08'},

            {'type': 'gripper', 'stroke': GRIP_READY, 'desc': '27_release_cap'},
            {'type': 'wait', 'seconds': 2.0, 'desc': 'wait08_1'},
            {'type': 'move_l', 'pos': [0.0, 100.0, 0.0, 0.0, 0.0, 0.0], 'mode': 1, 'desc': '28_retreat_y+100'},

            {'type': 'move_j', 'pos': self.home_j, 'desc': '29_go_home'},
        ]


def main(args=None):
    rclpy.init(args=args)
    node = DoosanCommanderNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
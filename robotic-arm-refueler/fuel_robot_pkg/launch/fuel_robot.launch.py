from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_model = LaunchConfiguration('robot_model')

    doosan_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dsr_bringup2'),
                'launch',
                'dsr_bringup2_rviz.launch.py'
            ])
        ),
        launch_arguments={
            'mode': mode,
            'host': robot_ip,
            'model': robot_model,
        }.items()
    )


    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='real'),
        DeclareLaunchArgument('robot_ip', default_value='110.120.1.52'),
        DeclareLaunchArgument('robot_model', default_value='e0509'),

        doosan_bringup,

        Node(
            package='fuel_robot_pkg',
            executable='ui_gateway_node',
            name='ui_gateway_node',
            output='screen'
        ),
        Node(
            package='fuel_robot_pkg',
            executable='fuel_port_perception_node',
            name='fuel_port_perception_node',
            output='screen'
        ),
        Node(
            package='fuel_robot_pkg',
            executable='fueling_task_manager_node',
            name='fueling_task_manager_node',
            output='screen'
        ),
        Node(
            package='fuel_robot_pkg',
            executable='doosan_commander_node',
            name='doosan_commander_node',
            namespace='dsr01',
            output='screen'
        ),
    ])
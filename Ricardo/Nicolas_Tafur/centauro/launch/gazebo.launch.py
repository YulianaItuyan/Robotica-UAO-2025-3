from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.getenv('HOME'),
        'ros2_ws', 'src', 'mi_br', 'urdf', 'mi_br_2.urdf.xacro'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    parameters=[
                        {'robot_description': os.popen(f'xacro {urdf_file}').read()},
                        {'use_sim_time': True}
                    ],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description', '-entity', 'simple_arm'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        )
    ])


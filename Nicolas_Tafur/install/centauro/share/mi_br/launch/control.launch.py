from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    pkg_mi_br = get_package_share_directory('mi_br')

    xacro_file = os.path.join(pkg_mi_br, 'urdf', 'simple_arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()



    delayed_controller_spawners = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        
        delayed_controller_spawners
    ])


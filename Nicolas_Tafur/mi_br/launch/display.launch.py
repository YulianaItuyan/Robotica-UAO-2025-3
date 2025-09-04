from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'mi_br',
        'urdf',
        'UDRF_LINKS.urdf'
    )

    return LaunchDescription([
        # Publica el estado del robot (pose de cada link)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': os.popen(f'xacro {urdf_path}').read()
            }]
        ),

        # Visualización en RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        Node(
            package='mi_br',
            executable='fk_marker',
            name='fk_marker'
        ),
    ])


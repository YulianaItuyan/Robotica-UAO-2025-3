from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Ruta al URDF exportado desde SolidWorks
    urdf_path = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'mi_br',
        'urdf',
        'URDF_LINKS.urdf'   # <-- aquí el .urdf plano
    )

    # Leemos el contenido del URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Publica el estado del robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc
            }]
        ),

        # Visualización en RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # Nodo de tus marcadores FK
        Node(
            package='mi_br',
            executable='fk_marker',
            name='fk_marker'
        ),
    ])


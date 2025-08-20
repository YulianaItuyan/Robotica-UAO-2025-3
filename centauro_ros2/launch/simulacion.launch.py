from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mi_br = get_package_share_directory('mi_br')

    # Procesar Xacro → URDF string
    xacro_file = os.path.join(pkg_mi_br, 'urdf', 'simple_arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Mundo vacío
    world = PathJoinSubstitution([pkg_mi_br, 'worlds', 'empty.world'])

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=world, description='Archivo del mundo Gazebo'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Publicar robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}],
        output='screen'
    )

    # Spawnear desde el tópico
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mi_br', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])


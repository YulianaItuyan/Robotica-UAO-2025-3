#!/usr/bin/env python3
# centauro.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Rutas de recursos
    pkg_share = get_package_share_directory('robot')
    urdf = os.path.join(pkg_share, 'urdf', 'URDF_LINKS.urdf')
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'robot.rviz'])

    # LaunchConfigurations (argumentos que se pueden pasar por terminal)
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    commander = LaunchConfiguration('commander')
    serial = LaunchConfiguration('serial')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    move_duration_s = LaunchConfiguration('move_duration_s')
    #smooth_motion = LaunchConfiguration('smooth_motion')

    # Cargar URDF como string
    with open(urdf, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    robot_description = ParameterValue(urdf_content, value_type=str)

    nodes = []

    # 1️⃣ Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    ))

    # 2️⃣ RViz (opcional)
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz)
    ))


    # 5️⃣ GUI (opcional)
    nodes.append(Node(
        package='robot',
        executable='pinterfaz',
        name='pinterfaz',
        output='screen',
        condition=IfCondition(gui)
    ))

    # 6️⃣ Joint Commander con interpolación suave (solo si commander y smooth_motion son true)
    # Joint Commander con interpolación suave
    nodes.append(Node(
        package='robot',
        executable='joint_commander_deg',
        name='joint_commander_deg',
        output='screen',
        parameters=[{
            'move_duration_s': move_duration_s
        }],
        condition=IfCondition(commander)
    ))


    # 7️⃣ Nodo serial hacia Arduino (opcional)
    nodes.append(Node(
        package='robot',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud
            #'use_smooth_motion': smooth_motion
        }],
        condition=IfCondition(serial)
    ))

    # 📦 Retornar LaunchDescription con argumentos y nodos
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('commander', default_value='true'),
        DeclareLaunchArgument('serial', default_value='true'),
        DeclareLaunchArgument('move_duration_s', default_value='1.0'),
        #DeclareLaunchArgument('smooth_motion', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        *nodes
    ])

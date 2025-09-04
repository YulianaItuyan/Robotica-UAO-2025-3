# mi_br/launch/centauro.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    # Ruta a la carpeta share del paquete
    pkg_share = FindPackageShare('centauro').find('mi_br')

    # Ruta completa al URDF estático
    urdf_path = os.path.join(pkg_share, 'urdf', 'URDF_LINKS.urdf')

    # Cargar el contenido del URDF como string
    with open(urdf_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'mi_br.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    commander = LaunchConfiguration('commander')
    serial = LaunchConfiguration('serial')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')


    robot_description = ParameterValue(urdf_content, value_type=str)
    
    nodes = []

    # Publica TF y robot_description
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

    # RViz
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz)
    ))

    # Visualizador de FK 
    #nodes.append(Node(
    #    package='mi_br',
    #    executable='fk_marker',
    #    name='fk_marker',

    #))

    # GUI
    nodes.append(Node(
        package='mi_br',
        executable='pinterfaz',
        name='pinterfaz',
        output='screen',
        condition=IfCondition(gui)
    ))

    # Commander que traduce /cmd_deg -> /joint_states
    nodes.append(Node(
        package='mi_br',
        executable='joint_commander_deg',
        name='joint_commander_deg',
        output='screen',
        condition=IfCondition(commander)
    ))

    # Puente serial hacia Arduino
    nodes.append(Node(
        package='mi_br',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{
            'port': port,
            'baud': baud
        }],
        condition=IfCondition(serial)
    ))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('commander', default_value='true'),
        DeclareLaunchArgument('serial', default_value='true'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        *nodes
    ])


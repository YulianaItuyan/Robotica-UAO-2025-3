# robot/launch/centauro.launch.py
from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare('robot').find('robot')
    urdf = os.path.join(pkg_share, 'urdf', 'URDF_LINKS.urdf')
    #urdf = PathJoinSubstitution([pkg_share, 'urdf', 'URDF_LINKS.urdf'])
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'robot.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    commander = LaunchConfiguration('commander')
    serial = LaunchConfiguration('serial')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')

    # Cargar el contenido del URDF como string
    with open(urdf, 'r') as urdf_file:
        urdf_content = urdf_file.read()
    
    robot_description = ParameterValue(urdf_content, value_type=str)
    

    nodes = []

    # Publica TF y robot_description-------------------
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
    nodes.append(Node(
        package='robot',
        executable='fk_marker',
        name='fk_marker'

    ))

    # Nodo ros2_control ------ creo que innecesario
    nodes.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description
        },
        PathJoinSubstitution([pkg_share, 'config', 'joint_trajectory_controller.yaml'])],
        output='screen'
    ))

    # # Spawner del JointTrajectoryController
    # nodes.append(Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_trajectory_controller'],
    #     output='screen'
    # ))

    # GUI
    nodes.append(Node(
        package='robot',
        executable='pinterfaz',
        name='pinterfaz',
        output='screen',
        condition=IfCondition(gui)
    ))

    # nodes.append(Node(
    # package='robot',
    # executable='joint_states_to_trajectory',
    # name='joint_states_to_trajectory',
    # output='screen'
    # ))
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    ))


    # # Commander que traduce /cmd_deg -> /joint_states
    nodes.append(Node(
        package='robot',
        executable='joint_commander_deg',
        name='joint_commander_deg',
         output='screen',
        condition=IfCondition(commander)
    ))

    # Puente serial hacia Arduino
    nodes.append(Node(
        package='robot',
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

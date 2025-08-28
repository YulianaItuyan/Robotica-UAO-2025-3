# robot/launch/centauro.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare('robot')
    urdf = PathJoinSubstitution([pkg_share, 'urdf', 'URDF_LINKS.urdf'])
    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'robot.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    commander = LaunchConfiguration('commander')
    serial = LaunchConfiguration('serial')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    smooth_motion = LaunchConfiguration('smooth_motion')  # Nueva opción

    
    robot_description = ParameterValue(
    PathJoinSubstitution([
        FindPackageShare('robot'),
        'urdf',
        'URDF_LINKS.urdf'
    ]),
    value_type=str
    )



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

    # Nodo ros2_control
    nodes.append(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description
        },
        PathJoinSubstitution([pkg_share, 'config', 'joint_trajectory_controller.yaml'])],
        output='screen'
    ))

    # Spawner del JointTrajectoryController ---------- crep qie innecesario
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    ))

    # GUI
    nodes.append(Node(
        package='robot',
        executable='pinterfaz',
        name='pinterfaz',
        output='screen',
        parameters=[{
            'use_smooth_motion': smooth_motion
        }],
        condition=IfCondition(gui)
    ))

    # Commander que traduce /cmd_deg -> /joint_states Y actúa como action server
    nodes.append(Node(
        package='robot',
        executable='joint_commander_deg',
        name='joint_commander_deg',
        output='screen',
        condition=IfCondition(commander)
    ))

    # NUEVO: Nodo para movimientos suaves (solo si smooth_motion está habilitado)
    nodes.append(Node(
        package='robot',
        executable='joint_trajectory_smoother',
        name='joint_trajectory_smoother',
        output='screen',
        parameters=[{
            'trajectory_duration': 2.0,  # Duración de cada movimiento
            'interpolation_points': 15   # Puntos de interpolación
        }],
        condition=IfCondition(smooth_motion)
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
        DeclareLaunchArgument('smooth_motion', default_value='true'),  # NUEVO: habilitar movimientos suaves
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        *nodes
    ])
#!/usr/bin/env python3

# robot/robot/joint_commander_deg.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from control_msgs.action import FollowJointTrajectory
import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class JointCommanderDeg(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')
        
        # Usar callback group para manejar múltiples callbacks
        self.callback_group = ReentrantCallbackGroup()
     
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.q_deg = np.array([90, 90, 90], dtype=float)
        self.target_positions = np.array([0.0, 0.0, 0.0], dtype=float)  # En radianes
        
        # Publisher para joint_states
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscription para comandos directos (mantener compatibilidad)
        self.sub = self.create_subscription(
            Float32MultiArray, 
            'cmd_deg',  # Mantener el mismo nombre que usa la interfaz
            self.on_cmd, 
            10,
            callback_group=self.callback_group
        )
        
        # Action server para FollowJointTrajectory
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_trajectory_callback,
            callback_group=self.callback_group
        )
        
        # Timer para publicar joint_states
        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz
        
        # Variables para la ejecución de trayectorias
        self.trajectory_executing = False
        self.trajectory_start_time = None
        self.current_trajectory = None
        
        self.get_logger().info('JointCommanderDeg iniciado con soporte para trayectorias suaves')
        self.get_logger().info('- Comandos directos en /cmd_deg')
        self.get_logger().info('- Servidor de trayectorias en /joint_trajectory_controller/follow_joint_trajectory')

    def on_cmd(self, msg: Float32MultiArray):
        """Callback para comandos directos (compatibilidad hacia atrás)"""
        if len(msg.data) != 3:
            self.get_logger().warn(f'Se esperaban 3 grados, llegaron {len(msg.data)}')
            return
        
        self.q_deg = np.array(msg.data, dtype=float)
        
        # Convertir a radianes con offset
        deg_shifted = self.q_deg - 90.0
        self.target_positions = np.deg2rad(deg_shifted)
        
        # Detener cualquier trayectoria en ejecución
        self.trajectory_executing = False

    def execute_trajectory_callback(self, goal_handle):
        """Callback para ejecutar una trayectoria"""
        self.get_logger().info('Ejecutando nueva trayectoria...')
        
        request = goal_handle.request
        trajectory = request.trajectory
        
        # Validar que los nombres de las articulaciones coincidan
        if trajectory.joint_names != self.joint_names:
            self.get_logger().error(f'Nombres de articulaciones no coinciden: {trajectory.joint_names} vs {self.joint_names}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result
        
        # Configurar la ejecución de la trayectoria
        self.current_trajectory = trajectory
        self.trajectory_start_time = self.get_clock().now()
        self.trajectory_executing = True
        
        goal_handle.succeed()
        
        # Esperar a que termine la trayectoria
        total_duration = 0.0
        if trajectory.points:
            last_point = trajectory.points[-1]
            total_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9
        
        self.get_logger().info(f'Trayectoria iniciada. Duración estimada: {total_duration:.2f}s')
        
        # Resultado exitoso
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def interpolate_trajectory_position(self, current_time):
        """Interpola la posición actual basada en la trayectoria y el tiempo"""
        if not self.trajectory_executing or not self.current_trajectory:
            return self.target_positions
        
        # Calcular tiempo transcurrido
        elapsed = current_time - self.trajectory_start_time
        elapsed_sec = elapsed.nanoseconds * 1e-9
        
        points = self.current_trajectory.points
        
        # Si no hay puntos, mantener posición actual
        if not points:
            self.trajectory_executing = False
            return self.target_positions
        
        # Buscar los dos puntos entre los cuales interpolar
        prev_point = None
        next_point = None
        
        for point in points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            if elapsed_sec <= point_time:
                next_point = point
                break
            prev_point = point
        
        # Si hemos pasado todos los puntos, usar el último
        if next_point is None:
            if points:
                self.target_positions = np.array(points[-1].positions)
            self.trajectory_executing = False
            self.get_logger().info('Trayectoria completada')
            return self.target_positions
        
        # Si no hay punto previo, usar el primero
        if prev_point is None:
            self.target_positions = np.array(next_point.positions)
            return self.target_positions
        
        # Interpolación lineal entre prev_point y next_point
        prev_time = prev_point.time_from_start.sec + prev_point.time_from_start.nanosec * 1e-9
        next_time = next_point.time_from_start.sec + next_point.time_from_start.nanosec * 1e-9
        
        # Factor de interpolación
        if next_time > prev_time:
            t = (elapsed_sec - prev_time) / (next_time - prev_time)
            t = max(0.0, min(1.0, t))  # Clamp entre 0 y 1
        else:
            t = 1.0
        
        # Interpolación lineal
        prev_pos = np.array(prev_point.positions)
        next_pos = np.array(next_point.positions)
        interpolated_pos = prev_pos + t * (next_pos - prev_pos)
        
        self.target_positions = interpolated_pos
        return interpolated_pos

    def on_timer(self):
        """Timer callback para publicar joint_states"""
        current_time = self.get_clock().now()
        
        # Si hay una trayectoria ejecutándose, interpolar la posición
        current_positions = self.interpolate_trajectory_position(current_time)
        
        # Crear y publicar mensaje JointState
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = self.joint_names
        msg.position = current_positions.tolist()
        msg.velocity = [0.0] * len(self.joint_names)  # Velocidades por defecto
        msg.effort = [0.0] * len(self.joint_names)    # Esfuerzos por defecto
        
        self.pub.publish(msg)

        
def main():
    rclpy.init()
    node = JointCommanderDeg()
    
    # Usar executor multi-hilo para manejar el action server
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
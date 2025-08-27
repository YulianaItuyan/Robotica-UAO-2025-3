#!/usr/bin/env python3

# robot/robot/joint_trajectory_smoother.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
import numpy as np


class JointTrajectorySmoother(Node):
    def __init__(self):
        super().__init__('joint_trajectory_smoother')
        
        # Nombres de las articulaciones
        self.joint_names = ['joint1', 'joint2', 'joint3']
        
        # Posición actual en grados (inicializada en posición neutra)
        self.current_deg = np.array([90.0, 90.0, 90.0])
        
        # Cliente de acción para JointTrajectory
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Suscripción al tópico de comandos de grados
        self.cmd_subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_deg_smooth',  # Usar un tópico diferente para evitar conflictos
            self.on_cmd_deg,
            10
        )
        
        # Parámetros de interpolación
        self.declare_parameter('trajectory_duration', 2.0)  # Duración del movimiento en segundos
        self.declare_parameter('interpolation_points', 10)   # Número de puntos intermedios
        
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.interpolation_points = self.get_parameter('interpolation_points').value
        
        self.get_logger().info(f'JointTrajectorySmoother iniciado. Esperando comandos en /cmd_deg_smooth')
        self.get_logger().info(f'Duración de trayectoria: {self.trajectory_duration}s, Puntos: {self.interpolation_points}')
        
        # Esperar a que el servidor de acción esté disponible
        self.wait_for_trajectory_server()
    
    def wait_for_trajectory_server(self):
        """Espera a que el servidor de trayectorias esté disponible"""
        self.get_logger().info('Esperando servidor de trayectorias...')
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Servidor de trayectorias no disponible!')
        else:
            self.get_logger().info('Servidor de trayectorias conectado.')
    
    def on_cmd_deg(self, msg: Float32MultiArray):
        """Callback cuando llega un nuevo comando de grados"""
        if len(msg.data) != 3:
            self.get_logger().warn(f'Se esperaban 3 valores, llegaron {len(msg.data)}')
            return
        
        # Nuevos ángulos objetivo
        target_deg = np.array(msg.data, dtype=float)
        
        self.get_logger().info(f'Nuevo objetivo: {target_deg} (actual: {self.current_deg})')
        
        # Generar y enviar trayectoria suave
        self.send_smooth_trajectory(self.current_deg, target_deg)
        
        # Actualizar posición actual
        self.current_deg = target_deg.copy()
    
    def send_smooth_trajectory(self, start_deg, end_deg):
        """Genera y envía una trayectoria suave entre dos posiciones"""
        
        # Crear el mensaje de objetivo de trayectoria
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Generar puntos intermedios
        for i in range(self.interpolation_points + 1):
            # Factor de interpolación (0.0 a 1.0)
            t = i / self.interpolation_points
            
            # Interpolación lineal entre posiciones
            interpolated_deg = start_deg + t * (end_deg - start_deg)
            
            # Convertir a radianes y aplicar offset (-90°)
            deg_shifted = interpolated_deg - 90.0
            interpolated_rad = np.deg2rad(deg_shifted)
            
            # Crear punto de trayectoria
            point = JointTrajectoryPoint()
            point.positions = interpolated_rad.tolist()
            
            # Calcular tiempo para este punto
            point_time = (t * self.trajectory_duration)
            point.time_from_start = Duration(
                sec=int(point_time),
                nanosec=int((point_time - int(point_time)) * 1e9)
            )
            
            # Velocidades y aceleraciones (opcionales, se pueden calcular automáticamente)
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            
            goal_msg.trajectory.points.append(point)
        
        # Enviar el objetivo
        self.get_logger().info(f'Enviando trayectoria con {len(goal_msg.trajectory.points)} puntos')
        
        # Enviar de manera asíncrona
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback cuando el servidor responde al objetivo"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trayectoria rechazada por el servidor')
            return
        
        self.get_logger().info('Trayectoria aceptada, ejecutándose...')
        
        # Obtener el resultado final
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback cuando la trayectoria se completa"""
        result = future.result().result
        self.get_logger().info(f'Trayectoria completada. Código: {result.error_code}')


def main():
    rclpy.init()
    node = JointTrajectorySmoother()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# joint_control_smoother.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
import numpy as np


class joint_control_smoother(Node):
    def __init__(self):
        super().__init__('joint_control_smoother')
        
        # TODOS los 6 joints del robot
        self.joint_names = [
            'joint1', 'joint2', 'joint3',    # Brazo A (Izquierdo)
            'joint5', 'joint6', 'joint7'     # Brazo B (Derecho)
        ]
        
        # Estado completo de todos los joints (6 joints)
        # Inicializar con valores neutros específicos
        self.current_deg_full = np.array([
            180.0 if name in ('joint3', 'joint7') else 90.0
            for name in self.joint_names
        ], dtype=float)
        
        # Brazo seleccionado actualmente ('A' = Izquierdo, 'B' = Derecho)
        self.selected_arm = 'A'
        
        # Cliente de acción para JointTrajectory
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Suscripción al tópico de comandos de grados suaves
        self.cmd_subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_deg_smooth',
            self.on_cmd_deg,
            10
        )
        
        # Suscriptor para selección de brazo
        self.sub_arm_select = self.create_subscription(
            String,
            '/cmd_arm_select',
            self.on_arm_select,
            10
        )
        
        # Parámetros de interpolación
        self.declare_parameter('trajectory_duration', 2.0)  # Duración del movimiento en segundos
        self.declare_parameter('interpolation_points', 10)   # Número de puntos intermedios
        
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.interpolation_points = self.get_parameter('interpolation_points').value
        
        self.get_logger().info(f'JointTrajectorySmootherDualArm iniciado con {len(self.joint_names)} joints')
        self.get_logger().info(f'Joints: {self.joint_names}')
        self.get_logger().info(f'Brazo seleccionado: {self.selected_arm}')
        self.get_logger().info(f'Duración de trayectoria: {self.trajectory_duration}s, Puntos: {self.interpolation_points}')
        self.get_logger().info('Esperando comandos en /cmd_deg_smooth y /cmd_arm_select')
        
        # Esperar a que el servidor de acción esté disponible
        self.wait_for_trajectory_server()
    
    def wait_for_trajectory_server(self):
        """Espera a que el servidor de trayectorias esté disponible"""
        self.get_logger().info('Esperando servidor de trayectorias...')
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('⚠️ Servidor de trayectorias no disponible!')
        else:
            self.get_logger().info('✅ Servidor de trayectorias conectado.')
    
    def on_arm_select(self, msg: String):
        """Callback para cambio de brazo seleccionado"""
        if msg.data in ['A', 'B']:
            self.selected_arm = msg.data
            arm_name = "Izquierdo" if msg.data == 'A' else "Derecho"
            self.get_logger().info(f'🔄 Brazo seleccionado: {msg.data} ({arm_name})')
        else:
            self.get_logger().warn(f'⚠️ Selección de brazo inválida: {msg.data}')
    
    def on_cmd_deg(self, msg: Float32MultiArray):
        """Callback cuando llega un nuevo comando de grados para el brazo seleccionado"""
        if len(msg.data) != 3:  # Siempre esperamos 3 joints por brazo
            self.get_logger().warn(f'⚠️ Se esperaban 3 grados, llegaron {len(msg.data)}')
            return
        
        received_angles = np.array(msg.data, dtype=float)
        
        # Obtener posición actual del brazo seleccionado
        if self.selected_arm == 'A':
            # Brazo A: joints 0, 1, 2 (joint1, joint2, joint3)
            current_arm_deg = self.current_deg_full[0:3].copy()
            arm_name = "Izquierdo"
        elif self.selected_arm == 'B':
            # Brazo B: joints 3, 4, 5 (joint5, joint6, joint7)
            current_arm_deg = self.current_deg_full[3:6].copy()
            arm_name = "Derecho"
        
        self.get_logger().info(f'🎯 Nuevo objetivo para brazo {self.selected_arm} ({arm_name}):')
        self.get_logger().info(f'   Actual: {current_arm_deg.tolist()}°')
        self.get_logger().info(f'   Objetivo: {received_angles.tolist()}°')
        
        # Generar y enviar trayectoria suave
        self.send_smooth_trajectory(current_arm_deg, received_angles)
        
        # Actualizar posición actual del brazo seleccionado
        if self.selected_arm == 'A':
            self.current_deg_full[0:3] = received_angles
        elif self.selected_arm == 'B':
            self.current_deg_full[3:6] = received_angles
        
        self.get_logger().info(f'📍 Estado completo actualizado: {self.current_deg_full.tolist()}°')
    
    def send_smooth_trajectory(self, start_deg, end_deg):
        """Genera y envía una trayectoria suave entre dos posiciones para TODOS los joints"""
        
        # Crear el mensaje de objetivo de trayectoria
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names  # Todos los 6 joints
        
        # Estado completo objetivo (copia del estado actual)
        target_full_deg = self.current_deg_full.copy()
        
        # Actualizar solo el brazo seleccionado en el objetivo
        if self.selected_arm == 'A':
            target_full_deg[0:3] = end_deg
        elif self.selected_arm == 'B':
            target_full_deg[3:6] = end_deg
        
        # Generar puntos intermedios
        for i in range(self.interpolation_points + 1):
            # Factor de interpolación (0.0 a 1.0)
            t = i / self.interpolation_points
            
            # Interpolación para TODOS los 6 joints
            interpolated_full_deg = self.current_deg_full + t * (target_full_deg - self.current_deg_full)
            
            # Convertir a radianes y aplicar offset (-90°)
            deg_shifted = interpolated_full_deg - 90.0
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
            
            # Velocidades y aceleraciones (se pueden calcular automáticamente)
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            
            goal_msg.trajectory.points.append(point)
        
        # Enviar el objetivo
        self.get_logger().info(f'🚀 Enviando trayectoria suave con {len(goal_msg.trajectory.points)} puntos para brazo {self.selected_arm}')
        
        # Enviar de manera asíncrona
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback cuando el servidor responde al objetivo"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Trayectoria rechazada por el servidor')
            return
        
        self.get_logger().info(f'✅ Trayectoria aceptada para brazo {self.selected_arm}, ejecutándose...')
        
        # Obtener el resultado final
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback cuando la trayectoria se completa"""
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info(f'🎉 Trayectoria completada exitosamente para brazo {self.selected_arm}')
        else:
            self.get_logger().warn(f'⚠️ Trayectoria completada con código de error: {result.error_code}')


def main():
    rclpy.init()
    node = joint_control_smoother()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
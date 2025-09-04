#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class JointCommanderDeg(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')
     
        # Ahora 6 joints AQUÍ DEBEN IR LOS NOMBRES IGUALES A LOS DEL URDF
        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint5', 'joint6', 'joint7'
        ]
        
        self.HOME_POSITIONS1 = [90.0, 180.0, 90.0, 90.0, 180.0, 90.0]  # 
        self.q_deg_current = self.HOME_POSITIONS1.copy() # ESTÁ EN EL APARTADO DEL BOTÓN DE STOP
        
        
        self.HOME_POSITIONS2 = self.HOME_POSITIONS1.copy()
        self.q_deg_target = self.HOME_POSITIONS2.copy() # ESTÁ EN EL APARTADO DEL BOTÓN DE STOP
        
        
        
        # Estado actual y objetivo
        self.q_deg_current = np.array([90.0] * len(self.joint_names), dtype=float)  # Posición actual
        self.q_deg_target = np.array([90.0] * len(self.joint_names), dtype=float)   # Posición objetivo
        
        # Configuración de smooth trajectories
        self.max_velocity_deg_s = 45.0  # Velocidad máxima en grados/segundo (ajustable)
        self.dt = 0.02  # Período del timer (50 Hz)
        self.trajectory_active = False
        
        # Publishers y subscribers
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'cmd_deg', self.on_cmd, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)
        
        self.get_logger().info(
            f'Joint Commander iniciado con smooth trajectories. '
            f'Velocidad máxima: {self.max_velocity_deg_s}°/s'
        )
    
    def on_cmd(self, msg: Float32MultiArray):
        """Recibe nuevo comando y activa la trayectoria suave."""
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(
                f'Se esperaban {len(self.joint_names)} grados, llegaron {len(msg.data)}'
            )
            return
        
        # Actualizar objetivo
        self.q_deg_target = np.array(msg.data, dtype=float)
        
        # Verificar si hay movimiento necesario
        diff = np.abs(self.q_deg_target - self.q_deg_current)
        if np.max(diff) > 0.5:  # Solo activar si hay diferencia significativa (>0.5°)
            self.trajectory_active = True
            self.get_logger().info(f'Nueva trayectoria iniciada. Máx. diff: {np.max(diff):.1f}°')
    
    def update_smooth_trajectory(self):
        """Actualiza la posición actual hacia el objetivo de forma suave."""
        if not self.trajectory_active:
            return
        
        # Calcular diferencia
        diff = self.q_deg_target - self.q_deg_current
        
        # Calcular máximo movimiento permitido en este ciclo
        max_step = self.max_velocity_deg_s * self.dt
        
        # Aplicar limitación de velocidad por joint
        step = np.clip(diff, -max_step, max_step)
        
        # Actualizar posición actual
        self.q_deg_current += step
        
        # Verificar si hemos llegado al objetivo (tolerancia de 0.1°)
        remaining_error = np.abs(self.q_deg_target - self.q_deg_current)
        if np.max(remaining_error) < 0.1:
            self.q_deg_current = self.q_deg_target.copy()  # Snap al objetivo exacto
            self.trajectory_active = False
            self.get_logger().info('Trayectoria completada')
    
    def on_timer(self):
        """Timer principal - actualiza trayectoria y publica joint states."""
        # Actualizar la trayectoria suave
        self.update_smooth_trajectory()
        
        # Crear mensaje JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Ajuste de grados a radianes centrados en 0°
        # Offset en grados para cada joint (lo que se resta antes de pasar a radianes)
        # joint1, joint2, joint3, joint5, joint6, joint7
        offsets = np.array([90.0, 90.0, 0.0,   90.0, 90.0, 0.0])
        deg_shifted = self.q_deg_current - offsets  # Usar posición actual, no objetivo
        q_rad = np.deg2rad(deg_shifted)
        msg.position = q_rad.tolist()
        
        # Opcionalmente agregar velocidades (útil para RViz)
        if self.trajectory_active:
            # Calcular velocidad actual aproximada
            diff = self.q_deg_target - self.q_deg_current
            max_step = self.max_velocity_deg_s * self.dt
            step = np.clip(diff, -max_step, max_step)
            velocity_deg_s = step / self.dt
            msg.velocity = np.deg2rad(velocity_deg_s).tolist()
        else:
            msg.velocity = [0.0] * len(self.joint_names)
        
        self.pub.publish(msg)
    
    def set_max_velocity(self, vel_deg_s):
        """Permite cambiar la velocidad máxima dinámicamente."""
        self.max_velocity_deg_s = max(1.0, vel_deg_s)  # Mínimo 1°/s
        self.get_logger().info(f'Velocidad máxima actualizada: {self.max_velocity_deg_s}°/s')

def main():
    rclpy.init()
    node = JointCommanderDeg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

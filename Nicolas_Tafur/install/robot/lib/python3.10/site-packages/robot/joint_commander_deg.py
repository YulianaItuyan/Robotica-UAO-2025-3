#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import time

class JointCommanderDeg(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')
     
        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint5', 'joint6', 'joint7'
        ]
        
        # Estado actual y objetivo
        self.q_deg_current = np.array([90.0, 90.0, 0.0,   90.0, 90.0, 0.0], dtype=float)
        self.q_deg_target  = np.array([90.0, 90.0, 0.0,   90.0, 90.0, 0.0], dtype=float)

        #parametros
        self.declare_parameter('move_duration_s', 2.0)  # Duración del movimiento
        # Leer el parámetro (si se quiere hacer dinámico en el futuro)
        self.move_duration_s = float(self.get_parameter('move_duration_s').value)

        
        # Duración deseada para completar un movimiento (segundos)
        self.move_duration_s = max(0.1, self.move_duration_s)  # mínimo 0.1s 

        self.trajectory_active = False
        self.selected_arm = 'A'

        # Variables para interpolación por tiempo
        self.start_deg = self.q_deg_current.copy()
        self.end_deg = self.q_deg_target.copy()
        self.start_time = None

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub_cmd = self.create_subscription(Float32MultiArray, '/cmd_deg', self.on_cmd, 10)
        self.sub_arm_select = self.create_subscription(String, '/cmd_arm_select', self.on_arm_select, 10)
        self.timer = self.create_timer(0.02, self.on_timer)

        self.get_logger().info(f'Joint Commander iniciado con control por tiempo ({self.move_duration_s}s por movimiento)')
        self.get_logger().info(f'Brazo seleccionado: {self.selected_arm}')

    def on_arm_select(self, msg: String):
        if msg.data in ['A', 'B']:
            self.selected_arm = msg.data
            arm_name = "Izquierdo" if msg.data == 'A' else "Derecho"
            self.get_logger().info(f'🔄 Brazo seleccionado: {msg.data} ({arm_name})')
        else:
            self.get_logger().warn(f'⚠️ Selección de brazo inválida: {msg.data}')

    def on_cmd(self, msg: Float32MultiArray):
        n = len(msg.data)

        if n == 3:
            received_angles = np.array(msg.data, dtype=float)
        elif n == 6:
            if self.selected_arm == 'A':
                received_angles = np.array(msg.data[0:3], dtype=float)
            else:
                received_angles = np.array(msg.data[3:6], dtype=float)
        else:
            self.get_logger().warn(f'⚠️ Longitud inesperada en /cmd_deg: {n}. Se esperaban 3 o 6 valores.')
            return

        if self.selected_arm == 'A':
            self.q_deg_target[0:3] = received_angles
            arm_name = "Izquierdo"
        elif self.selected_arm == 'B':
            self.q_deg_target[3:6] = received_angles
            arm_name = "Derecho"

        diff = np.abs(self.q_deg_target - self.q_deg_current)
        if np.max(diff) > 0.5:
            self.start_deg = self.q_deg_current.copy()
            self.end_deg = self.q_deg_target.copy()
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.trajectory_active = True
            self.get_logger().info(f'✅ Comando para brazo {self.selected_arm} ({arm_name}): {received_angles.tolist()}°')
            self.get_logger().info(f'Movimiento programado para {self.move_duration_s} segundos')

    def update_smooth_trajectory(self):
        if not self.trajectory_active or self.start_time is None:
            return
        
        now_s = self.get_clock().now().nanoseconds / 1e9
        t_elapsed = now_s - self.start_time
        alpha = min(t_elapsed / self.move_duration_s, 1.0)

        self.q_deg_current = self.start_deg + alpha * (self.end_deg - self.start_deg)

        if alpha >= 1.0:
            self.trajectory_active = False
            self.get_logger().info('Trayectoria completada')

    def on_timer(self):
        self.update_smooth_trajectory()
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        offsets = np.array([90, 90, 0.0,   90, 90, 0.0])
        deg_shifted = self.q_deg_current + offsets
        q_rad = np.deg2rad(deg_shifted)

        msg.position = q_rad.tolist()
        
        msg.velocity = [0.0] * len(self.joint_names)
        self.pub.publish(msg)

    def set_move_duration(self, duration_s):
        self.move_duration_s = max(0.1, duration_s)
        self.get_logger().info(f'Duración de movimiento actualizada: {self.move_duration_s}s')

def main():
    rclpy.init()
    node = JointCommanderDeg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

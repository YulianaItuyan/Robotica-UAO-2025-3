#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
import numpy as np

class JointCommanderDeg(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')

        self.joint_names = ['joint1','joint2','joint3','joint5','joint6','joint7']

        # Estado actual
        self.q_deg_current = np.array([90.0, 90.0, 0.0, 90.0, 90.0, 0.0], dtype=float)

        self.selected_arm = 'A'

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub_cmd = self.create_subscription(Float32MultiArray, '/sim_deg', self.on_cmd, 10)
        self.sub_arm_select = self.create_subscription(String, '/cmd_arm_select', self.on_arm_select, 10)
        self.timer = self.create_timer(0.02, self.on_timer)

        self.get_logger().info('Joint Commander iniciado SIN interpolación por tiempo.')
        self.get_logger().info(f'Brazo seleccionado: {self.selected_arm}')

    def on_arm_select(self, msg: String):
        if msg.data in ['A', 'B']:
            self.selected_arm = msg.data
            arm_name = "Izquierdo" if msg.data == 'B' else "Derecho"
            self.get_logger().info(f'🔄 Brazo seleccionado: {msg.data} ({arm_name})')
        else:
            self.get_logger().warn(f'⚠️ Selección de brazo inválida: {msg.data}')

    def on_cmd(self, msg: Float32MultiArray):
        n = len(msg.data)

        if n == 3:
            received_angles = np.array(msg.data, dtype=float)
        elif n == 6:
            received_angles = np.array(msg.data[0:3] if self.selected_arm == 'A' else msg.data[3:6], dtype=float)
        else:
            self.get_logger().warn(f'⚠️ Longitud inesperada en /sim_deg: {n}. Se esperaban 3 o 6 valores.')
            return

        # Aplicar DIRECTO al estado actual (sin interpolar)
        if self.selected_arm == 'A':
            self.q_deg_current[0:3] = received_angles
            arm_name = "Derecho"
        else:
            self.q_deg_current[3:6] = received_angles
            arm_name = "Izquierdo"

        self.get_logger().info(f'✅ Comando aplicado directo en brazo {self.selected_arm} ({arm_name}): {received_angles.tolist()}°')

    def on_timer(self):
        # Publicar el estado actual tal cual
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Desfase a URDF y radianes
        offsets = np.array([-90, -90, 0.0, -90, -90, 0.0])
        q_rad = np.deg2rad(self.q_deg_current + offsets)

        msg.position = q_rad.tolist()
        msg.velocity = [0.0] * len(self.joint_names)
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointCommanderDeg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

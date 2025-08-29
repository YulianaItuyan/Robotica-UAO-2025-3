#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np

class JointCommanderDeg(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')
     
        # Ahora 6 joints (ajusta nombres exactamente como en URDF)
        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint5', 'joint6', 'joint7'
        ]
        
        self.q_deg = np.array([90.0] * len(self.joint_names), dtype=float)

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'cmd_deg', self.on_cmd, 10)
        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz

        self.get_logger().info(
            f'Esperando grados en /cmd_deg (Float32MultiArray con {len(self.joint_names)} valores).'
        )

    def on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(
                f'Se esperaban {len(self.joint_names)} grados, llegaron {len(msg.data)}'
            )
            return
        self.q_deg = np.array(msg.data, dtype=float)

    def on_timer(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Ajuste de grados a radianes centrados en 0°
        # Offset en grados para cada joint (lo que se resta antes de pasar a radianes)
        # joint1, joint2, joint3, joint5, joint6, joint7
        offsets = np.array([90.0, 90.0, 0.0,   90.0, 90.0, 0.0])
        deg_shifted = self.q_deg - offsets

        q_rad = np.deg2rad(deg_shifted)
        msg.position = q_rad.tolist()

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = JointCommanderDeg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


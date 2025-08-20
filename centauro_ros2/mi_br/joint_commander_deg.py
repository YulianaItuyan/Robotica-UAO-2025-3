# mi_br/mi_br/joint_commander_deg.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np
from builtin_interfaces.msg import Time as TimeMsg

class JointCommanderDeg(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')
        # Ajusta estos nombres al URDF exacto
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.q_deg = np.array([90, 90, 90], dtype=float)

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'cmd_deg', self.on_cmd, 10)
        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz

        self.get_logger().info('Esperando grados en /cmd_deg (Float32MultiArray con 3 valores).')

    def on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) != 3:
            self.get_logger().warn(f'Se esperaban 3 grados, llegaron {len(msg.data)}')
            return
        self.q_deg = np.array(msg.data, dtype=float)

    def on_timer(self):
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.name = self.joint_names
        # Convierte grados a radianes para JointState
        #q_rad = np.deg2rad(self.q_deg).tolist()
        #msg.position = q_rad
        deg_shifted = self.q_deg - 90.0  # centra en 0°
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


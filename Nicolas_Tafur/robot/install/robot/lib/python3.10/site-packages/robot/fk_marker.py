# mi_br/mi_br/fk_marker.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np

def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def to_hom(R, p):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = p
    return T

def Tx(L):
    T = np.eye(4)
    T[2,3] = L
    return T

def forward_kinematics(q_deg, L):
    q = np.deg2rad(np.array(q_deg, dtype=float))
    L1, L2, L3 = L
    T01 = to_hom(rot_z(q[0]), np.zeros(3)) @ Tx(L1)
    T12 = to_hom(rot_y(q[1]), np.zeros(3)) @ Tx(L2)
    T23 = to_hom(rot_y(q[2]), np.zeros(3)) @ Tx(L3)
    T03 = T01 @ T12 @ T23
    p = T03[:3,3]
    return p

class FKMarker(Node):
    def __init__(self):
        super().__init__('fk_marker')
        # Ajusta a tus longitudes reales (m)
        self.L = np.array([0.35, 0.2, 0.2], dtype=float)
        # Nombres de juntas en el mismo orden que tu modelo Z, Y, Y
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.q_deg = np.array([0.0, 0.0, 0.0], dtype=float)

        self.sub = self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.pub = self.create_publisher(Marker, 'ee_marker', 10)
        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz

    def on_js(self, msg: JointState):
        idx = {name: i for i, name in enumerate(msg.name)}
        try:
            q = [msg.position[idx[n]] for n in self.joint_names]
        except KeyError:
            return
        self.q_deg = np.rad2deg(np.array(q))

    def on_timer(self):
        p = forward_kinematics(self.q_deg, self.L)
        m = Marker()
        m.header.frame_id = 'base_link'  # Ajusta a tu frame base del URDF
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'fk'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = float(p[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.02  # 2 cm
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
        self.pub.publish(m)

def main():
    rclpy.init()
    node = FKMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy as np

def to_hom(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def Txyz(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T

def rot_axis(axis, theta):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(theta), np.sin(theta)
    C = 1 - c
    R = np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]
    ])
    return R

def fk_chain(q_deg, origins, axes, ee_offset):
    """
    Calcula la posición del efector final en el frame base_link.
    """
    q = np.deg2rad(q_deg)
    T = np.eye(4)
    for i in range(len(q)):
        T = T @ Txyz(*origins[i]) @ to_hom(rot_axis(axes[i], q[i]), np.zeros(3))
    T = T @ Txyz(*ee_offset)
    return T[:3, 3]

def fk_chain_rel(q_deg, origins, axes, ee_offset, link_index):
    """
    Devuelve la posición del efector vista desde el frame del link intermedio.
    link_index = 1 corresponderá al primer link hijo (link_1 o link_5).
    """
    q = np.deg2rad(q_deg)
    T = np.eye(4)
    frames = [T.copy()]  # índice 0: base_link

    # Acumular transformaciones y guardar cada frame
    for i in range(len(q)):
        T = T @ Txyz(*origins[i]) @ to_hom(rot_axis(axes[i], q[i]), np.zeros(3))
        frames.append(T.copy())

    # Pose final en base_link
    T_ee = T @ Txyz(*ee_offset)

    # Frame de referencia seleccionado
    T_ref = frames[link_index]

    # Transformar pose del EE al frame link_index
    T_rel = np.linalg.inv(T_ref) @ T_ee
    return T_rel[:3, 3]

class FKMarker(Node):
    def __init__(self):
        super().__init__('fk_marker_dual')

        self.joint_names_left  = ['joint1', 'joint2', 'joint3']
        self.joint_names_right = ['joint5', 'joint6', 'joint7']

        self.origins_left = [
            (0.00995, -0.15355, 0.26905),
            (0.0, -0.01998, -0.03175),
            (-0.00786, 0.045, -0.10972)
        ]
        self.axes_left = [
            (0, 0, 1),
            (0, -1, 0),
            (0, -1, 0)
        ]

        self.origins_right = [
            (0.00995, 0.15355, 0.26905),
            (0.0, 0.01998, -0.03025),
            (-0.0034387, 0.005, -0.10995)
        ]
        self.axes_right = [
            (0, 0, -1),
            (0, -1, 0),
            (0, -1, 0)
        ]

        # Offset del efector final en el frame del último link
        self.ee_offset_left  = (0.00117, -0.02999, -0.16845)
        self.ee_offset_right = (0.00117, -0.02999, -0.16845)

        self.q_deg_left  = [0.0, 0.0, 0.0]
        self.q_deg_right = [0.0, 0.0, 0.0]

        self.sub = self.create_subscription(
            JointState, 'joint_states', self.on_js, 10)
        self.pub_left  = self.create_publisher(Marker, 'ee_marker_left', 10)
        self.pub_right = self.create_publisher(Marker, 'ee_marker_right', 10)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_js(self, msg: JointState):
        idx = {name: i for i, name in enumerate(msg.name)}
        try:
            ql = [msg.position[idx[n]] for n in self.joint_names_left]
            qr = [msg.position[idx[n]] for n in self.joint_names_right]
        except KeyError:
            return
        self.q_deg_left  = np.rad2deg(ql)
        self.q_deg_right = np.rad2deg(qr)

    def make_marker(self, pos, ns, mid, color):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def on_timer(self):
        # Posición del EE relativa al frame de link_1 (izq) o link_5 (der)
        p_left_rel  = fk_chain_rel(
            self.q_deg_left,  self.origins_left,  self.axes_left,
            self.ee_offset_left,  link_index=1
        )
        p_right_rel = fk_chain_rel(
            self.q_deg_right, self.origins_right, self.axes_right,
            self.ee_offset_right, link_index=1
        )

        # Imprimir en consola coordenadas relativas
        self.get_logger().info(
            f"Left EE (link_1 frame):  X={round(p_left_rel[0],4)}  "
            f"Y={round(p_left_rel[1],4)}  Z={round(p_left_rel[2],4)}"
        )
        self.get_logger().info(
            f"Right EE (link_5 frame): X={round(p_right_rel[0],4)}  "
            f"Y={round(p_right_rel[1],4)}  Z={round(p_right_rel[2],4)}"
        )

        # Publicar marcadores en RViz (en base_link)
        p_left  = fk_chain(
            self.q_deg_left,  self.origins_left,  self.axes_left,  self.ee_offset_left
        )
        p_right = fk_chain(
            self.q_deg_right, self.origins_right, self.axes_right, self.ee_offset_right
        )

        m_left  = self.make_marker(p_left,  'fk_left',  0, (1.0, 0.2, 0.2, 0.9))
        m_right = self.make_marker(p_right, 'fk_right', 0, (0.2, 0.2, 1.0, 0.9))

        self.pub_left.publish(m_left)
        self.pub_right.publish(m_right)

def main():
    rclpy.init()
    node = FKMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


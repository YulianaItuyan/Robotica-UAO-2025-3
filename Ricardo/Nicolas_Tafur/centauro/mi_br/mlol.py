#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class OlaYDabRealista(Node):
    def __init__(self):
        super().__init__('ola_y_dab_realista')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.tiempo = 0.0
        self.fase = 0

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint5', 'joint6', 'joint7']

        # Fase 0–100: Ola con brazo derecho
        if self.fase < 100:
            joint_state.position = [
                0.0, 0.0, 0.0,  # brazo izquierdo quieto
                math.sin(self.tiempo),               # joint5: hombro (Z)
                math.sin(self.tiempo - 0.5),         # joint6: bíceps (Y)
                math.sin(self.tiempo - 1.0)          # joint7: antebrazo (Y)
            ]
        # Fase 100–200: Ola con brazo izquierdo
        elif self.fase < 200:
            joint_state.position = [
                -math.sin(self.tiempo),              # joint1: hombro (-Z)
                math.sin(self.tiempo - 0.5),         # joint2: bíceps (Y)
                math.sin(self.tiempo - 1.0),         # joint3: antebrazo (Y)
                0.0, 0.0, 0.0                        # brazo derecho quieto
            ]
        # Fase 200+: Dab robótico
        else:
            joint_state.position = [
                -1.2, 1.0, 0.5,   # brazo izquierdo arriba
                0.8, -0.5, -0.3   # brazo derecho cruzado
            ]

        self.publisher_.publish(joint_state)
        self.tiempo += 0.05
        self.fase += 1

def main(args=None):
    rclpy.init(args=args)
    nodo = OlaYDabRealista()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import time

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')

        # Publisher de estados conjuntos
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Suscriptores
        self.subscription_cmd = self.create_subscription(
            JointState,
            '/cmd_deg',
            self.listener_callback,
            10)
        self.subscription_arm = self.create_subscription(
            String,
            '/cmd_arm_select',
            self.arm_select_callback,
            10)

        # Estado inicial
        self.joint_names = [
            'arm_A_joint1', 'arm_A_joint2', 'arm_A_joint3',
            'arm_B_joint1', 'arm_B_joint2', 'arm_B_joint3'
        ]
        self.current_positions = np.zeros(6)   # Posición actual
        self.target_positions = np.zeros(6)    # Meta a alcanzar
        self.start_positions = np.zeros(6)     # Para interpolar
        self.arm_selected = 'A'

        # Parámetros de interpolación
        self.trajectory_duration = 2.0  # segundos para moverse
        self.steps = 50                 # número de pasos en la trayectoria
        self.step_idx = 0
        self.active_motion = False
        self.start_time = 0.0

        # Timer a 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

    def arm_select_callback(self, msg):
        if msg.data in ['A', 'B']:
            self.arm_selected = msg.data
            self.get_logger().info(f'Brazo seleccionado: {self.arm_selected}')

    def listener_callback(self, msg):
        """Recibe 3 posiciones en grados y crea una trayectoria hacia ellas."""
        if len(msg.position) < 3:
            self.get_logger().warn("Mensaje /cmd_deg inválido, se esperaban 3 valores")
            return

        target_deg = np.array(msg.position[:3]) * np.pi / 180.0  # radianes

        # Copiar posición actual como punto de partida
        self.start_positions = self.current_positions.copy()
        self.target_positions = self.current_positions.copy()

        if self.arm_selected == 'A':
            self.target_positions[0:3] = target_deg
        else:
            self.target_positions[3:6] = target_deg

        self.step_idx = 0
        self.active_motion = True
        self.start_time = time.time()
        self.get_logger().info(
            f"Nueva trayectoria hacia {self.target_positions} (brazo {self.arm_selected})"
        )

    def timer_callback(self):
        """Interpola suavemente entre posiciones."""
        if self.active_motion:
            elapsed = time.time() - self.start_time
            alpha = min(elapsed / self.trajectory_duration, 1.0)  # 0→1
            self.current_positions = (
                (1 - alpha) * self.start_positions + alpha * self.target_positions
            )

            if alpha >= 1.0:
                self.active_motion = False
                self.get_logger().info("Movimiento completado")

        # Publicar JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

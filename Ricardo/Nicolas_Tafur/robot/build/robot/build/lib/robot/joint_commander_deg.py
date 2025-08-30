#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
import numpy as np

class JointCommanderDeg(Node):
    def __init__(self):
        super().__init__('joint_commander_deg')
     
        # TODOS los 6 joints del robot
        self.joint_names = [
            'joint1', 'joint2', 'joint3',    # Brazo A (Izquierdo)
            'joint5', 'joint6', 'joint7'     # Brazo B (Derecho)
        ]
        
        # Estado completo de todos los joints (6 joints)
        # Inicializar con valores neutros específicos como tenías antes
        self.q_deg_full = np.array([
            180.0 if name in ('joint3', 'joint7') else 90.0
            for name in self.joint_names
        ], dtype=float)
        
        # Brazo seleccionado actualmente ('A' = Izquierdo, 'B' = Derecho)
        self.selected_arm = 'A'

        # Publisher para joint_states
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Suscriptor para comandos de grados (3 valores por brazo)
        self.sub_cmd = self.create_subscription(
            Float32MultiArray,
            '/cmd_deg',
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
        
        self.timer = self.create_timer(0.02, self.on_timer)  # 50 Hz

        self.get_logger().info(f'JointCommanderDeg iniciado con {len(self.joint_names)} joints')
        self.get_logger().info(f'Joints: {self.joint_names}')
        self.get_logger().info(f'Brazo seleccionado: {self.selected_arm}')
        self.get_logger().info('Esperando comandos en /cmd_deg y /cmd_arm_select')

    def on_arm_select(self, msg: String):
        """Callback para cambio de brazo seleccionado"""
        if msg.data in ['A', 'B']:
            self.selected_arm = msg.data
            arm_name = "Izquierdo" if msg.data == 'A' else "Derecho"
            self.get_logger().info(f'🔄 Brazo seleccionado: {msg.data} ({arm_name})')
        else:
            self.get_logger().warn(f'⚠️ Selección de brazo inválida: {msg.data}')

    def on_cmd_deg(self, msg: Float32MultiArray):
        """Callback para comandos de grados del brazo seleccionado"""
        if len(msg.data) != 3:  # Siempre esperamos 3 joints por brazo
            self.get_logger().warn(f'⚠️ Se esperaban 3 grados, llegaron {len(msg.data)}')
            return
        
        received_angles = np.array(msg.data, dtype=float)
        
        # Actualizar los joints del brazo seleccionado
        if self.selected_arm == 'A':
            # Brazo A: joints 0, 1, 2 (joint1, joint2, joint3)
            self.q_deg_full[0:3] = received_angles
            arm_name = "Izquierdo"
        elif self.selected_arm == 'B':
            # Brazo B: joints 3, 4, 5 (joint5, joint6, joint7) 
            self.q_deg_full[3:6] = received_angles
            arm_name = "Derecho"
        
        self.get_logger().info(f'✅ Comando para brazo {self.selected_arm} ({arm_name}): {received_angles.tolist()}°')
        self.get_logger().info(f'Estado completo: {self.q_deg_full.tolist()}°')

    def on_timer(self):
        """Publica el estado de todos los joints"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Aplicar shift y convertir a radianes para TODOS los 6 joints
        deg_shifted = self.q_deg_full - 90.0
        q_rad = np.deg2rad(deg_shifted)
        
        msg.position = q_rad.tolist()
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.pub.publish(msg)
        
        # Debug ocasional (cada 50 iteraciones = cada segundo)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        if self._debug_counter % 50 == 0:
            self.get_logger().info(f'📡 Estado actual - Brazo {self.selected_arm}: {self.q_deg_full.tolist()}°')

def main():
    rclpy.init()
    node = JointCommanderDeg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
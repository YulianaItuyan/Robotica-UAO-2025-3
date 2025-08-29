#!/usr/bin/env python3
#serial_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # CAMBIADO: Ahora usa JointState
from std_msgs.msg import Float32MultiArray, String
import serial
import time
import math
import numpy as np

class SerialBridgeArduino(Node):
    def __init__(self):
        super().__init__('serial_bridge_arduino')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic_gripper', '/cmd_gripper')
        self.declare_parameter('selection_topic', '/cmd_arm_select')
        self.declare_parameter('send_only_on_change', True)
        self.declare_parameter('reconnect_secs', 2.0)

        # Lectura de parámetros
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.topic_gripper = self.get_parameter('topic_gripper').value
        self.selection_topic = self.get_parameter('selection_topic').value
        self.send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self.reconnect_secs = float(self.get_parameter('reconnect_secs').value)

        # Estado interno: 6 valores (J1,J2,J3,J5,J6,J7) en grados
        self.joint_vals = [90] * 6  # Posiciones en grados
        self.joint_names_expected = [
            'joint1', 'joint2', 'joint3',    # Brazo A
            'joint5', 'joint6', 'joint7'     # Brazo B  
        ]

        self.ser = None
        self.last_frame = None

        # CAMBIO CLAVE: Suscripción a /joint_states en lugar de /cmd_deg
        self.sub_joint_states = self.create_subscription(
            JointState, 
            '/joint_states',  # El tópico interpolado del joint_commander_deg
            self.on_joint_states, 
            10
        )
        
        self.sub_gripper = self.create_subscription(Float32MultiArray, self.topic_gripper, self.on_gripper_cmd, 10)
        self.sub_select = self.create_subscription(String, self.selection_topic, self.on_select, 10)

        # Conexión serie e intentos de reconexión
        self._connect_serial()
        self.reconnect_timer = self.create_timer(self.reconnect_secs, self._reconnect_if_needed)

        self.get_logger().info(f'Serial Bridge ACTUALIZADO:')
        self.get_logger().info(f'  - Escuchando /joint_states (interpolado)')
        self.get_logger().info(f'  - Gripper en {self.topic_gripper}')
        self.get_logger().info(f'  - Enviando a {self.port}@{self.baud}')

    def _connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                return
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.02)
            time.sleep(0.2)
            self.get_logger().info(f'Puerto serie abierto: {self.port} @ {self.baud}')
        except Exception as e:
            self.get_logger().warn(f'No se pudo abrir {self.port}: {e}')

    def _reconnect_if_needed(self):
        if self.ser is None or not self.ser.is_open:
            self._connect_serial()

    @staticmethod
    def _sanitize_angle(x):
        if x is None or (isinstance(x, float) and math.isnan(x)):
            return 90
        try:
            xi = int(round(float(x)))
        except Exception:
            xi = 90
        return max(0, min(180, xi))

    def on_joint_states(self, msg: JointState):
        """
        NUEVO: Callback que recibe joint_states interpolados del joint_commander_deg
        Convierte de radianes a grados y aplica reverse shift
        """
        if len(msg.name) != 6 or len(msg.position) != 6:
            self.get_logger().warn(f'JointState inesperado: {len(msg.name)} nombres, {len(msg.position)} posiciones')
            return

        # Verificar que los nombres coincidan con lo esperado
        if msg.name != self.joint_names_expected:
            self.get_logger().warn(f'Nombres de joints no coinciden. Esperado: {self.joint_names_expected}, Recibido: {msg.name}')
            return

        # Convertir de radianes a grados y aplicar reverse shift (+90)
        # joint_commander_deg hace: deg_shifted = current_positions - 90.0; q_rad = deg2rad(deg_shifted)
        # Nosotros hacemos el reverso: deg = rad2deg(q_rad) + 90.0
        positions_rad = np.array(msg.position)
        positions_deg_shifted = np.rad2deg(positions_rad)
        positions_deg = positions_deg_shifted + 90.0

        # Sanitizar y actualizar estado interno
        self.joint_vals = [self._sanitize_angle(deg) for deg in positions_deg]

        # Crear frame para Arduino
        frame = "<" + ",".join(str(v) for v in self.joint_vals) + ">\n"

        # Enviar solo si hay cambio (si está habilitado)
        if self.send_only_on_change and frame == self.last_frame:
            return

        self.last_frame = frame
        self._send_frame(frame)

        # Debug ocasional
        if hasattr(self, '_debug_count'):
            self._debug_count += 1
        else:
            self._debug_count = 0
            
        if self._debug_count % 50 == 0:  # Cada 50 mensajes
            self.get_logger().info(f'Enviado: {frame.strip()} (desde joint_states interpolados)')

    def on_select(self, msg: String):
        """Mantener compatibilidad con selección de brazo (opcional)"""
        self.get_logger().info(f'Selección de brazo: {msg.data}')

    def on_gripper_cmd(self, msg: Float32MultiArray):
        if not msg.data:
            return
        val = int(msg.data[0])
        frame = f"G<{val}>\n"
        self._send_frame(frame)

    def _send_frame(self, frame: str):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Puerto serie no disponible; intentando reconectar...')
            self._connect_serial()
            if self.ser is None or not self.ser.is_open:
                return

        try:
            self.ser.write(frame.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'Error escribiendo al puerto: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.get_logger().info('Cerrando puerto serie...')
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialBridgeArduino()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
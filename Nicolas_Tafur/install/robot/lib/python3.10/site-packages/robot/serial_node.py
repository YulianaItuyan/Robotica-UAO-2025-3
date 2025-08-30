#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import time
import math

class SerialBridgeArduino(Node):
    def __init__(self):
        super().__init__('serial_bridge_arduino')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/cmd_deg')               # comandos de brazo
        self.declare_parameter('topic_gripper', '/cmd_gripper')    # comandos de gripper
        self.declare_parameter('selection_topic', '/cmd_arm_select')  # selección de brazo
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('group_size', 3)
        self.declare_parameter('active_arm', 'A')                  # 'A' -> J1-3, 'B' -> J5-7
        self.declare_parameter('send_only_on_change', True)
        self.declare_parameter('reconnect_secs', 2.0)
        self.declare_parameter('accept_full_frame', False)         # permitir 6 valores en /cmd_deg

        # Lectura de parámetros
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.topic = self.get_parameter('topic').value
        self.topic_gripper = self.get_parameter('topic_gripper').value
        self.selection_topic = self.get_parameter('selection_topic').value
        self.num_joints = int(self.get_parameter('num_joints').value)
        self.group_size = int(self.get_parameter('group_size').value)
        self.active_arm = str(self.get_parameter('active_arm').value).upper()
        self.send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self.reconnect_secs = float(self.get_parameter('reconnect_secs').value)
        self.accept_full_frame = bool(self.get_parameter('accept_full_frame').value)

        # Validaciones básicas
        if self.num_joints != 6 or self.group_size != 3:
            self.get_logger().warn(
                f'Este nodo asume 2 brazos de 3 juntas cada uno (6 totales). '
                f'num_joints={self.num_joints}, group_size={self.group_size}.'
            )
        if self.active_arm not in ('A', 'B'):
            self.get_logger().warn(f'active_arm inválido "{self.active_arm}", usando "A".')
            self.active_arm = 'A'

        # Estado interno: 6 valores (J1,J2,J3,J5,J6,J7); arranque a 90 para seguridad
        self.joint_vals = [90] * self.num_joints

        self.ser = None
        self.last_frame = None

        # Suscripciones
        self.sub_cmd = self.create_subscription(Float32MultiArray, self.topic, self.on_cmd, 10)
        self.sub_gripper = self.create_subscription(Float32MultiArray, self.topic_gripper, self.on_gripper_cmd, 10)
        self.sub_select = self.create_subscription(String, self.selection_topic, self.on_select, 10)

        # Conexión serie e intentos de reconexión
        self._connect_serial()
        self.reconnect_timer = self.create_timer(self.reconnect_secs, self._reconnect_if_needed)

        self.get_logger().info(
            f'Escuchando {self.topic} (3 valores para brazo activo) y {self.topic_gripper} (gripper). '
            f'Selección de brazo en {self.selection_topic} ("A"=J1-3, "B"=J5-7). '
            f'Enviando por {self.port}@{self.baud} en formato "<v1,v2,v3,v5,v6,v7>\\n".'
        )
        self.get_logger().info(f'Brazo activo inicial: {self.active_arm}')

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

    def _active_indices(self):
        # Mapear A -> [0,1,2] (J1,J2,J3), B -> [3,4,5] (J5,J6,J7)
        return [0, 1, 2] if self.active_arm == 'A' else [3, 4, 5]

    def on_select(self, msg: String):
        val = (msg.data or '').strip().upper()
        # Acepta varias formas comunes
        if val in ('A', 'LEFT', 'L', 'J123', 'ARM1'):
            new_arm = 'A'
        elif val in ('B', 'RIGHT', 'R', 'J567', 'ARM2'):
            new_arm = 'B'
        else:
            self.get_logger().warn(f'Valor de selección desconocido: "{msg.data}". Usa "A" o "B".')
            return

        if new_arm != self.active_arm:
            self.active_arm = new_arm
            self.get_logger().info(f'Brazo activo cambiado a: {self.active_arm}')

    def on_cmd(self, msg: Float32MultiArray):
        n = len(msg.data)

        # Opción 1: mensajes de 3 valores (recomendado) -> solo actualiza el brazo activo
        if n == self.group_size:
            cleaned = [self._sanitize_angle(v) for v in msg.data]
            idx = self._active_indices()
            for k, i in enumerate(idx):
                self.joint_vals[i] = cleaned[k]

        # Opción 2: mensajes de 6 valores (opcional, si se permite)
        elif n == self.num_joints and self.accept_full_frame:
            self.joint_vals = [self._sanitize_angle(v) for v in msg.data]
        else:
            self.get_logger().warn(
                f'Longitud inesperada en {self.topic}: {n}. '
                f'Esperados {self.group_size} (brazo activo) '
                f'o {self.num_joints} si accept_full_frame=True.'
            )
            return

        frame = "<" + ",".join(str(v) for v in self.joint_vals) + ">\n"

        if self.send_only_on_change and frame == self.last_frame:
            return

        self.last_frame = frame
        self._send_frame(frame)

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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import math

class SerialBridgeArduino(Node):
    def __init__(self):
        super().__init__('serial_bridge_arduino')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/cmd_deg')
        self.declare_parameter('gripper_topic', '/cmd_gripper')
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('send_only_on_change', True)
        self.declare_parameter('reconnect_secs', 2.0)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.topic = self.get_parameter('topic').value
        self.gripper_topic = self.get_parameter('gripper_topic').value
        self.num_joints = int(self.get_parameter('num_joints').value)
        self.send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self.reconnect_secs = float(self.get_parameter('reconnect_secs').value)

        self.ser = None
        self.last_frame = None
        
        # Estado del gripper: [IN1, IN2, IN3, IN4] para L298N
        self.gripper_state = [0, 0, 0, 0]  # Inicialmente detenido

        # Suscripciones
        self.sub = self.create_subscription(Float32MultiArray, self.topic, self.on_cmd, 10)
        self.gripper_sub = self.create_subscription(Float32MultiArray, self.gripper_topic, self.on_gripper_cmd, 10)

        # Intentar conectar
        self._connect_serial()

        # Timer de reconexión
        self.reconnect_timer = self.create_timer(self.reconnect_secs, self._reconnect_if_needed)

        self.get_logger().info(
            f'Escuchando {self.topic} (joints) y {self.gripper_topic} (gripper). '
            f'Enviando por {self.port}@{self.baud}'
        )

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

    def on_gripper_cmd(self, msg: Float32MultiArray):
        """Procesa comandos del gripper: 1=cerrar, -1=abrir, 0=detener."""
        if len(msg.data) != 1:
            self.get_logger().warn(f'Gripper: se esperaba 1 valor, llegaron {len(msg.data)}')
            return
        
        command = int(msg.data[0])
        
        if command == 1:  # Cerrar gripper
            self.gripper_state = [1, 0, 1, 0]  # Motor A y B en una dirección
        elif command == -1:  # Abrir gripper
            self.gripper_state = [0, 1, 0, 1]  # Motor A y B en dirección opuesta
        else:  # Detener (command == 0 o cualquier otro valor)
            self.gripper_state = [0, 0, 0, 0]  # Ambos motores detenidos
        
        self.get_logger().info(f'Gripper comando: {command} -> Estado: {self.gripper_state}')
        
        # Enviar inmediatamente el estado del gripper
        self._send_gripper_frame()

    def on_cmd(self, msg: Float32MultiArray):
        """Procesa comandos de los joints (servos)."""
        if len(msg.data) != self.num_joints:
            self.get_logger().warn(
                f'Se esperaban {self.num_joints} valores, llegaron {len(msg.data)}'
            )
            return

        # Limpieza y clip a [0, 180]
        vals = [self._sanitize_angle(v) for v in msg.data]
        
        # Crear frame con joints + gripper
        # Formato: "<v1,v2,v3,v4,v5,v6,g1,g2,g3,g4>\n"
        all_vals = vals + self.gripper_state
        frame = "<" + ",".join(str(v) for v in all_vals) + ">\n"

        if self.send_only_on_change and frame == self.last_frame:
            return

        self.last_frame = frame
        self._send_frame(frame)

    def _send_gripper_frame(self):
        """Envía solo el estado del gripper manteniendo los últimos valores de joints."""
        # Si no tenemos valores previos de joints, usar neutros
        if self.last_frame is None:
            joint_vals = [90, 90, 90, 90, 90, 90]  # Valores neutros
        else:
            # Extraer valores de joints del último frame
            try:
                start = self.last_frame.index('<') + 1
                end = self.last_frame.index('>')
                vals_str = self.last_frame[start:end].split(',')
                joint_vals = [int(v) for v in vals_str[:6]]  # Tomar solo los primeros 6
            except:
                joint_vals = [90, 90, 0, 90, 90, 0]  # Fallback a neutros
        
        # Crear frame con joints actuales + nuevo estado del gripper
        all_vals = joint_vals + self.gripper_state
        frame = "<" + ",".join(str(v) for v in all_vals) + ">\n"
        self._send_frame(frame)

    def _send_frame(self, frame):
        """Envía frame por puerto serie."""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Puerto serie no disponible; intentando reconectar...')
            self._connect_serial()
            if self.ser is None or not self.ser.is_open:
                return

        try:
            self.ser.write(frame.encode('ascii'))
            self.get_logger().debug(f'Enviado: {frame.strip()}')
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

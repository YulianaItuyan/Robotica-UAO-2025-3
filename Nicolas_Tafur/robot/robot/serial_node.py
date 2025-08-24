# mi_br/mi_br/serial_bridge_arduino.py
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
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/cmd_deg')
        self.declare_parameter('topic_gripper', '/cmd_gripper')
        self.declare_parameter('send_only_on_change', True)
        self.declare_parameter('reconnect_secs', 2.0)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.topic = self.get_parameter('topic').value
        
        # OBTENEMOS EL VALOR DEL PARÁMETRO CORRECTAMENTE
        self.topic_gripper = self.get_parameter('topic_gripper').value 
        
        self.send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self.reconnect_secs = float(self.get_parameter('reconnect_secs').value)

        self.ser = None
        self.last_frame = None

        # Suscripciones
        self.sub = self.create_subscription(Float32MultiArray, self.topic, self.on_cmd, 10)
        self.sub_gripper = self.create_subscription(Float32MultiArray, self.topic_gripper, self.on_gripper_cmd, 10)


        # Intentar conectar
        self._connect_serial()

        # Timer de reconexión si se cae el puerto
        self.reconnect_timer = self.create_timer(self.reconnect_secs, self._reconnect_if_needed)

        self.get_logger().info(f'Escuchando {self.topic} y {self.topic_gripper}. '
                               f'Enviando por {self.port}@{self.baud}.')

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

    def on_cmd(self, msg: Float32MultiArray):
        """Callback para los comandos del brazo."""
        if len(msg.data) != 3:
            self.get_logger().warn(f'Se esperaban 3 valores para el brazo, llegaron {len(msg.data)}')
            return

        a, b, c = [self._sanitize_angle(v) for v in msg.data]
        frame = f"<{a},{b},{c}>\n"

        if self.send_only_on_change and frame == self.last_frame:
            return

        self.last_frame = frame
        self._send_frame(frame)

    def on_gripper_cmd(self, msg: Float32MultiArray):
        """Callback para los comandos del gripper."""
        if not msg.data:
            return  # no hay datos
        val = int(msg.data[0])  # primer valor enviado desde la GUI
        frame = f"G<{val}>\n"
        self._send_frame(frame)


    def _send_frame(self, frame):
        """Función auxiliar para enviar datos por el puerto serial."""
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
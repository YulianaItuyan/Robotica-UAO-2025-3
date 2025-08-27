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
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/cmd_deg')
        self.declare_parameter('send_only_on_change', True)
        self.declare_parameter('reconnect_secs', 2.0)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.topic = self.get_parameter('topic').value
        self.send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self.reconnect_secs = float(self.get_parameter('reconnect_secs').value)

        self.ser = None
        self.last_frame = None

        # Suscripción
        self.sub = self.create_subscription(Float32MultiArray, self.topic, self.on_cmd, 10)

        # Intentar conectar
        self._connect_serial()

        # Timer de reconexión si se cae el puerto
        self.reconnect_timer = self.create_timer(self.reconnect_secs, self._reconnect_if_needed)

        self.get_logger().info(f'Escuchando {self.topic} (Float32MultiArray con 3 grados). '
                               f'Enviando por {self.port}@{self.baud} en formato "<a,b,c>\\n".')

    def _connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                return
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.02)
            time.sleep(0.2)  # pequeña pausa tras abrir el puerto
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
        if len(msg.data) != 3:
            self.get_logger().warn(f'Se esperaban 3 valores, llegaron {len(msg.data)}')
            return

        # Limpieza y clip a [0, 180]
        a, b, c = [self._sanitize_angle(v) for v in msg.data]
        frame = f"<{a},{b},{c}>\n"

        # Evita tráfico redundante si está activado
        if self.send_only_on_change and frame == self.last_frame:
            return

        self.last_frame = frame

        # Enviar por serie
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Puerto serie no disponible; intentando reconectar...')
            self._connect_serial()
            if self.ser is None or not self.ser.is_open:
                return

        try:
            self.ser.write(frame.encode('ascii'))
            # self.get_logger().info(f'Enviado: {frame.strip()}')  # descomenta si quieres ver cada envío
        except Exception as e:
            self.get_logger().error(f'Error escribiendo al puerto: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None  # forzar reconexión

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


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
class ArmSerialBridge(Node):
    def __init__(self):
        super().__init__('arm_serial_bridge')

        # Suscriptor al tópico con los ángulos
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_deg',
            self.listener_callback,
            10
        )

        # Inicializar conexión serial con Arduino
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)  # esperar a que Arduino reinicie
            self.get_logger().info("✅ Conectado al Arduino en /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"❌ Error abriendo serial: {e}")
            self.ser = None

    def listener_callback(self, msg: Float32MultiArray):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("⚠️ No hay conexión serial")
            return

        if len(msg.data) < 6:
            self.get_logger().warn("⚠️ Se esperaban 6 valores (3 por brazo)")
            return

        # Separar ángulos para los dos brazos
        b1 = [int(msg.data[0]), int(msg.data[1]), int(msg.data[2])]
        b2 = [int(msg.data[3]), int(msg.data[4]), int(msg.data[5])]

        # Enviar al Arduino: primero brazo derecho (2)
        cmd_b1 = f"G<2>\n<{b1[0]},{b1[1]},{b1[2]}>\n"
        self.ser.write(cmd_b1.encode('utf-8'))
        self.get_logger().info(f"📤 Enviado Brazo 1: {cmd_b1.strip()}")

        # Luego brazo izquierdo (3)
        cmd_b2 = f"G<3>\n<{b2[0]},{b2[1]},{b2[2]}>\n"
        self.ser.write(cmd_b2.encode('utf-8'))
        self.get_logger().info(f"📤 Enviado Brazo 2: {cmd_b2.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

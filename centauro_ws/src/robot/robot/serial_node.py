#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import time
import re

# ====== Utilidades de CRC-8 Dallas/Maxim (poly 0x31, init 0x00) ======
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def make_line(payload: str) -> bytes:
    """Construye: b'PAYLOAD*CS\\n' con CS en hex mayúsculas."""
    cs = crc8_maxim(payload.encode('utf-8'))
    return f"{payload}*{cs:02X}\n".encode('utf-8')

def parse_and_verify(line: str):
    """
    Recibe una línea (con o sin '\n'), verifica '*XX' final y devuelve (ok, payload).
    ok=False si no tiene CRC válido.
    """
    line = line.strip()
    star = line.rfind('*')
    if star < 0 or len(line) - star != 3:
        return False, None
    payload = line[:star]
    try:
        rx = int(line[star+1:], 16)
    except ValueError:
        return False, None
    calc = crc8_maxim(payload.encode('utf-8'))
    return (rx == calc), payload


class ArmSerialBridge(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Suscriptores
        self.subscription = self.create_subscription(
            Float32MultiArray, '/cmd_deg', self.listener_callback, 10
        )
        self.sub_arm_select = self.create_subscription(
            String, '/cmd_arm_select', self.on_arm_selection, 10
        )

        # Publicadores
        # Ahora /arm_feedback es Float32MultiArray con 3 ángulos medidos
        self.pub_feedback = self.create_publisher(Float32MultiArray, '/arm_feedback', 10)

        self.selected_arm = 'A'

        # Timer para leer serial
        self.create_timer(0.01, self.read_serial)  # 100 Hz lectura

        # Conexión serial
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
            time.sleep(2)  # esperar a que Arduino reinicie
            self.get_logger().info("✅ Conectado al Arduino en /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"❌ Error abriendo serial: {e}")
            self.ser = None

        # Buffer de recepción si decides usar read() en vez de readline()
        self._rx_buf = bytearray()

    # ====== Envío con CRC ======
    def _send_payload(self, payload: str):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("⚠️ No hay conexión serial (tx)")
            return
        pkt = make_line(payload)
        self.ser.write(pkt)
        self.get_logger().info(f"📤 TX: {pkt.decode(errors='ignore').strip()}")

    # ====== Callback ROS: /cmd_deg ======
    def listener_callback(self, msg: Float32MultiArray):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("⚠️ No hay conexión serial")
            return

        if len(msg.data) < 6:
            self.get_logger().warn("⚠️ Se esperaban 6 valores (3 por brazo)")
            return

        # Separar ángulos por brazo (enteros 0..180)
        b1 = [int(msg.data[0]), int(msg.data[1]), int(msg.data[2])]
        b2 = [int(msg.data[3]), int(msg.data[4]), int(msg.data[5])]

        if self.selected_arm.upper() == 'A':
            # Brazo derecho → canal 2 en Arduino
            self._send_payload("G<2>")
            self._send_payload(f"<{b1[0]},{b1[1]},{b1[2]}>")
        else:
            # Brazo izquierdo → canal 3 en Arduino
            self._send_payload("G<3>")
            self._send_payload(f"<{b2[0]},{b2[1]},{b2[2]}>")

    def on_arm_selection(self, msg: String):
        arm = (msg.data or '').strip().upper()
        if arm in ('A', 'B'):
            self.selected_arm = arm
            self.get_logger().info(f"🔀 Brazo seleccionado: {self.selected_arm}")
        else:
            self.get_logger().warn(f"Brazo inválido: '{msg.data}' (usa 'A' o 'B')")

    # ====== Recepción con CRC ======
    _re_ok = re.compile(r'^(OKA|OKB)<([^>]*)>$')

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            # Usamos readline() para simplificar en este caso
            raw = self.ser.readline()
            if not raw:
                return
            line = raw.decode(errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error leyendo serial: {e}")
            return

        ok, payload = parse_and_verify(line)
        if not ok:
            # Comentar si no quieres ruido
            self.get_logger().warn(f"❌ CRC inválido: {line}")
            return

        self.get_logger().info(f"📥 RX: {payload}")

        # Esperamos OKA<meas1,meas2,meas3> o OKB<...>
        m = self._re_ok.match(payload)
        if not m:
            # Puedes manejar ACKA/ACKB/ERR<CS> aquí si quieres
            return

        tag = m.group(1)   # OKA / OKB
        body = m.group(2)  # "m1,m2,m3"

        try:
            parts = [p.strip() for p in body.split(',')]
            vals = [float(p) for p in parts if p != '']
            if len(vals) != 3:
                raise ValueError("Se esperaban 3 valores")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Parse de {tag} inválido: {body} ({e})")
            return

        # Publica los 3 medidos del brazo que reportó
        out = Float32MultiArray()
        out.data = vals
        self.pub_feedback.publish(out)
        self.get_logger().info(f"✅ Publicado /arm_feedback ({tag}): {vals}")

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

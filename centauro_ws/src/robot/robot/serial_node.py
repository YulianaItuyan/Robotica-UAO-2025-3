#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Empty
import serial
from serial import SerialException
import time
import re

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
        # Botón de reconexión manual (otro nodo publicará Empty en /serial_connect)
        self.sub_reconnect = self.create_subscription(
            Empty, '/serial_connect', self.on_reconnect_request, 10
        )

        # Publicadores
        self.pub_feedback = self.create_publisher(Float32MultiArray, '/arm_feedback', 10)
        self.pub_port     = self.create_publisher(String, '/serial_port', 10)

        self.selected_arm = 'A'

        # Conexión serial
        self.port = '/dev/ttyACM0'
        self.ser = None
        self._port_announced = False   # evita doble anuncio al inicio

        # Timers
        self.create_timer(0.01, self.read_serial)    # lectura ~100 Hz
        self.create_timer(1.0, self._auto_reconnect) # intento de reconexión si está desconectado

        # Regex para OKA/OKB
        self._re_ok = re.compile(r'^(OKA|OKB)<([^>]*)>$')

        # Intento inicial
        self.try_connect()  # anunciará puerto o "SIN PUERTO"

        # Anuncio único si aún no se ha anunciado
        self.create_timer(0.05, self._publish_port_once)

    # ---------- Utilidades de anuncio ----------
    def _announce_port(self, text: str):
        msg = String(); msg.data = text
        self.pub_port.publish(msg)
        self._port_announced = True
        self.get_logger().info(f"📢 /serial_port: {text}")

    def _publish_port_once(self):
        if not self._port_announced:
            # Si por alguna razón no se anunció aún, anuncia el puerto nominal
            self._announce_port(self.port)

    # ---------- Conexión / Desconexión ----------
    def try_connect(self):
        """Intenta abrir el puerto; anuncia estado."""
        try:
            # Cerrar si estaba abierto
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(0.05)

            self.ser = serial.Serial(self.port, 115200, timeout=0.01)
            time.sleep(2)  # esperar a que Arduino reinicie
            self.get_logger().info("✅ Conectado al Arduino en " + self.port)

            self._announce_port(self.port)
        except Exception as e:
            self.get_logger().error(f"❌ Error abriendo serial {self.port}: {e}")
            self.ser = None
            self._announce_port("SIN PUERTO")

    def _handle_disconnect(self, reason: str):
        """Maneja una desconexión en caliente: cierra y anuncia 'SIN PUERTO'."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.get_logger().warn(f"🔌 Serial desconectado ({reason}).")
        self._announce_port("SIN PUERTO")

    def _auto_reconnect(self):
        """Timer 1.0 s: si está desconectado, intenta reconectar automáticamente."""
        if self.ser is None:
            self.get_logger().debug("Intentando reconexión automática…")
            # Evita spamear logs si no está aún el dispositivo
            try:
                self.try_connect()
            except Exception as e:
                self.get_logger().debug(f"Auto-reconexión fallida: {e}")

    # ---------- Envío ----------
    def _send_payload(self, payload: str):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("⚠️ No hay conexión serial (tx)")
            return
        try:
            line = (payload + "\n").encode('utf-8')
            self.ser.write(line)
            self.get_logger().info(f"📤 TX: {payload}")
        except (SerialException, OSError) as e:
            # Si el dispositivo se fue en caliente durante un write
            self.get_logger().error(f"⚠️ Error TX; posible desconexión: {e}")
            self._handle_disconnect("write failure")

    # ---------- Callbacks ----------
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
            self._send_payload("G<2>")  # Brazo derecho
            self._send_payload(f"<{b1[0]},{b1[1]},{b1[2]}>")
        else:
            self._send_payload("G<3>")  # Brazo izquierdo
            self._send_payload(f"<{b2[0]},{b2[1]},{b2[2]}>")

    def on_arm_selection(self, msg: String):
        arm = (msg.data or '').strip().upper()
        if arm in ('A', 'B'):
            self.selected_arm = arm
            self.get_logger().info(f"🔀 Brazo seleccionado: {self.selected_arm}")
        else:
            self.get_logger().warn(f"Brazo inválido: '{msg.data}' (usa 'A' o 'B')")

    def on_reconnect_request(self, _msg: Empty):
        self.get_logger().info("🔁 Reconexión manual solicitada (/serial_connect)")
        # Permitimos que se vuelva a anunciar el estado actual
        self._port_announced = False
        self.try_connect()

    # ---------- Recepción ----------
    def read_serial(self):
        if self.ser is None:
            return
        if not self.ser.is_open:
            self._handle_disconnect("port closed")
            return

        try:
            raw = self.ser.readline()
            if not raw:
                return
            line = raw.decode(errors='ignore').strip()
            if not line:
                return
        except (SerialException, OSError) as e:
            # USB desconectado en caliente durante lectura
            self.get_logger().error(f"⚠️ Error RX; posible desconexión: {e}")
            self._handle_disconnect("read failure")
            return
        except Exception as e:
            self.get_logger().warn(f"⚠️ Error leyendo serial: {e}")
            return

        self.get_logger().info(f"📥 RX: {line}")

        # Esperamos OKA<meas1,meas2,meas3> o OKB<...>
        m = self._re_ok.match(line)
        if not m:
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
        try:
            if node.ser and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
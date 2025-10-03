#!/usr/bin/env python3
"""
arm_serial_cli.py — Envío/recepción simple para tu Arduino (sin ROS).

Requisitos:
  pip install pyserial

Uso típico:
  python arm_serial_cli.py -p /dev/ttyACM0 -b 115200
  # En el prompt, elige brazo: A o B
  # Luego envía metas: por ejemplo "120 60 30"
  # Comandos:
  #   arm A           -> seleccionar brazo A
  #   arm B           -> seleccionar brazo B
  #   send 120 60 30  -> enviar 3 ángulos (0..180)
  #   mode tag        -> usar protocolo de 1 trama A<...>/B<...>
  #   mode two        -> usar G<2|3> + <a,b,c> (por defecto)
  #   quit            -> salir
"""

import sys
import time
import threading
import argparse
import re

try:
    import serial
except ImportError:
    print("Falta pyserial. Instala con: pip install pyserial")
    sys.exit(1)


OK_RE = re.compile(r'^(OKA|OKB)<([^>]*)>$')
ACK_RE = re.compile(r'^(ACKA|ACKB)<([^>]*)>$')


class SerialWorker:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # Pequeña pausa para que Arduino reinicie
        time.sleep(2.0)
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._rx_loop, daemon=True)
        self._buf = bytearray()
        self.port = port

    def start(self):
        print(f"✅ Conectado a {self.port} @ {self.ser.baudrate}")
        self._th.start()

    def stop(self):
        self._stop.set()
        try:
            self._th.join(timeout=1.0)
        except Exception:
            pass
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("👋 Serial cerrado.")

    # -------- TX helpers --------
    def send_line(self, payload: str):
        if not self.ser or not self.ser.is_open:
            print("⚠️ Puerto no abierto.")
            return
        line = (payload + "\n").encode("utf-8", errors="ignore")
        self.ser.write(line)
        print(f"➡️  TX: {payload}")

    def select_arm_twoframe(self, arm: str):
        """Protocolo de 2 tramas: G<2|3>"""
        tag = arm.upper()
        if tag == "A":
            self.send_line("G<2>")
        elif tag == "B":
            self.send_line("G<3>")
        else:
            print("⚠️ Brazo inválido (use A o B).")

    def send_angles_twoframe(self, a: int, b: int, c: int):
        """Protocolo de 2 tramas: <a,b,c>"""
        self.send_line(f"<{a},{b},{c}>")

    def send_angles_tag(self, arm: str, a: int, b: int, c: int):
        """Protocolo de 1 trama: A<...> / B<...>"""
        tag = arm.upper()
        if tag not in ("A", "B"):
            print("⚠️ Brazo inválido (use A o B).")
            return
        self.send_line(f"{tag}<{a},{b},{c}>")

    # -------- RX loop --------
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(128)
            except Exception as e:
                print(f"⚠️ Error leyendo serial: {e}")
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            self._buf.extend(chunk)
            while b"\n" in self._buf:
                line, _, rest = self._buf.partition(b"\n")
                self._buf = bytearray(rest)
                text = line.decode("utf-8", errors="ignore").strip()
                if text:
                    self._handle_rx_line(text)

    def _handle_rx_line(self, line: str):
        # Log general
        stamp = time.strftime("%H:%M:%S")
        print(f"⬅️  RX [{stamp}]: {line}")

        # Parse opcional de OK/ACK
        m = OK_RE.match(line)
        if m:
            tag, body = m.group(1), m.group(2)
            parts = [p.strip() for p in body.split(",")]
            # Solo para feedback visual
            print(f"   ✔ Llegó {tag}: {parts}")
            return

        m = ACK_RE.match(line)
        if m:
            tag, body = m.group(1), m.group(2)
            print(f"   ✔ ACK {tag}: {body}")
            return
        # Otros prints del Arduino (setup, warnings, etc.) ya quedaron arriba.


def clamp_angle(x):
    try:
        v = float(x)
    except Exception:
        raise ValueError("Ángulo no numérico")
    if v < 0:
        v = 0
    if v > 180:
        v = 180
    return int(round(v))


def main():
    ap = argparse.ArgumentParser(description="CLI simple para tu Arduino de servos (sin ROS)")
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0", help="Puerto serie (e.g., /dev/ttyACM0 o /dev/ttyUSB0)")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="Baudios")
    ap.add_argument("--mode", choices=["two", "tag"], default="test",
                    help="Protocolo: 'two' = G<2|3> + <a,b,c> (por defecto), 'tag' = A<...>/B<...>")
    ap.add_argument("--arm", choices=["A", "B"], default="A", help="Brazo iTEnicial (A/B)")
    args = ap.parse_args()

    try:
        worker = SerialWorker(args.port, args.baud)
    except Exception as e:
        print(f"❌ No pude abrir {args.port}: {e}")
        print("   Sugerencias:")
        print("   - Verifica el nombre del puerto (ls /dev/ttyACM* /dev/ttyUSB*)")
        print("   - Agrega tu usuario al grupo dialout: sudo usermod -aG dialout $USER (y re-login)")
        sys.exit(1)

    worker.start()

    mode = args.mode      # "two" o "tag"
    arm = args.arm.upper()  # "A" o "B"

    print("\n=== Controles ===")
    print("  arm A|B          -> seleccionar brazo")
    print("  send a b c       -> enviar 3 ángulos (0..180)")
    print("  mode two|tag     -> cambiar protocolo")
    print("  test             -> demo rápida (120,60,30)")
    print("  quit             -> salir\n")

    # Selección inicial si estamos en modo "two"
    if mode == "two":
        worker.select_arm_twoframe(arm)

    try:
        while True:
            cmd = input("> ").strip()
            if not cmd:
                continue

            if cmd.lower() in ("q", "quit", "exit"):
                break

            if cmd.lower().startswith("arm "):
                _, val = cmd.split(None, 1)
                val = val.strip().upper()
                if val not in ("A", "B"):
                    print("⚠️ Usa: arm A | arm B")
                    continue
                arm = val
                if mode == "two":
                    worker.select_arm_twoframe(arm)
                print(f"🔀 Brazo seleccionado: {arm}")
                continue

            if cmd.lower().startswith("mode "):
                _, val = cmd.split(None, 1)
                val = val.strip().lower()
                if val not in ("two", "tag"):
                    print("⚠️ Usa: mode two | mode tag")
                    continue
                mode = val
                print(f"🔧 Protocolo: {mode}")
                # En modo two, conviene re-seleccionar brazo para dejar contexto claro
                if mode == "two":
                    worker.select_arm_twoframe(arm)
                continue

            if cmd.lower() == "test":
                a, b, c = 120, 60, 30
                if mode == "two":
                    worker.select_arm_twoframe(arm)
                    worker.send_angles_twoframe(a, b, c)
                else:
                    worker.send_angles_tag(arm, a, b, c)
                continue

            if cmd.lower().startswith("send "):
                parts = cmd.split()
                if len(parts) != 4:
                    print("⚠️ Usa: send a b c")
                    continue
                try:
                    a = clamp_angle(parts[1])
                    b = clamp_angle(parts[2])
                    c = clamp_angle(parts[3])
                except ValueError as e:
                    print(f"⚠️ {e}")
                    continue

                if mode == "two":
                    worker.select_arm_twoframe(arm)
                    worker.send_angles_twoframe(a, b, c)
                else:
                    worker.send_angles_tag(a, b, c)
                continue

            print("❓ Comando no reconocido. Opciones: arm, send, mode, test, quit")

    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()


if __name__ == "__main__":
    main()

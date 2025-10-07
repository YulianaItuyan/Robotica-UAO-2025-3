#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class FKManager(Node):
    def __init__(self):
        super().__init__('fk_node')

        # ================== Suscriptores ==================
        self.fk_sub = self.create_subscription(Float32MultiArray, 'fk_goal', self.goal_callback, 10)
        self.sub_arm_select = self.create_subscription(String, '/cmd_arm_select', self.on_arm_select, 10)
        self.sub_run_mode = self.create_subscription(String, '/run_mode', self.run_mode, 10)
        self.sub_feedback = self.create_subscription(Float32MultiArray, '/arm_feedback', self.on_feedback, 10)

        # ================== Publicadores ==================
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/cmd_deg', 10)   # vida real
        self.sim_pub = self.create_publisher(Float32MultiArray, '/sim_deg', 10)

        # ============ Estado general para publicar/mostrar =========
        # Mantendremos all_joints como ESTADO MEDIDO (lo que de verdad tiene el brazo)
        self.all_joints = [90.0, 90.0, 90.0, 90.0, 90.0, 0.0]
        # <<< CAMBIO REAL: buffer de comandos para publicar (no toca estado medido)
        self.all_cmd = [90.0, 90.0, 90.0, 90.0, 90.0, 0.0]

        # ================== Selecciones ====================
        self.selected_arm = 'A'     # 'A' = derecho, 'B' = izquierdo
        self.selected_mode = 'C'    # 'C' = simulación, 'R' = vida real

        # ================== Trayectoria ====================
        self.traj_active = False
        self.i = 0
        self.step = 5  # pasos de interpolación (ajustable)

        # Estados en espacio q (no servo)
        self.q1 = np.array([90.0, 90.0, 90.0], dtype=float)
        self.q2 = np.array([0.0, 0.0, 0.0], dtype=float)

        # ======== ESTADOS SEPARADOS PARA SIM Y REAL =========
        self.q_curr_sim_A = np.array([90.0, 90.0, 90.0], dtype=float)
        self.q_curr_sim_B = np.array([90.0, 90.0, 0.0], dtype=float)
        self.last_sim_q_cmd_A = None
        self.last_sim_q_cmd_B = None

        # Vida real: se actualiza con FEEDBACK (medidos)
        self.q_curr_real_A = np.array([90.0, 90.0, 90.0], dtype=float)
        self.q_curr_real_B = np.array([90.0, 90.0, 0.0], dtype=float)
        # <<< CAMBIO REAL: guardar últimos comandos enviados en real
        self.last_real_q_cmd_A = None
        self.last_real_q_cmd_B = None

        # Control de feedback (solo real)
        self.waiting_ack = False
        self.first_step_sent = False  # primer paso se envía sin esperar feedback

        # Tolerancia para considerar que alcanzó (deg)
        self.tol = np.array([0.5, 0.5, 0.5], dtype=float)

        # Timer principal (50 Hz aprox)
        self.timer = self.create_timer(0.02, self.timer_cb)

    # ================== Utilidades ==================
    def _get_curr_for_mode(self, arm: str) -> np.ndarray:
        if self.selected_mode == 'C':
            return (self.q_curr_sim_A if arm == 'A' else self.q_curr_sim_B).copy()
        else:
            return (self.q_curr_real_A if arm == 'A' else self.q_curr_real_B).copy()

    def _set_sim_curr(self, arm: str, q: np.ndarray) -> None:
        if arm == 'A':
            self.q_curr_sim_A = q.copy()
        else:
            self.q_curr_sim_B = q.copy()

    def _set_real_curr(self, arm: str, q: np.ndarray) -> None:
        if arm == 'A':
            self.q_curr_real_A = q.copy()
        else:
            self.q_curr_real_B = q.copy()

    def _current_q_along_traj(self) -> np.ndarray:
        if not self.traj_active:
            return None
        s = self.i / float(self.step)
        return (1.0 - s) * self.q1 + s * self.q2

    def _abort_current(self, arm_running: str, mode_running: str, reason: str):
        q_at = self._current_q_along_traj()
        if q_at is None:
            return

        if mode_running == 'C':
            if arm_running == 'A':
                self.q_curr_sim_A = q_at.copy()
            else:
                self.q_curr_sim_B = q_at.copy()
        else:
            # En real, al abortar NO modificamos el medido; queda como estaba.
            self._set_real_curr(arm_running, self._get_curr_for_mode(arm_running))

        self.traj_active = False
        self.waiting_ack = False
        self.first_step_sent = False
        self.get_logger().info(f"⏹️ Trayectoria abortada ({reason}). Estado {'SIM' if mode_running=='C' else 'REAL'} del brazo {arm_running} actualizado.")

    def _build_cmd_payload(self, out3, arm: str):
        """Construye un arreglo de 6 para /cmd_deg SIN tocar all_joints (estado medido)."""
        arr = self.all_cmd[:]  # copiar para no pisar
        start = 0 if arm == 'A' else 3
        for k, v in enumerate(out3):
            arr[start + k] = float(v)
        return arr

    # ================== Callbacks ==================
    def goal_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.q2 = np.array(msg.data[0:3] if self.selected_arm == 'A' else msg.data[3:6], dtype=float)
        elif len(msg.data) >= 3:
            self.q2 = np.array(msg.data[0:3], dtype=float)
        else:
            self.get_logger().warn("fk_goal necesita 3 o 6 valores (float).")
            return

        self.q1 = self._get_curr_for_mode(self.selected_arm)

        if np.allclose(self.q1, self.q2, atol=1e-3):
            self.get_logger().info("🎯 q1 == q2, no hay trayectoria que ejecutar.")
            return

        self.i = 0
        self.traj_active = True
        self.waiting_ack = False
        self.first_step_sent = False

        self.get_logger().info(
            f"▶️ Interpolación {self.selected_arm} [{'SIM' if self.selected_mode=='C' else 'REAL'}]: "
            f"{self.q1} → {self.q2} ({self.step} pasos)"
        )

    def on_arm_select(self, msg: String):
        new_arm = (msg.data or '').strip().upper()
        if new_arm not in ('A', 'B'):
            self.get_logger().warn(f"Brazo inválido: '{msg.data}' (usa 'A' o 'B')")
            return

        if self.traj_active and new_arm != self.selected_arm:
            prev_arm = self.selected_arm
            prev_mode = self.selected_mode
            self._abort_current(prev_arm, prev_mode, "cambio de brazo")

        self.selected_arm = new_arm
        self.get_logger().info(f"🔀 Brazo seleccionado: {self.selected_arm}")

    def run_mode(self, msg: String):
        val = (msg.data or '').strip().lower()
        new_mode = 'C' if val in ('c', 'sim', 'simulacion', 'simulation') else 'R'

        if self.traj_active and new_mode != self.selected_mode:
            prev_arm = self.selected_arm
            prev_mode = self.selected_mode
            self._abort_current(prev_arm, prev_mode, "cambio de modo")

        self.selected_mode = new_mode
        self.get_logger().info("🧪 Modo: C (simulación)" if new_mode=='C' else "🔧 Modo: R (vida real)")

    def on_feedback(self, msg: Float32MultiArray):
        vals = list(msg.data or [])
        if len(vals) != 3:
            self.get_logger().warn(f"⚠️ /arm_feedback inválido: {vals}")
            return

        meas = np.array(vals, dtype=float)
        # <<< CAMBIO REAL: estado medido SOLO con feedback
        self._set_real_curr(self.selected_arm, meas)

        if self.selected_mode == 'R' and self.traj_active:
            err = np.abs(meas - self.q2)
            if np.all(err <= self.tol):
                self.traj_active = False
                self.waiting_ack = False
                self.first_step_sent = False
                self.get_logger().info(f"🏁 Llegó a meta (REAL). err={np.round(err,1)}")

                # Actualizar estado medido en all_joints (1 decimal)
                start = 0 if self.selected_arm == 'A' else 3
                for k in range(3):
                    self.all_joints[start + k] = round(float(meas[k]), 1)
                return

            # Aún en trayecto: liberamos para siguiente paso
            self.waiting_ack = False
            # Actualizamos visualización del estado medido también aquí
            start = 0 if self.selected_arm == 'A' else 3
            for k in range(3):
                self.all_joints[start + k] = round(float(meas[k]), 1)
            self.get_logger().info(f"✅ Feedback REAL ({self.selected_arm}) = {np.round(meas,1)} → continúa interp.")

    # ================== Timer principal ==================
    def timer_cb(self):
        if not self.traj_active:
            return

        if self.i > self.step:
            self.traj_active = False
            self.waiting_ack = False
            self.first_step_sent = False
            self.get_logger().info("🏁 Interpolación finalizada (por pasos agotados)")
            return

        # Interpolación
        s = self.i / float(self.step)
        q = (1.0 - s) * self.q1 + s * self.q2

        # === Redondeo uniforme (1 decimal) ===
        q_rounded = np.round(q, 1)
        out3 = [float(q_rounded[0]), float(q_rounded[1]), float(q_rounded[2])]

        if self.selected_mode == 'C':
            # ======== SIMULACIÓN ========
            # En SIM sí reflejamos el "estado actual" con el comando, como antes
            start = 0 if self.selected_arm == 'A' else 3
            for k, v in enumerate(out3):
                self.all_joints[start + k] = v

            # Publicar a /sim_deg
            msg = Float32MultiArray()
            msg.data = self.all_joints
            self.sim_pub.publish(msg)

            if self.selected_arm == 'A':
                self.last_sim_q_cmd_A = q_rounded.copy()
                self._set_sim_curr('A', q_rounded)
            else:
                self.last_sim_q_cmd_B = q_rounded.copy()
                self._set_sim_curr('B', q_rounded)

            self.i += 1
            self.get_logger().info(f"📤 SIM {self.i}/{self.step} q={q_rounded}")

        else:
            # ======== VIDA REAL ========
            # <<< CAMBIO REAL: NO tocamos all_joints (estado medido) aquí.
            # Construimos payload de comandos por separado.
            cmd6 = self._build_cmd_payload(out3, self.selected_arm)
            msg = Float32MultiArray()
            msg.data = cmd6

            if not self.first_step_sent:
                self.cmd_pub.publish(msg)
                self.first_step_sent = True
                self.waiting_ack = True
                self.i += 1

                # Guardar último comando REAL
                if self.selected_arm == 'A':
                    self.last_real_q_cmd_A = q_rounded.copy()
                else:
                    self.last_real_q_cmd_B = q_rounded.copy()

                self.get_logger().info(f"🚀 REAL 1/{self.step} q={q_rounded} (cmd enviado, esperando feedback)")
                return

            if self.waiting_ack:
                # Aún esperando feedback del paso anterior
                return

            # Siguiente paso
            self.cmd_pub.publish(msg)
            self.i += 1
            self.waiting_ack = True

            # Guardar último comando REAL
            if self.selected_arm == 'A':
                self.last_real_q_cmd_A = q_rounded.copy()
            else:
                self.last_real_q_cmd_B = q_rounded.copy()

            self.get_logger().info(f"📤 REAL {self.i}/{self.step} q={q_rounded} → esperando feedback")

def main(args=None):
    rclpy.init(args=args)
    node = FKManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

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

        # AHORA /arm_feedback ES Float32MultiArray (3 medidos del encoder)
        self.sub_feedback = self.create_subscription(Float32MultiArray, '/arm_feedback', self.on_feedback, 10)

        # ================== Publicadores ==================
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/cmd_deg', 10)   # vida real
        self.sim_pub = self.create_publisher(Float32MultiArray, '/sim_deg', 10)

        # ============ Estado general para publicar =========
        self.all_joints = [90.0, 90.0, 0.0, 90.0, 90.0, 0.0]

        # ================== Selecciones ====================
        self.selected_arm = 'A'     # 'A' = derecho, 'B' = izquierdo
        self.selected_mode = 'C'    # 'C' = simulación, 'R' = vida real

        # ================== Trayectoria ====================
        self.traj_active = False
        self.i = 0
        self.step = 20  # pasos de interpolación (ajustable)

        # Estados en espacio q (no servo)
        self.q1 = np.array([90.0, 90.0, 0.0], dtype=float)
        self.q2 = np.array([0.0, 0.0, 0.0], dtype=float)

        # ======== ESTADOS SEPARADOS PARA SIM Y REAL =========
        self.q_curr_sim_A = np.array([90.0, 90.0, 0.0], dtype=float)
        self.q_curr_sim_B = np.array([90.0, 90.0, 0.0], dtype=float)
        self.last_sim_q_cmd_A = None
        self.last_sim_q_cmd_B = None

        # Vida real: se actualiza con FEEDBACK (medidos)
        self.q_curr_real_A = np.array([90.0, 90.0, 0.0], dtype=float)
        self.q_curr_real_B = np.array([90.0, 90.0, 0.0], dtype=float)

        # Control de feedback (solo real)
        self.waiting_ack = False
        self.first_step_sent = False  # <<< NUEVO: primer paso se envía sin esperar feedback

        # Tolerancia para considerar que alcanzó (deg) — alinea con Arduino
        self.tol = np.array([0.5, 0.5, 0.5], dtype=float)

        # Timer principal (50 Hz aprox)
        self.timer = self.create_timer(0.02, self.timer_cb)

    # ================== Utilidades ==================
    def _get_curr_for_mode(self, arm: str) -> np.ndarray:
        if self.selected_mode == 'C':  # simulación
            return (self.q_curr_sim_A if arm == 'A' else self.q_curr_sim_B).copy()
        else:  # vida real
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
            # En real, mantenemos el estado actual (no forzamos a q_at)
            self._set_real_curr(arm_running, self._get_curr_for_mode(arm_running))

        self.traj_active = False
        self.waiting_ack = False
        self.first_step_sent = False
        self.get_logger().info(f"⏹️ Trayectoria abortada ({reason}). Estado {'SIM' if mode_running=='C' else 'REAL'} del brazo {arm_running} actualizado.")

    # ================== Callbacks ==================
    def goal_callback(self, msg: Float32MultiArray):
        """Recibe meta en q (3 o 6 valores). Activa interpolación desde q_actual del brazo y modo seleccionados."""
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
        # Claves para el primer paso sin esperar feedback:
        self.waiting_ack = False
        self.first_step_sent = False  # <<< se rearmó para esta nueva trayectoria

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
        """
        Recibe MEDIDOS del Arduino (3 floats) vía /arm_feedback.
        - Aplica a brazo actualmente seleccionado (asunción: un brazo a la vez).
        - Actualiza estado REAL con esos medidos.
        - Libera el bloqueo de 'waiting_ack' para avanzar al siguiente paso.
        - Si está dentro de tolerancia contra q2, finaliza la trayectoria.
        """
        vals = list(msg.data or [])
        if len(vals) != 3:
            self.get_logger().warn(f"⚠️ /arm_feedback inválido: {vals}")
            return

        meas = np.array(vals, dtype=float)

        # Actualiza estado REAL del brazo activo con lo MEDIDO
        self._set_real_curr(self.selected_arm, meas)

        if self.selected_mode == 'R' and self.traj_active:
            # ¿Alcanzó meta (q2) dentro de tolerancia?
            err = np.abs(meas - self.q2)
            if np.all(err <= self.tol):
                self.traj_active = False
                self.waiting_ack = False
                self.first_step_sent = False
                self.get_logger().info(f"🏁 Llegó a meta (REAL). err={err.round(2)}")
                # Actualiza all_joints para reflejar lo real medido
                start = 0 if self.selected_arm == 'A' else 3
                for k in range(3):
                    self.all_joints[start + k] = round(float(meas[k]), 1)

                return

            # Si no alcanzó todavía, liberar el ACK para que el timer publique el siguiente punto
            self.waiting_ack = False
            self.get_logger().info(f"✅ Feedback REAL ({self.selected_arm}) = {meas.round(1)} → continúa interp.")

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

        s = self.i / float(self.step)
        q = (1.0 - s) * self.q1 + s * self.q2

        # === aquí limitamos a un decimal ===
        out3 = [round(float(q[0]), 1),
                round(float(q[1]), 1),
                round(float(q[2]), 1)]

        start = 0 if self.selected_arm == 'A' else 3
        for k, v in enumerate(out3):
            self.all_joints[start + k] = v

        msg = Float32MultiArray()
        msg.data = self.all_joints

        if self.selected_mode == 'C':
            # ======== SIMULACIÓN ========
            self.sim_pub.publish(msg)
            if self.selected_arm == 'A':
                self.last_sim_q_cmd_A = q.copy()
            else:
                self.last_sim_q_cmd_B = q.copy()
            self._set_sim_curr(self.selected_arm, q)

            self.i += 1
            self.get_logger().info(f"📤 SIM {self.i}/{self.step}")

        else:
            # ======== VIDA REAL ========
            # Enviar SIEMPRE el primer paso sin esperar feedback
            if not self.first_step_sent:
                self.cmd_pub.publish(msg)
                self.first_step_sent = True
                self.waiting_ack = True
                self.i += 1
                self.get_logger().info(f"🚀 REAL 1/{self.step} (primer paso enviado SIN esperar feedback)")
                return

            # A partir del segundo paso, sí respetar el feedback
            if self.waiting_ack:
                return  # esperando feedback de Arduino

            # Publica el siguiente punto (se lo enviará el serial_node al Arduino)
            self.cmd_pub.publish(msg)

            # Ahora esperamos feedback con medidos antes de continuar
            self.i += 1
            self.waiting_ack = True
            self.get_logger().info(f"📤 REAL {self.i}/{self.step} → esperando feedback (medidos)")

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

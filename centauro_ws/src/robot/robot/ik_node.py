#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String

class IKManager(Node):
    def __init__(self):
        super().__init__('ik_node')

        # Suscriptores
        self.goal_sub = self.create_subscription(PoseStamped, 'ik_goal', self.goal_callback, 10)
        self.sub_arm_select = self.create_subscription(String,'/cmd_arm_select',self.on_arm_select ,10)
        self.sub_run_mode = self.create_subscription(String,'/run_mode',self.run_mode ,10)
        # /arm_feedback ahora es Float32MultiArray (3 medidos)
        self.sub_feedback = self.create_subscription(Float32MultiArray, '/arm_feedback', self.on_feedback, 10)

        self.cmd_pub  = self.create_publisher(Float32MultiArray, '/cmd_deg', 10)
        self.sim_pub  = self.create_publisher(Float32MultiArray, '/sim_deg', 10)
        
        # Estado general de publicación (6 juntas)
        self.all_joints = [90.0, 90.0, 0.0, 90.0, 90.0, 0.0]

        # Selecciones
        self.selected_arm = 'A'   # 'A' o 'B'
        self.selected_mode = 'C'  # 'C' (sim) o 'D' (real)

        # Interpolación
        self.traj_active = False
        self.i = 0
        self.step = 100

        # Estados en q (no servo)
        self.q1 = np.array([90.0, 90.0, 0.0], dtype=float)
        self.q2 = np.array([0.0, 0.0, 0.0], dtype=float)

        # ===== Estados separados SIM / REAL =====
        # Simulación
        self.q_curr_sim_A = np.array([90.0, 90.0, 0.0], dtype=float)
        self.q_curr_sim_B = np.array([90.0, 90.0, 0.0], dtype=float)
        # NUEVO: buffers del último comando sim
        self.last_sim_q_cmd_A = None
        self.last_sim_q_cmd_B = None

        # Vida real (se actualiza con FEEDBACK medido)
        self.q_curr_real_A = np.array([90.0, 90.0, 0.0], dtype=float)
        self.q_curr_real_B = np.array([90.0, 90.0, 0.0], dtype=float)

        # Control de feedback
        self.waiting_ack = False

        # Tolerancia para considerar que alcanzó meta (alineada con Arduino)
        self.tol = np.array([1.5, 1.5, 1.5], dtype=float)

        self.timer = self.create_timer(0.02, self.timer_cb)

    # ========= Utilidades de estado =========
    def _get_curr_for_mode(self, arm: str) -> np.ndarray:
        """Devuelve q_actual del brazo según el modo activo."""
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

    def _current_q_along_traj(self) -> np.ndarray | None:
        """Punto actual de la interpolación (no altera tu lógica)."""
        if not self.traj_active:
            return None
        s = self.i / float(self.step)
        return (1.0 - s) * self.q1 + s * self.q2

    def _abort_current(self, arm_running: str, mode_running: str, reason: str):
        """Detiene la trayectoria, fija estado y limpia flags."""
        q_at = self._current_q_along_traj()
        if q_at is None:
            return

        if mode_running == 'C':
            # En sim: fijar estado sim al punto alcanzado (igual que antes)
            if arm_running == 'A':
                self.q_curr_sim_A = q_at.copy()
            else:
                self.q_curr_sim_B = q_at.copy()
        else:
            # En real: mantenemos el estado real actual (lo fija el feedback)
            self._set_real_curr(arm_running, self._get_curr_for_mode(arm_running))

        self.traj_active = False
        self.waiting_ack = False
        self.get_logger().info(
            f"⏹️ Trayectoria abortada por {reason}. Estado "
            f"{'SIM' if mode_running=='C' else 'REAL'} del brazo {arm_running} actualizado."
        )

    # ================ Callbacks ================
    def goal_callback(self, msg : PoseStamped):
        p = msg.pose.position

        # Trigger #1: (0,0,0) → q2 = [180, 180, 90]
        if np.isclose(p.x, 0.0, atol=1e-6) and np.isclose(p.y, 0.0, atol=1e-6) and np.isclose(p.z, 0.0, atol=1e-6):
            self.q1 = self._get_curr_for_mode(self.selected_arm)
            self.q2 = np.array([180.0,180.0,90.0], dtype=float)

            if np.allclose(self.q1, self.q2, atol=1e-3):
                self.get_logger().info("🎯 q_actual == q2; se cancela el arranque de la trayectoria.")
                return

            self.i = 0
            self.traj_active = True
            self.waiting_ack = False
            self.get_logger().info("🎯 Trigger (0,0,0): interpolación q1→q2")

        # Trigger #2: (1,1,1) → q2 = [120, 120, 50]
        elif np.isclose(p.x, 1.0, atol=1e-6) and np.isclose(p.y, 1.0, atol=1e-6) and np.isclose(p.z, 1.0, atol=1e-6):
            self.q1 = self._get_curr_for_mode(self.selected_arm)
            self.q2 = np.array([120.0,120.0,50.0], dtype=float)

            if np.allclose(self.q1, self.q2, atol=1e-3):
                self.get_logger().info("🎯 q_actual == q2; se cancela el arranque de la trayectoria.")
                return

            self.i = 0
            self.traj_active = True
            self.waiting_ack = False
            self.get_logger().info("🎯 Trigger (1,1,1): interpolación q1→q2")

        # Trigger #3: (3,3,3) → q2 = [90, 90, 0]
        elif np.isclose(p.x, 3.0, atol=1e-6) and np.isclose(p.y, 3.0, atol=1e-6) and np.isclose(p.z, 3.0, atol=1e-6):
            self.q1 = self._get_curr_for_mode(self.selected_arm)
            self.q2 = np.array([90.0,90.0,0.0], dtype=float)

            if np.allclose(self.q1, self.q2, atol=1e-3):
                self.get_logger().info("🎯 q_actual == q2; se cancela el arranque de la trayectoria.")
                return

            self.i = 0
            self.traj_active = True
            self.waiting_ack = False
            self.get_logger().info("🎯 Trigger (3,3,3): interpolación q1→q2")

        else:
            return

    def on_arm_select(self, msg: String):
        new_arm = (msg.data or '').strip().upper()
        if new_arm not in ('A', 'B'):
            self.get_logger().warn(f"Brazo inválido: '{msg.data}' (usa 'A' o 'B')")
            return

        # Parada segura si cambia el brazo durante trayectoria
        if self.traj_active and new_arm != self.selected_arm:
            prev_arm = self.selected_arm
            prev_mode = self.selected_mode
            self._abort_current(prev_arm, prev_mode, "cambio de brazo")

        self.selected_arm = new_arm
        self.get_logger().info(f"🔀 Brazo seleccionado: {self.selected_arm}")

    def run_mode(self, msg: String):
        val = (msg.data or '').strip().lower()
        new_mode = 'C' if val in ('c', 'sim', 'simulacion', 'simulation') else 'D'

        # Parada segura si cambia el modo durante trayectoria
        if self.traj_active and new_mode != self.selected_mode:
            prev_arm = self.selected_arm
            prev_mode = self.selected_mode
            self._abort_current(prev_arm, prev_mode, "cambio de modo")

        self.selected_mode = new_mode
        self.get_logger().info("🧪 Modo: C (simulación)" if new_mode=='C' else "🔧 Modo: D (vida real)")

    def on_feedback(self, msg: Float32MultiArray):
        """
        Recibe MEDIDOS del Arduino (3 floats) vía /arm_feedback.
        - Aplica al brazo actualmente seleccionado (asunción: un brazo a la vez).
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

        if self.selected_mode == 'D' and self.traj_active:
            err = np.abs(meas - self.q2)
            if np.all(err <= self.tol):
                self.traj_active = False
                self.waiting_ack = False
                self.get_logger().info(f"🏁 Llegó a meta (REAL). err={err.round(2)}")
                start = 0 if self.selected_arm == 'A' else 3
                for k in range(3):
                    self.all_joints[start + k] = float(meas[k])
                return

            self.waiting_ack = False
            self.get_logger().info(f"✅ Feedback REAL ({self.selected_arm}) = {meas.round(1)} → continúa interp.")

    # ============== Timer principal ==============
    def timer_cb(self):
        if not self.traj_active:
            return
        if self.i > self.step:
            self.traj_active = False
            self.get_logger().info("🏁 Interpolación finalizada")
            return

        # Calcular siguiente punto (NO se modifica tu lógica)
        s = self.i / float(self.step)
        q = (1.0 - s) * self.q1 + s * self.q2

        if self.selected_arm == 'A':
            out3 = [q[0], 180 - q[1], q[2]]
            start = 0
        else:
            out3 = [180 - q[0], 180 - q[1], q[2]]
            start = 3

        for idx, val in enumerate(out3):
            self.all_joints[start + idx] = val

        msg = Float32MultiArray()
        msg.data = self.all_joints

        if self.selected_mode == 'C':
            # ======== SIMULACIÓN ========
            self.sim_pub.publish(msg)
            # NUEVO: guarda el último comando sim
            if self.selected_arm == 'A':
                self.last_sim_q_cmd_A = q.copy()
            else:
                self.last_sim_q_cmd_B = q.copy()
            # y sigue actualizando el estado sim como antes
            self._set_sim_curr(self.selected_arm, q)

            self.i += 1
            self.get_logger().info(f"📤 Simulación punto {self.i}/{self.step}")

        else:
            # ======== VIDA REAL ========
            if self.waiting_ack:
                return   # esperando feedback
            self.cmd_pub.publish(msg)
            self.i += 1
            self.waiting_ack = True
            self.get_logger().info(f"📤 Real punto {self.i}/{self.step} → esperando feedback (medidos)")

def main(args=None):
    rclpy.init(args=args)
    node = IKManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

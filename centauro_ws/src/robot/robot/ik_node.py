#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from scipy.optimize import least_squares
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String

class IKManager(Node):
    def __init__(self):
        super().__init__('ik_node')

        # ========= Suscriptores =========
        self.goal_sub = self.create_subscription(PoseStamped, 'ik_goal', self.goal_callback, 10)
        self.sub_arm_select = self.create_subscription(String,'/cmd_arm_select', self.on_arm_select, 10)
        self.sub_run_mode   = self.create_subscription(String,'/run_mode', self.run_mode, 10)
        self.sub_feedback   = self.create_subscription(Float32MultiArray, '/arm_feedback', self.on_feedback, 10)

        # ========= Publicadores =========
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/cmd_deg', 10)   # vida real
        self.sim_pub = self.create_publisher(Float32MultiArray, '/sim_deg', 10)   # simulación

        # ========= Estado general de salida (6 juntas) =========
        self.all_joints = [90.0, 90.0, 0.0, 90.0, 90.0, 0.0]   # visual/estado
        self.all_cmd    = [90.0, 90.0, 0.0, 90.0, 90.0, 0.0]   # buffer de comando

        # ========= Selecciones =========
        self.selected_arm  = 'A'  # 'A' o 'B'
        self.selected_mode = 'C'  # 'C' (sim) o 'D' (real)

        # ========= Interpolación cartesiana =========
        self.traj_active = False
        self.i = 0
        self.step = 2
        self.dt   = 0.02
        self.P_base = np.array([0.0, 0.0, -0.371], dtype=float)
        self.P_goal = self.P_base.copy()

        # ========= Estados CARTESIANOS separados SIM / REAL =========
        self.p_curr_sim_A  = self.P_base.copy()
        self.p_curr_sim_B  = self.P_base.copy()
        self.p_curr_real_A = self.P_base.copy()
        self.p_curr_real_B = self.P_base.copy()

        # ========= Buffers / tolerancias =========
        self.waiting_ack = False
        self.tol = np.array([0.5, 0.5, 0.5], dtype=float)  # (dejado igual)

        # ========= (C) Semillas IK (en grados) =========
        self.q_seed_A = np.array([0.0, 90.0, -180.0], dtype=float)
        self.q_seed_B = np.array([0.0, 90.0, -180.0], dtype=float)

        # ========= (A) DH por brazo (mm/deg) =========
        # --- AJUSTA AQUI si el brazo B difiere ---
        self._dh_by_arm = {
            'A': [
                ("q2",   -51.0,   25.0,  0.0),  # θ2
                (90.0,     0.0,    0.0, 90.0),  # fijo
                ("q4",   -17.0, -105.0,  0.0),  # θ4
                ("q5",    -7.5,  215.0,  0.0),  # θ5
            ],
            'B': [
                
                ("q2",   -51.0,   -25.0,  0.0),
                (90.0,     0.0,    0.0, 90.0),
                ("q4",   17.0, -105.0,  0.0),
                ("q5",    7.5,  215.0,  0.0),
            ],
        }

        # ========= (B) Límites IK por brazo (en grados) =========
        # --- AJUSTA AQUI si el brazo B difiere ---
        self._limits_by_arm = {
            'A': (np.array([-90.0,  90.0, -180.0], dtype=float),
                  np.array([ 15.0, 180.0,  -90.0], dtype=float)),
            'B': (np.array([-15.0,  90.0, -180.0], dtype=float),
                  np.array([ 90.0, 180.0,  -90.0], dtype=float)),
        }

        # ========= (A+B) Estado activo de DH/Límites =========
        self._dh_params = None
        self.ik_lower_deg = None
        self.ik_upper_deg = None
        self._apply_arm_config(self.selected_arm)  # inicializa DH y límites para 'A'

        # ========= Timer =========
        self.timer = self.create_timer(self.dt, self.timer_cb)

        # ========= Aux =========
        self.last_sent_geom_q  = None
        self.last_sent_cmd_q   = None
        self.last_sim_geom_q_A = None
        self.last_sim_geom_q_B = None

    # ------------------- Utilidades de estado -------------------
    def _get_curr_cart_for_mode(self, arm: str) -> np.ndarray:
        if self.selected_mode == 'C':
            return (self.p_curr_sim_A if arm == 'A' else self.p_curr_sim_B).copy()
        else:
            return (self.p_curr_real_A if arm == 'A' else self.p_curr_real_B).copy()

    def _set_curr_cart_sim(self, arm: str, P: np.ndarray) -> None:
        if arm == 'A': self.p_curr_sim_A = P.copy()
        else:          self.p_curr_sim_B = P.copy()

    def _set_curr_cart_real(self, arm: str, P: np.ndarray) -> None:
        if arm == 'A': self.p_curr_real_A = P.copy()
        else:          self.p_curr_real_B = P.copy()

    def _current_cart_along_traj(self) -> np.ndarray | None:
        if not self.traj_active: return None
        s = self.i / float(self.step)
        return (1.0 - s) * self.P_base + s * self.P_goal

    def _abort_current(self, arm_running: str, mode_running: str, reason: str):
        P_at = self._current_cart_along_traj()
        if P_at is None: return
        if mode_running == 'C': self._set_curr_cart_sim(arm_running, P_at)
        else:                   self._set_curr_cart_real(arm_running, P_at)
        self.traj_active = False
        self.waiting_ack = False
        self.last_sent_geom_q = None
        self.last_sent_cmd_q  = None
        self.get_logger().info(f"⏹️ Trayectoria abortada por {reason}. Estado {'SIM' if mode_running=='C' else 'REAL'} del brazo {arm_running} actualizado.")

    def _build_cmd_payload(self, out3, arm: str):
        arr = self.all_cmd[:]  # copia
        start = 0 if arm == 'A' else 3
        for k, v in enumerate(out3):
            arr[start + k] = float(v)
        return arr

    # ------------------- Conmutación de configuración por brazo -------------------
    def _apply_arm_config(self, arm: str):
        """Aplica DH y límites correspondientes al brazo activo sin cambiar la lógica."""
        # DH
        self._dh_params = list(self._dh_by_arm[arm])  # copia defensiva
        # Límites
        lower, upper = self._limits_by_arm[arm]
        self.ik_lower_deg = lower.astype(float).copy()
        self.ik_upper_deg = upper.astype(float).copy()
        # Log
        self.get_logger().info(
            f"🧩 Config aplicada para brazo {arm}: DH({len(self._dh_params)} filas), "
            f"limits=[{self.ik_lower_deg.tolist()} .. {self.ik_upper_deg.tolist()}]"
        )

    # ------------------- IK NUMÉRICA (least_squares) -------------------
    @staticmethod
    def _A_matrix_deg(theta_deg, d_mm, a_mm, alpha_deg):
        th = math.radians(float(theta_deg))
        al = math.radians(float(alpha_deg))
        a = float(a_mm) / 1000.0
        d = float(d_mm) / 1000.0
        return np.array([
            [ math.cos(th), -math.sin(th)*math.cos(al),  math.sin(th)*math.sin(al), a*math.cos(th)],
            [ math.sin(th),  math.cos(th)*math.cos(al), -math.cos(th)*math.sin(al), a*math.sin(th)],
            [ 0.0,           math.sin(al),               math.cos(al),              d],
            [ 0.0,           0.0,                        0.0,                       1.0]
        ], dtype=float)

    def _fk_pos_from_q(self, q_deg_245: np.ndarray) -> np.ndarray:
        th2, th4, th5 = [float(q_deg_245[0]), float(q_deg_245[1]), float(q_deg_245[2])]
        T = np.eye(4, dtype=float)
        for item in self._dh_params:
            theta, d_mm, a_mm, alpha = item
            if theta == "q2": ang = th2
            elif theta == "q4": ang = th4
            elif theta == "q5": ang = th5
            else: ang = float(theta)
            T = T @ self._A_matrix_deg(ang, d_mm, a_mm, alpha)
        pos = T[:3, 3].copy()  # (x,y,z) en metros
        return pos

    def _ik_residuals(self, q_deg_245: np.ndarray, target_xyz: np.ndarray) -> np.ndarray:
        pos = self._fk_pos_from_q(q_deg_245)
        return pos - target_xyz

    def solve_ik(self, P_xyz_m: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        target = np.array(P_xyz_m, dtype=float)
        seed   = np.array(q_seed_deg, dtype=float)
        lower  = self.ik_lower_deg.astype(float)
        upper  = self.ik_upper_deg.astype(float)

        res = least_squares(
            self._ik_residuals,
            x0=seed,
            args=(target,),
            bounds=(lower, upper),
            xtol=1e-8, ftol=1e-8, gtol=1e-8, max_nfev=2000
        )
        q_sol = res.x.astype(float)
        return q_sol  # grados (sin redondeo aquí)

    # ------------------- Feedback REAL -------------------
    def on_feedback(self, msg: Float32MultiArray):
        vals = list(msg.data or [])
        if len(vals) != 3:
            self.get_logger().warn(f"⚠️ /arm_feedback inválido: {vals}")
            return
        meas = np.array(vals, dtype=float)

        if self.selected_mode == 'D' and self.traj_active:
            if self.last_sent_cmd_q is not None:
                err = np.abs(meas - self.last_sent_cmd_q)
                if np.all(err <= self.tol):
                    self.traj_active = False
                    self.waiting_ack = False
                    self.get_logger().info(
                        f"🏁 Llegó a meta (REAL). err=[{err[0]:.2f}, {err[1]:.2f}, {err[2]:.2f}]"
                    )
                    self._set_curr_cart_real(self.selected_arm, self.P_goal)
                    self.P_base = self.P_goal.copy()
                    if self.last_sent_geom_q is not None:
                        if self.selected_arm == 'A': self.q_seed_A = self.last_sent_geom_q.copy()
                        else:                         self.q_seed_B = self.last_sent_geom_q.copy()
                    self.last_sent_geom_q = None
                    self.last_sent_cmd_q  = None
                    return
                else:
                    self.get_logger().info(
                        f"⏳ REAL fuera de tol (err=[{err[0]:.2f}, {err[1]:.2f}, {err[2]:.2f}]). Esperando..."
                    )
                    return
            else:
                self.waiting_ack = False

            # Aproxima cartesianas mientras avanza
            P_at = self._current_cart_along_traj()
            if P_at is not None:
                self._set_curr_cart_real(self.selected_arm, P_at)

    # ------------------- Timer principal -------------------
    def timer_cb(self):
        if not self.traj_active:
            return

        if self.i > self.step:
            self.traj_active = False
            self.waiting_ack = False
            if self.selected_mode == 'C':
                self._set_curr_cart_sim(self.selected_arm, self.P_goal)
            else:
                self._set_curr_cart_real(self.selected_arm, self.P_goal)
            self.P_base = self.P_goal.copy()

            last_geom = (self.last_sim_geom_q_A if self.selected_arm == 'A' else self.last_sim_geom_q_B) \
                        if self.selected_mode == 'C' else self.last_sent_geom_q
            if last_geom is not None:
                if self.selected_arm == 'A': self.q_seed_A = last_geom.copy()
                else:                         self.q_seed_B = last_geom.copy()

            self.last_sent_geom_q = None
            self.last_sent_cmd_q  = None
            self.get_logger().info("🏁 Interpolación cartesiana finalizada (estado y base fijados al objetivo)")
            return

        # Punto cartesiano interpolado
        s = self.i / float(self.step)
        P = (1.0 - s) * self.P_base + s * self.P_goal

        # Semilla por brazo
        q_seed = self.q_seed_A if self.selected_arm == 'A' else self.q_seed_B

        # ======= IK directa (SIN MAPEOS) =======
        q_geom = self.solve_ik(P, q_seed)      # [θ2, θ4, θ5] en grados
        q_geom = np.round(q_geom, 2)

        # Actualiza semilla con geom
        if self.selected_arm == 'A': self.q_seed_A = q_geom.copy()
        else:                         self.q_seed_B = q_geom.copy()

        # ======= PUBLICACIÓN CON MAPEO =======
        out3 = q_geom.copy()
        if self.selected_mode == 'C' :
            if self.selected_arm == 'A':
                out3 = [90-out3[0], 180 - out3[1], 180 + out3[2]]
            else:
                out3 = [90 - out3[0], 180 - out3[1], 180 + out3[2]]
        else:
            if self.selected_arm == 'A':
                out3 = [90 + out3[0], 180 - out3[1], 270 + out3[2]]
            else:
                out3 = [out3[0], 180 - out3[1], out3[2]]  # θ2

        start = 0 if self.selected_arm == 'A' else 3

        if self.selected_mode == 'C':
            # ======== SIM ========
            for k in range(3):
                self.all_joints[start + k] = float(out3[k])
            msg = Float32MultiArray(); msg.data = self.all_joints
            self.sim_pub.publish(msg)
            if self.selected_arm == 'A': self.last_sim_geom_q_A = q_geom.copy()
            else:                        self.last_sim_geom_q_B = q_geom.copy()
            self._set_curr_cart_sim(self.selected_arm, P)
            self.i += 1
            self.get_logger().info(
                f"📤 SIM {self.i}/{self.step}\n"
                f"P=[{P[0]:.3f}, {P[1]:.3f}, {P[2]:.3f}]\n"
                f"q_deg=[{q_geom[0]:.2f}, {q_geom[1]:.2f}, {q_geom[2]:.2f}]\n"
                f"q_sim=[{out3[0]:.2f}, {out3[1]:.2f}, {out3[2]:.2f}]"
            )
        else:
            # ======== REAL ========
            if self.waiting_ack:
                return

            cmd6 = self._build_cmd_payload(out3, self.selected_arm)
            msg = Float32MultiArray(); msg.data = cmd6
            self.cmd_pub.publish(msg)

            self.last_sent_geom_q = q_geom.copy()
            self.last_sent_cmd_q  = np.array(out3, dtype=float).copy()
            self.waiting_ack = True
            self.i += 1
            self.get_logger().info(
                f"📤 REAL {self.i}/{self.step}  P=[{P[0]:.3f}, {P[1]:.3f}, {P[2]:.3f}]  q_deg=[{q_geom[0]:.2f}, {q_geom[1]:.2f}, {q_geom[2]:.2f}] → ACK"
            )

    # ------------------- Callbacks -------------------
    def goal_callback(self, msg: PoseStamped):
        p = msg.pose.position
        goal = np.array([float(p.x), float(p.y), float(p.z)], dtype=float)

        curr = self._get_curr_cart_for_mode(self.selected_arm)

        # <<< ABORT: si llega nuevo objetivo y hay trayectoria, aborta la actual
        if self.traj_active:
            self._abort_current(self.selected_arm, self.selected_mode, "nuevo objetivo")

        if np.linalg.norm(goal - curr) <= 1e-4:
            self.traj_active = False
            self.waiting_ack = False
            self.get_logger().info(
                f"✔️ Ya está en la misma posición [{goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}] — no se envía nada."
            )
            return

        self.P_base = curr
        self.P_goal = goal
        self.i = 0
        self.traj_active = True
        self.waiting_ack = False
        self.get_logger().info(
            f"🎯 Nueva meta cartesiana: [{self.P_goal[0]:.3f}, {self.P_goal[1]:.3f}, {self.P_goal[2]:.3f}] "
            f"desde [{self.P_base[0]:.3f}, {self.P_base[1]:.3f}, {self.P_base[2]:.3f}]"
        )

    def on_arm_select(self, msg: String):
        val = (msg.data or '').strip().upper()
        if val in ('A', 'B'):
            # <<< ABORT: si cambias de brazo con trayectoria activa
            if self.traj_active and val != self.selected_arm:
                self._abort_current(self.selected_arm, self.selected_mode, "cambio de brazo")
            self.selected_arm = val
            # ← AQUI aplicamos DH y límites específicos del brazo seleccionado
            self._apply_arm_config(self.selected_arm)



            self.get_logger().info(f"🅰️/🅱️ Brazo seleccionado: {self.selected_arm}")

    def run_mode(self, msg: String):
        val = (msg.data or '').strip().upper()
        if val in ('C', 'D'):
            # <<< ABORT: si cambias de modo con trayectoria activa
            if self.traj_active and val != self.selected_mode:
                self._abort_current(self.selected_arm, self.selected_mode, "cambio de modo")
            self.selected_mode = val
            self.get_logger().info(f"⚙️ Modo seleccionado: {'SIM' if val=='C' else 'REAL'}")

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

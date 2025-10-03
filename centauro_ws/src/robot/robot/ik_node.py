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
        self.all_joints = [90.0, 90.0, 0.0, 90.0, 90.0, 0.0]

        # ========= Selecciones =========
        self.selected_arm  = 'A'  # 'A' o 'B'
        self.selected_mode = 'C'  # 'C' (sim) o 'D' (real)

        # ========= Interpolación cartesiana =========
        self.traj_active = False
        self.i = 0
        self.step = 100
        self.dt   = 0.02
        self.P_base = np.array([0.0, 0.0, 0.0], dtype=float)
        self.P_goal = self.P_base.copy()

        # ========= Estados CARTESIANOS separados SIM / REAL =========
        self.p_curr_sim_A  = self.P_base.copy()
        self.p_curr_sim_B  = self.P_base.copy()
        self.p_curr_real_A = self.P_base.copy()
        self.p_curr_real_B = self.P_base.copy()

        # ========= Buffers / tolerancias =========
        self.waiting_ack = False
        self.tol = np.array([1.5, 1.5, 1.5], dtype=float)
        self.last_sent_geom_q = None   # último q geom [θ2,θ4,θ5] que resolvió IK
        self.last_sent_cmd_q  = None   # último q publicado a tópicos (mapeado a pinterfaz)
        self.last_sim_geom_q_A = None
        self.last_sim_geom_q_B = None

        # ========= Semillas IK (en grados, geométricos) =========
        self.q_seed_A = np.array([90.0, 90.0, 0.0], dtype=float)
        self.q_seed_B = np.array([90.0, 90.0, 0.0], dtype=float)

        # ========= Timer =========
        self.timer = self.create_timer(self.dt, self.timer_cb)

        # === Parámetros geom (si los usas en otra parte) ===
        self.L1 = 0.260
        self.L2 = 0.107
        self.L3 = 0.950

        # Visualizador con X/Y permutados
        self.swap_xy = True

        # ====== Offsets EXTRA (por si luego afinas a mano). Por defecto 0 ======
        self.extra_off_sim_A = np.array([0.0, 0.0, 0.0], dtype=float)
        self.extra_off_sim_B = np.array([0.0, 0.0, 0.0], dtype=float)
        self.extra_off_cmd_A = np.array([0.0, 0.0, 0.0], dtype=float)
        self.extra_off_cmd_B = np.array([0.0, 0.0, 0.0], dtype=float)

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

    # ------------------- Mapeo geom -> pinterfaz (con fórmulas exactas) -------------------
    def _map_to_topic(self, arm: str, mode: str, q_geom_deg: np.ndarray) -> np.ndarray:
        """
        q_geom_deg = [θ2, θ4, θ5] (grados) desde IK.
        Devuelve el vector de 3 que publica a /sim_deg o /cmd_deg, usando el mismo convenio que pinterfaz.
        Fórmulas derivadas de tu GUI:
          A: [θ2 - 90,  180 - θ4,  θ5 - 180]
          B: [ 90 - θ2,    - θ4,     θ5     ]
        + offsets extra opcionales (por modo).
        """
        q = np.asarray(q_geom_deg, dtype=float)
        if arm == 'A':
            base = np.array([q[0] - 90.0,  q[1], q[2] + 180.0], dtype=float)
            offs = self.extra_off_sim_A if mode == 'C' else self.extra_off_cmd_A
        else:
            base = np.array([90.0 - q[0], -q[1], q[2]], dtype=float)
            offs = self.extra_off_sim_B if mode == 'C' else self.extra_off_cmd_B
        return base + offs

    # ------------------- IK por DH (θ2, θ4, θ5) -------------------
    def solve_ik(self, P_des: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        def A_matrix_deg(theta_deg, d_mm, a_mm, alpha_deg):
            th = math.radians(float(theta_deg))
            al = math.radians(float(alpha_deg))
            a  = float(a_mm) / 1000.0
            d  = float(d_mm) / 1000.0
            return np.array([
                [ math.cos(th), -math.sin(th)*math.cos(al),  math.sin(th)*math.sin(al), a*math.cos(th)],
                [ math.sin(th),  math.cos(th)*math.cos(al), -math.cos(th)*math.sin(al), a*math.sin(th)],
                [ 0.0,           math.sin(al),               math.cos(al),              d],
                [ 0.0,           0.0,                        0.0,                       1.0]
            ], dtype=float)

        def fk_pos(theta2_deg, theta4_deg, theta5_deg, do_swap_xy: bool):
            T = A_matrix_deg(theta2_deg, -51.0, 25.0, 0.0)
            for (th, d, a, al) in [
                (90.0,        0.0,    0.0,   90.0),   # θ3 fijo
                (theta4_deg, -17.0, -105.0,   0.0),   # θ4
                (theta5_deg,  -7.5,  215.0,   0.0)    # θ5
            ]:
                T = T @ A_matrix_deg(th, d, a, al)
            pos = T[:3, 3].astype(float)
            if do_swap_xy:
                pos = np.array([pos[1], pos[0], pos[2]], dtype=float)
            return pos

        def residuals(q, target_xyz, do_swap_xy):
            pos = fk_pos(q[0], q[1], q[2], do_swap_xy)
            return pos - target_xyz

        target_xyz = np.array([float(P_des[0]), float(P_des[1]), float(P_des[2])], dtype=float)
        seed = np.asarray(q_seed_deg, dtype=float)
        seed = ((seed + 180.0) % 360.0) - 180.0
        bounds = (np.array([-180.0, -180.0, -180.0]), np.array([180.0, 180.0, 180.0]))

        res = least_squares(residuals, x0=seed, args=(target_xyz, bool(self.swap_xy)),
                            bounds=bounds, xtol=1e-8, ftol=1e-8, gtol=1e-8, max_nfev=2000)
        if not res.success:
            self.get_logger().warn("⚠️ IK (DH/LSQ) no reportó éxito; usando mejor solución.")
        return np.array(res.x, dtype=float)  # [θ2, θ4, θ5]

    # ------------------- Callbacks -------------------
    def goal_callback(self, msg: PoseStamped):
        p = msg.pose.position
        self.P_base = self._get_curr_cart_for_mode(self.selected_arm)  # partir del punto actual
        self.P_goal = np.array([float(p.x), float(p.y), float(p.z)], dtype=float)

        if np.allclose(self.P_base, self.P_goal, atol=1e-6):
            self.get_logger().info("🎯 P_base == P_goal; no se inicia trayectoria.")
            return

        self.i = 0
        self.traj_active = True
        self.waiting_ack = False
        self.last_sent_geom_q = None
        self.last_sent_cmd_q  = None
        self.get_logger().info(f"🎯 Recta cartesiana: P_base {self.P_base.round(3)} → P_goal {self.P_goal.round(3)}")

    def on_arm_select(self, msg: String):
        new_arm = (msg.data or '').strip().upper()
        if new_arm not in ('A', 'B'):
            self.get_logger().warn(f"Brazo inválido: '{msg.data}' (usa 'A' o 'B')")
            return
        if self.traj_active and new_arm != self.selected_arm:
            self._abort_current(self.selected_arm, self.selected_mode, "cambio de brazo")
        self.selected_arm = new_arm
        self.get_logger().info(f"🔀 Brazo seleccionado: {self.selected_arm}")

    def run_mode(self, msg: String):
        val = (msg.data or '').strip().lower()
        new_mode = 'C' if val in ('c', 'sim', 'simulacion', 'simulation') else 'D'
        if self.traj_active and new_mode != self.selected_mode:
            self._abort_current(self.selected_arm, self.selected_mode, "cambio de modo")
        self.selected_mode = new_mode
        self.get_logger().info("🧪 Modo: C (simulación)" if new_mode=='C' else "🔧 Modo: D (vida real)")

    def on_feedback(self, msg: Float32MultiArray):
        vals = list(msg.data or [])
        if len(vals) != 3:
            self.get_logger().warn(f"⚠️ /arm_feedback inválido: {vals}")
            return
        meas = np.array(vals, dtype=float)

        if self.selected_mode == 'D' and self.traj_active:
            if self.last_sent_cmd_q is not None:
                err = np.abs(meas - self.last_sent_cmd_q)   # comparar contra lo realmente publicado
                if np.all(err <= self.tol):
                    # Llegó a meta en REAL
                    self.traj_active = False
                    self.waiting_ack = False
                    self.get_logger().info(f"🏁 Llegó a meta (REAL). err={err.round(2)}")

                    # Fija cartesianas/base y deja semilla con el último geom usado
                    self._set_curr_cart_real(self.selected_arm, self.P_goal)
                    self.P_base = self.P_goal.copy()
                    if self.last_sent_geom_q is not None:
                        if self.selected_arm == 'A': self.q_seed_A = self.last_sent_geom_q.copy()
                        else:                         self.q_seed_B = self.last_sent_geom_q.copy()
                    self.last_sent_geom_q = None
                    self.last_sent_cmd_q  = None
                    return
                else:
                    self.get_logger().info(f"⏳ REAL fuera de tol (err={err.round(2)}). Esperando...")
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
            # fija estado y base al objetivo
            if self.selected_mode == 'C':
                self._set_curr_cart_sim(self.selected_arm, self.P_goal)
            else:
                self._set_curr_cart_real(self.selected_arm, self.P_goal)
            self.P_base = self.P_goal.copy()

            # mantiene rama con última geom
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

        # Resolver IK (geométrico)
        q_geom = self.solve_ik(P, q_seed)  # [θ2, θ4, θ5]
        # Actualiza semilla con geom
        if self.selected_arm == 'A': self.q_seed_A = q_geom.copy()
        else:                         self.q_seed_B = q_geom.copy()

        # ======= Mapeo EXACTO a tus tópicos (pinterfaz) =======
        out3 = self._map_to_topic(self.selected_arm, self.selected_mode, q_geom)

        # Escribe en el vector de 6 y publica
        start = 0 if self.selected_arm == 'A' else 3
        for k in range(3):
            self.all_joints[start + k] = float(out3[k])

        msg = Float32MultiArray(); msg.data = self.all_joints

        if self.selected_mode == 'C':
            # SIM
            self.sim_pub.publish(msg)
            if self.selected_arm == 'A': self.last_sim_geom_q_A = q_geom.copy()
            else:                        self.last_sim_geom_q_B = q_geom.copy()
            self._set_curr_cart_sim(self.selected_arm, P)
            self.i += 1
            self.get_logger().info(f"📤 SIM {self.i}/{self.step}  P={P.round(2)}  q_geom={q_geom.round(1)}  out={out3.round(1)}")
        else:
            # REAL
            if self.waiting_ack:
                return
            self.cmd_pub.publish(msg)
            self.last_sent_geom_q = q_geom.copy()   # para continuidad de rama
            self.last_sent_cmd_q  = out3.copy()     # para comparar feedback (en el mismo espacio del pinterfaz)
            self.waiting_ack = True
            self.i += 1
            self.get_logger().info(f"📤 REAL {self.i}/{self.step}  P={P.round(2)}  q_geom={q_geom.round(1)}  cmd={out3.round(1)} → ACK")

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

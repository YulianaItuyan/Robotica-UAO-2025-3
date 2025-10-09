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
        self.goal_sub       = self.create_subscription(PoseStamped, 'ik_goal', self.goal_callback, 10)
        self.sub_arm_select = self.create_subscription(String,'/cmd_arm_select', self.on_arm_select, 10)
        self.sub_run_mode   = self.create_subscription(String,'/run_mode', self.run_mode, 10)
        self.sub_feedback   = self.create_subscription(Float32MultiArray, '/arm_feedback', self.on_feedback, 10)
        self.sub_ik_mode    = self.create_subscription(String,'/ik_mode', self.on_ik_mode, 10)

        # ========= Publicadores =========
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/cmd_deg', 10)   # vida real
        self.sim_pub = self.create_publisher(Float32MultiArray, '/sim_deg', 10)   # simulación
        # === NUEVO: publicador único para grados puros y con offset (mismo tópico) ===
        # data = [q2_puro, q4_puro, q5_puro, q2_offset, q4_offset, q5_offset]
        self.ik_pub = self.create_publisher(Float32MultiArray, '/ik_deg', 10)

        # ========= Estado general (medido vs comando) =========
        self.all_joints = [90.0, 90.0, 90.0, 90.0, 90.0, 0.0]   # ESTADO MEDIDO (solo feedback o SIM)
        self.all_cmd    = [90.0, 90.0, 90.0, 90.0, 90.0, 0.0]   # BUFFER COMANDO

        # ========= Selecciones =========
        self.selected_arm  = 'A'  # 'A' o 'B'
        self.selected_mode = 'C'  # 'C' (sim) o 'D' (real)

        # ========= Interpolación cartesiana =========
        self.traj_active = False
        self.i    = 0
        self.step = 5
        self.dt   = 0.02
        self.P_base = np.array([0.0, 0.0, -0.371], dtype=float)
        self.P_goal = self.P_base.copy()

        # ========= Estados cartesianos (SIM/REAL) =========
        self.p_curr_sim_A  = self.P_base.copy()
        self.p_curr_sim_B  = self.P_base.copy()
        self.p_curr_real_A = self.P_base.copy()
        self.p_curr_real_B = self.P_base.copy()

        # ========= Buffers / tolerancias =========
        self.waiting_ack = False
        self.tol = np.array([0.5, 0.5, 0.5], dtype=float)

        # ========= Semillas IK (deg) =========
        self.q_seed_A = np.array([0.0, 90.0, -180.0], dtype=float)
        self.q_seed_B = np.array([0.0, 90.0, -180.0], dtype=float)

        # ========= DH por brazo =========
        self._dh_by_arm = {
            'A': [
                ("q2",   -51.0,   25.0,  0.0),
                (90.0,     0.0,    0.0, 90.0),
                ("q4",   -17.0, -105.0,  0.0),
                ("q5",    -7.5,  215.0,  0.0),
            ],
            'B': [
                ("q2",   -51.0,   -25.0,  0.0),
                (90.0,     0.0,     0.0, 90.0),
                ("q4",    17.0,  -105.0,  0.0),
                ("q5",     7.5,   215.0,  0.0),
            ],
        }

        # ========= Límites por brazo (deg) =========
        self._limits_by_arm = {
            'A': (np.array([-50.0,  90.0, -180.0], dtype=float),
                  np.array([ 15.0, 170.0,  -125.0], dtype=float)),
            'B': (np.array([-15.0,  90.0, -180.0], dtype=float),
                  np.array([ 50.0, 180.0,  -120.0], dtype=float)),
        }

        # ========= Estado activo DH/Límites =========
        self._dh_params = None
        self.ik_lower_deg = None
        self.ik_upper_deg = None
        self._apply_arm_config(self.selected_arm)

        # ========= Timer =========
        self.timer = self.create_timer(self.dt, self.timer_cb)

        # ========= Aux =========
        self.last_sent_geom_q  = None
        self.last_sent_cmd_q   = None
        self.last_sim_geom_q_A = None
        self.last_sim_geom_q_B = None

        # ========= Selector de solver IK =========
        self.ik_mode = 'ALG'   # 'ALG','NUM','GEOM','NEWT','GRAD','MTH'

       

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
        if P_at is not None and mode_running == 'C':
            self._set_curr_cart_sim(arm_running, P_at)
        self.traj_active = False
        self.waiting_ack = False
        self.last_sent_geom_q = None
        self.last_sent_cmd_q  = None
        self.get_logger().info(f"⏹️ Trayectoria abortada por {reason}.")

    def _build_cmd_payload(self, out3, arm: str):
        arr = self.all_cmd[:]  # copia
        start = 0 if arm == 'A' else 3
        for k, v in enumerate(out3):
            arr[start + k] = float(v)
        return arr

    # ------------------- Conmutación por brazo -------------------
    def _apply_arm_config(self, arm: str):
        self._dh_params = list(self._dh_by_arm[arm])
        lower, upper = self._limits_by_arm[arm]
        self.ik_lower_deg = lower.astype(float).copy()
        self.ik_upper_deg = upper.astype(float).copy()
        self.q_seed_A = self._clamp_to_limits(self.q_seed_A)
        self.q_seed_B = self._clamp_to_limits(self.q_seed_B)
        self.get_logger().info(
            f"🧩 Config brazo {arm}: limits={self.ik_lower_deg.tolist()}..{self.ik_upper_deg.tolist()}"
        )

    # ------------------- Utilidades IK/FK/Límites -------------------
    @staticmethod
    def _wrap_deg(a: float) -> float:
        return (a + 180.0) % 360.0 - 180.0

    @staticmethod
    def _wrap_pi(a: float) -> float:
        return (a + math.pi) % (2.0*math.pi) - math.pi

    def _clamp_to_limits(self, q_deg: np.ndarray) -> np.ndarray:
        return np.minimum(self.ik_upper_deg, np.maximum(self.ik_lower_deg, q_deg.astype(float)))

    def _project_into_limits(self, q_deg: np.ndarray) -> np.ndarray | None:
        out = q_deg.copy().astype(float)
        for i in range(3):
            lo, hi = float(self.ik_lower_deg[i]), float(self.ik_upper_deg[i])
            k_min = math.ceil((lo - out[i]) / 360.0)
            k_max = math.floor((hi - out[i]) / 360.0)
            if k_min > k_max: return None
            if 0 >= k_min and 0 <= k_max: k = 0
            else: k = k_min if abs(k_min) <= abs(k_max) else k_max
            out[i] = out[i] + 360.0 * k
        return out

    def _get_L123_from_dh(self) -> tuple[float, float, float]:
        L1_m = None; L2_m = None; L3_m = None
        for (theta, d_mm, a_mm, alpha) in self._dh_params:
            if theta == "q2": L1_m = float(d_mm) / 1000.0
            elif theta == "q4": L2_m = float(a_mm) / 1000.0
            elif theta == "q5": L3_m = float(a_mm) / 1000.0
        if None in (L1_m, L2_m, L3_m):
            raise RuntimeError("No se pudieron leer L1/L2/L3 del DH activo.")
        return L1_m, L2_m, L3_m

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
        return T[:3, 3].copy()

    # --------- Verificación unificada: workspace + límites articulares ----------
    def _is_reachable(self, P_xyz_m: np.ndarray) -> bool:
        x, y, z = float(P_xyz_m[0]), float(P_xyz_m[1]), float(P_xyz_m[2])
        L1, L2, L3 = self._get_L123_from_dh()
        a2, a3 = abs(L2), abs(L3)  # longitudes positivas

        # --- (1) Chequeo geométrico tipo 2R ---
        Rxy = float(np.hypot(x, y))
        th2 = 0.0 if Rxy < 1e-12 else float(np.arctan2(-x, y))
        x2 = float(y*math.cos(th2) - x*math.sin(th2))
        z2 = float(z - L1)
        dist = float(np.hypot(x2, z2))

        if not ((abs(a2 - a3) - 1e-9) <= dist <= (a2 + a3 + 1e-9)):
            return False  # fuera del workspace geométrico

        # --- (2) Chequeo de existencia de IK dentro de límites ---
        TOL_POS = 5e-4  # 0.5 mm

        # a) Analítico (down)
        try:
            q_alg = self._ik_algebraic_down(P_xyz_m)
            q_proj = self._project_into_limits(q_alg)
            if q_proj is not None:
                P_fk = self._fk_pos_from_q(q_proj)
                if float(np.linalg.norm(P_fk - P_xyz_m)) <= TOL_POS:
                    return True
        except Exception:
            pass

        # b) Numérico LSQ
        try:
            seed = (self.q_seed_A if self.selected_arm == 'A' else self.q_seed_B).copy()
            seed = self._clamp_to_limits(seed)
            lower = self.ik_lower_deg.astype(float)
            upper = self.ik_upper_deg.astype(float)

            res = least_squares(
                self._ik_residuals, x0=seed, args=(np.array(P_xyz_m, float),),
                bounds=(lower, upper), xtol=1e-8, ftol=1e-8, gtol=1e-8, max_nfev=1000
            )
            q_num = np.minimum(upper, np.maximum(lower, res.x.astype(float)))
            P_fk = self._fk_pos_from_q(q_num)
            if float(np.linalg.norm(P_fk - P_xyz_m)) <= TOL_POS:
                return True
        except Exception:
            pass

        return False

    # ------------------- IK (NUM y ALG) -------------------
    def _ik_residuals(self, q_deg_245: np.ndarray, target_xyz: np.ndarray) -> np.ndarray:
        pos = self._fk_pos_from_q(q_deg_245)
        return pos - target_xyz

    def solve_ik(self, P_xyz_m: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        target = np.array(P_xyz_m, dtype=float)
        seed   = self._clamp_to_limits(np.array(q_seed_deg, dtype=float))
        lower  = self.ik_lower_deg.astype(float)
        upper  = self.ik_upper_deg.astype(float)
        res = least_squares(
            self._ik_residuals, x0=seed, args=(target,),
            bounds=(lower, upper), xtol=1e-8, ftol=1e-8, gtol=1e-8, max_nfev=2000
        )
        q = res.x.astype(float)
        return self._clamp_to_limits(q)

    def _ik_algebraic_down(self, P_xyz_m: np.ndarray) -> np.ndarray:
        x, y, z = [float(P_xyz_m[0]), float(P_xyz_m[1]), float(P_xyz_m[2])]
        L1, L2, L3 = self._get_L123_from_dh()
        a2, a3 = abs(L2), abs(L3)
        Rxy = float(np.hypot(x, y))
        th2 = 0.0 if Rxy < 1e-12 else float(np.arctan2(-x, y))
        x2 = float(y*np.cos(th2) - x*np.sin(th2))
        z2 = float(z - L1)
        r2 = x2*x2 + z2*z2
        cos_th5 = (a2*a2 + a3*a3 - r2) / (2.0*a2*a3)
        cos_th5 = max(-1.0, min(1.0, cos_th5))
        th5 = -float(np.arccos(cos_th5))  # codo abajo
        A = L2 + L3*np.cos(th5)
        B = L3*np.sin(th5)
        th4 = float(np.arctan2(z2, x2) - np.arctan2(B, A))
        out_deg = np.degrees([th2, th4, th5]).astype(float)
        out_deg = np.array([self._wrap_deg(v) for v in out_deg], dtype=float)
        return out_deg

    def solve_ik_alg(self, P_xyz_m: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        q_down = self._ik_algebraic_down(P_xyz_m)
        q_proj = self._project_into_limits(q_down)
        if q_proj is None:
            self.get_logger().warn("⚠️ IK fuera de límites; aplicando clamp duro.")
            q_proj = self._clamp_to_limits(q_down)
        return self._clamp_to_limits(q_proj)

    # ------------------- SOLVER NEWTON (integrado) -------------------
    def solve_ik_newt(self, P_xyz_m: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        """
        Integración directa del método de Newton que enviaste.
        Trabaja en rad internamente; devuelve grados (q2,q4,q5).
        Sin clamps internos (tu pipeline los aplica después).
        """
        # ===== Parámetros desde tu DH activo =====
        L1, L2, L3 = self._get_L123_from_dh()  # m (incluye signos)

        # ===== Utilidades del solver (idénticas a tu código) =====
        def wrap_to_pi(a):
            return (a + math.pi) % (2*math.pi) - math.pi

        def dh_transform(theta, d, a, alpha):
            ct, st = math.cos(theta), math.sin(theta)
            ca, sa = math.cos(alpha), math.sin(alpha)
            return np.array([
                [ ct, -st*ca,  st*sa, a*ct],
                [ st,  ct*ca, -ct*sa, a*st],
                [  0,     sa,     ca,    d],
                [  0,      0,      0,    1],
            ], dtype=float)

        def forward_kinematics(th1, th2, th3):
            # DH exacta equivalente a tu script
            d     = [L1, 0.0, -0.017, -0.007]
            a     = [0.025,  0.0,  L2, L3]
            alpha = [0.0, math.pi/2, 0.0, 0.0]
            theta = [th1, math.pi/2, th2, th3]
            T1 = dh_transform(theta[0], d[0], a[0], alpha[0])
            T2 = dh_transform(theta[1], d[1], a[1], alpha[1])
            T3 = dh_transform(theta[2], d[2], a[2], alpha[2])
            T4 = dh_transform(theta[3], d[3], a[3], alpha[3])
            T  = T1 @ T2 @ T3 @ T4
            return T[0:3, 3]

        def fkine(q_rad):
            return forward_kinematics(wrap_to_pi(q_rad[0]), wrap_to_pi(q_rad[1]), wrap_to_pi(q_rad[2]))

        def newton_step(J, e):
            return np.linalg.solve(J, e)

        # ===== Inicialización =====
        target = np.array(P_xyz_m, dtype=float)
        q0_deg = np.array(q_seed_deg, dtype=float)          # [q2,q4,q5] en deg
        q      = np.radians(q0_deg)                         # a rad
        q[0] = wrap_to_pi(q[0]); q[1] = wrap_to_pi(q[1]); q[2] = wrap_to_pi(q[2])

        FD_TH   = 1e-3
        EPS     = 1e-6
        MAX_IT  = 5000

        # ===== Iteración =====
        for _ in range(MAX_IT):
            f0 = fkine(q)
            e  = target - f0
            if float(np.linalg.norm(e)) < EPS:
                break

            J = np.empty((3,3), dtype=float)
            q1p = np.array([wrap_to_pi(q[0] + FD_TH), q[1], q[2]])
            q2p = np.array([q[0], wrap_to_pi(q[1] + FD_TH), q[2]])
            q3p = np.array([q[0], q[1], wrap_to_pi(q[2] + FD_TH)])

            f1 = fkine(q1p); f2 = fkine(q2p); f3 = fkine(q3p)
            J[:,0] = (f1 - f0) / FD_TH
            J[:,1] = (f2 - f0) / FD_TH
            J[:,2] = (f3 - f0) / FD_TH

            dq = newton_step(J, e)
            q += dq
            q[0] = wrap_to_pi(q[0]); q[1] = wrap_to_pi(q[1]); q[2] = wrap_to_pi(q[2])

        # Devuelve en grados (tu orden q2,q4,q5)
        q_deg = np.degrees(q).astype(float)
        q_deg = np.array([self._wrap_deg(v) for v in q_deg], dtype=float)
        return q_deg

    # ------------------- SOLVER GRADIENTE (integrado) -------------------
    def solve_ik_grad(self, P_xyz_m: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        """
        Integración directa del gradiente + ADAM que enviaste.
        Trabaja en rad internamente; devuelve grados (q2,q4,q5).
        Sin clamps internos; respetamos tu pipeline actual.
        """
        # ===== Parámetros desde tu DH activo =====
        L1, L2, L3 = self._get_L123_from_dh()

        # ===== Utilidades (idénticas a tu script) =====
        def dh_matrix(theta, d, a, alpha):
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)
            return np.array([
                [ct, -st*ca,  st*sa, a*ct],
                [st,  ct*ca, -ct*sa, a*st],
                [0,   sa,     ca,    d],
                [0,   0,      0,     1]
            ], dtype=float)

        def forward_kinematics_dh(q):
            theta1, theta2, theta3 = q
            d     = [L1, 0.0, -0.017, -0.007]
            a     = [0.025,  0.0,  L2, L3]
            alpha = [0.0, np.pi/2, 0.0, 0.0]
            theta = [theta1, np.pi/2, theta2, theta3]
            T1 = dh_matrix(theta[0], d[0], a[0], alpha[0])
            T2 = dh_matrix(theta[1], d[1], a[1], alpha[1])
            T3 = dh_matrix(theta[2], d[2], a[2], alpha[2])
            T4 = dh_matrix(theta[3], d[3], a[3], alpha[3])
            T  = T1 @ T2 @ T3 @ T4
            return T[0:3, 3]

        def numerical_jacobian(q, delta=1e-7):
            J  = np.zeros((3,3), dtype=float)
            f0 = forward_kinematics_dh(q)
            for i in range(3):
                qp = q.copy()
                qp[i] += delta
                fp = forward_kinematics_dh(qp)
                J[:, i] = (fp - f0) / delta
            return J

        def cost(q, xd):
            f = forward_kinematics_dh(q)
            return 0.5 * float(np.linalg.norm(xd - f)**2)

        def wrap_to_pi_vec(q):
            return (q + np.pi) % (2*np.pi) - np.pi

        # ===== Inicialización =====
        xd = np.array(P_xyz_m, dtype=float)
        q  = np.radians(np.array(q_seed_deg, dtype=float))
        q  = wrap_to_pi_vec(q)

        # ADAM
        beta1, beta2 = 0.9, 0.999
        eps_adam = 1e-8
        m = np.zeros(3, dtype=float)
        v = np.zeros(3, dtype=float)
        alpha_adam = 0.1

        epsilon   = 1e-6
        max_iter  = 100000
        best_q    = q.copy()
        best_err  = cost(q, xd)
        no_improve = 0

        # ===== Optimización =====
        for i in range(max_iter):
            f = forward_kinematics_dh(q)
            e = xd - f
            err = float(np.linalg.norm(e))

            # Mejor-so-far
            if err < best_err:
                best_err  = err
                best_q    = q.copy()
                no_improve = 0
            else:
                no_improve += 1

            if err < epsilon:
                break

            # Gradiente
            J = numerical_jacobian(q)
            grad = J.T @ e

            # ADAM
            m = beta1*m + (1.0 - beta1)*grad
            v = beta2*v + (1.0 - beta2)*(grad**2)
            m_hat = m / (1.0 - beta1**(i+1))
            v_hat = v / (1.0 - beta2**(i+1))

            q = q + alpha_adam * m_hat / (np.sqrt(v_hat) + eps_adam)
            q = wrap_to_pi_vec(q)

            # Reinicio si estanca
            if no_improve > 500:
                q = best_q + np.random.randn(3)*0.1
                q = wrap_to_pi_vec(q)
                m[:] = 0.0; v[:] = 0.0
                no_improve = 0

        # Resultado final (mejor alcanzado)
        q = best_q
        q_deg = np.degrees(q).astype(float)
        q_deg = np.array([self._wrap_deg(v) for v in q_deg], dtype=float)
        return q_deg

    # ------------------- Fallbacks -------------------
    def solve_ik_geom(self, P_xyz_m: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        self.get_logger().warn("⚠️ IK GEOM no implementado aún — usando ALG (down) como fallback.")
        return self.solve_ik_alg(P_xyz_m, q_seed_deg)

    def solve_ik_mth(self, P_xyz_m: np.ndarray, q_seed_deg: np.ndarray) -> np.ndarray:
        self.get_logger().warn("⚠️ IK MTH no implementado aún — usando NUM (LSQ) como fallback.")
        return self.solve_ik(P_xyz_m, q_seed_deg)

    # ------------------- Feedback REAL -------------------
    def on_feedback(self, msg: Float32MultiArray):
        vals = list(msg.data or [])
        if len(vals) != 3:
            self.get_logger().warn(f"⚠️ /arm_feedback inválido: {vals}")
            return

        meas = np.array(vals, dtype=float)
        meas_clamped = self._clamp_to_limits(meas)

        start = 0 if self.selected_arm == 'A' else 3
        for k in range(3):
            self.all_joints[start + k] = round(float(meas_clamped[k]), 2)

        try:
            P_meas = self._fk_pos_from_q(meas_clamped)
            self._set_curr_cart_real(self.selected_arm, P_meas)
        except Exception as e:
            self.get_logger().warn(f"FK desde feedback falló: {e}")

        if self.selected_mode == 'D' and self.traj_active:
            if self.last_sent_geom_q is not None:
                ref = self._clamp_to_limits(self.last_sent_geom_q)
                err = np.abs(meas_clamped - ref)
                if np.all(err <= self.tol):
                    self.waiting_ack = False
                    if self.selected_arm == 'A': self.q_seed_A = ref.copy()
                    else:                         self.q_seed_B = ref.copy()
                    try:
                        self.P_base = P_meas.copy()
                    except Exception:
                        pass
                    self.get_logger().info(
                        f"✅ ACK paso REAL dentro de tol. err=[{err[0]:.2f}, {err[1]:.2f}, {err[2]:.2f}]"
                    )
                else:
                    self.waiting_ack = False
                    self.get_logger().info(
                        f"⏳ Paso REAL fuera de tol. err=[{err[0]:.2f}, {err[1]:.2f}, {err[2]:.2f}] → continuar"
                    )
            else:
                self.waiting_ack = False

    # ------------------- Timer principal (interpolación corregida) -------------------
    def timer_cb(self):
        if not self.traj_active:
            return

        # Finaliza cuando ya hicimos 'step' envíos
        if self.i >= self.step:
            self.traj_active = False
            self.waiting_ack = False
            if self.selected_mode == 'C':
                self._set_curr_cart_sim(self.selected_arm, self.P_goal)
            self.P_base = self.P_goal.copy()

            last_geom = (self.last_sim_geom_q_A if self.selected_arm == 'A' else self.last_sim_geom_q_B) \
                        if self.selected_mode == 'C' else self.last_sent_geom_q
            if last_geom is not None:
                if self.selected_arm == 'A': self.q_seed_A = self._clamp_to_limits(last_geom.copy())
                else:                         self.q_seed_B = self._clamp_to_limits(last_geom.copy())

            self.last_sent_geom_q = None
            self.last_sent_cmd_q  = None
            self.get_logger().info("🏁 Interpolación cartesiana finalizada.")
            return

        # ---- Interpolación lineal: primer paso ya avanza hacia el goal ----
        s = (self.i + 1) / float(self.step)
        if s > 1.0:
            s = 1.0
        P = (1.0 - s) * self.P_base + s * self.P_goal

        q_seed = (self.q_seed_A if self.selected_arm == 'A' else self.q_seed_B).copy()
        q_seed = self._clamp_to_limits(q_seed)

        mode = self.ik_mode
        if mode == 'ALG':
            q_ik_raw = self.solve_ik_alg(P, q_seed)
        elif mode == 'NUM':
            q_ik_raw = self.solve_ik(P, q_seed)
        elif mode == 'GEOM':
            q_ik_raw = self.solve_ik_geom(P, q_seed)
        elif mode == 'NEWT':
            q_ik_raw = self.solve_ik_newt(P, q_seed)
        elif mode == 'GRAD':
            q_ik_raw = self.solve_ik_grad(P, q_seed)
        elif mode == 'MTH':
            q_ik_raw = self.solve_ik_mth(P, q_seed)
        else:
            self.get_logger().warn(f"⚠️ IK mode desconocido '{mode}', usando ALG.")
            q_ik_raw = self.solve_ik_alg(P, q_seed)

        q_ik = self._clamp_to_limits(np.round(q_ik_raw, 2))

        # === NUEVO: publicar grados puros y con offset por el MISMO tópico '/ik_deg' ===
        # puros: q_ik (q2,q4,q5) en deg del método seleccionado
        # con offset: q_ik + offset_{A|B} (clamp a límites)
        q_off = q_ik.copy()
        if self.selected_arm == 'A':
            q_off = [(-1*q_off[0])+90,q_off[1], (1 * q_off[2]) + 180]
        else:
            q_off = [-1*(q_off[0]+90),q_off[1], (-1 * q_off[2]) - 90]
            

        ik_msg = Float32MultiArray()
        # data: [q2_puro, q4_puro, q5_puro, q2_off, q4_off, q5_off]
        ik_msg.data = list(np.concatenate([q_ik, q_off]).astype(float))
        self.ik_pub.publish(ik_msg)

        if self.selected_arm == 'A': self.q_seed_A = q_ik.copy()
        else:                         self.q_seed_B = q_ik.copy()

        out3 = q_ik.copy()
        if self.selected_mode == 'C':
            out3 = [90 - out3[0], 180 - out3[1], 180 + out3[2]]
        else:
            if self.selected_arm == 'A':
                out3 = [(-1* out3[0] + 90), 180 - out3[1],  (out3[2]*-1)-90]
            else:
                out3 = [out3[0], 180 - out3[1], out3[2]]

        start = 0 if self.selected_arm == 'A' else 3

        if self.selected_mode == 'C':
            for k in range(3):
                self.all_joints[start + k] = float(out3[k])
            msg = Float32MultiArray(); msg.data = self.all_joints
            self.sim_pub.publish(msg)

            if self.selected_arm == 'A': self.last_sim_geom_q_A = q_ik.copy()
            else:                        self.last_sim_geom_q_B = q_ik.copy()

            try:
                P_exec = self._fk_pos_from_q(q_ik)
            except Exception:
                P_exec = P.copy()
            self._set_curr_cart_sim(self.selected_arm, P_exec)

            self.i += 1
            self.get_logger().info(
                f"📤 SIM {self.i}/{self.step}  P=[{P[0]:.3f}, {P[1]:.3f}, {P[2]:.3f}]  "
                f"q_deg={q_ik.tolist()}  q_sim={np.round(out3,2)}"
            )
            self.P_base = P_exec.copy()

        else:
            if self.waiting_ack:
                return

            cmd6 = self._build_cmd_payload(out3, self.selected_arm)
            msg = Float32MultiArray(); msg.data = cmd6
            self.cmd_pub.publish(msg)

            self.last_sent_geom_q = q_ik.copy()
            self.last_sent_cmd_q  = np.array(out3, float)
            self.waiting_ack = True

            self.i += 1
            self.get_logger().info(
                f"📤 REAL {self.i}/{self.step}  P=[{P[0]:.3f}, {P[1]:.3f}, {P[2]:.3f}]  "
                f"q_deg={q_ik.tolist()}   q_cmd={np.round(out3,2)} → ACK"
            )

    # ------------------- Callbacks -------------------
    def goal_callback(self, msg: PoseStamped):
        p = msg.pose.position
        goal = np.array([float(p.x), float(p.y), float(p.z)], dtype=float)
        curr = self._get_curr_cart_for_mode(self.selected_arm)

        if not self._is_reachable(goal):
            self.waiting_ack = False
            self.last_sent_geom_q = None
            self.last_sent_cmd_q  = None
            self.get_logger().warn(
                f"❌ Coordenada inválida (fuera del espacio de trabajo). "
                f"Suministre una coordenada válida. Punto: "
                f"[{goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}]"
            )
            return

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
            if self.traj_active and val != self.selected_arm:
                self._abort_current(self.selected_arm, self.selected_mode, "cambio de brazo")
            self.selected_arm = val
            self._apply_arm_config(self.selected_arm)
            self.q_seed_A = self._clamp_to_limits(self.q_seed_A)
            self.q_seed_B = self._clamp_to_limits(self.q_seed_B)
            self.get_logger().info(f"🅰️/🅱️ Brazo seleccionado: {self.selected_arm}")

    def run_mode(self, msg: String):
        val = (msg.data or '').strip().upper()
        if val in ('C', 'D'):
            if self.traj_active and val != self.selected_mode:
                self._abort_current(self.selected_arm, self.selected_mode, "cambio de modo")
            self.selected_mode = val
            self.q_seed_A = self._clamp_to_limits(self.q_seed_A)
            self.q_seed_B = self._clamp_to_limits(self.q_seed_B)
            self.get_logger().info(f"⚙️ Modo: {'SIM' if val=='C' else 'REAL'}")

    def on_ik_mode(self, msg: String):
        val = (msg.data or '').strip().upper()
        if val in ('ALG', 'NUM', 'GEOM', 'NEWT', 'GRAD', 'MTH'):
            self.ik_mode = val
            self.get_logger().info(f"🔀 IK solver seleccionado: {self.ik_mode}")
        else:
            self.get_logger().warn(f"⚠️ IK solver inválido: '{msg.data}' (usa 'ALG','NUM','GEOM','NEWT','GRAD','MTH')")

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

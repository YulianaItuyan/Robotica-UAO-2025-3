#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_cli_demo.py — IK Gauss-Newton (DLS) con Jacobiano numérico en RAD.
- Sin ROS. Solo NumPy.
- Interpolación cartesiana lineal p0 -> tgt en 'STEPS' pasos.
- En cada paso: IK con semilla del paso anterior.
- Imprime IK "puro" (grados sliders) y "publicado" con mapeo A/B.

>>> python3 ik_cli_demo.py
"""

import numpy as np
from math import sin, cos, atan2, radians, degrees, sqrt

# ============================================================
# ===============  CONFIGURACIÓN — EDITA AQUÍ  ===============
# ============================================================

# --- Geometría del robot (metros) ---
# EDITA ESTOS VALORES a tu hardware real:
L1 = -0.051   # altura base -> hombro
L2 = -0.105   # longitud eslabón 2
L3 = 0.215   # longitud eslabón 3

# --- Límites articulares (grados) ---
# EDITA si necesitas otros rangos por junta:
JOINT_LIMITS_DEG = np.array([
    [  -180.0, 15.0],   # q1
    [  0.0, 180.0],   # q2
    [ -180.0,-90.0],   # q3
], dtype=float)

# --- Parámetros de la IK (Gauss-Newton DLS) ---
# Puedes jugar con estos para estabilidad/precisión:
IK_MAX_ITERS = 120     # iteraciones máx. por waypoint
IK_LAMBDA    = 2e-2    # amortiguación (↑ robusto, ↓ preciso)
IK_TOL_POS   = 1e-4    # tolerancia posicional (m)
IK_JAC_EPS   = 1e-3    # incremento para Jacobiano (rad)

# --- Interpolación cartesiana ---
# Punto inicial y destino en METROS:
P0   = np.array([0.0, 0.0, -0.371], dtype=float)   # EDITA si quieres otra posición inicial
TGT  = np.array([0.10, -0.05, -0.30], dtype=float) # <<< EDITA EL DESTINO AQUÍ
STEPS = 60                                         # cantidad de waypoints (↑=más fino)

# --- Semilla articular inicial (grados sliders) ---
Q0_SEED_DEG = np.array([90.0, 90.0, 0.0], dtype=float)  # EDITA si quieres otra semilla

# --- Brazo que quieres emular ('A' o 'B') para el mapeo de publicación ---
ARM = 'A'   # 'A' o 'B'  <<< EDITA SI QUIERES PROBAR EL OTRO BRAZO

# =============  FIN CONFIGURACIÓN  =============


# ============================================================
# ==================  IMPLEMENTACIÓN IK  =====================
# ============================================================

def clamp_to_limits_deg(q_deg: np.ndarray) -> np.ndarray:
    """Satura q_deg a los límites JOINT_LIMITS_DEG (en grados)."""
    q = np.array(q_deg, dtype=float)
    lo = JOINT_LIMITS_DEG[:, 0]
    hi = JOINT_LIMITS_DEG[:, 1]
    return np.minimum(np.maximum(q, lo), hi)

def fk_geom_rad(q_rad: np.ndarray) -> np.ndarray:
    """
    FK en rad:
      x = sin(q1)*(L2*cos q2 + L3*cos(q2+q3))
      y = -cos(q1)*(L2*cos q2 + L3*cos(q2+q3))
      z =  L1 + L2*sin q2 + L3*sin(q2+q3)
    """
    q1, q2, q3 = float(q_rad[0]), float(q_rad[1]), float(q_rad[2])
    r_xy = L2*cos(q2) + L3*cos(q2+q3)
    x = sin(q1) * r_xy
    y = -cos(q1) * r_xy
    z = L1 + L2*sin(q2) + L3*sin(q2+q3)
    return np.array([x, y, z], dtype=float)

def jac_numeric_rad(q_rad: np.ndarray, eps: float = IK_JAC_EPS) -> np.ndarray:
    """Jacobiano numérico 3x3: d pos / d rad."""
    q = np.array(q_rad, dtype=float)
    f0 = fk_geom_rad(q)
    J = np.zeros((3, 3), dtype=float)
    for i in range(3):
        dq = np.zeros(3); dq[i] = eps
        f1 = fk_geom_rad(q + dq)
        J[:, i] = (f1 - f0) / eps
    return J

def ik_gauss_newton_cart(target_xyz_m: np.ndarray,
                         seed_deg: np.ndarray) -> np.ndarray:
    """
    IK Gauss-Newton (DLS) sobre posición. Internamente en rad, salida en deg (sliders).
    - target_xyz_m: np.array([x,y,z]) en metros
    - seed_deg:     np.array([q1,q2,q3]) en grados (se recorta a límites)
    """
    q_deg = clamp_to_limits_deg(seed_deg)
    q_rad = np.radians(q_deg)

    # Empujoncito para q1 mirando hacia el objetivo en XY (ayuda a converger)
    x, y, _ = target_xyz_m
    if abs(x) + abs(y) > 1e-6:
        q1_guess = atan2(x, -y)
        q_rad[0] = 0.5*q_rad[0] + 0.5*q1_guess

    for _ in range(IK_MAX_ITERS):
        p = fk_geom_rad(q_rad)
        e = target_xyz_m - p
        if np.linalg.norm(e) < IK_TOL_POS:
            break
        J = jac_numeric_rad(q_rad, eps=IK_JAC_EPS)
        JT = J.T
        H  = JT @ J + IK_LAMBDA * np.eye(3)
        dq = np.linalg.lstsq(H, JT @ e, rcond=None)[0]
        if not np.all(np.isfinite(dq)):
            break
        q_rad = q_rad + dq

    q_out_deg = np.degrees(q_rad)
    return clamp_to_limits_deg(q_out_deg)

def map_publish_deg(q_deg: np.ndarray, arm: str) -> np.ndarray:
    """
    Mapeo de publicación igual al nodo:
      - Brazo A: [ q1, 180 - q2,  q3]
      - Brazo B: [180 - q1, 180 - q2, q3]
    """
    q1, q2, q3 = float(q_deg[0]), float(q_deg[1]), float(q_deg[2])
    if arm.upper() == 'A':
        return np.array([ q1, 180.0 - q2,  q3], dtype=float)
    else:
        return np.array([180.0 - q1, 180.0 - q2, q3], dtype=float)


# ============================================================
# ======================  DEMOSTRACIÓN  ======================
# ============================================================

def main():
    # Construir trayectoria cartesiana lineal
    steps = max(2, int(STEPS))
    pts = [P0 + (TGT - P0) * (i / steps) for i in range(1, steps + 1)]

    print("\n=== IK CLI DEMO (Gauss-Newton DLS) ===")
    print(f"Links (m): L1={L1:.3f}, L2={L2:.3f}, L3={L3:.3f}")
    print(f"Límites (deg): {JOINT_LIMITS_DEG.tolist()}")
    print(f"IK: iters={IK_MAX_ITERS}, lambda={IK_LAMBDA}, tol={IK_TOL_POS}, jac_eps={IK_JAC_EPS}")
    print(f"Brazo para mapeo: {ARM}")
    print(f"\nTrayectoria cartesiana:")
    print(f"  p0  = {np.round(P0, 6).tolist()}")
    print(f"  tgt = {np.round(TGT, 6).tolist()}")
    print(f"  steps = {steps}\n")

    q_seed = Q0_SEED_DEG.copy()
    for i, p in enumerate(pts, start=1):
        q_ik = ik_gauss_newton_cart(np.asarray(p, dtype=float), q_seed)
        q_pub = map_publish_deg(q_ik, ARM)
        print(f"Paso {i:3d}/{steps}: target={np.round(p,6).tolist()}  "
              f"| IK_puro(deg)={np.round(q_ik,3).tolist()}  "
              f"-> publicado={np.round(q_pub,3).tolist()}")
        # la semilla del siguiente paso es la solución actual
        q_seed = q_ik

    print("\nFIN. Último IK puro (deg):", np.round(q_seed, 3).tolist())
    print("Último publicado (deg):   ", np.round(map_publish_deg(q_seed, ARM), 3).tolist())
    print()

if __name__ == "__main__":
    main()

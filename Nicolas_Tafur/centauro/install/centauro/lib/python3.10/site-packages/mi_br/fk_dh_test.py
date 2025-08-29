#!/usr/bin/env python3
import numpy as np

# ---------- Definiciones ----------

# Parámetros DH de ejemplo (pon los reales aquí en metros y radianes)
# Formato: [a, alpha, d, theta_offset]
DH_PARAMS = [
    [0.01, -1.57,  0.03, 0],       # Joint 1
    [0.05, 0, 0.105, 0],       # Joint 2
    [0.025, -1.57, 0.15, 0],       # Joint 3
]

def dh_transform(a, alpha, d, theta):
    """Devuelve la matriz homogénea según parámetros DH."""
    cth, sth = np.cos(theta), np.sin(theta)
    cal, sal = np.cos(alpha), np.sin(alpha)
    T = np.array([
        [ cth,    -sth*cal,   sth*sal,   a*cth ],
        [ sth,     cth*cal,  -cth*sal,   a*sth ],
        [ 0.0,        sal,       cal,       d ],
        [ 0.0,       0.0,       0.0,      1.0 ]
    ], dtype=float)
    return T

def forward_kinematics_dh(q_deg, dh_params):
    """Calcula FK con la tabla DH dada."""
    q_rad = np.deg2rad(q_deg)
    T_total = np.eye(4)
    for i, (a, alpha, d, theta_off) in enumerate(dh_params):
        theta = q_rad[i] + theta_off
        T_total = T_total @ dh_transform(a, alpha, d, theta)
    p = T_total[:3, 3]
    R = T_total[:3, :3]
    return p, R, T_total

# ---------- Ejecución ----------
if __name__ == "__main__":
    print("Introduce los 3 ángulos (grados) separados por espacio:")
    entrada = input("> ")
    try:
        q_vals = [float(v) for v in entrada.strip().split()]
        if len(q_vals) != 3:
            raise ValueError("Debes ingresar exactamente 3 valores.")

        pos, rot, T = forward_kinematics_dh(q_vals, DH_PARAMS)

        np.set_printoptions(precision=4, suppress=True)
        print("\n--- Resultado FK ---")
        print(f"Posición (x, y, z) [m]: {pos}")
        print("\nMatriz de rotación R:")
        print(rot)
        print("\nMatriz de transformación homogénea T:")
        print(T)

    except Exception as e:
        print(f"Error: {e}")


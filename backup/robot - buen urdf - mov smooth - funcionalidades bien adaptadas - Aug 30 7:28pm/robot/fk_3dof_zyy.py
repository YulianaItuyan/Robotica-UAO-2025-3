#!/usr/bin/env python3
import numpy as np
import argparse
import sys

def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s, 0.0],
                  [s,  c, 0.0],
                  [0.0, 0.0, 1.0]])
    return R

def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[ c, 0.0,  s],
                  [0.0, 1.0, 0.0],
                  [-s, 0.0,  c]])
    return R

def to_hom(R, p):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = p
    return T

def Tx(L):
    T = np.eye(4)
    T[0,3] = L
    return T

def forward_kinematics(q_deg, L):
    """
    q_deg: [q1_deg, q2_deg, q3_deg] in degrees (0..180 expected)
    L:     [L1, L2, L3] link lengths (meters or your chosen unit)
    Model: T = Rz(q1) * Tx(L1) * Ry(q2) * Tx(L2) * Ry(q3) * Tx(L3)
    """
    q = np.deg2rad(np.array(q_deg, dtype=float))
    L1, L2, L3 = L

    T01 = to_hom(rot_z(q[0]), np.zeros(3)) @ Tx(L1)
    T12 = to_hom(rot_y(q[1]), np.zeros(3)) @ Tx(L2)
    T23 = to_hom(rot_y(q[2]), np.zeros(3)) @ Tx(L3)

    T03 = T01 @ T12 @ T23
    p = T03[:3, 3]
    R = T03[:3, :3]
    return p, R, T03

def main():
    parser = argparse.ArgumentParser(description="FK 3-DOF (Z, Y, Y)")
    parser.add_argument("--q", nargs=3, type=float, metavar=("Q1_deg","Q2_deg","Q3_deg"),
                        help="Ángulos de junta en grados (ej: --q 30 45 10)")
    parser.add_argument("--L", nargs=3, type=float, default=[0.12, 0.10, 0.08],
                        metavar=("L1","L2","L3"),
                        help="Longitudes de eslabón (ej: --L 0.12 0.10 0.08)")
    args = parser.parse_args()

    if args.q is None:
        try:
            raw = input("Ingresa 3 ángulos (grados, separados por espacio): ")
            q_deg = list(map(float, raw.strip().split()))
            if len(q_deg) != 3:
                raise ValueError
        except Exception:
            print("Error: debes ingresar exactamente 3 valores numéricos.", file=sys.stderr)
            sys.exit(1)
    else:
        q_deg = args.q

    # Validación blanda de rango (0..180)
    for i, ang in enumerate(q_deg):
        if not (0.0 <= ang <= 180.0):
            print(f"Advertencia: q{i+1}={ang}° está fuera del rango [0, 180]. Se continúa igual.")

    p, R, T = forward_kinematics(q_deg, args.L)

    np.set_printoptions(precision=6, suppress=True)
    print("\nResultados:")
    print(f"  Posición (x, y, z): {p}")
    print(f"\n  Matriz de rotación R:\n{R}")
    print(f"\n  Transformación homogénea T:\n{T}")

if __name__ == "__main__":
    main()


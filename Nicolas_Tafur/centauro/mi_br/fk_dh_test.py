#!/usr/bin/env python3
import numpy as np

# ---------- Define aquí la función denavit() ----------
def denavit(theta, d, a, alpha):
    cth, sth = np.cos(theta), np.sin(theta)
    cal, sal = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ cth,    -sth * cal,   sth * sal,   a * cth ],
        [ sth,     cth * cal,  -cth * sal,   a * sth ],
        [ 0.0,        sal,         cal,        d   ],
        [ 0.0,        0.0,         0.0,      1.0  ]
    ], dtype=float)

# Mostrar matrices con 4 decimales
np.set_printoptions(precision=4, suppress=True)

# Parámetros D-H [θ_offset (rad), d (m), a (m), α (rad)]
dh_params = [
    [   0.0,   -0.03,  -0.01,     0.0    ],   # link 1
    [   0.0,    0.00,   0.00,   -np.pi/2],   # link 2 (θ siempre 0)
    [-np.pi/2,  0.050, -0.105,    0.0    ],   # link 3
    [   0.0,    0.025, -0.16,     0.0    ],   # link 4
]

# Pide ángulos a usuario (joints 1, 3 y 4)
entrada = input(
    "Introduce ángulos de joint 1, 3 y 4 (grados, separados por espacio):\n> "
)
q_in = [float(x) for x in entrada.split()]
if len(q_in) != 3:
    raise ValueError("Debes ingresar 3 ángulos (joints 1, 3 y 4).")

# Reconstruye lista completa de ángulos (link2 siempre 0)
q_deg = [q_in[0], 0.0, q_in[1], q_in[2]]

# FK DH
T_total = np.eye(4)
for i, (theta_off, d, a, alpha) in enumerate(dh_params, start=1):
    theta = np.deg2rad(q_deg[i-1]) + theta_off
    T_i = denavit(theta, d, a, alpha)
    T_total = T_total @ T_i
    print(f"\nTransformación del eslabón {i}:")
    print(T_i)

print("\nTransformación total del sistema (base → efector):")
print(T_total)


# Mostrar matrices con 4 decimales y sin notación científica
np.set_printoptions(precision=4, suppress=True)

# Parámetros DH [θ_offset (rad), d (m), a (m), α (rad)]
dh_params = [
    [   0.0,   -0.03,  -0.01,     0.0    ],   # link 1
    [   0.0,    0.00,   0.00,   -np.pi/2],   # link 2 (θ always 0)
    [-np.pi/2,  0.050, -0.105,    0.0    ],   # link 3
    [   0.0,    0.025, -0.16,     0.0    ],   # link 4
]

# Pedir al usuario los ángulos de las articulaciones 1, 3 y 4
entrada = input(
    "Introduce ángulos de joint 1, 3 y 4 (en grados, separados por espacio):\n> "
)
q_input = [float(x) for x in entrada.strip().split()]
if len(q_input) != 3:
    raise ValueError("Debes ingresar EXACTAMENTE 3 valores (joints 1, 3 y 4).")

# Reconstruir la lista completa de ángulos en grados (link2 = 0 siempre)
# Índices:   0      1    2      3
q_deg = [q_input[0], 0.0, q_input[1], q_input[2]]

# FK DH
T_total = np.eye(4)
for i, (theta_off, d, a, alpha) in enumerate(dh_params, start=1):
    # Conversión a rad + offset
    theta_rad = np.deg2rad(q_deg[i-1]) + theta_off

    # Transformación DH del eslabón i
    T_i = denavit(theta_rad, d, a, alpha)
    T_total = T_total @ T_i

    print(f"\nTransformación del eslabón {i}:")
    print(T_i)

# Pose final del efector
print("\nTransformación total del sistema (base → efector):")
print(T_total)


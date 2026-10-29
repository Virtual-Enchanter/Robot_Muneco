"""
Utilidades comunes de cinemática de la pierna.

Única fuente de:
    - longitudes de eslabón L1..L5 [mm]
    - límites articulares q1..q3 [°]
    - matriz de transformación DH
    - cinemática directa 0A4 (fk_T)
    - avisos de límites articulares
"""

import numpy as np


# Longitudes de eslabón [mm]
L1 = 90.3
L2 = 48.5
L3 = 54.2
L4 = 138.5
L5 = 162.0

# Límites articulares [°]
q1_min, q1_max = 0, 90
q2_min, q2_max = -90, 90
q3_min, q3_max = -90, 90


def dh_matrix(theta, d, a, alpha):
    """Matriz homogénea 4x4 de un eslabón (Denavit-Hartenberg)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])


def fk_T(q1, q2, q3, L1, L2, L3, L4, L5):
    """Cinemática directa 0A4 de la pierna izquierda (q en radianes)."""
    A01 = dh_matrix(0, d=-L1, a=L2, alpha=-np.pi / 2)
    A12 = dh_matrix(q1, d=0, a=L3, alpha=np.pi / 2)
    A23 = dh_matrix(q2, d=0, a=L4, alpha=np.pi)
    A34 = dh_matrix(q3, d=0, a=L5, alpha=0)
    return A01 @ A12 @ A23 @ A34


def joint_limit_warnings(q1_deg, q2_deg, q3_deg, tol=1e-6):
    """Devuelve un aviso por cada articulación fuera de su rango [°]."""
    warnings = []
    for nombre, v, mn, mx in (
        ('q1', q1_deg, q1_min, q1_max),
        ('q2', q2_deg, q2_min, q2_max),
        ('q3', q3_deg, q3_min, q3_max),
    ):
        if not mn - tol <= v <= mx + tol:
            warnings.append(
                f'{nombre} ({v:.2f}°) fuera de rango [{mn}°, {mx}°]')
    return warnings

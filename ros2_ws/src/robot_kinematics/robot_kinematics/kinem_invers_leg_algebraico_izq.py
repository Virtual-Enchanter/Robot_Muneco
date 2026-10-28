"""
Cinemática inversa
Implementación algebraica (roll-pitch-pitch con offset DH).
Modelo de 5 longitudes (L1..L5), sin articulación fantasma ni L6.
Pierna IZQUIERDA.
"""

import numpy as np

from robot_kinematics.cinematica_directa_der_izq import L1, L2, L3, L4, L5
from robot_kinematics.kinem_common import (
    joint_limit_warnings,
)

x, y, z = L2 + L3 + L4 + L5, 0.0, -L1   # objetivo de prueba (q1=q2=q3=0)


def cinematica_inversa_pata_alg(x, y, z, L1, L2, L3, L4, L5, codo="arriba"):
    xp = x - L2
    zp = z + L1                                    # <- IZQUIERDA: z + L1
    rho = np.sqrt(xp**2 + zp**2)

    q1 = np.arctan2(-zp, xp)                       # <- IZQUIERDA: -zp

    c3 = ((rho - L3)**2 + y**2 - L4**2 - L5**2) / (2*L4*L5)
    if abs(c3) > 1 + 1e-9:                 # tolerancia numérica en el borde
        return None, None, None, False
    c3 = np.clip(c3, -1.0, 1.0)

    s3 = np.sqrt(1 - c3**2) * (1 if codo == "arriba" else -1)
    q3 = np.arctan2(s3, c3)

    A = L4 + L5*np.cos(q3)
    B = L5*np.sin(q3)
    det = A**2 + B**2
    c2 = (A*(rho - L3) - B*y) / det
    s2 = (B*(rho - L3) + A*y) / det
    q2 = np.arctan2(s2, c2)

    return q1, q2, q3, True


if __name__ == "__main__":
    q1, q2, q3, alcanzable = cinematica_inversa_pata_alg(x, y, z, L1, L2, L3, L4, L5, codo="arriba")

    print(f"{'='*50}")
    print(f"Objetivo: x = {x}  y = {y}  z = {z}")
    print(f"{'='*50}")

    if not alcanzable:
        print("No alcanzable")
    else:
        q1d, q2d, q3d = np.degrees([q1, q2, q3])
        print(f"q1 = {q1d:.2f}°  q2 = {q2d:.2f}°  q3 = {q3d:.2f}°")
        for aviso in joint_limit_warnings(q1d, q2d, q3d):
            print(f"Advertencia: {aviso}")

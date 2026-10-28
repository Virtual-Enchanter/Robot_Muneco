"""
Cinemática inversa de la pata del robot bípedo.

Implementación geométrica (seno/coseno explícitos, roll-pitch-pitch
con offset DH). Modelo de 5 longitudes (L1..L5), sin articulación
fantasma ni L6. Pata IZQUIERDA.
"""

import numpy as np

from robot_kinematics.cinematica_directa_der_izq import L1, L2, L3, L4, L5
from robot_kinematics.kinem_common import (
    joint_limit_warnings,
)

x, y, z = L2 + L3 + L4 + L5, 0.0, -L1   # objetivo de prueba (q1=q2=q3=0)


def cinematica_inversa_pata_geom(x, y, z, L1, L2, L3, L4, L5, codo="arriba"):
    """
    Resuelve q1, q2, q3 [rad] por el método geométrico (plano XZ para
    la cadera roll, plano del "brazo" para rodilla y cadera pitch),
    usando seno/coseno en vez de atan2 con offset.

    codo="arriba" -> sin(q3) positivo
    codo="abajo"  -> sin(q3) negativo

    Retorna (q1, q2, q3, alcanzable).
    """
    # Paso 1: traslación al hombro (plano XZ)
    xp = x - L2
    zp = z + L1                                    # <- IZQUIERDA: z + L1
    rho = np.sqrt(xp**2 + zp**2)
    if rho < 1e-12:
        return None, None, None, False

    # Paso 2: q1 (cadera roll)
    c1 = xp / rho
    s1 = -zp / rho                                 # <- IZQUIERDA: -zp
    q1 = np.arctan2(s1, c1)

    # Paso 3: posición del "brazo" (plano del muslo) y q3
    x_arm = rho - L3
    y_arm = y

    c3 = (x_arm**2 + y_arm**2 - L4**2 - L5**2) / (2*L4*L5)
    if abs(c3) > 1 + 1e-9:
        return None, None, None, False
    c3 = np.clip(c3, -1.0, 1.0)

    s3 = np.sqrt(1 - c3**2) * (1 if codo == "arriba" else -1)
    q3 = np.arctan2(s3, c3)

    # Paso 4: q2 (cadera pitch)
    k1 = L4 + L5*np.cos(q3)
    k2 = L5*np.sin(q3)
    denom2 = k1**2 + k2**2
    c2 = (k1*x_arm - k2*y_arm) / denom2
    s2 = (k2*x_arm + k1*y_arm) / denom2
    q2 = np.arctan2(s2, c2)

    return q1, q2, q3, True


if __name__ == "__main__":
    q1, q2, q3, alcanzable = cinematica_inversa_pata_geom(x, y, z, L1, L2, L3, L4, L5, codo="arriba")

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

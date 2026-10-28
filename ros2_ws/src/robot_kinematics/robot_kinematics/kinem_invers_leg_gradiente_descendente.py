"""
Cinemática inversa de la pata del robot bípedo.

Implementación por Método del Jacobiano (Gradiente Descendente iterativo).
Modelo de 5 parámetros (L1 a L5), 3 Grados de Libertad (q1, q2, q3).
Pata IZQUIERDA.
"""

import numpy as np

# Dimensiones físicas (mm)
from robot_kinematics.cinematica_directa_der_izq import L1, L2, L3, L4, L5
from robot_kinematics.kinem_common import (
    dh_matrix,
    joint_limit_warnings,
    q1_max,
    q1_min,
    q2_max,
    q2_min,
    q3_max,
    q3_min,
)


def calcular_jacobiano_rad(q_rad, L1, L2, L3, L4, L5):
    q1, q2, q3 = q_rad

    A01 = dh_matrix(0,  d=-L1, a=L2, alpha=-np.pi/2)
    A12 = dh_matrix(q1, d=0,   a=L3, alpha=np.pi/2)
    A23 = dh_matrix(q2, d=0,   a=L4, alpha=np.pi)
    A34 = dh_matrix(q3, d=0,   a=L5, alpha=0)

    T01 = A01
    T02 = T01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34

    p_e = T04[0:3, 3]

    J1 = np.cross(T01[0:3, 2], (p_e - T01[0:3, 3]))
    J2 = np.cross(T02[0:3, 2], (p_e - T02[0:3, 3]))
    J3 = np.cross(T03[0:3, 2], (p_e - T03[0:3, 3]))

    return np.column_stack((J1, J2, J3)), p_e


def cinematica_inversa_gradiente(objetivo, q_inicial_grados, alpha=5e-5, tolerancia=0.01, max_iter=10000):
    q_actual = np.radians(q_inicial_grados)

    q_min_rad = np.radians([q1_min, q2_min, q3_min])
    q_max_rad = np.radians([q1_max, q2_max, q3_max])

    # FRENO DE EMERGENCIA: Máximo giro permitido por iteración
    max_salto_rad = np.radians(2.0)
    k = 0.1

    for iteracion in range(max_iter):
        J, p_actual = calcular_jacobiano_rad(q_actual, L1, L2, L3, L4, L5)

        error = objetivo - p_actual
        distancia = np.linalg.norm(error)

        if distancia < tolerancia:
            return np.degrees(q_actual), p_actual, True, iteracion

        salto_sugerido = alpha * np.dot(J.T, error)

        magnitud_salto = np.linalg.norm(salto_sugerido)
        tope = min(max_salto_rad, k * np.radians(distancia))
        if magnitud_salto > tope:
            salto_sugerido = salto_sugerido * (tope / magnitud_salto) if tope > 0 else salto_sugerido * 0

        q_actual = q_actual + salto_sugerido
        q_actual = np.clip(q_actual, q_min_rad, q_max_rad)

    return np.degrees(q_actual), p_actual, False, max_iter


def cinematica_inversa_pata_grad(x, y, z,
                                 q1_seed_deg, q2_seed_deg, q3_seed_deg):
    """
    Misma interfaz que los demás métodos (la usa ik_node): punto
    (x, y, z) [mm] y semilla [°].

    Retorna (q1, q2, q3, alcanzable) con los ángulos en radianes.
    alcanzable es False si no converge dentro de max_iter.
    """
    q_deg, _, alcanzable, _ = cinematica_inversa_gradiente(
        np.array([x, y, z], dtype=float),
        np.array([q1_seed_deg, q2_seed_deg, q3_seed_deg], dtype=float))

    if not alcanzable:
        return None, None, None, False

    q1, q2, q3 = np.radians(q_deg)
    return q1, q2, q3, True


# ===============================================
# EJECUCIÓN Y PRUEBAS
# ===============================================
if __name__ == "__main__":
    objetivo_deseado = np.array([L2 + L3 + L4 + L5, 0.0, -L1])   # q1=q2=q3=0
    angulos_iniciales = np.array([10, 10, 10])

    angulos_finales, posicion_alcanzada, alcanzable, iteraciones = cinematica_inversa_gradiente(
        objetivo_deseado,
        angulos_iniciales
    )

    q1d, q2d, q3d = angulos_finales

    print(f"{'='*50}")
    print(f"Objetivo cartesiano: {np.round(objetivo_deseado, 4)}")
    print(f"{'='*50}")

    if not alcanzable:
        print(f"Límite alcanzado ({iteraciones} iter). El objetivo requiere más tiempo o es inalcanzable.")
    else:
        print(f"¡Éxito! Objetivo alcanzado en la iteración {iteraciones}")

    print(f"q1 = {q1d:.2f}°  q2 = {q2d:.2f}°  q3 = {q3d:.2f}°")
    print("Posición final:", np.round(posicion_alcanzada, 4))
    print("Error residual (mm):", np.max(np.abs(posicion_alcanzada - objetivo_deseado)))

    for aviso in joint_limit_warnings(q1d, q2d, q3d):
        print(f"Advertencia: {aviso}")

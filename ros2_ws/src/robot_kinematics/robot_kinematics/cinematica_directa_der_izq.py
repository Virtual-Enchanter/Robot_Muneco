"""
Cinemática directa del muñeco bípedo.

Implementación manual mediante Denavit-Hartenberg.
Los ángulos de entrada se manejan en grados y las longitudes en mm.
Modelo de 5 longitudes (L1..L5), sin articulación fantasma ni L6.

Tabla DH pierna DERECHA:

        theta   d    a     alpha
  0A1    0     L1    L2     90°
  1A2    q1    0     L3    -90°
  2A3    q2    0     L4     180°
  3A4    q3    0     L5     0

Tabla DH pierna IZQUIERDA (espejo en Z: cambian L1 y los alpha de 0A1 y 1A2):

        theta   d    a     alpha
  0A1    0    -L1    L2    -90°
  1A2    q1    0     L3     90°
  2A3    q2    0     L4     180°
  3A4    q3    0     L5     0

Sistema de referencia fijo {0}: origen en Base_link del URDF
(urdf_bipedo/Muneco_Bipedo_URDF_V3.urdf), con

    x0 = -Y del Base_link  (hacia abajo, a lo largo de la pierna)
    y0 = +X del Base_link
    z0 = +Z del Base_link  (lateral: +z pierna derecha, -z izquierda)

Como ambas tablas son espejo una de la otra, para los mismos
(q1, q2, q3) la posición del pie derecho es la del izquierdo con z
cambiada de signo.
"""

import numpy as np
from robot_kinematics.kinem_common import (
    L1,
    L2,
    L3,
    L4,
    L5,
    dh_matrix,
)


# ----------------------------------------------------------------------
# Parámetros geométricos [mm]
# ----------------------------------------------------------------------
#
# Medidos sobre el URDF del muñeco en la posición cero, expresados en
# el sistema {0} descrito arriba:
#
#   L1: separación lateral del plano de la pierna (cadera pitch,
#       rodilla y pie están en z = ±90.3 mm).
#   L2: altura de la cadera roll bajo Base_link (48.5 mm).
#   L3: cadera roll -> cadera pitch (102.7 - 48.5 = 54.2 mm).
#   L4: cadera pitch -> rodilla (138.5 mm, muslo).
#   L5: rodilla -> planta del pie (162.0 mm, pierna + pie).
#
# L2 + L3 + L4 + L5 = 403.2 mm, que es la altura de la base en
# muneco_bipedo.urdf.xacro (0.403 m), es decir, el pie toca el suelo
# con q = 0.
#
# Aproximación del modelo de 5 longitudes: en el CAD el eje de la
# cadera roll está en z = ±51.3 mm (no en ±90.3 mm), así que con
# q1 = 0 el modelo coincide exactamente con el URDF, y con q1 != 0
# hay una pequeña diferencia de posición.



# ----------------------------------------------------------------------
# CINEMÁTICA PIERNA DERECHA
# ----------------------------------------------------------------------

def forward_kinematics_right(q):
    """
    Calcula las transformaciones homogéneas de la pierna derecha.

    q:
        [hip_roll, hip_pitch, knee] en grados

    Retorna:
        T01, T02, T03, T04
    """

    if len(q) != 3:
        raise ValueError(
            "La pierna derecha debe recibir exactamente 3 ángulos."
        )

    q1, q2, q3 = np.radians(q)

    # --------------------------------------------------------------
    # Transformaciones DH
    # --------------------------------------------------------------

    A01 = dh_matrix(
        0,
        d=L1,
        a=L2,
        alpha=np.pi / 2
    )

    A12 = dh_matrix(          # hip_roll
        q1,
        d=0,
        a=L3,
        alpha=-np.pi / 2
    )

    A23 = dh_matrix(          # hip_pitch
        q2,
        d=0,
        a=L4,
        alpha=np.pi
    )

    A34 = dh_matrix(          # knee
        q3,
        d=0,
        a=L5,
        alpha=0
    )

    # --------------------------------------------------------------
    # Transformaciones acumuladas
    # --------------------------------------------------------------

    T01 = A01
    T02 = T01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34

    return T01, T02, T03, T04


# ----------------------------------------------------------------------
# CINEMÁTICA PIERNA IZQUIERDA
# ----------------------------------------------------------------------

def forward_kinematics_left(q):
    """
    Calcula las transformaciones homogéneas de la pierna izquierda.

    q:
        [hip_roll, hip_pitch, knee] en grados

    Retorna:
        T01, T02, T03, T04
    """

    if len(q) != 3:
        raise ValueError(
            "La pierna izquierda debe recibir exactamente 3 ángulos."
        )

    q1, q2, q3 = np.radians(q)

    # --------------------------------------------------------------
    # Transformaciones DH (mismas L2..L5 y mismos alpha en 2A3 y 3A4
    # que la pierna derecha; solo se refleja el signo de L1 y los
    # alpha del tramo cadera, 0A1 y 1A2)
    # --------------------------------------------------------------

    A01 = dh_matrix(
        0,
        d=-L1,
        a=L2,
        alpha=-np.pi / 2
    )

    A12 = dh_matrix(          # hip_roll
        q1,
        d=0,
        a=L3,
        alpha=np.pi / 2
    )

    A23 = dh_matrix(          # hip_pitch
        q2,
        d=0,
        a=L4,
        alpha=np.pi
    )

    A34 = dh_matrix(          # knee
        q3,
        d=0,
        a=L5,
        alpha=0
    )

    # --------------------------------------------------------------
    # Transformaciones acumuladas
    # --------------------------------------------------------------

    T01 = A01
    T02 = T01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34

    return T01, T02, T03, T04


# ----------------------------------------------------------------------
# CINEMÁTICA COMPLETA DEL BÍPEDO
# ----------------------------------------------------------------------

def forward_kinematics_biped(q_right, q_left):
    """
    Calcula todas las transformaciones de ambas piernas.

    Retorna:

        {
            "right": (T01, T02, T03, T04),
            "left":  (T01, T02, T03, T04)
        }
    """

    right = forward_kinematics_right(q_right)
    left = forward_kinematics_left(q_left)

    return {
        "right": right,
        "left": left
    }


# ----------------------------------------------------------------------
# Posición
# ----------------------------------------------------------------------

def get_position(T):
    """
    Extrae la posición XYZ de una MTH.
    """

    return T[0:3, 3]


# ----------------------------------------------------------------------
# Rotación
# ----------------------------------------------------------------------

def get_rotation(T):
    """
    Extrae la matriz de rotación 3x3.
    """

    return T[0:3, 0:3]


# ----------------------------------------------------------------------
# Formato para mostrar en la interfaz
# ----------------------------------------------------------------------

def format_matrix(T):
    """
    Convierte una matriz 4x4 en texto para mostrarla en Tkinter.
    """

    return (
        f"{T[0,0]:8.4f}  {T[0,1]:8.4f}  {T[0,2]:8.4f}  {T[0,3]:8.4f}\n"
        f"{T[1,0]:8.4f}  {T[1,1]:8.4f}  {T[1,2]:8.4f}  {T[1,3]:8.4f}\n"
        f"{T[2,0]:8.4f}  {T[2,1]:8.4f}  {T[2,2]:8.4f}  {T[2,3]:8.4f}\n"
        f"{T[3,0]:8.4f}  {T[3,1]:8.4f}  {T[3,2]:8.4f}  {T[3,3]:8.4f}"
    )

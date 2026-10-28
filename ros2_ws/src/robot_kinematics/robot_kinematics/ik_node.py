# ============================================================
# NODO DE CINEMÁTICA INVERSA
# ============================================================
#
# Atiende los cinco métodos de cinemática inversa en un solo proceso.
# Cada método tiene su tópico de objetivo y su tópico de resultado:
#
#   alg   : /robot/ik_target       (Point)         -> /robot/ik_result
#   newton: /robot/ik_newton_target (IKJacobTarget) -> /robot/ik_newton_result
#   des   : /robot/ik_des_target   (Point)         -> /robot/ik_des_result
#   geom  : /robot/ik_geom_target  (Point)         -> /robot/ik_geom_result
#   grad  : /robot/ik_grad_target  (IKJacobTarget) -> /robot/ik_grad_result
#
# Los métodos iterativos (newton, grad) usan la semilla del mensaje
# IKJacobTarget. Cada método tiene su propio grupo de callbacks, así
# que uno lento (gradiente descendente) no bloquea a los demás.
#
# Los métodos están escritos para la pierna IZQUIERDA. La pierna
# derecha es su espejo en z: con leg_side='right' se resuelve el
# objetivo (x, y, -z) con el mismo método.
#
# ============================================================

import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point
from robot_interfaces.msg import IKJacobTarget, IKResult

from robot_kinematics.kinem_common import (
    L1, L2, L3, L4, L5,
    joint_limit_warnings,
)
from robot_kinematics.kinem_invers_leg_algebraico_izq import (
    cinematica_inversa_pata_alg,
)
from robot_kinematics.kinem_invers_leg_Geometrico_izq import (
    cinematica_inversa_pata_geom,
)
from robot_kinematics.kinem_invers_leg_mth_desacople_izq import (
    cinematica_inversa_pata_des_xyz,
)
from robot_kinematics.kinem_invers_leg_Newton_izq import (
    cinematica_inversa_pata_newton,
)
from robot_kinematics.kinem_invers_leg_gradiente_descendente import (
    cinematica_inversa_pata_grad,
)


# Solver de cada método: recibe el mensaje y la z ya ajustada a la pierna.
# Devuelve (q1, q2, q3, alcanzable) con q en radianes.

def _solve_alg(msg, z):
    return cinematica_inversa_pata_alg(msg.x, msg.y, z, L1, L2, L3, L4, L5)


def _solve_geom(msg, z):
    return cinematica_inversa_pata_geom(msg.x, msg.y, z, L1, L2, L3, L4, L5)


def _solve_des(msg, z):
    return cinematica_inversa_pata_des_xyz(msg.x, msg.y, z)


def _solve_newton(msg, z):
    return cinematica_inversa_pata_newton(
        msg.x, msg.y, z, msg.q1_seed, msg.q2_seed, msg.q3_seed)


def _solve_grad(msg, z):
    return cinematica_inversa_pata_grad(
        msg.x, msg.y, z, msg.q1_seed, msg.q2_seed, msg.q3_seed)


# Tabla de métodos: tópico de objetivo, tipo de mensaje, tópico de resultado,
# solver y si el método necesita semilla (para el mensaje de error).
METHODS = {
    'alg': {
        'target': '/robot/ik_target', 'msg': Point,
        'result': '/robot/ik_result', 'solve': _solve_alg, 'seeded': False,
    },
    'newton': {
        'target': '/robot/ik_newton_target', 'msg': IKJacobTarget,
        'result': '/robot/ik_newton_result', 'solve': _solve_newton,
        'seeded': True,
    },
    'des': {
        'target': '/robot/ik_des_target', 'msg': Point,
        'result': '/robot/ik_des_result', 'solve': _solve_des, 'seeded': False,
    },
    'geom': {
        'target': '/robot/ik_geom_target', 'msg': Point,
        'result': '/robot/ik_geom_result', 'solve': _solve_geom, 'seeded': False,
    },
    'grad': {
        'target': '/robot/ik_grad_target', 'msg': IKJacobTarget,
        'result': '/robot/ik_grad_result', 'solve': _solve_grad, 'seeded': True,
    },
}


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node')

        self.declare_parameter('leg_side', 'left')
        self.leg_side = (
            self.get_parameter('leg_side')
            .get_parameter_value()
            .string_value
            .lower()
        )

        self.result_pubs = {}
        for key, spec in METHODS.items():
            # Un grupo por método: sus peticiones se atienden en orden,
            # pero los métodos distintos corren en paralelo.
            group = MutuallyExclusiveCallbackGroup()

            self.create_subscription(
                spec['msg'], spec['target'],
                lambda msg, key=key: self.on_target(key, msg),
                10, callback_group=group)

            self.result_pubs[key] = self.create_publisher(
                IKResult, spec['result'], 10)

        self.get_logger().info(
            f'ik_node listo con {len(METHODS)} métodos '
            f'(pierna {self.leg_side})')

    def on_target(self, key, msg):
        spec = METHODS[key]

        # Objetivo expresado en el sistema de la pierna izquierda.
        z = -msg.z if self.leg_side == 'right' else msg.z

        q1, q2, q3, alcanzable = spec['solve'](msg, z)

        result = IKResult()
        result.reachable = bool(alcanzable)

        if alcanzable:
            result.position = [float(q1), float(q2), float(q3)]

            q_deg = np.degrees([q1, q2, q3])
            for warning in joint_limit_warnings(*q_deg):
                self.get_logger().warn(f'[{key}] {warning}')
        else:
            result.position = []
            detalle = ''
            if spec['seeded']:
                detalle = (
                    f' desde la semilla ({msg.q1_seed:.1f}, '
                    f'{msg.q2_seed:.1f}, {msg.q3_seed:.1f})°')
            self.get_logger().warn(
                f'[{key}] objetivo ({msg.x:.1f}, {msg.y:.1f}, '
                f'{msg.z:.1f}) no alcanzable{detalle}.')

        self.result_pubs[key].publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

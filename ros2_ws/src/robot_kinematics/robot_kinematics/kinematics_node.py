# ============================================================
# NODO DE CINEMÁTICA UNIFICADO (robot_kinematics) - MUÑECO
# ============================================================
#
# Un solo proceso que reemplaza a los 6 nodos de los cortes 1 y 2:
#
#   kinematics_node  -> reenvío /robot/command -> /robot/joint_targets
#   ik_node          -> IK algebraica       (/robot/ik_*)
#   ik_des_node      -> IK por desacople    (/robot/ik_des_*)
#   ik_geom_node     -> IK geométrica       (/robot/ik_geom_*)
#   ik_newton_node   -> IK Newton-Raphson   (/robot/ik_newton_*)
#   ik_grad_node     -> IK gradiente desc.  (/robot/ik_grad_*)
#
# Los cinco nodos IK eran el mismo código copiado: solo cambiaban la
# función de cinemática inversa y el prefijo del tópico. Aquí cada
# método es una entrada de la tabla SOLVERS.
#
# Integración (rama feature/integracion-final-optimizaciones):
#   - Santiago Sarria: reenvío + 5 métodos IK en un solo proceso.
#   - Hean Felipe Gómez: un grupo de callbacks POR MÉTODO (antes había
#     un solo grupo para toda la IK) y la matemática común en
#     kinem_common.py (L1..L5, límites, DH, FK y avisos de límites).
#   Su ik_node.py (5 métodos en un proceso) quedó absorbido aquí, así
#   que la arquitectura sigue con UN nodo de cinemática.
#
# Los NOMBRES Y TIPOS DE TÓPICOS NO CAMBIAN, así que la tabla
# IK_METHODS de teleop_node sigue funcionando igual.
#
# Parámetros:
#   num_joints  (int, 3)        juntas que se reenvían a control.
#   leg_side    (str, 'left')   pierna que se está probando. Los
#                               módulos resuelven en el sistema de la
#                               pierna izquierda; para la derecha se
#                               refleja z (igual que los nodos antiguos).
#   enable_ik   (bool, True)    activa los resolutores IK.
#   ik_methods  (str[], todos)  subconjunto de métodos a activar.
#
# Concurrencia (ejecutor multihilo):
#   - El reenvío de /robot/command tiene su propio grupo: nunca queda
#     bloqueado mientras un método iterativo resuelve.
#   - Cada método IK tiene su propio grupo: el gradiente descendente
#     (hasta 10000 iteraciones) no bloquea a los otros cuatro.
#   Hilos = 1 (reenvío) + número de métodos activos.
# ============================================================

from functools import partial

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.msg import IKJacobTarget, IKResult, JointTarget, RobotCommand
from robot_kinematics import kinem_invers_leg_algebraico_izq as alg
from robot_kinematics import kinem_invers_leg_Geometrico_izq as geom
from robot_kinematics import kinem_invers_leg_gradiente_descendente as grad
from robot_kinematics import kinem_invers_leg_mth_desacople_izq as des
from robot_kinematics import kinem_invers_leg_Newton_izq as newton
from robot_kinematics.kinem_common import (
    L1, L2, L3, L4, L5,
    joint_limit_warnings,
)


# ------------------------------------------------------------
# TABLA DE MÉTODOS DE CINEMÁTICA INVERSA
# ------------------------------------------------------------
#   topic   : prefijo -> /robot/<topic>_target, /robot/<topic>_result
#   seed    : True si el objetivo trae semilla (IKJacobTarget)
#   solve   : función (x, y, z, semilla) -> (q1, q2, q3, alcanzable)
#   fail    : texto cuando no hay solución
# Los avisos de límites salen de kinem_common.joint_limit_warnings,
# la misma función para los cinco métodos.
# ------------------------------------------------------------

SOLVERS = {
    'algebraico': {
        'topic': 'ik',
        'seed': False,
        'solve': lambda x, y, z, _: alg.cinematica_inversa_pata_alg(
            x, y, z, L1, L2, L3, L4, L5),
        'fail': 'no alcanzable.',
    },
    'desacople': {
        'topic': 'ik_des',
        'seed': False,
        'solve': lambda x, y, z, _: des.cinematica_inversa_pata_des_xyz(
            x, y, z),
        'fail': 'no alcanzable.',
    },
    'geometrico': {
        'topic': 'ik_geom',
        'seed': False,
        'solve': lambda x, y, z, _: geom.cinematica_inversa_pata_geom(
            x, y, z, L1, L2, L3, L4, L5),
        'fail': 'no alcanzable.',
    },
    'newton': {
        'topic': 'ik_newton',
        'seed': True,
        'solve': lambda x, y, z, seed: newton.cinematica_inversa_pata_newton(
            x, y, z, *seed),
        'fail': 'no alcanzable o no convergió desde la semilla',
    },
    'gradiente': {
        'topic': 'ik_grad',
        'seed': True,
        'solve': lambda x, y, z, seed: grad.cinematica_inversa_pata_grad(
            x, y, z, *seed),
        'fail': 'no convergió desde la semilla',
    },
}


class KinematicsNode(Node):

    def __init__(self):
        super().__init__('kinematics_node')

        # ----------------------------------------------------
        # PARÁMETROS
        # ----------------------------------------------------
        self.declare_parameter('num_joints', 3)
        self.declare_parameter('leg_side', 'left')
        self.declare_parameter('enable_ik', True)
        self.declare_parameter('ik_methods', list(SOLVERS.keys()))

        # n = cantidad de juntas: 3 (una pata) o 6 (robot completo).
        self.n = int(self.get_parameter('num_joints').value)
        self.leg_side = str(self.get_parameter('leg_side').value).lower()
        enable_ik = bool(self.get_parameter('enable_ik').value)
        methods = [str(m) for m in self.get_parameter('ik_methods').value]

        # ----------------------------------------------------
        # REENVÍO teleop -> control (antes: kinematics_node)
        # Grupo propio: no espera a ningún método IK.
        # ----------------------------------------------------
        self._relay_group = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription(
            RobotCommand, '/robot/command', self.on_cmd, 10,
            callback_group=self._relay_group)
        self.pub = self.create_publisher(
            JointTarget, '/robot/joint_targets', 10)

        # ----------------------------------------------------
        # CINEMÁTICA INVERSA (antes: 5 nodos ik_*)
        # ----------------------------------------------------
        self.ik_pubs = {}
        if enable_ik:
            for key in methods:
                if key not in SOLVERS:
                    self.get_logger().error(
                        f"Método IK desconocido '{key}'. "
                        f'Opciones: {list(SOLVERS)}')
                    continue
                self._create_ik_interfaces(key)

        activos = list(self.ik_pubs) if self.ik_pubs else 'ninguno'
        self.get_logger().info(
            f'kinematics_node listo | juntas={self.n} | '
            f'pierna={self.leg_side} | IK={activos}')

    # --------------------------------------------------------
    # Creación de interfaces de un método IK
    # --------------------------------------------------------

    def _create_ik_interfaces(self, key):
        cfg = SOLVERS[key]
        topic = cfg['topic']

        # Un grupo por método: las peticiones de un mismo método se
        # atienden en orden, pero métodos distintos corren en paralelo.
        group = MutuallyExclusiveCallbackGroup()

        msg_type = IKJacobTarget if cfg['seed'] else Point
        self.create_subscription(
            msg_type, f'/robot/{topic}_target',
            partial(self.on_ik_target, key), 10,
            callback_group=group)

        self.ik_pubs[key] = self.create_publisher(
            IKResult, f'/robot/{topic}_result', 10)

    # --------------------------------------------------------
    # Reenvío de la orden del operador
    # --------------------------------------------------------

    def on_cmd(self, msg):
        # TODO: aquí va la IK real/trayectorias. Hoy copiamos
        # position/velocity y recortamos al tamaño n.
        target = JointTarget()
        target.position = list(msg.position[:self.n])
        target.velocity = list(msg.velocity[:self.n])
        self.pub.publish(target)

    # --------------------------------------------------------
    # Callbacks de cinemática inversa
    # --------------------------------------------------------

    def on_ik_target(self, key, msg):
        cfg = SOLVERS[key]
        seed = (msg.q1_seed, msg.q2_seed, msg.q3_seed) if cfg['seed'] else None

        # Objetivo expresado en el sistema de la pierna izquierda.
        z = -msg.z if self.leg_side == 'right' else msg.z

        q1, q2, q3, alcanzable = cfg['solve'](msg.x, msg.y, z, seed)

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
            if seed is not None:
                detalle = ' ({:.1f}, {:.1f}, {:.1f})°'.format(*seed)
            self.get_logger().warn(
                f'[{key}] Objetivo ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}) '
                f"{cfg['fail']}{detalle}")

        self.ik_pubs[key].publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    # 1 hilo para el reenvío + 1 por cada método IK activo.
    executor = MultiThreadedExecutor(num_threads=1 + max(1, len(node.ik_pubs)))
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

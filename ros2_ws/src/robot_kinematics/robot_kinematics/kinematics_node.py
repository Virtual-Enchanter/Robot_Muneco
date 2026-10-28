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
# método es una entrada de la tabla SOLVERS (misma estructura que en el
# repositorio del bípedo; solo cambian los módulos, porque el muñeco
# tiene sus propias longitudes L1..L5).
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
# Ejecutor multihilo con dos grupos de callbacks: el reenvío de
# /robot/command nunca queda bloqueado mientras un método iterativo
# (Newton o gradiente, hasta 10000 iteraciones) está resolviendo.
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


# ------------------------------------------------------------
# TABLA DE MÉTODOS DE CINEMÁTICA INVERSA
# ------------------------------------------------------------
#   topic   : prefijo -> /robot/<topic>_target, /robot/<topic>_result
#   seed    : True si el objetivo trae semilla (IKJacobTarget)
#   solve   : función (x, y, z, semilla) -> (q1, q2, q3, alcanzable)
#   warnings: función de advertencias de límites del mismo módulo
#   fail    : texto cuando no hay solución
# ------------------------------------------------------------

SOLVERS = {
    'algebraico': {
        'topic': 'ik',
        'seed': False,
        'solve': lambda x, y, z, _: alg.cinematica_inversa_pata_alg(
            x, y, z, alg.L1, alg.L2, alg.L3, alg.L4, alg.L5),
        'warnings': alg.joint_limit_warnings,
        'fail': 'no alcanzable.',
    },
    'desacople': {
        'topic': 'ik_des',
        'seed': False,
        'solve': lambda x, y, z, _: des.cinematica_inversa_pata_des_xyz(
            x, y, z),
        'warnings': des.joint_limit_warnings,
        'fail': 'no alcanzable.',
    },
    'geometrico': {
        'topic': 'ik_geom',
        'seed': False,
        'solve': lambda x, y, z, _: geom.cinematica_inversa_pata_geom(
            x, y, z, geom.L1, geom.L2, geom.L3, geom.L4, geom.L5),
        'warnings': geom.joint_limit_warnings,
        'fail': 'no alcanzable.',
    },
    'newton': {
        'topic': 'ik_newton',
        'seed': True,
        'solve': lambda x, y, z, seed: newton.cinematica_inversa_pata_newton(
            x, y, z, *seed),
        'warnings': newton.joint_limit_warnings,
        'fail': 'no alcanzable o no convergió desde la semilla',
    },
    'gradiente': {
        'topic': 'ik_grad',
        'seed': True,
        'solve': lambda x, y, z, seed: grad.cinematica_inversa_pata_grad(
            x, y, z, *seed),
        'warnings': grad.joint_limit_warnings,
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

        # Grupos de callbacks: el reenvío y la IK corren por separado.
        self._relay_group = MutuallyExclusiveCallbackGroup()
        self._ik_group = MutuallyExclusiveCallbackGroup()

        # ----------------------------------------------------
        # REENVÍO teleop -> control (antes: kinematics_node)
        # ----------------------------------------------------
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

        msg_type = IKJacobTarget if cfg['seed'] else Point
        self.create_subscription(
            msg_type, f'/robot/{topic}_target',
            partial(self.on_ik_target, key), 10,
            callback_group=self._ik_group)

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
            for warning in cfg['warnings'](*q_deg):
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
    executor = MultiThreadedExecutor(num_threads=2)
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

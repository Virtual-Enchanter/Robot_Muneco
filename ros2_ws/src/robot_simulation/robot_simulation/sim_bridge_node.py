# Puente control -> Gazebo (robot_simulation).
#
# Lleva /robot/joint_commands (JointTarget, publicado por control_node)
# al controlador de posición de Gazebo:
#     /joint_group_position_controller/commands (Float64MultiArray)
#
# CORRECCIÓN respecto al esqueleto anterior: el controlador de Gazebo
# (config/controllers.yaml) maneja las 6 articulaciones en este orden
#     [R_hip_roll, R_hip_pitch, R_knee, L_hip_roll, L_hip_pitch, L_knee]
# y rechaza cualquier comando que no tenga exactamente 6 valores. El
# esqueleto reenviaba los 3 valores de UNA pierna tal cual, así que
# Gazebo nunca los aplicaba. Ahora cada valor se coloca en su posición
# según el nombre del joint, y la otra pierna conserva su último valor.
#
# REINICIO DEL ROBOT (servicio /sim/reset_robot, std_srvs/Trigger):
# el muñeco no está fijo al mundo, así que al mover las piernas puede
# caerse. El servicio lo deja como recién creado:
#   1. Las 6 articulaciones vuelven a 0 rad (home).
#   2. Espera a que lleguen a 0 y queden quietas (/joint_states de
#      Gazebo). Si el modelo se mueve antes, el cuerpo conserva la
#      velocidad de las piernas estirándose y cae torcido otra vez.
#   3. El modelo vuelve a la pose del spawn (spawn_x/y/z, de pie) con
#      el servicio set_pose de Gazebo. Se llama con la herramienta de
#      línea de comandos: 'ign' en Fortress (Humble) y 'gz' en
#      Harmonic (Jazzy); se prueban las dos.
# Lo usa el botón "Reiniciar robot" de teleop_node. El ejecutor es
# multihilo para que /joint_states siga llegando durante la espera.
#
# Parámetros:
#   joint_names        joints que llegan en /robot/joint_commands
#                      (los de la pierna que se está probando).
#   controller_joints  joints del controlador, en su orden.
#   world_name         mundo de Gazebo (empty.sdf -> 'empty').
#   model_name         nombre del modelo en Gazebo ('muneco').
#   spawn_x/y/z        pose inicial del modelo [m] (misma del spawn).
#   settle_timeout     espera máxima a que las piernas queden en 0 [s].
import os
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.msg import JointTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

CONTROLLER_JOINTS = [
    'Right_Hip_Roll_Joint',
    'Right_Hip_Pitch_Joint',
    'Right_Knee_Joint',
    'Left_Hip_Roll_Joint',
    'Left_Hip_Pitch_Joint',
    'Left_Knee_Joint',
]

# Articulaciones "en home y quietas" para poder mover el modelo.
HOME_TOL_RAD = 0.02
QUIET_TOL_RAD_S = 0.05

# Herramienta de línea de comandos de Gazebo y paquete de mensajes.
GZ_CLI = {
    'gz': 'gz.msgs',          # Gazebo Harmonic (ROS 2 Jazzy)
    'ign': 'ignition.msgs',   # Gazebo Fortress (ROS 2 Humble)
}


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        self.declare_parameter('joint_names', CONTROLLER_JOINTS[:3])
        self.declare_parameter('controller_joints', CONTROLLER_JOINTS)
        self.declare_parameter('world_name', 'empty')
        self.declare_parameter('model_name', 'muneco')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.8)
        self.declare_parameter('settle_timeout', 3.0)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.controller_joints = list(
            self.get_parameter('controller_joints').value)
        self.world_name = str(self.get_parameter('world_name').value)
        self.model_name = str(self.get_parameter('model_name').value)
        self.spawn_pose = [
            float(self.get_parameter(p).value)
            for p in ('spawn_x', 'spawn_y', 'spawn_z')
        ]
        self.settle_timeout = float(
            self.get_parameter('settle_timeout').value)

        # Índice de cada joint de entrada dentro del vector del controlador.
        self.index = []
        for name in self.joint_names:
            if name in self.controller_joints:
                self.index.append(self.controller_joints.index(name))
            else:
                self.index.append(None)
                self.get_logger().error(
                    f"'{name}' no está en controller_joints; se ignora.")

        # Última posición conocida de cada joint del controlador [rad].
        # on_cmd y on_reset corren en hilos distintos: se protege con
        # un lock.
        self.command = [0.0] * len(self.controller_joints)
        self.lock = threading.Lock()

        # Estado real de Gazebo: nombre -> (posición, velocidad).
        self.joint_state = {}

        self.sub = self.create_subscription(
            JointTarget, '/robot/joint_commands', self.on_cmd, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)

        # Reinicio del robot (botón de teleop_node). Grupos separados:
        # mientras on_reset espera, /joint_states sigue actualizándose.
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.reset_srv = self.create_service(
            Trigger, '/sim/reset_robot', self.on_reset,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f'sim_bridge listo: {self.joint_names} -> '
            f'{len(self.controller_joints)} joints del controlador')

    def on_cmd(self, msg):
        if len(msg.position) != len(self.joint_names):
            self.get_logger().warn(
                f'Se esperaban {len(self.joint_names)} posiciones y '
                f'llegaron {len(msg.position)}',
                throttle_duration_sec=5.0)

        with self.lock:
            for idx, value in zip(self.index, msg.position):
                if idx is not None:
                    self.command[idx] = float(value)

        self.publish_command()

    def publish_command(self):
        out = Float64MultiArray()
        with self.lock:
            out.data = list(self.command)
        self.pub.publish(out)

    def on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.joint_state[name] = (pos, vel)

    # --------------------------------------------------------
    # Reinicio: home + volver a la pose del spawn
    # --------------------------------------------------------

    def on_reset(self, request, response):
        # 1. Las dos piernas a home (0 rad).
        with self.lock:
            self.command = [0.0] * len(self.controller_joints)
        self.publish_command()

        # 2. Esperar a que estén en 0 y quietas.
        quietas, espera = self.wait_for_home()
        if not quietas:
            self.get_logger().warn(
                f'Las piernas no quedaron quietas en {espera:.1f} s; '
                'se mueve el modelo igual.')

        # 3. El modelo de vuelta a la pose inicial, de pie.
        ok, detalle = self.teleport_to_spawn()

        # Se repite home por si control_node alcanzó a reenviar el
        # objetivo anterior mientras se movía el modelo.
        self.publish_command()

        response.success = ok
        response.message = detalle
        if ok:
            self.get_logger().info(f'Robot reiniciado: {detalle}')
        else:
            self.get_logger().warn(f'Reinicio incompleto: {detalle}')
        return response

    def wait_for_home(self):
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.settle_timeout:
            estados = [self.joint_state.get(j) for j in self.controller_joints]
            if all(e is not None for e in estados):
                en_home = max(abs(p) for p, _ in estados) < HOME_TOL_RAD
                quietas = max(abs(v) for _, v in estados) < QUIET_TOL_RAD_S
                if en_home and quietas:
                    # Un instante más para que el cuerpo también se asiente.
                    time.sleep(0.2)
                    return True, time.monotonic() - t0
            time.sleep(0.05)
        return False, time.monotonic() - t0

    def teleport_to_spawn(self):
        x, y, z = self.spawn_pose
        req = (
            f'name: "{self.model_name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
        )
        service = f'/world/{self.world_name}/set_pose'

        # En Humble se prueba primero 'ign' (Fortress); en Jazzy, 'gz'.
        orden = ['ign', 'gz'] if os.environ.get('ROS_DISTRO') == 'humble' \
            else ['gz', 'ign']

        for cli in orden:
            exe = shutil.which(cli)
            if exe is None:
                continue
            msgs = GZ_CLI[cli]
            cmd = [
                exe, 'service', '-s', service,
                '--reqtype', f'{msgs}.Pose',
                '--reptype', f'{msgs}.Boolean',
                '--timeout', '2000',
                '--req', req,
            ]
            try:
                t0 = time.monotonic()
                out = subprocess.run(
                    cmd, capture_output=True, text=True, timeout=6)
            except (subprocess.TimeoutExpired, OSError) as e:
                self.get_logger().warn(f'{cli} service falló: {e}')
                continue

            # La herramienta devuelve 0 aunque falle: se revisa la respuesta.
            if 'data: true' in out.stdout:
                return True, (
                    f'modelo "{self.model_name}" en ({x:.2f}, {y:.2f}, '
                    f'{z:.2f}) m vía {cli} '
                    f'({1000 * (time.monotonic() - t0):.0f} ms)')

        return False, (
            f'Gazebo no respondió en {service}. Las articulaciones '
            'quedaron en home, pero el modelo no se movió '
            '(¿está abierto Gazebo?).')


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    executor = MultiThreadedExecutor(num_threads=3)
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

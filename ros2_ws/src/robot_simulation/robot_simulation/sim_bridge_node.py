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
# Parámetros:
#   joint_names        joints que llegan en /robot/joint_commands
#                      (los de la pierna que se está probando).
#   controller_joints  joints del controlador, en su orden.
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_interfaces.msg import JointTarget
from std_msgs.msg import Float64MultiArray

CONTROLLER_JOINTS = [
    'Right_Hip_Roll_Joint',
    'Right_Hip_Pitch_Joint',
    'Right_Knee_Joint',
    'Left_Hip_Roll_Joint',
    'Left_Hip_Pitch_Joint',
    'Left_Knee_Joint',
]


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        self.declare_parameter('joint_names', CONTROLLER_JOINTS[:3])
        self.declare_parameter('controller_joints', CONTROLLER_JOINTS)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.controller_joints = list(
            self.get_parameter('controller_joints').value)

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
        self.command = [0.0] * len(self.controller_joints)

        self.sub = self.create_subscription(
            JointTarget, '/robot/joint_commands', self.on_cmd, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)

        self.get_logger().info(
            f'sim_bridge listo: {self.joint_names} -> '
            f'{len(self.controller_joints)} joints del controlador')

    def on_cmd(self, msg):
        if len(msg.position) != len(self.joint_names):
            self.get_logger().warn(
                f'Se esperaban {len(self.joint_names)} posiciones y '
                f'llegaron {len(msg.position)}',
                throttle_duration_sec=5.0)

        for idx, value in zip(self.index, msg.position):
            if idx is not None:
                self.command[idx] = float(value)

        out = Float64MultiArray()
        out.data = list(self.command)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Launch completo para probar UNA pierna del muñeco bípedo (RViz, sin Gazebo).

Reemplaza a bringup_der.launch.py y bringup_izq.launch.py, que eran el
mismo archivo (~990 líneas entre los dos) salvo por los nombres de los
joints y el lado de la pierna.

Integración: Hean Felipe Gómez hizo la misma unificación en su rama
(bringup.launch.py leg:=right|left). Aquí se conserva un solo launch y
se aceptan los dos nombres de pierna: izq|der y left|right.

Uso:
    ros2 launch robot_bringup bringup_pierna.launch.py leg:=izq
    ros2 launch robot_bringup bringup_pierna.launch.py leg:=der use_rviz:=false
    ros2 launch robot_bringup bringup_pierna.launch.py leg:=right   # alias

Para la simulación en Gazebo usar bringup_muneco.launch.py.

Este launch inicia:

    1. robot_state_publisher
       - Carga el muñeco completo (urdf_muneco/muneco_bipedo_v3.urdf).
    1.1 joint_state_publisher
       - Completa en /joint_states los 6 joints a partir de
         /joint_states_pata (los 3 de esta pierna, publicados por
         control_node); los de la otra pierna quedan en 0.

    2. kinematics_node (nodo unificado)
       - /robot/command -> /robot/joint_targets.
       - Cinemática inversa por los 5 métodos (/robot/ik*_target ->
         /robot/ik*_result). Para la pierna derecha refleja z.

    3. control_node
       - /robot/joint_targets -> límites -> /robot/joint_states,
         /joint_states_pata y /robot/joint_commands.

    4. sim_bridge
       - /robot/joint_commands (3 joints) -> vector de 6 posiciones en
         /joint_group_position_controller/commands.

    5. teleop_node
       - Interfaz gráfica de la pierna seleccionada. Publica
         /robot/command y /servo_commands (ESP32 con micro-ROS).

    6. RViz2 (use_rviz:=true)

IMPORTANTE:
    Una pierna a la vez: las dos piernas usan los mismos nombres de
    nodos y tópicos (no hay namespaces).

    La ESP32 (firmware esp32_servos) se comunica por micro-ROS con su
    propio micro-ROS agent; no se inicia desde este launch.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# ======================================================================
# CONFIGURACIÓN POR PIERNA
# ======================================================================
# Nombres de los joints definidos realmente en el URDF del muñeco.

PIERNAS = {
    'der': {
        'leg_side': 'right',
        'joints': [
            'Right_Hip_Roll_Joint',
            'Right_Hip_Pitch_Joint',
            'Right_Knee_Joint',
        ],
    },
    'izq': {
        'leg_side': 'left',
        'joints': [
            'Left_Hip_Roll_Joint',
            'Left_Hip_Pitch_Joint',
            'Left_Knee_Joint',
        ],
    },
}

# Nombres en inglés de la rama de Hean Felipe (bringup.launch.py).
PIERNAS['right'] = PIERNAS['der']
PIERNAS['left'] = PIERNAS['izq']


def _nodos_de_la_pierna(context):
    leg = LaunchConfiguration('leg').perform(context).lower()
    if leg not in PIERNAS:
        raise RuntimeError(
            f"leg:='{leg}' no es válido. Use leg:=izq|der (o left|right).")
    pierna = PIERNAS[leg]

    n = int(LaunchConfiguration('num_joints').perform(context))

    pkg_desc = get_package_share_directory('robot_description')
    urdf_file = os.path.join(
        pkg_desc, 'urdf', 'urdf_muneco', 'muneco_bipedo_v3.urdf')
    rviz_config = os.path.join(pkg_desc, 'config', 'robot.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    # 1. Modelo del muñeco completo.
    description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # 1.1 Completa los 6 joints (los de la otra pierna en 0).
    joints_muneco = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'source_list': ['/joint_states_pata'],
            'rate': 30,
        }],
    )

    # 2. Cinemática (reenvío + IK) en un solo proceso.
    kinematics = Node(
        package='robot_kinematics',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[{
            'num_joints': n,
            'leg_side': pierna['leg_side'],
        }],
    )

    # 3. Control: su /joint_states se renombra para que
    # joint_state_publisher lo complete con la otra pierna.
    control = Node(
        package='robot_control',
        executable='control_node',
        name='control_node',
        remappings=[('/joint_states', '/joint_states_pata')],
        parameters=[{
            'num_joints': n,
            'joint_names': pierna['joints'],
        }],
    )

    # 4. Puente con la simulación (3 joints -> vector del controlador).
    sim = Node(
        package='robot_simulation',
        executable='sim_bridge',
        name='sim_bridge',
        parameters=[{'joint_names': pierna['joints']}],
        output='screen',
    )

    # 5. Interfaz de teleoperación.
    teleop = Node(
        package='robot_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{'num_joints': n, 'leg_side': pierna['leg_side']}],
        output='screen',
    )

    # 6. RViz.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [description, joints_muneco, kinematics, control,
            sim, teleop, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'leg', default_value='izq',
            description='Pierna a probar: izq|der (o left|right)'),
        DeclareLaunchArgument(
            'num_joints', default_value='3',
            description='Juntas por pierna'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Abrir RViz2'),
        OpaqueFunction(function=_nodos_de_la_pierna),
    ])

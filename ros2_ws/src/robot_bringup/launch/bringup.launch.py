"""
Launch para probar UNA pierna del robot bípedo.

Uso:

    ros2 launch robot_bringup bringup.launch.py leg:=right
    ros2 launch robot_bringup bringup.launch.py leg:=left

Inicia:

    1. robot_state_publisher  (modelo completo del muñeco)
    2. joint_state_publisher  (completa los joints de la otra pierna en 0)
    3. kinematics_node        (reenvía /robot/command -> /robot/joint_targets)
    4. ik_node, ik_newton_node, ik_des_node, ik_geom_node, ik_grad_node
    5. control_node           (aplica límites y publica el estado)
    6. sim_bridge             (puente hacia la simulación)
    7. teleop_node            (sliders + cinemática directa de la pierna)
    8. rviz2                  (opcional, argumento use_rviz)

IMPORTANTE:
    Correr una sola pierna a la vez. Ambas usan los mismos nombres de
    nodos y tópicos, así que NO deben ejecutarse en paralelo sin namespaces.

El puente serial con la ESP32 (robot_serial_bridge) no se inicia desde
este launch.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


# ======================================================================
# CONFIGURACIÓN POR PIERNA
# ======================================================================
#
# Nombres de los joints definidos en el URDF de cada pierna. El modelo
# que se carga es el muñeco completo; estos nombres indican qué joints
# controla esta instancia.

LEG_JOINTS = {
    'right': [
        'Right_Hip_Roll_Joint',
        'Right_Hip_Pitch_Joint',
        'Right_Knee_Joint',
    ],
    'left': [
        'Left_Hip_Roll_Joint',
        'Left_Hip_Pitch_Joint',
        'Left_Knee_Joint',
    ],
}


# ======================================================================
# NODOS
# ======================================================================

def ik_node(executable, leg):
    """Nodo de cinemática inversa de un método concreto."""
    return Node(
        package='robot_kinematics',
        executable=executable,
        name=executable,
        parameters=[{'leg_side': leg}],
    )


def launch_setup(context, *args, **kwargs):
    leg = LaunchConfiguration('leg').perform(context)
    if leg not in LEG_JOINTS:
        raise RuntimeError(f"leg debe ser 'right' o 'left', no '{leg}'")

    joint_names = LEG_JOINTS[leg]
    num_joints = LaunchConfiguration('num_joints')

    # Modelo completo del muñeco, procesado con xacro.
    pkg_desc = get_package_share_directory('robot_description')
    xacro_file = os.path.join(
        pkg_desc, 'urdf', 'urdf_bipedo', 'muneco_bipedo.urdf.xacro')
    rviz_config = os.path.join(pkg_desc, 'config', 'robot.rviz')
    robot_description = Command(['xacro ', xacro_file])

    description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description, value_type=str),
        }],
    )

    # control_node solo publica los 3 joints de esta pierna en
    # /joint_states_pata; joint_state_publisher completa /joint_states
    # con los 6 joints del muñeco.
    joints_muneco = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description, value_type=str),
            'source_list': ['/joint_states_pata'],
            'rate': 30,
        }],
    )

    kinematics = Node(
        package='robot_kinematics',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[{'num_joints': num_joints}],
    )

    ik_nodes = [
        ik_node(executable, leg) for executable in (
            'ik_node',
            'ik_newton_node',
            'ik_des_node',
            'ik_geom_node',
            'ik_grad_node',
        )
    ]

    # Los límites articulares viven en control_node.py, no se pasan aquí.
    control = Node(
        package='robot_control',
        executable='control_node',
        name='control_node',
        remappings=[('/joint_states', '/joint_states_pata')],
        parameters=[{
            'num_joints': num_joints,
            'joint_names': joint_names,
        }],
    )

    sim = Node(
        package='robot_simulation',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen',
    )

    teleop = Node(
        package='robot_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{
            'num_joints': num_joints,
            'leg_side': leg,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [
        description,
        joints_muneco,
        kinematics,
        *ik_nodes,
        control,
        sim,
        teleop,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'leg',
            default_value='right',
            description="Pierna a controlar: 'right' o 'left'.",
        ),
        DeclareLaunchArgument(
            'num_joints',
            default_value='3',
            description='Número de articulaciones controladas.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Abrir RViz junto con el resto de nodos.',
        ),
        OpaqueFunction(function=launch_setup),
    ])

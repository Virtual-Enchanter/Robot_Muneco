"""
Muñeco en Gazebo + arquitectura completa (una pierna desde la interfaz).

Responde a la pregunta "con qué tópicos se comunican los datos del control
con la simulación":

  teleop_node --/robot/command--> kinematics_node --/robot/joint_targets-->
  control_node --/robot/joint_commands--> sim_bridge
  --/joint_group_position_controller/commands (6 joints)--> Gazebo
  Gazebo --/joint_states (joint_state_broadcaster)--> robot_state_publisher

Uso:
    ros2 launch robot_bringup bringup_muneco.launch.py                 # pierna izq + GUI
    ros2 launch robot_bringup bringup_muneco.launch.py leg:=der
    ros2 launch robot_bringup bringup_muneco.launch.py teleop:=false   # /robot/command a mano

Solo Gazebo (sin arquitectura): ros2 launch robot_simulation gazebo_muneco.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


PIERNAS = {
    'der': ('right', ['Right_Hip_Roll_Joint', 'Right_Hip_Pitch_Joint',
                      'Right_Knee_Joint']),
    'izq': ('left', ['Left_Hip_Roll_Joint', 'Left_Hip_Pitch_Joint',
                     'Left_Knee_Joint']),
}
PIERNAS['right'] = PIERNAS['der']   # alias (rama de Hean Felipe)
PIERNAS['left'] = PIERNAS['izq']


def _arquitectura(context):
    leg = LaunchConfiguration('leg').perform(context).lower()
    if leg not in PIERNAS:
        raise RuntimeError(
            f"leg:='{leg}' no es válido. Use leg:=izq|der (o left|right).")
    leg_side, joints = PIERNAS[leg]

    kinematics = Node(
        package='robot_kinematics',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[{'num_joints': 3, 'leg_side': leg_side}],
    )

    # En Gazebo el estado real lo publica joint_state_broadcaster en
    # /joint_states; el de control_node se renombra para no pisarlo.
    control = Node(
        package='robot_control',
        executable='control_node',
        name='control_node',
        remappings=[('/joint_states', '/control/joint_states')],
        parameters=[{'num_joints': 3, 'joint_names': joints}],
    )

    # sim_bridge también atiende /sim/reset_robot (botón "Reiniciar
    # robot" de la interfaz): necesita la misma altura del spawn.
    sim = Node(
        package='robot_simulation',
        executable='sim_bridge',
        name='sim_bridge',
        parameters=[{
            'joint_names': joints,
            'spawn_z': float(LaunchConfiguration('z').perform(context)),
        }],
        output='screen',
    )

    teleop = Node(
        package='robot_teleop',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{'num_joints': 3, 'leg_side': leg_side}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    return [kinematics, control, sim, teleop]


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_simulation'),
            'launch', 'gazebo_muneco.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'z': LaunchConfiguration('z'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('leg', default_value='izq',
                              description='Pierna que mueve la interfaz'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='Abrir la interfaz teleop_node'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Ventana de Gazebo'),
        DeclareLaunchArgument('z', default_value='0.8',
                              description='Altura inicial del robot [m] '
                                          '(spawn y reinicio)'),
        gazebo,
        OpaqueFunction(function=_arquitectura),
    ])

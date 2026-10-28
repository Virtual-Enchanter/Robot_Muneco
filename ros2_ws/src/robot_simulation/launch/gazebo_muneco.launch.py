"""
Simulación del muñeco bípedo en Gazebo (ros_gz_sim + gz_ros2_control).

Origen: bringup_muneco.launch.py del workspace muneco_ws (Gazebo
funcionando, compartido por el coordinador). Se mueve a robot_simulation
porque, según el README del repositorio, los launch de Gazebo viven en
este paquete. Cambios respecto al original:

  - Los controladores se cargan en cadena por eventos (cuando termina el
    spawn del robot, luego el joint_state_broadcaster) en lugar de
    esperar tiempos fijos de 8 s y 10 s: el orden queda garantizado y no
    depende de cuánto tarde Gazebo en cada PC.
  - Se exporta la ruta de recursos con los dos nombres de variable
    (GZ_SIM_RESOURCE_PATH y IGN_GAZEBO_RESOURCE_PATH) para que las
    mallas carguen tanto en Fortress (Humble) como en Harmonic (Jazzy).
  - URDF con nombre ASCII: urdf_muneco/muneco_bipedo_v3.urdf.
  - controllers.yaml pasa a robot_simulation/config. Con el
    controller_manager de Jazzy, una ruta de parámetros que contiene
    "robot_description" (…/share/robot_description/config/…) se descarta
    al crear los controladores y estos no cargan; en Humble no afectaba.

Uso (solo Gazebo, sin la arquitectura):
    ros2 launch robot_simulation gazebo_muneco.launch.py
    ros2 launch robot_simulation gazebo_muneco.launch.py gui:=false   # sin ventana

Mover el robot a mano (6 joints, orden de controllers.yaml):
    ros2 topic pub --once /joint_group_position_controller/commands \
      std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PythonExpression)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Ubicación de los paquetes instalados.
    description_share = get_package_share_directory('robot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Gazebo resuelve package://robot_description/... buscando la carpeta
    # PADRE de robot_description dentro de su ruta de recursos.
    gazebo_resource_path = os.path.dirname(description_share)
    for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH'):
        existing = os.environ.get(var, '')
        if existing:
            gazebo_resource_path = f'{gazebo_resource_path}:{existing}'
            break

    set_resource_paths = [
        SetEnvironmentVariable(name=var, value=gazebo_resource_path)
        for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH')
    ]

    urdf_file = os.path.join(
        description_share, 'urdf', 'urdf_muneco', 'muneco_bipedo_v3.urdf')

    # El URDF trae $(find robot_description) en el plugin de Gazebo:
    # xacro lo resuelve a la ruta de config/controllers.yaml.
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str)

    # Gazebo Sim. "-r" arranca la simulación sin pausa; "-s" = solo servidor.
    gz_args = PythonExpression([
        "'-r -v 3 empty.sdf' + ('' if '",
        LaunchConfiguration('gui'), "' == 'true' else ' -s')"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Descripción del robot y TF.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawn del robot desde /robot_description (Gazebo necesita unos
    # segundos para crear el mundo antes de aceptar entidades).
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_muneco',
        output='screen',
        arguments=[
            '-name', 'muneco',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', LaunchConfiguration('z'),
        ],
    )

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_{controller}',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '120',
            ],
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')
    position_controller = spawner('joint_group_position_controller')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Abrir la ventana de Gazebo (false = solo servidor)'),
        DeclareLaunchArgument(
            'z', default_value='0.8',
            description='Altura inicial del robot [m]'),
        *set_resource_paths,
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        # Cadena de carga: spawn -> joint_state_broadcaster -> posición.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[position_controller])),
    ])

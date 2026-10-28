"""
Carga un modelo en Gazebo (gazebo_ros).

Uso:

    ros2 launch robot_description gazebo.launch.py model:=right
    ros2 launch robot_description gazebo.launch.py model:=left
    ros2 launch robot_description gazebo.launch.py model:=bipedo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


# Archivo xacro y nombre de entidad en Gazebo de cada modelo.
MODELS = {
    'right': (
        os.path.join('urdf', 'urdf_der', 'pata_der.urdf.xacro'),
        'pata_der',
    ),
    'left': (
        os.path.join('urdf', 'urdf_izq', 'pata_izq.urdf.xacro'),
        'pata_izq',
    ),
    'bipedo': (
        os.path.join('urdf', 'urdf_bipedo', 'muneco_bipedo.urdf.xacro'),
        'muneco_bipedo',
    ),
}


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    if model not in MODELS:
        raise RuntimeError(
            f"model debe ser 'right', 'left' o 'bipedo', no '{model}'")

    xacro_path, entity = MODELS[model]
    pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg, xacro_path)
    robot_description = Command(['xacro ', xacro_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description, value_type=str),
        }],
    )

    tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'Base_link', 'base_footprint'],
        output='screen',
    )

    # Gazebo (gazebo_ros spawn_entity) no procesa xacro: se saca el robot del
    # topico /robot_description (ya expandido por robot_state_publisher) en
    # vez de pasarle el archivo .xacro directamente.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', entity,
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.0',
        ],
        output='screen',
    )

    return [rsp, tf_base, spawn_entity]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='right',
            description="Modelo: 'right', 'left' o 'bipedo'.",
        ),
        OpaqueFunction(function=launch_setup),
    ])

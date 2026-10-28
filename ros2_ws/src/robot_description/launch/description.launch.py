"""
Publica la descripción del modelo y un joint_state_publisher_gui para
mover sus articulaciones a mano (sin control ni RViz).

Uso:

    ros2 launch robot_description description.launch.py model:=right
    ros2 launch robot_description description.launch.py model:=left
    ros2 launch robot_description description.launch.py model:=bipedo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


# Archivo xacro de cada modelo, relativo a share/robot_description.
MODEL_XACRO = {
    'right': os.path.join('urdf', 'urdf_der', 'pata_der.urdf.xacro'),
    'left': os.path.join('urdf', 'urdf_izq', 'pata_izq.urdf.xacro'),
    'bipedo': os.path.join('urdf', 'urdf_bipedo', 'muneco_bipedo.urdf.xacro'),
}


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    if model not in MODEL_XACRO:
        raise RuntimeError(
            f"model debe ser 'right', 'left' o 'bipedo', no '{model}'")

    pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg, MODEL_XACRO[model])
    robot_description = Command(['xacro ', xacro_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description, value_type=str),
        }],
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_gui': True}],
    )

    return [rsp, jsp_gui]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='right',
            description="Modelo: 'right', 'left' o 'bipedo'.",
        ),
        OpaqueFunction(function=launch_setup),
    ])

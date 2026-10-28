"""
Visualización del modelo del muñeco bípedo (sin el resto de la arquitectura).

Reemplaza a display_bipedo, display_der/izq y description_der/izq. Los
cuatro de pierna cargaban los URDF V2.x de las patas del BÍPEDO (no del
muñeco) y display_bipedo cargaba el URDF V3 anterior del muñeco.

Modelo: urdf/urdf_muneco/muneco_bipedo_v3.urdf (Z hacia arriba, mismo
archivo que usa Gazebo, con las etiquetas ros2_control).

Uso:
    ros2 launch robot_description display.launch.py               # con RViz
    ros2 launch robot_description display.launch.py use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_description')
    urdf_file = os.path.join(
        pkg, 'urdf', 'urdf_muneco', 'muneco_bipedo_v3.urdf')
    rviz_config = os.path.join(pkg, 'config', 'robot.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        rsp,
        jsp_gui,
        rviz,
    ])

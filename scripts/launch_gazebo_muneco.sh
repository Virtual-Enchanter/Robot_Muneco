#!/usr/bin/env bash
# Compila el workspace y abre el muñeco en Gazebo con la arquitectura.
# Reemplaza al launchGazebo.sh suelto del workspace muneco_ws.
#
# Uso (desde cualquier carpeta):
#   ./scripts/launch_gazebo_muneco.sh              # Gazebo + interfaz (pierna izq)
#   ./scripts/launch_gazebo_muneco.sh leg:=der     # argumentos extra van al launch
set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${REPO_DIR}/ros2_ws"
colcon build
source install/setup.bash
ros2 launch robot_bringup bringup_muneco.launch.py "$@"

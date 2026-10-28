# ARQUITECTURA DEL MUÑECO BÍPEDO (ROS 2 HUMBLE)

Cómo está armado el proyecto, para que cualquiera del grupo entienda qué hace
cada parte y cómo se comunican, sin leer todo el código.

Estado (rama feature/optimizacion-arquitectura-muneco, corte 3): los 8
paquetes compilan; una pierna a la vez corre con **6 nodos** (antes 11) y el
muñeco completo se simula en Gazebo con la misma arquitectura. La estructura
es la misma del repositorio del bípedo para facilitar la transición. Los
cambios y su justificación están en docs/arquitectura/OPTIMIZACION_ARQUITECTURA.md.

## Paquetes y qué contiene cada uno

robot_interfaces
  Solo define los mensajes (la "regla de comunicación"). No corre nada.
  - msg/RobotCommand.msg, msg/JointTarget.msg, msg/JointState.msg.
  - msg/IKJacobTarget.msg, msg/IKResult.msg: cinemática inversa.

robot_description
  Un solo modelo: urdf/urdf_muneco/muneco_bipedo_v3.urdf (exportado de
  SolidWorks, Z hacia arriba, mallas en meshes/meshes_muneco). Es el mismo
  archivo para RViz y Gazebo: trae el bloque <ros2_control> (6 joints con
  interfaz de posición, hardware MunecoSystem) y el plugin gz_ros2_control.
  Joints: Right_/Left_ Hip_Roll_Joint, Hip_Pitch_Joint, Knee_Joint.
  Límites del URDF: Hip_Roll [-0.13963, 1.5708], Hip_Pitch y Knee
  [-1.5708, 1.5708].
  launch/display.launch.py: solo el modelo (robot_state_publisher +
  joint_state_publisher_gui + RViz opcional). Fixed Frame de RViz: Base_link.
  Eliminados: el URDF V3 anterior (Y hacia arriba, necesitaba un wrapper
  rotado), los URDF por pata V2.2/V2.5 del BÍPEDO con sus mallas, y los
  launch description_*/display_*/gazebo_* (Gazebo Classic).

robot_kinematics
  UN nodo: kinematics_node (antes 6 procesos).
  - Reenvío: /robot/command -> /robot/joint_targets (recorta a num_joints).
  - Cinemática inversa, 5 métodos en el mismo proceso (tabla SOLVERS):
        algebraico  /robot/ik_target        -> /robot/ik_result
        desacople   /robot/ik_des_target    -> /robot/ik_des_result
        geometrico  /robot/ik_geom_target   -> /robot/ik_geom_result
        newton      /robot/ik_newton_target -> /robot/ik_newton_result
        gradiente   /robot/ik_grad_target   -> /robot/ik_grad_result
    Los nombres de tópicos NO cambiaron (la tabla IK_METHODS de teleop_node
    sigue igual). Para la pierna derecha se refleja z, como antes.
  - Parámetros: num_joints, leg_side, enable_ik, ik_methods.
  - Ejecutor multihilo: el reenvío no se bloquea mientras el gradiente
    descendente itera (~1 s en objetivos que no converge).
  Módulos: cinematica_directa_der_izq.py (L1..L5 del muñeco) y los
  kinem_invers_leg_*.py. Eliminados kinem_leg_gen.py (FK duplicada que no
  importaba nadie) y kinem_leg_rtb.py (script de prueba con longitudes 1.0
  que dependía de roboticstoolbox sin declararlo).

robot_control
  Nodo control_node: /robot/joint_targets -> límites -> /robot/joint_states,
  /joint_states y /robot/joint_commands. Invierte el signo del Knee Pitch al
  publicar (convención del URDF).
  - verify_mode=True (defecto): clamp directo. False: PID (pid_controllers.py).
  - Parámetros cacheados: se leen una vez y se actualizan con
    "ros2 param set" (se rechaza una lista con tamaño distinto a num_joints).
    Defaults: joint_limits_lower [-0.261799, -1.5708, -1.5708],
    joint_limits_upper [1.5708, 1.5708, 1.5708].
  Eliminados: balance.py y sensor_processing.py (esqueletos sin uso),
  motor_control.py (integrado en pid_controllers.py).

robot_simulation
  - launch/gazebo_muneco.launch.py: Gazebo (ros_gz_sim), spawn del muñeco y
    controladores joint_state_broadcaster + joint_group_position_controller,
    cargados en cadena por eventos.
  - config/controllers.yaml: controller_manager a 100 Hz, 6 joints en orden
    [R_roll, R_pitch, R_knee, L_roll, L_pitch, L_knee].
  - Nodo sim_bridge: /robot/joint_commands (3 joints de la pierna) -> vector
    de 6 en /joint_group_position_controller/commands; la otra pierna
    conserva su último valor.

robot_teleop
  Nodo teleop_node: sliders + textbox, cinemática directa, pestañas de IK.
  Hardware: publica /servo_commands directo a la ESP32 (micro-ROS).
  Límites visuales en el código (lower_deg / upper_deg).

robot_serial_bridge
  Nodo serial_bridge: lee las líneas "[POS] ..." del firmware
  control_pata (sin micro-ROS) y publica /servo_states.

robot_bringup
  - launch/bringup_pierna.launch.py leg:=izq|der: arquitectura + RViz
    (reemplaza a bringup_der/bringup_izq).
  - launch/bringup_muneco.launch.py: Gazebo + arquitectura.

## Nodos y tópicos

Con RViz (bringup_pierna):
teleop_node --/robot/command--> kinematics_node --/robot/joint_targets--> control_node
teleop_node --/robot/ik*_target--> kinematics_node --/robot/ik*_result--> teleop_node
control_node --/joint_states_pata--> joint_state_publisher --/joint_states--> robot_state_publisher --TF--> RViz
control_node --/robot/joint_states--> teleop_node
control_node --/robot/joint_commands--> sim_bridge --/joint_group_position_controller/commands-->
teleop_node --/servo_commands--> ESP32 (micro-ROS) --/servo_states-->

Con Gazebo (bringup_muneco):
control_node --/robot/joint_commands--> sim_bridge --/joint_group_position_controller/commands--> Gazebo
Gazebo (joint_state_broadcaster) --/joint_states--> robot_state_publisher
(el /joint_states de control_node se renombra a /control/joint_states)

cualquiera --/robot/e_stop (Bool)--> control_node (frena)

## Cómo probar

Paso 1 - Compilar:
    cd <repo>/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash

Paso 2a - Una pierna en RViz:
    ros2 launch robot_bringup bringup_pierna.launch.py leg:=izq

Paso 2b - Muñeco en Gazebo moviendo una pierna desde la interfaz:
    ros2 launch robot_bringup bringup_muneco.launch.py leg:=izq
  o con el script (compila y lanza): ./scripts/launch_gazebo_muneco.sh
  Solo Gazebo: ros2 launch robot_simulation gazebo_muneco.launch.py

Paso 3 - Mover la pierna sin la interfaz (con bringup_muneco teleop:=false):
    ros2 topic pub --once /robot/command robot_interfaces/msg/RobotCommand "{mode: 0, position: [0.3, 0.4, 0.5], velocity: [0.0, 0.0, 0.0]}"
  Gazebo debe dejar Left_Hip_Roll=0.3, Left_Hip_Pitch=0.4, Left_Knee=-0.5
  (rodilla invertida por control_node):
    ros2 topic echo /joint_states

Paso 4 - Ver el grafo: ros2 node list ; rqt_graph

Paso 5 - Paro de emergencia:
    ros2 topic pub --once /robot/e_stop std_msgs/msg/Bool "{data: true}"

NO publicar a mano 3 valores en /joint_group_position_controller/commands:
el controlador espera 6; en Jazzy el controller_manager lo desactiva.

## Pendiente (próxima actividad)

- Comunicación con Arduino UNO Q (receptor iA8 para servos), IMU BNO085 y
  control aplicado a los nodos.
- teleop publica /servo_commands directo, sin pasar por control_node (sin
  e-stop ni límites internos). En el bípedo ya pasa por control_node
  (/robot/hardware_command); igualar en el muñeco al integrar el UNO Q.
- Alinear límites de cadera roll: control -0.2618 rad, teleop 0°, URDF
  -0.1396 rad.
- El modo PID (verify_mode=False) es un modelo de prueba: integra la consigna
  como velocidad y la junta se va al límite; reemplazar con el control real.
- Caminata: generador de pasos para las dos piernas a la vez (meta 6-nov).

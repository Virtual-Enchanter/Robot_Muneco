# ARQUITECTURA DEL MUÑECO BÍPEDO (ROS 2 HUMBLE)

Cómo está armado el proyecto, para que cualquiera del grupo entienda qué hace
cada parte y cómo se comunican, sin leer todo el código.

Estado (rama feature/integracion-final-optimizaciones, corte 3): integra la
optimización de Santiago Sarria (feature/optimizacion-arquitectura-muneco) y
la auditoría de Hean Felipe Gómez (refactor/auditoria-arquitectura). Quedan
**7 paquetes** (se eliminó robot_serial_bridge); una pierna a la vez corre con
**6 nodos** (antes 11) y el muñeco completo se simula en Gazebo con la misma
arquitectura. Justificación de cada cambio:
docs/arquitectura/OPTIMIZACION_ARQUITECTURA.md,
docs/INFORME_REFACTORIZACION_ARQUITECTURA.md y
docs/arquitectura/INTEGRACION_OPTIMIZACIONES.md.

## Paquetes y qué contiene cada uno

robot_interfaces
  Solo define los mensajes (la "regla de comunicación"). No corre nada.
  srv/ y action/ están vacíos por ahora.
  - RobotCommand.msg: orden del operador (mode, position[], velocity[]).
  - JointTarget.msg: consigna articular (position[], velocity[]).
  - JointState.msg: estado articular (name[], position[], velocity[],
    effort[], battery).
  - IKResult.msg: resultado de cinemática inversa (reachable, position[]).
  - IKJacobTarget.msg: objetivo de los métodos iterativos (x, y, z y
    semilla q1..q3).

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
  rotado), los URDF por pata V2.2/V2.5 del BÍPEDO con sus mallas, los
  launch description_*/display_*/gazebo_* (Gazebo Classic) y
  config/joint_names.yaml (ningún nodo lo cargaba).

robot_kinematics
  UN nodo: kinematics_node (antes 6 procesos).
  - Reenvío: /robot/command -> /robot/joint_targets (recorta a num_joints).
  - Cinemática inversa, 5 métodos en el mismo proceso (tabla SOLVERS):
        algebraico  /robot/ik_target        -> /robot/ik_result        (Point)
        desacople   /robot/ik_des_target    -> /robot/ik_des_result    (Point)
        geometrico  /robot/ik_geom_target   -> /robot/ik_geom_result   (Point)
        newton      /robot/ik_newton_target -> /robot/ik_newton_result (IKJacobTarget)
        gradiente   /robot/ik_grad_target   -> /robot/ik_grad_result   (IKJacobTarget)
    Los nombres de tópicos NO cambiaron (la tabla IK_METHODS de teleop_node
    sigue igual). Para la pierna derecha se refleja z, como antes.
  - Parámetros: num_joints, leg_side, enable_ik, ik_methods.
  - Ejecutor multihilo: el reenvío tiene su propio grupo de callbacks y
    cada método IK el suyo, así que el gradiente descendente (~1 s en
    objetivos que no converge) no bloquea ni el reenvío ni los demás
    métodos.
  Módulos:
  - kinem_common.py: fuente única de L1..L5, límites articulares,
    dh_matrix, fk_T y joint_limit_warnings (antes copiados en ~6 archivos).
  - cinematica_directa_der_izq.py: cinemática directa de las dos piernas
    (la usa teleop).
  - kinem_invers_leg_*.py: un módulo por método (pierna izquierda; la
    derecha es el espejo en z).
  Eliminados: ik_node, ik_des_node, ik_geom_node, ik_newton_node,
  ik_grad_node (absorbidos por kinematics_node), kinem_leg_gen.py (FK
  duplicada que no importaba nadie) y kinem_leg_rtb.py (script de prueba
  con longitudes 1.0 que dependía de roboticstoolbox sin declararlo).

robot_control
  Nodo control_node: /robot/joint_targets -> límites -> /robot/joint_states,
  /joint_states y /robot/joint_commands. Invierte el signo del Knee Pitch al
  publicar (convención del URDF).
  - verify_mode=True (defecto): clamp directo. False: PID (pid_controllers.py,
    que ahora incluye MotorController).
  - Parámetros cacheados: se leen una vez y se actualizan con
    "ros2 param set" (se rechaza una lista con tamaño distinto a num_joints).
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
  Hardware: publica /servo_commands directo a la ESP32 (micro-ROS) con
  6 valores [IZQ roll, pitch, knee, DER roll, pitch, knee]; la pierna que no
  controla va como NaN para que el firmware no la mueva.
  Límites visuales en el código (lower_deg / upper_deg).

robot_bringup
  - launch/bringup_pierna.launch.py leg:=izq|der (o left|right):
    arquitectura + RViz (reemplaza a bringup_der/bringup_izq).
  - launch/bringup_muneco.launch.py: Gazebo + arquitectura.

firmware/esp32_servos (fuera de ros2_ws)
  Firmware de la ESP32 (PlatformIO + micro-ROS). Controla los 6 servos
  (3 por pierna) con PWM directo desde los GPIO 18, 19, 23, 25, 26 y 27
  (librería ESP32Servo), sin el módulo PCA9685. Se suscribe a
  /servo_commands (6 valores, NaN = no mover) y publica /servo_states.
  Eliminados: el paquete robot_serial_bridge y el firmware
  control_pata_sin_ros (segundo camino por puerto serie, también con
  PCA9685, cuyo /servo_states no consumía ningún nodo).

## Nodos y tópicos

Con RViz (bringup_pierna):
teleop_node --/robot/command--> kinematics_node --/robot/joint_targets--> control_node
teleop_node --/robot/ik*_target--> kinematics_node --/robot/ik*_result--> teleop_node
control_node --/joint_states_pata--> joint_state_publisher --/joint_states--> robot_state_publisher --TF--> RViz
control_node --/robot/joint_states--> teleop_node
control_node --/robot/joint_commands--> sim_bridge --/joint_group_position_controller/commands-->
teleop_node --/servo_commands (6 valores)--> ESP32 (micro-ROS) --/servo_states-->

Con Gazebo (bringup_muneco):
control_node --/robot/joint_commands--> sim_bridge --/joint_group_position_controller/commands--> Gazebo
Gazebo (joint_state_broadcaster) --/joint_states--> robot_state_publisher
(el /joint_states de control_node se renombra a /control/joint_states)

cualquiera --/robot/e_stop (Bool)--> control_node (frena)

/servo_states lo publica la ESP32; hoy ningún nodo lo consume.

## Límites de los motores

El ángulo de cada motor se limita en tres lugares que hay que mantener a mano:

1. URDF (robot_description): <limit> de diseño; ningún nodo lo lee.
2. control_node (robot_control): clamp de seguridad en radianes.
     joint_limits_lower = [-0.261799, -1.5708, -1.5708]  (-15°, -90°, -90°)
     joint_limits_upper = [ 1.5708,    1.5708,  1.5708]  (+90°, +90°, +90°)
   En vivo: ros2 param set /control_node joint_limits_lower "[-0.261799, -1.5708, -1.5708]"
3. teleop_node (robot_teleop): rango de los sliders, fijo en el código
   (lower_deg = [0, -90, -90], upper_deg = [90, 90, 90]).

Cadera roll no coincide: control -15°, teleop 0°, URDF -8° (-0.1396 rad).

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

Paso 6 - Robot físico (ESP32 con esp32_servos): levantar el micro-ROS agent
  (ver firmware/esp32_servos/README.md):
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6

NO publicar a mano 3 valores en /joint_group_position_controller/commands:
el controlador espera 6; en Jazzy el controller_manager lo desactiva.

## Pendiente (próxima actividad)

- Comunicación con Arduino UNO Q (receptor iA8 para servos), IMU BNO085 y
  control aplicado a los nodos.
- Paro de emergencia: nadie publica /robot/e_stop y, cuando está activo,
  control_node deja de publicar /robot/joint_commands (el actuador conserva
  la última orden). teleop publica /servo_commands directo, sin pasar por el
  e-stop ni los límites de control_node.
- Alinear límites de cadera roll (control, teleop y URDF).
- Signo de la rodilla: control_node la invierte; confirmar en el robot que
  teleop no la vuelve a invertir.
- Calibrar los servos con PWM directo (servo_params.h: límites copiados del
  esquema con PCA9685).
- El modo PID (verify_mode=False) es un modelo de prueba: integra la consigna
  como velocidad y la junta se va al límite; reemplazar con el control real.
- Caminata: generador de pasos para las dos piernas a la vez (meta 6-nov).

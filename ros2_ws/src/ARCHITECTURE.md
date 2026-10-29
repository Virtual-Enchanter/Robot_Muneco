# ARQUITECTURA DEL ROBOT BÍPEDO (ROS 2 HUMBLE)

Cómo está armado el proyecto, para que cualquiera del grupo entienda qué hace
cada parte y cómo se comunican, sin leer todo el código.

Estado: el flujo de una pierna funciona (teleop -> cinemática -> control ->
robot_state_publisher -> RViz). La lógica pesada de cada sub-área está en
desarrollo.

## Paquetes

robot_interfaces
  Define los mensajes. No corre nada. srv/ y action/ están vacíos por ahora.
  - RobotCommand.msg: orden del operador (mode, position[], velocity[]).
  - JointTarget.msg: consigna articular (position[], velocity[]).
  - JointState.msg: estado articular (name[], position[], velocity[], effort[], battery).
  - IKResult.msg: resultado de cinemática inversa (reachable, position[]).
  - IKJacobTarget.msg: objetivo para los métodos iterativos (x, y, z, semilla q1..q3).

robot_description
  Modelos URDF/Xacro exportados desde SolidWorks, mallas y RViz.
  - urdf/urdf_bipedo/muneco_bipedo.urdf.xacro: muñeco completo (6 joints).
  - urdf/urdf_der/pata_der.urdf.xacro y urdf/urdf_izq/pata_izq.urdf.xacro:
    una pierna cada uno, con frame "world" para RViz.
  Joints: Right_/Left_ Hip_Roll_Joint, Hip_Pitch_Joint, Knee_Joint.
  Launches:
  - description.launch.py model:=right|left|bipedo: robot_state_publisher + joint_state_publisher_gui.
  - display.launch.py model:=...: lo anterior + RViz.
  - gazebo.launch.py model:=...: carga el modelo en Gazebo (requiere gazebo_ros).

robot_kinematics
  - kinematics_node: reenvía /robot/command a /robot/joint_targets. La cinemática
    inversa todavía no se aplica aquí (TODO en el código).
  - ik_node: atiende los cinco métodos de cinemática inversa en un solo proceso.
    Cada método tiene su par de tópicos (ver tabla de tópicos).
  - kinem_common.py: fuente única de L1..L5, límites articulares, dh_matrix,
    fk_T y joint_limit_warnings.
  - cinematica_directa_der_izq.py: cinemática directa (DH) de ambas piernas
    y del bípedo completo. Es la usada por teleop.
  - kinem_invers_leg_*.py: un módulo por método de cinemática inversa (pierna
    izquierda; la derecha es el espejo en z).

robot_control
  control_node: recibe /robot/joint_targets, aplica límites de seguridad y
  publica el estado. Ver "Modos de control" y "Límites de los motores".
  pid_controllers.py y motor_control.py solo se usan en modo PID.

robot_teleop
  teleop_node: interfaz gráfica (Tkinter) con un slider por motor, entrada de
  ángulos a mano, matriz 0A4 de cinemática directa y una pestaña por cada método
  de cinemática inversa. Requiere pantalla.

robot_bringup
  - bringup.launch.py leg:=right|left: levanta una pierna completa (ver abajo).
  - test_injector_node.py: publica una consigna fija en /robot/command para
    probar el flujo sin GUI. No está incluido en ningún launch.

robot_simulation
  sim_bridge: traduce /robot/joint_commands a /joint_group_position_controller/commands.
  Esqueleto: no hay mundo de Gazebo ni controlador conectado todavía.
  sim.launch.py: lanza solo el sim_bridge.

firmware/esp32_servos
  Firmware de la ESP32 (PlatformIO + micro-ROS). Controla los 6 servos (3 por
  pierna) con PWM directo desde los GPIO 18, 19, 23, 25, 26 y 27, sin PCA9685.
  Se suscribe a /servo_commands (6 valores: IZQ roll/pitch/knee, DER roll/pitch/knee;
  NaN = no mover) y publica /servo_states. Ver su README.

## Tópicos y nodos

Pierna (bringup.launch.py leg:=...):

  teleop_node --/robot/command------------> kinematics_node
  kinematics_node --/robot/joint_targets--> control_node
  control_node --/robot/joint_states-----> teleop_node
  control_node --/joint_states_pata-----> joint_state_publisher
  joint_state_publisher --/joint_states--> robot_state_publisher --TF--> RViz
  control_node --/robot/joint_commands--> sim_bridge

  teleop_node --/robot/ik_*_target-------> ik_node --/robot/ik_*_result--> teleop_node
  teleop_node --/servo_commands (6 valores)--> firmware ESP32 (micro-ROS)

Notas:
- control_node publica /joint_states y el launch lo remapea a /joint_states_pata,
  porque joint_state_publisher necesita juntar las dos piernas en un solo mensaje.
- /servo_states lo publica la ESP32; nadie lo consume dentro de ROS todavía.

Tabla de tópicos de cinemática inversa:

| Método | Objetivo (entrada) | Resultado (salida) | Tipo de objetivo |
|---|---|---|---|
| Algebraico | /robot/ik_target | /robot/ik_result | Point |
| Newton-Raphson | /robot/ik_newton_target | /robot/ik_newton_result | IKJacobTarget |
| Desacople | /robot/ik_des_target | /robot/ik_des_result | Point |
| Geométrico | /robot/ik_geom_target | /robot/ik_geom_result | Point |
| Gradiente descendente | /robot/ik_grad_target | /robot/ik_grad_result | IKJacobTarget |

Los métodos newton y grad usan IKJacobTarget porque necesitan semilla (q1, q2, q3 en grados).

## Modos de control

- verify_mode=True (por defecto): el valor llega a /robot/joint_states con el
  clamp de límites. Sin PID. Sirve para verificar que lo que mandan los sliders
  se refleja en RViz.
- verify_mode=False: se activa el PID (pid_controllers.py + motor_control.py).

Cambiar el modo en vivo:

    ros2 param set /control_node verify_mode false

## Límites de los motores

El ángulo de cada motor se limita en tres lugares. No están sincronizados por
un solo origen: hay que mantenerlos a mano.

1. URDF (robot_description): <limit> de diseño. Referencia, ningún nodo la lee.
2. control_node.py (robot_control): límite INTERNO en radianes. Es el clamp de
   seguridad. Por defecto:
     joint_limits_lower = [-0.261799, -1.5708, -1.5708]  (-15°, -90°, -90°)
     joint_limits_upper = [ 1.5708,    1.5708,  1.5708]  (+90°, +90°, +90°)
   Se puede cambiar en vivo:
     ros2 param set /control_node joint_limits_lower "[-0.261799, -1.5708, -1.5708]"
     ros2 param set /control_node joint_limits_upper "[1.5708, 1.5708, 1.5708]"
3. teleop_node.py (robot_teleop): rango de los sliders en grados. NO es un
   parámetro ROS: está fijo en el código como lower_deg = [0, -90, -90] y
   upper_deg = [90, 90, 90]. Para cambiarlo hay que editar teleop_node.py.

Hoy el límite inferior de cadera-roll es distinto en teleop (0°) y en control
(-15°). Si se quieren iguales, hay que ajustarlos juntos.

## Signo de la rodilla

control_node invierte el signo de la rodilla (índice 2) antes de publicar en
/robot/joint_states, /joint_states y /robot/joint_commands. La convención de
trabajo es que +Knee Pitch recoge la rodilla. teleop recibe el estado ya
invertido: hay que confirmar en el robot que la visualización de teleop no
vuelve a invertirla.

## Paro de emergencia (pendiente)

control_node se suscribe a /robot/e_stop (Bool). Hoy nadie lo publica: no hay
botón de paro en teleop ni otro nodo que lo envíe.

Cuando e_stop está activo, control_node publica ceros en /robot/joint_states y
/joint_states pero NO publica en /robot/joint_commands. Un actuador que
escuche ese tópico conserva la última orden. Esto está pendiente de corregir
antes de usar el robot físicamente.

## Cómo probar

Paso 1 - Compilar (una vez por terminal):

    cd ~/Documentos/Robot_Bipedo/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash

Paso 2 - Levantar una pierna (incluye sliders y RViz). Usar una terminal por
pierna, no las dos a la vez:

    ros2 launch robot_bringup bringup.launch.py leg:=right
    ros2 launch robot_bringup bringup.launch.py leg:=left

Argumentos útiles: use_rviz:=false para no abrir RViz.

Paso 3 - Ver que los tópicos circulan (otra terminal, con source):

    ros2 topic list
    ros2 topic echo /joint_states
    rqt_graph

Paso 4 - Robot físico (ESP32): levantar el micro-ROS agent con la ESP32
conectada. Ver firmware/esp32_servos/README.md:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6

Paso 5 - Cerrar: Ctrl+C en cada terminal.

Si RViz dice "No transform from [world]" o no aparece la pierna: confirmar que
el bringup corre y que `ros2 topic echo /joint_states` trae datos.

## Pendiente

- Cinemática: aplicar la cinemática inversa al flujo principal (kinematics_node).
- Descripción: URDF combinado de las dos piernas (torso/pelvis común).
- Simulación: mundo Gazebo real y controlador de posición.
- Control: PID, balance e IMU cuando haya hardware.
- Seguridad: publicar e_stop desde teleop y que control_node detenga los comandos.

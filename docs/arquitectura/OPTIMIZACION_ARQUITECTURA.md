# Entregable 1 — Optimización de la arquitectura ROS 2

_Robot bípedo y muñeco bípedo · Recuento, depuración y unificación de paquetes, nodos y tópicos_

- **Responsable:** Santiago Sarria Arcila — código 2215465
- **Fecha:** 28 de octubre de 2026
- **Repositorio bípedo:** `JuanMiguelC27/Robot_Bipedo` (base: main @ 697529a (14-oct-2026)) → rama `feature/optimizacion-arquitectura-bipedo`
- **Repositorio muñeco:** `juanperezpomar-source/Bipedo_Mu-eco` (base: main (descarga .zip) + muneco_ws (Gazebo)) → rama `feature/optimizacion-arquitectura-muneco`

> Versión en texto del PDF del entregable. El PDF es el documento oficial.

## 1. Resumen

Se auditaron los dos repositorios (8 paquetes ROS 2 cada uno) leyendo el código fuente extraído de los .zip y ejecutando la arquitectura en un entorno ROS 2 de prueba. La redundancia principal estaba en robot_kinematics: cinco nodos de cinemática inversa con el mismo código copiado, más el nodo de reenvío, corrían como seis procesos. Se unificaron en un solo kinematics_node que conserva exactamente los mismos tópicos, de modo que la interfaz (teleop_node) no cambia. También se unificaron los launch duplicados por pierna, se eliminaron caminos de tópicos sin publicador o sin suscriptor, módulos huérfanos de los cortes 1 y 2 y archivos de prueba. En el muñeco se integró la simulación de Gazebo del workspace muneco_ws y se corrigió el puente control → Gazebo, que enviaba 3 valores a un controlador de 6 articulaciones.

| Métrica | Bípedo antes | Bípedo después | Muñeco antes | Muñeco después |
|---|---|---|---|---|
| Nodos con una pierna (leg:=izq) | 11 | 6 | 11 | 6 |
| Procesos de cinemática | 6 | 1 | 6 | 1 |
| RAM de los procesos de cinemática | 420,6 MB | 70,9 MB | 417,9 MB | 69,9 MB |
| RAM total de los nodos del proyecto | 682,1 MB | 328,2 MB | 645,9 MB | 294,0 MB |
| Tópicos activos (leg:=izq) | 37 | 34 | 24 | 24 |
| Ejecutables (entry points) | 11 | 5 | 11 | 5 |
| Archivos launch | 9 | 2 | 10 | 4 |
| Líneas eliminadas / agregadas (código y docs) | — | −4524 / +828 | — | −3434 / +1540 |
| Advertencias de colcon build | 1 | 0 | 1 | 0 |
| Gazebo moviendo una pierna desde la arquitectura | no aplica | no aplica | no | sí (error 0,000 rad) |

## 2. Metodología

- Extracción de muneco_ws.zip (2383 archivos) y Bipedo_Mu-eco-main.zip (215 archivos) con **unzip**. muneco_ws contiene un clon del repositorio del bípedo (JuanMiguelC27/Robot_Bipedo, commit 697529a) con el trabajo de Gazebo del muñeco sin subir (URDF, mallas, controllers.yaml y bringup_muneco.launch.py). Bipedo_Mu-eco-main es el repositorio del muñeco.
- Análisis estático de los 68 + 65 archivos .py: árbol sintáctico (AST) para listar cada create_publisher / create_subscription con su tópico y tipo, imports entre módulos, entry points de setup.py, dependencias de package.xml y nodos lanzados por cada launch.
- Análisis dinámico: colcon build de ambos workspaces, ros2 launch de cada bringup, recuento de nodos y tópicos con la API de grafo de ROS 2 (lo mismo que muestran ros2 node list, ros2 topic info y rqt_graph), marcando tópicos sin publicador o sin suscriptor.
- Pruebas antes/después: equivalencia numérica de la IK, reenvío de comandos, límites en vivo, envío a hardware, puente a Gazebo y simulación del muñeco en Gazebo sin ventana.
- Entorno de prueba: ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04). El laboratorio usa Humble + Fortress: la lista de verificación de la sección 6 se debe correr allí antes de integrar a main.

## 3. Recuento de paquetes, nodos y tópicos

### Bípedo

| Paquete | Antes | Después |
|---|---|---|
| robot_interfaces | RobotCommand, JointTarget, JointState, IKJacobTarget, IKResult | Se conserva: todos los mensajes tienen publicador y suscriptor. |
| robot_description | URDF V9 completo + URDF por pata V2.2/V2.5, 6 launch | 1 URDF (V9) + display.launch.py |
| robot_kinematics | kinematics_node, ik_node, ik_des_node, ik_geom_node, ik_newton_node, ik_grad_node | kinematics_node (unificado) |
| robot_control | control_node + balance, sensor_processing, motor_control, pid_controllers | control_node + pid_controllers |
| robot_simulation | sim_bridge (esqueleto) + sim.launch.py | sim_bridge (vector de 6 joints) |
| robot_teleop | teleop_node + trayectorias_ui + forward_kinematics | teleop_node + trayectorias_ui |
| robot_serial_bridge | serial_bridge (bldc / servos) | Se conserva sin cambios (puente a las ESP32 en uso). |
| robot_bringup | bringup_der, bringup_izq, test_injector_node | bringup_pierna.launch.py leg:=izq\|der |

### Muñeco

| Paquete | Antes | Después |
|---|---|---|
| robot_interfaces | RobotCommand, JointTarget, JointState, IKJacobTarget, IKResult | Se conserva. |
| robot_description | URDF V3 del muñeco (Y arriba) + URDF por pata del bípedo, 7 launch | URDF Super_Final V3 (Z arriba, ros2_control) + display.launch.py |
| robot_kinematics | 6 nodos + kinem_leg_gen, kinem_leg_rtb | kinematics_node (unificado) |
| robot_control | control_node + balance, sensor_processing, motor_control, pid_controllers | control_node + pid_controllers |
| robot_simulation | sim_bridge (esqueleto) + sim.launch.py | sim_bridge + gazebo_muneco.launch.py + controllers.yaml |
| robot_teleop | teleop_node + forward_kinematics | teleop_node |
| robot_serial_bridge | serial_bridge (lee /servo_states del firmware control_pata) | Se conserva sin cambios. |
| robot_bringup | bringup_der, bringup_izq, test_injector_node | bringup_pierna.launch.py + bringup_muneco.launch.py |

### Tópicos sin publicador o sin suscriptor

| Repo | Tópico | Diagnóstico |
|---|---|---|
| Bípedo | /servo_states | control_node se suscribía; ningún nodo ni firmware lo publica (el firmware ya no usa micro-ROS). |
| Bípedo | /robot/hardware_state | Solo se publicaba al recibir /servo_states, así que nunca llegaba a teleop_node. |
| Bípedo | /servo_commands | Publicador legado en control_node (apagado por defecto) sin ningún suscriptor. |
| Ambos | /joint_group_position_controller/commands | Se publicaban 3 valores; el controlador de Gazebo espera 6 y los descarta (en Jazzy además se desactiva). |

### Qué se conserva sin cambios

| Elemento | Motivo |
|---|---|
| robot_interfaces | Los 5 mensajes tienen uso real. srv/ y action/ siguen vacías para la siguiente actividad. |
| kinem_invers_leg_*.py, cinematica_directa_der_izq.py | Cada método lo usa una pestaña de la interfaz y el nodo unificado. |
| kinem_v6.py, perfiles_temporales.py, trayectorias/*.txt (bípedo) | Los usa la pestaña Trayectorias y los cubren 12 pruebas unitarias que pasan. |
| robot_serial_bridge | Es el puente a hardware en uso. Se revisa en la actividad 2 (UNO Q y ESP32 + MCP2518FD). |
| teleop_node.py | Se le quitó solo el camino muerto. Refactorizarlo (4341 líneas en el bípedo) requiere pruebas con operador; queda como pendiente. |

## 4. Cambios realizados y justificación técnica

| Responsable | Repositorio | Paquete | Elemento modificado | Acción realizada | Justificación técnica |
|---|---|---|---|---|---|
| Santiago Sarria Arcila | Bípedo | robot_kinematics | ik_node, ik_des_node, ik_geom_node, ik_newton_node, ik_grad_node y kinematics_node → kinematics_node | Unificación | Los cinco nodos IK eran el mismo archivo (diff ≈ 95 % idéntico): solo cambiaban la función de IK y el prefijo del tópico. Ahora cada método es una entrada de la tabla SOLVERS dentro de kinematics_node, que además hace el reenvío /robot/command → /robot/joint_targets. Se mantienen los mismos tópicos y tipos, así que teleop_node no cambia. 6 procesos → 1 y RAM 420,6 MB → 70,9 MB. Se usa un ejecutor multihilo para que el reenvío no se bloquee mientras Newton o el gradiente iteran. El parámetro leg_side estaba declarado pero no se usaba; ahora enable_ik activa la IK solo en la pierna izquierda, que es la única validada. Prueba: 24/24 respuestas idénticas a llamar directamente cada función (incluye la rama del codo). |
| Santiago Sarria Arcila | Bípedo | robot_bringup | bringup_der.launch.py + bringup_izq.launch.py (906 líneas) → bringup_pierna.launch.py leg:=izq\|der | Unificación | Los dos archivos eran el mismo launch con otros nombres de joints, otro leg_side y los nodos IK solo en la izquierda. Un argumento leg selecciona la pierna. Con leg:=izq se pasa de 11 a 6 nodos y con leg:=der quedan 6 (igual que antes). |
| Santiago Sarria Arcila | Bípedo | robot_bringup | test_injector_node.py + entry point test_injector | Eliminación | Es un esqueleto del corte 1 que publica una orden fija en /robot/command a 5 Hz. Ningún launch lo usa y, si se corre, compite con teleop_node en el mismo tópico. Para pruebas manuales ya existen la interfaz y ros2 topic pub. |
| Santiago Sarria Arcila | Bípedo | robot_description | description_der/izq.launch.py + display_der/izq.launch.py → display.launch.py | Unificación | description_der y description_izq eran idénticos byte a byte. display_* era lo mismo más RViz. Los cuatro cargaban el mismo urdf_completo (V9). Queda un launch con use_rviz:=true\|false. Se quita el parámetro use_gui, que joint_state_publisher_gui ya no usa en ROS 2. |
| Santiago Sarria Arcila | Bípedo | robot_description | gazebo_der/izq.launch.py, urdf_der (V2.2), urdf_izq (V2.5), meshes_der/izq (8 STL), joint_names.yaml, dependencia gazebo_ros | Eliminación | Los launch de Gazebo usaban Gazebo Classic (spawn_entity) sin iniciar el servidor ni cargar ros2_control, así que no podían mover el robot. Eran los únicos que usaban los URDF por pata V2.x, ya reemplazados por el modelo completo V9. joint_names.yaml no lo lee ningún nodo y su texto estaba desactualizado. Quitar gazebo_ros evita que rosdep intente instalar Gazebo Classic. |
| Santiago Sarria Arcila | Bípedo | robot_control | Camino /servo_states → /robot/hardware_state, publicador /servo_commands y parámetro publish_legacy_servo_commands; suscripción y callback en teleop_node | Eliminación | En el grafo en ejecución, /servo_states no tenía publicador y /servo_commands no tenía suscriptor. El firmware actual de las ESP32 habla por serial con robot_serial_bridge (no micro-ROS) y la realimentación real llega por /robot/bldc_state y /robot/servos_state. Tópicos: 37 → 34. |
| Santiago Sarria Arcila | Bípedo | robot_control | balance.py y sensor_processing.py | Eliminación | Ningún archivo los importaba. sensor_processing integraba el giróscopo sin corrección, lo que deriva con el tiempo. La IMU BNO085 entrega la orientación ya fusionada, así que el equilibrio con IMU se implementa desde cero en la siguiente actividad. |
| Santiago Sarria Arcila | Bípedo | robot_control | motor_control.py → pid_controllers.py | Unificación | Era una clase de 6 líneas (consigna + corrección) que solo usa control_node junto al PID. Los imports se hacían dentro del bucle de 50 Hz; ahora van al inicio del módulo. |
| Santiago Sarria Arcila | Bípedo | robot_control | control_node: lectura de parámetros | Optimización | El bucle de 50 Hz leía 3 parámetros en cada ciclo (150 lecturas/s) y on_hardware_command leía 3 más. Ahora los valores se guardan una vez y se actualizan con un callback de "ros2 param set" que valida el tamaño. Antes, una lista de tamaño incorrecto congelaba el control sin avisar; ahora se rechaza con un mensaje. Prueba: lista inválida rechazada y límite nuevo aplicado al instante. |
| Santiago Sarria Arcila | Bípedo | robot_simulation | sim_bridge_node.py | Corrección | Reenviaba los 3 valores de una pierna a /joint_group_position_controller/commands, pero ese controlador maneja 6 joints y descarta el comando (en Jazzy además se desactiva: comprobado en Gazebo). Ahora ubica cada valor por nombre de joint en el vector de 6 y mantiene la otra pierna. Prueba: [0.2, 0.3, 0.5] → índices 0-2 (derecha) o 3-5 (izquierda). |
| Santiago Sarria Arcila | Bípedo | robot_simulation | sim.launch.py; marcador ament en setup.py | Eliminación | Solo lanzaba sim_bridge, que ya inicia el bringup. Se agrega el marcador del índice ament, con lo que desaparece la advertencia de colcon. |
| Santiago Sarria Arcila | Bípedo | robot_teleop | forward_kinematics.py + test_forward_kinematics.py | Eliminación | Es la tabla DH antigua (kinem_leg_gen, longitudes 1.0). teleop_node usa robot_kinematics/cinematica_directa_der_izq.py y el único uso de este módulo era su propio test. |
| Santiago Sarria Arcila | Bípedo | Raíz del repo | tests/Hola_mundo.py, firmware/esp32_servos/compile_commands.json, scripts/trazador/trazador_backup.py, scripts/analisis/ | Eliminación | Hola_mundo es una prueba de git. compile_commands.json lo genera PlatformIO con rutas absolutas de un PC (se agrega a .gitignore). trazador_backup duplica una versión anterior que ya está en el historial de git. scripts/analisis es la versión V6 del análisis de espacio de trabajo, reemplazada por scripts/espacio_trabajo según la propia documentación. |
| Santiago Sarria Arcila | Bípedo | Documentación | ARCHITECTURE.md, README.md, ESPACIO_TRABAJO_Y_SUPERFICIE_ESCRITURA.md | Actualización | Se reflejan los 8 paquetes reales (el README listaba 6), el grafo nuevo y los comandos de prueba. Se corrigen datos que ya no eran ciertos (micro-ROS, límites, parámetros del teleop que no existen). |
| Santiago Sarria Arcila | Muñeco | robot_kinematics | Los 5 nodos ik_* y kinematics_node → kinematics_node | Unificación | Mismo diseño que en el bípedo, para que las dos arquitecturas sean iguales. Se usan los módulos propios del muñeco (L1..L5) y se conserva el reflejo de z para la pierna derecha. 6 procesos → 1 y RAM 417,9 MB → 69,9 MB. Prueba: 15/15 respuestas idénticas en la pierna izquierda y equivalentes en la derecha. El gradiente tarda ~1 s cuando no converge y el reenvío sigue respondiendo. |
| Santiago Sarria Arcila | Muñeco | robot_kinematics | kinem_leg_gen.py y kinem_leg_rtb.py | Eliminación | Ningún archivo los importaba. kinem_leg_gen duplicaba la cinemática directa de cinematica_directa_der_izq. kinem_leg_rtb es un script de prueba con longitudes 1.0 que importa roboticstoolbox, librería no declarada en package.xml, y fallaría si alguien lo importara. |
| Santiago Sarria Arcila | Muñeco | robot_bringup | bringup_der/izq.launch.py (~990 líneas) → bringup_pierna.launch.py; test_injector_node | Unificación / Eliminación | Los dos launch eran iguales salvo por la pierna. Se pasa de 11 a 6 nodos por pierna. test_injector se elimina por las mismas razones que en el bípedo. |
| Santiago Sarria Arcila | Muñeco | robot_bringup | bringup_muneco.launch.py (nuevo, tomado de muneco_ws) | Unificación | Une el Gazebo del muñeco con la arquitectura: teleop → kinematics → control → sim_bridge → controlador de Gazebo, que era la pregunta abierta del coordinador sobre qué tópicos usar. El /joint_states de control_node se renombra para no chocar con el de joint_state_broadcaster. Prueba en Gazebo: /robot/command [0.3, 0.4, 0.5] → joints izquierdos [0.300, 0.400, −0.500] (rodilla invertida por el URDF), error 0,000 rad, y la pierna derecha no se mueve. |
| Santiago Sarria Arcila | Muñeco | robot_description | urdf_bipedo/Muneco_Bipedo_URDF_V3 + wrapper xacro + meshes_bipedo → urdf_muneco/muneco_bipedo_v3.urdf + meshes_muneco | Unificación | Había dos versiones del muñeco: V3 anterior (exportado con Y hacia arriba, necesitaba un xacro que lo rotara) y Super_Final V3 de muneco_ws (Z hacia arriba, con ros2_control). Queda una sola, la misma para RViz y Gazebo, con nombre ASCII (el original tenía "ñ"). Se verificó con cinemática directa que las piernas cuelgan en −Z. RViz pasa a Fixed Frame = Base_link. |
| Santiago Sarria Arcila | Muñeco | robot_description | urdf_der/izq (V2.2/V2.5) + meshes_der/izq (piernas del BÍPEDO), gazebo_*/description_*/display_*/display_bipedo, joint_names.yaml → display.launch.py | Eliminación / Unificación | Los URDF y mallas por pata son del bípedo, no del muñeco, y ningún bringup del muñeco los usaba. Los siete launch de visualización y Gazebo Classic se reemplazan por display.launch.py. |
| Santiago Sarria Arcila | Muñeco | robot_simulation | gazebo_muneco.launch.py + config/controllers.yaml | Optimización | El launch de Gazebo pasa a robot_simulation, que es donde lo ubica el README. Los controladores cargan en cadena por eventos (spawn → broadcaster → posición) en lugar de esperas fijas de 8 y 10 s, lo que garantiza el orden de carga (en la prueba el controlador de posición quedó activo a los 9,5 s del arranque), y se exporta la ruta de mallas para Fortress y Harmonic. controllers.yaml se mueve a robot_simulation porque, con el controller_manager de Jazzy, una ruta que contiene "robot_description" se descarta y los controladores no cargan (comprobado). El URDF solo cambia en esa ruta. |
| Santiago Sarria Arcila | Muñeco | robot_simulation | sim_bridge_node.py; sim.launch.py | Corrección / Eliminación | La misma corrección del vector de 6 joints que en el bípedo. En la prueba de Gazebo, un comando de 3 valores deja el controlador inactivo. sim.launch.py solo lanzaba sim_bridge. |
| Santiago Sarria Arcila | Muñeco | robot_control | balance.py, sensor_processing.py, motor_control.py; lectura de parámetros de control_node | Eliminación / Unificación / Optimización | Las mismas razones que en el bípedo: módulos sin import, MotorController integrado al PID y parámetros guardados con validación en vivo. Se conserva la inversión del Knee Pitch y todos los comentarios originales. |
| Santiago Sarria Arcila | Muñeco | robot_teleop | forward_kinematics.py + test | Eliminación | Es la DH antigua de longitudes unitarias y no la usa la interfaz. |
| Santiago Sarria Arcila | Muñeco | Raíz del repo | prueba.txt, tests/Hola_mundo.py; scripts/launch_gazebo_muneco.sh | Eliminación / Unificación | prueba.txt y Hola_mundo son pruebas de git. El launchGazebo.sh suelto de muneco_ws, que estaba fuera del repositorio, pasa a scripts/ y funciona desde cualquier carpeta. |
| Santiago Sarria Arcila | Muñeco | Documentación | ARCHITECTURE.md, README.md | Actualización | Describía URDF por pata y kinem_leg_gen. Ahora describe el modelo único, el flujo a Gazebo y los comandos de prueba. |

## 5. Verificación

| Prueba | Resultado |
|---|---|
| Compilación | colcon build: 8/8 paquetes en los dos repositorios, sin advertencias (antes 1). |
| Grafo en ejecución | Bípedo izq 11 nodos / 37 tópicos → 6 / 34. Bípedo der 6 / 37 → 6 / 34. Muñeco 11 / 24 → 6 / 24 por pierna. |
| Cinemática inversa | Bípedo 24/24 y muñeco 15/15 respuestas idénticas a llamar directamente a cada función, incluyendo objetivos inalcanzables y la rama del codo. |
| Reenvío y control | Reenvío /robot/command → /robot/joint_targets correcto. Límites cambiados en vivo con ros2 param set; lista inválida rechazada. |
| Hardware (bípedo) | /robot/hardware_command [0.1, 0.2, 0.3] → BLDC −84,27°, servos [11,46°, 17,19°]. Fuera de límite → "RECHAZADO". |
| Puente a Gazebo | 6 valores en el orden del controlador para las dos piernas. |
| Gazebo (muñeco) | Spawn a los 4,0 s y controlador de posición activo a los 9,5 s del arranque. /robot/command → joints [0.300, 0.400, −0.500], error 0,000 rad. |
| Pruebas unitarias | Bípedo: 12/12 (las 3 restantes del total anterior de 15 se eliminaron junto con el módulo obsoleto). |
| Estilo (flake8) | Archivos nuevos o reescritos sin errores. Errores totales del bípedo 908 → 842; del muñeco 542 → 438 (el resto es código existente no reescrito). |

## 6. Lista de verificación en el PC del laboratorio (Humble)

- [ ] colcon build sin errores.
- [ ] ros2 launch robot_bringup bringup_pierna.launch.py leg:=izq → ros2 node list muestra 6 nodos (+ rviz2) y rqt_graph no tiene ik_*.
- [ ] Pestaña de IK de la interfaz: un punto alcanzable devuelve q1, q2, q3 igual que antes.
- [ ] (Muñeco) ros2 launch robot_bringup bringup_muneco.launch.py → mover un slider mueve la pierna en Gazebo.
- [ ] (Bípedo, con ESP32) "Aplicar a motores" → /robot/hardware_status responde OK.

## 7. Hallazgos para la siguiente actividad

- Actividad 2: comunicación con Arduino UNO Q, IMU BNO085 y puente ESP32 + 2× MCP2518FD (CAN-FD a moteus) dentro de los nodos existentes.
- Muñeco: teleop publica /servo_commands directo, sin pasar por el e-stop ni los límites de control_node. En el bípedo ya pasa por control_node; igualarlo al integrar el UNO Q.
- Muñeco: alinear el límite de cadera roll (control −0,2618 rad, interfaz 0°, URDF −0,1396 rad) antes de probar en el robot físico.
- Modo PID (verify_mode=False): es un modelo de prueba que integra la consigna como velocidad, y la junta termina en el límite. Reemplazarlo por el control real.
- teleop_node (bípedo): las 5 pestañas de IK repiten 5 funciones cada una. Pasarlas a una tabla como IK_METHODS del muñeco.
- No hay botón de paro de emergencia en la interfaz; hoy solo funciona por tópico (/robot/e_stop).
- Bípedo: agregar ros2_control al URDF V9 para simularlo en Gazebo con el mismo flujo del muñeco.

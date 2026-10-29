# Informe de auditoría y refactorización de la arquitectura ROS 2 — Robot Bípedo

**Autores:**
- Hean Felipe Gómez Gonzalias — 2230669
- Santiago Sarria Arcila — 2215465

**Fecha:** 28 de octubre de 2026
**Rama:** `refactor/auditoria-arquitectura`
**Commits:** `e37a987` (launches y código sin uso) y `2d7e3c8` (nodos IK, firmware, documentación)

Written for: equipo del proyecto y docentes que revisan el avance del desarrollo.

---

## 1. Resumen

Se hizo una auditoría de solo lectura del repositorio `BABAKAR05/Bipedo_Mu-eco`
para inventariar paquetes, nodos, tópicos, launches y dependencias. A partir de
esa auditoría se propusieron cambios de bajo y medio riesgo, que fueron
aprobados y aplicados.

Resultados principales:

- Se redujeron los launches de 10 a 5: nueve duplicados se reemplazaron por cuatro launches parametrizados, y `sim.launch.py` no cambió.
- Se eliminaron 36 archivos: código sin uso o duplicado, un paquete ROS 2 completo (`robot_serial_bridge`) y una carpeta de firmware obsoleta (`control_pata_sin_ros`).
- Los cinco nodos de cinemática inversa se unificaron en uno solo, sin cambiar los tópicos.
- Se eliminó la duplicación de la matemática de cinemática (DH, FK y límites) en un módulo común.
- Se verificó numéricamente que la cinemática no cambió (resultados idénticos antes y después).
- Se reescribió la documentación de arquitectura para que coincida con el código.

Quedaron pendientes dos puntos que requieren decisión y pruebas con el robot
físico: el paro de emergencia y el manejo de dependencias.

## 2. Alcance y metodología

La auditoría siguió dos fases:

1. **Fase 1 — solo lectura.** Inventario de paquetes, nodos, tópicos, servicios,
   acciones y launches, con su función, quién lo inicia, qué publica y qué recibe.
   Ningún elemento se marcó como innecesario solo por no aparecer en un launch.
2. **Fase 2 — diagnóstico y propuesta.** Para cada cambio: motivo, dependencias
   afectadas, riesgos y forma de verificarlo. Los cambios se aplicaron solo después
   de la aprobación del equipo.

Los cambios se hicieron en una rama separada (`refactor/auditoria-arquitectura`),
sin tocar `main`.

## 3. Arquitectura antes y después

### Antes

- 8 paquetes ROS 2, incluido `robot_serial_bridge`.
- Dos launches por pierna (`bringup_der`, `bringup_izq`) casi idénticos, y siete
  launches de descripción y Gazebo también duplicados.
- Cinco nodos de cinemática inversa, cada uno con su propio proceso.
- La matemática DH estaba copiada en unas seis partes distintas, y los límites
  articulares en cuatro lugares.
- Dos firmwares para los mismos servos (`esp32_servos` por micro-ROS y
  `control_pata_sin_ros` por puerto serie), con un puente serial que no
  encajaba en el flujo principal.
- Documentación con valores y nodos que no existían en el código.

### Después

- 7 paquetes ROS 2 (`robot_serial_bridge` eliminado).
- Un launch de bringup (`bringup.launch.py leg:=right|left`) y tres launches de
  modelo (`description`, `display`, `gazebo`) con `model:=right|left|bipedo`.
- Un nodo `ik_node` que atiende los cinco métodos con multihilo y los mismos tópicos.
- Un módulo `kinem_common.py` como única fuente de la matemática común.
- Un solo firmware, `firmware/esp32_servos`, conectado por micro-ROS.
- Documentación (`ARCHITECTURE.md`, `README.md`) alineada con el código.

## 4. Cambios realizados

### 4.1 Launches consolidados (commit `e37a987`)

| Cambio | Archivos | Justificación | Riesgo |
|---|---|---|---|
| Un launch de bringup para ambas piernas | `bringup.launch.py` reemplaza `bringup_der.launch.py` y `bringup_izq.launch.py` | Eran ~95 % idénticos; solo cambiaban nombres de joints y `leg_side`. Ahora el argumento `leg` decide la pierna. | Bajo |
| Launches de descripción parametrizados | `description.launch.py`, `display.launch.py`, `gazebo.launch.py` reemplazan siete archivos | Los pares `der`/`izq` solo cambiaban el nombre del XACRO. `model:=bipedo` cubre el caso del muñeco completo. | Bajo |

Uso resultante:

```bash
ros2 launch robot_bringup bringup.launch.py leg:=right
ros2 launch robot_description display.launch.py model:=bipedo
```

### 4.2 Código sin uso eliminado (commit `e37a987`)

| Archivo eliminado | Motivo |
|---|---|
| `robot_kinematics/kinem_leg_gen.py` | Nadie lo importa. Sus longitudes (200/85/120/235/250 mm) eran distintas a las actuales (90.3/48.5/54.2/138.5/162 mm). La documentación decía que lo usaba `kinematics_node`, y era falso. |
| `robot_kinematics/kinem_leg_rtb.py` | Nadie lo importa, y depende de `roboticstoolbox`, que no está declarado en el paquete. |
| `robot_teleop/forward_kinematics.py` y su test | Duplicaba la cinemática directa con la cadena obsoleta. Teleop usa `cinematica_directa_der_izq.py`. El test validaba la cadena obsoleta, así que se eliminó en lugar de moverlo. |
| `robot_control/balance.py` y `sensor_processing.py` | Stubs sin ninguna importación. Son trabajo futuro de IMU, que no está en el alcance actual. |
| `prueba.txt` y `tests/Hola_mundo.py` | Archivos de prueba sin función en el proyecto. |

Nota sobre `pid_controllers.py` y `motor_control.py`: se conservaron. Se usan
en el modo PID de `control_node` (`verify_mode:=false`), así que no son código sin uso.

### 4.3 Matemática común centralizada (commit `2d7e3c8`)

Se creó `robot_kinematics/kinem_common.py` con:

- Longitudes `L1..L5` [mm].
- Límites articulares `q1_min..q3_max` [°].
- `dh_matrix`, la matriz DH homogénea.
- `fk_T`, la cinemática directa 0A4.
- `joint_limit_warnings`, los avisos de límites.

Los módulos `kinem_invers_leg_*.py` y `cinematica_directa_der_izq.py` importan
ahora estas funciones en lugar de tener sus propias copias. Las copias eran
matemáticamente idénticas, salvo por el formato del mensaje de
`joint_limit_warnings`: se adoptó la versión que incluye el valor del ángulo
y el símbolo de grado correcto (antes aparecía codificado de forma incorrecta).

**Verificación:** se comparó numéricamente la versión anterior con la nueva
sobre una malla de puntos, con cinemática directa de ambas piernas, los cinco
métodos de cinemática inversa y `verificar_con_mth`. Todos los resultados
coincidieron exactamente.

### 4.4 Cinemática inversa en un solo nodo (commit `2d7e3c8`)

`ik_node.py` atiende los cinco métodos en un solo proceso:

| Método | Tópico de objetivo | Tópico de resultado | Tipo |
|---|---|---|---|
| Algebraico | `/robot/ik_target` | `/robot/ik_result` | Point |
| Newton-Raphson | `/robot/ik_newton_target` | `/robot/ik_newton_result` | IKJacobTarget |
| Desacople | `/robot/ik_des_target` | `/robot/ik_des_result` | Point |
| Geométrico | `/robot/ik_geom_target` | `/robot/ik_geom_result` | Point |
| Gradiente descendente | `/robot/ik_grad_target` | `/robot/ik_grad_result` | IKJacobTarget |

- Cada método tiene su propio grupo de callbacks y el ejecutor es multihilo, así
  que el gradiente descendente (hasta 10 000 iteraciones) no bloquea a los demás.
- Los tópicos y tipos de mensaje no cambian, así que `teleop_node` no se modificó.
- Se eliminaron `ik_newton_node`, `ik_des_node`, `ik_geom_node` e `ik_grad_node`
  y sus entradas en `setup.py`.

**Verificación:** con stubs de `rclpy` y los mensajes, se comprobó que cada
tópico publica exactamente el resultado del solver correspondiente. La
ejecución real con ROS queda pendiente (ver sección 6).

### 4.5 Firmware y puente serial eliminados (commit `2d7e3c8`)

- Se eliminó `firmware/control_pata_sin_ros` y el paquete `robot_serial_bridge`.
- El firmware real es `firmware/esp32_servos`, que se comunica por micro-ROS:
  se suscribe a `/servo_commands` y publica `/servo_states` directamente.
- El puente serial solo leía texto de `control_pata` y publicaba en `/servo_states`,
  que nadie consumía. Mantenerlo creaba dos caminos hardware que competían por el
  mismo tópico.

### 4.6 Otros cambios (commit `2d7e3c8`)

- Se eliminó `robot_description/config/joint_names.yaml`: ningún launch ni nodo lo cargaba.
- `JointState.msg`: el comentario mencionaba `Ankle_Joint`, un joint que ya no existe.
  Se actualizó a los nombres reales (`Right_Hip_Roll_Joint`, `Right_Knee_Joint`).
- `control_node.py`: el comentario apuntaba a `bringup_izq.launch.py`, que ya no
  existe; se actualizó a `bringup.launch.py`.

### 4.7 Documentación (commit `2d7e3c8`)

- `ros2_ws/src/ARCHITECTURE.md` se reescribió para describir el código real:
  paquetes, tópicos, límites de motores, signo de la rodilla, paro de emergencia
  pendiente y pasos de prueba.
- `README.md`: se actualizó la estructura de carpetas (incluía subcarpetas que
  no existen en el repositorio) y se añadió `robot_bringup`.

## 5. Hallazgos que no se corrigieron

Estos puntos se documentaron, pero no se cambiaron porque requieren decisión
o pruebas con el robot físico:

1. **Paro de emergencia incompleto.** `/robot/e_stop` no tiene publicador. Cuando
   está activo, `control_node` no publica en `/robot/joint_commands`, así que un
   actuador conserva la última orden. Propuesta: publicar ceros en ese tópico y
   añadir un botón de paro en teleop. Riesgo alto: prueba en banco con el robot
   elevado.
2. **Límites inconsistentes entre nodos.** El límite de cadera-roll es -15° en
   `control_node` y 0° en `teleop_node`. Los límites de teleop están fijos en el
   código, no son parámetros ROS. El documento anterior citaba valores de
   -110° a 130° que no existen en el código; se eliminaron de la documentación.
   **Decisión pendiente:** confirmar los límites reales del hardware.
3. **Signo de la rodilla.** `control_node` invierte la rodilla antes de publicar.
   Hay que confirmar en el robot que teleop no la vuelve a invertir.
4. **Dependencias sin declarar.** Pendiente el paso 10 (no estaba en la lista
   aprobada): mover `gazebo_ros` a `robot_simulation`, y revisar las demás dependencias.

## 6. Verificación realizada y pendiente

| Verificación | Estado |
|---|---|
| Compilación de sintaxis de todos los `.py` | Hecha, sin errores |
| Regresión numérica de cinemática directa e inversa | Hecha, resultados idénticos |
| Nodo `ik_node` con stubs de ROS | Hecha, cada tópico publica el resultado correcto |
| Búsqueda de referencias a archivos eliminados | Hecha, sin referencias restantes |
| `colcon build` de los paquetes | **Pendiente** (requiere ROS 2 Humble) |
| `ros2 launch robot_bringup bringup.launch.py leg:=right/left` | **Pendiente** (requiere ROS 2 Humble) |
| Comparación de `ros2 node list`/`topic list` antes y después | **Pendiente** (requiere ROS 2 Humble) |
| Pruebas con el robot físico (paro de emergencia, servos) | **Pendiente** (requiere hardware) |

## 7. Anexo: resumen de archivos

**Eliminados (36):**
- Launches (9): `bringup_der.launch.py`, `bringup_izq.launch.py`,
  `description_der.launch.py`, `description_izq.launch.py`,
  `display_der.launch.py`, `display_izq.launch.py`, `display_bipedo.launch.py`,
  `gazebo_der.launch.py`, `gazebo_izq.launch.py`.
- Código (10): `kinem_leg_gen.py`, `kinem_leg_rtb.py`, `ik_newton_node.py`,
  `ik_des_node.py`, `ik_geom_node.py`, `ik_grad_node.py`, `balance.py`,
  `sensor_processing.py`, `forward_kinematics.py` y su test.
- Paquete `robot_serial_bridge/` (6 archivos) y carpeta
  `firmware/control_pata_sin_ros/` (8 archivos).
- Otros (3): `joint_names.yaml`, `prueba.txt`, `tests/Hola_mundo.py`.

**Creados (5):** `bringup.launch.py`, `description.launch.py`,
`display.launch.py`, `gazebo.launch.py` y `kinem_common.py`.

**Modificados:** `ik_node.py`, `cinematica_directa_der_izq.py`, seis módulos
`kinem_invers_leg_*.py`, `setup.py` de `robot_kinematics`, `control_node.py`,
`JointState.msg`, `ARCHITECTURE.md`, `README.md` y este informe.

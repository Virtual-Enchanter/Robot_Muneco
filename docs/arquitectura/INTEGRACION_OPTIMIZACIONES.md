# Integración de optimizaciones — rama `feature/integracion-final-optimizaciones`

- **Fecha:** 28 de octubre de 2026
- **Integra:**
  - `feature/optimizacion-arquitectura-muneco` — Santiago Sarria Arcila (2215465), repositorio `Virtual-Enchanter/Robot_Muneco`.
  - `refactor/auditoria-arquitectura` — Hean Felipe Gómez Gonzalias (2230669), repositorio `BABAKAR05/Bipedo_Mu-eco` (11 commits, del 28-oct-2026).
- **`main` no se modificó.**

## 1. Método

Las dos ramas parten del mismo código: el commit `21c3868` de Hean y el commit `9408ab1` de este repositorio tienen el mismo árbol (`24cda5d`). Como los historiales no están conectados, un `git merge --allow-unrelated-histories` no habría visto las eliminaciones de Hean. Por eso:

1. Se rebasaron sus 11 commits sobre `9408ab1` (`git rebase --onto 9408ab1 21c3868`). No hubo conflictos y el árbol final es idéntico al de su rama. Los commits conservan su autor.
2. Se creó `feature/integracion-final-optimizaciones` desde `feature/optimizacion-arquitectura-muneco` y se hizo `git merge --no-ff` de su rama. Así la base común es `9408ab1` y el merge es de tres vías.

## 2. Qué cambió Hean Felipe

| Área | Cambio |
|---|---|
| Multiplexor PWM (PCA9685) | `firmware/esp32_servos`: quitó el PCA9685 (I2C en GPIO 21/22, pin OE en GPIO 4, conversión a *ticks* de 12 bits, constante del oscilador y las librerías Adafruit PWM Servo Driver y BusIO). Los servos se manejan con PWM directo desde los GPIO 18, 19, 23, 25, 26 y 27 (`ESP32Servo`) y el firmware pasa de 3 a 6 servos. |
| Protocolo `/servo_commands` | 6 valores: [IZQ roll, pitch, rodilla, DER roll, pitch, rodilla]. `NaN` mantiene el servo. `teleop_node` envía su pierna y `NaN` para la otra. |
| Camino serie | Eliminó `firmware/control_pata_sin_ros` (8 archivos, 627 líneas, también con PCA9685) y el paquete `robot_serial_bridge` (6 archivos, 177 líneas). Ese nodo leía texto `[POS] ...` por USB y publicaba `/servo_states`, que ningún nodo consume; además competía con el firmware micro-ROS por el mismo tópico. |
| Matemática común | Nuevo `kinem_common.py` (L1..L5, límites, `dh_matrix`, `fk_T`, `joint_limit_warnings`); los 6 módulos `kinem_invers_leg_*` y `cinematica_directa_der_izq.py` lo importan: se borraron 180 líneas copiadas y el módulo común tiene 60. |
| Nodos IK | `ik_node.py` con los 5 métodos y un grupo de callbacks por método; borró los 4 `ik_*_node`. |
| Launches | `bringup.launch.py leg:=right\|left`; `description`, `display` y `gazebo` con `model:=right\|left\|bipedo`. |
| Código sin uso | Las mismas eliminaciones que Santiago: `kinem_leg_gen`, `kinem_leg_rtb`, `forward_kinematics` + test, `balance`, `sensor_processing`, `prueba.txt`, `tests/Hola_mundo.py`, `joint_names.yaml`. |
| Documentación | `JointState.msg` (comentario con `Ankle_Joint`), `ARCHITECTURE.md`, `README.md` y su informe `docs/INFORME_REFACTORIZACION_ARQUITECTURA.md`. |

## 3. Archivos en los que coincidimos y cómo se resolvió

| Archivo | Santiago | Hean | Resultado |
|---|---|---|---|
| `robot_kinematics/kinematics_node.py` + `ik_node.py` | Reenvío + 5 métodos en `kinematics_node`; borró `ik_node` | 5 métodos en `ik_node`, grupo por método | **Ambas:** un solo `kinematics_node` con la tabla `SOLVERS` y un grupo de callbacks por método más uno para el reenvío. Usa L1..L5 y `joint_limit_warnings` de `kinem_common`. `ik_node.py` se elimina (conflicto modificar/eliminar). |
| `robot_kinematics/setup.py` | 1 ejecutable | 2 (`kinematics_node`, `ik_node`) | 1 ejecutable: `kinematics_node`. |
| `kinem_invers_leg_*.py`, `cinematica_directa_der_izq.py` | Sin cambios | Importan `kinem_common` | Versión de Hean. Solo se corrige el comentario "la usa ik_node" → `kinematics_node`. |
| `robot_bringup/launch/bringup.launch.py` | `bringup_pierna.launch.py leg:=izq\|der` | `bringup.launch.py leg:=right\|left` | Un solo launch: `bringup_pierna`, que ahora acepta también `left\|right`. El de Hean cargaba `urdf_bipedo/muneco_bipedo.urdf.xacro` (eliminado) y lanzaba `ik_node`. `bringup_muneco` acepta los mismos alias. |
| `robot_description/launch/display.launch.py` | Modelo único `urdf_muneco` | `model:=right\|left\|bipedo` | Versión de Santiago: los tres modelos de Hean apuntan a URDF eliminados (patas V2.x del bípedo y V3 con Y arriba). |
| `robot_description/launch/description.launch.py`, `gazebo.launch.py` | — | Nuevos | No se incluyen: usan los mismos URDF eliminados y `gazebo.launch.py` es Gazebo Classic (`gazebo_ros`). Los reemplazan `display.launch.py` y `robot_simulation/gazebo_muneco.launch.py`. |
| `robot_control/control_node.py` | Parámetros en caché | Comentario `bringup_izq` → `bringup` | Código de Santiago; comentario apuntando a `bringup_pierna`/`bringup_muneco` y `urdf_muneco`. |
| `robot_serial_bridge/`, `firmware/control_pata_sin_ros/` | Se conservaban | Eliminados | Eliminados (Hean). No había referencias en launches ni nodos. |
| `firmware/esp32_servos/`, `teleop_node.py`, `JointState.msg` | Sin cambios | Modificados | Versión de Hean. En el README del firmware, `bringup.launch.py` → `bringup_pierna`/`bringup_muneco`. |
| `ros2_ws/src/ARCHITECTURE.md`, `README.md` | Reescritos | Reescritos | Unión de los dos: estructura de Santiago más `kinem_common`, firmware de 6 servos, límites, signo de rodilla y e-stop de Hean. |

## 4. Verificación (sin Gazebo)

| Prueba | Resultado |
|---|---|
| `colcon build` | 7/7 paquetes, 0 advertencias. |
| Regresión de cinemática: módulos originales (`9408ab1`) contra los fusionados | IK 428/428 y FK 108/108 idénticos (tolerancia 1e-12). |
| `kinematics_node` por tópicos, pierna izquierda y derecha | Todas las respuestas iguales a la llamada directa; reenvío correcto. |
| Concurrencia: gradiente sin converger (~0,9 s) + algebraico, geométrico y reenvío | Fusionado: 17 ms, 20 ms y 5 ms. Antes de integrar: 884 ms, 885 ms y 4 ms. |
| Launch `bringup_pierna` y `bringup_muneco` con `izq`, `der`, `left`, `right` | Mismos nodos y joints; `leg:=x` da error claro. |
| Referencias a archivos eliminados | Ninguna en launches, `setup.py`, `package.xml` ni imports. |
| RAM de `kinematics_node` | 66,3 MB (igual que antes de integrar: 66,4 MB). |

La prueba en Gazebo de la rama `feature/optimizacion-arquitectura-muneco` sigue vigente: la integración no cambió el URDF, `gazebo_muneco`, `controllers.yaml`, `sim_bridge` ni `control_node` (salvo un comentario).

## 5. Observaciones

- Los GPIO de servos (18, 19, 23, 25, 26, 27) son los mismos que usa el banco SPI doble con 2× MCP2518FD (VSPI 18/19/23; HSPI CS 25, INT 26, MISO 27). Hoy son placas distintas; si algún día se juntan en una sola ESP32, hay que reasignar pines.
- `servo_params.h` copia los límites del esquema con PCA9685: hay que calibrarlos con PWM directo antes de mover el muñeco físico.

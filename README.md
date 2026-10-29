# Robot Muñeco — Arquitectura ROS 2 optimizada

Muñeco bípedo de **2 piernas × 3 GDL** (cadera roll, cadera pitch, rodilla) con ROS 2 Humble, simulación en Gazebo y control por interfaz gráfica.

Este repositorio contiene la **optimización de la arquitectura ROS 2 del muñeco (Entregable 1, corte 3)**: recuento de paquetes, nodos y tópicos; eliminación de módulos de los cortes 1 y 2; unificación de nodos redundantes, e integración del muñeco con Gazebo.

**Responsables:**
- Santiago Sarria Arcila (2215465): cinemática, control, bringup, modelo URDF y Gazebo.
- Hean Felipe Gómez Gonzalias (2230669): matemática común de cinemática, firmware de servos sin PCA9685 y eliminación del camino serie (`robot_serial_bridge`). Su trabajo viene de [`BABAKAR05/Bipedo_Mu-eco`](https://github.com/BABAKAR05/Bipedo_Mu-eco) (rama `refactor/auditoria-arquitectura`).

Robótica, Universidad Autónoma de Occidente.
**Código base:** repositorio del equipo [`juanperezpomar-source/Bipedo_Mu-eco`](https://github.com/juanperezpomar-source/Bipedo_Mu-eco) y workspace `muneco_ws` (Gazebo).

---

## 🌿 Ramas

| Rama | Contenido |
|---|---|
| `main` | Código original del muñeco, tal como estaba antes de la optimización. |
| `feature/optimizacion-arquitectura-muneco` | Optimización de Santiago (7 commits, uno por paquete). |
| `feature/integracion-final-optimizaciones` | **Versión final:** la optimización de Santiago fusionada con la auditoría de Hean Felipe (sus 11 commits conservan su autoría). |

Ver todos los cambios juntos: **[main...feature/integracion-final-optimizaciones](../../compare/main...feature/integracion-final-optimizaciones)**

---

## 📊 Resultado

| Métrica | Antes | Después |
|---|---|---|
| Paquetes ROS 2 | 8 | **7** |
| Nodos corriendo con una pierna | 11 | **6** |
| Procesos de cinemática | 6 | **1** |
| RAM de la cinemática | 417,9 MB | **69,9 MB** |
| RAM total de los nodos del proyecto | 645,9 MB | **294,0 MB** |
| Ejecutables (entry points) | 11 | **4** |
| Archivos launch | 10 | **4** |
| Firmwares para los mismos servos | 2 | **1** |
| Copias de la matemática DH / límites | ~6 | **1** (`kinem_common.py`) |
| Respuesta de un método IK mientras el gradiente itera | ~880 ms | **~20 ms** |
| Advertencias de `colcon build` | 1 | **0** |
| Mover una pierna en Gazebo desde la interfaz | no | **sí** (error 0,000 rad) |

Medido con la arquitectura en ejecución (una pierna, sin RViz).

---

## 🔧 Qué se cambió

### `robot_kinematics` — 6 nodos → 1
- **Unificación (Santiago):** `ik_node`, `ik_des_node`, `ik_geom_node`, `ik_newton_node`, `ik_grad_node` y `kinematics_node` pasan a ser un solo **`kinematics_node`**. Los cinco nodos IK eran el mismo código copiado; solo cambiaban la función de cinemática inversa y el prefijo del tópico.
- **Tabla `SOLVERS`:** cada método (algebraico, desacople, geométrico, Newton-Raphson, gradiente descendente) es una entrada de la tabla, y el nodo crea sus tópicos en un bucle.
- **Mismos tópicos:** `/robot/ik*_target` → `/robot/ik*_result` no cambian, así que `teleop_node` no se tocó. Se conserva el reflejo de *z* para la pierna derecha.
- **Un grupo de callbacks por método (Hean):** el gradiente descendente (~1 s cuando no converge) ya no bloquea ni el reenvío `/robot/command → /robot/joint_targets` ni los otros cuatro métodos.
- **`kinem_common.py` (Hean):** única fuente de L1..L5, límites articulares, matriz DH, cinemática directa y avisos de límites, que estaban copiados en ~6 módulos.
- **Eliminados:** `kinem_leg_gen.py` (FK duplicada, nadie la importaba) y `kinem_leg_rtb.py` (script de prueba que dependía de `roboticstoolbox` sin declararlo).

### `robot_bringup` — un launch por modo
- **`bringup_pierna.launch.py leg:=izq|der`** reemplaza a `bringup_der` y `bringup_izq`, que eran el mismo archivo (~990 líneas). También acepta `leg:=left|right`, los nombres de la rama de Hean.
- **`bringup_muneco.launch.py`** (nuevo): Gazebo más la arquitectura completa. Desde la interfaz se mueve la pierna simulada.
- **Eliminado:** `test_injector_node`, un esqueleto del corte 1 que competía con el teleop en `/robot/command`.

### `robot_description` — un solo modelo
- **Modelo único** `urdf/urdf_muneco/muneco_bipedo_v3.urdf`: Z hacia arriba y bloque `ros2_control` con `gz_ros2_control`. Es el mismo archivo para RViz y Gazebo.
- **Reemplaza** al URDF V3 anterior, que estaba exportado con Y hacia arriba y necesitaba un xacro que lo rotara.
- **Eliminados:** los URDF y mallas por pata V2.x (son del bípedo, no del muñeco), 7 launch de visualización y Gazebo Classic, y `joint_names.yaml`.
- **Nuevo:** `display.launch.py` (RViz con Fixed Frame `Base_link`).

### `robot_simulation` — Gazebo funcionando
- **`launch/gazebo_muneco.launch.py`:** abre Gazebo, hace el spawn y carga los controladores **en cadena por eventos**, sin esperas fijas de 8 y 10 s. Funciona en Fortress (Humble) y Harmonic (Jazzy).
- **`config/controllers.yaml`:** `joint_group_position_controller` con las 6 articulaciones. Se movió aquí porque en Jazzy una ruta que contiene `robot_description` hace que los controladores no carguen.
- **`sim_bridge` corregido:** enviaba 3 valores a un controlador de 6 joints y Gazebo los descartaba. Ahora ubica cada valor por nombre de joint.
- **Reinicio del robot:** servicio `/sim/reset_robot` en `sim_bridge` y botón **⟲ Reiniciar robot** en la interfaz. Lleva las 6 articulaciones a 0, espera a que queden quietas y devuelve el modelo a la pose del spawn con `set_pose` de Gazebo (funciona en Fortress y Harmonic).

### `robot_control`
- **Parámetros:** se leen una vez y se validan con `ros2 param set`. Antes el bucle de 50 Hz hacía 150 lecturas por segundo.
- **Eliminados:** `balance.py` y `sensor_processing.py`, que nadie importaba.
- **`motor_control.py`** se integra en `pid_controllers.py`.
- **Se conserva** la inversión del *Knee Pitch* y todos los comentarios originales.

### Hardware de servos (Hean)
- **Sin PCA9685:** `firmware/esp32_servos` mueve los 6 servos con PWM directo desde los GPIO 18, 19, 23, 25, 26 y 27 (`ESP32Servo`). Se quitan el bus I2C, el pin OE y las librerías de Adafruit.
- **Protocolo de 6 valores:** `/servo_commands` = [IZQ roll, pitch, rodilla, DER roll, pitch, rodilla]; `NaN` mantiene el servo. `teleop_node` envía su pierna y `NaN` para la otra.
- **Eliminados:** el paquete `robot_serial_bridge` y el firmware `control_pata_sin_ros`: eran un segundo camino al mismo hardware (también con PCA9685) y su `/servo_states` no lo leía ningún nodo.

### Limpieza
- **Eliminados:** `prueba.txt`, `tests/Hola_mundo.py` y `robot_teleop/forward_kinematics.py` (DH antiguo sin uso).
- **Nuevo:** `scripts/launch_gazebo_muneco.sh` (compila y abre Gazebo desde cualquier carpeta).

La justificación detallada está en [`docs/arquitectura/INTEGRACION_OPTIMIZACIONES.md`](docs/arquitectura/INTEGRACION_OPTIMIZACIONES.md), [`docs/arquitectura/OPTIMIZACION_ARQUITECTURA.md`](docs/arquitectura/OPTIMIZACION_ARQUITECTURA.md) y [`docs/INFORME_REFACTORIZACION_ARQUITECTURA.md`](docs/INFORME_REFACTORIZACION_ARQUITECTURA.md). El detalle de nodos y tópicos está en [`ros2_ws/src/ARCHITECTURE.md`](ros2_ws/src/ARCHITECTURE.md).

---

## 🗺️ Arquitectura

```mermaid
flowchart LR
    T[teleop_node<br/>interfaz] -- /robot/command --> K[kinematics_node<br/>reenvío + 5 métodos IK]
    T -- /robot/ik*_target --> K
    K -- /robot/ik*_result --> T
    K -- /robot/joint_targets --> C[control_node<br/>límites · PID · rodilla invertida]
    C -- /robot/joint_states --> T
    C -- /robot/joint_commands --> S[sim_bridge<br/>3 joints → 6]
    S -- /joint_group_position_controller/commands --> G[(Gazebo<br/>gz_ros2_control)]
    G -- /joint_states --> R[robot_state_publisher<br/>TF · RViz]
    T -- /servo_commands 6 valores --> E[ESP32<br/>6 servos, PWM directo]
```

---

## 🚀 Cómo correrlo (rama `feature/integracion-final-optimizaciones`)

```bash
git checkout feature/integracion-final-optimizaciones
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

| Qué | Comando |
|---|---|
| Muñeco en Gazebo + interfaz (pierna izquierda) | `ros2 launch robot_bringup bringup_muneco.launch.py` |
| Igual, pierna derecha | `ros2 launch robot_bringup bringup_muneco.launch.py leg:=der` |
| Solo Gazebo | `ros2 launch robot_simulation gazebo_muneco.launch.py` |
| Una pierna en RViz | `ros2 launch robot_bringup bringup_pierna.launch.py leg:=izq` |
| Solo el modelo | `ros2 launch robot_description display.launch.py` |
| Todo en un paso (compila y abre Gazebo) | `./scripts/launch_gazebo_muneco.sh` |
| ESP32 de servos (micro-ROS) | `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6` |

Mover la pierna sin la interfaz (con `bringup_muneco.launch.py teleop:=false`):

```bash
ros2 topic pub --once /robot/command robot_interfaces/msg/RobotCommand \
  "{mode: 0, position: [0.3, 0.4, 0.5], velocity: [0.0, 0.0, 0.0]}"
ros2 topic echo /joint_states   # Left: 0.3, 0.4, -0.5 (rodilla invertida)
```

Si el muñeco se cae en Gazebo, el botón **⟲ Reiniciar robot** de la interfaz (debajo de los sliders) pone las articulaciones en 0° y lo vuelve a poner de pie en la pose inicial.

Paro de emergencia: `ros2 topic pub --once /robot/e_stop std_msgs/msg/Bool "{data: true}"`

---

## ✅ Verificación

- **Compilación:** `colcon build` compila los 7 paquetes sin advertencias.
- **Grafo:** 6 nodos por pierna; los mismos 24 tópicos de antes.
- **Cinemática:** los módulos con `kinem_common.py` dan resultados idénticos a los originales (428/428 cinemática inversa, 108/108 cinemática directa). El nodo responde igual que llamar a cada función, en las dos piernas.
- **Concurrencia:** con el gradiente iterando, el método algebraico responde en ~20 ms (antes esperaba ~880 ms).
- **Límites:** cambiados en vivo con `ros2 param set`; una lista con tamaño inválido se rechaza.
- **Gazebo:** `/robot/command [0.3, 0.4, 0.5]` dejó los joints izquierdos en `[0.300, 0.400, −0.500]` con error 0,000 rad, y la pierna derecha quieta (rama `feature/optimizacion-arquitectura-muneco`; la integración no cambió esa ruta).

Probado en ROS 2 Jazzy + Gazebo Harmonic. En el laboratorio (Humble + Fortress) hay que confirmar con los mismos comandos.

---

## 📌 Pendiente (siguiente actividad)

- **UNO Q e IMU:** comunicación con Arduino UNO Q (receptor iA8 para servos) e IMU BNO085.
- **Paro de emergencia:** nadie publica `/robot/e_stop`, y con el paro activo `control_node` deja de publicar `/robot/joint_commands` (el actuador conserva la última orden). Además, el teleop publica `/servo_commands` directo, sin pasar por el e-stop ni los límites de `control_node`.
- **Límite de cadera roll:** alinear control (−0,2618 rad), interfaz (0°) y URDF (−0,1396 rad).
- **Servos:** calibrar los límites de `servo_params.h` con PWM directo (se copiaron del esquema con PCA9685).
- **Modo PID** (`verify_mode=False`): es un modelo de prueba; reemplazarlo por el control real.
- **Caminata:** generador de pasos para las dos piernas (meta: 6 de noviembre).

---

## 📁 Estructura

```text
Robot_Muneco/
├── ros2_ws/src/
│   ├── robot_interfaces/     mensajes (RobotCommand, JointTarget, JointState, IK*)
│   ├── robot_description/    URDF del muñeco + mallas + display.launch.py
│   ├── robot_kinematics/     kinematics_node (reenvío + 5 métodos IK) + kinem_common
│   ├── robot_control/        control_node + PID
│   ├── robot_simulation/     Gazebo (launch + controllers.yaml) + sim_bridge
│   ├── robot_teleop/         interfaz gráfica
│   └── robot_bringup/        bringup_pierna y bringup_muneco
├── firmware/esp32_servos/    ESP32 + micro-ROS, 6 servos con PWM directo
├── scripts/                  launch_gazebo_muneco.sh
├── docs/                     arquitectura, informes y hojas de datos
├── matlab/                   modelos y análisis en MATLAB
├── tests/                    pruebas generales del proyecto
├── .gitignore
├── Robot_bipedoCode.code-workspace
└── README.md
```

No se suben `ros2_ws/build/`, `ros2_ws/install/` ni `ros2_ws/log/` (ver `.gitignore`).

# Firmware ESP32 - Control de 6 servos via micro-ROS

Controla los 6 servos del bípedo (3 por pierna) desde una **ESP32 NodeMCU-32S**
usando **micro-ROS** con transporte **serial (USB)**. El PWM sale directamente
de los GPIO de la ESP32, sin módulo PCA9685.

## Arquitectura

```
+------------------+   /servo_commands    +--------------------------------+
|   ROS2 (host)    | -------------------> |  micro-ROS Agent (USB -> host)  |
|  teleop / ik     |  Float32MultiArray   +--------------------------------+
+------------------+                                        |
                                                            | serial
                                                            v
+-------------------------------------------------------------+
|                      ESP32 (firmware)                       |
|  - Suscribe a /servo_commands (6 valores)                   |
|  - PWM 50 Hz directo a GPIO 18, 19, 23, 25, 26, 27          |
|  - Publica estado en /servo_states                          |
+-------------------------------------------------------------+
```

## Contenido

| Archivo | Descripcion |
|---|---|
| `platformio.ini` | Configuracion de PlatformIO (board, micro-ROS, ESP32Servo) |
| `include/servo_params.h` | Pin, reposo y límites de cada servo; orden del mensaje |
| `include/pins.h` | LED indicador |
| `include/helpers.h` | Conversión de grados a microsegundos |
| `src/main.cpp` | Firmware: suscripción, control de servos y publicación de estado |

## Conexion de los servos

| Servo | Articulacion | GPIO | Posicion en el mensaje |
|---|---|---|---|
| S1 | IZQ Hip Roll (cad roll) | 18 | data[0] |
| S3 | IZQ Hip Pitch (cad pitch) | 23 | data[1] |
| S2 | IZQ Knee (rod pitch) | 19 | data[2] |
| S5 | DER Hip Roll (cad roll) | 26 | data[3] |
| S6 | DER Hip Pitch (cad pitch) | 27 | data[4] |
| S4 | DER Knee (rod pitch) | 25 | data[5] |

Las señales van directo al GPIO de cada servo. La alimentación de los servos
es **externa de 5 V** (no desde el pin 5V de la ESP32, porque seis servos a la
vez consumen más de lo que da la placa). El GND de la fuente debe unirse al GND
de la ESP32.

> Verifica con el multímetro o el osciloscopio que cada GPIO llega a su servo
> correcto antes de conectar la alimentación.

## Protocolo `/servo_commands`

- **Tipo:** `std_msgs/Float32MultiArray`, con 6 valores en grados.
- **Orden:** `[IZQ Hip Roll, IZQ Hip Pitch, IZQ Knee, DER Hip Roll, DER Hip Pitch, DER Knee]`.
- **NaN:** un valor `NaN` significa "no cambiar este servo". El firmware
  conserva su última posición objetivo. El teleop lo usa para la pierna que no controla.
- Los valores se limitan a los rangos de `servo_params.h`.

Ejemplo (mueve solo la pierna izquierda, deja la derecha quieta):

```bash
ros2 topic pub --once /servo_commands std_msgs/msg/Float32MultiArray \
  "{data: [90.0, 90.0, 90.0, nan, nan, nan]}"
```

## Pasos

### 1. Flashear firmware

Con la ESP32 conectada al puerto `/dev/ttyUSB0`:

```bash
cd firmware/esp32_servos
pio run -t upload --upload-port /dev/ttyUSB0
```

### 2. Levantar el micro-ROS Agent (serial)

En una terminal aparte, desactiva cualquier monitor serial y lanza:

```bash
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

Deberia verse algo como `Client connected` cuando la ESP32 se conecte.

### 3. Verificar nodo en ROS2

En otra terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 node list            # deberia aparecer /esp32_servo_controller
ros2 topic list           # deberia estar /servo_commands y /servo_states
ros2 topic echo /servo_states
```

### 4. Mover los servos

```bash
# Mueve los 6 servos a 90 grados
ros2 topic pub --once /servo_commands std_msgs/msg/Float32MultiArray \
  "{data: [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]}"
```

## Tipos de mensaje

El firmware usa `std_msgs/Float32MultiArray`, que ya viene incluido en la librería
micro-ROS estandar, por lo que no requiere generar mensajes custom.

## Notas

- Los límites y el reposo de cada servo están copiados del esquema anterior y
  son **nominales, pendientes de calibrar**. Ver `include/servo_params.h`.
- La compilación de la stack micro-ROS tarda la primera vez (descarga y compila
  todos los paquetes). Las siguientes compilaciones son rapidas.
- **IMPORTANTE:** no uses un `extra_micro_ros.meta` con opciones del host
  (`-DUHANDLER_AGENT_URL`, `-DMICROROS_TRANSPORT`), rompe el build del firmware y
  la ESP32 no bootea la app (se queda en modo descarga dando ruido `0x80`).
  El transporte se configura solo con `board_microros_transport = serial`.
- Despues de flashear, si la ESP32 queda en modo descarga (ruido constante en el
  puerto), haz un reset manual cerrando y abriendo el puerto (DTR/RTS) antes de
  lanzar el agente, para que bootee la app.
- El transporte serial usa el mismo puerto USB que la consola. Cierra el monitor
  serial (`pio device monitor`) antes de lanzar el agent serial.
- El firmware publica `/servo_states` cada 0.5 s para confirmar al host que la
  ESP32 esta viva y conectada.

## Integracion con los sliders del teleop

El `teleop_node` del workspace ROS2 (lanzado por `bringup.launch.py`) publica en
`/servo_commands` seis valores: los tres de su pierna y `NaN` para la otra. El
firmware recibe así las órdenes de cada pierna sin mover la contraria.

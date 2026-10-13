# vtol_tiltrotor_description

URDF/xacro de un **VTOL tiltrotor de tres motores tipo SNOOP** para ROS 2 Jazzy
(Gazebo Harmonic) o Humble (Gazebo Fortress, ver más abajo).

> **Este paquete ahora lo vuela `vtol_control`.** Consulta el README del
> workspace para los controles de teclado y
> [`../../README.md#tres-defectos-del-modelo`](../../README.md#tres-defectos-del-modelo)
> para los tres defectos del modelo que hubo que corregir antes de que simulara:
> los plugins `LiftDrag` estaban asociados a links que no sobreviven a la
> conversión de URDF a SDF, las ganancias de los servos eran demasiado bajas para
> alcanzar sus consignas, y los servos se quedan bloqueados en los límites de sus
> articulaciones. `gazebo_fortress.xacro` es la capa de plugins para Humble;
> `gazebo_harmonic.xacro` es el mismo archivo con los nombres de Harmonic.

Basado en el **Cannon SNOOP 18**, la aeronave de la foto de referencia. Según el
fabricante es un tilt-rotor de tres motores: los dos rotores delanteros pasan de
vertical (VTOL) a horizontal (vuelo de avión), y el **rotor trasero se inclina
para dirigir**.

## Disposición

| | |
|---|---|
| Rotores delanteros | En brazos que sobresalen por delante del borde de ataque del ala. Se inclinan sobre el eje de **cabeceo**: `0` = estacionario, `pi/2` = crucero. |
| Rotor trasero | Sobre un pilón encima del botalón de cola. Se inclina sobre el eje de **alabeo**, ±30°. **Es el actuador de guiñada en estacionario**, al estilo tricóptero. *No* se inclina para la transición. |
| Cola | Cola en V con dos ruddervators. |
| Ala | 1,70 m de envergadura, con estrechamiento, puntas exteriores en flecha y alerones. |
| Tren | Cuatro patines con puntas de alta visibilidad, los cuatro en contacto en z = −0,240. |

## Valores verificados

Calculados a partir del URDF expandido, no estimados:

```
masa                5.496 kg      (MTOW del SNOOP 18: 5.5 kg)
envergadura         1.700 m       (publicado: 1700 mm)
longitud            1.310 m       (longitud de fuselaje publicada: 1310 mm)
centro de masas     x = -0.004 m, z = +0.036 m
equilibrio en estacionario  16.0 N por rotor delantero, 21.8 N el trasero
empuje máximo       36.0 N por rotor  ->  T/W = 2.00
links / joints      26 / 25
```

Articulaciones actuadas:

```
tilt_left_joint         revolute  eje y   [-0.17, 1.571]   transición
tilt_right_joint        revolute  eje y   [-0.17, 1.571]   transición
rear_tilt_joint         revolute  eje x   [-0.524, 0.524]  GUIÑADA en estacionario
aileron_left_joint      revolute  eje y   [-0.44, 0.44]
aileron_right_joint     revolute  eje y   [-0.44, 0.44]
ruddervator_left_joint  revolute  eje y   [-0.52, 0.52]
ruddervator_right_joint revolute  eje y   [-0.52, 0.52]
rotor_left_joint        continuous
rotor_right_joint       continuous
rotor_rear_joint        continuous
```

Mezcla de la cola en V: **profundidad** = ambos ruddervators con el mismo signo;
**timón** = signos opuestos.

## Lee esto antes de suponer que vuela

1. **Un URDF no tiene aerodinámica.** La sustentación, la resistencia y el empuje
   vienen por completo de `gazebo_harmonic.xacro`. Si lo quitas, tienes un modelo
   estático.
2. **No hay controlador en *este* paquete.** Ni estabilización, ni mezclador, ni
   secuencia de transición. `vtol_control` aporta las tres cosas; otra opción es
   usar PX4/ArduPilot SITL.
3. **Los coeficientes aerodinámicos son valores genéricos de placa plana**, no
   datos del SNOOP: `cla=4.75`, `alpha_stall=0.30`. Ajústalos con datos reales o
   con XFLR5 antes de dar por buenos los números de crucero.
4. **En estacionario el rotor trasero empuja un 36 % más que cada rotor
   delantero**, por la longitud de los brazos. Los VTOL tricóptero reales lo
   resuelven con una hélice de cola más grande o un motor de cola más potente; si
   quieres igualarlos, mueve `rear_x` a `-0.720`.
5. **La geometría son primitivas.** Cambia `<geometry>` por mallas cuando las
   tengas, pero conserva los marcos de los links: el `<cp>` de `LiftDrag` está en
   el marco del link, y los marcos del ala y de la cola en V están colocados a
   propósito en el centro aerodinámico de cada panel.

## Compilar y visualizar

```bash
cd ~/ros2_ws/src && cp -r vtol_tiltrotor_description .
cd ~/ros2_ws && colcon build --packages-select vtol_tiltrotor_description
source install/setup.bash
ros2 launch vtol_tiltrotor_description display.launch.py
```

Mueve `tilt_left_joint` / `tilt_right_joint` de 0 a 1,57 para ver la transición,
y `rear_tilt_joint` para la vectorización de guiñada.

## Simular (Jazzy + Gazebo Harmonic)

```bash
xacro urdf/vtol_tiltrotor.urdf.xacro use_gazebo:=true > /tmp/vtol.urdf
gz sim -r empty.sdf
ros2 run ros_gz_sim create -file /tmp/vtol.urdf -name vtol -z 0.3
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix vtol_tiltrotor_description)/share/vtol_tiltrotor_description/config/gz_bridge.yaml
```

```bash
# velocidades de rotor aproximadas para estacionario (rad/s): [front_left, front_right, rear]
ros2 topic pub /vtol/command/motor_speed actuator_msgs/msg/Actuators \
  "{velocity: [1000.0, 1000.0, 1167.0]}"

# guiñada a la derecha en estacionario
ros2 topic pub /vtol/rear_tilt/cmd std_msgs/msg/Float64 "{data: 0.35}"

# iniciar la transición
ros2 topic pub /vtol/tilt_left/cmd  std_msgs/msg/Float64 "{data: 1.57}"
ros2 topic pub /vtol/tilt_right/cmd std_msgs/msg/Float64 "{data: 1.57}"
```

No mantendrá la actitud por sí solo. Es lo esperado: ver la limitación 2. Para
vuelo estabilizado usa `ros2 launch vtol_control vtol_sim.launch.py`, que hace el
bridge anterior y añade el controlador.

## Humble

Humble va con Gazebo Fortress. En `gazebo_harmonic.xacro` sustituye cada nombre
de archivo `gz-sim-*` por `ignition-gazebo-*` y cada `gz::sim::systems::` por
`ignition::gazebo::systems::`. Por lo demás, los nombres de los parámetros son
idénticos. Humble + Gazebo **Classic** 11 usa un conjunto de plugins
completamente distinto (`libgazebo_motor_model.so`, `libLiftDragPlugin.so`) que
requiere la biblioteca de plugins `PX4/PX4-SITL_gazebo-classic` en
`GAZEBO_PLUGIN_PATH`.

## Si el objetivo de verdad es volar transiciones

Usa PX4 SITL. Su airframe `tiltrotor` tiene la misma disposición de dos rotores
delanteros basculantes, y la máquina de estados de transición VTOL ya está
escrita y probada en vuelo; conéctala a ROS 2 con `px4_ros_com` / uXRCE-DDS. Ten
en cuenta que el tiltrotor estándar de PX4 hace la guiñada con inclinación
diferencial de los rotores delanteros, así que si quieres el rotor de cola
basculante del SNOOP necesitarás un mezclador propio en cualquier caso. Deja este
URDF para visualización, TF, montaje de sensores y trabajo con `ros2_control`.

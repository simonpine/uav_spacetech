# VTOL tiltrotor: simulador

Simulador de un dron VTOL (despega en vertical como un dron y vuela como un
avión). Se pilota con el teclado y la **barra espaciadora** cambia entre modo
dron y modo avión.

Corre sobre **ROS 2 Humble** y **Gazebo Fortress**, y está escrito en Python.

## Ejecutarlo

```bash
./fly.sh               # simulador con ventana + piloto de teclado
./fly.sh --headless    # sin ventana de Gazebo (se sigue pudiendo volar)
./fly.sh --demo        # vuelo automático de prueba, sin teclado
./fly.sh --rviz        # abre también RViz
```

La primera vez compila el proyecto (`colcon build`). Después abre Gazebo y una
segunda terminal con el piloto de teclado. **Las teclas funcionan en esa
segunda terminal.**

## Teclas

| Tecla | Acción |
|---|---|
| `ENTER` | Encender / apagar motores |
| `T` | Despegue automático hasta 40 m |
| `L` | Aterrizaje automático |
| `ESPACIO` | Cambiar entre modo dron y modo avión (necesita al menos 25 m de altura) |
| `W` / `S` | Adelante / atrás (en modo avión: bajar / subir el morro) |
| `A` / `D` | Inclinarse a la izquierda / derecha |
| `Q` / `E` | Girar a la izquierda / derecha |
| `R` / `F` | Dron: subir / bajar. Avión: más / menos velocidad |
| `H` | Nivelar y quedarse quieto |
| `Z` | Soltar todos los mandos |
| `X` | Apagar motores de inmediato (emergencia) |
| `?` | Mostrar la ayuda (en inglés) |

**Vuelo típico:** `ENTER` → `T` → esperar a que llegue a 40 m → `ESPACIO`
(pasa a avión) → volar → `ESPACIO` (vuelve a dron) → `L`.

## Cómo está armado

Hay tres procesos que se comunican por **topics** de ROS 2 (canales de
mensajes tipo publicar/suscribir):

```
 ┌──────────────────┐  mandos   ┌────────────────────┐  motores y   ┌──────────────┐
 │ keyboard_teleop  │ ────────► │ flight_controller  │  servos      │   Gazebo     │
 │ (lee el teclado) │           │ (decide qué hacer  │ ───────────► │ (simula la   │
 │                  │ ◄──────── │  con los motores)  │ ◄─────────── │  física)     │
 └──────────────────┘  estado   └────────────────────┘  sensores    └──────────────┘
```

- **`keyboard_teleop`** convierte las teclas en mensajes. No sabe nada de física.
- **`flight_controller`** es el cerebro. Recibe los sensores 250 veces por
  segundo y calcula qué velocidad dar a cada motor y qué ángulo a cada servo
  para obedecer los mandos sin caerse.
- **Gazebo** simula el avión y publica lo que "sienten" sus sensores. Un
  *bridge* traduce sus mensajes a topics de ROS 2.

`demo_flight` puede ocupar el lugar de `keyboard_teleop`: envía exactamente los
mismos mensajes, pero desde un guion.

### Topics

| Topic | Tipo | De → a | Contenido |
|---|---|---|---|
| `/vtol/manual_input` | `geometry_msgs/Twist` | teleop → controlador | Mandos del piloto |
| `/vtol/mode_request` | `std_msgs/String` | teleop → controlador | Cambio de modo, despegue, aterrizaje |
| `/vtol/arm_request` | `std_msgs/Bool` | teleop → controlador | Encender / apagar motores |
| `/vtol/status` | `std_msgs/String` | controlador → teleop | Estado actual (modo, altura, velocidad) |
| `/vtol/imu` | `sensor_msgs/Imu` | Gazebo → controlador | Orientación y giro (250 Hz) |
| `/vtol/odom` | `nav_msgs/Odometry` | Gazebo → controlador | Posición y velocidad |
| `/vtol/command/motor_speed` | `actuator_msgs/Actuators` | controlador → Gazebo | Velocidad de los 3 motores |
| `/vtol/<servo>/cmd` | `std_msgs/Float64` | controlador → Gazebo | Ángulo de cada servo |

Para ver lo que pasa mientras vuela:

```bash
ros2 topic echo /vtol/status
```

## Estructura del repositorio

| Ruta | Qué contiene |
|---|---|
| `fly.sh` | Script que arranca todo |
| `src/vtol_control/` | Controlador, piloto de teclado y vuelo de prueba (Python) |
| `src/vtol_control/config/flight_controller.yaml` | **Todos los parámetros ajustables** del controlador |
| `src/vtol_tiltrotor_description/` | El modelo 3D del avión y la configuración de Gazebo ([README](src/vtol_tiltrotor_description/README.md)) |
| `tools/` | Scripts para recalcular constantes físicas (rara vez hacen falta) |
| `docs/notas-tecnicas.md` | Detalles de control y física, para quien quiera profundizar |

Archivos principales de `src/vtol_control/vtol_control/`:

| Archivo | Responsabilidad |
|---|---|
| `flight_controller.py` | Nodo controlador |
| `airframe.py` | Constantes físicas del avión (masa, geometría, etc.) |
| `keyboard_teleop.py` | Nodo de piloto de teclado |
| `demo_flight.py` | Vuelo automático de prueba |

## Cambiar parámetros

Edita `src/vtol_control/config/flight_controller.yaml` y vuelve a lanzar
`./fly.sh`. No hace falta recompilar. Cada parámetro tiene un comentario que
explica para qué sirve. Los más útiles:

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `takeoff_altitude` | 40 m | Altura del despegue automático |
| `trans_min_altitude` | 25 m | Altura mínima para pasar a modo avión |
| `v_target_default` | 20 m/s | Velocidad inicial en modo avión |
| `max_climb_rate` | 3,5 m/s | Velocidad máxima de subida en modo dron |

## Prueba automática

Comprueba que el avión sigue volando bien después de un cambio. Recorre 15
etapas (despegue, cambio a avión, giros, vuelta a dron, aterrizaje) y falla si
alguna no se cumple.

```bash
./fly.sh --demo
```

O a mano, guardando los datos del vuelo en un CSV:

```bash
ros2 launch vtol_control vtol_sim.launch.py gui:=false &
ros2 run vtol_control demo_flight --csv /tmp/flight.csv
```

## Problemas comunes

| Síntoma | Solución |
|---|---|
| No llegan mensajes por ningún topic, o el avión sube muy lento | Quedó memoria compartida basura de ROS 2. Cierra todo y ejecuta: `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` (`fly.sh` ya lo hace al arrancar). |
| `fly.sh` dice que ya hay un Gazebo corriendo | Ciérralo con `pkill -f "ign gazebo"`, o usa otro dominio: `ROS_DOMAIN_ID=42 ./fly.sh` |
| El avión salta de posición o "explota" | Hay dos simuladores a la vez. Ver la fila anterior. |
| No se abre la ventana de Gazebo | Usa `./fly.sh --headless`, o `ros2 launch vtol_control vtol_sim.launch.py render_engine:=ogre` |
| Las teclas no hacen nada | Escribe en la terminal del piloto de teclado, no en la de `fly.sh` |
| Perdiste el avión de vista en Gazebo | Clic derecho sobre él en el *Entity Tree* → *Follow* |

Para cerrar procesos usa siempre `kill` normal, **nunca `kill -9`**: deja
memoria compartida huérfana y provoca el primer problema de la tabla.

## Glosario

| Término | Significado |
|---|---|
| VTOL | Aeronave de despegue y aterrizaje vertical |
| Tiltrotor | Los motores giran: apuntan hacia arriba para flotar y hacia delante para volar como avión |
| Transición | El paso de modo dron a modo avión (o al revés). Dura unos 3 s |
| Armar | Encender los motores |
| Alabeo / cabeceo / guiñada | Girar sobre el eje largo / subir o bajar el morro / girar a izquierda o derecha |
| Pérdida | Ir tan lento que el ala deja de sostener al avión (por debajo de 12,2 m/s) |
| Servo | Motor pequeño que mueve una pieza a un ángulo exacto (alerones, inclinación de motores) |
| PID | Algoritmo de control que corrige la diferencia entre lo que se pide y lo que se mide |

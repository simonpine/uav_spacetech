# vtol_tiltrotor_description

El **modelo del avión**: su forma, piezas, articulaciones y la configuración
que Gazebo necesita para simularlo. No contiene lógica de vuelo; eso está en
`vtol_control`.

Para volar el avión, ver el [README principal](../../README.md).

## Archivos

| Archivo | Qué es |
|---|---|
| `urdf/vtol_tiltrotor.urdf.xacro` | Forma, masa y articulaciones del avión |
| `urdf/gazebo_fortress.xacro` | Motores, aerodinámica y sensores para Gazebo Fortress (ROS 2 Humble) |
| `urdf/gazebo_harmonic.xacro` | Lo mismo para Gazebo Harmonic (ROS 2 Jazzy) |
| `config/gz_bridge.yaml` | Qué topics se pasan entre Gazebo y ROS 2 |
| `worlds/vtol_field.sdf` | El escenario |
| `launch/display.launch.py` | Muestra el modelo en RViz, sin simular |

## Piezas móviles

| Articulación | Qué mueve | Rango |
|---|---|---|
| `tilt_left_joint`, `tilt_right_joint` | Inclinación de los motores delanteros (0 = arriba, 1,57 = adelante) | −0,17 a 1,57 rad |
| `rear_tilt_joint` | Inclinación lateral del motor trasero (sirve para girar en modo dron) | ±0,52 rad |
| `aileron_left_joint`, `aileron_right_joint` | Alerones | ±0,44 rad |
| `ruddervator_left_joint`, `ruddervator_right_joint` | Cola en V | ±0,52 rad |
| `rotor_left_joint`, `rotor_right_joint`, `rotor_rear_joint` | Hélices | giro continuo |

## Datos del modelo

```
masa          5,5 kg
envergadura   1,70 m
longitud      1,31 m
empuje máx.   36 N por motor (el doble del peso total con los 3)
```

## Ver el modelo sin simular

```bash
ros2 launch vtol_tiltrotor_description display.launch.py
```

Mueve los deslizadores de `tilt_left_joint` y `tilt_right_joint` para ver cómo
giran los motores.

## Limitaciones

- **La aerodinámica es aproximada.** Los coeficientes son genéricos, no datos
  reales del SNOOP 18.
- **La forma es simplificada** (cajas y cilindros). Se pueden cambiar por mallas
  3D, pero sin mover los marcos de referencia de las piezas: la aerodinámica
  depende de ellos.
- Los errores del modelo original que hubo que corregir para que volara están
  en las [notas técnicas](../../docs/notas-tecnicas.md#tres-errores-del-modelo-original-que-hubo-que-corregir).

# Notas técnicas

Detalles para quien vaya a modificar el controlador o el modelo. Para usar el
simulador basta con el [README principal](../README.md).

## La aeronave

Basada en el **Cannon SNOOP 18**:

| | |
|---|---|
| Masa | 5,5 kg |
| Envergadura | 1,70 m |
| Motores | 3: dos delanteros que giran de vertical a horizontal, uno trasero que se inclina a los lados para girar |
| Velocidad de pérdida | 12,2 m/s |
| Velocidad de crucero | ~20 m/s |

## El controlador en cuatro ideas

**1. Un solo controlador para todo el vuelo.** No hay un controlador "de dron"
y otro "de avión" que se intercambian. Cada corrección se reparte entre los
motores y las superficies del ala según cuánto puede hacer cada uno en ese
momento:

```
mandos → ángulo deseado → PID → fuerza y giro necesarios
                                        │
                     ┌──────────────────┴──────────────────┐
                 motores                              superficies del ala
        más peso con los motores verticales     más peso cuanto más rápido vuela
```

Intercambiar dos controladores falla en este avión: en el punto de cambio
ninguno de los dos tiene fuerza suficiente.

**2. Los motores giran según la velocidad, no según el tiempo.** Solo llegan a
horizontal cuando el avión va lo bastante rápido para que el ala lo sostenga.
Si no acelera, dejan de girar. La transición dura ~3 s y la altura varía menos
de 1 m.

**3. En modo avión, la altura se controla con el morro y la velocidad con los
motores.** `R`/`F` fijan una velocidad objetivo (14–30 m/s), no la potencia.
Motivo: en este modelo la resistencia del aire *baja* al acelerar, así que con
potencia fija el avión nunca se estabiliza en una velocidad.

**4. Protecciones automáticas.**
- No pasa a modo avión por debajo de 25 m.
- Vuelve solo a modo dron si va demasiado lento o baja de 15 m.
- Si el piloto de teclado deja de enviar mensajes 1,5 s, suelta los mandos.

Los valores de todas estas reglas están en
`src/vtol_control/config/flight_controller.yaml`.

## Tres errores del modelo original que hubo que corregir

El paquete `vtol_tiltrotor_description` no volaba tal como venía. Ninguno de
estos errores muestra un mensaje claro: el avión simplemente no funciona.

**1. El ala no generaba sustentación.** Al convertir el modelo de URDF a SDF,
Gazebo fusiona las piezas fijas (alas, cola) con el cuerpo, así que los plugins
de aerodinámica no encontraban sus piezas. Corrección: los plugins se aplican
al cuerpo (`base_link`) con las posiciones de cada panel indicadas a mano.

**2. Los servos no llegaban a la posición pedida.** Se pedía 0,30 rad y llegaban
a 0,02 rad. Corrección: ahora se controlan por velocidad
(`<use_velocity_commands>`) y llegan exactamente.

**3. Los servos se bloqueaban en sus topes.** Si una articulación llega a su
límite, el motor de física la deja trabada. Corrección: el controlador nunca
pide el límite exacto, se queda unos puntos por dentro. Este error era muy
confuso: el avión caía en picada mientras los comandos registrados eran
correctos.

Las tres correcciones están en `gazebo_fortress.xacro` y `gazebo_harmonic.xacro`.

## Constantes medidas

`AIL_MX_PER_Q`, `ELE_MY_PER_Q` y `RUD_MZ_PER_Q` en `airframe.py` dicen cuánto
gira el avión por cada grado de alerón o cola. **Si su signo está mal, el
controlador empuja en sentido contrario y el avión se cae.**

Se midieron en simulación con `tools/measure_surfaces.py`: se fija una
superficie y se mide cuánto tienen que compensar los motores. Los valores
medidos son entre el 40 % y el 85 % de los calculados en teoría, y se usan los
medidos. El docstring de ese archivo explica por qué fallaron otros métodos de
medición.

Hay que volver a medir solo si se cambia la aerodinámica o la geometría:

```bash
ros2 launch vtol_control vtol_sim.launch.py gui:=false publish_surfaces:=false
python3 tools/measure_surfaces.py

python3 tools/mass_props.py    # recalcula masa e inercias
```

## Resultados de la prueba automática

Las 15 etapas de `demo_flight` pasaron en la última ejecución. Cada etapa
comprueba, además de su objetivo, que no ocurrió nada anormal en el camino (por
ejemplo, que no se estrelló), para que un choque no cuente como éxito.

| Etapa | Modo | Altura | Velocidad |
|---|---|---|---|
| Estacionario | Dron | 40,0 → 40,0 m | 1,6 m/s |
| Transición (3,2 s) | → Avión | 39,9 → 40,7 m | 7,6 → 21,8 m/s |
| Crucero (10 s) | Avión | 37,7 → 40,1 m | 20,6 → 24,1 m/s |
| Giros ±34° | Avión | 36 → 38 m | 21 m/s |
| Ascenso | Avión | 36 → 55,7 m | 21 m/s |
| Transición de vuelta (3,5 s) | → Dron | 55,7 → 56,8 m | 22,6 → 20,0 m/s |

## Entorno probado

ROS 2 Humble + Gazebo Fortress 6.17.1 en una Jetson (aarch64), a velocidad de
tiempo real con la ventana abierta.

Para ROS 2 Jazzy + Gazebo Harmonic existe `gazebo_harmonic.xacro`, que es el
mismo archivo con otros nombres de plugins. Se elige con el argumento de xacro
`gz_flavor:=harmonic` (el launch actual lo fija en `fortress`).

# VTOL tiltrotor — simulación pilotable con el teclado

Simulación de vuelo del VTOL tiltrotor de tres motores tipo SNOOP, controlada
desde el teclado. **La barra espaciadora alterna entre modo dron y modo avión.**

```bash
./fly.sh
```

La primera vez compila el workspace; después arranca Gazebo con una cámara de
seguimiento y abre el piloto de teclado en su propia ventana.

```
  ESPACIO  alterna DRONE <-> AIRPLANE     (para pasar a avión necesita 25 m de altura)
  ENTER    armar / desarmar               X  apaga los motores de inmediato

  W / S    cabeceo  morro abajo / morro arriba
             DRONE: W vuela hacia delante, S hacia atrás
             AIRPLANE: funciona como una palanca - W pica, S sube
  A / D    alabeo   izquierda / derecha
  Q / E    guiñada  izquierda / derecha   (DRONE: inclina el rotor de cola. AIRPLANE: timón)
  R / F    potencia DRONE: subir / bajar.  AIRPLANE: ajusta la velocidad aerodinámica objetivo
             las flechas Arriba/Abajo hacen lo mismo que R/F

  T  despegue automático a 40 m    L  aterrizaje automático
  H  nivelar y mantener            Z  centrar todos los mandos   ?  volver a mostrar la ayuda
```

Un vuelo típico: **ENTER**, **T**, esperar a que termine el ascenso, **ESPACIO**,
volar un rato, **ESPACIO**, **L**.

La ayuda que imprime el propio programa (tecla **?**) sigue en inglés.

Otras formas de ejecutarlo:

```bash
./fly.sh --headless     # sin ventana de Gazebo; sigues teniendo el HUD y puedes volar
./fly.sh --demo         # misión con guion, sin teclado
./fly.sh --rviz         # RViz junto a Gazebo
```

## Qué estás pilotando

El Cannon SNOOP 18: 5,496 kg, 1,70 m de envergadura, dos rotores delanteros que
se inclinan de vuelo estacionario a crucero, y un rotor trasero sobre un pilón
que se inclina lateralmente para la guiñada, al estilo de un tricóptero. La
entrada en pérdida es a 12,2 m/s y la velocidad de crucero ronda los 20 m/s.

| | |
|---|---|
| `src/vtol_tiltrotor_description` | la aeronave: URDF/xacro, capa de plugins de Gazebo, mundo, configuración del bridge |
| `src/vtol_control` | el controlador de vuelo, el piloto de teclado, la misión con guion |
| `tools/` | los scripts de medición de los que salen las constantes del controlador |

El paquete de descripción se escribió solo como *descripción*, y su propio README
lo dice: "No hay controlador en *este* paquete. Ni estabilización, ni mezclador,
ni secuencia de transición." `vtol_control` es esa capa que faltaba.

## Cómo funciona el controlador

Un único conjunto de lazos cubre el vuelo estacionario, la transición y el
crucero. La alternativa obvia, dos controladores que se cambian a 45° de
inclinación, falla en esta aeronave: en el punto de cambio ninguno de los dos
grupos de actuadores tiene autoridad completa. El brazo de cabeceo de los rotores
ya se ha reducido y el ala todavía no tiene presión dinámica suficiente.

```
   mandos ─► actitud ─► PID de velocidad angular ─► Fz, Mx, My, Mz deseados
                                            │
                        ┌───────────────────┴───────────────────┐
                    rotores                              superficies
       par delantero + rotor trasero + incl. trasera   alerones + ruddervators
             peso ~ cos(inclinación)                    peso ~ (V/16)²
```

La demanda de cada eje se reparte entre los dos grupos de actuadores en
proporción a la autoridad que tiene cada uno *en ese momento*, y las dos partes
siempre suman la demanda completa. No hay conmutación ni discontinuidad.

Conviene conocer tres decisiones:

**El ángulo de inclinación se programa según la velocidad aerodinámica, no con un
temporizador.** Las góndolas solo llegan a 90° cuando la aeronave va realmente lo
bastante rápido para que el ala la sostenga, y si la aceleración se estanca la
inclinación deja de avanzar en lugar de dejar caer la aeronave. Así la transición
hacia delante se regula sola: se completa en unos 3 segundos y mantiene la
altitud dentro de un metro mientras ocurre.

**El rotor trasero se calcula para el cabeceo, después de decidir el empuje del
par delantero.** Resolver cabeceo y empuje vertical juntos como un sistema 2×2 es
exacto en estacionario, pero se vuelve singular a 84° de inclinación. Resolver el
cabeceo al final hace que ese eje sea exacto en cualquier ángulo de inclinación y
deja que el eje vertical, que tiene un integrador de velocidad de ascenso detrás,
absorba el margen.

**En crucero la altitud se mantiene con el cabeceo y la potencia mantiene la
velocidad, no al revés.** El modelo de resistencia de esta aeronave es
`cd = cda·alpha` sin término parásito, así que la resistencia *disminuye* al
acelerar y con un acelerador fijo no existe una velocidad de equilibrio estable.
Por eso R/F ajustan una velocidad aerodinámica objetivo entre 14 y 30 m/s, y la
aeronave encuentra sola la potencia necesaria.

El controlador también se protege a sí mismo: se niega a hacer la transición por
debajo de 25 m sobre el suelo, y vuelve solo al modo dron si la velocidad
aerodinámica cae hacia la pérdida o si se acerca a menos de 15 m del suelo en
modo avión.

## Tres defectos del modelo

El paquete de descripción, tal como venía, no simulaba. Ninguno de estos defectos
da error: la aeronave simplemente no funciona. Por eso están documentados tanto
en el punto donde se corrigen como aquí.

**1. Ninguna aerodinámica.** Al convertir URDF a SDF, sdformat fusiona cada hijo
de una articulación fija con su padre, así que `wing_left`, `tip_left`,
`vtail_left` y sus simétricos no sobreviven como links SDF. Los seis plugins
`LiftDrag` fallaban con *"Link with name[wing_left] not found. The LiftDrag will
not generate forces"*: una aeronave sin sustentación ni resistencia.

Mantener los links con `<preserveFixedJoint>` corrige ese error pero provoca otro
peor (ver más abajo), así que ahora los seis paneles se acoplan a `base_link`
con su `<cp>` y su `<upward>` dados explícitamente en el marco de la base.
Físicamente es idéntico, porque los paneles ya estaban unidos rígidamente, y
conserva el amortiguamiento aerodinámico, porque `LiftDrag` evalúa el flujo en
`<cp>` usando la velocidad angular del link además de la lineal.

**2. Los servos no alcanzaban sus consignas.** Las ganancias del
`JointPositionController` en la capa de plugins original son demasiado bajas para
un PID basado en fuerza: una orden de 0,30 rad al alerón se estabilizaba en
**0,02 rad**, así que en crucero la aeronave prácticamente no tenía control de
cabeceo ni de alabeo. Ahora usan `<use_velocity_commands>`, con lo que se
comportan como servos reales controlados en posición, con la velocidad limitada
por `cmd_max`, que se toma del `<limit velocity="…">` de cada articulación en el
URDF. Una orden de 0,30 rad da ahora 0,300 rad.

**3. Los servos se quedan bloqueados en sus topes.** Es una consecuencia de la
corrección anterior: llevar una articulación hasta su límite activa la
restricción de límite de dartsim y la articulación deja de responder. Para
`rear_tilt`, el único actuador de guiñada en estacionario, eso es fatal. El
controlador ahora ordena posiciones unos pocos puntos porcentuales por dentro de
cada límite.

Este merece detenerse en él, porque produjo el síntoma más engañoso de todo el
proyecto: en modo avión la aeronave picaba y se estrellaba contra el suelo
mientras la orden de profundidad registrada era perfectamente razonable y tenía
el signo correcto. La orden estaba bien. La articulación estaba bloqueada.

## Los coeficientes de las superficies de control se midieron, no se dedujeron

`AIL_MX_PER_Q`, `ELE_MY_PER_Q` y `RUD_MZ_PER_Q`, en `airframe.py`, convierten un
momento deseado en una deflexión, así que su **signo** decide si los lazos de
cabeceo y alabeo se cierran con realimentación negativa o positiva. Salen de
`tools/measure_surfaces.py`, que vuela la aeronave solo con los rotores, fija una
superficie a ±0,30 rad y lee cuánto momento de rotor necesita el controlador para
compensarla. En régimen permanente el momento total es cero, así que los rotores
compensan exactamente la superficie, y el momento de los rotores se obtiene en
forma cerrada a partir de las velocidades ordenadas y la geometría.

Antes se probaron tres métodos de respuesta al escalón en lazo abierto, y los tres
fallaron. Los motivos están en el docstring de ese archivo, porque es fácil caer
en ellos: dos ensayos con +D y −D fijos no son comparables una vez que una
aeronave sin estabilizar ha divergido; una ventana de 0,35 s subestima la
potencia de control entre cuatro y cinco veces, porque los 140 N·m/rad de rigidez
en cabeceo de la cola en V anulan el efecto de la profundidad dentro del periodo
corto de 0,4 s; y una ventana de 70 ms, lo bastante corta para quedar dentro del
periodo corto, tiene una relación señal/ruido de aproximadamente uno.

Los signos medidos coinciden con la deducción analítica. Las magnitudes son entre
el 40 y el 85 % de las analíticas, y las que se usan son las medidas.

```bash
# volver a medir, p. ej. después de cambiar los bloques aerodinámicos
ros2 launch vtol_control vtol_sim.launch.py gui:=false publish_surfaces:=false
python3 tools/measure_surfaces.py

# volver a calcular las propiedades másicas después de cambiar la geometría
python3 tools/mass_props.py
```

## Prueba de regresión

`demo_flight` recorre toda la envolvente de vuelo usando exactamente los mismos
mensajes que envía el teclado, así que ejercita el camino de control real y no
una puerta trasera:

```bash
ros2 launch vtol_control vtol_sim.launch.py gui:=false &
ros2 run vtol_control demo_flight --csv /tmp/flight.csv
```

Quince etapas: armado, despegue automático a 40 m, estacionario, mantenimiento de
altitud, guiñada, traslación, transición, crucero, viraje a la derecha, viraje a
la izquierda, ascenso en crucero, transición de vuelta, estabilización y
aterrizaje automático. Cada etapa comprueba un *invariante* además de una
condición de finalización. Sin eso, una etapa cuya condición es "mode == DRONE"
pasaría trivialmente después de que la aeronave se estrellara y la lógica de
seguridad la devolviera al modo dron. Un accidente es un fallo, no un éxito.

En la última ejecución pasaron las quince:

| etapa | modo | altitud | velocidad aerodinámica |
|---|---|---|---|
| estacionario, mantenimiento de altitud | DRONE | 40,0 → 40,0 m | 1,6 m/s |
| transición, 3,2 s | → AIRPLANE | 39,9 → 40,7 m | 7,6 → 21,8 m/s |
| crucero, 10 s | AIRPLANE | 37,7 → 40,1 m | 20,6 → 24,1 m/s |
| viraje ±34° | AIRPLANE | 36 → 38 m | 21 m/s |
| ascenso en crucero | AIRPLANE | 36 → 55,7 m | 21 m/s |
| transición de vuelta, 3,5 s | → DRONE | 55,7 → 56,8 m | 22,6 → 20,0 m/s |

## Si deja de funcionar

**Todo se queda en silencio: no hay topics, pero los procesos parecen sanos.**
FastDDS deja sus segmentos de memoria compartida en `/dev/shm` cuando un nodo
recibe `SIGKILL`, y si se acumulan suficientes huérfanos se rompe el
descubrimiento. El síntoma engaña: la aeronave se arma y luego despega a una
décima parte de la velocidad de ascenso ordenada, porque el controlador solo
recibe parte de las muestras de la IMU. `fly.sh` los borra al arrancar cuando
nada más está usando ROS; a mano sería:

```bash
pkill -f "ign gazebo"           # SIGTERM, no -9
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

**Dos simuladores a la vez.** Los dos publican `/vtol/odom` y los dos escuchan
`/vtol/command/motor_speed`, así que el controlador ve una posición que salta
entre dos aeronaves: velocidades de miles de m/s y altitudes que caen a cero.
`fly.sh` se niega a arrancar un segundo simulador. `ROS_DOMAIN_ID=42 ./fly.sh`
le da a una ejecución su propio dominio.

**No se abre la ventana de Gazebo.** Prueba `./fly.sh --headless`, o
`ros2 launch vtol_control vtol_sim.launch.py render_engine:=ogre` si el driver de
la GPU no soporta ogre2. En modo headless se puede volar perfectamente: el HUD
está en la terminal del piloto de teclado, no en Gazebo.

**Perdiste la aeronave en la vista de Gazebo.** Haz clic derecho sobre ella en el
Entity Tree y elige Follow. El launch lo hace automáticamente unos segundos
después de que aparece.

## Entorno

Compilado y probado con ROS 2 Humble y Gazebo Fortress (ign-gazebo 6.17.1) en
una Jetson aarch64, con un factor de tiempo real ≈ 1,0 con la interfaz gráfica
abierta.

`gazebo_harmonic.xacro` es la misma capa de plugins para Jazzy / Gazebo Harmonic:
idéntica salvo por los nombres `gz-sim-*` / `gz::sim::systems::`. Se selecciona
con `gz_flavor:=harmonic`. Las dos se generan desde la misma fuente, y las
correcciones descritas arriba están en ambas.

Solo cinemática, sin simulador:

```bash
ros2 launch vtol_tiltrotor_description display.launch.py
```
# uav_spacetech

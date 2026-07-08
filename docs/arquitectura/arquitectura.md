# Arquitectura

Flujo de alto nivel:

```text
sensores / simulacion / rosbag
  -> maze_slam
  -> mapa de ocupacion
  -> maze_nav: mapa + MCL + A* + controlador
  -> maze_perception: deteccion del cono rojo
  -> maze_mission: FSM de mision y goals
  -> cmd_vel / evidencia / replay
```

## Parte A

`maze_slam` implementa Grid-Based FastSLAM. Consume LIDAR y odometria, mantiene
particulas con mapas de ocupacion propios y publica el mapa resultante.

Salidas principales:

- `/map`
- `/belief`
- `/maze_slam/particles`
- TF `map -> odom` o frame equivalente del robot.

## Parte B

`maze_nav` consume el mapa de Parte A y separa tres responsabilidades:

- `map_publisher`: publica el mapa estatico.
- `localizer`: MCL sobre mapa de ocupacion.
- `navigator`: A*, costmap inflado, seguimiento de path, orientacion final y
  recovery ante obstaculos no mapeados.

Salidas principales:

- `/amcl_pose`
- `/particlecloud`
- `/plan`
- `/nav_state`
- `/cmd_vel`

## Parte C

`maze_perception` detecta el cono rojo y publica detecciones. `maze_mission`
observa pose, mapa, scan, detecciones y estado de navegacion; decide cuando
patrullar, cuando emitir un goal al cono y cuando confirmar la mision.

El fallback `SERVO_TO_CONE` cubre el caso real donde la camara ve el cono pero el
LIDAR mide la pared de fondo porque el cono queda por debajo del plano del haz.
En ese caso se emiten micro-goals validados contra el mapa inflado, en vez de
descartar la deteccion.

## Evidencia

La evidencia no cambia el comportamiento del sistema. Los scripts de `scripts/`
y `shs/` ayudan a grabar rosbags, logs, CSV y replays RViz.

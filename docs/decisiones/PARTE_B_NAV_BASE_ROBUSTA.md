# Parte B - Decision de implementacion: navegacion base robusta

> Estado: aceptada  
> Fecha: 2026-06-27  
> Alcance: primera implementacion ejecutable de Parte B.

## Contexto

Parte B necesita una base funcional para mover el robot sin teleop y sin
arriesgar choques. La prioridad inicial es seguridad: velocidades bajas, freno
por LIDAR, watchdog y comportamiento conservador ante dudas. El mapa de Parte A
se usa para visualizar y para la navegacion a objetivo, pero la primera capa
usable debe poder recorrer en modo reactivo aunque todavia no haya localizacion
robusta.

## Opciones consideradas

- **Nav2 completo**: robusto y estandar, pero agrega mucha configuracion y tuning
  antes de tener una prueba chica defendible.
- **Nodo unico centrado en goal navigation**: rapido para prototipar, pero si el
  follower oscila o la pose inicial esta mal, el robot queda girando sin una
  capa de seguridad simple.
- **Paquete propio modular chico con `safe_drive` primero**: permite validar
  recorrido sin teleop antes de activar A* y follower; deja MCL o Nav2 como
  reemplazo futuro.

## Decision

Implementar `maze_nav` como paquete separado con dos modos:

- `safe_drive`: control reactivo por LIDAR, sin dependencia del mapa, velocidades
  muy bajas y freno por watchdog.
- `goal`: mapa YAML/PGM o `/map`, publicacion de `/map`, `/global_costmap`,
  `/planned_path`, `/nav_state` y `/nav_debug`; A* con waypoints densos sin
  simplificacion agresiva; follower conservador con lookahead corto, progreso
  monotono de waypoints, estados explicitos y overlay dinamico de obstaculos
  detectados por LIDAR.

## Consecuencias

- La pose es parametrizable (`/odom`, `/calc_odom` o `/belief`) para `goal`.
- La primera version no implementa MCL complejo ni Nav2.
- El mapa desconocido se trata como obstaculo por defecto.
- El sistema prioriza frenar antes que intentar maniobras agresivas.
- `safe_drive` y `goal` mantienen velocidades bajas por defecto:
  `linear <= 0.06 m/s`, `angular <= 0.30 rad/s`.
- El goal se considera alcanzado por posicion por defecto (`9 cm`) y no exige
  alinear la orientacion final del click de RViz, para evitar giros finales
  confusos. Se puede activar con `align_final_yaw:=true`.
- El overlay dinamico de LIDAR esta activo por defecto en `goal`; marca clusters
  de scan como obstaculos temporales en el costmap, ignora puntos aislados y
  fuerza replanning si el path actual pisa una celda marcada por scan.
- Hay dos perfiles RViz: uno limpio para demo y otro de debug con costmap y
  diagnostico.
- Los tests de rollout cubren avance recto, goal detras, goal perpendicular,
  diagonal, goal corto, set variado de goals, pared con abertura, pasillo en L,
  pasillo en S y bloqueo frontal para detectar regresiones de giro infinito,
  lentitud excesiva o recorte de esquinas.

## Como correr

Desde el workspace:

```bash
cd ~/Robotica/tp_final_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Terminal 1, simulacion headless:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py gui:=false
```

Terminal 2, navegacion con mapa YAML de Parte A y pose de simulacion:

```bash
ros2 launch maze_nav nav_base.launch.py \
  mode:=goal \
  map_source:=yaml \
  map_yaml:=results/parte_a/casa_map_tuned.yaml \
  pose_topic:=/odom \
  pose_topic_type:=odometry \
  use_sim_time:=true
```

Terminal 3, RViz limpio:

```bash
LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 \
rviz2 -d src/maze_nav/rviz/nav_clean.rviz
```

En RViz usar `2D Goal Pose` sobre `/goal_pose`. El nodo publica:

- `/cmd_vel`: comandos conservadores al robot.
- `/planned_path`: path A* vigente.
- `/nav_state`: estado textual (`IDLE`, `ROTATE_TO_PATH`, `TRACK_PATH`,
  `BLOCKED_STOP`, `STUCK_RECOVERY`, `GOAL_REACHED`, `WATCHDOG_STOP`).
- `/nav_debug`: JSON con pose, goal, target, clearance frontal, overlay de scan
  y motivo de freno.

Atajos:

```bash
./scripts/nav.sh safe   # modo reactivo por scan
./scripts/nav.sh yaml   # goal nav con mapa guardado + /odom
./scripts/nav.sh live   # goal nav con /map vivo + /belief
```

## Validacion automatica

Unitarios y rollouts sin Gazebo:

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select maze_nav --event-handlers console_direct+
colcon test-result --verbose
```

Smoke end-to-end con Gazebo headless:

```bash
./scripts/smoke_goal_nav.sh
```

Ese smoke levanta `custom_casa`, lanza `maze_nav`, manda goals por
`/goal_pose` y valida por `/odom`, `/nav_state`, `/cmd_vel`, `/planned_path` y
`/nav_debug`. Por defecto espera dos goals alcanzados y un goal bloqueado de
forma segura.

Tambien se pueden pasar goals custom:

```bash
SMOKE_SKIP_BUILD=1 ./scripts/smoke_goal_nav.sh \
  0.8,0.0,0 \
  0.9,0.45,0 \
  blocked:1.15,1.25,0
```

## Como seguir

- Si el robot frena en un punto donde el mapa dice que esta libre, mirar
  `/nav_debug`: `front_clearance_m`, `front_emergency_m` y
  `scan_overlay_cells` indican si el LIDAR esta viendo algo que el mapa no tiene.
- Si el mapa de Parte A o `/belief` mejora, probar `./scripts/nav.sh live` para
  usar `/map` y `/belief` en vez del YAML fijo y `/odom`.
- Si un caso manual falla, convertirlo primero en smoke o test unitario:
  `scripts/smoke_goal_nav.sh` acepta secuencias de goals y goals esperados como
  `blocked:x,y,yaw`.
- No relajar velocidades ni distancias de freno para "hacer que llegue": si el
  LIDAR contradice el mapa, la politica correcta de esta base es frenar,
  replanificar o declarar `BLOCKED_STOP`.

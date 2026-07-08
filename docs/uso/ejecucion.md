# Ejecucion

## Parte A - SLAM

Simulacion TurtleBot3:

```bash
./shs/casa.sh
./shs/slam_casa.sh
./shs/teleop.sh
./shs/save_map.sh
```

TurtleBot4 real:

```bash
export ROS_DOMAIN_ID=<dominio_del_TB4>
./shs/mapear_tb4.sh --ns tb4_0
./shs/save_map.sh laberinto_lab_$(date +%Y%m%d)
```

## Parte B - navegacion

Simulacion TurtleBot3:

```bash
./shs/casa.sh obs
./shs/nav.sh
```

En RViz usar `2D Pose Estimate` y luego `2D Goal Pose`.

TurtleBot4 real:

```bash
./shs/navegar_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml
```

## Parte C - mision del cono

TurtleBot4 real:

```bash
./shs/parte_c_tb4.sh --ns tb4_0 --map maps/laberinto_lab_20260702.yaml
```

El wrapper levanta navegacion, `mission_node`, `cone_detector` y RViz. La FSM de
mision patrulla waypoints, gira para buscar el cono, valida el objetivo contra el
mapa y publica `/goal_pose` para que `maze_nav` navegue.

## Replay con bag

Varios wrappers aceptan `--bag` para validar contra rosbag sin robot:

```bash
./shs/bag.sh
./shs/mapear_tb4.sh --bag
./shs/navegar_tb4.sh --bag
./shs/parte_c_tb4.sh --bag
```

Los rosbags son pesados y estan ignorados por git.

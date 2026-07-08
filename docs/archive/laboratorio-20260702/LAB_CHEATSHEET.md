# Cheatsheet — turno del lab

## Setup (una vez al empezar)

```bash
docker exec -it udesa-robotica bash
cd ~/ros2_ws/src/robotica/robot_maze_runner
export ROS_DOMAIN_ID=<X>
ros2 topic list | grep /tb4_1              # confirmar que ves scan, odom, tf
ros2 run tf2_tools view_frames             # confirmar frames pelados o con prefijo
```

## Parte A — mapeo

```bash
# T1
./shs/mapear_tb4.sh --ns tb4_1

# T2 (teleop para manejar el TB4)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb4_1/cmd_vel

# T3 (cuando el mapa esta lindo)
./shs/save_map.sh laberinto_lab_$(date +%Y%m%d)
```

## Parte B — navegación

```bash
# T1
./shs/navegar_tb4.sh --ns tb4_1

# En RViz: 2D Pose Estimate -> 2D Goal Pose
```

## Parte C — búsqueda de conos

```bash
# T1
./shs/parte_c_tb4.sh --ns tb4_1

# En RViz: 2D Pose Estimate. Mission maneja solo.
```

## E-stop (dejar corriendo aparte)

```bash
./shs/estop.sh tb4_1
```

## Fixes rápidos en vivo

```bash
# HSV del cono si el rojo se ve raro
ros2 param set /cone_detector hue_min 0
ros2 param set /cone_detector hue_max 10

# bajar velocidad
ros2 param set /navigator v_max 0.08

# si los frames del TF vienen prefijados (tb4_1/odom en vez de odom)
# relanzar cambiando 'odom_frame' del localizer via param
```

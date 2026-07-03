#!/usr/bin/env bash
# Parte A live sobre el TB4 REAL: FastSLAM + RViz. Genera el mapa del laberinto
# en vivo mientras se manejale el robot por teleop (u otra fuente de cmd_vel).
#
# Uso:
#   ./shs/mapear_tb4.sh                     # default ns=tb4_0
#   ./shs/mapear_tb4.sh --ns tb4_1          # el TB4 del lab
#   ./shs/mapear_tb4.sh --ns tb4_1 --no-rviz
#   ./shs/mapear_tb4.sh --bag               # probar contra rosbag (sim_time=true)
#
# Probar sin robot (validacion previa al turno):
#   T1: ./shs/bag.sh                # reproduce el rosbag de mapeo del laberinto
#   T2: ./shs/mapear_tb4.sh --bag   # SLAM contra ese bag (ns por defecto=tb4_0)
#
# Antes de correr en el ROBOT REAL:
#   1) TB4 encendido y booteado.
#   2) export ROS_DOMAIN_ID=<X>   (el mismo que usa el TB4).
#   3) ros2 topic list | grep tb4_1   -> tenes que ver scan/odom/tf.
#
# Guardar el mapa cuando quede bueno (en otra terminal, mismo container):
#   ./shs/save_map.sh laberinto_lab_<yyyymmdd>
#
# Manejar el robot mientras mapea: teleop de la catedra o
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
#       -r cmd_vel:=/<ns>/cmd_vel
set -e
source "$(dirname "$0")/_common.sh"
source "$(dirname "$0")/_tb4_common.sh"
cd "$WS_DIR"

# Parsear args primero: en modo --bag NO matamos el 'ros2 bag play' del usuario.
NS="tb4_0"
WITH_RVIZ=1
USE_SIM=false     # --bag lo pone en true (rosbag publica /clock)
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ns)       NS="$2"; shift 2 ;;
        --no-rviz)  WITH_RVIZ=0; shift ;;
        --bag)      USE_SIM=true; shift ;;
        *)          echo "arg desconocido: $1" >&2; shift ;;
    esac
done

# Limpieza selectiva. Mata nodos y RViz, pero preserva el rosbag en --bag
# (kill_all mata "ros2 bag play" -> mataba lo que abriste vos en la otra term).
if [[ "$USE_SIM" == "true" ]]; then
    for pat in rviz2 fastslam_node map_publisher localizer navigator \
               "ros2 launch" "ros2 run"; do
        pkill -9 -f "$pat" 2>/dev/null || true
    done
    sleep 0.5
    echo "Limpieza (preservando rosbag) ok."
else
    bash "$WS_DIR/shs/kill_all.sh"
    sleep 0.5
fi

"$WS_DIR/shs/build.sh"
source "$INSTALL_BASE/local_setup.bash"

tb4_precheck "$NS" "/$NS/scan" "/$NS/odom"

echo "Mapear TB4: ns=$NS use_sim_time=$USE_SIM"

cleanup() {
    [[ -n "${RVIZ_PID:-}" ]] && kill "$RVIZ_PID" 2>/dev/null || true
    [[ -n "${STACK_PID:-}" ]] && kill "$STACK_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "$WITH_RVIZ" -eq 1 ]]; then
    # La config de SLAM ya viene con overlay de scan/mapa. Escuchamos el /tf
    # namespaced del TB4 (mismo remap que la launch).
    RVIZ_CFG="$WS_DIR/src/maze_slam/rviz/maze_slam.rviz"
    if [[ ! -f "$RVIZ_CFG" ]]; then RVIZ_CFG="$WS_DIR/src/maze_slam/rviz/casa.rviz"; fi
    rviz2 -d "$RVIZ_CFG" \
        --ros-args -p use_sim_time:="$USE_SIM" \
        -r /tf:="/$NS/tf" -r /tf_static:="/$NS/tf_static" &
    RVIZ_PID=$!
fi

# use_sim_time va como override a la launch a traves de --ros-args del proceso.
ros2 launch maze_slam slam_tb4_live.launch.py ns:="$NS" use_sim_time:="$USE_SIM" &
STACK_PID=$!

wait -n

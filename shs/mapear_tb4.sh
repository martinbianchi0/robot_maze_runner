#!/usr/bin/env bash
# Parte A live sobre el TB4 REAL: FastSLAM + RViz. Genera el mapa del laberinto
# en vivo mientras se manejale el robot por teleop (u otra fuente de cmd_vel).
#
# Uso:
#   ./shs/mapear_tb4.sh                     # default ns=tb4_0
#   ./shs/mapear_tb4.sh --ns tb4_1          # el TB4 del lab
#   ./shs/mapear_tb4.sh --ns tb4_1 --no-rviz
#
# Antes de correr:
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

# Limpieza total antes de arrancar.
bash "$WS_DIR/shs/kill_all.sh"
sleep 0.5

"$WS_DIR/shs/build.sh"
source "$INSTALL_BASE/local_setup.bash"

NS="tb4_0"
WITH_RVIZ=1
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ns)       NS="$2"; shift 2 ;;
        --no-rviz)  WITH_RVIZ=0; shift ;;
        *)          echo "arg desconocido: $1" >&2; shift ;;
    esac
done

tb4_precheck "$NS" "/$NS/scan" "/$NS/odom"

echo "Mapear TB4: ns=$NS use_sim_time=false"

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
        --ros-args -p use_sim_time:=false \
        -r /tf:="/$NS/tf" -r /tf_static:="/$NS/tf_static" &
    RVIZ_PID=$!
fi

ros2 launch maze_slam slam_tb4_live.launch.py ns:="$NS" &
STACK_PID=$!

wait -n

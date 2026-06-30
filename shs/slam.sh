#!/usr/bin/env bash
# Buildea, arranca el nodo de Grid-Based FastSLAM y abre RViz, todo junto.
# Lee /tb4_0/scan + /tb4_0/odom y publica /map, /belief, TF map->odom.
#
# Al cortar con Ctrl-C se cierran los dos (nodo + RViz).
# Flag --no-rviz para correr solo el nodo.
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Matar cualquier nodo/rviz de SLAM previo (zombies de corridas anteriores que
# publicarian un /map de otro tamanio y harian parpadear RViz).
pkill -f fastslam_node 2>/dev/null || true
pkill -f "rviz2 .*maze_slam" 2>/dev/null || true
sleep 0.5

# Build incremental (rapido si no cambiaste codigo). Asi no tenes que buildear a mano.
"$WS_DIR/shs/build.sh"
source "$INSTALL_BASE/local_setup.bash"

WITH_RVIZ=1
[[ "${1:-}" == "--no-rviz" ]] && WITH_RVIZ=0

cleanup() {
    [[ -n "${RVIZ_PID:-}" ]] && kill "$RVIZ_PID" 2>/dev/null || true
    [[ -n "${SLAM_PID:-}" ]] && kill "$SLAM_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "$WITH_RVIZ" -eq 1 ]]; then
    RVIZ_CFG="$WS_DIR/src/maze_slam/rviz/maze_slam.rviz"
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
fi

ros2 launch maze_slam slam.launch.py &
SLAM_PID=$!

# Esperar a cualquiera de los dos: si uno muere, el trap mata al otro.
wait -n


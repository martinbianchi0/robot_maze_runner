#!/usr/bin/env bash
# FastSLAM sobre la simulacion de la casa (TB3). Arranca el nodo + RViz.
# Correr ./shs/casa.sh en otra terminal primero (y ./shs/teleop.sh para manejar).
#
# Flag --no-rviz para solo el nodo.
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Matar nodo/rviz de SLAM previo (zombies que publicarian otro /map).
pkill -f fastslam_node 2>/dev/null || true
pkill -f "rviz2 .*casa.rviz" 2>/dev/null || true
sleep 0.5

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
    rviz2 -d "$WS_DIR/src/maze_slam/rviz/casa.rviz" --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
fi

ros2 launch maze_slam slam_casa.launch.py &
SLAM_PID=$!

wait -n

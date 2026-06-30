#!/usr/bin/env bash
# Abre RViz con la config preconfigurada para ver el SLAM (mapa, scan, belief, particulas).
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

RVIZ_CFG="$WS_DIR/src/maze_slam/rviz/maze_slam.rviz"
# Si rviz tiene problemas de GPU (WSL u otros), descomentar:
# export LIBGL_ALWAYS_SOFTWARE=1
exec rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true

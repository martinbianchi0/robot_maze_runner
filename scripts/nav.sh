#!/usr/bin/env bash
# Terminal D: navegacion Parte B.
#
# Uso:
#   ./scripts/nav.sh safe       # safe_drive reactivo, no necesita mapa
#   ./scripts/nav.sh yaml       # goal nav con mapa guardado + /odom
#   ./scripts/nav.sh live       # goal nav con /map vivo de maze_slam + /belief

set -e

MODE="${1:-safe}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
cd "${ROOT_DIR}"

echo ">>> colcon build --packages-select maze_nav --symlink-install"
colcon build --packages-select maze_nav --symlink-install

source install/setup.bash

case "${MODE}" in
  safe)
    echo ">>> maze_nav safe_drive"
    exec ros2 launch maze_nav nav_base.launch.py \
      mode:=safe_drive \
      map_source:=yaml \
      map_yaml:=results/parte_a/casa_map_tuned.yaml \
      pose_topic:=/odom \
      pose_topic_type:=odometry \
      use_sim_time:=true
    ;;
  yaml)
    echo ">>> maze_nav goal con mapa YAML + /odom"
    exec ros2 launch maze_nav nav_base.launch.py \
      mode:=goal \
      map_source:=yaml \
      map_yaml:=results/parte_a/casa_map_tuned.yaml \
      pose_topic:=/odom \
      pose_topic_type:=odometry \
      publish_loaded_map:=true \
      use_sim_time:=true
    ;;
  live)
    echo ">>> maze_nav goal con /map vivo + /belief de maze_slam"
    exec ros2 launch maze_nav nav_base.launch.py \
      mode:=goal \
      map_source:=topic \
      map_topic:=/map \
      publish_loaded_map:=false \
      pose_topic:=/belief \
      pose_topic_type:=pose_stamped \
      replan_on_map_update:=false \
      use_sim_time:=true
    ;;
  *)
    echo "Modo invalido: ${MODE}" >&2
    echo "Usar: safe | yaml | live" >&2
    exit 2
    ;;
esac

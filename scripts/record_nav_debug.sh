#!/usr/bin/env bash
# Graba evidencia de una prueba de navegacion para diagnosticar despues.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${1:-${ROOT_DIR}/rosbags/nav_debug_${STAMP}}"

source /opt/ros/humble/setup.bash
cd "${ROOT_DIR}"
mkdir -p "$(dirname "${OUT}")"
source install/setup.bash

echo ">>> Grabando debug de navegacion en: ${OUT}"
echo ">>> Cortar con Ctrl+C cuando termine la prueba."

exec ros2 bag record -o "${OUT}" \
  /scan \
  /odom \
  /calc_odom \
  /belief \
  /cmd_vel \
  /nav_state \
  /nav_debug \
  /planned_path \
  /map \
  /global_costmap \
  /tf \
  /tf_static

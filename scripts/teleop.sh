#!/usr/bin/env bash
# Terminal C: teleop por teclado.
# Controles dentro de la app: w/x adelante/atras, a/d girar, s parar.
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
export TURTLEBOT3_MODEL=burger

exec ros2 run turtlebot3_teleop teleop_keyboard

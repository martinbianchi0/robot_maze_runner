#!/usr/bin/env bash
# Terminal A: simulacion Gazebo + TurtleBot3 burger.
# Limpia procesos previos antes de arrancar para evitar dos sims publicando
# /calc_odom a la vez (sintoma: belief vuela, particulas se dispersan).
#
# Uso: ./scripts/sim.sh [mundo]
#   mundo default: custom_casa
#   alternativas:  custom_casa_obs / custom_casa_obs2 / custom_room / custom_maze

set -e
MUNDO="${1:-custom_casa}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo ">>> Limpiando procesos previos..."
"${SCRIPT_DIR}/kill_all.sh" > /dev/null 2>&1 || true
sleep 1

source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/install/setup.bash"
export TURTLEBOT3_MODEL=burger

echo ">>> ros2 launch turtlebot3_custom_simulation ${MUNDO}.launch.py"
exec ros2 launch turtlebot3_custom_simulation "${MUNDO}.launch.py"

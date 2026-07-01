#!/usr/bin/env bash
# Smoke test C4: nav-to-goal en lazo cerrado con mini-sim (sin Gazebo).
# Lanza la pila de navegacion de toma-2 (map_publisher + navigator) + el mini-sim
# cinematico (fake_diff_drive), publica un goal y verifica el contrato
# /goal_pose -> /nav_state -> REACHED. Es tambien el interface smoke test: correr
# tras cada rebase sobre toma-2 para detectar si el contrato de maze_nav cambio.
#
# Uso: ./scripts/smoke_goal_nav.sh [map.yaml] [start_x start_y goal_x goal_y] [timeout]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
# shellcheck disable=SC1091
source scripts/parte_c_env.sh

MAP="${1:-maps/maze_slam.yaml}"
START_X="${2:-2.47}"
START_Y="${3:--2.06}"
GOAL_X="${4:-0.71}"
GOAL_Y="${5:--1.12}"
DURATION="${6:-40}"

cleanup() {
  pkill -9 -f 'map_publisher' 2>/dev/null || true
  pkill -9 -f 'maze_nav.*navigator|navigator' 2>/dev/null || true
  pkill -9 -f 'fake_diff_drive' 2>/dev/null || true
  [ -x shs/kill_all.sh ] && bash shs/kill_all.sh 2>/dev/null || true
}
trap cleanup EXIT

echo "[smoke_goal_nav] map=$MAP start=($START_X,$START_Y) goal=($GOAL_X,$GOAL_Y) timeout=${DURATION}s"
[ -f "$ROOT_DIR/$MAP" ] || { echo "no existe el mapa $MAP" >&2; exit 1; }

ros2 run maze_nav map_publisher --ros-args -p map_yaml:="$ROOT_DIR/$MAP" &
ros2 run maze_nav navigator &
python scripts/fake_diff_drive.py --ros-args -p init_x:="$START_X" -p init_y:="$START_Y" &
sleep 6

python scripts/goal_nav_client.py "$GOAL_X" "$GOAL_Y" "$DURATION"

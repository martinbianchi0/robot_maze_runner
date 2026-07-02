#!/usr/bin/env bash
# Smoke test C5: mision completa con cono MOCKEADO (sin percepcion real).
# Levanta map_publisher + navigator + mini-sim (fake_diff_drive) + mission_node +
# mock_cone_publisher + mission_monitor, y valida la FSM de mision end-to-end.
#
# Escenarios:
#   reachable -> el cono cae en espacio libre; la FSM debe llegar a DONE.
#   wall      -> el cono cae sobre una pared del mapa (visto "a traves"); la FSM
#                debe RECHAZARLO y NO emitir ningun goal hacia la pared.
#
# Uso: ./scripts/smoke_mission.sh [reachable|wall] [duracion_s]
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
# shellcheck disable=SC1091
source scripts/parte_c_env.sh

SCEN="${1:-reachable}"
DURATION="${2:-50}"
MODE="${3:-monitor}"   # monitor (assert PASS/FAIL) | view (graba PNG top-down)
MAP="maps/maze_slam.yaml"
START_X=2.45
START_Y=-2.00
if [ "$SCEN" = "wall" ]; then
  TX=3.25; TY=-2.08; EXPECT=reject
else
  TX=4.04; TY=-2.17; EXPECT=done
fi
PARAMS="$ROOT_DIR/config/parte_c/sim.yaml"

cleanup() {
  # 'maze_mission' matchea tanto el launch como el ejecutable del nodo
  # (.../maze_mission/mission); mission_monitor NO contiene 'maze_mission'.
  for p in mission_monitor mock_cone_publisher fake_diff_drive maze_mission navigator map_publisher; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  [ -x shs/kill_all.sh ] && bash shs/kill_all.sh 2>/dev/null || true
}
trap cleanup EXIT
cleanup   # matar cualquier resto de una corrida anterior
sleep 1

echo "[smoke_mission] escenario=$SCEN cono_target=($TX,$TY) expect=$EXPECT"
ros2 run maze_nav map_publisher --ros-args -p map_yaml:="$ROOT_DIR/$MAP" &
ros2 run maze_nav navigator --ros-args -p scan_topic:=/nav_scan_unused &
python scripts/fake_diff_drive.py --ros-args -p init_x:="$START_X" -p init_y:="$START_Y" &
sleep 4
# El mock arranca ANTES que la mision: asi el cono ya se ve desde el start (robot
# quieto -> estimacion limpia) y la mision no maneja waypoints placeholder primero.
python scripts/mock_cone_publisher.py --ros-args -p target_x:="$TX" -p target_y:="$TY" &
sleep 2
ros2 launch maze_mission mission.launch.py params_file:="$PARAMS" &
sleep 1

if [ "$MODE" = "view" ]; then
  python scripts/mission_view.py "$TX" "$TY" "$DURATION" "$SCEN" "$MAP"
else
  python scripts/mission_monitor.py "$EXPECT" "$DURATION" "$MAP"
fi

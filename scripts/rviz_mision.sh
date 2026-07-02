#!/usr/bin/env bash
# Vista EN VIVO en RViz de la mision C5: mini-sim (fake_diff_drive) + navigator +
# mission_node + cono mockeado + markers. Abris RViz y ves al robot navegar
# hacia/alrededor del cono. La percepcion esta MOCKEADA (RViz no renderiza camara);
# lo que se ve es la logica de mision + navegacion interactuando con el cono.
#
# Paredes/robot/cono/goal van como MARKERS (no se usa el display Map de RViz, que
# tiene un bug de GLSL en macOS).
#
# Uso: ./scripts/rviz_mision.sh [reachable|wall]   (cerra RViz o Ctrl-C para salir)
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
# shellcheck disable=SC1091
source scripts/parte_c_env.sh

SCEN="${1:-reachable}"
MAP="maps/maze_slam.yaml"
START_X=2.45
START_Y=-2.00
if [ "$SCEN" = "wall" ]; then TX=3.25; TY=-2.08; else TX=4.04; TY=-2.17; fi
PARAMS="$ROOT_DIR/config/parte_c/sim.yaml"

cleanup() {
  for p in rviz2 rviz_markers mock_cone_publisher fake_diff_drive maze_mission navigator map_publisher; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  [ -x shs/kill_all.sh ] && bash shs/kill_all.sh 2>/dev/null || true
}
trap cleanup EXIT
cleanup
sleep 1

echo "[rviz_mision] escenario=$SCEN cono=($TX,$TY). Abriendo RViz..."
ros2 run maze_nav map_publisher --ros-args -p map_yaml:="$ROOT_DIR/$MAP" &
ros2 run maze_nav navigator --ros-args -p scan_topic:=/nav_scan_unused &
python scripts/fake_diff_drive.py --ros-args -p init_x:="$START_X" -p init_y:="$START_Y" -p publish_tf:=true &
python scripts/rviz_markers.py --ros-args -p cone_x:="$TX" -p cone_y:="$TY" &
sleep 4
python scripts/mock_cone_publisher.py --ros-args -p target_x:="$TX" -p target_y:="$TY" &
sleep 2
ros2 launch maze_mission mission.launch.py params_file:="$PARAMS" &
sleep 1

echo "[rviz_mision] RViz abierto. Cerra la ventana o Ctrl-C para terminar."
rviz2 -d "$ROOT_DIR/rviz/parte_c_mision.rviz"

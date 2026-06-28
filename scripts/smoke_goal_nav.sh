#!/usr/bin/env bash
# Smoke test end-to-end: Gazebo custom_casa + maze_nav + goals reales.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_LOG="/tmp/maze_nav_smoke_sim.log"
NAV_LOG="/tmp/maze_nav_smoke_nav.log"

GOALS=("$@")
if [ "${#GOALS[@]}" -eq 0 ]; then
  GOALS=("0.8,0.0,0" "0.9,0.45,0" "blocked:1.15,1.25,0")
fi

source /opt/ros/humble/setup.bash
cd "${ROOT_DIR}"

cleanup() {
  set +e
  if [ -n "${NAV_PID:-}" ]; then kill "${NAV_PID}" 2>/dev/null || true; fi
  if [ -n "${SIM_PID:-}" ]; then kill "${SIM_PID}" 2>/dev/null || true; fi
  sleep 2
  if [ -n "${NAV_PID:-}" ]; then kill -9 "${NAV_PID}" 2>/dev/null || true; fi
  if [ -n "${SIM_PID:-}" ]; then kill -9 "${SIM_PID}" 2>/dev/null || true; fi
  "${SCRIPT_DIR}/kill_all.sh" >/tmp/maze_nav_smoke_cleanup.log 2>&1 || true
}
trap cleanup EXIT

echo ">>> Limpiando procesos previos"
"${SCRIPT_DIR}/kill_all.sh" >/tmp/maze_nav_smoke_kill.log 2>&1 || true

if [ "${SMOKE_SKIP_BUILD:-0}" != "1" ]; then
  echo ">>> Build de paquetes necesarios"
  colcon build --symlink-install --packages-select turtlebot3_custom_simulation maze_nav
fi
source install/setup.bash
export TURTLEBOT3_MODEL=burger

rm -f "${SIM_LOG}" "${NAV_LOG}"

echo ">>> Lanzando custom_casa headless"
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py gui:=false \
  >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

echo ">>> Esperando /scan, /odom y /robot_description"
timeout 150 bash -lc '
  source /opt/ros/humble/setup.bash
  source ~/Robotica/tp_final_ws/install/setup.bash
  until ros2 topic list | grep -qx /scan; do sleep 1; done
  until ros2 topic list | grep -qx /odom; do sleep 1; done
  until ros2 topic list | grep -qx /robot_description; do sleep 1; done
' || {
  echo "FAIL: la simulacion no publico topics de robot"
  tail -n 80 "${SIM_LOG}" || true
  exit 1
}

echo ">>> Lanzando maze_nav yaml"
ros2 launch maze_nav nav_base.launch.py \
  mode:=goal \
  map_source:=yaml \
  map_yaml:=results/parte_a/casa_map_tuned.yaml \
  pose_topic:=/odom \
  pose_topic_type:=odometry \
  publish_loaded_map:=true \
  use_sim_time:=true \
  >"${NAV_LOG}" 2>&1 &
NAV_PID=$!
sleep 4

CLIENT_ARGS=()
for goal in "${GOALS[@]}"; do
  CLIENT_ARGS+=(--goal "${goal}")
done

echo ">>> Goals: ${GOALS[*]}"
set +e
python3 "${SCRIPT_DIR}/smoke_goal_nav_client.py" "${CLIENT_ARGS[@]}"
STATUS=$?
set -e

echo ">>> Tail nav log"
tail -n 60 "${NAV_LOG}" || true
echo ">>> Tail sim log"
tail -n 60 "${SIM_LOG}" || true

exit "${STATUS}"

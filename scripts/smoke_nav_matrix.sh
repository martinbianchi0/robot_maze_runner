#!/usr/bin/env bash
# Merge gate headless: custom_casa + maze_nav + matriz de casos diagnosticos.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${NAV_MATRIX_OUTPUT_DIR:-/tmp/maze_nav_matrix_${STAMP}}"
SIM_LOG="${OUTPUT_DIR}/sim.log"
NAV_LOG="${OUTPUT_DIR}/nav.log"
RESULTS_JSONL="${OUTPUT_DIR}/results.jsonl"

CASES=("$@")
if [ "${#CASES[@]}" -eq 0 ]; then
  CASES=(
    "straight_easy:reached:0.8,0.0,0"
    "perpendicular_turn:reached:0.0,1.0,90"
    "curve_goal:reached:0.9,0.45,0"
    "problem_white_points:diagnostic:0.55,0.10,0"
    "near_wall_probe:diagnostic:0.9,0.55,0"
    "blocked_known:blocked:1.15,1.25,0"
  )
fi

source /opt/ros/humble/setup.bash
cd "${ROOT_DIR}"
mkdir -p "${OUTPUT_DIR}"
: >"${RESULTS_JSONL}"

cleanup_case() {
  set +e
  if [ -n "${NAV_PID:-}" ]; then kill "${NAV_PID}" 2>/dev/null || true; fi
  if [ -n "${SIM_PID:-}" ]; then kill "${SIM_PID}" 2>/dev/null || true; fi
  sleep 2
  if [ -n "${NAV_PID:-}" ]; then kill -9 "${NAV_PID}" 2>/dev/null || true; fi
  if [ -n "${SIM_PID:-}" ]; then kill -9 "${SIM_PID}" 2>/dev/null || true; fi
  "${SCRIPT_DIR}/kill_all.sh" >/tmp/maze_nav_matrix_cleanup.log 2>&1 || true
  unset NAV_PID SIM_PID
}
trap cleanup_case EXIT

echo ">>> Output: ${OUTPUT_DIR}"
echo ">>> Limpiando procesos previos"
"${SCRIPT_DIR}/kill_all.sh" >/tmp/maze_nav_matrix_kill.log 2>&1 || true

if [ "${SMOKE_SKIP_BUILD:-0}" != "1" ]; then
  echo ">>> Build de paquetes necesarios"
  colcon build --symlink-install --packages-select turtlebot3_custom_simulation maze_nav
fi
source install/setup.bash
export TURTLEBOT3_MODEL=burger

GATE_STATUS=0

for spec in "${CASES[@]}"; do
  IFS=':' read -r CASE_NAME EXPECTED GOAL <<<"${spec}"
  if [ -z "${CASE_NAME}" ] || [ -z "${EXPECTED}" ] || [ -z "${GOAL}" ]; then
    echo "FAIL: caso invalido '${spec}', usar name:expected:x,y,yaw"
    exit 2
  fi

  CASE_DIR="${OUTPUT_DIR}/${CASE_NAME}"
  CASE_JSON="${CASE_DIR}/result.json"
  mkdir -p "${CASE_DIR}"
  echo
  echo ">>> Caso ${CASE_NAME}: expected=${EXPECTED} goal=${GOAL}"
  cleanup_case

  ros2 launch turtlebot3_custom_simulation custom_casa.launch.py gui:=false \
    >"${CASE_DIR}/sim.log" 2>&1 &
  SIM_PID=$!

  timeout 150 bash -lc '
    source /opt/ros/humble/setup.bash
    source ~/Robotica/tp_final_ws/install/setup.bash
    topics_file=/tmp/maze_nav_matrix_topics.txt
    has_topic() {
      ros2 topic list >"${topics_file}" 2>/dev/null || return 1
      grep -qx "$1" "${topics_file}"
    }
    until has_topic /scan; do sleep 1; done
    until has_topic /odom; do sleep 1; done
    until has_topic /robot_description; do sleep 1; done
  ' || {
    echo "FAIL: la simulacion no publico topics de robot"
    tail -n 80 "${CASE_DIR}/sim.log" || true
    exit 1
  }

  ros2 launch maze_nav nav_base.launch.py \
    mode:=goal \
    map_source:=yaml \
    map_yaml:=results/parte_a/casa_map_tuned.yaml \
    pose_topic:=/odom \
    pose_topic_type:=odometry \
    publish_loaded_map:=true \
    use_sim_time:=true \
    >"${CASE_DIR}/nav.log" 2>&1 &
  NAV_PID=$!
  sleep 4

  set +e
  python3 "${SCRIPT_DIR}/smoke_nav_matrix_client.py" \
    --case-name "${CASE_NAME}" \
    --expected "${EXPECTED}" \
    --goal "${GOAL}" \
    --output-json "${CASE_JSON}"
  CASE_STATUS=$?
  set -e

  if [ -f "${CASE_JSON}" ]; then
    cat "${CASE_JSON}" >>"${RESULTS_JSONL}"
  else
    echo "{\"case\":\"${CASE_NAME}\",\"classification\":\"NEEDS_MANUAL_RVIZ_REVIEW\",\"gate_ok_for_parte_b\":false,\"diagnosis\":\"client did not write result\"}" >>"${RESULTS_JSONL}"
  fi

  if [ "${CASE_STATUS}" -ne 0 ]; then
    GATE_STATUS=1
  fi

  echo ">>> Tail nav log ${CASE_NAME}"
  tail -n 25 "${CASE_DIR}/nav.log" || true
  cleanup_case
done

echo
echo ">>> Matriz resumida"
python3 - "${RESULTS_JSONL}" <<'PY'
import json
import sys

path = sys.argv[1]
rows = []
with open(path, encoding='utf-8') as fh:
    for line in fh:
        if line.strip():
            rows.append(json.loads(line))

headers = [
    'case',
    'expected',
    'result',
    'classification',
    'gate',
    'err_m',
    'moved_m',
    'min_front_m',
    'replans',
    'path_valid',
    'reason',
]
print('| ' + ' | '.join(headers) + ' |')
print('| ' + ' | '.join(['---'] * len(headers)) + ' |')
for row in rows:
    debug = row.get('nav_debug') or {}
    values = [
        row.get('case'),
        row.get('expected'),
        row.get('result'),
        row.get('classification'),
        'OK' if row.get('gate_ok_for_parte_b') else 'FAIL',
        row.get('goal_error_m'),
        row.get('moved_m'),
        row.get('min_front_scan_m'),
        row.get('replans_observed'),
        row.get('path_valid_costmap'),
        row.get('nav_debug_reason') or debug.get('reason'),
    ]
    print('| ' + ' | '.join(str(v) for v in values) + ' |')

print()
print(f'JSONL: {path}')
PY

echo ">>> Resultado JSONL: ${RESULTS_JSONL}"
exit "${GATE_STATUS}"

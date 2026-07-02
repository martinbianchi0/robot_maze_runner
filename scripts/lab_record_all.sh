#!/usr/bin/env bash
# Kit de grabacion para demo/evidencia de Parte C en TurtleBot4 real.
#
# Uso:
#   ./scripts/lab_record_all.sh [tb4_0|tb4_1] [map_yaml]
#
# Crea results/labo_demo/<timestamp>/ con rosbag, CSV/JSONL, metadata y reporte.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

NS="${1:-tb4_0}"
MAP_YAML="${2:-maps/laberinto_lab_20260702.yaml}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_DIR:-results/labo_demo/${STAMP}}"
META_DIR="${OUT_DIR}/metadata"
mkdir -p "$META_DIR"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[lab_record_all] ERROR: no existe /opt/ros/humble/setup.bash" >&2
  exit 2
fi

if [ -f install/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u
else
  echo "[lab_record_all] aviso: no existe install/setup.bash; uso solo entorno ROS base"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[lab_record_all] ERROR: ros2 no esta en PATH" >&2
  exit 2
fi

BRANCH="$(git branch --show-current 2>/dev/null || true)"
COMMIT="$(git rev-parse HEAD 2>/dev/null || true)"
DATE_ISO="$(date --iso-8601=seconds)"

cat >"${META_DIR}/run.json" <<EOF
{
  "date": "${DATE_ISO}",
  "ns": "${NS}",
  "map_yaml": "${MAP_YAML}",
  "branch": "${BRANCH}",
  "commit": "${COMMIT}",
  "cwd": "${ROOT_DIR}",
  "ros_domain_id": "${ROS_DOMAIN_ID:-}",
  "rmw_implementation": "${RMW_IMPLEMENTATION:-}"
}
EOF

git status -sb >"${META_DIR}/git_status_sb.txt" 2>&1 || true
git log --oneline --decorate --graph -20 >"${META_DIR}/git_log_20.txt" 2>&1 || true
ros2 topic list >"${META_DIR}/topic_list_before.txt" 2>&1 || true
ros2 node list >"${META_DIR}/node_list_before.txt" 2>&1 || true

run_hz_probe() {
  local topic="$1"
  local out="$2"
  if command -v timeout >/dev/null 2>&1; then
    timeout 6s ros2 topic hz "$topic" >"$out" 2>&1 || true
  else
    echo "timeout no disponible; correr manual: ros2 topic hz ${topic}" >"$out"
  fi
}

run_hz_probe "/${NS}/scan" "${META_DIR}/hz_scan.txt"
run_hz_probe "/amcl_pose" "${META_DIR}/hz_amcl_pose.txt"
run_hz_probe "/nav_state" "${META_DIR}/hz_nav_state.txt"

TOPICS=(
  "/${NS}/scan"
  "/${NS}/odom"
  "/${NS}/cmd_vel"
  "/${NS}/oakd/rgb/preview/image_raw"
  "/${NS}/oakd/rgb/preview/camera_info"
  "/map"
  "/amcl_pose"
  "/particlecloud"
  "/goal_pose"
  "/nav_state"
  "/nav_debug"
  "/plan"
  "/cone_detections"
  "/cone_debug_image"
  "/cone_mask"
  "/mission_state"
  "/tf"
  "/tf_static"
  "/${NS}/tf"
  "/${NS}/tf_static"
)

LOGGER_PID=""
BAG_PID=""

cleanup() {
  trap - INT TERM EXIT
  echo
  echo "[lab_record_all] cerrando grabacion..."
  ros2 topic list >"${META_DIR}/topic_list_after.txt" 2>&1 || true
  ros2 node list >"${META_DIR}/node_list_after.txt" 2>&1 || true

  if [ -n "${BAG_PID}" ] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  if [ -n "${LOGGER_PID}" ] && kill -0 "${LOGGER_PID}" 2>/dev/null; then
    kill -INT "${LOGGER_PID}" 2>/dev/null || true
    wait "${LOGGER_PID}" 2>/dev/null || true
  fi

  if python3 scripts/lab_make_report.py "${OUT_DIR}"; then
    echo "[lab_record_all] reporte listo: ${OUT_DIR}/summary.md"
  else
    echo "[lab_record_all] no pude generar el reporte automaticamente."
    echo "[lab_record_all] correr: python3 scripts/lab_make_report.py ${OUT_DIR}"
  fi
}
trap cleanup INT TERM EXIT

echo "[lab_record_all] salida: ${OUT_DIR}"
echo "[lab_record_all] robot: /${NS}"
echo "[lab_record_all] mapa: ${MAP_YAML}"
echo
echo "Comandos sugeridos para la demo:"
echo
echo "# Terminal 2 - nav"
echo "ros2 launch maze_nav nav_tb4_live.launch.py \\"
echo "  map_yaml:=${MAP_YAML} ns:=${NS}"
echo
echo "# Terminal 3 - mission"
echo "ros2 launch maze_mission mission.launch.py \\"
echo "  params_file:=\$(pwd)/config/parte_c/real.yaml"
echo
echo "# Terminal 4 - detector"
echo "ros2 launch maze_perception cone_detector.launch.py \\"
echo "  params_file:=\$(pwd)/config/parte_c/real.yaml"
echo
echo "# Terminal 5 - RViz"
echo "rviz2 -d src/maze_nav/rviz/nav.rviz --ros-args -p use_sim_time:=false \\"
echo "  --remap /tf:=/${NS}/tf --remap /tf_static:=/${NS}/tf_static --remap /scan:=/${NS}/scan"
echo
echo "# Post-run manual si hiciera falta"
echo "python3 scripts/lab_make_report.py ${OUT_DIR}"
echo

python3 scripts/lab_live_logger.py --out-dir "${OUT_DIR}" --ns "${NS}" --map-yaml "${MAP_YAML}" \
  >"${META_DIR}/lab_live_logger.stdout.log" 2>"${META_DIR}/lab_live_logger.stderr.log" &
LOGGER_PID="$!"

ros2 bag record -o "${OUT_DIR}/rosbag" "${TOPICS[@]}" \
  >"${META_DIR}/rosbag.stdout.log" 2>"${META_DIR}/rosbag.stderr.log" &
BAG_PID="$!"

echo "[lab_record_all] logger PID=${LOGGER_PID}, rosbag PID=${BAG_PID}"
echo "[lab_record_all] grabando. Cortar con Ctrl-C cuando termine la prueba."

wait "${BAG_PID}"

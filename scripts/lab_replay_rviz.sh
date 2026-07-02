#!/usr/bin/env bash
# Replay de una corrida de laboratorio: rosbag + markers + RViz dedicado.
#
# Uso:
#   ./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> [tb4_0|tb4_1]
#
# Opcional:
#   RATE=0.5 ./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> tb4_0
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

RUN_DIR="${1:-}"
NS="${2:-tb4_0}"
RATE="${RATE:-1.0}"
RVIZ_CONFIG="${RVIZ_CONFIG:-rviz/lab_replay.rviz}"

if [ -z "$RUN_DIR" ]; then
  echo "Uso: ./scripts/lab_replay_rviz.sh results/labo_demo/<timestamp> [tb4_0|tb4_1]" >&2
  exit 2
fi

BAG_DIR="${RUN_DIR%/}/rosbag"
if [ ! -d "$RUN_DIR" ]; then
  echo "[lab_replay_rviz] falta run_dir: $RUN_DIR" >&2
  exit 2
fi
if [ ! -d "$BAG_DIR" ]; then
  echo "[lab_replay_rviz] falta rosbag: $BAG_DIR" >&2
  echo "[lab_replay_rviz] esperado: <run_dir>/rosbag" >&2
  exit 2
fi
if ! find "$BAG_DIR" -maxdepth 1 \( -name 'metadata.yaml' -o -name '*.db3' -o -name '*.mcap' \) | grep -q .; then
  echo "[lab_replay_rviz] $BAG_DIR no parece contener un rosbag ROS 2" >&2
  exit 2
fi
if [ ! -f "$RVIZ_CONFIG" ]; then
  echo "[lab_replay_rviz] falta config RViz: $RVIZ_CONFIG" >&2
  exit 2
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[lab_replay_rviz] ERROR: no existe /opt/ros/humble/setup.bash" >&2
  exit 2
fi

if [ -f install/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u
else
  echo "[lab_replay_rviz] aviso: no existe install/setup.bash; uso solo ROS base"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[lab_replay_rviz] ERROR: ros2 no esta en PATH" >&2
  exit 2
fi
if ! command -v rviz2 >/dev/null 2>&1; then
  echo "[lab_replay_rviz] ERROR: rviz2 no esta en PATH" >&2
  exit 2
fi

BAG_PID=""
VIZ_PID=""

cleanup() {
  trap - INT TERM EXIT
  echo
  echo "[lab_replay_rviz] cerrando replay..."
  if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [ -n "$VIZ_PID" ] && kill -0 "$VIZ_PID" 2>/dev/null; then
    kill -INT "$VIZ_PID" 2>/dev/null || true
    wait "$VIZ_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

echo "[lab_replay_rviz] run_dir=$RUN_DIR"
echo "[lab_replay_rviz] rosbag=$BAG_DIR"
echo "[lab_replay_rviz] ns=/$NS rate=$RATE"
echo "[lab_replay_rviz] pantalla opcional:"
echo "./scripts/lab_record_rviz.sh $RUN_DIR"
echo

python3 scripts/lab_viz_markers.py --ns "$NS" --ros-args -p use_sim_time:=true \
  >"${RUN_DIR%/}/lab_viz_markers.stdout.log" \
  2>"${RUN_DIR%/}/lab_viz_markers.stderr.log" &
VIZ_PID="$!"

ros2 bag play "$BAG_DIR" --clock --rate "$RATE" &
BAG_PID="$!"

rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true \
  --remap /tf:=/"$NS"/tf \
  --remap /tf_static:=/"$NS"/tf_static


#!/usr/bin/env bash
# Graba evidencia de la mision de Parte C en un rosbag (para el informe/defensa
# y el diagnostico sim-to-real). Extiende el patron de record_nav_debug.sh con
# los topicos de percepcion y de la maquina de estados de mision.
#
# Uso: ./scripts/record_mission.sh [directorio_salida]
# Correr en paralelo a la mision; grabar SIEMPRE desde el minuto 0 (sobre todo
# en el robot real), pase lo que pase.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
# shellcheck disable=SC1091
source scripts/parte_c_env.sh

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${1:-results/parte_c/evidencia/mission_${STAMP}}"
mkdir -p "$(dirname "$OUT_DIR")"

echo "[record_mission] grabando en $OUT_DIR (Ctrl-C para terminar)"
# Se listan los nombres de sim/bag y los de /tb4_0/ del robot real: ros2 bag
# record solo captura los que existan en cada corrida.
ros2 bag record -o "$OUT_DIR" \
  /scan /tb4_0/scan \
  /belief /odom /calc_odom /tb4_0/odom \
  /cmd_vel /tb4_0/cmd_vel \
  /nav_state /nav_debug /planned_path /map /global_costmap \
  /cone_detections /cone_debug_image /cone_mask /mission_state \
  /tf /tf_static /tb4_0/tf /tb4_0/tf_static

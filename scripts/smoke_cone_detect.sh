#!/usr/bin/env bash
# Smoke test C1/C2: percepcion del cono rojo contra un rosbag (o camara).
# Lanza cone_detector con el perfil dado, reproduce el bag de conos y deja
# corriendo la deteccion (con debug) para inspeccion visual en
# rqt_image_view / RViz sobre /cone_debug_image y /cone_mask.
#
# Uso: ./scripts/smoke_cone_detect.sh [perfil] [bag] [duracion_s]
#   perfil: bag (default) | sim | real
#   bag:    ruta al rosbag (default rosbags/laberinto_conos)
#
# NOTA (M1): el cliente que etiqueta segmentos, cuenta tasa de deteccion,
# mide falsos positivos por distractores y guarda mascaras/capturas en
# results/parte_c/C1 se agrega en la etapa C1. Este script es la base de arranque
# (env + launch + bag + health-check de la tasa de publicacion).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
# shellcheck disable=SC1091
source scripts/parte_c_env.sh

PROFILE="${1:-bag}"
BAG="${2:-rosbags/laberinto_conos}"
DURATION="${3:-60}"
PARAMS="$ROOT_DIR/config/parte_c/${PROFILE}.yaml"
TIMEOUT_BIN="$(command -v timeout || command -v gtimeout || true)"

cleanup() {
  [ -n "${DET_PID:-}" ] && kill "$DET_PID" 2>/dev/null || true
  [ -n "${BAG_PID:-}" ] && kill "$BAG_PID" 2>/dev/null || true
  sleep 1
  pkill -9 -f cone_detector 2>/dev/null || true
  [ -x scripts/kill_all.sh ] && ./scripts/kill_all.sh 2>/dev/null || true
}
trap cleanup EXIT

echo "[smoke_cone_detect] perfil=$PROFILE bag=$BAG duracion=${DURATION}s"
[ -f "$PARAMS" ] || { echo "no existe el perfil $PARAMS" >&2; exit 1; }
[ -d "$BAG" ] || { echo "no existe el bag $BAG" >&2; exit 1; }

ros2 launch maze_perception cone_detector.launch.py params_file:="$PARAMS" &
DET_PID=$!
sleep 3

ros2 bag play "$BAG" &
BAG_PID=$!

echo "[smoke_cone_detect] midiendo tasa de /cone_detections por ${DURATION}s ..."
if [ -n "$TIMEOUT_BIN" ]; then
  "$TIMEOUT_BIN" "$DURATION" ros2 topic hz /cone_detections || true
else
  echo "(sin 'timeout'; inspeccionar manualmente) ros2 topic hz /cone_detections"
  wait "$BAG_PID"
fi
echo "[smoke_cone_detect] listo. Debug visual: rqt_image_view /cone_debug_image"

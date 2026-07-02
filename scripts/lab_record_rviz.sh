#!/usr/bin/env bash
# Grabacion opcional de pantalla/RViz. No es requisito para la demo: si ffmpeg o
# DISPLAY no estan disponibles, avisa y sale sin romper la corrida principal.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

OUT_DIR="${1:-results/labo_demo/rviz_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT_DIR"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "[lab_record_rviz] ffmpeg no esta instalado."
  echo "[lab_record_rviz] Opcional: sudo apt install ffmpeg"
  exit 0
fi

if [ -z "${DISPLAY:-}" ]; then
  echo "[lab_record_rviz] DISPLAY no esta definido; no puedo usar x11grab."
  echo "[lab_record_rviz] Abrir RViz en una sesion X11 y volver a correr este script."
  exit 0
fi

OUT="${OUT_DIR}/rviz_screen_$(date +%Y%m%d_%H%M%S).mp4"
echo "[lab_record_rviz] grabando ${DISPLAY} -> ${OUT}"
echo "[lab_record_rviz] cortar con Ctrl-C"
ffmpeg -y -video_size "${VIDEO_SIZE:-1920x1080}" -framerate "${FPS:-15}" \
  -f x11grab -i "${DISPLAY}" -codec:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT"

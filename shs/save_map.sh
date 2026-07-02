#!/usr/bin/env bash
# Pide al nodo de SLAM que guarde el mapa actual en maps/<nombre>.{pgm,yaml}.
#
# Uso:
#   ./shs/save_map.sh              -> maps/casa_slam.{pgm,yaml}  (default)
#   ./shs/save_map.sh maze_slam    -> maps/maze_slam.{pgm,yaml}
#   ./shs/save_map.sh casa_v2      -> maps/casa_v2.{pgm,yaml}
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Sin argumento: mandamos String vacio para que el nodo use su save_basename
# (casa_slam por default; slam_tb4_live.launch.py lo pisa a maze_slam). Asi el
# fallback correcto lo elige la launch que este corriendo, no este script.
NAME="${1:-}"
NAME="${NAME%.pgm}"
NAME="${NAME%.yaml}"

ros2 topic pub --once /maze_slam/save_request_named std_msgs/msg/String "{data: '$NAME'}"
sleep 0.5
echo ""
# Sin nombre, no sabemos que basename uso el nodo -> listamos los .yaml mas
# recientes para que el usuario verifique cual quedo.
if [[ -z "$NAME" ]]; then
    echo "Mapas mas recientes en maps/:"
    ls -lt "$WS_DIR/maps/"*.yaml 2>/dev/null | head -3
else
    ls -lh "$WS_DIR/maps/" | grep "$NAME" || \
        echo "Aun no encuentro maps/$NAME.*  (el nodo lo escribe asincrono, reintenta en 1s)"
fi

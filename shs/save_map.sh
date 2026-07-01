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

NAME="${1:-casa_slam}"
NAME="${NAME%.pgm}"
NAME="${NAME%.yaml}"

ros2 topic pub --once /maze_slam/save_request_named std_msgs/msg/String "{data: '$NAME'}"
sleep 0.5
echo ""
ls -lh "$WS_DIR/maps/" | grep "$NAME" || \
    echo "Aun no encuentro maps/$NAME.*  (el nodo lo escribe asincrono, reintenta en 1s)"

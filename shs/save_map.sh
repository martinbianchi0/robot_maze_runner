#!/usr/bin/env bash
# Pide al nodo de SLAM que guarde el mapa actual en maps/casa_slam.{pgm,yaml}.
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

ros2 topic pub --once /maze_slam/save_request std_msgs/msg/Empty "{}"
sleep 0.5
echo ""
ls -lh "$WS_DIR/maps/" | grep casa_slam || echo "Aun no encuentro maps/casa_slam.*  (el nodo lo escribe asincrono, reintenta en 1s)"

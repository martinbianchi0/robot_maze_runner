#!/usr/bin/env bash
# Terminal C: teleop por teclado.
# Controles dentro de la app: w/x adelante/atras, a/d girar, s parar.
set -e

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

exec ros2 run turtlebot3_teleop teleop_keyboard

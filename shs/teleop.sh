#!/usr/bin/env bash
# Teleop por teclado para manejar el TurtleBot3 en la simulacion (casa).
# Correr en otra terminal mientras corre ./shs/casa.sh.
set -e
source "$(dirname "$0")/_common.sh"

exec ros2 run turtlebot3_teleop teleop_keyboard

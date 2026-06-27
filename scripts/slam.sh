#!/usr/bin/env bash
# Terminal B: build (si hace falta) + SLAM + RViz preconfigurado.
# Uso: ./scripts/slam.sh [args ros2 launch extra]
#   Ejemplo: ./scripts/slam.sh rviz:=false

set -e

source /opt/ros/humble/setup.bash
cd ~/ros2_ws

echo ">>> colcon build --packages-select maze_slam --symlink-install"
colcon build --packages-select maze_slam --symlink-install

source install/setup.bash

echo ">>> ros2 launch maze_slam fastslam.launch.py $*"
exec ros2 launch maze_slam fastslam.launch.py "$@"

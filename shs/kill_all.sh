#!/usr/bin/env bash
# Mata Gazebo, RViz, teleop y nodos ROS huerfanos. Util cuando algo queda colgado.
pkill -f "gz(server|client)"   2>/dev/null || true
pkill -f gzserver              2>/dev/null || true
pkill -f gzclient              2>/dev/null || true
pkill -f rviz2                 2>/dev/null || true
pkill -f teleop_keyboard       2>/dev/null || true
pkill -f fastslam_node         2>/dev/null || true
pkill -f "ros2 bag play"       2>/dev/null || true
pkill -f "ros2 launch"         2>/dev/null || true
pkill -f "ros2 run"            2>/dev/null || true
echo "Limpieza ok."

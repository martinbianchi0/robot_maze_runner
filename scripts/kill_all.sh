#!/usr/bin/env bash
# Mata todo lo del workflow: sim, SLAM, RViz, teleop.
# Si /calc_odom salta entre valores inconsistentes, casi seguro hay dos
# instancias del simulador publicando al mismo topic. Esto las mata todas.

echo ">>> Killing gazebo (gzserver, gzclient)..."
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null

echo ">>> Killing turtlebot3_custom_simulation..."
pkill -9 -f turtlebot3_custom_simulation 2>/dev/null

echo ">>> Killing SLAM nodes..."
pkill -9 -f grid_fastslam 2>/dev/null
pkill -9 -f occupancy_mapper 2>/dev/null

echo ">>> Killing rviz2, ros2 launch, teleop..."
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f teleop_keyboard 2>/dev/null
pkill -9 -f turtlebot3_teleop 2>/dev/null

sleep 2
REMAINING=$(ps -ef | grep -cE "gzserver|gzclient|turtlebot3_custom|grid_fastslam|ros2 launch (maze_slam|turtlebot)" | grep -v grep)
echo ">>> Procesos relevantes restantes: ${REMAINING}"
ps -ef | grep -E "gzserver|gzclient|turtlebot3_custom|grid_fastslam|ros2 launch (maze|turtle)" | grep -v grep || echo "(ninguno)"

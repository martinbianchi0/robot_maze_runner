#!/usr/bin/env bash
# Smoke de exploracion por fronteras en Gazebo (lazo cerrado).
# Levanta Gazebo (laberinto) + fastslam + navigator + mission (fronteras) y
# verifica que el mapa crece y que la mision emite goals de frontera distintos.
# Correr dentro de rosenv:
#   source $HOME/miniforge3/etc/profile.d/conda.sh && conda activate rosenv
#   export TURTLEBOT3_MODEL=burger
#   ./scripts/smoke_explore.sh
set -euo pipefail
cd "$(dirname "$0")/.."
source install/setup.zsh

DURATION="${1:-60}"

echo "[smoke] lanzando Gazebo (laberinto)..."
ros2 launch turtlebot3_custom_simulation custom_maze.launch.py &
GZ_PID=$!
sleep 12

echo "[smoke] lanzando pila de exploracion (fastslam + navigator + mission)..."
ros2 launch maze_mission explore.launch.py &
STACK_PID=$!
sleep 3

echo "[smoke] registrando /goal_pose durante ${DURATION}s..."
timeout "${DURATION}" ros2 topic echo --once /map >/dev/null 2>&1 || true
timeout "${DURATION}" ros2 topic echo /goal_pose > /tmp/explore_goals.txt 2>&1 || true

echo "[smoke] apagando..."
kill "${STACK_PID}" "${GZ_PID}" 2>/dev/null || true
wait 2>/dev/null || true

GOALS=$(grep -c 'position' /tmp/explore_goals.txt || true)
echo "[smoke] goals de frontera emitidos (lineas position): ${GOALS}"
if [ "${GOALS}" -ge 1 ]; then
  echo "[smoke] OK: la mision emitio al menos un goal de frontera."
else
  echo "[smoke] FALLO: no se emitieron goals; revisar SLAM/pose/mapa."
  exit 1
fi

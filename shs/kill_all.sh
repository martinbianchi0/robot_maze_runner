#!/usr/bin/env bash
# Mata Gazebo, RViz, teleop, el SLAM y los nodos de la sim que quedan huerfanos.
# Importante: el nodo de la catedra (turtlebot3_custom_simulation) y robot_state_publisher
# NO mueren solos al cortar el launch -> si no se matan, se acumulan varias instancias
# publicando /calc_odom a la vez y el SLAM recibe odometria contradictoria (mapa roto).
for pat in \
    gzserver gzclient \
    rviz2 \
    teleop_keyboard \
    fastslam_node \
    map_publisher \
    localizer \
    navigator \
    turtlebot3_custom_simulation \
    robot_state_publisher \
    parameter_bridge \
    "ros2 bag play" \
    "ros2 launch" \
    "ros2 run"; do
    pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 1
echo "Limpieza ok."

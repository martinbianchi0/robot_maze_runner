#!/usr/bin/env bash
# Lanza la simulacion de la cátedra: Gazebo Classic + TurtleBot3 Burger en la casa.
# Es el entorno de la Parte B (navegacion en simulacion).
#
# Uso:
#   ./shs/casa.sh          -> custom_casa        (casa vacia)
#   ./shs/casa.sh obs      -> custom_casa_obs    (con obstaculos simples)
#   ./shs/casa.sh obs2     -> custom_casa_obs2   (mas obstaculos / rutas cerradas)
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Limpieza automatica: mata Gazebo/nodos de sim previos. CLAVE en la casa, porque
# el nodo de la catedra y robot_state_publisher NO mueren al cortar el launch -> sin
# esto se acumulan varias instancias publicando /calc_odom y el mapa sale roto.
bash "$WS_DIR/shs/kill_all.sh"
sleep 0.5

# Build incremental (por si tocaste el paquete de simulacion).
"$WS_DIR/shs/build.sh"
source "$INSTALL_BASE/local_setup.bash"

case "${1:-}" in
    ""|casa)  LAUNCH="custom_casa.launch.py" ;;
    obs)      LAUNCH="custom_casa_obs.launch.py" ;;
    obs2)     LAUNCH="custom_casa_obs2.launch.py" ;;
    *) echo "variante desconocida: $1 (usa: casa | obs | obs2)" >&2; exit 1 ;;
esac

echo "Lanzando $LAUNCH (TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL)..."
exec ros2 launch turtlebot3_custom_simulation "$LAUNCH"

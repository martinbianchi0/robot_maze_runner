#!/usr/bin/env bash
# Parte B: navegacion autonoma en la casa (localizacion MCL + A* + control + FSM).
#
# Flujo (3 terminales):
#   T1: ./shs/casa.sh              # simulacion (o ./shs/casa.sh obs)
#   T2: ./shs/nav.sh               # esta pila de navegacion + RViz
#   En RViz: "2D Pose Estimate" (pose inicial) y despues "2D Goal Pose" (objetivo).
#
# Mapa: por defecto usa el mapa de la Parte A (maps/casa_slam.yaml). Si no existe,
# cae al mapa de referencia de la catedra. Se puede pisar:  ./shs/nav.sh /ruta/mapa.yaml
# Flag --no-rviz para solo la pila.
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Matar pila de nav / rviz previa (evita dobles publicadores).
for p in map_publisher localizer navigator "rviz2 .*nav.rviz"; do
    pkill -f "$p" 2>/dev/null || true
done
sleep 0.5

"$WS_DIR/shs/build.sh"
source "$INSTALL_BASE/local_setup.bash"

WITH_RVIZ=1
MAP=""
for a in "$@"; do
    case "$a" in
        --no-rviz) WITH_RVIZ=0 ;;
        *.yaml)    MAP="$a" ;;
    esac
done

# Elegir mapa: arg > mapa de la Parte A > mapa de referencia de la catedra.
if [[ -z "$MAP" ]]; then
    if [[ -f "$WS_DIR/maps/casa_slam.yaml" ]]; then
        MAP="$WS_DIR/maps/casa_slam.yaml"
    else
        MAP="$WS_DIR/src/turtlebot3_custom_simulation/worlds/map/map.yaml"
        echo "AVISO: no hay mapa de la Parte A (maps/casa_slam.yaml); uso el de la catedra."
    fi
fi
echo "Mapa de navegacion: $MAP"

cleanup() {
    [[ -n "${RVIZ_PID:-}" ]] && kill "$RVIZ_PID" 2>/dev/null || true
    [[ -n "${NAV_PID:-}" ]] && kill "$NAV_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "$WITH_RVIZ" -eq 1 ]]; then
    rviz2 -d "$WS_DIR/src/maze_nav/rviz/nav.rviz" --ros-args -p use_sim_time:=true &
    RVIZ_PID=$!
fi

ros2 launch maze_nav nav.launch.py map_yaml:="$MAP" use_sim_time:=true &
NAV_PID=$!

wait -n

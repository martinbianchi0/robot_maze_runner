#!/usr/bin/env bash
# Parte B live sobre el TB4 REAL: mapa + MCL + navigator + RViz. El robot se
# localiza en un mapa hecho antes (con ./shs/mapear_tb4.sh + save_map) y
# navega a la goal que le pases con la herramienta "2D Goal Pose".
#
# Uso:
#   ./shs/navegar_tb4.sh                                  # ns=tb4_0, mapa auto
#   ./shs/navegar_tb4.sh --ns tb4_1                       # el TB4 del lab
#   ./shs/navegar_tb4.sh --ns tb4_1 --map /ruta.yaml
#   ./shs/navegar_tb4.sh --ns tb4_1 --v-max 0.10          # mas lento
#   ./shs/navegar_tb4.sh --bag                            # probar con rosbag
#
# Probar sin robot (validacion previa al turno):
#   T1: ./shs/bag.sh                       # rosbag del laberinto (o bag_conos.sh)
#   T2: ./shs/navegar_tb4.sh --bag         # nav contra el bag (mapa maze_slam.yaml)
#   Nota: el bag ejecuta la trayectoria grabada, cmd_vel no mueve nada. Sirve
#   para validar MCL, plan y RViz sin robot.
#
# Antes de correr en el ROBOT REAL:
#   1) TB4 encendido y booteado.
#   2) export ROS_DOMAIN_ID=<X>   (el mismo que usa el TB4).
#   3) Tener un mapa (default: maps/laberinto_lab_*.yaml o maps/maze_slam.yaml).
#
# En RViz:
#   - "2D Pose Estimate" en la pose real del robot (con orientacion aproximada).
#   - Chequear que el LaserScan se pega a las paredes.
#   - "2D Goal Pose" al objetivo.
set -e
source "$(dirname "$0")/_common.sh"
source "$(dirname "$0")/_tb4_common.sh"
cd "$WS_DIR"

NS="tb4_0"
WITH_RVIZ=1
MAP=""
VMAX="0.12"
WMAX="0.8"
USE_SIM=false     # --bag lo pone en true (rosbag publica /clock)
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ns)       NS="$2"; shift 2 ;;
        --no-rviz)  WITH_RVIZ=0; shift ;;
        --map)      MAP="$2"; shift 2 ;;
        --v-max)    VMAX="$2"; shift 2 ;;
        --w-max)    WMAX="$2"; shift 2 ;;
        --bag)      USE_SIM=true; shift ;;
        *)          echo "arg desconocido: $1" >&2; shift ;;
    esac
done

# Limpieza selectiva: en modo --bag NO matamos el 'ros2 bag play' del usuario
# (kill_all lo mataba y despues el pre-flight fallaba porque no habia scan).
if [[ "$USE_SIM" == "true" ]]; then
    for pat in rviz2 fastslam_node map_publisher localizer navigator \
               "ros2 launch" "ros2 run"; do
        pkill -9 -f "$pat" 2>/dev/null || true
    done
    sleep 0.5
    echo "Limpieza (preservando rosbag) ok."
else
    bash "$WS_DIR/shs/kill_all.sh"
    sleep 0.5
fi

"$WS_DIR/shs/build.sh"
source "$INSTALL_BASE/local_setup.bash"

# Elegir mapa del laberinto. Orden: --map > el mas reciente laberinto_lab_* >
# maze_slam.yaml. NO caemos a casa_slam.yaml: este script es solo para el
# laberinto real; si no hay un mapa de laberinto, abortamos.
if [[ -z "$MAP" ]]; then
    MAP="$(ls -1t "$WS_DIR/maps/laberinto_lab_"*.yaml 2>/dev/null | head -1 || true)"
fi
if [[ -z "$MAP" ]] && [[ -f "$WS_DIR/maps/maze_slam.yaml" ]]; then
    MAP="$WS_DIR/maps/maze_slam.yaml"
fi
if [[ -z "$MAP" ]]; then
    echo "ERROR: no encuentro un mapa del laberinto." >&2
    echo "  Opciones:" >&2
    echo "    - Mapear en el turno: ./shs/mapear_tb4.sh --ns $NS + ./shs/save_map.sh" >&2
    echo "    - Pasar uno explicito: ./shs/navegar_tb4.sh --map /ruta.yaml --ns $NS" >&2
    exit 1
fi

tb4_precheck "$NS" "/$NS/scan" "/$NS/odom"

echo "Navegar TB4: ns=$NS mapa=$MAP v_max=$VMAX w_max=$WMAX use_sim_time=$USE_SIM"

cleanup() {
    [[ -n "${RVIZ_PID:-}" ]] && kill "$RVIZ_PID" 2>/dev/null || true
    [[ -n "${STACK_PID:-}" ]] && kill "$STACK_PID" 2>/dev/null || true
    # Salvavidas: parar el robot al morir el script.
    tb4_stop_cmd_vel "$NS"
}
trap cleanup EXIT INT TERM

if [[ "$WITH_RVIZ" -eq 1 ]]; then
    RVIZ_CFG="$WS_DIR/src/maze_nav/rviz/nav.rviz"
    # Ver comentario largo en mapear_tb4.sh: en --bag el tf_static queda en
    # el bus global, en --real ambos son namespaced.
    if [[ "$USE_SIM" == "true" ]]; then
        RVIZ_REMAPS=(-r /tf:="/$NS/tf" -r /scan:="/$NS/scan")
    else
        RVIZ_REMAPS=(-r /tf:="/$NS/tf" -r /tf_static:="/$NS/tf_static" -r /scan:="/$NS/scan")
    fi
    rviz2 -d "$RVIZ_CFG" \
        --ros-args -p use_sim_time:="$USE_SIM" \
        "${RVIZ_REMAPS[@]}" &
    RVIZ_PID=$!
fi

ros2 launch maze_nav nav_tb4_live.launch.py \
    map_yaml:="$MAP" ns:="$NS" v_max:="$VMAX" w_max:="$WMAX" \
    use_sim_time:="$USE_SIM" &
STACK_PID=$!

wait -n

#!/usr/bin/env bash
# Parte C live sobre el TB4 REAL (tb4_0 o tb4_1): nav + mission + cone detector
# + RViz. El mission_node patrulla waypoints haciendo giro-scan; cuando el
# detector ve un cono lo estima en /map, valida contra el costmap inflado y
# publica /goal_pose para que el navigator lo alcance.
#
# Uso:
#   ./shs/parte_c_tb4.sh                    # default ns=tb4_0, perfil real.yaml
#   ./shs/parte_c_tb4.sh --ns tb4_1         # perfil real_tb4_1.yaml auto
#   ./shs/parte_c_tb4.sh --ns tb4_1 --map maps/otro.yaml
#   ./shs/parte_c_tb4.sh --ns tb4_1 --params config/parte_c/otro.yaml
#   ./shs/parte_c_tb4.sh --bag              # probar con rosbag_conos
#
# Antes de correr en el ROBOT REAL:
#   1) TB4 encendido y booteado, camara + LIDAR activos.
#   2) export ROS_DOMAIN_ID=<X>   (el mismo que usa el TB4).
#   3) 'ros2 topic list | grep /tb4_1'  -> ver scan/odom/tf/preview/image.
#
# Que ves en RViz (config parte_c.rviz):
#   - Mapa + LaserScan + nube MCL + pose estimada.
#   - Waypoints (esferas azules/amarillo/verde) + path A*.
#   - Marker rojo del CONO estimado en /map cuando el detector lo confirma.
#   - Imagen anotada del detector (topico /cone_debug_image).
#
# E-stop en otra terminal:
#   ./shs/estop.sh tb4_1
set -e
source "$(dirname "$0")/_common.sh"
source "$(dirname "$0")/_tb4_common.sh"
cd "$WS_DIR"

NS="tb4_0"
WITH_RVIZ=1
MAP=""
PARAMS=""
USE_SIM=false           # --bag lo pone en true
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ns)       NS="$2"; shift 2 ;;
        --no-rviz)  WITH_RVIZ=0; shift ;;
        --map)      MAP="$2"; shift 2 ;;
        --params)   PARAMS="$2"; shift 2 ;;
        --bag)      USE_SIM=true; shift ;;
        *)          echo "arg desconocido: $1" >&2; shift ;;
    esac
done

# Limpieza. En --bag preservamos el ros2 bag play que el usuario levanto en T1.
if [[ "$USE_SIM" == "true" ]]; then
    for pat in rviz2 fastslam_node map_publisher localizer navigator \
               cone_detector mission_node "ros2 launch" "ros2 run"; do
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

# Perfil default segun ns si el usuario no lo forzo con --params.
# En --bag NO auto-cambiamos a bag.yaml: ese perfil espera /scan sin ns (asume
# remap), pero nuestro bag.sh no lo remapea -> mission_node se colgaba sin scan.
# real.yaml usa /tb4_0/scan directo, funciona con bag y con robot real.
if [[ -z "$PARAMS" ]]; then
    case "$NS" in
        tb4_1) PARAMS="$WS_DIR/config/parte_c/real_tb4_1.yaml" ;;
        *)     PARAMS="$WS_DIR/config/parte_c/real.yaml" ;;
    esac
fi
if [[ ! -f "$PARAMS" ]]; then
    echo "ERROR: no encuentro perfil de parametros: $PARAMS" >&2
    exit 1
fi

# Mapa: --map > laberinto_lab_*.yaml mas nuevo > maze_slam.yaml.
if [[ -z "$MAP" ]]; then
    MAP="$(ls -1t "$WS_DIR/maps/laberinto_lab_"*.yaml 2>/dev/null | head -1 || true)"
fi
if [[ -z "$MAP" ]] && [[ -f "$WS_DIR/maps/maze_slam.yaml" ]]; then
    MAP="$WS_DIR/maps/maze_slam.yaml"
fi
if [[ -z "$MAP" ]]; then
    echo "ERROR: no encuentro mapa. Usa --map /ruta.yaml (o mapea con mapear_tb4.sh)." >&2
    exit 1
fi

# Pre-flight de topicos.
tb4_precheck "$NS" "/$NS/scan" "/$NS/odom"

echo "Parte C TB4: ns=$NS mapa=$MAP params=$PARAMS use_sim_time=$USE_SIM"

cleanup() {
    [[ -n "${RVIZ_PID:-}" ]] && kill "$RVIZ_PID" 2>/dev/null || true
    [[ -n "${NAV_PID:-}" ]] && kill "$NAV_PID" 2>/dev/null || true
    [[ -n "${MISSION_PID:-}" ]] && kill "$MISSION_PID" 2>/dev/null || true
    [[ -n "${CONE_PID:-}" ]] && kill "$CONE_PID" 2>/dev/null || true
    # Frenar al TB4 al morir.
    tb4_stop_cmd_vel "$NS"
}
trap cleanup EXIT INT TERM

if [[ "$WITH_RVIZ" -eq 1 ]]; then
    RVIZ_CFG="$INSTALL_BASE/share/maze_mission/rviz/parte_c.rviz"
    if [[ ! -f "$RVIZ_CFG" ]]; then
        RVIZ_CFG="$WS_DIR/src/maze_nav/rviz/nav.rviz"
    fi
    # Mismo criterio que en mapear/navegar: en --bag no remapear /tf_static
    # (bag.sh lo puso en el bus global). En real, remapear ambos.
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

# 1) nav (map_publisher + localizer + navigator). Remapea /cmd_vel a /<ns>/cmd_vel.
ros2 launch maze_nav nav_tb4_live.launch.py \
    map_yaml:="$MAP" ns:="$NS" use_sim_time:="$USE_SIM" &
NAV_PID=$!

# 2) mission (patrol de waypoints + logica de cono).
ros2 launch maze_mission mission.launch.py params_file:="$PARAMS" &
MISSION_PID=$!

# 3) cone detector (segmenta HSV + publica ConeDetections + imagen debug).
ros2 launch maze_perception cone_detector.launch.py params_file:="$PARAMS" &
CONE_PID=$!

wait -n

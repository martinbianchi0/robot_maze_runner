#!/usr/bin/env bash
# Genera el MAPA ENTREGABLE de Parte A offline, con maxima calidad (config C).
# Procesa todo el bag sin presion de tiempo real -> reproduce la calidad de los
# PNG de tuning (el nodo en vivo con 80 particulas no llega y sale peor).
#
#   ./shs/build_map.sh          # config C (80p, res 0.03) - tarda varios min
#   ./shs/build_map.sh --fast   # config liviana para iterar rapido
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Asegurar la copia local sana del bag (bag.sh la extrae a disco local).
LOCAL_DIR="${MAZE_BAG_CACHE:-/var/tmp/maze_slam_bag}"
if [[ ! -f "$LOCAL_DIR/laberinto/laberinto_0.db3" ]]; then
    echo "No hay bag local; corré ./shs/bag.sh una vez (extrae a disco local)." >&2
    echo "Lo extraigo ahora..."
    bash "$WS_DIR/shs/bag.sh" --extract-only
fi

exec python3 "$WS_DIR/shs/build_map.py" "$@"

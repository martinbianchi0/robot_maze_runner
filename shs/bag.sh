#!/usr/bin/env bash
# Reproduce el rosbag del laberinto (TB4 real, grabado en el lab).
# Topics: /tb4_0/scan, /tb4_0/odom, /tb4_0/tf, /tb4_0/imu, /tb4_0/oakd/...
#
# IMPORTANTE: el bag se extrae a DISCO LOCAL (no a la raid5). El .db3 de 8GB se
# corrompe al escribirse en /dev/md0 (raid5); en disco local esta perfecto.
#
# Flags:
#   --rate N   velocidad (default 1.0). Bajá a 0.5 si el SLAM no llega.
#   --loop     reproduce en loop.
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"

# Limpieza automatica: mata bag/slam/rviz/gazebo previos para arrancar en limpio
# (evita dos nodos publicando /map, bags zombies, etc). Corre antes del play propio.
bash "$WS_DIR/shs/kill_all.sh"
sleep 0.5

"$WS_DIR/shs/build.sh"
source "$INSTALL_BASE/local_setup.bash"

BAG_ZIP="$WS_DIR/maps/laberinto.zip"           # zip en la raid5 (se lee con CRC, viaja sano)
LOCAL_DIR="${MAZE_BAG_CACHE:-/var/tmp/maze_slam_bag}"   # destino LOCAL (no raid5)
LOCAL_BAG="$LOCAL_DIR/laberinto"

db3_ok() {
    python3 - "$1" <<'PYEOF' 2>/dev/null
import sqlite3, sys
try:
    sqlite3.connect(f'file:{sys.argv[1]}?mode=ro', uri=True).execute(
        "SELECT COUNT(*) FROM messages").fetchone()
except Exception:
    sys.exit(1)
PYEOF
}

# Asegurar una copia LOCAL sana del bag.
if ! db3_ok "$LOCAL_BAG/laberinto_0.db3"; then
    if [[ ! -f "$BAG_ZIP" ]]; then
        echo "ERROR: no encuentro $BAG_ZIP" >&2
        exit 1
    fi
    echo "Extrayendo el bag a disco local ($LOCAL_DIR), ~8GB, una sola vez..."
    rm -rf "$LOCAL_DIR"; mkdir -p "$LOCAL_DIR"
    python3 - "$BAG_ZIP" "$LOCAL_DIR" <<'PYEOF'
import sys, zipfile, time
zip_path, out_dir = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(zip_path) as z:
    members = z.infolist()
    total = sum(m.file_size for m in members) or 1
    done = 0; t0 = time.time()
    for m in members:
        # .read()+write valida el CRC del zip al terminar cada archivo
        with z.open(m) as src:
            dst_path = out_dir + '/' + m.filename
            import os
            os.makedirs(os.path.dirname(dst_path), exist_ok=True) if m.filename.endswith('/') is False else os.makedirs(dst_path, exist_ok=True)
            if m.filename.endswith('/'):
                continue
            with open(dst_path, 'wb') as out:
                while True:
                    chunk = src.read(16 * 1024 * 1024)
                    if not chunk: break
                    out.write(chunk); done += len(chunk)
                    pct = 100.0 * done / total
                    print(f'\r  {pct:5.1f}%  ({done/1e9:.2f}/{total/1e9:.2f} GB)  {m.filename}',
                          end='', flush=True)
print()
PYEOF
    if ! db3_ok "$LOCAL_BAG/laberinto_0.db3"; then
        echo "ERROR: el .db3 quedo corrupto incluso en disco local ($LOCAL_DIR)." >&2
        echo "       Probá otro destino: MAZE_BAG_CACHE=/otro/path ./shs/bag.sh" >&2
        exit 1
    fi
    echo "Bag local OK: $LOCAL_BAG"
fi

# --extract-only: solo asegurar la copia local y salir (lo usa build_map.sh).
if [[ "${1:-}" == "--extract-only" ]]; then
    echo "Bag local listo en $LOCAL_BAG (extract-only)."
    exit 0
fi

RATE="1.0"; EXTRA=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --rate) RATE="$2"; shift 2;;
        --loop) EXTRA+=(--loop); shift;;
        *) EXTRA+=("$1"); shift;;
    esac
done

echo "Reproduciendo $LOCAL_BAG a rate ${RATE}x..."
# Remapeamos SOLO el tf_static (base_link->rplidar_link, sin problema de timing).
# El /tf dinamico del bag (odom->base_link) NO se remapea: va atrasado respecto a
# los scans y haria extrapolar a RViz. Ese transform lo publica nuestro nodo desde
# la odometria, estampado adelante (ver fastslam_node.publish_map_odom_tf).
exec ros2 bag play "$LOCAL_BAG" --rate "$RATE" --clock \
    --remap /tb4_0/tf_static:=/tf_static \
    "${EXTRA[@]}"

#!/usr/bin/env bash
# E-stop del TB4: publica cmd_vel=0 en loop a alta frecuencia para pisarle
# el control a mission_node/navigator. Dejalo corriendo en una terminal
# aparte; Ctrl-C para soltar el freno.
#
# Uso:
#   ./shs/estop.sh            # default ns=tb4_0
#   ./shs/estop.sh tb4_1
set -e
source "$(dirname "$0")/_common.sh"
cd "$WS_DIR"
source "$INSTALL_BASE/local_setup.bash" 2>/dev/null || true

NS="${1:-tb4_0}"
echo "E-STOP activo sobre /$NS/cmd_vel (Ctrl-C para soltar)."

# ros2 topic pub a 20 Hz publica 0 sin parar; al matarlo, mission recupera.
exec ros2 topic pub -r 20 "/$NS/cmd_vel" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

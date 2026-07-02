# Helpers compartidos por mapear_tb4.sh y navegar_tb4.sh.
# Sourcealo despues de _common.sh (necesita WS_DIR e INSTALL_BASE).

# tb4_precheck <ns> <req1> [req2 ...]
# Verifica que los topicos requeridos esten publicando. Aborta si no.
tb4_precheck() {
    local ns="$1"; shift
    echo "Pre-flight: chequeo que el TB4 (ns=$ns) este publicando..."
    local topics
    topics="$(timeout 3 ros2 topic list 2>/dev/null || true)"
    if [[ -z "$topics" ]]; then
        echo "" >&2
        echo "ERROR: 'ros2 topic list' vacio o cortado." >&2
        echo "  - ROS_DOMAIN_ID actual: ${ROS_DOMAIN_ID:-<vacio>}" >&2
        echo "  - Preguntale a la catedra que dominio usa el TB4 y exportalo." >&2
        exit 1
    fi
    local missing=()
    for t in "$@"; do
        if ! grep -qx "$t" <<<"$topics"; then
            missing+=("$t")
        fi
    done
    if (( ${#missing[@]} > 0 )); then
        echo "" >&2
        echo "ERROR: faltan estos topicos del TB4 (ns=$ns):" >&2
        for t in "${missing[@]}"; do echo "    $t" >&2; done
        echo "" >&2
        echo "Chequeos:" >&2
        echo "  1) TB4 encendido y booteado (~1 min)." >&2
        echo "  2) ROS_DOMAIN_ID matchea con el TB4 (actual: ${ROS_DOMAIN_ID:-<vacio>})." >&2
        echo "  3) Estas en la misma red que el TB4." >&2
        echo "  4) 'ros2 topic list | grep $ns' te devuelve algo." >&2
        echo "  5) Probaste con la otra ns? (tb4_0 <-> tb4_1)." >&2
        exit 1
    fi
    echo "  OK: veo $* publicando."
}

# tb4_stop_cmd_vel <ns>
# Publica cmd_vel=0 al topico namespaced del TB4. Se llama en el trap EXIT.
tb4_stop_cmd_vel() {
    local ns="$1"
    source "$INSTALL_BASE/local_setup.bash" 2>/dev/null || true
    timeout 1 ros2 topic pub -1 "/$ns/cmd_vel" geometry_msgs/msg/Twist \
        "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
}

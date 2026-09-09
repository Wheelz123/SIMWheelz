#!/usr/bin/env bash
# Start the carsim ECU/physics server. Extra args are passed through,
# e.g.:  ./run_sim.sh --follower        (ICSim-style drive input)
#        ./run_sim.sh --host 0.0.0.0    (expose to the LAN)
#
# Idempotent: when a sim is already listening on the requested ctrl port
# (default 20103) this prints a note and exits 0 instead of starting a
# second engine.  --help/--version/--selftest always run.
set -euo pipefail
cd "$(dirname "$0")"

CTRL=20103
HOST=127.0.0.1
SPECIAL=""
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
    a="${args[$i]}"
    case "$a" in
        --ctrl-port)
            if ((i + 1 < ${#args[@]})); then CTRL="${args[$((i + 1))]}"; fi ;;
        --ctrl-port=*) CTRL="${a#*=}" ;;
        --host)
            if ((i + 1 < ${#args[@]})); then HOST="${args[$((i + 1))]}"; fi ;;
        --host=*) HOST="${a#*=}" ;;
        --help|--version|--selftest) SPECIAL=1 ;;
    esac
done

if [ -z "$SPECIAL" ] &&
   python3 -c 'import socket,sys
s = socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=0.3)
s.close()' "$HOST" "$CTRL" 2>/dev/null; then
    echo "carsim already listening on $HOST:$CTRL - nothing to start" >&2
    exit 0
fi

exec python3 tools/carsim.py "$@"

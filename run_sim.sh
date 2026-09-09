#!/usr/bin/env bash
# Start the carsim ECU/physics server. Extra args are passed through,
# e.g.:  ./run_sim.sh --follower        (ICSim-style drive input)
#        ./run_sim.sh --ctrl-port 21300 (alternate control port)
#
# Loopback-only: the sim binds 127.0.0.1.  Idempotent: when a sim is
# already listening on the requested ctrl port (default 20103) this prints
# a note and exits 0 instead of starting a second engine.
# --help/--version/--selftest always run.
set -euo pipefail
cd "$(dirname "$0")"

CTRL=20103
SPECIAL=""
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
    a="${args[$i]}"
    case "$a" in
        --ctrl-port)
            if ((i + 1 < ${#args[@]})); then CTRL="${args[$((i + 1))]}"; fi ;;
        --ctrl-port=*) CTRL="${a#*=}" ;;
        --help|--version|--selftest) SPECIAL=1 ;;
    esac
done

if [ -z "$SPECIAL" ] &&
   python3 -c 'import socket,sys
s = socket.create_connection(("127.0.0.1", int(sys.argv[1])), timeout=0.3)
s.close()' "$CTRL" 2>/dev/null; then
    echo "carsim already listening on 127.0.0.1:$CTRL - nothing to start" >&2
    exit 0
fi

exec python3 tools/carsim.py "$@"

#!/usr/bin/env bash
# Start the carsim cockpit GUI. Extra args are passed through, e.g.:
#   ./run_cockpit.sh --host 192.168.1.50    (sim on another host, JSON ctrl channel)
set -euo pipefail
cd "$(dirname "$0")"
exec python3 tools/carsim_gui.py "$@"

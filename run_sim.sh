#!/usr/bin/env bash
# Start the carsim ECU/physics server. Extra args are passed through,
# e.g.:  ./run_sim.sh --follower        (ICSim-style drive input)
#        ./run_sim.sh --host 0.0.0.0    (expose to the LAN/setup)
set -euo pipefail
cd "$(dirname "$0")"
exec python3 tools/carsim.py "$@"

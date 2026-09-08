#!/usr/bin/env bash
# Headless verification: syntax, carsim selftest, GUI logic checks, and the
# two e2e regressions against a fresh local sim (started and stopped here).
set -euo pipefail
cd "$(dirname "$0")"

SIM_PID=""
cleanup() { [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true; }
trap cleanup EXIT

echo "== compile =="
python3 -m py_compile tools/carsim.py tools/carsim_gui.py

echo "== carsim selftest =="
python3 tools/carsim.py --selftest

echo "== carsim_gui headless check =="
python3 tools/carsim_gui.py --check

echo "== starting fresh sim for e2e =="
python3 tools/carsim.py --host 127.0.0.1 \
    --slcan-port 20102 --ctrl-port 20103 &
SIM_PID=$!
sleep 1.2

echo "== cockpit e2e =="
python3 tests/cockpit_e2e.py
echo "== autopilot phase e2e =="
python3 tests/ap_phase_e2e.py

echo "== ALL TESTS PASS =="

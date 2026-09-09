#!/usr/bin/env bash
# Headless verification: syntax, carsim selftest, GUI logic checks, and the
# e2e regressions (drive, autopilot park-gate, CAN body inject, LIN inject)
# against a fresh local sim on an isolated port (never collides with a live
# cockpit).
set -euo pipefail
cd "$(dirname "$0")"

SIM_PID=""
cleanup() { [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true; }
trap cleanup EXIT

echo "== compile =="
python3 -m py_compile tools/carsim.py tools/carsim_gui.py

echo "== renderer gate (park lane-drift) =="
python3 tests/renderer_gate_test.py

echo "== carsim selftest =="
python3 tools/carsim.py --selftest

echo "== carsim_gui headless check =="
python3 tools/carsim_gui.py --check

# Isolated port so the suite never collides with a live cockpit on 20103.
TEST_PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()')
export CARSIM_PORT=$TEST_PORT
echo "== starting fresh sim for e2e (port $TEST_PORT) =="
python3 tools/carsim.py --ctrl-port "$TEST_PORT" &
SIM_PID=$!
sleep 1.2

echo "== cockpit e2e =="
python3 tests/cockpit_e2e.py
echo "== autopilot phase e2e (park gear gate) =="
python3 tests/ap_phase_e2e.py
echo "== CAN body inject (0x130 trunk/doors latch) =="
python3 tests/body_inject_test.py
echo "== LIN inject (body modules behind the BCM) =="
python3 tests/lin_inject_test.py

echo "== ALL TESTS PASS =="

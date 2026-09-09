# CarSimWheelz — cockpit simulator quickstart

A drivable virtual car with a full OBD-II/UDS ECU cluster, all in software. No
vehicle or CAN hardware is required — the whole bus (engine, TCM and ABS ECUs
plus the classic broadcast frames) lives in `tools/carsim.py`, and
`tools/carsim_gui.py` is the dashboard you drive it from. **One command starts
the entire bench.**

| File | What it is |
|---|---|
| `tools/carsim.py` | Physics engine + ECU cluster (engine / TCM / ABS) + the CAN bus the cockpit drives |
| `tools/carsim_gui.py` | tkinter cockpit: canvas road, 3×3 gauge cluster, lamps + live data, CAN console + quick inject, CAN BUS monitor, keyboard driving, cruise + 3-phase autopilot, body switches, faults, units |
| `run_cockpit.sh` | Launcher for the cockpit — **the single entry point** |
| `run_tests.sh` | Full headless verification (compile + selftest + `--check` + both e2e) |

Requires **Python 3.10+** (tkinter for the GUI).

---

## 1. Run the bench (one command)

```bash
./run_cockpit.sh
```

That's it. If no sim is already listening on the loopback control port, the
cockpit **auto-starts the bundled engine** as `--follower` (so injected frames
actually drive the car) and prints:

```
engine auto-started (ctrl 127.0.0.1:20103); log in /tmp/carsim_engine.log
```

If a sim is already running, it is left completely untouched:

```
sim already listening on 127.0.0.1:20103 - reusing it (engine not started)
```

Behind the scenes:

- The auto-started engine runs with **`--follower`**, so quick-inject and CAN
  INJECT frames (`400#…`) actually move the car.
- The cockpit waits up to ~5 s for the engine to open its ports, and reports
  clearly if the spawn failed, the engine exited early, or a port never opened
  — each failure message points at the engine log (`/tmp/carsim_engine.log`).
- Closing the cockpit stops an engine it started itself (SIGTERM, escalating to
  SIGKILL after a grace period). A sim that was already running before the
  cockpit started is never touched.

---

## 2. Run the checks (no display needed)

```bash
./run_tests.sh          # the whole suite against a fresh sim
```

or step by step:

```bash
python3 -m py_compile tools/carsim.py tools/carsim_gui.py
python3 tools/carsim.py --selftest          # physics + OBD/UDS protocol
python3 tools/carsim_gui.py --check         # GUI decode/inject helpers
python3 tests/cockpit_e2e.py                # JSON end-to-end (needs sim running)
python3 tests/ap_phase_e2e.py               # autopilot sequence (needs sim running)
```

Expected tails: `carsim selftest: ALL PASS`, `carsim_gui headless check: ALL
PASS`, `E2E ALL PASS`, `ap phase-machine e2e: ALL PASS`.

---

## 3. Drive the cockpit

### Layout

- **LEFT** — scrolling road + car sprite (the game view)
- **MID** — tiled 3×3 instrument cluster (small, separate round gauges) + odo
- **RIGHT** — warning lamps, live-data table, CAN console + quick inject
- **FOOTER** — keyboard hints and the broadcast-ID legend

### Driving (real-car order — the sim enforces it)

1. Gear **P** is default. Press **i** (ignition ON) and wait — the starter
   cranks ~0.7 s and the engine catches at idle (~800 rpm). **I** (shift-I)
   turns ignition OFF.
2. Press **D**, then hold **↑** (throttle). Brake with **↓**, steer **←/→**.
3. Shift **P/R/N/D** with the letter keys; the TCM blocks R/P shifts above
   8 km/h, so brake first.
4. **space** = park brake, **r** = reset (back to P, engine off, DTCs cleared).

### Controls reference

| Key | Action |
|---|---|
| ↑ / ↓ (or w / s) | throttle / brake (hold) |
| ← / → | steer (hold) |
| P R N D | shift gear |
| i / I | ignition ON / OFF |
| c / C | cruise control toggle (needs D and ≥ 40 km/h) |
| + / = / - | cruise set-point ±5 km/h |
| a / A | **autopilot**: full cold-start → drive → cruise state machine (below) |
| u | units km/h ↔ mph |
| space | park brake |
| h | hazard lights |
| r | reset to P, engine off, DTCs cleared |
| F1 | in-cockpit help line |

### CAN console + quick inject (can-utils / ICSim style)

The **CAN INJECT** box parses `cansend`-style `ID#DATA` lines and sends them
straight into the sim's frame handler. Press **Enter** to send; the console
answers **OK**, **FAIL**, or a **NOTE** explaining why the frame was refused:

```
7DF#02010D0000000000      OBD mode-01 PID 0D (speed) request -> ECU replies
400#FF3203FF              DRIVE_IN: full throttle, D, steer right
0x400#FF3203FF            (0x-prefixed ids accepted too)
```

Frame legend: **0x100** ENG · **0x110** CHAS · **0x120** STEER+lights ·
**0x130** BODY · **0x140** GEAR/fuel/odo · **0x400** DRIVE_IN (throttle · brake
· gear · steer bytes) · diag **0x7DF → 0x7E8**. Click a CAN BUS line to copy it
to the clipboard; double-click loads it into the inject box.

> Injected frames move the car only when the sim runs with `--follower` (the
> auto-started engine always does). Without it, frames still enter the bus
> stream but the physics ignores them. Quick-inject buttons (throttle
> +/brake +/D/R/steer/neutral) send `0x400` for you.

### Cruise / autopilot

- **Cruise (c)** — plain speed-hold; needs D and ≥ 40 km/h; +/- adjusts the
  set-point.
- **Autopilot (a)** — 3-phase state machine, toggle is situational:
  - already driving → hold speed now (phase 3);
  - engine idling → engage D first (phase 2);
  - engine off → full cold start (phase 1: P/N → crank ~0.7 s → wait → D →
    closed-loop P-controller to target, default 100 km/h slider).
  Toggle OFF zeroes inputs immediately (control back to the driver).

### Faults / raw bus

- Fault checkboxes inject failures (MIL: misfire P0300, overheat P0217, flat
  tire); the OBD/UDS clear paths and the reset button remove them.
- The CAN BUS monitor shows the live stream: 0x100 engine @20 ms, 0x110
  chassis @20 ms, 0x120 steering/lights @50 ms, 0x130 body @50 ms, 0x140
  gear/fuel/odo @100 ms — plus anything you inject (changed frames **cyan**,
  cockpit-sent frames **amber**).

---

## 4. Troubleshooting

- **Injected frames don't move the car** — the sim needs `--follower`. The
  auto-started engine always has it; an engine you started by hand may not
  (frames still appear in the monitor without it).
- **Gears won't shift** — R/P are blocked above 8 km/h and you cannot crank in
  D. Brake, shift to P/N, then start.
- **Cruise won't engage** — it needs D *and* ≥ 40 km/h.
- **Port already in use at launch** — the run scripts are idempotent:
  `./run_sim.sh` prints `nothing to start` and exits 0 when a sim is already
  up, instead of crashing with `Address already in use`. `./run_tests.sh` starts
  its own fresh sim on a quiet port, so it can always test.
- **A single 0-frame readout right after another test completes** is a
  known connect-race transient; an immediate re-probe streams frames again.
  Not a regression.

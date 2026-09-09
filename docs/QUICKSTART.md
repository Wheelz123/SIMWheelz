# carsim — cockpit simulator quickstart

Drivable virtual car + OBD-II/UDS ECU cluster. One command runs the whole
bench: the cockpit auto-starts the engine whenever a loopback sim is not
already listening on the control port.

| File | What it is |
|---|---|
| `tools/carsim.py` | Physics engine + fake ECUs + SLCAN-over-TCP server + JSON control channel + optional SocketCAN (wired CAN bus) mode + ICSim-style `--follower` drive input |
| `tools/carsim_gui.py` | tkinter cockpit (three non-overlapping columns per the approved mockup): canvas road + tiled 3×3 gauge cluster, lamps, live-data table, CAN console + quick inject, keyboard driving, cruise + 3-phase autopilot, fault injection, units |
| `run_cockpit.sh` | Launcher for the cockpit (the single entry point) |
| `run_sim.sh` | Idempotent launcher for the engine alone |
| `tests/cockpit_e2e.py` | End-to-end JSON regression test (start → drive → MIL → reset) |
| `tests/ap_phase_e2e.py` | Autopilot 3-phase state-machine replay (cold start → D → cruise → stop) |

Requires **Python 3.10+** (tkinter for the GUI).

---

## 1. Run the bench (single entry)

```bash
./run_cockpit.sh        # or: python3 tools/carsim_gui.py
```

The cockpit connects to the JSON control channel at `127.0.0.1:20103`
(default). When nothing is listening there yet, it **auto-starts the bundled
engine** (`tools/carsim.py --follower`, SLCAN :20102 / ctrl :20103) and tells
you:

```
engine auto-started (ctrl 127.0.0.1:20103, slcan :20102); log in /tmp/carsim_engine.log
```

When a sim is already up (for example from an earlier run that is still alive),
the cockpit reuses it untouched:

```
sim already listening on 127.0.0.1:20103 - reusing it (engine not started)
```

Behind the scenes:

- Auto-start only happens for loopback hosts (`""`, `localhost`, `127.0.0.1`,
  `::1`). With `--host` pointing elsewhere the cockpit assumes a remote engine
  and just connects.
- The auto-started engine runs with **`--follower`**, so quick-inject and CAN
  INJECT frames (`400#…`) actually move the car.
- The cockpit waits up to ~5 s for the engine to open its ports. Spawn failure,
  early exit, or a port that never opens each produce a specific message that
  points at the engine log — `tempfile.gettempdir()` + `/carsim_engine.log`
  (usually `/tmp/carsim_engine.log`).
- Closing the cockpit stops an engine it started itself — SIGTERM, escalating
  to SIGKILL after a grace period. A sim that was already running before the
  cockpit started is never touched.
- `./run_sim.sh` is the engine-only launcher and is **idempotent**: if a sim
  already listens on the ctrl port it prints `carsim already listening on
  HOST:PORT - nothing to start` and exits 0. `--help` / `--version` /
  `--selftest` always run through.

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

Point the e2e tests at another host/port with `CARSIM_HOST` / `CARSIM_PORT`
(defaults 127.0.0.1 / 20103).

Expected tails: `carsim selftest: ALL PASS`, `carsim_gui headless check:
ALL PASS`, `E2E ALL PASS`, `ap phase-machine e2e: ALL PASS`.

---

## 3. Drive the cockpit

### Layout (approved mockup — no overlapping gauges)

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
straight into the sim's frame handler — the same code path a SocketCAN/vcan0
wire uses. Press **Enter** to send; the console answers **OK**, **FAIL**, or a
**NOTE** explaining why the frame was refused:

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
  tyre, …); the OBD/UDS clear paths and the reset button remove them.
- The CAN BUS monitor shows the live stream: 0x100 engine @20 ms, 0x110
  chassis @20 ms, 0x120 steering/lights @50 ms, 0x130 body @50 ms, 0x140
  gear/fuel/odo @100 ms — plus anything you inject (changed frames **cyan**,
  cockpit-sent frames **amber**).

---

## 4. Advanced — engine on its own, wire sessions, SocketCAN

### 4.1 Engine separately

```bash
./run_sim.sh --follower                     # idempotent engine launcher
python3 tools/carsim.py --host 127.0.0.1 --slcan-port 20102 --ctrl-port 20103
python3 tools/carsim.py --host 127.0.0.1 --slcan-port 20202 --ctrl-port 20203  # second bench
```

Engine CLI (`--help` for the rest):

| Option | Meaning |
|---|---|
| `--host ADDR` | bind address (default: all interfaces) |
| `--slcan-port N` | SLCAN-over-TCP port (default **20102**) |
| `--ctrl-port N` | JSON control port (default **20103**) |
| `--iface can0` | **SocketCAN mode**: ECUs + broadcasts on a real CAN wire, TCP SLCAN server off |
| `--tcp-also` | keep the TCP SLCAN server in `--iface` mode too |
| `--follower` | ICSim-style drive: frames 0x100/0x110/0x120/0x140/0x400 as remote drive input |
| `--no-traffic` | silent bench: no broadcast frames, only ECU replies |
| `--ecus 1\|2\|3` | 1 = engine only, 2 = +TCM, 3 = +ABS (default 3) |
| `--selftest` | headless physics/protocol assertions, then exit |
| `--version` | print version and exit |

Cockpit on the same host connects to the engine's ctrl port:

```bash
python3 tools/carsim_gui.py                 # -> 127.0.0.1:20103
python3 tools/carsim_gui.py --port 20203 --slcan-port 20202
```

### 4.2 Remote engine

```bash
# engine on the bench host:
python3 tools/carsim.py --host 0.0.0.0 --follower

# cockpit anywhere on the LAN / VPN:
python3 tools/carsim_gui.py --host 192.168.1.50
```

Auto-start is loopback-only by design: with a remote `--host` the cockpit never
spawns an engine, so the remote sim must be started first.

### 4.3 Talk to it from the command line

SLCAN-over-TCP (Lawicel framing):

```bash
# nc 127.0.0.1 20102   then:
V      -> V1013
O      -> open the receive channel (frames start streaming)
C      -> close the receive channel
t1100FF3203FF   # or raw "send" framings the console accepts
```

JSON control channel (`20103`), one command per line:

```json
{"t":"reset"}
{"t":"ignition","on":true}
{"t":"gear","gear":"D"}
{"t":"input","throttle":0.7,"brake":0.0,"steer":0.0}
{"t":"fault","name":"mil","on":true}
{"t":"cruise","on":true}
{"t":"cruise","delta":5}
```

and the sim pushes a full state snapshot per tick:

```json
{"t":"state","ts":1.6,"gear":"D","gear_num":1,"rpm":2790.0,"speed":5.2,
 "engine_on":true,"odo":0.0,"cruise_on":false,"mode":"run",
 "lamps":{...},"dtc":[],"dtc_abs":[],"faults":{...},"frames":[...],
 "events":[...],"switches":{...}}
```

> State key for road speed is **`speed`** (km/h).

### 4.4 SocketCAN (wired CAN bus) mode

```bash
sudo ip link set can0 up type can bitrate 500000
python3 tools/carsim.py --iface can0 --follower       # ECUs + traffic + drive on the wire
python3 tools/carsim.py --iface can0 --tcp-also       # ...and keep TCP SLCAN too
```

The ECUs answer functional requests on 0x7DF and stream 0x100–0x140 exactly
like bench mode. With `--follower`, a real steering wheel / pedal set (or
`cansend can0 400#…`) drives the simulated car.

---

## 5. Troubleshooting

- **GUI says "connecting…" forever** — the cockpit only auto-starts the engine
  for *loopback* hosts. With a remote `--host`, or when an auto-start failed,
  nothing is listening: start the engine first (`./run_sim.sh --follower`) or
  check the auto-start failure message and the engine log
  (`/tmp/carsim_engine.log`) for why it exited.
- **Injected frames don't move the car** — the sim needs `--follower`. The
  auto-started engine always has it; an engine you started by hand may not
  (frames still appear in the monitor without it).
- **Gears won't shift** — R/P are blocked above 8 km/h and you cannot crank in
  D. Brake, shift to P/N, then start.
- **Cruise won't engage** — it needs D *and* ≥ 40 km/h.
- **Port already in use at launch** — the launchers are idempotent on the
  ctrl port (`nothing to start`); if you forced a second bench anyway, pick
  fresh ports (`--slcan-port 20202 --ctrl-port 20203` on both sides).
- **A single 0-frame SLCAN readout right after another test completes** is a
  known connect-race transient; an immediate re-probe streams frames again.
  Not a regression.

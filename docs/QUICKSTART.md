# carsim — cockpit simulator quickstart

Drivable virtual car + OBD-II/UDS ECU cluster. Two components:

| File | What it is |
|---|---|
| `tools/carsim.py` | Physics engine + fake ECUs + SLCAN-over-TCP server + JSON control channel + optional SocketCAN (wired CAN bus) mode + ICSim-style `--follower` drive input |
| `tools/carsim_gui.py` | tkinter cockpit (three non-overlapping columns per the approved mockup): canvas road + tiled 3×3 gauge cluster, lamps, live-data table, CAN console + quick inject, keyboard driving, cruise + 3-phase autopilot, fault injection, units |
| `tests/cockpit_e2e.py` | End-to-end JSON regression test (start → drive → MIL → reset) |
| `tests/ap_phase_e2e.py` | Autopilot 3-phase state-machine replay (cold start → D → cruise → stop) |

Or just use the launchers: `./run_sim.sh`, `./run_cockpit.sh`, `./run_tests.sh`.

---

## 1. Start the sim

```bash
python3 tools/carsim.py --host 127.0.0.1 --slcan-port 20102 --ctrl-port 20103
```

Default bench mode: engine (0x7E8) + TCM + ABS ECUs, broadcast bus
streaming, TCP servers only. For the full self-driving/can-utils experience
add `--follower` so injected frames (0x400 DRIVE_IN etc.) actually move the
car:

```bash
python3 tools/carsim.py --follower          # same ports, ICSim-style drive
```

Startup banner:

```
carsim 0.1 - SLCAN server on 127.0.0.1:20102
  JSON control channel on 127.0.0.1:20103
  SLCAN-over-TCP: nc 127.0.0.1 20102   (Lawicel V/N/F/t framing)
  follower drive: ON (reads 0x100/0x110/0x120/0x140/0x400 as remote input)
  Ctrl-C to stop
```

Full CLI (`--help` for the rest):

| Option | Meaning |
|---|---|
| `--host ADDR` | bind address (default: all interfaces) |
| `--slcan-port N` | SLCAN-over-TCP port (default **20102**) |
| `--ctrl-port N` | JSON control port (default **20103**) |
| `--iface can0` | **SocketCAN mode**: put ECUs + broadcasts on a real CAN wire, disable the TCP SLCAN server |
| `--tcp-also` | keep the TCP SLCAN server in `--iface` mode too |
| `--follower` | ICSim-style drive: treat bus frames 0x100/0x110/0x120/0x140/0x400 as remote drive input |
| `--no-traffic` | silent bench: no broadcast frames, only ECU replies |
| `--ecus 1\|2\|3` | 1 = engine only, 2 = +TCM, 3 = +ABS (default 3) |
| `--selftest` | headless physics/protocol assertions, then exit |
| `--version` | print version and exit |

## 2. Run the checks (no display needed)

```bash
python3 -m py_compile tools/carsim.py tools/carsim_gui.py
python3 tools/carsim.py --selftest          # physics + OBD/UDS protocol
python3 tools/carsim_gui.py --check         # GUI decode/inject helpers
python3 tests/cockpit_e2e.py                # JSON end-to-end (needs sim running)
python3 tests/ap_phase_e2e.py               # autopilot sequence (needs sim running)
```

Point the e2e tests at another host/port with `CARSIM_HOST` / `CARSIM_PORT`.

Expected tails: `carsim selftest: ALL PASS`, `carsim_gui headless check:
ALL PASS`, `E2E ALL PASS`, `ap phase-machine e2e: ALL PASS`.

## 3. Start the cockpit

```bash
python3 tools/carsim_gui.py                 # defaults to 127.0.0.1:20103
python3 tools/carsim_gui.py --host 192.168.1.50   # sim on another host
```

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

The SLCAN server (20102) speaks standard Lawicel framing, so the **CAN
console** sends real `cansend`-style lines straight into the sim's frame
handler — the same code path a SocketCAN/vcan0 wire uses:

```
7DF#02010D0000000000      OBD mode-01 PID 0D (speed) request -> ECU replies
400#FF3203FF              DRIVE_IN: full throttle, D, steer right
0x400#FF3203FF            (0x-prefixed ids accepted too)
```

Frame legend: **0x100** ENG · **0x110** CHAS · **0x120** STEER+lights ·
**0x130** BODY · **0x140** GEAR/fuel/odo · **0x400** DRIVE_IN
(throttle · brake · gear · steer bytes) · diag **0x7DF → 0x7E8**.

> For injected frames to move the car, the sim must be running with
> `--follower`. Without it, frames still enter the bus stream (watchable in
> the frame inspector) but the physics ignores them. Quick-inject buttons
> (throttle +/brake +/D/R/steer/neutral) send `0x400` for you.

### Cruise / autopilot

- **Cruise (c)** — plain speed-hold; needs D and ≥ 40 km/h; +/- adjusts the
  set-point.
- **Autopilot (a)** — 3-phase state machine, toggle is situational:
  - already driving → hold speed now (phase 3);
  - engine idling → engage D first (phase 2);
  - engine off → full cold start (phase 1: P/N → crank ~0.7 s → wait →
    D → closed-loop P-controller to target, default 100 km/h slider).
  Toggle OFF zeroes inputs immediately (control back to the driver).

### Faults / raw bus

- Fault checkboxes inject failures (MIL: misfire P0300, overheat P0217, flat
  tyre, …); the OBD/UDS clear paths and the reset button remove them.
- The frame inspector shows the live stream: 0x100 engine @20 ms, 0x110
  chassis @20 ms, 0x120 steering/lights @50 ms, 0x130 body @50 ms, 0x140
  gear/fuel/odo @100 ms — plus anything you inject.

## 4. Talk to it from the command line

SLCAN-over-TCP (Lawicel framing):

```bash
# nc 127.0.0.1 20102   then:
V      -> V1013
O      -> open the receive channel (frames start streaming)
C      -> close the receive channel
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

## 5. SocketCAN (wired CAN bus) mode

```bash
sudo ip link set can0 up type can bitrate 500000
python3 tools/carsim.py --iface can0 --follower       # ECUs + traffic + drive on the wire
python3 tools/carsim.py --iface can0 --tcp-also       # ...and keep TCP SLCAN too
```

The ECUs answer functional requests on 0x7DF and stream 0x100–0x140 exactly
like bench mode. With `--follower`, a real steering wheel / pedal set (or
`cansend can0 400#…`) drives the simulated car.

## 6. Troubleshooting

- **GUI says "connecting…" forever** — the sim isn't running, or you pointed
  `--host` at the wrong address. Start the sim first; then the GUI.
- **Injected frames don't move the car** — the sim needs `--follower`
  (frames still appear in the inspector without it).
- **Old server versions dropped idle clients after 0.5 s** (a `socket.timeout`
  swallowed by a broad `except OSError`). Fixed — both reader loops now treat
  read timeouts as idle and keep the client. Passive listeners and idle GUI
  windows stay alive indefinitely.
- **Gears won't shift** — R/P are blocked above 8 km/h and you cannot crank in
  D. Brake, shift to P/N, then start.
- **Cruise won't engage** — it needs D *and* ≥ 40 km/h.
- **A single 0-frame SLCAN readout right after another test completes** is a
  known connect-race transient; an immediate re-probe streams frames again.
  Not a regression.

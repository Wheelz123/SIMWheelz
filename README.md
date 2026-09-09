# CarSimWheelz — drivable virtual CAN simulator + cockpit

A virtual car that speaks **real OBD-II / UDS and CAN over SLCAN**, with a
drivable game cockpit on top. No vehicle or CAN hardware is required: the whole
bus — engine, TCM and ABS ECUs plus the classic broadcast frames — lives in
`tools/carsim.py`, and `tools/carsim_gui.py` is the dashboard you drive it
from. **One command starts the entire bench.** Use the sim to develop and test
diagnostic clients, injection tooling and CAN monitors against a deterministic
setup, then bind the same sim to a wired SocketCAN bus if you want other bus
participants involved.

| File | What it is |
|---|---|
| `tools/carsim.py` | Physics engine + ECU cluster (engine / TCM / ABS) + SLCAN-over-TCP server + JSON control channel + optional SocketCAN (wired CAN bus) mode + ICSim-style `--follower` drive input |
| `tools/carsim_gui.py` | tkinter cockpit: canvas road, 3×3 gauge cluster, lamps + live data, CAN console + quick inject, CAN BUS monitor, keyboard driving, cruise + 3-phase autopilot, body switches, faults, units |
| `run_cockpit.sh` | Launcher for the cockpit — the single entry point (args pass through) |
| `run_sim.sh` | Launcher for the engine alone — idempotent, won't double-start a running sim |
| `run_tests.sh` | Full headless verification (compile + selftest + `--check` + both e2e) |

Requires **Python 3.10+** (tkinter for the GUI).

---

## 1. Quick start — one command

```bash
./run_cockpit.sh        # or: python3 tools/carsim_gui.py
```

That is the whole story. If no sim is listening on the loopback control port,
the cockpit **auto-starts the bundled engine** as `--follower` (so injected
frames actually drive the car) and prints:

```
engine auto-started (ctrl 127.0.0.1:20103, slcan :20102); log in /tmp/carsim_engine.log
```

If a sim is already running, it is left completely untouched:

```
sim already listening on 127.0.0.1:20103 - reusing it (engine not started)
```

Behind the scenes:

- **Auto-start** only fires for loopback hosts (`""`, `localhost`, `127.0.0.1`,
  `::1`). Point `--host` at a remote sim and the cockpit simply connects.
- The auto-started engine is stopped when the cockpit exits (SIGTERM,
  escalating to SIGKILL) — no orphan sims. Its console output is appended to
  the engine log (`tempfile.gettempdir()` + `/carsim_engine.log`, usually
  `/tmp/carsim_engine.log`).
- The cockpit waits up to ~5 s for the engine to open its ports and reports
  clearly if the spawn failed, the engine exited early, or the port never
  opened — each failure message points at the engine log.
- `./run_sim.sh` starts the engine on its own and is **idempotent**: if a sim
  already listens on the control port it prints `carsim already listening on
  127.0.0.1:20103 - nothing to start` and exits 0 (`--help` / `--version` /
  `--selftest` always run through). Extra args pass to `tools/carsim.py`.
- `./run_tests.sh` runs the whole headless suite against a fresh sim.

> If you start the engine by hand, remember `--follower` — without it,
> injected `0x400` DRIVE_IN frames appear on the bus but never move the car.

---

## 2. Driving the cockpit

### Layout
The cockpit is a single window with three non-overlapping columns (road · tiled
3×3 gauge cluster · lamps / live data / CAN console), plus a footer of keyboard
instructions. The footer is always visible under plain-tk (no ttk theme
overrides).

### Keyboard
```
DRIVE    W/↑ gas   S/↓ brake   A/← steer L   D/→ steer R
KEYBOARD P R N D gear   I IGNITION   SPACE parkbrake
         H hazards   C cruise   +/- set   A autopilot   U units   R reset
```

Start order matters (just like a real car): shift to **P or N**, press **I** to
crank (~0.7 s), then shift to **D** and drive. R/P are blocked above 8 km/h and
you cannot crank in D.

### Cruise & autopilot
- **C** — plain speed-hold; needs **D** and ≥ 40 km/h; `+`/`-` adjust the
  set-point.
- **A** — 3-phase autopilot, situational:
  - already driving → hold speed now;
  - engine idling → engage D, then hold;
  - engine off → full cold start (P/N → crank → wait → D → closed-loop
    speed hold). Toggling OFF zeroes inputs immediately.

---

## 3. Body switches (doors / trunk / hood / belt)

The **Switches** panel in the service column has checkboxes that put real
traffic onto the bus:

```
DOOR FL   DOOR FR   DOOR RL   DOOR RR   TRUNK   HOOD   BELT (default on)
```

Each toggle is passed to the sim, which broadcasts a **0x130 BODY** frame every
50 ms. This is exactly how a real BCM would announce body state, so you can
*see* (not just imagine) what each control does on the wire.

`0x130` byte 0 bit layout:

| bit | value | meaning |
|---|---|---|
| 0 | `0x01` | door FL |
| 1 | `0x02` | door FR |
| 2 | `0x04` | door RL |
| 3 | `0x08` | door RR |
| 4 | `0x10` | trunk |
| 5 | `0x20` | hood |
| 6 | `0x40` | seatbelt |

Example: `door_fl` ON broadcasts `0x130 → 0100000000000000`; reset clears all
body controls back to closed (belt stays on by default).

---

## 4. Reading the CAN BUS monitor

The **CAN BUS** tab streams every broadcast frame it hears plus anything you
inject. Five frames repeat on the bus:

| ID | name | rate | decoded fields |
|---|---|---|---|
| `0x100` | ENGINE | 20 ms | RPM · load · coolant · MAF |
| `0x110` | CHASSIS | 20 ms | speed · flags |
| `0x120` | STEER | 50 ms | steering · lamp bits |
| `0x130` | BODY | 50 ms | doors · trunk · hood · belt |
| `0x140` | GEAR | 100 ms | gear · fuel · odo |
| `0x400` | DRIVE_IN | on inject | throttle · brake · gear · steer |

### It's readable now
- **Named decode.** `0x130` prints the doors by name instead of a bitmask
  number, e.g. `doors FL,FR` or `all closed`, plus `trunk/hood/belt`. You can
  tell exactly which door is open without decoding hex.
- **Nothing clips.** The monitor wraps long frames onto a continuation line
  (`wrap=char`), so a dense line like `| doors FL  trunk closed  hood closed
  belt no` is fully visible instead of being cut off at the window edge.
- **Change highlight.** The frame that *just* changed is shown in **cyan**
  (`chg`); any frame the cockpit itself sent is **amber** (`tx`). In the middle
  of a live flood you spot the moment you flipped a switch instantly.
- **FREEZE.** Stops the live monitor so you can scroll back and click-copy a
  frame; the button flips to `LIVE >>` to resume. Dragging the scrollbar up
  also freezes into scrollback.

### Console / quick inject
The **CAN INJECT** bar parses `cansend`-style `ID#DATA` lines (e.g.
`400#FF3203FF`) and sends them onto the bus; press **Enter** in the box to
send. Quick-inject buttons emit `DRIVE_IN` frames. Anything you inject shows up
amber in the monitor and, in `--follower` mode, actually moves the car.

---

## 5. Self-tests (headless, no display)

```bash
./run_tests.sh          # compile + selftest + --check + both e2e (fresh sim)
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
PASS`, `E2E ALL PASS`, `ap phase-machine e2e: ALL PASS`. Point the e2e tests at
another host/port with `CARSIM_HOST` / `CARSIM_PORT`.

---

## 6. Advanced — running the engine separately

The cockpit normally brings the engine with it, but you can start the two
halves independently (e.g. engine on this host, cockpit elsewhere):

```bash
./run_sim.sh --follower                     # engine, ctrl :20103 / slcan :20102
./run_cockpit.sh                            # cockpit, connects to 127.0.0.1:20103
```

Engine CLI (`tools/carsim.py --help` for the rest):

| Option | Meaning |
|---|---|
| `--host ADDR` | bind address (default: all interfaces) |
| `--slcan-port N` | SLCAN-over-TCP port (default **20102**) |
| `--ctrl-port N` | JSON control port (default **20103**) |
| `--iface can0` | **SocketCAN mode**: ECUs + broadcasts on a real CAN wire, TCP SLCAN server off |
| `--tcp-also` | keep the TCP SLCAN server in `--iface` mode too |
| `--follower` | ICSim-style drive: treat frames 0x100/0x110/0x120/0x140/0x400 as remote drive input |
| `--no-traffic` | silent bench: no broadcast frames, only ECU replies |
| `--ecus 1\|2\|3` | 1 = engine only, 2 = +TCM, 3 = +ABS (default 3) |
| `--selftest` | headless physics/protocol assertions, then exit |
| `--version` | print version and exit |

The cockpit accepts the same `--host` / `--port` / `--slcan-port` flags, so a
remote cockpit is `python3 tools/carsim_gui.py --host 192.168.1.50`. For the
wire-level sessions (raw SLCAN over `nc`, the JSON control channel, and
SocketCAN bring-up) see `docs/QUICKSTART.md` § 4.

---

MIT licensed.

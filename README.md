# carsim — drivable CAN simulator ("adapter server" + cockpit)

A virtual car that speaks **real OBD-II / UDS over SLCAN-over-TCP**, exactly
like the network CAN adapter, with a drivable game cockpit on top. Use it to
develop and test the remote-CAN monitor tooling (client, GUI, can-utils)
without touching a vehicle — then put the same sim on a real SocketCAN wire and
let `cansend` drive it.

| File | What it is |
|---|---|
| `tools/carsim.py` | Physics engine + ECU cluster (engine / TCM / ABS) + SLCAN-over-TCP server + JSON control channel + optional real SocketCAN (SocketCAN) wire mode + ICSim-style `--follower` drive input |
| `tools/carsim_gui.py` | tkinter cockpit: canvas road, 3×3 gauge cluster, lamps + live data, CAN console + quick inject, CAN BUS monitor, keyboard **and** controller controller (USB) driving, cruise + 3-phase autopilot, body switches, faults, units |

Requires **Python 3.10+** (tkinter for the GUI). `SDL` is optional — it only
enables the controller/controller driver; the cockpit falls back to the keyboard.

---

## 1. Quick start

Open two terminals in the repo root:

```bash
# 1) run the sim (the "adapter server" — add --follower so injected frames drive the car)
python3 tools/carsim.py --follower

# 2) run the cockpit GUI
python3 tools/carsim_gui.py
```

Or use the launchers:

```bash
./run_sim.sh          # starts carsim.py; extra args pass through (e.g. --host 0.0.0.0)
./run_cockpit.sh      # starts the GUI; extra args pass through
./run_tests.sh        # headless self-test (compile + selftest + --check + both e2e)
```

Default ports: **20102** SLCAN-over-TCP (the adapter server) and **20103** JSON
control. Point the GUI or the e2e tests at another host with `--host` /
`CARSIM_HOST`.

---

## 2. Driving the cockpit

### Layout
The cockpit is a single window with three non-overlapping columns (road ·
tiled 3×3 gauge cluster · lamps / live data / CAN console), plus a footer of
keyboard instructions. The footer is always visible under plain-tk (no ttk
theme overrides).

### Keyboard
```
DRIVE    W/↑ gas   S/↓ brake   A/← steer L   D/→ steer R
KEYBOARD P R N D gear   I IGNITION   SPACE parkbrake
         H hazards   C cruise   +/- set   A autopilot   U units   R reset
```

Start order matters (just like a real car): shift to **P or N**, press
**I** to crank (~0.7 s), then shift to **D** and drive. R/P are blocked above
8 km/h and you cannot crank in D.

### controller / controller (optional)
Plug in (or pair over radio) the controller **before** starting the GUI —
autodetect runs at launch and re-runs on hot-plug. Triggers = gas/brake, left
stick = steer, face buttons = gear. No SDL/SDL → keyboard only, by design.

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

Each toggle is passed to the sim, which broadcasts a **0x130 BODY** frame
every 50 ms. This is exactly how a real BCM would announce body state, so you
can *see* (not just imagine) what each control does on the wire.

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
  (`chg`); any frame the cockpit itself sent is **amber** (`tx`). In the
  middle of a live flood you spot the moment you flipped a switch instantly.
- **FREEZE.** Stops the live monitor so you can scroll back and click-copy a
  frame; the button flips to `LIVE >>` to resume. Dragging the scrollbar up
  also freezes into scrollback.

### Console / quick inject
The **CAN INJECT** bar parses `cansend`-style `ID#DATA` lines (e.g.
`400#FF3203FF`) and sends them onto the bus. Quick-inject buttons emit
`DRIVE_IN` frames. Anything you inject shows up amber in the monitor and, in
`--follower` mode, actually moves the car.

---

## 5. Talking to it from the command line

SLCAN server (byte-identical to the network adapter):

```bash
# nc 127.0.0.1 20102   then:
V   -> V1013
O   -> open the receive channel (frames start streaming)
C   -> close the receive channel
```

Real diagnostics with the sibling tooling's `client.py` client:

```bash
client.py --host 127.0.0.1 --port 20102 ping
client.py --host 127.0.0.1 --port 20102 dtc
client.py --host 127.0.0.1 --port 20102 vin
client.py --host 127.0.0.1 --port 20102 live 0C 0D 05
```

JSON control channel (`20103`), one command per line:

```json
{"t":"reset"}
{"t":"ignition","on":true}
{"t":"gear","gear":"D"}
{"t":"input","throttle":0.7,"brake":0.0,"steer":0.0}
{"t":"switch","name":"door_fl","on":true}
{"t":"fault","name":"mil","on":true}
{"t":"cruise","on":true}
```

The sim pushes a full state snapshot per tick (gear, rpm, speed, odo, lamps,
DTCs, faults, `frames`, `events`, `switches`).

---

## 6. Wiring to a real CAN bus (SocketCAN mode)

```bash
sudo ip link set can0 up type can bitrate 500000
python3 tools/carsim.py --iface can0 --follower       # ECUs + traffic + drive on the wire
python3 tools/carsim.py --iface can0 --tcp-also       # ...and keep TCP SLCAN too
```

The ECUs answer functional requests on `0x7DF` and stream `0x100–0x140`
exactly like bench mode. With `--follower`, a real steering wheel / pedal setup
(or `cansend can0 400#…`) drives the simulated car. See
`docs/WIRING_network_PROOF.md` for setup network wiring and network-proofing rules.

---

## 7. Full CLI reference

`tools/carsim.py`

| Option | Meaning |
|---|---|
| `--host ADDR` | bind address (default: all interfaces) |
| `--slcan-port N` | SLCAN-over-TCP port (default 20102) |
| `--ctrl-port N` | JSON control port (default 20103) |
| `--iface can0` | SocketCAN mode: ECUs + broadcasts on a real SocketCAN wire, TCP SLCAN off |
| `--tcp-also` | keep the TCP SLCAN server in `--iface` mode |
| `--follower` | ICSim-style drive: treat bus frames as remote drive input |
| `--no-traffic` | silent bench: no broadcast frames, only ECU replies |
| `--ecus 1\|2\|3` | 1 engine, 2 +TCM, 3 +ABS (default 3) |
| `--selftest` | headless physics/protocol assertions, then exit |
| `--version` | print version and exit |

---

## 8. Self-tests (headless, no display)

```bash
python3 -m py_compile tools/carsim.py tools/carsim_gui.py
python3 tools/carsim.py --selftest          # physics + OBD/UDS protocol
python3 tools/carsim_gui.py --check         # GUI decode/inject/controller helpers
python3 tests/cockpit_e2e.py                # JSON end-to-end (needs sim running)
python3 tests/ap_phase_e2e.py               # autopilot sequence (needs sim running)
```

Expected tails: `carsim selftest: ALL PASS`, `carsim_gui headless check:
ALL PASS`, `E2E ALL PASS`, `ap phase-machine e2e: ALL PASS`. `./run_tests.sh`
runs them all against a freshly-started sim.

---

## 9. Troubleshooting

- **GUI says "connecting…" forever** — the sim isn't running, or you pointed
  `--host` at the wrong address. Start the sim first, then the GUI.
- **Injected frames don't move the car** — the sim needs `--follower`
  (frames still appear in the monitor without it).
- **"controller: none connected"** — plug in / pair before starting the GUI, or
  hot-plug it (autodetect re-runs). No SDL → keyboard only.
- **Gears won't shift** — R/P are blocked above 8 km/h and you cannot crank in
  D. Brake, shift to P/N, then start.
- **Cruise won't engage** — it needs D **and** ≥ 40 km/h.
- **The monitor is a wall of text** — look for the **cyan** (just-changed)
  and **amber** (sent by the cockpit) lines, and hit **FREEZE** to read a
  specific frame.

---

## 10. Tests & docs

| File | Purpose |
|---|---|
| `docs/QUICKSTART.md` | full CLI / keybinding / controller / injection reference |
| `docs/WIRING_network_PROOF.md` | bench / network-setup / real-CAN wiring diagrams |
| `docs/TEST_EVIDENCE.md` | verification evidence and regression notes |
| `tests/cockpit_e2e.py` | JSON drive-cycle e2e (`CARSIM_HOST`/`CARSIM_PORT` overridable) |
| `tests/ap_phase_e2e.py` | autopilot 3-phase state-machine replay |

---

## Version highlights (v0.9.x cockpit line)

- **v0.9.3** — restored the DRIVE + KEYBOARD instruction rows and the
  Bus-broadcast legend in the footer (after they were accidentally removed);
  fixed the grey/blank window caused by `ttk.PanedWindow`.
- **v0.9.4** — added **body switches** (DOOR FL/FR/RL/RR, TRUNK, HOOD, BELT)
  that synthesize a 0x130 BODY TX frame on the bus.
- **v0.9.5** — CAN BUS monitor **highlight**: changed RX frames in **cyan**,
  cockpit-sent TX frames in **amber**.
- **v0.9.6** — monitor **readability**: `0x130` decoded by door name
  (`doors FL,FR` / `all closed`) and long frames **wrap** so nothing clips.

`carsim.py` is v0.3.3 (independent version line, unchanged by cockpit work).

MIT licensed.

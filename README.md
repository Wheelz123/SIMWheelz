# SIMWheelz — drivable virtual CAN simulator + cockpit

A virtual car that speaks **real OBD-II / UDS and CAN** over a loopback-only
JSON control channel, with a drivable game cockpit on top. In this virtual environment, you can simulate what a real attacker who gains access to a can bus could do, reverse engineer can frames, and learn about vehicle vulnerabilities in a fun to play game interface.  No vehicle or CAN hardware is required: the whole bus — engine, TCM and ABS ECUs plus the classic broadcast frames — lives in `tools/carsim.py`, and `tools/carsim_gui.py` is the dashboard you drive it from.
**One command starts the entire bench.** 
---

## 1. Quick start — one command

```bash
./run_cockpit.sh
```


---

## 2. Driving the cockpit

### Layout
The cockpit is a single window with three non-overlapping columns (road · tiled
3×3 gauge cluster · lamps / live data / CAN console), plus a footer of keyboard
instructions.

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
- **Names decoded.** `0x130` prints the doors by name instead of a bitmask
  number, e.g. `doors FL,FR` or `all closed`, plus `trunk/hood/belt`. You can
  tell exactly which door is open without decoding hex.
- **Change highlight.** The frame that *just* changed is shown in **cyan**
  (`chg`); any frame the cockpit itself sent is **yellow** (`tx`). In the middle
  of a live flood you spot the moment you flipped a switch instantly.

### Console / quick inject
To make a can injection, you can make the car do various things.The best way to do this to demonstrate a replay attack on the vehicle is to take the example of hitting the accelerator (the up arrow). You will see a yellow frame highlight. Then hit freeze on the can bus to stop the traffic and click the frame twice. Make sure it is a frame that starts with Tx, as these are frames that send can messages. It will populate that frame in the box after clicking twice. Once you have the frame in the box, hit enter. It will then accelerate the car and cause it to speed up. This can be done for all kinds of functions such as braking, turning the lights on, etc.

You can also hit record to capture an entire session. Can bus traffic will be recorded. Press Record -> perform actions on the vehicle ->  Stop -> Save -> Load - to upload the candump file -> Replay. This will cause all of the traffic that you have recorded to be replayed on the vehicle. This is a candump replay attack.

Have fun playing with the car! This is how a real life attacker would compromise a vehicle if they are able to gain a foothold on the canbus. 

---

```

---
| File | What it is |
|---|---|
| `tools/carsim.py` | Physics engine + ECU cluster (engine / TCM / ABS) + JSON control channel + loopback CAN INJECT + Sim-style `--follower` drive input |
| `tools/carsim_gui.py` | tkinter cockpit: canvas road, 3×3 gauge cluster, lamps + live data, CAN console + quick inject, CAN BUS monitor, keyboard driving, cruise + 3-phase autopilot, body switches, faults, units |
| `run_cockpit.sh` | Launcher for the cockpit — **the single entry point** (args pass through) |
| `run_tests.sh` | Full headless verification (compile + selftest + `--check` + both e2e) |
Requires **Python 3.10+** (tkinter for the GUI).


MIT licensed.

# SIMWheelz — drivable virtual CAN simulator + cockpit

<img width="1914" height="980" alt="image" src="https://github.com/user-attachments/assets/a5236f6b-cf88-4166-888d-de393ce290c5" />


A virtual car that speaks **real OBD-II / UDS and CAN** with a drivable game cockpit. In this virtual environment, you can simulate what a real attacker who gains access to a can bus could do, reverse engineer can frames, and learn about vehicle vulnerabilities in a fun to play game interface.  No vehicle or CAN hardware is required: the whole bus — engine, TCM, and ABS ECUs plus the classic broadcast frames — lives in `tools/carsim.py`, and `tools/carsim_gui.py` is the dashboard you drive it from.
**One command starts the entire bench.** 
---

## 1. Quick start — one command

```bash
./run_cockpit.sh
```
IMPORTANT: When you open the game, you must start the car like any vehicle in real life. That means the vehicle will be in park when the program starts. Press I to start the ignition. Then press D to put the car in drive. Follow the driving instructions on the footer of the window after this. Enjoy!

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

Injected `0x130` frames work exactly like the `0x120` lamp hack: the byte-0
bits latch the matching switches off the bus in **any** mode, so a single
`130#1000...` pops the trunk (`130#5000...` pops it with the belt still on)
and the sim re-broadcasts the state. Doors are bits `0x01`/`0x02`/`0x04`/`0x08`.


### LIN INJECT (body modules behind the BCM)

Real cars actuate body functions over different architectures, and this bench
models both -- the network selector in the inject box decides which one you
are talking to:

| architecture | where the command lives | sim path |
|---|---|---|
| naive-trust (the ICSim / 2015-Jeep model) | the CAN broadcast itself is trusted and actuates | `0x120`/`0x130` CAN INJECT latch (above) |
| LIN-slave (many modern cars) | actuator commands behind the BCM, off the CAN status frames | LIN INJECT (this section) |
| defended (modern premium cars) | rolling counters / CRC / SecOC reject naive replays | not modelled -- the wire accepts the frame, the receiver rejects it |

**How it works.**  A LIN frame is the *forged response* that won a master
poll slot, not a command from the master: LIN is master/slave, the BCM
polls and slaves answer.  The attack (demonstrated in Takahashi et al.,
*Automotive Attacks and Countermeasures on LIN-Bus*, IPSJ-JIP 25:220, 2017)
is a collision, not a broadcast:

1. the master polls a module (break + sync + PID);
2. the genuine slave starts answering with its TRUE state;
3. the attacker injects colliding bits mid-response;
4. the slave's bit-integrity check sees the overwrite, aborts, and waits
   for the next header;
5. the attacker finishes the slot with a forged response byte;
6. receivers accept it -- LIN response frames carry no authentication,
   only a checksum the attacker recomputes.

**Use it:** switch the inject box to **LIN** and send a mnemonic command or a
raw module frame:

```
LIN_TRUNK_OPEN      LIN_WIPER_ON       LIN_LIGHT_ON        LIN_HAZARD_ON
LIN_HIGHBEAM_ON     LIN_LIGHTS_FULL    LIN_20#03           LIN_30#01
```

The frame ID is the module address; the payload byte is the module's
**complete output state**:

| id | module | bits |
|---|---|---|
| `0x10` | wiper | `0x01` wipers |
| `0x20` | light | `0x01` headlights, `0x02` highbeam |
| `0x21` | turn | `0x01` left, `0x02` right, `0x04` hazard |
| `0x30` | liftgate | `0x01` trunk |
| `0x40` | door | `0x01` FL, `0x02` FR, `0x04` RL, `0x08` RR |
| `0x41` | hood | `0x01` hood, `0x02` belt |

**Whole-byte semantics:** the forged response replaces the module's whole
output state, not a single bit -- `LIN_HIGHBEAM_ON` (`0x20#02`) also clears
the low beam, and `0x20#03` is headlights + highbeam.  The BCM notices the
change and the CAN status frames (`0x120` lamps, `0x130` body) re-encode it,
exactly as a real cluster would.

Which architecture a given car uses varies by make/model/year: on many
vehicles these functions are genuine CAN command frames (central locking
everywhere; the 2015 Jeep wipers/headlights research; Car Hacking Village and
CTF demos), on others they are LIN slaves or BCM-internal, and on modern
premium cars rolling counters / CRC / SecOC block naive replays.  The bench
lets you practice both paths; on a real bus the first job is classifying
which one you are looking at.

**Capture:** tick the **LIN view** checkbox in the CAN BUS monitor and
the window shows the LIN capture ring instead of the CAN stream.  Every
`RX` record is a genuine slave answering its poll slot (`LIN_20#03  RX`)
-- including the answer a collision aborts; the attacker's forged response
appears as `TX` (`LIN_30#01  TX`), so the ring shows the kill-and-replace
in order: true answer killed, forged answer accepted.  Each line is raw
frame syntax, so copy it straight into the **LIN** inject box.

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

To begin playing with this console, you have to start it like you would start a real car. First you have to start up the vehicle in park by pressing I. This will start the ignition. Then you must put the vehicle in drive by pressing D. Then you can start playing with various functions in the vehicle outlined in the instructions section on the console. For example, the gas pedal is the up arrow. The brake pedal is the down arrow, and so on. 

To make a can injection, you can make the car do various things. The best way to do this to demonstrate a replay attack on the vehicle is to take the example of hitting the accelerator (the up arrow). You will see a yellow frame highlight. Then hit freeze on the can bus to stop the traffic and click the frame twice. It will populate that frame in the box after clicking twice. Once you have the frame in the box, hit enter. It will then accelerate the car and cause it to speed up. This can be done for all kinds of functions such as braking, turning the lights on, etc. This is how a hacker would inject traffic onto the bus. 

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

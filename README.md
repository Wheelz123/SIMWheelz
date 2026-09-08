# carsim — drivable CAN simulator ("adapter server" + cockpit)

A virtual car that speaks **real OBD-II / UDS over SLCAN-over-TCP**, exactly
like the network CAN adapter, with a drivable game cockpit on top. Use it to
develop and test the remote-CAN monitor tooling (client, GUI, can-utils)
without touching a vehicle — then put the same sim on a real SocketCAN wire and
let `cansend` drive it.

```
tools/carsim.py      physics engine + ECU cluster (engine/TCM/ABS) + servers
tools/carsim_gui.py  tkinter cockpit: road, 3x3 gauge cluster, lamps, CAN
                     console, keyboard + controller (USB), cruise, autopilot
```

## Quick start

```bash
# 1) run the sim (add --follower so injected frames drive the car)
python3 tools/carsim.py --follower

# 2) drive it
python3 tools/carsim_gui.py
```

Self-test everything headless:

```bash
./run_tests.sh
```

## Feature highlights

- **Physics + ECUs**: engine (0x7E8), TCM, ABS; functional diag 0x7DF;
  real-car start order (P/N → crank ~0.7 s → idle), auto gearbox, cruise,
  faults (MIL P0300 / overheat P0217 / flat tyre) with OBD and UDS
  clear paths; J1979 PIDs, mode 03/04 DTCs, mode 09 VIN multi-frame,
  ISO-TP SF/FF/FC.
- **Two TCP planes** (byte-compatible with the adapter):
  - `20102` SLCAN-over-TCP — Lawicel server: `V`, `O`, `C`, broadcasts +
    ECU replies, `cansend`-style injection.
  - `20103` JSON control — inputs/gear/ignition/faults/cruise/reset + full
    state snapshot every tick.
- **ICSim-style drive**: `--follower` folds bus frames 0x100/0x110/0x120/
  0x140/0x400 into the physics, from a SocketCAN wire (`--iface can0`) or the
  TCP server. `cansend can0 400#FF3203FF` = floor it.
- **Cockpit GUI**: three non-overlapping columns (road / tiled gauge cluster /
  lamps + live data + CAN console), keyboard **and** controller controller over USB
  or radio (optional SDL, autodetect, keyboard fallback), km/h ↔ mph.
- **SocketCAN / SocketCAN mode**: `--iface can0` puts ECUs and traffic on a
  real wire (see docs/WIRING_network_PROOF.md).

## Tests & docs

| File | Purpose |
|---|---|
| `docs/QUICKSTART.md` | full CLI/keybinding/controller/injection reference |
| `docs/WIRING_network_PROOF.md` | bench / network-setup / real-CAN wiring diagrams |
| `docs/TEST_EVIDENCE.md` | verification evidence and regression notes |
| `tests/cockpit_e2e.py` | JSON drive-cycle e2e (`CARSIM_HOST`/`CARSIM_PORT` overridable) |
| `tests/ap_phase_e2e.py` | autopilot 3-phase state-machine replay |

Requires Python 3.10+ (tkinter for the GUI, SDL optional for the
controller). MIT licensed.

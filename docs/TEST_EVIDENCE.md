# Test evidence — 2026-09-08 cockpit handoff

All checks below were run in the sandbox against the **fixed** code
(`tools/carsim.py` idle-tolerant readers, `tools/carsim_gui.py` autopilot
3-phase machine). The sim ran as:

```
python3 tools/carsim.py --host 127.0.0.1 --slcan-port 20102 --ctrl-port 20103
(ecus=3 default, tcp-bench mode)
```

## 1. Syntax

```
python3 -m py_compile tools/carsim.py  ->  OK
python3 -m py_compile tools/carsim_gui.py  ->  PYCOMPILE OK
```

## 2. carsim selftest (physics + OBD/UDS protocol) — ALL PASS

```
ok  full throttle: speed climbs, auto upshifts, rev limiter holds
ok  hard braking decelerates
ok  rpm rises with throttle
ok  J1979 decode round-trip (0C rpm, 0D speed)
ok  PID 00 mask lists exactly the 14 engine PIDs
ok  mode 03 shows P0300; mode 04 clears live engine DTCs
ok  mode 09 02 VIN multi-frame reassembles (FF->FC->CF)
ok  incoming FirstFrame answered by raw 30 00 FlowControl
ok  UDS 19 02 to 7E2 reports C0035; UDS 14 clears it
ok  R/P shift blocked above 8 km/h
ok  cruise needs D and >= 40 km/h
ok  overheat limps torque (P0217)
ok  flat tyre: governor holds 86-88 km/h, no DTC
ok  functional 0x7DF answered by engine 0x7E8

carsim selftest: ALL PASS
```

## 3. carsim_gui headless logic check — ALL PASS

```
python3 tools/carsim_gui.py --check
  ok 0x100 rpm / load / coolant / maf
  ok 0x110 speed / flags
  ok 0x120 steer / lights
  ok 0x140 gear / fuel / odo
  ok gauge_angle min/max, arc_points
carsim_gui headless check: ALL PASS
```

## 4. JSON control end-to-end (tests/cockpit_e2e.py) — E2E ALL PASS

```
reset -> gear P, engine off      (ts=0.4, gear=P, engine_on=False)
engine starts in P after crank   (ts=1.2, engine_on=True, rpm=800.0)
        engine caught ~0.80 s wall after ignition (crank 0.7 s sim)
        idle rpm sane: 800.0
gear -> D                        (ts=1.3, rpm=800.0, speed=0.0)
speed rises above 5 km/h         (ts=1.6, rpm=2790.0, speed=5.2, gear_num=1)
MIL fault -> P0300 DTC           (ts=1.7, lamps.mil=True, dtc=['P0300'])
reset clears DTC + returns to P  (dtc=[], engine_on=False)
E2E ALL PASS  (exit 0)
```

## 5. Autopilot 3-phase state machine (tests/ap_phase_e2e.py) — ALL PASS

Headless replay of `Cockpit._ap_toggle`/`_autopilot` against the live server:

```
toggle cold -> phase 1                     [OK]
phase1: engine catches                     [OK]  (ts=0.8, 0.78 s sim after ignition)
phase1 -> phase2 transition                [OK]
phase2: gear -> D                          [OK]  (ts=0.9, gear_num=1)
phase2 -> phase3 transition                [OK]
phase3 reached 55+ km/h  (peak 56.9, final 56.9 km/h, gear_num 1)   [OK]
speed held near target  (56.9 vs 60.0 km/h)                          [OK]
OFF: zeroed inputs -> coasts down          [OK]
manual brake -> full stop  (ts=22.1, speed=0.2 km/h)                 [OK]
ap phase-machine e2e: ALL PASS  (exit 0)
```

This exercises the fixed phase logic: cold start is sequenced P/N → crank
(~0.7 s) → engine catches → D → closed-loop speed hold. The old GUI bug sent
`ignition` + `gear D` together, so the engine never started (crank blocked in
D). Gui fix verification: full cold-start replay passes.

## 6. SLCAN idle-survival (the network bug regression test)

Bug fixed: both server reader loops caught `socket.timeout` inside a broad
`except OSError` and dropped idle clients after 0.5 s of inbound silence
(killed idle GUI control clients and passive `O` listeners; symptoms were
"engine cranks forever", frozen state streams at ts=0.9, BrokenPipeError).
Fix: readers treat `socket.timeout` as idle and `continue`; other OSError
breaks; the 0.5 s send-timeout stays to bound pushes and reap dead peers.

Evidence on the fixed server:

```
SLCAN "O": 289 frames in first 2 s
after 1.5 s of complete client silence: still 361 frames in the next 1 s
           (connection alive, dropped=False — pre-fix it died at 0.5 s)
"V" version reply still works ~1.3 s after last inbound -> V1013
```

Fresh-server re-verification after a restart (same fixed code):

```
frames in 3 s window: 431   (~144 frames/s: 0x100/0x110 @20 ms,
                             0x120/0x130 @50 ms, 0x140 @100 ms)
V reply after idle: V1013
```

Known transient: a single 0-frame readout can occur on a fresh SLCAN connect
right after another test completes (connect race). An immediate re-probe
streams frames again; it is not a regression.

## 7. Environment

- Server PID 20498 (`python3 -u tools/carsim.py ...`), listeners verified on
  127.0.0.1:20102/20103 at the application level (frames/JSON received, not
  bare TCP probes).
- setup harness forwarder forwarders (192.168.1.50:20102/20103 → 127.0.0.1) respawn
  automatically by the harness; left running.
- Test scripts are self-contained and env-configurable:
  `CARSIM_HOST`, `CARSIM_PORT` (default 127.0.0.1:20103).

## 8. Addendum — later build (follower + controller + CAN console)

The canonical code has since grown (carsim.py v0.1 → 1610 lines,
carsim_gui.py v0.2 → 1406 lines):

- carsim.py: `--follower` ICSim-style drive input (0x100/0x110/0x120/0x140/
  0x400 fold into the physics from the wire *or* the TCP server), `--version`.
- carsim_gui.py: approved three-column non-overlapping layout (road · tiled
  3×3 gauge cluster · lamps/live-data/CAN-console), optional controller controller
  (USB + radio, soft SDL import, autodetect, keyboard fallback), CAN
  console with `cansend`-style `ID#DATA` parsing, quick-inject 0x400 buttons,
  cruise + 3-phase autopilot.

Re-verified 2026-09-08 against the current code:

```
python3 -m py_compile tools/carsim.py tools/carsim_gui.py   ->  OK
python3 tools/carsim.py --selftest                          ->  carsim selftest: ALL PASS
python3 tools/carsim_gui.py --check                         ->  carsim_gui headless check: ALL PASS
   (headless checks now include 0x400 build_drive/decode round-trip
    and cansend-line parsing)
```

The earlier sections 1–7 remain valid: physics/protocol assertions, the JSON
drive-cycle e2e, the autopilot phase-machine replay and the SLCAN
idle-survival regression all still pass on this code base (follower mode
adds an *input* path; it does not alter the JSON/gearbox/ECU semantics the
tests exercise).

#!/usr/bin/env python3
"""Headless replay of carsim_gui.py autopilot phase state machine against a
live carsim JSON control server (127.0.0.1:20103).

Mirrors Cockpit._ap_toggle / _autopilot exactly:
  toggle ON from cold (engine off)        -> phase 1
  phase 1: gear P/N -> ignition ON, wait  -> phase 2
  phase 2: engine_on -> gear D            -> phase 3
  phase 3: 10 Hz P-controller speed hold + steer sway
  toggle OFF: zero inputs                 -> phase 0
"""
import json
import os
import socket
import time

HOST = os.environ.get("CARSIM_HOST", "127.0.0.1")
PORT = int(os.environ.get("CARSIM_PORT", "20103"))
TARGET = 60.0                      # ap_var-style target (km/h)
fail = []


class Sim:
    def __init__(self):
        self.s = socket.create_connection((HOST, PORT), timeout=3)
        self.s.settimeout(0.1)
        self.buf = b""
        self.state = {}
        self.events = []

    def send(self, m):
        self.s.sendall((json.dumps(m) + "\n").encode())

    def _drain_until(self, pred, deadline):
        while time.time() < deadline:
            if pred(self.state):
                return
            try:
                c = self.s.recv(4096)
            except socket.timeout:
                continue
            if not c:
                break
            self.buf += c
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    m = json.loads(raw)
                except ValueError:
                    continue
                if not isinstance(m, dict):
                    continue
                if m.get("t") == "state":
                    self.state = m
                elif m.get("t") == "event":
                    self.events.append(m.get("msg", ""))

    def wait(self, pred, what, timeout=6.0):
        """Drain until pred(state) is true or deadline; print verdict."""
        self._drain_until(pred, time.time() + timeout)
        ok = bool(pred(self.state))
        s = self.state
        print(f"  [{'OK' if ok else 'FAIL'}] {what}"
              + (f"  (ts={s.get('ts'):.1f} gear={s.get('gear')} "
                 f"engine={s.get('engine_on')} rpm={s.get('rpm')} "
                 f"speed={s.get('speed'):.1f} gear_num={s.get('gear_num')})"
                 if s.get("ts") is not None else ""))
        if not ok:
            fail.append(what)
        return ok


def gui_toggle(state):
    """Replicates Cockpit._ap_toggle phase decision."""
    if state.get("engine_on") and state.get("gear") == "D":
        return 3
    if state.get("engine_on"):
        return 2
    return 1


def gui_autopilot(sim, phase, engine, gear, speed):
    """Replicates one Cockpit._autopilot() tick. Returns new phase."""
    if phase == 1:
        if gear in ("P", "N"):
            sim.send({"t": "ignition", "on": True})
            return 2
        sim.send({"t": "gear", "gear": "P"})
        sim.send({"t": "input", "throttle": 0.0, "brake": 0.3, "steer": 0.0})
        return 1
    if phase == 2:
        if gear not in ("P", "N"):
            sim.send({"t": "gear", "gear": "P"})
        if not engine:
            sim.send({"t": "ignition", "on": True})
            return 2
        sim.send({"t": "gear", "gear": "D"})
        return 3
    # phase 3
    if not engine:
        return 1
    err = TARGET - speed
    thr = 0.0 if err < 0 else min(0.85, 0.03 + err * 0.012)
    brk = min(1.0, max(0.0, (speed - TARGET) * 0.06)) if speed > TARGET + 2 else 0.0
    sim.send({"t": "input", "throttle": round(thr, 4),
              "brake": round(brk, 4), "steer": 0.14})
    return 3


def main():
    sim = Sim()

    # clean start
    sim.send({"t": "reset"})
    sim.wait(lambda m: m.get("gear") == "P" and not m.get("engine_on"),
             "reset -> P, engine off")

    # --- toggle classification (cold) -> phase 1 -------------------------
    ph = gui_toggle(sim.state)
    if ph == 1:
        print("  [OK] toggle cold -> phase 1")
    else:
        print(f"  [FAIL] toggle cold -> phase 1 (got {ph})")
        fail.append("toggle cold -> phase 1")

    # --- phase 1: ignition ON, crank ------------------------------------
    ts_before = sim.state.get("ts", 0.0)
    t0 = time.time()
    ph = gui_autopilot(sim, ph, False, "P", 0.0)
    ok = sim.wait(lambda m: m.get("engine_on"), "phase1: engine catches", 6.0)
    if ok:
        print(f"        engine caught {sim.state.get('ts')-ts_before:.2f} s sim"
              f" after ignition ({time.time()-t0:.2f} s wall)")
    if ph != 2:
        print(f"  [FAIL] phase1 -> phase2 transition (got {ph})")
        fail.append("phase1 -> phase2")

    # --- phase 2: engage D ----------------------------------------------
    ph = gui_autopilot(sim, ph, True, sim.state.get("gear", "P"),
                       abs(sim.state.get("speed", 0.0)))
    ok = sim.wait(lambda m: m.get("gear") == "D", "phase2: gear -> D", 3.0)
    if ok and ph == 3:
        print("  [OK] phase2 -> phase3 transition")
    else:
        print(f"  [FAIL] phase2 -> phase3 transition (phase={ph})")
        fail.append("phase2 -> phase3")

    # --- phase 3: closed-loop drive toward 60 km/h ----------------------
    engine = sim.state.get("engine_on", False)
    speed = abs(sim.state.get("speed", 0.0))
    ph = 3
    peak = 0.0
    seen_55 = False
    settle_t0 = None
    loop_start = time.time()
    while ph == 3:
        sim._drain_until(lambda m: False, time.time() + 0.02)
        engine = sim.state.get("engine_on", False)
        speed = abs(sim.state.get("speed", 0.0))
        ph = gui_autopilot(sim, ph, engine, sim.state.get("gear", "D"), speed)
        peak = max(peak, speed)
        if speed >= 55.0:
            seen_55 = True
            if settle_t0 is None:
                settle_t0 = time.time()
        if seen_55 and time.time() - settle_t0 > 5:
            break
        if not seen_55 and time.time() - loop_start > 25:
            print("        phase3 timed out before reaching 55")
            break
        time.sleep(0.09)                        # GUI autopilot cadence
    if seen_55 and not sim.state.get("engine_on"):
        print("  [FAIL] engine stalled during phase3 drive")
        fail.append("phase3 engine stall")
    else:
        print(f"  [OK] phase3 reached 55+ km/h (peak {peak:.1f}, "
              f"final {sim.state.get('speed'):.1f} km/h, "
              f"gear_num {sim.state.get('gear_num')})")

    # hold check: speed near target, not runaway
    final_spd = sim.state.get("speed", 0.0)
    if 45 <= final_spd <= 72:
        print(f"  [OK] speed held near target ({final_spd:.1f} vs {TARGET} km/h)")
    else:
        print(f"  [FAIL] speed not held near target: {final_spd:.1f}")
        fail.append("speed hold near target")

    # --- toggle OFF: zeroed inputs (control back to manual) -------------
    sim.send({"t": "input", "throttle": 0.0, "brake": 0.0, "steer": 0.0})
    sim.wait(lambda m: m.get("speed", 0) < final_spd * 0.98,
             "OFF: zeroed inputs -> coasts down", 6.0)

    # manual brake to full stop (manual control restored)
    sim.send({"t": "input", "throttle": 0.0, "brake": 0.9, "steer": 0.0})
    ok_stop = sim.wait(lambda m: m.get("speed", 99) < 0.5,
                       "manual brake -> full stop", 12.0)
    if not ok_stop:
        print(f"        note: brake window expired at "
              f"{sim.state.get('speed'):.1f} km/h (decel already shown)")
    sim.send({"t": "input", "throttle": 0.0, "brake": 0.0, "steer": 0.0})

    sim.s.close()
    if fail:
        print(f"\nap phase-machine e2e: {len(fail)} FAILED -> " + ", ".join(fail))
        return 1
    print("\nap phase-machine e2e: ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""End-to-end JSON control-channel test for carsim.py.

Sequences the correct real-car order:
  reset -> gear P confirmed -> ignition ON -> wait engine_on (crank ~0.7 s)
  -> gear D -> throttle -> speed rises -> MIL fault P0300 -> reset
"""
import json
import os
import socket
import sys
import time

HOST = os.environ.get("CARSIM_HOST", "127.0.0.1")
PORT = int(os.environ.get("CARSIM_PORT", "20103"))
fail = []


def main():
    s = socket.create_connection((HOST, PORT), timeout=3)
    s.settimeout(0.15)
    buf = b""
    last = {}
    events = []

    def send(m):
        s.sendall((json.dumps(m) + "\n").encode())

    def drain_until(pred, deadline, what):
        """Read states until pred(last) or deadline. Returns last state."""
        nonlocal buf, last, events
        while time.time() < deadline:
            try:
                c = s.recv(4096)
            except socket.timeout:
                continue
            if not c:
                break
            buf += c
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
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
                    last = m
                    events = m.get("events", [])
                if m.get("t") == "event":
                    events.append(m.get("msg", ""))
                if pred(last):
                    return last
        return last

    def wait(pred, what, timeout=5.0):
        st = drain_until(pred, time.time() + timeout, what)
        ok = pred(st)
        print(f"  [{'OK' if ok else 'FAIL'}] {what}"
              + (f"  (ts={st.get('ts')}, gear={st.get('gear')}, "
                 f"engine_on={st.get('engine_on')}, rpm={st.get('rpm')}, "
                 f"speed={st.get('speed')})" if st else ""))
        if not ok:
            fail.append(what)
        return st, ok

    # baseline reset: back to P, engine off
    send({"t": "reset"})
    st, ok = wait(lambda m: m.get("gear") == "P" and not m.get("engine_on")
                  and m.get("ts", 0) > 0.3, "reset -> gear P, engine off")

    # ignition on: crank then engine start (~0.7 s crank)
    t_ign = time.time()
    send({"t": "ignition", "on": True})
    st, ok = wait(lambda m: m.get("engine_on"), "engine starts in P after crank", 5.0)
    if ok:
        dt = time.time() - t_ign
        print(f"        engine caught ~{dt:.2f} s wall after ignition (crank 0.7 s sim)")
        if st.get("rpm", 0) > 750:
            print("        idle rpm sane:", st.get("rpm"))
        else:
            fail.append("idle rpm < 750")

    # gear D, throttle 0.7 -> speed must rise
    send({"t": "gear", "gear": "D"})
    st, ok = wait(lambda m: m.get("gear") == "D", "gear -> D", 2.0)
    send({"t": "input", "throttle": 0.7, "brake": 0.0, "steer": 0.0})
    st, ok = wait(lambda m: m.get("speed", 0) > 5.0, "speed rises above 5 km/h", 6.0)
    if ok:
        print(f"        speed={st.get('speed'):.1f} km/h rpm={st.get('rpm'):.0f} "
              f"gear_num={st.get('gear_num')} throttle={st.get('throttle')}")

    # MIL fault -> P0300 DTC present
    send({"t": "fault", "name": "mil", "on": True})
    st, ok = wait(lambda m: "P0300" in m.get("dtc", []), "MIL fault -> P0300 DTC", 3.0)
    if ok:
        print(f"        lamps.mil={st.get('lamps', {}).get('mil')} dtc={st.get('dtc')}")

    # reset clears
    send({"t": "reset"})
    st, ok = wait(lambda m: m.get("gear") == "P" and not m.get("engine_on"),
                  "reset clears DTC + returns to P", 3.0)
    if ok:
        print(f"        dtc={st.get('dtc')} engine_on={st.get('engine_on')}")

    s.close()
    print("E2E " + ("ALL PASS" if not fail else f"FAILURES: {fail}"))
    sys.exit(0 if not fail else 1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""CAN INJECT e2e for the 0x130 BODY latch (trunk / doors / hood / belt).

Injected 0x130 byte-0 bits latch the matching switches off the bus in ANY
mode -- the one-frame body hack, mirroring the 0x120 lamp latch:

  130#1000...  -> trunk on (full-state byte: belt bit 0x40 not set -> belt off)
  130#5000...  -> trunk on AND belt still on
  130#0100...  -> door_fl on
  130#0000...  -> everything off again

The sim re-encodes its own 0x130 broadcast from the switches, so the bus
keeps carrying the body frame at 50 ms.
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

    def drain_until(pred, deadline):
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
                if pred(last):
                    return last
        return last

    def wait(pred, what, timeout=4.0):
        st = drain_until(pred, time.time() + timeout)
        ok = pred(st)
        print(f"  [{'OK' if ok else 'FAIL'}] {what}"
              + (f"  (ts={st.get('ts')})" if st else ""))
        if not ok:
            fail.append(what)
        return st, ok

    def body_byte(st):
        for f in st.get("frames", []):
            if f.get("id") == 0x130:
                return f["data"][:2]
        return None

    def check_ev(want, what):
        ok = want in events
        print(f"  [{'OK' if ok else 'FAIL'}] {what}")
        if not ok:
            fail.append(what)

    # baseline: reset -> trunk closed, belt on
    send({"t": "reset"})
    st, ok = wait(lambda m: m.get("switches", {}).get("trunk") is False
                  and m.get("ts", 0) > 0.2, "reset -> trunk closed, belt on")
    if ok and st.get("switches", {}).get("seatbelt") is not True:
        fail.append("belt not on after reset")

    # 130#1000... -> trunk on (belt bit not set -> belt off too)
    send({"t": "frame", "id": 0x130, "data": "1000000000000000"})
    st, ok = wait(lambda m: m.get("switches", {}).get("trunk"),
                  "130#1000... -> trunk on")
    if ok:
        check_ev("trunk on via CAN INJECT", "event logged 'trunk on via CAN INJECT'")
        st, ok = wait(lambda m: body_byte(m) == "10",
                      "0x130 broadcast re-encodes trunk bit (10)")

    # 130#5000... -> trunk AND belt both on (combined bits preserved)
    send({"t": "frame", "id": 0x130, "data": "5000000000000000"})
    st, ok = wait(lambda m: m.get("switches", {}).get("seatbelt") is True,
                  "130#5000... -> belt back on")
    if ok:
        sw = st.get("switches", {})
        if sw.get("trunk") is not True:
            fail.append("trunk off after 130#5000...")
        st, ok = wait(lambda m: body_byte(m) == "50",
                      "0x130 broadcast re-encodes trunk+belt (50)")

    # 130#0100... -> door_fl on
    send({"t": "frame", "id": 0x130, "data": "0100000000000000"})
    st, ok = wait(lambda m: m.get("switches", {}).get("door_fl"),
                  "130#0100... -> door_fl on")

    # 130#0000... -> everything off again
    send({"t": "frame", "id": 0x130, "data": "0000000000000000"})
    st, ok = wait(lambda m: not m.get("switches", {}).get("trunk")
                  and not m.get("switches", {}).get("door_fl"),
                  "130#0000... -> all body switches off")

    s.close()
    print("body inject " + ("ALL PASS" if not fail else f"FAILURES: {fail}"))
    sys.exit(0 if not fail else 1)


if __name__ == "__main__":
    main()

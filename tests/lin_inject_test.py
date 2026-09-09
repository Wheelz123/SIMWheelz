#!/usr/bin/env python3
"""LIN INJECT e2e for body modules behind the BCM -- the second real-world
architecture (modern LIN-slave cars), alongside the 0x120/0x130 CAN latch
(naive-trust vehicles like the ICSim / 2015-Jeep model).

A LIN frame is the attacker's *forged response* that won a master poll
slot: the BCM master polls a module, the genuine slave answers, the
attacker collides with that answer (the slave's error handling aborts it)
and finishes the slot with the forged byte -- which the receiver accepts
because LIN responses carry no authentication (Takahashi et al., IPSJ-JIP
25:220, 2017).  The payload byte is the whole module output state, so
0x20#02 (highbeam) also clears 0x01 (headlights).  The BCM notices and the
CAN status frames re-encode:

  {"t":"lin","cmd":"LIN_TRUNK_OPEN"}  -> trunk on, 0x130 re-encodes 50 (belt latched at reset)
  {"t":"lin","id":0x20,"data":"02"}   -> highbeam on, headlights OFF,
                                         0x120 lamp byte becomes 08

The CAN latch keeps working too -- both paths share the switches.
The capture ring works too: the genuine slave's aborted answer is logged
as an RX record, the forged response as TX (kill-and-replace, in order),
and the sim's 10 Hz LIN master poll logs RX records whenever a module's
output state changes -- so the recorded lines (LIN_XX#dd TX/RX) paste
straight into the LIN INJECT box.
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
    seen_events = []   # union of all events observed across the session

    def send(m):
        s.sendall((json.dumps(m) + "\n").encode())

    def drain_until(pred, deadline):
        nonlocal buf, last, events, seen_events
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
                    seen_events.extend(events)
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

    def lamp_byte(st):
        for f in st.get("frames", []):
            if f.get("id") == 0x120:
                return f["data"][2:4]      # byte 1 = light bits (2 hex chars)
        return None

    def check_ev(want, what):
        ok = want in seen_events
        print(f"  [{'OK' if ok else 'FAIL'}] {what}")
        if not ok:
            fail.append(what)

    def sw(st, name):
        return st.get("switches", {}).get(name)

    def lin_idx(st, lin_id, data, direction):
        for i, r in enumerate(st.get("lin_frames", [])):
            if r.get("id") == lin_id and r.get("data") == data \
               and r.get("dir") == direction:
                return i
        return -1

    # baseline
    send({"t": "reset"})
    st, ok = wait(lambda m: sw(m, "trunk") is False and m.get("ts", 0) > 0.2,
                  "reset -> trunk closed, belt on")

    # mnemonic: LIN_TRUNK_OPEN -> trunk on via LIN, 0x130 re-encodes 50 (belt held)
    send({"t": "lin", "cmd": "LIN_TRUNK_OPEN"})
    st, ok = wait(lambda m: sw(m, "trunk"),
                  "LIN_TRUNK_OPEN -> trunk on")
    if ok:
        check_ev("trunk on via LIN INJECT", "event 'trunk on via LIN INJECT'")
        st, ok = wait(lambda m: body_byte(m) == "50",
                      "0x130 re-encodes trunk + seatbelt (50; belt latched at reset)")
        st, ok = wait(lambda m: any(
            r.get("dir") == "TX" and r.get("id") == 0x30
            and r.get("data") == "01"
            for r in m.get("lin_frames", [])),
            "LIN capture: injected LIN_30#01 logged as TX")
        # Collision event: kill-and-replace documented in the event stream.
        # The genuine module state was 00 (trunk closed); the forged byte
        # was 01 (trunk open).
        collision_ev = ("LIN collision: module 0x30 genuine 00 response "
                        "aborted -> false 01 accepted")
        check_ev(collision_ev,
                 "collision event documents the kill-and-replace sequence")
        st, ok = wait(lambda m: lin_idx(m, 0x30, "00", "RX") >= 0
                      and lin_idx(m, 0x30, "01", "TX") >= 0
                      and lin_idx(m, 0x30, "00", "RX")
                          < lin_idx(m, 0x30, "01", "TX"),
                      "ring order: genuine 30#00 aborted (RX) BEFORE forged 30#01 (TX)")

    # mnemonic: LIN_TRUNK_CLOSE -> trunk off (belt stays on -> 40)
    send({"t": "lin", "cmd": "LIN_TRUNK_CLOSE"})
    st, ok = wait(lambda m: not sw(m, "trunk"),
                  "LIN_TRUNK_CLOSE -> trunk off")
    if ok:
        st, ok = wait(lambda m: body_byte(m) == "40",
                      "0x130 re-encodes belt-only (40)")

    # raw module frame: light module 0x20#01 -> headlights on
    send({"t": "lin", "id": 0x20, "data": "01"})
    st, ok = wait(lambda m: sw(m, "headlights"),
                  "LIN_20#01 -> headlights on")
    if ok:
        check_ev("headlights on via LIN INJECT",
                 "event 'headlights on via LIN INJECT'")
        st, ok = wait(lambda m: lamp_byte(m) == "10",
                      "0x120 lamp byte re-encodes headlights (10)")

    # WHOLE-BYTE teaching point: 0x20#02 -> highbeam on AND headlights off
    send({"t": "lin", "id": 0x20, "data": "02"})
    st, ok = wait(lambda m: sw(m, "highbeam") and not sw(m, "headlights"),
                  "LIN_20#02 -> highbeam on, headlights OFF (whole byte)")
    if ok:
        st, ok = wait(lambda m: lamp_byte(m) == "08",
                      "0x120 lamp byte re-encodes highbeam only (08)")

    # mnemonic: LIN_HAZARD_ON -> hazard switch latched
    send({"t": "lin", "cmd": "LIN_HAZARD_ON"})
    st, ok = wait(lambda m: sw(m, "hazard"),
                  "LIN_HAZARD_ON -> hazard on")
    if ok:
        check_ev("hazard on via LIN INJECT", "event 'hazard on via LIN INJECT'")

    # raw module frame: door module 0x40#01 -> door_fl on -> 0x130 41 (belt on)
    send({"t": "lin", "id": 0x40, "data": "01"})
    st, ok = wait(lambda m: sw(m, "door_fl"),
                  "LIN_40#01 -> door_fl on")
    if ok:
        st, ok = wait(lambda m: body_byte(m) == "41",
                      "0x130 re-encodes door_fl + belt (41)")

    # CAN-latch regression: 0x130 CAN INJECT still pops the trunk (both paths)
    send({"t": "frame", "id": 0x130, "data": "1000000000000000"})
    st, ok = wait(lambda m: sw(m, "trunk"),
                  "regression: 130#1000... still pops trunk via CAN")
    if ok:
        check_ev("trunk on via CAN INJECT", "event 'trunk on via CAN INJECT'")
        st, ok = wait(lambda m: any(
            r.get("dir") == "RX" and r.get("id") == 0x30
            and r.get("data") == "01"
            for r in m.get("lin_frames", [])),
            "LIN capture: 10 Hz master poll logged LIN_30#01 as RX")

    # negative: unknown module id is ignored, state untouched
    send({"t": "lin", "id": 0x99, "data": "01"})
    time.sleep(0.4)
    st = drain_until(lambda m: False, time.time() + 0.3)
    if sw(st, "trunk") is not True:
        fail.append("unknown LIN module changed state")
    print("  [OK] LIN_99#01 (unknown module) ignored - state untouched")

    # cleanup: CAN 130#1000 latch cleared the 0x40 belt (whole-byte write) -> restore via LIN
    send({"t": "lin", "cmd": "LIN_TRUNK_CLOSE"})
    send({"t": "lin", "cmd": "LIN_DOORS_OFF"})
    st, ok = wait(lambda m: not sw(m, "trunk") and not sw(m, "door_fl"),
                  "LIN_TRUNK_CLOSE + LIN_DOORS_OFF -> all closed")
    if ok:
        send({"t": "lin", "cmd": "LIN_BELT_ON"})
        st, ok = wait(lambda m: body_byte(m) == "40",
                      "LIN_BELT_ON -> 0x130 back to belt-only (40)")

    s.close()
    print("lin inject " + ("ALL PASS" if not fail else f"FAILURES: {fail}"))
    sys.exit(0 if not fail else 1)


if __name__ == "__main__":
    main()

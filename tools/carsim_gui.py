#!/usr/bin/env python3
"""
carsim_gui.py — the cockpit for carsim.py (the "driver" side of the sim).

The simulator (tools/carsim.py) is the ECU cluster + physics engine.  It
publishes everything over a JSON control channel (default 127.0.0.1:20103):

    TX {"t":"state", ...physics, lamps, faults, switches, frames, events}
    RX {"t":"input","throttle":0.0,"brake":0.0,"steer":0.0}
    RX {"t":"gear","gear":"D"}  {"t":"ignition","on":true}
    RX {"t":"switch","name":"hazard","on":true}   {"t":"fault","name":"mil","on":true}
    RX {"t":"cruise","on":true}  {"t":"cruise","delta":5}   {"t":"reset"}

The simulator is loopback-only: raw CAN frames are injected over the same
JSON control channel, so the CAN CONSOLE below sends "cansend-style"
frames as {"t":"frame","id":...} messages straight into
CarSim.handle_rx_frame() -- the same path a SocketCAN/vcan0 wire used
before the loopback-only refactor.  With the sim started --follower,
injected 0x100/0x110/0x120/0x140/0x400 frames fold into the physics and
the game reacts.
  * LEFT  - scrolling road + car sprite (the "game view")
  * MID   - tiled 3x3 instrument cluster (small non-overlapping gauges) + odo
  * RIGHT - warning lamps, live-data table, CAN console + quick inject, controls
  * FOOTER- keyboard drive hints, panel layout buttons and the
  *         broadcast-ID legend

  * CAN BUS tab - click a line to copy that frame to the clipboard, or
    double-click to load it straight into the CAN INJECT box.
  * RECORD captures the live stream (scope RX/TX/ALL, optional id filter,
    RX-changed-only) to a text dump; SAVE writes it; LOAD reads a dump back
    and REPLAY re-injects it at the original timing into whichever CAN
    output the cockpit is wired to (a second sim).
Run (single entry - the cockpit auto-starts the engine on loopback when no
sim is already listening on the ctrl port):
    python3 tools/carsim_gui.py

Run as separate processes (advanced / remote engine):
    python3 tools/carsim.py --follower &   # ECU side, ctrl :20103
    python3 tools/carsim_gui.py


Headless logic check:  python3 tools/carsim_gui.py --check
"""

import argparse
import atexit
import json
import math
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import filedialog
    _HAS_TK = True
except Exception:                       # non-GUI env / --check still works
    tk = None
    ttk = None
    filedialog = None
    _HAS_TK = False


__version__ = "0.9.11"

DEFAULT_HOST = "127.0.0.1"
CTRL_PORT = 20103            # JSON control channel (matches carsim CTRL_PORT)
KMH_TO_MPH = 0.6213712

# --------------------------------------------------------------------------- #
#  Broadcast frame decode (pure functions -> unit-testable without a display)
# --------------------------------------------------------------------------- #
FRAME_NAMES = {
    0x100: "ENGINE",  0x110: "CHASSIS", 0x120: "STEER",
    0x130: "BODY",    0x140: "GEAR",    0x400: "DRIVE_IN",
}
GEAR_LETTER = {0: "P", 1: "R", 2: "N", 3: "D"}

# External drive-input frame layout (ICSim-style)
#   [0] throttle 0..255   [1] brake 0..100   [2] gear_enum
#   [3] steer 0..255 (128=centre)   [4..7] spare
DRIVE_IN = 0x400


def _byte(data, i):
    return data[i] if i < len(data) else 0


def decode_frame(fr):
    """Turn a raw {"id","data"} broadcast into a human-readable dict."""
    fid = fr["id"]
    data = bytes.fromhex(fr.get("data", ""))
    hexs = fr.get("data", "-")
    name = FRAME_NAMES.get(fid, hex(fid))
    fields = {}
    if fid == 0x100:                                # engine
        rpm = (_byte(data, 0) << 8) | _byte(data, 1)
        fields = {
            "RPM": f"{rpm} rpm",
            "load": f"{_byte(data, 2) / 2.55:.0f} %",
            "throttle": f"{_byte(data, 3) / 2.55:.0f} %",
            "coolant": f"{_byte(data, 4) - 40} C",
            "MAF": f"{((_byte(data, 5) << 8) | _byte(data, 6)) / 100.0:.2f} g/s",
            "oil": f"{_byte(data, 7) - 40} C",
        }
    elif fid == 0x110:                              # chassis
        spd = _byte(data, 0)
        brk = _byte(data, 1)
        flags = []
        if _byte(data, 2) & 0x01:
            flags.append("BRK")
        if _byte(data, 2) & 0x02:
            flags.append("ABS")
        if _byte(data, 2) & 0x04:
            flags.append("TC")
        if _byte(data, 2) & 0x08:
            flags.append("PARK")
        fields = {
            "speed": f"{spd} km/h",
            "brake": f"{brk} %",
            "flags": ",".join(flags) if flags else "-",
        }
    elif fid == 0x120:                              # steer + lamp bits
        s = _byte(data, 0)
        if s >= 128:
            s -= 256
        b1 = _byte(data, 1)
        lights = []
        if b1 & 0x01:
            lights.append("L")
        if b1 & 0x02:
            lights.append("R")
        if b1 & 0x04:
            lights.append("HAZ")
        if b1 & 0x08:
            lights.append("HI")
        if b1 & 0x10:
            lights.append("HL")
        if b1 & 0x20:
            lights.append("WIP")
        fields = {
            "steer": f"{s}",
            "lights": ",".join(lights) if lights else "-",
        }
    elif fid == 0x130:                              # body
        b = _byte(data, 0)
        open_doors = [n for i, n in enumerate(("FL", "FR", "RL", "RR"))
                      if b & (1 << i)]
        fields = {"doors": ",".join(open_doors) if open_doors else "all closed",
                  "trunk": "open" if b & 0x10 else "closed",
                  "hood": "open" if b & 0x20 else "closed",
                  "belt": "yes" if b & 0x40 else "no"}
    elif fid == 0x140:                              # gear / fuel / odo
        odo = (_byte(data, 2) | (_byte(data, 3) << 8)
               | (_byte(data, 4) << 16) | (_byte(data, 5) << 24))
        rt = (_byte(data, 6) | (_byte(data, 7) << 8))
        fields = {
            "gear": GEAR_LETTER.get(_byte(data, 0), "?"),
            "fuel": f"{_byte(data, 1) / 2.55:.0f} %",
            "odo": f"{odo / 1000.0:.1f}k m",
            "runmin": f"{rt} min",
        }
    elif fid == 0x400:                              # inject frame (raw)
        fields = {
            "thr": f"{_byte(data, 0) / 255.0:.0%}",
            "brk": f"{_byte(data, 1)}%",
            "gear": GEAR_LETTER.get(_byte(data, 2) & 0x0F, "?"),
            "steer": f"{_byte(data, 3) - 128:+d}",
        }
    else:
        fields = {"data": hexs or "-"}
    return {"id": fid, "name": name, "dlc": len(data), "hex": hexs, "fields": fields}


# --------------------------------------------------------------------------- #
#  Inject-frame builders (pure -> unit-testable; the "cansend side")
# --------------------------------------------------------------------------- #
def build_drive(throttle=0.0, brake=0.0, gear="D", steer=0.0):
    """ICSim-style control frame 0x400. Returns (can_id, bytes)."""
    gear_enum = {"P": 0, "R": 1, "N": 2, "D": 3}.get(gear, 3)
    steer_b = max(0, min(255, int(round(gearshift=128 + steer * 127)))) if False else \
        max(0, min(255, int(round(128 + steer * 127))))
    data = bytes([
        max(0, min(255, int(round(throttle * 255)))),
        max(0, min(100, int(round(brake * 100)))),
        gear_enum,
        steer_b,
        0, 0, 0, 0,
    ])
    return DRIVE_IN, data


def build_realistic_0x100(rpm=800, load_pct=0, throttle=0.0, coolant=90,
                          maf_gps=0.0, oil=100):
    rpm = max(0, min(8000, int(rpm)))
    maf = max(0, min(65535, int(maf_gps * 100)))
    data = bytes([
        (rpm >> 8) & 0xFF, rpm & 0xFF,
        max(0, min(255, int(load_pct * 2.55))),
        max(0, min(255, int(throttle * 255))),
        max(40, min(215, int(coolant + 40))),
        (maf >> 8) & 0xFF, maf & 0xFF,
        max(40, min(215, int(oil + 40))),
    ])
    return 0x100, data


def build_realistic_0x110(speed_kmh=0, brake_pct=0, flags=0):
    data = bytes([max(0, min(255, int(speed_kmh))),
                  max(0, min(100, int(brake_pct))),
                  flags & 0xFF, 0, 0, 0, 0, 0])
    return 0x110, data


def build_realistic_0x120(steer=0.0, lamps=0):
    s = max(-128, min(127, int(round(steer * 100)))) & 0xFF
    data = bytes([s, lamps & 0xFF, 0, 0, 0, 0, 0, 0])
    return 0x120, data


def build_realistic_0x140(gear="D", fuel_pct=50, odo_km=0, runtime_min=0):
    gear_enum = {"P": 0, "R": 1, "N": 2, "D": 3}.get(gear, 3)
    odo = int(odo_km * 1000)
    data = bytes([
        gear_enum, max(0, min(255, int(fuel_pct * 2.55))),
        odo & 0xFF, (odo >> 8) & 0xFF, (odo >> 16) & 0xFF, (odo >> 24) & 0xFF,
        runtime_min & 0xFF, (runtime_min >> 8) & 0xFF,
    ])
    return 0x140, data


# --------------------------------------------------------------------------- #
#  Slcan "cansend" console client
# --------------------------------------------------------------------------- #
SLCAN_RE = re.compile(r"(?:cansend\s+\S+\s+)?(0x[0-9A-Fa-f]+|[0-9A-Fa-f]{3,8})\s*[#\s]\s*([0-9A-Fa-f]{2,16})$")


def parse_cansend(text):
    """Parse 'cansend vcan0 400#FF00038000000000' -> (can_id, bytes) or None."""
    t = text.strip().rstrip(";")
    m = SLCAN_RE.match(t)
    if not m:
        return None
    try:
        can_id = int(m.group(1), 16)
        data = bytes.fromhex(m.group(2))
    except ValueError:
        return None
    if len(data) > 8 or can_id > 0x1FFFFFFF:
        return None
    return can_id, data


# --------------------------------------------------------------------------- #
#  Frame-semantics advisory (pure -> unit-testable)
# --------------------------------------------------------------------------- #
# 0x400 DRIVE_IN is the only frame the sim consumes as a drivetrain command,
# and only when carsim.py was started with --follower.  The 0x1xx IDs are the
# status broadcasts the sim itself publishes every cycle (engine, chassis,
# steer/lights, body, gear/fuel): putting them on the wire never moves the
# car - but 0x120's lamp bits latch the headlight/wiper/hazard/highbeam/
# indicator switches in ANY mode (one-frame lamp hack).
STATUS_IDS = frozenset((0x100, 0x110, 0x120, 0x130, 0x140))


def inject_advisory(can_id):
    """Truthful one-line note on what injecting this ID can do on the bench.
    Returns None when no disclaimer is needed."""
    if can_id == DRIVE_IN:
        return ("DRIVE_IN cmd - the sim applies 0x400 only when started with "
                "--follower (keyboard drive regardless)")
    if can_id == 0x120:                # steer + lamp bits
        return ("0x120 STEER lamp bits latch the switches off the bus in "
                "any mode: byte 1 headlights 0x10, wipers 0x20, hazard "
                "0x04, highbeam 0x08, left/right indicator 0x01/0x02; "
                "120#0000... all off (steer byte0 itself still needs "
                "--follower)")
    if can_id in STATUS_IDS:
        return ("status frame (%s %s) - the sim publishes these itself, so "
                "injecting one never commands the drivetrain; to drive, send "
                "400#FF00038000000000 with the sim in --follower mode"
                % (hex(can_id), FRAME_NAMES.get(can_id, "")))
    return None


# --------------------------------------------------------------------------- #
#  Frame-capture / recorder helpers (pure -> unit-testable)
# --------------------------------------------------------------------------- #
def fmt_can_id(fid):
    s = "%X" % int(fid)
    return s if len(s) >= 3 else s.zfill(3)


def slcan_tx_line(can_id, data):
    """Lawicel wire frame for a raw CAN frame (used by the injector + tests).
    't' + 3 hex id (11-bit) / 'T' + 8 hex id (29-bit), then dlc + data."""
    if can_id is None or can_id < 0 or can_id > 0x1FFFFFFF or len(data) > 8:
        return None
    if can_id > 0x7FF:
        return "T%08X%d%s" % (can_id, len(data), data.hex().upper())
    return "t%03X%d%s" % (can_id, len(data), data.hex().upper())


def capture_cansend(fid, hexs):
    """'cansend vcan0 ID#DATA' text for a captured frame (COPY / load to inject)."""
    try:
        fid = int(fid)
        hexs = (hexs or "").strip().upper()
        bytes.fromhex(hexs)
    except (TypeError, ValueError):
        return None
    if len(hexs) // 2 > 8 or fid < 0 or fid > 0x1FFFFFFF or hexs == "":
        return None
    return "cansend vcan0 %s#%s" % (fmt_can_id(fid), hexs)


CAP_LINE_RE = re.compile(
    r"^\s*([0-9]+(?:\.[0-9]+)?)\s+(RX|TX)\s+"
    r"([0-9A-Fa-f]{3,8})\s+([0-9A-Fa-f]{2,16})(?:\s.*)?$")


def parse_capture_text(text):
    """Parse a recorder dump back into replay items.
    Lines: '<sec> RX|TX <id-hex> <data-hex> [# optional src]', '#' = comment."""
    out = []
    for raw in (text or "").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        m = CAP_LINE_RE.match(line)
        if not m:
            continue
        sec, d, hid, hx = m.groups()
        try:
            fid = int(hid, 16)
            data = bytes.fromhex(hx)
        except ValueError:
            continue
        if len(data) > 8:
            continue
        out.append({"sec": float(sec), "dir": d, "id": fid,
                    "data": hx.upper(), "dlc": len(data)})
    return out


def rec_to_cansend(items):
    """Concat a saved/parsed dump into 'cansend vcan0 ID#DATA' lines."""
    out = []
    for it in items:
        c = capture_cansend(it.get("id"), it.get("data"))
        if c:
            out.append(c)
    return "\n".join(out)


class CanInjector:
    """Injects raw CAN frames over the sim's JSON control channel.

    Loopback-only carsim has no SLCAN/CAN socket: a frame goes in as a
    {"t":"frame","id":<can_id>,"data":"<hex>"} message on the ctrl
    port -- exactly what the cockpit and external scripts send.  The sim
    feeds it to CarSim.handle_rx_frame(), the same path a SocketCAN/vcan0
    wire used before the loopback-only refactor."""

    def __init__(self, host, ctrl_port, on_log=None):
        self.host = host
        self.port = ctrl_port
        self.on_log = on_log
        self.status = "offline"
        self.sock = None
        self._lock = threading.Lock()
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def stop(self):
        self._run = False
        s = self.sock
        if s is not None:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _loop(self):
        while self._run:
            try:
                s = socket.create_connection((self.host, self.port), timeout=3)
            except OSError:
                self.status = "offline"
                time.sleep(2.0)
                continue
            try:
                s.settimeout(1.0)
                with self._lock:
                    self.sock = s
                self.status = "online"
                if self.on_log:
                    self.on_log(f"CAN injector online {self.host}:{self.port} (ctrl)")
            except OSError:
                self.status = "offline"
                with self._lock:
                    self.sock = None
                try:
                    s.close()
                except OSError:
                    pass
            while self._run:
                try:
                    s.recv(1024)                      # drain (we ignore rx here)
                except socket.timeout:
                    continue
                except OSError:
                    break
            with self._lock:
                if self.sock is s:
                    self.sock = None
            self.status = "offline"

    def send_frame(self, can_id, data):
        if not data:
            return False
        msg = {"t": "frame", "id": can_id, "data": data.hex()}
        with self._lock:
            s = self.sock
        if s is None:
            self.status = "offline"
            return False
        try:
            s.sendall((json.dumps(msg) + "\n").encode("ascii"))
            return True
        except OSError:
            return False

    def send_cansend(self, text):
        parsed = parse_cansend(text)
        if parsed is None:
            return False, "bad frame (want 400#FF00038000000000)"
        can_id, data = parsed
        if self.send_frame(can_id, data):
            return True, slcan_tx_line(can_id, data) or ""
        return False, "injector offline"


# --------------------------------------------------------------------------- #
#  Gauge geometry (pure helpers; tkinter-arc convention kept consistent)
# --------------------------------------------------------------------------- #
def _pt(cx, cy, r, deg):
    a = math.radians(deg)
    return cx + r * math.cos(a), cy - r * math.sin(a)


def gauge_angle(value, vmax, start=135.0, sweep=270.0):
    v = max(0.0, min(value, vmax))
    return start + (v / vmax) * sweep if vmax else start


def arc_points(cx, cy, r, a0, a1, n=48):
    pts = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        pts += list(_pt(cx, cy, r, a))
    return pts


# --------------------------------------------------------------------------- #
#  JSON control client (thread-safe, importable without a display)
# --------------------------------------------------------------------------- #
class CockpitClient:
    """Connects to carsim's JSON control channel; stores the latest state."""

    def __init__(self, host, port, on_state=None, on_log=None):
        self.host = host
        self.port = port
        self.on_state = on_state
        self.on_log = on_log
        self.state = {}
        self.status = "offline"
        self.sock = None
        self._lock = threading.Lock()
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def stop(self):
        self._run = False
        s = self.sock
        if s is not None:
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _set_status(self, s):
        self.status = s

    def _loop(self):
        while self._run:
            try:
                s = socket.create_connection((self.host, self.port), timeout=3)
            except OSError:
                self._set_status("offline")
                if self.on_log:
                    self.on_log(f"connect {self.host}:{self.port} failed - retrying")
                time.sleep(2.0)
                continue
            try:
                s.settimeout(0.5)
                self.sock = s
                self._set_status("online")
                if self.on_log:
                    self.on_log(f"connected to sim {self.host}:{self.port}")
            except OSError:
                s.close()
                self._set_status("offline")
                continue
            buf = b""
            while self._run:
                try:
                    chunk = s.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    raw = raw.strip()
                    if not raw:
                        continue
                    try:
                        msg = json.loads(raw.decode("utf-8", "replace"))
                    except ValueError:
                        continue
                    if isinstance(msg, dict) and msg.get("t") == "state":
                        self.state = msg
                        if self.on_state:
                            self.on_state(msg)
            with self._lock:
                if self.sock is s:
                    self.sock = None
            try:
                s.close()
            except OSError:
                pass
            self._set_status("offline")
            if self._run and self.on_log:
                self.on_log("disconnected - retrying")

    def send(self, msg):
        with self._lock:
            s = self.sock
        if s is None:
            return False
        try:
            s.sendall((json.dumps(msg) + "\n").encode("utf-8"))
            return True
        except OSError:
            return False

    @property
    def online(self):
        return self.status == "online"



# --------------------------------------------------------------------------- #
#  Road + gauge drawing helpers (only used when Tk is present)
# --------------------------------------------------------------------------- #
def _arc(cv, cx, cy, r, a0, a1, width=3, color="#888", n=48):
    cv.create_line(*arc_points(cx, cy, r, a0, a1, n), fill=color,
                   width=width, capstyle="round", smooth=True)


def _draw_gauge(cv, cx, cy, r, value, vmax, label, color,
                start=135.0, sweep=270.0, major=4, units=""):
    cv.create_oval(cx - r - 5, cy - r - 5, cx + r + 5, cy + r + 5,
                   outline="#0a0a0a", fill="#101010")
    _arc(cv, cx, cy, r, start, start + sweep, width=3, color="#3a3a3a")
    for i in range(major + 1):
        v = vmax * i / major
        a = gauge_angle(v, vmax, start, sweep)
        x1, y1 = _pt(cx, cy, r - 3, a)
        x2, y2 = _pt(cx, cy, r - 11, a)
        cv.create_line(x1, y1, x2, y2, fill="#bbb", width=2)
        lx, ly = _pt(cx, cy, r - 20, a)
        cv.create_text(lx, ly, text=str(int(v)), fill="#999",
                       font=("Helvetica", 7, "bold"))
    _arc(cv, cx, cy, r, start + sweep * 0.85, start + sweep,
         width=3, color="#c0392b")
    a = gauge_angle(value, vmax, start, sweep)
    nx, ny = _pt(cx, cy, r - 13, a)
    cv.create_line(cx, cy, nx, ny, fill=color, width=3)
    cv.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=color, outline="#000")
    # value + units inside the dial; caption BELOW the dial face
    val_txt = f"{value:.0f}" if vmax >= 100 else f"{value:.1f}"
    cv.create_text(cx, cy - 2, text=val_txt, fill="#e8e8e8",
                   font=("Helvetica", 12, "bold"))
    if units:
        cv.create_text(cx, cy + 12, text=units, fill="#8b98a5",
                       font=("Helvetica", 7))
    cv.create_text(cx, cy + r + 14, text=label, fill="#9fd2ff",
                   font=("Helvetica", 8, "bold"))


# --------------------------------------------------------------------------- #
#  The cockpit window
# --------------------------------------------------------------------------- #
class Cockpit:
    def __init__(self, root, host, ctrl_port):
        if not _HAS_TK:
            raise RuntimeError("tkinter not available")
        self.root = root
        self.host = host
        self.port = ctrl_port
        self.units = "kmh"
        self.autopilot = False
        self.ap_target = 100.0
        self._state = {}
        self._ev_q = []            # sim events queued by the RX thread
        self._last_ev = ""         # most recently logged event text
        self._ev_seen = []         # small ring of logged events (spam guard)
        self._last_hint = None     # hint text currently shown
        self.hint_lbl = None       # footer hint label (built in footer)
        self._scroll = 0.0
        self._car_lat = 0.0          # lateral car position (road stays fixed)
        self._lat_rate = 12.0        # px per render tick at full steer (lane-hold)
        self._prev_frames = {}
        self.frame_history = []
        self._last_frame_val = {}   # id -> last data (mark changed)
        self._bus_seq = 0
        self._bus_pause = False      # FREEZE the CAN BUS monitor
        self._bus_frozen_at = None   # seq at which the freeze snapshot was taken
        self._bus_pause_btn = None
        self._sb_last_frac = 1.0

        self._bus_view = []          # records rendered in CAN BUS monitor
        self._cap_lbl = None         # capture-feedback label
        self._rec_cnt_lbl = None     # live frame-count label
        self._rec_btn = None         # RECORD toggle button
        self._rp_btn = None          # REPLAY/STOP toggle button
        self._rec_on = False         # traffic recorder running
        self._rec_scope = "all"      # all | rx | tx
        self._rec_filter = ""        # optional hex id filter ("," / space list)
        self._rx_delta = False       # RX changed-only mode
        self._rec_last = {}          # id -> last RX data (delta mode)
        self._rec_frames = []        # recorded / loaded items (ms, dir, id, data, src)
        self._rec_t0 = None
        self._rp_job = None          # pending root.after id (replay chain)
        self._rp_stop = False
        self._rp_items = []
        self._rp_i = 0
        self._rp_t0 = 0.0
        self._ap_t0 = 0.0
        self._ap_phase = 0
        self._last_live = None

        self.client = CockpitClient(host, ctrl_port, on_state=self._on_state,
                                    on_log=self._log)
        self.injector = CanInjector(host, ctrl_port, on_log=self._log)

        self.root.title("CAN FIRECOCKPIT - SIMWheelz")
        self.root.configure(bg="#0b0d10")
        self._build_ui()
        self._bind_keys()
        self._last_render = 0.0
        self._ap_last = 0.0
        self._tick()

    # ------------------------------------------------------------ state hook
    def _on_state(self, msg):
        self._state = msg
        self._ev_q.extend(msg.get("events", []))
        self._sync_frames(msg.get("frames", []))

    def _sync_frames(self, frames):
        # log EVERY received frame so the CAN BUS monitor shows the live
        # stream (each id updates at its own broadcast period)
        for fr in frames:
            self._bus_seq += 1
            key = fr.get("id")
            val = fr.get("data", "")
            changed = self._last_frame_val.get(key) != val
            self._last_frame_val[key] = val
            rec = {"ts": time.strftime("%H:%M:%S"),
                   "id": key, "dlc": fr.get("dlc", 8),
                   "data": val, "n": self._bus_seq, "changed": changed}
            self.frame_history.append(rec)
            self._rec_frame(rec)
        self.frame_history = self.frame_history[-120:]

    def _drain_events(self):
        """Move sim events queued by the RX thread into the Service log.

        The sim re-pushes its rolling event window on every state update,
        so a plain append would spam repeats; keep a small ring of recently
        logged events and only write the ones not shown yet."""
        while self._ev_q:
            ev = self._ev_q.pop(0)
            if ev in self._ev_seen:
                continue
            self._last_ev = ev
            self._ev_seen.append(ev)
            del self._ev_seen[:-10]
            self._log("EV  " + ev)

    def _hint_text(self, st):
        """Engine/gear-aware footer hint: what the next key will do."""
        gear = st.get("gear", "P")
        if not st.get("engine_on"):
            if gear in ("P", "N"):
                return ("ENGINE OFF gear %s | press i / START to crank, "
                        "then D and hold %s" % (gear, "↑"))
            return ("ENGINE OFF gear %s | shift to P or N, then i / START"
                    % gear)
        if gear == "P":
            return "park: %s only revs the engine - press D to drive" % "↑"
        if gear == "N":
            return "neutral: no drive - press D"
        if gear == "R":
            return "reverse: %s accelerates backward" % "↑"
        if st.get("speed", 0.0) < 0.5:
            if st.get("brake", 0.0) > 0.05:
                return ("brake held - release %s then hold %s to launch"
                        % ("↓", "↑"))
            return "gear D: hold %s to launch" % "↑"
        return "gear D engine ON"

    def _hint_colour(self, text):
        low = text.lower()
        if any(k in low for k in ("engine off", "park", "neutral",
                                  "brake held")):
            return "#ffd60a"      # blocking - driver action needed
        return "#9fc7e8"

    def _update_hint(self):
        if self.hint_lbl is None:
            return
        text = self._hint_text(self._state)
        if text != self._last_hint:
            self._last_hint = text
            self.hint_lbl.configure(text=text, fg=self._hint_colour(text))

    def _log(self, text):
        if getattr(self, "log_box", None) is not None:
            self.log_box.insert("end", text + "\n")
            self.log_box.see("end")

    def _log_tx(self, fid, data, src):
        """Log a locally-produced CAN frame into the bus monitor (TX side)
        so every cockpit action is visibly tied to the frame it produces."""
        self._bus_seq += 1
        rec = {"ts": time.strftime("%H:%M:%S"), "id": fid, "dlc": len(data),
               "data": data.hex().upper(), "tx": True, "src": src,
               "n": self._bus_seq}
        self.frame_history.append(rec)
        self.frame_history = self.frame_history[-120:]
        self._rec_frame(rec)

    # ------------------------------------------------------- traffic recorder
    def _rec_pass(self, rec):
        """Scope / RX-delta / id-filter gate for the traffic recorder."""
        tx = bool(rec.get("tx"))
        if self._rec_scope == "rx" and tx:
            return False
        if self._rec_scope == "tx" and not tx:
            return False
        if self._rx_delta and not tx:
            key = rec.get("id")
            hexs = rec.get("data", "")
            if self._rec_last.get(key) == hexs:
                return False
            self._rec_last[key] = hexs
        filt = (self._rec_filter or "").strip()
        if filt:
            want = []
            for tok in filt.replace(",", " ").split():
                try:
                    want.append(int(tok, 16))
                except ValueError:
                    continue
            if want and rec.get("id") not in want:
                return False
        return True

    def _rec_frame(self, rec):
        """Append a monitor record to the recorder when it is running."""
        if not self._rec_on or not self._rec_pass(rec):
            return
        if str(rec.get("src", "")).startswith("replay"):   # never re-record replay
            return
        if self._rec_t0 is None:
            self._rec_t0 = time.monotonic()
        ms = int(round((time.monotonic() - self._rec_t0) * 1000.0))
        hexs = rec.get("data") or ""
        self._rec_frames.append({"ms": ms,
                                 "dir": "TX" if rec.get("tx") else "RX",
                                 "id": rec.get("id"),
                                 "dlc": rec.get("dlc", len(hexs) // 2),
                                 "data": hexs.upper(),
                                 "src": rec.get("src", "")})
        self._update_rec_cnt()

    # ============================================================== UI build
    def _build_ui(self):
        # =================================================================
        #  Single clean grid on the root so nothing overlaps:
        #    row 0  status bar
        #    row 1  paned main area: CAR band + PANEL band in one vertical
        #          PanedWindow -- drag the sash to expand either vertically
        #    row 2  footer / control instructions
        #  Every panel has a [-] / [+] button in its header to minimize or
        #  expand it, and the three bottom panels share a horizontal
        #  PanedWindow whose sashes resize them left / right.
        # =================================================================
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)      # paned main area
        self.root.minsize(960, 600)

        # ---- row 0: top status bar (plain tk: native ttk theme paints green)
        top = tk.Frame(self.root, bg="#0b0d10")
        top.grid(row=0, column=0, sticky="ew", padx=8, pady=(6, 2))
        self.status_lbl = tk.Label(top, text="status: connecting...",
                                   bg="#0b0d10", fg="#d7e2ea")
        self.status_lbl.pack(side="left")
        tk.Label(top, text="  mode:", bg="#0b0d10", fg="#8b98a5").pack(side="left")
        self.mode_lbl = tk.Label(top, text="-", bg="#0b0d10", fg="#e8e8e8")
        self.mode_lbl.pack(side="left", padx=(0, 10))
        tk.Label(top, text="time:", bg="#0b0d10", fg="#8b98a5").pack(side="left")
        self.ts_lbl = tk.Label(top, text="0.0 s", bg="#0b0d10", fg="#e8e8e8")
        self.ts_lbl.pack(side="left", padx=(0, 10))
        tk.Label(top, text="odo:", bg="#0b0d10", fg="#8b98a5").pack(side="left")
        self.odo_lbl = tk.Label(top, text="0.0 km", bg="#0b0d10", fg="#e8e8e8")
        self.odo_lbl.pack(side="left", padx=(0, 12))
        self.source_lbl = tk.Label(top, text="drive: keyboard",
                                  bg="#0b0d10", fg="#ffd60a")
        self.source_lbl.pack(side="left", padx=(0, 12))
        self.units_var = tk.StringVar(value="km/h")
        tk.Radiobutton(top, text="km/h", variable=self.units_var, value="km/h",
                       command=self._toggle_units, bg="#0b0d10", fg="#d7e2ea",
                       selectcolor="#1c2733", activebackground="#0b0d10",
                       activeforeground="#ffffff", highlightthickness=0).pack(side="right")
        tk.Radiobutton(top, text="mph", variable=self.units_var, value="mph",
                       command=self._toggle_units, bg="#0b0d10", fg="#d7e2ea",
                       selectcolor="#1c2733", activebackground="#0b0d10",
                       activeforeground="#ffffff", highlightthickness=0).pack(side="right")

        # ---- row 1: vertical master sash: CAR band over PANEL band
        self._vp = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        self._vp.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 4))

        self._band_car = ttk.Frame(self._vp)
        self._vp.add(self._band_car, weight=3)
        self._band_pan = ttk.Frame(self._vp)
        self._vp.add(self._band_pan, weight=2)

        # panel bookkeeping + per-band orientation (side-by-side / stacked)
        self._panels = {}
        self._band_orient = {"car": "horizontal", "pan": "horizontal"}

        # ---- CAR band: ROAD / GAME VIEW  +  INSTRUMENT CLUSTER
        road = self._panel("car", "ROAD / GAME VIEW", key="road", weight=3)
        self.road_cv = tk.Canvas(road, bg="#0d1208", highlightthickness=0)
        self.road_cv.pack(fill="both", expand=True)

        clus = self._panel("car", "INSTRUMENT CLUSTER", key="cluster", weight=1)
        clus.rowconfigure(0, weight=1)
        clus.columnconfigure(0, weight=1)
        self.cluster_cv = tk.Canvas(clus, bg="#101010", highlightthickness=0)
        self.cluster_cv.grid(row=0, column=0, sticky="nsew")

        # ---- PANEL band: COCKPIT / CAN BUS / SERVICE
        self._nb = None
        self._build_right()

        # grid both bands per their current orientation
        self._relayout_band("car")
        self._relayout_band("pan")

        # ---- row 2: footer / control instructions
        self._build_footer()
        # Once the window has a real size, place the master sash.
        self.root.after(80, self._split_initial)

    def _panel(self, band, title, key, weight=1):
        """Create one resizable / collapsible / maximizable panel in a band.
        `band` is 'car' or 'pan'; the outer frame is gridded by
        _relayout_band for the band's current orientation.  Header row has a
        title, a [-] collapse toggle and an M/R maximize toggle.  Returns the
        body frame that content grids into."""
        master = self._band_car if band == "car" else self._band_pan
        outer = ttk.Frame(master)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(1, weight=1)
        hdr = ttk.Frame(outer)
        hdr.grid(row=0, column=0, sticky="ew")
        hdr.columnconfigure(0, weight=1)
        ttk.Label(hdr, text=title, font=("Consolas", 9, "bold"),
                  anchor="w").grid(row=0, column=0, sticky="w",
                                   padx=(4, 6), pady=(2, 0))
        btn_min = ttk.Button(hdr, text="-", width=2,
                             command=lambda k=key: self._panel_min(k))
        btn_min.grid(row=0, column=1, padx=(0, 2), pady=(2, 0))
        btn_max = ttk.Button(hdr, text="M", width=2,
                             command=lambda k=key: self._panel_max(k))
        btn_max.grid(row=0, column=2, padx=(0, 4), pady=(2, 0))
        body = ttk.Frame(outer)
        body.grid(row=1, column=0, sticky="nsew", pady=(2, 0))
        self._panels[key] = {"band": band, "weight": weight, "outer": outer,
                             "body": body, "min": False, "max": False,
                             "btn_min": btn_min, "btn_max": btn_max}
        return body

    def _relayout_band(self, band):
        """Re-grid every panel in one band for the band's orientation,
        honouring collapsed (min) and maximized (max) states."""
        bandf = self._band_car if band == "car" else self._band_pan
        items = [p for p in self._panels.values() if p["band"] == band]
        for i in range(len(items) + 2):
            bandf.grid_columnconfigure(i, weight=0)
            bandf.grid_rowconfigure(i, weight=0)
        maxed = [p for p in items if p["max"]]
        if maxed:
            p = maxed[0]
            for q in items:
                if q is p:
                    q["outer"].grid(row=0, column=0, sticky="nsew")
                else:
                    q["outer"].grid_remove()
            bandf.grid_rowconfigure(0, weight=1)
            bandf.grid_columnconfigure(0, weight=1)
        else:
            orient = self._band_orient.get(band, "horizontal")
            n = len(items)
            if orient == "horizontal":
                bandf.grid_rowconfigure(0, weight=1)
                for i, p in enumerate(items):
                    w = 0 if p["min"] else p["weight"]
                    p["outer"].grid(row=0, column=i, sticky="nsew",
                                    padx=(0, 4) if i < n - 1 else (0, 0))
                    bandf.grid_columnconfigure(i, weight=w)
            else:
                bandf.grid_columnconfigure(0, weight=1)
                for i, p in enumerate(items):
                    w = 0 if p["min"] else p["weight"]
                    p["outer"].grid(row=i, column=0, sticky="nsew",
                                    pady=(0, 4) if i < n - 1 else (0, 0))
                    bandf.grid_rowconfigure(i, weight=w)
        for p in items:
            if p["min"]:
                p["body"].grid_remove()
            else:
                p["body"].grid(row=1, column=0, sticky="nsew", pady=(2, 0))
        self.root.update_idletasks()

    def _panel_min(self, key):
        p = self._panels[key]
        p["min"] = not p["min"]
        p["btn_min"].configure(text="+" if p["min"] else "-")
        self._relayout_band(p["band"])
        self._log("panel %s %s" % (key, "minimized - click [+] to expand"
                                  if p["min"] else "expanded"))

    def _panel_max(self, key):
        p = self._panels[key]
        p["max"] = not p["max"]
        p["btn_max"].configure(text="R" if p["max"] else "M")
        self._relayout_band(p["band"])
        self._log("panel %s %s" % (key, "maximized" if p["max"] else "restored"))

    def _panel_orient(self, orient):
        orient = "vertical" if orient == "vertical" else "horizontal"
        self._band_orient["car"] = orient
        self._band_orient["pan"] = orient
        self._relayout_band("car")
        self._relayout_band("pan")
        self._log("layout %s" % ("stacked" if orient == "vertical"
                                 else "side-by-side"))

    def _toggle_mon(self):
        vis = self._mon_vis.get()
        try:
            if vis:
                self.bus_row.grid(row=3, column=0, sticky="nsew",
                                  padx=4, pady=(0, 4))
            else:
                self.bus_row.grid_remove()
            self.root.update_idletasks()
        except Exception as exc:
            self._log("mon toggle error: %r" % (exc,))

    def _split_initial(self):
        """Place the master sash once the window has a real size (called
        80 ms after the UI is built)."""
        try:
            self.root.update_idletasks()
            h = self._vp.winfo_height()
            if h > 200:
                self._vp.sashpos(0, int(h * 0.45))
        except Exception:
            pass

    def _build_right(self):
        # Three collapsible / maximizable panels in the PANEL band, re-gridded
        # by _relayout_band("pan") for the band's current orientation.
        # ---------------- COCKPIT
        cockpit = self._panel("pan", "COCKPIT", key="cockpit", weight=1)
        cockpit.rowconfigure(1, weight=1)
        cockpit.columnconfigure(0, weight=1)
        self.lamps_cv = tk.Canvas(cockpit, height=56, bg="#0e0e0e",
                                  highlightthickness=0)
        self.lamps_cv.grid(row=0, column=0, sticky="ew", padx=4, pady=(4, 0))
        self.live_txt = tk.Text(cockpit, bg="#0e0e0e", fg="#cfd8e0",
                                font=("Consolas", 9), padx=6, pady=3,
                                relief="flat")
        self.live_txt.grid(row=1, column=0, sticky="nsew", padx=4, pady=(4, 0))
        cframe = ttk.LabelFrame(cockpit, text="CAN INJECT (cansend / console)")
        cframe.grid(row=2, column=0, sticky="ew", padx=4, pady=(4, 2))
        self.inject_var = tk.StringVar(value="cansend vcan0 400#FF00038000000000")
        row = ttk.Frame(cframe)
        row.pack(fill="x", padx=4, pady=(4, 2))
        self.inject_entry = ttk.Entry(row, textvariable=self.inject_var)
        self.inject_entry.pack(side="left", fill="x", expand=True)
        self.inject_entry.bind("<Return>", self._send_inject)
        self.inject_entry.bind("<KP_Enter>", self._send_inject)
        self.inject_btn = ttk.Button(row, text="SEND",
                                     command=self._send_inject)
        self.inject_btn.pack(side="left", padx=(4, 0))
        # Always-visible feedback line: Enter-key / SEND results at a glance.
        try:
            _fb_bg = ttk.Style().lookup("TLabelFrame", "background") or "#d9d9d9"
        except Exception:
            _fb_bg = "#d9d9d9"
        self.inject_fb = tk.Label(
            cframe, text="type ID#DATA (e.g. 110#4700000000000000) or "
                         "cansend vcan0 ID#DATA, then press Enter or SEND",
            bg=_fb_bg, fg="#5a6b78", font=("Consolas", 8), anchor="w",
            justify="left", wraplength=0)
        self.inject_fb.pack(fill="x", padx=6, pady=(0, 4))
        self.inject_fb.bind("<Configure>", self._fb_fit)
        # ---------------- CAN BUS  (pane 1, widest; monitor + FREEZE)
        canbus = self._panel("pan", "CAN BUS", key="canbus", weight=2)
        canbus.columnconfigure(0, weight=1)
        canbus.rowconfigure(3, weight=1)          # monitor row expands
        tk.Label(canbus,
                 text="CLICK = copy   DOUBLE-CLICK = load into CAN INJECT",
                 bg="#0a0a0a", fg="#ffd60a", font=("Consolas", 8),
                 anchor="w").grid(row=0, column=0, sticky="ew", padx=4, pady=(4, 0))
        recf = ttk.LabelFrame(canbus, text="TRAFFIC RECORD / REPLAY")
        recf.grid(row=1, column=0, sticky="ew", padx=4, pady=(3, 2))
        r1 = ttk.Frame(recf)
        r1.pack(fill="x", padx=4, pady=2)
        self._bus_pause_btn = ttk.Button(r1, text="FREEZE",
                                         command=self._bus_pause_toggle)
        self._bus_pause_btn.pack(side="left")
        self._rec_btn = ttk.Button(r1, text="RECORD",
                                   command=self._rec_toggle)
        self._rec_btn.pack(side="left", padx=(6, 0))
        self._rec_cnt_lbl = ttk.Label(r1, text="0 frames")
        self._rec_cnt_lbl.pack(side="left", padx=8)
        self._scope_var = tk.StringVar(value="all")
        for lab, val in (("ALL", "all"), ("RX", "rx"), ("TX", "tx")):
            rb = ttk.Radiobutton(r1, text=lab, value=val,
                                variable=self._scope_var,
                                command=lambda v=val: self._set_scope(v))
            rb.pack(side="left", padx=(6, 0))
        self._rxdelta_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(r1, text="RX changed only",
                        variable=self._rxdelta_var).pack(side="left", padx=8)
        r2 = ttk.Frame(recf)
        r2.pack(fill="x", padx=4, pady=(0, 3))
        ttk.Label(r2, text="id filter").pack(side="left")
        self._idfilt_var = tk.StringVar(value="")
        e_filt = ttk.Entry(r2, textvariable=self._idfilt_var, width=10)
        e_filt.pack(side="left", padx=4)
        e_filt.bind("<KeyRelease>",
                    lambda e: setattr(self, "_rec_filter", self._idfilt_var.get()))
        ttk.Button(r2, text="SAVE", command=self._rec_save).pack(side="left",
                                                                  padx=(8, 0))
        ttk.Button(r2, text="LOAD", command=self._rec_load).pack(side="left",
                                                                  padx=(4, 0))
        self._rp_btn = ttk.Button(r2, text="REPLAY", command=self._replay_toggle)
        self._rp_btn.pack(side="left", padx=(4, 0))
        self._mon_vis = tk.BooleanVar(value=True)
        ttk.Checkbutton(r2, text="MON", variable=self._mon_vis,
                        command=self._toggle_mon).pack(side="left", padx=(8, 0))
        self._cap_lbl = tk.Label(canbus, text="", bg="#0a0a0a", fg="#9fc7e8",
                                 font=("Consolas", 8), anchor="w")
        self._cap_lbl.grid(row=2, column=0, sticky="ew", padx=4, pady=(0, 2))
        busrow = ttk.Frame(canbus)
        self.bus_row = busrow
        busrow.grid(row=3, column=0, sticky="nsew", padx=4, pady=(0, 4))
        busrow.columnconfigure(0, weight=1)
        busrow.rowconfigure(0, weight=1)
        self.bus_txt = tk.Text(busrow, bg="#0a0a0a", fg="#cfd8e0",
                               font=("Consolas", 8), relief="flat", padx=4,
                               pady=2, wrap="char")
        self.bus_sb = ttk.Scrollbar(busrow, orient="vertical",
                                    command=self._bus_sb_cmd)
        self.bus_sb_h = ttk.Scrollbar(busrow, orient="horizontal",
                                      command=self.bus_txt.xview)
        self.bus_txt.configure(yscrollcommand=self.bus_sb.set,
                               xscrollcommand=self.bus_sb_h.set)
        self.bus_txt.grid(row=0, column=0, sticky="nsew")
        self.bus_sb.grid(row=0, column=1, sticky="ns")
        self.bus_sb_h.grid(row=1, column=0, sticky="ew")
        self.bus_txt.bind("<Button-1>", self._bus_capture)
        self.bus_txt.bind("<Double-Button-1>", self._bus_load)
        self.bus_txt.bind("<MouseWheel>", self._bus_wheel)
        self.bus_txt.bind("<Button-4>", self._bus_wheel)
        self.bus_txt.bind("<Button-5>", self._bus_wheel)

        # ---------------- SERVICE / DIAGNOSTICS  (pane 2)
        service = self._panel("pan", "SERVICE / DIAGNOSTICS", key="service", weight=1)
        service.columnconfigure(1, weight=1)
        service.columnconfigure(2, weight=1)
        service.columnconfigure(3, weight=1)
        ttk.Label(service, text="Ignition").grid(row=0, column=0, sticky="w")
        self.ign_btn = ttk.Button(service, text="START",
                                  command=lambda: self._ign(True))
        self.ign_btn.grid(row=0, column=1, padx=2, sticky="ew")
        ttk.Button(service, text="OFF", command=lambda: self._ign(False)).grid(
            row=0, column=2, sticky="ew")
        ttk.Label(service, text="Gear").grid(row=1, column=0, sticky="w")
        for i, g in enumerate("PRND"):
            ttk.Button(service, text=g, command=lambda g=g: self.client.send(
                {"t": "gear", "gear": g})).grid(row=1, column=1 + i, sticky="ew")
        ttk.Label(service, text="Throttle").grid(row=2, column=0, sticky="w")
        self.thr_var = tk.DoubleVar(value=0.0)
        ttk.Scale(service, from_=0, to=100, variable=self.thr_var,
                  command=lambda v: self.client.send(
                      {"t": "input", "throttle": float(v) / 100.0,
                       "brake": self.brk_var.get() / 100.0,
                       "steer": self.steer_var.get() / 100.0})).grid(
            row=2, column=1, columnspan=3, sticky="ew")
        ttk.Label(service, text="Brake").grid(row=3, column=0, sticky="w")
        self.brk_var = tk.DoubleVar(value=0.0)
        ttk.Scale(service, from_=0, to=100, variable=self.brk_var,
                  command=lambda v: self.client.send(
                      {"t": "input", "throttle": self.thr_var.get() / 100.0,
                       "brake": float(v) / 100.0,
                       "steer": self.steer_var.get() / 100.0})).grid(
            row=3, column=1, columnspan=3, sticky="ew")
        ttk.Label(service, text="Steer").grid(row=4, column=0, sticky="w")
        self.steer_var = tk.DoubleVar(value=0.0)
        ttk.Scale(service, from_=-100, to=100, variable=self.steer_var,
                  command=lambda v: self.client.send(
                      {"t": "input", "throttle": self.thr_var.get() / 100.0,
                       "brake": self.brk_var.get() / 100.0,
                       "steer": float(v) / 100.0})).grid(
            row=4, column=1, columnspan=3, sticky="ew")
        ttk.Button(service, text="CRUISE",
                   command=self._cruise_toggle).grid(row=5, column=1, sticky="ew")
        self.cruise_lbl = ttk.Label(service, text="off", foreground="#888")
        self.cruise_lbl.grid(row=5, column=2, sticky="ew")
        self.ap_btn = ttk.Button(service, text="AUTOPILOT OFF",
                                 command=self._ap_toggle)
        self.ap_btn.grid(row=5, column=3, sticky="ew")
        ttk.Button(service, text="RESET", command=lambda: self.client.send(
            {"t": "reset"})).grid(row=6, column=1, sticky="ew")
        self.dtc_lbl = ttk.Label(service, text="no active DTCs", foreground="#888",
                                 wraplength=220, justify="left")
        self.dtc_lbl.grid(row=7, column=0, columnspan=4, sticky="w")
        swf = ttk.LabelFrame(service, text="Switches")
        swf.grid(row=8, column=0, columnspan=4, sticky="ew", pady=5)
        self.sw_checks = {}
        sw_defs = (
            ("headlights", "HEADLIGHT", False),
            ("wipers", "WIPERS", False),
            ("hazard", "HAZARD", False),
            ("parkbrake", "PARK BRAKE", False),
            ("door_fl", "DOOR FL", False),
            ("door_fr", "DOOR FR", False),
            ("door_rl", "DOOR RL", False),
            ("door_rr", "DOOR RR", False),
            ("trunk", "TRUNK", False),
            ("hood", "HOOD", False),
            ("seatbelt", "BELT", True),
        )
        for i, (name, label, default) in enumerate(sw_defs):
            v = tk.BooleanVar(value=default)
            cb = ttk.Checkbutton(swf, text=label, variable=v,
                                 command=lambda n=name, v=v: self._switch3(n, v))
            cb.grid(row=i // 3, column=i % 3, sticky="w", padx=4, pady=2)
            self.sw_checks[name] = v
        ftf = ttk.LabelFrame(service, text="Faults")
        ftf.grid(row=9, column=0, columnspan=4, sticky="ew", pady=5)
        self.fl_checks = {}
        for i, name in enumerate(("mil", "overheat", "flat", "abs")):
            v = tk.BooleanVar(value=False)
            cb = ttk.Checkbutton(ftf, text=name, variable=v,
                                 command=lambda n=name, v=v: self._fault3(n, v))
            cb.grid(row=i // 2, column=i % 2, sticky="w", padx=4, pady=2)
            self.fl_checks[name] = v
        ttk.Button(ftf, text="clear all", command=self._clear_faults).grid(
            row=2, column=0, columnspan=2)
        service.rowconfigure(10, weight=1)
        self.log_box = tk.Text(service, bg="#0e0e0e", fg="#9ff",
                               font=("Consolas", 8), relief="flat")
        self.log_box.grid(row=10, column=0, columnspan=4, sticky="nsew",
                          pady=(4, 0))

    # ------------------------------------------------------- CAN BUS freeze
    def _bus_pause_toggle(self):
        """FREEZE the live monitor so you can scroll back and click-copy a frame."""
        self._bus_pause = not self._bus_pause
        self._bus_frozen_at = None
        if self._bus_pause_btn is not None:
            self._bus_pause_btn.configure(
                text="LIVE >>" if self._bus_pause else "FREEZE")
        self._log("CAN BUS %s" % ("FROZEN - scroll back & click to copy"
                                  if self._bus_pause else "LIVE"))
        self._render_bus()

    def _render_bus_lines(self, view, tail):
        if not getattr(self, "bus_txt", None):
            return
        self._bus_view = view
        self.bus_txt.configure(state="normal")
        self.bus_txt.delete("1.0", "end")
        self.bus_txt.tag_configure("capture", background="#26313d",
                                   foreground="#ffffff")
        self.bus_txt.tag_configure("tx", foreground="#ffd60a")
        self.bus_txt.tag_configure("chg", background="#17323d",
                                   foreground="#34d1ce")
        for rec in view:
            try:
                dec = decode_frame(rec)
                fields = "  ".join(f"{k} {v}"
                                   for k, v in dec["fields"].items())
                d = "TX" if rec.get("tx") else "RX"
                src = " <- " + rec["src"] if rec.get("src") else ""
                line = (f"{rec.get('ts', '')}  {d} 0x{dec['id']:03X} "
                        f"{dec['name']:<9} {dec['hex']:<20} | {fields}{src}")
            except Exception:
                line = (f"{rec.get('ts', '')}  {rec.get('id', '?'):#x} "
                        f"{rec.get('data', '')}")
            self.bus_txt.insert("end", line + "\n")
            if rec.get("tx"):
                self.bus_txt.tag_add("tx", "end-%dc" % (len(line) + 1),
                                     "end-1c")
            elif rec.get("changed"):
                self.bus_txt.tag_add("chg", "end-%dc" % (len(line) + 1),
                                     "end-1c")
        self.bus_txt.configure(state="disabled")
        if tail:
            self.bus_txt.see("end")

    def _bus_sb_cmd(self, *args):
        """Scrollbar command: freezes the monitor when dragged upward, resumes
        when dragged to the bottom.  Forwards the real move to the text."""
        if len(args) >= 2 and args[0] == "moveto":
            try:
                frac = float(args[1])
                if frac < self._sb_last_frac and not self._bus_pause:
                    # dragged up -> freeze into scrollback
                    self._bus_pause = True
                    self._bus_frozen_at = None
                    if self._bus_pause_btn is not None:
                        self._bus_pause_btn.configure(text="LIVE >>")
                    self._log("CAN BUS FROZEN - scroll back & click to copy")
                    self._render_bus()
                if self._bus_pause and frac >= 0.999:
                    # dragged to bottom -> resume live
                    self._bus_pause = False
                    self._bus_frozen_at = None
                    if self._bus_pause_btn is not None:
                        self._bus_pause_btn.configure(text="FREEZE")
                    self._log("CAN BUS LIVE")
                    self._render_bus()
                self._sb_last_frac = frac
            except Exception:
                pass
        if self.bus_txt is not None:
            self.bus_txt.yview(*args)

    def _bus_wheel(self, ev):
        """Scroll the bus text.  Scrolling UP auto-pauses (freezes) the live
        stream so you can scroll back and click-copy a fast frame; scrolling
        back down to the bottom resumes live updates."""
        up = False
        down = False
        try:
            num = getattr(ev, "num", None)
            dd = getattr(ev, "delta", 0)
            if num == 4 or (num is None and dd > 0):
                up = True
            elif num == 5 or (num is None and dd < 0):
                down = True
            else:
                if dd:
                    up = dd > 0
                    down = dd < 0
        except Exception:
            up = down = False

        if up and not self._bus_pause:
            # user scrolled up while live -> freeze into a scrollback snapshot
            self._bus_pause = True
            self._bus_frozen_at = None
            if self._bus_pause_btn is not None:
                self._bus_pause_btn.configure(text="LIVE >>")
            self._log("CAN BUS FROZEN - scroll back & click a frame to copy")
            self._render_bus()          # snap the frozen snapshot
            self.bus_txt.yview_scroll(3, "units")   # keep a little context
            return "break"

        # when frozen, scrolling back down to the very bottom resumes live
        if down and self._bus_pause:
            try:
                _at_bottom = self.bus_txt.yview()[1] >= 0.999
            except Exception:
                _at_bottom = False
            if _at_bottom:
                self._bus_pause = False
                self._bus_frozen_at = None
                if self._bus_pause_btn is not None:
                    self._bus_pause_btn.configure(text="FREEZE")
                self._log("CAN BUS LIVE")
                self._render_bus()
                return "break"

        try:
            self.bus_txt.yview_scroll(-3 if up else 3, "units")
        except Exception:
            pass
        return "break"

    def _build_footer(self):
        # Built from plain tk widgets (no ttk): native themes ignore ttk
        # Style settings and painted this footer green-on-light -> unreadable
        f = tk.Frame(self.root, bg="#101418")
        f.grid(row=2, column=0, sticky="ew", padx=8, pady=(0, 6))
        # Contextual drive hint: engine/gear-aware guidance, always visible.
        self.hint_lbl = tk.Label(f, text="", bg="#101418", fg="#9fc7e8",
                                 font=("Consolas", 10, "bold"), anchor="w")
        self.hint_lbl.pack(fill="x", pady=(0, 2))
        # PANELS row: horizontal / vertical band orientation + legend.
        prow = tk.Frame(f, bg="#101418")
        prow.pack(fill="x", pady=(0, 4))
        tk.Label(prow, text="PANELS", bg="#101418", fg="#8b98a5",
                 font=("Consolas", 9), anchor="w").pack(side="left")
        tk.Button(prow, text="H side-by-side", bg="#1c2733", fg="#d7e2ea",
                  activebackground="#0b0d10", activeforeground="#ffffff",
                  highlightthickness=0, relief="flat", padx=8,
                  command=lambda: self._panel_orient("horizontal")).pack(
            side="left", padx=(8, 0))
        tk.Button(prow, text="V stacked", bg="#1c2733", fg="#d7e2ea",
                  activebackground="#0b0d10", activeforeground="#ffffff",
                  highlightthickness=0, relief="flat", padx=8,
                  command=lambda: self._panel_orient("vertical")).pack(
            side="left", padx=(6, 0))
        tk.Label(prow, text="[-] collapse   [M] maximize   R restore",
                 bg="#101418", fg="#5c6a76", font=("Consolas", 8),
                 anchor="w").pack(side="left", padx=(10, 0))

        # Primary drive row: larger + amber so the arrow keys are legible.
        tk.Label(f, text="DRIVE   W/↑ gas   S/↓ brake   A/← steer L   D/→ steer R",
                 bg="#101418", fg="#ffd60a", font=("Consolas", 11),
                 anchor="w").pack(fill="x")
        kb = ("KEYBOARD   P R N D gear   I IGNITION   SPACE parkbrake   "
              "H hazards   C cruise   +/- set   A autopilot   U units   R reset")
        tk.Label(f, text=kb, bg="#101418", fg="#d7e2ea",
                 font=("Consolas", 9), anchor="w").pack(fill="x")
        tk.Label(f, text="Bus broadcast / drive IDs       0x100 ENG   0x110 CHAS   "
                          "0x120 STEER+lights   0x130 BODY   0x140 GEAR/fuel/odo   "
                          "0x400 DRIVE_IN (thr·brk·gear·steer)   diag 0x7DF->0x7E8",
                 bg="#101418", fg="#9fc7e8", font=("Consolas", 9),
                 anchor="w").pack(fill="x")

    # ------------------------------------------------------------- controls
    # Console-inject feedback colours (readable on the light ttk
    # LabelFrame background of the CAN INJECT card).
    INJ_COL = {"hint": "#5a6b78", "ok": "#1e7e34", "fail": "#c0392b",
               "note": "#9a6b00"}

    def _fb_fit(self, ev=None):
        lb = getattr(self, "inject_fb", None)
        if lb is None:
            return
        w = lb.winfo_width()
        if w > 40 and int(lb.cget("wraplength") or 0) != w - 12:
            lb.configure(wraplength=w - 12)

    def _inject_fb(self, kind, text):
        """Update the feedback line under the inject entry."""
        if getattr(self, "inject_fb", None) is not None:
            self.inject_fb.configure(text=text,
                                     fg=self.INJ_COL.get(kind, "#5a6b78"))
        if kind == "fail":
            try:
                self.root.bell()
            except Exception:
                pass

    def _send_inject(self, _ev=None):
        text = self.inject_var.get()
        # Debounce keyboard auto-repeat from a held Enter key.
        if (text == getattr(self, "_inj_last_txt", None)
                and time.time() - getattr(self, "_inj_last_t", 0.0) < 0.35):
            return
        self._inj_last_txt = text
        self._inj_last_t = time.time()
        parsed = parse_cansend(text)
        if parsed is None:
            self._log("INJECT FAIL bad frame: " + text.strip())
            self._inject_fb("fail", "bad frame - use e.g. "
                            "cansend vcan0 400#FF00038000000000")
            return
        can_id, data = parsed
        ok, msg = self.injector.send_cansend(text)
        if not ok:
            self._log("INJECT FAIL " + msg)
            self._inject_fb("fail", msg + " - is the engine running?")
            return
        self._log_tx(can_id, data, src="console")
        self._log("INJECT OK " + msg)
        hexs = data.hex().upper()
        adv = inject_advisory(can_id)
        if adv:
            self._log("INJECT NOTE " + adv)
        if can_id == DRIVE_IN:
            self._inject_fb("note", "0x400 DRIVE_IN sent - the engine "
                            "applies it only while running as follower")
        elif can_id == 0x120:
            self._inject_fb("ok", "%s lamp bits sent - byte1 headlights "
                            "0x10, wipers 0x20, hazard 0x04, highbeam "
                            "0x08, indicators 0x01/0x02 latch the "
                            "switches in any mode" % hex(can_id))
        elif can_id in STATUS_IDS:
            self._inject_fb("note", "%s %s sent - it is an engine status "
                            "broadcast and cannot move the car (see Service "
                            "log)" % (hex(can_id),
                                      FRAME_NAMES.get(can_id, "")))
        else:
            self._inject_fb("ok", "sent %X#%s (%d bytes)"
                            % (can_id, hexs, len(data)))

    def _bind_keys(self):
        self.root.bind("<KeyPress-Up>", lambda e: self._key_thr(1.0))
        self.root.bind("<KeyRelease-Up>", lambda e: self._key_thr(0.0))
        self.root.bind("<KeyPress-w>", lambda e: self._key_thr(1.0))
        self.root.bind("<KeyRelease-w>", lambda e: self._key_thr(0.0))
        self.root.bind("<KeyPress-Down>", lambda e: self._key_brk(1.0))
        self.root.bind("<KeyRelease-Down>", lambda e: self._key_brk(0.0))
        self.root.bind("<KeyPress-s>", lambda e: self._key_brk(1.0))
        self.root.bind("<KeyRelease-s>", lambda e: self._key_brk(0.0))
        self.root.bind("<KeyPress-Left>", lambda e: self._key_steer(-1.0))
        self.root.bind("<KeyRelease-Left>", lambda e: self._key_steer(0.0))
        self.root.bind("<KeyPress-Right>", lambda e: self._key_steer(1.0))
        self.root.bind("<KeyRelease-Right>", lambda e: self._key_steer(0.0))

        # Arrows must steer the car; a focused ttk.Notebook normally
        # grabs <Left>/<Right> to cycle tabs (TNotebook class binding).
        # Bind the arrows on the notebook widget itself (first bindtag)
        # and return "break" so the class binding never flips the tab.
        _nb = getattr(self, "_nb", None)
        if _nb is not None:
            for sym, press, rel in (
                    ("Up", lambda: self._key_thr(1.0), lambda: self._key_thr(0.0)),
                    ("Down", lambda: self._key_brk(1.0), lambda: self._key_brk(0.0)),
                    ("Left", lambda: self._key_steer(-1.0), lambda: self._key_steer(0.0)),
                    ("Right", lambda: self._key_steer(1.0), lambda: self._key_steer(0.0))):
                _nb.bind(f"<KeyPress-{sym}>", lambda e, fn=press: (fn(), "break")[1])
                _nb.bind(f"<KeyRelease-{sym}>", lambda e, fn=rel: (fn(), "break")[1])
        self.root.bind("<KeyPress-i>", lambda e: self._ign(True))
        self.root.bind("<KeyPress-I>", lambda e: self._ign(False))
        for g in "prndPRND":
            self.root.bind(f"<KeyPress-{g}>",
                           lambda e, g=g: self._key_gear(g.upper()))
        self.root.bind("<KeyPress-c>", lambda e: self._cruise_toggle())
        self.root.bind("<KeyPress-C>", lambda e: self._cruise_toggle())
        self.root.bind("<KeyPress-plus>", lambda e: self._cruise_delta(5))
        self.root.bind("<KeyPress-equal>", lambda e: self._cruise_delta(5))
        self.root.bind("<KeyPress-minus>", lambda e: self._cruise_delta(-5))
        self.root.bind("<KeyPress-a>", lambda e: self._ap_toggle())
        self.root.bind("<KeyPress-A>", lambda e: self._ap_toggle())
        self.root.bind("<KeyPress-u>", lambda e: self._toggle_units())
        self.root.bind("<KeyPress-space>", lambda e: self._switch("parkbrake"))
        self.root.bind("<KeyPress-h>", lambda e: self._switch("hazard"))
        self.root.bind("<KeyPress-r>", lambda e: self.client.send({"t": "reset"}))
        self.root.bind("<KeyPress-F1>", lambda e: self._log(
            "keys: W/↑ S/↓ gas/brake, A/← D/→ steer (arrows or WASD), "
            "PRND gear, i ignition, space parkbrake, h hazard, c cruise, "
            "+/- set, a autopilot, u units, r reset   |   CAN BUS tab: "
            "click a frame = copy, double-click = load into inject   |   "
            "RECORD/SAVE/LOAD/REPLAY capture the stream and replay it"))

    def _ign(self, on):
        self.client.send({"t": "ignition", "on": on})
        self.ign_btn.configure(text="START" if not on else "RUNNING")
        self._log_tx(*build_realistic_0x100(
            rpm=800 if on else 0, throttle=0.0),
            src=f"ign {'on' if on else 'off'}")

    def _cruise_toggle(self):
        self.client.send({"t": "cruise", "on": not self._state.get(
            "cruise_on", False)})

    def _cruise_delta(self, d):
        self.client.send({"t": "cruise", "delta": d})

    def _ap_toggle(self):
        self.autopilot = not self.autopilot
        self.ap_btn.configure(text="AUTOPILOT ON" if self.autopilot else "OFF")
        if self.autopilot:
            self._ap_t0 = time.monotonic()
            st = self._state
            if st.get("engine_on") and st.get("gear") == "D":
                self._ap_phase = 3
                self._log("autopilot ON - already driving")
            elif st.get("engine_on"):
                self._ap_phase = 2
                self._log("autopilot ON - engaging D")
            else:
                self._ap_phase = 1
                self._log("autopilot ON - engine start sequence")
        else:
            self._ap_phase = 0
            self.client.send({"t": "input", "throttle": 0.0, "brake": 0.0,
                              "steer": 0.0})
            self._log("autopilot OFF")

    def _switch(self, name):
        cur = self.sw_checks.get(name)
        if cur is not None:
            cur.set(not cur.get())
            self.client.send({"t": "switch", "name": name, "on": cur.get()})
            self._switch_tx(name, cur.get())

    def _switch3(self, name, var):
        self.client.send({"t": "switch", "name": name, "on": var.get()})
        self._switch_tx(name, var.get())

    def _switch_tx(self, name, on):
        """Synthetic TX for the broadcast frame that carries this switch."""
        body_names = ("door_fl", "door_fr", "door_rl", "door_rr")
        if name in body_names or name in ("trunk", "hood", "seatbelt"):
            bits = 0x00
            for i, n in enumerate(body_names):
                if n in self.sw_checks and self.sw_checks[n].get():
                    bits |= 1 << i
            for n, m in (("trunk", 0x10), ("hood", 0x20), ("seatbelt", 0x40)):
                if n in self.sw_checks and self.sw_checks[n].get():
                    bits |= m
            self._log_tx(0x130, bytes([bits, 0, 0, 0, 0, 0, 0, 0]),
                         src=f"sw {name} {'ON' if on else 'OFF'}")
            return
        if name == "parkbrake":
            self._log_tx(*build_realistic_0x110(
                speed_kmh=float(self._state.get("speed", 0.0)),
                brake_pct=100 if on else 0,
                flags=0x08 if on else 0),
                src=f"sw {name} {'ON' if on else 'OFF'}")
            return
        lamps = {"headlights": 0x10, "wipers": 0x20, "hazard": 0x04}.get(
            name, 0x01)
        self._log_tx(*build_realistic_0x120(
            steer=float(self._state.get("steer", 0.0)) / 100.0,
            lamps=lamps if on else 0),
            src=f"sw {name} {'ON' if on else 'OFF'}")

    def _fault3(self, name, var):
        self.client.send({"t": "fault", "name": name, "on": var.get()})
        self._fault_tx(name, var.get())

    def _fault_tx(self, name, on):
        """Synthetic diagnostic frame for the CAN BUS monitor, mirroring
        byte-for-byte the sim's OBD answer to a scan read right after this
        fault change (see sim handle_request() / handle_rx_frame() sf path:
        ISO-TP single-frame PCI prefix + zero pad to 8).

          mil/overheat/flat -> mode 03 stored-DTC list, engine ECU 0x7E8
          abs               -> UDS 19 02 report-by-status, ABS ECU 0x7EA

        flat() has no DTC (FAULT_DTCS['flat'] == []) so its frame re-reports
        the current engine code list - the toggle still shows up as a TX row.
        Engine codes use the sim's own iteration order (mil then overheat)."""
        f = self.fl_checks
        if name == "abs":
            body = (b"\x59\x02\xFF\x40\x35\x08" if (on or f["abs"].get())
                    else b"\x59\x02\xFF")          # C0035 / none stored
            fid = 0x7EA
        else:
            order = ("mil", "overheat")              # mirrors engine_dtcs()
            hilo = {"mil": (0x03, 0x00),             # P0300
                    "overheat": (0x02, 0x17)}        # P0217
            active = [n for n in order
                      if (f[n].get() if n != name else on)]
            pairs = b"".join(bytes(hilo[n]) for n in active)
            body = bytes([0x43, len(active)]) + pairs
            fid = 0x7E8
        frm = bytes([len(body)]) + body              # ISO-TP single frame
        self._log_tx(fid, frm + b"\x00" * (8 - len(frm)),
                     src=f"fault {name} {'SET' if on else 'CLEAR'}")

    def _clear_faults(self):
        had_eng = had_abs = False
        for name, v in self.fl_checks.items():
            if v.get():
                had_eng |= name != "abs"
                had_abs |= name == "abs"
            v.set(False)
            self.client.send({"t": "fault", "name": name, "on": False})
        if had_eng:                                  # mode 03: 0 DTCs stored
            frm = b"\x02\x43\x00"
            self._log_tx(0x7E8, frm + b"\x00" * (8 - len(frm)),
                         src="fault clear-all (engine 0 DTCs)")
        if had_abs:                                  # UDS 19 02: none stored
            frm = b"\x03\x59\x02\xFF"
            self._log_tx(0x7EA, frm + b"\x00" * (8 - len(frm)),
                         src="fault clear-all (ABS 0 DTCs)")
        self._log("all faults cleared")

    def _toggle_units(self):
        self.units = "mph" if self.units_var.get() == "mph" else "kmh"

    def _key_drive(self, thr, brk, steer, src):
        """Send one keyboard drive command per *real* change.

        Holding a key fires OS auto-repeat events (plus the final release),
        which used to log 10+ identical TX lines per second into the CAN BUS
        monitor.  Sliders still track every event; the bus gets a single TX
        per actual change (press / release / new value).
        """
        cmd = (thr, brk, steer)
        if cmd == getattr(self, "_last_key_drive", None):
            return
        self._last_key_drive = cmd
        self.client.send({"t": "input", "throttle": thr, "brake": brk,
                          "steer": steer})
        self._log_tx(*build_drive(throttle=thr, brake=brk, steer=steer,
                                  gear=self._state.get("gear", "D")),
                     src=src)

    def _key_thr(self, v):
        self.thr_var.set(v * 100.0)
        self._key_drive(v, self.brk_var.get() / 100.0,
                        self.steer_var.get() / 100.0, f"key thr {v:.0f}")

    def _key_brk(self, v):
        self.brk_var.set(v * 100.0)
        self._key_drive(self.thr_var.get() / 100.0, v,
                        self.steer_var.get() / 100.0, f"key brk {v:.0f}")

    def _key_steer(self, v):
        self.steer_var.set(v * 100.0)
        self._key_drive(self.thr_var.get() / 100.0,
                        self.brk_var.get() / 100.0, v, f"key steer {v:.0f}")

    def _key_gear(self, g):
        self.client.send({"t": "gear", "gear": g})
        self._log_tx(*build_drive(gear=g), src=f"key gear {g}")

    # ------------------------------------------------------------ autopilot
    def _autopilot(self):
        st = self._state
        gear = st.get("gear", "P")
        engine = st.get("engine_on", False)
        speed = abs(st.get("speed", 0.0))
        if self._ap_phase == 1:
            if gear in ("P", "N"):
                self.client.send({"t": "ignition", "on": True})
                self._ap_phase = 2
                self._log("cranking in %s..." % gear)
            else:
                if speed > 1.0:
                    self._log("autopilot: shifting P (waiting for stop)")
                self.client.send({"t": "gear", "gear": "P"})
                self.client.send({"t": "input", "throttle": 0.0,
                                  "brake": 0.3, "steer": 0.0})
            return
        if self._ap_phase == 2:
            if gear not in ("P", "N"):
                self.client.send({"t": "gear", "gear": "P"})
            if not engine:
                self.client.send({"t": "ignition", "on": True})
                return
            self.client.send({"t": "gear", "gear": "D"})
            self._ap_phase = 3
            self._log("engine started - engaging D")
            return
        if not engine:
            self._ap_phase = 1
            self._log("engine stalled - restarting")
            return
        target = self.ap_target
        err = target - speed
        thr = 0.0 if err < 0 else min(0.85, 0.03 + err * 0.012)
        brk = min(1.0, max(0.0, (speed - target) * 0.06)) if speed > target + 2 else 0.0
        t = time.monotonic() - self._ap_t0
        steer = 0.14 * math.sin(t * 0.7)
        self.thr_var.set(thr * 100.0)
        self.brk_var.set(brk * 100.0)
        self.steer_var.set(steer * 100.0)
        self.client.send({"t": "input", "throttle": thr, "brake": brk,
                          "steer": steer})

    # --------------------------------------------------------------- helpers
    def _spd(self, kmh):
        return kmh * 0.6213712 if self.units == "mph" else kmh

    def _spdu(self):
        return "mph" if self.units == "mph" else "km/h"

    # --------------------------------------------------------------- render
    def _tick(self):
        now = time.monotonic()
        self._render()
        self._drain_events()
        self._update_hint()
        if self.autopilot and (now - self._ap_last) > 0.1:
            self._ap_last = now
            self._autopilot()
        self.root.after(100, self._tick)

    def _render(self):
        st = self._state
        self.status_lbl.configure(
            text=f"status: {self.client.status}")
        # Truthful START/RUNNING: _ign() sets the label optimistically, but
        # the sim may reject the crank (engine off while in D, etc.), so the
        # label is driven from state.engine_on every frame.
        self.ign_btn.configure(
            text="RUNNING" if st.get("engine_on") else "START")
        self.mode_lbl.configure(text=st.get("mode", "-"))
        self.ts_lbl.configure(text=f"{st.get('ts', 0.0):.1f} s")
        self.odo_lbl.configure(text=f"{st.get('odo', 0.0):.1f} km")
        self.source_lbl.configure(
            text="drive: " + st.get("source", "keyboard"))
        self.cruise_lbl.configure(text=("on" if st.get("cruise_on") else "off"))
        self.dtc_lbl.configure(text=self._dtc_text(st))
        self._render_road(st)
        self._render_cluster(st)
        self._render_lamps(st)
        self._render_live(st)
        self._render_bus()

    def _render_bus(self):
        """Live CAN BUS monitor.  When FROZEN it snaps a scrollback snapshot and
        stops re-rendering so you can scroll back and click-copy a frame."""
        if not getattr(self, "bus_txt", None):
            return
        if self._bus_pause:
            if self._bus_frozen_at is None:
                self._bus_frozen_at = self._bus_seq
                self._render_bus_lines(list(self.frame_history[-120:]), tail=False)
            return
        self._bus_frozen_at = None
        seq = getattr(self, "_bus_seq", 0)
        if seq == getattr(self, "_bus_rendered", -1):
            return
        self._bus_rendered = seq
        self._render_bus_lines(self.frame_history[-36:], tail=True)

    # ------------------------------------------------- CAN BUS capture + recorder
    def _bus_line_rec(self, index):
        try:
            ln = int(str(index).split(".")[0])
        except Exception:
            return None
        view = getattr(self, "_bus_view", [])
        if 1 <= ln <= len(view):
            return view[ln - 1]
        return None

    def _clipline(self, line):
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(line)
        except Exception:
            pass

    def _bus_highlight(self, ev):
        try:
            idx = self.bus_txt.index("@%d,%d" % (ev.x, ev.y))
            ln = idx.split(".")[0]
            self.bus_txt.tag_remove("capture", "1.0", "end")
            self.bus_txt.tag_add("capture", ln + ".0", ln + ".end")
        except Exception:
            pass

    def _bus_capture(self, ev):
        """Single click: copy the frame under the cursor to the clipboard."""
        rec = self._bus_line_rec(self.bus_txt.index("@%d,%d" % (ev.x, ev.y)))
        if not rec:
            return
        line = capture_cansend(rec.get("id"), rec.get("data"))
        if not line:
            return
        self._clipline(line)
        self._bus_highlight(ev)
        self._log("copied " + line)
        if self._cap_lbl is not None:
            self._cap_lbl.configure(text="COPIED  " + line)

    def _bus_load(self, ev):
        """Double click: copy AND load the frame into the CAN INJECT box."""
        rec = self._bus_line_rec(self.bus_txt.index("@%d,%d" % (ev.x, ev.y)))
        if not rec:
            return
        line = capture_cansend(rec.get("id"), rec.get("data"))
        if not line:
            return
        self._clipline(line)
        self.inject_var.set(line)
        self._bus_highlight(ev)
        self._log("inject <- " + line)
        if self._cap_lbl is not None:
            self._cap_lbl.configure(text="INJECT <-   " + line)

    def _set_scope(self, v):
        self._rec_scope = v

    def _update_rec_cnt(self):
        if self._rec_cnt_lbl is not None:
            self._rec_cnt_lbl.configure(text="%d frames" % len(self._rec_frames))

    def _rec_toggle(self):
        self._rec_on = not self._rec_on
        if self._rec_on:
            self._rec_frames = []
            self._rec_t0 = None
            self._rec_last = {}
            self._rx_delta = bool(self._rxdelta_var.get())
            self._rec_filter = self._idfilt_var.get()
            if self._rec_btn is not None:
                self._rec_btn.configure(text="STOP")
            self._log("record: ON   scope=%s  filter='%s'  changed-only=%s"
                      % (self._rec_scope, self._rec_filter, self._rx_delta))
        else:
            if self._rec_btn is not None:
                self._rec_btn.configure(text="\u25cf RECORD")
            self._log("record: OFF   %d frames" % len(self._rec_frames))
        self._update_rec_cnt()

    def _rec_save(self):
        if not self._rec_frames:
            self._log("save: nothing recorded")
            return
        if filedialog is None:
            self._log("save: no file dialog (tk unavailable)")
            return
        path = filedialog.asksaveasfilename(
            title="Save CAN frame dump", defaultextension=".txt",
            initialfile="can_dump_" + time.strftime("%H%M%S") + ".txt",
            filetypes=[("CAN frame dump", "*.txt"), ("All files", "*.*")])
        if not path:
            return
        lines = []
        for it in self._rec_frames:
            sec = it.get("ms", 0) / 1000.0
            src = it.get("src", "")
            tail = ("  # %s" % src) if src else ""
            lines.append("%.3f  %s  %s  %s%s" % (
                sec, it.get("dir", "RX"), fmt_can_id(it.get("id")),
                (it.get("data") or ""), tail))
        header = "# carsim traffic dump  scope=%s  frames=%d  %s\n" % (
            self._rec_scope, len(self._rec_frames), time.strftime("%H:%M:%S"))
        cansend = "# cansend lines (paste into CAN INJECT / cansend):\n" \
            + rec_to_cansend(self._rec_frames) + "\n"
        try:
            with open(path, "w") as f:
                f.write(header)
                f.write("\n".join(lines) + "\n\n")
                f.write(cansend)
        except OSError as e:
            self._log("save FAIL: %s" % e)
            return
        self._log("save: %d frames -> %s"
                  % (len(self._rec_frames), os.path.basename(path)))

    def _rec_load(self):
        if filedialog is None:
            self._log("load: no file dialog (tk unavailable)")
            return
        path = filedialog.askopenfilename(
            title="Load CAN frame dump",
            filetypes=[("CAN frame dump", "*.txt"), ("All files", "*.*")])
        if not path:
            return
        try:
            txt = open(path).read()
        except OSError as e:
            self._log("load FAIL: %s" % e)
            return
        items = parse_capture_text(txt)
        if not items:
            self._log("load: no frames parsed from %s" % os.path.basename(path))
            return
        for it in items:
            it["ms"] = int(round(it.get("sec", 0.0) * 1000.0))
        self._rec_frames = items
        self._log("load: %d frames from %s"
                  % (len(items), os.path.basename(path)))
        self._update_rec_cnt()

    def _q_send_raw(self, fid, data, src):
        if self.injector.send_frame(fid, data):
            self._log("TX %s#%s  <%s>" % (fmt_can_id(fid),
                                          data.hex().upper(), src))
            self._log_tx(fid, data, src=src)
        else:
            self._log("replay FAIL: injector offline")
            self._replay_stop()

    def _replay_toggle(self):
        if self._rp_job is not None and not self._rp_stop:
            self._replay_stop()
            return
        if not self._rec_frames:
            self._log("replay: nothing queued (record or LOAD first)")
            return
        self._rp_items = list(self._rec_frames)
        self._rp_i = 0
        self._rp_t0 = time.monotonic()
        self._rp_stop = False
        if self._rp_btn is not None:
            self._rp_btn.configure(text="STOP")
        self._log("replay: %d frames" % len(self._rp_items))
        self._replay_tick()

    def _replay_tick(self):
        if self._rp_stop:
            return
        items = self._rp_items
        if self._rp_i >= len(items):
            self._replay_finish()
            return
        item = items[self._rp_i]
        base = items[0]["ms"] if items else 0
        target = item["ms"] - base
        now_ms = int((time.monotonic() - self._rp_t0) * 1000.0)
        if now_ms < target:
            self._rp_job = self.root.after(min(target - now_ms, 10000),
                                          self._replay_tick)
            return
        self._rp_i += 1
        data = bytes.fromhex(item.get("data", "") or "00")
        self._q_send_raw(item.get("id"), data, "replay")
        self._rp_job = self.root.after(5, self._replay_tick)

    def _replay_stop(self):
        self._rp_stop = True
        if self._rp_job is not None:
            try:
                self.root.after_cancel(self._rp_job)
            except Exception:
                pass
            self._rp_job = None
        if self._rp_btn is not None:
            self._rp_btn.configure(text="REPLAY")
        self._log("replay stopped (%d frames)" % self._rp_i)

    def _replay_finish(self):
        self._rp_job = None
        self._rp_stop = False
        if self._rp_btn is not None:
            self._rp_btn.configure(text="REPLAY")
        self._log("replay done (%d frames)" % self._rp_i)

    def _dtc_text(self, st):
        parts = []
        dtc = st.get("dtc") or []
        dtc_abs = st.get("dtc_abs") or []
        if dtc:
            parts.append("ENG: " + " ".join(dtc))
        if dtc_abs:
            parts.append("ABS: " + " ".join(dtc_abs))
        if st.get("faults", {}).get("flat"):
            parts.append("flat tyre limiter")
        return "\n".join(parts) if parts else "no active DTCs"

    # ------------------------------------------------------------ road scene
    def _render_road(self, st):
        cv = self.road_cv
        cv.delete("all")
        w = int(cv["width"])
        h = int(cv["height"])
        speed_kmh = abs(st.get("speed", 0.0))
        self._scroll += speed_kmh / 3.6 * 0.1
        steer = st.get("steer", 0.0) / 100.0
        # road stays fixed; the CAR moves laterally within it.  Steering is
        # INTEGRATED, so the car changes lane while the key is held and HOLDS
        # its lane when the key is released (no auto-recenter to the middle).
        self._car_lat += steer * getattr(self, "_lat_rate", 12.0)
        self._car_lat = max(-90.0, min(90.0, self._car_lat))
        road_center = w / 2.0
        road_w = 300.0
        cv.create_rectangle(road_center - road_w / 2, 0, road_center + road_w / 2,
                            h, fill="#1a1a1a", outline="")
        cv.create_line(road_center - road_w / 2, 0, road_center - road_w / 2, h,
                       fill="#555", width=3)
        cv.create_line(road_center + road_w / 2, 0, road_center + road_w / 2, h,
                       fill="#555", width=3)
        spacing = 60
        for k in range(-2, (h // spacing) + 3):
            yy = (k * spacing + self._scroll) % h
            cv.create_line(road_center, yy - spacing / 2, road_center,
                           yy + spacing / 2, fill="#e8c93a", width=4)
        cx = road_center + self._car_lat
        cy = h * 0.78
        car_w, car_h = 74, 128
        sw = st.get("switches", {})
        if sw.get("headlights"):
            cv.create_polygon(cx - car_w * 0.42, cy - car_h * 0.5,
                              cx - car_w * 0.42, cy - car_h * 0.5 - 90,
                              cx + car_w * 0.42, cy - car_h * 0.5 - 90,
                              cx + car_w * 0.42, cy - car_h * 0.5,
                              fill="#fff3b0", stipple="gray50", outline="")
        cv.create_rectangle(cx - car_w / 2, cy - car_h / 2, cx + car_w / 2,
                            cy + car_h / 2, fill="#2d3f66", outline="#0a0a0a",
                            width=2)
        cv.create_rectangle(cx - car_w / 2 + 6, cy - car_h * 0.30,
                            cx + car_w / 2 - 6, cy - car_h * 0.05,
                            fill="#1b2a45", outline="#111")
        cv.create_rectangle(cx - car_w / 2 + 4, cy - car_h * 0.05,
                            cx + car_w / 2 - 4, cy + car_h * 0.28,
                            fill="#26385c", outline="#111")
        wl, ww = car_w + 8, 14
        for wy in (cy - car_h * 0.30, cy + car_h * 0.18):
            cv.create_rectangle(cx - wl / 2, wy, cx - wl / 2 + ww, wy + 20,
                                fill="#0a0a0a")
            cv.create_rectangle(cx + wl / 2 - ww, wy, cx + wl / 2, wy + 20,
                                fill="#0a0a0a")
        if st.get("brake", 0.0) > 0.05 or sw.get("parkbrake"):
            cv.create_rectangle(cx - car_w / 2 - 4, cy - car_h * 0.02,
                                cx - car_w / 2 + 6, cy + car_h * 0.10,
                                fill="#ff3b30", outline="")
            cv.create_rectangle(cx + car_w / 2 - 6, cy - car_h * 0.02,
                                cx + car_w / 2 + 4, cy + car_h * 0.10,
                                fill="#ff3b30", outline="")
        if sw.get("hazard") or sw.get("left") or sw.get("right"):
            blink = int(self._state.get("ts", 0.0) * 2.0) % 2 == 0
            if blink and (sw.get("hazard") or sw.get("left")):
                cv.create_rectangle(cx - car_w / 2 - 4, cy - car_h * 0.28,
                                    cx - car_w / 2 + 4, cy - car_h * 0.16,
                                    fill="#ff9f0a", outline="")
            if blink and (sw.get("hazard") or sw.get("right")):
                cv.create_rectangle(cx + car_w / 2 - 4, cy - car_h * 0.28,
                                    cx + car_w / 2 + 4, cy - car_h * 0.16,
                                    fill="#ff9f0a", outline="")
        if sw.get("wipers"):
            cv.create_line(cx, cy - car_h * 0.30, cx - car_w * 0.5,
                           cy - car_h * 0.14, fill="#ccf", width=2)
        gear = st.get("gear", "P")
        cv.create_text(cx, cy - car_h * 0.34, text=gear, fill="#e8c93a",
                       font=("Helvetica", 26, "bold"))
        cv.create_text(10, 16, anchor="w",
                       text=f"{self._spd(speed_kmh):.0f} {self._spdu()}",
                       fill="#e8e8e8", font=("Helvetica", 16, "bold"))

    # ------------------------------------------------------------ cluster
    def _render_cluster(self, st):
        cv = self.cluster_cv
        cv.delete("all")
        w, h = int(cv["width"]), int(cv["height"])
        cv.create_rectangle(0, 0, w, h, fill="#101010", outline="")
        spd = abs(st.get("speed", 0.0))
        spd_disp = self._spd(spd)
        spd_max = 240.0 if self.units == "kmh" else 150.0
        rpm = st.get("rpm", 0.0)
        coolant = st.get("coolant", 40.0)
        fuel = st.get("fuel", 0.0)
        load = st.get("load", 0.0)
        voltage = st.get("voltage", 12.4)
        odo = st.get("odo", 0.0)
        trip = st.get("trip", 0.0)
        maf = st.get("maf", 0.0)
        runmin = st.get("runtime", 0.0) / 60.0
        # 3x3 tiled gauges, evenly spaced, NO overlap; caption BELOW dial
        ncol, nrow = 3, 3
        gx0, gy0 = w / (ncol * 2), 50.0
        gx_step, gy_step = w / ncol, 130.0
        rad = min(gx_step, gy_step) * 0.28
        cfg = [
            ("RPM", rpm, 6400, "#ff6b3d", "rpm"),
            ("SPEED", spd_disp, spd_max, "#34d1ce", self._spdu()),
            ("COOL", max(0, coolant - 40), 120, "#ff9f0a", "C"),
            ("VOLT", voltage, 16.0, "#7bd17b", "V"),
            ("FUEL", fuel, 100.0, "#7bd17b", "%"),
            ("LOAD", load, 100.0, "#ffd60a", "%"),
            ("OIL", st.get("oil", 40.0) - 40, 160, "#ffd60a", "C"),
            ("MAF", maf, 30.0, "#34d1ce", "g/s"),
            ("TRIP", trip, 900.0, "#e0a0ff", "km"),
        ]
        for idx, (label, val, vmax, col, unit) in enumerate(cfg):
            row, col_i = divmod(idx, ncol)
            cx = gx0 + col_i * gx_step
            cy = gy0 + row * gy_step
            _draw_gauge(cv, cx, cy, rad, val, vmax, label, col, units=unit)
        # odo / trip / run on a clean bottom band (never overlaps dials)
        cv.create_text(w / 2, h - 6, anchor="s",
                       text=f"ODO {odo:06.0f} km   {self._spdu()}: "
                            f"{self._spd(spd):.0f}   trip {trip:.1f} km   "
                            f"run {runmin:.0f} min",
                       fill="#e8e8e8", font=("Helvetica", 11, "bold"))

    def _render_lamps(self, st):
        cv = self.lamps_cv
        cv.delete("all")
        w = int(cv["width"])
        lamps = st.get("lamps", {})
        sw = st.get("switches", {})
        flag = []
        def add(name, col):
            flag.append((name, col))
        if lamps.get("mil"):
            add(("MIL"), "#ff5a3c")
        if lamps.get("abs"):
            add(("ABS"), "#ff9f0a")
        if lamps.get("tc"):
            add(("TC"), "#ffd60a")
        if lamps.get("battery"):
            add(("BATT"), "#ff3b30")
        if lamps.get("seatbelt"):
            add(("BELT"), "#ff3b30")
        if lamps.get("lowfuel"):
            add(("FUEL"), "#ffd60a")
        if lamps.get("door"):
            add(("DOOR"), "#ff9f0a")
        if lamps.get("parkbrake"):
            add(("P!"), "#ff3b30")
        if sw.get("highbeam"):
            add(("HI"), "#7db8ff")
        if sw.get("hazard"):
            add(("HZ"), "#ff9f0a")
        if st.get("rev_limit"):
            add(("REV"), "#ff3b30")
        if st.get("limp"):
            add(("LIMP"), "#ff9f0a")
        if not flag:
            cv.create_text(w / 2, 24, text="ALL OK", fill="#e8e8e8",
                           font=("Helvetica", 11, "bold"))
            return
        n = len(flag)
        step = min(72, (w - 20) / max(1, n))
        x = 20
        for name, col in flag:
            cv.create_oval(x - 13, 10, x + 13, 36, outline=col, width=2)
            cv.create_text(x, 23, text=name, fill=col, font=("Helvetica", 7, "bold"))
            x += step

    def _render_live(self, st):
        s = st.get("source", "")
        lines = [
            f"source            : {s}",
            f"Engine RPM        : {st.get('rpm', 0):.0f}",
            f"Speed             : {self._spd(abs(st.get('speed', 0))):.1f} {self._spdu()}",
            f"Coolant           : {st.get('coolant', 0):.0f} C",
            f"Intake air        : {st.get('intake', 0):.0f} C",
            f"Throttle pos      : {st.get('throttle', 0):.0f} %",
            f"Brake             : {st.get('brake', 0):.0f} %",
            f"Fuel level        : {st.get('fuel', 0):.0f} %",
            f"Engine load       : {st.get('load', 0):.0f} %",
            f"MAF               : {st.get('maf', 0):.1f} g/s",
            f"Voltage           : {st.get('voltage', 0):.1f} V",
            f"Oil temp          : {st.get('oil', 0):.0f} C",
            f"Gear              : {st.get('gear', 'P')}  {st.get('gear_num', 0)}",
            f"Odometer          : {st.get('odo', 0):.1f} km",
        ]
        blob = "\n".join(lines)
        if blob != self._last_live:
            self._last_live = blob
            self.live_txt.delete("1.0", "end")
            self.live_txt.insert("1.0", blob)


# --------------------------------------------------------------------------- #
#  Entry points
# --------------------------------------------------------------------------- #
def _headless_check():
    """No-display sanity check of decode/inject/mapping helpers."""
    checks = []

    def enc(fid, data):
        return {"id": fid, "dlc": len(data), "data": data.hex().upper()}

    # ---- zero-length / malformed guard
    d = bytes([0x04, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])   # 0x100 rpm 0x040E = 1038
    dec = decode_frame(enc(0x100, d))
    checks.append(("0x100 rpm=1038", dec["fields"]["RPM"] == "1038 rpm"))

    # ---- 0x400 decode round-trip
    fid, data = build_drive(throttle=1.0, brake=0.5, gear="D", steer=1.0)
    checks.append(("build_drive id", fid == 0x400))
    checks.append(("build_drive thr", data[0] == 255))
    checks.append(("build_drive brk", data[1] == 50))
    checks.append(("build_drive gear", data[2] == 3))
    checks.append(("build_drive steer", data[3] == 255))
    dec = decode_frame(enc(fid, data))
    checks.append(("0x400 decode thr", dec["fields"]["thr"] == "100%"))
    checks.append(("0x400 decode brk", dec["fields"]["brk"] == "50%"))
    checks.append(("0x400 decode gear", dec["fields"]["gear"] == "D"))

    # ---- realistic frames
    fid, d = build_realistic_0x100(rpm=4200, throttle=0.2, coolant=90, oil=100,
                                   maf_gps=12.34)
    checks.append(("0x100 builder id", fid == 0x100))
    checks.append(("0x100 builder rpm", (d[0] << 8 | d[1]) == 4200))
    checks.append(("0x100 builder throttle", d[3] == 51))
    checks.append(("0x100 builder coolant", d[4] == 130))
    fid, d = build_realistic_0x110(speed_kmh=72, brake_pct=55, flags=0x03)
    checks.append(("0x110 builder", d[0] == 72 and d[1] == 55 and d[2] == 0x03))
    fid, d = build_realistic_0x120(steer=-0.3, lamps=0x19)
    checks.append(("0x120 builder", d[0] == 0xE2 and d[1] == 0x19))
    fid, d = build_realistic_0x140(gear="D", fuel_pct=50, odo_km=123.456,
                                   runtime_min=5)
    odo = d[2] | d[3] << 8 | d[4] << 16 | d[5] << 24
    checks.append(("0x140 builder odo", odo == 123456))

    # ---- cansend parser (exactly what can-utils types)
    p = parse_cansend("cansend vcan0 400#FF00038000000000")
    checks.append(("parse cansend id", p is not None and p[0] == 0x400))
    checks.append(("parse cansend data", p is not None and p[1].hex() ==
                   "ff00038000000000"))
    p2 = parse_cansend("cansend vcan0 100#28024463C0000900")
    checks.append(("parse 0x100 cansend", p2 is not None and p2[0] == 0x100))
    checks.append(("parse rejects junk", parse_cansend("hello world") is None))
    checks.append(("parse rejects >8", parse_cansend("400#112233445566778899") is None))
    p3 = parse_cansend("110#4700000000000000")
    checks.append(("parse 110#47.. bare", p3 is not None and p3[0] == 0x110
                   and p3[1].hex() == "4700000000000000"))
    p4 = parse_cansend("cansend vcan0 110#4700000000000000")
    checks.append(("parse 110#47.. cansend", p4 is not None and p4[0] == 0x110
                   and p4[1].hex() == "4700000000000000"))

    # ---- frame-semantics advisory (v0.6.2)
    checks.append(("advisory 0x400", "DRIVE_IN" in (inject_advisory(0x400) or "")))
    checks.append(("advisory status",
                   "0x110" in (inject_advisory(0x110) or "") and
                   "status" in (inject_advisory(0x110) or "")))
    checks.append(("advisory body",
                   "0x130" in (inject_advisory(0x130) or "")))
    checks.append(("advisory diag none", inject_advisory(0x7E8) is None))


    # ---- gauge geometry
    checks.append(("gauge_angle min", abs(gauge_angle(0, 240) - 135.0) < 1e-6))
    checks.append(("gauge_angle max", abs(gauge_angle(240, 240) - 405.0) < 1e-6))
    pts = arc_points(100, 100, 80, 135, 405, 48)
    checks.append(("arc_points", len(pts) == 2 * (48 + 1)))

    # ---- v0.6 capture + recorder helpers
    s = slcan_tx_line(0x400, bytes.fromhex("FF00038000000000"))
    checks.append(("slcan 11-bit", s == "t4008FF00038000000000"))
    s = slcan_tx_line(0x18DA10F1, bytes.fromhex("0601040100000000"))
    checks.append(("slcan 29-bit", s == "T18DA10F180601040100000000"))
    cc = capture_cansend(0x100, "28024463C0000900")
    checks.append(("capture cansend", cc == "cansend vcan0 100#28024463C0000900"))
    cc = capture_cansend(0x400, "FF00038000000000")
    checks.append(("capture 0x400", cc == "cansend vcan0 400#FF00038000000000"))
    checks.append(("capture rejects bad hex", capture_cansend(0x100, "ZZ") is None))
    items = parse_capture_text(
        "# comment\n0.01  RX  100  28024463C0000900\n"
        "0.02  TX  400  FF00038000000000\njunk\n")
    checks.append(("parse dump len", len(items) == 2))
    checks.append(("parse dump id0", items[0]["id"] == 0x100 and
                   items[0]["dir"] == "RX"))
    checks.append(("parse dump id1", items[1]["id"] == 0x400 and
                   items[1]["dir"] == "TX"))
    c = rec_to_cansend(items)
    checks.append(("rec_to_cansend", "400#FF00038000000000" in c and
                   "100#28024463C0000900" in c))

    fail = 0
    for name, ok in checks:
        print(("  ok " if ok else "  FAIL ") + name)
        if not ok:
            fail += 1
    if fail:
        print(f"carsim_gui headless check: {fail} FAILED")
        return 1
    print("carsim_gui headless check: ALL PASS")
    return 0


def _is_loopback_host(host):
    """True when host names this machine's own loopback."""
    return host in ("", "localhost", "127.0.0.1", "::1")


def _port_open(host, port, timeout=0.3):
    """True when something accepts TCP connections on host:port."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _engine_log_path():
    """Where the auto-started engine's console output is appended."""
    return os.path.join(tempfile.gettempdir(), "carsim_engine.log")


def _spawn_engine(ctrl_port):
    """Start the bundled carsim.py (--follower) bound to loopback.

    The engine's stdout/stderr are appended to _engine_log_path() so a
    cockpit launched from a desktop launcher still leaves a diagnosable
    trail.  Returns the Popen handle, or None when the engine could not
    be started.  The caller must ensure _stop_engine() runs on exit.
    """
    global _ENGINE_PROC
    engine = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "carsim.py")
    try:
        log_fd = os.open(_engine_log_path(),
                         os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    except OSError:
        log_fd = os.open(os.devnull, os.O_WRONLY)
    try:
        proc = subprocess.Popen(
            [sys.executable, "-u", engine,
             "--ctrl-port", str(ctrl_port),
             "--follower"],
            stdin=subprocess.DEVNULL, stdout=log_fd,
            stderr=subprocess.STDOUT)
    except OSError as exc:
        os.close(log_fd)
        print(f"could not auto-start the engine: {exc}", file=sys.stderr)
        return None
    os.close(log_fd)          # the child already holds its own copy
    _ENGINE_PROC = proc
    return proc


def _wait_port(host, port, proc=None, timeout=5.0):
    """Poll host:port until it accepts connections.

    Gives up early (False) when proc exits before the port opens.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc is not None and proc.poll() is not None:
            return False
        if _port_open(host, port):
            return True
        time.sleep(0.05)
    return False


def _stop_engine(proc=None):
    """Tear down an auto-started engine (SIGTERM, escalate to SIGKILL)."""
    if proc is None:
        proc = _ENGINE_PROC
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass



def main():
    ap = argparse.ArgumentParser(description="Cockpit GUI for carsim.py")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help=f"carsim host (default {DEFAULT_HOST})")
    ap.add_argument("--port", type=int, default=CTRL_PORT,
                    help=f"JSON control port (default {CTRL_PORT})")
    ap.add_argument("--check", action="store_true",
                    help="run headless decode/inject checks and exit")
    args = ap.parse_args()

    if args.check:
        sys.exit(_headless_check())

    if not _HAS_TK:
        print("tkinter not available in this environment", file=sys.stderr)
        return 1

    # Single-entry cockpit (v0.9.10):
    # not yet listening, auto-start the bundled engine as --follower so a
    # bare `python3 tools/carsim_gui.py` brings up the whole bench.  A sim
    # that is already running on the ctrl port is left completely untouched.
    atexit.register(_stop_engine)
    if _is_loopback_host(args.host):
        if not _port_open("127.0.0.1", args.port):
            engine = _spawn_engine(args.port)
            if engine is None:
                print("could not auto-start the engine; the cockpit will keep "
                      "retrying the connection", file=sys.stderr)
            elif not _wait_port("127.0.0.1", args.port, proc=engine):
                exited = engine.poll() is not None
                _stop_engine(engine)
                if exited:
                    print("auto-started engine exited early - see "
                          f"{_engine_log_path()}", file=sys.stderr)
                else:
                    print(f"auto-started engine never opened "
                          f"127.0.0.1:{args.port} - see "
                          f"{_engine_log_path()}", file=sys.stderr)
                return 1
            else:
                print(f"engine auto-started (ctrl 127.0.0.1:{args.port}); "
                      f"log in {_engine_log_path()}")
        else:
            print(f"sim already listening on 127.0.0.1:{args.port} - "
                  "reusing it (engine not started)")

    try:
        root = tk.Tk()
    except Exception as exc:              # tk importable but no usable display
        print(f"cannot open the cockpit display: {exc}", file=sys.stderr)
        return 1
    Cockpit(root, args.host, args.port)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

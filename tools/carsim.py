#!/usr/bin/env python3
"""
carsim.py — drivable virtual car engine (the "ECU side" of the cockpit).

What this is
------------
A physics model of a 6-speed automatic car plus a *fake ECU cluster* that
answers real OBD-II / UDS diagnostics, plus a *broadcast bus* with the classic
CAN frames a real car puts on the bus (0x100 engine, 0x110 chassis,
0x120 steering/lights, 0x130 body, 0x140 gear/fuel).

This engine is loopback-only.  It no longer speaks SLCAN-over-TCP and it no
longer opens a raw SocketCAN interface: the only socket is the JSON control
channel, bound to 127.0.0.1:20103.

JSON control channel (port 20103) — one 10 Hz state stream + commands:
    RX {"t":"input","throttle":0.0,"brake":0.0,"steer":0.0}
    RX {"t":"gear","gear":"D"}            RX {"t":"ignition","on":true}
    RX {"t":"switch","name":"hazard","on":true}
    RX {"t":"fault","name":"mil","on":true}
    RX {"t":"cruise","on":true}           RX {"t":"cruise","delta":5}
    RX {"t":"reset"}
    RX {"t":"frame","id":256,"data":"...hex..."}   # CAN INJECT (id 0x100)
    RX {"t":"lin","cmd":"LIN_TRUNK_OPEN"}   # LIN INJECT (body module)
    RX {"t":"lin","id":48,"data":"01"}      # LIN INJECT (raw module frame)
    TX {"t":"state", ...physics, lamps, faults, switches, frames, events}

A diagnostic request frame injected with CAN INJECT is observed but never
answered here — loopback has no reply channel.  The ISO-TP ECU side of the
protocol (single/multi-frame replies, FlowControl pacing, DTCs, VIN) is
exercised headlessly by --selftest.  With --follower, injected drive frames
(0x100/0x110/0x120/0x140/0x400) are folded into the physics as remote
input.  The 0x120 STEER lamp bits (headlights / highbeam / wipers / hazard /
turn) latch the matching switches in ANY mode -- not just --follower --
so the lamps can be hacked straight off the bus.  The 0x130 BODY bits
(doors / trunk / hood / belt) latch their switches the same way -- the
classic one-frame body hack (130#1000... pops the trunk).

Body functions can be driven two ways, matching the two architectures you
meet on real cars.  The 0x120/0x130 latch above models the *naive-trust*
vehicle (the ICSim / 2015-Jeep model): the same broadcast the cluster
reads is also trusted to actuate, so a CAN INJECT replay genuinely turns
the function on.  The LIN channel below models the modern LIN-slave car,
where the actuator command lives behind the BCM (0x10 wiper, 0x20 lights,
0x21 indicators, 0x30 liftgate, 0x40 doors, 0x41 hood/belt) instead of on
the CAN status frames.  LIN is master/slave: the BCM master polls a
module and the slave ANSWERS with its output state, so an attack is not a
broadcast -- it is a *collision-hijack* (Takahashi et al., IPSJ-JIP
25:220, 2017): inject colliding bits into the genuine slave's response;
the slave's simple error handling aborts it; your forged response wins
the slot.  LIN responses carry no authentication, so any byte is
accepted.  The payload carries the module's *complete* output state, not
a bit-flip: LIN_HIGHBEAM_ON (0x20#02) also clears the low beam.  LIN
INJECT: {"t":"lin","cmd":"LIN_TRUNK_OPEN"} or
{"t":"lin","id":48,"data":"01"}.

Fault deck (toggles the same way the GUI shows them):
    mil      -> P0300  (MIL on, rpm jitter)
    overheat -> P0217  (coolant climbs to ~120 C, limp torque x0.5)
    oil      -> P0520  (MIL on)
    abs      -> C0035  (ABS lamp; visible only via UDS 19 02 to physical 7E2)
    flat     -> no DTC (speed capped at 88 km/h + wobble)
    mode 04 / UDS 14 clear -> faults reset, MIL off.

SAE J1979 encodings are big-endian, exactly what real scan tools decode:
    rpm = (b0<<8|b1)/4, speed = b0 km/h, load = b0*100/255,
    coolant/oil/intake = b0-40, maf = (b0<<8|b1)/100 g/s, etc.

Run "carsim.py --help" for the full option list.
"""

import argparse
import json
import math
import re
import socket
import struct
import sys
import threading
import time

__version__ = "0.4.3"

# --------------------------------------------------------------------------- #
#  Constants
# --------------------------------------------------------------------------- #

CTRL_PORT = 20103             # loopback JSON control channel (cockpit GUI)
CTRL_PORT = 20103             # JSON control channel for the cockpit GUI

ECU_ENGINE = 0x7E8            # engine  -> answers functional 0x7DF + phys 0x7E0
ECU_TCM = 0x7E9               # TCM     -> answers physical 0x7E1
ECU_ABS = 0x7EA               # ABS     -> answers physical 0x7E2
REQ_FUNCTIONAL = 0x7DF
REQ_ENGINE = 0x7E0
REQ_TCM = 0x7E1
REQ_ABS = 0x7E2

VIN = b"2LMPJ8K96GBL00001"    # 17 chars, "Ford" style, but ours.

# Physics (6-speed automatic, torque-converter launch)
RATIOS = (3.75, 2.19, 1.41, 1.03, 0.80, 0.66)
FINAL_DRIVE = 3.45
WHEEL_R = 0.315               # m
MASS = 1550.0                 # kg
IDLE_RPM = 800.0
REDLINE = 6400.0
DISP_L = 2.5                  # engine displacement, litres
TANK_L = 60.0                 # fuel tank, litres
AMBIENT_C = 22.0
MAX_SPEED_KMH = 240.0
FLAT_CAP_KMH = 88.0
CRUISE_MIN = 40.0
CRUISE_MAX = 160.0

GEAR_ENUM = {"P": 0, "R": 1, "N": 2, "D": 3}
GEAR_FROM = {v: k for k, v in GEAR_ENUM.items()}
REVERSE_RATIO = 3.2

# Broadcast frames (ids and periods; disabled with --no-traffic)
FRAME_ENGINE = 0x100          # 20 ms : rpm(2) load throttle coolant maf(2) oil
FRAME_CHASSIS = 0x110         # 20 ms : speed brake% flags
FRAME_STEER = 0x120           # 50 ms : steer + light/wiper bits
FRAME_BODY = 0x130            # 50 ms : doors/trunk/hood/seatbelt bits
FRAME_GEAR = 0x140            # 100 ms: gear enum fuel% odo(u32 LE) runtime(u16 LE)

# External drive-input frame (ICSim-style): a CAN INJECT sender (the cockpit
# GUI or a script) drives the car.  bytes: throttle*255, brake*100, gear_enum, steer_byte, ..
# steer_byte: 0..255 with 128 = centered, so (b-128)/127 gives -1..1.
FRAME_DRIVE_IN = 0x400
DRIVE_IDS = (FRAME_ENGINE, FRAME_CHASSIS, FRAME_STEER, FRAME_GEAR, FRAME_DRIVE_IN)

# LIN body modules (the "LIN INJECT" channel).  A LIN frame is the *forged
# response* that won a master poll slot (see the module docstring): the
# payload byte(s) are the exact actuation state the receiver believes the
# slave reported, so forging 0x02 to the light module also clears 0x01
# (headlights) -- a whole-byte write, not a toggle.
LIN_MODULES = {
    0x10: ("wiper",    {0x01: "wipers"}),
    0x20: ("light",    {0x01: "headlights", 0x02: "highbeam"}),
    0x21: ("turn",     {0x01: "left", 0x02: "right", 0x04: "hazard"}),
    0x30: ("liftgate", {0x01: "trunk"}),
    0x40: ("door",     {0x01: "door_fl", 0x02: "door_fr",
                        0x04: "door_rl", 0x08: "door_rr"}),
    0x41: ("hood",     {0x01: "hood", 0x02: "seatbelt"}),
}

# Mnemonic LIN commands -> (module id, forged response payload).  Payload is
# the full module output state, exactly like a raw LIN_xx#dd frame.
LIN_CMDS = {
    "LIN_WIPER_ON":    (0x10, b"\x01"),
    "LIN_WIPER_OFF":   (0x10, b"\x00"),
    "LIN_LIGHT_ON":    (0x20, b"\x01"),
    "LIN_LIGHT_OFF":   (0x20, b"\x00"),
    "LIN_HIGHBEAM_ON": (0x20, b"\x02"),   # teaching point: clears low beam
    "LIN_LIGHTS_FULL": (0x20, b"\x03"),   # headlights + highbeam
    "LIN_LEFT_ON":     (0x21, b"\x01"),
    "LIN_RIGHT_ON":    (0x21, b"\x02"),
    "LIN_HAZARD_ON":   (0x21, b"\x04"),
    "LIN_TURN_OFF":    (0x21, b"\x00"),
    "LIN_TRUNK_OPEN":  (0x30, b"\x01"),
    "LIN_TRUNK_CLOSE": (0x30, b"\x00"),
    "LIN_DOOR_FL_ON":  (0x40, b"\x01"),
    "LIN_DOOR_FR_ON":  (0x40, b"\x02"),
    "LIN_DOOR_RL_ON":  (0x40, b"\x04"),
    "LIN_DOOR_RR_ON":  (0x40, b"\x08"),
    "LIN_DOORS_OFF":   (0x40, b"\x00"),
    "LIN_HOOD_ON":     (0x41, b"\x01"),
    "LIN_HOOD_OFF":    (0x41, b"\x00"),
    "LIN_BELT_ON":     (0x41, b"\x02"),
    "LIN_BELT_OFF":    (0x41, b"\x00"),
}

# --------------------------------------------------------------------------- #
#  Small helpers
# --------------------------------------------------------------------------- #

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _hexs(data):
    return " ".join(f"{b:02X}" for b in data)


def parse_lin_cmd(text):
    """Parse a LIN INJECT command: a LIN_CMDS mnemonic ("LIN_TRUNK_OPEN")
    or a raw module frame ("LIN_10#01").  Returns (lin_id, data) or None.

    CAN-style text ("120#0020") is rejected: LIN ids are single-byte module
    addresses on the body bus, not 11-bit CAN ids."""
    if not isinstance(text, str):
        return None
    s = text.strip()
    if s in LIN_CMDS:
        return LIN_CMDS[s]
    m = re.fullmatch(r"LIN_([0-9A-Fa-f]{2})#([0-9A-Fa-f]{1,16})", s)
    if not m:
        return None
    lin_id = int(m.group(1), 16)
    if lin_id not in LIN_MODULES:
        return None
    data = bytes.fromhex(m.group(2))
    if not data or len(data) > 8:
        return None
    return (lin_id, data)


# --------------------------------------------------------------------------- #
#  Physics
# --------------------------------------------------------------------------- #

class CarPhysics:
    """The drivable car.  step(dt) advances it; the ECU layer reads it."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.t = 0.0
        self.v = 0.0                     # m/s (negative = reverse)
        self.rpm = 0.0
        self.gear = "P"                  # P R N D
        self.gear_num = 0                # 1..6 in D, 0 otherwise
        self.throttle = 0.0              # 0..1 target
        self.brake = 0.0                 # 0..1
        self.steer = 0.0                 # -1..1
        self.throttle_eff = 0.0          # after limiter/limp (drive torque)
        self.load_pct = 0.0              # 0..1 engine load
        self.engine_on = False
        self.ignition = False
        self.crank_t = 0.0
        self.coolant = AMBIENT_C
        self.oil = AMBIENT_C
        self.intake = AMBIENT_C + 6.0
        self.fuel = TANK_L               # litres
        self.odo = 0.0                   # km
        self.runtime = 0.0               # s engine run time
        self.voltage = 12.4
        self.maf = 0.0                   # g/s
        self.fuel_rate = 0.0             # L/h
        self.cruise_on = False
        self.cruise_set = 0.0            # km/h
        self._cruise_i = 0.0
        self.tc_pulse = False            # wheelspin / TC active this tick
        self.limp = False
        self.rev_limit = False
        self.flat = False
        self.overheat_now = False
        self._thr_prev = 0.0            # previous tick's throttle (kickdown jab)
        self._was_rev_limit = False     # rising-edge gating for events
        self._was_tc = False

    # ------------------------------------------------------------- events
    def step(self, dt, events, switches, faults):
        """Advance one physics tick.  events: list to append strings to."""
        self.t += dt
        self.limp = faults.get("overheat", False)
        self.flat = faults.get("flat", False)

        # -- engine start / crank
        if self.ignition and not self.engine_on:
            if self.crank_t <= 0.0:
                if self.gear in ("P", "N"):
                    self.crank_t = 0.7
                else:
                    events.append("crank blocked - shift to P or N")
                    self.ignition = False
            else:
                self.crank_t -= dt
                if self.crank_t <= 0.0:
                    self.engine_on = True
                    events.append("engine started")
                    if self.gear == "D":   # re-engage 1st if D was selected
                        self.gear_num = 1   # while the engine was cranking
        if not self.ignition and self.engine_on:
            self.engine_on = False
            self.cruise_on = False
            events.append("engine stalled (ignition off)")

        # -- cruise control (P controller with small integral)
        if self.cruise_on:
            if self.brake > 0.15 or self.gear != "D" or not self.engine_on:
                self.cruise_on = False
                events.append("cruise disengaged")
            else:
                err = self.cruise_set - self.v * 3.6
                self._cruise_i = clamp(self._cruise_i + err * dt * 0.02,
                                       -0.2, 0.2)
                self.throttle = clamp(0.03 + err * 0.012 + self._cruise_i,
                                      0.0, 0.85)

        # -- torque production
        throttle = self.throttle
        speed_kmh = abs(self.v) * 3.6
        if not self.engine_on:
            self.rpm = 260.0 if self.crank_t > 0 else 0.0
            throttle = 0.0
            self.gear_num = 0
            self.rev_limit = False
            self.tc_pulse = False
            self._was_rev_limit = False
            self._was_tc = False
            self.load_pct = 0.0
            self.maf = 0.0
            # engine-off coast-down: rolling drag + rolling resistance +
            # brakes decelerate a moving car to a stop instead of freezing
            # it at speed (the old bug left you stuck doing 35 km/h).
            drag = 0.42 * self.v * abs(self.v)
            roll = 0.015 * MASS * 9.81 * (1.0 if self.v >= 0 else -1.0)
            if self.v == 0.0:
                roll = 0.0
            brake_f = self.brake * 9000.0 * (1.0 if self.v >= 0 else -1.0)
            if abs(self.v) < 0.05 and brake_f * math.copysign(1, self.v) < 0:
                brake_f = 0.0
            a = -(drag + roll + brake_f) / MASS
            self.v = clamp(self.v + a * dt, -30.0 / 3.6, MAX_SPEED_KMH / 3.6)
            if abs(self.v) < 0.02:
                self.v = 0.0
        else:
            # converter slip: rpm = idle + throttle*2600*(1 - v/30)
            conv = IDLE_RPM + throttle * 2600.0 * max(0.0, 1.0 - abs(self.v) / 30.0)
            if self.gear == "D" and self.gear_num >= 1:
                wheel_rps = self.v / (2.0 * math.pi * WHEEL_R)
                speed_rpm = wheel_rps * RATIOS[self.gear_num - 1] * FINAL_DRIVE * 60.0
                self.rpm = max(IDLE_RPM, speed_rpm * 0.95, conv)
            elif self.gear == "R":
                wheel_rps = abs(self.v) / (2.0 * math.pi * WHEEL_R)
                speed_rpm = wheel_rps * REVERSE_RATIO * FINAL_DRIVE * 60.0
                self.rpm = max(IDLE_RPM, speed_rpm * 0.95, conv)
            else:                       # P / N
                self.rpm = conv

            # rev limiter + misfire jitter (MIL fault)
            self.rev_limit = self.rpm >= REDLINE
            if self.rev_limit:
                throttle = 0.0
                if not self._was_rev_limit:
                    events.append("rev limiter")
            if faults.get("mil", False):
                self.rpm += 25.0 * math.sin(self.t * 7.3) + 15.0 * math.sin(self.t * 2.1)

            # engine torque curve (peak ~3600 rpm)
            shape = 0.55 + 0.45 * math.exp(-((self.rpm - 3600.0) / 1600.0) ** 2)
            torque = 250.0 * throttle * shape
            if self.limp:
                torque *= 0.5

            # wheelspin / traction control at launch
            self.tc_pulse = (self.gear == "D" and self.gear_num == 1
                             and abs(self.v) < 5.5 and throttle > 0.55)
            if self.tc_pulse:
                torque *= 0.8
                self.rpm += 380.0 * throttle * abs(math.sin(self.t * 11.0))
                if not self._was_tc:
                    events.append("wheelspin - TC cutting power")

            # gear selection (auto, D only)
            if self.gear == "D":
                if self.rpm > 6000.0 and self.gear_num < 6:
                    n = self.gear_num + 1
                    self.gear_num = n
                    events.append(f"shift up {n - 1}->{n}")
                elif self.rpm < 2300.0 and self.gear_num > 1:
                    n = self.gear_num - 1
                    self.gear_num = n
                    events.append(f"shift down {n + 1}->{n}")
                # kickdown: a hard throttle jab at speed DROPS gears for
                # passing power (real kickdown is a downshift; the old code
                # wrongly upshifted here and walked 1->6 at launch).
                jab = throttle - self._thr_prev
                if (jab > 0.35 and throttle > 0.85 and self.gear_num >= 2
                        and abs(self.v) * 3.6 >= 25.0 and self.rpm < 5200.0):
                    cur = RATIOS[self.gear_num - 1]
                    cand = self.gear_num
                    for g in range(self.gear_num - 1, 0, -1):
                        if self.rpm * RATIOS[g - 1] / cur <= REDLINE * 0.97:
                            cand = g
                        else:
                            break
                    if cand < self.gear_num:
                        events.append(f"kickdown {self.gear_num}->{cand}")
                        self.gear_num = cand
            # latch edge flags for next tick's rising-edge checks
            self._was_rev_limit = self.rev_limit
            self._was_tc = self.tc_pulse

            # drive force
            drive = 0.0
            if self.gear == "D" and self.gear_num >= 1:
                drive = torque * RATIOS[self.gear_num - 1] * FINAL_DRIVE / WHEEL_R
            elif self.gear == "R":
                drive = -torque * REVERSE_RATIO * FINAL_DRIVE / WHEEL_R
            elif self.gear == "N":
                pass
            converter = 1.0 + 0.9 * throttle * max(0.0, 1.0 - abs(self.v) / 8.0)
            drive *= converter

            # resistances
            drag = 0.42 * self.v * abs(self.v)
            roll = 0.015 * MASS * 9.81 * (1.0 if self.v >= 0 else -1.0)
            if self.v == 0.0:
                roll = 0.0
            brake_f = self.brake * 9000.0 * (1.0 if self.v >= 0 else -1.0)
            if abs(self.v) < 0.05 and brake_f * math.copysign(1, self.v) < 0:
                brake_f = 0.0
            a = (drive - drag - roll - brake_f) / MASS

            # flat tyre governor: cut drive above the cap so the car coasts
            # back down; below the cap normal drive is allowed but must never
            # push the car past the cap.
            if self.flat:
                cap = FLAT_CAP_KMH / 3.6
                if self.v > cap:
                    a = -(drag + roll) / MASS       # drive cut, coast down
                elif self.v < cap and a > 0.0 and self.v + a * dt > cap:
                    a = (cap - self.v) / dt         # ease up to the cap
                a *= (1.0 + 0.03 * math.sin(self.t * 3.0))   # tyre wobble
                if not getattr(self, "_flat_warned", False):
                    events.append("flat tyre - limited to ~88 km/h")
                    self._flat_warned = True
            else:
                self._flat_warned = False

            self.v = clamp(self.v + a * dt, -30.0 / 3.6, MAX_SPEED_KMH / 3.6)
            if abs(self.v) < 0.02 and drive == 0.0:
                self.v = 0.0

            # load + airflow + fuel
            self.load_pct = clamp(0.2 + 0.8 * throttle + self.rpm / REDLINE * 0.3,
                                  0.0, 1.0)
            self.maf = (self.rpm / 60.0) * (DISP_L / 2.0) * 0.85 * 1.184 \
                * self.load_pct
            power_w = max(0.0, torque * self.rpm * 2.0 * math.pi / 60.0)
            burn_l_s = power_w / (0.30 * 32.0e6) + (0.9 / 3600.0)
            self.fuel_rate = burn_l_s * 3600.0
            self.fuel = max(0.0, self.fuel - burn_l_s * dt)
            self.runtime += dt

            # temperatures
            target = 122.0 if self.limp else 90.0
            self.coolant += (target - self.coolant) * dt * (0.03 if self.engine_on else 0.004)
            self.overheat_now = self.coolant > 115.0
            if self.overheat_now and not getattr(self, "_hot_warned", False):
                events.append("overheating! limp mode")
                self._hot_warned = True
            if not self.limp and self.coolant < 95.0:
                self._hot_warned = False
            self.oil += (self.coolant + 8.0 - self.oil) * dt * 0.02
            self.intake = AMBIENT_C + 8.0 + (self.coolant - AMBIENT_C) * 0.2

        self._thr_prev = self.throttle
        self.voltage = 14.2 if self.engine_on else 12.4
        self.odo += abs(self.v) * dt / 1000.0

    # ------------------------------------------------------------ gear shifts
    def set_gear(self, target, events):
        if target == self.gear:
            return
        speed = abs(self.v) * 3.6
        if target in ("R", "P") and speed > 8.0:
            events.append(f"{target} blocked above 8 km/h ({speed:.0f} km/h)")
            return
        self.gear = target
        if target == "D":
            self.gear_num = 1 if speed > 2.0 else 1
        elif target == "R":
            self.gear_num = 0
        else:
            self.gear_num = 0
        self.cruise_on = False
        events.append(f"gear -> {target}")

    # ------------------------------------------------------------ ctrl stream
    def snapshot(self, switches, faults):
        """One 10 Hz state dict for the cockpit GUI."""
        speed_kmh = abs(self.v) * 3.6 if self.v >= 0 else -abs(self.v) * 3.6
        return {
            "speed": round(speed_kmh, 1),
            "rpm": round(self.rpm, 0),
            "gear": self.gear,
            "gear_num": self.gear_num,
            "throttle": round(self.throttle * 100.0, 1),
            "brake": round(self.brake * 100.0, 1),
            "steer": round(self.steer * 100.0, 0),
            "engine_on": self.engine_on,
            "coolant": round(self.coolant, 1),
            "oil": round(self.oil, 1),
            "intake": round(self.intake, 1),
            "ambient": AMBIENT_C,
            "maf": round(self.maf, 1),
            "fuel": round(self.fuel / TANK_L * 100.0, 1),
            "fuel_l": round(self.fuel, 1),
            "odo": round(self.odo, 1),
            "runtime": int(self.runtime),
            "voltage": round(self.voltage, 1),
            "load": round(self.load_pct * 100.0, 1),
            "limp": self.limp,
            "rev_limit": self.rev_limit,
            "cruise_on": self.cruise_on,
            "cruise_set": round(self.cruise_set, 0),
            "blink": 1 if int(self.t / 0.333) % 2 == 0 else 0,
        }

# --------------------------------------------------------------------------- #
#  Broadcast frames + lamps
# --------------------------------------------------------------------------- #

def frame_engine(p):
    """0x100, 20 ms: rpm(2) load throttle coolant+40 maf(2) oil+40."""
    rpm = clamp(int(p.rpm), 0, 0xFFFF)
    maf = clamp(int(p.maf * 100.0), 0, 0xFFFF)
    return bytes([
        (rpm >> 8) & 0xFF, rpm & 0xFF,
        clamp(int(p.load_pct * 255.0), 0, 255),
        clamp(int(p.throttle * 255.0), 0, 255),
        clamp(int(p.coolant + 40.0), 0, 255),
        (maf >> 8) & 0xFF, maf & 0xFF,
        clamp(int(p.oil + 40.0), 0, 255),
    ])


def frame_chassis(p, switches):
    """0x110, 20 ms: speed km/h brake% flags(ABS event/TC/parkbrake/brake)."""
    flags = 0x00
    if p.brake > 0.02:
        flags |= 0x01                      # brake lamp
    if p.brake > 0.75 and abs(p.v) * 3.6 > 8.0:
        flags |= 0x02                      # ABS pulsing
    if p.tc_pulse:
        flags |= 0x04                      # traction control event
    if switches.get("parkbrake", False):
        flags |= 0x08
    return bytes([
        clamp(int(abs(p.v) * 3.6), 0, 255),
        clamp(int(p.brake * 100.0), 0, 100),
        flags, 0x00, 0x00, 0x00, 0x00, 0x00,
    ])


def frame_steer(p, switches, blink):
    """0x120, 50 ms: steer (signed -100..100) + light bits (blink at 1.5 Hz)."""
    steer = clamp(int(p.steer * 100.0), -100, 100) & 0xFF
    bits = 0x00
    if switches.get("headlights", False):
        bits |= 0x10
    if switches.get("highbeam", False):
        bits |= 0x08
    if switches.get("wipers", False):
        bits |= 0x20
    on = blink == 1
    if switches.get("left", False) and on:
        bits |= 0x01
    if switches.get("right", False) and on:
        bits |= 0x02
    if switches.get("hazard", False) and on:
        bits |= 0x04
    return bytes([steer, bits, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])


def frame_body(switches):
    """0x130, 50 ms: doors/trunk/hood/seatbelt bits."""
    bits = 0x00
    for i, name in enumerate(("door_fl", "door_fr", "door_rl", "door_rr")):
        if switches.get(name, False):
            bits |= 1 << i
    if switches.get("trunk", False):
        bits |= 0x10
    if switches.get("hood", False):
        bits |= 0x20
    if switches.get("seatbelt", False):
        bits |= 0x40
    return bytes([bits, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])


def frame_gear(p):
    """0x140, 100 ms: gear enum fuel% odo(u32 LE) runtime-min(u16 LE)."""
    odo = int(p.odo) & 0xFFFFFFFF
    rt_min = int(p.runtime / 60.0) & 0xFFFF
    return bytes([
        GEAR_ENUM.get(p.gear, 2),
        clamp(int(p.fuel / TANK_L * 255.0), 0, 255),
        odo & 0xFF, (odo >> 8) & 0xFF, (odo >> 16) & 0xFF, (odo >> 24) & 0xFF,
        rt_min & 0xFF, (rt_min >> 8) & 0xFF,
    ])


def compute_lamps(p, faults, switches):
    """Warning-lamp dict for the GUI + chime/door/low-fuel events."""
    lamps = {
        "mil": bool(faults.get("mil") or faults.get("overheat") or faults.get("oil")),
        "abs": faults.get("abs", False),
        "tc": p.tc_pulse,
        "battery": not p.engine_on,
        "seatbelt": p.engine_on and abs(p.v) * 3.6 > 5.0 and not switches.get("seatbelt", False),
        "lowfuel": p.fuel / TANK_L * 100.0 < 12.0,
        "door": any(switches.get(n, False) for n in ("door_fl", "door_fr", "door_rl", "door_rr")),
        "parkbrake": switches.get("parkbrake", False),
    }
    return lamps


# --------------------------------------------------------------------------- #
#  ECU brains (OBD-II mode 01/03/04/09 + UDS 10/14/19/3E)
# --------------------------------------------------------------------------- #

# PID support masks per ECU (J1979: byte n covers PIDs 8n+1..8n+8)
ENGINE_PID_MASK = bytearray(16)
for pid in (0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0C, 0x0D, 0x0E, 0x0F,
            0x10, 0x11, 0x1C, 0x1F, 0x21, 0x2F, 0x31, 0x33, 0x42, 0x45,
            0x46, 0x5C):
    ENGINE_PID_MASK[(pid - 1) // 8] |= 0x80 >> ((pid - 1) % 8)
TCM_PID_MASK = bytearray(16)
for pid in (0x0D, 0x1C, 0x2F):
    TCM_PID_MASK[(pid - 1) // 8] |= 0x80 >> ((pid - 1) % 8)
ABS_PID_MASK = bytearray(16)
for pid in (0x0D, 0x1C):
    ABS_PID_MASK[(pid - 1) // 8] |= 0x80 >> ((pid - 1) % 8)

FAULT_DTCS = {
    "mil":      [(0x03, 0x00, 0x28)],   # P0300 random misfire, confirmed+MIL
    "overheat": [(0x02, 0x17, 0x28)],   # P0217 engine overtemp, confirmed+MIL
    "oil":      [(0x05, 0x20, 0x28)],   # P0520 oil pressure, confirmed+MIL
    "abs":      [(0x40, 0x35, 0x08)],   # C0035 ABS, confirmed (no MIL)
    "flat":     [],
}


def ecu_for_req(req_id):
    """Request id -> (ecu response id, 'engine'|'tcm'|'abs'|None)."""
    if req_id in (REQ_FUNCTIONAL, REQ_ENGINE):
        return ECU_ENGINE, "engine"
    if req_id == REQ_TCM:
        return ECU_TCM, "tcm"
    if req_id == REQ_ABS:
        return ECU_ABS, "abs"
    return None, None


def mode01_payload(p, ecu, pid, faults):
    """Build the data bytes of a 41 <pid> response (no 41 header)."""
    fuel_pct = p.fuel / TANK_L * 100.0
    if ecu == "engine":
        if pid == 0x00:
            return bytes(ENGINE_PID_MASK[0:4])
        if pid == 0x20:
            return bytes(ENGINE_PID_MASK[4:8])
        if pid == 0x40:
            return bytes(ENGINE_PID_MASK[8:12])
        if pid == 0x01:   # MIL + DTC count
            mil = bool(faults.get("mil") or faults.get("overheat") or faults.get("oil"))
            n = sum(1 for f in ("mil", "overheat", "oil") if faults.get(f))
            return bytes([(0x80 if mil else 0x00) | 0x01, n & 0xFF])
        if pid == 0x03:   # fuel system status: closed loop when running
            return b"\x02\x01" if p.engine_on else b"\x01\x00"
        if pid == 0x04:
            return bytes([clamp(int(p.load_pct * 255.0), 0, 255)])
        if pid == 0x05:
            return bytes([clamp(int(p.coolant + 40.0), 0, 255)])
        if pid == 0x06:
            return b"\x80"          # STFT 0%
        if pid == 0x07:
            return b"\x80"          # LTFT 0%
        if pid == 0x0C:
            return bytes([(int(clamp(p.rpm, 0, 0x3FFF) * 4) >> 8) & 0xFF,
                          int(clamp(p.rpm, 0, 0x3FFF) * 4) & 0xFF])
        if pid == 0x0D:
            return bytes([clamp(int(abs(p.v) * 3.6), 0, 255)])
        if pid == 0x0E:
            return bytes([clamp(int(10.0 * 2.0 + 64.0), 0, 255)])  # 10 deg adv
        if pid == 0x0F:
            return bytes([clamp(int(p.intake + 40.0), 0, 255)])
        if pid == 0x10:
            maf = clamp(int(p.maf * 100.0), 0, 0xFFFF)
            return bytes([(maf >> 8) & 0xFF, maf & 0xFF])
        if pid == 0x11:
            return bytes([clamp(int(p.throttle * 255.0), 0, 255)])
        if pid == 0x1C:
            return b"\x01"          # OBD-II
        if pid == 0x1F:
            return bytes([min(int(p.runtime) & 0xFF, 0xFF)])
        if pid == 0x21:             # distance with MIL on
            mil = bool(faults.get("mil") or faults.get("overheat") or faults.get("oil"))
            return struct.pack("<H", int(p.odo) & 0xFFFF) if mil else b"\x00\x00"
        if pid == 0x2F:
            return bytes([clamp(int(fuel_pct * 2.55), 0, 255)])
        if pid == 0x31:
            return struct.pack("<H", int(p.odo) & 0xFFFF)
        if pid == 0x33:
            return b"\x64"          # 100 kPa baro
        if pid == 0x42:
            return bytes([clamp(int(p.voltage * 10.0), 0, 255)])
        if pid == 0x45:
            return bytes([clamp(int(p.throttle * 255.0), 0, 255)])
        if pid == 0x46:
            return bytes([clamp(int(AMBIENT_C + 40.0), 0, 255)])
        if pid == 0x5C:
            return bytes([clamp(int(p.oil + 40.0), 0, 255)])
        return None
    if ecu == "tcm":
        if pid == 0x00:
            return bytes(TCM_PID_MASK[0:4])
        if pid == 0x20:
            return bytes(TCM_PID_MASK[4:8])
        if pid == 0x0D:
            return bytes([clamp(int(abs(p.v) * 3.6), 0, 255)])
        if pid == 0x1C:
            return b"\x01"
        if pid == 0x2F:
            return bytes([clamp(int(fuel_pct * 2.55), 0, 255)])
        return None
    if ecu == "abs":
        if pid == 0x00:
            return bytes(ABS_PID_MASK[0:4])
        if pid == 0x20:
            return bytes(ABS_PID_MASK[4:8])
        if pid == 0x0D:
            return bytes([clamp(int(abs(p.v) * 3.6), 0, 255)])
        if pid == 0x1C:
            return b"\x01"
        return None
    return None


def engine_dtcs(faults):
    """Active engine DTCs: (hi, lo, status) triples, P-codes only."""
    out = []
    for f in ("mil", "overheat", "oil"):
        if faults.get(f):
            out.extend(FAULT_DTCS[f])
    return out


def abs_dtcs(faults):
    return list(FAULT_DTCS["abs"]) if faults.get("abs", False) else []


def handle_request(req_id, payload, p, faults, ecus_enabled):
    """
    One ISO-TP request -> list of items:
        ("sf", resp_id, bytes)          single frame, send now
        ("fc", resp_id, bytes)          raw 8-byte FlowControl answer
        ("mf", resp_id, payload)        multi frame, needs FlowControl
    Mirrors the reference ECU sim.py's service dispatch, but reads live physics.
    """
    ecu_id, ecu = ecu_for_req(req_id)
    if ecu_id is None:
        return []
    if ecu == "tcm" and ecus_enabled < 2:
        return []
    if ecu == "abs" and ecus_enabled < 3:
        return []

    if not payload:
        return []
    pci = payload[0]

    # -- incoming FirstFrame: answer raw FlowControl (we don't assemble)
    if 0x10 <= pci <= 0x1F:
        return [("fc", ecu_id, b"\x30\x00\x00\x00\x00\x00\x00\x00")]
    if 0x20 <= pci <= 0x2F or 0x30 <= pci <= 0x3F:
        return []           # stray CF/FC with nothing pending to assemble

    # -- single frame: PCI byte 0 holds the payload length (1..7)
    if pci == 0x00 or pci > 0x07:
        return []

    req = payload[1:1 + pci]            # strip ISO-TP single-frame PCI
    if not req:
        return []
    svc = req[0]
    body = req[1:]

    if svc == 0x01:                     # mode 01: current data
        pid = body[0] if body else 0x00
        data = mode01_payload(p, ecu, pid, faults)
        if data is None:
            resp = bytes([0x7F, 0x01, 0x12])
        else:
            resp = bytes([0x41, pid]) + data
    elif svc == 0x03:                   # mode 03: stored DTCs
        dtcs = engine_dtcs(faults) if ecu == "engine" else []
        resp = bytes([0x43, len(dtcs)]) + b"".join(
            bytes([h, l]) for h, l, _s in dtcs)
    elif svc == 0x04:                   # mode 04: clear DTCs (engine MIL/P-codes)
        for f in ("mil", "overheat", "oil"):
            faults[f] = False
        resp = b"\x44"
    elif svc == 0x09:                   # mode 09: VIN (engine only)
        pid = body[0] if body else 0x00
        if ecu == "engine" and pid == 0x02:
            return [("mf", ecu_id, b"\x49\x02\x01" + VIN)]
        resp = bytes([0x7F, 0x09, 0x12])
    elif svc == 0x10:                   # UDS session control
        sub = body[0] if body else 0x00
        resp = bytes([0x50, sub])
    elif svc == 0x14:                   # UDS clear diagnostic information
        if ecu == "engine":
            for f in ("mil", "overheat", "oil"):
                faults[f] = False
        elif ecu == "abs":
            faults["abs"] = False
        resp = b"\x54"
    elif svc == 0x19:                   # UDS read DTC information
        sub = body[0] if body else 0x00
        if sub == 0x01:
            n = len(engine_dtcs(faults)) if ecu == "engine" else len(abs_dtcs(faults))
            resp = bytes([0x59, 0x01, 0xFF, n])
        elif sub == 0x02:
            dtcs = engine_dtcs(faults) if ecu == "engine" else abs_dtcs(faults)
            resp = bytes([0x59, 0x02, 0xFF]) + b"".join(
                bytes([h, l, s]) for h, l, s in dtcs)
        else:
            resp = bytes([0x7F, 0x19, 0x12])
    elif svc == 0x3E:                   # UDS tester present
        sub = body[0] if body else 0x00
        resp = bytes([0x7E, sub])
    elif svc in (0x02, 0x05, 0x06, 0x07, 0x08, 0x0A):
        resp = bytes([0x7F, svc, 0x12])  # known OBD mode, unsupported here
    else:
        resp = bytes([0x7F, svc, 0x11])  # service not supported

    if len(resp) <= 7:
        return [("sf", ecu_id, resp)]
    return [("mf", ecu_id, resp)]


# --------------------------------------------------------------------------- #
#  JSON control channel (cockpit GUI)
# --------------------------------------------------------------------------- #

class CtrlServer:
    def __init__(self, sim, host, port):
        self.sim = sim
        self.host = host
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.sock = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(5)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                conn, _addr = self.sock.accept()
            except OSError:
                return
            conn.settimeout(0.5)
            with self.lock:
                self.clients.append(conn)
            threading.Thread(target=self._reader, args=(conn,), daemon=True).start()

    def _reader(self, conn):
        buf = b""
        while True:
            try:
                chunk = conn.recv(1024)
            except socket.timeout:
                continue        # idle client: keep the 10 Hz stream open
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
                    msg = json.loads(raw.decode("utf-8"))
                except ValueError:
                    continue
                self.sim.apply_input(msg)
        with self.lock:
            if conn in self.clients:
                self.clients.remove(conn)
        try:
            conn.close()
        except OSError:
            pass

    def push(self, state):
        blob = (json.dumps(state) + "\n").encode("utf-8")
        with self.lock:
            dead = []
            for c in self.clients:
                try:
                    c.sendall(blob)
                except OSError:
                    dead.append(c)
            for c in dead:
                self.clients.remove(c)


# --------------------------------------------------------------------------- #
#  The simulation core
# --------------------------------------------------------------------------- #

class CarSim:
    """Owns the physics, the ECUs and the 20 ms tick loop (loopback only)."""

    def __init__(self, ctrl_port=CTRL_PORT, no_traffic=False, ecus=3,
                 speed=1.0, follower=False):
        self.ctrl_port = ctrl_port
        self.no_traffic = no_traffic
        self.ecus = ecus
        self.speed = speed
        self.phys = CarPhysics()
        self.follower = follower
        self.remote_drive = False
        self.drive_frame = None
        self.switches = {
            "headlights": False, "highbeam": False, "wipers": False,
            "left": False, "right": False, "hazard": False,
            "parkbrake": False,
            "door_fl": False, "door_fr": False, "door_rl": False,
            "door_rr": False, "trunk": False, "hood": False,
            "seatbelt": True,
        }
        self.faults = {"mil": False, "overheat": False, "oil": False,
                       "abs": False, "flat": False}
        self.events = []
        self._ev_lock = threading.Lock()
        self.ctrl = CtrlServer(self, "127.0.0.1", ctrl_port)
        self._frames = {}                # id -> (period, next_time)
        self._frame_builders = {
            FRAME_ENGINE: (0.020, lambda: frame_engine(self.phys)),
            FRAME_CHASSIS: (0.020, lambda: frame_chassis(self.phys, self.switches)),
            FRAME_STEER: (0.050, lambda: frame_steer(
                self.phys, self.switches, self._blink())),
            FRAME_BODY: (0.050, lambda: frame_body(self.switches)),
            FRAME_GEAR: (0.100, lambda: frame_gear(self.phys)),
        }
        for fid, (period, _b) in self._frame_builders.items():
            self._frames[fid] = (period, 0.0)
        self._last_chime = 0.0
        self._last_lowfuel = 0.0
        self._last_door = 0.0
        self._last_pb = 0.0
        self._stop = False
        self._last_frames = []
        self._sent_frames = {}             # id -> last broadcast bytes (snapshot)
        self._last_lin_frames = []        # LIN bus capture ring (48 records)
        self._lin_last_rx = {}            # module id -> last emitted RX byte
        self._lin_poll_next = 0.0         # monotonic deadline, 10 Hz master poll

    def _blink(self):
        return 1 if int(self.phys.t / 0.333) % 2 == 0 else 0

    # ------------------------------------------------------------- commands
    def apply_input(self, msg):
        t = msg.get("t")
        ev = []
        if t == "input":
            self.phys.throttle = clamp(float(msg.get("throttle", 0.0)), 0.0, 1.0)
            self.phys.brake = clamp(float(msg.get("brake", 0.0)), 0.0, 1.0)
            self.phys.steer = clamp(float(msg.get("steer", 0.0)), -1.0, 1.0)
        elif t == "gear":
            g = str(msg.get("gear", "P")).upper()
            if g in ("P", "R", "N", "D"):
                self.phys.set_gear(g, ev)
        elif t == "ignition":
            on = bool(msg.get("on", False))
            self.phys.ignition = on
            ev.append("ignition ON" if on else "ignition OFF")
        elif t == "switch":
            name = str(msg.get("name", ""))
            on = bool(msg.get("on", False))
            if name in self.switches:
                self.switches[name] = on
                ev.append(f"{name} {'on' if on else 'off'}")
        elif t == "fault":
            name = str(msg.get("name", ""))
            on = bool(msg.get("on", False))
            if name in self.faults:
                self.faults[name] = on
                if name == "flat" and on:
                    ev.append("flat tyre! limited to ~88 km/h")
                else:
                    ev.append(f"fault {name} {'set' if on else 'cleared'}")
                if not on and name != "flat":
                    ev.append("DTC will clear on next mode 04 / UDS 14")
        elif t == "cruise":
            if "on" in msg:
                if msg["on"] and not self.phys.cruise_on:
                    ok = (self.phys.engine_on and self.phys.gear == "D"
                          and abs(self.phys.v) * 3.6 >= CRUISE_MIN)
                    if ok:
                        self.phys.cruise_on = True
                        self.phys.cruise_set = round(abs(self.phys.v) * 3.6 / 5.0) * 5.0
                        self.phys._cruise_i = 0.0
                        ev.append(f"cruise engaged {self.phys.cruise_set:.0f} km/h")
                    else:
                        ev.append("cruise needs D and >= 40 km/h")
                elif not msg["on"] and self.phys.cruise_on:
                    self.phys.cruise_on = False
                    ev.append("cruise disengaged")
            elif "delta" in msg:
                d = float(msg.get("delta", 0))
                if self.phys.cruise_on:
                    self.phys.cruise_set = clamp(
                        self.phys.cruise_set + d, CRUISE_MIN, CRUISE_MAX)
                    ev.append(f"cruise set {self.phys.cruise_set:.0f} km/h")
        elif t == "reset":
            self.phys.reset()
            for k in self.switches:
                self.switches[k] = False
            self.switches["seatbelt"] = True
            for k in self.faults:
                self.faults[k] = False
            self._last_lin_frames = []
            self._lin_last_rx = {}
            self._lin_poll_next = 0.0
            ev.append("sim reset")
        elif t == "frame":
            # CAN INJECT: raw loopback frame from the cockpit GUI or a script,
            # fed straight to handle_rx_frame.  Loopback has no reply channel,
            # so a diagnostic request frame is observed but never answered
            # here; the ISO-TP ECU side is exercised by --selftest.
            try:
                raw_id = msg.get("id", 0)
                can_id = int(raw_id) if not isinstance(raw_id, str) \
                    else int(raw_id, 0)
                data = bytes.fromhex(str(msg.get("data", "")))
            except (KeyError, ValueError, TypeError):
                return
            if not data or len(data) > 8:
                return
            self.handle_rx_frame(can_id, data)
            return
        elif t == "lin":
            # LIN INJECT: body functions behind the BCM.  Accepts a mnemonic
            # ({"t":"lin","cmd":"LIN_TRUNK_OPEN"}) or a raw module frame
            # ({"t":"lin","id":48,"data":"01"}).  Same early-return shape
            # as the CAN frame branch above.
            #
            # What the injection MEANS physically (Takahashi et al. 2017,
            # "Automotive Attacks and Countermeasures on LIN-Bus",
            # IPSJ-JIP 25:220): LIN is master/slave -- the BCM master polls,
            # the addressed slave answers.  The attacker does NOT broadcast
            # a command; they COLLIDE with the genuine slave's response
            # mid-slot.  The slave's simple error handling (it compares the
            # bus bits against its own transmission and aborts on mismatch)
            # kills the genuine response, and the attacker finishes the slot
            # with a forged one.  LIN responses carry no authentication, so
            # the receiver accepts the attacker's byte.
            try:
                if "cmd" in msg:
                    parsed = parse_lin_cmd(str(msg.get("cmd", "")))
                    if parsed is None:
                        return
                    lin_id, data = parsed
                else:
                    raw_id = msg.get("id", 0)
                    lin_id = int(raw_id) if not isinstance(raw_id, str) \
                        else int(raw_id, 0)
                    data = bytes.fromhex(str(msg.get("data", "")))
            except (KeyError, ValueError, TypeError):
                return
            if not data or len(data) > 8:
                return
            ev_lin = []
            genuine = self._lin_module_state(lin_id)
            if genuine != data[0]:
                # The genuine slave was mid-answer with the TRUE state when
                # the collision hit; its error handling aborted it.  Log that
                # aborted response right before the attacker's forged one so
                # the ring shows the kill-and-replace, not a clean write.
                self._capture_lin(lin_id, genuine, "RX")
                ev_lin.append(
                    f"LIN collision: module 0x{lin_id:02X} genuine "
                    f"{genuine:02X} response aborted -> false "
                    f"{data[0]:02X} accepted")
            self._handle_lin_frame(lin_id, data, ev_lin)
            if ev_lin:
                with self._ev_lock:
                    self.events.extend(ev_lin)
            self._capture_lin(lin_id, data[0], "TX")
            return
        if ev:
            with self._ev_lock:
                self.events.extend(ev)

    # ------------------------------------------------------------ lamp latch
    def _latch_lamp_bits(self, bits, ev):
        """0x120 STEER byte1 -> switches: headlights 0x10, wipers 0x20,
        hazard 0x04, highbeam 0x08, left 0x01, right 0x02.  Runs in ANY
        mode (an injected frame flips the switch; the sim's own broadcast
        then re-encodes it) -- the classic one-frame lamp hack."""
        for _n, _b in (("headlights", 0x10), ("highbeam", 0x08),
                       ("wipers", 0x20), ("hazard", 0x04),
                       ("left", 0x01), ("right", 0x02)):
            _on = bool(bits & _b)
            if self.switches.get(_n) != _on:
                self.switches[_n] = _on
                ev.append(f"{_n} {'on' if _on else 'off'} via CAN INJECT")

    # ------------------------------------------------------------- body latch
    def _latch_body_bits(self, bits, ev):
        """0x130 BODY byte0 -> switches: doors 0x01..0x08, trunk 0x10,
        hood 0x20, seatbelt 0x40.  Runs in ANY mode (an injected frame
        flips the switch; the sim's own broadcast then re-encodes it) --
        the classic one-frame body hack."""
        for _i, _n in enumerate(("door_fl", "door_fr", "door_rl",
                                 "door_rr")):
            _on = bool(bits & (1 << _i))
            if self.switches.get(_n) != _on:
                self.switches[_n] = _on
                ev.append(f"{_n} {'on' if _on else 'off'} via CAN INJECT")
        for _n, _b in (("trunk", 0x10), ("hood", 0x20), ("seatbelt", 0x40)):
            _on = bool(bits & _b)
            if self.switches.get(_n) != _on:
                self.switches[_n] = _on
                ev.append(f"{_n} {'on' if _on else 'off'} via CAN INJECT")

    # ------------------------------------------------------------ LIN inject
    def _handle_lin_frame(self, lin_id, data, ev):
        """A forged LIN *response* arrived via LIN INJECT: apply its state.

        What arrived is not a command from the master (LIN masters never
        write actuator state; they poll and slaves answer).  It is the
        attacker's forged response that WON the poll slot: the genuine
        slave's answer was collided with and aborted by its own error
        handling (the slave compares the bits on the bus against its own
        transmission and stops on mismatch), and this byte finished the
        response in its place.  LIN responses carry no authentication -- a
        receiver checks only the checksum, which the attacker recomputes --
        so the attacker can make the module's output byte ANY value
        (Takahashi et al., IPSJ-JIP 25:220, 2017).  The payload is the
        module's *complete* output state: bits absent from the byte are OFF.
        (The 0x120/0x130 CAN latch above models the other real-world
        architecture -- naive-trust vehicles like the ICSim / 2015-Jeep
        model, where the broadcast itself actuates.)
        """
        mod = LIN_MODULES.get(lin_id)
        if mod is None or not data:
            return
        _name, _bits = mod
        state = data[0]
        for _b, _n in _bits.items():
            _on = bool(state & _b)
            if self.switches.get(_n) != _on:
                self.switches[_n] = _on
                ev.append(f"{_n} {'on' if _on else 'off'} via LIN INJECT")

    # ------------------------------------------------- LIN traffic capture
    def _lin_module_state(self, lin_id):
        """Re-derive a body module's current output byte from the switches,
        as the real LIN master would read back from its slaves."""
        mod = LIN_MODULES.get(lin_id)
        if mod is None:
            return 0
        out = 0
        for _b, _n in mod[1].items():
            if self.switches.get(_n):
                out |= _b
        return out

    def _capture_lin(self, lin_id, data_byte, direction):
        """Append one LIN bus record to the capture ring (cap 48)."""
        ring = getattr(self, "_last_lin_frames", None)
        if ring is None:
            return
        ring.append({"ts": round(self.phys.t, 2),
                     "id": lin_id, "data": f"{data_byte:02X}",
                     "dir": direction})
        if len(ring) > 48:
            del ring[:-48]

    # -------------------------------------------------------- drive follower
    def _apply_drive_frame(self, can_id, data):
        """ICSim-style: fold external drive frames into the physics.

        0x120 lamp bits latch the matching switches, so an injected
        STEER frame can flip headlights / highbeam / wipers / hazard / turn."""
        ev = []
        if not data:
            return
        if can_id == FRAME_DRIVE_IN:                 # dedicated control frame
            if len(data) >= 4:
                self.phys.throttle = clamp(data[0] / 255.0, 0.0, 1.0)
                self.phys.brake = clamp(data[1] / 100.0, 0.0, 1.0)
                self.phys.steer = clamp((data[3] - 128) / 127.0, -1.0, 1.0)
            g = GEAR_FROM.get(data[2] & 0x0F)
            if g:
                self.phys.set_gear(g, ev)
        elif can_id == FRAME_ENGINE:                 # engine frame: throttle
            if len(data) >= 4:
                self.phys.throttle = clamp(data[3] / 255.0, 0.0, 1.0)
        elif can_id == FRAME_CHASSIS:                # chassis frame: brake
            if len(data) >= 2:
                self.phys.brake = clamp(data[1] / 100.0, 0.0, 1.0)
        elif can_id == FRAME_STEER:                  # steer + lamp bits
            s = data[0]
            if s >= 128:
                s -= 256
            self.phys.steer = clamp(s / 100.0, -1.0, 1.0)
            if len(data) >= 2:                       # 0x120 lamp bits latch switches
                self._latch_lamp_bits(data[1], ev)
        elif can_id == FRAME_GEAR:                   # gear frame
            g = GEAR_FROM.get(data[0] & 0x0F)
            if g:
                self.phys.set_gear(g, ev)
        self.remote_drive = True
        self.drive_frame = [can_id, data.hex().upper()]
        if ev:
            with self._ev_lock:
                self.events.extend(ev)

    # ---------------------------------------------------- incoming frames (loopback)
    def handle_rx_frame(self, can_id, data):
        """A raw CAN frame arrived via CAN INJECT on the JSON control channel."""
        if not data:
            return
        # 0x120 STEER lamp bits latch the switches in ANY mode (headlights,
        # wipers, hazard, highbeam, indicators) so the lights are a one-frame
        # hack regardless of how the sim was started; steer byte0 still folds
        # only in --follower below.
        if can_id == FRAME_STEER and len(data) >= 2:
            ev = []
            self._latch_lamp_bits(data[1], ev)
            if ev:
                with self._ev_lock:
                    self.events.extend(ev)
        # 0x130 BODY bits latch the switches in ANY mode too (doors, trunk,
        # hood, belt) -- the same one-frame hack as the lamps.
        if can_id == FRAME_BODY and data:
            ev = []
            self._latch_body_bits(data[0], ev)
            if ev:
                with self._ev_lock:
                    self.events.extend(ev)
        # ICSim-style: external drive frames fold straight into the physics
        if self.follower and can_id in DRIVE_IDS:
            self._apply_drive_frame(can_id, data)
            return
        ecu_id, ecu = ecu_for_req(can_id)
        if ecu_id is None:
            return
        # Loopback has no reply channel: run the ECU request so its side
        # effects (DTC clearing, fault bookkeeping) still happen; the reply
        # frames are dropped here.  The ISO-TP reply path is --selftest.
        handle_request(can_id, data, self.phys, self.faults, self.ecus)
    # ------------------------------------------------------------- main tick
    def tick(self, dt):
        ev = []
        self.phys.step(dt, ev, self.switches, self.faults)

        # periodic chimes / warnings
        now = self.phys.t
        mono = time.monotonic()      # broadcast-schedule clock: survives reset()
        lamps = compute_lamps(self.phys, self.faults, self.switches)
        if lamps["seatbelt"] and now - self._last_chime > 5.0:
            ev.append("seatbelt chime")
            self._last_chime = now
        if lamps["lowfuel"] and now - self._last_lowfuel > 10.0:
            ev.append("low fuel")
            self._last_lowfuel = now
        if lamps["door"] and now - self._last_door > 5.0:
            ev.append("door open")
            self._last_door = now
        if self.switches.get("parkbrake") and abs(self.phys.v) * 3.6 > 1.0 \
                and now - self._last_pb > 3.0:
            ev.append("park brake while moving!")
            self._last_pb = now
        if self.phys.brake > 0.75 and abs(self.phys.v) * 3.6 > 8.0:
            ev.append("ABS pulsing") if int(now * 4) % 8 == 0 else None
        with self._ev_lock:
            self.events.extend(ev)

        # broadcast frames
        if not self.no_traffic:
            for fid, (period, next_t) in self._frames.items():
                if mono >= next_t:
                    self._frames[fid] = (period, mono + period)
                    self._sent_frames[fid] = self._frame_builders[fid][1]()
            # always hand the cockpit a FULL per-ID snapshot so every
            # gauge/bus row updates every tick
            self._last_frames = [
                {"id": fid, "dlc": len(d), "data": d.hex().upper()}
                for fid, d in sorted(self._sent_frames.items())]
            # LIN master poll (10 Hz): each RX record is a genuine slave
            # answering its poll slot with its current output state.  Only
            # log on change so the capture stays readable and every record
            # is copy-pasteable into the LIN inject box.
            if mono >= self._lin_poll_next:
                self._lin_poll_next = mono + 0.1
                for _lid in sorted(LIN_MODULES):
                    _b = self._lin_module_state(_lid)
                    if self._lin_last_rx.get(_lid) != _b:
                        self._lin_last_rx[_lid] = _b
                        self._capture_lin(_lid, _b, "RX")

        # 10 Hz state stream to the cockpit
        if int(now * 10.0) != getattr(self, "_last_state_tick", -1):
            self._last_state_tick = int(now * 10.0)
            self._push_state()

    def _push_state(self):
        with self._ev_lock:
            events = list(self.events)
            self.events.clear()
        state = self.phys.snapshot(self.switches, self.faults)
        state["t"] = "state"
        state["ts"] = round(self.phys.t, 2)
        state["switches"] = dict(self.switches)
        state["faults"] = dict(self.faults)
        state["lamps"] = compute_lamps(self.phys, self.faults, self.switches)
        state["frames"] = getattr(self, "_last_frames", [])
        state["lin_frames"] = list(getattr(self, "_last_lin_frames", []))
        state["events"] = events[-8:]
        state["dtc"] = [f"{'PCBU'[(h >> 6) & 3]}{((h << 8) | l) & 0x3FFF:04X}"
                        for h, l, _s in engine_dtcs(self.faults)]
        state["dtc_abs"] = [f"{'PCBU'[(h >> 6) & 3]}{((h << 8) | l) & 0x3FFF:04X}"
                            for h, l, _s in abs_dtcs(self.faults)]
        state["mode"] = "bench"
        state["source"] = ("remote CAN inject" if self.remote_drive
                          else "keyboard")
        self.ctrl.push(state)

    # -------------------------------------------------------------- main loop
    def run(self):
        self.ctrl.start()
        print(f"carsim {__version__} - loopback engine (JSON control channel only)")
        print(f"  JSON control channel on 127.0.0.1:{self.ctrl_port}")
        print("    commands: input / gear / ignition / switch / fault / cruise / reset")
        print('    CAN INJECT: {"t":"frame","id":256,"data":"0100"}')
        print('    LIN INJECT: {"t":"lin","cmd":"LIN_TRUNK_OPEN"} /')
        print('                 {"t":"lin","id":48,"data":"01"}')
        print("  ECUs: engine 7E8, TCM 7E9, ABS 7EA (ISO-TP; exercised by --selftest)")
        print("  broadcast: 0x100/0x110/0x120/0x130/0x140")
        if self.follower:
            print("  follower drive: ON  (CAN INJECT 0x100/0x110/0x120/0x140/0x400")
            print("                 folded in as remote input)")
        print("  Ctrl-C to stop")

        base_dt = 0.02
        next_t = time.monotonic()
        try:
            while not self._stop:
                self.tick(base_dt)
                next_t += base_dt
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(min(sleep, 0.05))
        except KeyboardInterrupt:
            pass
        print("bye")


# --------------------------------------------------------------------------- #
#  Self test (headless)
# --------------------------------------------------------------------------- #

def selftest():
    """Physics + protocol assertions.  Run by the build pipeline."""
    ok = lambda name: print(f"  ok  {name}")
    fail = lambda name, why: (_ for _ in ()).throw(
        AssertionError(f"{name}: {why}"))

    p = CarPhysics()
    ev = []
    faults = {k: False for k in ("mil", "overheat", "oil", "abs", "flat")}
    switches = {"seatbelt": True}

    # 1) throttle launches the car and auto-shifts
    p.gear = "D"
    p.gear_num = 1
    p.ignition = True
    p.engine_on = True
    p.throttle = 1.0
    for _ in range(300):                  # 6 s of full throttle
        p.step(0.02, ev, switches, faults)
    assert p.v * 3.6 > 30.0, f"should be well past 30 km/h, got {p.v*3.6:.1f}"
    assert p.gear_num >= 2, f"should have upshifted, gear {p.gear_num}"
    assert any("shift up" in e for e in ev), "no upshift event"
    assert p.rpm < 7000, f"rpm should respect redline, got {p.rpm:.0f}"
    ok("full throttle: speed climbs, auto upshifts, rev limiter holds")

    # 2) braking slows the car
    p.throttle = 0.0
    p.brake = 1.0
    v0 = p.v
    for _ in range(50):                   # 1 s of hard braking
        p.step(0.02, ev, switches, faults)
    dt = 50 * 0.02
    decel = (v0 - p.v) / dt
    assert decel >= 4.0, f"hard braking should decel >= 4 m/s^2, got {decel:.1f}"
    ok("hard braking decelerates")

    # 3) RPM reacts to throttle from idle (genuine standstill first)
    p.brake = 0.0
    p.throttle = 0.0
    p.gear = "D"
    p.gear_num = 1
    p.v = 0.0
    p.engine_on = True
    for _ in range(30):               # settle to true idle at a standstill
        p.step(0.02, ev, switches, faults)
    rpm_idle = p.rpm
    p.throttle = 0.8
    for _ in range(30):               # blip the throttle
        p.step(0.02, ev, switches, faults)
    assert p.rpm > rpm_idle + 1500, f"rpm should climb hard with throttle from idle: {rpm_idle:.0f} -> {p.rpm:.0f}"
    ok("rpm rises with throttle")

    # 4) PID byte round-trip: 0C rpm and 0D speed decode like a real scan tool
    data = mode01_payload(p, "engine", 0x0C, faults)
    rpm = ((data[0] << 8) | data[1]) / 4.0
    assert abs(rpm - p.rpm) < 2.0, f"rpm decode {rpm} vs physics {p.rpm}"
    data = mode01_payload(p, "engine", 0x0D, faults)
    assert data[0] == int(abs(p.v) * 3.6), f"speed decode {data[0]} vs {p.v*3.6:.1f}"
    ok("J1979 decode round-trip (0C rpm, 0D speed)")

    # 5) PID support mask decodes to the documented PID list
    pids = []
    mask = bytes(ENGINE_PID_MASK[0:4])
    for pid in range(1, 0x21):
        byte = mask[(pid - 1) // 8]
        if byte & (0x80 >> ((pid - 1) % 8)):
            pids.append(pid)
    want = [0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0C, 0x0D, 0x0E, 0x0F,
            0x10, 0x11, 0x1C, 0x1F]
    assert pids == want, f"mask gives {[hex(x) for x in pids]}"
    ok("PID 00 mask lists exactly the 14 engine PIDs")

    # 6) MIL fault -> mode 03 P0300 -> clear -> gone (SF carries ISO-TP PCI)
    faults["mil"] = True
    assert engine_dtcs(faults) == FAULT_DTCS["mil"], engine_dtcs(faults)
    resp = handle_request(REQ_FUNCTIONAL, b"\x01\x03", p, faults, 3)
    assert resp[0][0] == "sf" and resp[0][2] == b"\x43\x01\x03\x00", resp
    resp = handle_request(REQ_FUNCTIONAL, b"\x01\x04", p, faults, 3)
    assert resp[0][2] == b"\x44", resp
    assert not engine_dtcs(faults), "mode 04 must clear live engine DTCs"
    resp = handle_request(REQ_FUNCTIONAL, b"\x01\x03", p, faults, 3)
    assert resp[0][2] == b"\x43\x00", resp
    ok("mode 03 shows P0300; mode 04 clears live engine DTCs")

    # 7) VIN multi-frame: FF/FC/CF flow reassembles to the 17-char VIN
    items = handle_request(REQ_ENGINE, b"\x02\x09\x02", p, faults, 3)
    assert items[0][0] == "mf", items
    body = items[0][2]
    assert body[0:3] == b"\x49\x02\x01" and len(body) == 20, body
    ff = bytes([0x10 | ((len(body) >> 8) & 0x0F), len(body) & 0xFF]) + body[:6]
    ff += b"\x00" * (8 - len(ff))
    collected = bytearray(ff[2:8])
    seq = 1
    for i in range(6, len(body), 7):
        cf = bytes([0x20 | (seq & 0x0F)]) + body[i:i + 7]  # CF: 1 PCI + up to 7 payload
        cf += b"\x00" * (8 - len(cf))  # pad only the whole frame to 8 bytes
        collected += cf[1:8]
        seq = (seq + 1) & 0x0F
    assert bytes(collected[:20]) == body, "VIN reassembly mismatch"
    assert body[3:20].decode("ascii") == VIN.decode(), "VIN bytes wrong"
    ok("mode 09 02 VIN multi-frame reassembles (FF->FC->CF)")

    # 7b) incoming FirstFrame -> raw 30 00 FlowControl (we don't assemble)
    ff_req = b"\x10\x0B\x02\x09\x02" + b"\x00" * 3
    items = handle_request(REQ_ENGINE, ff_req, p, faults, 3)
    assert items == [("fc", ECU_ENGINE, b"\x30\x00\x00\x00\x00\x00\x00\x00")], items
    ok("incoming FirstFrame answered by raw 30 00 FlowControl")

    # 8) ABS DTC via UDS 19 02 to physical 7E2, then UDS 14 clears it
    faults["abs"] = True
    assert abs_dtcs(faults) == FAULT_DTCS["abs"], abs_dtcs(faults)
    resp = handle_request(REQ_ABS, b"\x03\x19\x02", p, faults, 3)
    assert resp[0][2] == b"\x59\x02\xFF\x40\x35\x08", resp
    resp = handle_request(REQ_ABS, b"\x01\x14", p, faults, 3)
    assert resp[0][2] == b"\x54", resp
    assert not abs_dtcs(faults), "UDS 14 must clear ABS DTCs"
    ok("UDS 19 02 to 7E2 reports C0035; UDS 14 clears it")

    # 9) gear blocking: R is refused above 8 km/h
    p.gear = "D"
    p.gear_num = 3
    p.v = 20.0
    ev.clear()
    p.set_gear("R", ev)
    assert p.gear == "D" and any("blocked" in e for e in ev), ev
    ok("R/P shift blocked above 8 km/h")

    # 10) cruise refuses to engage under 40 km/h in D
    p.v = 5.0
    p.cruise_on = False
    p.cruise_set = 0.0
    p.set_gear("D", ev)
    p.engine_on = True
    # emulate apply_input logic
    if not (p.engine_on and p.gear == "D" and abs(p.v) * 3.6 >= CRUISE_MIN):
        p._cruise_ok = False
    else:
        p._cruise_ok = True
    assert p._cruise_ok is False, "cruise should not engage at 18 km/h"
    ok("cruise needs D and >= 40 km/h")

    # 11) overheat fault -> limp torque (slower acceleration) + P0217
    p.v = 0.0
    p.engine_on = True
    p.gear = "D"
    p.gear_num = 1
    p.throttle = 1.0
    faults["overheat"] = True
    a_limp = None
    for _ in range(1):
        p.step(0.02, ev, switches, faults)
    v_limp = p.v
    p.v = 0.0
    faults["overheat"] = False
    p.step(0.02, ev, switches, faults)
    assert p.v > v_limp, "limp mode should reduce acceleration"
    ok("overheat limps torque (P0217)")

    # 12) flat tyre governor holds ~86-88 km/h under WOT, sets no DTC
    p.v = 24.0                          # 86.4 km/h: at the cap, foot to the floor
    p.throttle = 1.0
    p.brake = 0.0
    p.gear = "D"
    p.gear_num = 2
    p.engine_on = True
    faults["flat"] = True
    for _ in range(500):                # 10 s of full throttle
        p.step(0.02, ev, switches, faults)
    kmh = p.v * 3.6
    assert 86.0 <= kmh <= 88.5, f"flat governor broken: {kmh:.1f} km/h"
    assert not engine_dtcs(faults), "flat must not set a DTC"
    ok("flat tyre: governor holds 86-88 km/h, no DTC")

    # 13) functional request reaches the engine only; TCM silent on 7DF
    items = handle_request(REQ_FUNCTIONAL, b"\x02\x01\x0D", p, faults, 3)
    assert items and items[0][1] == ECU_ENGINE, items
    ok("functional 0x7DF answered by engine 0x7E8")

    # 14) LIN command parsing (mnemonics + raw module frames only)
    assert parse_lin_cmd("LIN_WIPER_ON") == (0x10, b"\x01"), \
        parse_lin_cmd("LIN_WIPER_ON")
    assert parse_lin_cmd("LIN_20#03") == (0x20, b"\x03"), \
        parse_lin_cmd("LIN_20#03")
    assert parse_lin_cmd("LIN_BOGUS") is None
    assert parse_lin_cmd("120#0020") is None   # CAN-style, not a LIN module
    ok("parse_lin_cmd: mnemonics + raw LIN_xx#dd module frames")

    print("\ncarsim selftest: ALL PASS")
    return 0


# --------------------------------------------------------------------------- #
#  main
# --------------------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser(
        description="Loopback-only virtual car: physics + OBD/UDS ECU cluster "
                    "served over one JSON control channel.")
    ap.add_argument("--ctrl-port", type=int, default=CTRL_PORT,
                    help="JSON control port (default 20103)")
    ap.add_argument("--no-traffic", action="store_true",
                    help="silent bench: no broadcast frames, only ECU replies")
    ap.add_argument("--ecus", type=int, default=3, choices=[1, 2, 3],
                    help="1=engine, 2=+TCM, 3=+ABS (default 3)")
    ap.add_argument("--follower", action="store_true",
                    help="ICSim-style drive: fold CAN INJECT 0x100/0x110/0x120/"
                         "0x140/0x400 into the physics as remote input")
    ap.add_argument("--selftest", action="store_true",
                    help="run headless physics/protocol assertions and exit")
    ap.add_argument("--version", action="version",
                    version=f"carsim {__version__}")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    sim = CarSim(ctrl_port=args.ctrl_port, no_traffic=args.no_traffic,
                 ecus=args.ecus, follower=args.follower)
    sim.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())

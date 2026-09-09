#!/usr/bin/env python3
"""Unit test for the road-renderer lane-drift gate (Cockpit._render_road).

Real-car semantics: with the engine running in Park the front wheels CAN
be steered (and the 0x120 STEER CAN frame keeps flowing), but the car body
cannot change lanes.  The renderer must only integrate steer into the
lateral road position while the car is actually moving; at standstill
_car_lat must stay frozen no matter the steer input (autopilot sway,
manual arrow keys, or injected CAN frames).

Runs headless: _render_road only needs road_cv / _car_lat / _scroll /
_state / units on the Cockpit instance, so a __new__-based instance with a
mock canvas is sufficient -- no DISPLAY, no Tk.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from tools.carsim_gui import Cockpit  # noqa: E402


class MockCanvas(dict):
    """dict-like canvas: cv["width"]/cv["height"] + no-op draw calls."""

    def __getattr__(self, name):
        if name.startswith("create_") or name == "delete":
            def _noop(*args, **kwargs):
                return None
            return _noop
        raise AttributeError(name)


def make_cockpit():
    c = Cockpit.__new__(Cockpit)
    c._car_lat = 0.0
    c._scroll = 0.0
    c._state = {"ts": 0.0}
    c.units = "kmh"
    c.road_cv = MockCanvas(width=800, height=600)
    return c


def main():
    fail = []

    def check(name, ok):
        print(("  ok " if ok else "  FAIL ") + name)
        if not ok:
            fail.append(name)

    # parked (0 km/h): steer input must NOT move the car laterally
    c = make_cockpit()
    c._render_road({"speed": 0.0, "steer": 50.0, "switches": {}})
    check("parked: steer 50% -> _car_lat frozen at 0.0",
          c._car_lat == 0.0)

    c = make_cockpit()
    c._render_road({"speed": 0.0, "steer": -50.0, "switches": {}})
    check("parked: steer -50% -> _car_lat frozen at 0.0",
          c._car_lat == 0.0)

    # moving: steering still drifts lanes as before (rate 12 px per frame)
    c = make_cockpit()
    c._render_road({"speed": 60.0, "steer": 50.0, "switches": {}})
    check("moving 60 km/h: steer 50% -> +6.0 px drift",
          abs(c._car_lat - 6.0) < 1e-9)

    c = make_cockpit()
    c._render_road({"speed": 60.0, "steer": -50.0, "switches": {}})
    check("moving 60 km/h: steer -50% -> -6.0 px drift",
          abs(c._car_lat - (-6.0)) < 1e-9)

    # creep below the threshold: no lane change (matches real low-speed feel)
    c = make_cockpit()
    c._render_road({"speed": 0.5, "steer": 100.0, "switches": {}})
    check("creep 0.5 km/h: full steer -> still frozen",
          c._car_lat == 0.0)

    if fail:
        print(f"renderer gate test: {len(fail)} FAILED -> " + ", ".join(fail))
        return 1
    print("renderer gate test: ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

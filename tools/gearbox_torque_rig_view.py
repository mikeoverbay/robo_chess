#!/usr/bin/env python3
"""
gearbox_torque_rig_view.py - show the torque test rig set up on the bench, in the "gearbox" Robot Studio.

Mike 2026-11-14: "Do we have a model of the arm to test on the scale?" -> "Show it".
The rig is the arm session's (torque_test.py -> print/Torque test arm.step, print/Torque test stand.step); this only
puts a view copy together, nothing here is printed:
  gearbox print file (print/Gearbox assembly (print 4).step) + the 17HS19 stand-in (cycloid.motor_body / motor_shaft)
  + the test arm + the stand + a 3 x 20 dowel in the 220 hole, stood up as on the bench (axis level, foot down).

    python gearbox_torque_rig_view.py    -> viewer_sessions/gearbox/torque_rig_upright.step, added to that studio's
                                            scene.json (replaces an earlier copy)
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import cycloid as C  # noqa: E402
from view_step import read_step  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402

PRINT = os.path.join(os.path.dirname(HERE), "print")
GEARBOX = os.path.join(PRINT, "Gearbox assembly (print 4).step")
ARM = os.path.join(PRINT, "Torque test arm.step")
STAND = os.path.join(PRINT, "Torque test stand.step")
STUDIO = os.path.join(HERE, "viewer_sessions", "gearbox")
OUT = os.path.join(STUDIO, "torque_rig_upright.step")
NAME = "gearbox torque test rig"   # "gearbox ..." so the studio's gearbox colour rules apply to its parts

BENCH = -53.0                      # drive frame: the stand's foot underside (y -(45 + 8)), axis on z
PIN_X, PIN_L = 220.0, 20.0         # the dowel in the longest lever hole, out past the bar's outer face
Z_OUT, BAR_T = 59.2, 8.0
FLOOR_Y = 257.0                    # the studio's floor; up is -y
AT = [600.0, FLOOR_Y + BENCH, 0.0]  # right of the pin gauge (x 460); the foot on the floor


def main():
    parts = []
    for n, s, _ in read_step(GEARBOX):
        parts.append((n.replace("Gearbox ", "", 1), s))
    parts.append(("motor body (17HS19, 48 long)", C.motor_body()))
    parts.append(("motor shaft", C.motor_shaft()))
    for path in (ARM, STAND):
        for n, s, _ in read_step(path):
            parts.append((n, s))
    parts.append(("dowel 3 x 20 (lever pin, 220 hole)",
                  cq.Solid.makeCylinder(1.5, PIN_L, V(PIN_X, 0, Z_OUT), V(0, 0, 1))))
    # stand it up: drive +y (from the bench to the axis) -> studio -y (up); 180 about x keeps the arm along +x
    up = [(n, s.rotate(V(0, 0, 0), V(1, 0, 0), 180), None) for n, s in parts]
    os.makedirs(STUDIO, exist_ok=True)
    write_flat(up, OUT)
    scene = os.path.join(STUDIO, "scene.json")
    sc = json.load(open(scene, encoding="utf-8"))
    sc["models"] = [m for m in sc["models"] if m.get("path") != OUT]
    sc["models"].append({"path": OUT, "name": NAME, "offset": AT})
    rule = {"match": f"^{NAME} / dowel", "color": "#c9ccd2", "by": "gearbox_torque_rig_view.py"}
    rules = [r for r in sc.get("colors", []) if r.get("by") != rule["by"]]
    sc["colors"] = [rule] + rules                  # first match wins: ahead of the catch-all
    open(scene, "w", encoding="utf-8").write(json.dumps(sc, indent=1))
    print(f"wrote {OUT} ({len(up)} bodies), added '{NAME}' at {AT}")


if __name__ == "__main__":
    main()

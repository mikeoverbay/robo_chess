"""gearbox_pin_gauge.py - a printed go gauge for the housing's 38 ring pins (Robo Chess gearbox, 2026-11-12).

Mike, relayed by "Robot arm control and kinematics": "I need a model made to see if the fit of the pins is okay ... an
inner print part that I can see if it'll fit between all the pins ... For all the 38 pins. And I'll print it, measure
it, get it to size" / "Put some screw holes around the outside for a number 4 screw so I can screw them in it so I can
pull this thing back out".

A ring that fills in between the pins: 38 scallops of radius 1.5 + CLEAR on the pin centres (r 28), lands out to TIP_R
between them (the housing wall is at 27.6). It slides down over the pins only if every pin is within CLEAR of where the
model puts it, inward or sideways. CLEAR 0.10 is the disc's own least gap to the pins, so a pin the gauge passes can't
bind the disc. A pin sitting outward passes: that is backlash, not a bind.

Check the print before using it: two 3 mm dowels in opposite scallops measure 59 - 2 x CLEAR over the dowels (58.80),
and opposite lands measure 2 x TIP_R across (54.60). The bed side has a LEAD chamfer on the scallops and lands, which
also takes off the elephant foot: that side goes in first. The #4 holes are D2.3: the #43 tap drill for 4-40, and a
pilot for a #4 thread-forming screw.

    python gearbox_pin_gauge.py [--clear 0.10] [--check]
        -> print/Gearbox pin gauge.step, and an upright copy shown in the "gearbox" studio
"""
import argparse
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

import cycloid as C  # noqa: E402
from gearbox_step import cyl, cut_all, one_solid  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402

CLEAR = 0.10        # scallop radius over the pin's 1.5: the disc's least gap to the pins
THICK = 6.0         # as thick as one disc
TIP_R = 27.3        # lands between the pins: 0.3 inside the housing wall (27.6)
BORE = 30.0         # open middle
SCREW_D = 2.3       # #4: 4-40 tap drill (#43, 2.26) / #4 thread-forming pilot
SCREW_R = 21.0      # "screw holes around the outside"
SCREWS = (0.0, 90.0, 180.0, 270.0)
LEAD = 0.4          # 45 deg lead-in on the bed side's scallops and lands

PRINT = os.path.join(os.path.dirname(HERE), "print", "Gearbox pin gauge.step")
STUDIO = os.path.join(HERE, "viewer_sessions", "gearbox")
UPRIGHT = os.path.join(STUDIO, "pin_gauge_upright.step")
VIEW_AT = [460.0, 257.0, 0.0]   # on the floor (y 257), beside the exploded gearbox at x 380


def outline(clear, grow, z0, h):
    """the scalloped disc (no bore), its outline moved in by grow: lands smaller, scallops bigger"""
    d = C.PIN_D + 2 * (clear + grow)
    return cut_all(cyl(2 * (TIP_R - grow), h, z0), [cyl(d, h + 2, z0 - 1, *C.polar(C.R_PINS, a))
                                                     for a in C.ring_pin_angles()])


def bottom_wire(s):
    z = s.BoundingBox().zmin
    return next(f for f in s.Faces() if abs(f.Center().z - z) < 1e-6 and f.normalAt().z < -0.999).outerWire()


def gauge(clear=CLEAR):
    # OCC's edge chamfer fails on the 76 sharp scallop/land corners, so the lead-in is a ruled loft from the outline
    # moved in by LEAD at the bed to the full outline LEAD up: a 45 deg lead-in all the way round
    body = outline(clear, 0.0, LEAD, THICK - LEAD)
    if LEAD > 0:
        lead = cq.Solid.makeLoft([bottom_wire(outline(clear, LEAD, 0.0, 1.0)), bottom_wire(body)], True)
        assert lead.isValid(), "gauge: lead-in loft not valid"
        body = one_solid(body.fuse(lead).clean())
    cuts = [cyl(BORE, THICK + 2, -1)] + [cyl(SCREW_D, THICK + 2, -1, *C.polar(SCREW_R, a)) for a in SCREWS]
    body = cut_all(body, cuts)
    assert BRepCheck_Analyzer(body.wrapped).IsValid(), "gauge: not a valid solid"
    return body


def overlap(a, b):
    return a.intersect(b).Volume()


def check(g):
    """Put the gauge in the housing at disc 1's height and move one pin inward: it must bind past CLEAR."""
    C.set_spread(14)
    z = C.Z_D1
    h = C.housing(False)
    gz = g.translate(V(0, 0, z))
    pin = lambda a, dr=0.0, dt=0.0: cyl(C.PIN_D, C.PIN_LEN, C.Z_PIN0, *C.polar(C.R_PINS - dr, a)).rotate(
        V(0, 0, 0), V(0, 0, 1), dt)
    print(f"  housing wall: overlap {overlap(gz, h):.4f} mm^3")
    tot = sum(overlap(gz, pin(a)) for a in C.ring_pin_angles())
    print(f"  38 pins where the model puts them: overlap {tot:.4f} mm^3")
    for dr in (CLEAR - 0.02, CLEAR + 0.02):
        print(f"  one pin {dr:.2f} inward: overlap {overlap(gz, pin(0.0, dr)):.4f} mm^3")
    for dd in (CLEAR - 0.02, CLEAR + 0.02):
        dt = (dd / C.R_PINS) * 180 / 3.141592653589793
        print(f"  one pin {dd:.2f} sideways: overlap {overlap(gz, pin(0.0, 0.0, dt)):.4f} mm^3")


def show(g):
    os.makedirs(STUDIO, exist_ok=True)
    write_flat([(NAME, g.rotate(V(0, 0, 0), V(1, 0, 0), 90), cq.Color(0.95, 0.55, 0.15))], UPRIGHT)
    scene = os.path.join(STUDIO, "scene.json")
    sc = json.load(open(scene, encoding="utf-8"))
    sc["models"] = [m for m in sc["models"] if m.get("path") != UPRIGHT]
    sc["models"].append({"path": UPRIGHT, "name": "gearbox pin gauge (38 pins)", "offset": VIEW_AT})
    open(scene, "w", encoding="utf-8").write(json.dumps(sc, indent=1))


def main():
    global NAME, CLEAR
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clear", type=float, default=CLEAR, help="scallop radius over the pin (0.10 = the disc's gap)")
    ap.add_argument("--check", action="store_true", help="fit it in the housing and prove it catches a pin 0.12 out")
    ap.add_argument("--no-show", action="store_true", help="don't add it to the gearbox studio")
    a = ap.parse_args()
    CLEAR = a.clear
    NAME = (f"Gearbox pin gauge: 38 x {C.PIN_D:g} mm pins on D{2 * C.R_PINS:g}, {CLEAR:.2f} clearance, "
            f"{THICK:g} thick, {2 * TIP_R:g} across the lands, 4 x #4 holes")
    g = gauge(CLEAR)
    write_flat([(NAME, g, cq.Color(0.95, 0.55, 0.15))], PRINT)
    print(f"{NAME}\n  {g.Volume():.0f} mm^3 -> {PRINT}")
    print(f"  check the print: over two 3 mm dowels in opposite scallops {2 * C.R_PINS + C.PIN_D - 2 * CLEAR:.2f}; "
          f"across opposite lands {2 * TIP_R:.2f}")
    if a.check:
        check(g)
    if not a.no_show:
        show(g)
        print(f"  shown in the gearbox studio at {VIEW_AT}")


NAME = ""
if __name__ == "__main__":
    main()

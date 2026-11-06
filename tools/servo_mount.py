#!/usr/bin/env python3
"""
servo_mount.py - the claw's cable servo on link A, by the first knuckle (J2), mark-up version (2026-11-04, Mike: "a
servo motor with an arm on it", mounted on the body, centre line, first knuckle).

    python servo_mount.py

Servo: a standard-size metal-gear servo (MG996R ~10 kg.cm or DS3218 ~20 kg.cm - same body). Stand-in from the usual
published dimensions; MEASURE the real one and update the SERVO_* numbers before printing the bracket.
Load: claw needs ~16 N to open, ~35-40 N at the servo end after friction in the PTFE; a 15 mm horn arm needs ~0.6 N.m
(6 kg.cm) and turns ~23 deg for the claw's 6 mm of cable travel.

Placement (link A's design frame = the arm's: link along -x from J1 (x 0) to J2 (x -200); its outside "top" wall face
is y = -35.25 (design -y is world up when the arm is level); the arm's centre line is z = 30.6):
  - bracket: a plate on that wall face, bolted through the wall with 3 x M3 flat heads (countersunk, flush under the
    servo) + nuts inside the link (before the lid goes on), on the one band that is solid all along: z 36..41 (above
    the chevron slots, below the lid-screw nubs), and between the nubs (x -144/-112/-80/-48 +-4); a cradle for the
    servo; a tube stop on the J2 end for the Bowden (4 mm PTFE).
  - servo lying on the plate along the link, shaft pointing away from the wall (world up), horn swinging over the link;
    the cable runs from the horn tip out to the tube stop and on over J2.
Out: complete assembly/servo_mount.step (servo, horn, bracket) - parts ride with link A (joint J1) in Robot Studio.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "complete assembly", "servo_mount.step")

# ---- servo (standard size; measure yours)
SERVO_L, SERVO_W, SERVO_H = 40.7, 19.7, 37.0      # case
FLANGE_L, FLANGE_T, FLANGE_Z = 54.5, 2.5, 27.0    # mounting tabs: overall length, thickness, underside above the case bottom
FLANGE_HOLE_D, FLANGE_HOLES_X, FLANGE_HOLES_W = 4.4, 49.5, 10.0
SHAFT_FROM_END = 10.2                             # output shaft from the near end of the case
TURRET_D, TURRET_H, SPLINE_D, SPLINE_H = 12.0, 3.0, 5.8, 4.0
HORN_R, HORN_T, HORN_HUB_D, HORN_W = 15.0, 2.5, 9.0, 6.0

# ---- placement on link A
WALL_Y = -35.25            # outside face of link A's top wall
Z_C = 30.6                 # centre line
PLATE_T = 4.0
PLATE_X = (-152.0, -55.0)  # along the link (walls end at -152.5)
PLATE_Z = (12.0, 49.2)
BOLT_D = 3.4
BOLTS = [(-130.0, 38.5), (-96.0, 38.5), (-64.0, 38.5)]     # (x, z): solid band, clear of the nubs
SERVO_X0 = -140.0          # case end toward J2; the shaft sits at the other (J1) end, cable pulls toward J1
CRADLE_T = 3.0
STOP_X = (-152.0, -147.5)  # tube stop on the plate's J2 end, 1 clear of the servo tab (4 mm PTFE through: glue it)
TUBE_D = 4.2


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} pieces"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def ycyl(d, y0, y1, x, z):
    """cylinder along -y (away from the wall) from y0 to y1 (y1 < y0)."""
    return cq.Solid.makeCylinder(d / 2, y0 - y1, V(x, y0, z), V(0, -1, 0))


def parts():
    yb = WALL_Y - PLATE_T                        # servo case bottom (on the plate)
    x0, x1 = SERVO_X0, SERVO_X0 + SERVO_L
    z0, z1 = Z_C - SERVO_W / 2, Z_C + SERVO_W / 2
    # servo: case, tabs, turret, spline (shaft at the J1 end)
    sx = x1 - SHAFT_FROM_END
    servo = box(x0, x1, yb - SERVO_H, yb, z0, z1)
    fx0 = (x0 + x1) / 2 - FLANGE_L / 2
    servo = servo.fuse(box(fx0, fx0 + FLANGE_L, yb - FLANGE_Z - FLANGE_T, yb - FLANGE_Z, z0, z1))
    for hx in (-FLANGE_HOLES_X / 2, FLANGE_HOLES_X / 2):
        for hz in (-FLANGE_HOLES_W / 2, FLANGE_HOLES_W / 2):
            servo = servo.cut(ycyl(FLANGE_HOLE_D, yb - FLANGE_Z + 1, yb - FLANGE_Z - FLANGE_T - 1, (x0 + x1) / 2 + hx, Z_C + hz))
    servo = servo.fuse(ycyl(TURRET_D, yb - SERVO_H + 0.01, yb - SERVO_H - TURRET_H, sx, Z_C))
    servo = servo.fuse(ycyl(SPLINE_D, yb - SERVO_H - TURRET_H + 0.01, yb - SERVO_H - TURRET_H - SPLINE_H, sx, Z_C))
    servo = one(servo.clean(), "servo")
    # horn: one arm, pointing toward J2 at rest (claw closed), cable hole at HORN_R
    yh = yb - SERVO_H - TURRET_H - SPLINE_H + 1.5
    horn = ycyl(HORN_HUB_D, yh, yh - HORN_T, sx, Z_C)
    horn = horn.fuse(box(sx - HORN_R, sx, yh - HORN_T, yh, Z_C - HORN_W / 2, Z_C + HORN_W / 2))
    horn = horn.fuse(ycyl(HORN_W, yh, yh - HORN_T, sx - HORN_R, Z_C))
    horn = horn.cut(ycyl(1.6, yh + 1, yh - HORN_T - 1, sx - HORN_R, Z_C))
    horn = horn.cut(ycyl(SPLINE_D, yh + 1, yb - SERVO_H - TURRET_H - SPLINE_H, sx, Z_C))     # socket on the spline
    horn = one(horn.clean(), "horn")
    # bracket: plate on the wall + two posts under the tabs + the tube stop
    plate = box(PLATE_X[0], PLATE_X[1], yb, WALL_Y, PLATE_Z[0], PLATE_Z[1])
    for sxp in (fx0, fx0 + FLANGE_L - 7):
        plate = plate.fuse(box(sxp, sxp + 7, yb - FLANGE_Z + 0.01, yb + 0.01, z0 - 1, z1 + 1))
    yc = yh - HORN_T / 2                                                                 # cable height (horn plane)
    plate = plate.fuse(box(STOP_X[0], STOP_X[1], yc - 5, yb + 0.01, Z_C - 7, Z_C + 7))
    cuts = [box(x0 - 0.3, x1 + 0.3, yb - SERVO_H, yb + 0.2, z0 - 0.3, z1 + 0.3)]          # servo pocket clearance
    for hx in (-FLANGE_HOLES_X / 2, FLANGE_HOLES_X / 2):                                 # tab screws (M3 self-tapping)
        for hz in (-FLANGE_HOLES_W / 2, FLANGE_HOLES_W / 2):
            cuts.append(ycyl(2.5, yb - FLANGE_Z + 12, yb - FLANGE_Z - 0.5, (x0 + x1) / 2 + hx, Z_C + hz))
    k = (6.3 - BOLT_D) / 2 / 0.8693                                                      # M3 flat head, flush
    for bx, bz in BOLTS:                                                                 # through the wall
        cuts.append(ycyl(BOLT_D, WALL_Y + 6, yb - 1, bx, bz))
        cuts.append(cq.Solid.makeCone(BOLT_D / 2, 6.3 / 2, k, V(bx, yb + k - 0.01, bz), V(0, -1, 0)))
    cuts.append(cq.Solid.makeCylinder(TUBE_D / 2, 14, V(STOP_X[0] - 2, yc, Z_C), V(1, 0, 0)))   # tube along the link
    for cu in cuts:
        plate = plate.cut(cu)
    bracket = one(plate.clean(), "bracket")
    return servo, horn, bracket, (sx - HORN_R, yc, Z_C)


def main():
    servo, horn, bracket, cable_pt = parts()
    for n, s in (("servo", servo), ("horn", horn), ("bracket", bracket)):
        b = s.BoundingBox()
        print(f"{n:8s} {s.Volume() / 1000:5.1f} cm^3  x {b.xmin:.1f}..{b.xmax:.1f} y {b.ymin:.1f}..{b.ymax:.1f} z {b.zmin:.1f}..{b.zmax:.1f}")
    print(f"cable leaves the horn at {tuple(round(v, 1) for v in cable_pt)} (design frame)")
    for a, an, b_, bn in ((servo, "servo", bracket, "bracket"), (horn, "horn", bracket, "bracket"), (horn, "horn", servo, "servo")):
        v = a.intersect(b_).Volume()
        if v > 0.05:
            print(f"   OVERLAP {an} x {bn}: {v:.2f}")
    a = cq.Assembly(name="servo_mount")
    a.add(bracket, name="servo bracket", color=cq.Color(0.9, 0.6, 0.2))
    a.add(servo, name="servo (MG996R size)", color=cq.Color(0.12, 0.12, 0.14))
    a.add(horn, name="servo horn", color=cq.Color(0.95, 0.95, 0.95))
    write_step(a, OUT)
    print(OUT)


if __name__ == "__main__":
    main()

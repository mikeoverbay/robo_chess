#!/usr/bin/env python3
"""
arm_base.py - the arm's base (J1), mark-up version (2026-11-04, Mike's idea): one link's DRIVE end, cut off and stood
on end on a 100 mm square foot that screws down (4 corner #8 screws) onto the swivel's turntable (swivel_base.py).

    python arm_base.py

Design frame = the arm's (J1 axis on z at the origin; the viewer turns it so design +y is world DOWN):
  - the link (arm_2_part_chevron.step: body + top) moved so its drive end sits on J1 and turned 90 deg about J1 so the
    rest of it points down (+y); cut off at Y_CUT below J1. The drive mount, wall ends, 6808 pocket and cap hole are the
    links' own, so J1 is the same joint as J2/J3 (range 90 +- 104 deg about straight up).
  - foot: round, FOOT_D (= the swivel gear's outside diameter, Mike) x FOOT_T, under the cut, fused to the body;
    4 holes for #8 screws on r HOLE_R at the middle of each column side, countersunk on top;
    a WIRE_D hole in the middle (inside the column's channel) down into the turntable's hollow spindle.
  - drive J1 (the stand-in) turned the same 90 deg so its lid screws meet the link's mount holes.
  The board mark-up now comes from swivel_base.py (it sits on the wood under the swivel).
Out: complete assembly/arm_base.step (base body, base top, drive J1).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402
from arm2_screw_nubs import heal  # noqa: E402

ROOT = os.path.dirname(HERE)
LINK = os.path.join(ROOT, "complete assembly", "arm_2_part_chevron.step")
DRIVE = os.path.join(ROOT, "cycloid", "cycloid_drive_standin.step")
OUT_BASE = os.path.join(ROOT, "complete assembly", "arm_base.step")

Y_CUT = 160.0            # cut below J1: link x -40, just short of its cup end (cut-away starts at x -38.2)
FOOT_D, FOOT_T = 2 * (54.0 + 1.5 * math.cos(math.atan2(72, 18))), 8.0   # = swivel 72T gear OD (108.73)
Z_MID = (-4.0 + 65.2) / 2  # middle of the link's thickness: the foot (and the swivel axis) centre on it
HOLE_R = 45.0            # screw holes: column sides at 35.3 / 34.6, foot edge at 54.4 -> 5 clear each way
SCREW_CLEAR, CSK_D = 4.6, 9.2   # #8 wood screw, 82 deg flat head
WIRE_D = 20.0            # wires: column channel -> foot -> turntable spindle


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} pieces"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def place(s):
    """link frame -> base: drive end (link x -200) onto J1, the rest turned to point down (+y)."""
    return s.translate(V(200, 0, 0)).rotate(V(0, 0, 0), V(0, 0, 1), 90)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def foot_holes():
    """(x, z) of the 4 screws (design frame): on r HOLE_R at 0 / 90 / 180 / 270 deg, the middle of each column side.
    The box-link base (this file) only; the tube-link base and the turntable use foot_holes_tube()."""
    return [(HOLE_R, Z_MID), (-HOLE_R, Z_MID), (0.0, Z_MID + HOLE_R), (0.0, Z_MID - HOLE_R)]


# Mike (2026-11-05): the tube-link base's foot screws "rotated 45 degrees", off the middle of the back so the servo
# tab, its mount and gussets sit there. On r 45 at exactly 45 deg two countersinks would cut 2.8 mm into the column
# (x +-30, 3.4 off centre toward +z), so each hole keeps the 45 deg position's offset along the column side (z +-31.8)
# and moves out just clear of the column: countersink edge 0.5 off its face -> r 47.4, ~42 deg off the old spots.
TUBE_COLUMN_X = 30.0
TUBE_HOLE_X = TUBE_COLUMN_X + CSK_D / 2 + 0.5          # 35.1
TUBE_HOLE_DZ = HOLE_R * math.sin(math.radians(45.0))   # 31.8


def foot_holes_tube():
    """(x, z) of the tube-link base's 4 foot screws (design frame), near the diagonals: (+-35.1, Z_MID +- 31.8)"""
    return [(sx * TUBE_HOLE_X, Z_MID + sz * TUBE_HOLE_DZ) for sx in (1, -1) for sz in (1, -1)]


def foot():
    f = cq.Solid.makeCylinder(FOOT_D / 2, FOOT_T, V(0, Y_CUT, Z_MID), V(0, 1, 0))
    k = (CSK_D - SCREW_CLEAR) / 2 / 0.8693     # depth of the 82 deg countersink
    for x, z in foot_holes():
        f = f.cut(cq.Solid.makeCylinder(SCREW_CLEAR / 2, FOOT_T + 2, V(x, Y_CUT - 1, z), V(0, 1, 0)))
        f = f.cut(cq.Solid.makeCone(CSK_D / 2, SCREW_CLEAR / 2, k, V(x, Y_CUT, z), V(0, 1, 0)))
    return f.cut(cq.Solid.makeCylinder(WIRE_D / 2, FOOT_T + 2, V(0, Y_CUT - 1, Z_MID), V(0, 1, 0)))


def base_parts():
    link = {n: s for n, s, _ in read_step(LINK)}
    keep = box(-300, 300, -300, Y_CUT, -100, 200)
    body = one(place(link["body"]).intersect(keep).fuse(foot()).clean(), "base body")
    body = heal(body)
    top = one(place(link["top"]).intersect(keep).clean(), "base top")
    drive = [(n, s.rotate(V(0, 0, 0), V(0, 0, 1), 90)) for n, s, _ in read_step(DRIVE)]
    return body, top, drive


def main():
    body, top, drive = base_parts()
    bb = body.BoundingBox()
    print(f"base body {body.Volume() / 1000:.1f} cm^3, x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f}"
          f" z {bb.zmin:.1f}..{bb.zmax:.1f}; base top {top.Volume() / 1000:.1f} cm^3")
    for _, d in drive:
        for n, p in (("body", body), ("top", top)):
            v = d.intersect(p).Volume()
            if v > 0.5:
                print(f"   OVERLAP drive J1 x base {n}: {v:.1f} mm^3")
    print(f"J1 axis {Y_CUT + FOOT_T:.0f} above the foot's underside")
    a = cq.Assembly(name="arm_base")
    a.add(body, name="base body", color=cq.Color(0.45, 0.6, 0.85))
    a.add(top, name="base top", color=cq.Color(0.3, 0.45, 0.8))
    for n, d in drive:
        a.add(d, name=f"drive J1 {n}", color=cq.Color(0.75, 0.75, 0.78))
    write_step(a, OUT_BASE)
    print(OUT_BASE)


if __name__ == "__main__":
    main()

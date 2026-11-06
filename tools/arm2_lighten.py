#!/usr/bin/env python3
"""
arm2_lighten.py - lighter, striped versions of the split arm part (Mike, 2026-11-04): two design options.

    python arm2_lighten.py

In:  complete assembly/arm_2_part_nubs.step (body + top, with the screw nubs).
Both options:
  - the floor between the side walls comes down to FLOOR_T (4 mm), thinned from the INSIDE (a pocket from z 0 down
    to -6.8 with 45 deg ends) so the bottom stays flat and flush (first version raised the bottom instead; Mike had
    it brought back flush 2026-11-04). The plate under the elbow drive (x < -152.5, where the 6 mount screws are)
    and the pivot end (x > -38.2) keep their thickness.
  - diagonal cut-outs through the floor, the top and both side walls, over the same x span so the stripes line up.
    They stay clear of the screw nubs (wall band above z 36), the countersinks (|y| > 28.25) and the steps.
Option A "shield":  one row of long parallel slots leaning 60 deg, rounded ends - a motorcycle exhaust heat shield.
Option B "chevron": two rows of shorter 45 deg slots leaning opposite ways (herringbone) with a rib between the rows.
Out: complete assembly/arm_2_part_shield.step and arm_2_part_chevron.step (assemblies "body" + "top").
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "complete assembly", "arm_2_part_nubs.step")
OUT = os.path.join(ROOT, "complete assembly", "arm_2_part_{}.step")
NYLON = 1.10e-3                                # g/mm^3

Z_BOT, FLOOR_T = -4.0, 4.0     # 2026-11-04: the base part is now cut to 4 mm plates top and bottom (arm2_screw_nubs),
POCKETS = False                # so the floor/top pockets below are off (kept for the record)
X_BOX0, X_BOX1 = -152.5, -38.2                 # where the box runs: walls start / pivot-end wall
X0, X1 = -141.0, -49.0                         # cut-out span (same on every face)
WALL_Z = (6.0, 36.0)                           # wall band: floor top is z 0, nubs start at 41.2
PLATE_Y = (-23.0, 23.0)                        # floor and top band: walls inside at 31.25, countersinks from 28.25
WALL_IN = 31.25                                # inside faces of the side walls
W_HALF = 35.25                                 # half the link's width
SEAT_R = 35.25 + 1.0                           # the drive's seat on the plate stays full thickness (housing 70.5)
X_FRONT = -200.0 + SEAT_R - 2.0                # floor pocket runs forward to the seat
FLOOR_Z = (Z_BOT - 1, Z_BOT + FLOOR_T + 1)     # the floor: z -4 .. 0
LID_Z = (60.0, 70.5)

OPTIONS = {
    "shield": {"label": "A heat shield", "rows": lambda b: [(b[0], b[1], 60.0)], "w": 6.0, "land": 5.0},
    "chevron": {"label": "B chevron", "w": 5.0, "land": 4.5,
                "rows": lambda b: [(b[0], (b[0] + b[1]) / 2 - 2.25, 45.0), ((b[0] + b[1]) / 2 + 2.25, b[1], -45.0)]},
}


def slots(v0, v1, ang, w, land):
    """Parallel slots leaning ang (deg from +x) filling the band v0..v1 inside X0..X1, centred on the span."""
    th, r = math.radians(abs(ang)), w / 2
    hc = ((v1 - v0) / 2 - r) / math.sin(th)                 # slot centre to cap centre
    ext = hc * math.cos(th) + r                             # half the slot's length along x
    pitch = (w + land) / math.sin(th)
    n = int(((X1 - X0) - 2 * ext) / pitch) + 1
    x_first = (X0 + X1) / 2 - (n - 1) * pitch / 2
    return [(x_first + i * pitch, (v0 + v1) / 2, 2 * hc + w, ang) for i in range(n)]


def cutters(spec, b, w0, w1):
    """Slot prisms drawn in a local u-v plane (u = x), extruded along local w from w0 to w1."""
    out = []
    for v0, v1, ang in spec["rows"](b):
        for x, v, length, a in slots(v0, v1, ang, spec["w"], spec["land"]):
            out.append(cq.Workplane("XY", origin=(0, 0, w0)).center(x, v).slot2D(length, spec["w"], a)
                       .extrude(w1 - w0).val())
    return out


def floor_raise():
    """The floor thinned to FLOOR_T from the INSIDE (Mike 2026-11-04: the bottom back flush with the rest, so it
    prints flat with no support): a pocket between the side walls from the floor top (z 0) down to Z_BOT + FLOOR_T,
    with 45 deg ends."""
    zt, zf = 0.0, Z_BOT + FLOOR_T
    d = zt - zf
    # between the walls: flat from the front of the box to the pivot end, 45 deg there
    pts = [(X_FRONT, zt + 1), (X_FRONT, zf), (X_BOX1 - d, zf), (X_BOX1, zt), (X_BOX1, zt + 1)]
    # draw in x-z: build in XY (u = x, v = z) and turn the prism +90 deg about x: (u, v, w) -> (u, -w, v)
    s = cq.Workplane("XY", origin=(0, 0, -WALL_IN)).polyline(pts).close().extrude(2 * WALL_IN).val()
    s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90)
    # Mike: the plate in front of the box (no walls there) down to 4 mm too, all but the drive's seat
    front = cq.Solid.makeBox(X_BOX0 - X_FRONT + 0.01, 2 * (W_HALF + 1), zt + 1 - zf, cq.Vector(X_FRONT, -W_HALF - 1, zf))
    s = s.fuse(front)
    seat = cq.Solid.makeCylinder(SEAT_R, 20, cq.Vector(-200.0, 0, zf - 5), cq.Vector(0, 0, 1))
    return s.cut(seat)


LID_UNDER, LID_T = 61.2, 4.0                   # underside of the top; the top thinned to LID_T like the floor
LID_POCKET_Y = 26.0                            # pocket half width: clear of the screw nubs (inner edge 27.25)
LID_POCKET_X = (X_BOX0 + 4.0, X_BOX1 - 4.0)    # inside the box ends


def lid_pocket():
    """Mike 2026-11-04: the top the same thickness as the floor. A pocket in its underside between the walls, from
    z 61.2 up to 69.2 - LID_T, 45 deg ends; the rim over the walls and nubs (screws) and the cap keep 8 mm."""
    zb = LID_UNDER
    zt = 69.2 - LID_T
    d = zt - zb
    x0, x1 = LID_POCKET_X
    pts = [(x0, zb - 1), (x0, zb), (x0 + d, zt), (x1 - d, zt), (x1, zb), (x1, zb - 1)]
    s = cq.Workplane("XY", origin=(0, 0, -LID_POCKET_Y)).polyline(pts).close().extrude(2 * LID_POCKET_Y).val()
    return s.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90)


def printed_g(s):
    v, a = s.Volume(), s.Area()
    shell = min(v, a * 1.2)                                # 3 perimeters of 0.4 mm
    return (shell + 0.2 * (v - shell)) * NYLON             # + 20 % infill


def main():
    solids = sorted([s for _, s, _ in read_step(SRC)], key=lambda s: -s.Volume())
    body0, top0 = solids[0], solids[1]
    print(f"now: body {body0.Volume() * NYLON:.0f} g solid / {printed_g(body0):.0f} g printed, "
          f"top {top0.Volume() * NYLON:.0f} / {printed_g(top0):.0f} g")
    raise_cut = floor_raise() if POCKETS else None
    for key, spec in OPTIONS.items():
        body = body0.cut(raise_cut) if raise_cut is not None else body0
        v_raise = body0.Volume() - body.Volume()
        walls = [c.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90) for c in cutters(spec, WALL_Z, -40, 40)]
        floor = cutters(spec, PLATE_Y, *FLOOR_Z)
        lid = cutters(spec, PLATE_Y, *LID_Z)
        for c in walls + floor:
            body = body.cut(c)
        top = top0.cut(lid_pocket()) if POCKETS else top0
        for c in lid:
            top = top.cut(c)
        body, top = body.clean(), top.clean()
        for nm, s in (("body", body), ("top", top)):
            assert len(s.Solids()) == 1, f"{key} {nm}: {len(s.Solids())} solids"
            assert BRepCheck_Analyzer(s.wrapped).IsValid(), f"{key} {nm} not valid"
        body, top = body.Solids()[0], top.Solids()[0]
        # the cut-outs must not touch the nubs, pilots, countersinks or the elbow plate
        for x in (-48, -80, -112, -144):
            for y in (-31.25, 31.25):
                assert body.isInside(cq.Vector(x, y + (2.5 if y < 0 else -2.5), 50)), "nub damaged"
        print(f"{spec['label']}: {len(walls) // 1} wall slots (both walls each), {len(floor)} floor, {len(lid)} top; "
              f"floor raise -{v_raise * NYLON:.0f} g")
        print(f"   body {body.Volume() * NYLON:.0f} g solid / {printed_g(body):.0f} g printed "
              f"({(body.Volume() - body0.Volume()) * NYLON:+.0f} / {printed_g(body) - printed_g(body0):+.0f}); "
              f"top {top.Volume() * NYLON:.0f} / {printed_g(top):.0f} g "
              f"({(top.Volume() - top0.Volume()) * NYLON:+.0f} / {printed_g(top) - printed_g(top0):+.0f})")
        assy = cq.Assembly(name=f"arm_2_part_{key}")
        assy.add(body, name="body", color=cq.Color(0.72, 0.74, 0.78))
        assy.add(top, name="top", color=cq.Color(0.36, 0.55, 0.94))
        write_step(assy, OUT.format(key))
        print("   " + OUT.format(key))


if __name__ == "__main__":
    main()

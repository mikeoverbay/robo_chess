#!/usr/bin/env python3
"""
arm2_screw_nubs.py - screw nubs for Mike's split arm part (2026-11-04).

    python arm2_screw_nubs.py [IN.step] [OUT.step]

In:  complete assembly/arm_2_part.step - his Shapr3D export: the body and the removable top (both named "root").
For 8 x #4 x 3/4" flat-head wood screws holding the top on, four per side (he is ordering 3/4"):
  body: a half-round nub (r 4) on the inside of each side wall under each screw, centred on the wall's inside face,
        solid 16 mm down from the wall top, then a 45 deg taper back into the wall so it prints without support;
        a 2.4 mm pilot 14 mm deep in each.
  top:  a 3.4 mm clearance hole through at each screw with an 82 deg countersink (6.0 mm at the top face) so the
        flat head sits flush.
Also (2026-11-04, Mike): the top 6808 (40 x 52 x 7) at each joint, inner race on the next link's 40 boss:
  body: the 52-dia, 0.5 mm step round the boss base (face 4 in the viewer, z 53.2-53.7) is gone; the 43-dia riser
        that seats the inner race now runs from the cup top (53.2) to 54.2, so the outer race clears by 1 mm.
  top:  the cap over the elbow gets a ring under it (70.5 OD, like the cap) with a 52.2 pocket for the bearing's
        outer race, z 53.7-61.2 (slip fit, so the bearing settles where the drive puts it); the cap's 40 hole is
        opened to 46 so the cap presses only on the outer race (inner race OD 42.5, outer race ID 49.5).
And (2026-11-04, Mike: "only as wide as it needs to be to hold the bearings"): the top cut down to z 65.2 and the
bottom up to z -4, so both outside plates are 4 mm; the drive still sits on z 0 and the 6808 under the cap (61.2) is
unchanged. Drive mount screws become M3 x 10; the #4 x 3/4" lid screws stay (pilot deepened to 17.5).
Out: complete assembly/arm_2_part_nubs.step, an assembly of "body" and "top". The original file is not touched.
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
import cycloid as C  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "complete assembly", "arm_2_part.step")
DST = os.path.join(ROOT, "complete assembly", "arm_2_part_nubs.step")

XS = (-48.0, -80.0, -112.0, -144.0)      # along the arm: 32 apart, ~9 in from each end of the walls (-38.9 .. -152.5)
Y_IN = 31.25                              # inside faces of the side walls (walls are 31.25 .. 35.25)
Z_WALL = 61.2                             # wall tops = underside of the top part
Z_LID = 65.2                              # Mike 2026-11-04: top cut down to 4 mm (was 69.2 / cap 69.7)
Z_BOTTOM = -4.0                           # Mike 2026-11-04: bottom plate cut up to 4 mm (was -10.8); drive seat stays z 0
R_NUB, NUB_SOLID, NUB_TAPER = 4.0, 16.0, 4.0
PILOT_D, PILOT_DEPTH = 2.4, 17.5          # 3/4" screws through the 4 mm top: tip at 46.15, pilot to 43.7 (in the cone)
CLEAR_D, CSK_D, CSK_ANGLE = 3.4, 6.0, 82.0
SCREW_L = 19.05                           # #4 x 3/4", flat head: length includes the head

PIVOT, ELBOW = (0.0, 0.0), (-200.0, 0.0)  # the joint this link turns on / the joint it carries
Z_CUP, Z_RISER_TOP = 53.2, 54.2           # cup top; top of the 43 riser = underside of the 6808
STEP_D, RISER_D = 52.0, 43.0              # the step to remove / the riser that stays
Z_CAP = 61.2                              # underside of the cap = top of the 6808
POCKET_D, RING_OD, Z_RING_BOT = 52.2, 70.5, 53.7
CAP_HOLE_D = 46.0


def cyl(r, z0, z1, x, y):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def cone(r0, r1, z0, z1, x, y):
    return cq.Solid.makeCone(r0, r1, z1 - z0, V(x, y, z0), V(0, 0, 1))


def one(shape):
    s = shape.Solids()
    assert len(s) == 1, f"{len(s)} solids"
    return s[0]


def heal(s, precision=0.12):
    """Remove faces narrower than `precision` (export slivers, 0.1 mm kinks) and merge coplanar faces."""
    from OCP.ShapeFix import ShapeFix_FixSmallFace
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    f = ShapeFix_FixSmallFace()
    f.Init(s.wrapped)
    f.SetPrecision(precision)
    f.Perform()
    u = ShapeUpgrade_UnifySameDomain(f.FixShape(), True, True, True)
    u.Build()
    r = one(cq.Shape.cast(u.Shape()))
    assert BRepCheck_Analyzer(r.wrapped).IsValid(), "healing broke the solid"
    assert abs(r.Volume() - s.Volume()) < 20.0, f"healing changed the volume by {r.Volume() - s.Volume():.1f} mm^3"
    return r


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else SRC
    dst = sys.argv[2] if len(sys.argv) > 2 else DST
    solids = sorted([s for _, s, _ in read_step(src)], key=lambda s: -s.Volume())
    body, top = solids[0], solids[1]
    # only as thick as the joint needs: everything below Z_BOTTOM and above Z_LID comes off
    body = one(body.cut(cq.Solid.makeBox(1000, 200, 50, V(-600, -100, Z_BOTTOM - 50))).clean())
    top = one(top.cut(cq.Solid.makeBox(1000, 200, 50, V(-600, -100, Z_LID))).clean())
    # Mike flagged faces that don't belong: the Shapr3D export carries a zero-width step at z 62.7 on the front-corner
    # bevels (sliver faces) and two 0.1 mm kinks where the top's end face turns into its angled corners (x -38.2,
    # |y| 32.25). OpenCascade's small-face healing removes them and merges what is left (the top loses 8 mm^3 at the
    # kinks). Box patching here left a slit, then steps (2026-11-04) - healing replaced it.
    body, top = heal(body), heal(top)
    v_body, v_top = body.Volume(), top.Volume()
    sites = [(x, s * Y_IN) for x in XS for s in (-1, 1)]

    # what is there now, at every screw: wall top, top part's faces, nothing inside the box where a nub goes
    for x, y in sites:
        s = 1 if y > 0 else -1
        assert body.isInside(V(x, y + s * 2.0, Z_WALL - 0.1)), f"no wall under ({x}, {y})"
        assert not body.isInside(V(x, y + s * 2.0, Z_WALL + 0.1)), "wall goes higher than expected"
        assert top.isInside(V(x, y, Z_WALL + 0.1)) and top.isInside(V(x, y, Z_LID - 0.1)), "top part not there"
        assert not top.isInside(V(x, y, Z_LID + 0.1)), "top part thicker than expected"
        for dz in (0.5, 8.0, NUB_SOLID + NUB_TAPER - 0.5):
            assert not body.isInside(V(x, y - s * (R_NUB - 0.3), Z_WALL - dz)), "something already inside the box"

    # body: nubs, then pilots
    z_solid = Z_WALL - NUB_SOLID
    nubs = []
    for x, y in sites:
        nubs += [cyl(R_NUB, z_solid, Z_WALL, x, y), cone(0.0, R_NUB, z_solid - NUB_TAPER, z_solid, x, y)]
    new_body = body
    for n in nubs:
        new_body = new_body.fuse(n)
    for x, y in sites:
        new_body = new_body.cut(cyl(PILOT_D / 2, Z_WALL - PILOT_DEPTH, Z_WALL + 1, x, y))
    # Mike: #4 x 3/4" flat heads up through the 4 mm plate into the drive housing too (the housing's 2.5 holes): an
    # 82 deg countersink on the underside of each of the 6 plate holes so the heads sit flush. From z -4 a 3/4" screw
    # ends at z 15.05; the drive's own lid screws (M3 x 25 from z 41.7) end at 16.7 in the same holes: 1.6 mm apart.
    for a in C.LID_SCREW_ANGLES:
        hx, hy = C.polar(C.LID_SCREW_R, a)
        hx += ELBOW[0]
        assert not body.isInside(V(hx, hy, Z_BOTTOM + 2.0)), "no plate hole where expected"
        h = (CSK_D - CLEAR_D) / 2 / math.tan(math.radians(CSK_ANGLE / 2))
        over = 0.5
        new_body = new_body.cut(cone(CSK_D / 2 + over * math.tan(math.radians(CSK_ANGLE / 2)), CLEAR_D / 2,
                                     Z_BOTTOM - over, Z_BOTTOM + h, hx, hy))
    # the 52 step round the boss base goes; the 43 riser stays (it now starts at the cup top)
    assert body.isInside(V(PIVOT[0] + 24.0, 0, Z_CUP + 0.25)), "no 52 step where expected"
    step = cyl(STEP_D / 2 + 0.01, Z_CUP, Z_RING_BOT + 0.01, *PIVOT).cut(cyl(RISER_D / 2, Z_CUP - 1, Z_RING_BOT + 1, *PIVOT))
    new_body = new_body.cut(step)
    new_body = one(new_body.clean())

    # top: clearance holes + countersinks (82 deg: 6.0 at the face down to 3.4)
    h_csk = (CSK_D - CLEAR_D) / 2 / math.tan(math.radians(CSK_ANGLE / 2))
    over = 0.5
    new_top = top
    for x, y in sites:
        new_top = new_top.cut(cyl(CLEAR_D / 2, Z_WALL - 1, Z_LID + 1, x, y))
        new_top = new_top.cut(cone(CLEAR_D / 2, CSK_D / 2 + over * math.tan(math.radians(CSK_ANGLE / 2)),
                                   Z_LID - h_csk, Z_LID + over, x, y))
    # the bearing pocket under the cap, and the cap hole opened so only the outer race is pressed
    assert top.isInside(V(ELBOW[0] + 30.0, 0, Z_CAP + 0.2)) and not top.isInside(V(ELBOW[0] + 30.0, 0, Z_CAP - 0.2)), \
        "cap underside not at 61.2"
    ring_ = cyl(RING_OD / 2, Z_RING_BOT, Z_CAP + 0.01, *ELBOW).cut(cyl(POCKET_D / 2, Z_RING_BOT - 1, Z_CAP, *ELBOW))
    new_top = new_top.fuse(ring_)
    new_top = new_top.cut(cyl(CAP_HOLE_D / 2, Z_CAP - 1, Z_LID + 2, *ELBOW))
    new_top = one(new_top.clean())

    for nm, s in (("body", new_body), ("top", new_top)):
        assert BRepCheck_Analyzer(s.wrapped).IsValid(), f"{nm} is not a valid solid"
    nub_one = cyl(R_NUB, z_solid, Z_WALL, 0, 0).Volume() / 2 + cone(0, R_NUB, 0, NUB_TAPER, 0, 0).Volume() / 2
    pilot = math.pi * (PILOT_D / 2) ** 2 * PILOT_DEPTH
    print(f"body: {v_body:.0f} -> {new_body.Volume():.0f} mm^3 ({new_body.Volume() - v_body:+.0f}; "
          f"expected {8 * (nub_one - pilot):+.0f}: 8 half nubs {nub_one:.0f} each less the pilots)")
    print(f"top:  {v_top:.0f} -> {new_top.Volume():.0f} mm^3 ({new_top.Volume() - v_top:+.0f})")
    print(f"countersink depth {h_csk:.2f} mm; screw tip at z {Z_LID - SCREW_L:.2f}: {Z_WALL - (Z_LID - SCREW_L):.1f} mm of "
          f"thread in the nub, pilot bottom at z {Z_WALL - PILOT_DEPTH:.1f} ({(Z_LID - SCREW_L) - (Z_WALL - PILOT_DEPTH):.1f} below "
          f"the tip), nub solid down to z {z_solid:.1f}")
    print(f"plastic round the pilot: {R_NUB - PILOT_D / 2:.1f} mm in the nub, {35.25 - Y_IN - PILOT_D / 2:.1f} mm to the "
          f"outside of the wall; top part: {35.25 - Y_IN - CLEAR_D / 2:.2f} mm from hole to edge, "
          f"{35.25 - Y_IN - CSK_D / 2:.2f} mm from countersink to edge")

    assy = cq.Assembly(name="arm_2_part_nubs")
    assy.add(new_body, name="body", color=cq.Color(0.72, 0.74, 0.78))
    assy.add(new_top, name="top", color=cq.Color(0.36, 0.55, 0.94))
    write_step(assy, dst)
    print(dst)


if __name__ == "__main__":
    main()

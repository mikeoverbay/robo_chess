#!/usr/bin/env python3
"""
torque_test.py - bench rig to MEASURE the shoulder drive's torque (Mike 2026-11-13: "print an arm with a known length.
And clamp ... the gearbox down, and then test the scale on the arm. See how much pressure it pushes before it skips.")

    python torque_test.py            -> print/Torque test arm.step, print/Torque test stand.step (assembled in place
                                        on the gearbox, drive frame: axis z, housing bottom z 0, output face z 59.2)
    python torque_test.py --rig-view -> viewer_session/torque_rig_adapter_upright.step (main studio: the rig as he
                                        builds it - his printed stand + the adapter ring + today's gearbox)

Two printed parts, both bolting on exactly as the real arm does (cycloid.py's shared bolt circles):
  - test arm: the arm link's cup (70.5 x 8, 3 x M3 on r 15 with 8 x 4 counterbores, the same 3 output screws) and a
    flat bar 40 deep x 8 thick with D3.2 pin holes at known distances from the axis (PIN_R). A 3 mm dowel or an M3
    screw through the chosen hole presses on the scale: on a flat scale the contact is right under the pin's centre,
    so the lever is the hole's distance (x cos(tilt): keep the bar level). Prints flat, cup face down.
  - stand: the elbow mounting ring's plate (8 thick, 6 x M3 countersunk from behind on r 32.4 - on the arms' circle,
    mockup_link.ARM_SCREW_*, 30 deg off the lid screws since 2026-11-15 - the 44 x 44 opening
    the motor goes through) standing on a foot with two gussets; clamp tabs left and right. Prints on its foot.
The motor pushes the bar down onto a kitchen scale until it skips: torque = reading x g x lever (+ the bar's own
weight moment, printed below - small). Run it with the 48 mm 17HS19 he has: that measures the gearbox's efficiency,
and the 60 mm shoulder motor (not bought yet) is then a calculation.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import cycloid as C  # noqa: E402
import mockup_link as ML  # noqa: E402
import motor_speed as MS  # noqa: E402
from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
PRINT = os.path.join(ROOT, "print")
GEARBOX = os.path.join(PRINT, "Gearbox assembly (print 4).step")

Z_OUT = 59.2                     # output face (mockup_link.Z_OUT: 45.2 + the spread 14)
CUP_D, CUP_T = 70.5, 8.0         # the arm link's cup
BAR_W = 40.0                     # in the turning plane (bending depth): 2.4 mm sag at 90 N on 220 in PLA
PIN_D = 3.2                      # a 3 mm dowel (the gearbox's 3 x 20) or an M3 screw
PIN_R = (100.0, 150.0, 200.0, 220.0)   # lever lengths; 220 is the longest that fits the Ender 3 Pro's 220 bed
END_R = BAR_W / 2                # the bar ends in a half round about the last hole
BED = 220.0

PLATE_T = 8.0                    # the mounting ring's plate (mockup_link.PLATE_T)
PLATE_HALF = 45.0                # plate 90 x 90 about the axis
MOTOR_CUT = 22.0                 # 44 x 44 opening (mockup_link: the next drive's motor goes through)
CSK_D, CLEAR_D = 6.3, 3.4        # M3 countersunk from behind, as the mounting ring
FOOT_T = 8.0
FOOT_HALF = 75.0                 # foot 150 wide: clamp tabs 30 wide each side of the plate
FOOT_DEPTH = 70.0                # behind the plate, under the motor (17HS19 48 long, 60 mm motor 60)
GUSSET_T = 8.0
GUSSET_X = PLATE_HALF - GUSSET_T  # gussets at |x| 37..45: clear of the countersinks (r 32.4 + 3.15 = 35.6)
GUSSET_UP = 75.0                 # up the plate's back from the foot


def cyl(d, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def one(shape, what):
    s = cq.Shape.cast(shape.wrapped) if not isinstance(shape, cq.Shape) else shape
    solids = s.Solids()
    assert len(solids) == 1, f"{what}: {len(solids)} solids"
    assert solids[0].isValid(), f"{what}: invalid"
    return solids[0]


def test_arm():
    z0, z1 = Z_OUT, Z_OUT + CUP_T
    body = cyl(CUP_D, z0, z1).fuse(box(0, PIN_R[-1], -BAR_W / 2, BAR_W / 2, z0, z1)).fuse(cyl(BAR_W, z0, z1, PIN_R[-1]))
    cuts = []
    for a in C.OUT_SCREW_ANGLES:                                  # the arm's 3 output screws
        x, y = C.polar(C.OUT_SCREW_R, a)
        cuts += [cyl(C.OUT_DISC_HOLE, z0 - 1, z1 + 1, x, y), cyl(8.0, z1 - 4.0, z1 + 1, x, y)]
    for r in PIN_R:
        cuts.append(cyl(PIN_D, z0 - 1, z1 + 1, r))
    for c in cuts:
        body = body.cut(c)
    try:                                                          # each hole's lever length, cut 0.6 into the top
        for r in PIN_R:
            t = (cq.Workplane("XY").workplane(offset=z1 - 0.6).center(r - 13.0, -11.0)
                 .text(f"{r:g}", 7.0, 0.6 + 0.01, halign="center", valign="center", combine=False))
            body = body.cut(t.val())
    except Exception as e:                                        # no font: the holes still are where they are
        print(f"  (labels left off: {e})")
    return one(body.clean(), "test arm")


def stand(angles=None, r=None):
    # plate under the housing (z -8..0); the foot is the bench, at y -(PLATE_HALF + FOOT_T); the motor hangs to -z
    # angles / r: the 6 holes (default the arms' circle, ML.ARM_SCREW_*; the one Mike printed has C.LID_SCREW_*)
    angles = ML.ARM_SCREW_ANGLES if angles is None else angles
    r = ML.ARM_SCREW_R if r is None else r
    yb = -PLATE_HALF
    body = box(-PLATE_HALF, PLATE_HALF, yb, PLATE_HALF, -PLATE_T, 0)
    body = body.fuse(box(-FOOT_HALF, FOOT_HALF, yb - FOOT_T, yb, -PLATE_T - FOOT_DEPTH, 0))
    for sx in (-1, 1):                                            # gussets: plate back face to the foot's top
        x0 = GUSSET_X if sx > 0 else -PLATE_HALF
        pts = [(yb, -PLATE_T), (yb + GUSSET_UP, -PLATE_T), (yb, -PLATE_T - FOOT_DEPTH)]
        g = (cq.Workplane("YZ").workplane(offset=x0).polyline(pts).close().extrude(GUSSET_T)).val()
        body = body.fuse(g)
    cuts = [box(-MOTOR_CUT, MOTOR_CUT, -MOTOR_CUT, MOTOR_CUT, -PLATE_T - 1, 1)]
    k = (CSK_D - CLEAR_D) / 2 / math.tan(math.radians(45))
    for a in angles:                                              # 6 x M3 from behind into the housing, as the arms
        x, y = C.polar(r, a)                                      # (2026-11-15: 30 deg off the lid screws)
        cuts.append(cyl(CLEAR_D, -PLATE_T - 1, 1, x, y))
        cuts.append(cq.Solid.makeCone(CSK_D / 2, CLEAR_D / 2, k, V(x, y, -PLATE_T - 0.01), V(0, 0, 1)))
    for c in cuts:
        body = body.cut(c)
    return one(body.clean(), "test stand")


RIG_VIEW = os.path.join(HERE, "viewer_session", "torque_rig_adapter_upright.step")
ADAPTER = os.path.join(PRINT, "Gearbox stand adapter ring.step")
ADAPTER_T = 10.0                 # the gearbox session's ring (gearbox_stand_adapter.py): z -10..0 under the housing


def rig_view():
    """--rig-view: the rig as Mike builds it (2026-11-15): the stand he printed (2026-11-13: holes on the LID angles),
    the gearbox session's adapter ring under the housing (the stand drops ADAPTER_T), the print-4 gearbox as it is now,
    the test arm and a 48 mm motor; checked for overlaps, then turned 180 about x so the stand's foot stands on the
    robot scene's floor (main's scene.json places it at [600, 204, 0]). Re-run after any gearbox change."""
    import nema17
    old = stand(C.LID_SCREW_ANGLES, C.LID_SCREW_R).translate(V(0, 0, -ADAPTER_T))
    bodies = [(n, s, (0.95, 0.55, 0.10)) for n, s, _ in read_step(ADAPTER)]
    bodies.append(("Torque test stand (the one printed 2026-11-13)", old, (0.35, 0.55, 0.85)))
    bodies += [(n, s, (0.72, 0.74, 0.78)) for n, s, _ in read_step(GEARBOX)]
    bodies.append(("Torque test arm", test_arm(), (0.90, 0.55, 0.15)))
    body, caps = nema17.build(48.0)
    bodies += [("NEMA 17 48 mm", body, (0.12, 0.12, 0.13)), ("NEMA 17 end caps + shaft", caps, (0.78, 0.79, 0.81))]
    bad = 0
    for i, (n, s, _) in enumerate(bodies):
        for m, t, _ in bodies[i + 1:]:
            if "NEMA" in n + m and ("Gearbox" in n + m or ("NEMA" in n and "NEMA" in m)):
                continue                                          # the motor's shaft runs in the gearbox
            v = s.intersect(t).Volume()
            if v > 0.01:
                print(f"  OVERLAP {n} x {m}: {v:.2f} mm^3")
                bad += 1
    a = cq.Assembly(name="torque_rig_adapter")
    for n, s, c in bodies:
        a.add(s.rotate(V(0, 0, 0), V(1, 0, 0), 180), name=n, color=cq.Color(*c))
    write_step(a, RIG_VIEW)
    print(f"{len(bodies)} bodies, {'no overlaps' if not bad else f'{bad} OVERLAPS'}\nwrote {RIG_VIEW}")
    return 1 if bad else 0


def overlaps(part, what):
    worst = 0.0
    for n, s, _ in read_step(GEARBOX):
        v = part.intersect(s).Volume()
        if v > 1e-3:
            print(f"  FAIL {what} overlaps {n}: {v:.2f} mm^3")
        worst = max(worst, v)
    return worst


def main():
    arm, std = test_arm(), stand()
    fails = []

    # --- fit on the gearbox
    if overlaps(arm, "test arm") > 1e-3:
        fails.append("arm overlap")
    if overlaps(std, "stand") > 1e-3:
        fails.append("stand overlap")
    print(f"  ok   cup face on the output face z {arm.BoundingBox().zmin:.1f}; stand face on the housing bottom "
          f"z {std.BoundingBox().zmax:.1f}")

    # --- fits the bed (arm along the diagonal, stand on its foot)
    span = PIN_R[-1] + END_R + CUP_D / 2
    # the outline is the hull of the cup circle and the end round (the bar is no wider than either), so laid along the
    # diagonal its square footprint is: centres' distance / sqrt 2 + both radii
    sq = PIN_R[-1] / math.sqrt(2) + CUP_D / 2 + END_R
    ok = sq <= BED - 4
    print(f"  {'ok  ' if ok else 'FAIL'} test arm {span:.1f} long: laid on the bed's diagonal it takes {sq:.1f} x "
          f"{sq:.1f} of the {BED:g} x {BED:g} bed")
    if not ok:
        fails.append("arm bed")
    sb = std.BoundingBox()
    ok = sb.xlen <= BED and sb.zlen <= BED
    print(f"  {'ok  ' if ok else 'FAIL'} stand footprint {sb.xlen:.0f} x {sb.zlen:.0f}, {sb.ylen:.0f} tall")

    # --- the bar's own weight, its bending, what to expect on the scale
    m_arm = arm.Volume() * MS_RHO / 1000.0                        # g, solid
    com = arm.Center()
    print(f"\ntest arm {m_arm:.0f} g solid, centre of mass {com.x:.1f} mm out: adds {m_arm * com.x / 1000:.1f} g.m "
          f"(= {m_arm * 9.81e-6 * com.x:.3f} N.m) to the motor's torque - add it, or ignore it (under 1 %)")
    e_pla = 3000.0
    i_bar = CUP_T * BAR_W ** 3 / 12
    print(f"bar 40 x 8: sags {90 * PIN_R[-1] ** 3 / (3 * e_pla * i_bar):.1f} mm at 90 N on {PIN_R[-1]:g} "
          f"(doesn't change the reading)")
    axis_h = PLATE_HALF + FOOT_T
    print(f"axis {axis_h:g} mm above the bench: the scale's top {axis_h - 1.5:.1f} mm up for the bar to be level on a "
          f"3 mm pin (shim the scale)")
    print("\nexpected scale reading (kg) when the motor skips, slow push, at rated current:")
    print("  motor                         eff    " + "  ".join(f"{r:5.0f} mm" for r in PIN_R))
    for motor in ("48 mm (17HS19-2004S1)", "60 mm (73 N.cm, 2 A)"):
        hold = MS.MOTORS[motor][0]
        for eff in (0.54, 0.65, 0.83):
            t = hold * C_RATIO * eff
            print(f"  {motor:28s}  {eff:.2f}   " + "  ".join(f"{t / 9.81 / (r / 1000):8.1f}" for r in PIN_R))
    print(f"  (the arm's worst load is 11.9 N.m solid / 11.0 as printed = "
          + ", ".join(f"{11.9 / 9.81 / (r / 1000):.1f} kg at {r:g}" for r in PIN_R) + ")")

    os.makedirs(PRINT, exist_ok=True)
    for name, part, col in (("Torque test arm", arm, (0.9, 0.55, 0.15)), ("Torque test stand", std, (0.35, 0.55, 0.85))):
        a = cq.Assembly(name="rig")
        a.add(part, name=name, color=cq.Color(*col))
        path = os.path.join(PRINT, name + ".step")
        write_step(a, path)
        print(f"wrote {path}")
    print("\nALL OK" if not fails else f"\n{len(fails)} FAIL: {', '.join(fails)}")
    return 1 if fails else 0


MS_RHO = 1.24                    # PLA g/cm^3
C_RATIO = 37.0                   # the cycloid (cycloid.py N_PINS - 1 = 37)

if __name__ == "__main__":
    sys.exit(rig_view() if "--rig-view" in sys.argv else main())

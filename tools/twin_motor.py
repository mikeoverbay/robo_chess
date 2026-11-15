#!/usr/bin/env python3
"""
twin_motor.py - TEST (Mike 2026-11-13): two 48 mm NEMA 17s on the shoulder, belted 1:1, into the one J1 gearbox.

    "what if we use a NEMA 17 48mm body with a dual shaft and run a belt between that backside shaft up to a second
     motor" ... "I have two sets of belts ... two of the 5 mm bore pulleys. We can put the one with the double ended
     shaft straight into our gearbox. Run the belt up, along the side of the arm, and mount the motor on the back side
     of that plate. Save our other work because this is just a test."

    python twin_motor.py      -> print/Twin motor bracket (test).step
                                 complete assembly/arm_base_tube_twin_test.step (the shoulder base with the test drive)
                                 (studio "twin" shows it; main's arm_base_tube.step and scene are NOT touched)

Base frame (as arm_base_tube.step): J1 on z through (0, 0), the gearbox's motor face z 0 (housing toward +z), y down.
  - motor A: 17HS19-2004D, 48 mm, dual shaft (front 24 into the gearbox as now, rear D5 x 13 - the length StepperOnline
    lists for its 42 x 48 / 40 dual-shaft models; CHECK the 2004D's own) in place of the 60 mm motor
  - motor B: a 48 mm 17HS19, turned round (shaft back, -z), straight below A at the belt's centre distance, behind the
    column - the sector link A never swings into (J1 -14..180)
  - his kit: GT2 20T 5 mm bore pulleys (one on A's rear shaft, hub toward the motor; one on B, teeth toward the motor)
    and one 200 mm x 6 mm belt: 100 teeth = 2 C + 20 teeth x 2 mm -> C = 80.0
  - bracket (printed): B's face plate (6 thick, 3 over the belt; B's 4 screws and its boss in slots, +-2.5 to tension
    the belt), two cheeks beside the motors, a brace below A, and a crossbar under A that goes UNDER the column's two
    lower drive screws (#4 wood screws: 1" there, 3/4" elsewhere) - no new holes in the column, the gearbox or anything else
  - 2026-11-15: the column's 6 drive screws turned 30 deg off the gearbox's lid screws (mockup_link.ARM_SCREW_*), so
    the lower two are now at y ~28 either side of A's bottom: the two side pads became one crossbar under A. The screw
    spots are measured off arm_base_tube.step's column, so the bracket follows the column.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import nema17  # noqa: E402
from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
BASE = os.path.join(ROOT, "complete assembly", "arm_base_tube.step")
SERVO = os.path.join(ROOT, "complete assembly", "servo_on_base.step")
OUT_BASE = os.path.join(ROOT, "complete assembly", "arm_base_tube_twin_test.step")
OUT_PRINT = os.path.join(ROOT, "print", "Twin motor bracket (test).step")

BODY_L = 48.0                    # 17HS19
REAR_SHAFT = 13.0                # A's back shaft, D5 (ESTIMATE until his motor's listing / calipers)
BOSS_H = 2.0                     # NEMA 17 pilot boss D22 x 2
SQ = 42.3
SCREW_SQ = 31.0 / 2              # face screws on a 31 mm square

# GT2 20T, 5 mm bore, for a 6 mm belt (typical aluminium): flanges D16 x 1, teeth OD 12.22 (pitch D 12.73) x 7, hub
# D13 x 7 with 2 grub screws -> 16 long
P_FL, P_FL_T, P_TOOTH_OD, P_TEETH_L, P_HUB_D, P_HUB_L = 16.0, 1.0, 12.22, 7.0, 13.0, 7.0
PITCH_R = 20 * 2.0 / math.pi / 2       # 6.366
BELT_L, BELT_W, BELT_T = 200.0, 6.0, 1.38
C_DIST = (BELT_L - 2 * math.pi * PITCH_R) / 2     # 80.0

B_FACE = -58.5                   # B's front face (toward -z): body -58.5..-10.5, >= 3.8 off the column's back
PULLEY_GAP = 0.5                 # boss / back cap to the pulley
BELT_Z = B_FACE - BOSS_H - PULLEY_GAP - P_FL_T - P_TEETH_L / 2      # -65.5: the belt's middle plane

PLATE_T, PLATE_THIN = 6.0, 3.0   # B's face plate; 3 where the belt runs under it (1 mm to the belt's edge)
CHEEK_X0, CHEEK_T = 22.5, 4.0    # cheeks at |x| 22.5..26.5 (the motors are 21.15 either side)
PLATE_Y = (C_DIST - 27.0, C_DIST + 27.0)
SLOT = 2.5                       # belt tension: B slides +-2.5 along y
PAD_T = 5.0                      # the crossbar under the drive screws, on the column plate's back face (z -8)
RING_BACK = -8.0
BAR_Y = (22.5, 35.0)             # the crossbar: 1.35 below A's body (21.15), the screws' countersinks inside it
CSK_D, CLEAR_D = 6.3, 3.4
BRACE_Y = (42.0, 48.0)           # below A, above B


def cyl(d, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1 and s[0].isValid(), f"{what}: {len(s)} solids"
    return s[0]


def pulley(z_teeth_mid, hub_up):
    """GT2 20T on the z axis, teeth centred at z_teeth_mid; hub_up: hub on the +z side of the teeth"""
    t0, t1 = z_teeth_mid - P_TEETH_L / 2, z_teeth_mid + P_TEETH_L / 2
    p = cyl(P_TOOTH_OD, t0, t1).fuse(cyl(P_FL, t0 - P_FL_T, t0)).fuse(cyl(P_FL, t1, t1 + P_FL_T))
    p = p.fuse(cyl(P_HUB_D, t1 + P_FL_T, t1 + P_FL_T + P_HUB_L) if hub_up else cyl(P_HUB_D, t0 - P_FL_T - P_HUB_L, t0 - P_FL_T))
    for k in range(20):                                   # the 20 grooves, for the look
        a = 2 * math.pi * k / 20
        p = p.cut(cyl(1.3, t0, t1, (P_TOOTH_OD / 2 + 0.2) * math.cos(a), (P_TOOTH_OD / 2 + 0.2) * math.sin(a)))
    return one(p.cut(cyl(5.0, -200, 200)).clean(), "pulley")


def belt():
    """the loop round both pulleys: pitch line PITCH_R, teeth in, back out"""
    ri, ro = PITCH_R - 0.75, PITCH_R - 0.75 + BELT_T
    z0, z1 = BELT_Z - BELT_W / 2, BELT_Z + BELT_W / 2
    outer = cyl(2 * ro, z0, z1).fuse(cyl(2 * ro, z0, z1, 0, C_DIST)).fuse(box(-ro, ro, 0, C_DIST, z0, z1))
    inner = cyl(2 * ri, z0 - 1, z1 + 1).fuse(cyl(2 * ri, z0 - 1, z1 + 1, 0, C_DIST)).fuse(box(-ri, ri, 0, C_DIST, z0 - 1, z1 + 1))
    return one(outer.cut(inner).clean(), "belt")


def motors():
    body, caps = nema17.build(BODY_L)                     # front face z 0, body -48..0, shaft +z
    rear = cyl(5.0, -BODY_L - REAR_SHAFT, -BODY_L + 1.0)  # A's back shaft (1 mm into its end cap)
    flip = cq.Location(V(0, C_DIST, B_FACE), V(1, 0, 0), 180)   # B: turned round about x, face at B_FACE
    return body, caps, rear, body.moved(flip), caps.moved(flip)


def column_screws():
    """the column plate's 6 drive screw holes (D3.4 on z), base frame, read off arm_base_tube.step"""
    body = next(s for n, s, _ in read_step(BASE) if n == "base body")
    found = {}
    for f in body.Faces():
        if f.geomType() != "CYLINDER":
            continue
        c = f._geomAdaptor().Cylinder()
        p, d = c.Axis().Location(), c.Axis().Direction()
        if abs(c.Radius() - CLEAR_D / 2) < 0.01 and abs(abs(d.Z()) - 1) < 1e-6 and math.hypot(p.X(), p.Y()) < 40:
            found[(round(p.X(), 2), round(p.Y(), 2))] = (p.X(), p.Y())
    assert len(found) == 6, f"column drive screws: found {len(found)}"
    return sorted(found.values(), key=lambda q: q[1])


def bracket(screws):
    y0, y1 = PLATE_Y
    zt = B_FACE                                           # the plate's face against B
    zb = zt - PLATE_T
    x1 = CHEEK_X0 + CHEEK_T
    for x, y in screws:
        assert BAR_Y[0] + CSK_D / 2 + 0.5 <= y <= BAR_Y[1] - CSK_D / 2 - 0.5 and abs(x) < x1 - CSK_D / 2, \
            f"drive screw ({x:.1f}, {y:.1f}) not on the crossbar"
    plate = box(-x1, x1, y0, y1, zb, zt)
    cheeks = [box(s * CHEEK_X0 if s > 0 else -x1, x1 if s > 0 else -CHEEK_X0, BAR_Y[0], y1, zb, RING_BACK) for s in (1, -1)]
    bar = box(-x1, x1, BAR_Y[0], BAR_Y[1], RING_BACK - PAD_T, RING_BACK)
    brace = box(-CHEEK_X0, CHEEK_X0, BRACE_Y[0], BRACE_Y[1], zb + 4.0, -12.0)
    b = plate
    for p in cheeks + [bar, brace]:
        b = b.fuse(p)
    cuts = []
    # B's boss and its screws in slots along y (belt tension)
    cuts.append(cyl(23.0, zb - 1, zt + 1, 0, C_DIST - SLOT).fuse(cyl(23.0, zb - 1, zt + 1, 0, C_DIST + SLOT))
                .fuse(box(-11.5, 11.5, C_DIST - SLOT, C_DIST + SLOT, zb - 1, zt + 1)))
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * SCREW_SQ, C_DIST + sy * SCREW_SQ
            cuts.append(cyl(CLEAR_D, zb - 1, zt + 1, x, y - SLOT).fuse(cyl(CLEAR_D, zb - 1, zt + 1, x, y + SLOT))
                        .fuse(box(x - CLEAR_D / 2, x + CLEAR_D / 2, y - SLOT, y + SLOT, zb - 1, zt + 1)))
    # the belt runs up under the plate: 3 left over it
    cuts.append(box(-11.0, 11.0, y0 - 1, C_DIST, zb - 1, zt - PLATE_THIN))
    # the two drive screws through the crossbar: countersunk from behind, as in the column
    k = (CSK_D - CLEAR_D) / 2
    for x, y in screws:
        cuts.append(cyl(CLEAR_D, RING_BACK - PAD_T - 1, RING_BACK + 1, x, y))
        cuts.append(cq.Solid.makeCone(CSK_D / 2, CLEAR_D / 2, k, V(x, y, RING_BACK - PAD_T - 0.01), V(0, 0, 1)))
    for c in cuts:
        b = b.cut(c)
    return one(b.clean(), "bracket")


def main():
    base = read_step(BASE)
    keep = [(n, s, c) for n, s, c in base if not n.startswith("drive J1 NEMA 17 60 mm")]
    others = {n: s for n, s, _ in keep}
    others.update({n: s for n, s, _ in read_step(SERVO)})
    a_body, a_caps, a_rear, b_body, b_caps = motors()
    pa = pulley(BELT_Z, hub_up=True)                                  # on A's rear shaft, hub toward A
    pb = pulley(BELT_Z, hub_up=False).moved(cq.Location(V(0, C_DIST, 0)))   # on B, teeth toward B (hub further back)
    screws = column_screws()
    lower = screws[-2:]                                               # the two nearest the foot (y down)
    bt, br = belt(), bracket(lower)
    new = {"twin A: NEMA 17 48 mm dual shaft": a_body, "twin A: end caps + shafts": a_caps, "twin A: rear shaft": a_rear,
           "twin B: NEMA 17 48 mm": b_body, "twin B: end caps + shaft": b_caps,
           "twin pulley A (GT2 20T)": pa, "twin pulley B (GT2 20T)": pb, "twin belt (GT2 200 x 6)": bt,
           "Twin motor bracket (test)": br}
    fails = []
    print(f"belt {BELT_L:g} = {BELT_L / 2:g} teeth -> centre distance {C_DIST:.2f} mm; belt plane z {BELT_Z:.1f}")
    # A's rear shaft in its pulley's hub
    hub0 = BELT_Z + P_TEETH_L / 2 + P_FL_T
    hub1 = hub0 + P_HUB_L
    shaft_end = -BODY_L - REAR_SHAFT
    print(f"A's rear shaft ends z {shaft_end:.1f}; its pulley hub z {hub0:.1f}..{hub1:.1f} (A's back at {-BODY_L:g}, "
          f"{-BODY_L - hub1:.1f} gap): {min(hub1, -BODY_L) - max(hub0, shaft_end):.1f} of {P_HUB_L:g} mm of hub on the shaft")
    if hub1 > -BODY_L or min(hub1, -BODY_L) - max(hub0, shaft_end) < 5.0:
        fails.append("A pulley on its shaft")
    # clearances: every new part against the base, servo, drive and each other (touching pairs are expected:
    # motor/pulley on its shaft, bracket on B's face and on the column plate)
    expect = {("twin A: rear shaft", "twin pulley A (GT2 20T)"), ("twin A: rear shaft", "twin A: end caps + shafts"),
              ("twin A: rear shaft", "twin A: NEMA 17 48 mm dual shaft"), ("twin A: end caps + shafts", "twin pulley A (GT2 20T)"), ("twin B: end caps + shaft", "twin pulley B (GT2 20T)"),
              ("Twin motor bracket (test)", "twin B: NEMA 17 48 mm"), ("Twin motor bracket (test)", "twin B: end caps + shaft"),
              ("Twin motor bracket (test)", "base body"), ("twin A: NEMA 17 48 mm dual shaft", "twin A: end caps + shafts"),
              ("twin B: NEMA 17 48 mm", "twin B: end caps + shaft"), ("twin A: NEMA 17 48 mm dual shaft", "drive J1 mockup drive (spread 6808s)"),
              ("twin A: end caps + shafts", "drive J1 mockup drive (spread 6808s)"),
              ("twin A: NEMA 17 48 mm dual shaft", "drive J1 housing floor"),       # 2026-11-15: the two-piece
              ("twin A: end caps + shafts", "drive J1 housing floor"),              # housing's floor (stand-in)
              ("twin belt (GT2 200 x 6)", "twin pulley A (GT2 20T)"), ("twin belt (GT2 200 x 6)", "twin pulley B (GT2 20T)")}
    allp = dict(others, **new)
    names = list(new)
    print("clearances (mm, new parts):")
    for i, n in enumerate(names):
        for m in list(others) + names[i + 1:]:
            if m == n:
                continue
            s, t = allp[n], allp[m]
            if (n, m) in expect or (m, n) in expect:
                v = s.intersect(t).Volume()
                if v > 0.5 and "bracket" in n + m:
                    print(f"  FAIL {n} INTO {m}: {v:.1f} mm^3")
                    fails.append(f"{n}/{m}")
                continue
            d = s.distance(t)
            if d < 6.0:
                ov = s.intersect(t).Volume() if d < 1e-6 else 0.0
                bad = ov > 1e-3
                print(f"  {'FAIL' if bad else 'ok  '} {n} - {m}: {d:.1f}" + (f" OVERLAP {ov:.1f} mm^3" if bad else ""))
                if bad:
                    fails.append(f"{n}/{m}")
    # all 6 drive screws still get a driver (7 mm) straight in from behind: through the twin parts and the base
    print("drive screws (column, from behind):")
    for x, y in screws:
        z0 = RING_BACK - (PAD_T if (x, y) in lower else 0.0) - 0.05
        way = cyl(7.0, z0 - 120.0, z0, x, y)
        hit = [(n, way.intersect(s).Volume()) for n, s in allp.items()]
        hit = [(n, v) for n, v in hit if v > 0.01]
        on = "through the crossbar: #4 x 1\" (tip z 12.4, pilot to 16)" if (x, y) in lower else "#4 x 3/4\""
        print(f"  {'ok  ' if not hit else 'FAIL'} ({x:5.1f}, {y:5.1f}) {on}" +
              ("" if not hit else ": driver hits " + ", ".join(f"{n} {v:.1f}" for n, v in hit)))
        if hit:
            fails.append(f"driver ({x:.0f}, {y:.0f})")
    m = br.Volume() * 1.24 / 1000
    bb = br.BoundingBox()
    print(f"bracket {m:.0f} g solid PLA, {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f}")

    a = cq.Assembly(name="rig")
    a.add(br, name="Twin motor bracket (test)", color=cq.Color(0.35, 0.55, 0.85))
    write_step(a, OUT_PRINT)
    cols = {"twin belt (GT2 200 x 6)": (0.08, 0.08, 0.08), "Twin motor bracket (test)": (0.35, 0.55, 0.85)}
    a = cq.Assembly(name="base twin")
    for n, s, c in keep:
        a.add(s, name=n, color=c if isinstance(c, cq.Color) else None)
    for n, s in new.items():
        c = cols.get(n) or ((0.12, 0.12, 0.13) if "NEMA" in n else (0.78, 0.79, 0.81))
        a.add(s, name=n, color=cq.Color(*c))
    write_step(a, OUT_BASE)
    print(f"wrote {OUT_PRINT}\nwrote {OUT_BASE}")
    print("\nALL OK" if not fails else f"\n{len(fails)} FAIL: {', '.join(fails)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

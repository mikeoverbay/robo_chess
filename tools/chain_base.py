#!/usr/bin/env python3
"""
chain_base.py - the shoulder for the chain-drive arm. Mike, 2026-11-20 (relayed verbatim by Robo Chess gearbox):
"Let's just go with the chain drives on everything." / "Figure out how to connect it to the base." / "Send to arm session"

    python chain_base.py

The arm is the gearbox session's chain_drive_arm.py: link A turns on a 6808 in each wall (|y| 33..40) round a fixed
D40 boss; a #25 25T sprocket (his CNC-cut plate, D20 bore, 4 x M3 on D32) bolts to each boss's inner end (|y|
25.6..28.4) and the chains walk the link round it. Nothing hangs behind the shoulder now.

What this builds (Robot arm control and kinematics, 2026-11-20):
  - chain base body, ONE print standing on its foot:
      - arm_tube.py's column (foot, foot screws, the wire hole, servo tab + plate + gussets, J0 Hall pocket), cut off
        COL_TOP below J1. Link A's hub rings (r 32) pass 4 mm over it at every angle.
      - a yoke on top, as wide as the cheeks, with 45 deg chamfers under its overhangs.
      - two cheeks (|y| 44..54) rising round J1. The boss holes are diamonds, so they print without support.
  - shoulder boss (x2, the same part; print it flange down):
      - D40 shank through the 6808 to the sprocket;
      - D43 collar (|y| 40..44) that presses only the 6808's inner ring - the two collars locate link A;
      - 44 square key, set as a diamond, in the cheek: it carries the joint's torque;
      - D80 flange on the cheek's outside;
      - D16 through the middle (wires along the joint axis, if wanted).
Assembly (service first):
  1. Link A goes in from above with its bearings, chains and sprockets in (88 between the cheeks, 80 across its hubs).
  2. Each boss pushes in from outside, through the cheek and the 6808.
  3. 4 x M3 x 30 from the flange into the sprocket (tapped M3; or M3 x 35 + a nyloc inside).
  4. 4 x M3 x 8 through the flange into M3 x 4 x 5 inserts in the cheek.
  Reverse it to take link A out; the column stays on the turntable.

Frames: built in the chain design frame (x along link A at J1 = 0, y the joint axis, z up, J1 at the origin), turned
into the arm's frame with chain_drive.to_scene and moved +Z_MID in z so the chain plane is the arm's plane (the yaw
axis, 30.6). The gearbox studio shows the chain links at z 0: in main they get offset [0, 0, 30.6].
Out: complete assembly/chain_base.step. arm_base_tube.step is left alone (twin_motor.py and torque_test.py read it).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

import arm_base as AB  # noqa: E402
import arm_tube as AT  # noqa: E402
import chain_drive as D  # noqa: E402
import chain_drive_compact as CC  # noqa: E402
import chain_drive_arm as CA  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "complete assembly", "chain_base.step")
Z_MID = AB.Z_MID                      # 30.6: the arm's plane (yaw axis) in the arm frame
COL_TOP = 36.0                        # the column's top, below J1: link A's hub rings are r 32
YOKE_T = 16.0                         # yoke from COL_TOP to COL_TOP + YOKE_T below J1
COL_X = AB.TUBE_COLUMN_X              # 30: the column's front / back faces
SPR_OUT = CC.YC + D.SPR_T / 2         # 28.4: the sprocket's outer face = the boss's inner end
BRG = (CC.WALL_Y, CC.WALL_Y + 7.0)    # 6808 in link A's walls, |y| 33..40
SHANK_D = 40.0                        # the 6808's bore
COLLAR, COLLAR_D = (40.0, 44.0), 43.0  # 6808 inner ring shoulder ~43.6: the collar touches the inner ring only
CHEEK = (44.0, 54.0)                  # cheek faces at |y| >= 44 (gearbox session); 10 thick
CHEEK_R = 42.0
KEY_AF, KEY_CLEAR = 44.0, 0.2         # square key, set as a diamond; the hole 0.2 bigger across flats
FLANGE, FLANGE_D = (54.0, 58.0), 80.0
CORE_D = 16.0
FL_SCREW_R = 35.0                     # flange screws on the diamond's flats (45 deg off its corners)
INSERT_HOLE, INSERT_DEPTH = 4.2, 4.5  # Mike's M3 x 4 x 5 (cycloid.py's numbers)
M3_CLEAR, CBORE_D, CBORE_T = 3.4, 6.5, 3.5
SCREW_LEN, FL_SCREW_LEN = 30.0, 8.0
WIRE_D, WIRE_X = 16.0, -18.0          # wires into the column: behind J1 (design -x), on the arm's plane
BACK_R, BACK_TO, BACK_Y = 50.0, 245.0, 41.0   # the wedge for link A's top plate corner (r 48): clear to J1 155 deg
SWEEP = (2.5, 7.6, 20.0, 40.0, 60.0, 75.0, 90.0, 100.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0, 140.0, 142.2, 147.3)
# J1 2.5..147.3: 144.8 deg of nut travel centred on chess 7.6..142.2 (gearbox session 2026-11-20)
PRINT, STEEL, BRASS, DARK = (0.72, 0.74, 0.78), (0.78, 0.79, 0.81), (0.85, 0.65, 0.25), (0.15, 0.15, 0.16)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_y(d, y0, y1, x=0.0, z=0.0):
    return cq.Solid.makeCylinder(d / 2, y1 - y0, V(x, y0, z), V(0, 1, 0))


def ys(side, a, b):
    return tuple(sorted((side * a, side * b)))


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} pieces"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def to_arm(s):
    """chain design frame -> the arm frame, chain plane on the arm's plane"""
    return D.to_scene(s).translate(V(0, 0, Z_MID))


def diamond(af, y0, y1):
    """a square af across flats on the joint axis, corners up / down / fore / aft"""
    return box(-af / 2, af / 2, y0, y1, -af / 2, af / 2).rotate(V(0, 0, 0), V(0, 1, 0), 45)


def flange_screw_pts():
    return [(FL_SCREW_R * math.cos(math.radians(a)), FL_SCREW_R * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]


def boss(side, bolts):
    """one shoulder boss (design frame) on side +1 / -1"""
    b = cyl_y(SHANK_D, *ys(side, SPR_OUT, COLLAR[0])).fuse(cyl_y(COLLAR_D, *ys(side, *COLLAR)))
    b = b.fuse(diamond(KEY_AF, *ys(side, *CHEEK))).fuse(cyl_y(FLANGE_D, *ys(side, *FLANGE)))
    b = b.cut(cyl_y(CORE_D, -100, 100))
    for x, z in bolts:
        b = b.cut(cyl_y(M3_CLEAR, -100, 100, x, z))
        b = b.cut(cyl_y(CBORE_D, *ys(side, FLANGE[1] - CBORE_T, FLANGE[1] + 1), x, z))
    for x, z in flange_screw_pts():
        b = b.cut(cyl_y(M3_CLEAR, *ys(side, FLANGE[0] - 1, FLANGE[1] + 1), x, z))
    return one(b.clean(), "boss")


def cheek(side):
    zb = -(COL_TOP + YOKE_T)
    trap = (cq.Workplane("XZ", origin=(0, 0, 0)).polyline([(-CHEEK_R, 0), (CHEEK_R, 0), (COL_X, zb), (-COL_X, zb)])
            .close().extrude(-(CHEEK[1] - CHEEK[0])).val().translate(V(0, CHEEK[0], 0)))
    c = cyl_y(2 * CHEEK_R, *CHEEK).fuse(trap)
    if side < 0:
        c = c.mirror("XZ")
    return c


def yoke():
    z0, z1 = -(COL_TOP + YOKE_T), -COL_TOP
    y = box(-COL_X, COL_X, -CHEEK[1], CHEEK[1], z0, z1)
    drop = CHEEK[1] - 35.0                                   # 45 deg chamfers down to the column's sides (|y| ~35)
    ch = (cq.Workplane("YZ", origin=(-COL_X, 0, 0)).polyline([(-CHEEK[1], z0), (CHEEK[1], z0), (35.0, z0 - drop),
                                                               (-35.0, z0 - drop)]).close().extrude(2 * COL_X).val())
    return y.fuse(ch)


def base_cuts(bolts):
    """design-frame cuts: the diamond holes, the flange screws' inserts, the wire hole into the column"""
    out = []
    for side in (1, -1):
        out.append(diamond(KEY_AF + KEY_CLEAR, *ys(side, CHEEK[0] - 1, CHEEK[1] + 1)))
        for x, z in flange_screw_pts():
            out.append(cyl_y(INSERT_HOLE, *ys(side, CHEEK[1] - INSERT_DEPTH, CHEEK[1] + 1), x, z))
            out.append(cyl_y(3.2, *ys(side, CHEEK[1] - INSERT_DEPTH - 3.0, CHEEK[1]), x, z))
    zb = -(COL_TOP + YOKE_T) - (CHEEK[1] - 35.0)
    out.append(cq.Solid.makeCylinder(WIRE_D / 2, (-COL_TOP + 1) - (zb - 6), V(WIRE_X, 0, zb - 6), V(0, 0, 1)))
    # folded back, link A's top plate corner (x 0, z 48: r 48) swings down behind J1 onto the yoke's back edge (0 clear
    # at 147.4): a wedge off the yoke between the walls, r BACK_R, from straight back down to BACK_TO deg
    a0, a1 = math.radians(180.0), math.radians(BACK_TO)
    pts = [(0.0, 0.0)] + [(BACK_R * math.cos(a0 + (a1 - a0) * k / 24), BACK_R * math.sin(a0 + (a1 - a0) * k / 24))
                          for k in range(25)]
    out.append(cq.Workplane("XZ").polyline(pts).close().extrude(-2 * BACK_Y).val().translate(V(0, -BACK_Y, 0)))
    return out


def hardware(bolts):
    """screws and inserts (design frame) as compounds, one set per side"""
    out = []
    for side, nm in ((1, "+y"), (-1, "-y")):
        seat = FLANGE[1] - CBORE_T
        sc = [cyl_y(3.0, *ys(side, seat - SCREW_LEN, seat), x, z).fuse(cyl_y(5.5, *ys(side, seat, seat + 3.0), x, z))
              for x, z in bolts]
        out.append((f"sprocket screws M3 x {SCREW_LEN:g} {nm} (flange into the tapped sprocket)",
                    cq.Compound.makeCompound(sc), DARK))
        fs = [cyl_y(3.0, *ys(side, FLANGE[1] - FL_SCREW_LEN, FLANGE[1]), x, z)
              .fuse(cyl_y(5.5, *ys(side, FLANGE[1], FLANGE[1] + 3.0), x, z)) for x, z in flange_screw_pts()]
        out.append((f"flange screws M3 x {FL_SCREW_LEN:g} {nm}", cq.Compound.makeCompound(fs), DARK))
        ins = [cyl_y(4.6, *ys(side, CHEEK[1] - 4.0, CHEEK[1]), x, z).cut(cyl_y(3.0, -100, 100, x, z))
               for x, z in flange_screw_pts()]
        out.append((f"M3 x 4 x 5 inserts {nm} (cheek)", cq.Compound.makeCompound(ins), BRASS))
    return out


def build():
    a_mv, a_joint, _ = CA.build_link("A")
    bolts = CA.bolt_pts_of(a_joint)
    assert len(bolts) == 4
    col, _ = AT.base_parts()                                               # arm frame
    col = col.intersect(AB.box(-300, 300, COL_TOP, 400, -200, 200))
    new = yoke().fuse(cheek(1)).fuse(cheek(-1))
    body = col.fuse(to_arm(new))
    for c in base_cuts(bolts):
        body = body.cut(to_arm(c))
    body = one(body.clean(), "chain base body")
    bosses = [(f"shoulder boss {nm} (printed, flange down)", boss(side, bolts)) for side, nm in ((1, "+y"), (-1, "-y"))]
    sprockets = [(n, s) for n, s, _ in a_joint if "25T sprocket" in n]
    return body, bosses, sprockets, hardware(bolts), a_mv, bolts


def sweep_check(body, bosses, sprockets, a_mv):
    """link A (design frame) at each J1 angle against the base, all in the arm frame; returns the clashes and the least
    gap from the column / yoke / cheeks to link A's printed walls and floor"""
    near = body.intersect(AB.box(-70, 160, -70, 140, -60, 120))           # all link A can reach: faster checks
    from view_step import read_step
    servo = [(f"claw servo: {n}", s) for n, s, _ in read_step(os.path.join(ROOT, "complete assembly", "servo_on_base.step"))]
    fixed = [("chain base body", near)] + [(n, to_arm(s)) for n, s in bosses + sprockets] + servo
    moving = [(n, to_arm(s)) for n, s, _ in a_mv if not isinstance(s, cq.Compound)]
    hits, gaps = [], []
    for th in SWEEP:
        for n, s in moving:
            r = s.rotate(V(0, 0, 0), V(0, 0, 1), th)
            for m, t in fixed:
                if "6808" in n and "boss" in m:                            # the inner ring on the shank: meant to touch
                    continue
                if "roller chain" in n and "25T sprocket" in m:            # the chain turned rigidly with the link
                    continue                                               # lands off the teeth: not a real clash
                bb1, bb2 = r.BoundingBox(), t.BoundingBox()
                if (bb1.xmax < bb2.xmin or bb2.xmax < bb1.xmin or bb1.ymax < bb2.ymin or bb2.ymax < bb1.ymin
                        or bb1.zmax < bb2.zmin or bb2.zmax < bb1.zmin):
                    continue
                v = r.intersect(t).Volume()
                if v > 0.01:
                    hits.append((th, round(v, 2), n, m))
            if n.startswith(("link A wall", "link A floor", "link A top plate")):
                gaps.append((round(r.distance(near), 2), th, n))
    return hits, sorted(gaps)[:6]


def main():
    body, bosses, sprockets, hw, a_mv, bolts = build()
    bb = body.BoundingBox()
    print(f"chain base body {body.Volume() / 1000:.1f} cm^3 (PA12 ~{body.Volume() / 1000 * 1.01:.0f} g), x {bb.xmin:.1f}.."
          f"{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}")
    for n, s in bosses:
        print(f"{n}: {s.Volume() / 1000:.1f} cm^3")
    print(f"sprocket bolts (design x, z): " + ", ".join(f"({x:.2f}, {z:.2f})" for x, z in bolts))
    seat = FLANGE[1] - CBORE_T
    print(f"sprocket screws: head seat |y| {seat:.1f}, tip |y| {seat - SCREW_LEN:.1f} (sprocket |y| "
          f"{CC.YC - D.SPR_T / 2:.1f}..{SPR_OUT:.1f}): {SPR_OUT - (seat - SCREW_LEN) - D.SPR_T:+.1f} past its inner face")
    # the boss in the 6808 / the diamond in its hole / the boss against the body (should be 0)
    for n, s in bosses:
        v = to_arm(s).intersect(body).Volume()
        print(f"   {n} x base body: {v:.3f} mm^3")
    hits, gaps = sweep_check(body, bosses, sprockets, a_mv)
    print(f"J1 sweep {SWEEP[0]}..{SWEEP[-1]} ({len(SWEEP)} angles): {len(hits)} clashes")
    for h in hits[:80]:
        print(f"   {h[0]:6.1f} deg {h[1]:8.2f} mm^3  {h[2][:50]}  <->  {h[3][:40]}")
    for g, th, n in gaps:
        print(f"   least gap {g:.2f} mm at {th:g} deg: {n[:50]} - base body")
    servo = os.path.join(ROOT, "complete assembly", "servo_on_base.step")
    from view_step import read_step
    for n, s, _ in read_step(servo):
        v = s.intersect(body).Volume()
        if v > 0.5 and n not in ("servo bracket", "servo cover"):
            print(f"   OVERLAP {n} x base body: {v:.1f} mm^3")
    pcb, chip, _ = AT.hall_parts()
    parts = [("chain base body", body, PRINT)]
    parts += [(n, to_arm(s), PRINT) for n, s in bosses]
    parts += [(n, to_arm(s), DARK) for n, s in sprockets]
    parts += [(n, to_arm(s), c) for n, s, c in hw]
    parts += [("J0 Hall breakout", pcb, (0.1, 0.42, 0.22)), ("J0 Hall DRV5032FB", chip, DARK)]
    write_flat([(n, s, cq.Color(*c)) for n, s, c in parts], OUT)
    print(OUT)


if __name__ == "__main__":
    main()

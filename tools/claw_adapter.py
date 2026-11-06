#!/usr/bin/env python3
"""
claw_adapter.py - the wrist piece that carries Mike's spring-closed, Bowden-opened claw on the last gearbox (J4)'s
output, at the far end of link C (2026-11-04).

    python claw_adapter.py

Drive frame (as cycloid.py and the stand-in): J4 axis on z, housing bottom z 0 = top of link C's plate.
  - the arm links' output connection: cup floor z 45.2-53.2 on the clamp ring, 3 x 4.4 holes on r 15 with 8 mm
    counterbores (M3 x 20 into the flange inserts), the 43 riser and the 40 boss (z 53.2-60.2) into the 6808 that sits
    in link C's cap pocket
  - an arc round the drive housing (73 bore, 4 mm wall, z 0.8-45.2: above link C's plate, under its cap), wrapping
    330 deg; the 30 deg opening is on the side away from the claw
  - a projection out to a flat face (56 wide, full height), the claw cap's two M4 screws go into it (tapped 3.3,
    12 deep). Behind the face a window (16 wide, through) holds the PC4-M6 Bowden fitting on the cap's boss and lets
    the tube bend out sideways instead of running into the drive.
  - the claw hangs along +y: with link C level and J4 at 0 it points straight down (the viewer's frame).
Out (robo_chess/gripper/): claw_adapter.step (the printed part), claw_on_wrist.step (adapter + claw, closed, placed).

    python claw_adapter.py --spread

For Mike's tube links (2026-11-05) on the SPREAD gearbox (output face z 59.2, no outer 6808): the cup is the tube
links' own (70.5 x 8, z 59.2-67.2, same 3 screws and counterbores), no riser / boss (nothing above it now), no arc
round the gearbox (link C's end wall is 2.75 from it), the projection drops from the cup to the same face.
Claw axis (2026-11-05, Mike: "fix the claw"): at drive z Z_CLAW_SPREAD 30.6, the swivel's centre line (the turntable,
both 6808s and the collar are centred on design z 30.6), which is where the IK aims. At 27.0 (the middle of the
sleeve) it hung 3.6 mm to the side, so its three fingers would have pulled every piece 3.6 mm off its square.
Out: claw_adapter_spread.step, claw_on_wrist_spread.step.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402

import cycloid as C  # noqa: E402
from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "gripper")
CLAW = os.path.join(OUT, "gripper_assembly_closed.step")

Z_FLOOR0, Z_CUP, Z_RISER, Z_BOSS = 45.2, 53.2, 54.2, 60.2      # as the arm links' pivot end
BOSS_D, RISER_D = 40.0, 43.0
OUT_HOLE_D, OUT_CB_D, Z_CB = 4.4, 8.0, 52.7
R_BORE, WALL = 36.5, 4.0                                        # same 73 bore as the links' sleeves
Z_SLEEVE0 = 0.8                                                 # above link C's plate (its top is z 0)
GAP_DEG = 30.0                                                  # opening in the arc, away from the claw
FACE_Y, FACE_T, FACE_HALF_W = 70.0, 6.0, 28.0                   # face centre (0, FACE_Y); plate 6 thick, 56 wide
FACE_TURN = -45.0        # Mike: face (and the slot behind it) turned 45 deg about the face centre, so the Bowden line
#                          misses the drive (passes FACE_Y * sin 45 = 49.5 from the wrist axis). Turned this way (not
#                          +45) the claw hangs straight down at J4 = +45, the side with no stop, and the slot then
#                          points straight up; +45 put the claw's vertical at J4 = -45, past the -23.6 stop at link C.
FITTING_D = 16.0         # Bowden bore: room for the PC4-M6 fitting, then straight out through the top, one size
M4_CB_D = 8.5                                                   # deep counterbores for the two M4 socket heads
Z_CLAW = 27.0                                                   # claw axis height (middle of 0.8..53.2), old arm
Z_CLAW_SPREAD = 30.6     # the tube-link arm: on the swivel's centre line, where the IK aims (was 27.0: 3.6 mm off).
#                          The old arm's face ends at z 53.2, too low for the cap's upper M4 at 30.6 + 20: left at 27.
M4_CLEAR, M4_DZ = 4.5, 20.0                                     # claw cap's M4 pair, now along the wrist axis
BOSS_HOLE_D = 14.0                                              # the cap's 12 boss + fitting
EDGE_CHAMFER = 6.0


def face_frame():
    """u: claw direction (out of the face, towards the fingers); w: across the face, in the arm's plane."""
    a = math.radians(FACE_TURN)
    u = (-math.sin(a), math.cos(a))
    w = (math.cos(a), math.sin(a))
    return (0.0, FACE_Y), u, w


def at(f, u, w, du, dw):
    return (f[0] + du * u[0] + dw * w[0], f[1] + du * u[1] + dw * w[1])


def cyl(d, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


SPREAD_FLOOR, SPREAD_CUP, SPREAD_CUP_D = 59.2, 67.2, 70.5      # the spread gearbox's output face; the tube links' cup
SPREAD_LOW_R, SPREAD_LOW_Z = 44.0, 12.0                         # nothing inside r 44 below z 12 (link C's gusset points)
# J4 home magnet (Mike 2026-11-05: "6mm magnet into that face above the Hall effect switch ... do this to all the
# remaining joints"): link C's Hall (mockup_link.py HALL_*) sits at +x from J4 in link C's frame; with the wrist at its
# home (scene J4 90) that faces this cup's rim at 0 - 90 = -90 deg (the side away from the claw's tab). A 6 x 2 magnet
# radially in the rim, 3 deep (Mike) with the magnet at the bottom, 1.0 under the rim, centred on the cup's 8 (0.9 of
# wall above and below; no round on this cup).
J4_MAGNET_DEG, J4_MAGNET_Z = -90.0, (SPREAD_FLOOR + SPREAD_CUP) / 2
MAGNET_D, MAGNET_DEPTH = 6.2, 3.0


def j4_magnet(pocket=True):
    """the pocket (or, pocket=False, the magnet itself and its outer-face centre), drive frame"""
    r, a = SPREAD_CUP_D / 2, math.radians(J4_MAGNET_DEG)
    d = V(math.cos(a), math.sin(a), 0)
    if pocket:
        return cq.Solid.makeCylinder(MAGNET_D / 2, MAGNET_DEPTH + 1, V(0, 0, J4_MAGNET_Z) + d * (r + 1), d * -1)
    face = V(0, 0, J4_MAGNET_Z) + d * (r - (MAGNET_DEPTH - 2.0))
    return cq.Solid.makeCylinder(3.0, 2.0, face, d * -1), face


def adapter(spread=False):
    r_out = R_BORE + WALL
    z_floor, z_cup = (SPREAD_FLOOR, SPREAD_CUP) if spread else (Z_FLOOR0, Z_CUP)
    z_cb = z_cup - 4.0 if spread else Z_CB
    if spread:          # the tube links' cup, nothing above it, no arc (link C's end wall sits 2.75 from the gearbox)
        body = cyl(SPREAD_CUP_D, z_floor, z_cup)
    else:
        # cup floor + riser + boss (the arm links' output connection)
        body = cyl(2 * r_out, Z_FLOOR0, Z_CUP).fuse(cyl(RISER_D, Z_CUP, Z_RISER)).fuse(cyl(BOSS_D, Z_RISER, Z_BOSS))
        # the arc round the drive, 330 deg: a full ring with a 30 deg wedge taken out at -y (away from the claw)
        ring = cyl(2 * r_out, Z_SLEEVE0, Z_FLOOR0 + 0.01).cut(cyl(2 * R_BORE, Z_SLEEVE0 - 1, Z_FLOOR0))
        h = r_out + 5
        a = math.radians(GAP_DEG / 2)
        wedge = cq.Workplane("XY", origin=(0, 0, Z_SLEEVE0 - 1)).polyline(
            [(0, 0), (h * math.sin(a), -h * math.cos(a)), (-h * math.sin(a), -h * math.cos(a))]).close().extrude(Z_FLOOR0 - Z_SLEEVE0 + 1).val()
        body = body.fuse(ring.cut(wedge))
    # the projection out to the flat: the face plate (turned FACE_TURN) and a body back to the ring, full height
    f, u, w = face_frame()
    c = EDGE_CHAMFER
    pts = [at(f, u, w, 0, -FACE_HALF_W + c), at(f, u, w, 0, FACE_HALF_W - c), at(f, u, w, -c, FACE_HALF_W),
           at(f, u, w, -FACE_T - 4, FACE_HALF_W), (r_out - 6.0, 18.0), (-(r_out - 6.0), 18.0),
           at(f, u, w, -c, -FACE_HALF_W)]
    tab = cq.Workplane("XY", origin=(0, 0, Z_SLEEVE0)).polyline(pts).close().extrude(z_cup - Z_SLEEVE0).val()
    body = body.fuse(tab)
    cuts = [cyl(2 * R_BORE, Z_SLEEVE0 - 1, z_floor)]                                   # the drive's room
    if spread:          # the tube link C's outer elbow gussets (z 0..10 near their points, r 38..42 from J4) caught the
        cuts.append(cyl(2 * SPREAD_LOW_R, Z_SLEEVE0 - 1, SPREAD_LOW_Z))  # tab's low inner corner at J4 ~160: cleared
        cuts.append(j4_magnet())                                        # the J4 home magnet's pocket
    for ang in C.OUT_SCREW_ANGLES:                                                     # output screws
        x, y = C.polar(C.OUT_SCREW_R, ang)
        cuts += [cyl(OUT_HOLE_D, z_floor - 1, z_cb + 0.01, x, y), cyl(OUT_CB_D, z_cb, max(Z_BOSS, z_cup) + 1, x, y)]
    # Mike: no open channel. Solid, with round holes straight along the Bowden line out through the top:
    #   the cap's boss through the plate, a pocket behind it for the PC4-M6 fitting, then the tube hole;
    #   the two M4s: clearance through the plate and a deep counterbore each, so a hex key reaches the heads.
    fv = V(f[0], f[1], Z_CLAW_SPREAD if spread else Z_CLAW)
    back = V(-u[0], -u[1], 0)
    fwd = V(u[0], u[1], 0)
    cuts.append(cq.Solid.makeCylinder(BOSS_HOLE_D / 2, FACE_T + 2, fv + back * (FACE_T + 1), fwd))
    # Mike: one 16 bore straight through - no shoulder inside for the cable or tube to catch on
    cuts.append(cq.Solid.makeCylinder(FITTING_D / 2, 160, fv + back * (FACE_T + 160 - 0.01), fwd))
    for sz in (-1, 1):
        p = fv + V(0, 0, sz * M4_DZ)
        cuts.append(cq.Solid.makeCylinder(M4_CLEAR / 2, FACE_T + 2, p + back * (FACE_T + 1), fwd))
        cuts.append(cq.Solid.makeCylinder(M4_CB_D / 2, 160, p + back * (FACE_T + 160 - 0.01), fwd))
    for cu in cuts:
        body = body.cut(cu)
    s = body.clean().Solids()
    assert len(s) == 1, f"{len(s)} pieces"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), "not valid"
    return s[0]


# The Bowden's anchor at the claw (Mike 2026-11-05: "anchor the Bowden tube at the gripper too"): gripper.py's PC4-M6
# push fitting, screwed into the cap's 12 x 8 boss (its 5 mm hole - the fitting cuts its own thread), standing in this
# adapter's 16 bore; the tube runs down the bore into its collet. Drawn like the servo end's PC4-M10 (servo_tube.py),
# sizes GUESSED for a common PC4-M6 - measure yours: hex 10 across flats x 5.5, a 9 neck, the collet 8.5 with a 10
# collar, 13.5 tall above the boss in all; the thread in the boss is not drawn.
CAP_BOSS_TOP = 64.0                                             # claw frame: cap plate top 56 + the 8 boss
PC4M6_HEX_AC, PC4M6_HEX_T, PC4M6_NECK_D, PC4M6_COLLET_D, PC4M6_TALL, PC4M6_BORE = 11.55, 5.5, 9.0, 8.5, 13.5, 4.1


def fitting_m6():
    """(body, collet) of the PC4-M6 on the cap's boss, claw frame (cap top z 56, up = away from the fingers)"""
    z0 = CAP_BOSS_TOP

    def pl(z):
        return cq.Plane(origin=(0, 0, z), xDir=(1, 0, 0), normal=(0, 0, 1))
    body = cq.Workplane(pl(z0)).polygon(6, PC4M6_HEX_AC).extrude(PC4M6_HEX_T).val()
    body = body.fuse(cq.Workplane(pl(z0 + PC4M6_HEX_T - 0.01)).circle(PC4M6_NECK_D / 2).extrude(3.01).val())
    collet = cq.Workplane(pl(z0 + PC4M6_HEX_T + 3.0)).circle(PC4M6_COLLET_D / 2).extrude(PC4M6_TALL - PC4M6_HEX_T - 4.0).val()
    collet = collet.fuse(cq.Workplane(pl(z0 + PC4M6_TALL - 1.0)).circle(PC4M6_NECK_D / 2 + 0.5).extrude(1.0).val())
    bore = cq.Solid.makeCylinder(PC4M6_BORE / 2, PC4M6_TALL + 2, V(0, 0, z0 - 1))
    return body.cut(bore).clean(), collet.cut(bore).clean()


def claw_to_wrist(spread=False):
    """Claw frame (hub bottom z 0, cap top z 56, fingers to -z) -> drive frame: claw axis along u (fingers out of the
    face), cap top centred on the face; the cap's M4 pair (claw y) along the wrist axis (drive z)."""
    f, u, w = face_frame()
    # columns: claw x -> w, claw y -> drive z, claw z -> -u
    r = [[w[0], 0.0, -u[0]], [w[1], 0.0, -u[1]], [0.0, 1.0, 0.0]]
    t3 = (f[0] - 56.0 * r[0][2], f[1] - 56.0 * r[1][2], (Z_CLAW_SPREAD if spread else Z_CLAW) - 56.0 * r[2][2])
    t = gp_Trsf()
    t.SetValues(r[0][0], r[0][1], r[0][2], t3[0],
                r[1][0], r[1][1], r[1][2], t3[1],
                r[2][0], r[2][1], r[2][2], t3[2])
    return t


def main():
    spread = "--spread" in sys.argv
    tag = "_spread" if spread else ""
    a = adapter(spread)
    bb = a.BoundingBox()
    print(f"adapter: {a.Volume() / 1000:.1f} cm^3 ({a.Volume() * 1.10e-3:.0f} g solid nylon), "
          f"x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}")
    os.makedirs(OUT, exist_ok=True)
    one = cq.Assembly(name="claw_adapter")
    one.add(a, name="claw adapter", color=cq.Color(0.55, 0.75, 0.45))
    write_step(one, os.path.join(OUT, f"claw_adapter{tag}.step"))

    t = claw_to_wrist(spread)
    claw =[(n, cq.Shape.cast(s.wrapped.Moved(cq.Location(t).wrapped))) for n, s, _ in read_step(CLAW)]
    if spread:          # Mike (2026-11-05): no dummy chess piece in the claw - he drags the arm by the gripper itself
        claw = [(n, s) for n, s in claw if n != "piece_stand_in"]
    cap = [s for n, s in claw if n == "cap"][0]
    cb = cap.BoundingBox()
    print(f"claw cap top at y {cb.ymin:.2f} (face at {FACE_Y}); claw reaches y {max(s.BoundingBox().ymax for _, s in claw):.1f}")
    fit, collet = (cq.Shape.cast(s.wrapped.Moved(cq.Location(t).wrapped)) for s in fitting_m6())
    claw += [("PC4-M6 fitting", fit), ("fitting collet", collet)]
    for n, s in claw:
        v = s.intersect(a).Volume()
        if v > 0.01:
            print(f"   OVERLAP adapter x claw {n}: {v:.2f} mm^3")
    for n, s in claw[-2:]:
        gap = min(s.distance(a), *(s.distance(o) for m, o in claw if m not in ("cap", "PC4-M6 fitting", "fitting collet")))
        print(f"   {n}: {gap:.2f} mm clear of the adapter and the claw's other parts")
    f, u, w = face_frame()
    top = PC4M6_TALL + CAP_BOSS_TOP - 56.0                     # the collet's top, back from the face along -u
    zc = Z_CLAW_SPREAD if spread else Z_CLAW
    for n, s in claw:                                          # the claw's parts: where its axis is, and what is closest
        if n == "hub":
            hb = s.BoundingBox()
            print(f"   claw axis at drive z {(hb.zmin + hb.zmax) / 2:.2f} (hub z {hb.zmin:.2f}..{hb.zmax:.2f}); "
                  f"face z {Z_SLEEVE0}..{SPREAD_CUP if spread else Z_CUP}")
    print(f"   tube end (the collet's top), drive frame: [{f[0] - top * u[0]:.2f}, {f[1] - top * u[1]:.2f}, {zc}], "
          f"into the claw along [{u[0]:.4f}, {u[1]:.4f}, 0]")
    assy = cq.Assembly(name="claw_on_wrist")
    assy.add(a, name="claw adapter", color=cq.Color(0.55, 0.75, 0.45))
    cols = {"hub": (0.85, 0.85, 0.87), "cap": (0.85, 0.85, 0.87), "plunger": (0.95, 0.75, 0.3), "piece_stand_in": (0.2, 0.2, 0.22),
            "PC4-M6 fitting": (0.78, 0.77, 0.72), "fitting collet": (0.17, 0.42, 0.84)}
    for n, s in claw:
        assy.add(s, name=f"claw {n}", color=cq.Color(*cols.get(n, (0.9, 0.9, 0.92))))
    if spread:
        mag, face = j4_magnet(pocket=False)
        if mag.intersect(a).Volume() > 0.01:
            print(f"   OVERLAP J4 magnet x adapter: {mag.intersect(a).Volume():.2f}")
        print(f"   J4 home magnet: rim at {J4_MAGNET_DEG:+.0f} deg, face centre ({face.x:.2f}, {face.y:.2f}, {face.z:.2f})")
        assy.add(mag, name="J4 home magnet 6x2", color=cq.Color(0.82, 0.83, 0.86))
    write_step(assy, os.path.join(OUT, f"claw_on_wrist{tag}.step"))
    print(os.path.join(OUT, f"claw_adapter{tag}.step"))
    print(os.path.join(OUT, f"claw_on_wrist{tag}.step"))


if __name__ == "__main__":
    main()

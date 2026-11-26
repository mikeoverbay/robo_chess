#!/usr/bin/env python3
"""
chain_drive_arm.py - the shoulder (link A, J1) and elbow (link B, J2) chain drives, both laid out the same way
(Mike, 2026-11-20): "move that 60-millimeter long body motor on the shoulder ... up so it looks the same as the other
ones. That way we can get rid of that extra long coupler ... and just use a belt drive", on the factory-machined
SFU1204s he ordered ("I have the 200 millimeter and the 150 millimeter in my cart"; standard BK10/BF10 ends: drive end
D8 x 15 | M10x1 x 16 | D10 x 23 seat, other end D8 x 10 with a snap-ring groove 7.9 from the step).

    python chain_drive_arm.py [--ends]

Each link, from its inner joint out: two fixed 25T sprockets (on that joint's bosses), the wrap idlers (2 x 625 on a
5 mm shaft through both walls), the screw's free end, the nut (flange inboard, the aluminium carrier round its body, a
steel chain tab each side), the far idlers (5 mm shaft from the wall to an inboard upright), then the screw's machined
drive end: AXK1024 | 12 mm printed wall with a 6000 | AXK1024 | spacer, clamped on the D10 seat by an M10x1 locknut;
a 60T (his, 8 mm bore) on the D8 stub, belted 1:1 to a 60T on the motor, which sits on the top plate over the nut.

Why 1:1, not 3:1 (gearbox session 2026-11-20): the arm's planned speeds are J1 45 and J2 80 deg/s. Through 3:1 (119:1
overall) the elbow's 48 mm motor would have to spin ~1600 rpm, where it has no torque at 24 V. At 1:1 (39.8:1):
  J1 60 mm: worst gravity 10.56 N.m (motor_speed.py) + ~0.7 for its own motor now out at the elbow end = ~11.3 N.m:
     446 N of chain pull in all, screw 0.315 N.m, motor 0.33 N.m vs 0.495 pull-out = x1.5. Stalled at full current
     it pulls 980 N in all, ~575 N on the tighter chain with 100 N pretension (#25 working 623 N): cap at ~80 %.
  J2 48 mm: ~5.9 N.m: 233 N, screw 0.165, motor 0.174 N.m at 531 rpm (80 deg/s) vs 0.318 = x1.8. Stall 834 N in all.
  Belt (6 mm GT2 on 60T, PD 38.2): 17 N at J1, 9 N at J2.
Screws: link A from the 200 mm, plain end cut off: 166 overall (112 of thread). Link B the 150 mm as it comes (86 of
thread, 1.5 mm spare each end of the nut's travel; its D8 x 10 end takes a 608 and a snap ring at the elbow).

Frames as in chain_drive_compact.py / chain_drive_elbow.py (design: x along the link from its inner joint, y the joint
axis, z up). Writes the studio's link A, base and link B files (replacing the inline-motor version's).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import nema17  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402
import chain_drive as D  # noqa: E402
import chain_drive_compact as C  # noqa: E402
from chain_drive import (cyl_x, cyl_y, box, ring_x, ring_y, one, hull_pts, hull_solid, chain, to_scene, pulley,  # noqa: E402
                         PRINT, STEEL, ALU, DARK, BLACK, BRONZE)

L = 200.0
Z_S, YC, WALL_Y, TOP_Z, FLOOR_Z = C.Z_S, C.YC, C.WALL_Y, C.TOP_Z, C.FLOOR_Z
# the studios' files on the arm's plane: the yaw axis, the claw and the board are at model z 30.6 (main session +
# gearbox session 2026-11-21; Mike: centre the arm over the turntable and line it up with the board). The design STEP
# stays in the design frame; chain_drive.to_scene itself is unchanged (chain_base.py adds its own 30.6)
Z_MID = 30.6


def to_studio(s):
    return to_scene(s).translate(V(0, 0, Z_MID))
ELBOW_SHOW = -60.0
SWEEP = (4.6, 0.0, -15.0, -30.0, -45.0, -60.0, -75.0, -90.0, -103.4, -104.0)
BELT_L = 210.0                                  # 60T + 60T 1:1: C 45, the two D42 flanges clear by 3 (TYPICAL length)
SLOT, SLOT_DN = 4.0, 0.0                        # motor slots: 4 up for tension, none down: the motor sits on the top plate (Mike 2026-11-20:
#                                                 "put some slots in those mounts ... a washer on the backside")


def belt_c(length, r1, r2):
    s = length - math.pi * (r1 + r2)
    return (s + math.sqrt(s * s - 8 * (r1 - r2) ** 2)) / 4


def drive(sp):
    """(motor pulley teeth, belt length, centre distance, motor axis z) for a link"""
    rm = sp["motor_t"] * 2.0 / math.pi / 2
    c = belt_c(sp["belt"], D.R60, rm)
    return sp["motor_t"], sp["belt"], c, Z_S + c
FLOOR_END = 168.0                               # floors stop 32 short of the next joint: its chains wrap there
# link C's wrist end (main session 2026-11-23; Mike, relayed by the herringbone session: "Arm session fixes link C"): the
# J4 herringbone planetary (herringbone_planetary.py) sits between the walls on the J4 axis, its frame's z 0 (the motor
# plate's underside, studio z 0) at design y = Z_MID, its 17HE08 pancake out through the +y wall. Body D70.5 (r 35.25); 6 x #4 x 3/4" wood screws up through the wall into its plate and half A on r 32.4 at
# cycloid.py's ARM_SCREW_ANGLES (360/38 x 3.5, 9.5 ... 34.5; gearbox frame, x_g = -x, y_g = -z here). The floor stops
# short of the ring, the +y wall's hub becomes a D73 ring with the motor's 43.3 square through it, a 2.4 seat inside
WR_BODY_R, WR_MOTOR, WR_SEAT_R, WR_RING_R = 35.25, 42.3 + 1.0, 35.0, 36.5
WR_SCREW_R, WR_SCREW_A = 32.4, tuple(360.0 / 38 * n for n in (3.5, 9.5, 15.5, 22.5, 28.5, 34.5))
WR_HOLE, WR_CB_D, WR_CB = 3.2, 6.5, 2.0         # #4 clearance; a 2 deep counterbore puts a 3/4" tip 11.6 into the gearbox
WR_FLOOR_END = L - math.sqrt(WR_RING_R ** 2 - FLOOR_Z[1] ** 2) - 0.5   # 163.2: clear of the ring (the body by 1.8)


def wrist_wall(w):
    """link C's +y wall at J4: the D73 ring, the seat inside, the pancake's square and the six screw holes"""
    w = w.fuse(cyl_y(2 * WR_RING_R, WALL_Y, WALL_Y + 7.0, L, 0)).fuse(cyl_y(2 * WR_SEAT_R, Z_MID, WALL_Y + 0.01, L, 0))
    sq = box(L - WR_MOTOR / 2, L + WR_MOTOR / 2, Z_MID - 1, WALL_Y + 8, -WR_MOTOR / 2, WR_MOTOR / 2)
    w = w.cut(sq.fillet(1.0, sq.edges("|Y")))
    for a in WR_SCREW_A:
        x, z = L - WR_SCREW_R * math.cos(math.radians(a)), -WR_SCREW_R * math.sin(math.radians(a))
        w = w.cut(cyl_y(WR_HOLE, Z_MID - 1, WALL_Y + 8, x, z)).cut(cyl_y(WR_CB_D, WALL_Y + 7.0 - WR_CB, WALL_Y + 8, x, z))
    return w
SPEC = {
    # the shoulder's belt (Mike 2026-11-20, in his cart): 60T on the screw, 32T 5 mm bore on the motor (1.875:1), 182 mm
    "A": dict(name="link A", travel=64.0, f_min=38.5, motor_l=60.0, wall_end=165.0, free="6801", motor_t=32, belt=182.0,
              screw="from the 200 mm, plain end cut: 166 overall"),
    "B": dict(name="link B", travel=48.0, f_min=38.5, motor_l=48.0, wall_end=165.0, free="608", motor_t=60, belt=BELT_L,
              screw="the 150 mm as it comes"),
    # J3 (2026-11-20, "chain drives on everything"): chess uses J3 -103.4..0, the same as the elbow -> link B's drive
    "C": dict(name="link C", travel=48.0, f_min=38.5, motor_l=48.0, wall_end=L, free="608", motor_t=60, belt=BELT_L,
              screw="a 150 mm as it comes"),
}
CHEEK_X0 = 125.0
CHEEK_JOG = 165.0                               # the cheek's Z steps out here (its jog web is x 161..165)
FIT = 0.1                                       # bearing seats a slip fit, not pressed (Mike 2026-11-20: "a couple of
#                                                 thousandths ain't going to kill us"); bosses 0.1 under


def path(xa, xb, x_i):
    """C.path with the far idler at x_i (copied from chain_drive_elbow.py)"""
    pts = []

    def line(p0, p1):
        n = max(2, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / 0.02))
        pts.extend((p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n) for k in range(n))

    def arc(c, r, a0, a1):
        n = max(2, int(abs(a1 - a0) * r / 0.02))
        pts.extend((c[0] + r * math.cos(a0 + (a1 - a0) * k / n), c[1] + r * math.sin(a0 + (a1 - a0) * k / n))
                   for k in range(n))

    al, cw, r1, r2, zl, zi = C.ALPHA, C.CW, C.R1, C.R2, C.ZL, C.Z_I
    s = (r1 * math.cos(-al), r1 * math.sin(-al))
    t = (cw[0] - r2 * math.cos(al), cw[1] + r2 * math.sin(al))
    line((xa, r1), (0.0, r1))
    arc((0.0, 0.0), r1, math.pi / 2, 2 * math.pi - al)
    line(s, t)
    arc(cw, r2, math.pi - al, math.pi / 2)
    line((cw[0], zl), (x_i, zl))
    arc((x_i, zi), r2, 1.5 * math.pi, 2.5 * math.pi)
    line((x_i, r1), (xb, r1))
    pts.append((xb, r1))
    return pts


def tri_prism(pts, y0, y1):
    """a triangle (x, z) points extruded from y0 to y1 (gussets)"""
    vs = [V(x, y0, z) for x, z in pts]
    return cq.Solid.extrudeLinear(cq.Wire.makePolygon(vs + vs[:1]), [], V(0, y1 - y0, 0))


def pulley_at(g, y, z):
    """chain_drive.pulley only bores x -100..100: build it 200 further in and slide it out"""
    sh = 200.0
    g2 = dict(g, teeth=(g["teeth"][0] - sh, g["teeth"][1] - sh), hub=(g["hub"][0] - sh, g["hub"][1] - sh))
    return pulley(g2, y, z).translate(V(sh, 0, 0))


# idler bosses (Mike 2026-11-20: "put a boss on the inside of that cover, on the side covers for the idlers, for a
# spacer"): a D7 boss 1.5 tall on the cover's inside and a matching one on the inner support (thrust wall foot / free-end
# wall, now |y| <= 20 there) bear on the 625s' inner race only (625 abutment d_a min 7), so the clamped stack no longer
# drags the outer race; the guide washers ride loose round the bosses with 0.5 mm float
BOSS_D, BOSS_H = 7.0, 1.5
IN_Y = 20.0                                     # the inner support's face (was 21)
WASH_BORE = 7.5
B_IN, B_OUT = IN_Y + BOSS_H, WALL_Y - BOSS_H     # the 625 pair's faces: 21.5 .. 31.5 (0.5 inboard of the chain plane)


def idler_stack(x, z, side, label):
    """two 625s between the bosses, a loose printed guide washer round each boss"""
    out = []
    for i, (a, b) in enumerate(((B_IN, B_IN + C.BRG[2]), (B_IN + C.BRG[2], B_OUT))):
        out.append((f"625 bearing ({label}) {i + 1}/2", ring_y(C.BRG[0], C.BRG[1], *C.ys(side, a, b), x, z), STEEL))
    for a in (IN_Y + 0.25, B_OUT + 0.25):
        out.append((f"guide washer ({label}, printed, D7.5 bore, loose round the boss)",
                    ring_y(WASH_BORE, C.GUIDE_D, *C.ys(side, a, a + C.GUIDE_T), x, z), PRINT))
    return out


def bosses(x, z, a, b):
    """a D7 idler boss on both sides, |y| a..b"""
    return cyl_y(BOSS_D, a, b, x, z).fuse(cyl_y(BOSS_D, -b, -a, x, z))


T0 = -3.5                                       # the screw's drive stub tip: the pulleys sit at the joint (Mike 2026-11-20:
#                                                 "Move the motors" - each motor by the joint it drives, not out at the far end)


def geometry(sp, nut_at):
    """the screw line, joint end out: pulleys on the D8 stub | motor plate | M10 locknut | spacer, AXK, thrust wall with
    a 6000, AXK on the D10 seat | step | the nut's travel | free-end wall (its bearing; the far idler shafts' inner ends)"""
    tr = sp["travel"]
    s = T0 + 54.0                               # the step from the D10 seat to the thread
    f_min = s + 1.5
    f_max = f_min + tr
    # links B / C: the far idler shaft two pitches out (the main session 2026-11-21, Mike in chaindrive on P514.F94: "I
    # want to move that shaft back. It's running in that block" - "One chain link"); link A's can't (its idler chain
    # would touch link B's on the elbow sprocket: gap -0.04 mm at one pitch more)
    return dict(s=s, t0=T0, f_min=f_min, f_max=f_max, x_i=f_max + 37.5 + (2 * D.P if sp["free"] == "608" else 0.0), f=f_min + tr * nut_at, motor_face=T0 + 23.0)


# ---------------------------------------------------------------- the covers' screws (Mike 2026-11-20: "We need to put in
# the holes to hold the covers on ... number 4 by 3/4 inch screws ... all the way around ... within 50 millimeter, but
# don't run into any of the guts" / "we can use 5/8, or I can use half" / "half Tylenol shaped little bubbles ... so
# that we can have more meat around our screws" / "Do all of them on the outside rather than the inside. And add a
# little bump out for the radius of where the screw's going into the covers"). Inside, the chain's lower run is 3.7 mm
# over the floor and the carrier sweeps under the top plate, so the bubbles sit OUTSIDE: on the top plate and under the
# floor along each side edge, a screw going in sideways through a rounded tab on the cover (or the cheek) - his M3
# shoulder screws now (below), first #4 x 1/2" pan heads
# into the bubble and the plate. His two faces (the thrust wall's side bridges) take one each, through the wall.
# his shoulder screws (Mike 2026-11-20, a picture: "M3 x 4 x 14mm ... electroplated black zinc" - "I can put inserts in
# and these will work" / "If the covers are 4 millimeters thick, we're going to have to put a pocket into the sides to
# allow for that to work as a locating pin"): a D4 x 5 shoulder, 8 of M3 thread. Through the 4 mm cover (or tab) on a
# D4.1 slip fit, the shoulder's last 1 mm in a D4.2 pocket in the frame (it locates the cover), an M3 heat-set insert
# (5.7 long) below the pocket, D3.2 on past the screw's tip
SH_D, SH_L, TH_D, TH_L, HEAD_D, HEAD_H = 4.0, 5.0, 3.0, 8.0, 6.0, 1.2
SH_CLEAR, POCKET, INS_HOLE, INS_OD, INS_L, TH_CLEAR = 4.1, 1.2, 4.2, 4.6, 5.7, 3.2
BUB_R = 4.5                                     # the bubbles and tabs: D9 round the screw
TAB_T = 4.0                                     # the tab, as thick as the cover
HOLE_DEPTH = (SH_L - TAB_T) + TH_L + 0.5        # past the frame's face: the shoulder's 1, the thread's 8, 0.5 more


def cover_screws(k):
    """one side's screws [(x, z, kind)]: 'top' (on the top plate), 'deep' / 'floor' (under the floor), 'bridge' (into the
    thrust wall's side bridges)"""
    cheek = SPEC[k]["wall_end"] < L
    xe = 156.0 if cheek else 163.0                             # A / B: the head clear of the cheek's jog (x 161)
    # A / B: a pair at the cheek's leading edge too (Mike 2026-11-20: "we're going to need at least another pair in that
    # because there's a seam there between the two halves" - joined, cover and cheek wouldn't print flat)
    seam = (130.0,) if cheek else ()
    xf = xe if cheek else 157.0                                # link C: its floor ends at 163.2 (the J4 herringbone)
    return ([(x, TOP_Z[1], "top") for x in (17.0, 62.0, 112.0) + seam + (xe,)] + [(x, C.DEEP[0], "deep") for x in (30.0, 55.0)]
            + [(x, FLOOR_Z[0], "floor") for x in (72.0, 118.0) + seam + (xf,)] + [(T0 + 44.0, z, "bridge") for z in (14.0, 37.0)])


def _both(sol):
    return sol.fuse(sol.mirror("XZ"))


def _face(kind):
    """the face the screw goes into: the frame's edge (37) or, for the bridges, the thrust wall's end (33)"""
    return WALL_Y if kind == "bridge" else WALL_Y + 4.0


def bubbles(k):
    """the frame's half-capsule bubbles, both sides: a D9 half round on the plate, round inner end"""
    out = None
    y0 = WALL_Y + 4.0 - HOLE_DEPTH - 1.5                       # 1.5 of bubble past the hole's end
    for x, z, kind in cover_screws(k):
        if kind == "bridge":
            continue
        yc = y0 + BUB_R
        # a whole sphere for the round inner end (makeSphere's default is only the upper half: the bubbles under the
        # floor came out flat-ended - Mike 2026-11-20, P454.F49: "so they have a radius ... a quarter of a sphere")
        b = cyl_y(2 * BUB_R, yc, WALL_Y + 4.0, x, z).fuse(cq.Solid.makeSphere(BUB_R, V(x, yc, z), angleDegrees1=-90))
        if kind == "top":
            keep = box(x - 10, x + 10, -60, 60, TOP_Z[0], 80)
        else:
            keep = box(x - 10, x + 10, -60, 60, -80, FLOOR_Z[1] if kind == "floor" else C.DEEP[1])
        b = b.intersect(keep)
        out = b if out is None else out.fuse(b)
    return _both(out)


def pilots(k, kinds=("top", "deep", "floor")):
    """the holes the screws go into, both sides: the shoulder's pocket and the insert (one D4.2), then D3.2"""
    out = None
    for x, z, kind in cover_screws(k):
        if kind not in kinds:
            continue
        yf = _face(kind)
        h = cyl_y(INS_HOLE, yf - POCKET - INS_L, yf + 0.1, x, z).fuse(cyl_y(TH_CLEAR, yf - HOLE_DEPTH, yf, x, z))
        out = h if out is None else out.fuse(h)
    return _both(out)


def tabs(k, side, x_lo, x_hi):
    """the cover's (or the cheek's) rounded tabs over the plate edge, one side, for the screws with x_lo <= x < x_hi;
    returns (tabs, their shoulder holes)"""
    t, holes = None, None
    y0, y1 = C.ys(side, WALL_Y + 4.0, WALL_Y + 4.0 + TAB_T)
    for x, z, kind in cover_screws(k):
        if kind == "bridge" or not x_lo <= x < x_hi:
            continue
        za, zb = (TOP_Z[0] - 4.0, z) if kind == "top" else (z, (FLOOR_Z[1] if kind == "floor" else C.DEEP[1]) + 4.0)
        tb = box(x - BUB_R, x + BUB_R, y0, y1, za, zb).fuse(cyl_y(2 * BUB_R, y0, y1, x, z))
        hl = cyl_y(SH_CLEAR, y0 - 0.1, y1 + 0.1, x, z)
        t = tb if t is None else t.fuse(tb)
        holes = hl if holes is None else holes.fuse(hl)
    return t, holes


def screws(k, side, nm0, nm):
    """the shoulder screws and their inserts, one side"""
    out = []
    for x, z, kind in cover_screws(k):
        yf = _face(kind)
        y_out = yf + 4.0                                       # under the head: the cover's (or tab's) outer face
        ys = lambda a, b: C.ys(side, a, b)                     # noqa: E731
        sh = cyl_y(HEAD_D, *ys(y_out, y_out + HEAD_H), x, z).fuse(cyl_y(SH_D, *ys(y_out - SH_L, y_out), x, z))
        sh = sh.fuse(cyl_y(TH_D, *ys(y_out - SH_L - TH_L, y_out - SH_L), x, z))
        out.append((f"M3 x 4 x 14 shoulder screw, black zinc ({nm0} {nm}, cover {kind} x {x:.0f})", one(sh.clean(), "screw"), BLACK))
        ins = ring_y(TH_D, INS_OD, *ys(yf - POCKET - INS_L, yf - POCKET), x, z)
        out.append((f"M3 heat-set insert ({nm0} {nm}, cover {kind} x {x:.0f})", ins, BRONZE))
    return out


# links B / C far idler pin (main session 2026-11-23, Mike: "the bolt that I have selected, the idler, chain idlers roll
# on. I want to turn that into a solid pin with lock collars on the end. And if there's room inside, or if there isn't,
# make holes in the side of the cover to allow for the lock collars. And we'll leave the lock collars up against the
# face"): no room inside (the 625 pair and its guide washers fill boss to boss), so a plain 5 x 100 rod (a stock length)
# and a 5 mm lock collar (D10 x 6, as link A's) on each boss's outer face, |y| 45..51, where the M5 washer and nut were.
# Link B: the J3 cheeks' boss carries it out to 45. Link C: its wall boss does, and the side covers (not drawn yet)
# need a hole over the D24 boss there.
# Then (Mike, on link A's elbow cheek's outer face: "put a hole in that face ... for that lock collar ... back deep
# enough so that we don't have the lock collar sticking out the way it is now. It's not even going to be able to clamp
# onto the rod" - link A's shaft stopped at 45, under the collar): every far idler collar (links A, B, C) sits in a
# pocket in its boss face, flush, |y| 39..45, the rod through it to the face (B / C: 90 long now); a D3 hole in from
# the joint side through the boss's rim reaches its set screw (turn the collar so the screw faces it; 1.5 hex key)
PIN_HALF, COLLAR = 45.0, (10.0, 6.0)           # half the B / C rod; the collar's OD and width
COLLAR_POCKET, COLLAR_KEY = 10.6, 3.0           # the pocket's bore (slides, turns); the set screw's access hole


def collar_pocket(x, z, side):
    """the far idler collar's pocket in a boss face at |y| 45, and the access hole to its set screw (from -x)"""
    yo = WALL_Y + 12.0
    ys = lambda a, b: C.ys(side, a, b)                                           # noqa: E731
    return cyl_y(COLLAR_POCKET, *ys(yo - COLLAR[1], yo + 1.0), x, z).fuse(
        cyl_x(COLLAR_KEY, x - 13.0, x, side * (yo - COLLAR[1] / 2), z))


# the wrap idler pin's e-clips are out (Mike 2026-11-23: "We can't use E-clips. I have no way to cut the grooves in the
# rods. Replace those E-clips with lock collars" / "Same treatment with the covers. Put a hole in there for the lock
# collar"): the same 5 mm lock collar (D10 x 6) sunk flush in each wall's D20 boss, |y| 34..40, the pin 80 long (a stock
# length) out to the collar faces. Nothing may stand past 40: the parent's cheek (or the base boss) is outside it. Its
# set screw's D3 hole comes in from +x, away from the 6808's seat. The side covers (not drawn yet) need a hole over the
# D20 boss. The pin now bears on 2.5 mm of plastic each side (the D7 boss + 1 of wall) instead of 8.5: ~283 N per side
# at J1 stall (575 N chain, 59 deg of wrap) = ~23 MPa, ~13 MPa holding.
WRAP_YO = WALL_Y + 7.0                          # the wrap idler boss's outer face (40)


def wrap_collar_pocket(x, z, side):
    """the wrap idler collar's pocket in the wall's D20 boss, flush at |y| 40, and the access hole to its set screw"""
    ys = lambda a, b: C.ys(side, a, b)                                           # noqa: E731
    return cyl_y(COLLAR_POCKET, *ys(WRAP_YO - COLLAR[1], WRAP_YO + 1.0), x, z).fuse(
        cyl_x(COLLAR_KEY, x, x + 13.0, side * (WRAP_YO - COLLAR[1] / 2), z))


def hex_y(af, y0, y1, x, z):
    """a hex prism along y from y0 to y1 (a nut), af across the flats, flats top and bottom"""
    r = af / math.cos(math.pi / 6) / 2                                          # the corners' radius
    wire = cq.Wire.makePolygon([V(x + r * math.cos(math.radians(60 * k)), y0, z + r * math.sin(math.radians(60 * k)))
                                for k in range(6)], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(0, y1 - y0, 0))


FREE_SHOULDER = 1.5                             # the free-end bearing's shoulder (Mike: "We only need a shoulder with 1.5")
FW_BUB_R = 5.5                                  # links B / C: the free-end wall's bubble over the far idler shaft


GB, GB_CLEAR, GB_SLOT, GB_Y, GB_DX = (3.0, 6.0, 2.0), 0.5, 2.6, 22.0, 0.0   # his 3x6x2 guide bearings (Mike 2026-11-20:
#   "I have them ... I think I've got 10 of them, maybe 20"): inside the carrier's 10 thickness, 0.5 under the plate, in a
#   2.6 slot at |y| 22 (2.2 of aluminium outside it), on an M3 x 10 tapped into the carrier

# the chain tensioner (main session 2026-11-23, Mike: "I can get a piece of angle iron ... drill a hole in it ... mount
# it on the sides of the nut carrier. And then we can put our eye bolt through it" / "move that back, whatever the face
# of our angle iron is, up to the center line of our hole" / "Go ahead and build it. If it doesn't work, we'll revert
# it"; his eye bolt, M5 x 25 forged, 15 of thread, a 4 hole in its eye). Each side: a 1" x 1" x 1/8" steel angle, its
# drilled leg standing in front of the nut flange, tapped M5 on the chain line; its flat leg back along the carrier's
# side, which is recessed from 25.5 to CARRIER_RECESS at chain height for it, and 2 x M4 through it into the carrier.
# The eye bolt screws into the leg from the joint side with a jam nut on the eye side (behind the leg there is 3 mm to
# the flat leg; an M5 nylock needs 4.6): 7.8 of adjustment. The chain's end goes in its eye through a #25 connecting
# link, so the eye must be <= 4.5 thick: its OD and thickness here are ASSUMED (measure his). At the end of travel the
# eye sits in the thrust wall's chain window, opened to TW_WINDOW for it. The far chain end stays pinned through a
# steel plate in the chain plane where it was (link A's far idler leaves no room further out), on the same 2 x M4 over
# a 1.5 spacer
TEN_T, TEN_W, TEN_L, TEN_OUT = 3.2, 25.4, 20.0, 31.5   # angle: thickness, flat leg along x, cut length (z), drilled leg's edge |y|
CARRIER_RECESS = 20.8                                  # the carrier's side at chain height (was 25.5; the flange is D40)
EYE = dict(thread=5.0, overall=25.0, od=10.0, hole=4.0, t=4.5)   # his M5 x 25 eye bolt (od, t ASSUMED)
JAM = (8.0, 4.0)                                       # M5 jam nut: across flats, thickness
TAB_BOLT_DZ = 5.5                                      # the 2 x M4, above and below the chain line
TW_WINDOW = (19.0, 31.5)                               # the thrust wall's chain window, between its bridges


# the ball screw support plates' ribs (main session 2026-11-23, Mike with a sketch: "every one of the ball screw support
# plates, I want a rib on each side to support it better. It can be beveled on the outside and would look better" /
# "Modify the ribs so that they clear the nut. Just put a little cutout on them so that they still have the support on
# the sides, but the nut can't hit them"): a 45 deg gusset at each corner of both faces of the thrust wall and the
# free-end wall, top ones from the top plate, bottom ones from the floor, part of the frame. The thrust wall is its own
# part (printed flat): its ribs stand FIT off it and hold it front and back - it still slides in sideways between them.
# Then (Mike: "gussets closer to the outside at the top, rather than gouge out a part of the gusset. Just move the
# gussets out some") and now (Mike: "Make them smaller, run them all the way across. So they're flush with the outside
# of that block. Make sure to leave room for the ball screw nut. These can be considerably smaller. 4mm Tall"): a 4 x 4
# 45 deg strip right across each face, top and bottom, its ends flush with the plate's sides - the thrust wall |y| 21
# (its foot, under the bottom ones, 20), the free-end wall 20. 4 out from the face, the thrust wall's nut-side strips
# stop 1.4 short of the nut flange at the end of its travel; the nut's swept envelope (+1) is still cut out as a guard
RIB_L = 4.0                                      # 45 deg, 4 x 4
RIB_W_TW, RIB_W_FOOT, RIB_W_FW = 21.0, IN_Y, IN_Y   # half widths: thrust wall, its foot, free-end wall


# the thrust wall's top screws (main session 2026-11-23, Mike on link B's thrust wall: "put some screws in the top of
# that to hold it inside of the frame. Maybe a couple on the top, just inside the gussets. Use the number 4 by 3/4 wood
# screw"): two #4 x 3/4" FLAT heads down through the top plate into the wall's top edge, in its middle (x T0 + 44) at
# |y| 18.5, just inside the motor plate's gussets (|y| 22..26; the countersink's rim 0.45 short of them). Flat heads
# sunk 0.2 under flush, because the motor's underside is only 1.2 over the top plate (link A's 60 mm: 0.3): heads
# 1.4 under the motor (A: 0.5). The tips reach 15 into the wall: 4.4 of plastic to the 6000's pocket, 3.6 to the side
# bridges' cover screws. In after the wall slides home, before the motor goes on
TWS_X, TWS_Y = 44.0, 18.5                       # x = T0 + TWS_X (the wall's middle), |y|
WS4 = dict(d=2.84, head=5.5, length=19.05, clear=3.2, pilot=2.4, sink=0.2)   # #4 x 3/4" flat head wood screw


def cyl_z(d, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def tw_screw_holes(t0, kind):
    """'plate': the top plate's clearance holes and 90 deg countersinks; 'wall': the thrust wall's pilots"""
    out, top = None, TOP_Z[1]
    for s in (1, -1):
        x, y = t0 + TWS_X, s * TWS_Y
        if kind == "plate":
            r0, r1 = WS4["clear"] / 2, WS4["head"] / 2 + WS4["sink"] + 0.1          # 0.1 round the sunk head
            h = cyl_z(WS4["clear"], TOP_Z[0] - 0.1, top + 0.1, x, y)
            h = h.fuse(cq.Solid.makeCone(r0, r1, r1 - r0, V(x, y, top - (r1 - r0)), V(0, 0, 1)))
            h = h.fuse(cyl_z(2 * r1, top - 0.001, top + 1.0, x, y))
        else:
            h = cyl_z(WS4["pilot"], top - WS4["sink"] - WS4["length"] - 0.5, TOP_Z[0] + 0.1, x, y)
        out = h if out is None else out.fuse(h)
    return out


def tw_screws(t0, nm0):
    out, top = [], TOP_Z[1] - WS4["sink"]
    r, hr = WS4["d"] / 2, WS4["head"] / 2
    for s, nm in ((1, "+y"), (-1, "-y")):
        x, y = t0 + TWS_X, s * TWS_Y
        sc = cyl_z(WS4["d"], top - WS4["length"], top - (hr - r) + 0.01, x, y)
        sc = sc.fuse(cq.Solid.makeCone(r, hr, hr - r, V(x, y, top - (hr - r)), V(0, 0, 1)))
        out.append((f'#4 x 3/4" flat head wood screw ({nm0} thrust wall {nm}, down through the top plate)',
                    one(sc.clean(), "screw"), STEEL))
    return out


def nut_envelope(g):
    """the ball nut, its flange, the carrier and the guide bearings / axle heads, swept over the whole travel, + 1"""
    f0, f1, zt = g["f_min"], g["f_max"], C.NUT_FL_FLATS / 2 + 1.0
    fl = cyl_x(C.NUT_FL_D + 2.0, f0 - 1.0, f1 + 11.0, 0, Z_S).intersect(box(f0 - 2, f1 + 12, -40, 40, Z_S - zt, Z_S + zt))
    env = fl.fuse(cyl_x(C.NUT_BODY_D + 2.0, f0 + 9.0, f1 + 36.0, 0, Z_S))
    env = env.fuse(box(f0 + 9.0, f1 + 21.0, -26.5, 26.5, Z_S - 17.0, Z_S + 17.0))              # the carrier
    gz = TOP_Z[0] - GB_CLEAR - GB[1] / 2
    return env.fuse(box(f0 + 15.0 + GB_DX - 5.0, f1 + 15.0 + GB_DX + 5.0, -29.5, 29.5, gz - 4.0, TOP_Z[0]))


def support_ribs(g, fx, floor_end, top_end):
    """the eight strips (both faces of both plates, top and bottom), cut clear of the nut (a guard: see RIB_L)"""
    tw0, tw1, foot = g["t0"] + 38.0, g["t0"] + 50.0, 30.0       # the thrust wall's faces; its foot's joint-side face
    tz, fz, dz = TOP_Z[0], FLOOR_Z[1], C.DEEP[1]
    e = 0.01                                                    # into the plate / wall it grows from
    tw, ft, fw = RIB_W_TW, RIB_W_FOOT, RIB_W_FW
    #        x at the face, which way, z it hangs from / stands on, up, x kept from .. to, half width
    spec = [(tw0 - FIT, -1, tz, -1, -1e9, 1e9, tw), (foot - FIT, -1, dz, 1, 24.0, 1e9, ft),        # thrust wall, joint side
            (tw1 + FIT, 1, tz, -1, -1e9, 1e9, tw), (tw1 + FIT, 1, dz, 1, -1e9, 1e9, ft),          # thrust wall, nut side
            (fx[0] + e, -1, tz, -1, -1e9, 1e9, fw), (fx[0] + e, -1, fz, 1, -1e9, 1e9, fw),        # free-end wall, nut side
            (fx[1] - e, 1, tz, -1, -1e9, top_end, fw), (fx[1] - e, 1, fz, 1, -1e9, floor_end, fw)]  # free-end wall, far
    out = None
    for x, d, z, up, lo, hi, hw in spec:
        ze = z - up * e
        pts = [(x, ze), (x + d * RIB_L, ze), (x, ze + up * RIB_L)]
        r = tri_prism(pts, -hw, hw).intersect(box(lo, hi, -60, 60, -60, 80))
        out = r if out is None else out.fuse(r)
    return out.cut(nut_envelope(g))


def build_link(key, nut_at=0.5):
    """one link in its own frame (inner joint at x 0). Returns (link parts, the inner joint's sprockets + bolts, info)"""
    sp = SPEC[key]
    g = geometry(sp, nut_at)
    s, x_i, f, mface, t0, f_max = g["s"], g["x_i"], g["f"], g["motor_face"], g["t0"], g["f_max"]
    nm0 = sp["name"]
    # links B / C: both far idlers on ONE 5 mm steel pin straight through, clear of the 608 by 4.2 (main session
    # 2026-11-21, Mike: "replace those two bolts with a pin that goes through, all the way through"); since 2026-11-23 a
    # plain rod with a lock collar each end instead of M5 threads and nuts (PIN_HALF); link A keeps its two stubs (a pin
    # through would cut its 6801)
    pin = sp["free"] == "608"
    mv, joint = [], []
    add = lambda lst, name, sol, c: lst.append((name, sol, c))                  # noqa: E731
    xb = f + 23.5                                                              # far chain end pin, past the carrier
    pts = C.walk(path(f - 12.0, xb, x_i))
    eye = pts[0][0]
    ph1 = C.phase(pts, (0.0, 0.0), C.R1)
    bolts = [(16 * math.cos(ph1 + k * math.pi / 2), 16 * math.sin(ph1 + k * math.pi / 2)) for k in range(4)]  # hole 1 on a gap
    zi, cw = C.Z_I, C.CW

    for side, nm in ((1, "+y"), (-1, "-y")):
        ys = lambda a, b: C.ys(side, a, b)                                      # noqa: E731
        add(joint, f"#25 25T sprocket {nm} ({nm0}'s joint; his CNC-cut plate, 4 x M3 on D32)",
            C.sprocket(25, 0, 0, ph1, 20.0, side, bolts=bolts), DARK)
        add(joint, f"sprocket screws M3 {nm} ({nm0}'s joint)",
            cq.Compound.makeCompound([cyl_y(3.0, *ys(YC - 1.5, WALL_Y + 10), bx, bz) for bx, bz in bolts]), BLACK)
        y0, y1 = ys(WALL_Y, WALL_Y + 4)
        r0, r1 = ys(WALL_Y, WALL_Y + 7)
        # the wall stands ON the full-width floor from x 64 out (Mike 2026-11-20: why isn't the floor "flush with the
        # outside edge" like the top plate); by the joint it still reaches down past the deep floor, round the hub
        w = box(64, sp["wall_end"], y0, y1, FLOOR_Z[1], TOP_Z[0]).fuse(box(0, 64, y0, y1, C.WALL_DEEP, TOP_Z[0]))
        w = w.fuse(cyl_y(64.0, r0, r1, 0, 0))
        # ... and from x 24 out it sits ON the deep floor too, now full width (Mike 2026-11-20, P492.F19: "Why is that
        # set back?" - "Yes, make it flush"); the hub ring loses a 0.9 mm sliver at x 24
        w = w.cut(box(24, 64, y0, y1, C.DEEP[0], C.DEEP[1])).cut(box(60, 64, y0, y1, C.DEEP[0], FLOOR_Z[1]))
        if sp["wall_end"] >= L and side == 1:
            w = wrist_wall(w)                                                   # the J4 herringbone's motor side
        elif sp["wall_end"] >= L:
            w = w.fuse(cyl_y(64.0, r0, r1, L, 0)).cut(cyl_y(52.0 + FIT, -60, 60, L, 0))     # output side: the claw's way
        w = w.fuse(cyl_y(20.0, r0, r1, *cw)).fuse(cyl_y(24.0, *ys(WALL_Y, WALL_Y + 12), x_i, zi))
        w = w.fuse(cyl_y(BOSS_D, *ys(B_OUT, WALL_Y + 0.01), *cw)).fuse(cyl_y(BOSS_D, *ys(B_OUT, WALL_Y + 0.01), x_i, zi))  # idler bosses, inside
        win = box(66, min(x_i - 14, 120), y0 - 1, y1 + 1, 2, 38)
        w = w.cut(win.fillet(8, win.edges("|Y")))
        # the joint's 6808 seat, with a lip on the inside so the outer ring can't slide in (arm session 2026-11-20: "Your
        # walls' D52 bearing holes have no shoulder ... Needs a lip on the inner side"); the lip stays clear of the chain
        w = w.fuse(ring_y(48.0, 54.0, *ys(WALL_Y - 1.0, WALL_Y), 0, 0))
        w = w.cut(cyl_y(52.0 + FIT, *ys(WALL_Y, WALL_Y + 8.0), 0, 0)).cut(cyl_y(48.0, -60, 60, 0, 0))
        w = w.cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, *cw)).cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, x_i, zi))
        if sp["wall_end"] >= L:                                                 # link C: the boss is the wall's
            w = w.cut(collar_pocket(x_i, zi, side))                             # (A / B: in the cheeks)
        w = w.cut(wrap_collar_pocket(*cw, side))                                # the wrap idler's collar, flush
        if sp["wall_end"] < L:                                                  # the next joint's cheeks go on outside
            w = w.cut(box(100, 260, *ys(WALL_Y + 4, 60), -60, 80))
        tb, th = tabs(key, side, 0.0, CHEEK_X0 - BUB_R if sp["wall_end"] < L else 1e9)   # under a cheek: its tabs
        if tb is not None:
            w = w.fuse(tb).cut(th)
        for x, z, kind in cover_screws(key):
            if kind == "bridge":
                w = w.cut(cyl_y(SH_CLEAR, *ys(WALL_Y - 0.1, WALL_Y + 4.1), x, z))
        add(mv, f"{nm0} wall {nm} (printed)", one(w.clean(), "wall"), PRINT)
        for n_, s_, c_ in screws(key, side, nm0, nm):
            add(mv, n_, s_, c_)
        add(mv, f"6808 bearing ({nm0} inner joint, {nm})", ring_y(40.0, 52.0, r0, r1), STEEL)
        for n_, s_, c_ in idler_stack(cw[0], cw[1], side, f"{nm0} wrap idler {nm}"):   # the thrust wall's foot
            add(mv, n_, s_, c_)                                                                  # locates them now
        add(mv, f"5 mm lock collar ({nm0} wrap idler pin, {nm}; set screw; in its pocket in the wall's boss, flush)",
            ring_y(C.SHAFT_D, COLLAR[0], *ys(WRAP_YO - COLLAR[1], WRAP_YO), *cw), STEEL)
        for n_, s_, c_ in idler_stack(x_i, zi, side, f"{nm0} far idler {nm}"):
            add(mv, n_, s_, c_)
        if pin:                                      # one plain pin through both far idlers, a lock collar each end
            yo = WALL_Y + 12.0                                                  # the boss's outer face (45)
            if side == 1:
                add(mv, f"far idler pin 5 mm ({nm0}, plain steel rod {2 * PIN_HALF:.0f} long: collar, boss, wall, idlers, "
                    f"free-end wall, idlers, wall, boss, collar)", cyl_y(C.SHAFT_D, -PIN_HALF, PIN_HALF, x_i, zi), STEEL)
            add(mv, f"5 mm lock collar ({nm0} far idler pin, {nm}; set screw; in its pocket, flush with the boss face)",
                ring_y(C.SHAFT_D, COLLAR[0], *ys(yo - COLLAR[1], yo), x_i, zi), STEEL)
        else:
            add(mv, f"far idler shaft 5 mm ({nm0} {nm}, free-end wall to link wall)", cyl_y(C.SHAFT_D, *ys(10.0, WALL_Y + 12.0), x_i, zi), STEEL)
            add(mv, f"5 mm lock collar ({nm0} far idler {nm}; set screw; in its pocket in the cheek, flush)",
                ring_y(C.SHAFT_D, COLLAR[0], *ys(WALL_Y + 12.0 - COLLAR[1], WALL_Y + 12.0), x_i, zi), STEEL)
        add(mv, f"#25 roller chain {nm} ({nm0}, {len(pts)} pitches)", chain(pts).translate(V(0, side * YC, 0)), DARK)
        # the far chain end's anchor plate and the tensioner (TEN_*, EYE: Mike's angle iron and eye bolt)
        holes = [cyl_y(4.4, -60, 60, f + 15.0, Z_S + dz) for dz in (-TAB_BOLT_DZ, TAB_BOLT_DZ)]
        tab = box(f + 10.0, xb + 2.5, *ys(YC - 1.5, YC + 1.5), Z_S - 4, Z_S + 4)
        tab = tab.fuse(box(f + 10.0, f + 20.0, *ys(YC - 1.5, YC + 1.5), Z_S - 8, Z_S + 8)).cut(cyl_y(2.4, -60, 60, xb, Z_S))
        sp_ = box(f + 10.0, f + 20.0, *ys(CARRIER_RECESS + TEN_T, YC - 1.5), Z_S - 8, Z_S + 8)
        x0, zt = f - 4.0, TEN_L / 2                                          # the drilled leg's joint-side face
        ang = box(x0, x0 + TEN_T, *ys(CARRIER_RECESS, TEN_OUT), Z_S - zt, Z_S + zt)
        ang = ang.fuse(box(x0, x0 + TEN_W, *ys(CARRIER_RECESS, CARRIER_RECESS + TEN_T), Z_S - zt, Z_S + zt))
        ang = ang.cut(cyl_x(EYE["thread"] - 0.8, x0 - 1, x0 + TEN_T + 1, side * YC, Z_S))   # tapped M5 (4.2 drill)
        for h in holes:
            tab, sp_, ang = tab.cut(h), sp_.cut(h), ang.cut(h)
        add(mv, f"chain tab {nm} ({nm0}, steel 3 thick: the far chain end's anchor, pinned through it; 2 x M4 through "
            f"it, a spacer and the tensioner angle into the carrier)", one(tab.clean(), "tab"), DARK)
        add(mv, f"chain tab spacer {nm} ({nm0}, steel {YC - 1.5 - CARRIER_RECESS - TEN_T:.1f}: the anchor plate on the "
            f"chain line)", one(sp_.clean(), "spacer"), STEEL)
        add(mv, f"tensioner angle {nm} ({nm0}; 1 x 1 x 1/8 in steel angle, {TEN_L:.0f} long, drilled leg cut to "
            f"{TEN_OUT - CARRIER_RECESS:.1f}: tapped M5 on the chain line; flat leg on the carrier's recessed side)",
            one(ang.clean(), "angle"), DARK)
        yo = YC + 1.5                                                        # the M4s: button heads on the anchor plate
        for k, dz in enumerate((-TAB_BOLT_DZ, TAB_BOLT_DZ)):
            sc_ = cyl_y(4.0, *ys(CARRIER_RECESS - 8.0, yo), f + 15.0, Z_S + dz).fuse(cyl_y(7.6, *ys(yo, yo + 2.2), f + 15.0, Z_S + dz))
            add(mv, f"M4 x 16 button head screw {k + 1}/2 ({nm0} chain tab {nm}: plate, spacer, angle, tapped in the "
                f"carrier)", one(sc_.clean(), "screw"), BLACK)
        eb = ring_y(EYE["hole"], EYE["od"], *ys(YC - EYE["t"] / 2, YC + EYE["t"] / 2), eye, Z_S)
        eb = eb.fuse(cyl_x(EYE["thread"], eye + EYE["od"] / 2 - 0.5, eye - EYE["od"] / 2 + EYE["overall"], side * YC, Z_S))
        add(mv, f"M5 x 25 eye bolt {nm} ({nm0}; forged, 4 hole; the chain's end in its eye through a #25 connecting "
            f"link; screwed into the tensioner angle)", one(eb.clean(), "eye bolt"), STEEL)
        add(mv, f"M5 jam nut {nm} ({nm0} eye bolt, locks it against the tensioner angle)",
            ring_x(EYE["thread"], JAM[0] / math.cos(math.pi / 6), x0 - JAM[1], x0, side * YC, Z_S), STEEL)

    # --- printed structure
    floor_end = FLOOR_END if sp["wall_end"] < L else WR_FLOOR_END             # link C: clear of the J4 herringbone
    floor = box(64, floor_end, -(WALL_Y + 4), WALL_Y + 4, *FLOOR_Z).fuse(box(24, 64, -(WALL_Y + 4), WALL_Y + 4, *C.DEEP))
    floor = floor.fuse(box(60, 64, -(WALL_Y + 4), WALL_Y + 4, C.DEEP[0], FLOOR_Z[1]))   # flush outside its whole length
    mt, blen, bc, Z_M = drive(sp)
    top_end = 165.0 if sp["wall_end"] < L else FLOOR_END
    top = box(t0 + 16.0, top_end, -(WALL_Y + 4), WALL_Y + 4, *TOP_Z)              # open over the pulleys and belt
    # no opening under the motor (Mike 2026-11-20: "Remove the holes. We'll just slide the motor into the hub center bore
    # and then tighten the set screw" - the belt goes on both pulleys first, the motor shaft slides into its pulley)
    # no overhang over the next joint's cheeks any more: the cheeks come up flush with the top instead, and the frame
    # stays |y| <= 37 so it lies flat on its side (Mike 2026-11-20: "this looks goofy the way it is")
    # the motor plate: 6 thick, 5 mm wider than the motor each side, a gusset each side running back along the top plate
    # over the thrust wall (Mike 2026-11-20: "extend that over a little bit and put a couple of gussets on there on each
    # one of those motor plates so it doesn't fold or bend")
    mp = box(mface - 6.0, mface, -26.0, 26.0, TOP_Z[0], Z_M + 21.15 + SLOT + 2.0)   # the motor face bolts to its +x side
    for side in (1, -1):
        mp = mp.fuse(tri_prism([(mface - 0.01, TOP_Z[1] - 0.01), (mface - 0.01, TOP_Z[1] + 34.0), (mface + 30.0, TOP_Z[1] - 0.01)],
                               *C.ys(side, 22.0, 26.0)))
    boss = cyl_x(22.5, mface - 7, mface + 1, 0, Z_M - SLOT_DN).fuse(cyl_x(22.5, mface - 7, mface + 1, 0, Z_M + SLOT))
    mp = mp.cut(boss.fuse(box(mface - 7, mface + 1, -11.25, 11.25, Z_M - SLOT_DN, Z_M + SLOT)))    # the motor boss slides too
    for sy in (-15.5, 15.5):
        for sz in (-15.5, 15.5):
            mp = mp.cut(box(mface - 7, mface + 1, sy - 1.7, sy + 1.7, Z_M + sz - 1.7 - SLOT_DN, Z_M + sz + 1.7 + SLOT))  # M3 + washer
    # the free-end wall (Mike 2026-11-20: "make that plate that the ball screw goes into thicker. You're not moving the
    # front of it, you're moving the back of it. Then ... move the bolt locations for the idler back by one link length"
    # / "a pocket for the bearing ... the through hole needs to be fairly sizable. We only need a shoulder with 1.5"):
    # its joint-side face stays, it reaches back past the far idler shafts (now one pitch further out); the bearing
    # goes in from the front on the screw into a pocket, a 1.5 shoulder behind it, then a big hole on through (the
    # 608's snap ring is fitted through it from the back)
    fx1_min = lambda f1: min(max(f1, x_i + 5.5), top_end)            # noqa: E731
    if sp["free"] == "6801":
        # link A: one link back put the idler and its chain wrap into link B at the elbow, so its idler stays and the
        # plate grows forward instead (its 6801 is on the thread, and the carrier stops 12 short of the plate)
        fx = (min(f_max + 36.5, x_i - 5.5), fx1_min(f_max + 42.5))
        b0, b1 = f_max + 37.0, f_max + 42.0                           # the 6801, on the thread
        end = f_max + 43.0
        fw = box(*fx, -IN_Y, IN_Y, FLOOR_Z[1], TOP_Z[0]).cut(cyl_x(21.0 + FIT, fx[0] - 1, b1, 0, Z_S))
        fw = fw.cut(cyl_x(21.0 - 2 * FREE_SHOULDER, b1 - 0.01, fx[1] + 1, 0, Z_S))
        # the ball nut's D24 body reaches f_max + 35 at full travel: a D25 relief in the plate's front for it
        fw = fw.cut(cyl_x(C.NUT_BODY_D + 1.0, fx[0] - 1, f_max + 36.0, 0, Z_S))
        add(mv, f"6801 bearing 12x21x5 ({nm0} free end, on the thread)", ring_x(12.0, 21.0, b0, b1, 0, Z_S), STEEL)
    else:
        te = t0 + 140.0                                                        # the 150's thread end, then its D8 x 10
        end = te + 10.0
        # the wall's back face stays where it was (5.5 past the shaft's old place); the shaft one pitch further out gets
        # a half-round bubble over it instead, the same 5.5 round it, the wall's width (main session 2026-11-21, Mike:
        # "it's pulling the other direction. So that side doesn't need to be that thick ... just put a bubble, you
        # know, half capsule over it if it would fit and just make it consistent")
        fx = (te - 0.5, min(max(te + 7.5, x_i - D.P + 5.5), top_end))
        fw = box(*fx, -IN_Y, IN_Y, FLOOR_Z[1], TOP_Z[0]).fuse(cyl_y(2 * FW_BUB_R, -IN_Y, IN_Y, x_i, zi))
        fw = fw.cut(cyl_x(22.0 + FIT, fx[0] - 1, te + 7, 0, Z_S))
        fw = fw.cut(cyl_x(22.0 - 2 * FREE_SHOULDER, te + 7 - 0.01, max(fx[1], x_i + FW_BUB_R) + 1, 0, Z_S))
        add(mv, f"608 bearing 8x22x7 ({nm0} free end, on the machined D8 end)", ring_x(8.0, 22.0, te, te + 7, 0, Z_S), STEEL)
        add(mv, f"snap ring 8 mm ({nm0} free end, in the machined groove)", ring_x(8.0, 12.0, te + 7.9, te + 8.8, 0, Z_S), STEEL)
    fw = fw.fuse(bosses(x_i, zi, IN_Y - 0.01, B_IN))                         # the far idlers' inner bosses
    fw = fw.cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, x_i, zi))                     # the far idler shafts' inner ends
    # one printed piece, printed on its side (Mike 2026-11-20): floor + top plate + motor plate and gussets + free-end
    # wall; the side walls and the thrust wall bolt to it
    add(mv, f"{nm0} frame (printed on its side, one piece): floor, top plate, motor mount and gussets, free-end wall",
        one(floor.fuse(top).fuse(mp).fuse(fw).fuse(support_ribs(g, fx, floor_end, top_end)).fuse(bubbles(key))
            .cut(pilots(key)).cut(tw_screw_holes(t0, "plate")).clean(), "frame"), PRINT)
    tw = box(t0 + 38, t0 + 50, -21, 21, 3.0, TOP_Z[0])
    # its foot down to the deep floor, between the two wrap idlers, the wrap idler pin straight through it (Mike
    # 2026-11-20: "Can we extend that down to ... and then just put a pin all the way through that?"); widened toward the
    # joint so the pin has solid plastic round it
    tw = tw.fuse(box(30.0, t0 + 50, -IN_Y, IN_Y, C.DEEP[1], 3.01)).fuse(bosses(*cw, IN_Y - 0.01, B_IN))   # + the wrap idlers' inner bosses
    tw = tw.cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, *cw))
    for side in (1, -1):
        # bridges over the chain's top run and between its two runs; the window between them 19..31.5 since 2026-11-23
        # (was 20..30) so the tensioner's eye bolt (D10 eye) passes at the end of travel
        tw = tw.fuse(box(t0 + 38, t0 + 50, *C.ys(side, 21, WALL_Y), TW_WINDOW[1], TOP_Z[0]))
        tw = tw.fuse(box(t0 + 38, t0 + 50, *C.ys(side, 21, WALL_Y), 8.0, TW_WINDOW[0]))
    # the 6000's pocket opens on the joint-side face so it presses in (it was closed both sides); a shim takes the
    # inboard thrust bearing's push on the 6000's outer ring only
    tw = tw.cut(cyl_x(14.0, t0 + 37, t0 + 51, 0, Z_S)).cut(cyl_x(26.0 + FIT, t0 + 37, t0 + 47, 0, Z_S))
    tw = tw.cut(pilots(key, ("bridge",))).cut(tw_screw_holes(t0, "wall"))   # + the two top screws' pilots
    add(mv, f"{nm0} thrust wall (printed flat, bolted in): a 6000, an AXK1024 each face; its foot carries the wrap idler pin",
        one(tw.clean(), "thrust wall"), PRINT)
    for n_, s_, c_ in tw_screws(t0, nm0):
        add(mv, n_, s_, c_)
    add(mv, f"wrap idler pin 5 mm ({nm0}, plain hardened rod {2 * WRAP_YO:.0f} long: collar, wall, idler, thrust wall "
        f"foot, idler, wall, collar)", cyl_y(C.SHAFT_D, -WRAP_YO, WRAP_YO, *cw), STEEL)

    # --- the screw and its machined drive end
    sc = cyl_x(8.0, t0, t0 + 15, 0, Z_S).fuse(cyl_x(10.0, t0 + 15, s, 0, Z_S))
    if sp["free"] == "608":
        sc = sc.fuse(cyl_x(12.0, s, end - 10.0, 0, Z_S)).fuse(cyl_x(8.0, end - 10.0, end, 0, Z_S))
    else:
        sc = sc.fuse(cyl_x(12.0, s, end, 0, Z_S))
    sc = groove(sc, s + 1.0, (end if sp["free"] == "6801" else end - 10.0) - 1.0)
    add(mv, f"ball screw SFU1204 ({nm0}: {sp['screw']}; D10 seat, M10x1, D8 stub as machined)", one(sc.clean(), "screw"), STEEL)
    for i, (a, b) in enumerate(((t0 + 34, t0 + 38), (t0 + 50, t0 + 54))):
        add(mv, f"AXK1024 thrust bearing + 2 AS1024 washers {i + 1}/2 ({nm0})", ring_x(10.0, 24.0, a, b, 0, Z_S), STEEL)
    add(mv, f"6000 bearing 10x26x8 ({nm0} thrust wall, pressed in from the joint side)", ring_x(10.0, 26.0, t0 + 39, t0 + 47, 0, Z_S), STEEL)
    add(mv, f"shim washer 16 x 26 x 1 ({nm0}, steel: the inboard thrust bearing bears on the 6000's outer ring)",
        ring_x(16.0, 26.0, t0 + 38, t0 + 39, 0, Z_S), STEEL)
    add(mv, f"spacer 10 x 16 x 3 ({nm0})", ring_x(10.0, 16.0, t0 + 31, t0 + 34, 0, Z_S), STEEL)
    add(mv, f"M10x1 locknut ({nm0}, clamps the bearing stack)", ring_x(10.0, 19.0, t0 + 23, t0 + 31, 0, Z_S), STEEL)
    g60s = dict(n=60, bore=8.0, teeth=(t0, t0 + 7), fl_d=42.0, hub=(t0 + 8, t0 + 15), hub_d=25.0)
    rm = mt * 2.0 / math.pi / 2
    gm = dict(g60s, n=mt, bore=5.0, fl_d=42.0 if mt == 60 else 2 * rm + 4.0, hub_d=25.0 if mt == 60 else 16.0)
    add(mv, f"GT2 60T pulley, 8 mm bore ({nm0} screw; his, file a flat for the set screw)", pulley(g60s, 0, Z_S), ALU)
    add(mv, f"GT2 {mt}T pulley, 5 mm bore ({nm0} motor)", pulley(gm, 0, Z_M), ALU)
    bx = t0 + 3.5
    to_yz = lambda u, v: V(bx - D.BELT_W / 2, u, v)                              # noqa: E731
    outer = hull_solid((0, Z_S), D.R60 - 0.75 + D.BELT_T, (0, Z_M), rm - 0.75 + D.BELT_T, to_yz, V(D.BELT_W, 0, 0))
    inner = hull_solid((0, Z_S), D.R60 - 0.75, (0, Z_M), rm - 0.75, lambda u, v: V(bx - 5, u, v), V(10, 0, 0))
    add(mv, f"GT2 belt {blen:.0f} x 6 mm ({nm0}, {60 / mt:.3g}:1, C {bc:.1f})", one(outer.cut(inner).clean(), "belt"), BLACK)
    body, caps = nema17.load(sp["motor_l"])
    caps = nema17.trimmed_caps(caps, 24.0)
    place = lambda sol: sol.rotate(V(0, 0, 0), V(0, 1, 0), -90).translate(V(mface, 0, Z_M))   # noqa: E731
    add(mv, f"NEMA 17 {sp['motor_l']:.0f} mm ({nm0}, on top by its own joint)", place(body), BLACK)
    add(mv, f"NEMA 17 end caps ({nm0})", place(caps), STEEL)

    # --- nut and carrier
    nut = cyl_x(C.NUT_BODY_D, f + 10, f + 35, 0, Z_S)
    fl = cyl_x(C.NUT_FL_D, f, f + 10, 0, Z_S).cut(box(f - 1, f + 11, -30, 30, Z_S + C.NUT_FL_FLATS / 2, Z_S + 30))
    fl = fl.cut(box(f - 1, f + 11, -30, 30, Z_S - 30, Z_S - C.NUT_FL_FLATS / 2))
    add(mv, f"ball nut SFU1204 ({nm0}, flange inboard)", one(nut.fuse(fl).cut(cyl_x(12.0, 0, 300, 0, Z_S)).clean(), "nut"), STEEL)
    cp = box(f + 10, f + 20, -25.5, 25.5, Z_S - 16, Z_S + 16).cut(cyl_x(25.0, f + 9, f + 21, 0, Z_S))
    for side in (1, -1):                         # the tensioner angle's recess at chain height, its 2 x M4 tapped
        cp = cp.cut(box(f + 9, f + 21, *C.ys(side, CARRIER_RECESS, 30.0), Z_S - TEN_L / 2 - 0.5, Z_S + TEN_L / 2 + 0.5))
        for dz in (-TAB_BOLT_DZ, TAB_BOLT_DZ):
            cp = cp.cut(cyl_y(3.3, *C.ys(side, CARRIER_RECESS - 8.5, CARRIER_RECESS + 1.0), f + 15.0, Z_S + dz))
    # guide bearings (Mike 2026-11-20: "cut a slot in both sides ... put a screw through there ... two bearings on the
    # top of that surface that ride on the inside of that upper wall ... 0.5mm should be plenty" - to take the ball
    # nut's turning force off the chains, "how that could cause error in positioning"): a slot down each side of the
    # carrier's top, his 3x6x2 in it on an M3 x 10 across it (its head clears the chain tab), its top 0.5 under the top
    # plate. Whichever side tries to rise rolls on the plate; ~1.3 deg of play = 0.014 mm of nut travel
    gx, gz = f + 15.0 + GB_DX, TOP_Z[0] - GB_CLEAR - GB[1] / 2
    for side, nm in ((1, "+y"), (-1, "-y")):
        ys = lambda a, b: C.ys(side, a, b)                                      # noqa: E731
        cp = cp.cut(box(f + 9, f + 21, *ys(GB_Y - GB_SLOT / 2, GB_Y + GB_SLOT / 2), gz - GB[1] / 2 - 0.5, 80))
        cp = cp.cut(cyl_y(3.4, *ys(GB_Y, 26.0), gx, gz)).cut(cyl_y(2.5, *ys(25.5 - 10.5, GB_Y), gx, gz))   # M3: clear / tapped
        add(mv, f"3x6x2 guide bearing ({nm0} nut carrier {nm}; rides {GB_CLEAR} under the top plate)",
            ring_y(GB[0], GB[1], *ys(GB_Y - GB[2] / 2, GB_Y + GB[2] / 2), gx, gz), STEEL)
        ax = cyl_y(3.0, *ys(25.5 - 10.0, 25.5), gx, gz).fuse(cyl_y(5.5, *ys(25.5, 28.5), gx, gz))
        add(mv, f"M3 x 10 axle screw ({nm0} nut carrier guide bearing {nm}; tapped in the carrier)", one(ax.clean(), "axle"), STEEL)
    add(mv, f"nut carrier (aluminium, 10 thick, round the nut body; bolts through the flange; guide bearing slots)", one(cp.clean(), "carrier"), ALU)
    return mv, joint, dict(g, pts=pts, eye=eye, pitches=len(pts), screw_len=end - t0, thread=(end if sp["free"] == "6801" else end - 10.0) - s)


# ---------------------------------------------------------------- moving parts (Mike 2026-11-20: "How would we go about
# animating these chain links and the screw and the nut and the motor and the pulleys" / "I'd rather you went ahead and
# did this. Can we model the links of the chain so they look like links on a chain? Can we model the ball screw so it
# looks like it has some kind of grooves in it")
LEAD, BALL_D, BALL_R = 4.0, 2.5, 6.1            # SFU1204: 4 mm lead, right hand; the 2.5 ball's track on ~D12.2


def groove(sc, x0, x1):
    """the ball track: a right-hand helix along +x, cut 4 turns at a time (one long sweep cuts nothing, silently)"""
    x = x0
    while x < x1 - 0.01:
        h = min(4 * LEAD, x1 - x)
        hx = cq.Wire.makeHelix(pitch=LEAD, height=h, radius=BALL_R, center=V(x, 0, Z_S), dir=V(1, 0, 0))
        prof = cq.Wire.makeCircle(BALL_D / 2, hx.startPoint(), hx.tangentAt(0))
        sc = sc.cut(cq.Solid.sweep(prof, [], hx, isFrenet=True))
        x += h
    return sc


def _plate(y0, y1, holes):
    """a #25 side plate from x 0 to P: round ends, waisted middle"""
    re = D.PLATE_H / 2
    pl = cyl_y(D.PLATE_H, y0, y1, 0, 0).fuse(cyl_y(D.PLATE_H, y0, y1, D.P, 0)).fuse(box(0, D.P, y0, y1, -re, re))
    for sz in (1, -1):
        pl = pl.cut(cyl_y(5.2, y0 - 1, y1 + 1, D.P / 2, sz * (re + 2.05)))
    if holes:
        pl = pl.cut(cyl_y(D.PIN_D, y0 - 1, y1 + 1, 0, 0)).cut(cyl_y(D.PIN_D, y0 - 1, y1 + 1, D.P, 0))
    return pl


def _link_templates():
    """one link each way, along +x from a pin at the origin: inner (plates + rollers, or an end pin where the chain
    ends at the tab / the eye) and outer (plates + pins)"""
    yi, t = D.ROLLER_W / 2, D.PLATE_T
    inner_pl = _plate(yi, yi + t, True).fuse(_plate(-yi - t, -yi, True))
    roller = lambda x: ring_y(D.PIN_D, D.ROLLER_D, -yi, yi, x, 0)              # noqa: E731
    pin = lambda x: cyl_y(D.PIN_D, -3.95, 3.95, x, 0)                          # noqa: E731
    outer = _plate(yi + t, yi + 2 * t, False).fuse(_plate(-yi - 2 * t, -yi - t, False)).fuse(pin(0)).fuse(pin(D.P))
    tp = {}
    for a_end in (False, True):
        for b_end in (False, True):
            sh = inner_pl.fuse(pin(0) if a_end else roller(0)).fuse(pin(D.P) if b_end else roller(D.P))
            tp[("in", a_end, b_end)] = one(sh.clean(), "link")
    tp["out"] = one(outer.clean(), "link")
    return tp


_TP = None


def chain_links(pts, side, nm, nm0):
    """the chain link by link (one solid each, so the studio can run them along the path): [(name, solid)]"""
    global _TP
    if _TP is None:
        _TP = _link_templates()
    out, n = [], len(pts)
    for i in range(n - 1):
        (xa, za), (xb, zb) = pts[i], pts[i + 1]
        ang = math.degrees(math.atan2(zb - za, xb - xa))
        t = _TP[("in", i == 0, i + 1 == n - 1)] if i % 2 == 0 else _TP["out"]
        sol = t.rotate(V(0, 0, 0), V(0, 1, 0), -ang).translate(V(xa, side * YC, za))
        out.append((f"#25 roller chain {nm} link {i + 1:02d} ({nm0}, {n} pitches)", sol))
    return out


def loop(x_i):
    """the chain's closed path in the link's frame, as vertices (x, z): from (x_i, R1) along the top -x past the nut and
    carrier to the 25T, round its back, up over the wrap idler, along to the far idler and round it back to the start"""
    al, cw, r1, r2, zl, zi = C.ALPHA, C.CW, C.R1, C.R2, C.ZL, C.Z_I
    out = [(x_i, r1), (0.0, r1)]

    def arc(c, r, a0, a1):
        n = max(2, int(abs(a1 - a0) / math.radians(3.0)))
        out.extend((c[0] + r * math.cos(a0 + (a1 - a0) * k / n), c[1] + r * math.sin(a0 + (a1 - a0) * k / n))
                   for k in range(1, n + 1))
    arc((0.0, 0.0), r1, math.pi / 2, 2 * math.pi - al)
    out.append((cw[0] - r2 * math.cos(al), cw[1] + r2 * math.sin(al)))
    arc(cw, r2, math.pi - al, math.pi / 2)
    out.append((x_i, zl))
    arc((x_i, zi), r2, 1.5 * math.pi, 2.5 * math.pi)
    if math.hypot(out[-1][0] - out[0][0], out[-1][1] - out[0][1]) < 1e-6:
        out.pop()                                                               # closed: the start isn't repeated
    return out


def loop_s(vs, pt):
    """arc length along the closed loop vs to the point nearest pt"""
    best, acc = (1e9, 0.0), 0.0
    for i in range(len(vs)):
        a, b = vs[i], vs[(i + 1) % len(vs)]
        dx, dz = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(dx, dz)
        u = max(0.0, min(1.0, ((pt[0] - a[0]) * dx + (pt[1] - a[1]) * dz) / (seg * seg or 1)))
        d = math.hypot(a[0] + dx * u - pt[0], a[1] + dz * u - pt[1])
        if d < best[0]:
            best = (d, acc + seg * u)
        acc += seg
    return best[1], acc


def cheeks(x_i_a, bolt_pts, joint="elbow", key=None):
    """a link's outer end: a printed cheek each side (spacer on the link's wall + plate + the boss the next link's 6808
    runs on, carrying the next link's 25T sprockets)"""
    out = []
    for side, nm in ((1, "+y"), (-1, "-y")):
        ys = lambda a, b: C.ys(side, a, b)                                      # noqa: E731
        # a Z, 4 thick all through (Mike 2026-11-20: "make that into a Z to go from outside to be flush with the inside of
        # the other ring"): on the wall at |y| 37..41, a jog at x 161..165, then 41..45 out to the joint; top and bottom
        # flush with the top plate's top and the floor's underside
        zc = (FLOOR_Z[0], TOP_Z[1])
        ch = box(CHEEK_X0, CHEEK_JOG, *ys(WALL_Y + 4, WALL_Y + 8), *zc)
        ch = ch.fuse(box(CHEEK_JOG - 4.0, CHEEK_JOG, *ys(WALL_Y + 4, WALL_Y + 12), *zc))
        ch = ch.fuse(box(CHEEK_JOG - 4.0, L, *ys(WALL_Y + 8, WALL_Y + 12), *zc))
        ch = ch.fuse(cyl_y(24.0, *ys(WALL_Y + 8 - 0.01, WALL_Y + 12), x_i_a, C.Z_I))   # far idler shaft boss, out to 45
        # ... and part of the outer face: the 41..45 plate reaches back to it (main session 2026-11-21, Mike: "make that
        # boss become part of that other face")
        ch = ch.fuse(box(x_i_a, CHEEK_JOG - 3.99, *ys(WALL_Y + 8, WALL_Y + 12), C.Z_I - 12.0, C.Z_I + 12.0))
        ch = ch.fuse(cyl_y(64.0, *ys(WALL_Y + 8, WALL_Y + 12), L, 0))
        ch = ch.fuse(cyl_y(40.0 - FIT, *ys(YC + D.SPR_T / 2, WALL_Y + 8), L, 0))
        ch = ch.cut(cyl_y(16.0, -60, 60, L, 0)).cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, x_i_a, C.Z_I))
        ch = ch.cut(collar_pocket(x_i_a, C.Z_I, side))                         # the far idler's collar, flush
        for (bx, bz) in bolt_pts:
            ch = ch.cut(cyl_y(3.4, -60, 60, L + bx, bz))
        if key:                                                                 # the covers' screws there
            tb, th = tabs(key, side, CHEEK_X0 - BUB_R, 1e9)
            if tb is not None:
                ch = ch.fuse(tb).cut(th)
        out.append((f"{joint} cheek {nm} (printed; bolts to the link's wall, its boss carries the next link's 25T)",
                    one(ch.clean(), "cheek"), PRINT))
    return out


def base_side(bolt_pts):
    """the shoulder's bosses (stand-ins for the base cheeks)"""
    out = []
    for side, nm in ((1, "+y"), (-1, "-y")):
        ys = lambda a, b: C.ys(side, a, b)                                      # noqa: E731
        b = cyl_y(40.0, *ys(YC + D.SPR_T / 2, 48.0)).fuse(cyl_y(70.0, *ys(44.0, 48.0))).cut(cyl_y(16.0, -60, 60))
        for (bx, bz) in bolt_pts:
            b = b.cut(cyl_y(3.4, -60, 60, bx, bz))
        out.append((f"shoulder boss {nm} (stand-in for the base cheek, printed)", one(b.clean(), "boss"), PRINT))
    return out


def bolt_pts_of(joint):
    """the sprocket screw positions (x, z) from a joint's screw compound"""
    for n, s, _ in joint:
        if n.startswith("sprocket screws"):
            return [(sol.BoundingBox().center.x, sol.BoundingBox().center.z) for sol in s.Solids()]
    return []


def place_b(s, theta):
    return s.translate(V(L, 0, 0)).rotate(V(L, 0, 0), V(L, 1, 0), -theta)


def overlap(a, b):
    A, B = a.BoundingBox(), b.BoundingBox()
    if A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin:
        return 0.0
    return a.intersect(b).Volume()


def inside(parts, key=None):
    sol = [(n, s) for n, s, _ in parts if not isinstance(s, cq.Compound)]
    out = []
    for i in range(len(sol)):
        for j in range(i + 1, len(sol)):
            if key and not any(k in sol[i][0] or k in sol[j][0] for k in key):
                continue
            v = overlap(sol[i][1], sol[j][1])
            if v > 1.0:
                out.append((round(v, 1), sol[i][0], sol[j][0]))
    return out


def sweep(a_parts, b_parts):
    """link B (its chain included: it rides round with link B) against link A and the elbow's sprockets/cheeks, at
    each elbow angle; link B's own chain on its own 25T and the boss in its 6808 are meant to touch"""
    hits = []
    A = [(n, s) for n, s, _ in a_parts if not isinstance(s, cq.Compound)]
    for th in SWEEP:
        for n, s, _ in b_parts:
            if isinstance(s, cq.Compound):
                continue
            r = place_b(s, th)
            for m, t in A:
                if "roller chain" in n and ("25T sprocket" in m or " cheek " in m):
                    continue
                if "6808 bearing" in n and " cheek " in m:
                    continue
                v = overlap(r, t)
                if v > 1.0:
                    hits.append((th, round(v, 1), n, m))
    return hits


# ---------------------------------------------------------------- the studio: static parts, and the moving ones
STUDIO_LINKS = {"A": ("J1c_link_A", "chain drive J1 compact (on link A)", "J1 shoulder"),
                "B": ("J2_link_B", "chain drive J2 (link B)", "J2 elbow"),
                "C": ("J3_link_C", "chain drive J3 (link C)", "J3")}
MOVING = (("screw", ("ball screw SFU1204", "M10x1 locknut", "spacer 10 x 16 x 3", "GT2 60T pulley, 8 mm bore")),
          ("motor pulley", ("pulley, 5 mm bore",)),
          ("nut", ("ball nut SFU1204", "nut carrier", "chain tab", "tensioner angle", "eye bolt", "guide bearing")))
ZERO = {"A": 74.9, "B": -49.7, "C": -49.7}       # the joint angle with the nut mid-travel (the static files are built
#                                                   there, nut_at 0.5): J1 mid chess range 7.6..142.2; J2 and J3 the
#                                                   elbow's -104..+4.6 (SWEEP), which is the 48 mm travel exactly. The
#                                                   joints' sliders are held to the travel (Mike 2026-11-20: "you have to
#                                                   limit the rotations. Otherwise, the animation goes clear out of the end
#                                                   of the part")
MM_DEG = C.R1 * math.pi / 180.0                 # nut travel per joint degree: the chain wraps on the 25T's pitch circle


def scene_pt(k, x, y, z):
    """a point in link k's design frame -> the studio's model frame (to_scene of the link placed straight out)"""
    xo = x + "ABC".index(k) * L
    return [round(-xo, 4), round(-z, 4), round(-y + Z_MID, 4)]


def write_studio(rest, chains, infos):
    """each link: its static file (same name as before) plus one file per motion; scene.json gets the models, the
    joints they ride on, and their motions ("gears": the screw, the motor pulley, the nut sliding; "paths": the chains)"""
    import json
    import shutil
    groups, gears, paths = [], [], []
    for k in "ABC":
        tag, mname, joint = STUDIO_LINKS[k]
        sp = SPEC[k]
        sh = (lambda s: to_studio(s)) if k == "A" else (lambda s, _k=k: to_studio(s.translate(V("ABC".index(_k) * L, 0, 0))))
        stat, mov = [], {g: [] for g, _ in MOVING}
        for q in rest[k]:
            g = next((g for g, keys in MOVING if any(key in q[0] for key in keys)), None)
            (mov[g] if g else stat).append(q)
        write_flat([(n, sh(s), c) for n, s, c in stat], os.path.join(C.STUDIO, f"chain_drive_{tag}.step"))
        sets = list(mov.items()) + [(f"chain {nm}", chains[k][nm]) for nm in ("+y", "-y")]
        for g, lst in sets:
            fn = os.path.join(C.STUDIO, f"chain_drive_{tag}_{g.replace(' ', '_').replace('+', 'p').replace('-', 'm')}.step")
            write_flat([(n, sh(s), c) for n, s, c in lst], fn)
            groups.append((fn, f"{mname} {g}", joint))
        i = infos[k]
        mt, _, _, z_m = drive(sp)
        half = sp["travel"] / 2 / MM_DEG
        common = dict(follows=joint, zero=ZERO[k], range=[round(ZERO[k] - half, 2), round(ZERO[k] + half, 2)],
                      by="chain_drive_arm.py")
        r_screw = MM_DEG / LEAD * 360.0
        gears.append(dict(model=f"{mname} screw", ratio=round(r_screw, 4), at=scene_pt(k, 0, 0, Z_S), axis=[1, 0, 0], **common,
                          why=f"nut {MM_DEG:.4f} mm per joint degree (25T pitch radius), 4 mm lead, right hand"))
        gears.append(dict(model=f"{mname} motor pulley", ratio=round(r_screw * 60.0 / mt, 4), at=scene_pt(k, 0, 0, z_m),
                          axis=[1, 0, 0], **common, why=f"belt 60T:{mt}T"))
        gears.append(dict(model=f"{mname} nut", ratio=0, slide=round(MM_DEG, 5), at=scene_pt(k, i["f"], 0, Z_S),
                          axis=[-1, 0, 0], **common, why="the nut, its carrier, the chain tabs and adjusters run along the screw"))
        vs = loop(i["x_i"])
        for side, nm in ((1, "+y"), (-1, "-y")):
            ss = [loop_s(vs, q)[0] for q in i["pts"]]
            tot = loop_s(vs, i["pts"][0])[1]
            parts = [[round(ss[j], 4), round((ss[j + 1] - ss[j]) % tot, 4)] for j in range(len(ss) - 1)]
            paths.append(dict(model=f"{mname} chain {nm}", mm_per_deg=round(-MM_DEG, 5), closed=True,
                              o=scene_pt(k, 0, side * YC, 0), u=[-1, 0, 0], v=[0, -1, 0],
                              pts=[[round(x, 4), round(z, 4)] for x, z in vs], parts=parts, **common))
    # the gearbox studio, and the chain drives' own studio (Mike 2026-11-20: "Can we start another server that only
    # displays what we're working on?" - tools/studios.json "chaindrive", port 8770)
    for sc in (os.path.join(C.STUDIO, "scene.json"), os.path.join(C.HERE, "viewer_sessions", "chaindrive", "scene.json")):
        if os.path.exists(sc):
            scene_update(sc, groups, gears, paths)


def scene_update(sc, groups, gears, paths):
    """one studio's scene.json: the models, the joints they ride on (held to the drives' travel), gears and paths"""
    import json
    import shutil
    if not os.path.exists(sc + ".bak_before_anim"):
        shutil.copy2(sc, sc + ".bak_before_anim")
    with open(sc, encoding="utf-8") as fh:
        d = json.load(fh)
    have = {m["path"] for m in d["models"]}
    for fn, name, joint in groups:
        if fn not in have:
            d["models"].append({"path": fn, "name": name})
        j = next(j for j in d["joints"] if j["name"] == joint)
        if name not in j.setdefault("models", []):
            j["models"].append(name)
    for g in gears:                                        # each joint held to its drive's travel
        j = next(j for j in d["joints"] if j["name"] == g["follows"])
        j["min"], j["max"] = round(g["range"][0], 1), round(g["range"][1], 1)
        j["drive_limit"] = "the chain drive's nut travel (chain_drive_arm.py)"
    d["gears"] = [g for g in d.get("gears", []) if g.get("by") != "chain_drive_arm.py"] + gears
    d["paths"] = [q for q in d.get("paths", []) if q.get("by") != "chain_drive_arm.py"] + paths
    with open(sc, "w", encoding="utf-8") as fh:
        json.dump(d, fh, indent=1)


def main():
    a_mv, a_joint, ia = build_link("A")
    b_mv, b_joint, ib = build_link("B")
    c_mv, c_joint, ic = build_link("C")
    a_all = a_mv + cheeks(ia["x_i"], bolt_pts_of(b_joint), key="A") + [(n, place_b(s, 0.0), c) for n, s, c in b_joint]
    b_all = b_mv + cheeks(ib["x_i"], bolt_pts_of(c_joint), "J3", key="B") + [(n, place_b(s, 0.0), c) for n, s, c in c_joint]
    base = a_joint + base_side(bolt_pts_of(a_joint))
    for k, i in (("A", ia), ("B", ib), ("C", ic)):
        print(f"link {k}: screw {i['screw_len']:.0f} overall, {i['thread']:.0f} of thread, step at {i['s']:.1f}, far idler at "
              f"{i['x_i']:.1f}, motor face {i['motor_face']:.1f}; chain {i['pitches']} pitches, eye {i['eye']:.1f}")
    for k in ("A", "B", "C"):
        mt, blen, bc, zm = drive(SPEC[k])
        print(f"link {k} belt 60T:{mt}T {blen:.0f} mm: C {bc:.2f}, motor axis z {zm:.2f}, underside {zm - 21.15:.1f}, top {zm + 21.15:.1f}")
    skip = lambda p, q: (("roller chain" in p or "roller chain" in q) and ("chain tab" in p + q or "eye bolt" in p + q))         or ("eye bolt" in p + q and "tensioner angle" in p + q)         or ("button head" in p + q and "nut carrier" in p + q)         or ("GT2 60T pulley" in p + q and "GT2 belt" in p + q)          or (("shoulder screw" in p + q or "heat-set insert" in p + q) and ("frame" in p + q or "thrust wall" in p + q or "heat-set insert" in p + q)) or ("axle screw" in p + q and "nut carrier" in p + q) or ("wood screw" in p + q and "thrust wall" in p + q)      # threads in their pilots                  # noqa: E731
    for lab, parts in (("A", a_all + base), ("B", b_all), ("C", c_mv + c_joint)):
        for v, p, q in inside(parts):
            if not skip(p, q):
                print(f"  link {lab} {v:8.1f} mm3: {p[:58]}  <->  {q[:58]}")
    for lab, fixed, moving in (("elbow", a_all, b_all), ("J3", b_all, c_mv)):
        for th, v, p, q in sweep(fixed, moving):
            print(f"  {lab} {th:6.1f} {v:8.1f} mm3: {p[:58]}  <->  {q[:58]}")
    if "--ends" in sys.argv:
        key = ("ball nut", "carrier", "chain tab", "tensioner angle", "eye bolt", "roller chain")
        for k in ("A", "B", "C"):
            for at in (0.0, 1.0):
                mv_e, j_e, inf = build_link(k, at)
                print(f"link {k} nut at {at}: flange at {inf['f']:.1f}, eye {inf['eye']:.1f}")
                for v, p, q in inside(mv_e + j_e, key):
                    if not skip(p, q):
                        print(f"     {v:8.1f} mm3: {p[:55]}  <->  {q[:55]}")
    out_dir = os.path.join(C.ROOT, "chain_drive", "step")
    design = os.path.join(out_dir, "chain_drive_arm_design.step")
    infos = {"A": ia, "B": ib, "C": ic}
    carried = {"A": a_all, "B": b_all, "C": c_mv}          # each link with what rides on it (cheeks, next joint's 25Ts)
    rest, chains = {}, {}
    for k in "ABC":                                        # the studio and the design file get the chain link by link
        rest[k] = [q for q in carried[k] if not q[0].startswith("#25 roller chain")]
        chains[k] = {nm: [(n, s_, DARK) for n, s_ in chain_links(infos[k]["pts"], side, nm, SPEC[k]["name"])]
                     for side, nm in ((1, "+y"), (-1, "-y"))}
    full = {k: rest[k] + chains[k]["+y"] + chains[k]["-y"] for k in "ABC"}
    th3 = -45.0
    write_flat(full["A"] + base + [(n, place_b(s, ELBOW_SHOW), c) for n, s, c in full["B"]]
               + [(n, place_b(place_b(s, th3), ELBOW_SHOW), c) for n, s, c in full["C"]], design)
    write_flat([(n, to_studio(s), c) for n, s, c in base], os.path.join(C.STUDIO, "chain_drive_J1c_base.step"))
    write_studio(rest, chains, infos)
    print("wrote", design, "and the studio's link A, base, link B and link C files")


if __name__ == "__main__":
    main()

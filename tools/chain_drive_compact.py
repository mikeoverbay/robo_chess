#!/usr/bin/env python3
"""
chain_drive_compact.py - the shoulder (J1) chain + ball screw drive made as compact as it goes (Mike, 2026-11-20):

    "Two chains one screw. Chains mount to nut centered on nut Y. Idlers to wrap chain 270 or about around large
     sprocket."  "No rail."  "I want this as compact as we can get it. That's why I had a 270 wrap idler. We don't
     have to be a full 270."  "Can we put the motor inline with shaft?"  "...put part of the motor into the joint
     that doesn't turn that would be opposite of the chain drive"  "Let's see it"

    python chain_drive_compact.py

Changes from chain_drive.py (the first model, kept as it was):
  * two #25 chains, one each side of the screw (y +-YC), both pulling on the screw's centre line: the nut is never
    tipped, so there is no rail. The chains also stop the nut turning (about 3 deg of twist at 100 N pretension).
  * the screw sits level with the top of the two 25T sprockets (z = R1), so the pulling strand runs straight in.
  * a 12T back-bend idler beside each 25T holds the return strand level and wraps the 25T about 228 deg (the most
    that keeps the idler's teeth 1.5 mm off the sprocket's).
  * the motor (60 mm NEMA 17) is on the screw's line BEHIND the shoulder, through a 5-to-12 jaw coupler: no belt,
    39.8:1 overall. It swings in the base between the shoulder cheeks; the base must leave that space open (the
    "swing space" part, base side).
Loads (strength first; numbers worked in the gearbox session 2026-11-20):
  worst gravity 11.36 N.m -> 448 N total chain pull, 224 per chain; screw 0.32 N.m; the 60 mm motor's pull-out at
  45 deg/s is 0.495 N.m (x1.56). Stalled at full current it can only pull 1032 N in all (about 600 N on the tighter
  chain with 100 N pretension, #25 working rating 623 N). Printed parts at the 1032 N stall: thrust wall bending
  ~8 MPa, its AXK face ~3 MPa, idler 608 seats ~2 MPa; far idler M8 stubs 8 MPa in a 12 mm deep wall + boss.

Frame of the design STEP: x along link A from the shoulder axis (elbow at x 200), y = the joint axis, z up.
The studio copies are turned into the arm's model frame (link A along -x, up -y), as in chain_drive.py.
Sizes marked TYPICAL are catalogue-typical: check the real parts when they come.
"""
import json
import math
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import nema17  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402
from chain_drive import (P, ROLLER_D, SPR_T, PLATE_H, pd, od, cyl_x, cyl_y, box, ring_x, ring_y, one, hull_solid,  # noqa: E402
                         in_xz, chain, to_scene, PRINT, STEEL, ALU, DARK, BLACK)

ROOT = os.path.dirname(HERE)
OUT_DIR = os.path.join(ROOT, "chain_drive", "step")
STUDIO = os.path.join(HERE, "viewer_sessions", "gearbox")

N_JOINT = 25
R1, T1 = pd(N_JOINT) / 2, od(N_JOINT) / 2      # 25.33 pitch, 27.04 tips
# idlers (Mike 2026-11-20): no sprockets - two 625s (5 x 16 x 5, pulled from his Ender POM wheels) side by side, the
# chain riding on its plate edges, a printed guide washer each side; 5 mm shafts, lock collars
BRG = (5.0, 16.0, 5.0)
GUIDE_D, GUIDE_T, SHAFT_D = 20.0, 1.0, 5.0
R2 = math.hypot(BRG[1] / 2 + PLATE_H / 2, P / 2)   # 11.37: chain pin circle with the plates resting on the bearings
T2 = GUIDE_D / 2                                 # the idler's outermost radius (guide washers)
YC = 27.0                                      # chain planes at y +-27 (1.9 mm off the motor's sides)
Z_S = R1                                       # screw axis on the 25T top line
ZL = R1 - 2 * R2                               # return strand height (the far idler's bottom)
SCREW_D, LEAD = 12.0, 4.0
NUT_BODY_D, NUT_FL_D, NUT_FL_FLATS, NUT_FL_T, NUT_L = 24.0, 40.0, 30.0, 10.0, 35.0   # SFU1204, flats up / down
TRAVEL = 64.0                                  # 145 deg on the 25T (chess 7.6..142.2 deg)
NUT_AT = 0.5                                   # drawn at mid travel

# the screw line, joint end out: motor | plate | jaw coupler (its 12 hub is the outer collar) | AXK | thrust wall
# with a 6001 | AXK | collar | ... nut travel ... | floating wall with a 6801
TW = (29.0, 41.0)                              # printed thrust wall, 12 thick (x >= 29: its lower bridges clear the 25T)
B6001 = (31.0, 39.0)
AXK = ((25.0, 29.0), (41.0, 45.0))             # AXK1226 + 2 AS1226 washers, 12 x 26 x 4
COLLAR = (45.0, 55.0)                          # 12 mm set collar D22 x 10, set screw in the ball groove
COUPLER = (-5.0, 25.0)                         # 5-to-12 jaw coupler D25 x 30 (TYPICAL), 15 mm on each shaft
MOTOR_L, MOTOR_SQ, SHAFT_L = 60.0, 42.3, 24.0
MOTOR_FACE = COUPLER[0] + 15.0 - SHAFT_L       # -14
MPLATE = (MOTOR_FACE, MOTOR_FACE + 5.0)        # printed motor plate, hangs from the top plate
C0_MIN = COLLAR[1] + 1.0                       # carrier plate's -x face at the joint end of the travel
X_I = 155.0                                    # far idler (shaft from the wall to an inboard upright: the nut body passes between)
Z_I = R1 - R2
FLOAT = (167.0, 173.0)                         # floating wall, 6801 12 x 21 x 5
SCREW_X = (COUPLER[1] - 15.0, FLOAT[1] + 1.0)
WALL_Y = YC + 6.0                              # link walls |y| 33..37, 6808 hub rings to 40
TOP_Z = (44.0, 48.0)                           # top plate (screws off for service)
FLOOR_Z = (-8.0, -4.0)


def wrap_idler():
    """the back-bend idler beside the 25T: the most wrap whose guide washers stay 1.5 mm off the 25T's teeth and leave
    >= 1 pitch of free chain between them. Returns (alpha, centre, free length): the chain leaves the 25T at
    360 - alpha deg."""
    for a10 in range(200, 700):
        al = math.radians(a10 / 10)
        sx, sz = R1 * math.cos(al), -R1 * math.sin(al)
        gain = R2 * (1 - math.cos(math.pi / 2 - al))
        d = (ZL - sz - gain) / math.cos(al)
        c = (sx + d * math.sin(al) + R2 * math.cos(al), sz + d * math.cos(al) - R2 * math.sin(al))
        if d >= P and math.hypot(*c) >= T1 + T2 + 1.5:
            return al, c, d
    raise ValueError("no wrap idler position")


ALPHA, CW, FREE = wrap_idler()
DEEP_TOP = CW[1] - T2 - 1.5                    # the floor under the wrap idlers
DEEP = (DEEP_TOP - 4.0, DEEP_TOP)
WALL_DEEP = min(DEEP[0], CW[1] - 12.0)         # the walls' deep part (the wrap shaft's boss is D20)


def idler_stack(x, z, side, label, collar_in):
    """one idler: two 625s centred on the chain plane, a printed guide washer each side; collar_in adds a 5 mm lock
    collar inboard (it pushes the stack out against the wall and keeps a through shaft in)"""
    out = []
    y_in, y_out = YC - BRG[2], YC + BRG[2]
    for i, (a, b) in enumerate(((y_in, YC), (YC, y_out))):
        out.append((f"625 bearing ({label}) {i + 1}/2", ring_y(BRG[0], BRG[1], *ys(side, a, b), x, z), STEEL))
    for a, b in ((y_in - GUIDE_T, y_in), (y_out, y_out + GUIDE_T)):
        out.append((f"guide washer ({label}, printed)", ring_y(SHAFT_D + 0.2, GUIDE_D, *ys(side, a, b), x, z), PRINT))
    if collar_in:
        out.append((f"5 mm lock collar ({label}, inboard)",
                    ring_y(SHAFT_D, 10.0, *ys(side, y_in - GUIDE_T - 6.0, y_in - GUIDE_T), x, z), STEEL))
    return out


def far_idler(x, z, side, label, wall_out):
    """a far idler: the stack, its 5 mm shaft from the inboard upright (|y| 13..21, part of the floor) out through the
    wall to wall_out, and a lock collar outside. Supported both ends, the nut body passes between the uprights."""
    out = idler_stack(x, z, side, label, False)
    out.append((f"far idler shaft 5 mm ({label}, upright to wall)", cyl_y(SHAFT_D, *ys(side, 13.0, wall_out), x, z), STEEL))
    out.append((f"5 mm lock collar ({label}, outside the wall)", ring_y(SHAFT_D, 10.0, *ys(side, wall_out, wall_out + 6), x, z), STEEL))
    return out


def upright(x, z, side):
    """the printed upright inboard of a far idler (fused to the floor)"""
    return box(x - 8, x + 8, *ys(side, 13.0, YC - BRG[2] - GUIDE_T), FLOOR_Z[1] - 0.5, z + 7.0)


# ---------------------------------------------------------------- chain path (one plane; x, z)
def path(xa, xb):
    """dense points from the joint-side end (xa, R1) to the far end (xb, R1): -x to the 25T top, round its back
    to 360 - alpha, up to the wrap idler, over it (back-bend), level to the far 12T, round it, back to xb."""
    pts = []

    def line(p0, p1):
        n = max(2, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / 0.02))
        pts.extend((p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n) for k in range(n))

    def arc(c, r, a0, a1):
        n = max(2, int(abs(a1 - a0) * r / 0.02))
        pts.extend((c[0] + r * math.cos(a0 + (a1 - a0) * k / n), c[1] + r * math.sin(a0 + (a1 - a0) * k / n))
                   for k in range(n))

    s = (R1 * math.cos(-ALPHA), R1 * math.sin(-ALPHA))
    t = (CW[0] - R2 * math.cos(ALPHA), CW[1] + R2 * math.sin(ALPHA))
    line((xa, R1), (0.0, R1))
    arc((0.0, 0.0), R1, math.pi / 2, 2 * math.pi - ALPHA)
    line(s, t)
    arc(CW, R2, math.pi - ALPHA, math.pi / 2)
    line((CW[0], ZL), (X_I, ZL))
    arc((X_I, Z_I), R2, 1.5 * math.pi, 2.5 * math.pi)
    line((X_I, R1), (xb, R1))
    pts.append((xb, R1))
    return pts


def walk(dense):
    """pitch points from the far end back: each the chord P from the last (so every arc point sits 2 pi / n apart)"""
    out = [dense[-1]]
    i = len(dense) - 1
    while i > 0:
        x0, z0 = out[-1]
        j = i
        while j > 0 and math.hypot(dense[j][0] - x0, dense[j][1] - z0) < P:
            j -= 1
        if math.hypot(dense[j][0] - x0, dense[j][1] - z0) < P:
            break
        (xa, za), (xb, zb) = dense[j + 1], dense[j]                  # interpolate the crossing
        da, db = math.hypot(xa - x0, za - z0), math.hypot(xb - x0, zb - z0)
        f = (P - da) / (db - da) if db != da else 0
        out.append((xa + (xb - xa) * f, za + (zb - za) * f))
        i = j
    return out[::-1]                                                 # joint-side end first


def phase(points, c, r):
    """angle of a pitch point on the circle (c, r): the sprocket's tooth gaps are turned to meet it"""
    for x, z in points:
        if abs(math.hypot(x - c[0], z - c[1]) - r) < 0.05:
            return math.atan2(z - c[1], x - c[0])
    return math.pi / 2


def sprocket(n, cx, cz, ph, bore, side, hub=None, bolts=()):
    """#25 sprocket in the plane y = side * YC, a tooth gap at angle ph; the hub (D, length) points out to the wall"""
    s = cyl_y(od(n), -SPR_T / 2, SPR_T / 2, cx, cz)
    rp = pd(n) / 2
    seat = 1.005 * ROLLER_D + 0.076
    for k in range(n):
        t = ph + 2 * math.pi * k / n
        c1 = (cx + rp * math.cos(t), cz + rp * math.sin(t))
        c2 = (cx + (rp + 3.2) * math.cos(t), cz + (rp + 3.2) * math.sin(t))
        s = s.cut(hull_solid(c1, seat / 2, c2, 2.6, in_xz(-SPR_T), V(0, 2 * SPR_T, 0)))
    if hub:
        s = s.fuse(cyl_y(hub[0], SPR_T / 2 - 0.1, SPR_T / 2 + hub[1], cx, cz))
    s = s.cut(cyl_y(bore, -50, 50, cx, cz))
    for (bx, bz) in bolts:
        s = s.cut(cyl_y(3.4, -50, 50, bx, bz))
    s = one(s.clean(), f"{n}T sprocket")
    if side < 0:
        s = s.mirror("XZ")
    return s.translate(V(0, side * YC, 0))


def ys(side, a, b):
    """the y range a..b (distances from the middle) on one side"""
    return tuple(sorted((side * a, side * b)))


# ---------------------------------------------------------------- the parts
def build():
    moving, fixed = [], []
    add = lambda lst, name, s, c: lst.append((name, s, c))     # noqa: E731
    c0 = C0_MIN + TRAVEL * NUT_AT                              # carrier plate -x face
    fl0, fl1 = c0 + 10.0, c0 + 10.0 + NUT_FL_T                 # nut flange
    body1 = c0 + 10.0 + NUT_L                                  # nut body end
    xb = c0 + 20.0                                             # far chain end pin (on the tab; further out, the tab
    #                                                            hits the far 12T at the end of travel)
    dense = path(c0 - 12.0, xb)
    pts = walk(dense)
    eye = pts[0][0]                                            # joint-side chain end: the M4 adjuster's eye
    ph1 = phase(pts, (0.0, 0.0), R1)
    bolts = [(16 * math.cos(math.radians(30 + 60 * k)), 16 * math.sin(math.radians(30 + 60 * k))) for k in range(6)]

    for side, nm in ((1, "+y"), (-1, "-y")):
        # --- base side (never turns with J1)
        add(fixed, f"#25 {N_JOINT}T plate sprocket 25A25 {nm} (bolted to the boss)",
            sprocket(N_JOINT, 0, 0, ph1, 20.0, side, bolts=bolts), DARK)
        b = cyl_y(40.0, *ys(side, YC + SPR_T / 2, 48.0)).fuse(cyl_y(70.0, *ys(side, 44.0, 48.0))).cut(cyl_y(16.0, -60, 60))
        for (bx, bz) in bolts:
            b = b.cut(cyl_y(3.4, -60, 60, bx, bz))
        add(fixed, f"shoulder boss {nm} (stand-in for the base cheek, printed)", one(b.clean(), "boss"), PRINT)
        add(fixed, f"sprocket bolts M3 x 6 {nm}",
            cq.Compound.makeCompound([cyl_y(3.0, *ys(side, YC - 1.5, 46.0), bx, bz) for bx, bz in bolts]), BLACK)

        # --- link A walls (printed)
        y0, y1 = ys(side, WALL_Y, WALL_Y + 4)
        r0, r1 = ys(side, WALL_Y, WALL_Y + 7)
        w = box(0, 200, y0, y1, FLOOR_Z[0], TOP_Z[0])
        w = w.fuse(box(MOTOR_FACE, 0, y0, y1, 26.0, TOP_Z[0]))                    # behind the joint: holds the top plate
        w = w.fuse(box(0, 64, y0, y1, WALL_DEEP, FLOOR_Z[0]))                      # the deep end under the wrap idlers
        for hx in (0.0, 200.0):
            w = w.fuse(cyl_y(64.0, r0, r1, hx, 0))
        w = w.fuse(cyl_y(20.0, r0, r1, *CW))                                       # wrap-idler shaft boss (7 deep)
        w = w.fuse(cyl_y(24.0, *ys(side, WALL_Y, WALL_Y + 12), X_I, Z_I))         # far-idler shaft boss (12 deep)
        win = box(72, 138, y0 - 1, y1 + 1, 2, 38)
        w = w.cut(win.fillet(8, win.edges("|Y")))
        for hx in (0.0, 200.0):
            w = w.cut(cyl_y(52.0, -60, 60, hx, 0))                                 # 6808 seats
        w = w.cut(cyl_y(SHAFT_D + 0.2, -60, 60, *CW)).cut(cyl_y(SHAFT_D + 0.2, -60, 60, X_I, Z_I))
        add(moving, f"link A wall {nm} (printed)", one(w.clean(), "wall"), PRINT)

        # --- bearings, idlers
        add(moving, f"6808 bearing (shoulder, {nm})", ring_y(40.0, 52.0, r0, r1), STEEL)
        for n_, s_, c_ in idler_stack(CW[0], CW[1], side, f"wrap idler {nm}", True):
            add(moving, n_, s_, c_)
        for n_, s_, c_ in far_idler(X_I, Z_I, side, f"far idler {nm}", WALL_Y + 12.0):
            add(moving, n_, s_, c_)

        # --- chain, chain tab, adjuster
        add(moving, f"#25 roller chain {nm} ({len(pts)} pitches)", chain(pts).translate(V(0, side * YC, 0)), DARK)
        tab = box(c0 - 4.0, xb + 2.5, *ys(side, YC - 1.5, YC + 1.5), Z_S - 4, Z_S + 4)
        tab = tab.fuse(box(c0 - 4.0, c0, *ys(side, YC - 3.0, YC + 3.0), Z_S - 4, Z_S + 4))
        tab = tab.cut(cyl_y(2.4, -60, 60, xb, Z_S)).cut(cyl_x(4.4, c0 - 5, c0 + 1, side * YC, Z_S))
        tab = tab.fuse(box(c0, c0 + 10.0, *ys(side, YC - 1.5, YC + 1.5), Z_S - 8, Z_S + 8))  # 2 x M4 into the plate's side
        add(moving, f"chain tab {nm} (steel, 3 thick; far chain end pinned to it)", one(tab.clean(), "tab"), DARK)
        adj = cyl_x(4.0, eye + 1.5, c0 - 0.5, side * YC, Z_S).fuse(ring_y(2.4, 7.0, *ys(side, YC - 1.5, YC + 1.5), eye, Z_S))
        adj = adj.fuse(cyl_x(7.0, c0 - 7.0, c0 - 4.0, side * YC, Z_S))
        add(moving, f"M4 chain adjuster {nm} (eye bolt into the tab's tapped end, lock nut; sets the 100 N pretension)",
            one(adj.clean(), "adj"), STEEL)

    # --- link A floor, top plate, thrust wall, floating wall, motor plate (printed)
    floor = box(64, 180, -WALL_Y, WALL_Y, *FLOOR_Z).fuse(box(24, 64, -WALL_Y, WALL_Y, *DEEP))
    floor = floor.fuse(box(60, 64, -WALL_Y, WALL_Y, DEEP[0], FLOOR_Z[1]))
    for side in (1, -1):
        floor = floor.fuse(upright(X_I, Z_I, side))
    floor = floor.cut(cyl_y(SHAFT_D + 0.2, -60, 60, X_I, Z_I))
    add(moving, "link A floor (printed; with the far idlers' inboard uprights)", one(floor.clean(), "floor"), PRINT)
    top = box(MOTOR_FACE, 180, -(WALL_Y + 4), WALL_Y + 4, *TOP_Z)
    add(moving, "link A top plate (printed; screws off for service)", top, PRINT)
    tw = box(*TW, -21, 21, 6.0, TOP_Z[0])
    for side in (1, -1):
        tw = tw.fuse(box(*TW, *ys(side, 21, WALL_Y), 8.0, 20.0))                   # lower bridges (the chain runs under)
    tw = tw.cut(cyl_x(14.0, TW[0] - 1, TW[1] + 1, 0, Z_S)).cut(cyl_x(28.0, *B6001, 0, Z_S))
    add(moving, "thrust wall (printed, 12 thick): 6001 + an AXK1226 each face", one(tw.clean(), "thrust wall"), PRINT)
    fw = box(*FLOAT, -16, 16, FLOOR_Z[1], TOP_Z[0]).cut(cyl_x(14.0, FLOAT[0] - 1, FLOAT[1] + 1, 0, Z_S))
    fw = fw.cut(cyl_x(21.0, FLOAT[0] + 0.5, FLOAT[1] - 0.5, 0, Z_S))
    add(moving, "floating end wall (printed): 6801", one(fw.clean(), "float wall"), PRINT)
    mp = box(*MPLATE, -21.15, 21.15, Z_S - 21.15, TOP_Z[0]).cut(cyl_x(22.5, MPLATE[0] - 1, MPLATE[1] + 1, 0, Z_S))
    for sy in (-15.5, 15.5):
        for sz in (-15.5, 15.5):
            mp = mp.cut(cyl_x(3.4, MPLATE[0] - 1, MPLATE[1] + 1, sy, Z_S + sz))
    add(moving, "motor plate (printed): the 60 mm stepper bolts to it, hangs from the top plate", one(mp.clean(), "motor plate"), PRINT)

    # --- screw line
    add(moving, f"ball screw SFU1204 (cut to {SCREW_X[1] - SCREW_X[0]:.0f})", cyl_x(SCREW_D, *SCREW_X, 0, Z_S), STEEL)
    add(moving, "6001 bearing 12x28x8 (thrust wall)", ring_x(12.0, 28.0, *B6001, 0, Z_S), STEEL)
    for i, (a, b) in enumerate(AXK):
        add(moving, f"AXK1226 thrust bearing + 2 AS1226 washers {i + 1}/2", ring_x(12.0, 26.0, a, b, 0, Z_S), STEEL)
    col = ring_x(12.0, 22.0, *COLLAR, 0, Z_S).cut(cyl_y(4.0, 0, 20, sum(COLLAR) / 2, Z_S))
    add(moving, "shaft collar 12 mm (set screw in the ball groove)", one(col.clean(), "collar"), STEEL)
    cp = ring_x(5.0, 25.0, COUPLER[0], COUPLER[0] + 15, 0, Z_S).fuse(ring_x(12.0, 25.0, COUPLER[0] + 15, COUPLER[1], 0, Z_S))
    add(moving, "jaw coupler 5 to 12 mm D25 x 30 (TYPICAL; 12 hub = outer thrust collar, set screws in the groove)",
        one(cp.clean(), "coupler"), ALU)
    add(moving, "6801 bearing 12x21x5 (floating end)", ring_x(12.0, 21.0, FLOAT[0] + 0.5, FLOAT[1] - 0.5, 0, Z_S), STEEL)
    add(moving, "wrap idler shaft 5 mm (hardened rod through both walls; the inboard collars keep it in)",
        cyl_y(SHAFT_D, -(WALL_Y + 7), WALL_Y + 7, *CW), STEEL)
    body, caps = nema17.load(MOTOR_L)
    caps = nema17.trimmed_caps(caps, SHAFT_L)
    place = lambda s: s.rotate(V(0, 0, 0), V(0, 1, 0), 90).translate(V(MOTOR_FACE, 0, Z_S))   # noqa: E731
    add(moving, "NEMA 17 60 mm (inline behind the shoulder, shaft into the coupler)", place(body), BLACK)
    add(moving, "NEMA 17 end caps", place(caps), STEEL)

    # --- nut and its carrier
    nut = cyl_x(NUT_BODY_D, fl1, body1, 0, Z_S)
    fl = cyl_x(NUT_FL_D, fl0, fl1, 0, Z_S).cut(box(fl0 - 1, fl1 + 1, -30, 30, Z_S + NUT_FL_FLATS / 2, Z_S + 30))
    fl = fl.cut(box(fl0 - 1, fl1 + 1, -30, 30, Z_S - 30, Z_S - NUT_FL_FLATS / 2))
    add(moving, "ball nut SFU1204 (flats up and down)", one(nut.fuse(fl).cut(cyl_x(SCREW_D, 0, 200, 0, Z_S)).clean(), "nut"), STEEL)
    cp = box(c0, c0 + 10.0, -25.5, 25.5, Z_S - 16, Z_S + 16).cut(cyl_x(13.0, c0 - 1, c0 + 11, 0, Z_S))
    add(moving, "nut carrier plate (aluminium, 10 thick): nut flange bolts on, a chain tab each side",
        one(cp.clean(), "carrier"), ALU)

    # --- swing space the base must leave open (the motor, 7.6..142.2 deg), base side
    mbox = box(MOTOR_FACE - MOTOR_L, MPLATE[1], -21.15, 21.15, Z_S - 21.15, Z_S + 21.15)
    angs = [7.6 + k * 2.5 for k in range(54)] + [142.2]
    sw = [mbox.rotate(V(0, 0, 0), V(0, 1, 0), -a) for a in angs]
    sw = sw[0].fuse(*sw[1:]).clean()
    add(fixed, "inline motor swing space, chess range 7.6..142.2 deg (the base keeps this open)", sw,
        cq.Color(0.95, 0.88, 0.55))
    info = dict(c0=c0, eye=eye, pitches=len(pts), wrap=270 - math.degrees(ALPHA), free=FREE, cw=CW,
                motor_back=MOTOR_FACE - MOTOR_L, screw=SCREW_X[1] - SCREW_X[0])
    return moving, fixed, info


def check(moving, fixed, angles=(0.0,)):
    """overlaps over 1 mm3 between solid parts, the link turned to each J1 angle against the base side (the chain
    and its sprockets, and parts meant to touch, show up; anything else is a real clash)"""
    hits = []
    mv = [(n, s) for n, s, _ in moving if not isinstance(s, cq.Compound)]
    fx = [(n, s) for n, s, _ in fixed if not isinstance(s, cq.Compound) and "swing space" not in n]

    def ov(a, b):
        A, B = a.BoundingBox(), b.BoundingBox()
        if A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin:
            return 0.0
        return a.intersect(b).Volume()

    for i in range(len(mv)):
        for j in range(i + 1, len(mv)):
            v = ov(mv[i][1], mv[j][1])
            if v > 1.0:
                hits.append((0.0, round(v, 1), mv[i][0], mv[j][0]))
    for a in angles:
        for n, s in mv:
            if "roller chain" in n:
                continue
            r = s.rotate(V(0, 0, 0), V(0, 1, 0), -a)
            for m, f in fx:
                v = ov(r, f)
                if v > 1.0:
                    hits.append((a, round(v, 1), n, m))
    return hits


def main():
    moving, fixed, info = build()
    os.makedirs(OUT_DIR, exist_ok=True)
    design = os.path.join(OUT_DIR, "chain_drive_J1_compact_design.step")
    write_flat(moving + fixed, design)
    sm = os.path.join(STUDIO, "chain_drive_J1c_link_A.step")
    sf = os.path.join(STUDIO, "chain_drive_J1c_base.step")
    write_flat([(n, to_scene(s), c) for n, s, c in moving], sm)
    write_flat([(n, to_scene(s), c) for n, s, c in fixed], sf)
    lo = min(s.BoundingBox().zmin for n, s, _ in moving if "wall" in n)
    print(f"wrap {info['wrap']:.1f} deg ({info['wrap'] / 360 * N_JOINT:.1f} teeth), wrap idler at ({CW[0]:.2f}, {CW[1]:.2f}) "
          f"free chain {info['free']:.1f}; chain {info['pitches']} pitches each, adjuster eye at x {info['eye']:.1f} "
          f"(carrier at {info['c0']:.1f}); screw {info['screw']:.0f}; motor back at x {info['motor_back']:.0f}")
    print(f"link A: z {FLOOR_Z[0]}..{TOP_Z[1]} along the run ({TOP_Z[1] - FLOOR_Z[0]:.0f} tall), walls |y| {WALL_Y}..{WALL_Y + 4} "
          f"({2 * (WALL_Y + 4):.0f} wide), deepest {lo:.1f}")
    for a, v, p, q in check(moving, fixed, angles=(7.6, 75.0, 142.2)):
        print(f"  J1 {a:5.1f}  overlap {v:8.1f} mm3: {p}  <->  {q}")
    print("wrote", design, sm, sf, sep="\n  ")


if __name__ == "__main__":
    main()

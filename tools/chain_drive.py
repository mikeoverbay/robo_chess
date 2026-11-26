#!/usr/bin/env python3
"""
chain_drive.py - Mike's chain + ball screw joint drive (2026-11-20), first model: the SHOULDER (J1) on link A.

    "Chain sprocket on joint. Idler to wrap it around. Smaller sprocket farther up in arm. Chain attached to nut on
     12 mm ball nut ... Custom printed parts so chain runs under ball screw and carries ball screw bearing ..."
    "Motor can be on top of arm segments." "I'd buy more nuts and trucks." "I'll buy new ones 12mm bore" (60T)

    python chain_drive.py

How it works: a #25 25T plate sprocket is bolted to the BASE (it never turns). Link A carries an SFU1204 ball screw
above the chain. The nut sits in an aluminium block that is part of the chain loop (chain end - block - chain end);
the loop runs round the fixed sprocket and a 12T idler up the link. The motor on top turns the screw through GT2
20T -> 60T (3:1); the nut moves along the link, pulls the chain, and the chain walks the link round the fixed
sprocket. An upside-down MGN12H truck on a rail under the deck holds the block level (the chain pulls 20.7 mm below
the screw: 12.4 N.m of tilt at the 600 N current limit, HIWIN static pitch rating 36.3 N.m).

Loads (chain_screw.py in the gearbox session's scratchpad; strength first, 2026-11-20 rule):
  J1 worst gravity 11.36 N.m -> chain 448 N on the 25T (PD 50.66); driver current capped so the chain never
  passes 600 N (#25: 140 lb = 623 N working, 1,050 lb break). Screw 0.32 N.m, motor 0.11 N.m through 3:1 (19 % of
  the 48 mm 17HS19), belt 17 N. Printed parts at 600 N: thrust shoulders ~3 MPa, idler 608 seats ~4 MPa, fixed end
  wall 10 thick at the screw (a 4 mm plate there would see ~56 MPa).
Travel: chess uses the shoulder from 7.6 to 142.2 deg above horizontal (motor_speed.arm over every square and tray
spot at 0 / 60 / 125 up); 64 mm of nut travel = 145 deg on the 25T. The scene's full -14..180 does NOT fit link A.

Frame of the design STEP: x along link A from the shoulder axis (elbow at x 200), y = the joint axis, z up; the
chain runs in the y = 0 plane. The studio copies are turned into the arm's model frame (link A along -x, up -y).
Positions marked TYPICAL are catalogue-typical sizes: check the real parts when they come.
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

ROOT = os.path.dirname(HERE)
OUT_DIR = os.path.join(ROOT, "chain_drive", "step")
STUDIO = os.path.join(HERE, "viewer_sessions", "gearbox")

# ---------------------------------------------------------------- bought parts (TYPICAL catalogue sizes)
P = 6.35                                   # #25 chain: pitch, roller D x width, plate height / thickness
ROLLER_D, ROLLER_W, PLATE_H, PLATE_T, PIN_D = 3.30, 3.18, 5.84, 0.76, 2.31
SPR_T = 2.79                               # #25 sprocket tooth width (0.110 in)


def pd(n):
    return P / math.sin(math.pi / n)


def od(n):
    return P * (0.6 + 1 / math.tan(math.pi / n))


N_JOINT, N_IDLER = 25, 12
R1, R2 = pd(N_JOINT) / 2, pd(N_IDLER) / 2  # 25.33, 12.27
SCREW_D, LEAD = 12.0, 4.0                  # SFU1204
Z_S = 51.0                                 # screw axis height (60T flange clears the chain on the sprocket by 1.8)
NUT_BODY_D, NUT_FL_D, NUT_FL_FLATS, NUT_FL_T, NUT_L = 24.0, 40.0, 30.0, 10.0, 35.0   # SFU1204 nut, flats down/up
BLOCK_L, BLOCK_W, BLOCK_Z = 30.0, 40.0, (18.0, 67.0)    # aluminium nut block (DSG12H-style or cut from bar)
RAIL_W, RAIL_H = 12.0, 8.0                 # MGN12
TRUCK_W, TRUCK_L, TRUCK_H, TRUCK_BODY = 27.0, 45.4, 13.0, 10.0   # MGN12H: H = rail base to mounting face
DECK_Z = (80.0, 88.0)                      # rail under it, motor on top (8 thick: rail bolts through with washers)
RAIL_Z = (DECK_Z[0] - RAIL_H, DECK_Z[0])
TRAVEL = 64.0                              # 145 deg on the 25T

# fixed end (joint end), x from the outside in: 60T pulley (its hub is the outer collar) | AXK1226 | wall with a
# 6001 | AXK1226 | collar. The pulley's set screw sits IN the ball groove (the lands are 56-62 HRC).
BELT_X = -16.0                             # belt middle plane
G60 = dict(n=60, bore=12.0, teeth=(-19.5, -12.5), fl_d=42.0, hub=(-11.5, -4.5), hub_d=25.0)
G20 = dict(n=20, bore=5.0, teeth=(-19.5, -12.5), fl_d=16.0, hub=(-11.5, -4.5), hub_d=13.0)
AXK = ((-4.5, -0.5), (9.5, 13.5))          # AXK1226 + two AS1226 washers: 12 x 26 x 4
WALL_X = (-0.5, 9.5)                       # printed fixed end wall, 10 thick at the screw
B6001 = (0.5, 8.5)                         # 12 x 28 x 8
COLLAR = (13.5, 23.5)                      # 12 mm set collar D22 x 10
FL_MIN = 25.5                              # nut flange outer face at the joint end of the travel
SCREW_X = (-20.5, 148.0)
FLOAT_X = (141.0, 147.0)                   # floating end wall with a 6801 (12 x 21 x 5)
X_I = 150.0                                # 12T idler centre (its top tangent at z R1: the nut strand runs level)
Z_I = R1 - R2
BELT_L, BELT_T, BELT_W = 200.0, 1.38, 6.0  # his 200 x 6 GT2 loops
R60, R20 = 60 * 2.0 / math.pi / 2, 20 * 2.0 / math.pi / 2


def belt_c():
    """centre distance for the 200 mm loop on 20T + 60T (no tensioner deflection)"""
    k = (R60 - R20) ** 2
    s = BELT_L - math.pi * (R60 + R20)
    return (s + math.sqrt(s * s - 8 * k)) / 4


Z_M = Z_S + belt_c()                       # motor axis height
MOTOR_FACE_X, MOTOR_L, MOTOR_SQ = 3.5, 48.0, 42.3
WALL_Y = 24.0                              # link walls at |y| 24..28, hub rings to 31
LINK_Z = (-35.0, DECK_Z[1])
NUT_AT = 0.5                               # nut drawn at mid travel

PRINT = cq.Color(0.73, 0.74, 0.76)
STEEL, ALU, DARK, BLACK, BRONZE = (cq.Color(0.78, 0.79, 0.81), cq.Color(0.86, 0.87, 0.89), cq.Color(0.35, 0.36, 0.38),
                                   cq.Color(0.16, 0.17, 0.19), cq.Color(0.72, 0.55, 0.25))


# ---------------------------------------------------------------- helpers
def cyl_x(d, x0, x1, y=0.0, z=0.0):
    return cq.Solid.makeCylinder(d / 2, x1 - x0, V(x0, y, z), V(1, 0, 0))


def cyl_y(d, y0, y1, x=0.0, z=0.0):
    return cq.Solid.makeCylinder(d / 2, y1 - y0, V(x, y0, z), V(0, 1, 0))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def ring_x(d_in, d_out, x0, x1, y=0.0, z=0.0):
    return cyl_x(d_out, x0, x1, y, z).cut(cyl_x(d_in, x0 - 1, x1 + 1, y, z))


def ring_y(d_in, d_out, y0, y1, x=0.0, z=0.0):
    return cyl_y(d_out, y0, y1, x, z).cut(cyl_y(d_in, y0 - 1, y1 + 1, x, z))


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1 and s[0].isValid(), f"{what}: {len(s)} solids"
    return s[0]


def hull_pts(c1, r1, c2, r2):
    """the outline of two circles' convex hull: (P1a, P2a, far2, P2b, P1b, far1) in the circles' plane"""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d, phi = math.hypot(dx, dy), math.atan2(dy, dx)
    a = math.acos((r1 - r2) / d)
    pt = lambda c, r, t: (c[0] + r * math.cos(t), c[1] + r * math.sin(t))   # noqa: E731
    return (pt(c1, r1, phi + a), pt(c2, r2, phi + a), pt(c2, r2, phi), pt(c2, r2, phi - a), pt(c1, r1, phi - a),
            pt(c1, r1, phi + math.pi))


def hull_solid(c1, r1, c2, r2, to3d, depth_vec):
    p1a, p2a, f2, p2b, p1b, f1 = [to3d(*p) for p in hull_pts(c1, r1, c2, r2)]
    w = cq.Wire.assembleEdges([cq.Edge.makeLine(p1a, p2a), cq.Edge.makeThreePointArc(p2a, f2, p2b),
                               cq.Edge.makeLine(p2b, p1b), cq.Edge.makeThreePointArc(p1b, f1, p1a)])
    return cq.Solid.extrudeLinear(w, [], depth_vec)


def in_xz(y0):
    return lambda u, v: V(u, y0, v)


# ---------------------------------------------------------------- sprockets, pulleys, chain
def sprocket(n, cx, cz, bore, hub=None, bolts=()):
    """#25 sprocket in the y = 0 plane (teeth SPR_T wide), a tooth gap pointing straight up"""
    s = cyl_y(od(n), -SPR_T / 2, SPR_T / 2, cx, cz)
    rp = pd(n) / 2
    seat = 1.005 * ROLLER_D + 0.076
    for k in range(n):
        t = math.pi / 2 + 2 * math.pi * k / n
        c1 = (cx + rp * math.cos(t), cz + rp * math.sin(t))
        c2 = (cx + (rp + 3.2) * math.cos(t), cz + (rp + 3.2) * math.sin(t))
        s = s.cut(hull_solid(c1, seat / 2, c2, 2.6, in_xz(-SPR_T), V(0, 2 * SPR_T, 0)))
    if hub:
        s = s.fuse(cyl_y(hub[0], SPR_T / 2 - 0.1, hub[1], cx, cz))
    s = s.cut(cyl_y(bore, -50, 50, cx, cz))
    for (bx, bz) in bolts:
        s = s.cut(cyl_y(3.4, -50, 50, bx, bz))
    return one(s.clean(), f"{n}T sprocket")


def pulley(g, y=0.0, z=0.0):
    t0, t1 = g["teeth"]
    r_t = g["n"] * 2.0 / math.pi / 2 - 0.254
    p = cyl_x(2 * r_t, t0, t1, y, z).fuse(cyl_x(g["fl_d"], t0 - 1, t0, y, z)).fuse(cyl_x(g["fl_d"], t1, t1 + 1, y, z))
    p = p.fuse(cyl_x(g["hub_d"], *g["hub"], y, z))
    for k in range(g["n"]):
        a = 2 * math.pi * k / g["n"]
        p = p.cut(cyl_x(1.3, t0, t1, y + (r_t + 0.2) * math.cos(a), z + (r_t + 0.2) * math.sin(a)))
    p = p.cut(cyl_x(g["bore"], -100, 100, y, z))
    p = p.cut(cyl_y(3.0, y, y + 20, (g["hub"][0] + g["hub"][1]) / 2, z))      # the set screw (into the ball groove)
    return one(p.clean(), f"GT2 {g['n']}T")


def chain_points(x_end1, x_end2):
    """pitch points of the loop, broken at the nut block: the nut strand from the sprocket's top tangent (x 0) to
    x_end1, the block, then from x_end2 to the idler's top tangent; round the idler, back along the bottom strand,
    round the back of the fixed sprocket. Returns [[(x, z), ...] per run] (each run is one continuous piece)."""
    p1a, p2a, _, p2b, p1b, _ = hull_pts((X_I, Z_I), R2, (0.0, 0.0), R1)       # idler first: a = the top (nut) side
    # p1a/p2a: idler / sprocket on one side, p2b/p1b the other; pick by z
    if p1a[1] < p1b[1]:
        p1a, p1b, p2a, p2b = p1b, p1a, p2b, p2a
    top_i, top_s, bot_s, bot_i = p1a, p2a, p2b, p1b
    run2 = []                                                   # x_end2 -> idler top -> round -> bottom -> sprocket
    x = x_end2
    while x < top_i[0] - 0.01:
        run2.append((x, R1))
        x += P
    a0 = math.atan2(top_i[1] - Z_I, top_i[0] - X_I)
    a1 = math.atan2(bot_i[1] - Z_I, bot_i[0] - X_I)
    step = 2 * math.pi / N_IDLER
    a = a0
    while a > a1:
        run2.append((X_I + R2 * math.cos(a), Z_I + R2 * math.sin(a)))
        a -= step
    L = math.hypot(bot_s[0] - bot_i[0], bot_s[1] - bot_i[1])
    ux, uz = (bot_s[0] - bot_i[0]) / L, (bot_s[1] - bot_i[1]) / L
    s = 0.0
    while s < L:
        run2.append((bot_i[0] + ux * s, bot_i[1] + uz * s))
        s += P
    b0 = math.atan2(bot_s[1], bot_s[0]) % (2 * math.pi)
    step = 2 * math.pi / N_JOINT
    b = math.pi / 2 + math.floor((b0 - math.pi / 2) / step) * step   # rollers in the tooth gaps (a gap points up)
    while b > math.pi / 2 - 1e-6:                                # round the back, from the bottom up to the top
        run2.append((R1 * math.cos(b), R1 * math.sin(b)))
        b -= step
    x = P
    while x < x_end1 + 0.01:
        run2.append((x, R1))
        x += P
    return run2


def chain(points):
    """rollers, pins and alternating inner / outer plates on the pitch points (a compound: one part)"""
    sol = []
    for i, (x, z) in enumerate(points):
        if 0 < i < len(points) - 1:                              # the end pins go through the bracket / the eye
            sol.append(cyl_y(ROLLER_D, -ROLLER_W / 2, ROLLER_W / 2, x, z))
        sol.append(cyl_y(PIN_D, -3.95, 3.95, x, z))
    for i in range(len(points) - 1):
        (xa, za), (xb, zb) = points[i], points[i + 1]
        L = math.hypot(xb - xa, zb - za)
        if L > 1.5 * P:                                          # the gap at the nut block
            continue
        ang = math.degrees(math.atan2(zb - za, xb - xa))
        y_in = ROLLER_W / 2 + (0 if i % 2 == 0 else PLATE_T)
        for sgn in (-1, 1):
            y0 = y_in if sgn > 0 else -y_in - PLATE_T
            pl = box(-PLATE_H / 2, L + PLATE_H / 2, y0, y0 + PLATE_T, -PLATE_H / 2, PLATE_H / 2)
            sol.append(pl.rotate(V(0, 0, 0), V(0, 1, 0), -ang).translate(V(xa, 0, za)))
    fused = sol[0].fuse(*sol[1:]).clean()                       # the pins tie every plate and roller together
    return one(fused, "chain")


# ---------------------------------------------------------------- the parts
def build():
    moving, fixed = [], []
    add = lambda lst, name, s, c: lst.append((name, s, c))     # noqa: E731
    dx = TRAVEL * NUT_AT
    fl0 = FL_MIN + dx                                          # nut flange outer face
    blk0 = fl0 + NUT_FL_T                                      # block -x face
    blk1 = blk0 + BLOCK_L

    # --- base side (does not turn with J1)
    bolts = [(16 * math.cos(math.radians(30 + 60 * k)), 16 * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    add(fixed, f"#25 {N_JOINT}T plate sprocket 25A25 (bolted to the base)", sprocket(N_JOINT, 0, 0, 20.0, bolts=bolts), DARK)
    for sgn, nm in ((1, "+y"), (-1, "-y")):
        y0, y1 = sorted((sgn * SPR_T / 2, sgn * 40.0))
        b = cyl_y(40.0, y0, y1)
        f0, f1 = sorted((sgn * 36.0, sgn * 40.0))
        b = b.fuse(cyl_y(70.0, f0, f1)).cut(cyl_y(16.0, -60, 60))
        for (bx, bz) in bolts:
            b = b.cut(cyl_y(3.4, -60, 60, bx, bz))
        add(fixed, f"shoulder boss {nm} (stand-in for the base cheek, printed)", one(b.clean(), "boss"), PRINT)
    add(fixed, "sprocket bolts M3 x 6 (through both bosses)",
        cq.Compound.makeCompound([cyl_y(3.0, -40, 40, bx, bz) for bx, bz in bolts]), BLACK)

    # --- link A (printed)
    for sgn, nm in ((1, "+y"), (-1, "-y")):
        y0, y1 = sorted((sgn * WALL_Y, sgn * (WALL_Y + 4)))
        r0, r1 = sorted((sgn * WALL_Y, sgn * (WALL_Y + 7)))
        w = box(0, 200, y0, y1, LINK_Z[0], LINK_Z[1])
        for hx in (0.0, 200.0):
            w = w.fuse(cyl_y(64.0, r0, r1, hx, 0))
        win = box(40, 124, y0 - 1, y1 + 1, -20, 66)
        win = win.fillet(10, win.edges("|Y"))
        w = w.cut(win)
        for hx in (0.0, 200.0):
            w = w.cut(cyl_y(52.0, -60, 60, hx, 0))                            # 6808 seats
        w = w.fuse(ring_y(8.0, 30.0, r0, r1, X_I, Z_I)).cut(cyl_y(22.0, *sorted((sgn * WALL_Y, sgn * (WALL_Y + 7.01))), X_I, Z_I))
        add(moving, f"link A wall {nm} (printed)", one(w.clean(), "wall"), PRINT)
    add(moving, "link A floor (printed)", box(0, 200, -WALL_Y, WALL_Y, LINK_Z[0], LINK_Z[0] + 4), PRINT)
    deck = box(WALL_X[1], 160, -WALL_Y, WALL_Y, *DECK_Z)
    for k in range(5):                                                         # rail bolts through, nuts on top
        deck = deck.cut(cq.Solid.makeCylinder(1.7, 20, V(32.5 + 25 * k, 0, DECK_Z[0] - 1), V(0, 0, 1)))
    add(moving, "link A deck (printed; MGN12 rail under, M3 through with washers)", deck, PRINT)

    fw = box(WALL_X[0], WALL_X[1], -WALL_Y, WALL_Y, 33.0, DECK_Z[1])               # 10 thick at the screw, to the deck top
    fw = fw.fuse(box(WALL_X[0], MOTOR_FACE_X, -WALL_Y, WALL_Y, DECK_Z[1], Z_M + MOTOR_SQ / 2))  # the plate the motor bolts to
    fw = fw.cut(cyl_x(14.0, -10, 20, 0, Z_S)).cut(cyl_x(28.0, *B6001, 0, Z_S))
    fw = fw.cut(cyl_x(22.5, -10, 20, 0, Z_M))
    for sy in (-15.5, 15.5):
        for sz in (-15.5, 15.5):
            fw = fw.cut(box(-5, 10, sy - 1.7, sy + 1.7, Z_M + sz - 3.7, Z_M + sz + 3.7))   # slots: +-2 to tension the belt
    add(moving, "fixed end wall (printed): 6001 + 2 x AXK1226 at the screw, the 48 mm stepper bolts to its top",
        one(fw.clean(), "fixed end wall"), PRINT)
    flw = box(*FLOAT_X, -WALL_Y, WALL_Y, 31.0, DECK_Z[0]).cut(cyl_x(14.0, 130, 160, 0, Z_S)).cut(cyl_x(21.0, FLOAT_X[0] + 0.5, FLOAT_X[1] - 0.5, 0, Z_S))
    add(moving, "floating end wall (printed): 6801", one(flw.clean(), "float wall"), PRINT)

    # --- screw, bearings, collar, pulleys, belt, motor
    add(moving, f"ball screw SFU1204 (cut to {SCREW_X[1] - SCREW_X[0]:.0f})", cyl_x(SCREW_D, *SCREW_X, 0, Z_S), STEEL)
    add(moving, "6001 bearing 12x28x8 (fixed end)", ring_x(12.0, 28.0, *B6001, 0, Z_S), STEEL)
    for i, (a, b) in enumerate(AXK):
        add(moving, f"AXK1226 thrust bearing + 2 AS1226 washers {i + 1}/2", ring_x(12.0, 26.0, a, b, 0, Z_S), STEEL)
    col = ring_x(12.0, 22.0, *COLLAR, 0, Z_S).cut(cyl_y(4.0, 0, 20, sum(COLLAR) / 2, Z_S))
    add(moving, "shaft collar 12 mm (set screw in the ball groove)", one(col.clean(), "collar"), STEEL)
    add(moving, "6801 bearing 12x21x5 (floating end)", ring_x(12.0, 21.0, FLOAT_X[0] + 0.5, FLOAT_X[1] - 0.5, 0, Z_S), STEEL)
    add(moving, "GT2 60T pulley, 12 mm bore (its hub is the outer collar)", pulley(G60, 0, Z_S), ALU)
    add(moving, "GT2 20T pulley, 5 mm bore", pulley(G20, 0, Z_M), ALU)
    to_yz = lambda u, v: V(BELT_X - BELT_W / 2, u, v)                          # noqa: E731
    outer = hull_solid((0, Z_S), R60 - 0.75 + BELT_T, (0, Z_M), R20 - 0.75 + BELT_T, to_yz, V(BELT_W, 0, 0))
    inner = hull_solid((0, Z_S), R60 - 0.75, (0, Z_M), R20 - 0.75, lambda u, v: V(BELT_X - 5, u, v), V(10, 0, 0))
    add(moving, f"GT2 belt {BELT_L:.0f} x 6 mm (C {belt_c():.1f})", one(outer.cut(inner).clean(), "belt"), BLACK)
    p1a, p2a, *_ = hull_pts((0, Z_S), R60 - 0.75 + BELT_T, (0, Z_M), R20 - 0.75 + BELT_T)
    n = ((p1a[0] + p2a[0]) / 2, (p1a[1] + p2a[1]) / 2)
    ln = math.hypot(p2a[0] - p1a[0], p2a[1] - p1a[1])
    nrm = ((p2a[1] - p1a[1]) / ln, -(p2a[0] - p1a[0]) / ln)                    # outward normal of that span
    if nrm[0] < 0:
        nrm = (-nrm[0], -nrm[1])
    ti = (n[0] + 6.1 * nrm[0], n[1] + 6.1 * nrm[1])                            # toothless idler OD 12.2 on the belt back
    idl = cyl_x(12.2, BELT_X - 3, BELT_X + 3, *ti).fuse(cyl_x(18.0, BELT_X - 4, BELT_X - 3, *ti)).fuse(cyl_x(18.0, BELT_X + 3, BELT_X + 4, *ti))
    add(moving, "GT2 toothless idler 5 mm bore (belt tensioner)", one(idl.cut(cyl_x(5.0, -50, 50, *ti)).clean(), "idler"), ALU)
    arm = cyl_x(12.0, BELT_X + 4, WALL_X[0], *ti).cut(cyl_x(5.3, -50, 50, *ti))
    add(moving, "belt tensioner standoff (printed; M5 through a slot in the fixed end wall)", one(arm.clean(), "standoff"), PRINT)
    body, caps = nema17.load(48)
    caps = nema17.trimmed_caps(caps, 24.0)
    place = lambda s: s.rotate(V(0, 0, 0), V(0, 1, 0), -90).translate(V(MOTOR_FACE_X, 0, Z_M))   # noqa: E731
    add(moving, "NEMA 17 48 mm 17HS19 (on top of link A, shaft toward the joint)", place(body), BLACK)
    add(moving, "NEMA 17 end caps", place(caps), STEEL)

    # --- nut, block, truck, rail, chain ends
    nut = cyl_x(NUT_BODY_D, blk0, blk0 + NUT_L - NUT_FL_T, 0, Z_S)
    fl = cyl_x(NUT_FL_D, fl0, blk0, 0, Z_S).cut(box(fl0 - 1, blk0 + 1, -30, 30, Z_S + NUT_FL_FLATS / 2, Z_S + 30))
    fl = fl.cut(box(fl0 - 1, blk0 + 1, -30, 30, Z_S - 30, Z_S - NUT_FL_FLATS / 2))
    add(moving, "ball nut SFU1204 (spare single nut)", one(nut.fuse(fl).cut(cyl_x(SCREW_D, 0, 200, 0, Z_S)).clean(), "nut"), STEEL)
    blk = box(blk0, blk1, -BLOCK_W / 2, BLOCK_W / 2, *BLOCK_Z).cut(cyl_x(NUT_BODY_D, blk0 - 1, blk1 + 1, 0, Z_S))
    add(moving, "nut block (aluminium): nut in it, truck on top, chain ends on its faces", one(blk.clean(), "block"), ALU)
    tc = (blk0 + blk1) / 2
    face = BLOCK_Z[1]
    truck = box(tc - TRUCK_L / 2, tc + TRUCK_L / 2, -TRUCK_W / 2, TRUCK_W / 2, face, face + TRUCK_BODY)
    truck = truck.cut(box(tc - 30, tc + 30, -RAIL_W / 2 - 0.3, RAIL_W / 2 + 0.3, RAIL_Z[0] - 0.3, face + TRUCK_BODY + 1))
    add(moving, "MGN12H truck (upside down, bolted to the block)", one(truck.clean(), "truck"), STEEL)
    rail = box(FL_MIN - 0.5, FLOAT_X[0] - 1, -RAIL_W / 2, RAIL_W / 2, *RAIL_Z)
    add(moving, f"MGN12 rail (cut to {FLOAT_X[0] - 1 - FL_MIN + 0.5:.0f})", rail, STEEL)
    # the adjuster goes on the joint side: there is ~20 mm of bare chain there at the end of travel, while on the
    # idler side anything longer than a plain bracket hits the 12T at full travel
    lug = box(blk0 - 3, blk0, -10, 10, BLOCK_Z[0], Z_S - NUT_FL_FLATS / 2 - 1).fuse(box(blk0 - 6, blk0, -4, 4, R1 - 4, R1 + 4))
    lug = lug.cut(cyl_x(4.4, blk0 - 7, blk0 + 1, 0, R1))
    add(moving, "chain tension lug (steel), -x face of the block, under the nut flange", one(lug.clean(), "lug"), DARK)
    adj = cyl_x(4.0, blk0 - 16, blk0 - 1, 0, R1).fuse(cyl_x(7.0, blk0 - 18, blk0 - 15, 0, R1))
    add(moving, "M4 chain adjuster (eye bolt + 2 nuts)", one(adj.clean(), "adj"), STEEL)
    brk = box(blk1, blk1 + 3, -10, 10, BLOCK_Z[0], Z_S - NUT_BODY_D / 2).fuse(box(blk1, blk1 + 8, -1.5, 1.5, R1 - 3.9, R1 + 3.9))
    add(moving, "chain end bracket (steel), +x face of the block", one(brk.cut(cyl_y(PIN_D + 0.1, -5, 5, blk1 + 5, R1)).clean(), "brk"), DARK)
    pts = chain_points(blk0 - 20, blk1 + 5)
    add(moving, f"#25 roller chain ({len(pts)} pitches)", chain(pts), DARK)
    add(moving, f"#25 {N_IDLER}T idler sprocket, 8 mm bore", sprocket(N_IDLER, X_I, Z_I, 8.0, hub=(15.0, 10.0)), DARK)
    add(moving, "idler axle 8 mm (steel)", cyl_y(8.0, -(WALL_Y + 7), WALL_Y + 7, X_I, Z_I), STEEL)
    for sgn in (1, -1):
        add(moving, f"608 bearing (idler, {'+' if sgn > 0 else '-'}y wall)", ring_y(8.0, 22.0, *sorted((sgn * WALL_Y, sgn * (WALL_Y + 7))), X_I, Z_I), STEEL)
        add(moving, f"6808 bearing (shoulder, {'+' if sgn > 0 else '-'}y)", ring_y(40.0, 52.0, *sorted((sgn * WALL_Y, sgn * (WALL_Y + 7)))), STEEL)
    return moving, fixed, dict(fl0=fl0, blk0=blk0, blk1=blk1, pitches=len(pts))


def to_scene(s):
    """design frame (x out along link A, y joint axis, z up) -> the arm's model frame (link A along -x, up -y)"""
    return s.rotate(V(0, 0, 0), V(1, 0, 0), 90).rotate(V(0, 0, 0), V(0, 1, 0), 180)


def check(moving, fixed):
    """every pair of solid parts that overlap by more than a sliver (the chain, belt and the parts meant to touch are
    listed so a real clash stands out)"""
    parts = [(n, s) for n, s, _ in moving + fixed if not isinstance(s, cq.Compound)]
    hits = []
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            a, b = parts[i][1], parts[j][1]
            ba, bb = a.BoundingBox(), b.BoundingBox()
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                    or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue
            v = a.intersect(b).Volume()
            if v > 1.0:
                hits.append((round(v, 1), parts[i][0], parts[j][0]))
    return hits


def main():
    moving, fixed, info = build()
    os.makedirs(OUT_DIR, exist_ok=True)
    design = os.path.join(OUT_DIR, "chain_drive_J1_design.step")
    write_flat(moving + fixed, design)
    sm = os.path.join(STUDIO, "chain_drive_J1_link_A.step")
    sf = os.path.join(STUDIO, "chain_drive_J1_base.step")
    write_flat([(n, to_scene(s), c) for n, s, c in moving], sm)
    write_flat([(n, to_scene(s), c) for n, s, c in fixed], sf)
    print(f"belt C {belt_c():.2f}  motor axis z {Z_M:.1f}  screw z {Z_S}  idler ({X_I}, {Z_I:.2f})  "
          f"nut flange at {info['fl0']:.1f} (travel {FL_MIN}..{FL_MIN + TRAVEL})  chain {info['pitches']} pitches")
    for v, a, b in check(moving, fixed):
        print(f"  overlap {v:8.1f} mm3: {a}  <->  {b}")
    print("wrote", design, sm, sf, sep="\n  ")


if __name__ == "__main__":
    main()

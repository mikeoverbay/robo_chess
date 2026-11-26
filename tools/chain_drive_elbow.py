#!/usr/bin/env python3
"""
chain_drive_elbow.py - link B (the elbow, J2) on the compact shoulder drive, so the two sections can be seen together
and bent (Mike, 2026-11-20): "Can we draw another section so we can see how these things fit together and if they'll
even bend properly?" ... elbow layout: "Motor on top, belt". Link length stays 200 ("200 be easier to print").

    python chain_drive_elbow.py

Why the elbow is not a copy of the shoulder: the shoulder's inline motor hangs 74 mm behind the joint, which is free
space in the base. Behind the elbow is link A's own screw, nut and end bearing, so link B's motor goes on top and
drives its screw through GT2 20T -> 60T (3:1), all at link B's OUTER end, leaving the elbow end clean to bend.

The elbow joint: link A's walls stop 35 mm short of the elbow; a printed cheek on each side (bolted to link A's wall
through a spacer) carries a boss that runs in link B's 6808 and holds link B's two fixed 25T sprockets - the same
yoke the base gives the shoulder. Link B nests inside the cheeks.

Loads (strength first): elbow worst gravity 5.43 N.m (motor_speed.py's arm model) + ~0.45 for its own motor now out
on link B -> ~5.9 N.m: 232 N of chain pull in all, 116 per chain (about 215 on the tighter one with 100 N
pretension). Screw 0.16 N.m, motor 0.058 N.m through 3:1 (15 % of the 48 mm's 0.40 pull-out). Through 3:1 a full-
current stall would pull 2.4 kN, so the driver current is capped (as in chain_drive.py) to keep the chains under
600 N in all. Printed parts see less than at the shoulder.

Frames: the design STEP has link A level (x along it from the shoulder, y the joint axes, z up) and link B bent to
ELBOW_SHOW. The studio copies are in the arm's model frame: link A on J1, link B straight on J2 (the studio bends it).
"""
import json
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
from chain_drive import (P, cyl_x, cyl_y, box, ring_x, ring_y, one, hull_pts, hull_solid, chain, to_scene,  # noqa: E402
                         pulley, PRINT, STEEL, ALU, DARK, BLACK)

L = 200.0                                      # joint to joint
ELBOW_SHOW = -60.0                             # the design STEP's elbow angle (chess uses -103.4..0)
SWEEP = (4.6, 0.0, -15.0, -30.0, -45.0, -60.0, -75.0, -90.0, -103.4, -104.0)
# link A's elbow end
A_WALL_END = 165.0                             # link A's walls stop here (link B's 6808 hub ring starts at 168)
A_PLATE_END = 175.0                            # its floor and top plate run on to here (carry its floating end wall)
CHEEK_X0 = 125.0                               # cheek + spacer bolt to link A's wall from here
# link B, in its own frame (x from the elbow axis toward J3)
TRAVEL_B = 48.0                                # 108.6 deg on the 25T: elbow -104..+4.6 (chess -103.4..0, home -90)
NUT_AT_B = 0.5
FLOAT_B = (28.0, 34.0)                         # floating end wall (6801), near the elbow
C0_MIN_B = 39.0                                # adjuster eye stays >= 20 out, clear of the 25T's teeth
X_I_B = 122.0                                  # far idlers (shaft wall to inboard upright; clear of the tab at full travel)
COLLAR_B, TW_B, B6001_B = (133.0, 143.0), (147.0, 159.0), (149.0, 157.0)
AXK_B = ((143.0, 147.0), (159.0, 163.0))
G60B = dict(n=60, bore=12.0, teeth=(171.0, 178.0), fl_d=42.0, hub=(163.0, 170.0), hub_d=25.0)   # hub = outer collar
G20B = dict(n=20, bore=5.0, teeth=(171.0, 178.0), fl_d=16.0, hub=(163.0, 170.0), hub_d=13.0)
SCREW_B = (FLOAT_B[0] - 1.0, 179.0)
BELT_X_B, BELT_L_B = 174.5, 166.0              # 166 mm GT2 loop (TYPICAL: check it's sold; 200 puts the motor 18 higher)
MOTOR_L_B, SHAFT_L = 48.0, 24.0
MOTOR_FACE_B = SCREW_B[1] - SHAFT_L            # 155: the body runs back over the nut, x 107..155
Z_S, YC, WALL_Y = C.Z_S, C.YC, C.WALL_Y
TOP_Z, FLOOR_Z = C.TOP_Z, C.FLOOR_Z


def belt_c(length):
    k = (D.R60 - D.R20) ** 2
    s = length - math.pi * (D.R60 + D.R20)
    return (s + math.sqrt(s * s - 8 * k)) / 4


Z_M_B = Z_S + belt_c(BELT_L_B)                 # motor axis: its body clears link B's thrust wall (top 44) by ~1


def path(xa, xb, x_i):
    """C.path with the far idler at x_i"""
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


def link_a():
    """the compact shoulder drive with link A's elbow end made into a yoke for link B (moving with J1, fixed for J2)"""
    mv, fx, info = C.build()
    out = []
    for n, s, c in mv:
        if n.startswith("link A wall"):
            s = s.cut(box(A_WALL_END, 260, -60, 60, -60, 80)).cut(box(100, 260, WALL_Y + 4, 60, -60, 80))
            s = s.cut(box(100, 260, -60, -(WALL_Y + 4), -60, 80))
            s = one(s.clean(), n)
        elif n.startswith("link A floor"):
            s = one(s.cut(box(A_PLATE_END, 260, -60, 60, -60, 80)).clean(), n)
        elif n.startswith("link A top plate"):
            s = s.cut(box(A_PLATE_END, 260, -60, 60, -60, 80)).fuse(box(CHEEK_X0, A_PLATE_END, -(WALL_Y + 12), WALL_Y + 12, *TOP_Z))
            s = one(s.clean(), n)
        out.append((n, s, c))
    bolts = [(L + 16 * math.cos(math.radians(30 + 60 * k)), 16 * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    for side, nm in ((1, "+y"), (-1, "-y")):
        ys = lambda a, b: C.ys(side, a, b)                                          # noqa: E731
        ch = box(CHEEK_X0, A_WALL_END, *ys(WALL_Y + 4, WALL_Y + 8), FLOOR_Z[0], TOP_Z[0])   # spacer on link A's wall
        ch = ch.fuse(box(CHEEK_X0, L, *ys(WALL_Y + 8, WALL_Y + 12), FLOOR_Z[0], TOP_Z[0]))
        ch = ch.fuse(cyl_y(64.0, *ys(WALL_Y + 8, WALL_Y + 12), L, 0))
        ch = ch.fuse(cyl_y(40.0, *ys(YC + D.SPR_T / 2, WALL_Y + 8), L, 0))          # the boss, through link B's 6808
        ch = ch.cut(cyl_y(16.0, -60, 60, L, 0)).cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, C.X_I, C.Z_I))   # link A's far-idler shaft
        for (bx, bz) in bolts:
            ch = ch.cut(cyl_y(3.4, -60, 60, bx, bz))
        out.append((f"elbow cheek {nm} (printed; bolts to link A's wall, its boss carries link B's 25T)",
                    one(ch.clean(), "cheek"), PRINT))
    return out, fx, info


def link_b(nut_at=NUT_AT_B):
    """link B in its own frame (elbow axis at the origin, x toward J3). Returns (parts of link B, the elbow's fixed
    sprockets + bolts, which belong to link A, in link B's frame at 0 deg, and info)."""
    mv, fixed = [], []
    add = lambda lst, name, s, c: lst.append((name, s, c))                         # noqa: E731
    c0 = C0_MIN_B + TRAVEL_B * nut_at
    fl0, fl1 = c0 + 10.0, c0 + 10.0 + C.NUT_FL_T
    body1 = c0 + 10.0 + C.NUT_L
    xb = c0 + 20.0
    pts = C.walk(path(c0 - 12.0, xb, X_I_B))
    eye = pts[0][0]
    ph1 = C.phase(pts, (0.0, 0.0), C.R1)
    bolts = [(16 * math.cos(math.radians(30 + 60 * k)), 16 * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    cw, zi = C.CW, C.Z_I
    for side, nm in ((1, "+y"), (-1, "-y")):
        ys = lambda a, b: C.ys(side, a, b)                                          # noqa: E731
        add(fixed, f"#25 25T plate sprocket 25A25 {nm} (elbow, on the cheek boss)",
            C.sprocket(25, 0, 0, ph1, 20.0, side, bolts=bolts), DARK)
        add(fixed, f"sprocket bolts M3 x 6 {nm} (elbow)",
            cq.Compound.makeCompound([cyl_y(3.0, *ys(YC - 1.5, WALL_Y + 10), bx, bz) for bx, bz in bolts]), BLACK)
        y0, y1 = ys(WALL_Y, WALL_Y + 4)
        r0, r1 = ys(WALL_Y, WALL_Y + 7)
        w = box(0, L, y0, y1, FLOOR_Z[0], TOP_Z[0]).fuse(box(0, 64, y0, y1, C.WALL_DEEP, FLOOR_Z[0]))
        for hx in (0.0, L):
            w = w.fuse(cyl_y(64.0, r0, r1, hx, 0))
        w = w.fuse(cyl_y(20.0, r0, r1, *cw)).fuse(cyl_y(24.0, *ys(WALL_Y, WALL_Y + 12), X_I_B, zi))
        win = box(66, 104, y0 - 1, y1 + 1, 2, 38)
        w = w.cut(win.fillet(8, win.edges("|Y")))
        for hx in (0.0, L):
            w = w.cut(cyl_y(52.0, -60, 60, hx, 0))
        w = w.cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, *cw)).cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, X_I_B, zi))
        add(mv, f"link B wall {nm} (printed)", one(w.clean(), "wall B"), PRINT)
        add(mv, f"6808 bearing (elbow, {nm})", ring_y(40.0, 52.0, r0, r1), STEEL)
        for n_, s_, c_ in C.idler_stack(cw[0], cw[1], side, f"link B wrap idler {nm}", True):
            add(mv, n_, s_, c_)
        for n_, s_, c_ in C.far_idler(X_I_B, zi, side, f"link B far idler {nm}", WALL_Y + 12.0):
            add(mv, n_, s_, c_)
        add(mv, f"#25 roller chain {nm} (link B, {len(pts)} pitches)", chain(pts).translate(V(0, side * YC, 0)), DARK)
        tab = box(c0 - 4.0, xb + 2.5, *ys(YC - 1.5, YC + 1.5), Z_S - 4, Z_S + 4)
        tab = tab.fuse(box(c0 - 4.0, c0, *ys(YC - 3.0, YC + 3.0), Z_S - 4, Z_S + 4))
        tab = tab.cut(cyl_y(2.4, -60, 60, xb, Z_S)).cut(cyl_x(4.4, c0 - 5, c0 + 1, side * YC, Z_S))
        tab = tab.fuse(box(c0, c0 + 10.0, *ys(YC - 1.5, YC + 1.5), Z_S - 8, Z_S + 8))
        add(mv, f"chain tab {nm} (link B, steel)", one(tab.clean(), "tab B"), DARK)
        adj = cyl_x(4.0, eye + 1.5, c0 - 0.5, side * YC, Z_S).fuse(ring_y(2.4, 7.0, *ys(YC - 1.5, YC + 1.5), eye, Z_S))
        adj = adj.fuse(cyl_x(7.0, c0 - 7.0, c0 - 4.0, side * YC, Z_S))
        add(mv, f"M4 chain adjuster {nm} (link B)", one(adj.clean(), "adj B"), STEEL)

    floor = box(64, L - 5, -WALL_Y, WALL_Y, *FLOOR_Z).fuse(box(24, 64, -WALL_Y, WALL_Y, *C.DEEP))
    floor = floor.fuse(box(60, 64, -WALL_Y, WALL_Y, C.DEEP[0], FLOOR_Z[1]))
    for side in (1, -1):
        floor = floor.fuse(C.upright(X_I_B, zi, side))
    floor = floor.cut(cyl_y(C.SHAFT_D + 0.2, -60, 60, X_I_B, zi))
    add(mv, "link B floor (printed; with the far idlers' inboard uprights)", one(floor.clean(), "floor B"), PRINT)
    top = box(0, TW_B[0], -(WALL_Y + 4), WALL_Y + 4, *TOP_Z).cut(box(MOTOR_FACE_B - MOTOR_L_B - 3, TW_B[0] + 1, -23, 23, 40, 50))
    add(mv, "link B top plate (printed; open under the motor)", one(top.clean(), "top B"), PRINT)
    fw = box(*FLOAT_B, -16, 16, -2.0, TOP_Z[0]).cut(cyl_x(14.0, FLOAT_B[0] - 1, FLOAT_B[1] + 1, 0, Z_S))   # clears the wrap collars
    fw = fw.cut(cyl_x(21.0, FLOAT_B[0] + 0.5, FLOAT_B[1] - 0.5, 0, Z_S))
    add(mv, "link B floating end wall (printed): 6801", one(fw.clean(), "float B"), PRINT)
    tw = box(*TW_B, -WALL_Y, WALL_Y, FLOOR_Z[1], TOP_Z[0])
    tw = tw.fuse(box(MOTOR_FACE_B, TW_B[1], -21.15, 21.15, TOP_Z[0] - 1, Z_M_B + 21.15))      # motor plate
    tw = tw.cut(cyl_x(14.0, TW_B[0] - 1, TW_B[1] + 1, 0, Z_S)).cut(cyl_x(28.0, *B6001_B, 0, Z_S))
    tw = tw.cut(cyl_x(22.5, MOTOR_FACE_B - 1, TW_B[1] + 1, 0, Z_M_B))
    for sy in (-15.5, 15.5):
        for sz in (-15.5, 15.5):
            tw = tw.cut(box(MOTOR_FACE_B - 1, TW_B[1] + 1, sy - 1.7, sy + 1.7, Z_M_B + sz - 3.7, Z_M_B + sz + 3.7))  # +-2 slots
    add(mv, "link B thrust wall (printed, 12 thick): 6001 + 2 x AXK1226; its top is the plate the 48 mm motor bolts to",
        one(tw.clean(), "thrust wall B"), PRINT)

    add(mv, f"ball screw SFU1204 (link B, cut to {SCREW_B[1] - SCREW_B[0]:.0f})", cyl_x(C.SCREW_D, *SCREW_B, 0, Z_S), STEEL)
    add(mv, "6001 bearing 12x28x8 (link B thrust wall)", ring_x(12.0, 28.0, *B6001_B, 0, Z_S), STEEL)
    for i, (a, b) in enumerate(AXK_B):
        add(mv, f"AXK1226 thrust bearing + 2 AS1226 washers {i + 1}/2 (link B)", ring_x(12.0, 26.0, a, b, 0, Z_S), STEEL)
    col = ring_x(12.0, 22.0, *COLLAR_B, 0, Z_S).cut(cyl_y(4.0, 0, 20, sum(COLLAR_B) / 2, Z_S))
    add(mv, "shaft collar 12 mm (link B, set screw in the ball groove)", one(col.clean(), "collar B"), STEEL)
    add(mv, "6801 bearing 12x21x5 (link B floating end)", ring_x(12.0, 21.0, FLOAT_B[0] + 0.5, FLOAT_B[1] - 0.5, 0, Z_S), STEEL)
    add(mv, "wrap idler shaft 5 mm (link B, hardened rod through both walls; the inboard collars keep it in)",
        cyl_y(C.SHAFT_D, -(WALL_Y + 7), WALL_Y + 7, *cw), STEEL)
    add(mv, "GT2 60T pulley, 12 mm bore (link B screw; its hub is the outer collar)", pulley_b(G60B, 0, Z_S), ALU)
    add(mv, "GT2 20T pulley, 5 mm bore (link B motor)", pulley_b(G20B, 0, Z_M_B), ALU)
    to_yz = lambda u, v: V(BELT_X_B - D.BELT_W / 2, u, v)                         # noqa: E731
    outer = hull_solid((0, Z_S), D.R60 - 0.75 + D.BELT_T, (0, Z_M_B), D.R20 - 0.75 + D.BELT_T, to_yz, V(D.BELT_W, 0, 0))
    inner = hull_solid((0, Z_S), D.R60 - 0.75, (0, Z_M_B), D.R20 - 0.75, lambda u, v: V(BELT_X_B - 5, u, v), V(10, 0, 0))
    add(mv, f"GT2 belt {BELT_L_B:.0f} x 6 mm (link B; TYPICAL length, C {belt_c(BELT_L_B):.1f})", one(outer.cut(inner).clean(), "belt"), BLACK)
    # no idler tensioner: on a 41 mm centre it hits both pulleys. The motor's +-2 mm slots tension the belt.
    body, caps = nema17.load(MOTOR_L_B)
    caps = nema17.trimmed_caps(caps, SHAFT_L)
    place = lambda s: s.rotate(V(0, 0, 0), V(0, 1, 0), 90).translate(V(MOTOR_FACE_B, 0, Z_M_B))   # noqa: E731
    add(mv, "NEMA 17 48 mm 17HS19 (link B, on top over the nut, shaft out toward J3)", place(body), BLACK)
    add(mv, "NEMA 17 end caps (link B)", place(caps), STEEL)

    nut = cyl_x(C.NUT_BODY_D, fl1, body1, 0, Z_S)
    fl = cyl_x(C.NUT_FL_D, fl0, fl1, 0, Z_S).cut(box(fl0 - 1, fl1 + 1, -30, 30, Z_S + C.NUT_FL_FLATS / 2, Z_S + 30))
    fl = fl.cut(box(fl0 - 1, fl1 + 1, -30, 30, Z_S - 30, Z_S - C.NUT_FL_FLATS / 2))
    add(mv, "ball nut SFU1204 (link B)", one(nut.fuse(fl).cut(cyl_x(C.SCREW_D, 0, 300, 0, Z_S)).clean(), "nut B"), STEEL)
    cp = box(c0, c0 + 10.0, -25.5, 25.5, Z_S - 16, Z_S + 16).cut(cyl_x(13.0, c0 - 1, c0 + 11, 0, Z_S))
    add(mv, "nut carrier plate (link B, aluminium)", one(cp.clean(), "carrier B"), ALU)
    return mv, fixed, dict(c0=c0, eye=eye, pitches=len(pts), motor_top=Z_M_B + 21.15, c=belt_c(BELT_L_B))


def pulley_b(g, y, z):
    """chain_drive.pulley only bores x -100..100: build it 200 further in and slide it out"""
    sh = 200.0
    g2 = dict(g, teeth=(g["teeth"][0] - sh, g["teeth"][1] - sh), hub=(g["hub"][0] - sh, g["hub"][1] - sh))
    return pulley(g2, y, z).translate(V(sh, 0, 0))


def place_b(s, theta):
    """link B part from its own frame to link A's (elbow at x L), bent theta deg (negative = down, as the studio's J2)"""
    return s.translate(V(L, 0, 0)).rotate(V(L, 0, 0), V(L, 1, 0), -theta)


def overlap(a, b):
    A, B = a.BoundingBox(), b.BoundingBox()
    if A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin:
        return 0.0
    return a.intersect(b).Volume()


def sweep(a_parts, b_parts):
    """link B against link A (+ the elbow's fixed sprockets) at each elbow angle; the chain wrapped on its own 25T
    and the boss inside the 6808 are meant to touch"""
    hits = []
    A = [(n, s) for n, s, _ in a_parts if not isinstance(s, cq.Compound)]
    for th in SWEEP:
        for n, s, _ in b_parts:
            if isinstance(s, cq.Compound) or "roller chain" in n:
                continue
            r = place_b(s, th)
            for m, t in A:
                if "6808 bearing (elbow" in n and m.startswith("elbow cheek"):
                    continue
                v = overlap(r, t)
                if v > 1.0:
                    hits.append((th, round(v, 1), n, m))
    return hits


def inside(parts):
    """overlaps between one link's own parts (chains on their own sprockets show as small numbers)"""
    sol = [(n, s) for n, s, _ in parts if not isinstance(s, cq.Compound)]
    out = []
    for i in range(len(sol)):
        for j in range(i + 1, len(sol)):
            v = overlap(sol[i][1], sol[j][1])
            if v > 1.0:
                out.append((round(v, 1), sol[i][0], sol[j][0]))
    return out


def main():
    a_mv, j1_fixed, info_a = link_a()
    b_mv, b_fixed, info_b = link_b()
    a_all = a_mv + [(n, place_b(s, 0.0), c) for n, s, c in b_fixed]          # the elbow sprockets ride on link A
    print(f"link B: carrier at {info_b['c0']:.1f}, chain {info_b['pitches']} pitches each, belt C {info_b['c']:.2f}, "
          f"motor top z {info_b['motor_top']:.1f} (link A's top {TOP_Z[1]})")
    for v, p, q in inside(b_mv + [(n, s, c) for n, s, c in b_fixed]):
        print(f"  link B itself  {v:8.1f} mm3: {p[:60]}  <->  {q[:60]}")
    for th, v, p, q in sweep(a_all, b_mv):
        print(f"  elbow {th:6.1f}  {v:8.1f} mm3: {p[:60]}  <->  {q[:60]}")
    if "--ends" in sys.argv:                                   # link B's nut at both ends of its travel
        key = ("ball nut", "carrier", "chain tab", "adjuster", "roller chain")
        for at in (0.0, 1.0):
            mv_e, fx_e, inf = link_b(at)
            print(f"link B nut at {at}: carrier {inf['c0']:.1f}, eye {inf['eye']:.1f}")
            for v, p, q in inside(mv_e + fx_e):
                if any(k in p or k in q for k in key):
                    print(f"     {v:8.1f} mm3: {p[:55]}  <->  {q[:55]}")
    out_dir = os.path.join(C.ROOT, "chain_drive", "step")
    design = os.path.join(out_dir, "chain_drive_J1_J2_design.step")
    write_flat(a_all + j1_fixed + [(n, place_b(s, ELBOW_SHOW), c) for n, s, c in b_mv], design)
    sa = os.path.join(C.STUDIO, "chain_drive_J1c_link_A.step")
    sb = os.path.join(C.STUDIO, "chain_drive_J2_link_B.step")
    write_flat([(n, to_scene(s), c) for n, s, c in a_all], sa)
    write_flat([(n, to_scene(place_b(s, 0.0)), c) for n, s, c in b_mv], sb)
    print("wrote", design, sa, sb, sep="\n  ")


if __name__ == "__main__":
    main()

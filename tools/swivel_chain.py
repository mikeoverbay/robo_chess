#!/usr/bin/env python3
"""
swivel_chain.py - the arm's swivel (J0) driven by chain, replacing the 4:1 plastic bevel pair (Mike 2026-11-20: "I'd
really like to try to keep a chain tensioner on it and get rid of that bevel gear ... It's plastic. It's going to give
out" / "we'll go with the cut gear, cut sprocket, and we'll also use a reducer. And I want the shaft that the reducer
is on, that the chain sprocket is on ... kicked at an angle in the back so that we can have our belt center line
straight across in line with the back of the table ... like a triangle shape" / "reduce ... having so much sticking
out the back" / "Yes, model it"). Layout agreed on chain_drive/swivel_chain_layout.png. swivel_base.py (the main
session's bevel swivel) is untouched; this is the replacement for review.

Swivel frame (as swivel_base.py): Z up from the wood, X toward the board, yaw axis = Z. Same bearings and heights:
two 6808s at z 18 and 61 in a tower, the hollow D40 spindle, the turntable's top at z 89 (so the arm stands where it
does). New: under the turntable his CNC-cut 90T #25 sprocket (z 73.5..76.5); chain back at an angle to a jackshaft
(10T #25 at chain level, his 60T GT2 8 mm bore pulley one level down, on a tower with 2 x 608); a GT2 belt straight across the back
to a NEMA 17 standing up (20T). The 608s sit in an eccentric cam tube (3 mm throw) in the jackshaft's tower: twist it
to tension the chain, then a split clamp's two #4 screws lock it (no idler). 9:1 x 3:1 = 27:1.

    python swivel_chain.py           STEP files in complete assembly/, the clash check, the studio entries
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import arm_base as AB  # noqa: E402
import chain_drive as D  # noqa: E402
import chain_drive_arm as A  # noqa: E402
import chain_drive_compact as C  # noqa: E402
import nema17  # noqa: E402
from chain_drive import ALU, BLACK, DARK, PRINT, STEEL, one, pulley  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402
from swivel_base import to_design  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "complete assembly")
STUDIOS = [os.path.join(HERE, "viewer_sessions", s, "scene.json") for s in ("gearbox", "chaindrive")]
TAG = "(swivel chain)"

P = D.P
FIT = 0.1
N_BIG, N_SMALL, T_JACK, T_MOTOR = 90, 10, 60, 20
LINKS = 96                                       # chain: 96 links; the idler sets the tension
Y_WOOD = AB.Y_CUT + AB.FOOT_T + 89.0             # the turntable top at z 89, as the bevel swivel: J1 stays put
BASE_T = 6.0
Z_B1, Z_B2, B_W = 18.0, 61.0, 7.0                # the two 6808s (40 x 52 x 7), as before
TOWER_OD, TOWER_TOP, SHOULDER_D = 60.0, 68.0, 47.0
SPINDLE_OD, SPINDLE_ID = 40.0, 30.0
COLLAR = (13.0, 18.0, 46.0)                      # under the lower bearing's inner race: z, z, OD
PLATE = (77.0, 89.0, 2 * (54.0 + 1.5 * math.cos(math.atan2(72, 18))))   # the turntable plate: the arm foot's D108.73
Z_CHAIN = 75.0                                   # chain / sprocket mid-plane; the sprocket 73.6..76.4
BOSS_R, BOSS_N = 44.0, 6                         # the sprocket's bolts up into the turntable (spacer bosses)
Z_BELT = (61.0, 68.0)                            # GT2 teeth, both pulleys
Z_MOTOR_FACE = 54.0                              # the motor stands on the base plate, its face up
# the jackshaft's eccentric (Mike 2026-11-21: "an eccentric cam tube ... that our bearings are in, so we can twist it
# and take up the slop ... get rid of the idler wheel and the post ... a wraparound clamp with at least 2 number 4
# screws that clamps the OD of our cam"; "the wall thicker ... 3 millimeter thick gussets all the way around")
ECC = 3.0                                        # the cam's throw: the chain's centre distance +-3 (about one link)
CAM_OD, CAM_CLEAR, CAM_Z0 = 34.0, 0.2, 2.0       # the cam tube turns in the tower's bore, on its floor at z 2
CAM_FL_D, CAM_FL_T = 44.0, 5.0                   # its flange on the tower's top: 4 radial holes for a 3 mm rod
J_B1, J_B2 = 3.0, 45.5                           # the 608s (bottoms) in the cam; the upper one's top 52.5, under the pulley's hub
CAM_TOP = J_B2 + 7.0
TJ_OD, TJ_TOP = 48.0, CAM_TOP - CAM_FL_T         # the jackshaft tower: wall 6.9 round the cam (was a D30 post)
GUS_T, GUS_N, GUS_R, GUS_H = 3.0, 8, 12.0, 26.0  # gussets round the tower's foot
CLAMP_W, CLAMP_D, CLAMP_Z0, SLIT = 22.0, 12.0, 14.0, 1.5   # the split clamp's ears, the slit from z 14 to the top
N4 = dict(clear=3.0, pilot=2.3, head_d=6.0, head_h=2.4, d=2.84, L=19.05, z=(24.0, 40.0))   # his #4 x 3/4 sheet-metal screws
COVER_IN, COVER_T, COVER_TOP = 85.0, 4.0, 89.0  # the cover: walls to its roof, roof 85..89, open round the turntable
COVER_OPEN = PLATE[2] + 3.0                      # 1.5 round the turntable
WALL = 3.0
# the yoke (Mike 2026-11-21: "a yoke with a base that's strong enough it won't flex ... a platform with fins, disc on
# the top, fork on that, base mounted onto the output"): the arm centred over the turntable, J1's bosses in its cheeks
Z_J1 = Y_WOOD                                    # J1's axis (design y 0) above the wood
BOSS_PTS = [(15.94, 1.33), (-1.33, 15.94), (-15.94, -1.33), (1.33, -15.94)]   # the 25T's M3s, studio (x, y) about J1
# the keyed shoulder boss (the main session's chain_base.py, adopted 2026-11-21): D40 shank in link A's 6808 from the
# 25T's face, a D43 collar on the 6808's inner ring, a 44 square key set as a diamond in the cheek (it carries J1's
# torque), a D80 flange outside; 4 x M3 x 30 from the flange into the tapped 25T, 4 x M3 x 8 flange -> cheek inserts
SPR_OUT = C.YC + D.SPR_T / 2                     # 28.4: the 25T's outer face = the boss's inner end
SHANK_D, COLLAR_Y, COLLAR_D = 40.0, (40.0, 44.0), 43.0
CHEEK_IN, CHEEK_T, CHEEK_R = 44.0, 10.0, 42.0    # the cheeks |y| 44..54, round top; link A stays inside |y| 44 near J1
KEY_AF, KEY_CLEAR = 44.0, 0.2
FLANGE_Y, FLANGE_D = (54.0, 58.0), 80.0
FL_SCREW_R, INSERT_HOLE, INSERT_DEPTH = 35.0, 4.2, 4.5   # M3 x 4 x 5 inserts on the diamond's flats
SPR_SCREW_L, FL_SCREW_L, CBORE_D, CBORE_T = 30.0, 8.0, 6.5, 3.5
PLAT = (PLATE[1], PLATE[1] + 6.0)                # the platform's foot on the turntable
DISC = (144.0, 150.0)                            # its top disc
DISC_D = 156.0
FORK = (150.0, 160.0)                            # the fork's base plate (link A's motor swings down to z 186 behind)
FORK_X = 42.0
FIN_H = 55.0                                     # the cheeks' outer fins: up from the plate, under the bosses' flanges
FIN_T, FIN_N = 6.0, 6
TUBE = (44.0, 30.0)                              # the platform's centre tube: the wires come up the spindle
FOOT_R, FORK_R = 46.0, 32.0                      # M4s: platform -> turntable (between the fins), fork -> disc (on them)
# the claw's servo on a shelf off the fork's -y cheek (Mike 2026-11-21: "move it up and add a bracket to mount it and
# make it part of the base" / "a rectangular plate with a rectangular hole in it and a couple of gussets on it, so it
# can't move up and down from flex when the arm is moving"): out there link A never comes (it stays inside |y| 51)
SV_L, SV_W, SV_H, SV_TAB_L, SV_TAB_T, SV_TAB_Z = 40.7, 19.7, 37.0, 54.0, 2.5, 26.5   # MG996R: case, tabs (underside up)
SV_TURRET, SV_SPLINE, SV_SHAFT_X = (12.0, 3.0), (5.8, 4.0), 10.2                     # the shaft 10.2 from the case's end
SV_HOLES = (49.5, 10.0)                          # its tab screws
HORN_R, HORN_T, HORN_W, HORN_HUB = 15.0, 2.5, 6.0, 9.0
SHELF_Z, SHELF_T = 205.0, 5.0                    # the shelf's top (the tabs sit on it)
SV_C = (0.0, -76.0)                              # the servo's centre (x, y): its long side along x, clear of the fins
SHELF_X, SHELF_Y = (-50.0, 34.0), (-(CHEEK_IN + CHEEK_T) + 1.0, -98.0)
SH_GUS_X, SH_GUS_T, SH_GUS_LOW = (-34.0, 30.0), 4.0, 163.0
FIT_POST = (-48.0, -40.0)                        # the Bowden's PC4-M10 fitting: a post behind, the cable straight back
M10_TAP = 8.5
# the Bowden's J0 guide ring (the main session's route, 2026-11-21: studio [120, -35, 108]): on a post off the shelf's
# back, the post inboard of the tube (the tube comes up from the fitting on the outboard side), clear of link A (|y| 51)
RING_AT = (-120.0, AB.Z_MID - 108.0, Y_WOOD + 35.0)
RING_ID, RING_T = 8.0, 2.4                       # a short round ring (a torus): the tube's tangent swings 49.5 deg
RING_AXIS = (-0.443, 0.136, 0.886)               # through it as link A moves (main's bowden_route.py; studio [0.443, -0.886, -0.136])
RING_POST_Y, RING_POST_W = -67.0, 8.0            # the post's centre y, its square section
RING_ARM_Z = (195.0, SHELF_Z)                    # the arm back from the shelf, 10 deep
SV_GREY = cq.Color(0.54, 0.557, 0.584)
LIGHT = cq.Color(0.90, 0.91, 0.93)               # Mike: the cam tube and its tower lighter ("make that base lighter color")
MW_T, MW_GUS, MW_GUS_X = 7.0, (3.0, 14.0, 34.0), (-18.0, 18.0)   # the motor's wall; its gussets (thick, out, up), x off M
TOWER_GUS = (8, 3.0, 14.0, 30.0)                 # the bearing tower's gussets (Mike: "put gussets around the part I have marked")
# hold-down screws (Mike 2026-11-24, relayed by the arm session: "need to have screws to hold it down. Yes, we'll need
# screws"): #8 x 1/2" flat-head wood screws, as the park stand's #8s, countersunk flush in the 6 mm plate. The board's
# 3 deep pocket leaves 9 of its 12 mm wood, so 6.7 of thread goes in. The lower 6808 and the collar go in from below, so
# the base is screwed down after J0 is built: inside the cover the 90T (tips r 93), the motor plate and the belt leave a
# screwdriver only two spots at the back (cover off); the front gets lugs outside the cover's wall. Not straight at the
# board (Robo chessboard design 2026-11-24: the 0 deg lug ran under the main board's back edge at X 107; under Z 27
# nothing may pass X 106, so a lug needs |angle| >= 24 deg): four, at +-30 and +-75.
HD = dict(clear=4.5, head_d=8.4, d=4.17, L=12.7, half=41.0)    # #8: clearance, head, shank, length, half the 82 deg csk
HD_BACK = [(-104.0, 28.0), (-104.0, -18.0)]      # inside the cover: behind the belt, clear of the wire slot and gussets
HD_LUG_A, HD_LUG_OUT, HD_LUG_W = (-75.0, -30.0, 30.0, 75.0), 8.0, 16.0   # the lugs: directions, hole past the wall, width


def pd(n):
    return P / math.sin(math.pi / n)


def gt2(n):
    return 2.0 * n / math.pi


def cyl(d, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def ring(d1, d2, z0, z1, x=0.0, y=0.0):
    """a ring between the two diameters (either order)"""
    do, di = max(d1, d2), min(d1, d2)
    return cyl(do, z0, z1, x, y).cut(cyl(di, z0 - 1, z1 + 1, x, y))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def prism(pts, z0, z1):
    w = cq.Wire.makePolygon([V(x, y, z0) for x, y in pts] + [V(pts[0][0], pts[0][1], z0)])
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), V(0, 0, z1 - z0))


def hull2(points):
    pts = sorted(set(points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def offset(poly, d):
    """a convex polygon grown by d (rounded by sampling)"""
    pts = []
    for x, y in poly:
        pts += [(x + d * math.cos(math.radians(a)), y + d * math.sin(math.radians(a))) for a in range(0, 360, 10)]
    return hull2(pts)


# ---------------------------------------------------------------- layout (top view, swivel frame)
def chain_c():
    """centre distance for LINKS links on N_BIG / N_SMALL (the idler just touching)"""
    a = 2.0 / P
    b = (N_BIG + N_SMALL) / 2.0 - LINKS
    k = ((N_BIG - N_SMALL) / (2 * math.pi)) ** 2 * P
    return (-b + math.sqrt(b * b - 4 * a * k)) / (2 * a)


def belt_c(length):
    R, r = gt2(T_JACK) / 2, gt2(T_MOTOR) / 2
    # L = 2C + pi (R + r) + (R - r)^2 / C
    b = math.pi * (R + r) - length
    return (-b + math.sqrt(b * b - 8 * (R - r) ** 2)) / 4


BELT_L = 400.0                                   # a common GT2 loop (200 teeth); the motor's slots take up the rest
X_BACK = -80.0
CC = chain_c()
J = (X_BACK, math.sqrt(CC ** 2 - X_BACK ** 2))
M = (X_BACK, J[1] - belt_c(BELT_L))
R_BIG, R_SMALL = pd(N_BIG) / 2, pd(N_SMALL) / 2


def tangent_pts(c1, r1, c2, r2, s):
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    t = math.atan2(dy, dx) + s * math.acos((r1 - r2) / d)
    return (c1[0] + r1 * math.cos(t), c1[1] + r1 * math.sin(t)), (c2[0] + r2 * math.cos(t), c2[1] + r2 * math.sin(t)), t


def chain_loop():
    """the chain's closed path (pitch line), counter-clockwise round the big sprocket, as dense (x, y) points"""
    a1, b1, t1 = tangent_pts((0, 0), R_BIG, J, R_SMALL, 1)
    a2, b2, t2 = tangent_pts((0, 0), R_BIG, J, R_SMALL, -1)
    pts = []

    def arc(c, r, f, t, step=0.02):
        n = max(2, int(abs(t - f) * r / step))
        pts.extend((c[0] + r * math.cos(f + (t - f) * k / n), c[1] + r * math.sin(f + (t - f) * k / n)) for k in range(n))

    def line(p, q, step=0.02):
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
        pts.extend((p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) for k in range(n))
    # t1 = a + b, t2 = a - b (a: the direction to J): CCW round the big one the long way (away from J), out along one
    # strand, CCW round the small one on its far side, back along the other
    arc((0, 0), R_BIG, t1, t2 + 2 * math.pi)
    line(a2, b2)
    arc(J, R_SMALL, t2, t1)
    line(b1, a1)
    return pts


def pitch_points(dense, n):
    """n points round the closed path, each the chord P from the last"""
    out = [dense[0]]
    i = 0
    for _ in range(n - 1):
        x0, y0 = out[-1]
        while i < len(dense) - 1 and math.hypot(dense[i][0] - x0, dense[i][1] - y0) < P:
            i += 1
        (xa, ya), (xb, yb) = dense[i - 1], dense[i]
        da, db = math.hypot(xa - x0, ya - y0), math.hypot(xb - x0, yb - y0)
        f = (P - da) / (db - da) if db != da else 0
        out.append((xa + (xb - xa) * f, ya + (yb - ya) * f))
    return out


# ---------------------------------------------------------------- parts
def sprocket(n, x, y, z_mid, bore, phase_xy=0.0, hub=None):
    """C.sprocket (built about y in its x-z plane) turned to lie flat in the swivel's x-y plane"""
    s = C.sprocket(n, 0.0, 0.0, -phase_xy, bore, 1, hub=hub).translate(V(0, -C.YC, 0))
    s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90)                     # its y axis -> z; its (x, z) -> (x, -y)
    return s.translate(V(x, y, z_mid))


def gt2_pulley(n, x, y, bore, hub_below=True):
    """chain_drive.pulley (built along x) turned to stand on z, its teeth at Z_BELT"""
    t0, t1 = Z_BELT
    hub = (t0 - 8.0, t0 - 1.0) if hub_below else (t1 + 1.0, t1 + 8.0)
    g = dict(n=n, bore=bore, teeth=(t0, t1), fl_d=gt2(n) + 4.0, hub=hub, hub_d=16.0 if n <= 20 else 25.0)
    p = pulley(g, 0.0, 0.0)
    return p.rotate(V(0, 0, 0), V(0, 1, 0), -90).translate(V(x, y, 0))   # x -> z


def cyl_y(d, y0, y1, x, z):
    return cq.Solid.makeCylinder(d / 2, y1 - y0, V(x, y0, z), V(0, 1, 0))


def xz_prism(pts, y0, y1):
    """a polygon in the x-z plane, from y0 to y1"""
    w = cq.Wire.makePolygon([V(x, y0, z) for x, z in pts] + [V(pts[0][0], y0, pts[0][1])])
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), V(0, y1 - y0, 0))


def yz_prism(pts, x0, x1):
    w = cq.Wire.makePolygon([V(x0, y, z) for y, z in pts] + [V(x0, pts[0][0], pts[0][1])])
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(w), V(x1 - x0, 0, 0))


def hex_y(af, y0, y1, x, z):
    r = af / math.sqrt(3)
    return xz_prism([(x + r * math.cos(math.radians(30 + 60 * k)), z + r * math.sin(math.radians(30 + 60 * k))) for k in range(6)], y0, y1)


def diamond(af, y0, y1, z=None):
    """a square af across flats on J1's axis, corners up / down / fore / aft"""
    zc = Z_J1 if z is None else z
    return box(-af / 2, af / 2, y0, y1, -af / 2, af / 2).rotate(V(0, 0, 0), V(0, 1, 0), 45).translate(V(0, 0, zc))


def fl_screw_pts():
    return [(FL_SCREW_R * math.cos(math.radians(a)), Z_J1 + FL_SCREW_R * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]


def spr_pts():
    """the 25T's four M3s, swivel (x, z)"""
    return [(-bx, Z_J1 - by) for bx, by in BOSS_PTS]


def ys(side, a, b):
    return tuple(sorted((side * a, side * b)))


def boss(side):
    """one keyed shoulder boss (print it flange down)"""
    b = cyl_y(SHANK_D, *ys(side, SPR_OUT, COLLAR_Y[0]), 0, Z_J1).fuse(cyl_y(COLLAR_D, *ys(side, *COLLAR_Y), 0, Z_J1))
    b = b.fuse(diamond(KEY_AF, *ys(side, CHEEK_IN, CHEEK_IN + CHEEK_T))).fuse(cyl_y(FLANGE_D, *ys(side, *FLANGE_Y), 0, Z_J1))
    b = b.cut(cyl_y(16.0, -100, 100, 0, Z_J1))
    for x, z in spr_pts():
        b = b.cut(cyl_y(3.4, -100, 100, x, z)).cut(cyl_y(CBORE_D, *ys(side, FLANGE_Y[1] - CBORE_T, FLANGE_Y[1] + 1), x, z))
    for x, z in fl_screw_pts():
        b = b.cut(cyl_y(3.4, *ys(side, FLANGE_Y[0] - 1, FLANGE_Y[1] + 1), x, z))
    return one(b.clean(), "boss")


def boss_hardware(side):
    seat = FLANGE_Y[1] - CBORE_T
    sc = [cyl_y(3.0, *ys(side, seat - SPR_SCREW_L, seat), x, z).fuse(cyl_y(5.5, *ys(side, seat, seat + 3.0), x, z)) for x, z in spr_pts()]
    fs = [cyl_y(3.0, *ys(side, FLANGE_Y[1] - FL_SCREW_L, FLANGE_Y[1]), x, z).fuse(cyl_y(5.5, *ys(side, FLANGE_Y[1], FLANGE_Y[1] + 3.0), x, z))
          for x, z in fl_screw_pts()]
    yo = CHEEK_IN + CHEEK_T
    ins = [cyl_y(4.6, *ys(side, yo - 4.0, yo), x, z).cut(cyl_y(3.0, -100, 100, x, z)) for x, z in fl_screw_pts()]
    return cq.Compound.makeCompound(sc), cq.Compound.makeCompound(fs), cq.Compound.makeCompound(ins)


def sv_z0():
    return SHELF_Z - SV_TAB_Z                    # the servo case's bottom


def sv_shaft():
    return (SV_C[0] - SV_L / 2 + SV_SHAFT_X, SV_C[1])   # the shaft at the case's back (-x) end, nearest the fitting


def horn_z():
    z_spl = sv_z0() + SV_H + SV_TURRET[1] + SV_SPLINE[1]
    return (z_spl - 1.5, z_spl - 1.5 + HORN_T)


def servo_shelf():
    """the shelf (printed with the fork): a plate off the cheek, two gussets under it, the fitting's post behind"""
    sh = box(*SHELF_X, SHELF_Y[1], SHELF_Y[0], SHELF_Z - SHELF_T, SHELF_Z)
    for gx in SH_GUS_X:
        tri = [(SHELF_Y[0], SHELF_Z - SHELF_T + 0.01), (SHELF_Y[1] + 3.0, SHELF_Z - SHELF_T + 0.01), (SHELF_Y[0], SH_GUS_LOW)]
        sh = sh.fuse(yz_prism(tri, gx - SH_GUS_T / 2, gx + SH_GUS_T / 2))
    hz = sum(horn_z()) / 2
    tip_y = sv_shaft()[1] - HORN_R
    sh = sh.fuse(box(*FIT_POST, tip_y - 7.0, tip_y + 7.0, SHELF_Z - 0.01, hz + 8.0))
    # the ring's post: an arm back from the shelf, a post up, a bridge out to the ring
    rx, ry, rz = RING_AT
    w = RING_POST_W / 2
    sh = sh.fuse(box(rx - w, SHELF_X[0] + 0.01, RING_POST_Y - w, RING_POST_Y + w, *RING_ARM_Z))
    sh = sh.fuse(box(rx - w, rx + w, RING_POST_Y - w, RING_POST_Y + w, RING_ARM_Z[0], rz + w))
    u = ring_dir()
    R = RING_ID / 2 + RING_T / 2
    sh = sh.fuse(cq.Solid.makeTorus(R, RING_T / 2, V(rx, ry, rz), V(*u)))
    yv = [(1.0 if k == 1 else 0.0) - u[1] * u[k] for k in range(3)]   # the ring's own +y direction, in its plane
    n = math.sqrt(sum(c * c for c in yv))
    edge = [(rx, ry, rz)[k] + R * yv[k] / n for k in range(3)]       # its tube's centre on the post's side
    sh = sh.fuse(box(edge[0] - 2.5, edge[0] + 2.5, edge[1] - 0.6, RING_POST_Y, edge[2] - 2.5, edge[2] + 2.5))
    return sh


def ring_dir():
    """the ring's axis (swivel frame), unit"""
    n = math.sqrt(sum(c * c for c in RING_AXIS))
    return [c / n for c in RING_AXIS]


def servo_shelf_cuts():
    cx, cy = SV_C
    c = box(cx - SV_L / 2 - 0.3, cx + SV_L / 2 + 0.3, cy - SV_W / 2 - 0.3, cy + SV_W / 2 + 0.3, SHELF_Z - SHELF_T - 1, SHELF_Z + 1)
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = c.fuse(cyl(2.5, SHELF_Z - SHELF_T - 1, SHELF_Z + 1, cx + sx * SV_HOLES[0] / 2, cy + sy * SV_HOLES[1] / 2))
    hz = sum(horn_z()) / 2
    tip_y = sv_shaft()[1] - HORN_R
    c = c.fuse(cq.Solid.makeCylinder(M10_TAP / 2, 20, V(FIT_POST[0] - 1, tip_y, hz), V(1, 0, 0)))
    return c


def servo_parts():
    """the MG996R (case, tabs, turret, spline), its one-arm horn (pointing out, -y, at rest: the cable runs straight
    back from its tip to the fitting) and the PC4-M10 fitting"""
    cx, cy = SV_C
    z0 = sv_z0()
    sv = box(cx - SV_L / 2, cx + SV_L / 2, cy - SV_W / 2, cy + SV_W / 2, z0, z0 + SV_H)
    sv = sv.fuse(box(cx - SV_TAB_L / 2, cx + SV_TAB_L / 2, cy - SV_W / 2, cy + SV_W / 2, SHELF_Z, SHELF_Z + SV_TAB_T))
    for sx in (-1, 1):
        for sy in (-1, 1):
            sv = sv.cut(cyl(4.4, SHELF_Z - 1, SHELF_Z + SV_TAB_T + 1, cx + sx * SV_HOLES[0] / 2, cy + sy * SV_HOLES[1] / 2))
    hx, hy = sv_shaft()
    sv = sv.fuse(cyl(SV_TURRET[0], z0 + SV_H - 0.01, z0 + SV_H + SV_TURRET[1], hx, hy))
    sv = sv.fuse(cyl(SV_SPLINE[0], z0 + SV_H + SV_TURRET[1] - 0.01, z0 + SV_H + SV_TURRET[1] + SV_SPLINE[1], hx, hy))
    h0, h1 = horn_z()
    hn = cyl(HORN_HUB, h0, h1, hx, hy).fuse(box(hx - HORN_W / 2, hx + HORN_W / 2, hy - HORN_R, hy, h0, h1))
    hn = hn.fuse(cyl(HORN_W, h0, h1, hx, hy - HORN_R)).cut(cyl(1.6, h0 - 1, h1 + 1, hx, hy - HORN_R))
    hn = hn.cut(cyl(SV_SPLINE[0], h0 - 1, h0 + SV_SPLINE[1] - 1.5 + 0.01, hx, hy))
    hz = (h0 + h1) / 2
    tip_y = hy - HORN_R
    fit = cq.Solid.makeCylinder(5.0, 8.0 + 2.0, V(FIT_POST[0] - 2.0, tip_y, hz), V(1, 0, 0))           # M10 body
    fit = fit.fuse(cq.Solid.makeCylinder(6.0, 6.0, V(FIT_POST[0] - 8.0, tip_y, hz), V(1, 0, 0)))        # hex (round)
    fit = fit.fuse(cq.Solid.makeCylinder(4.0, 4.0, V(FIT_POST[0] - 12.0, tip_y, hz), V(1, 0, 0)))       # collet
    fit = fit.cut(cq.Solid.makeCylinder(2.05, 40.0, V(FIT_POST[0] - 20.0, tip_y, hz), V(1, 0, 0)))
    return one(sv.clean(), "servo"), one(hn.clean(), "horn"), one(fit.clean(), "fitting")


def yoke():
    """the platform (foot on the turntable, centre tube, fins, top disc) and the fork (plate, two finned cheeks); swivel
    frame, the arm's mid-plane at y 0"""
    # platform
    pl = cyl(PLATE[2], PLAT[0], PLAT[1]).fuse(cyl(TUBE[0], PLAT[1] - 0.01, DISC[0] + 0.01)).fuse(cyl(DISC_D, *DISC))
    for k in range(FIN_N):
        fin = xz_prism([(TUBE[0] / 2 - 1, PLAT[1] - 0.01), (PLATE[2] / 2 - 1, PLAT[1] - 0.01), (DISC_D / 2 - 2, DISC[0] + 0.01),
                        (TUBE[0] / 2 - 1, DISC[0] + 0.01)], -FIN_T / 2, FIN_T / 2)
        fin = fin.fuse(cyl(11.0, DISC[0] - 8, DISC[0] + 0.01, FORK_R, 0))           # under the fork's insert
        pl = pl.fuse(fin.rotate(V(0, 0, 0), V(0, 0, 1), 360.0 * k / FIN_N))
    pl = pl.cut(cyl(TUBE[1], 0, 300))
    for k in range(FIN_N):
        a = math.radians(30 + 60 * k)
        fx, fy = FOOT_R * math.cos(a), FOOT_R * math.sin(a)
        pl = pl.cut(cyl(4.5, PLAT[0] - 1, PLAT[1] + 1, fx, fy)).cut(cyl(12.0, DISC[0] - 1, DISC[1] + 1, fx, fy))   # + key access
        b = math.radians(60 * k)
        pl = pl.cut(cyl(5.6, DISC[1] - 10, DISC[1] + 1, FORK_R * math.cos(b), FORK_R * math.sin(b)))         # M4 insert
    # fork: the plate, the cheeks round J1, fins outside them
    yo = CHEEK_IN + CHEEK_T
    plate = box(-FORK_X, FORK_X, -yo - 12, yo + 12, *FORK).intersect(cyl(DISC_D, FORK[0] - 1, FORK[1] + 1))
    prof = hull2([(CHEEK_R * math.cos(math.radians(a)), Z_J1 + CHEEK_R * math.sin(math.radians(a))) for a in range(0, 360, 6)]
                 + [(-FORK_X, FORK[0]), (FORK_X, FORK[0])])
    fk = plate
    for s in (1, -1):
        y0, y1 = sorted((s * CHEEK_IN, s * yo))
        fk = fk.fuse(xz_prism(prof, y0, y1))
        for fx in (-24.0, 24.0):
            tri = [(s * (yo - 0.01), FORK[1] - 0.01), (s * (yo + 12), FORK[1] - 0.01), (s * (yo - 0.01), FORK[1] + FIN_H)]
            fk = fk.fuse(yz_prism(tri, fx - FIN_T / 2, fx + FIN_T / 2))
        fk = fk.cut(diamond(KEY_AF + KEY_CLEAR, y0 - 1, y1 + 1))    # the boss's key: a diamond prints without support
        for x, z in fl_screw_pts():
            i0, i1 = sorted((s * (yo - INSERT_DEPTH), s * (yo + 1)))
            p0, p1 = sorted((s * (yo - INSERT_DEPTH - 3.0), s * yo))
            fk = fk.cut(cyl_y(INSERT_HOLE, i0, i1, x, z)).cut(cyl_y(3.2, p0, p1, x, z))
    fk = fk.cut(cyl(TUBE[1], FORK[0] - 1, FORK[1] + 1))
    fk = fk.fuse(servo_shelf()).cut(servo_shelf_cuts())
    for k in range(FIN_N):
        b = math.radians(60 * k)
        fk = fk.cut(cyl(4.5, FORK[0] - 1, FORK[1] + 1, FORK_R * math.cos(b), FORK_R * math.sin(b)))
    return one(pl.clean(), "platform"), one(fk.clean(), "fork")


def mw_y():
    """the motor wall's y span: its inner face clear of the motor slid +4 toward the belt"""
    y0 = M[1] + 42.3 / 2 + 4.0 + 0.5
    return y0, y0 + MW_T


def jack_tower_at():
    """the tower's centre T (the 608s' centre J sits ECC off it, square to the line from the swivel axis: mid-throw, so
    a twist either way moves the jackshaft along that line, +-ECC) and the clamp's ear direction (outward)"""
    r = math.hypot(*J)
    ux, uy = J[0] / r, J[1] / r
    T = (J[0] + ECC * uy, J[1] - ECC * ux)                          # J = T + ECC * (-uy, ux)
    return T, math.atan2(uy, ux)


def jack_tower(T, ear_a):
    rt = TJ_OD / 2
    tw = cyl(TJ_OD, BASE_T - 0.01, TJ_TOP, *T)
    tw = tw.fuse(cq.Solid.makeCone(rt + 5, rt, 5, V(T[0], T[1], BASE_T - 0.01), V(0, 0, 1)))   # a fillet at its foot
    for k in range(GUS_N):
        a = ear_a + math.radians(22.5 + 45 * k)
        g = xz_prism([(rt - 1, BASE_T - 0.01), (rt + GUS_R, BASE_T - 0.01), (rt - 1, BASE_T + GUS_H)], -GUS_T / 2, GUS_T / 2)
        tw = tw.fuse(g.rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(a)).translate(V(T[0], T[1], 0)))
    # the clamp's ears (built along +x, turned to ear_a): a slit through ear and wall, two #4s across it
    ear = box(rt - 2, rt + CLAMP_D, -CLAMP_W / 2, CLAMP_W / 2, CLAMP_Z0, TJ_TOP)
    cut = box(0, rt + CLAMP_D + 1, -SLIT / 2, SLIT / 2, CLAMP_Z0, TJ_TOP + 1)
    xs = rt + CLAMP_D / 2 + 0.5
    for z in N4["z"]:
        cut = cut.fuse(cq.Solid.makeCylinder(N4["clear"] / 2, CLAMP_W / 2 + 1, V(xs, -CLAMP_W / 2 - 1, z), V(0, 1, 0)))
        cut = cut.fuse(cq.Solid.makeCylinder(N4["head_d"] / 2, 3.0 + 1, V(xs, -CLAMP_W / 2 - 1, z), V(0, 1, 0)))  # head counterbore
        cut = cut.fuse(cq.Solid.makeCylinder(N4["pilot"] / 2, CLAMP_W / 2 + 1, V(xs, 0, z), V(0, 1, 0)))
    turn = lambda s: s.rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(ear_a)).translate(V(T[0], T[1], 0))   # noqa: E731
    tw = tw.fuse(turn(ear)).cut(turn(cut))
    tw = tw.cut(cyl(CAM_OD + CAM_CLEAR, CAM_Z0, TJ_TOP + 1, *T)).cut(cyl(12.0, -1, CAM_Z0 + 0.01, *T))   # bore; a push-out hole
    return tw


def ray_out(poly, a):
    """how far out from the swivel's axis the outline poly reaches along the direction a (radians)"""
    d, best = (math.cos(a), math.sin(a)), 0.0
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        ex, ey = q[0] - p[0], q[1] - p[1]
        den = d[0] * ey - d[1] * ex
        if abs(den) < 1e-9:
            continue
        t, u = (p[0] * ey - p[1] * ex) / den, (p[0] * d[1] - p[1] * d[0]) / den
        if t > 0 and -1e-9 <= u <= 1 + 1e-9:
            best = max(best, t)
    return best


def hold_down(outer):
    """the hold-down screws [(x, y, where)] and the front lugs' solids"""
    pts, lugs = [], []
    for a_deg in HD_LUG_A:
        a = math.radians(a_deg)
        r_e = ray_out(outer, a)
        lug = box(r_e - 5.0, r_e + HD_LUG_OUT, -HD_LUG_W / 2, HD_LUG_W / 2, 0.0, BASE_T).fuse(
            cyl(HD_LUG_W, 0.0, BASE_T, r_e + HD_LUG_OUT, 0.0))
        lugs.append(lug.rotate(V(0, 0, 0), V(0, 0, 1), a_deg))
        pts.append(((r_e + HD_LUG_OUT) * math.cos(a), (r_e + HD_LUG_OUT) * math.sin(a), f"lug {a_deg:+.0f} deg"))
    pts += [(x, y, "back, inside the cover") for x, y in HD_BACK]
    return pts, lugs


def hd_hole(x, y):
    """a #8 flat head's clearance hole and countersink, flush with the plate's top"""
    h = (HD["head_d"] + 0.4 - HD["clear"]) / 2 / math.tan(math.radians(HD["half"]))
    return cyl(HD["clear"], -1.0, BASE_T + 1.0, x, y).fuse(
        cq.Solid.makeCone(HD["clear"] / 2, (HD["head_d"] + 0.4) / 2, h, V(x, y, BASE_T - h), V(0, 0, 1))).fuse(
        cyl(HD["head_d"] + 0.4, BASE_T - 0.01, BASE_T + 1.0, x, y))


def hd_screw(x, y):
    """a #8 x 1/2" flat-head wood screw, its head flush with the plate's top, the rest down into the wood"""
    h = (HD["head_d"] - HD["d"]) / 2 / math.tan(math.radians(HD["half"]))
    return cq.Solid.makeCone(HD["d"] / 2, HD["head_d"] / 2, h, V(x, y, BASE_T - h), V(0, 0, 1)).fuse(
        cyl(HD["d"], BASE_T - HD["L"], BASE_T - h + 0.01, x, y))


def cam_tube(T):
    c = cyl(CAM_OD, CAM_Z0, TJ_TOP + 0.01, *T).fuse(cyl(CAM_FL_D, TJ_TOP, CAM_TOP, *T))
    c = c.cut(cyl(22.0 + FIT, CAM_Z0 - 1, J_B1 + 7, *J)).cut(cyl(10.0, J_B1, J_B2 + 1, *J)).cut(cyl(22.0 + FIT, J_B2, CAM_TOP + 1, *J))
    for k in range(4):                                              # 3 mm rod holes, radial, in the flange's rim
        a = math.radians(45 + 90 * k)
        h = cq.Solid.makeCylinder(1.6, 9.0, V(T[0] + (CAM_FL_D / 2 + 1) * math.cos(a), T[1] + (CAM_FL_D / 2 + 1) * math.sin(a),
                                              (TJ_TOP + CAM_TOP) / 2), V(-math.cos(a), -math.sin(a), 0))
        c = c.cut(h)
    return one(c.clean(), "cam")


def clamp_screws(T, ear_a):
    rt = TJ_OD / 2
    xs = rt + CLAMP_D / 2 + 0.5
    sc = []
    for z in N4["z"]:
        y_head = -CLAMP_W / 2 + 3.0                                 # the head in its 3 deep counterbore
        s = cq.Solid.makeCylinder(N4["d"] / 2, N4["L"], V(xs, y_head, z), V(0, 1, 0)).fuse(
            cq.Solid.makeCylinder(N4["head_d"] / 2 - 0.2, N4["head_h"], V(xs, y_head - N4["head_h"], z), V(0, 1, 0)))
        sc.append(s.rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(ear_a)).translate(V(T[0], T[1], 0)))
    return cq.Compound.makeCompound(sc)


def link_solids(pts):
    tp = A._link_templates()
    out = []
    n = len(pts)
    for i in range(n):
        (xa, ya), (xb, yb) = pts[i], pts[(i + 1) % n]
        ang = math.degrees(math.atan2(yb - ya, xb - xa))
        t = tp[("in", False, False)] if i % 2 == 0 else tp["out"]
        s = t.rotate(V(0, 0, 0), V(1, 0, 0), 90).rotate(V(0, 0, 0), V(0, 0, 1), ang).translate(V(xa, ya, Z_CHAIN))
        out.append(s)
    return out


def build():
    parts = []                                                     # (name, solid, colour, group)
    add = lambda n, s, c, g: parts.append((f"{n} {TAG}", s, c, g))  # noqa: E731
    dense = chain_loop()
    pts = pitch_points(dense, LINKS)
    gap = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    big_ph = math.atan2(pts[0][1], pts[0][0])
    j_pt = min(pts, key=lambda p: abs(math.hypot(p[0] - J[0], p[1] - J[1]) - R_SMALL))
    small_ph = math.atan2(j_pt[1] - J[1], j_pt[0] - J[0])

    # --- the fixed side: base plate, bearing tower, jackshaft tower + its eccentric, motor plate + posts, cover
    T, ear_a = jack_tower_at()
    out_pts = [((R_BIG + 4) * math.cos(math.radians(a)), (R_BIG + 4) * math.sin(math.radians(a))) for a in range(0, 360, 4)]
    rr = TJ_OD / 2 + GUS_R + 1
    out_pts += [(T[0] + rr * math.cos(math.radians(a)), T[1] + rr * math.sin(math.radians(a))) for a in range(0, 360, 10)]
    re = TJ_OD / 2 + CLAMP_D + 1                                    # the clamp's ears
    out_pts += [(T[0] + math.hypot(re, CLAMP_W / 2) * math.cos(ear_a + k * math.atan2(CLAMP_W / 2, re)),
                 T[1] + math.hypot(re, CLAMP_W / 2) * math.sin(ear_a + k * math.atan2(CLAMP_W / 2, re))) for k in (-1, 1)]
    out_pts += [(M[0] + sx * 30, M[1] + sy * 30) for sx in (-1, 1) for sy in (-1, 1)]
    mw1 = mw_y()[1]
    out_pts += [(M[0] + gx + sx * 3, mw1 + MW_GUS[1] + 2) for gx in MW_GUS_X for sx in (-1, 1)]   # the wall's gussets
    inner = hull2(out_pts)
    outer = offset(inner, WALL)
    base = prism(outer, 0.0, BASE_T)
    hd_pts, lugs = hold_down(outer)
    for lug in lugs:
        base = base.fuse(lug)
    tower = cyl(TOWER_OD, BASE_T - 0.01, TOWER_TOP)
    base = base.fuse(tower)
    n_g, t_g, r_g, h_g = TOWER_GUS
    base = base.fuse(cq.Solid.makeCone(TOWER_OD / 2 + 5, TOWER_OD / 2, 5, V(0, 0, BASE_T - 0.01), V(0, 0, 1)))
    for k in range(n_g):
        g = xz_prism([(TOWER_OD / 2 - 1, BASE_T - 0.01), (TOWER_OD / 2 + r_g, BASE_T - 0.01), (TOWER_OD / 2 - 1, BASE_T + h_g)],
                     -t_g / 2, t_g / 2)
        base = base.fuse(g.rotate(V(0, 0, 0), V(0, 0, 1), 22.5 + 360.0 * k / n_g))
    base = base.cut(cyl(52.0 + FIT, -1, Z_B1 + B_W))                # the lower 6808 goes in from below
    base = base.cut(cyl(SHOULDER_D, Z_B1, Z_B2 + 0.01)).cut(cyl(52.0 + FIT, Z_B2, TOWER_TOP + 1))
    base = base.cut(box(-200, 0, -6, 6, -1, 4))                     # the wires: out of the spindle, under the plate to the back
    # jackshaft: the 608s sit in an eccentric cam tube turning in the tower; twist it (a 3 mm rod in its flange) to
    # tension the chain, then the split clamp's two #4s lock it. The pulley and the 10T on the shaft's overhang above.
    base = base.fuse(jack_tower(T, ear_a)).cut(cyl(CAM_OD + CAM_CLEAR, CAM_Z0, TJ_TOP + 1, *T)).cut(cyl(12.0, -1, CAM_Z0 + 0.01, *T))
    # motor: a plate at its face on a wall all the way across its +y side, two gussets in front of the wall (Mike
    # 2026-11-21: "make that solid all the way across in front of that motor. And I want two gussets in front of it ...
    # that post in the corner ... removed"); slots +-4 along the belt (y) for its tension, the wall clear at +4
    mw0, mw1 = mw_y()
    mp = box(M[0] - 29, M[0] + 29, M[1] - 29, mw1, Z_MOTOR_FACE, Z_MOTOR_FACE + 4)
    mp = mp.fuse(box(M[0] - 29, M[0] + 29, mw0, mw1, BASE_T - 0.01, Z_MOTOR_FACE + 0.01))
    for gx in MW_GUS_X:
        tri = [(mw1 - 0.01, BASE_T - 0.01), (mw1 + MW_GUS[1], BASE_T - 0.01), (mw1 - 0.01, BASE_T + MW_GUS[2])]
        mp = mp.fuse(yz_prism(tri, M[0] + gx - MW_GUS[0] / 2, M[0] + gx + MW_GUS[0] / 2))
    slot = lambda d, x, y: cyl(d, Z_MOTOR_FACE - 1, Z_MOTOR_FACE + 5, x, y - 4).fuse(cyl(d, Z_MOTOR_FACE - 1, Z_MOTOR_FACE + 5, x, y + 4)).fuse(
        box(x - d / 2, x + d / 2, y - 4, y + 4, Z_MOTOR_FACE - 1, Z_MOTOR_FACE + 5))    # noqa: E731
    mp = mp.cut(slot(23.0, *M))
    for sx in (-15.5, 15.5):
        for sy in (-15.5, 15.5):
            mp = mp.cut(slot(3.4, M[0] + sx, M[1] + sy))
    base = base.fuse(mp)
    for x, y, _ in hd_pts:
        base = base.cut(hd_hole(x, y))
    add("swivel base (printed): plate, bearing tower, jackshaft tower with its split clamp, motor plate", one(base.clean(), "base"), PRINT, "fixed")
    add("#8 x 1/2\" flat-head wood screws (hold-down: the base into the wood)",
        cq.Compound.makeCompound([hd_screw(x, y) for x, y, _ in hd_pts]), DARK, "fixed")
    add(f"jackshaft eccentric (printed): the 608s in a cam tube, {ECC:g} mm throw; a 3 mm rod in its flange twists it",
        cam_tube(T), LIGHT, "fixed")
    add("#4 x 3/4 sheet-metal screws (jackshaft clamp)", clamp_screws(T, ear_a), DARK, "fixed")
    cover = prism(outer, BASE_T, COVER_TOP).cut(prism(inner, BASE_T - 1, COVER_IN)).cut(cyl(COVER_OPEN, COVER_IN - 1, COVER_TOP + 1))
    add("swivel cover (printed): walls and roof, lifts off", one(cover.clean(), "cover"), PRINT, "cover")

    # bearings, shafts, motor
    for nm, zb in (("lower", Z_B1), ("upper", Z_B2)):
        add(f"6808 bearing ({nm})", ring(40.0, 52.0, zb, zb + B_W), STEEL, "fixed")
    add("608 bearing (jackshaft, lower)", ring(8.0, 22.0, J_B1, J_B1 + 7, *J), STEEL, "fixed")
    add("608 bearing (jackshaft, upper)", ring(8.0, 22.0, J_B2, J_B2 + 7, *J), STEEL, "fixed")
    body, caps = nema17.load(48.0)
    caps = nema17.trimmed_caps(caps, 15.0)
    add("NEMA 17 48 mm (swivel motor, standing up)", body.translate(V(M[0], M[1], Z_MOTOR_FACE)), BLACK, "fixed")
    add("NEMA 17 end caps (swivel motor)", caps.translate(V(M[0], M[1], Z_MOTOR_FACE)), STEEL, "fixed")

    # --- turns with J0: spindle + collar + turntable plate (one printed piece), his 90T, its screws
    tt = cyl(SPINDLE_OD, COLLAR[1], PLATE[0] + 0.01).fuse(cyl(PLATE[2], PLATE[0], PLATE[1]))
    for k in range(BOSS_N):
        a = 2 * math.pi * k / BOSS_N
        tt = tt.fuse(cyl(10.0, Z_CHAIN + C.SPR_T / 2 + 0.05, PLATE[0] + 0.01, BOSS_R * math.cos(a), BOSS_R * math.sin(a)))
        tt = tt.cut(cyl(2.5, Z_CHAIN, PLATE[0] + 8, BOSS_R * math.cos(a), BOSS_R * math.sin(a)))
    tt = tt.cut(cyl(SPINDLE_ID, 0, PLATE[1] + 1))
    for k in range(FIN_N):                                          # the yoke platform's M4 inserts
        a = math.radians(30 + 60 * k)
        tt = tt.cut(cyl(5.6, PLATE[1] - 8, PLATE[1] + 1, FOOT_R * math.cos(a), FOOT_R * math.sin(a)))
    add("swivel turntable (printed): spindle and plate", one(tt.clean(), "turntable"), PRINT, "turn")
    plat, fork = yoke()
    add("yoke platform (printed): foot, fins, top disc; 6 x M4 into the turntable", plat, PRINT, "yoke")
    add("yoke fork (printed): plate and cheeks; the J1 bosses key into them", fork, PRINT, "yoke")
    sv, hn, fit = servo_parts()
    add("MG996R servo on the yoke's shelf (the claw's cable)", sv, SV_GREY, "yoke")
    add("MG996R horn, one arm 15 (the claw's cable)", hn, cq.Color(0.95, 0.95, 0.95), "yoke")
    add("PC4-M10 fitting (the claw's Bowden starts here, straight back)", fit, ALU, "yoke")
    for side, nm in ((1, "+y"), (-1, "-y")):
        add(f"shoulder boss {nm} (keyed, printed flange down)", boss(side), PRINT, "yoke")
        sc, fs, ins = boss_hardware(side)
        add(f"M3 x {SPR_SCREW_L:g} screws {nm} (boss flange into the tapped 25T)", sc, DARK, "yoke")
        add(f"M3 x {FL_SCREW_L:g} screws {nm} (boss flange into the cheek)", fs, DARK, "yoke")
        add(f"M3 x 4 x 5 inserts {nm} (yoke cheek)", ins, cq.Color(0.85, 0.65, 0.25), "yoke")
    add("swivel collar", ring(SPINDLE_OD, COLLAR[2], COLLAR[0], COLLAR[1]), STEEL, "turn")
    big = sprocket(N_BIG, 0.0, 0.0, Z_CHAIN, 72.0, big_ph)
    for k in range(BOSS_N):
        a = 2 * math.pi * k / BOSS_N
        big = big.cut(cyl(3.4, 60, 90, BOSS_R * math.cos(a), BOSS_R * math.sin(a)))
    add(f"{N_BIG}T #25 sprocket (his CNC cut, 1/8 aluminium, 6 x M3 up into the turntable)", big, ALU, "turn")

    # --- the jackshaft (turns 9 x J0) and the motor pulley (27 x J0)
    shaft = cyl(8.0, J_B1, Z_CHAIN + C.SPR_T / 2 + 4.0, *J)
    small = sprocket(N_SMALL, J[0], J[1], Z_CHAIN, 8.0, small_ph, hub=(16.0, 4.0))
    jp = gt2_pulley(T_JACK, *J, 8.0)
    add("jackshaft 8 mm", shaft, STEEL, "jack")
    add(f"{N_SMALL}T #25 sprocket (jackshaft)", small, DARK, "jack")
    add(f"GT2 {T_JACK}T pulley, 8 mm bore (jackshaft; his)", jp, ALU, "jack")
    mpul = gt2_pulley(T_MOTOR, *M, 5.0, hub_below=False)                 # its hub up: the motor's boss is under it
    add(f"GT2 {T_MOTOR}T pulley, 5 mm bore (swivel motor)", mpul, ALU, "motor")
    # the belt (still; its teeth don't show)
    R, r = gt2(T_JACK) / 2 + 0.75, gt2(T_MOTOR) / 2 + 0.75
    outer_b = D.hull_solid(J, R, M, r, lambda u, v: V(u, v, Z_BELT[0]), V(0, 0, Z_BELT[1] - Z_BELT[0]))
    inner_b = D.hull_solid(J, R - 1.5, M, r - 1.5, lambda u, v: V(u, v, Z_BELT[0] - 1), V(0, 0, Z_BELT[1] - Z_BELT[0] + 2))
    add(f"GT2 belt {BELT_L:.0f} x 6 mm (swivel, {T_JACK}:{T_MOTOR})", one(outer_b.cut(inner_b).clean(), "belt"), BLACK, "fixed")
    # --- the chain, link by link (it runs round as J0 turns)
    for i, s in enumerate(link_solids(pts)):
        add(f"#25 roller chain link {i + 1:02d} (swivel, {LINKS} links)", s, DARK, "chain")
    return parts, dict(pts=pts, dense=dense, gap=gap, T=T, hd=hd_pts)


def check(parts):
    """every pair of parts that share volume, the turning side at several J0 angles"""
    fixed = [(n, s) for n, s, c, g in parts if g in ("fixed", "cover", "jack", "motor")]
    turn = [(n, s) for n, s, c, g in parts if g in ("turn", "yoke")]
    chain = [(n, s) for n, s, c, g in parts if g == "chain"]
    skip = lambda a, b: ("608" in a and "jackshaft 8" in b) or ("jackshaft 8" in a and "608" in b) or \
        ("#4 x 3/4" in a + b and "swivel base" in a + b) or("pulley" in a + b and ("jackshaft 8" in a + b or "end caps" in a + b)) or \
        ("sprocket (jackshaft)" in a + b and "jackshaft 8" in a + b) or ("GT2 belt" in a + b and "pulley" in a + b) or \
        ("6808" in a + b and ("swivel base" in a + b or "turntable" in a + b)) or ("collar" in a + b and "turntable" in a + b) or         ("yoke platform" in a + b and "turntable" in a + b) or ("yoke platform" in a + b and "yoke fork" in a + b) or         (("shoulder boss" in a + b or "M3 x" in a + b) and ("yoke fork" in a + b or "shoulder boss" in a + b)) or \
        (("MG996R" in a + b or "PC4-M10" in a + b) and ("yoke fork" in a + b or "MG996R" in a + b))   # noqa: E731
    hits = []

    def ov(a, b):
        A_, B_ = a.BoundingBox(), b.BoundingBox()
        if A_.xmax < B_.xmin or B_.xmax < A_.xmin or A_.ymax < B_.ymin or B_.ymax < A_.ymin or A_.zmax < B_.zmin or B_.zmax < A_.zmin:
            return 0.0
        return a.intersect(b).Volume()
    allp = fixed + turn
    for i in range(len(allp)):
        for j in range(i + 1, len(allp)):
            if skip(allp[i][0], allp[j][0]):
                continue
            v = ov(allp[i][1], allp[j][1])
            if v > 0.5:
                hits.append((round(v, 1), allp[i][0], allp[j][0]))
    for ang in (30, 60, 90, 135, 200):
        for n, s in turn:
            t = s.rotate(V(0, 0, 0), V(0, 0, 1), ang)
            for m, f in fixed:
                if skip(n, m):
                    continue
                v = ov(t, f)
                if v > 0.5:
                    hits.append((round(v, 1), f"{n} @J0 {ang}", m))
    chain_u = chain[0][1].fuse(*[s for _, s in chain[1:]]) if chain else None
    if chain_u is not None:
        for m, f in fixed + turn:
            if "sprocket" in m or "625" in m:
                continue
            v = ov(chain_u, f)
            if v > 0.5:
                hits.append((round(v, 1), "the chain", m))
    return hits


def write_flat_fc(items, path, face_col):
    """write_flat, plus a colour on some faces of a body: [(name, shape, colour, face predicate or None)]"""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_StepModelType
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_DocumentTool
    from OCP.XSControl import XSControl_WorkSession
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    for name, shape, color, faces in items:
        sh = shape.wrapped
        if not sh.Location().IsIdentity():
            sh = BRepBuilderAPI_Transform(sh.Located(TopLoc_Location()), sh.Location().Transformation(), True).Shape()
        faces = [f for f in cq.Shape.cast(sh).Faces() if faces(f)] if faces else []      # a predicate on the baked faces
        lab = st.AddShape(sh, False)
        TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
        if color is not None:
            ct.SetColor(lab, color.wrapped.GetRGB(), XCAFDoc_ColorSurf)
        for f in faces:
            sub = st.AddSubShape(lab, f.wrapped)
            if not sub.IsNull():
                ct.SetColor(sub, face_col.wrapped.GetRGB(), XCAFDoc_ColorSurf)
    w = STEPCAFControl_Writer(XSControl_WorkSession(), False)
    w.SetColorMode(True)
    w.SetNameMode(True)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    w.Transfer(doc, STEPControl_StepModelType.STEPControl_AsIs)
    assert w.Write(path) == IFSelect_RetDone


def studio(parts, info):
    """STEP files (the arm's design frame = the studio's model frame) and the studios' models / J0 / gears / paths"""
    files = {"fixed": "swivel_chain_fixed.step", "cover": "swivel_chain_cover.step", "turn": "swivel_chain_turntable.step",
             "jack": "swivel_chain_jackshaft.step", "motor": "swivel_chain_motor_pulley.step", "chain": "swivel_chain_chain.step",
             "yoke": "swivel_chain_yoke.step"}
    names = {"fixed": "swivel chain (fixed)", "cover": "swivel chain cover", "turn": "swivel chain turntable",
             "jack": "swivel chain jackshaft", "motor": "swivel chain motor pulley", "chain": "swivel chain chain",
             "yoke": "swivel chain yoke"}
    T = info["T"]
    tdx, tdz, ylim = -T[0], AB.Z_MID - T[1], Y_WOOD - (BASE_T - 0.05)
    reach = TJ_OD / 2 + max(GUS_R, CLAMP_D) + 10.0

    def tower_face(f):                                              # the jackshaft tower's faces (design frame)
        b = f.BoundingBox()
        r = max(math.hypot(x - tdx, z - tdz) for x in (b.xmin, b.xmax) for z in (b.zmin, b.zmax))
        return r <= reach and (b.ymax <= ylim or r <= CAM_OD / 2 + 1)
    for g, fn in files.items():
        items = []
        for n, s, c, gg in parts:
            if gg != g:
                continue
            sd = to_design(s, Y_WOOD)
            items.append((n, sd, c, tower_face if n.startswith("swivel base") else None))
        write_flat_fc(items, os.path.join(OUT, fn), LIGHT)
    dpt = lambda x, y, z: [round(-x, 4), round(Y_WOOD - z, 4), round(-y + AB.Z_MID, 4)]   # noqa: E731   (to_design)
    pts, dense = info["pts"], info["dense"]
    cum, s_at = [0.0], {}
    for k in range(1, len(dense)):
        cum.append(cum[-1] + math.hypot(dense[k][0] - dense[k - 1][0], dense[k][1] - dense[k - 1][1]))
    tot = cum[-1] + math.hypot(dense[0][0] - dense[-1][0], dense[0][1] - dense[-1][1])

    def s_of(p):
        best = (1e9, 0.0)
        for k in range(0, len(dense), 3):
            d = math.hypot(dense[k][0] - p[0], dense[k][1] - p[1])
            if d < best[0]:
                best = (d, cum[k])
        return best[1]
    ss = [s_of(p) for p in pts]
    step = max(1, len(dense) // 1500)
    path_pts = [[round(x, 3), round(y, 3)] for x, y in dense[::step]]
    pinfo = [[round(ss[i], 3), round((ss[(i + 1) % len(ss)] - ss[i]) % tot, 3)] for i in range(len(ss))]
    gears = [dict(model=names["jack"], follows="J0 swivel", ratio=N_BIG / N_SMALL, at=dpt(J[0], J[1], 0), axis=[0, -1, 0],
                  by="swivel_chain.py", why=f"the jackshaft: {N_BIG}/{N_SMALL} x J0, the same way round (open chain)"),
             dict(model=names["motor"], follows="J0 swivel", ratio=N_BIG / N_SMALL * T_JACK / T_MOTOR, at=dpt(M[0], M[1], 0),
                  axis=[0, -1, 0], by="swivel_chain.py", why=f"the motor: x {T_JACK}/{T_MOTOR} more (open belt)")]
    paths = [dict(model=names["chain"], follows="J0 swivel", mm_per_deg=round(R_BIG * math.pi / 180, 5), closed=True,
                  o=dpt(0, 0, Z_CHAIN), u=[-1, 0, 0], v=[0, 0, -1], pts=path_pts, parts=pinfo, by="swivel_chain.py")]
    for sc in STUDIOS:
        if not os.path.exists(sc):
            continue
        with open(sc, encoding="utf-8") as fh:
            d = json.load(fh)
        have = {m["path"] for m in d["models"]}
        for g, fn in files.items():
            p = os.path.join(OUT, fn)
            if p not in have:
                d["models"].append({"path": p, "name": names[g]})
        j0 = next(j for j in d["joints"] if j["name"] == "J0 swivel")
        for g in ("turn", "yoke"):
            if names[g] not in j0.setdefault("models", []):
                j0["models"].append(names[g])
        d["gears"] = [g for g in d.get("gears", []) if g.get("by") != "swivel_chain.py"] + gears
        d["paths"] = [q for q in d.get("paths", []) if q.get("by") != "swivel_chain.py"] + paths
        mine = [dict(match=r"^swivel chain \(fixed\) / swivel base", color="#b9bcc2", by="swivel_chain.py",
                     why="Mike 2026-11-21: 'make it the same color as the rest of the body parts' ('motor plate' in its name "
                         "hit the NEMA|motor rule)"),
                dict(match=r"^swivel chain \(fixed\) / jackshaft eccentric", color="#e2e5e9", by="swivel_chain.py",
                     why="Mike 2026-11-21: the cam tube and its tower lighter (the tower's faces are coloured in the STEP)"),
                dict(match=r"^swivel chain yoke / MG996R servo", color="#8a8e95", by="swivel_chain.py", why="the servo grey"),
                dict(match=r"^swivel chain yoke / MG996R horn", color="#eeeeee", by="swivel_chain.py")]
        d["colors"] = mine + [c for c in d.get("colors", []) if c.get("by") != "swivel_chain.py"]   # first match wins
        hx, hy = sv_shaft()
        hz = sum(horn_z()) / 2
        d.setdefault("claw", {})["horn"] = dict(
            part=f"MG996R horn, one arm 15 (the claw's cable) {TAG}", pivot=dpt(hx, hy, hz), axis=[0, 1, 0], deg=23.58,
            hole=dpt(hx, hy - HORN_R, hz), by="swivel_chain.py",
            why="Mike 2026-11-21: the servo on the yoke's shelf; the horn turns asin(6 / 15) to pull the claw's 6 mm of "
                "cable straight back into the fitting")
        with open(sc, "w", encoding="utf-8") as fh:
            json.dump(d, fh, indent=1)
    print("wrote", ", ".join(files.values()), "in complete assembly/; studios updated")


def solve_layout():
    """the centre distance where LINKS links close exactly (the chain sits on the sprockets as a polygon, so the belt
    formula's 118.5 comes out a link-and-a-half long); the jackshaft goes there, the idler then takes up the slack"""
    global CC, J, M

    def gap(c):
        global J
        J = (X_BACK, math.sqrt(c * c - X_BACK ** 2))
        pts = pitch_points(chain_loop(), LINKS)
        return math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) - P
    lo, hi = CC - 6.0, CC
    for _ in range(40):
        mid = (lo + hi) / 2
        if gap(mid) > 0:
            hi = mid
        else:
            lo = mid
    CC = (lo + hi) / 2
    J = (X_BACK, math.sqrt(CC ** 2 - X_BACK ** 2))
    M = (X_BACK, J[1] - belt_c(BELT_L))


def main():
    solve_layout()
    parts, info = build()
    print(f"chain C {CC:.2f} for {LINKS} links (closure gap {info['gap']:.2f} mm vs pitch {P}); jackshaft at "
          f"({J[0]:.1f}, {J[1]:.1f}); motor at ({M[0]:.1f}, {M[1]:.1f}), belt C {J[1] - M[1]:.1f} for {BELT_L:.0f} mm; "
          f"eccentric tower at ({info['T'][0]:.1f}, {info['T'][1]:.1f}), chain C {CC - ECC:.1f}..{CC + ECC:.1f}; "
          f"ratio {N_BIG / N_SMALL * T_JACK / T_MOTOR:.0f}:1")
    bb = [s.BoundingBox() for n, s, c, g in parts if g in ("fixed", "cover")]
    print(f"footprint x {min(b.xmin for b in bb):.1f} .. {max(b.xmax for b in bb):.1f}, y {min(b.ymin for b in bb):.1f} .. "
          f"{max(b.ymax for b in bb):.1f}, height {max(b.zmax for b in bb):.1f}")
    for v, a, b in check(parts):
        print(f"  CLASH {v:8.1f} mm3: {a[:70]}  <->  {b[:70]}")
    print("hold-down #8 x 1/2\" flat heads (swivel X, Y | design / studio x, z; the plate's underside at design y "
          f"{Y_WOOD:.1f}):")
    for x, y, where in info["hd"]:
        print(f"  ({x:8.2f}, {y:8.2f})  |  ({-x:8.2f}, {-y + AB.Z_MID:8.2f})  {where}")
    if "--no-studio" not in sys.argv:
        studio(parts, info)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
herringbone_planetary.py - Mike's herringbone planetary (his sketch, 2026-11-23), 5.25:1, module 1.5.

Layout (his): motor in -> a 6808 carries the input shaft so the motor's own bearings carry no gear load -> the shaft
steps down to the sun -> three planets, 11 wide, each on one 624 (4x13x5, against a lip on the motor side) on an
M4 x 20 cap screw (head inside the planet, motor side; 2026-11-25, was two 3x6x2 on a 3 x 20 dowel) ->
the screws thread into a spider whose planet bosses hang on spring blades that push every planet out into the ring (no
slop at the ring) -> the spider is the output, in a second 6808 -> the sun's tip runs in a 6800 in the spider.
The ring is split at the point of the herringbone V (it can't be assembled in one piece).
2026-11-25 (Mike): the motor hangs 22.2 below the plate on four 8 mm standoff tubes (air between motor and plate), and
drives through a dog bone: a printed hub covers the whole motor shaft, and a link with a crowned (ball-end) hex at each
end drives a hex socket in the input shaft. The motor's bearings and the input 6808 no longer fight: the bone takes the
tilt and offset between them, and the hub and bone are the replaceable parts.

    python herringbone_planetary.py [--sun-backlash 0.06] [--ring-backlash 0.0] [--spider-mat PA12|PLA]
                                    [--preload 8] [--preload-defl 0.6] [--no-checks]

Writes robo_chess/herringbone/:
  step/   every part in its assembled place (z = 0 is the motor face) + assembly.step
  print/  every printed part as it sits on the bed (z = 0 on the bed)
  report.txt  gear sizes, mesh and overlap checks, loads, hardware, assembly order

z stack (mm): -45.2 .. -22.2 motor (17HE08)     -22.2 .. 1.0 standoff tubes (tops in 1.0 pockets in the plate)
              -19.7 .. 6.3   motor hub           -0.2 .. 13.7 dog bone (ball centres 2.8 and 10.7)
                0.0 .. 5.2   motor plate         5.2 .. 7.2  its register boss (in half A's counterbore)
                7.2 .. 14.2  input 6808 (outer race on the plate boss, inner race on the input shaft's disc)
               15.9 .. 26.9  sun and planets (V point at 21.4)       5.2 .. 21.4 ring half A, 21.4 .. 49.4 half B
               27.4 .. 37.4  spider bosses + blades   38.4 .. 41.4 spider back disc   42.4 .. 49.4 output 6808
               49.4 .. 54.4  retainer ring            49.4 .. 55.4 output disc (3 M3 x 4 x 5 inserts on r 15)
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector  # noqa: E402

OUT = os.path.join(os.path.dirname(HERE), "herringbone")

# ---------------------------------------------------------------- gears
MN = 1.5                       # normal module (tooth size), Mike
AN = math.radians(20.0)        # normal pressure angle
BETA = math.radians(22.0)      # herringbone helix angle
ZS, ZP, ZR, NP = 8, 13, 34, 3  # sun, planet, ring teeth, planets: (8 + 34) / 3 = 14, so all three planets are alike
MT = MN / math.cos(BETA)
AT = math.atan(math.tan(AN) / math.cos(BETA))
XS = 1 - ZS * math.sin(AT) ** 2 / (2 * math.cos(BETA))       # sun shift: no undercut on 8 teeth
XP = -XS                                                     # planet takes the opposite shift: standard centres
RS, RP, RR = MT * ZS / 2, MT * ZP / 2, MT * ZR / 2
A = RS + RP                                                  # planet circle
RATIO = 1 + ZR / ZS
FLANK_STEPS = 14

# ---------------------------------------------------------------- axial stack
PW = 11.0                      # planet (and sun) width, Mike
Z_P0 = 15.9
Z_P1 = Z_P0 + PW               # 26.9
Z_AP = (Z_P0 + Z_P1) / 2       # 21.4 herringbone V point = ring split
PLATE_T = 5.2                  # plate body; its register boss stands 2.0 above it
REG_H, REG_D = 2.0, 58.0       # plate register boss in half A's counterbore
REG_HOLE = 58.1
REG_RECESS = REG_H + 0.52      # half A's recess over the boss, 2.51 deep (was 2.01) (Mike, 2026-11-25: +0.5, support marks on its ceiling were
                               # holding the halves off the plate). Half A's bottom face on the plate still clamps.
Z_SEAT = PLATE_T + REG_H       # 7.2 input 6808 sits on the boss top
BRG_W = 7.0
Z_RING_A0 = 14.4               # half A teeth start (the input 6808 passes through them to its seat)
Z_RING_B1 = Z_P1 + 0.75        # half B teeth end
Z_SP0 = Z_P1 + 0.5             # 27.4 spider front (bosses, hub, rim)
BOSS_H = 10.0
Z_SP_DISC0 = Z_SP0 + BOSS_H + 1.0   # 38.4 back disc: 1.0 behind the bosses/blades so they can move
DISC_T = 3.0
Z_SHOULDER0 = Z_SP_DISC0 + DISC_T   # 41.4
Z_OUT_BRG0 = Z_SHOULDER0 + 1.0      # 42.4
Z_OUT_BRG1 = Z_OUT_BRG0 + BRG_W     # 49.4
Z_LIP0 = Z_SHOULDER0 + 0.2          # 41.6 half B lip under the output 6808's outer race
RET_T = 6.0                    # the clamp bolt heads sit on it, 0.8 under the claw cup
Z_OUT_FACE = 59.2              # the spread cycloid's output face: the claw adapter / tube-link cup bolt on unchanged
OUTD_T = Z_OUT_FACE - Z_OUT_BRG1   # 9.8

# ---------------------------------------------------------------- bought parts
B6808 = dict(d=40.0, D=52.0, w=7.0, inner_od=42.5, outer_id=49.5)
B6800 = dict(d=10.0, D=19.0, w=5.0, inner_od=12.5, outer_id=16.3)
B624 = dict(d=4.0, D=13.0, w=5.0, inner_od=6.7, outer_id=10.6)  # Mike's planet bearing (2026-11-25), static ~490 N;
                               # ring shoulders approximate: anything touching the inner ring stays between 5.6 and 6.7
M4 = dict(head_d=7.0, head_h=4.0, major=3.95, tap=3.5)   # his M4 x 20 cap screws (24 overall); tap: loose in PLA (Mike)
PLANET_SCREW = 20.0
M3 = dict(head_d=5.5, head_h=3.0, major=2.95, clear=3.4, tap=2.5, cb_d=6.6)
SHAFT_D, SHAFT_LEN, FLAT_DEPTH = 5.0, 20.0, 0.5   # 17HE08 pancake: D5 x 20 D-cut
NEMA = dict(face=42.3, body=23.0, pilot_d=22.0, pilot_h=2.0, screw=31.0)

# ---------------------------------------------------------------- fits (printed sizes: his printer prints holes small)
SEAT_6808_IN = 39.9            # input/output shaft seats in the 6808 inner races (light press)
POCKET_6808 = 52.0             # outer-race seats
POCKET_6800 = 19.0
POCKET_624 = 13.0              # the planet's press seat for its 624
# Mike, 2026-11-25: the lip moved to the motor side so the motor-side teeth sit on 4.0 of rim (over the 8.0 head pocket)
# instead of 1.4 over a 13.2 one; the spider-side teeth sit over the steel 624. The 624 presses in from the spider side.
LIP_T, LIP_HOLE = 1.0, 8.0     # on the 624's outer ring, clear of the inner ring; the same 8.0 bore is the head pocket
Z_PB0 = Z_P1 - 5.0             # 21.9: the 624 (21.9 .. 26.9), flush with the planet's spider face
SPACER_OD, SPACER_ID = 6.0, 4.3
HEAD_SPACER = 1.5              # printed tube under the head, through the lip to the inner ring: head 16.4 .. 20.4
NOSE_D = SPACER_OD             # 2026-11-26 (Mike): the boss spacers are part of the spider now: a 6.0 nose on each boss,
                               # 0.5 out to the 624's inner ring (the spider prints output face down, noses up)
JOURNAL = 10.0                 # sun tip in the 6800
HUB_BORE = 5.0
FLAT_START = 9.5               # the bore's flat starts here: fits any motor whose flat starts <= 9.5 from the face
BODY_D = 70.5                  # the cycloid's body: the arm's screw circle r 32.4 needs it
GRID = 360.0 / 38              # cycloid.py's ring-pin grid: the arm screws sit on its half steps
ARM_R, ARM_PILOT, ARM_PILOT_TOP, ARM_LEAD = 32.4, 2.4, 16.0, 0.5   # cycloid.py ARM_SCREW_*: #4 x 3/4" wood screws
ARM_ANGLES = tuple(GRID * n for n in (3.5, 9.5, 15.5, 22.5, 28.5, 34.5))   # 33.16, 90, ... 326.84 (arm's, unchanged)
BOLT_R = 31.5                  # clamp bolts: heads on the retainer, into inserts in the plate's underside
BOLT_ANGLES = (0.0, 120.0, 240.0)   # on lands, 26.8+ deg from every arm screw
FIT_HOLE = 3.0                 # the ring halves on the bolts' plain shanks (clocks the two tooth halves)
INSERT_HOLE, INSERT_DEPTH = 4.2, 4.5   # his M3 x 4 x 5 heat-set inserts
# grooved OD on the ring halves, like a spline-drive lug nut (Mike's photo, 2026-11-23): narrow round-bottom grooves,
# wide lands, soft lips. Grooves at 15 + 30k deg: every arm screw and clamp bolt lands 11.8+ deg off a groove
N_GROOVE = 12
GROOVE_START = 15.0
GROOVE_R = 3.0                 # round bottom, axis on the OD: 6 wide, 3 deep (floor r 32.25)
LIP_FILLET = 1.0               # rounded lips

# ---------------------------------------------------------------- spider
RIM_IN, RIM_OUT = 22.5, 24.5
BOSS_D = 9.0                   # 2026-11-25: holds an M4 thread (was 7 for the 3 mm dowels)
DISC_TIP_HOLE = 6.0            # the M4 tips run 3.0 into the back disc: clear, so the boss still moves
HUB_D = 22.2
SPOKE_W = 4.0

# ---------------------------------------------------------------- motor on standoffs + dog-bone coupling (Mike, 2026-11-25)
Z_MOTOR = -22.2                # motor face (was 0.0, on the plate): M3 x 30 screws, heads on z 4.0, 3.8 into the motor
MOTOR_SCREW = 30
STANDOFF_D, STANDOFF_POCKET = 8.0, 1.0   # tubes on the motor screws; their tops in 8.4 pockets 1.0 deep in the plate
PLATE_HOLE = 24.0              # through the plate: the hub lifts out through it
# Mike, 2026-11-25: "the root diameter of that coupling is ridiculously small" -> hex 10 -> 12, neck 7 -> 9 (2.1 x in
# twist), hub 18 -> 20 (3.0 wall at the socket corners), plate hole 22 -> 24
HEX_AF = 12.0                  # dog bone hex, across flats
HEX_CLEAR = 0.10               # sockets are HEX_AF + this (holes print small: ~0 .. 0.05 running)
HEX_CORNER_D = 13.2            # bone corners cut back from 13.86 to clear the printed sockets' rounded corners
CROWN_R = 15.0                 # each flat crowned to this radius along the axis (ball-end hex): rocks without binding
BALL_L = 6.0                   # crowned length of each bone end
NECK_D = 9.0
SOCKET_DEPTH = 7.0
END_GAP = 0.5                  # bone end to socket floor, each end
MHUB_D = 20.0                  # motor hub
MHUB_FLOOR = 1.0               # between the motor shaft's tip and the hub's socket: the shaft is covered
Z_SHAFT_TIP = Z_MOTOR + SHAFT_LEN                          # -2.2
Z_MHUB0 = Z_MOTOR + NEMA["pilot_h"] + 0.5                  # -19.7: clear of the motor's pilot boss
Z_MSOCK0 = Z_SHAFT_TIP + 0.5 + MHUB_FLOOR                  # -0.7: hub socket floor
Z_MHUB1 = Z_MSOCK0 + SOCKET_DEPTH                          # 6.3: hub top, 0.9 under the input shaft
Z_BALL_LO = Z_MSOCK0 + END_GAP + BALL_L / 2                # 2.8
Z_BALL_HI = Z_SEAT + SOCKET_DEPTH - END_GAP - BALL_L / 2   # 10.7: mid-socket in the input shaft's disc
GRUB_Z = Z_MOTOR + 11.5        # -10.7: M3 x 6 grub on the shaft flat (flat from 9.5) in an M3 x 4 x 5 insert, 2.7 below
                               # link C's plate (z -8): reached from the side between the standoffs


def V(x, y, z=0.0):
    return Vector(x, y, z)


def inv(a):
    return math.tan(a) - a


def pol(r, a, z=0.0):
    return V(r * math.cos(a), r * math.sin(a), z)


def cyl(d, h, z=0.0, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, h, V(x, y, z))


def ring(od, idia, h, z=0.0, x=0.0, y=0.0):
    return cyl(od, h, z, x, y).cut(cyl(idia, h + 2, z - 1, x, y))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def one(shape):
    s = shape.Solids()
    if len(s) != 1:
        s = sorted(s, key=lambda q: -q.Volume())
    return s[0]


def cut_all(body, cutters):
    for c in cutters:
        body = body.cut(c)
    return body


# ---------------------------------------------------------------- gear geometry
class Gear:
    """transverse involute data for one gear"""

    def __init__(self, z, x, ra, rf, s_t=None, e_t=None, internal=False):
        self.z, self.x, self.ra, self.rf, self.internal = z, x, ra, rf, internal
        self.rp = MT * z / 2
        self.rb = self.rp * math.cos(AT)
        self.s_t, self.e_t = s_t, e_t

    def half(self, r):
        """external: tooth half-angle at r; internal: space half-angle at r (a ring's space is shaped like an
        external tooth: it narrows outward)"""
        r = max(r, self.rb)
        w = self.e_t if self.internal else self.s_t
        return w / (2 * self.rp) + inv(AT) - inv(math.acos(self.rb / r))

    def wire(self, rot, z=0.0):
        n, pitch = self.z, 2 * math.pi / self.z
        edges = []
        if not self.internal:
            ra, rf, rb = self.ra, self.rf, self.rb
            r0 = max(rb, rf)
            fl = [r0 + (ra - r0) * i / FLANK_STEPS for i in range(FLANK_STEPS + 1)]
            h0 = self.half(r0)
            for k in range(n):
                c = rot + pitch * k
                if rf < rb - 1e-9:
                    edges.append(cq.Edge.makeLine(pol(rf, c - h0, z), pol(rb, c - h0, z)))
                edges.append(cq.Edge.makeSpline([pol(r, c - self.half(r), z) for r in fl]))
                ht = self.half(ra)
                edges.append(cq.Edge.makeThreePointArc(pol(ra, c - ht, z), pol(ra, c, z), pol(ra, c + ht, z)))
                edges.append(cq.Edge.makeSpline([pol(r, c + self.half(r), z) for r in reversed(fl)]))
                if rf < rb - 1e-9:
                    edges.append(cq.Edge.makeLine(pol(rb, c + h0, z), pol(rf, c + h0, z)))
                a0, a1 = c + h0, c + pitch - h0
                edges.append(cq.Edge.makeThreePointArc(pol(rf, a0, z), pol(rf, (a0 + a1) / 2, z), pol(rf, a1, z)))
        else:
            ra, rf = self.ra, self.rf            # ra = tip (inner), rf = root (outer)
            fl = [ra + (rf - ra) * i / FLANK_STEPS for i in range(FLANK_STEPS + 1)]
            for k in range(n):
                c = rot + pitch * k              # space centre
                edges.append(cq.Edge.makeSpline([pol(r, c - self.half(r), z) for r in fl]))
                hr = self.half(rf)
                edges.append(cq.Edge.makeThreePointArc(pol(rf, c - hr, z), pol(rf, c, z), pol(rf, c + hr, z)))
                edges.append(cq.Edge.makeSpline([pol(r, c + self.half(r), z) for r in reversed(fl)]))
                ha = self.half(ra)
                a0, a1 = c + ha, c + pitch - ha
                edges.append(cq.Edge.makeThreePointArc(pol(ra, a0, z), pol(ra, (a0 + a1) / 2, z), pol(ra, a1, z)))
        return cq.Wire.assembleEdges(edges)


def herring(gear, rot, sign, z0, z1, z_ap=Z_AP):
    """herringbone solid from z0 to z1 with its V point at z_ap; the profile turns sign * |z - z_ap| * tan(b) / rp"""
    k = math.tan(BETA) / gear.rp
    parts = []
    if z0 < z_ap - 1e-9:
        h = z_ap - z0
        w = gear.wire(rot + sign * h * k, z0)
        parts.append(cq.Solid.extrudeLinearWithRotation(w, [], V(0, 0, z0), V(0, 0, h), math.degrees(-sign * h * k)))
    if z1 > z_ap + 1e-9:
        lo = max(z0, z_ap)
        h = z1 - lo
        r0 = rot + sign * (lo - z_ap) * k
        w = gear.wire(r0, lo)
        parts.append(cq.Solid.extrudeLinearWithRotation(w, [], V(0, 0, lo), V(0, 0, h), math.degrees(sign * h * k)))
    s = parts[0]
    for p in parts[1:]:
        s = s.fuse(p).clean()
    return one(s)


def make_gears(sun_bl, ring_bl):
    s_sun = MT * (math.pi / 2 + 2 * XS * math.tan(AN)) - sun_bl
    s_pl = MT * (math.pi / 2 + 2 * XP * math.tan(AN))
    rb_s, rb_p, rb_r = RS * math.cos(AT), RP * math.cos(AT), RR * math.cos(AT)
    asn = A * math.sin(AT)
    lim_sun_tip = math.sqrt(rb_s ** 2 + asn ** 2)            # beyond this the sun tip digs below the planet's base
    lim_pl_tip = math.sqrt(rb_p ** 2 + asn ** 2)
    lim_ring_tip = math.sqrt(rb_r ** 2 + asn ** 2)           # the ring's tips must stay OUTSIDE this
    ra_s = min(RS + MN * (1 + XS), lim_sun_tip - 0.05)
    ra_p = min(RP + MN * (1 + XP), lim_pl_tip - 0.05)
    rf_s = RS - MN * (1.25 - XS)
    rf_p = RP - MN * (1.25 - XP)
    ra_r = math.ceil((lim_ring_tip + 0.03) * 20) / 20       # ring tips: just clear of tip interference
    rf_r = A + ra_p + 0.25 * MN
    sun = Gear(ZS, XS, ra_s, rf_s, s_t=s_sun)
    pl = Gear(ZP, XP, ra_p, rf_p, s_t=s_pl)
    rg = Gear(ZR, -XS, ra_r, rf_r, e_t=s_pl + ring_bl, internal=True)
    info = dict(lim_sun_tip=lim_sun_tip, lim_pl_tip=lim_pl_tip, lim_ring_tip=lim_ring_tip)
    return sun, pl, rg, info


def planet_rot(k):
    """planet k's tooth phase at the V point (space toward the sun at phi = 0, rolled round a still sun)"""
    phi = 2 * math.pi * k / NP
    return math.pi + math.pi / ZP + phi * (ZS + ZP) / ZP, phi


def contact_ratios(sun, pl, rg):
    pb = math.pi * MT * math.cos(AT)
    asn = A * math.sin(AT)
    e_sp = (math.sqrt(sun.ra ** 2 - sun.rb ** 2) + math.sqrt(pl.ra ** 2 - pl.rb ** 2) - asn) / pb
    e_pr = (math.sqrt(pl.ra ** 2 - pl.rb ** 2) - math.sqrt(rg.ra ** 2 - rg.rb ** 2) + asn) / pb
    e_beta = (PW / 2) * math.sin(BETA) / (math.pi * MN)
    return e_sp, e_pr, e_beta


def tip_thickness(g):
    return 2 * g.ra * g.half(g.ra) if not g.internal else g.ra * (2 * math.pi / g.z - 2 * g.half(g.ra))


# ---------------------------------------------------------------- printed parts
def body(z0, h):
    return cyl(BODY_D, h, z0)


def groove_angles():
    return [GROOVE_START + 360.0 / N_GROOVE * k for k in range(N_GROOVE)]


def fluted(z0, h):
    """the ring halves' body: 70.5 cylinder, 12 round-bottom grooves full length, lips rounded"""
    b = cyl(BODY_D, h, z0)
    for a in groove_angles():
        b = b.cut(cyl(2 * GROOVE_R, h + 2, z0 - 1, *polar(BODY_D / 2, a)))
    half = math.degrees(2 * math.asin(GROOVE_R / 2 / (BODY_D / 2))) + 1.0     # lip angle from a groove centre + margin

    def lip(e):
        if e.geomType() != "LINE":
            return False
        c = e.Center()
        if abs(math.hypot(c.x, c.y) - BODY_D / 2) > 0.05 or abs(e.tangentAt(0.5).z) < 0.999:
            return False
        ang = math.degrees(math.atan2(c.y, c.x)) % 360
        return any(abs((ang - g + 180) % 360 - 180) < half for g in groove_angles())
    lips = [e for e in b.Edges() if lip(e)]
    return one(b.fillet(LIP_FILLET, lips))


def polar(r, a_deg):
    return r * math.cos(math.radians(a_deg)), r * math.sin(math.radians(a_deg))


def bolt_holes(d, z0, h):
    return [cyl(d, h + 2, z0 - 1, *polar(BOLT_R, a)) for a in BOLT_ANGLES]


def cone_at(r0, r1, h, z, x, y):
    return cq.Solid.makeCone(r0, r1, h, V(x, y, z))


def motor_plate():
    b = body(0.0, PLATE_T).fuse(cyl(REG_D, REG_H, PLATE_T)).clean()
    # 2026-11-25: the motor hangs on standoff tubes and its pilot no longer locates in the plate (the dog bone takes the
    # misalignment). The hub passes up through the centre hole and lifts out through it once the gearbox is off.
    cuts = [cyl(PLATE_HOLE, Z_SEAT + 2, -1),
            cyl(2 * 23.5, 0.4, Z_SEAT - 0.4)]                                   # relief under the inner race + disc
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * NEMA["screw"] / 2, sy * NEMA["screw"] / 2
            cuts += [cyl(M3["clear"], Z_SEAT + 2, -1, x, y), cyl(M3["cb_d"], 3.2 + 1, Z_SEAT - 3.2, x, y),
                     cyl(STANDOFF_D + 0.4, STANDOFF_POCKET + 0.01, -0.01, x, y)]   # standoff tube tops
    for a in BOLT_ANGLES:                       # clamp bolts: insert from the underside (the bolt pulls it in), clearance above
        x, y = polar(BOLT_R, a)
        cuts += [cyl(INSERT_HOLE, INSERT_DEPTH + 0.01, -0.01, x, y), cone_at(2.4, 2.1, 0.3, -0.001, x, y),
                 cyl(M3["clear"], PLATE_T + 1, INSERT_DEPTH - 0.01, x, y)]
    rp, pl = ARM_PILOT / 2, ARM_LEAD
    for a in ARM_ANGLES:                        # the arm's #4 x 3/4" wood screws, up from link C's plate, on into half A
        x, y = polar(ARM_R, a)
        cuts += [cyl(ARM_PILOT, PLATE_T + 2, -1, x, y),
                 cone_at(rp + pl + 0.5, rp, pl + 0.5, -0.5, x, y),
                 cone_at(rp, rp + pl + 0.5, pl + 0.5, PLATE_T - pl, x, y)]
    return one(cut_all(b, cuts))


def ring_half_a(rg, rot):
    z0, z1 = PLATE_T, Z_AP
    b = fluted(z0, z1 - z0)
    hole = herring(rg, rot, -1, Z_RING_A0, z1)
    cuts = [cyl(REG_HOLE, REG_RECESS, z0 - 0.01),                            # register on the plate boss
            cyl(POCKET_6808, Z_RING_A0 - Z_SEAT + 0.01, Z_SEAT),              # input 6808 seat
            hole]
    cuts += bolt_holes(FIT_HOLE, z0, z1 - z0)
    rp, pl = ARM_PILOT / 2, ARM_LEAD
    for a in ARM_ANGLES:                        # arm screw pilots on up to z 16 (cycloid.py's ARM_PILOT_TOP)
        x, y = polar(ARM_R, a)
        cuts += [cyl(ARM_PILOT, ARM_PILOT_TOP - z0 + 0.01, z0 - 0.01, x, y),
                 cone_at(rp + pl + 0.5, rp, pl + 0.5, z0 - 0.5, x, y)]
    return one(cut_all(b, cuts))


def ring_half_b(rg, rot):
    z0, z1 = Z_AP, Z_OUT_BRG1
    b = fluted(z0, z1 - z0)
    hole = herring(rg, rot, -1, z0, Z_RING_B1)
    tips = 2 * rg.ra
    cone = cq.Solid.makeCone(tips / 2, 25.0, tips / 2 - 25.0, V(0, 0, Z_RING_B1 - 0.01))   # 45 deg step in to the 50 bore
    cuts = [hole, cone,
            cyl(50.0, Z_LIP0 - Z_RING_B1 + 0.02, Z_RING_B1 - 0.01),            # spider zone
            cyl(B6808["outer_id"], Z_OUT_BRG0 - Z_LIP0 + 0.02, Z_LIP0 - 0.01),   # lip under the outer race
            cyl(POCKET_6808, Z_OUT_BRG1 - Z_OUT_BRG0 + 0.01, Z_OUT_BRG0)]      # output 6808 seat
    cuts += bolt_holes(FIT_HOLE, z0, z1 - z0)
    return one(cut_all(b, cuts))


def retainer():
    z0 = Z_OUT_BRG1
    b = body(z0, RET_T)
    cuts = [cyl(49.0, RET_T + 2, z0 - 1)] + bolt_holes(M3["clear"], z0, RET_T)
    return one(cut_all(b, cuts))


def input_shaft(sun_g):
    z0 = Z_SEAT
    disc = cyl(SEAT_6808_IN, BRG_W, z0)
    shoulder = cyl(42.0, Z_P0 - 0.7 - (z0 + BRG_W), z0 + BRG_W)            # on the inner race, 0.7 under the planets
    neck = cyl(10.0, 0.7 + 0.01, Z_P0 - 0.7)
    teeth = herring(sun_g, 0.0, +1, Z_P0, Z_P1)
    journal = cyl(JOURNAL, Z_SP0 + B6800["w"] - 0.3 - Z_P1, Z_P1 - 0.01)
    body = disc.fuse(shoulder).fuse(neck).fuse(teeth).fuse(journal).clean()
    # 2026-11-25: no motor shaft in here any more (solid sun); the dog bone's top hex drives a socket in the disc.
    # Three slots under the planets: the planet screws and their head spacers go in through them, before the input
    # 6808 is pressed on (it closes the slots' outer edge).
    cuts = [hex_prism(HEX_AF + HEX_CLEAR, SOCKET_DEPTH + 0.01, z0 - 0.01)]
    for k in range(NP):
        phi = 2 * math.pi * k / NP
        cuts.append(cyl(M4["head_d"] + 0.6, Z_P0 - 0.7 - z0 + 0.02, z0 - 0.01, A * math.cos(phi), A * math.sin(phi)))
    return one(cut_all(body, cuts))


def hex_prism(af, h, z0):
    """hex across flats af, flats facing 0, 60, ... deg (the bone's crowned flats face the same way)"""
    rc = af / math.sqrt(3)
    pts = [(rc * math.cos(math.radians(30 + 60 * k)), rc * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    return one(cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(h).val())


def xcyl(d, x0, x1, z, y=0.0):
    return cq.Solid.makeCylinder(d / 2, x1 - x0, V(x0, y, z), V(1, 0, 0))


def crowned_hex(zc):
    """ball-end hex: each flat is a cylinder of radius CROWN_R across it, touching HEX_AF / 2 at zc"""
    c = HEX_AF / 2 - CROWN_R
    b = cyl(HEX_CORNER_D, BALL_L, zc - BALL_L / 2)
    for k in range(6):
        a = math.radians(60 * k)
        nx, ny, tx, ty = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
        b = b.intersect(cq.Solid.makeCylinder(CROWN_R, 40.0, V(c * nx - 20 * tx, c * ny - 20 * ty, zc), V(tx, ty, 0)))
    return b


def dog_bone():
    b = crowned_hex(Z_BALL_LO).fuse(crowned_hex(Z_BALL_HI)).fuse(cyl(NECK_D, Z_BALL_HI - Z_BALL_LO, Z_BALL_LO))
    return one(b.clean())


def motor_hub():
    """covers the whole motor shaft (D bore, grub on the flat); the bone's bottom hex in its socket"""
    b = cyl(MHUB_D, Z_MHUB1 - Z_MHUB0, Z_MHUB0)
    r, ro = HUB_BORE / 2, MHUB_D / 2
    bore = cyl(HUB_BORE, Z_SHAFT_TIP + 0.5 - Z_MHUB0 + 1, Z_MHUB0 - 1)
    bore = bore.cut(box(r - FLAT_DEPTH, r + 1, -r - 1, r + 1, Z_MOTOR + FLAT_START, Z_SHAFT_TIP + 1))   # D past 9.5
    cuts = [bore, hex_prism(HEX_AF + HEX_CLEAR, SOCKET_DEPTH + 0.01, Z_MSOCK0),
            xcyl(INSERT_HOLE, ro - INSERT_DEPTH, ro + 1, GRUB_Z),                   # insert, on the flat side (+x)
            xcyl(3.2, r - FLAT_DEPTH - 0.5, ro - INSERT_DEPTH + 0.01, GRUB_Z)]      # grub on through to the flat
    return one(cut_all(b, cuts))


def standoff(x=0.0, y=0.0):
    return one(ring(STANDOFF_D, M3["clear"], STANDOFF_POCKET - Z_MOTOR, Z_MOTOR, x, y))


def sun_test(sun_g):
    """the sun alone (11 long) on a D5 bore, to try in the planets before printing the input shaft"""
    teeth = herring(sun_g, 0.0, +1, 0.0, PW, z_ap=PW / 2)
    r = HUB_BORE / 2
    bore = cyl(HUB_BORE, PW + 2, -1).cut(box(r - FLAT_DEPTH, r + 1, -r - 1, r + 1, -2, PW + 2))
    return one(teeth.cut(bore))


def planet(pl_g, rot):
    """one 624 pressed in from the spider side onto a lip on the motor side; the M4 head sits in the 8.0 bore below it"""
    body = herring(pl_g, rot, -1, Z_P0, Z_P1)
    cuts = [cyl(LIP_HOLE, Z_PB0 - Z_P0 + 0.02, Z_P0 - 0.01),
            cyl(POCKET_624, B624["w"] + 0.01, Z_PB0)]
    return one(cut_all(body, cuts))


def spider(boss_r):
    """boss_r: printed (free) radius, or the seated radius A for the assembly picture"""
    z0, zb1 = Z_SP0, Z_SP0 + BOSS_H
    rim = ring(2 * RIM_OUT, 2 * RIM_IN, Z_SP_DISC0 - z0, z0)
    hub = cyl(HUB_D, Z_SP_DISC0 - z0, z0)
    disc = cyl(2 * RIM_OUT, DISC_T, Z_SP_DISC0)
    shoulder = cyl(42.0, Z_OUT_BRG0 - Z_SHOULDER0, Z_SHOULDER0)
    collar = cyl(SEAT_6808_IN, BRG_W - 0.1, Z_OUT_BRG0)
    body = rim.fuse(hub).fuse(disc).fuse(shoulder).fuse(collar)
    y_rim = math.sqrt(RIM_IN ** 2 - boss_r ** 2)
    for k in range(NP):
        phi = 360.0 * k / NP
        boss = cyl(BOSS_D, BOSS_H, z0, boss_r, 0).fuse(cyl(NOSE_D, z0 - Z_P1 + 0.01, Z_P1, boss_r, 0))   # + its nose
        blade = box(boss_r - BLADE_T / 2, boss_r + BLADE_T / 2, 0.0, y_rim + 1.0, z0, zb1)
        # rigid spoke hub -> rim between this planet's blade tip and the next boss
        sp_a = math.radians(80.0)
        spoke = box(HUB_D / 2 - 1, RIM_IN + 0.5, -SPOKE_W / 2, SPOKE_W / 2, z0, Z_SP_DISC0).rotate(V(0, 0, 0), V(0, 0, 1), 80.0)
        piece = boss.fuse(blade).fuse(spoke).rotate(V(0, 0, 0), V(0, 0, 1), phi)
        body = body.fuse(piece)
    body = body.clean()
    cuts = [cyl(POCKET_6800, B6800["w"] + 0.01, z0 - 0.01), cyl(14.5, 0.4, z0 + B6800["w"])]
    for k in range(NP):
        phi = 2 * math.pi * k / NP
        x, y = boss_r * math.cos(phi), boss_r * math.sin(phi)
        cuts += [cyl(M4["tap"], BOSS_H + z0 - Z_P1 + 0.02, Z_P1 - 0.01, x, y),           # the M4 cuts its own thread
                 cone_at(M4["tap"] / 2 + 0.3, M4["tap"] / 2, 0.3, Z_P1 - 0.001, x, y),  # lead: the tip finds it 0.6 off
                 cyl(DISC_TIP_HOLE, DISC_T + 0.5 + 0.01, Z_SP_DISC0 - 0.01, A * math.cos(phi), A * math.sin(phi))]
    for a in (60.0, 180.0, 300.0):
        x, y = 10 * math.cos(math.radians(a)), 10 * math.sin(math.radians(a))
        cuts.append(cyl(M3["tap"], 8.0, Z_OUT_BRG0 + BRG_W - 0.1 - 8.0, x, y))
    return one(cut_all(body, cuts))


def output_disc():
    z0 = Z_OUT_BRG1
    body = cyl(47.0, OUTD_T, z0)
    cuts = [ring(47.2, 42.0, 0.3, z0 - 0.01)]                                  # clears the outer race, clamps the inner
    for a in (60.0, 180.0, 300.0):
        x, y = 10 * math.cos(math.radians(a)), 10 * math.sin(math.radians(a))
        cuts += [cyl(M3["clear"], OUTD_T + 2, z0 - 1, x, y), cyl(M3["cb_d"], OUTD_T - 2.8 + 0.01, z0 + 2.8, x, y)]
    for a in (0.0, 120.0, 240.0):                                             # his M3 x 4 x 5 inserts, the old r 15 circle
        x, y = 15 * math.cos(math.radians(a)), 15 * math.sin(math.radians(a))
        cuts += [cyl(4.2, 4.5 + 0.01, z0 + OUTD_T - 4.5, x, y),
                 cq.Solid.makeCone(2.4, 2.1, 0.3, V(x, y, z0 + OUTD_T - 0.3))]
    return one(cut_all(body, cuts))


# ---------------------------------------------------------------- bought parts (envelopes)
def ball_bearing(b, z):
    inner = ring(b["inner_od"], b["d"], b["w"], z)
    outer = ring(b["D"], b["outer_id"], b["w"], z)
    seal = ring(b["outer_id"] + 0.02, b["inner_od"] - 0.02, 0.3, z + 0.2).fuse(
        ring(b["outer_id"] + 0.02, b["inner_od"] - 0.02, 0.3, z + b["w"] - 0.5))
    return inner.fuse(outer).fuse(seal)


def cap_screw(length, x, y, z_under, up=True):
    """up: head below z_under, shank going up; else head above, shank going down"""
    head = cyl(M3["head_d"], M3["head_h"], z_under - M3["head_h"] if up else z_under, x, y)
    shank = cyl(M3["major"], length, z_under if up else z_under - length, x, y)
    return head.fuse(shank)


def nema17():
    f = NEMA["face"] / 2
    body = box(-f, f, -f, f, -NEMA["body"], 0.0)
    pilot = cyl(NEMA["pilot_d"], NEMA["pilot_h"])
    r = SHAFT_D / 2
    shaft = cyl(SHAFT_D, SHAFT_LEN).cut(box(r - FLAT_DEPTH, r + 1, -r - 1, r + 1, 5.0, SHAFT_LEN + 1))
    return body.fuse(pilot).fuse(shaft)


# ---------------------------------------------------------------- blade sizing
E_MAT = {"PA12": 1600.0, "PLA": 3500.0}


def blade_size(mat, preload, defl, boss_r):
    """thickness so the boss pushes `preload` N out when held `defl` inside its printed place; stress at that"""
    y_rim = math.sqrt(RIM_IN ** 2 - boss_r ** 2)
    c = BOSS_D / 2                       # boss: rigid
    l = y_rim - c                        # free blade
    k = preload / defl
    i_need = (l ** 3 / 3 + l ** 2 * c + l * c ** 2) * k / E_MAT[mat]
    t = (12 * i_need / BOSS_H) ** (1 / 3)
    sigma = 6 * preload * (l + c) / (BOSS_H * t ** 2)
    k_tan = E_MAT[mat] * BOSS_H * t / (l + c)
    return t, sigma, l, k_tan


# ---------------------------------------------------------------- loads
def loads(sun, pl, rg):
    """per N.m at the output: Lewis bending, Hertz contact, pin and bearing loads (load share 1.3, dynamic 1.2)"""
    YT = [(12, .245), (13, .261), (14, .277), (15, .290), (16, .296), (17, .303), (18, .309), (19, .314), (20, .322),
          (21, .328), (22, .331), (24, .337), (26, .346), (28, .353), (30, .359), (34, .371), (38, .384)]
    def Y(z):
        if z <= YT[0][0]:
            return YT[0][1] * z / YT[0][0]
        for (a, ya), (b, yb) in zip(YT, YT[1:]):
            if a <= z <= b:
                return ya + (yb - ya) * (z - a) / (b - a)
        return 0.39
    ft = 1e3 / RATIO / RS / NP * 1.3                         # N per mesh per N.m out
    zv = lambda z: z / math.cos(BETA) ** 3                   # noqa: E731
    sb_s = ft * 1.2 / (PW * MN * Y(zv(ZS)) * (1 + 0.6 * XS))
    sb_p = ft * 1.2 / (PW * MN * Y(zv(ZP)) * (1 - 0.6 * XS))
    sb_r = ft * 1.2 / (PW * MN * 0.485 * 1.15)
    bb = math.atan(math.tan(BETA) * math.cos(AT))
    estar = 1 / (2 * (1 - 0.35 ** 2) / 3500.0)
    L = PW / math.cos(bb)
    fn = ft * 1.2 / (math.cos(AT) * math.cos(bb))
    rho = lambda r: r * math.sin(AT) / math.cos(bb)          # noqa: E731
    sh_s = math.sqrt(fn / L * (1 / rho(RS) + 1 / rho(RP)) * estar / math.pi)
    sh_r = math.sqrt(fn / L * (1 / rho(RP) - 1 / rho(RR)) * estar / math.pi)
    pin = 1e3 / A / NP * 1.3
    return dict(ft=ft, sb_s=sb_s, sb_p=sb_p, sb_r=sb_r, sh_s=sh_s, sh_r=sh_r, pin=pin)


# ---------------------------------------------------------------- main
def main():
    global BLADE_T
    ap = argparse.ArgumentParser()
    ap.add_argument("--sun-backlash", type=float, default=0.06, help="sun mesh clearance at the pitch circle, mm")
    ap.add_argument("--ring-backlash", type=float, default=0.0)
    ap.add_argument("--spider-mat", default="PLA", choices=list(E_MAT))   # Mike prints it in PLA (2026-11-24)
    ap.add_argument("--preload", type=float, default=8.0, help="N each planet is pushed into the ring")
    ap.add_argument("--preload-defl", type=float, default=0.6, help="mm the bosses are printed outside their seat")
    ap.add_argument("--no-checks", action="store_true")
    a = ap.parse_args()

    sun, pl, rg, info = make_gears(a.sun_backlash, a.ring_backlash)
    boss_free = A + a.preload_defl
    BLADE_T, sigma, l_free, k_tan = blade_size(a.spider_mat, a.preload, a.preload_defl, boss_free)
    BLADE_T = round(BLADE_T, 2)

    rep = []
    P = rep.append
    P("herringbone planetary (Mike's sketch, 2026-11-23) - tools/herringbone_planetary.py")
    P(f"module {MN} normal, {math.degrees(AN):.0f} deg, herringbone {math.degrees(BETA):.0f} deg; transverse module {MT:.4f}, "
      f"transverse pressure angle {math.degrees(AT):.2f} deg")
    P(f"sun {ZS} / planets {NP} x {ZP} / ring {ZR}: ratio {RATIO:.2f}:1 (carrier out, ring fixed)")
    P(f"planet circle r {A:.3f}; sun shift +{XS:.3f}, planet {XP:.3f}")
    for nm, g in (("sun", sun), ("planet", pl)):
        P(f"  {nm:6s}: pitch d {2 * g.rp:.3f}  base d {2 * g.rb:.3f}  tip d {2 * g.ra:.3f}  root d {2 * g.rf:.3f}  "
          f"tip land {tip_thickness(g):.2f}")
    P(f"  ring  : pitch d {2 * rg.rp:.3f}  base d {2 * rg.rb:.3f}  TIP d {2 * rg.ra:.3f}  root d {2 * rg.rf:.3f}  "
      f"tip land {tip_thickness(rg):.2f}")
    P(f"  tip-interference limits: sun tip d <= {2 * info['lim_sun_tip']:.3f}, planet tip d <= {2 * info['lim_pl_tip']:.3f}, "
      f"ring tip d >= {2 * info['lim_ring_tip']:.3f}")
    P(f"  -> the ring's tips can't be 52.0: at 52.0 they would dig into the planet roots. They are {2 * rg.ra:.2f}; "
      f"the 6808 seats are plain 52.0 bores beside the teeth, and the bearing still slides in through the teeth.")
    e_sp, e_pr, e_b = contact_ratios(sun, pl, rg)
    P(f"  contact ratio: sun-planet {e_sp:.2f}, planet-ring {e_pr:.2f}, + helix overlap {e_b:.2f} per half")
    P(f"  sun mesh clearance {a.sun_backlash:.3f} at the pitch circle ({math.degrees(a.sun_backlash / 2 / A) * 60:.1f} arcmin at the output); "
      f"ring mesh {a.ring_backlash:.3f} (the blades seat the planets)")
    P(f"  clearances: sun tip to planet root {A - sun.ra - pl.rf:.3f}, planet tip to sun root {A - pl.ra - sun.rf:.3f}, "
      f"planet tip to ring root {rg.rf - A - pl.ra:.3f}")
    P("")
    P(f"spider ({a.spider_mat}): bosses printed at r {boss_free:.3f}, seated at r {A:.3f} -> {a.preload:.1f} N out per planet; "
      f"blades {BLADE_T:.2f} thick x {BOSS_H:.0f} high, {l_free:.1f} free; bending {sigma:.1f} MPa held; "
      f"wind-up {math.degrees(1e3 / A / NP * 1.3 / k_tan / A) * 60:.1f} arcmin per N.m out")
    P("")

    os.makedirs(os.path.join(OUT, "step"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "print"), exist_ok=True)

    parts = {}
    print("building ...", flush=True)
    parts["motor_plate"] = motor_plate()
    rot_ring = 0.0                                               # a space centred on the planet at phi = 0
    parts["ring_half_a"] = ring_half_a(rg, rot_ring)
    parts["ring_half_b"] = ring_half_b(rg, rot_ring)
    parts["retainer"] = retainer()
    parts["input_shaft"] = input_shaft(sun)
    for k in range(NP):
        rot, phi = planet_rot(k)
        parts[f"planet_{k + 1}"] = planet(pl, rot).translate(V(A * math.cos(phi), A * math.sin(phi), 0))
    parts["spider"] = spider(A)                                 # as assembled (blades straightened in the picture)
    spider_free = spider(boss_free)                              # as printed
    parts["output_disc"] = output_disc()
    coupling = {"motor_hub": motor_hub(), "dog_bone": dog_bone()}     # 2026-11-25: after the old parts (solid order)
    for sx in (-1, 1):
        for sy in (-1, 1):
            coupling[f"standoff_{len(coupling) - 1}"] = standoff(sx * NEMA["screw"] / 2, sy * NEMA["screw"] / 2)
    print("parts done", flush=True)

    bought = {}
    bought["motor_17HE08"] = nema17().translate(V(0, 0, Z_MOTOR))
    bought["bearing_6808_input"] = ball_bearing(B6808, Z_SEAT)
    bought["bearing_6808_output"] = ball_bearing(B6808, Z_OUT_BRG0)
    bought["bearing_6800_sun_tip"] = ball_bearing(B6800, Z_SP0)
    for k in range(NP):
        phi = 2 * math.pi * k / NP
        x, y = A * math.cos(phi), A * math.sin(phi)
        # 2026-11-25: three solids per planet, as before (14-22: were 2 x 3x6x2 + dowel), so the indices stay put
        z_head = Z_PB0 - HEAD_SPACER
        bought[f"bearing_624_{k + 1}"] = ball_bearing(B624, Z_PB0).translate(V(x, y, 0))
        bought[f"screw_M4x20_planet_{k + 1}"] = cyl(M4["head_d"], M4["head_h"], z_head - M4["head_h"], x, y).fuse(
            cyl(M4["major"], PLANET_SCREW, z_head, x, y))
        bought[f"spacer_head_{k + 1}"] = ring(SPACER_OD, SPACER_ID, HEAD_SPACER, z_head, x, y)   # (printed)
    z_under = Z_OUT_BRG1 + RET_T                                # heads on the retainer: reachable with the arm on
    bolt_len = 55.0
    for i, ang in enumerate(BOLT_ANGLES):
        x, y = polar(BOLT_R, ang)
        bought[f"bolt_M3x{bolt_len:.0f}_{i + 1}"] = cap_screw(bolt_len, x, y, z_under, up=False)
        bought[f"insert_M3x4x5_plate_{i + 1}"] = ring(4.9, 3.0, 4.0, 0.0, x, y)
    for i, ang in enumerate((60.0, 180.0, 300.0)):
        x, y = polar(10.0, ang)
        bought[f"screw_M3x10_disc_{i + 1}"] = cap_screw(10.0, x, y, Z_OUT_BRG1 + 2.8, up=False)
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * NEMA["screw"] / 2, sy * NEMA["screw"] / 2
            bought[f"screw_M3x{MOTOR_SCREW}_motor_{len([b for b in bought if 'motor_' in b and 'screw' in b]) + 1}"] = \
                cap_screw(float(MOTOR_SCREW), x, y, Z_SEAT - 3.2, up=False)
    for i, ang in enumerate((0.0, 120.0, 240.0)):
        x, y = 15 * math.cos(math.radians(ang)), 15 * math.sin(math.radians(ang))
        bought[f"insert_M3x4x5_{i + 1}"] = ring(4.9, 3.0, 4.0, Z_OUT_BRG1 + OUTD_T - 4.0, x, y)
    added = {}                              # 2026-11-25 coupling hardware: last of all, so the old solid indices stay put
    x_flat, ro = HUB_BORE / 2 - FLAT_DEPTH, MHUB_D / 2
    added["grub_M3x6_hub"] = xcyl(M3["major"], x_flat, x_flat + 6.0, GRUB_Z)
    added["insert_M3x4x5_hub"] = xcyl(4.9, ro - 4.0, ro, GRUB_Z).cut(xcyl(3.0, ro - 5.0, ro + 1.0, GRUB_Z))

    # ------------------------------------------------ checks
    if not a.no_checks:
        P("checks")
        P(f"  part volumes (cm3): " + ", ".join(f"{n} {s.Volume() / 1e3:.2f}" for n, s in list(parts.items()) +
                                                 list(coupling.items())[:3]))
        # gear mesh through a sun tooth pitch: carrier still, sun turns e, planets -e*Zs/Zp, ring -e*Zs/Zr
        worst = 0.0
        lines = []
        for step in range(4):
            e = 2 * math.pi / ZS * step / 4
            s_g = parts["input_shaft"].rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(e))
            ra_ = parts["ring_half_a"].rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(-e * ZS / ZR))
            rb_ = parts["ring_half_b"].rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(-e * ZS / ZR))
            for k in range(NP):
                phi = 2 * math.pi * k / NP
                c = V(A * math.cos(phi), A * math.sin(phi), 0)
                p = parts[f"planet_{k + 1}"].rotate(c, c + V(0, 0, 1), math.degrees(-e * ZS / ZP))
                v1 = p.intersect(s_g).Volume()
                v2 = p.intersect(ra_).Volume() + p.intersect(rb_).Volume()
                worst = max(worst, v1, v2)
                lines.append(f"    sun turned {math.degrees(e):5.1f} deg, planet {k + 1}: overlap with sun {v1:.4f}, ring {v2:.4f} mm3")
        P(f"  gear mesh (assembled, planets seated): worst overlap {worst:.4f} mm3")
        rep.extend(lines)
        # every pair of parts in the assembly
        allp = dict(parts)
        allp.update(coupling)
        allp.update(bought)
        allp.update(added)
        names = list(allp)
        bbs = {n: allp[n].BoundingBox() for n in names}
        hits = []
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                b1, b2 = bbs[names[i]], bbs[names[j]]
                if b1.xmax < b2.xmin or b2.xmax < b1.xmin or b1.ymax < b2.ymin or b2.ymax < b1.ymin or \
                        b1.zmax < b2.zmin or b2.zmax < b1.zmin:
                    continue
                v = allp[names[i]].intersect(allp[names[j]]).Volume()
                if v > 0.01:
                    hits.append((v, names[i], names[j]))
        P(f"  part overlaps > 0.01 mm3: {len(hits)}")
        for v, n1, n2 in sorted(hits, reverse=True):
            P(f"    {n1} x {n2}: {v:.3f} mm3")
        # dog bone: the motor off-centre by e (the input shaft fixed): the bone rocks about its top ball centre
        bone, hub, ish = coupling["dog_bone"], coupling["motor_hub"], parts["input_shaft"]
        span = Z_BALL_HI - Z_BALL_LO
        free = {}
        for phi, toward in ((0.0, "a flat"), (30.0, "a corner")):
            free[toward] = 0.0
            for e in (0.2, 0.4, 0.6, 0.8, 1.0):
                c, u = V(0, 0, Z_BALL_HI), V(-math.sin(math.radians(phi)), math.cos(math.radians(phi)), 0)
                b = bone.rotate(c, c + u, -math.degrees(math.atan(e / span)))
                h = hub.translate(V(e * math.cos(math.radians(phi)), e * math.sin(math.radians(phi)), 0))
                if b.intersect(ish).Volume() > 0.01 or b.intersect(h).Volume() > 0.01:
                    break
                free[toward] = e
        lo, hi = 0.0, 5.0                     # free turn of the bone in a socket (as drawn; printed holes come out smaller)
        for _ in range(14):
            mid = (lo + hi) / 2
            if bone.rotate(V(0, 0, 0), V(0, 0, 1), mid).intersect(ish).Volume() > 0.001:
                hi = mid
            else:
                lo = mid
        P(f"  dog bone: joint centres {span:.1f} apart; runs free with the motor off-centre up to "
          f"{free['a flat']:.1f} (toward a flat) / {free['a corner']:.1f} (toward a corner), "
          f"tilt {math.degrees(math.atan(free['a flat'] / span)):.1f} deg")
        P(f"    free turn in a socket as drawn: +-{lo:.2f} deg per end -> +-{2 * lo / RATIO * 60:.0f} arcmin at the output "
          f"(both ends; printed sockets come out smaller, so less)")
        P("")

    # ------------------------------------------------ loads
    L = loads(sun, pl, rg)
    P("loads per N.m at the output (load share 1.3, dynamic 1.2; PLA E 3.5 GPa)")
    P(f"  tooth force {L['ft']:.1f} N per mesh; bending sun {L['sb_s']:.1f}, planet {L['sb_p']:.1f}, ring {L['sb_r']:.1f} MPa; "
      f"contact sun {L['sh_s']:.0f}, ring {L['sh_r']:.0f} MPa (x sqrt of the torque)")
    P(f"  planet screw {L['pin']:.0f} N on its one 624 (static rating ~490 N)")
    # thin rim: the motor-side helix half sits over the 13.2 head pocket. Free ring under the tooth force of one helix
    # half on each mesh: M ~ F r / pi, section (PW / 2) x t
    rim_t = pl.rf - LIP_HOLE / 2
    rim_r = pl.rf - rim_t / 2
    sig_rim = L["ft"] * 1.2 / 2 * rim_r / math.pi / ((PW / 2) * rim_t ** 2 / 6)
    P(f"  planet rim: motor-side teeth on {rim_t:.2f} over the {LIP_HOLE} head pocket (ring bending {sig_rim:.1f} MPa on top "
      f"of the tooth bending); spider-side teeth on {pl.rf - POCKET_624 / 2:.2f} pressed on the steel 624")
    worst_b = max(L["sb_s"], L["sb_p"] + sig_rim, L["sb_r"])
    P(f"  safe output: teeth {25 / worst_b:.1f} N.m at 25 MPa (planet rim included); bearings {490 / L['pin']:.1f} N.m at "
      f"their static rating")
    for nm, t in (("17HE08 pancake (0.115 N.m pull-out)", 0.115), ("17HS19 (0.40 N.m pull-out)", 0.40),
                  ("17HS19 holding (0.59 N.m)", 0.59)):
        out = t * RATIO * 0.91
        P(f"  {nm}: {out:.2f} N.m out at ~91% -> bending {worst_b * out:.1f} MPa, sun contact {L['sh_s'] * math.sqrt(out):.0f} MPa")
    P(f"  dog bone (PLA, hex {HEX_AF:g} across flats, flats crowned R {CROWN_R:g}; 3 of the 6 flats carrying, 2 mm of "
      f"contact each):")
    xc = 0.8 * math.sqrt((HEX_CORNER_D / 2) ** 2 - (HEX_AF / 2) ** 2)          # contact lever, near the cut corner
    estar = 3500.0 / (2 * (1 - 0.35 ** 2))
    for nm, t in (("17HE08 pull-out", 0.115), ("17HS19 holding", 0.59)):
        f = t * 1e3 / (3 * xc)
        bw = math.sqrt(4 * f * CROWN_R / (math.pi * 2.0 * estar))
        P(f"    {nm} {t} N.m: {f:.0f} N per flat -> contact {2 * f / (math.pi * bw * 2.0):.0f} MPa, neck twist "
          f"{16 * t * 1e3 / (math.pi * NECK_D ** 3):.1f} MPa (PLA yields ~60)")
    tube_a = math.pi / 4 * (STANDOFF_D ** 2 - M3["clear"] ** 2)
    P(f"  standoff tubes: snug the M3 x {MOTOR_SCREW}s (~300 N -> {300 / tube_a:.0f} MPa in a tube); full M3 torque "
      f"(~1 kN, {1000 / tube_a:.0f} MPa) would creep the PLA")
    P("")
    P("arm fit (drops in where the spread cycloid sits: claw_adapter.py / mockup_link.py frame, z 0 = top of link C's plate)")
    P(f"  body {BODY_D} like the cycloid; output face z {Z_OUT_FACE} (spread cycloid: 59.2); 3 x M3 on r 15 at 0/120/240 (same)")
    P(f"  arm screws: 6 x #4 x 3/4\" wood screws on r {ARM_R} at " + ", ".join(f"{x:.2f}" for x in ARM_ANGLES) +
      f" deg, D{ARM_PILOT} pilots up to z {ARM_PILOT_TOP} (cycloid.py's ARM_SCREW_*): through the plate into half A")
    P(f"  motor: face z {Z_MOTOR} on four {STANDOFF_D:g} mm standoff tubes, bottom z {Z_MOTOR - NEMA['body']:.1f} "
      f"(was {-NEMA['body']:.1f}): through link C's 44 square opening in its 8 plate; the hub's grub at z {GRUB_Z:.1f}, "
      f"{-8.0 - GRUB_Z:.1f} under link C's plate")
    P(f"  clamp bolt heads on the retainer: top {Z_OUT_BRG1 + RET_T + M3['head_h']:.1f}, "
      f"{Z_OUT_FACE - (Z_OUT_BRG1 + RET_T + M3['head_h']):.1f} under the claw cup")
    def ow(r, a, rad):                      # wall from a hole to the nearest groove and to the OD
        gx = [polar(BODY_D / 2, g) for g in groove_angles()]
        hx, hy = polar(r, a)
        to_groove = min(math.hypot(hx - x, hy - y) for x, y in gx) - rad - GROOVE_R
        return min(to_groove, BODY_D / 2 - r - rad)
    P(f"  grooved OD (spline lug-nut look): {N_GROOVE} grooves {2 * GROOVE_R:.0f} wide x {GROOVE_R:.0f} deep at "
      f"{GROOVE_START:.0f} + {360 / N_GROOVE:.0f}k deg, lips rounded {LIP_FILLET}; every screw on a land")
    P(f"    walls: arm pilots {min(ow(ARM_R, a, ARM_PILOT / 2) for a in ARM_ANGLES):.2f}, clamp bolt holes "
      f"{min(ow(BOLT_R, a, FIT_HOLE / 2) for a in BOLT_ANGLES):.2f}, plate inserts to the OD "
      f"{BODY_D / 2 - BOLT_R - 2.5:.2f}; groove floor to the ring's tooth roots {BODY_D / 2 - GROOVE_R - rg.rf:.2f}")
    if not a.no_checks:
        smooth = {n: cyl(BODY_D, h, 0).Volume() - fluted(0, h).Volume() for n, h in (("ring_half_a", Z_AP - PLATE_T),
                                                                                       ("ring_half_b", Z_OUT_BRG1 - Z_AP))}
        P(f"    the grooves take out {smooth['ring_half_a'] / 1e3:.2f} + {smooth['ring_half_b'] / 1e3:.2f} cm3")
    P("")
    P("hardware")
    P("  2 x 6808 (40x52x7), 1 x 6800 (10x19x5), 3 x 624 (4x13x5), 3 x M4 x 20 cap screws (the planet axles, heads")
    P(f"    inside the planets; they cut their own thread in the {M4['tap']} boss holes, {BOSS_H - 0.5:.1f} "
      f"deep); printed per planet: a {HEAD_SPACER} head spacer ({SPACER_ID} x {SPACER_OD}, through the lip); on the")
    P(f"    other side the spider's {NOSE_D} boss nose. Both sit on the 624's inner ring only")
    P(f"  3 x M3 x {bolt_len:.0f} partly threaded, heads on the retainer, through half B and half A into 3 x M3 x 4 x 5")
    P("    heat-set inserts in the plate's underside (the bolt pulls each insert up into the plate)")
    P(f"  4 x M3 x {MOTOR_SCREW} motor screws (from inside the plate, heads 0.2 under the input 6808, down through the")
    P(f"    standoff tubes, {MOTOR_SCREW - (Z_SEAT - 3.2 - Z_MOTOR):.1f} into the motor), 3 x M3 x 10 output disc screws")
    P("  1 x M3 x 6 grub + 1 x M3 x 4 x 5 heat-set insert in the motor hub (on the shaft's flat)")
    P("  3 x M3 x 4 x 5 heat-set inserts in the output disc (r 15, 0/120/240 deg: the old output circle)")
    P("")
    P("assembly")
    P("  1 a 624 pressed into each planet from the spider side, down onto its lip")
    P("  2 spider into half B from the V end; output 6808 onto its collar from the far end; output disc screwed on")
    P("    (clamps the inner race); retainer on the half B end")
    P("  3 the three planets meshed round the input shaft's sun (its 6808 NOT on yet)")
    P("  4 that cluster straight into half B (the planets spin as they go in), each planet over its boss nose; turn")
    P("    the input shaft so its 3 slots sit under the planets; through each slot a head spacer and an M4 x 20 up through")
    P("    the planet's 624 into the nose: its tip pulls the boss in the 0.6 as it starts its thread; snug, 3 mm key")
    P("  5 input 6808 pressed onto the input shaft's disc (it closes the slots); half A over the planets from the motor")
    P("    side (turning), the 6808 into its seat")
    P("  6 insert into the motor hub, hub onto the motor shaft, grub on the flat; motor screwed to the plate through the")
    P("    4 standoff tubes (heads inside the plate; snug: PLA tubes); dog bone into the hub; plate on, turning the output a")
    P("    little until the bone's top hex drops into the input shaft's socket; the 3 long bolts down from the retainer")
    P("    into the plate's inserts. On the arm: link C's 6 wood screws up into the plate")
    P("  service (hub or bone): off the arm (6 wood screws), out with the 3 long bolts, lift the gearbox off the plate:")
    P(f"    the bone pulls out; loosen the grub (from the side, between the standoffs) and the hub lifts off the shaft")
    P(f"    through the plate's {PLATE_HOLE:g} hole. The motor stays on the plate.")
    P("")
    P("print")
    P("  motor plate: motor side down. half A: motor end down. half B: V end down. retainer: flat. output disc: outer")
    P("  face down (insert holes on the bed).")
    P("  input shaft: disc down. planets: motor face down (624 pocket up). sun tests: either face.")
    P("  spider: output face (collar end) down, boss noses up (Mike, 2026-11-26). Support under the back disc's rim and in")
    P("  the 1.0 gap under the blades and bosses: clear ALL of it out of that gap, or the blades can't flex.")
    P("  motor hub: bore end down (socket up). dog bone: standing on end. standoff: print 4, standing.")
    P("  head spacer: print 3, flat.")
    text = "\n".join(rep)
    print(text)

    # ------------------------------------------------ STEP
    from cadquery import exporters
    for n, s in list(parts.items()) + list(coupling.items()):
        exporters.export(cq.Workplane().add(s), os.path.join(OUT, "step", f"{n}.step"))
    exporters.export(cq.Workplane().add(spider_free), os.path.join(OUT, "step", "spider_as_printed.step"))
    # Robot arm control and kinematics reads assembly.step BY SOLID INDEX (2026-11-23: housing on J3 = 0-4, 10-13,
    # 23-28, 32-35; the rest turns with J4). Keep the part order (parts, then bought) or tell that session.
    assy = cq.Assembly(name="herringbone_planetary")
    colors = dict(motor_plate=(0.55, 0.55, 0.58), ring_half_a=(0.27, 0.51, 0.71), ring_half_b=(0.53, 0.81, 0.98),
                  retainer=(0.55, 0.55, 0.58), input_shaft=(1.0, 0.55, 0.0), spider=(0.2, 0.65, 0.3),
                  output_disc=(0.85, 0.7, 0.2))
    for n, s in parts.items():
        assy.add(s, name=n, color=cq.Color(*colors.get(n, (0.9, 0.35, 0.25))))
    for n, s in bought.items():
        assy.add(s, name=n, color=cq.Color(*((0.1, 0.1, 0.1) if "motor_17" in n else (0.75, 0.75, 0.78))))
    # 2026-11-25: the coupling parts and hardware go LAST (indices 39 on), so every older index is unchanged
    for n, s in coupling.items():
        assy.add(s, name=n, color=cq.Color(*{"motor_hub": (0.25, 0.45, 0.9), "dog_bone": (0.85, 0.25, 0.6)}.get(
            n, (0.55, 0.55, 0.58))))
    for n, s in added.items():
        assy.add(s, name=n, color=cq.Color(0.75, 0.75, 0.78))
    assy.save(os.path.join(OUT, "step", "assembly.step"))

    def to_bed(s, flip=False):
        if flip:
            s = s.rotate(V(0, 0, 0), V(1, 0, 0), 180)
        bb = s.BoundingBox()
        return s.translate(V(0, 0, -bb.zmin))
    printed = dict(parts)
    printed["spider"] = spider_free
    printed.update(motor_hub=coupling["motor_hub"], dog_bone=coupling["dog_bone"], standoff=standoff(),
                   spacer_head=ring(SPACER_OD, SPACER_ID, HEAD_SPACER))
    for n, s in printed.items():
        flip = n in ("output_disc", "spider")  # output disc outer face down; spider collar end down (noses up)
        exporters.export(cq.Workplane().add(to_bed(s, flip)), os.path.join(OUT, "print", f"{n}.step"))
    for i, bl in enumerate((0.0, 0.03, 0.06, 0.09, 0.12, 0.15)):
        sg, _, _, _ = make_gears(bl, a.ring_backlash)
        exporters.export(cq.Workplane().add(sun_test(sg)), os.path.join(OUT, "print", f"sun_test_{int(bl * 1000):03d}.step"))
    with open(os.path.join(OUT, "report.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print("written to", OUT)


BLADE_T = 2.1

if __name__ == "__main__":
    main()

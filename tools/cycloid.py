#!/usr/bin/env python3
"""
cycloid.py - 37:1 cycloidal drive straight on the 17HS19 (Atlas-drive style), for the arm joints.

Mike's choices (2026-11-03): layout A of gear_ratio.py's study; 38 ring pins + 8 output pins, all 3 x 20 mm
hardened dowels (ISO 8734 m6); output on two of his 6808s; a 6800 (10 x 19 x 5) in each disc on a printed
eccentric; one switch between steel dowels and PRINTED pins for testing; holes through wherever a longer
screw can be used.  M3 x 8 into the motor, M3 x 25..45 for the lid, M3 x 30..40 through his 70.5 x 24 output
disc into his M3 x 4 x 5 heat-set inserts.

    python cycloid.py [--clear 0.10] [--hub-bore 5.0] [--cam-d 10.0] [--ecc-bore 19.0] [--pin-hole 3.0] [--insert-hole 4.2]

Writes robo_chess/cycloid/ (robo_chess is the folder above tools/; complete assembly/cycloid gearbox is a junction to it): step/ (every part in its assembled position + one assembly per pin
variant), stl/ (every part flat as printed; the two parts that change with the pins come in both
variants), report.txt (sizes, motion checks, part overlaps, loads, hardware list).

z = 0 is the motor face.  Stack (mm):
   0.0 .. 7.2   housing floor (motor screws from inside, heads 0.2 below its top)
   7.7 .. 9.7   output support ring (holds the lower ends of the 8 output pins)
  10.2 .. 16.2  disc 1 (eccentric +e)       16.7 .. 22.7  disc 2 (eccentric -e, the same part turned 180 deg)
  23.2 .. 29.2  output flange back disc (upper ends of the output pins) inside the pin ring
  29.2 .. 43.2  two 6808s on the output journal, outer races in the lid
  43.2 .. 45.2  clamp ring on the upper inner race; the 70.5 output disc sits on it
Ring pins: 3.5 mm in the floor, then in grooves in the housing wall that wrap 211 deg round them, tops
0.5 mm into the pin ring.  Output pins: 2 mm through the support ring, 4.5 mm into the flange.
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

import gearbox as G  # noqa: E402
import gearbox_step as GS  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

OUT = os.path.join(os.path.dirname(HERE), "cycloid")   # robo_chess/cycloid (also seen as complete assembly/cycloid gearbox: a junction)

# ---------------------------------------------------------------- cycloid
N_PINS = 38                  # ring pins; lobes = 37 = ratio
LOBES = N_PINS - 1
R_PINS = 28.0                # ring pin circle radius
PIN_D = 3.0                  # dowel
ECC = 0.368                  # eccentricity: K1 = ECC * N_PINS / R_PINS = 0.50 (the tested printed drive: 0.49)
CLEAR = 0.10                 # disc profile pulled in this far all round (running clearance + print), --clear
N_OUT = 8                    # output pins
R_OUT = 20.5                 # output pin circle radius
OUT_ANGLE0 = 22.5            # first output pin angle, deg (clear of the 3 output screws at 0/120/240)
HOLE_CLEAR = 0.05            # output pin hole: pin + 2 e + this each side
WALL_R = 27.6                # housing wall inner radius: the ring pins sit 0.4 mm proud of it

# ---------------------------------------------------------------- stack
FLOOR_T = 7.2                # 4.0 under the motor screw heads + 3.2 counterbore
GAP = 0.5
RING_T = 2.0                 # output support ring
DISC_T = 6.0
Z_RING = FLOOR_T + GAP                       # 7.7
Z_D1 = Z_RING + RING_T + GAP                 # 10.2
Z_D2 = Z_D1 + DISC_T + GAP                   # 16.7
Z_TOP = Z_D2 + DISC_T + GAP                  # 23.2: housing top, pin ring and back disc bottoms
BACK_T = 6.0
Z_BRG = Z_TOP + BACK_T                       # 29.2: lower 6808
Z_BRG_TOP = Z_BRG + 2 * G.CAGE_W             # 43.2
CLAMP_T = 2.0
Z_DISC_OUT = Z_BRG_TOP + CLAMP_T             # 45.2: the 70.5 output disc
LIP_T = 1.5
Z_LID_TOP = Z_BRG_TOP + LIP_T                # 44.7
LID_STEP_R = 28.5                            # lid top steps down outside this radius ...
Z_LID_STEP = Z_LID_TOP - 3.0                 # ... to here, so the lid screw heads stay under the output disc
# SPREAD output bearings (the tube-link arm, 2026-11-04; Mike 2026-11-06: "if all the gearboxes are the same, we need
# one good working printable model"): the two 6808s pulled SPREAD apart for tilt stiffness (stacked, their ball rows are
# only 7 mm apart). The lid, the output journal and the clamp ring rise by SPREAD; a printed sleeve between the outer
# races and one between the inner races hold them apart - both bearings still slide in from the ends, as before.
#     python cycloid.py --spread 14     -> robo_chess/cycloid_spread/ (STEP only); 0 = the stock drive in cycloid/
SPREAD = 0.0
SPACER_OUT = (G.CAGE_POCKET - 0.3, 46.0)   # outer-race sleeve: OD (slides in the lid's bore), ID (bears on the outer
#                                            races only, r 23..26; the inner races end at r 21.25)
SPACER_IN = (44.0, G.CAGE_SEAT + 0.3)      # inner-race sleeve: OD (bears on the inner races only), ID (slides on the
#                                            journal); 1 mm between the two sleeves


def set_spread(sp):
    global SPREAD, Z_BRG_TOP, Z_DISC_OUT, Z_LID_TOP, Z_LID_STEP
    SPREAD = sp
    Z_BRG_TOP = Z_BRG + 2 * G.CAGE_W + sp
    Z_DISC_OUT = Z_BRG_TOP + CLAMP_T
    Z_LID_TOP = Z_BRG_TOP + LIP_T
    Z_LID_STEP = Z_LID_TOP - 3.0
PIN_LEN = 20.0
Z_PIN0 = FLOOR_T - 3.5                       # ring pins 3.7 .. 23.7
Z_OPIN0 = Z_RING                             # output pins 7.7 .. 27.7

# ---------------------------------------------------------------- bought parts
B6800_ID, B6800_OD, B6800_W = 10.0, 19.0, 5.0
B6800_INNER_OD, B6800_OUTER_ID = 12.4, 16.6  # check model only (envelope)
COLLAR_D = 11.0              # concentric hub collars: r 5.5 -/+ e stays on the 6800 inner rings (r 5.0..6.2)
HUB_BORE = 5.0               # --hub-bore
CAM_D = 10.0                 # --cam-d
ECC_BORE = 19.0              # --ecc-bore
# Mike (2026-11-06): "put a star-shaped pattern ... female on one, and male on the other. So the two lobes are locked
# together": the two eccentrics (always 180 deg apart) lock where they meet (z 15.7), so the lower one - all of it on
# the motor's D - drives the upper one, whose cam a short shaft only half fills (the wrist's 17HE08: 20 mm, ends at 20.0
# under the upper cam's 17.2..22.2). A male star on the lower cam's top, the socket in the upper collar's underside.
# Points stay inside the cam faces (nearest edge r 4.63) and the socket inside the upper collar (1.1 wall); both parts
# still print collar-down.
KEY_STAR = (6, 4.3, 3.4, 1.0, 1.2, 0.1)   # points, point r, valley r, male height, socket depth, socket clearance
PIN_HOLE = 3.0               # --pin-hole: printed hole for a press-fit dowel; drill to 3.0
PIN_CAP_HOLE = 3.3           # loose pockets over the ring pin tops in the pin ring
FLAT_START = 9.0             # 17HS19-2004S1: D flat starts 9 mm out (7.75 .. 10.25)
FLAT_LATE = 10.25
MOTOR_SCREW_L = 8
# Mike's inserts (2026-11-06, Amazon B0FDKYTMZ8): brass M3 x 4 x 5, spiral knurl, length 4 (listing 0.15 in), knurl OD 5
# (listing 0.19 in = 4.83). Hole a little over the knurl's root so the knurl cuts in; 0.5 deeper than the insert (the D3.2
# screw hole below takes the melt); a 0.3 x 45 deg lead-in starts it straight. INSERT_D is the knurl: walls only (the
# STEP's brass stand-in is the hole size, so the files stay overlap-free).
INSERT_D, INSERT_HOLE, INSERT_L = 5.0, 4.2, 4.0   # --insert-hole
INSERT_DEPTH = INSERT_L + 0.5
INSERT_LEAD = 0.3
# SHARED BOLT CIRCLES (Mike 2026-11-06: "If you change any of the bolt circles, you have to make sure that the arms are
# updated ... and also the claw. Wrist."): OUT_SCREW_* (the output face) and LID_SCREW_* (the housing / lid ring) are
# read by mockup_link.py (the arms: cup + elbow mounting ring), arm_tube.py (the shoulder column, cut from that link),
# claw_adapter.py (the claw mount), cycloid_standin.py, arm2_screw_nubs.py; Z_DISC_OUT 59.2 is hard-coded in
# mockup_link.py (45.2 + SPREAD) and claw_adapter.py (SPREAD_FLOOR). After changing any of them rebuild those, then
# print_parts.py, and run gearbox_fit_check.py until it says ALL FIT.
OUT_SCREW_R = 15.0           # his 70.5 x 24 disc: 3 x 4.4 holes on a 30 mm circle
OUT_SCREW_ANGLES = (0.0, 120.0, 240.0)
OUT_DISC_T, OUT_DISC_HOLE = 24.0, 4.4
LID_SCREW_R = 32.4
LID_SCREW_ANGLES = tuple(180.0 / N_PINS / 3 + 60.0 * k for k in range(6))   # 1.58 deg off the pin grid
# Lid screws (Mike 2026-11-06): "M3 by 55 millimeter partially threaded cap head screws, because M3 by 58 are hard to
# find". ISO 4762 / DIN 912 partly threaded M3: 18 mm of thread (b ref = 2d + 12), plain 2.95 shank above it. The
# housing's 2.5 self-tap hole opens to clearance down to just past the shank's end, so the plain shank never gets
# forced into the tapped hole (its walls are ~1.6 thick there); the thread bites only below.
LID_THREAD = 18.0
LID_SHANK_MARGIN = 0.7          # clearance goes this far below the shank's end (runout, length tolerance)


def lid_screw_len():
    """M3 lid screw length: 40 on the stock drive, 55 with the spread 14 (heads on the lid step)"""
    return 40 + 5 * math.ceil(SPREAD / 5)


def lid_shank_end():
    """z where the lid screw's plain shank ends and its thread starts"""
    return Z_LID_STEP - (lid_screw_len() - LID_THREAD)
CENTRE_POCKET_D = 20.0       # lightening pocket in the output journal face (22 until 2026-11-06: 20 leaves the 5 mm
#                              insert knurls 2.5 mm of wall to it)
Z_OUT_HOLE_BOTTOM = 28.5     # output screw holes stop here (pin sockets end at 27.7)

# ---------------------------------------------------------------- loads (gear_ratio.py, joint 3)
T_DESIGN = 9.38              # N*m: joint 3 straight out with gripper + piece, x 1.5
ETA = 0.65                   # measured on a printed cycloidal (HowToMechatronics)
T_STALL = 0.59 * LOBES * ETA
TEST_PIN_FORCE = 88.0        # N per disc where the tested printed 25:1 gave up (gear_ratio.cycloid_test_force)

NYLON = 1.10e-3
V = GS.V
cyl, ring, box = GS.cyl, GS.ring, GS.box
one_solid, cut_all = GS.one_solid, GS.cut_all


def polar(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)


def ring_pin_angles():
    return [360.0 * k / N_PINS for k in range(N_PINS)]


def out_pin_angles():
    return [OUT_ANGLE0 + 360.0 * k / N_OUT for k in range(N_OUT)]


# ---------------------------------------------------------------- the disc profile
def profile_xy(n_per_lobe=48, clear=None):
    """Disc outline in the disc's own frame (centre at the origin), for the pose where the disc centre is at
    (+ECC, 0) and the ring pins are at 360*k/38 deg: every pin then sits PIN_D/2 + clear from it."""
    c = CLEAR if clear is None else clear
    rr = PIN_D / 2 + c
    t = np.linspace(0, 2 * math.pi, LOBES * n_per_lobe, endpoint=False)
    psi = np.arctan2(np.sin((1 - N_PINS) * t), R_PINS / (ECC * N_PINS) - np.cos((1 - N_PINS) * t))
    x = R_PINS * np.cos(t) - rr * np.cos(t + psi) - ECC * np.cos(N_PINS * t)
    y = -R_PINS * np.sin(t) + rr * np.sin(t + psi) + ECC * np.sin(N_PINS * t)
    return np.stack([x, y], 1)


def disc_wire():
    """One spline per lobe, joined end to end."""
    n = 48
    p = profile_xy(n)
    edges = []
    for k in range(LOBES):
        idx = [(k * n + i) % len(p) for i in range(n + 1)]
        edges.append(cq.Edge.makeSpline([V(p[i, 0], p[i, 1]) for i in idx]))
    return cq.Wire.assembleEdges(edges)


# ---------------------------------------------------------------- 2D checks
def dist_to_polyline(pts, poly):
    a = poly
    ab = np.roll(poly, -1, 0) - a
    ab2 = (ab * ab).sum(1)
    out = np.empty(len(pts))
    for i, q in enumerate(pts):
        ap = q - a
        tt = np.clip((ap * ab).sum(1) / ab2, 0.0, 1.0)
        out[i] = np.sqrt(((ap - tt[:, None] * ab) ** 2).sum(1)).min()
    return out


def inside(pts, poly):
    a, b = poly, np.roll(poly, -1, 0)
    res = []
    for q in pts:
        cond = (a[:, 1] > q[1]) != (b[:, 1] > q[1])
        dy = np.where(b[:, 1] == a[:, 1], 1e-12, b[:, 1] - a[:, 1])
        xi = a[:, 0] + (q[1] - a[:, 1]) * (b[:, 0] - a[:, 0]) / dy
        res.append(np.count_nonzero(cond & (q[0] < xi)) % 2 == 1)
    return np.array(res)


def disc_pose(theta, which):
    """Disc centre and rotation (rad) for input angle theta. Disc 2 = disc 1's pose turned 180 deg about the axis."""
    phi = -theta / LOBES
    c = np.array([ECC * math.cos(theta), ECC * math.sin(theta)])
    if which == 2:
        return -c, phi + math.pi
    return c, phi


def to_disc_frame(pts, centre, phi):
    c, s = math.cos(-phi), math.sin(-phi)
    q = pts - centre
    return np.stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]], 1)


def motion_check(rep):
    w = rep.append
    poly = profile_xy(200)
    pins = np.array([polar(R_PINS, a) for a in ring_pin_angles()])
    rr = PIN_D / 2
    worst, pens, touch = 1e9, 0, []
    wall = 1e9
    hole_r = (PIN_D + 2 * ECC) / 2 + HOLE_CLEAR
    hole_gap = 1e9
    contact_ang = 0.0
    for deg in np.arange(0, 360, 1.0):
        th = math.radians(deg)
        for which in (1, 2):
            c, phi = disc_pose(th, which)
            q = to_disc_frame(pins, c, phi)
            d = dist_to_polyline(q, poly)
            if inside(q, poly).any():
                pens += 1
            worst = min(worst, d.min() - rr)
            touch.append(int(np.count_nonzero(d - rr < CLEAR + 0.02)))
            # disc against the housing wall
            rw = np.linalg.norm(poly, axis=1).max() + ECC
            wall = min(wall, WALL_R - rw)
            # output pins in their holes. The output turns with disc 1 (-theta/37); a hole of disc 2 is disc 1's
            # hole turned 180 deg about the axis, which lands on the output pin 4 places round (8 pins)
            phi_out = -th / LOBES
            for a in out_pin_angles():
                hole = c + np.array(polar(R_OUT, math.degrees(phi) + a))
                pins_out = np.array([polar(R_OUT, math.degrees(phi_out) + b) for b in out_pin_angles()])
                gap = hole_r - rr - np.linalg.norm(pins_out - hole, axis=1).min()
                hole_gap = min(hole_gap, gap)
            if deg % 10 == 0 and which == 1:
                # where on each pin the disc pushes, measured from 'straight in towards the axis'
                cph, sph = math.cos(phi), math.sin(phi)
                for k in range(N_PINS):
                    j = int(np.argmin(((poly - q[k]) ** 2).sum(1)))
                    n_disc = poly[j] - q[k]
                    n_world = np.array([cph * n_disc[0] - sph * n_disc[1], sph * n_disc[0] + cph * n_disc[1]])
                    inward = -pins[k] / np.linalg.norm(pins[k])
                    contact_ang = max(contact_ang, math.degrees(math.acos(np.clip(
                        n_world @ inward / np.linalg.norm(n_world), -1, 1))))
    w(f"Motion, input turned 0..360 deg in 1 deg steps, both discs (720 poses):")
    w(f"  disc never inside a ring pin: {'PASS' if pens == 0 else f'FAIL ({pens} poses)'}; least gap to a ring pin "
      f"{worst:.3f} mm (designed {CLEAR:g})")
    w(f"  ring pins within {CLEAR + 0.02:.2f} of the disc at once: {min(touch)} .. {max(touch)} of {N_PINS} "
      "(in a perfect print all share the load; in practice about half)")
    w(f"  disc to the housing wall {wall:.2f} mm; contact on the ring pins up to {contact_ang:.0f} deg off 'straight in' "
      f"(the wall leaves {math.degrees(math.acos((R_PINS - WALL_R) / (PIN_D / 2))):.0f} deg of each pin exposed)")
    w(f"  output pins in the disc holes: gap {hole_gap:.3f} mm all the way round")
    # backlash: lock the input at 0, turn the disc about its centre until it touches a pin
    c, phi = disc_pose(0.0, 1)
    plays = []
    for sgn in (1, -1):
        lo, hi = 0.0, 0.02
        for _ in range(40):
            mid = (lo + hi) / 2
            q = to_disc_frame(pins, c, phi + sgn * mid)
            if (dist_to_polyline(q, poly) - rr).min() > 0:
                lo = mid
            else:
                hi = mid
        plays.append(lo)
    play = sum(plays) + 2 * HOLE_CLEAR / R_OUT
    w(f"  backlash with the motor held: {math.degrees(play):.2f} deg at the output "
      f"(disc {math.degrees(sum(plays)):.2f} + output pin holes {math.degrees(2 * HOLE_CLEAR / R_OUT):.2f}) = "
      f"{math.radians(math.degrees(play)) * 487:.1f} mm at the wrist from joint 3 (487 mm)")
    rmin = np.linalg.norm(poly, axis=1).min()
    return rmin


# ---------------------------------------------------------------- printed parts (assembled position)
def housing(printed_pins):
    """Floor + wall. Steel: each ring pin in a 3.5 deep floor socket and a groove up the wall. Printed: the
    pins grow out of the floor, fused to the wall."""
    body = cyl(G.BODY_OD, Z_TOP)
    cuts = [cyl(WALL_R * 2, Z_TOP, FLOOR_T),
            cyl(22.2, 3.2, -1.0),                       # motor pilot 22 x 2
            cyl(8.0, FLOOR_T + 2, -1)]                  # shaft
    head_z = FLOOR_T - 0.2 - G.M3["head_h"]          # under-head plane 4.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * 15.5, sy * 15.5
            cuts.append(cyl(G.MOTOR_HOLE, FLOOR_T + 2, -1, x, y))
            cuts.append(cyl(G.COUNTERBORE_D, FLOOR_T, head_z, x, y))
    z_clear = lid_shank_end() - LID_SHANK_MARGIN      # partly threaded lid screws: shank clearance from the top to here
    for a in LID_SCREW_ANGLES:
        x, y = polar(LID_SCREW_R, a)
        cuts.append(cyl(G.BODY_HOLE, Z_TOP + 2, -1, x, y))
        if z_clear < Z_TOP:
            cuts.append(cyl(G.M3["clear"], Z_TOP + 1 - z_clear, z_clear, x, y))
    if not printed_pins:
        cuts += [cyl(PIN_HOLE, Z_TOP - Z_PIN0 + 1, Z_PIN0, *polar(R_PINS, a)) for a in ring_pin_angles()]
    b = cut_all(body, cuts)
    if printed_pins:
        for a in ring_pin_angles():
            x, y = polar(R_PINS, a)
            b = b.fuse(cyl(PIN_D, Z_PIN0 + PIN_LEN - FLOOR_T + 0.01, FLOOR_T - 0.01, x, y))
        b = one_solid(b.clean())
    return b


def support_ring(printed_pins):
    od, idia = 2 * (R_OUT + PIN_D / 2 + 2.0), 15.0
    body = ring(od, idia, RING_T, Z_RING)
    if printed_pins:
        for a in out_pin_angles():
            x, y = polar(R_OUT, a)
            body = body.fuse(cyl(PIN_D, PIN_LEN - 0.01, Z_OPIN0 + 0.01, x, y))
        return one_solid(body.clean())
    return cut_all(body, [cyl(PIN_HOLE, RING_T + 2, Z_RING - 1, *polar(R_OUT, a)) for a in out_pin_angles()])


def disc():
    face = cq.Face.makeFromWires(disc_wire())
    body = cq.Solid.extrudeLinear(face, V(0, 0, DISC_T))
    hole_d = PIN_D + 2 * ECC + 2 * HOLE_CLEAR
    cuts = [cyl(ECC_BORE, DISC_T + 2, -1)]
    cuts += [cyl(hole_d, DISC_T + 2, -1, *polar(R_OUT, a)) for a in out_pin_angles()]
    return cut_all(body, cuts)


def hub(upper):
    """Eccentric in two pieces on the D shaft (a 6800's 10 mm bore can't pass a collar, so each bearing slides
    onto its cam from above): lower = collar + cam 1 (+e along x); upper = collar + cam 2 (-e). Collars are
    concentric and 11 mm, so they bear on the inner rings only (band r 5.0..6.2 round each cam's centre)."""
    b1 = Z_D1 + (DISC_T - B6800_W) / 2           # 10.7
    b2 = Z_D2 + (DISC_T - B6800_W) / 2           # 17.2
    if upper:
        z0, zc, ex = b1 + B6800_W, b2, -ECC       # collar 15.7..17.2, cam 17.2..22.2
    else:
        z0, zc, ex = FLOOR_T + 0.3, b1, ECC      # collar 7.5..10.7, cam 10.7..15.7
    z1 = zc + B6800_W
    body = cyl(COLLAR_D, zc - z0, z0).fuse(cyl(CAM_D, B6800_W, zc, ex, 0))
    n, ro, ri, h, d, c = KEY_STAR
    if upper:                                 # the star key's socket in the upper collar's underside
        body = body.cut(star(ro, ri, n, d + 1, z0 - 1, c))
    else:                                     # the star key standing on the lower cam's top face
        body = body.fuse(star(ro, ri, n, h, z1))
        z1 += h
    body = body.cut(cyl(HUB_BORE, z1 - z0 + 2, z0 - 1))
    # D flat: material back in the bore beyond x = -(2.5 - 0.5), from past the latest flat start (in the upper one,
    # from above its socket: the lower one's star, with its own flat, fills the socket)
    zf = max(FLAT_LATE + 0.5, z0 + (d if upper else 0.0))
    flat = cyl(HUB_BORE, z1 - zf, zf).intersect(box(-HUB_BORE, -(G.SHAFT_D / 2 - G.SHAFT_FLAT_DEPTH), -5, 5, zf, z1))
    return one_solid(body.fuse(flat).clean())


def star(ro, ri, n, h, z, grow=0.0):
    """an n-point star prism about the shaft axis (points at ro, valleys at ri, one point on +x), grown outward by
    `grow` all round (the socket's clearance), h tall from z"""
    pts = [((ro if k % 2 == 0 else ri) * math.cos(math.pi * k / n), (ro if k % 2 == 0 else ri) * math.sin(math.pi * k / n))
           for k in range(2 * n)]
    w = cq.Workplane("XY", origin=(0, 0, z)).polyline(pts).close()
    if grow:
        w = w.offset2D(grow, "intersection")
    return w.extrude(h).val()


def output_flange():
    back = cyl(2 * 24.0, BACK_T, Z_TOP)
    journal = cyl(G.CAGE_SEAT, Z_BRG_TOP - 0.1 - Z_BRG, Z_BRG)
    body = back.fuse(journal)
    face = Z_BRG_TOP - 0.1
    # the motor shaft's tip: a D7 hole straight through into the lightening pocket (Mike 2026-11-06, "will this gearbox
    # handle the shaft lengths of all the motors we're using?"): it used to stop at z 25, so a 25 mm shaft (some 60 mm
    # NEMA 17s) would have rubbed the slowly turning flange; now any length fits. Short ones: the wrist's 20 mm is fine
    # (the star key drives the upper eccentric).
    cuts = [cyl(7.0, (Z_BRG + 1.0) + 0.5 - (Z_TOP - 1), Z_TOP - 1),      # motor shaft tip, through
            cyl(CENTRE_POCKET_D, face - (Z_BRG + 1.0) + 1, Z_BRG + 1.0)]  # lightening pocket from the face
    for a in out_pin_angles():
        cuts.append(cyl(PIN_HOLE, 4.5 + 1, Z_TOP - 1, *polar(R_OUT, a)))
    for a in OUT_SCREW_ANGLES:
        x, y = polar(OUT_SCREW_R, a)
        cuts.append(cyl(INSERT_HOLE, INSERT_DEPTH + 1, face - INSERT_DEPTH, x, y))
        cuts.append(GS.chamfer_cone(INSERT_HOLE, INSERT_LEAD, face, flip=True, x=x, y=y))   # lead-in at the face
        cuts.append(cyl(3.2, face - Z_OUT_HOLE_BOTTOM, Z_OUT_HOLE_BOTTOM, x, y))
    return cut_all(body, cuts)


def pin_ring():
    body = ring(G.BODY_OD, 49.0, Z_BRG - Z_TOP, Z_TOP)
    cuts = [cyl(PIN_CAP_HOLE, 0.7 + 1, Z_TOP - 1, *polar(R_PINS, a)) for a in ring_pin_angles()]
    cuts += [cyl(G.M3["clear"], Z_BRG - Z_TOP + 2, Z_TOP - 1, *polar(LID_SCREW_R, a)) for a in LID_SCREW_ANGLES]
    return cut_all(body, cuts)


def lid():
    body = cyl(G.BODY_OD, Z_LID_STEP - Z_BRG, Z_BRG).fuse(cyl(2 * LID_STEP_R, Z_LID_TOP - Z_BRG, Z_BRG))
    cuts = [cyl(G.CAGE_POCKET, Z_BRG_TOP - Z_BRG + 1, Z_BRG - 1), cyl(G.CAGE_DA_H, 5, Z_BRG_TOP - 0.01)]
    cuts += [cyl(G.M3["clear"], Z_LID_TOP - Z_BRG + 2, Z_BRG - 1, *polar(LID_SCREW_R, a)) for a in LID_SCREW_ANGLES]
    return cut_all(body, cuts)


def clamp_ring():
    return ring(48.0, 36.0, CLAMP_T, Z_BRG_TOP)


def spacer_outer():
    """between the two 6808s' outer races (SPREAD only)"""
    return ring(SPACER_OUT[0], SPACER_OUT[1], SPREAD, Z_BRG + G.CAGE_W)


def spacer_inner():
    """between the two 6808s' inner races, on the journal (SPREAD only)"""
    return ring(SPACER_IN[0], SPACER_IN[1], SPREAD, Z_BRG + G.CAGE_W)


# ---------------------------------------------------------------- bought parts and stand-ins
def dowel(x, y, z0):
    return cyl(PIN_D, PIN_LEN, z0, x, y)


def b6800():
    inner = ring(B6800_INNER_OD, B6800_ID, B6800_W)
    outer = ring(B6800_OD, B6800_OUTER_ID, B6800_W)
    seal = ring(B6800_OUTER_ID + 0.05, B6800_INNER_OD - 0.05, 0.3, 0.3).fuse(
        ring(B6800_OUTER_ID + 0.05, B6800_INNER_OD - 0.05, 0.3, B6800_W - 0.6))
    return one_solid(inner.fuse(outer).fuse(seal))


def motor_body():
    b = box(-21.15, 21.15, -21.15, 21.15, -48.0, 0.0).fuse(cyl(22.0, 2.0))
    for sx in (-1, 1):
        for sy in (-1, 1):
            b = b.cut(cyl(2.5, 4.5 + 0.01, -4.5, sx * 15.5, sy * 15.5))
    return one_solid(b)


def motor_shaft():
    r = G.SHAFT_D / 2
    s = cyl(G.SHAFT_D, G.SHAFT_LEN - 2.0, 2.0)
    return one_solid(s.cut(box(-r - 1, -(r - G.SHAFT_FLAT_DEPTH), -r - 1, r + 1, FLAT_START, G.SHAFT_LEN + 1)))


def insert():
    return ring(INSERT_HOLE, 2.95, INSERT_L)


def out_disc():
    b = cyl(G.BODY_OD, OUT_DISC_T, Z_DISC_OUT)
    return cut_all(b, [cyl(OUT_DISC_HOLE, OUT_DISC_T + 2, Z_DISC_OUT - 1, *polar(OUT_SCREW_R, a)) for a in OUT_SCREW_ANGLES])


def screw(length, z_head, x, y, thread_d=None, plain_len=None):
    return GS.cap_screw(G.M3, length, thread_d, plain_len).translate(V(x, y, z_head))


# ---------------------------------------------------------------- assembly at an input angle
def place(shape, centre=(0, 0), phi_deg=0.0):
    s = shape.rotate(V(0, 0, 0), V(0, 0, 1), phi_deg) if phi_deg else shape
    return s.translate(V(centre[0], centre[1], 0)) if centre != (0, 0) else s


def build(printed_pins, theta_deg, parts):
    th = math.radians(theta_deg)
    out = []                                       # (name, solid, colour, printed)
    blue, grey, steel, brass, black = (0.2, 0.45, 0.85), (0.6, 0.6, 0.62), (0.75, 0.75, 0.78), (0.8, 0.65, 0.2), (0.15, 0.15, 0.15)
    out.append(("housing", parts["housing_p" if printed_pins else "housing_s"], blue, True))
    out.append(("pin ring", parts["pin_ring"], blue, True))
    out.append(("lid", parts["lid"], blue, True))
    c1, p1 = disc_pose(th, 1)
    c2, p2 = disc_pose(th, 2)
    # the disc is built centred on the origin in its own frame: turn it, then move it to its centre
    out.append(("disc 1", place(parts["disc"], tuple(c1), math.degrees(p1)).translate(V(0, 0, Z_D1)), (0.9, 0.5, 0.1), True))
    out.append(("disc 2", place(parts["disc"], tuple(c2), math.degrees(p2)).translate(V(0, 0, Z_D2)), (0.95, 0.65, 0.2), True))
    out.append(("eccentric lower", parts["hub_lo"].rotate(V(0, 0, 0), V(0, 0, 1), theta_deg), (0.3, 0.75, 0.3), True))
    out.append(("eccentric upper", parts["hub_up"].rotate(V(0, 0, 0), V(0, 0, 1), theta_deg), (0.4, 0.85, 0.4), True))
    phi = math.degrees(-th / LOBES)
    out.append(("output flange", parts["flange"].rotate(V(0, 0, 0), V(0, 0, 1), phi), (0.2, 0.3, 0.6), True))
    out.append(("output support ring", parts["sring_p" if printed_pins else "sring_s"].rotate(V(0, 0, 0), V(0, 0, 1), phi),
                (0.2, 0.3, 0.6), True))
    out.append(("clamp ring", parts["clamp"].rotate(V(0, 0, 0), V(0, 0, 1), phi), (0.2, 0.3, 0.6), True))
    if SPREAD:
        out.append(("outer race spacer", parts["sp_out"], blue, True))
        out.append(("inner race spacer", parts["sp_in"].rotate(V(0, 0, 0), V(0, 0, 1), phi), (0.2, 0.3, 0.6), True))
    # bought - named with what they are and their sizes (Mike 2026-11-06: "When you apply names to the part components,
    # explain what their names, their sizes. Like the bolts, inserts")
    b68 = f"ball bearing 6800, {B6800_ID:g} bore x {B6800_OD:g} OD x {B6800_W:g} wide"
    out.append((f"6800 in disc 1: {b68}", parts["b6800"].translate(V(c1[0], c1[1], Z_D1 + 0.5)), steel, False))
    out.append((f"6800 in disc 2: {b68}", parts["b6800"].translate(V(c2[0], c2[1], Z_D2 + 0.5)), steel, False))
    for k in range(2):
        out.append((f"6808 {k + 1}: ball bearing 6808, 40 bore x 52 OD x {G.CAGE_W:g} wide",
                    parts["b6808"].translate(V(0, 0, Z_BRG + k * (G.CAGE_W + SPREAD))), steel, False))
    dow = f"dowel pin {PIN_D:g} x {PIN_LEN:g}, hardened steel ISO 8734 m6"
    if not printed_pins:
        for i, a in enumerate(ring_pin_angles()):
            out.append((f"ring pin {i + 1}: {dow}", dowel(*polar(R_PINS, a), Z_PIN0), steel, False))
        for i, a in enumerate(out_pin_angles()):
            out.append((f"output pin {i + 1}: {dow}", dowel(*polar(R_OUT, a + phi), Z_OPIN0), steel, False))
    out.append(("motor body: NEMA 17 stepper 17HS19-2004S1, 42.3 square x 48 long (stand-in)", parts["motor"], black, False))
    out.append((f"motor shaft: {G.SHAFT_D:g} mm D-flat shaft, {G.SHAFT_LEN:g} long",
                parts["shaft"].rotate(V(0, 0, 0), V(0, 0, 1), theta_deg), steel, False))
    head_z = FLOOR_T - 0.2 - G.M3["head_h"]
    k = 0
    for sx in (-1, 1):
        for sy in (-1, 1):
            k += 1
            out.append((f"motor screw {k}: M3 x {MOTOR_SCREW_L} socket head cap screw",
                        screw(MOTOR_SCREW_L, head_z, sx * 15.5, sy * 15.5, 2.5, head_z), black, False))
    L = lid_screw_len()
    for k, a in enumerate(LID_SCREW_ANGLES, start=1):
        x, y = polar(LID_SCREW_R, a)
        out.append((f"lid screw {k}: M3 x {L} socket head cap screw, partly threaded ({LID_THREAD:g} thread)",
                    screw(L, Z_LID_STEP, x, y, G.BODY_HOLE, L - LID_THREAD), black, False))
    face = Z_BRG_TOP - 0.1
    for k, a in enumerate(OUT_SCREW_ANGLES, start=1):
        x, y = polar(OUT_SCREW_R, a + phi)
        out.append((f"insert {k}: M3 x {INSERT_L:g} x {INSERT_D:g} brass heat-set insert ({INSERT_L:g} long, "
                    f"{INSERT_D:g} knurl)", insert().translate(V(x, y, face - INSERT_L)), brass, False))
        out.append((f"output screw {k}: M3 x 30 socket head cap screw", screw(30, Z_DISC_OUT + OUT_DISC_T, x, y),
                    black, False))
    out.append((f"your output disc: {G.BODY_OD:g} x {OUT_DISC_T:g} (stand-in)",
                parts["outdisc"].rotate(V(0, 0, 0), V(0, 0, 1), phi), grey, False))
    seen = {}
    named = []
    for nm, s, col, pr in out:                    # unique names: "motor screw M3x8 2" ...
        seen[nm] = seen.get(nm, 0) + 1
        total = sum(1 for o in out if o[0] == nm)
        named.append((f"{nm} {seen[nm]}" if total > 1 else nm, s, col, pr))
    return named


def overlaps(items, skip=()):
    """Pairwise common volume (bounding boxes first)."""
    bbs = [s.BoundingBox() for _, s, _, _ in items]
    res = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = bbs[i], bbs[j]
            if a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin:
                continue
            ni, nj = items[i][0], items[j][0]
            if (ni, nj) in skip or (nj, ni) in skip:
                continue
            v = items[i][1].intersect(items[j][1]).Volume()
            if v > 0.001:
                res.append((ni, nj, v))
    return res


def main():
    global CLEAR, HUB_BORE, CAM_D, ECC_BORE, PIN_HOLE, INSERT_HOLE
    ap = argparse.ArgumentParser()
    ap.add_argument("--insert-hole", type=float, default=INSERT_HOLE, help="printed hole for the M3 x 4 x 5 heat-set inserts")
    ap.add_argument("--clear", type=float, default=CLEAR)
    ap.add_argument("--hub-bore", type=float, default=HUB_BORE)
    ap.add_argument("--cam-d", type=float, default=CAM_D)
    ap.add_argument("--ecc-bore", type=float, default=ECC_BORE)
    ap.add_argument("--pin-hole", type=float, default=PIN_HOLE)
    ap.add_argument("--spread", type=float, default=SPREAD, help="mm between the two output 6808s (the arm: 14)")
    a = ap.parse_args()
    CLEAR, HUB_BORE, CAM_D, ECC_BORE, PIN_HOLE = a.clear, a.hub_bore, a.cam_d, a.ecc_bore, a.pin_hole
    INSERT_HOLE = a.insert_hole
    set_spread(a.spread)
    out_dir = OUT if not SPREAD else os.path.join(os.path.dirname(OUT), "cycloid_spread")
    os.makedirs(os.path.join(out_dir, "step"), exist_ok=True)
    if not SPREAD:
        os.makedirs(os.path.join(out_dir, "stl"), exist_ok=True)
    rep = []
    w = rep.append
    k1 = ECC * N_PINS / R_PINS
    w(f"cycloid.py - {LOBES}:1 cycloidal drive on the 17HS19 (z = 0 at the motor face)")
    w(f"Ring: {N_PINS} pins {PIN_D:g} mm on a {2 * R_PINS:g} circle; eccentricity {ECC:g} (K1 {k1:.2f}), lobe height "
      f"{2 * ECC:.2f}; disc profile pulled in {CLEAR:g}; {N_OUT} output pins on a {2 * R_OUT:g} circle in "
      f"{PIN_D + 2 * ECC + 2 * HOLE_CLEAR:.2f} holes.")
    w(f"Body {G.BODY_OD:g} OD; motor face to the output disc seat {Z_DISC_OUT:g} mm."
      + (f" Output 6808s SPREAD {SPREAD:g} apart (ball rows {G.CAGE_W + SPREAD:g} apart, was {G.CAGE_W:g}) by a printed "
         f"sleeve between the outer races ({SPACER_OUT[0]:g} / {SPACER_OUT[1]:g}) and one between the inner races "
         f"({SPACER_IN[0]:g} / {SPACER_IN[1]:g})." if SPREAD else "") + "\n")

    rmin = motion_check(rep)
    w(f"  disc material: bearing bore {ECC_BORE:g} -> output holes {R_OUT - (PIN_D + 2 * ECC) / 2 - HOLE_CLEAR - ECC_BORE / 2:.1f} mm, "
      f"output holes -> lobe roots {rmin - R_OUT - (PIN_D + 2 * ECC) / 2 - HOLE_CLEAR:.1f} mm")
    sw = R_PINS - WALL_R
    w(f"  ring pin grooves wrap {2 * (180 - math.degrees(math.acos(sw / (PIN_D / 2)))):.0f} deg of each pin "
      f"(opening {2 * math.sqrt((PIN_D / 2) ** 2 - sw ** 2):.2f} < {PIN_D:g}: pins can't fall inwards)")

    print("\n".join(rep))
    print("building parts ...", flush=True)
    G.FLAT_START = FLAT_START
    parts = dict(housing_s=housing(False), housing_p=housing(True), pin_ring=pin_ring(), lid=lid(), disc=disc(),
                 hub_lo=hub(False), hub_up=hub(True), flange=output_flange(), sring_s=support_ring(False), sring_p=support_ring(True),
                 clamp=clamp_ring(), b6800=b6800(), b6808=GS.bearing_6808(), motor=motor_body(), shaft=motor_shaft(),
                 outdisc=out_disc())
    if SPREAD:
        parts.update(sp_out=spacer_outer(), sp_in=spacer_inner())
    poly_area = 0.5 * abs(np.sum(profile_xy(200)[:, 0] * np.roll(profile_xy(200)[:, 1], -1)
                                 - np.roll(profile_xy(200)[:, 0], -1) * profile_xy(200)[:, 1]))
    raw = cq.Solid.extrudeLinear(cq.Face.makeFromWires(disc_wire()), V(0, 0, 1)).Volume()
    w(f"\nDisc spline vs the exact profile: area {raw:.2f} vs {poly_area:.2f} mm^2 ({(raw / poly_area - 1) * 100:+.3f} %)")

    # ---- checks: overlaps at three input angles, both pin variants
    w("\nPart overlaps (every pair, common volume > 0.001 mm^3), assembled at three input angles:")
    for printed in (False, True):
        for th in (0.0, 47.0, 200.0):
            items = build(printed, th, parts)
            ov = overlaps(items)
            lab = "printed pins" if printed else "steel pins"
            w(f"  {lab:12s} input {th:5.1f} deg: " + ("none" if not ov else
              "; ".join(f"{a} x {b} {v:.3f}" for a, b, v in ov)))

    # ---- loads
    f_ring = 4 * (T_DESIGN / 2) / (k1 * LOBES * R_PINS / 1000)
    f_out = 4 * (T_DESIGN / 2) / (N_OUT * R_OUT / 1000)
    # output pin: supported at the ring (mid 8.7) and the flange face (23.2); disc load at mid-planes
    za, zb = Z_RING + RING_T / 2, Z_TOP
    L = zb - za
    m = max(f_out * (Z_D1 + DISC_T / 2 - za) * (zb - Z_D1 - DISC_T / 2) / L,
            f_out * (Z_D2 + DISC_T / 2 - za) * (zb - Z_D2 - DISC_T / 2) / L)
    sig = 32 * m / (math.pi * PIN_D ** 3)
    t_in = T_DESIGN / (LOBES * ETA)
    f_ecc = t_in / 2 / (ECC / 1000)
    w(f"\nLoads at joint 3's design torque {T_DESIGN:g} N*m (gear_ratio.py: straight out, gripper + piece, x1.5); "
      f"stall {T_STALL:.1f} N*m (0.59 N*m x {LOBES} x {ETA:g}):")
    w(f"  most loaded ring pin per disc {f_ring:.0f} N ({f_ring / TEST_PIN_FORCE:.2f} x where the tested printed drive gave up); "
      f"grooves carry it along the whole pin")
    w(f"  most loaded output pin per disc {f_out:.0f} N; pin held at both ends, bending {sig:.0f} MPa "
      f"(stall {sig * T_STALL / T_DESIGN:.0f}): fine for hardened dowels; a PRINTED 3 mm pin is good for "
      f"roughly {T_DESIGN * 25 / sig:.1f} N*m (25 MPa) - testing only")
    w(f"  each 6800: about {f_ecc:.0f} N (input torque {t_in:.2f} N*m over e {ECC:g}); "
      f"{f_ecc * T_STALL / T_DESIGN:.0f} N at stall - check against the static rating on your bearings' listing")
    w(f"  input {t_in:.2f} N*m from a motor that gives 0.4 at 300 rpm; output {300 / LOBES * 6:.0f} deg/s at 300 rpm")
    # output inserts: if the clamp's friction carried none of it, the torque crosses to your disc through the 3 screws, each
    # insert pressing sideways on the nylon over its knurl's projected area (L x D)
    f_ins = T_DESIGN / (len(OUT_SCREW_ANGLES) * OUT_SCREW_R / 1000)
    p_ins = f_ins / (INSERT_L * INSERT_D)
    w(f"  each output insert (M3 x {INSERT_L:g} x {INSERT_D:g}, if the clamp's friction carried none): {f_ins:.0f} N sideways, "
      f"{p_ins:.1f} MPa on the nylon (stall {p_ins * T_STALL / T_DESIGN:.1f}; allowable 20.7, half moulded nylon's 41.4); "
      f"knurl wall {OUT_SCREW_R - INSERT_D / 2 - CENTRE_POCKET_D / 2:.2f} to the centre pocket, "
      f"{G.CAGE_SEAT / 2 - OUT_SCREW_R - INSERT_D / 2:.2f} to the journal OD")

    # ---- masses
    w("\nPrinted parts (solid nylon, g):")
    names = [("housing (steel pins)", "housing_s"), ("housing (printed pins)", "housing_p"), ("pin ring", "pin_ring"),
             ("lid", "lid"), ("disc (x2)", "disc"), ("eccentric lower", "hub_lo"), ("eccentric upper", "hub_up"), ("output flange", "flange"),
             ("output support ring (steel pins)", "sring_s"), ("output support ring (printed pins)", "sring_p"),
             ("clamp ring", "clamp")] + ([("outer race spacer", "sp_out"), ("inner race spacer", "sp_in")] if SPREAD else [])
    for lab, k in names:
        w(f"  {lab:36s} {parts[k].Volume() * NYLON:6.1f}")
    tot = sum(parts[k].Volume() for k in ("housing_s", "pin_ring", "lid", "hub_lo", "hub_up", "flange", "sring_s", "clamp")
              + (("sp_out", "sp_in") if SPREAD else ())) + 2 * parts["disc"].Volume()
    w(f"  set with steel pins: {tot * NYLON:.0f} g printed + 46 dowels {46 * math.pi / 4 * PIN_D ** 2 * PIN_LEN * 7.85e-3:.0f} g "
      f"+ 2 x 6808 + 2 x 6800 + screws (the 5:1 gearbox it replaces: {591 - 390:.0f} g without the motor)")

    # ---- hardware
    w("\nHardware per joint: 46 dowels 3 x 20 (38 ring + 8 output) | 2 x 6800 10x19x5 | 2 x 6808 40x52x7 |")
    w("  4 x M3 x 8 motor screws (from inside, 4.0 into the motor's 4.5 deep threads) |")
    fits = []
    for L in (20, 25, 30, 35, 40, 45, 50, 55, 60, 65):
        tip = Z_LID_STEP - L
        if tip > Z_TOP - 4:
            fits.append(f"x{L} too short")
        elif tip >= 0:
            fits.append(f"x{L} {Z_TOP - tip:.1f} into the housing")
        else:
            fits.append(f"x{L} sticks out {-tip:.1f} under the floor")
    L, s_end = lid_screw_len(), lid_shank_end()
    tip = Z_LID_STEP - L
    z_clear = min(Z_TOP, s_end - LID_SHANK_MARGIN)
    w(f"  6 x M3 x {L} socket head cap screws, PARTLY threaded ({LID_THREAD:g} thread, ISO 4762), heads on the lid step "
      f"at z {Z_LID_STEP:g}: the plain shank through lid + pin ring + a {G.M3['clear']:g} clearance in the housing's top "
      f"down to z {z_clear:.1f} (shank ends at {s_end:.1f}), the thread self-tapping the {G.BODY_HOLE:g} hole below: "
      f"{z_clear - max(tip, 0):.1f} mm of thread in the housing, tip {tip:.1f} above its underside |")
    w(f"     (fully threaded, for comparison: " + ", ".join(fits) + "; the tips come out beside the motor) |")
    face = Z_BRG_TOP - 0.1
    w(f"  3 x M3 output screws through your 24 mm disc + the 2 mm clamp ring: M3 x 30 bites "
      f"{30 - (OUT_DISC_T + CLAMP_T) - (Z_DISC_OUT - CLAMP_T - face):.1f} into the insert, M3 x 40 ends "
      f"{(Z_DISC_OUT + OUT_DISC_T - 40) - Z_OUT_HOLE_BOTTOM:.1f} above the hole bottom | 3 x M3 x {INSERT_L:g} x {INSERT_D:g} "
      f"heat-set inserts in {INSERT_HOLE:g} holes {INSERT_DEPTH:g} deep, {INSERT_LEAD:g} lead-in, flush with the flange face")
    w("  Discs: print solid (100 % infill, 4+ walls) - the tested printed drive cracked round its eccentric bearing first.")
    w("\nAssembly:")
    w("  1  Drill the printed pin holes to 3.0 (housing floor sockets, support ring, flange sockets) so the dowels press in.")
    w("  2  Motor to the housing: 4 x M3 x 8 from inside, heads into the counterbores.")
    w("  3  Steel pins: press the 38 ring pins down the wall grooves into the floor (tops stand 0.5 above the housing).")
    w("  4  Lower eccentric onto the motor shaft, D to D, down to 0.3 above the floor.")
    w("  5  Press the 8 output pins into the support ring (flush underneath) and drop it in over the hub.")
    w("  6  Press a 6800 into each disc, grease the lobes. Disc 1 down onto the lower cam and over the output pins;")
    w("     upper eccentric onto the shaft, its star socket down over the lower one's star (they lock 180 deg apart);")
    w("     disc 2 onto the upper cam, turned 180 deg")
    w("     (the same part; its lobes then fall between the pins).")
    w("  7  Both 6808s onto the flange journal; push them up into the lid from below (the lip stops the top race);"
      if not SPREAD else
      "  7  Onto the flange journal: a 6808, the inner race spacer, the other 6808; push the lot up into the lid from\n"
      "     below with the outer race spacer between the two outer races (the lip stops the top race);")
    w("     pin ring under the lid; lower the three together so the flange sockets take the output pin tops.")
    w("  8  6 lid screws through lid + pin ring into the housing.  9  Clamp ring on the upper inner race, your disc")
    w("     on top, 3 x M3 into the flange inserts: this clamps both inner races to the flange.")
    w("  Fits to tune after a test print: --clear (backlash), --hub-bore, --cam-d, --ecc-bore, --pin-hole, --insert-hole.")

    # ---- write
    print("writing STEP / STL ...", flush=True)
    frames = {  # part: (shape, z offset to put it on the bed, flip?)
        "housing_steel_pins": (parts["housing_s"], 0.0, False),
        "housing_printed_pins": (parts["housing_p"], 0.0, False),
        "pin_ring": (parts["pin_ring"], -Z_TOP, False),
        "lid": (parts["lid"], -Z_BRG, False),
        "disc": (parts["disc"], 0.0, False),
        "eccentric_lower": (parts["hub_lo"], -(FLOOR_T + 0.3), False),
        "eccentric_upper": (parts["hub_up"], -(Z_D1 + (DISC_T - B6800_W) / 2 + B6800_W), False),
        "output_flange": (parts["flange"], -Z_TOP, False),
        "output_support_ring_steel_pins": (parts["sring_s"], -Z_RING, False),
        "output_support_ring_printed_pins": (parts["sring_p"], -Z_RING, False),
        "clamp_ring": (parts["clamp"], -Z_BRG_TOP, False),
    }
    if SPREAD:
        frames["outer_race_spacer"] = (parts["sp_out"], -(Z_BRG + G.CAGE_W), False)
        frames["inner_race_spacer"] = (parts["sp_in"], -(Z_BRG + G.CAGE_W), False)
    for nm, (s, dz, _) in frames.items():
        if not SPREAD:                            # (the spread drive: STEP only - Mike exports his own STLs)
            flat = s.translate(V(0, 0, dz))
            cq.exporters.export(flat, os.path.join(out_dir, "stl", nm + ".stl"), tolerance=0.01, angularTolerance=0.1)
        cq.exporters.export(s, os.path.join(out_dir, "step", nm + ".step"))
    for printed in (False, True):
        assy = cq.Assembly(name="cycloid 37 to 1 " + ("printed pins" if printed else "steel pins"))
        for nm, s, col, _ in build(printed, 0.0, parts):
            assy.add(s, name=nm, color=cq.Color(*col))
        write_step(assy, os.path.join(out_dir, "step", f"assembly_{'printed' if printed else 'steel'}_pins.step"))
    txt = "\n".join(rep)
    open(os.path.join(out_dir, "report.txt"), "w").write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()

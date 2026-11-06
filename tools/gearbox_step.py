#!/usr/bin/env python3
"""
gearbox_step.py - the same gearbox as gearbox.py, rebuilt as true B-rep solids (CadQuery / OpenCascade)
and written out as STEP: one file per part, in the part's own design frame, plus gearbox_assembly.step
with every printed part, both 6808s, the six planet bearings, the screws and a stand-in NEMA 17 in the
assembled position (z = 0 is the motor face, as in gearbox.py's report).

  python gearbox_step.py [the same options as gearbox.py] [--out DIR]

Needs gearbox.py and planetary.py beside it and:  python -m pip install cadquery

Nothing is redesigned here: all sizes come from gearbox.py's constants and its Stack, and the involute
flanks use planetary.py's maths, so the STEP and the STL sets describe the same parts.  Cylinders,
cones and bores are analytic surfaces; each gear flank is a spline through the same involute points
the STL used; tip and root lands are arcs.  Print-only features that are real geometry (stepped bridge
roofs, the housing's stepped lip) are kept, so the solids match the STLs; the planet's 0.15 mm
first-layer foot relief is not (it is a print allowance).  The script compares every part's volume
with the mesh version and reports it.

Part frames (same as gearbox.py): motor_plate z=0 at the motor face; housing z=0 at its motor end;
spider z=0 at its underside; carrier z=0 at its pinch face; lid z=0 at its underside; arm_hub z=0 at
its underside on the boss face (its ring hangs below); sun and planet z=0 at their bottom faces.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cadquery as cq  # noqa: E402
from cadquery import Vector  # noqa: E402

import gearbox as G  # noqa: E402
import planetary as P  # noqa: E402


# ---------------------------------------------------------------- primitives (same names as gearbox.py)
def V(x, y, z=0.0):
    return Vector(x, y, z)


def cyl(d, h, z=0.0, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, h, V(x, y, z))


def ring(od, idia, h, z=0.0, x=0.0, y=0.0):
    return cyl(od, h, z, x, y).cut(cyl(idia, h + 2, z - 1, x, y))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cone(r_bottom, r_top, h, z=0.0, x=0.0, y=0.0):
    return cq.Solid.makeCone(r_bottom, r_top, h, V(x, y, z))


def chamfer_cone(d_small, depth, z, flip=False, x=0.0, y=0.0):
    r0, r1 = d_small / 2 + depth + 0.01, d_small / 2
    if flip:
        return cone(r1, r0, depth + 0.01, z - depth, x, y)
    return cone(r0, r1, depth + 0.01, z - 0.01, x, y)


def stepped_bridge(hole_d, roof_d, z, x, y):
    s = G.BRIDGE_LAYER
    slot = box(x - roof_d / 2 - 0.1, x + roof_d / 2 + 0.1, y - hole_d / 2, y + hole_d / 2, z - 0.001, z + s + 0.001)
    square = box(x - hole_d / 2, x + hole_d / 2, y - hole_d / 2, y + hole_d / 2, z + s - 0.001, z + 2 * s + 0.001)
    return slot.fuse(square)


def pinch_recess(z_face, x, y, up):
    if up:
        return ring(G.RECESS_D, G.BOSS_OD, G.RECESS + 1, z_face - G.RECESS, x, y)
    return ring(G.RECESS_D, G.BOSS_OD, G.RECESS + 1, z_face - 1, x, y)


def one_solid(shape):
    """OCC booleans hand back a Compound round the result; STEP written from that compound reads back
    as a shell, not a solid, so unwrap it (and complain if a part came out in pieces)."""
    if shape.ShapeType() == "Solid":
        return shape
    solids = shape.Solids()
    if len(solids) != 1:
        raise RuntimeError(f"expected one solid, got {len(solids)}")
    return solids[0]


def cut_all(body, cutters):
    for c in cutters:
        body = body.cut(c)
    return one_solid(body)


# ---------------------------------------------------------------- involute gears as wires
def _pol(r, a):
    return V(r * math.cos(a), r * math.sin(a))


def gear_wire(n, m, backlash, steps, rot=0.0, internal=False):
    """Closed wire of the gear boundary: spline flanks through planetary.py's involute points, arcs
    for the tip and root lands, straight stubs below the base circle.  Same tooth phase as
    planetary.external_profile / internal_profile."""
    rp = m * n / 2
    rb = rp * math.cos(P.PA)
    psi = math.pi / (2 * n) - backlash / (4 * rp)
    inva = math.tan(P.PA) - P.PA
    pitch = 2 * math.pi / n
    edges = []
    if not internal:
        ra = rp + P.ADDENDUM * m
        rf = rp - P.DEDENDUM * m
        while (psi + inva - P.inv(rb, ra)) * ra * 2 < 0.3 and ra > rp:
            ra -= 0.02 * m
        r0 = max(rb, rf)
        flank = [(r0 + (ra - r0) * i / steps) for i in range(steps + 1)]      # root/base -> tip
        ang = lambda r: psi + inva - P.inv(rb, r)                               # noqa: E731
        a_root = ang(r0)                            # = psi + inva at the base circle, less above it
        for k in range(n):
            c = rot + pitch * k
            if rf < rb - 1e-9:
                edges.append(cq.Edge.makeLine(_pol(rf, c - a_root), _pol(rb, c - a_root)))
            edges.append(cq.Edge.makeSpline([_pol(r, c - ang(r)) for r in flank]))
            a_tip = ang(ra)
            edges.append(cq.Edge.makeThreePointArc(_pol(ra, c - a_tip), _pol(ra, c), _pol(ra, c + a_tip)))
            edges.append(cq.Edge.makeSpline([_pol(r, c + ang(r)) for r in reversed(flank)]))
            if rf < rb - 1e-9:
                edges.append(cq.Edge.makeLine(_pol(rb, c + a_root), _pol(rf, c + a_root)))
            a0, a1 = c + a_root, c + pitch - a_root
            edges.append(cq.Edge.makeThreePointArc(_pol(rf, a0), _pol(rf, (a0 + a1) / 2), _pol(rf, a1)))
    else:
        ra = rp - P.ADDENDUM * m                    # tip, inner
        rf = rp + P.DEDENDUM * m                    # root, outer
        r0 = max(ra, rb)
        flank = [(r0 + (rf - r0) * i / steps) for i in range(steps + 1)]      # tip/base -> root
        ang = lambda r: psi - inva + P.inv(rb, r)                               # noqa: E731
        a_tip = ang(r0)                             # = psi - inva at the base circle, more above it
        a_root = ang(rf)
        for k in range(n):
            c = rot + pitch * k
            edges.append(cq.Edge.makeSpline([_pol(r, c - ang(r)) for r in reversed(flank)]))   # root -> tip, minus side
            if ra < rb - 1e-9:
                edges.append(cq.Edge.makeLine(_pol(rb, c - a_tip), _pol(ra, c - a_tip)))
            edges.append(cq.Edge.makeThreePointArc(_pol(ra, c - a_tip), _pol(ra, c), _pol(ra, c + a_tip)))
            if ra < rb - 1e-9:
                edges.append(cq.Edge.makeLine(_pol(ra, c + a_tip), _pol(rb, c + a_tip)))
            edges.append(cq.Edge.makeSpline([_pol(r, c + ang(r)) for r in flank]))              # tip -> root, plus side
            a0, a1 = c + a_root, c + pitch - a_root
            edges.append(cq.Edge.makeThreePointArc(_pol(rf, a0), _pol(rf, (a0 + a1) / 2), _pol(rf, a1)))
    return cq.Wire.assembleEdges(edges)


def prism(wire, h, z=0.0, holes=()):
    face = cq.Face.makeFromWires(wire, list(holes))
    return cq.Solid.extrudeLinear(face, V(0, 0, h)).translate(V(0, 0, z))


# ---------------------------------------------------------------- printed parts
def motor_plate(d):
    body = cyl(G.BODY_OD, G.PLATE_T)
    cuts = [cyl(P.NEMA17_PILOT_D + 0.2, G.PLATE_T + 2, -1),
            chamfer_cone(P.NEMA17_PILOT_D + 0.2, 0.4, 0.0)]
    cb_top = G.PLATE_T - G.MOTOR_HEAD_SINK
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * P.NEMA17_BOLT_HALF, sy * P.NEMA17_BOLT_HALF
            cuts.append(cyl(G.MOTOR_HOLE, G.PLATE_T + 2, -1, x, y))
            cuts.append(cyl(G.COUNTERBORE_D, G.PLATE_T, cb_top - G.M3["head_h"], x, y))
    cuts.append(ring(G.RELIEF_OD, G.CAGE_D - 1.0, G.INNER_RELIEF + 1, G.PLATE_T - G.INNER_RELIEF))
    cb = G.M3["head_h"] + 0.2
    for x, y in G.polar(G.SCREW_R, G.SCREW_ANGLES):
        cuts.append(cyl(G.M3["cb_d"], cb + 1, -1, x, y))
        cuts.append(stepped_bridge(G.M3["clear"], G.M3["cb_d"], cb, x, y))
        cuts.append(cyl(G.M3["clear"], G.PLATE_T, cb + 2 * G.BRIDGE_LAYER - 0.001, x, y))
    for x, y in G.polar(G.SCREW_R, G.BASE_ANGLES):
        cuts.append(cyl(G.M3["tap"], G.PLATE_T + 2, -1, x, y))
    return cut_all(body, cuts)


def housing(d, backlash, steps, z):
    h = z.housing1 - z.housing0
    ring_rot = 0.0 if d['planet'] % 2 == 0 else math.pi / d['ring']
    teeth = prism(gear_wire(d['ring'], d['module'], backlash, steps, ring_rot, internal=True), h + 2, -1)
    body = cyl(G.BODY_OD, h).fuse(ring(G.FLANGE_OD, G.BODY_OD - 0.5, G.FLANGE_T)).cut(teeth)
    cuts = [cyl(G.CAGE_POCKET, z.lip1 - z.housing0 + 1, -1)]
    for x, y in G.polar(G.FLANGE_R, G.FLANGE_ANGLES):
        cuts.append(cyl(G.M3["clear"], G.FLANGE_T + 2, -1, x, y))
    for i in range(G.lip_steps()):
        dia = max(G.CAGE_POCKET - 2 * G.LIP_STEP * (i + 1), G.CAGE_DA_H)
        cuts.append(cyl(dia, G.BRIDGE_LAYER + 0.002, z.lip1 - z.housing0 + i * G.BRIDGE_LAYER - 0.001))
    cuts.append(cyl(G.CAGE_DA_H, z.teeth0 - z.housing0 + 1, -1))
    cuts.append(cone(G.CAGE_DA_H / 2, G.CAGE_DA_H / 2 - G.TOOTH_LEAD_H, G.TOOTH_LEAD_H + 0.001, z.teeth0 - z.housing0 - 0.001))
    cuts.append(cyl(G.CAGE_DA_H, h - (z.teeth1 - z.housing0) + 1, z.teeth1 - z.housing0))
    cuts.append(cyl(G.CAGE_POCKET, G.UPPER_IN + 1, h - G.UPPER_IN))
    for x, y in G.polar(G.SCREW_R, G.SCREW_ANGLES):
        cuts.append(cyl(G.BODY_HOLE, h + 2, -1, x, y))
    return cut_all(body, cuts)


def spider(d):
    hole = 2 * d['ra_sun'] + 2.0
    t = G.spider_t()
    seat = G.spider_seat_h()
    cb = G.AXLE["head_h"] + G.CB_RECESS
    body = cyl(G.CAGE_SEAT, seat).fuse(cyl(G.CAGE_DA, t - seat, seat))
    cuts = [cyl(hole, t + 2, -1)]
    for j in range(d['planets']):
        x, y = G.pin_xy(d, j)
        cuts.append(cyl(G.AXLE["cb_d"], cb + 1, -1, x, y))
        cuts.append(stepped_bridge(G.AXLE["clear"], G.AXLE["cb_d"], cb, x, y))
        cuts.append(cyl(G.AXLE["clear"], t - G.TUBE_POCKET - cb + 0.002, cb + 2 * G.BRIDGE_LAYER - 0.001, x, y))
        cuts.append(cyl(G.TUBE_HOLE, G.TUBE_POCKET + 1, t - G.TUBE_POCKET, x, y))
        cuts.append(pinch_recess(t, x, y, up=True))
    r = G.CAGE_SEAT / 2                                                  # lead-in on the seat's bed edge
    cuts.append(cyl(G.CAGE_SEAT + 4, 1.0 + G.SEAT_LEAD, -1.0).cut(cone(r - G.SEAT_LEAD - 1.0, r, 1.0 + G.SEAT_LEAD, -1.0)))
    return cut_all(body, cuts)


def carrier(d, z):
    sh = z.c_shoulder - z.carrier0
    seat1 = sh + G.CAGE_W
    top = z.boss1 - z.carrier0
    body = cyl(G.CAGE_DA, sh).fuse(cyl(G.CAGE_SEAT, G.CAGE_W, sh)).fuse(cyl(G.BOSS_D, top - seat1, seat1))
    cuts = [cyl(G.CENTRE_HOLE, top + 3, -2)]
    for j in range(d['planets']):
        x, y = G.pin_xy(d, j)
        cuts.append(pinch_recess(0.0, x, y, up=False))
        cuts.append(cyl(G.TUBE_HOLE, G.TUBE_POCKET + 1, -1, x, y))
        cuts.append(cyl(G.AXLE["tap"], top + 2, -1, x, y))
    for x, y in G.link_xy():
        cuts.append(cyl(G.LINK_HOLE, G.LINK_DEPTH + 1, top - G.LINK_DEPTH, x, y))
        cuts.append(chamfer_cone(G.LINK_HOLE, 0.4, top, flip=True, x=x, y=y))
    return cut_all(body, cuts)


def lid(d):
    body = cyl(G.BODY_OD, G.LID_T)
    cuts = [cyl(G.CAGE_POCKET + G.LID_POCKET_EXTRA, G.CAGE_W - G.UPPER_IN + G.CAGE_PLAY + 1, -1),
            cyl(G.CAGE_DA_H, G.LID_T + 2, -1)]
    for x, y in G.polar(G.SCREW_R, G.SCREW_ANGLES):
        cuts.append(cyl(G.M3["clear"], G.LID_T + 2, -1, x, y))
    (mx, my), = G.polar(G.HOME_R, (G.HOME_ANGLE,))
    cuts.append(cyl(G.HOME_MAGNET_D + G.MAGNET_CLEAR, G.magnet_pocket_depth() + 1, G.LID_T - G.magnet_pocket_depth(), mx, my))
    return cut_all(body, cuts)


def arm_hub(z):
    body = cyl(G.BODY_OD, G.HUB_T).fuse(ring(G.CAGE_DA, G.HUB_BORE, z.retainer_h + 0.001, -z.retainer_h))
    cuts = [cyl(G.HUB_RELIEF_ID, G.HUB_RELIEF_H + 1, -z.retainer_h - 1)]
    seat = G.HUB_T - G.M3["head_h"] - 0.5
    for x, y in G.link_xy():
        cuts.append(cyl(G.M3["clear"], G.HUB_T + 2, -1, x, y))
        cuts.append(cyl(G.M3["cb_d"], G.HUB_T, seat, x, y))
        cuts.append(stepped_bridge(G.M3["clear"], G.M3["cb_d"], 0.0, x, y).mirror("XY").translate(V(0, 0, seat)))
    for x in (-G.ARM_BOLT_X, G.ARM_BOLT_X):
        cuts.append(cyl(G.M5["clear"], G.HUB_T + 2, -1, x, 0.0))
        cuts.append(cyl(G.M5["cb_d"], G.M5["head_h"] + 0.5 + 1, -1, x, 0.0))
    tw, rd, dp = G.SENSOR_POCKET
    (sx, sy), = G.polar(G.HOME_R, (G.SENSOR_ANGLE,))
    pocket = box(-tw / 2, tw / 2, -rd / 2, rd / 2, -1, dp)
    pocket = pocket.fuse(box(-G.SENSOR_CHANNEL[0] / 2, G.SENSOR_CHANNEL[0] / 2, rd / 2 - 0.5, G.BODY_OD, -1, G.SENSOR_CHANNEL[1]))
    cuts.append(pocket.rotate(V(0, 0, 0), V(0, 0, 1), G.SENSOR_ANGLE - 90.0).translate(V(sx, sy, 0)))
    return cut_all(body, cuts)


def sensor_board(z):
    tw, rd, dp = G.SENSOR_POCKET
    (sx, sy), = G.polar(G.HOME_R, (G.SENSOR_ANGLE,))
    b = box(-5.0, 5.0, -4.0, 4.0, G.SENSOR_RECESS + 1.1, G.SENSOR_RECESS + 2.7)
    b = b.fuse(box(-1.5, 1.5, -1.5, 1.5, G.SENSOR_RECESS, G.SENSOR_RECESS + 1.1 + 0.001))
    return one_solid(b.rotate(V(0, 0, 0), V(0, 0, 1), G.SENSOR_ANGLE - 90.0).translate(V(sx, sy, z.boss1)))


def home_magnet(z):
    (mx, my), = G.polar(G.HOME_R, (G.HOME_ANGLE,))
    return cyl(G.HOME_MAGNET_D, G.HOME_MAGNET_T, z.lid1 - G.HOME_MAGNET_SINK - G.HOME_MAGNET_T, mx, my)


def planet_spacer():
    return one_solid(ring(G.SPACER_OD, G.SPACER_ID, G.spacer_t()))


def sun(d, backlash, steps, z):
    length = z.sun1 - z.sun0
    bore = G.SUN_BORE or d['sun_bore']
    body = prism(gear_wire(d['sun'], d['module'], backlash, steps), length)
    bore_cut = cyl(bore, length + 2, -1)
    if not G.ROUND_SHAFT:
        rr = bore / 2
        flat_x = -(G.SHAFT_D / 2 - G.SHAFT_FLAT_DEPTH) - (bore - G.SHAFT_D) / 2
        bore_cut = bore_cut.cut(box(-rr - 1, flat_x, -rr - 1, rr + 1, -2, length + 2))    # D: the flat stays
    return cut_all(body, [bore_cut, chamfer_cone(bore, G.SUN_CHAMFER, 0.0),
                          chamfer_cone(bore, G.SUN_CHAMFER, length, flip=True)])


def planet(d, backlash, steps, bore):
    h = G.planet_t()
    cb = G.cb_depth()
    # no first-layer foot relief here: that 0.15 mm is a print allowance, not part geometry
    body = prism(gear_wire(d['planet'], d['module'], backlash, steps), h)
    cuts = [cyl(G.LEDGE_BORE, h + 2, -1),
            cyl(bore, cb + 1, -1),
            cyl(bore, cb + 1, h - cb),
            chamfer_cone(bore, G.ENTRY_CHAMFER, 0.0),
            chamfer_cone(bore, G.ENTRY_CHAMFER, h, flip=True)]
    return cut_all(body, cuts)


# ---------------------------------------------------------------- bought parts, as reference solids
def bearing_6808():
    """Inner ring, outer ring and two seals as one solid: the real envelope for CAD, not a model of the balls."""
    inner = ring(G.CAGE_INNER_OD, G.CAGE_D, G.CAGE_W)
    outer = ring(G.CAGE_OD, G.CAGE_OUTER_ID, G.CAGE_W)
    seals = None
    for zs in (G.CAGE_SEAL_RECESS, G.CAGE_W - G.CAGE_SEAL_RECESS - 0.4):
        s = ring(G.CAGE_OUTER_ID + 0.05, G.CAGE_INNER_OD - 0.05, 0.4, zs)
        seals = s if seals is None else seals.fuse(s)
    return one_solid(inner.fuse(outer).fuse(seals))


def axle_tube():
    """The planet axle tube at its nominal size, 6 OD, 4.2 bore (the check model in gearbox.py uses the worst small one)."""
    return one_solid(ring(G.TUBE_OD, G.TUBE_ID_MIN, G.tube_len()))


def bearing_686():
    """(inner ring, outer ring with its shields): two solids, because in the upper planet bearing the
    inner ring sits 0.1 higher than the outer (the spacer is 0.1 longer than the gear's ledge)."""
    inner = ring(G.INNER_LAND, G.BEARING_ID, G.BEARING_W)
    outer = ring(G.BEARING_OD, G.OUTER_LAND, G.BEARING_W)
    for zs in (G.SHIELD_RECESS, G.BEARING_W - G.SHIELD_RECESS - 0.25):
        outer = outer.fuse(ring(G.OUTER_LAND + 0.05, G.SHIELD_ID, 0.25, zs))
    return one_solid(inner), one_solid(outer)


def cap_screw(spec, length, thread_d=None, plain_len=None):
    """Socket head cap screw standing on z = 0 at the under-head plane, head above, shank below.
    thread_d: shank diameter (the printed hole's, where the screw cuts its own thread); plain_len:
    the first part of the shank at the full major diameter (through clearance holes and bearings)."""
    head = cyl(spec["head_d"], spec["head_h"], 0.0)
    if plain_len:
        shank = cyl(spec["major"], plain_len, -plain_len).fuse(cyl(thread_d, length - plain_len, -length))
    else:
        shank = cyl(thread_d or spec["major"], length, -length)
    hexs = cq.Workplane("XY").polygon(6, spec["head_d"] * 0.55).extrude(spec["head_h"] * 0.6) \
        .translate((0, 0, spec["head_h"] * 0.4)).val()
    return one_solid(head.fuse(shank).cut(hexs))


def nema17():
    body = box(-P.NEMA17_FACE / 2, P.NEMA17_FACE / 2, -P.NEMA17_FACE / 2, P.NEMA17_FACE / 2, -40.0, 0.0)
    pilot = cyl(P.NEMA17_PILOT_D, P.NEMA17_PILOT_H)
    r = G.SHAFT_D / 2
    fs = max(G.FLAT_START, P.NEMA17_PILOT_H + 0.01)
    shaft = cyl(G.SHAFT_D, G.SHAFT_LEN)
    if not G.ROUND_SHAFT:
        shaft = shaft.cut(box(-r - 1, -(r - G.SHAFT_FLAT_DEPTH), -r - 1, r + 1, fs, G.SHAFT_LEN + 1))
    return one_solid(body.fuse(pilot).fuse(shaft))


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sun", type=int, default=12)
    ap.add_argument("--planet", type=int, default=18)
    ap.add_argument("--module", type=float, default=1.0)
    ap.add_argument("--planets", type=int, default=3)
    ap.add_argument("--backlash", type=float, default=0.10)
    ap.add_argument("--steps", type=int, default=14)
    ap.add_argument("--bearing-bore", type=float, default=G.BEARING_BORE)
    ap.add_argument("--cage-seat", type=float, default=G.CAGE_SEAT)
    ap.add_argument("--cage-pocket", type=float, default=G.CAGE_POCKET)
    ap.add_argument("--sun-bore", type=float, default=0.0)
    ap.add_argument("--flat-start", type=float, default=G.FLAT_START)
    ap.add_argument("--screw-length", type=int, default=0)
    ap.add_argument("--shaft-length", type=float, default=G.SHAFT_LEN)
    ap.add_argument("--round-shaft", action="store_true")
    ap.add_argument("--out", default="gearbox_step")
    a = ap.parse_args()
    G.N_PLANETS, G.ROUND_SHAFT, G.FLAT_START = a.planets, a.round_shaft, a.flat_start
    G.BEARING_BORE, G.SHAFT_LEN = a.bearing_bore, a.shaft_length
    G.CAGE_SEAT, G.CAGE_POCKET = a.cage_seat, a.cage_pocket
    G.SUN_BORE = a.sun_bore or None

    d = P.design(a.module, a.planets, a.sun, a.planet, G.BEARING_ID, 0.0)
    if not d['ok']:
        sys.exit("that tooth combination fails the planetary design rules")
    z = G.Stack(a.screw_length or G.auto_screw_length())
    os.makedirs(a.out, exist_ok=True)

    print("Building B-rep parts")
    parts = {
        "motor_plate": motor_plate(d),
        "housing": housing(d, a.backlash, a.steps, z),
        "spider": spider(d),
        "carrier": carrier(d, z),
        "lid": lid(d),
        "arm_hub": arm_hub(z),
        "planet_spacer": planet_spacer(),
        "sun": sun(d, a.backlash, a.steps, z),
        "planet": planet(d, a.backlash, a.steps, G.BEARING_BORE),
    }

    # ---- the same parts from gearbox.py's mesh modeller, for the volume comparison
    _, _, ring_poly = P.assembly(d, a.backlash, a.steps)
    mesh = {
        "motor_plate": G.motor_plate(d),
        "housing": G.housing(d, ring_poly, z),
        "spider": G.spider(d),
        "carrier": G.carrier(d, z),
        "lid": G.lid(d),
        "arm_hub": G.arm_hub(z),
        "planet_spacer": G.planet_spacer(),
        "sun": G.sun(d, a.backlash, a.steps, z),
    }
    foot = G.FOOT_RELIEF
    G.FOOT_RELIEF = 0.0                          # the STEP planet has no first-layer foot relief
    mesh["planet"] = G.planet(d, a.backlash, a.steps, G.BEARING_BORE)
    G.FOOT_RELIEF = foot
    print("Volume check, STEP solid against the STL mesh (128-segment circles make the mesh ~0.1% smaller)")
    worst = 0.0
    for name, solid in parts.items():
        ok = solid.isValid()
        vs, vm = solid.Volume(), mesh[name].volume()
        dev = (vs - vm) / vm * 100
        worst = max(worst, abs(dev))
        bb = solid.BoundingBox()
        print(f"  {name:14s} valid={ok}  STEP {vs:9.1f}  STL {vm:9.1f} mm^3  {dev:+.2f}%  "
              f"bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")
        if not ok:
            sys.exit(f"{name}: invalid solid")
    if worst > 0.6:
        sys.exit(f"a STEP part differs from its STL by {worst:.2f}% in volume: investigate before using it")

    # ---- per-part STEP files, design frame
    for name, solid in parts.items():
        path = os.path.join(a.out, f"{name}.step")
        cq.exporters.export(cq.Workplane("XY").add(solid), path)
    b686_inner, b686_outer = bearing_686()
    tube = axle_tube()
    cq.exporters.export(cq.Workplane("XY").add(bearing_6808()), os.path.join(a.out, "bearing_6808_2RS.step"))
    cq.exporters.export(cq.Workplane("XY").add(b686_inner).add(b686_outer), os.path.join(a.out, "bearing_686ZZ.step"))
    cq.exporters.export(cq.Workplane("XY").add(tube), os.path.join(a.out, "axle_tube.step"))

    # ---- assembly
    colours = {"motor_plate": (0.6, 0.6, 0.6), "housing": (0.25, 0.5, 0.8), "spider": (0.2, 0.6, 0.25),
               "carrier": (0.9, 0.6, 0.15), "lid": (0.55, 0.3, 0.65), "sun": (0.8, 0.2, 0.2),
               "planet": (0.5, 0.75, 0.4), "arm_hub": (0.9, 0.4, 0.7), "planet_spacer": (0.9, 0.4, 0.7),
               "bearing": (0.35, 0.35, 0.35), "screw": (0.7, 0.65, 0.2), "motor": (0.8, 0.8, 0.8),
               "sensor": (0.1, 0.5, 0.1), "magnet": (0.8, 0.8, 0.2)}

    def col(k):
        return cq.Color(*colours[k], 1.0)

    assy = cq.Assembly(name="planetary_gearbox")

    def add(name, solid, kind, dz=0.0, dx=0.0, dy=0.0, rot_deg=0.0):
        loc = cq.Location(V(dx, dy, dz), V(0, 0, 1), rot_deg)
        assy.add(cq.Workplane("XY").add(solid), name=name, loc=loc, color=col(kind))

    add("motor_plate", parts["motor_plate"], "motor_plate")
    add("housing", parts["housing"], "housing", z.housing0)
    add("spider", parts["spider"], "spider", z.spider0)
    add("carrier", parts["carrier"], "carrier", z.carrier0)
    add("lid", parts["lid"], "lid", z.lid0)
    add("arm_hub", parts["arm_hub"], "arm_hub", z.boss1)
    add("hall_sensor_breakout", sensor_board(z), "sensor")
    add("homing_magnet_6x1.5", home_magnet(z), "magnet")
    std_len_hub = G.hub_screw_len()
    hub_s = cap_screw(G.M3, std_len_hub, G.LINK_HOLE, G.hub_plain_len())
    for i, (x, y) in enumerate(G.link_xy()):
        add(f"hub_screw_M3x{std_len_hub}_{i + 1}", hub_s, "screw", z.boss1 + G.HUB_T - G.M3["head_h"] - 0.5, x, y)
    add("sun", parts["sun"], "sun", z.sun0)
    b68 = bearing_6808()
    add("bearing_6808_lower", b68, "bearing", z.b1)
    add("bearing_6808_upper", b68, "bearing", z.b2)
    spacer = parts["planet_spacer"]
    flip = cq.Location(V(0, 0, 0), V(1, 0, 0), 180)          # head below, shank up
    axle = cap_screw(G.AXLE, z.screw_len, G.AXLE["tap"], z.carrier0 + G.TUBE_POCKET - z.underhead).moved(flip)
    for j in range(d['planets']):
        phi = 2 * math.pi * j / d['planets']
        rho = phi + math.pi - math.pi / d['planet'] + d['sun'] * phi / d['planet']
        x, y = G.pin_xy(d, j)
        add(f"planet_{j + 1}", parts["planet"], "planet", z.gear0, x, y, math.degrees(rho))
        add(f"bearing_686_{j + 1}a_inner", b686_inner, "bearing", z.planet0, x, y)
        add(f"bearing_686_{j + 1}a_outer", b686_outer, "bearing", z.planet0, x, y)
        add(f"bearing_686_{j + 1}b_inner", b686_inner, "bearing", z.spacer0 + G.spacer_t(), x, y)
        add(f"bearing_686_{j + 1}b_outer", b686_outer, "bearing", z.ledge1, x, y)
        add(f"axle_tube_{j + 1}", tube, "screw", z.spider1 - G.TUBE_POCKET + G.TUBE_END_GAP, x, y)
        add(f"planet_spacer_{j + 1}", spacer, "planet_spacer", z.spacer0, x, y)
        add(f"planet_screw_M4x{z.screw_len}_{j + 1}", axle, "screw", z.underhead, x, y)   # under-head plane
    lid_s = cap_screw(G.M3, z.lid_screw, G.BODY_HOLE)
    plate_s = cap_screw(G.M3, z.plate_screw, G.BODY_HOLE).moved(flip)
    cb = G.M3["head_h"] + 0.2
    for i, (x, y) in enumerate(G.polar(G.SCREW_R, G.SCREW_ANGLES)):
        add(f"lid_screw_M3x{z.lid_screw}_{i + 1}", lid_s, "screw", z.lid1, x, y)
        add(f"plate_screw_M3x{z.plate_screw}_{i + 1}", plate_s, "screw", cb, x, y)   # head in the counterbore below
    add("nema17_stand_in", nema17(), "motor")
    path = os.path.join(a.out, "gearbox_assembly.step")
    assy.export(path)
    print(f"wrote {len(parts) + 2} part files and gearbox_assembly.step to {a.out}")
    print(f"  the arm hub's underside is at z {z.boss1:g}; the arm extrusion sits on its top at z {z.boss1 + G.HUB_T:g}")
    print(f"  stack z (motor face = 0): spider {z.spider0:g}, planets {z.gear0:g}..{z.gear1:g}, carrier {z.carrier0:g}, "
          f"lid top {z.lid1:g}, boss face {z.boss1:g}")


if __name__ == "__main__":
    main()

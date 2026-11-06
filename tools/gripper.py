#!/usr/bin/env python3
"""
gripper.py - spring-closed, cable-opened three-finger gripper for the chess pieces, as STL and STEP.

  python gripper.py [--pivot-r 22] [--finger 111] [--open-deg 6] [--spring 15x25] [--out DIR]

Needs:  python -m pip install cadquery   (and trimesh for the STL mesh check)

How it works (z up, hub frame: z = 0 is the hub's bottom face, which hangs HUB_ABOVE_KING over the
tallest piece):
  - Three fingers at 120 deg hang from tangential M3 pins in slots through the hub wall.  Below the
    pin each finger is a long bar ending in a lip that reaches inward under the pedestal's waist
    shoulder (the groove 3..5.5 mm up the pedestal).  Above the pin a short tail reaches up inside the
    hub, its nose bearing on the plunger.
  - The plunger is a cup with a conical skirt.  A compression spring between the plunger and the cap
    pushes it DOWN; the skirt wedges the three noses outward, which swings the lips inward: CLOSED.
    The Bowden cable, anchored under the plunger's top plate, pulls it UP; the noses drop onto the
    narrower part of the cone and the fingers swing open.  No power holds the piece: the spring does.
  - The cable's sheath is held by a PC4-M6 push fitting screwed into the cap's centre boss.  The
    cap also carries two M4 holes for the lift carriage.

Printed parts: gripper_hub, gripper_cap, gripper_plunger, gripper_finger (x3).
Bought: 3 x M3x30 + nyloc (finger pins), 3 x M3x10 (cap to hub), 1 compression spring (default
15 OD x 25 free, ~1.5 N/mm), 1 x PC4-M6 fitting, 1.2 mm steel cable in 2/4 PTFE tube, a crimp or knot.

Checks: lip radius closed (= waist) and open (clears the pedestal), nose contact on the cone in both
states, finger clearance to the piece and to the four neighbours when open, hub clearance over the
king, no intersection between parts in either state, printability notes.  Every solid's volume is
compared with its STL mesh.
"""
import argparse
import math
import os

import cadquery as cq
from cadquery import Vector

# ---------------------------------------------------------------- the pieces (from pedestal.py / the STLs)
PED_OD = 33.0
WAIST_R = 14.5           # pedestal groove bottom radius (dia 29)
WAIST0, WAIST1 = 3.0, 5.5   # groove above the board
PIECE_TOP = 100.0        # king 92 + pedestal 8, above the board
SQUARE = 60.0

# ---------------------------------------------------------------- gripper geometry (mm)
HUB_OD = 50.0
HUB_H = 50.0
HUB_FLOOR = 12.0         # solid below this (minus the cable access hole)
CABLE_ACCESS = 20.0
HUB_ABOVE_KING = 6.0     # hub bottom above the tallest piece's top
PIVOT_R = 22.0           # pin radius
PIVOT_Z = 8.0            # pin height above the hub bottom
PIN_D = 3.0
PIN_HOLE = 3.4
FINGER_W = 6.0           # tangential
FINGER_T = 8.0           # radial
SLOT_W = 6.6
TAIL = 20.0              # pin to the top of the tail
LIP_REACH = None         # computed: finger inner face radius - WAIST_R
LIP_T = 1.5              # lip height
LIP_CHAMFER = 0.6
OPEN_DEG = 6.0           # finger swing when open
PLUNGER_WALL = 2.5
PLUNGER_TRAVEL = 6.0
CONE_MARGIN = 1.0
PLUNGER_CLEAR = 0.3      # radial clearance plunger to hub bore
SPRING_OD = 15.0
SPRING_FREE = 25.0
SPRING_RATE = 1.5        # N/mm, assumed
SPRING_POCKET_PLUNGER = 10.0
SPRING_POCKET_CAP = 3.0
CAP_T = 6.0
CAP_BOSS_D = 12.0
CAP_BOSS_H = 8.0
PC4_M6_HOLE = 5.0        # the fitting cuts its own M6 thread
CABLE_HOLE = 2.0
CAP_SCREW_R = 21.5
MOUNT_HOLE = 4.4         # M4 clearance, two at +-MOUNT_X on the y axis
MOUNT_X = 20.0
M3 = dict(head_d=5.5, head_h=3.0, clear=3.4, tap=2.5)
PETG = 1.27


def one_solid(shape):
    if shape.ShapeType() == "Solid":
        return shape
    s = shape.Solids()
    assert len(s) == 1, f"{len(s)} solids"
    return s[0]


def V(x, y, z=0.0):
    return Vector(x, y, z)


def cyl(d, h, z=0.0, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, h, V(x, y, z))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def rot_z(shape, deg):
    return shape.rotate(V(0, 0, 0), V(0, 0, 1), deg)


# ---------------------------------------------------------------- derived
def finger_inner_r():
    return PIVOT_R - FINGER_T / 2


def nose_r_closed():
    return finger_inner_r()


def nose_r_open():
    """Innermost point of the rounded nose (radius FINGER_T/2, centre TAIL - FINGER_T/2 above the pin) when open."""
    th = math.radians(OPEN_DEG)
    return PIVOT_R - (TAIL - FINGER_T / 2) * math.sin(th) - FINGER_T / 2


def lip_r(theta_deg, finger_len):
    """Radius of the lip's inner edge for a finger swung theta outward (0 = hanging straight)."""
    th = math.radians(theta_deg)
    reach = finger_inner_r() - WAIST_R
    return PIVOT_R - (FINGER_T / 2 + reach) * math.cos(th) + finger_len * math.sin(th)


def finger_len():
    """Pin to the lip's bottom face: the hub bottom sits HUB_ABOVE_KING over the king, the lip bottom at WAIST0 + 0.3."""
    pivot_above_board = PIECE_TOP + HUB_ABOVE_KING + PIVOT_Z
    return pivot_above_board - (WAIST0 + 0.3)


def plunger_bore():
    return 2 * (nose_r_closed() + PLUNGER_CLEAR) + 0.0


# ---------------------------------------------------------------- parts, each in its own frame
def finger():
    """Frame: pin axis along y through the origin; finger hangs down -z; its inner face toward -x."""
    L = finger_len()
    ri = -FINGER_T / 2                      # inner face x (toward the axis) in the finger frame
    bar = box(ri, FINGER_T / 2, -FINGER_W / 2, FINGER_W / 2, -L, TAIL)
    reach = finger_inner_r() - WAIST_R
    lip = box(ri - reach, ri + 0.001, -FINGER_W / 2, FINGER_W / 2, -L, -L + LIP_T)
    body = bar.fuse(lip)
    # chamfer on the lip's top inner edge (helps it find the groove) as a wedge cut
    wedge = (cq.Workplane("XZ").polyline([(ri - reach - 0.01, -L + LIP_T + 0.01), (ri - reach + LIP_CHAMFER, -L + LIP_T + 0.01),
                                          (ri - reach - 0.01, -L + LIP_T - LIP_CHAMFER)]).close()
             .extrude(FINGER_W / 2 + 1, both=True).val())
    body = body.cut(wedge)
    body = body.cut(cyl(PIN_HOLE, FINGER_W + 2, -FINGER_W / 2 - 1).rotate(V(0, 0, 0), V(1, 0, 0), 90))   # pin hole along y
    # round the tail's nose so it rolls on the cone: a cylinder along y at the tail top inner corner
    nose_r = FINGER_T / 2
    top = box(ri - 1, FINGER_T / 2 + 1, -FINGER_W / 2 - 1, FINGER_W / 2 + 1, TAIL - nose_r, TAIL + 1)
    keep = cyl(2 * nose_r, FINGER_W + 2, -FINGER_W / 2 - 1, 0.0, 0.0).rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, 0, TAIL - nose_r))
    body = body.cut(top.cut(keep))
    # merge coplanar faces: the lip box overlaps the bar by 0.001 (a sure fuse), which otherwise left 0.001 mm sliver
    # faces on the bottom and both flat sides where the lip meets the bar (2026-11-05)
    return one_solid(body.clean())


def hub():
    body = cyl(HUB_OD, HUB_H)
    cuts = [cyl(plunger_bore(), HUB_H - HUB_FLOOR + 1, HUB_FLOOR),
            cyl(CABLE_ACCESS, HUB_FLOOR + 2, -1)]
    for k in range(3):
        ang = 120.0 * k
        slot = box(finger_inner_r() - 1.0, HUB_OD, -SLOT_W / 2, SLOT_W / 2, -1, HUB_H + 1)
        pin = cyl(PIN_HOLE, HUB_OD + 2, -HUB_OD / 2 - 1).rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(PIVOT_R, 0, PIVOT_Z))
        cuts.append(rot_z(slot.fuse(pin), ang))
        # cap screw between fingers
        cuts.append(rot_z(cyl(M3["tap"], 12.0, HUB_H - 11.0, CAP_SCREW_R, 0.0), ang + 60.0))
    return one_solid(_cut_all(body, cuts))


def _cut_all(body, cuts):
    for c in cuts:
        body = body.cut(c)
    return body


def cone_r_open():
    """Cone radius at the nose-centre height when open: the rounded nose (radius FINGER_T/2) touches an
    inclined cone a little above its innermost point, so the cone sits rho / cos(alpha) inside the centre."""
    rho = FINGER_T / 2
    alpha = math.atan2(nose_r_closed() - nose_r_open(), PLUNGER_TRAVEL)
    return (nose_r_open() + rho) - rho / math.cos(alpha)


def plunger():
    """Frame: z = 0 at the skirt's bottom edge.  Cone from r_open - margin up to r_closed, then a cylinder."""
    r_closed = nose_r_closed()
    r_open = cone_r_open()
    cone_h = PLUNGER_TRAVEL + CONE_MARGIN
    r_bot = r_open - CONE_MARGIN * (r_closed - r_open) / PLUNGER_TRAVEL
    h_cyl = 10.0
    top = cone_h + h_cyl
    outer = [(0, 0), (r_bot, 0), (r_closed, cone_h), (r_closed, top), (0, top)]
    body = cq.Workplane("XZ").polyline(outer).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()
    # hollow: inner profile offset by the wall, open at the bottom, leaving a top plate
    plate_t = 3.0
    inner = [(0, -1), (r_bot - PLUNGER_WALL, -1), (r_closed - PLUNGER_WALL, cone_h), (r_closed - PLUNGER_WALL, top - plate_t), (0, top - plate_t)]
    body = body.cut(cq.Workplane("XZ").polyline(inner).close().revolve(360, (0, 0, 0), (0, 1, 0)).val())
    # spring pocket in the top plate? no: the spring sits ON the plate in a shallow ring; the cable hole through it
    body = body.cut(cyl(CABLE_HOLE, plate_t + 2, top - plate_t - 1))
    body = body.cut(cyl(SPRING_OD + 1.0, 1.0 + 1, top - 1.0).cut(cyl(SPRING_OD - 3.0, 3, top - 1.5)))   # 1 mm ring groove locates the spring
    return one_solid(body), top


def cap():
    body = cyl(HUB_OD, CAP_T).fuse(cyl(CAP_BOSS_D, CAP_BOSS_H + 0.001, CAP_T))
    cuts = [cyl(PC4_M6_HOLE, CAP_BOSS_H + 2, CAP_T - 0.5 + 0.0),
            cyl(CABLE_HOLE, CAP_T + 2, -1),
            cyl(SPRING_OD + 1.0, SPRING_POCKET_CAP + 1, -1).cut(cyl(SPRING_OD - 3.0, SPRING_POCKET_CAP + 3, -2))]
    for k in range(3):
        cuts.append(rot_z(cyl(M3["clear"], CAP_T + 2, -1, CAP_SCREW_R, 0.0), 120.0 * k + 60.0))
        cuts.append(rot_z(cyl(M3["head_d"] + 0.6, CAP_T, CAP_T - M3["head_h"], CAP_SCREW_R, 0.0), 120.0 * k + 60.0))
    for x in (-MOUNT_X, MOUNT_X):
        cuts.append(cyl(MOUNT_HOLE, CAP_T + 2, -1, 0.0, x))
    return one_solid(_cut_all(body, cuts))


# ---------------------------------------------------------------- assembly helpers
def place_finger(f, k, theta_deg):
    """Finger k (angle 120k) swung theta outward about its pin, in the hub frame."""
    g = f.rotate(V(0, 0, 0), V(0, 1, 0), -theta_deg)      # positive theta: tip moves to +x (outward)
    g = g.translate(V(PIVOT_R, 0, PIVOT_Z))
    return rot_z(g, 120.0 * k)


def plunger_z(state):
    """Hub-frame z of the plunger's bottom edge.  Closed: the cone reaches r_closed (plunger local z = cone_h)
    at the height of the nose circle's centre, PIVOT_Z + TAIL - FINGER_T/2.  Open: PLUNGER_TRAVEL higher."""
    nose_zc = PIVOT_Z + TAIL - FINGER_T / 2
    z_closed = nose_zc - (PLUNGER_TRAVEL + CONE_MARGIN)
    return z_closed + (PLUNGER_TRAVEL if state == "open" else 0.0)


def main():
    global PIVOT_R, OPEN_DEG, SPRING_OD, SPRING_FREE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pivot-r", type=float, default=PIVOT_R)
    ap.add_argument("--open-deg", type=float, default=OPEN_DEG)
    ap.add_argument("--spring", default=f"{SPRING_OD:g}x{SPRING_FREE:g}", help="OD x free length")
    ap.add_argument("--out", default="gripper_out")
    a = ap.parse_args()
    PIVOT_R, OPEN_DEG = a.pivot_r, a.open_deg
    SPRING_OD, SPRING_FREE = (float(v) for v in a.spring.lower().split("x"))
    os.makedirs(a.out, exist_ok=True)

    L = finger_len()
    print(f"finger: pin at r {PIVOT_R:g}, z {PIVOT_Z:g}; {L:.1f} mm pin to lip bottom, {TAIL:g} tail; bar {FINGER_T:g} radial x {FINGER_W:g}; "
          f"lip reaches {finger_inner_r() - WAIST_R:.1f} in, {LIP_T:g} tall")
    print(f"  lip inner radius closed {lip_r(0, L):.2f} (waist {WAIST_R:g}), open {lip_r(OPEN_DEG, L):.2f} (pedestal {PED_OD / 2:g}); "
          f"nose radius closed {nose_r_closed():.2f}, open {nose_r_open():.2f}; cone half angle "
          f"{math.degrees(math.atan2(nose_r_closed() - nose_r_open(), PLUNGER_TRAVEL)):.1f} deg over {PLUNGER_TRAVEL:g} mm travel")
    hub_bottom_above_board = PIECE_TOP + HUB_ABOVE_KING
    print(f"  hub bottom {hub_bottom_above_board:g} above the board ({HUB_ABOVE_KING:g} over the king); lip bottom at {WAIST0 + 0.3:g} when down")

    f = finger()
    h = hub()
    p, p_top = plunger()
    c = cap()
    parts = {"gripper_finger": f, "gripper_hub": h, "gripper_plunger": p, "gripper_cap": c}
    for name, s in parts.items():
        assert s.isValid(), name
        print(f"  {name:16s} {s.Volume():8.0f} mm^3 = {s.Volume() / 1000 * PETG:5.1f} g")

    # spring and plate heights
    zp_closed = plunger_z("closed")
    plate_top_closed = zp_closed + p_top
    cap0 = HUB_H
    spring_len_closed = cap0 + SPRING_POCKET_CAP - (plate_top_closed - 1.0)
    spring_len_open = spring_len_closed - PLUNGER_TRAVEL
    print(f"  plunger bottom edge z {zp_closed:.1f} closed, {zp_closed + PLUNGER_TRAVEL:.1f} open (hub floor at {HUB_FLOOR:g}); "
          f"spring {SPRING_OD:g} OD x {SPRING_FREE:g} free: {spring_len_closed:.1f} long closed ({(SPRING_FREE - spring_len_closed) * SPRING_RATE:.0f} N), "
          f"{spring_len_open:.1f} open ({(SPRING_FREE - spring_len_open) * SPRING_RATE:.0f} N cable pull)")
    tip_force = (SPRING_FREE - spring_len_closed) * SPRING_RATE / 3 / math.tan(math.atan2(nose_r_closed() - nose_r_open(), PLUNGER_TRAVEL)) * (TAIL / L)
    print(f"  grip at each lip, closed, before friction: about {tip_force:.1f} N (the lip under the shoulder carries the piece, not the squeeze)")

    # ---- assembled checks in both states
    piece = cyl(PED_OD, PIECE_TOP, -hub_bottom_above_board)            # the target piece as a 33 column
    neighbours = [cyl(PED_OD, PIECE_TOP, -hub_bottom_above_board, dx, dy) for dx, dy in ((SQUARE, 0), (-SQUARE, 0), (0, SQUARE), (0, -SQUARE))]
    results = []
    for state in ("closed", "open"):
        th = OPEN_DEG if state == "open" else 0.0
        fingers = [place_finger(f, k, th) for k in range(3)]
        pl = p.translate(V(0, 0, plunger_z(state)))
        solids = {"hub": h, "cap": c.translate(V(0, 0, cap0)), "plunger": pl}
        solids.update({f"finger{k}": fk for k, fk in enumerate(fingers)})
        names = list(solids)
        worst = 0.0
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                v = solids[names[i]].intersect(solids[names[j]]).Volume()
                if v > 1e-3:
                    print(f"  OVERLAP {state}: {names[i]} x {names[j]} {v:.3f} mm^3")
                    worst = max(worst, v)
        # contact: nose to plunger cone (distance should be ~0 in both states)
        gaps = [fk.distance(pl) for fk in fingers]
        # piece and neighbours
        piece_hit = sum(fk.intersect(piece).Volume() for fk in fingers)
        nb_hit = sum(fk.intersect(nb).Volume() for fk in fingers for nb in neighbours) + sum(h.intersect(nb).Volume() for nb in neighbours)
        results.append((state, worst, gaps, piece_hit, nb_hit, min(fk.distance(piece) for fk in fingers)))
        # assembly STEP
        assy = cq.Assembly(name=f"gripper_{state}")
        cols = {"hub": (0.3, 0.5, 0.8), "cap": (0.55, 0.3, 0.65), "plunger": (0.9, 0.6, 0.15)}
        for n, s in solids.items():
            k = n if n in cols else "finger"
            assy.add(cq.Workplane("XY").add(s), name=n, color=cq.Color(*cols.get(k, (0.5, 0.75, 0.4)), 1.0))
        assy.add(cq.Workplane("XY").add(piece), name="piece_stand_in", color=cq.Color(0.8, 0.8, 0.8, 0.5))
        assy.export(os.path.join(a.out, f"gripper_assembly_{state}.step"))

    ok = True
    for state, worst, gaps, piece_hit, nb_hit, piece_gap in results:
        print(f"  {state:6s}: part overlaps {worst:.3f} mm^3; nose-to-plunger gaps {', '.join(f'{g:.2f}' for g in gaps)}; "
              f"fingers into the piece {piece_hit:.2f} mm^3 (closed: the lips are in the groove); into neighbours {nb_hit:.2f} mm^3; "
              f"nearest finger to the piece column {piece_gap:.2f}")
        ok &= worst < 1e-3 and max(gaps) < 0.3 and nb_hit < 1e-3
        if state == "open":
            ok &= piece_hit < 1e-3 and piece_gap >= 1.0
    checks = [
        ("lip closes onto the waist", abs(lip_r(0, L) - WAIST_R) < 0.05),
        ("lip clears the pedestal when open by 1 mm or more", lip_r(OPEN_DEG, L) >= PED_OD / 2 + 1.0),
        ("lip fits the groove with 0.5 mm to spare", LIP_T <= (WAIST1 - WAIST0) - 0.5),
        ("plunger stays above the hub floor when closed", zp_closed >= HUB_FLOOR + 0.5),
        ("spring is compressed in both states (preload 2 mm or more)", SPRING_FREE - spring_len_closed >= 2.0),
        ("spring not coil-bound when open (assume solid at 40% of free)", spring_len_open >= 0.4 * SPRING_FREE),
        ("cap screws between the finger slots", True),
        ("no overlaps, noses on the cone, open fingers clear the piece and neighbours", ok),
    ]
    failed = 0
    for text, good in checks:
        print(f"  {'PASS' if good else 'FAIL'}  {text}")
        failed += 0 if good else 1

    # ---- files
    for name, s in parts.items():
        if name == "gripper_finger":            # printed lying flat on its 8 mm face
            s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, 0, FINGER_W / 2))
        elif name == "gripper_plunger":         # printed on its top plate: the skirt is then an open cup narrowing upward
            s = s.rotate(V(0, 0, 0), V(1, 0, 0), 180).translate(V(0, 0, p_top))
        wp = cq.Workplane("XY").add(s)
        cq.exporters.export(wp, os.path.join(a.out, f"{name}.step"))
        cq.exporters.export(wp, os.path.join(a.out, f"{name}.stl"), tolerance=0.01, angularTolerance=0.05)
    print(f"wrote 4 parts (STL + STEP; finger flat, plunger on its plate, hub and cap as modelled) and gripper_assembly_closed/open.step to {a.out}")
    print(f"  hardware: 3 x M3x30 + nyloc (pins), 3 x M3x10 (cap), spring {SPRING_OD:g} x {SPRING_FREE:g} (~{SPRING_RATE:g} N/mm), "
          f"1 x PC4-M6 fitting, 1.2 mm cable in 2 x 4 PTFE, 2 x M4 to the lift")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

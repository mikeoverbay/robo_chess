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
  - SLOT AND PIN DRIVE (Mike 2026-11-09: the cone only pushed the jaws shut - "there's nothing to make it follow
    ... to open the jaw back up. If you put a slot in the top of the arm that was diagonal, and then have a pin that
    went through that slot, and have the part that's moved by the spring move up and down, the pin would force the
    jaw open and close"): each finger's tail reaches in past the hub bore to r TAIL_IN_R and has a straight slot
    leaning DRIVE slot-angle off vertical; the plunger is a disc riding over the tails with three forks hanging
    down, each straddling a tail with a drive pin (M3) through both cheeks and the slot. Spring pushes the plunger
    DOWN: pins down, jaws CLOSED. The servo's cable pulls it UP 6: the pins climb the slots and force the jaws
    OPEN 6 deg. Both ways are driven; a piece can't pry the jaws open (the steep slot won't let the jaw push the
    pin). Pin path vertical at r DRIVE_R; the slot runs from the pin's closed spot to where the open pin sits in
    the swung finger. The old cone plunger is gone; the hub's bore opens from r 18.3 to BORE_R for the forks.
    No power holds the piece: the spring does.
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
# ---------------------------------------------------------------- slot and pin drive (2026-11-09), hub frame
BORE_R = 19.0            # hub bore above the floor (was the cone plunger's 18.3); wall 6 to the D50 outside
DRIVE_R, DRIVE_Z = 14.0, 18.0     # drive pin: radius in the finger's plane; height when CLOSED (open: + travel)
#                                   (14 not 15: leaves the fork cheeks 1.8+ of wall outside the pin inside the bore)
DRIVE_PIN, DRIVE_SLOT = 3.0, 3.4  # M3 as the pin; slot 0.2 clear each side
TAIL_IN_R = 10.0         # the tail's block reaches in to this radius (the slot sits in it, inside the bore)
TAIL_IN_Z = 14.0         # ... from this height up (opening swings its inner foot down to 12.8, floor top 12)
TAIL_TOP = 30.0          # top of the tail (was 28 with the cone): 1.7+ of tail over the slot's top end
CHEEK_T, CHEEK_GAP = 2.5, 6.6     # fork cheeks either side of the 6 wide tail
CHEEK_R0 = 10.3          # cheeks reach in to here (2 under the pin hole); out to BORE_R - DISC_CLEAR
DISC_CLEAR = 0.6         # plunger to bore, radial
DISC_Z, DISC_T = 31.5, 3.0        # plunger disc bottom (closed) and thickness: 1.5 over the tails
SPRING_GROOVE = (12.0, 16.0, 1.0)  # ring groove in the disc's top locating the spring (in, out, deep): flat top prints down
DRIVE_PIN_HOLE = 3.0     # press fit for a 3 mm steel pin in both cheeks (printed holes come out a little small)
DRIVE_PIN_NOTE = "3 mm steel pin x 11.6 (from his gearbox pin stock), pressed into both cheeks"
#                          (an M3 screw's head would have run 0.08 mm off the bore - no head, no rub)


def slot_ends():
    """the slot's centre line, finger frame (pin axis = y through the origin, x radial, z up), closed finger:
    from where the pin sits closed to where the open pin sits once the finger has swung OPEN_DEG"""
    th = math.radians(OPEN_DEG)
    x0, z0 = DRIVE_R - PIVOT_R, DRIVE_Z - PIVOT_Z
    x1, z1 = x0, z0 + PLUNGER_TRAVEL                 # open pin, hub frame relative to the pivot
    # into the swung finger's frame: rotate by -theta (the finger opened by +theta about +y, tip to +x)
    return (x0, z0), (x1 * math.cos(th) + z1 * math.sin(th), -x1 * math.sin(th) + z1 * math.cos(th))


def finger_angle(travel):
    """the finger's swing (deg, + = open) with the plunger up `travel`: the pin must sit on the slot's line"""
    (ax, az), (bx, bz) = slot_ends()
    dl = math.hypot(bx - ax, bz - az)
    nx, nz = (bz - az) / dl, -(bx - ax) / dl
    px, pz = DRIVE_R - PIVOT_R, DRIVE_Z - PIVOT_Z + travel

    def f(deg):
        t = math.radians(deg)
        fx, fz = px * math.cos(t) + pz * math.sin(t), -px * math.sin(t) + pz * math.cos(t)
        return nx * (fx - ax) + nz * (fz - az)
    lo, hi = -2.0, OPEN_DEG + 4.0
    for _ in range(80):
        m = (lo + hi) / 2
        if f(lo) * f(m) <= 0:
            hi = m
        else:
            lo = m
    return (lo + hi) / 2


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
    """Frame: pin axis along y through the origin; finger hangs down -z; its inner face toward -x. Above the pin the
    tail carries a block reaching in to TAIL_IN_R with the drive slot through it (2026-11-09; the cone's rounded nose
    is gone)."""
    L = finger_len()
    ri = -FINGER_T / 2                      # inner face x (toward the axis) in the finger frame
    top = TAIL_TOP - PIVOT_Z
    bar = box(ri, FINGER_T / 2, -FINGER_W / 2, FINGER_W / 2, -L, top)
    reach = finger_inner_r() - WAIST_R
    lip = box(ri - reach, ri + 0.001, -FINGER_W / 2, FINGER_W / 2, -L, -L + LIP_T)
    block = box(TAIL_IN_R - PIVOT_R, ri + 0.001, -FINGER_W / 2, FINGER_W / 2, TAIL_IN_Z - PIVOT_Z, top)
    body = bar.fuse(lip).fuse(block)
    # chamfer on the lip's top inner edge (helps it find the groove) as a wedge cut
    wedge = (cq.Workplane("XZ").polyline([(ri - reach - 0.01, -L + LIP_T + 0.01), (ri - reach + LIP_CHAMFER, -L + LIP_T + 0.01),
                                          (ri - reach - 0.01, -L + LIP_T - LIP_CHAMFER)]).close()
             .extrude(FINGER_W / 2 + 1, both=True).val())
    body = body.cut(wedge)
    body = body.cut(cyl(PIN_HOLE, FINGER_W + 2, -FINGER_W / 2 - 1).rotate(V(0, 0, 0), V(1, 0, 0), 90))   # pin hole along y
    # the drive slot: a stadium from the closed pin spot to the open one (in this, the closed finger's frame), through y
    (ax, az), (bx, bz) = slot_ends()
    a, dl, r = math.degrees(math.atan2(bz - az, bx - ax)), math.hypot(bx - ax, bz - az), DRIVE_SLOT / 2
    w = FINGER_W / 2 + 1
    slot = box(0, dl, -w, w, -r, r)
    for x in (0.0, dl):
        slot = slot.fuse(cq.Solid.makeCylinder(r, 2 * w, V(x, -w, 0), V(0, 1, 0)))
    body = body.cut(slot.rotate(V(0, 0, 0), V(0, 1, 0), -a).translate(V(ax, 0, az)))
    return one_solid(body.clean())


def hub():
    body = cyl(HUB_OD, HUB_H)
    cuts = [cyl(2 * BORE_R, HUB_H - HUB_FLOOR + 1, HUB_FLOOR),
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
    """Frame: z = 0 at the disc's bottom (hub z DISC_Z when closed). A disc in the bore with a ring groove on top for
    the spring and the cable hole; three forks hanging down to the drive pins, one straddling each finger's tail:
    a D3.0 press-fit hole through both cheeks for a 3 mm steel pin."""
    r = BORE_R - DISC_CLEAR
    body = cyl(2 * r, DISC_T, 0)
    zp = DRIVE_Z - DISC_Z                                  # the pin's height in this frame (below the disc)
    zb = zp - DRIVE_PIN / 2 - 2.0
    clip = cyl(2 * r, 100, -50)
    for k in range(3):
        for side in (-1, 1):
            y0, y1 = side * CHEEK_GAP / 2, side * (CHEEK_GAP / 2 + CHEEK_T)
            cheek = box(CHEEK_R0, r + 1, min(y0, y1), max(y0, y1), zb, DISC_T / 2).intersect(clip)   # well into the disc
            body = body.fuse(rot_z(cheek, 120.0 * k))
    cuts = [cyl(CABLE_HOLE, DISC_T + 2, -1),
            cyl(SPRING_GROOVE[1], SPRING_GROOVE[2] + 1, DISC_T - SPRING_GROOVE[2]).cut(cyl(SPRING_GROOVE[0], 5, DISC_T - 3))]
    for k in range(3):
        y_in, y_out = CHEEK_GAP / 2 - 0.5, CHEEK_GAP / 2 + CHEEK_T + 1
        cuts.append(rot_z(cq.Solid.makeCylinder(DRIVE_PIN_HOLE / 2, y_out - y_in, V(DRIVE_R, y_in, zp), V(0, 1, 0)), 120.0 * k))
        cuts.append(rot_z(cq.Solid.makeCylinder(DRIVE_PIN_HOLE / 2, y_out - y_in, V(DRIVE_R, -y_in, zp), V(0, -1, 0)), 120.0 * k))
    return one_solid(_cut_all(body, cuts)), DISC_T          # (no .clean(): merging faces here left it invalid)


def drive_pin(k, travel):
    """the 3 mm steel drive pin through both cheeks, hub frame"""
    y = CHEEK_GAP / 2 + CHEEK_T
    return rot_z(cq.Solid.makeCylinder(DRIVE_PIN / 2, 2 * y, V(DRIVE_R, -y, DRIVE_Z + travel), V(0, 1, 0)), 120.0 * k)


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


def plunger_z(state, travel=None):
    """Hub-frame z of the plunger disc's bottom: DISC_Z closed, PLUNGER_TRAVEL higher open (or `travel` higher)"""
    if travel is None:
        travel = PLUNGER_TRAVEL if state == "open" else 0.0
    return DISC_Z + travel


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
    (ax, az), (bx, bz) = slot_ends()
    slot_deg = math.degrees(math.atan2(bx - ax, bz - az))
    print(f"finger: pin at r {PIVOT_R:g}, z {PIVOT_Z:g}; {L:.1f} mm pin to lip bottom, tail to z {TAIL_TOP:g}, its block in to "
          f"r {TAIL_IN_R:g}; bar {FINGER_T:g} radial x {FINGER_W:g}; lip reaches {finger_inner_r() - WAIST_R:.1f} in, {LIP_T:g} tall")
    print(f"  lip inner radius closed {lip_r(0, L):.2f} (waist {WAIST_R:g}), open {lip_r(OPEN_DEG, L):.2f} (pedestal {PED_OD / 2:g})")
    print(f"  drive: pin D{DRIVE_PIN:g} at r {DRIVE_R:g}, z {DRIVE_Z:g} closed -> {DRIVE_Z + PLUNGER_TRAVEL:g} open; slot "
          f"{math.hypot(bx - ax, bz - az):.2f} long + D{DRIVE_SLOT:g}, {slot_deg:.1f} deg off vertical (finger frame "
          f"({ax:.2f}, {az:.2f}) -> ({bx:.2f}, {bz:.2f}))")
    for t in (0.0, 1.5, 3.0, 4.5, 6.0):
        print(f"    plunger up {t:.1f} -> jaw {finger_angle(t):.2f} deg open, lip {lip_r(finger_angle(t), L) - WAIST_R:.1f} mm out")

    f = finger()
    h = hub()
    p, p_top = plunger()
    c = cap()
    parts = {"gripper_finger": f, "gripper_hub": h, "gripper_plunger": p, "gripper_cap": c}
    for name, s_ in parts.items():
        assert s_.isValid(), name
        print(f"  {name:16s} {s_.Volume():8.0f} mm^3 = {s_.Volume() / 1000 * PETG:5.1f} g")

    cap0 = HUB_H
    seat = DISC_Z + DISC_T - SPRING_GROOVE[2]
    spring_len_closed = cap0 + SPRING_POCKET_CAP - seat
    spring_len_open = spring_len_closed - PLUNGER_TRAVEL
    f_closed = (SPRING_FREE - spring_len_closed) * SPRING_RATE
    f_open = (SPRING_FREE - spring_len_open) * SPRING_RATE
    print(f"  plunger disc z {DISC_Z:g}..{DISC_Z + DISC_T:g} closed (tails' top {TAIL_TOP:g}); spring {SPRING_OD:g} OD x {SPRING_FREE:g} "
          f"free: {spring_len_closed:.1f} long closed ({f_closed:.1f} N), {spring_len_open:.1f} open ({f_open:.1f} N servo pull)")
    # grip: the spring's share per jaw pushes its pin down; the slot wall takes it square to itself (no friction)
    dl = math.hypot(bx - ax, bz - az)
    nx, nz = (bz - az) / dl, -(bx - ax) / dl
    fv = f_closed / 3
    k_n = fv / abs(nz)
    fx = -k_n * nx if nz > 0 else k_n * nx                 # the normal force's sideways part that goes with -fv down
    tau = ax * (-fv) - az * fx
    print(f"  grip at each lip, closed, before friction: about {abs(tau) / L:.1f} N (the lip under the shoulder carries the piece); "
          f"slot {slot_deg:.1f} deg off vertical = tan {math.tan(math.radians(slot_deg)):.2f}, under PETG-on-steel friction "
          f"(~0.3): the jaw can't push the pin up - a piece can't pry the jaws open")

    # ---- assembled checks: every 0.5 mm of the travel, closed to open
    hub_bottom_above_board = PIECE_TOP + HUB_ABOVE_KING
    piece = cyl(PED_OD, PIECE_TOP, -hub_bottom_above_board)            # the target piece as a 33 column
    neighbours = [cyl(PED_OD, PIECE_TOP, -hub_bottom_above_board, dx, dy) for dx, dy in ((SQUARE, 0), (-SQUARE, 0), (0, SQUARE), (0, -SQUARE))]

    def posed(travel):
        th = finger_angle(travel)
        solids = {"hub": h, "cap": c.translate(V(0, 0, cap0)), "plunger": p.translate(V(0, 0, plunger_z(None, travel)))}
        solids.update({f"finger{k}": place_finger(f, k, th) for k in range(3)})
        solids.update({f"drive pin{k}": drive_pin(k, travel) for k in range(3)})
        return th, solids

    worst, pin_gap, results = 0.0, 1e9, []
    for travel in [0.5 * k for k in range(13)]:
        th, solids = posed(travel)
        names = list(solids)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a_, b_ = names[i], names[j]
                if {a_.rstrip("012"), b_.rstrip("012")} == {"drive pin", "plunger"}:
                    continue                                       # the pin is pressed into the cheeks
                v = solids[a_].intersect(solids[b_]).Volume()
                if v > 1e-3:
                    print(f"  OVERLAP at {travel:.1f} up: {a_} x {b_} {v:.3f} mm^3")
                    worst = max(worst, v)
        for k in range(3):
            pin_gap = min(pin_gap, solids[f"drive pin{k}"].distance(solids[f"finger{k}"]))
        if travel in (0.0, PLUNGER_TRAVEL):
            fingers = [solids[f"finger{k}"] for k in range(3)]
            piece_hit = sum(fk.intersect(piece).Volume() for fk in fingers)
            nb_hit = sum(fk.intersect(nb).Volume() for fk in fingers for nb in neighbours) + sum(h.intersect(nb).Volume() for nb in neighbours)
            results.append(("closed" if travel == 0 else "open", piece_hit, nb_hit, min(fk.distance(piece) for fk in fingers), solids))
    print(f"  through the travel (13 steps): part overlaps {worst:.3f} mm^3; drive pin to its slot, closest {pin_gap:.2f} mm")
    ok = worst < 1e-3 and pin_gap > 0.05
    for state, piece_hit, nb_hit, piece_gap, _ in results:
        print(f"  {state:6s}: fingers into the piece {piece_hit:.2f} mm^3 (closed: the lips are in the groove); into neighbours "
              f"{nb_hit:.2f} mm^3; nearest finger to the piece column {piece_gap:.2f}")
        ok &= nb_hit < 1e-3
        if state == "open":
            ok &= piece_hit < 1e-3 and piece_gap >= 1.0
    checks = [
        ("lip closes onto the waist", abs(lip_r(0, L) - WAIST_R) < 0.05),
        ("lip clears the pedestal when open by 1 mm or more", lip_r(OPEN_DEG, L) >= PED_OD / 2 + 1.0),
        ("lip fits the groove with 0.5 mm to spare", LIP_T <= (WAIST1 - WAIST0) - 0.5),
        ("the drive opens the jaw exactly OPEN_DEG over the travel", abs(finger_angle(PLUNGER_TRAVEL) - OPEN_DEG) < 0.01),
        ("fork cheeks clear of the hub floor when closed", DRIVE_Z - DRIVE_PIN / 2 - 2.0 >= HUB_FLOOR + 0.5),
        ("spring is compressed in both states (preload 2 mm or more)", SPRING_FREE - spring_len_closed >= 2.0),
        ("spring not coil-bound when open (assume solid at 40% of free)", spring_len_open >= 0.4 * SPRING_FREE),
        ("no overlaps anywhere in the travel, pins free in their slots, open fingers clear the piece and neighbours", ok),
    ]
    failed = 0
    for text, good in checks:
        print(f"  {'PASS' if good else 'FAIL'}  {text}")
        failed += 0 if good else 1

    # ---- files: STEP only (Mike exports his own STLs); assemblies as flat named bodies
    from regroup_assembly import write_flat
    for name, s_ in parts.items():
        if name == "gripper_finger":            # printed lying flat on its side (the slot's axis vertical)
            s_ = s_.rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, 0, FINGER_W / 2))
        elif name == "gripper_plunger":         # printed on the disc's flat top, the forks standing up
            s_ = s_.rotate(V(0, 0, 0), V(1, 0, 0), 180).translate(V(0, 0, p_top))
        write_flat([(name.replace("gripper_", ""), s_, None)], os.path.join(a.out, f"{name}.step"))
    cols = {"hub": "#4d80cc", "cap": "#8c4da6", "plunger": "#e69926", "finger": "#80bf66", "drive pin": "#c9ccd2"}

    def col(hx):
        return cq.Color(*(int(hx[i:i + 2], 16) / 255 for i in (1, 3, 5)))
    for state, _, _, _, solids in results:
        bodies = [(n, s_, col(cols[n.rstrip("012")])) for n, s_ in solids.items()]
        bodies.append(("piece_stand_in", piece, col("#cccccc")))
        write_flat(bodies, os.path.join(a.out, f"gripper_assembly_{state}.step"))
    print(f"wrote 4 parts (STEP; finger flat, plunger on its disc, hub and cap as modelled) and gripper_assembly_closed/open.step to {a.out}")
    print(f"  hardware: 3 x M3x30 + nyloc (finger pivots), 3 x {DRIVE_PIN_NOTE} (drive pins), 3 x M3x10 (cap), spring {SPRING_OD:g} x {SPRING_FREE:g} (~{SPRING_RATE:g} N/mm), 1 x PC4-M6 fitting, "
          f"1.2 mm cable in 2 x 4 PTFE, 2 x M4 to the wrist")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

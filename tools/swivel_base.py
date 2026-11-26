#!/usr/bin/env python3
"""
swivel_base.py - the arm's swivel (yaw) base, mark-up version (2026-11-04, Mike: "two more 6808s... a bevel gear,
relatively good ratios so it isn't jerky... motor straight out the back of the base").

    python swivel_base.py

Swivel frame: Z up from the wood (Z 0), X toward the board, yaw axis = Z axis.
  - housing (fixed, screwed to the wood): flange + tower with two 6808 seats (outer races against a middle shoulder,
    centres 43 apart) + NEMA 17 face plate at the back, motor shaft along +X (pointing at the axis), 2 gussets.
    Cable: through the hollow spindle, out a slot at the bottom of the tower and a groove under the flange to the back.
  - turntable (turns): square plate (the arm base's 100 mm foot screws onto it, corners cut to clear the motor plate),
    72T straight bevel gear under it (teeth down), hollow spindle 40 OD through both bearings.
  - collar: under the lower bearing's inner race, 3 #4 screws up into the spindle wall.
  - pinion: 18T on the motor shaft (5 mm D bore, M3 grub screw), hub back to the motor face.
  Bevel pair: module 1.5 at the outer end, 18:72 = 4:1, 20 deg, face 14, standard addenda, 0.15 backlash per gear.
  Teeth: straight bevel, Tredgold - each tooth's outer section is the involute tooth of the virtual spur gear
  (z / cos delta) in the plane tangent to the back cone, swept straight to the cone apex (all flank lines meet there).
  - board (mark-up): 60 mm squares, 15 border, 25 thick, on the same wood, playing edge D_EDGE in front of the axis.
Placed into the arm's design frame for the viewer (design -x = toward the board, -y = up, z = across).
Out: complete assembly/swivel_fixed.step, complete assembly/swivel_turntable.step, chessboard/board_layout.step.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402

from regroup_assembly import write_step  # noqa: E402
from view_step import read_step  # noqa: E402
import arm_base as AB  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT_FIXED = os.path.join(ROOT, "complete assembly", "swivel_fixed.step")
OUT_TURN = os.path.join(ROOT, "complete assembly", "swivel_turntable.step")
# the 18T pinion on its own (Mike 2026-11-06: "the bevel gear on the motor does not spin and stay in sync with the bevel
# gear on the turntable"): Robot Studio turns it about the motor shaft, 4 x J0 (scene.json "gears")
OUT_PINION = os.path.join(ROOT, "complete assembly", "swivel_pinion.step")
PINION_AXIS = (0.0, None, 30.6)    # design frame: the motor shaft, along design -x at y = y_wood - Z_APEX, z = Z_MID
OUT_BOARD = os.path.join(ROOT, "chessboard", "board_layout.step")
BEARING = os.path.join(ROOT, "complete assembly", "arm2.step")   # his 6808 model: solid 0, centre (-200, 0), z 54.2..61.2

# ---- bevel pair
MOD, ZG, ZP, PA = 1.5, 72, 18, math.radians(20)
FACE, BACKLASH = 14.0, 0.15
ADD, DED = 1.0 * MOD, 1.25 * MOD
DG, DP = math.atan2(ZG, ZP), math.atan2(ZP, ZG)           # pitch cone half angles
RG, RP = MOD * ZG / 2, MOD * ZP / 2                       # 54, 13.5
RCONE = math.hypot(RG, RP)                                 # outer cone distance 55.66
K_IN = (RCONE - FACE) / RCONE

# ---- layout (swivel frame)
Z_APEX = 60.0                       # both cone apexes; pinion / motor axis height
B_W, B_OD, B_ID = 7.0, 52.0, 40.0   # 6808
Z_B1, Z_B2 = 18.0, 61.0             # bottoms of the lower / upper bearing
SHOULDER_D = 47.0                   # middle shoulder bore (outer races stop on it)
TOWER_R = 30.0
FLANGE = (-78.0, 62.0, 62.0)        # x from, x to, half width in y (back edge flush with the motor upright, Mike)
FLANGE_T = 8.0                      # 8 so the cover's #4 screws can go into its edge
FLANGE_HOLE_IN = 14.0               # mounting (wood) screws this far in from both edges at each corner: inside the cover
# cover screws, horizontal through the cover's lip into the flange edge: (x, y, outward normal x, y), mirrored in y
FLANGE_SCREWS = [(-40.0, 62.0, 0, 1), (35.0, 62.0, 0, 1),                         # side (Mike: two a side)
                 (62.0, 10.0, 1, 0),                                             # front (Mike: 10 off centre)
                 (-78.0, 44.0, -1, 0)]                                           # back
FLANGE_PILOT = (2.4, 12.0)          # #4 pilot into the flange edge: dia, depth
Z_PLATE0, PLATE_T = 77.0, 12.0
GEAR_OD = 2 * (RG + ADD * math.cos(DG))   # 108.73: the turntable plate is this round too (Mike), as is the arm foot
Z_PILOT0 = 74.0                     # foot screw pilots go down to here (1 mm above the gear's root cone at r 45)
SPINDLE_OD, SPINDLE_ID = 40.0, 30.0
WIRE_D = 28.0                       # through the turntable centre
COLLAR = (13.0, 5.0, 45.0)          # bottom z, thickness, OD
X_FACE = -72.0                      # motor face plate, front face (motor flange against its back face)
FACE_T = 6.0
# Mike: the wire conduit comes out of the RIGHT side (standing behind the robot, facing the board) = -y, not the back
CABLE_X0, CABLE_X1, CABLE_H = 2.0, 22.0, 16.0   # exit just in front of the -y side wall; 20 wide, 16 tall
WALL_TOP = 66.0                     # side walls tower <-> motor plate: 3 under the gear's lowest tooth tip (69)
WALL_Y, WALL_T = 24.5, 11.0         # side walls y 19..30: outside face tangent to the bearing barrel (r 30, Mike).
#                                     Pinion tips reach 15. Housing is ONE part (Mike merged the
#                                     split halves back together)
WALL_SCREW_X = -62.0                # Mike: a screw hole down each wall's top face, NO countersink - his split cover
#                                     screws on here; r 66.5 so a screwdriver comes down clear of the gear (r 54.4)
WALL_PILOT = (2.4, 20.0)            # straight #4 pilot: dia, depth (the cover carries the clearance hole)
COVER_T, COVER_CLEAR = 4.0, 0.2     # split cover: wall above the flange; clearance to the flange edge
COVER_LIP_T = 3.0                   # Mike: a rabbet - the lip comes down over the flange edge, flush with the base bottom
COVER_INSET = 0.8                   # the cover's inside face sits this far in from the flange edge (rests on the flange)
# Mike: the back opening is the upright's outline - its two sides and its TOP (the roof bridges over the upright)
COVER_EDGE_R = 2.0                  # Mike: outside corners and top edges rounded
COVER_GEAR_R = 56.0                 # round opening: gear / turntable OD 54.36 + 1.6
COVER_PAD_D = 12.0                  # pads from the roof down onto the wall tops (z 66)
COVER_CB_D, COVER_FLOOR = 6.5, 4.0  # deep counterbore down each pad, 4 mm floor under the screw head (#4 x 3/4: 15 in the wall)
# J0 homing (Mike 2026-11-05: "rotate that arm the maximum we'll ever rotate back to pick up the queen or the part in the
# corner ... put the sensor in the bottom of the case under the servo drive and put the magnet centered under the angle
# of where that rotation is in the top of the case"). The Hall (DRV5032FB breakout) sits in the servo tab's underside
# (arm_tube.py) on the arm's centre plane, HOME_R out from the yaw axis, and rides with J0; the 6 x 2 magnet sits in
# the RIGHT cover half's roof (his pick, P50.F12) right under it when J0 is at j0_reach_max(): the arm on the spare
# queen's tray spot on his left (black's, the corner spot beside the robot). The tab passes 2.35 over the roof.
HOME_R = 60.3                       # the roof ring there runs r 56 (the gear opening) to 63.5 (its flat top ends at
#                                     y -63.2): the magnet's pocket 1.2 from the opening, its rim on the flat
MAGNET_D, MAGNET_DEPTH = 6.2, 2.2   # 6 x 2 magnet glued 0.2 under the roof top: flush is best (tools/hall_homing.py -
#                                     sunk deeper the trip point hardly moves and the field's edge only gets softer)
# Mike: no lobes. A rib from the bearing tower out to the inside of the cover's front wall; two #4 through the front
# wall into it near the top (the flange screws hold the bottom)
FRONT_RIB_W = 20.0                  # centred on y = 0, flange to WALL_TOP
FRONT_SCREW_Y, FRONT_SCREW_Z = 5.0, 58.0    # one per cover half: y +-5, under the rib's top (66)
SCREW_CLEAR4, SCREW_PILOT4 = 3.4, 2.4
# Mike (2026-11-05, P54.E31 - a side screw's hole in the right cover's lip): "That should be beveled for our wood
# screws": every screw through the covers' sides (the flange screws in the lip, the front screw into the rib) gets an
# 82 deg countersink on the outside for a #4 flat-head wood screw (head 0.225" = 5.7) so the head sits flush.
CSK4_D = 6.0


def csk4(face, n):
    """an 82 deg countersink for a #4 flat head: its rim (CSK4_D) on the outer face at `face`, going in against n"""
    k = (CSK4_D - SCREW_CLEAR4) / 2 / math.tan(math.radians(41))
    e = 0.01                                                   # starts just outside, so the face is cut cleanly
    return cq.Solid.makeCone(CSK4_D / 2 + e * math.tan(math.radians(41)), SCREW_CLEAR4 / 2, k + e, face + n * e, n * -1)
NEMA, NEMA_L, NEMA_HOLES, NEMA_PILOT = 42.3, 48.0, 31.0, 22.0
UPRIGHT_TOP = Z_APEX + NEMA / 2 + 4  # 85.15, the motor upright's top face
COVER_ROOF_TOP = UPRIGHT_TOP + 0.5 + 4.0   # roof 4 thick over the upright (0.5 clear): top 89.65, about the turntable's top
SHAFT_D, SHAFT_L = 5.0, 24.0
HUB_D = 16.0
WOOD_SCREW_CLEAR, WOOD_CSK = 4.6, 9.2      # #8
FOOT_PILOT = 3.5                          # #8 into plastic (the arm foot's corner screws)
SQ, BORDER, BOARD_T = 60.0, 15.0, 22.0
# BOARD_T (the squares' top over the robot's bottom face) was 25: the base and the dock stood 3 down in pockets in the
# board's sheet. 2026-11-24 Mike: "If we don't put the pockets on the board, the recesses for the parking stand, will
# it still work? I can't get this thing onto the machine to cut those pockets." / "Yeah, so let's do that." - no
# pockets: the base and the dock stand on the sheet's top, which the cells stand on too (cell underside to its wood's
# top: 22), so the squares are 22 over the robot's face; the arm's board_y is 235 (was 232), the shoulder 235 above.
D_EDGE = 122.0                    # yaw axis -> playing edge: shoulder 232 above the board -> reach 152..640;
#                                   nearest squares 154, far corners 609


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} pieces"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def cyl(d, z0, z1, x=0.0, y=0.0, axis=(0, 0, 1)):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0) if axis == (0, 0, 1) else V(x, y, z0), V(*axis))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def revolve(pts):
    """solid of revolution about Z from (r, z) points (closed polygon in the r >= 0 half plane)."""
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()


# ------------------------------------------------------------------ straight bevel gear (apex at origin, axis +Z)
def tooth_2d(zv, thick):
    """One tooth of the virtual spur gear: (u, v), v = 0 on the pitch circle, +v toward the tip, u across."""
    rv = zv * MOD / 2
    rb, ra, rf = rv * math.cos(PA), rv + ADD, rv - DED
    beta = thick / (2 * rv)
    inv = lambda a: math.tan(a) - a  # noqa: E731
    flank = []
    r0 = max(rb, rf)
    n = 10
    for i in range(n + 1):
        rho = r0 + (ra - r0) * i / n
        a = math.acos(min(1.0, rb / rho))
        flank.append((rho, beta + inv(PA) - inv(a)))
    if rf < rb:
        flank.insert(0, (rf, flank[0][1]))
    flank.insert(0, (rf - 0.6 * MOD, flank[0][1]))          # down into the blank
    right = [(r * math.sin(p), r * math.cos(p) - rv) for r, p in flank]
    left = [(-u, v) for u, v in reversed(right)]
    return right + left


def bevel(z, delta, thick, blank_pts, phase_deg=0.0):
    """Gear with z teeth on a pitch cone of half angle delta; blank_pts = (r, z) polygon revolved and fused."""
    zv = z / math.cos(delta)
    g = V(math.sin(delta), 0, math.cos(delta))
    nrm = V(math.cos(delta), 0, -math.sin(delta))           # toward the tooth tips
    t = V(0, 1, 0)
    P = g * RCONE
    pts = [P + t * u + nrm * v for u, v in tooth_2d(zv, thick)]
    outer = cq.Wire.makePolygon(pts, close=True)
    inner = cq.Wire.makePolygon([p * K_IN for p in pts], close=True)
    tooth = cq.Solid.makeLoft([outer, inner], ruled=True)
    teeth = [tooth.rotate(V(0, 0, 0), V(0, 0, 1), phase_deg + k * 360.0 / z) for k in range(z)]
    blank = revolve(blank_pts)
    return blank.fuse(*teeth).clean()


def root_pts(delta):
    """(r, z) of the root cone at the outer and inner ends."""
    g = V(math.sin(delta), 0, math.cos(delta))
    nrm = V(math.cos(delta), 0, -math.sin(delta))
    o = g * RCONE - nrm * DED
    return (o.x, o.z), (o.x * K_IN, o.z * K_IN)


def tooth_thickness():
    t = math.pi * MOD / 2 - BACKLASH
    return t, t


# ------------------------------------------------------------------ parts
def pilot_bottom(r):
    """a foot-screw pilot's bottom: Z_PILOT0, or 1 mm above the gear's root cone at the hole's outer edge if that is
    higher (Mike 2026-11-05 moved the foot screws out to r 47.4, where the root rises to 74.05 at the edge)"""
    (ro, zo), (ri, zi) = root_pts(DG)
    re = r + FOOT_PILOT / 2
    return max(Z_PILOT0, Z_APEX + zi + (re - ri) / (ro - ri) * (zo - zi) + 1.0)


def turntable():
    (ro, zo), (ri, zi) = root_pts(DG)
    za = Z_APEX
    top = Z_PLATE0 - za + 0.5
    blank = [(SPINDLE_ID / 2, zi), (ri, zi), (ro, zo), (ro, top), (SPINDLE_ID / 2, top)]
    tg, _ = tooth_thickness()
    gear = bevel(ZG, DG, tg, blank, phase_deg=0.0).translate(V(0, 0, za))
    # round plate, the gear's outside diameter (the arm foot is the same)
    pl = cyl(GEAR_OD, Z_PLATE0, Z_PLATE0 + PLATE_T)
    z_bt = Z_B2 + B_W                                     # top of the upper bearing: spindle shoulder sits on it
    spindle = cyl(SPINDLE_OD, Z_B1, z_bt + 0.01).fuse(cyl(44.8, z_bt, za + zi + 0.5))   # shoulder on the inner race
    body = gear.fuse(pl).fuse(spindle)
    cuts = [cyl(SPINDLE_ID, Z_B1 - 1, za + zi + 1), cyl(WIRE_D, za + zi, Z_PLATE0 + PLATE_T + 1)]
    for x, z in AB.foot_holes_tube():                     # the arm foot's 4 screws (design x, z -> swivel -X, Y)
        cuts.append(cyl(FOOT_PILOT, pilot_bottom(math.hypot(x, z - AB.Z_MID)), Z_PLATE0 + PLATE_T + 1,
                        -x, -(z - AB.Z_MID)))
    for k in range(3):                                    # collar screws (pilots in the spindle wall)
        a = math.radians(90 + 120 * k)
        r = (SPINDLE_OD + SPINDLE_ID) / 4
        cuts.append(cyl(2.4, Z_B1 - 1, Z_B1 + 15, r * math.cos(a), r * math.sin(a)))
    for cu in cuts:
        body = body.cut(cu)
    return one(body.clean(), "turntable")


def collar():
    z0, t, od = COLLAR
    c = cyl(od, z0, z0 + t).cut(cyl(SPINDLE_ID, z0 - 1, z0 + t + 1))
    r = (SPINDLE_OD + SPINDLE_ID) / 4
    for k in range(3):
        a = math.radians(90 + 120 * k)
        x, y = r * math.cos(a), r * math.sin(a)
        c = c.cut(cyl(3.2, z0 - 1, z0 + t + 1, x, y))
        c = c.cut(cq.Solid.makeCone(3.0, 1.6, 1.6, V(x, y, z0 - 0.01), V(0, 0, 1)))
    return one(c.clean(), "collar")


def pinion_local():
    """Pinion in its own frame (apex origin, axis +Z): teeth, blank, hub to the motor face, D bore, grub hole."""
    (ro, zo), (ri, zi) = root_pts(DP)
    hub_end = -X_FACE - 1.5                   # stops 1.5 short of the motor face (its pilot boss is 2 tall)
    blank = [(0, zi), (ri, zi), (ro, zo), (HUB_D / 2, zo), (HUB_D / 2, hub_end), (0, hub_end)]
    _, tp = tooth_thickness()
    p = bevel(ZP, DP, tp, blank, phase_deg=180.0 / ZP)
    bore = cyl(SHAFT_D + 0.15, zi - 1, hub_end + 1)
    flat = box(-5, 5, SHAFT_D / 2 - 0.5 + 0.1, 5, zi - 1, hub_end + 1)     # D flat (4.5 across)
    p = p.cut(bore.cut(flat))
    zg = (zo + hub_end) / 2
    p = p.cut(cq.Solid.makeCylinder(1.25, HUB_D, V(0, 0, zg), V(0, -1, 0)))  # M3 grub (tap 2.5) through the flat side
    return one(p.clean(), "pinion")


def pinion_to_swivel():
    """pinion frame (x', y', z') -> swivel: z' -> -X (toward the back), x' -> +Z, y' -> +Y; apex at (0, 0, Z_APEX)."""
    t = gp_Trsf()
    t.SetValues(0, 0, -1, 0,
                0, 1, 0, 0,
                1, 0, 0, Z_APEX)
    return cq.Location(t)


def motor():
    """Mike's NEMA 17 (cycloid/nema17.step): front face on the motor plate's back, shaft along +X on the pinion axis,
    D flat on +Y like the pinion's bore. Returns (body, end caps + shaft)."""
    import nema17 as N
    body, caps = N.load()
    caps = caps.cut(box(-5, 5, SHAFT_D / 2 - 0.5, 5, 4, 30)).clean()            # D flat, 4 mm from the face on
    t = gp_Trsf()
    t.SetValues(0, 0, 1, X_FACE - FACE_T,                                     # motor z (shaft) -> +X
                0, 1, 0, 0,
                -1, 0, 0, Z_APEX)
    loc = cq.Location(t)
    return (cq.Shape.cast(body.wrapped.Moved(loc.wrapped)), cq.Shape.cast(caps.wrapped.Moved(loc.wrapped)))


def housing():
    x0, x1, hw = FLANGE
    h = box(x0, x1, -hw, hw, 0, FLANGE_T)
    h = h.fuse(cyl(2 * TOWER_R, 0, Z_B2 + B_W))
    # motor face plate + 2 side walls back into the tower (they carry the cap above the split) + a screw boss each
    top = Z_APEX + NEMA / 2 + 4
    h = h.fuse(box(X_FACE - FACE_T, X_FACE, -NEMA / 2 - 4, NEMA / 2 + 4, 0, top))
    for sy in (-1, 1):
        h = h.fuse(box(X_FACE - 0.01, 0.0, sy * WALL_Y - WALL_T / 2, sy * WALL_Y + WALL_T / 2, FLANGE_T - 0.01, WALL_TOP))   # to the tangent point
    cuts = [cyl(B_OD, -1, Z_B1 + B_W), cyl(SHOULDER_D, Z_B1 + B_W - 0.01, Z_B2 + 0.01), cyl(B_OD, Z_B2, Z_B2 + B_W + 1)]
    # motor: pilot clearance + 4 x M3
    cuts.append(cq.Solid.makeCylinder(NEMA_PILOT / 2 + 0.5, FACE_T + 2, V(X_FACE - FACE_T - 1, 0, Z_APEX), V(1, 0, 0)))
    for sy in (-1, 1):
        for sz in (-1, 1):
            cuts.append(cq.Solid.makeCylinder(1.7, FACE_T + 2, V(X_FACE - FACE_T - 1, sy * NEMA_HOLES / 2, Z_APEX + sz * NEMA_HOLES / 2), V(1, 0, 0)))
    # front rib: tower -> just inside the cover's front wall (0.2 clear), 20 wide, flange to the wall tops
    x_rib = x1 - COVER_INSET - 0.2
    h = h.fuse(box(TOWER_R - 2, x_rib, -FRONT_RIB_W / 2, FRONT_RIB_W / 2, FLANGE_T - 0.01, WALL_TOP))
    for sy in (-1, 1):
        cuts.append(cq.Solid.makeCylinder(SCREW_PILOT4 / 2, 13, V(x_rib + 1, sy * FRONT_SCREW_Y, FRONT_SCREW_Z), V(-1, 0, 0)))
    # #4 pilots into the flange edge for the cover's lip screws
    for fx, fy, nx, ny in FLANGE_SCREWS:
        for sy in (-1, 1):
            yy, ny2 = fy * sy, ny * sy
            cuts.append(cq.Solid.makeCylinder(FLANGE_PILOT[0] / 2, FLANGE_PILOT[1] + 1,
                                              V(fx + nx * 1, yy + ny2 * 1, FLANGE_T / 2), V(-nx, -ny2, 0)))
    # #4 pilots straight down each side wall's top face (no countersink): the split cover screws on here
    for sy in (-1, 1):
        y = sy * WALL_Y
        cuts.append(cyl(WALL_PILOT[0], WALL_TOP - WALL_PILOT[1], WALL_TOP + 1, WALL_SCREW_X, y))
    # cable: slot out of the tower's -y side (below the lower bearing) and a slot through the flange to its -y edge
    cuts.append(box(CABLE_X0, CABLE_X1, -TOWER_R - 6, 0, -1, CABLE_H))
    cuts.append(box(CABLE_X0, CABLE_X1, -hw - 1, -TOWER_R + 4, -1, FLANGE_T))
    # 4 wood screws, countersunk
    k = (WOOD_CSK - WOOD_SCREW_CLEAR) / 2 / 0.8693
    for x in (x0 + FLANGE_HOLE_IN, x1 - FLANGE_HOLE_IN):
        for sy in (-1, 1):
            y = sy * (hw - FLANGE_HOLE_IN)
            cuts.append(cyl(WOOD_SCREW_CLEAR, -1, FLANGE_T + 1, x, y))
            cuts.append(cq.Solid.makeCone(WOOD_SCREW_CLEAR / 2, WOOD_CSK / 2, k, V(x, y, FLANGE_T - k + 0.01), V(0, 0, 1)))
    for cu in cuts:
        h = h.cut(cu)
    return one(h.clean(), "housing")


def j0_reach_max():
    """the furthest J0 ever turns in a game (deg, + = his left): over every square and tray spot, the yaw that puts the
    claw over it - the viewer's own (ikSolve: atan2(-dz, -dx) about the yaw axis, design frame). Black's spare queen
    in the corner spot beside the robot sets it (white's is the same to the right)."""
    import board_pieces as BP                                  # (it imports this module: load it only when asked)
    pts = [BP.square_at(f + r) for f in "abcdefgh" for r in "12345678"]
    pts += BP.tray_spots("white") + BP.tray_spots("black")
    # 2026-11-10: the trays lost their two rows beside the robot (Mike: "only 16", no spare queens). The J0 home magnet
    # (home_magnet_xy) and the cover's roof were placed for the old corner spot, so those rows stay in here: re-running
    # this or hall_homing.py doesn't move them.
    y0 = 4 * SQ + BORDER + BP.TRAY_GAP + BP.TRAY_PITCH / 2
    pts += [BP.design(-28.0 + r * BP.TRAY_PITCH, s * (y0 + c * BP.TRAY_PITCH)) for r in (0, 1) for c in (0, 1) for s in (-1, 1)]
    yaws = [math.degrees(math.atan2(-(p[2] - AB.Z_MID), -p[0])) for p in pts]
    return max(yaws), min(yaws)


def home_magnet_xy():
    """the magnet's centre (swivel frame): where the Hall is (arm's centre plane, HOME_R out the back) at J0's
    furthest left turn. The arm's direction at J0 = q is the swivel's X turned q about Z; the tab points opposite."""
    q = math.radians(j0_reach_max()[0] + 180.0)
    return HOME_R * math.cos(q), HOME_R * math.sin(q)


def cover():
    """Mike's split cover (2026-11-04): a 4 mm shell round the outside of the square base flange, split on the centre
    line (y = 0) into left / right halves. Roof with a round opening for the gear (1.6 clear of its OD), set high
    enough to clear the pinion's hub so it prints flat, roof-down; two pads under it land on the side walls' #4 pilots.
    Back: an opening the shape of the motor upright (its sides and its top) for the upright, the motor and the
    swivel motor's own wires; the roof bridges over it. The arm's cables leave through a notch at the bottom of the
    -y side (right, seen from behind). Front: a #4 through the front wall into the housing's front rib, one per half."""
    x0, x1, hw = FLANGE
    c, t = COVER_CLEAR, COVER_T
    lip, ins = COVER_LIP_T, COVER_INSET
    ox0, ox1, oy = x0 - c - lip, x1 + c + lip, hw + c + lip           # outside faces
    shell = box(ox0, ox1, -oy, oy, 0, COVER_ROOF_TOP)                  # down to the base bottom
    # Mike (P14.E1/E6/E25/E28/E61, both covers): r COVER_EDGE_R on the outside's vertical corners and top edges
    shell = shell.fillet(COVER_EDGE_R, [e for e in shell.Edges() if abs(e.Center().z - COVER_ROOF_TOP) < 1e-6
                                        or abs(e.startPoint().z - e.endPoint().z) > 1.0])
    shell = shell.cut(box(x0 + ins, x1 - ins, -hw + ins, hw - ins, -1, COVER_ROOF_TOP - t))   # inside, above the flange
    shell = shell.cut(box(x0 - c, x1 + c, -hw - c, hw + c, -1, FLANGE_T))                     # rabbet over the flange
    shell = shell.cut(box(CABLE_X0 - 0.5, CABLE_X1 + 0.5, -oy - 1, -hw + ins + 1, -1, CABLE_H + 0.5))   # cable exit, -y side
    for fx, fy, nx, ny in FLANGE_SCREWS:                      # clearance holes through the lip, countersunk outside
        for sy in (-1, 1):
            yy, ny2 = fy * sy, ny * sy
            shell = shell.cut(cq.Solid.makeCylinder(SCREW_CLEAR4 / 2, lip + c + 2, V(fx - nx * 1, yy - ny2 * 1, FLANGE_T / 2),
                                                    V(nx, ny2, 0)))
            face = V(fx if nx == 0 else (ox1 if nx > 0 else ox0), yy if ny2 == 0 else (oy if ny2 > 0 else -oy), FLANGE_T / 2)
            shell = shell.cut(csk4(face, V(nx, ny2, 0)))
    for sy in (-1, 1):                                        # pads down onto the wall tops
        shell = shell.fuse(cyl(COVER_PAD_D, WALL_TOP, COVER_ROOF_TOP - t + 0.01, WALL_SCREW_X, sy * WALL_Y))
    shell = shell.cut(cyl(2 * COVER_GEAR_R, 0, COVER_ROOF_TOP + 1))            # round opening for the gear
    # back opening = the upright's outline + 0.5: its sides and up to just over its top (the roof bridges over it)
    shell = shell.cut(box(x0 - 20, X_FACE + 0.5, -NEMA / 2 - 4 - 0.5, NEMA / 2 + 4 + 0.5, -1, UPRIGHT_TOP + 0.5))
    for sy in (-1, 1):                                        # screws into the walls: deep counterbore + clearance
        x, y = WALL_SCREW_X, sy * WALL_Y
        shell = shell.cut(cyl(SCREW_CLEAR4, WALL_TOP - 1, WALL_TOP + COVER_FLOOR + 0.01, x, y))
        shell = shell.cut(cyl(COVER_CB_D, WALL_TOP + COVER_FLOOR, COVER_ROOF_TOP + 1, x, y))
    halves = []
    for sy in (-1, 1):
        h = shell.intersect(box(x0 - 30, x1 + 30, 0 if sy > 0 else -hw - 30, hw + 30 if sy > 0 else 0, -1, COVER_ROOF_TOP + 1))
        # one #4 through the front wall into the housing's front rib, near the top
        h = h.cut(cq.Solid.makeCylinder(SCREW_CLEAR4 / 2, t + c + lip + 4, V(x1 - ins - 1, sy * FRONT_SCREW_Y, FRONT_SCREW_Z),
                                        V(1, 0, 0)))
        h = h.cut(csk4(V(ox1, sy * FRONT_SCREW_Y, FRONT_SCREW_Z), V(1, 0, 0)))          # countersunk, flush head
        if sy < 0:                                            # the J0 home magnet, in the right half's roof
            mx, my = home_magnet_xy()
            h = h.cut(cyl(MAGNET_D, COVER_ROOF_TOP - MAGNET_DEPTH, COVER_ROOF_TOP + 1, mx, my))
        halves.append(one(h.clean(), "cover " + ("left" if sy > 0 else "right")))
    # Mike's view: standing behind the robot facing the board, +y is on the LEFT, -y on the RIGHT
    return halves[1], halves[0]                                # (left = +y, right = -y with the cable exit)


def board():
    """on the wood (Z 0..25), playing area in front (+X), centred on the axis."""
    xn = D_EDGE
    half = 4 * SQ
    slab = box(xn - BORDER, xn + 8 * SQ + BORDER, -half - BORDER, half + BORDER, 0, BOARD_T)
    for r in range(8):
        for c in range(8):
            if (r + c) % 2 == 1:          # a1 dark, h1 light (r 0 = rank 8 by the robot, c 0 = the a-file)
                x0 = xn + r * SQ
                y0 = -half + c * SQ
                slab = slab.cut(box(x0 + 0.75, x0 + SQ - 0.75, y0 + 0.75, y0 + SQ - 0.75, BOARD_T - 0.5, BOARD_T + 1))
    return one(slab.clean(), "board")


# ------------------------------------------------------------------ checks
def mesh_check(gear, pin_loc, pin):
    """overlap of the bevel pair while turning through one gear pitch (gear theta, pinion 4 x theta)."""
    worst = 0.0
    for sgn in (1, -1):
        w = 0.0
        for i in range(5):
            th = i * 360.0 / ZG / 4
            g = gear.rotate(V(0, 0, 0), V(0, 0, 1), th)
            p = cq.Shape.cast(pin.rotate(V(0, 0, 0), V(0, 0, 1), sgn * th * ZG / ZP).wrapped.Moved(pin_loc.wrapped))
            w = max(w, g.intersect(p).Volume())
        print(f"   pinion turning {'+' if sgn > 0 else '-'}: worst overlap {w:.3f} mm^3"
              + ("  (the wrong way - this one isn't how they turn)" if w > 1.0 else "  (the way they turn)"))
        worst = w if sgn == 1 else min(worst, w)
    return worst


def to_design(shape, y_wood):
    """swivel frame -> the arm's design frame: X -> -x, Y -> -z (+ Z_MID), Z (up) -> -y (+ y of the wood)."""
    t = gp_Trsf()
    t.SetValues(-1, 0, 0, 0,
                0, 0, -1, y_wood,
                0, -1, 0, AB.Z_MID)
    return cq.Shape.cast(shape.wrapped.Moved(cq.Location(t).wrapped))


def main():
    assert abs(AB.FOOT_D - GEAR_OD) < 0.01, (AB.FOOT_D, GEAR_OD)
    tt, co, ho = turntable(), collar(), housing()
    mo_body, mo_caps = motor()
    mo = mo_body.fuse(mo_caps)
    pl = pinion_to_swivel()
    pin = pinion_local()
    pin_sw = cq.Shape.cast(pin.wrapped.Moved(pl.wrapped))
    turn_top = Z_PLATE0 + PLATE_T
    y_wood = AB.Y_CUT + AB.FOOT_T + turn_top           # design y of the wood (J1 at y 0)
    print(f"turntable top {turn_top:.0f} above the wood; J1 {y_wood:.0f} above the wood, "
          f"{y_wood - BOARD_T:.0f} above the board top")
    for n, s in (("turntable", tt), ("collar", co), ("housing", ho), ("pinion", pin_sw), ("motor", mo)):
        b = s.BoundingBox()
        print(f"   {n:9s} {s.Volume() / 1000:6.1f} cm^3  x {b.xmin:6.1f}..{b.xmax:6.1f} y {b.ymin:6.1f}..{b.ymax:6.1f} z {b.zmin:5.1f}..{b.zmax:5.1f}")
    # clearances between fixed and turning parts (turntable swept: check at several angles)
    for ang in (0, 15, 30, 45):
        t2 = tt.rotate(V(0, 0, 0), V(0, 0, 1), ang)
        for n, s in (("housing", ho), ("motor", mo)):
            v = t2.intersect(s).Volume()
            if v > 0.01:
                print(f"   OVERLAP turntable@{ang} x {n}: {v:.2f}")
    for n, s in (("housing", ho), ("turntable", tt)):
        v = co.intersect(s).Volume()
        if v > 0.01:
            print(f"   OVERLAP collar x {n}: {v:.2f}")
    for n, s in (("housing", ho), ("motor", mo)):
        v = pin_sw.intersect(s).Volume()
        if v > 0.01:
            print(f"   OVERLAP pinion x {n}: {v:.2f}")
    print("bevel mesh:")
    mesh_check(tt, pl, pin)

    fixed = cq.Assembly(name="swivel_fixed")
    fixed.add(to_design(ho, y_wood), name="swivel housing", color=cq.Color(0.55, 0.55, 0.6))
    pin_a = cq.Assembly(name="swivel_pinion")
    pin_a.add(to_design(pin_sw, y_wood), name="swivel pinion 18T", color=cq.Color(0.95, 0.75, 0.3))
    write_step(pin_a, OUT_PINION)
    print(f"pinion axis (design): through (0, {y_wood - Z_APEX:g}, {AB.Z_MID:g}) along -x; it turns +{ZG / ZP:g} x J0 about it")
    fixed.add(to_design(mo_body, y_wood), name="swivel motor NEMA 17", color=cq.Color(0.12, 0.12, 0.13))
    fixed.add(to_design(mo_caps, y_wood), name="swivel motor NEMA 17 end caps", color=cq.Color(0.78, 0.79, 0.81))
    cl, cr = cover()
    v = cl.intersect(cr).Volume()
    if v > 0.01:
        print(f"   OVERLAP cover left x right: {v:.2f}")
    for side, cv in (("left", cl), ("right", cr)):
        b = cv.BoundingBox()
        print(f"   cover {side}: {cv.Volume() / 1000:.1f} cm^3, {b.xlen:.0f} x {b.ylen:.0f} x {b.zlen:.0f}")
        for n, s in (("housing", ho), ("motor", mo), ("pinion", pin_sw)):
            v = cv.intersect(s).Volume()
            if v > 0.01:
                print(f"   OVERLAP cover {side} x {n}: {v:.2f}")
        for ang in (0, 20, 40):
            v = cv.intersect(tt.rotate(V(0, 0, 0), V(0, 0, 1), ang)).Volume()
            if v > 0.01:
                print(f"   OVERLAP cover {side} x turntable@{ang}: {v:.2f}")
    fixed.add(to_design(cl, y_wood), name="swivel cover left", color=cq.Color(0.35, 0.55, 0.75))
    fixed.add(to_design(cr, y_wood), name="swivel cover right", color=cq.Color(0.35, 0.6, 0.8))
    mx, my = home_magnet_xy()
    jmax, jmin = j0_reach_max()
    mag = cyl(6.0, COVER_ROOF_TOP - MAGNET_DEPTH, COVER_ROOF_TOP - MAGNET_DEPTH + 2.0, mx, my)
    rr = math.hypot(mx, my)
    print(f"J0 reach {jmin:.2f} .. {jmax:.2f} deg; home magnet at ({mx:.2f}, {my:.2f}) r {rr:.2f}: "
          f"{rr - MAGNET_D / 2 - COVER_GEAR_R:.2f} from the gear opening, rim at y {my - MAGNET_D / 2:.2f} "
          f"(the roof's flat ends at y {-(FLANGE[2] + COVER_CLEAR + COVER_LIP_T - COVER_EDGE_R):.2f})")
    v = mag.intersect(cr).Volume()
    if v > 0.01:
        print(f"   OVERLAP magnet x cover right: {v:.2f}")
    fixed.add(to_design(mag, y_wood), name="J0 home magnet 6x2", color=cq.Color(0.82, 0.83, 0.86))
    brg = read_step(BEARING)[0][1]
    for nm, zb in (("lower", Z_B1), ("upper", Z_B2)):
        fixed.add(to_design(brg.translate(V(200, 0, zb - 54.2)), y_wood), name=f"6808 swivel {nm}",
                  color=cq.Color(0.8, 0.8, 0.85))
    write_step(fixed, OUT_FIXED)
    turn = cq.Assembly(name="swivel_turntable")
    turn.add(to_design(tt, y_wood), name="swivel turntable 72T", color=cq.Color(0.9, 0.55, 0.3))
    turn.add(to_design(co, y_wood), name="swivel collar", color=cq.Color(0.6, 0.6, 0.65))
    write_step(turn, OUT_TURN)
    b = cq.Assembly(name="board_layout")
    b.add(to_design(board(), y_wood), name="board", color=cq.Color(0.86, 0.78, 0.62))
    write_step(b, OUT_BOARD)
    for p in (OUT_FIXED, OUT_TURN, OUT_BOARD, OUT_PINION):
        print(p)


if __name__ == "__main__":
    main()

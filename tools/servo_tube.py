#!/usr/bin/env python3
"""
servo_tube.py - the claw's cable servo for the tube-link arm (2026-11-05, Mike: "the servo motor with an arm on it...
on the back of the middle arm... tilts up at 30 degrees so we pre-start the arc angle"; "make the bottom of the mount
fit those two holes... I'll just glue the mount into those two holes if they have a boss").

    python servo_tube.py

Same servo, horn, cradle and tube stop as servo_mount.py (MG996R / DS3218 size, one-arm horn, cable from the horn tip
to the stop, the tube glued in the stop), moved onto link B (the middle arm) by J2 and tilted 30 deg in the arm's plane
about its J2 end: the J3 end, where the tube leaves, stands out from the link, so the tube starts already curving
into the arc over the J3 knuckle.
Mount (printed, one piece):
  - base strip flat on link B's side panel (its outside face, design y -30) along the line of the two side holes Mike
    picked (P2.E50 / E53: D16 at (-286, 35.6) and (-254.5, 45.7)), 4 thick, BASE_W wide;
  - two bosses into those holes (D15.8 snug for glue, BOSS_L long - the holes open into the tube after ~3 mm);
  - two triangular side ribs from the strip up to the tilted cradle plate (light, not a solid wedge);
  - the old cradle plate, tab posts and tube stop, tilted 30 deg.
The horn tip, even pointing straight at the stop, stays HORN_CLEAR from it: it can't reach the tube's end.
Prints the tube's start (where it leaves the stop and which way) for scene.json "bowden".
Out: complete assembly/servo_on_link_b.step (servo, horn, servo bracket) - rides with link B (joint J2).

    python servo_tube.py --on base [--out DIR]      (the default)

The same box straight up on the column's foot (Mike 2026-11-05, see ON below): no tilt, no yaw, bolted flat onto the
vertical mount plate that arm_tube.py builds (with the tab and gussets) on the base body.
Out: complete assembly/servo_on_base.step - rides with the column (joint J0).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402

import servo_mount as SM  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
OUTS = {"link_b": os.path.join(ROOT, "complete assembly", "servo_on_link_b.step"),
        "base": os.path.join(ROOT, "complete assembly", "servo_on_base.step")}
# Where the box goes (python servo_tube.py [--on base|link_b]).
# base (Mike 2026-11-05, replacing the box glued into the column's D16 holes): "on the outside of [the foot] extend a
# tab straight out the back ... put a couple of gussets on it to support the vertical mount for the servo"; "remove the
# twist ... straight up, and straight out, no tilt, no nothing, just mounted on that plate". The same box (cradle,
# servo, horn, fitting) with NO tilt and NO yaw: its plate flat on a vertical mount plate on the column's flat back face
# (x +30), standing straight up on the foot with the fitting end on top (the tube leaves straight up), the servo's shaft
# straight out the back (+x), centred on the arm's centre plane; 4 x M3 flat heads into the mount plate. arm_tube.py
# builds the mount plate, the tab and two gussets (beside the box) onto the base body from tab_layout(). Rides with J0.
ON = "base"
FOOT_Y = 158.5                                   # the column's foot (its top face, design y): the box stands on it
MOUNT_X, MOUNT_T = 30.0, 8.0                     # the column's flat back face (design x); the mount plate on it
#                                                  (8 thick so the M3 pilots stay in it: the column wall behind has
#                                                  its D16 holes)
BOX_Z = 30.6                                     # the box's plate centred on the arm's centre plane
BOX_GAP = 0.3                                    # the box's lower end this far above the foot's top
TOP_ABOVE = 3.0                                  # the mount plate reaches this far above the box
GUSSET_T, GUSSET_GAP, GUSSET_UP = 4.0, 1.4, 50.0  # thickness, clear of the box's (lid's) outline, height up the plate
# The cover (Mike 2026-11-05: "a cover ... a couple of slots that that tab slides into ... push the cover straight in
# ... friction on those two faces ... print it to size, it'll stay put. We won't need any screws"): a channel pushed
# straight in toward the column, its side walls a snug fit between the gussets' inner faces (exactly their gap, no
# clearance: the friction holds its bottom), slots in the side walls taking the end wall's edges (the end wall - the
# Bowden fitting's "tab" - holds its top; the walls stand COVER_ABOVE over it so it can't lift off), the outer wall
# past the end wall's edge, standing on the tab, stopping on the mount plate. Open toward the column (the mount plate
# closes it), at the bottom (the tab) and over the end wall (the fitting). A notch for the servo's wire at the bottom.
COVER_T, COVER_SIDE_T, COVER_CLEAR = 2.0, 3.2, 0.2   # outer wall; side walls (gusset face to the cradle plate's edge
#                                                    less 0.2); clearance round the end wall and the cradle plate
COVER_ABOVE = 2.0                                # side walls this far above the end wall's top
WIRE_NOTCH = None                                # (was an 8 x 5 notch in the cover; the wires now go into the column)
# Mike (2026-11-05): "a hole through ... the arm swivel riser ... so we can run the wires down the center of the hollow
# pivoting tube on the rotating table ... big enough to get a 4-pin connector, JST style through there": D16 (a 4-pin
# JST XH housing is 12.4 x 5.75, 13.7 across its corners; the column's own side holes are D16) straight along x through
# the servo bracket's plate, the mount plate and the column's back wall into the hollow column, then down the foot's
# D20 wire hole into the turntable's hollow spindle. Under the servo, between the two lowest bolts (1.35 from their
# countersinks), 2 above the column's floor inside.
WIRE_HOLE_D, WIRE_HOLE_Y = 16.0, 144.0
TAB_PAST = COVER_CLEAR + COVER_T + 1.0           # the tab and gussets end this far past the box's outside (the end
#                                                  wall's edge): under the cover's outer wall, 1 to spare (3.2)
TAB_IN = 40.0                                    # the tab starts inside the foot (fused), design x
TAB_UNDER = 165.0                                # tab underside (design y): 92 above the wood, 2.35 over the swivel
#                                                  cover's roof (89.65) - 3 above the foot's underside: print supported
BOLTS = [(-69.0, 9.0), (-69.0, 34.0), (-104.0, 34.0), (-122.0, 34.5)]   # M3 flat heads, servo_mount frame (x, z):
#                                                  under the lower tab post, and along the fitting side clear of the
#                                                  servo's pocket and the fitting's web
M3_CLEAR, M3_CSK, M3_PILOT = 3.4, 6.3, 2.5
M3_PILOT_DEPTH = MOUNT_T - 0.3                   # into the mount plate only (M3 x 10 flat heads: 4 bracket + 6)


def placed(shape):
    """link B's frame -> where the box goes (link_b only; ON == "base" uses on_tab)"""
    return shape

SIDE_Y = -30.0                                   # link B's outside face (design frame, joint J2)
HOLES = [(-286.0, 35.614), (-254.5, 45.694)]     # Mike's two D16 side holes (x, z)
HOLE_D = 16.0
BOSS_D, BOSS_L = 15.8, 5.0
BASE_T, BASE_W, BASE_END = 4.0, 20.0, 12.0       # strip thickness, width, past each hole centre
RIB_T = 4.0
PIVOT_IN = 8.0                                   # the tilt's pivot this far in from the strip's J2 end
TILTS = {"link_b": 30.0, "base": 0.0}            # deg about z: the J3 (fitting) end out. On the column the box
#                                                  stands flat: the fitting then points straight up at the J1 hub (the
#                                                  30 deg lean aimed it up and BACK, and the tube had to hook round)


def tilt():
    return TILTS[ON]
EDGE = ((-359.0, -8.0), (-241.5, 31.6))          # Mike's P2.E37, the side panel's lower edge (x, z): the servo's side
YAW = math.degrees(math.atan2(EDGE[1][1] - EDGE[0][1], EDGE[1][0] - EDGE[0][0]))   # turned to run along it (18.6)
RIB_W = (4.5, 8.5)                               # ribs this far either side of the strip's centre line (on the strip)
SLIDE = 10.0                                     # Mike: the servo slid down the plate (toward J2), away from the tube end
# Bowden push fitting like the Ender 3's (PC4-M10, screwed into the end wall, its clip keeps the tube in). Sizes are
# generous guesses - no maker publishes the M10's: measure yours and set these.
FIT_TAP_D = 9.0                                  # M10 x 1 fitting cuts its own thread in the print
FIT_THREAD = 8.0                                 # thread hole depth
FIT_HEX_AC = 16.2                                # hex across corners (14 A/F): the wall's face carries it
FIT_OUT = 14.0                                   # collet top above the wall's face: where the tube leaves
FIT_WALL = 12.0                                  # through the fitting's boss: thread 8, the tube's socket to 10.5 (it
CABLE_D = 2.5                                    # bottoms on that shoulder - the pull goes into the wall), cable hole
WALL_T = 4.0                                     # the end wall itself; the boss (FIT_BOSS_D) carries the rest of the
FIT_BOSS_D = 15.0                                # thread, and a web (WEB_T, WEB_RUN along the plate) takes the cable's
WEB_T, WEB_RUN = 5.0, 20.0                       # pull (MG996R stall ~66 N at 15 mm, 44 up) down into the plate
# room for a snap-on lid over the whole thing: its sides wrap the plate's long edges and hook under them, its top over
# the horn; the end wall is the lid's outline so the lid closes flush against it
LID_T, LID_CLEAR, HORN_SCREW = 2.0, 1.5, 2.0
# the push fitting itself, shown (Mike: "make the fitting visible"), PC4-M10 like the Ender's - the same guessed sizes:
# hex FIT_HEX_AC across corners x 6, a neck, the blue release collet to FIT_OUT; the thread in the wall is not drawn
FIT_HEX_T, FIT_NECK_D, FIT_COLLET_D, FIT_BORE = 6.0, 11.0, 10.0, 4.1
HORN_TURN = 90.0                                 # Mike: the horn turned 90 deg, square to the cable at rest ...
SERVO_DZ = -SM.HORN_R                            # ... and the servo moved over so the horn's cable hole stays on the
#                                                  tube's line: the hole then moves along the cable (R sin a) with only
#                                                  R (1 - cos a) sideways - ~0.3 mm for the claw's 6 mm centred on square


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} pieces"
    return s[0]


def plate_z():
    """the cradle plate's z span (servo_mount frame): follows the servo over, and carries the fitting's hex"""
    zs = SM.Z_C + SERVO_DZ
    return (min(SM.PLATE_Z[0], zs - SM.SERVO_W / 2 - 3.0),
            max(zs + SM.SERVO_W / 2 + 3.0, SM.Z_C + FIT_HEX_AC / 2 + 1.5))


def lid_top():
    """the box's outside over the servo (servo_mount frame y): the lid's top, as cradle() sets the end wall"""
    yc = SM.WALL_Y - SM.PLATE_T - (SM.SERVO_H + SM.TURRET_H + SM.SPLINE_H - 1.5 + SM.HORN_T / 2)
    return min(yc - SM.HORN_T / 2 - HORN_SCREW - LID_CLEAR - LID_T, yc - FIT_HEX_AC / 2 - 0.5,
               yc - FIT_BOSS_D / 2 - LID_CLEAR - LID_T)


def tab_shift():
    """servo_mount frame -> on the base: x -> design y (the fitting end, -x, up), -y -> design +x (the plate's back on
    the mount plate's face, the shaft out the back), z -> design z. The translation (x, y, z)."""
    pz0, pz1 = plate_z()
    return (MOUNT_X + MOUNT_T + SM.WALL_Y, FOOT_Y - BOX_GAP - SM.PLATE_X[1], BOX_Z - (pz0 + pz1) / 2)


def to_tab(x, y, z):
    cx, cy, cz = tab_shift()
    return (-y + cx, x + cy, z + cz)


def on_tab(shape):
    cx, cy, cz = tab_shift()
    t = gp_Trsf()
    t.SetValues(0, -1, 0, cx,
                1, 0, 0, cy,
                0, 0, 1, cz)
    return cq.Shape.cast(shape.wrapped.Moved(cq.Location(t).wrapped))


def tab_layout():
    """what arm_tube.py builds onto the base body for the box (design frame = the column's, J0):
    plate: (x0, x1), (y_top, y_foot), (z0, z1) - the vertical mount plate on the column's back face
    tab: (x0, x1), (y_top, y_under), (z0, z1) - straight out the back from the foot, under the box
    gussets: [(z0, z1)] - triangles from the plate's face at the tab's top, GUSSET_UP up the plate, out to the tab's end
    pilots: [(y, z)] - the M3 pilots into the plate's face (x1), M3_PILOT x M3_PILOT_DEPTH
    box: the box's (lid's) outline: x out, y top, (z0, z1)"""
    pz0, pz1 = plate_z()
    cz = tab_shift()[2]
    lz0, lz1 = pz0 - LID_T + cz, pz1 + LID_T + cz            # the box across (the end wall = the lid's outline)
    out_x = to_tab(0.0, lid_top(), 0.0)[0]
    top_y = to_tab(SM.PLATE_X[0], 0.0, 0.0)[1]
    g = [(lz0 - GUSSET_GAP - GUSSET_T, lz0 - GUSSET_GAP), (lz1 + GUSSET_GAP, lz1 + GUSSET_GAP + GUSSET_T)]
    z0, z1 = g[0][0], g[1][1]
    x_end = out_x + TAB_PAST
    return {"plate": ((MOUNT_X, MOUNT_X + MOUNT_T), (top_y - TOP_ABOVE, FOOT_Y), (z0, z1)),
            "tab": ((TAB_IN, x_end), (FOOT_Y, TAB_UNDER), (z0, z1)),
            "gussets": g, "gusset_up": GUSSET_UP, "x_end": x_end,
            "pilots": [to_tab(bx, 0.0, bz)[1:] for bx, bz in BOLTS],
            "box": (out_x, top_y, (lz0, lz1)),
            "wire_hole": (WIRE_HOLE_Y, BOX_Z, WIRE_HOLE_D)}


def cover_on_tab():
    """the servo box's push-on cover, design frame (the column's, J0) - see COVER_* above"""
    L = tab_layout()
    (_, px1), _, _ = L["plate"]
    gz0, gz1 = L["gussets"][0][1], L["gussets"][1][0]           # the gussets' inner faces (P3.F7 / F6)
    out_x, top_y, (lz0, lz1) = L["box"]                          # the end wall: outer edge, top, sides
    ew_bot = top_y + WALL_T                                      # its underside
    x0, x1 = px1 + COVER_CLEAR, out_x + COVER_CLEAR + COVER_T    # open end (by the mount plate), outer face
    y0, y1 = top_y - COVER_CLEAR - COVER_ABOVE, FOOT_Y           # top, bottom (on the tab)
    zi0, zi1 = gz0 + COVER_SIDE_T, gz1 - COVER_SIDE_T            # the side walls' inner faces
    pz0, pz1 = plate_z()
    cz = tab_shift()[2]
    assert zi0 <= pz0 + cz - COVER_CLEAR + 1e-9 and zi1 >= pz1 + cz + COVER_CLEAR - 1e-9, "cover rubs the cradle plate"
    c = SM.box(x0, x1, y0, y1, gz0, gz1)
    c = c.cut(SM.box(x0 - 1, out_x + COVER_CLEAR, y0 - 1, y1 + 1, zi0, zi1))            # inside: open top, bottom, end
    c = c.cut(SM.box(x0 - 1, out_x + COVER_CLEAR, top_y - COVER_CLEAR, ew_bot + COVER_CLEAR,
                     lz0 - COVER_CLEAR, lz1 + COVER_CLEAR))                            # the end wall's slots
    if WIRE_NOTCH:
        w, h = WIRE_NOTCH
        c = c.cut(SM.box(x0 - 1, x0 + w, y1 - h, y1 + 1, gz0 - 1, zi0 + 1))             # servo wire out
    return one(c.clean(), "servo cover")


def frame(plate_mid):
    """the strip's line (direction t in x-z, unit), its ends, the pivot and the rigid move of the old parts: the old
    plate's J2 end (its middle, plate_mid) onto the pivot, tilted TILT out about z, then turned YAW about the panel's
    normal (y) so the servo runs along Mike's edge"""
    (x1, z1), (x2, z2) = HOLES
    L = math.hypot(x2 - x1, z2 - z1)
    t = ((x2 - x1) / L, (z2 - z1) / L)
    a = (x1 - BASE_END * t[0], z1 - BASE_END * t[1])           # J3 end of the strip
    b = (x2 + BASE_END * t[0], z2 + BASE_END * t[1])           # J2 end
    pivot = V(b[0] - PIVOT_IN * t[0], SIDE_Y - BASE_T, b[1] - PIVOT_IN * t[1])   # on the strip's centre line
    shift = V(pivot.x - SM.PLATE_X[1], pivot.y - SM.WALL_Y, pivot.z - plate_mid)

    def yaw(s):
        return s.rotate(pivot, pivot + V(0, 1, 0), -YAW)                 # the J3 end down, along the edge

    def move(s):
        return yaw(s.translate(shift).rotate(pivot, pivot + V(0, 0, 1), tilt()))
    return t, a, b, pivot, shift, move, yaw


def cradle():
    """servo_mount.py's bracket, cleaned (its posts overlapped the servo clearance by 0.4, leaving 0.7 mm fins, and the
    clearance cut 0.2 into the plate): plate, posts under the tabs, the end wall with the Bowden fitting - in
    servo_mount's (link A) frame. The old through-wall bolts are left out (this one is glued on by its bosses).
    Returns the cradle, the cable's height, the plate's z span and the box's (lid's) outline."""
    yb = SM.WALL_Y - SM.PLATE_T
    x0, x1 = SM.SERVO_X0 + SLIDE, SM.SERVO_X0 + SM.SERVO_L + SLIDE         # the servo, slid toward J2
    zs = SM.Z_C + SERVO_DZ                                                 # the servo's centre, moved over
    z0, z1 = zs - SM.SERVO_W / 2, zs + SM.SERVO_W / 2
    cl = 0.3
    fx0 = (x0 + x1) / 2 - SM.FLANGE_L / 2
    pz0, pz1 = plate_z()                                                   # follows the servo over, carries the hex
    hh = SM.SERVO_H + SM.TURRET_H + SM.SPLINE_H - 1.5 + SM.HORN_T / 2
    yc = yb - hh                                                            # cable height (horn plane)
    horn_top = yc - SM.HORN_T / 2
    y_top = min(horn_top - HORN_SCREW - LID_CLEAR - LID_T,                 # the lid's outside top: over the horn,
                yc - FIT_HEX_AC / 2 - 0.5,                                 # the wall's face carries the fitting's hex,
                yc - FIT_BOSS_D / 2 - LID_CLEAR - LID_T)                   # and the lid clears the boss
    xw = SM.PLATE_X[0] + WALL_T                                            # the end wall's inner face
    xb = SM.PLATE_X[0] + FIT_WALL                                          # the fitting boss's end
    assert fx0 > xb + 2.0, (fx0, xb)                                       # the servo's tab clears the boss
    plate = SM.box(SM.PLATE_X[0], SM.PLATE_X[1], yb, SM.WALL_Y, pz0, pz1)
    plate = plate.fuse(SM.box(SM.PLATE_X[0], xw + 0.01, y_top, SM.WALL_Y, pz0 - LID_T, pz1 + LID_T))   # the end wall
    plate = plate.fuse(cq.Solid.makeCylinder(FIT_BOSS_D / 2, xb - xw + 0.01, V(xw, yc, SM.Z_C), V(1, 0, 0)))
    web = (cq.Workplane("XY", origin=(0, 0, SM.Z_C - WEB_T / 2))
           .polyline([(xw - 0.01, yc), (xb - 0.5, yc), (xb - 0.5, yc + 4.0), (xw + WEB_RUN, yb + 0.01),
                     (xw - 0.01, yb + 0.01)]).close()
           .extrude(WEB_T).val())                                          # from the boss down to the plate, 4 under the cable
    plate = plate.fuse(web)
    for pa, pb in ((fx0, x0 - cl), (x1 + cl, fx0 + SM.FLANGE_L)):          # posts under the tabs, clear of the case
        plate = plate.fuse(SM.box(pa, pb, yb - SM.FLANGE_Z + 0.01, yb + 0.01, z0 - 1, z1 + 1))
    cuts = []
    for hx in (-SM.FLANGE_HOLES_X / 2, SM.FLANGE_HOLES_X / 2):              # tab screws (M3 self-tapping)
        for hz in (-SM.FLANGE_HOLES_W / 2, SM.FLANGE_HOLES_W / 2):
            cuts.append(SM.ycyl(2.5, yb - SM.FLANGE_Z + 12, yb - SM.FLANGE_Z - 0.5, (x0 + x1) / 2 + hx, zs + hz))
    xo = SM.PLATE_X[0]                                                      # the wall's outside face
    for d, depth in ((FIT_TAP_D, FIT_THREAD), (SM.TUBE_D, FIT_WALL - 1.5), (CABLE_D, FIT_WALL + 1.0)):
        cuts.append(cq.Solid.makeCylinder(d / 2, depth + 1.0, V(xo - 1.0, yc, SM.Z_C), V(1, 0, 0)))
    for cu in cuts:
        plate = plate.cut(cu)
    return one(plate.clean(), "cradle"), yc, (pz0, pz1), (xw, y_top)


def fitting(yc):
    """(body, collet) of the push fitting on the end wall's outside face, on the cable line, pointing out"""
    xo = SM.PLATE_X[0]

    def pl(off):                                      # a plane across the fitting's axis, off mm out from the wall
        return cq.Plane(origin=(xo - off, yc, SM.Z_C), xDir=(0, 0, 1), normal=(-1, 0, 0))
    body = cq.Workplane(pl(0)).polygon(6, FIT_HEX_AC).extrude(FIT_HEX_T).val()
    body = body.fuse(cq.Workplane(pl(FIT_HEX_T - 0.01)).circle(FIT_NECK_D / 2).extrude(4.01).val())
    collet = cq.Workplane(pl(FIT_HEX_T + 4.0)).circle(FIT_COLLET_D / 2).extrude(FIT_OUT - FIT_HEX_T - 5.0).val()
    collet = collet.fuse(cq.Workplane(pl(FIT_OUT - 1.0)).circle(FIT_NECK_D / 2).extrude(1.0).val())   # the collar
    bore = cq.Workplane(pl(-1.0)).circle(FIT_BORE / 2).extrude(FIT_OUT + 2.0).val()
    return one(body.cut(bore).clean(), "fitting"), one(collet.cut(bore).clean(), "collet")


def parts():
    servo, horn, _old_bracket, tip = SM.parts()
    bracket, yc, (pz0, pz1), (xw, y_top) = cradle()
    assert abs(yc - tip[1]) < 1e-6, (yc, tip[1])
    sx = SM.SERVO_X0 + SM.SERVO_L - SM.SHAFT_FROM_END                       # the servo's shaft (x), in its old frame
    horn = horn.rotate(V(sx, 0, SM.Z_C), V(sx, 1, SM.Z_C), HORN_TURN).translate(V(SLIDE, 0, SERVO_DZ))
    servo = servo.translate(V(SLIDE, 0, SERVO_DZ))
    sx += SLIDE
    lid = (SM.PLATE_X[1] - xw, pz1 - pz0 + 2 * LID_T, SM.WALL_Y - y_top)
    if ON == "base":                    # straight onto the mount plate: no tilt, no yaw, 4 x M3 flat heads
        assert abs(y_top - lid_top()) < 1e-9 and (pz0, pz1) == plate_z()
        yb = SM.WALL_Y - SM.PLATE_T                                        # the plate's servo side
        k = (M3_CSK - M3_CLEAR) / 2 / 0.8693
        for bx, bz in BOLTS:
            bracket = bracket.cut(SM.ycyl(M3_CLEAR, SM.WALL_Y + 1, yb - 1, bx, bz))
            bracket = bracket.cut(cq.Solid.makeCone(M3_CLEAR / 2, M3_CSK / 2, k, V(bx, yb + k - 0.01, bz), V(0, -1, 0)))
        cy, cz = tab_shift()[1:]                                           # the wires' hole into the column
        bracket = bracket.cut(SM.ycyl(WIRE_HOLE_D, SM.WALL_Y + 1, yb - 1, WIRE_HOLE_Y - cy, BOX_Z - cz))
        bracket = one(bracket.clean(), "servo bracket")
        fit, collet = fitting(yc)
        exit_pt = V(*to_tab(SM.PLATE_X[0] - FIT_OUT, tip[1], SM.Z_C))   # the collet's top
        d = (0.0, -1.0, 0.0)                                                # straight up
        return (on_tab(servo), on_tab(horn), on_tab(bracket), exit_pt, d, sx - xw, lid, on_tab(fit), on_tab(collet))
    t, a, b, pivot, shift, move, yaw = frame((pz0 + pz1) / 2)
    n = (-t[1], t[0])
    w = BASE_W / 2
    poly = [(a[0] + n[0] * w, a[1] + n[1] * w), (b[0] + n[0] * w, b[1] + n[1] * w),
            (b[0] - n[0] * w, b[1] - n[1] * w), (a[0] - n[0] * w, a[1] - n[1] * w)]
    base = cq.Workplane("XZ", origin=(0, SIDE_Y, 0)).polyline(poly).close().extrude(BASE_T).val()   # y -30 .. -34
    for hx, hz in HOLES:
        base = base.fuse(cq.Solid.makeCylinder(BOSS_D / 2, BOSS_L, V(hx, SIDE_Y - 0.01, hz), V(0, 1, 0)))
    # two ribs from the strip (y -34) up to the tilted plate's inner face, inside the strip's width; built before the
    # turn, then turned with the plate (the strip is 0.9 deg off the edge: they stay on it)
    c, s = math.cos(math.radians(tilt())), math.sin(math.radians(tilt()))
    plate_len = SM.PLATE_X[1] - SM.PLATE_X[0]
    far = (pivot.x - plate_len * c, pivot.y - plate_len * s)       # the tilted plate's inner face, J3 end
    run = math.hypot(b[0] - a[0], b[1] - a[1]) - PIVOT_IN - 2.0    # along the strip from the pivot, 2 short of its end
    tri = [(pivot.x, pivot.y), (far[0], far[1]), (pivot.x - run, pivot.y)]
    for w0, w1 in ((RIB_W[0], RIB_W[1]), (-RIB_W[1], -RIB_W[0])) if tilt() > 0.5 else ():   # flat: lies on the strip
        rib = cq.Workplane("XY", origin=(0, 0, pivot.z + w0)).polyline(tri).close().extrude(w1 - w0).val()
        base = base.fuse(yaw(rib))
    mount = one(base.fuse(move(bracket)).clean(), "servo bracket")
    # the tube's start: where it leaves the fitting's collet, toward the joint, moved with the rest
    xo = SM.PLATE_X[0] - FIT_OUT
    exit_pt = move(cq.Vertex.makeVertex(xo, tip[1], SM.Z_C)).Center()
    ahead = move(cq.Vertex.makeVertex(xo - 1.0, tip[1], SM.Z_C)).Center() - exit_pt
    horn_clear = sx - xw                                                   # the hole (over the shaft's x) to the wall
    lid = (SM.PLATE_X[1] - xw, pz1 - pz0 + 2 * LID_T, SM.WALL_Y - y_top)
    fit, collet = fitting(yc)
    o = placed(cq.Vertex.makeVertex(0, 0, 0)).Center()
    exit_pt = placed(cq.Vertex.makeVertex(*exit_pt.toTuple())).Center()
    d = placed(cq.Vertex.makeVertex(ahead.x, ahead.y, ahead.z)).Center() - o      # a direction: turned, not moved
    mount = placed(mount)
    return (placed(move(servo)), placed(move(horn)), mount, exit_pt, (d.x, d.y, d.z), horn_clear, lid,
            placed(move(fit)), placed(move(collet)))


def main():
    global ON
    if "--on" in sys.argv:
        ON = sys.argv[sys.argv.index("--on") + 1]
        assert ON in OUTS, ON
    servo, horn, mount, exit_pt, d, clear, lid, fit, collet = parts()
    for nm, sh in (("servo", servo), ("horn", horn), ("bracket", mount)):
        bb = sh.BoundingBox()
        print(f"{nm}: x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} z {bb.zmin:.1f}..{bb.zmax:.1f}"
              + (f"  {sh.Volume() / 1000:.1f} cm^3" if nm == "bracket" else ""))
    print(f"horn's cable hole to the end wall, at rest (square to the cable): {clear:.1f} mm")
    print(f"room for the lid (outside, from the end wall to the plate's J2 end): {lid[0]:.1f} long x {lid[1]:.1f} wide x "
          f"{lid[2]:.1f} tall from the plate's underside")
    print(f"tube start (the collet's top): p [{exit_pt.x:.2f}, {exit_pt.y:.2f}, {exit_pt.z:.2f}], "
          f"d [{d[0]:.4f}, {d[1]:.4f}, {d[2]:.4f}]  ({'joint J0 swivel' if ON == 'base' else 'joint J2 elbow'})")
    cover = None
    if ON == "base":
        cover = cover_on_tab()
        bb = cover.BoundingBox()
        print(f"cover: x {bb.xmin:.2f}..{bb.xmax:.2f} y {bb.ymin:.2f}..{bb.ymax:.2f} z {bb.zmin:.2f}..{bb.zmax:.2f}, "
              f"{cover.Volume() / 1000:.1f} cm^3")
        for nm, sh in (("bracket", mount), ("servo", servo), ("horn", horn), ("fitting", fit), ("collet", collet)):
            print(f"   cover to the {nm}: {cover.distance(sh):.2f} mm, overlap {cover.intersect(sh).Volume():.3f} mm^3")
        sx = SM.SERVO_X0 + SM.SERVO_L - SM.SHAFT_FROM_END + SLIDE
        _, ay, az = to_tab(sx, 0.0, SM.Z_C + SERVO_DZ)                       # the shaft: design +x through here
        sweep = min(cover.distance(horn.rotate(V(0, ay, az), V(1, ay, az), a)) for a in range(-90, 91, 15))
        print(f"   cover to the horn over its whole +-90 deg swing: {sweep:.2f} mm")
    a = cq.Assembly(name=f"servo_on_{ON}")
    if cover is not None:
        a.add(cover, name="servo cover", color=cq.Color(0.35, 0.55, 0.75))
    a.add(mount, name="servo bracket", color=cq.Color(0.55, 0.75, 0.45))
    a.add(servo, name="servo (MG996R size)", color=cq.Color(0.11, 0.11, 0.12))
    a.add(horn, name="servo horn", color=cq.Color(0.95, 0.95, 0.95))
    a.add(fit, name="PC4-M10 fitting", color=cq.Color(0.78, 0.77, 0.72))
    a.add(collet, name="fitting collet", color=cq.Color(0.17, 0.42, 0.84))
    out = OUTS[ON]
    if "--out" in sys.argv:                                     # a folder (to check before replacing the real one)
        out = os.path.join(sys.argv[sys.argv.index("--out") + 1], os.path.basename(out))
    write_step(a, out)
    print(out)


if __name__ == "__main__":
    main()

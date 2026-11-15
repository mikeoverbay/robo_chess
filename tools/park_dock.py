#!/usr/bin/env python3
"""
park_dock.py - the arm's parking dock, a tool-changer style fork the claw parks in (Mike 2026-11-08: "put a flange
around the claw up at the top and have this thing move in and park itself in that claw ... In that U-shape like a
tool changer ... that pocket that it goes into recessed a little bit so that ring around the top of the gripper can't
move around. And that'll lock that arm in place"; then: at power-up the arm assumes it is docked and moves out of it
from there before homing - "that should help unload all that mass of having the arm sticking out").

    python park_dock.py            (writes the STEPs, the scene models, scene ik.park, and the checks)

Parts (all PETG):
  gripper/gripper_hub_flanged.step   gripper.py's hub + a FLANGE at its top: rim D76 x 8 (hub z 42..50), a 2 mm flat
                                     land under it (r 36..38, the bearing face), a 45 deg cone down to the hub (prints
                                     bottom-down with no support). The three finger slots stay open to r 27 (the fingers
                                     still drop in from the top) and a 6 x 2 magnet sits radially in the rim, on the
                                     finger-0 side (towards the dock's closed end when parked).
  park_dock/park_dock_fork.step      the fork: 16 thick, a U channel 60 wide (the hub D50 and closed fingers D52 pass)
                                     open away from the yaw axis, the seat = a recess D77.2 x 3 deep the rim drops into
                                     (0.6 total play, 1 mm lead-in) with a flat floor for the land; a lug at the closed
                                     end holds the dock's Hall (DRV5032FB, slid in from the top) facing the rim magnet;
                                     its leads run down and along a groove under the fork into the post.
  park_dock/park_dock_post.step      tube D24 / D14 (the Hall's wires inside), into sockets in the fork and the foot,
                                     an M3 across each end.
  park_dock/park_dock_foot.step      on the wood: a bar along the base's left cover with a wall that takes the cover's
                                     two side flange screws (#4, now 5 mm longer), so the dock is located BY THE BASE;
                                     plus wood screws near the post.

Where (design frame): J0 130 - behind the black tray's end, outside every game move's swing (J0 -95.4..94.5) and clear
of the home pose's claw. Grip point 179.5 from the yaw axis, 20.5 above the board: there J4 sits level with J1, which
is what holds the parked arm. With the claw locked in the seat, links A-B-C still form a four-bar (J1 and J4 pinned)
with one free fold; with J4 level with J1 the IK's pose (J2 = J3) is that fold's balance point - gravity's push along it
~0.1 N.m - so the gearboxes' own friction holds it (scratchpad dock_fourbar/dock_rest studies, 2026-11-09; an elbow
bumper was ruled out: stopping J2 at -101 loses squares d8 and e8).
Park: claw closed, over the dock at a safe height, down outside the fork at reach PARK_OUT, in along the reach at
LIFT above the seat, down LIFT into it, motors off. Unpark (and the first move at power-up): the reverse, then the
swivel back to the game side before the homing cycle (homing J1..J4 with the swivel still at 130 would fold the arm
over the fork).
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace  # noqa: E402
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism  # noqa: E402
from OCP.gp import gp_Pnt, gp_Vec, gp_Trsf  # noqa: E402

import gripper as GR  # noqa: E402
import claw_adapter as CA  # noqa: E402
import studio  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402
from view_step import read_step  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "park_dock")
GRIP_OUT = os.path.join(ROOT, "gripper")
SCENE = studio.scene_path()

# ---------------------------------------------------------------- the parked pose
PSI, RHO, H = 130.0, 179.5, 20.5        # J0; grip point's reach from the yaw axis; its height above the board
BOARD_Y, WOOD_Y = 232.0, 257.0          # design y (down) of the board top and of the wood the base + board sit on
HUB_ABOVE_GRIP = 106.0                  # gripper.py: hub bottom 106 above the grip point (checked in Robot Studio)
LIFT = 11.0                             # rim out of the recess and the flange's cone above the fork, to slide
PARK_OUT = 262.0                        # reach to come down at, outside the fork (its edge 226.1 + fingers 26 + 10)
SAFE_ABOVE = 25.0                       # swinging over to the dock: the claw's lips this far above the fork's top

# ---------------------------------------------------------------- flange (hub frame: z 0 hub bottom, z 50 hub top)
FL_R, FL_Z0, FL_Z1 = 38.0, 42.0, 50.0   # rim D76, 8 tall (the 6.2 magnet pocket leaves 0.9 above and below)
LAND_R = 36.0                           # flat land r 36..38 at z 42: the face that sits on the seat
CONE_Z0 = FL_Z0 - (LAND_R - GR.HUB_OD / 2)   # 45 deg from r 36 at z 42 to the hub (r 25) at z 31
NOTCH_R = 27.0                          # finger slots kept open to r 27 (finger outer face r 26)
MAG_D, MAG_DEPTH, MAG_Z = 6.2, 2.4, 46.0
MAG = (6.0, 2.0)                        # the magnet itself, D x thick
# ---------------------------------------------------------------- fork (same hub frame, claw parked)
PLATE_TOP, PLATE_BOT = 45.0, 29.0       # rim sits 3 into the recess; 16 thick
RECESS_R, LEAD = 38.6, 1.0              # 0.6 radial play all round; 1 mm x 45 lead-in at the top edges
HOLE_R = 36.4                           # under the floor: the flange's cone (r 36 at the land) clears it by 0.4;
#                                         the land bears on r 36.4..38 (was 34: the cone dug 2 mm into the fork)
CHANNEL_HALF = 30.0                     # U channel 60 wide: hub D50, closed fingers D52, 4 mm each side
PLATE_R = 46.6                          # 8 mm round the recess
LUG_DEG, LUG_TOP = 20.0, FL_Z1          # the closed end's lug, +-20 deg, up to the rim top
POST_Y, POST_OD, POST_ID = 58.0, 24.0, 14.0   # post centre at hub-frame (0, -58): beside the fork, behind it
SOCKET_D, SOCKET_DEPTH = 24.4, 12.0
BOSS_R, BAR_HALF = 17.0, 15.0
HALL = (1.6, 4.1, 3.1)                  # DRV5032FB TO-92 (LPG) body: thick (radial) x wide x tall
HALL_POCKET = (1.9, 4.5, 3.5)
HALL_FACE_R = RECESS_R + 0.8            # its sensing face 0.8 behind the recess wall, 1.6 + play from the magnet
GROOVE_W, GROOVE_D = 3.0, 2.0
M3_CLEAR = 3.4
# ---------------------------------------------------------------- foot (design frame)
FOOT_T, FOOT_BOSS_H, FOOT_SOCKET = 8.0, 20.0, 16.0
COVER_FACE_Z = -34.6                    # the base's left cover, outside face (swivel Y 65.2: flange 62 + 0.2 + lip 3)
COVER_SCREWS_X = (40.0, -35.0)          # its two side flange screws (swivel_base.FLANGE_SCREWS X -40 / 35), design x
COVER_SCREW_UP = 4.0                    # 4 above the wood (half the flange)
TAB_T, TAB_H, TAB_GAP = 5.0, 16.0, 0.2
SCREW4_CLEAR, SCREW4_CSK = 3.2, 6.5
WOOD_CLEAR, WOOD_CSK = 4.5, 9.0         # #8 wood screws


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def sector(r0, r1, a0, a1, z0, z1):
    """annular sector between angles a0..a1 (deg, about +z from +x)"""
    ring = cyl_z(r1, z0, z1).cut(cyl_z(r0, z0 - 1, z1 + 1))
    n = 48
    pts = [(0.0, 0.0)] + [((r1 + 5) * math.cos(math.radians(a0 + (a1 - a0) * k / n)),
                           (r1 + 5) * math.sin(math.radians(a0 + (a1 - a0) * k / n))) for k in range(n + 1)]
    wedge = prism_xy(pts, z0 - 1, z1 + 1)
    return ring.intersect(wedge)


def prism_xy(pts, z0, z1):
    poly = BRepBuilderAPI_MakePolygon()
    for x, y in pts:
        poly.Add(gp_Pnt(x, y, z0))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return cq.Shape.cast(BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, z1 - z0)).Shape())


def prism_design_xz(pts, y0, y1):
    """a prism in the design frame: outline in (x, z), from design y0 to y1"""
    poly = BRepBuilderAPI_MakePolygon()
    for x, z in pts:
        poly.Add(gp_Pnt(x, y0, z))
    poly.Close()
    face = BRepBuilderAPI_MakeFace(poly.Wire()).Face()
    return cq.Shape.cast(BRepPrimAPI_MakePrism(face, gp_Vec(0, y1 - y0, 0)).Shape())


def one(shape):
    s = shape.Solids()
    assert len(s) == 1, f"{len(s)} solids"
    return s[0]


# ---------------------------------------------------------------- the hub frame at the parked pose -> design
def hub_to_design(lift=0.0, out=0.0):
    """hub frame (claw parked; lifted `lift`, moved `out` along the reach) -> design frame. Claw x = towards the yaw
    axis (finger 0, measured in Robot Studio), claw z = up (design -y), claw y = z x x."""
    c, s = math.cos(math.radians(PSI)), math.sin(math.radians(PSI))
    rho = RHO + out
    o = (-c * rho, BOARD_Y - H - HUB_ABOVE_GRIP - lift, 30.6 - s * rho)
    xa, ya, za = (c, 0.0, s), (-s, 0.0, c), (0.0, -1.0, 0.0)
    t = gp_Trsf()
    t.SetValues(xa[0], ya[0], za[0], o[0],
                xa[1], ya[1], za[1], o[1],
                xa[2], ya[2], za[2], o[2])
    return t


def placed(shape, t):
    return cq.Shape.cast(shape.wrapped.Moved(cq.Location(t).wrapped))


# ---------------------------------------------------------------- parts
def flange_body(r_in):
    rim = cyl_z(FL_R, FL_Z0, FL_Z1)
    cone = cq.Solid.makeCone(GR.HUB_OD / 2, LAND_R, FL_Z0 - CONE_Z0, V(0, 0, CONE_Z0), V(0, 0, 1))
    body = rim.fuse(cone).cut(cyl_z(r_in, CONE_Z0 - 1, FL_Z1 + 1))
    for k in range(3):                                   # the finger slots, open to r 27 through the flange
        body = body.cut(box(GR.finger_inner_r() - 1.0, NOTCH_R, -GR.SLOT_W / 2, GR.SLOT_W / 2, CONE_Z0 - 1, FL_Z1 + 1)
                        .rotate(V(0, 0, 0), V(0, 0, 1), 120.0 * k))
    pocket = cq.Solid.makeCylinder(MAG_D / 2, MAG_DEPTH + 1, V(FL_R + 1, 0, MAG_Z), V(-1, 0, 0))
    return body.cut(pocket)


def hub_flanged():
    return one(GR.hub().fuse(flange_body(GR.HUB_OD / 2 - 0.1)).clean())


def magnet():
    return cq.Solid.makeCylinder(MAG[0] / 2, MAG[1], V(FL_R - 0.2, 0, MAG_Z), V(-1, 0, 0))


def fork():
    body = cyl_z(PLATE_R, PLATE_BOT, PLATE_TOP)
    body = body.fuse(box(-BAR_HALF, BAR_HALF, -POST_Y, 0, PLATE_BOT, PLATE_TOP))
    body = body.fuse(cyl_z(BOSS_R, PLATE_BOT, PLATE_TOP, 0, -POST_Y))
    body = body.fuse(sector(RECESS_R - 0.5, PLATE_R, -LUG_DEG, LUG_DEG, PLATE_TOP - 0.01, LUG_TOP))
    cuts = [cyl_z(RECESS_R, FL_Z0, LUG_TOP + 1),                                   # the recess (through the lug)
            cyl_z(HOLE_R, PLATE_BOT - 1, FL_Z0 + 0.01),                             # under the floor
            box(-PLATE_R - 30, 0, -CHANNEL_HALF, CHANNEL_HALF, PLATE_BOT - 1, LUG_TOP + 1),   # the U
            cyl_z(SOCKET_D / 2, PLATE_BOT - 1, PLATE_BOT + SOCKET_DEPTH, 0, -POST_Y),   # the post's socket
            cq.Solid.makeCylinder(M3_CLEAR / 2, 2 * BOSS_R + 2, V(-BOSS_R - 1, -POST_Y, PLATE_BOT + SOCKET_DEPTH / 2),
                                  V(1, 0, 0))]
    for z_top in (PLATE_TOP, LUG_TOP):                                             # lead-ins
        cuts.append(cq.Solid.makeCone(RECESS_R, RECESS_R + LEAD + 0.01, LEAD + 0.01, V(0, 0, z_top - LEAD), V(0, 0, 1)))
    hp = HALL_POCKET                                                               # the Hall: in from the top
    cuts.append(box(HALL_FACE_R, HALL_FACE_R + hp[0], -hp[1] / 2, hp[1] / 2, MAG_Z - hp[2] / 2, LUG_TOP + 1))
    cuts.append(box(HALL_FACE_R, HALL_FACE_R + 1.6, -1.5, 1.5, PLATE_BOT - 1, MAG_Z))   # its leads, down
    gr = HALL_FACE_R + 0.8                                                          # and under the fork to the post
    cuts.append(sector(gr - GROOVE_W / 2, gr + GROOVE_W / 2, -90.0, 0.0, PLATE_BOT - 1, PLATE_BOT + GROOVE_D))
    cuts.append(box(-GROOVE_W / 2, GROOVE_W / 2, -POST_Y, -gr + GROOVE_W / 2, PLATE_BOT - 1, PLATE_BOT + GROOVE_D))
    for c in cuts:
        body = body.cut(c)
    return one(body.clean())


def hall():
    h = HALL
    return box(HALL_FACE_R + 0.15, HALL_FACE_R + 0.15 + h[0], -h[1] / 2, h[1] / 2, MAG_Z - h[2] / 2, MAG_Z + h[2] / 2)


def post_design(t):
    """the post in the design frame: from the foot socket's floor up into the fork's socket (0.5 short)"""
    top = placed(cq.Vertex.makeVertex(0, -POST_Y, PLATE_BOT + SOCKET_DEPTH - 0.5), t).Center()
    y_bot = WOOD_Y - FOOT_BOSS_H + FOOT_SOCKET           # the foot socket's floor (design y, down)
    tube = cq.Solid.makeCylinder(POST_OD / 2, y_bot - top.y, V(top.x, top.y, top.z), V(0, 1, 0))
    tube = tube.cut(cq.Solid.makeCylinder(POST_ID / 2, y_bot - top.y + 2, V(top.x, top.y - 1, top.z), V(0, 1, 0)))
    for y in (top.y + SOCKET_DEPTH / 2 - 0.5, WOOD_Y - FOOT_BOSS_H + FOOT_SOCKET / 2):   # M3 pilots at both ends
        tube = tube.cut(cq.Solid.makeCylinder(1.25, POST_OD + 2, V(top.x - POST_OD / 2 - 1, y, top.z), V(1, 0, 0))
                        .rotate(V(top.x, y, top.z), V(top.x, y + 1, top.z), -PSI))   # along hub x, as the fork's
    return one(tube), (top.x, top.z)


def foot_design(pxz):
    px, pz = pxz
    zf = COVER_FACE_Z - TAB_GAP
    x0, x1 = min(COVER_SCREWS_X) - 12.0, max(COVER_SCREWS_X) + 12.0
    y_top = WOOD_Y - FOOT_T
    body = prism_design_xz([(x0, zf), (x1, zf), (x1, zf - 22.0), (x0, zf - 22.0)], y_top, WOOD_Y)   # along the cover
    ang = math.atan2(pz - (zf - 11.0), px - x1)                                  # bar from it to the post
    nx, nz = -math.sin(ang) * 13.0, math.cos(ang) * 13.0
    body = body.fuse(prism_design_xz([(x1 - 25.0, zf - 11.0 + nz), (px + nx, pz + nz), (px - nx, pz - nz),
                                      (x1 - 25.0, zf - 11.0 - nz)], y_top, WOOD_Y))
    body = body.fuse(cq.Solid.makeCylinder(16.0, FOOT_BOSS_H, V(px, WOOD_Y - FOOT_BOSS_H, pz), V(0, 1, 0)))
    body = body.fuse(prism_design_xz([(x0, zf), (x1, zf), (x1, zf - TAB_T), (x0, zf - TAB_T)], WOOD_Y - TAB_H, WOOD_Y))
    cuts = [cq.Solid.makeCylinder(SOCKET_D / 2, FOOT_SOCKET + 0.01, V(px, WOOD_Y - FOOT_BOSS_H - 0.01, pz), V(0, 1, 0)),
            cq.Solid.makeCylinder(M3_CLEAR / 2, 40.0, V(px, WOOD_Y - FOOT_BOSS_H + FOOT_SOCKET / 2, pz), V(1, 0, 0))
            .translate(V(-20.0, 0, 0)).rotate(V(px, 0, pz), V(px, 1, pz), -PSI),
            box(px - 2.5, px + 2.5, WOOD_Y - FOOT_BOSS_H + FOOT_SOCKET - 0.01, WOOD_Y - 3.0 + 0.01, pz - 2.5, pz + 2.5),
            box(px - 2.5, px + 40.0, WOOD_Y - 3.0, WOOD_Y + 1, pz - 2.5, pz + 2.5)]   # the wires: down, out the back
    for x in COVER_SCREWS_X:                                                     # the cover's two side screws
        cuts.append(cq.Solid.makeCylinder(SCREW4_CLEAR / 2, TAB_T + 2, V(x, WOOD_Y - COVER_SCREW_UP, zf + 1), V(0, 0, -1)))
        cuts.append(cq.Solid.makeCone(SCREW4_CLEAR / 2, SCREW4_CSK / 2, (SCREW4_CSK - SCREW4_CLEAR) / 2 + 0.01,
                                      V(x, WOOD_Y - COVER_SCREW_UP, zf - TAB_T + (SCREW4_CSK - SCREW4_CLEAR) / 2),
                                      V(0, 0, -1)))
    d = math.hypot(px - x1, pz - (zf - 11.0))
    wood = [(x0 + 22.0, zf - 13.0)] + [(px - (px - x1) * f, pz - (pz - (zf - 11.0)) * f) for f in (24.0 / d, 0.62)]
    for x, z in wood:                                                            # #8 wood screws, countersunk
        cuts.append(cq.Solid.makeCylinder(WOOD_CLEAR / 2, FOOT_T + 2, V(x, y_top - 1, z), V(0, 1, 0)))
        cuts.append(cq.Solid.makeCone(WOOD_CSK / 2, WOOD_CLEAR / 2, (WOOD_CSK - WOOD_CLEAR) / 2, V(x, y_top - 0.01, z),
                                      V(0, 1, 0)))
    cuts.append(prism_design_xz([(x0 - 50, zf), (px + 50, zf), (px + 50, zf + 80), (x0 - 50, zf + 80)],
                                WOOD_Y - 60, WOOD_Y + 1))   # nothing past the cover's face (the bar's corner poked 1.4 in)
    for c in cuts:
        body = body.cut(c)
    return one(body.clean()), wood


# ---------------------------------------------------------------- checks
def claw_parts_hub_frame():
    """the claw's own parts (closed) in the hub frame, from gripper_assembly_closed.step (the file claw_adapter reads)
    plus the adapter, carried from the drive frame by the inverse of claw_adapter.claw_to_wrist"""
    parts = [(n, s) for n, s, _ in read_step(CA.CLAW) if n != "piece_stand_in"]
    inv = CA.claw_to_wrist(True).Inverted()
    parts.append(("claw adapter", placed(CA.adapter(True), inv)))
    return parts


def main():
    os.makedirs(OUT, exist_ok=True)
    hub = hub_flanged()
    fl_only = one(flange_body(GR.HUB_OD / 2).clean())
    mag = magnet()
    fk, hl = fork(), hall()
    print(f"flanged hub {hub.Volume() / 1000:.1f} cm^3 (plain hub {GR.hub().Volume() / 1000:.1f}); fork {fk.Volume() / 1000:.1f} cm^3")

    # the flange against the rest of the claw (hub frame; the hub itself is replaced by the flanged one)
    claw = [(n, s) for n, s in claw_parts_hub_frame() if n != "hub"]
    for n, s in claw:
        v = s.intersect(hub).Volume()
        print(f"   flanged hub vs {n}: {'OVERLAP %.3f mm^3' % v if v > 1e-3 else 'clear, %.2f mm' % s.distance(fl_only)}")
    # parked: everything of the claw against the fork; the land must sit on the floor and nothing else touch
    every = claw + [("flanged hub", hub), ("magnet", mag)]
    print("parked:")
    for n, s in every:
        v = s.intersect(fk).Volume()
        print(f"   {n:16s} {'OVERLAP %.3f mm^3' % v if v > 1e-3 else 'gap %.2f' % s.distance(fk)}")
    print(f"   magnet face to the Hall's face {HALL_FACE_R + 0.15 - FL_R:.2f} mm (+ up to 0.6 play)")
    # the moves: down into the seat from LIFT, and in along the reach at LIFT from outside the fork
    worst = {}
    for kind, steps in (("drop", [(LIFT * k / 11, 0.0) for k in range(12)]),
                        ("slide", [(LIFT, -(PLATE_R + 34.0) * k / 20) for k in range(21)])):
        for lift, dx in steps:
            for n, s in every:
                m = s.translate(V(dx, 0, lift))
                g = m.distance(fk)
                if lift < 1e-6 and n == "flanged hub":
                    continue                                     # sits on the floor at the end: that's the point
                if kind not in worst or g < worst[kind][0]:
                    worst[kind] = (g, n, lift, dx)
    for k, (g, n, lift, dx) in worst.items():
        print(f"   {k}: closest {g:.2f} mm ({n} at lift {lift:.1f}, {-dx:.1f} out)")

    # into the design frame
    t = hub_to_design()
    post, pxz = post_design(t)
    foot, wood = foot_design(pxz)
    fk_d, hl_d = placed(fk, t), placed(hl, t)
    for a, b, an, bn in ((post, fk_d, "post", "fork"), (post, foot, "post", "foot")):
        v = a.intersect(b).Volume()
        print(f"   {an} in the {bn}: {'OVERLAP %.2f' % v if v > 1e-3 else 'fits, %.2f' % a.distance(b)}")
    bb = fk_d.BoundingBox()
    print(f"fork, design: x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} (top {BOARD_Y - bb.ymin:.1f} "
          f"above the board) z {bb.zmin:.1f}..{bb.zmax:.1f}; post at ({pxz[0]:.1f}, {pxz[1]:.1f}), "
          f"{math.hypot(pxz[0], pxz[1] - 30.6):.1f} from the yaw axis; wood screws at "
          + ", ".join(f"({x:.0f}, {z:.0f})" for x, z in wood))

    col = lambda h: cq.Color(*(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)))  # noqa: E731
    write_flat([("gripper hub (flanged)", hub, col("#d8d8dc")), ("dock magnet 6x2", mag, col("#c9ccd2"))],
               os.path.join(GRIP_OUT, "gripper_hub_flanged.step"))
    dock = [("park dock fork", fk_d, col("#e0a030")), ("park dock post", post, col("#b07a20")),
            ("park dock foot", foot, col("#8a6018")), ("park dock Hall (DRV5032FB)", hl_d, col("#202024"))]
    write_flat(dock, os.path.join(OUT, "park_dock.step"))
    for n, s, c in dock[:3]:
        write_flat([(n, s, c)], os.path.join(OUT, n.replace(" ", "_") + ".step"))
    # the flange (and its magnet) ride with J4 in Robot Studio: the claw's drive frame, J4 at design x -600
    tw = CA.claw_to_wrist(True)
    write_flat([("claw hub flange", placed(fl_only, tw), col("#d8d8dc")), ("claw flange magnet 6x2", placed(mag, tw),
                col("#c9ccd2"))], os.path.join(OUT, "claw_flange_on_wrist.step"))
    scene(t)


def park_pose():
    """the parked joint angles: the viewer's own IK at the grip point (J2 = J3 there; J4 level with J1)"""
    sc = json.load(open(SCENE, encoding="utf-8"))
    ik = sc["ik"]
    wx, wz = RHO - ik["tool"][0], -(BOARD_Y - H) - ik["tool"][1]       # the wrist (J4) from J1, as ikSolve
    d = math.hypot(wx, wz)
    q = -math.acos((d / ik["link"] - 1) / 2)
    q1 = math.atan2(wz, wx) - q
    a = {ik["yaw"]: PSI, ik["chain"][0]: math.degrees(q1), ik["chain"][1]: math.degrees(q), ik["chain"][2]: math.degrees(q)}
    a[ik["wrist"]] = ik["wrist_sum"] - (a[ik["chain"][0]] + 2 * a[ik["chain"][1]])
    return {k: round(v, 3) for k, v in a.items()}, wz


STARTUP = {
    "why": "2026-11-09 Mike: \"check if it's sitting in the dock before we try to home it out\" - the dock's Hall "
           "(DRV5032FB in the fork's lug) sees the 6x2 magnet in the claw's ring only when the claw is seated.",
    "power_up": [
        "read the dock Hall (several reads, as homing does: 5 Hz sensor)",
        "MADE = parked: take 'pose' as the joint angles and run the plain home cycle (J1..J4, then J0) - no special "
        "moves (Mike 2026-11-09: 'just running a regular home cycle when the arm tilts back, it's going to clear the "
        "dock'). Measured 2026-11-10 (dock_homing_check.py, J1 at 0.1 deg): nothing touches the station; closest "
        "0.24 mm at J1 +0.6 deg as the claw flange rises past the pocket wall, 1.0 mm by +1 deg, 6 mm by +4 deg; "
        "the J2, J3, J4, J0 stages and the move to home stay >= 20 mm off",
        "CLEAR = not in the dock: the angles are unknown, so no park moves and no blind homing; motors off, say so "
        "on the screen, wait for the player (push it into the dock by hand, or choose home-from-here) - to settle "
        "with Mike"],
    "parking": "after over -> outside -> lifted -> seat, the dock Hall must read MADE before the arm is called "
               "parked (the parked flag is set) and the motors let go; CLEAR there = a missed seat: stop, report"}


def scene(t):
    sc = json.load(open(SCENE, encoding="utf-8"))
    names = {"park dock": os.path.join(OUT, "park_dock.step"),
             "claw flange (on wrist)": os.path.join(OUT, "claw_flange_on_wrist.step")}
    sc["models"] = [m for m in sc["models"] if m.get("name") not in names]
    sc["models"].append({"path": names["park dock"], "name": "park dock", "by": "park_dock.py"})
    sc["models"].append({"path": names["claw flange (on wrist)"], "offset": [-600, 0, 0], "name": "claw flange (on wrist)",
                         "by": "park_dock.py"})
    for j in sc["joints"]:
        if j["name"] == "J4 wrist" and "claw flange (on wrist)" not in j["models"]:
            j["models"].append("claw flange (on wrist)")
    pose, wz = park_pose()
    c, s = math.cos(math.radians(PSI)), math.sin(math.radians(PSI))
    at = lambda rho, up: [round(-c * rho, 2), round(BOARD_Y - up, 2), round(30.6 - s * rho, 2)]  # noqa: E731
    fork_top = H + HUB_ABOVE_GRIP + LUG_TOP                        # above the board
    sc["ik"]["park"] = {
        "why": "2026-11-09 Mike: tool-changer dock (park_dock.py). Grip points (design) for the moves; 'pose' = the "
               "parked joint angles (the IK's own there: J4 level with J1, J2 = J3), which the arm assumes at power-up.",
        "pose": pose, "seat": at(RHO, H), "lifted": at(RHO, H + LIFT), "outside": at(PARK_OUT, H + LIFT),
        "over": at(PARK_OUT, fork_top + SAFE_ABOVE), "lift": LIFT, "j4_above_j1": round(wz, 2),
        "keep_out": {"j0": [112.0, 160.0], "below_up": round(fork_top + SAFE_ABOVE, 1),
                     "note": "the fork, post and foot: no claw below 'below_up' (mm above the board) with J0 in this "
                             "range except the park moves themselves"},
        "startup": STARTUP}
    if any(m.get("name") == "dock station v2" for m in sc["models"]):    # his one-piece station is shown instead
        sc["models"] = [m for m in sc["models"] if m.get("name") != "park dock"]
    with open(SCENE, "w", encoding="utf-8") as f:
        json.dump(sc, f, indent=1)
    print(f"scene: park pose {pose} (J4 {wz:+.2f} vs J1); seat {sc['ik']['park']['seat']}")


def export_assemblies():
    """For Mike to work on in Shapr3D (2026-11-09: "export that assembly ... for the robot park. I want to do some
    work on it myself" + "also export the claw. And the ring assembly"). Flat named bodies, no wrappers:
      park_dock/park_assembly.step  dock + the whole claw parked in it + the swivel base it bolts to, in Robot
                                    Studio's frame (z up; design (x, y, z) -> (-x, -z, -y))
      park_dock/claw_with_ring.step the claw on its adapter in its own frame (hub bottom at z 0, z up), the ring
                                    (flange) a body of its own beside the plain hub so it can be edited alone"""
    col = lambda h: cq.Color(*(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)))  # noqa: E731
    cols = {"hub": "#d8d8dc", "cap": "#d8d8dc", "plunger": "#f0bf4c", "adapter": "#8cbf73", "fitting": "#c7c4b8",
            "collet": "#2b6bd6", "finger": "#e6e6ea"}
    claw = []
    for n, s, _ in read_step(CA.CLAW):
        if n == "piece_stand_in":
            continue
        claw.append((f"claw {n}", s, col(cols["finger" if n.startswith("finger") else n] if n in cols or
                                           n.startswith("finger") else "#e6e6ea")))
    claw.append(("claw ring (flange)", one(flange_body(GR.HUB_OD / 2).clean()), col("#b9bcc4")))
    claw.append(("claw ring magnet 6x2", magnet(), col("#7d828c")))
    claw.append(("claw adapter", placed(CA.adapter(True), CA.claw_to_wrist(True).Inverted()), col(cols["adapter"])))
    fit, collet = CA.fitting_m6()
    claw += [("claw PC4-M6 fitting", fit, col(cols["fitting"])), ("claw fitting collet", collet, col(cols["collet"]))]
    write_flat(claw, os.path.join(OUT, "claw_with_ring.step"))

    t = hub_to_design()
    post, pxz = post_design(t)
    foot, _ = foot_design(pxz)
    world = gp_Trsf()
    world.SetValues(-1, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0)        # design -> Robot Studio (z up)
    tw = world.Multiplied(t)
    parts = [(n, placed(s, tw), c) for n, s, c in claw]
    parts += [("park dock fork", placed(fork(), tw), col("#e0a030")), ("park dock Hall (DRV5032FB)", placed(hall(), tw),
              col("#202024")), ("park dock post", placed(post, world), col("#b07a20")),
              ("park dock foot", placed(foot, world), col("#8a6018"))]
    for n, s, _ in read_step(os.path.join(ROOT, "complete assembly", "swivel_fixed.step")):
        parts.append((f"base: {n}", placed(s, world), col("#9aa0a8")))
    write_flat(parts, os.path.join(OUT, "park_assembly.step"))
    for f in ("park_assembly.step", "claw_with_ring.step"):
        p = os.path.join(OUT, f)
        print(f"{p}: {os.path.getsize(p) / 1e6:.1f} MB")
    print("park_assembly bodies:", ", ".join(n for n, _, _ in parts))


if __name__ == "__main__":
    if "--export" in sys.argv:
        export_assemblies()
    else:
        main()

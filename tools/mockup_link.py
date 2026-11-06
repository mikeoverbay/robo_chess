#!/usr/bin/env python3
"""
mockup_link.py - MOCK-UP (2026-11-04, Mike): a lighter, single-sided arm section with a plus-sign beam, on gearboxes
whose two output 6808s are spread apart so the drive alone carries the tilting loads (the link's outer 6808, its cap
and top plate go away). The real arm models are NOT changed.

    python mockup_link.py

Drive mock-up (cycloid stand-in, drive frame: axis z, housing bottom z 0): cut between its two output 6808s (z 36) and
the top moved up SPREAD - the lid gets a spacer ring, the output face rises from 45.2 to 45.2 + SPREAD. Mike's NEMA 17
under it as before.
Link mock-up (arm frame: pivot on the joint at x 0, elbow at x -200, like link B):
  - cup: 70.5 disc on the output face, 3 x M3 on r 15 (8 counterbores), a wire hole; a neck out to the beam
  - beam ("the cage", Mike): one round tube, 2 mm wall, r 30 on the old webs' centre line, between two end walls
    (half round, half square to the neck / plate); no webs inside
  - mounting ring at the elbow end under the next drive: 6 countersunk M3 from below on r 32.4, the motor's opening
Out: complete assembly/mockups/mockup_link_plus.step, mockup_drive_spread.step; prints mass and stiffness compared
with the present box link.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

import cycloid as C  # noqa: E402
from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
STANDIN = os.path.join(ROOT, "cycloid", "cycloid_drive_standin.step")
BOX_LINK = os.path.join(ROOT, "complete assembly", "arm_2_part_chevron.step")
OUT_DIR = os.path.join(ROOT, "complete assembly", "mockups")

SPREAD = 14.0                    # spacer between the drive's two output 6808s
SPLIT = 36.2                     # between them (29.2..36.2 | 36.2..43.2)
Z_OUT = 45.2 + SPREAD            # output face
CUP_T = 8.0
WEB_H = 60.0                     # the cage tube's diameter, and the neck / plate width (the box link is 70.5)
RING_R = 44.0                    # mounting ring at the elbow (Mike, P52.F89: r 39 -> 44, to see if it bends less)
BEAM_X = (-162.0, -41.5)         # beam between the two drives (housings r 35.25 at x 0 and x -200); the hub end at
#                                  41.5 so its end wall clears the previous link's mounting ring (r 39) under the drive
PLA, FILL = 1.24e-3, 0.85        # g/mm^3 (Mike prints the links in Creality PLA), printed fill (thin walls, 3-4 perimeters)
END_WALL_T = 3.0                 # Mike: a wall across each end of the beam; the half away from its joint plate rounded
RING_T = 2.5                     # Mike: the cage tube's wall (it started as thin rings round the beam); 2 -> 3 -> 2.5,
#                                  always from the inside (the outside stays r 30)
# Mike: the cage is a true circle r 30 centred on the in-plane rib's centre line (z 31.6): full 180 deg rounds on the
# end walls, full-circle rings, the cross web trimmed to the circle (so it can be skinned with thin clear plastic)
# Mike (P52.F12): the in-plane rib's outer edges out to the circle, rounded r 30 like the rings, and the strut along each
# edge as thick as the rings: its inside on r 28 (the truss windows' outer edges follow that arc)
GUSSET_T = 2.0                   # Mike: gussets in the corners at both ends, flush with the outer sides (y +-30)
# leg along the neck / plate; the leg on the wall runs to the frame's centre line (the in-plane web's middle, z 31.6)
GUSSET_HUB = 18.0                # under the neck; turns past the drive housing (r 35.25): its tip at r 36.6
GUSSET_ELBOW = 12.0              # on the mounting plate, ends on its straight side (the ring starts 13.1 out)
GUSSETS_ACROSS = 6               # Mike: more gussets across each end (along P52.E177), evenly between the outer two
#                                  (8.3 mm pitch, 6.3 mm gaps); each follows its outer one's line (P52.F52 / F74) and
#                                  its foot is cut on an arc GEAR_CLEAR outside the gearbox body (P53.E2, r 35.25):
GEAR_CLEAR = 1.0                 # hub: the drive turns past them; elbow: the next gearbox (its motor hole is inside it)
HOUSING_R = 35.25                # the gearbox body (70.5)
EDGE_ROUND = 1.0                 # Mike: r 1 on the mounting ring's bottom outer edge and the cup's top outer edge
BORE_GUSSETS, BORE_GUSSET_LEG = 0, 10.0   # Mike: gussets round the INSIDE of the tube where it meets each end wall -
#                                  radial plates GUSSET_T thick, 10 down the wall face x 10 along the bore, every 45 deg
#                                  (starting 22.5 off the arm's plane). Tested: no gain (claw 1.57 either way); set to 0
#                                  so the side holes fit (8 puts them back)
SIDE_HOLES, SIDE_HOLE_D = 4, 16.0          # Mike (P52.F89, between edges E236 / E119; D25 -> 16): holes through the side panel at
#                                  y -30 and the tube wall into the bore, on the line midway between the panel's two
#                                  diagonal edges, evenly between the end walls (2 mm clear of each), each edge 1 mm
#                                  inside the bore; then (Mike) "all the way through to the backside": side to side.
#                                  So the inside is open for printing.
# Mike: in the corner of the tube and each end wall's square half - first 8 gussets (elbow P52.E9 / E135, hub
# P52.E2 / E6), then one solid 45 deg wedge the full width (P52.F78), then (P52.E9 -> E4, "good on both sides") the
# wedge's toe run all the way to the far end: a ruled "fat fillet" the full width (flush with the sides) from the end
# wall's sharp straight edge to the FAR end wall's round edge - underneath: from the elbow wall's bottom edge (z -8)
# to the hub wall's round bottom (its outer face, x -41.5); on top: from the hub wall's top edge (z 67.2) to the
# elbow wall's round top (x -162). The underside fairs from the square plate into the round tube. Bore stays clear.
PLATE_T = CUP_T                  # Mike: the elbow mounting plate as thick as the cup (P52.F137 -> under P56.F1), 4 -> 8;
#                                  the extra goes underneath, the gearbox seat stays at z 0 (its 6 M3 get 4 mm longer)
# Home-switch lobe (Mike 2026-11-05; first on the base at J1 - "a little lobe like we did with the screws before, half a
# capsule shape ... out underneath the center line of that radius of that joint ... a magnetic sensor straight down
# into the bottom of that radius ... a big enough hole to bring this in and down into the arm ... we'll take it through
# the side holes" - then "repeat that at every single arm joint"). At the elbow end, on the tube's top, right under the
# NEXT link's cup (r 35.25 about x -200, z 59.2..67.2: 2.75 past the elbow end wall's outer face, x -162): a half
# capsule along x, centred under the cup's mid line, its end flush with the end wall's face, solid HALL_LOBE_SOLID in at
# its tip then 45 deg down into the tube (as the screw nubs). A DRV5032FB breakout in a pocket in its end, chip facing the
# cup's rim; a slot from the pocket's inner side through the end wall into the bore takes its wires (and a 3-pin plug)
# out of a side hole. Every link has it (J2 on link A, J3 on B, J4 on C), and so does the base (cut from this link: J1).
HALL_FACE_X = BEAM_X[0]          # -162: the elbow end wall's outer face
# Mike (2026-11-05): "Do these have to be so big? ... the Hall Effects are tiny. I really don't like these sticking out
# past the face of the joint ... taper the thing back ... enough room for wires to get through. But where that Hall
# effect switch fits, it can be small." His part is the DRV5032FBLPGM: TO-92 (LPG), no board. So the lobe only holds
# the TO-92's body, flat (branded) face toward the cup's rim, centred on the magnet's height; its top stays under the
# joint's face (the cup top, z 67.2). The leads bend straight back off the body's bottom into a wire drop right behind
# the pocket, down through the tube wall into the bore (then out a side hole); the 45 deg taper covers the drop.
HALL_Z = Z_OUT + (CUP_T - EDGE_ROUND) / 2   # 62.7: the cup rim's straight face, where the magnets sit
HALL_LOBE_R, HALL_LOBE_IN = 3.8, 59.5   # half width = end radius (top z 66.5: 0.7 under the joint face); its flat
#                                  underside inside the tube wall (r 27.5..30 about z 31.6: 59.1..61.6 at y 0)
HALL_LOBE_SOLID = 4.0            # solid this far in from the face at its tip, then the 45 deg slope (the deeper pocket)
TO92 = (4.1, 3.2, 1.6)           # the DRV5032FB LPG body: across (y), tall (z), thick (x); element ~0.5 behind its face
HALL_POCKET = (4.5, 3.6, 2.8)    # across (y), tall (z), deep (x) - Mike: "a little bit deeper ... a millimeter at least"
# Mike: "Just have the hole go straight down. And have a little channel for the wires to go in ... they go back and
# then to a hole and then it goes straight down ... the channel's at least the same width as the Hall effect switch ...
# [the hole] at least 3/16 so I can get my wires through there easily": the leads drop off the TO-92's bottom into a
# channel down the end wall's face (on the base: across the column's top, toward its middle), then a hole straight in
# along the link (on the base: straight down the column) through the end wall into the bore.
# Mike: "Make the channel deep enough so that it gets past the leads of the Hall effect switch" and then, on its end
# face (base P3.F29): "just move it back ... Make it 5 millimeter": the channel runs 5 further (its end from z 52 to
# 47), the hole with it; the TO-92's ~15.5 leads (tips near z 45.6) can be trimmed to solder in it. Open on the face
# for the hot glue ("we'll ... hot glue the whole mess").
HALL_CHANNEL = (6.0, 2.0)        # wide (y: the TO-92 is 4.1; 0.5 round the hole), deep (x) - 1.0 of end wall left
HALL_WIRE_HOLE = (5.0, 50.0, 6.0)   # dia (>= 3/16" = 4.76), centre z (0.5 above the channel's end), length
# Home magnets (Mike 2026-11-05: "6mm magnet into that face above the Hall effect switch" - his P1.F72, link A's cup rim
# - "do this to all the remaining joints"): a 6 x 2 magnet radially in this link's cup rim where the Hall that watches
# the cup sits when the arm is in its home pose (scene homes J1 135, J2 -90, J3 -90). A cup point at angle a (this link's
# frame, about its pivot) shows at a + q in the parent's frame, so:
#   J1: the base's Hall is straight below J1 (design +y, angle 90)        -> link A's magnet at 90 - 135 = -45
#   J2, J3: the Hall on the previous link's elbow end is at angle 0 (+x) -> links B, C at 0 - (-90) = +90
# All three links are one print, so every link gets BOTH pockets; glue the magnet in -45 on link A and in +90 on links
# B and C only (a magnet in the other pocket would be a second home). 3 deep (Mike: "at least 2 millimeter deep. I
# would make them 3"), the magnet pressed to the bottom, 1.0 under the rim; centred on the rim's straight face
# (z 59.2..66.2 below the 1 mm round): 0.4 of wall above and below - the cup is only 8 thick.
CUP_MAGNETS = {"J1 (link A)": -45.0, "J2 / J3 (links B, C)": 90.0}
MAGNET_D, MAGNET_DEPTH, MAGNET_Z = 6.2, 3.0, (Z_OUT + Z_OUT + CUP_T - EDGE_ROUND) / 2   # z 62.7


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} pieces"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(d, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def spread_drive():
    d = {n: s for n, s, _ in read_step(STANDIN)}
    h = d["cycloid drive"]
    lower = h.intersect(box(-60, 60, -60, 60, -1, SPLIT))
    upper = h.intersect(box(-60, 60, -60, 60, SPLIT, 100)).translate(V(0, 0, SPREAD))
    spacer = cyl(70.5, SPLIT - 0.01, SPLIT + SPREAD + 0.01)
    for a in C.LID_SCREW_ANGLES:                                  # the lid screws just get longer
        x, y = C.polar(C.LID_SCREW_R, a)
        spacer = spacer.cut(cyl(3.4, SPLIT - 1, SPLIT + SPREAD + 1, x, y))
    tall = one(lower.fuse(spacer).fuse(upper).clean(), "spread drive")
    return tall, d["NEMA 17"], d["NEMA 17 end caps"]


def plus_link():
    zt = Z_OUT + CUP_T
    zc = (-4.0 + zt) / 2
    body = cyl(70.5, Z_OUT, zt)                                               # cup on the output
    body = body.fuse(box(BEAM_X[1], 0.0, -WEB_H / 2, WEB_H / 2, Z_OUT, zt))  # neck out to the beam, flush with its end (Mike: E67 to E109)
    R = WEB_H / 2                                   # the cage circle: r 30 about (y 0, z zc), along x

    def xcyl(r, xa, xb):
        return cq.Solid.makeCylinder(r, xb - xa, V(xa, 0, zc), V(1, 0, 0))

    # hub end: square above the centre line where the neck is, a full half circle below; elbow end: square below on
    # its plate, a full half circle above
    xa, xb = BEAM_X[1] - END_WALL_T, BEAM_X[1]
    body = body.fuse(xcyl(R, xa, xb).fuse(box(xa, xb, -R, R, zc, zt)))
    xa, xb = BEAM_X[0], BEAM_X[0] + END_WALL_T
    body = body.fuse(xcyl(R, xa, xb).fuse(box(xa, xb, -R, R, -PLATE_T, zc)))

    def gussets(x_wall, z_plate, along, sx, x_gear):
        """right triangles in the corner of the wall face (x_wall) and a plate face (z_plate): `along` x (sx), and
        on the wall to the frame's centre line; GUSSET_T thick. The outer two flush with the sides (y +-30), then
        GUSSETS_ACROSS evenly between them, their feet cut on an arc GEAR_CLEAR outside the gearbox at x_gear."""
        tri = [(x_wall, z_plate), (x_wall + sx * along, z_plate), (x_wall, zc)]
        y_out = R - GUSSET_T / 2
        pitch = 2 * y_out / (GUSSETS_ACROSS + 1)
        clear = cq.Solid.makeCylinder(HOUSING_R + GEAR_CLEAR, Z_OUT + 10, V(x_gear, 0, -5), V(0, 0, 1))
        out = []
        for k in range(GUSSETS_ACROSS + 2):
            yc = -y_out + k * pitch
            g = cq.Workplane("XZ", origin=(0, yc + GUSSET_T / 2, 0)).polyline(tri).close().extrude(GUSSET_T).val()
            if 0 < k < GUSSETS_ACROSS + 1:                 # the outer two already clear the gearbox
                g = g.cut(clear)
            out.append(g)
        return out
    for g in gussets(BEAM_X[1], Z_OUT, GUSSET_HUB, 1, 0.0) + gussets(BEAM_X[0], 0.0, GUSSET_ELBOW, -1, -200.0):
        body = body.fuse(g)
    # Mike: the cage is ONE continuous round tube, RING_T wall (r 27..30), end wall to end wall (into the walls: one
    # solid). It grew from rings, then sleeves from the walls to the rings, then four struts between them.
    bore = xcyl(R - RING_T, BEAM_X[0] - 1, BEAM_X[1] + 1)
    body = body.fuse(xcyl(R, BEAM_X[0], BEAM_X[1]).cut(bore))
    def fairing(x_flat, z_flat, x_arc, top):
        """filled from the centre line out to a ruled surface between the straight edge (x_flat, z_flat, y -R..R) and
        the half circle r R at x_arc on the same side (above the centre line if top); what lands in the tube merges"""
        from OCP.BRepFill import BRepFill
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Sewing, BRepBuilderAPI_MakeSolid
        from OCP.ShapeFix import ShapeFix_Solid
        from OCP.TopoDS import TopoDS
        sg = 1 if top else -1
        line = cq.Edge.makeLine(V(x_flat, -R, z_flat), V(x_flat, R, z_flat))
        arc = cq.Edge.makeThreePointArc(V(x_arc, -R, zc), V(x_arc, 0, zc + sg * R), V(x_arc, R, zc))

        def poly(pts):
            return cq.Face.makeFromWires(cq.Wire.makePolygon([V(*q) for q in pts], close=True))
        faces = [cq.Face(BRepFill.Face_s(line.wrapped, arc.wrapped)),
                 poly([(x_flat, -R, z_flat), (x_flat, R, z_flat), (x_flat, R, zc), (x_flat, -R, zc)]),
                 poly([(x_flat, -R, zc), (x_flat, R, zc), (x_arc, R, zc), (x_arc, -R, zc)]),
                 poly([(x_flat, R, z_flat), (x_arc, R, zc), (x_flat, R, zc)]),
                 poly([(x_flat, -R, z_flat), (x_flat, -R, zc), (x_arc, -R, zc)]),
                 cq.Face.makeFromWires(cq.Wire.assembleEdges([arc, cq.Edge.makeLine(V(x_arc, R, zc),
                                                                                     V(x_arc, -R, zc))]))]
        sew = BRepBuilderAPI_Sewing(1e-4)
        for f in faces:
            sew.Add(f.wrapped)
        sew.Perform()
        fix = ShapeFix_Solid(BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(sew.SewedShape())).Solid())
        fix.Perform()
        return one(cq.Solid(fix.Solid()), "fairing")
    for f in (fairing(BEAM_X[0] + END_WALL_T, -PLATE_T, BEAM_X[1], False),     # underneath: elbow plate -> hub
              fairing(BEAM_X[1] - END_WALL_T, zt, BEAM_X[0], True)):           # on top: hub neck -> elbow
        assert f.Volume() > 0
        body = body.fuse(f.cut(bore))
    rb, L = R - RING_T, BORE_GUSSET_LEG             # bore radius; each plate also runs 1 mm into the tube wall so its
    for xw, inward in ((BEAM_X[0] + END_WALL_T, 1), (BEAM_X[1] - END_WALL_T, -1)):   # edge never leaves a sliver
        tri = [(xw, rb + 1.0), (xw, rb - L), (xw + inward * L * (L + 1.0) / L, rb + 1.0)]
        plate = cq.Workplane("XY", origin=(0, 0, zc - GUSSET_T / 2)).polyline(tri).close().extrude(GUSSET_T).val()
        for k in range(BORE_GUSSETS):
            body = body.fuse(plate.rotate(V(0, 0, zc), V(1, 0, zc), 22.5 + k * 360.0 / BORE_GUSSETS))
    body = body.fuse(cyl(2 * RING_R, -PLATE_T, 0.0, -200.0))                  # mounting ring at the elbow
    body = body.fuse(box(-200.0, BEAM_X[0] + END_WALL_T, -WEB_H / 2, WEB_H / 2, -PLATE_T, 0.0))   # flush with the end wall (Mike)
    cuts = [cyl(20.0, Z_OUT - 1, zt + 1)]                                     # wires
    xa_, xb_ = BEAM_X[0] + END_WALL_T, BEAM_X[1] - END_WALL_T                 # the side panel's two diagonal edges:
    bottom = lambda x: -PLATE_T + (zc + PLATE_T) * (x - xa_) / (BEAM_X[1] - xa_)          # elbow bottom -> hub round
    top = lambda x: zc + (zt - zc) * (x - BEAM_X[0]) / (xb_ - BEAM_X[0])                  # elbow round -> hub top
    r_h = SIDE_HOLE_D / 2
    mid = lambda x: (bottom(x) + top(x)) / 2
    reach = (R - RING_T - 1.0) - r_h                  # a hole's edge stays 1 mm inside the bore (else it grooves the wall)
    slope = (mid(xb_) - mid(xa_)) / (xb_ - xa_)
    x0 = max(xa_ + r_h + 2, xa_ + (zc - reach - mid(xa_)) / slope)            # 2 mm clear of the walls
    x1 = min(xb_ - r_h - 2, xa_ + (zc + reach - mid(xa_)) / slope)
    for k in range(SIDE_HOLES):
        x = x0 + k * (x1 - x0) / max(SIDE_HOLES - 1, 1)
        cuts.append(cq.Solid.makeCylinder(r_h, 2 * R + 2, V(x, -R - 1, mid(x)), V(0, 1, 0)))   # side to side
    for a in C.OUT_SCREW_ANGLES:                                              # output screws
        x, y = C.polar(C.OUT_SCREW_R, a)
        cuts += [cyl(4.4, Z_OUT - 1, zt + 1, x, y), cyl(8.0, zt - 4.0, zt + 1, x, y)]
    cuts.append(box(-200 - 22, -200 + 22, -22, 22, -PLATE_T - 1, 1))          # the next drive's motor goes through
    k = (6.3 - 3.4) / 2 / 0.8693
    for a in C.LID_SCREW_ANGLES:                                              # 6 x M3 up into the next drive
        x, y = C.polar(C.LID_SCREW_R, a)
        cuts.append(cyl(3.4, -PLATE_T - 1, 1, x - 200, y))
        cuts.append(cq.Solid.makeCone(6.3 / 2, 3.4 / 2, k, V(x - 200, y, -PLATE_T - 0.01), V(0, 0, 1)))
    for cu in cuts:
        body = body.cut(cu)
    body = one(body.clean(), "plus link")
    # Mike (P52.E104, E216): r EDGE_ROUND on the mounting ring's bottom outer edge and the cup's top outer edge
    def circle_edge(r, z):
        return [e for e in body.Edges() if e.geomType() == "CIRCLE" and abs(e.radius() - r) < 1e-4
                and abs(e.Center().z - z) < 1e-3 and e.Length() > 50]
    edges = circle_edge(RING_R, -PLATE_T) + circle_edge(70.5 / 2, zt)
    assert len(edges) == 2, f"rounded edges: found {len(edges)}"
    body = one(body.fillet(EDGE_ROUND, edges), "plus link")
    body = one(body.fuse(hall_lobe()).clean(), "plus link + Hall lobe")
    for cu in hall_cuts() + [magnet_pocket(a) for a in CUP_MAGNETS.values()]:
        body = body.cut(cu)
    return one(body.clean(), "plus link")


def magnet_pocket(a, extra=1.0):
    """a home magnet's radial pocket in the cup rim at angle a (deg, link frame about the pivot)"""
    r = 70.5 / 2
    d = V(math.cos(math.radians(a)), math.sin(math.radians(a)), 0)
    return cq.Solid.makeCylinder(MAGNET_D / 2, MAGNET_DEPTH + extra, V(0, 0, MAGNET_Z) + d * (r + extra), d * -1)


def magnet_parts():
    """the magnets shown, one per pocket angle (in CUP_MAGNETS' order), and each one's outer-face centre"""
    r = 70.5 / 2 - (MAGNET_DEPTH - 2.0)                  # the magnet's outer face: 0.2 under the rim
    out = []
    for a in CUP_MAGNETS.values():
        d = V(math.cos(math.radians(a)), math.sin(math.radians(a)), 0)
        out.append((cq.Solid.makeCylinder(3.0, 2.0, V(0, 0, MAGNET_Z) + d * r, d * -1), V(0, 0, MAGNET_Z) + d * r))
    return out


def hall_lobe():
    """the home switch's half-capsule lobe at the elbow end (see HALL_* above), link frame"""
    r, zc, zi, x0 = HALL_LOBE_R, HALL_Z, HALL_LOBE_IN, HALL_FACE_X
    tip = zc + r
    c = x0 + HALL_LOBE_SOLID + tip                   # the slope: x + z <= c (45 deg)
    x1 = c - zi + 1
    end = cq.Solid.makeCylinder(r, x1 - x0, V(x0, 0, zc), V(1, 0, 0)).intersect(
        box(x0 - 1, x1 + 1, -r - 1, r + 1, zc, zc + r + 1))                  # the outer half only (not into the bore)
    u = box(x0, x1, -r, r, zi, zc).fuse(end)
    w = (cq.Workplane("XZ").polyline([(x0 - 1, zi - 2), (x0 - 1, tip + 2), (c - tip - 2, tip + 2), (c - zi + 2, zi - 2)])
         .close().extrude(2 * r + 2).val())
    wb = w.BoundingBox()
    w = w.translate(V(0, -r - 1 - wb.ymin, 0))                              # across y -r-1 .. r+1 whichever way it went
    return u.intersect(w)


def hall_cuts():
    """the TO-92's pocket, the wire channel down the end wall's face (on the base: across the column's top) and the hole
    straight in to the bore at its end, link frame"""
    (pw, ph, pd), (cw, cd), (hd, hz, hl), x0 = HALL_POCKET, HALL_CHANNEL, HALL_WIRE_HOLE, HALL_FACE_X
    bottom = HALL_Z - ph / 2                                  # the pocket's low face
    return [box(x0 - 1, x0 + pd, -pw / 2, pw / 2, bottom, HALL_Z + ph / 2),
            box(x0 - 1, x0 + cd, -cw / 2, cw / 2, hz - hd / 2 - 0.5, bottom),
            cq.Solid.makeCylinder(hd / 2, hl, V(x0 + cd, 0, hz), V(1, 0, 0))]


def hall_parts():
    """the TO-92 shown in its pocket (back on the pocket's floor) and its three leads down the channel and in the hole:
    (body, leads, the Hall element's point)"""
    (bw, bh, bt), x0 = TO92, HALL_FACE_X
    floor = x0 + HALL_POCKET[2]
    body = box(floor - bt, floor, -bw / 2, bw / 2, HALL_Z - bh / 2, HALL_Z + bh / 2)
    zb, (cw, cd), (hd, hz, hl) = HALL_Z - bh / 2, HALL_CHANNEL, HALL_WIRE_HOLE   # (leads run down to the hole)
    w, xl = 1.27 + 0.2, x0 + cd - 0.6                    # the three leads (1.27 pitch) as one bent ribbon, 0.2 off
    leads = box(xl - 0.2, xl + 0.2, -w, w, hz - 0.2, zb + 0.05).fuse(                  # the channel floor: down ...
        box(xl - 0.2, x0 + cd + hl - 1, -w, w, hz - 0.2, hz + 0.2)).clean()            # ... then in the hole
    return body, leads, (floor - bt + 0.5, 0.0, HALL_Z)


def section(shape, x):
    """second moments of the whole cross-section at x (mm^4), about its own centroid: in-plane bending (spread in y)
    and sideways bending (spread in z). All the pieces of the slice together (parallel-axis included)."""
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    f = shape.intersect(box(x - 0.5, x + 0.5, -100, 100, -100, 100))
    faces = [fc for fc in f.Faces() if abs(fc.normalAt().x) > 0.99 and abs(fc.Center().x - (x - 0.5)) < 0.01]
    g = GProp_GProps()
    BRepGProp.SurfaceProperties_s(cq.Compound.makeCompound(faces).wrapped, g)
    m = g.MatrixOfInertia()                    # about the combined centroid; for a face normal to x:
    return m.Value(3, 3), m.Value(2, 2)        # Izz = integral y^2 dA (in-plane), Iyy = integral z^2 dA (sideways)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    link = plus_link()
    drive, mot, caps = spread_drive()
    box_link = sum(s.Volume() for _, s, _ in read_step(BOX_LINK))
    print(f"plus link: {link.Volume() * PLA * FILL:.0f} g in PLA   (the box link with its top: {box_link * PLA * FILL:.0f} g; "
          f"its outer 6808 another 33 g)")
    for xs in (-60.0, -100.0, -140.0):
        iz_p, iy_p = section(link, xs)
        print(f"beam section at x {xs:.0f}: in-plane bending I = {iz_p:,.0f} mm^4, sideways I = {iy_p:,.0f} mm^4")
    box_body = [s for n, s, _ in read_step(BOX_LINK) if n == "body"][0]
    iz_b, iy_b = section(box_body, -100.0)
    print(f"box link body at its middle: in-plane I = {iz_b:,.0f} mm^4, sideways I = {iy_b:,.0f} mm^4 (lid not counted)")
    a = cq.Assembly(name="mockup_link_plus")
    a.add(link, name="mockup plus link", color=cq.Color(0.72, 0.74, 0.78))
    write_step(a, os.path.join(OUT_DIR, "mockup_link_plus.step"))
    pcb, chip, elem = hall_parts()
    for n, s in (("Hall TO-92", pcb), ("Hall leads", chip)):
        if s.intersect(link).Volume() > 0.01:
            print(f"   OVERLAP {n} x link: {s.intersect(link).Volume():.2f}")
    print(f"home switch lobe: Hall element at link ({elem[0]:.2f}, {elem[1]:.2f}, {elem[2]:.2f}), "
          f"{elem[0] - (-200 + 70.5 / 2):.2f} from the next cup's rim")
    h = cq.Assembly(name="hall_on_link")
    h.add(pcb, name="Hall DRV5032FB", color=cq.Color(0.08, 0.08, 0.09))
    h.add(chip, name="Hall leads", color=cq.Color(0.78, 0.79, 0.81))
    write_step(h, os.path.join(OUT_DIR, "hall_on_link.step"))
    m = cq.Assembly(name="home_magnets_link")
    for (name, a), (s, face) in zip(CUP_MAGNETS.items(), magnet_parts()):
        if s.intersect(link).Volume() > 0.01:
            print(f"   OVERLAP magnet {name} x link: {s.intersect(link).Volume():.2f}")
        print(f"home magnet {name}: cup rim at {a:+.0f} deg, face centre ({face.x:.2f}, {face.y:.2f}, {face.z:.2f})")
        m.add(s, name=f"home magnet {name}", color=cq.Color(0.82, 0.83, 0.86))
    write_step(m, os.path.join(OUT_DIR, "home_magnets_link.step"))
    b = cq.Assembly(name="mockup_drive_spread")
    b.add(drive, name="mockup drive (spread 6808s)", color=cq.Color(0.72, 0.74, 0.78))
    b.add(mot, name="NEMA 17", color=cq.Color(0.12, 0.12, 0.13))
    b.add(caps, name="NEMA 17 end caps", color=cq.Color(0.78, 0.79, 0.81))
    write_step(b, os.path.join(OUT_DIR, "mockup_drive_spread.step"))
    db = drive.BoundingBox()
    print(f"spread drive: output face z {Z_OUT:.1f} (was 45.2), housing {drive.Volume() / 1000:.0f} cm^3, top z {db.zmax:.1f}")
    for v, n in ((link.intersect(drive), "link x pivot drive"), (link.intersect(drive.translate(V(-200, 0, 0))), "link x elbow drive")):
        if v.Volume() > 0.05:
            print(f"   OVERLAP {n}: {v.Volume():.1f} mm^3")
    print(OUT_DIR)


if __name__ == "__main__":
    main()

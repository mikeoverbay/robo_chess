#!/usr/bin/env python3
"""
cell_edit.py - edits to the Professor's own square cell (Robo chessboard design, 2026-11-06).

    python cell_edit.py      chessboard/cell/mikes_cell.step  ->  chessboard/cell/mikes_cell_v3.step (with the look-only
                             wires, Hall, diode), bucket_v3.step and base_plate_v3.step (the printed parts only)

His model (his CAD, from square_cell.py's parts) is the master; his file is only read, never written.
  bucket      59.8 square x 24: 1.2 top wall (the lit rim), ledge at z 18.82 for the wood square, a hollow funnel shell
              carrying a 6 x 6 peg at the centre for the Hall, the light chamber under it, and a 2 x 2 ledge inside the
              walls at z 3.7..5.7 that the base plate bears up against
  base plate  55.2 square x 3.7, the bottom wiring (LED pocket and hole, rank trenches, column trench) - left as his
Earlier edits, already in his file: equal 2.2 walls ("Consistent"), plate 0.1 clear ("I don't want to press fit").
History: v3 ran the Hall's leads down through the peg to holes in the base plate - dropped: "no way to get in there and
wire this ... the cap's in the way" (the funnel shell blocks the peg's underside).

Now (his words 2026-11-06): the Hall folded onto the dropped peg, "the pins are facing P1 face 1" (the +x wall); "I want
troughs and holes in F1. And I want the troughs to be diagonal"; "color code the pins. Ground is black. Red is hot.
Signal is white." Everything is reachable: from above before the wood square goes in, and from below for the joints.
  - peg: its top drops by the Hall's thickness; the Hall lies on it bevelled face up, 0.05 under the wood. Pins at +x:
    seen from above, + (red) at -y, GND (black) in the middle, OUT (white) at +y. (Flipping it bevel-down would only
    swap + and OUT - GND is the middle pin either way - and with the bevel up + already sits on the ROW side.)
  - ledge troughs, open on top under the wood: the leads (and the 30 AWG kynar extensions soldered on from above)
    run out to the +x wall; every channel sized for 1.5 heat-shrink round the wire (his ask): 1.7 x 1.7 troughs and
    holes, 1.7 x 1.4 face troughs (+ the 0.2 gap to the next bucket), 1.7 x 1.7 landings under the wall
  - holes through F1 under the rim, one per wire
  - diagonal troughs down F1's outside, spreading each wire to its own place at the bottom: + to the ROW lane, GND to
    the GND lane, OUT beside the 5V lane; there they turn in under the wall through a notch
  - the column (signal) rail moved from x -19 to x 20.6, on the side the signal comes down (his ask); the white wire
    runs on into the plate to the 1N4001 in a recess between the column trench and the plate's edge, its stripe
    (cathode) lead over to the column wire - every joint on the bottom; his old column trench filled
  - heat-shrink pockets at the Hall rails' joints (+ on ROW, GND on GND, the diode on the column): wider than the
    trench, narrow trench either side, so the tubing is trapped (his ask); none on the LED's data / 5V
  - notches through the bottoms of the walls for the bare rank wires (both x walls) and the column wire (both y walls):
    his walls had none, so no wire could pass from one cell to the next
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

import view_step as VS  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "chessboard", "cell", "mikes_cell.step")
OUT = os.path.join(ROOT, "chessboard", "cell", "mikes_cell_v3.step")

# ---- Hall: DRV5032FBLPGM, TO-92 (TI SLVSDC7H)
HALL_W, HALL_L, HALL_T = 4.0, 3.15, 1.52     # across the leads, along them, thickness
HALL_LEAD_MIN, HALL_LEAD_T = 15.1, 0.43
FACE_GAP = 0.05                              # Hall face under the wood square
POST_HALF = 3.0                              # his 6 x 6 peg
PINS = {"+": -1.27, "GND": 0.0, "OUT": 1.27}  # y where each lead leaves the body (pins at +x, bevel up)
COLOUR = {"+": (0.85, 0.1, 0.1), "GND": (0.08, 0.08, 0.08), "OUT": (0.97, 0.97, 0.97)}   # his code: red hot, black, white

# ---- the cell (z 0 = the underside; his rank trenches along x, his column trench along y at x -19)
HALF = 29.9                                  # bucket's outside
INNER = 27.7                                 # walls' inside
LANES = {"GND": -2.54, "data": 0.0, "5V": 2.54, "ROW": -8.5}   # (the data-return lane is gone: the Mega drives each
#                                              rank's LEDs on its own line, his pick 2026-11-06)
LANE_W, LANE_D = 1.0, 1.0
OLD_COL_X = -19.0                            # where his plate's column trench was (filled)
COL_X, COL_W, COL_D = 20.6, 1.6, 2.4         # his ask: "move the signal rail down to where the signal is coming out of
#                                              the Hall effect" - the column trench now runs along y on F1's side, just
#                                              inside the + / GND joint pockets
DIODE_D, DIODE_L, LEAD_D = 2.7, 5.2, 0.8     # 1N4001 (DO-41)
DIODE_REC = (22.6, 25.6, 5.2, 19.4, 2.9)     # x0, x1, y0, y1, depth: between the column trench and the plate's edge
# heat-shrink over the Hall rails' joints (his ask: room for the tubing, but "I don't want the slot continuously wide
# because I want it to trap that shrink wrap tubing") - a wider pocket at each joint, the narrow trench either side of it
JP_X, JP_W, JP_D = (22.6, 27.0), 2.0, 1.7    # + -> ROW and GND joints: pocket x span, width, depth (0.6 of narrow
#                                              trench left at the plate's edge)
COL_JP = (22.5, 26.9, 2.6, 2.6)              # column joint: y span, width, depth (0.7 of narrow trench at the edge)
JT_D, JT_L, CJT_D = 1.5, 3.2, 2.0            # the tubing stand-ins: rank joints 1.5 OD, the column joint 2.0
DIODE_Y = (13.0, 18.2)                       # anode end (toward the white wire), stripe / cathode end. His ask: keep the
#                                              soldering heat off the diode - both joints sit 6+ mm out along its leads
ANODE_JOINT_Y = 6.8                          # the white wire meets the anode lead here (6.2 of lead from the body)
CATH_Y = 24.3                                # the cathode lead runs on to here, then across to the column wire
INS_D, WIRE_D = 1.2, 0.8                     # the insulated column wire; bare rank wires

# ---- the wire run: ledge troughs -> holes in F1 -> diagonal troughs down F1 -> notches under the wall
TUBE_D = 1.5                                 # his ask: room for heat-shrink round the wire (1.5 shrunk, over a joint too)
TR_Y = {"+": -3.0, "GND": 0.0, "OUT": 3.0}   # ledge troughs and holes (1.3 of plastic between them)
TR_W, TR_D = 1.7, 1.7                        # width; depth below the ledge (the wood square closes them)
SLOT_X0 = 23.0                               # from here to the wall the ledge is too thin (1.4 at the wall) for a 1.7
#                                              trough floor, so the trough goes clean through it - no knife-edge floor
TR_X0 = 14.0                                 # the troughs start inside the funnel (where the leads end)
FAN_L = 4.5                                  # a pocket that long where the wires fan out from 1.27 apart
FT_W, FT_D = 1.7, 1.4                        # diagonal troughs on F1: width, depth into the 2.2 wall (+ the 0.2 gap
#                                              to the next bucket = 1.6 for the 1.5 tube; 0.8 of wall behind)
LAND_W, LAND_H = 1.7, 1.7                    # where each lands at the bottom, under the +x wall
BOTTOM_Y = {"+": LANES["ROW"], "GND": LANES["GND"], "OUT": 6.0}   # where each lands at the bottom
NOTCH_H = 1.0
EXT_D = 0.55                                 # 30 AWG kynar


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} solids"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def path(pts, d):
    segs = []
    for a, b in zip(pts, pts[1:]):
        a, b = V(*a), V(*b)
        segs.append(cq.Solid.makeCylinder(d / 2, (b - a).Length, a, (b - a).normalized()))
        segs.append(cq.Solid.makeSphere(d / 2, b, angleDegrees1=-90, angleDegrees2=90))
    s = segs[0]
    for q in segs[1:]:
        s = s.fuse(q)
    return s.clean()


def face_trough(y0, z0, y1, z1):
    """a straight groove on the +x face from (y0, z0) down to (y1, z1): FT_W wide, FT_D deep"""
    dy, dz = y1 - y0, z1 - z0
    L = math.hypot(dy, dz)
    g = box(-FT_D - 0.01, 1.0, -FT_W / 2, FT_W / 2, -L / 2 - FT_W / 2, L / 2 + FT_W / 2)    # along z, centred
    g = g.rotate(V(0, 0, 0), V(1, 0, 0), math.degrees(math.atan2(-dy, dz)))               # +z onto (0, dy, dz)
    return g.translate(V(HALF, (y0 + y1) / 2, (z0 + z1) / 2))


def main():
    parts = VS.read_step(SRC)
    assert len(parts) == 2, [p[0] for p in parts]
    bucket, plate = sorted((p[1] for p in parts), key=lambda s: -s.BoundingBox().zlen)
    col = bucket.intersect(box(-0.01, 0.01, -0.01, 0.01, -1, 30)).Solids()
    assert len(col) == 1, "expected one solid peg at the centre"
    peg_bot, ledge = col[0].BoundingBox().zmin, col[0].BoundingBox().zmax
    peg_top = ledge - FACE_GAP - HALL_T
    zl = peg_top + HALL_T / 2                    # the leads leave the body here
    z_tr = ledge - TR_D                          # ledge troughs' floor
    z_top = ledge - TR_D / 2                     # where the face troughs begin (the holes' middle)

    # ---- bucket
    b = bucket.cut(box(-POST_HALF - 0.1, POST_HALF + 0.1, -POST_HALF - 0.1, POST_HALF + 0.1, peg_top, ledge + 0.5))
    b = b.cut(box(TR_X0, TR_X0 + FAN_L, min(TR_Y.values()) - TR_W / 2, max(TR_Y.values()) + TR_W / 2, z_tr - 0.1,
                  ledge + 0.01))                                                                   # fan-out pocket
    for k, y in TR_Y.items():
        b = b.cut(box(TR_X0, HALF + 1, y - TR_W / 2, y + TR_W / 2, z_tr, ledge + 0.01))          # trough + hole in F1
        b = b.cut(box(SLOT_X0, INNER, y - TR_W / 2, y + TR_W / 2, z_tr - 3.0, z_tr + 0.01))      # through the thin ledge
        b = b.cut(box(INNER - 0.2, HALF + 1, BOTTOM_Y[k] - LAND_W / 2, BOTTOM_Y[k] + LAND_W / 2, -1, LAND_H))  # landing
        b = b.cut(face_trough(y, z_top, BOTTOM_Y[k], -FT_W))                                       # down F1, diagonal
    for y in set(LANES.values()) | {BOTTOM_Y["OUT"]}:                                              # notches: rank wires
        for sx in (1, -1):
            x0, x1 = sorted((sx * (INNER - 0.2), sx * (HALF + 1)))
            b = b.cut(box(x0, x1, y - LANE_W / 2, y + LANE_W / 2, -1, NOTCH_H))
    for sy in (1, -1):                                                                             # and the column wire
        y0, y1 = sorted((sy * (INNER - 0.2), sy * (HALF + 1)))
        b = b.cut(box(COL_X - COL_W / 2, COL_X + COL_W / 2, y0, y1, -1, COL_D))
    b = one(b.clean(), "bucket")

    # ---- base plate: the column trench moves from x -19 to under the signal landing; the diode beside it
    ph = plate.BoundingBox()
    pl = plate.fuse(box(OLD_COL_X - COL_W / 2 - 0.01, OLD_COL_X + COL_W / 2 + 0.01, ph.ymin, ph.ymax, 0, COL_D + 0.01))
    for y in LANES.values():                                              # his rank trenches through the filled part
        pl = pl.cut(box(OLD_COL_X - COL_W, OLD_COL_X + COL_W, y - LANE_W / 2, y + LANE_W / 2, -1, LANE_D))
    pl = pl.cut(box(COL_X - COL_W / 2, COL_X + COL_W / 2, ph.ymin - 1, ph.ymax + 1, -1, COL_D))   # the new column trench
    yo = BOTTOM_Y["OUT"]
    x0, x1, y0, y1, dep = DIODE_REC
    pl = pl.cut(box(x1, ph.xmax + 1, yo - LAND_W / 2, yo + LAND_W / 2, -1, LAND_H))   # the white wire in from its landing
    pl = pl.cut(box(x0, x1, y0, y1, -1, dep))                                          # the diode
    xdc = (x0 + x1) / 2
    pl = pl.cut(box(xdc - 0.5, xdc + 0.5, y1 - 0.1, CATH_Y + 0.5, -1, 1.6))            # the cathode lead on along y
    pl = pl.cut(box(COL_X, xdc + 0.5, CATH_Y - 0.5, CATH_Y + 0.5, -1, 1.6))            # ... and across to the column
    for y in (LANES["ROW"], LANES["GND"]):                                             # heat-shrink pockets: Hall rails
        pl = pl.cut(box(JP_X[0], JP_X[1], y - JP_W / 2, y + JP_W / 2, -1, JP_D))
    pl = pl.cut(box(COL_X - COL_JP[2] / 2, COL_X + COL_JP[2] / 2, COL_JP[0], COL_JP[1], -1, COL_JP[3]))
    pl = one(pl.clean(), "base plate")

    # ---- stand-ins: the Hall, its leads straight out (colour-coded), the extensions to the bottom
    hall = (cq.Workplane("XY").box(HALL_L, HALL_W, HALL_T).edges("|X and >Z").chamfer(0.55).val()
            .translate(V(0, 0, peg_top + HALL_T / 2)))
    leads, ext, run, bare = {}, {}, {}, {}
    xd = (DIODE_REC[0] + DIODE_REC[1]) / 2                         # the diode's axis (along y)
    xt = HALF - FT_D + TUBE_D / 2 + 0.05         # the tube's centre in the face trough
    zw = z_tr + TUBE_D / 2 + 0.05                # ... and in a ledge trough
    tip = HALL_L / 2 + HALL_LEAD_MIN
    for k, y in PINS.items():
        leads[k] = path([(HALL_L / 2, y, zl), (tip, y, zl)], HALL_LEAD_T)
        yt, yb = TR_Y[k], BOTTOM_Y[k]
        zb = TUBE_D / 2 + 0.05
        y_at = yt + (yb - yt) * (z_top - zb) / (z_top + FT_W)          # the face trough's centre at that height
        y_w = yt + (yb - yt) * (z_top - zw) / (z_top + FT_W)           # ... and where the wire comes out of the hole
        pts = [(tip - 1.5, y, zw), (TR_X0 + FAN_L - 0.5, yt, zw), (xt - 0.6, yt, zw), (xt, y_w, zw), (xt, y_at, zb),
               (xt, yb, zb), (INNER + 0.7, yb, zb)]                   # heat-shrunk to the landing, clear of the plate
        ext[k] = path(pts, TUBE_D)
        run[k] = sum((V(*q) - V(*p)).Length for p, q in zip(pts, pts[1:]))
        if k != "OUT":                                                 # + / GND: the bare end on into his trench
            bare[k] = path([(INNER + 0.7, yb, zb), (INNER - 1.0, yb, EXT_D / 2 + 0.05)], EXT_D)
        else:                                                          # signal: on into the plate to the diode's anode
            ext[k] = ext[k].fuse(path([(INNER + 0.7, yb, zb), (DIODE_REC[1] + 0.4, yb, zb)], TUBE_D)).clean()
            bare[k] = path([(DIODE_REC[1] + 0.4, yb, zb), (xd + 0.7, yb + 0.3, 0.9), (xd + 0.5, ANODE_JOINT_Y + 1.2, 0.9)],
                           EXT_D)
    zd = DIODE_REC[4] - DIODE_D / 2 - 0.05
    dbody = cq.Solid.makeCylinder(DIODE_D / 2, DIODE_L, V(xd, DIODE_Y[0], zd), V(0, 1, 0))
    dband = cq.Solid.makeCylinder(DIODE_D / 2 + 0.02, 0.8, V(xd, DIODE_Y[1] - 1.2, zd), V(0, 1, 0))
    dbody = dbody.cut(dband)
    danode = path([(xd, DIODE_Y[0], zd), (xd - 0.2, DIODE_Y[0] - 1.0, 0.9), (xd - 0.2, ANODE_JOINT_Y - 0.4, 0.9)], LEAD_D)
    dcath = path([(xd, DIODE_Y[1], zd), (xd, DIODE_Y[1] + 0.8, 0.9), (xd, CATH_Y, 0.9), (COL_X, CATH_Y, 0.9), (COL_X, CATH_Y, COL_D - INS_D - 0.05)],
                 LEAD_D)
    colw = cq.Solid.makeCylinder(INS_D / 2, 2 * INNER, V(COL_X, -INNER, COL_D - INS_D / 2 - 0.05), V(0, 1, 0))
    jt = {"+": cq.Solid.makeCylinder(JT_D / 2, JT_L, V(JP_X[0] + 0.6, LANES["ROW"], JT_D / 2 + 0.05), V(1, 0, 0)),
          "GND": cq.Solid.makeCylinder(JT_D / 2, JT_L, V(JP_X[0] + 0.6, LANES["GND"], JT_D / 2 + 0.05), V(1, 0, 0)),
          "column": cq.Solid.makeCylinder(CJT_D / 2, JT_L, V(COL_X, COL_JP[0] + 0.5, CJT_D / 2 + 0.3), V(0, 1, 0))}

    # ---- report
    print(f"peg: top {ledge:.2f} -> {peg_top:.2f} (Hall face {FACE_GAP} under the wood); Hall pins at +x, leads straight "
          f"out at z {zl:.2f} to x {tip:.2f}")
    print(f"ledge troughs {TR_W} x {TR_D} deep at y {list(TR_Y.values())} from x {TR_X0} out through F1 (holes under the "
          f"rim, z {z_tr:.2f}..{ledge:.2f}); diagonal troughs {FT_W} x {FT_D} deep down F1 to y "
          f"{ {k: v for k, v in BOTTOM_Y.items()} } at the bottom")
    ts = {k: face_trough(TR_Y[k], z_top, BOTTOM_Y[k], -FT_W).intersect(box(HALF - 2, HALF + 2, -40, 40, 0, ledge))
          for k in TR_Y}
    print(f"   plastic between face troughs: + / GND {ts['+'].distance(ts['GND']):.2f}, GND / OUT "
          f"{ts['GND'].distance(ts['OUT']):.2f}; wall behind them {2.2 - FT_D:.1f}; ledge trough floor at x {SLOT_X0} "
          f"{z_tr - 16.48:.2f} thick, through-slot from there to the wall")
    for k in PINS:
        print(f"   {k:3s} ({'red' if k == '+' else 'black' if k == 'GND' else 'white'}): extension from the lead's tip to "
              f"the {'ROW' if k == '+' else 'GND' if k == 'GND' else '(diode) y ' + str(BOTTOM_Y['OUT'])} notch about "
              f"{run[k]:.0f} mm")
    bad = 0
    for k, s in {**{f"lead {k}": v for k, v in leads.items()}, **{f"tube {k}": v for k, v in ext.items()},
                 **{f"bare end {k}": v for k, v in bare.items()}, "diode": dbody, "diode cathode lead": dcath,
                 "column wire": colw, **{f"heat-shrink {k} joint": v for k, v in jt.items()},
                 "Hall": hall}.items():
        for nm, part in (("bucket", b), ("plate", pl)):
            vv = s.intersect(part).Volume()
            if vv > 1e-3:
                bad += 1
                print(f"   OVERLAP {k} x {nm}: {vv:.3f}")
    print(f"   bucket x plate overlap {b.intersect(pl).Volume():.3f}; {'no overlaps' if not bad else str(bad) + ' overlaps'}")
    print(f"   bucket {bucket.Volume() / 1000:.2f} -> {b.Volume() / 1000:.2f} cm^3, plate {plate.Volume() / 1000:.2f} -> "
          f"{pl.Volume() / 1000:.2f}")
    print(f"   column trench x {COL_X} ({COL_W} x {COL_D}), {ph.xmax - (COL_X + COL_W / 2):.2f} from the plate's edge; diode "
          f"recess x {x0}..{x1} y {y0}..{y1} {dep} deep ({ph.zmax - dep:.1f} roof), {x0 - (COL_X + COL_W / 2):.2f} from the "
          f"column trench; old trench at x {OLD_COL_X} filled")
    print(f"   diode joints off the body: anode (white wire) {DIODE_Y[0] - ANODE_JOINT_Y:.1f} mm of lead, cathode (column "
          f"wire) {(CATH_Y - DIODE_Y[1]) + (xdc - COL_X):.1f} mm of lead; each lead is 25.4")
    print(f"   heat-shrink pockets (Hall rails only): + and GND joints x {JP_X[0]}..{JP_X[1]}, {JP_W} wide x {JP_D} deep in "
          f"the 1.0 trenches ({JP_X[0] - (COL_X + COL_W / 2):.1f} clear of the column trench, "
          f"{-0.5 - (LANES['GND'] + JP_W / 2):.2f} of the data trench); column joint y {COL_JP[0]}..{COL_JP[1]}, {COL_JP[2]} wide x {COL_JP[3]} deep in the {COL_W} trench "
          f"({xdc - 0.5 - (COL_X + COL_JP[2] / 2):.1f} to the cathode lead's channel)")

    a = cq.Assembly(name="mikes_cell_v3")
    a.add(b, name="bucket", color=cq.Color(0.95, 0.95, 0.93))
    a.add(pl, name="base plate", color=cq.Color(0.6, 0.75, 1.0))
    a.add(dbody, name="diode 1N4001", color=cq.Color(0.15, 0.15, 0.17))
    a.add(dband, name="diode stripe (cathode)", color=cq.Color(0.8, 0.8, 0.82))
    a.add(danode, name="diode anode lead", color=cq.Color(0.85, 0.55, 0.3))
    a.add(dcath, name="diode cathode lead", color=cq.Color(0.85, 0.55, 0.3))
    a.add(colw, name="column wire (insulated)", color=cq.Color(0.2, 0.4, 0.9))
    for k, v in jt.items():
        a.add(v, name=f"heat-shrink on the {k} joint", color=cq.Color(0.55, 0.55, 0.6))
    a.add(hall, name="Hall DRV5032FB (bevelled face up)", color=cq.Color(0.3, 0.3, 0.33))
    for k in PINS:
        nm = {"+": "+ red", "GND": "GND black", "OUT": "signal white"}[k]
        a.add(leads[k], name=f"Hall lead {nm}", color=cq.Color(*COLOUR[k]))
        a.add(ext[k], name=f"wire {nm} in 1.5 heat-shrink", color=cq.Color(*COLOUR[k]))
        if k in bare:
            a.add(bare[k], name=f"wire {nm} bare end", color=cq.Color(*COLOUR[k]))
    write_step(a, OUT)
    print(OUT)
    # the two printed parts on their own, nothing else (his board pattern copied the look-only wires and Hall with the
    # cell and merged them into the bucket - pattern these instead)
    for nm, part, col in (("bucket_v3", b, (0.95, 0.95, 0.93)), ("base_plate_v3", pl, (0.6, 0.75, 1.0))):
        q = cq.Assembly(name=nm + "_part")
        q.add(part, name=nm.replace("_v3", "").replace("_", " "), color=cq.Color(*col))
        f = os.path.join(os.path.dirname(OUT), nm + ".step")
        write_step(q, f)
        print(f)


if __name__ == "__main__":
    main()

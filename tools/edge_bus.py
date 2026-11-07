#!/usr/bin/env python3
"""
edge_bus.py - the 5V and GND buses along the board's two edges (Robo chessboard design, 2026-11-06).

    python edge_bus.py     -> chessboard/cell/edge_bus_gnd.step, chessboard/cell/edge_bus_5v.step  (STEP only;
                              each strip as 3 printable pieces, 180 / 180 / 120 long)

The Professor's board (chessboard/cell/mikes_frame_ways.step): 64 cells on a 60 pitch, centres at (60 i, 60 j), z 0 =
the underside. Each rank's bare lanes run along x through its 8 cells (y offsets in a cell: ROW -8.5, GND -2.54, data 0,
5V +2.54) and out through notches in the end walls. His "partial way path" was a 24-wide strip off the -x edge.
His picks (2026-11-06): the edge buses carry only 5V and GND ("rest out" - ROW goes to the Nano on its own wires, data
hops rank to rank); and "Two edges", so nothing crosses: GND runs down the -x edge (his strip's place), 5V down the +x
edge. Each rank's lane runs straight on from its end wall in its own feeder trench to the bus, where a round pocket
gives room to solder ("a little area around circle where we could have some room to solder").
Trenches open on the underside like the base plates', so every joint is made from below.
"""
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
CELL = os.path.join(ROOT, "chessboard", "cell")
OUT_GND = os.path.join(CELL, "edge_bus_gnd.step")
OUT_5V = os.path.join(CELL, "edge_bus_5v.step")

PITCH, N = 60.0, 8
BUCKET_HALF = 29.9                       # the cells' outside
LANES = {"ROW": -8.5, "GND": -2.54, "data": 0.0, "5V": 2.54}
LANE_W, LANE_D = 1.0, 1.0                # bare 20 AWG, as the base plates' trenches
STRIP_W, STRIP_T = 24.0, 3.7             # his strip: 24 wide, the base plate's 3.7 thick
STRIP_GAP = 0.55                         # his strip stood this far off the cells
BUS_X_IN = 12.0                          # the bus trench, this far in from the strip's board-side edge (mid-strip)
POCKET_D, POCKET_DEPTH = 5.0, 1.7        # round soldering pocket at each feeder's joint with the bus
SPLITS = (150.0, 330.0)                  # his pick "Print in sections": cut between ranks 2|3 and 5|6 -> 180 / 180 / 120 long,
SPLIT_GAP = 0.2                          # all under the Ender's ~220 bed; the pieces butt with this gap


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} solids"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def strip(side, lane):
    """side -1: off the -x edge, +1: off the +x edge; lane: the rank lane it gathers"""
    y0, y1 = -BUCKET_HALF, (N - 1) * PITCH + BUCKET_HALF
    edge = -BUCKET_HALF if side < 0 else (N - 1) * PITCH + BUCKET_HALF        # the board's outside along x
    inner = edge + side * STRIP_GAP                                             # the strip's board-side edge
    outer = inner + side * STRIP_W
    xa, xb = sorted((inner, outer))
    s = box(xa, xb, y0, y1, 0, STRIP_T)
    xbus = inner + side * BUS_X_IN
    s = s.cut(box(xbus - LANE_W / 2, xbus + LANE_W / 2, y0 - 1, y1 + 1, -1, LANE_D))              # the bus, end to end
    for j in range(N):
        y = j * PITCH + LANES[lane]
        fa, fb = sorted((inner - side * 1.0, xbus))
        s = s.cut(box(fa, fb, y - LANE_W / 2, y + LANE_W / 2, -1, LANE_D))                       # the rank's feeder
        s = s.cut(cq.Solid.makeCylinder(POCKET_D / 2, POCKET_DEPTH + 1, V(xbus, y, -1), V(0, 0, 1)))  # solder room
    return one(s.clean(), f"{lane} strip"), xbus, (xa, xb)


def main():
    gnd, xg, spg = strip(-1, "GND")
    v5, x5, sp5 = strip(+1, "5V")
    # ---- checks against the edge cells (his printed parts, from cell_edit.py)
    bucket = VS.read_step(os.path.join(CELL, "bucket_v3.step"))[0][1]
    plate = VS.read_step(os.path.join(CELL, "base_plate_v3.step"))[0][1]
    worst = 1e9
    for i, s in ((0, gnd), (N - 1, v5)):
        for j in (0, N - 1):
            for part in (bucket, plate):
                q = part.translate(V(i * PITCH, j * PITCH, 0))
                assert s.intersect(q).Volume() < 1e-6, "strip overlaps a cell"
                worst = min(worst, s.distance(q))
    # the feeders line up with the cells' end-wall notches for that lane (the notch is open at the lane's y)
    for i, s, lane in ((0, gnd, "GND"), (N - 1, v5, "5V")):
        side = -1 if i == 0 else 1
        x = i * PITCH + side * (BUCKET_HALF - 0.5)
        for j in range(N):
            y = j * PITCH + LANES[lane]
            q = bucket.translate(V(i * PITCH, j * PITCH, 0))
            assert q.intersect(box(x - 0.05, x + 0.05, y - 0.05, y + 0.05, 0.3, 0.6)).Volume() < 1e-9, \
                f"no wall notch for {lane} at rank {j}"
    for s, x, sp, lane in ((gnd, xg, spg, "GND"), (v5, x5, sp5, "5V")):
        bb = s.BoundingBox()
        print(f"{lane} bus strip: x {sp[0]:.2f}..{sp[1]:.2f}, y {bb.ymin:.1f}..{bb.ymax:.1f} ({bb.ylen:.0f} long), "
              f"{STRIP_T} thick; bus trench {LANE_W} x {LANE_D} at x {x:.2f}, open at both ends; 8 feeders at rank y + "
              f"{LANES[lane]}; solder pockets D{POCKET_D} x {POCKET_DEPTH} deep; {s.Volume() / 1000:.1f} cm^3")
    print(f"   clear of the edge cells by {worst:.2f}; every feeder meets its lane's end-wall notch")
    for s, f, nm, col in ((gnd, OUT_GND, "GND bus strip", (0.2, 0.2, 0.22)), (v5, OUT_5V, "5V bus strip", (0.85, 0.2, 0.2))):
        bb = s.BoundingBox()
        cuts = [bb.ymin - 1] + [c for c in SPLITS] + [bb.ymax + 1]
        a = cq.Assembly(name=os.path.basename(f)[:-5] + "_part")
        for k in range(len(cuts) - 1):
            ya = cuts[k] + (SPLIT_GAP / 2 if k > 0 else 0)
            yb = cuts[k + 1] - (SPLIT_GAP / 2 if k < len(cuts) - 2 else 0)
            piece = one(s.intersect(box(bb.xmin - 1, bb.xmax + 1, ya, yb, -1, STRIP_T + 1)).clean(), f"{nm} piece")
            pb = piece.BoundingBox()
            feeders = [j for j in range(N) if pb.ymin < j * PITCH < pb.ymax]
            near = min(min(abs(y - (j * PITCH + o)) for j in range(N) for o in (LANES["GND"], LANES["5V"]))
                       for y in (pb.ymin, pb.ymax) if bb.ymin + 1 < y < bb.ymax - 1) if k or k < len(cuts) - 2 else 0
            print(f"   {nm} {k + 1}/{len(cuts) - 1}: y {pb.ymin:.1f}..{pb.ymax:.1f} ({pb.ylen:.1f} long), ranks {feeders}, "
                  f"cut ends {near - POCKET_D / 2:.1f} from the nearest solder pocket")
            a.add(piece, name=f"{nm} {k + 1} of {len(cuts) - 1}", color=cq.Color(*col))
        write_step(a, f)
        print(f)


if __name__ == "__main__":
    main()

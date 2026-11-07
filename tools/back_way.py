#!/usr/bin/env python3
"""
back_way.py - the back way: the Hall column (signal) wires out the back of the board, toward the arm (Robo chessboard
design, 2026-11-06).

    python back_way.py     -> chessboard/cell/back_way.step  (STEP only; 3 printable pieces)

The Professor: "I want that signal to come to the back. Go to the back. Because everything's going to be back there.
We'll have the shortest run of wires." and "Hall effect switches get their ground from the ground bus on the LEDs. It
gets its high voltage from one of the pins coming along the left edge [the causeway, the a-file edge, looking toward the
robot]. And its signal goes out the back."

Board frame: cell centres at (60 i, 60 j), z 0 = the underside; files run along y, rank 8 (j 7, next to the arm) at the
+y end. Each file's insulated column wire (24 AWG, 1.2) runs in its base-plate trench at x 60 i + 20.6 and out through
the notch in the last square's back wall. Here a strip along the back edge takes each one in a short feeder into one
bundle channel running along x to the a-file corner - where the causeway's buses and bundle also leave - so all the
board's wiring meets in one back corner for the Mega. Open on the underside, like the base plates: laid in from below.
The causeway's own corner is left clear (this strip starts at the board's a-file edge) so its wires go straight out.
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
OUT = os.path.join(CELL, "back_way.step")

PITCH, N = 60.0, 8
BUCKET_HALF = 29.9
COL_X, COL_W, COL_D = 20.6, 1.6, 2.4     # each file's column trench in the base plate (cell frame)
INS_D = 1.2                              # the insulated column wire
GAP = 0.55                               # off the cells, as the causeway
W, T = 10.0, 3.7                         # strip width (along y) and thickness (the base plate's)
CH_W, CH_D = 5.2, 2.6                    # bundle channel: 8 x 1.2 wire, 4 across x 2 deep; 1.1 roof
SPLITS, SPLIT_GAP = (110.6, 290.6), 0.2  # between files' feeders -> 3 pieces, all under the Ender's ~220 bed


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} solids"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def main():
    x0, x1 = -BUCKET_HALF, (N - 1) * PITCH + BUCKET_HALF        # the board's a-file edge .. h-file edge
    y0 = (N - 1) * PITCH + BUCKET_HALF + GAP                    # just behind rank 8
    y1 = y0 + W
    yc = y0 + W / 2 + 0.5                                       # channel centre (a little back from the board side)
    s = box(x0, x1, y0, y1, 0, T)
    s = s.cut(box(x0 - 1, x1 + 1, yc - CH_W / 2, yc + CH_W / 2, -1, CH_D))          # the bundle, open at both ends
    for i in range(N):
        x = i * PITCH + COL_X
        s = s.cut(box(x - COL_W / 2, x + COL_W / 2, y0 - 1, yc, -1, COL_D))         # the file's feeder
    s = one(s.clean(), "back way")

    # ---- checks: the back squares, the causeway's corner, the column trenches line up
    bucket = VS.read_step(os.path.join(CELL, "bucket_v3.step"))[0][1]
    plate = VS.read_step(os.path.join(CELL, "base_plate_v3.step"))[0][1]
    clear = 1e9
    for i in (0, 3, N - 1):
        for q in (bucket, plate):
            qq = q.translate(V(i * PITCH, (N - 1) * PITCH, 0))
            assert s.intersect(qq).Volume() < 1e-6, "back way overlaps a square"
            clear = min(clear, s.distance(qq))
    for i in range(N):                                          # the column notch in each back wall is open
        x = i * PITCH + COL_X
        q = bucket.translate(V(i * PITCH, (N - 1) * PITCH, 0))
        assert q.intersect(box(x - 0.05, x + 0.05, y0 - GAP - 1.0, y0 - GAP - 0.9, 1.5, 1.8)).Volume() < 1e-9, \
            f"file {i}: no column notch in the back wall"
    cw = os.path.join(CELL, "causeway.step")
    if os.path.exists(cw):
        cs = [p[1] for p in VS.read_step(cw)]
        assert all(s.intersect(c).Volume() < 1e-6 for c in cs), "back way overlaps the causeway"
    print(f"back way: x {x0}..{x1} ({x1 - x0:.1f} long), y {y0:.2f}..{y1:.2f} ({W} wide), {T} thick; clear of the squares "
          f"by {clear:.2f}")
    print(f"   bundle channel {CH_W} x {CH_D} deep at y {yc:.2f}, open both ends (a-file corner -> the Mega); "
          f"8 feeders {COL_W} x {COL_D} at x 60 i + {COL_X}, lined up with every back-wall column notch")
    a = cq.Assembly(name="back_way_part")
    cuts = [x0 - 1, *SPLITS, x1 + 1]
    for k in range(len(cuts) - 1):
        xa = cuts[k] + (SPLIT_GAP / 2 if k else 0)
        xb = cuts[k + 1] - (SPLIT_GAP / 2 if k < len(cuts) - 2 else 0)
        piece = one(s.intersect(box(xa, xb, y0 - 1, y1 + 1, -1, T + 1)).clean(), "back way piece")
        pb = piece.BoundingBox()
        print(f"   piece {k + 1}/3: x {pb.xmin:.1f}..{pb.xmax:.1f} ({pb.xlen:.1f} long)")
        a.add(piece, name=f"back way {k + 1} of 3", color=cq.Color(0.71, 0.83, 0.96))
    write_step(a, OUT)
    print(OUT)


if __name__ == "__main__":
    main()

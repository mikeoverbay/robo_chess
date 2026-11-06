#!/usr/bin/env python3
"""
board_mockup.py - a simple block mockup of the chess board, the captured-piece tray and the spare-queen spot,
as STEP (and STL), for laying out the gantry's side mounts.  Not a build model: every square is a plain block.

  python board_mockup.py [--square 60] [--height 76.2] [--margin 9.525] [--out DIR]

Layout, seen from white (x across the files a..h left to right, y from the front edge toward the back, z up):
  - 8 x 8 squares, SQUARE on a side, HEIGHT tall, light and dark blocks
  - behind rank 8, after a MARGIN divider: the tray, 2 rows of 8 blocks lined up with the files
  - behind the tray, after another divider: one block for the spare queen, centred
  - a MARGIN-wide frame round all of it, the same height
The frame (white) is one solid; every square, tray block and the spare-queen block is its own solid.
  board_mockup.step         an assembly: 82 named, coloured parts
  board_mockup_bodies.step  the same 82 solids as plain separate bodies (no assembly), for CAD that prefers that
  board_mockup.stl          the lot as one mesh
Also prints the reach the gantry needs for this layout: the lean to rank 1 and to the furthest spot, with the
column pivots placed so the two leans are equal.
"""
import argparse
import math
import os

import cadquery as cq
from cadquery import Vector

SQUARE = 60.0
HEIGHT = 76.2           # 3 in
MARGIN = 9.525          # 3/8 in
BEAM = 400.0            # gantry: pivot to beam joint (gantry_kin.py)
HEAD_OFFSET = 76.1      # gantry: gripper axis ahead of the column plane (gantry_kin.py)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def layout():
    m, s = MARGIN, SQUARE
    board = (m, m + 8 * s, m, m + 8 * s)
    ty0 = board[3] + m
    tray = (m, m + 8 * s, ty0, ty0 + 2 * s)
    qy0 = tray[3] + m
    qx0 = m + 4 * s - s / 2
    queen = (qx0, qx0 + s, qy0, qy0 + s)
    return board, tray, queen


def build():
    board, tray, queen = layout()
    m, s = MARGIN, SQUARE
    solids = []
    for f in range(8):
        for r in range(8):
            x0, y0 = board[0] + f * s, board[2] + r * s
            dark = (f + r) % 2 == 0                     # a1 dark
            solids.append((f"square_{'abcdefgh'[f]}{r + 1}", box(x0, x0 + s, y0, y0 + s, 0, HEIGHT),
                           (0.35, 0.22, 0.12) if dark else (0.85, 0.72, 0.52)))
    for row in range(2):
        for c in range(8):
            x0, y0 = tray[0] + c * s, tray[2] + row * s
            solids.append((f"tray_{row + 1}_{c + 1}", box(x0, x0 + s, y0, y0 + s, 0, HEIGHT), (0.55, 0.55, 0.6)))
    solids.append(("spare_queen", box(queen[0], queen[1], queen[2], queen[3], 0, HEIGHT), (0.85, 0.65, 0.15)))
    # frame: each group's outline grown by the margin, joined, minus the blocks
    outline = None
    for g in (board, tray, queen):
        b = box(g[0] - m, g[1] + m, g[2] - m, g[3] + m, 0, HEIGHT)
        outline = b if outline is None else outline.fuse(b)
    holes = None
    for g in (board, tray, queen):
        b = box(g[0], g[1], g[2], g[3], -1, HEIGHT + 1)
        holes = b if holes is None else holes.fuse(b)
    frame = outline.cut(holes).clean()
    fs = frame.Solids()
    assert len(fs) == 1, len(fs)
    solids.append(("frame", fs[0], (0.96, 0.96, 0.96)))
    return solids, (board, tray, queen)


def main():
    global SQUARE, HEIGHT, MARGIN
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--square", type=float, default=SQUARE)
    ap.add_argument("--height", type=float, default=HEIGHT)
    ap.add_argument("--margin", type=float, default=MARGIN)
    ap.add_argument("--out", default="board_mockup")
    a = ap.parse_args()
    SQUARE, HEIGHT, MARGIN = a.square, a.height, a.margin
    os.makedirs(a.out, exist_ok=True)

    solids, (board, tray, queen) = build()
    width = board[1] + MARGIN
    depth_sides = tray[3] + MARGIN
    depth_queen = queen[3] + MARGIN
    print(f"squares {SQUARE:g} mm, blocks {HEIGHT:g} mm tall, margin {MARGIN:g} mm")
    print(f"  board {8 * SQUARE:g} x {8 * SQUARE:g}; tray 2 x 8 behind it; spare queen centred behind the tray")
    print(f"  footprint {width:.2f} wide x {depth_sides:.2f} deep at the sides, {depth_queen:.2f} at the queen spot "
          f"({width / 25.4:.2f} x {depth_sides / 25.4:.2f} / {depth_queen / 25.4:.2f} in)")
    # reach: target centres from rank 1 to the spare queen
    y_front = board[2] + SQUARE / 2
    y_back = queen[2] + SQUARE / 2
    y_tray2 = tray[2] + 1.5 * SQUARE
    half = (y_back - y_front) / 2
    pivot_y = (y_front + y_back) / 2 - HEAD_OFFSET
    lean = math.degrees(math.asin(half / BEAM))
    lean_tray = math.degrees(math.asin((y_tray2 - HEAD_OFFSET - pivot_y) / BEAM))
    print(f"  gantry (column {BEAM:g} pivot to beam, gripper {HEAD_OFFSET:g} ahead of the columns): put the pivots "
          f"{pivot_y:.1f} mm behind the front edge -> lean {lean:.1f} deg to rank 1 and to the spare queen, "
          f"{lean_tray:.1f} to the back tray row")
    print(f"  the board top is {HEIGHT:g} above the table: the pivot axis goes {HEIGHT - 25:+.1f} mm higher than with the 25 mm board")

    assy = cq.Assembly(name="board_mockup")
    for name, sld, col in solids:
        assy.add(cq.Workplane("XY").add(sld), name=name, color=cq.Color(*col, 1.0))
    assy.export(os.path.join(a.out, "board_mockup.step"))
    cq.exporters.export(cq.Workplane("XY").add([s for _, s, _ in solids]), os.path.join(a.out, "board_mockup_bodies.step"))
    comp = cq.Compound.makeCompound([s for _, s, _ in solids])
    cq.exporters.export(cq.Workplane("XY").add(comp), os.path.join(a.out, "board_mockup.stl"), tolerance=0.05, angularTolerance=0.2)
    print(f"wrote board_mockup.step (assembly, {len(solids)} parts), board_mockup_bodies.step ({len(solids)} bodies) and board_mockup.stl to {a.out}")
    return solids


if __name__ == "__main__":
    main()

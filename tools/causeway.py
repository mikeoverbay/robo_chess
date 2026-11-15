#!/usr/bin/env python3
"""
causeway.py - the edge causeway, the "terminator" every rank ends in (Robo chessboard design, 2026-11-06).

    python causeway.py     -> chessboard/cell/causeway.step  (STEP only; 3 printable pieces, 180 / 180 / 120 long)

His sketch ("Bottom Ways", "Thru" holes, taps for G, D in, +5 and "To Halls Row Power") and his words: "this causeway is
for our rows and our power for our LEDs and control of our LEDs"; "lift the DN and not use it on this bus, and then use
it on the other side"; "make it as wide as you need to make it. And we'll have to work the board around it".
It replaces the two edge strips (edge_bus.py). Along the board's -x edge, in the board frame (cell centres at 60 i,
60 j; z 0 = the underside; each rank's bare lanes: ROW -8.5, return -13, GND -2.54, data 0, +5 +2.54).

  BOTTOM face (bare wire, soldered from below, like the base plates):
    GND bus and +5 bus, end to end; each rank's GND runs straight on into the GND bus (round solder pocket); the
    rank's +5, ROW, D in and data return each come out to a through-hole near the board
  TOP face ("the other side": insulated 30 AWG kynar only):
    the +5 tap - up its hole, along a groove, down a hole onto the +5 bus (another solder pocket), so it never meets
    the GND bus; each rank's ROW wire and its own LED data wire (the Mega 2560 drives every rank's LEDs on a separate
    line - his pick) - up their holes and along grooves into one bundle channel that runs the board's length
No wire crosses another: on each face every groove runs at its own y, and the bundle channel is beyond them all.
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
OUT = os.path.join(CELL, "causeway.step")

PITCH, N = 60.0, 8
BUCKET_HALF = 29.9
LANES = {"ROW": -8.5, "GND": -2.54, "data": 0.0, "5V": 2.54}
GAP = 0.55                               # off the cells, as his strip
T = 4.5                                  # bottom trenches 1.0, top grooves 1.0 / channel 1.5, >= 2.0 of plastic between
LANE_W, LANE_D = 1.0, 1.0                # bare 22 AWG solid (0.64, his, 2026-11-07) trenches (bottom), grooves (top)
U_GND, U_5V = 4.0, 7.5                   # bus lanes, measured out from the board-side edge (+5 was 8.0)
U_HOLE = 1.6                             # the rank's through-holes, just off the board
HOLE_D = 1.2
POCKET_D, POCKET_DEPTH = 4.5, 1.7        # solder room where a tap meets a bus (bottom); 0.6 clear of the D in hole
# 2026-11-10: the top layer is cut in 3 pieces with a seam at |Y| 255 (arm frame), 14.55 past the causeway's board-side
# edge, and the causeway must stay inside the main piece (its wires go into the main's cells). Was 18.3 wide (bundle 12.0..
# 16.8 x 1.4, 8 across x 2 deep); now the bundle is 4 x 4 in a narrower, deeper channel past the +5 pocket (to u 9.75),
# with nothing under it on the bottom face.
CH_U0, CH_U1, CH_D = 10.4, 12.8, 2.4     # top bundle channel: 8 ROW + 8 LED data wires (0.55 kynar), 4 across x 4 deep
WIDTH = 14.0                             # outer edge 0.55 inside the seam
SPLITS, SPLIT_GAP = (150.0, 330.0), 0.2  # 3 printable pieces, cut between ranks


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(d, x, y, z0, z1):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(x, y, z0), V(0, 0, 1))


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} solids"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


EDGE = -BUCKET_HALF - GAP                 # the strip's board-side edge (x); u runs outward, toward -x


def X(u):
    return EDGE - u


def xspan(u0, u1):
    return sorted((X(u0), X(u1)))


def main():
    y0, y1 = -BUCKET_HALF, (N - 1) * PITCH + BUCKET_HALF
    xa, xb = xspan(0, WIDTH)
    s = box(xa, xb, y0, y1, 0, T)
    w = LANE_W / 2
    bot = lambda u0, u1, y: box(*xspan(u0, u1), y - w, y + w, -1, LANE_D)                       # noqa: E731
    top = lambda u0, u1, y, d=LANE_D: box(*xspan(u0, u1), y - w, y + w, T - d, T + 1)            # noqa: E731
    for u in (U_GND, U_5V):                                                                      # the two buses
        s = s.cut(box(X(u) - w, X(u) + w, y0 - 1, y1 + 1, -1, LANE_D))
    s = s.cut(box(*xspan(CH_U0, CH_U1), y0 - 1, y1 + 1, T - CH_D, T + 1))                      # the bundle channel
    for j in range(N):
        yr = j * PITCH
        g, p5, row, din = (yr + LANES[k] for k in ("GND", "5V", "ROW", "data"))
        s = s.cut(bot(-0.1, U_GND, g))                                                          # GND straight in
        s = s.cut(cyl(POCKET_D, X(U_GND), g, -1, POCKET_DEPTH))
        for y in (p5, row, din):                                                                # up to the top face
            s = s.cut(bot(-0.1, U_HOLE, y))
            s = s.cut(cyl(HOLE_D, X(U_HOLE), y, -1, T + 1))
        s = s.cut(top(U_HOLE, U_5V, p5))                                                        # +5 over the GND bus
        s = s.cut(cyl(HOLE_D, X(U_5V), p5, -1, T + 1))
        s = s.cut(cyl(POCKET_D, X(U_5V), p5, -1, POCKET_DEPTH))
        for y in (row, din):                                                                    # into the bundle
            s = s.cut(top(U_HOLE, CH_U0 + 0.5, y))
    s = one(s.clean(), "causeway")

    # ---- checks: the cells beside it, walls between features
    bucket = VS.read_step(os.path.join(CELL, "bucket_v3.step"))[0][1]
    plate = VS.read_step(os.path.join(CELL, "base_plate_v3.step"))[0][1]
    clear = min(s.distance(q.translate(V(0, j * PITCH, 0))) for j in (0, N - 1) for q in (bucket, plate))
    for j in range(N):
        for k in ("ROW", "GND", "data", "5V"):
            y = j * PITCH + LANES[k]
            q = bucket.translate(V(0, j * PITCH, 0))
            assert q.intersect(box(-BUCKET_HALF + 0.45, -BUCKET_HALF + 0.55, y - 0.05, y + 0.05, 0.3, 0.6)).Volume() < 1e-9, \
                f"no wall notch for {k} at rank {j}"
    chan = box(*xspan(CH_U0, CH_U1), y0 - 1, y1 + 1, T - CH_D, T + 1)        # top cuts vs bottom pockets: plastic left
    p5 = cyl(POCKET_D, X(U_5V), LANES["5V"], -1, POCKET_DEPTH)
    gp = cyl(POCKET_D, X(U_GND), LANES["GND"], -1, POCKET_DEPTH)
    grooves = [top(U_HOLE, CH_U0 + 0.5, LANES[k]) for k in ("ROW", "data")] + [top(U_HOLE, U_5V, LANES["5V"])]
    skin = min([chan.distance(p5), chan.distance(gp)] + [g.distance(p) for g in grooves for p in (p5, gp)])
    assert skin >= 0.7, f"only {skin:.2f} of plastic between a top cut and a bottom pocket"
    ys = sorted(LANES.values())
    print(f"causeway: x {xa:.2f}..{xb:.2f} ({WIDTH} wide, {GAP} off the cells), y {y0}..{y1}, {T} thick; "
          f"{s.Volume() / 1000:.1f} cm^3")
    print(f"   bottom: GND bus x {X(U_GND):.2f}, +5 bus x {X(U_5V):.2f} (1.0 x 1.0, open both ends); per rank: GND "
          f"straight in, D{POCKET_D} solder pockets at both bus joints")
    print(f"   through-holes D{HOLE_D} at x {X(U_HOLE):.2f} for +5 / ROW / D in; +5 down again at x {X(U_5V):.2f}")
    print(f"   top: grooves {LANE_W} x {LANE_D}, bundle channel x {X(CH_U1):.2f}..{X(CH_U0):.2f} {CH_D} deep (ROW x 8 + "
          f"LED data x 8); thinnest plastic between a top cut and a bottom solder pocket {skin:.2f}")
    print(f"   rank feature spacing {min(b - a for a, b in zip(ys, ys[1:])):.2f} -> {min(b - a for a, b in zip(ys, ys[1:])) - HOLE_D:.2f}"
          f" of plastic between neighbouring holes; clear of the cells by {clear:.2f}; every rank's 4 lanes meet a notch")
    a = cq.Assembly(name="causeway_part")
    cuts = [y0 - 1, *SPLITS, y1 + 1]
    for k in range(len(cuts) - 1):
        ya = cuts[k] + (SPLIT_GAP / 2 if k else 0)
        yb = cuts[k + 1] - (SPLIT_GAP / 2 if k < len(cuts) - 2 else 0)
        piece = one(s.intersect(box(xa - 1, xb + 1, ya, yb, -1, T + 1)).clean(), "causeway piece")
        pb = piece.BoundingBox()
        print(f"   piece {k + 1}/3: y {pb.ymin:.1f}..{pb.ymax:.1f} ({pb.ylen:.1f} long)")
        a.add(piece, name=f"causeway {k + 1} of 3", color=cq.Color(0.25, 0.25, 0.28))
    write_step(a, OUT)
    print(OUT)


if __name__ == "__main__":
    main()

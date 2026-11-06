#!/usr/bin/env python3
"""
print_parts.py - the printed parts, grouped and named the way Mike names them, for Shapr3D -> STL (Mike 2026-11-06:
"clean up the step files ... import this into Shaper 3D so I can export it as STLs"; "sensible names for this stuff.
Turntable base. Shoulder turntable. Arm 1, arm 2, arm 3. Wrist. Claw mount. Claw assembly. Gearbox assembly. If all
the gearboxes are the same, we need one good working printable model. I believe the wrist is part of the gearbox.")

    python print_parts.py

Writes robo_chess/print/<group>.step, one file per group, each a flat set of named bodies (no folders) in their
assembled places - only what gets printed: no motors, bearings, servo, horn, fittings, dowels, magnets or Hall chips.
Export each body to STL in Shapr3D; orient it in the slicer. "(print N)" = how many to print.
  Turntable base         the fixed swivel housing, its two cover halves, the 18T pinion on the swivel motor
  Shoulder turntable     the 72T turntable gear and its collar, the shoulder column (base body: J1's gearbox sits in its
                         top) and the claw servo's bracket and cover on its back
  Arm (print 3)          arms 1, 2, 3 are one part (its home magnet: arm 1 in the -45 pocket, arms 2 and 3 in the +90)
  Claw mount             on the wrist gearbox's output (the wrist IS the fourth gearbox)
  Claw assembly          hub, cap, plunger, three fingers (one part, x3)
  Gearbox assembly       the spread cycloid drive (cycloid.py --spread 14), print 4: shoulder, elbow, J3, wrist -
  (print 4)              steel dowel pins; the two discs are one part (x2)
  Chess pieces           each CAD piece and the two pedestals as SEPARATE bodies (print them apart: magnet glued
                         into the pedestal's bottom pocket, pennies in its top cavity, then the piece glued on - which
                         closes the cavity; fused they'd seal it). ONE pedestal for all: a stepped recess, queen / king
                         (28.8 bottoms) in its 29.3 top step, the rest (25.3..25.5) in the 25.9 step below. Counts are both sides plus
                         the two spare queens. The knight is Mike's own STL (chessboard/pieces/Knight_10x2mm.stl).
Re-run after any change (and cycloid.py --spread 14 after a gearbox change). Nothing outside print/ is touched.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

from view_step import read_step  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402

ROOT = os.path.dirname(HERE)
CA = os.path.join(ROOT, "complete assembly")
OUT = os.path.join(ROOT, "print")
SWIVEL_FIXED = os.path.join(CA, "swivel_fixed.step")
SWIVEL_TURN = os.path.join(CA, "swivel_turntable.step")
SWIVEL_PINION = os.path.join(CA, "swivel_pinion.step")
BASE = os.path.join(CA, "arm_base_tube.step")
SERVO = os.path.join(CA, "servo_on_base.step")
LINK = os.path.join(CA, "mockups", "mockup_link_plus.step")
CLAW = os.path.join(ROOT, "gripper", "claw_on_wrist_spread.step")
GEARBOX = os.path.join(ROOT, "cycloid_spread", "step", "assembly_steel_pins.step")
PIECE = os.path.join(ROOT, "chessboard", "pieces", "step", "{}.step")       # chess_pieces.py's CAD pieces
PEDESTAL = os.path.join(ROOT, "chessboard", "pedestal", "pedestal{}.step")
BLUE, ORANGE, GREY, GREEN = (0.35, 0.6, 0.8), (0.9, 0.55, 0.3), (0.72, 0.74, 0.78), (0.45, 0.75, 0.45)
# (file, [(source STEP, its part name, the body's name here, colour)])
GROUPS = [
    ("Turntable base", [(SWIVEL_FIXED, "swivel housing", "Base housing", GREY),
                        (SWIVEL_FIXED, "swivel cover left", "Base cover left", BLUE),
                        (SWIVEL_FIXED, "swivel cover right", "Base cover right", BLUE),
                        (SWIVEL_PINION, "swivel pinion 18T", "Base pinion 18T", ORANGE)]),
    ("Shoulder turntable", [(SWIVEL_TURN, "swivel turntable 72T", "Turntable gear 72T", ORANGE),
                            (SWIVEL_TURN, "swivel collar", "Turntable collar", GREY),
                            (BASE, "base body", "Shoulder column", BLUE),
                            (SERVO, "servo bracket", "Claw servo bracket", GREY),
                            (SERVO, "servo cover", "Claw servo cover", GREEN)]),
    ("Arm (print 3)", [(LINK, "mockup plus link", "Arm (print 3)", GREY)]),
    ("Claw mount", [(CLAW, "claw adapter", "Claw mount", GREEN)]),
    ("Claw assembly", [(CLAW, "claw hub", "Claw hub", GREY), (CLAW, "claw cap", "Claw cap", GREY),
                       (CLAW, "claw plunger", "Claw plunger", ORANGE),
                       (CLAW, "claw finger0", "Claw finger 1 (one part, print 3)", BLUE),
                       (CLAW, "claw finger1", "Claw finger 2", BLUE), (CLAW, "claw finger2", "Claw finger 3", BLUE)]),
    ("Gearbox assembly (print 4)", [(GEARBOX, "housing", "Gearbox housing", BLUE),
                                    (GEARBOX, "output support ring", "Gearbox output support ring", GREY),
                                    (GEARBOX, "disc 1", "Gearbox disc 1 (one part, print 2)", ORANGE),
                                    (GEARBOX, "disc 2", "Gearbox disc 2", ORANGE),
                                    (GEARBOX, "eccentric lower", "Gearbox eccentric lower", GREEN),
                                    (GEARBOX, "eccentric upper", "Gearbox eccentric upper", GREEN),
                                    (GEARBOX, "output flange", "Gearbox output flange", GREY),
                                    (GEARBOX, "pin ring", "Gearbox pin ring", BLUE),
                                    (GEARBOX, "lid", "Gearbox lid", BLUE),
                                    (GEARBOX, "inner race spacer", "Gearbox inner race spacer", GREY),
                                    (GEARBOX, "outer race spacer", "Gearbox outer race spacer", BLUE),
                                    (GEARBOX, "clamp ring", "Gearbox clamp ring", GREY)]),
]
PIECES = [("Pawn", 16), ("Rook", 4), ("Bishop", 4), ("Queen", 4), ("King", 2)]   # both sides + 2 spare queens
PEDESTALS = [("", 34)]                        # one pedestal for every piece: 32 + the 2 spare queens


def main():
    os.makedirs(OUT, exist_ok=True)
    cache = {}

    def part(src, name):
        if src not in cache:
            cache[src] = read_step(src)
        found = [s for nm, s, _ in cache[src] if name is None or nm == name]
        assert found, f"{name} not in {src}"
        s = found[0]
        assert len(s.Solids()) == 1 and BRepCheck_Analyzer(s.wrapped).IsValid(), f"{name}: not one clean solid"
        return s

    groups = [(f, [(part(src, nm), body, col) for src, nm, body, col in items]) for f, items in GROUPS]
    pieces = []
    for i, (kind, n) in enumerate(PIECES):                 # side by side, standing on z 0
        s = part(PIECE.format(kind), None)
        b = s.BoundingBox()
        pieces.append((s.translate(V(45.0 * i - b.center.x, -b.center.y, -b.zmin)), f"{kind} (print {n})", GREY))
    for i, (size, n) in enumerate(PEDESTALS):
        s = part(PEDESTAL.format(size), None)
        b = s.BoundingBox()
        pieces.append((s.translate(V(45.0 * i - b.center.x, 60.0 - b.center.y, -b.zmin)),
                       f"Pedestal (print {n})", BLUE))
    groups.append(("Chess pieces", pieces))
    for f, bodies in groups:
        write_flat([(body, s, cq.Color(*col)) for s, body, col in bodies], os.path.join(OUT, f + ".step"))
        print(f"{f + '.step':34s} " + ", ".join(body for _, body, _ in bodies))
    print(OUT)


if __name__ == "__main__":
    main()

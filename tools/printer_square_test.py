#!/usr/bin/env python3
"""
printer_square_test.py - is the printer's X square to its Y? (Mike, 2026-11-23)

Four round posts on the corners of a 70.711 mm square (100 x 0.7071), so post centre to post centre across each
diagonal is exactly 100.000. Thin bars join them so the part stays flat. Letters A B C D are cut into the post tops:
A front-left, B front-right, C back-right, D back-left (as it sits on the bed, X to the right, Y to the back).

Measure with calipers, outside jaws, across the posts at mid-height (above the bars):
    AC, BD   the two diagonals (corner post to opposite corner post)
    AB       the X side,  AD  the Y side
    d        one post's own diameter (the 0.4 line width and any over-extrusion are in it; subtracting it cancels them)

    python printer_square_test.py                     writes robo_chess/print/xy_square_test.step
    python printer_square_test.py AC BD AB AD d       works out the skew and the X / Y scale
"""
import math
import os
import sys

SIDE = 100.0 / math.sqrt(2)     # 70.711: centre to centre along X and Y
POST_D, POST_H = 8.0, 5.0      # Mike: small, about 5 tall
BAR_W, BAR_T = 3.0, 0.6        # three 0.2 layers: just holds the posts in place (no brim)
LETTER_H, LETTER_DEPTH = 5.0, 0.6
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "print", "xy_square_test.step")


def build():
    import cadquery as cq
    h = SIDE / 2
    corners = {"A": (-h, -h), "B": (h, -h), "C": (h, h), "D": (-h, h)}
    part = None

    def add(s):
        nonlocal part
        part = s if part is None else part.union(s)

    for (x0, y0), (x1, y1) in (((-h, -h), (h, -h)), ((h, -h), (h, h)), ((h, h), (-h, h)), ((-h, h), (-h, -h)),
                               ((-h, -h), (h, h)), ((h, -h), (-h, h))):
        L = math.hypot(x1 - x0, y1 - y0)
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
        bar = (cq.Workplane("XY").rect(L, BAR_W).extrude(BAR_T)
               .rotate((0, 0, 0), (0, 0, 1), ang).translate(((x0 + x1) / 2, (y0 + y1) / 2, 0)))
        add(bar)
    for name, (x, y) in corners.items():
        add(cq.Workplane("XY").center(x, y).circle(POST_D / 2).extrude(POST_H))
    for name, (x, y) in corners.items():
        letter = (cq.Workplane("XY").workplane(offset=POST_H - LETTER_DEPTH).center(x, y)
                  .text(name, LETTER_H, LETTER_DEPTH + 0.1, kind="bold", halign="center", valign="center"))
        part = part.cut(letter)
    return part


def solve(ac, bd, ab, ad, d):
    AC, BD, a, b = ac - d, bd - d, ab - d, ad - d          # centre-to-centre
    cos_g = (AC ** 2 - BD ** 2) / (4 * a * b)               # parallelogram: AC^2 - BD^2 = 4ab cos(angle between X and Y)
    gamma = math.degrees(math.acos(max(-1.0, min(1.0, cos_g))))
    skew = 90.0 - gamma
    print(f"centre to centre: AC {AC:.3f}  BD {BD:.3f}  (both should be 100.000)   AB {a:.3f}  AD {b:.3f}  (both 70.711)")
    print(f"X scale {a / SIDE * 100:.2f} %   Y scale {b / SIDE * 100:.2f} %")
    print(f"angle between X and Y: {gamma:.3f} deg -> out of square by {abs(skew):.3f} deg "
          f"= {abs(math.tan(math.radians(skew))) * 100:.2f} mm over 100 mm")
    if abs(AC - BD) < 0.1:
        print("diagonals within 0.1 mm of each other: square enough for these parts")
    else:
        longer = "AC (front-left to back-right)" if AC > BD else "BD (front-right to back-left)"
        print(f"the longer diagonal is {longer}, by {abs(AC - BD):.2f} mm")


if __name__ == "__main__":
    if len(sys.argv) == 6:
        solve(*map(float, sys.argv[1:]))
    else:
        from cadquery import exporters
        p = build()
        s = p.val()
        print("valid", s.isValid(), "volume", round(s.Volume() / 1e3, 2), "cm3")
        bb = s.BoundingBox()
        print(f"size {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm")
        exporters.export(p, OUT)
        print("written", OUT)

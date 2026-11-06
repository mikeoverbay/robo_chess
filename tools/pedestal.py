#!/usr/bin/env python3
"""
pedestal.py - a standard printed base that every chess piece glues onto, written as STL and STEP.

  python pedestal.py [--od 33] [--height 8] [--magnet 6x1.5] [--weights 3x19.05x1.52] [--weight-g 2.5]
                     [--locate 25.6,29.0] [--out DIR]

Weights: the default cavity takes three US pennies (19.05 x 1.52, 2.5 g each; zinc, so no magnetic
backing).  For two M6 fender washers instead: --weights 2x25x2 --weight-g 7.2 (steel: they also push
more of the magnet's field down toward the board sensor).

Needs:  python -m pip install cadquery

Why: the gripper needs the same lip at the same height on every piece, each piece needs a magnet the
board's Hall switch can see, and the magnet cannot be dropped in during the print because a hardened
steel nozzle is magnetic and lifts it out.  So the magnet is glued into a pocket that opens at the
BOTTOM, 0.3 mm up from the floor so it never scrapes the board.  Weights (pennies or washers) go in
from the TOP into a cavity the piece's own bottom face closes when it is glued on.

Profile, bottom to top:
  - 1 mm 45 deg chamfer on the bottom edge
  - plain band, then the finger waist: a groove 2 mm deep from WAIST0 to WAIST1 with a square lower
    shoulder for the fingers to seat under and a 45 deg upper shoulder so it prints without support
  - plain band to the top; a shallow locating recess the piece's bottom drops into for gluing
Printed bottom face down.  The only roof is the 1 mm skin over the magnet pocket, a 6.3 mm bridge.

ONE pedestal for every piece (Mike 2026-11-06: "If all of these fit on a larger pedestal, get rid of the small
pedestal"): a STEPPED locating recess - the queen / king bottom (28.7) in the top step, the pawn / rook / bishop /
knight bottom (25.3) one step deeper - so each piece sits centred. It stands 0.4 taller than the old pair (8.4), so
the penny cavity still fits under the deeper seat with the 1 mm skin over the magnet; the finger waist is measured
from the bottom, so the claw grips it exactly where it always did. Writes pedestal.step (STEP only).
"""
import argparse
import math
import os

import cadquery as cq
from cadquery import Vector

OD = 33.0            # outside diameter; the king's bottom is 28.7, its bead 32.7
HEIGHT = 8.4         # 8 + one more recess step (was two pedestals, 8 tall)
CHAMFER = 1.0        # bottom edge
WAIST0 = 3.0         # finger groove, from the bottom face
WAIST1 = 5.5
WAIST_DEPTH = 2.0
MAGNET_D = 6.0
MAGNET_T = 1.5
MAGNET_CLEAR = 0.3   # radial clearance in the pocket (diameter 6.3) and the glue line
MAGNET_RECESS = 0.3  # magnet face stays this far above the floor
SKIN = 1.0           # between the magnet pocket and the washer cavity
WASHER_N = 3         # weights in the cavity: default three US pennies
WASHER_OD = 19.05
WASHER_T = 1.52
WASHER_G = 2.5       # grams each (US Mint, post-1982 penny)
WASHER_CLEAR = 0.5   # diameter clearance in the cavity
LOCATE_DEPTH = 0.4   # recess in the top face the piece's bottom drops into
LOCATE_CLEAR = 0.3   # over the piece's bottom diameter
PETG_DENSITY = 1.27  # g/cm^3


def one_solid(shape):
    if shape.ShapeType() == "Solid":
        return shape
    s = shape.Solids()
    assert len(s) == 1, len(s)
    return s[0]


def pedestal(locate_ds):
    """locate_ds: the piece bottom diameters it centres, each a LOCATE_DEPTH step below the wider one"""
    ds = sorted(locate_ds, reverse=True)
    r = OD / 2
    # outer profile in the XZ half plane, revolved: (radius, z) going up the outside
    pts = [(0, 0), (r - CHAMFER, 0), (r, CHAMFER), (r, WAIST0), (r - WAIST_DEPTH, WAIST0),
           (r - WAIST_DEPTH, WAIST1 - WAIST_DEPTH), (r, WAIST1), (r, HEIGHT), (0, HEIGHT)]
    body = cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()
    body = one_solid(body)
    # magnet pocket from below: floor + recess up to the magnet's top
    pocket_h = MAGNET_RECESS + MAGNET_T
    body = body.cut(cq.Solid.makeCylinder((MAGNET_D + MAGNET_CLEAR) / 2, pocket_h + 1, Vector(0, 0, -1)))
    # washer cavity from above, down to the skin (under the deepest seat: the smallest piece closes it lowest)
    cav_top = HEIGHT - LOCATE_DEPTH * len(ds)
    cav_h = WASHER_N * WASHER_T + 0.2
    cav0 = pocket_h + SKIN
    assert cav_top - cav_h >= cav0 - 1e-9, f"not tall enough: cavity needs {cav0 + cav_h:.1f}"
    body = body.cut(cq.Solid.makeCylinder((WASHER_OD + WASHER_CLEAR) / 2, cav_h + 1, Vector(0, 0, cav_top - cav_h)))
    # locating recesses for the pieces, stepped, concentric with the cavity (a piece's bottom closes the cavity)
    for k, d in enumerate(ds, start=1):
        body = body.cut(cq.Solid.makeCylinder((d + LOCATE_CLEAR) / 2, LOCATE_DEPTH * k + 1, Vector(0, 0, HEIGHT - LOCATE_DEPTH * k)))
    return one_solid(body), cav0, cav_h


def main():
    global OD, HEIGHT, MAGNET_D, MAGNET_T, WASHER_N, WASHER_OD, WASHER_T, WASHER_G
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--od", type=float, default=OD)
    ap.add_argument("--height", type=float, default=HEIGHT)
    ap.add_argument("--magnet", default=f"{MAGNET_D:g}x{MAGNET_T:g}", help="disc magnet DxT, mm")
    ap.add_argument("--weights", default=f"{WASHER_N}x{WASHER_OD:g}x{WASHER_T:g}", help="count x OD x thickness of the coins or washers")
    ap.add_argument("--weight-g", type=float, default=WASHER_G, help="grams per coin or washer")
    ap.add_argument("--locate", default="25.6,29.0", help="piece bottom diameters: small,large")
    ap.add_argument("--out", default="pedestal_out")
    a = ap.parse_args()
    OD, HEIGHT = a.od, a.height
    MAGNET_D, MAGNET_T = (float(v) for v in a.magnet.lower().split("x"))
    n, wod, wt = a.weights.lower().split("x")
    WASHER_N, WASHER_OD, WASHER_T, WASHER_G = int(n), float(wod), float(wt), a.weight_g
    small, large = (float(v) for v in a.locate.split(","))
    os.makedirs(a.out, exist_ok=True)

    wall_cavity = (OD / 2 - WAIST_DEPTH) - (WASHER_OD + WASHER_CLEAR) / 2
    washer_g = WASHER_N * WASHER_G
    print(f"pedestal OD {OD:g} x {HEIGHT:g}; finger waist z {WAIST0:g}..{WAIST1:g}, {WAIST_DEPTH:g} deep "
          f"(grip diameter {OD - 2 * WAIST_DEPTH:g})")
    print(f"  magnet {MAGNET_D:g} x {MAGNET_T:g} in a {MAGNET_D + MAGNET_CLEAR:g} pocket from below, face {MAGNET_RECESS:g} above the floor; "
          f"{SKIN:g} skin; {WASHER_N} weights {WASHER_OD:g} x {WASHER_T:g} from above ({washer_g:.1f} g)")
    print(f"  wall between the waist and the washer cavity {wall_cavity:.2f} mm (want 1.5)")
    assert wall_cavity >= 1.5 - 1e-9
    solid, cav0, cav_h = pedestal((small, large))
    assert solid.isValid()
    vol = solid.Volume()
    seat_l, seat_s = HEIGHT - LOCATE_DEPTH, HEIGHT - 2 * LOCATE_DEPTH
    print(f"  pedestal: recess {large + LOCATE_CLEAR:g} (queen, king; their bottoms sit at z {seat_l:g}) over "
          f"{small + LOCATE_CLEAR:g} (pawn, rook, bishop, knight; z {seat_s:g}); cavity z {seat_s - cav_h:g}..{seat_s:g} "
          f"({cav_h:g} for the weights), skin {seat_s - cav_h - (MAGNET_RECESS + MAGNET_T):.2f} over the magnet pocket; "
          f"plastic {vol:.0f} mm^3 = {vol / 1000 * PETG_DENSITY:.1f} g, with the weights {vol / 1000 * PETG_DENSITY + washer_g:.1f} g")
    cq.exporters.export(cq.Workplane("XY").add(solid), os.path.join(a.out, "pedestal.step"))
    print(f"wrote {os.path.join(a.out, 'pedestal.step')}")


if __name__ == "__main__":
    main()

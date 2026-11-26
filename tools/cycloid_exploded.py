#!/usr/bin/env python3
"""
cycloid_exploded.py - the spread cycloid gearbox (cycloid.py --spread 14) as an EXPLODED view (Mike 2026-11-06: "produce
me a step file of the new gearbox in an exploded view ... something similar to this [a hand-drawn exploded tape measure].
All pushed up along the axis.")

    python cycloid_exploded.py [--gap 12] [--bearings]

--bearings: the cycloid.py --spread 14 --out-bearings variant (bearings on M3 output bolts, no support ring), from and
to cycloid_spread_bearings/step/.

Reads cycloid_spread/step/assembly_steel_pins.step (run cycloid.py --spread 14 first) and writes
cycloid_spread/step/assembly_exploded.step: every part slid straight up the motor axis from where it sits assembled,
in the order it goes on, each level GAP above the one below. Screws, pins and inserts float just above the part they
go into, still pointing in. Nothing is turned or moved sideways, so each part is still on its own axis line.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

from regroup_assembly import write_flat  # noqa: E402
from view_step import read_step  # noqa: E402

SRC = os.path.join(os.path.dirname(HERE), "cycloid_spread", "step", "assembly_steel_pins.step")
DST = os.path.join(os.path.dirname(HERE), "cycloid_spread", "step", "assembly_exploded.step")
BLUE, ORANGE, ORANGE2, GREEN, GREEN2 = (0.2, 0.45, 0.85), (0.9, 0.5, 0.1), (0.95, 0.65, 0.2), (0.3, 0.75, 0.3), (0.4, 0.85, 0.4)
NAVY, STEEL, BRASS, BLACK, GREY = (0.2, 0.3, 0.6), (0.75, 0.75, 0.78), (0.8, 0.65, 0.2), (0.15, 0.15, 0.15), (0.6, 0.6, 0.62)
# bottom to top: (name prefix(es), colour). Each level is everything whose name starts with one of its prefixes.
LEVELS = [
    (("motor body", "motor shaft"), BLACK),       # stays put
    (("housing floor",), BLUE),                    # the two-piece housing (2026-11-15): the floor on the motor,
    (("motor screw",), BLACK),                     # from inside the floor, down into the motor,
    (("housing ring",), BLUE),                     # then the ring on the floor
    (("ring pin",), STEEL),                        # down the wall grooves into the floor
    (("output support ring",), NAVY),
    (("output pin",), STEEL),                      # pressed into the support ring
    (("eccentric spacer",), GREEN2),               # the cartridge: spacer, disc 1 + bearing from below,
    (("output spacer under disc 1",), STEEL),     # the bolts' pinched stacks (2026-11-22): steel spacers between the pairs
    (("disc 1",), ORANGE),
    (("6800 in disc 1",), STEEL),
    (("output pin bearing, disc 1",), STEEL),     # 3 x 6 x 2s down the output pins into disc 1's pockets (2026-11-19)
    (("output spacer between the discs",), STEEL),
    (("eccentric",), GREEN),                       # the one-piece eccentric,
    (("6800 in disc 2",), STEEL),                  # disc 2 + bearing from above
    (("disc 2",), ORANGE2),
    (("output pin bearing, disc 2",), STEEL),     # ... and disc 2's
    (("output spacer over disc 2",), STEEL),
    (("top bearing",), STEEL),                     # pressed onto the eccentric's top journal (2026-11-17)
    (("pin ring",), BLUE),
    (("output bolt",), BLACK),                     # the bearings variant: M3s up into the flange (2026-11-21)
    (("output flange",), NAVY),
    (("insert",), BRASS),                          # heat-set into the flange face
    (("6808 1",), STEEL),
    (("inner race spacer", "outer race spacer"), NAVY),
    (("6808 2",), STEEL),
    (("lid",), BLUE),
    (("lid screw",), BLACK),
    (("clamp ring",), NAVY),
    (("your output disc",), GREY),
    (("output screw",), BLACK),
]


def label(name, s):
    """bought parts already carry their spec from cycloid.py ("lid screw 1: M3 x 55 ..."); printed ones get their
    material and size here (Mike 2026-11-06: "explain what their names, their sizes"). The assembly files keep the
    plain printed names because print_parts.py looks them up by name."""
    if ":" in name:
        return name
    b = s.BoundingBox()
    solid = " solid (100 % infill)" if name.startswith("disc") else ""
    return f"{name}: printed PA12{solid}, {max(b.xlen, b.ylen):.1f} across x {b.zlen:.1f} tall"


def level_of(name):
    """the level whose prefix matches the name longest ("lid screw M3x55 2" is a lid screw, not the lid)"""
    hits = [(len(p), i) for i, (prefixes, _) in enumerate(LEVELS) for p in prefixes if name.startswith(p)]
    if not hits:
        raise ValueError(f"no level for part {name!r}: add it to LEVELS")
    return max(hits)[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gap", type=float, default=12.0, help="mm between one level's top and the next one's bottom")
    ap.add_argument("--bearings", action="store_true", help="the --out-bearings variant (cycloid_spread_bearings)")
    ap.add_argument("--tubes", action="store_true", help="the --tube-pins variant (cycloid_spread_tubes)")
    a = ap.parse_args()
    src, dst = SRC, DST
    if a.bearings:
        src, dst = (f.replace(os.sep + "cycloid_spread" + os.sep, os.sep + "cycloid_spread_bearings" + os.sep) for f in (SRC, DST))
    if a.tubes:
        src, dst = (f.replace(os.sep + "cycloid_spread" + os.sep, os.sep + "cycloid_spread_tubes" + os.sep) for f in (SRC, DST))
    # levels that only one build has: the main one has no pin bearings or bolts; the variant no support ring or pins
    variant = {"output pin bearing, disc 1", "output pin bearing, disc 2", "output bolt", "output spacer under disc 1",
               "output spacer between the discs", "output spacer over disc 2"}
    optional = ({"output support ring", "output pin"} if a.bearings else variant | {"output support ring"} if a.tubes else
                {"output pin bearing, disc 1", "output pin bearing, disc 2", "output bolt", "output spacer under disc 1",
                 "output spacer between the discs", "output spacer over disc 2"})
    parts = read_step(src)
    by_level = [[] for _ in LEVELS]
    for name, s, _ in parts:
        by_level[level_of(name)].append((name, s))
    out = []
    top = None
    for i, items in enumerate(by_level):
        if not items:
            if LEVELS[i][0][0] in optional:
                continue
            raise ValueError(f"level {LEVELS[i][0]} is empty: a part was renamed in cycloid.py?")
        z0 = min(s.BoundingBox().zmin for _, s in items)
        z1 = max(s.BoundingBox().zmax for _, s in items)
        dz = 0.0 if top is None else top + a.gap - z0
        top = z1 + dz
        col = cq.Color(*LEVELS[i][1])
        for name, s in items:
            c = cq.Color(*BRASS) if "brass tube" in name else col      # the --tube-pins output pins
            out.append((label(name, s), s.translate(V(0, 0, dz)) if dz else s, c))
        print(f"{' / '.join(LEVELS[i][0]):40s} x{len(items):<3d} up {dz:6.1f}  ->  z {z0 + dz:6.1f} .. {top:6.1f}")
    write_flat(out, dst)
    print(f"{len(out)} bodies, {top - min(s.BoundingBox().zmin for _, s, _ in out):.0f} mm tall -> {dst}")


if __name__ == "__main__":
    main()

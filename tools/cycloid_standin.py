#!/usr/bin/env python3
"""
cycloid_standin.py - a one-piece stand-in for the 37:1 cycloid drive (with its motor), for fit checks and wiring
in the arm. Outside only: no gears, pins or bearings, but every outside surface and hole is where the real drive
has it, because it is built from cycloid.py's own parts.

    python cycloid_standin.py

Kept (drive frame: housing bottom z 0, axis on z, the same frame as the arm's joints):
  - housing / pin ring / lid outline: 70.5 dia to z 41.7, lid top 57 dia to z 44.7
  - output: clamp ring 48 dia top face at z 45.2 (where a link's cup sits), flange face at 43.1 inside it,
    3 x insert holes 4.0 dia on r 15 at 0/120/240 deg for the M3 output screws
  - 6 x lid screw holes on r 32.4 at 1.58 + 60k deg: 3.4 through lid and pin ring, 2.5 down the housing to its
    bottom (the M3 x 16 mount screws come up into these from the link plate)
  - motor: Mike's own NEMA 17 (cycloid/nema17.step, from nema17.py) under the housing, front face on the housing
    bottom, kept as its own parts ("NEMA 17" black body, "NEMA 17 end caps") so they keep their colours; the shaft
    is cut at the housing bottom (inside, it would only overlap the filled stand-in)
Out: robo_chess/cycloid/cycloid_drive_standin.step (also seen as complete assembly/cycloid gearbox/...).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.BRepClass3d import BRepClass3d  # noqa: E402

import cycloid as C  # noqa: E402
import nema17 as N  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

OUT = os.path.join(os.path.dirname(HERE), "cycloid", "cycloid_drive_standin.step")
MOTOR_CHAMFER = 4.5            # corner chamfer leg: 42.3 - 4.5 -> 26.7 from the axis (53.4 across)


def motor():
    h = 21.15
    c = h - MOTOR_CHAMFER
    pts = [(h, c), (c, h), (-c, h), (-h, c), (-h, -c), (-c, -h), (c, -h), (h, -c)]
    body = cq.Workplane("XY", origin=(0, 0, -48.0)).polyline(pts).close().extrude(48.0).val()
    return body.fuse(C.cyl(22.0, 2.0))                     # the 22 x 2 pilot boss up into the housing


def main():
    parts = [C.housing(True), C.pin_ring(), C.lid(), C.clamp_ring(), C.output_flange()]
    # fill the inside so only the outer skin is left: the housing cavity, the lid's bearing pocket, the flange's
    # centre pocket - all under the parts above, none of it reaching an outside surface or a screw hole
    # the bearings (left out) fill the lid pocket up to the clamp ring; the 3 output insert holes stay open
    pocket = C.cyl(52.0, C.Z_BRG_TOP - C.Z_BRG + 0.01, C.Z_BRG - 0.01)
    face = C.Z_BRG_TOP - 0.1
    for a in C.OUT_SCREW_ANGLES:
        pocket = pocket.cut(C.cyl(C.INSERT_HOLE, C.INSERT_DEPTH + 2, face - C.INSERT_DEPTH, *C.polar(C.OUT_SCREW_R, a)))
    fill = [C.cyl(2 * C.WALL_R, C.Z_TOP - C.FLOOR_T + 0.01, C.FLOOR_T - 0.01),          # housing cavity
            C.cyl(49.0, C.Z_BRG - C.Z_TOP + 0.02, C.Z_TOP - 0.01),                       # inside the pin ring
            pocket]
    s = parts[0]
    for p in parts[1:] + fill:
        s = s.fuse(p)
    s = s.clean()
    solids = s.Solids()
    assert len(solids) == 1, f"{len(solids)} pieces"
    outer = BRepClass3d.OuterShell_s(solids[0].wrapped)
    solid = cq.Solid(BRepBuilderAPI_MakeSolid(outer).Solid()).fix()
    assert BRepCheck_Analyzer(solid.wrapped).IsValid(), "not a valid solid"

    # the holes that matter are still open to the outside
    for a in C.LID_SCREW_ANGLES:
        x, y = C.polar(C.LID_SCREW_R, a)
        for z in (-0.5 + 1.0, 10.0, 26.0, 35.0, 43.0):
            assert not solid.isInside(cq.Vector(x, y, z)), f"lid screw hole closed at z {z}"
    for a in C.OUT_SCREW_ANGLES:
        x, y = C.polar(C.OUT_SCREW_R, a)
        assert not solid.isInside(cq.Vector(x, y, 40.0)), "output insert hole closed"
    assert solid.isInside(cq.Vector(0, 0, 20.0)), "inside not filled"
    bb = solid.BoundingBox()
    print(f"one solid, {len(solid.Faces())} faces, {solid.Volume() / 1000:.0f} cm^3; "
          f"x {bb.xmin:.2f}..{bb.xmax:.2f}  z {bb.zmin:.2f}..{bb.zmax:.2f}")

    body, caps = N.load()
    caps = N.trimmed_caps(caps, 0.0)
    for n, m in (("NEMA 17", body), ("end caps", caps)):
        v = m.intersect(solid).Volume()
        if v > 0.05:
            print(f"   OVERLAP motor {n} x housing: {v:.2f} mm^3")
    assy = cq.Assembly(name="cycloid_drive_standin")
    assy.add(solid, name="cycloid drive", color=cq.Color(0.83, 0.55, 0.25))
    assy.add(body, name="NEMA 17", color=cq.Color(0.12, 0.12, 0.13))
    assy.add(caps, name="NEMA 17 end caps", color=cq.Color(0.78, 0.79, 0.81))
    write_step(assy, OUT)
    print(OUT)


if __name__ == "__main__":
    main()

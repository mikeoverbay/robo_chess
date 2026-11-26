"""Gearbox sleeve: a plain tube round the whole fixed gearbox body, 71 ID x 77 OD (Mike 2026-11-22).

His measures of the 70.5 body: 70.44, 70.53, 70.52, the pin ring 70.4 -> 71 ID (0.47 over the biggest), 3 mm wall.
Long enough to cover floor + housing ring + pin ring + lid from z 0 (motor face) to the lid step at z 55.7, where
the lid drops to 57 OD; it stays 3.5 under the turning output disc (z 59.2). Writes print/Gearbox sleeve.step.
    python tools/gearbox_sleeve.py
"""
import os
import cadquery as cq

ID, OD, L = 71.0, 77.0, 55.7
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "print", "Gearbox sleeve.step")

part = cq.Workplane("XY").circle(OD / 2).circle(ID / 2).extrude(L)

if __name__ == "__main__":
    cq.exporters.export(part, OUT)
    bb = part.val().BoundingBox()
    print(f"{os.path.normpath(OUT)}: {bb.xlen:.2f} x {bb.ylen:.2f} x {bb.zlen:.2f}, vol {part.val().Volume():.1f} mm3")

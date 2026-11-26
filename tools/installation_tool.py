"""Installation tool: a plain bushing, 10 OD x 5.5 ID x 14.5 long (Mike 2026-11-22).

Slides over the 5 mm brass tube pins (0.25 a side). Writes print/installation tool.step.
    python tools/installation_tool.py
"""
import os
import cadquery as cq

OD, ID, L = 10.0, 5.5, 14.5
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "print", "installation tool.step")

part = cq.Workplane("XY").circle(OD / 2).circle(ID / 2).extrude(L)

if __name__ == "__main__":
    cq.exporters.export(part, OUT)
    bb = part.val().BoundingBox()
    print(f"{os.path.normpath(OUT)}: {bb.xlen:.2f} x {bb.ylen:.2f} x {bb.zlen:.2f}, vol {part.val().Volume():.1f} mm3")

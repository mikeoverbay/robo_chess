#!/usr/bin/env python3
"""
nema17.py - Mike's NEMA 17 (from his "NEMA 17 with T5 Pulley.STP"), pulley and pulley screw removed, as the motor
for every drive in the arm (2026-11-04: "replace the dummy motors").

    python nema17.py            # writes robo_chess/cycloid/nema17.step from the desktop file
    python nema17.py --length 60    # Mike's J1 motor (2026-11-05: 42 x 60 body, 2 A, 73 N.cm) -> nema17_60.step
    python nema17.py --length 23    # his pancakes (42 x 23, 15.5 N.cm) for J3 and J4; also --length 40

Frame of the written model: shaft axis on z, the mounting (front) face at z 0, the body behind it (z -46..0), the
shaft out of the front face (+z). Two parts so they keep their own colours: "NEMA 17" = the black body stack,
"NEMA 17 end caps" = the aluminium end caps, the 4 long screws, the shaft and its bearings.
Other scripts: load() -> (body, caps); caps(shaft_len=...) trims the shaft to what sticks out in that use.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
SRC = r"C:\Users\theco\OneDrive\Desktop\STEP files\NEMA 17 with T5 Pulley.STP"
OUT = os.path.join(ROOT, "cycloid", "nema17.step")
DROP = ("T5 Pulley", "socket button head cap screw")     # the pulley and its set screw
BODY_L = 48.0            # Mike's motors (17HS19 family): 48 long; the downloaded model is 46 - its body stack is
#                          stretched and the rear cap moved back to match


def stretch_z(shape, z_fixed, factor):
    """scale a shape along z only, about the plane z = z_fixed."""
    from OCP.gp import gp_GTrsf, gp_Mat, gp_XYZ
    from OCP.BRepBuilderAPI import BRepBuilderAPI_GTransform
    g = gp_GTrsf()
    g.SetVectorialPart(gp_Mat(1, 0, 0, 0, 1, 0, 0, 0, factor))
    g.SetTranslationPart(gp_XYZ(0, 0, z_fixed * (1 - factor)))
    return cq.Shape.cast(BRepBuilderAPI_GTransform(shape.wrapped, g, True).Shape())


def out_path(length=BODY_L):
    return OUT if abs(length - BODY_L) < 1e-6 else os.path.join(ROOT, "cycloid", f"nema17_{length:g}.step")


def build(body_l=BODY_L):
    """Mike's way (2026-11-05): slice the motor through the middle of its body and either add body there (the cut
    face extruded) or take a slab out, shifting the rear end to close up. The caps, bearings, screws and the shaft's
    ends keep their real sizes; only the body's length changes. Works for his 23 (pancake), 40, 48 and 60 mm motors.
    Source frame: rear cap "Base" 0..12, body "Cuerpo" 10..34, front cap "Tapa" 32..46, shaft "Eje" 0..68."""
    items = read_step(SRC)
    keep = [(n, s) for n, s, _ in items if not n.startswith(DROP)]
    zmin = min(s.BoundingBox().zmin for n, s in keep if not n.startswith("Eje"))
    zmax = max(s.BoundingBox().zmax for n, s in keep if n.startswith("Tapa"))
    rear_top = max(s.BoundingBox().zmax for n, s in keep if n.startswith("Base"))     # 12
    front_bot = min(s.BoundingBox().zmin for n, s in keep if n.startswith("Tapa"))   # 32
    z_s = (rear_top + front_bot) / 2                                                 # 22: the bare body's middle
    grow = body_l - (zmax - zmin)

    def half(shape, lo, hi):
        piece = shape.intersect(cq.Solid.makeBox(200, 200, hi - lo, V(-100, -100, lo)))
        return piece if piece.Volume() > 1e-6 else None

    if abs(grow) > 0.01:
        out = []
        if grow > 0:                       # add body: the cut face extruded back by grow, the rear end moved back
            for n, s in keep:
                b = s.BoundingBox()
                if b.zmin >= z_s:
                    out.append((n, s))
                    continue
                if b.zmax <= z_s:
                    out.append((n, s.translate(V(0, 0, -grow))))
                    continue
                lo, hi = half(s, b.zmin - 1, z_s), half(s, z_s, b.zmax + 1)
                band = half(s, z_s, z_s + 1.0)                   # the section just above the cut: a prism there
                band = stretch_z(band, z_s, grow).translate(V(0, 0, -grow))   # -> z_s - grow .. z_s
                piece = band
                for q in (hi, lo.translate(V(0, 0, -grow)) if lo else None):
                    if q is not None:
                        piece = piece.fuse(q)
                out.append((n, piece.clean()))
        else:                              # take a slab out of the bare body (into the rear cap if more is needed)
            d = -grow
            free = front_bot - rear_top
            lo_z, hi_z = (z_s - d / 2, z_s + d / 2) if d <= free else (front_bot - d, front_bot)
            for n, s in keep:
                b = s.BoundingBox()
                lo = half(s, b.zmin - 1, lo_z) if b.zmin < lo_z else None
                hi = half(s, hi_z, b.zmax + 1) if b.zmax > hi_z else None
                piece = None
                for q in (hi, lo.translate(V(0, 0, d)) if lo else None):
                    if q is not None:
                        piece = q if piece is None else piece.fuse(q)
                if piece is not None:
                    out.append((n, piece.clean()))
        keep = out
    body_parts = [s for n, s in keep if n.startswith("Cuerpo")]
    cap_parts = [s for n, s in keep if not n.startswith("Cuerpo")]
    caps = cap_parts[0]
    for s in cap_parts[1:]:
        caps = caps.fuse(s)
    body = body_parts[0]
    for s in body_parts[1:]:
        body = body.fuse(s)
    body = body.cut(caps)                     # no overlap where the caps sit into the body stack
    assert len(caps.Solids()) == 1, f"end caps came apart into {len(caps.Solids())} pieces"
    # normalise: axis on z at x = y = 0, front face (top of the front cap, "Tapa") at z 0, body below
    bb = cq.Compound.makeCompound([body, caps]).BoundingBox()
    tapa = [s for n, s in keep if n.startswith("Tapa")][0].BoundingBox()
    shift = V(-(tapa.xmin + tapa.xmax) / 2, -(tapa.ymin + tapa.ymax) / 2, -tapa.zmax)
    body, caps = body.translate(shift).clean(), caps.translate(shift).clean()
    return body, caps


def load(length=BODY_L):
    """(body, caps) from robo_chess/cycloid/nema17.step (or nema17_<length>.step)."""
    d = {n: s for n, s, _ in read_step(out_path(length))}
    return d["NEMA 17"], d["NEMA 17 end caps"]


def trimmed_caps(caps, shaft_len):
    """the caps with the shaft cut to shaft_len out of the front face (0 = flush)."""
    cut = cq.Solid.makeBox(200, 200, 200, V(-100, -100, shaft_len))
    return caps.cut(cut).clean()


def main():
    length = float(sys.argv[sys.argv.index("--length") + 1]) if "--length" in sys.argv else BODY_L
    body, caps = build(length)
    for n, s in (("body", body), ("caps+shaft", caps)):
        b = s.BoundingBox()
        print(f"{n:10s} {s.Volume() / 1000:5.1f} cm^3  x {b.xmin:.1f}..{b.xmax:.1f} y {b.ymin:.1f}..{b.ymax:.1f} z {b.zmin:.1f}..{b.zmax:.1f}")
    a = cq.Assembly(name="nema17")
    a.add(body, name="NEMA 17", color=cq.Color(0.12, 0.12, 0.13))
    a.add(caps, name="NEMA 17 end caps", color=cq.Color(0.78, 0.79, 0.81))
    write_step(a, out_path(length))
    print(out_path(length))


if __name__ == "__main__":
    main()

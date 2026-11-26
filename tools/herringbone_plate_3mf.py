#!/usr/bin/env python3
"""
herringbone_plate_3mf.py - lay the herringbone print parts out on Mike's Ender bed as one 3MF for Cura.

Reads robo_chess/herringbone/print/*.step (already in print orientation, z 0 on the bed), places them inside a
200 x 200 safe zone centred on the bed (Cura: 235 x 230, origin front-left), writes
robo_chess/herringbone/print/herringbone_plate.3mf. Each part is its own object, so Cura can give the spider its
supports and leave the rest without.

    python herringbone_plate_3mf.py
"""
import math
import os
import zipfile
from xml.sax.saxutils import escape

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
PRINT = os.path.join(os.path.dirname(HERE), "herringbone", "print")
OUT = os.path.join(PRINT, "herringbone_plate.3mf")
BED = (235.0, 230.0)          # Cura's machine_width / machine_depth for his Ender-3 Pro
SAFE = 200.0                  # Mike, 2026-11-26: 200 x 200 is the safe zone
CX, CY = BED[0] / 2, BED[1] / 2

# (print file, name in Cura, x, y) - offsets from the bed centre, mm. 2026-11-26 set: the 624-planet change.
LAYOUT = [
    ("spider.step", "spider (output) - SUPPORT", -32.0, 10.0),
    ("input_shaft.step", "input shaft", 30.0, 10.0),
    ("planet_1.step", "planet 1", -32.0, -35.0),
    ("planet_2.step", "planet 2", 0.0, -35.0),
    ("planet_3.step", "planet 3", 32.0, -35.0),
] + [("spacer_head.step", f"head spacer {i + 1}", -18.0 + 12.0 * i, -58.0) for i in range(4)]   # 3 + a spare


def mesh(shape, tol=0.01, ang=0.15):
    """welded triangle mesh of a solid (OCC meshes shared edges once, so face borders weld exactly)"""
    verts, tris = shape.tessellate(tol, ang)
    key, idx, out_v = {}, [], []
    for v in verts:
        k = (round(v.x, 6), round(v.y, 6), round(v.z, 6))
        if k not in key:
            key[k] = len(out_v)
            out_v.append(k)
        idx.append(key[k])
    out_t = [(idx[a], idx[b], idx[c]) for a, b, c in tris]
    out_t = [t for t in out_t if len(set(t)) == 3]
    return out_v, out_t


def closed(tris):
    """every edge used once each way"""
    edges = {}
    for a, b, c in tris:
        for e in ((a, b), (b, c), (c, a)):
            edges[e] = edges.get(e, 0) + 1
    return all(edges.get((b, a), 0) == n == 1 for (a, b), n in edges.items())


def volume(v, tris):
    s = 0.0
    for a, b, c in tris:
        (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = v[a], v[b], v[c]
        s += (x1 * (y2 * z3 - y3 * z2) - y1 * (x2 * z3 - x3 * z2) + z1 * (x2 * y3 - x3 * y2)) / 6
    return s


def main():
    objs, items, report = [], [], []
    for oid, (fn, name, dx, dy) in enumerate(LAYOUT, start=1):
        s = cq.importers.importStep(os.path.join(PRINT, fn)).val()
        bb = s.BoundingBox()
        s = s.translate(cq.Vector(-bb.center.x, -bb.center.y, -bb.zmin))   # own origin: centred, on z 0
        v, t = mesh(s)
        ok, vol = closed(t), volume(v, t)
        report.append((name, fn, CX + dx, CY + dy, bb.xlen, bb.ylen, bb.zlen, len(t), ok, vol, s.Volume()))
        vx = "".join(f'<vertex x="{x:.5f}" y="{y:.5f}" z="{z:.5f}"/>' for x, y, z in v)
        tx = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t)
        objs.append(f'<object id="{oid}" name="{escape(name)}" type="model"><mesh><vertices>{vx}</vertices>'
                    f'<triangles>{tx}</triangles></mesh></object>')
        items.append(f'<item objectid="{oid}" transform="1 0 0 0 1 0 0 0 1 {CX + dx:.4f} {CY + dy:.4f} 0"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             '<metadata name="Title">herringbone print plate</metadata>'
             f'<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
    types = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
             '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
             '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>'
             '</Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", types)
        z.writestr("_rels/.rels", rels)
        z.writestr("3D/3dmodel.model", model)

    x0, x1, y0, y1 = CX - SAFE / 2, CX + SAFE / 2, CY - SAFE / 2, CY + SAFE / 2
    print(f"bed {BED[0]:g} x {BED[1]:g}, safe zone x {x0:g}..{x1:g}, y {y0:g}..{y1:g}")
    boxes = []
    for name, fn, x, y, w, d, h, n, ok, vol, solid_vol in report:
        inside = x0 <= x - w / 2 and x + w / 2 <= x1 and y0 <= y - d / 2 and y + d / 2 <= y1
        boxes.append((name, x - w / 2, x + w / 2, y - d / 2, y + d / 2))
        print(f"  {name:28s} at ({x:6.1f}, {y:6.1f})  {w:5.1f} x {d:5.1f} x {h:5.1f}  {n:6d} tris  "
              f"closed {ok}  mesh {vol / 1e3:.3f} / solid {solid_vol / 1e3:.3f} cm3  {'in' if inside else 'OUTSIDE'}")
    gap = min(max(b2[1] - a[2], a[1] - b2[2], b2[3] - a[4], a[3] - b2[4])
              for i, a in enumerate(boxes) for b2 in boxes[i + 1:])
    print(f"  smallest gap between parts (boxes): {gap:.1f}")
    print("written", OUT, f"{os.path.getsize(OUT) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()

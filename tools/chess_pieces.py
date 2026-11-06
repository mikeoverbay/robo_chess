#!/usr/bin/env python3
"""
NOTE (2026-11-06): Mike had his original STLs (chessboard/pieces/<Piece>_10x2mm.stl, all but the knight) removed -
they are in the Windows Recycle Bin. The STEP pieces in chessboard/pieces/step are now the masters; to run this
again (it measures those STLs), restore them first.
chess_pieces.py - Mike's hexagonal chess pieces (pawn, rook, bishop, queen, king) rebuilt as true CAD (2026-11-05).

    python chess_pieces.py [Pawn Rook ...] [--out DIR]

Mike: "turn these STLs into step files so they actually have edges ... fix some of the geometry errors too ... the
best looking chess pieces money can buy". He has no source CAD; his STLs (chessboard/pieces/<Piece>_10x2mm.stl, left
untouched) have holes, flipped and zero-area triangles, debris. Measured from them (slices every 0.1-0.5 mm), every
piece is built the same way, and this rebuilds it so:
  - every cross-section is a regular hexagon (corners at 0 deg, flats facing +-y) with its vertical corners rounded
    RC = 0.5 mm (the STLs' corner tips sit 0.077 short of sharp: exactly a 0.5 radius on a 120 deg corner)
  - base: a doubled 30 deg chamfered ring - apothem a0 at z 0, a0 + 3 tan30 at 3, a0 at 6, a0 + 2 tan30 at 8;
    a0 = 11.0 (pawn, rook, bishop) or 12.5 (queen, king)
  - body (z 8 up to its top): a hexagon that shrinks along a smooth curve and twists one facet (-60 deg) - MEASURED from
    the STL every 0.5 mm (apothem + corner angle), smoothed, lofted through sections every 1 mm (smooth B-spline faces,
    the six helical corner rounds between them)
  - head: straight-sided zones (45 / 30 / 15 / 10 deg slopes, round numbers) - see SPEC
  - each piece's feature (FEATURES): rook - a 45 deg hexagonal cup and 6 battlement notches 4 wide; bishop - the mitre
    slot, a 2 mm slab leaning 7.4 deg, and the knob; queen - the crown's V ridge, recess and centre spike; king - the
    hexagonal ring standing on edge (4 thick, hexagonal hole)
  - the sealed magnet cavity (a 10 x 2 magnet, dropped in on a print pause): a true cylinder D11, z 0.5..2.6 (2.7 bishop)
Each STEP is one valid solid, checked against the STL (python chess_pieces.py --check): every STL surface point within
~0.15 mm of the CAD.
Out: chessboard/pieces/step/<Piece>.step
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
PIECES = os.path.join(ROOT, "chessboard", "pieces")
OUT = os.path.join(PIECES, "step")
T30, T15, T10, T60 = (math.tan(math.radians(d)) for d in (30, 15, 10, 60))
COS30 = math.cos(math.radians(30))
RC = 0.5                       # vertical corner rounding

# ---------------------------------------------------------------- the pieces: base, body top, head zones (z, apothem)
SPEC = {
    "Pawn": dict(a0=11.0, body_top=34.0, a_top=6.0,
                 head=[(34, 6.0), (36, 4.0), (40, 6.8), (47, 6.8), (50, 3.8)]),
    "Rook": dict(a0=11.0, body_top=41.0, a_top=7.0,
                 head=[(41, 7.0), (45, 11.0), (55, 11.0)]),
    "Bishop": dict(a0=11.0, body_top=44.0, a_top=7.0,
                   head=[(44, 7.0), (45.5, 8.5), (50, 4.0), (53, 7.0), (56, 7.8), (59, 7.8), (66, 7.8 - 7 * T30),
                         (68, 5.8 - 7 * T30), (69, 5.8 - 7 * T30), (70, 4.8 - 7 * T30)]),
    "Queen": dict(a0=12.5, body_top=58.0, a_top=7.0,
                  head=[(58, 7.0), (60, 7 + 2 * T30), (63, 7 - T30), (71, 7 - T30 + 8 * T10),
                        (74, 7 + 2 * T30 + 8 * T10), (75, 7 + 2 * T30 + 8 * T10), (77, 7 + 2 * T30 + 8 * T10 - 2 * T60)]),
    "King": dict(a0=12.5, body_top=62.0, a_top=7.0,
                 head=[(62, 7.0), (64, 9.0), (67, 6.0), (77, 6 + 10 * T15), (79, 6 + 10 * T15 - 2 * T30),
                       (80, 5 + 10 * T15 - 2 * T30)]),
}
CAVITY = {"Bishop": (0.5, 2.7)}          # z span of the magnet cavity (default 0.5 .. 2.6), D11


# ---------------------------------------------------------------- hexagon sections
def rhex(a, phi_deg, z, rc=RC):
    """a regular hexagon (apothem a, a corner at phi_deg) at height z, its corners rounded rc: 6 lines + 6 arcs"""
    rc = min(rc, 0.3 * a)
    R = a / COS30
    edges, pts = [], []
    for k in range(6):
        c = math.radians(phi_deg + 60 * k)
        cx, cy = math.cos(c), math.sin(c)
        # the arc's centre sits rc / sin60 in from the corner; its tangent points are rc tan30 * 2 / sqrt3 along each side
        d = rc / math.sin(math.radians(60))
        oc = (R - d) * cx, (R - d) * cy
        t = []
        for s in (-1, 1):                                # tangent points on the two sides meeting at this corner
            n = math.radians(phi_deg + 60 * k + s * 30)   # the side's outward normal
            t.append(V(oc[0] + rc * math.cos(n), oc[1] + rc * math.sin(n), z))
        mid = V(oc[0] + rc * cx, oc[1] + rc * cy, z)
        pts.append((t[0], mid, t[1]))
    for k in range(6):
        a0, m, a1 = pts[k]
        edges.append(cq.Edge.makeThreePointArc(a0, m, a1))
        edges.append(cq.Edge.makeLine(a1, pts[(k + 1) % 6][0]))
    return cq.Wire.assembleEdges(edges)


def loft(sections, ruled, rc=RC):
    return cq.Solid.makeLoft([rhex(a, p, z, rc) for z, a, p in sections], ruled)


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} solids"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


# ---------------------------------------------------------------- the body, measured from Mike's STL
def stl_shells(piece):
    import trimesh
    m = trimesh.load(os.path.join(PIECES, f"{piece}_10x2mm.stl"), force="mesh")
    s = sorted(m.split(only_watertight=False), key=lambda q: -len(q.faces))
    return s[0], s[1]


def measure_body(piece):
    """(z, apothem, corner angle) every 0.5 mm up the twisted body, from slices of the STL (off its vertex rows;
    corner angle from the outermost points' 6-fold moment, unwrapped; apothem = the median flat distance)"""
    import trimesh
    m, _ = stl_shells(piece)
    sp = SPEC[piece]
    rows_z = np.unique(np.round(m.vertices[:, 2], 4))
    out, prev = [], 0.0
    for z in np.arange(8.0, sp["body_top"] + 1e-9, 0.5):
        z = min(max(z, 8.03), sp["body_top"] - 0.03)
        j = np.searchsorted(rows_z, z)
        if any(0 <= i < len(rows_z) and abs(rows_z[i] - z) < 0.006 for i in (j - 1, j)):
            z += 0.011
        segs = trimesh.intersections.mesh_plane(m, [0, 0, 1], [0, 0, z])[:, :, :2]
        L = np.linalg.norm(segs[:, 1] - segs[:, 0], axis=1)
        n = np.maximum(1, np.ceil(L / 0.02).astype(int))
        p = np.vstack([segs[i, 0] + (segs[i, 1] - segs[i, 0]) * (np.arange(n[i] + 1)[:, None] / n[i])
                       for i in range(len(segs))])
        r, th = np.hypot(p[:, 0], p[:, 1]), np.arctan2(p[:, 1], p[:, 0])
        w = (r / r.max()) ** 24
        phi = (np.degrees(np.angle((w * np.exp(6j * th)).sum())) / 6) % 60
        phi = phi + 60 * round((prev - phi) / 60)
        prev = phi
        ang = np.radians(phi + 30 + 60 * np.arange(6))
        h = (p @ np.stack([np.cos(ang), np.sin(ang)])).max(1)
        out.append((z, np.median(h[h > h.max() - 1.0]), phi))
    return np.array(out)


def body_sections(piece, step=1.0):
    """smoothed body sections every step mm, the ends snapped to the base's top and the head's bottom"""
    from scipy.interpolate import UnivariateSpline
    sp = SPEC[piece]
    B = measure_body(piece)
    z, a, phi = B[:, 0], B[:, 1], B[:, 2]
    phi = phi - phi[0]                                   # 0 at the base (a corner at 0 deg) ...
    if phi[-1] > 0:
        phi = phi - 60.0                                 # ... -60 at the top, wherever the measure started
    z[0], z[-1] = 8.0, sp["body_top"]
    fa = UnivariateSpline(z, a, k=3, s=len(z) * 0.004 ** 2)
    fp = UnivariateSpline(z, phi, k=3, s=len(z) * 0.08 ** 2)
    zs = np.linspace(8.0, sp["body_top"], int(round((sp["body_top"] - 8.0) / step)) + 1)
    A, P = fa(zs), fp(zs)
    A[0], A[-1] = sp["a0"] + 2 * T30, sp["a_top"]
    P[0], P[-1] = 0.0, -60.0
    return list(zip(zs, A, P))


# ---------------------------------------------------------------- the pieces' features
def fillet_edges(solid, pick, r=RC, what=""):
    """round the edges pick(edge) -> True; returns the solid unchanged if OCC can't"""
    edges = [e for e in solid.Edges() if pick(e)]
    if not edges:
        return solid
    try:
        f = one(solid.fillet(r, edges), what)
        print(f"   {what}: {len(edges)} edges rounded r {r}")
        return f
    except Exception as ex:                              # noqa: BLE001
        print(f"   {what}: rounding {len(edges)} edges FAILED ({type(ex).__name__}) - left sharp")
        return solid


def rook_top(s):
    """a 45 deg hexagonal cup (apothem 4 at z 47 -> 8 at 51, then straight up) and 6 notches 4 wide over the flats,
    floor z 51; the notches' vertical edges rounded like the corners"""
    cup = loft([(47.0, 4.0, 0), (51.0, 8.0, 0), (56.0, 8.0, 0)], True)
    s = s.cut(cup)
    for k in range(6):                                   # over each flat: its normal at 30 + 60k deg
        slot = cq.Solid.makeBox(8.0, 4.0, 6.0, V(6.0, -2.0, 51.0)).rotate(V(0, 0, 0), V(0, 0, 1), 30 + 60 * k)
        s = s.cut(slot)
    s = one(s.clean(), "rook")

    def notch_edge(e):                                   # vertical, z 51..55, on a notch wall (2.0 off a flat's axis)
        a, b = e.startPoint(), e.endPoint()
        if not (abs(a.x - b.x) < 1e-6 and abs(a.y - b.y) < 1e-6 and min(a.z, b.z) > 50.9 and max(a.z, b.z) > 54.9):
            return False
        th = math.radians(30 + 60 * round((math.degrees(math.atan2(a.y, a.x)) - 30) / 60))
        return abs(abs(-a.x * math.sin(th) + a.y * math.cos(th)) - 2.0) < 0.01
    return fillet_edges(s, notch_edge, what="rook notch edges")


SLOT_TILT, SLOT_W = 7.4, 2.0
SLOT_REF = (1.505, 57.51)            # a point on the slot's centre plane (x, z). The STL's is 1.655, where the walls
#                                      graze the knob's corner and the z 66 corners (faces down to 0.05 mm^2); scanned
#                                      -0.3..+0.3 in 0.05: -0.15 is the only spot leaving nothing under 0.2 mm^2 / 0.2 mm
SLOT_FLOOR = 1.53 + 0.40 / 0.9917    # the floor, this far below SLOT_REF along the slot's axis: 0.4 deeper than the
#                                      STL's, whose floor ends straddled the z 56 crease by 0.14 / 0.12 - sliver faces
SLOT_V = (10.0, 6.18)                # the floor dips in a V along its middle: 10 deg sides, flat beyond |y| 6.18


def bishop_slot(s):
    """the mitre slot: a 2 mm slab leaning SLOT_TILT, right across the head (along y); its floor square to the walls,
    with a shallow V (SLOT_V) along the middle, 1.09 deep - as the STL has it"""
    t = math.radians(SLOT_TILT)
    up = V(math.sin(t), 0, math.cos(t))                  # the slot's axis, leaning toward +x going up
    fx, fz = SLOT_REF[0] - SLOT_FLOOR * up.x, SLOT_REF[1] - SLOT_FLOOR * up.z
    dv = SLOT_V[1] * math.tan(math.radians(SLOT_V[0]))
    prof = [(-20.0, 0.0), (-SLOT_V[1], 0.0), (0.0, -dv), (SLOT_V[1], 0.0), (20.0, 0.0), (20.0, 30.0), (-20.0, 30.0)]
    slab = (cq.Workplane("YZ", origin=(-SLOT_W / 2, 0, 0)).polyline(prof).close().extrude(SLOT_W).val())
    slab = slab.rotate(V(0, 0, 0), V(0, 1, 0), SLOT_TILT).translate(V(fx, 0, fz))
    s = one(s.cut(slab).clean(), "bishop")
    n = V(math.cos(t), 0, -math.sin(t))                  # the slot walls' normal

    def slot_edge(e):                                    # the walls' edges with the outside (not the floor's)
        c = e.Center()
        dn = abs((c.x - fx) * n.x + (c.z - fz) * n.z)
        return abs(dn - SLOT_W / 2) < 0.02 and abs(c.y) > 1.0 and e.geomType() != "CIRCLE"
    return fillet_edges(s, slot_edge, what="bishop slot edges")


def queen_crown(s):
    """the crown: its outside rises at 60 deg to a V ridge (z 77), the inside drops at 30 deg from vertical to a floor
    at z 76, and the centre spike (60 deg) stands from the floor to z 79"""
    a_ridge = SPEC["Queen"]["head"][-1][1]
    recess = loft([(76.0, a_ridge - T30, 0), (77.0, a_ridge, 0), (78.0, a_ridge + T30, 0)], True)
    s = s.cut(recess)
    spike = loft([(76.0, 3.0225, 0), (79.0, 3.0225 - 3 * T30, 0)], True)
    return one(s.fuse(spike).clean(), "queen")


def king_ring(s):
    """the ornament: a hexagon (flats at z 82 and 92, points at x +-5.774) with a hexagonal hole (apothem 2), 4 thick,
    on a 60 deg foot down to the head's top (z 80)"""
    o = [(-4.0415, 80.0), (4.0415, 80.0), (2.8868, 82.0), (5.7735, 87.0), (2.8868, 92.0), (-2.8868, 92.0),
         (-5.7735, 87.0), (-2.8868, 82.0)]
    h = [(2.3094, 87.0), (1.1547, 89.0), (-1.1547, 89.0), (-2.3094, 87.0), (-1.1547, 85.0), (1.1547, 85.0)]
    ring = cq.Workplane("XZ").polyline(o).close().extrude(2.0, both=True).val()
    hole = cq.Workplane("XZ").polyline(h).close().extrude(3.0, both=True).val()
    ring = ring.translate(V(0, 0, -0.01)).cut(hole)      # 0.01 into the head: a sure fuse
    return one(s.fuse(ring).clean(), "king")


FEATURES = {"Rook": rook_top, "Bishop": bishop_slot, "Queen": queen_crown, "King": king_ring}


# ---------------------------------------------------------------- build
def build(piece):
    sp = SPEC[piece]
    a0 = sp["a0"]
    base = loft([(0, a0, 0), (3, a0 + 3 * T30, 0), (6, a0, 0), (8, a0 + 2 * T30, 0)], True)
    body = loft(body_sections(piece), False)
    head = loft([(z, a, 0) for z, a in sp["head"]], True)
    s = one(base.fuse(body).fuse(head).clean(), piece)
    if piece in FEATURES:
        s = FEATURES[piece](s)
    z0, z1 = CAVITY.get(piece, (0.5, 2.6))
    s = one(s.cut(cq.Solid.makeCylinder(5.5, z1 - z0, V(0, 0, z0))).clean(), piece)
    return s


def check(piece, s):
    """STL -> CAD distance (every point of Mike's surface to the CAD's), dense samples + KD-tree"""
    import trimesh
    from scipy.spatial import cKDTree
    body, mag = stl_shells(piece)
    stl = trimesh.util.concatenate([body, mag])
    v, t = s.tessellate(0.005, 0.05)
    cad = trimesh.Trimesh(np.array([p.toTuple() for p in v]), np.array(t))
    a = np.vstack([stl.vertices, trimesh.sample.sample_surface_even(stl, 300000, seed=4)[0]])
    b = np.vstack([cad.vertices, trimesh.sample.sample_surface_even(cad, 500000, seed=5)[0]])
    d = cKDTree(b).query(a)[0]
    i = int(d.argmax())
    return d.max(), np.percentile(d, 99), d.mean(), a[i]


def main():
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    out = OUT
    if "--out" in sys.argv:
        out = sys.argv[sys.argv.index("--out") + 1]
        args = [x for x in args if x != out]
    pieces = args or list(SPEC)
    os.makedirs(out, exist_ok=True)
    for p in pieces:
        s = build(p)
        bb = s.BoundingBox()
        print(f"{p}: valid, {len(s.Faces())} faces, {len(s.Edges())} edges, {s.Volume() / 1000:.2f} cm^3, "
              f"{bb.xlen:.2f} x {bb.ylen:.2f} x {bb.zlen:.2f}")
        if "--check" in sys.argv:
            mx, p99, mean, at = check(p, s)
            print(f"   vs the STL: max {mx:.3f}, 99% {p99:.3f}, mean {mean:.4f} mm (worst at z {at[2]:.1f}, "
                  f"r {math.hypot(at[0], at[1]):.1f})")
        a = cq.Assembly(name=f"{p.lower()}_piece")
        a.add(s, name=p.lower(), color=cq.Color(0.93, 0.89, 0.82))
        path = os.path.join(out, f"{p}.step")
        write_step(a, path)
        print("  ", path)


if __name__ == "__main__":
    main()

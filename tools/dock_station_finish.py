#!/usr/bin/env python3
"""
dock_station_finish.py - finishing touches on Mike's one-piece dock station (2026-11-09: his
park_dock/dock_station.step, "combined into one piece. It's tall. I need to have some holes put around the outside of
it. And I want to put a radius on everything ... Break every corner by 2 millimeters. Except for on the very bottom
and where the cup where it needs to be a cup").

    python dock_station_finish.py

His file is read, never written. The station is its largest body (frame: Robot Studio's, z up, wood at z -257).
  1. the 0.2 mm sleeve left inside it from merging the old post into its socket (post D24 in a D24.4 bore, the full
     height) is filled, so the post prints solid with the tower; the wire bore and his two wire holes stay open
  2. every edge gets r 2 except the bottom face's, the cup's (seat recess, floor, lead-ins, the hole under the floor,
     the Hall pocket, its lead slot and the groove under the fork) and the small holes' (wire bore, wire holes)
  3. hold-down holes: one #8 wood screw in each of the four flat pads between the ribs (his picks), countersunk from
     the top
Out: park_dock/dock_station_v2.step (the station alone, to print) and park_dock/dock_station_assembly_v2.step (his
file with the station swapped for the finished one). Checks: the parked claw, the drop and the slide against it.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE  # noqa: E402
from OCP.TopExp import TopExp  # noqa: E402
from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402

import park_dock as PD  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402
from view_step import read_step  # noqa: E402

SRC = os.path.join(PD.OUT, "dock_station.step")
WOOD_Z = -257.0
CUP = (-115.38, 106.90)                 # the seat's axis (world x, y) in HIS file = park_dock's parked claw until 2026-11-24
# Where it is screwed down: DOCK_OUT further out than in his file, straight out along J0 PSI (2026-11-24, Mike: "We
# have an issue with the park stand. It's colliding with the new base." / "Move the stand wherever it will work." -
# park_dock.py RHO has the numbers). The station's shape and the print file are unchanged; only where it stands moves.
DOCK_OUT = PD.RHO - 179.5
DOCK_SHIFT = (DOCK_OUT * math.cos(math.radians(PD.PSI)), DOCK_OUT * math.sin(math.radians(PD.PSI)), 0.0)   # world
CUP_R, CUP_Z = 42.5, -77.0              # a face entirely inside r 42.5 of it, above z -77, is the cup's
POST = (-159.8, 69.6)
R = 2.0
HOLD_IN, HOLD_CLEAR, HOLD_CSK = 10.0, 4.5, 9.0   # corner holes 10 in from the base plate's edges; #8 wood screws


def world_to_hub():
    """Robot Studio frame -> the parked claw's hub frame (inverse of park_dock's export transform)"""
    w = gp_Trsf()
    w.SetValues(-1, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0)
    return w.Multiplied(PD.hub_to_design()).Inverted()


def fill_sleeve(st):
    """the 0.2 sleeve round the post (r 12..12.2, the post's full height) and the 0.5 over its top -> solid; the bore
    (r 7) and the holes through the post wall stay"""
    # A solid filler r 7.05..12.25 fused in, every face of it a hair off his (coincident faces made OCC's booleans
    # return nothing), then each small hole that crossed the post wall drilled again over its own length only.
    fill = cq.Solid.makeCylinder(12.25, 188.4, V(*POST, -252.95), V(0, 0, 1)).cut(
        cq.Solid.makeCylinder(7.05, 200.0, V(*POST, -260.0), V(0, 0, 1)))
    zone = cq.Solid.makeCylinder(12.4, 200.0, V(*POST, -260.0), V(0, 0, 1))
    groups = {}
    for f in st.Faces():
        if f.geomType() == "CYLINDER":
            c = BRepAdaptor_Surface(f.wrapped).Cylinder()
            if c.Radius() < 6.0:
                a, p = c.Axis().Direction(), c.Location()
                k = (round(c.Radius(), 3), round(a.X(), 3), round(a.Y(), 3), round(a.Z(), 3))
                g = groups.setdefault(k, {"r": c.Radius(), "a": (a.X(), a.Y(), a.Z()), "p": (p.X(), p.Y(), p.Z()), "t": []})
                for v in f.Vertices():
                    q = v.toTuple()
                    g["t"].append(sum((q[i] - g["p"][i]) * g["a"][i] for i in range(3)))
    holes = []
    for g in groups.values():
        t0, t1 = min(g["t"]) - 0.5, max(g["t"]) + 0.5
        o = [g["p"][i] + t0 * g["a"][i] for i in range(3)]
        holes.append(cq.Solid.makeCylinder(g["r"], t1 - t0, V(*o), V(*g["a"])))
    holes.append(cq.Solid.makeCone(13.01, 7.0, 6.0, V(*POST, WOOD_Z - 0.001), V(0, 0, 1)))   # the bore's bottom flare
    v0 = st.Volume()
    out = st.fuse(fill)
    for h in holes:
        out = out.cut(h.intersect(zone))
    out = PD.one(out.clean())
    print(f"sleeve filled: +{out.Volume() - v0:.0f} mm^3; {len(holes)} hole(s) through the post wall kept open")
    return out


def face_kind(f):
    t = f.geomType()
    vs = [v.toTuple() for v in f.Vertices()] or [f.Center().toTuple()]
    rmax = max(math.hypot(x - CUP[0], y - CUP[1]) for x, y, z in vs)
    zmin = min(z for x, y, z in vs)
    if t == "CYLINDER":
        c = BRepAdaptor_Surface(f.wrapped).Cylinder()
        if c.Radius() <= 7.01:
            return "hole"
    b = f.BoundingBox()
    if t == "CONE" and b.zmin < WOOD_Z + 0.01:
        return "hole"                                          # the bore's flare at the bottom
    flat_bottom = t == "PLANE" and abs(b.zmax - b.zmin) < 0.01 and abs(b.zmin - (CUP_Z + 0.5)) < 0.02
    if b.zmin >= CUP_Z + 0.4 and b.zmax <= CUP_Z + 2.6 and not flat_bottom:
        return "cup"                                           # the lead groove under the fork (2 deep), all of it
    if rmax <= CUP_R and zmin >= CUP_Z:
        return "cup"
    return "other"


def edge_faces(shape):
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape.wrapped, TopAbs_EDGE, TopAbs_FACE, m)
    return m


def candidates(st):
    kinds = {f.hashCode(): face_kind(f) for f in st.Faces()}
    m = edge_faces(st)
    out, skipped = [], {"bottom": 0, "cup": 0, "hole": 0}
    for e in st.Edges():
        vs = [v.toTuple() for v in e.Vertices()]
        if all(z < WOOD_Z + 0.01 for x, y, z in vs) and e.Center().z < WOOD_Z + 0.01:
            skipped["bottom"] += 1
            continue
        fl = m.FindFromKey(e.wrapped)
        ks = {kinds.get(cq.Face(f).hashCode(), "other") for f in fl}
        if "cup" in ks:
            skipped["cup"] += 1
        elif "hole" in ks:
            skipped["hole"] += 1
        else:
            out.append(e)
    return out, skipped


def fillet(st, edges, r=R):
    try:
        mk = BRepFilletAPI_MakeFillet(st.wrapped)
        for e in edges:
            mk.Add(r, e.wrapped)
        mk.Build()
        if not mk.IsDone():
            return None
        s = cq.Shape.cast(mk.Shape())
    except Exception:
        return None
    if not s.isValid() or len(s.Solids()) != 1:
        return None
    return s.Solids()[0]


def is_sharp(e, m):
    """a real corner: its two faces meet at more than 8 deg (a fillet's own edges are tangent - leave them)"""
    fl = [cq.Face(f) for f in m.FindFromKey(e.wrapped)]
    if len(fl) != 2:
        return False
    p = e.positionAt(0.5)
    try:
        return fl[0].normalAt(p).dot(fl[1].normalAt(p)) < math.cos(math.radians(8.0))
    except Exception:
        return True


def vertical(e):
    return abs(e.tangentAt(0.5).z) > 0.9


def stage(st, pick, r):
    """round every sharp candidate edge that `pick` takes: all at once if OCC can, else grown edge by edge"""
    edges, _ = candidates(st)
    m = edge_faces(st)
    edges = [e for e in edges if is_sharp(e, m) and pick(e)]
    if not edges:
        return st, 0, []
    s = fillet(st, edges, r)
    if s is not None:
        return s, len(edges), []
    good, bad = [], []
    for e in edges:
        if fillet(st, good + [e], r) is not None:
            good.append(e)
        else:
            bad.append(e)
    return (fillet(st, good, r) if good else st), len(good), bad


def sequential(st, r, tried=None):
    """one edge at a time on the shape as it changes (OCC blends a corner where three rounds meet far better that
    way than all at once): vertical corners first; an edge that fails is tried again only once its shape changed"""
    tried = set() if tried is None else tried
    n = 0
    while True:
        edges, _ = candidates(st)
        m = edge_faces(st)
        edges = sorted((e for e in edges if is_sharp(e, m)), key=lambda e: (not vertical(e), -e.Length()))
        done = False
        for e in edges:
            c = e.Center()
            key = (round(c.x, 1), round(c.y, 1), round(c.z, 1), round(e.Length(), 1))
            if key in tried:
                continue
            tried.add(key)
            s = fillet(st, [e], r)
            if s is not None:
                st, n, done = s, n + 1, True
                break
        if not done:
            return st, n


def hold_down(st):
    """One #8 in each of the four flat pads between the ribs, 12 above the wood (2026-11-09: his picks in Robot Studio,
    P262.F16/F43/F64/F71; the foot is round, D100 - the first try assumed a square plate and cut its corners in thin air).
    Each hole goes where its pad has the most room (largest circle inside the pad's outline), countersunk from the top."""
    pads = [f for f in st.Faces() if f.geomType() == "PLANE" and f.normalAt().z > 0.99
            and abs(f.Center().z - (WOOD_Z + 12.0)) < 0.05 and f.Area() > 1000.0]
    assert len(pads) == 4, f"{len(pads)} pads at 12 above the wood"
    k = (HOLD_CSK - HOLD_CLEAR) / 2
    pts = []
    for f in pads:
        top, b, edges = f.Center().z, f.BoundingBox(), f.Edges()
        best = (-1.0, None)
        step = 1.0
        for i in range(int((b.xmax - b.xmin) / step) + 1):
            for j in range(int((b.ymax - b.ymin) / step) + 1):
                v = cq.Vertex.makeVertex(b.xmin + i * step, b.ymin + j * step, top)
                if f.distance(v) > 1e-6:
                    continue                                   # off the pad
                d = min(e.distance(v) for e in edges)
                if d > best[0]:
                    best = (d, (v.X, v.Y))
        room, (x, y) = best
        over = st.intersect(cq.Solid.makeCylinder(6.0, 250.0, V(x, y, top + 0.5), V(0, 0, 1)))   # a driver's way in
        pts.append((x, y, room, min((s.BoundingBox().zmin - top for s in over.Solids()), default=None)
                    if over.Volume() > 1e-3 else None))
        st = st.cut(cq.Solid.makeCylinder(HOLD_CLEAR / 2, 14.0, V(x, y, WOOD_Z - 1), V(0, 0, 1)))
        st = st.cut(cq.Solid.makeCone(HOLD_CLEAR / 2, HOLD_CSK / 2 + 1.0, k + 1.0, V(x, y, top - k), V(0, 0, 1)))
    return PD.one(st.clean()), pts


def main():
    src = read_step(SRC)
    def is_station(s):                    # round the post, from the wood up to the fork (the swivel housing is bigger)
        b = s.BoundingBox()
        return b.xmin < POST[0] < b.xmax and b.ymin < POST[1] < b.ymax and b.zmin < WOOD_Z + 0.01 and b.zmax > -60.0
    i_st = max((i for i in range(len(src)) if is_station(src[i][1])), key=lambda i: src[i][1].Volume())
    st0 = src[i_st][1]
    print(f"station: {st0.Volume():.0f} mm^3, {len(st0.Faces())} faces, {len(st0.Edges())} edges")
    st = fill_sleeve(st0)
    edges, skipped = candidates(st)
    print(f"edges to round: {len(edges)}; left sharp: {skipped}")
    # in stages (OCC rounds the outlines once their vertical corners are round; all at once it refused 71 of 131):
    # the vertical corners, then the rest at r 2, then r 1 on whatever won't take 2
    st_r, bad = st, []
    # OCC could not round his Shapr3D blends (all at once 60 of 131, staged 57, one at a time 25; sliver edges
    # 0.3..2.4 long where the fork meets the post boss and at the ribs' feet): the rounds are left to Shapr3D unless
    # --round is given; the file then carries only the sleeve fill and the hold-down holes
    for r in ((R, 1.0) if "--round" in sys.argv else ()):
        st_r, n = sequential(st_r, r)
        print(f"   r {r:g}: {n} edges, one at a time", flush=True)
    edges, _ = candidates(st_r)
    m = edge_faces(st_r)
    bad = [e for e in edges if is_sharp(e, m)]
    print(f"   still sharp: {len(bad)}")
    if bad:
        print("   left sharp: " + ", ".join(f"({e.Center().x:.1f}, {e.Center().y:.1f}, {e.Center().z:.1f}) "
                                           f"{e.Length():.1f} long" for e in bad[:16]))
    v_before = st_r.Volume()
    st_f, pts = hold_down(st_r)
    print(f"#8 holes in the 4 pads ({v_before - st_f.Volume():.0f} mm^3 cut):")
    for x, y, room, clear in pts:
        print(f"   ({x:.1f}, {y:.1f}), {math.degrees(math.atan2(y - POST[1], x - POST[0])) % 360:.0f} deg / "
              f"{math.hypot(x - POST[0], y - POST[1]):.1f} from the post: pad edge {room:.1f} from the centre "
              f"(countersink r {HOLD_CSK / 2:g}); "
              + ("nothing above it within r 6 - a straight driver reaches it" if clear is None
                 else f"solid above it within r 6 from {clear:.0f} over the pad"))
    for n, s, _ in src:                                          # anything of his it now touches
        if s is st0:
            continue
        v = st_f.intersect(s).Volume()
        if v > 1e-3:
            print(f"   OVERLAP with {n} ({s.Volume():.0f} mm^3 body): {v:.2f} mm^3")
    # the parked claw against it, and the moves in and out (park_dock's checks, in the hub frame)
    sh = PD.placed(st_f.translate(V(*DOCK_SHIFT)), world_to_hub())          # where it stands (DOCK_OUT)
    claw = [(n, s) for n, s in PD.claw_parts_hub_frame() if n != "hub"] + [("flanged hub", PD.hub_flanged()),
                                                                           ("magnet", PD.magnet())]
    worst = {}
    for kind, steps in (("parked", [(0.0, 0.0)]), ("drop", [(PD.LIFT * k / 11, 0.0) for k in range(1, 12)]),
                        ("slide", [(PD.LIFT, -(PD.PLATE_R + 34.0) * k / 20) for k in range(21)])):
        for lift, dx in steps:
            for n, s in claw:
                m = s.translate(V(dx, 0, lift))
                v = m.intersect(sh).Volume()
                g = -v if v > 1e-3 else m.distance(sh)
                if kind == "parked" and n == "flanged hub" and v < 1e-3:
                    continue
                if kind not in worst or g < worst[kind][0]:
                    worst[kind] = (g, n, lift, dx)
    for k, (g, n, lift, dx) in worst.items():
        print(f"   {k}: {'OVERLAP %.3f mm^3' % -g if g < 0 else 'closest %.2f mm' % g} ({n}, lift {lift:.1f}, {-dx:.1f} out)")
    print(f"station v2: {st_f.Volume():.0f} mm^3 ({st_f.Volume() * 1.27e-3:.0f} g solid PETG), valid {st_f.isValid()}")

    col = lambda h: cq.Color(*(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)))  # noqa: E731
    write_flat([("dock station", st_f, col("#e0a030"))], os.path.join(PD.OUT, "dock_station_v2.step"))
    parts = []
    for k, (n, s, c) in enumerate(src):
        cc = cq.Color(*c) if c else None
        parts.append(("dock station", st_f, col("#e0a030")) if k == i_st else (n, s, cc))
    write_flat(parts, os.path.join(PD.OUT, "dock_station_assembly_v2.step"))
    print(os.path.join(PD.OUT, "dock_station_v2.step"))
    print(os.path.join(PD.OUT, "dock_station_assembly_v2.step"))


def to_scene():
    """Robot Studio shows the finished station (2026-11-09: "pull it up on the display so I can look at it"): a copy
    turned into the scene's design frame (y down) in viewer_session/, in place of park_dock.py's multi-part dock"""
    import json
    (n, s, c), = read_step(os.path.join(PD.OUT, "dock_station_v2.step"))
    w = gp_Trsf()
    w.SetValues(-1, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, 0)          # Robot Studio <-> design: its own inverse
    out = os.path.join(os.path.dirname(PD.SCENE), "dock_station_v2_scene.step")
    write_flat([("dock station v2", PD.placed(s.translate(V(*DOCK_SHIFT)), w), cq.Color(*c) if c else None)], out)
    sc = json.load(open(PD.SCENE, encoding="utf-8"))
    entry = {"path": out, "name": "dock station v2", "by": "dock_station_finish.py --scene"}
    names = [m.get("name") for m in sc["models"]]
    if "dock station v2" in names:                               # in its place (the parts after it keep their numbers)
        sc["models"][names.index("dock station v2")] = entry
    else:
        sc["models"] = [m for m in sc["models"] if m.get("name") != "park dock"] + [entry]
    PD.write_scene(sc)
    print(f"scene: 'dock station v2' {DOCK_OUT:g} out from his file's place ({out})")


if __name__ == "__main__":
    to_scene() if "--scene" in sys.argv else main()

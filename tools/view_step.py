#!/usr/bin/env python3
"""
view_step.py - Robot Studio (Mike's model viewer and robot sandbox): look at models, cut sections, pick faces and parts, and see Claude's changes and
notes, in a Chrome window, before taking anything into Shapr3D.

LIVE APP (what Mike uses):
    python view_step.py --serve [FILE.step ...]          (port 8765; opens a Chrome window)
  - Open STEP... (or drag files in) adds models; x closes one. STEP/.stp is the format; STL also opens.
  - click = select a face, double-click = select a part, Ctrl adds/removes, Esc clears. Every pick gets a label
    (P2 = part 2, P2.F14 = face 14 of part 2) in the window and in the side panel.
  - The selection is written to viewer_session/selection.json every time it changes (Claude reads it).
  - The side panel (drag its edge to resize) holds the section, view, parts, selection and Claude's notes.
  - Claude talks back through viewer_session/: scene.json (the models shown), notes.md (the notes panel; P2.F14 in
    the text becomes a link), show.json (faces to highlight, labels to place, a view to jump to). The window
    follows every change within a second, and reloads a model whenever its file changes on disk.

SEVERAL STUDIOS AT ONCE (Mike 2026-11-06: "You run your version, it runs its version ... Think of it as multi-user
interface"): python view_step.py --serve --studio NAME [--restart] runs studio NAME (tools/studios.json: its port
and its own folder for scene / show / notes / selection; see studio.py). The menu by the title switches the window
between studios and starts one that isn't running. Without --studio: ROBOT_STUDIO, else main (8765, viewer_session).

STATIC PAGE / PICTURE (what Claude uses from the command line):
    python view_step.py FILE.step [...] [--view front] [--cut y=0] [--keep +|-] [--hide 2 ...] [--only 1 ...]
                        [--at x,y,z] [--zoom 2] [--no-edges] [--png pic.png] [--size 1600x1000] [--out page.html] [--open]
  Writes a self-contained page (tools/views/<name>.html); --png saves a picture of that view with headless Chrome.
  The same settings work after '#' in any page address:  ...html#view=front&cut=y:0&hide=2&zoom=2&ui=0

Pages load three.js from cdn.jsdelivr.net, so they need the internet.
"""
import argparse
import base64
import hashlib
import json
import math
import re
import os
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from OCP.GeomAbs import GeomAbs_Line, GeomAbs_Circle
import cadquery as cq
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GCPnts import GCPnts_QuasiUniformDeflection, GCPnts_AbscissaPoint
from OCP.GeomAbs import (GeomAbs_BSplineSurface, GeomAbs_BezierSurface, GeomAbs_Cone, GeomAbs_Cylinder, GeomAbs_Plane,
                         GeomAbs_Sphere, GeomAbs_SurfaceOfExtrusion, GeomAbs_SurfaceOfRevolution, GeomAbs_Torus)
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

import studio as ST

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS)                                   # robo_chess
SESSION = os.path.join(TOOLS, "viewer_session")
PAGE = os.path.join(TOOLS, "viewer.html")
PORT = 8765
PALETTE = ["#b9bdc6", "#5b8def", "#e8a33d", "#6cc070", "#d9645a", "#a17fe0", "#4fc1c9", "#c9b458",
           "#8f9aa8", "#e07fb5", "#7fae4f", "#c98a5a"]
SURF = {GeomAbs_Plane: "plane", GeomAbs_Cylinder: "cylinder", GeomAbs_Cone: "cone", GeomAbs_Sphere: "sphere",
        GeomAbs_Torus: "torus", GeomAbs_BSplineSurface: "freeform", GeomAbs_BezierSurface: "freeform",
        GeomAbs_SurfaceOfRevolution: "revolved", GeomAbs_SurfaceOfExtrusion: "extruded"}
BROWSERS = [r"C:\Program Files\Google\Chrome\Application\chrome.exe",          # Chrome first: Edge's headless
            r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",    # mode exits without a picture here
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"]


# ---------------------------------------------------------------- reading
def read_step(path, face_colors=False):
    """Every solid in the file, placed: [(name, cq.Shape, (r, g, b) linear or None)]; with face_colors, a 4th item:
    the colour of each face (explorer order) or None, for models with coloured faces (Shapr3D can colour faces)."""
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    r = STEPCAFControl_Reader()
    r.SetNameMode(True)
    r.SetColorMode(True)
    if r.ReadFile(path) != IFSelect_RetDone:
        raise ValueError(f"can't read {path}")
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name_of(lab):
        a = TDataStd_Name()
        return a.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else ""

    def color_of(*labs):
        c = Quantity_Color()
        for lab in labs:
            for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
                try:
                    if ct.GetColor(lab, t, c):
                        return (c.Red(), c.Green(), c.Blue())
                except Exception:
                    pass
        return None

    out = []

    def walk(lab, loc, inst, col):
        if st.IsReference_s(lab):
            ref = TDF_Label()
            st.GetReferredShape_s(lab, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(lab)), lab, color_of(lab) or col)
        elif st.IsAssembly_s(lab):
            kids = TDF_LabelSequence()
            st.GetComponents_s(lab, kids, False)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc, None, color_of(lab) or col)
        else:
            base = st.GetShape_s(lab)
            sh = base.Moved(loc)
            name = name_of(lab) or (name_of(inst) if inst is not None else "") or "part"
            colr = color_of(lab) or col
            e, eb = TopExp_Explorer(sh, TopAbs_SOLID), TopExp_Explorer(base, TopAbs_SOLID)
            while e.More():
                item = (name, cq.Shape.cast(e.Current()), colr)
                if face_colors:
                    fc = []                                   # colours sit on the unmoved faces; same order
                    fe = TopExp_Explorer(eb.Current(), TopAbs_FACE)
                    while fe.More():
                        fc.append(color_of(fe.Current()))
                        fe.Next()
                    item = item + (fc if any(fc) else None,)
                out.append(item)
                e.Next()
                eb.Next()

    tops = TDF_LabelSequence()
    st.GetFreeShapes(tops)
    for i in range(1, tops.Length() + 1):
        walk(tops.Value(i), TopLoc_Location(), None, None)
    return out


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode("ascii")


def r3(v):
    return [round(float(x), 4) for x in v]


# ---------------------------------------------------------------- meshing, face by face
def face_info(f):
    """Type, area, centre and the numbers that matter for that type (normal, axis, radius, hole or outside)."""
    s = BRepAdaptor_Surface(f, True)
    g = GProp_GProps()
    BRepGProp.SurfaceProperties_s(f, g)
    c = g.CentreOfMass()
    t = s.GetType()
    info = {"type": SURF.get(t, "surface"), "area": round(g.Mass(), 3), "c": r3((c.X(), c.Y(), c.Z()))}
    rev = f.Orientation() == TopAbs_REVERSED
    try:
        if t == GeomAbs_Plane:
            d = s.Plane().Axis().Direction()
            n = (-d.X(), -d.Y(), -d.Z()) if rev else (d.X(), d.Y(), d.Z())
            info["n"] = r3(n)
        elif t in (GeomAbs_Cylinder, GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_Torus):
            if t == GeomAbs_Cylinder:
                q = s.Cylinder(); ax = q.Axis(); info["r"] = round(q.Radius(), 4)
            elif t == GeomAbs_Cone:
                q = s.Cone(); ax = q.Axis(); info["r"] = round(q.RefRadius(), 4)
                info["half_angle_deg"] = round(np.degrees(q.SemiAngle()), 3)
            elif t == GeomAbs_Sphere:
                q = s.Sphere(); ax = q.Position().Axis(); info["r"] = round(q.Radius(), 4)
            else:
                q = s.Torus(); ax = q.Axis(); info["r"] = round(q.MinorRadius(), 4)
                info["R"] = round(q.MajorRadius(), 4)
            d, o = ax.Direction(), ax.Location()
            info["axis"] = r3((d.X(), d.Y(), d.Z()))
            info["axis_at"] = r3((o.X(), o.Y(), o.Z()))
            u = (s.FirstUParameter() + s.LastUParameter()) / 2
            v = (s.FirstVParameter() + s.LastVParameter()) / 2
            pr = BRepLProp_SLProps(s, u, v, 1, 1e-6)
            if pr.IsNormalDefined():
                nn = pr.Normal()
                n = np.array((nn.X(), nn.Y(), nn.Z())) * (-1 if rev else 1)
                p = pr.Value()
                w = np.array((p.X() - o.X(), p.Y() - o.Y(), p.Z() - o.Z()))
                dd = np.array((d.X(), d.Y(), d.Z()))
                radial = w - w.dot(dd) * dd
                info["hole"] = bool(n.dot(radial) < 0)          # surface faces the axis: a hole / bore
    except Exception:
        pass
    return info


def mesh_solid(shape, tol=0.05, ang=0.15, face_cols=None):
    """Triangles face by face (each face keeps its own vertex and triangle range), face details, and the model's
    real edges (seams and degenerate edges left out)."""
    w = shape.wrapped
    BRepMesh_IncrementalMesh(w, tol, False, ang, True)
    pos, idx, faces = [], [], []
    vbase = tbase = 0
    seam = set()
    emap = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(w, TopAbs_EDGE, emap)
    ex = TopExp_Explorer(w, TopAbs_FACE)
    k = 0
    while ex.More():
        f = TopoDS.Face_s(ex.Current())
        ex.Next()
        k += 1
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        nv = nt = 0
        if tri is not None:
            trsf = loc.Transformation()
            nv, nt = tri.NbNodes(), tri.NbTriangles()
            for i in range(1, nv + 1):
                p = tri.Node(i).Transformed(trsf)
                pos.append((p.X(), p.Y(), p.Z()))
            rev = f.Orientation() == TopAbs_REVERSED
            for i in range(1, nt + 1):
                a, b, c = tri.Triangle(i).Get()
                idx.append((vbase + a - 1, vbase + c - 1, vbase + b - 1) if rev else (vbase + a - 1, vbase + b - 1, vbase + c - 1))
        fi = face_info(f)
        fi.update(f=k, v0=vbase, vn=nv, t0=tbase, tn=nt)
        if face_cols and k - 1 < len(face_cols) and face_cols[k - 1]:
            fi["col"] = [round(v, 4) for v in face_cols[k - 1]]
        faces.append(fi)
        vbase += nv
        tbase += nt
        ee = TopExp_Explorer(f, TopAbs_EDGE)
        fi["_eidx"] = []
        while ee.More():
            e = TopoDS.Edge_s(ee.Current())
            ee.Next()
            if BRep_Tool.IsClosed_s(e, f):
                seam.add(emap.FindIndex(e))
            else:
                fi["_eidx"].append(emap.FindIndex(e))
    segs, einfo, enum = [], [], {}
    for i in range(1, emap.Extent() + 1):
        if i in seam:
            continue
        e = TopoDS.Edge_s(emap.FindKey(i))
        if BRep_Tool.Degenerated_s(e):
            continue
        try:
            cu = BRepAdaptor_Curve(e)
            d = GCPnts_QuasiUniformDeflection(cu, tol)
            pts = [d.Value(j) for j in range(1, d.NbPoints() + 1)] if d.IsDone() else []
        except Exception:
            pts = []
        s0 = len(segs) // 6
        for a, b in zip(pts, pts[1:]):
            segs.extend((a.X(), a.Y(), a.Z(), b.X(), b.Y(), b.Z()))
        sn = len(segs) // 6 - s0
        if not sn:
            continue
        # the edge as Mike sees it: P#.E# with its kind, length, ends, and for arcs the centre / radius / axis
        gp = lambda q: [round(q.X(), 3), round(q.Y(), 3), round(q.Z(), 3)]  # noqa: E731
        info = {"e": len(einfo) + 1, "s0": s0, "sn": sn, "a": gp(pts[0]), "b": gp(pts[-1])}
        try:
            info["len"] = round(GCPnts_AbscissaPoint.Length_s(cu), 3)
            t = cu.GetType()
            if t == GeomAbs_Line:
                info["type"] = "line"
            elif t == GeomAbs_Circle:
                c = cu.Circle()
                closed = pts[0].Distance(pts[-1]) < 1e-6
                info.update(type="circle" if closed else "arc", r=round(c.Radius(), 3), c=gp(c.Location()),
                            axis=[round(c.Axis().Direction().X(), 4), round(c.Axis().Direction().Y(), 4),
                                  round(c.Axis().Direction().Z(), 4)],
                            deg=round(math.degrees(info["len"] / c.Radius()), 1))
            else:
                info["type"] = "curve"
        except Exception:
            info["type"] = "edge"
        einfo.append(info)
        enum[i] = info["e"]
    for fi in faces:                                  # the edges around each face, by their E# (Robot Studio picks them)
        fi["edges"] = sorted({enum[i] for i in fi.pop("_eidx", []) if i in enum})
    return (np.array(pos, dtype=np.float32).ravel(), np.array(idx, dtype=np.uint32).ravel(), faces,
            np.array(segs, dtype=np.float32), einfo)


def mesh_stl(path):
    import trimesh
    m = trimesh.load(path, force="mesh")
    pos = np.asarray(m.vertices, dtype=np.float32).ravel()
    idx = np.asarray(m.faces, dtype=np.uint32).ravel()
    face = {"f": 1, "type": "mesh", "area": round(float(m.area), 3), "c": r3(m.centroid), "v0": 0,
            "vn": len(m.vertices), "t0": 0, "tn": len(m.faces)}
    return [("mesh", pos, idx, [face], None, float(m.volume) if m.is_watertight else 0.0, None, [])]


def load_model(path, tol):
    """[(name, pos, idx, faces, edges, volume, color)] for every solid in the file."""
    if path.lower().endswith(".stl"):
        return mesh_stl(path)
    out = []
    for name, s, col, fcols in read_step(path, face_colors=True):
        pos, idx, faces, segs, einfo = mesh_solid(s, tol, face_cols=fcols)
        out.append((name, pos, idx, faces, segs, s.Volume(), col, einfo))
    return out


def entry(m):
    """A scene entry: a path, or {"path": ..., "offset": [dx, dy, dz], "name": "...", "solids": [0, 2],
    "part_name": "..."}: show the model moved aside (the file is not moved), only some of its solids (counted from
    0 in file order), and/or with its parts called part_name."""
    return {"path": m} if isinstance(m, str) else dict(m)


def shifted(faces, off):
    out = []
    for f in faces:
        g = dict(f)
        for k in ("c", "axis_at"):
            if k in g:
                g[k] = [round(a + b, 4) for a, b in zip(g[k], off)]
        out.append(g)
    return out


def build_scene(paths, tol, cache=None):
    """Scene for the page: models and their parts, numbered P1, P2 ... across all models in order."""
    models, solids = [], []
    for mi, ent in enumerate(entry(p) for p in paths):
        path = ent["path"]
        off = [float(v) for v in ent.get("offset", [0, 0, 0])]
        key = (path, os.path.getmtime(path), os.path.getsize(path), tol)
        if cache is not None and cache.get(path, (None,))[0] == key:
            items = cache[path][1]
        else:
            items = load_model(path, tol)
            if cache is not None:
                cache[path] = (key, items)
        models.append({"id": mi, "name": ent.get("name") or os.path.basename(path), "path": path,
                       "offset": off if any(off) else None, "material": ent.get("material"),
                       "pick": ent.get("pick", True), "measure": ent.get("measure", True)})
        keep = ent.get("solids")
        for si, (name, pos, idx, faces, segs, vol, col, einfo) in enumerate(items):
            if keep is not None and si not in keep:
                continue
            if ent.get("part_name"):
                name = ent["part_name"]
            if any(off):
                o = np.array(off, dtype=np.float32)
                pos = (pos.reshape(-1, 3) + o).ravel()
                segs = (segs.reshape(-1, 3) + o).ravel() if segs is not None and len(segs) else segs
                faces = shifted(faces, off)
                einfo = [{**e, **{k: [round(a + b, 4) for a, b in zip(e[k], off)] for k in ("a", "b", "c") if k in e}}
                         for e in einfo]
            solids.append((mi, si, path, (name, pos, idx, faces, segs, vol, col, einfo), off if any(off) else None,
                           ent.get("color")))
    cols = [s[3][6] for s in solids if not s[5]]
    use_step_colors = bool(cols) and all(c is not None for c in cols) and \
        len({tuple(round(v, 3) for v in c) for c in cols}) >= 2
    parts, seen = [], {}
    for k, (mi, si, path, (name, pos, idx, faces, segs, vol, col, einfo), off, force_col) in enumerate(solids):
        pid = k + 1
        if name.strip().lower() in ("", "root", "part", "compound", "mesh"):   # Shapr3D calls every body "root"
            label = f"part {pid}"
        else:
            seen[name] = seen.get(name, 0) + 1
            label = name if seen[name] == 1 else f"{name} {seen[name]}"
        color = {"lin": list(col)} if use_step_colors and col is not None else {"hex": PALETTE[k % len(PALETTE)]}
        if force_col:                                   # scene entry "color": "#rrggbb" wins (e.g. chess piece sets)
            color = {"hex": force_col, "forced": True}
        parts.append({"pid": pid, "key": f"{path}#{si}", "model": mi, "solid": si, "file": os.path.basename(path),
                      "path": path, "offset": off, "name": label, "color": color, "vol": round(vol, 2), "pos": b64(pos),
                      "idx": b64(idx), "faces": faces, "edges": b64(segs) if segs is not None and len(segs) else None,
                      "edgeInfo": einfo})
    title = " + ".join(m["name"] for m in models) or "no models open"
    return {"title": title, "models": models, "parts": parts}


def page(static=None):
    html = open(PAGE, encoding="utf-8").read()
    if static is not None:
        html = html.replace("/*STATIC*/null", json.dumps(static).replace("</", "<\\/"))
    return html


# ---------------------------------------------------------------- pictures
def snapshot(url, png, size):
    """Render a page in a hidden Chrome (Edge as a fallback) and save a PNG."""
    exes = [b for b in BROWSERS if os.path.isfile(b)]
    if not exes:
        raise SystemExit("no Chrome or Edge found for --png")
    w, h = (int(v) for v in size.lower().split("x"))
    png = os.path.abspath(png)
    os.makedirs(os.path.dirname(png), exist_ok=True)
    for exe in exes:
        if os.path.exists(png):
            os.remove(png)
        prof = os.path.join(os.environ.get("TEMP", os.path.dirname(png)), "view_step_" + os.path.basename(exe)[:-4])
        cmd = [exe, "--headless=new", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist",
               f"--user-data-dir={prof}", "--hide-scrollbars", "--force-device-scale-factor=1",
               f"--window-size={w},{h}", "--virtual-time-budget=20000", f"--screenshot={png}", url]
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=180)
        except subprocess.TimeoutExpired:
            continue
        if os.path.exists(png):
            return png
    raise SystemExit("no browser wrote the picture")


def file_url(path):
    return "file:///" + urllib.parse.quote(os.path.abspath(path).replace("\\", "/"))


def open_chrome(url):
    chrome = next((b for b in BROWSERS if os.path.isfile(b) and "chrome" in b.lower()), None)
    if chrome:
        subprocess.Popen([chrome, "--new-window", url])
    else:
        webbrowser.open(url)


# ---------------------------------------------------------------- the live app
class App:
    def __init__(self, tol, session=SESSION, name=ST.MAIN):
        self.tol = tol
        self.session, self.name, self.srv = session, name, None
        self._scene_cache = None
        self.lock = threading.Lock()
        self.cache = {}
        self.scene_key = None
        self.scene_txt = None
        os.makedirs(self.session, exist_ok=True)
        for name, text in (("scene.json", '{"models": []}\n'), ("show.json", "{}\n"), ("notes.md", "")):
            p = os.path.join(self.session, name)
            if not os.path.exists(p):
                open(p, "w", encoding="utf-8").write(text)

    def f(self, name):
        return os.path.join(self.session, name)

    def read_json(self, name, default):
        if name == "scene.json":                 # ~10 MB: parsed once per change, not on every poll of every page
            try:
                st = os.stat(self.f(name))
                k = (st.st_mtime_ns, st.st_size)
                if self._scene_cache and self._scene_cache[0] == k:
                    return self._scene_cache[1]
                v = json.load(open(self.f(name), encoding="utf-8"))
                self._scene_cache = (k, v)
                return v
            except Exception:
                return default
        try:
            return json.load(open(self.f(name), encoding="utf-8"))
        except Exception:
            return default

    def model_paths(self):
        """Scene entries (a path, or {"path", "offset", "name"}) whose files exist."""
        return [m for m in self.read_json("scene.json", {}).get("models", []) if os.path.isfile(entry(m)["path"])]

    def set_models(self, paths):
        try:                                                  # a fresh copy (read_json's is shared)
            sc = json.load(open(self.f("scene.json"), encoding="utf-8"))   # keep the joints (and the rest) as they are
        except Exception:
            sc = {}
        sc["models"] = paths
        json.dump(sc, open(self.f("scene.json"), "w", encoding="utf-8"), indent=1)

    def key(self):
        ents = self.model_paths()
        sc = self.read_json("scene.json", {})
        return hashlib.md5(repr([(json.dumps(m, sort_keys=True), os.path.getmtime(entry(m)["path"]),
                                  os.path.getsize(entry(m)["path"])) for m in ents] +
                                [json.dumps(sc.get("joints", []), sort_keys=True),
                                 json.dumps(sc.get("frame"), sort_keys=True),
                                 json.dumps(sc.get("poses", []), sort_keys=True),
                                 json.dumps(sc.get("ik"), sort_keys=True),
                                 json.dumps(sc.get("moves", []), sort_keys=True),
                                 json.dumps(sc.get("claw"), sort_keys=True),
                                 json.dumps(sc.get("colors"), sort_keys=True),
                                 json.dumps(sc.get("bowden"), sort_keys=True),
                                 json.dumps(sc.get("piece_home"), sort_keys=True),
                                 json.dumps(sc.get("game"), sort_keys=True),
                                 json.dumps(sc.get("homing"), sort_keys=True),
                                 json.dumps(sc.get("gears"), sort_keys=True),
                                 json.dumps(sc.get("rigs"), sort_keys=True),
                                 json.dumps(sc.get("drawings"), sort_keys=True)]).encode()).hexdigest()[:12]

    def scene(self):
        with self.lock:
            k = self.key()
            if k != self.scene_key:
                paths = self.model_paths()
                t = time.time()
                try:
                    sc = build_scene(paths, self.tol, self.cache)
                    # kinematics: joints name the models (by their scene "name") that turn about them
                    sc["joints"] = self.read_json("scene.json", {}).get("joints", [])
                    sc["frame"] = self.read_json("scene.json", {}).get("frame")       # world pose of the whole model
                    sc["poses"] = self.read_json("scene.json", {}).get("poses", [])   # saved joint poses (buttons)
                    sc["ik"] = self.read_json("scene.json", {}).get("ik")             # drag-the-claw solver settings
                    sc["moves"] = self.read_json("scene.json", {}).get("moves", [])   # animated piece moves (buttons)
                    sc["claw"] = self.read_json("scene.json", {}).get("claw")         # finger pins / plunger for animation
                    sc["bowden"] = self.read_json("scene.json", {}).get("bowden")     # the claw's Bowden tube route
                    sc["piece_home"] = self.read_json("scene.json", {}).get("piece_home", [])   # where each piece starts
                    sc["trays"] = self.read_json("scene.json", {}).get("trays")      # every tray spot (board_pieces.py)
                    sc["game"] = self.read_json("scene.json", {}).get("game")         # a self-play game (selfplay.py)
                    sc["homing"] = self.read_json("scene.json", {}).get("homing")     # home switches (hall_homing.py)
                    sc["gears"] = self.read_json("scene.json", {}).get("gears")       # models geared to a joint (pinion)
                    sc["rigs"] = self.read_json("scene.json", {}).get("rigs")         # test rigs (gearbox_anim.py)
                    sc["drawings"] = self.read_json("scene.json", {}).get("drawings")  # SVGs as pen strokes (svg_draw.py)
                    try:                                       # Mike's wood pictures for the board (woods.py)
                        import woods as WD
                        sc["woods"] = WD.wood_data(embed=False, quiet=True)
                    except Exception:
                        sc["woods"] = None
                    # colour rules: [{"match": regex on "<model> / <part>", "color": "#rrggbb"}], first match wins,
                    # for every part whose scene entry has no colour of its own (Mike: printed parts one filament)
                    rules = self.read_json("scene.json", {}).get("colors") or []
                    mnames = {m["id"]: m["name"] for m in sc.get("models", [])}
                    for part in sc.get("parts", []):
                        if part["color"].get("forced"):
                            continue
                        key = f"{mnames.get(part['model'], '')} / {part['name']}"
                        for r in rules:
                            if re.search(r["match"], key, re.I):
                                part["color"] = {"hex": r["color"]}
                                break
                except Exception as e:                       # a half-written file: try again next poll
                    sc = {"title": f"error: {e}", "models": [], "parts": []}
                    k = None
                self.scene_txt = json.dumps(sc)
                self.scene_key = k
                print(f"scene: {len(paths)} model(s), {len(sc['parts'])} part(s), {time.time() - t:.1f} s", flush=True)
            return self.scene_txt

    def state(self):
        def mt(n):
            try:
                return os.path.getmtime(self.f(n))
            except OSError:
                return 0
        notes = open(self.f("notes.md"), encoding="utf-8").read() if os.path.exists(self.f("notes.md")) else ""
        return {"scene": self.key(), "notes": notes, "notes_v": mt("notes.md"),
                "notes_time": time.strftime("%H:%M:%S", time.localtime(mt("notes.md"))) if notes.strip() else "",
                "show": self.read_json("show.json", {}), "show_v": mt("show.json"),
                "page_v": os.path.getmtime(PAGE),                  # the page reloads itself when viewer.html changes
                "fea": {k: v for k, v in FEA.status().items() if k in ("state", "result_v")}}   # load tests: every page follows

    def save_capture(self, png):
        """Every picture Mike copies or saves also lands here, so Claude can look at what he sees."""
        d = self.f("captures")
        os.makedirs(d, exist_ok=True)
        p = os.path.join(d, time.strftime("capture_%Y-%m-%d_%H%M%S.png"))
        open(p, "wb").write(png)
        open(os.path.join(d, "latest.png"), "wb").write(png)
        return p

    def save_selection(self, sel):
        if sel.pop("joints_only", False):           # a joint move: keep the faces/parts already picked
            old = self.read_json("selection.json", {})
            old["joints"] = sel.get("joints", {})
            sel = old
        sel["time"] = time.strftime("%Y-%m-%d %H:%M:%S")
        json.dump(sel, open(self.f("selection.json"), "w", encoding="utf-8"), indent=1)

    def open_upload(self, name, data):
        """A file picked in the browser arrives as bytes (browsers don't give the path). Use the original if it is
        somewhere under robo_chess (same name and contents), otherwise keep the copy in viewer_session/uploads."""
        name = os.path.basename(name)
        digest = hashlib.md5(data).hexdigest()
        found = None
        for d, _, files in os.walk(ROOT):
            if os.path.abspath(d).startswith(os.path.abspath(self.session)):
                continue
            if name in files:
                p = os.path.join(d, name)
                if os.path.getsize(p) == len(data) and hashlib.md5(open(p, "rb").read()).hexdigest() == digest:
                    found = p
                    break
        if not found:
            os.makedirs(self.f("uploads"), exist_ok=True)
            found = self.f(os.path.join("uploads", name))
            open(found, "wb").write(data)
        ents = self.model_paths()
        if found not in [entry(m)["path"] for m in ents]:
            self.set_models(ents + [found])
        return found

    def remove(self, path):
        self.set_models([m for m in self.model_paths() if entry(m)["path"] != path])


class FeaRunner:
    """Load tests (fea.py) run as their own process, so a long solve never holds up the page; one at a time.
    Progress: viewer_session/fea/status.json, result: fea/result.json, log: fea/run.log."""
    def __init__(self, session=SESSION, name=ST.MAIN):
        self.proc, self.lock = None, threading.Lock()
        self.dir, self.studio = os.path.join(session, "fea"), name

    def info(self):
        sys.path.insert(0, TOOLS)
        import importlib
        import fea
        importlib.reload(fea)                                   # fea.py edits show up without a server restart
        return {"parts": list(fea.RIGS), "material": fea.MATERIAL, "materials": fea.MATERIALS,
                "cases": {k: {kk: v[kk] for kk in ("F", "M", "gravity", "note")} for k, v in fea.CASES.items()}}

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def run(self, job):
        with self.lock:
            if self.running():
                return False
            os.makedirs(self.dir, exist_ok=True)
            jp = os.path.join(self.dir, "job.json")
            json.dump(job, open(jp, "w", encoding="utf-8"), indent=1)
            json.dump({"state": "running", "msg": "starting...", "time": time.time()},
                      open(os.path.join(self.dir, "status.json"), "w", encoding="utf-8"))
            log = open(os.path.join(self.dir, "run.log"), "w", encoding="utf-8")
            self.proc = subprocess.Popen([sys.executable, os.path.join(TOOLS, "fea.py"), "--job", jp], cwd=TOOLS,
                                         env={**os.environ, "ROBOT_STUDIO": self.studio},
                                         stdout=log, stderr=subprocess.STDOUT,
                                         creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            print("load test:", job.get("part"), "/", job.get("case"), flush=True)
            return True

    def status(self):
        try:
            s = json.load(open(os.path.join(self.dir, "status.json"), encoding="utf-8"))
        except Exception:
            s = {"state": "none", "msg": ""}
        if s.get("state") == "running" and not self.running():
            if self.proc is not None:
                s = {"state": "error", "msg": "the load test stopped (see viewer_session/fea/run.log)"}
            elif time.time() - s.get("time", 0) > 120:       # a command-line run that was cut off part way
                s = {"state": "error", "msg": "the last load test stopped part way: Run it again"}
        r = os.path.join(self.dir, "result.json")
        s["result_v"] = os.path.getmtime(r) if os.path.exists(r) else 0
        return s

    def result(self):
        return open(os.path.join(self.dir, "result.json"), encoding="utf-8").read()


FEA = FeaRunner()


def make_handler(app):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code, body, ctype="application/json"):
            b = body if isinstance(body, bytes) else body.encode("utf-8")
            try:
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                pass                       # the page went away mid-answer (a reload, a studio switch): nothing to tell

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            try:
                if u.path in ("/", "/index.html"):
                    self.send(200, page(), "text/html; charset=utf-8")
                elif u.path == "/api/state":
                    self.send(200, json.dumps(app.state()))
                elif u.path == "/api/scene":
                    self.send(200, app.scene())
                elif u.path == "/api/ping":                # is this studio up, and which is it (studio.alive)
                    self.send(200, json.dumps({"studio": app.name}))
                elif u.path == "/api/studios":
                    self.send(200, json.dumps(studios_state(app)))
                elif u.path == "/api/fea/info":
                    self.send(200, json.dumps(FEA.info()))
                elif u.path == "/api/fea/status":
                    self.send(200, json.dumps(FEA.status()))
                elif u.path == "/api/fea/result":
                    self.send(200, FEA.result())
                elif u.path.startswith("/textures/"):         # Mike's wood pictures (robo_chess\textures, woods.py)
                    import woods as WD
                    p = WD.texture_file(u.path[len("/textures/"):])
                    if p:
                        self.send(200, open(p, "rb").read(), WD.mime(p))
                    else:
                        self.send(404, "{}")
                else:
                    self.send(404, "{}")
            except Exception as e:
                self.send(500, json.dumps({"error": str(e)}))

        def do_POST(self):
            u = urllib.parse.urlparse(self.path)
            body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            try:
                if u.path == "/api/selection":
                    app.save_selection(json.loads(body))
                elif u.path == "/api/open":
                    name = urllib.parse.parse_qs(u.query).get("name", ["model.step"])[0]
                    print("opened", app.open_upload(name, body), flush=True)
                elif u.path == "/api/remove":
                    app.remove(json.loads(body)["path"])
                elif u.path == "/api/capture":
                    print("picture", app.save_capture(body), flush=True)
                elif u.path == "/api/studio/start":
                    n = json.loads(body)["name"]
                    if n not in ST.studios():
                        return self.send(404, '{"error": "no such studio"}')
                    start_studio(n)
                elif u.path == "/api/quit":                 # view_step.py --restart (a session restarting its studio)
                    self.send(200, '{"ok": true}')
                    threading.Thread(target=app.srv.shutdown, daemon=True).start()
                    return
                elif u.path == "/api/fea/run":
                    if not FEA.run(json.loads(body)):
                        return self.send(409, '{"error": "a load test is already running"}')
                else:
                    return self.send(404, "{}")
                self.send(200, '{"ok": true}')
            except Exception as e:
                self.send(500, json.dumps({"error": str(e)}))
    return H


def studios_state(app):
    """The studios, which of them are running, and which one this is (the page's studio menu)."""
    reg = ST.studios()
    up = {}

    def ping(n, p):
        up[n] = n == app.name or ST.alive(p)
    ts = [threading.Thread(target=ping, args=(n, int(v["port"]))) for n, v in reg.items()]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    return {"current": app.name, "studios": [{"name": n, "port": int(v["port"]), "who": v.get("who", ""),
                                              "what": v.get("what", ""), "running": up.get(n, False)}
                                             for n, v in reg.items()]}


def start_studio(n):
    """Run studio n as its own process (no window; its output in its folder's server.log), unless it's running."""
    if ST.alive(ST.port(n)):
        return
    d = ST.ensure(n)
    log = open(os.path.join(d, "server.log"), "a", encoding="utf-8")
    flags = (getattr(subprocess, "DETACHED_PROCESS", 0) | getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
             | getattr(subprocess, "CREATE_NO_WINDOW", 0))
    subprocess.Popen([sys.executable, os.path.abspath(__file__), "--serve", "--studio", n, "--no-browser"], cwd=TOOLS,
                     stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, creationflags=flags)


def serve(files, tol, port, browser, name=ST.MAIN, restart=False):
    global FEA
    session = ST.ensure(name)
    os.environ["ROBOT_STUDIO"] = name                     # scripts this server runs (fea.py) use its folder
    port = port or ST.port(name)
    app = App(tol, session, name)
    FEA = FeaRunner(session, name)
    url = f"http://127.0.0.1:{port}/"
    if restart and ST.alive(port):                            # ask the running one to stop, then take its place
        try:
            urllib.request.urlopen(urllib.request.Request(url + "api/quit", data=b"{}", method="POST"), timeout=5).read()
        except Exception:
            pass
        for _ in range(50):
            if not ST.alive(port, 0.2):
                break
            time.sleep(0.2)
    if files:
        app.set_models([os.path.abspath(f) for f in files])
    try:                                                       # already running? then just show the window
        urllib.request.urlopen(url + "api/state", timeout=2).read()
        print(f"viewer already running at {url}")
        if browser:
            open_chrome(url)
        return
    except Exception:
        pass
    srv = ThreadingHTTPServer(("127.0.0.1", port), make_handler(app))
    app.srv = srv
    print(f"Robot Studio '{name}' at {url}  (session files: {session})", flush=True)
    app.scene()
    if browser:
        open_chrome(url)
    srv.serve_forever()


# ---------------------------------------------------------------- command line
def main():
    ap = argparse.ArgumentParser(description="Robot Studio; see the top of this file.")
    ap.add_argument("files", nargs="*")
    ap.add_argument("--serve", action="store_true", help="run the live app (port 8765) and open it in Chrome")
    ap.add_argument("--port", type=int, default=None, help="default: the studio's own (main 8765)")
    ap.add_argument("--studio", default=None, help="which Robot Studio (tools/studios.json); default ROBOT_STUDIO, else main")
    ap.add_argument("--restart", action="store_true", help="stop that studio's running server first, then serve")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--out", help="static page to write (default tools/views/<first file>.html)")
    ap.add_argument("--tol", type=float, default=0.05, help="mesh tolerance, mm")
    ap.add_argument("--open", action="store_true", help="open the static page in a new Chrome window")
    ap.add_argument("--view", default="iso", help="iso iso2 iso3 iso4 front back left right top bottom, or dx,dy,dz")
    ap.add_argument("--cut", help="section plane: x, y or z, with a position: y=0, x=-150.5 (no position = middle)")
    ap.add_argument("--keep", choices=["+", "-"], help="side of the cut that stays (default: the side away from you)")
    ap.add_argument("--hide", nargs="+", default=[], help="parts to hide: P number (2 or P2) or part of the name")
    ap.add_argument("--only", nargs="+", help="show only these parts")
    ap.add_argument("--at", help="look at this point x,y,z instead of the middle")
    ap.add_argument("--zoom", type=float, default=1.0, help="2 = twice as close")
    ap.add_argument("--no-edges", action="store_true")
    ap.add_argument("--png", help="also save a picture of this view (headless Chrome)")
    ap.add_argument("--size", default="1600x1000", help="picture size for --png")
    a = ap.parse_args()

    if a.serve:
        return serve(a.files, a.tol, a.port, not a.no_browser, ST.name(a.studio), a.restart)
    if not a.files:
        ap.error("give a STEP file, or --serve")

    init = {"view": a.view, "zoom": a.zoom, "edges": not a.no_edges}
    if a.cut:
        ax, _, p = a.cut.partition("=")
        init["cut"] = {"axis": ax.strip().lower(), "pos": float(p) if p.strip() else None}
    if a.keep:
        init["keep"] = a.keep
    if a.hide:
        init["hide"] = a.hide
    if a.only:
        init["only"] = a.only
    if a.at:
        init["at"] = [float(v) for v in a.at.split(",")]
    sc = build_scene([os.path.abspath(f) for f in a.files], a.tol)
    for p in sc["parts"]:
        print(f"  P{p['pid']} {p['name']}: {len(p['faces'])} faces", flush=True)
    out = a.out or os.path.join(TOOLS, "views", os.path.splitext(os.path.basename(a.files[0]))[0] + ".html")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    open(out, "w", encoding="utf-8").write(page({"scene": sc, "init": init}))
    print(out)
    if a.png:
        print(snapshot(file_url(out) + "#ui=0", a.png, a.size))
    if a.open:
        open_chrome(file_url(out))


if __name__ == "__main__":
    main()

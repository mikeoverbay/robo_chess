"""Regroup Mike's Shapr3D assembly.step by joint and put each joint's gearbox back together.

Shapr3D exported one product per part TYPE holding that part from every gearbox
(bearing_6808_upper holds five bearings, 'Folder 54' holds 25 screws from all of them).
This script
  1. reads every solid in world coordinates (names and colours kept where the file has them),
  2. finds the joints (one per motor plate; axis = the plate's thin direction, pointing away
     from the motor) and gives every solid to the joint it sits on, checking that its own
     axis is parallel to that joint's axis,
  3. measures each part's position along its joint axis against the reference stack
     (joint 2, the shoulder, which matches gearbox_assembly.step) and moves parts that are
     off along the axis. The anchor is the sun, so the output side (carrier, lid, the
     70.5 discs and the rods on them) stays put and the motor side comes back to it.
     The housing (not in joint 2) is seated with its motor-side face on the plate and
     turned about the axis until its three 2.5 holes sit on the plate/lid screws,
  4. centres every round part whose bore (largest cylinder parallel to the joint axis) is
     off the axis; a 70.5 disc that has a centred copy goes to an 'Extra copies' folder,
  5. writes a STEP whose tree is joint -> motor / printed / bearings / screws, links, extras,
     and checks every pair of solids for overlap before and after.

    python regroup_assembly.py [in.step] [out.step]
"""
import sys
import os
import math
import numpy as np
import cadquery as cq
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf, XCAFDoc_ColorGen
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopLoc import TopLoc_Location
from OCP.Quantity import Quantity_Color
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.gp import gp_Trsf, gp_Vec, gp_Ax1, gp_Pnt, gp_Dir
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_StepModelType
from OCP.XSControl import XSControl_WorkSession
from OCP.Interface import Interface_Static
from cadquery.occ_impl.assembly import toCAF

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)            # robo_chess (this script lives in robo_chess/tools)
IN = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "assembly.step")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "assembly_by_joint.step")

# part type by volume (mm^3), all measured from this file
DISC = "Body 73 disc 70.5 x 24"
TYPES = [  # (volume, tolerance, type, folder)
    (2933.6, 1, "6808 bearing", "bearings"),
    (193.3, 1, "624 outer ring", "bearings"),
    (78.5, 1, "624 inner ring", "bearings"),
    (413.8, 1, "planet screw M4x25", "screws"),
    (109.7, 1, "plate screw M3x10", "screws"),
    (139.1, 1, "lid screw M3x16", "screws"),
    (14.5, 1, "planet spacer", "printed"),
    (145.4, 1, "planet spacer 11 long", "printed"),
    (674.0, 1, "race retainer", "printed"),
    (1144.1, 1, "planet", "printed"),
    (1790.6, 1, "sun", "printed"),
    (7725.3, 2, "lid 64.5 OD", "printed"),
    (9319.7, 2, "spider", "printed"),
    (12046.7, 2, "lid", "printed"),
    (14724.6, 2, "carrier", "printed"),
    (19913.2, 2, "motor plate", "printed"),
    (47984.2, 5, "housing", "printed"),
    (21742.0, 5, "NEMA 17 front", "motor"),
    (33825.8, 5, "NEMA 17 body", "motor"),
    (20625.4, 5, "NEMA 17 rear", "motor"),
    (3927.0, 2, "rod 5 x 200", "links"),
    (92592.2, 10, DISC, "printed"),
    (95546.6, 10, "Body 41 (1) shoulder housing", "printed"),
    (109186.1, 10, "Body 41 base block", "printed"),
]
REF_JOINT = 2       # the shoulder: its stack matches gearbox_assembly.step
MOVE_TOL = 0.05
CENTRE_TOL = 0.1    # the NEMA model's own 0.02 is left alone
ON_AXIS = {"6808 bearing", "spider", "carrier", "lid", "lid 64.5 OD", "race retainer", "sun", "motor plate",
           "housing", "NEMA 17 front", "NEMA 17 rear", DISC}


def name_of(lab):
    a = TDataStd_Name()
    if lab.FindAttribute(TDataStd_Name.GetID_s(), a):
        return a.Get().ToExtString()
    return ""


def read(path):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    r = STEPCAFControl_Reader()
    r.SetNameMode(True)
    r.SetColorMode(True)
    assert r.ReadFile(path) == IFSelect_RetDone
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    out = []

    def colour(shape, lab):
        c = Quantity_Color()
        for t in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if ct.GetColor(shape, t, c):
                return c
        for t in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if ct.GetColor(lab, t, c):
                return c
        return None

    def walk(lab, loc):
        if st.IsReference_s(lab):
            ref = TDF_Label()
            st.GetReferredShape_s(lab, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(lab)))
            return
        if st.IsAssembly_s(lab):
            kids = TDF_LabelSequence()
            st.GetComponents_s(lab, kids, False)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc)
            return
        e = TopExp_Explorer(st.GetShape_s(lab), TopAbs_SOLID)
        while e.More():
            s = e.Current()
            e.Next()
            out.append(dict(solid=s.Moved(loc), colour=colour(s, lab)))

    tops = TDF_LabelSequence()
    st.GetFreeShapes(tops)
    for i in range(1, tops.Length() + 1):
        walk(tops.Value(i), TopLoc_Location())
    return out


def measure(p):
    s = p["solid"]
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(s, g)
    c = g.CentreOfMass()
    b = Bnd_Box()
    BRepBndLib.Add_s(s, b)
    p["vol"], p["c"], p["bb"] = g.Mass(), np.array([c.X(), c.Y(), c.Z()]), np.array(b.Get())


def classify(v):
    for vol, tol, t, grp in TYPES:
        if abs(v - vol) <= tol:
            return t, grp
    return f"unknown {v:.0f} mm3", "other"


def own_axis(p):
    """Index of the part's symmetry axis (the bbox side whose other two sides are equal), or None."""
    d = p["bb"][3:] - p["bb"][:3]
    hits = [k for k in range(3) if abs(d[(k + 1) % 3] - d[(k + 2) % 3]) < 0.6]
    return hits[0] if len(hits) == 1 else None


def bore_offset(p, j):
    """Radial vector from the joint axis to the axis of the part's largest cylinder parallel to it."""
    best = None
    for f in cq.Shape.cast(p["solid"]).Faces():
        if f.geomType() != "CYLINDER":
            continue
        cyl = f._geomAdaptor().Cylinder()
        d = cyl.Axis().Direction()
        if abs(abs(np.dot([d.X(), d.Y(), d.Z()], j["ax"])) - 1) > 1e-6:
            continue
        if best is None or cyl.Radius() > best[0]:
            loc = cyl.Axis().Location()
            best = (cyl.Radius(), np.array([loc.X(), loc.Y(), loc.Z()]))
    if best is None:
        return None
    e = best[1] - j["o"]
    return e - np.dot(e, j["ax"]) * j["ax"]


def axial_span(p, j):
    """Min and max of the part's bounding box along the joint axis, from the plate centre."""
    b = p["bb"]
    vals = [np.dot(np.array([x, y, z]) - j["o"], j["ax"]) for x in b[[0, 3]] for y in b[[1, 4]] for z in b[[2, 5]]]
    return min(vals), max(vals)


def plane_angle(j, pt):
    """Angle (deg) of a point about the joint axis, right-handed about j['ax']."""
    u = np.zeros(3)
    u[(j["k"] + 1) % 3] = 1.0
    v = np.cross(j["ax"], u)
    e = pt - j["o"]
    return math.degrees(math.atan2(np.dot(e, v), np.dot(e, u)))


def screw_holes(p, j):
    """Centres of the part's small holes parallel to the joint axis, 20..40 mm out."""
    out = []
    for f in cq.Shape.cast(p["solid"]).Faces():
        if f.geomType() != "CYLINDER":
            continue
        cyl = f._geomAdaptor().Cylinder()
        d = cyl.Axis().Direction()
        if cyl.Radius() > 1.5 or abs(abs(np.dot([d.X(), d.Y(), d.Z()], j["ax"])) - 1) > 1e-6:
            continue
        loc = cyl.Axis().Location()
        e = np.array([loc.X(), loc.Y(), loc.Z()]) - j["o"]
        e = e - np.dot(e, j["ax"]) * j["ax"]
        if 20 < np.linalg.norm(e) < 40 and not any(np.linalg.norm(e - q) < 0.05 for q in out):
            out.append(e)
    return [q + j["o"] for q in out]


def turn(p, j, deg):
    t = gp_Trsf()
    t.SetRotation(gp_Ax1(gp_Pnt(*map(float, j["o"])), gp_Dir(*map(float, j["ax"]))), math.radians(deg))
    p["solid"] = p["solid"].Moved(TopLoc_Location(t))
    measure(p)


def move(p, vec):
    t = gp_Trsf()
    t.SetTranslation(gp_Vec(*map(float, vec)))
    p["solid"] = p["solid"].Moved(TopLoc_Location(t))
    measure(p)


def overlaps(parts):
    """Every pair of solids sharing more than 0.5 mm3."""
    found = []
    for a in range(len(parts)):
        for b in range(a + 1, len(parts)):
            A, B = parts[a]["bb"], parts[b]["bb"]
            if np.any(A[:3] > B[3:]) or np.any(B[:3] > A[3:]):
                continue
            g = GProp_GProps()
            BRepGProp.VolumeProperties_s(BRepAlgoAPI_Common(parts[a]["solid"], parts[b]["solid"]).Shape(), g)
            if g.Mass() > 0.5:
                found.append((parts[a], parts[b], g.Mass()))
    return found


def write_flat(parts, path):
    """[(name, cq.Shape, cq.Color or None)] -> a STEP of plain top-level bodies, each named and coloured: no assembly
    wrapper, no instances (Mike 2026-11-06: Shapr3D showed a wrapper's parts as empty folders, bodies elsewhere)."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    for name, shape, color in parts:
        sh = shape.wrapped
        if not sh.Location().IsIdentity():               # a placed body would be written as a wrapper round an
            sh = BRepBuilderAPI_Transform(sh.Located(TopLoc_Location()), sh.Location().Transformation(), True).Shape()
        lab = st.AddShape(sh, False)                      # unplaced one: bake the placement into the geometry
        TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))
        if color is not None:
            ct.SetColor(lab, color.wrapped.GetRGB(), XCAFDoc_ColorSurf)
    session = XSControl_WorkSession()
    w = STEPCAFControl_Writer(session, False)
    w.SetColorMode(True)
    w.SetNameMode(True)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    w.Transfer(doc, STEPControl_StepModelType.STEPControl_AsIs)
    assert w.Write(path) == IFSelect_RetDone


def write_step(assy, path):
    """cadquery's writer, except every instance carries its part's name (cadquery leaves a
    number there, and some CAD programs show the instance name as the folder name).
    A one-level assembly (only plain parts in it - every file of the arm; regroup's joint groups stay nested) is
    written FLAT instead: each part a named top-level body (write_flat)."""
    kids = list(assy.children)
    if assy.obj is None and kids and all(not k.children and k.obj is not None for k in kids):
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        parts = []
        for k in kids:
            s = k.obj if isinstance(k.obj, cq.Shape) else cq.Compound.makeCompound(k.obj.vals())
            if not k.loc.wrapped.IsIdentity():           # bake a placement into the body itself
                s = cq.Shape.cast(BRepBuilderAPI_Transform(s.wrapped, k.loc.wrapped.Transformation(), True).Shape())
            parts.append((k.name, s, k.color))
        return write_flat(parts, path)
    _, doc = toCAF(assy, True)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    labels = TDF_LabelSequence()
    st.GetShapes(labels)
    for i in range(1, labels.Length() + 1):
        kids = TDF_LabelSequence()
        st.GetComponents_s(labels.Value(i), kids, False)
        for k in range(1, kids.Length() + 1):
            ref = TDF_Label()
            if st.GetReferredShape_s(kids.Value(k), ref) and name_of(ref):
                TDataStd_Name.Set_s(kids.Value(k), TCollection_ExtendedString(name_of(ref)))
    session = XSControl_WorkSession()
    w = STEPCAFControl_Writer(session, False)
    w.SetColorMode(True)
    w.SetNameMode(True)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    w.Transfer(doc, STEPControl_StepModelType.STEPControl_AsIs)
    assert w.Write(path) == IFSelect_RetDone


def main():
    parts = read(IN)
    for p in parts:
        measure(p)
        p["type"], p["group"] = classify(p["vol"])
    rep = []
    w = rep.append
    w(f"Input  {IN}\nOutput {OUT}\n{len(parts)} solids read.")

    # ---- joints: one per motor plate
    joints = []
    for p in parts:
        if p["type"] == "motor plate":
            k = int(np.argmin(p["bb"][3:] - p["bb"][:3]))
            ax = np.zeros(3)
            ax[k] = 1.0
            m = min((q for q in parts if q["type"] == "NEMA 17 body"), key=lambda q: np.linalg.norm(q["c"] - p["c"]))
            if np.dot(m["c"] - p["c"], ax) > 0:
                ax = -ax
            joints.append(dict(o=p["c"].copy(), ax=ax, k=k, parts=[]))
    joints.sort(key=lambda j: (j["k"] != 2, j["o"][1]))  # upright one first, then along y
    for n, j in enumerate(joints, 1):
        j["n"] = n
        j["name"] = f"Joint {n} - " + ("upright" if j["k"] == 2 else "on its side") + f" at y {j['o'][1]:.0f}"

    def ar(j, c):
        d = c - j["o"]
        a = float(np.dot(d, j["ax"]))
        return a, float(np.linalg.norm(d - a * j["ax"]))

    links, loose = {}, []
    for p in parts:
        if p["group"] == "links":
            lo, hi = p["bb"][1], p["bb"][4]
            ja = min(joints[1:], key=lambda j: abs(j["o"][1] - lo))
            jb = min(joints[1:], key=lambda j: abs(j["o"][1] - hi))
            links.setdefault((ja["n"], jb["n"]), []).append(p)
            continue
        k = own_axis(p)
        best = None
        for j in joints:
            if k is not None and k != j["k"]:
                continue
            a, r = ar(j, p["c"])
            if -70 <= a <= 70 and r <= 45 and (best is None or r < best[0]):
                best = (r, j)
        if best:
            best[1]["parts"].append(p)
        else:
            loose.append(p)

    # ---- reference stack (along the axis, from the plate centre)
    jr = joints[REF_JOINT - 1]
    ref = {}
    for p in jr["parts"]:
        ref.setdefault(p["type"], []).append(ar(jr, p["c"])[0])
    ref = {t: sorted(v) for t, v in ref.items()}

    def stack_lines(j):
        by = {}
        for p in j["parts"]:
            by.setdefault(p["type"], []).append(round(ar(j, p["c"])[0], 1))
        return {t: sorted(v) for t, v in by.items()}

    before = overlaps(parts)

    # ---- put each joint back together, along its own axis only, anchored on the sun
    w("\nMoves (along each joint's axis; + = toward the output). Your original file is untouched.")
    moved = 0
    extras = []
    for j in joints:
        sun = [p for p in j["parts"] if p["type"] == "sun"][0]
        shift = ar(j, sun["c"])[0] - ref["sun"][0]
        lines = []
        if abs(shift) > MOVE_TOL:
            # the true plate centre is where the sun says it is
            j["o"] = j["o"] + shift * j["ax"]
        by = {}
        for p in j["parts"]:
            by.setdefault(p["type"], []).append(p)
        for t, ps in sorted(by.items()):
            if t not in ref:
                continue
            want = ref[t]
            ps.sort(key=lambda p: ar(j, p["c"])[0])
            if len(ps) == len(want):
                pairs = list(zip(ps, want))
            else:
                pairs = [(p, min(want, key=lambda v: abs(v - ar(j, p["c"])[0]))) for p in ps]
            for p, target in pairs:
                d = target - ar(j, p["c"])[0]
                if abs(d) > MOVE_TOL:
                    move(p, d * j["ax"])
                    lines.append((t, d))
        # housing: motor-side face on the plate's output face
        plate = [p for p in j["parts"] if p["type"] == "motor plate"][0]
        for p in by.get("housing", []):
            d = axial_span(plate, j)[1] - axial_span(p, j)[0]
            if abs(d) > MOVE_TOL:
                move(p, d * j["ax"])
                lines.append(("housing (seated on the plate)", d))
        # round parts: bore on the joint axis; a 70.5 disc with a centred copy goes to extras
        centred_disc = any(np.linalg.norm(bore_offset(p, j)) < CENTRE_TOL for p in by.get(DISC, []))
        for p in list(j["parts"]):
            if p["type"] not in ON_AXIS:
                continue
            e = bore_offset(p, j)
            r = float(np.linalg.norm(e)) if e is not None else 0.0
            if r < CENTRE_TOL:
                continue
            if p["type"] == DISC and centred_disc:
                j["parts"].remove(p)
                p["extra"] = f"J{j['n']} {DISC} copy, {r:.1f} mm off the axis"
                extras.append(p)
                continue
            move(p, -e)
            lines.append((f"{p['type']} centred on the axis (bore was {r:.2f} off)", 0.0))
            if p["type"] == DISC:
                centred_disc = True
        # housing: turn it so its 2.5 holes sit on the plate screws' axes
        screws = [plate_angle for plate_angle in
                  (plane_angle(j, q["c"]) for q in j["parts"] if q["type"] == "plate screw M3x10")]
        for p in by.get("housing", []):
            holes = [plane_angle(j, q) for q in screw_holes(p, j)]
            if len(holes) != len(screws) or not holes:
                continue

            def wrap(a):
                return (a + 180) % 360 - 180
            best = None
            for d in sorted({round(wrap(sa - ha), 3) for sa in screws for ha in holes}, key=abs):
                if all(min(abs(wrap(h + d - sa)) for sa in screws) < 0.2 for h in holes):
                    best = d
                    break
            if best is not None and abs(best) > 0.05:
                turn(p, j, best)
                lines.append((f"housing turned {best:+.1f} deg about the axis (holes onto the screws)", 0.0))
        if lines:
            w(f"  {j['name']}:")
            agg = {}
            for t, d in lines:
                agg.setdefault((t, round(d, 1)), 0)
                agg[(t, round(d, 1))] += 1
            for (t, d), n in agg.items():
                w(f"    {n} x {t}" + (f"  {d:+.1f} mm" if d else ""))
                moved += n
        else:
            w(f"  {j['name']}: nothing to move")

    # ---- stack table after the moves
    w("\nStack along each joint axis after the moves (mm from the motor plate centre):")
    for j in joints:
        w(f"\n== {j['name']}  plate centre {np.round(j['o'], 1).tolist()}, axis {j['ax'].astype(int).tolist()}")
        for t, ax in sorted(stack_lines(j).items()):
            flag = ""
            if t in ref and len(ax) == len(ref[t]) and max(abs(a - b) for a, b in zip(ax, ref[t])) > 0.2:
                flag = "  <-- still off"
            elif t in ref and len(ax) != len(ref[t]):
                flag = f"  ({len(ref[t])} in joint {REF_JOINT})"
            w(f"  {len(ax):2d} x {t:30s} {ax}{flag}")
        types = {p["type"] for p in j["parts"]}
        miss = [t for t in ref if t not in types and t not in ("Body 41 (1) shoulder housing",)]
        if miss:
            w("  not here: " + ", ".join(miss))
        if "planet" not in types:
            w("  no planet gears in this joint")

    # ---- names
    for j in joints:
        by = {}
        for p in j["parts"]:
            by.setdefault(p["type"], []).append(p)
        for t, ps in by.items():
            ps.sort(key=lambda p: (round(ar(j, p["c"])[0], 1),
                                   round(math.atan2(*np.delete(p["c"] - j["o"], j["k"])), 2)))
            for k, p in enumerate(ps, 1):
                p["name"] = f"J{j['n']} {t}" + (f" {k}" if len(ps) > 1 else "")
                if t == "6808 bearing" and len(ps) == 2:
                    p["name"] = f"J{j['n']} 6808 bearing " + ("motor side" if k == 1 else "output side")
    for (a, b), ps in links.items():
        ps.sort(key=lambda p: tuple(np.round(p["c"], 1)))
        for k, p in enumerate(ps, 1):
            p["name"] = f"L{a}-{b} rod {k}"
    for p in loose:
        p["name"] = f"loose {p['type']}"
    for p in extras:
        p["name"] = p["extra"]

    after = overlaps(parts)

    def olines(found):
        out = []
        for a, b, v in found:
            out.append(f"  {a.get('name', a['type'])}  x  {b.get('name', b['type'])}: {v:.0f} mm3")
        return out

    w(f"\nOverlapping pairs (> 0.5 mm3) in the whole file: {len(before)} before, {len(after)} after.")
    w("After:")
    rep.extend(olines(after) or ["  none"])

    w("\nNot at any joint (put in 'Not at a joint'):")
    for p in loose:
        w(f"  {p['type']} at {np.round(p['c'], 1).tolist()}")
    if extras:
        w("Extra copies (put in 'Extra copies'):")
        for p in extras:
            w(f"  {p['name']} at {np.round(p['c'], 1).tolist()}")

    # ---- write the tree
    root = cq.Assembly(name="robo_chess_by_joint")

    def add(asm, p):
        col = None
        if p["colour"] is not None:
            col = cq.Color(p["colour"].Red(), p["colour"].Green(), p["colour"].Blue())
        asm.add(cq.Shape.cast(p["solid"]), name=p["name"], color=col)

    for j in joints:
        ja = cq.Assembly(name=j["name"])
        for grp in ("motor", "printed", "bearings", "screws", "other"):
            ps = [p for p in j["parts"] if p["group"] == grp]
            if ps:
                ga = cq.Assembly(name=f"J{j['n']} {grp}")
                for p in sorted(ps, key=lambda p: p["name"]):
                    add(ga, p)
                ja.add(ga)
        root.add(ja)
    la = cq.Assembly(name="Links")
    for (a, b), ps in sorted(links.items()):
        ka = cq.Assembly(name=f"Link joint {a} to joint {b}")
        for p in ps:
            add(ka, p)
        la.add(ka)
    root.add(la)
    for title, ps in (("Not at a joint", loose), ("Extra copies", extras)):
        if ps:
            fa = cq.Assembly(name=title)
            for p in ps:
                add(fa, p)
            root.add(fa)
    write_step(root, OUT)

    back = cq.importers.importStep(OUT).val()
    vb = sorted(round(s.Volume(), 0) for s in back.Solids())
    va = sorted(round(p["vol"], 0) for p in parts)
    w(f"\nRead back: {len(vb)} solids (in {len(va)}), every volume matches: {vb == va}")
    for j in joints:
        w(f"  {j['name']}: {len(j['parts'])} bodies")
    for (a, b), ps in sorted(links.items()):
        w(f"  Link joint {a} to joint {b}: {len(ps)} rods")
    w(f"  Not at a joint: {len(loose)}   Extra copies: {len(extras)}")
    txt = "\n".join(rep)
    print(txt)
    with open(os.path.splitext(OUT)[0] + "_report.txt", "w") as f:
        f.write(txt + "\n")


if __name__ == "__main__":
    main()

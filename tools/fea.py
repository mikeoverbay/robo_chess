#!/usr/bin/env python3
"""
fea.py - load tests for Robot Studio (Mike 2026-11-04: "show in 3D on our model where it's getting hot, bending, giving").

gmsh cuts the part's STEP into quadratic tetrahedra (C3D10), CalculiX solves it (both ship with FreeCAD 0.19), and the
result goes back to the page: the part's outer skin with the stress (von Mises) and the movement at every point, so the
page can paint it hot / cold and bend it (scaled up so the give can be seen).

    python fea.py                          # the mock-up link, arm weight (worst pose), prints a summary
    python fea.py --case twist
    python fea.py --job job.json           # what Robot Studio runs: the loads come in job.json, progress goes to
                                           # viewer_session/fea/status.json, the result to fea/result.json

How a part is held and loaded is its "rig" (RIGS below): the face that is held fixed, and the face the load comes in
through, made rigid (a bolted-on housing) and loaded at a reference point with forces and moments. Its own weight is on.

Units: mm, N, MPa (= N/mm^2), tonne/mm^3. Loads in the link's frame: x along the beam (elbow at x -200), y in the arm's
plane (gravity -y with the arm level), z along the joint axes.
"""
import argparse
import base64
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
import studio  # noqa: E402
SESSION = studio.session_dir()                     # the studio that ran it (ROBOT_STUDIO; main = viewer_session)
WORK = os.path.join(SESSION, "fea")
FC_BIN = r"C:\Program Files\FreeCAD 0.19\bin"
GMSH = os.path.join(FC_BIN, "gmsh.exe")
CCX = os.path.join(FC_BIN, "ccx.exe")

# Printed plastics, on the safe side: E = stiffness (MPa), strength = across the layers (the weakest way; along the
# layers is about twice that), rho = density x 0.85 (thin parts print nearly solid). The part is treated as SOLID
# plastic: walls and infill are not modelled, so thick sections (the 8 mm cup and plate) printed with sparse infill
# come out softer than this. Mike prints CREALITY standard PLA, gray (2026-11-04). Creality publishes no full sheet for
# it: Ender PLA tensile 34 MPa, bending 77; Hyper PLA tensile 53, flexural modulus 2490; Hyper PLA-CF flexural modulus
# 3552, bending 103 (retailer / aggregator listings). Across the layers taken as ~60 % of along them.
MATERIALS = {
    "PLA (Creality)": {"name": "PLA (Creality)", "E": 2500.0, "nu": 0.35, "rho": 1.24e-9 * 0.85, "strength": 20.0,
                       "note": "Creality standard PLA; softens from about 55 C (warm motors) and creeps under steady load"},
    "PLA-CF (Creality Hyper)": {"name": "PLA-CF (Creality Hyper)", "E": 3550.0, "nu": 0.35, "rho": 1.29e-9 * 0.85,
                                "strength": 20.0,
                                "note": "carbon-filled PLA, ~40 % stiffer; abrasive: needs a hardened nozzle"},
    "ABS, printed": {"name": "ABS, printed", "E": 2000.0, "nu": 0.35, "rho": 1.04e-9 * 0.85, "strength": 18.0,
                     "note": "takes heat to about 95 C (fine by warm motors), tough; prints in an enclosure (Mike: a hot "
                             "box); typical printed values, ~33 MPa along the layers"},
    "PETG, printed": {"name": "PETG, printed", "E": 2000.0, "nu": 0.38, "rho": 1.27e-9 * 0.85, "strength": 25.0,
                      "note": "tougher than PLA, takes heat to about 75 C, a fifth softer"},
    "nylon, printed": {"name": "nylon, printed", "E": 1700.0, "nu": 0.39, "rho": 1.14e-9 * 0.85, "strength": 25.0,
                       "note": "tough, but soft and it creeps; stiffness drops when it takes up moisture"},
}
MATERIAL = MATERIALS["PLA (Creality)"]

# the mock-up link (mockup_link.py): its numbers, so a change there carries over
Z_OUT, CUP_T, HOUSING_R, ELBOW_X = 59.2, 8.0, 35.25, -200.0
ZC = (-4.0 + Z_OUT + CUP_T) / 2                       # the cage's centre line (31.6)
RIGS = {
    "mockup plus link": {
        "step": os.path.join(ROOT, "complete assembly", "mockups", "mockup_link_plus.step"),
        "fixed": {"z": Z_OUT, "c": [0.0, 0.0], "r": HOUSING_R, "what": "cup bolted on the drive's output disc"},
        "load": {"z": 0.0, "c": [ELBOW_X, 0.0], "r": HOUSING_R, "ref": [ELBOW_X, 0.0, ZC],
                 "what": "the next drive's housing bolted on the mounting plate"},
        "gravity": [0.0, -1.0, 0.0],
        "claw": [-449.5, -211.5, 0.0],                # the claw's grip from the elbow, arm straight out and level
        "regions": [(-41.5, 99, "the cup / neck at the hub"), (-44.5, -41.5, "the hub end wall"),
                    (-159.0, -44.5, "the cage"), (-162.0, -159.0, "the elbow end wall"),
                    (-999, -162.0, "the mounting plate at the elbow")],
        "gussets": {"y": 28.0, "x": [(-41.5, -23.0), (-174.5, -162.0)]},
    },
}

# loads at the elbow for the first link (J1 to J2) of an arm built from these links, straight out and level (the worst
# pose), with a 75 g king in the claw. Beyond the elbow: 3 drives with motors and their 6808 (0.71 kg each, at 0 / 200
# / 400 mm), 2 links (0.104 kg at 100 / 300), claw + adapter (0.275 kg at 425), the king (0.075 kg at 450).
_W = [(0.71, 0), (0.104, 100), (0.71, 200), (0.104, 300), (0.71, 400), (0.275, 424.75), (0.075, 449.5)]
ARM_W = 9.81 * sum(m for m, _ in _W)                  # N
ARM_M = 9.81 * sum(m * x for m, x in _W) / 1000       # N.m
CASES = {   # forces N (Fx Fy Fz), moments N.m (Mx = twist about the beam, My = sideways bend, Mz = in the arm's plane)
    "arm weight": {"F": [0, -ARM_W, 0], "M": [0, 0, ARM_M], "gravity": True,
                   "note": f"first link, arm straight out and level, a king in the claw: {ARM_W:.1f} N and {ARM_M:.2f} N.m"},
    "twist": {"F": [0, 0, 0], "M": [1.0, 0, 0], "gravity": False, "note": "1 N.m twisting the cage about its length"},
    "claw bumped sideways": {"F": [0, 0, 5.0], "M": [-5.0 * 0.2115, 5.0 * 0.45, 0], "gravity": False,
                             "note": "5 N sideways on the claw tip (450 out, 211.5 down): side bend + twist"},
}
CASES["all together"] = {"F": [a + b for a, b in zip(CASES["arm weight"]["F"], CASES["claw bumped sideways"]["F"])],
                         "M": [a + b for a, b in zip(CASES["arm weight"]["M"], CASES["claw bumped sideways"]["M"])],
                         "gravity": True, "note": "arm weight + the sideways bump"}
for _c in CASES.values():
    _c.setdefault("part", "mockup plus link")

# ---- the swivel turntable (swivel_base.py: 72T bevel gear under a round plate, hollow spindle in two 6808s), in its
# own frame (Z up from the wood, yaw axis = Z). Held where the 6808 inner races grip the spindle (r 20, z 18-25 and
# 61-68) and where its shoulder sits on the upper race (z 68, r 20-22.4); loaded through the arm's foot, bolted to the
# plate top (z 89, the whole Ø108.7), made rigid. The arm on it, straight out and level (worst tipping): the base and
# J1 gearbox (0.3 + 0.71 kg) on the axis, 3 tube links (0.15 kg printed) at 100 / 300 / 500 mm, 3 gearboxes (0.71) at
# 200 / 400 / 600, claw + adapter (0.28) at 625, a king (0.075) at 650.
_TT_W = [(0.3, 0), (0.71, 0), (0.15, 100), (0.71, 200), (0.15, 300), (0.71, 400), (0.15, 500), (0.71, 600),
         (0.28, 625), (0.075, 650)]
TT_W = 9.81 * sum(m for m, _ in _TT_W)                # N, all of it down through the plate
TT_M = 9.81 * sum(m * x for m, x in _TT_W) / 1000     # N.m tipping (arm toward +X: weight at +X -> +Y moment)
RIGS["swivel turntable 72T"] = {
    "make": "swivel_turntable",                        # built from swivel_base.turntable() into WORK when needed
    "step": os.path.join(WORK, "swivel_turntable_blank.step"),   # the gear as its blank to the tooth roots: the thin
    #                                                       swept teeth mesh inside-out; they are checked by hand
    "fixed": [{"cyl": 20.0, "c": [0.0, 0.0], "z": [18.0, 25.0]}, {"cyl": 20.0, "c": [0.0, 0.0], "z": [61.0, 68.0]},
              {"z": 68.0, "c": [0.0, 0.0], "r": 22.4, "r_in": 20.0}],
    "held": "the spindle in its two 6808s (and its shoulder on the upper one)",
    "load": {"z": 89.0, "c": [0.0, 0.0], "r": 54.4, "ref": [0.0, 0.0, 89.0]},
    "loaded": "the arm's foot, bolted to the plate top",
    "gravity": [0.0, 0.0, -1.0],
    "point": "The top plate", "tilt_axes": [0, 1], "twist_axis": 2,
    "regions": [(-1e9, 25.5, "the spindle"), (25.5, 68.5, "the spindle / gear hub"),
                (68.5, 77.0, "the gear"), (77.0, 1e9, "the plate")],
    "region_axis": 2,
}
CASES["arm reaching out"] = {"part": "swivel turntable 72T", "F": [0, 0, -TT_W], "M": [0, TT_M, 0], "gravity": True,
                             "note": f"the whole arm straight out and level on it: {TT_W:.0f} N down, "
                                     f"{TT_M:.1f} N.m tipping (gear teeth not modelled: by hand x1.9 pinion, "
                                     "x2.9 gear at the motor's full torque)"}


def make_step(rig):
    """build a rig's STEP when it comes from a script (newer script -> rebuilt)"""
    if rig.get("make") != "swivel_turntable":
        return
    src = os.path.join(HERE, "swivel_base.py")
    if os.path.exists(rig["step"]) and os.path.exists(rig["step"] + ".showvol") and             os.path.getmtime(rig["step"]) >= os.path.getmtime(src):
        return
    import contextlib
    import io
    import cadquery as cq
    sys.path.insert(0, HERE)
    with contextlib.redirect_stdout(io.StringIO()):
        import swivel_base as SB
        real = SB.turntable()                       # with teeth: the part in the scene (its volume, to paint on it)
        bevel = SB.bevel
        SB.bevel = lambda z, delta, thick, blank_pts, phase_deg=0.0: SB.revolve(blank_pts)
        try:
            tt = SB.turntable()                     # the same with the gear as its blank (tooth roots)
        finally:
            SB.bevel = bevel
    os.makedirs(WORK, exist_ok=True)
    cq.exporters.export(tt, rig["step"])
    open(rig["step"] + ".showvol", "w").write(repr(real.Volume()))


def to_view(rig):
    """(R, t): the rig's frame -> the frame the part is shown in, or None"""
    if rig.get("make") != "swivel_turntable":
        return None
    import contextlib
    import io
    sys.path.insert(0, HERE)
    with contextlib.redirect_stdout(io.StringIO()):
        import arm_base as AB
        import swivel_base as SB
    y_wood = AB.Y_CUT + AB.FOOT_T + SB.Z_PLATE0 + SB.PLATE_T          # as swivel_base.to_design
    return np.array([[-1.0, 0, 0], [0, 0, -1.0], [0, -1.0, 0]]), np.array([0.0, y_wood, AB.Z_MID])


def status(msg, state="running", **kw):
    os.makedirs(WORK, exist_ok=True)
    s = {"state": state, "msg": msg, "time": time.time(), **kw}
    tmp = os.path.join(WORK, "status.json.tmp")
    json.dump(s, open(tmp, "w", encoding="utf-8"))
    os.replace(tmp, os.path.join(WORK, "status.json"))
    print(msg, flush=True)


# ---------------------------------------------------------------- mesh (gmsh)
def mesh(step, size, tag="mesh", straight=False):
    """Quadratic tets of the STEP's solid, about `size` mm (finer on small holes and curves). Cached on the file's
    time/size and the mesh size. Returns (node ids, xyz, elements as rows of 10 node ids in CalculiX order).
    straight: straight-sided quadratic tets (never inside-out; for thin knife-edge tapers where curved ones fold)."""
    os.makedirs(WORK, exist_ok=True)
    inp = os.path.join(WORK, f"{tag}.inp")
    key = f"{os.path.abspath(step)}|{os.path.getmtime(step)}|{os.path.getsize(step)}|{size}|{straight}"
    kf = inp + ".key"
    if not (os.path.exists(inp) and os.path.exists(kf) and open(kf, encoding="utf-8").read() == key):
        geo = os.path.join(WORK, f"{tag}.geo")
        open(geo, "w", encoding="utf-8").write(
            'SetFactory("OpenCASCADE");\n'
            f'Merge "{step.replace(os.sep, "/")}";\n'
            f"Mesh.MeshSizeMax = {size};\nMesh.MeshSizeMin = {size / 4};\nMesh.MeshSizeFromCurvature = 12;\n"
            "Mesh.ElementOrder = 2;\nMesh.Optimize = 1;\n" + ("Mesh.SecondOrderLinear = 1;\n" if straight else "") +
            "Physical Volume(1) = Volume{:};\n")
        r = subprocess.run([GMSH, geo, "-3", "-format", "inp", "-o", inp, "-nt", "8"], capture_output=True, text=True)
        if r.returncode or not os.path.exists(inp):
            raise RuntimeError("gmsh failed: " + (r.stdout + r.stderr)[-800:])
        open(kf, "w", encoding="utf-8").write(key)
    return read_inp(inp)


def read_inp(path):
    ids, xyz, els, mode = [], [], [], None
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("*"):
            u = line.upper()
            mode = "n" if u.startswith("*NODE") else ("e" if u.startswith("*ELEMENT") and "C3D10" in u else None)
            continue
        if mode == "n":
            v = line.split(",")
            if len(v) >= 4:
                ids.append(int(v[0])); xyz.append([float(v[1]), float(v[2]), float(v[3])])
        elif mode == "e":
            v = [int(t) for t in line.replace(",", " ").split()]
            if len(v) == 11:
                els.append(v[1:])
    return np.array(ids), np.array(xyz), np.array(els)


# ---------------------------------------------------------------- solve (CalculiX)
def disc_nodes(ids, xyz, face):
    """nodes on the plane z = face["z"] within r (and outside r_in) of its centre; or, with "cyl": on the cylinder of
    that radius about the z-parallel axis through c, between z[0] and z[1]; a list = all of them"""
    if isinstance(face, (list, tuple)):
        return np.unique(np.concatenate([disc_nodes(ids, xyz, f) for f in face]))
    d = np.hypot(xyz[:, 0] - face["c"][0], xyz[:, 1] - face["c"][1])
    if "cyl" in face:
        return ids[(np.abs(d - face["cyl"]) < 0.05) & (xyz[:, 2] >= face["z"][0] - 1e-3) & (xyz[:, 2] <= face["z"][1] + 1e-3)]
    return ids[(np.abs(xyz[:, 2] - face["z"]) < 1e-3) & (d <= face["r"] + 1e-3) & (d >= face.get("r_in", 0.0) - 1e-3)]


def solve(ids, xyz, els, fixed, loaded, ref, F, M, gravity, mat=MATERIAL, tag="job"):
    """Static solve. fixed: node ids held; loaded: node ids made one rigid body, loaded at ref with forces F (N) and
    moments M (N.m). Returns (disp per node, stress 6 per node, ref displacement, ref rotation in rad)."""
    nref, nrot = int(ids.max()) + 1, int(ids.max()) + 2
    deck = os.path.join(WORK, f"{tag}.inp")
    with open(deck, "w", encoding="utf-8") as f:
        f.write("*HEADING\nRobot Studio load test\n*NODE, NSET=NALL\n")
        f.write("".join(f"{i},{x:.6f},{y:.6f},{z:.6f}\n" for i, (x, y, z) in zip(ids, xyz)))
        rp = ",".join(f"{v:.6f}" for v in ref)
        f.write(f"*NODE\n{nref},{rp}\n{nrot},{rp}\n")
        f.write("*ELEMENT, TYPE=C3D10, ELSET=EALL\n")
        f.write("".join(f"{k + 1}," + ",".join(str(n) for n in e) + "\n" for k, e in enumerate(els)))

        def nset(name, nodes):
            f.write(f"*NSET, NSET={name}\n")
            for k in range(0, len(nodes), 16):
                f.write(",".join(str(n) for n in nodes[k:k + 16]) + "\n")
        nset("NFIX", fixed)
        nset("NLOAD", loaded)
        # every number SHORT: CalculiX reads at most 20 characters a field and silently drops the rest - ABS's density
        # written as 8.839999999999999e-10 became 8.8e-1, a billion times too heavy (2026-11-04)
        f.write(f"*MATERIAL, NAME=MAT\n*ELASTIC\n{mat['E']:.6g},{mat['nu']:.6g}\n*DENSITY\n{mat['rho']:.6e}\n")
        f.write("*SOLID SECTION, ELSET=EALL, MATERIAL=MAT\n")
        f.write(f"*RIGID BODY, NSET=NLOAD, REF NODE={nref}, ROT NODE={nrot}\n")
        f.write("*STEP\n*STATIC\n*BOUNDARY\nNFIX,1,3\n*CLOAD\n")
        for k in range(3):
            if F[k]:
                f.write(f"{nref},{k + 1},{F[k]:.6g}\n")
            if M[k]:
                f.write(f"{nrot},{k + 1},{M[k] * 1000.0:.6g}\n")           # N.m -> N.mm
        if gravity:
            g = gravity if isinstance(gravity, (list, tuple)) else [0, -1, 0]
            f.write(f"*DLOAD\nEALL,GRAV,9810.,{g[0]},{g[1]},{g[2]}\n")
        f.write(f"*NSET, NSET=NREFS\n{nref},{nrot}\n")             # the rigid body's point: its move and turn
        f.write("*NODE FILE\nU\n*EL FILE\nS\n*NODE PRINT, NSET=NREFS\nU\n*END STEP\n")
    for ext in (".frd", ".dat", ".sta", ".cvg"):
        try:
            os.remove(os.path.join(WORK, tag + ext))
        except OSError:
            pass
    # the equation solver on ONE thread: FreeCAD 0.19's ccx 2.17 multi-threaded SPOOLES gave a wrong answer once in 11
    # identical runs (2026-11-04); single-threaded it is repeatable and only ~3 s slower. Stresses still use all cores.
    env = dict(os.environ, OMP_NUM_THREADS=str(os.cpu_count() or 4), CCX_NPROC_EQUATION_SOLVER="1",
               CCX_NPROC_STIFFNESS=str(os.cpu_count() or 4), CCX_NPROC_RESULTS=str(os.cpu_count() or 4))
    r = subprocess.run([CCX, "-i", tag], cwd=WORK, capture_output=True, text=True, env=env)
    open(os.path.join(WORK, tag + "_ccx.log"), "w", encoding="utf-8").write(r.stdout + r.stderr)
    frd = os.path.join(WORK, tag + ".frd")
    if not os.path.exists(frd) or "ERROR" in r.stdout.upper().replace("NO ERROR", ""):
        raise RuntimeError("CalculiX failed: " + (r.stdout + r.stderr)[-1200:])
    res = read_frd(frd)
    index = {n: k for k, n in enumerate(ids)}
    U = np.zeros((len(ids), 3)); S = np.zeros((len(ids), 6))
    for n, v in res["DISP"].items():
        if n in index:
            U[index[n]] = v[:3]
    for n, v in res["STRESS"].items():
        if n in index:
            S[index[n]] = v[:6]
    pts = read_dat_disp(os.path.join(WORK, tag + ".dat"))
    return U, S, np.array(pts.get(nref, [0, 0, 0])), np.array(pts.get(nrot, [0, 0, 0]))


def read_dat_disp(path):
    """*NODE PRINT displacements from the .dat: {node: [ux, uy, uz]} (the rot node's are its rotations, rad)"""
    out, on = {}, False
    for line in open(path, encoding="utf-8", errors="replace"):
        if "displacements" in line:
            on = True
            continue
        v = line.split()
        if on and len(v) == 4:
            try:
                out[int(v[0])] = [float(t) for t in v[1:]]
            except ValueError:
                on = False
        elif on and v:
            on = False
    return out


def read_frd(path):
    """CalculiX .frd (ascii): the DISP and STRESS blocks, {node: [values]}"""
    out, cur = {}, None
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith(" -4"):
            name = line.split()[1]
            cur = out.setdefault(name, {})
        elif line.startswith(" -1") and cur is not None:
            n = int(line[3:13])
            body = line[13:].rstrip("\n")
            cur[n] = [float(body[i:i + 12]) for i in range(0, len(body) - 11, 12)]
        elif line.startswith(" -3"):
            cur = None
    return out


def von_mises(S):
    sx, sy, sz, txy, tyz, tzx = S.T
    return np.sqrt(0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2) + 3 * (txy ** 2 + tyz ** 2 + tzx ** 2))


# ---------------------------------------------------------------- the outer skin for the page
def skin(els):
    """boundary faces of the tet mesh, each split in 4 using its mid-side nodes, wound outward.
    Returns (node indices used, triangles as indices into that list) - indices are row numbers, not node ids."""
    corner = {(0, 1): 4, (1, 2): 5, (0, 2): 6, (0, 3): 7, (1, 3): 8, (2, 3): 9}
    faces = {}
    for e in range(len(els)):
        for tri, opp in (((0, 1, 2), 3), ((0, 1, 3), 2), ((1, 2, 3), 0), ((0, 2, 3), 1)):
            k = tuple(sorted(els[e][list(tri)]))
            if k in faces:
                faces[k] = None                       # shared by two tets: inside
            else:
                faces[k] = (e, tri, opp)
    return [v for v in faces.values() if v is not None], corner


def skin_arrays(ids, xyz, els):
    rows = {n: k for k, n in enumerate(ids)}
    bfaces, corner = skin(els)
    tris = []
    for e, (a, b, c), opp in bfaces:
        el = els[e]
        A, B, C, O = (rows[el[a]], rows[el[b]], rows[el[c]], rows[el[opp]])
        mid = lambda i, j: rows[el[corner[(min(i, j), max(i, j))]]]
        ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
        n = np.cross(xyz[B] - xyz[A], xyz[C] - xyz[A])
        if np.dot(n, xyz[A] - xyz[O]) < 0:            # wind outward
            B, C, ab, ca = C, B, ca, ab
            bc = bc
        tris += [(A, ab, ca), (ab, B, bc), (ca, bc, C), (ab, bc, ca)]
    tris = np.array(tris)
    used = np.unique(tris)
    remap = -np.ones(len(ids), dtype=np.int64); remap[used] = np.arange(len(used))
    return used, remap[tris]


def where(rig, p):
    g = rig.get("gussets")
    if g and abs(p[1]) >= g["y"] - 1.5:
        end = "hub" if p[0] > -100 else "elbow"
        for a, b in g["x"]:
            tip = b if end == "hub" else a                     # the gusset's point on the neck / plate
            if abs(p[0] - tip) <= 2.0:
                return f"the {'neck' if end == 'hub' else 'plate'}, where the {end} gusset ends"
            if a <= p[0] <= b:
                return f"a gusset at the {end}"
    ax = rig.get("region_axis", 0)
    for a, b, name in rig.get("regions", []):
        if a < p[ax] <= b:
            return name
    return ""


def b64(a, dt):
    return base64.b64encode(np.ascontiguousarray(a, dtype=dt).tobytes()).decode("ascii")


# ---------------------------------------------------------------- one load test
def run(part="mockup plus link", case="arm weight", F=None, M=None, gravity=None, size=3.0, mat=None):
    rig = RIGS[part]
    c = CASES.get(case, {})
    F = list(F if F is not None else c.get("F", [0, 0, 0]))
    M = list(M if M is not None else c.get("M", [0, 0, 0]))
    grav = c.get("gravity", True) if gravity is None else gravity
    if isinstance(mat, str):
        mat = MATERIALS[mat]
    mat = {**MATERIAL, **(mat or {})}
    t0 = time.time()
    make_step(rig)
    status(f"meshing {part} ({size} mm tets)...")
    ids, xyz, els = mesh(rig["step"], size)
    fixed, loaded = disc_nodes(ids, xyz, rig["fixed"]), disc_nodes(ids, xyz, rig["load"])
    if not len(fixed) or not len(loaded):
        raise RuntimeError(f"rig faces not found: {len(fixed)} held nodes, {len(loaded)} loaded nodes")
    try:                       # the part's exact volume: the page only paints a result on the shape it was run on
        import cadquery as cq
        volume = float(sum(sol.Volume() for sol in cq.importers.importStep(rig["step"]).solids().vals()))
    except Exception:
        volume = None
    if os.path.exists(rig["step"] + ".showvol"):  # tested without some detail (the gear's teeth): shown on the real part
        volume = float(open(rig["step"] + ".showvol").read())
    status(f"solving {len(els):,} elements, {len(ids):,} nodes ({3 * len(ids):,} unknowns)...")
    try:
        U, S, uref, rref = solve(ids, xyz, els, fixed, loaded, rig["load"]["ref"], F, M,
                                 rig["gravity"] if grav else None, mat)
    except RuntimeError as e:          # a curved piece folded inside-out in a thin taper: straight-sided pieces instead
        if "jacobian" not in str(e).lower():
            raise
        status("a curved piece folded inside-out: meshing again with straight-sided pieces...")
        ids, xyz, els = mesh(rig["step"], size, straight=True)
        fixed, loaded = disc_nodes(ids, xyz, rig["fixed"]), disc_nodes(ids, xyz, rig["load"])
        status(f"solving {len(els):,} elements, {len(ids):,} nodes ({3 * len(ids):,} unknowns)...")
        U, S, uref, rref = solve(ids, xyz, els, fixed, loaded, rig["load"]["ref"], F, M,
                                 rig["gravity"] if grav else None, mat)
    # never show a broken solve as an answer: no numbers, or a part that moves a fifth of its own size
    size = float(np.linalg.norm(xyz.max(axis=0) - xyz.min(axis=0)))
    if not (np.all(np.isfinite(U)) and np.all(np.isfinite(S))) or np.abs(U).max() > 0.2 * size:
        raise RuntimeError(f"the solve came out impossible (biggest move {np.abs(U).max():.3g} mm on a {size:.0f} mm "
                           "part) - not shown; see viewer_session/fea/job_ccx.log")
    vm = von_mises(S)
    status("building the picture...")
    used, tris = skin_arrays(ids, xyz, els)
    # the hot spots: the highest stresses away from the held face and the bolted-on plate (where the model's perfectly
    # stiff supports put stress peaks that the real, slightly soft bolted joint doesn't have)
    from scipy.spatial import cKDTree
    rows = {n: k for k, n in enumerate(ids)}
    bc = xyz[[rows[n] for n in np.concatenate([fixed, loaded])]]
    near_bc = cKDTree(bc).query(xyz, distance_upper_bound=2.5)[0] <= 2.5
    vm_free = np.where(near_bc, 0, vm)
    top = np.argsort(vm_free)[::-1][:1]
    um = np.linalg.norm(U, axis=1)
    claw = uref + np.cross(rref, np.array(rig.get("claw", [0.0, 0.0, 0.0])))   # the claw's grip moves with the elbow
    hot = xyz[top[0]]
    tilt = float(math.degrees(np.linalg.norm([rref[a] for a in rig.get("tilt_axes", [2])])))
    summary = {
        "part": part, "case": case, "note": c.get("note", "custom loads"), "volume": volume,
        "F": F, "M": M, "gravity": bool(grav), "material": mat,
        "elements": int(len(els)), "nodes": int(len(ids)), "seconds": round(time.time() - t0, 1),
        "max_stress": float(vm_free.max()), "max_stress_at": [float(v) for v in xyz[top[0]]],
        "max_stress_bc": float(vm.max()),
        "safety": float(mat["strength"] / max(vm_free.max(), 1e-9)),
        "max_move": float(um.max()),
        "elbow_move": [float(v) for v in uref], "elbow_rot_deg": [float(math.degrees(v)) for v in rref],
        "twist_deg": float(math.degrees(rref[rig.get("twist_axis", 0)])), "tilt_deg": tilt,
        "point": rig.get("point", "Elbow"), "has_claw": "claw" in rig,
        "claw_move": [float(v) for v in claw], "claw_error": float(np.linalg.norm(claw)),
        "hot_where": where(rig, hot),
        "held": rig.get("held") or rig["fixed"]["what"], "loaded": rig.get("loaded") or rig["load"]["what"],
        "ref": rig["load"]["ref"],
    }
    pos, disp = xyz[used], U[used]
    view = to_view(rig)
    if view is not None:                     # shown in the scene's frame
        Rv, tv = view
        pos, disp = pos @ Rv.T + tv, disp @ Rv.T
        summary["max_stress_at"] = [float(v) for v in Rv @ hot + tv]
    result = {**summary, "pos": b64(pos, "<f4"), "disp": b64(disp, "<f4"), "vm": b64(vm[used], "<f4"),
              "umag": b64(um[used], "<f4"), "idx": b64(tris, "<u4"), "nverts": int(len(used)), "ntris": int(len(tris))}
    tmp = os.path.join(WORK, "result.json.tmp")
    json.dump(result, open(tmp, "w", encoding="utf-8"))
    os.replace(tmp, os.path.join(WORK, "result.json"))
    status(f"done in {time.time() - t0:.0f} s", state="done", summary=summary)
    return summary


class busy:
    """one load test at a time in WORK (the page's and the command line's share the files)"""
    def __enter__(self):
        os.makedirs(WORK, exist_ok=True)
        self.p, t0 = os.path.join(WORK, "busy.lock"), time.time()
        while True:
            try:
                fd = os.open(self.p, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
                os.write(fd, str(os.getpid()).encode()); os.close(fd)
                return self
            except FileExistsError:
                try:
                    if time.time() - os.path.getmtime(self.p) > 180:        # left behind by a run cut off part way
                        os.remove(self.p)
                        continue
                except OSError:
                    continue
                if time.time() - t0 > 300:
                    raise RuntimeError("another load test is still running")
                status("waiting for the load test before this one...")
                time.sleep(1.0)

    def __exit__(self, *a):
        try:
            os.remove(self.p)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description="Robot Studio load tests; see the top of this file.")
    ap.add_argument("--part", default="mockup plus link")
    ap.add_argument("--case", default="arm weight", choices=list(CASES))
    ap.add_argument("--size", type=float, default=3.0, help="tet size, mm")
    ap.add_argument("--material", default=MATERIAL["name"], choices=list(MATERIALS))
    ap.add_argument("--job", help="job.json from Robot Studio: {part, case, F, M, gravity, size, material}")
    a = ap.parse_args()
    try:
        lock = busy().__enter__()
    except Exception as e:
        status(f"failed: {e}", state="error")
        raise
    try:
        if a.job:
            j = json.load(open(a.job, encoding="utf-8"))
            s = run(j.get("part", a.part), j.get("case", "custom"), j.get("F"), j.get("M"), j.get("gravity"),
                    float(j.get("size", a.size)), j.get("material"))
        else:
            s = run(a.part, a.case, size=a.size, mat=a.material)
    except Exception as e:
        status(f"failed: {e}", state="error")
        raise
    finally:
        lock.__exit__()
    print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in s.items()
                      if k not in ("material",)}, indent=1))


if __name__ == "__main__":
    main()

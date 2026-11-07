#!/usr/bin/env python3
"""
gearbox_anim.py - the arm's 37:1 cycloid gearbox (cycloid.py --spread 14, steel dowels: the one being printed) as a
working model in Robot Studio, and a 3D check that no two parts touch while it turns.

Mike 2026-11-06: "take the housing, the pins, and everything required to make the gearbox work, and I want you to animate
it. I want it exposed so I can see what's going on inside ... slice this with our viewer while it's animating, or hide
parts of it." / "I'm in the middle of printing it. I don't want to print a bunch of plastic and have it not actually work."

    python gearbox_anim.py            build every part with cycloid.py's own code (read only, nothing of the gearbox
                                      changes), write viewer_session/gearbox_anim/*.step - one file per motion - and put
                                      them in scene.json with the "gearbox motor" test rig that drives them
    python gearbox_anim.py --sweep    also turn it (the same motions the viewer plays) through one motor turn and one
                                      whole output turn and look for any two parts that share volume -> sweep.txt

The motions (cycloid.py disc_pose / build): the eccentric, its spacer and the motor shaft turn with the motor (theta);
each disc's centre runs round a circle of radius ECC (disc 2 half a turn behind) while the disc turns -theta/37, its 6800
going with it; the output (flange, support ring, clamp ring, inner race spacer, output pins, inserts, screws, Mike's
output disc) turns -theta/37; the rest (housing, ring pins, pin ring, lid, outer race spacer, 6808s, motor, motor and lid
screws) stays put.

Placed where the gearbox session put its static copy (2026-11-06): upright on the floor at x 260, beside the base.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import cycloid as C  # noqa: E402
import gearbox as G  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

SESSION = os.path.join(HERE, "viewer_session")
OUT = os.path.join(SESSION, "gearbox_anim")
import studio  # noqa: E402
SCENE = studio.scene_path()      # this studio's scene (ROBOT_STUDIO; default main); the STEP files stay in viewer_session
SPREAD = 14.0
RIG = "gearbox motor"
AT = [260.0, 209.0, 0.0]         # design frame: the gearbox axis at the motor face (floor at y 257 = motor's back end)
AXIS = [0.0, -1.0, 0.0]          # gearbox +z is design -y (up); the files are turned +90 about x to stand it up
U = [1.0, 0.0, 0.0]              # gearbox +x in the design frame (the orbit's 0 deg direction); +y is design +z
TAG = "gearbox_anim.py"

GROUPS = [  # (key, model name, how it moves)
    ("fixed", "gearbox anim: housing, ring pins, lid, motor", None),
    ("input", "gearbox anim: eccentric + motor shaft (motor speed)", {"ratio": 1.0}),
    ("disc1", "gearbox anim: disc 1 + its 6800", {"ratio": -1.0 / C.LOBES, "orbit": 0.0}),
    ("disc2", "gearbox anim: disc 2 + its 6800", {"ratio": -1.0 / C.LOBES, "orbit": 180.0}),
    ("output", "gearbox anim: output (1/37 speed)", {"ratio": -1.0 / C.LOBES}),
]


def group_of(name):
    n = name.lower()
    if n.startswith(("eccentric", "motor shaft")):
        return "input"
    if n.startswith(("disc 1", "6800 in disc 1")):
        return "disc1"
    if n.startswith(("disc 2", "6800 in disc 2")):
        return "disc2"
    if n.startswith(("output flange", "output support ring", "clamp ring", "inner race spacer", "output pin",
                     "insert", "output screw", "your output disc")):
        return "output"
    return "fixed"


def build_parts():
    """cycloid.py main()'s parts dict, steel pins, spread 14 (its defaults otherwise)."""
    C.set_spread(SPREAD)
    G.FLAT_START = C.FLAT_START
    return dict(housing_s=C.housing(False), pin_ring=C.pin_ring(), lid=C.lid(), disc=C.disc(), hub=C.hub(),
                ecc_sp=C.ecc_spacer(), flange=C.output_flange(), sring_s=C.support_ring(False), clamp=C.clamp_ring(),
                b6800=C.b6800(), b6808=C.GS.bearing_6808(), motor=C.motor_body(), shaft=C.motor_shaft(),
                outdisc=C.out_disc(), sp_out=C.spacer_outer(), sp_in=C.spacer_inner())


# ---------------------------------------------------------------- the viewer's motion, in the gearbox's own frame
def pose(shape, key, theta_deg):
    """Where the viewer puts a part of group `key` (built at input 0) at input theta: spin about the gearbox axis (or,
    for a disc, about its own moving centre) and, for a disc, the centre run round its circle."""
    if key == "fixed":
        return shape
    mv = dict((g[0], g[2]) for g in GROUPS)[key]
    spin = mv["ratio"] * theta_deg
    if "orbit" not in mv:
        return shape.rotate(V(0, 0, 0), V(0, 0, 1), spin) if spin else shape
    ph = math.radians(mv["orbit"])
    c0 = V(C.ECC * math.cos(ph), C.ECC * math.sin(ph), 0)
    th = math.radians(theta_deg) + ph
    c1 = V(C.ECC * math.cos(th), C.ECC * math.sin(th), 0)
    s = shape.translate(-c0)
    if spin:
        s = s.rotate(V(0, 0, 0), V(0, 0, 1), spin)
    return s.translate(c1)


def items_at0(parts):
    return [(nm, s, col, group_of(nm)) for nm, s, col, _ in C.build(False, 0.0, parts)]


# ---------------------------------------------------------------- export + scene
def export(items):
    os.makedirs(OUT, exist_ok=True)
    files = {}
    for key, model, _ in GROUPS:
        assy = cq.Assembly(name=model)
        for nm, s, col, g in items:
            if g == key:
                assy.add(s.rotate(V(0, 0, 0), V(1, 0, 0), 90), name=nm, color=cq.Color(*col))
        path = os.path.join(OUT, f"{key}.step")
        write_step(assy, path)
        files[key] = path
        print(f"  {model}: {sum(1 for i in items if i[3] == key)} parts -> {os.path.relpath(path, HERE)}", flush=True)
    return files


COLOR_RULES = [  # first match wins; "<model> / <part>" (view_step.py)
    ("^gearbox.* / (6800|6808)", "#c9ccd2"),
    ("^gearbox.* / (ring pin|output pin) ", "#c9ccd2"),
    ("^gearbox.* / motor shaft", "#c9ccd2"),
    ("^gearbox.* / motor body", "#2a2c30"),
    ("^gearbox.* / (motor screw|lid screw|output screw)", "#2a2c30"),
    ("^gearbox.* / insert", "#c9a03a"),
    ("^gearbox.* / disc 1", "#e8801a"),
    ("^gearbox.* / disc 2", "#f5b041"),
    ("^gearbox.* / eccentric", "#4cbf4c"),
    ("^gearbox.* / (output flange|output support ring|clamp ring|inner race spacer)", "#3350a0"),
    ("^gearbox.* / your output disc", "#9a9da3"),
    ("^gearbox.* / (housing|pin ring|lid|outer race spacer)", "#5b8def"),
]


def update_scene(files):
    sc = json.load(open(SCENE, encoding="utf-8"))
    names = {m for _, m, _ in GROUPS}
    old = sc.get("models", [])
    # the gearbox session's static copy (same place) gives way to the working one; its exploded copy stays
    keep = [m for m in old if not (isinstance(m, dict) and (m.get("name") in names
                                                             or m.get("name") == "gearbox (Robo Chess gearbox)"))]
    for key, model, _ in GROUPS:
        keep.append({"path": files[key], "name": model, "offset": AT})
    sc["models"] = keep
    gears = [g for g in (sc.get("gears") or []) if g.get("follows") != RIG]
    for key, model, mv in GROUPS:
        if mv is None:
            continue
        g = {"model": model, "follows": RIG, "ratio": mv["ratio"], "at": AT, "axis": AXIS}
        if "orbit" in mv:
            g["orbit"] = {"r": C.ECC, "phase": mv["orbit"], "u": U}
        g["why"] = f"{TAG}: cycloid.py's motion (disc_pose / build)"
        gears.append(g)
    sc["gears"] = gears
    ztop = C.Z_DISC_OUT + C.OUT_DISC_T
    rigs = [r for r in (sc.get("rigs") or []) if r.get("name") != RIG]
    rigs.append({
        "name": RIG, "rpm": 30, "turns": C.LOBES, "at": [AT[0], AT[1] - ztop / 2, AT[2]],
        "models": [m for _, m, _ in GROUPS],
        "readouts": [{"label": "output", "ratio": -1.0 / C.LOBES}],
        # a cut through each disc's middle (design-frame points; the cut is along world Z there), and one down the axis
        # (view: the camera direction, world frame - nearly straight down on a disc, square onto the axis cut)
        "views": [{"label": "cut disc 1", "cut": "z", "point": [AT[0], AT[1] - (C.Z_D1 + C.DISC_T / 2), AT[2]],
                   "view": "0.15,-0.3,1"},
                  {"label": "cut disc 2", "cut": "z", "point": [AT[0], AT[1] - (C.Z_D2 + C.DISC_T / 2), AT[2]],
                   "view": "0.15,-0.3,1"},
                  {"label": "cut down the axis", "cut": "y", "point": AT, "view": "0.3,-1,0.3"},
                  {"label": "no cut", "cut": "off", "view": "iso"}],
        "why": f"{TAG}: the 37:1 cycloid (spread 14, steel dowels) - Mike 2026-11-06 'animate it ... slice it while "
               "it's animating, or hide parts of it'"})
    sc["rigs"] = rigs
    rules = [r for r in (sc.get("colors") or []) if r.get("by") != TAG]
    sc["colors"] = [{"match": m, "color": c, "by": TAG} for m, c in COLOR_RULES] + rules
    open(SCENE, "w", encoding="utf-8").write(json.dumps(sc, indent=1))
    print(f"scene.json: {len(GROUPS)} gearbox models at {AT}, rig '{RIG}', {len(COLOR_RULES)} colour rules", flush=True)


# ---------------------------------------------------------------- the sweep
def common_volume(a, b):
    try:
        return a.intersect(b).Volume()
    except Exception:
        return -1.0


def sweep(items, parts, out_txt):
    """Every pair of parts in DIFFERENT motion groups (parts in one group never move relative to each other; cycloid.py
    checks those) at each pose: bounding boxes first, then the common volume of the real solids."""
    base = [(nm, s, g) for nm, s, _, g in items]
    poses = [float(d) for d in range(0, 360, 5)]                               # one motor turn: the discs' whole cycle
    poses += [C.LOBES * 7.5 * k for k in range(1, 48)]                         # one output turn, 7.5 deg of output apart
    rep = [f"gearbox_anim.py --sweep {time.strftime('%Y-%m-%d %H:%M')}: cycloid.py spread {SPREAD:g}, steel dowels, "
           f"{len(base)} parts; the viewer's motions; {len(poses)} poses (input 0..355 by 5, then 47 more a 7.5 deg "
           "output step apart); every pair in different motion groups whose boxes meet: common volume > 0.001 mm^3"]
    worst, n_pairs, t0 = [], 0, time.time()
    check_at = {47.0, 200.0}
    for i, th in enumerate(poses):
        placed = [(nm, pose(s, g, th), g) for nm, s, g in base]
        if th in check_at:                                                     # the viewer's motion = cycloid.py's
            ref = {nm: s for nm, s, _, _ in C.build(False, th, parts)}
            dev = max(s.Volume() - common_volume(s, ref[nm]) for nm, s, g in placed if g != "fixed")
            rep.append(f"  input {th:g}: viewer motion vs cycloid.build: largest volume not shared {dev:.4f} mm^3")
        bbs = [s.BoundingBox() for _, s, _ in placed]
        hits = []
        for a in range(len(placed)):
            for b in range(a + 1, len(placed)):
                if placed[a][2] == placed[b][2]:
                    continue
                A, B = bbs[a], bbs[b]
                if A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin:
                    continue
                n_pairs += 1
                v = common_volume(placed[a][1], placed[b][1])
                if v > 0.001 or v < 0:
                    hits.append((placed[a][0], placed[b][0], v))
        line = f"  input {th:8.1f} deg (output {-th / C.LOBES:7.2f}): " + (
            "no contact" if not hits else "; ".join(f"{x} x {y} {'ERROR' if v < 0 else f'{v:.3f} mm^3'}" for x, y, v in hits))
        rep.append(line)
        worst += hits
        print(f"[{i + 1}/{len(poses)} {time.time() - t0:.0f} s] {line.strip()}", flush=True)
    rep.append(f"RESULT: {'NO PART TOUCHES ANOTHER at any pose' if not worst else f'{len(worst)} contacts (above)'}; "
               f"{n_pairs} pair checks, {time.time() - t0:.0f} s")
    open(out_txt, "w", encoding="utf-8").write("\n".join(rep) + "\n")
    print(rep[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true", help="also run the 3D collision sweep (slow: minutes)")
    ap.add_argument("--no-scene", action="store_true", help="don't touch scene.json")
    ap.add_argument("--sweep-only", action="store_true", help="only the sweep: leave the STEP files and scene.json alone")
    a = ap.parse_args()
    t = time.time()
    print("building the gearbox parts (cycloid.py) ...", flush=True)
    parts = build_parts()
    items = items_at0(parts)
    for key, model, _ in GROUPS:
        print(f"  {key:6s}: " + ", ".join(nm.split(':')[0] for nm, _, _, g in items if g == key), flush=True)
    if not a.sweep_only:
        files = export(items)
        if not a.no_scene:
            update_scene(files)
    print(f"{time.time() - t:.0f} s", flush=True)
    if a.sweep or a.sweep_only:
        sweep(items, parts, os.path.join(OUT, "sweep.txt"))


if __name__ == "__main__":
    main()

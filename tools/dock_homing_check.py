#!/usr/bin/env python3
"""
dock_homing_check.py - does a plain homing cycle, started with the arm parked in the dock, clear the dock?

Mike 2026-11-09: "just running a regular home cycle when the arm tilts back, it's going to clear the dock. So there's
nothing special about any moves". Measured here instead of assumed: the cycle exactly as Robot Studio runs it (scene
"homing": switches in list order J1, J2, J3, J4, then J0; a joint already past its switch first goes back to 5 deg
before it at full speed; each seek overshoots its switch by up to seek/sample_hz (the Hall reads late) plus the hard
stop's distance), from the parked pose (scene ik.park.pose), every part that moves against the dock station.

    python dock_homing_check.py [--step 1]

Frame: the scene's design frame (y down); the dock is the scene's "dock station v2" model.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf  # noqa: E402

import studio  # noqa: E402
from view_step import read_step  # noqa: E402

SCENE = studio.scene_path()
DOCK = "dock station v2"


def rot(at, axis, deg):
    t = gp_Trsf()
    t.SetRotation(gp_Ax1(gp_Pnt(*at), gp_Dir(*axis)), math.radians(deg))
    return t


def chain(joints, angles):
    """each joint's placement (gp_Trsf) at these angles: nested rotations about each joint's own point (Robot Studio)"""
    out, by = {}, {j["name"]: j for j in joints}
    def place(n):
        if n in out:
            return out[n]
        j = by[n]
        t = rot(j["at"], j["axis"], angles[n])
        if j.get("parent"):
            t = place(j["parent"]).Multiplied(t)
        out[n] = t
        return t
    for n in by:
        place(n)
    return out


def stages(sc, start):
    """[(label, joint, from, to, others)]: each stage sweeps one joint over everything it can reach, the rest held"""
    hm, mo = sc["homing"], sc["ik"]["motion"]
    lim = {j["name"]: (j["min"], j["max"]) for j in sc["joints"]}
    hz = hm.get("sample_hz", 5.0)
    q, out = dict(start), []
    for sw in hm["switches"]:
        n, d, at = sw["joint"], sw.get("dir", 1), sw["trips_at_deg"]
        a = mo[n]["a"] * sw.get("hard_stop", 4.0)
        over = sw["seek"] / hz + sw["seek"] ** 2 / (2 * a) + 1.0       # late read + hard stop + 1 spare
        pre = at - d * 5.0 if (q[n] - at) * d > 0 else q[n]            # past its switch: back to 5 before it first
        far = min(lim[n][1], max(lim[n][0], at + d * over))
        out.append((f"{n.split()[0]} {q[n]:.1f} -> {pre:.1f} -> seek, overshoot to {far:.1f} -> home {at:.1f}",
                    n, min(q[n], pre, far), max(q[n], pre, far), dict(q)))
        q[n] = at
    homes = {j["name"]: j.get("home", 0) for j in sc["joints"]}
    out.append(("to the home pose", None, 0, 1, (dict(q), homes)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", type=float, default=1.0, help="degrees between samples")
    a = ap.parse_args()
    sc = json.load(open(SCENE, encoding="utf-8"))
    joints = sc["joints"]
    models = {m["name"]: m for m in sc["models"] if isinstance(m, dict) and "name" in m}
    dock = [s for _, s, _ in read_step(models[DOCK]["path"])]
    dock = dock[0] if len(dock) == 1 else cq.Compound.makeCompound(dock)
    db = dock.BoundingBox()
    moving = []                                                     # (joint, model name, solid name, solid)
    for j in joints:
        for mn in j["models"]:
            m = models[mn]
            off = m.get("offset", [0, 0, 0])
            for sn, s, _ in read_step(m["path"]):
                if any(off):
                    s = s.translate(cq.Vector(*off))
                moving.append((j["name"], mn, sn, s))
    print(f"dock: {DOCK}, x {db.xmin:.0f}..{db.xmax:.0f} y {db.ymin:.0f}..{db.ymax:.0f} z {db.zmin:.0f}..{db.zmax:.0f}; "
          f"{len(moving)} moving solids on {len(joints)} joints")

    def worst_at(angles, pad=15.0):
        T = chain(joints, angles)
        best = (1e9, None)
        for jn, mn, sn, s in moving:
            p = cq.Shape.cast(s.wrapped.Moved(_loc(T[jn])))
            b = p.BoundingBox()
            if b.xmin > db.xmax + pad or b.xmax < db.xmin - pad or b.ymin > db.ymax + pad or b.ymax < db.ymin - pad \
                    or b.zmin > db.zmax + pad or b.zmax < db.zmin - pad:
                continue
            v = p.intersect(dock).Volume()
            g = -v if v > 1e-3 else p.distance(dock)
            if g < best[0]:
                best = (g, f"{mn}: {sn}")
        return best

    park = sc["ik"]["park"]["pose"]
    g, who = worst_at(park)
    print(f"parked ({', '.join(f'{k.split()[0]} {v:g}' for k, v in park.items())}): closest {g:.2f} mm ({who})")
    fails = 0
    for label, jn, lo, hi, held in stages(sc, park):
        worst = (1e9, None, None)
        if jn is None:                                              # the last move: straight to the home pose
            fr, to = held
            for k in range(21):
                ang = {n: fr[n] + (to[n] - fr[n]) * k / 20 for n in fr}
                g, who = worst_at(ang)
                if g < worst[0]:
                    worst = (g, who, f"{k * 5}%")
        else:
            n = max(2, int(math.ceil((hi - lo) / a.step)))
            for k in range(n + 1):
                ang = dict(held)
                ang[jn] = lo + (hi - lo) * k / n
                g, who = worst_at(ang)
                if g < worst[0]:
                    worst = (g, who, f"{jn.split()[0]} {ang[jn]:.1f}")
        g, who, where = worst
        bad = g < 0.5
        fails += bad
        txt = "nothing within 15 mm" if who is None else (f"OVERLAP {-g:.1f} mm^3" if g < 0 else f"closest {g:.1f} mm") + \
            f" ({who}, at {where})"
        print(f"  {'HIT ' if bad else 'ok  '} {label}: {txt}")
    print("a plain home cycle from the dock " + ("CLEARS it" if not fails else f"HITS it in {fails} stage(s)"))


def _loc(t):
    from OCP.TopLoc import TopLoc_Location
    return TopLoc_Location(t)


if __name__ == "__main__":
    main()

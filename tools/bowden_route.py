"""The claw's Bowden tube: its route through the arm, checked against the real parts over the arm's whole motion.

Robot arm control and kinematics, 2026-11-21. Mike: "We still need to move that tube ... I want that tube moved and
internal wire" / "[the tube] is running right through the middle of motors and stuff" / "new anchor points" - the
chain-drive arm put motors, pulleys and belts on top of every link, where the old route ran (2026-11-05, "along the
OUTSIDE of the arch"). The servo now sits on the shelf of the yoke's -Y cheek (swivel_chain.py, the gearbox session).

The route is the viewer's (viewer.html bowdenUpdate): a cubic Bezier chain through anchors, each in its joint's frame
(as built: every joint at 0), rings taking the tangent of their neighbours, ends taking d x 3k. This script copies that
math, poses the arm (scene.json joints: pivot at 'at', turn about 'axis', parents), and measures:
  - the tightest bend over every pose (PTFE 4 x 2: min r 30),
  - the closest the tube's SURFACE comes to any part riding a joint (the meshes Robot Studio serves, sampled ~1 per
    1.5 mm^2), with the tube's first 20 mm (out of the fitting) and last 50 mm (into the claw's bore) left out.
Poses: every square and tray spot at 0 / 10 / 60 / 125 up (the viewer's IK, claw vertical) plus a grid over J1..J3
with J4 holding the claw vertical, and home.

    python bowden_route.py            # check the route below
    python bowden_route.py --write    # ... and put it in main's scene.json (and the chain studios')
"""
import argparse
import base64
import json
import math
import os
import sys
import urllib.request

import numpy as np
import trimesh
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "viewer_session", "scene.json")
OTHER = [os.path.join(HERE, "viewer_sessions", s, "scene.json") for s in ("chaindrive", "gearbox")]
API = "http://127.0.0.1:8765/api/scene"
OD, MIN_R = 4.0, 30.0
SKIP0, SKIP1 = 20.0, 50.0

# ---------------------------------------------------------------- the route (studio frame, every joint at 0: link A
# along -x from J1 at the origin, y down, the arm's plane z 30.6; the -Y side of the links is +z)
# The old route's shape (a ring behind the shoulder, then rings reaching over each knuckle on the OUTSIDE of the
# arch), moved off the arm's plane to its -Y side, z ~100: outboard of everything there (cheeks 75.6, the far-idler
# nuts 82.1, the yoke's boss flange 88.6, the M3 heads 91.6) and clear of the motors and belts on top (z <= 51.6).
# Rings ON the joint axes were tried first: once a joint folds past ~90 deg the tube doubles back on them (r 2.5).
ZS = 100.0
ROUTE = [
    {"joint": "J0 swivel", "p": [60.0, 34.75, 121.6], "d": [1.0, 0.0, 0.0], "k": 0.55,
     "note": "the PC4-M10 fitting's collet on the yoke's servo shelf (swivel_chain.py): the tube leaves straight back"},
    {"joint": "J0 swivel", "p": [120.0, -35.0, 108.0], "ring": True,
     "note": "guide ring behind and above the shoulder on a post off the yoke: the loop up out of the fitting (r >= 31.6)"},
    {"joint": "J1 shoulder", "p": [-283.4, -71.8, ZS], "ring": True,
     "note": "guide ring on link A's elbow end, reaching over J2 (83 past, 72 out) on the -Y side"},
    {"joint": "J2 elbow", "p": [-444.6, -60.0, ZS], "ring": True,
     "note": "guide ring on link B's end, reaching over J3 (45 past, 60 out) on the -Y side"},
    {"joint": "J3", "p": [-500.0, -70.0, 70.0], "ring": True,
     "note": "guide ring on link C past its motor (20 beyond its end, 18 outboard of it): the tube starts in toward the claw"},
    {"joint": "J4 wrist", "p": [-647.7, 22.3, 30.6], "d": [0.7071, 0.7071, 0.0], "k": 0.46,
     "note": "the claw's lead-in: 23 straight out of the bore"},
    {"joint": "J4 wrist", "p": [-615.2, 54.8, 30.6], "d": [0.7071, 0.7071, 0.0],
     "note": "anchored at the gripper: the top of the PC4-M6 fitting's collet in the claw adapter's 16 bore (claw_adapter.py)"},
]


# ---------------------------------------------------------------- the arm's kinematics (viewer applyJoint)
def rot(axis, deg):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    t = math.radians(deg); c, s = math.cos(t), math.sin(t); x, y, z = a
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


def fk(joints, ang):
    """{joint: 4x4} the joint's frame in the root frame, as posed"""
    out, by = {}, {j["name"]: j for j in joints}

    def get(n):
        if n in out:
            return out[n]
        j = by[n]
        at = np.asarray(j.get("at", [0, 0, 0]), float)
        M = np.eye(4); M[:3, :3] = rot(j["axis"], ang.get(n, 0.0)); M[:3, 3] = at - M[:3, :3] @ at
        P = get(j["parent"]) if j.get("parent") else np.eye(4)
        out[n] = P @ M
        return out[n]
    for j in joints:
        get(j["name"])
    return out


# ---------------------------------------------------------------- the tube (viewer bowdenTangents + bowdenUpdate)
def tube(route, F, n=240):
    A = []
    for pt in route:
        M = F[pt["joint"]]
        p = M[:3, :3] @ np.asarray(pt["p"], float) + M[:3, 3]
        d = (M[:3, :3] @ np.asarray(pt["d"], float)) if pt.get("d") else None
        A.append(dict(p=p, d=None if d is None else d / np.linalg.norm(d), k=pt.get("k", 1 / 3 if pt.get("ring") else 0.4),
                      ring=bool(pt.get("ring"))))
    for i, a in enumerate(A):
        if a["d"] is None and not a["ring"]:
            v = A[min(len(A) - 1, i + 1)]["p"] - A[max(0, i - 1)]["p"]; a["d"] = v / np.linalg.norm(v)
    P = [a["p"] for a in A]
    h = [max(np.linalg.norm(P[i + 1] - P[i]), 1e-6) for i in range(len(P) - 1)]
    T = []
    for i, a in enumerate(A):
        if not a["ring"]:
            T.append(a["d"] * 3 * a["k"]); continue
        hp, hn = (h[i - 1] if i > 0 else 0), (h[i] if i < len(A) - 1 else 0)
        vp = (P[i] - P[i - 1]) / hp if i > 0 else None
        vn = (P[i + 1] - P[i]) / hn if i < len(A) - 1 else None
        t = vp * (hn / (hp + hn)) + vn * (hp / (hp + hn)) if vp is not None and vn is not None else (vp if vp is not None else vn)
        T.append(t / np.linalg.norm(t) * 3 * a["k"])
    segs = [(P[i], P[i] + T[i] * h[i] / 3, P[i + 1] - T[i + 1] * h[i] / 3, P[i + 1]) for i in range(len(P) - 1)]
    tot, cum = sum(h), np.concatenate([[0], np.cumsum(h)])
    pts, rad = [], []
    for k in range(n + 1):
        w = tot * k / n; s = min(np.searchsorted(cum, w, side="right") - 1, len(segs) - 1)
        t = min(1.0, max(0.0, (w - cum[s]) / h[s])); S = segs[s]; u = 1 - t
        b = S[0] * u ** 3 + S[1] * 3 * u * u * t + S[2] * 3 * u * t * t + S[3] * t ** 3
        d1 = 3 * u * u * (S[1] - S[0]) + 6 * u * t * (S[2] - S[1]) + 3 * t * t * (S[3] - S[2])
        d2 = 6 * u * (S[2] - 2 * S[1] + S[0]) + 6 * t * (S[3] - 2 * S[2] + S[1])
        sp = np.linalg.norm(d1); kk = np.linalg.norm(np.cross(d1, d2)) / sp ** 3 if sp > 1e-9 else 0
        pts.append(b); rad.append(1 / kk if kk > 1e-12 else math.inf)
    pts = np.array(pts)
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    return pts, np.array(rad), arc


# ---------------------------------------------------------------- the parts, as Robot Studio serves them
SKIP_MODELS = ("white ", "black ", "board ", "gearbox ", "torque test", "dock station")


def load_parts(sc):
    """{joint or None: KD-tree of surface samples in that joint's frame (as built)}, names alongside"""
    data = json.load(urllib.request.urlopen(API, timeout=300))
    rides = {n: j["name"] for j in sc["joints"] for n in j.get("models", [])}
    pts = {}
    for p in data["parts"]:
        mname = data["models"][p["model"]]["name"]
        jn = rides.get(mname)
        if jn is None or mname.lower().startswith(SKIP_MODELS):
            continue                       # the fixed base and the board stay far below the tube
        v = np.frombuffer(base64.b64decode(p["pos"]), dtype="<f4").reshape(-1, 3).astype(float)
        f = np.frombuffer(base64.b64decode(p["idx"]), dtype="<u4").reshape(-1, 3)
        m = trimesh.Trimesh(v, f, process=False)
        k = int(min(60000, max(300, m.area / 1.5)))
        s, _ = trimesh.sample.sample_surface(m, k, seed=1)
        pts.setdefault(jn, []).append((s, f"{mname}: {p['name']}"))
    out = {}
    for jn, lst in pts.items():
        allp = np.vstack([s for s, _ in lst]); who = np.concatenate([[i] * len(s) for i, (s, _) in enumerate(lst)])
        out[jn] = (cKDTree(allp), who, [n for _, n in lst])
    return out


def poses(sc, grid=True):
    """claw vertical (J4 = wrist_sum - J1 - J2 - J3): every square / tray spot at 4 heights, a J1..J3 grid, home"""
    sys.path.insert(0, HERE)
    import board_pieces as BP
    ik = sc["ik"]; rng = {j["name"]: (j["min"], j["max"]) for j in sc["joints"]}
    L, ax, tool = ik["link"], ik["yaw_axis"], ik["tool"]
    lim = math.radians(min(-rng[ik["chain"][1]][0], -rng[ik["chain"][2]][0]))
    out = []

    def solve(T):
        rho = math.hypot(T[0] - ax[0], T[2] - ax[1]); wx, wz = rho - tool[0], -T[1] - tool[1]
        d, phi = math.hypot(wx, wz), math.atan2(wz, wx); c = (d / L - 1) / 2
        if c > 1:
            return None
        q = -math.acos(max(-1.0, c))
        if -q <= lim + 1e-9:
            q1, q2, q3 = phi - q, q, q
        else:
            q3 = -lim; l2 = 2 * L * math.cos(lim / 2); cb = (d * d - L * L - l2 * l2) / (2 * L * l2)
            if cb < -1:
                return None
            b = -math.acos(min(1.0, cb)); q1 = phi - math.atan2(l2 * math.sin(b), L + l2 * math.cos(b)); q2 = b + lim / 2
        a = {ik["chain"][0]: math.degrees(q1), ik["chain"][1]: math.degrees(q2), ik["chain"][2]: math.degrees(q3)}
        a[ik["wrist"]] = ik["wrist_sum"] - sum(a.values())
        return a if all(rng[n][0] - 1e-6 <= v <= rng[n][1] + 1e-6 for n, v in a.items()) else None
    spots = [BP.square_at(f + r) for f in "abcdefgh" for r in "12345678"]
    for c in ("white", "black"):
        spots += list(BP.tray_spots(c))
    for h in (0.0, 10.0, 60.0, 125.0):
        for p in spots:
            a = solve([p[0], p[1] - h, p[2]])
            if a:
                out.append(a)
    for j1 in (np.linspace(2.5, 147.3, 7) if grid else ()):
        for j2 in np.linspace(-104, 4.6, 6):
            for j3 in np.linspace(-104, 4.6, 6):
                j4 = ik["wrist_sum"] - (j1 + j2 + j3)
                if 9 <= j4 <= 171:
                    out.append({"J1 shoulder": j1, "J2 elbow": j2, "J3": j3, "J4 wrist": j4})
    out.append(dict(next(p for p in sc["poses"] if p["name"] == "home")["joints"]))
    return out


def check(route, sc, parts, ps):
    worst_r, worst_c, lens = (math.inf, None), (math.inf, None), []
    for a in ps:
        F = fk(sc["joints"], a)
        P, R, arc = tube(route, F)
        lens.append(arc[-1])
        i = int(np.argmin(R))
        if R[i] < worst_r[0]:
            worst_r = (R[i], a, P[i])
        keep = (arc >= SKIP0) & (arc <= arc[-1] - SKIP1)
        Q = P[keep]
        for jn, (tree, who, names) in parts.items():
            M = F[jn]; L = (Q - M[:3, 3]) @ M[:3, :3]          # into the joint's frame
            dist, idx = tree.query(L)
            k = int(np.argmin(dist)); c = dist[k] - OD / 2
            if c < worst_c[0]:
                worst_c = (c, a, names[who[idx[k]]], Q[k])
    return worst_r, worst_c, (min(lens), max(lens))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    sc = json.load(open(SCENE))
    ps = poses(sc)
    print(f"{len(ps)} poses; loading the parts from Robot Studio ...")
    parts = load_parts(sc)
    (r, ra, rp), (c, ca, cn, cp), (l0, l1) = check(ROUTE, sc, parts, ps)
    fmt = lambda a: {k.split()[0]: round(v, 1) for k, v in a.items()}           # noqa: E731
    print(f"tightest bend r {r:.1f} (min {MIN_R:.0f}) at {np.round(rp, 1)} pose {fmt(ra)}")
    print(f"closest {c:.1f} mm (tube surface) to {cn} at {np.round(cp, 1)} pose {fmt(ca)}")
    print(f"length {l0:.0f}..{l1:.0f} mm")
    if args.write:
        for path in [SCENE] + [p for p in OTHER if os.path.exists(p)]:
            d = json.load(open(path))
            if not d.get("bowden"):
                continue
            b = d["bowden"]
            b.setdefault("points_column_servo", b["points"])
            b["points"] = ROUTE
            b["why_2026_11_21"] = ("Robot arm control and kinematics (bowden_route.py): the chain-drive arm's motors and belts "
                                   "sit on top of the links where the old route ran, so the tube now leaves the yoke's servo "
                                   f"shelf fitting, loops up behind the shoulder and runs down the arm's -Y side at z {ZS:.0f} "
                                   "through a ring on each joint axis, then in over the wrist to the claw. Over every square "
                                   f"and tray spot: tightest bend r {r:.0f}, closest {c:.1f} mm to {cn}.")
            json.dump(d, open(path, "w"), indent=1)
            print("wrote", path)


if __name__ == "__main__":
    main()

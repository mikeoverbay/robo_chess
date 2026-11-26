#!/usr/bin/env python3
"""
arm_drawing.py - a mechanical drawing of the arm, made from its STEP files (Mike 2026-11-24, for the website's link
preview: "We have a mechanical drawing of our arm, I believe. If not, make one and use that." - there was none).

    python arm_drawing.py [--rho 420] [--up 60] [--side +z|-z]

Side elevation, looking along the joints' axes: every model of the main studio's scene that is the robot (the chain
swivel and everything on the joints) plus the board and the pieces as they start, each placed as Robot Studio places
it (scene.json joints: nested rotations about each joint's point), the arm posed by the viewer's own IK (J2 = J3, the
claw upright) with the grip point --rho from the yaw axis and --up over the board. OCC's polygonal hidden-line removal
(HLRBRep_PolyAlgo) keeps what is seen: outlines heavy, other edges light. The chains inside the links are left out
(the covers hide them). Joint centre marks, the J0 axis, the three 200 links dimensioned, and a title block.

Out: chain_drive/robo_chess_arm_drawing.pdf (vector) and chain_drive/robo_chess_arm_preview.jpg (1200 x 630, the size
link previews use; web_export.py --play puts it beside the page as preview.jpg).
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
from OCP.BRepAdaptor import BRepAdaptor_Curve  # noqa: E402
from OCP.BRepMesh import BRepMesh_IncrementalMesh  # noqa: E402
from OCP.GCPnts import GCPnts_QuasiUniformDeflection  # noqa: E402
from OCP.HLRAlgo import HLRAlgo_Projector  # noqa: E402
from OCP.HLRBRep import HLRBRep_PolyAlgo, HLRBRep_PolyHLRToShape  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt  # noqa: E402

import dock_homing_check as D  # noqa: E402  (chain(): the viewer's joint placements; _loc)
from view_step import read_step  # noqa: E402

ROOT = os.path.dirname(HERE)
SCENE = os.path.join(HERE, "viewer_session", "scene.json")
OUT_PDF = os.path.join(ROOT, "chain_drive", "robo_chess_arm_drawing.pdf")
OUT_JPG = os.path.join(ROOT, "chain_drive", "robo_chess_arm_preview.jpg")
LEAVE_OUT = ("chain +y", "chain -y", "dock station")      # inside the covers / not the robot
BOARD = ("board base sheet", "board top layer", "board tray")
ORANGE, INK, FAINT = "#cc5500", "#1d1f24", "#8a8d96"


def pose(sc, rho, up):
    """the viewer's IK (as park_dock.park_pose): grip point rho from the yaw axis, up over the board; J0 0 (the file
    pose: the arm along -x, over the board's middle)"""
    ik = sc["ik"]
    wx, wz = rho - ik["tool"][0], -(ik["board_y"] - up) - ik["tool"][1]
    d = math.hypot(wx, wz)
    q = -math.acos((d / ik["link"] - 1) / 2)
    q1 = math.atan2(wz, wx) - q
    a = {ik["yaw"]: 0.0, ik["chain"][0]: math.degrees(q1), ik["chain"][1]: math.degrees(q), ik["chain"][2]: math.degrees(q)}
    a[ik["wrist"]] = ik["wrist_sum"] - (a[ik["chain"][0]] + 2 * a[ik["chain"][1]])
    for j in sc["joints"]:
        assert j["min"] <= a[j["name"]] <= j["max"], (j["name"], a[j["name"]])
    return a


def shapes(sc, angles):
    """every solid to draw, placed: (shape, kind)"""
    on = {mn: j["name"] for j in sc["joints"] for mn in j["models"]}
    T = D.chain(sc["joints"], angles)
    home = {h["part"]: h["at"] for h in sc.get("piece_home", [])}
    out, cache = [], {}
    for m in sc["models"]:
        n = m["name"]
        if any(w in n for w in LEAVE_OUT) or not m["path"].lower().endswith((".step", ".stp")):
            continue
        robot = n in on or n.startswith("swivel chain")
        board = n.startswith(BOARD)
        piece = n in home
        if not (robot or board or piece):
            continue
        if m["path"] not in cache:
            cache[m["path"]] = read_step(m["path"])
        sols = cache[m["path"]]
        keep = m.get("solids")
        off = m.get("offset", [0, 0, 0])
        for i, (sn, s, _) in enumerate(sols):
            if keep is not None and i not in keep:
                continue
            if any(off):
                s = s.translate(cq.Vector(*off))
            if piece:
                s = s.translate(cq.Vector(*home[n]))
            if n in on:
                s = cq.Shape.cast(s.wrapped.Moved(D._loc(T[on[n]])))
            out.append((s, "board" if (board or piece) else "robot"))
    return out, T


def hlr(items, side):
    """visible edges seen along z: [(polyline [(x, y)...], outline?)] in the design frame's x, y (y down)"""
    d = gp_Dir(0, 0, 1 if side == "+z" else -1)
    proj = HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), d, gp_Dir(1, 0, 0) if side == "+z" else gp_Dir(-1, 0, 0)))
    algo = HLRBRep_PolyAlgo()
    for s, _ in items:
        BRepMesh_IncrementalMesh(s.wrapped, 0.15, False, 0.25, True)
        algo.Load(s.wrapped)
    algo.Projector(proj)
    algo.Update()
    to = HLRBRep_PolyHLRToShape()
    to.Update(algo)
    lines = []
    for comp, outline in ((to.VCompound(), False), (to.OutLineVCompound(), True)):
        if comp.IsNull():
            continue
        ex = TopExp_Explorer(comp, TopAbs_EDGE)
        while ex.More():
            c = BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
            g = GCPnts_QuasiUniformDeflection(c, 0.05)
            if g.IsDone() and g.NbPoints() > 1:
                pts = [g.Value(k) for k in range(1, g.NbPoints() + 1)]
                lines.append(([(p.X(), p.Y()) for p in pts], outline))
            ex.Next()
    flip = 1 if side == "+z" else -1                     # back to the design x either way
    return [([(flip * x, y) for x, y in pl], o) for pl, o in lines]


def joint_xy(T, name, local):
    p = gp_Pnt(*local).Transformed(T[name])
    return p.X(), p.Y()


def draw(lines, sc, T, angles, rho, up):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Circle, FancyBboxPatch, Rectangle

    fig = plt.figure(figsize=(12, 6.3), dpi=100)
    fig.patch.set_facecolor("white")
    # the sheet: border and a thin inner frame
    fr = fig.add_axes([0, 0, 1, 1]); fr.set_axis_off(); fr.set_xlim(0, 1200); fr.set_ylim(0, 630)
    fr.add_patch(Rectangle((10, 10), 1180, 610, fill=False, lw=2.0, ec=INK))
    fr.add_patch(Rectangle((16, 16), 1168, 598, fill=False, lw=0.6, ec=INK))
    for k in range(1, 12):                                         # zone ticks along the border
        fr.plot([10 + k * 1180 / 12] * 2, [10, 16], color=INK, lw=0.6)
        fr.plot([10 + k * 1180 / 12] * 2, [614, 620], color=INK, lw=0.6)
    ax = fig.add_axes([0.025, 0.045, 0.66, 0.91]); ax.set_axis_off()
    thin = [pl for pl, o in lines if not o]
    heavy = [pl for pl, o in lines if o]
    ax.add_collection(LineCollection([[(x, -y) for x, y in pl] for pl in thin], colors=INK, linewidths=0.55))
    ax.add_collection(LineCollection([[(x, -y) for x, y in pl] for pl in heavy], colors=INK, linewidths=1.1))
    xs = [x for pl, _ in lines for x, _ in pl]; ys = [-y for pl, _ in lines for _, y in pl]
    pad = 25
    ax.set_xlim(min(xs) - pad, max(xs) + pad); ax.set_ylim(min(ys) - pad, max(ys) + 60)
    ax.set_aspect("equal")
    # the J0 axis (dash-dot) and the joint centre marks with their names
    yb = -sc["ik"]["board_y"]                                      # the J0 axis: up through the base to J1 and a little over
    ax.plot([0, 0], [yb - 40, 55], color=ORANGE, lw=0.8, dashes=(10, 3, 2, 3))
    ax.text(6, yb - 32, "J0", color=ORANGE, fontsize=10, fontweight="bold", va="bottom")
    js = {"J1": joint_xy(T, "J1 shoulder", (0, 0, 0)), "J2": joint_xy(T, "J2 elbow", (-200, 0, 0)),
          "J3": joint_xy(T, "J3", (-400, 0, 0)), "J4": joint_xy(T, "J4 wrist", (-600, 0, 0))}
    for k, (x, y) in js.items():
        y = -y
        ax.add_patch(Circle((x, y), 9, fill=False, ec=ORANGE, lw=0.9))
        ax.plot([x - 16, x + 16], [y, y], color=ORANGE, lw=0.7); ax.plot([x, x], [y - 16, y + 16], color=ORANGE, lw=0.7)
    # the three links dimensioned (aligned, offset to the outside of the arch) and the joints named beside them
    names = list(js)
    cx = sum(x for x, _ in js.values()) / 4; cy = sum(-y for _, y in js.values()) / 4 - 150
    for a, b in zip(names, names[1:]):
        (x1, y1), (x2, y2) = js[a], js[b]
        y1, y2 = -y1, -y2
        L = math.hypot(x2 - x1, y2 - y1)
        ux, uy = (x2 - x1) / L, (y2 - y1) / L
        nx, ny = -uy, ux
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        if (mx - cx) * nx + (my - cy) * ny < 0:
            nx, ny = -nx, -ny
        off = 78
        p1, p2 = (x1 + nx * off, y1 + ny * off), (x2 + nx * off, y2 + ny * off)
        for (x, y), (px, py) in (((x1, y1), p1), ((x2, y2), p2)):
            ax.plot([x + nx * 14, px + nx * 6], [y + ny * 14, py + ny * 6], color=FAINT, lw=0.5)
        ax.annotate("", xy=p1, xytext=p2, arrowprops=dict(arrowstyle="<|-|>", lw=0.7, color=INK, shrinkA=0, shrinkB=0,
                                                          mutation_scale=7))
        ang = math.degrees(math.atan2(uy, ux))
        ang = ang - 180 if ang > 90 else ang + 180 if ang < -90 else ang
        ax.text((p1[0] + p2[0]) / 2 + nx * 10, (p1[1] + p2[1]) / 2 + ny * 10, "200", rotation=ang, ha="center",
                va="center", fontsize=10, color=INK, bbox=dict(fc="white", ec="none", pad=0.6))
    for k, (x, y) in js.items():
        y = -y
        nx, ny = x - cx, y - cy
        L = math.hypot(nx, ny) or 1
        ax.text(x - nx / L * 34, y - ny / L * 34, k, color=ORANGE, fontsize=10, fontweight="bold", ha="center",
                va="center")
    # the title block
    tb = fig.add_axes([0.705, 0.045, 0.27, 0.91]); tb.set_axis_off(); tb.set_xlim(0, 1); tb.set_ylim(0, 1)
    tb.text(0.0, 0.965, "ROBO CHESS", fontsize=30, fontweight="bold", color=ORANGE, va="top")
    tb.text(0.0, 0.855, "3D-printed robot arm that plays\nchess on a real board", fontsize=13.5, color=INK, va="top",
            linespacing=1.3)
    specs = ["5 joints: swivel, shoulder, elbow, J3, wrist",
             "NEMA 17 steppers driving SFU1204 ball",
             "screws and #25 roller chains",
             "Printed 5.25:1 herringbone wrist gearbox",
             "Servo claw",
             "Links 3 × 200 mm"]
    y = 0.69
    for s in specs:
        tb.text(0.0, y, s, fontsize=10.5, color=INK, va="top"); y -= 0.052
    tb.add_patch(FancyBboxPatch((0.0, 0.175), 1.0, 0.135, boxstyle="round,pad=0.0,rounding_size=0.02", fc=ORANGE,
                                ec="none"))
    tb.text(0.5, 0.2425, "Play it in your browser", fontsize=15, fontweight="bold", color="white", ha="center",
            va="center")
    # the drawing's own block: what, scale, from what
    tb.add_patch(Rectangle((0.0, 0.0), 1.0, 0.13, fill=False, ec=INK, lw=0.8))
    tb.plot([0.0, 1.0], [0.065, 0.065], color=INK, lw=0.5); tb.plot([0.62, 0.62], [0.0, 0.13], color=INK, lw=0.5)
    tb.text(0.03, 0.097, "ARM - SIDE ELEVATION", fontsize=8.5, color=INK, va="center", fontweight="bold")
    tb.text(0.03, 0.032, "drawn from the CAD model", fontsize=8, color=FAINT, va="center")
    tb.text(0.65, 0.097, "UNITS mm", fontsize=8.5, color=INK, va="center")
    tb.text(0.65, 0.032, time.strftime("%Y-%m-%d"), fontsize=8, color=FAINT, va="center")
    fig.savefig(OUT_PDF)
    fig.savefig(OUT_JPG, dpi=100, pil_kwargs={"quality": 92})
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rho", type=float, default=420.0, help="grip point from the yaw axis, mm")
    ap.add_argument("--up", type=float, default=60.0, help="grip point over the board, mm")
    ap.add_argument("--side", default="+z", choices=("+z", "-z"), help="which side the arm is seen from")
    a = ap.parse_args()
    sc = json.load(open(SCENE, encoding="utf-8"))
    angles = pose(sc, a.rho, a.up)
    print("pose:", {k: round(v, 1) for k, v in angles.items()}, flush=True)
    t = time.time()
    items, T = shapes(sc, angles)
    print(f"{len(items)} solids placed ({time.time() - t:.0f} s)", flush=True)
    t = time.time()
    lines = hlr(items, a.side)
    print(f"hidden lines removed: {len(lines)} visible edges ({time.time() - t:.0f} s)", flush=True)
    draw(lines, sc, T, angles, a.rho, a.up)
    print(OUT_PDF); print(OUT_JPG)


if __name__ == "__main__":
    main()

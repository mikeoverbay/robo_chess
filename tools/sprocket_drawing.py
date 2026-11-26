#!/usr/bin/env python3
"""
sprocket_drawing.py - the working drawing (PDF) of the joint sprocket Mike cuts on his CNC (2026-11-20):
"All right, make me a working PDF drawing of the gear, the cog."

    python sprocket_drawing.py [--png preview.png]

#25 / 25H chain, 25 teeth, ANSI B29.1 tooth form (Machinery's Handbook construction: seating curve R about the
roller seat a, working curve E about c, straight yz, topping curve F about b), cut from 1/8 in 6061 plate with the
tooth ring faced on one side to 0.105-0.110 in. His process (same day): blanks first (bore, 4 holes, rough OD),
then each blank on a fixture (20 mm spigot + 4 x M3), face the ring, profile the teeth with a 3 mm end mill.
The bore and hole pattern are the drum's (chain_drive drum, to be modelled): 20 mm bore, 4 x M3 on a 32 mm circle.
"""
import argparse
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "chain_drive", "joint_sprocket_25T.pdf")
IN = 25.4

# ---------------------------------------------------------------- the sprocket (mm)
N = 25
P, DR = 0.250 * IN, 0.130 * IN                     # #25: pitch, roller
R = (1.005 * 0.130 + 0.003) * IN / 2               # seating curve radius
A = math.radians(35 + 60 / N)
B = math.radians(18 - 56 / N)
E = (1.3025 * 0.130 + 0.0015) * IN                 # working curve radius
F = (0.130 * (0.8 * math.cos(B) + 1.4 * math.cos(math.radians(17 - 64 / N)) - 1.3025) - 0.0015) * IN   # topping
M, T = 0.8 * DR * math.cos(A), 0.8 * DR * math.sin(A)          # a -> c
W, V = 1.4 * DR * math.cos(math.pi / N), 1.4 * DR * math.sin(math.pi / N)   # a -> b (along the line of centres)
YZ = 0.130 * (1.4 * math.sin(math.radians(17 - 64 / N)) - 0.8 * math.sin(B)) * IN
XY = (2.605 * 0.130 + 0.003) * math.sin(math.radians(9 - 28 / N)) * IN
PD = P / math.sin(math.pi / N)
OD = P * (0.6 + 1 / math.tan(math.pi / N))
BD = PD - DR
CD = PD * math.cos(math.pi / (2 * N)) - DR         # caliper: odd tooth count, gap to the gap nearest opposite
MOP = PD * math.cos(math.pi / (2 * N)) + DR        # over two roller-size pins in those gaps
BORE, HOLE, BC = 20.0, 2.5, 32.0                   # rev C: M3 TAPPED (2.5 drill) - the arm session's base screws M3 x 30
#                                                    from its boss flange into the sprocket (2026-11-20)
FACE_D, BLANK_D = 43.0, 58.0
STOCK = 0.125 * IN
RING = (0.105 * IN, 0.110 * IN)
TOOL = 3.0


def rot(p, ang):
    c, s = math.cos(ang), math.sin(ang)
    return np.column_stack((p[:, 0] * c - p[:, 1] * s, p[:, 0] * s + p[:, 1] * c))


def flank():
    """the +x flank of the top gap, roller seat a at (0, r): seat -> working curve -> yz -> topping -> the tip
    circle. Returns (points, construction dict)."""
    r = PD / 2
    a = np.array([0.0, r])
    pts = [a + R * np.array([math.cos(t), math.sin(t)]) for t in np.linspace(-math.pi / 2, -A, 40)]
    c = np.array([-M, r + T])
    pts += [c + E * np.array([math.cos(t), math.sin(t)]) for t in np.linspace(-A, -A + B, 25)]
    y = pts[-1]
    d = np.array([-math.sin(-A + B), math.cos(-A + B)])
    z = y + YZ * d
    b = np.array([W, r - V])
    assert abs(np.hypot(*(z - b)) - F) < 0.01, f"topping curve misses z by {np.hypot(*(z - b)) - F:.3f}"
    pts.append(z)
    t0 = math.atan2(z[1] - b[1], z[0] - b[0])
    lo, hi = t0, t0 - math.pi / 2                       # walk clockwise to the tip circle
    for _ in range(60):
        mid = (lo + hi) / 2
        if np.hypot(*(b + F * np.array([math.cos(mid), math.sin(mid)]))) < OD / 2:
            lo = mid
        else:
            hi = mid
    pts += [b + F * np.array([math.cos(t), math.sin(t)]) for t in np.linspace(t0, lo, 25)][1:]
    return np.array(pts), dict(a=a, c=c, b=b, x=pts[39], y=y, z=z, tip=pts[-1])


def outline():
    """the whole profile, clockwise, as one closed polygon"""
    fr, _ = flank()
    fl = (fr * np.array([-1.0, 1.0]))[::-1]
    out = []
    step = 2 * math.pi / N
    for k in range(N):
        g = rot(np.vstack((fl, fr)), -k * step)
        out.append(g)
        t_end = math.atan2(g[-1, 1], g[-1, 0])                         # tip arc to the next gap's left flank
        nxt = rot(fl[:1], -(k + 1) * step)[0]
        t_nxt = math.atan2(nxt[1], nxt[0])
        while t_nxt > t_end:
            t_nxt -= 2 * math.pi
        ts = np.linspace(t_end, t_nxt, 8)[1:-1]
        out.append(np.column_stack((OD / 2 * np.cos(ts), OD / 2 * np.sin(ts))))
    return np.vstack(out)


# ---------------------------------------------------------------- drawing helpers (paper mm)
ARROW = dict(lw=0.3, color="k", shrinkA=0, shrinkB=0, mutation_scale=6)


class Sheet:
    def __init__(self):
        self.fig = plt.figure(figsize=(11, 8.5))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, 279.4)
        self.ax.set_ylim(0, 215.9)
        self.ax.set_aspect("equal")
        self.ax.axis("off")

    def line(self, pts, lw=0.35, color="k", ls="-", **kw):
        pts = np.asarray(pts, float)
        return self.ax.plot(pts[:, 0], pts[:, 1], lw=lw, color=color, ls=ls, solid_capstyle="butt", **kw)[0]

    def circle(self, c, r, lw=0.35, ls="-", color="k"):
        self.ax.add_patch(Circle(c, r, fill=False, lw=lw, ls=ls, edgecolor=color))

    def text(self, x, y, s, size=5.4, ha="left", va="center", **kw):
        self.ax.text(x, y, s, fontsize=size, ha=ha, va=va, family="DejaVu Sans", linespacing=1.15, **kw)

    def leader(self, tip, at, s, size=5.2):
        """arrow onto tip from `at`, a short shoulder pointing away from the tip, then the text"""
        tip, at = np.asarray(tip, float), np.asarray(at, float)
        right = tip[0] < at[0]
        sh = 3.0 if right else -3.0
        self.ax.annotate("", xy=tuple(tip), xytext=tuple(at),
                         arrowprops=dict(arrowstyle="-|>,head_length=0.7,head_width=0.25", **ARROW))
        self.line([at, at + (sh, 0)], lw=0.3)
        self.text(at[0] + sh + (0.7 if right else -0.7), at[1], s, size=size, ha="left" if right else "right")

    def dim(self, p1, p2, off, s, size=5.2, vertical=False):
        p1, p2 = np.array(p1, float), np.array(p2, float)
        sg = 1.0 if off > 0 else -1.0
        if vertical:
            x = p1[0] + off
            self.line([p1, (x + sg * 1.2, p1[1])], lw=0.25)
            self.line([p2, (x + sg * 1.2, p2[1])], lw=0.25)
            a, b = (x, p1[1]), (x, p2[1])
        else:
            y = p1[1] + off
            self.line([p1, (p1[0], y + sg * 1.2)], lw=0.25)
            self.line([p2, (p2[0], y + sg * 1.2)], lw=0.25)
            a, b = (p1[0], y), (p2[0], y)
        self.ax.annotate("", xy=a, xytext=b,
                         arrowprops=dict(arrowstyle="<|-|>,head_length=0.7,head_width=0.25", **ARROW))
        if vertical:
            self.text(x + sg * 1.2, (a[1] + b[1]) / 2, s, size=size, ha="left" if off > 0 else "right")
        else:
            self.text((a[0] + b[0]) / 2, y + sg * 1.9, s, size=size, ha="center")


def f2(v, d=2):
    """round half up (3.175 -> 3.18)"""
    q = 10 ** d
    return f"{math.floor(v * q + 0.5 + 1e-9) / q:.{d}f}"


def mm_in(v, d=2, di=3):
    return f"{f2(v, d)} [{f2(v / IN, di)}]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", help="also save a preview picture")
    args = ap.parse_args()
    prof = outline()
    fr, k = flank()
    S = Sheet()
    ax = S.ax
    S.line([(6, 6), (273.4, 6), (273.4, 209.9), (6, 209.9), (6, 6)], lw=0.7)          # border
    chain_ls = (0, (10, 2, 1.5, 2))

    # ------------------------------------------------ front view, 2:1, faced side towards you
    C, s = np.array([85.0, 134.0]), 2.0
    P_ = lambda p: C + s * np.asarray(p, float)                                       # noqa: E731
    q = lambda deg, r: P_((r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))))   # noqa: E731
    ax.add_patch(Polygon(P_(prof), closed=True, fill=False, lw=0.55))
    S.circle(C, s * BORE / 2, lw=0.55)
    S.circle(C, s * FACE_D / 2, lw=0.3)
    S.circle(C, s * BC / 2, lw=0.2, ls=chain_ls)
    S.circle(C, s * PD / 2, lw=0.2, ls=chain_ls)
    S.circle(C, s * BLANK_D / 2, lw=0.25, ls=(0, (10, 2, 1.5, 2, 1.5, 2)), color="0.45")
    holes = [(BC / 2 * math.cos(math.radians(t)), BC / 2 * math.sin(math.radians(t))) for t in (90, 0, 270, 180)]
    for h in holes:
        hp = P_(h)
        S.circle(hp, s * HOLE / 2, lw=0.55)
        S.line([hp - (5, 0), hp + (5, 0)], lw=0.2, ls=(0, (6, 1.5, 1, 1.5)))
        S.line([hp - (0, 5), hp + (0, 5)], lw=0.2, ls=(0, (6, 1.5, 1, 1.5)))
    S.text(*(P_(holes[0]) + (4.2, 3.0)), "1", size=6.5, weight="bold")
    S.line([C - (64, 0), C + (64, 0)], lw=0.2, ls=chain_ls)
    S.line([C - (0, 60), C + (0, 60)], lw=0.2, ls=chain_ls)
    for yy in (C[1] + 63, C[1] - 63):                                                 # cutting plane A-A
        S.line([(C[0], yy - 2.5), (C[0], yy + 2.5)], lw=0.9)
        S.line([(C[0], yy), (C[0] + 6, yy)], lw=0.5)
        ax.annotate("", xy=(C[0] + 6, yy - (2.2 if yy > C[1] else -2.2)), xytext=(C[0] + 6, yy),
                    arrowprops=dict(arrowstyle="-|>,head_length=0.8,head_width=0.3", **ARROW))
        S.text(C[0] + 8, yy, "A", size=7.5, weight="bold")
    gb = P_((0, PD / 2))
    S.circle(gb, 12.0, lw=0.3)                                                        # detail B
    S.text(gb[0] - 15.5, gb[1] + 8, "B", size=7.5, weight="bold")
    S.leader(q(140, FACE_D / 2), (40, 199), f"Ø{FACE_D:.1f} [{FACE_D / IN:.3f}] FACED STEP")
    S.leader(P_(holes[3]) + (-1.8, 1.8), (40, 186), "4× M3 TAPPED THRU (Ø2.5 DRILL)\nON Ø32 B.C., 90° APART")
    S.leader(q(54.0, OD / 2), (116, 200), f"Ø{mm_in(OD)}\nTIPS (TURNED OD)")     # 54.0 deg: a tooth tip
    S.leader(q(-38, PD / 2), (118, 84), f"Ø{mm_in(PD, 2, 4)}\nPITCH CIRCLE")
    S.leader(q(-128, BLANK_D / 2), (40, 76), "Ø58 BLANK, ROUGH\n(PHANTOM)")
    S.leader(q(215, BORE / 2), (40, 92), f"Ø{BORE:.2f} BORE\nSLIP FIT ON THE Ø20 SPIGOT")
    S.text(C[0], 66.5, "FRONT VIEW  2:1  (FACED SIDE)", size=7, ha="center", weight="bold")
    S.text(C[0], 62.5, "HOLE 1 ON A TOOTH-GAP CENTRELINE", size=5.4, ha="center")

    # ------------------------------------------------ section A-A, 4:1: axis to the tooth tip on the hole-3 side
    s4, O = 4.0, np.array([14.0, 20.0])                      # O: the axis, at the unfaced face
    Q = lambda r_, z_: O + (s4 * r_, s4 * z_)                # noqa: E731
    ring = sum(RING) / 2
    for poly in ([Q(BORE / 2, 0), Q(BC / 2 - HOLE / 2, 0), Q(BC / 2 - HOLE / 2, STOCK), Q(BORE / 2, STOCK)],
                 [Q(BC / 2 + HOLE / 2, 0), Q(OD / 2, 0), Q(OD / 2, ring), Q(FACE_D / 2, ring), Q(FACE_D / 2, STOCK),
                  Q(BC / 2 + HOLE / 2, STOCK)]):
        ax.add_patch(Polygon(poly, closed=True, fill=False, lw=0.55, hatch="////", edgecolor="k"))
    S.line([Q(BC / 2 - HOLE / 2, 0), Q(BC / 2 + HOLE / 2, 0)], lw=0.55)              # the hole's far edges
    S.line([Q(BC / 2 - HOLE / 2, STOCK), Q(BC / 2 + HOLE / 2, STOCK)], lw=0.55)
    S.line([Q(0, -1.2), Q(0, STOCK + 1.2)], lw=0.2, ls=chain_ls)
    S.line([Q(BC / 2, -1.0), Q(BC / 2, STOCK + 1.0)], lw=0.2, ls=(0, (6, 1.5, 1, 1.5)))
    S.dim(Q(OD / 2, 0), Q(OD / 2, ring), 5.5, f"{f2(RING[0])}–{f2(RING[1])}\n[0.105–0.110]\nTOOTH RING", vertical=True)
    S.dim(Q(BORE / 2, 0), Q(BORE / 2, STOCK), -5.5, f"{mm_in(STOCK)}\nSTOCK", vertical=True)
    S.leader(Q(FACE_D / 2 + 3, ring), (112, 44), f"FACE {f2(0.015 * IN)}–{f2(0.020 * IN)}\n[0.015–0.020] DEEP,\nONE SIDE")
    S.dim(Q(0, STOCK), Q(FACE_D / 2, STOCK), 5.5, f"R{FACE_D / 2:.1f} (Ø{FACE_D:.0f})")
    S.dim(Q(0, 0), Q(OD / 2, 0), -6.5, f"R{f2(OD / 2)} (Ø{f2(OD)}) TIP")
    S.text(O[0] + 1, O[1] - 3.2, "AXIS", size=5.0)
    S.text(O[0], 49.5, "SECTION A–A  4:1", size=7, weight="bold")

    # ------------------------------------------------ detail B, 7:1: one gap, the tooth after it, the next seat
    box = (151.0, 140.0, 120.0, 67.0)
    ax.add_patch(Rectangle(box[:2], box[2], box[3], fill=False, lw=0.3))
    s7, D0 = 7.0, np.array([190.0, 168.0])                   # D0: paper point of the seat a
    r = PD / 2
    loc = lambda p: D0 + s7 * (np.asarray(p, float) - (0.0, r))   # noqa: E731
    clip = Rectangle(box[:2], box[2], box[3], transform=ax.transData)
    S.line(loc(prof), lw=0.6).set_clip_path(clip)
    for rad, ls in ((PD / 2, chain_ls), (OD / 2, (0, (2, 1.5)))):
        tt = np.linspace(math.pi / 2 - 0.42, math.pi / 2 + 0.2, 200)
        S.line(loc(np.column_stack((rad * np.cos(tt), rad * np.sin(tt)))), lw=0.2, ls=ls).set_clip_path(clip)
    a_, c_, b_ = loc(k["a"]), loc(k["c"]), loc(k["b"])
    a2 = loc(rot(np.array([k["a"]]), -2 * math.pi / N)[0])
    for p_ in (a_, a2):
        S.circle(p_, s7 * DR / 2, lw=0.3, ls=(0, (6, 1.5, 1.5, 1.5)), color="0.45")      # rollers
    S.circle(a_ + (0, -s7 * (R - TOOL / 2)), s7 * TOOL / 2, lw=0.35, ls=(0, (3, 1.5)), color="tab:blue")
    for p_, lab in ((a_, "a"), (c_, "c"), (b_, "b"), (a2, "")):
        S.line([p_ - (1.4, 0), p_ + (1.4, 0)], lw=0.3)
        S.line([p_ - (0, 1.4), p_ + (0, 1.4)], lw=0.3)
        if lab:
            S.text(p_[0] + 1.0, p_[1] + 1.7, lab, size=5.8, style="italic")
    S.line([a_, a2], lw=0.2, ls=(0, (6, 1.5, 1, 1.5)))
    mid = (a_ + a2) / 2
    S.text(mid[0] + 2, mid[1] - 3.0, f"P {mm_in(P)}", size=5.0, rotation=-7.2)
    S.text(153, 203.5, "DETAIL B  7:1  — ANSI B29.1 TOOTH FORM", size=7, weight="bold")
    S.leader(a_ + np.array([-0.7, 0.7]) * s7 * DR / 2, (186, 195), f"ROLLER Ø{f2(DR)} [0.130] (PHANTOM)")
    S.leader(loc(fr[-8]), (226, 196), f"R{f2(F)} [{f2(F / IN, 4)}] TOPPING\nCURVE, ABOUT b")
    S.leader(loc(k["z"]), (226, 184), f"STRAIGHT y–z\n{f2(YZ)} [{f2(YZ / IN, 4)}]")
    S.leader(loc(k["c"] + E * np.array([math.cos(-A + B / 2), math.sin(-A + B / 2)])), (226, 156),
             f"R{f2(E)} [{f2(E / IN, 4)}] WORKING\nCURVE, ABOUT c")
    S.leader(loc(k["a"] + R * np.array([math.cos(-2.0), math.sin(-2.0)])), (165, 150),
             f"R{f2(R)} [{f2(R / IN, 4)}]\nSEAT, ABOUT a")
    S.leader(a_ + (s7 * TOOL / 2 * 0.71, -s7 * (R - TOOL / 2) - s7 * TOOL / 2 * 0.71), (206, 145),
             "Ø3 END MILL FITS THE SEAT\n(R1.50 < R1.70)")

    # ------------------------------------------------ data, notes, title block
    data = [("CHAIN", "#25 / 25H (ANSI 25)"), ("TEETH", f"{N}"), ("PITCH", mm_in(P)),
            ("ROLLER", f"Ø{mm_in(DR)}"), ("PITCH DIA", f"Ø{mm_in(PD, 2, 4)}"), ("TIP DIA (OD)", f"Ø{mm_in(OD)}"),
            ("BOTTOM DIA", f"Ø{mm_in(BD)}  (PITCH − ROLLER)"),
            ("CALIPER DIA", f"{mm_in(CD)}  GAP TO THE GAP NEAREST OPPOSITE"),
            ("OVER PINS", f"{mm_in(MOP)}  Ø3.30 PINS IN THOSE TWO GAPS"),
            ("TOOTH WIDTH", f"{f2(RING[0])}–{f2(RING[1])} [0.105–0.110]"),
            ("SEAT / WORK / TOP", f"R{f2(R)} / R{f2(E)} / R{f2(F)}")]
    S.text(152, 134.5, "SPROCKET DATA  mm [in]", size=6.5, weight="bold")
    for i, (kk, vv) in enumerate(data):
        yy = 130.0 - 3.75 * i
        S.text(152, yy, kk, size=5.3)
        S.text(184, yy, vv, size=5.3)
    notes = [
        "MATERIAL: 6061-T6 ALUMINUM, 1/8 IN PLATE (5052-H32 OK).  QTY 8 (2 PER JOINT, J1–J4) + SPARES: 12 BLANKS.",
        "BLANK: Ø20 BORE, 4× Ø2.5 DRILL + M3 TAP THRU ON Ø32 B.C., ROUGH OD Ø58.",
        "FIXTURE: Ø20 SPIGOT (SLIP FIT), 4× Ø3.4 ON Ø32 B.C.: M3 SCREWS FROM BELOW INTO THE BLANK. PAD Ø42 MAX.",
        "FACE ONE SIDE FROM Ø43 OUT TO A 0.105–0.110 IN TOOTH RING (CHAIN SIDE PLATES CLEAR THE STEP ~0.9).",
        "PROFILE THE TEETH WITH A Ø3 END MILL, LIGHT PASSES (THE RING OVERHANGS THE PAD ~6).",
        "SAME PROGRAM FOR EVERY SPROCKET, HOLE 1 ON A GAP CENTRELINE: THE TWO ON A DRUM LINE UP TOOTH FOR TOOTH.",
        "DEBURR TOOTH FLANKS AND SIDES (CHAIN SIDE PLAY ONLY ~0.4–0.5).  BREAK OTHER EDGES 0.2.",
        "UNTOLERANCED ±0.1.  CHECK WITH THE CALIPER-DIA AND OVER-PINS FIGURES.",
    ]
    S.text(152, 87.0, "NOTES", size=6.5, weight="bold")
    for i, n_ in enumerate(notes):
        S.text(152, 82.5 - 4.2 * i, f"{i + 1}.  {n_}", size=4.8)
    tb = (181.0, 8.0, 92.4, 32.0)
    ax.add_patch(Rectangle(tb[:2], tb[2], tb[3], fill=False, lw=0.6))
    S.line([(tb[0], 27.0), (tb[0] + tb[2], 27.0)], lw=0.3)
    S.line([(tb[0], 15.0), (tb[0] + tb[2], 15.0)], lw=0.3)
    S.line([(tb[0] + 46, 8.0), (tb[0] + 46, 15.0)], lw=0.3)
    S.text(tb[0] + 2, 35.5, "ROBO CHESS — CHAIN DRIVE", size=6.5, weight="bold")
    S.text(tb[0] + 2, 31.0, "JOINT SPROCKET  #25 / 25H, 25 TEETH", size=6.0)
    S.text(tb[0] + 2, 23.0, "UNITS: mm [in]     SCALES: AS SHOWN", size=5.2)
    S.text(tb[0] + 2, 19.0, "chain_drive\\joint_sprocket_25T.pdf  (tools\\sprocket_drawing.py)", size=4.8)
    S.text(tb[0] + 2, 11.5, "DATE: 2026-11-20", size=5.2)
    S.text(tb[0] + 48, 11.5, "REV C     SHEET 1 OF 1", size=5.2)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    S.fig.savefig(OUT)
    if args.png:
        S.fig.savefig(args.png, dpi=300, facecolor="white")      # for sharing where a PDF won't go (Facebook)
    print(f"R {R:.3f}  E {E:.3f}  F {F:.3f}  PD {PD:.3f}  OD {OD:.3f}  BD {BD:.3f}  CD {CD:.3f}  MOP {MOP:.3f}  "
          f"xy {XY:.3f}  yz {YZ:.3f}  gap at the seat {2 * R:.3f}")
    print("wrote", OUT)


if __name__ == "__main__":
    main()

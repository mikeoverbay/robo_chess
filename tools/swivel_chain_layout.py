#!/usr/bin/env python3
"""
swivel_chain_layout.py - top-view layout for a chain-driven swivel (J0), to replace the 4:1 bevel pair (Mike
2026-11-20: "we'll go with the cut gear, cut sprocket, and we'll also use a reducer. And I want the shaft that the
reducer is on, that the chain sprocket is on ... kicked at an angle in the back so that we can have our belt center
line straight across in line with the back of the table ... like a triangle shape" / "reduce ... having so much
sticking out the back"). A sketch to agree the triangle before modelling; swivel_base.py (the main session's) is
untouched.

Swivel frame as swivel_base.py: X toward the board, Y across, yaw axis at the origin. Drawn with the board at the top.
Levels: the big sprocket and the jackshaft's small sprocket on top (under the turntable); the GT2 belt one level
lower, so it can run under the big sprocket; the motor stands under the sprocket's rim.
    python swivel_chain_layout.py      -> chain_drive/swivel_chain_layout.png and .pdf
"""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Polygon  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "chain_drive")

P = 6.35                                        # #25 chain pitch
N_BIG, N_SMALL = 90, 10                         # his CNC-cut sprocket on the spindle; the jackshaft's
Z_GT2_MOTOR, Z_GT2_JACK = 20, 60                # GT2 pulleys: motor, jackshaft
D_EDGE = 122.0                                  # yaw axis -> the board's playing edge (swivel_base.py)
X_BACK = -80.0                                  # the back line: jackshaft and motor centres
Y_JACK, Y_MOTOR = 80.0, -60.0                   # jackshaft to one side, the motor to the other: belt straight across
NEMA = 42.3
WALL = 6.0                                      # the case, round everything


def pd(n):
    return P / math.sin(math.pi / n)


def od(n):
    return P * (0.6 + 1 / math.tan(math.pi / n))


def gt2(n):
    return 2.0 * n / math.pi


def tangents(c1, r1, c2, r2):
    """the two outer tangent lines between circles (open belt / chain)"""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    a = math.atan2(dy, dx)
    b = math.acos((r1 - r2) / d)
    out = []
    for s in (1, -1):
        t = a + s * b
        out.append(((c1[0] + r1 * math.cos(t), c1[1] + r1 * math.sin(t)), (c2[0] + r2 * math.cos(t), c2[1] + r2 * math.sin(t))))
    return out


def convex_hull(points):
    """Andrew's monotone chain"""
    pts = sorted(set(points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def main():
    S, J, M = (0.0, 0.0), (X_BACK, Y_JACK), (X_BACK, Y_MOTOR)
    R_big, R_small = pd(N_BIG) / 2, pd(N_SMALL) / 2
    R_j, R_m = gt2(Z_GT2_JACK) / 2, gt2(Z_GT2_MOTOR) / 2
    c_chain = math.hypot(*J)
    c_belt = abs(Y_JACK - Y_MOTOR)
    links = 2 * c_chain / P + (N_BIG + N_SMALL) / 2 + ((N_BIG - N_SMALL) / (2 * math.pi)) ** 2 * P / c_chain
    belt = 2 * c_belt + math.pi * (R_j + R_m) + (R_j - R_m) ** 2 / c_belt
    ratio = N_BIG / N_SMALL * Z_GT2_JACK / Z_GT2_MOTOR
    step = 1.8 / ratio
    reach = 572.0
    case_r = od(N_BIG) / 2 + WALL

    pt = lambda p: (p[1], p[0])                  # noqa: E731   matplotlib: across = Y, up the page = X (to the board)
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.set_aspect("equal")
    ink, acc, chain_c, belt_c, muted = "#1d2330", "#2f6db5", "#555b66", "#c0582f", "#5a6270"

    # the board
    ax.fill_between([-160, 160], D_EDGE, D_EDGE + 26, color="#efe6d2", zorder=0)
    ax.plot([-160, 160], [D_EDGE, D_EDGE], color=ink, lw=1.2)
    ax.text(0, D_EDGE + 13, "chessboard - its playing edge 122 in front of the swivel axis", ha="center", va="center",
            fontsize=9, color=ink)

    # the case: the convex hull round the big sprocket, the jackshaft's pulley and the motor, WALL out
    pts = [(case_r * math.cos(math.radians(t)), case_r * math.sin(math.radians(t))) for t in range(0, 360, 3)]
    pts += [(J[0] + (R_j + WALL) * math.cos(math.radians(t)), J[1] + (R_j + WALL) * math.sin(math.radians(t)))
            for t in range(0, 360, 8)]
    h = NEMA / 2 + WALL
    pts += [(M[0] + sx * h, M[1] + sy * h) for sx in (-1, 1) for sy in (-1, 1)]
    hull = convex_hull(pts)
    back = min(q[0] for q in hull)
    width = max(q[1] for q in hull) - min(q[1] for q in hull)
    ax.add_patch(Polygon([pt(q) for q in hull], closed=True, fill=False, ec="#8a8f99", lw=1.1, ls="--"))

    # the big sprocket, spindle, the jackshaft's sprocket and pulley, the motor
    ax.add_patch(Circle(pt(S), R_big, fill=False, ec=chain_c, lw=1.8))
    ax.add_patch(Circle(pt(S), od(N_BIG) / 2, fill=False, ec=chain_c, lw=0.6, ls=":"))
    ax.add_patch(Circle(pt(S), 20, fill=False, ec=ink, lw=0.8))
    ax.text(*pt((0, 0)), "spindle\nD40", ha="center", va="center", fontsize=8, color=ink)
    ax.text(*pt((R_big * 0.5, 0)), f"{N_BIG}T #25 - your CNC cut\nPD {pd(N_BIG):.1f}   OD {od(N_BIG):.1f}", ha="center",
            fontsize=9.5, color=chain_c)
    ax.add_patch(Circle(pt(J), R_j, fill=False, ec=belt_c, lw=1.4))
    ax.add_patch(Circle(pt(J), R_small, fill=False, ec=chain_c, lw=1.8))
    ax.add_patch(Circle(pt(M), R_m, fill=False, ec=belt_c, lw=1.4))
    ax.add_patch(FancyBboxPatch((M[1] - NEMA / 2, M[0] - NEMA / 2), NEMA, NEMA, boxstyle="round,pad=0,rounding_size=4",
                                fill=False, ec=ink, lw=1.0))

    # chain and belt runs
    runs = tangents(S, R_big, J, R_small)
    for a, b in runs:
        ax.plot([pt(a)[0], pt(b)[0]], [pt(a)[1], pt(b)[1]], color=chain_c, lw=2.4)
    for a, b in tangents(J, R_j, M, R_m):
        ax.plot([pt(a)[0], pt(b)[0]], [pt(a)[1], pt(b)[1]], color=belt_c, lw=2.0)
    (a, b) = runs[0]
    mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    ax.add_patch(Circle(pt(mid), 7, fill=False, ec=acc, lw=1.3))

    # callouts, out in the margins
    def call(p, xy, text, color=ink):
        ax.annotate(text, pt(p), xytext=xy, fontsize=8.5, color=color, ha="left", va="center",
                    arrowprops=dict(arrowstyle="-", color=color, lw=0.7))
    call(J, (165, -40), f"jackshaft (2 x 608): {N_SMALL}T #25 on top,\n{Z_GT2_JACK}T GT2 one level below")
    call(mid, (165, 20), "chain tensioner idler\n(slack side)", acc)
    call(M, (-40, -150), f"NEMA 17 standing up, one level below\nthe big sprocket (so it can sit under its rim);\n{Z_GT2_MOTOR}T GT2 on its shaft")
    ax.plot([pt(S)[0], pt(J)[0]], [pt(S)[1], pt(J)[1]], color=acc, lw=0.7, ls="--")
    ax.text(*pt((J[0] * 0.45, J[1] * 0.45 + 6)), f"chain C {c_chain:.0f}", color=acc, fontsize=8.5, rotation=-45)
    call((X_BACK - R_j + 1, 15), (215, -150), f"GT2 belt C {c_belt:.0f}, straight across the back,\n"
         "one level below the big sprocket", belt_c)
    ax.text(*pt((back - 6, 40)), f"case (dashed): about {width:.0f} wide; its back {abs(back):.0f} mm behind the axis "
            f"(the bevel base's flange went to 78)", color=muted, fontsize=8, ha="center", va="top")

    txt = (f"Chain {N_BIG}:{N_SMALL} = {N_BIG / N_SMALL:.0f}:1   x   belt {Z_GT2_JACK}:{Z_GT2_MOTOR} = "
           f"{Z_GT2_JACK / Z_GT2_MOTOR:.0f}:1   =   {ratio:.0f}:1    (the bevel pair was 4:1)\n"
           f"Full step {step:.4f} deg = {math.radians(step) * reach:.2f} mm at the claw, {reach:.0f} mm out;"
           f"  1/16 step {math.radians(step / 16) * reach:.3f} mm\n"
           f"Chain about {links:.1f} pitches -> {math.ceil(links / 2) * 2} links, the idler takes up the rest;"
           f"  GT2 belt about {belt:.0f} mm loop (nearest stock length, then the motor's slots)\n"
           f"Full rotation possible, no gears; the turn only fights inertia, so the motor has torque to spare.")
    ax.text(0.0, -0.02, txt, transform=ax.transAxes, fontsize=9, color=ink, va="top",
            bbox=dict(boxstyle="round,pad=0.5", fc="#f4f6fa", ec="#d5dbe5"))
    ax.set_title("Swivel (J0) chain drive - top view, layout for review (board at the top)", fontsize=13, color=ink, loc="left")
    ax.set_xlim(-170, 260)
    ax.set_ylim(back - 75, D_EDGE + 30)
    ax.invert_xaxis()
    ax.axis("off")
    os.makedirs(OUT, exist_ok=True)
    fig.savefig(os.path.join(OUT, "swivel_chain_layout.png"), dpi=170, bbox_inches="tight")
    fig.savefig(os.path.join(OUT, "swivel_chain_layout.pdf"), bbox_inches="tight")
    print(f"ratio {ratio:.1f}:1, chain C {c_chain:.1f} ({links:.1f} pitches), belt C {c_belt:.0f} ({belt:.0f} mm), "
          f"case {width:.0f} wide x {case_r - back:.0f} deep, back {abs(back):.0f} behind the axis")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
drill_template.py - a 1:1 paper template for drilling the base sheet by hand (Robo chessboard design, 2026-11-24).

Mike couldn't get the sheet onto his CNC to cut the pockets ("I can't get this thing onto the machine to cut those
pockets"), so the swivel base and the dock stand on the bare sheet (one_piece_board.py) and he drills their pilots and
the arm's cable hole himself. This prints them at full size on Letter paper, tiled, to tape together and lay on the
sheet:

    python drill_template.py     -> chessboard/drill_template_base_dock.pdf
                                    page 1: the whole template (not to scale), how to use it, every hole's position
                                    pages 2..: the template at 1:1, tiled, with a 10 mm grid to line the pages up by

What's on it, all from one_piece_board.py (so it follows the scene's base and dock when they move):
  - the base's and the dock's outlines where they stand on the sheet, to set the parts on
  - a 1/8" pilot (7 deep) under each of their #8 screws, numbered
  - the arm's 1/2" cable hole, through, under the swivel's wire slot
  - the yaw axis (the turntable's centre)
  - reference lines: the sheet's back edge (trim the paper there and butt it against the edge), the main board's back
    edge (332 in front of it), the board's centre line (400 from either side edge of the sheet)

Drawn as seen from ABOVE, standing at the back edge looking toward the board: the board is up the page, the player's
right (+Y, black's side, where the dock is) on the LEFT.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

import one_piece_board as B  # noqa: E402

OUT = os.path.join(os.path.dirname(HERE), "chessboard", "drill_template_base_dock.pdf")
MM = 1 / 25.4
PAGE = (215.9, 279.4)                       # Letter, mm
WIN = (186.0, 246.0)                        # the 1:1 drawing window on each page, mm (printers leave ~6 mm)
WIN_AT = ((PAGE[0] - WIN[0]) / 2, 20.0)     # its lower-left corner on the page
OVERLAP = 12.0                              # tiles share this much, to tape over
# the template's extent, in the sheet's frame (X toward the board, Y the player's right): from the sheet's back edge to
# the main board's back edge, and wide enough for both parts
X0, X1 = B.BACK, B.TOP_FROM
Y0, Y1 = -130.0, 180.0
INK, GRID, GRID5, REF, PART, HOLE = "#000000", "#d9d9d9", "#a8a8a8", "#0050c8", "#202020", "#d01010"


def page_xy(x, y):
    """sheet (X, Y) -> drawing (u, v) in mm: seen from above, from the back edge: Y to the left, X up the page"""
    return -y, x


def holes():
    """[(label, X, Y, kind)]: the base's screws 1.., the dock's after, the cable hole C"""
    feet = {w: B.footprint(n) for w, n in B.STANDS.items()}
    out, k = [], 1
    for what in ("robot base", "dock"):
        pts = feet[what]["screws"]
        pts = sorted(pts, key=lambda p: (-p[0], p[1]))                 # nearest the board first
        for x, y in pts:
            out.append((str(k), x, y, what))
            k += 1
    x, y, _ = B.CABLE_HOLE
    out.append(("C", x, y, "cable"))
    return feet, out


def side(y):
    return f"{abs(y):.1f} {'L' if y > 0 else 'R'}" if abs(y) > 0.05 else "0.0 (on it)"


def draw(ax, feet, hs, full):
    """everything, in drawing mm (u, v)"""
    u0, u1 = sorted(page_xy(0, Y0)[0:1] + page_xy(0, Y1)[0:1])
    # the 10 mm grid, from the back edge (v) and the centre line (u)
    for g in range(int(math.floor(u0 / 10)) * 10, int(u1) + 11, 10):
        ax.plot([g, g], [X0, X1], color=GRID5 if g % 50 == 0 else GRID, lw=0.5 if g % 50 == 0 else 0.3, zorder=1)
    for g in range(0, int(X1 - X0) + 1, 10):
        v = X0 + g
        ax.plot([u0, u1], [v, v], color=GRID5 if g % 50 == 0 else GRID, lw=0.5 if g % 50 == 0 else 0.3, zorder=1)
        if g % 50 == 0 and 0 < g:
            for uu in range(int(math.ceil(u0 / 50)) * 50, int(u1), 50):
                ax.text(uu + 1, v + 1, f"{g}", fontsize=5, color="#888888", zorder=2, clip_on=True)
    for uu in range(int(math.ceil(u0 / 50)) * 50, int(u1) + 1, 50):
        if uu:
            ax.text(uu + 1, X0 + 3, f"{abs(uu)} {'L' if uu < 0 else 'R'}", fontsize=5, color="#888888", zorder=2, clip_on=True)
    # reference lines, their labels repeated along them so every tile carries its own
    ax.plot([u0, u1], [X0, X0], color=REF, lw=1.6, zorder=5)
    ax.plot([u0, u1], [X1, X1], color=REF, lw=1.6, zorder=5)
    ax.plot([0, 0], [X0, X1], color=REF, lw=1.0, ls=(0, (8, 3)), zorder=5)
    step = 150 if full else 95
    for uu in [u0 + 6 + k * step for k in range(int((u1 - u0) // step) + 1)]:
        ax.text(uu, X0 + 2, "SHEET'S BACK EDGE: trim here, butt against the edge", fontsize=6 if full else 6.5,
                color=REF, weight="bold", zorder=6, clip_on=True)
        ax.text(uu, X1 - 5, f"MAIN BOARD'S BACK EDGE ({X1 - X0:.0f} from the back edge)", fontsize=6 if full else 6.5,
                color=REF, weight="bold", zorder=6, clip_on=True)
    for vv in [X0 + 30 + k * 110 for k in range(3)]:
        ax.text(1.5, vv, "CENTRE LINE: 400 from either side edge of the sheet", fontsize=5.5, color=REF, rotation=90,
                va="bottom", zorder=6, clip_on=True)
    # the parts' outlines
    for what, f in feet.items():
        for o in B.outlines(f):
            uv = [page_xy(x, y) for x, y in o] + [page_xy(*o[0])]
            ax.plot([p[0] for p in uv], [p[1] for p in uv], color=PART, lw=1.0, zorder=7)
    bu, bv = page_xy(-60, 70)
    ax.text(bu, bv, "SWIVEL BASE\n(robot)", ha="center", fontsize=8, color=PART, weight="bold", zorder=8)
    du, dv = page_xy(-185, 114)
    ax.text(du, dv, "DOCK\n(park stand)", ha="center", fontsize=8, color=PART, weight="bold", zorder=8)
    # the yaw axis
    ax.plot([-8, 8], [0, 0], color=PART, lw=0.6, zorder=8)
    ax.plot([0, 0], [-8, 8], color=PART, lw=0.6, zorder=8)
    ax.text(2, 2, "yaw axis", fontsize=5, color=PART, zorder=8, clip_on=True)
    # the holes
    for lab, x, y, kind in hs:
        u, v = page_xy(x, y)
        d = B.CABLE_HOLE[2] if kind == "cable" else B.PILOT_D
        ax.add_patch(plt.Circle((u, v), d / 2, fill=False, color=HOLE, lw=0.8, zorder=9))
        c = d / 2 + 4
        ax.plot([u - c, u + c], [v, v], color=HOLE, lw=0.4, zorder=9)
        ax.plot([u, u], [v - c, v + c], color=HOLE, lw=0.4, zorder=9)
        ax.text(u + c * 0.6, v + c * 0.6, lab, fontsize=9 if full else 7, color=HOLE, weight="bold", zorder=10, clip_on=True)


def scale_bars(fig):
    """a 100 mm bar each way, on the page itself: measure them before drilling"""
    ax = fig.add_axes([0, 0, 1, 1], facecolor="none")
    ax.set_xlim(0, PAGE[0]); ax.set_ylim(0, PAGE[1]); ax.axis("off")
    x0, y0 = WIN_AT[0], 8.0
    ax.plot([x0, x0 + 100], [y0, y0], color=INK, lw=1.2)
    for t in range(0, 101, 10):
        ax.plot([x0 + t, x0 + t], [y0, y0 + (2.5 if t % 50 else 4)], color=INK, lw=0.6)
    ax.text(x0 + 104, y0 - 1, "this bar must measure 100 mm - print at 100 % / Actual size, never Fit", fontsize=6.5)
    return ax


def main():
    feet, hs = holes()
    us = [page_xy(0, Y0)[0], page_xy(0, Y1)[0]]
    U0, U1 = min(us) - 4, max(us) + 4
    V0, V1 = X0 - 4, X1 + 4
    with PdfPages(OUT) as pdf:
        # ---- page 1: the whole thing, how to use it, the hole list
        fig = plt.figure(figsize=(PAGE[0] * MM, PAGE[1] * MM))
        fig.text(0.5, 0.965, "Robo Chess - base sheet drilling template (swivel base + dock)", ha="center", fontsize=12,
                 weight="bold")
        fig.text(0.5, 0.945, "Seen from ABOVE, standing at the sheet's back edge looking toward the board. Pages 2-5: "
                 "full size, tiled.", ha="center", fontsize=7.5)
        ax = fig.add_axes([0.08, 0.40, 0.84, 0.53])
        ax.set_xlim(U0, U1); ax.set_ylim(V0, V1); ax.set_aspect("equal"); ax.axis("off")
        draw(ax, feet, hs, True)
        nc = math.ceil((U1 - U0 - OVERLAP) / (WIN[0] - OVERLAP))
        nr = math.ceil((V1 - V0 - OVERLAP) / (WIN[1] - OVERLAP))
        tiles = []
        for r in range(nr):
            for c in range(nc):
                tu = U0 + c * (WIN[0] - OVERLAP)
                tv = V0 + r * (WIN[1] - OVERLAP)
                tiles.append((f"{'AB'[r]}{c + 1}", tu, tv))
                ax.add_patch(plt.Rectangle((tu, tv), WIN[0], WIN[1], fill=False, ls=":", color="#999999", lw=0.6))
                ax.text(tu + 3, tv + 3, f"page {'AB'[r]}{c + 1}", fontsize=6, color="#777777")
        rows = [f"{'hole':5s}{'what':34s}{'from back edge':>15s}{'from centre line':>19s}"]
        for lab, x, y, kind in hs:
            what = {"robot base": "base: 1/8\" pilot, 7 deep (#8 x 1/2\")",
                    "dock": "dock: 1/8\" pilot, 7 deep (#8)", "cable": "arm cables: 1/2\" hole, THROUGH"}[kind]
            rows.append(f"{lab:5s}{what:34s}{x - X0:>12.1f} mm{side(y):>16s} mm")
        fig.text(0.08, 0.375, "\n".join(rows), fontsize=7, family="monospace", va="top")
        steps = [
            "1. Print pages 2-5 at 100 % (Actual size). Measure the 100 mm bar on each page.",
            "2. Trim each page along its overlap edges, line the 10 mm grid up, tape them together.",
            "3. Trim the whole template on the SHEET'S BACK EDGE line. Butt that line against the sheet's back edge,",
            "   with the dashed centre line 400 mm from either side edge (check: the MAIN BOARD'S BACK EDGE line falls",
            f"   on the main board's back edge, {X1 - X0:.0f} mm in). Tape it down.",
            "4. Centre-punch every hole through the paper; lift the template; drill.",
            "   1-10: 1/8\" pilots, 7 mm deep (tape a flag on the bit). C: 1/2\" all the way through (Forstner,",
            "   finish from below or clamp a scrap under it so the bottom doesn't tear out).",
            "5. Check: set the base and the dock on their outlines - every screw hole over its pilot.",
            "L / R: as seen standing at the back edge looking at the board (L = the player's right, black's side).",
        ]
        fig.text(0.08, 0.20, "\n".join(steps), fontsize=7, va="top")
        fig.text(0.08, 0.025, "Made by tools/drill_template.py from one_piece_board.py (the scene's swivel base and dock); "
                 "sheet 12 mm assumed - if yours is thinner than 8, drill the pilots 1 mm shy of through.",
                 fontsize=5.5, color="#555555")
        pdf.savefig(fig); plt.close(fig)
        # ---- pages 2..: 1:1 tiles
        for name, tu, tv in tiles:
            fig = plt.figure(figsize=(PAGE[0] * MM, PAGE[1] * MM))
            ax = fig.add_axes([WIN_AT[0] / PAGE[0], WIN_AT[1] / PAGE[1], WIN[0] / PAGE[0], WIN[1] / PAGE[1]])
            ax.set_xlim(tu, tu + WIN[0]); ax.set_ylim(tv, tv + WIN[1]); ax.set_aspect("auto"); ax.axis("off")
            draw(ax, feet, hs, False)
            ax.add_patch(plt.Rectangle((tu, tv), WIN[0], WIN[1], fill=False, color="#999999", lw=0.5, ls=":"))
            fig.text(0.5, 0.975, f"page {name} of the drilling template - 1:1 - board up the page, back edge down",
                     ha="center", fontsize=8, weight="bold")
            nbr = []
            r, c = "AB".index(name[0]), int(name[1]) - 1
            if c > 0: nbr.append(f"left: {name[0]}{c}")
            if c < nc - 1: nbr.append(f"right: {name[0]}{c + 2}")
            if r > 0: nbr.append(f"below: {'AB'[r - 1]}{c + 1}")
            if r < nr - 1: nbr.append(f"above: {'AB'[r + 1]}{c + 1}")
            fig.text(0.5, 0.958, f"overlaps {OVERLAP:g} mm with " + ", ".join(nbr), ha="center", fontsize=6.5,
                     color="#555555")
            scale_bars(fig)
            pdf.savefig(fig); plt.close(fig)
    print(f"{OUT}: {len(tiles)} tiles ({nr} x {nc}) at 1:1 + the overview; holes:")
    for lab, x, y, kind in hs:
        print(f"   {lab:2s} {kind:10s} X {x:8.2f} Y {y:8.2f}  ->  {x - X0:6.1f} from the back edge, {side(y)} of the centre")


if __name__ == "__main__":
    main()

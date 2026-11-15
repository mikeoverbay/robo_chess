#!/usr/bin/env python3
"""
wiring_sheet.py - a printable wiring sheet of the chessboard's corner (Robo chessboard design, 2026-11-06).

    python wiring_sheet.py     -> chessboard/wiring_corner_a8.pdf  (letter, landscape)

The Professor: "I would like to inkjet this layout with arrows showing which one of the wires are which ... zoom in on the
top left three by three ... get text in there to show which wires are which" and "label some of the pins for where they
would go on the Arduino".
Drawn from the same numbers as cell_edit.py / causeway.py / back_way.py: squares a6..c8 (files a-c, ranks 6-8), the
causeway along the a-file edge and the back way along rank 8, where all the wiring meets for the Mega 2560.
Seen from ABOVE as if the squares were clear: the robot at the top of the page, the causeway on the left (as you stand at
the human's side looking toward the robot). Working from underneath, left and right swap.
Solid colour = bare wire in a trench on the underside (soldered from below); dashed = insulated wire (top face / ledge).
Mega pins (proposed): ROW (Hall +) rank r -> D(21 + r) (PORTA); LED data rank r -> D(1 + r); column file a..h -> D30..D37
(PORTC; 10k pull-down to GND on each).
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle, Circle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "chessboard", "wiring_corner_a8.pdf")

P = 60.0
HALF, INNER = 29.9, 27.7
LANE = {"ROW": -8.5, "GND": -2.54, "data": 0.0, "5V": 2.54}
COL = {"ROW": "#7b4fd0", "GND": "#111111", "data": "#f08c00", "5V": "#d62728", "column": "#1f6fd0", "white": "#8a8a8a"}
FILES, RANKS = (0, 1, 2), (5, 6, 7)                  # a, b, c ; ranks 6, 7, 8
import causeway as CW  # noqa: E402  (the causeway's own numbers, so the sheet follows it)
CW_EDGE, CW_GND, CW_5V, CW_HOLE = CW.EDGE, CW.X(CW.U_GND), CW.X(CW.U_5V), CW.X(CW.U_HOLE)
CW_B0, CW_B1, CW_OUT = CW.X(CW.CH_U0), CW.X(CW.CH_U1), CW.X(CW.WIDTH)
BW_Y0, BW_Y1, BW_C0, BW_C1 = 450.45, 460.45, 453.35, 458.55
COL_X = 20.6
LW_BARE, LW_INS = 1.6, 1.2


def pin_row(r):
    return f"D{21 + r}"


def pin_led(r):
    return f"D{1 + r}"


def pin_col(f):
    return f"D{30 + f}"


def main():
    fig = plt.figure(figsize=(11, 8.5))
    ax = fig.add_axes([0.0, 0.06, 0.58, 0.86])
    ax.set_aspect("equal")
    ax.set_xlim(-84, 158)
    ax.set_ylim(262, 478)
    ax.axis("off")
    line = lambda xs, ys, c, ls="-", lw=LW_BARE, z=3: ax.plot(xs, ys, color=c, ls=ls, lw=lw, zorder=z,  # noqa: E731
                                                            solid_capstyle="butt")
    tag = {}

    def callout(n, x, y, to=None):
        if to:                                                      # a leader to what it names
            ax.plot([x, to[0]], [y, to[1]], color="#333333", lw=0.5, zorder=10)
            ax.add_patch(Circle(to, 0.5, fc="#333333", zorder=10))
        ax.add_patch(Circle((x, y), 3.0, fc="white", ec="#333333", lw=0.7, zorder=10))
        ax.text(x, y, str(n), ha="center", va="center", fontsize=6.5, zorder=11)
        tag[n] = (x, y)

    # ---- squares
    for i in FILES:
        for j in RANKS:
            cx, cy = i * P, j * P
            ax.add_patch(Rectangle((cx - HALF, cy - HALF), 2 * HALF, 2 * HALF, fc="#faf8f3", ec="#9a9a9a", lw=0.6, zorder=1))
            ax.text(cx - 24, cy + 23, f"{'abcdefgh'[i]}{j + 1}", fontsize=9, color="#555555", zorder=2)
            ax.add_patch(Rectangle((cx - 7.9, cy - 5.35), 15.8, 10.7, fc="#e9e9e9", ec="#777777", lw=0.5, zorder=4))
            ax.add_patch(Rectangle((cx - 2.5, cy - 2.5), 5, 5, fc="white", ec="#555555", lw=0.5, zorder=5))
            # diode recess, diode, its leads
            ax.add_patch(Rectangle((cx + 22.6, cy + 5.2), 3.0, 14.2, fc="none", ec="#999999", lw=0.4, zorder=2))
            ax.add_patch(Rectangle((cx + 22.75, cy + 13.0), 2.7, 5.2, fc="#333333", ec="#333333", zorder=6))
            ax.add_patch(Rectangle((cx + 22.75, cy + 17.0), 2.7, 0.8, fc="#dddddd", ec="none", zorder=7))
            line([cx + 24.1, cx + 24.1], [cy + 13.0, cy + 6.8], "#b07d3a", lw=0.9, z=6)
            line([cx + 24.1, cx + 24.1, cx + COL_X], [cy + 18.2, cy + 24.3, cy + 24.3], "#b07d3a", lw=0.9, z=6)
            ax.add_patch(Rectangle((cx + COL_X - 1.3, cy + 22.5), 2.6, 4.4, fc="none", ec=COL["column"], lw=0.5, zorder=3))
            ax.add_patch(Circle((cx + COL_X, cy + 24.3), 0.9, fc=COL["column"], zorder=7))
            # the Hall (on its peg, above) and its three wires along the ledge troughs, out through F1, down its face
            ax.add_patch(Rectangle((cx - 1.6, cy - 2.0), 3.15, 4.0, fc="none", ec="#444444", lw=0.6, ls=(0, (2, 1)), zorder=8))
            for k, yt, yl, c in (("+", -3.0, LANE["ROW"], COL["5V"]), ("GND", 0.0, LANE["GND"], COL["GND"]),
                                 ("OUT", 3.0, 6.0, COL["white"])):
                line([cx + 1.6, cx + 14, cx + 29.4, cx + 29.4, cx + (25.5 if k != "OUT" else 25.6)],
                     [cy + yt * 0.4, cy + yt, cy + yt, cy + yl, cy + yl], c, ls=(0, (3, 1.5)), lw=LW_INS, z=8)
            for y in (LANE["ROW"], LANE["GND"]):                    # heat-shrink pockets on the Hall rails' joints
                ax.add_patch(Rectangle((cx + 22.6, cy + y - 1.0), 4.4, 2.0, fc="none", ec="#999999", lw=0.4, zorder=2))
                ax.add_patch(Circle((cx + 24.8, cy + y), 0.9, fc="#444444", zorder=7))
            line([cx + 25.6, cx + 24.6], [cy + 6.0, cy + 6.8], COL["white"], ls=(0, (3, 1.5)), lw=LW_INS, z=8)
            ax.add_patch(Circle((cx + 24.3, cy + 6.8), 0.9, fc="#444444", zorder=9))
            ax.plot([cx - 1.2, cx + 1.2], [cy - 1.2, cy + 1.2], color="black", lw=0.8, zorder=9)    # cut the data wire
            ax.plot([cx - 1.2, cx + 1.2], [cy + 1.2, cy - 1.2], color="black", lw=0.8, zorder=9)
    # ---- the rank lanes (bare), through every square and on to the causeway
    for j in RANKS:
        cy = j * P
        for k, y in LANE.items():
            line([CW_EDGE + 0.5 if k == "GND" else CW_HOLE, 158], [cy + y, cy + y], COL[k], z=3)
    # ---- the column wires (insulated) along the files, out the back
    for i in FILES:
        x = i * P + COL_X
        line([x, x], [262, (BW_C0 + BW_C1) / 2], COL["column"], lw=2.0, z=2)
    # ---- the causeway (a-file edge)
    ax.add_patch(Rectangle((CW_OUT, 262), CW_EDGE - CW_OUT, 449.9 - 262, fc="#dbe9f7", ec="#6d95c2", lw=0.6, zorder=0))
    line([CW_GND, CW_GND], [262, 468], COL["GND"], lw=2.0, z=3)
    line([CW_5V, CW_5V], [262, 468], COL["5V"], lw=2.0, z=3)
    ax.add_patch(Rectangle((CW_B1, 262), CW_B0 - CW_B1, 468 - 262, fc="none", ec="#6d95c2", lw=0.6, ls=(0, (3, 2)), zorder=1))
    for j in RANKS:
        cy = j * P
        ax.add_patch(Circle((CW_GND, cy + LANE["GND"]), 2.25, fc="none", ec="#666666", lw=0.5, zorder=4))
        ax.add_patch(Circle((CW_5V, cy + LANE["5V"]), 2.25, fc="none", ec="#666666", lw=0.5, zorder=4))
        for k in ("5V", "ROW", "data"):
            ax.add_patch(Circle((CW_HOLE, cy + LANE[k]), 0.6, fc="white", ec="#333333", lw=0.6, zorder=6))
        line([CW_HOLE, CW_5V], [cy + LANE["5V"], cy + LANE["5V"]], COL["5V"], ls=(0, (3, 1.5)), lw=LW_INS, z=5)
        ax.add_patch(Circle((CW_5V, cy + LANE["5V"]), 0.6, fc="white", ec="#333333", lw=0.6, zorder=6))
        for k in ("ROW", "data"):
            line([CW_HOLE, CW_B0 - 1.0, CW_B0 - 1.0], [cy + LANE[k], cy + LANE[k], 468], COL[k], ls=(0, (3, 1.5)),
                 lw=LW_INS, z=5)
        ax.text(CW_OUT - 1.5, cy - 6.0, f"rank {j + 1}\nROW {pin_row(j + 1)}\ndata {pin_led(j + 1)}", ha="right",
                va="center", fontsize=6.5, color="#333333")
    # ---- the back way (rank-8 edge)
    ax.add_patch(Rectangle((-29.9, BW_Y0), 158 + 29.9, BW_Y1 - BW_Y0, fc="#e4f0d6", ec="#7fae4e", lw=0.6, zorder=0))
    ax.add_patch(Rectangle((-29.9, BW_C0), 158 + 29.9, BW_C1 - BW_C0, fc="none", ec="#7fae4e", lw=0.5, zorder=1))
    line([-29.9, 158], [(BW_C0 + BW_C1) / 2] * 2, COL["column"], lw=2.0, z=2)
    for i in FILES:
        ax.text(i * P + COL_X, BW_Y1 + 1.0, f"col {'abcdefgh'[i]} -> {pin_col(i)}", fontsize=6.5, color=COL["column"],
                ha="center", va="bottom", zorder=12)
    ax.annotate("to the Mega", xy=(-64, 471), xytext=(-24, 471), fontsize=8, va="center",
                arrowprops=dict(arrowstyle="->", lw=0.8))
    ax.annotate("toward the robot", xy=(110, 478), xytext=(110, 467.5), fontsize=8, ha="center", va="bottom",
                arrowprops=dict(arrowstyle="->", lw=0.8))
    # ---- callouts (key on the right)
    a7 = (0.0, 6 * P)
    X, Y = a7
    callout(1, X - 24, Y - 15, (X - 21, Y + LANE["GND"]))
    callout(2, X - 22, Y + 10, (X - 19, Y + LANE["5V"]))
    callout(3, X - 14, Y + 14, (X - 12, Y + LANE["data"]))
    callout(4, X + 4, Y + 14, (X + 0.6, Y + 0.6))
    callout(5, X - 14, Y - 19, (X - 12, Y + LANE["ROW"]))
    callout(6, X - 5, Y + 15, (X - 4, Y + 5.35))
    callout(7, X + 13, Y + 13, (X + 12, Y + 3.0))
    callout(8, X + 16, Y - 19, (X + 24.8, Y + LANE["ROW"]))
    callout(9, X + 7, Y - 15, (X + 24.8, Y + LANE["GND"]))
    callout(10, X + 42, Y + 8, (X + 24.3, Y + 6.8))
    callout(11, X + 42, Y + 16, (X + 25.45, Y + 15.6))
    callout(12, X + 12, Y + 23, (X + COL_X, Y + 24.3))
    callout(13, CW_GND + 0.2, 7 * P + 20)
    callout(14, CW_5V - 0.2, 7 * P + 13)
    callout(15, (CW_B0 + CW_B1) / 2, 7 * P + 26)
    callout(16, 110, (BW_C0 + BW_C1) / 2)
    callout(17, -40, 457)

    key = [
        (1, "GND lane - bare, along the rank (LEDs' and Halls' ground)"),
        (2, "+5 lane - bare, along the rank (LEDs only)"),
        (3, "LED data lane - bare: D out of one LED to D in of the next"),
        (4, "X = cut the data wire here, under every LED\n     (between its DI and DO pads)"),
        (5, "ROW lane - bare: the rank's Hall + (the Mega switches it)"),
        (6, "LED segment, pads down: DI on the left, DO on the right"),
        (7, "Hall wires, insulated, in the ledge troughs: + red, GND black,\n"
            "     signal white; out through the right wall (F1), down its outside"),
        (8, "Hall + onto the ROW lane (heat-shrink pocket)"),
        (9, "Hall GND onto the GND lane (heat-shrink pocket)"),
        (10, "signal (white) onto the diode's plain end, 6 mm out along its lead"),
        (11, "1N4001 diode - stripe (cathode) toward the column wire"),
        (12, "column wire, insulated, along the file -> out the back;\n     diode joint in its pocket"),
        (13, "causeway GND bus (bare, underside): each rank's GND\n     straight in, solder pocket"),
        (14, "causeway +5 bus (bare): rank +5 up a hole, over the GND\n     bus on top, down a hole"),
        (15, "causeway bundle (insulated, top face): each rank's ROW\n     and LED data -> Mega"),
        (16, "back way bundle (insulated): the 8 column wires -> the corner"),
        (17, "the corner: everything meets here for the Mega 2560"),
    ]
    tx = fig.add_axes([0.595, 0.06, 0.40, 0.86])
    tx.axis("off")
    y = 1.0
    tx.text(0.0, y, "Chessboard wiring - corner a6 to c8", fontsize=10, weight="bold", va="top")
    y -= 0.03
    tx.text(0.0, y, "seen from above; robot at the top, causeway on the left", fontsize=7.5, va="top")
    y -= 0.035
    tx.text(0.0, y, "solid colour = bare wire in an underside trench (solder from below)\n"
                    "dashed = insulated wire (top face / ledge troughs)", fontsize=7, va="top", color="#444444")
    y -= 0.07
    for n, s in key:
        tx.text(0.0, y, f"{n:>2}", fontsize=7, va="top", weight="bold")
        tx.text(0.05, y, s, fontsize=7, va="top")
        y -= 0.0185 * (1 + s.count("\n")) + 0.0075
    y -= 0.015
    tx.text(0.0, y, "Mega 2560 pins (proposed)", fontsize=8.5, weight="bold", va="top")
    y -= 0.03
    rows = [("rank", [str(r) for r in range(1, 9)], True), ("ROW (Hall +)", [pin_row(r) for r in range(1, 9)], False),
            ("LED data", [pin_led(r) for r in range(1, 9)], False), None,
            ("file", list("abcdefgh"), True), ("column (signal in)", [pin_col(f) for f in range(8)], False)]
    for r in rows:
        if r is None:
            y -= 0.008
            continue
        name, cells, head = r
        tx.text(0.0, y, name, fontsize=7, va="top", weight="bold")
        for k, c in enumerate(cells):
            tx.text(0.285 + 0.09 * k, y, c, fontsize=7, va="top", ha="center", weight="bold" if head else "normal")
        y -= 0.021
    y -= 0.01
    tx.text(0.02, y, "10k pull-down from each of D30-D37 to GND.  Rows: one HIGH at a time.\n"
                     "+5 bus -> 5 V supply +;  GND bus -> supply - and Mega GND.", fontsize=7, va="top")
    pos = ax.get_position()                       # equal aspect: the tighter of the two fits sets the scale
    scale = min(pos.width * 11 * 25.4 / (158 + 84), pos.height * 8.5 * 25.4 / (470 - 262))
    fig.text(0.02, 0.025, "Robo Chess - chessboard wiring sheet, drawn by wiring_sheet.py from the CAD numbers "
                          f"(cell_edit.py, causeway.py, back_way.py).  Scale {scale:.2f} : 1 (1 mm on the board = "
                          f"{scale:.2f} mm on the page).", fontsize=6.5, color="#666666")
    fig.savefig(OUT)
    print(OUT)


if __name__ == "__main__":
    main()

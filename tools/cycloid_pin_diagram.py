#!/usr/bin/env python3
"""
cycloid_pin_diagram.py - a paper drawing of the gearbox's output pin holes against the cam (Mike 2026-11-18: "draw a
diagram that shows the circle of those pins holes for the transfer into the power shaft in regards to the center cam
... show the circles of all of them ... the lower disc also drawn underneath with a contrast color on paper").

    python cycloid_pin_diagram.py [--out "../cycloid/Output pin holes and cam.pdf"] [--png DIR]

Everything comes from cycloid.py (spread 14), posed at input 0 deg and seen from the lid side:
  page 1  the two discs at 3:1 - disc 2 (upper) in blue over disc 1 (lower) in orange - with the output pins, the
          output pin circle on the motor axis, each disc's hole circle on its own cam, and the little circle each
          hole centre runs round its pin; a 10:1 detail of one pin in its two holes.
  page 2  the cams at 10:1 on the D shaft (the flat faces along the cam line), and the whole thing at 1:1.
Letter paper, print at Actual size: each view has a 10 mm bar to check the scale.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.path import Path  # noqa: E402
from matplotlib.patches import PathPatch, Circle, FancyArrowPatch  # noqa: E402

import cycloid as C  # noqa: E402
import gearbox as G  # noqa: E402

C.set_spread(14.0)
PAGE_W, PAGE_H = 215.9, 279.4                      # letter, mm
E = C.ECC
HOLE_D = C.PIN_D + 2 * E + 2 * C.HOLE_CLEAR        # the hole as it prints (the model adds OUT_HOLE_PRINT for his printer)
HOLE_MODEL = HOLE_D + C.OUT_HOLE_PRINT
R_RING = 52.90 / 2 + C.PIN_D / 2                    # his ring pins as the housing prints them (pin gauge 52.90)
ORANGE, ORANGE_FILL = "#c8560a", "#f8cfa6"         # disc 1, lower
BLUE, BLUE_FILL = "#1d4f9a", "#5b8fd6"             # disc 2, upper (see-through)
STEEL, PIN_GREY, INK = "#4a4a4a", "#c9c9c9", "#111111"
C1, P1 = C.disc_pose(0.0, 1)
C2, P2 = C.disc_pose(0.0, 2)


def ring_xy(cx, cy, r, n=240, cw=False):
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    if cw:
        t = t[::-1]
    return np.stack([cx + r * np.cos(t), cy + r * np.sin(t)], 1)


def ccw(p):
    a = 0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
    return p if a > 0 else p[::-1]


def compound(outer, holes):
    """a filled shape with holes: outer counter-clockwise, holes clockwise (fills right under either fill rule)"""
    verts, codes = [], []
    for k, p in enumerate([ccw(outer)] + [ccw(h)[::-1] for h in holes]):
        verts += list(p) + [p[0]]
        codes += [Path.MOVETO] + [Path.LINETO] * (len(p) - 1) + [Path.CLOSEPOLY]
    return Path(np.array(verts), codes)


def to_world(p, centre, phi):
    c, s = math.cos(phi), math.sin(phi)
    return np.stack([c * p[:, 0] - s * p[:, 1] + centre[0], s * p[:, 0] + c * p[:, 1] + centre[1]], 1)


def disc_parts(centre, phi):
    """outline and holes (bore, output pin holes as printed, pull holes) in world coords"""
    outline = to_world(C.profile_xy(240), centre, phi)
    holes = [ring_xy(centre[0], centre[1], C.ECC_BORE / 2)]
    for a in C.out_pin_angles():
        hx, hy = to_world(np.array([C.polar(C.R_OUT, a)]), centre, phi)[0]
        holes.append(ring_xy(hx, hy, HOLE_D / 2, 120))
    for a in C.PULL_ANGLES:
        hx, hy = to_world(np.array([C.polar(C.PULL_R, a)]), centre, phi)[0]
        holes.append(ring_xy(hx, hy, C.PULL_HOLE / 2, 60))
    return outline, holes


def hole_centres(centre, phi):
    return [to_world(np.array([C.polar(C.R_OUT, a)]), centre, phi)[0] for a in C.out_pin_angles()]


def shaft_xy():
    """the motor shaft at input 0: D 5, flat 0.5 deep facing -x (cycloid.py's flat box); closing the outline draws
    the flat"""
    r = G.SHAFT_D / 2
    a = math.acos(-(r - G.SHAFT_FLAT_DEPTH) / r)          # the arc keeps x >= -(r - flat depth)
    t = np.linspace(-a, a, 240)
    return np.stack([r * np.cos(t), r * np.sin(t)], 1)


def view(fig, cx_mm, cy_mm, half_w, half_h, scale, centre=(0.0, 0.0)):
    """axes whose data mm print at exactly `scale` on the page, centred on paper point (cx_mm, cy_mm)"""
    w, h = 2 * half_w * scale, 2 * half_h * scale
    ax = fig.add_axes([(cx_mm - w / 2) / PAGE_W, (cy_mm - h / 2) / PAGE_H, w / PAGE_W, h / PAGE_H])
    ax.set_xlim(centre[0] - half_w, centre[0] + half_w)
    ax.set_ylim(centre[1] - half_h, centre[1] + half_h)
    ax.set_aspect("auto")
    ax.axis("off")
    return ax


def scale_bar(ax, x, y, scale, length=10.0):
    ax.plot([x, x + length], [y, y], color=INK, lw=1.2, solid_capstyle="butt")
    for xx in (x, x + length):
        ax.plot([xx, xx], [y - 0.06 * length, y + 0.06 * length], color=INK, lw=0.8)
    ax.text(x + length / 2, y + 0.12 * length, f"{length:g} mm at {scale:g}:1 = {length * scale:g} mm on paper",
            ha="center", va="bottom", fontsize=6.5)


def cross(ax, x, y, size, color, lw=0.7):
    ax.plot([x - size, x + size], [y, y], color=color, lw=lw)
    ax.plot([x, x], [y - size, y + size], color=color, lw=lw)


def draw_discs(ax, lw=0.7, show_orbits=True):
    o1, h1 = disc_parts(C1, P1)
    o2, h2 = disc_parts(C2, P2)
    for a in C.ring_pin_angles():                                        # ring pins (context)
        ax.add_patch(Circle(C.polar(R_RING, a), C.PIN_D / 2, fc=PIN_GREY, ec="#8a8a8a", lw=0.3))
    ax.add_patch(PathPatch(compound(o1, h1), fc=ORANGE_FILL, ec=ORANGE, lw=lw))           # disc 1, under
    ax.add_patch(PathPatch(compound(o2, h2), fc=(*matplotlib.colors.to_rgb(BLUE_FILL), 0.15), ec=BLUE, lw=lw))
    ax.add_patch(Circle((0, 0), C.R_OUT, fill=False, ec=INK, lw=0.5, ls=(0, (6, 2, 1, 2))))   # output pin circle
    ax.add_patch(Circle(tuple(C1), C.R_OUT, fill=False, ec=ORANGE, lw=0.5, ls=(0, (3, 2))))   # disc 1 hole circle
    ax.add_patch(Circle(tuple(C2), C.R_OUT, fill=False, ec=BLUE, lw=0.5, ls=(0, (3, 2))))     # disc 2 hole circle
    for a in C.out_pin_angles():
        px, py = C.polar(C.R_OUT, a)
        ax.add_patch(Circle((px, py), C.PIN_D / 2, fc=STEEL, ec=INK, lw=0.3))                # output pins
        if show_orbits:
            ax.add_patch(Circle((px, py), E, fill=False, ec="white", lw=0.5, ls=(0, (1, 1))))
    # the cams and the shaft
    ax.add_patch(Circle(tuple(C1), C.CAM_D / 2, fill=False, ec=ORANGE, lw=lw * 1.6))
    ax.add_patch(Circle(tuple(C2), C.CAM_D / 2, fill=False, ec=BLUE, lw=lw * 1.6))
    ax.add_patch(PathPatch(compound(shaft_xy(), []), fc=STEEL, ec=INK, lw=0.3))
    cross(ax, 0, 0, 1.2, INK, 0.5)


def label(ax, text, xy, xytext, color=INK, fs=7, ha="left"):
    ax.annotate(text, xy=xy, xytext=xytext, fontsize=fs, color=color, ha=ha, va="center",
                arrowprops=dict(arrowstyle="-", lw=0.5, color=color, shrinkA=0, shrinkB=0))


def page1(pdf):
    fig = plt.figure(figsize=(PAGE_W / 25.4, PAGE_H / 25.4))
    fig.text(0.5, 268 / PAGE_H, "Gearbox: output pin holes and the cam", ha="center", fontsize=14, weight="bold")
    fig.text(0.5, 261 / PAGE_H, "Seen from the lid side, motor input at 0 deg (cam 1 points right).  "
             "Disc 2 (upper) blue, over disc 1 (lower) orange.  Scale 3:1.", ha="center", fontsize=8)
    ax = view(fig, PAGE_W / 2, 157, 31.5, 31.5, 3.0)
    draw_discs(ax)
    # labels round the outside
    label(ax, "disc 2 (upper), centre on cam 2,\n0.368 left of the motor axis", (-24.5, 12.0), (-31.3, 26.0), BLUE)
    label(ax, "disc 1 (lower), centre on cam 1,\n0.368 right of the motor axis", (24.6, -12.0), (14.5, -29.5), ORANGE)
    label(ax, "output pin circle D41,\non the motor axis", C.polar(C.R_OUT, 135), (-31.3, 31.0))
    label(ax, "ring pins as your\nhousing prints them\n(52.90 inside)", C.polar(R_RING, 60), (16.5, 30.0), "#555555")
    hx, hy = C.polar(C.R_OUT, 45)
    label(ax, "disc 1 hole circle D41,\ncentred on cam 1", (C1[0] + hx, C1[1] + hy), (22.0, 23.5), ORANGE)
    hx, hy = C.polar(C.R_OUT, 200)
    label(ax, "disc 2 hole circle D41,\ncentred on cam 2", (C2[0] + hx, C2[1] + hy), (-31.3, -22.0), BLUE)
    scale_bar(ax, -31.0, -31.0, 3.0)

    # detail: the pin at 22.5 deg in its two holes, 10:1
    px, py = C.polar(C.R_OUT, C.out_pin_angles()[0])
    d = view(fig, 50, 36, 2.5, 2.5, 10.0, centre=(px, py))
    big = np.array([[px - 4, py - 4], [px + 4, py - 4], [px + 4, py + 4], [px - 4, py + 4]])
    (h1x, h1y), (h2x, h2y) = hole_centres(C1, P1)[0], [h for h in hole_centres(C2, P2) if math.hypot(h[0] - px, h[1] - py) < 1][0]
    d.add_patch(PathPatch(compound(big, [ring_xy(h1x, h1y, HOLE_D / 2)]), fc=ORANGE_FILL, ec="none"))
    d.add_patch(Circle((h1x, h1y), HOLE_D / 2, fill=False, ec=ORANGE, lw=1.0))
    d.add_patch(PathPatch(compound(big, [ring_xy(h2x, h2y, HOLE_D / 2)]), fc=(*matplotlib.colors.to_rgb(BLUE_FILL), 0.22), ec="none"))
    d.add_patch(Circle((h2x, h2y), HOLE_D / 2, fill=False, ec=BLUE, lw=1.0))
    d.add_patch(Circle((px, py), C.PIN_D / 2, fc=STEEL, ec=INK, lw=0.5))
    d.add_patch(Circle((px, py), E, fill=False, ec="white", lw=0.8, ls=(0, (2, 2))))
    cross(d, px, py, 0.25, "white", 0.6)
    cross(d, h1x, h1y, 0.18, ORANGE, 0.8)
    cross(d, h2x, h2y, 0.18, BLUE, 0.8)
    d.add_patch(plt.Rectangle((px - 2.5, py - 2.5), 5.0, 5.0, fill=False, ec=INK, lw=0.6))
    scale_bar(d, px + 1.2, py - 2.4, 10.0, 1.0)
    fig.text(85 / PAGE_W, 60 / PAGE_H, "Detail, 10:1: one output pin in its two holes", fontsize=9, weight="bold")
    notes = [
        ("Pin D3 (steel), on the output pin circle - it only turns with the output (1/37 speed).", STEEL),
        (f"Disc 1 hole D{HOLE_D:.2f} as printed (the model is D{HOLE_MODEL:.2f} for your printer);", ORANGE),
        ("its centre sits 0.368 RIGHT of the pin, the same offset as cam 1.", ORANGE),
        (f"Disc 2 hole D{HOLE_D:.2f}; its centre sits 0.368 LEFT of the pin, like cam 2.", BLUE),
        ("White dotted circle, radius 0.368: as the motor turns, each hole centre runs", INK),
        ("round its pin on this circle - the cam's throw, copied out to every hole.", INK),
        (f"Hole D{HOLE_D:.3f} = pin 3 + 2 x 0.368 throw + 2 x {C.HOLE_CLEAR:g} clearance.", INK),
        ("So the pin always rides 0.05 off the hole wall on the side away from the cam.", INK),
        ("If the cam throws more than 0.368 + 0.05, the pin is pinched in the hole;", INK),
        ("more than about 0.10, the lobes are pushed into the ring pins.", INK),
    ]
    for k, (t, col) in enumerate(notes):
        fig.text(85 / PAGE_W, (53 - 4.6 * k) / PAGE_H, t, fontsize=7.2, color=col)
    fig.text(0.5, 3.5 / PAGE_H, f"cycloid.py, 37:1, lobes drawn for pin radius {C.DISC_PIN_R:g}, ring pins as printed "
             f"(52.90), 2026-11-18.  Print at Actual size (not Fit to page); check the 10 mm bars.",
             ha="center", fontsize=6.5, color="#555555")
    pdf.savefig(fig)
    return fig


def page2(pdf):
    fig = plt.figure(figsize=(PAGE_W / 25.4, PAGE_H / 25.4))
    fig.text(0.5, 268 / PAGE_H, "Gearbox: the cams on the motor shaft", ha="center", fontsize=14, weight="bold")
    fig.text(0.5, 261 / PAGE_H, "Seen from the lid side, input at 0 deg.  Scale 10:1.  Cam 1 (under disc 1) orange, "
             "cam 2 (under disc 2) blue.", ha="center", fontsize=8)
    ax = view(fig, PAGE_W / 2, 192, 6.6, 6.0, 10.0)
    ax.add_patch(Circle((0, 0), C.COLLAR_D / 2, fill=False, ec="#777777", lw=0.8, ls=(0, (5, 3))))
    ax.add_patch(Circle(tuple(C1), C.CAM_D / 2, fc=(*matplotlib.colors.to_rgb(ORANGE_FILL), 0.6), ec=ORANGE, lw=1.6))
    ax.add_patch(Circle(tuple(C2), C.CAM_D / 2, fc=(*matplotlib.colors.to_rgb(BLUE_FILL), 0.25), ec=BLUE, lw=1.6))
    ax.add_patch(PathPatch(compound(shaft_xy(), []), fc="#b4b4b4", ec=INK, lw=0.6))
    cross(ax, 0, 0, 0.9, INK, 0.8)
    cross(ax, C1[0], C1[1], 0.45, ORANGE, 1.4)
    cross(ax, C2[0], C2[1], 0.45, BLUE, 1.4)
    yd = 3.3                                                           # the throw dimensions, above the shaft
    for x, col in ((0.0, INK), (C1[0], ORANGE), (C2[0], BLUE)):
        ax.plot([x, x], [0.5, yd + 0.25], color=col, lw=0.45)
    for (cx, col, ha) in ((C1[0], ORANGE, "left"), (C2[0], BLUE, "right")):
        ax.add_patch(FancyArrowPatch((0, yd), (cx, yd), arrowstyle="<|-|>", mutation_scale=5, lw=0.7, color=col))
        ax.text(cx + (0.12 if cx > 0 else -0.12), yd, "0.368", color=col, fontsize=7.5, ha=ha, va="center")
    ax.text(0, yd + 0.45, "0.736 between the cam centres", fontsize=7, ha="center", va="bottom")
    xf = -(G.SHAFT_D / 2 - G.SHAFT_FLAT_DEPTH)
    label(ax, "motor shaft D5, D flat 0.5 deep:\nthe flat faces along the cam line", (xf - 0.05, 1.2), (-6.5, 4.6))
    label(ax, "collar D11 (between the cams),\non the motor axis", (C.COLLAR_D / 2 * math.cos(2.3), C.COLLAR_D / 2 * math.sin(2.3)),
          (-6.5, 5.7), "#555555")
    label(ax, "cam 1 D10, centre 0.368 right", (C1[0] + 5 * math.cos(-0.6), 5 * math.sin(-0.6)), (2.6, -5.5), ORANGE)
    label(ax, "cam 2 D10, centre 0.368 left", (C2[0] + 5 * math.cos(-2.5), 5 * math.sin(-2.5)), (-6.5, -5.5), BLUE)
    ax.add_patch(FancyArrowPatch((-1.7, -3.1), (1.7, -3.1), arrowstyle="<->", mutation_scale=7, lw=0.9, color="#b00020"))
    ax.text(0, -3.35, "any play of the eccentric on the shaft\nmoves BOTH cams along this line",
            color="#b00020", fontsize=6.8, ha="center", va="top")
    scale_bar(ax, 3.4, 5.0, 10.0, 1.0)
    notes = [
        "The two cams are the same D10 circle, 0.368 each side of the motor axis, 0.736 apart, on one line.",
        "The D flat faces along that same line. If the eccentric can shift on the shaft by d along it, one",
        "disc throws 0.368 + d and the other 0.368 - d: both discs move the same way, so the net push walks",
        "the housing round once per motor turn. Your lobes have about 0.10 to the ring pins, so d must stay",
        "well under 0.10 - the bore has to grip the shaft with no rock.",
    ]
    for k, t in enumerate(notes):
        fig.text(18 / PAGE_W, (124 - 4.6 * k) / PAGE_H, t, fontsize=7.4)
    fig.text(0.5, 92 / PAGE_H, "The whole drive at 1:1 - lay a printed disc on it", ha="center", fontsize=9, weight="bold")
    full = view(fig, PAGE_W / 2, 52, 31.5, 31.5, 1.0)
    draw_discs(full, lw=0.4, show_orbits=False)
    scale_bar(full, 15.0, -31.0, 1.0)
    fig.text(0.5, 6 / PAGE_H, "Print at Actual size (not Fit to page); check the 10 mm bars.", ha="center",
             fontsize=6.5, color="#555555")
    pdf.savefig(fig)
    return fig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(HERE), "cycloid", "Output pin holes and cam.pdf"))
    ap.add_argument("--png", help="also write page PNGs into this folder (for checking)")
    a = ap.parse_args()
    with PdfPages(a.out) as pdf:
        figs = [page1(pdf), page2(pdf)]
    if a.png:
        for k, f in enumerate(figs, 1):
            f.savefig(os.path.join(a.png, f"pin_diagram_p{k}.png"), dpi=150)
    print("wrote", a.out)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
electronics_case.py - the Robo Chess electronics case, CONCEPT v0 (layout only: shell, screen face, fan, boards).

Mike 2026-11-06: "Let's make a case. I have one that we use for the drum machine, but we're going to have to make
substantial changes to it. We may have to just go ahead and slope the face of it all the way up and then have our board
sitting in it like they would be sitting in a PC on their edge. Then we can put a fan on the one end, like a maybe a 50
millimeter, so we get lots of flow ... plug that into one of the fan ports on the Octopus board ... make individual
adapter boards that screw down into the roof of our case. We're going to be turning this upside down and putting boards
into it, the screen into it, everything, and then put the bottom on it." / "We won't be lighting more than a few LEDs at
a time. We need little 5V." / "option to with 24V psu".

What it is (v0): the drum-amp jukebox console (Desktop/drum-amp-jukebox/cad: the Ender-3 V2 DWIN screen on a slope,
caliper-measured) with the slope run all the way up to the back, and the boards standing on edge front to back like PC
cards, each hanging from the slope on its own adapter (adapters come in v1). A 50 mm fan in the +x end wall blows along
them; slots in the -x end let the air out. The bottom cover goes on last. --psu adds a 24 V supply (Mean Well
LRS-150-24 size, 159 x 97 x 30) standing on edge behind the Octopus.

    python electronics_case.py [--psu] [--tilt 35]  -> robo_chess/electronics_case/ (shell + cover STEP, the boards
                                                       placed, for viewing) and the "case" Robot Studio's scene

Case frame: x across (fan end +x), y front (0) to back, z up from the floor (0). Boards are "cards": a plane at y, the
component side facing the front (-y), the long edge horizontal.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402

import studio  # noqa: E402

ROOT = os.path.dirname(HERE)
REF = os.path.join(ROOT, "ref models")
OUT = os.path.join(ROOT, "electronics_case")
V = cq.Vector

# ---------------------------------------------------------------- the screen (from drum_case.py, calipers)
# Ender-3 V2 DWIN board: board coordinates, front (LCD) toward you, knob on the right, origin bottom-left, z 0 = PCB top
BOARD_W, BOARD_H, BOARD_T = 141.0, 74.0, 1.7
_hx, _hy = (60.95, 57.46), (36.95, 33.10)
KNOB_X = 138.41 - 6.0
BOARD_HOLES = [(_hx[0] - _hx[1], _hy[0] - _hy[1]), (_hx[0] - _hx[1], _hy[0] + _hy[1]),
               (_hx[0] + _hx[1], _hy[0] - _hy[1]), (_hx[0] + _hx[1], _hy[0] + _hy[1]), (KNOB_X, 26.0)]
LCD_H = 5.2                     # LCD front above the PCB top
VA = (12.2, 109.9, 11.2, 67.7)  # visible area
ENC = (KNOB_X, 39.0)
ENC_SHAFT_D = 6.7
DISPLAY_BACK = 9.6 + 8.0        # deepest part behind the PCB (2x5 header) + the ribbon plug and its bend
SCREEN_MARGIN = 14.05           # slope face beyond the board, each side
LCD_GAP = 0.2
WINDOW_INSET = -0.4
KNOB_HOLE_D = ENC_SHAFT_D + 2.8
BOSS_D, BOARD_PILOT_D = 7.0, 2.1  # M2.5 self-tappers

# ---------------------------------------------------------------- the case
TILT = 35.0          # the slope, up from the desk, all the way to the back (--tilt)
WALL = 3.0
BOTTOM_T = 3.0
FRONT_H_MIN = 15.0   # front lip, raised if the screen's lowest part (its ribbon plug) needs it
FLOOR_CLEAR = 3.0    # screen parts above the bottom cover
WIRE_LOW = 17.0      # the cards' lower edges above whatever is under them: room for the plugs and wires
# 24 V supply (Mike: "at the very bottom", "on its side", "run the terminals to the back"): lying on the bottom
# cover, long side front to back, terminal end TERM_ROOM from the back wall. LRS-150-24 size (an LRS-100-24, 129 long,
# fits the same space). It screws to the bottom cover (the case is built upside down; the cover goes on last).
PSU_L, PSU_W, PSU_H = 159.0, 97.0, 30.0
TERM_ROOM = 22.0     # supply terminal end to the back wall: screwdriver + wire bends
PSU_GAP = 3.0        # supply top to the wire room under the cards
# Mains inlet in the back wall (Mike's IEC C14 + fuse + rocker, screw ears; listing photos 2026-11-07: flange 60,
# body 47.3 behind the panel, 26.8 deep; ref models/Fused switch AC 250V male socket AC 08: body 47.4 x 27, flange
# 58.3 x 49, ear holes 40 apart). Lying sideways, low, beside the supply's terminals, so its body fits under the cards.
INLET_CUT = (47.5 + 0.4, 27.0 + 0.4)   # hole: body + 0.2 a side
INLET_FLANGE = (60.0, 49.0)
INLET_SCREWS = 40.0                    # ear hole spacing (caliper check), M3 clearance 3.4
INLET_DEPTH = 26.8 + 12.0              # body + spade terminals and their wire bends
ADAPT = 12.0         # card top edge to the inside of the slope (over its component side): the adapter + its screws
CARD_GAP = 4.0       # between one card's back and the next card's components
BACK_ROOM = 6.0      # last card's back to the back wall
END_ROOM = 22.0      # card ends to the end walls: plugs at the short edges (more at the fan end: the fan's 10 mm)
FAN = 50.0           # fan size (square), thickness 10
FAN_T = 10.0

# ---------------------------------------------------------------- the cards (front to back)
# len along x, height, front depth (components + plugs + wire bend), back depth (pins); model file + placement
CARDS = [
    dict(name="Arduino Mega 2560", file="Arduino MEGA 2560 R3.STEP", L=101.6, h=53.34, f=30.0, b=5.0,
         # model: board in its XZ plane, x -53.34..0 (width), z -101.6..0 (length), components +y
         R=((0, 0, 1), (0, -1, 0), (1, 0, 0)), t0=(50.8, 0.0, 53.34)),
    dict(name="Raspberry Pi 4", file="Raspberry Pi 4 Model B.STEP", L=85.0, h=56.0, f=18.0, b=3.0,
         # model: board in its XZ plane, x -42.5..42.5, z -28..28, components +y
         R=((-1, 0, 0), (0, -1, 0), (0, 0, 1)), t0=(0.0, 0.0, 28.0)),
    dict(name="BTT Octopus", file="BIGTREETECH Octopus V1.0.stp", L=160.0, h=100.0, f=26.0, b=5.0,
         # model: board in its XY plane, x 0..160, y 20.04..120.04, top z 0, components +z; f includes the
         # stepper driver modules with heatsinks (not in the model)
         R=((1, 0, 0), (0, 0, -1), (0, 1, 0)), t0=(-80.0, 0.0, -20.04)),
]
PSU = dict(name="24 V supply (LRS-150-24 size)", file=None, L=159.0, h=97.0, f=0.0, b=30.0)


def front_h(tilt):
    """Front lip: the screen's lowest part (its back, the ribbon plug) must clear the bottom cover."""
    T = math.radians(tilt)
    s0 = SCREEN_MARGIN                                               # the screen board's lower edge along the slope
    d = WALL + LCD_GAP + LCD_H + BOARD_T + DISPLAY_BACK               # its deepest part below the surface
    return max(FRONT_H_MIN, BOTTOM_T + FLOOR_CLEAR - (s0 * math.sin(T) - d * math.cos(T)))


def layout(cards, tilt, z_low):
    """Each card's plane y (its PCB's component face), front to back: as far forward as the slope (over its front
    face, less ADAPT) and the card in front allow."""
    tanT = math.tan(math.radians(tilt))
    FRONT_H = front_h(tilt)
    z_in0 = FRONT_H - WALL / math.cos(math.radians(tilt))      # the inside of the slope at y 0
    y_prev_back = 0.0
    out = []
    T = math.radians(tilt)
    U, N = (math.cos(T), math.sin(T)), (-math.sin(T), math.cos(T))
    s0, s1 = SCREEN_MARGIN, SCREEN_MARGIN + BOARD_H                  # the screen board along the slope
    keep = WALL + LCD_GAP + LCD_H + BOARD_T + DISPLAY_BACK + 3.0      # its deepest part below the surface, + 3
    for c in cards:
        top = z_low + c["h"]
        y_front = max((top + ADAPT - z_in0) / tanT, y_prev_back + CARD_GAP)
        while True:                                                   # and clear of the screen's back
            py, pz = y_front, top - FRONT_H
            s, d = py * U[0] + pz * U[1], -(py * N[0] + pz * N[1])
            if not (s0 - 5 < s < s1 + 5) or d >= keep:
                break
            y_front += 0.5
        y = y_front + c["f"]
        out.append(dict(c, y=y, y_front=y_front, y_back=y + c["b"], z0=z_low, z1=top))
        y_prev_back = y + c["b"]
    D = y_prev_back + BACK_ROOM + WALL
    H = FRONT_H + D * tanT
    return out, D, H, FRONT_H


def trsf(R, t):
    tr = gp_Trsf()
    tr.SetValues(R[0][0], R[0][1], R[0][2], t[0], R[1][0], R[1][1], R[1][2], t[1], R[2][0], R[2][1], R[2][2], t[2])
    return cq.Location(tr)


def shell(W, D, H, tilt, FRONT_H):
    """Outer wedge: lip, one slope to the back edge, back wall; hollowed by WALL, open underneath."""
    tanT = math.tan(math.radians(tilt))
    prof = [(0, 0), (0, FRONT_H), (D, FRONT_H + D * tanT), (D, 0)]
    outer = cq.Workplane("YZ").polyline(prof).close().extrude(W / 2, both=True).val()
    zi = FRONT_H - WALL / math.cos(math.radians(tilt))
    inner_prof = [(WALL, -1), (WALL, zi + WALL * tanT), (D - WALL, zi + (D - WALL) * tanT), (D - WALL, -1)]
    inner = cq.Workplane("YZ").polyline(inner_prof).close().extrude(W / 2 - WALL, both=True).val()
    return outer, outer.cut(inner)


def screen_place(tilt, FRONT_H):
    T = math.radians(tilt)
    U, N = V(0, math.cos(T), math.sin(T)), V(0, -math.sin(T), math.cos(T))
    s_c = SCREEN_MARGIN + BOARD_H / 2
    pcb_depth = WALL + LCD_GAP + LCD_H
    org = V(0, 0, FRONT_H) + U * (s_c - BOARD_H / 2) - V(1, 0, 0) * (BOARD_W / 2) - N * pcb_depth
    plane = cq.Plane(origin=org.toTuple(), xDir=(1, 0, 0), normal=N.toTuple())
    face = cq.Plane(origin=(V(0, 0, FRONT_H) + U * s_c).toTuple(), xDir=(1, 0, 0), normal=N.toTuple())
    return cq.Location(plane), face, U, N, s_c


def screen_features(hood, face, tilt):
    """Window, knob hole and the 5 board posts (as drum_case.py)."""
    def fxy(bx, by):
        return bx - BOARD_W / 2, by - BOARD_H / 2
    wx, wy = fxy((VA[0] + VA[1]) / 2, (VA[2] + VA[3]) / 2)
    ww, wh = VA[1] - VA[0] - 2 * WINDOW_INSET, VA[3] - VA[2] - 2 * WINDOW_INSET
    hood = hood.cut(cq.Workplane(face).workplane(offset=1).center(wx, wy).rect(ww, wh).extrude(-(WALL + 2)).val())
    kx, ky = fxy(*ENC)
    hood = hood.cut(cq.Workplane(face).workplane(offset=1).center(kx, ky).circle(KNOB_HOLE_D / 2)
                    .extrude(-(WALL + 2)).val())
    boss = LCD_GAP + LCD_H
    for hx, hy in BOARD_HOLES:
        bx, by = fxy(hx, hy)
        hood = hood.fuse(cq.Workplane(face).workplane(offset=-WALL + 0.01).center(bx, by).circle(BOSS_D / 2)
                         .extrude(-boss - 0.01).val())
        hood = hood.cut(cq.Workplane(face).workplane(offset=-1.5).center(bx, by).circle(BOARD_PILOT_D / 2)
                        .extrude(-(boss + WALL)).val())
    return hood


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-psu", action="store_true", help="no 24 V supply inside (and no inlet)")
    ap.add_argument("--tilt", type=float, default=TILT)
    ap.add_argument("--no-boards", action="store_true", help="skip re-exporting the board meshes")
    ap.add_argument("--assembly", action="store_true", help="also write case_assembly_v1.step (the case, screen, fan, supply and inlet in place, one named body each, for Shapr3D) and boards/*_in_case.step (each board in place)")
    a = ap.parse_args()
    psu = not a.no_psu
    cards = CARDS
    Z_LOW = BOTTOM_T + (PSU_H + PSU_GAP if psu else 0.0) + WIRE_LOW
    placed, D, H, FRONT_H = layout(cards, a.tilt, Z_LOW)
    W = max(BOARD_W + 2 * SCREEN_MARGIN, max(c["L"] for c in cards) + 2 * END_ROOM + 2 * WALL)
    T = math.radians(a.tilt)
    print(f"case {W:.1f} wide x {D:.1f} deep x {H:.1f} tall (slope {a.tilt:g} deg){' with the 24 V supply' if psu else ''}; lip {FRONT_H:.1f}, cards from z {Z_LOW:.1f}")
    for c in placed:
        print(f"  {c['name']:32s} plane y {c['y']:6.1f} (front {c['y_front']:6.1f}, back {c['y_back']:6.1f}), z {c['z0']:.0f}..{c['z1']:.1f}")
    os.makedirs(os.path.join(OUT, "view"), exist_ok=True)
    outer, hood = shell(W, D, H, a.tilt, FRONT_H)
    loc, face, U, N, s_c = screen_place(a.tilt, FRONT_H)
    tanT = math.tan(T)
    keep = []                                  # keep-out boxes: (name, x0, x1, y0, y1, z0, z1)
    for c in placed:
        keep.append((c["name"], -c["L"] / 2, c["L"] / 2, c["y_front"], c["y_back"], c["z0"] - 6, c["z1"]))
    if psu:
        py1 = D - WALL - TERM_ROOM
        px0 = -W / 2 + WALL + 8                 # toward the inlet's end
        psu_box = ("24 V supply", px0, px0 + PSU_W, py1 - PSU_L, py1, BOTTOM_T, BOTTOM_T + PSU_H)
        keep.append(psu_box)
        keep.append(("supply terminals + wires", px0, px0 + PSU_W, py1, D - WALL, BOTTOM_T, BOTTOM_T + PSU_H))
        ix = px0 + PSU_W + 6 + INLET_FLANGE[0] / 2      # beside the supply, its flange on the back wall
        iz = INLET_FLANGE[1] / 2 + 1.0                  # flange just above the case's bottom edge
        keep.append(("mains inlet", ix - INLET_CUT[0] / 2, ix + INLET_CUT[0] / 2, D - WALL - INLET_DEPTH, D - WALL,
                     iz - INLET_CUT[1] / 2, iz + INLET_CUT[1] / 2))
        hood = hood.cut(cq.Workplane("XZ", origin=(0, D + 1, 0)).center(ix, iz).rect(*INLET_CUT).extrude(WALL + 2).val())
        for sx in (-1, 1):
            hood = hood.cut(cq.Workplane("XZ", origin=(0, D + 1, 0)).center(ix, iz + sx * INLET_SCREWS / 2).circle(1.7)
                            .extrude(WALL + 2).val())
        if ix + INLET_FLANGE[0] / 2 > W / 2 - WALL:
            print(f"  !! the inlet's flange runs past the side wall by {ix + INLET_FLANGE[0] / 2 - (W / 2 - WALL):.1f}")
    hood = screen_features(hood, face, a.tilt)
    # fan in the +x end wall, centred on the Octopus's component side, low (its drivers)
    octo = next(c for c in placed if c["name"] == "BTT Octopus")
    fy, fz = octo["y_front"] + FAN / 2 - 4, Z_LOW + FAN / 2 + 8
    keep.append(("fan", W / 2 - WALL - FAN_T, W / 2 - WALL, fy - FAN / 2, fy + FAN / 2, fz - FAN / 2, fz + FAN / 2))
    hood = hood.cut(cq.Workplane("YZ", origin=(W / 2 + 1, 0, 0)).center(fy, fz).circle(FAN / 2 - 2).extrude(-(WALL + 2)).val())
    for sx in (-1, 1):
        for sy in (-1, 1):
            hood = hood.cut(cq.Workplane("YZ", origin=(W / 2 + 1, 0, 0)).center(fy + sx * 20, fz + sy * 20)
                            .circle(1.7).extrude(-(WALL + 2)).val())
    # exhaust slots in the -x end wall, level with the cards
    for k in range(9):
        yk = placed[0]["y_front"] + 4 + k * (octo["y_back"] - placed[0]["y_front"] - 8) / 8
        z_top = min(Z_LOW + 70, FRONT_H - WALL / math.cos(T) + (yk - 2) * tanT - 8)
        if z_top - Z_LOW < 15:
            continue
        hood = hood.cut(cq.Workplane("YZ", origin=(-W / 2 - 1, 0, 0)).center(yk, (Z_LOW + z_top) / 2)
                        .rect(3.0, z_top - Z_LOW).extrude(WALL + 2).val())
    cover = cq.Workplane("XY").center(0, D / 2).rect(W - 2 * WALL - 0.6, D - 2 * WALL - 0.6).extrude(BOTTOM_T).val()
    # checks: keep-outs against each other and under the inside of the slope (their top front corner)
    z_in = lambda y: FRONT_H - WALL / math.cos(T) + y * tanT  # noqa: E731
    bad = []
    for i, A in enumerate(keep):
        if z_in(A[3]) < A[6] + 1.0:
            bad.append(f"{A[0]}: top {A[6]:.1f} vs slope {z_in(A[3]):.1f} at y {A[3]:.1f}")
        for B in keep[i + 1:]:
            ov = [min(A[k + 1], B[k + 1]) - max(A[k], B[k]) for k in (1, 3, 5)]
            if min(ov) > 0:
                bad.append(f"{A[0]} x {B[0]}: overlap {ov[0]:.1f} x {ov[1]:.1f} x {ov[2]:.1f}")
    print("  clearance: " + ("every part clear" if not bad else "; ".join(bad)))
    cq.exporters.export(hood, os.path.join(OUT, "case_hood_v1.step"))
    cq.exporters.export(cover, os.path.join(OUT, "case_bottom_v1.step"))
    fan = (cq.Workplane("YZ", origin=(W / 2 - WALL - FAN_T, 0, 0)).center(fy, fz).rect(FAN, FAN).extrude(FAN_T)
           .faces(">X").workplane().circle(FAN / 2 - 2.5).cutBlind(-FAN_T).val())
    cq.exporters.export(fan, os.path.join(OUT, "view", "fan_50mm.step"))
    col = lambda h: cq.Color(*(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)))  # noqa: E731
    asm = [("case hood", hood, col("#5b8def")), ("case bottom cover", cover, col("#3350a0")),
           ("fan 50 x 50 x 10 (stand-in)", fan, col("#2a2c30"))]
    models = [dict(path=os.path.join(OUT, "case_hood_v1.step"), name="case: hood (v1 concept)", color="#5b8def"),
              dict(path=os.path.join(OUT, "case_bottom_v1.step"), name="case: bottom cover", color="#3350a0"),
              dict(path=os.path.join(OUT, "view", "fan_50mm.step"), name="fan 50 x 50 x 10", color="#2a2c30")]
    # the screen
    disp = cq.importers.importStep(os.path.join(REF, "Ender3 V2 DisplayV2.step")).val().moved(loc)
    p = os.path.join(OUT, "view", "screen.step")
    cq.exporters.export(disp, p)
    models.append(dict(path=p, name="Ender-3 V2 screen", color="#1f6b3a"))
    asm.append(("Ender-3 V2 screen (DisplayV2)", disp, col("#1f6b3a")))
    for c in placed:
        p = os.path.join(OUT, "view", c["name"].split(" (")[0].replace(" ", "_") + ".stl")
        if not a.no_boards or not os.path.exists(p) or a.assembly:
            print(f"  placing {c['file']} ...", flush=True)
            m = cq.importers.importStep(os.path.join(REF, c["file"])).val()
            t = (c["t0"][0], c["y"] + c["t0"][1], Z_LOW + c["t0"][2])
            m = m.moved(trsf(c["R"], t))
            if not a.no_boards or not os.path.exists(p):
                cq.exporters.export(m, p, tolerance=0.15, angularTolerance=0.3)
            if a.assembly:   # each board its own STEP, in case coordinates: written as is, so the many repeated
                #              parts in these models stay shared (baked one by one they made a 727 MB file)
                os.makedirs(os.path.join(OUT, "boards"), exist_ok=True)
                bp = os.path.join(OUT, "boards", c["name"].replace(" ", "_") + "_in_case.step")
                cq.exporters.export(m, bp)
                print(f"  {os.path.basename(bp)}: {os.path.getsize(bp) / 1e6:.1f} MB", flush=True)
        models.append(dict(path=p, name=c["name"], color="#2f8f4e" if "Pi" in c["name"] else "#1d5fa8"))
    if psu:
        b = psu_box
        pb = cq.Workplane("XY").box(b[2] - b[1], b[4] - b[3], b[6] - b[5], centered=False).val().translate(V(b[1], b[3], b[5]))
        p = os.path.join(OUT, "view", "psu_24V.step")
        cq.exporters.export(pb, p)
        models.append(dict(path=p, name=f"24 V supply ({PSU_L:g} x {PSU_W:g} x {PSU_H:g})", color="#9a9da3"))
        asm.append((f"24 V supply (stand-in, {PSU_L:g} x {PSU_W:g} x {PSU_H:g})", pb, col("#9a9da3")))
        im = cq.importers.importStep(os.path.join(REF, "Fused switch AC 250V male socket AC 08 by prodes.STEP")).val()
        # model: long side x, ears at z +-20, flange facing +y with the body behind it (-y): out through the back wall
        im = im.translate(V(ix, D, iz))
        p = os.path.join(OUT, "view", "mains_inlet.step")
        cq.exporters.export(im, p)
        models.append(dict(path=p, name="mains inlet (C14 + fuse + rocker)", color="#1b1b1e"))
        asm.append(("mains inlet: IEC C14 + fuse + rocker (AC08 model)", im, col("#1b1b1e")))
    if a.assembly:
        from regroup_assembly import write_flat
        path = os.path.join(OUT, "case_assembly_v1.step")
        write_flat(asm, path)
        print(f"assembly: {path} ({len(asm)} bodies)", flush=True)
    # the "case" studio's scene
    if "case" not in studio.studios():
        studio.add("case", who="Robot arm control and kinematics", what="the electronics case (electronics_case.py)")
    scp = studio.scene_path("case")
    for m in models:
        m["part_name"] = m["name"]
    sc = {"models": models, "frame": None, "colors": [{"match": ".", "color": "#b9bcc2"}],
          "notes": "electronics_case.py v1 concept"}
    json.dump(sc, open(scp, "w", encoding="utf-8"), indent=1)
    print("scene:", scp)


if __name__ == "__main__":
    main()

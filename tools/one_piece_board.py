#!/usr/bin/env python3
"""
one_piece_board.py - Mike's one-piece board (2026-11-10) as two models for the Robot Studio scene.

His words: "Rather than have the board made out of individual pieces, with the side holding areas I'd like this to be
one board. I would like to have the same amount of width as there is on the side on the front ... enough room on the
back to support the robot's base." He chose (option "1"): a base sheet under everything - the squares-and-trays layer
on top of it, the robot and the dock screwed to the bare sheet - so every arm height stays as it is today.
Then: "Put it in our scene. And render it. And the grain should run towards the back."

    viewer_session/board_base_sheet.step   the sheet (from a 4 x 4 ft sheet): X -225..617, Y +/-400 (800 x 842),
                                           with a POCKET deep pocket for the robot's base and one for the dock
    viewer_session/board_top_layer.step    the top layer, CELL_H (24) thick - two 1/2" plies - U-shaped: full width
                                           from X 107 to the front edge (617), and the two tray arms back to X 47 beside the
                                           robot; its face flush with the light buckets' rims

Frame here: the robot's yaw axis at 0,0, X toward the player, Y to the player's right, Z up from the robot's standing
face (the pockets' floor = the table today). The squares and the tray spots don't move (their reach was checked).
The STEP files are in the arm's design frame, like every model in the scene.

2026-11-10, later: "the light buckets to be flush from the top of the board" - the cell is 24 from its underside to the
rim, the wood 2.0 under the rim (CHESSBOARD_BRIEF.md), so with the wood where the arm expects it (25 over the robot's
face) the cells' undersides were 3 over the sheet. Mike picked the Robo chessboard design session's option A: pocket
the sheet 3 deep where the robot (and the dock) stand; the cells sit on the sheet and the top layer is two 12 mm plies.
And the trays: 2 x 8 spots (board_pieces.py), so the tray arms start at X 47; 2026-11-12 they run out flush with the
main's front (X 617) with a 9th row of buckets: its inner one the 17th spot, its outer one out of reach (just the look).

Open with him and the Robo chessboard design session (the board's owner): the sheet's thickness (SHEET_T is a guess:
1/2" ply, which leaves 9 under the pockets) and how the layers join.

    python one_piece_board.py      writes both STEP files and puts them in scene.json in place of the old separate
                                   board ("board") and trays ("capture trays") - those are kept in scene.json
                                   "replaced_models", to put back by hand if wanted
"""
import json
import os
import sys

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_pieces as BP  # noqa: E402
import swivel_base as S  # noqa: E402

OUT = os.path.join(HERE, "viewer_session")
SCENE = os.path.join(OUT, "scene.json")
SHEET_T = 12.0                                   # a guess (1/2" ply) - his to settle
TOP_T = S.BOARD_T                                # 25: the wood squares' top (where the pieces stand; the arm's setting)
CELL_H, CELL_WOOD = 24.0, 22.0                   # the cell: underside to rim, underside to its wood's top (the brief)
# Mike 2026-11-10: "the light buckets to be flush from the top of the board". The cell's lit rim stands 2.0 over its wood
# square (CHESSBOARD_BRIEF.md), so the board's top face is 2.0 over the squares, with an opening the bucket's size
# (59.8) at every square and tray spot - the rims flush with it. The squares stay where the arm expects them.
RIM_UP, BUCKET = 2.0, 59.8
POCKET = TOP_T - CELL_WOOD                       # 3: the sheet's top over the robot's face, so the cells sit on it
# what stands in the pockets (their footprints, measured in Robot Studio from swivel_fixed.step and
# dock_station_v2_scene.step), each pocket 0.5 bigger all round
POCKETS = {"robot base": (-126.0, 65.0, -65.0, 65.0), "dock": (-210.0, -69.0, 50.0, 184.0)}
# the edge round the outside buckets: the trays' outer column sits 15 inside the side edges (HALF_W 400); Mike
# 2026-11-10: "Cut off the front of the board so it has the same margin as the trays on the left and right have to the"
# [edge] - so the front edge is 15 past rank 1's buckets too (was a 160 front strip, his earlier ask)
EDGE = 15.0
BACK, HALF_W = -225.0, 400.0                     # the sheet's back (the dock) and its sides
FRONT = S.D_EDGE + 8 * S.SQ + EDGE               # 617: rank 1's front edge (602) + EDGE
TOP_FROM = S.D_EDGE - S.BORDER                   # 107: the playing area's 15 border starts here
ARM_BACK = BP.TRAY_X0 - BP.TRAY_PITCH / 2 - 15   # 47: 15 behind the last tray bucket (X 62)
# Mike 2026-11-10: "I can't machine it. Gonna have to come up with something that'll fit in a 24-inch window. May have to
# make this out of separate pieces. Make the part trays separate pieces from the main. Same thickness." - the top layer
# is three pieces now: the main board (the squares and their 15 edge) and a tray piece each side, butting its side edge
# (the 10 between the squares' border and the trays' buckets is the tray piece's own inner edge).
WINDOW = 24 * 25.4                               # 609.6: his machine's window - every top piece fits in it
TRAY_IN = 4 * S.SQ + S.BORDER                    # 255: the main's side edge = the tray pieces' inner edge
# Mike 2026-11-12: "Can we extend the tray to be flush with the front of the board? ... just to keep it symmetric" -
# the tray pieces to the main's front (was 557, 15 past the 8th row): the 9th row's buckets end 15 inside it too
TRAY_FRONT = FRONT
assert abs(BP.FRONT_X + BP.TRAY_PITCH / 2 + EDGE - TRAY_FRONT) < 0.5
NAMES = {"sheet": "board base sheet", "top": "board top layer", "tray_w": "board tray (white)", "tray_b": "board tray (black)"}
FILES = {"sheet": "board_base_sheet.step", "top": "board_top_layer.step", "tray_w": "board_tray_white.step",
         "tray_b": "board_tray_black.step"}


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def spots():
    """every place a piece stands: the 64 squares' and the tray spots' centres (X, Y)"""
    out = [BP.square(f, r) for f in range(8) for r in range(1, 9)]
    for c in ("white", "black"):                      # + the out-of-reach front buckets: a bucket all the same
        out += BP.tray_xy(c) + BP.tray_unused_xy(c)
    return out


def solids():
    sheet = box(BACK, FRONT, -HALF_W, HALF_W, POCKET - SHEET_T, POCKET)
    pk = [box(x0 - 0.5, x1 + 0.5, y0 - 0.5, y1 + 0.5, 0, POCKET + 1) for x0, x1, y0, y1 in POCKETS.values()]
    hole = pk[0]
    for b in pk[1:]:                                   # the dock's pocket runs into the robot's: one pocket
        hole = hole.fuse(b)
    sheet = sheet.cut(hole.clean()).clean()
    assert len(sheet.Solids()) == 1 and sheet.isValid(), "sheet: not one valid solid"
    sheet = sheet.Solids()[0]
    face = TOP_T + RIM_UP                                                 # flush with the rims: POCKET + CELL_H
    assert abs(face - (POCKET + CELL_H)) < 1e-6
    pieces = {"top": box(TOP_FROM, FRONT, -TRAY_IN, TRAY_IN, POCKET, face)}   # the main board
    for key, s in (("tray_w", -1), ("tray_b", 1)):                        # white's tray on the a-file side (-Y)
        y0, y1 = sorted((s * TRAY_IN, s * HALF_W))
        pieces[key] = box(ARM_BACK, TRAY_FRONT, y0, y1, POCKET, face)
    h = BUCKET / 2                                                        # a bucket's opening at every spot, down to the
    holes = cq.Compound.makeCompound([box(x - h, x + h, y - h, y + h, TOP_T, TOP_T + RIM_UP + 1) for x, y in spots()])
    for k in pieces:                                                      # squares' top
        p = pieces[k].cut(holes).clean()
        assert len(p.Solids()) == 1 and p.isValid(), f"{NAMES[k]}: not one valid solid"
        pieces[k] = p.Solids()[0]
    return sheet, pieces


def checks(pieces, sheet):
    """every top piece inside his machine's window and clear of the others; the squares on the main, each tray spot on
    its tray piece; the openings; the robot's footprint; the pockets in the sheet"""
    for k, p in pieces.items():
        bb = p.BoundingBox()
        assert bb.xlen <= WINDOW and bb.ylen <= WINDOW, f"{NAMES[k]}: {bb.xlen:.0f} x {bb.ylen:.0f} - over the window"
        print(f"  {NAMES[k]}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm ({bb.xlen / 25.4:.2f} x {bb.ylen / 25.4:.2f} in)"
              f" - fits the {WINDOW / 25.4:g} in window")
    ks = list(pieces)
    for i in range(len(ks)):
        for j in range(i + 1, len(ks)):
            v = pieces[ks[i]].intersect(pieces[ks[j]]).Volume()
            assert v < 1e-6, f"{NAMES[ks[i]]} and {NAMES[ks[j]]} overlap by {v:.1f} mm3"
    for what, (x0, x1, y0, y1) in POCKETS.items():                        # a pocket under each, and sheet beside it
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        assert not sheet.isInside(cq.Vector(cx, cy, POCKET / 2)), f"{what}: no pocket"
        assert sheet.isInside(cq.Vector(cx, cy, -1)), f"{what}: pocket through the sheet"
    assert sheet.isInside(cq.Vector(*BP.square(4, 4), POCKET - 0.5)), "no sheet under the cells"
    sb = sheet.BoundingBox()
    print(f"checks: the sheet's top {POCKET:g} over the robot's face (the cells' undersides sit on it); pockets "
          f"{POCKET:g} deep for the {' and the '.join(POCKETS)}, {SHEET_T - POCKET:g} of sheet left under them; the sheet "
          f"is {sb.xlen:.0f} x {sb.ylen:.0f} - {'OVER' if max(sb.xlen, sb.ylen) > WINDOW else 'inside'} the window")
    top = pieces["top"]
    for f in range(8):                                                    # every square on the main
        for r in range(1, 9):
            x, y = BP.square(f, r)
            assert top.isInside(cq.Vector(x, y, TOP_T - 1)), f"{'abcdefgh'[f]}{r}: not on the main board"
    n = 0
    for key, colour in (("tray_w", "white"), ("tray_b", "black")):        # every tray bucket on its own piece
        for x, y in BP.tray_xy(colour) + BP.tray_unused_xy(colour):
            for dx in (-29.9, 29.9):
                for dy in (-29.9, 29.9):
                    assert pieces[key].isInside(cq.Vector(x + dx, y + dy, TOP_T - 1)), f"tray spot {x},{y}: off {NAMES[key]}"
            n += 1
    for k in ("tray_w", "tray_b"):                                        # flush with the main's front
        assert abs(pieces[k].BoundingBox().xmax - pieces["top"].BoundingBox().xmax) < 1e-6, f"{NAMES[k]}: not flush" 
    for x, y in spots():                                                  # each opening open over its square
        assert not any(p.isInside(cq.Vector(x, y, TOP_T + RIM_UP / 2)) for p in pieces.values()), f"{x},{y}: no opening"
    robot = box(-126, 65, -65, 65, 0, TOP_T + RIM_UP)                      # the swivel base's footprint
    turn = cq.Solid.makeCylinder(77, TOP_T + RIM_UP, cq.Vector(0, 0, 0))   # the turntable
    for k, p in pieces.items():
        for what, s in (("swivel base", robot), ("turntable", turn)):
            v = p.intersect(s).Volume()
            assert v < 1e-6, f"{what} overlaps {NAMES[k]} by {v:.1f} mm3"
    print(f"checks: 64 squares on the main board, {n} tray buckets (17 used a side + 1 out of reach) on the tray "
          f"pieces, flush with the main's front, each in a {BUCKET} opening "
          f"{RIM_UP} deep; the pieces don't overlap; the robot's base and turntable clear of them")


def main():
    sheet, pieces = solids()
    checks(pieces, sheet)
    y_wood = S.AB.Y_CUT + S.AB.FOOT_T + S.Z_PLATE0 + S.PLATE_T            # as board_pieces.to_design()
    files = {}
    for key, shape in [("sheet", sheet)] + list(pieces.items()):
        d = S.to_design(shape, y_wood)
        path = os.path.join(OUT, FILES[key])
        cq.exporters.export(cq.Workplane().add(d), path)
        files[key] = path
        bb = d.BoundingBox()
        print(f"{NAMES[key]} -> {path}  (design y {bb.ymin:.1f}..{bb.ymax:.1f})")
    print(f"squares' top at design y {y_wood - TOP_T:.2f} (the arm's board_y is "
          f"{json.load(open(SCENE, encoding='utf-8')).get('ik', {}).get('board_y')}); the board's top face, flush with "
          f"the rims, at {y_wood - TOP_T - RIM_UP:.2f}")

    sc = json.load(open(SCENE, encoding="utf-8"))
    old = [m for m in sc["models"] if isinstance(m, dict) and m.get("name") in ("board", "capture trays")]
    keep = [m for m in sc["models"] if not (isinstance(m, dict) and m.get("name") in ("board", "capture trays", *NAMES.values()))]
    at = next((i for i, m in enumerate(keep) if isinstance(m, dict) and m.get("name") == "board light squares"), 0)
    colour = {"sheet": "#d8c8a8"}
    new = [{"path": files[k], "name": NAMES[k], "part_name": NAMES[k], "color": colour.get(k, "#c99a5b"), "pick": False,
            "by": "one_piece_board.py"} for k in files]
    sc["models"] = keep[:at] + new + keep[at:]
    if old:
        sc["replaced_models"] = {"why": "one_piece_board.py 2026-11-10: Mike's one-piece board replaced the separate "
                                        "board and trays; put these back in models to see them again", "models": old}
    tmp = SCENE + ".tmp"                         # whole or not at all: the running studio reads it every second
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(sc, f, indent=1)
    os.replace(tmp, SCENE)
    print(f"scene.json: {', '.join(NAMES[k] for k in files)} in; {', '.join(m['name'] for m in old) or 'nothing'} out "
          f"(kept in replaced_models)")


if __name__ == "__main__":
    main()

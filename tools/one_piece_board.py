#!/usr/bin/env python3
"""
one_piece_board.py - Mike's one-piece board (2026-11-10) as two models for the Robot Studio scene.

His words: "Rather than have the board made out of individual pieces, with the side holding areas I'd like this to be
one board. I would like to have the same amount of width as there is on the side on the front ... enough room on the
back to support the robot's base." He chose (option "1"): a base sheet under everything - the squares-and-trays layer
on top of it, the robot and the dock screwed to the bare sheet - so every arm height stays as it is today.
Then: "Put it in our scene. And render it. And the grain should run towards the back."

    viewer_session/board_base_sheet.step   the sheet (from a 4 x 4 ft sheet): X -225..617, Y +/-400 (800 x 842), flat:
                                           the robot's base and the dock stand on its top; their screws' pilots and
                                           the arm's cable hole are drilled by hand (drill_template.py)
    viewer_session/board_top_layer.step    the top layer, CELL_H (24) thick - two 1/2" plies - U-shaped: full width
                                           from X 107 to the front edge (617), and the two tray arms back to X 47 beside the
                                           robot; its face flush with the light buckets' rims

Frame here: the robot's yaw axis at 0,0, X toward the player, Y to the player's right, Z up from the robot's standing
face (= the sheet's top since 2026-11-24). The squares and the tray spots don't move in X, Y (their reach was checked).
The STEP files are in the arm's design frame, like every model in the scene.

2026-11-10, later: "the light buckets to be flush from the top of the board" - the cell is 24 from its underside to the
rim, the wood 2.0 under the rim (CHESSBOARD_BRIEF.md), so with the wood where the arm expects it (25 over the robot's
face) the cells' undersides were 3 over the sheet. Mike picked the Robo chessboard design session's option A: pocket
the sheet 3 deep where the robot (and the dock) stand; the cells sit on the sheet and the top layer is two 12 mm plies.
And the trays: 2 x 8 spots (board_pieces.py), so the tray arms start at X 47; 2026-11-12 they run out flush with the
main's front (X 617) with a 9th row of buckets: its inner one the 17th spot, its outer one out of reach (just the look).

2026-11-24 Mike (relayed by Robot arm control and kinematics): "Whoever's in charge of the chessboard, have them fix the
chessboard to match the new base and the new position of the parking stand." (Robo chessboard design session): the
pockets are no longer hand-typed boxes - each is traced from the scene's own model of what stands in it (the chain
swivel's fixed part, the dock station v2) where it meets the sheet, FIT bigger all round; a sliver of wood thinner than
2 x CLOSE between the two is cut away with them; the arm's cables run on from the swivel's own slot under its plate to
the sheet's back edge in a slot of their own; #8 pilots under the dock's four screw holes. Run it again whenever the
base or the dock moves.

2026-11-24, later - NO POCKETS. Mike: "If we don't put the pockets on the board, the recesses for the parking stand,
will it still work? I can't get this thing onto the machine to cut those pockets." and, to Robot arm control and
kinematics' plan, "Yeah, so let's do that." That session set swivel_base.BOARD_T 25 -> 22, so the sheet's top is the
robot's face (SHEET_TOP 0): the base and the dock stand on it, the cells sit on it, the whole board is 3 lower in the
design frame (the squares' top at design y 235). The sheet is flat; what's left of the pockets is drilled by hand:
a 1/8" pilot under each of the base's and the dock's #8 screws, and one 1/2" hole down through the sheet under the
swivel's wire slot for the arm's cables. drill_template.py prints them 1:1 to drill through.

Open with him and the Robo chessboard design session (the board's owner): the sheet's thickness (SHEET_T is a guess:
1/2" ply) and how the layers join.

    python one_piece_board.py      writes both STEP files and puts them in scene.json in place of the old separate
                                   board ("board") and trays ("capture trays") - those are kept in scene.json
                                   "replaced_models", to put back by hand if wanted
"""
import json
import os
import sys

import math

import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_CompCurve
from OCP.GCPnts import GCPnts_QuasiUniformAbscissa
from OCP.gp import gp_Trsf

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_pieces as BP  # noqa: E402
import swivel_base as S  # noqa: E402
from view_step import read_step  # noqa: E402

OUT = os.path.join(HERE, "viewer_session")
SCENE = os.path.join(OUT, "scene.json")
SHEET_T = 12.0                                   # a guess (1/2" ply) - his to settle
TOP_T = S.BOARD_T                                # 25: the wood squares' top (where the pieces stand; the arm's setting)
CELL_H, CELL_WOOD = 24.0, 22.0                   # the cell: underside to rim, underside to its wood's top (the brief)
# Mike 2026-11-10: "the light buckets to be flush from the top of the board". The cell's lit rim stands 2.0 over its wood
# square (CHESSBOARD_BRIEF.md), so the board's top face is 2.0 over the squares, with an opening the bucket's size
# (59.8) at every square and tray spot - the rims flush with it. The squares stay where the arm expects them.
RIM_UP, BUCKET = 2.0, 59.8
SHEET_TOP = TOP_T - CELL_WOOD                    # the sheet's top over the robot's face, so the cells sit on it: 0 since
assert abs(SHEET_TOP) < 1e-9, "the base and the dock stand on the sheet (no pockets): BOARD_T must be CELL_WOOD"   # (11-24)
# what stands on the sheet: the scene's models of those names (2026-11-24: the chain swivel and the moved dock); each
# one's outline where it meets the sheet (just over its own bottom), filled - the swivel's cover sits on its plate with
# the plate's outline, so it covers the plate's own cable slot and spindle bore; the plate gives the screw lugs. They
# are what drill_template.py draws to place the parts by.
STANDS = {"robot base": ("swivel chain (fixed)", "swivel chain cover"), "dock": ("dock station v2",)}
PILOT_D, PILOT_DEPTH = 3.2, 7.0                  # #8 wood screws into ply: 1/8" pilots, 7 deep, drilled by hand
# the arm's cables: down the spindle and out the back under the swivel's plate in a 12 wide slot (swivel_chain.py "the
# wires: out of the spindle, under the plate to the back", Y -6..6), then down through the sheet: a 1/2" hole, through,
# under the slot halfway between the spindle's bore and the plate's back edge (checked open above it)
CABLE_HOLE = (-70.0, 0.0, 12.7)                  # X, Y, D
SCREW_HOLE = (4.0, 5.5)                          # what counts as a screw hole through a foot: its diameter (the dock's 4.5)
TURNING = ("swivel chain turntable", "swivel chain yoke")   # what turns with J0 about the yaw axis
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


def y_wood():
    return S.AB.Y_CUT + S.AB.FOOT_T + S.Z_PLATE0 + S.PLATE_T          # the robot's face, design y (as main())


def from_design(shape, yw):
    """the arm's design frame -> this one: the inverse of swivel_base.to_design (X = -x, Y = Z_MID - z, Z = y_wood - y)"""
    t = gp_Trsf()
    t.SetValues(-1, 0, 0, 0,
                0, 0, -1, S.AB.Z_MID,
                0, -1, 0, yw)
    return cq.Shape.cast(shape.wrapped.Moved(cq.Location(t).wrapped))


def scene_model(name):
    """the scene's model of that name, in this frame (one compound)"""
    sc = json.load(open(SCENE, encoding="utf-8"))
    m = next((m for m in sc["models"] if isinstance(m, dict) and m.get("name") == name), None)
    assert m, f"scene.json has no model '{name}'"
    assert not m.get("offset") and not m.get("rotate"), f"'{name}' is moved in scene.json - not handled here"
    return cq.Compound.makeCompound([from_design(p[1], y_wood()) for p in read_step(m["path"])])


def plane(z):
    return cq.Face.makePlane(4000, 4000, cq.Vector(0, 0, z), cq.Vector(0, 0, 1))


def polyline(w, step=0.5):
    """a closed wire as straight runs every `step` mm along it, at Z 0 (offsetting OCC's B-splines - the dock's foot -
    can fail; a polygon offsets cleanly)"""
    c = BRepAdaptor_CompCurve(w.wrapped)
    d = GCPnts_QuasiUniformAbscissa(c, max(16, math.ceil(w.Length() / step)))
    pts = [c.Value(d.Parameter(i)) for i in range(1, d.NbPoints() + 1)]
    pts = [cq.Vector(p.X(), p.Y(), 0) for p in pts]
    if (pts[0] - pts[-1]).Length < 1e-6:
        pts = pts[:-1]
    return cq.Wire.makePolygon(pts, close=True)


def tops(solid, h):
    """the outer boundaries of a slab's top faces (at Z h), as polygons at Z 0"""
    return [polyline(f.outerWire()) for f in solid.Faces() if f.normalAt().z > 0.99 and f.Center().z > h - 1e-6]


def outlines(foot):
    """where a footprint meets the sheet, as closed (X, Y) point lists every 0.5 mm (drill_template.py draws these)"""
    return [[(v.X, v.Y) for v in w.Vertices()] for w in tops(foot["body"], 1.0)]


def standing(m):
    """a model split into what stands on the robot's face and its screws into the wood (solids that reach under the face):
    (compound, [(X, Y, depth into the wood)])"""
    up = [s for s in m.Solids() if s.BoundingBox().zmin > -0.05]
    down = [s.BoundingBox() for s in m.Solids() if s.BoundingBox().zmin <= -0.05]
    into = {}                                          # (a screw can be more than one solid: one place each)
    for b in down:
        k = (round((b.xmin + b.xmax) / 2, 1), round((b.ymin + b.ymax) / 2, 1))
        into[k] = max(into.get(k, 0.0), -b.zmin)
    return cq.Compound.makeCompound(up), [(x, y, d) for (x, y), d in into.items()]


def footprint(names):
    """what of the scene's models stands on the robot's face: {shape, body (its plan outline where it meets the sheet,
    filled, as a slab Z 0..1), screws (X Y of its screws into the wood, or of the holes through its foot)} - each
    model's section just over its own bottom, each section's outer boundary"""
    h = 1.0
    models, screws = [], []
    for n in names:
        m, into = standing(scene_model(n))
        for x, y, d in into:
            assert d <= PILOT_DEPTH, f"'{n}': a screw {d:.1f} into the wood at ({x:.1f}, {y:.1f}), the pilots are {PILOT_DEPTH}"
            screws.append((x, y))
        models.append(m)
    body = None
    for n, m in zip(names, models):
        z0 = m.BoundingBox().zmin
        assert z0 < 10, f"'{n}' starts {z0:.1f} over the robot's face - nowhere near the sheet"
        for z in (z0 + 0.3,):
            for f in m.intersect(plane(z)).Faces():
                s = cq.Solid.extrudeLinear(cq.Face.makeFromWires(f.outerWire().translate(cq.Vector(0, 0, -z))),
                                           cq.Vector(0, 0, h))
                body = s if body is None else body.fuse(s)
                if z < 1:
                    for w in f.innerWires():
                        b = w.BoundingBox()
                        if SCREW_HOLE[0] < b.xlen < SCREW_HOLE[1] and abs(b.xlen - b.ylen) < 0.1:
                            screws.append(((b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2))
    assert body is not None, f"{names}: nothing where the sheet is"
    one_each = []                                      # a modelled screw and its hole through the foot: one pilot
    for p in screws:
        if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 1.0 for q in one_each):
            one_each.append(p)
    return {"shape": cq.Compound.makeCompound(models), "body": body.clean(), "screws": one_each}


def drilled(feet):
    """what's drilled into the sheet by hand: a pilot under every screw, the cable hole through"""
    x, y, d = CABLE_HOLE
    cut = cq.Solid.makeCylinder(d / 2, SHEET_T + 2, cq.Vector(x, y, SHEET_TOP - SHEET_T - 1))
    for f in feet.values():
        for px, py in f["screws"]:
            cut = cut.fuse(cq.Solid.makeCylinder(PILOT_D / 2, PILOT_DEPTH + 0.1,
                                                 cq.Vector(px, py, SHEET_TOP - PILOT_DEPTH)))
    return cut.clean()


def spots():
    """every place a piece stands: the 64 squares' and the tray spots' centres (X, Y)"""
    out = [BP.square(f, r) for f in range(8) for r in range(1, 9)]
    for c in ("white", "black"):                      # + the out-of-reach front buckets: a bucket all the same
        out += BP.tray_xy(c) + BP.tray_unused_xy(c)
    return out


def solids(feet):
    sheet = box(BACK, FRONT, -HALF_W, HALF_W, SHEET_TOP - SHEET_T, SHEET_TOP)
    sheet = sheet.cut(drilled(feet)).clean()
    assert len(sheet.Solids()) == 1 and sheet.isValid(), "sheet: not one valid solid"
    sheet = sheet.Solids()[0]
    face = TOP_T + RIM_UP                                                 # flush with the rims: SHEET_TOP + CELL_H
    assert abs(face - (SHEET_TOP + CELL_H)) < 1e-6
    pieces = {"top": box(TOP_FROM, FRONT, -TRAY_IN, TRAY_IN, SHEET_TOP, face)}   # the main board
    for key, s in (("tray_w", -1), ("tray_b", 1)):                        # white's tray on the a-file side (-Y)
        y0, y1 = sorted((s * TRAY_IN, s * HALF_W))
        pieces[key] = box(ARM_BACK, TRAY_FRONT, y0, y1, SHEET_TOP, face)
    h = BUCKET / 2                                                        # a bucket's opening at every spot, down to the
    holes = cq.Compound.makeCompound([box(x - h, x + h, y - h, y + h, TOP_T, TOP_T + RIM_UP + 1) for x, y in spots()])
    for k in pieces:                                                      # squares' top
        p = pieces[k].cut(holes).clean()
        assert len(p.Solids()) == 1 and p.isValid(), f"{NAMES[k]}: not one valid solid"
        pieces[k] = p.Solids()[0]
    return sheet, pieces


def checks(pieces, sheet, feet):
    """every top piece inside his machine's window and clear of the others; the squares on the main, each tray spot on
    its tray piece; the openings; the robot's base and the dock standing on the sheet, clear of every piece; what turns
    with J0 clear of the top layer all the way round; the pilots and the cable hole"""
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
    board = dict(pieces, sheet=sheet)
    for what, f in feet.items():                                           # each standing on the sheet, nothing in it
        b = f["shape"].BoundingBox()
        assert abs(b.zmin - SHEET_TOP) < 0.05, f"{what}: stands at Z {b.zmin:.2f}, not on the sheet's top"
        for k, p in board.items():
            v = p.intersect(f["shape"]).Volume()
            assert v < 1e-3, f"{what} overlaps {NAMES.get(k, k)} by {v:.1f} mm3"
        for x, y in f["screws"]:                                           # a pilot under each screw, blind
            assert not sheet.isInside(cq.Vector(x, y, SHEET_TOP - PILOT_DEPTH + 0.5)), f"{what}: no pilot at {x:.1f},{y:.1f}"
            assert sheet.isInside(cq.Vector(x, y, SHEET_TOP - SHEET_T + 0.5)), f"{what}: pilot through at {x:.1f},{y:.1f}"
        print(f"checks: {what} ({', '.join(STANDS[what])}) on the sheet, X {b.xmin:.1f}..{b.xmax:.1f} Y {b.ymin:.1f}.."
              f"{b.ymax:.1f}; {f['shape'].distance(pieces['top']):.1f} from the main board, "
              f"{min(f['shape'].distance(pieces[k]) for k in ('tray_w', 'tray_b')):.1f} from a tray piece; "
              f"{len(f['screws'])} screws, a {PILOT_D} x {PILOT_DEPTH:g} pilot under each")
    x, y, d = CABLE_HOLE                                                   # the cable hole: through, under the swivel's
    base = feet["robot base"]                                              # open slot, hidden under its plate
    assert not sheet.isInside(cq.Vector(x, y, SHEET_TOP - SHEET_T / 2)), "no cable hole"
    assert base["body"].isInside(cq.Vector(x, y, 0.5)), "the cable hole isn't under the swivel"
    slot = cq.Solid.makeCylinder(5.9, 4.0, cq.Vector(x, y, SHEET_TOP))
    assert base["shape"].intersect(slot).Volume() < 1e-3, "the swivel's plate is over the cable hole, not its slot"
    sc = json.load(open(SCENE, encoding="utf-8"))
    names = [m["name"] for m in sc["models"] if isinstance(m, dict) and str(m.get("name", "")).startswith("swivel chain")]
    face = SHEET_TOP + CELL_H
    for n in names:                                                        # the rest of the base, where it stands
        c = standing(scene_model(n))[0]                                    # (its screws bite the wood: that's their job)
        for k, p in board.items():
            v = p.intersect(c).Volume()
            assert v < 1e-3, f"{n} overlaps {NAMES.get(k, k)} by {v:.1f} mm3"
        if n in TURNING:                                                   # and what turns: all the way round
            low = c.intersect(box(-1000, 1000, -1000, 1000, -50, face + 1))
            r = max((math.hypot(q.x, q.y) for e in low.Edges() for q in e.positions([i / 16 for i in range(17)])),
                    default=0.0)
            ring = cq.Solid.makeCylinder(r + 1, face + 1 - SHEET_TOP, cq.Vector(0, 0, SHEET_TOP))
            for k, p in pieces.items():
                assert p.intersect(ring).Volume() < 1e-6, f"{n} (r {r:.0f} under the board's face) sweeps {NAMES[k]}"
    print(f"checks: {', '.join(names)} clear of every board piece; {' and '.join(TURNING)} clear of the top layer all "
          f"the way round J0")
    assert sheet.isInside(cq.Vector(*BP.square(4, 4), SHEET_TOP - 0.5)), "no sheet under the cells"
    sb = sheet.BoundingBox()
    print(f"checks: the sheet's top is the robot's face (the base, the dock and the cells' undersides stand on it), "
          f"flat - no pockets; the cable hole D{d} through at ({x:g}, {y:g}), under the swivel's open slot; the sheet "
          f"is {sb.xlen:.0f} x {sb.ylen:.0f} - {'OVER' if max(sb.xlen, sb.ylen) > WINDOW else 'inside'} the window "
          f"(hand-drilled: drill_template.py)")
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
    print(f"checks: 64 squares on the main board, {n} tray buckets (17 used a side + 1 out of reach) on the tray "
          f"pieces, flush with the main's front, each in a {BUCKET} opening "
          f"{RIM_UP} deep; the pieces don't overlap")


def main():
    feet = {what: footprint(name) for what, name in STANDS.items()}
    for what, f in feet.items():
        b = f["body"].BoundingBox()
        print(f"{what} ({', '.join(STANDS[what])}): on the sheet X {b.xmin:.2f}..{b.xmax:.2f} Y {b.ymin:.2f}..{b.ymax:.2f}"
              f"; screw holes {', '.join(f'({x:.1f}, {y:.1f})' for x, y in f['screws']) or 'none'}")
    sheet, pieces = solids(feet)
    checks(pieces, sheet, feet)
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
    same = lambda a, b: bool(a and b) and os.path.normcase(os.path.normpath(a)) == os.path.normcase(os.path.normpath(b))  # noqa: E731
    have = {m.get("name"): m.get("path") for m in sc["models"] if isinstance(m, dict)}
    if not old and all(same(have.get(NAMES[k]), files[k]) for k in files):   # the scene is Robot Studio's: leave it be
        print("scene.json: already lists the board's models at these paths - not touched")
        return
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

#!/usr/bin/env python3
"""
board_pieces.py - sets out the chess pieces (Mike's STLs on the gripper pedestals) in the starting position on the
board mark-up, for the viewer (2026-11-04).

    python board_pieces.py

Pieces (2026-11-05): pawn, rook, bishop, queen, king are the true-CAD rebuilds (chess_pieces.py ->
chessboard/pieces/step/<Name>.step), each fused onto its pedestal and written as viewer_session/piece_<type>.step (real
faces and edges in Robot Studio) plus a .stl mesh of it; the knight is still Mike's STL, tidied (debris dropped,
triangles facing out). Mike's originals: chessboard/pieces/*_10x2mm.stl (bottom at z 0, centred). Pedestal: chessboard/pedestal/pedestal.step (pedestal.py), ONE for every piece: OD 33 x 8.4, a
stepped locating recess on top - queen / king 0.4 down, pawn / rook / bishop / knight 0.8 down. The pedestal stands on the board top.
Board (swivel frame, as swivel_base.py): squares 60, playing edge D_EDGE in front of the yaw axis, centred on it.
White (the human) is at the far end: rank 1 is the far row, the a-file is on the human's left (-y), so h1 is the far
corner on +y. Black (the robot) is on the robot's side. Knights face the other side.
Out (viewer data only, not model files), in the arm's design frame like the board:
  tools/viewer_session/piece_<type>.stl  one mesh per piece type (pedestal + piece), base centre at the origin; the
                                         scene shows 32 + 2 spare queens of them, each its own part, so a game can move
                                         every piece (selfplay.py)
  tools/viewer_session/board_trays.stl   the capture trays either side of the board (white's pieces on the a-file
                                         side, black's on the h-file side), flush with the board top
  board_light.stl / board_dark.stl       the squares
and in scene.json: the piece models (replacing the old two whole sets) and "piece_home" [{part, at, yaw}] - where each
piece starts (Reset pieces / Play game put them there).
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
import cadquery as cq  # noqa: E402

import swivel_base as S  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
PIECES = os.path.join(ROOT, "chessboard", "pieces")
CAD = os.path.join(PIECES, "step")                 # chess_pieces.py: the pieces rebuilt as true CAD (2026-11-05)
CAD_TYPES = ("Pawn", "Rook", "Bishop", "Queen", "King")   # the knight stays Mike's (sculpted) STL, tidied
OUT = os.path.join(HERE, "viewer_session")

PED_H, LOCATE = 8.4, 0.4                # ONE pedestal (pedestal.py, 2026-11-06): stepped recess - the queen / king
PEDESTAL = os.path.join(ROOT, "chessboard", "pedestal", "pedestal.step")   # sit one step (0.4) down, the rest two (0.8)


def seat(name):
    """how far down into the pedestal's top a piece's bottom sits"""
    return LOCATE if name in LARGE else 2 * LOCATE
LARGE = {"Queen", "King"}
BACK_RANK = ["Rook", "Knight", "Bishop", "Queen", "King", "Bishop", "Knight", "Rook"]   # a..h
KNIGHT_FACES_DEG = 90.0          # the knight STL's head points +y; turn it so it faces along x
TYPES = ("Pawn", "Rook", "Knight", "Bishop", "Queen", "King")
COLOURS = {"white": "#ece4d0", "black": "#2c2c31"}
# capture trays: 2 columns at the squares' pitch (the open claw needs the room), 10 rows from beside the robot out
# toward rank 1, 10 mm off the board's side; every spot checked in reach by selfplay.py
TRAY_GAP, TRAY_PITCH, TRAY_ROWS, TRAY_COLS, TRAY_X0 = 10.0, 60.0, 10, 2, -28.0
SCENE = os.path.join(OUT, "scene.json")


def step_mesh(path):
    shape = cq.importers.importStep(path).val()
    verts, tris = shape.tessellate(0.05, 0.2)
    return trimesh.Trimesh(np.array([v.toTuple() for v in verts]), np.array(tris), process=True)


def rot_z(deg):
    return trimesh.transformations.rotation_matrix(math.radians(deg), [0, 0, 1])


def move(x, y, z):
    return trimesh.transformations.translation_matrix([x, y, z])


def to_design():
    """swivel frame -> the arm's design frame (as swivel_base.to_design)."""
    y_wood = S.AB.Y_CUT + S.AB.FOOT_T + S.Z_PLATE0 + S.PLATE_T
    m = np.eye(4)
    m[:3, :3] = [[-1, 0, 0], [0, 0, -1], [0, -1, 0]]
    m[:3, 3] = [0, y_wood, S.AB.Z_MID]
    return m


def square(file_i, rank):
    """centre of a square in the swivel frame: file_i 0..7 = a..h, rank 1..8 (rank 8 next to the robot)."""
    x = S.D_EDGE + S.SQ / 2 + (8 - rank) * S.SQ
    y = -4 * S.SQ + S.SQ / 2 + file_i * S.SQ
    return x, y


def design(x, y, z=None):
    """a point in the swivel frame (z default: the board top) -> the arm's design frame"""
    z = S.BOARD_T if z is None else z
    return [round(float(v), 3) for v in (to_design() @ np.array([x, y, z, 1.0]))[:3]]


def square_at(name):
    """a square's centre on the board top, design frame ("e4")"""
    return design(*square("abcdefgh".index(name[0]), int(name[1])))


def tray_spots(colour):
    """the tray spots for colour's pieces off the board, in filling order (inner column from beside the robot out,
    then the outer one), design frame. White's are on the a-file side (-y), black's on the h-file side."""
    side = -1.0 if colour == "white" else 1.0
    y0 = 4 * S.SQ + S.BORDER + TRAY_GAP + TRAY_PITCH / 2
    return [design(TRAY_X0 + r * TRAY_PITCH, side * (y0 + c * TRAY_PITCH))
            for c in range(TRAY_COLS) for r in range(TRAY_ROWS)]


def setup():
    """[(part, type, colour, square or None, at, yaw)]: the start position plus each side's spare queen in its tray
    (for a promotion). Knights face the other side: yaw about design up."""
    out = []
    for colour, back, pawns, face in (("white", 1, 2, -1), ("black", 8, 7, 1)):
        layout = [(BACK_RANK[i], i, back) for i in range(8)] + [("Pawn", i, pawns) for i in range(8)]
        for name, fi, rank in layout:
            sq = "abcdefgh"[fi] + str(rank)
            yaw = KNIGHT_FACES_DEG * face if name == "Knight" else 0.0
            out.append((f"{colour} {name.lower()} {sq}", name, colour, sq, square_at(sq), yaw))
        out.append((f"{colour} queen spare", "Queen", colour, None, tray_spots(colour)[0], 0.0))
    return out


def write_scene():
    """scene.json: the old whole sets out, one model per piece in, the trays, and piece_home"""
    sc = json.load(open(SCENE, encoding="utf-8"))
    drop = {"white pieces", "black pieces", "white knight g1", "capture trays"}
    models = [m for m in sc["models"] if not (isinstance(m, dict) and (m.get("name") in drop or
                                                                         str(m.get("name", "")).startswith(("white ", "black "))))]
    at = next((i for i, m in enumerate(models) if isinstance(m, dict) and m.get("name") == "board dark squares"),
              len(models) - 1) + 1
    new = [{"path": os.path.join(OUT, "board_trays.stl"), "name": "capture trays", "part_name": "capture trays",
            "color": "#5b3a24", "pick": False}]
    home = []
    for part, kind, colour, sq, pos, yaw in setup():
        ext = "step" if kind in CAD_TYPES else "stl"   # the CAD pieces as STEP: real faces and edges
        new.append({"path": os.path.join(OUT, f"piece_{kind.lower()}.{ext}"), "name": part, "part_name": part,
                    "color": COLOURS[colour], "pick": False})
        home.append({"part": part, "at": pos, "yaw": yaw})
    sc["models"] = models[:at] + new + models[at:]
    sc["piece_home"] = home
    open(SCENE, "w", encoding="utf-8").write(json.dumps(sc, indent=1))
    print(f"scene.json: {len(home)} pieces + trays, piece_home written")


def cad_piece(name, place):
    """the true-CAD piece (chess_pieces.py -> chessboard/pieces/step/<Name>.step) fused onto its pedestal, seated in
    the 0.4 recess - one solid, in the design orientation (base centre at the origin)"""
    from OCP.gp import gp_Trsf
    ped = cq.importers.importStep(PEDESTAL).val()
    piece = cq.importers.importStep(os.path.join(CAD, f"{name}.step")).val().translate(cq.Vector(0, 0, PED_H - seat(name)))
    s = ped.fuse(piece).clean().Solids()
    assert len(s) == 1 and s[0].isValid(), f"{name}: pedestal + piece is not one valid solid"
    t = gp_Trsf()
    t.SetValues(*[float(place[i][j]) for i in range(3) for j in range(4)])
    return cq.Shape.cast(s[0].wrapped.Moved(cq.Location(t).wrapped))


def knight_mesh():
    """Mike's knight STL (not rebuilt: sculpted), tidied: its debris shells dropped, every triangle facing out"""
    m = trimesh.load(os.path.join(PIECES, "Knight_10x2mm.stl"), force="mesh")
    shells = sorted(m.split(only_watertight=False), key=lambda q: -len(q.faces))[:2]   # the piece + the magnet cavity
    for s in shells:
        trimesh.repair.fix_winding(s)
        trimesh.repair.fix_normals(s)
    shells[1].invert()                                 # the cavity faces into the piece
    return trimesh.util.concatenate(shells)


def main():
    ped_mesh = step_mesh(PEDESTAL)
    D = to_design()
    os.makedirs(OUT, exist_ok=True)
    place = D.copy()
    place[:3, 3] = 0                                   # base centre at the origin, design orientation
    for name in TYPES:
        p = os.path.join(OUT, f"piece_{name.lower()}.stl")
        if name in CAD_TYPES:                          # real edges in Robot Studio: the scene loads the STEP
            s = cad_piece(name, place)
            a = cq.Assembly(name=f"piece_{name.lower()}")
            a.add(s, name=f"{name.lower()} on its pedestal", color=cq.Color(0.93, 0.89, 0.82))
            write_step(a, os.path.join(OUT, f"piece_{name.lower()}.step"))
            v, t = s.tessellate(0.02, 0.1)             # and a mesh of it (selfplay measures its profile)
            trimesh.Trimesh(np.array([q.toTuple() for q in v]), np.array(t)).export(p)
            print(f"{name}: {os.path.join(OUT, f'piece_{name.lower()}.step')} (+ .stl)")
            continue
        ped = ped_mesh.copy()
        ped.apply_transform(place)
        piece = knight_mesh()
        piece.apply_transform(place @ move(0, 0, PED_H - seat(name)))
        trimesh.util.concatenate([ped, piece]).export(p)
        print(f"{name}: {p}")
    # the trays: two slabs (white's side, black's side), flush with the board top, standing on the table
    slabs = []
    y0 = 4 * S.SQ + S.BORDER + TRAY_GAP
    for side in (-1.0, 1.0):
        t = trimesh.creation.box(extents=[TRAY_ROWS * TRAY_PITCH, TRAY_COLS * TRAY_PITCH, S.BOARD_T])
        t.apply_transform(D @ move(TRAY_X0 - TRAY_PITCH / 2 + TRAY_ROWS * TRAY_PITCH / 2,
                                   side * (y0 + TRAY_COLS * TRAY_PITCH / 2), S.BOARD_T / 2))
        slabs.append(t)
    trimesh.util.concatenate(slabs).export(os.path.join(OUT, "board_trays.stl"))
    print("trays ->", os.path.join(OUT, "board_trays.stl"))
    # squares: a1 dark, h1 light ("light on the right"); thin tiles on the board top, coloured in the scene
    tiles = {"light": [], "dark": []}
    for fi in range(8):
        for rank in range(1, 9):
            x, y = square(fi, rank)
            t = trimesh.creation.box(extents=[S.SQ, S.SQ, 0.4])
            t.apply_transform(D @ move(x, y, S.BOARD_T + 0.2))
            tiles["dark" if (fi + rank) % 2 == 1 else "light"].append(t)
    for k, ms in tiles.items():
        p = os.path.join(OUT, f"board_{k}.stl")
        trimesh.util.concatenate(ms).export(p)
        print(f"{k} squares -> {p}")
    write_scene()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
bus_preview.py - puts the whole chessboard mock-up (64 squares + the causeway) into the chessboard's own Robot Studio
(Robo chessboard design, 2026-11-06).

    python bus_preview.py      writes tools/viewer_session/board_bus_preview_{cell,causeway}.step and sets the
                               chessboard studio's (port 8766) preview entries: one per square + the causeway

The Professor: "Can you show it in Robot Studio?" then "Let's load the mock-up of the entire board with the ways so we
can start figuring this out." (Never open new windows; never write the main studio's scene - the chessboard has its own,
tools/viewer_sessions/chessboard.) The board frame (cell centres at 60 i, 60 j; z 0 = the underside; x along a rank,
a -> h) goes into the arm's design frame (up = -y, the board top at y 232, the a-file at z +240.6, rank 1 farthest at x
-572), moved PREVIEW_DX along x so it sits on the floor in front of the real board instead of inside it.
One square (his printed bucket + base plate) is written once; the scene places it 64 times by offset (the studio meshes
a file once and reuses it), named a1 .. h8 so he can pick squares by name.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from OCP.gp import gp_Trsf  # noqa: E402

import studio  # noqa: E402
import view_step as VS  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
CELL = os.path.join(ROOT, "chessboard", "cell")
SESSION = os.path.join(HERE, "viewer_session")                      # model files (shared by all studios)
SCENE = studio.scene_path(os.environ.get("ROBOT_STUDIO") or "chessboard")
PITCH, N = 60.0, 8
X_RANK1, Y_UNDER, Z_AFILE = -572.0, 256.0, 240.6     # design frame: rank 1's centre, the cells' underside, the a-file
PREVIEW_DX = -560.0                                  # in front of the real board, on the floor
PREFIX = "chessboard preview:"
CELL_COL, CAUSEWAY_COL = "#e9e4da", "#b5d4f4"   # causeway light (his: dark was unreadable turned over)
BACK_COL = "#c0dd97"                            # the back way (column wires out the back), light green
LOOK = "flat"                  # his: the printed-layer shader "sucks. It's aliasing on the screen really bad when you zoom in"


def to_design(shape):
    """board frame -> design frame: x_d = X0 + y, y_d = Y0 - z, z_d = Z0 - x"""
    t = gp_Trsf()
    t.SetValues(0, 1, 0, X_RANK1 + PREVIEW_DX,
                0, 0, -1, Y_UNDER,
                -1, 0, 0, Z_AFILE)
    return cq.Shape.cast(shape.wrapped.Moved(cq.Location(t).wrapped))


def main():
    bucket = VS.read_step(os.path.join(CELL, "bucket_v3.step"))[0][1]
    plate = VS.read_step(os.path.join(CELL, "base_plate_v3.step"))[0][1]
    cell = os.path.join(SESSION, "board_bus_preview_cell.step")
    a = cq.Assembly(name="board_bus_preview_cell")
    a.add(to_design(bucket), name="bucket")
    a.add(to_design(plate), name="base plate")
    write_step(a, cell)
    ways = {}
    for k, f in (("causeway", "causeway.step"), ("back_way", "back_way.step")):
        ways[k] = os.path.join(SESSION, f"board_bus_preview_{k}.step")
        a = cq.Assembly(name=f"board_bus_preview_{k}")
        for n, s, *_ in VS.read_step(os.path.join(CELL, f)):
            a.add(to_design(s), name=n)
        write_step(a, ways[k])
    print(cell, *ways.values(), sep="\n")

    sc = json.load(open(SCENE, encoding="utf-8"))                   # the current file - only "models" changes
    keep = [m for m in sc["models"] if not (isinstance(m, dict) and str(m.get("name", "")).startswith(PREFIX))]
    mine = []
    for i in range(N):                                              # files a .. h  (board x  -> design -z)
        for j in range(N):                                          # ranks 1 .. 8  (board y  -> design +x)
            mine.append({"path": cell, "name": f"{PREFIX} {'abcdefgh'[i]}{j + 1}", "color": CELL_COL, "material": LOOK,
                         "offset": [round(j * PITCH, 3), 0.0, round(-i * PITCH, 3)]})
    mine.append({"path": ways["causeway"], "name": f"{PREFIX} causeway (a-file edge, 3 pieces)", "color": CAUSEWAY_COL,
                 "material": LOOK})
    mine.append({"path": ways["back_way"], "name": f"{PREFIX} back way (column wires out the back, 3 pieces)",
                 "color": BACK_COL, "material": LOOK})
    sc["models"] = keep + mine
    open(SCENE, "w", encoding="utf-8").write(json.dumps(sc, indent=1))
    print(f"{SCENE}: {len(mine)} preview entries (64 squares + the causeway + the back way), {len(keep)} others kept")


if __name__ == "__main__":
    main()

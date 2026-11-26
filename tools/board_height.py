#!/usr/bin/env python3
"""
board_height.py - the main scene's arm data follows the board's height (swivel_base.BOARD_T, the squares' top over the
robot's bottom face). 2026-11-24 Mike: "If we don't put the pockets on the board, the recesses for the parking stand,
will it still work? I can't get this thing onto the machine to cut those pockets." / "Yeah, so let's do that." - the
base and the dock stand on the sheet's top, BOARD_T 25 -> 22, the board's top 3 lower under the arm (board_y 232 -> 235).

    python board_pieces.py      first: the square tiles, piece_home, the tray spots at the new height
    python board_height.py      then: ik.board_y; every board-level point of the demo moves and the self-play game (their
                                from / to / msq_at - the lifts are heights over the board and stay); board_cells
    python park_dock.py --pose  the park pose (its seat stays put over the wood: park_dock.H)

The board's own models are the Robo chessboard design session's (one_piece_board.py). Written whole, CRLF kept.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_pieces as BP  # noqa: E402

SCENE = os.path.join(HERE, "viewer_session", "scene.json")
r3 = lambda a: [round(float(v), 3) for v in a]  # noqa: E731


def main():
    sc = json.load(open(SCENE, encoding="utf-8"))
    new = BP.square_at("a1")[1]                                  # the squares' top, design y (down)
    old = sc["ik"]["board_y"]
    moved = 0

    def walk(o):
        nonlocal moved
        if isinstance(o, dict):
            for v in o.values():
                walk(v)
        elif isinstance(o, list):
            if len(o) == 3 and all(isinstance(v, (int, float)) for v in o):
                if abs(o[1] - old) < 0.01 and abs(new - old) > 1e-9:
                    o[1] = new
                    moved += 1
            else:
                for v in o:
                    walk(v)

    walk(sc.get("moves", []))
    walk((sc.get("game") or {}).get("moves", []))
    sc["ik"]["board_y"] = new
    squares = {f + str(r): r3(BP.square_at(f + str(r))) for f in "abcdefgh" for r in range(1, 9)}
    cells = sc.get("board_cells") or {}
    cells.update({"squares": squares,
                  "trays": {c: [r3(s) for s in BP.tray_spots(c)] for c in ("white", "black")},
                  "trays_unused": {c: [r3(s) for s in BP.tray_unused(c)] for c in ("white", "black")}})
    sc["board_cells"] = cells
    tmp = SCENE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:                   # text mode on Windows: CRLF, as the file is
        json.dump(sc, f, indent=1)
    os.replace(tmp, SCENE)                                        # whole, so the studio never reads half a scene
    print(f"board_y {old} -> {new}; {moved} board-level points of the moves moved; board_cells at y "
          f"{squares['a1'][1]} ({len(squares)} squares)")


if __name__ == "__main__":
    main()

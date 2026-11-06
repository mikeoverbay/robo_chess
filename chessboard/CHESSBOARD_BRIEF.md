# Robo chessboard design brief

Written 2026-11-06 by the session titled **Robot arm control and kinematics**, which runs the Robot Studio viewer
("Robot Studio" below). This is the handoff for the chessboard session.

The full history is in the memory file `project-robo-chess-arm.md`. It is 131 KB, so grep it instead of reading it whole.
A sister session, **Robo Chess gearbox**, works from `robo_chess\cycloid\GEARBOX_BRIEF.md`.

## The owner
- Mike Overbay (mikeoverbay); call him **Professor**.
- He is an experienced hardware designer: measure causes, don't guess, and give numbers.
- He dictates by voice, so his words are often garbled.
- He likes to pose things himself in the viewer and have you read the result. His words: "they can visualize it where
  you cannot".

## Standing rules
- **Files:** STEP only, no STLs from us. All Python lives in `robo_chess\tools\`. Deleted files go to the Recycle Bin,
  never hard deletes.
- **STEP quality:** keep them clean, with no overlaps and no sliver edges.
- **Money and making:** no spending this month (he orders later). No slicing unless a design question needs it.
- **Directions:** his left/right means standing behind the robot.
- **Never** touch `! Mikes Password List.txt` on his desktop.
- **Materials:** structure in PLA; nylon only for gearbox parts.

## The board today
**Size.** 8×8 squares, 60 mm each, with a 15 mm border, 25 mm thick hardwood.
- These are `SQ`, `BORDER` and `BOARD_T` in `tools\swivel_base.py`. Its `board()` writes
  `chessboard\board_layout.step` as one solid with the dark squares as 0.5 mm pockets. a1 is dark.
- Colours: light #f0d9b5, dark #a8784f, walnut frame #5b3a24.

**Who sits where.** The arm sits at the centre of black's edge, and white (the human) sits at the far end.
- His rule: "not making this arm any bigger", so the board adapts to the arm's reach.
- Fit study: 65.4 mm is the biggest square that fits. The far corners are ~601 mm from the yaw axis; reach at pen
  height is 153..603.

**Coordinates.** The viewer's design frame has up = −y.
- Yaw axis at (x 0, z 30.6); board centre at (−362, 30.6).
- Rank 1 at x −572 (far); rank 8 at x −152 (nearest the robot).
- a-file at z 240.6, h-file at z −179.4.
- Board top y 232. The square tiles' tops are at 231.6.

**Trays.** Each side has a 2-column × 10 tray at 60 mm pitch, 10 mm off the board's side. White's is on the a-file side.
- `board_pieces.py` builds them (TRAY_*) and writes `board_trays.stl`.
- The spare queen sits on each tray's first spot.
- The front rows double as the 20 pen stands of the draw demo. Tell Robot Studio before moving them.
- Tray pitch must be ≥ 55 mm along the finger line; 40 collided with the claw.

**Pieces.** His STLs, measured and rebuilt as CAD.
- `tools\chess_pieces.py` writes `chessboard\pieces\step\<Name>.step`: hexagon sections with twisted bodies.
- The knight is still his own STL (`chessboard\pieces\Knight_10x2mm.stl`), not rebuilt.
- Each piece has a sealed Ø11 cavity for a 10×2 magnet.

**One pedestal for every piece.** `tools\pedestal.py` writes `chessboard\pedestal\pedestal.step`.
- OD 33, height 8.4.
- Stepped recess: 29.3 for queen/king (seat z 8.0) over 25.9 for pawn/rook/bishop/knight (seat z 7.6).
- 3-penny weight cavity from z 2.84 to 7.6, with a 1.04 skin.
- A 6×1.5 magnet in the bottom: this is the one the board will read.
- Claw waist at z 3..5.5, Ø29. Print 34.

**Claw limits.**
- Closed fingers reach 29.5 mm from the grip axis; the pedestal radius is 16.5.
- Low slides keep piece centres ≥ 53.7 apart (they touch at 46).

**Viewer data.**
- `tools\board_pieces.py` writes `viewer_session\piece_<type>.step/.stl`, `board_light/dark.stl` and `board_trays.stl`.
- It also writes `piece_home` and the piece models into `scene.json`.
- `print\Chess pieces.step` holds the 5 CAD pieces plus "Pedestal (print 34)" (written by `print_parts.py`).

## His plans (not designed yet)
- **Human play.** An addressable LED and a Hall-effect switch under every square. The computer detects which piece
  moved, checks whether the move is legal, and accepts or rejects it.
- **Glass over the board?** Thickness unknown. It would raise the playing surface, which changes the arm's reach and lifts.
- **Electronics.** His desktop folder "STEP files" has "Arduino MEGA 2560 R3.STEP" for the case.
- **Old print files.** `chessboard\CE3PRO_pieces.gcode` (42 MB), `pieces.3mf` and `queen.gcode`. I asked whether to
  recycle them; he hasn't answered.

## Who owns what
**You own:**
- `chess_pieces.py`, `pedestal.py`, `board_pieces.py`
- the board part of `swivel_base.py` (SQ/BORDER/BOARD_T and `board()`); the rest of that file is the arm's turntable
- everything in `chessboard\`
- `print\Chess pieces.step`

**Tell Robot Studio first** (short message, your title in the first line) about any change to:
- square size, board thickness or height, or board position
- trays or spare-queen spots
- pedestal height or diameter

Those move the IK reach, the game/demo/draw plans (`selfplay.py --replan`, `demo_swap.py`) and the pen stands.

**Do not edit:** `viewer.html`, `selfplay.py`, `demo_swap.py`, `svg_draw.py` or `show.json`. Change `scene.json` only
through `board_pieces.py`, and tell Robot Studio after.

**Do not start a second viewer server.** http://localhost:8765 is already running.

**Memory:** write to a new `project-robo-chess-board.md` in the same memory folder.

**Git:** `robo_chess` is becoming a local git repository (Robot Studio is setting it up).
- Stage only your own paths with `git add <paths>`, never `git add -A` or `git add .`.
- If `.git\index.lock` exists, another session is committing: wait and retry.
- Commit when the Professor says to.

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
- Fit study (2026-11-04, with an older yaw-axis spot): 65.4 mm was the biggest square that fit.
- Now the a1/h1 square centres are 609 mm from the yaw axis. Piece grabs there pass reach_check in the swap demo and
  the game. The 153..603 reach is the pen tip at paper level, not piece grabs. (Corrected 2026-11-06, Robo chessboard
  design.)

**Coordinates.** The viewer's design frame has up = −y.
- Yaw axis at (x 0, z 30.6); board centre at (−362, 30.6).
- Rank 1 at x −572 (far); rank 8 at x −152 (nearest the robot).
- a-file at z 240.6, h-file at z −179.4.
- Board top y 232. The square tiles' tops are at 231.6.

**Trays.** Each side has a 2-column × 8 tray (plus a 9th row by rank 1, below) at 60 mm pitch, 10 mm off the board's side. White's is on the a-file side.
- `board_pieces.py` builds them (TRAY_*) and writes `board_trays.stl`.
- Each tray's inner column doubles as the draw demo's pen stands (16). Tell Robot Studio before moving them.
- Tray pitch must be ≥ 55 mm along the finger line; 40 collided with the claw.
- **16 spots per tray** (spare queens: see below; the Professor, 2026-11-10: "We don't need more than 16 ... you don't need
  spares"). Clip the two rows at the robot end, where the spare queen was: 2 × 8, centres X 92..512 in the arm frame.
  That gives 32 tray lights and 16 pen stands. DONE by Robot arm control and kinematics in board_pieces.py
  (TRAY_ROWS 8, TRAY_X0 92); checked 2026-11-10.
  - Spot centres X 92..512 at 60 pitch, Y ±295 inner and ±355 outer. These are the old rows 2-9, so reach is
    unchanged.
  - Buckets X 62..542, Y ±265..385. The draw demo's 16 pen stands are each tray's inner column.
- **The spare queens are back, one per side** (the Professor, 2026-11-12: "The extra queens are missing"; relayed by
  Robot arm control and kinematics). The trays stay 2 × 8. Each tray's spare sits in spot 9 (index 8): the outer
  column, the row nearest the robot, centre X 92, |Y| 355. At most 15 captures plus the spare = 16, so a tray never
  overflows. The print list is now 4 queens and 34 pedestals. The board doesn't change.
- **A 9th row, flush with the main's front** (the Professor, relayed by Robot arm control and kinematics, 2026-11-12).
  The tray pieces now run X 47..617 (570 × 145), and there is a 9th row of buckets at X 572, beside rank 1.
  - The inner one (|Y| 295, index 16) is a 17th usable spot. The arm reaches it with the claw up to 104 mm high.
  - The outer one (|Y| 355) is 673 from the yaw axis, out of reach. It is a bucket for looks only, never used or lit.
  - 36 tray buckets in all, 34 of them usable. Indices 0..15 haven't moved. The STEPs are board_tray_white/black.step.
- **Light buckets at the tray spots too** (the Professor, relayed by Robot arm control and kinematics, 2026-11-10).
  Not designed yet: 34 lit (the two out-of-reach buckets stay dark unless he wants them lit for symmetry). Open: a
  Hall under each tray spot as well? The LED chains and Mega pins for 34 more cells.

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

## Lit squares and piece sensing (decided 2026-11-06 with the Professor; not modelled yet)
Robo chessboard design wrote this section.

**What it's for.**
- A lit ring around a square shows which piece the robot picked and where it's going.
- The Hall under each square confirms each pick-up and set-down, and catches the human's moves.
- Rim colours (the Professor, relayed by Robot arm control and kinematics, 2026-11-10): start green, end red, possible
  moves blue; an error blinks red.
- Woods (same relay): Acacia for the frame and trays, Ackland Hevea for the dark squares; textures in `textures\`.

**Each square:**
- An opaque cover in the square's colour.
- A **1.2 mm** translucent ring around the cover (three 0.4 lines).
- One light box per square underneath.
- The pitch stays 60, the board stays 25 thick and the top stays at the same height, so the arm is not affected.
- Still open: whether a 0.8 dark divider goes between neighbouring rings.

**LED.** One TREEDIX WS2812B segment per square: PCB 15.4 × 10.3 × 1.6, 3.11 tall with its 5050 LED. The pads run
GND | data | 5V at both ends. Wiring follows his wall-art method:
- printed conduits with grooves for long bare copper wires
- the data wire is cut between each DO and the next DI

**Hall.** DRV5032FBLPGM, TO-92 (TI SLVSDC7H):
- Pins 1 VCC, 2 GND, 3 OUT; pitch 1.27; leads 15.1-15.5 long.
- Push-pull output; tON 100 µs max; 2.7 mA peak.
- It lies flat on a stem at the square's centre, **bevelled face up** (the element is 0.49 under that face), with its
  leads bent 90° down the stem.
- Today's 6×1.5 pedestal magnet trips it only within about 4.9 mm of the Hall (1.5 cover skin).
  - The robot places pieces close enough.
  - Hand-placed pieces can sit up to about 13 mm off-centre. Covering the whole square needs a 20×2 magnet (pedestal
    change: tell Robot Studio first).

**Matrix (approved; changed 2026-11-06 to v2c: "we're going to turn these on with the output of the nano").** A
diode matrix, 8 rows × 8 columns, on the Nano, with no multiplexer:
- Hall GND goes into the LED segment's GND pad hole (common ground).
- Hall + goes to its rank's ROW wire. The Nano drives it HIGH to power that rank's 8 Halls; LOW = off.
- Hall OUT goes to the diode's anode. The diode's cathode (stripe) goes to its file's COLUMN wire, with a 10k
  pull-down per column at the Nano (8 resistors: the 328P has no internal pull-downs).
- Read: column HIGH = no magnet, LOW = a piece.
- Levels: a row pin sourcing 21.6 mA peak sits at about 4.3 V. The Hall's high output minus about 0.55 V across the
  diode gives about 3.6 V at the column, against the 3.0 V VIH (0.6 V margin).
- Switched-off rows: their Halls' OUT sits near 0 V, so their diodes are reverse-biased and nothing feeds back.
- Scan: one row HIGH, wait 100 µs, read the columns. About 1 ms for the board.
- Why not both supply leads on the LED: the Halls would then be always on, the matrix couldn't select a row, and it
  would take 64 input pins. The Professor chose GND on the LED and + on the row.
- Diodes: his BOJACK rectifiers, 50 V 1 A, so 1N4001; case "BJ-401" (DO-41 size); 125 on hand.
  - The DO-41 standard body is 4.1-5.2 long × 2.0-2.7 diameter, with 0.71-0.86 leads. Get his caliper numbers before
    sizing the trough.
  - At the pull-up's 0.1-0.25 mA the 1N4001 drops about 0.5 V, which is inside the 1.2 V worst case.

**Which controller (the Professor, 2026-11-06, later the same day; replaces "share the arm's Mega").**
- A **BIGTREETECH Octopus** runs the arm's steppers and servo.
- **LEDs:** the 64 board LEDs can run off the Octopus's RGB (Neopixel) header as one chain.
  - Under Klipper this does not stall the steppers. Klipper's src/neopixel.c turns interrupts off for only about
    200 ns per bit, and a disturbed transfer is reported as failed instead of blocking.
  - The Professor, 2026-11-06:
    - The Octopus runs **custom firmware**, flashed from an SD card.
    - A **Pi 4B 2 GB (OctoPi) running Stockfish** sends it commands over serial.
    - A DWIN display goes on the Octopus's display port.
    - "You probably won't be changing the LEDs during the motor movement", so the WS2812B interrupt block is not a
      problem. The rule stands: write the LEDs only while the steppers are stopped.
  - The header pin is not confirmed yet (probably PB0; ask the BIGTREETECH Octopus V1.0 session).
  - LED power comes from its own 5V supply (64 × 60 mA = 3.8 A at full white), with GND common to the Octopus. Only
    the data line comes from the header.
- **Halls:** the 16-pin matrix goes on his **Nano** (ATmega328P), which reports square changes to the Pi over USB.
  Proposed pin map:
  - Digital pins besides serial: D2-D13 plus A0-A5 = 18. A6/A7 are analog-only, with no pull-ups.
  - Rows (outputs, HIGH = that rank's Halls on): D2-D9.
  - Columns (INPUT, each with an external 10k pull-down): D10, D11, D12, A0, A1, A2, A3, A4.
  - Spare: D13, A5, A6, A7. D13 is not a column: the Nano's on-board LED and its 1k resistor would clamp a column
    near the LED's ~1.8 V. D13 or A5 could carry the LED data if the Nano drives the LEDs instead of the Octopus.
  - One row at a time: 21.6 mA peak per row pin, well under the 328P's 100 mA port-group limit.
- The Mega stall below applies only if a stepper-driving AVR also writes the WS2812Bs.

Robot Studio raised the stall on 2026-11-06:
- A WS2812B update turns interrupts off for about 30 µs per LED (24 bits × 1.25 µs). For 64 squares that is
  **1.92 ms** per update.
- On the arm's Mega that would drop about 10 steps per moving joint (5-7 k steps/s from a timer ISR).
- The Hall scan does not cause this: its 100 µs waits leave interrupts on.
- Fixes:
  1. Update the LEDs only while the arm is still. The from/to lights change between moves, but pulsing during a move
     would be out.
  2. APA102/SK9822 LEDs: clocked, so interrupts stay on, and they need 1 more pin. That means buying new LEDs.
  3. Give the board its own MCU. An Uno has 18 free pins besides serial, and the board needs 17.

## Square cell: HIS MODEL IS THE MASTER (2026-11-06)
He took `bucket.step` and `spider.step` into his own CAD and made `chessboard\cell\mikes_cell.step`, then removed
square_cell.py's outputs. Don't rerun square_cell.py over his work.
- **His design:**
  - The bucket has a 1.2 top wall, which is the lit rim, so there is no separate ring.
  - A ledge at z 18.83 holds the wood square in a 57.4 pocket.
  - A solid funnel block down to z 11.07 carries a 6 × 6 Hall post. A light chamber runs from z 6 to 11.
  - A separate 6-thick base plate carries the bottom wiring. The spider is gone.
- **`tools\cell_edit.py` writes `mikes_cell_v2.step`** from his file (his file is only read):
  - All four walls are 2.2. This is his ask, "Consistent".
  - The base plate is 55.2, 0.1 clear of the 55.4 opening. This is his ask, "I don't want to press fit".
- **His re-export (same name, 2026-11-06, 17:09):**
  - It is built on v2: equal 2.2 walls, symmetric, and the 55.2 plate with a 0.1 gap and no overlap. Both solids valid.
  - The funnel is hollowed into a shell, taking the bucket from 27.9 to 17.7 cm³. The shell is open underneath down to
    the plate.
  - The shell is thin by the post: 0.64 vertical at its edge, 0.9 at 5 mm out, 1.2 at 7, and 2.0+ beyond 13.
  - The post runs z 11.91..18.82, solid, with no Hall pocket.
  - The wood pocket is unchanged at 57.4 × 5.18 deep. The wood isn't in the file.
- **His re-export at 17:17:**
  - A 2.0-wide × 2.0 ledge inside the walls at z 3.7..5.7. The base plate, now 3.7 thick, bears up against its
    underside: face contact, no overlap, 0.1 side clearance.
  - The funnel shell is unchanged: still 0.64 by the post.
- **v3 (`tools\cell_edit.py` writes `mikes_cell_v3.step` from his file; his ask: fold the Hall onto the peg, drop
  the peg, route the wiring, no shorts):**
  - **Peg:** dropped from 18.82 to 17.25. The Hall lies on it, bevelled face up, 0.05 under the wood.
  - **NOW: wires out through F1 (his asks: "troughs and holes in F1 ... diagonal"; "color code the pins. Ground is
    black. Red is hot. Signal is white"):**
    - The Hall's pins point +x, bevel up: + (red) at y -1.27, GND (black) at 0, signal (white) at +1.27. Flipping it
      would only swap + and signal, since GND is the middle pin.
    - COLUMN (SIGNAL) RAIL MOVED, his ask "move the signal rail down to where the signal is coming out of the Hall":
      - The column trench runs along y at x +20.6. His old one at x -19 is filled, and the wall notches moved with it.
      - The white wire comes in at y 6 to a diode recess (x 22.6..25.6, y 5.2..19.4, 2.9 deep) between the trench and
        the plate's edge. The body is at y 13..18.2.
      - His ask, relayed by the gearbox session: keep the soldering heat off the diode. The white wire meets the anode
        lead at y 6.8, 6.2 mm from the body. The cathode lead runs in a channel to y 24.3, then across onto the column
        wire, 9.6 mm from the body. The column joint's heat-shrink pocket moved to y 22.5..26.9.
    - HEAT-SHRINK POCKETS on the Hall rails' joints only. His ask: wide enough for the tubing, "but I don't want the
      slot continuously wide because I want it to trap" it.
      - + on ROW and GND on GND: 2.0 wide × 1.7 deep pockets, x 22.6..27.0, in the 1.0 trenches, with 0.6 of narrow
        trench at the plate's edge.
      - The column joint: 2.6 × 2.6 pocket, y 11.1..17.1, in the 1.6 trench.
      - Walls: 1.2 to the column trench, 1.04 to the data trench, 0.7 diode-to-column pocket. No overlaps.
      - The old corner hole in his plate (from my v2) is still there, unused.
    - SIZED FOR 1.5 HEAT-SHRINK (his ask, "large enough so I can have some shrink wrap tubing"):
      - Ledge troughs and the holes through F1 are 1.7 × 1.7, at y -3 / 0 / +3, with 1.3 of plastic between them.
      - From x 23 to the wall the ledge is only 1.4-0.6 thick, so the troughs go clean through it as slots.
      - Face troughs are 1.7 wide × 1.4 deep; with the 0.2 gap to the next bucket that's 1.6 of room. At least 1.08
        of plastic between them, 0.8 of wall behind.
      - Landings under the wall are 1.7 × 1.7.
      - The wires, drawn at 1.5 OD the whole way, clear everything.
    - (earlier, superseded) Ledge troughs, 0.9 × 1.0 under the wood, at y -2 / 0 / +2 from x 14, with a fan-out
      pocket.
    - Through F1 under the rim: one 0.9 × 1.0 hole per wire.
    - Diagonal troughs, 0.9 × 0.7, down F1's outside to the bottom: + to the ROW lane (y -8.5), GND to the GND lane
      (-2.54), signal to y +6 (diode still to place). At least 0.96 of plastic between troughs, 1.5 of wall behind them.
    - Notches through every wall bottom for the rank wires (both x walls) and the column (both y walls); his walls had
      none.
    - Extensions are about 35 mm each (30 AWG kynar). The joints are made from above before the wood goes in, and at
      the bottom.
    - No overlaps. The base plate is unchanged.
  - **(before) his pose, P1.F1:** the Hall's pins point at the +x wall with the leads straight out, unrouted. The peg's
    lead holes and the extensions are gone. The leads end at x 16.7, 11 short of the wall's inside (27.7), and graze
    the funnel shell at the tips.
    - Why: "no way to get in there and wire this ... the cap's in the way". The funnel shell blocks reaching the
      peg's underside from below.
    - He's thinking about running the wires out the side and down between two neighbouring buckets' walls.
  - **(superseded) Hall turned 180° (his ask):** its leads point +y. The peg's lead holes are at y +2.15, with + at x +1.8, GND at 0
    and OUT at -1.8.
  - **Hall leads:** folded at the package and run down through three 0.8 holes in the peg, fanned to 1.8 apart (1.0
    of plastic between them), to z 9.4. That is 9.2 of each 15.1 lead, leaving 5.9 spare.
  - **Extensions:** 30 AWG kynar, heat-shrunk, soldered under the peg from below before the plate goes in.
  - **Base-plate holes (1.0):**
    - + at (-3, -8.5), into the ROW trench.
    - GND at (9.5, -2.0), just past the LED's +x end and opening into the GND trench, so it solders straight onto the bare
      GND rank wire. His P2.F27, "the opposite side"; 1.0 clear of the data trench and 1.1 of the LED pocket.
    - OUT at (-9.6, -5.5), into a diode recess beside the column trench.
  - **Diode recess:** x -17.6..-9.0, y -7.1..-3.9, 2.9 deep, with a 0.8 roof. The anode faces the OUT hole and the
    stripe faces the column trench.
  - **Walls:** 0.86 or more to the GND and ROW trenches and the LED pocket.
  - **Checks:** no overlaps.
  - **Still open:** the shell's 0.64 at the peg; the old corner hole and channel left in his plate from my v2.
- **Still open:**
  - A Hall pocket in the post top (1.52 deep, or the wood can't sit down).
  - The wood sits 2.0 below the rim (pocket 5.18 deep, wood 3.175).
  - No wire path from the post down, and no + / GND holes in the base plate.
  - Where the diode goes now.

## His board pattern: `chessboard\cell\mikes_frame_ways.step` (2026-11-06, 21:08, 257 bodies)
- **What's in it:**
  - 64 cells on the 60 pitch. Each cell is ONE solid: the bucket, base plate, Hall, leads and wire tubes all merged
    (28.67 cm³).
  - Plus, per cell, the look-only column wire + diode as one body, and 2 heat-shrink stand-ins. These were copied from
    mikes_cell_v3.step, which carries them for viewing.
  - He said: "There should be only 64 ... It copied multiple times."
  - The fix: `cell_edit.py` now also writes print-only `bucket_v3.step` (18.21 cm³) and `base_plate_v3.step`
    (10.24 cm³) to pattern from.
- **His "partial way path":** one strip, `mikes_cell_v3.step (1)`, x -54.45..-30.45, y 394.9..450.1, 3.7 thick.
  - It is the v3 base plate turned 90° and cut to 24 wide.
  - Four 1.0 trenches run along y (the board's length) at x -48.35 (ROW), -42.25 (GND), -39.85 (data), -37.31 (5V),
    off the board's -x edge.
  - His plan: run it the whole length of the board as the main bus, fan each rank's lanes into it, and give each
    joint a round soldering pocket.

## Back way (`tools\back_way.py` -> `chessboard\cell\back_way.step`, 2026-11-06)
His words: "I want that signal to come to the back ... everything's going to be back there. We'll have the shortest
run of wires." Also: Hall GND from the LEDs' GND bus, Hall + from the ROW pins along the left edge looking toward
the robot (the causeway), signal out the back.
- **Size and place:** a strip along the rank-8 edge (the arm side), 0.55 off the squares. 479.8 long (the a-file edge
  to the h-file edge), 10 wide, 3.7 thick.
- **Feeders:** each file's insulated column wire leaves the last square's back-wall notch into a 1.6 × 2.4 feeder at
  x 60 i + 20.6.
- **Bundle:** one 5.2 × 2.6 channel (8 × 1.2 wire, 4 across × 2 deep, 1.1 roof), open at both ends.
- **Corner:** it runs to the a-file corner, where the causeway's buses and bundle also leave. Its own corner is left
  clear, so everything meets in one back corner for the Mega.
- **Printing:** 3 pieces, 140.4 / 179.8 / 159.2, cut between files.
- **Checks:** clear of the squares and the causeway.
- **In his chessboard studio:** light green.

## Controller: DECIDED - an Arduino Mega 2560 runs the whole board (the Professor, 2026-11-06, late)
"We're going to use mega2560 to do both. It's got more than enough pins ... We'll just need to build a case for it. To
go with our Octopus."
- The Octopus runs the arm only. The Mega drives the 8 LED data lines (one per rank) and the Hall matrix: 8 row
  outputs and 8 column inputs, each column with a 10k pull-down.
- 5 V logic:
  - WS2812B data needs no level fix.
  - A Hall column reads about 3.6 V when empty (VIH 3.0).
  - A row pin sources the 21.6 mA peak.
  - No steppers on the Mega, so a blocking LED update (1.92 ms for all 64) is harmless.
- Pin idea, from Robot Studio earlier: rows on PORTA (D22-D29, one register write), columns on PORTC (D30-D37, one
  register read), LED data on any 8 others.
  - On the wiring sheet (proposed, not final): ROW rank r -> D(21 + r), LED data rank r -> D(1 + r) (D2-D9), column
    file a..h -> D30..D37 (D30 = PC7 ... D37 = PC0, so file a is the register's top bit).
- Printable wiring sheet: chessboard/wiring_corner_a8.pdf, made by tools/wiring_sheet.py from the CAD numbers.
  - It shows the corner squares a6-c8, the causeway and the back way, with 17 numbered callouts and the Mega pin table.
  - Seen from above, with the robot at the top. Working from underneath, left and right swap.
- Next: a case for the Mega beside the Octopus. His desktop "STEP files" folder has "Arduino MEGA 2560 R3.STEP".
- DONE (his "Yes, rework them"):
  - The data-return lane is out of the cell: no trench, no wall notches.
  - The causeway carries each rank's own LED data wire. Per rank: a D in through-hole and a top groove into the bundle,
    which now holds 8 ROW + 8 data wires (4.8 × 1.4).
  - The causeway is 18.3 wide.
  - Within each rank the LEDs daisy-chain as he describes: the Mega's line goes into the first LED's DI, then each DO to
    the next DI. The data wire is snipped under each LED between its DI and DO pads.

## Controller plan, his previous idea (2026-11-06, evening; superseded by the Mega above)
- GND and 5V run as a bus across the back.
- 8 separate LED data channels, one per rank, instead of one chain. The data-return lane isn't needed any more.
- An ESP32 DevKitC drives the LEDs. It doesn't power them.
- His question: one board, other than a Mega, for 8 LED outputs + 8 Hall row outputs + 8 column inputs (24 pins).
  - The ESP32 DevKitC has exactly 24 usable: 20 I/O + 4 input-only (34, 35, 36, 39). That leaves UART0 for the Pi and
    no spares. Its RMT gives 8 independent LED channels, and its pins source up to 40 mA.
  - The ESP32-S3-DevKitC-1 has ~36.
  - A Pico has 26, but 12 mA pins are too weak for a 21.6 mA Hall row.
- 3.3 V caveats:
  - WS2812B data needs 3.5 V at 5 V supply. Feed each rank's first LED through a 1N4001 (4.3 V), so its output
    re-drives at a valid level.
  - The Hall column high, at 3.3 V row power, is about 2.65 V against VIH 2.48 with a 1N4001. Schottky diodes would
    give more margin.

## Edge causeway, BUILT (`tools\causeway.py` -> `chessboard\cell\causeway.step`, 2026-11-06)
His ask: "make it as wide as you need to make it. And we'll have to work the board around it."
- **Size and place (narrowed 2026-11-10):** 14.0 wide × 4.5 thick × 480 long, off the a-file edge, 0.55 from the
  cells: board x −44.45..−30.45, arm Y −254.45..−240.45. Printed as 3 pieces, 179.8 / 179.8 / 119.8, cut between
  ranks.
  - It was 18.3 wide and reached Y −258.75, across the 3-piece top layer's seam at |Y| 255. It must stay in the main
    piece because its wires go into the main piece's cells. It now ends 0.55 inside the seam.
- **Bare wire (his, 2026-11-07): 22 AWG solid copper, 0.64.** Measured in the STEPs: every bare run is 0.99 wide x
  1.0 deep - plate lanes, bucket wall notches, causeway buses, rank stubs and top grooves. That leaves 0.35 of
  clearance and the wire sits 0.36 below the face. Holes D1.2; bus solder pockets D4.5 x 1.7 deep.
- **Bottom face (bare, soldered from below):**
  - GND bus at x -34.45 and +5 bus at x -37.95 (was -38.45), 1.0 × 1.0, open at both ends.
  - Each rank's GND runs straight into the GND bus.
  - D4.5 × 1.7 solder pockets at both bus joints.
- **Through-holes:** D1.2 at x -32.05 for each rank's +5 / ROW / D in. The +5 comes down again at x -37.95, so it
  crosses over the GND bus on the top face.
- **Top face:** 1.0 grooves, each at its own y, into one bundle channel at x -43.25..-40.85, 2.4 deep. It holds the
  8 ROW and 8 LED data wires (30 AWG kynar, 0.55), 4 across × 4 deep, and nothing is under it on the bottom face.
- **Clearances, measured by causeway.py:** nothing crosses.
  - At least 1.34 of plastic between neighbouring holes, and 0.64 between the GND pocket and the D in hole.
  - The thinnest plastic between a top cut and a bottom solder pocket is 0.76: the +5 pocket's rim beside the bundle.
    It was 1.8 before the narrowing, and the script now asserts at least 0.7.
- The return lane and the data hops are gone: each rank has its own data wire (see the Controller section).
- The column wires leave by the back way.
- **Wiring sheet:** `tools\wiring_sheet.py` reads these numbers from causeway.py.
- **Superseded:** the edge_bus_gnd / 5v strips (files kept). In Robot Studio the preview now shows the cells plus the
  causeway.

## Edge causeway = the terminator (his sketch, 2026-11-06, later than the two-edge strips below)
- One causeway along one edge ends every rank. It carries:
  - GND and +5 for the LEDs
  - the Halls' ROW power, 8 separate lanes, one per rank
  - That's 10 lanes, about 27 wide at 2.54 pitch.
- Each rank's wires reach their lane through staggered through-holes ("Thru ⌀"). A tap crosses the other lanes on the
  other face, so it touches only its own lane.
- DATA IS NOT ON THE BUS. His words: "It's better to lift the DN and not use it on this bus, and then use it on the
  other side". A shared data lane would drive every rank's D in at once.
  - The chain runs rank to rank with its own short hops, snaking: the hops alternate ends.
  - The first rank's D in comes from the controller.
- This supersedes the two-edge GND / 5V strips below unless he says otherwise.

## Edge buses (his picks 2026-11-06: "5V + GND, rest out"; "Two edges") - superseded by the causeway above?
`tools\edge_bus.py` writes `chessboard\cell\edge_bus_gnd.step` and `edge_bus_5v.step`, in his board's coordinates
(cell centres at 60 i, 60 j; z 0 = the underside).
- **GND strip:** off the -x edge, where his strip was: x -54.45..-30.45.
- **5V strip:** off the +x edge: x 450.45..474.45.
- **Each strip:** 480 long (the board's length) × 24 × 3.7, 0.55 off the cells.
  - A 1.0 × 1.0 bus trench runs end to end, open at both ends, 12 in from the board side.
  - 8 feeders: each rank's GND (or 5V) lane runs straight on from its end-wall notch to the bus.
  - A round D5 × 1.7 pocket at each joint gives room to solder.
  - Everything is open on the underside, and nothing crosses. Checked: clear of the edge cells, every feeder lines up
    with its lane's notch.
- **ROW** (8 wires to the Nano) and **data** (rank-to-rank hops) are not in the strips yet.
- **Shown in his Robot Studio window** (`tools\bus_preview.py`, after telling Robot Studio):
  - 3 entries in scene.json, named "chessboard preview: ...", files `viewer_session\board_bus_preview_*.step`.
  - They show the 16 cells of the two edge files plus the 6 strip pieces, in the design frame.
  - Moved 560 along -x so they sit on the floor in front of the real board.
  - Re-running only rewrites the files.
- **Printing (his pick, "Print in sections"):** each strip is 3 bodies in its STEP.
  - Cut between ranks 2|3 and 5|6, at y 150 and 330, with a 0.2 butt gap.
  - Pieces are 179.8 / 179.8 / 119.8 long, holding ranks 0-2 / 3-5 / 6-7.
  - Each cut is 24.9 or more from the nearest solder pocket.
- **Trays:** the strips sit under the board's 15 mm border, out to 24.45 past the squares' edge. The trays start 25
  past it (15 border + 10 gap), so the case wood around the strips has little room on the a-file and h-file sides.

## Square cell, draft v2d (2026-11-06) - superseded by his model above

**v2d (his ask: "utilize the length of the leads ... so we don't have to add extensions"):**
- The diode now sits at the CORNER end of the wire leg (anode end at u 23.6), centred on the leg.
- The Hall's own OUT lead runs along the leg's groove to u 13.1. The diode's anode lead can reach back to u -1.5, so
  there is up to 14.6 mm of overlap to solder at the bench; it's drawn trimmed to a 5 mm overlap.
- The cathode lead goes straight down the corner hole to the COLUMN wire, which moved to x -19 beside that hole. It
  uses 19.9 of its 25.4 mm.
- The Hall's + and GND still need extensions: even a straight line to the LED's GND pad is about 19.6 mm against a
  15.1 mm lead.
- The text below is from v2c; where it differs, the diode position and column x are as above.

`tools\square_cell.py` writes `chessboard\cell\square_cell.step`. v1 had a tall wiring strip inside the bucket and the
Hall on a centre stem, and he rejected it.

**His rules:**
- Wire channels on the bottom, outside the buckets.
- A separate corner-to-corner frame (the "spider") with a post for the Hall.
- Every solder joint reachable from the bottom: the board is soldered upside down before the bay is closed.
- The Hall's post turned 45° so the wires leave down one leg.
- + and GND go straight down through the floor.
- He lights the bucket itself, not the wood, so shadows inside don't matter.

**What v2c has** (z 0 = the bucket's underside, the play surface at 24):
- **Bucket:** a white pyramid funnel down to the LED at the centre (8 wide at z 4). The top has a 0.4 black wall (M600
  at z 21), the ring pocket and four 7 × 7 corner seats at z 14.
- **Underside of the bucket:**
  - **LED segment:** in a pocket, pads down, its LED up through a 5.4 hole.
  - **Bare rank wires:** in 1.0 × 1.0 channels. GND -2.54, data 0 and 5V +2.54 run under the LED's end pads; ROW is
    at -8.5. Cut the data wire under each LED, between its DI and DO pads.
  - **Column wire:** insulated, at x -12, in a channel 2.4 deep, so it crosses 0.30 above the bare rank wires.
  - **Cathode channel:** from the corner hole (-21.77, -21.77) over to the column.
  - **Floor holes (1.2):** one at (-6.4, -2.54), over the LED's GND pad, where the Hall's GND goes into the pad hole.
    The other at (-4, -8.5), where the Hall's + lands on the ROW wire. The pad position is 1.3 in from the board's end (the Professor: "Use 1.3"; not measured).
- **Spider:**
  - Arms 6.8 wide × 3.5 tall, square ends filling the seats.
  - A 3.6 post turned 45° holds the Hall (bevelled face up) against the cover.
  - The wire leg (-x-y) has a 2.2 groove down the post and a 2.0 drop hole near the post for + and GND.
  - The diode lies in a 2.9 trough: anode toward the Hall's OUT, stripe at the far end. Its cathode lead runs along a
    groove to the 1.9 corner hole and down.
- **Hall wiring:** the Hall's three wires (30 AWG kynar) and the diode are soldered at the bench, before the spider goes
  in.
- **Checks:** no part overlaps.
- **Open questions:**
  - The bucket is 48 cm³ solid, so it needs low infill.
  - Glowing buckets that touch may light their neighbours.
  - The case support that leaves the undersides reachable.

## One-piece board: base sheet + 25 mm top layer (the Professor, relayed by Robot arm control and kinematics, 2026-11-10)
Robo chessboard design wrote this section. It replaces the stacked box below; the electronics-bay rings are gone.
**Arm frame:** yaw axis at 0,0; X toward the player; Y to the player's right. From our board frame: X = 572 − y,
Y = x − 210 (rank 8 nearest the robot at X 152, the a-file on the player's left at Y −210).
- **Base sheet** under everything: X −225..762, Y ±400 (987 × 800). Without the dock its back edge is X −141 (903 deep).
  Robot and dock screw to the bare sheet; arm heights unchanged.
- **Top layer, 25 thick, U-shaped:**
  - It runs full width from X 107 to the front edge at 762. The squares are X 122..602, Y ±240.
  - Two tray arms run back to X 47 at |Y| 250..400 (X −73 before the 16-spot trays). Tray buckets are X 62..542,
    Y ±265..385.
  - Between the arms, behind X 107, it is open for the robot.
- **His asks:** front margin = side width = 160; room at the back for the robot's base.
- **Robot base** X −126..65, Y ±65; turntable Ø154 at 0,0. **Dock** X −210..−69, Y 50..184.
- **Our wiring fits the borders:**
  - The causeway is Y −258.75..−240.45, leaving 6.3 to white's tray buckets at −265.
  - The back way is X 111.55..121.55, leaving 4.5 to the layer's back edge at 107.
- **Open, ours to settle with him:**
  - The base sheet's thickness.
  - How the layers join.
  - 800 is wider than his 2×4 ft stock and the CNC's 610 Y travel. DECIDED (his pick, 2026-11-10): buy a 4×4 ft
    sheet for the base. It is still wider than the CNC's Y travel, so saw the rectangle and drill to templates.
  - The cells are 24 tall and two 1/2" plies are about 24, against the 25 asked.
  - **Flush rims (his ask, relayed 2026-11-10):** the main session's `tools\one_piece_board.py` puts the face at sheet
    + 27, flush with the rims, and the wood at sheet + 25 (the arm's height). A 24 cell then has its underside 3.0 above
    the sheet. Options sent to the main session:
    - A (my pick): a 3 mm pocket under the robot, so the top layer is 24, two 12 mm plies.
    - B: a 3 mm sub-floor under the cells.
    - C: 27-tall cells.
  - **DECIDED: A** (his pick, relayed 2026-11-10; built by `tools\one_piece_board.py`):
    - The sheet's top is 3 above the robot's face, and the cells sit on the sheet.
    - The top layer is Z 3..27 over the robot's face, 24 thick, with its face flush with the rims. It covers X 107..762,
      and the tray arms X 47..107 at |Y| 250..400.
    - One 3-deep pocket for the robot and dock together, 0.5 clearance all round: robot base X −126.5..65.5, Y ±65.5;
      dock X −210.5..−68.5, Y 49.5..184.5.
    - The sheet is 12 thick, a guess until he measures it.
  - **Top layer in 3 pieces** (relayed 2026-11-10; his CNC's window is 24 in). All pieces are 24 thick.
    - Main board: X 107..617, Y ±255, 510 × 510. The front is cut 15 past rank 1, his ask.
    - Tray pieces: X 47..617 (flush with the main's front, 2026-11-12), |Y| 255..400, 570 × 145 each. White's is on −Y. Each butts the main's side edge.
    - The base sheet, 800 × 842, is over his window, so the robot/dock pockets are open again. The main session
      suggested option B: a 1/8" hardboard bottom in each piece.
  - **What the main piece must leave room for, under its border:**
    - The causeway: Y −255..−240.45, X 122..602, 4.5 tall.
    - The back way: X 111.55..121.55, Y ±240, 3.7 tall.
    - With two 12 mm plies, both are through-slots in the lower ply. The wires leave at the back a-file corner (X 107,
      Y −241..−255) into the open area beside the robot.
  - Where the Mega and the Octopus go.

## One-piece wooden base (the Professor, 2026-11-06; concept only; SUPERSEDED in part by the section above)
**His aims:**
- A CNC-cut wooden case that holds everything: the squares' light buckets and conduits, the Nano, the Octopus, and the
  display at the front (the human's end).
- The arm mounted on the same wood, precisely located: he named a raised boss under the arm's case.
- No cable between the board and a separate controller ("little Jimmy" tripping and yanking it out).

**Rule for keeping the arm's plans valid:**
- The yaw axis stays 122 behind the playing edge, on the board centreline.
- The arm's mounting face stays 25 below the playing surface.
- The pitch stays 60.
- Given those, the base can be as deep as the electronics need.

**Arm interface** (confirmed by Robot Studio, 2026-11-06; swivel_base.py frame, +x toward the board; the rank-8 edge is
at x +122, which is design x -122):
- The flange is x -78..62, y ±62, 8 thick, with a #8 wood screw at each corner 14 in from both edges (FLANGE_HOLE_IN).
- The covers' lips wrap the flange edges, with #4 screws going in sideways (FLANGE_SCREWS). A pocket must leave room
  for the lips (3.2 a side), or locate with dowels.
- The arm cable leaves along the wood out the right (-y) side: slot x 2..22, 16 tall from the flange bottom
  (CABLE_X0/X1/H). The base needs a matching channel there.
- Wires also come down through the 28 mm turntable centre. A hole under the yaw axis into the bay below would hide
  them completely.
- The locator is Robot Studio's part to approve: send them the proposal.

**Rough footprint:** about 770 wide (board 510 + trays 2 × 130) × about 790 deep (behind the arm to the front display).

**Materials (the Professor, 2026-11-06):**
- **Square covers:** 1/8" (3.175) craft wood, cut on his table saw, stained natural (light) and walnut (dark).
  - With the Hall face glued to the underside the gap is 3.96, giving 52 mT at the centre. It trips within 5.6 mm of
    the 6×1.5 magnet, so no Hall pocket is needed.
  - A 12×24" sheet gives up to 5 × 10 squares of 57.6 with a 1/8" kerf. Buy 3 sheets: 2 for the 64, 1 for spares and
    stain tests.
- **Box:** 1/2" solid-core birch ply, stacked and glued in layers. His CNC travels 4 ft (the other axis is unknown).
- **Footprint:** 770 wide (board 510 + trays 2 × 130) × 713 deep, about 813 with a 100 mm display apron at the front.
- **Draft stack, 12 mm per ply (measure his sheets):**
  1. Top frame: border, trays, square-field opening.
  2. Second frame: the field opening, plus the cut-out for the arm.
  3. Deck: the cells and the arm flange sit on it. It carries the holes for the wires and the arm-centre hole.
  4. Electronics-bay ring.
  5. Electronics-bay ring.
  6. Electronics-bay ring (the three rings give a 36 mm bay).
  7. Bottom. It can be an inset panel cut from a ring's drop-out, which saves a layer.
  - Total about 84 tall.
  - **His CNC is 2×4 ft: X 1220 (4 ft) × Y 610 (2 ft)**, so no 770 × 813 layer fits in one piece. APPROVED
    2026-11-06: each layer is cut as pieces that fit 1220 × 610, lined up with dowels:
    - frames and bay rings as rails: side rails about 145 × 813, front and back rails 770 long
    - the deck as two halves of 385 × 813
    - seams staggered from layer to layer
    - CNC-drilled dowel holes at the same XY in every layer to register the glue-up (the same holes can locate the arm)
  - Area: about 2.06 m² of ply (frame layer 375k, ring 228k, deck 626k mm²), so about **5 sheets of 2×4 ft** at 60-70%
    nesting. A 1/4" bottom panel can be table-sawn.
  - Two plies = 24, not the arm's 25. Shim the flange 1 mm or have Robot Studio replan, once his real ply thickness
    is known.
- **20×2 magnet under 1/8" wood:** 95 mT at the centre, but a dead band at 12.0-13.2 mm off-centre. Recheck before
  choosing it.

**Not yet known:**
- his CNC's other axis
- his second way to locate the arm
- whether the power supply and the Pi go inside
- the display's size

## His plans (not designed yet)
- **Human play.** The computer detects which piece moved, checks whether the move is legal, and accepts or rejects it.
  The squares section above covers the hardware.
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

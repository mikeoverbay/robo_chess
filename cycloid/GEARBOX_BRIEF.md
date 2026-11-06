# Robo Chess gearbox brief

Written 2026-11-06 by the session titled **Robot arm control and kinematics**, which runs the Robot Studio viewer
("Robot Studio" below). Handoff for the gearbox session. Full history lives in the memory file
`project-robo-chess-arm.md` (131 KB). Search it with grep; it is too long to read whole.

## The owner
- Mike Overbay (mikeoverbay). Call him **Professor**.
- He is an experienced hardware designer. Measure causes, don't guess, and give numbers.
- He dictates by voice, so expect garbled words: "gauze/Gauss/pens" = pins/dowels, "globe gears" = lobe gears,
  "stand-up fans" = standoffs.

## Standing rules
- STEP only, no STLs from us. All Python lives in `robo_chess\tools\`.
- Use the Recycle Bin, never hard deletes.
- STEP files stay clean: no overlaps, no sliver edges.
- No spending this month; he orders later.
- His left/right means standing behind the robot.
- No slicing unless a design question needs it.
- Never touch `! Mikes Password List.txt` on his desktop.
- Materials: PA12 nylon ONLY for the cycloid discs and gearbox parts. Links and structure are PLA.

## What is identical
The four ARM joints all use the SAME gearbox, printed 4 times (`print\Gearbox assembly (print 4).step`):
J1 shoulder, J2 elbow, J3, and J4 wrist. Only the motors differ.
The turntable (J0) is NOT a cycloid. It is a printed bevel pair, m1.5 18:72 = 4:1 (`tools\swivel_base.py`).

## Files
- `tools\cycloid.py --spread 14` writes `cycloid_spread\step\` and `report.txt`. **This is the current design.**
- **Bolt circles are shared** (Professor, 2026-11-06: change one and the arms, shoulder column and claw mount must
  follow). Output 3 × M3 r 15 at 0/120/240°; lid 6 × M3 r 32.4 from 1.58° every 60°; output face z 59.2.
  - Readers: mockup_link.py (arms), arm_tube.py (shoulder column), claw_adapter.py --spread (claw mount), then
    print_parts.py.
  - `tools\gearbox_fit_check.py` checks the printed files hole by hole against cycloid.py. It must say ALL FIT.
    It did on 2026-11-06.
- `tools\cycloid_exploded.py [--gap 12]` writes `cycloid_spread\step\assembly_exploded.step`: every part pushed up the
  motor axis in assembly order (Robo Chess gearbox, 2026-11-06). Re-run after cycloid.py.
- Plain `cycloid.py` writes `cycloid\step\`, the older narrow-bearing version (not rebuilt since the star key).
- `complete assembly\cycloid gearbox` is an NTFS JUNCTION to `cycloid\`. Never replace it with a copy, and never
  clean it as a duplicate.
- `tools\print_parts.py` writes `print\*.step`. `tools\cycloid_standin.py` writes `cycloid\cycloid_drive_standin.step`.
  `tools\nema17.py --length N` writes `cycloid\nema17_N.step`.
- `tools\motor_speed.py`: the torque and pull-out model per joint, which sets the viewer's speeds.
- Superseded, kept for history: `planetary.py`, `gearbox.py`, `gearbox_step.py` and `planetary_12_18_48\` (the 5:1
  planetary), plus `gear box.step` and `assembly.step`.

## How we got here
1. **Planetary.** He rejected a belt SCARA and asked for a planetary on a NEMA 17: module 1, sun 12 / planet 18 /
   ring 48, 5:1, planets on 686ZZ bearings.
2. **The arm couldn't lift itself.** The load check on 2026-11-03 used 786 g per link. Straight out, the arm needed
   J2 10.5, J3 5.5 and J4 1.7 N·m, against 1.8 available from a 17HS19 + 5:1.
   gear_ratio.py, with the gripper + 60 g and ×1.5, said J2 needs ~50:1, J3 ~27:1, J4 ~9:1.
3. **Options tried:**
   - Two stacked 5:1: fails (the second sun reaches 88/47 MPa).
   - Internal ring gear: works but is big.
   - A second planetary stage sized for the torque: works.
   - **Cycloidal (his "gyrocentric")**: chosen.
4. **Cycloid reference data.** A printed PLA 25:1 (HowToMechatronics) measured 65 % efficiency and skipped at 15,
   then 21.5 N·m. That is 88 N max pin force per disc, from F = 4T/(K1·Z·R).
5. **DECIDED 2026-11-03:**
   - Atlas-style layout straight on the motor, 37:1, one design for every arm joint.
   - He ORDERED hardware against it, so hold the model to that hardware.

## Design philosophy (his)
- **One good printable gearbox for every joint.** Steel where the load is (dowels); print the rest.
- **One flag** switches printed pins (for testing) to steel dowels.
- **"Make the holes through where you can, I can use longer screws."**
- **Outside kept plain** for his shell standoffs.
- **Walk the assembly order.** Overlap checks can't see an unassemblable part: the first one-piece eccentric
  couldn't pass a 6800 over its collar.
- **Measure the first built box** before trusting the efficiency figure.

## Current design (cycloid_spread)
**Gear geometry**
- 38 ring pins, 3 mm, on a 56 mm circle; 37 lobes → **37:1**.
- Eccentricity 0.368 (K1 0.50); lobe height 0.74.
- Profile from the Younis formula, pulled in 0.10 all round (CLEAR, `--clear`). Disc 2 is disc 1 turned 180°.

**Output and discs**
- 8 output pins, 3 mm, on a 41 mm circle, in 3.84 mm holes. A support ring holds the pins at both ends; with one
  end only they hit ~490 MPa.
- Two discs 6 mm thick (the same part), printed solid. The tested printed drive cracked at its eccentric bearing first.

**Eccentric**
- Two pieces (lower and upper) on 6800s (Ø19 bore).
- **Star-keyed** 180° apart: KEY_STAR has 6 points, r 4.3/3.4, male 1.0 tall, socket 1.2 deep + 0.1 clearance.
  The lower one, fully on the D-flat, drives the upper one.
- It can't be one piece: each 6800 must slide past an 11 mm collar.

**Bearings and body**
- Output on 2 × 6808, spread 14 apart (ball rows 21 apart, was 7) by sleeves: outer 51.8/46, inner 44/40.2.
- Body Ø70.5. Motor face to the output seat: 59.2.

**Hardware per joint**
- 46 dowels, 3×20, hardened ISO 8734 m6 (52100 steel).
- 2 × 6800 and 2 × 6808.
- Lid screws (Professor 2026-11-06): 6 × M3×55 socket head cap screws, partly threaded (18 thread, ISO 4762).
  - He chose 55 because M3×58 is special order.
  - Shank through lid + pin ring + a Ø3.4 clearance in the housing top down to z 18.0.
  - 17.3 mm of thread self-taps the Ø2.5 hole. Tip at z 0.7.
- **CONFIRMED and ORDERED by the Professor (2026-11-06):** M3×55. "Keep the bolts the size that they are ... It's all
  confirmed already." The arm-mount screws from below share these 6 holes; he has that in hand.
- Screws: 4 × M3×8 for the motor; 3 × M3×4×5 heat-set
  inserts (his, Amazon B0FDKYTMZ8: 4 long, 5 knurl) in the output flange face, r 15 at 0/120/240°: Ø4.2 holes
  (`--insert-hole`) 4.5 deep with a 0.3 lead-in; centre pocket Ø22 → Ø20 for a 2.5 knurl wall
  (Robo Chess gearbox, 2026-11-06).

**Checks**
- No penetration over 720 poses; least gap 0.100.
- Wall clearance 0.46; output pin gap 0.050.
- No part overlaps at 3 angles in both pin variants.

**Backlash and mass**
- Backlash 1.13° at the output (disc 0.85 + pin holes 0.28) = 9.6 mm at the wrist.
- Printed set 196 g (flange 39.6 since the insert change).

**Shafts**
- One design fits every motor shaft. The D-flat must start ≤ 10.75 from the motor face.
- The 20 mm pancake shaft keeps 4.3 of 6.5 mm in the upper eccentric: OK for the wrist.

## Motors
| Joint | Motor | Notes |
|---|---|---|
| J0 turntable | 17HS19 48 mm | through the 4:1 bevel |
| J1 shoulder | NEMA 17, 60 mm, 73 N·cm, 2 A | he buys it next month; coil R/L ESTIMATED; NEMA 23 plan superseded |
| J2, J3 | 17HS19-2004S1, 48 mm, 59 N·cm | 2 A, 1.6 Ω (his datasheet drawing A0842; motor_speed.py had 1.4, fixed 2026-11-06), 3 mH; shaft 5×24, D-flat from 9 mm |
| J4 wrist | 17HE08-1004S pancake, 17 N·cm, 1 A | shaft D5×20 D-cut. On 2026-11-06 he first proposed a 40 mm 44 N·cm 1.7 A NEMA 17 (+0.14 kg at J4: J1 ×1.16 → ×1.08), then named this pancake: it is the current choice |

## Torque (`motor_speed.py`: 24 V, 37:1 at 65 %, ×1.5 margin, input capped at 600 rpm until bench-tested)
| Joint | Worst gravity at output | At the motor | Low-speed margin |
|---|---|---|---|
| J1 shoulder | 10.22 N·m (far corners, at the board) | 0.425 N·m | **×1.16** - THE weak joint |
| J2 | 5.37 | 0.223 | ×1.79 |
| J3 | 1.92 | 0.080 | ×5.0 |
| J4 | 0.10 | 0.004 | ×26.6 |

- **Output capacity:** motor torque × 37 × efficiency. 17HS19 at 65 %: stall 14.2 N·m; moving ~9.6 (0.40 N·m at 300 rpm × 37 × 0.65).
- **J1 fixes:**
  - counterbalance spring
  - higher ratio (69:1 discussed for the shoulder)
  - better efficiency
- **Efficiency (pitch-point model):**
  - Ball bearings cost ~1 %. The losses are sliding at the ring pins (μ × 2.48 of output power) and the output pins (μ × 0.87).
  - Result: 83 % at μ .05, 74 % at .08, 68 % at .10, 62 % at .12, 54 % at .15.
  - The 65 % we use corresponds to μ ≈ 0.11.
  - Biggest lever: rollers or sleeves on the ring pins (loss × pin r / sleeve r).
- **Stresses at the old J3 design torque 9.38 N·m:**
  - Ring pin 36 N per disc, 0.41× the tested failure.
  - Output pin 114 N; bending 134 MPa with both ends held (203 at stall).
  - A printed 3 mm pin is good for only ~1.8 N·m.
  - 6800: ~530 N (802 at stall).
- **Spur teeth (Lewis):** allowable 20.7 MPa, half of 41.4 MPa moulded nylon (no printed figure has been published).

## Open items
- Shoulder (J1) margin.
- Ring-pin rollers.
- Bench test on the first build: string + weight on the output, lever + scale on the input. The Uno sketch is not written.
- The stock `cycloid\` folder is not rebuilt with the star key.
- His arm.step motor pocket is 57.1 across the chamfers; the real 17HS19 must be smaller (he should measure it).

## Working with the Robot Studio session
- **Robot Studio owns:** viewer.html, view_step.py, selfplay.py, demo_swap.py, svg_draw.py and
  viewer_session\scene.json / show.json.
- **The gearbox session owns:** cycloid.py, cycloid_standin.py, gearbox*.py, planetary.py, cycloid*\ output and the
  Gearbox print file.
- **Shared:** motor_speed.py and scene.json `ik.motion` (ratio, mesh, speeds). Tell Robot Studio about any change to
  ratio, efficiency or speed, and it updates the viewer.
- **Do not start a second server.** Robot Studio already serves http://localhost:8765.
- **Memory:** record gearbox facts in a new `project-robo-chess-gearbox.md` in the same memory folder. Robot Studio
  edits `project-robo-chess-arm.md`.
- **Messages:** short, with your title and the finding in the first line.

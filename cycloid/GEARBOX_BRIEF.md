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
- Plain `cycloid.py` writes `cycloid\step\`, the older narrow-bearing version (stale: last built 2026-11-03).
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
- Ring pin wall grooves in the housing ring are D3.2, radius 1.6 (`--ring-pin-hole`). The floor sockets were too, until
  his two-piece housing made them D3.4 (2026-11-15, below).
  - The Professor, 2026-11-07: his printer prints holes small; "make all of those radius 0.2 bigger", then
    "Can we make it 0.1? So the radius will be 1.6."
  - If they print true, a pin can move 0.1 out and backlash grows 1.13° → 1.88°: about 0.38° per 0.1 mm.
  - Thinnest walls: 1.18 to a lid screw hole, 1.42 between grooves.
- Eccentricity 0.368 (K1 0.50); lobe height 0.74.
- Profile from the Younis formula, pulled in 0.10 all round (CLEAR, `--clear`). Disc 2 is disc 1 turned 180°.
- Fitted to Mike's printer and housing (2026-11-17, from his test disc at Cura horizontal expansion 0/0; his pin gauge
  puts the pins' inner edges on 52.90). His words: "reduce the OD on diameter by 0.05. Keep the pin radius, but move
  everything in" - the profile is drawn for pins on 55.95 (`DISC_PIN_R`); output holes +0.1 in the model only
  (`OUT_HOLE_PRINT`, 3.94); bore 18.98 ("just a little tighter"). Print the discs at Cura 0/0. Also written as
  `print\Revised lobe disk.step`. (Robo Chess gearbox)

**Output and discs**
- 8 output pins, 3 mm, on a 41 mm circle, in 3.84 mm holes. A support ring holds the pins at both ends; with one
  end only they hit ~490 MPa.
- Two discs 6 mm thick (the same part), printed solid. The tested printed drive cracked at its eccentric bearing first.
- Pull holes (Mike 2026-11-14, from his `print\lobe.step`): 2 × Ø2.5 through (M3 self-tap) on r 17.17 at 90°/270°
  about the disc's own centre, between output holes, with a 0.8 × 45° lead-in on the top face. Fit the lead-in face up.
  Two M3 screws lift a disc out; disc 1 brings the eccentric with it. (`PULL_*` in cycloid.py.)

**Eccentric**
- **ONE PIECE** since 2026-11-06 (Professor: "not really liking that little star coupling"). The star-keyed halves
  are gone. It runs z 7.5..27.25 on 6800s (the discs' Ø19 bore). Bottom to top:
  - neck Ø8.5 on cam 1's axis, with a 45° lead-in
  - cam 1 Ø10 at +e
  - middle collar Ø11, concentric
  - cam 2 Ø10 at −e
  - a Ø9 step, concentric, to z 23.05
  - the top journal Ø8.15 (modelled over 8 for his printer; `TOP_JOURNAL`), z 23.05..27.25, 0.3 lead-in on top
- **Top bearing** (Mike 2026-11-17: "support the end of the camshaft and put it into the output"; "I want to use these
  thinner, smaller ones"): an 8 × 14 × 4 flanged bearing, 15.6 × 0.8 flange (MF148 / F678 size), PRESSED onto the
  journal on the bench, flange down, 0.2 above the Ø9 step. The output flange's underside has a Ø14 finger-push
  pocket (to z 27.55) and a Ø15.9 × 0.85 flange seat; it slides down over the bearing at assembly. It props the
  eccentric's top: peak motor shaft bending about 25-40 % less, but ONLY with no play (scratchpad top_support2.py:
  0.03 of play and it carries almost nothing). The steel of a 24 mm shaft ends at z 24, inside the bearing.
- The D grips z 10.75..27.25 (the motor shaft only reaches 24).
- A solid one-piece can't take both bearings from the top. The lower 6800 can't pass cam 2 (offset 0.74 the other
  way) or the collar, and there is only 1.5 between the bearings.
- So disc 1's bearing goes on FROM BELOW. The old bottom collar is a loose **eccentric spacer** ring (Ø12.2 × Ø8.8 ×
  3.2, on cam 1's axis, inner ring only, no torque).
- Assembly is a cartridge: both discs go onto the eccentric on the bench, then the lot drops onto the shaft.
- Checked: nothing below cam 1 or above cam 2 sticks out of a 10 bore on that cam's axis.

**Bearings and body**
- Output on 2 × 6808, spread 14 apart (ball rows 21 apart, was 7) by sleeves: outer 51.8/46, inner 44/40.2.
- Body Ø70.5. Motor face to the output seat: 59.2.
- Housing in two pieces (Mike 2026-11-15, his `print\hosuing 2 piece .step`, then "build the two-piece housing into
  cycloid.py"): `housing_floor()` z 0..7.2 and `housing_ring()` 7.2..23.2; the gearbox lists "housing floor" and
  "housing ring". cycloid.py makes his file exactly (0.000 mm³ either way).
  - Floor: the ring pin sockets are Ø3.4 (`FLOOR_PIN_SOCKET`, his: 0.2 over the Ø3.2 grooves, so the ring places the
    pins). The lid screws self-tap its Ø2.5 holes: 5.9 mm of thread.
  - Ring: the lid screw holes are Ø3.4 clearance, then Ø3.0 (`LID_FIT_HOLE`) on the plain shank from z 19.4 to the
    top. His words: "The bolts should line up the ring and the base."
  - 0.6 × 45° chamfers (`JOINT_CHAMFER`) on the lid screw holes, both faces of the joint; 0.5 on the arm pilots.
  - `housing(printed_pins)` is still there: the two pieces fused, for the stand-in, the animation and the pin gauge.
  (Robo Chess gearbox)
- Motor pilot recess under the housing: Ø22.2 × 3.2 deep (`PILOT_DEPTH`) for the 22 × 2 boss, 4.0 of floor above
  it. It was 2.2 deep. The Professor, 2026-11-09, on his printed housing: "That's a little shallow. At least on this
  one motor ... I'd move it a millimeter." (Robo Chess gearbox)

**Hardware per joint**
- 46 dowels, 3×20, hardened ISO 8734 m6 (52100 steel).
- 2 × 6800 and 2 × 6808.
- 1 × 8×14×4 flanged bearing (15.6 × 0.8 flange, MF148 / F678 size) on the eccentric's top, in the output flange.
- Lid screws (Professor 2026-11-06): 6 × M3×55 socket head cap screws, partly threaded (18 thread, ISO 4762).
  - He chose 55 because M3×58 is special order.
  - Shank through lid + pin ring + a Ø3.4 clearance in the housing top down to z 18.0.
  - 17.3 mm of thread self-taps the Ø2.5 hole. Tip at z 0.7.
- **CONFIRMED and ORDERED by the Professor (2026-11-06):** M3×55. "Keep the bolts the size that they are ... It's all
  confirmed already." The arm-mount screws from below used to share these 6 holes; since 2026-11-15 they have
  their own (next bullet).
- Arm-mount screws (Mike 2026-11-15): "Six additional holes to bolt the arm segments onto. Same bolt diameter, for a
  number 4 by 3/4 inch long wood screw", and "We have to be careful with how much we displace material".
  - 6 × #4 × 3/4" wood screws come up through the arm plate into Ø2.4 pilots in the housing floor, z 0 to 16, with a
    0.5 × 45° entry chamfer. Any plate from 3 to 8 mm leaves the screw tip in the pilot.
  - On the lid circle, r 32.4, at 33.16 / 90 / 146.84 / 213.16 / 270 / 326.84°. A straight 30° turn off the lid
    screws put two holes right behind a ring pin (60° is 6 1/3 pin pitches), so each sits midway between two pins.
    The pattern repeats every 180°, not every 60°.
  - Walls: 1.65 to the outside, 2.26 to the pin grooves (1.60 at the worst two with the straight turn).
  - `ARM_SCREW_R` / `ARM_SCREW_ANGLES` / `ARM_SCREW_PILOT` / `ARM_PILOT_TOP` in cycloid.py; the arm session's
    mockup_link.py reads them, gearbox_fit_check.py checks the arm and the shoulder column against them.
  - Also in his two-piece housing (`print\hosuing 2 piece .step`), with 0.5 chamfers on both faces of the joint.
  (Robo Chess gearbox)
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
- D-flat grip in the one-piece eccentric:
  - 24 mm shaft (17HS19): 13.25 mm. The two-piece had 6.0 in the lower half.
  - 20 mm wrist pancake: 9.25 mm, no key needed.

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
- The stock `cycloid\` folder (no spread) has not been rebuilt since before the one-piece eccentric.
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

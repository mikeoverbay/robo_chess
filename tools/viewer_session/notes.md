## Robot Studio: layout, measurements, mouse

- **Parts** list is in its own panel on the right. Selecting a part or face in 3D lights up its row and scrolls to it.
- The **Dimensions** button (top-left toolbar) turns the live measurements on and off. All values are in mm:
  - Joint centre to joint centre along each link (200.0).
  - Width of each link (w 70.5).
  - Overall height, from the table to the highest point (H).
  - Overall width, along the arm's direction (W).
  - All of them update as the arm moves.
- The **board and pieces ignore the mouse**: no hover highlight and no selecting. The arm and claw stay pickable and draggable. (scene.json entries with "pick": false)
- **Dummy piece** in the claw: hidden while the Demo runs, back afterward.
- The board now has proper colours: a1 dark, h1 light, walnut frame.

## Pieces on the board, and the Demo

**Pieces:** your STL pieces on the gripper pedestals, in the starting position. Black is on the robot's side, white at your end with h1 on your right. Knights face the other side.
- `pedestal.py` is back in `tools\`, and the two pedestal STEPs are in `chessboard\pedestal\`. Both came from the Recycle Bin.
- Viewer-only meshes live in `tools\viewer_session\`, written by `tools\board_pieces.py`.

**▶ Demo** (Joints panel) plays every move in scene.json "moves", in order. First one: **Knight g1-f3**.
- Each move: straight up to carry height, across at 125 mm above the board, down, grip, lift, carry in a straight line, set down, let go, lift off. After the last move it returns home.
- The knight rides in the claw and turns with the swivel. Every frame goes through the IK: claw vertical, J2 = J3.
- **Reset pieces** puts them back.
- The claw's fingers don't open or close, because the claw is a fixed model.

**Home** is now your park pose: J0 90, J1 135, J2 −90, J3 −90, J4 90. It's swung clear of the board, folded back and up, with the claw curled in and about 160 mm up. The Demo starts at home, moves the knight, and returns to home.

## Swivel cover (split) and base flange

**Flange:** 8 mm thick now, so the cover screws can go into its edge.
- Its back edge is flush with the motor upright.
- The 4 mounting holes are 14 mm in from both edges at each corner, inside the cover. Screw the base down first.

**Cover:** split on the centre line into left and right halves (P12, P13), 4 mm walls.
- A rabbet at the bottom: the cover sits on the flange top, and a 3 mm lip wraps the flange edge, flush with the base bottom.
- 10 horizontal #4 × ½" pan heads go through the lip into the flange edge: per half, 3 on the side, 1 at the front, 1 at the back.
- The roof's top is at 89.7 mm, level with the turntable, with a round opening 1.6 mm clear of the turntable/gear.
- The back opening is the motor upright's outline (both sides and the top, 0.5 mm clear). The roof bridges over it, and the motor and cables pass through.
- Two pads with deep counterbores reach down to the side walls' #4 pilots: #4 × ¾", 15 mm in the wall.
- Front: no lobes. A 20 mm wide rib runs from the bearing tower to just inside the cover's front wall, 0.2 mm clear, from the flange up to 66 mm. One #4 goes through the front wall into the rib near the top, 5 mm either side of the centre line.
- The bottom front flange screws are now 10 mm off centre.

Checked: no contact with the housing, motor, pinion or turntable (at 0, 20 and 40°). Each half is 159 × 65 × 90 mm and prints roof-down.

## Swivel: round turntable and foot, split housing

**Round:** the turntable plate and the column's foot are both 108.7 mm round, the gear's outside diameter. The gear, plate and spindle print as one part.
- The foot screws to it with 4 × #8 on a 45 mm radius, one at the middle of each column side, about 5 mm clear of the rim and of the column walls.
- The pilot holes run 15 mm deep into the gear body, so use #8 × 1".

**Housing:** one part again (you merged the halves).
- Side walls are 10 mm thick.
- Each wall top has a straight #4 pilot, 2.4 mm × 20 deep with no chamfer, 62 mm back from the axis so a screwdriver clears the gear. Your split cover screws on there.

## Drag the claw (IK)

Press on the piece or the claw and drag: the arm follows. No checkbox needed, and a click without moving still selects faces.
**Show grab point** adds a green ball at the piece's base with a dashed line down to the board.
- **Drag** moves it across the board at its current height.
- **Shift + drag** moves it straight up or down. It never goes below the board top.
- The readout shows how far out and how high it is. The ball turns **red** when the spot is out of reach or a joint is at its limit.

How it solves:
- **J0** points the arm at the piece.
- **J2 and J3 share the bend equally.** Three equal links with equal bends q reach d = 200 (1 + 2 cos q), so there's a direct solution with no iterating.
- **J4** keeps the claw vertical: J4 = 45 − (J1 + J2 + J3).
- **Too close for equal bends:** J3 stays at −104° and J1/J2 solve the two-link case.

Checked: 0.00 mm error from 155 to 609 mm out, on the board and lifted, straight out and off to the side.

Not yet: it doesn't check that the links clear the pieces.

Also: the side panel now scrolls, so Parts is reachable. The cable passage under the swivel motor plate is now 22 mm high (it was 6).

## Swivel base (J0): two 6808s and a 4:1 bevel pair

Built by `tools\swivel_base.py`; parts in `complete assembly\swivel_fixed.step` and `swivel_turntable.step`.

**Bevel gears:**
- 18:72 straight bevel, module 1.5, 20°, 14 mm face, 0.15 mm backlash per gear.
- 4:1 gives 0.028° per microstep at 1/16, which is 0.3 mm at the far corner.
- Mesh checked: no overlap through a full tooth. Free play is ±0.5° at the pinion, about 0.3° at the turntable, roughly 3 mm at 600 mm.

**Housing (screwed to the wood, 4 × #8):**
- Tower with both 6808s, centres 43 mm apart. The outer races stop on a middle shoulder.
- NEMA 17 face plate at the back, with the motor straight out the back.

**Turntable:**
- Square plate, 12 mm thick; the arm's foot screws on with 4 × #8 into 3.5 mm pilots.
- 72T gear under the plate, teeth down.
- Hollow spindle, Ø40 outside, Ø30 inside. A collar under the lower bearing takes 3 × #4 screws.

**Wires:** down the column channel, through the foot (Ø20), down the spindle, out a slot at the bottom of the tower, then under the flange to the back. The swivel range is set to ±170° so the bundle only twists.

**Height:** J1 is now 257 mm above the wood, 232 above the board top. Reach is 152 to 640 mm, so the board moved to 122 mm from the axis. Nearest squares are 155 mm out, far corners 609.

**Printing (all fit 220 × 220):**
- Turntable: plate down, teeth up.
- Housing: flange down.
- Pinion: big end down.

## Base and board mark-up

**Base (P5–P7)** `complete assemblyrm_base.step`, built by `toolsrm_base.py`:
- One link's drive end, cut 160 mm below J1 and stood on end. The drive mount, wall ends, 6808 pocket and cap hole are the link's own, so J1 behaves like J2/J3: 90 ± 104° about straight up, slider range −14..194.
- A 100 × 100 × 8 foot is fused to the cut. Four #8 wood screw holes 8 mm in from the edges, countersunk on top.
- Drive J1 is turned 90° with the link so its lid screws meet the mount holes.
- J1 sits 168 mm above the wood, 143 above the board top.
- Prints standing on its foot: 100 × 100 footprint, 203 mm tall.

**Board (P8)** `chessboardoard_layout.step`:
- 60 mm squares, 15 mm border, 25 mm thick, on the same wood. The playing edge is 100 mm in front of the base axis.
- Reach at this height is 127 to 617 mm (set down and lifted 125). Nearest squares 134, far corners a1/h1 589.

**Poses:** home, nearest square, rank 1, rank 1 lifted, far corner. The viewer has no swivel, so "far corner" is shown straight out.

**Bed check (220 × 220):** link body 212 × 212 turned 45° (fits, ~4 mm per corner; no skirt or brim), link top 173 × 173, claw adapter 105 × 106.

## Reach: saved poses, closest and farthest

The Joints panel now has buttons for the saved poses **home**, **closest** and **farthest**. They are saved in scene.json under "poses".

Farthest piece center from the J1 axis, with the claw vertical:

| Shoulder above board | Farthest | Pose (J1, J2, J3, J4) |
|---|---|---|
| 100 | 639 mm | 10.4, 1.3, -1.7, 35.0 |
| 150 | 646 mm | 5.9, 2.3, -4.7, 41.5 |
| 200 | 649 mm | 1.2, 2.3, -5.1, 46.5 |
| 250 | 648 mm | -3.1, 1.1, -4.0, 51.0 |
| 300 | 642 mm | -6.3, 0.2, -6.9, 58.0 |

The arm is almost straight here: 600 mm of links plus the claw's 49.5 mm offset. This is the worst pose for gravity torque and a singular one, so plan on a few cm less in use.

# Home (park) pose

**Home** in Joints now goes to each joint's home angle instead of 0:
- **J1:** 90°, upper arm straight up.
- **J2:** −90°, forearm level, reaching out.
- **J3:** −90°, link C pointing down.
- **J4:** 135°, claw straight down.

**0** next to a joint still zeroes just that joint. To use a different home pose, set the sliders and tell me.

## Where the magnets and Hall switches could go (nothing modeled yet)
- **The spot:** at every joint, the ring under the cap of the link that carries the drive (fixed) sits face to face with the cup of the next link (turning) at z 53.2–53.7. The gap is only 0.5 mm, at a radius of about 26–35 mm.
- **The parts:** a magnet in a pocket in the cup's top face, and the Hall switch in a pocket in the ring's underside, at the same radius. They line up exactly at the home angle, and the wires leave through a groove to the outside of the ring.
- **No gearbox changes:** both sides are printed link parts.
- **J4 (wrist):** the claw adapter has the same cup, under link C's cap.
- **J1 (shoulder):** this needs the base, which isn't designed yet.

---

# Stray faces cleaned up

- **Mine:** the faces you picked (P12.F7, F9, F11) were my mistake: a 0.1 mm slit and steps left by my first corner patch.
- **From the original export:** zero-width slivers at the front corners (the z 62.7 step), and two 0.1 mm kinks where the top's end face turns into its angled corners.
- **The fix:** instead of patching by hand, the build now runs OpenCascade's small-face healing on your original body and top. It removes faces narrower than 0.12 mm and merges what's left. The only change is 8 mm³ at those kinks.
- **Checked:** no face under 1 mm² is left anywhere on the body or top, both solids are valid, and the joints still swing ±104°.

---

# Screws: #4 × ¾" flat head everywhere on the links

- **Drive mount holes** (6 per link, the one you selected, under each drive): 82° countersinks on the underside, so a **#4 × ¾"** flat head sits flush.
  - From the plate bottom it reaches z 15.05. The drive's own lid screws come down the same holes and end at z 16.7, so they miss by 1.6 mm.
  - The ¾" screw gets 15 mm of thread in the housing's 2.5 mm hole.
- **Top screws** (8 per link): also #4 × ¾", 15 mm of thread in the nubs.
- **Count:** 14 per link, **42 for the three links**. You already have the ¾" ones, so nothing new to buy for these.
- **Corner bevels cleaned up:** the 45° bevels on the box's front corners are one clean face now, top to bottom. The zero-width step at z 62.7 from the original export is gone, with no slivers left.

---

# Links slimmed: only as thick as the joint needs

- **Thickness:** each link is now **69.2 mm thick (was 80.5)**. The top is cut down to z 65.2 and the bottom up to z −4, so both outside plates are a flat **4 mm**, including the drive plate and the bearing cap. The pockets are gone; they aren't needed now.
- **The joint stack doesn't move:** the drive still sits on z 0, and the 6808 still sits under the cap at 61.2.
- **Screws:**
  - The drive mount screws go through only 4 mm of plate now, so use **M3 × 10** (was × 16).
  - Your **#4 × ¾"** top screws still work; I deepened the pilots to 17.5 mm, which gives 15 mm of thread.
- **Weight (chevron link):** body 159 g solid / about 110 g printed; top 56 g / about 40 g. That's about 150 g printed per link, down from about 172 g.
- **Checks:** joints still swing ±104°; wrist −33° to +180°; nothing overlaps.

---

# (older) Top and floor both 4 mm

- **Top (P4.F5 side):** pocketed from the inside to 4 mm between the walls, with 45° ends. It keeps 8 mm over the walls, around the screws, and at the cap that holds the bearing.
- **Floor side (P3.F86):** the thick band in front of the box is now 4 mm too, right up to a ring 1 mm outside the drive's seat. The seat under the drive stays full thickness, because the drive sits on it at z 0. Lowering it would drop the whole joint and the next link's boss would miss the bearing.
- **Weights (chevron link):** body 184 g solid / about 122 g printed; top 82 g / about 50 g.
- **Checks:** the bottom is still one flat face, and the joints still swing ±104°.

---

# Floor: bottom back flush

- **Bottom:** the underside of every link is one flat face again at z −10.8, flush with the drive plate, so it prints flat with no support.
- **Floor:** still 4 mm. The thinning is now a pocket from the inside, between the walls (top at z −6.8), with 45° ends.
- **Chevron link:** body 190 g solid, about 124 g printed.

---

# Claw adapter: your changes

- **Face turned 45°** about its center. The Bowden line now passes 49.5 mm beside the wrist axis instead of into the drive. I turned it the way that puts the claw vertical at **J4 = +45°** (link C level), on the side with no stop. The other way, vertical would have been past the stop at link C.
- **No channel.** The part is solid, with:
  - **one Ø16 bore** straight along the Bowden line, through the face plate and out the top. It's one size the whole way (no shoulder to catch the cable), with room for the PC4-M6 fitting.
  - **two M4 holes** (Ø4.5 through the face plate) with **deep Ø8.5 counterbores** out the top, so a long hex key reaches the heads. They screw into the claw cap; tap the cap's two holes to 3.3, or use nuts inside it.
- **Checked:** the bore and both counterbores are open all the way, no overlaps, and the wrist range is **−33° to +180°**. The claw stays vertical with link C anywhere from 78° above level downward.
- **Shown:** the view is cut through the Bowden bore.

---

# Claw on the wrist (J4)

New joint **J4 wrist** with the claw. Shown reaching down: J1 60°, J2 −90°, J3 −30°, J4 60°, claw straight down.

## The adapter: `gripper\claw_adapter.step` (printed part, about 180 g solid)
- **Output connection, same as the arm links:** cup on J4's output with 3 × M3 × 20, the 43 riser, and the 40 boss into the 6808 in link C's cap.
- **Arc round the drive:** 330°, 73 bore, 4 mm wall. It sits above link C's plate and under its cap. The 30° opening is on the side away from the claw.
- **Projection out to a flat face, 56 wide:** the claw cap's two M4 screws go into it (tapped 3.3, 12 deep).
- **Window behind the face:** the cap's boss and the PC4-M6 Bowden fitting sit in it, and the tube can bend out sideways instead of running into the drive.
- **Claw position:** the claw hangs square to link C; with C level and J4 at 0 it points straight down. Its tip is 232 mm from the wrist axis.
- **Both together:** `gripper\claw_on_wrist.step` is the adapter with the claw on it (closed, holding a piece).

## Checked
- No overlap with link C, the J4 drive or the 6808.
- **J4 range:** +180° away from link C (nothing in the way), but only **−25°** folding back toward it. At −25° the adapter's corner reaches the open end of link C's side walls.
- That means the claw can only stay vertical while link C points no more than 25° above level. Rounding the adapter's corners or narrowing its root would buy more if you need it.

---

# Arm stood up: it now moves in the X-Z plane

- **Orientation:** the whole arm is turned 90° about X, so the joint axes run along world Y and the arm swings up and down in the X-Z plane, like a standard robot arm. The motors stick out the side. Section X, Y and Z are world directions now, so Z is up.
- **Sign:** a positive joint angle lifts.
- **Shown pose:** J1 60° (upper arm up), J2 −100° (forearm down), J3 −20°. **Home** lays it out straight.
- **Files and numbers unchanged:** the STEP files stay in their design frame (joint axes on z), and so do the coordinates in your selection and in my notes. Only the view is turned.

---

# Chevron chosen

All three links (A, B, C) are now the chevron version, `arm_2_part_chevron.step`: two rows of 45° slots on the side walls, the top and the floor. The joint ranges don't change (±104°), because the ends of the links are the same in both versions.

The heat-shield file `arm_2_part_shield.step` is still in `complete assembly`, in case you want it for comparison; say if you'd like it deleted.

---

# Joints and materials

## Joints (new section above Parts)
- **J1 shoulder** turns link A, the elbow drive bolted to A, and the 6808 on A's boss.
- **J2 elbow** rides on J1 and turns link B, B's far drive, and the 6808s at both ends of B.
- **Moving them:** drag the sliders; **0** puts one joint back, **Home** puts both back.
- **Labels and cuts:** selection labels ride along with their parts, and section cuts still work while bent.
- **J2 limit, ±104°:** that's what I measured. Past 104.4° (or −104.2°), the outside corner of B's box at its pivot end (about 40 mm from the elbow axis) hits the open front ends of A's side walls. Rounding those corners would buy more.
- **J1 limit, ±150°:** a placeholder until the base is designed.

## Materials (the menu at the end of each row in Parts)
- **printed:** the part's color, slight blotchy variation, surface grain, and 0.2 mm layer lines that show up as you zoom in. The default for links and drives.
- **steel:** brushed and reflective, for screws.
- **polished:** mirror-like bearing steel. The default for the 6808s; their black seals stay black.
- **rust:** generated rust patches over steel. Rough, dull, bumped rust; shiny bare steel between.
- **flat:** the old plain look.

Your choice for each part is remembered in your browser.

---

# Top 6808 at each joint: pocket in the top, step gone

Your 6808 from `arm2.step` is in at all three joints. Your 6808 from `arm2.step` (black seals and all) is at all three joints: P8–P10, labeled **6808**.

- **Step removed** (orange, P3.F3): the 52 mm, 0.5 mm step round the boss base is gone. The 43 riser now runs from the cup top to the inner race (1 mm tall), and the outer race clears the cup by 1 mm.
- **Bearing pocket in the blue top** (pink): a ring under the cap, 70.5 OD like the cap, with a **52.2 pocket**, z 53.7–61.2. That's a 0.1 mm slip fit all round, so the bearing settles where the drive puts it. Its bottom stays 0.5 mm above the next link's cup.
- **Cap hole opened from 40 to 46**: the cap now presses only on the outer race. The inner race (42.5 OD) turns with the other link and would have rubbed.
- **Checked:** no overlaps between the bearing, the pocket, the boss and the drive, and the next link still swings the full ±90° past the new ring.
- **Where:** both options got it, because it went into the base build (`arm_2_part_nubs.step`), which both are made from.

For the record: **6808** = 40 × 52 × 7 (this one, and the two on each drive's output); **6800** = 10 × 19 × 5 (the small pair on the eccentric inside each drive).

---

# Cycloid drive stand-in: added

`cycloid\cycloid_drive_standin.step`, also in `complete assembly\cycloid gearbox\`. One solid, shown at all three joints: x 0, −200 and −400 (P5–P7, "cycloid drive" in Parts).

- **What it is:** the outside of the real drive, built from the gearbox script's own parts, so every surface and hole is where the real one is. No gears, pins or bearings.
- **Holes:**
  - 6 lid screw holes on r 32.4; your mount screws come up into these from the link plate.
  - 3 output insert holes on r 15.
- **Output face:** the clamp ring top at z 45.2, where a link's cup sits.
- **Motor:** 42.3 square × 48 with chamfered corners like a real NEMA 17 (53.4 mm across the chamfers).
- **Fit:** no overlap with either link at any of the three joints (checked).

**Pictures:** the two big buttons at the top left of the view. **Copy picture** (or Ctrl+C) puts the view on the clipboard, labels included; **Save picture** saves a PNG. I get a copy of every picture you take.

---

# Two options: diagonal stripes + 4 mm floor

Shown end to end, 200 mm joint to joint like the arm. **A** (P1 body, P2 top) is in its real place, turning on the joint at x 0. **B** (P3 body, P4 top) is moved −200 in x, so its pivot sits on A's elbow. They nest with no overlap (checked). The move is only in the viewer; the B file is in its normal place. The files are `arm_2_part_shield.step` and `arm_2_part_chevron.step`, both built from the version with the screw nubs.

## Both options
- **Floor at 4 mm:** the bottom is raised from z −10.8 to −4 between the walls, with 45° steps at both ends. Turn the model over to see it.
- **Not thinned:** the plate under the elbow drive stays 10.8 mm. The 6 mount screws are sized for it and it holds the drive square. If you want it thinner too, the screws become M3 × 10.
- **Where the stripes go:** through the floor, the top and both walls, over the same span, so they line up all round.
- **What they avoid:** the screw nubs, the countersinks and the steps.

## A: heat shield
- One row of long slots leaning 60°, 6 mm wide with 5 mm between, rounded ends.
- 6 per wall, 5 in the floor and 5 in the top.

## B: chevron
- Two rows of shorter 45° slots leaning opposite ways, 5 mm wide, so they make arrows pointing along the arm.
- 6 + 6 per wall, floor and top.
- The rib between the rows makes B a bit stiffer than A.

## Weight (nylon; solid / printed)
- **Now, with nubs:** body 256 / 135 g, top 101 / 51 g
- **A heat shield:** body 182 / 117 g, top 88 / 49 g
- **B chevron:** body 184 / 119 g, top 87 / 51 g

"Printed" means 3 walls and 20 % infill. At those settings most of the saving (about 18 g) comes from the floor. Every slot edge prints as solid wall, so the slots are mostly looks; they save more at higher infill. Solid weight drops about 85 g either way.

## Printing
- With the floor raised, the box bottom sits 6.8 mm above the bed between the drive plate and the pivot end, so it needs support under it.
- If you'd rather print with no support, I can thin the floor from the inside instead: same 4 mm, flat bottom.
- The slots print fine, since 60° and 45° need no support.

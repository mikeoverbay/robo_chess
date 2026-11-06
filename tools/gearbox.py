#!/usr/bin/env python3
"""
gearbox.py - printable planetary gearbox for a NEMA 17, written out as STL files.

  python gearbox.py [--sun 12 --planet 18 --module 1.0 --planets 3]
                    [--bearing-bore 13.10] [--cage-seat 39.9] [--cage-pocket 52.1]
                    [--sun-bore 5.15] [--flat-start 5] [--screw-length 0=auto]
                    [--shaft-length 24] [--round-shaft] [--out DIR]

Needs planetary.py beside it and:  python -m pip install manifold3d trimesh numpy networkx pillow

The carrier cage (spider + planets + carrier) runs on two 6808-2RS bearings (40 x 52 x 7): one in
the bottom of the body round the spider, one at the top of the body round the carrier, seated 2 mm
down into the body so the body alone lines the cage up with the ring gear; the lid only holds it.  The body is a plain
70.5 mm cylinder with three 2.5 mm holes straight through it; short M3 screws thread into those
holes from the motor plate below and from the lid above.  No long bolts, no nuts.

Printed parts.  Each STL is written in its print orientation.  Tuned for 0.2 mm layers.
Slice with Generate Support OFF: nothing needs it, and support would fill the recesses.
Turn on Experimental > Enable Bridge Settings: the spider's screw-head seats are bridges.  The
planets' 1 mm ledges print as plain short overhangs.
  motor_plate   bolts to the motor; the motor's pilot boss sits in its centre hole, motor screw
                heads counterbored below its top face.  Three body screws go up through it, heads
                counterbored flush into its underside (stepped bridges over them); three 2.5 mm
                holes between them take base screws from below
  housing       the ring gear in a plain 70.5 mm cylinder with a 92 mm mounting flange at its motor
                end (four M3 clearance holes on a 82 mm circle: it sits on the tower top, plate and
                motor hanging through a 72 mm hole), three 2.5 mm screw holes through it, a 7.2 mm
                pocket for the lower cage bearing at its motor end and a 2 mm pocket for the upper
                one at its lid end.  Printed motor end down: the upper race's seat is
                then a top face, the lower pocket's lip is cut as 0.2 mm steps and the teeth grow
                out of the lip bore on a 45 deg cone, so nothing bridges
  spider        bottom carrier disc on the lower cage bearing: a 39.9 mm seat for the inner race,
                a 42 mm shoulder above it, M4 planet screws counterbored into its underside, and
                on its top face the three pinch islands (7.2 mm, left standing in 14 x 0.4
                recesses) that press on the planet bearings' inner races, each with a 6 mm pocket
                3 deep for the axle tube.  Printed as modelled, seat face down (a 0.4 lead-in on
                the seat's bed edge), the head seats bridged, islands on top
  carrier       top carrier = the output (the drive disc).  Same islands and tube pockets on its
                underside; the planet screws thread into it below the pockets.  Above them a 42 mm
                shoulder, the 39.9 mm seat for the upper cage bearing, then the 39.5 mm boss the
                next arm bolts to.  Printed boss face down, so the islands are on top
  lid           covers the 5 mm of the upper cage bearing that stands out of the body, in a
                pocket 0.2 mm larger than the body's so it never fights the body's location;
                three lid screws go down through it into the body.  A 6.3 mm pocket in its top
                face takes the 6 x 1.5 homing magnet, glued in 2 mm below the surface.  Written
                pocket up (top face on the bed)
  arm_hub       the first link's hub: a 70.5 mm disc, 10 thick, that bolts to the carrier boss with
                three M3 and carries the arm (a 2020 extrusion across its top on two M5 T-nut
                bolts).  A 42 / 39.6 ring on its underside drops over the boss onto the upper
                bearing's inner race, so the hub itself holds the cage down (no loose retainer).
                A pocket in its underside takes the DRV5032 breakout, sensor face down, 0.3 mm
                inside the surface, with a wire channel to the rim; it passes 1 mm over the lid
                screw heads.  Printed top face down
  sun           on the motor shaft, held by the fit of its bore (no set screw): a D hole straight
                through, open at both ends.  The shaft's flat must reach down to the sun's bottom
                face: measure where your motor's flat starts and give it as --flat-start
  planet x3     a plain 686ZZ pressed into a 13 mm counterbore 4.5 deep in each face, down onto a
                1 mm ledge (11 mm through bore); each bearing stands 0.5 mm proud of its gear face,
                which is the gear's running clearance to the plates
  planet_spacer x3  a 7.2 / 6.1 ring, 1.1 long, dropped in between the two inner races before the
                second bearing goes in: the pinch runs through it, not through the balls (or cut
                it from 7 mm x 0.45 brass tube, bore 6.1)
  bearing_fit_test  five rings with the planet's wall section, bores 13.00..13.20; print it first and
                pick the bore that needs a firm press (the planet rim relies on it)
  cage_fit_test one 52.1 pocket and one 39.9 seat for the 6808s: the outer race should push into
                the pocket by hand and stay, the inner race should need a light press onto the
                seat.  Adjust with --cage-pocket and --cage-seat
  sun_fit_test  four short D bores, 5.00..5.15; pick the one that slides on by hand with no rock

Bought parts (default build; the report prints the exact list for any options):
   2 x ball bearing 6808-2RS, 40 x 52 x 7: the cage bearings
   6 x ball bearing 686ZZ: 6 bore, 13 OD, 5 wide (plain, no flange): the planet bearings
   3 x axle tube, 6 mm OD, bore 4.2 or more, cut to the length the report gives (16.5): brass
       (K&S 6 x 0.45), aluminium 6 x 4 or steel; the races and islands centre on it
   3 x M4x30 socket head cap screw: through the tubes, they clamp the planet stacks
   4 x M3x6 socket or button head: motor to plate
   3 x M3x12 socket head: plate to body, from below
   3 x M3x16 socket or button head: lid to body, from above
   3 x M3x12 socket head: arm hub to the carrier boss (blind holes 7 mm deep in the boss)
   2 x M5x10 + 2020 T-nuts: arm extrusion to the hub
   4 x M3 + nuts or T-nuts: body flange to the tower top
   1 x disc magnet 6 x 1.5 in the lid; 1 x DRV5032 on a SOT-23 breakout in the hub

How the cage runs.  Each 6808's inner race is pressed onto a 39.9 seat and sits against a 42 mm
shoulder (the bearing's own abutment diameter), its outer race pushes into a 52.1 pocket whose lip
is 50 mm (the housing abutment diameter), so nothing touches the seals.  The lower outer race sits
on the motor plate under the housing lip; the upper one sits on the floor of the body's top pocket,
2 mm down, under the lid lip; each has 0.2 mm of axial play.  The cage therefore floats about 0.2 mm, and tilts only within
the bearings' own clearance.

How the planets run.  The two inner races of each planet and the spacer between them sit on a 6 mm
axle tube as one stack, pinched between the island on the spider and the island on the carrier.  The
tube runs 3 mm into a drilled pocket in each island, so islands and races centre on the same tube
(0.045 mm worst offset); the M4 screw down the tube's bore only pulls the plates together, and the
tube stops 0.3 mm short of both pocket floors so it never takes the clamp.  A 686 inner race's face
runs from its bore chamfer (6.6 mm at the worst) out to its land at 7.33 mm (NMB L-1360ZZ), where the
shield begins, so the islands and the spacer are 7.2 mm across: they touch the inner races and nothing
else.  The 14 mm recess round each island clears the outer race and its shield by 0.4 mm.  The spacer
is 0.1 longer than the gear's ledge, so the clamp closes on the spacer and the balls see none of it.
The screws only need to stop the inner races turning: snug them, about 0.1 N*m, and no more, or the
islands creep.

Assembly
  1  Run a 6.0 mm drill into the six tube pockets, 3 mm deep in from the island faces (spider and
     carrier), so a tube slides in without play.  Drill the three carrier holes 3.3 from the pocket
     floors and tap them M4; do not drive screws into the printed holes.  Run a 2.5 mm drill
     through the three body holes if they printed tight.  Stand a planet bearing on each island and
     spin it: it must not touch the recess wall.
  2  Press one 686 into one face of each planet, pushing on the outer ring, until it sits on the
     ledge.  Drop a spacer in from the other face (it passes the 11 mm bore and lands on the inner
     race), then press the second bearing in until it sits on the ledge.  Each bearing stands
     0.5 mm proud of the gear; that is the gear's clearance to the plates.  Push a tube through both
     inner races and the spacer: it centres the spacer.
  3  Press a 6808 inner race onto the spider's seat, from the motor side, until it meets the
     shoulder; press the other onto the carrier's seat, from the boss end, until it meets the
     carrier shoulder.  Push on the inner race only.
  4  Spider counterbore side down on the bench.  Push the three M4 screws up through it so they
     stand on their heads, then lower a planet onto each screw so its tube drops into the spider's
     pocket.  Lower the carrier onto the screw tips, islands down, its pockets over the tube tops.
     Hold spider and carrier together, turn the pair over and drive the screws, only snug: about
     0.1 N*m.  Each planet should spin freely.  If one drags, back its screw off and look at the
     island for a strand or a burr reaching the shield; trim with a blade and re-snug.
  5  Motor plate onto the motor (four M3x6, heads in the top counterbores).  Sun onto the shaft
     (the D bore only fits one way) until its bottom face is the height the report gives above
     the plate's top face.  Choose --sun-bore with sun_fit_test.stl first.
  6  Body onto the plate, deep pocket down, and three M3 (length in the report) up from under
     the plate into the body.  Lower the cage in over the sun, turning the planets until all three
     drop into mesh; its lower bearing's outer race pushes into the bottom pocket and its upper
     one into the 2 mm top pocket.
  7  Lid over the carrier boss, its pocket over the upper bearing; three M3 (length in the
     report) down into the body.
     Glue the magnet into the lid's pocket.
  8  Arm hub over the boss, ring down, three M3x12 through its counterbores into the boss: snug,
     they also preload the cage.  Glue the sensor breakout into the hub's pocket, face down, and
     lead the wires out through the channel.  Then the 2020 arm across the hub on the two M5 T-nut
     bolts from below (one M3 head ends up under the extrusion, so the hub goes on first).

z = 0 is the motor face.  The script checks every pair of parts (printed and bought) for
intersection in the assembled position and with the cage at the top of its float, reports the
smallest gap between each kind of part, runs the design checks, and writes slices.png and
section.png.
"""
import argparse
import math
import os
import re
import sys

import numpy as np
import manifold3d as m3d
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import planetary as P  # noqa: E402

SEG = 128
m3d.set_circular_segments(SEG)

# ---------------------------------------------------------------- planet bearing: 686ZZ
BEARING_ID = 6.0
BEARING_OD = 13.0
BEARING_W = 5.0
PROTRUDE = 0.5           # each bearing stands this far out of its gear face = gear-to-plate clearance
LEDGE_BORE = 11.0        # through bore between the two counterbores: a 1 mm ledge each bearing bottoms on
LEDGE_T = 1.0            # ledge thickness
SPACER_OD = 7.2          # printed spacer between the inner races: on the band 6.6..7.33, clear of the shields
SPACER_ID = 6.1          # slides over the axle tube
SPACER_EXTRA = 0.1       # spacer longer than the ledge: the clamp closes on the spacer, not the balls
BEARING_W_TOL = 0.125    # each bearing may be up to this much narrow (+0/-0.125)
N_BEARINGS = 2           # per planet
INNER_LAND = 7.33        # inner ring OD at its face: NMB L-1360ZZ (= 686ZZ) reference inner diameter Li 7.33
INNER_CONTACT_MIN = 6.6  # r min 0.15, ISO 582 r max 0.3 radial: the bore chamfer can run out to 6 + 2 x 0.3
OUTER_LAND = 11.44       # outer ring ID at its face (NMB reference outer diameter Lo 11.44)
SHIELD_ID = 7.53         # check model: shield inner edge, assumed land + 0.2 (no maker publishes it) ...
SHIELD_RECESS = 0.0      # ... flush with the ring faces (no maker publishes a recess: assume none)

# ---------------------------------------------------------------- cage bearing: 6808-2RS
CAGE_D = 40.0
CAGE_OD = 52.0
CAGE_W = 7.0
CAGE_DA = 42.0           # shaft abutment diameter (NSK / SKF 61808: 42): the spider and carrier shoulders
CAGE_DA_H = 50.0         # housing abutment diameter (50): the housing and lid lips
CAGE_INNER_OD = 42.5     # check model: inner ring OD
CAGE_OUTER_ID = 49.5     # check model: outer ring ID
CAGE_SEAL_RECESS = 0.3   # check model: seal faces set back from the ring faces
CAGE_PLAY = 0.2          # axial play of each outer race in its pocket
CAGE_SEAT = 39.9         # printed inner-race seat (light press); --cage-seat
CAGE_POCKET = 52.1       # printed outer-race pocket (push fit); --cage-pocket
SHOULDER_H = 2.5         # the 42 mm shoulder above the spider seat (lifts the planets above the ring teeth's lead-in)

# ---------------------------------------------------------------- other bought parts
SCREWS = {               # ISO 4762 socket head cap screws
    "M3": dict(head_d=5.5, head_h=3.0, major=2.95, major_min=2.874, clear=3.4, tap=2.5, cb_d=6.6),
    "M4": dict(head_d=7.0, head_h=4.0, major=3.95, major_min=3.838, clear=4.5, tap=3.5, cb_d=7.6),
}
AXLE = SCREWS["M4"]      # planet screws
M3 = SCREWS["M3"]        # body, lid, motor and next-arm screws
SCREW_LENGTHS = (4, 5, 6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50)
SHAFT_D = 5.0
SHAFT_LEN = 24.0         # NEMA 17 shaft from the mounting face: 20 (Oriental) .. 24 (most), check yours
FLAT_START = 5.0         # --flat-start: where the shaft's D flat begins, mm from the motor face (5..10 on most NEMA 17s)
FLAT_MARGIN = 0.5        # the end of a milled flat ramps out; the sun's bottom stays this far above it
SHAFT_FLAT_DEPTH = 0.5

# ---------------------------------------------------------------- printed geometry (mm)
LAYER = 0.2              # everything axial lands on this grid
PLATE_T = 6.0            # motor plate: pilot pocket, motor heads sunk below the top, body-screw heads sunk below
GAP = 0.5                # axial running clearance between moving and fixed parts
RECESS = 0.4             # recess round each pinch island = outer race to plate clearance (bridged roof)
RECESS_D = 14.0          # recess diameter: clears the 13 outer ring by 0.5 radially less the worst float
CARRIER_WEB = 1.4        # solid plastic between the carrier's recess floor and its 6808 shoulder face
UPPER_IN = 2.0           # the upper 6808 sits this far down into the body: the body alone locates the cage
LID_POCKET_EXTRA = 0.2   # the lid pocket is this much larger than the body's, so it only covers the race
LIP_STEP = 0.3           # the lower lip is cut as 0.2 mm layers each stepping in this much (no bridge)
TOP_BAND = 1.0           # plain 50 bore under the upper race's seat, between the teeth and the top pocket
TOOTH_LEAD_H = 2.0       # the ring teeth grow out of the 50 bore on a 45 deg cone this tall (no tooth bottom hangs in air)
BOSS_OD = 7.2            # pinch island: on the 686 inner race band 6.6..7.33, clear of the shield even with float
TUBE_OD = 6.0            # planet axle tube: the races and the islands both centre on it
TUBE_OD_MIN = 5.98       # worst small tube
TUBE_ID_MIN = 4.2        # the M4 must pass
TUBE_HOLE = 6.0          # printed pocket in each island; drill it 6.0
TUBE_HOLE_MAX = 6.05     # the drilled pocket at its loosest
TUBE_POCKET = 3.0        # pocket depth below each island face
TUBE_END_GAP = 0.3       # the tube stops this short of each pocket floor: it never takes the clamp
SCREW_D = 4.0            # planet screw nominal diameter (clamp force)
SEAT_LEAD = 0.4          # 45 deg lead-in on the spider seat's bed edge (first-layer squish)
LIP_T = 1.6              # pocket lip thickness in the housing and the lid
LID_T = CAGE_W - UPPER_IN + CAGE_PLAY + LIP_T   # 6.8
BODY_OD = 70.5           # plain cylinder out to where the old ears were
SCREW_R = 30.75          # body screws on this radius
SCREW_ANGLES = (90.0, 210.0, 330.0)
BASE_ANGLES = (30.0, 150.0, 270.0)
BODY_HOLE = 2.5          # through the housing, M3 self-tap from both ends
HEAD_CLEAR = 1.0         # boss face at least this far above the lid screw heads
TIP_COVER = 0.8          # boss face at least this far above the planet screw tips
BOSS_D = 39.5            # carrier boss above the seat (slides through the bearing)
LINK_HOLE = 2.7          # M3, tap or self-tap
LINK_DEPTH = 7.0
LINK_PCD = 30.0
FLANGE_OD = 92.0         # mounting flange at the body's motor end
FLANGE_T = 4.0
FLANGE_R = 41.0          # its four M3 clearance holes
FLANGE_ANGLES = (45.0, 135.0, 225.0, 315.0)
HOME_MAGNET_D = 6.0      # homing magnet in the lid, 6 x 1.5, glued
HOME_MAGNET_T = 1.5
HOME_MAGNET_SINK = 2.0   # magnet face this far below the lid top: narrows the switch window
HOME_R = 30.5            # magnet and sensor radius: outside the lid's bearing pocket, inside the rim
HOME_ANGLE = 30.0        # magnet angle, between two lid screws
MAGNET_CLEAR = 0.3
HUB_T = 10.0             # arm hub thickness
HUB_BORE = 39.6          # over the 39.5 boss
HUB_RELIEF_ID = 40.4     # the ring's last 1 mm: clear of the inner race's bore chamfer
HUB_RELIEF_H = 1.0
SENSOR_ANGLE = 90.0      # sensor pocket in the hub frame (the arm runs along +x)
SENSOR_POCKET = (11.0, 9.0, 3.0)   # tangential x radial x deep, for a SOT-23 breakout face down
SENSOR_RECESS = 0.3      # sensor face inside the hub's underside
SENSOR_CHANNEL = (3.0, 2.0)        # wire channel width x depth, pocket to rim
ARM_BOLT_X = 26.0        # the two M5 T-nut bolts, on the arm axis
M5 = dict(head_d=8.5, head_h=5.0, clear=5.5, cb_d=9.0)
CENTRE_HOLE = 16.0       # the sun can rise into the carrier without touching it; the shaft end passes
COUNTERBORE_D = 6.2      # motor screw heads in the motor plate
MOTOR_HOLE = 3.4
MOTOR_HEAD_SINK = 0.6    # motor heads this far below the plate's top face
INNER_RELIEF = 0.4       # groove in the plate under the lower bearing's inner race and seal
RELIEF_OD = 46.0         # the groove reaches to the ball pitch circle; the outer race lands on 46..52.1
CB_RECESS = 0.4          # planet screw heads sit this far inside the spider underside
SOLID_WEB = 1.0          # full-section plastic between the screw head seat and the recess floor
MIN_ENGAGE = 8.0         # thread engagement wanted in the carrier (2 x d for M4)
BRIDGE_LAYER = 0.2
FOOT_RELIEF = 0.15       # planet: tooth profile pulled in this much ...
FOOT_H = 0.4             # ... over its first two layers
RIM_MIN = 1.0            # plastic between the bearing bore and the tooth roots: below this FAIL ...
RIM_RULE = 1.5           # ... below this WARN (planetary.py's printed-wall rule; steel-backed with a firm press)
PINCH_TORQUE = 0.1       # N*m on the planet screws
K_PETG = 0.3             # torque coefficient, steel screw in PETG
PINCH_STRESS_MAX = 25.0  # MPa on the island's worst contact band
SUN_ABOVE_PLANET = 0.6   # sun teeth run this far past the top of the planet gear
SUN_CHAMFER = 0.3        # lead-in at both ends of the sun bore

BEARING_BORE = 13.10     # printed planet bore for the press fit; tune with bearing_fit_test.stl
ENTRY_CHAMFER = 0.2      # 45 deg lead-in at each end of the bore (under the flanges)

# set from the command line in main()
ROUND_SHAFT = False
N_PLANETS = 3
SUN_BORE = None          # None: planetary.py's 5.15


def grid(v):
    return math.ceil(v / LAYER - 1e-9) * LAYER


def spacer_t():
    return LEDGE_T + SPACER_EXTRA


def stack_len():
    """Clamped length between the islands: two inner races and the spacer."""
    return N_BEARINGS * BEARING_W + spacer_t()


def cb_depth():
    """Counterbore depth in each gear face."""
    return BEARING_W - PROTRUDE


def planet_t():
    return 2 * cb_depth() + LEDGE_T


def lip_steps():
    """Number of 0.2 layers in the stepped lower lip (52.1 down to 50)."""
    return math.ceil((CAGE_POCKET - CAGE_DA_H) / 2 / LIP_STEP - 1e-9)


def spider_seat_h():
    """Seat height on the spider: the inner race's top is flush with the outer race's, so the shoulder
    sits GAP above the plate less the race width (both races on the plate's top face level)."""
    return CAGE_W - GAP


def spider_t():
    """Seat, shoulder, recess."""
    return grid(spider_seat_h() + SHOULDER_H + RECESS)


def solid_web():
    """Full-section plastic in the spider between the screw head seat and the tube pocket's floor."""
    return spider_t() - TUBE_POCKET - (AXLE["head_h"] + CB_RECESS)


def tube_len():
    """Axle tube: through the clamped stack and into both pockets, stopping TUBE_END_GAP short of each floor."""
    return stack_len() + 2 * (TUBE_POCKET - TUBE_END_GAP)


def link_angles():
    return tuple(180.0 / N_PLANETS + 360.0 * k / N_PLANETS for k in range(N_PLANETS))


def float_e():
    """Worst radial offset between an island and the inner race it presses on: both centre on the axle tube,
    so it is the tube loose in the island's drilled pocket plus the race loose on the tube."""
    return (TUBE_HOLE_MAX - TUBE_OD_MIN) / 2 + (BEARING_ID - TUBE_OD_MIN) / 2


def std_length(need):
    for L in SCREW_LENGTHS:
        if L >= need - 1e-9:
            return L
    return SCREW_LENGTHS[-1]


def auto_screw_length():
    """Shortest stocked M4 length that gives MIN_ENGAGE of thread in the carrier."""
    for L in SCREW_LENGTHS:
        if Stack(L).engage >= MIN_ENGAGE - 1e-9:
            return L
    return SCREW_LENGTHS[-1]


class Stack:
    def __init__(self, screw_len):
        self.plate0 = 0.0
        self.plate1 = PLATE_T
        self.b1 = self.plate1                               # lower cage bearing sits on the plate
        self.b1_top = self.b1 + CAGE_W
        self.housing0 = self.plate1
        self.lip1 = self.b1 + CAGE_W + CAGE_PLAY            # housing lip above the lower outer race
        self.teeth0 = self.lip1 + lip_steps() * BRIDGE_LAYER  # ring teeth start above the stepped lip
        self.teeth_full0 = self.teeth0 + TOOTH_LEAD_H       # full depth from here (45 deg lead-in cone below)
        self.spider0 = self.plate1 + GAP
        self.spider1 = self.spider0 + spider_t()            # spider pinch face
        self.shoulder0 = self.spider0 + spider_seat_h()     # spider shoulder = inner race top
        self.planet0 = self.spider1                         # bottom of the planet bearing stack
        self.planet1 = self.planet0 + stack_len()
        self.gear0 = self.planet0 + PROTRUDE                # gear faces sit inside the bearing faces
        self.gear1 = self.gear0 + planet_t()
        self.ledge0 = self.planet0 + BEARING_W              # the gear's ledge
        self.ledge1 = self.ledge0 + LEDGE_T
        self.carrier0 = self.planet1                        # carrier pinch face
        self.c_shoulder = self.carrier0 + RECESS + CARRIER_WEB   # carrier shoulder top = upper inner race bottom
        self.b2 = self.c_shoulder                           # upper cage bearing
        self.b2_top = self.b2 + CAGE_W
        self.housing1 = self.b2 + UPPER_IN                  # upper outer race sits UPPER_IN down in the body's top pocket
        self.teeth1 = self.b2 - TOP_BAND                    # a 50 bore band under the upper race: its flat seat
        self.lid0 = self.housing1
        self.lid1 = self.lid0 + LID_T
        self.lid_lip0 = self.b2_top + CAGE_PLAY
        # the D bore runs the whole sun, so the sun cannot sit lower than the shaft's flat allows
        self.sun0 = self.spider0 if ROUND_SHAFT else max(self.spider0, FLAT_START + FLAT_MARGIN)
        self.sun1 = self.sun0 + grid(self.gear1 + SUN_ABOVE_PLANET - self.sun0)
        self.sun_grip = min(SHAFT_LEN, self.sun1) - self.sun0   # shaft length inside the sun
        self.screw_len = screw_len
        self.head0 = self.spider0 + CB_RECESS
        self.underhead = self.head0 + AXLE["head_h"]
        self.tip = self.underhead + screw_len
        self.heads1 = self.lid1 + M3["head_h"]             # tops of the lid screw heads
        self.boss1 = self.carrier0 + grid(max(self.heads1 + HEAD_CLEAR, self.tip + TIP_COVER, self.b2_top + 2.0) - self.carrier0)
        self.thread0 = self.carrier0 + TUBE_POCKET          # thread counted from the tube pocket's floor up
        self.engage = self.tip - self.thread0
        self.retainer_h = self.boss1 - self.b2_top
        self.lid_screw = std_length(LID_T + 6.0)
        self.spacer0 = self.planet0 + BEARING_W
        self.plate_screw = std_length(PLATE_T - M3["head_h"] - 0.2 + 7.0)
        self.float_up = CAGE_PLAY
        self.short = N_BEARINGS * BEARING_W_TOL             # worst planet-bearing-stack shortfall


# ---------------------------------------------------------------- primitives
def cyl(d, h, z=0.0, x=0.0, y=0.0):
    return m3d.Manifold.cylinder(h, d / 2, -1.0, SEG).translate([x, y, z])


def ring(od, idia, h, z=0.0, x=0.0, y=0.0):
    return cyl(od, h, z, x, y) - cyl(idia, h + 2, z - 1, x, y)


def box(x0, x1, y0, y1, z0, z1):
    return m3d.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def extrude(polys, h, z=0.0):
    return m3d.CrossSection([[list(p) for p in poly] for poly in polys], m3d.FillRule.EvenOdd).extrude(h).translate([0, 0, z])


def circle_poly(r, n=SEG):
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def polar(r, angles):
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in angles]


def holes(dia, h, z, pts):
    out = None
    for x, y in pts:
        c = cyl(dia, h, z, x, y)
        out = c if out is None else out + c
    return out


def pin_xy(d, j):
    phi = 2 * math.pi * j / d['planets']
    return d['carrier_r'] * math.cos(phi), d['carrier_r'] * math.sin(phi)


def link_xy():
    return polar(LINK_PCD / 2, link_angles())


def chamfer_cone(d_small, depth, z, flip=False, x=0.0, y=0.0):
    """45 deg lead-in: diameter d_small + 2*depth at the open face, d_small `depth` inside.
    flip=False: the open face is at z and the cone goes up; flip=True: open face at z, cone goes down."""
    r0, r1 = d_small / 2 + depth + 0.01, d_small / 2
    if flip:
        return m3d.Manifold.cylinder(depth + 0.01, r1, r0, SEG).translate([x, y, z - depth])
    return m3d.Manifold.cylinder(depth + 0.01, r0, r1, SEG).translate([x, y, z - 0.01])


def stepped_bridge(hole_d, roof_d, z, x, y):
    """Cut for a hole that starts at the flat roof of a wider recess printed roof-over-air: a slot
    layer bridged wall to wall, then a square layer bridged across the slot; the round hole above."""
    s = BRIDGE_LAYER
    slot = box(x - roof_d / 2 - 0.1, x + roof_d / 2 + 0.1, y - hole_d / 2, y + hole_d / 2, z - 0.001, z + s + 0.001)
    square = box(x - hole_d / 2, x + hole_d / 2, y - hole_d / 2, y + hole_d / 2, z + s - 0.001, z + 2 * s + 0.001)
    return slot + square


def pinch_recess(z_face, x, y, up):
    """Recess cut into a plate face at z_face round the screw at (x, y), leaving the island.
    up=True: the face is the top of the part (cut downward); up=False: the face is its underside."""
    if up:
        return ring(RECESS_D, BOSS_OD, RECESS + 1, z_face - RECESS, x, y)
    return ring(RECESS_D, BOSS_OD, RECESS + 1, z_face - 1, x, y)


# ---------------------------------------------------------------- printed parts, each in its own frame
def motor_plate(d):
    body = cyl(BODY_OD, PLATE_T)
    body -= cyl(P.NEMA17_PILOT_D + 0.2, PLATE_T + 2, -1)                     # pilot boss locates in it; sun passes
    body -= chamfer_cone(P.NEMA17_PILOT_D + 0.2, 0.4, 0.0)                   # lead-in on the bed edge
    cb_top = PLATE_T - MOTOR_HEAD_SINK                                       # motor heads end here
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * P.NEMA17_BOLT_HALF, sy * P.NEMA17_BOLT_HALF
            body -= cyl(MOTOR_HOLE, PLATE_T + 2, -1, x, y)
            body -= cyl(COUNTERBORE_D, PLATE_T, cb_top - M3["head_h"], x, y)
    body -= ring(RELIEF_OD, CAGE_D - 1.0, INNER_RELIEF + 1, PLATE_T - INNER_RELIEF)   # groove under the inner race
    cb = M3["head_h"] + 0.2
    for x, y in polar(SCREW_R, SCREW_ANGLES):                                # body screws from below
        body -= cyl(M3["cb_d"], cb + 1, -1, x, y)
        body -= stepped_bridge(M3["clear"], M3["cb_d"], cb, x, y)
        body -= cyl(M3["clear"], PLATE_T, cb + 2 * BRIDGE_LAYER - 0.001, x, y)
    body -= holes(M3["tap"], PLATE_T + 2, -1, polar(SCREW_R, BASE_ANGLES))   # base screws from below
    return body


def housing(d, ring_poly, z):
    """Ring gear cylinder.  z = 0 is the motor end; printed that way up, so the upper pocket's floor
    (the upper race's seat) is a top face and the lower pocket's lip is a stair of 0.2 mm layers."""
    h = z.housing1 - z.housing0
    body = extrude([circle_poly(BODY_OD / 2), ring_poly], h)
    body += ring(FLANGE_OD, BODY_OD - 0.5, FLANGE_T)                         # mounting flange at the motor end
    body -= holes(M3["clear"], FLANGE_T + 2, -1, polar(FLANGE_R, FLANGE_ANGLES))
    body -= cyl(CAGE_POCKET, z.lip1 - z.housing0 + 1, -1)                    # lower bearing pocket
    for i in range(lip_steps()):                                             # stepped lip, 52.1 in to 50
        dia = max(CAGE_POCKET - 2 * LIP_STEP * (i + 1), CAGE_DA_H)
        body -= cyl(dia, BRIDGE_LAYER + 0.002, z.lip1 - z.housing0 + i * BRIDGE_LAYER - 0.001)
    body -= cyl(CAGE_DA_H, z.teeth0 - z.housing0 + 1, -1)                    # 50 bore (the abutment dia) below the teeth
    r_top = CAGE_DA_H / 2 - TOOTH_LEAD_H                                     # 45 deg cone: tooth bottoms print without a bridge
    body -= m3d.Manifold.cylinder(TOOTH_LEAD_H + 0.001, CAGE_DA_H / 2, r_top, SEG).translate([0, 0, z.teeth0 - z.housing0 - 0.001])
    body -= cyl(CAGE_DA_H, h - (z.teeth1 - z.housing0) + 1, z.teeth1 - z.housing0)   # 50 band under the upper race
    body -= cyl(CAGE_POCKET, UPPER_IN + 1, h - UPPER_IN)                     # upper bearing pocket, 2 mm deep
    body -= holes(BODY_HOLE, h + 2, -1, polar(SCREW_R, SCREW_ANGLES))        # straight through
    return body


def spider(d):
    """Lower cage disc.  z = 0 is its underside (the motor side); the pinch face is on top."""
    hole = 2 * d['ra_sun'] + 2.0
    t = spider_t()
    seat = spider_seat_h()
    cb = AXLE["head_h"] + CB_RECESS
    body = cyl(CAGE_SEAT, seat) + cyl(CAGE_DA, t - seat, seat)
    body -= cyl(hole, t + 2, -1)
    for j in range(d['planets']):
        x, y = pin_xy(d, j)
        body -= cyl(AXLE["cb_d"], cb + 1, -1, x, y)                              # cap screw head, from below
        body -= stepped_bridge(AXLE["clear"], AXLE["cb_d"], cb, x, y)            # printed seat down: the head seat is a roof
        body -= cyl(AXLE["clear"], t - TUBE_POCKET - cb + 0.002, cb + 2 * BRIDGE_LAYER - 0.001, x, y)   # screw clearance
        body -= cyl(TUBE_HOLE, TUBE_POCKET + 1, t - TUBE_POCKET, x, y)           # axle tube pocket, drill 6.0
        body -= pinch_recess(t, x, y, up=True)
    r = CAGE_SEAT / 2                                                            # lead-in on the seat's bed edge
    body -= cyl(CAGE_SEAT + 4, 1.0 + SEAT_LEAD, -1.0) - m3d.Manifold.cylinder(1.0 + SEAT_LEAD, r - SEAT_LEAD - 1.0, r, SEG).translate([0, 0, -1.0])
    return body


def carrier(d, z):
    """Pinch face at z = 0, shoulder, bearing seat, then the boss.  Written boss face down."""
    sh = z.c_shoulder - z.carrier0
    seat1 = sh + CAGE_W
    top = z.boss1 - z.carrier0
    body = cyl(CAGE_DA, sh) + cyl(CAGE_SEAT, CAGE_W, sh) + cyl(BOSS_D, top - seat1, seat1)
    body -= cyl(CENTRE_HOLE, top + 3, -2)
    for j in range(d['planets']):
        x, y = pin_xy(d, j)
        body -= pinch_recess(0.0, x, y, up=False)
        body -= cyl(TUBE_HOLE, TUBE_POCKET + 1, -1, x, y)                        # axle tube pocket, drill 6.0
        body -= cyl(AXLE["tap"], top + 2, -1, x, y)                              # planet screws thread in below the pocket
    for x, y in link_xy():
        body -= cyl(LINK_HOLE, LINK_DEPTH + 1, top - LINK_DEPTH, x, y)           # next arm, blind
        body -= chamfer_cone(LINK_HOLE, 0.4, top, flip=True, x=x, y=y)
    return body


def lid(d):
    """z = 0 is the underside on the body; pocket from below, lip above.  Written top face down."""
    body = cyl(BODY_OD, LID_T)
    body -= cyl(CAGE_POCKET + LID_POCKET_EXTRA, CAGE_W - UPPER_IN + CAGE_PLAY + 1, -1)
    body -= cyl(CAGE_DA_H, LID_T + 2, -1)
    body -= holes(M3["clear"], LID_T + 2, -1, polar(SCREW_R, SCREW_ANGLES))
    (mx, my), = polar(HOME_R, (HOME_ANGLE,))
    body -= cyl(HOME_MAGNET_D + MAGNET_CLEAR, magnet_pocket_depth() + 1, LID_T - magnet_pocket_depth(), mx, my)
    return body


def magnet_pocket_depth():
    return HOME_MAGNET_SINK + HOME_MAGNET_T + MAGNET_CLEAR


def hub_plain_len():
    """Hub screw: under-head seat to the hub's underside (the boss face)."""
    return HUB_T - M3["head_h"] - 0.5


def hub_screw_len():
    """Shortest stocked M3 that leaves 1.5 mm before the bottom of the boss hole."""
    return std_length(hub_plain_len() + LINK_DEPTH - 1.5)


def arm_hub(z):
    """z = 0 is the hub's underside on the boss face; the retaining ring hangs below it."""
    body = cyl(BODY_OD, HUB_T)
    body += ring(CAGE_DA, HUB_BORE, z.retainer_h + 0.001, -z.retainer_h)
    body -= cyl(HUB_RELIEF_ID, HUB_RELIEF_H + 1, -z.retainer_h - 1)               # clear of the race's bore chamfer
    seat = HUB_T - M3["head_h"] - 0.5
    for x, y in link_xy():                                                        # M3 to the carrier boss, from the top
        body -= cyl(M3["clear"], HUB_T + 2, -1, x, y)
        body -= cyl(M3["cb_d"], HUB_T, seat, x, y)
        body -= stepped_bridge(M3["clear"], M3["cb_d"], 0.0, x, y).mirror([0, 0, 1]).translate([0, 0, seat])   # printed top down: the seat is a roof
    for x in (-ARM_BOLT_X, ARM_BOLT_X):                                           # M5 T-nut bolts, heads from below
        body -= cyl(M5["clear"], HUB_T + 2, -1, x, 0.0)
        body -= cyl(M5["cb_d"], M5["head_h"] + 0.5 + 1, -1, x, 0.0)
    tw, rd, dp = SENSOR_POCKET
    (sx, sy), = polar(HOME_R, (SENSOR_ANGLE,))
    pocket = box(-tw / 2, tw / 2, -rd / 2, rd / 2, -1, dp)                        # tangential along x, radial along y
    pocket += box(-SENSOR_CHANNEL[0] / 2, SENSOR_CHANNEL[0] / 2, rd / 2 - 0.5, BODY_OD, -1, SENSOR_CHANNEL[1])
    body -= pocket.rotate([0, 0, SENSOR_ANGLE - 90.0]).translate([sx, sy, 0])
    return body


def sensor_board(z):
    """Check model: a 10 x 8 x 1.6 breakout with a 1.1 mm SOT-23 under it, face down in the hub pocket."""
    tw, rd, dp = SENSOR_POCKET
    (sx, sy), = polar(HOME_R, (SENSOR_ANGLE,))
    b = box(-5.0, 5.0, -4.0, 4.0, SENSOR_RECESS + 1.1, SENSOR_RECESS + 2.7)
    b += box(-1.5, 1.5, -1.5, 1.5, SENSOR_RECESS, SENSOR_RECESS + 1.1 + 0.001)
    return b.rotate([0, 0, SENSOR_ANGLE - 90.0]).translate([sx, sy, z.boss1])


def home_magnet(z):
    (mx, my), = polar(HOME_R, (HOME_ANGLE,))
    return cyl(HOME_MAGNET_D, HOME_MAGNET_T, z.lid1 - HOME_MAGNET_SINK - HOME_MAGNET_T, mx, my)


def d_key(bore, z0, z1):
    """The flat inside a D bore, from z0 to z1, keeping the same clearance to the shaft flat as the
    round part has to the shaft."""
    rr = bore / 2
    flat_x = -(SHAFT_D / 2 - SHAFT_FLAT_DEPTH) - (bore - SHAFT_D) / 2
    return box(-rr - 0.5, flat_x, -rr, rr, z0, z1) ^ cyl(bore + 0.02, z1 - z0 + 2, z0 - 1)


def sun(d, backlash, steps, z):
    """Teeth the full length; D bore straight through, lead-in at both ends."""
    length = z.sun1 - z.sun0
    bore = SUN_BORE or d['sun_bore']
    body = extrude([P.external_profile(d['sun'], d['module'], backlash, steps)], length)
    body -= cyl(bore, length + 2, -1)
    if not ROUND_SHAFT:
        body += d_key(bore, 0.0, length)
    body -= chamfer_cone(bore, SUN_CHAMFER, 0.0)
    body -= chamfer_cone(bore, SUN_CHAMFER, length, flip=True)
    return body


def planet(d, backlash, steps, bore):
    """Gear with a bearing counterbore in each face down to a 1 mm ledge, 11 mm through, lead-ins."""
    h = planet_t()
    cb = cb_depth()
    prof = m3d.CrossSection([P.external_profile(d['planet'], d['module'], backlash, steps)], m3d.FillRule.EvenOdd)
    body = prof.extrude(h - FOOT_H).translate([0, 0, FOOT_H])
    body += prof.offset(-FOOT_RELIEF, m3d.JoinType.Round).extrude(FOOT_H + 0.001)
    body -= cyl(LEDGE_BORE, h + 2, -1)
    body -= cyl(bore, cb + 1, -1)                                            # lower counterbore
    body -= cyl(bore, cb + 1, h - cb)                                        # upper counterbore (its floor is a bridge)
    # no stepped bridge here: the ledge is only 1.05 wide and prints as a short overhang; a slot-and-square
    # bridge sized for the 11 bore has corners at r 8.6 and 7.8, through the 7.75 tooth roots (it split the rim)
    body -= chamfer_cone(bore, ENTRY_CHAMFER, 0.0)
    body -= chamfer_cone(bore, ENTRY_CHAMFER, h, flip=True)
    return body


def planet_spacer():
    return ring(SPACER_OD, SPACER_ID, spacer_t())


def bearing_fit_test(root_d, bores=(13.00, 13.05, 13.10, 13.15, 13.20)):
    """One ring per trial bore with the planet's wall section (OD = tooth root diameter), as deep as
    the gear's counterbore, standing on a thin bar with the gear's 11 mm bore under each as the
    push-out hole; a notch on the end next to the smallest bore."""
    depth = cb_depth()
    pitch, base, wdt = 18.0, 1.6, 18.0
    n = len(bores)
    body = box(0, pitch * n, -wdt / 2, wdt / 2, 0, base)
    for i, b in enumerate(bores):
        x = pitch * (i + 0.5)
        body += cyl(root_d, depth, base, x)
        body -= cyl(b, depth + 1, base, x)
        body -= chamfer_cone(b, ENTRY_CHAMFER, base + depth, flip=True, x=x)
        body -= cyl(LEDGE_BORE, base + 2, -1, x)
    body -= m3d.Manifold.cube([2.0, 2.0, base + 2]).rotate([0, 0, 45]).translate([0, -1.4, -1])
    return body


def cage_fit_test():
    """A pocket ring and a seat ring for the 6808s, side by side on a bar."""
    base = 1.6
    pocket = ring(CAGE_POCKET + 6.0, CAGE_POCKET, CAGE_W, base, 0.0, 0.0)
    pocket += ring(CAGE_POCKET + 6.0, CAGE_DA_H, base, 0.0, 0.0, 0.0)
    seat = cyl(CAGE_SEAT, CAGE_W, base, 52.0, 0.0) - cyl(CAGE_SEAT - 8.0, CAGE_W + 2, base - 1, 52.0, 0.0)
    seat += ring(CAGE_DA, CAGE_SEAT - 8.0, base, 0.0, 52.0, 0.0)
    return pocket + seat


def sun_fit_test(bores=(5.00, 5.05, 5.10, 5.15)):
    """Short D-bore tubes on a bar, one per trial sun bore, each with the flat the sun would get;
    a notch on the end next to the smallest bore."""
    pitch, base, h, od = 12.0, 1.6, 6.0, 10.0
    n = len(bores)
    body = box(0, pitch * n, -6.0, 6.0, 0, base)
    for i, b in enumerate(bores):
        x = pitch * (i + 0.5)
        tube = cyl(od, h, base) - cyl(b, h + base + 2, -1)
        tube -= chamfer_cone(b, SUN_CHAMFER, base + h, flip=True)
        if not ROUND_SHAFT:
            tube += d_key(b, base, base + h)
        body = body - cyl(b + 0.6, base + 2, -1, x) + tube.translate([x, 0, 0])
    body -= m3d.Manifold.cube([2.0, 2.0, base + 2]).rotate([0, 0, 45]).translate([0, -1.4, -1])
    return body


# ---------------------------------------------------------------- bought parts, for the checks
def inner_race(z, x, y):
    return ring(INNER_LAND, BEARING_ID, BEARING_W, z, x, y)


def outer_race(z, x, y):
    """Outer ring + both shields (shields flush with the faces unless SHIELD_RECESS)."""
    o = ring(BEARING_OD, OUTER_LAND, BEARING_W, z, x, y)
    for zs in (z + SHIELD_RECESS, z + BEARING_W - SHIELD_RECESS - 0.25):
        o += ring(OUTER_LAND + 0.01, SHIELD_ID, 0.25, zs, x, y)
    return o


def cage_inner(z):
    return ring(CAGE_INNER_OD, CAGE_D, CAGE_W, z)


def cage_outer(z):
    """Outer ring with both seals attached (they turn with neither race in the model; fixed side)."""
    o = ring(CAGE_OD, CAGE_OUTER_ID, CAGE_W, z)
    for zs in (z + CAGE_SEAL_RECESS, z + CAGE_W - CAGE_SEAL_RECESS - 0.4):
        o += ring(CAGE_OUTER_ID + 0.01, CAGE_INNER_OD + 0.1, 0.4, zs)
    return o


def axle_tube(z, x, y):
    """Worst (smallest) tube, from 0.3 above the spider's pocket floor to 0.3 below the carrier's."""
    return ring(TUBE_OD_MIN, TUBE_ID_MIN, tube_len(), z.spider1 - TUBE_POCKET + TUBE_END_GAP, x, y)


def planet_screw(z, x, y):
    s = cyl(AXLE["head_d"], AXLE["head_h"], z.head0, x, y)
    s += cyl(AXLE["major"], z.carrier0 - z.underhead, z.underhead, x, y)     # plain through spider and bearings
    s += cyl(AXLE["tap"], z.tip - z.carrier0, z.carrier0, x, y)              # cut thread from the island face up
    return s


def motor_parts():
    body = box(-P.NEMA17_FACE / 2, P.NEMA17_FACE / 2, -P.NEMA17_FACE / 2, P.NEMA17_FACE / 2, -40.0, 0.0)
    pilot = cyl(P.NEMA17_PILOT_D, P.NEMA17_PILOT_H)
    r = SHAFT_D / 2
    fs = max(FLAT_START, P.NEMA17_PILOT_H + 0.01)
    shaft = cyl(SHAFT_D, fs - P.NEMA17_PILOT_H, P.NEMA17_PILOT_H)
    flat = cyl(SHAFT_D, SHAFT_LEN - fs, fs)
    keep = box(-(r - SHAFT_FLAT_DEPTH), r + 1, -r - 1, r + 1, -1, SHAFT_LEN + 1)
    shaft += flat if ROUND_SHAFT else flat ^ keep
    heads = []
    cb_top = PLATE_T - MOTOR_HEAD_SINK
    for sx in (-1, 1):
        for sy in (-1, 1):
            heads.append(cyl(M3["head_d"], M3["head_h"], cb_top - M3["head_h"], sx * P.NEMA17_BOLT_HALF, sy * P.NEMA17_BOLT_HALF))
    return body, pilot, shaft, heads


def lid_screw(z, x, y):
    s = cyl(M3["head_d"], M3["head_h"], z.lid1, x, y)
    s += cyl(BODY_HOLE, z.lid_screw, z.lid1 - z.lid_screw, x, y)
    return s


def plate_screw(z, x, y):
    cb = M3["head_h"] + 0.2
    s = cyl(M3["head_d"], M3["head_h"], cb - M3["head_h"], x, y)
    s += cyl(BODY_HOLE, z.plate_screw, cb, x, y)
    return s


CAGE_KINDS = ("spider", "carrier", "planet", "inner_race", "outer_race", "spacer", "screw", "tube", "cage_inner", "cage_outer",
              "arm_hub", "sensor")


# ---------------------------------------------------------------- checks and output
def kind(name):
    return re.sub(r'[\d_]+$', '', name)


def to_trimesh(man):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    f = np.asarray(mesh.tri_verts)
    return trimesh.Trimesh(v, f, process=False)


def boxes_near(a, b, margin):
    pa, pb = a.bounding_box(), b.bounding_box()
    return all(pa[i] - margin <= pb[i + 3] and pb[i] - margin <= pa[i + 3] for i in range(3))


def overlaps(pairs):
    out = []
    for (ni, mi), (nk, mk) in pairs:
        if boxes_near(mi, mk, 0.0) and (mi ^ mk).volume() > 1e-6:
            out.append((ni, nk))
    return out


def slices_png(path, placed, zs, size):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return False
    colours = ["#c00", "#06c", "#080", "#e80", "#808", "#088", "#333", "#8a2", "#a28", "#2a8", "#555", "#a50", "#05a"]
    px = 520
    cols = 3
    rows = (len(zs) + cols - 1) // cols
    img = Image.new("RGB", (px * cols, px * rows), "white")
    dr = ImageDraw.Draw(img)
    k = px / size
    kinds = []
    for name, _ in placed:
        if kind(name) not in kinds:
            kinds.append(kind(name))
    for n, z in enumerate(zs):
        ox, oy = (n % cols) * px + px / 2, (n // cols) * px + px / 2
        dr.text((ox - px / 2 + 6, oy - px / 2 + 4), f"z = {z} mm", fill="black")
        for name, man in placed:
            cs = man.slice(z)
            for poly in cs.to_polygons():
                pts = [(ox + x * k, oy - y * k) for x, y in np.asarray(poly)]
                if len(pts) > 2:
                    dr.polygon(pts, outline=colours[kinds.index(kind(name)) % len(colours)])
    img.save(path)
    return True


PALETTE = {"motor_plate": (150, 150, 150), "housing": (60, 120, 200), "spider": (40, 150, 60),
           "carrier": (230, 140, 30), "lid": (140, 70, 160), "sun": (200, 40, 40),
           "planet": (110, 180, 90), "inner_race": (25, 25, 25), "outer_race": (85, 85, 85),
           "cage_inner": (25, 25, 25), "cage_outer": (85, 85, 85), "arm_hub": (230, 90, 170),
           "sensor": (20, 120, 20), "magnet": (200, 200, 60),
           "screw": (190, 170, 40), "shaft": (120, 120, 140), "motor": (205, 205, 205)}


def section_png(path, placed, y_cut, views):
    """Vertical section in the plane y = y_cut, filled per part.
    views: list of (x0, x1, z0, z1, px_per_mm, title) panels, stacked top to bottom."""
    try:
        from PIL import Image, ImageDraw, ImageChops, ImageFont
    except ImportError:
        return False

    def colour(name):
        k = kind(name)
        for key, c in PALETTE.items():
            if k == key or k.endswith(key):
                return c
        if k.startswith("motor") or k == "pilot":
            return PALETTE["motor"]
        return (0, 0, 0)
    try:
        font = ImageFont.load_default(size=15)
    except TypeError:
        font = ImageFont.load_default()
    cuts = [(n, [np.asarray(p) for p in m.rotate([90, 0, 0]).slice(y_cut).to_polygons()]) for n, m in placed]
    panels = []
    for x0, x1, z0, z1, k, title in views:
        w, h = int((x1 - x0) * k), int((z1 - z0) * k)
        img = Image.new("RGB", (w, h + 26), "white")
        dr = ImageDraw.Draw(img)

        def P2(x, zz):
            return (x - x0) * k, 26 + (z1 - zz) * k
        for name, polys in cuts:
            mask = Image.new("1", (w, h + 26), 0)
            for poly in polys:
                if len(poly) < 3:
                    continue
                one = Image.new("1", (w, h + 26), 0)
                ImageDraw.Draw(one).polygon([P2(x, -yy) for x, yy in poly], fill=1)
                mask = ImageChops.logical_xor(mask, one)
            img.paste(colour(name), (0, 0, w, h + 26), mask)
            for poly in polys:
                if len(poly) >= 3:
                    dr.polygon([P2(x, -yy) for x, yy in poly], outline=(0, 0, 0))
        dr.rectangle([0, 0, w, 25], fill="white")
        dr.text((6, 4), title, fill="black", font=font)
        panels.append(img)
    W = max(p.width for p in panels)
    items = [(k_, c) for k_, c in PALETTE.items() if k_ not in ("inner_race", "outer_race")]
    legend_rows = [items[:7], items[7:]]
    out = Image.new("RGB", (W, sum(p.height for p in panels) + 30 * len(legend_rows)), "white")
    y = 0
    for p in panels:
        out.paste(p, (0, y))
        y += p.height
    dr = ImageDraw.Draw(out)
    for row in legend_rows:
        x = 6
        for key, c in row:
            dr.rectangle([x, y + 8, x + 16, y + 22], fill=c, outline=(0, 0, 0))
            label = key.replace("_", " ")
            dr.text((x + 22, y + 6), label, fill="black", font=font)
            x += 22 + int(dr.textlength(label, font=font)) + 18
        y += 30
    out.save(path)
    return True


def main():
    global BEARING_BORE, SHAFT_LEN, N_PLANETS, ROUND_SHAFT, SUN_BORE, FLAT_START, CAGE_SEAT, CAGE_POCKET
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sun", type=int, default=12)
    ap.add_argument("--planet", type=int, default=18)
    ap.add_argument("--module", type=float, default=1.0)
    ap.add_argument("--planets", type=int, default=3)
    ap.add_argument("--backlash", type=float, default=0.10, help="circular backlash per mesh, mm")
    ap.add_argument("--steps", type=int, default=14)
    ap.add_argument("--bearing-bore", type=float, default=BEARING_BORE,
                    help="printed planet bore for the 13 mm bearings (pick it with bearing_fit_test.stl)")
    ap.add_argument("--cage-seat", type=float, default=CAGE_SEAT, help="printed seat for the 6808 inner races (light press)")
    ap.add_argument("--cage-pocket", type=float, default=CAGE_POCKET, help="printed pocket for the 6808 outer races (push fit)")
    ap.add_argument("--sun-bore", type=float, default=0.0,
                    help="sun bore for the 5 mm shaft; 0 = 5.15.  Smaller grips harder (there is no set screw)")
    ap.add_argument("--flat-start", type=float, default=FLAT_START,
                    help="where the motor shaft's D flat begins, mm from the mounting face (measure it); the sun sits above it")
    ap.add_argument("--screw-length", type=int, default=0,
                    help="planet screw length under the head; 0 picks the shortest stocked length with enough thread")
    ap.add_argument("--shaft-length", type=float, default=SHAFT_LEN, help="motor shaft length from the mounting face")
    ap.add_argument("--round-shaft", action="store_true", help="motor shaft has no flat: round sun bore")
    ap.add_argument("--out", default="gearbox_stl")
    a = ap.parse_args()
    N_PLANETS, ROUND_SHAFT, FLAT_START = a.planets, a.round_shaft, a.flat_start
    BEARING_BORE, SHAFT_LEN = a.bearing_bore, a.shaft_length
    CAGE_SEAT, CAGE_POCKET = a.cage_seat, a.cage_pocket
    SUN_BORE = a.sun_bore or None

    d = P.design(a.module, a.planets, a.sun, a.planet, BEARING_ID, 0.0)
    if not d['ok']:
        sys.exit("that tooth combination fails the planetary design rules; run planetary.py list")
    if 2 * d['ra_ring'] > CAGE_DA_H - 0.2:
        sys.exit(f"ring gear tips ({2 * d['ra_ring']:.1f}) reach past the bearing lip bore ({CAGE_DA_H}); use a smaller ring")
    rim = d['rf_planet'] - BEARING_BORE / 2
    face_wall = d['rf_planet'] - FOOT_RELIEF - BEARING_BORE / 2 - ENTRY_CHAMFER
    z = Stack(a.screw_length or auto_screw_length())
    os.makedirs(a.out, exist_ok=True)
    sun_bore = SUN_BORE or d['sun_bore']
    sun_wall = d['rf_sun'] - sun_bore / 2

    sun_poly, planet_polys, ring_poly = P.assembly(d, a.backlash, a.steps)

    parts = {
        "motor_plate": motor_plate(d),
        "housing": housing(d, ring_poly, z),
        "spider": spider(d),
        "carrier": carrier(d, z),
        "lid": lid(d),
        "arm_hub": arm_hub(z),
        "planet_spacer": planet_spacer(),
        "sun": sun(d, a.backlash, a.steps, z),
        "planet": planet(d, a.backlash, a.steps, BEARING_BORE),
    }
    print_rot = {"carrier": True, "lid": True, "arm_hub": True}   # written upside down (the spider prints as modelled)

    # ---- everything in the assembled position
    motor_body, pilot, shaft, motor_heads = motor_parts()
    placed = [("motor_plate", parts["motor_plate"]),
              ("housing", parts["housing"].translate([0, 0, z.housing0])),
              ("spider", parts["spider"].translate([0, 0, z.spider0])),
              ("carrier", parts["carrier"].translate([0, 0, z.carrier0])),
              ("lid", parts["lid"].translate([0, 0, z.lid0])),
              ("arm_hub", parts["arm_hub"].translate([0, 0, z.boss1])),
              ("sensor", sensor_board(z)), ("magnet", home_magnet(z)),
              ("sun", parts["sun"].translate([0, 0, z.sun0])),
              ("cage_inner1", cage_inner(z.b1)), ("cage_outer1", cage_outer(z.b1)),
              ("cage_inner2", cage_inner(z.b2)), ("cage_outer2", cage_outer(z.b2)),
              ("motor", motor_body), ("pilot", pilot), ("shaft", shaft)]
    placed += [(f"motor_head{i}", h) for i, h in enumerate(motor_heads)]
    for j in range(d['planets']):
        phi = 2 * math.pi * j / d['planets']
        rho = phi + math.pi - math.pi / d['planet'] + d['sun'] * phi / d['planet']
        x, y = pin_xy(d, j)
        placed.append((f"planet{j}", parts["planet"].rotate([0, 0, math.degrees(rho)]).translate([x, y, z.gear0])))
        # bearing 0 sits on the ledge from below; bearing 1's outer ring sits on the ledge from above and
        # its inner ring is pushed SPACER_EXTRA further up by the spacer (within the bearing's axial play)
        placed.append((f"inner_race{j}_0", inner_race(z.planet0, x, y)))
        placed.append((f"outer_race{j}_0", outer_race(z.planet0, x, y)))
        placed.append((f"spacer{j}", planet_spacer().translate([x, y, z.spacer0])))
        placed.append((f"inner_race{j}_1", inner_race(z.spacer0 + spacer_t(), x, y)))
        placed.append((f"outer_race{j}_1", outer_race(z.ledge1, x, y)))
        placed.append((f"screw{j}", planet_screw(z, x, y)))
        placed.append((f"tube{j}", axle_tube(z, x, y)))
    for i, (x, y) in enumerate(polar(SCREW_R, SCREW_ANGLES)):
        placed.append((f"lid_screw{i}", lid_screw(z, x, y)))
        placed.append((f"plate_screw{i}", plate_screw(z, x, y)))

    # ---- intersection and gap report
    print("Pairwise check in the assembled position (volume of overlap must be 0):")
    bad = 0
    nearest = {}
    for i in range(len(placed)):
        for k in range(i + 1, len(placed)):
            ni, mi = placed[i]
            nk, mk = placed[k]
            if not boxes_near(mi, mk, 2.0):
                continue
            if {kind(ni), kind(nk)} == {"sun", "shaft"} and sun_bore < SHAFT_D:
                continue                            # chosen interference fit, reported below
            press = ({kind(ni), kind(nk)} in ({"cage_inner", "spider"}, {"cage_inner", "carrier"})
                     and CAGE_SEAT < CAGE_D)        # press fit on the seat, by design: no overlap test
            vol = 0.0 if press else (mi ^ mk).volume()
            if vol > 1e-6:
                bad += 1
                print(f"  OVERLAP {ni} x {nk}: {vol:.4f} mm^3")
            gap = mi.min_gap(mk, 2.0)
            key = tuple(sorted((kind(ni), kind(nk))))
            if gap < 2.0 and (key not in nearest or gap < nearest[key][0]):
                nearest[key] = (gap, ni, nk)
    print("  smallest gap between each kind of part (0.000 = touching):")
    for key, (gap, ni, nk) in sorted(nearest.items(), key=lambda kv: (kv[1][0], kv[0])):
        print(f"    {gap:6.3f} mm  {key[0]} - {key[1]}")
    print(f"  overlapping pairs: {bad}")

    # ---- the cage at the top of its float
    cage = [(n, m) for n, m in placed if kind(n) in CAGE_KINDS]
    fixed = [(n, m) for n, m in placed if kind(n) not in CAGE_KINDS]
    moved = [(n, m.translate([0, 0, z.float_up * 0.98])) for n, m in cage]
    clear = overlaps([(c, f) for c in moved for f in fixed])
    moved = [(n, m.translate([0, 0, z.float_up + 0.02])) for n, m in cage]
    stops = sorted({tuple(sorted((kind(a_), kind(b_)))) for a_, b_ in overlaps([(c, f) for c in moved for f in fixed])})
    print("Cage float")
    print(f"  up {z.float_up:.2f} mm: overlaps just short of the stop {len(clear)}; stops on "
          + ", ".join(f"{a_}/{b_}" for a_, b_ in stops))

    def g(k1, k2):
        return nearest.get(tuple(sorted((k1, k2))), (99.0,))[0]

    hm = dict(placed)["housing"]
    zc = (z.teeth_full0 + z.teeth1) / 2
    sl = hm.slice(zc).to_polygons()
    ring_bore = 2 * min(math.hypot(px, py) for poly in sl for px, py in poly)
    wall_tap = BOSS_D / 2 - (d['carrier_r'] + AXLE["tap"] / 2)
    n_pl = d['planets']
    e = float_e()
    arm_low = z.boss1 - z.short - z.heads1
    band = (min(BOSS_OD, INNER_LAND) - max(TUBE_HOLE_MAX, INNER_CONTACT_MIN)) / 2
    pinch_n = PINCH_TORQUE / (K_PETG * SCREW_D / 1000.0)
    pinch_mpa = pinch_n / (math.pi / 4 * (min(BOSS_OD, INNER_LAND) ** 2 - max(TUBE_HOLE_MAX, INNER_CONTACT_MIN) ** 2))
    island_wall = (BOSS_OD - TUBE_HOLE_MAX) / 2
    race_radial = (RECESS_D - BEARING_OD) / 2 - e
    seat_wall = CAGE_SEAT / 2 - (d['carrier_r'] + AXLE["cb_d"] / 2)
    body_wall_in = SCREW_R - BODY_HOLE / 2 - CAGE_POCKET / 2
    body_wall_out = BODY_OD / 2 - SCREW_R - BODY_HOLE / 2
    screw_meet = (z.plate_screw - (PLATE_T - M3["head_h"] - 0.2)) + (z.lid_screw - LID_T)
    if rim >= RIM_RULE - 1e-9:
        rim_check = ("PASS", f"plastic between the bearing bore and the tooth roots {rim:.2f} mm (want {RIM_RULE:g})", True)
    else:
        rim_check = ("WARN" if rim >= RIM_MIN - 1e-9 else "FAIL",
                     f"plastic between the bearing bore and the tooth roots {rim:.2f} mm, under the {RIM_RULE:g} rule: "
                     f"fine only with a firm press fit (--sun 11 --planet 19 gives 1.70)", False)
    checks = [
        ("PASS", f"{N_BEARINGS} bearings per planet; inner races and the spacer form one clamped stack, "
                 f"spacer {spacer_t():g} on a {LEDGE_T:g} ledge",
         g("inner_race", "spacer") < 1e-3 and spacer_t() > LEDGE_T),
        ("PASS", f"gear {planet_t():g} long: counterbores {cb_depth():g} deep, bearings {PROTRUDE:g} proud of the faces "
                 f"({PROTRUDE - BEARING_W_TOL:.2f} with a thin bearing), ledge {(BEARING_BORE - LEDGE_BORE) / 2:.2f} wide",
         PROTRUDE - BEARING_W_TOL >= 0.3 and (BEARING_BORE - LEDGE_BORE) / 2 >= 0.8),
        ("PASS", f"spacer {SPACER_OD:g} OD inside the inner land ({INNER_LAND:g}), clear of the shields; {SPACER_ID:g} bore on the tube",
         SPACER_OD <= INNER_LAND and SPACER_ID > TUBE_OD and g("spacer", "outer_race") >= 0.1),
        ("PASS", f"axle tube {TUBE_OD:g} x {tube_len():.1f}: {TUBE_POCKET - TUBE_END_GAP:.1f} mm into each plate, ends {TUBE_END_GAP:g} clear of "
                 f"both pocket floors (never takes the clamp); M4 passes its {TUBE_ID_MIN:g} bore",
         TUBE_POCKET - TUBE_END_GAP >= 2.5 and g("tube", "spider") >= 0.0 and g("tube", "carrier") >= 0.0 and TUBE_ID_MIN > AXLE["major"]),
        ("PASS", f"island ring {island_wall:.2f} mm wide round the tube pocket, printed as a top face (want 0.5)", island_wall >= 0.5),
        rim_check,
        ("PASS", f"plastic at the gear faces, over the bore lead-ins {face_wall:.2f} mm (want 0.8)", face_wall >= 0.8 - 1e-9),
        ("PASS", "pinch islands on the spider and carrier touch the end inner races",
         g("spider", "inner_race") < 1e-3 and g("carrier", "inner_race") < 1e-3),
        ("PASS", f"island contact at {PINCH_TORQUE:g} N*m: {pinch_mpa:.0f} MPa on the narrowest band (want under {PINCH_STRESS_MAX:g})",
         pinch_mpa <= PINCH_STRESS_MAX),
        ("PASS", f"island face overlaps the inner race band ({INNER_CONTACT_MIN:g}..{INNER_LAND:g}) by {band:.2f} mm, "
                 f"{band - e:.2f} mm with the race {e:.3f} off-centre on the tube (want 0.15)", band - e >= 0.15),
        ("PASS", f"islands clear of the shield edge by {(SHIELD_ID - BOSS_OD) / 2 - e:.2f} mm with the race {e:.3f} off-centre (want 0.1)",
         (SHIELD_ID - BOSS_OD) / 2 - e >= 0.1 - 1e-9),
        ("PASS", f"outer races clear the recess floor by {RECESS:g} mm and its wall by {race_radial:.2f} mm "
                 f"with the race {e:.3f} off-centre (want 0.3)", race_radial >= 0.3 - 1e-9 and RECESS >= 0.4 - 1e-9),
        ("PASS", f"solid web between each screw head and the tube pocket {solid_web():.1f} mm (want {SOLID_WEB:g})", solid_web() >= SOLID_WEB - 1e-9),
        ("PASS", f"planet gears {g('planet', 'spider'):.2f} mm off the spider and carrier faces (want {PROTRUDE:g})",
         min(g("spider", "planet"), g("carrier", "planet")) >= PROTRUDE - 1e-6),
        ("PASS", f"upper 6808 {UPPER_IN:g} mm into the body, {CAGE_W - UPPER_IN:g} in the lid (lid pocket "
                 f"{CAGE_POCKET + LID_POCKET_EXTRA:g}); outer race on the body's pocket floor",
         UPPER_IN >= 2.0 and CAGE_W - UPPER_IN >= 3.0 and g("cage_outer", "housing") < 1e-3),
        ("PASS", f"6808 inner races sit on the spider and carrier shoulders ({CAGE_DA:g}, the abutment dia) and the seats ({CAGE_SEAT:g})",
         g("cage_inner", "spider") < 1e-3 and g("cage_inner", "carrier") < 1e-3 and CAGE_DA <= CAGE_INNER_OD),
        ("PASS", f"6808 outer races: lower on the plate under the housing lip, upper on the body under the lid lip, {CAGE_PLAY:g} play",
         g("cage_outer", "motor_plate") < 1e-3 and g("cage_outer", "housing") < 1e-3
         and abs(z.lip1 - z.b1_top - CAGE_PLAY) < 1e-6 and abs(z.lid_lip0 - z.b2_top - CAGE_PLAY) < 1e-6
         and abs((z.housing1 - UPPER_IN) - z.b2) < 1e-6),
        ("PASS", f"lips ({CAGE_DA_H:g}) at the housing abutment dia; nothing touches the 6808 seals",
         g("cage_outer", "spider") >= 0.3 and g("cage_outer", "carrier") >= 0.3 and g("cage_inner", "housing") >= 0.3
         and g("cage_inner", "lid") >= 0.3 and g("cage_inner", "motor_plate") >= 0.3),
        ("PASS", f"planet screw counterbores leave {seat_wall:.2f} mm of wall inside the spider seat (want 1.0)", seat_wall >= 1.0),
        ("PASS", f"body screw holes: {body_wall_in:.2f} mm to the bearing pocket, {body_wall_out:.2f} to the outside (want 2.5)",
         min(body_wall_in, body_wall_out) >= 2.5),
        ("PASS", f"plate and lid screws in the same through hole stay {z.housing1 - z.housing0 - screw_meet:.1f} mm apart",
         z.housing1 - z.housing0 - screw_meet >= 2.0),
        ("PASS", "ring gear tips clear the lid and housing lip bores",
         2 * d['ra_ring'] <= CAGE_DA_H - 0.2),
        ("PASS", f"ring teeth full depth from z {z.teeth_full0:.1f} (lead-in cone from {z.teeth0:.1f}) to {z.teeth1:.1f}: "
                 f"cover the planet gears ({z.gear0:.1f}..{z.gear1:.1f}) by 0.4 below and 1 mm above the float",
         z.teeth_full0 <= z.gear0 - 0.4 + 1e-9 and z.teeth1 >= z.gear1 + z.float_up + 1.0 - 1e-9
         and CAGE_DA_H / 2 - TOOTH_LEAD_H <= d['ra_ring'] + 1e-9),      # the cone ends inside the tip circle
        ("PASS", f"ring gear bore measured at mid-height {ring_bore:.2f} (tips {2 * d['ra_ring']:.2f})",
         abs(ring_bore - 2 * d['ra_ring']) < 0.1),
        ("PASS", f"sun wall round the {sun_bore:g} bore {sun_wall:.2f} mm (want {P.MIN_SUN_WALL:g})", sun_wall >= P.MIN_SUN_WALL - 1e-9),
        ("PASS", f"thread in the carrier {z.engage:.1f} mm above the tube pocket's floor (want {MIN_ENGAGE:g})", z.engage >= MIN_ENGAGE),
        ("PASS", f"plastic between each tap hole and the boss rim {wall_tap:.1f} mm (want 1.8)", wall_tap >= 1.8),
        ("PASS", f"hub ring {z.retainer_h:.1f} mm tall sits on the upper inner race and clears its seal; the hub clears the lid "
                 f"screw heads by {arm_low:.2f} mm (want 0.4)",
         arm_low >= 0.4 - 1e-9 and z.retainer_h >= 1.0 and g("arm_hub", "cage_inner") < 1e-3 and g("arm_hub", "cage_outer") >= 0.3),
        ("PASS", f"sensor face {SENSOR_RECESS:g} inside the hub underside, {g('sensor', 'lid_screw'):.2f} mm over the lid screw heads, "
                 f"{g('sensor', 'magnet') if g('sensor', 'magnet') < 90 else z.boss1 + SENSOR_RECESS - (z.lid1 - HOME_MAGNET_SINK):.1f} mm "
                 f"to the magnet face (want 4..8)",
         g("sensor", "lid_screw") >= 0.5 and 4.0 <= z.boss1 + SENSOR_RECESS - (z.lid1 - HOME_MAGNET_SINK) <= 8.0),
        ("PASS", f"magnet pocket: {LID_T - magnet_pocket_depth():.1f} mm of lid under it (want 2.5), "
                 f"{HOME_R - (HOME_MAGNET_D + MAGNET_CLEAR) / 2 - (CAGE_POCKET + LID_POCKET_EXTRA) / 2:.2f} to the bearing pocket, "
                 f"{BODY_OD / 2 - HOME_R - (HOME_MAGNET_D + MAGNET_CLEAR) / 2:.2f} to the rim (want 1.0)",
         LID_T - magnet_pocket_depth() >= 2.5 and HOME_R - (HOME_MAGNET_D + MAGNET_CLEAR) / 2 - (CAGE_POCKET + LID_POCKET_EXTRA) / 2 >= 1.0
         and BODY_OD / 2 - HOME_R - (HOME_MAGNET_D + MAGNET_CLEAR) / 2 >= 1.0),
        ("PASS", f"hub M5 counterbores clear the ring by {ARM_BOLT_X - M5['cb_d'] / 2 - CAGE_DA / 2:.1f} and the M3 heads by "
                 f"{ARM_BOLT_X - LINK_PCD / 2 - M5['cb_d'] / 2 - M3['cb_d'] / 2:.1f} mm (want 0.5)",
         ARM_BOLT_X - M5['cb_d'] / 2 - CAGE_DA / 2 >= 0.5 and ARM_BOLT_X - LINK_PCD / 2 - M5['cb_d'] / 2 - M3['cb_d'] / 2 >= 0.5),
        ("PASS", f"hub screws M3x{hub_screw_len():g}: {hub_screw_len() - hub_plain_len():.1f} mm in the boss hole (want 5), "
                 f"{LINK_DEPTH - (hub_screw_len() - hub_plain_len()):.1f} short of its bottom (want 1)",
         hub_screw_len() - hub_plain_len() >= 5.0 and LINK_DEPTH - (hub_screw_len() - hub_plain_len()) >= 1.0),
        ("PASS", "flange holes clear of the body screw holes and the body", FLANGE_R - M3["clear"] / 2 > BODY_OD / 2 + 0.5
         and FLANGE_R + M3["clear"] / 2 < FLANGE_OD / 2 - 1.5),
        ("PASS", f"sun teeth ({z.sun0:g}..{z.sun1:g}) cover the planet gears ({z.gear0:g}..{z.gear1:g})",
         z.sun0 <= z.gear0 - 0.3 and z.sun1 >= z.gear1 + 0.3),
        ("PASS", f"shaft inside the sun {z.sun_grip:.1f} mm (want 10): flat from z {FLAT_START:g}, sun bottom at z {z.sun0:g}",
         z.sun_grip >= 10.0 - 1e-9),
        ("PASS", f"motor shaft ({SHAFT_LEN:g}) tip clear of the carrier and below the boss face",
         SHAFT_LEN <= z.boss1 - 0.5 and g("shaft", "carrier") > 0.5),
        ("PASS", "the sun is not an axial stop for the carrier", g("carrier", "sun") >= GAP + 0.3),
        ("PASS", "link holes clear of the planet screw holes by 3 mm or more",
         min(math.dist(p, pin_xy(d, j)) for p in link_xy() for j in range(n_pl)) - LINK_HOLE / 2 - AXLE["tap"] / 2 >= 3.0),
        ("PASS", "no part overlaps another", bad == 0),
        ("PASS", "no overlaps with the cage at the top of its float; it stops on the bearing lips",
         not clear and (("cage_outer", "housing") in stops or ("cage_outer", "lid") in stops)),
    ]
    print("Design checks")
    failed = 0
    for level, text, ok in checks:
        tag = "PASS" if ok else ("WARN" if level == "WARN" else "FAIL")
        print(f"  {tag}  {text}")
        failed += 0 if ok or level == "WARN" else 1

    # ---- files
    parts["bearing_fit_test"] = bearing_fit_test(2 * d['rf_planet'])
    parts["cage_fit_test"] = cage_fit_test()
    parts["sun_fit_test"] = sun_fit_test()
    for name, man in parts.items():
        if print_rot.get(name):
            man = man.rotate([180, 0, 0])
            man = man.translate([0, 0, -man.bounding_box()[2]])
        tm = to_trimesh(man)
        tm.export(os.path.join(a.out, f"{name}.stl"))
        print(f"  {name + '.stl':21s} watertight={tm.is_watertight}  volume={man.volume():8.1f} mm^3  "
              f"bbox={np.round(tm.extents, 1).tolist()}")
    try:
        show = [(n, m) for n, m in placed if kind(n) not in ("motor", "pilot", "next_arm")]
        scene = trimesh.Scene({n: to_trimesh(m) for n, m in show})
        scene.export(os.path.join(a.out, "assembly.3mf"))
        print("  assembly.3mf (printed parts and hardware in the assembled position)")
    except Exception as ex:  # noqa: BLE001
        print(f"  assembly.3mf skipped: {ex}")

    zs = [3.0, round(z.b1 + 3.5, 1), round(z.spider1 - 0.3, 1), round((z.gear0 + z.gear1) / 2, 1),
          round(z.carrier0 + 0.3, 1), round(z.b2 + 3.5, 1), round(z.lid1 - 1.0, 1), round(z.boss1 - 1.0, 1)]
    if slices_png(os.path.join(a.out, "slices.png"),
                  [(n, m) for n, m in placed if kind(n) not in ("motor", "next_arm")], zs, FLANGE_OD + 6):
        print("  slices.png")
    hw = FLANGE_OD / 2 + 1.5
    cx = d['carrier_r']
    if section_png(os.path.join(a.out, "section.png"), [(n, m) for n, m in placed if kind(n) != "next_arm"], 0.0,
                   [(-hw, hw, -2.0, z.boss1 + HUB_T + 1.5, 13, "Section through the sun, planet 0, both cage bearings and the arm hub (plane y = 0)"),
                    (cx - 12.0, cx + 12.0, z.plate1 - 1.0, z.lid1 + 1.0, 32,
                     "Planet 0 close up: cage bearings on the spider and carrier, planet bearings pinched by the islands")]):
        print("  section.png")

    # ---- the numbers you need at the bench
    print("Stack, z from the motor face (mm)")
    print(f"  motor plate  {z.plate0:5.1f} .. {z.plate1:5.1f}   OD {BODY_OD:g}; {P.NEMA17_PILOT_D + 0.2:g} centre hole takes the pilot boss; "
          f"motor heads {MOTOR_HEAD_SINK:g} below the top; body screw heads flush underneath; base holes {M3['tap']} at {BASE_ANGLES} deg")
    print(f"  lower 6808   {z.b1:5.1f} .. {z.b1_top:5.1f}   outer race in the body pocket ({CAGE_POCKET:g}) on the plate, lip at {z.lip1:.1f}; "
          f"inner race on the spider seat ({CAGE_SEAT:g}), shoulder at {z.shoulder0:.1f}")
    print(f"  spider       {z.spider0:5.1f} .. {z.spider1:5.1f}   seat {CAGE_SEAT:g} x {spider_seat_h():g}, shoulder {CAGE_DA:g}; M4 counterbores "
          f"{AXLE['cb_d']} x {AXLE['head_h'] + CB_RECESS:g} from below (bridged seats); islands {BOSS_OD:g} in {RECESS_D:g} x {RECESS:g} "
          f"recesses on top, each with a {TUBE_HOLE:g} x {TUBE_POCKET:g} tube pocket (drill 6.0); printed seat face down")
    print(f"  planets      {z.planet0:5.1f} .. {z.planet1:5.1f}   {N_BEARINGS} x 686ZZ each in {BEARING_BORE:.2f} counterbores "
          f"{cb_depth():g} deep on a {LEDGE_T:g} ledge ({LEDGE_BORE:g} through); gear {planet_t():g} long, z {z.gear0:.1f}..{z.gear1:.1f}; "
          f"bearings {PROTRUDE:g} proud; spacer {SPACER_OD:g}/{SPACER_ID:g} x {spacer_t():g} between the inner races; "
          f"rim under the roots {rim:.2f}, {face_wall:.2f} at the faces")
    print(f"  sun          {z.sun0:5.1f} .. {z.sun1:5.1f}   bottom face {z.sun0 - z.plate1:.1f} mm above the plate's top face "
          f"(top {z.sun1 - z.plate1:.1f}); {sun_bore:g} {'round' if ROUND_SHAFT else 'D'} bore straight through, no set screw; "
          f"shaft flat from z {FLAT_START:g}, {z.sun_grip:.1f} mm of shaft inside the sun"
          + (f"; INTERFERENCE {SHAFT_D - sun_bore:.2f} on the shaft" if sun_bore < SHAFT_D else ""))
    print(f"  pinch        islands {BOSS_OD:g} dia on both plates press on the inner races ({pinch_n:.0f} N at {PINCH_TORQUE:g} N*m); "
          f"the {RECESS_D:g} x {RECESS:g} recesses clear the outer rings")
    print(f"  axle tubes   {n_pl} x {TUBE_OD:g} OD x {tube_len():.1f} long, bore {TUBE_ID_MIN:g} or more (brass 6 x 0.45, aluminium 6 x 4); "
          f"from z {z.spider1 - TUBE_POCKET + TUBE_END_GAP:.1f} to {z.carrier0 + TUBE_POCKET - TUBE_END_GAP:.1f}")
    print(f"  carrier      {z.carrier0:5.1f} .. {z.boss1:5.1f}   shoulder {CAGE_DA:g} to z {z.c_shoulder:.1f}, seat {CAGE_SEAT:g} to z {z.b2_top:.1f}, "
          f"boss {BOSS_D:g} above; islands with {TUBE_HOLE:g} x {TUBE_POCKET:g} tube pockets and {AXLE['tap']} tap holes below them in "
          f"{RECESS_D:g} x {RECESS:g} recesses underneath; printed boss face down")
    print(f"  upper 6808   {z.b2:5.1f} .. {z.b2_top:5.1f}   outer race {UPPER_IN:g} mm down in the body's top pocket, "
          f"the rest in the lid's {CAGE_POCKET + LID_POCKET_EXTRA:g} pocket, lid lip at {z.lid_lip0:.1f}; "
          f"inner race on the carrier seat against its shoulder; the hub's ring {z.retainer_h:.1f} tall on top")
    print(f"  housing      {z.housing0:5.1f} .. {z.housing1:5.1f}   OD {BODY_OD:g}; {CAGE_POCKET:g} pockets {z.lip1 - z.housing0:g} deep "
          f"at the motor end (stepped lip to z {z.teeth0:.1f}) and {UPPER_IN:g} deep at the lid end; "
          f"ring teeth z {z.teeth0:.1f}..{z.teeth1:.1f} (full depth from {z.teeth_full0:.1f}); "
          f"{BODY_HOLE:g} holes through on r = {SCREW_R} at {SCREW_ANGLES} deg; printed motor end down")
    print(f"  lid          {z.lid0:5.1f} .. {z.lid1:5.1f}   OD {BODY_OD:g}, pocket {CAGE_POCKET + LID_POCKET_EXTRA:g} x "
          f"{CAGE_W - UPPER_IN + CAGE_PLAY:g} underneath, "
          f"lip bore {CAGE_DA_H:g}; printed top face down")
    print(f"  float        cage {z.float_up:.1f} mm axially (outer race play), tilt only within the 6808s' clearance")
    print(f"  planet screws {n_pl} x M4 x {z.screw_len} socket head, heads {CB_RECESS:g} up inside the spider, "
          f"tips at z {z.tip:.1f}; thread {z.engage:.1f} mm in the carrier, {wall_tap:.1f} mm of wall to the boss rim; "
          f"only snug, about {PINCH_TORQUE:g} N*m")
    print(f"  tap holes    {AXLE['tap']} mm below the tube pockets: drill 3.3 and tap M4 (a cut thread, not a formed one)")
    print(f"  body screws  3 x M3 x {z.plate_screw} up from under the plate, 3 x M3 x {z.lid_screw} down through the lid, "
          f"into the {BODY_HOLE:g} holes (drill 2.5 if tight); they stay {z.housing1 - z.housing0 - screw_meet:.1f} mm apart inside")
    print(f"  motor screws 4 x M3 x 6, heads in the plate's top counterbores")
    print(f"  plate holes  3 x {M3['tap']} at {BASE_ANGLES} deg on r = {SCREW_R} through the plate (spare, for a plate-down mount)")
    print(f"  slicing      support OFF; Enable Bridge Settings on (spider head seats); islands print as top faces")
    print(f"  bearings     2 x 6808-2RS (40 x 52 x 7); {N_BEARINGS * n_pl} x 686ZZ (6 x 13 x 5, plain); "
          f"{n_pl} printed planet spacers")
    print(f"  arm hub      {z.boss1:5.1f} .. {z.boss1 + HUB_T:5.1f}   OD {BODY_OD:g} x {HUB_T:g}; ring {CAGE_DA:g}/{HUB_BORE:g} x {z.retainer_h:.1f} "
          f"below it on the inner race; 3 x M3x{hub_screw_len():g} at {link_angles()} deg on {LINK_PCD:g} PCD into the boss "
          f"({hub_screw_len() - hub_plain_len():.1f} mm engaged of {LINK_DEPTH:g}); "
          f"2 x M5 T-nut bolts at x = +-{ARM_BOLT_X:g} (2020 along x); sensor pocket {SENSOR_POCKET[0]:g} x {SENSOR_POCKET[1]:g} x "
          f"{SENSOR_POCKET[2]:g} at r {HOME_R:g}, {SENSOR_ANGLE:g} deg; underside {z.boss1 - z.lid1:.1f} above the lid; printed top face down")
    print(f"  homing       magnet {HOME_MAGNET_D:g} x {HOME_MAGNET_T:g} glued in the lid at r {HOME_R:g}, {HOME_ANGLE:g} deg, face "
          f"{HOME_MAGNET_SINK:g} below the lid top; sensor face {z.boss1 + SENSOR_RECESS - (z.lid1 - HOME_MAGNET_SINK):.1f} above it")
    print(f"  base mount   flange {FLANGE_OD:g} x {FLANGE_T:g} at the body's motor end, 4 x {M3['clear']} holes on r = {FLANGE_R:g} at "
          f"{FLANGE_ANGLES} deg; the tower top needs a {BODY_OD + 1.5:.0f} mm hole for the plate and motor")
    if bad or failed:
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
square_cell.py - one chessboard square's light cell (Robo chessboard design; v2 2026-11-06, v2d).

    python square_cell.py            -> chessboard/cell/square_cell.step, bucket.step, spider.step  (STEP only)

The Professor's rules for v2 (after v1): "I want the wire channels to be on the bottom of the box, outside of the
light buckets ... a frame inside the light bucket that goes from corner to corner, that has a post on it that goes up
to hold the Hall effect switch. That frame can be a separate part. The diode recess to be off to the side of the wire
channel ... I need everything on the bottom of this that needs to be soldered."

So every joint is made from underneath, with the board upside down, before the bay is closed:
  bucket   white PLA light box: a pyramid funnel narrowing from the ring down to the LED at the square's centre; its
           solid floor carries, on the UNDERSIDE: the LED segment's pocket (board up against the floor, the LED through a
           hole into the funnel's point, its pads facing down onto the wires), the rank channels (bare 20 AWG: GND,
           data, 5V under the LED's end pads, ROW beside it), the COLUMN channel along the file (deeper: insulated
           wire, crossing over the bare rank wires), the holes the Hall's GND (into the LED segment's GND pad hole)
           and + (onto the ROW wire) come down through, and the channel the diode's cathode lead runs in to the
           column wire. Corner seats inside the funnel hold the frame; a hole down through the solid corner
           takes the Hall's wires to the underside. Top: 0.4 outer wall (print the top 3 mm black, M600) and the ring
           pocket, as v1.
  frame    the "spider", separate white part: two arms corner to corner (an X), as wide as the corner seats, their
           square ends filling the seats (his P1.F105); the post at the crossing, turned 45 deg with the Hall, so the
           Hall's leads point down ONE leg (the -x-y one); the three wires come down the post into that leg; + and GND drop
           through a hole near the post and straight down through the floor, the diode lies at the CORNER end of the leg (his v2d:
           "utilize the length of the leads ... so we don't have to add extensions"): the Hall's own OUT lead and the
           diode's anode lead meet side by side in a groove along the leg, and the cathode lead goes straight down
           the corner hole to the column wire, which now runs right beside that hole
  ring     translucent PLA, 1.2 wide, flush with the play surface, with a lip the cover rests on (as v1)
  cover    1/8" craft wood, stained natural / walnut (as v1)
Stand-ins: DRV5032FB (TO-92, bevelled face up), its three wires and the diode on its OUT wire (soldered at the bench,
heat-shrunk, before the spider goes in; the only joints not made from below), the TREEDIX WS2812B segment, the 1N4001 (DO-41), the bare rank wires and the insulated
column wire.

Matrix (the Professor, v2c: "We're going to turn these on with the output of the nano"): the Hall's GND goes to the
LED's GND, its + to the rank's ROW wire, which the Nano drives HIGH to power that rank's eight Halls (LOW = off). OUT
-> diode anode; diode cathode (stripe) -> the file's COLUMN wire, with a 10k pull-down per column at the Nano.
Column HIGH = no magnet, LOW = a piece on the square.

Cell frame: x along the rank, y along the file, z up; z 0 = the bucket's underside (the soldering face), centre of the
square at x = y = 0. The play surface is FIELD_D above z 0.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402

from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "chessboard", "cell", "square_cell.step")

# ---------------------------------------------------------------- the stack
PITCH = 60.0            # square pitch (unchanged: the arm's plans depend on it)
FIELD_D = 24.0          # underside -> play surface (two 1/2" plies; the arm wants 25 - open)
COVER_T = 3.175         # 1/8" craft wood
GAP = 0.1
WALL_TOP = 0.4          # bucket's outer wall at the ring: two of them = the dark line between rings
RING_W = 1.2
LIP_W, LIP_T = 1.0, 1.0
RING_BELOW = 1.0
COLOUR_CHANGE = 3.0     # print black from FIELD_D - this

# ---------------------------------------------------------------- floor and the underside
FLOOR_T = 4.0           # the funnel's point (and the LED) is at the floor's top
LED_L, LED_W, LED_PCB, LED_H = 15.4, 10.3, 1.6, 3.11      # his calipers
LED_CHIP = 5.0
LED_HOLE = 5.4
LED_POCKET_TOP = 2.6    # board's top face against the pocket's roof: the LED stands 1.51 -> its face just above the floor
APEX_HALF = 4.0         # funnel's half width at the floor top
WIRE_D = 0.8            # bare 20 AWG
LANE_W, LANE_D = 1.0, 1.0
LANES = {"GND": -2.54, "data": 0.0, "5V": 2.54, "ROW": -8.5}  # y; the LED's pads GND | data | 5V at both its ends;
#                                                                ROW = the Hall's + (the Nano's row output)
GND_PAD_XY = (-6.4, -2.54)    # the LED segment's GND pad hole at its -x end, 1.3 in from the end (the Professor: use 1.3)
PLUS_HOLE_XY = (-4.0, -8.5)   # the Hall's + comes down here onto the ROW wire
FLOOR_HOLE_D = 1.2
INS_D = 1.2             # insulated column wire: 24 AWG hook-up
HALL_WIRE_D = 0.55      # the Hall's three wires: 30 AWG kynar (wire-wrap), side by side in a 1.6 groove
BUNDLE_D = 1.2          # the three of them together (stand-in)
COL_X = -19.0           # the file's COLUMN wire, along y: beside the corner hole, so the diode's cathode lead reaches it
COL_W, COL_D = 1.6, 2.4  # deeper than the rank channels: it crosses OVER the bare rank wires, 0.25 clear
DIODE_D, DIODE_L = 2.7, 5.2                       # DO-41 max body - measure his
LEAD_D = 0.8                                      # 1N4001 lead
HALL_HOLE_XY = (-21.77, -21.77)                   # down through the spider's pad and the bucket's solid corner
HALL_HOLE_D = 1.9                                 # VCC + GND (30 AWG kynar) + the anode lead; 0.5 wall each side
HALL_CH_X = HALL_HOLE_XY[0]                       # the Hall's wires along y on the underside, to their lanes
HALL_CH_W, HALL_CH_D = 2.2, COL_D

# ---------------------------------------------------------------- the frame (X with the Hall post)
SEAT_Z = 14.0           # the arms' underside, in the funnel's corners
ARM_H = 3.5             # deep enough for the diode's trough in the wire leg
SEAT = 7.0              # seat pocket, square, at each corner (room for the arm's end and the wire hole)
ARM_W = SEAT - 2 * GAP  # Mike (P1.F105, the seat floor): the spider's arms as wide as the seat, their ends fill it
POST = 3.6              # post cross-section
WIRE_GROOVE = 2.2       # the Hall's three wires side by side (3 x 30 AWG kynar), down the post and into the leg
# the wire leg, in its own frame: u along the leg from the centre toward the -x-y corner, v across it
LEG_ANG = 225.0
VG = (0.5, 1.8)         # VCC + GND groove across the leg
VT = (-1.45, 1.45)      # diode trough, centred on the leg (it fits the leg's neck by the corner pad)
VA = (-1.1, 0.1)        # the Hall's OUT lead and the diode's anode lead, side by side, from the post to the diode
DIODE_LEAD_MIN = 25.4   # DO-41 lead length, each end (1" min - measure his)
HALL_LEAD_MIN = 15.1    # TO-92 LPG lead length from the body (15.1 .. 15.5)
HALL_LEAD_T = 0.43
U_DIODE = 23.6          # the diode's anode end; stripe (cathode) at the far end, by the corner hole
ANODE_TO_U = 8.0        # the diode's anode lead, laid back to here: it overlaps the Hall's OUT lead (which ends at u 13.1) by 5
DROP_U, DROP_V, DROP_D = 3.4, 1.0, 2.0   # where + and GND drop through the leg, near the post

# ---------------------------------------------------------------- Hall: DRV5032FBLPGM, TO-92
HALL_W, HALL_L, HALL_T = 4.0, 3.15, 1.52

# derived
HALF = PITCH / 2
BUCKET_HALF = HALF - GAP
RING_OUT = BUCKET_HALF - WALL_TOP - GAP
RING_IN = RING_OUT - RING_W
COVER_HALF = RING_IN - GAP
Z_COVER = FIELD_D - COVER_T
Z_LIP = Z_COVER - LIP_T
Z_RING0 = Z_LIP - RING_BELOW
Z_HALL0 = Z_COVER - HALL_T
SLOPE = (RING_IN - APEX_HALF) / (Z_RING0 - FLOOR_T)     # funnel half-width per mm of height


def funnel_half(z):
    return APEX_HALF + (z - FLOOR_T) * SLOPE


SEAT_C = funnel_half(SEAT_Z) - 0.5                       # seat centre (|x| = |y|)
ARM_END = SEAT_C + 1.5                                   # arm reaches this |x| = |y|


def leg(u, v, z):
    """a point in the wire leg's frame -> the cell frame"""
    a = math.radians(LEG_ANG)
    return (u * math.cos(a) - v * math.sin(a), u * math.sin(a) + v * math.cos(a), z)


def leg_solid(s):
    return s.rotate(V(0, 0, 0), V(0, 0, 1), LEG_ANG)


U_HOLE = math.hypot(*HALL_HOLE_XY)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cbox(cx, cy, sx, sy, z0, z1):
    return box(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, z0, z1)


def one(shape, what):
    s = shape.Solids()
    assert len(s) == 1, f"{what}: {len(s)} solids"
    assert BRepCheck_Analyzer(s[0].wrapped).IsValid(), f"{what}: not valid"
    return s[0]


def frustum(h0, h1, z0, z1):
    w0 = cq.Wire.makePolygon([V(-h0, -h0, z0), V(h0, -h0, z0), V(h0, h0, z0), V(-h0, h0, z0)], close=True)
    w1 = cq.Wire.makePolygon([V(-h1, -h1, z1), V(h1, -h1, z1), V(h1, h1, z1), V(-h1, h1, z1)], close=True)
    return cq.Solid.makeLoft([w0, w1], True)


def bucket():
    b = box(-BUCKET_HALF, BUCKET_HALF, -BUCKET_HALF, BUCKET_HALF, 0, FIELD_D)
    b = b.cut(frustum(APEX_HALF, RING_IN, FLOOR_T, Z_RING0 + 0.001))                     # the funnel
    b = b.cut(cbox(0, 0, 2 * (RING_OUT + GAP), 2 * (RING_OUT + GAP), Z_RING0, FIELD_D + 1))  # ring pocket
    for sx in (1, -1):                                                                  # the frame's corner seats
        for sy in (1, -1):
            b = b.cut(cbox(sx * SEAT_C, sy * SEAT_C, SEAT, SEAT, SEAT_Z, Z_RING0 + 0.5))
    # ---- the underside
    b = b.cut(cbox(0, 0, LED_L + 0.4, LED_W + 0.4, -1, LED_POCKET_TOP))                  # LED segment, from below
    b = b.cut(cbox(0, 0, LED_HOLE, LED_HOLE, 0, FLOOR_T + 1))                           # the LED up into the funnel
    for y in LANES.values():
        b = b.cut(box(-HALF - 1, HALF + 1, y - LANE_W / 2, y + LANE_W / 2, -1, LANE_D))
    b = b.cut(box(COL_X - COL_W / 2, COL_X + COL_W / 2, -HALF - 1, HALF + 1, -1, COL_D))
    hx, hy = HALL_HOLE_XY
    b = b.cut(cq.Solid.makeCylinder(HALL_HOLE_D / 2, SEAT_Z + 1, V(hx, hy, -1), V(0, 0, 1)))
    b = b.cut(box(hx, COL_X, hy - 0.7, hy + 0.7, -1, 1.6))                             # cathode lead over to the column
    gx, gy = GND_PAD_XY                                    # the Hall's GND, down into the LED's GND pad hole
    b = b.cut(cq.Solid.makeCylinder(FLOOR_HOLE_D / 2, 10, V(gx, gy, LED_POCKET_TOP - 0.01), V(0, 0, 1)))
    px, py = PLUS_HOLE_XY                                  # the Hall's +, down onto the ROW wire
    b = b.cut(cq.Solid.makeCylinder(FLOOR_HOLE_D / 2, 11, V(px, py, -1), V(0, 0, 1)))
    return one(b.clean(), "bucket")


def frame():
    """the spider: two arms corner to corner, as wide as the corner seats, their square ends filling the seats; the
    post at the crossing holds the Hall; kept 0.1 clear of the funnel and the seat walls"""
    top = SEAT_Z + ARM_H
    arms = None
    for ang in (45, 135):
        L = 2 * (SEAT_C + SEAT / 2) * math.sqrt(2)
        a = cbox(0, 0, L, ARM_W, SEAT_Z, top).rotate(V(0, 0, 0), V(0, 0, 1), ang)
        arms = a if arms is None else arms.fuse(a)
    pads = [cbox(sx * SEAT_C, sy * SEAT_C, SEAT - 2 * GAP, SEAT - 2 * GAP, SEAT_Z, top) for sx in (1, -1) for sy in (1, -1)]
    f = arms
    for q in pads:
        f = f.fuse(q)
    # stay inside the light box: the funnel (narrowest at the arms' underside) plus the four seats
    h = funnel_half(SEAT_Z) - GAP
    room = cbox(0, 0, 2 * h, 2 * h, SEAT_Z - 1, top + 1)
    for q in pads:
        room = room.fuse(q.translate(V(0, 0, 0)))
    f = f.intersect(room)
    f = f.fuse(leg_solid(cbox(0, 0, POST, POST, top - 0.01, Z_HALL0)))            # the post, turned with the Hall
    # the wire leg (-x-y): three wires down the post's face into the leg; + and GND drop through a hole near the
    # post; OUT on into the diode (trough); the diode's cathode lead along a groove into the corner hole
    cuts = [box(POST / 2 - 0.8, POST / 2 + 1, -WIRE_GROOVE / 2, WIRE_GROOVE / 2, top - 1.0, Z_HALL0 + 1),
            box(0.8, DROP_U + 1.0, -WIRE_GROOVE / 2, WIRE_GROOVE / 2, top - 1.0, top + 1),
            box(0.8, U_DIODE, VA[0], VA[1], top - 1.2, top + 1),
            box(U_DIODE - 1.0, U_HOLE + 0.5, VT[0], VT[1], top - DIODE_D - 0.2, top + 1)]
    for q in cuts:
        f = f.cut(leg_solid(q))
    f = f.cut(cq.Solid.makeCylinder(DROP_D / 2, ARM_H + 2, V(*leg(DROP_U, DROP_V, SEAT_Z - 1)), V(0, 0, 1)))
    hx, hy = HALL_HOLE_XY
    f = f.cut(cq.Solid.makeCylinder(HALL_HOLE_D / 2, ARM_H + 2, V(hx, hy, SEAT_Z - 1), V(0, 0, 1)))
    return one(f.clean(), "frame")


def ring():
    r = cbox(0, 0, 2 * RING_OUT, 2 * RING_OUT, Z_RING0, FIELD_D)
    r = r.cut(cbox(0, 0, 2 * RING_IN, 2 * RING_IN, Z_RING0 - 1, FIELD_D + 1))
    lip_in = RING_IN - LIP_W
    lip = cbox(0, 0, 2 * RING_IN, 2 * RING_IN, Z_LIP, Z_COVER).cut(cbox(0, 0, 2 * lip_in, 2 * lip_in, Z_LIP - 1, Z_COVER + 1))
    return one(r.fuse(lip).clean(), "ring")


def cover():
    return one(cbox(0, 0, 2 * COVER_HALF, 2 * COVER_HALF, Z_COVER, FIELD_D), "cover")


# ---------------------------------------------------------------- stand-ins
def path(pts, d):
    segs = []
    for a, b in zip(pts, pts[1:]):
        a, b = V(*a), V(*b)
        segs.append(cq.Solid.makeCylinder(d / 2, (b - a).Length, a, (b - a).normalized()))
        segs.append(cq.Solid.makeSphere(d / 2, b, angleDegrees1=-90, angleDegrees2=90))
    s = segs[0]
    for q in segs[1:]:
        s = s.fuse(q)
    return s.clean()


def hall():
    h = cq.Workplane("XY").box(HALL_L, HALL_W, HALL_T).edges("|X and >Z").chamfer(0.55).val()
    return h.translate(V(0, 0, Z_HALL0 + HALL_T / 2)).rotate(V(0, 0, 0), V(0, 0, 1), LEG_ANG - 180)   # leads down the leg


Z_LEAD = Z_HALL0 + HALL_T / 2


def funnel_z(x, y):
    """height of the funnel's surface at (x, y)"""
    m = max(abs(x), abs(y))
    return FLOOR_T if m <= APEX_HALF else FLOOR_T + (m - APEX_HALF) / SLOPE


def hall_wires():
    """+, GND and OUT soldered to the Hall at the bench (30 AWG kynar, heat-shrunk), down the post's face into the
    wire leg; + and GND drop through the leg's hole and straight down through the floor - GND into the LED segment's
    GND pad hole, + onto the ROW wire - both soldered from below; OUT along the leg into the diode's anode"""
    top = SEAT_Z + ARM_H
    w = HALL_WIRE_D
    zg = top - 1.0 + w / 2
    pf = POST / 2 + 0.25
    gx, gy = GND_PAD_XY
    px, py = PLUS_HOLE_XY
    out = {}
    for k, v0, dv, end in (("+", 0.6, 0.3, [(px, py, funnel_z(px, py) + 0.3), (px, py, WIRE_D + w / 2 + 0.05)]),
                           ("GND", 0.0, -0.3, [(gx, gy, funnel_z(gx, gy) + 0.3),
                                               (gx, gy, LED_POCKET_TOP - LED_PCB + 0.2)])):
        d = leg(DROP_U, DROP_V + dv, zg)
        out[k] = path([leg(HALL_L / 2, v0, Z_LEAD), leg(pf, v0, Z_LEAD), leg(pf, v0, zg), d, (d[0], d[1], SEAT_Z - 0.5)]
                      + end, w)
    # OUT: the Hall's own lead, no extension - down the post, along the leg's groove as far as it goes
    zo = top - 1.2 + HALL_LEAD_T / 2
    pts = [leg(HALL_L / 2, -0.6, Z_LEAD), leg(pf, -0.6, Z_LEAD), leg(pf, -0.6, zo), leg(pf + 1.0, VA[0] + 0.3, zo)]
    used = sum((V(*b) - V(*a)).Length for a, b in zip(pts, pts[1:]))
    pts.append(leg(pf + 1.0 + (HALL_LEAD_MIN - used), VA[0] + 0.3, zo))
    out["OUT"] = path(pts, HALL_LEAD_T)
    out["OUT_end_u"] = pf + 1.0 + (HALL_LEAD_MIN - used)
    return out


def led():
    z0 = LED_POCKET_TOP - LED_PCB
    pcb = cbox(0, 0, LED_L, LED_W, z0, LED_POCKET_TOP)
    chip = cbox(0, 0, LED_CHIP, LED_CHIP, LED_POCKET_TOP, LED_POCKET_TOP + (LED_H - LED_PCB))
    return pcb, chip


def diode():
    """the 1N4001 at the corner end of the wire leg: its anode lead runs back along the leg's groove beside the Hall's
    OUT lead (solder them where they overlap, at the bench); its cathode (stripe) lead goes straight down the corner
    hole and over to the column wire beside it - no extensions"""
    hx, hy = HALL_HOLE_XY
    top = SEAT_Z + ARM_H
    zc = top - DIODE_D / 2 - 0.15
    body = leg_solid(cq.Solid.makeCylinder(DIODE_D / 2, DIODE_L, V(U_DIODE, 0, zc), V(1, 0, 0)))
    band = leg_solid(cq.Solid.makeCylinder(DIODE_D / 2 + 0.02, 0.8, V(U_DIODE + DIODE_L - 1.2, 0, zc), V(1, 0, 0)))
    za = top - 1.2 + LEAD_D / 2
    va = VA[1] - LEAD_D / 2 - 0.05
    apts = [leg(U_DIODE, 0, zc), leg(U_DIODE - 1.0, va, za)]
    used = sum((V(*b) - V(*a)).Length for a, b in zip(apts, apts[1:]))
    apts.append(leg(ANODE_TO_U, va, za))          # back along the groove, 5 mm past the end of the Hall's OUT lead (trim the rest)
    anode = path(apts, LEAD_D)
    cpts = [leg(U_DIODE + DIODE_L, 0, zc), leg(U_HOLE, 0, zc), leg(U_HOLE, 0, 1.0),
            (COL_X, hy, 1.0), (COL_X, hy, COL_D - INS_D - 0.05)]
    cath = path(cpts, LEAD_D)
    lens = {"anode reaches u": U_DIODE - 1.0 - (DIODE_LEAD_MIN - used),
            "cathode used": sum((V(*b) - V(*a)).Length for a, b in zip(cpts, cpts[1:]))}
    return body.cut(band), band, cath, anode, lens


def wires():
    w = {k: cq.Solid.makeCylinder(WIRE_D / 2, PITCH, V(-HALF, y, WIRE_D / 2 + 0.05), V(1, 0, 0)) for k, y in LANES.items()}
    w["COLUMN (insulated)"] = cq.Solid.makeCylinder(INS_D / 2, PITCH, V(COL_X, -HALF, COL_D - INS_D / 2 - 0.05), V(0, 1, 0))
    return w


def main():
    parts = {"bucket": bucket(), "frame": frame(), "ring": ring(), "cover": cover()}
    pcb, chip = led()
    dbody, dband, dcath, danode, dlens = diode()
    hw = hall_wires()
    out_end = hw.pop("OUT_end_u")
    wr = wires()
    ref = {"Hall DRV5032FB (bevelled face up)": hall(), "LED segment PCB (pads down)": pcb, "LED WS2812B": chip,
           "diode 1N4001": dbody, "diode stripe (cathode)": dband, "diode anode lead": danode,
           "diode cathode lead": dcath}
    ref.update({f"Hall wire {k}": s for k, s in hw.items()})
    ref.update({f"wire {k}": s for k, s in wr.items()})

    # ---- report
    print(f"square {PITCH:g}; play surface {FIELD_D:g} above the underside; cover {2 * COVER_HALF:.1f} (1/8\" wood); "
          f"ring {RING_W:g} wide; line between rings {2 * (HALF - RING_OUT):.1f}")
    print(f"funnel {2 * APEX_HALF:g} at z {FLOOR_T:g} (the LED) -> {2 * RING_IN:.1f} at z {Z_RING0:.3f}; LED face z "
          f"{LED_POCKET_TOP + LED_H - LED_PCB:.2f}, {Z_COVER - (LED_POCKET_TOP + LED_H - LED_PCB):.1f} under the cover")
    print(f"frame: X arms {ARM_W:g} x {ARM_H:g} at z {SEAT_Z:g}, ends in corner seats at |x|=|y| {SEAT_C:.2f}; post "
          f"{POST:g} square (turned 45 with the Hall) to z {Z_HALL0:.3f}; wire leg -x-y: + and GND drop at the post, "
          f"diode trough {VT[1] - VT[0]:.1f} x {DIODE_D + 0.2:.1f} deep, hole {HALL_HOLE_D:g}")
    sl = parts["frame"].intersect(cbox(0, 0, 2 * HALF, 2 * HALF, SEAT_Z, SEAT_Z + ARM_H)).Volume() / ARM_H
    op = (2 * funnel_half(SEAT_Z)) ** 2
    print(f"spider covers {sl:.0f} of the funnel's {op:.0f} mm^2 opening at z {SEAT_Z:g} ({100 * sl / op:.0f}%; the "
          f"LED's light to the ring passes through the rest)")
    names = list(parts)
    bad = 0
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            v = parts[a].intersect(parts[b]).Volume()
            if v > 1e-3:
                bad += 1
                print(f"   OVERLAP {a} x {b}: {v:.3f}")
    for k, s in ref.items():
        for a in names:
            v = s.intersect(parts[a]).Volume()
            if v > 1e-3:
                bad += 1
                print(f"   OVERLAP {k} x {a}: {v:.3f}")
    # bare wires: which conductors touch which
    cond = {**{f"wire {k}": s for k, s in wr.items() if "insulated" not in k},
            "diode anode lead": danode, "LED PCB": pcb}
    for k in ("wire GND", "wire data", "wire 5V", "wire ROW"):
        for k2 in ("wire GND", "wire data", "wire 5V", "wire ROW"):
            if k < k2:
                d = cond[k].distance(cond[k2])
                if d < 0.5:
                    print(f"   CLOSE {k} - {k2}: {d:.2f}")
    print(f"   column wire over the bare rank wires: {min(wr['COLUMN (insulated)'].distance(wr[k]) for k in LANES):.2f} clear")
    print(f"   Hall + wire end to the ROW wire: {hw['+'].distance(wr['ROW']):.2f}; Hall GND wire into the LED's GND pad: "
          f"{hw['GND'].distance(pcb):.2f}")
    print(f"   no extensions: the Hall's OUT lead reaches u {out_end:.1f} along the leg; the diode's anode lead reaches back to "
          f"u {dlens['anode reaches u']:.1f} -> they overlap {out_end - dlens['anode reaches u']:.1f} mm to solder; the "
          f"cathode lead needs {dlens['cathode used']:.1f} of its {DIODE_LEAD_MIN:g}")
    print(f"   cathode lead to the column wire: {dcath.distance(wr['COLUMN (insulated)']):.2f}; OUT wire to the anode lead: "
          f"{hw['OUT'].distance(danode):.2f}; cathode lead clear of the bare rank wires: "
          f"{min(dcath.distance(wr[k]) for k in LANES):.2f}")
    print(f"   {'no overlaps' if not bad else str(bad) + ' overlaps'}")
    for a, s in parts.items():
        bb = s.BoundingBox()
        print(f"   {a:7s} {s.Volume() / 1000:6.2f} cm^3  {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")

    col = {"bucket": (0.95, 0.95, 0.93), "frame": (0.9, 0.9, 0.95), "ring": (0.75, 0.9, 1.0), "cover": (0.86, 0.72, 0.52)}
    a = cq.Assembly(name="square_cell")
    for k, s in parts.items():
        a.add(s, name=k, color=cq.Color(*col[k]))
    for k, s in ref.items():
        c = (0.15, 0.15, 0.17)
        if k.startswith("wire") or "lead" in k:
            c = (0.85, 0.55, 0.3)
        if "insulated" in k or k.startswith("Hall wire"):
            c = {"+": (0.85, 0.15, 0.15), "GND": (0.1, 0.1, 0.1), "OUT": (0.95, 0.85, 0.2)}.get(k.split()[-1], (0.2, 0.4, 0.9))
        if "stripe" in k:
            c = (0.8, 0.8, 0.82)
        if k == "LED WS2812B":
            c = (1.0, 1.0, 0.9)
        a.add(s, name=k, color=cq.Color(*c))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    write_step(a, OUT)
    print(OUT)
    # the printed parts on their own, for the Professor to edit (cell frame: z 0 = the bucket's underside)
    for k, fn in (("bucket", "bucket.step"), ("frame", "spider.step")):
        one_part = cq.Assembly(name=fn[:-5] + "_part")
        one_part.add(parts[k], name="spider" if k == "frame" else k, color=cq.Color(*col[k]))
        q = os.path.join(os.path.dirname(OUT), fn)
        write_step(one_part, q)
        print(q)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
gearbox_stand_adapter.py - adapter ring between the gearbox housing and the torque test stand Mike was already printing
when the arm screws moved (Mike 2026-11-15: "We'll need to print an adapter ring, one that screws onto the bottom of
the housing floor, and then I can screw it from the back side of the fixture").

    python gearbox_stand_adapter.py     -> print/Gearbox stand adapter ring.step (drive frame: axis z, housing bottom
                                           z 0, the ring under it)

The ring (OD x ID x T), printed flat, housing face down on the bed (the countersinks then face up):
  - 6 x #4 x 3/4" flat-head wood screws UP through it into the housing's arm screw pilots (cycloid.ARM_SCREW_*):
    D3.4 clearance, 82 deg countersink D6.0 in the ring's underside so the heads sit flush on the stand.
    Through T = 10 a 3/4" screw goes 9.05 into the housing (its pilots are 16 deep).
  - 6 x M3 pilots D2.5 for the stand's own countersunk M3s from behind, on the LID circle angles (1.58 + 60k):
    blind from the underside, ending 1.0 under the ring's top so a tip can never reach the lid screw tips in the
    housing above (z 0.7); 0.5 x 45 deg entry chamfer. Through the stand's 8 mm plate an M3 x 12..16 goes 4..8 in.
  - ID clears the NEMA 17 body (chamfered corners at r 26.7) by 1.3; OD 74 so the countersinks keep a rim.
The stand he is printing is the 2026-11-13 one (the arm session, 2026-11-15): 90 x 90 x 8 plate, 44 x 44 motor
opening, 6 x D3.4 with 90 deg countersinks D6.3 from behind on r 32.4 at the LID angles (1.58 + 60k) - right on the
ring's M3 pilots, gearbox as drawn. (The stand's later versions, 30 deg off / on ARM_SCREW_*, would need a quarter turn
or no ring at all.) The ring lifts the gearbox T off the stand's plate; the axis height on the bench stays the same.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import cycloid as C  # noqa: E402
import nema17 as N  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402

OUT = os.path.join(os.path.dirname(HERE), "print", "Gearbox stand adapter ring.step")
OD, ID, T = 74.0, 56.0, 10.0
CLEAR, CSK = 3.4, 6.0            # #4 flat head: 0.112" shank, head 0.212..0.225" (5.4..5.7)
M3_PILOT, M3_TOP_SKIN, LEAD = 2.5, 1.0, 0.5
NAME = (f"Gearbox stand adapter ring ({OD:g} x {ID:g} x {T:g}, 6 x #4 countersunk up into the housing, "
        f"6 x M3 pilots for the stand)")


def ring():
    body = cq.Solid.makeCylinder(OD / 2, T, V(0, 0, -T)).cut(cq.Solid.makeCylinder(ID / 2, T + 2, V(0, 0, -T - 1)))
    k = (CSK - CLEAR) / 2 / math.tan(math.radians(41))        # 82 deg countersink depth
    for a in C.ARM_SCREW_ANGLES:                                 # #4 up into the housing
        x, y = C.polar(C.ARM_SCREW_R, a)
        body = body.cut(cq.Solid.makeCylinder(CLEAR / 2, T + 2, V(x, y, -T - 1)))
        body = body.cut(cq.Solid.makeCone(CSK / 2, CLEAR / 2, k, V(x, y, -T - 0.001)))
    rp = M3_PILOT / 2
    for a in C.LID_SCREW_ANGLES:                                 # the stand's M3s from behind
        x, y = C.polar(C.LID_SCREW_R, a)
        body = body.cut(cq.Solid.makeCylinder(rp, T - M3_TOP_SKIN + 1, V(x, y, -T - 1)))
        body = body.cut(cq.Solid.makeCone(rp + LEAD + 0.5, rp, LEAD + 0.5, V(x, y, -T - 0.5)))
    return body.clean()


def main():
    C.set_spread(14.0)
    r = ring()
    assert len(r.Solids()) == 1 and r.isValid()
    # checks: the motor clears; walls; the screws
    body, caps = N.load()
    caps = N.trimmed_caps(caps, 0.0)
    ov = r.intersect(body).Volume() + r.intersect(caps).Volume()
    box = cq.Solid.makeBox(120, 120, T, V(-60, -60, -T))       # the motor within the ring's height (its top end cap)
    slab = [s.intersect(box) for s in (body, caps)]
    mr = max(math.hypot(v.X, v.Y) for s in slab for v in s.Vertices())
    dmin = min(abs((a - b + 180) % 360 - 180) for a in C.ARM_SCREW_ANGLES for b in C.LID_SCREW_ANGLES)
    gap = 2 * C.LID_SCREW_R * math.sin(math.radians(dmin) / 2) - CSK / 2 - (M3_PILOT / 2 + LEAD)
    print(f"ring {OD:g} x {ID:g} x {T:g}: {r.Volume():.0f} mm3; motor overlap {ov:.3f} (body out to r {mr:.2f} "
          f"in the ring's height, ID r {ID / 2:g})")
    print(f"  walls: M3 pilot to OD {OD / 2 - C.LID_SCREW_R - M3_PILOT / 2:.2f}, to ID {C.LID_SCREW_R - M3_PILOT / 2 - ID / 2:.2f}; "
          f"countersink to OD {OD / 2 - C.ARM_SCREW_R - CSK / 2:.2f}, to ID {C.ARM_SCREW_R - CSK / 2 - ID / 2:.2f}; "
          f"#4 countersink to the nearest M3 chamfer {gap:.2f} ({dmin:.2f} deg apart)")
    print(f"  #4 x 3/4\" through the ring: {19.05 - T:.2f} into the housing (pilots {C.ARM_PILOT_TOP:g} deep); "
          f"M3 from the stand's 8 mm plate: x12 -> {12 - 8:g} in, x16 -> {16 - 8:g} in, x18 would reach "
          f"{18 - 8 - (T - M3_TOP_SKIN):+.1f} past the pilot top (pilot {T - M3_TOP_SKIN:g} deep)")
    write_flat([(NAME, r, None)], OUT)
    print(OUT)


if __name__ == "__main__":
    main()

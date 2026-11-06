#!/usr/bin/env python3
"""
arm_tube.py - the arm with Mike's tube links (2026-11-05: "replace all of the arm sections in our robot assembly with
these new arm sections"). Builds the base for J1 the same way arm_base.py does with the box link: one tube link's
DRIVE (elbow) end on J1, the rest turned to point down (+y), cut off on the link's hub wall, on the same round foot,
with the spread gearbox (mockup_link.py's drive) at J1.

    python arm_tube.py

  - the link: complete assembly/mockups/mockup_link_plus.step (mockup_link.py) - the cut lands exactly on its hub
    wall's outer face (link x -41.5), so the cup, neck and outer hub gussets go and nothing thin is left
  - foot: as arm_base.py (swivel 72T gear OD 108.73, the 20 wire hole in the middle), now FOOT_T thick so J1 stays
    168 above the foot's underside as before (board, pieces, IK board_y all unchanged); the wire hole goes on through
    the link's hub wall into the tube. 4 x #8 countersunk near the diagonals (arm_base.foot_holes_tube(), Mike
    2026-11-05 "rotate the mounting holes 45 degrees"): (+-35.1, +-31.8) from the axis, r 47.4 - the turntable's
    pilots moved with them (swivel_base.py), 13.95 deep there: #8 x 7/8" screws. The two on the -z side sit under
    the J1 hub (128 mm up): a stubby screwdriver.
  - servo mount (Mike 2026-11-05): a tab straight out the back from the foot (underside 3 mm up, over the swivel
    cover's roof as it turns - print it supported), a vertical mount plate on the column's flat back face and two
    gussets beside the servo box; the box (servo_tube.py --on base) bolts flat on the plate, 4 x M3 flat heads into
    2.5 pilots. Layout: servo_tube.tab_layout().
  - drive J1: mockup_drive_spread.step (housing), turned the same 90 deg as the link, with Mike's J1 motor: a NEMA 17
    60 mm long (2 A, 73 N.cm; nema17.py --length 60) in place of the 48 mm one - same flange, bolts and 5 mm D shaft
  - the old box-link base: python arm_base.py rebuilds complete assembly/arm_base.step (recycled 2026-11-06).
Out: complete assembly/arm_base_tube.step (base body, drive J1 ...).
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402
from cadquery import Vector as V  # noqa: E402

import arm_base as AB  # noqa: E402
import servo_tube as ST  # noqa: E402
from view_step import read_step  # noqa: E402
from regroup_assembly import write_step  # noqa: E402

ROOT = os.path.dirname(HERE)
LINK = os.path.join(ROOT, "complete assembly", "mockups", "mockup_link_plus.step")
DRIVE = os.path.join(ROOT, "complete assembly", "mockups", "mockup_drive_spread.step")
OUT = os.path.join(ROOT, "complete assembly", "arm_base_tube.step")

LINK_HUB_FACE = -41.5            # the tube link's hub wall, outer face (link frame)
J1_MOTOR_L = 60.0                # Mike (2026-11-05): J1's NEMA 17 is the 60 mm one (73 N.cm)
DRIVE_MOTORS = (23.0,)           # gearbox models with other motor lengths: J4 has his 42 x 23 pancake (15.5 N.cm);
#                                  J2 / J3 keep the 48 (mockup_drive_spread.step), the swivel its own 48
Y_CUT = 200.0 + LINK_HUB_FACE   # 158.5 below J1 (the elbow, link x -200, sits on J1)
J1_HEIGHT = AB.Y_CUT + AB.FOOT_T  # 168: J1 above the foot's underside, as with the box-link base
FOOT_T = J1_HEIGHT - Y_CUT       # 9.5
assert abs(ST.FOOT_Y - Y_CUT) < 1e-9 and abs(ST.MOUNT_X - AB.TUBE_COLUMN_X) < 1e-9
# J0 home switch (Mike 2026-11-05, "the sensor in the bottom of the case under the servo drive", his P3.F94): the
# DRV5032FB breakout in a pocket in the servo tab's underside, on the arm's centre plane, swivel_base.HOME_R out from
# the yaw axis (under the servo's inner end), chip side down; the magnet is in the swivel cover's roof (swivel_base.py).
# Its wires: a channel in the foot's underside straight in to the foot's D20 wire hole and down the turntable's
# spindle with the rest (the turntable closes the channel; lay the wires in before the foot goes on).
HALL_POCKET = (9.0, 11.0, 3.0)   # along the arm (radial), across, deep: the ~8 x 10 x 2.7 breakout, chip down
HALL_BOARD = (8.0, 10.0, 1.6)    # the breakout shown (PCB), pressed to the pocket's ceiling
HALL_CHIP = (1.6, 2.9, 1.1)      # SOT-23 under it; its Hall element ~0.6 in from the package's face
HALL_WIRE = (4.0, 3.5)           # the wire channel: wide, deep (from the foot's underside)
# J1 home switch (Mike 2026-11-05, his P3.F16 = the column's tube skin, P1.F20 = link A's round end plate): the
# half-capsule lobe with the breakout's pocket and the wire slot is part of the tube link (mockup_link.py HALL_*, at its
# elbow end under the next cup) - so the column, cut from that link, has it at its top, right under J1 and link A's cup.
# The TO-92 is shown here (j1_hall_parts); its wires go down the drop and out a D16 side hole (top pair: y 51, z 15.5).


def foot_screws():
    """the 4 foot screws' clearance holes + countersinks (cut last, so nothing built on the foot covers them)"""
    k = (AB.CSK_D - AB.SCREW_CLEAR) / 2 / 0.8693
    out = []
    for x, z in AB.foot_holes_tube():
        out.append(cq.Solid.makeCylinder(AB.SCREW_CLEAR / 2, FOOT_T + 2, V(x, Y_CUT - 1, z), V(0, 1, 0)))
        out.append(cq.Solid.makeCone(AB.CSK_D / 2, AB.SCREW_CLEAR / 2, k, V(x, Y_CUT, z), V(0, 1, 0)))
    return out


def foot():
    return cq.Solid.makeCylinder(AB.FOOT_D / 2, FOOT_T, V(0, Y_CUT, AB.Z_MID), V(0, 1, 0))


def servo_mount():
    """Mike (2026-11-05): "on the outside of [the foot] extend a tab straight out the back ... put a couple of gussets on
    it to support the vertical mount for the servo". The vertical mount plate stands on the foot against the column's
    flat back face (fused to both); the tab runs straight out the back from the foot under the servo box, its underside
    raised to clear the swivel cover's roof as it turns; a gusset each side of the box, from the plate out to the tab's
    end. Layout from servo_tube.tab_layout() (the box's own numbers). Returns (solid, the bolt pilots to cut)."""
    L = ST.tab_layout()
    (px0, px1), (py0, py1), (z0, z1) = L["plate"]
    (tx0, tx1), (ty0, ty1), _ = L["tab"]
    m = AB.box(px0, px1, py0, py1 + 0.5, z0, z1)                       # on the column's face, 0.5 into the foot
    m = m.fuse(AB.box(tx0, tx1, ty0, ty1, z0, z1))
    for g0, g1 in L["gussets"]:                     # 1 into the plate and the tab, the tip 1 short of the tab's end
        tri = [(px1 - 1.0, ty0 + 1.0), (tx1 - 1.0, ty0 + 1.0), (px1 - 1.0, ty0 - L["gusset_up"])]
        m = m.fuse(cq.Workplane("XY", origin=(0, 0, g0)).polyline(tri).close().extrude(g1 - g0).val())
    pilots = [cq.Solid.makeCylinder(ST.M3_PILOT / 2, ST.M3_PILOT_DEPTH + 1, V(px1 + 1, y, z), V(-1, 0, 0))
              for y, z in L["pilots"]]                                  # in the plate only, 0.3 short of the column
    wy, wz, wd = L["wire_hole"]                                         # the wires: through the plate and the
    pilots.append(cq.Solid.makeCylinder(wd / 2, px1 + 1 - 18.0, V(px1 + 1, wy, wz), V(-1, 0, 0)))   # back wall, in
    return m, pilots


def hall_cuts():
    """the J0 Hall breakout's pocket in the tab's underside and its wire channel in to the foot's wire hole"""
    import swivel_base as S
    r, (pl, pw, pd), (ww, wd) = S.HOME_R, HALL_POCKET, HALL_WIRE
    under = ST.TAB_UNDER
    pocket = AB.box(r - pl / 2, r + pl / 2, under - pd, under + 1, AB.Z_MID - pw / 2, AB.Z_MID + pw / 2)
    chan = AB.box(AB.WIRE_D / 2 - 2, r - pl / 2 + 1, J1_HEIGHT - wd, J1_HEIGHT + 1, AB.Z_MID - ww / 2, AB.Z_MID + ww / 2)
    return [pocket, chan]


def hall_parts():
    """the breakout shown in its pocket: (PCB, chip), and the Hall element's point (design frame, J0 unposed)"""
    import swivel_base as S
    r, under, pd = S.HOME_R, ST.TAB_UNDER, HALL_POCKET[2]
    (bl, bw, bt), (cl, cw, ct) = HALL_BOARD, HALL_CHIP
    top = under - pd                                                   # the pocket's ceiling
    pcb = AB.box(r - bl / 2, r + bl / 2, top, top + bt, AB.Z_MID - bw / 2, AB.Z_MID + bw / 2)
    chip = AB.box(r - cl / 2, r + cl / 2, top + bt, top + bt + ct, AB.Z_MID - cw / 2, AB.Z_MID + cw / 2)
    return pcb, chip, (r, top + bt + ct - 0.6, AB.Z_MID)


def j1_hall_parts():
    """the J1 TO-92, as every link carries it (mockup_link.hall_parts) placed with the column: (body, leads, the Hall
    element's point, design frame)"""
    import mockup_link as ML
    pcb, chip, (ex, ey, ez) = ML.hall_parts()
    e = AB.place(cq.Vertex.makeVertex(ex, ey, ez)).Center()
    return AB.place(pcb), AB.place(chip), (e.x, e.y, e.z)


def base_parts():
    link = [s for _, s, _ in read_step(LINK)]
    assert len(link) == 1, "the tube link STEP should hold one solid"
    keep = AB.box(-300, 300, -300, Y_CUT, -100, 200)
    mount, pilots = servo_mount()
    body = AB.place(link[0]).intersect(keep).fuse(foot()).fuse(mount)
    wire = cq.Solid.makeCylinder(AB.WIRE_D / 2, FOOT_T + 6, V(0, Y_CUT - 5, AB.Z_MID), V(0, 1, 0))  # into the tube
    for cu in [wire] + pilots + foot_screws() + hall_cuts():
        body = body.cut(cu)
    body = AB.one(body.clean(), "base body")
    import nema17 as N17
    drive = [(n, s) for n, s, _ in read_step(DRIVE) if not n.startswith("NEMA 17")]
    body60, caps60 = N17.load(J1_MOTOR_L)
    drive += [(f"NEMA 17 {J1_MOTOR_L:g} mm", body60),
              (f"NEMA 17 {J1_MOTOR_L:g} mm end caps", N17.trimmed_caps(caps60, 0.0))]   # shaft flush, as the stand-in
    drive = [(n, s.rotate(V(0, 0, 0), V(0, 0, 1), 90)) for n, s in drive]
    return body, drive


def drive_with_motor(length):
    """the spread gearbox with a NEMA 17 of another length (nema17.py --length), shaft flush as in the stand-in"""
    import nema17 as N17
    parts = [(n, s) for n, s, _ in read_step(DRIVE) if not n.startswith("NEMA 17")]
    body, caps = N17.load(length)
    parts += [(f"NEMA 17 {length:g} mm", body), (f"NEMA 17 {length:g} mm end caps", N17.trimmed_caps(caps, 0.0))]
    out = os.path.join(ROOT, "complete assembly", "mockups", f"mockup_drive_spread_m{length:g}.step")
    a = cq.Assembly(name=f"mockup_drive_spread_m{length:g}")
    cols = {"mockup drive (spread 6808s)": (0.75, 0.75, 0.78)}
    for n, p in parts:
        a.add(p, name=n, color=cq.Color(*cols.get(n, (0.12, 0.12, 0.13))))
    write_step(a, out)
    return out


def main():
    out = OUT
    if "--out" in sys.argv:                                     # a folder (to check before replacing the real one)
        out = os.path.join(sys.argv[sys.argv.index("--out") + 1], os.path.basename(OUT))
    else:
        for length in DRIVE_MOTORS:
            print(drive_with_motor(length))
    body, drive = base_parts()
    bb = body.BoundingBox()
    print(f"base body {body.Volume() / 1000:.1f} cm^3, x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f}"
          f" z {bb.zmin:.1f}..{bb.zmax:.1f}; foot {FOOT_T:.1f} thick, J1 {J1_HEIGHT:.0f} above its underside")
    for n, d in drive:
        v = d.intersect(body).Volume()
        if v > 0.5:
            print(f"   OVERLAP drive J1 {n} x base body: {v:.1f} mm^3")
    L = ST.tab_layout()
    (px0, px1), (py0, py1), (z0, z1) = L["plate"]
    print(f"servo mount: plate x {px0}..{px1}, design y {py0:.1f}..{py1} ({Y_CUT + FOOT_T - py0:.1f} above the foot's "
          f"underside), z {z0:.2f}..{z1:.2f}; tab to x {L['x_end']:.2f} (r "
          f"{max((L['x_end'] ** 2 + (zz - AB.Z_MID) ** 2) ** 0.5 for zz in (z0, z1)):.1f} at its corners), underside "
          f"{L['tab'][1][1] - Y_CUT - FOOT_T:+.1f} from the foot's; gussets z {L['gussets']}")
    # the foot screws: their countersinks (and a screwdriver, r 3.5) must stay clear of what was built on the foot
    k = (AB.CSK_D - AB.SCREW_CLEAR) / 2 / 0.8693
    mount, _ = servo_mount()
    for x, z in AB.foot_holes_tube():
        head = cq.Solid.makeCone(AB.CSK_D / 2, AB.SCREW_CLEAR / 2, k, V(x, Y_CUT, z), V(0, 1, 0))
        driver = cq.Solid.makeCylinder(3.5, 150, V(x, Y_CUT - 150, z), V(0, 1, 0))
        print(f"   foot screw ({x:.1f}, {z:.1f}) r {((x ** 2 + (z - AB.Z_MID) ** 2) ** 0.5):.1f}: countersink "
              f"{head.distance(mount):.2f} from the mount, a screwdriver {driver.distance(mount):.2f}; column in the "
              f"driver's way: {driver.intersect(body).Volume():.1f} mm^3")
    pcb, chip, elem = hall_parts()
    for n, s in (("Hall breakout", pcb), ("Hall chip", chip)):
        v = s.intersect(body).Volume()
        if v > 0.01:
            print(f"   OVERLAP {n} x base body: {v:.2f} mm^3")
    print(f"J0 Hall: element at design ({elem[0]:.2f}, {elem[1]:.2f}, {elem[2]:.2f}) (J0 unposed), "
          f"{ST.TAB_UNDER - elem[1]:.2f} above the tab's underside; pocket {HALL_POCKET}, wires {HALL_WIRE}")
    a = cq.Assembly(name="arm_base_tube")
    a.add(body, name="base body", color=cq.Color(0.45, 0.6, 0.85))
    a.add(pcb, name="J0 Hall breakout", color=cq.Color(0.1, 0.42, 0.22))
    a.add(chip, name="J0 Hall DRV5032FB", color=cq.Color(0.08, 0.08, 0.09))
    pcb1, chip1, elem1 = j1_hall_parts()
    for n, s in (("J1 Hall TO-92", pcb1), ("J1 Hall leads", chip1)):
        v = s.intersect(body).Volume()
        if v > 0.01:
            print(f"   OVERLAP {n} x base body: {v:.2f} mm^3")
    print(f"J1 Hall: element at design ({elem1[0]:.2f}, {elem1[1]:.2f}, {elem1[2]:.2f}), "
          f"{elem1[1] - 38.0:.2f} under the column's top (from the link's lobe)")
    a.add(pcb1, name="J1 Hall DRV5032FB", color=cq.Color(0.08, 0.08, 0.09))
    a.add(chip1, name="J1 Hall leads", color=cq.Color(0.78, 0.79, 0.81))
    for n, d in drive:
        a.add(d, name=f"drive J1 {n}", color=cq.Color(0.75, 0.75, 0.78))
    write_step(a, out)
    print(out)


if __name__ == "__main__":
    main()

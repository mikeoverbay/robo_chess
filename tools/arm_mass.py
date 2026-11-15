#!/usr/bin/env python3
"""
arm_mass.py - what the arm really weighs, part by part, and what that does to the joints' gravity load
(Mike 2026-11-12: "Calculations for this arm take into account all of the pins, bearings, screws, the nuts. Plastic,
all of it." - motor_speed.py used one lumped mass per assembly; this counts every solid).

    python arm_mass.py

Every solid in the STEP files the arm is built from, in the scene's zero pose (links straight out along -x):
  links     complete assembly/mockups/mockup_link_plus.step (the same body as print/Arm (print 3).step), PLA
  drives    cycloid_spread/step/assembly_steel_pins.step: the printed set (nylon), 38 ring + 8 output steel dowels,
            2 x 6808, 2 x 6800, 4 motor screws, 6 lid screws, 3 output screws + their inserts, the motor (its net
            catalogue weight covers the shaft). J2 / J3: the 48 mm 17HS19-2004S1; J4: the 23 mm 17HE08-1004S pancake.
            The file's "output disc (stand-in)" is the next link's hub - counted with the link, not here.
  claw      gripper/claw_on_wrist_spread.step + park_dock/claw_flange_on_wrist.step: PLA + steel pins, the PC4-M6
            fitting (brass), its collet, the home magnet
  sensors   the Hall chips and home magnets on the links (scene models)
  a king    viewer_session/piece_king.step (king + pedestal, PLA) + 3 pennies (pedestal.py) + its 6 x 1.5 magnet
  cables    in no STEP: motor cables, Hall leads and the claw's Bowden along the links - ESTIMATED per metre (CABLE_G_M)
Densities g/cm^3: PLA 1.24, nylon 1.10 (as cycloid.py), steel 7.85, brass 8.5, NdFeB 7.5, POM 1.41. Plastic is taken
SOLID (an upper bound) and x0.85 "as printed" (3-4 walls, as mockup_link.py; the discs print solid either way).
Bought parts by catalogue: 17HS19-2004S1 390 g (StepperOnline net), 6808 29 g (retailers 28.6..30), 6800 5.5 g,
17HE08-1004S 200 g (StepperOnline lists only its SHIPPING weight - an upper bound; weigh yours).
Each part sits at its own centre of mass in its joint's frame; then motor_speed.py's worst-pose search (every square
and tray spot, 0 / 60 / 125 up, claw vertical) with these masses in place of its lumped ones.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402

import board_pieces as BP  # noqa: E402
import motor_speed as MS  # noqa: E402
from view_step import read_step  # noqa: E402

ROOT = os.path.dirname(HERE)
GEARBOX = os.path.join(ROOT, "cycloid_spread", "step", "assembly_steel_pins.step")
KING = os.path.join(HERE, "viewer_session", "piece_king.step")
RHO = {"PLA": 1.24, "nylon": 1.10, "steel": 7.85, "brass": 8.5, "NdFeB": 7.5, "POM": 1.41, "chip": 1.5}   # g/cm^3
FILL = 0.85                                  # plastic as printed (3-4 walls); the discs print solid
MOTOR_G = {"48 mm (17HS19-2004S1)": 390.0, "23 mm pancake (15.5 N.cm)": 200.0}
BEARING_G = {"6808": 29.0, "6800": 5.5}
PENNIES_G, PED_MAGNET_G = 3 * 2.5, math.pi * 3.0 ** 2 * 1.5 / 1000 * 7.5   # pedestal.py: 3 US pennies, a 6 x 1.5 magnet
# cables (g per metre, ESTIMATED): a stepper's 4 x 22 AWG lead, a Hall chip's 3 x 28 AWG lead, the claw's Bowden
# (PTFE 4 x 2 at 2.2 g/cm^3 = 20.7 g/m + a 1 mm steel inner 6.2 g/m); each run 0.22 m along a link (200 + slack)
CABLE_G_M = {"motor": 15.0, "hall": 5.0, "bowden": 27.0}
RUN_M = 0.22
# Mike's scale (2026-11-12), replacing the calculated grams for that lot (the parts keep their own centres of mass):
MEASURED = {("gearbox hardware", "48 mm (17HS19-2004S1)"): 154.0}   # one elbow / J3 gearbox's bearings, dowels,
#                                                    screws, inserts (calculated 149.4); the wrist's not weighed yet
GEARBOX_PRINTED = ("housing floor", "housing ring", "pin ring", "lid", "disc 1", "disc 2", "eccentric", "eccentric spacer", "output flange",
                   "output support ring", "clamp ring", "outer race spacer", "inner race spacer")


def material(name):
    """(what, density or None for a catalogue part, catalogue grams or None, prints solid)"""
    n = name.lower()
    if "output disc" in n and "stand-in" in n:
        return None                                        # the next link's hub: counted with the link
    if n.startswith("motor shaft"):
        return None                                        # in the motor's net weight
    if n.startswith("motor body"):
        return ("motor", None, None, False)
    for b, g in BEARING_G.items():
        if n.startswith(b) or f" {b}" in n.split(":")[0]:
            return ("bearing", None, g, False)
    if "magnet" in n:
        return ("NdFeB", RHO["NdFeB"], None, True)
    if "insert" in n or "pc4-m6" in n:
        return ("brass", RHO["brass"], None, True)
    if "dowel" in n or "screw" in n or "drive pin" in n:
        return ("steel", RHO["steel"], None, True)
    if "collet" in n:
        return ("POM", RHO["POM"], None, True)
    if "hall" in n:
        return ("chip", RHO["chip"], None, True)
    if n in GEARBOX_PRINTED:
        return ("nylon", RHO["nylon"], None, n.startswith("disc"))
    return ("PLA", RHO["PLA"], None, False)


def tally(path, solids=None, offset=(0, 0, 0), motor=None):
    """[(name, what, g solid, g printed, centre of mass (x, y, z) in the scene)] for every solid in a STEP file"""
    out = []
    for i, (nm, s, _) in enumerate(read_step(path)):
        if solids is not None and i not in solids:
            continue
        m = material(nm)
        if m is None:
            continue
        what, rho, cat, solid = m
        c = cq.Shape.centerOfMass(s)
        at = (c.x + offset[0], c.y + offset[1], c.z + offset[2])
        if what == "motor":
            g = gp = MOTOR_G[motor]
        elif cat is not None:
            g = gp = cat
        else:
            g = s.Volume() / 1000.0 * rho
            gp = g if (solid or what not in ("PLA", "nylon")) else g * FILL
        out.append((nm, what, g, gp, at))
    return out


def groups(sc):
    """the four moving groups (beyond J1 .. beyond J4): [(joint name, joint at, [parts])], the drives' mockups swapped
    for the real gearbox (on its joint's axis) and the cables added at each link's middle"""
    models = {m["name"]: m for m in sc["models"] if isinstance(m, dict)}
    joints = {j["name"]: j for j in sc["joints"]}
    drive_motor = {"drive J2 (x -200)": MS.JOINT_MOTOR["J2 elbow"], "drive J3 (x -400)": MS.JOINT_MOTOR["J3"],
                   "drive J4 (x -600)": MS.JOINT_MOTOR["J4 wrist"]}
    real = {}
    out = []
    runs = {"J1 shoulder": 3, "J2 elbow": 2, "J3": 1}      # motors / Hall chips whose leads run along this link
    for jn in ("J1 shoulder", "J2 elbow", "J3", "J4 wrist"):
        j = joints[jn]
        parts = []
        for mn in j["models"]:
            m = models[mn]
            off = m.get("offset", (0, 0, 0))
            if mn in drive_motor:                          # the real gearbox, on the next joint's axis
                mot = drive_motor[mn]
                if mot not in real:
                    real[mot] = tally(GEARBOX, motor=mot)
                    hw = [q for q in real[mot] if q[1] in ("bearing", "steel", "brass")]
                    if ("gearbox hardware", mot) in MEASURED:   # his weighed total, in the calculated proportions
                        k = MEASURED[("gearbox hardware", mot)] / sum(q[2] for q in hw)
                        real[mot] = [(nm, w, g * k, gp * k, at) if w in ("bearing", "steel", "brass") else
                                     (nm, w, g, gp, at) for nm, w, g, gp, at in real[mot]]
                axis = (off[0], off[1])
                parts += [(f"{mn}: {nm}", w, g, gp, (axis[0] + at[0], axis[1] + at[1], at[2]))
                          for nm, w, g, gp, at in real[mot]]
            else:
                parts += [(f"{mn}: {nm}", w, g, gp, at) for nm, w, g, gp, at in
                          tally(m["path"], m.get("solids"), off)]
        if jn in runs:                                     # the cables along this link, at its middle
            x = j["at"][0] - 100.0
            n = runs[jn]
            for kind, k in (("motor", n), ("hall", n), ("bowden", 1)):
                g = CABLE_G_M[kind] * RUN_M * k
                parts.append((f"cables along {jn}'s link: {k} x {kind} (ESTIMATE)", "cable", g, g, (x, 0.0, 0.0)))
        out.append((jn, j["at"], parts))
    return out


def king():
    s = read_step(KING)[0][1]
    return s.Volume() / 1000.0 * RHO["PLA"] + PENNIES_G + PED_MAGNET_G


def torques(ik, rng, T, grp, king_g, printed):
    """gravity torque N.m at J1..J4 holding the arm at grip point T (design frame), claw vertical: every part at its
    own centre of mass. Scene zero pose -> arm plane: u = -(x - joint x) along the link, v = -y (up when the link
    points out); a link group turns with its link (s1, s2, s3), the claw group is held at wrist_sum (claw vertical)."""
    (J1, J2, J3, J4), _ = MS.arm(ik, rng, T)
    js = (J1, J2, J3, J4)
    ang = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(js, js[1:])] + [math.radians(ik["wrist_sum"])]
    pts = []                                               # (kg, x in the arm plane, the first joint it is beyond)
    for k, (jn, at, parts) in enumerate(grp):
        c, s = math.cos(ang[k]), math.sin(ang[k])
        for nm, w, g, gp, p in parts:
            u, v = -(p[0] - at[0]), -(p[1] - at[1])
            pts.append(((gp if printed else g) / 1000.0, js[k][0] + u * c - v * s, k + 1))
    grip_x = J4[0] + ik["tool"][0]
    pts.append((king_g / 1000.0, grip_x, 4))
    return [MS.G * sum(m * (x - Jk[0]) for m, x, beyond in pts if beyond >= k) / 1000.0
            for k, Jk in enumerate(js, start=1)]


def worst(fn):
    spots = [BP.square_at(f + str(r)) for f in "abcdefgh" for r in range(1, 9)]
    spots += BP.tray_spots("white") + BP.tray_spots("black")
    w = [0.0] * 4
    for p in spots:
        for up in (0.0, 60.0, 125.0):
            w = [max(a, abs(b)) for a, b in zip(w, fn([p[0], p[1] - up, p[2]]))]
    return w


def margins(w):
    out = []
    for n, t in zip(("J1 shoulder", "J2 elbow", "J3", "J4 wrist"), w):
        m = MS.JOINT_MOTOR[n]
        need = t / (MS.RATIO * MS.EFF)
        top = 0
        for rpm in range(10, 2001, 5):
            if MS.pullout(m, rpm) >= MS.MARGIN * need:
                top = rpm
            else:
                break
        out.append((n, t, MS.pullout(m, 100) / need, top))
    return out


def main():
    sc = json.load(open(MS.SCENE, encoding="utf-8"))
    ik = sc["ik"]
    rng = {j["name"]: (j["min"], j["max"]) for j in sc["joints"]}
    grp = groups(sc)
    kg = king()
    print("Every part beyond the shoulder (g: solid / as printed):")
    by_what = {}
    for jn, at, parts in grp:
        tot = sum(g for _, _, g, _, _ in parts)
        totp = sum(gp for _, _, _, gp, _ in parts)
        print(f"\n  beyond {jn}: {tot:7.1f} / {totp:7.1f} g")
        rows = {}
        for nm, w, g, gp, p in parts:
            key = nm.split(":")[0] + ": " + (w if w not in ("PLA", "nylon") else f"{w} parts")
            r = rows.setdefault(key, [0, 0.0, 0.0])
            r[0] += 1
            r[1] += g
            r[2] += gp
            by_what.setdefault(w, [0.0, 0.0])
            by_what[w][0] += g
            by_what[w][1] += gp
        for key, (n, g, gp) in rows.items():
            print(f"     {key:62s} x{n:3d} {g:7.1f} / {gp:7.1f}")
    allg = sum(v[0] for v in by_what.values())
    allp = sum(v[1] for v in by_what.values())
    print(f"\n  + a king in the claw: {kg:.1f} g (PLA king + pedestal, 3 pennies, magnet)")
    print(f"  the moving arm: {allg:.0f} g solid / {allp:.0f} g as printed, + the king")
    print("  by material: " + ", ".join(f"{w} {v[0]:.0f}" + (f"/{v[1]:.0f}" if abs(v[1] - v[0]) > 0.5 else "")
                                       for w, v in sorted(by_what.items(), key=lambda x: -x[1][0])))
    old = [MS.M_LINK, MS.M_DRIVE, MS.M_LINK, MS.M_DRIVE, MS.M_LINK, MS.M_WRIST_DRIVE[MS.JOINT_MOTOR["J4 wrist"]], MS.M_CLAW]
    print(f"  (motor_speed.py's lumped masses: {sum(old) * 1000:.0f} g + a {MS.M_KING * 1000:.0f} g king)")

    w_old = worst(lambda T: MS.loads(ik, rng, T))
    w_s = worst(lambda T: torques(ik, rng, T, grp, kg, False))
    w_p = worst(lambda T: torques(ik, rng, T, grp, kg, True))
    print("\nWorst gravity torque at the output, N.m, and the motor's margin (pull-out at 100 rpm over the load; the "
          f"top rpm that keeps x{MS.MARGIN}):")
    print(f"  {'':12s} {'lumped (motor_speed.py)':>32s} {'every part, solid plastic':>32s} {'every part, as printed':>32s}")
    for a, b, c in zip(margins(w_old), margins(w_s), margins(w_p)):
        cell = lambda r: f"{r[1]:6.2f}  x{r[2]:5.2f}  {r[3]:4d} rpm"   # noqa: E731
        print(f"  {a[0]:12s} {cell(a):>32s} {cell(b):>32s} {cell(c):>32s}")


if __name__ == "__main__":
    main()

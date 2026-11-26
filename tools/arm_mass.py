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
            J4 since 2026-11-23: herringbone/step/assembly.step (tools/herringbone_planetary.py, 5.25:1) as the scene
            shows it - its own pancake, 2 x 6808, 6800, 6 x 3x6x2, dowels, bolts; the printed set PLA, the spider PA12.
  claw     gripper/claw_on_wrist_spread.step + park_dock/claw_flange_on_wrist.step: PLA + steel pins, the PC4-M6
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
RHO = {"PLA": 1.24, "nylon": 1.10, "steel": 7.85, "brass": 8.5, "NdFeB": 7.5, "POM": 1.41, "chip": 1.5,
       "aluminium": 2.70, "belt": 1.3}   # g/cm^3
FILL = 0.85                                  # plastic as printed (3-4 walls); the discs print solid
MOTOR_G = {"48 mm (17HS19-2004S1)": 390.0, "23 mm pancake (15.5 N.cm)": 200.0,
           "60 mm (73 N.cm, 2 A)": 500.0}     # 17HS24-2104S: StepperOnline / Oyostepper 500 g net (2026-11-22)
# catalogue grams (the models are solid rings); the chain arm's added 2026-11-22: 608 12, 625 5, 6000 19, 6801 6,
# AXK1024 + 2 AS1024 washers ~7.5 a set (modelled as two halves), the nut carrier's 3x6x2 guide bearings 0.3
BEARING_G = {"6808": 29.0, "6800": 5.5, "6801": 6.0, "6000": 19.0, "608": 12.0, "625": 5.0, "axk1024": 3.75,
             "3x6x2": 0.3}
MOTOR_BY_NAME = {"nema 17 60 mm": "60 mm (73 N.cm, 2 A)", "nema 17 48 mm": "48 mm (17HS19-2004S1)",
                 "nema 17 23 mm": "23 mm pancake (15.5 N.cm)"}
# the chain-drive arm (2026-11-22, Mike: "the correct expected accel and decel we can get out of that at speeds"): main
# shows each link's frame, motor and idlers; its screw, nut + carrier, motor pulley and chains move, so they are their
# own models in the chaindrive studio - counted from there, with the link that carries them
CHAINDRIVE = os.path.join(HERE, "viewer_sessions", "chaindrive", "scene.json")
MOVING_PREFIX = {"J1 shoulder": "chain drive J1 compact (on link A) ", "J2 elbow": "chain drive J2 (link B) ",
                 "J3": "chain drive J3 (link C) "}
# the side covers (not modelled yet; 74 M3 shoulder screws wait for them): 4 mm printed, both sides, ~165 x 80 each -
# ESTIMATED at the link's middle until they are drawn
COVER_G = 2 * 165 * 80 * 4.0 / 1000 * RHO["PLA"] * FILL
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
# the J4 herringbone planetary (2026-11-23, herringbone_planetary.py; its STEP names parts in snake_case): printed PLA,
# the spider PA12 (nylon); its motor is the 17HE08 pancake
HERRINGBONE_PRINTED = ("motor plate", "ring half a", "ring half b", "retainer", "input shaft", "planet 1", "planet 2",
                       "planet 3", "output disc")


def material(name):
    """(what, density or None for a catalogue part, catalogue grams or None, prints solid)"""
    n = name.lower().replace("_", " ")                     # herringbone_planetary.py's names are snake_case
    if n in HERRINGBONE_PRINTED:
        return ("PLA", RHO["PLA"], None, False)
    if n == "spider":
        return ("nylon", RHO["nylon"], None, False)
    if n.startswith("motor 17he08"):
        return ("motor", None, None, False, "23 mm pancake (15.5 N.cm)")
    if "output disc" in n and "stand-in" in n:
        return None                                        # the next link's hub: counted with the link
    if n.startswith("motor shaft"):
        return None                                        # in the motor's net weight
    if n.startswith("motor body"):
        return ("motor", None, None, False)
    if n.startswith("nema 17"):
        if "end caps" in n:
            return None                                    # in the motor's net weight
        return next((("motor", None, None, False, mot) for k, mot in MOTOR_BY_NAME.items() if n.startswith(k)), None)
    for b, g in BEARING_G.items():
        if n.startswith(b) or f" {b}" in n.split(":")[0]:
            return ("bearing", None, g, False)
    if "magnet" in n:
        return ("NdFeB", RHO["NdFeB"], None, True)
    if "insert" in n or "pc4-m6" in n:
        return ("brass", RHO["brass"], None, True)
    h = n.split("(")[0]                                    # the part itself, not its "(where / what for)" note
    if "printed" in n:
        return ("PLA", RHO["PLA"], None, False)            # e.g. the loose guide washers
    if "belt" in h:
        return ("belt", RHO["belt"], None, True)
    if ("pulley" in h or "carrier" in h or "sprocket" in h) and "screw" not in h:
        return ("aluminium", RHO["aluminium"], None, True)  # GT2 pulleys, the nut carrier, his CNC-cut plate sprockets
    if ("dowel" in h or "screw" in h or "bolt" in h or "tensioner angle" in h or "drive pin" in h or "chain" in h or "nut" in h or "washer" in h
            or "pin " in h or "shaft" in h or "e-clip" in h or "collar" in h or "snap ring" in h or "spacer 10" in h):
        return ("steel", RHO["steel"], None, True)          # ball screw + nut, chains, pins, shafts, nuts, washers
    if "collet" in n:
        return ("POM", RHO["POM"], None, True)
    if "hall" in n:
        return ("chip", RHO["chip"], None, True)
    if n in GEARBOX_PRINTED:
        return ("nylon", RHO["nylon"], None, n.startswith("disc"))
    return ("PLA", RHO["PLA"], None, False)


OWN_K2 = {}                                    # (part name, centre) -> its own k^2 (mm^2) about z: bounding box, > 20 g only


def tally(path, solids=None, offset=(0, 0, 0), motor=None):
    """[(name, what, g solid, g printed, centre of mass (x, y, z) in the scene)] for every solid in a STEP file"""
    out = []
    for i, (nm, s, _) in enumerate(read_step(path)):
        if solids is not None and i not in solids:
            continue
        m = material(nm)
        if m is None:
            continue
        what, rho, cat, solid = m[:4]
        c = cq.Shape.centerOfMass(s)
        at = (c.x + offset[0], c.y + offset[1], c.z + offset[2])
        if what == "motor":
            g = gp = MOTOR_G[m[4] if len(m) > 4 else motor]
        elif cat is not None:
            g = gp = cat
        else:
            g = s.Volume() / 1000.0 * rho
            gp = g if (solid or what not in ("PLA", "nylon")) else g * FILL
        if gp > 20.0:                                      # a long part's own spread (a frame: m L^2 / 12)
            bb = s.BoundingBox()
            OWN_K2[(nm, at)] = (bb.xlen ** 2 + bb.ylen ** 2) / 12.0
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
    chain = any("chain_drive" in models[mn].get("path", "") for mn in joints["J1 shoulder"]["models"])
    cd_models = json.load(open(CHAINDRIVE, encoding="utf-8"))["models"] if chain else []
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
        if chain and jn in MOVING_PREFIX:                  # the chain drive's moving parts, with their link
            for cm in cd_models:
                if cm["name"].startswith(MOVING_PREFIX[jn]):
                    parts += [(f"{cm['name']}: {nm}", w, g, gp, at) for nm, w, g, gp, at in
                              tally(cm["path"], cm.get("solids"), cm.get("offset", (0, 0, 0)))]
            parts.append((f"{jn}'s link: side covers, 4 mm both sides (ESTIMATE, not drawn yet)", "PLA",
                          COVER_G / FILL, COVER_G, (j["at"][0] - 100.0, 0.0, 30.6)))
        if jn in runs:                                     # the cables along this link, at its middle
            x = j["at"][0] - 100.0
            n = runs[jn]
            for kind, k in (("motor", n), ("hall", n), ("bowden", 1)):
                g = CABLE_G_M[kind] * RUN_M * k
                parts.append((f"cables along {jn}'s link: {k} x {kind} (ESTIMATE)", "cable", g, g, (x, 0.0, 0.0)))
        out.append((jn, j["at"], parts))
    return out


def swivel_parts(sc):
    """what turns with the swivel but is not the arm (turntable, yoke, servo, the J1 sprockets): [(name, what, g, g
    printed, centre)] - for the yaw inertia"""
    models = {m["name"]: m for m in sc["models"] if isinstance(m, dict)}
    j = next(j for j in sc["joints"] if j["name"] == sc["ik"]["yaw"])
    parts = []
    for mn in j["models"]:
        m = models[mn]
        parts += [(f"{mn}: {nm}", w, g, gp, at) for nm, w, g, gp, at in
                  tally(m["path"], m.get("solids"), m.get("offset", (0, 0, 0)))]
    return j["at"], parts


def bodies(grp, king_g):
    """each moving group as one body in its link's frame, for motor_speed.py and the viewer's planner: [{joint (1 = link
    A .. 4 = the claw), m kg, at [u, v] mm (u along the link from its joint, v up, zero pose), r mm (radius of
    gyration about its own centre, in the arm plane)}]. Each turns with the sum of the joint angles up to its own,
    the claw group too (held claw-vertical that sum is wrist_sum, as torques() has it); the king rides at the grip."""
    out = []
    for k, (jn, at, parts) in enumerate(grp):
        m = sum(gp for _, _, _, gp, _ in parts) / 1000.0
        uv = [(gp / 1000.0, -(p[0] - at[0]), -(p[1] - at[1]), OWN_K2.get((nm.split(": ", 1)[-1], p), 0.0))
              for nm, _, _, gp, p in parts]
        cu = sum(w * u for w, u, _, _ in uv) / m
        cv = sum(w * v for w, _, v, _ in uv) / m
        i = sum(w * ((u - cu) ** 2 + (v - cv) ** 2 + k2) for w, u, v, k2 in uv)   # kg mm^2 about its centre
        out.append(dict(joint=k + 1, name=jn, m=round(m, 4), at=[round(cu, 1), round(cv, 1)],
                        r=round(math.sqrt(i / m), 1)))
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
        m, ratio, eff = MS.drive(n)[:3]                    # the chain arm's drives once its bodies are in the scene
        need = t / (ratio * eff)
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

    bd = bodies(grp, kg)
    at0, sw = swivel_parts(sc)
    base_i = sum(gp / 1000.0 * ((p[0] - at0[0]) ** 2 + (p[2] - at0[2]) ** 2 + OWN_K2.get((nm.split(": ", 1)[-1], p), 0.0))
                 for nm, _, _, gp, p in sw) / 1e6
    print("\nAs bodies for the planner (as printed; u along the link from its joint, v up, r its radius of gyration):")
    for b in bd:
        print(f"  {b['name']:12s} {b['m']:6.3f} kg at u {b['at'][0]:6.1f} v {b['at'][1]:6.1f} mm, r {b['r']:5.1f} mm")
    print(f"  the swivel's own turning parts (turntable, yoke, servo, J1 sprockets): "
          f"{sum(q[3] for q in sw) / 1000:.3f} kg, {base_i:.4f} kg.m^2 about the yaw axis; a king {kg:.1f} g")
    if "--write" in sys.argv:                              # scene ik.motion.dynamics: motor_speed.py and the viewer
        dyn = sc["ik"].setdefault("motion", {}).setdefault("dynamics", {})
        dyn["bodies"] = bd
        dyn["bodies_why"] = ("2026-11-22 tools/arm_mass.py --write: every solid of the chain-drive arm at its own centre "
                             "(main's parts + the chaindrive studio's screws, nuts, pulleys and chains; covers and cables "
                             "ESTIMATED), one body per moving group in its link's frame. Replaces masses.link / drive / "
                             "wrist_drive / claw where present; masses.king stays. 2026-11-23: J4 is the herringbone "
                             "planetary (459 g as printed, was the cycloid's 597): housing with link C, carrier side "
                             "with the claw.")
        dyn.setdefault("masses", {})["king"] = round(kg / 1000.0, 4)
        dyn["base_i"] = round(base_i, 5)
        bak = os.path.join(os.path.dirname(MS.SCENE), "scene_before_arm_bodies_backup.json")
        if not os.path.exists(bak):
            with open(bak, "w", encoding="utf-8") as f:
                json.dump(json.load(open(MS.SCENE, encoding="utf-8")), f, indent=1)
        with open(MS.SCENE, "w", encoding="utf-8") as f:
            json.dump(sc, f, indent=1)
        print(f"wrote ik.motion.dynamics.bodies + base_i to {MS.SCENE} (backup {os.path.basename(bak)})")


if __name__ == "__main__":
    main()

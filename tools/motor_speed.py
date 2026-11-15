#!/usr/bin/env python3
"""
motor_speed.py - how fast each joint's motor can cruise (Mike, 2026-11-05: "push the cruising RPM as high as we can").

    python motor_speed.py

A stepper keeps up only while its pull-out torque at that speed beats the load with margin. So, per joint:
  load    the worst gravity torque the joint sees anywhere the arm works (every square and tray spot, down on the board,
          60 up and at the 125 carry height, through the viewer's IK, claw vertical), from the arm's masses (fea.py's
          model: drives 0.71 kg with a 48 mm motor, tube links 0.15, claw + adapter 0.28, a king 0.075, plus the servo
          box on link B), carried by the motor through the 37:1 printed cycloid at 65 % (the measured printed-cycloid
          efficiency, HowToMechatronics). The S-curve accelerations add almost nothing next to gravity (checked below).
  motor   pull-out torque against speed at 24 V: the driver holds the phase current until the supply can't push it in
          against the coil's resistance, its inductance (50 pole pairs) and the back-EMF; torque = k x Ke x current,
          with k set so the 48 mm 17HS19 gives the 0.40 N.m at 300 rpm used so far (holding 0.59 -> k 0.68).
  limit   the fastest rpm whose pull-out torque is >= MARGIN x the load, capped at GEARBOX_RPM (the printed cycloid has
          not been run fast; raise it after a bench run) and checked against the step rate.
The swivel has no gravity load; its speed is set by how hard the arm may be swung (wobble), not by its motor.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_pieces as BP  # noqa: E402

import studio  # noqa: E402
SCENE = studio.scene_path()          # this studio's scene (ROBOT_STUDIO; default main = viewer_session/scene.json)
V_SUPPLY = 24.0
MARGIN = 1.5                     # pull-out torque over the load
GEARBOX_RPM = 600.0              # input-speed cap for the printed cycloid until it is bench-tested faster
RATIO, EFF = 37.0, 0.65
MICROSTEP = 8
G = 9.81
# motors: holding N.m, rated A (rms), phase ohm, phase H, where the numbers come from
MOTORS = {
    "48 mm (17HS19-2004S1)": (0.59, 2.0, 1.6, 3.0e-3, "his datasheet drawing A0842 (photo 2026-10-25): 1.60 ohm"),
    "60 mm (73 N.cm, 2 A)": (0.73, 2.0, 1.6, 3.0e-3, "ESTIMATED from the 17HS24-2104S (1.6 ohm, 3 mH): check your label"),
    "23 mm pancake (15.5 N.cm)": (0.17, 1.0, 3.6, 4.0e-3, "his StepperOnline 17HE08-1004S: 17 N.cm, 1 A, 3.6 ohm, "
                                  "4.0 mH (StepperOnline listing); shaft D5 x 20 D-cut"),
    "40 mm (44 N.cm, 1.7 A)": (0.44, 1.7, 1.5, 2.8e-3, "his wrist motor (2026-11-06); R, L ESTIMATED as a typical 1.7 A "
                               "40 mm (17HS4401): check your label"),
}
JOINT_MOTOR = {"J1 shoulder": "60 mm (73 N.cm, 2 A)", "J2 elbow": "48 mm (17HS19-2004S1)",
               "J3": "48 mm (17HS19-2004S1)", "J4 wrist": "23 mm pancake (15.5 N.cm)"}   # (the 40 mm: see below)
# Mike (2026-11-06): the wrist gets a 40 mm, 1.7 A, 44 N.cm NEMA 17 instead of the 23 mm 17HS4023 pancake ("this should
# be able to handle it. But I don't know what that does to our arm being able to lift this extra weight").
# THEN (same day) he named his StepperOnline 17HE08-1004S pancake as the wrist motor -> J4 is back on the pancake;
# the 40 mm entry stays for comparison.
# Masses (kg) the joints carry (fea.py's model): a tube link, a gearbox with a 48 mm motor and its 6808s, the wrist
# gearbox with ITS motor, the claw + adapter, a king. The wrist gearbox: 0.50 with the pancake (~0.14 kg motor); a
# 40 mm NEMA 17 is ~0.28 kg -> 0.64.
M_LINK, M_DRIVE, M_CLAW, M_KING = 0.15, 0.71, 0.28, 0.075
M_WRIST_DRIVE = {"23 mm pancake (15.5 N.cm)": 0.50, "40 mm (44 N.cm, 1.7 A)": 0.64}
M_SERVO_ON_B = 0.0               # the claw servo rode on link B (0.11); since 2026-11-05 it's on the base (J0): no load
K_PO = 0.40 / 0.59               # pull-out vs holding, set by the 17HS19's 0.40 N.m at 300 rpm (below its corner speed)


def pullout(motor, rpm):
    """pull-out torque N.m at rpm (24 V, rated current)"""
    th, irms, r, l, _ = MOTORS[motor]
    ip = math.sqrt(2) * irms
    ke = th / ip                                # V.s/rad (peak per phase)
    w = rpm * 2 * math.pi / 60
    x = 50 * w * l                              # coil reactance at the electrical frequency
    # largest current i <= ip with (i r + ke w)^2 + (i x)^2 <= V^2
    a, b, c = r * r + x * x, 2 * r * ke * w, (ke * w) ** 2 - V_SUPPLY ** 2
    if c >= 0:
        return 0.0
    i = (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)
    return K_PO * ke * min(ip, i)


# ---------------------------------------------------------------- the arm's gravity loads (arm plane: x out, z up)
def loads(ik, rng, T):
    """output torques N.m at J1..J4 holding the arm at grip point T (design frame), claw vertical"""
    j, masses = arm(ik, rng, T)
    return [G * sum(m * (p[0] - Jk[0]) for m, p, beyond in masses if beyond >= k) / 1000.0
            for k, Jk in enumerate(j, start=1)]


def arm(ik, rng, T):
    """the arm at grip point T (design frame), claw vertical: joints J1..J4 in the arm plane (mm: x out from the yaw
    axis, z up from J1) and the masses [(kg, point, the first joint it is beyond: 1 = beyond J1 ...)]"""
    L = ik["link"]
    tool = ik["tool"]
    ax = ik["yaw_axis"]
    rho = math.hypot(T[0] - ax[0], T[2] - ax[1])
    wx, wz = rho - tool[0], -T[1] - tool[1]
    d, phi = math.hypot(wx, wz), math.atan2(wz, wx)
    lim = math.radians(min(-rng[ik["chain"][1]][0], -rng[ik["chain"][2]][0]))
    c = min(1.0, (d / L - 1) / 2)
    q = -math.acos(max(-1.0, c))
    if -q <= lim + 1e-9:
        q1, q2, q3 = phi - q, q, q
    else:
        q3 = -lim
        l2 = 2 * L * math.cos(lim / 2)
        cb = max(-1.0, (d * d - L * L - l2 * l2) / (2 * L * l2))
        b = -math.acos(min(1.0, cb))
        q1, q2 = phi - math.atan2(l2 * math.sin(b), L + l2 * math.cos(b)), b + lim / 2
    s1, s2, s3 = q1, q1 + q2, q1 + q2 + q3
    j = [(0.0, 0.0)]
    for s in (s1, s2, s3):
        j.append((j[-1][0] + L * math.cos(s), j[-1][1] + L * math.sin(s)))
    J1, J2, J3, J4 = j
    mid = lambda a, b, f=0.5: (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)   # noqa: E731
    claw_c = (J4[0] + tool[0] / 2, J4[1] + tool[1] / 2)
    grip = (J4[0] + tool[0], J4[1] + tool[1])
    masses = [  # (kg, point, the first joint it is beyond: 1 = beyond J1 ...)
        (M_LINK, mid(J1, J2), 1), (M_DRIVE, J2, 1),
        (M_LINK, mid(J2, J3), 2), (M_SERVO_ON_B, mid(J2, J3, 0.45), 2), (M_DRIVE, J3, 2),
        (M_LINK, mid(J3, J4), 3), (M_WRIST_DRIVE[JOINT_MOTOR["J4 wrist"]], J4, 3),
        (M_CLAW, claw_c, 4), (M_KING, grip, 4),
    ]
    return (J1, J2, J3, J4), masses


# ---------------------------------------------------------------- acceleration (Mike 2026-11-07: "speed up the
# acceleration on these arms ... it's really, really just takes forever to get up to speed")
# A joint may speed up as hard as its motor can while still keeping the same x MARGIN over the load it then carries:
#     pull-out(cruise rpm) >= MARGIN x [ (|gravity| + I_arm alpha) / (ratio eff) + J_rotor ratio alpha ]
# taken at the worst pose (every square and tray spot, down on the board, 60 up and the 125 carry height). I_arm is
# everything beyond the joint (point masses + each tube link's own m L^2 / 12). The swivel turns the whole arm: I about
# the yaw axis from the same masses' reach, plus the turning base (ESTIMATED).
ROTOR = {"48 mm (17HS19-2004S1)": 82e-7, "60 mm (73 N.cm, 2 A)": 120e-7,          # kg.m^2 (82 g.cm^2 = StepperOnline's
         "23 mm pancake (15.5 N.cm)": 20e-7, "40 mm (44 N.cm, 1.7 A)": 57e-7}      # 17HS19 figure; the others ESTIMATED)
SWIVEL = dict(motor="48 mm (17HS19-2004S1)", ratio=4.0, eff=0.90,                  # 18T:72T bevel, eff ESTIMATED
              base_i=0.010)                     # kg.m^2: turntable, column, J1's drive, claw servo near the axis (ESTIMATED)


def inertias(j, masses):
    """kg.m^2 about J1..J4 (everything beyond each) and about the yaw axis (the whole arm, base not included)"""
    rod = M_LINK * 0.2 ** 2 / 12
    out = []
    for k, Jk in enumerate(j, start=1):
        i = sum(m * ((p[0] - Jk[0]) ** 2 + (p[1] - Jk[1]) ** 2) for m, p, beyond in masses if beyond >= k) / 1e6
        out.append(i + rod * sum(1 for m, p, beyond in masses if m == M_LINK and beyond >= k))
    yaw = sum(m * p[0] ** 2 for m, p, beyond in masses) / 1e6 + rod * 3
    return out, yaw


def accel_limits(ik, rng, mot, margin=MARGIN):
    """{joint: (alpha deg/s^2 allowed at the worst pose, that pose, I there, the claw's sideways g for the swivel)}"""
    spots = [BP.square_at(f + str(r)) for f in "abcdefgh" for r in range(1, 9)]
    spots += BP.tray_spots("white") + BP.tray_spots("black")
    names = ["J1 shoulder", "J2 elbow", "J3", "J4 wrist"]
    worst = {}
    for p in spots:
        for up in (0.0, 60.0, 125.0):
            T = [p[0], p[1] - up, p[2]]
            j, masses = arm(ik, rng, T)
            tg = loads(ik, rng, T)
            ii, iyaw = inertias(j, masses)
            for n, g, i in zip(names, tg, ii):
                m = JOINT_MOTOR[n]
                rpm = mot.get(n, {}).get("v", 0) * RATIO / 6
                po = pullout(m, rpm) / margin
                a = (po - abs(g) / (RATIO * EFF)) / (i / (RATIO * EFF) + ROTOR[m] * RATIO)
                if n not in worst or a < worst[n][0]:
                    worst[n] = (a, T, i, g)
            m, r, e = SWIVEL["motor"], SWIVEL["ratio"], SWIVEL["eff"]
            rpm = mot.get(ik["yaw"], {}).get("v", 0) * r / 6
            iy = iyaw + SWIVEL["base_i"]
            a = (pullout(m, rpm) / margin) / (iy / (r * e) + ROTOR[m] * r)
            reach = (j[3][0] + ik["tool"][0]) / 1000.0
            if ik["yaw"] not in worst or a < worst[ik["yaw"]][0]:
                worst[ik["yaw"]] = (a, T, iy, reach)
    return {n: (math.degrees(a), T, i, x) for n, (a, T, i, x) in worst.items()}


def main():
    sc = json.load(open(SCENE, encoding="utf-8"))
    ik = sc["ik"]
    rng = {j["name"]: (j["min"], j["max"]) for j in sc["joints"]}
    spots = [BP.square_at(f + str(r)) for f in "abcdefgh" for r in range(1, 9)]
    spots += BP.tray_spots("white") + BP.tray_spots("black")
    worst = [0.0] * 4
    for p in spots:
        for up in (0.0, 60.0, 125.0):
            t = loads(ik, rng, [p[0], p[1] - up, p[2]])
            worst = [max(w, abs(v)) for w, v in zip(worst, t)]
    names = ["J1 shoulder", "J2 elbow", "J3", "J4 wrist"]
    mot = ik.get("motion", {})
    print(f"24 V, {MICROSTEP}x microstepping, 37:1 at {EFF:.0%}, margin x{MARGIN}, gearbox input cap {GEARBOX_RPM:.0f} rpm\n")
    print("pull-out torque N.m at the motor:")
    print("  rpm      " + "".join(f"{r:>7d}" for r in (100, 200, 300, 400, 500, 600, 800, 1000)))
    for m in MOTORS:
        print(f"  {m[:26]:26s}" + "".join(f"{pullout(m, r):7.3f}" for r in (100, 200, 300, 400, 500, 600, 800, 1000))
              + f"   ({MOTORS[m][4]})")
    print()
    new = {}
    for n, w in zip(names, worst):
        m = JOINT_MOTOR[n]
        need = w / (RATIO * EFF)                         # N.m at the motor
        top = 0.0
        for rpm in range(10, 2001, 5):
            if pullout(m, rpm) >= MARGIN * need:
                top = rpm
            else:
                break
        cap = min(top, GEARBOX_RPM)
        cur = mot.get(n, {}).get("v", 0) * RATIO / 6
        new[n] = cap * 6 / RATIO
        print(f"{n:12s} worst gravity {w:5.2f} N.m out -> {need:.3f} N.m at the motor; pull-out x{MARGIN} up to "
              f"{top:4.0f} rpm -> cruise {cap:4.0f} rpm = {new[n]:5.1f} deg/s out "
              f"(now {cur:4.0f} rpm, {mot.get(n, {}).get('v', 0)} deg/s); "
              f"low-speed margin x{pullout(m, 100) / need:.2f}; {cap * 200 * MICROSTEP / 60:5.0f} steps/s")
    # acceleration's share at the shoulder (arm stretched out): I alpha
    a1 = math.radians(mot.get("J1 shoulder", {}).get("a", 40))
    inertia = sum(m * r * r for m, r in ((0.15, .1), (0.71, .2), (0.15, .3), (0.11, .29), (0.71, .4), (0.15, .5),
                                         (0.5, .6), (0.355, .63)))
    print(f"\nshoulder: arm stretched out, I ~{inertia:.2f} kg.m^2 x {math.degrees(a1):.0f} deg/s^2 = {inertia * a1:.2f} N.m "
          f"(vs {worst[0]:.1f} N.m of gravity)")
    accel(sc, ik, rng, mot)
    return new


def accel(sc, ik, rng, mot):
    """print each joint's acceleration limit; --write puts them and the viewer's torque model in scene ik.motion"""
    lim = accel_limits(ik, rng, mot)
    print(f"\nacceleration, x{MARGIN} margin kept with gravity + inertia + rotor at the WORST pose, capped at "
          f"{MOTOR_ACC_CAP:g} rev/s^2 at the motor (printed cycloid / printed arm: raise after a bench run):")
    out = {}
    for n in [ik["yaw"], "J1 shoulder", "J2 elbow", "J3", "J4 wrist"]:
        a, T, i, x = lim[n]
        ratio = SWIVEL["ratio"] if n == ik["yaw"] else RATIO
        cap = MOTOR_ACC_CAP * 360 / ratio
        old = mot.get(n, {}).get("a", 40)
        # the scalar a = the worst pose's limit, never below today's (J1: gravity alone at full reach already eats the
        # margin, so its scalar stays and the viewer's pose-aware planner gives it more where the pose allows)
        out[n] = max(old, math.floor(min(a, cap) / 5) * 5)
        extra = (f"the claw ~{x * math.radians(out[n]) / G:.2f} g sideways at {x * 1000:.0f} mm out" if n == ik["yaw"]
                 else f"I {i:.3f} kg.m^2, gravity {x:.2f} N.m")
        print(f"  {n:12s} worst pose {a:7.0f} deg/s^2 (at {[round(v) for v in T]}; {extra}), cap {cap:4.0f} "
              f"-> a {out[n]:4.0f} (was {old})")
    return out


MOTOR_ACC_CAP = 40.0             # rev/s^2 at the motor (~ an Ender's X motor at 1600 mm/s^2): until bench-tested


def write_motion(sc, ik, rng, mot):
    """scene ik.motion: new scalar a's, each joint's torque figures and the arm's mass model (the viewer's planner)"""
    out = accel(sc, ik, rng, mot)
    for n, a in out.items():
        e = mot.setdefault(n, {})
        if n == ik["yaw"]:
            m, ratio, eff = SWIVEL["motor"], SWIVEL["ratio"], SWIVEL["eff"]
        else:
            m, ratio, eff = JOINT_MOTOR[n], RATIO, EFF
        rpm = e.get("v", 30) * ratio / 6
        e.update(a=a, ratio=ratio, eff=eff, pullout=round(pullout(m, rpm), 4), rotor=ROTOR[m],
                 a_cap=round(MOTOR_ACC_CAP * 360 / ratio, 1), motor=m)
    mot["dynamics"] = {
        "why": "2026-11-07 Mike: 'speed up the acceleration ... takes forever to get up to speed'. tools/motor_speed.py "
               "--write. The viewer's planner times each move with the arm's own dynamics along its path: every joint's "
               "torque = sum of m (a - g) . dp/dq over the masses below (+ rotor J R^2 eff q''), kept under "
               "pullout x ratio x eff / margin at every sample of the move; a joint's acceleration is never held below "
               "its scalar a (today's) nor above a_cap (motor rev/s^2 cap). Masses kg: links as two points (their own "
               "m L^2/12), drives at the joints, claw half way down the tool, a king at the grip.",
        "margin": MARGIN, "g": G,
        "masses": {"link": M_LINK, "drive": M_DRIVE, "wrist_drive": M_WRIST_DRIVE[JOINT_MOTOR["J4 wrist"]],
                   "servo_on_b": M_SERVO_ON_B, "claw": M_CLAW, "king": M_KING},
        "base_i": SWIVEL["base_i"],
    }
    bak = os.path.join(os.path.dirname(SCENE), "scene_before_accel_backup.json")
    if not os.path.exists(bak):
        with open(bak, "w", encoding="utf-8") as f:
            json.dump(json.load(open(SCENE, encoding="utf-8")), f, indent=1)
    with open(SCENE, "w", encoding="utf-8") as f:
        json.dump(sc, f, indent=1)
    print(f"wrote {SCENE} ik.motion (backup {os.path.basename(bak)})")


if __name__ == "__main__":
    if "--write" in sys.argv:
        _sc = json.load(open(SCENE, encoding="utf-8"))
        write_motion(_sc, _sc["ik"], {j["name"]: (j["min"], j["max"]) for j in _sc["joints"]}, _sc["ik"]["motion"])
    else:
        main()

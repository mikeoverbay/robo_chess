#!/usr/bin/env python3
"""
hall_homing.py - the home switches' trip spheres for Robot Studio's homing cycle (Mike 2026-11-05: "put a zone on each
one of our magnetic Hall effect switches ... make the other one a sphere and this one a sphere, and if the two spheres
touch, then it's in the zone ... seek home at a relatively moderate speed, once we hit home back off, slow down, and
home out just like you would on a CNC machine").

    python hall_homing.py          (prints the field study, writes scene.json "homing")

Each switch = a DRV5032FB Hall (omnipolar, trips at |B| >= ~4.8 mT, samples 5 times a second) passing sideways over
a 6 x 2 magnet (axially magnetised). The field: the magnet's side surface current as a stack of current loops
(elliptic integrals), the Hall reads B along its own axis (square to the package: vertical here). It trips where that
field reaches the threshold - a fixed distance from the magnet whatever way the Hall comes at it, so the viewer gives
the Hall's point and the magnet's top-face centre a sphere each, radius = half that distance: touching = made.

Switches so far:
  J0 swivel - Hall in the servo tab's underside (arm_tube.hall_parts), magnet in the right swivel cover's roof
              (swivel_base.home_magnet_xy) under it at J0's furthest left turn (the spare queen's corner spot).
  J1..J4    - Hall in a lobe on the column / each link's elbow end (mockup_link.py HALL_*), magnet in the next part's
              cup rim (mockup_link.py CUP_MAGNETS, claw_adapter.py J4_MAGNET_DEG) over it in the home pose.
Grade of his magnets unknown: N42 (Br 1.32 T) used; N35..N52 moves the trip point 0.15 mm.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
from scipy.special import ellipk, ellipe  # noqa: E402

SCENE = os.path.join(HERE, "viewer_session", "scene.json")
MAG_R, MAG_L = 3.0, 2.0          # 6 x 2
BR = 1.32                        # T, N42 (N35 1.20 .. N52 1.43)
BOP = 4.8                        # mT, DRV5032FB operate point (TI's typical; release is lower)
SAMPLE_HZ = 5.0                  # DRV5032FB: looks at the field 5 times a second
# The cycle (Mike 2026-11-05: "initial seek run full out, max feed we can get. And then once it hits the limit switch,
# we're going to feed back out at a little bit slower speed, and then we're going to creep back at even a little bit
# slower feed"; "if it gets within 0.01 millimeter, call it good"):
#   seek  as fast as the joint may go (ik.motion v), but no faster than (a) crosses the switch's zone in ZONE_SAFE Hall
#         reads (faster and it can slip through between two 5 Hz looks) and (b) stops - a read late plus braking at
#         the joint's acceleration - STOP_MARGIN short of the joint's limit past the switch;
# Then (Mike 2026-11-06: "Once we come in and touch the probe the first time ... reverse direction slowly ... continue
# on until it comes off. And then back up slower until it makes contact. But when we first hit it, we're overshooting it
# a long ways. And then we start going the other direction too fast ... first contact, we come back, we slow it down to
# about a quarter of the speed") - the passes back and forth went out; one CNC-style cycle:
#   seek  as above; on first contact a HARD stop (HARD_STOP x the joint's acceleration - that position isn't used, so
#         losing a step there costs nothing);
#   back  BACK_SHARE (a quarter) of the seek speed the other way, through the switch (if the seek carried right past it,
#         it makes contact again on the way) until it comes off, PULL_OFF more, stop;
#   in    toward it again: at the back speed up to where it last read clear, then CREEP - slow enough that a 5 Hz read
#         is at most TOL_MM late at the Hall - until it makes contact: home. (0.01 mm took 30-90 s a joint this way.)
ZONE_SAFE, STOP_MARGIN, BACK_SHARE, HARD_STOP, TOL_MM = 1.25, 3.0, 0.25, 4.0, 0.05
PULL_OFF, MAX_TRAVEL = 0.5, 200.0               # past where it comes off; give up after this much travel


def cycle(joint, d, trips_at, early, r_hall):
    """the per-switch speeds (deg/s) for the scene, and why"""
    sc = json.load(open(SCENE, encoding="utf-8"))
    J = {j["name"]: j for j in sc["joints"]}[joint]
    m = sc["ik"]["motion"][joint]
    per = 1.0 / SAMPLE_HZ
    room = abs((J["max"] if d > 0 else J["min"]) - trips_at) - STOP_MARGIN
    v_room = m["a"] * (-per + math.sqrt(per * per + 2 * room / m["a"]))
    v_zone = 2 * early / (ZONE_SAFE * per)
    seek = min(m["v"], v_zone, v_room)
    why = {m["v"]: "the joint's top speed", v_zone: "the switch's zone vs 5 Hz reads", v_room: "room to stop before the limit"}[seek]
    tol_deg = math.degrees(TOL_MM / r_hall)
    creep = 0.9 * tol_deg * SAMPLE_HZ
    back = BACK_SHARE * seek
    print(f"   {joint}: seek {seek:.1f} deg/s ({why}; top {m['v']}, zone {v_zone:.0f}, room {room:.0f} deg -> {v_room:.0f}), "
          f"hard stop {seek * per + seek * seek / (2 * HARD_STOP * m['a']):.1f} deg at most past first contact; back "
          f"{back:.1f}; creep {creep:.3f} deg/s ({TOL_MM} mm at the Hall, r {r_hall:.1f}) over the last "
          f"{back * per:.1f} deg (~{back * per / creep:.0f} s)")
    return {"seek": round(seek, 2), "back": round(back, 2), "creep": round(creep, 4), "hard_stop": HARD_STOP,
            "tol_mm": TOL_MM, "r_hall": round(r_hall, 3), "pull_off": PULL_OFF, "max_travel": MAX_TRAVEL}


def bz(rho, z, br=BR):
    """field along the magnet's axis (mT) at radius rho from that axis, z above its top face (mm); rho an array"""
    rho = np.atleast_1d(np.asarray(rho, float))
    zs = np.linspace(-MAG_L, 0.0, 401)[:, None]
    zz = z - zs
    rr = np.maximum(rho[None, :], 1e-9)
    den = (MAG_R + rr) ** 2 + zz ** 2
    m = 4 * MAG_R * rr / den
    f = (ellipk(m) + (MAG_R ** 2 - rr ** 2 - zz ** 2) / ((MAG_R - rr) ** 2 + zz ** 2) * ellipe(m)) / np.sqrt(den)
    return br * 1000 / (2 * np.pi) * np.trapezoid(f, dx=MAG_L / 400, axis=0)


def trip_rho(h, br=BR, bop=BOP):
    """how far out sideways the Hall trips at height h over the magnet's top face"""
    rho = np.linspace(0, 20, 4001)
    b = bz(rho, h, br)
    i = np.where(b >= bop)[0]
    return float(rho[i.max()]) if len(i) else 0.0, float(b.min())


def j0_switch():
    import arm_base as AB
    import arm_tube as AT
    import swivel_base as S
    _, _, elem = AT.hall_parts()                                   # design frame, J0 unposed
    y_wood = AB.Y_CUT + AB.FOOT_T + S.Z_PLATE0 + S.PLATE_T          # as swivel_base.main()
    mx, my = S.home_magnet_xy()
    top = S.COVER_ROOF_TOP - S.MAGNET_DEPTH + MAG_L                 # the magnet's top face (swivel Z)
    mag = (-mx, y_wood - top, AB.Z_MID - my)                        # swivel -> design (swivel_base.to_design)
    h = mag[1] - elem[1]
    print(f"J0: Hall element {tuple(round(v, 2) for v in elem)} (rides with J0), magnet top {tuple(round(v, 2) for v in mag)}"
          f" (fixed); {h:.2f} apart vertically")
    for br, g in ((1.20, "N35"), (1.32, "N42"), (1.43, "N52")):
        row = []
        for bop in (3.0, 4.8, 7.0):
            r, neg = trip_rho(h, br, bop)
            row.append(f"{bop} mT -> {r:.2f}")
        print(f"   {g}: peak {bz(0.0, h, br)[0]:.0f} mT, trips sideways at " + ", ".join(row) +
              f"; the field's reversed ring outside reaches {neg:.2f} mT (under the trip point: no false trip)")
    rho, _ = trip_rho(h)
    trip = math.hypot(rho, h)
    jmax = S.j0_reach_max()[0]
    r = math.hypot(elem[0], elem[2] - AB.Z_MID)
    early = math.degrees(2 * math.asin(rho / (2 * r)))
    print(f"   trip distance {trip:.2f} (sideways {rho:.2f} at {h:.2f} up); magnet centred at J0 {jmax:.2f}, Hall on r {r:.2f}"
          f" -> coming from below it trips at J0 {jmax - early:.2f} ({early:.2f} early)")
    return {"joint": "J0 swivel", "name": "J0 home",
            "sensor": {"on": "J0 swivel", "at": [round(v, 3) for v in elem], "what": "DRV5032FB, tab underside"},
            "magnet": {"on": None, "at": [round(v, 3) for v in mag], "what": "6 x 2 magnet, right cover roof"},
            "trip": round(trip, 3), "dir": 1, "centre_deg": round(jmax, 3), "trips_at_deg": round(jmax - early, 3),
            **cycle("J0 swivel", 1, jmax - early, early, r)}


# J1..J4 (Mike 2026-11-05: a Hall in a lobe at each joint, "6mm magnet into that face above the Hall effect switch ...
# do this to all the remaining joints"): each Hall looks radially at the next part's cup rim (r 35.25) from r 38.9;
# its magnet sits in that rim, flush, where the Hall is in the home pose. Seek direction: a single magnet in the middle of
# a joint's travel can only be found from one side, so each joint seeks toward the side the game uses most (Robot
# Studio, the seed-5 game, home pose included: J1 23.7..142.2, J2 / J3 -103.7..-22.5, J4 56.9..159.4).
GAME_RANGE = {"J0 swivel": (-95.4, 94.5), "J1 shoulder": (23.7, 142.2), "J2 elbow": (-103.7, -22.5),
              "J3": (-103.7, -22.5), "J4 wrist": (56.9, 159.4)}
CUP_DIR = {"J1 shoulder": 1, "J2 elbow": -1, "J3": -1, "J4 wrist": 1}   # each the way the arm closes up
#                                  (Mike 2026-11-05: the wrist went the wrong way at -1; + curls the claw in)
ORDER = ["J1 shoulder", "J2 elbow", "J3", "J4 wrist", "J0 swivel"]   # Mike 2026-11-05: "j0 to j3 then turn table"
#                                  (his J0..J3 = the scene's J1 shoulder .. J4 wrist; the turntable = scene J0)


def cup_switches():
    import arm_tube as AT
    import mockup_link as ML
    import claw_adapter as CA
    sc = json.load(open(SCENE, encoding="utf-8"))
    J = {j["name"]: j for j in sc["joints"]}
    _, _, h_link = ML.hall_parts()                                  # link frame (link A = design, J1 at 0)
    mags = [f for _, f in ML.magnet_parts()]                         # [J1 on link A, J2/J3 on links B, C]
    _, j4f = CA.j4_magnet(pocket=False)                              # claw adapter's drive frame (J4 at 0)
    defs = [("J1 shoulder", "J0 swivel", AT.j1_hall_parts()[2], mags[0], 0.0),
            ("J2 elbow", "J1 shoulder", h_link, mags[1], -200.0),
            ("J3", "J2 elbow", (h_link[0] - 200, h_link[1], h_link[2]), mags[1], -400.0),
            ("J4 wrist", "J3", (h_link[0] - 400, h_link[1], h_link[2]), j4f, -600.0)]
    out = []
    for joint, parent, sen, mag, off in defs:
        mag = (mag.x + off, mag.y, mag.z)
        at = J[joint]["at"]
        rs = math.hypot(sen[0] - at[0], sen[1] - at[1])              # the Hall's radius about the joint (the magnet's
        rm = math.hypot(mag[0] - at[0], mag[1] - at[1])              # turns with it)
        dz = sen[2] - mag[2]
        h = rs - rm
        rho, neg = trip_rho(h)
        trip = math.hypot(rho, h)
        home = J[joint].get("home", 0.0)
        early = math.degrees(math.acos((rs * rs + rm * rm + dz * dz - trip * trip) / (2 * rs * rm)))
        d = CUP_DIR[joint]
        lo, hi = GAME_RANGE[joint]
        zone = (home - early, home + early)
        bad = (lo, zone[0]) if d < 0 else (zone[1], hi)                # game positions it can't find home from
        print(f"{joint}: Hall r {rs:.2f}, magnet face r {rm:.2f} ({h:.2f} apart, {dz:+.2f} in z); trips {rho:.2f} "
              f"sideways -> sphere distance {trip:.2f}; made within +-{early:.1f} deg of {home} "
              f"({zone[0]:.1f}..{zone[1]:.1f}); seek {'+' if d > 0 else '-'}: from a game pose at "
              + (f"{bad[0]:.1f}..{bad[1]:.1f} it runs away from the magnet" if bad[1] > bad[0] else "any game pose it finds it"))
        out.append({"joint": joint, "name": f"{joint.split()[0]} home",
                    "sensor": {"on": parent, "at": [round(v, 3) for v in sen], "what": "DRV5032FB, lobe"},
                    "magnet": {"on": joint, "at": [round(v, 3) for v in mag], "what": "6 x 2 magnet, cup rim"},
                    "trip": round(trip, 3), "dir": d, "centre_deg": home,
                    "trips_at_deg": round(home - d * early, 3),
                    **cycle(joint, d, home - d * early, early, rs)})
    return out


def main():
    sw = cup_switches() + [j0_switch()]
    sw.sort(key=lambda s: ORDER.index(s["joint"]))
    sc = json.load(open(SCENE, encoding="utf-8"))
    sc["homing"] = {
        "why": "2026-11-05 Mike: the Home button runs a CNC-style homing cycle. tools/hall_homing.py: each switch's Hall "
               "point and magnet carry a sphere, radius trip/2 - touching = the Hall is made (trip = the distance at "
               "which a DRV5032FB, ~4.8 mT, switches over a 6 x 2 magnet). Joints without a switch yet go straight to "
               "their home angle first; then each switch: seek (dir) at seek deg/s until it reads made, back off at "
               "back until it reads clear + pull_off deg, creep back on at creep: where it reads made is home. The "
               "Hall reads the field sample_hz times a second, so it reports up to 1/sample_hz late. Then the arm goes "
               "to its home pose. Switches home in list order: shoulder, elbow, J3, wrist, then the turntable.",
        "sample_hz": SAMPLE_HZ,
        "park_first": [j["name"] for j in sc.get("joints", []) if j["name"] not in {s["joint"] for s in sw}],
        "switches": sw}
    json.dump(sc, open(SCENE, "w", encoding="utf-8"), indent=1)
    print("scene.json homing:", json.dumps(sc["homing"]["switches"]))


if __name__ == "__main__":
    main()

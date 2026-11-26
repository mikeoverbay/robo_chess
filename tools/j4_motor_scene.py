#!/usr/bin/env python3
"""
j4_motor_scene.py - the wrist (J4) motor in the main studio as a real NEMA 17 (Mike 2026-11-24: "Can we replace the
standard NEMA? The little motor on the claw with an actual model of a NEMA 17?" / "Looks like it's a 23mm body.").

The herringbone planetary's assembly (herringbone_planetary.py, the gearbox sessions' file - not touched here) carries
a plain block for its 17HE08 pancake ("motor_17HE08": 42.3 square, body z -23..0, D5 shaft to z +20). Here that block
is left out of the scene's "J4 herringbone (x -600)" model and his own NEMA 17 (nema17.py --length 23: the 42 x 23
pancake from his "NEMA 17 with T5 Pulley" STEP) stands in its place, front face where the block's is, shaft +z,
the shaft cut to the block's length, as a model of its own riding J3 with the gearbox housing.

2026-11-25 (the gearbox session, Mike's change): the motor hangs on standoffs and drives through a dog-bone coupling,
so its face is no longer at z 0 (now -22.2); the face is read off the block (bottom + the 23 body), not assumed.

    python j4_motor_scene.py          writes viewer_session/j4_motor_nema17_23.step, checks it against the assembly,
                                      and puts it in the main scene (scene.json, kept CRLF, written whole)
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402

import nema17 as N  # noqa: E402
from regroup_assembly import write_flat  # noqa: E402
from view_step import read_step  # noqa: E402

ROOT = os.path.dirname(HERE)
ASSY = os.path.join(ROOT, "herringbone", "step", "assembly.step")
SCENE = os.path.join(HERE, "viewer_session", "scene.json")
OUT = os.path.join(HERE, "viewer_session", "j4_motor_nema17_23.step")
HB = "J4 herringbone (x -600)"
NAME = "J4 motor NEMA 17 x 23 (x -600)"
BLOCK = "motor_17HE08"
BODY_L = 23.0


def main():
    assy = read_step(ASSY)
    i_block = next(i for i, (n, _, _) in enumerate(assy) if n == BLOCK)
    bb = assy[i_block][1].BoundingBox()
    face_z = round(bb.zmin + BODY_L, 3)                           # the block's front face (its body is BODY_L long)
    shaft_len = round(bb.zmax - face_z, 3)                        # face to the block's shaft tip
    body, caps = N.load(BODY_L)
    caps = N.trimmed_caps(caps, shaft_len)
    # the 17HE08's D-cut shaft, as the block has it (herringbone_planetary.nema17(): D5, flat 0.5 deep facing +x from
    # z 5 to the tip) - the input shaft's D bore fills in past that flat, so a round shaft runs into it
    r = 2.5
    caps = caps.cut(cq.Solid.makeBox(1.5, 2 * r + 2, shaft_len, cq.Vector(r - 0.5, -r - 1, 5.0))).clean()
    body, caps = (s.translate(cq.Vector(0, 0, face_z)) for s in (body, caps))   # face 0 -> the block's face
    print(f"the block's front face z {face_z:g}, shaft {shaft_len:g} long")
    for n, s in (("body", body), ("caps", caps)):
        b = s.BoundingBox()
        print(f"his NEMA 17 x {BODY_L:g} {n}: x {b.xmin:.2f}..{b.xmax:.2f} y {b.ymin:.2f}..{b.ymax:.2f} "
              f"z {b.zmin:.2f}..{b.zmax:.2f}")
    print(f"the block it replaces: x {bb.xmin:.2f}..{bb.xmax:.2f} y {bb.ymin:.2f}..{bb.ymax:.2f} z {bb.zmin:.2f}..{bb.zmax:.2f}")
    # against everything else in the assembly (the plate it bolts to, its 4 screws, the input shaft on its shaft)
    hits = []
    for i, (n, s, _) in enumerate(assy):
        if i == i_block:
            continue
        for mn, m in (("body", body), ("caps", caps)):
            v = m.intersect(s).Volume()
            if v > 0.05:
                hits.append((n, mn, v))
    print("overlaps with the assembly:", ", ".join(f"{n} x {mn} {v:.2f} mm^3" for n, mn, v in hits) or "none")
    col = lambda h: cq.Color(*(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)))  # noqa: E731
    write_flat([("NEMA 17 x 23 (J4, 17HE08 pancake)", body, col("#2a2c30")),
                ("NEMA 17 end caps (J4)", caps, col("#c5c8cd"))], OUT)
    print(OUT)

    sc = json.load(open(SCENE, encoding="utf-8"))
    hb = next(m for m in sc["models"] if m["name"] == HB)
    if i_block in hb.get("solids", []):
        hb["solids"] = [i for i in hb["solids"] if i != i_block]
    entry = {"path": OUT, "offset": list(hb.get("offset", [0, 0, 0])), "name": NAME, "by": "j4_motor_scene.py",
             "note": f"2026-11-24 Mike: \"an actual model of a NEMA 17\" for the wrist motor - his NEMA 17 x 23 (nema17.py) "
                     f"in place of the herringbone assembly's plain block (its solid {i_block}, '{BLOCK}', left out of "
                     f"'{HB}')"}
    names = [m["name"] for m in sc["models"]]
    if NAME in names:
        sc["models"][names.index(NAME)] = entry
    else:
        sc["models"].append(entry)
    for j in sc["joints"]:
        if HB in j["models"] and NAME not in j["models"]:
            j["models"].insert(j["models"].index(HB) + 1, NAME)
    tmp = SCENE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:                   # text mode on Windows: CRLF, as the file is
        json.dump(sc, f, indent=1)
    os.replace(tmp, SCENE)                                        # whole, so the studio never reads half a scene
    print(f"scene: '{NAME}' on joint {next(j['name'] for j in sc['joints'] if NAME in j['models'])}; "
          f"'{HB}' solids now {hb['solids']}")


if __name__ == "__main__":
    main()

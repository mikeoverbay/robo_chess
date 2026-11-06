#!/usr/bin/env python3
"""
gearbox_fit_check.py - does everything that bolts to the gearbox still line up with it? (Mike 2026-11-06: "If you
change any of the bolt circles, you have to make sure that the arms are updated. The bolt onto this. And also, the
claw. Wrist.")

    python gearbox_fit_check.py          (exit code 1 if anything is off)

Reads the bolt circles from cycloid.py itself and finds the holes in the files that get PRINTED:
  output face   3 x M3 on r OUT_SCREW_R at OUT_SCREW_ANGLES: the flange's insert holes; the arm link's cup
                (print/Arm (print 3).step); the claw mount on the wrist (print/Claw mount.step)
  lid circle    6 x M3 on r LID_SCREW_R at LID_SCREW_ANGLES: the gearbox's lid, pin ring and housing; the arm link's
                mounting ring at its elbow end (x -200, the next drive); the shoulder column (print/Shoulder
                turntable.step), checked hole-for-hole against J1's drive placed in complete assembly/arm_base_tube.step
  output face height  Z_DISC_OUT (spread 14) against the link's cup and the claw mount's cup.
A hole "matches" when a cylindrical face of a hole's radius has its axis on the expected one (0.05 mm).
If a bolt circle changes in cycloid.py, rebuild, in order: cycloid.py --spread 14, mockup_link.py (the arms),
arm_tube.py (the shoulder column), claw_adapter.py --spread (the claw mount), then print_parts.py - and run this.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cadquery as cq  # noqa: E402

import cycloid as C  # noqa: E402
from view_step import read_step  # noqa: E402

ROOT = os.path.dirname(HERE)
PRINT = os.path.join(ROOT, "print")
BASE_TUBE = os.path.join(ROOT, "complete assembly", "arm_base_tube.step")
TOL = 0.05
LINK_ELBOW_X = -200.0            # mockup_link.py: the next drive's axis on the link
CUP_T = 8.0                      # mockup_link.py / claw_adapter.py: the 70.5 x 8 cup on the output face

C.set_spread(14.0)
fails = []


def body(path, name):
    for n, s, _ in read_step(path):
        if n == name:
            return s
    raise ValueError(f"{name!r} not in {path}")


def holes(shape, rmin, rmax):
    """axes (point, unit dir) of every cylindrical face with rmin <= radius <= rmax"""
    out = []
    for f in shape.Faces():
        if f.geomType() == "CYLINDER":
            c = f._geomAdaptor().Cylinder()
            if rmin - 1e-3 <= c.Radius() <= rmax + 1e-3:
                p, d = c.Axis().Location(), c.Axis().Direction()
                out.append((cq.Vector(p.X(), p.Y(), p.Z()), cq.Vector(d.X(), d.Y(), d.Z())))
    return out


def on_axis(axes, p, d):
    """is there an axis through p along d (either way)?"""
    for q, e in axes:
        if abs(abs(e.dot(d)) - 1) < 1e-6:
            w = p - q
            if (w - e * w.dot(e)).Length < TOL:
                return True
    return False


def check(label, axes, centres, d=cq.Vector(0, 0, 1)):
    miss = [f"({x:.2f}, {y:.2f})" for x, y in centres if not on_axis(axes, cq.Vector(x, y, 0), d)]
    ok = not miss
    print(f"  {'ok  ' if ok else 'FAIL'} {label}: {len(centres) - len(miss)} of {len(centres)} holes on the circle"
          + ("" if ok else " - missing at " + ", ".join(miss)))
    if not ok:
        fails.append(label)


def solid_at(shape, x, y, z):
    return shape.isInside(cq.Vector(x, y, z), 1e-4)


def face_height(label, shape, z):
    """the part's underside is the output face: solid just above z, nothing just below (at r 25 on +x, clear of
    the holes and the wire hole)"""
    ok = solid_at(shape, 25.0, 0.0, z + 0.2) and not solid_at(shape, 25.0, 0.0, z - 0.2)
    print(f"  {'ok  ' if ok else 'FAIL'} {label}: sits on the output face at z {z:g}")
    if not ok:
        fails.append(label)


def main():
    out_c = [C.polar(C.OUT_SCREW_R, a) for a in C.OUT_SCREW_ANGLES]
    lid_c = [C.polar(C.LID_SCREW_R, a) for a in C.LID_SCREW_ANGLES]
    print(f"cycloid.py: output 3 x M3 on r {C.OUT_SCREW_R:g} at {', '.join(f'{a:g}' for a in C.OUT_SCREW_ANGLES)} deg; "
          f"lid {len(lid_c)} x M3 on r {C.LID_SCREW_R:g} from {C.LID_SCREW_ANGLES[0]:.2f} deg every 60; "
          f"output face z {C.Z_DISC_OUT:g}")

    print("gearbox (print/Gearbox assembly (print 4).step):")
    gb = os.path.join(PRINT, "Gearbox assembly (print 4).step")
    check("output flange insert holes", holes(body(gb, "Gearbox output flange"), C.INSERT_HOLE / 2, C.INSERT_HOLE / 2),
          out_c)
    for nm in ("Gearbox lid", "Gearbox pin ring", "Gearbox housing"):
        check(f"{nm[8:]} lid-screw holes", holes(body(gb, nm), 1.2, 1.8), lid_c)

    print("arm link (print/Arm (print 3).step):")
    arm = body(os.path.join(PRINT, "Arm (print 3).step"), "Arm (print 3)")
    check("cup output-screw holes", holes(arm, 2.2, 2.2), out_c)
    check("elbow mounting-ring holes (next drive)", holes(arm, 1.7, 1.7), [(x + LINK_ELBOW_X, y) for x, y in lid_c])
    face_height("cup", arm, C.Z_DISC_OUT)

    print("claw mount on the wrist (print/Claw mount.step):")
    claw = body(os.path.join(PRINT, "Claw mount.step"), "Claw mount")
    check("cup output-screw holes", holes(claw, 2.2, 2.2), out_c)
    face_height("cup", claw, C.Z_DISC_OUT)

    print("shoulder column (print/Shoulder turntable.step) against J1's drive (complete assembly/arm_base_tube.step):")
    col = body(os.path.join(PRINT, "Shoulder turntable.step"), "Shoulder column")
    drive = next(s for n, s, _ in read_step(BASE_TUBE) if n.startswith("drive J1 mockup drive"))
    rim = holes(drive, 35.25, 35.25)                     # the 70.5 body: J1's axis
    ax_p, ax_d = rim[0]
    col_h = holes(col, 1.7, 1.7)
    drv_h = [(p, d) for p, d in holes(drive, 1.2, 1.8) if abs(abs(d.dot(ax_d)) - 1) < 1e-6]
    ref = holes(drive, 2.0, 2.1)                         # its output insert holes: the angle reference
    # the drive's own lid holes on the cycloid.py circle, angles measured from its first insert hole
    u = ref[0][0] - ax_p
    u = (u - ax_d * u.dot(ax_d)).normalized()
    v = ax_d.cross(u)
    ang = set()
    for p, _ in drv_h:
        w = p - ax_p
        w = w - ax_d * w.dot(ax_d)
        if abs(w.Length - C.LID_SCREW_R) < TOL:
            ang.add(round(math.degrees(math.atan2(w.dot(v), w.dot(u))) % 60.0, 2))
    want = round((C.LID_SCREW_ANGLES[0] - C.OUT_SCREW_ANGLES[0]) % 60.0, 2)
    ok = ang == {want} or ang == {round((-want) % 60.0, 2)}      # (mirror if the axis came out flipped)
    print(f"  {'ok  ' if ok else 'FAIL'} J1 drive model: lid holes on r {C.LID_SCREW_R:g}, {sorted(ang)} deg past an "
          f"output hole (cycloid.py: {want:g})")
    if not ok:
        fails.append("J1 drive model")
    n_ok = sum(on_axis(drv_h, p, d) for p, d in col_h)
    ok = len(col_h) >= len(lid_c) and n_ok == len(col_h)
    print(f"  {'ok  ' if ok else 'FAIL'} column: {n_ok} of {len(col_h)} of its M3 holes on J1's drive's lid holes")
    if not ok:
        fails.append("shoulder column")

    print("\nALL FIT" if not fails else f"\n{len(fails)} OFF: " + "; ".join(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())

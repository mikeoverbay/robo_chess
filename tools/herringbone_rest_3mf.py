#!/usr/bin/env python3
"""
herringbone_rest_3mf.py - the second herringbone print plate: the parts not on herringbone_plate.3mf.

Mike 2026-11-26: "Give me a 3MF for the remaining parts." / "I don't have the outside ring gears. I don't have the
output shafts. Adapter. I'm missing a lot of parts." The gearbox session's herringbone_plate_3mf.py (its file, not
changed here) lays out the spider, input shaft, planets and head spacers; this runs its writer and checks with the
rest of robo_chess/herringbone/print/*.step (already in print orientation per herringbone/report.txt "print", none
of them needing supports) -> robo_chess/herringbone/print/herringbone_plate_2.3mf.

    python herringbone_rest_3mf.py

Rerun after the gearbox session rebuilds the print STEPs (a coupler revision was coming: motor hub + dog bone).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import herringbone_plate_3mf as H  # noqa: E402

# (print file, name in Cura, x, y) from the bed centre, inside the 200 x 200 safe zone: two rows of the 70.5 square
# parts (5 apart, 1 in from the edge), the output disc and the small parts in the third column
REST = [
    ("motor_plate.step", "motor plate (motor side down)", -63.75, 40.0),
    ("ring_half_a.step", "ring half A (motor end down)", 11.75, 40.0),
    ("output_disc.step", "output disc (outer face down)", 75.5, 40.0),
    ("ring_half_b.step", "ring half B (V end down)", -63.75, -40.0),
    ("retainer.step", "retainer", 11.75, -40.0),
    ("motor_hub.step", "motor hub (coupler)", 64.0, -20.0),
    ("dog_bone.step", "dog bone (coupler)", 88.0, -20.0),
] + [("standoff.step", f"standoff {i + 1}", 64.0 + 16.0 * (i % 2), -45.0 - 16.0 * (i // 2)) for i in range(4)]
#   the standoffs 2 x 2, 8 apart: tall and thin (8 x 23.2), so room for a brim round each

if __name__ == "__main__":
    H.LAYOUT = REST
    H.OUT = os.path.join(H.PRINT, "herringbone_plate_2.3mf")
    H.main()

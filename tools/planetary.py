#!/usr/bin/env python3
"""
planetary.py - single-stage planetary gearbox for a NEMA 17 stepper, sized for FDM printing.

  python planetary.py list   [options]                 enumerate every tooth combination that fits
  python planetary.py design --sun S --planet P [options]
                                                       dimension sheet + DXF/SVG profiles + mesh check

Options (defaults in brackets)
  --module M      gear module in mm [1.0]       (1.0 is the practical floor for a 0.4 mm nozzle)
  --planets N     number of planets [3]
  --max-od D      largest housing OD you accept, mm [56]
  --backlash B    circular backlash per mesh, mm [0.10]; each tooth is thinned B/2 at the pitch circle
  --pin D         planet pin diameter, mm [3.0]
  --bearing D     planet bearing OD, mm [0 = plain printed bore on the pin]
  --microsteps K  driver microstepping, for the resolution line only [16]
  --out DIR       output directory [.]
  --steps K       points per flank in the profiles [14]

Fixed design rules (standard 20 deg involute, no profile shift)
  addendum 1.00 m, dedendum 1.25 m
  ring = sun + 2 * planet                       same module everywhere, planets fit between
  (sun + ring) % planets == 0                   equally spaced planets can all mesh
  neighbouring planets clear tip to tip         by NEIGHBOUR_CLEAR
  sun root clears the 5 mm D shaft bore         by MIN_SUN_WALL
  planet root clears its pin or bearing         by MIN_PLANET_WALL
  ring root + HOUSING_WALL fits in --max-od

Ratio with the ring fixed and the carrier as output:  1 + ring / sun
Output files (design): sun.dxf planet.dxf ring.dxf motor_plate.dxf assembly.svg  (+ assembly.png if Pillow is present)
"""
import argparse
import math
import os
import sys
from math import gcd

# ---------------------------------------------------------------- NEMA 17 interface
NEMA17_FACE = 42.3          # square body, mm
NEMA17_PILOT_D = 22.0       # locating boss on the front face
NEMA17_PILOT_H = 2.0        # its height
NEMA17_BOLT_HALF = 15.5     # four M3 tapped holes at (+-15.5, +-15.5)
NEMA17_BOLT_R = NEMA17_BOLT_HALF * math.sqrt(2)   # 21.92 mm from the axis
NEMA17_SHAFT_D = 5.0
NEMA17_SHAFT_FLAT = 0.5     # D flat depth
M3_HEAD_D = 5.5             # socket head cap screw

# ---------------------------------------------------------------- gear rules
PA = math.radians(20.0)
ADDENDUM = 1.00
DEDENDUM = 1.25
MIN_SUN_WALL = 1.5          # printed wall between the shaft bore and the sun root, mm
MIN_PLANET_WALL = 1.5       # printed wall between the pin/bearing bore and the planet root, mm
HOUSING_WALL = 2.5          # printed wall outside the ring root, mm
NEIGHBOUR_CLEAR = 1.0       # tip-to-tip gap between neighbouring planets, mm
BORE_CLEAR = 0.15           # added to printed bore diameters
MIN_RING_TEETH = 34         # below this a 20 deg internal tooth tip drops under the base circle


# ================================================================ design arithmetic
def design(m, n, s, p, pin, bearing, microsteps=16):
    r = s + 2 * p
    d = dict(module=m, planets=n, sun=s, planet=p, ring=r)
    d['ratio'] = 1.0 + r / s
    d['spaced'] = (s + r) % n == 0

    rp_s, rp_p, rp_r = m * s / 2, m * p / 2, m * r / 2
    d['rp_sun'], d['rp_planet'], d['rp_ring'] = rp_s, rp_p, rp_r
    d['ra_sun'] = rp_s + ADDENDUM * m
    d['rf_sun'] = rp_s - DEDENDUM * m
    d['ra_planet'] = rp_p + ADDENDUM * m
    d['rf_planet'] = rp_p - DEDENDUM * m
    d['ra_ring'] = rp_r - ADDENDUM * m          # internal: tip is the inner radius
    d['rf_ring'] = rp_r + DEDENDUM * m          # internal: root is the outer radius

    d['carrier_r'] = m * (s + p) / 2            # planet pin circle radius
    d['housing_od'] = 2 * (d['rf_ring'] + HOUSING_WALL)
    d['sun_bore'] = NEMA17_SHAFT_D + BORE_CLEAR
    d['sun_wall'] = d['rf_sun'] - d['sun_bore'] / 2
    d['planet_bore'] = (bearing + 0.10) if bearing else (pin + BORE_CLEAR)
    d['planet_wall'] = d['rf_planet'] - d['planet_bore'] / 2
    d['max_bearing_od'] = 2 * (d['rf_planet'] - MIN_PLANET_WALL)
    d['neighbour_gap'] = 2 * d['carrier_r'] * math.sin(math.pi / n) - 2 * d['ra_planet']
    d['sweep_r'] = d['carrier_r'] + d['ra_planet']          # outermost point a planet reaches
    d['bolt_clear'] = NEMA17_BOLT_R - M3_HEAD_D / 2 - d['sweep_r']   # <0: counterbore the heads flush
    d['out_step_deg'] = 1.8 / microsteps / d['ratio']
    d['hunting'] = gcd(s, p) == 1 and gcd(p, r) == 1

    d['ok'] = (d['spaced'] and d['sun_wall'] >= MIN_SUN_WALL and d['planet_wall'] >= MIN_PLANET_WALL
               and d['neighbour_gap'] >= NEIGHBOUR_CLEAR and r >= MIN_RING_TEETH)
    return d


def candidates(m, n, max_od, pin, bearing, microsteps, ratio_max=8.0):
    out = []
    for s in range(6, 40):
        for p in range(6, 80):
            d = design(m, n, s, p, pin, bearing, microsteps)
            if d['ok'] and d['housing_od'] <= max_od and d['ratio'] <= ratio_max:
                out.append(d)
    out.sort(key=lambda d: (d['ratio'], d['housing_od']))
    return out


def print_list(cands, m, n, max_od):
    print(f"module {m}  planets {n}  housing OD <= {max_od} mm  (NEMA 17 body is {NEMA17_FACE} mm square)")
    print(f"{'sun':>4} {'pl':>4} {'ring':>5} {'ratio':>7} {'OD':>6} {'sunwall':>8} {'plwall':>7} {'gap':>6} "
          f"{'sweep':>6} {'bolt':>6} {'maxbrg':>7} {'step':>7} hunt")
    for d in cands:
        print(f"{d['sun']:>4} {d['planet']:>4} {d['ring']:>5} {d['ratio']:>7.3f} {d['housing_od']:>6.1f} "
              f"{d['sun_wall']:>8.2f} {d['planet_wall']:>7.2f} {d['neighbour_gap']:>6.1f} "
              f"{d['sweep_r']:>6.1f} {d['bolt_clear']:>6.1f} {d['max_bearing_od']:>7.1f} "
              f"{d['out_step_deg']:>7.4f} {'y' if d['hunting'] else '-'}")
    print("sunwall/plwall: printed wall around the bore, mm.  gap: planet tip to neighbouring planet tip, mm.")
    print("sweep: outermost planet radius, mm.  bolt: clearance from that sweep to the M3 heads (negative = counterbore them flush).")
    print("maxbrg: largest planet bearing OD that leaves a wall.  step: output degrees per microstep.  hunt: no common factors.")


def print_sheet(d, backlash, microsteps):
    m = d['module']
    print(f"Planetary stage, module {m}, {d['planets']} planets, ring fixed, carrier output")
    print(f"  sun {d['sun']}T  planet {d['planet']}T  ring {d['ring']}T   ratio {d['ratio']:.4f}:1")
    print(f"  output step {d['out_step_deg']:.4f} deg at {microsteps} microsteps "
          f"({360/d['out_step_deg']:.0f} microsteps per output turn)")
    print(f"  equal spacing {'ok' if d['spaced'] else 'FAILS'}   hunting tooth {'yes' if d['hunting'] else 'no'}")
    print("Pitch / tip / root diameters (mm)")
    print(f"  sun     {2*d['rp_sun']:7.2f} / {2*d['ra_sun']:7.2f} / {2*d['rf_sun']:7.2f}   "
          f"bore {d['sun_bore']:.2f} D-shaft, wall {d['sun_wall']:.2f}")
    print(f"  planet  {2*d['rp_planet']:7.2f} / {2*d['ra_planet']:7.2f} / {2*d['rf_planet']:7.2f}   "
          f"bore {d['planet_bore']:.2f}, wall {d['planet_wall']:.2f}, max bearing OD {d['max_bearing_od']:.1f}")
    print(f"  ring    {2*d['rp_ring']:7.2f} / {2*d['ra_ring']:7.2f} / {2*d['rf_ring']:7.2f}   "
          f"housing OD {d['housing_od']:.1f} with {HOUSING_WALL} mm wall")
    print(f"  carrier pin circle radius {d['carrier_r']:.2f}  (pins at {d['planets']} x {360/d['planets']:.0f} deg)")
    print(f"  centre distance sun-planet {d['rp_sun']+d['rp_planet']:.2f}, ring-planet {d['rp_ring']-d['rp_planet']:.2f}")
    print(f"  neighbouring planets tip gap {d['neighbour_gap']:.2f} mm")
    print(f"  planet sweep radius {d['sweep_r']:.2f} vs M3 heads at r={NEMA17_BOLT_R:.2f}: "
          + (f"clear by {d['bolt_clear']:.2f} mm" if d['bolt_clear'] >= 0
             else f"collides by {-d['bolt_clear']:.2f} mm, counterbore the heads flush in the motor plate"))
    print(f"  circular backlash {backlash:.2f} mm per mesh (each tooth thinned {backlash/2:.2f}); tooth depth {2.25*m:.2f} mm, "
          f"tooth thickness at pitch {math.pi*m/2 - backlash/2:.2f} mm")
    print(f"  NEMA 17: pilot boss {NEMA17_PILOT_D} x {NEMA17_PILOT_H} mm, bolts M3 at +-{NEMA17_BOLT_HALF}, shaft {NEMA17_SHAFT_D} D")
    if d['sun'] < 17:
        print(f"  note: {d['sun']}T sun is below the 17T undercut limit at 20 deg; fine for a printed low-speed stage")


# ================================================================ involute profiles
def inv(rb, r):
    if r <= rb:
        return 0.0
    th = math.acos(rb / r)
    return math.tan(th) - th


def external_profile(n, m, backlash, steps, rot=0.0):
    """Closed CCW polygon of an external spur gear, tooth 0 centred on angle `rot`."""
    rp = m * n / 2
    rb = rp * math.cos(PA)
    ra = rp + ADDENDUM * m
    rf = rp - DEDENDUM * m
    psi = math.pi / (2 * n) - backlash / (4 * rp)      # half tooth angle at the pitch circle
    inva = math.tan(PA) - PA
    # keep a tip land of at least 0.3 mm (small suns go pointed otherwise)
    while (psi + inva - inv(rb, ra)) * ra * 2 < 0.3 and ra > rp:
        ra -= 0.02 * m
    r0 = max(rb, rf)
    half = []                                   # (radius, half angle) root -> tip on one flank
    if rf < rb:
        half.append((rf, psi + inva))           # radial stub below the base circle
    for i in range(steps + 1):
        r = r0 + (ra - r0) * i / steps
        half.append((r, psi + inva - inv(rb, r)))
    pts = []
    pitch = 2 * math.pi / n
    for k in range(n):
        c = rot + pitch * k
        for r, a in half:
            pts.append((r, c - a))
        for r, a in reversed(half):
            pts.append((r, c + a))
        a0 = c + half[0][1]
        a1 = c + pitch - half[0][1]
        for t in (1 / 3, 2 / 3):
            pts.append((rf, a0 + (a1 - a0) * t))
    return [(r * math.cos(a), r * math.sin(a)) for r, a in pts]


def internal_profile(n, m, backlash, steps, rot=0.0):
    """Closed CCW polygon of the ring's inner (toothed) boundary, tooth 0 centred on `rot`.
    The ring material lies OUTSIDE this polygon."""
    rp = m * n / 2
    rb = rp * math.cos(PA)
    ra = rp - ADDENDUM * m                      # tip, inner
    rf = rp + DEDENDUM * m                      # root, outer
    psi = math.pi / (2 * n) - backlash / (4 * rp)
    inva = math.tan(PA) - PA
    r0 = max(ra, rb)
    half = []                                   # tip -> root
    if ra < rb:
        half.append((ra, psi - inva))
    for i in range(steps + 1):
        r = r0 + (rf - r0) * i / steps
        half.append((r, psi - inva + inv(rb, r)))
    pts = []
    pitch = 2 * math.pi / n
    for k in range(n):
        c = rot + pitch * k
        for r, a in reversed(half):             # root -> tip on the minus side
            pts.append((r, c - a))
        for r, a in half:                       # tip -> root on the plus side
            pts.append((r, c + a))
        a0 = c + half[-1][1]
        a1 = c + pitch - half[-1][1]
        for t in (1 / 3, 2 / 3):
            pts.append((rf, a0 + (a1 - a0) * t))
    return [(r * math.cos(a), r * math.sin(a)) for r, a in pts]


def d_bore(dia, flat_depth, segs=48):
    """D-shaped bore polygon: circle of `dia` with a flat `flat_depth` deep on the -x side."""
    rr = dia / 2
    xf = -(rr - flat_depth)
    a0 = math.acos(-xf / rr)                    # half angle of the flat chord, measured from +x... from -x
    pts = []
    start, end = math.pi - a0, math.pi + a0     # arc from the top of the flat, around the right side, to its bottom
    span = 2 * math.pi - (end - start)
    for i in range(segs + 1):
        a = end + span * i / segs
        pts.append((rr * math.cos(a), rr * math.sin(a)))
    return pts                                   # closing edge is the flat


def transform(pts, rot, dx, dy):
    c, s = math.cos(rot), math.sin(rot)
    return [(x * c - y * s + dx, x * s + y * c + dy) for x, y in pts]


# ================================================================ assembly phases
def assembly(d, backlash, steps):
    """Sun at the origin, planets on the carrier circle, ring around, all in mesh."""
    m, s, p, r, n = d['module'], d['sun'], d['planet'], d['ring'], d['planets']
    sun = external_profile(s, m, backlash, steps, 0.0)
    ring_rot = 0.0 if p % 2 == 0 else math.pi / r
    ring = internal_profile(r, m, backlash, steps, ring_rot)
    planets = []
    for j in range(n):
        phi = 2 * math.pi * j / n
        rho = phi + math.pi - math.pi / p + s * phi / p     # planet tooth phase that meshes with the sun
        pts = external_profile(p, m, backlash, steps, 0.0)
        planets.append(transform(pts, rho, d['carrier_r'] * math.cos(phi), d['carrier_r'] * math.sin(phi)))
    return sun, planets, ring


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xcross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < xcross:
                inside = not inside
        j = i
    return inside


def bbox(poly):
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def overlap_count(a, b):
    """Vertices of a inside polygon b (bbox pre-filter)."""
    x0, y0, x1, y1 = bbox(b)
    return sum(1 for x, y in a if x0 <= x <= x1 and y0 <= y <= y1 and point_in_poly(x, y, b))


def mesh_check(sun, planets, ring):
    bad = 0
    for j, pl in enumerate(planets):
        a = overlap_count(sun, pl) + overlap_count(pl, sun)
        b = overlap_count(ring, pl) + sum(1 for x, y in pl if not point_in_poly(x, y, ring))
        print(f"  planet {j}: sun/planet overlaps {a}, ring/planet overlaps {b}")
        bad += a + b
    for j in range(len(planets)):
        k = (j + 1) % len(planets)
        if k != j:
            bad += overlap_count(planets[j], planets[k])
    return bad


# ================================================================ writers
class DXF:
    def __init__(self):
        self.e = []

    def _g(self, code, value):
        self.e.append(f"{code:>3}")
        self.e.append(str(value))

    def poly(self, pts, layer):
        self._g(0, "POLYLINE"); self._g(8, layer); self._g(66, 1); self._g(70, 1)
        for x, y in pts:
            self._g(0, "VERTEX"); self._g(8, layer)
            self._g(10, f"{x:.4f}"); self._g(20, f"{y:.4f}"); self._g(30, "0.0")
        self._g(0, "SEQEND")

    def circle(self, x, y, r, layer):
        self._g(0, "CIRCLE"); self._g(8, layer)
        self._g(10, f"{x:.4f}"); self._g(20, f"{y:.4f}"); self._g(30, "0.0"); self._g(40, f"{r:.4f}")

    def write(self, path):
        head = ["  0", "SECTION", "  2", "HEADER", "  9", "$ACADVER", "  1", "AC1009", "  0", "ENDSEC",
                "  0", "SECTION", "  2", "ENTITIES"]
        tail = ["  0", "ENDSEC", "  0", "EOF"]
        with open(path, "w", encoding="ascii") as f:
            f.write("\n".join(head + self.e + tail) + "\n")


def svg_path(pts):
    return "M " + " L ".join(f"{x:.3f},{y:.3f}" for x, y in pts) + " Z"


def write_svg(path, size, items):
    """items: list of (kind, data, colour). kind 'poly' -> pts, 'circle' -> (x,y,r). y is flipped to CAD sense."""
    h = size / 2
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}mm" height="{size}mm" '
           f'viewBox="{-h} {-h} {size} {size}">',
           '<g transform="scale(1,-1)" fill="none" stroke-width="0.15">']
    for kind, data, colour in items:
        if kind == 'poly':
            out.append(f'<path d="{svg_path(data)}" stroke="{colour}"/>')
        else:
            x, y, r = data
            out.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{r:.3f}" stroke="{colour}"/>')
    out.append('</g></svg>')
    with open(path, "w", encoding="ascii") as f:
        f.write("\n".join(out) + "\n")


def write_png(path, size, items, px=1400):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return False
    img = Image.new("RGB", (px, px), "white")
    dr = ImageDraw.Draw(img)
    k = px / size
    def P(x, y):
        return (px / 2 + x * k, px / 2 - y * k)
    for kind, data, colour in items:
        if kind == 'poly':
            dr.polygon([P(x, y) for x, y in data], outline=colour)
        else:
            x, y, r = data
            dr.ellipse([P(x - r, y + r), P(x + r, y - r)], outline=colour)
    img.save(path)
    return True


# ================================================================ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["list", "design"])
    ap.add_argument("--module", type=float, default=1.0)
    ap.add_argument("--planets", type=int, default=3)
    ap.add_argument("--max-od", type=float, default=56.0)
    ap.add_argument("--backlash", type=float, default=0.10)
    ap.add_argument("--pin", type=float, default=3.0)
    ap.add_argument("--bearing", type=float, default=0.0)
    ap.add_argument("--microsteps", type=int, default=16)
    ap.add_argument("--sun", type=int)
    ap.add_argument("--planet", type=int)
    ap.add_argument("--out", default=".")
    ap.add_argument("--steps", type=int, default=14)
    a = ap.parse_args()

    if a.cmd == "list":
        print_list(candidates(a.module, a.planets, a.max_od, a.pin, a.bearing, a.microsteps),
                   a.module, a.planets, a.max_od)
        return

    if a.sun is None or a.planet is None:
        sys.exit("design needs --sun and --planet tooth counts (run `list` to pick them)")
    d = design(a.module, a.planets, a.sun, a.planet, a.pin, a.bearing, a.microsteps)
    print_sheet(d, a.backlash, a.microsteps)
    if not d['ok']:
        print("  WARNING: this combination breaks one of the design rules (see the list columns)")

    os.makedirs(a.out, exist_ok=True)
    m = a.module
    sun, planets, ring = assembly(d, a.backlash, a.steps)

    print("Mesh check (polygon overlaps at the assembled phase, 0 is correct)")
    bad = mesh_check(sun, planets, ring)
    print(f"  total overlaps: {bad}")

    # individual parts, each centred on its own axis
    x = DXF()
    x.poly(external_profile(d['sun'], m, a.backlash, a.steps), "SUN")
    x.poly(d_bore(d['sun_bore'], NEMA17_SHAFT_FLAT), "SUN_BORE")
    x.write(os.path.join(a.out, "sun.dxf"))

    x = DXF()
    x.poly(external_profile(d['planet'], m, a.backlash, a.steps), "PLANET")
    x.circle(0, 0, d['planet_bore'] / 2, "PLANET_BORE")
    x.write(os.path.join(a.out, "planet.dxf"))

    x = DXF()
    x.poly(internal_profile(d['ring'], m, a.backlash, a.steps), "RING_TEETH")
    x.circle(0, 0, d['housing_od'] / 2, "HOUSING")
    x.write(os.path.join(a.out, "ring.dxf"))

    x = DXF()
    x.circle(0, 0, d['housing_od'] / 2, "HOUSING")
    x.circle(0, 0, (NEMA17_PILOT_D + 0.2) / 2, "PILOT_POCKET")
    x.circle(0, 0, (2 * d['ra_sun'] + 1.0) / 2, "SUN_CLEARANCE")
    for sx in (-1, 1):
        for sy in (-1, 1):
            x.circle(sx * NEMA17_BOLT_HALF, sy * NEMA17_BOLT_HALF, 3.4 / 2, "M3_HOLE")
            x.circle(sx * NEMA17_BOLT_HALF, sy * NEMA17_BOLT_HALF, (M3_HEAD_D + 0.6) / 2, "M3_COUNTERBORE")
    x.write(os.path.join(a.out, "motor_plate.dxf"))

    # whole stage in mesh position, one DXF, parts on their own layers
    x = DXF()
    x.poly(sun, "SUN")
    x.poly(d_bore(d['sun_bore'], NEMA17_SHAFT_FLAT), "SUN_BORE")
    for j, pl in enumerate(planets):
        phi = 2 * math.pi * j / d['planets']
        x.poly(pl, f"PLANET{j}")
        x.circle(d['carrier_r'] * math.cos(phi), d['carrier_r'] * math.sin(phi), d['planet_bore'] / 2, "PLANET_BORE")
    x.poly(ring, "RING_TEETH")
    x.circle(0, 0, d['housing_od'] / 2, "HOUSING")
    x.circle(0, 0, d['carrier_r'], "PIN_CIRCLE")
    for sx in (-1, 1):
        for sy in (-1, 1):
            x.circle(sx * NEMA17_BOLT_HALF, sy * NEMA17_BOLT_HALF, 3.4 / 2, "M3_HOLE")
    x.write(os.path.join(a.out, "assembly.dxf"))

    # assembly preview
    items = [('poly', sun, "#c00"), ('poly', ring, "#06c"),
             ('circle', (0, 0, d['housing_od'] / 2), "#999"),
             ('circle', (0, 0, NEMA17_BOLT_R), "#bbb")]
    for sx in (-1, 1):
        for sy in (-1, 1):
            items.append(('circle', (sx * NEMA17_BOLT_HALF, sy * NEMA17_BOLT_HALF, M3_HEAD_D / 2), "#888"))
    for j, pl in enumerate(planets):
        phi = 2 * math.pi * j / d['planets']
        items.append(('poly', pl, "#080"))
        items.append(('circle', (d['carrier_r'] * math.cos(phi), d['carrier_r'] * math.sin(phi),
                                 d['planet_bore'] / 2), "#080"))
    size = d['housing_od'] + 6
    write_svg(os.path.join(a.out, "assembly.svg"), size, items)
    png = write_png(os.path.join(a.out, "assembly.png"), size, items)
    print(f"wrote sun.dxf planet.dxf ring.dxf motor_plate.dxf assembly.dxf assembly.svg{' assembly.png' if png else ''} in {a.out}")
    if bad:
        sys.exit(1)


if __name__ == "__main__":
    main()

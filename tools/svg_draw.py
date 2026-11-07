#!/usr/bin/env python3
"""
svg_draw.py - Robo Chess: turn SVG drawings into pen strokes for Robot Studio's Draw demo.

Mike, 2026-11-06, gave E:\\...\\robo_chess\\hand.svg to draw. Every *.svg in the robo_chess folder and in
robo_chess\\drawings is read; each becomes a choice in the Draw menu ("svg: <name>").

    python svg_draw.py              all of them -> scene.json "drawings"
    python svg_draw.py --pitch 0.5  fill lines closer together (default 0.6 mm)
    python svg_draw.py --pens 8     at most 8 pens (colours merged to fit)
    python svg_draw.py hand         just hand.svg

How a pen draws an SVG:
  - a FILLED shape (the SVG default) is filled the way a plotter does it well: lines that follow its outline, nested a
    pen-width apart (PITCH) from just inside the edge to the middle - long continuous strokes, few pen lifts. They are the
    contours of the distance to the shape's edge: the shape is drawn into a fine pixel grid (RES mm), scipy measures each
    pixel's distance to the edge, contourpy traces that distance at EDGE_IN, EDGE_IN + PITCH, ... A thick band gets
    several nested lines, a thin brush stroke one or two, its sharp tips stop where they get thinner than about EDGE_IN x 2.
  - a STROKED shape (fill none) is drawn along its centre line, once.
  - colours: each drawing gets its own pens, up to MAX_PENS (20: the stands the arm reaches for a pen change, the front
    rows of both trays) - the SVG's own colours if it has that few, else merged to that many; near-white is the paper.
    Shapes are painted in the SVG's order first, so where one covers another only the colour on top is drawn.
  - the drawing is scaled to fit the paper the pen can reach (BOX: 440 across, 418 from the white side to 171 mm from the
    swivel axis, measured with the IK), its top toward the robot, as the white player reads it.
  - strokes are ordered nearest-next (a closed loop starts at its point nearest the pen, an open line from its nearer end)
    so the empty pen travels little.
Supported: path (all commands, arcs included), line, polyline, polygon, rect, circle, ellipse; transform on elements and
groups (matrix, translate, scale, rotate, skewX/Y); fill / stroke / fill-rule as attributes or in style. Not: text,
images, clip paths, gradients (a gradient fill counts as black).
"""
import argparse
import glob
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
import studio  # noqa: E402
SCENE = studio.scene_path()          # this studio's scene (ROBOT_STUDIO; default main = viewer_session/scene.json)
RES = 0.1                    # mm per pixel of the fill grid
EDGE_IN = 0.15               # the first fill line this far inside the edge (the ink, ~0.5 wide, reaches the edge)
SIMPLIFY = 0.04              # mm: points dropped while the line stays this close
BOX_W, BOX_H = 440.0, 418.0  # the paper the pen reaches (viewer: x -589..-171 at the board's middle, z +-220)
BOX_DX = -18.0               # the box's centre from the 64 squares' centre, toward the white side (mm)
PENS = {"black": (0x1b, 0x1b, 0x1f), "blue": (0x1d, 0x3c, 0x9e), "red": (0xb3, 0x26, 0x1e)}
NAMED = {"black": "#000000", "white": "#ffffff", "red": "#ff0000", "blue": "#0000ff", "green": "#008000",
         "gray": "#808080", "grey": "#808080", "navy": "#000080", "maroon": "#800000"}


# ---------------------------------------------------------------- SVG geometry -> polylines (SVG units)
def tmat(s):
    """an SVG transform attribute -> 3x3 matrix"""
    M = np.eye(3)
    for name, args in re.findall(r"(\w+)\s*\(([^)]*)\)", s or ""):
        a = [float(v) for v in re.findall(r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?", args)]
        if name == "matrix":
            T = np.array([[a[0], a[2], a[4]], [a[1], a[3], a[5]], [0, 0, 1]])
        elif name == "translate":
            T = np.array([[1, 0, a[0]], [0, 1, a[1] if len(a) > 1 else 0], [0, 0, 1]])
        elif name == "scale":
            T = np.diag([a[0], a[1] if len(a) > 1 else a[0], 1])
        elif name == "rotate":
            c, s_ = math.cos(math.radians(a[0])), math.sin(math.radians(a[0]))
            T = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
            if len(a) == 3:
                T = np.array([[1, 0, a[1]], [0, 1, a[2]], [0, 0, 1]]) @ T @ np.array([[1, 0, -a[1]], [0, 1, -a[2]], [0, 0, 1]])
        elif name == "skewX":
            T = np.array([[1, math.tan(math.radians(a[0])), 0], [0, 1, 0], [0, 0, 1]])
        elif name == "skewY":
            T = np.array([[1, 0, 0], [math.tan(math.radians(a[0])), 1, 0], [0, 0, 1]])
        else:
            continue
        M = M @ T
    return M


def bez(pts, n):
    """a cubic or quadratic Bezier (control points) -> n points after the first"""
    P = np.array(pts, float)
    t = np.linspace(0, 1, n + 1)[1:, None]
    if len(P) == 4:
        return ((1 - t) ** 3) * P[0] + 3 * ((1 - t) ** 2) * t * P[1] + 3 * (1 - t) * t * t * P[2] + t ** 3 * P[3]
    return ((1 - t) ** 2) * P[0] + 2 * (1 - t) * t * P[1] + t * t * P[2]


def arc(p0, rx, ry, phi, large, sweep, p1, step):
    """an SVG elliptical arc (endpoint form, SVG 1.1 F.6.5) -> points after p0"""
    if rx == 0 or ry == 0:
        return [p1]
    rx, ry = abs(rx), abs(ry)
    c, s = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    dx, dy = (p0[0] - p1[0]) / 2, (p0[1] - p1[1]) / 2
    x1, y1 = c * dx + s * dy, -s * dx + c * dy
    lam = x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry)
    if lam > 1:
        rx, ry = rx * math.sqrt(lam), ry * math.sqrt(lam)
    num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1
    k = math.sqrt(max(0.0, num / (rx * rx * y1 * y1 + ry * ry * x1 * x1)))
    if large == sweep:
        k = -k
    cx1, cy1 = k * rx * y1 / ry, -k * ry * x1 / rx
    cx, cy = c * cx1 - s * cy1 + (p0[0] + p1[0]) / 2, s * cx1 + c * cy1 + (p0[1] + p1[1]) / 2
    ang = lambda ux, uy, vx, vy: math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
    t0 = ang(1, 0, (x1 - cx1) / rx, (y1 - cy1) / ry)
    dt = ang((x1 - cx1) / rx, (y1 - cy1) / ry, (-x1 - cx1) / rx, (-y1 - cy1) / ry)
    if not sweep and dt > 0:
        dt -= 2 * math.pi
    elif sweep and dt < 0:
        dt += 2 * math.pi
    n = max(4, int(math.ceil(abs(dt) * max(rx, ry) / step)))
    out = []
    for i in range(1, n + 1):
        t = t0 + dt * i / n
        out.append((cx + rx * math.cos(t) * c - ry * math.sin(t) * s, cy + rx * math.cos(t) * s + ry * math.sin(t) * c))
    return out


def path_polys(d, step):
    """SVG path data -> [(points, closed)] in SVG units, curves every ~step"""
    toks = re.findall(r"[MmLlHhVvCcSsQqTtAaZz]|[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?", d)
    polys, cur, start, pts = [], (0.0, 0.0), (0.0, 0.0), []
    cmd, i, last_ctrl, last_cmd = None, 0, None, None

    def num():
        nonlocal i
        v = float(toks[i]); i += 1
        return v

    def flush(closed):
        nonlocal pts
        if len(pts) > 1:
            polys.append((pts, closed))
        pts = []

    def nseg(ctrl):
        L = sum(math.dist(ctrl[k], ctrl[k + 1]) for k in range(len(ctrl) - 1))
        return max(2, int(math.ceil(L / step)))

    while i < len(toks):
        if re.match(r"[A-Za-z]", toks[i]):
            cmd = toks[i]; i += 1
        rel = cmd.islower()
        C = cmd.upper()
        if C == "Z":
            if pts:
                pts.append(start)
            flush(True)
            cur = start
            last_cmd = "Z"
            continue
        if i >= len(toks) or re.match(r"[A-Za-z]", toks[i]):
            continue                                   # a command with no numbers left: skip
        ox, oy = cur if rel else (0.0, 0.0)
        if C == "M":
            flush(False)
            cur = (ox + num(), oy + num()); start = cur; pts = [cur]
            cmd = "l" if rel else "L"
        elif C == "L":
            cur = (ox + num(), oy + num()); pts.append(cur)
        elif C == "H":
            cur = ((ox if rel else 0) + num(), cur[1]); pts.append(cur)
        elif C == "V":
            cur = (cur[0], (oy if rel else 0) + num()); pts.append(cur)
        elif C in "CS":
            if C == "C":
                c1 = (ox + num(), oy + num())
            else:
                c1 = (2 * cur[0] - last_ctrl[0], 2 * cur[1] - last_ctrl[1]) if last_cmd in ("C", "S") else cur
            c2 = (ox + num(), oy + num()); p = (ox + num(), oy + num())
            pts += [tuple(q) for q in bez([cur, c1, c2, p], nseg([cur, c1, c2, p]))]
            last_ctrl, cur = c2, p
        elif C in "QT":
            if C == "Q":
                c1 = (ox + num(), oy + num())
            else:
                c1 = (2 * cur[0] - last_ctrl[0], 2 * cur[1] - last_ctrl[1]) if last_cmd in ("Q", "T") else cur
            p = (ox + num(), oy + num())
            pts += [tuple(q) for q in bez([cur, c1, p], nseg([cur, c1, p]))]
            last_ctrl, cur = c1, p
        elif C == "A":
            rx, ry, phi, la, sw = num(), num(), num(), num(), num()
            p = (ox + num(), oy + num())
            pts += arc(cur, rx, ry, phi, int(la), int(sw), p, step)
            cur = p
        if not pts:
            pts = [cur]
        last_cmd = C
    flush(False)
    return polys


def style_of(el, inherited):
    st = dict(inherited)
    for k in ("fill", "stroke", "fill-rule", "display", "visibility"):
        if el.get(k) is not None:
            st[k] = el.get(k).strip()
    for part in (el.get("style") or "").split(";"):
        if ":" in part:
            k, v = part.split(":", 1)
            if k.strip() in ("fill", "stroke", "fill-rule", "display", "visibility"):
                st[k.strip()] = v.strip()
    return st


def shapes(svg_path, step):
    """-> [{"polys": [(pts Nx2 SVG units, closed)], "fill": colour or None, "stroke": colour or None}]"""
    root = ET.parse(svg_path).getroot()
    out = []

    def tag(el):
        return el.tag.split("}")[-1]

    def walk(el, M, st):
        t = tag(el)
        if t in ("defs", "clipPath", "mask", "symbol", "metadata", "title", "desc", "style", "text"):
            return
        M = M @ tmat(el.get("transform"))
        st = style_of(el, st)
        if st.get("display") == "none" or st.get("visibility") == "hidden":
            return
        polys = []
        f = lambda k, dflt=0.0: float(re.sub(r"[a-z%]+$", "", el.get(k, str(dflt))) or dflt)
        if t == "path":
            polys = path_polys(el.get("d", ""), step)
        elif t in ("polyline", "polygon"):
            v = [float(q) for q in re.findall(r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?", el.get("points", ""))]
            p = list(zip(v[0::2], v[1::2]))
            if t == "polygon" and p:
                p.append(p[0])
            polys = [(p, t == "polygon")]
        elif t == "line":
            polys = [([(f("x1"), f("y1")), (f("x2"), f("y2"))], False)]
        elif t == "rect":
            x, y, w, h = f("x"), f("y"), f("width"), f("height")
            polys = [([(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)], True)]
        elif t in ("circle", "ellipse"):
            cx, cy = f("cx"), f("cy")
            rx = f("r") if t == "circle" else f("rx")
            ry = f("r") if t == "circle" else f("ry")
            n = max(24, int(math.ceil(2 * math.pi * max(rx, ry) / step)))
            polys = [([(cx + rx * math.cos(2 * math.pi * k / n), cy + ry * math.sin(2 * math.pi * k / n)) for k in range(n + 1)], True)]
        if polys:
            pp = []
            for p, closed in polys:
                a = np.c_[np.array(p, float), np.ones(len(p))] @ M.T
                pp.append((a[:, :2], closed))
            fill = st.get("fill", "#000000")
            stroke = st.get("stroke")
            out.append({"polys": pp, "fill": None if fill in ("none", "transparent") else fill,
                        "stroke": None if stroke in (None, "none", "transparent") else stroke})
        for ch in el:
            walk(ch, M, st)

    walk(root, np.eye(3), {})
    return out


def rgb_of(colour):
    """an SVG colour -> (r, g, b) 0..255 (unknown -> black)"""
    c = (colour or "#000").strip().lower()
    c = NAMED.get(c, c)
    m = re.match(r"#([0-9a-f]{3}|[0-9a-f]{6})$", c)
    if m:
        h = m.group(1)
        h = "".join(ch * 2 for ch in h) if len(h) == 3 else h
        return tuple(int(h[k:k + 2], 16) for k in (0, 2, 4))
    m2 = re.match(r"rgb\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)", c)
    return tuple(int(v) for v in m2.groups()) if m2 else (0, 0, 0)


def lab(rgb):
    """sRGB 0..255 (Nx3) -> CIE L*a*b* (D65): distances there match what the eye sees"""
    c = np.asarray(rgb, float).reshape(-1, 3) / 255.0
    c = np.where(c > 0.04045, ((c + 0.055) / 1.055) ** 2.4, c / 12.92)
    xyz = c @ np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]]).T
    xyz /= np.array([0.95047, 1.0, 1.08883])
    f = np.where(xyz > 0.008856, np.cbrt(xyz), 7.787 * xyz + 16 / 116)
    return np.c_[116 * f[:, 1] - 16, 500 * (f[:, 0] - f[:, 1]), 200 * (f[:, 1] - f[:, 2])]


def kmeans(X, w, k, iters=60):
    """weighted k-means (k-means++ start, fixed seed) -> labels, centres"""
    rng = np.random.default_rng(1)
    C = [X[int(np.argmax(w))]]
    for _ in range(1, k):
        d = np.min(((X[:, None, :] - np.array(C)[None]) ** 2).sum(-1), axis=1) * w
        C.append(X[int(rng.choice(len(X), p=d / d.sum()))] if d.sum() > 0 else X[int(rng.integers(len(X)))])
    C = np.array(C)
    for _ in range(iters):
        lab_ = np.argmin(((X[:, None, :] - C[None]) ** 2).sum(-1), axis=1)
        C2 = np.array([np.average(X[lab_ == j], axis=0, weights=w[lab_ == j]) if np.any(lab_ == j) else C[j] for j in range(k)])
        if np.allclose(C2, C):
            break
        C = C2
    return np.argmin(((X[:, None, :] - C[None]) ** 2).sum(-1), axis=1), C


HUES = [(0, "red"), (25, "orange"), (50, "yellow"), (75, "olive"), (120, "green"), (170, "teal"), (210, "blue"),
        (270, "purple"), (320, "pink"), (360, "red")]


def colour_name(rgb):
    """a plain name for a pen colour, e.g. 'dark brown', 'light green'"""
    r, g, b = [v / 255 for v in rgb]
    mx, mn = max(r, g, b), min(r, g, b)
    L = (mx + mn) / 2
    if mx - mn < 0.08:
        return "black" if L < 0.18 else "grey" if L < 0.8 else "white"
    d = mx - mn
    h = (60 * ((g - b) / d % 6) if mx == r else 60 * ((b - r) / d + 2) if mx == g else 60 * ((r - g) / d + 4)) % 360
    s = d / (1 - abs(2 * L - 1)) if 0 < L < 1 else 0
    name = min(HUES, key=lambda q: abs(q[0] - h))[1]
    if name in ("orange", "red", "yellow", "olive") and s < 0.55 and L < 0.55:
        name = "brown"
    if name in ("orange", "red") and L > 0.7:
        name = "peach" if name == "orange" else "pink"
    shade = "dark " if L < 0.3 else "light " if L > 0.72 else ""
    return (shade + name).strip()


# ---------------------------------------------------------------- filling, simplifying, ordering
def simplify(P, tol):
    """Douglas-Peucker on an Nx2 polyline"""
    if len(P) < 3:
        return P
    keep = np.zeros(len(P), bool); keep[0] = keep[-1] = True
    stack = [(0, len(P) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        A, B = P[a], P[b]
        d = B - A; L = math.hypot(*d)
        seg = P[a + 1:b] - A
        dist = np.abs(seg[:, 0] * d[1] - seg[:, 1] * d[0]) / L if L > 1e-12 else np.hypot(seg[:, 0], seg[:, 1])
        k = int(np.argmax(dist))
        if dist[k] > tol:
            m = a + 1 + k; keep[m] = True; stack += [(a, m), (m, b)]
    return P[keep]


def composite(fills, lo, W, H):
    """paint the filled shapes in the SVG's own order into a W x H grid (RES mm, from lo): -> per pixel the index of the
    colour that shows there (-1 = nothing), the colours (rgb), the pixels each covers"""
    from PIL import Image, ImageDraw
    label = np.full((H, W), -1, np.int16)
    cols = {}
    for polys, colour in fills:
        if not polys:
            continue
        rgb = rgb_of(colour)
        ci = cols.setdefault(rgb, len(cols))
        q = [(p - lo) / RES for p, _ in polys]
        allq = np.vstack(q)
        x0, y0 = np.maximum(np.floor(allq.min(0)).astype(int) - 1, 0)
        x1, y1 = np.minimum(np.ceil(allq.max(0)).astype(int) + 2, [W, H])
        if x1 <= x0 or y1 <= y0:
            continue
        m = np.zeros((y1 - y0, x1 - x0), bool)
        for p in q:                                # even-odd: each outline toggles what it encloses
            if len(p) < 3:
                continue
            img = Image.new("1", (int(x1 - x0), int(y1 - y0)), 0)
            ImageDraw.Draw(img).polygon([(v[0] - x0, v[1] - y0) for v in p], fill=1)
            m ^= np.array(img, bool)
        label[y0:y1, x0:x1][m] = ci                # later shapes cover earlier ones, as the SVG paints
    rgbs = np.array(sorted(cols, key=cols.get), float).reshape(-1, 3)
    area = np.bincount(label[label >= 0].ravel(), minlength=len(rgbs))
    return label, rgbs, area


SMOOTH = 0.5                 # mm: the fill lines are smoothed this much (their 0.1 mm pixel stair-steps out)


def smooth(P):
    """a line traced off the pixel grid -> the same line without its pixel stair-steps: resampled every 0.25 mm and
    Gaussian-smoothed (SMOOTH mm; closed loops all round). Without it every stair-step that turns past the viewer's
    TURN_SPLIT is a stop: the couple took 67 000 moves and 9.4 h; the arm runs each move as fast as its joints allow."""
    from scipy.ndimage import gaussian_filter1d
    P = np.asarray(P, float)
    seg = np.hypot(*np.diff(P, axis=0).T)
    L = float(seg.sum())
    if L < 1.0 or len(P) < 4:
        return P
    closed = np.allclose(P[0], P[-1], atol=1e-6)
    s_ = np.r_[0, np.cumsum(seg)]
    n = max(4, int(L / 0.25))
    t = np.linspace(0, L, n + 1)
    Q = np.c_[np.interp(t, s_, P[:, 0]), np.interp(t, s_, P[:, 1])]
    sig = SMOOTH / 0.25
    if closed:
        Q = Q[:-1]
        Q = np.c_[gaussian_filter1d(Q[:, 0], sig, mode="wrap"), gaussian_filter1d(Q[:, 1], sig, mode="wrap")]
        return np.vstack([Q, Q[:1]])
    return np.c_[gaussian_filter1d(Q[:, 0], sig, mode="nearest"), gaussian_filter1d(Q[:, 1], sig, mode="nearest")]


def mask_lines(mask, lo, pitch):
    """a filled area (bool grid) -> nested fill lines: contours of the distance to its edge, EDGE_IN + k * pitch"""
    from scipy.ndimage import distance_transform_edt
    import contourpy
    if not mask.any():
        return []
    dist = distance_transform_edt(mask) * RES
    H, W = mask.shape
    gen = contourpy.contour_generator(x=lo[0] + np.arange(W) * RES, y=lo[1] + np.arange(H) * RES, z=dist)
    out = []
    for lv in np.arange(EDGE_IN, dist.max(), pitch):
        for line in gen.lines(lv):
            if len(line) >= 3 and np.sum(np.hypot(*np.diff(line, axis=0).T)) > 2 * pitch:
                out.append(line)
    return out


def order(lines, start):
    """nearest-next: closed loops start where they are nearest, open lines from their nearer end"""
    if not lines:
        return []
    ends = np.array([[L[0][0], L[0][1], L[-1][0], L[-1][1]] for L in lines])
    closed = np.array([np.allclose(L[0], L[-1], atol=1e-6) for L in lines])
    todo = np.ones(len(lines), bool); out = []; at = np.array(start, float)
    while todo.any():
        idx = np.flatnonzero(todo)
        dd = np.minimum(np.hypot(ends[idx, 0] - at[0], ends[idx, 1] - at[1]), np.hypot(ends[idx, 2] - at[0], ends[idx, 3] - at[1]))
        best = None
        for k in idx[np.argsort(dd)[:8]]:          # a closed loop can start anywhere: its nearest point, among the nearest few
            L = lines[k]
            if closed[k]:
                d = np.hypot(L[:, 0] - at[0], L[:, 1] - at[1]); j = int(np.argmin(d)); v = d[j]
            else:
                a0, a1 = math.dist(L[0], at), math.dist(L[-1], at); j = 0 if a0 <= a1 else -1; v = min(a0, a1)
            if best is None or v < best[0]:
                best = (v, k, j)
        _, k, j = best
        L = lines[k]
        if closed[k]:
            L = np.vstack([L[j:-1], L[:j + 1]])
        elif j == -1:
            L = L[::-1]
        out.append(L); at = L[-1]; todo[k] = False
    return out


JOIN = 2.5                   # x pitch: a next line starting this close is reached with the pen down, if the way is ink


def join(lines, mask, lo, pitch):
    """ordered lines -> fewer, longer ones: where the next line starts within JOIN x pitch of where this one ended and the
    straight step between them stays inside the colour's own area (the mask), the pen goes on without lifting - the step
    is drawn over ink that area gets anyway. Nested fill lines are a pitch apart, so most of a fill becomes one stroke."""
    if mask is None or not lines:
        return lines
    H, W = mask.shape

    def inside(a, b):
        for t in np.linspace(0, 1, 7):
            x, y = a + (b - a) * t
            i, j = int(round((y - lo[1]) / RES)), int(round((x - lo[0]) / RES))
            if not (0 <= i < H and 0 <= j < W and mask[i, j]):
                return False
        return True
    out = [lines[0]]
    for L in lines[1:]:
        a, b = out[-1][-1], L[0]
        if math.dist(a, b) <= JOIN * pitch and inside(np.asarray(a), np.asarray(b)):
            out[-1] = np.vstack([out[-1], L])
        else:
            out.append(L)
    return out


def board_frame():
    sc = json.load(open(SCENE, encoding="utf-8"))
    at = {h["part"]: h["at"] for h in sc.get("piece_home", [])}
    a1, h8 = at.get("white rook a1"), at.get("black rook h8")
    c = [(a1[0] + h8[0]) / 2, (a1[2] + h8[2]) / 2] if a1 and h8 else [-362.0, 30.6]
    return sc, c


# Mike 2026-11-06 (missingcouple1930pd.svg, 33 colours): "multi color pens find max colors". The arm can hold as many pens
# as it has stands it reaches for a pen change: every spot of both trays' front rows - 20 (checked with the IK: the stand,
# above it, in front of it on the board side, and TOOL_UP over it). A row-2 stand would have the held pen pass over the
# row-1 pens. So a drawing gets up to MAX_PENS colours: the SVG's own if it has that few, else they are merged (weighted
# k-means in L*a*b*, so close shades merge first and big areas keep their colour). Near-white is the paper: not drawn.
# The shapes are painted in the SVG's own order first, so where one covers another only the top colour is drawn.
MAX_PENS = 20
PAPER_L, PAPER_CHROMA = 92.0, 12.0           # L*a*b*: this light and this grey is the paper's own colour
STANDS = [("white", k) for k in range(10)] + [("black", k) for k in range(10)]


def convert(svg_path, pitch, c, max_pens=MAX_PENS):
    sh = shapes(svg_path, step=1.0)                # a first look for the size
    pts = np.vstack([p for s in sh for p, _ in s["polys"]])
    lo_s, hi_s = pts.min(0), pts.max(0)
    w, h = hi_s[0] - lo_s[0], hi_s[1] - lo_s[1]
    k = min(BOX_W / w, BOX_H / h)                  # mm per SVG unit
    sh = shapes(svg_path, step=0.25 / k)           # again, curves every ~0.25 mm on the paper
    mid = (lo_s + hi_s) / 2

    def mm(p):                                     # SVG (y down) -> mm on the paper (X right, Y up, from its middle)
        return np.c_[(p[:, 0] - mid[0]) * k, -(p[:, 1] - mid[1]) * k]
    fills = [([(mm(p), cl) for p, cl in s["polys"] if len(p) >= 3], s["fill"]) for s in sh if s["fill"]]
    strokes = [(mm(p), s["stroke"]) for s in sh if s["stroke"] for p, _ in s["polys"]]
    lo = np.array([-w * k / 2 - 2, -h * k / 2 - 2])
    W, H = int(math.ceil((w * k + 4) / RES)) + 1, int(math.ceil((h * k + 4) / RES)) + 1
    if fills:
        label, rgbs, area = composite(fills, lo, W, H)
    else:
        label, rgbs, area = np.full((H, W), -1, np.int16), np.zeros((0, 3)), np.zeros(0, int)
    srgb = np.array([rgb_of(cl) for _, cl in strokes], float).reshape(-1, 3)
    allrgb = np.vstack([rgbs, srgb])
    weight = np.r_[area.astype(float), np.full(len(srgb), 1000.0)]   # a stroked line counts as a small area
    Lab = lab(allrgb) if len(allrgb) else np.zeros((0, 3))
    paper = (Lab[:, 0] >= PAPER_L) & (np.hypot(Lab[:, 1], Lab[:, 2]) <= PAPER_CHROMA)
    use = np.flatnonzero(~paper & (weight > 0))
    if len(use) == 0:
        return None
    k_pens = min(max_pens, len(use))
    if k_pens < len(use):
        grp, _ = kmeans(Lab[use], weight[use], k_pens)
    else:
        grp = np.arange(len(use))
    group = np.full(len(allrgb), -1); group[use] = grp
    pens = []
    for g in range(k_pens):
        members = use[grp == g]
        if not len(members):
            continue
        rgb = np.average(allrgb[members], axis=0, weights=np.maximum(weight[members], 1e-9))
        fill_idx = [int(m) for m in members if m < len(rgbs)]
        mask = np.isin(label, fill_idx) if fill_idx else None
        lines = [smooth(q) for q in mask_lines(mask, lo, pitch)] if fill_idx else []
        lines += [p for (p, _), gi in zip(strokes, group[len(rgbs):]) if gi == g]
        lines = [simplify(np.asarray(q, float), SIMPLIFY) for q in lines]
        lines = [q for q in lines if len(q) >= 2]
        if not lines:
            continue
        pens.append({"rgb": rgb, "lines": lines, "mask": mask, "L": float(lab(rgb)[0, 0]), "colours": int(len(members)),
                     "area_mm2": round(float(area[fill_idx].sum()) * RES * RES, 1) if fill_idx else 0.0})
    pens.sort(key=lambda q: -q["L"])                # light colours first, dark ones over their edges last
    drawing = {"src": os.path.relpath(svg_path, ROOT), "size_mm": [round(w * k, 1), round(h * k, 1)], "pitch": pitch,
               "svg_colours": int(len(allrgb)), "paper_colours": int(paper.sum()), "pens": []}
    at = (0.0, BOX_H / 2)
    for i, P in enumerate(pens):
        lines = order(P["lines"], start=at)
        n0 = len(lines)
        lines = join(lines, P["mask"], lo, pitch)
        at = tuple(lines[-1][-1])
        hexc = "#%02x%02x%02x" % tuple(int(round(v)) for v in P["rgb"])
        tray, spot = STANDS[i]
        drawing["pens"].append({"name": f"{colour_name(P['rgb'])} {hexc}", "color": hexc, "tray": tray, "spot": spot,
                                "svg_colours": P["colours"], "area_mm2": P["area_mm2"], "joined_from": n0,
                                # to the board: X across (a-file to h-file = -z), Y toward the robot (+x)
                                "lines": [[[round(c[0] + BOX_DX + y, 2), round(c[1] - x, 2)] for x, y in q] for q in lines]})
    drawing["stats"] = {"pens": len(drawing["pens"]), "lines": sum(len(p["lines"]) for p in drawing["pens"]),
                        "points": sum(len(q) for p in drawing["pens"] for q in p["lines"]),
                        "ink_m": round(sum(float(np.sum(np.hypot(*np.diff(np.array(q), axis=0).T)))
                                           for p in drawing["pens"] for q in p["lines"]) / 1000, 2)}
    return drawing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pitch", type=float, default=0.6, help="mm between fill lines (about the pen's line width)")
    ap.add_argument("--pens", type=int, default=MAX_PENS, help=f"most pens (colours) a drawing may use, up to {MAX_PENS}")
    ap.add_argument("only", nargs="*", help="just these SVG names (default: all)")
    args = ap.parse_args()
    sc, c = board_frame()
    files = sorted(glob.glob(os.path.join(ROOT, "*.svg")) + glob.glob(os.path.join(ROOT, "drawings", "*.svg")))
    if args.only:
        files = [f for f in files if os.path.splitext(os.path.basename(f))[0] in args.only]
    if not files:
        sys.exit("no .svg in robo_chess or robo_chess\\drawings")
    out = dict(sc.get("drawings") or {})
    for f in files:
        name = os.path.splitext(os.path.basename(f))[0]
        d = convert(f, args.pitch, c, min(args.pens, MAX_PENS))
        if not d:
            print(f"{name}: nothing to draw (all paper-white)")
            continue
        out[name] = d
        print(f"{name}: {d['size_mm'][0]} x {d['size_mm'][1]} mm, {d['svg_colours']} colours in the SVG "
              f"({d['paper_colours']} paper-white) -> {d['stats']['pens']} pens, {d['stats']['lines']} lines, "
              f"{d['stats']['ink_m']} m of ink")
        for p in d["pens"]:
            print(f"   {p['tray']} tray {p['spot'] + 1:2d}: {p['name']:24s} {p['svg_colours']:2d} SVG colours, "
                  f"{len(p['lines']):5d} strokes (from {p['joined_from']} lines), {p['area_mm2'] / 100:8.1f} cm2")
    sc["drawings"] = out
    open(SCENE, "w", encoding="utf-8").write(json.dumps(sc, indent=1))
    print("scene.json drawings:", ", ".join(out))


if __name__ == "__main__":
    main()

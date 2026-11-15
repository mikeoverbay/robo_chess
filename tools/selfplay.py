#!/usr/bin/env python3
"""
selfplay.py - Robo Chess: the arm plays a whole game against itself, both sides, micro-Max at its top level
(Mike, 2026-11-05: "just want to see it play itself, both sides of the board ... set to the hard level with the
harder engine"). Robot Studio's "Play game" button then shows the arm making every move.

    python selfplay.py                 a new game (the opening book picks at random; the seed is printed)
    python selfplay.py --seed 7        the same game again
    python selfplay.py --skill 5       an easier level (3..7)
    python selfplay.py --replan        the game in scene.json: only the empty claw's heights again (plan_travel)

Engine: micro-Max 4.8 (umax/umax_cli.exe, built by umax/build_umax.py from Robo Chess's own copy in umax/engine),
loaded with each position as the arm's Mega 2560 would load it (side, castling and en-passant flags). Its clock is the
Mega's: nodes at ~700 a second (README, umaxbench), so SKILL 7's 20 s budget reaches the depth the Mega reaches.
Opening book: umax/BOOK.TXT, a random matching line each move.
Rules: python-chess. The game ends at checkmate, stalemate, or a back-and-forth repetition
(MicroChess's rule: a side undoing its own last move for the third time LOSES) - plus a draw for bare kings / too
little to mate (the device would shuffle on until the repetition rule), and a draw at MAX_PLIES.

Arm: every chess move becomes pick-and-place moves for the viewer - a capture first takes the captured piece to its
colour's tray (white's on the a-file side, black's on the h-file side), castling moves the king then the rook, en
passant lifts the passed pawn, a promotion swaps the pawn for a queen of that side from its tray (its spare - back
2026-11-12 - or a captured one), else any captured piece of that side as a stand-in.
Carry height (Mike, 2026-11-05: "only lift the piece if there are pieces in the way ... if it's an open path, just lift
high enough to move it across the board, maybe 10 mm max"; then "why are we lifting it all the way up above a king
when we're only moving a piece over pawns ... are we not checking what's in the way of the path, and how high those
objects are?"): each carry goes only as high as the pieces near its path need - the lowest height (LOW_LIFT at least)
at which nothing touches anywhere along the path (the same line, or arc round the base, the viewer moves it on):
  - every other piece (board and trays) by its real shape: its widest radius at each height (from its mesh, all round,
    so a knight's nose counts whichever way it faces), padded VGAP above its top and CLEAR sideways
  - the carried piece by its own profile, centred in the claw; the closed claw's three finger bars (6 x 8, measured in
    Robot Studio, turning with the arm's yaw) from 3.3 above the piece's base up to the hub; the hub (r 25) above 106
So a piece slides past its neighbours at LOW_LIFT, clears pawns at a pawn's top + VGAP, a king only when it passes a
king, and a diagonal step between two pieces lifts just until the fingers clear their narrowing sides. "over" lists
the pieces that set the height, in path order. The empty claw (open) between a drop and the next pick goes only
as high as the pieces near its way need, too (plan_travel; Mike 2026-11-06); only the first move, from home, at LIFT.
Every pick and drop, down on the board, at the travel height and at the carry height, and every carry path along its
length, is checked against the viewer's IK (claw vertical, joint ranges).

Out: scene.json "game" (the viewer plays it) and games/selfplay_<date>_seed<N>.pgn.
"""
import argparse
import datetime
import json
import math
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chess  # noqa: E402
import chess.pgn  # noqa: E402

import board_pieces as BP  # noqa: E402

ROOT = os.path.dirname(HERE)
import studio  # noqa: E402
SCENE = studio.scene_path()          # this studio's scene (ROBOT_STUDIO; default main = viewer_session/scene.json)
UMAX = os.path.join(HERE, "umax", "umax_cli.exe")
BOOK = os.path.join(HERE, "umax", "BOOK.TXT")       # Robo Chess's own copy (2026-11-06; was read from Megachess)
GAMES = os.path.join(ROOT, "games")

SKILL_BUDGET_MS = {3: 1000, 4: 2500, 5: 5000, 6: 10000, 7: 20000}   # think budget per level, ms on the Mega
MAX_DEPTH = 20                                                     # the Mega build's
MEGA_NPS = 700.0                                                   # micro-Max nodes a second on the Mega
PAWN = 74                                                          # micro-Max's pawn
MAX_PLIES = 400
LIFT = 125.0                                    # the empty claw's travel height; the most a carry ever goes up
LOW_LIFT = 10.0                                 # a carry with nothing in the way: just clear of the board (Mike)
VGAP, CLEAR = 10.0, 3.0                         # over the top of a piece passed over (Mike: "5 or 10"); at the sides
# The closed claw, measured in Robot Studio (viewerIK.partVerts) once the claw was centred on its piece: three finger
# bars, outlines in the arm's radial frame (u out from the yaw axis, v across), from F_LO above the carried piece's base
# (the lips) up to the hub; the hub (r HUB_R) from HUB_LO up. Everything wider sits 162+ above the base.
BARS = [[(-25.83, -2.92), (-17.83, -2.92), (-17.83, 3.08), (-25.83, 3.08)],
        [(6.57, -17.02), (10.57, -23.92), (15.77, -20.92), (11.77, -14.02)],
        [(6.57, 17.18), (11.77, 14.18), (15.77, 21.08), (10.57, 24.08)]]
F_LO, F_HI, HUB_LO, HUB_R = 3.3, 114.0, 106.0, 25.0
NEAR = 26.4 + 16.6 + CLEAR + 1.0                # a piece's centre farther than this from the path can't be touched
ZTOP = 240                                      # height profiles: 1 mm steps up to here
R_ROUTE = 160.0                                 # the viewer's: a carry passing closer to the yaw axis goes round in an arc
NAMES = {chess.PAWN: "pawn", chess.KNIGHT: "knight", chess.BISHOP: "bishop", chess.ROOK: "rook",
         chess.QUEEN: "queen", chess.KING: "king"}


# ---------------------------------------------------------------- micro-Max
class Umax:
    def __init__(self):
        if not os.path.isfile(UMAX):
            sys.exit(f"{UMAX} missing: run  python umax/build_umax.py")
        self.p = subprocess.Popen([UMAX], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        assert self.ask("new") == "ok"

    def ask(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        return self.p.stdout.readline().strip()

    def think(self, board, moved, ep, budget):
        """the position as the Mega loads it into micro-Max; -> (uci, depth, score, nodes, ms) or None"""
        codes = []
        for row in range(8):                                       # row 0 = rank 8
            for f in range(8):
                sq = chess.square(f, 7 - row)
                pc = board.piece_at(sq)
                codes.append(code_of(pc, row, moved.get(sq, False)) if pc else 0)
        r = self.ask(f"think {budget} {MAX_DEPTH} {MEGA_NPS} {'w' if board.turn else 'b'} {ep} "
                     + " ".join(map(str, codes)))
        if not r.startswith("move"):
            return None
        _, fr, to, depth, score, nodes, ms = r.split()
        return sq88(int(fr)) + sq88(int(to)), int(depth), int(score), int(nodes), int(ms)

    def close(self):
        try:
            self.ask("quit")
        except OSError:
            pass
        self.p.wait(timeout=5)


def code_of(pc, row, moved):
    """micro-Max piece code, flags as umax_pick() sets them"""
    white = pc.color == chess.WHITE
    t = pc.piece_type
    if t == chess.PAWN:
        c = 1 if white else 2
        if row != (6 if white else 1):
            c |= 32                                                # off its start rank: moved
        if row == (2 if white else 5):
            c |= 64                                                # on its 6th rank
        if row == (1 if white else 6):
            c |= 128                                               # on its 7th
    elif t == chess.KNIGHT:
        c = 3 | 32
    elif t == chess.BISHOP:
        c = 5 | 32
    elif t == chess.QUEEN:
        c = 7 | 32
    elif t == chess.ROOK:
        c = 6 | (32 if moved else 0)                               # castling rights
    else:
        c = 4 | (32 if moved else 0)
    return c | (8 if white else 16)


def sq88(s):
    return "abcdefgh"[s & 7] + str(8 - (s >> 4))


def ep_88(board):
    """the en-passant square as micro-Max takes it: the last move was a pawn's double step"""
    if not board.move_stack:
        return 128
    m = board.peek()
    pc = board.piece_at(m.to_square)
    dr = chess.square_rank(m.to_square) - chess.square_rank(m.from_square)
    if pc and pc.piece_type == chess.PAWN and abs(dr) == 2:
        row = 7 - (chess.square_rank(m.from_square) + dr // 2)
        return row * 16 + chess.square_file(m.to_square)
    return 128


# ---------------------------------------------------------------- book
def load_book():
    lines = []
    for raw in open(BOOK, encoding="utf-8"):
        raw = raw.split("#")[0].split()
        if raw:
            lines.append(raw)
    return lines


# ---------------------------------------------------------------- MicroChess's repetition rule
def would_repeat(history, mv):
    """history newest first [(from, to)], 5 kept. True: this move undoes the side's last move, which undid the one
    before (chessutil.cpp would_repeat)"""
    if len(history) < 5:
        return False
    m = mv
    for i in (1, 3):
        if history[i][1] == m[0] and history[i][0] == m[1]:
            m = history[i]
        else:
            return False
    return True


# ---------------------------------------------------------------- the viewer's IK (claw vertical), for reach checks
class Reach:
    def __init__(self, sc):
        self.ik = sc["ik"]
        self.rng = {j["name"]: (j["min"], j["max"]) for j in sc["joints"]}

    def ok(self, T):
        a = self.ik
        L, ax, tool = a["link"], a["yaw_axis"], a["tool"]
        dx, dz = T[0] - ax[0], T[2] - ax[1]
        rho = math.hypot(dx, dz)
        yaw = math.degrees(math.atan2(-dz, -dx))
        wx, wz = rho - tool[0], -T[1] - tool[1]
        d, phi = math.hypot(wx, wz), math.atan2(wz, wx)
        r2, r3 = self.rng[a["chain"][1]], self.rng[a["chain"][2]]
        lim = math.radians(min(-r2[0], -r3[0]))
        good = True
        c = (d / L - 1) / 2
        if c > 1:
            c, good = 1.0, False
        q = -math.acos(max(-1.0, c))
        if -q <= lim + 1e-9:
            q1, q2, q3 = phi - q, q, q
        else:
            q3 = -lim
            l2 = 2 * L * math.cos(lim / 2)
            cb = (d * d - L * L - l2 * l2) / (2 * L * l2)
            if cb < -1:
                cb, good = -1.0, False
            b = -math.acos(min(1.0, cb))
            q1 = phi - math.atan2(l2 * math.sin(b), L + l2 * math.cos(b))
            q2 = b + lim / 2
        ang = {a["yaw"]: yaw, a["chain"][0]: math.degrees(q1), a["chain"][1]: math.degrees(q2),
               a["chain"][2]: math.degrees(q3)}
        ang[a["wrist"]] = a["wrist_sum"] - (ang[a["chain"][0]] + ang[a["chain"][1]] + ang[a["chain"][2]])
        for n, v in ang.items():
            lo, hi = self.rng[n]
            if v < lo - 1e-6 or v > hi + 1e-6:
                good = False
        return good


# ---------------------------------------------------------------- the carry path, as the viewer moves it
def axis_dist(a, b, ax):
    """closest the segment a-b comes to the yaw axis, in plan (viewer axisDist)"""
    px, pz, dx, dz = a[0] - ax[0], a[2] - ax[1], b[0] - a[0], b[2] - a[2]
    L2 = dx * dx + dz * dz
    t = max(0.0, min(1.0, -(px * dx + pz * dz) / L2)) if L2 > 1e-9 else 0.0
    return math.hypot(px + dx * t, pz + dz * t)


def path_at(a, b, e, ax):
    """the grip point a share e along a carry from a to b (viewer hop + pathAt): a line, or an arc round the base"""
    if axis_dist(a, b, ax) < R_ROUTE:
        def yaw_of(q):
            return math.atan2(-(q[2] - ax[1]), -(q[0] - ax[0]))
        ra, rb = math.hypot(a[0] - ax[0], a[2] - ax[1]), math.hypot(b[0] - ax[0], b[2] - ax[1])
        y = yaw_of(a) + (yaw_of(b) - yaw_of(a)) * e
        r = ra + (rb - ra) * e
        return [ax[0] - math.cos(y) * r, a[1] + (b[1] - a[1]) * e, ax[1] - math.sin(y) * r]
    return [u + (v - u) * e for u, v in zip(a, b)]


def path_pts(a, b, ax, step=1.0):
    n = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[2] - a[2]) * 1.6 / step)))   # 1.6: an arc is longer
    return [path_at(a, b, k / n, ax) for k in range(n + 1)]


# ---------------------------------------------------------------- how high a carry has to go
_PROF, _PAD, _FPTS = {}, {}, None


def profile(kind):
    """a piece type's widest radius (pedestal + piece) in each 1 mm of height, from its mesh (viewer_session/
    piece_<kind>.stl, base centre at the origin, design frame: up is -y) - vertices + an even surface sample"""
    if kind not in _PROF:
        import numpy as np
        import trimesh
        m = trimesh.load(os.path.join(HERE, "viewer_session", f"piece_{kind}.stl"), force="mesh")
        pts = np.vstack([m.vertices, trimesh.sample.sample_surface_even(m, 300000, seed=1)[0]])
        up, r = -pts[:, 1], np.hypot(pts[:, 0], pts[:, 2])
        prof = np.zeros(ZTOP)
        np.maximum.at(prof, np.clip(np.floor(up).astype(int), 0, ZTOP - 1), r)
        _PROF[kind] = prof
    return _PROF[kind]


def padded(kind):
    """profile() widened to the widest within VGAP above or below each height: the gap kept over a top (one band more:
    the profile is in 1 mm bands and the lift in 0.5 steps - with VGAP bands alone the gap came out 9.3)"""
    if kind not in _PAD:
        import numpy as np
        p, g = profile(kind), int(math.ceil(VGAP)) + 1
        _PAD[kind] = np.array([p[max(0, z - g):z + g + 1].max() for z in range(ZTOP)])
    return _PAD[kind]


def finger_pts():
    """the finger bars' outlines every 0.25 mm, (u, v)"""
    global _FPTS
    if _FPTS is None:
        import numpy as np
        out = []
        for b in BARS:
            for i in range(4):
                a, c = np.array(b[i]), np.array(b[(i + 1) % 4])
                n = int(np.ceil(np.linalg.norm(c - a) / 0.25))
                out += [a + (c - a) * s / n for s in range(n)]
        _FPTS = np.array(out)
    return _FPTS


def carry_height(kind, a, b, others, ax):
    """the lowest lift (LOW_LIFT .. LIFT, 0.5 steps) at which a 'kind' piece carried from a to b touches nothing:
    others = [(where, kind, position)]. -> (lift, [where of the pieces that set it, in path order])"""
    import numpy as np
    held = profile(kind)
    P = np.array(path_pts(a, b, ax, 1.0))[:, [0, 2]]
    u = P - np.array(ax)
    u /= np.linalg.norm(u, axis=1)[:, None]
    v = np.stack([-u[:, 1], u[:, 0]], 1)
    F = finger_pts()
    W = P[:, None, :] + F[None, :, 0, None] * u[:, None, :] + F[None, :, 1, None] * v[:, None, :]
    z = np.arange(ZTOP)
    need = []
    for where, k2, q in others:
        q2 = np.array([q[0], q[2]])
        dP = np.linalg.norm(P - q2, axis=1)
        i, D = int(dP.argmin()), float(dP.min())
        if D > NEAR:
            continue
        dF = float(np.min(np.linalg.norm(W - q2, axis=2)))           # nearest any finger comes to its centre line
        ro = padded(k2)
        for L in np.arange(LOW_LIFT, LIFT + 0.01, 0.5):
            hz = z - L                                                # height above the carried piece's base
            rh = np.where(hz >= 0, held[np.clip(hz.astype(int), 0, ZTOP - 1)], 0.0)
            on = ro > 0
            if not (np.any(on & (rh > 0) & (rh + ro + CLEAR > D))                       # the carried piece
                    or np.any(on & (hz >= F_LO) & (hz <= F_HI) & (ro + CLEAR > dF))     # the fingers
                    or np.any(on & (hz >= HUB_LO) & (HUB_R + ro + CLEAR > D))):         # the hub
                break
        if L > LOW_LIFT:
            need.append((i, float(L), where))
    return max([LOW_LIFT] + [n[1] for n in need]), [w for _, _, w in sorted(need)]


# ---------------------------------------------------------------- how high the EMPTY claw has to go
# Mike 2026-11-06: "If it's moving from a piece to go pick up a piece, it should only lift high enough to pick that
# piece up unless there's something in the way ... All that extra travel really makes it take longer." The empty claw
# went up to LIFT (125) between every drop and the next pick; now it goes only as high as the pieces near its way need.
# It travels OPEN. The open claw, measured in Robot Studio (claw open over e4, viewerIK.partVerts every 0.5 mm along
# the finger meshes, per 1 mm of height above the grip point): three identical straight finger bars (within 0.14 mm)
# at 180 / -60 / +60 deg from u (out from the yaw axis), each 6.0 across, 8.1 deep, leaning in 6 deg toward the top -
# outer face at u = -(37.45 - 0.10475 (z - 5)) - with the lips (z 3..5) reaching in to u -26.0. The hub (r HUB_R)
# from HUB_LO as in the closed claw; everything else on the claw sits 129+ up, over any piece's top + VGAP.
OPEN_LO, OPEN_HI, OPEN_SLOPE, OPEN_DEEP, OPEN_HALF = 3.0, 124.0, 0.10475, 8.12, 3.01
_OPEN = None


def open_finger_pts():
    """the open fingers' outlines, every 0.5 mm round and every 1 mm up: (u, v, z above the grip point)"""
    global _OPEN
    if _OPEN is None:
        import numpy as np
        out = []
        for z in np.arange(OPEN_LO, OPEN_HI + 0.01, 1.0):
            uo = -(37.45 - OPEN_SLOPE * (z - 5.0)) if z >= 5 else -37.56 + (3.49 if z < 4 else 0.0)
            ui = -26.0 if z < 5 else uo + OPEN_DEEP
            corners = [(uo, -OPEN_HALF), (ui, -OPEN_HALF), (ui, OPEN_HALF), (uo, OPEN_HALF)]
            ring = []
            for i in range(4):
                a, c = np.array(corners[i]), np.array(corners[(i + 1) % 4])
                n = max(1, int(np.ceil(np.linalg.norm(c - a) / 0.5)))
                ring += [a + (c - a) * s / n for s in range(n)]
            ring = np.array(ring)
            for deg in (0.0, 120.0, -120.0):                 # finger0 on -u; the others a third of a turn round
                t = math.radians(deg)
                rot = ring @ np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
                out.append(np.column_stack([rot, np.full(len(rot), z)]))
        _OPEN = np.vstack(out)
    return _OPEN


def empty_height(a, b, others, ax):
    """the lowest height (LOW_LIFT .. LIFT, 0.5 steps) the empty, open claw's grip point can travel at from over a
    (where it just let go) to over b (the next piece to pick up) touching nothing: others = [(where, kind, position)]
    - every piece as it stands then, the one just set down at a and the one to pick up at b included (the claw has to
    lift its fingers off the one and over the other). -> (height, [where of the pieces that set it, in path order])"""
    import numpy as np
    P = np.array(path_pts(a, b, ax, 1.0))[:, [0, 2]]
    u = P - np.array(ax)
    u /= np.linalg.norm(u, axis=1)[:, None]
    v = np.stack([-u[:, 1], u[:, 0]], 1)
    F = open_finger_pts()
    reach = float(np.max(np.hypot(F[:, 0], F[:, 1])))
    z = np.arange(ZTOP)
    need = []
    for where, k2, q in others:
        q2 = np.array([q[0], q[2]])
        dP = np.linalg.norm(P - q2, axis=1)
        ro = padded(k2)
        near = dP <= reach + float(ro.max()) + CLEAR + 1.0
        if not near.any():
            continue
        i, D = int(dP.argmin()), float(dP.min())
        W = P[near, None, :] + F[None, :, 0, None] * u[near, None, :] + F[None, :, 1, None] * v[near, None, :]
        dk = np.linalg.norm(W - q2, axis=2).min(axis=0)       # nearest each finger point comes to its centre line
        for L in np.arange(LOW_LIFT, LIFT + 0.01, 0.5):
            zi = np.clip((F[:, 2] + L).astype(int), 0, ZTOP - 1)
            rk = ro[zi]
            hz = z - L
            if not (np.any((rk > 0) & (rk + CLEAR > dk))                                  # the fingers
                    or np.any((ro > 0) & (hz >= HUB_LO) & (HUB_R + ro + CLEAR > D))):     # the hub
                break
        if L > LOW_LIFT:
            need.append((i, float(L), where))
    return max([LOW_LIFT] + [n[1] for n in need]), [w for _, _, w in sorted(need)]


def plan_travel(moves, ax, loop=False):
    """each move's travel (the empty claw's height on its way in from the last drop) and leave (the height it lifts off
    to after its drop: the next move's travel), from where every piece stands at the time. The first move comes from
    home at LIFT (the viewer keeps that). loop: the list ends where it began and plays again, so the last move leaves
    for the first at the height that hop needs, and the first one's travel is that (after the first time round)."""
    pos, kind, label = {}, {}, {}
    for part, k, colour, sq, at, yaw in BP.setup():
        pos[part], kind[part] = list(at), k
    for f in "abcdefgh":
        for r in "12345678":
            label[tuple(round(c, 1) for c in BP.square_at(f + r))] = f + r
    for c in ("white", "black"):
        for n, s in enumerate(BP.tray_spots(c)):
            label[tuple(round(x, 1) for x in s)] = f"{c} tray {n + 1}"

    def where(q):
        return label.get(tuple(round(c, 1) for c in q), "?")

    def hop(src, mv):
        others = [(where(q), kind[p], q) for p, q in pos.items()]
        return empty_height(src, mv["from"], others, ax)

    prev = None
    for k, mv in enumerate(moves):
        at = pos[mv["part"]]
        assert math.dist(at, mv["from"]) < 0.05, f"{mv.get('name')}: {mv['part']} is at {at}, not {mv['from']}"
        if prev is None:
            mv["travel"], mv["travel_over"] = LIFT, []
        else:
            h, way = hop(prev["to"], mv)
            mv["travel"], mv["travel_over"] = round(h, 1), way
            prev["leave"] = mv["travel"]
        pos[mv["part"]] = list(mv["to"])
        prev = mv
    if moves:
        moves[-1]["leave"] = moves[-1]["travel"]
    if loop and moves:
        assert all(math.dist(pos[p], at) < 0.05 for p, k, c, s, at, y in BP.setup()), "the list doesn't end where it began"
        h, way = hop(moves[-1]["to"], moves[0])
        moves[0]["travel"], moves[0]["travel_over"] = round(h, 1), way
        moves[-1]["leave"] = moves[0]["travel"]
    return moves


# ---------------------------------------------------------------- the pieces on the table
class Table:
    """which part stands where (squares and tray spots), and the arm moves that change it"""

    def __init__(self):
        self.on = {}                    # square name -> part
        self.moved = set()              # parts that have moved (MicroChess's hasMoved)
        self.spots = {c: BP.tray_spots(c) for c in ("white", "black")}
        self.axis = json.load(open(SCENE, encoding="utf-8"))["ik"]["yaw_axis"]
        self.tray = {"white": {}, "black": {}}       # spot index -> part
        for part, kind, colour, sq, at, yaw in BP.setup():
            if sq:
                self.on[sq] = part
            else:
                self.tray[colour][BP.SPARE_SPOT] = part     # the spare queen
        self.arm = []

    def pos(self, where):
        return BP.square_at(where) if isinstance(where, str) else self.spots[where[0]][where[1]]

    @staticmethod
    def where(w):
        return w if isinstance(w, str) else f"{w[0]} tray {w[1] + 1}"

    @staticmethod
    def kind(part):
        return part.split()[1]                      # "white pawn e2" -> pawn, "black queen spare" -> queen

    def lift(self, part, a, b, what):
        pa, pb = self.pos(a), self.pos(b)
        others = [(sq, self.kind(p), self.pos(sq)) for sq, p in self.on.items() if p != part]
        others += [((c, i), self.kind(p), self.pos((c, i))) for c in self.tray for i, p in self.tray[c].items()
                   if p != part]
        h, way = carry_height(self.kind(part), pa, pb, others, self.axis)
        self.arm.append({"part": part, "from": pa, "to": pb, "lift": round(h, 1), "travel": LIFT,
                         "over": [self.where(w) for w in way], "what": what, "fromSq": self.where(a),
                         "toSq": self.where(b)})

    def free_spot(self, colour):
        for i in range(len(self.spots[colour])):
            if i not in self.tray[colour]:
                return i
        raise RuntimeError(f"{colour}'s tray is full")

    def off(self, sq, colour):          # a piece off the board into its colour's tray
        part = self.on.pop(sq)
        i = self.free_spot(colour)
        self.tray[colour][i] = part
        self.lift(part, sq, (colour, i), f"{part.rsplit(' ', 1)[0]} off to the tray")

    def go(self, a, b):
        part = self.on.pop(a)
        self.on[b] = part
        self.moved.add(part)
        self.lift(part, a, b, f"{part.rsplit(' ', 1)[0]} {a}-{b}")

    def queen_in(self, sq, colour, only):   # a promotion: a queen of that colour from its tray (the spare, or a captured
        pick = [i for i in only if " queen" in self.tray[colour][i]]       # one); only: the spots filled before the
        if not pick:                                                        # pawn went in
            pick = list(only)                              # no queen there: any piece of that colour stands in for it
        i = sorted(pick)[0]
        part = self.tray[colour].pop(i)
        self.on[sq] = part
        self.moved.add(part)
        self.lift(part, (colour, i), sq, f"{part.rsplit(' ', 1)[0]} in as the new queen")

    def moved_by_square(self, board):
        return {sq: self.on.get(chess.square_name(sq)) in self.moved for sq in chess.SQUARES if board.piece_at(sq)}


def play(seed, skill):
    rnd = random.Random(seed)
    book = load_book()
    board = chess.Board()
    table = Table()
    eng = Umax()
    budget = SKILL_BUDGET_MS[skill]
    history, plies, result, reason = [], [], None, None
    try:
        while True:
            n = len(board.move_stack)
            done = [m.uci() for m in board.move_stack]
            mv, info, think = None, "", 0.0
            line = [ln for ln in book if len(ln) > n and ln[:n] == done]
            if line:
                cand = chess.Move.from_uci(rnd.choice(line)[n])
                if cand in board.legal_moves:
                    mv, info = cand, "book"
            if mv is None:
                r = eng.think(board, table.moved_by_square(board), ep_88(board), budget)
                if r:
                    uci, depth, score, nodes, ms = r
                    pc = board.piece_at(chess.parse_square(uci[:2]))
                    if pc and pc.piece_type == chess.PAWN and uci[3] in "18":
                        uci += "q"                                 # micro-Max promotes to a queen
                    cand = chess.Move.from_uci(uci)
                    if cand in board.legal_moves:
                        mv, think = cand, ms / 1000.0
                        info = f"micro-Max depth {depth}, {score / PAWN:+.2f}, {nodes} nodes, {ms / 1000:.1f} s on the Mega"
                if mv is None:                                     # no engine move: pick one
                    legal = list(board.legal_moves)
                    caps = [m for m in legal if board.is_capture(m)]
                    mv = rnd.choice(caps or legal)
                    info = "fallback (micro-Max gave no legal move)"
                    print("  !! micro-Max gave no legal move at ply", n + 1)
            # the arm's moves for it
            colour = "white" if board.turn else "black"
            other = "black" if board.turn else "white"
            frm, to = chess.square_name(mv.from_square), chess.square_name(mv.to_square)
            first = len(table.arm)
            if board.is_en_passant(mv):
                table.off(to[0] + frm[1], other)
            elif board.is_capture(mv):
                table.off(to, other)
            if board.is_castling(mv):
                kingside = chess.square_file(mv.to_square) > chess.square_file(mv.from_square)
                r0, r1 = ("h", "f") if kingside else ("a", "d")
                table.go(frm, to)
                table.go(r0 + frm[1], r1 + frm[1])
            else:
                table.go(frm, to)
            if mv.promotion and table.tray[colour]:                # (an empty tray - the spare used, nothing else of
                was = set(table.tray[colour])                      #  theirs taken: the pawn stays as the queen)
                table.off(to, colour)                              # the pawn leaves ...
                table.queen_in(to, colour, was)                    # ... and a captured queen (or a stand-in) comes in
            num = board.fullmove_number
            label = f"{num}{'.' if board.turn else '...'} {board.san(mv)}"
            for k, a in enumerate(table.arm[first:]):
                a["name"] = f"{label}  {a['what']}" + (f" ({k + 1}/{len(table.arm) - first})" if len(table.arm) - first > 1 else "")
                a["ply"] = n + 1
                mark_ply(a, colour, label, frm, to)
                if k == 0:
                    a["info"] = info
                    if think > 0:
                        a["think"] = round(think, 2)
            plies.append({"ply": n + 1, "san": label, "info": info})
            print(f"{label:16s} {info}")
            rep = would_repeat(history, (frm, to))
            history.insert(0, (frm, to))
            del history[5:]
            board.push(mv)
            if board.is_checkmate():
                result, reason = ("1-0" if colour == "white" else "0-1"), "checkmate"
            elif board.is_stalemate():
                result, reason = "1/2-1/2", "stalemate"
            elif rep:
                result, reason = ("0-1" if colour == "white" else "1-0"), f"repetition ({colour} repeated, MicroChess's rule)"
            elif board.is_insufficient_material():
                result, reason = "1/2-1/2", "too little left to mate"
            elif len(board.move_stack) >= MAX_PLIES:
                result, reason = "1/2-1/2", "move limit"
            if result:
                break
    finally:
        eng.close()
    return board, table, plies, result, reason


def mark_ply(a, side, label, frm, to):
    """what the viewer shows while the arm makes this chess move (Mike 2026-11-06: who's thinking, how long, the move it
    decided on; its from and to squares flash until the arm is done): side, the move, its squares and their centres"""
    a["side"], a["san"], a["msq"] = side, label, [frm, to]
    a["msq_at"] = [[round(v, 2) for v in BP.square_at(frm)], [round(v, 2) for v in BP.square_at(to)]]


def mark_plies(game):
    """mark_ply for a game already in scene.json, from its moves (pgn) and each arm move's ply"""
    import io
    g = chess.pgn.read_game(io.StringIO(game["pgn"]))
    board, by_ply = g.board(), {}
    for n, mv in enumerate(g.mainline_moves(), 1):
        side = "white" if board.turn else "black"
        label = f"{board.fullmove_number}{'.' if board.turn else '...'} {board.san(mv)}"
        by_ply[n] = (side, label, chess.square_name(mv.from_square), chess.square_name(mv.to_square))
        board.push(mv)
    for a in game["moves"]:
        mark_ply(a, *by_ply[a["ply"]])
        assert a["name"].startswith(a["san"] + "  "), (a["name"], a["san"])


def reach_check(moves, sc, ax):
    """every pick and drop - down, at the carry height, at the travel heights in and out - every carry path and every
    empty hop from one drop to the next pick, against the viewer's IK (claw vertical, joint ranges)"""
    reach = Reach(sc)
    bad = []

    def up(p, h):
        return [p[0], p[1] - h, p[2]]
    prev = None
    for a in moves:
        for p, hs in ((a["from"], (0.0, a["lift"], a["travel"])), (a["to"], (0.0, a["lift"], a.get("leave", a["travel"])))):
            for h in hs:
                if not reach.ok(up(p, h)):
                    bad.append((a["name"], [round(v, 1) for v in p], h))
        miss = [p for p in path_pts(up(a["from"], a["lift"]), up(a["to"], a["lift"]), ax, 5.0) if not reach.ok(p)]
        if miss:
            bad.append((a["name"], "carry path", len(miss)))
        if prev is not None:
            miss = [p for p in path_pts(up(prev["to"], a["travel"]), up(a["from"], a["travel"]), ax, 5.0)
                    if not reach.ok(p)]
            if miss:
                bad.append((a["name"], "empty hop", len(miss)))
        prev = a
    return bad


def travel_summary(moves):
    t = [a["travel"] for a in moves[1:]]
    print(f"empty claw between moves: {min(t):g}..{max(t):g} mm up (was {LIFT:g} every time), mean {sum(t) / len(t):.1f}")


def replan():
    """the game already in scene.json: new travel / leave heights, the engine not run again"""
    sc = json.load(open(SCENE, encoding="utf-8"))
    ax = sc["ik"]["yaw_axis"]
    mv = sc["game"]["moves"]
    mark_plies(sc["game"])
    plan_travel(mv, ax)
    bad = reach_check(mv, sc, ax)
    sc["game"]["reach_ok"] = not bad
    print(f"game: {len(mv)} arm moves re-planned; reach " + ("OK" if not bad else f"{len(bad)} OUT OF REACH"))
    for b in bad[:10]:
        print("   ", b)
    travel_summary(mv)
    open(SCENE + ".tmp", "w", encoding="utf-8").write(json.dumps(sc, indent=1)); os.replace(SCENE + ".tmp", SCENE)   # whole or not at all: the studio reads it every second


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--skill", type=int, default=7, choices=sorted(SKILL_BUDGET_MS))
    ap.add_argument("--replan", action="store_true", help="only re-plan the empty claw's heights of the game in scene.json")
    args = ap.parse_args()
    if args.replan:
        return replan()
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(1, 100000)
    print(f"seed {seed}, SKILL {args.skill} (micro-Max, {SKILL_BUDGET_MS[args.skill] / 1000:g} s budget on the Mega)")
    board, table, plies, result, reason = play(seed, args.skill)
    print(f"\n{result} - {reason}, {len(board.move_stack)} plies, {len(table.arm)} arm moves")

    sc = json.load(open(SCENE, encoding="utf-8"))
    plan_travel(table.arm, table.axis)
    bad = reach_check(table.arm, sc, table.axis)
    print("reach: every pick and drop (down, carried, travelling) and every carry and empty hop,"
          + (" in reach" if not bad else f" {len(bad)} OUT OF REACH"))
    for b in bad[:10]:
        print("   ", b)
    travel_summary(table.arm)
    low = [a for a in table.arm if not a["over"]]
    up = sorted(round(a["lift"]) for a in table.arm if a["over"])
    print(f"carries: {len(low)} slid across {LOW_LIFT:g} mm up (nothing in the way), {len(up)} lifted just over what "
          f"is in their way: {up[0] if up else '-'}..{up[-1] if up else '-'} mm "
          f"({sum(1 for x in up if x < 50)} under 50, {sum(1 for x in up if x > 100)} over 100)")

    think_total = sum(a.get("think", 0.0) for a in table.arm)
    game = chess.pgn.Game.from_board(board)
    title = f"micro-Max 4.8 (Robo Chess SKILL {args.skill})"
    game.headers.update({"Event": "Robot arm self-play (Robot Studio)", "Site": "robo_chess", "White": title,
                         "Black": title, "Result": result, "Date": datetime.date.today().strftime("%Y.%m.%d"),
                         "Termination": reason, "Annotator": f"selfplay.py seed {seed}"})
    os.makedirs(GAMES, exist_ok=True)
    pgn_path = os.path.join(GAMES, f"selfplay_{datetime.datetime.now():%Y%m%d_%H%M%S}_seed{seed}.pgn")
    open(pgn_path, "w", encoding="utf-8").write(str(game) + "\n")
    print("pgn ->", pgn_path)

    sc["game"] = {"title": f"{title} plays itself", "seed": seed, "skill": args.skill, "result": result,
                  "reason": reason, "plies": len(board.move_stack), "think_s": round(think_total, 1),
                  "pgn": str(game.mainline_moves()), "moves": table.arm, "reach_ok": not bad}
    open(SCENE + ".tmp", "w", encoding="utf-8").write(json.dumps(sc, indent=1)); os.replace(SCENE + ".tmp", SCENE)   # whole or not at all: the studio reads it every second
    print(f"scene.json game: {len(table.arm)} arm moves; the Mega would think {think_total / 60:.1f} min in all")


if __name__ == "__main__":
    main()

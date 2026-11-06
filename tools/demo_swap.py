#!/usr/bin/env python3
"""
demo_swap.py - the moves Robot Studio's Demo button plays: white and black change sides, then change back, and the
viewer loops it until Home is pushed.

Mike, 2026-11-06: "I want it to swap the pieces ... go down the row, move the pawns up by one, and then go and move the
pawns from the black all the way down and put them where the white pawns were, and then move the white pawns back to
where the black pawns were. We do the same thing with the royalty row. Move them up two places in front of the pawns,
and then bring the pieces from the back all the way up to replace the ones, and then take the royalty row that we moved
the first time and move it back to the other side ... by rows ... be faster." Then: "I just want this to repeat until
the home button is pushed" and "I want the demo to work like it's playing chess. Just lift the pieces high enough to
make the moves." Then the order, file by file (2026-11-06): "move the pawns up one row. And then we're going to grab
the black pawn ... bring it down and ... put it where the white one was originally. And then we're going to grab the
pawn that's right in front of it. And bring it back. It'll save us an extra trip ... do the same thing with the Royals."

One swap, 48 moves (near = the side on ranks 1-2, far = the side on 7-8):
    1  near pawns up one, rank 2 -> 3, a -> h
    2  file by file, h -> a: the far pawn x7 -> x2 (into the near pawn's old square), then the near pawn right in front
       of it, x3 -> x7, back into the far pawn's - the claw lets go at x2 and picks up next door at x3, and lets go at x7
       next door to the next file's far pawn
    3  near back rank up two, rank 1 -> 3 (in front of the pawns), a -> h
    4  file by file, h -> a: the far piece x8 -> x1, then the near one x3 -> x8
The sweeps up end where the file-by-file steps start (h), and those end at a, where the next sweep starts. A swap leaves
every piece on its own file, its rank mirrored (2 <-> 7, 1 <-> 8); the same squares again put them all back, so two swaps
(96 moves) are one closed loop.

Each carry goes only as high as the pieces near its path need - the same carry_height selfplay.py uses for a game
(the carried piece's and the closed claw's real shapes against every other piece's): a pawn stepping up one slides
across 10 mm up, a carry over a rank of pawns clears the pawns, over the back rank clears the kings. The empty claw
goes only as high as its way needs, too (selfplay.plan_travel: the open claw's fingers over the piece it just let
go of, the one it picks up next and anything between - Mike: "it should only lift high enough to pick that piece up
unless there's something in the way"); the last move leaves for the first at the height that hop needs, so the loop
runs on without a trip home. Every pick, drop, carry and empty hop is checked against the viewer's IK (selfplay's
reach_check).

    python demo_swap.py          writes scene.json "moves"
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import selfplay as SP                                  # noqa: E402  (Table, Reach, path_pts, the carry heights)

FILES = "abcdefgh"


def build():
    t = SP.Table()

    def go(half, step, label, a, b):
        t.go(a, b)
        t.arm[-1]["name"] = f"Swap {half}/2 · {step}/4 {label} · {a}-{b}"

    for half in (1, 2):
        near = t.on["a2"].split()[0]                    # whose pawns stand on rank 2 now
        far = "black" if near == "white" else "white"
        for f in FILES:                                 # 1  near pawns up one
            go(half, 1, f"{near} pawns up one", f"{f}2", f"{f}3")
        for f in FILES[::-1]:                           # 2  file by file: far pawn down, near pawn back up
            go(half, 2, f"pawns change sides, file {f}", f"{f}7", f"{f}2")
            go(half, 2, f"pawns change sides, file {f}", f"{f}3", f"{f}7")
        for f in FILES:                                 # 3  near back rank up two
            go(half, 3, f"{near} back rank up two", f"{f}1", f"{f}3")
        for f in FILES[::-1]:                           # 4  file by file: far piece down, near piece back up
            go(half, 4, f"back ranks change sides, file {f}", f"{f}8", f"{f}1")
            go(half, 4, f"back ranks change sides, file {f}", f"{f}3", f"{f}8")
    return t


def main():
    t = build()
    start = SP.Table().on
    assert t.on == start, "the two swaps don't put every piece back"
    sc = json.load(open(SP.SCENE, encoding="utf-8"))
    SP.plan_travel(t.arm, t.axis, loop=True)
    bad = SP.reach_check(t.arm, sc, t.axis)
    print(f"{len(t.arm)} moves, every piece back where it started after the second swap")
    print("reach: every pick and drop (down, carried, travelling) and every carry and empty hop,"
          + (" in reach" if not bad else f" {len(bad)} OUT OF REACH"))
    for b in bad[:10]:
        print("   ", b)
    for k, (i0, i1) in enumerate(((0, 8), (8, 24), (24, 32), (32, 48))):   # the lifts, step by step (first swap)
        mv = t.arm[i0:i1]
        lifts = sorted({m["lift"] for m in mv})
        over = sorted({w.rstrip("12345678") + "" for m in mv for w in m["over"]})
        tr = sorted(m["travel"] for m in (mv[1:] if k == 0 else mv))      # the very first comes from home
        print(f"  {mv[0]['name'].split(' · ')[1].split(', file')[0]:34s} carry {lifts[0]:g}..{lifts[-1]:g} mm"
              + (f" (over {len({w for m in mv for w in m['over']})} pieces)" if over else " (nothing in the way)")
              + f", empty claw in {tr[0]:g}..{tr[-1]:g} mm")
    SP.travel_summary(t.arm)
    sc["moves"] = [{k: m[k] for k in ("name", "part", "fromSq", "toSq", "from", "to", "lift", "travel", "leave", "over",
                                      "travel_over")}
                   for m in t.arm]
    open(SP.SCENE, "w", encoding="utf-8").write(json.dumps(sc, indent=1))
    print(f"scene.json moves: {len(t.arm)} (the Demo button loops them until Home)")


if __name__ == "__main__":
    main()

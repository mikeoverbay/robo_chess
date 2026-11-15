#!/usr/bin/env python3
"""
web_export.py - Robot Studio as a folder of plain files for a website (Mike 2026-11-10: "possible to run this on a
server ... I have a website" - InterServer, unlimited storage).

    python web_export.py [--studio main] [--out FOLDER] [--title "Robo Chess"]

Takes the scene from that studio's RUNNING server (view_step.py --serve; it has already meshed every STEP file) and
writes FOLDER (default tools/web_export/<studio>) with:
    index.html   the whole studio in one page: the robot, board, pieces, homing, Park, Demo, Play game, Draw.
                 The arm runs in the visitor's browser; the website only hands out the file (three.js comes from
                 cdn.jsdelivr.net). Server-only things are off in it: Open STEP, the load test, studio switching,
                 Claude's views, pictures sent back.
    .htaccess    asks an Apache / LiteSpeed host to send it compressed (61 MB -> ~18 MB) and not to cache a stale copy.
Upload the folder's contents (FTP or cPanel File Manager) to e.g. public_html/robochess/ -> yoursite/robochess/.

Your PC's folder names are taken out of the page (each model keeps only its file name). A snapshot: after a design
change, run this again and upload the new index.html.

    python web_export.py --play        -> tools/web_export/play: the PLAY version (Mike 2026-11-10: "a version that will
                                          play its self or against an opponent ... everything hidden not related to the
                                          bot and the chess set"): Robo Chess - play the robot (Stockfish from the CDN,
                                          chess.js for the rules), watch it play itself, or Draw. The gearbox models and
                                          test rigs are left out; the panel shows only the game, the robot and the view.
"""
import argparse
import json
import os
import re
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import studio as ST  # noqa: E402

HTACCESS = """# Robot Studio (web_export.py): send the page compressed, and always check for a newer copy
<IfModule mod_deflate.c>
  AddOutputFilterByType DEFLATE text/html application/json
</IfModule>
<IfModule mod_headers.c>
  <FilesMatch "\\.html$">
    Header set Cache-Control "no-cache"
  </FilesMatch>
</IfModule>
"""


def scrub(o, roots):
    """every string: a known folder prefix taken off; any other absolute path cut to its file name"""
    if isinstance(o, dict):
        return {k: scrub(v, roots) for k, v in o.items()}
    if isinstance(o, list):
        return [scrub(v, roots) for v in o]
    if isinstance(o, str) and ("\\" in o or "/" in o) and re.match(r"^([A-Za-z]:[\\/]|/|\\\\)", o):
        for r in roots:
            if o.lower().startswith(r.lower()):
                return o[len(r):].replace("\\", "/")
        return re.split(r"[\\/]", o)[-1]
    return o


def leftovers(o, out):
    if isinstance(o, dict):
        for v in o.values():
            leftovers(v, out)
    elif isinstance(o, list):
        for v in o:
            leftovers(v, out)
    elif isinstance(o, str) and re.search(r"[A-Za-z]:[\\/]|\\Users\\|/Users/|Mikes_creations", o):
        out.append(o[:80])


NOT_PLAY = ("gearbox", "dock")              # model names starting with these stay out of the play version


def play_data(sc, github=None):
    """the play version: the gearbox models, the dock (Mike 2026-11-10: "remove the homing stand for the player version.
    And remove the home button and all that ... Just have the robot be home") and the test rigs out of the scene (in
    place), with the dock's park moves and the home switches; the 64 squares' centres, both trays' spots
    (board_pieces.py, design frame), the lift for the viewer's game, and the project's GitHub page if given"""
    import board_pieces as BP
    drop = {i for i, m in enumerate(sc.get("models", [])) if str(m.get("name", "")).startswith(NOT_PLAY)}
    names = {sc["models"][i]["name"] for i in drop}
    sc["parts"] = [p for p in sc.get("parts", []) if p.get("model") not in drop]
    sc["rigs"] = []
    sc["gears"] = [g for g in sc.get("gears", []) if g.get("model") not in names]
    if isinstance(sc.get("ik"), dict):
        sc["ik"].pop("park", None)
    sc.pop("homing", None)
    squares = {f + str(r): [round(v, 3) for v in BP.square_at(f + str(r))] for f in "abcdefgh" for r in range(1, 9)}
    half = abs(squares["b1"][2] - squares["a1"][2]) / 2 or abs(squares["b1"][0] - squares["a1"][0]) / 2
    print(f"play version: left out {', '.join(sorted(names))}; squares {2 * half:.1f} mm")
    out = {"squares": squares, "half": round(half, 3), "lift": 125, "lifts": lift_data(),
           "trays": {c: [[round(v, 3) for v in s] for s in BP.tray_spots(c)] for c in ("white", "black")},
           "trays_unused": {c: [[round(v, 3) for v in s] for s in BP.tray_unused(c)] for c in ("white", "black")}}
    if github:
        out["github"] = github
    return out


def lift_data():
    """what the viewer needs to plan each lift as selfplay.py does (Mike 2026-11-10: "It's lifting the parts really high
    again"): every piece type's height profile (widest radius per 1 mm, from its mesh) and its padded copy, the claw's
    finger bars and the clearances"""
    import selfplay as SP
    kinds = ("pawn", "rook", "knight", "bishop", "queen", "king")
    r3 = lambda a: [round(float(v), 3) for v in a]  # noqa: E731
    return {"prof": {k: r3(SP.profile(k)) for k in kinds}, "pad": {k: r3(SP.padded(k)) for k in kinds},
            "LIFT": SP.LIFT, "LOW_LIFT": SP.LOW_LIFT, "CLEAR": SP.CLEAR, "BARS": SP.BARS, "F_LO": SP.F_LO,
            "F_HI": SP.F_HI, "HUB_LO": SP.HUB_LO, "HUB_R": SP.HUB_R, "NEAR": SP.NEAR, "ZTOP": SP.ZTOP,
            "open": {"LO": SP.OPEN_LO, "HI": SP.OPEN_HI, "SLOPE": SP.OPEN_SLOPE, "DEEP": SP.OPEN_DEEP,
                     "HALF": SP.OPEN_HALF}}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--studio", default=None, help="which studio (default ROBOT_STUDIO, else main)")
    ap.add_argument("--out", help="folder to write (default tools/web_export/<studio>)")
    ap.add_argument("--title", default=None, help="the browser tab's title")
    ap.add_argument("--play", action="store_true", help="the play version (tools/web_export/play)")
    ap.add_argument("--github", default=None, help="the play version: the project's GitHub page, linked at the panel's foot")
    a = ap.parse_args()
    a.title = a.title or ("Robo Chess" if a.play else "Robo Chess - Robot Studio")
    name = ST.name(a.studio)
    port = ST.port(name)
    if not ST.alive(port):
        sys.exit(f"studio '{name}' is not running (port {port}): start it with  python view_step.py --serve --studio {name}")
    print(f"studio '{name}' on port {port}: fetching its scene ...", flush=True)
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/scene", timeout=600) as r:
        sc = json.loads(r.read().decode("utf-8"))
    project = os.path.dirname(HERE)                                  # ...\robo_chess
    roots = [project + os.sep, project.replace("\\", "/") + "/"]
    sc = scrub(sc, roots)
    left = []
    leftovers(sc, left)
    if left:
        sys.exit("folder names still in the scene, not written:\n  " + "\n  ".join(left[:10]))
    import woods as WD                                      # his wood pictures, inside the page (the studio serves them)
    sc["woods"] = WD.wood_data(embed=True)
    static = {"scene": sc, "init": {"view": "iso"}}
    if a.play:
        static["play"] = play_data(sc, a.github)
    import view_step as V                                            # its page(): viewer.html with the scene inside
    html = V.page(static)
    html = re.sub(r"<title>.*?</title>", f"<title>{a.title}</title>", html, count=1, flags=re.S)
    out = a.out or os.path.join(HERE, "web_export", "play" if a.play else name)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "index.html"), "w", encoding="utf-8") as f:
        f.write(html)
    with open(os.path.join(out, ".htaccess"), "w", encoding="utf-8", newline="\n") as f:
        f.write(HTACCESS)
    n = sum(1 for _ in sc.get("parts", []))
    print(f"wrote {out}\\index.html ({os.path.getsize(os.path.join(out, 'index.html')) / 1e6:.1f} MB, {n} parts) "
          f"and .htaccess\nupload both to a folder in public_html (e.g. public_html/robochess/)")


if __name__ == "__main__":
    main()

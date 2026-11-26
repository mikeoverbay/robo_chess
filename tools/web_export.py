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

DATA = "scene_data.json"                # the scene, beside index.html (2026-11-24: the page loads it with a bar)
# the link preview (Mike 2026-11-24: "How do websites have a preview pop up on when you share a link?" / the arm's
# mechanical drawing as its picture): Open Graph tags in the play page's head, the picture beside it as preview.jpg
SITE = "https://tnmshouse.com/claude_robo_chess/"
PREVIEW_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chain_drive",
                           "robo_chess_arm_preview.jpg")          # arm_drawing.py
PREVIEW = "preview.jpg"
OG_TEXT = ("A 3D-printed robot arm that plays chess on a real board. Play it in your browser: Stockfish picks the "
           "robot's moves and the arm makes them.")
HTACCESS = """# Robot Studio (web_export.py): send the page and its scene data compressed, and always check for a newer page
AddType application/json .json
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
    if isinstance(o, str) and re.fullmatch(r"[A-Za-z0-9+/=]+", o):
        return o                    # a mesh's base64 (it can start with "/": 2026-11-24 four swivel chain links were cut
                                    # as if they were paths and the page died building them); a path has a '.' or a space
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


NOT_PLAY = ("gearbox", "dock", "torque")    # model names starting with these stay out of the play version
# the play version shows the chain drives inside the links: the +y walls and cheeks and their screws hidden, as the
# chaindrive studio does (Mike 2026-11-24: "I want all the new chain stuff being lit up on display"; the gearbox
# session's demo, 2026-11-20: "Leave the one side off, just so people can see how the inside of it's working")
PLAY_HIDE = ["wall +y", "cheek +y", "black zinc (link a +y", "black zinc (link b +y", "black zinc (link c +y",
             "swivel cover", "(stand-in for the base cheek", "sprocket screws m3 +y (link a's joint)",
             "sprocket screws m3 -y (link a's joint)"]
WEB_OMNI_SHADOW = 1024                      # the coloured lights' cube shadow maps on the website: 2048 is ~270 MB of
                                            # video memory each (three of them) - too much for a visitor's laptop


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
    ap.add_argument("--url", default=SITE, help="the play version: where it is on the web (for the link preview's tags)")
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
    for L in (sc.get("lighting") or {}).get("lights", []):     # the studio's lighting (3 coloured lights + the lamp)
        if L.get("shadow") is not False:
            L["shadow_size"] = min(L.get("shadow_size", 2048), WEB_OMNI_SHADOW)
    if a.play:
        static["play"] = play_data(sc, a.github)
        static["init"]["hide"] = PLAY_HIDE
    import view_step as V                                            # its page(): viewer.html with the stub inside
    # Mike 2026-11-24: "Add a loading indicator that the thing is loading data." The scene goes in a file of its own
    # (DATA) beside the page, which fetches it with a loading bar (viewer.html staticLoad); the page itself carries only
    # where it is (with a version, so no browser uses a stale copy), its size for the bar, and what kind of page it is
    import hashlib
    text = json.dumps(static, separators=(",", ":"))                 # ASCII (ensure_ascii): its length is its bytes
    stub = {"data": f"{DATA}?v={hashlib.md5(text.encode('ascii')).hexdigest()[:10]}", "bytes": len(text),
            "title": a.title, "ui": "play" if a.play else "studio"}
    html = V.page(stub)
    html = re.sub(r"<title>.*?</title>", f"<title>{a.title}</title>", html, count=1, flags=re.S)
    out = a.out or os.path.join(HERE, "web_export", "play" if a.play else name)
    os.makedirs(out, exist_ok=True)
    if a.play and os.path.isfile(PREVIEW_SRC):                       # the link preview: picture + tags
        import shutil
        shutil.copyfile(PREVIEW_SRC, os.path.join(out, PREVIEW))
        v = hashlib.md5(open(PREVIEW_SRC, "rb").read()).hexdigest()[:10]
        url = a.url if a.url.endswith("/") else a.url + "/"
        tags = "\n".join([
            f'<meta name="description" content="{OG_TEXT}">',
            '<meta property="og:type" content="website">',
            f'<meta property="og:title" content="{a.title}">',
            f'<meta property="og:description" content="{OG_TEXT}">',
            f'<meta property="og:url" content="{url}">',
            f'<meta property="og:image" content="{url}{PREVIEW}?v={v}">',
            '<meta property="og:image:width" content="1200">',
            '<meta property="og:image:height" content="630">',
            '<meta property="og:image:alt" content="Mechanical drawing of the Robo Chess robot arm reaching over the '
            'chessboard">',
            '<meta name="twitter:card" content="summary_large_image">'])
        html = html.replace("</title>", "</title>\n" + tags, 1)
        print(f"link preview: {PREVIEW} and its tags ({url})")
    elif a.play:
        print(f"no link preview picture yet ({PREVIEW_SRC}: python arm_drawing.py)")
    with open(os.path.join(out, DATA), "w", encoding="ascii", newline="\n") as f:
        f.write(text)
    with open(os.path.join(out, "index.html"), "w", encoding="utf-8") as f:
        f.write(html)
    with open(os.path.join(out, ".htaccess"), "w", encoding="utf-8", newline="\n") as f:
        f.write(HTACCESS)
    n = sum(1 for _ in sc.get("parts", []))
    print(f"wrote {out}: {DATA} ({len(text) / 1e6:.1f} MB, {n} parts), index.html "
          f"({os.path.getsize(os.path.join(out, 'index.html')) / 1e3:.0f} kB) and .htaccess\n"
          f"upload all three to the site's folder (the data file first)")


if __name__ == "__main__":
    main()

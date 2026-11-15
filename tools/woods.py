#!/usr/bin/env python3
"""
woods.py - Mike's wood pictures for the board (2026-11-10), shared by the Robot Studio server (view_step.py: served
from /textures/) and the web copies (web_export.py: put inside the page).

His words: "two woods" - Acacia on the frame and the trays (and the light squares until he finds a light wood), Ackland
Hevea (dark) on the dark squares; "the grain should run towards the back" (the viewer lays it along the robot - player
line). Each picture is a butcher-block panel on a white margin: the panel is found here (PIL) and only it is used.
WOOD_MM is the panel's height in mm (from its strips: ~25 mm acacia, ~42 mm hevea), which sets the grain's size.
"""
import base64
import os
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
FOLDER = os.path.join(os.path.dirname(HERE), "textures")
WOODS = {"frame": "Acacia.webp", "light": "Acacia.webp", "dark": "Ackland+Hevea+Prestained.webp"}
WOOD_MM = {"Acacia.webp": 600, "Ackland+Hevea+Prestained.webp": 550}
MIME = {"webp": "image/webp", "jpg": "image/jpeg", "jpeg": "image/jpeg", "png": "image/png"}


def mime(f):
    return MIME.get(f.rsplit(".", 1)[-1].lower(), "application/octet-stream")


def wood_data(embed=True, quiet=False):
    """{"srcs": {file: data URL (embed) or /textures/<file>}, "uses": {use: {src, crop [u0 v0 u1 v1] (v up), mm [w h]}}};
    None if a picture is missing"""
    from PIL import Image, ImageChops
    srcs, uses = {}, {}
    for use, f in WOODS.items():
        p = os.path.join(FOLDER, f)
        if not os.path.isfile(p):
            if not quiet:
                print(f"  wood: {f} not found - the board keeps its colours")
            return None
        im = Image.open(p).convert("RGB")
        w, h = im.size
        ink = ImageChops.difference(im, Image.new("RGB", im.size, (255, 255, 255))).convert("L").point(lambda v: 255 if v > 24 else 0)
        l, t, r, b = ink.getbbox()
        l, t, r, b = l + 4, t + 4, r - 4, b - 4                      # off the panel's anti-aliased edge
        if f not in srcs:
            srcs[f] = (f"data:{mime(f)};base64," + base64.b64encode(open(p, "rb").read()).decode("ascii")) if embed \
                else "/textures/" + urllib.parse.quote(f)
        hm = WOOD_MM.get(f, 600)
        uses[use] = {"src": f, "crop": [round(l / w, 5), round(1 - b / h, 5), round(r / w, 5), round(1 - t / h, 5)],
                     "mm": [round(hm * (r - l) / (b - t), 1), hm]}
        if not quiet:
            print(f"  wood {use}: {f}, panel {r - l} x {b - t} px = {uses[use]['mm'][0]:.0f} x {hm} mm")
    return {"srcs": srcs, "uses": uses}


def texture_file(name):
    """the path of a picture in the textures folder for /textures/<name> (its own name only), or None"""
    n = os.path.basename(urllib.parse.unquote(name))
    p = os.path.join(FOLDER, n)
    return p if n and os.path.isfile(p) else None

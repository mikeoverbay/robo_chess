#!/usr/bin/env python3
"""
studio.py - several Robot Studios at once, one per job, each with its own models, views, notes and settings.

Mike 2026-11-06: "Is there any way we can run two of these at the same time? So I can switch between them." / "You run
your version, it runs its version. Different stuff loaded, different things being used. Different settings being
saved. Think of it as multi-user interface."

A studio is a port and a folder. The folder holds that studio's scene.json (the models shown, joints, rigs ...),
show.json, notes.md, selection.json, captures/, uploads/ and fea/. Model files (STEP / STL) are shared: scene.json
names them by full path, so every studio can show the same file. The browser keeps each studio's settings apart by
itself (they are different addresses: 127.0.0.1:8765, :8766 ...). The list lives in tools/studios.json.

    python studio.py                         list the studios and which are running
    python studio.py add NAME [--who TITLE]  a new studio: next free port, folder viewer_sessions/NAME, its scene
                                             a copy of main's

A script that edits a scene picks the studio from the ROBOT_STUDIO environment variable (default "main", the original
tools/viewer_session on port 8765):  scene_path(), session_dir().  view_step.py --serve --studio NAME runs one;
the studio menu at the top of the page switches the window between them and starts one that isn't running.
"""
import json
import os
import shutil
import sys
import urllib.error
import urllib.request

TOOLS = os.path.dirname(os.path.abspath(__file__))
REGISTRY = os.path.join(TOOLS, "studios.json")
MAIN = "main"
DEFAULT = {MAIN: {"port": 8765, "dir": "viewer_session", "who": "Robot arm control and kinematics",
                  "what": "the arm: games, demos, drawing, homing, the gearbox rig"}}


def studios():
    """{name: {port, dir, who, what}} - main first."""
    try:
        reg = json.load(open(REGISTRY, encoding="utf-8"))
    except (OSError, ValueError):
        reg = {}
    out = dict(DEFAULT)
    out.update({k: v for k, v in reg.items() if isinstance(v, dict) and "port" in v and "dir" in v})
    return out


def name(n=None):
    """The studio asked for: the argument, else ROBOT_STUDIO, else main."""
    n = (n or os.environ.get("ROBOT_STUDIO") or MAIN).strip()
    if n not in studios():
        raise SystemExit(f"no Robot Studio called '{n}' (tools/studios.json has: {', '.join(studios())}); "
                         f"python studio.py add {n}")
    return n


def info(n=None):
    return studios()[name(n)]


def session_dir(n=None):
    d = os.path.join(TOOLS, info(n)["dir"])
    os.makedirs(d, exist_ok=True)
    return d


def scene_path(n=None):
    return os.path.join(session_dir(n), "scene.json")


def port(n=None):
    return int(info(n)["port"])


def alive(p, timeout=1.0):
    """Something answers on that port (a studio's /api/ping; any HTTP answer counts - an older server has no ping).
    A stopped port on Windows takes ~2 s to refuse, so the timeout decides how long a stopped one costs."""
    try:
        urllib.request.urlopen(f"http://127.0.0.1:{p}/api/ping", timeout=timeout).read()
        return True
    except urllib.error.HTTPError:
        return True
    except Exception:
        return False


def ensure(n=None):
    """A studio's folder, with a scene to start from: a copy of main's (the models, the arm, its rigs) the first time."""
    n = name(n)
    d = session_dir(n)
    sc = os.path.join(d, "scene.json")
    main_sc = scene_path(MAIN)
    if not os.path.exists(sc) and n != MAIN and os.path.exists(main_sc):
        shutil.copyfile(main_sc, sc)
    return d


def add(n, who="", what=""):
    reg = {k: v for k, v in studios().items() if k != MAIN}
    if n in studios():
        return studios()[n]
    used = {int(v["port"]) for v in studios().values()}
    p = 8766
    while p in used:
        p += 1
    reg[n] = {"port": p, "dir": f"viewer_sessions/{n}", "who": who, "what": what}
    json.dump(reg, open(REGISTRY, "w", encoding="utf-8"), indent=1)
    ensure(n)
    return reg[n]


def main():
    a = sys.argv[1:]
    if a[:1] == ["add"] and len(a) >= 2:
        who = a[a.index("--who") + 1] if "--who" in a else ""
        s = add(a[1], who)
        print(f"studio '{a[1]}': port {s['port']}, folder tools/{s['dir']}  (python view_step.py --serve --studio {a[1]})")
        return
    for k, v in studios().items():
        print(f"  {k:12s} 127.0.0.1:{v['port']}  {'running' if alive(v['port']) else 'stopped':8s}  tools/{v['dir']}  "
              f"{v.get('who', '')}")


if __name__ == "__main__":
    main()

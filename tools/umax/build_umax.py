#!/usr/bin/env python3
"""
build_umax.py - compile umax_cli.exe: micro-Max 4.8 (Robo Chess's own copy in umax/engine/) + umax_cli.cpp.

    python umax/build_umax.py

Needs Visual Studio's C++ compiler (VS 18 Community).
Robo Chess is its own project, apart from Megachess (Mike, 2026-11-06: "This robot arm project is actually called Robo
Chess, not Mega Chess ... We should move it away from Mega Chess completely"): engine/ is a copy of the engine taken
2026-11-06, the hard-stop fix included, and it is changed here only.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
UMAX = os.path.join(HERE, "engine")
VCVARS = r"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
EXE = os.path.join(HERE, "umax_cli.exe")
BUILD = os.path.join(HERE, "build")


def main():
    trace = "--trace" in sys.argv                      # micro-Max's own root trace (UMAX_TRACE), for debugging
    exe = os.path.join(HERE, "umax_trace.exe") if trace else EXE
    os.makedirs(BUILD, exist_ok=True)
    flags = ["/nologo", "/O2", "/EHsc", "/std:c++17", "/W1", "/wd4244", "/wd4267", "/wd4996",
             "/I", os.path.join(HERE, "shim"), "/I", UMAX, "/Fe" + exe]          # objects land in BUILD (the cwd)
    if trace:
        flags.append("/DUMAX_TRACE")
    sources = [os.path.join(HERE, "umax_cli.cpp"), os.path.join(UMAX, "umax.cpp")]
    cmd = 'call "%s" >nul 2>&1 && cl %s %s' % (VCVARS, " ".join('"%s"' % f if " " in f else f for f in flags),
                                                " ".join('"%s"' % s for s in sources))
    r = subprocess.run(cmd, shell=True, cwd=BUILD, capture_output=True, text=True)
    if r.returncode != 0:
        print("COMPILE FAILED")
        print((r.stdout + r.stderr)[-4000:])
        sys.exit(1)
    print("built", exe)


if __name__ == "__main__":
    main()

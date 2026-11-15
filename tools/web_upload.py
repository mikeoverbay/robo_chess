#!/usr/bin/env python3
"""
web_upload.py - put the exported Robot Studio (web_export.py -> tools/web_export/<studio>/: index.html, .htaccess) on
the website by FTP over TLS (Mike 2026-11-10: InterServer, an FTP account made for this; "there is no reason to go
anywhere else at my site").

    python web_upload.py --user NAME [--host ftp.tnmshouse.com] [--studio main]

YOU type the password when it asks; it is not shown, stored or passed anywhere else. The files go into the folder the
account logs in to, and nowhere else: no other folder on the site is listed, changed or deleted.
"""
import argparse
import ftplib
import getpass
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FILES = ["index.html", ".htaccess"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--user", required=True, help="the FTP account")
    ap.add_argument("--host", default="ftp.tnmshouse.com")
    ap.add_argument("--studio", default="main", help="which export (tools/web_export/<studio>)")
    a = ap.parse_args()
    src = os.path.join(HERE, "web_export", a.studio)
    missing = [f for f in FILES if not os.path.isfile(os.path.join(src, f))]
    if missing:
        sys.exit(f"not exported yet ({', '.join(missing)} missing in {src}): run  python web_export.py  first")
    pw = getpass.getpass(f"password for {a.user} on {a.host} (not shown): ")
    ftp = ftplib.FTP_TLS(a.host, timeout=120)
    ftp.login(a.user, pw)
    del pw
    ftp.prot_p()                                         # the files travel encrypted too
    home = ftp.pwd()
    print(f"logged in; uploading to {home}")
    for f in FILES:
        p = os.path.join(src, f)
        size, sent = os.path.getsize(p), [0, -1]
        def tick(block, size=size, sent=sent, f=f):
            sent[0] += len(block)
            pct = int(sent[0] * 100 / size)
            if pct // 10 != sent[1]:
                sent[1] = pct // 10
                print(f"  {f}: {pct}%", flush=True)
        with open(p, "rb") as fh:
            ftp.storbinary(f"STOR {f}", fh, blocksize=1 << 16, callback=tick)
        try:
            remote = ftp.size(f)
        except ftplib.error_perm:
            remote = None
        ok = remote == size
        print(f"  {f}: {size / 1e6:.2f} MB sent, {'size on the site matches' if ok else f'site says {remote} bytes - CHECK'}")
    ftp.quit()
    print("done")


if __name__ == "__main__":
    main()

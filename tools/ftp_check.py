#!/usr/bin/env python3
"""
ftp_check.py - why a directory listing fails on the website's FTP (Mike 2026-11-24: "I can't get a directory listing
back from it"). Login runs on the control connection (port 21); a listing or an upload needs a second, data
connection, so each way of opening one is tried in turn and what happens is printed:

  1. PASV: the address the server hands out (a private one can't be reached), and a bare TCP connect to that port
  2. EPSV: the same, the newer way (port only)
  3. a listing over passive + TLS data, as web_upload.py does
  4. the same with the TLS session reused on the data channel (ProFTPD can require it; plain Python can't do it,
     FileZilla can)
  5. a listing over passive with the data channel in clear (some servers refuse: that refusal is an answer too)

    python ftp_check.py --user NAME [--host ftp.tnmshouse.com]

The password is asked for (not shown), used for the login only and never printed or stored. Nothing is uploaded,
changed or deleted: only PWD, PASV/EPSV and listings of the folder the account opens in.
"""
import argparse
import ftplib
import getpass
import ipaddress
import re
import socket
import ssl
import time

T = 15.0          # seconds to wait for any one step


class ReuseTLS(ftplib.FTP_TLS):
    """the data channel's TLS resumes the control channel's session (ProFTPD's TLSOptions without
    NoSessionReuseRequired insists on it)"""
    def ntransfercmd(self, cmd, rest=None):
        conn, size = ftplib.FTP.ntransfercmd(self, cmd, rest)
        if self._prot_p:
            conn = self.context.wrap_socket(conn, server_hostname=self.host, session=self.sock.session)
        return conn, size


def tcp(ip, port):
    t = time.time()
    try:
        socket.create_connection((ip, port), timeout=T).close()
        return f"connects ({time.time() - t:.2f} s)"
    except socket.timeout:
        return f"NO ANSWER in {T:g} s (blocked by a firewall somewhere between)"
    except OSError as e:
        return f"FAILS: {e}"


def listing(ftp, label):
    t = time.time()
    try:
        names = ftp.nlst()
        print(f"  {label}: WORKS - {len(names)} entries in {time.time() - t:.1f} s: {', '.join(names[:8])}")
        return True
    except Exception as e:                                       # noqa: BLE001 - every failure is the answer here
        print(f"  {label}: FAILS after {time.time() - t:.1f} s - {type(e).__name__}: {e}")
        return False


def session(cls, host, user, pw, tls12=False):
    ftp = cls(timeout=T)
    if tls12:                                                    # session reuse is dependable on TLS 1.2
        ctx = ssl.create_default_context()
        ctx.maximum_version = ssl.TLSVersion.TLSv1_2
        ftp.context = ctx
    ftp.connect(host, 21)
    ftp.login(user, pw)
    ftp.prot_p()
    return ftp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--user", required=True)
    ap.add_argument("--host", default="ftp.tnmshouse.com")
    a = ap.parse_args()
    host_ip = socket.gethostbyname(a.host)
    print(f"{a.host} = {host_ip}")
    ftp = ftplib.FTP_TLS(timeout=T)
    print("server says:", ftp.connect(a.host, 21))
    try:
        print("features:", " ".join(ftp.sendcmd("FEAT").split()[1:-1]))
    except ftplib.all_errors as e:
        print("features: (refused)", e)
    pw = getpass.getpass(f"password for {a.user} on {a.host} (not shown): ")
    try:
        print("login:", ftp.login(a.user, pw))
    except ftplib.error_perm as e:
        print(f"login REFUSED: {e}  - the server is up and answering; it does not take this account + password "
              f"(a typo, a changed password, or the account changed or locked in InterServer's control panel)")
        return
    ftp.prot_p()
    print("folder:", ftp.pwd())

    print("\n1. PASV")
    try:
        r = ftp.sendcmd("PASV")
        print("  reply:", r)
        n = [int(x) for x in re.search(r"(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)", r).groups()]
        ip, port = ".".join(map(str, n[:4])), n[4] * 256 + n[5]
        private = ipaddress.ip_address(ip).is_private
        print(f"  hands out {ip}:{port}" + ("  <- a PRIVATE address: a client that trusts it can't reach it" if private
                                           else ""))
        print(f"  TCP to {ip}:{port}: {tcp(ip, port)}")
        if ip != host_ip:
            print(f"  TCP to {host_ip}:{port} (the server's own address): {tcp(host_ip, port)}")
    except Exception as e:                                       # noqa: BLE001
        print("  FAILS:", e)

    print("\n2. EPSV")
    try:
        r = ftp.sendcmd("EPSV")
        print("  reply:", r)
        port = int(re.search(r"\|\|\|(\d+)\|", r).group(1))
        print(f"  TCP to {host_ip}:{port}: {tcp(host_ip, port)}")
    except Exception as e:                                       # noqa: BLE001
        print("  FAILS:", e)
    try:
        ftp.quit()
    except Exception:                                            # noqa: BLE001
        ftp.close()

    print("\n3-5. listings")
    for label, cls, tls12, clear in (("3. passive, TLS data (as web_upload.py)", ftplib.FTP_TLS, False, False),
                                     ("4. passive, TLS data, session reused (TLS 1.2)", ReuseTLS, True, False),
                                     ("5. passive, data in clear", ftplib.FTP_TLS, False, True)):
        try:
            f = session(cls, a.host, a.user, pw, tls12)
            if clear:
                f.prot_c()
            listing(f, label)
            try:
                f.quit()
            except Exception:                                    # noqa: BLE001
                f.close()
        except Exception as e:                                   # noqa: BLE001
            print(f"  {label}: could not even log in again - {type(e).__name__}: {e}")
    del pw


if __name__ == "__main__":
    main()

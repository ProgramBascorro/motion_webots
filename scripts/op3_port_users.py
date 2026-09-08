#!/usr/bin/env python3
"""Siapa yang sedang memegang port serial robot ini.

Kenapa alat ini ada: `port_handler_linux.cpp:184` membuka port TANPA `TIOCEXCL`,
jadi proses kedua tetap boleh `open()` port yang sama dan Linux tidak protes sama
sekali. Yang terjadi kemudian bukan "port sedang dipakai", melainkan dua pembaca
berebut satu aliran byte: paket balasan servo/OpenCR diambil proses yang lain,
yang minta jadi timeout, lalu muncul pesan yang menuduh perangkat -- padahal
perangkatnya sehat. Dua gejala nyata di robot ini:

    [ERROR] Failed to turn on the Power of DXLs!     <- OpenCR sebenarnya sehat
    [ERROR] first bulk read fail!!                   <- servo sebenarnya sehat

Ditambah satu jebakan lagi: proses yang portnya sempat re-enumerate akan
memegang fd yang menunjuk node terhapus (`/dev/ttyUSB0 (deleted)`) SELAMANYA --
robotis_controller resolve by-id sekali saat start dan tidak pernah buka ulang.
Proses seperti itu hidup, 0% CPU, tidak menerbitkan apa pun, dan tidak memberi
satu pun pesan error. Alat ini menandainya sebagai "fd basi".

Nama prosesnya juga menipu: `comm` dipotong 15 karakter jadi `op3_action_edit`,
sehingga `pgrep -x op3_action_editor` SELALU kosong walau prosesnya jelas ada.

Pemakaian:
    scripts/op3_port_users.py            # lapor, keluar 1 kalau ada yang pegang
    scripts/op3_port_users.py --quiet    # diam kalau bebas (dipakai pre-flight)
    scripts/op3_port_users.py --kill     # matikan pemegangnya (sengaja manual)
"""

import argparse
import os
import re
import signal
import sys
import time

WS = os.environ.get("OP3_WS") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_FILE = os.path.join(WS, "src/ROBOTIS-OP3/op3_manager/config/OP3.robot")

BOLD, RED, GREEN, YELLOW, DIM, RST = (
    "\033[1m", "\033[31m", "\033[32m", "\033[33m", "\033[2m", "\033[0m",
)


def robot_file_ports(path=ROBOT_FILE):
    """Alamat port yang diminta OP3.robot, apa adanya (belum di-resolve)."""
    ports = []
    if not os.path.isfile(path):
        return ports
    for line in open(path, "r", encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if not line or line.startswith("["):
            continue
        for m in re.findall(r"/dev/\S+", line.split("|")[0]):
            if m not in ports:
                ports.append(m)
    return ports


def wanted_ports():
    """Port OP3.robot plus override env yang dipakai launcher."""
    ports = robot_file_ports()
    for var in ("OP3_SERIAL_DEVICE", "OP3_SUB_CONTROLLER_DEVICE"):
        dev = os.environ.get(var, "").strip()
        if dev and dev not in ports:
            ports.append(dev)
    return ports


# Node tty yang sudah DIHAPUS tidak mungkin milik perangkat yang masih hidup:
# itu pasti fd basi milik adapter yang sempat re-enumerate (ttyUSB0 -> ttyUSB2).
# Justru pemegang seperti itu yang paling perlu disapu -- dan dia TIDAK akan
# pernah cocok lewat realpath, karena nama lamanya sudah tidak ada lagi di /dev.
STALE_NODE_RE = re.compile(r"^/dev/tty(USB\d+|ACM\d+|OP3)$")


def holders(ports):
    """Proses yang memegang salah satu port. [(pid, comm, cmdline, target, basi)]"""
    # Node bisa berganti minor (ttyUSB0 -> ttyUSB1 -> ttyUSB2) sepanjang malam,
    # jadi cocokkan lewat realpath, bukan lewat nama by-id.
    targets = set()
    for port in ports:
        targets.add(port)
        if os.path.exists(port):
            targets.add(os.path.realpath(port))
    basenames = {os.path.basename(t) for t in targets}

    found = []
    me = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        fd_dir = "/proc/%d/fd" % pid
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue  # proses milik user lain atau sudah mati -- lewati diam-diam
        for fd in fds:
            try:
                link = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            stale = link.endswith(" (deleted)")
            real = link[: -len(" (deleted)")] if stale else link
            if real in targets or os.path.basename(real) in basenames:
                pass
            elif stale and STALE_NODE_RE.match(real):
                pass  # fd basi -- lihat STALE_NODE_RE
            else:
                continue
            try:
                comm = open("/proc/%d/comm" % pid).read().strip()
                cmdline = open("/proc/%d/cmdline" % pid).read().replace("\0", " ").strip()
            except OSError:
                continue
            found.append((pid, comm, cmdline, real, stale))
            break  # satu baris per proses sudah cukup
    return found


def describe(found):
    lines = []
    for pid, comm, cmdline, target, stale in found:
        tag = "%sfd basi%s" % (RED, RST) if stale else "aktif"
        lines.append("  pid %-6d %-16s %s  (%s)" % (pid, comm, target, tag))
        lines.append("    %s%s%s" % (DIM, cmdline[:150], RST))
    return "\n".join(lines)


def kill_holders(found):
    pids = sorted({pid for pid, _, _, _, _ in found})
    if not pids:
        return 0
    print("%smematikan:%s %s" % (BOLD, RST, " ".join(str(p) for p in pids)))
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    # Proses yang nyangkut di read() pada fd basi tidak menanggapi SIGTERM --
    # itu bukan teori, itu yang terjadi pada op3_manager 2026-08-18.
    time.sleep(3)
    left = []
    for pid in pids:
        try:
            os.kill(pid, 0)
            left.append(pid)
        except OSError:
            continue
    for pid in left:
        print("  %s tidak menanggapi SIGTERM -> SIGKILL" % pid)
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(1)
    still = [p for p in pids if os.path.exists("/proc/%d" % p)]
    if still:
        print("%sgagal%s masih hidup: %s" % (RED, RST, still))
        return 1
    print("%sok%s port bebas. Catatan: torsi servo TETAP menyala." % (GREEN, RST))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--quiet", action="store_true", help="diam kalau port bebas")
    ap.add_argument("--brief", action="store_true",
                    help="hanya daftar pemegangnya, tanpa penjelasan (untuk doctor)")
    ap.add_argument("--kill", action="store_true", help="matikan pemegang port")
    args = ap.parse_args()

    ports = wanted_ports()
    if not ports:
        print("%swarn%s tidak ada path port di %s" % (YELLOW, RST, ROBOT_FILE))
        return 0

    found = holders(ports)
    if not found:
        if not args.quiet and not args.brief:
            print("%sok%s tidak ada proses yang memegang port robot:" % (GREEN, RST))
            for port in ports:
                where = os.path.realpath(port) if os.path.exists(port) else "TIDAK ADA"
                print("    %s -> %s" % (os.path.basename(port), where))
        return 0

    if args.brief:
        # Pemanggilnya (doctor) sudah punya bingkai sendiri -- jangan diulang.
        print(describe(found))
        return 1

    print("%sPort robot sedang dipegang proses lain:%s" % (RED, RST))
    print(describe(found))
    if args.kill:
        return kill_holders(found)
    print()
    print("Selama proses itu hidup, start berikutnya TIDAK akan bekerja: portnya")
    print("tetap bisa dibuka, tapi balasan servo/OpenCR diambil proses tadi, dan")
    print("errornya menuduh perangkat ('Failed to turn on the Power of DXLs!',")
    print("'first bulk read fail!!') padahal perangkatnya sehat.")
    print()
    print("  Matikan dengan: %sscripts/op3_port_users.py --kill%s" % (BOLD, RST))
    return 1


if __name__ == "__main__":
    sys.exit(main())

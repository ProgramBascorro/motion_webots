#!/usr/bin/env python3
"""Sesuaikan repo ini dengan perangkat yang benar-benar tercolok di robot ini.

Kenapa alat ini ada: alamat serial robot ditulis TANGAN di lima berkas, dan satu
saja tertinggal, gejalanya menyesatkan -- "Error opening serial port", atau lebih
buruk, tanpa error sama sekali (manager jalan, gait berputar, tapi robot diam).
Memindahkan repo ini ke robot lain berarti mengganti semuanya secara konsisten.

Pemakaian:
    scripts/op3_serial_setup.py                  # lihat saja, tidak mengubah apa pun
    scripts/op3_serial_setup.py --apply          # terapkan alamat perangkat tercolok
    scripts/op3_serial_setup.py --apply --opencr=bus   # OpenCR lewat bus servo (board sehat)
    scripts/op3_serial_setup.py --apply --opencr=usb   # OpenCR lewat USB CDC sendiri

Tentang --opencr: OP3 normal menaruh OpenCR (ID 200) di bus TTL yang sama dengan
servo -- itu `bus`, dan itu yang benar untuk board sehat. Repo ini default `usb`
karena transceiver bus di board ALPHONSE MATI (ID 200 diam di semua baud
9600-4M sementara 20 servo menjawab bersih, tapi lewat micro-USB dia sehat:
err=0, model 0x7400, IMU 1g, 12.5V). Jangan bawa workaround itu ke robot lain
tanpa membuktikan board-nya memang rusak dengan cara yang sama.
"""

import argparse
import glob
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ROBOT_FILE = os.path.join(WS, "src/ROBOTIS-OP3/op3_manager/config/OP3.robot")

# Setiap berkas yang menyebut alamat serial secara harfiah. Kalau menambah
# pemakai baru, daftarkan di sini juga -- itulah gunanya satu daftar.
FILES = [
    ROBOT_FILE,
    os.path.join(WS, "src/ROBOTIS-OP3/op3_manager/launch/op3_manager.launch.py"),
    os.path.join(WS, "scripts/op3_tmux/lib/profiles.sh"),
    os.path.join(WS, "scripts/wait_u2d2_then_manager.sh"),
    os.path.join(WS, "src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/executor.py"),
]

BY_ID = "/dev/serial/by-id"
U2D2_GLOB = os.path.join(BY_ID, "usb-FTDI_*-if00-port0")
OPENCR_GLOB = os.path.join(BY_ID, "usb-ROBOTIS_OpenCR_*-if00")

BOLD, RED, GREEN, YELLOW, RST = "\033[1m", "\033[31m", "\033[32m", "\033[33m", "\033[0m"


def detect(pattern):
    """Perangkat yang benar-benar tercolok sekarang."""
    return sorted(glob.glob(pattern))


def configured(kind):
    """Alamat yang saat ini tertulis di OP3.robot."""
    if not os.path.isfile(ROBOT_FILE):
        return None
    needle = "usb-FTDI_" if kind == "u2d2" else "usb-ROBOTIS_OpenCR_"
    for line in open(ROBOT_FILE):
        if line.lstrip().startswith("#"):
            continue
        m = re.search(r"(/dev/serial/by-id/\S*%s\S*)" % re.escape(needle), line)
        if m:
            return m.group(1)
    return None


def replace_everywhere(old, new, apply_changes):
    """Ganti satu alamat di semua berkas. Mengembalikan jumlah kena per berkas."""
    hits = []
    for path in FILES:
        if not os.path.isfile(path):
            hits.append((path, None))  # berkas hilang -- laporkan, jangan diam
            continue
        text = open(path).read()
        n = text.count(old)
        if n and apply_changes:
            open(path, "w").write(text.replace(old, new))
        hits.append((path, n))
    return hits


def set_opencr_topology(mode, u2d2_path, opencr_path, apply_changes):
    """Pindahkan OpenCR antara bus servo (`bus`) dan USB CDC-nya sendiri (`usb`).

    Hanya menyentuh OP3.robot: satu baris di [ port info ] dan satu di
    [ device info ]. Baris servo tidak pernah disentuh.
    """
    lines = open(ROBOT_FILE).read().split("\n")
    out, changed = [], []
    in_ports = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("["):
            in_ports = stripped.lower().startswith("[ port info")
        # Baris port milik OpenCR: hanya ada pada mode usb.
        if in_ports and "usb-ROBOTIS_OpenCR_" in line and not stripped.startswith("#"):
            if mode == "bus":
                changed.append("hapus baris port OpenCR (ikut bus servo)")
                continue
        # Baris sensor: arahkan ke port yang sesuai.
        if stripped.startswith("sensor") and "|" in line:
            target = u2d2_path if mode == "bus" else opencr_path
            new_line = re.sub(r"/dev/serial/by-id/\S+", target, line, count=1)
            if new_line != line:
                changed.append("arahkan sensor OpenCR ke %s" % os.path.basename(target))
                line = new_line
        out.append(line)

    if mode == "usb":
        # Pastikan baris port OpenCR ada; kalau hilang, tambahkan setelah baris servo.
        if not any("usb-ROBOTIS_OpenCR_" in l and not l.strip().startswith("#") and "|" in l
                   and not l.strip().startswith("sensor") for l in out):
            for i, l in enumerate(out):
                if "usb-FTDI_" in l and "|" in l and not l.strip().startswith(("#", "dynamixel")):
                    out.insert(i + 1, "%s | 2000000   | open-cr" % opencr_path)
                    changed.append("tambahkan baris port OpenCR (USB CDC terpisah)")
                    break

    if changed and apply_changes:
        open(ROBOT_FILE, "w").write("\n".join(out))
    return changed


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--apply", action="store_true", help="tulis perubahan (default: lihat saja)")
    ap.add_argument("--opencr", choices=["bus", "usb"], default=None,
                    help="bus = OpenCR di bus servo (board sehat); usb = port USB sendiri")
    ap.add_argument("-h", "--help", action="store_true")
    args = ap.parse_args()
    if args.help:
        print(__doc__)
        return 0

    print("%sPerangkat tercolok di host%s" % (BOLD, RST))
    u2d2_found = detect(U2D2_GLOB)
    opencr_found = detect(OPENCR_GLOB)
    for label, found in (("U2D2 (bus servo)", u2d2_found), ("OpenCR", opencr_found)):
        if not found:
            print("  %sgagal%s  %s TIDAK ada. Colokkan dulu, lalu ulangi." % (RED, RST, label))
        else:
            for f in found:
                print("  %sok%s     %-18s %s -> %s"
                      % (GREEN, RST, label, os.path.basename(f), os.path.realpath(f)))
            if len(found) > 1:
                print("  %swarn%s   lebih dari satu -- dipakai yang pertama" % (YELLOW, RST))

    print()
    print("%sYang tertulis di OP3.robot sekarang%s" % (BOLD, RST))
    cur_u2d2, cur_opencr = configured("u2d2"), configured("opencr")
    print("  U2D2   : %s" % (cur_u2d2 or "(tidak ada)"))
    print("  OpenCR : %s" % (cur_opencr or "(tidak ada -- OpenCR ikut bus servo)"))

    if not u2d2_found:
        print("\n%sBerhenti: U2D2 tidak tercolok, tidak ada yang bisa disesuaikan.%s" % (RED, RST))
        return 1

    pairs = [(cur_u2d2, u2d2_found[0], "U2D2")]
    if cur_opencr and opencr_found:
        pairs.append((cur_opencr, opencr_found[0], "OpenCR"))

    print()
    print("%sPerubahan alamat%s%s" % (BOLD, RST, "" if args.apply else "  (pratinjau saja)"))
    total = 0
    for old, new, label in pairs:
        if not old or old == new:
            print("  %sok%s     %s sudah benar" % (GREEN, RST, label))
            continue
        print("  %s:\n    dari : %s\n    jadi : %s" % (label, old, new))
        for path, n in replace_everywhere(old, new, args.apply):
            rel = os.path.relpath(path, WS)
            if n is None:
                print("    %swarn%s %-58s berkas tidak ada" % (YELLOW, RST, rel))
            elif n:
                print("    %s%-58s %d baris%s" % ("      ", rel, n, ""))
                total += n
    if total == 0:
        print("  (tidak ada yang perlu diubah)")

    if args.opencr:
        print()
        print("%sTopologi OpenCR -> %s%s" % (BOLD, args.opencr, RST))
        changes = set_opencr_topology(args.opencr, u2d2_found[0],
                                      opencr_found[0] if opencr_found else "",
                                      args.apply)
        for c in changes:
            print("    %s" % c)
        if not changes:
            print("    (sudah sesuai)")

    print()
    if args.apply:
        print("%sSelesai.%s Lanjut: colcon build --symlink-install lalu scripts/op3_docker.sh doctor"
              % (GREEN, RST))
    else:
        print("Belum ada yang diubah. Jalankan lagi dengan %s--apply%s untuk menerapkan."
              % (BOLD, RST))
    return 0


if __name__ == "__main__":
    sys.exit(main())

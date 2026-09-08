#!/usr/bin/env python3
"""Salin pose WALKING_READY (action page 2) ke op3_base_module/data/ini_pose.yaml.

KENAPA ADA DUA SALINAN POSE YANG SAMA
-------------------------------------
Robot ini berdiri di pose WALKING_READY lewat DUA jalan yang berbeda:

  * action_module memainkan HALAMAN 2 dari ALPHONSE.bin  -> dipakai demo saat boot
  * base_module memutar ke ini_pose.yaml                 -> dipakai tombol INIT
    (Bascorro Studio, op3_gui_demo, joy teleop, demo_node.goInitPose)

Kalau isi keduanya beda, robot berdiri di pose yang BERBEDA tergantung tombol
mana yang ditekan. ini_pose.yaml tidak dibaca dari .bin -- ia salinan tangan --
jadi tiap kali halaman 2 di-teach ulang, berkas itu harus ikut disegarkan.
Skrip ini yang mengerjakannya, supaya tidak ada lagi angka yang disalin manual.

PAKAI
-----
    python3 scripts/sync_ini_pose_from_page2.py            # lihat dulu (dry run)
    python3 scripts/sync_ini_pose_from_page2.py --write    # tulis

ini_pose.yaml disimbolik-link dari install/, jadi TIDAK perlu colcon build.
Cukup jalankan ulang demo/manager.

CATATAN
-------
* Hanya `tar_pose` yang ditulis ulang. `mov_time`, `via_num`, `via_time`, dan
  `via_pose` dibiarkan apa adanya -- itu setelan gerak, bukan pose tujuan.
* Nilai mentah dengan bit INVALID (0x4000) atau TORQUE_OFF (0x2000) dilewati:
  sendi itu memang tidak diperintah oleh halaman tersebut, jadi nilai lamanya
  di ini_pose.yaml dipertahankan.
* Rumus derajat sama dengan yang dipakai op3_action_module:
  (raw - 2048) * 360 / 4096.
"""

import argparse
import os
import re
import struct
import sys

PAGE_SIZE = 512
STEP_OFFSET = 64
STEP_SIZE = 64
INVALID_BIT = 0x4000
TORQUE_OFF_BIT = 0x2000

JOINT_NAMES = {
    1: "r_sho_pitch", 2: "l_sho_pitch", 3: "r_sho_roll", 4: "l_sho_roll",
    5: "r_el", 6: "l_el", 7: "r_hip_yaw", 8: "l_hip_yaw",
    9: "r_hip_roll", 10: "l_hip_roll", 11: "r_hip_pitch", 12: "l_hip_pitch",
    13: "r_knee", 14: "l_knee", 15: "r_ank_pitch", 16: "l_ank_pitch",
    17: "r_ank_roll", 18: "l_ank_roll", 19: "head_pan", 20: "head_tilt",
}

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BIN = os.path.join(HERE, "src/ROBOTIS-OP3/op3_action_module/data/ALPHONSE.bin")
DEFAULT_YAML = os.path.join(HERE, "src/ROBOTIS-OP3/op3_base_module/data/ini_pose.yaml")


def raw_to_deg(raw):
    return (raw - 2048) * 360.0 / 4096.0


def read_page_step0(path, page):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < (page + 1) * PAGE_SIZE:
        raise SystemExit("berkas terlalu pendek untuk halaman %d: %s" % (page, path))

    base = page * PAGE_SIZE
    name = data[base:base + 14].split(b"\x00")[0].decode("ascii", "replace").strip()
    step_num = data[base + 20]
    checksum_ok = (sum(data[base:base + PAGE_SIZE]) & 0xFF) == 0xFF
    if step_num < 1:
        raise SystemExit("halaman %d tidak punya step (stepnum=%d)" % (page, step_num))

    positions = struct.unpack_from("<31H", data, base + STEP_OFFSET)
    return name, step_num, checksum_ok, positions


def parse_existing_tar_pose(text):
    """Ambil angka tar_pose yang sekarang, untuk ditampilkan sebagai selisih."""
    out = {}
    block = re.search(r"^tar_pose\s*:\s*$(.*)", text, re.M | re.S)
    if not block:
        return out
    for line in block.group(1).splitlines():
        m = re.match(r"\s*(\d+)\s*:\s*([-+0-9.eE]+)", line)
        if not m:
            if line.strip() and not line.strip().startswith("#") and not line.startswith(" "):
                break
            continue
        out[int(m.group(1))] = float(m.group(2))
    return out


def build_tar_pose_block(positions, previous, bin_path, page):
    lines = [
        "# target pose [deg]",
        "#",
        "# DIBUAT OTOMATIS oleh scripts/sync_ini_pose_from_page2.py.",
        "# Sumber: halaman %d (WALKING_READY) step[0] dari" % page,
        "#   %s" % os.path.relpath(bin_path, HERE),
        "#",
        "# JANGAN disunting tangan. Tiap kali halaman 2 di-teach ulang di action",
        "# editor, jalankan ulang skrip itu -- kalau tidak, tombol INIT (base_module)",
        "# dan boot demo (action_module) akan berdiri di pose yang berbeda.",
        "#",
        "# Konversi: derajat = (raw - 2048) * 360 / 4096.",
        "",
        "tar_pose :",
    ]
    changes = []
    for jid in range(1, 21):
        raw = positions[jid]
        name = JOINT_NAMES[jid]
        if raw & (INVALID_BIT | TORQUE_OFF_BIT):
            keep = previous.get(jid, 0.0)
            lines.append("  %-2d : %-14s # %-12s (halaman 2 tidak memerintah sendi ini)"
                         % (jid, "%.6f" % keep, name))
            continue
        deg = raw_to_deg(raw)
        lines.append("  %-2d : %-14s # %-12s (raw %d)" % (jid, "%.6f" % deg, name, raw))
        old = previous.get(jid)
        if old is None or abs(old - deg) > 0.01:
            changes.append((jid, name, old, deg))
    return "\n".join(lines) + "\n", changes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=DEFAULT_BIN)
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--page", type=int, default=2)
    ap.add_argument("--write", action="store_true", help="tulis; tanpa ini cuma ditampilkan")
    args = ap.parse_args()

    name, step_num, checksum_ok, positions = read_page_step0(args.bin, args.page)
    print("halaman %d : nama %r, %d step, checksum %s"
          % (args.page, name, step_num, "OK" if checksum_ok else "RUSAK"))
    if not checksum_ok:
        print("  !! checksum halaman tidak sah -- periksa berkasnya sebelum dipakai")

    with open(args.yaml, encoding="utf-8") as f:
        text = f.read()
    previous = parse_existing_tar_pose(text)

    block, changes = build_tar_pose_block(positions, previous, args.bin, args.page)

    if not changes:
        print("ini_pose.yaml SUDAH sama dengan halaman %d -- tidak ada yang diubah."
              % args.page)
        return 0

    print("\nselisih terhadap ini_pose.yaml yang sekarang:")
    print("  %-3s %-12s %10s %10s %10s"
          % ("id", "sendi", "lama", "baru", "selisih"))
    for jid, jname, old, new in changes:
        if old is None:
            print("  %-3d %-12s %10s %10.4f %10s" % (jid, jname, "-", new, "baru"))
        else:
            print("  %-3d %-12s %10.4f %10.4f %+10.4f" % (jid, jname, old, new, new - old))

    head = text.split("# target pose [deg]")[0].rstrip("\n")
    new_text = head + "\n\n\n" + block

    if not args.write:
        print("\n(dry run) tambahkan --write untuk menulis ke %s"
              % os.path.relpath(args.yaml, HERE))
        return 0

    with open(args.yaml, "w", encoding="utf-8") as f:
        f.write(new_text)
    print("\nditulis: %s" % os.path.relpath(args.yaml, HERE))
    print("ini_pose.yaml disimbolik-link dari install/, jadi tidak perlu colcon build.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Forward kinematics dari satu halaman action -> init_x/y/z_offset walking_module.

Asalnya scripts/init_baru_fk.py di CHRONUS_NEW. Dua hal diubah untuk robot ini:

  * Namanya. Di sana page 2 bernama INIT_BARU waktu script itu ditulis. Di robot
    ini page 2 sudah bernama WALKING_READY dan INIT_BARU pindah ke page 3, jadi
    nama berkas lama justru menunjuk halaman yang salah.
  * Isinya. Versi CHRONUS menyimpan nilai sendi sebagai dict tetap di dalam
    script, disalin tangan dari bin. Salinan itu basi begitu page-nya di-capture
    ulang di action editor -- dan memang sudah basi di sana (l_ank_pitch 1956,
    padahal bin robot ini 1977). Di sini page-nya dibaca langsung dari bin, jadi
    tidak ada yang perlu disalin dan tidak ada yang bisa basi.

=============================================================================
 BACA DULU SEBELUM MEMAKAI ANGKANYA
=============================================================================
Script ini menganggap raw dynamixel 2048 == sendi lurus mekanis untuk semua
servo. Itu kira-kira benar di robot sumber ("Robot 0"), tapi TIDAK benar di
robot ini: kalibrasi nol servonya beda, jadi page 2 terbaca seolah kakinya
hampir lurus dan init_z_offset keluar di luar rentang masuk akal, padahal
robotnya jelas berdiri menekuk.

Karena itu offset hasil FK di sini TIDAK dipakai. op3_walking_module memakai
mekanisme capture/bias (op3_walking_module.cpp ~504-544 dan ~1213): begitu
walking_module dinyalakan, ia menangkap pose page 2 yang sedang dipegang
controller lalu membiasi gait ke atasnya. Itu membaca pose SUNGGUHAN saat
jalan, jadi tidak butuh rantai FK yang benar maupun kalibrasi nol yang benar.
param.yaml menyimpan offset hasil tuning empiris -- dicetak di bawah supaya
bisa langsung dibandingkan.

Script ini berguna untuk: melihat isi sebuah page tanpa membuka action editor
(editor memegang port DXL, jadi tidak bisa dibuka bersamaan dengan manager),
dan memeriksa apakah dua page berbeda jauh setelah di-capture ulang.
=============================================================================

Pemakaian:
    python3 scripts/action_page_fk.py                 # page 2 (WALKING_READY)
    python3 scripts/action_page_fk.py --page 3        # INIT_BARU
    python3 scripts/action_page_fk.py --bin lain.bin --page 2
    python3 scripts/action_page_fk.py --page 2 --raw  # cuma daftar nilai sendi
"""
import argparse
import math
import os
import re
import struct
import sys

# ===== Format berkas action (op3_action_module/include/.../action_file_define.h) =====
PAGE_SIZE = 512          # header 64 + 7 step x 64
HEADER_SIZE = 64
NAME_LEN = 14            # MAXNUM_NAME(13) + NUL
NUM_JOINTS = 31          # position[] diindeks dengan ID dynamixel, 0 tidak dipakai
INVALID_BIT_MASK = 0x4000
TORQUE_OFF_BIT_MASK = 0x2000

# ID -> nama, sama persis dengan op3_manager/config/OP3.robot.
JOINT_NAME = {
    1: "r_sho_pitch",  2: "l_sho_pitch",  3: "r_sho_roll",   4: "l_sho_roll",
    5: "r_el",         6: "l_el",         7: "r_hip_yaw",    8: "l_hip_yaw",
    9: "r_hip_roll",  10: "l_hip_roll",  11: "r_hip_pitch", 12: "l_hip_pitch",
    13: "r_knee",     14: "l_knee",      15: "r_ank_pitch", 16: "l_ank_pitch",
    17: "r_ank_roll", 18: "l_ank_roll",  19: "head_pan",    20: "head_tilt",
}

# ===== Panjang link OP3 sesuai spesifikasi (m) =====
THIGH = 0.093
CALF = 0.093
ANKLE = 0.0335
HIP_Y = 0.037                        # setengah lebar pinggul
LEG_LENGTH = THIGH + CALF + ANKLE    # 0.2195

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BIN = os.path.join(
    REPO, "src/ROBOTIS-OP3/op3_action_module/data/motion_4095_ORION.bin")
PARAM_YAML = os.path.join(
    REPO, "src/ROBOTIS-OP3/op3_walking_module/config/param.yaml")


def read_page(path, index):
    """(nama, {nama_sendi: raw}) untuk step 0 dari halaman ke-index."""
    with open(path, "rb") as handle:
        handle.seek(index * PAGE_SIZE)
        page = handle.read(PAGE_SIZE)
    if len(page) < PAGE_SIZE:
        sys.exit("page %d di luar berkas %s (%d halaman)"
                 % (index, path, os.path.getsize(path) // PAGE_SIZE))

    name = page[:NAME_LEN].split(b"\x00")[0].decode("ascii", "replace").strip()
    stepnum = page[20]
    if stepnum == 0:
        sys.exit("page %d (%r) kosong: stepnum=0" % (index, name))

    raw = struct.unpack_from("<%dH" % NUM_JOINTS, page, HEADER_SIZE)
    joints, skipped = {}, []
    for jid, jname in JOINT_NAME.items():
        value = raw[jid]
        # Kedua bit ini bukan bagian dari sudut: INVALID berarti sendi tidak
        # ikut dimainkan halaman ini, TORQUE_OFF berarti dilemaskan. Kalau tidak
        # dibuang, nilainya terbaca belasan ribu dan FK-nya jadi omong kosong.
        if value & INVALID_BIT_MASK:
            skipped.append(jname)
            continue
        joints[jname] = value & ~TORQUE_OFF_BIT_MASK
    return name, stepnum, joints, skipped


def dxl_to_rad(raw):
    return (raw - 2048) * (2 * math.pi / 4096)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def matvec(a, v):
    return [sum(a[i][j] * v[j] for j in range(3)) for i in range(3)]


def add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def fk_leg(side, hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll):
    """side: +1 kiri, -1 kanan. Kaki kanan OP3 tanda pitch-nya terbalik."""
    q1, q2, q3, q4, q5, q6 = (dxl_to_rad(x) for x in
                              (hip_yaw, hip_roll, hip_pitch, knee, ank_pitch, ank_roll))
    if side == -1:
        q3, q4, q5 = -q3, -q4, -q5
    hip_origin = [0, side * HIP_Y, 0]
    rot = matmul(rot_z(q1), rot_x(q2))
    rot = matmul(rot, rot_y(q3))
    knee_origin = add(hip_origin, matvec(rot, [0, 0, -THIGH]))
    rot = matmul(rot, rot_y(q4))
    ank_origin = add(knee_origin, matvec(rot, [0, 0, -CALF]))
    rot = matmul(rot, rot_y(q5))
    rot = matmul(rot, rot_x(q6))
    return add(ank_origin, matvec(rot, [0, 0, -ANKLE]))


def param_offsets():
    """x/y/z_offset yang sedang dipakai walking_module, kalau param.yaml terbaca."""
    if not os.path.exists(PARAM_YAML):
        return {}
    found = {}
    with open(PARAM_YAML) as handle:
        for line in handle:
            match = re.match(r"\s*([xyz]_offset)\s*:\s*(-?[\d.]+)", line)
            if match:
                found[match.group(1)] = float(match.group(2))
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("=====")[0].strip())
    parser.add_argument("--bin", default=DEFAULT_BIN, help="berkas action (default: %(default)s)")
    parser.add_argument("--page", type=int, default=2, help="nomor halaman (default: 2)")
    parser.add_argument("--raw", action="store_true", help="cuma cetak nilai sendi, tanpa FK")
    args = parser.parse_args()

    if not os.path.exists(args.bin):
        sys.exit("berkas action tidak ada: %s" % args.bin)

    name, stepnum, joints, skipped = read_page(args.bin, args.page)
    print("%s  page %d: %r  (%d step)" % (os.path.basename(args.bin), args.page, name, stepnum))
    if skipped:
        print("  sendi tidak dimainkan halaman ini: %s" % ", ".join(skipped))
    print()
    for jid in sorted(JOINT_NAME):
        jname = JOINT_NAME[jid]
        if jname in joints:
            print("  %-12s %5d  (%+7.2f deg)"
                  % (jname, joints[jname], math.degrees(dxl_to_rad(joints[jname]))))
    if args.raw:
        return 0

    missing = [n for n in (
        "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
        "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
    ) if n not in joints]
    if missing:
        sys.exit("\nFK dilewati: halaman ini tidak memainkan %s" % ", ".join(missing))

    right = fk_leg(-1, joints["r_hip_yaw"], joints["r_hip_roll"], joints["r_hip_pitch"],
                   joints["r_knee"], joints["r_ank_pitch"], joints["r_ank_roll"])
    left = fk_leg(+1, joints["l_hip_yaw"], joints["l_hip_roll"], joints["l_hip_pitch"],
                  joints["l_knee"], joints["l_ank_pitch"], joints["l_ank_roll"])

    avg_x = (right[0] + left[0]) / 2
    avg_z = (right[2] + left[2]) / 2
    derived = {
        "x_offset": -avg_x,
        "y_offset": abs(right[1] - left[1]) / 2,
        "z_offset": LEG_LENGTH + avg_z,
    }

    print()
    print("Telapak kanan (m):  x=%+.4f  y=%+.4f  z=%+.4f" % tuple(right))
    print("Telapak kiri  (m):  x=%+.4f  y=%+.4f  z=%+.4f" % tuple(left))
    print()
    in_use = param_offsets()
    print("%-12s %10s %10s" % ("", "hasil FK", "param.yaml"))
    for key in ("x_offset", "y_offset", "z_offset"):
        actual = in_use.get(key)
        print("%-12s %+10.4f %10s"
              % (key, derived[key], "%+.4f" % actual if actual is not None else "-"))

    print()
    sane = 0.02 < derived["z_offset"] < 0.10
    print("kewarasan (0.02 < z_offset < 0.10): %s" % sane)
    if not sane:
        print("!! z_offset di luar rentang -> FK tidak bisa dipercaya untuk kalibrasi robot ini.")
        print("!! JANGAN disalin ke param.yaml. Angka di kolom kanan sudah benar:")
        print("!! walking_module menangkap pose page 2 saat runtime (capture/bias),")
        print("!! jadi tidak bergantung pada FK ini sama sekali.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

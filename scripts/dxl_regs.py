#!/usr/bin/env python3
"""Baca register Dynamixel protokol 2.0 (tanpa dependensi), + mode pantau.

Saudara dari op3_ping.py: kalau op3_ping.py menjawab "servo mana yang hidup",
berkas ini menjawab "servo itu dikonfigurasi seperti apa". Sengaja tidak
memakai dynamixel_sdk dengan alasan yang sama -- modulnya tidak ada di image
robot ini, dan alat diagnosa harus tetap jalan justru ketika lingkungannya
bermasalah.

    scripts/dxl_regs.py                      # tabel register sendi kaki
    scripts/dxl_regs.py --ids 1-20           # semua sendi
    scripts/dxl_regs.py --watch 15,16        # pantau present_position + delta

Kenapa dua register ini yang paling sering dicari:

  homing_offset (20, 4 byte, bertanda)
      Mengompensasi horn servo yang terpasang miring. Present Position =
      posisi encoder + homing_offset, jadi nilai goal yang kita tulis sudah
      berada di frame logis. JANGAN mengurangkan homing_offset dari goal untuk
      mendapat "sudut fisik" -- itu justru membatalkan normalisasinya.

  drive_mode (10, 1 byte)
      Bit 0: 0 = Normal, 1 = Reverse. Pada pasangan sendi kiri-kanan, nilai ini
      HARUS sama di dalam satu pasangan: pencerminan kiri-kanan sudah diurus di
      software lewat getJointDirection() (joint_axis di op3_kinematics_dynamics).
      Kalau satu servo Reverse dan pasangannya Normal, perintah gait yang
      bercermin menghasilkan gerakan fisik yang TIDAK bercermin.

      Ini beda sifat dari homing_offset. Offset konstan diserap oleh
      walking_bias_ di op3_walking_module (bias = captured - neutral), jadi
      homing_offset seliar apa pun tidak merusak gait. Pembalikan tanda TIDAK
      terserap -- itu muncul sebagai kaki yang berayun ke arah salah.

Mode --watch dipakai untuk membuktikan arah sendi tanpa memerintah gerakan
sama sekali: torque OFF, gerakkan sendi dengan TANGAN, lihat delta angkanya.
Dua servo sepasang yang digerakkan ke arah fisik yang SAMA harus menghasilkan
delta dengan tanda BERLAWANAN (karena terpasang bercermin). Kalau tandanya
sama, drive_mode-nya tidak sepasang.
"""

import argparse
import os
import re
import sys
import termios
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_FILE = os.path.join(WS, "src/ROBOTIS-OP3/op3_manager/config/OP3.robot")

GREEN, RED, YELLOW, DIM, BOLD, RST = (
    "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[1m", "\033[0m")

NAMES = {
    1: "r_sho_pitch", 2: "l_sho_pitch", 3: "r_sho_roll", 4: "l_sho_roll",
    5: "r_el", 6: "l_el", 7: "r_hip_yaw", 8: "l_hip_yaw",
    9: "r_hip_roll", 10: "l_hip_roll", 11: "r_hip_pitch", 12: "l_hip_pitch",
    13: "r_knee", 14: "l_knee", 15: "r_ank_pitch", 16: "l_ank_pitch",
    17: "r_ank_roll", 18: "l_ank_roll", 19: "head_pan", 20: "head_tilt",
}

# Pasangan cermin kiri-kanan (id kanan, id kiri).
MIRROR_PAIRS = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10),
                (11, 12), (13, 14), (15, 16), (17, 18)]

LEG_IDS = list(range(7, 19))

# (nama, alamat, panjang byte, bertanda) -- control table protokol 2.0.
REGS = [
    ("drive_mode", 10, 1, False),
    ("oper_mode", 11, 1, False),
    ("homing_offset", 20, 4, True),
    ("torque_on", 64, 1, False),
    ("pos_p_gain", 84, 2, False),
    ("goal_pos", 116, 4, True),
    ("present_pos", 132, 4, True),
]


def crc16(data):
    """CRC-16/BUYPASS (poly 0x8005) -- CRC protokol 2.0 Dynamixel."""
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_packet(dxl_id, addr, length):
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x07, 0x00, 0x02,
                  addr & 0xFF, (addr >> 8) & 0xFF, length & 0xFF, (length >> 8) & 0xFF])
    crc = crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, "B%d" % baud, None)
    if speed is None:
        os.close(fd)
        raise ValueError("baud %d tidak didukung termios" % baud)
    attrs[0] = 0                      # iflag
    attrs[1] = 0                      # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                      # lflag
    attrs[4] = attrs[5] = speed
    attrs[6] = list(attrs[6])
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_reg(fd, dxl_id, addr, length, signed=False, timeout=0.06):
    termios.tcflush(fd, termios.TCIOFLUSH)
    os.write(fd, read_packet(dxl_id, addr, length))
    deadline = time.time() + timeout
    buf = b""
    want = 11 + length
    while time.time() < deadline:
        try:
            chunk = os.read(fd, 64)
        except BlockingIOError:
            chunk = b""
        if chunk:
            buf += chunk
            idx = buf.find(b"\xff\xff\xfd\x00")
            if idx >= 0 and len(buf) >= idx + want:
                pkt = buf[idx:idx + want]
                if pkt[4] == dxl_id and pkt[7] == 0x55:
                    return int.from_bytes(pkt[9:9 + length], "little", signed=signed)
        else:
            time.sleep(0.001)
    return None


def ports_from_robot_file():
    """Port yang benar-benar dipakai OP3.robot (komentar dibuang dulu)."""
    if not os.path.isfile(ROBOT_FILE):
        return []
    found, seen = [], set()
    with open(ROBOT_FILE) as handle:
        for line in handle:
            for match in re.findall(r"/dev/[^\s|]+", line.split("#", 1)[0]):
                if match not in seen:
                    seen.add(match)
                    found.append(match)
    return found


def parse_ids(spec):
    ids = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-")
            ids.extend(range(int(a), int(b) + 1))
        else:
            ids.append(int(part))
    return ids


def cmd_table(fd, ids):
    header = f"{'ID':>3} {'nama':<14}" + "".join(f"{n:>15}" for n, _, _, _ in REGS)
    print(header)
    print("-" * len(header))
    values = {}
    for dxl_id in ids:
        values[dxl_id] = {}
        row = f"{dxl_id:3d} {NAMES.get(dxl_id, '?'):<14}"
        for name, addr, length, signed in REGS:
            val = read_reg(fd, dxl_id, addr, length, signed)
            values[dxl_id][name] = val
            row += f"{'--':>15}" if val is None else f"{val:>15d}"
        print(row)
    return values


def report_pairs(values):
    pairs = [(r, l) for r, l in MIRROR_PAIRS if r in values and l in values]
    if not pairs:
        return
    print(f"\n{BOLD}Pemeriksaan pasangan cermin{RST}")
    problems = 0
    for r, l in pairs:
        dm_r, dm_l = values[r].get("drive_mode"), values[l].get("drive_mode")
        if dm_r is None or dm_l is None:
            continue
        label = f"  {NAMES[r]:<13} vs {NAMES[l]:<13}"
        if dm_r != dm_l:
            problems += 1
            print(f"{label} {RED}drive_mode {dm_r} / {dm_l} -- TIDAK SEPASANG{RST}")
            print(f"{DIM}      Pencerminan sudah diurus software; satu servo Reverse dan"
                  f" pasangannya Normal{RST}")
            print(f"{DIM}      membuat gait berayun ke arah fisik yang salah di sendi ini.{RST}")
        else:
            print(f"{label} {GREEN}drive_mode {dm_r} / {dm_l} sepasang{RST}")

    print(f"\n{BOLD}Simetri goal_position (jumlah kiri+kanan idealnya 4096){RST}")
    print(f"{DIM}  Homing offset TIDAK dikurangkan: goal sudah berada di frame logis.{RST}")
    for r, l in pairs:
        gr, gl = values[r].get("goal_pos"), values[l].get("goal_pos")
        if gr is None or gl is None:
            continue
        total = gr + gl
        dev = total - 4096
        deg = dev * 0.087890625
        mark = GREEN if abs(dev) <= 120 else (YELLOW if abs(dev) <= 400 else RED)
        print(f"  {NAMES[r]:<13} {gr:5d} + {NAMES[l]:<13} {gl:5d} = {total:5d}"
              f"  {mark}simpangan {dev:+6d} ({deg:+6.1f} derajat){RST}")

    if problems:
        print(f"\n{RED}{problems} pasangan bermasalah.{RST} Buktikan dulu dengan"
              f" --watch sebelum menulis EEPROM:")
        print(f"{DIM}  torque OFF, gerakkan kedua sendi ke arah fisik yang SAMA dengan tangan.{RST}")
        print(f"{DIM}  Delta bertanda berlawanan = sepasang. Tanda sama = drive_mode kacau.{RST}")


def cmd_watch(fd, ids, interval):
    base = {}
    print(f"{BOLD}Pantau present_position{RST} -- Ctrl-C untuk berhenti")
    print(f"{DIM}Torque harus OFF. Gerakkan sendi dengan TANGAN ke arah fisik yang sama,{RST}")
    print(f"{DIM}lalu bandingkan tanda kolom delta.{RST}\n")
    for dxl_id in ids:
        val = read_reg(fd, dxl_id, 64, 1)
        if val:
            print(f"{YELLOW}  peringatan: torque ID {dxl_id} ({NAMES.get(dxl_id, '?')})"
                  f" masih ON -- sendi tidak bisa digerakkan tangan{RST}")
    header = "  " + "".join(f"{NAMES.get(i, i):>22}" for i in ids)
    print(header)
    try:
        while True:
            row = "  "
            for dxl_id in ids:
                pos = read_reg(fd, dxl_id, 132, 4, True)
                if pos is None:
                    row += f"{'--':>22}"
                    continue
                if dxl_id not in base:
                    base[dxl_id] = pos
                delta = pos - base[dxl_id]
                colour = GREEN if delta > 15 else (RED if delta < -15 else DIM)
                row += f"{colour}{pos:8d} ({delta:+6d}){RST}".rjust(22 + len(colour) + len(RST))
            print(row, end="\r", flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n")
        print(f"{BOLD}Delta akhir{RST}")
        for dxl_id in ids:
            pos = read_reg(fd, dxl_id, 132, 4, True)
            if pos is None or dxl_id not in base:
                continue
            print(f"  {NAMES.get(dxl_id, dxl_id):<14} {base[dxl_id]:5d} -> {pos:5d}"
                  f"  delta {pos - base[dxl_id]:+6d}")


def main():
    ap = argparse.ArgumentParser(
        description="Baca register Dynamixel protokol 2.0 (tanpa dependensi)")
    ap.add_argument("--port", help="port (default: dari OP3.robot)")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--ids", default=None,
                    help="daftar ID, contoh '7-18' atau '13,14' (default: sendi kaki)")
    ap.add_argument("--watch", metavar="IDS",
                    help="pantau present_position untuk ID ini, contoh '15,16'")
    ap.add_argument("--interval", type=float, default=0.15)
    args = ap.parse_args()

    ports = [args.port] if args.port else ports_from_robot_file()
    if not ports:
        print("Tidak ada port. Beri --port, atau pastikan OP3.robot terbaca.", file=sys.stderr)
        return 1

    fd = open_port(ports[0], args.baud)
    print(f"{BOLD}{ports[0]}{RST}  @ {args.baud} baud\n")
    try:
        if args.watch:
            cmd_watch(fd, parse_ids(args.watch), args.interval)
        else:
            ids = parse_ids(args.ids) if args.ids else LEG_IDS
            report_pairs(cmd_table(fd, ids))
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())

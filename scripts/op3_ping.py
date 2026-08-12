#!/usr/bin/env python3
"""Ping perangkat Dynamixel protokol 2.0 di sebuah port. Tanpa dependensi.

Sengaja tidak memakai dynamixel_sdk: modul Python-nya tidak terpasang di image
ini, dan alat diagnosa harus tetap jalan justru ketika lingkungannya bermasalah.
Cukup os.open + termios.

Gunanya saat memasang repo ini di robot baru: membuktikan siapa yang benar-benar
ada di bus, sebelum menyalahkan konfigurasi.

    scripts/op3_ping.py                          # scan ID 1-20 + 200 di port OP3.robot
    scripts/op3_ping.py --port /dev/ttyUSB0      # port lain
    scripts/op3_ping.py --ids 200                # satu ID saja
    scripts/op3_ping.py --baud 1000000           # baud lain (default 2000000)

Membaca: "ID 200 DIAM di bus servo tapi 20 servo menjawab" = transceiver TTL
OpenCR mati; pakai `op3_serial_setup.py --opencr=usb`. Kalau ID 200 menjawab di
bus servo, board-nya sehat: pakai `--opencr=bus` (itu susunan OP3 normal).
"""

import argparse
import os
import re
import sys
import termios
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_FILE = os.path.join(WS, "src/ROBOTIS-OP3/op3_manager/config/OP3.robot")

GREEN, RED, DIM, BOLD, RST = "\033[32m", "\033[31m", "\033[2m", "\033[1m", "\033[0m"


def crc16(data):
    """CRC-16/BUYPASS (poly 0x8005) -- CRC protokol 2.0 Dynamixel."""
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ping_packet(dxl_id):
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id, 0x03, 0x00, 0x01])
    crc = crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, "B%d" % baud, None)
    if speed is None:
        os.close(fd)
        raise ValueError("baud %d tidak didukung termios" % baud)
    # Raw 8N1, tanpa flow control, tanpa echo, tanpa pemrosesan apa pun.
    attrs[0] = 0                      # iflag
    attrs[1] = 0                      # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                      # lflag
    attrs[4] = attrs[5] = speed       # ispeed, ospeed
    attrs[6] = list(attrs[6])
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def ping(fd, dxl_id, timeout=0.05):
    termios.tcflush(fd, termios.TCIOFLUSH)
    os.write(fd, ping_packet(dxl_id))
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        try:
            chunk = os.read(fd, 64)
        except BlockingIOError:
            chunk = b""
        if chunk:
            buf += chunk
            # Status ping: header + id + len + 0x55 + err + model_l + model_h + fw + crc
            idx = buf.find(b"\xff\xff\xfd\x00")
            if idx >= 0 and len(buf) >= idx + 14:
                pkt = buf[idx:idx + 14]
                if pkt[4] == dxl_id and pkt[7] == 0x55:
                    return pkt[9] | (pkt[10] << 8), pkt[8]  # model, error
        else:
            time.sleep(0.001)
    return None, None


def ports_from_robot_file():
    if not os.path.isfile(ROBOT_FILE):
        return []
    found, seen = [], set()
    for line in open(ROBOT_FILE):
        if line.lstrip().startswith("#"):
            continue
        for m in re.findall(r"/dev/serial/by-id/\S+", line):
            if m not in seen:
                seen.add(m)
                found.append(m)
    return found


def main():
    ap = argparse.ArgumentParser(description="Ping Dynamixel protokol 2.0 (tanpa dependensi)")
    ap.add_argument("--port", action="append", help="port; boleh diulang (default: dari OP3.robot)")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--ids", help="daftar ID, contoh '1-20,200' (default: 1-20,200)",
                    default="1-20,200")
    args = ap.parse_args()

    ids = []
    for part in args.ids.split(","):
        if "-" in part:
            a, b = part.split("-")
            ids.extend(range(int(a), int(b) + 1))
        else:
            ids.append(int(part))

    ports = args.port or ports_from_robot_file()
    if not ports:
        print("Tidak ada port. Beri --port, atau pastikan OP3.robot terbaca.", file=sys.stderr)
        return 1

    rc = 0
    for path in ports:
        print("%s%s%s  @ %d baud" % (BOLD, path, RST, args.baud))
        if not os.path.exists(path):
            print("  %sgagal%s port tidak ada di sistem\n" % (RED, RST))
            rc = 1
            continue
        try:
            fd = open_port(path, args.baud)
        except OSError as exc:
            print("  %sgagal%s tidak bisa dibuka: %s" % (RED, RST, exc))
            print("  %s(port dipakai proses lain? manager/editor masih jalan?)%s\n" % (DIM, RST))
            rc = 1
            continue
        answered = []
        for i in ids:
            model, err = ping(fd, i)
            if model is not None:
                answered.append((i, model, err))
        os.close(fd)

        if not answered:
            print("  %stidak ada yang menjawab%s" % (RED, RST))
        for i, model, err in answered:
            note = "  <- OpenCR di bus ini" if i == 200 else ""
            print("  %sok%s  ID %3d  model 0x%04X  err=%d%s" % (GREEN, RST, i, model, err, note))
        missing = [i for i in ids if i not in [a[0] for a in answered]]
        if missing:
            print("  %sdiam: %s%s" % (DIM, ",".join(str(m) for m in missing), RST))
        print()
    return rc


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Ping perangkat Dynamixel protokol 2.0 di sebuah port. Tanpa dependensi.

Sengaja tidak memakai dynamixel_sdk: modul Python-nya tidak terpasang di image
robot ini (dibuktikan 2026-08-12 -- `python3 -c "import dynamixel_sdk"` di dalam
container gagal meski install/setup.bash sudah di-source), dan alat diagnosa
justru harus tetap jalan ketika lingkungannya bermasalah. Cukup os.open +
termios.

    scripts/op3_ping.py                          # scan ID 1-20 + 200 di port yang terdeteksi
    scripts/op3_ping.py --port /dev/ttyUSB1      # port lain
    scripts/op3_ping.py --ids 200                # satu ID saja
    scripts/op3_ping.py --baud 1000000           # baud lain (default 2000000)

Port default mengikuti urutan yang sama dengan detect_dynamixel_device() di
op3_manager/launch/op3_manager.launch.py: env OP3_DEVICE menang, lalu port yang
tertulis di OP3.robot kalau memang ada, lalu /dev/ttyUSB0-9, lalu /dev/ttyOP3.
Nomor ttyUSB memang tidak dipegang mati di robot ini -- adapter FTDI bisa pindah
antara ttyUSB0/ttyUSB1 tiap kali dicolok ulang.

Membacanya di robot ini:
    - Semua diam, termasuk 200  -> bukan servo mati. Bus DXL-nya yang tidak ada:
      cek `lsusb | grep 0403` dan `ls -l /dev/ttyUSB*`.
    - 20 servo + ID 200 menjawab di satu port -> susunan normal: OpenCR di-flash
      opencr_op3 standar, ID 200 dilayani di DXL_PORT = Serial3, bus TTL yang
      sama dengan servo (persis seperti OP3.robot menuliskannya).
    - Sebagian ID diam berurutan dari satu titik ke bawah -> kabel putus di
      antara servo terakhir yang menjawab dan yang pertama diam, bukan 4 servo
      rusak berbarengan.
    - "tidak bisa dibuka" -> port dipegang proses lain. Bus DXL bermaster
      tunggal: op3_manager dan op3_action_editor tidak boleh jalan bersamaan.

Kolom err menampakkan bit hardware-error yang ter-latch di servo -- err != 0
berarti servo menjawab tapi sedang mengeluh. Bit-nya (control table XM430,
alamat 70): 0x01 input voltage, 0x04 overheating, 0x08 encoder, 0x10 electrical
shock, 0x20 overload. OVERLOAD dan OVERHEAT ter-latch: torsinya tidak akan
menyala lagi sampai servo di-reboot atau robot dimatikan-nyalakan.
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

# Arti bit hardware error (control table XM430-W350, alamat 70).
HW_ERROR_BITS = [
    (0x01, "INPUT_VOLTAGE"),
    (0x04, "OVERHEATING"),
    (0x08, "ENCODER"),
    (0x10, "ELECTRICAL_SHOCK"),
    (0x20, "OVERLOAD"),
]


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


def describe_error(err):
    if not err:
        return ""
    names = [name for bit, name in HW_ERROR_BITS if err & bit]
    return "  <- " + " ".join(names) if names else ""


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
    """Port yang tertulis di OP3.robot.

    Komentar dibuang dulu supaya baris penjelasan tidak ikut terbaca sebagai
    port wajib.
    """
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


def default_ports():
    """Urutan yang sama dengan detect_dynamixel_device() di op3_manager.launch.py.

    OP3.robot boleh saja menyebut ttyUSB0 sementara adapternya sedang di ttyUSB1:
    launch file menambal salinan OP3.robot dengan port hasil pindaian sebelum
    manager start, jadi alat ini harus memindai dengan cara yang sama -- kalau
    tidak, ia melapor "port tidak ada" untuk bus yang sebenarnya sehat.

    Disaring dengan realpath, sama seperti dxl_candidates() di
    scripts/op3_docker.sh: /dev/ttyOP3 adalah symlink udev ke salah satu ttyUSB
    yang sudah disebut di atas, dan di dalam container symlink itu ikut terlihat
    (karena /dev di-bind). Tanpa saringan ini satu bus fisik di-ping dua kali dan
    laporannya seolah ada dua bus yang sama-sama diam.
    """
    override = os.environ.get("OP3_DEVICE", "").strip()
    if override:
        return [override]
    candidates = [p for p in ports_from_robot_file() if os.path.exists(p)]
    candidates += ["/dev/ttyUSB%d" % i for i in range(10)]
    candidates.append("/dev/ttyOP3")

    ports, seen = [], set()
    for dev in candidates:
        if not os.path.exists(dev):
            continue
        real = os.path.realpath(dev)
        if real in seen:
            continue
        seen.add(real)
        ports.append(dev)
    return ports


def main():
    ap = argparse.ArgumentParser(description="Ping Dynamixel protokol 2.0 (tanpa dependensi)")
    ap.add_argument("--port", action="append", help="port; boleh diulang (default: auto-detect)")
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

    ports = args.port or default_ports()
    if not ports:
        print("Tidak ada port sama sekali. Cek `lsusb | grep 0403` -- kalau kosong,"
              " adapter DXL-nya yang tidak tercolok.", file=sys.stderr)
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
            note = "  <- OpenCR di bus ini" if i == 200 else describe_error(err)
            print("  %sok%s  ID %3d  model 0x%04X  err=%d%s" % (GREEN, RST, i, model, err, note))
        missing = [i for i in ids if i not in [a[0] for a in answered]]
        if missing:
            print("  %sdiam: %s%s" % (DIM, ",".join(str(m) for m in missing), RST))
        print()
    return rc


if __name__ == "__main__":
    sys.exit(main())

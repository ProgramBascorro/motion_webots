#!/usr/bin/env python3
"""
opencr_check.py - Diagnostik komunikasi OpenCR (ID 200) di bus OP3.

Membuktikan apakah OpenCR benar-benar menjawab sebagai device ID 200 dan
mengembalikan data hidup (tegangan, tombol, gyro, roll/pitch/yaw). Alamat
register diambil persis dari OPEN-CR.device milik repo ini.

Pakai:
    # sourcekan dulu workspace supaya dynamixel_sdk (python) tersedia
    source install/setup.bash

    # cek OpenCR (ID 200) di bus DXL
    python3 scripts/opencr_check.py --port /dev/ttyOP3

    # sekalian scan semua servo + OpenCR di bus
    python3 scripts/opencr_check.py --port /dev/ttyOP3 --scan

Catatan penting soal topologi (robot ini):
    - /dev/ttyOP3 = U2D2 (FTDI 0403:6014), master bus TTL. Firmware standar
      opencr_op3 melayani ID 200 di DXL_PORT = Serial3, yaitu bus TTL yang sama
      dengan servo. Jadi di /dev/ttyOP3 servo 1-20 DAN ID 200 sama-sama menjawab.
    - /dev/ttyACM0 = micro-USB OpenCR, hanya konsol debug/flash. Port ini tidak
      pernah membalas paket DXL, jadi wajar kalau scan di sana kosong total.
    - Kalau tidak ada satu pun ID menjawab, cek dulu U2D2-nya benar tercolok:
      lsusb | grep 0403   (kosong = adapter tidak ada, bukan servo mati)
    - Board yang di-flash opencr_op3_usb berperilaku sebaliknya: ID 200 dilayani
      lewat micro-USB-nya, dan OP3.robot harus memberi sensor OPEN-CR port itu.
"""

import argparse
import sys

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    sys.exit(
        "ERROR: modul 'dynamixel_sdk' tidak ditemukan.\n"
        "Jalankan dulu:  source install/setup.bash  (atau build paket DynamixelSDK), "
        "lalu ulangi perintah ini."
    )

PROTOCOL = 2.0
OPENCR_ID = 200

# addr, name, length(bytes), signed - persis dari OPEN-CR.device
READ_ITEMS = [
    (0,  "model_number",         2, False),
    (2,  "version_of_firmware",  1, False),
    (3,  "ID",                   1, False),
    (30, "button",               1, False),
    (31, "present_voltage",      1, False),
    (32, "gyro_x",               2, True),
    (34, "gyro_y",               2, True),
    (36, "gyro_z",               2, True),
    (44, "roll",                 2, False),
    (46, "pitch",                2, False),
    (48, "yaw",                  2, False),
]


def to_signed(value, length):
    bits = 8 * length
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


def read_item(packet, port, dxl_id, addr, length):
    if length == 1:
        val, comm, err = packet.read1ByteTxRx(port, dxl_id, addr)
    elif length == 2:
        val, comm, err = packet.read2ByteTxRx(port, dxl_id, addr)
    else:
        val, comm, err = packet.read4ByteTxRx(port, dxl_id, addr)
    return val, comm, err


def ping(packet, port, dxl_id):
    model, comm, err = packet.ping(port, dxl_id)
    return model, comm, err


def main():
    ap = argparse.ArgumentParser(description="Diagnostik OpenCR / bus OP3")
    ap.add_argument("--port", default="/dev/ttyOP3")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--id", type=int, default=OPENCR_ID, help="ID OpenCR (default 200)")
    ap.add_argument("--scan", action="store_true", help="scan ID 1-20 + 200")
    args = ap.parse_args()

    port = PortHandler(args.port)
    packet = PacketHandler(PROTOCOL)

    if not port.openPort():
        sys.exit(f"ERROR: gagal membuka port {args.port} "
                 f"(port dipakai proses lain? op3_manager masih jalan? "
                 f"jalankan: fuser {args.port})")
    if not port.setBaudRate(args.baud):
        sys.exit(f"ERROR: gagal set baudrate {args.baud}")

    print(f"Port  : {args.port} @ {args.baud} bps, protocol {PROTOCOL}")
    print("-" * 56)

    if args.scan:
        print("Scan bus (ID 1-20 + 200):")
        found = []
        for dxl_id in list(range(1, 21)) + [OPENCR_ID]:
            model, comm, err = ping(packet, port, dxl_id)
            if comm == COMM_SUCCESS:
                found.append(dxl_id)
                tag = "  <-- OpenCR" if dxl_id == OPENCR_ID else ""
                print(f"  ID {dxl_id:3d}  OK   model={model}{tag}")
        if not found:
            print("  (tidak ada device menjawab)")
            print("  -> Cek U2D2 tercolok (lsusb | grep 0403) dan power robot menyala.")
        elif OPENCR_ID not in found:
            print(f"\n  Servo terbaca {found}, tapi ID 200 (OpenCR) TIDAK menjawab.")
            print("  -> OpenCR tidak ada di bus ini: tidak bertenaga, atau di-flash")
            print("     opencr_op3_usb sehingga ID 200 hanya ada di micro-USB-nya.")
        print("-" * 56)

    print(f"Ping OpenCR (ID {args.id}) ...")
    model, comm, err = ping(packet, port, args.id)
    if comm != COMM_SUCCESS:
        print(f"  GAGAL: {packet.getTxRxResult(comm)}")
        print("\n  Kesimpulan: OpenCR tidak menjawab di port ini.")
        print("  Cek: (1) U2D2 tercolok dan port ini memang bus TTL (lsusb | grep 0403);")
        print("       (2) OpenCR bertenaga dan kabel TTL-nya masuk ke bus servo;")
        print("       (3) tidak ada proses lain memegang port (fuser %s)." % args.port)
        port.closePort()
        sys.exit(1)

    print(f"  MENJAWAB. model_number={model}")
    print("-" * 56)
    print("Data hidup dari OpenCR:")
    for addr, name, length, signed in READ_ITEMS:
        val, comm, err = read_item(packet, port, args.id, addr, length)
        if comm != COMM_SUCCESS:
            print(f"  {name:20s} : <gagal baca>")
            continue
        if signed:
            val = to_signed(val, length)
        if name == "present_voltage":
            print(f"  {name:20s} : {val} ({val * 0.1:.1f} V)")
        else:
            print(f"  {name:20s} : {val}")

    print("-" * 56)
    print("OK: OpenCR terbaca dan mengirim data. Komunikasi ID 200 sehat.")
    port.closePort()


if __name__ == "__main__":
    main()

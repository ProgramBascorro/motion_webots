#!/usr/bin/env python3
"""
opencr_check.py - Diagnostik komunikasi OpenCR (ID 200) di robot CHRONUS.

Membuktikan apakah OpenCR benar-benar menjawab sebagai device ID 200 dan
mengembalikan data hidup (tegangan, tombol, gyro, roll/pitch/yaw, magnetometer).
Alamat register diambil persis dari OPEN-CR.device milik repo ini.

Pakai:
    # sourcekan dulu workspace supaya dynamixel_sdk (python) tersedia
    source install/setup.bash

    # cek OpenCR di micro-USB-nya
    python3 scripts/opencr_check.py

    # scan servo di bus U2D2 (di sana ID 200 memang TIDAK ada lagi)
    python3 scripts/opencr_check.py --port /dev/ttyOP3 --scan

    # perintah kalibrasi tanpa ROS
    python3 scripts/opencr_check.py --zero-yaw
    python3 scripts/opencr_check.py --cali-mag     # lalu putar robot 20 detik

Topologi robot ini (firmware "opencr_op3 edited by Fayyadh Raja Gula"):
    - /dev/ttyOPENCR (micro-USB OpenCR, CDC) = sub controller ID 200 saja.
      Firmware memindahkan DXL_PORT dari Serial3 ke Serial, jadi IMU dan
      magnetometer dilayani lewat USB, terpisah dari jalur servo.
    - /dev/ttyOP3 (U2D2, FTDI 0403:6014) = 20 servo saja. ID 200 TIDAK akan
      menjawab di sini, dan itu memang yang diinginkan -- justru itu tujuan
      pemisahan jalurnya: bulk read servo tidak lagi berebut bus dengan IMU.
    - Kalau tidak ada satu pun ID menjawab di ttyOP3, cek U2D2-nya dulu:
      lsusb | grep 0403   (kosong = adapter tidak ada, bukan servo mati)
    - Board dengan firmware BAWAAN berperilaku sebaliknya: ID 200 ada di bus
      TTL bersama servo, dan micro-USB-nya cuma konsol debug.

Firmware yang benar melaporkan version_of_firmware = 3. Kalau yang muncul 2,
board masih memakai firmware lama: register magnetometer akan terbaca nol dan
yaw masih hasil integrasi gyro yang melayang.
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
    (38, "acc_x",                2, True),
    (40, "acc_y",                2, True),
    (42, "acc_z",                2, True),
    (44, "roll",                 2, True),
    (46, "pitch",                2, True),
    (48, "yaw",                  2, True),
    # --- fw 0x03 ke atas: magnetometer ---
    (51, "mag_x",                2, True),
    (53, "mag_y",                2, True),
    (55, "mag_z",                2, True),
    (57, "mag_heading",          2, False),
    (59, "yaw_gyro",             2, True),
    (61, "mag_status",           1, False),
    (6,  "mag_x_offset",         2, True),
    (8,  "mag_y_offset",         2, True),
    (10, "mag_z_offset",         2, True),
    (12, "mag_cali_valid",       1, False),
]

# Register yang isinya 0.1 derajat, bukan cacahan mentah.
DECIDEG_ITEMS = {"roll", "pitch", "yaw", "mag_heading", "yaw_gyro"}

ADDR_IMU_CONTROL = 50
IMU_CTRL_CALI_GYRO = 1 << 3
IMU_CTRL_CALI_MAG = 1 << 4
IMU_CTRL_ZERO_YAW = 1 << 5

MAG_STATUS_TEXT = {
    0: "no_sensor  (papan tidak mengirim data magnetometer)",
    1: "settling   (menunggu referensi 0 derajat, normal ~3 detik awal)",
    2: "ok         (yaw kompas valid)",
    3: "calibrating(sedang kalibrasi hard-iron, putar robot)",
}



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
    ap.add_argument("--port", default="/dev/ttyOPENCR")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--id", type=int, default=OPENCR_ID, help="ID OpenCR (default 200)")
    ap.add_argument("--scan", action="store_true", help="scan ID 1-20 + 200")
    ap.add_argument("--zero-yaw", action="store_true",
                    help="jadikan arah robot saat ini sebagai yaw 0 derajat")
    ap.add_argument("--cali-mag", action="store_true",
                    help="mulai kalibrasi hard-iron 20 detik (putar robot pelan)")
    ap.add_argument("--cali-gyro", action="store_true",
                    help="mulai kalibrasi bias gyro (robot harus diam)")
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
        elif name == "mag_status":
            print(f"  {name:20s} : {val}  {MAG_STATUS_TEXT.get(val, '?')}")
        elif name == "version_of_firmware":
            note = "" if val >= 3 else "  <-- firmware LAMA, magnetometer tidak ada"
            print(f"  {name:20s} : {val}{note}")
        elif name == "mag_cali_valid":
            note = "  (offset hasil kalibrasi)" if val == 0x5A else "  (belum pernah dikalibrasi)"
            print(f"  {name:20s} : 0x{val:02X}{note}")
        elif name in DECIDEG_ITEMS:
            print(f"  {name:20s} : {val} ({val * 0.1:.1f} deg)")
        else:
            print(f"  {name:20s} : {val}")

    print("-" * 56)

    # Firmware membersihkan sendiri bit perintahnya setelah selesai
    # (dxl_node_op3_loop), jadi cukup satu tulisan -- tidak perlu menulis nol
    # sesudahnya, dan menulis nol lebih awal justru membatalkan kalibrasinya.
    commands = []
    if args.zero_yaw:
        commands.append((IMU_CTRL_ZERO_YAW, "yaw di-nol-kan di arah sekarang"))
    if args.cali_mag:
        commands.append((IMU_CTRL_CALI_MAG,
                         "kalibrasi magnetometer dimulai -- putar robot pelan "
                         "satu putaran penuh selama 20 detik"))
    if args.cali_gyro:
        commands.append((IMU_CTRL_CALI_GYRO,
                         "kalibrasi gyro dimulai -- jangan gerakkan robot"))

    for bits, message in commands:
        comm, err = packet.write1ByteTxRx(port, args.id, ADDR_IMU_CONTROL, bits)
        if comm != COMM_SUCCESS:
            print(f"  GAGAL kirim perintah 0x{bits:02X}: {packet.getTxRxResult(comm)}")
        else:
            print(f"  {message}")

    if commands:
        print("-" * 56)

    print("OK: OpenCR terbaca dan mengirim data. Komunikasi ID 200 sehat.")
    port.closePort()


if __name__ == "__main__":
    main()

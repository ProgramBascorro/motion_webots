#!/usr/bin/env python3
"""Lepas (atau pasang) batas kecepatan servo: profile_velocity + profile_acceleration.

KENAPA ADA
----------
Tombol "Torque ON" di Bascorro Studio memasang batas kecepatan di servo supaya
sendi tidak menyentak saat torsi menyala. Batas itu SENGAJA dibiarkan terpasang
sampai dilepas -- dan ia hidup di RAM servo, jadi bertahan melewati restart
op3_manager. Yang melepasnya cuma Studio (sekarang otomatis sebelum tiap gait)
atau mematikan daya servo.

Kalau batas itu tertinggal, robot berjalan PELAN DENGAN LANGKAH KECIL padahal
parameter walking-nya tidak diubah sedikit pun. Terukur 2026-09-09 dengan preset
JALAN MENUJU MEKKAH:

    batas terpasang : perintah ayun pinggul 23,21 deg -> nyata  5,0 deg  (22%)
                      perintah ayun lutut   36,30 deg -> nyata  4,0 deg  (11%)
                      telat ~350 ms, hampir setengah periode gait
    dilepas (0)     : pinggul 23,21 -> 17,50 deg (75%), lutut 36,30 -> 21,88 (60%)

Berdiri diam TIDAK terpengaruh (galat cuma 0,2-0,4 deg), jadi gejalanya hanya
kelihatan saat berjalan -- itulah kenapa dulu susah dilacak.

CATATAN: 0 = TANPA BATAS di tabel kontrol Dynamixel, dan itu nilai pabrik. Jadi
menjalankan skrip ini saat tidak ada batas sama sekali tidak berbahaya.

PAKAI
-----
    python3 scripts/clear_servo_speed_limit.py              # lepas (jadi 0)
    python3 scripts/clear_servo_speed_limit.py --set 20     # pasang batas ~27 deg/s
    python3 scripts/clear_servo_speed_limit.py --kaki       # kaki saja

Jalankan DI DALAM kontainer, dengan op3_manager hidup:
    docker exec op3 bash -lc 'source /opt/ros/humble/setup.bash \
        && source /ros2_ws/install/setup.bash \
        && python3 /ros2_ws/scripts/clear_servo_speed_limit.py'
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from robotis_controller_msgs.msg import SyncWriteItem

KAKI = ["r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
        "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee",
        "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll"]
LAIN = ["r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll",
        "r_el", "l_el", "head_pan", "head_tilt"]

# XM540-W270 / MX 2.0: satu hitungan profile_velocity = 0,229 rev/menit.
DEG_PER_SEC_PER_COUNT = 0.229 * 360.0 / 60.0

# robotis_controller menggabungkan sync_write_item per port per siklus (~8 ms).
# Dua item yang dikirim dalam satu siklus dulu bertabrakan, jadi diberi jarak.
JEDA_ANTAR_ITEM_S = 0.3


def main() -> int:
    urai = argparse.ArgumentParser(description=__doc__,
                                   formatter_class=argparse.RawDescriptionHelpFormatter)
    urai.add_argument("--set", type=int, default=0, metavar="N",
                      help="nilai profile_velocity (0 = tanpa batas, bawaan 0)")
    urai.add_argument("--accel", type=int, default=None, metavar="N",
                      help="nilai profile_acceleration (bawaan: setengah --set)")
    urai.add_argument("--kaki", action="store_true",
                      help="hanya 12 sendi kaki, jangan sentuh lengan dan kepala")
    arg = urai.parse_args()

    if arg.set < 0:
        print("profile_velocity tidak boleh negatif", file=sys.stderr)
        return 2
    accel = arg.accel if arg.accel is not None else arg.set // 2
    sendi = KAKI if arg.kaki else KAKI + LAIN

    rclpy.init()
    node = Node("clear_servo_speed_limit")
    pub = node.create_publisher(SyncWriteItem, "/robotis/sync_write_item", 10)

    def tunggu(detik: float) -> None:
        habis = time.monotonic() + detik
        while rclpy.ok() and time.monotonic() < habis:
            rclpy.spin_once(node, timeout_sec=0.01)

    # Penerbit anggota + jeda: penemuan DDS asinkron, terbit langsung hilang.
    tunggu(1.5)
    for nama, nilai in (("profile_velocity", arg.set), ("profile_acceleration", accel)):
        pub.publish(SyncWriteItem(item_name=nama, joint_name=sendi,
                                  value=[nilai] * len(sendi)))
        print(f"  {nama:<22} <- {nilai:<5} ({len(sendi)} sendi)")
        tunggu(JEDA_ANTAR_ITEM_S)
    tunggu(0.5)

    if arg.set == 0:
        print("\nBatas kecepatan DILEPAS (0 = tanpa batas, nilai pabrik).")
    else:
        print(f"\nBatas kecepatan terpasang: ~{arg.set * DEG_PER_SEC_PER_COUNT:.0f} deg/detik.")
        print("INGAT: gait akan ikut terkekang selama batas ini terpasang.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

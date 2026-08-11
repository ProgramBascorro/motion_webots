#!/usr/bin/env python3
"""Rekam goal vs present sendi saat manager boot, untuk tahu di mana pose hilang.

PEMAKAIAN
    Pane 1:  python3 scripts/diag_boot_pose.py
    Pane 2:  jalankan manager seperti biasa (./script.sh atau launcher)

Tunggu sampai skrip mencetak tabelnya (default 25 detik), lalu kirim hasilnya.

CARA BACANYA
    goal bergerak, present ikut      -> pose terbentuk; robot memang sudah/berhasil
                                        di posisi itu, masalahnya bukan di sini
    goal bergerak, present DIAM      -> perintah sampai tapi servo tidak menurut:
                                        torque mati, daya servo mati, atau bus mati
    goal TIDAK bergerak              -> action module tidak pernah menulis goal:
                                        masalah di sisi modul/urutan boot
    goal == present, keduanya diam   -> tidak ada yang memerintah apa pun

Kolom "gap akhir" adalah selisih goal-present di sampel terakhir: kalau besar,
servo tertinggal jauh dari yang diperintahkan (indikasi torque mati/beban).
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

CAPTURE_SECONDS = 25.0
RAD2DEG = 180.0 / math.pi

# Sendi yang paling informatif: kaki menentukan pose berdiri, kepala adalah
# keluhan tersendiri, bahu ikut karena walking module memakainya juga.
WATCH = [
    "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee",
    "r_ank_pitch", "l_ank_pitch", "r_sho_pitch", "l_sho_pitch",
    "head_pan", "head_tilt",
]


class BootPoseDiag(Node):
    def __init__(self):
        super().__init__("diag_boot_pose")
        self.goal = {}
        self.present = {}
        self.last_goal = {}
        self.last_present = {}
        self.goal_msgs = 0
        self.present_msgs = 0
        # Dibaca oleh loop di main(). Memanggil rclpy.shutdown() dari dalam
        # callback timer membuat spin() menggantung, jadi prosesnya tidak pernah
        # keluar sendiri dan harus di-Ctrl+C.
        self.done = False

        self.create_subscription(JointState, "/robotis/goal_joint_states", self.on_goal, 10)
        self.create_subscription(JointState, "/robotis/present_joint_states", self.on_present, 10)
        self.create_timer(CAPTURE_SECONDS, self.report)

        print(f"Merekam {CAPTURE_SECONDS:.0f} detik. Jalankan manager sekarang...", flush=True)

    @staticmethod
    def _collect(store, last, msg):
        for name, position in zip(msg.name, msg.position):
            if name not in WATCH:
                continue
            store.setdefault(name, []).append(position)
            last[name] = position

    def on_goal(self, msg):
        self.goal_msgs += 1
        self._collect(self.goal, self.last_goal, msg)

    def on_present(self, msg):
        self.present_msgs += 1
        self._collect(self.present, self.last_present, msg)

    def report(self):
        # Semua cetakan di-flush: tanpa ini tabelnya tertahan di buffer sampai
        # proses berakhir, dan kalau dijalankan lewat timeout/pipe hasilnya hilang.
        def out(text=""):
            print(text, flush=True)

        out()
        out(f"pesan diterima: goal={self.goal_msgs}  present={self.present_msgs}")
        if self.goal_msgs == 0 and self.present_msgs == 0:
            out("TIDAK ADA DATA. Manager tidak jalan, atau ROS_DOMAIN_ID/")
            out("ROS_LOCALHOST_ONLY skrip ini beda dengan manager.")
            self.done = True
            return

        out()
        header = "%-14s %12s %12s %12s   %s" % ("joint", "goal(deg)", "present(deg)", "gap akhir", "kesimpulan")
        out(header)
        out("-" * len(header))

        for name in WATCH:
            g = self.goal.get(name, [])
            p = self.present.get(name, [])
            if not g and not p:
                continue

            g_span = (max(g) - min(g)) * RAD2DEG if g else 0.0
            p_span = (max(p) - min(p)) * RAD2DEG if p else 0.0
            gap = ((self.last_goal.get(name, 0.0) - self.last_present.get(name, 0.0)) * RAD2DEG
                   if (name in self.last_goal and name in self.last_present) else float("nan"))

            # Ambang 1 derajat: di bawah itu hanya derau enkoder, bukan gerakan.
            if g_span < 1.0 and p_span < 1.0:
                verdict = "diam semua"
            elif g_span >= 1.0 and p_span < 1.0:
                verdict = "GOAL JALAN, SERVO DIAM  <<<"
            elif g_span < 1.0 and p_span >= 1.0:
                verdict = "servo bergerak tanpa goal"
            else:
                verdict = "ok, servo mengikuti"

            out("%-14s %12.2f %12.2f %12.2f   %s" % (name, g_span, p_span, gap, verdict))

        out()
        out("Kirim seluruh tabel ini apa adanya.")
        self.done = True


def main():
    rclpy.init()
    node = BootPoseDiag()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

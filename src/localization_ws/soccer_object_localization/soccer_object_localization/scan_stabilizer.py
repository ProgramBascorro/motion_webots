#!/usr/bin/env python3
"""
scan_stabilizer.py — IMU-Gated Scan Filter untuk AMCL
======================================================
Masalah inti: kamera OP3 berguncang saat robot berjalan.
  Guncangan → proyeksi ground plane berubah → point cloud terdistorsi
  → AMCL mendapat false yaw update → pose error

Solusi: gate scan menggunakan data IMU.
  - Robot STABIL (guncangan kecil)  → forward scan ke AMCL
  - Robot BERJALAN (guncangan besar) → tahan scan, kirim scan stabil terakhir
    sebagai fallback agar AMCL tidak stale terlalu lama

Mengapa ini efektif:
  - AMCL yang kuat adalah AMCL yang mendapat scan BERSIH, bukan scan banyak
  - Saat robot diam/stabil, scan sangat akurat → AMCL konvergen cepat
  - Saat robot berjalan, odom KF (akurasi 0.22%/m) cukup untuk tracking posisi
  - Efek: error posisi hanya terakumulasi dari odom saat berjalan,
    lalu dikoreksi AMCL saat robot berhenti/melambat

Topik:
  Subscribe: /field_scan          (LaserScan — bisa berguncang)
             /robotis_op3/imu     (atau /robotis/open_cr/imu untuk hw)
  Publish:   /field_scan_stable   (LaserScan — hanya saat kamera stabil)

Parameter (bisa di-override dari launch file):
  imu_topic            default: /robotis_op3/imu
  roll_threshold_deg   default: 4.0  (deg) — toleransi roll kamera
  pitch_threshold_deg  default: 5.0  (deg) — toleransi pitch (gait OP3 ~4°)
  max_hold_sec         default: 1.5  (s)   — max diam sebelum kirim fallback
  min_stable_count     default: 2          — jumlah scan stabil berturut sebelum forward
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, Imu


def quat_to_euler(q):
    """Quaternion → (roll, pitch) dalam radian."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    return roll, pitch


class ScanStabilizer(Node):

    def __init__(self):
        super().__init__('scan_stabilizer')

        # ── Parameter ────────────────────────────────────────────────
        self.declare_parameter('imu_topic',           '/robotis_op3/imu')
        self.declare_parameter('roll_threshold_deg',   4.0)
        self.declare_parameter('pitch_threshold_deg',  5.0)
        self.declare_parameter('max_hold_sec',         1.5)
        self.declare_parameter('min_stable_count',     2)

        imu_topic       = self.get_parameter('imu_topic').value
        self.roll_thr   = math.radians(self.get_parameter('roll_threshold_deg').value)
        self.pitch_thr  = math.radians(self.get_parameter('pitch_threshold_deg').value)
        self.max_hold   = self.get_parameter('max_hold_sec').value
        self.min_stable = self.get_parameter('min_stable_count').value

        # ── State ─────────────────────────────────────────────────────
        self.roll           = 0.0
        self.pitch          = 0.0
        self.has_imu        = False
        self.stable_count   = 0          # scan stabil berturut-turut
        self.last_good_scan = None       # scan bersih terakhir (untuk fallback)
        self.last_pub_time  = None       # waktu terakhir publish

        # statistik untuk log
        self._total_scans    = 0
        self._passed_scans   = 0
        self._fallback_count = 0

        # ── QoS ───────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Pub / Sub ─────────────────────────────────────────────────
        self.create_subscription(Imu,       imu_topic,    self._cb_imu,  qos)
        self.create_subscription(LaserScan, 'field_scan', self._cb_scan, qos)
        self.pub = self.create_publisher(LaserScan, 'field_scan_stable', qos)

        # Timer untuk log statistik tiap 10 detik
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info("━" * 56)
        self.get_logger().info("  Scan Stabilizer aktif")
        self.get_logger().info(f"  IMU        : {imu_topic}")
        self.get_logger().info(f"  Threshold  : |roll|<{math.degrees(self.roll_thr):.1f}°"
                               f"  |pitch|<{math.degrees(self.pitch_thr):.1f}°")
        self.get_logger().info(f"  max_hold   : {self.max_hold}s")
        self.get_logger().info(f"  min_stable : {self.min_stable} scan")
        self.get_logger().info("  Input  : /field_scan")
        self.get_logger().info("  Output : /field_scan_stable")
        self.get_logger().info("━" * 56)

    # ─────────────────────────────────────────────────────────────────
    def _cb_imu(self, msg: Imu):
        self.roll, self.pitch = quat_to_euler(msg.orientation)
        self.has_imu = True

    # ─────────────────────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        now = self.get_clock().now()
        self._total_scans += 1

        # Belum ada IMU → forward langsung agar AMCL tidak stuck di awal
        if not self.has_imu:
            self.pub.publish(msg)
            self._passed_scans += 1
            return

        roll_deg  = abs(math.degrees(self.roll))
        pitch_deg = abs(math.degrees(self.pitch))
        is_stable = (abs(self.roll) < self.roll_thr and
                     abs(self.pitch) < self.pitch_thr)

        if is_stable:
            self.stable_count += 1
        else:
            if self.stable_count >= self.min_stable:
                # Baru saja jadi tidak stabil — log transisi
                self.get_logger().debug(
                    f"[UNSTABLE] roll={roll_deg:.1f}° pitch={pitch_deg:.1f}°"
                    f" — scan ditahan"
                )
            self.stable_count = 0

        if self.stable_count >= self.min_stable:
            # ── Kirim scan bersih ──────────────────────────────────
            self.pub.publish(msg)
            self.last_good_scan = msg
            self.last_pub_time  = now
            self._passed_scans += 1

            if self.stable_count == self.min_stable:
                self.get_logger().debug(
                    f"[STABLE] roll={roll_deg:.1f}° pitch={pitch_deg:.1f}°"
                    f" — scan diforward"
                )
        else:
            # ── Robot tidak stabil — cek apakah perlu kirim fallback ──
            if self.last_pub_time is None:
                # Belum pernah ada scan stabil — beri peringatan sekali
                if not hasattr(self, '_warned'):
                    self.get_logger().warn(
                        "Belum ada scan stabil sejak start. "
                        f"IMU: roll={roll_deg:.1f}° pitch={pitch_deg:.1f}°. "
                        "Coba set robot diam beberapa detik saat awal."
                    )
                    self._warned = True
                return

            elapsed = (now - self.last_pub_time).nanoseconds / 1e9
            if elapsed >= self.max_hold and self.last_good_scan is not None:
                # ── Fallback: kirim scan stabil terakhir ──────────
                # Timestamp di-update ke sekarang agar AMCL tidak reject
                fallback = LaserScan()
                fallback.header           = msg.header   # timestamp baru
                fallback.angle_min        = self.last_good_scan.angle_min
                fallback.angle_max        = self.last_good_scan.angle_max
                fallback.angle_increment  = self.last_good_scan.angle_increment
                fallback.time_increment   = self.last_good_scan.time_increment
                fallback.scan_time        = self.last_good_scan.scan_time
                fallback.range_min        = self.last_good_scan.range_min
                fallback.range_max        = self.last_good_scan.range_max
                fallback.ranges           = list(self.last_good_scan.ranges)
                fallback.intensities      = list(self.last_good_scan.intensities)

                self.pub.publish(fallback)
                self.last_pub_time = now
                self._fallback_count += 1

                self.get_logger().debug(
                    f"[FALLBACK] {elapsed:.1f}s tanpa scan stabil"
                    f" — kirim scan lama (fallback #{self._fallback_count})"
                )

    # ─────────────────────────────────────────────────────────────────
    def _log_stats(self):
        if self._total_scans == 0:
            return
        pct = 100.0 * self._passed_scans / self._total_scans
        self.get_logger().info(
            f"[Stats] scan masuk={self._total_scans} "
            f"lolos={self._passed_scans} ({pct:.0f}%) "
            f"fallback={self._fallback_count} | "
            f"IMU roll={math.degrees(self.roll):.1f}° "
            f"pitch={math.degrees(self.pitch):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Global Relocalization Node untuk OP3
Melakukan relocalization otomatis TANPA perlu tahu posisi awal robot.

Cara kerja:
1. Saat node start → trigger global localization (partikel menyebar ke seluruh peta)
2. Monitor confidence AMCL dari /particle_cloud (seberapa rapat partikel mengumpul)
3. Kalau confidence rendah terlalu lama → trigger relocalization ulang
4. Publish status relocalization ke /relocalization_status

Triggers relocalization:
  - Saat node pertama kali start (otomatis)
  - Saat confidence AMCL turun di bawah threshold terlalu lama (robot tersesat)
  - Saat service /trigger_relocalization dipanggil (manual dari luar)
  - Saat topic /relocalize std_msgs/Bool diterima dengan data=True (dari tombol/gamepad)

Cara pakai di RoboCup:
  # Trigger manual via terminal:
  ros2 service call /trigger_relocalization std_srvs/srv/Empty
  
  # Atau via topic:
  ros2 topic pub /relocalize std_msgs/msg/Bool "data: true" --once
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseArray

from collections import deque


# ── State machine ────────────────────────────────────────────
class RelocState:
    IDLE         = "IDLE"           # Normal, confidence bagus
    GLOBALIZING  = "GLOBALIZING"    # Partikel sedang menyebar global
    CONVERGING   = "CONVERGING"     # Partikel mulai mengumpul
    LOST         = "LOST"           # Confidence buruk terlalu lama


class GlobalRelocalizationNode(Node):

    def __init__(self):
        super().__init__('global_relocalization_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('confidence_threshold',    0.6)
        self.declare_parameter('lost_timeout_sec',        15.0)
        self.declare_parameter('convergence_timeout_sec', 30.0)
        self.declare_parameter('check_interval_sec',      1.0)
        self.declare_parameter('auto_relocalize_on_start',True)
        self.declare_parameter('auto_relocalize_on_lost', True)
        # Seberapa rapat partikel harus mengumpul (meter)
        self.declare_parameter('cluster_radius_threshold', 0.5)
        # Minimum partikel yang harus ada dalam radius cluster
        self.declare_parameter('cluster_ratio_threshold',  0.6)

        self.conf_thresh      = self.get_parameter('confidence_threshold').value
        self.lost_timeout     = self.get_parameter('lost_timeout_sec').value
        self.conv_timeout     = self.get_parameter('convergence_timeout_sec').value
        self.check_interval   = self.get_parameter('check_interval_sec').value
        self.auto_on_start    = self.get_parameter('auto_relocalize_on_start').value
        self.auto_on_lost     = self.get_parameter('auto_relocalize_on_lost').value
        self.cluster_r        = self.get_parameter('cluster_radius_threshold').value
        self.cluster_ratio    = self.get_parameter('cluster_ratio_threshold').value

        # ── State ────────────────────────────────────────────
        self.state             = RelocState.IDLE
        self.confidence        = 0.0
        self.low_conf_start    = None   # kapan confidence mulai rendah
        self.globalizing_start = None   # kapan global localization dimulai
        self.conf_history      = deque(maxlen=10)
        self.last_particle_pos = None   # posisi estimasi dari partikel
        self.reloc_count       = 0      # berapa kali sudah relocalize

        # ── ROS interfaces ───────────────────────────────────

        # Service client ke AMCL untuk global localization
        self.global_loc_client = self.create_client(
            Empty, '/reinitialize_global_localization')

        # Publisher: trigger initial pose reset
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Publisher: status relocalization
        self.status_pub = self.create_publisher(String, '/relocalization_status', 10)

        # Subscriber: particle cloud → hitung confidence
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.create_subscription(
            PoseArray, '/particle_cloud',
            self.particle_cb, qos_be
        )

        # Subscriber: trigger manual dari luar (tombol/gamepad)
        self.create_subscription(
            Bool, '/relocalize', self.manual_trigger_cb, 10)

        # Service: trigger manual via service call
        self.create_service(
            Empty, '/trigger_relocalization', self.service_trigger_cb)

        # Timer: cek confidence secara periodik
        self.create_timer(self.check_interval, self.check_confidence)

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Global Relocalization Node started")
        self.get_logger().info(
            f"   conf_thresh={self.conf_thresh} | "
            f"lost_timeout={self.lost_timeout}s")
        self.get_logger().info(
            f"   auto_on_start={self.auto_on_start} | "
            f"auto_on_lost={self.auto_on_lost}")
        self.get_logger().info(
            f"   Manual trigger:")
        self.get_logger().info(
            f"     ros2 service call /trigger_relocalization std_srvs/srv/Empty")
        self.get_logger().info(
            f"     ros2 topic pub /relocalize std_msgs/msg/Bool 'data: true' --once")
        self.get_logger().info("=" * 60)

        # Auto relocalize saat pertama start
        if self.auto_on_start:
            # Delay 3 detik agar AMCL sudah aktif
            self.create_timer(3.0, self._initial_relocalize)

    # ── Initial relocalize (hanya sekali) ───────────────────
    def _initial_relocalize(self):
        self.get_logger().info("[Reloc] Auto relocalize saat startup...")
        self.trigger_global_localization()

    # ── Particle Cloud → Confidence ──────────────────────────
    def particle_cb(self, msg: PoseArray):
        """
        Hitung confidence dari sebaran partikel.
        
        Metode: cluster ratio
        - Ambil posisi rata-rata (estimasi posisi robot)
        - Hitung berapa % partikel dalam radius cluster_radius_threshold
        - Makin banyak partikel mengumpul → confidence makin tinggi
        """
        if not msg.poses:
            return

        # Posisi semua partikel
        positions = np.array([
            [p.position.x, p.position.y] for p in msg.poses
        ])

        # Estimasi posisi = median (lebih robust dari mean)
        est_pos = np.median(positions, axis=0)
        self.last_particle_pos = est_pos

        # Hitung jarak semua partikel dari estimasi
        distances = np.linalg.norm(positions - est_pos, axis=1)

        # Berapa persen partikel dalam radius threshold
        in_cluster = np.sum(distances < self.cluster_r)
        ratio = in_cluster / len(positions)

        self.confidence = float(ratio)
        self.conf_history.append(self.confidence)

    # ── Cek confidence secara periodik ──────────────────────
    def check_confidence(self):
        now = self.get_clock().now()

        # Rata-rata confidence dari history
        avg_conf = float(np.mean(self.conf_history)) if self.conf_history else 0.0

        # ── State machine ────────────────────────────────────
        if self.state == RelocState.IDLE:
            if avg_conf < self.conf_thresh:
                self.low_conf_start = now
                self.state = RelocState.LOST
                self.get_logger().warn(
                    f"[Reloc] Confidence turun: {avg_conf:.2f} < {self.conf_thresh}"
                    f" → state: LOST")

        elif self.state == RelocState.LOST:
            if avg_conf >= self.conf_thresh:
                # Pulih sendiri
                self.state = RelocState.IDLE
                self.low_conf_start = None
                self.get_logger().info(
                    f"[Reloc] Confidence pulih: {avg_conf:.2f} → state: IDLE")
            else:
                # Cek apakah sudah hilang terlalu lama
                elapsed = (now - self.low_conf_start).nanoseconds / 1e9
                if elapsed > self.lost_timeout and self.auto_on_lost:
                    self.get_logger().warn(
                        f"[Reloc] Lost selama {elapsed:.1f}s → trigger relocalization!")
                    self.trigger_global_localization()

        elif self.state == RelocState.GLOBALIZING:
            # Cek apakah sudah konvergen
            elapsed = (now - self.globalizing_start).nanoseconds / 1e9

            if avg_conf >= self.conf_thresh:
                self.state = RelocState.CONVERGING
                self.get_logger().info(
                    f"[Reloc] Partikel mulai konvergen: {avg_conf:.2f}"
                    f" ({elapsed:.1f}s) → state: CONVERGING")

            elif elapsed > self.conv_timeout:
                # Terlalu lama tidak konvergen → coba lagi
                self.get_logger().warn(
                    f"[Reloc] Timeout {elapsed:.1f}s, belum konvergen "
                    f"(conf={avg_conf:.2f}) → retry!")
                self.trigger_global_localization()

        elif self.state == RelocState.CONVERGING:
            if avg_conf >= self.conf_thresh:
                self.state = RelocState.IDLE
                pos_str = ""
                if self.last_particle_pos is not None:
                    pos_str = (f" | estimasi pos=("
                               f"{self.last_particle_pos[0]:.2f},"
                               f"{self.last_particle_pos[1]:.2f})")
                self.get_logger().info(
                    f"✅ [Reloc] BERHASIL konvergen! "
                    f"conf={avg_conf:.2f}{pos_str}")
            elif avg_conf < self.conf_thresh * 0.5:
                # Confidence turun drastis setelah sempat naik → lost lagi
                self.state = RelocState.LOST
                self.low_conf_start = now
                self.get_logger().warn(
                    f"[Reloc] Confidence turun lagi: {avg_conf:.2f} → LOST")

        # Publish status
        self._pub_status(avg_conf)

    # ── Trigger Global Localization ──────────────────────────
    def trigger_global_localization(self):
        """
        Memanggil service AMCL /reinitialize_global_localization.
        AMCL akan menyebar partikel ke seluruh peta.
        """
        self.reloc_count += 1
        self.state             = RelocState.GLOBALIZING
        self.globalizing_start = self.get_clock().now()
        self.conf_history.clear()

        self.get_logger().info(
            f"[Reloc] Trigger global localization #{self.reloc_count}..."
            f" Partikel menyebar ke seluruh peta.")

        if not self.global_loc_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "[Reloc] Service /reinitialize_global_localization tidak tersedia! "
                "Pastikan AMCL sudah aktif.")
            # Fallback: publish initialpose dengan covariance besar
            # (simulasi global localization manual)
            self._fallback_global_init()
            return

        req = Empty.Request()
        future = self.global_loc_client.call_async(req)
        future.add_done_callback(self._on_global_loc_done)

    def _on_global_loc_done(self, future):
        try:
            future.result()
            self.get_logger().info(
                "[Reloc] Global localization triggered ✅ "
                "Menunggu partikel konvergen...")
        except Exception as e:
            self.get_logger().error(f"[Reloc] Service call gagal: {e}")
            self._fallback_global_init()

    def _fallback_global_init(self):
        """
        Fallback: publish /initialpose dengan covariance sangat besar.
        Efeknya mirip global localization — partikel menyebar luas.
        """
        self.get_logger().info(
            "[Reloc] Fallback: publish initialpose dengan covariance besar")

        pose = PoseWithCovarianceStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        # Posisi di tengah peta (0, 0)
        pose.pose.pose.position.x    = 0.0
        pose.pose.pose.position.y    = 0.0
        pose.pose.pose.orientation.w = 1.0

        # Covariance sangat besar → partikel menyebar ke seluruh peta
        # Diagonal: [xx, yy, zz, roll, pitch, yaw]
        pose.pose.covariance[0]  = 9.0    # std 3m di X
        pose.pose.covariance[7]  = 9.0    # std 3m di Y
        pose.pose.covariance[35] = 3.14   # std ~180° di yaw

        self.init_pose_pub.publish(pose)

    # ── Manual Triggers ──────────────────────────────────────
    def manual_trigger_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("[Reloc] Manual trigger dari topic /relocalize")
            self.trigger_global_localization()

    def service_trigger_cb(self, request, response):
        self.get_logger().info(
            "[Reloc] Manual trigger dari service /trigger_relocalization")
        self.trigger_global_localization()
        return response

    # ── Status Publisher ─────────────────────────────────────
    def _pub_status(self, avg_conf):
        pos_str = "unknown"
        if self.last_particle_pos is not None:
            pos_str = (f"({self.last_particle_pos[0]:.2f},"
                       f"{self.last_particle_pos[1]:.2f})")

        msg = String()
        msg.data = (
            f"state={self.state} | "
            f"conf={avg_conf:.2f} | "
            f"est_pos={pos_str} | "
            f"reloc_count={self.reloc_count}"
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GlobalRelocalizationNode())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
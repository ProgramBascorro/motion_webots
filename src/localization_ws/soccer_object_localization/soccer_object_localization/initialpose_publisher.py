#!/usr/bin/env python3
"""
initialpose_publisher.py — Jembatan TF map→odom saat startup
=============================================================
Masalah: saat launch, AMCL butuh scan pertama sebelum publish TF map→odom.
Selama TF map→odom belum ada, RViz error dan 2D Pose Estimate tidak bisa.

Program LAMA tidak masalah karena gt_odom_to_amcl langsung publish
/initialpose ke AMCL di detik pertama → AMCL langsung publish TF map→odom.

Node ini menggantikan fungsi tersebut tanpa butuh Webots ground truth:

  1. Saat start: publish static TF map→odom = identity (0,0,0)
     → RViz langsung happy, TF chain lengkap
     → 2D Pose Estimate bisa langsung digunakan

  2. Publish /initialpose ke AMCL (pose awal = 0,0,0 atau dari parameter)
     → AMCL langsung aktif dan mulai publish TF map→odom sendiri
     → TF dari AMCL menggantikan static TF ini secara otomatis

  3. Setelah AMCL aktif (terdeteksi dari /amcl_pose), node ini selesai
     (opsional — bisa dibiarkan terus publish jika AMCL belum aktif)

Parameter:
  initial_x    (default: 0.0) — pose X awal dalam frame map
  initial_y    (default: 0.0) — pose Y awal dalam frame map
  initial_yaw  (default: 0.0) — yaw awal dalam radian
  map_frame    (default: 'map')
  odom_frame   (default: 'odom')
  amcl_timeout (default: 30.0) — detik tunggu AMCL sebelum terus publish
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initialpose_publisher')

        # ── Parameter ────────────────────────────────────────────────
        self.declare_parameter('initial_x',    0.0)
        self.declare_parameter('initial_y',    0.0)
        self.declare_parameter('initial_yaw',  0.0)
        self.declare_parameter('map_frame',    'map')
        self.declare_parameter('odom_frame',   'odom')
        self.declare_parameter('amcl_timeout', 30.0)

        self.ix       = self.get_parameter('initial_x').value
        self.iy       = self.get_parameter('initial_y').value
        self.iyaw     = self.get_parameter('initial_yaw').value
        self.map_fr   = self.get_parameter('map_frame').value
        self.odom_fr  = self.get_parameter('odom_frame').value
        self.timeout  = self.get_parameter('amcl_timeout').value

        # ── State ─────────────────────────────────────────────────────
        self.amcl_active   = False
        self.pose_sent     = False
        self._start_time   = self.get_clock().now()

        # ── TF Broadcaster ────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Publisher: /initialpose ────────────────────────────────────
        # Latched-like: gunakan TRANSIENT_LOCAL agar AMCL yang baru start
        # tetap menerima pesan ini
        qos_latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos_latched
        )

        # ── Subscriber: /amcl_pose (deteksi AMCL aktif) ───────────────
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._cb_amcl_pose,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        )

        # ── Timer: publish TF + initialpose ───────────────────────────
        # 10Hz cukup untuk placeholder TF
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info("━" * 50)
        self.get_logger().info("  Initial Pose Publisher aktif")
        self.get_logger().info(f"  Pose awal: x={self.ix} y={self.iy} yaw={self.iyaw:.3f}rad")
        self.get_logger().info(f"  TF: {self.map_fr} → {self.odom_fr}")
        self.get_logger().info(f"  Timeout AMCL: {self.timeout}s")
        self.get_logger().info("━" * 50)

    def _cb_amcl_pose(self, msg):
        """Saat AMCL mulai publish pose, tandai sebagai aktif."""
        if not self.amcl_active:
            self.amcl_active = True
            self.get_logger().info(
                "✅ AMCL aktif dan publish pose. "
                "TF map→odom sekarang dikelola AMCL."
            )

    def _timer_cb(self):
        now = self.get_clock().now()

        # Selama AMCL belum aktif, publish static TF map→odom = identity
        # Ini membuat TF chain lengkap dan RViz tidak error
        if not self.amcl_active:
            self._publish_static_tf(now)

        # Publish /initialpose ke AMCL (sekali, latch)
        # Dikirim berulang selama AMCL belum aktif agar tidak terlewat
        if not self.amcl_active and not self.pose_sent:
            elapsed = (now - self._start_time).nanoseconds / 1e9
            # Tunda 1 detik agar AMCL sudah benar-benar berjalan
            if elapsed >= 1.0:
                self._publish_initialpose(now)
                self.pose_sent = True

        # Jika sudah terlalu lama (timeout) tapi AMCL masih belum aktif,
        # kirim ulang /initialpose (mungkin AMCL start terlambat)
        if not self.amcl_active and self.pose_sent:
            elapsed = (now - self._start_time).nanoseconds / 1e9
            if elapsed > self.timeout:
                self.get_logger().warn(
                    f"AMCL belum aktif setelah {self.timeout:.0f}s — "
                    "kirim ulang /initialpose"
                )
                self._publish_initialpose(now)
                self._start_time = now   # reset timer

    def _publish_static_tf(self, now):
        """Publish TF map→odom = identity sebagai placeholder."""
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = self.map_fr
        t.child_frame_id  = self.odom_fr

        # Offset sesuai initial pose
        yaw = self.iyaw
        t.transform.translation.x = self.ix
        t.transform.translation.y = self.iy
        t.transform.translation.z = 0.0
        t.transform.rotation.x    = 0.0
        t.transform.rotation.y    = 0.0
        t.transform.rotation.z    = math.sin(yaw / 2.0)
        t.transform.rotation.w    = math.cos(yaw / 2.0)

        self.tf_broadcaster.sendTransform(t)

    def _publish_initialpose(self, now):
        """Publish /initialpose agar AMCL segera aktif dan publish TF."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = now.to_msg()
        msg.header.frame_id = self.map_fr

        yaw = self.iyaw
        msg.pose.pose.position.x    = self.ix
        msg.pose.pose.position.y    = self.iy
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Covariance diagonal: posisi 0.25m², yaw 0.07rad² (≈4°)
        # Cukup lebar agar partikel AMCL tersebar di sekitar pose awal
        cov = [0.0] * 36
        cov[0]  = 0.25   # x
        cov[7]  = 0.25   # y
        cov[35] = 0.07   # yaw
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)
        self.get_logger().info(
            f"📍 /initialpose dikirim → "
            f"x={self.ix:.2f} y={self.iy:.2f} yaw={math.degrees(self.iyaw):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
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
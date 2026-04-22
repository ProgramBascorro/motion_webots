#!/usr/bin/env python3
"""
odom_to_amcl_node.py v4

Fix dari v3:
  1. Loop publish: node publish /initialpose → node terima lagi → origin di-reset
     Fix: bedakan dari covariance — RViz pakai 0.25, kita pakai 0.05
          kalau cov[0] <= 0.15 = dari node sendiri → abaikan

  2. Joint odom steps=0 → delta selalu (0,0) → pose tidak bergerak
     Fix: tambah fallback mode — kalau /odom tidak bergerak selama timeout,
          tampilkan warning agar user tahu perlu diagnosa joint odom

Alur:
  1. Startup → tunggu set manual dari user
  2. User klik 2D Pose Estimate → node tangkap /initialpose (cov besar = dari RViz)
  3. Catat map_origin dan odom_origin
  4. Tiap 300ms: hitung pose = map_origin + delta_odom → publish
  5. Kalau odom tidak bergerak > 10 detik → warning di log
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


def quat_to_yaw(q):
    return 2.0 * math.atan2(q.z, q.w)


def yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class OdomToAmclNode(Node):

    def __init__(self):
        super().__init__('odom_to_amcl_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('trust_level',   'high_trust')
        self.declare_parameter('cov_xy',         -1.0)
        self.declare_parameter('cov_yaw',        -1.0)
        self.declare_parameter('publish_rate',    0.3)
        self.declare_parameter('always_publish',  True)
        # Ambang covariance untuk bedakan RViz set vs node publish
        # RViz default = 0.25, kita pakai 0.05 → threshold = 0.15
        self.declare_parameter('rviz_cov_threshold', 0.15)

        trust   = self.get_parameter('trust_level').value
        cov_xy  = self.get_parameter('cov_xy').value
        cov_yaw = self.get_parameter('cov_yaw').value

        presets = {
            'gt_like':    (0.01, 0.01),
            'high_trust': (0.05, 0.03),
            'medium':     (0.2,  0.08),
            'low_trust':  (0.6,  0.2),
        }
        if cov_xy < 0 or cov_yaw < 0:
            self.cov_xy, self.cov_yaw = presets.get(trust, presets['high_trust'])
        else:
            self.cov_xy  = cov_xy
            self.cov_yaw = cov_yaw

        self.pub_rate      = self.get_parameter('publish_rate').value
        self.always_pub    = self.get_parameter('always_publish').value
        self.rviz_cov_thr  = self.get_parameter('rviz_cov_threshold').value

        # ── State ────────────────────────────────────────────
        self.map_origin_x    = None
        self.map_origin_y    = None
        self.map_origin_yaw  = None
        self.odom_origin_x   = None
        self.odom_origin_y   = None
        self.odom_origin_yaw = None
        self.initialized     = False

        self.current_odom    = None
        self.last_odom_x     = None
        self.last_odom_y     = None
        self.last_move_time  = None   # untuk deteksi odom stuck
        self.pub_count       = 0
        self.warn_count      = 0

        # ── ROS ──────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )

        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_be)

        # Subscribe /initialpose untuk tangkap set dari RViz
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.user_set_cb,
            10
        )

        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_timer(self.pub_rate, self.publish_pose)

        # Timer diagnosa: cek apakah odom bergerak
        self.create_timer(5.0, self.diagnose_odom)

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Odom to AMCL Node v4 started")
        self.get_logger().info(
            f"   trust={trust} | cov_xy={self.cov_xy} | cov_yaw={self.cov_yaw}")
        self.get_logger().info(
            f"   rviz_cov_threshold={self.rviz_cov_thr} "
            f"(RViz=0.25 > thr, Node={self.cov_xy} < thr)")
        self.get_logger().info(
            "   ⏳ Menunggu 2D Pose Estimate dari RViz...")
        self.get_logger().info("=" * 60)

    # ── Callbacks ────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg

        # Track apakah odom bergerak
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_odom_x is None:
            self.last_odom_x = x
            self.last_odom_y = y
            self.last_move_time = self.get_clock().now()
        else:
            dist = math.sqrt((x - self.last_odom_x)**2 + (y - self.last_odom_y)**2)
            if dist > 0.001:   # bergerak > 1mm — lebih sensitif untuk langkah kecil
                self.last_odom_x    = x
                self.last_odom_y    = y
                self.last_move_time = self.get_clock().now()

    def user_set_cb(self, msg: PoseWithCovarianceStamped):
        """
        Tangkap /initialpose dari RViz.
        Bedakan dari publish node sendiri via covariance.
        RViz default cov[0] = 0.25, node kita cov[0] = cov_xy (0.05).
        """
        cov_x = msg.pose.covariance[0]

        # Kalau covariance KECIL → ini dari node kita sendiri → abaikan
        if cov_x <= self.rviz_cov_thr:
            return

        # Covariance BESAR → dari RViz user → catat sebagai origin baru
        if self.current_odom is None:
            self.get_logger().warn(
                "[OdomToAMCL] /initialpose diterima tapi /odom belum ada. "
                "Pastikan joint_odom_node berjalan.")
            return

        map_x   = msg.pose.pose.position.x
        map_y   = msg.pose.pose.position.y
        map_yaw = quat_to_yaw(msg.pose.pose.orientation)

        odom = self.current_odom
        self.map_origin_x    = map_x
        self.map_origin_y    = map_y
        self.map_origin_yaw  = map_yaw
        self.odom_origin_x   = odom.pose.pose.position.x
        self.odom_origin_y   = odom.pose.pose.position.y
        self.odom_origin_yaw = quat_to_yaw(odom.pose.pose.orientation)
        self.initialized     = True

        self.get_logger().info(
            f"✅ [OdomToAMCL] Pose di-set! "
            f"map=({map_x:.2f}, {map_y:.2f}) yaw={math.degrees(map_yaw):.1f}° | "
            f"cov_rviz={cov_x:.3f}")
        self.get_logger().info(
            f"   odom_origin=({self.odom_origin_x:.3f}, "
            f"{self.odom_origin_y:.3f})")
        self.get_logger().info(
            "   🚀 Mulai tracking pose dari titik ini...")

    # ── Publish pose ─────────────────────────────────────────

    def publish_pose(self):
        if not self.initialized or self.current_odom is None:
            return

        odom = self.current_odom
        odom_x   = odom.pose.pose.position.x
        odom_y   = odom.pose.pose.position.y
        odom_yaw = quat_to_yaw(odom.pose.pose.orientation)

        # Delta dari odom origin
        dx_odom    = odom_x   - self.odom_origin_x
        dy_odom    = odom_y   - self.odom_origin_y
        d_yaw      = odom_yaw - self.odom_origin_yaw

        # Rotasikan delta ke frame map
        rot = self.map_origin_yaw - self.odom_origin_yaw
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)
        dx_map = dx_odom * cos_r - dy_odom * sin_r
        dy_map = dx_odom * sin_r + dy_odom * cos_r

        final_x   = self.map_origin_x   + dx_map
        final_y   = self.map_origin_y   + dy_map
        final_yaw = self.map_origin_yaw + d_yaw

        pose = PoseWithCovarianceStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x    = final_x
        pose.pose.pose.position.y    = final_y
        pose.pose.pose.position.z    = 0.0
        pose.pose.pose.orientation   = yaw_to_quat(final_yaw)

        c = pose.pose.covariance
        c[0]  = self.cov_xy
        c[7]  = self.cov_xy
        c[14] = 0.001
        c[21] = 0.001
        c[28] = 0.001
        c[35] = self.cov_yaw

        self.init_pose_pub.publish(pose)
        self.pub_count += 1

        if self.pub_count % 20 == 1:
            dist_moved = math.sqrt(dx_odom**2 + dy_odom**2)
            self.get_logger().info(
                f"[OdomToAMCL] #{self.pub_count}: "
                f"map=({final_x:.3f}, {final_y:.3f}) "
                f"yaw={math.degrees(final_yaw):.1f}° | "
                f"jarak_dari_origin={dist_moved:.3f}m")

    # ── Diagnosa odom stuck ──────────────────────────────────

    def diagnose_odom(self):
        if not self.initialized:
            return
        if self.last_move_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_move_time).nanoseconds / 1e9

        if elapsed > 10.0:
            self.warn_count += 1
            if self.warn_count % 3 == 1:
                self.get_logger().warn(
                    f"⚠️  [OdomToAMCL] /odom tidak bergerak selama {elapsed:.0f}s! "
                    f"Kemungkinan joint_odom steps=0. "
                    f"Cek: ros2 topic echo /rosout | grep JointOdom")
        else:
            self.warn_count = 0


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(OdomToAmclNode())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
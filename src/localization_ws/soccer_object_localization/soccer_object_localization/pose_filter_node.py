#!/usr/bin/env python3
"""
pose_filter_node.py

Layer tambahan di atas AMCL untuk menangani masalah lapangan simetris.

Masalah: AMCL sesekali "tertipu" scan field line yang simetris
         → /amcl_pose loncat jauh ke posisi yang salah total

Solusi: Filter /amcl_pose sebelum digunakan:
  1. Outlier rejection — tolak kalau delta dari odom > max_jump
  2. Temporal smoothing — kalau valid, haluskan dengan EMA
  3. Odom fallback — kalau AMCL tidak update lama, pakai odom murni
  4. Confidence tracking — track seberapa yakin kita sama AMCL

Input:  /amcl_pose (PoseWithCovarianceStamped dari AMCL)
        /odom      (Odometry dari joint_odom_node)
Output: /robot_pose (PoseWithCovarianceStamped — output final)
        TF map→base_link (opsional, untuk visualisasi langsung)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from collections import deque


def quat_to_yaw(q) -> float:
    return 2.0 * math.atan2(q.z, q.w)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


def angle_diff(a: float, b: float) -> float:
    """Selisih sudut terkecil (−π .. +π)."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class PoseFilterNode(Node):

    def __init__(self):
        super().__init__('pose_filter_node')

        # ── Parameters ───────────────────────────────────────
        # Jarak maksimum yang diizinkan antara AMCL pose dan odom pose
        # Lebih besar = lebih toleran tapi lebih mudah tertipu
        # Lebih kecil = lebih ketat tapi mungkin reject pose valid
        self.declare_parameter('max_jump_xy',   0.3)   # meter (dari 0.5)
        self.declare_parameter('max_jump_yaw',  0.5)   # radian (~30°, dari 0.8)
        # Kecepatan fisik maksimum robot (m/s)
        # OP3 walking max ~0.15 m/s
        # Kalau delta_pose / delta_time > ini → pasti outlier
        self.declare_parameter('max_velocity',  0.2)   # m/s

        # EMA alpha untuk smoothing: 0.0=tidak update, 1.0=no smoothing
        # 0.3 = pose baru hanya 30% berpengaruh, 70% dari pose lama
        self.declare_parameter('ema_alpha_xy',  0.3)
        self.declare_parameter('ema_alpha_yaw', 0.4)

        # Timeout odom fallback: kalau AMCL tidak update N detik → pakai odom
        self.declare_parameter('amcl_timeout',  3.0)   # detik

        # Publish filtered pose ke TF map→base_link juga?
        self.declare_parameter('publish_tf',    True)

        # Minimum covariance AMCL yang diterima
        # Pose dengan covariance sangat besar = AMCL tidak yakin → tolak
        self.declare_parameter('max_amcl_cov',  2.0)

        self.max_jump_xy   = self.get_parameter('max_jump_xy').value
        self.max_jump_yaw  = self.get_parameter('max_jump_yaw').value
        self.max_velocity  = self.get_parameter('max_velocity').value
        self.ema_xy        = self.get_parameter('ema_alpha_xy').value
        self.ema_yaw       = self.get_parameter('ema_alpha_yaw').value
        self.amcl_timeout  = self.get_parameter('amcl_timeout').value
        self.publish_tf    = self.get_parameter('publish_tf').value
        self.max_amcl_cov  = self.get_parameter('max_amcl_cov').value

        # ── State ────────────────────────────────────────────
        # Pose filter saat ini (output)
        self.filtered_x   = None
        self.filtered_y   = None
        self.filtered_yaw = None
        self.initialized  = False

        # Odom tracking (untuk outlier rejection dan fallback)
        self.odom_x   = 0.0
        self.odom_y   = 0.0
        self.odom_yaw = 0.0

        # Odom origin saat AMCL terakhir diterima
        self.odom_at_last_amcl_x   = None
        self.odom_at_last_amcl_y   = None
        self.odom_at_last_amcl_yaw = None

        # Stats
        self.max_velocity     = 0.0  # diisi dari parameter
        self.last_amcl_stamp  = None  # timestamp AMCL terakhir
        self.amcl_received    = 0
        self.amcl_accepted    = 0
        self.amcl_rejected    = 0
        self.last_amcl_time   = None
        self.using_odom_fb    = False  # sedang dalam odom fallback mode

        # History untuk log
        self.reject_reasons = deque(maxlen=10)

        # ── ROS ──────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )

        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_be)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot_pose', 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Timer: publish pose dan cek fallback
        self.create_timer(0.1, self.publish_pose)   # 10Hz
        self.create_timer(1.0, self.log_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Pose Filter Node started")
        self.get_logger().info(
            f"   max_jump: {self.max_jump_xy}m / {math.degrees(self.max_jump_yaw):.0f}° | "
            f"max_velocity: {self.max_velocity}m/s")
        self.get_logger().info(
            f"   ema_alpha: xy={self.ema_xy} yaw={self.ema_yaw}")
        self.get_logger().info(
            f"   amcl_timeout: {self.amcl_timeout}s → fallback ke odom")
        self.get_logger().info("=" * 60)

    # ── Odom callback ────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        prev_x   = self.odom_x
        prev_y   = self.odom_y
        prev_yaw = self.odom_yaw

        self.odom_x   = msg.pose.pose.position.x
        self.odom_y   = msg.pose.pose.position.y
        self.odom_yaw = quat_to_yaw(msg.pose.pose.orientation)

        # Kalau belum initialized, set pose awal dari odom
        if not self.initialized:
            # Tunggu sampai ada AMCL pose dulu
            return

        # Propagate filtered pose menggunakan delta odom
        # Ini membuat filtered pose bergerak smooth ikut odom
        # bahkan saat tidak ada update AMCL
        if self.filtered_x is not None:
            dx_odom = self.odom_x   - prev_x
            dy_odom = self.odom_y   - prev_y
            dyaw    = angle_diff(self.odom_yaw, prev_yaw)

            self.filtered_x   += dx_odom
            self.filtered_y   += dy_odom
            self.filtered_yaw  = self.filtered_yaw + dyaw

    # ── AMCL callback ────────────────────────────────────────

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.amcl_received += 1
        now = self.get_clock().now()

        amcl_x   = msg.pose.pose.position.x
        amcl_y   = msg.pose.pose.position.y
        amcl_yaw = quat_to_yaw(msg.pose.pose.orientation)
        amcl_cov = msg.pose.covariance[0]  # var_x sebagai proxy confidence

        # ── Cek 1: Covariance terlalu besar → AMCL tidak yakin ──
        if amcl_cov > self.max_amcl_cov:
            self.amcl_rejected += 1
            self.reject_reasons.append(
                f"cov={amcl_cov:.2f}>{self.max_amcl_cov}")
            return

        # ── Inisialisasi pertama ─────────────────────────────
        if not self.initialized:
            self.filtered_x   = amcl_x
            self.filtered_y   = amcl_y
            self.filtered_yaw = amcl_yaw
            self.odom_at_last_amcl_x   = self.odom_x
            self.odom_at_last_amcl_y   = self.odom_y
            self.odom_at_last_amcl_yaw = self.odom_yaw
            self.initialized  = True
            self.last_amcl_time = now
            self.amcl_accepted += 1
            self.get_logger().info(
                f"✅ [PoseFilter] Initialized dari AMCL: "
                f"({amcl_x:.2f}, {amcl_y:.2f}) yaw={math.degrees(amcl_yaw):.1f}°")
            return

        # ── Cek 2: Outlier rejection ─────────────────────────
        # Hitung di mana odom memperkirakan robot berada sekarang
        # berdasarkan delta dari last AMCL
        dx_since  = self.odom_x - self.odom_at_last_amcl_x
        dy_since  = self.odom_y - self.odom_at_last_amcl_y
        dyaw_since = angle_diff(self.odom_yaw, self.odom_at_last_amcl_yaw)

        # Prediksi posisi berdasarkan filtered_pose + delta_odom
        pred_x   = self.filtered_x   + dx_since
        pred_y   = self.filtered_y   + dy_since
        pred_yaw = self.filtered_yaw + dyaw_since

        # Delta antara prediksi odom dan AMCL
        delta_xy  = math.sqrt((amcl_x - pred_x)**2 + (amcl_y - pred_y)**2)
        delta_yaw = abs(angle_diff(amcl_yaw, pred_yaw))

        if delta_xy > self.max_jump_xy or delta_yaw > self.max_jump_yaw:
            self.amcl_rejected += 1
            reason = (f"jump: Δxy={delta_xy:.2f}m "
                      f"Δyaw={math.degrees(delta_yaw):.0f}°")
            self.reject_reasons.append(reason)
            self.get_logger().warn(
                f"⚠️  [PoseFilter] AMCL DITOLAK — {reason} "
                f"| pred=({pred_x:.2f},{pred_y:.2f}) "
                f"amcl=({amcl_x:.2f},{amcl_y:.2f})")
            return

        # ── Cek 3: Physics-based velocity check ─────────────
        # Kalau pose berubah lebih cepat dari kecepatan max robot → outlier
        if self.last_amcl_stamp is not None:
            try:
                stamp_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                dt = stamp_now - self.last_amcl_stamp
                if dt > 0.05:  # minimal 50ms antar update
                    velocity = delta_xy / dt
                    if velocity > self.max_velocity:
                        self.amcl_rejected += 1
                        reason = (f"velocity: {velocity:.2f}m/s > "
                                  f"max {self.max_velocity}m/s (dt={dt:.2f}s)")
                        self.reject_reasons.append(reason)
                        self.get_logger().warn(
                            f"⚠️  [PoseFilter] AMCL DITOLAK — {reason}")
                        return
            except Exception:
                pass

        # ── Update diterima: EMA smoothing ──────────────────
        # EMA: new_val = alpha * amcl + (1-alpha) * current
        # Alpha kecil = lebih smooth, alpha besar = lebih responsif
        self.filtered_x   = (self.ema_xy  * amcl_x
                            + (1 - self.ema_xy)  * self.filtered_x)
        self.filtered_y   = (self.ema_xy  * amcl_y
                            + (1 - self.ema_xy)  * self.filtered_y)

        # Angle EMA — harus handle wrap-around
        dyaw_correction = angle_diff(amcl_yaw, self.filtered_yaw)
        self.filtered_yaw = self.filtered_yaw + self.ema_yaw * dyaw_correction

        # Update reference untuk delta odom berikutnya
        self.last_amcl_stamp = (msg.header.stamp.sec
                                + msg.header.stamp.nanosec * 1e-9)
        self.odom_at_last_amcl_x   = self.odom_x
        self.odom_at_last_amcl_y   = self.odom_y
        self.odom_at_last_amcl_yaw = self.odom_yaw
        self.last_amcl_time        = now
        self.amcl_accepted        += 1
        self.using_odom_fb         = False

        self.get_logger().debug(
            f"[PoseFilter] AMCL diterima: "
            f"({amcl_x:.3f},{amcl_y:.3f}) → "
            f"filtered=({self.filtered_x:.3f},{self.filtered_y:.3f}) "
            f"Δxy={delta_xy:.3f}m")

    # ── Publish filtered pose ────────────────────────────────

    def publish_pose(self):
        if not self.initialized or self.filtered_x is None:
            return

        # Cek odom fallback
        if self.last_amcl_time is not None:
            elapsed = (self.get_clock().now() - self.last_amcl_time).nanoseconds / 1e9
            if elapsed > self.amcl_timeout and not self.using_odom_fb:
                self.using_odom_fb = True
                self.get_logger().warn(
                    f"⚠️  [PoseFilter] AMCL tidak update {elapsed:.0f}s → "
                    f"odom fallback mode")

        now = self.get_clock().now().to_msg()

        # Publish /robot_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = now
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x    = self.filtered_x
        msg.pose.pose.position.y    = self.filtered_y
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation   = yaw_to_quat(self.filtered_yaw)

        # Covariance kecil saat yakin (tidak ada fallback)
        cov_val = 0.05 if not self.using_odom_fb else 0.3
        c = msg.pose.covariance
        c[0]  = cov_val
        c[7]  = cov_val
        c[14] = 0.001
        c[21] = 0.001
        c[28] = 0.001
        c[35] = cov_val * 0.5

        self.pose_pub.publish(msg)

        # Publish TF map→base_link
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = now
            tf.header.frame_id = 'map'
            tf.child_frame_id  = 'base_link_filtered'
            tf.transform.translation.x = self.filtered_x
            tf.transform.translation.y = self.filtered_y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = yaw_to_quat(self.filtered_yaw)
            self.tf_broadcaster.sendTransform(tf)

    # ── Log status ───────────────────────────────────────────

    def log_status(self):
        if not self.initialized:
            return

        accept_rate = (self.amcl_accepted / max(1, self.amcl_received)) * 100
        mode = "ODOM_FB" if self.using_odom_fb else "AMCL+EMA"

        self.get_logger().info(
            f"[PoseFilter] mode={mode} | "
            f"pose=({self.filtered_x:.3f},{self.filtered_y:.3f}) "
            f"yaw={math.degrees(self.filtered_yaw):.1f}° | "
            f"accept={self.amcl_accepted}/{self.amcl_received} "
            f"({accept_rate:.0f}%) | "
            f"reject={self.amcl_rejected}")

        if self.reject_reasons:
            recent = list(self.reject_reasons)[-3:]
            self.get_logger().info(
                f"   Recent rejects: {recent}")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(PoseFilterNode())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
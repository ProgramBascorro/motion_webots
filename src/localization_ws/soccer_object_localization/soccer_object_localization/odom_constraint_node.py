#!/usr/bin/env python3
"""
odom_constraint_node.py

Fungsi utama:
  1. RELAY /initialpose dari RViz (RELIABLE) ke AMCL (BEST_EFFORT)
     Tanpa ini, RViz tidak bisa set 2D Pose Estimate ke AMCL!
     (RViz publish RELIABLE, AMCL subscribe BEST_EFFORT → tidak kompatibel)

  2. Setelah user set pose, publish /initialpose periodik (tiap N detik)
     dengan covariance kecil untuk "menjangkar" partikel ke posisi odom
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


def quat_to_yaw(q) -> float:
    return 2.0 * math.atan2(q.z, q.w)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


# QoS yang match dengan AMCL subscriber
QOS_AMCL = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE
)

# QoS untuk terima dari RViz (publish RELIABLE)
QOS_RVIZ = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE
)

# QoS untuk odom
QOS_ODOM = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


class OdomConstraintNode(Node):

    def __init__(self):
        super().__init__('odom_constraint_node')

        self.declare_parameter('constraint_interval', 3.0)
        self.declare_parameter('cov_xy',   0.04)
        self.declare_parameter('cov_yaw',  0.02)
        self.declare_parameter('min_move', 0.02)

        self.interval  = self.get_parameter('constraint_interval').value
        self.cov_xy    = self.get_parameter('cov_xy').value
        self.cov_yaw   = self.get_parameter('cov_yaw').value
        self.min_move  = self.get_parameter('min_move').value

        # State
        self.initialized      = False
        self.current_odom     = None
        self.map_origin_x     = None
        self.map_origin_y     = None
        self.map_origin_yaw   = None
        self.odom_origin_x    = None
        self.odom_origin_y    = None
        self.odom_origin_yaw  = None
        self.last_pub_x       = None
        self.last_pub_y       = None
        self.pub_count        = 0

        # Publisher ke AMCL — WAJIB BEST_EFFORT
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', QOS_AMCL)

        # Subscribe /odom
        self.create_subscription(Odometry, '/odom', self.odom_cb, QOS_ODOM)

        # Subscribe /initialpose dari RViz (RELIABLE)
        # Node ini sebagai RELAY: terima dari RViz → forward ke AMCL
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self.user_set_cb, QOS_RVIZ)

        self.create_timer(self.interval, self.publish_constraint)

        self.get_logger().info("=" * 55)
        self.get_logger().info("✅ Odom Constraint Node started")
        self.get_logger().info(
            f"   QoS publisher: BEST_EFFORT (match AMCL)")
        self.get_logger().info(
            f"   interval={self.interval}s | cov_xy={self.cov_xy}")
        self.get_logger().info(
            "   ⏳ Menunggu 2D Pose Estimate dari RViz...")
        self.get_logger().info("=" * 55)

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg

    def user_set_cb(self, msg: PoseWithCovarianceStamped):
        """
        Tangkap /initialpose dari RViz dan relay ke AMCL.
        RViz publish RELIABLE, AMCL subscribe BEST_EFFORT → tidak match.
        Node ini menjembatani dengan re-publish BEST_EFFORT.
        """
        cov_x = msg.pose.covariance[0]

        # Skip kalau ini dari node sendiri (cov kecil = dari publish_constraint)
        # RViz default cov = 0.25, publish_constraint cov = self.cov_xy (0.04)
        if self.initialized and cov_x < 0.1:
            return

        if self.current_odom is None:
            self.get_logger().warn(
                "[OdomConstraint] /initialpose diterima tapi /odom belum ada!")
            # Tetap relay ke AMCL meski odom belum ada
            relay = PoseWithCovarianceStamped()
            relay.header = msg.header
            relay.pose   = msg.pose
            self.pub.publish(relay)
            return

        # Simpan origin
        self.map_origin_x   = msg.pose.pose.position.x
        self.map_origin_y   = msg.pose.pose.position.y
        self.map_origin_yaw = quat_to_yaw(msg.pose.pose.orientation)

        odom = self.current_odom
        self.odom_origin_x   = odom.pose.pose.position.x
        self.odom_origin_y   = odom.pose.pose.position.y
        self.odom_origin_yaw = quat_to_yaw(odom.pose.pose.orientation)

        self.initialized = True
        self.last_pub_x  = self.map_origin_x
        self.last_pub_y  = self.map_origin_y

        # RELAY ke AMCL dengan QoS BEST_EFFORT
        relay = PoseWithCovarianceStamped()
        relay.header = msg.header
        relay.pose   = msg.pose
        self.pub.publish(relay)

        self.get_logger().info(
            f"✅ [Relay] RViz → AMCL: "
            f"({self.map_origin_x:.2f}, {self.map_origin_y:.2f}) "
            f"yaw={math.degrees(self.map_origin_yaw):.1f}°")

    def publish_constraint(self):
        """Publish /initialpose periodik untuk jangkar partikel ke odom."""
        if not self.initialized or self.current_odom is None:
            return

        odom = self.current_odom
        dx    = odom.pose.pose.position.x - self.odom_origin_x
        dy    = odom.pose.pose.position.y - self.odom_origin_y
        d_yaw = quat_to_yaw(odom.pose.pose.orientation) - self.odom_origin_yaw

        rot   = self.map_origin_yaw - self.odom_origin_yaw
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)

        final_x   = self.map_origin_x + dx * cos_r - dy * sin_r
        final_y   = self.map_origin_y + dx * sin_r + dy * cos_r
        final_yaw = self.map_origin_yaw + d_yaw

        # Skip kalau belum bergerak cukup
        if self.last_pub_x is not None:
            dist = math.sqrt(
                (final_x - self.last_pub_x)**2 +
                (final_y - self.last_pub_y)**2)
            if dist < self.min_move:
                return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x  = final_x
        msg.pose.pose.position.y  = final_y
        msg.pose.pose.orientation = yaw_to_quat(final_yaw)

        c = msg.pose.covariance
        c[0]  = self.cov_xy
        c[7]  = self.cov_xy
        c[35] = self.cov_yaw

        self.pub.publish(msg)
        self.pub_count  += 1
        self.last_pub_x  = final_x
        self.last_pub_y  = final_y

        self.get_logger().info(
            f"[Constraint] #{self.pub_count} "
            f"({final_x:.3f}, {final_y:.3f}) "
            f"yaw={math.degrees(final_yaw):.1f}°")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(OdomConstraintNode())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
pose_relay_node.py — Jembatan RViz → TF map→odom

Masalah yang diselesaikan:
  Tanpa AMCL, tidak ada yang publish TF map→odom
  RViz tidak bisa set pose robot di frame map
  
Solusi:
  Node ini subscribe /initialpose dari RViz
  Lalu publish TF map→odom secara kontinyu
  TF tree: map → odom → base_link → lengkap

Satu-satunya node yang dibutuhkan untuk fase odom murni.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster


def quat_to_yaw(q) -> float:
    return 2.0 * math.atan2(q.z, q.w)


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class PoseRelayNode(Node):

    def __init__(self):
        super().__init__('pose_relay_node')

        self.declare_parameter('tf_publish_rate', 20.0)
        self.rate = self.get_parameter('tf_publish_rate').value

        # State: offset map→odom
        self.map_x   = 0.0
        self.map_y   = 0.0
        self.map_yaw = 0.0

        # Odom saat pose di-set
        self.odom_origin_x   = 0.0
        self.odom_origin_y   = 0.0
        self.odom_origin_yaw = 0.0

        self.initialized = False
        self.current_odom = None

        self.tf_br = TransformBroadcaster(self)

        # Subscribe odom
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_be)

        # Subscribe /initialpose dari RViz (RELIABLE)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self.pose_cb, qos_rel)

        # Publish /robot_pose untuk dibaca node lain
        from rclpy.qos import QoSDurabilityPolicy
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.robot_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/robot_pose', qos_pub)

        # Timer: publish TF map→odom terus-menerus
        self.create_timer(1.0 / self.rate, self.publish_tf)

        self.get_logger().info("=" * 55)
        self.get_logger().info("✅ Pose Relay Node started (odom-only mode)")
        self.get_logger().info("   ⏳ Klik '2D Pose Estimate' di RViz untuk set posisi")
        self.get_logger().info("   TF akan di-publish: map → odom → base_link")
        self.get_logger().info("=" * 55)

    def odom_cb(self, msg: Odometry):
        self.current_odom = msg

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """Terima pose dari RViz, simpan sebagai origin map→odom."""
        if self.current_odom is None:
            self.get_logger().warn(
                "[PoseRelay] /initialpose diterima tapi /odom belum ada!")
            return

        # Pose yang di-set user di frame map
        self.map_x   = msg.pose.pose.position.x
        self.map_y   = msg.pose.pose.position.y
        self.map_yaw = quat_to_yaw(msg.pose.pose.orientation)

        # Odom saat di-set (sebagai referensi)
        odom = self.current_odom
        self.odom_origin_x   = odom.pose.pose.position.x
        self.odom_origin_y   = odom.pose.pose.position.y
        self.odom_origin_yaw = quat_to_yaw(odom.pose.pose.orientation)

        self.initialized = True

        self.get_logger().info(
            f"✅ [PoseRelay] Pose set: map=({self.map_x:.2f},{self.map_y:.2f}) "
            f"yaw={math.degrees(self.map_yaw):.1f}°")
        self.get_logger().info(
            "   TF map→odom aktif. Robot tracking via odom murni.")

    def publish_tf(self):
        """
        Publish TF map→odom berdasarkan pose yang di-set + delta odom.
        
        map→odom transform = pose yang di-set user
        odom→base_link     = dari joint_odom_node (bergerak sesuai langkah)
        
        Robot position di map = map→odom + odom→base_link
        """
        if not self.initialized or self.current_odom is None:
            # Sebelum di-set: publish identity transform
            # agar TF tree tidak broken
            self._publish_map_odom(0.0, 0.0, 0.0)
            return

        odom = self.current_odom
        # Delta odom sejak pose di-set
        dx   = odom.pose.pose.position.x - self.odom_origin_x
        dy   = odom.pose.pose.position.y - self.odom_origin_y
        dyaw = quat_to_yaw(odom.pose.pose.orientation) - self.odom_origin_yaw

        # Rotasi delta odom ke frame map
        rot   = self.map_yaw - self.odom_origin_yaw
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)
        dx_map = dx * cos_r - dy * sin_r
        dy_map = dx * sin_r + dy * cos_r

        # TF map→odom = pose_awal - delta_odom
        # (karena TF map→odom adalah offset, bukan pose robot)
        tf_x   = self.map_x - dx_map
        tf_y   = self.map_y - dy_map
        tf_yaw = self.map_yaw - (quat_to_yaw(odom.pose.pose.orientation))

        self._publish_map_odom(tf_x, tf_y, tf_yaw)
        self._publish_robot_pose()

    def _publish_map_odom(self, x, y, yaw):
        """Publish TF map→odom."""
        tf = TransformStamped()
        tf.header.stamp    = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id  = 'odom'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quat(yaw)
        self.tf_br.sendTransform(tf)

    def _publish_robot_pose(self):
        """Publish /robot_pose untuk dibaca sistem lain."""
        if not self.initialized or self.current_odom is None:
            return

        odom = self.current_odom
        dx   = odom.pose.pose.position.x - self.odom_origin_x
        dy   = odom.pose.pose.position.y - self.odom_origin_y
        dyaw = quat_to_yaw(odom.pose.pose.orientation) - self.odom_origin_yaw

        rot   = self.map_yaw - self.odom_origin_yaw
        cos_r, sin_r = math.cos(rot), math.sin(rot)
        robot_x   = self.map_x + dx * cos_r - dy * sin_r
        robot_y   = self.map_y + dx * sin_r + dy * cos_r
        robot_yaw = self.map_yaw + dyaw

        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x  = robot_x
        msg.pose.pose.position.y  = robot_y
        msg.pose.pose.orientation = yaw_to_quat(robot_yaw)
        msg.pose.covariance[0]  = 0.05
        msg.pose.covariance[7]  = 0.05
        msg.pose.covariance[35] = 0.02
        self.robot_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(PoseRelayNode())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
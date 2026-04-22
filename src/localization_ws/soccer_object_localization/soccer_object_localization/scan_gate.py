#!/usr/bin/env python3
"""
scan_gate.py — Adaptive Scan Filter for Near-Goal Area  [v1.0]
==============================================================
MOTIVASI:
  Di area dekat gawang (|robot_x| > 2.5m), scan laser dari
  simple_pc2scan/scan_stabilizer mendeteksi JARING GAWANG sebagai
  obstacle. Peta AMCL tidak mengandung jaring → mismatch → particle collapse.

  Node ini menggantikan /field_scan_stable dengan versi yang sudah
  difilter: range yang terlalu dekat (kemungkinan jaring) dihapus
  ketika robot berada di dekat gawang.

CARA KERJA:
  1. Subscribe /field_scan_stable (dari scan_stabilizer)
  2. Subscribe /odometry/filtered (pose robot)
  3. Jika robot jauh dari gawang → forward scan tanpa modifikasi
  4. Jika robot dekat gawang (|x| > near_goal_x):
     - Hapus range < close_range_m (kemungkinan jaring gawang)
     - Set range tersebut ke range_max (= tidak ada obstacle)
     - Ini membuat AMCL "tidak melihat" jaring → likelihood lebih baik
  5. Publish ke /field_scan_gated

AMCL di launch file diubah scan_topic → /field_scan_gated

PARAMETER KUNCI:
  near_goal_x_m   : 2.5m  — mulai aktif filter
  close_range_m   : 1.5m  — range < ini dihapus di dekat gawang
  full_filter_x_m : 3.5m  — filter SEMUA range < 2.0m (dalam kotak penalty)
  
  Reasoning:
  - Jaring gawang biasanya di 0.5-1.5m dari robot ketika robot di kotak penalty
  - Garis lapangan (point cloud) range biasanya > 0.3m
  - Filter ini hanya aktif di area dekat gawang, tidak mempengaruhi
    area lain di lapangan
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class ScanGate(Node):
    def __init__(self):
        super().__init__('scan_gate')

        # ── Parameter ────────────────────────────────────────────────────────
        self.declare_parameter('near_goal_x_m',    2.5)   # mulai filter (m dari center)
        self.declare_parameter('full_filter_x_m',  3.5)   # filter agresif (penalty box)
        self.declare_parameter('close_range_m',    1.5)   # hapus range < ini (mode near)
        self.declare_parameter('full_close_range',  2.0)  # hapus range < ini (mode full)
        self.declare_parameter('field_half_len',    4.5)  # panjang setengah lapangan (m)

        # Topic names
        self.declare_parameter('input_topic',  '/field_scan_stable')
        self.declare_parameter('output_topic', '/field_scan_gated')

        # ── Load ─────────────────────────────────────────────────────────────
        p = self.get_parameter
        self.near_goal_x   = p('near_goal_x_m').value
        self.full_filter_x = p('full_filter_x_m').value
        self.close_range   = p('close_range_m').value
        self.full_close    = p('full_close_range').value
        self.fhl           = p('field_half_len').value
        in_topic           = p('input_topic').value
        out_topic          = p('output_topic').value

        # ── State ─────────────────────────────────────────────────────────────
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.n_filtered = 0
        self.n_pass     = 0

        # ── Sub / Pub ─────────────────────────────────────────────────────────
        self.sub_scan = self.create_subscription(
            LaserScan, in_topic, self._cb_scan, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self._cb_odom, qos_profile_sensor_data)

        self.pub_scan = self.create_publisher(LaserScan, out_topic, 10)

        self.get_logger().info(
            f'ScanGate v1.0: {in_topic} → {out_topic}  '
            f'near_goal={self.near_goal_x}m  close_range={self.close_range}m')

        # Timer untuk log statistik
        self.create_timer(10.0, self._log_stats)

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_scan(self, msg: LaserScan):
        abs_x = abs(self.robot_x)

        # Mode 1: Robot jauh dari gawang → forward tanpa modifikasi
        if abs_x < self.near_goal_x:
            self.pub_scan.publish(msg)
            self.n_pass += 1
            return

        # Mode 2: Robot dekat gawang → filter range pendek
        ranges = list(msg.ranges)
        n_removed = 0

        if abs_x >= self.full_filter_x:
            # Mode agresif: di dalam / sangat dekat penalty box
            # Hapus semua range < full_close_range
            threshold = self.full_close
        else:
            # Mode normal: hapus range < close_range
            threshold = self.close_range

        for i, r in enumerate(ranges):
            if math.isfinite(r) and r < threshold:
                ranges[i] = msg.range_max  # = tidak ada obstacle
                n_removed += 1

        # Buat scan baru dengan ranges yang sudah difilter
        filtered_msg = LaserScan()
        filtered_msg.header         = msg.header
        filtered_msg.angle_min      = msg.angle_min
        filtered_msg.angle_max      = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time      = msg.scan_time
        filtered_msg.range_min      = msg.range_min
        filtered_msg.range_max      = msg.range_max
        filtered_msg.ranges         = ranges
        filtered_msg.intensities    = msg.intensities

        self.pub_scan.publish(filtered_msg)
        self.n_filtered += 1

        if n_removed > 0:
            self.get_logger().debug(
                f'ScanGate: robot_x={self.robot_x:.2f}m  '
                f'removed {n_removed}/{len(ranges)} ranges < {threshold}m',
                throttle_duration_sec=2.0)

    # ─────────────────────────────────────────────────────────────────────────
    def _log_stats(self):
        total = self.n_pass + self.n_filtered
        if total == 0:
            return
        self.get_logger().info(
            f'ScanGate stats: pass={self.n_pass}  filtered={self.n_filtered}  '
            f'filter_rate={self.n_filtered/total*100:.0f}%  '
            f'robot_x={self.robot_x:.2f}m')


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
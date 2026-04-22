#!/usr/bin/env python3
"""
Simple PointCloud to LaserScan Converter
Fixed: tambah NaN/Inf filter dan diagnostics log

CHANGELOG:
  v2 — Fix NaN/Inf titik tidak difilter (is_dense=False point cloud
       bisa mengandung NaN yang lolos tanpa check → converted_count=0
       tapi tidak crash, semua range tetap .inf)
     — Tambah log: berapa titik valid dari total
     — Tambah log: sample range yang berhasil dikonversi
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import struct
import math

class SimplePC2Scan(Node):
    def __init__(self):
        super().__init__('simple_pc2scan')
        
        # Parameters
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.0174533)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('scan_height', 0.0)
        
        self.angle_min      = self.get_parameter('angle_min').value
        self.angle_max      = self.get_parameter('angle_max').value
        self.angle_increment= self.get_parameter('angle_increment').value
        self.range_min      = self.get_parameter('range_min').value
        self.range_max      = self.get_parameter('range_max').value
        self.scan_height    = self.get_parameter('scan_height').value
        
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        
        self.scan_pub = self.create_publisher(LaserScan, '/field_scan', 10)
        self.pc_sub   = self.create_subscription(
            PointCloud2,
            '/field_point_cloud',
            self.convert_callback,
            10
        )
        
        self.get_logger().info('Simple PC2Scan v2 started')
        self.get_logger().info(
            f'  range=[{self.range_min:.2f}, {self.range_max:.2f}]m  '
            f'angle=[{math.degrees(self.angle_min):.0f}°, {math.degrees(self.angle_max):.0f}°]  '
            f'bins={self.num_ranges}'
        )

    def convert_callback(self, pc_msg):
        point_step = pc_msg.point_step
        total_points = 0
        nan_points   = 0
        range_skip   = 0
        angle_skip   = 0

        ranges      = [float('inf')] * self.num_ranges
        intensities = [0.0]         * self.num_ranges
        converted   = 0

        for i in range(0, len(pc_msg.data), point_step):
            total_points += 1
            chunk = pc_msg.data[i:i + point_step]
            if len(chunk) < 12:
                nan_points += 1
                continue

            x, y, z = struct.unpack('<fff', bytes(chunk[:12]))

            # ── FIX v2: filter NaN/Inf ────────────────────────────────
            # is_dense=False point cloud bisa mengandung NaN/Inf.
            # Tanpa check ini, math.atan2(NaN) = NaN, semua comparison
            # NaN < x = False → titik tidak di-skip tapi juga tidak masuk
            # index dengan benar → semua range tetap .inf
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                nan_points += 1
                continue

            angle    = math.atan2(y, x)
            distance = math.sqrt(x*x + y*y)

            if angle < self.angle_min or angle > self.angle_max:
                angle_skip += 1
                continue
            if distance < self.range_min or distance > self.range_max:
                range_skip += 1
                continue

            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.num_ranges:
                if distance < ranges[index]:
                    ranges[index] = distance
                    converted += 1

        # Publish selalu (meski kosong) agar downstream tidak stale
        scan_msg                 = LaserScan()
        scan_msg.header          = pc_msg.header
        scan_msg.angle_min       = self.angle_min
        scan_msg.angle_max       = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment  = 0.0
        scan_msg.scan_time       = 0.1
        scan_msg.range_min       = self.range_min
        scan_msg.range_max       = self.range_max
        scan_msg.ranges          = ranges
        scan_msg.intensities     = intensities
        self.scan_pub.publish(scan_msg)

        # ── Diagnostics log ──────────────────────────────────────────
        valid_ranges = [r for r in ranges if math.isfinite(r)]
        if converted > 0:
            self.get_logger().info(
                f'[OK] {total_points} pts → {converted} hits  '
                f'nan={nan_points} range_skip={range_skip}  '
                f'scan_valid={len(valid_ranges)}/{self.num_ranges} bins',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().warn(
                f'[EMPTY] {total_points} pts, 0 hits  '
                f'nan={nan_points}  range_skip={range_skip}  angle_skip={angle_skip}  '
                f'range=[{self.range_min},{self.range_max}]',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimplePC2Scan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
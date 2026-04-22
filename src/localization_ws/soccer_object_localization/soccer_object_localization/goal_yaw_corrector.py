#!/usr/bin/env python3
"""
goal_yaw_corrector.py - Yaw-Only Correction via Goal Line Detection

Mendeteksi kemiringan garis horizontal gawang dari kamera,
lalu publish HANYA koreksi yaw ke /initialpose.

FILOSOFI:
  Gawang sejajar sumbu Y -> garis atas/bawah gawang selalu horizontal.
  Kemiringan garis di gambar = yaw error robot terhadap arah gawang.

SAFETY:
  - max_yaw_delta_deg: koreksi max dari odom yaw
  - cooldown_sec: jeda antar koreksi
  - cov_xy = 9.0: posisi tidak disentuh AMCL
  - cov_yaw = 0.015: AMCL percaya koreksi yaw ini

CHANGELOG v2.1:
  [FIX 1] roi_top 0.05->0.02, roi_bottom 0.50->0.42 — fokus ke area gawang saja
  [FIX 2] white_threshold 180->200 — lebih ketat, noise rumput tidak lolos
  [FIX 3] Tambah morphological opening sebelum Canny — hapus noise pixel sporadis
  [FIX 4] min_line_length_px 120->200 — hanya garis panjang (crossbar gawang)
  [FIX 5] Tambah area filter: tolak garis yang y-nya terlalu rendah (bukan gawang)
  [FIX 6] Jika tidak ada gawang terdeteksi → tidak publish, tidak crash
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time


class GoalYawCorrector(Node):
    def __init__(self):
        super().__init__('goal_yaw_corrector')

        self.declare_parameter('focal_length',         900.0)
        self.declare_parameter('image_width',          1280)
        self.declare_parameter('image_height',         720)
        # [FIX 1] ROI lebih sempit — hanya area gawang, bukan rumput
        self.declare_parameter('roi_top',              0.02)   # 2% dari atas (was 0.05)
        self.declare_parameter('roi_bottom',           0.42)   # 42% dari atas (was 0.50)
        # [FIX 2] white_threshold lebih ketat
        self.declare_parameter('white_threshold',      200)    # was 180
        # [FIX 4] min_line_length lebih panjang — hanya crossbar gawang
        self.declare_parameter('min_line_length_px',   200)    # was 120
        self.declare_parameter('max_line_angle_deg',   25.0)
        self.declare_parameter('max_yaw_delta_deg',    20.0)
        self.declare_parameter('min_confidence',       0.5)
        self.declare_parameter('cooldown_sec',         2.0)
        self.declare_parameter('cov_xy',               9.0)
        self.declare_parameter('cov_yaw',              0.015)

        self.focal         = self.get_parameter('focal_length').value
        self.img_w         = self.get_parameter('image_width').value
        self.img_h         = self.get_parameter('image_height').value
        self.roi_top       = self.get_parameter('roi_top').value
        self.roi_bottom    = self.get_parameter('roi_bottom').value
        self.white_thr     = self.get_parameter('white_threshold').value
        self.min_line_px   = self.get_parameter('min_line_length_px').value
        self.max_angle_deg = self.get_parameter('max_line_angle_deg').value
        self.max_yaw_delta = math.radians(self.get_parameter('max_yaw_delta_deg').value)
        self.min_conf      = self.get_parameter('min_confidence').value
        self.cooldown      = self.get_parameter('cooldown_sec').value
        self.cov_xy        = self.get_parameter('cov_xy').value
        self.cov_yaw       = self.get_parameter('cov_yaw').value

        self.bridge        = CvBridge()
        self.odom_x        = 0.0
        self.odom_y        = 0.0
        self.odom_yaw      = 0.0
        self.last_pub_time = 0.0

        self.sub_image = self.create_subscription(
            Image, '/robotis_op3/camera/image_raw',
            self._cb_image, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom',
            self._cb_odom, qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_debug = self.create_publisher(
            Image, '/goal_yaw/debug_image', 10)

        self.get_logger().info('GoalYawCorrector v2.1 started')
        self.get_logger().info(
            f'  ROI {self.roi_top*100:.0f}%-{self.roi_bottom*100:.0f}%  '
            f'white_thr={self.white_thr}  '
            f'min_line={self.min_line_px}px  '
            f'max_delta={self.get_parameter("max_yaw_delta_deg").value}deg  '
            f'cooldown={self.cooldown}s')

    def _cb_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)

    def _cb_image(self, msg):
        if time.time() - self.last_pub_time < self.cooldown:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}', throttle_duration_sec=5.0)
            return

        result = self._detect_goal_line(frame)
        self._publish_debug(frame, result, msg.header.stamp)

        if result is None:
            return

        line_angle_rad, confidence, line_pts = result

        if confidence < self.min_conf:
            return

        # Tentukan arah hadap ke gawang dari odom yaw
        if abs(self.odom_yaw) < math.pi / 2.0:
            goal_facing_yaw = 0.0        # menghadap gawang +X
        else:
            goal_facing_yaw = math.pi    # menghadap gawang -X

        # Yaw robot = arah ke gawang - kemiringan garis di gambar
        yaw_corrected = goal_facing_yaw - line_angle_rad

        # Normalisasi [-pi, pi]
        while yaw_corrected > math.pi:
            yaw_corrected -= 2 * math.pi
        while yaw_corrected < -math.pi:
            yaw_corrected += 2 * math.pi

        # Safety: delta dari odom
        yaw_delta = yaw_corrected - self.odom_yaw
        while yaw_delta > math.pi:
            yaw_delta -= 2 * math.pi
        while yaw_delta < -math.pi:
            yaw_delta += 2 * math.pi

        if abs(yaw_delta) > self.max_yaw_delta:
            self.get_logger().warn(
                f'delta={math.degrees(abs(yaw_delta)):.1f}deg > '
                f'{self.get_parameter("max_yaw_delta_deg").value}deg -> skip',
                throttle_duration_sec=2.0)
            return

        # Publish /initialpose - hanya koreksi yaw
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp    = msg.header.stamp
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = self.odom_x
        pose_msg.pose.pose.position.y = self.odom_y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw_corrected / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw_corrected / 2.0)

        # Covariance: posisi tidak disentuh (cov_xy besar), yaw kecil
        cov = [0.0] * 36
        cov[0]  = self.cov_xy
        cov[7]  = self.cov_xy
        cov[14] = 0.01
        cov[21] = 0.01
        cov[28] = 0.01
        cov[35] = self.cov_yaw
        pose_msg.pose.covariance = cov

        self.pub_pose.publish(pose_msg)
        self.last_pub_time = time.time()

        self.get_logger().info(
            f'[YAW] line={math.degrees(line_angle_rad):+.1f}deg  '
            f'odom={math.degrees(self.odom_yaw):+.1f}deg  '
            f'corrected={math.degrees(yaw_corrected):+.1f}deg  '
            f'delta={math.degrees(abs(yaw_delta)):.1f}deg  '
            f'conf={confidence:.2f}')

    def _detect_goal_line(self, frame):
        h, w = frame.shape[:2]
        y_top = int(h * self.roi_top)
        y_bot = int(h * self.roi_bottom)
        roi   = frame[y_top:y_bot, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # [FIX 2] Threshold lebih ketat (200 vs 180)
        _, binary = cv2.threshold(gray, self.white_thr, 255, cv2.THRESH_BINARY)

        # [FIX 3] Morphological opening — hapus noise pixel sporadis sebelum Canny
        # Kernel 3x3 cukup untuk hapus noise 1-2px tanpa merusak garis gawang
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

        edges = cv2.Canny(binary, 50, 150)

        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi/180,
            threshold=40,
            minLineLength=self.min_line_px,   # [FIX 4] 200px (was 120)
            maxLineGap=20)

        if lines is None:
            return None

        best_line   = None
        best_length = 0.0
        max_angle   = math.radians(self.max_angle_deg)

        # [FIX 5] Batas y maksimum dalam ROI untuk garis gawang
        # Crossbar gawang ada di bagian ATAS ROI, bukan bagian bawah
        # Tolak garis yang y-nya lebih dari 60% tinggi ROI (itu area rumput)
        roi_h = y_bot - y_top
        max_y_in_roi = int(roi_h * 0.60)

        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length = math.sqrt(dx*dx + dy*dy)
            angle  = math.atan2(dy, dx)

            # Filter sudut: hanya garis horizontal
            if abs(angle) > max_angle:
                continue

            # [FIX 5] Filter posisi: garis di bawah 60% ROI bukan gawang
            y_mid = (y1 + y2) / 2.0
            if y_mid > max_y_in_roi:
                continue

            if length > best_length:
                best_length = length
                best_line   = (x1, y1 + y_top, x2, y2 + y_top, angle)

        if best_line is None:
            return None

        x1, y1, x2, y2, angle = best_line

        confidence = min(best_length / (self.min_line_px * 3.0), 1.0)
        if best_length > w / 3.0:
            confidence = min(confidence + 0.2, 1.0)

        return angle, confidence, (x1, y1, x2, y2)

    def _publish_debug(self, frame, result, stamp):
        debug = frame.copy()
        h, w  = debug.shape[:2]
        y_top = int(h * self.roi_top)
        y_bot = int(h * self.roi_bottom)

        # Garis ROI kuning
        cv2.line(debug, (0, y_top), (w, y_top), (0, 255, 255), 1)
        cv2.line(debug, (0, y_bot), (w, y_bot), (0, 255, 255), 1)

        # [FIX 5] Garis batas max_y untuk gawang (visualisasi)
        roi_h = y_bot - y_top
        max_y_abs = y_top + int(roi_h * 0.60)
        cv2.line(debug, (0, max_y_abs), (w, max_y_abs), (255, 128, 0), 1)  # oranye

        if result is not None:
            angle, conf, (x1, y1, x2, y2) = result
            color = (0, 255, 0) if conf >= self.min_conf else (0, 165, 255)
            cv2.line(debug, (x1, y1), (x2, y2), color, 3)
            label = f'angle={math.degrees(angle):+.1f}deg  conf={conf:.2f}'
            cv2.putText(debug, label, (x1, max(y1-10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        else:
            cv2.putText(debug, 'no goal line', (10, y_top + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.putText(debug,
                    f'odom_yaw={math.degrees(self.odom_yaw):+.1f}deg',
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)
        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            dbg_msg.header.stamp = stamp
            self.pub_debug.publish(dbg_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GoalYawCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
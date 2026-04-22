#!/usr/bin/env python3
"""
goal_detector.py — Goal-Based Pose Correction for AMCL
=======================================================
Mendeteksi gawang dari kamera, mengestimasi yaw dan posisi robot,
lalu mempublikasikan koreksi ke AMCL via /initialpose.

CARA KERJA:
  1. Deteksi gawang: cari region putih padat (grid horizontal+vertikal)
     dari debug_image atau raw image
  2. Estimasi yaw: dari offset horizontal center gawang terhadap cx
  3. Estimasi jarak: dari lebar gawang di pixel vs lebar real 2.6m
  4. Estimasi sisi: dari odom x (positif = sisi kanan, negatif = sisi kiri)
  5. Full pose: kombinasi jarak + sisi + yaw → x, y, yaw absolut
  6. Publish /initialpose hanya jika confidence cukup tinggi

OUTPUT:
  /initialpose (geometry_msgs/PoseWithCovarianceStamped)
    - yaw-only correction: cov_x=cov_y besar, cov_yaw kecil
    - full pose correction: semua covariance kecil jika confidence tinggi

PARAMETER:
  goal_width_m        : lebar gawang di dunia nyata (default 2.6m)
  goal_height_m       : tinggi gawang (default 1.2m)
  focal_length        : focal length kamera (default 900.0 px)
  image_width         : lebar gambar (default 1280)
  image_height        : tinggi gambar (default 720)
  field_half_length   : setengah panjang lapangan, jarak dari center ke gawang (default 4.5m)
  min_goal_width_px   : lebar minimum gawang untuk dianggap valid (default 80px)
  min_confidence      : confidence minimum untuk publish koreksi (default 0.6)
  correction_interval : jarak minimum (meter) sebelum publish koreksi lagi (default 0.5)
  yaw_correction_only_threshold : jika confidence < ini, hanya koreksi yaw (default 0.75)
  max_yaw_correction_deg : maksimum koreksi yaw yang diizinkan, hindari false positive (default 30.0)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time


class GoalDetector(Node):
    def __init__(self):
        super().__init__('goal_detector')

        # ── Parameter ────────────────────────────────────────────────
        self.declare_parameter('goal_width_m',          2.6)
        self.declare_parameter('goal_height_m',         1.2)
        self.declare_parameter('focal_length',          900.0)
        self.declare_parameter('image_width',           1280)
        self.declare_parameter('image_height',          720)
        self.declare_parameter('field_half_length',     4.5)   # center → gawang (m)
        self.declare_parameter('field_half_width',      3.0)   # center → sisi (m)
        self.declare_parameter('min_goal_width_px',     80)
        self.declare_parameter('min_confidence',        0.6)
        self.declare_parameter('correction_interval',   0.5)   # meter
        self.declare_parameter('yaw_only_threshold',    0.75)  # confidence < ini → yaw only
        self.declare_parameter('max_yaw_correction_deg', 30.0)
        self.declare_parameter('white_threshold',       200)   # pixel threshold

        self.goal_w        = self.get_parameter('goal_width_m').value
        self.goal_h        = self.get_parameter('goal_height_m').value
        self.focal         = self.get_parameter('focal_length').value
        self.img_w         = self.get_parameter('image_width').value
        self.img_h         = self.get_parameter('image_height').value
        self.field_half_x  = self.get_parameter('field_half_length').value
        self.field_half_y  = self.get_parameter('field_half_width').value
        self.min_goal_px   = self.get_parameter('min_goal_width_px').value
        self.min_conf      = self.get_parameter('min_confidence').value
        self.corr_interval = self.get_parameter('correction_interval').value
        self.yaw_only_thr  = self.get_parameter('yaw_only_threshold').value
        self.max_yaw_deg   = self.get_parameter('max_yaw_correction_deg').value
        self.white_thr     = self.get_parameter('white_threshold').value

        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0

        # ── State ────────────────────────────────────────────────────
        self.bridge          = CvBridge()
        self.current_odom_x  = 0.0
        self.current_odom_y  = 0.0
        self.current_odom_yaw= 0.0
        self.last_correction_x = None
        self.last_correction_y = None
        self.last_correction_time = 0.0

        # ── Sub / Pub ────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image,
            '/robotis_op3/camera/image_raw',
            self._cb_image,
            rclpy.qos.qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._cb_odom,
            rclpy.qos.qos_profile_sensor_data
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.debug_pub = self.create_publisher(
            Image,
            '/goal_detector/debug_image',
            10
        )

        self.get_logger().info('Goal Detector started')
        self.get_logger().info(
            f'  Goal size: {self.goal_w}m x {self.goal_h}m  '
            f'  field_half: {self.field_half_x}m  '
            f'  focal: {self.focal}px'
        )

    # ── Callback: odom ───────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_odom_yaw = math.atan2(siny, cosy)

    # ── Callback: image ──────────────────────────────────────────────
    def _cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}', throttle_duration_sec=5.0)
            return

        result = self._detect_goal(frame)
        if result is None:
            return

        bbox, confidence = result
        x1, y1, x2, y2 = bbox
        goal_w_px = x2 - x1
        goal_cx_px = (x1 + x2) / 2.0

        # ── Debug image ──────────────────────────────────────────────
        debug = frame.copy()
        cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(debug, f'conf={confidence:.2f} w={goal_w_px}px',
                    (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, 'bgr8'))
        except Exception:
            pass

        if confidence < self.min_conf:
            self.get_logger().debug(
                f'Goal detected but confidence too low: {confidence:.2f}',
                throttle_duration_sec=2.0
            )
            return

        if goal_w_px < self.min_goal_px:
            self.get_logger().debug(
                f'Goal too small: {goal_w_px}px < {self.min_goal_px}px',
                throttle_duration_sec=2.0
            )
            return

        # ── Estimasi yaw ─────────────────────────────────────────────
        # Offset horizontal center gawang dari center gambar
        # Positif = gawang di kanan gambar → robot menghadap sedikit ke kanan gawang
        dx_px = goal_cx_px - self.cx
        yaw_to_goal = math.atan2(dx_px, self.focal)  # radian, kecil jika centered

        # Yaw absolut robot: jika gawang di depan, robot menghadap 0° atau 180°
        # Tentukan arah berdasarkan odom x
        if self.current_odom_x >= 0:
            # Robot di sisi positif → menghadap gawang kanan (+X) → yaw ≈ 0
            goal_facing_yaw = 0.0
            goal_x_abs = self.field_half_x
        else:
            # Robot di sisi negatif → menghadap gawang kiri (-X) → yaw ≈ π
            goal_facing_yaw = math.pi
            goal_x_abs = -self.field_half_x

        # Yaw robot = arah ke gawang + offset dari center gambar
        estimated_yaw = goal_facing_yaw - yaw_to_goal

        # Validasi: koreksi yaw tidak boleh terlalu besar dari odom
        yaw_diff = abs(estimated_yaw - self.current_odom_yaw)
        # Normalisasi ke [-π, π]
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        yaw_diff = abs(yaw_diff)

        if math.degrees(yaw_diff) > self.max_yaw_deg:
            self.get_logger().warn(
                f'Yaw correction too large: {math.degrees(yaw_diff):.1f}° > {self.max_yaw_deg}°, skip',
                throttle_duration_sec=2.0
            )
            return

        # ── Estimasi jarak ───────────────────────────────────────────
        # Lebar gawang: goal_w_m = goal_w_px * distance / focal
        # → distance = goal_w_m * focal / goal_w_px
        distance_to_goal = (self.goal_w * self.focal) / goal_w_px

        # ── Full pose estimation ──────────────────────────────────────
        # Posisi robot = posisi gawang - jarak dalam arah yaw
        robot_x = goal_x_abs - distance_to_goal * math.cos(estimated_yaw)
        robot_y = self.current_odom_y  # y dari odom, lebih reliable

        # Cek apakah sudah cukup jauh dari koreksi terakhir
        now = time.time()
        if self.last_correction_x is not None:
            dist_since_last = math.sqrt(
                (self.current_odom_x - self.last_correction_x) ** 2 +
                (self.current_odom_y - self.last_correction_y) ** 2
            )
            time_since_last = now - self.last_correction_time
            if dist_since_last < self.corr_interval and time_since_last < 2.0:
                return  # Belum cukup jauh, skip

        # ── Publish pose correction ───────────────────────────────────
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = robot_x
        pose_msg.pose.pose.position.y = robot_y
        pose_msg.pose.pose.position.z = 0.0

        # Quaternion dari yaw
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(estimated_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(estimated_yaw / 2.0)

        # Covariance: 6x6 diagonal [x, y, z, roll, pitch, yaw]
        cov = [0.0] * 36
        if confidence >= self.yaw_only_thr:
            # Full pose correction — confidence tinggi
            cov[0]  = 0.08    # x: ±28cm
            cov[7]  = 0.15    # y: ±39cm (dari odom, tidak terlalu akurat)
            cov[35] = 0.02    # yaw: ±8°
            mode = 'FULL'
        else:
            # Yaw-only correction — confidence rendah
            cov[0]  = 1.0     # x: besar, jangan paksa posisi
            cov[7]  = 1.0     # y: besar
            cov[35] = 0.02    # yaw: kecil, ini yang kita yakin
            mode = 'YAW-ONLY'

        pose_msg.pose.covariance = cov
        self.pose_pub.publish(pose_msg)

        self.last_correction_x = self.current_odom_x
        self.last_correction_y = self.current_odom_y
        self.last_correction_time = now

        self.get_logger().info(
            f'[{mode}] goal conf={confidence:.2f}  '
            f'dist={distance_to_goal:.2f}m  '
            f'yaw_est={math.degrees(estimated_yaw):.1f}°  '
            f'pose=({robot_x:.2f}, {robot_y:.2f})',
            throttle_duration_sec=1.0
        )

    # ── Goal Detection ────────────────────────────────────────────────
    def _detect_goal(self, frame: np.ndarray):
        """
        Deteksi gawang dari frame kamera.
        Gawang = region putih padat dengan grid horizontal+vertikal.
        Return: (bbox, confidence) atau None
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # Hanya proses bagian atas gambar (gawang di atas horizon)
        roi_h = int(h * 0.5)  # 50% atas
        roi = gray[:roi_h, :]

        # Threshold: ambil pixel putih
        _, binary = cv2.threshold(roi, self.white_thr, 255, cv2.THRESH_BINARY)

        # Cari region padat dengan banyak garis horizontal dan vertikal
        # Gunakan morfologi untuk menutup grid gawang menjadi satu blob
        kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 3))
        kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 10))
        kernel_fill = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))

        # Deteksi garis horizontal
        horiz = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_h)
        # Deteksi garis vertikal
        vert  = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_v)

        # Gabungkan: region yang punya kedua komponen = kandidat gawang
        combined = cv2.add(horiz, vert)
        filled   = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel_fill)
        filled   = cv2.dilate(filled, kernel_fill, iterations=2)

        # Cari contour
        contours, _ = cv2.findContours(filled, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best_bbox  = None
        best_score = 0.0

        for cnt in contours:
            x1, y1, bw, bh = cv2.boundingRect(cnt)
            if bw < self.min_goal_px:
                continue
            if bh < 20:
                continue

            # Aspect ratio gawang: w/h = 2.6/1.2 = 2.17
            expected_ratio = self.goal_w / self.goal_h  # 2.17
            actual_ratio   = bw / max(bh, 1)
            ratio_score    = 1.0 - min(abs(actual_ratio - expected_ratio) / expected_ratio, 1.0)

            # Density: berapa banyak pixel putih di dalam bbox
            roi_bbox = binary[y1:y1+bh, x1:x1+bw]
            density  = np.sum(roi_bbox > 0) / max(bw * bh, 1)
            # Gawang punya density ~0.3-0.6 (grid, bukan solid putih)
            density_score = 1.0 - abs(density - 0.45) / 0.45
            density_score = max(0.0, density_score)

            # Posisi: gawang harus di atas atau sekitar horizon (bagian atas ROI)
            pos_score = 1.0 - (y1 + bh/2) / roi_h  # lebih tinggi = lebih baik

            confidence = (ratio_score * 0.4 + density_score * 0.4 + pos_score * 0.2)

            if confidence > best_score:
                best_score = confidence
                best_bbox  = (x1, y1, x1 + bw, y1 + bh)

        if best_bbox is None:
            return None

        return best_bbox, best_score


def main(args=None):
    rclpy.init(args=args)
    node = GoalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
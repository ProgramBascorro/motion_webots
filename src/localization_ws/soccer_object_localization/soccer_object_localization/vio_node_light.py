#!/usr/bin/env python3
"""
VIO Node v3.1 - Optimized for Performance
Perbaikan dari v3:
- Resize frame ke 320x240 SEBELUM processing (hemat ~4x komputasi)
- VP detection hanya tiap N frame (tidak setiap frame)
- History window dikurangi
- Semua processing di resolusi kecil
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

import math
from collections import deque

OP3_MAX_SPEED       = 0.20
IMU_SHAKE_THRESHOLD = 0.8

# Resolusi kerja internal (bukan resolusi kamera asli)
PROC_WIDTH  = 320
PROC_HEIGHT = 240


class VanishingPointDetector:
    """
    VP Detector yang dioptimalkan:
    - Hanya dipanggil tiap N frame (bukan setiap frame)
    - History lebih kecil (10 → 7)
    - Early exit kalau garis sedikit
    """

    def __init__(self):
        self.yaw_vp_history = deque(maxlen=7)  # dikurangi dari 15
        self.vp_yaw_offset  = None
        self.vp_confidence  = 0.0
        # Cache hasil terakhir (dipakai antar frame)
        self._cached_yaw    = 0.0
        self._cached_conf   = 0.0

    def estimate_yaw_from_lines(self, lines) -> tuple:
        if lines is None or len(lines) < 4:  # naikkan minimum dari 3 ke 4
            return self._cached_yaw, self._cached_conf * 0.9  # decay cache

        horizontal_angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1:
                continue
            angle = math.atan2(y2 - y1, x2 - x1)
            if angle < 0: angle += math.pi
            length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            # Hanya ambil garis cukup panjang (>20px di resolusi 320x240)
            if length < 20:
                continue
            if abs(angle) < math.radians(35) or abs(angle - math.pi) < math.radians(35):
                horizontal_angles.append((angle, length))

        if len(horizontal_angles) < 3:
            return self._cached_yaw, self._cached_conf * 0.9

        total_w = sum(l for _, l in horizontal_angles)
        if total_w == 0:
            return self._cached_yaw, 0.0

        sin_s = sum(math.sin(2*a) * l for a, l in horizontal_angles)
        cos_s = sum(math.cos(2*a) * l for a, l in horizontal_angles)
        mean_angle = math.atan2(sin_s, cos_s) / 2.0
        conf = min(len(horizontal_angles) / 6.0, 1.0)

        self.yaw_vp_history.append((mean_angle, conf))

        if len(self.yaw_vp_history) >= 4:
            angles = [a for a, _ in self.yaw_vp_history]
            confs  = [c for _, c in self.yaw_vp_history]
            stable = float(np.median(angles))
            sconf  = float(np.mean(confs))

            if self.vp_yaw_offset is None:
                self.vp_yaw_offset = stable
                self._cached_yaw, self._cached_conf = 0.0, sconf
                return 0.0, sconf

            yaw = stable - self.vp_yaw_offset
            while yaw >  math.pi: yaw -= 2*math.pi
            while yaw < -math.pi: yaw += 2*math.pi

            self._cached_yaw  = yaw
            self._cached_conf = sconf
            self.vp_confidence = sconf
            return yaw, sconf

        return self._cached_yaw, self._cached_conf * 0.9


class AdaptiveLandmarkDetector:

    def __init__(self, focal_length=900.0, camera_height=0.475, camera_tilt=-0.349,
                 orig_w=1280, orig_h=720):
        self.camera_tilt = camera_tilt

        # Skala focal length ke resolusi kerja
        scale_x = PROC_WIDTH  / orig_w
        scale_y = PROC_HEIGHT / orig_h
        self.focal_length = focal_length * ((scale_x + scale_y) / 2.0)

        self.DEAD_ZONE_STATIC  = 1.5
        self.DEAD_ZONE_WALKING = 4.0
        self.current_dead_zone = self.DEAD_ZONE_STATIC

        avg_dist    = 2.5
        tilt_factor = math.cos(abs(camera_tilt))
        eff_dist    = avg_dist / max(tilt_factor, 0.5)
        self.SCALE  = eff_dist / self.focal_length

        self.velocity_history   = deque(maxlen=5)  # dikurangi dari 7
        self.line_count_history = deque(maxlen=4)  # dikurangi dari 5

        print(f"[VIO] SCALE={self.SCALE:.6f} m/px @ {PROC_WIDTH}x{PROC_HEIGHT}")

    def set_walking_mode(self, is_walking: bool):
        self.current_dead_zone = (
            self.DEAD_ZONE_WALKING if is_walking else self.DEAD_ZONE_STATIC
        )

    def get_white_mask(self, image: np.ndarray) -> np.ndarray:
        """Image sudah di-resize ke PROC_WIDTH x PROC_HEIGHT sebelum masuk sini."""
        hsv        = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        grass_mask = cv2.inRange(hsv, (35, 40, 0), (85, 255, 255))
        non_grass  = cv2.bitwise_not(grass_mask)
        gray       = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, white   = cv2.threshold(gray, 165, 255, cv2.THRESH_BINARY)
        result     = cv2.bitwise_and(white, white, mask=non_grass)
        h, w       = result.shape
        roi        = np.zeros_like(result)
        roi[int(h*0.35):int(h*0.92), :] = 255
        return cv2.bitwise_and(result, result, mask=roi)

    def detect_lines(self, white_mask: np.ndarray):
        edges = cv2.Canny(white_mask, 50, 150)
        return cv2.HoughLinesP(
            edges, rho=1, theta=np.pi/180, threshold=50,
            minLineLength=12, maxLineGap=20
        )

    def get_line_confidence(self, lines) -> float:
        if lines is None:
            self.line_count_history.append(0)
            return 0.0
        n = len(lines)
        self.line_count_history.append(n)
        conf = min(n / 8.0, 1.0)
        if len(self.line_count_history) >= 3:
            avg = np.mean(list(self.line_count_history)[:-1])
            if avg > 0 and (n / avg) < 0.3:
                conf *= 0.5
        return conf

    def estimate_flow(self, lines_prev, lines_curr, dt, imu_av=0.0):
        if lines_prev is None or lines_curr is None or len(lines_prev) < 3:
            return 0.0, 0.0, 0.0, 0.0

        def centroid(lines):
            xs = [(l[0][0] + l[0][2]) / 2 for l in lines]
            ys = [(l[0][1] + l[0][3]) / 2 for l in lines]
            return float(np.median(xs)), float(np.median(ys))

        cx_p, cy_p = centroid(lines_prev)
        cx_c, cy_c = centroid(lines_curr)
        dx = cx_c - cx_p
        dy = cy_c - cy_p

        if abs(dx) < self.current_dead_zone: dx = 0.0
        if abs(dy) < self.current_dead_zone: dy = 0.0
        if dx == 0.0 and dy == 0.0:
            return 0.0, 0.0, 0.0, 1.0

        vx, vy = 0.0, 0.0
        if dt > 0.001:
            vx = max(-OP3_MAX_SPEED, min(OP3_MAX_SPEED, -dy * self.SCALE / dt))
            vy = max(-OP3_MAX_SPEED, min(OP3_MAX_SPEED, -dx * self.SCALE / dt))

        imu_f = 1.0
        if imu_av > IMU_SHAKE_THRESHOLD:
            imu_f = max(0.1, 1.0 - (imu_av - IMU_SHAKE_THRESHOLD) * 0.5)
        vx *= imu_f
        vy *= imu_f

        conf = self.get_line_confidence(lines_curr) * imu_f
        return vx, vy, 0.0, conf

    def smooth_velocity(self, vx, vy, vyaw):
        self.velocity_history.append((vx, vy, vyaw))
        if len(self.velocity_history) < 3:
            return vx, vy, vyaw
        arr = list(self.velocity_history)
        return (
            float(np.median([v[0] for v in arr])),
            float(np.median([v[1] for v in arr])),
            float(np.median([v[2] for v in arr])),
        )


class VIONodeV3(Node):

    def __init__(self):
        super().__init__('vio_node')

        self.declare_parameter('base_frame',         'cam_link')
        self.declare_parameter('odom_frame',         'odom')
        self.declare_parameter('focal_length',       900.0)
        self.declare_parameter('camera_height',      0.475)
        self.declare_parameter('camera_tilt',       -0.349)
        self.declare_parameter('orig_image_width',   1280)
        self.declare_parameter('orig_image_height',  720)
        self.declare_parameter('smoothing_alpha',    0.5)
        self.declare_parameter('min_visual_motion',  0.01)
        self.declare_parameter('motion_threshold',   0.005)
        self.declare_parameter('pose_cov_xy',        0.5)
        self.declare_parameter('pose_cov_yaw',       0.2)
        self.declare_parameter('twist_cov_xy',       0.5)
        self.declare_parameter('twist_cov_yaw',      0.2)
        self.declare_parameter('use_vp_yaw',         True)
        # VP hanya dihitung tiap N frame (hemat CPU)
        self.declare_parameter('vp_every_n_frames',  5)

        self.base_frame   = self.get_parameter('base_frame').value
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.alpha        = self.get_parameter('smoothing_alpha').value
        self.min_motion   = self.get_parameter('min_visual_motion').value
        self.mot_thresh   = self.get_parameter('motion_threshold').value
        self.cov_xy       = self.get_parameter('pose_cov_xy').value
        self.cov_yaw      = self.get_parameter('pose_cov_yaw').value
        self.tcov_xy      = self.get_parameter('twist_cov_xy').value
        self.tcov_yaw     = self.get_parameter('twist_cov_yaw').value
        self.use_vp_yaw   = self.get_parameter('use_vp_yaw').value
        self.vp_every_n   = self.get_parameter('vp_every_n_frames').value
        orig_w = self.get_parameter('orig_image_width').value
        orig_h = self.get_parameter('orig_image_height').value

        # State
        self.x = self.y = self.yaw = 0.0
        self.vx_s = self.vy_s = self.vyaw = 0.0
        self.prev_lines = None
        self.prev_ts    = None

        # IMU
        self.imu_yaw         = 0.0
        self.imu_init_yaw    = None
        self.has_imu         = False
        self.imu_angular_vel = 0.0

        # VP yaw cache
        self.vp_yaw  = 0.0
        self.vp_conf = 0.0

        # Joint motion
        self.is_walking     = False
        self.prev_joints    = {}
        self.stationary_cnt = 0
        self.motion_cnt     = 0

        focal  = self.get_parameter('focal_length').value
        height = self.get_parameter('camera_height').value
        tilt   = self.get_parameter('camera_tilt').value

        self.detector  = AdaptiveLandmarkDetector(focal, height, tilt, orig_w, orig_h)
        self.vp_det    = VanishingPointDetector()

        self.br       = CvBridge()
        self.tf_bc    = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.create_subscription(Image,      '/robotis_op3/camera/image_raw', self.image_cb, qos)
        self.create_subscription(Imu,        '/robotis_op3/imu',              self.imu_cb,   qos)
        self.create_subscription(Imu,        '/imu/data',                     self.imu_cb,   qos)
        self.create_subscription(JointState, '/robotis_op3/joint_states',     self.joint_cb, 10)

        self.frame_cnt   = 0
        self.vp_used_cnt = 0
        self.create_timer(5.0, self.log_stats)
        self.create_timer(1.0, self.log_motion)

        self.get_logger().info("=" * 65)
        self.get_logger().info("✅ VIO Node v3.1 (Optimized) started")
        self.get_logger().info(f"   Processing at {PROC_WIDTH}x{PROC_HEIGHT} (from {orig_w}x{orig_h})")
        self.get_logger().info(f"   VP every {self.vp_every_n} frames | use_vp={self.use_vp_yaw}")
        self.get_logger().info(f"   alpha={self.alpha} | pose_cov=({self.cov_xy},{self.cov_yaw})")
        self.get_logger().info("=" * 65)

    def imu_cb(self, msg: Imu):
        q    = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)
        raw  = math.atan2(siny, cosy)
        if self.imu_init_yaw is None:
            self.imu_init_yaw = raw
            self.get_logger().info(f"[IMU] offset={math.degrees(raw):.1f}°")
        self.imu_yaw = raw - self.imu_init_yaw
        self.has_imu = True
        av = msg.angular_velocity
        self.imu_angular_vel = math.sqrt(av.x**2 + av.y**2 + av.z**2)

    def joint_cb(self, msg: JointState):
        leg_kw = ['hip_p', 'kn_p', 'knee', 'an_p', 'ank', 'hip_y', 'hip_r']
        moving = False
        for i, name in enumerate(msg.name):
            if any(kw in name for kw in leg_kw) and i < len(msg.position):
                cur = msg.position[i]
                if name in self.prev_joints:
                    if abs(cur - self.prev_joints[name]) > self.mot_thresh:
                        moving = True; break
                self.prev_joints[name] = cur
        if moving:
            self.motion_cnt += 1; self.stationary_cnt = 0
            if self.motion_cnt >= 3: self.is_walking = True
        else:
            self.stationary_cnt += 1; self.motion_cnt = 0
            if self.stationary_cnt >= 10: self.is_walking = False
        self.detector.set_walking_mode(self.is_walking)

    def image_cb(self, msg: Image):
        try:
            # ── RESIZE ke resolusi kerja dulu (hemat 4x komputasi) ──
            full_img = self.br.imgmsg_to_cv2(msg, 'bgr8')
            img = cv2.resize(full_img, (PROC_WIDTH, PROC_HEIGHT),
                             interpolation=cv2.INTER_LINEAR)

            mask       = self.detector.get_white_mask(img)
            curr_lines = self.detector.detect_lines(mask)
            stamp      = msg.header.stamp
            ts         = stamp.sec + stamp.nanosec * 1e-9

            dt = 0.0
            if self.prev_ts is not None:
                dt = max(0.001, min(ts - self.prev_ts, 0.5))

            # ── Translasi (optical flow) ──
            vx, vy, conf = 0.0, 0.0, 0.0
            if self.prev_lines is not None and dt > 0:
                vx, vy, _, conf = self.detector.estimate_flow(
                    self.prev_lines, curr_lines, dt, self.imu_angular_vel)
                vx, vy, _ = self.detector.smooth_velocity(vx, vy, 0.0)

            # ── VP Yaw (hanya tiap N frame) ──
            if self.use_vp_yaw and (self.frame_cnt % self.vp_every_n == 0):
                self.vp_yaw, self.vp_conf = \
                    self.vp_det.estimate_yaw_from_lines(curr_lines)
                if self.vp_conf > 0.3:
                    self.vp_used_cnt += 1

            # ── Fusi Yaw: IMU dominan, VP koreksi drift ──
            if self.has_imu:
                if self.use_vp_yaw and self.vp_conf > 0.2:
                    vp_w = self.vp_conf * 0.35  # VP max 35%
                    self.yaw = (1 - vp_w) * self.imu_yaw + vp_w * self.vp_yaw
                else:
                    self.yaw = self.imu_yaw

            # ── Motion gating ──
            if not self.is_walking:
                vx *= 0.1; vy *= 0.1

            eff_thresh = self.min_motion / max(conf, 0.1)
            if math.sqrt(vx**2 + vy**2) < eff_thresh:
                vx = vy = 0.0

            # ── EMA smoothing ──
            self.vx_s = self.alpha * self.vx_s + (1 - self.alpha) * vx
            self.vy_s = self.alpha * self.vy_s + (1 - self.alpha) * vy

            # ── Integrasi posisi ──
            if dt > 0:
                cy = math.cos(self.yaw); sy = math.sin(self.yaw)
                self.x += self.vx_s * dt * cy - self.vy_s * dt * sy
                self.y += self.vx_s * dt * sy + self.vy_s * dt * cy

            self.prev_lines = curr_lines
            self.prev_ts    = ts
            self.frame_cnt += 1

            self._pub_tf(stamp)
            self._pub_odom(stamp, conf, self.vp_conf)

        except Exception as e:
            self.get_logger().error(f"[VIO] {e}", throttle_duration_sec=5.0)

    def _pub_tf(self, stamp):
        t                         = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = self.odom_frame
        t.child_frame_id          = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = math.sin(self.yaw / 2.0)
        t.transform.rotation.w    = math.cos(self.yaw / 2.0)
        self.tf_bc.sendTransform(t)

    def _pub_odom(self, stamp, flow_conf=1.0, vp_conf=0.0):
        xy_scale  = 1.0 + (1.0 - max(flow_conf, 0.1)) * 3.0
        yaw_scale = max(0.5, 1.0 - vp_conf * 0.5)
        odom                         = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = self.odom_frame
        odom.child_frame_id          = self.base_frame
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x    = self.vx_s
        odom.twist.twist.linear.y    = self.vy_s
        odom.twist.twist.angular.z   = self.vyaw
        odom.pose.covariance[0]      = self.cov_xy  * xy_scale
        odom.pose.covariance[7]      = self.cov_xy  * xy_scale
        odom.pose.covariance[35]     = self.cov_yaw * yaw_scale
        odom.twist.covariance[0]     = self.tcov_xy
        odom.twist.covariance[7]     = self.tcov_xy
        odom.twist.covariance[35]    = self.tcov_yaw
        self.odom_pub.publish(odom)

    def log_motion(self):
        self.get_logger().info(
            f"[VIO] walk={self.is_walking} | imu_av={self.imu_angular_vel:.2f} | "
            f"dz={self.detector.current_dead_zone:.1f}px | "
            f"vp_conf={self.vp_conf:.2f} | yaw={math.degrees(self.yaw):.1f}° | "
            f"vel=({self.vx_s:.3f},{self.vy_s:.3f})",
            throttle_duration_sec=1.0)

    def log_stats(self):
        self.get_logger().info(
            f"[VIO] frames={self.frame_cnt} | vp_used={self.vp_used_cnt} | "
            f"pos=({self.x:.3f},{self.y:.3f}) | yaw={math.degrees(self.yaw):.1f}° | "
            f"has_imu={self.has_imu}")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(VIONodeV3())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
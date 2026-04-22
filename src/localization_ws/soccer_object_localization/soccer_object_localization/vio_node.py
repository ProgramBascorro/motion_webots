#!/usr/bin/env python3
"""
VIO Node v4 - Standalone (tanpa AMCL)
Upgrade dari v3:

1. Sparse Optical Flow (Lucas-Kanade) menggantikan centroid shift
   - Track titik individual pada garis lapangan
   - RANSAC-style outlier rejection (median ± IQR)
   - Lebih akurat karena tidak terpengaruh garis masuk/keluar frame

2. Resize frame SEBELUM processing (1280x720 → 320x240)
   - Hemat ~4x komputasi OpenCV
   - Scale factor otomatis disesuaikan

3. VP detection hanya tiap N frame (tidak setiap frame)

4. Publisher tambahan untuk RViz tanpa AMCL:
   - /vio_pose  (PoseWithCovarianceStamped)
   - /vio_path  (Path — trajektori robot)

5. Static TF map→odom (identity) agar RViz bisa render di frame map
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, Imu, JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import (TransformStamped, PoseStamped,
                                PoseWithCovarianceStamped)
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import math
from collections import deque

# ── Konstanta fisik OP3 ──────────────────────────────────────
OP3_MAX_SPEED       = 0.20
IMU_SHAKE_THRESHOLD = 0.8
PROC_W, PROC_H      = 320, 240


# ════════════════════════════════════════════════════════════
# SPARSE OPTICAL FLOW — Lucas-Kanade
# Menggantikan centroid shift dari v3
# ════════════════════════════════════════════════════════════
class SparseFlowEstimator:
    """
    Estimasi kecepatan menggunakan Lucas-Kanade sparse optical flow
    pada feature points dari area garis lapangan (white mask).

    Keunggulan vs centroid shift (v3):
    - Track titik individual → tidak terpengaruh garis masuk/keluar frame
    - RANSAC-style rejection via median+IQR → noise lebih sedikit
    - Confidence score dari jumlah inlier (bukan hanya jumlah garis)
    """

    def __init__(self, scale: float):
        self.scale = scale

        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        self.feature_params = dict(
            maxCorners=80,
            qualityLevel=0.2,
            minDistance=7,
            blockSize=7
        )

        self.prev_gray        = None
        self.prev_pts         = None
        self.velocity_history = deque(maxlen=5)

    def update(self, gray_curr, mask, dt, imu_av=0.0):
        """Returns: (vx, vy, confidence)"""
        if self.prev_gray is None or self.prev_pts is None or dt <= 0:
            self._redetect(gray_curr, mask)
            return 0.0, 0.0, 0.0

        if len(self.prev_pts) < 4:
            self._redetect(gray_curr, mask)
            return 0.0, 0.0, 0.0

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray_curr, self.prev_pts, None, **self.lk_params
        )

        if curr_pts is None:
            self._redetect(gray_curr, mask)
            return 0.0, 0.0, 0.0

        good_prev = self.prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        if len(good_prev) < 4:
            self._redetect(gray_curr, mask)
            return 0.0, 0.0, 0.0

        flow   = good_curr - good_prev
        dx_all = flow[:, 0]
        dy_all = flow[:, 1]

        # RANSAC-style outlier rejection
        def robust_median(arr):
            med = np.median(arr)
            iqr = np.percentile(arr, 75) - np.percentile(arr, 25)
            inliers = arr[np.abs(arr - med) < 1.5 * max(iqr, 0.5)]
            return float(np.median(inliers)) if len(inliers) > 0 else float(med)

        dx_px = robust_median(dx_all)
        dy_px = robust_median(dy_all)

        # Dead zone
        if abs(dx_px) < 1.5: dx_px = 0.0
        if abs(dy_px) < 1.5: dy_px = 0.0

        vx, vy = 0.0, 0.0
        if dx_px != 0.0 or dy_px != 0.0:
            vx = max(-OP3_MAX_SPEED, min(OP3_MAX_SPEED, -dy_px * self.scale / dt))
            vy = max(-OP3_MAX_SPEED, min(OP3_MAX_SPEED, -dx_px * self.scale / dt))

        # IMU shake attenuation
        if imu_av > IMU_SHAKE_THRESHOLD:
            f = max(0.1, 1.0 - (imu_av - IMU_SHAKE_THRESHOLD) * 0.5)
            vx *= f; vy *= f

        # Confidence dari jumlah inlier
        inlier_ratio = len(good_prev) / max(len(self.prev_pts), 1)
        conf = min(len(good_prev) / 20.0, 1.0) * inlier_ratio

        # Temporal median smoothing
        self.velocity_history.append((vx, vy))
        if len(self.velocity_history) >= 3:
            vx = float(np.median([v[0] for v in self.velocity_history]))
            vy = float(np.median([v[1] for v in self.velocity_history]))

        self.prev_gray = gray_curr.copy()
        self.prev_pts  = good_curr.reshape(-1, 1, 2)

        if len(self.prev_pts) < 10:
            self._redetect(gray_curr, mask)

        return vx, vy, conf

    def _redetect(self, gray, mask):
        pts = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
        self.prev_gray = gray.copy()
        self.prev_pts  = pts

    def reset(self):
        self.prev_gray = None
        self.prev_pts  = None
        self.velocity_history.clear()


# ════════════════════════════════════════════════════════════
# VANISHING POINT DETECTOR — dari v3, dioptimalkan
# ════════════════════════════════════════════════════════════
class VanishingPointDetector:

    def __init__(self):
        self.yaw_history = deque(maxlen=7)
        self.yaw_offset  = None
        self.confidence  = 0.0
        self._cache_yaw  = 0.0
        self._cache_conf = 0.0

    def estimate_yaw(self, lines) -> tuple:
        if lines is None or len(lines) < 4:
            self._cache_conf *= 0.9
            return self._cache_yaw, self._cache_conf

        h_angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 == x1: continue
            angle  = math.atan2(y2 - y1, x2 - x1)
            if angle < 0: angle += math.pi
            length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            if length < 20: continue
            if (abs(angle) < math.radians(35) or
                    abs(angle - math.pi) < math.radians(35)):
                h_angles.append((angle, length))

        if len(h_angles) < 3:
            self._cache_conf *= 0.9
            return self._cache_yaw, self._cache_conf

        sin_s = sum(math.sin(2*a) * l for a, l in h_angles)
        cos_s = sum(math.cos(2*a) * l for a, l in h_angles)
        mean_a = math.atan2(sin_s, cos_s) / 2.0
        conf   = min(len(h_angles) / 6.0, 1.0)

        self.yaw_history.append((mean_a, conf))

        if len(self.yaw_history) >= 4:
            stable = float(np.median([a for a, _ in self.yaw_history]))
            sconf  = float(np.mean([c for _, c in self.yaw_history]))

            if self.yaw_offset is None:
                self.yaw_offset = stable
                self._cache_yaw, self._cache_conf = 0.0, sconf
                return 0.0, sconf

            yaw = stable - self.yaw_offset
            while yaw >  math.pi: yaw -= 2*math.pi
            while yaw < -math.pi: yaw += 2*math.pi

            self._cache_yaw  = yaw
            self._cache_conf = sconf
            self.confidence  = sconf
            return yaw, sconf

        return self._cache_yaw, self._cache_conf * 0.9


# ════════════════════════════════════════════════════════════
# YAW FUSION — dari v3, dipertahankan
# ════════════════════════════════════════════════════════════
class YawFusion:

    def __init__(self):
        self.history = deque(maxlen=10)

    def fuse(self, imu_yaw, vp_yaw, vp_conf) -> float:
        vp_w      = vp_conf * 0.4
        candidate = (1.0 - vp_w) * imu_yaw + vp_w * vp_yaw
        self.history.append(candidate)
        if len(self.history) >= 3:
            return float(np.median(list(self.history)[-5:]))
        return candidate


# ════════════════════════════════════════════════════════════
# HELPER
# ════════════════════════════════════════════════════════════
def get_white_mask(image):
    hsv       = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    grass     = cv2.inRange(hsv, (35, 40, 0), (85, 255, 255))
    non_grass = cv2.bitwise_not(grass)
    gray      = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, white  = cv2.threshold(gray, 165, 255, cv2.THRESH_BINARY)
    result    = cv2.bitwise_and(white, white, mask=non_grass)
    h, w      = result.shape
    roi       = np.zeros_like(result)
    roi[int(h*0.35):int(h*0.92), :] = 255
    return cv2.bitwise_and(result, result, mask=roi)

def detect_lines(white_mask):
    edges = cv2.Canny(white_mask, 50, 150)
    return cv2.HoughLinesP(
        edges, rho=1, theta=np.pi/180, threshold=50,
        minLineLength=12, maxLineGap=20
    )


# ════════════════════════════════════════════════════════════
# MAIN NODE
# ════════════════════════════════════════════════════════════
class VIONodeV4(Node):

    def __init__(self):
        super().__init__('vio_node')

        # ── Parameters ──────────────────────────────────────
        self.declare_parameter('base_frame',        'cam_link')
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('focal_length',      900.0)
        self.declare_parameter('camera_height',     0.475)
        self.declare_parameter('camera_tilt',      -0.349)
        self.declare_parameter('orig_image_width',  1280)
        self.declare_parameter('orig_image_height', 720)
        self.declare_parameter('smoothing_alpha',   0.4)
        self.declare_parameter('min_visual_motion', 0.008)
        self.declare_parameter('motion_threshold',  0.005)
        self.declare_parameter('use_vp_yaw',        True)
        self.declare_parameter('vp_every_n_frames', 5)
        self.declare_parameter('pose_cov_xy',       0.3)
        self.declare_parameter('pose_cov_yaw',      0.15)
        self.declare_parameter('publish_path',      True)
        self.declare_parameter('path_max_poses',    300)

        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame  = self.get_parameter('map_frame').value
        self.alpha      = self.get_parameter('smoothing_alpha').value
        self.min_motion = self.get_parameter('min_visual_motion').value
        self.mot_thresh = self.get_parameter('motion_threshold').value
        self.use_vp_yaw = self.get_parameter('use_vp_yaw').value
        self.vp_every_n = self.get_parameter('vp_every_n_frames').value
        self.cov_xy     = self.get_parameter('pose_cov_xy').value
        self.cov_yaw    = self.get_parameter('pose_cov_yaw').value
        self.pub_path   = self.get_parameter('publish_path').value
        self.path_max   = self.get_parameter('path_max_poses').value
        orig_w = self.get_parameter('orig_image_width').value
        orig_h = self.get_parameter('orig_image_height').value

        # Scale disesuaikan ke resolusi PROC
        focal = self.get_parameter('focal_length').value
        tilt  = self.get_parameter('camera_tilt').value
        sx    = PROC_W / orig_w
        sy    = PROC_H / orig_h
        f_proc = focal * (sx + sy) / 2.0
        self.SCALE = (2.5 / max(math.cos(abs(tilt)), 0.5)) / f_proc

        # ── State ────────────────────────────────────────────
        self.x = self.y = self.yaw = 0.0
        self.vx_s = self.vy_s = 0.0
        self.prev_ts = None

        # IMU
        self.imu_yaw      = 0.0
        self.imu_init_yaw = None
        self.has_imu      = False
        self.imu_av       = 0.0

        # VP cache
        self.vp_yaw  = 0.0
        self.vp_conf = 0.0

        # Joint motion
        self.is_walking     = False
        self.prev_joints    = {}
        self.stationary_cnt = 0
        self.motion_cnt     = 0

        # ── Components ───────────────────────────────────────
        self.flow_est  = SparseFlowEstimator(self.SCALE)
        self.vp_det    = VanishingPointDetector()
        self.yaw_fuser = YawFusion()

        # ── ROS interfaces ───────────────────────────────────
        self.br        = CvBridge()
        self.tf_bc     = TransformBroadcaster(self)
        self.static_bc = StaticTransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry,                  '/odom',     10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/vio_pose', 10)
        self.path_pub = self.create_publisher(Path,                      '/vio_path', 10)

        self.path_msg              = Path()
        self.path_msg.header.frame_id = self.odom_frame

        # Static TF map→odom (identity) agar RViz bisa render di frame map
        self._pub_static_map_odom()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.create_subscription(
            Image,      '/robotis_op3/camera/image_raw', self.image_cb, qos)
        self.create_subscription(
            Imu,        '/robotis_op3/imu',              self.imu_cb,   qos)
        self.create_subscription(
            Imu,        '/imu/data',                     self.imu_cb,   qos)
        self.create_subscription(
            JointState, '/robotis_op3/joint_states',     self.joint_cb, 10)

        self.frame_cnt   = 0
        self.vp_used_cnt = 0
        self.create_timer(5.0, self.log_stats)
        self.create_timer(1.0, self.log_motion)

        self.get_logger().info("=" * 65)
        self.get_logger().info("✅ VIO Node v4 (Standalone) started")
        self.get_logger().info(
            f"   Processing @ {PROC_W}x{PROC_H}  SCALE={self.SCALE:.6f} m/px")
        self.get_logger().info(
            f"   use_vp={self.use_vp_yaw} | vp_every={self.vp_every_n}")
        self.get_logger().info(
            f"   alpha={self.alpha} | cov_xy={self.cov_xy} | cov_yaw={self.cov_yaw}")
        self.get_logger().info(
            f"   Topics: /odom  /vio_pose  /vio_path")
        self.get_logger().info("=" * 65)

    # ── Static map→odom ─────────────────────────────────────
    def _pub_static_map_odom(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id  = self.odom_frame
        t.transform.rotation.w = 1.0
        self.static_bc.sendTransform(t)
        self.get_logger().info(
            f"[VIO] Static TF: {self.map_frame} → {self.odom_frame} (identity)")

    # ── IMU ──────────────────────────────────────────────────
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
        self.imu_av = math.sqrt(av.x**2 + av.y**2 + av.z**2)

    # ── Joint State ──────────────────────────────────────────
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

    # ── Image (main loop) ────────────────────────────────────
    def image_cb(self, msg: Image):
        try:
            # Resize dulu — hemat 4x CPU
            full = self.br.imgmsg_to_cv2(msg, 'bgr8')
            img  = cv2.resize(full, (PROC_W, PROC_H), interpolation=cv2.INTER_LINEAR)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            mask = get_white_mask(img)

            stamp = msg.header.stamp
            ts    = stamp.sec + stamp.nanosec * 1e-9
            dt    = 0.0
            if self.prev_ts is not None:
                dt = max(0.001, min(ts - self.prev_ts, 0.5))

            # ── 1. Sparse optical flow (translasi) ──────────
            vx, vy, conf = self.flow_est.update(gray, mask, dt, self.imu_av)

            # ── 2. VP yaw (tiap N frame saja) ───────────────
            if self.use_vp_yaw and (self.frame_cnt % self.vp_every_n == 0):
                lines = detect_lines(mask)
                self.vp_yaw, self.vp_conf = self.vp_det.estimate_yaw(lines)
                if self.vp_conf > 0.3:
                    self.vp_used_cnt += 1

            # ── 3. Yaw fusion: IMU + VP ──────────────────────
            if self.has_imu:
                if self.use_vp_yaw and self.vp_conf > 0.2:
                    self.yaw = self.yaw_fuser.fuse(
                        self.imu_yaw, self.vp_yaw, self.vp_conf)
                else:
                    self.yaw = self.imu_yaw

            # ── 4. Motion gating ─────────────────────────────
            if not self.is_walking:
                vx *= 0.1; vy *= 0.1

            if math.sqrt(vx**2 + vy**2) < self.min_motion / max(conf, 0.1):
                vx = vy = 0.0

            # ── 5. EMA smoothing ─────────────────────────────
            self.vx_s = self.alpha * self.vx_s + (1 - self.alpha) * vx
            self.vy_s = self.alpha * self.vy_s + (1 - self.alpha) * vy

            # ── 6. Integrasi posisi ──────────────────────────
            if dt > 0:
                cy = math.cos(self.yaw); sy = math.sin(self.yaw)
                self.x += self.vx_s * dt * cy - self.vy_s * dt * sy
                self.y += self.vx_s * dt * sy + self.vy_s * dt * cy

            self.prev_ts    = ts
            self.frame_cnt += 1

            # ── 7. Publish ───────────────────────────────────
            self._pub_tf(stamp)
            self._pub_odom(stamp, conf)
            self._pub_pose(stamp, conf)
            if self.pub_path:
                self._pub_path(stamp)

        except Exception as e:
            self.get_logger().error(f"[VIO] {e}", throttle_duration_sec=5.0)

    # ── Publishers ───────────────────────────────────────────
    def _qz_qw(self):
        return math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)

    def _pub_tf(self, stamp):
        qz, qw = self._qz_qw()
        t = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = self.odom_frame
        t.child_frame_id          = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_bc.sendTransform(t)

    def _pub_odom(self, stamp, conf=1.0):
        qz, qw   = self._qz_qw()
        xy_scale = 1.0 + (1.0 - max(conf, 0.1)) * 2.0
        yaw_cov  = self.cov_yaw * max(0.5, 1.0 - self.vp_conf * 0.5)

        odom = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = self.odom_frame
        odom.child_frame_id          = self.base_frame
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = self.vx_s
        odom.twist.twist.linear.y    = self.vy_s
        odom.pose.covariance[0]      = self.cov_xy * xy_scale
        odom.pose.covariance[7]      = self.cov_xy * xy_scale
        odom.pose.covariance[35]     = yaw_cov
        odom.twist.covariance[0]     = 0.3
        odom.twist.covariance[7]     = 0.3
        odom.twist.covariance[35]    = 0.1
        self.odom_pub.publish(odom)

    def _pub_pose(self, stamp, conf=1.0):
        """Pose untuk visualisasi RViz — add sebagai PoseWithCovariance display."""
        qz, qw   = self._qz_qw()
        xy_scale = 1.0 + (1.0 - max(conf, 0.1)) * 2.0
        p = PoseWithCovarianceStamped()
        p.header.stamp            = stamp
        p.header.frame_id         = self.odom_frame
        p.pose.pose.position.x    = self.x
        p.pose.pose.position.y    = self.y
        p.pose.pose.orientation.z = qz
        p.pose.pose.orientation.w = qw
        p.pose.covariance[0]      = self.cov_xy * xy_scale
        p.pose.covariance[7]      = self.cov_xy * xy_scale
        p.pose.covariance[35]     = self.cov_yaw
        self.pose_pub.publish(p)

    def _pub_path(self, stamp):
        """Trajektori untuk visualisasi di RViz — tambahkan Path display."""
        qz, qw = self._qz_qw()
        ps = PoseStamped()
        ps.header.stamp       = stamp
        ps.header.frame_id    = self.odom_frame
        ps.pose.position.x    = self.x
        ps.pose.position.y    = self.y
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.path_max:
            self.path_msg.poses.pop(0)
        self.path_msg.header.stamp = stamp
        self.path_pub.publish(self.path_msg)

    # ── Debug ────────────────────────────────────────────────
    def log_motion(self):
        n_pts = len(self.flow_est.prev_pts) if self.flow_est.prev_pts is not None else 0
        self.get_logger().info(
            f"[VIO] walk={self.is_walking} | imu_av={self.imu_av:.2f} | "
            f"flow_pts={n_pts} | vp_conf={self.vp_conf:.2f} | "
            f"yaw={math.degrees(self.yaw):.1f}° | "
            f"vel=({self.vx_s:.3f},{self.vy_s:.3f}) m/s",
            throttle_duration_sec=1.0)

    def log_stats(self):
        self.get_logger().info(
            f"[VIO] frames={self.frame_cnt} | vp_used={self.vp_used_cnt} | "
            f"pos=({self.x:.3f},{self.y:.3f}) | "
            f"yaw={math.degrees(self.yaw):.1f}° | has_imu={self.has_imu}")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(VIONodeV4())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
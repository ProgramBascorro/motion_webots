#!/usr/bin/env python3
"""
joint_odom_node.py v5 — Clean AMCL Motion Model

Perubahan mendasar dari v4:
  v4: hitung displacement per langkah → publish /odom saat langkah
  v5: publish /odom SETIAP FRAME (50Hz) → AMCL dapat motion model terus-menerus

Kenapa v4 bermasalah untuk AMCL:
  AMCL butuh update /odom setiap kali scan datang (~20Hz)
  Kalau /odom hanya update saat langkah → AMCL gap update 0.5-18 detik
  AMCL DifferentialMotionModel hitung displacement ANTARA dua /odom terakhir
  Kalau gap terlalu besar → displacement salah hitung

Cara kerja v5:
  - Subscribe /joint_states dan /imu
  - SETIAP FRAME: update pose (x, y, yaw) dari akumulasi displacement
  - Publish /odom dan TF odom→base_link di SETIAP frame (timer 20Hz)
  - Displacement per langkah dihitung dari akumulasi FK saat transisi stance
  - Di antara langkah: pose tidak berubah (velocity=0) tapi tetap di-publish

Konvensi joint knee Webots OP3:
  Left  knee lurus = +~1.4 rad  (bervariasi tergantung posisi berdiri)
  Right knee lurus = -~1.4 rad
  Stance: knee_L > stance_l_min, knee_R < -stance_r_min
  Default stance_l_min = 1.0 (tunable)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from collections import deque
import numpy as np


# ── Dimensi link OP3 (meter, dari URDF) ──────────────────────
L_THIGH = 0.093
L_SHANK = 0.093
L_FOOT  = 0.035

STEP_MIN = 0.002   # m — minimum akumulasi valid (2mm)
STEP_MAX = 0.15    # m — maksimum akumulasi valid (15cm, sanity check)

JOINT_L_HIP  = 'l_hip_pitch'
JOINT_L_KNEE = 'l_knee'
JOINT_L_ANK  = 'l_ank_pitch'
JOINT_R_HIP  = 'r_hip_pitch'
JOINT_R_KNEE = 'r_knee'
JOINT_R_ANK  = 'r_ank_pitch'

REQUIRED_JOINTS = [
    JOINT_L_HIP, JOINT_L_KNEE, JOINT_L_ANK,
    JOINT_R_HIP, JOINT_R_KNEE, JOINT_R_ANK,
]


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q


class JointOdomNode(Node):

    def __init__(self):
        super().__init__('joint_odom_node')

        # ── Parameters ───────────────────────────────────────
        self.declare_parameter('base_frame',      'base_link')
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('use_imu_yaw',     True)
        self.declare_parameter('step_scale',       0.8)
        self.declare_parameter('stance_l_min',     1.0)
        self.declare_parameter('stance_r_min',     1.0)
        # Covariance untuk /odom — tuning utama untuk AMCL
        # Besar = AMCL lebih percaya scan untuk koreksi
        # Kecil = AMCL lebih percaya odom
        self.declare_parameter('cov_xy',           0.05)
        self.declare_parameter('cov_yaw',          0.02)
        # Publish rate (Hz) — harus >= AMCL update rate
        self.declare_parameter('publish_rate',     10.0)
        # Sign konvensi arah maju:
        # +1.0 = maju menghasilkan +x (default standar REP-103)
        # -1.0 = flip — kalau di log odom bergerak NEGATIF saat maju
        self.declare_parameter('forward_sign',    -1.0)
        # Offset yaw IMU dalam derajat — koreksi kalau IMU dipasang miring
        # 0.0 = tidak ada koreksi, 90.0/-90.0/180.0 untuk kalibrasi arah
        self.declare_parameter('imu_yaw_offset_deg', 0.0)

        self.base_frame   = self.get_parameter('base_frame').value
        self.odom_frame   = self.get_parameter('odom_frame').value
        self.use_imu_yaw  = self.get_parameter('use_imu_yaw').value
        self.step_scale   = self.get_parameter('step_scale').value
        self.stance_l_min = self.get_parameter('stance_l_min').value
        self.stance_r_min = self.get_parameter('stance_r_min').value
        self.cov_xy       = self.get_parameter('cov_xy').value
        self.cov_yaw      = self.get_parameter('cov_yaw').value
        self.pub_rate     = self.get_parameter('publish_rate').value
        self.fwd_sign        = self.get_parameter('forward_sign').value
        yaw_offset_deg       = self.get_parameter('imu_yaw_offset_deg').value
        self.imu_yaw_offset  = math.radians(yaw_offset_deg)

        # ── Pose state ───────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0   # dari IMU (kalau use_imu_yaw=True)

        # ── IMU ──────────────────────────────────────────────
        self.imu_yaw       = 0.0
        self.imu_init_yaw  = None
        self.has_imu       = False

        # ── Joint / Stance ───────────────────────────────────
        self.joint_pos   = {}
        self.joint_found = False

        # Stance state
        self.l_stance = False
        self.r_stance = False

        # FK foot position (sagittal x)
        self.l_foot_prev = None
        self.r_foot_prev = None

        # Akumulasi displacement selama fase stance
        self.l_accum = 0.0
        self.r_accum = 0.0

        # Stats
        self.step_count = 0
        self.step_hist  = deque(maxlen=30)

        # Velocity (untuk covariance dan publish)
        self.vx = 0.0
        self.vy = 0.0

        # ── ROS ──────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )

        self.create_subscription(Imu, '/robotis_op3/imu',
                                  self.imu_cb, qos_be)
        self.create_subscription(JointState, '/robotis_op3/joint_states',
                                  self.joint_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer publish — setiap frame tanpa menunggu langkah
        period = 1.0 / self.pub_rate
        self.create_timer(period, self.publish_odom)

        # Timer log status
        self.create_timer(1.0, self.log_status)

        # Cek CPU estimate
        ops_per_sec = 800 * 30 * 10  # max_particles * beams * hz
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Joint Odometry Node v5 started (Clean AMCL mode)")
        self.get_logger().info(
            f"   Joints: {REQUIRED_JOINTS}")
        self.get_logger().info(
            f"   forward_sign={self.fwd_sign} "
            f"(+1=maju→+x, -1=flip jika odom negatif saat maju)")
        self.get_logger().info(
            f"   L={L_THIGH}/{L_SHANK}/{L_FOOT}m | "
            f"stance_L>{self.stance_l_min} | stance_R<-{self.stance_r_min}")
        self.get_logger().info(
            f"   cov_xy={self.cov_xy} | cov_yaw={self.cov_yaw} | "
            f"publish_rate={self.pub_rate}Hz")
        self.get_logger().info("=" * 60)

    # ── IMU callback ─────────────────────────────────────────

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        raw_yaw = 2.0 * math.atan2(q.z, q.w)

        if self.imu_init_yaw is None:
            self.imu_init_yaw = raw_yaw
            self.get_logger().info(
                f"[IMU] init yaw={math.degrees(raw_yaw):.1f}°")

        self.imu_yaw = raw_yaw - self.imu_init_yaw + self.imu_yaw_offset
        self.has_imu = True

        if self.use_imu_yaw:
            self.yaw = self.imu_yaw

    # ── FK helper ────────────────────────────────────────────

    def _fk_foot_x(self, hip, knee, ank) -> float:
        """
        FK sagittal: posisi kaki dalam arah maju (x) relatif ke base.
        Webots OP3: knee konvensi terbalik dari standar URDF.
        """
        x = (L_THIGH * math.sin(hip)
             + L_SHANK * math.sin(hip + knee)
             + L_FOOT  * math.sin(hip + knee + ank))
        return x

    def _is_l_stance(self, knee_l: float) -> bool:
        return knee_l > self.stance_l_min

    def _is_r_stance(self, knee_r: float) -> bool:
        return knee_r < -self.stance_r_min

    # ── Joint callback ───────────────────────────────────────

    def joint_cb(self, msg: JointState):
        # Bangun dict nama→posisi
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

        # Cek apakah semua joint sudah ada
        if not self.joint_found:
            if all(j in self.joint_pos for j in REQUIRED_JOINTS):
                self.joint_found = True
                self.get_logger().info("✅ [JointOdom] Semua joint ditemukan!")
            else:
                return

        j = self.joint_pos

        # FK posisi kaki dalam sagittal x
        # Kaki kiri: knee positif saat stance → FK positif → OK
        # Kaki kanan: knee NEGATIF saat stance → FK negatif → harus di-negate
        # agar kedua kaki punya konvensi yang sama (maju = positif)
        l_fx =  self._fk_foot_x(j[JOINT_L_HIP], j[JOINT_L_KNEE], j[JOINT_L_ANK])
        r_fx = -self._fk_foot_x(j[JOINT_R_HIP], j[JOINT_R_KNEE], j[JOINT_R_ANK])

        # Stance detection
        l_now = self._is_l_stance(j[JOINT_L_KNEE])
        r_now = self._is_r_stance(j[JOINT_R_KNEE])

        # ── Akumulasi displacement selama fase stance ─────────
        if l_now and self.l_foot_prev is not None:
            self.l_accum += l_fx - self.l_foot_prev

        if r_now and self.r_foot_prev is not None:
            self.r_accum += r_fx - self.r_foot_prev

        # ── Deteksi langkah: saat stance BERAKHIR ─────────────
        step_dx = 0.0
        stepped = False

        if self.l_stance and not l_now:
            # Kaki kiri baru angkat
            candidate = self.l_accum * self.step_scale
            self.get_logger().debug(
                f"[Step-L] accum={self.l_accum:.4f} "
                f"dx={candidate:.4f}m "
                f"valid={STEP_MIN < abs(candidate) < STEP_MAX}")
            if STEP_MIN < abs(candidate) < STEP_MAX:
                step_dx += candidate
                self.step_hist.append(abs(candidate))
                stepped = True
            self.l_accum = 0.0

        if self.r_stance and not r_now:
            # Kaki kanan baru angkat
            candidate = self.r_accum * self.step_scale
            self.get_logger().debug(
                f"[Step-R] accum={self.r_accum:.4f} "
                f"dx={candidate:.4f}m "
                f"valid={STEP_MIN < abs(candidate) < STEP_MAX}")
            if STEP_MIN < abs(candidate) < STEP_MAX:
                step_dx += candidate
                self.step_hist.append(abs(candidate))
                stepped = True
            self.r_accum = 0.0

        if stepped:
            # Jika dua kaki, rata-rata
            if self.l_stance and not l_now and self.r_stance and not r_now:
                step_dx /= 2.0
            # Terapkan forward_sign untuk koreksi konvensi arah
            step_dx *= self.fwd_sign
            # Simpan posisi sebelum update untuk log diagnostik
            prev_x, prev_y = self.x, self.y
            # Update posisi dalam frame odom
            self.x   += step_dx * math.cos(self.yaw)
            self.y   += step_dx * math.sin(self.yaw)
            self.vx   = step_dx * math.cos(self.yaw)
            self.vy   = step_dx * math.sin(self.yaw)
            self.step_count += 1
            # Log INFO setiap 5 langkah — untuk diagnosa arah tanpa debug mode
            if self.step_count % 5 == 0:
                self.get_logger().info(
                    f"[OdomDir] step#{self.step_count} "
                    f"dx={step_dx:+.4f}m yaw={math.degrees(self.yaw):.1f}° "
                    f"pos: ({prev_x:.3f},{prev_y:.3f}) → ({self.x:.3f},{self.y:.3f}) "
                    f"[fwd_sign={self.fwd_sign:+.0f}]"
                )
        else:
            # Decay velocity saat tidak ada langkah
            self.vx *= 0.5
            self.vy *= 0.5

        # Reset akumulasi saat stance dimulai
        if not self.l_stance and l_now:
            self.l_accum    = 0.0
            self.l_foot_prev = l_fx
        if not self.r_stance and r_now:
            self.r_accum    = 0.0
            self.r_foot_prev = r_fx

        # Update foot prev & stance
        self.l_foot_prev = l_fx if l_now else None
        self.r_foot_prev = r_fx if r_now else None
        self.l_stance    = l_now
        self.r_stance    = r_now

    # ── Publish /odom setiap frame ───────────────────────────

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        # ── TF odom → base_link ──────────────────────────────
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        q = yaw_to_quat(self.yaw)
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)

        # ── Odometry message ─────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = q

        # Velocity (estimasi dari langkah terakhir)
        odom.twist.twist.linear.x  = self.vx
        odom.twist.twist.linear.y  = self.vy
        odom.twist.twist.angular.z = 0.0

        # Covariance — kunci untuk AMCL
        # Diagonal: [x, y, z, roll, pitch, yaw]
        c_p = odom.pose.covariance
        c_p[0]  = self.cov_xy    # var_x
        c_p[7]  = self.cov_xy    # var_y
        c_p[14] = 0.001
        c_p[21] = 0.001
        c_p[28] = 0.001
        c_p[35] = self.cov_yaw   # var_yaw

        c_t = odom.twist.covariance
        c_t[0]  = self.cov_xy * 4
        c_t[7]  = self.cov_xy * 4
        c_t[35] = self.cov_yaw * 4

        self.odom_pub.publish(odom)

    # ── Log status setiap detik ──────────────────────────────

    def log_status(self):
        avg = float(np.mean(self.step_hist)) if self.step_hist else 0.0
        self.get_logger().info(
            f"[JointOdom] pos=({self.x:.3f},{self.y:.3f}) | "
            f"yaw={math.degrees(self.yaw):.1f}° | "
            f"steps={self.step_count} | avg_step={avg:.4f}m | "
            f"stance=L{int(self.l_stance)}R{int(self.r_stance)} | "
            f"knee=L{self.joint_pos.get(JOINT_L_KNEE, 0):.3f}"
            f"R{self.joint_pos.get(JOINT_R_KNEE, 0):.3f}rad")


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(JointOdomNode())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
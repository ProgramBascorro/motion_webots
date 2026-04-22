#!/usr/bin/env python3
"""
crossing_amcl_constraint.py — Crossing Landmark Constraint for AMCL  [v1.0]
============================================================================
Referensi: Paper 2 (Schulz & Behnke, 2012) — Section 3, Eq.5

PERAN NODE INI:
  crossing_detector menghasilkan /detected_crossings (PoseArray).
  Setiap crossing terdeteksi berisi:
    pose.position.x  = world_x (m, robot-relative → dikonversi ke map frame)
    pose.position.y  = world_y (m)
    pose.position.z  = tipe (2=L, 3=T, 4=X)
    pose.orientation.w = confidence

  Node ini membandingkan crossing yang terdeteksi dengan peta crossing
  lapangan yang sudah diketahui (hardcoded dari dimensi lapangan),
  menghitung likelihood constraint, dan publish /initialpose ke EKF
  hanya ketika ada crossing yang MATCH dengan peta.

MODEL LIKELIHOOD (Paper 2 Eq.5):
  p(z | x) ∝ exp( -(d_est - d_obs)² / [2*(σ_d + λ*d_obs)²]
                   -(β_est - β_obs)² / [2*σ_β²] )

  di mana:
    d_obs   = jarak crossing yang terdeteksi dari robot (observasi)
    d_est   = jarak crossing peta yang paling dekat (estimasi dari pose AMCL)
    β_obs   = bearing crossing yang terdeteksi
    β_est   = bearing crossing dari peta
    σ_d     = 0.2m (deviasi jarak dasar)
    λ       = 0.05 (faktor proporsional — error lebih besar = jarak lebih jauh)
    σ_β     = 0.15 rad

CARA KERJA INTEGRASI:
  1. Subscribe /detected_crossings + /odometry/filtered (pose robot saat ini)
  2. Untuk setiap crossing terdeteksi:
     a. Hitung jarak (d_obs) dan bearing (β_obs) dari robot
     b. Cari crossing peta TERDEKAT yang TIPENYA COCOK
     c. Hitung d_est dan β_est berdasarkan pose AMCL sekarang
     d. Hitung likelihood score dari Eq.5
  3. Jika ada ≥1 crossing dengan likelihood tinggi (≥ min_likelihood):
     Publish /initialpose sebagai koreksi RINGAN ke EKF
     (dengan covariance yang disesuaikan dengan kualitas match)

LAPANGAN KIDSIZE RoboCup (9m × 6m):
  T-crossing penalty kiri:   (-3.1,  ±1.3) × 2
  T-crossing penalty kanan:  (+3.1,  ±1.3) × 2
  L-crossing sudut lapangan: (±4.5,  ±3.0) × 4
  X-crossing center mark:    (0.0,    0.0) × 1

IMPORTANT — SYMMETRY BREAKING:
  Lapangan sepak bola simetris ↔ (kiri-kanan) dan ↕ (atas-bawah).
  Crossing saja tidak bisa memecah simetri penuh.
  Tapi kombinasi crossing TYPE + POSISI sudah mengurangi ambiguitas:
    - X-crossing (center mark) ada hanya di (0,0) → unik
    - T-crossing ada di 4 posisi → 4-fold ambiguity
    - L-crossing ada di 4 posisi → 4-fold ambiguity
  
  Kita gunakan pose AMCL yang ada sebagai konteks:
  Match crossing ke kandidat peta yang paling KONSISTEN dengan pose sekarang.

TOPIC:
  Sub: /detected_crossings  (PoseArray dari crossing_detector)
  Sub: /odometry/filtered    (PoseWithCovarianceStamped dari EKF)
  Pub: /initialpose          (PoseWithCovarianceStamped → EKF reset ringan)
  Pub: /crossing_constraint_debug (String — debug info)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import (PoseArray, PoseWithCovarianceStamped,
                                Pose, Quaternion)
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import numpy as np
from typing import List, Tuple, Optional, NamedTuple
from dataclasses import dataclass


# ─────────────────────────────────────────────────────────────────────────────
# PETA CROSSING LAPANGAN
# ─────────────────────────────────────────────────────────────────────────────

class CrossingType:
    L = 2
    T = 3
    X = 4

@dataclass
class MapCrossing:
    x: float       # map frame (m)
    y: float       # map frame (m)
    ctype: int     # CrossingType.L/T/X
    label: str

def build_field_map(field_half_len: float = 4.5,
                    field_half_wid: float = 3.0,
                    penalty_x: float = 3.1,
                    penalty_y: float = 1.3) -> List[MapCrossing]:
    """
    Buat daftar crossing di peta lapangan kidsize.
    Koordinat: origin di tengah lapangan, x ke depan, y ke kiri.
    """
    crossings = []

    # ── X-crossing: center mark (hanya 1) ────────────────────────────────────
    crossings.append(MapCrossing(0.0, 0.0, CrossingType.X, 'center_mark'))

    # ── T-crossing: 4 sudut penalty box ──────────────────────────────────────
    for sx in [-1, 1]:
        for sy in [-1, 1]:
            crossings.append(MapCrossing(
                sx * penalty_x, sy * penalty_y,
                CrossingType.T,
                f'penalty_T_{["L","R"][sx>0]}_{["B","F"][sy>0]}'
            ))

    # ── L-crossing: 4 sudut lapangan ─────────────────────────────────────────
    for sx in [-1, 1]:
        for sy in [-1, 1]:
            crossings.append(MapCrossing(
                sx * field_half_len, sy * field_half_wid,
                CrossingType.L,
                f'corner_{["L","R"][sx>0]}_{["B","F"][sy>0]}'
            ))

    return crossings


# ─────────────────────────────────────────────────────────────────────────────
# NODE
# ─────────────────────────────────────────────────────────────────────────────

class CrossingAmclConstraint(Node):
    def __init__(self):
        super().__init__('crossing_amcl_constraint')

        # ── Parameter ────────────────────────────────────────────────────────
        self.declare_parameter('field_half_len',    4.5)
        self.declare_parameter('field_half_wid',    3.0)
        self.declare_parameter('penalty_x',         3.1)
        self.declare_parameter('penalty_y',         1.3)

        # Paper 2 Eq.5 — noise model
        self.declare_parameter('sigma_d',           0.20)   # deviasi jarak dasar (m)
        self.declare_parameter('lambda_d',          0.05)   # faktor proporsional
        self.declare_parameter('sigma_beta',        0.15)   # deviasi bearing (rad)

        # Integrasi threshold
        self.declare_parameter('min_likelihood',    0.25)   # min score per crossing
        self.declare_parameter('min_confidence',    0.50)   # min confidence dari detector
        self.declare_parameter('max_match_dist_m',  1.50)   # max jarak match ke peta (m)
        self.declare_parameter('cooldown_sec',      2.0)    # min interval antara koreksi

        # Covariance koreksi (kecil = percaya koreksi, besar = tidak)
        # Kita set besar agar tidak override AMCL terlalu agresif
        self.declare_parameter('cov_x',             0.10)   # m²
        self.declare_parameter('cov_y',             0.10)   # m²
        self.declare_parameter('cov_yaw',           0.05)   # rad²

        # Max koreksi posisi yang diperbolehkan per cycle
        self.declare_parameter('max_correction_m',  0.50)   # m

        # ── Load ─────────────────────────────────────────────────────────────
        p = self.get_parameter
        self.sigma_d        = p('sigma_d').value
        self.lambda_d       = p('lambda_d').value
        self.sigma_beta     = p('sigma_beta').value
        self.min_likelihood = p('min_likelihood').value
        self.min_conf       = p('min_confidence').value
        self.max_match_dist = p('max_match_dist_m').value
        self.cooldown       = p('cooldown_sec').value
        self.cov_x          = p('cov_x').value
        self.cov_y          = p('cov_y').value
        self.cov_yaw        = p('cov_yaw').value
        self.max_corr       = p('max_correction_m').value

        # ── Peta crossing lapangan ────────────────────────────────────────────
        self.field_map = build_field_map(
            field_half_len = p('field_half_len').value,
            field_half_wid = p('field_half_wid').value,
            penalty_x      = p('penalty_x').value,
            penalty_y      = p('penalty_y').value,
        )
        self.get_logger().info(
            f'Field map: {len(self.field_map)} crossings  '
            f'(X={sum(1 for c in self.field_map if c.ctype==CrossingType.X)}, '
            f'T={sum(1 for c in self.field_map if c.ctype==CrossingType.T)}, '
            f'L={sum(1 for c in self.field_map if c.ctype==CrossingType.L)})'
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.last_correction_time = 0.0
        self.total_corrections = 0

        # ── Sub / Pub ─────────────────────────────────────────────────────────
        self.sub_crossings = self.create_subscription(
            PoseArray, '/detected_crossings',
            self._cb_crossings, qos_profile_sensor_data)

        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered',
            self._cb_odom, qos_profile_sensor_data)

        # /initialpose: AMCL dan EKF baca ini untuk koreksi pose
        qos_latch = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos_latch)

        self.pub_debug = self.create_publisher(String, '/crossing_constraint_debug', 10)

        self.get_logger().info(
            f'CrossingAmclConstraint v1.0 started  '
            f'σ_d={self.sigma_d}  λ={self.lambda_d}  σ_β={self.sigma_beta}')

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_crossings(self, msg: PoseArray):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_correction_time < self.cooldown:
            return

        if not msg.poses:
            return

        # ── Parse deteksi ────────────────────────────────────────────────────
        detections = []
        for pose in msg.poses:
            wx    = pose.position.x   # sudah map frame dari crossing_detector
            wy    = pose.position.y
            ctype = int(pose.position.z)
            conf  = pose.orientation.w
            if conf < self.min_conf:
                continue
            # Jarak dan bearing dari robot ke crossing (map frame)
            dx = wx - self.robot_x
            dy = wy - self.robot_y
            d_obs    = math.hypot(dx, dy)
            beta_obs = math.atan2(dy, dx) - self.robot_yaw
            beta_obs = math.atan2(math.sin(beta_obs), math.cos(beta_obs))  # normalize
            detections.append({
                'wx': wx, 'wy': wy, 'ctype': ctype, 'conf': conf,
                'd_obs': d_obs, 'beta_obs': beta_obs,
            })

        if not detections:
            return

        # ── Match ke peta ────────────────────────────────────────────────────
        best_score  = 0.0
        best_dx_corr = 0.0
        best_dy_corr = 0.0
        n_matched   = 0
        debug_lines = []

        for det in detections:
            # Cari crossing peta dengan tipe yang sama, yang paling dekat
            # ke posisi deteksi DAN paling konsisten dengan pose robot
            candidates = [c for c in self.field_map if c.ctype == det['ctype']]
            if not candidates:
                continue

            best_map_crossing = None
            best_map_score    = 0.0

            for mc in candidates:
                # Estimasi: berapa jauh crossing ini dari robot menurut peta
                dx_map = mc.x - self.robot_x
                dy_map = mc.y - self.robot_y
                d_est    = math.hypot(dx_map, dy_map)
                beta_est = math.atan2(dy_map, dx_map) - self.robot_yaw
                beta_est = math.atan2(math.sin(beta_est), math.cos(beta_est))

                # Jangan match crossing yang terlalu jauh
                if d_est > self.max_match_dist:
                    continue

                # Paper 2 Eq.5: likelihood
                sigma_d_total = self.sigma_d + self.lambda_d * det['d_obs']
                diff_d    = det['d_obs'] - d_est
                diff_beta = det['beta_obs'] - beta_est
                diff_beta = math.atan2(math.sin(diff_beta), math.cos(diff_beta))

                score = math.exp(
                    -0.5 * (diff_d / sigma_d_total)**2
                    -0.5 * (diff_beta / self.sigma_beta)**2
                )

                if score > best_map_score:
                    best_map_score    = score
                    best_map_crossing = mc

            if best_map_crossing is None or best_map_score < self.min_likelihood:
                continue

            # Koreksi posisi: geser robot ke arah konsistensi
            # Jika robot saat ini di (rx, ry) dan crossing terdeteksi di (wx, wy),
            # tapi crossing peta ada di (mc.x, mc.y),
            # maka error = (mc.x - wx, mc.y - wy)
            # Tapi kita tidak bisa langsung koreksi penuh — cukup fraksi
            err_x = best_map_crossing.x - det['wx']
            err_y = best_map_crossing.y - det['wy']

            # Clamp error
            err_mag = math.hypot(err_x, err_y)
            if err_mag > self.max_corr:
                scale = self.max_corr / err_mag
                err_x *= scale
                err_y *= scale

            # Weight by score
            best_dx_corr += err_x * best_map_score
            best_dy_corr += err_y * best_map_score
            best_score    = max(best_score, best_map_score)
            n_matched    += 1

            debug_lines.append(
                f'{best_map_crossing.label}  '
                f'score={best_map_score:.2f}  '
                f'err=({err_x:.2f},{err_y:.2f})'
            )

        if n_matched == 0 or best_score < self.min_likelihood:
            return

        # Normalisasi koreksi
        best_dx_corr /= n_matched
        best_dy_corr /= n_matched

        # ── Publish /initialpose ──────────────────────────────────────────────
        corr_x = self.robot_x + best_dx_corr
        corr_y = self.robot_y + best_dy_corr

        pw = PoseWithCovarianceStamped()
        pw.header.frame_id = 'map'
        pw.header.stamp    = self.get_clock().now().to_msg()

        pw.pose.pose.position.x = corr_x
        pw.pose.pose.position.y = corr_y
        pw.pose.pose.position.z = 0.0

        # Yaw tidak dikoreksi — crossing tidak memberikan info yaw
        cy = math.cos(self.robot_yaw / 2.0)
        sy = math.sin(self.robot_yaw / 2.0)
        pw.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)

        # Covariance 6×6 (row-major)
        cov = [0.0] * 36
        cov[0]  = self.cov_x    # xx
        cov[7]  = self.cov_y    # yy
        cov[14] = 0.01          # zz (tidak berubah)
        cov[21] = 0.01          # roll
        cov[28] = 0.01          # pitch
        cov[35] = self.cov_yaw  # yaw
        pw.pose.covariance = cov

        self.pub_pose.publish(pw)
        self.last_correction_time = now
        self.total_corrections   += 1

        # ── Debug ─────────────────────────────────────────────────────────────
        debug_msg = String()
        debug_msg.data = (
            f'[CROSS_CORR #{self.total_corrections}] '
            f'n={n_matched}  best_score={best_score:.2f}  '
            f'corr=({best_dx_corr:+.2f},{best_dy_corr:+.2f})  '
            f'robot=({self.robot_x:.2f},{self.robot_y:.2f})  '
            f'→({corr_x:.2f},{corr_y:.2f})  |  '
            + '  '.join(debug_lines)
        )
        self.pub_debug.publish(debug_msg)
        self.get_logger().info(debug_msg.data, throttle_duration_sec=1.0)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = CrossingAmclConstraint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
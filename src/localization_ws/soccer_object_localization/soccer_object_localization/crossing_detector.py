#!/usr/bin/env python3
"""
crossing_detector.py — L/T/X Crossing Detection  [Fase 3, v2.4]
================================================================
Referensi: Paper 2 (Schulz & Behnke, 2012) Section 2C–2D

PIPELINE:
  /field_line_segments → interseksi (endpoint filter) → cluster
  → GAWANG FILTER → klasifikasi L/T/X → merge world → /detected_crossings

v2.4 — Tiga Perbaikan:

  FIX 1: FILTER GAWANG (goal post rejection)
    Tiang gawang = garis vertikal dengan aspect ratio sangat tinggi
    (panjang px >> lebar px, ada di area atas frame)
    Segmen yang tergolong tiang gawang dikeluarkan sebelum interseksi
    Ciri: angle hampir vertikal (70°-90°) DAN ujung atas di ROI atas (y < 30%)
    → Tiang gawang hanya di area atas, garis vertikal lapangan di bawah

  FIX 2: MERGE DISTANCE lebih besar + DEDUPLICATE world coords
    cluster_dist_px: 35 → 60px (kurangi cluster padat)
    Setelah back-project: merge crossing yang world-dist < 0.25m
    → Hanya 1 crossing per lokasi fisik

  FIX 3: KLASIFIKASI L/T/X diperbaiki
    Masalah sebelumnya: semua jadi X karena n_lines dihitung dari segmen
    yang di-merge oleh cluster, bukan dari sudut yang benar-benar berbeda.
    
    Fix: hitung DIRECTION GROUPS — kelompokkan segmen berdasarkan arah
    (selisih sudut < 20° = satu group/direction).
    n_directions menentukan tipe:
      2 directions, salah satu ujung → L-crossing
      2 directions, ~90°, badan+ujung → T-crossing  
      2 directions, badan+badan       → X-crossing
      3+ directions                   → T atau X

v2.0-v2.3 fixes retained (intersection approach, endpoint filter,
camera model, ROI, white pixel removed).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, Header

import numpy as np
import cv2
import math
import time
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field
from enum import IntEnum


# ─────────────────────────────────────────────────────────────────────────────
# TIPE DATA
# ─────────────────────────────────────────────────────────────────────────────

class CrossingType(IntEnum):
    L_CROSSING = 2
    T_CROSSING = 3
    X_CROSSING = 4

CROSSING_COLOR = {
    CrossingType.L_CROSSING: (0,   255, 255),   # kuning
    CrossingType.T_CROSSING: (255, 128,   0),   # oranye
    CrossingType.X_CROSSING: (255,   0, 128),   # magenta
}

@dataclass
class LineSegment:
    x1: float; y1: float
    x2: float; y2: float
    length: float
    confidence: float
    angle_deg: float = 0.0

@dataclass
class CrossingCandidate:
    px: float; py: float
    ctype: CrossingType
    confidence: float
    n_lines: int = 0
    world_x: float = 0.0
    world_y: float = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS GEOMETRI
# ─────────────────────────────────────────────────────────────────────────────

def line_angle_deg(x1, y1, x2, y2) -> float:
    return math.degrees(math.atan2(y2-y1, x2-x1)) % 180.0

def angle_diff_deg(a1: float, a2: float) -> float:
    d = abs(a1 - a2) % 180.0
    return min(d, 180.0 - d)

def line_intersection(s1: LineSegment,
                       s2: LineSegment) -> Optional[Tuple[float, float]]:
    x1,y1,x2,y2 = s1.x1,s1.y1,s1.x2,s1.y2
    x3,y3,x4,y4 = s2.x1,s2.y1,s2.x2,s2.y2
    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(denom) < 1e-6:
        return None
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom
    return (x1 + t*(x2-x1), y1 + t*(y2-y1))

def point_near_endpoint(px, py, seg: LineSegment, ratio: float) -> bool:
    tol = seg.length * ratio
    return min(math.hypot(px-seg.x1, py-seg.y1),
               math.hypot(px-seg.x2, py-seg.y2)) < tol

def point_on_body(px, py, seg: LineSegment, ratio: float) -> bool:
    tol = seg.length * ratio
    d1 = math.hypot(px-seg.x1, py-seg.y1)
    d2 = math.hypot(px-seg.x2, py-seg.y2)
    return d1 > tol and d2 > tol


# ─────────────────────────────────────────────────────────────────────────────
# NODE
# ─────────────────────────────────────────────────────────────────────────────

class CrossingDetector(Node):
    def __init__(self):
        super().__init__('crossing_detector')

        # ── Parameter ────────────────────────────────────────────────────────
        self.declare_parameter('image_width',            1280)
        self.declare_parameter('image_height',            720)
        self.declare_parameter('focal_length',            793.3)
        self.declare_parameter('cam_pitch_deg',           -20.0)
        self.declare_parameter('camera_height_m',          0.475)
        self.declare_parameter('field_half_len',            4.5)
        self.declare_parameter('field_half_wid',            3.0)

        self.declare_parameter('min_seg_confidence',       0.20)
        self.declare_parameter('min_seg_length',           25.0)
        self.declare_parameter('cluster_dist_px',          60)   # FIX2: 35→60px
        self.declare_parameter('min_angle_diff_deg',       25.0)
        self.declare_parameter('max_in_frame_margin_px',   50)
        self.declare_parameter('near_endpoint_ratio',       0.30)
        self.declare_parameter('roi_top_ratio',             0.25)
        self.declare_parameter('roi_bottom_ratio',          0.97)
        self.declare_parameter('min_confidence',            0.40)
        self.declare_parameter('max_crossings_per_frame',    6)   # FIX2: 10→6

        # FIX 1: Goal post filter
        # Tiang gawang: vertikal (angle>70°) dan ujung atas di area gawang (y<30%)
        self.declare_parameter('goal_filter_angle_deg',    70.0)  # min angle untuk dianggap vertikal
        self.declare_parameter('goal_filter_top_ratio',     0.30) # ujung atas harus di atas 30%

        # FIX 2: World-space dedup
        self.declare_parameter('world_merge_dist_m',        0.25) # merge crossing <0.25m di world

        # ── Load ─────────────────────────────────────────────────────────────
        self.img_w        = self.get_parameter('image_width').value
        self.img_h        = self.get_parameter('image_height').value
        self.focal        = self.get_parameter('focal_length').value
        self.cam_pitch    = math.radians(self.get_parameter('cam_pitch_deg').value)
        self.cam_h        = self.get_parameter('camera_height_m').value
        self.fhl          = self.get_parameter('field_half_len').value
        self.fhw          = self.get_parameter('field_half_wid').value
        self.min_seg_conf  = self.get_parameter('min_seg_confidence').value
        self.min_seg_len   = self.get_parameter('min_seg_length').value
        self.cluster_dist  = self.get_parameter('cluster_dist_px').value
        self.min_angle     = self.get_parameter('min_angle_diff_deg').value
        self.frame_margin  = self.get_parameter('max_in_frame_margin_px').value
        self.ep_ratio      = self.get_parameter('near_endpoint_ratio').value
        self.roi_top       = self.get_parameter('roi_top_ratio').value
        self.roi_bot       = self.get_parameter('roi_bottom_ratio').value
        self.min_conf      = self.get_parameter('min_confidence').value
        self.max_cross     = self.get_parameter('max_crossings_per_frame').value
        self.goal_ang      = self.get_parameter('goal_filter_angle_deg').value   # FIX1
        self.goal_top      = self.get_parameter('goal_filter_top_ratio').value   # FIX1
        self.world_merge   = self.get_parameter('world_merge_dist_m').value      # FIX2

        # Endpoint proximity filter
        # FP circle: ~190px dari semua ujung | Crossing sejati: 0-60px
        # 80px memberi margin untuk segmen yang sedikit terpotong
        self.declare_parameter('max_ep_dist_px', 80)
        self.max_ep_dist = self.get_parameter('max_ep_dist_px').value

        # ── State ─────────────────────────────────────────────────────────────
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self._frame_count = 0
        self._seg_recv_count = 0
        self._last_recv_time = time.time()

        # ── Subscriptions ────────────────────────────────────────────────────
        self.sub_segs = self.create_subscription(
            Float32MultiArray, '/field_line_segments',
            self._cb_segments, 10)

        self.sub_odom = self.create_subscription(
            Odometry, '/odom',
            self._cb_odom, qos_profile_sensor_data)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_crossings = self.create_publisher(
            PoseArray, '/detected_crossings', 10)
        self.pub_debug = self.create_publisher(
            Image, '/crossing_debug_image', 10)

        self.create_timer(5.0, self._status_log)

        self.get_logger().info('=' * 64)
        self.get_logger().info('CrossingDetector v2.4 — goal filter + world dedup + classify fix')
        self.get_logger().info(
            f'  min_seg={self.min_seg_len}px  cluster={self.cluster_dist}px  '
            f'ep_dist={self.max_ep_dist}px  angle={self.min_angle}°')
        self.get_logger().info(
            f'  goal_filter: angle>{self.goal_ang}° top<{self.goal_top*100:.0f}%  '
            f'world_merge={self.world_merge}m')
        self.get_logger().info(
            f'  roi={self.roi_top*100:.0f}%-{self.roi_bot*100:.0f}%  '
            f'max_cross={self.max_cross}')
        self.get_logger().info('=' * 64)

    # ── Periodic status ───────────────────────────────────────────────────────

    def _status_log(self):
        now = time.time()
        dt  = now - self._last_recv_time
        self.get_logger().info(
            f'[STATUS] frames={self._frame_count}  '
            f'seg_msgs={self._seg_recv_count}  '
            f'last_msg={dt:.1f}s ago',
            throttle_duration_sec=0.0)
        if dt > 3.0:
            self.get_logger().warn(
                '  No /field_line_segments for >3s — '
                'check segment_classifier_gw is running')

    # ── Odometry ──────────────────────────────────────────────────────────────

    def _cb_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))

    # ── Main callback ─────────────────────────────────────────────────────────

    def _cb_segments(self, msg):
        self._seg_recv_count += 1
        self._last_recv_time = time.time()
        self._frame_count   += 1

        data = list(msg.data)
        n_floats = len(data)

        if n_floats == 0:
            self.get_logger().debug('Empty /field_line_segments', throttle_duration_sec=2.0)
            self._publish_debug([], [], info='no segments')
            return

        # ── [1] Parse segmen ──────────────────────────────────────────────────
        # Format bisa 6 atau 8 fields tergantung versi segment_classifier_gw
        # [x1,y1,x2,y2,len,conf]         (6 fields) atau
        # [x1,y1,x2,y2,len,conf,gw_x,gw_y] (8 fields)
        SEG_FIELDS = 8 if (n_floats % 8 == 0) else 6
        if n_floats % SEG_FIELDS != 0:
            # Coba 6
            SEG_FIELDS = 6
            if n_floats % 6 != 0:
                self.get_logger().warn(
                    f'Unexpected segment data length: {n_floats} '
                    f'(not divisible by 6 or 8)',
                    throttle_duration_sec=5.0)
                return

        segments: List[LineSegment] = []
        n_total = n_floats // SEG_FIELDS

        for i in range(0, n_floats - SEG_FIELDS + 1, SEG_FIELDS):
            x1,y1,x2,y2 = data[i],data[i+1],data[i+2],data[i+3]
            seg_len = data[i+4]
            conf    = data[i+5]

            if conf < self.min_seg_conf:
                continue
            if seg_len < self.min_seg_len:
                continue

            seg = LineSegment(x1=float(x1), y1=float(y1),
                              x2=float(x2), y2=float(y2),
                              length=float(seg_len), confidence=float(conf))
            seg.angle_deg = line_angle_deg(x1, y1, x2, y2)
            segments.append(seg)

        self.get_logger().debug(
            f'frame#{self._frame_count}: '
            f'{n_total} raw segs → {len(segments)} after filter '
            f'(conf≥{self.min_seg_conf} len≥{self.min_seg_len})',
            throttle_duration_sec=1.0)

        # ── [1b] FIX 1: Goal post filter ─────────────────────────────────────
        # Tiang gawang: angle hampir vertikal (>goal_ang°) DAN ujung atas
        # ada di area atas frame (y < goal_top ratio)
        # Hanya garis VERTIKAL yang ada di area ATAS yang dibuang.
        # Garis vertikal lapangan (penalty box side) ada di area BAWAH frame.
        before_goal = len(segments)
        segments = [s for s in segments if not self._is_goal_post(s)]
        n_removed = before_goal - len(segments)
        if n_removed > 0:
            self.get_logger().debug(
                f'  goal filter: removed {n_removed} goal post segments',
                throttle_duration_sec=1.0)

        if len(segments) < 2:
            self.get_logger().debug(
                f'Only {len(segments)} segs after filter (need ≥2)',
                throttle_duration_sec=2.0)
            self._publish_debug(segments, [], info=f'{n_total}→{len(segments)} segs')
            return

        # ── [2] Hitung semua interseksi ───────────────────────────────────────
        raw_ints: List[Dict] = []
        for i in range(len(segments)):
            for j in range(i+1, len(segments)):
                s1, s2 = segments[i], segments[j]
                ang = angle_diff_deg(s1.angle_deg, s2.angle_deg)
                if ang < self.min_angle:
                    continue
                pt = line_intersection(s1, s2)
                if pt is None:
                    continue
                px, py = pt
                m = self.frame_margin
                if not (-m <= px <= self.img_w+m and -m <= py <= self.img_h+m):
                    continue

                # ── Endpoint proximity filter ──────────────────────────────
                # FP dari circle: extended lines bertemu jauh dari semua endpoint
                # Crossing sejati: titik interseksi dekat ujung setidaknya 1 segmen
                # Gap yang terukur: FP ~190px, crossing sejati ~0px
                min_ep_dist = min(
                    math.hypot(px-s1.x1, py-s1.y1),
                    math.hypot(px-s1.x2, py-s1.y2),
                    math.hypot(px-s2.x1, py-s2.y1),
                    math.hypot(px-s2.x2, py-s2.y2))

                if min_ep_dist > self.max_ep_dist:
                    continue  # terlalu jauh dari semua ujung → FP

                raw_ints.append({'px': px, 'py': py,
                                  'segs': [i, j], 'ang': ang,
                                  'ep_dist': min_ep_dist})

        self.get_logger().debug(
            f'  → {len(raw_ints)} intersections (after ep filter)',
            throttle_duration_sec=1.0)

        if not raw_ints:
            self._publish_debug(segments, [],
                                info=f'{len(segments)} segs, 0 intersects')
            return

        # ── [3] Cluster ───────────────────────────────────────────────────────
        clusters = self._cluster(raw_ints, segments)

        # ── [4] Klasifikasi ───────────────────────────────────────────────────
        candidates = self._classify(clusters, segments)

        # ── [5] Filter ────────────────────────────────────────────────────────
        filtered = self._filter(candidates)

        # ── [6] Back-project ──────────────────────────────────────────────────
        projected = self._project(filtered)

        # ── [6b] FIX 2: Dedup di world space ─────────────────────────────────
        # Merge crossing yang jaraknya < world_merge_dist_m di koordinat world
        projected = self._world_dedup(projected)

        if len(projected) > self.max_cross:
            projected.sort(key=lambda c: c.confidence, reverse=True)
            projected = projected[:self.max_cross]

        # ── Publish ───────────────────────────────────────────────────────────
        self._publish_crossings(projected)
        self._publish_debug(segments, projected,
                            info=f'{len(segments)}s/{len(raw_ints)}i/{len(clusters)}cl')

        if projected:
            self.get_logger().info(
                f'[CROSS] N={len(projected)}  '
                + ' '.join(f'{CrossingType(c.ctype).name[0]}'
                            f'({c.confidence:.2f})'
                            f'@({c.world_x:.2f},{c.world_y:.2f})'
                            for c in projected))
        else:
            self.get_logger().debug(
                f'  candidates={len(candidates)} filtered={len(filtered)} projected=0',
                throttle_duration_sec=1.0)

    # ── FIX 1: Goal post filter ───────────────────────────────────────────────

    def _is_goal_post(self, seg: LineSegment) -> bool:
        """
        Deteksi apakah segmen adalah tiang gawang.

        Tiang gawang memiliki dua ciri:
        1. Hampir vertikal: angle mendekati 90° (> goal_filter_angle_deg)
        2. Posisi di area atas frame: ujung ATAS di y < goal_filter_top_ratio * img_h

        Garis vertikal LAPANGAN (sisi penalty box) juga vertikal, tapi
        ujung atas-nya ada di BAWAH area gawang (tidak terlalu tinggi di frame).
        """
        # Cek kevertikalan
        ang = seg.angle_deg
        # angle_deg dalam [0,180), vertikal = 90°
        is_vertical = abs(ang - 90.0) < (90.0 - self.goal_ang)

        if not is_vertical:
            return False

        # Ujung atas segmen (y lebih kecil = lebih atas di image)
        top_y = min(seg.y1, seg.y2)
        threshold_y = self.goal_top * self.img_h

        return top_y < threshold_y

    # ── FIX 2: World-space deduplication ─────────────────────────────────────

    def _world_dedup(self,
                      cands: List[CrossingCandidate]) -> List[CrossingCandidate]:
        """
        Merge crossing yang terlalu dekat di world space.
        Jika dua crossing berjarak < world_merge_dist_m, pertahankan
        yang confidence-nya lebih tinggi, atau yang tipe-nya lebih spesifik
        (T > L, X > T dalam arti lebih banyak informasi).
        """
        if not cands:
            return cands

        used   = [False] * len(cands)
        result = []
        d2     = self.world_merge ** 2

        # Urutkan dari confidence tertinggi
        order = sorted(range(len(cands)),
                       key=lambda i: cands[i].confidence, reverse=True)

        for i in order:
            if used[i]:
                continue
            c1 = cands[i]
            used[i] = True

            for j in order:
                if used[j] or j == i:
                    continue
                c2 = cands[j]
                wd2 = (c1.world_x - c2.world_x)**2 + (c1.world_y - c2.world_y)**2
                if wd2 < d2:
                    used[j] = True  # absorb c2 ke c1

            result.append(c1)

        return result

    # ── Clustering ────────────────────────────────────────────────────────────

    def _cluster(self, raw: List[Dict],
                  segs: List[LineSegment]) -> List[Dict]:
        used = [False]*len(raw); clusters = []; d2 = self.cluster_dist**2
        for i, r1 in enumerate(raw):
            if used[i]: continue
            pts = [(r1['px'], r1['py'])]; seg_ids = set(r1['segs']); used[i] = True
            for j, r2 in enumerate(raw):
                if used[j]: continue
                if (r1['px']-r2['px'])**2 + (r1['py']-r2['py'])**2 <= d2:
                    pts.append((r2['px'], r2['py']))
                    seg_ids.update(r2['segs']); used[j] = True
            cx = sum(p[0] for p in pts)/len(pts)
            cy = sum(p[1] for p in pts)/len(pts)
            sids = list(seg_ids)
            angles = [segs[s].angle_deg for s in sids]
            confs  = [segs[s].confidence for s in sids]
            clusters.append({'px':cx,'py':cy,'seg_ids':sids,
                              'n':len(sids),'angles':angles,
                              'conf':sum(confs)/len(confs)})
        return clusters

    # ── FIX 3: Klasifikasi berbasis direction groups ───────────────────────────

    def _classify(self, clusters: List[Dict],
                   segs: List[LineSegment]) -> List[CrossingCandidate]:
        """
        Klasifikasi crossing berdasarkan DIRECTION GROUPS, bukan jumlah segmen.

        Masalah sebelumnya: n_lines dihitung dari jumlah segmen di cluster.
        Tapi satu garis lapangan bisa dipecah menjadi beberapa segmen pendek
        oleh segment_classifier_gw → n=4 padahal hanya 2 arah nyata.

        Fix: kelompokkan segmen berdasarkan arah (angle_diff < dir_tol = 20°).
        Jumlah DIRECTION GROUPS yang berbeda = jumlah arah unik.

        Klasifikasi:
          n_dir == 2:
            - Titik di ujung kedua arah           → L-crossing
            - Titik di ujung 1, badan lainnya     → T-crossing
            - Titik di badan kedua arah            → X-crossing
          n_dir == 3: T-crossing (3 arah)
          n_dir >= 4: X-crossing
        """
        DIR_TOL = 20.0   # selisih sudut < 20° = arah yang sama
        cands   = []

        for cl in clusters:
            px   = cl['px']
            py   = cl['py']
            conf = cl['conf']
            sids = cl['seg_ids']

            if len(sids) < 2:
                continue

            # ── Kelompokkan segmen ke direction groups ────────────────────────
            angles = [segs[s].angle_deg for s in sids]
            groups: List[List[int]] = []   # list of seg index lists

            for idx, ang in enumerate(angles):
                merged = False
                for grp in groups:
                    rep_ang = angles[grp[0]]
                    if angle_diff_deg(ang, rep_ang) < DIR_TOL:
                        grp.append(idx)
                        merged = True
                        break
                if not merged:
                    groups.append([idx])

            n_dir = len(groups)

            if n_dir >= 4:
                cands.append(CrossingCandidate(px, py, CrossingType.X_CROSSING,
                                               min(1.0, conf + 0.1), n_dir))
                continue

            if n_dir == 3:
                cands.append(CrossingCandidate(px, py, CrossingType.T_CROSSING,
                                               conf, n_dir))
                continue

            if n_dir < 2:
                continue  # hanya 1 arah = garis tunggal, bukan crossing

            # n_dir == 2: tentukan L/T/X dari posisi titik terhadap tiap arah
            # Representatif segmen terpanjang dari tiap group
            rep_segs = []
            for grp in groups:
                best = max(grp, key=lambda i: segs[sids[i]].length)
                rep_segs.append(segs[sids[best]])

            s1, s2 = rep_segs[0], rep_segs[1]
            ang_diff = angle_diff_deg(s1.angle_deg, s2.angle_deg)

            ep1 = point_near_endpoint(px, py, s1, self.ep_ratio)
            ep2 = point_near_endpoint(px, py, s2, self.ep_ratio)
            bd1 = point_on_body(px, py, s1, self.ep_ratio)
            bd2 = point_on_body(px, py, s2, self.ep_ratio)
            near_perp = (ang_diff > 60.0)

            if bd1 and bd2:
                ctype = CrossingType.X_CROSSING;  bonus = 0.15
            elif near_perp and ((ep1 and bd2) or (ep2 and bd1)):
                ctype = CrossingType.T_CROSSING;  bonus = 0.05
            elif near_perp and ep1 and ep2:
                # ~90°, keduanya ujung: di lapangan biasanya T-crossing sudut penalty
                ctype = CrossingType.T_CROSSING;  bonus = 0.0
            elif (not near_perp) and (ep1 or ep2):
                ctype = CrossingType.L_CROSSING;  bonus = 0.05
            else:
                ctype = CrossingType.T_CROSSING if near_perp else CrossingType.L_CROSSING
                bonus = 0.0

            cands.append(CrossingCandidate(px, py, ctype,
                                           min(1.0, conf + bonus), n_dir))
        return cands

    # ── Filter ────────────────────────────────────────────────────────────────

    def _filter(self, cands: List[CrossingCandidate]) -> List[CrossingCandidate]:
        roi_top = self.img_h * self.roi_top
        roi_bot = self.img_h * self.roi_bot
        out = []
        for c in cands:
            if c.py < roi_top or c.py > roi_bot: continue
            if c.px < 0 or c.px > self.img_w:    continue
            if c.confidence < self.min_conf:       continue
            out.append(c)
        return out

    # ── Back-projection ───────────────────────────────────────────────────────

    def _project(self, cands: List[CrossingCandidate]) -> List[CrossingCandidate]:
        out = []
        for c in cands:
            wp = self._px_to_world(c.px, c.py)
            if wp:
                c.world_x, c.world_y = wp
                out.append(c)
        return out

    def _px_to_world(self, u, v) -> Optional[Tuple[float,float]]:
        """
        Back-project pixel (u,v) ke koordinat world di lantai.

        Model kamera pinhole dengan tilt ke bawah (pitch < 0):
          Camera tilted DOWN by |pitch|° → ray kamera dirotasi sekitar X-axis
          world_fwd  = rc_z*cos(|p|) - rc_y*sin(|p|)  (ke depan)
          world_down = rc_z*sin(|p|) + rc_y*cos(|p|)  (ke bawah)
          t = cam_h / world_down  → interseksi lantai
        """
        cx = self.img_w / 2.0
        cy = self.img_h / 2.0

        # Ray di camera frame (z maju, y bawah, x kanan)
        rc_x = (u - cx) / self.focal   # lateral
        rc_y = (v - cy) / self.focal   # vertical (positif = bawah)
        rc_z = 1.0                      # maju (normalized)

        # Rotasi pitch: kamera tilt ke bawah |pitch|°
        ap = abs(self.cam_pitch)   # cam_pitch negatif = tilt bawah
        world_fwd  = rc_z * math.cos(ap) - rc_y * math.sin(ap)
        world_down = rc_z * math.sin(ap) + rc_y * math.cos(ap)

        if world_down < 1e-6:
            return None  # ray mengarah ke atas, tidak kena lantai

        t = self.cam_h / world_down
        if t < 0.05 or t > 6.0:
            return None

        # Koordinat di ground plane (relative to camera, belum ada yaw)
        gnd_fwd = t * world_fwd
        gnd_lat = t * rc_x

        cam_dist = math.hypot(gnd_fwd, gnd_lat)
        if cam_dist < 0.05 or cam_dist > 5.0:
            return None

        # Transform ke world frame via robot yaw
        yaw = self.odom_yaw
        wx = self.odom_x + gnd_fwd * math.cos(yaw) - gnd_lat * math.sin(yaw)
        wy = self.odom_y + gnd_fwd * math.sin(yaw) + gnd_lat * math.cos(yaw)

        if abs(wx) > self.fhl * 1.5 or abs(wy) > self.fhw * 1.5:
            return None
        return (wx, wy)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_crossings(self, cands: List[CrossingCandidate]):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for c in cands:
            p = Pose()
            p.position.x = c.world_x; p.position.y = c.world_y
            p.position.z = float(c.ctype); p.orientation.w = c.confidence
            msg.poses.append(p)
        self.pub_crossings.publish(msg)

    def _publish_debug(self, segs: List[LineSegment],
                        crossings: List[CrossingCandidate],
                        info: str = ''):
        """
        Build debug image dari scratch — tidak tergantung /camera/line_image.
        Background hitam, garis hijau, crossing berwarna.
        """
        h, w = self.img_h, self.img_w
        debug = np.zeros((h, w, 3), dtype=np.uint8)

        # Grid lapangan (abu-abu sangat gelap, untuk orientasi)
        for x in range(0, w, w//9):
            cv2.line(debug, (x,0), (x,h), (20,20,20), 1)
        for y in range(0, h, h//6):
            cv2.line(debug, (0,y), (w,y), (20,20,20), 1)

        # ROI lines
        roi_top_px = int(h * self.roi_top)
        roi_bot_px = int(h * self.roi_bot)
        cv2.line(debug, (0,roi_top_px), (w,roi_top_px), (50,50,80), 1)
        cv2.putText(debug, f'ROI top {self.roi_top*100:.0f}%',
                    (5, roi_top_px-4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (60,60,90), 1)

        # Segmen lapangan (hijau)
        for seg in segs:
            pt1 = (int(seg.x1), int(seg.y1))
            pt2 = (int(seg.x2), int(seg.y2))
            # Tebal lebih tinggi = confidence lebih tinggi
            thick = 1 + int(seg.confidence * 2)
            cv2.line(debug, pt1, pt2, (0, int(150+105*seg.confidence), 0), thick)
            cv2.circle(debug, pt1, 3, (0,255,128), -1)
            cv2.circle(debug, pt2, 3, (0,255,128), -1)
            # Label confidence
            mx = (pt1[0]+pt2[0])//2; my = (pt1[1]+pt2[1])//2
            cv2.putText(debug, f'{seg.confidence:.2f}',
                        (mx,my-6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.3, (0,200,0), 1)

        # Crossings
        for c in crossings:
            px, py = int(c.px), int(c.py)
            color  = CROSSING_COLOR[c.ctype]
            label  = CrossingType(c.ctype).name[0]
            r = 22
            cv2.circle(debug, (px,py), r, color, 2)
            cv2.circle(debug, (px,py), 5, color, -1)
            cv2.putText(debug,
                        f'{label}({c.confidence:.2f}) [{c.world_x:.1f},{c.world_y:.1f}m]',
                        (px+r+4, py+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

        # Info bar
        info_text = (f'segs:{len(segs)} cross:{len(crossings)} | {info} | '
                     f'frame#{self._frame_count}')
        cv2.rectangle(debug, (0,h-22), (w,h), (20,20,20), -1)
        cv2.putText(debug, info_text, (5,h-7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180,180,180), 1)

        # Publish
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'cam_link'
        out.height = h; out.width = w
        out.encoding = 'bgr8'; out.step = w*3
        out.data = debug.tobytes()
        self.pub_debug.publish(out)


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CrossingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
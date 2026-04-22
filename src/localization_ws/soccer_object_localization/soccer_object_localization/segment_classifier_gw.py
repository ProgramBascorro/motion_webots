#!/usr/bin/env python3
"""
segment_classifier_gw.py — Field Line Segment Classifier (GW⃗ / GB⃗)
======================================================================
Referensi: Paper 1 (Melo dkk.) — "Efficient Lines Detection for Robot Soccer"

PROGRESS:
  v2.1: 18 ok / 15 rej (11 GW-fail) — 55%
  v2.3: 25 ok /  5 rej ( 0 GW-fail) — 83%  [perp scan]
  v2.4: 31 ok /  3 rej ( 0 GW-fail) — 91%  [dual-zone Hough]
  v2.5: target >95%

CHANGELOG v2.5:
  [FIX 21] Hough FAR lebih agresif: min_line 25→15, threshold 30→20
           Tangkap segmen sangat pendek di garis penalty box yang jauh

  [FIX 22] min_segment_len_far: 20→12
           Segmen 12-20px masih valid landmark untuk AMCL di zona jauh

  [FIX 23] Adaptive classifier per zona:
           FAR:  vote_thr=0.35, min_proj=0.10, scan_half=6
           NEAR: vote_thr=0.45, min_proj=0.15, scan_half=4
           Garis jauh → gradient lebih lemah → threshold lebih longgar

  [FIX 24] Debug lengkap: pisahkan counter GW-fail vs len-fail vs boundary
           Tambah warna ungu untuk segmen yang di-classify sebagai BOUNDARY
           → Memudahkan debugging: bisa lihat apakah ada false boundary

  [FIX 25] far_zone_split default 0.60 (was 0.55)
           Dari observasi: zone split line di screenshot ada di ~65% frame
           → 0.60 lebih representatif untuk lapangan Webots

Semua fix v2.1–v2.4 dipertahankan.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


SEG_FIELDS         = 8
SEG_LABEL_LINE     = 1.0
SEG_LABEL_BOUNDARY = 2.0
GW_CALIB_INTERVAL  = 30


class SegmentClassifierGW(Node):
    def __init__(self):
        super().__init__('segment_classifier_gw')

        # ── Parameter declarations ──────────────────────────────────────────
        # Klasifikasi NEAR (default)
        self.declare_parameter('max_angle_deg',        40.0)
        self.declare_parameter('min_projection',       0.15)
        self.declare_parameter('vote_threshold',       0.45)
        self.declare_parameter('scan_half',            4)

        # Klasifikasi FAR — [FIX 23] lebih longgar
        self.declare_parameter('max_angle_deg_far',    45.0)
        self.declare_parameter('min_projection_far',   0.10)
        self.declare_parameter('vote_threshold_far',   0.35)
        self.declare_parameter('scan_half_far',        6)

        # Hough NEAR
        self.declare_parameter('hough_threshold',      60)
        self.declare_parameter('hough_min_line',       60)
        self.declare_parameter('hough_max_gap',        15)
        self.declare_parameter('min_segment_len',      30)
        self.declare_parameter('max_segment_len',      600)

        # Hough FAR — [FIX 21][FIX 22]
        self.declare_parameter('hough_threshold_far',  20)   # was 30
        self.declare_parameter('hough_min_line_far',   15)   # was 25
        self.declare_parameter('hough_max_gap_far',    25)
        self.declare_parameter('min_segment_len_far',  12)   # was 20

        # Zone & NMS
        self.declare_parameter('far_zone_split',       0.60)  # [FIX 25] was 0.55
        self.declare_parameter('nms_dist',             15)

        # Misc
        self.declare_parameter('white_threshold',      200)
        self.declare_parameter('use_boundary_roi',     True)
        self.declare_parameter('publish_debug',        True)
        self.declare_parameter('use_line_image',       True)
        self.declare_parameter('roi_top_fallback',     0.35)
        self.declare_parameter('auto_calibrate_gw',    True)

        # ── Load parameters ─────────────────────────────────────────────────
        self.max_angle      = math.radians(self.get_parameter('max_angle_deg').value)
        self.min_proj       = self.get_parameter('min_projection').value
        self.vote_thr       = self.get_parameter('vote_threshold').value
        self.scan_half      = self.get_parameter('scan_half').value

        self.max_angle_far  = math.radians(self.get_parameter('max_angle_deg_far').value)
        self.min_proj_far   = self.get_parameter('min_projection_far').value
        self.vote_thr_far   = self.get_parameter('vote_threshold_far').value
        self.scan_half_far  = self.get_parameter('scan_half_far').value

        self.hough_thr      = self.get_parameter('hough_threshold').value
        self.hough_min      = self.get_parameter('hough_min_line').value
        self.hough_gap      = self.get_parameter('hough_max_gap').value
        self.min_len        = self.get_parameter('min_segment_len').value
        self.max_len        = self.get_parameter('max_segment_len').value

        self.hough_thr_far  = self.get_parameter('hough_threshold_far').value
        self.hough_min_far  = self.get_parameter('hough_min_line_far').value
        self.hough_gap_far  = self.get_parameter('hough_max_gap_far').value
        self.min_len_far    = self.get_parameter('min_segment_len_far').value

        self.far_split      = self.get_parameter('far_zone_split').value
        self.nms_dist       = self.get_parameter('nms_dist').value
        self.white_thr      = self.get_parameter('white_threshold').value
        self.use_bnd        = self.get_parameter('use_boundary_roi').value
        self.pub_debug      = self.get_parameter('publish_debug').value
        self.use_line_image = self.get_parameter('use_line_image').value
        self.roi_top_fb     = self.get_parameter('roi_top_fallback').value
        self.auto_calib     = self.get_parameter('auto_calibrate_gw').value

        # ── GW vector ideal Webots BGR ────────────────────────────────────────
        gw_ideal = np.array([186.0, 81.0, 186.0])
        self.gw_vec = gw_ideal / np.linalg.norm(gw_ideal)
        gb_raw = np.array([-34.0, -139.0, -34.0])
        self.gb_vec = gb_raw / np.linalg.norm(gb_raw)

        # ── State ────────────────────────────────────────────────────────────
        self.bridge         = CvBridge()
        self.last_boundary  = None
        self.last_raw_frame = None
        self._calib_counter = 0

        # ── Subscriptions ────────────────────────────────────────────────────
        self.sub_raw = self.create_subscription(
            Image, '/robotis_op3/camera/image_raw',
            self._cb_raw, qos_profile_sensor_data)

        if self.use_line_image:
            self.sub_image = self.create_subscription(
                Image, '/camera/line_image',
                self._cb_line_image, qos_profile_sensor_data)
            self.get_logger().info('Input: /camera/line_image')
        else:
            self.sub_image = self.create_subscription(
                Image, '/robotis_op3/camera/image_raw',
                self._cb_image_raw_mode, qos_profile_sensor_data)

        if self.use_bnd:
            self.sub_boundary = self.create_subscription(
                Polygon, '/field_boundary', self._cb_boundary, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_segments  = self.create_publisher(Float32MultiArray, '/field_line_segments', 10)
        self.pub_debug_img = self.create_publisher(Image, '/field_line_debug_image', 10)

        self.get_logger().info('SegmentClassifierGW v2.5 started')
        self.get_logger().info(
            f'  NEAR: thr={self.hough_thr} min={self.hough_min} gap={self.hough_gap} '
            f'vote={self.vote_thr} proj={self.min_proj} scan={self.scan_half}')
        self.get_logger().info(
            f'  FAR:  thr={self.hough_thr_far} min={self.hough_min_far} gap={self.hough_gap_far} '
            f'vote={self.vote_thr_far} proj={self.min_proj_far} scan={self.scan_half_far} '
            f'split={self.far_split:.0%}')

    # ════════════════════════════════════════════════════════════════════════
    # GW AUTO-CALIBRATION
    # ════════════════════════════════════════════════════════════════════════

    def _auto_calibrate_gw(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        strip    = frame[int(h*0.80):int(h*0.95), int(w*0.1):int(w*0.9)]
        hsv      = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        gm       = cv2.inRange(hsv, (30, 50, 30), (90, 255, 255))
        grass_px = strip[gm > 0]

        roi  = frame[int(h*0.35):int(h*0.85), :]
        b, g, r = roi[:,:,0].astype(int), roi[:,:,1].astype(int), roi[:,:,2].astype(int)
        white_px = roi[(b > self.white_thr) & (g > self.white_thr) & (r > self.white_thr)]

        if len(grass_px) < 200 or len(white_px) < 30:
            return
        gw_raw = white_px.mean(axis=0).astype(np.float64) - grass_px.mean(axis=0).astype(np.float64)
        norm = np.linalg.norm(gw_raw)
        if norm < 10.0:
            return
        new_gw = gw_raw / norm
        ideal  = np.array([0.676, 0.294, 0.676])
        if np.dot(new_gw, ideal) > 0.85:
            self.gw_vec = new_gw
            self.get_logger().debug(
                f'GW calib: gw={np.round(self.gw_vec,3)}', throttle_duration_sec=5.0)

    # ════════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════════════

    def _cb_boundary(self, msg):
        self.last_boundary = msg

    def _cb_raw(self, msg):
        try:
            self.last_raw_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        if self.auto_calib:
            self._calib_counter += 1
            if self._calib_counter >= GW_CALIB_INTERVAL:
                self._calib_counter = 0
                self._auto_calibrate_gw(self.last_raw_frame)

    def _cb_line_image(self, msg):
        if self.last_raw_frame is None:
            return
        try:
            line_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}', throttle_duration_sec=5.0)
            return
        h, w     = line_mask.shape[:2]
        roi_mask = self._make_roi_mask(h, w)
        edges    = cv2.Canny(cv2.bitwise_and(line_mask, roi_mask), 50, 150)
        raw_lines = self._dual_zone_hough(edges, h)
        self._process_lines(raw_lines, self.last_raw_frame, roi_mask, h, msg.header.stamp)

    def _cb_image_raw_mode(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}', throttle_duration_sec=5.0)
            return
        h, w     = frame.shape[:2]
        roi_mask = self._make_roi_mask(h, w)
        gray     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.white_thr, 255, cv2.THRESH_BINARY)
        hsv      = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        non_grass = cv2.bitwise_not(cv2.inRange(hsv, (35,40,0), (85,255,255)))
        binary   = cv2.bitwise_and(cv2.bitwise_and(binary, non_grass), roi_mask)
        edges    = cv2.Canny(binary, 50, 150)
        raw_lines = self._dual_zone_hough(edges, h)
        self._process_lines(raw_lines, frame, roi_mask, h, msg.header.stamp)

    # ════════════════════════════════════════════════════════════════════════
    # DUAL-ZONE HOUGH + NMS
    # ════════════════════════════════════════════════════════════════════════

    def _dual_zone_hough(self, edges: np.ndarray, h: int):
        split_y = int(h * self.far_split)

        # NEAR zone
        e_near = edges.copy(); e_near[:split_y, :] = 0
        ln_near = cv2.HoughLinesP(e_near, 1, np.pi/180,
                                  threshold=self.hough_thr,
                                  minLineLength=self.hough_min,
                                  maxLineGap=self.hough_gap)

        # FAR zone
        e_far = edges.copy(); e_far[split_y:, :] = 0
        ln_far = cv2.HoughLinesP(e_far, 1, np.pi/180,
                                 threshold=self.hough_thr_far,
                                 minLineLength=self.hough_min_far,
                                 maxLineGap=self.hough_gap_far)

        all_lines = []
        if ln_near is not None:
            for ln in ln_near: all_lines.append((*ln[0], 'near'))
        if ln_far is not None:
            for ln in ln_far:  all_lines.append((*ln[0], 'far'))

        if not all_lines:
            return None
        return self._line_nms(all_lines)

    def _line_nms(self, lines):
        def seg_len(ln): return math.hypot(ln[2]-ln[0], ln[3]-ln[1])
        lines_sorted = sorted(lines, key=seg_len, reverse=True)
        kept = []
        for cand in lines_sorted:
            cx1,cy1,cx2,cy2 = cand[:4]
            cmx,cmy = (cx1+cx2)/2, (cy1+cy2)/2
            c_ang   = math.atan2(cy2-cy1, cx2-cx1)
            dup = False
            for ex in kept:
                ex1,ey1,ex2,ey2 = ex[:4]
                emx,emy = (ex1+ex2)/2, (ey1+ey2)/2
                e_ang   = math.atan2(ey2-ey1, ex2-ex1)
                mid_d   = math.hypot(cmx-emx, cmy-emy)
                a_diff  = abs(math.atan2(math.sin(c_ang-e_ang), math.cos(c_ang-e_ang)))
                a_diff  = min(a_diff, math.pi-a_diff)
                if mid_d < self.nms_dist and a_diff < math.radians(20):
                    dup = True; break
            if not dup:
                kept.append(cand)
        return np.array([[ln[:4]] for ln in kept], dtype=np.int32)

    # ════════════════════════════════════════════════════════════════════════
    # PROCESS LINES
    # ════════════════════════════════════════════════════════════════════════

    def _process_lines(self, raw_lines, frame, roi_mask, h, stamp):
        split_y      = int(h * self.far_split)
        segments_out = []
        debug_pass, debug_fail, debug_boundary = [], [], []

        if raw_lines is not None:
            for line in raw_lines:
                x1, y1, x2, y2 = line[0]
                seg_len  = math.hypot(x2-x1, y2-y1)
                y_center = (y1+y2)/2
                is_far   = y_center < split_y

                min_len_eff = self.min_len_far if is_far else self.min_len
                if seg_len < min_len_eff or seg_len > self.max_len:
                    debug_fail.append((x1,y1,x2,y2,'len',is_far))
                    continue

                label, conf, vr = self._classify_segment(
                    frame, x1, y1, x2, y2, seg_len, is_far)

                if label == SEG_LABEL_LINE:
                    segments_out.extend([x1,y1,x2,y2,seg_len,conf,conf,label])
                    debug_pass.append((x1,y1,x2,y2,conf,vr,is_far))
                elif label == SEG_LABEL_BOUNDARY:
                    # [FIX 24] Tampilkan boundary dengan warna ungu
                    debug_boundary.append((x1,y1,x2,y2,is_far))
                else:
                    debug_fail.append((x1,y1,x2,y2,'gw',is_far))

        seg_msg      = Float32MultiArray()
        seg_msg.data = [float(v) for v in segments_out]
        self.pub_segments.publish(seg_msg)

        n_pass = len(segments_out) // SEG_FIELDS
        n_raw  = len(raw_lines) if raw_lines is not None else 0
        self.get_logger().debug(
            f'Segments: {n_raw} raw → {n_pass} LINE  '
            f'{len(debug_boundary)} BOUNDARY  {len(debug_fail)} FAIL',
            throttle_duration_sec=2.0)

        if self.pub_debug:
            self._publish_debug(
                frame, roi_mask, debug_pass, debug_fail, debug_boundary, h, stamp)

    # ════════════════════════════════════════════════════════════════════════
    # [FIX 23] ADAPTIVE CLASSIFY PER ZONA
    # ════════════════════════════════════════════════════════════════════════

    def _classify_segment(self, frame, x1, y1, x2, y2, seg_len, is_far: bool):
        """
        [FIX 23] Parameter klasifikasi adaptif berdasarkan zona:
          FAR:  scan_half=6, vote_thr=0.35, min_proj=0.10, max_angle=45deg
          NEAR: scan_half=4, vote_thr=0.45, min_proj=0.15, max_angle=40deg

        Garis jauh memiliki gradient lebih lemah (resolusi kecil + noise tinggi)
        sehingga threshold lebih longgar diperlukan untuk recall yang baik.
        """
        h, w = frame.shape[:2]
        dx, dy = x2-x1, y2-y1
        nx, ny = -dy/seg_len,  dx/seg_len

        # Pilih parameter berdasarkan zona
        scan_h   = self.scan_half_far  if is_far else self.scan_half
        vote_t   = self.vote_thr_far   if is_far else self.vote_thr
        min_p    = self.min_proj_far   if is_far else self.min_proj
        max_a    = self.max_angle_far  if is_far else self.max_angle

        n_samples = max(int(seg_len/5), 5)
        sobel_x_k = np.array([[-1,0,1],[-2,0,2],[-1,0,1]], np.float32)
        sobel_y_k = np.array([[-1,-2,-1],[0,0,0],[1,2,1]], np.float32)

        votes_line = 0; votes_bnd = 0; proj_sum = 0.0; valid = 0

        for k in range(n_samples):
            t  = k/(n_samples-1) if n_samples > 1 else 0.5
            cx = int(x1 + t*dx)
            cy = int(y1 + t*dy)

            best_norm = -1.0; best_grad = None
            for off in range(-scan_h, scan_h+1):
                sx = cx + int(round(off*nx))
                sy = cy + int(round(off*ny))
                if sx < 1 or sx >= w-1 or sy < 1 or sy >= h-1:
                    continue
                patch = frame[sy-1:sy+2, sx-1:sx+2].astype(np.float32)
                if patch.shape[0] != 3 or patch.shape[1] != 3:
                    continue
                gv = np.empty(3, np.float64)
                for c in range(3):
                    gx = float(np.sum(sobel_x_k * patch[:,:,c]))/4.0
                    gy = float(np.sum(sobel_y_k * patch[:,:,c]))/4.0
                    gv[c] = gx*nx + gy*ny
                gn = float(np.linalg.norm(gv))
                if gn > best_norm:
                    best_norm = gn
                    best_grad = gv/gn if gn > 1e-6 else None

            if best_grad is None or best_norm < 1e-6:
                continue
            valid += 1

            gw_p = abs(float(np.dot(best_grad, self.gw_vec)))
            if math.acos(np.clip(gw_p,0.0,1.0)) < max_a and gw_p > min_p:
                votes_line += 1; proj_sum += gw_p; continue

            gb_p = abs(float(np.dot(best_grad, self.gb_vec)))
            if math.acos(np.clip(gb_p,0.0,1.0)) < max_a and gb_p > min_p:
                votes_bnd += 1

        if valid == 0:
            return 0.0, 0.0, 0.0

        vr          = votes_line / valid
        min_votes_a = max(2, int(valid * 0.30))

        if vr >= vote_t and votes_line >= min_votes_a:
            return SEG_LABEL_LINE, min(proj_sum/votes_line/0.8, 1.0), vr
        if votes_bnd > votes_line and votes_bnd >= min_votes_a:
            return SEG_LABEL_BOUNDARY, 0.0, vr
        return 0.0, 0.0, vr

    # ════════════════════════════════════════════════════════════════════════
    # ROI & DEBUG
    # ════════════════════════════════════════════════════════════════════════

    def _make_roi_mask(self, h, w):
        mask = np.ones((h, w), dtype=np.uint8) * 255
        if self.use_bnd and self.last_boundary is not None:
            pts = [[int(p.x), int(p.y)] for p in self.last_boundary.points]
            if len(pts) >= 3:
                blank = np.zeros((h, w), dtype=np.uint8)
                cv2.fillPoly(blank, [np.array(pts, np.int32)], 255)
                mask = blank
        else:
            mask[:int(h * self.roi_top_fb), :] = 0
        return mask

    def _publish_debug(self, frame, roi_mask, debug_pass, debug_fail, debug_boundary, h, stamp):
        debug   = frame.copy()
        fw      = debug.shape[1]
        split_y = int(h * self.far_split)

        # Status bar
        cv2.rectangle(debug, (0, 0), (fw, 36), (0, 0, 0), -1)

        # Zone split
        cv2.line(debug, (0, split_y), (fw, split_y), (180, 80, 0), 1)
        cv2.putText(debug, f'FAR (thr={self.hough_thr_far} min={self.hough_min_far})',
                    (4, split_y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 80, 0), 1)
        cv2.putText(debug, f'NEAR (thr={self.hough_thr} min={self.hough_min})',
                    (4, split_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 80, 0), 1)

        # ROI
        ctrs, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, ctrs, -1, (0, 220, 220), 1)

        # [FIX 24] Boundary → ungu
        for d in debug_boundary:
            cv2.line(debug, (d[0],d[1]), (d[2],d[3]), (180, 0, 180), 1)

        # Rejected
        n_gw  = sum(1 for d in debug_fail if d[4]=='gw')
        n_len = sum(1 for d in debug_fail if d[4]=='len')
        for d in debug_fail:
            color = (0,0,200) if d[4]=='gw' else (0,60,140)
            cv2.line(debug, (d[0],d[1]), (d[2],d[3]), color, 1)

        # Passed
        for d in debug_pass:
            x1,y1,x2,y2,conf,vr,is_far = d
            color = (0, int(120+100*conf), int(80*conf)) if is_far else (0, int(150+105*conf), 0)
            cv2.line(debug, (x1,y1), (x2,y2), color, 2)
            mx,my = (x1+x2)//2, (y1+y2)//2
            cv2.putText(debug, f'{vr:.0%}',
                        (mx, my-4), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0,255,160), 1)

        n_pass = len(debug_pass)
        n_bnd  = len(debug_boundary)
        n_fail = len(debug_fail)
        cv2.putText(debug,
                    f'GW: {n_pass} line  {n_bnd} bnd  ({n_fail} rej: {n_gw} GW {n_len} len)',
                    (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.57, (0, 255, 0), 2)
        cv2.putText(debug,
                    f'gw=[{self.gw_vec[0]:.2f},{self.gw_vec[1]:.2f},{self.gw_vec[2]:.2f}]',
                    (fw-255, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160,160,0), 1)

        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            dbg_msg.header.stamp = stamp
            self.pub_debug_img.publish(dbg_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SegmentClassifierGW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
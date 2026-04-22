#!/usr/bin/env python3
"""
field_boundary_detector.py — Dynamic Field Boundary Extraction v2.1
====================================================================
Referensi: Paper 2 (Schulz & Behnke, 2012)

PERUBAHAN v2.1 vs v2.0 (3 failure mode baru dari gambar diperbaiki):
  ──────────────────────────────────────────────────────────────────
  MASALAH v2.0 yang terlihat di gambar:
    A) Kiri: boundary terlalu tinggi (y≈30px) — sky→hijau langsung
    B) Kanan: boundary turun jauh mengikuti arc/circle putih
    C) Tiang gawang: FIX-E sebelumnya tidak cukup efektif

  FIX-1  Ganti scan top-down → BOTTOM-UP dengan ROLLING WINDOW.
         Scan dari bawah per kolom, hitung rolling mean W blok.
         Boundary = baris di mana rolling ratio < scan_ratio
         (transisi dari green ke non-green → tepi atas rumput).
         Jauh lebih robust: tidak tertipu "fraksi hijau ke bawah".

  FIX-2  Morfologi: erode → CLOSE (erode + dilate besar).
         Close mengisi GAP dari garis putih tebal (arc, circle)
         sehingga garis putih tidak dianggap non-green.
         Sequence: erode(5px) → dilate(15px)

  FIX-3  MEDIAN OUTLIER CLAMP sebelum smooth.
         Hitung median boundary seluruh kolom.
         Kolom > median + K blok → clamp ke median+K.
         Memotong spike ke bawah dari arc atau tiang.

  FIX-4  Deteksi tiang gawang via CONNECTED COMPONENTS (blob).
         Cari blob terang (V>185, S<80) dengan aspect ratio h/w > 3.
         Kolom yang terkena blob → interpolasi dari tetangga.

PIPELINE:
  green_mask → erode → close → pool 8×8 → binary
  → scan bottom-up window (FIX-1)
  → goal blob detection + interpolasi (FIX-4)
  → median clamp (FIX-3)
  → smooth + convex_up
  → publish

OUTPUT:
  /field_boundary         — polygon ROI (pixel coords)
  /field_boundary_image   — debug image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge
import cv2
import numpy as np


class FieldBoundaryDetector(Node):
    def __init__(self):
        super().__init__('field_boundary_detector')

        # ── Parameter ────────────────────────────────────────────────
        self.declare_parameter('subsample_x',        8)
        self.declare_parameter('subsample_y',        8)

        # Green HSV
        self.declare_parameter('green_h_low',        35)
        self.declare_parameter('green_h_high',       85)
        self.declare_parameter('green_s_low',        50)
        self.declare_parameter('green_v_low',        50)

        # FIX-2: morfologi
        self.declare_parameter('erode_kernel',       5)   # hilangkan noise kecil
        self.declare_parameter('close_kernel',       15)  # isi gap garis putih tebal

        # Pool
        self.declare_parameter('t_pix',              0.28)

        # FIX-1: scan bottom-up window
        self.declare_parameter('scan_window',        4)    # W blok rolling
        self.declare_parameter('scan_ratio',         0.35) # threshold green ratio

        # FIX-3: median clamp
        self.declare_parameter('median_k',           3)    # blok, max spike dari median

        # FIX-4: tiang gawang blob
        self.declare_parameter('goal_bright_v',      185)
        self.declare_parameter('goal_min_aspect',    2.5)  # h/w ratio
        self.declare_parameter('goal_min_area',      60)
        self.declare_parameter('goal_top_ratio',     0.60) # cek di atas 60% frame

        # Post-processing
        self.declare_parameter('convex_iters',       15)
        self.declare_parameter('smooth_kernel',      9)
        self.declare_parameter('boundary_margin_px', 6)
        self.declare_parameter('min_boundary_row',   0.03)
        self.declare_parameter('max_boundary_row',   0.85)

        self.declare_parameter('publish_debug',      True)

        # ── Baca ─────────────────────────────────────────────────────
        p = self.get_parameter
        self.sub_x         = p('subsample_x').value
        self.sub_y         = p('subsample_y').value
        self.green_h_low   = p('green_h_low').value
        self.green_h_high  = p('green_h_high').value
        self.green_s_low   = p('green_s_low').value
        self.green_v_low   = p('green_v_low').value
        self.erode_k       = p('erode_kernel').value
        self.close_k       = p('close_kernel').value
        self.t_pix         = p('t_pix').value
        self.scan_window   = p('scan_window').value
        self.scan_ratio    = p('scan_ratio').value
        self.median_k      = p('median_k').value
        self.goal_bright_v = p('goal_bright_v').value
        self.goal_min_asp  = p('goal_min_aspect').value
        self.goal_min_area = p('goal_min_area').value
        self.goal_top_ratio= p('goal_top_ratio').value
        self.convex_iters  = p('convex_iters').value
        self.smooth_k      = p('smooth_kernel').value
        self.margin        = p('boundary_margin_px').value
        self.min_bnd       = p('min_boundary_row').value
        self.max_bnd       = p('max_boundary_row').value
        self.pub_debug     = p('publish_debug').value

        self.bridge = CvBridge()
        self.last_boundary_px = None

        # ── Sub / Pub ────────────────────────────────────────────────
        self.sub_image = self.create_subscription(
            Image, '/robotis_op3/camera/image_raw',
            self._cb_image, qos_profile_sensor_data)
        self.pub_boundary  = self.create_publisher(Polygon, '/field_boundary', 10)
        self.pub_debug_img = self.create_publisher(Image,  '/field_boundary_image', 10)

        self.get_logger().info('FieldBoundaryDetector v2.1 started')

    # ─────────────────────────────────────────────────────────────────
    def _cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}', throttle_duration_sec=5.0)
            return

        h, w = frame.shape[:2]
        boundary_px = self._extract_boundary(frame, h, w)

        if boundary_px is None:
            boundary_px = (self.last_boundary_px
                           if self.last_boundary_px is not None
                           else np.full(w // self.sub_x, int(h * 0.4), np.int32))
        else:
            self.last_boundary_px = boundary_px.copy()

        # Polygon
        poly   = Polygon()
        n_cols = len(boundary_px)
        for c, ry in enumerate(boundary_px):
            poly.points.append(Point32(
                x=float(c * self.sub_x + self.sub_x // 2),
                y=float(min(ry + self.margin, h - 1)),
                z=0.0))
        poly.points.append(Point32(x=float(w - 1), y=float(h - 1), z=0.0))
        poly.points.append(Point32(x=0.0,           y=float(h - 1), z=0.0))
        self.pub_boundary.publish(poly)

        if self.pub_debug:
            self._publish_debug(frame, boundary_px, msg.header.stamp)

    # ─────────────────────────────────────────────────────────────────
    def _extract_boundary(self, frame, h, w):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ── Green mask ───────────────────────────────────────────────
        green_mask = cv2.inRange(
            hsv,
            np.array([self.green_h_low,  self.green_s_low,  self.green_v_low]),
            np.array([self.green_h_high,  255,               255])
        )

        # FIX-2: erode → close (dilate besar)
        if self.erode_k > 1:
            ek = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.erode_k, self.erode_k))
            green_mask = cv2.erode(green_mask, ek, iterations=1)
        if self.close_k > 1:
            # Close = dilate setelah erode, untuk mengisi gap garis putih tebal
            ck = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self.close_k, self.close_k))
            green_mask = cv2.dilate(green_mask, ck, iterations=1)

        field_score = green_mask.astype(np.float32) / 255.0

        # ── Pool 8×8 ─────────────────────────────────────────────────
        n_cols = w // self.sub_x
        n_rows = h // self.sub_y
        fs_crop = field_score[:n_rows * self.sub_y, :n_cols * self.sub_x]
        pool = (fs_crop
                .reshape(n_rows, self.sub_y, n_cols, self.sub_x)
                .mean(axis=(1, 3))
                .astype(np.float32))
        binary = (pool >= self.t_pix).astype(np.uint8)

        # ── FIX-1: Scan bottom-up rolling window ─────────────────────
        boundary_sub = self._scan_bottomup(binary, n_rows, n_cols)

        # ── FIX-4: Goal post blob detection ──────────────────────────
        goal_cols = self._detect_goal_cols(hsv, n_cols, h, w)
        if goal_cols.any():
            boundary_sub = self._interpolate_cols(boundary_sub, goal_cols, n_cols)

        # ── FIX-3: Median outlier clamp ──────────────────────────────
        boundary_sub = self._median_clamp(boundary_sub)

        # ── Smooth ───────────────────────────────────────────────────
        if self.smooth_k > 1:
            kernel = np.ones(self.smooth_k, np.float32) / self.smooth_k
            boundary_sub = np.convolve(
                boundary_sub.astype(np.float32), kernel, mode='same'
            ).astype(np.int32)

        # ── Convex up ────────────────────────────────────────────────
        boundary_sub = self._convex_up(boundary_sub, n_cols)

        # ── Clamp + ke pixel ─────────────────────────────────────────
        min_r = int(self.min_bnd * n_rows)
        max_r = int(self.max_bnd * n_rows)
        boundary_sub = np.clip(boundary_sub, min_r, max_r)
        return (boundary_sub * self.sub_y).astype(np.int32)

    # ─────────────────────────────────────────────────────────────────
    def _scan_bottomup(self, binary, n_rows, n_cols):
        """
        FIX-1: Scan dari bawah ke atas.
        Rolling mean W blok terakhir.
        Boundary = baris di mana mean < scan_ratio
        (transisi dari green ke non-green, arah bottom-up).
        """
        W = max(self.scan_window, 1)
        boundary = np.zeros(n_cols, dtype=np.int32)
        for c in range(n_cols):
            col = binary[:, c].astype(np.float32)
            found = False
            for r in range(n_rows - 1, W - 1, -1):
                w_mean = col[r - W + 1: r + 1].mean()
                if w_mean < self.scan_ratio:
                    # Non-green zone → boundary tepat di baris ini + 1
                    boundary[c] = min(r + 1, n_rows - 1)
                    found = True
                    break
            if not found:
                boundary[c] = 0  # Semua hijau → boundary paling atas
        return boundary

    # ─────────────────────────────────────────────────────────────────
    def _detect_goal_cols(self, hsv, n_cols, h, w):
        """
        FIX-4: Deteksi tiang gawang via connected components.
        Blob terang (V>goal_bright_v, S<80) dengan h/w > goal_min_asp.
        """
        goal_cols = np.zeros(n_cols, dtype=bool)
        top_h = int(h * self.goal_top_ratio)
        if top_h < 10:
            return goal_cols

        v_ch = hsv[:top_h, :, 2]
        s_ch = hsv[:top_h, :, 1]
        bright = ((v_ch > self.goal_bright_v) & (s_ch < 80)).astype(np.uint8) * 255

        # Close agar tiang yang terputus terhubung
        ck = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9))
        bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, ck)

        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(bright)
        for lbl in range(1, n_labels):
            bw   = stats[lbl, cv2.CC_STAT_WIDTH]
            bh   = stats[lbl, cv2.CC_STAT_HEIGHT]
            area = stats[lbl, cv2.CC_STAT_AREA]
            x0   = stats[lbl, cv2.CC_STAT_LEFT]

            if area < self.goal_min_area:
                continue
            if (bh / max(bw, 1)) < self.goal_min_asp:
                continue

            c_start = max(0, x0 // self.sub_x)
            c_end   = min(n_cols, (x0 + bw) // self.sub_x + 1)
            goal_cols[c_start:c_end] = True

        return goal_cols

    # ─────────────────────────────────────────────────────────────────
    def _interpolate_cols(self, boundary, mask_cols, n_cols):
        """Isi kolom yang di-mask dengan interpolasi linear dari tetangga."""
        result = boundary.copy()
        i = 0
        while i < n_cols:
            if mask_cols[i]:
                j = i
                while j < n_cols and mask_cols[j]:
                    j += 1
                lv = result[i - 1] if i > 0     else (result[j] if j < n_cols else 0)
                rv = result[j]     if j < n_cols else lv
                for k in range(i, j):
                    t = (k - i + 0.5) / max(j - i, 1)
                    result[k] = int(lv * (1 - t) + rv * t)
                i = j
            else:
                i += 1
        return result

    # ─────────────────────────────────────────────────────────────────
    def _median_clamp(self, boundary):
        """
        FIX-3: Potong outlier yang terlalu jauh di bawah median.
        Dalam satuan blok (sub-unit).
        """
        med = np.median(boundary)
        upper = med + self.median_k
        return np.where(boundary > upper, int(upper), boundary).astype(np.int32)

    # ─────────────────────────────────────────────────────────────────
    def _convex_up(self, boundary, n_cols):
        """Iteratif tarik titik yang menjorok ke bawah ke atas."""
        b = boundary.astype(np.float32)
        for _ in range(self.convex_iters):
            changed = False
            nb = b.copy()
            for i in range(1, n_cols - 1):
                interp = (b[i - 1] + b[i + 1]) / 2.0
                if b[i] > interp:
                    nb[i] = interp
                    changed = True
            b = nb
            if not changed:
                break
        return b.astype(np.int32)

    # ─────────────────────────────────────────────────────────────────
    def _publish_debug(self, frame, boundary_px, stamp):
        debug  = frame.copy()
        h, w   = debug.shape[:2]
        n_cols = len(boundary_px)

        # Overlay
        mask = np.zeros((h, w), dtype=np.uint8)
        pts  = []
        for c in range(n_cols):
            x = int(c * self.sub_x + self.sub_x // 2)
            y = int(min(boundary_px[c] + self.margin, h - 1))
            pts.append([x, y])
        pts.append([w - 1, h - 1])
        pts.append([0, h - 1])
        pts = np.array(pts, dtype=np.int32)
        cv2.fillPoly(mask, [pts], 255)

        ov = debug.copy()
        ov[mask > 0] = (
            ov[mask > 0].astype(np.float32) * 0.7 +
            np.array([0, 60, 0], dtype=np.float32) * 0.3
        ).astype(np.uint8)
        cv2.addWeighted(ov, 0.65, debug, 0.35, 0, debug)

        for c in range(n_cols - 1):
            x1 = int(c * self.sub_x + self.sub_x // 2)
            x2 = int((c + 1) * self.sub_x + self.sub_x // 2)
            y1 = int(min(boundary_px[c] + self.margin, h - 1))
            y2 = int(min(boundary_px[c + 1] + self.margin, h - 1))
            cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.putText(debug, 'FIELD BOUNDARY (dynamic)', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        try:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            dbg_msg.header.stamp = stamp
            self.pub_debug_img.publish(dbg_msg)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FieldBoundaryDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
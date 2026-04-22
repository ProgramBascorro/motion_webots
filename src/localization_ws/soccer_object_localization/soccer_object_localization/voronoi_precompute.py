#!/usr/bin/env python3
"""
voronoi_precompute.py — Voronoi LUT Precomputation untuk Cox Registration
=========================================================================
Referensi: Paper 3 (Whelan dkk.) — "Line Point Registration"

Precompute Voronoi diagram dari peta lapangan (PGM/image),
sehingga "closest field feature" bisa dilookup O(1) saat runtime.

CARA KERJA (Paper 3 Section 4):
  1. Load peta lapangan (PGM occupancy grid)
  2. Ekstrak semua piksel "garis lapangan" (auto-detect terang/gelap)
  3. Hitung distance transform → setiap piksel tahu jarak ke garis terdekat
  4. Hitung normal vector field via gradien distance transform
  5. Hitung r_offset: dot(q_closest, normal) untuk Cox equation
  6. Simpan sebagai .npz

PENGGUNAAN:
  python3 voronoi_precompute.py \
    --map_path  maps/soccer_field_kidsize.pgm \
    --yaml_path maps/soccer_field_kidsize.yaml \
    --output    config/voronoi_lut

CHANGELOG:
  [FIX 1] Auto-detect apakah garis terang atau gelap di PGM
          (sebelumnya selalu asumsi garis = pixel terang → salah untuk
          peta kidsize yang memakai pixel 0 = garis, 205 = background)
  [FIX 2] Default yaml_path diupdate ke soccer_field_kidsize.yaml
  [FIX 3] Visualisasi PNG hasil LUT disimpan berdampingan dengan .npz
          untuk verifikasi cepat tanpa RViz
"""

import argparse
import numpy as np
import cv2
import yaml
import os
import sys


# ─────────────────────────────────────────────────────────────
# YAML loader
# ─────────────────────────────────────────────────────────────

def load_map_yaml(yaml_path):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    resolution  = float(data['resolution'])
    origin      = data['origin']           # [x, y, theta]
    negate      = int(data.get('negate', 0))
    occ_thresh  = float(data.get('occupied_thresh', 0.65))
    free_thresh = float(data.get('free_thresh', 0.196))
    return resolution, origin, negate, occ_thresh, free_thresh


# ─────────────────────────────────────────────────────────────
# AUTO-DETECT: apakah garis terang atau gelap
# ─────────────────────────────────────────────────────────────

def _auto_detect_line_mask(img: np.ndarray, line_threshold: int) -> np.ndarray:
    """
    [FIX 1] Deteksi otomatis apakah garis lapangan = piksel GELAP atau TERANG.

    Strategi:
    - Hitung n_dark  = piksel < line_threshold
    - Hitung n_light = piksel > (255 - line_threshold)
    - Ambil yang lebih sedikit sebagai "garis"
      (garis lapangan biasanya lebih sempit dari background)
    - Jika ambiguous, gunakan dark (ROS occupancy map konvensi: occupied = gelap)
    """
    n_dark  = int((img <  line_threshold).sum())
    n_light = int((img > (255 - line_threshold)).sum())
    total   = img.size

    print(f'  Auto-detect: n_dark={n_dark} ({100*n_dark/total:.1f}%)  '
          f'n_light={n_light} ({100*n_light/total:.1f}%)')

    if n_dark == 0 and n_light == 0:
        print('  WARNING: Tidak ada piksel yang memenuhi threshold!')
        print(f'  Pixel range di PGM: min={img.min()} max={img.max()}')
        print('  Coba turunkan --line_thr atau periksa file PGM.')
        sys.exit(1)

    if n_light == 0 or (n_dark > 0 and n_dark <= n_light):
        mode = 'dark'
        line_mask = (img < line_threshold).astype(np.uint8) * 255
        print(f'  Mode: garis GELAP (pixel < {line_threshold})  → {n_dark} piksel')
    else:
        mode = 'light'
        line_mask = (img > (255 - line_threshold)).astype(np.uint8) * 255
        print(f'  Mode: garis TERANG (pixel > {255-line_threshold})  → {n_light} piksel')

    return line_mask, mode


# ─────────────────────────────────────────────────────────────
# MAIN COMPUTATION
# ─────────────────────────────────────────────────────────────

def compute_voronoi_lut(pgm_path: str, yaml_path: str,
                        output_path: str, line_threshold: int = 100):
    """
    Hitung Voronoi LUT dari peta lapangan PGM + YAML metadata.

    Args:
        pgm_path:       Path ke file .pgm
        yaml_path:      Path ke file .yaml (metadata ROS map)
        output_path:    Path output tanpa ekstensi (akan ditambah .npz)
        line_threshold: Threshold untuk deteksi garis (0-255)
    """
    # ── Load PGM ──────────────────────────────────────────────
    print(f'Loading map: {pgm_path}')
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f'ERROR: Tidak bisa load {pgm_path}')
        sys.exit(1)

    h, w = img.shape
    print(f'  Map size  : {w} × {h} pixels')
    print(f'  Pixel range: min={img.min()} max={img.max()}')

    # ── Load YAML ─────────────────────────────────────────────
    print(f'Loading YAML: {yaml_path}')
    resolution, origin, negate, _, _ = load_map_yaml(yaml_path)
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    print(f'  Resolution: {resolution:.4f} m/px')
    print(f'  Origin    : ({origin_x:.3f}, {origin_y:.3f}) m')
    print(f'  Coverage  : {w*resolution:.2f} × {h*resolution:.2f} m')
    print(f'  Negate    : {negate}')

    if negate:
        img = 255 - img
        print('  Applied negate (255 - img)')

    # ── [FIX 1] Auto-detect garis terang/gelap ────────────────
    line_mask, detect_mode = _auto_detect_line_mask(img, line_threshold)
    n_line = int((line_mask > 0).sum())
    print(f'  Line pixels: {n_line} ({100*n_line/(h*w):.2f}%)')

    # ── Distance Transform ────────────────────────────────────
    print('Computing distance transform...')
    # distanceTransform mengukur jarak ke piksel bernilai 0
    # → invert: garis=0, background=255
    inv_mask = 255 - line_mask
    dist_px  = cv2.distanceTransform(inv_mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    dist_m   = dist_px.astype(np.float32) * resolution

    print(f'  Distance: max={dist_m.max():.3f} m  mean={dist_m.mean():.3f} m')

    # ── Normal Vector Field ───────────────────────────────────
    print('Computing normal vectors...')
    grad_x   = cv2.Sobel(dist_px, cv2.CV_32F, 1, 0, ksize=3)
    grad_y   = cv2.Sobel(dist_px, cv2.CV_32F, 0, 1, ksize=3)
    grad_mag = np.sqrt(grad_x**2 + grad_y**2) + 1e-10

    # Normal mengarah KE garis terdekat (bukan menjauh)
    normal_x = (-grad_x / grad_mag).astype(np.float32)
    normal_y = (-grad_y / grad_mag).astype(np.float32)

    # Di piksel garis itu sendiri: normal tidak terdefinisi, set default
    normal_x[line_mask > 0] = 1.0
    normal_y[line_mask > 0] = 0.0

    # ── r_offset untuk Cox Equation ──────────────────────────
    # r_i = dot(q_i, u_i), di mana q_i = titik garis terdekat
    # q_i = p + dist * normal  (dalam world frame)
    print('Computing r_offset...')
    cols    = np.arange(w, dtype=np.float32)
    rows    = np.arange(h, dtype=np.float32)
    world_xx = origin_x + np.tile(cols,           (h, 1)) * resolution
    world_yy = origin_y + np.tile((h-1-rows).reshape(-1,1), (1, w)) * resolution

    q_x      = world_xx + dist_m * normal_x
    q_y      = world_yy + dist_m * normal_y
    r_offset = (q_x * normal_x + q_y * normal_y).astype(np.float32)

    print(f'  r_offset range: [{r_offset.min():.3f}, {r_offset.max():.3f}]')

    # ── Validasi titik-titik landmark lapangan ────────────────
    print('\nValidasi landmark:')
    landmarks = [
        ( 0.0,  0.0, 'center'),
        ( 4.5,  0.0, 'goal_line_right'),
        (-4.5,  0.0, 'goal_line_left'),
        ( 0.0,  3.0, 'side_line_top'),
        ( 0.0, -3.0, 'side_line_bottom'),
    ]
    all_ok = True
    for lx, ly, label in landmarks:
        col = int((lx - origin_x) / resolution)
        row = int(h - 1 - (ly - origin_y) / resolution)
        if 0 <= col < w and 0 <= row < h:
            d = float(dist_m[row, col])
            status = '✓' if d < 0.10 else f'⚠ dist={d:.3f}m'
            print(f'  {label:22s} ({lx:5.1f},{ly:5.1f}) → dist={d:.4f}m  {status}')
            if d > 0.10:
                all_ok = False
        else:
            print(f'  {label:22s} → OUT OF MAP')
            all_ok = False

    if not all_ok:
        print('\nWARNING: Beberapa landmark jauh dari garis. '
              'Cek apakah origin/resolution YAML sudah benar.')

    # ── Simpan LUT ────────────────────────────────────────────
    if not output_path.endswith('.npz'):
        save_path = output_path + '.npz'
    else:
        save_path = output_path

    print(f'\nSaving LUT → {save_path}')
    np.savez_compressed(
        save_path,
        distance   = dist_m,
        normal_x   = normal_x,
        normal_y   = normal_y,
        r_offset   = r_offset,
        origin_x   = np.float64(origin_x),
        origin_y   = np.float64(origin_y),
        resolution = np.float64(resolution),
        map_width  = np.int32(w),
        map_height = np.int32(h),
    )
    size_mb = os.path.getsize(save_path) / 1e6
    print(f'  Saved: {size_mb:.2f} MB')

    # ── [FIX 3] Simpan visualisasi PNG ───────────────────────
    _save_visualization(img, dist_m, normal_x, normal_y, line_mask,
                        origin_x, origin_y, resolution,
                        output_path.replace('.npz','') + '_viz.png')

    print('\n✓ Voronoi LUT selesai.')
    print(f'  Load di cox_registration.py dengan:')
    print(f"    voronoi_lut_path: '{save_path}'")


def _save_visualization(img, dist_m, nx, ny, line_mask,
                        origin_x, origin_y, resolution, out_png):
    """[FIX 3] Simpan 3-panel visualization untuk verifikasi cepat."""
    h, w = dist_m.shape

    # Panel 1: garis terdeteksi (merah) + landmark (lingkaran)
    p1 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    p1[line_mask > 0] = [0, 0, 200]
    for lx, ly, color in [
        ( 0.0,  0.0, (0,255,255)),
        ( 4.5,  0.0, (0,255,0)),
        (-4.5,  0.0, (0,255,0)),
        ( 0.0,  3.0, (255,100,0)),
        ( 0.0, -3.0, (255,100,0)),
    ]:
        col = int((lx - origin_x) / resolution)
        row = int(h - 1 - (ly - origin_y) / resolution)
        if 0 <= col < w and 0 <= row < h:
            cv2.circle(p1, (col, row), 5, color, -1)

    # Panel 2: distance heatmap
    d_norm = (dist_m / max(dist_m.max(), 1e-6) * 255).astype(np.uint8)
    p2     = cv2.applyColorMap(d_norm, cv2.COLORMAP_JET)

    # Panel 3: normal vectors (area dekat garis saja)
    p3 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    step = 20
    for row in range(step//2, h, step):
        for col in range(step//2, w, step):
            if float(dist_m[row, col]) < 0.5:
                ux = float(nx[row, col])
                uy = float(ny[row, col])
                x2 = max(0, min(w-1, int(col + ux * 8)))
                y2 = max(0, min(h-1, int(row - uy * 8)))
                cv2.arrowedLine(p3, (col, row), (x2, y2), (0,220,0), 1, tipLength=0.4)

    panel = np.hstack([p1, p2, p3])
    labels = ['Lines + Landmarks', 'Distance Map', 'Normal Vectors (d<0.5m)']
    for i, label in enumerate(labels):
        cv2.putText(panel, label, (i*w + 8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    cv2.imwrite(out_png, panel)
    print(f'  Visualization: {out_png}')


# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────

def main():
    base = 'src/motion_webots/src/localization_ws/soccer_object_localization'

    parser = argparse.ArgumentParser(
        description='Precompute Voronoi LUT untuk Cox Registration (Paper 3)')
    parser.add_argument('--map_path',
        default=f'{base}/maps/soccer_field_kidsize.pgm',
        help='Path ke file PGM peta lapangan')
    parser.add_argument('--yaml_path',          # [FIX 2] default diupdate
        default=f'{base}/maps/soccer_field_kidsize.yaml',
        help='Path ke file YAML metadata peta')
    parser.add_argument('--output',
        default=f'{base}/config/voronoi_lut',
        help='Path output tanpa .npz')
    parser.add_argument('--line_thr', type=int, default=100,
        help='Threshold deteksi garis (0-255, default 100). '
             'Auto-detect terang/gelap dari histogram.')
    args = parser.parse_args()

    compute_voronoi_lut(args.map_path, args.yaml_path, args.output, args.line_thr)


if __name__ == '__main__':
    main()
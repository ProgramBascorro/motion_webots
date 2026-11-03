"""Batas tegak sapuan kepala: angka nyata di head_tracking.yaml + penjepitnya.

Tanpa ROS: yang diuji hanya pola sapuan murni (head_scan_pattern.py) dan
berkas konfigurasi yang dimuat semua peluncur head_tracking_node.
"""

import math
import pathlib

import pytest
import yaml

from op3_ball_localization.head_scan_pattern import (
    SCAN_TILT_UPPER_MIN_RAD,
    build_scan_positions,
    resolve_scan_tilt_upper,
)

CONFIG = pathlib.Path(__file__).resolve().parents[1] / "config" / "head_tracking.yaml"


def params():
    with open(CONFIG) as f:
        return yaml.safe_load(f)["head_tracking_node"]["ros__parameters"]


def test_real_config_upper_is_higher_than_lower_and_inside_verified_range():
    p = params()
    upper, lower = p["scan_tilt_upper_rad"], p["scan_tilt_down_rad"]
    used, problem = resolve_scan_tilt_upper(upper, lower)
    assert problem is None
    assert used == upper
    # POSITIF = MENUNDUK, jadi batas atas harus angka yang lebih KECIL.
    assert SCAN_TILT_UPPER_MIN_RAD <= upper < lower
    # Batas sendi head_control_module (-30 .. +75 deg): tidak ada yang dijepit diam-diam.
    for tilt in (upper, lower):
        assert math.radians(-30.0) <= tilt <= math.radians(75.0)


def test_real_config_values_of_this_change():
    p = params()
    assert p["scan_tilt_down_rad"] == 0.4391          # batas bawah TIDAK berubah
    assert p["scan_tilt_upper_rad"] == -0.12          # batas atas 2026-09-15
    assert p["scan_pan_rad"] == 0.9599                # servo 19 kiri-kanan tidak berubah
    assert p["scan_pan_down_ratio"] == 1.0
    # Dua nama lama tidak boleh tertinggal: node tidak membacanya lagi, jadi
    # mengubahnya tidak akan berefek apa pun.
    assert "scan_tilt_forward_rad" not in p
    assert "scan_tilt_down_side_rad" not in p


def test_only_the_upper_rows_move_compared_with_the_previous_pattern():
    p = params()
    pan, ratio = p["scan_pan_rad"], p["scan_pan_down_ratio"]
    before = build_scan_positions(pan, ratio, 0.1745, 0.4391)   # forward = down_side = 0,1745
    after = build_scan_positions(pan, ratio, p["scan_tilt_upper_rad"], p["scan_tilt_down_rad"])
    assert len(before) == len(after) == 5
    assert [pt[0] for pt in after] == [pt[0] for pt in before]  # semua pan sama
    assert after[0] == before[0] == (0.0, 0.4391)               # titik terendah sama persis
    assert max(t for _, t in after) == max(t for _, t in before) == 0.4391
    for (_, t_before), (_, t_after) in zip(before[1:], after[1:]):
        assert t_after < t_before                               # lebih mendongak
    assert min(t for _, t in after) == p["scan_tilt_upper_rad"]


def test_pattern_shape_front_uses_lower_and_pan_ends_use_upper():
    pts = build_scan_positions(0.9599, 1.0, -0.12, 0.4391)
    assert pts == [
        (0.0, 0.4391),
        (-0.9599, -0.12),
        (-0.9599, -0.12),
        (0.9599, -0.12),
        (0.9599, -0.12),
    ]


def test_upper_beyond_verified_mechanical_range_is_clamped():
    used, problem = resolve_scan_tilt_upper(math.radians(-40.0), 0.4391)
    assert used == SCAN_TILT_UPPER_MIN_RAD
    assert problem and "mekanis" in problem


def test_upper_exactly_at_verified_limit_is_accepted():
    used, problem = resolve_scan_tilt_upper(SCAN_TILT_UPPER_MIN_RAD, 0.4391)
    assert used == SCAN_TILT_UPPER_MIN_RAD
    assert problem is None


def test_upper_lower_than_the_lower_limit_is_clamped_to_it():
    used, problem = resolve_scan_tilt_upper(0.60, 0.4391)
    assert used == 0.4391
    assert problem and "batas bawah" in problem


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_upper_never_points_the_head_somewhere_unknown(bad):
    used, problem = resolve_scan_tilt_upper(bad, 0.4391)
    assert used == 0.4391
    assert problem

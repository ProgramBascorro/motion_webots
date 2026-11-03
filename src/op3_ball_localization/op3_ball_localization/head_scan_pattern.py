"""Pola sapuan kepala head_tracking_node, tanpa ROS supaya bisa diuji langsung.

ARAH head_tilt DI ROBOT INI: POSITIF = MENUNDUK, NEGATIF = MENDONGAK.
Diverifikasi 2026-09-15 dalam satu run, kepala saja (pan 0, badan diam): frame
kamera pada head_tilt terbaca -6,5 / +10,1 / +25,3 deg bergeser dari lantai
2-3 m di depan ke lantai tepat di depan kaki. Offset head_tilt di offset.yaml
nol, jadi derajat servo 20 = 180 + sudut sendi. Akibatnya:

    batas BAWAH sapuan (paling menunduk)  = sudut TERBESAR  = scan_tilt_down_rad
    batas ATAS  sapuan (paling mendongak) = sudut TERKECIL  = scan_tilt_upper_rad

dan kepala melihat LEBIH TINGGI kalau scan_tilt_upper_rad DIKURANGI.
"""

import math
from typing import List, Optional, Tuple

# Sudut paling mendongak yang SUDAH terbukti dicapai kepala: servo 20 terbaca
# 150,82 deg (head_tilt -29,18 deg) di pose INIT D58 2026-09-15, lalu kepala
# bergerak normal lagi. head_control_module sendiri menjepit perintah di
# head_tilt_min_deg -30 deg. Batas atas sapuan tidak boleh melewati angka ini.
SCAN_TILT_UPPER_MIN_RAD = math.radians(-29.0)


def resolve_scan_tilt_upper(
    upper_rad: float, lower_rad: float
) -> Tuple[float, Optional[str]]:
    """Sudut batas atas yang dipakai, dan alasan kalau angka konfigurasi ditolak.

    Rentang sah: SCAN_TILT_UPPER_MIN_RAD .. lower_rad. Batas bawah tidak pernah
    diubah di sini; angka di luar rentang dijepit ke ujung terdekat.
    """
    if not math.isfinite(upper_rad):
        return lower_rad, (
            "scan_tilt_upper_rad bukan angka berhingga (%r); dipakai batas bawah "
            "%+.4f rad supaya kepala tidak mendongak ke sudut yang tidak diketahui"
            % (upper_rad, lower_rad)
        )
    if upper_rad < SCAN_TILT_UPPER_MIN_RAD:
        used = min(SCAN_TILT_UPPER_MIN_RAD, lower_rad)
        return used, (
            "scan_tilt_upper_rad %+.4f rad (%+.1f deg) lebih mendongak dari batas "
            "mekanis terverifikasi %+.4f rad (%+.1f deg); dijepit ke %+.4f rad"
            % (upper_rad, math.degrees(upper_rad), SCAN_TILT_UPPER_MIN_RAD,
               math.degrees(SCAN_TILT_UPPER_MIN_RAD), used)
        )
    if upper_rad > lower_rad:
        return lower_rad, (
            "scan_tilt_upper_rad %+.4f rad (%+.1f deg) lebih MENUNDUK dari batas "
            "bawah scan_tilt_down_rad %+.4f rad (%+.1f deg); dijepit ke batas bawah"
            % (upper_rad, math.degrees(upper_rad), lower_rad, math.degrees(lower_rad))
        )
    return upper_rad, None


def build_scan_positions(
    pan_rad: float, pan_down_ratio: float, tilt_upper_rad: float, tilt_lower_rad: float
) -> List[Tuple[float, float]]:
    """Lima titik sapuan (pan, tilt) dalam radian, searah keliling.

    Titik depan (pan 0) memakai batas BAWAH; keempat titik ujung pan memakai
    batas ATAS. Sasaran pan ujung sengaja muncul dua kali berturut-turut supaya
    kepala sempat sampai dan diam di sana (lihat scan_period_sec).
    """
    p = pan_rad
    pd = pan_rad * pan_down_ratio
    return [
        (0.0, tilt_lower_rad),
        (-pd, tilt_upper_rad),
        (-p, tilt_upper_rad),
        (+p, tilt_upper_rad),
        (+pd, tilt_upper_rad),
    ]

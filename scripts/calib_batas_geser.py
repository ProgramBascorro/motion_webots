#!/usr/bin/env python3
"""Kalibrasi batas geser kiper D58: waktu efektif geser dari TENGAH ke TEPI area kiper.

Area kiper lebar 1,8 m (0,9 m tiap sisi). Operator menjalankan versi "geser kanan" /
"geser kiri" lewat BASCORRO Studio dari tengah sampai mendekati tepi (tandai dengan
selotip). Skrip ini HANYA MENDENGAR -- tidak pernah memerintah robot -- lalu menulis
data/kalibrasi_batas_geser.yaml yang dibaca demo D58 (mode kiper) sebagai batas geser.

WAKTU EFEKTIF (operator 2026-11-11)
  Tiap geser diawali soft start: skala amplitudo naik LINEAR 0 -> 1 selama soft_start_time
  (op3_walking_module.cpp). Waktu efektif = integral skala itu dari "start" sampai "stop"
  (+ ramp turun 1/2 x soft_start untuk "soft_stop"). Demo menghitung dengan rumus yang SAMA,
  jadi hasilnya berlaku walau demo memotong geser jadi banyak segmen pendek.

EKOR PER SEGMEN (fase opsional "putus")
  Sesudah "stop" kaki masih menyelesaikan langkahnya -- gerak yang tak terhitung, dan
  menumpuk di demo yang geser terputus-putus. Rekam lintasan tengah->tepi YANG SAMA dua
  kali: sekali "terus" (sekali jalan) dan sekali "putus" (banyak start/stop ~1 dtk):
      E_terus + n_terus*k = E_putus + n_putus*k   ->   k = ekor per segmen (dtk efektif)
  Tombol Stop Studio mengirim "stop" biasa (amplitudo TIDAK dinolkan dulu), ekornya sedikit
  lebih panjang dari stop demo -> k dari sini sedikit melebihkan = aman (berhenti lebih awal).

Langkah operator (container, ROS_DOMAIN_ID=1; op3_manager jalan, D58 TIDAK aktif):
    python3 scripts/calib_batas_geser.py rekam --arah kanan --mode terus
    python3 scripts/calib_batas_geser.py rekam --arah kiri  --mode terus
    python3 scripts/calib_batas_geser.py rekam --arah kanan --mode putus   # disarankan
    python3 scripts/calib_batas_geser.py rekam --arah kiri  --mode putus   # disarankan
Tiap `rekam` langsung menghitung ulang YAML. Subperintah lain: hitung, status, hapus,
uji-sintetis.
"""
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(REPO, "src", "ROBOTIS-OP3-Demo", "op3_demo", "data")
SAMPLE_FILE = os.path.join(DATA_DIR, "kalibrasi_batas_geser_sampel.jsonl")
RESULT_FILE = os.path.join(DATA_DIR, "kalibrasi_batas_geser.yaml")
VERSIONS_FILE = os.path.join(REPO, "src", "ROBOTIS-OP3", "op3_action_module", "data", "walking_versions.json")

SETENGAH_LEBAR_M = 0.9          # area kiper 1,8 m
Y_MIN = 1e-4                    # |y_move| di bawah ini = tidak geser
ARAH_TANDA = {"kiri": +1, "kanan": -1}   # y_move + = kiri (konvensi OP3)
NAMA_VERSI = {"kiri": "geser kiri", "kanan": "geser kanan"}
MIN_SELISIH_SEGMEN = 3          # n_putus - n_terus minimum agar ekor bisa dihitung


# == matematika (dipakai rekaman langsung DAN uji sintetis) ==================

def integral_skala(u0, u1, soft):
    """Integral min(1, u/soft) du dari u0 ke u1 (u = waktu sejak start)."""
    if u1 <= u0:
        return 0.0
    if soft <= 0.0:
        return u1 - u0

    def F(u):
        u = max(0.0, u)
        return u * u / (2.0 * soft) if u <= soft else soft / 2.0 + (u - soft)
    return F(u1) - F(u0)


def skala_pada(u, soft):
    return 1.0 if soft <= 0.0 else min(1.0, max(0.0, u / soft))


class Penghitung:
    """Mengintegrasikan waktu efektif geser per arah dari urutan peristiwa walking."""

    def __init__(self, soft=0.0, y=0.0, period=0.0):
        self.soft = soft
        self.y = y
        self.period = period
        self.jalan = False
        self.t_start = 0.0
        self.t_akhir = 0.0
        self.seg = None
        self.segmen = []                       # segmen yang sudah selesai

    def _arah(self):
        if abs(self.y) < Y_MIN:
            return None
        return "kiri" if self.y > 0 else "kanan"

    def _majukan(self, t):
        if self.jalan and t > self.t_akhir:
            arah = self._arah()
            e = integral_skala(self.t_akhir - self.t_start, t - self.t_start, self.soft)
            if arah is not None:
                self.seg["efektif"][arah] += e
                self.seg["mentah"][arah] += t - self.t_akhir
            self.t_akhir = t

    def params(self, t, y, period):
        self._majukan(t)
        self.y = y
        self.period = period

    def set_soft(self, t, soft):
        self._majukan(t)
        self.soft = soft

    def start(self, t):
        if self.jalan:
            return False                       # walking_module mengabaikan start ganda
        self.jalan = True
        self.t_start = self.t_akhir = t
        self.seg = {"t0": t, "efektif": {"kiri": 0.0, "kanan": 0.0},
                    "mentah": {"kiri": 0.0, "kanan": 0.0}, "soft": self.soft,
                    "y": self.y, "period": self.period, "akhir": None}
        return True

    def stop(self, t, lunak=False):
        if not self.jalan:
            return None
        self._majukan(t)
        if lunak and self.soft > 0.0:
            # soft_stop: skala turun linear dari nilai sekarang ke 0 selama soft_start_time.
            arah = self._arah()
            ekor = skala_pada(t - self.t_start, self.soft) * self.soft / 2.0
            if arah is not None:
                self.seg["efektif"][arah] += ekor
                self.seg["mentah"][arah] += self.soft
        self.seg["akhir"] = "soft_stop" if lunak else "stop"
        self.seg["t1"] = t
        self.jalan = False
        self.segmen.append(self.seg)
        s, self.seg = self.seg, None
        return s

    def ringkas(self, arah, dari=0):
        """(efektif, mentah, jumlah segmen, efektif arah lain) segmen ke-`dari` dst."""
        segs = self.segmen[dari:]
        lain = "kanan" if arah == "kiri" else "kiri"
        e = sum(s["efektif"][arah] for s in segs)
        m = sum(s["mentah"][arah] for s in segs)
        n = sum(1 for s in segs if s["efektif"][arah] > 0.0)
        e_lain = sum(s["efektif"][lain] for s in segs)
        return e, m, n, e_lain

    def efektif_berjalan(self, t):
        """Waktu efektif segmen yang SEDANG berjalan sampai t (tanpa mengubah keadaan)."""
        if not self.jalan:
            return 0.0, None
        arah = self._arah()
        if arah is None:
            return 0.0, None
        sisa = integral_skala(self.t_akhir - self.t_start, t - self.t_start, self.soft)
        return self.seg["efektif"][arah] + sisa, arah


def hitung_ekor(terus, putus):
    """Ekor per segmen k (dtk efektif) dari lintasan terus & putus satu arah -> (k, catatan).

    Model: jarak = v * (efektif + segmen * k). Dengan jarak terukur di KEDUA rekaman:
        k = (d_p*E_t - d_t*E_p) / (d_t*n_p - d_p*n_t)     (titik awal/akhir boleh beda)
    Tanpa jarak, hanya sah bila keduanya dari tengah ke selotip yang SAMA (d_t = d_p):
        k = (E_t - E_p) / (n_p - n_t)
    """
    if putus is None:
        return 0.0, "tanpa fase putus -> ekor 0, toleransi saja yang menjaga"
    E_t, n_t, d_t = terus["efektif_sec"], terus["segmen"], terus.get("jarak_m")
    E_p, n_p, d_p = putus["efektif_sec"], putus["segmen"], putus.get("jarak_m")
    if d_t and d_p:
        dn = n_p - n_t * d_p / d_t              # selisih segmen, disetarakan jaraknya
        basis = "jarak terukur"
    elif terus.get("asal", "tengah") == "tengah" and putus.get("asal", "tengah") == "tengah":
        d_t = d_p = 1.0                         # selotip yang sama -> jarak sama
        dn = n_p - n_t
        basis = "selotip yang sama"
    else:
        return 0.0, ("isi jarak terukur di rekaman terus DAN putus (atau keduanya dari tengah "
                     "ke selotip yang sama) -> ekor 0")
    if dn < MIN_SELISIH_SEGMEN:
        return 0.0, ("fase putus hanya ~%.1f segmen lebih banyak dari terus (< %d) -> ekor 0; ulangi "
                     "putus dengan lebih banyak start/stop" % (dn, MIN_SELISIH_SEGMEN))
    k = (d_p * E_t - d_t * E_p) / (d_t * n_p - d_p * n_t)
    if k < 0.0:
        return 0.0, ("ekor terhitung NEGATIF (%.3f dtk): jarak/selotip kedua lintasan tak cocok? "
                     "-> dipakai 0" % k)
    return k, "ekor dari terus (%d segmen) vs putus (%d segmen), %s" % (n_t, n_p, basis)


def cocok_kuadrat_terkecil(rs):
    """jarak = a*E + b*n (a = kecepatan, b = kecepatan*ekor) atas SEMUA rekaman berjarak.
    -> dict (a, b, galat baku, RMS, LOO maks positif) atau None bila tak bisa ditentukan."""
    X = [(r["efektif_sec"], float(r["segmen"])) for r in rs]
    y = [r["jarak_m"] for r in rs]
    Sxx = sum(e * e for e, n in X)
    Snn = sum(n * n for e, n in X)
    Sxn = sum(e * n for e, n in X)
    D = Sxx * Snn - Sxn * Sxn
    if len(rs) < 3 or D <= 1e-9 * Sxx * Snn:
        return None

    def solve(Xs, ys):
        sxx = sum(e * e for e, n in Xs)
        snn = sum(n * n for e, n in Xs)
        sxn = sum(e * n for e, n in Xs)
        sxd = sum(e * d for (e, n), d in zip(Xs, ys))
        snd = sum(n * d for (e, n), d in zip(Xs, ys))
        dd = sxx * snn - sxn * sxn
        return (sxd * snn - snd * sxn) / dd, (snd * sxx - sxd * sxn) / dd

    a, b = solve(X, y)
    res = [d - a * e - b * n for (e, n), d in zip(X, y)]
    s2 = sum(r * r for r in res) / max(1, len(rs) - 2)
    loo = []
    for i in range(len(rs)):
        Xi, yi = X[:i] + X[i + 1:], y[:i] + y[i + 1:]
        if len(Xi) >= 2:
            ai, bi = solve(Xi, yi)
            loo.append(y[i] - ai * X[i][0] - bi * X[i][1])
    return {"a": a, "b": b, "se_a": math.sqrt(s2 * Snn / D), "se_b": math.sqrt(s2 * Sxx / D),
            "rms": math.sqrt(sum(r * r for r in res) / len(res)),
            "loo_pos_maks": max([0.0] + loo), "n_rekaman": len(rs)}


# Toleransi saran (operator 2026-11-12: SELURUH kaki harus di dalam area kiper).
#   kaki  : tepi luar telapak dari titik tengah robot = pinggul 0,035 + offset telapak 0,0125
#           + setengah lebar telapak 0,040 (op3_kinematics_dynamics.cpp, robotis_op3.structure.*leg.xacro)
#           + ayunan langkah terakhir ~1 cm
#   galat : 2 sigma (ketidakpastian kecepatan x setengah lebar area  (+)  hanyut ekor sqrt(N) segmen)
SETENGAH_KAKI_M = 0.0875
AYUN_KAKI_M = 0.010
N_SEGMEN_HANYUT = 30            # ~3 mnt main pada ~10 geser/mnt (log demo 2026-11-11)


def hitung(rows, cetak=print, setengah_lebar=SETENGAH_LEBAR_M):
    """Kecepatan & ekor dari SEMUA rekaman berjarak (kuadrat terkecil) bila ada >= 3; kalau tidak,
    cara lama (rekaman terus/putus terbaru). Batas = setengah lebar area / kecepatan."""
    hasil = hitung_pasangan(rows, cetak=lambda *_: None, setengah_lebar=setengah_lebar)
    for arah in ("kanan", "kiri"):
        rs = [r for r in rows if r["arah"] == arah and not r.get("campur") and r.get("jarak_m")]
        f = cocok_kuadrat_terkecil(rs)
        h = hasil.get(arah)
        if f is None or h is None:
            if h is not None:
                cetak("  %-5s: tepi %.2f dtk efektif, ekor %.3f -- %s (kurang dari 3 rekaman berjarak)" % (
                    arah, h["tepi_efektif_sec"], h["ekor_sec"], h["catatan"]))
            else:
                cetak("  %-5s: BELUM ada rekaman 'terus' yang sah" % arah)
            continue
        v = f["a"]
        soft = h["terus"].get("soft", 0.5)
        period = h["terus"].get("period", 0.6)
        k = max(-soft / 2.0, min(period / 2.0, f["b"] / v))
        tepi = setengah_lebar / v
        galat = 2.0 * math.hypot(setengah_lebar * f["se_a"] / v, math.sqrt(N_SEGMEN_HANYUT) * f["se_b"])
        tol = SETENGAH_KAKI_M + AYUN_KAKI_M + galat
        h.update({"tepi_efektif_sec": tepi, "ekor_sec": k, "asal": "semua",
                  "jarak_tepi_m": setengah_lebar, "kecepatan_m_per_dtk": v,
                  "langkah": tepi / (period / 2.0) if period > 0 else None,
                  "cocok": f, "toleransi_saran_m": tol,
                  "catatan": "kuadrat terkecil %d rekaman berjarak: RMS %.1f cm, LOO +%.1f cm" % (
                      f["n_rekaman"], f["rms"] * 100, f["loo_pos_maks"] * 100)})
        cetak("  %-5s: %.2f cm/dtk efektif (+-%.2f), ekor %+.3f dtk/segmen (%+.2f +-%.2f cm), tepi 0,9 m = "
              "%.2f dtk efektif; toleransi saran %.2f m (kaki %.3f + galat 2sigma %.3f)" % (
                  arah, v * 100, f["se_a"] * 100, k, f["b"] * 100, f["se_b"] * 100, tepi, tol,
                  SETENGAH_KAKI_M + AYUN_KAKI_M, galat))
    return hasil


def hitung_pasangan(rows, cetak=print, setengah_lebar=SETENGAH_LEBAR_M):
    """Rekaman terbaru per (arah, mode) yang sah -> hasil per arah.

    asal "tengah": mulai di tengah, berhenti di selotip tepi -> waktu efektif = batas langsung.
    asal "ujung" : mulai di ujung area, geser ke arah seberang, jarak WAJIB diukur ->
                   kecepatan nyata -> batas = setengah lebar area / kecepatan.
    """
    hasil = {}
    for arah in ("kanan", "kiri"):
        terbaru = {}
        for r in rows:
            if r["arah"] != arah or r.get("campur"):
                continue
            if r.get("asal", "tengah") == "ujung" and not r.get("jarak_m"):
                continue                        # dari ujung tanpa jarak tak berarti apa-apa
            terbaru[r["mode"]] = r              # baris belakangan = lebih baru
        terus = terbaru.get("terus")
        putus = terbaru.get("putus")
        if terus is None:
            cetak("  %-5s: BELUM ada rekaman 'terus' yang sah" % arah)
            hasil[arah] = None
            continue
        k, catatan = hitung_ekor(terus, putus)
        efektif_terus = terus["efektif_sec"] + terus["segmen"] * k
        jarak = terus.get("jarak_m")
        asal = terus.get("asal", "tengah")
        if asal == "ujung":
            kec = jarak / efektif_terus
            tepi = setengah_lebar / kec
            jarak_tepi = setengah_lebar
        else:
            kec = jarak / efektif_terus if (jarak and efektif_terus > 0) else None
            tepi = efektif_terus
            jarak_tepi = jarak
        langkah = tepi / (terus["period"] / 2.0) if terus.get("period", 0) > 0 else None
        hasil[arah] = {"tepi_efektif_sec": tepi, "ekor_sec": k, "langkah": langkah, "asal": asal,
                       "jarak_tepi_m": jarak_tepi, "kecepatan_m_per_dtk": kec,
                       "catatan": catatan, "terus": terus, "putus": putus}
        cetak("  %-5s: tepi %.2f dtk efektif (~%s langkah), ekor %.3f dtk/segmen, dari %s, jarak %s -- %s" % (
            arah, tepi, "%.1f" % langkah if langkah else "?", k, asal,
            "%.1f cm (%.2f cm/dtk)" % (jarak * 100, kec * 100) if kec else "belum diukur", catatan))
    return hasil


# == berkas ==================================================================

def baca_sampel(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def ikuti_pemilik_folder(path):
    """Skrip berjalan sebagai root di container; kembalikan pemilik ke pemilik folder."""
    try:
        st = os.stat(os.path.dirname(path))
        os.chown(path, st.st_uid, st.st_gid)
    except OSError:
        pass


def tulis_teks(path, teks):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(teks)
    os.replace(tmp, path)
    ikuti_pemilik_folder(path)


def tulis_sampel(path, rows):
    tulis_teks(path, "".join(json.dumps(r, ensure_ascii=False) + "\n" for r in rows))


def tulis_yaml(path, hasil):
    sah = all(hasil.get(a) is not None for a in ("kanan", "kiri"))
    L = ["# Kalibrasi batas geser kiper D58 -- dibuat scripts/calib_batas_geser.py",
         "# JANGAN diedit tangan; ulangi `rekam` lalu `hitung`.",
         "# Batas demo: waktu efektif (integral skala soft start) tengah->tepi + ekor per segmen.",
         "valid: %s" % ("true" if sah else "false"),
         "tanggal: \"%s\"" % datetime.now().isoformat(timespec="seconds"),
         "setengah_lebar_area_m: %.3f" % SETENGAH_LEBAR_M]
    for arah in ("kanan", "kiri"):
        h = hasil.get(arah)
        if h is None:
            L.append("%s: null" % arah)
            continue
        t = h["terus"]
        p = h["putus"]
        L.append("%s:" % arah)
        L.append("  tepi_efektif_sec: %.4f" % h["tepi_efektif_sec"])
        L.append("  ekor_sec: %.4f" % h["ekor_sec"])
        L.append("  langkah: %s" % ("%.2f" % h["langkah"] if h["langkah"] else "null"))
        L.append("  asal: %s   # semua = kuadrat terkecil semua rekaman berjarak; tengah/ujung = rekaman terbaru"
                 % h["asal"])
        L.append("  jarak_tepi_m: %s   # null = pakai setengah_lebar_area_m" % (
            "%.4f" % h["jarak_tepi_m"] if h["jarak_tepi_m"] else "null"))
        L.append("  kecepatan_cm_per_dtk: %s" % (
            "%.3f" % (h["kecepatan_m_per_dtk"] * 100) if h["kecepatan_m_per_dtk"] else "null"))
        f = h.get("cocok")
        if f:
            L.append("  toleransi_saran_m: %.3f   # kaki %.4f + ayun %.3f + galat 2sigma (v & hanyut %d segmen)" % (
                h["toleransi_saran_m"], SETENGAH_KAKI_M, AYUN_KAKI_M, N_SEGMEN_HANYUT))
            L.append("  cocok: {rekaman: %d, galat_baku_kecepatan_cm_per_dtk: %.3f, galat_baku_ekor_cm: %.3f, "
                     "rms_cm: %.2f, loo_positif_maks_cm: %.2f}" % (
                         f["n_rekaman"], f["se_a"] * 100, f["se_b"] * 100, f["rms"] * 100,
                         f["loo_pos_maks"] * 100))
        L.append("  catatan: \"%s\"" % h["catatan"].replace('"', "'"))
        L.append("  y_move: %.4f" % t["y_move"])
        L.append("  period_sec: %.4f" % t["period"])
        L.append("  soft_start_sec: %.3f" % t["soft"])
        for nama, r in (("terus", t), ("putus", p)):
            if r is None:
                L.append("  %s: null" % nama)
            else:
                L.append("  %s: {asal: %s, efektif_sec: %.4f, mentah_sec: %.3f, segmen: %d, jarak_m: %s, "
                         "waktu: \"%s\"}" % (
                             nama, r.get("asal", "tengah"), r["efektif_sec"], r["mentah_sec"], r["segmen"],
                             "%.4f" % r["jarak_m"] if r.get("jarak_m") else "null", r["waktu"]))
    tulis_teks(path, "\n".join(L) + "\n")
    return sah


def versi_terbaru(arah):
    try:
        with open(VERSIONS_FILE) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    c = [v for v in d if v.get("name", "").strip().lower() == NAMA_VERSI[arah]]
    if not c:
        return None
    return max(c, key=lambda v: v.get("updatedAt", 0)).get("params", {})


# == rekam (robot) ===========================================================

def pasang_pendengar_walking(node, peristiwa, String, WalkingParam, ParameterEvent):
    """Dengarkan perintah, parameter, dan soft_start_time walking -> antrean (t, jenis, data)."""
    node.create_subscription(String, "/robotis/walking/command",
                             lambda m: peristiwa.append((time.monotonic(), "cmd", m.data)), 10)
    node.create_subscription(WalkingParam, "/robotis/walking/set_params",
                             lambda m: peristiwa.append((time.monotonic(), "param",
                                                         (m.y_move_amplitude, m.period_time))), 10)

    def on_param_event(ev):
        if ev.node.rstrip("/").split("/")[-1] != "op3_walking_module":
            return
        for p in list(ev.changed_parameters) + list(ev.new_parameters):
            if p.name == "soft_start_time":
                peristiwa.append((time.monotonic(), "soft", p.value.double_value))
    node.create_subscription(ParameterEvent, "/parameter_events", on_param_event, 50)


def baca_keadaan_walking(node, rclpy, GetWalkingParam, GetParameters):
    """(y_move, period, soft_start) sekarang dari op3_manager; None bila tak menjawab."""
    def panggil(cli, req, batas=3.0):
        if not cli.wait_for_service(timeout_sec=batas):
            return None
        fut = cli.call_async(req)
        t_end = time.monotonic() + batas
        while time.monotonic() < t_end and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    r = panggil(node.create_client(GetWalkingParam, "/robotis/walking/get_params"),
                GetWalkingParam.Request(get_param=True))
    if r is None:
        return None
    rs = panggil(node.create_client(GetParameters, "/op3_walking_module/get_parameters"),
                 GetParameters.Request(names=["soft_start_time"]))
    soft = rs.values[0].double_value if rs is not None and rs.values else 0.0
    return r.parameters.y_move_amplitude, r.parameters.period_time, soft

def cmd_rekam(args):
    import select
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from op3_walking_module_msgs.msg import WalkingParam
    from op3_walking_module_msgs.srv import GetWalkingParam
    from rcl_interfaces.msg import ParameterEvent
    from rcl_interfaces.srv import GetParameters

    arah = args.arah
    rclpy.init()
    node = Node("kalibrasi_batas_geser")
    peristiwa = []                                  # (t, jenis, data) -- diolah di loop utama
    pasang_pendengar_walking(node, peristiwa, String, WalkingParam, ParameterEvent)

    def selesai(kode, pesan=None):
        node.destroy_node()
        rclpy.shutdown()
        if pesan:
            print(pesan)
        sys.exit(kode)

    def panggil(cli, req, batas=3.0):
        if not cli.wait_for_service(timeout_sec=batas):
            return None
        fut = cli.call_async(req)
        t_end = time.monotonic() + batas
        while time.monotonic() < t_end and not fut.done():
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    # -- prasyarat -----------------------------------------------------------
    print("Membaca parameter walking dari op3_manager ...", flush=True)
    r = panggil(node.create_client(GetWalkingParam, "/robotis/walking/get_params"),
                GetWalkingParam.Request(get_param=True))
    if r is None:
        selesai(2, "GAGAL: /robotis/walking/get_params tak menjawab -- op3_manager belum jalan "
                   "(atau ROS_DOMAIN_ID bukan 1).")
    rq = GetParameters.Request(names=["soft_start_time"])
    rs = panggil(node.create_client(GetParameters, "/op3_walking_module/get_parameters"), rq)
    soft = rs.values[0].double_value if rs is not None and rs.values else 0.0
    if rs is None:
        print("PERINGATAN: soft_start_time tak terbaca -- dianggap 0 sampai Studio mengubahnya.")
    p = Penghitung(soft=soft, y=r.parameters.y_move_amplitude, period=r.parameters.period_time)

    nama = set(node.get_node_names())
    if "demo_node" in nama and not args.paksa:
        selesai(2, "GAGAL: demo_node hidup -- matikan demo (atau pastikan D58 OFF lalu --paksa). "
                   "Perintah walking dari demo akan ikut terekam.")

    acuan = versi_terbaru(arah)
    if acuan is None:
        print("PERINGATAN: versi \"%s\" tak ditemukan di %s" % (NAMA_VERSI[arah], VERSIONS_FILE))

    print()
    print("== KALIBRASI GESER %s, mode %s ==" % (arah.upper(), args.mode.upper()))
    print("  soft_start_time sekarang %.2f dtk, y_move %.4f, period %.3f" % (p.soft, p.y, p.period))
    if args.mode == "terus":
        print("  Jalankan \"%s\" di Studio SEKALI jalan dari tengah sampai selotip tepi." % NAMA_VERSI[arah])
    else:
        print("  Jalankan \"%s\" di Studio dengan BANYAK start/stop pendek (~1 dtk) dari tengah "
              "sampai selotip tepi yang SAMA." % NAMA_VERSI[arah])
    diam = args.diam if args.diam is not None else (4.0 if args.mode == "terus" else 10.0)
    if args.otomatis:
        # Tanpa Enter (operator 2026-11-12): dijalankan di latar belakang sementara operator
        # menggerakkan robot lewat Studio. Selesai sendiri bila sesudah >= 1 segmen geser
        # tidak ada "start" baru selama `diam` detik.
        print("  OTOMATIS: merekam MULAI SEKARANG (robot harus sudah di TENGAH). Selesai sendiri "
              "%.0f dtk sesudah geser terakhir berhenti; batas %.0f dtk." % (diam, args.batas_waktu),
              flush=True)
    else:
        try:
            input("  Robot sudah di TENGAH dan diam? Tekan Enter untuk mulai merekam ... ")
        except EOFError:
            selesai(2, "stdin tertutup -- jalankan dengan `docker exec -it` (atau --otomatis).")
        print("  Merekam. Tekan Enter sesudah robot BERHENTI di tepi.", flush=True)

    tanda = ARAH_TANDA[arah]
    t_mulai = time.monotonic()
    t_stop_terakhir = [None]

    def olah(t, jenis, data):
        if jenis == "param":
            y, per = data
            p.params(t, y, per)
        elif jenis == "soft":
            p.set_soft(t, data)
            print("    soft_start_time -> %.2f dtk" % data, flush=True)
        elif jenis == "cmd":
            if data == "start":
                if p.start(t):
                    if abs(p.y) < Y_MIN:
                        print("    start (y_move 0 -- bukan geser, tak dihitung)", flush=True)
                    elif (p.y > 0) != (tanda > 0):
                        print("    PERINGATAN: start ke arah BERLAWANAN (y %.4f) -- fase ini akan "
                              "ditandai tercampur" % p.y, flush=True)
                    elif acuan and (abs(p.y - acuan.get("y_move_amplitude", p.y)) > 1e-4 or
                                    abs(p.period - acuan.get("period_time", p.period)) > 1e-4):
                        print("    PERINGATAN: y %.4f / period %.3f BEDA dari versi \"%s\" terbaru "
                              "(y %.4f / period %.3f)" % (p.y, p.period, NAMA_VERSI[arah],
                                                          acuan.get("y_move_amplitude", 0.0),
                                                          acuan.get("period_time", 0.0)), flush=True)
            elif data in ("stop", "soft_stop"):
                s = p.stop(t, lunak=(data == "soft_stop"))
                if s is not None:
                    t_stop_terakhir[0] = t
                    e, m, n, _ = p.ringkas(arah)
                    print("    segmen %d (%s): %.2f dtk mentah, %.2f dtk efektif  | total %s: %.2f dtk "
                          "efektif dalam %d segmen" % (len(p.segmen), data, s["mentah"][arah],
                                                        s["efektif"][arah], arah, e, n), flush=True)

    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        while peristiwa:
            olah(*peristiwa.pop(0))
        if args.otomatis:
            now = time.monotonic()
            if (not p.jalan and t_stop_terakhir[0] is not None and p.ringkas(arah)[2] >= 1 and
                    now - t_stop_terakhir[0] >= diam):
                print("  Diam %.0f dtk sesudah geser terakhir -- rekaman selesai." % diam, flush=True)
                break
            if now - t_mulai >= args.batas_waktu:
                print("  Batas waktu %.0f dtk tercapai -- rekaman diakhiri." % args.batas_waktu, flush=True)
                break
        elif select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break

    t_akhir = time.monotonic()
    if p.jalan:
        print("  PERINGATAN: robot masih geser saat Enter -- segmen terakhir dipotong di sini.")
        p.stop(t_akhir)
    e, m, n, e_lain = p.ringkas(arah)
    campur = e_lain > 0.2
    if n == 0:
        selesai(1, "Tidak ada segmen geser %s yang terekam -- tidak disimpan." % arah)
    if args.asal == "ujung" and not args.jarak_cm:
        selesai(1, "Dari ujung: --jarak-cm wajib (jarak yang ditempuh, diukur) -- tidak disimpan.")
    row = {"arah": arah, "mode": args.mode, "asal": args.asal, "efektif_sec": e, "mentah_sec": m, "segmen": n,
           "efektif_arah_lain_sec": e_lain, "campur": campur, "soft": p.segmen[0]["soft"],
           "y_move": p.segmen[0]["y"], "period": p.segmen[0]["period"],
           "jarak_m": args.jarak_cm / 100.0 if args.jarak_cm else None,
           "waktu": datetime.now().isoformat(timespec="seconds"),
           "rincian": [{"akhir": s["akhir"], "efektif": s["efektif"], "mentah": s["mentah"],
                        "soft": s["soft"], "y": s["y"], "durasi": s["t1"] - s["t0"]} for s in p.segmen]}
    print()
    print("  HASIL %s %s: %.2f dtk efektif (%.2f mentah) dalam %d segmen%s" % (
        arah, args.mode, e, m, n,
        "  -- TERCAMPUR arah lain %.2f dtk, tidak dipakai `hitung`" % e_lain if campur else ""))
    if args.tidak_simpan:
        selesai(0, "(--tidak-simpan: data tidak ditulis)")
    rows = baca_sampel(args.file)
    rows.append(row)
    tulis_sampel(args.file, rows)
    print("  Disimpan ke %s" % args.file)
    print()
    hasil = hitung(rows)
    sah = tulis_yaml(args.hasil, hasil)
    print("  YAML: %s (%s)" % (args.hasil, "valid" if sah else "belum valid -- perlu kanan DAN kiri 'terus'"))
    selesai(0)


TOPIK_STATUS = "/kalibrasi_batas_geser/status"      # JSON -> Studio (panel Kalibrasi Batas Geser)
TOPIK_PERINTAH = "/kalibrasi_batas_geser/perintah"  # JSON <- Studio


def cmd_layan(args):
    """Server untuk panel Studio (operator 2026-11-12): mendengar walking terus-menerus,
    fase dimulai/disimpan dari Studio, status (termasuk perkiraan cm) dikirim tiap 0,5 dtk."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from op3_walking_module_msgs.msg import WalkingParam
    from op3_walking_module_msgs.srv import GetWalkingParam
    from rcl_interfaces.msg import ParameterEvent
    from rcl_interfaces.srv import GetParameters

    rclpy.init()
    node = Node("kalibrasi_batas_geser_layan")
    peristiwa = []
    pasang_pendengar_walking(node, peristiwa, String, WalkingParam, ParameterEvent)
    pub = node.create_publisher(String, TOPIK_STATUS, 10)
    node.create_subscription(String, TOPIK_PERINTAH,
                             lambda m: peristiwa.append((time.monotonic(), "perintah", m.data)), 10)

    awal = None
    while awal is None:
        awal = baca_keadaan_walking(node, rclpy, GetWalkingParam, GetParameters)
        if awal is None:
            print("Menunggu op3_manager (/robotis/walking/get_params) ...", flush=True)
    p = Penghitung(soft=awal[2], y=awal[0], period=awal[1])
    st = {"fase": None, "pesan": "Perekam siap. Pilih arah & mode lalu Mulai Rekam.", "galat": False}
    rows = baca_sampel(args.file)
    hasil = hitung(rows, cetak=lambda *_: None)
    print("Perekam batas geser siap: status di %s, perintah di %s" % (TOPIK_STATUS, TOPIK_PERINTAH),
          flush=True)

    def pesan(teks, galat=False):
        st["pesan"], st["galat"] = teks, galat
        print(("GALAT: " if galat else "") + teks, flush=True)

    def kec(arah):
        h = hasil.get(arah)
        return (h["kecepatan_m_per_dtk"], h["ekor_sec"]) if h and h.get("kecepatan_m_per_dtk") else (None, 0.0)

    def cm(arah, efektif, selesai):
        v, k = kec(arah)
        if v is None or arah is None:
            return None
        return (efektif + (k if selesai else 0.0)) * v * 100.0

    def segmen_json(i, s):
        a = "kiri" if s["efektif"]["kiri"] > s["efektif"]["kanan"] else "kanan"
        return {"no": i + 1, "arah": a, "efektif": s["efektif"][a], "mentah": s["mentah"][a],
                "akhir": s["akhir"], "cm": cm(a, s["efektif"][a], True)}

    def simpan_ulang():
        nonlocal hasil
        tulis_sampel(args.file, rows)
        hasil = hitung(rows)
        return tulis_yaml(args.hasil, hasil)

    def muat_ulang():
        # Berkas bisa ditulis proses lain (mis. `rekam` di terminal) -- selalu baca ulang
        # sebelum mengubah, supaya rekamannya tidak tertimpa.
        nonlocal hasil
        rows[:] = baca_sampel(args.file)
        hasil = hitung(rows, cetak=lambda *_: None)

    def perintah(data):
        try:
            d = json.loads(data)
        except ValueError:
            return pesan("Perintah bukan JSON: %r" % data, True)
        aksi = d.get("aksi")
        muat_ulang()
        if aksi == "mulai":
            if d.get("arah") not in ARAH_TANDA or d.get("mode") not in ("terus", "putus"):
                return pesan("Mulai: arah/mode tidak sah", True)
            if p.jalan:
                return pesan("Robot masih geser -- Stop dulu, letakkan di titik awal, lalu Mulai Rekam", True)
            asal = d.get("asal", "tengah")
            if asal not in ("tengah", "ujung"):
                return pesan("Mulai: asal harus tengah/ujung", True)
            st["fase"] = {"arah": d["arah"], "mode": d["mode"], "asal": asal, "dari": len(p.segmen),
                          "mulai": datetime.now().isoformat(timespec="seconds")}
            if asal == "tengah":
                pesan("Merekam %s %s dari TENGAH. Jalankan \"%s\" sampai selotip tepi, Stop, ukur jarak, Simpan."
                      % (d["arah"], d["mode"], NAMA_VERSI[d["arah"]]))
            else:
                pesan("Merekam %s %s dari UJUNG %s. Jalankan \"%s\" ke arah ujung seberang, Stop, ukur jarak "
                      "yang ditempuh (WAJIB), Simpan." % (d["arah"], d["mode"],
                                                         "KIRI" if d["arah"] == "kanan" else "KANAN",
                                                         NAMA_VERSI[d["arah"]]))
        elif aksi == "batal":
            st["fase"] = None
            pesan("Rekaman dibatalkan, tidak disimpan.")
        elif aksi == "simpan":
            f = st["fase"]
            if f is None:
                return pesan("Belum ada rekaman berjalan", True)
            if p.jalan:
                return pesan("Robot masih geser -- Stop dulu sebelum Simpan", True)
            e, m, n, e_lain = p.ringkas(f["arah"], f["dari"])
            if n == 0:
                return pesan("Belum ada segmen geser %s di rekaman ini" % f["arah"], True)
            segs = p.segmen[f["dari"]:]
            jarak_cm = d.get("jarak_cm")
            if f["asal"] == "ujung" and not jarak_cm:
                return pesan("Dari ujung: isi jarak terukur (cm) yang ditempuh dulu", True)
            row = {"arah": f["arah"], "mode": f["mode"], "asal": f["asal"], "efektif_sec": e,
                   "mentah_sec": m, "segmen": n,
                   "efektif_arah_lain_sec": e_lain, "campur": e_lain > 0.2, "soft": segs[0]["soft"],
                   "y_move": segs[0]["y"], "period": segs[0]["period"],
                   "jarak_m": float(jarak_cm) / 100.0 if jarak_cm else None,
                   "waktu": datetime.now().isoformat(timespec="seconds"),
                   "rincian": [{"akhir": s["akhir"], "efektif": s["efektif"], "mentah": s["mentah"],
                                "soft": s["soft"], "y": s["y"], "durasi": s["t1"] - s["t0"]} for s in segs]}
            rows.append(row)
            sah = simpan_ulang()
            st["fase"] = None
            pesan("Tersimpan: %s %s %.2f dtk efektif, %d segmen%s%s. YAML %s." % (
                row["arah"], row["mode"], e, n,
                ", jarak %.1f cm" % jarak_cm if jarak_cm else "",
                " -- TERCAMPUR arah lain, tak dipakai" if row["campur"] else "",
                "VALID (restart demo untuk memakai)" if sah else "belum valid (perlu kanan & kiri 'terus')"))
        elif aksi == "jarak":
            # Isi/ubah jarak terukur rekaman 'terus' terbaru suatu arah (mis. lupa saat Simpan).
            arah = d.get("arah")
            cand = [r for r in rows if r["arah"] == arah and r["mode"] == "terus" and not r.get("campur")]
            if not cand or not d.get("jarak_cm"):
                return pesan("Jarak: tak ada rekaman 'terus' %s / jarak kosong" % arah, True)
            cand[-1]["jarak_m"] = float(d["jarak_cm"]) / 100.0
            sah = simpan_ulang()
            pesan("Jarak %s diset %.1f cm. YAML %s." % (arah, float(d["jarak_cm"]),
                                                       "VALID" if sah else "belum valid"))
        elif aksi == "hapus":
            i = d.get("nomor")
            if not isinstance(i, int) or not 0 <= i < len(rows):
                return pesan("Hapus: nomor tidak ada", True)
            r = rows.pop(i)
            simpan_ulang()
            pesan("Dihapus #%d (%s %s)" % (i, r["arah"], r["mode"]))
        else:
            pesan("Aksi tidak dikenal: %r" % aksi, True)

    def kirim_status():
        t = time.monotonic()
        e_jalan, a_jalan = p.efektif_berjalan(t)
        f = st["fase"]
        fase = None
        if f is not None:
            segs = p.segmen[f["dari"]:]
            e, m, n, e_lain = p.ringkas(f["arah"], f["dari"])
            tambah = e_jalan if a_jalan == f["arah"] else 0.0
            v, k = kec(f["arah"])
            fase = {"arah": f["arah"], "mode": f["mode"], "asal": f["asal"], "mulai": f["mulai"],
                    "segmen": [segmen_json(f["dari"] + i, s) for i, s in enumerate(segs)],
                    "total_efektif": e + tambah, "total_segmen": n, "efektif_arah_lain": e_lain,
                    "perkiraan_cm": ((e + tambah + n * k) * v * 100.0) if v else None}
        kal = {"valid": all(hasil.get(a) for a in ("kanan", "kiri"))}
        for a in ("kanan", "kiri"):
            h = hasil.get(a)
            kal[a] = None if h is None else {
                "tepi_efektif_sec": h["tepi_efektif_sec"], "ekor_sec": h["ekor_sec"], "asal": h["asal"],
                "toleransi_saran_m": h.get("toleransi_saran_m"),
                "rekaman_dicocokkan": h["cocok"]["n_rekaman"] if h.get("cocok") else None,
                "jarak_tepi_m": h["jarak_tepi_m"],
                "cm_per_dtk": h["kecepatan_m_per_dtk"] * 100 if h["kecepatan_m_per_dtk"] else None,
                "punya_putus": h["putus"] is not None}
        terakhir = segmen_json(len(p.segmen) - 1, p.segmen[-1]) if p.segmen else None
        msg = {"waktu": time.time(), "soft": p.soft, "y": p.y, "period": p.period, "jalan": p.jalan,
               "berjalan": {"arah": a_jalan, "efektif": e_jalan, "cm": cm(a_jalan, e_jalan, False)}
               if p.jalan else None,
               "segmen_terakhir": terakhir, "fase": fase, "kalibrasi": kal,
               "rekaman": [{"nomor": i, "arah": r["arah"], "mode": r["mode"], "asal": r.get("asal", "tengah"),
                            "efektif_sec": r["efektif_sec"],
                            "segmen": r["segmen"], "jarak_m": r.get("jarak_m"), "campur": bool(r.get("campur")),
                            "waktu": r["waktu"]} for i, r in enumerate(rows)],
               "pesan": st["pesan"], "galat": st["galat"]}
        pub.publish(String(data=json.dumps(msg)))

    t_kirim = 0.0
    t_muat = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - t_muat >= 2.0:
                muat_ulang()
                t_muat = time.monotonic()
            ada = bool(peristiwa)
            while peristiwa:
                t, jenis, data = peristiwa.pop(0)
                if jenis == "param":
                    p.params(t, *data)
                elif jenis == "soft":
                    p.set_soft(t, data)
                elif jenis == "cmd":
                    if data == "start":
                        p.start(t)
                    elif data in ("stop", "soft_stop"):
                        s = p.stop(t, lunak=(data == "soft_stop"))
                        if s is not None:
                            j = segmen_json(len(p.segmen) - 1, s)
                            print("segmen %d %s: %.2f dtk efektif%s" % (
                                j["no"], j["arah"], j["efektif"],
                                ", ~%.1f cm" % j["cm"] if j["cm"] is not None else ""), flush=True)
                elif jenis == "perintah":
                    perintah(data)
            if ada or time.monotonic() - t_kirim >= 0.5:
                kirim_status()
                t_kirim = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cmd_hitung(args):
    rows = baca_sampel(args.file)
    if not rows:
        sys.exit("Belum ada rekaman di %s -- jalankan `rekam` dulu." % args.file)
    hasil = hitung(rows)
    if args.tidak_simpan:
        print("(--tidak-simpan: hasil tidak ditulis)")
        return
    sah = tulis_yaml(args.hasil, hasil)
    print("YAML ditulis ke %s (%s)" % (args.hasil, "valid" if sah else "belum valid"))


def cmd_status(args):
    rows = baca_sampel(args.file)
    if not rows:
        print("Belum ada rekaman (%s)." % args.file)
        return
    print("%3s %-5s %-5s %9s %9s %7s %7s  %s" % ("#", "arah", "mode", "efektif", "mentah", "segmen",
                                                 "campur", "waktu"))
    for i, r in enumerate(rows):
        print("%3d %-5s %-5s %8.2fs %8.2fs %7d %7s  %s" % (i, r["arah"], r["mode"], r["efektif_sec"],
                                                          r["mentah_sec"], r["segmen"],
                                                          "YA" if r.get("campur") else "-", r["waktu"]))
    print()
    hitung(rows)


def cmd_hapus(args):
    rows = baca_sampel(args.file)
    if not 0 <= args.nomor < len(rows):
        sys.exit("Nomor %d tidak ada (lihat `status`)." % args.nomor)
    r = rows.pop(args.nomor)
    tulis_sampel(args.file, rows)
    print("Dihapus: #%d %s %s %s" % (args.nomor, r["arah"], r["mode"], r["waktu"]))
    hasil = hitung(rows)
    tulis_yaml(args.hasil, hasil)


def cmd_uji_sintetis(args):
    """Robot rekaan: jarak = v * (waktu efektif + ekor k per segmen). Dua lintasan ke titik
    tepi yang sama (terus vs putus) harus menghasilkan k yang benar dan batas yang sama."""
    gagal = 0

    def cek(nama, ok, rincian):
        nonlocal gagal
        print("  [%s] %s -- %s" % ("OK " if ok else "GAL", nama, rincian))
        gagal += 0 if ok else 1

    # 1. integral skala: tanpa soft start, ramp penuh, potongan sebagian
    cek("integral tanpa soft", abs(integral_skala(0, 3, 0) - 3) < 1e-9, "%.4f" % integral_skala(0, 3, 0))
    cek("integral ramp 0,5 di 3 dtk", abs(integral_skala(0, 3, 0.5) - 2.75) < 1e-9,
        "%.4f (harus 2,75)" % integral_skala(0, 3, 0.5))
    cek("integral dipotong = utuh", abs(integral_skala(0, 0.2, 0.5) + integral_skala(0.2, 3, 0.5) - 2.75) < 1e-9,
        "jumlah potongan sama dengan utuh")

    soft, k_benar, v = 0.5, 0.12, 0.05          # dtk, dtk efektif/segmen, m per dtk efektif
    jarak_tepi = 0.80

    def lintasan(durasi_segmen):
        """Simulasikan robot sampai jarak_tepi; kembalikan rekaman ala `rekam`."""
        p = Penghitung(soft=soft, y=-0.02, period=0.595)
        t, jarak = 0.0, 0.0
        for d in durasi_segmen:
            p.start(t)
            p.stop(t + d)
            jarak += v * (integral_skala(0, d, soft) + k_benar)
            t += d + 1.0
            if jarak >= jarak_tepi:
                break
        e, m, n, _ = p.ringkas("kanan")
        return {"arah": "kanan", "efektif_sec": e, "mentah_sec": m, "segmen": n, "period": 0.595,
                "soft": soft, "y_move": -0.02, "waktu": "sintetis"}, jarak

    # Durasi segmen terakhir dipilih supaya robot TEPAT di tepi (operator berhenti di selotip).
    def sampai_tepi(potong):
        sisa = jarak_tepi
        durasi = []
        while True:
            maju_penuh = v * (integral_skala(0, potong, soft) + k_benar)
            if maju_penuh >= sisa:
                # cari d dengan v*(integral(0,d)+k) = sisa
                lo, hi = 0.0, potong
                for _ in range(60):
                    mid = (lo + hi) / 2
                    if v * (integral_skala(0, mid, soft) + k_benar) < sisa:
                        lo = mid
                    else:
                        hi = mid
                durasi.append(hi)
                return durasi
            durasi.append(potong)
            sisa -= maju_penuh

    terus, j1 = lintasan(sampai_tepi(60.0))
    terus["mode"] = "terus"
    putus, j2 = lintasan(sampai_tepi(1.0))
    putus["mode"] = "putus"
    cek("kedua lintasan sampai tepi", abs(j1 - jarak_tepi) < 1e-6 and abs(j2 - jarak_tepi) < 1e-6,
        "%.4f / %.4f m" % (j1, j2))
    h = hitung([terus, putus], cetak=lambda *_: None)["kanan"]
    cek("ekor terbaca kembali", abs(h["ekor_sec"] - k_benar) < 1e-6, "%.4f (benar %.4f)" % (h["ekor_sec"], k_benar))

    # Demo memakai batas yang sama dengan segmen 0,7 dtk: berhenti tepat di tepi?
    tepi = h["tepi_efektif_sec"]
    pos, n = 0.0, 0
    while pos < tepi - 1e-9:
        e = integral_skala(0, 0.7, soft)
        if pos + e + k_benar > tepi:
            break
        pos += e + k_benar
        n += 1
    jarak_demo = v * pos
    cek("demo segmen 0,7 dtk tak lewat tepi", jarak_demo <= jarak_tepi + 1e-9,
        "%.3f m dari %.3f m (%d segmen)" % (jarak_demo, jarak_tepi, n))

    # Tanpa fase putus: ekor 0 -> demo terputus-putus MELEBIHI perkiraan (alasan toleransi).
    h0 = hitung([terus], cetak=lambda *_: None)["kanan"]
    pos, n = 0.0, 0
    while pos + integral_skala(0, 0.7, soft) <= h0["tepi_efektif_sec"]:
        pos += integral_skala(0, 0.7, soft)
        n += 1
    nyata = v * (pos + n * k_benar)
    print("  info: tanpa fase putus, demo %d segmen 0,7 dtk mengira %.3f m tapi nyata %.3f m "
          "(kelebihan %.0f cm -> ditutup toleransi)" % (n, v * pos, nyata, (nyata - v * pos) * 100))

    # Dari UJUNG (2026-11-12): jarak terukur di kedua rekaman, panjang lintasan BEDA.
    def jalur(durasi_segmen):
        p = Penghitung(soft=soft, y=-0.02, period=0.595)
        t, jarak = 0.0, 0.0
        for d in durasi_segmen:
            p.start(t)
            p.stop(t + d)
            jarak += v * (integral_skala(0, d, soft) + k_benar)
            t += d + 1.0
        e, m, n, _ = p.ringkas("kanan")
        return {"arah": "kanan", "asal": "ujung", "efektif_sec": e, "mentah_sec": m, "segmen": n,
                "period": 0.595, "soft": soft, "y_move": -0.02, "waktu": "sintetis", "jarak_m": jarak}
    tu = jalur([30.0])
    tu["mode"] = "terus"
    pu = jalur([1.0] * 25)
    pu["mode"] = "putus"
    hu = hitung([tu, pu], cetak=lambda *_: None)["kanan"]
    cek("ujung: ekor terbaca (lintasan %.2f m vs %.2f m)" % (tu["jarak_m"], pu["jarak_m"]),
        abs(hu["ekor_sec"] - k_benar) < 1e-6, "%.4f (benar %.4f)" % (hu["ekor_sec"], k_benar))
    cek("ujung: kecepatan nyata", abs(hu["kecepatan_m_per_dtk"] - v) < 1e-9,
        "%.4f m/dtk (benar %.4f)" % (hu["kecepatan_m_per_dtk"], v))
    cek("ujung: batas = setengah lebar / kecepatan",
        abs(hu["tepi_efektif_sec"] - SETENGAH_LEBAR_M / v) < 1e-6 and hu["jarak_tepi_m"] == SETENGAH_LEBAR_M,
        "%.2f dtk efektif = %.2f m" % (hu["tepi_efektif_sec"], hu["jarak_tepi_m"]))
    # Kuadrat terkecil atas 4 rekaman (2026-11-12): tengah & ujung, terus & putus.
    rekaman = []
    for asal, durasi in (("tengah", [17.0]), ("tengah", [1.6] * 9), ("ujung", [30.0]), ("ujung", [1.8] * 16)):
        r = jalur(durasi)
        r["asal"] = asal
        r["mode"] = "terus" if len(durasi) == 1 else "putus"
        rekaman.append(r)
    hl = hitung(rekaman, cetak=lambda *_: None)["kanan"]
    cek("kuadrat terkecil: kecepatan & ekor", abs(hl["kecepatan_m_per_dtk"] - v) < 1e-9 and
        abs(hl["ekor_sec"] - k_benar) < 1e-9, "%.4f m/dtk, ekor %.4f dtk" % (hl["kecepatan_m_per_dtk"], hl["ekor_sec"]))
    cek("kuadrat terkecil: toleransi saran = kaki saja (data tanpa derau)",
        abs(hl["toleransi_saran_m"] - (SETENGAH_KAKI_M + AYUN_KAKI_M)) < 1e-6, "%.4f m" % hl["toleransi_saran_m"])

    tanpa_jarak = dict(tu, jarak_m=None)
    cek("ujung tanpa jarak diabaikan", hitung([tanpa_jarak], cetak=lambda *_: None)["kanan"] is None,
        "rekaman dari ujung wajib berjarak")

    print("SEMUA OK" if gagal == 0 else "%d GAGAL" % gagal)
    sys.exit(1 if gagal else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--file", default=SAMPLE_FILE, help="berkas rekaman (jsonl)")
    ap.add_argument("--hasil", default=RESULT_FILE, help="berkas YAML hasil (dibaca demo)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("rekam", help="rekam satu lintasan tengah->tepi yang dijalankan lewat Studio")
    r.add_argument("--arah", choices=("kanan", "kiri"), required=True)
    r.add_argument("--mode", choices=("terus", "putus"), required=True,
                   help="terus = sekali jalan; putus = banyak start/stop pendek ke tepi yang sama")
    r.add_argument("--asal", choices=("tengah", "ujung"), default="tengah",
                   help="tengah = tengah->selotip tepi; ujung = dari ujung area ke arah seberang (--jarak-cm wajib)")
    r.add_argument("--jarak-cm", type=float, default=None, help="jarak yang ditempuh, diukur (cm)")
    r.add_argument("--paksa", action="store_true", help="jalan walau demo_node hidup (D58 harus OFF)")
    r.add_argument("--tidak-simpan", action="store_true", help="uji saja, data tidak ditulis")
    r.add_argument("--otomatis", action="store_true",
                   help="tanpa Enter: mulai seketika, selesai sendiri sesudah geser berhenti --diam dtk")
    r.add_argument("--diam", type=float, default=None,
                   help="dtk tanpa start baru sesudah geser terakhir = selesai (bawaan: terus 4, putus 10)")
    r.add_argument("--batas-waktu", type=float, default=600.0, help="dtk maksimum satu rekaman otomatis")
    r.set_defaults(func=cmd_rekam)

    ly = sub.add_parser("layan", help="server untuk panel Kalibrasi Batas Geser di BASCORRO Studio")
    ly.set_defaults(func=cmd_layan)

    h = sub.add_parser("hitung", help="hitung ulang YAML dari semua rekaman")
    h.add_argument("--tidak-simpan", action="store_true")
    h.set_defaults(func=cmd_hitung)

    st = sub.add_parser("status", help="daftar rekaman + hasil")
    st.set_defaults(func=cmd_status)

    hp = sub.add_parser("hapus", help="buang satu rekaman (nomor dari `status`)")
    hp.add_argument("--nomor", type=int, required=True)
    hp.set_defaults(func=cmd_hapus)

    u = sub.add_parser("uji-sintetis", help="uji rumus dengan robot rekaan (tanpa robot)")
    u.set_defaults(func=cmd_uji_sintetis)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()

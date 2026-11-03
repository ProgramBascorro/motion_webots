#!/usr/bin/env python3
"""Hitung template walking dari percobaan tab "RL Training" Bascorro Studio.

Cara hitung yang dipakai di CHRONUS 2026-10-28 (template "geser kanan RL"):
  1. Ambil percobaan satu sesi, satu target, satu data awal (base_name) --
     parameter yang tidak diacak ikut data awal, jadi data awal lain tidak sebanding.
     Percobaan yang gagal karena sebab luar (mis. "nabrak obstakel") dikeluarkan
     dengan --kecuali.
  2. Skor = kestabilan (normal +2 / hampir jatuh 0 / jatuh -4)
          + kesesuaian (sesuai +3 / mendekati +1 / tidak -1),
     dihitung ULANG dari penilaian (trial lama tanpa "match": move == target).
  3. Tampilkan statistik per setelan dan per parameter -- setelan identik bisa
     memberi hasil berbeda, jadi yang dicari yang TERBUKTI BERULANG.
  4. Uji apakah model (kernel tetangga) memprediksi lebih baik dari rata-rata
     biasa (leave-one-out). Kalau tidak, jangan percaya "titik terbaik" model.
  5. Kandidat = kisi di antara setelan yang sesuai/mendekati & tidak jatuh
     (tidak pernah keluar dari daerah yang sudah diuji). Perkiraan skornya
     disusutkan ke rata-rata keseluruhan seolah ada PRIOR_N percobaan rata-rata
     di titik itu: (bobot*pred + PRIOR_N*rata2) / (bobot + PRIOR_N). Daerah yang
     cuma didukung satu-dua percobaan beruntung tidak bisa menang.
  6. --tulis: simpan kandidat teratas sebagai versi walking BARU (field yang
     tidak diacak disalin dari percobaan terakhir data awal itu).

Bedanya di ALPHONSE:
  * --tulis lewat server penyimpanan Studio (plugin Vite, sama dengan menu
    Training) -- satu-satunya penulis walking_versions.json di robot ini; server
    itu yang membuat cadangan di data/backup/ dan membaca balik isinya. Bascorro
    Studio harus menyala. Versi aktif operator dikembalikan seperti semula.
  * Percobaan menu Training (scripts/rl_impor_training.py) ikut mengacak offset
    pose, jadi dimensinya bisa 9+; kisi kandidat dikasarkan sampai <= MAX_GRID
    titik supaya tidak bermiliar.

Contoh (di dalam container, butuh numpy):
  python3 rl_hitung_template.py --sesi "jalan maju (training lama)" --data-awal "JALAN BARU" \\
      --target maju [--dari-no 60] [--kecuali 12,29]
  ... --tulis --nama "JALAN BARU RL"
  ... --tulis --setelan "x=0.0145,y=0,angle=-0.005,..."   # setelan yang terbukti berulang, bukan kisi
"""
import argparse
import collections
import itertools
import json
import os
import sys
import time
import uuid

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_walking as walk  # noqa: E402
import autotune_walking_core as wcore  # noqa: E402

WS = os.environ.get("OP3_WS", "/ros2_ws")
ST = {"normal": 2, "hampir_jatuh": 0, "jatuh": -4}
MS = {"sesuai": 3, "mendekati": 1, "tidak": -1}
PRIOR_N = 3.0
MAX_GRID = 200000
# Langkah uji terkecil yang bermakna per parameter = satuan jarak kernel.
# Offset pose hanya diacak oleh percobaan menu Training yang diimpor.
SCALE = {"x_move_amplitude": 0.001, "y_move_amplitude": 0.001, "angle_move_amplitude": 0.01,
         "period_time": 0.01, "dsp_ratio": 0.01, "z_move_amplitude": 0.001, "y_swap_amplitude": 0.001,
         "init_x_offset": 0.002, "init_y_offset": 0.002, "init_pitch_offset": 0.01, "hip_pitch_offset": 0.01}


def short(key):
    return key.replace("_move_amplitude", "").replace("_amplitude", "")


def match_of(t):
    return t.get("match") or ("sesuai" if t.get("move") == t.get("target") else "tidak")


def score_of(t):
    return ST.get(t.get("stability"), 0) + MS.get(match_of(t), 0)


def is_good(t):
    return t.get("stability") != "jatuh" and match_of(t) in ("sesuai", "mendekati")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", default=os.path.join(WS, "rl_runs/studio/trials.json"))
    ap.add_argument("--sesi", required=True)
    ap.add_argument("--data-awal", required=True, help="base_name percobaan, mis. 'JALAN BARU'")
    ap.add_argument("--target", required=True, help="maju | mutar | putar_kiri | putar_kanan | geser_kanan | geser_kiri")
    ap.add_argument("--dari-no", type=int, default=0, help="abaikan percobaan bernomor lebih kecil")
    ap.add_argument("--kecuali", default="", help="nomor dikeluarkan, mis. '29,31'")
    ap.add_argument("--tulis", action="store_true", help="simpan kandidat teratas sebagai versi baru")
    ap.add_argument("--nama", default=None, help="nama versi baru (bawaan: '<data awal> RL')")
    ap.add_argument("--setelan", default="",
                    help="tulis setelan ini, bukan kandidat kisi teratas -- untuk saat LOO bilang model tidak lebih "
                         "baik; semua parameter yang diacak wajib disebut, mis. 'x=0.0145,angle=-0.005,...'")
    args = ap.parse_args()

    skip = {int(s) for s in args.kecuali.split(",") if s.strip()}
    trials = json.load(open(args.trials, encoding="utf-8"))["trials"]
    H = sorted([t for t in trials if t.get("session") == args.sesi and t.get("base_name") == args.data_awal
                and t.get("target") == args.target and t.get("no", 0) >= args.dari_no and t.get("no") not in skip],
               key=lambda t: t["no"])
    if len(H) < 5:
        raise SystemExit(f"hanya {len(H)} percobaan cocok -- terlalu sedikit untuk dihitung")

    keys = sorted({k for t in H for k in t.get("varied", {})})
    # Field yang TIDAK diacak harus sama di semua percobaan, kalau tidak data awalnya berubah di tengah.
    fixed_diff = {}
    for k in sorted({k for t in H for k in t["params"]} - set(keys)):
        vals = {json.dumps(t["params"].get(k)) for t in H}
        if len(vals) > 1:
            fixed_diff[k] = vals
    print(f"{len(H)} percobaan (#{H[0]['no']}-#{H[-1]['no']}), diacak: {keys}")
    if fixed_diff:
        print("PERINGATAN -- field tidak diacak berubah antar percobaan (pisahkan dengan --dari-no):")
        for k, v in fixed_diff.items():
            print(f"   {k}: {sorted(v)}")

    y = np.array([score_of(t) for t in H], float)
    fall = np.array([t.get("stability") == "jatuh" for t in H], float)
    good = np.array([is_good(t) for t in H], float)
    X = np.array([[t["params"][k] for k in keys] for t in H], float)
    scale = np.array([SCALE.get(k, 0.01) for k in keys])

    print("\nPER SETELAN  " + " ".join(short(k) for k in keys))
    groups = collections.defaultdict(list)
    for i, t in enumerate(H):
        groups[tuple(round(t["params"][k], 4) for k in keys)].append(i)
    for key, idx in sorted(groups.items(), key=lambda kv: -y[kv[1]].mean()):
        print(f"  {key}  n={len(idx):2d}  skor {y[idx].mean():5.2f}  jatuh {int(fall[idx].sum())}  baik {int(good[idx].sum())}")

    print("\nPER PARAMETER")
    for j, k in enumerate(keys):
        parts = []
        for v in sorted(set(np.round(X[:, j], 4))):
            idx = np.where(np.round(X[:, j], 4) == v)[0]
            parts.append(f"{v}: n={len(idx)} skor={y[idx].mean():.2f} jatuh={100 * fall[idx].mean():.0f}%")
        print(f"  {k}: " + " | ".join(parts))

    def kern(Xtr, ytr, q, h=1.0):
        d = (((Xtr - q) / scale) ** 2).sum(1)
        w = np.exp(-d / (2 * h * h))
        return (w @ ytr) / max(w.sum(), 1e-12), w.sum(), w

    loo_mean, loo_kern = [], []
    for i in range(len(y)):
        tr = np.arange(len(y)) != i
        loo_mean.append(y[tr].mean())
        loo_kern.append(kern(X[tr], y[tr], X[i])[0])
    rm = np.sqrt(np.mean((np.array(loo_mean) - y) ** 2))
    rk = np.sqrt(np.mean((np.array(loo_kern) - y) ** 2))
    print(f"\nGALAT PREDIKSI LOO: rata-rata biasa {rm:.2f} | kernel {rk:.2f}"
          + ("  -> model TIDAK lebih baik; data didominasi noise, andalkan setelan yang terbukti berulang"
             if rk >= rm - 0.05 else "  -> model sedikit membantu"))

    G = X[good.astype(bool)]
    if len(G) == 0:
        raise SystemExit("belum ada percobaan sesuai/mendekati & tidak jatuh -- latih dulu")
    steps = scale / 2

    def axis(j):
        lo, hi = G[:, j].min(), G[:, j].max()
        return np.round(np.arange(lo, hi + steps[j] / 2, steps[j]), 5) if hi > lo else np.array([lo])

    axes = [axis(j) for j in range(len(keys))]
    while np.prod([len(a) for a in axes], dtype=float) > MAX_GRID:
        j = int(np.argmax([len(a) for a in axes]))
        steps[j] *= 2
        axes[j] = axis(j)
    if (steps > scale / 2).any():
        print(f"\n(kisi dikasarkan supaya <= {MAX_GRID} titik: "
              + ", ".join(f"{short(k)} langkah {s:g}" for k, s, s0 in zip(keys, steps, scale / 2) if s > s0) + ")")
    cands = []
    for q in itertools.product(*axes):
        q = np.array(q)
        pred, W, w = kern(X, y, q)
        pf = (w @ fall) / max(W, 1e-12)
        pg = (w @ good) / max(W, 1e-12)
        cands.append(((W * pred + PRIOR_N * y.mean()) / (W + PRIOR_N), pred, pf, pg, W, q))
    cands.sort(key=lambda c: -c[0])
    print(f"\nKANDIDAT TERATAS (skor disusutkan ke rata-rata {y.mean():.2f}, prior {PRIOR_N:g} percobaan)")
    for obj, pred, pf, pg, W, q in cands[:8]:
        print("  " + " ".join(f"{short(k)}={v:.4f}" for k, v in zip(keys, q))
              + f"  skor {obj:.2f} (mentah {pred:.2f})  jatuh~{100 * pf:.0f}%  baik~{100 * pg:.0f}%  bobot {W:.1f}")

    best = cands[0][5]
    rng = np.random.default_rng(0)
    wins = 0
    for _ in range(2000):
        idx = rng.integers(0, len(y), len(y))
        wins += kern(X[idx], y[idx], best)[0] > kern(X[idx], y[idx], X[groups_most(groups)])[0]
    print(f"\nbootstrap: kandidat teratas > setelan paling sering diuji pada {100 * wins / 2000:.0f}% sampel ulang")

    if args.setelan:
        names = {**{short(k): k for k in keys}, **{k: k for k in keys}}
        chosen = {}
        for item in args.setelan.split(","):
            k, _, v = item.partition("=")
            if k.strip() not in names:
                raise SystemExit(f"--setelan: '{k.strip()}' bukan parameter yang diacak ({', '.join(map(short, keys))})")
            chosen[names[k.strip()]] = float(v)
        missing = [short(k) for k in keys if k not in chosen]
        if missing:
            raise SystemExit(f"--setelan: belum disebut {', '.join(missing)} -- semua parameter yang diacak wajib diisi")
        best = np.array([chosen[k] for k in keys])
        pred, W, w = kern(X, y, best)
        print("\nSETELAN PILIHAN  " + " ".join(f"{short(k)}={v:.4f}" for k, v in zip(keys, best))
              + f"  skor model {pred:.2f}  jatuh~{100 * (w @ fall) / max(W, 1e-12):.0f}%  bobot {W:.1f}")

    if not args.tulis:
        print("\n(belum ditulis -- tambahkan --tulis untuk menyimpan sebagai versi baru)")
        return
    name = args.nama or f"{args.data_awal} RL"
    try:
        store = walk.studio_request("GET")
    except (OSError, ValueError, walk.StudioRefused) as error:
        raise SystemExit(f"server Studio {walk.STUDIO_URL} tidak menjawab ({error}) -- nyalakan Bascorro Studio dulu")
    versions = store.get("versions")
    if not isinstance(versions, list):
        raise SystemExit(f"{walk.STUDIO_URL} bukan server penyimpanan versi walking")
    if any(v.get("name", "").strip().lower() == name.lower() for v in versions):
        raise SystemExit(f'versi "{name}" sudah ada -- pakai --nama lain (versi lama tidak ditimpa)')
    params = dict(H[-1]["params"])
    params.update({k: float(round(v, 5)) for k, v in zip(keys, best)})
    if not wcore.leg_reachable(params):
        raise SystemExit("kandidat teratas DI LUAR jangkauan kaki (IK gagal tiap siklus) -- tidak ditulis")
    now = int(time.time() * 1000)
    version = {"id": str(uuid.uuid4()), "name": name, "createdAt": now, "updatedAt": now, "params": params}
    active = store.get("activeVersionId")
    walk.studio_request("POST", {"op": "upsert", "version": version})
    if active and active != version["id"]:
        # upsert menjadikan versi baru aktif; pilihan operator di Studio dikembalikan
        walk.studio_request("POST", {"op": "activate", "id": active})
    stored = next((v for v in walk.studio_request("GET")["versions"] if v.get("id") == version["id"]), None)
    if stored is None or stored.get("params") != params:
        raise SystemExit(f'versi "{name}" TIDAK terbaca balik sama dari server Studio -- periksa Studio')
    print(f'\nversi BARU "{name}" ditulis lewat server Studio (dibaca balik sama; cadangan di data/backup/)')
    print("REFRESH Studio sebelum menekan Save apa pun -- cache peramban bisa mengirim daftar lama.")


def groups_most(groups):
    """Indeks satu percobaan dari setelan yang paling sering diuji (pembanding bootstrap)."""
    return max(groups.values(), key=len)[0]


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Masukkan percobaan jalan maju dari menu Training (data/autotune) ke tab "RL Training".

Keputusan operator 2026-10-28:
  * jb1 + jalan-baru-1026-2 (sama-sama dari JALAN BARU; offset pose jb1 sudah
    dimigrasi ke acuan page 2 sekarang lewat --follow-reference) digabung jadi
    SATU sesi -> tab RL membuka sesi itu dan mode Belajar langsung memakainya.
  * coba (23 Okt) masih memakai acuan page 2 LAMA (init pose beda), jadi sesi
    arsip sendiri dengan data awal "JALAN BARU (page 2 lama)" -- tidak pernah
    tercampur ke riwayat belajar JALAN BARU.
  * page104-* (page Action Editor, bukan walking) dan jalan-baru-1026 (kosong)
    tidak ikut.

Pemetaan penilaian:
  kestabilan  = label operator (jatuh / hampir / normal); tanpa label -> detektor IMU
  gerakan     = jalan maju (x_move dikunci maju); |yaw| < 5 deg -> sesuai, selain itu
                mendekati (sama dengan kolom "arah" laporan Training)
  pose condong (gerbang pose, "start" tidak dikirim) -> hampir jatuh, diam, tidak sesuai
  tanpa metrik yaw -> diam / tidak jelas, tidak sesuai
  catatan     = sumber + label + belok, plus teks feedback operator kalau ada

Idempoten: id = "autotune-<sesi>-<n>", yang sudah ada dilewati. Tanpa --tulis hanya
melapor. Jalankan di container saat tidak ada yang sedang menyimpan penilaian di tab RL.

  python3 scripts/rl_impor_training.py            # lihat dulu
  python3 scripts/rl_impor_training.py --tulis
"""
import argparse
import collections
import datetime
import json
import os

WS = os.environ.get("OP3_WS", "/ros2_ws")
AUTOTUNE = os.path.join(WS, "src/bascorro_studio/data/autotune")
# (sesi menu Training, sesi tab RL, data awal)
SOURCES = [
    ("jb1", "jalan maju (training lama)", "JALAN BARU"),
    ("jalan-baru-1026-2", "jalan maju (training lama)", "JALAN BARU"),
    ("coba", "arsip coba 23-10 (page 2 lama)", "JALAN BARU (page 2 lama)"),
]
STRAIGHT_DEG = 5.0
ST = {"normal": 2, "hampir_jatuh": 0, "jatuh": -4}
MS = {"sesuai": 3, "mendekati": 1, "tidak": -1}
LABEL = {"jatuh": "jatuh", "hampir": "hampir_jatuh", "normal": "normal"}


def ms(iso):
    return int(datetime.datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp() * 1000)


def feedback_texts(path):
    out = collections.defaultdict(list)
    try:
        with open(path, encoding="utf-8") as handle:
            for line in handle:
                entry = json.loads(line)
                if entry.get("trial") and entry.get("text"):
                    out[entry["trial"]].append(entry["text"].strip())
    except OSError:
        pass
    return out


def convert(src, session, base_name, state, t, texts):
    metrics = t.get("metrics") or {}
    label = t.get("label")
    yaw = metrics.get("yaw_drift_deg")
    parts = [f"impor {src} #{t['n']}"]
    if metrics.get("stance_unstable"):
        stability, move, match = "hampir_jatuh", "lainnya", "tidak"
        parts.append(f"{metrics.get('note', 'pose berdiri condong')} -- start tidak dikirim")
    else:
        if label in LABEL:
            stability = LABEL[label]
            parts.append(f"label operator: {label}")
        else:
            stability = "jatuh" if metrics.get("fell") else "normal"
            parts.append(f"tanpa label operator (detektor IMU: {'jatuh' if metrics.get('fell') else 'tidak jatuh'})")
        if yaw is None:
            move, match = "lainnya", "tidak"
            parts.append("tanpa metrik yaw")
        else:
            arah = "lurus" if abs(yaw) < STRAIGHT_DEG else ("kanan" if yaw < 0 else "kiri")
            move, match = "maju", "sesuai" if arah == "lurus" else "mendekati"
            parts.append(f"belok {yaw:+.1f} deg ({arah})")
    parts += [f'operator: "{text}"' for text in texts.get(t["n"], [])]
    generation = t.get("generation")
    return {
        "id": f"autotune-{src}-{t['n']}",
        "session": session,
        "target": "maju",
        "base_name": base_name,
        "mode": "autotune",
        "basis": f"impor menu Training {src} #{t['n']} ({t['kind']}"
                 + (f" gen {generation}" if generation is not None else "") + ")",
        "duration": state["walk_sec"],
        # baseline = data awal apa adanya; CMA mengacak semua parameter sesi itu
        "varied": {} if t["kind"] == "baseline" else {k: t["params"][k] for k in state["params"]},
        "params": {**state["start_params"], **t["params"]},
        "stability": stability,
        "move": move,
        "match": match,
        "score": ST[stability] + MS[match],
        "note": " · ".join(parts),
        "at": ms(t["time"]),
        "imported": {
            "from": f"src/bascorro_studio/data/autotune/{src}/state.json",
            "n": t["n"], "kind": t["kind"], "generation": generation, "label": label,
            "cost": t.get("cost"), "yaw_drift_deg": yaw,
            "reference_id": t.get("reference_id") or state.get("reference_id"),
            "metrics_version": state.get("metricsVersion"),
        },
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", default=os.path.join(WS, "rl_runs/studio/trials.json"))
    ap.add_argument("--tulis", action="store_true", help="tulis ke trials.json (tanpa ini hanya melapor)")
    args = ap.parse_args()

    try:
        with open(args.trials, encoding="utf-8") as handle:
            existing = json.load(handle)["trials"]
    except OSError:
        existing = []
    known = {t.get("id") for t in existing}

    new = []
    for src, session, base_name in SOURCES:
        with open(os.path.join(AUTOTUNE, src, "state.json"), encoding="utf-8") as handle:
            state = json.load(handle)
        texts = feedback_texts(os.path.join(AUTOTUNE, src, "feedback.jsonl"))
        new += [convert(src, session, base_name, state, t, texts) for t in state["trials"]]
    skipped = [t for t in new if t["id"] in known]
    new = sorted((t for t in new if t["id"] not in known), key=lambda t: t["at"])
    for t in new:
        t["no"] = 1 + max((x["no"] for x in existing + new if x["session"] == t["session"] and "no" in x), default=0)

    for session in dict.fromkeys(t["session"] for t in new):
        rows = [t for t in new if t["session"] == session]
        print(f"\n== {session}: {len(rows)} percobaan baru (data awal {rows[0]['base_name']})")
        print("   kestabilan:", dict(collections.Counter(t["stability"] for t in rows)),
              " kesesuaian:", dict(collections.Counter(t["match"] for t in rows)))
        for t in rows:
            print(f"   #{t['no']:2d} skor {t['score']:+d}  {t['stability']:12s} {t['match']:9s}  {t['note']}")
    if skipped:
        print(f"\n{len(skipped)} percobaan sudah ada (id sama), dilewati")
    if not new:
        print("\ntidak ada yang perlu dimasukkan")
        return
    if not args.tulis:
        print(f"\n(belum ditulis -- tambahkan --tulis untuk menyimpan ke {args.trials})")
        return
    os.makedirs(os.path.dirname(args.trials), exist_ok=True)
    tmp = args.trials + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump({"trials": existing + new}, handle, indent=2)
    os.replace(tmp, args.trials)
    print(f"\n{len(new)} percobaan ditulis ke {args.trials}. Tab RL Training: tekan Muat ulang.")


if __name__ == "__main__":
    main()

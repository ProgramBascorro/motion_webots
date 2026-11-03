"""Feedback operator -> perubahan yang DIPAKAI iterasi training berikutnya.

MURNI (tanpa ROS); dipakai pelatih (autotune_walking.py, autotune_action.py --
tombol "c" saat memberi label) dan menu (scripts/training.py feedback-plan/apply).
Alurnya sama untuk Bascorro Studio dan Action Editor:

  1. PEMAHAMAN  catatan teks (training_feedback.understand): per kalimat perilaku
                yang dikeluhkan + frasa pemicunya, fase, sisi, arah yang diinginkan,
                hal yang sudah baik, potongan yang tidak dipahami.
  2. PERUBAHAN  pemahaman + KONTEKS (preset/page, nilai kandidat yang dinilai,
                metrik IMU-nya, langkah feedback sebelumnya) -> variabel gerakan
                dengan arah dan besar langkah tertentu (STEP), bobot penilaian, atau
                "tidak bisa" + alasan / informasi yang masih dibutuhkan.
  3. VALIDASI   batas nilai (tabel BOUNDS yang sama dengan pencarian CMA), satuan,
                jangkauan kaki IK (walking) / page hasil tanpa sendi terjepit (page).
  4. LANGKAH    disimpan di state.json sesi, "feedback_steps", status "menunggu".
                Pelatih menjalankannya SEBAGAI PERCOBAAN BERIKUTNYA (jenis
                "feedback"), sebelum kandidat CMA mana pun.
  5. BUKTI      sesudah percobaan itu: nilai yang tercatat dipakai (parameter
                walking yang DIBACA BALIK dari walking_module / page kerja yang
                DIBACA BALIK dari .bin) dicocokkan dengan nilai langkah, lalu
                dibandingkan dengan percobaan yang dinilai feedback.
  6. LANJUT     operator mempertahankan -> titik tengah CMA pindah ke titik itu
                (generasi yang belum selesai dibuang), variabel di luar ruang CMA
                masuk "fixed" dan ikut di SEMUA percobaan berikutnya; atau
                membatalkan -> bobot dikembalikan, CMA tidak berubah.

Status langkah: usulan (belum disimpan) -> menunggu -> diuji -> dipakai | dibatalkan;
juga ditolak (operator memilih "catatan saja"), diganti (usulan lebih baru untuk
percobaan yang sama), tidak_berlaku (tidak ada perubahan yang sah). "Diterapkan"
HANYA berarti status dipakai/diuji: nilai itu benar-benar sudah dijalankan robot.

Batas jujur: pemahaman = pola kata (bukan model bahasa); arah perubahan = pedoman
tuning tetap (tabel _walk_effects/_action_effects), besar langkah kecil dan
dijepit; yang memutuskan berhasil-tidaknya tetap percobaan di robot + label operator.
"""

import math

import numpy as np

import autotune_action_core as acore
import autotune_walking_core as wcore
import training_feedback as feedback
import training_sources as src

# Besar SATU langkah feedback per variabel, dalam satuan aslinya (WalkingParam = satuan
# ROS m / rad / s; kenop page = derajat / pengali). Sengaja kecil: kandidat page104
# dengan kenop ~1 deg sudah jatuh (sesi 26 Okt), jadi page 0,5 deg per langkah.
STEP = {
    "hip_pitch_offset": (math.radians(1.0), "rad"),
    "init_pitch_offset": (math.radians(1.0), "rad"),
    "z_move_amplitude": (0.005, "m"),
    "y_swap_amplitude": (0.004, "m"),
    "dsp_ratio": (0.03, ""),
    "period_time": (0.03, "s"),
    "soft_start_time": (0.25, "s"),
    "angle_move_amplitude": (0.01, "rad"),
    "pitch_hip_deg": (0.5, "deg"),
    "pitch_ankle_deg": (0.5, "deg"),
    "roll_hip_deg": (0.5, "deg"),
    "roll_ankle_deg": (0.5, "deg"),
    "time_scale": (0.05, "x"),
}
PHASE_TIME_STEP = 0.10      # time_scale fase (mis. kembali ke posisi siap): pengali +10% per langkah
GROW, SHRINK, MAX_GROWTH = 1.5, 0.5, 2.0
WEIGHT_RANGE = (0.5, 2.0)   # bobot tiap metrik dijaga 0,5..2x bobot bawaan, lalu dinormalkan
MEANINGFUL = 0.05           # selisih skor < 5% = setara (derau satu percobaan)
TERMINAL = ("dipakai", "dibatalkan")


# ---- sesi -------------------------------------------------------------------------

def kind_of(state):
    return "action" if state.get("source") == "action_editor" else "studio"


def default_weights(state):
    return wcore.WEIGHTS if kind_of(state) == "studio" else acore.PROFILES[state["profile"]]["weights"]


def current_weights(state):
    return state.get("weights") or default_weights(state)


def metric_text(state):
    return wcore.METRIC_TEXT if kind_of(state) == "studio" else acore.METRIC_TEXT


def full_params(state, trial):
    """Nilai lengkap yang dijalankan percobaan itu (preset 29 field / kenop page)."""
    return dict(state["start_params"], **trial["params"])


def effective_start(state):
    """Titik dasar kandidat CMA: nilai awal + variabel yang ditetapkan feedback ("fixed")."""
    return dict(state["start_params"], **state.get("fixed", {}))


def _close(a, b):
    try:
        return abs(float(a) - float(b)) <= 1e-6 * max(1.0, abs(float(a)), abs(float(b)))
    except (TypeError, ValueError):
        return a == b


def recorded(state, params):
    """Yang dicatat per percobaan: semua parameter yang dicari + variabel lain yang
    berbeda dari nilai awal (hasil feedback) -- cukup untuk mengulang persis."""
    out = {k: params[k] for k in state["params"]}
    for key, value in params.items():
        if key not in out and not _close(state["start_params"].get(key, _neutral(key)), value):
            out[key] = value
    return out


def _neutral(key):
    knob, _ = acore.split_key(key)
    return 1.0 if knob == "time_scale" else 0.0


def extra_keys(state):
    """Variabel di luar ruang CMA yang pernah diubah feedback (untuk kolom CSV/laporan)."""
    keys = []
    for trial in state["trials"]:
        keys += [k for k in trial.get("params", {}) if k not in state["params"] and k not in keys]
    return keys


def steps(state):
    return state.get("feedback_steps", [])


def pending(state):
    return next((s for s in steps(state) if s["status"] == "menunggu"), None)


def find_step(state, step_id):
    return next((s for s in steps(state) if s["id"] == step_id), None)


def step_of_trial(state, n):
    """Langkah feedback yang DIJALANKAN di percobaan n (None kalau percobaan biasa)."""
    return next((s for s in steps(state) if s.get("trial") == n), None)


def _trainer(state):
    """(apakah sah, hitung ulang semua skor) milik pelatih sesi itu -- impor saat dipakai
    karena pelatih sendiri mengimpor modul ini."""
    if kind_of(state) == "action":
        import autotune_action as act
        return act.usable_for(state), act.recost_all
    import autotune_walking as walk
    return walk.usable, walk.recost_all


def _cost(state, trial):
    runs = [trial["cost"]] + [t["cost"] for t in state["trials"] if t.get("rerun_of") == trial["n"]]
    return sum(runs) / len(runs), len(runs) - 1


# ---- konteks: page & catatan --------------------------------------------------------

def snapshot(state):
    page = src.decode_page(bytes.fromhex(state["page_hex"]), 0)
    page["index"] = state["page"]
    return page


def phases_of(state):
    if kind_of(state) != "action":
        return None
    return acore.page_phases(snapshot(state), "%s %s" % (state.get("page_name", ""), state.get("motion_name", "")))


def same_config(state, source):
    """Kandidat dengan nilai PERSIS sama dengan sumber (mis. semua baseline): catatan pada
    salah satunya berbicara tentang konfigurasi yang sama."""
    target = full_params(state, source)
    out = {source["n"]}
    for trial in state["trials"]:
        if trial.get("rerun_of") or "params" not in trial:
            continue
        other = full_params(state, trial)
        if set(other) == set(target) and all(_close(other[k], target[k]) for k in target):
            out.add(trial["n"])
    return out


def open_notes(state, session_dir, source):
    """Catatan untuk kandidat ini (dan kandidat berkonfigurasi sama) yang belum dipakai
    langkah yang sudah dijalankan."""
    used = {note for s in steps(state) if s["status"] in TERMINAL + ("diuji",) for note in s["notes"]}
    candidates = {str(n) for n in same_config(state, source)}
    return [n for n in feedback.notes(session_dir)
            if str(n.get("candidate", n.get("trial"))) in candidates and n["id"] not in used]


# ---- pemahaman -> efek ----------------------------------------------------------------
# efek: ("param", kunci, arah +1/-1, alasan) | ("weight", metrik, pengali)
#       ("limit", teks) | ("need", teks) | ("evidence", teks)

def _walk_effects(state, b, note_behaviors, metrics):
    effects, bid, phases = [], b["id"], set(b["phases"])
    locked = {"x_move_amplitude", "y_move_amplitude"} | (
        {"angle_move_amplitude"} if wcore.motion_keeps_turn(state.get("motion", "maju")) else set())
    lean = (metrics or {}).get("lean_deg")
    if bid in ("condong_belakang", "condong_depan"):
        forward = bid == "condong_belakang"
        effects.append(("param", "hip_pitch_offset", +1 if forward else -1,
                        "pinggul %s: badan atas condong ke %s" % ("lebih menekuk" if forward else "lebih tegak",
                                                                   "DEPAN" if forward else "BELAKANG")))
        effects.append(("weight", "lean_deg", 1.5))
        if lean is not None:
            agrees = (lean < -1.0) if forward else (lean > 1.0)
            effects.append(("evidence", "IMU percobaan ini: condong %+.1f deg saat jalan dibanding berdiri (%s) -- %s"
                            % (lean, "hampir tegak" if abs(lean) < 1.0 else "ke belakang" if lean < 0 else "ke depan",
                               "cocok dengan feedback" if agrees else "TIDAK sejelas feedback; arah tetap mengikuti "
                               "operator, periksa lagi di percobaan berikutnya")))
        if phases and phases <= {"awal"}:
            effects.append(("limit", "walking_module tidak punya condong khusus awal langkah: hip_pitch_offset "
                                     "berlaku sejak berdiri sampai berhenti"))
    elif bid == "tidak_menapak":
        leaning = [x["id"] for x in note_behaviors if x["id"] in ("condong_belakang", "condong_depan")]
        if leaning:
            forward = leaning[0] == "condong_belakang"
            effects.append(("param", "init_pitch_offset", -1 if forward else +1,
                            "seluruh badan diputar ke %s di engkel supaya telapak menapak rata"
                            % ("DEPAN" if forward else "BELAKANG")))
        else:
            effects.append(("need", "telapak tidak menapak: bagian mana yang terangkat (tumit atau ujung kaki), "
                                    "atau badan condong ke mana"))
    elif bid == "condong_samping":
        effects.append(("weight", "tilt_std_deg", 1.3))
        effects.append(("limit", "walking_module tidak punya parameter condong ke samping yang simetris "
                                 "(init_roll_offset memiringkan kedua telapak berlawanan arah); yang diubah hanya "
                                 "penilaian sebaran kemiringan"))
    elif bid == "kaki_mentok":
        effects.append(("param", "z_move_amplitude", +1, "kaki diangkat lebih tinggi tiap langkah"))
        if b["foot"] in ("kiri", "kanan"):
            effects.append(("limit", "tinggi angkat walking_module berlaku untuk KEDUA kaki (tidak ada setelan "
                                     "per kaki); kaki %s ikut naik" % ("kanan" if b["foot"] == "kiri" else "kiri")))
    elif bid in ("goyah", "berat_badan"):
        effects.append(("param", "dsp_ratio", +1, "fase dua kaki menapak lebih lama tiap langkah (perpindahan berat "
                                                  "lebih bertahap, lebih stabil)"))
        effects += [("weight", "gyro_rms_dps", 1.4), ("weight", "tilt_std_deg", 1.3)]
    elif bid == "jatuh":
        effects += [("weight", "gyro_rms_dps", 1.3), ("weight", "tilt_std_deg", 1.3),
                    ("need", "jatuh ke arah mana (depan/belakang/kiri/kanan) -- dengan arah, condong badan bisa "
                             "diubah, bukan hanya penilaiannya")]
    elif bid == "kasar":
        if phases & {"awal", "berhenti"}:
            effects.append(("param", "soft_start_time", +1, "langkah dimulai (dan dihentikan) lebih bertahap: "
                                                            "amplitudo naik/turun lebih lama"))
        else:
            effects += [("weight", "gyro_rms_dps", 1.3),
                        ("need", "kapan terasa kuat/kasar: di AWAL jalan, tiap langkah, atau saat BERHENTI")]
    elif bid == "belok":
        effects.append(("weight", "yaw_drift_deg", 1.5))
        want, yaw = b["want"], (metrics or {}).get("yaw_drift_deg")
        if want is None and yaw is not None and abs(yaw) >= wcore.TURN_STRAIGHT_DEG:
            want = "kanan" if yaw > 0 else "kiri"
            effects.append(("evidence", "arah dari IMU percobaan ini: belok %s" % wcore.turn_text(yaw)))
        if "angle_move_amplitude" in locked:
            effects.append(("limit", "preset putar: belok adalah perintahnya sendiri (angle_move_amplitude dikunci)"))
        elif want:
            effects.append(("param", "angle_move_amplitude", +1 if want == "kiri" else -1,
                            "koreksi belok ke %s (positif = kiri, REP-103)" % want.upper()))
        else:
            effects.append(("need", "belok ke mana (kiri/kanan)"))
    elif bid in ("tempo_cepat", "tempo_lambat"):
        effects.append(("param", "period_time", +1 if bid == "tempo_cepat" else -1,
                        "satu langkah lebih %s" % ("lama (tempo lebih lambat)" if bid == "tempo_cepat" else
                                                   "singkat (tempo lebih cepat)")))
    elif bid == "sulit_kembali":
        effects.append(("limit", "training walking menilai saat berjalan; kembali ke posisi siap sesudah stop "
                                 "tidak diukur"))
    elif bid == "tendangan_lemah":
        effects.append(("limit", "sesi ini melatih preset walking, bukan page tendangan"))
    return effects


def _action_effects(state, b, note_behaviors, ph):
    effects, bid, profile = [], b["id"], state["profile"]
    weights = current_weights(state)
    phase_of = {"tendang": "satu_kaki", "kembali": "kembali", "awal": "awal"}
    named = [phase_of[p] for p in b["phases"] if p in phase_of]
    available = (ph or {}).get("phases", {})

    def where(default):
        """(kunci fase, rentang step) atau (None, None) = seluruh page."""
        for name in named + [default]:
            if name in available:
                return name, available[name]
        return None, None

    def lateral(toward, default, reason):
        name, steps_range = where(default)
        sign = acore.BODY_RIGHT["roll_ankle_deg"] * (1 if toward == "kanan" else -1)
        effects.append(("param", acore.phase_key("roll_ankle_deg", steps_range), sign,
                        "%s -- badan condong ke %s di atas engkel %s" % (
                            reason, toward.upper(), "selama step %d-%d (%s)" % (steps_range[0], steps_range[1],
                                                                              acore.PHASE_NAME[name])
                            if steps_range else "di SEMUA step (fase tidak terlihat dari data page)")))

    def pitch(forward, default, reason):
        name, steps_range = where(default)
        sign = acore.BODY_FORWARD["pitch_ankle_deg"] * (1 if forward else -1)
        effects.append(("param", acore.phase_key("pitch_ankle_deg", steps_range), sign,
                        "%s -- badan condong ke %s di atas engkel %s" % (
                            reason, "DEPAN" if forward else "BELAKANG", "selama step %d-%d (%s)" % (
                                steps_range[0], steps_range[1], acore.PHASE_NAME[name])
                            if steps_range else "di SEMUA step")))

    def weight(metric, factor):
        if metric in weights:
            effects.append(("weight", metric, factor))

    def tempo(slower, reason, default="satu_kaki"):
        name, steps_range = where(default)
        if profile == "kick" and name != "kembali":
            effects.append(("limit", "kecepatan tendangan sengaja DIKUNCI (lebih pelan = lebih lemah tapi selalu "
                                     "menang skor); hanya fase KEMBALI ke posisi siap yang boleh diubah tempo-nya"))
            return
        if profile == "kick":
            effects.append(("param", acore.phase_key("time_scale", steps_range), +1 if slower else -1,
                            "%s -- step %d-%d (%s) %s, kekuatan tendangan tidak berubah" % (
                                reason, steps_range[0], steps_range[1], acore.PHASE_NAME[name],
                                "lebih pelan" if slower else "lebih cepat")))
        else:
            effects.append(("param", "time_scale", +1 if slower else -1,
                            "%s -- seluruh page %s" % (reason, "lebih pelan" if slower else "lebih cepat")))

    kick_side = (ph or {}).get("kick")
    if bid == "kaki_mentok":
        foot = b["foot"] if b["foot"] in ("kiri", "kanan") else acore.SIDE_TEXT.get(kick_side)
        if foot is None:
            effects.append(("need", "kaki mana yang mentok (kiri/kanan)"))
        else:
            toward = "kanan" if foot == "kiri" else "kiri"
            lateral(toward, "satu_kaki", "berat dipindah ke kaki %s supaya kaki %s lebih bebas dari lantai" % (
                toward, foot))
            effects.append(("evidence", "tinggi kaki tidak terukur IMU: berhasil-tidaknya dinilai dari label/feedback "
                                        "operator"))
    elif bid == "condong_samping":
        lateral(b["want"], "satu_kaki", "badan diminta lebih ke %s" % b["want"])
        weight("tilt_max_deg", 1.3)
    elif bid in ("condong_belakang", "condong_depan"):
        pitch(bid == "condong_belakang", "satu_kaki", "badan diminta lebih ke %s" % (
            "depan" if bid == "condong_belakang" else "belakang"))
        weight("tilt_max_deg", 1.3)
        weight("tilt_std_deg", 1.3)
        weight("settle_dev_deg", 1.2)
    elif bid in ("goyah", "berat_badan"):
        weight("gyro_rms_dps", 1.4)
        weight("tilt_max_deg", 1.3)
        weight("tilt_std_deg", 1.3)
        effects.append(("need", "goyah ke arah mana (ke kaki %s, ke luar, ke depan, ke belakang) -- dengan arah, badan "
                                "bisa digeser; tanpa arah hanya penilaian kestabilan yang dinaikkan"
                        % (acore.SIDE_TEXT[kick_side] if kick_side else "tendang")))
    elif bid == "sulit_kembali":
        weight("settle_dev_deg", 1.5)
        weight("settle_gyro_dps", 1.4)
        if "kembali" in available:
            tempo(True, "kembali ke posisi siap diberi waktu lebih", "kembali")
        else:
            effects.append(("limit", "fase kembali tidak terlihat dari data page"))
    elif bid in ("kasar", "tempo_cepat"):
        weight("gyro_rms_dps", 1.3)
        tempo(True, "gerakan diminta lebih halus/pelan")
    elif bid == "tempo_lambat":
        weight("time_to_upright_s", 1.5)
        tempo(False, "gerakan diminta lebih cepat")
    elif bid == "jatuh":
        weight("gyro_rms_dps", 1.3)
        weight("tilt_max_deg", 1.3)
        effects.append(("need", "jatuh ke arah mana (depan/belakang/kiri/kanan) dan kapan (persiapan/menendang/kembali)"))
    elif bid == "belok":
        weight("yaw_abs_deg", 1.5)
        if "yaw_abs_deg" not in weights:
            effects.append(("limit", "profil %s tidak mengukur badan berputar" % profile))
    elif bid == "tendangan_lemah":
        effects.append(("limit", "kekuatan tendangan tidak dilatih (kecepatan dikunci, skor tidak mengukur bola); ubah "
                                 "step tendangannya di Action Editor"))
    elif bid == "tidak_menapak":
        effects.append(("need", "telapak mana yang tidak menapak dan bagian mana yang terangkat (tumit/ujung/sisi)"))
    return effects


# ---- usulan ---------------------------------------------------------------------------

def _new_weights(current, default, factors):
    raw = {k: current[k] * factors.get(k, 1.0) for k in current}
    raw = {k: min(max(v, WEIGHT_RANGE[0] * default[k]), WEIGHT_RANGE[1] * default[k]) for k, v in raw.items()}
    total = sum(raw.values()) or 1.0
    return {k: round(v / total, 4) for k, v in raw.items()}


def _bounds(state, key, base):
    """(bawah, atas) nilai sah variabel ini, atau None kalau tidak ada batas aman."""
    if kind_of(state) == "studio":
        if key in state["params"]:
            i = state["params"].index(key)
            return state["lo"][i], state["hi"][i]
        if key not in wcore.BOUNDS:
            return None
        space = wcore.Space([key], state["start_params"], state["reference_offsets"])
        return float(space.lo[0]), float(space.hi[0])
    knob, steps_range = acore.split_key(key)
    if knob not in acore.BOUNDS:
        return None
    _, lo, hi = acore.BOUNDS[knob]
    if steps_range is None:
        if key in state["params"]:
            i = state["params"].index(key)
            return state["lo"][i], state["hi"][i]
        return lo, hi
    # kenop fase bertumpuk di atas kenop global: jumlahnya yang dijaga di dalam batas
    globals_ = [base.get(knob, _neutral(knob))]
    if knob == "time_scale":
        return lo / max(globals_), hi / max(globals_)
    return lo - min(globals_), hi - max(globals_)


def _display(key, value):
    knob, _ = acore.split_key(key)
    unit = STEP.get(knob, (None, ""))[1]
    if unit == "rad":
        return "%.4f rad (%.2f deg)" % (value, math.degrees(value))
    if unit == "deg":
        return "%+.2f deg" % value
    if unit == "x":
        return "x%.3f" % value
    if unit == "m":
        return "%.4f m (%.1f mm)" % (value, 1000 * value)
    if unit == "s":
        return "%.3f s" % value
    return "%.4f" % value


def propose(state, session_dir, source, where, metrics=None, extra_notes=(), previous=None):
    """Usulan langkah feedback untuk kandidat `source` (dict percobaan; boleh percobaan yang
    sedang dilabeli dan belum ada di state). previous = langkah yang menghasilkan percobaan
    sumber (bawaan: dicari dari state). Tidak mengubah apa pun."""
    kind = kind_of(state)
    waiting = pending(state)
    notes = open_notes(state, session_dir, source)
    for note in extra_notes:
        if note["id"] not in {n["id"] for n in notes}:
            notes.append(note)
    # langkah yang masih menunggu akan DIGANTI usulan ini: bobot dihitung dari sebelum langkah itu
    weights_before = dict(waiting["weights_before"] if waiting else current_weights(state))
    base = full_params(state, source)
    metrics = metrics if metrics is not None else source.get("metrics")
    ph = phases_of(state)
    previous = previous or step_of_trial(state, source["n"])
    understood, proposals, factors, limits, needs, evidence = [], {}, {}, [], [], []
    for note in notes:
        result = feedback.understand(note["text"])
        understood.append({"note": note["id"], "candidate": note.get("candidate"), "text": note["text"],
                           "sentences": result["sentences"]})
        behaviors = [b for s in result["sentences"] for b in s["behaviors"]]
        for s in result["sentences"]:
            needs += [("bagian ini belum dipahami: \"%s\" -- tulis perilakunya (mis. miring ke mana, kaki mana yang "
                       "mentok, kapan)" % u, note["id"]) for u in s["unclear"]]
        for b in behaviors:
            trigger = {"note": note["id"], "behavior": b["id"], "phrases": b["phrases"]}
            effects = (_walk_effects(state, b, behaviors, metrics) if kind == "studio"
                       else _action_effects(state, b, behaviors, ph))
            for effect in effects:
                if effect[0] == "param":
                    _, key, direction, reason = effect
                    entry = proposals.setdefault(key, {"directions": set(), "reasons": [], "triggers": [],
                                                       "scale": 0.0, "still": False})
                    entry["directions"].add(direction)
                    if reason not in entry["reasons"]:
                        entry["reasons"].append(reason)
                    entry["triggers"].append(trigger)
                    entry["scale"] = max(entry["scale"], b["scale"])
                    entry["still"] = entry["still"] or b["still"]
                elif effect[0] == "weight":
                    factors[effect[1]] = min(MAX_GROWTH, factors.get(effect[1], 1.0) * effect[2])
                    factors.setdefault("_triggers", {}).setdefault(effect[1], []).append(trigger)
                elif effect[0] == "limit":
                    limits.append((effect[1], note["id"]))
                elif effect[0] == "need":
                    needs.append((effect[1], note["id"]))
                else:
                    evidence.append((effect[1], note["id"]))
    weight_triggers = factors.pop("_triggers", {})
    changes, rejected = [], []
    after = dict(base)
    for key, entry in proposals.items():
        if len(entry["directions"]) > 1:
            rejected.append({"key": key, "why": "feedback bertentangan (diminta naik DAN turun) -- tidak diubah",
                             "triggers": entry["triggers"]})
            continue
        direction = entry["directions"].pop()
        knob, steps_range = acore.split_key(key)
        unit_step = PHASE_TIME_STEP if knob == "time_scale" and steps_range else STEP[knob][0]
        factor, why = entry["scale"] or 1.0, []
        if entry["scale"] < 1.0:
            why.append("diminta 'sedikit': setengah langkah")
        if previous:
            last = next((c for c in previous["changes"] if c["key"] == key), None)
            if last and (last["delta"] > 0) == (direction > 0):
                factor = min(MAX_GROWTH, last["factor"] * GROW)
                why.append("%s: langkah %s sebelumnya (%s) belum cukup, langkah dibesarkan x%.2f" % (
                    "MASIH" if entry["still"] else "lagi", previous["id"], _display(key, last["delta"]).strip(), factor))
            elif last:
                factor = last["factor"] * SHRINK
                why.append("arah berbalik dari langkah %s: setengah langkah sebelumnya" % previous["id"])
        before = base.get(key, _neutral(knob))
        target = before + direction * unit_step * factor
        limits_ = _bounds(state, key, base)
        if limits_ is None:
            rejected.append({"key": key, "why": "tidak ada batas aman untuk %s -- tidak diubah" % key,
                             "triggers": entry["triggers"]})
            continue
        lo, hi = limits_
        clipped = min(max(target, lo), hi)
        if _close(clipped, before):
            rejected.append({"key": key, "why": "sudah di batas %s (%s) -- tidak bisa lebih jauh ke arah itu" % (
                "atas" if direction > 0 else "bawah", _display(key, clipped).strip()), "triggers": entry["triggers"]})
            continue
        if not _close(clipped, target):
            why.append("dijepit ke batas %s %s" % ("atas" if direction > 0 else "bawah", _display(key, clipped).strip()))
        after[key] = round(clipped, 6)
        changes.append({"key": key, "before": before, "after": after[key], "delta": round(after[key] - before, 6),
                        "factor": round(factor, 3), "unit": STEP[knob][1], "reasons": entry["reasons"], "notes": why,
                        "triggers": entry["triggers"], "range": [lo, hi], "in_search": key in state["params"]})
    checks = _validate(state, base, after, changes, rejected)
    weights_after = _new_weights(weights_before, default_weights(state), factors)
    weight_changes = [{"metric": k, "factor": round(f, 3), "triggers": weight_triggers.get(k, [])}
                      for k, f in factors.items() if k in weights_before]
    motion = bool(changes)
    weighted = any(abs(weights_after[k] - weights_before[k]) > 1e-4 for k in weights_before)
    trial_count = len(state["trials"]) + (0 if any(t["n"] == source["n"] for t in state["trials"]) else 1)
    number = len(steps(state)) + 1
    return {"id": "if-%d" % number, "number": number, "created_at": feedback.now_iso(), "where": where,
            "status": "usulan" if (motion or weighted) else "tidak_berlaku", "kind": kind,
            "source": {"trial": source["n"], "trial_kind": source.get("kind"), "label": source.get("label"),
                       "configs": sorted(same_config(state, source))},
            "notes": [n["id"] for n in notes], "understanding": understood,
            "changes": changes, "rejected": rejected, "checks": checks,
            "weights_before": weights_before, "weights_after": weights_after, "weight_changes": weight_changes,
            "limits": _unique(limits), "needs": _unique(needs), "evidence": _unique(evidence),
            "base": base, "after": after, "phases": ph and {k: list(v) for k, v in ph["phases"].items()},
            "next_trial": trial_count + 1, "replaces": waiting["id"] if waiting else None,
            "trial": None, "verified": None, "result": None}


def _unique(pairs):
    out = {}
    for text, note in pairs:
        out.setdefault(text, [])
        if note not in out[text]:
            out[text].append(note)
    return [{"text": t, "notes": n} for t, n in out.items()]


def _validate(state, base, after, changes, rejected):
    """Pemeriksaan isi sesudah perubahan; perubahan yang membuatnya tidak sah dibuang."""
    checks = []
    if not changes:
        return checks
    if kind_of(state) == "studio":
        if wcore.leg_reachable(after):
            checks.append((True, "IK kaki sanggup di pose berdiri & ujung langkah (walking_ready_fk.reachable)"))
        else:
            pose = [c for c in changes if c["key"] in ("hip_pitch_offset", "init_pitch_offset", "init_x_offset",
                                                       "init_y_offset")]
            for change in pose:
                changes.remove(change)
                after[change["key"]] = change["before"]
                rejected.append({"key": change["key"], "why": "kaki tidak terjangkau IK dengan nilai ini",
                                 "triggers": change["triggers"]})
            checks.append((False, "IK kaki TIDAK sanggup -- perubahan pose dibuang"))
        checks.append((True, "satuan WalkingParam (m, rad, s) dan batas BOUNDS sesi (autotune_walking_core.py)"))
        return checks
    page = snapshot(state)
    clamped = acore.clamped_joints(page, after)
    if clamped:
        for change in list(changes):
            changes.remove(change)
            after[change["key"]] = change["before"]
            rejected.append({"key": change["key"], "why": "sendi terjepit di 0/4095: %s" % ", ".join(
                "step %d %s" % c for c in clamped[:4]), "triggers": change["triggers"]})
        checks.append((False, "page hasil membuat sendi keluar 0..4095 -- perubahan dibuang"))
        return checks
    tuned = acore.apply_knobs(page, after)
    encoded = src.decode_page(src.encode_page(tuned), 0)
    checks.append((encoded["checksum_ok"] and encoded["stepnum"] == page["stepnum"],
                   "page hasil sah: %d step, checksum benar, semua sendi di 0..4095" % encoded["stepnum"]))
    checks.append((True, "batas kenop autotune_action_core.BOUNDS dijaga untuk JUMLAH kenop global + fase"))
    return checks


# ---- menyimpan & menjalankan ------------------------------------------------------------

def apply(state, step, recost):
    """Simpan usulan di sesi. Perubahan gerak -> "menunggu" (dijalankan pelatih sebagai
    percobaan berikutnya); hanya bobot -> langsung dipakai penilaian (semua skor dihitung
    ulang). Langkah menunggu yang lain diganti."""
    for other in steps(state):
        if other["status"] == "menunggu":
            other["status"], other["replaced_by"] = "diganti", step["id"]
    state.setdefault("feedback_steps", []).append(step)
    step["applied_at"] = feedback.now_iso()
    if step["weights_after"] != current_weights(state):
        state["weights"] = dict(step["weights_after"])
        recost(state)
    if step["changes"]:
        step["status"] = "menunggu"
    elif step["status"] == "usulan":
        step["status"] = "dipakai"
        step["used"] = "bobot penilaian: semua skor sesi dihitung ulang dengan bobot baru"
    return step


def record(state, step, status):
    """Catat usulan yang TIDAK diterapkan (ditolak operator / tidak berlaku) -- tetap terlacak.
    Catatan yang sama dengan status yang sama tidak dicatat dua kali (menu dibuka ulang)."""
    step["status"] = status
    step["applied_at"] = None
    same = next((s for s in steps(state) if s["status"] == status and s["notes"] == step["notes"]
                 and s["source"]["trial"] == step["source"]["trial"]), None)
    if same is not None:
        return same
    state.setdefault("feedback_steps", []).append(step)
    return step


def finish(state, step, trial):
    """Percobaan jenis feedback selesai: bukti nilai yang dipakai + perbandingan."""
    step["trial"] = trial["n"]
    step["ran_at"] = trial.get("time")
    full = full_params(state, trial)
    mismatch = [k for k, v in step["after"].items() if not _close(full.get(k, _neutral(k)), v)]
    step["verified"] = {"same": not mismatch, "mismatch": mismatch, "loaded": trial.get("loaded")}
    step["status"] = "diuji"
    step["result"] = compare(state, step, trial)
    return step


def settle(state, step, keep, recost):
    """Sesudah operator melihat hasilnya: pertahankan (CMA lanjut dari titik ini) atau batalkan."""
    if keep:
        step["status"] = "dipakai"
        after = step["after"]
        lo, hi = np.array(state["lo"]), np.array(state["hi"])
        unit = np.clip((np.array([after[k] for k in state["params"]], dtype=float) - lo) / (hi - lo), 0.0, 1.0)
        dropped = len(state.get("pending") or [])
        state["cma"]["mean"] = unit.tolist()
        state["pending"], state["evaluated"] = [], []
        state["fixed"] = {k: v for k, v in after.items()
                          if k not in state["params"] and not _close(state["start_params"].get(k, _neutral(k)), v)}
        state["anchor"] = {"step": step["id"], "trial": step["trial"]}
        step["used"] = ("titik tengah CMA dipindah ke nilai percobaan #%d%s; %s" % (
            step["trial"], " (%d kandidat generasi lama yang belum jalan dibuang)" % dropped if dropped else "",
            "variabel tetap untuk semua percobaan berikutnya: %s" % ", ".join(
                "%s=%g" % kv for kv in state["fixed"].items()) if state["fixed"] else "tanpa variabel tetap"))
    else:
        step["status"] = "dibatalkan"
        if state.get("weights") != step["weights_before"] and step["weights_after"] == current_weights(state):
            state["weights"] = dict(step["weights_before"]) if step["weights_before"] != default_weights(state) else None
            recost(state)
        step["used"] = "dibatalkan operator sesudah diuji: CMA dan bobot kembali seperti sebelum langkah ini"
    step["settled_at"] = feedback.now_iso()
    return step


def anchored(state):
    """Operator sudah mengambil alih titik awal: ada langkah feedback yang dijalankan dan
    dipertahankan. Baseline (preset/page asli) yang terus jatuh tidak diulang lagi."""
    return any(s["status"] == "dipakai" and s.get("trial") for s in steps(state))


# ---- perbandingan -------------------------------------------------------------------------

def compare(state, step, trial):
    is_usable, _ = _trainer(state)
    source = next((t for t in state["trials"] if t["n"] == step["source"]["trial"]), None)
    weights = current_weights(state)
    out = {"source": step["source"]["trial"], "trial": trial["n"], "rows": []}
    if source is None:
        out["verdict"] = "tidak bisa dibandingkan: percobaan sumber #%s tidak tercatat" % step["source"]["trial"]
        return out
    src_cost, src_reruns = _cost(state, source)
    new_cost = trial["cost"]
    src_ok, new_ok = is_usable(source), is_usable(trial)
    status = lambda t, ok: ("sah" if ok else (t.get("label") or ("jatuh" if (t.get("metrics") or {}).get("fell")
                                                                 else "tidak sah")))
    out["rows"].append({"name": "status", "before": status(source, src_ok), "after": status(trial, new_ok),
                        "better": None if new_ok == src_ok else new_ok})
    same = abs(new_cost - src_cost) <= MEANINGFUL * max(src_cost, 1e-6)
    out["rows"].append({"name": "skor", "before": round(src_cost, 4), "after": round(new_cost, 4),
                        "better": None if same else new_cost < src_cost, "note": "rata-rata %d run" % (src_reruns + 1)
                        if src_reruns else ""})
    m0, m1 = source.get("metrics") or {}, trial.get("metrics") or {}
    keys = list(weights) + [k for k in ("lean_deg", "tilt_max_deg") if k in m1 and k not in weights]
    for key in keys:
        if m0.get(key) is None or m1.get(key) is None:
            continue
        before, after = m0[key], m1[key]
        if abs(after - before) <= max(1e-3, 0.005 * max(abs(after), abs(before))):
            better = None                        # sama (dalam pembulatan tampilan)
        elif key == "lean_deg" and any(c["key"] == "hip_pitch_offset" for c in step["changes"]):
            forward = next(c for c in step["changes"] if c["key"] == "hip_pitch_offset")["delta"] > 0
            better = (after > before) if forward else (after < before)
        else:
            better = abs(after) < abs(before)
        out["rows"].append({"name": key, "before": round(before, 3), "after": round(after, 3), "better": better})
    if src_ok != new_ok:
        verdict = "LEBIH BAIK" if new_ok else "LEBIH BURUK"
        why = "sekarang %s, sebelumnya %s" % (status(trial, new_ok), status(source, src_ok))
    elif not new_ok:
        verdict, why = "BELUM BERHASIL", "masih %s (sebelumnya %s)" % (status(trial, new_ok), status(source, src_ok))
    else:
        change = (new_cost - src_cost) / max(src_cost, 1e-6)
        verdict = "LEBIH BAIK" if change < -MEANINGFUL else "LEBIH BURUK" if change > MEANINGFUL else "SETARA"
        why = "skor %+.0f%% (%.3f -> %.3f; makin kecil makin baik)%s" % (
            100 * change, src_cost, new_cost, "" if verdict != "SETARA" else ", di bawah derau satu percobaan")
    out["verdict"], out["why"] = verdict, why
    out["cost_weights"] = dict(weights)
    return out


# ---- teks untuk operator -------------------------------------------------------------------

def _change_line(change):
    knob, steps_range = acore.split_key(change["key"])
    where = " (step %d-%d saja)" % steps_range if steps_range else ""
    return "%-24s %s -> %s  [%s%s]%s" % (
        change["key"], _display(change["key"], change["before"]).strip(), _display(change["key"], change["after"]).strip(),
        "+" if change["delta"] > 0 else "", _display(change["key"], change["delta"]).strip().lstrip("+"), where)


def target_text(state):
    if kind_of(state) == "studio":
        return "preset %r (Bascorro Studio)" % state["start_name"]
    return "%s (Action Editor)" % state["start_name"]


def step_text(state, step, session_name=""):
    """Bukti sebelum dijalankan: feedback, pemahaman, perubahan sebelum->sesudah + alasan,
    yang tidak bisa, target, dan identitas iterasi yang akan memakainya."""
    lines = ["Iterasi feedback %s -- sesi %s, dari percobaan #%s (%s%s)" % (
        step["id"], session_name, step["source"]["trial"], step["source"].get("trial_kind") or "-",
        ", label %s" % step["source"]["label"] if step["source"].get("label") else "")]
    if len(step["source"]["configs"]) > 1:
        lines.append("  konfigurasi sama dengan percobaan %s -- catatan di semuanya ikut dibaca" % ", ".join(
            "#%d" % n for n in step["source"]["configs"]))
    lines.append("\n  FEEDBACK DITERIMA (%d catatan):" % len(step["understanding"]))
    for item in step["understanding"]:
        lines.append("    %s (kandidat #%s):" % (item["note"], item["candidate"]))
        lines += ["    | " + line for line in item["text"].splitlines()]
        lines.append("    dipahami sebagai:")
        lines.append(feedback.understanding_text({"sentences": item["sentences"]}, indent="      "))
    lines.append("\n  PERUBAHAN YANG AKAN DIPAKAI:")
    if not step["changes"]:
        lines.append("    (tidak ada variabel gerakan yang berubah)")
    for change in step["changes"]:
        lines.append("    " + _change_line(change))
        for reason in change["reasons"]:
            lines.append("        alasan : %s" % reason)
        lines.append("        pemicu : %s" % "; ".join('"%s" (%s)' % ('", "'.join(t["phrases"]), t["note"])
                                                     for t in change["triggers"]))
        lines.append("        batas  : %s .. %s%s" % (_display(change["key"], change["range"][0]).strip(),
                                                    _display(change["key"], change["range"][1]).strip(),
                                                    "; dicari CMA (titik tengahnya ikut pindah)" if change["in_search"]
                                                    else "; di luar ruang CMA -> ditetapkan untuk percobaan berikutnya"))
        for note in change["notes"]:
            lines.append("        catatan: %s" % note)
    changed = [k for k in step["weights_before"] if abs(step["weights_after"][k] - step["weights_before"][k]) > 1e-4]
    if changed:
        lines.append("    bobot penilaian (pengali, dijaga %.1f..%.0fx bobot bawaan, lalu dinormalkan ke 100%%):"
                     % WEIGHT_RANGE)
        factors = {w["metric"]: w for w in step["weight_changes"]}
        for key in step["weights_before"]:
            mark = ("x%.2f karena %s" % (factors[key]["factor"], ", ".join(sorted({t["behavior"] for t in
                                                                                  factors[key]["triggers"]})))
                    if key in factors else "(ikut bergeser karena normalisasi)")
            lines.append("      %-18s %5.1f%% -> %5.1f%%  %s" % (key, 100 * step["weights_before"][key],
                                                                100 * step["weights_after"][key], mark))
    for item in step["evidence"]:
        lines.append("    data robot: %s" % item["text"])
    for check in step["checks"]:
        lines.append("    %s %s" % ("✓" if check[0] else "✗", check[1]))
    if step["rejected"] or step["limits"] or step["needs"]:
        lines.append("\n  TIDAK DIUBAH / PERLU INFORMASI:")
        for item in step["rejected"]:
            lines.append("    ✗ %s: %s" % (item["key"], item["why"]))
        for item in step["limits"]:
            lines.append("    ! %s (%s)" % (item["text"], ", ".join(item["notes"])))
        for item in step["needs"]:
            lines.append("    ? perlu: %s (%s)" % (item["text"], ", ".join(item["notes"])))
    if kind_of(state) == "action" and step["changes"]:
        lines.append("\n  NILAI SERVO PAGE YANG BERUBAH (raw, %s):" % "kandidat sumber -> sesudah feedback")
        lines += ["    " + line for line in servo_table(state, step["base"], step["after"])]
    lines.append("\n  TARGET : %s -- isi aslinya TIDAK diubah; perubahan berlaku di sesi training ini" % target_text(state))
    if step["changes"]:
        lines.append("  DIPAKAI: percobaan berikutnya sesi %s (#%d, jenis feedback), sebelum kandidat CMA lain;\n"
                     "           sesudahnya CMA melanjutkan dari titik ini kalau dipertahankan."
                     % (session_name, step["next_trial"]))
    elif step["status"] == "usulan":
        lines.append("  DIPAKAI: hanya penilaian -- semua skor sesi dihitung ulang dengan bobot baru, CMA berikutnya "
                     "memakainya. Tidak ada percobaan khusus (gerakan tidak berubah).")
    else:
        lines.append("  STATUS : feedback BELUM bisa diterapkan -- tidak ada perubahan yang sah dari catatan ini.")
    return "\n".join(lines)


def servo_table(state, base, after, limit=12):
    """Baris "step N sendi raw_lama -> raw_baru (selisih deg)" untuk page hasil."""
    page = snapshot(state)
    a, b = acore.apply_knobs(page, base), acore.apply_knobs(page, after)
    rows = []
    for number, (s0, s1) in enumerate(zip(a["steps"], b["steps"]), 1):
        for joint, joint_id in sorted(src.JOINT_IDS.items(), key=lambda kv: kv[1]):
            if s0["positions"][joint_id] != s1["positions"][joint_id]:
                rows.append("step %d %-11s %4d -> %4d  (%+.2f deg)" % (
                    number, joint, s0["positions"][joint_id], s1["positions"][joint_id],
                    (s1["positions"][joint_id] - s0["positions"][joint_id]) / acore.RAW_PER_DEG))
        if s0["time"] != s1["time"] or s0["pause"] != s1["pause"]:
            rows.append("step %d time %d -> %d, pause %d -> %d (x8 ms x speed/32)" % (
                number, s0["time"], s1["time"], s0["pause"], s1["pause"]))
    if len(rows) > limit:
        rows = rows[:limit] + ["... %d baris lagi" % (len(rows) - limit)]
    return rows or ["(tidak ada nilai servo yang berubah)"]


def result_text(state, step):
    """Hasil iterasi: bukti nilai dipakai + perbandingan dengan percobaan sumber."""
    lines = ["Hasil iterasi feedback %s: percobaan #%s dibanding #%s (sumber feedback)" % (
        step["id"], step.get("trial"), step["source"]["trial"])]
    verified = step.get("verified") or {}
    if verified:
        lines.append("  %s nilai yang dijalankan = nilai langkah ini (%s)%s" % (
            "✓" if verified["same"] else "✗", "; ".join(_change_line(c).split("  [")[0].strip()
                                                        for c in step["changes"]) or "bobot saja",
            "" if verified["same"] else " -- BEDA: %s" % ", ".join(verified["mismatch"])))
        if verified.get("loaded"):
            lines.append("  ✓ %s" % verified["loaded"])
    result = step.get("result")
    if not result:
        return "\n".join(lines + ["  (belum dijalankan)"])
    for row in result["rows"]:
        mark = "" if row.get("better") is None else ("✓" if row["better"] else "✗")
        before, after = row["before"], row["after"]
        lines.append("  %-16s %10s -> %-10s %s %s" % (
            row["name"], "%.3f" % before if isinstance(before, float) else before,
            "%.3f" % after if isinstance(after, float) else after, mark, row.get("note", "")))
    lines.append("  KESIMPULAN: %s -- %s" % (result.get("verdict"), result.get("why", "")))
    lines.append("  (satu percobaan; label operator tetap penentu. Ulangi lewat 'Uji hasil ini' untuk memastikan.)")
    return "\n".join(lines)


STATUS_TEXT = {"usulan": "usulan (belum disimpan)", "menunggu": "MENUNGGU dijalankan di percobaan berikutnya",
               "diuji": "sudah dijalankan, menunggu keputusan operator", "dipakai": "DITERAPKAN",
               "dibatalkan": "dijalankan lalu DIBATALKAN operator", "ditolak": "tidak diterapkan (catatan saja)",
               "diganti": "diganti usulan yang lebih baru", "tidak_berlaku": "tidak bisa diterapkan"}


def history_text(state, session_name=""):
    """Riwayat: catatan -> pemahaman -> perubahan -> percobaan -> hasil, per langkah."""
    if not steps(state):
        return "Belum ada iterasi feedback di sesi %s." % session_name
    lines = []
    for step in steps(state):
        lines.append("%s · %s · dari #%s · catatan %s" % (
            step["id"], STATUS_TEXT.get(step["status"], step["status"]), step["source"]["trial"],
            ", ".join(step["notes"]) or "-"))
        for change in step["changes"]:
            lines.append("    " + _change_line(change))
        changed = [k for k in step["weights_before"]
                   if abs(step["weights_after"][k] - step["weights_before"][k]) > 1e-4]
        if changed:
            lines.append("    bobot: %s" % ", ".join("%s %.0f%%->%.0f%%" % (
                k, 100 * step["weights_before"][k], 100 * step["weights_after"][k]) for k in changed))
        for item in step["needs"]:
            lines.append("    ? perlu: %s" % item["text"])
        if step.get("trial"):
            result = step.get("result") or {}
            lines.append("    -> percobaan #%s: %s %s" % (step["trial"], result.get("verdict", "-"), result.get("why", "")))
        if step.get("used"):
            lines.append("    -> %s" % step["used"])
    return "\n".join(lines)

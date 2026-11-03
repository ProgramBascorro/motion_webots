#!/usr/bin/env python3
"""Training page Action Editor (tendangan, bangun, pose, gerakan lain) di robot asli.

Pasangan scripts/autotune_walking.py (preset Bascorro Studio). Cara mencarinya
sama -- CMA-ES dengan baseline, label operator, sesi yang bisa dilanjutkan --
tapi yang diubah adalah KENOP yang dikenakan ke seluruh step page (pengali
waktu dan condong pinggul/engkel; lihat autotune_action_core.py), dan yang
dinilai sesuai jenis gerakannya (--profile, bawaan ditebak dari nama
Motion_Index/page):

  kick   tendangan  : badan paling tenang & kembali tegak; kecepatan DIKUNCI
  getup  bangun     : WAJIB berhasil tegak; lalu paling cepat & halus
  pose   pose diam  : paling sedikit goyang saat pindah ke pose & menahannya
  umum   lainnya    : badan paling tenang & kembali tegak

Isi page dibaca dari .bin yang SEDANG dipakai action_module (env op3_manager)
saat sesi dibuat. Kandidat diputar lewat page kerja 254 (page aslinya TIDAK
pernah ditulis selama training); isi 254 dikembalikan persis saat alat selesai.
Kalau page aslinya diubah di Action Editor di tengah sesi, training berhenti:
lanjutkan dengan --follow-page (kenop berlaku ke isi page yang baru).

KESELAMATAN -- WAJIB
--------------------
* Tendangan/pose/umum: robot DIGANTUNG tali kendur, mulai dari page 2. Ada
  orang di dekatnya.
* Bangun (getup): robot di MATRAS, berbaring (tengkurap/telentang sesuai page),
  tali kendur atau dipegang longgar. Percobaan hanya mulai kalau IMU membaca
  robot memang berbaring.
* Tiap percobaan menunggu Enter. Ctrl+C = page dihentikan (brake), sesi disimpan.
* Jatuh (roll/pitch tertapis > 45 deg) = page dihentikan seketika.
* Jangan menyimpan page di Action Editor/Studio selama training.

PAKAI (di DALAM kontainer, op3_manager hidup, terminal interaktif -it)
-----
  python3 /ros2_ws/scripts/autotune_action.py --session tk1 --page 105
  ...atau lewat menu: ./script.sh -> Training -> Train dari Action Editor

  --session NAMA      sesi baru, atau dilanjutkan kalau sudah ada
  --page N            page yang dilatih (sesi baru)
  --profile P         kick | getup | pose | umum (bawaan: ditebak)
  --trials N          anggaran TOTAL percobaan CMA (bawaan 32); sesi lama: menaikkannya
  --report            tabel semua percobaan
  --rerun-best N      ulangi kandidat terbaik N kali
  --trial N           dengan --rerun-best: ulangi percobaan N (menu: "Uji hasil ini di robot").
                      Boleh walau page aslinya sudah diubah: yang diputar tetap isi page saat
                      sesi dibuat + kenop kandidat itu, persis hasil yang disimpan/diekspor
  --label "2=h,4=j"   penilaian operator (n normal/berhasil, j jatuh/gagal, h hampir/dibantu)
  --rescore           hitung ulang metrik dari rekaman mentah
  --follow-page       lanjutkan sesi walau page aslinya sudah diubah
  --warm-start SESI[:N]  sesi BARU mulai dari kenop terbaik (atau percobaan N) sesi lain, isi page sama
  --weights k=v,...   sesi BARU: bobot skor lain (kunci = metrik profil, dinormalkan ke 1)
  --feedback-run      jalankan iterasi feedback yang MENUNGGU (training.py feedback-apply), lalu berhenti

Sesudah tiap percobaan: label n/j/h, atau "c" = tulis catatan bebas (beberapa baris).
Catatan itu langsung dipahami dan diterjemahkan ke kenop (training_iteration.py),
mis. "kaki kiri mentok saat menendang" pada tendangan kiri = badan 0,5 deg lebih ke
kanan HANYA di step bertumpu satu kaki (roll_ankle_deg@2-5). Kalau disetujui,
percobaan BERIKUTNYA memakainya (jenis "feedback"), lalu dibandingkan.

Hasil: src/bascorro_studio/data/autotune/<sesi>/ (sama dengan training walking).
Simpan / kirim ke page .bin / YAML action_yaml: scripts/training.py save|deploy (atau menu Riwayat hasil).
"""

import argparse
import datetime
import gzip
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_action_core as core  # noqa: E402
import autotune_walking as walk  # noqa: E402
import autotune_walking_core as wcore  # noqa: E402
import training_iteration as iteration  # noqa: E402
import training_sources as src  # noqa: E402

STATE_FORMAT = "bascorro-studio/action-autotune"
DEFAULT_BUDGET = 32
SCRATCH_NAME = "train_tmp"


class ActionSession(walk.Session):
    FORMAT = STATE_FORMAT

    def write_csv(self, writer):
        state = self.state
        names = state["params"] + iteration.extra_keys(state)
        keys = list(core.PROFILES[state["profile"]]["weights"])
        writer.writerow(["n", "jenis", "generasi", "skor", "gagal", "label", "durasi_s", "volt"] + names + keys)
        check = usable_for(state)
        for t in state["trials"]:
            m = t.get("metrics") or {}
            full = iteration.full_params(state, t)
            writer.writerow([t["n"], t["kind"], t.get("generation", ""), "%.4f" % t["cost"], int(not check(t)),
                             t.get("label", ""), "" if m.get("duration_s") is None else "%.2f" % m["duration_s"],
                             t.get("voltage") or ""]
                            + [full.get(k, iteration._neutral(k)) for k in names]
                            + ["" if m.get(k) is None else "%.3f" % m[k] for k in keys])


# ---- skor ---------------------------------------------------------------------

def weights(state):
    """Bobot skor sesi: rencana dari feedback (state "weights") atau bawaan profil."""
    return state.get("weights") or core.PROFILES[state["profile"]]["weights"]


def usable_for(state):
    w, profile = weights(state), state["profile"]

    def check(trial):
        m = trial.get("metrics") or {}
        return bool(m) and not core.failed(m, trial.get("label"), profile) and all(m.get(k) is not None for k in w)
    return check


def score(state, trial):
    w = weights(state)
    base = walk.baseline_trials(state, usable_for(state))
    # tanpa baseline sah (page asli selalu gagal): skala minimum, percobaan berhasil tetap < skor gagal
    scales = core.scales_from_baseline([t["metrics"] for t in base], w) if base else {k: core.FLOORS[k] for k in w}
    return core.cost(trial["metrics"], scales, w, state["profile"], trial.get("label"))


def recost_all(state):
    for trial in state["trials"]:
        trial["cost"] = score(state, trial)


def best_trial(state):
    return walk.best_trial(state, usable_for(state))


# ---- page -----------------------------------------------------------------------

def snapshot_page(state):
    """Isi page saat sesi dibuat (atau --follow-page terakhir) -- dasar semua kandidat."""
    page = src.decode_page(bytes.fromhex(state["page_hex"]), 0)
    page["index"] = state["page"]
    return page


def candidate_page(state, knobs):
    page = core.apply_knobs(snapshot_page(state), dict(core.NEUTRAL, **knobs))
    if page["next"] == state["page"]:      # page yang mengulang dirinya: ulangi kandidatnya
        page["next"] = src.TRAINING_SCRATCH_PAGE
    return src.encode_page(page, name=SCRATCH_NAME)


def page_state(data, index):
    page = src.decode_page(data, index)
    chain, _ = src.page_chain(data, index)
    return {"page_hex": page["raw"].hex(), "page_sha": src.page_sha(page), "page_name": page["name"],
            "chain": chain, "planned_s": round(src.chain_seconds(data, chain), 3)}


def bin_rel(path):
    return os.path.relpath(path, walk.HERE) if path.startswith(walk.HERE) else path


def new_state(args):
    bin_path, _ = src.active_action_bin()
    data = src.read_bin(bin_path)
    errors, warnings = src.check_page(data, args.page)
    if errors:
        raise SystemExit("page %d tidak bisa dilatih: %s" % (args.page, "; ".join(errors)))
    for warning in warnings:
        print("  catatan: %s" % warning)
    names = src.motion_index()
    page = src.decode_page(data, args.page)
    guessed, reason = src.guess_profile(args.page, page, names)
    profile = args.profile or guessed
    params = args.params.split(",") if args.params else list(core.PROFILES[profile]["params"])
    start = dict(core.NEUTRAL)
    space = wcore.Space(params, start, {}, bounds=core.BOUNDS)
    x0, warm = space.to_unit(start), None
    if args.warm_start:
        x0, warm = warm_start_point(args.warm_start, space, args.page, src.page_sha(page))
    es = wcore.CMAES(x0, args.sigma, popsize=args.popsize, seed=args.seed)
    label = " (%s)" % names[args.page] if args.page in names else ""
    state = {
        "format": STATE_FORMAT, "formatVersion": 1, "metricsVersion": core.METRICS_VERSION,
        "createdAt": walk.now_iso(), "source": "action_editor",
        "page": args.page, "motion_name": names.get(args.page, ""), "bin_path": bin_rel(bin_path),
        "start_name": "page %d %s%s" % (args.page, page["name"], label),
        "profile": profile, "profile_reason": reason if not args.profile else "dipilih operator",
        "params": params, "lo": space.lo.tolist(), "hi": space.hi.tolist(), "start_params": start,
        "sigma0": args.sigma, "popsize": es.lam,
        "budget": args.trials if args.trials is not None else DEFAULT_BUDGET,
        "baseline_runs": args.baseline_runs, "cma": es.state(), "pending": [], "evaluated": [],
        "trials": [], "page_history": [], "warm_start": warm,
        "weights": walk.parse_weights(args.weights, core.PROFILES[profile]["weights"]) if args.weights else None,
        "feedback_steps": [], "fixed": {},
    }
    state.update(page_state(data, args.page))
    return state


def warm_start_point(spec, space, page, page_sha):
    """Titik awal CMA = kenop kandidat terbaik (atau percobaan N) sesi lain untuk ISI page yang sama."""
    name, _, which = spec.partition(":")
    previous = ActionSession(name)
    if not previous.exists():
        raise SystemExit("sesi --warm-start %r tidak ada" % name)
    state = previous.load()
    if state["page"] != page or state["page_sha"] != page_sha:
        raise SystemExit("--warm-start %s: sesi itu melatih page %d isi %s, sekarang page %d isi %s -- kenopnya "
                         "tidak berlaku untuk isi yang lain" % (name, state["page"], state["page_sha"], page, page_sha))
    candidate = (next((t for t in state["trials"] if str(t["n"]) == which), None) if which
                 else best_trial(state))
    if candidate is None:
        raise SystemExit("--warm-start %s: %s" % (name, "percobaan %s tidak ada" % which if which
                                                   else "belum ada kandidat yang sah"))
    cost, _ = walk.candidate_cost(state, candidate)
    print("  --warm-start: mulai dari sesi %s percobaan %d (skor %.3f)" % (name, candidate["n"], cost))
    unit = wcore.np.clip(space.to_unit(dict(core.NEUTRAL, **candidate["params"])), 0.0, 1.0)
    return unit, {"session": name, "n": candidate["n"], "cost": cost, "chosen": bool(which)}


def page_changed(state):
    """None kalau page di .bin aktif masih sama dengan snapshot sesi, selain itu alasannya."""
    bin_path, _ = src.active_action_bin()
    if bin_rel(bin_path) != state["bin_path"]:
        return "action_module sekarang memakai %s, sesi dibuat dari %s" % (bin_rel(bin_path), state["bin_path"])
    now = src.page_sha(src.decode_page(src.read_bin(bin_path), state["page"]))
    if now != state["page_sha"]:
        return "page %d di %s sudah diubah sejak sesi dimulai (%s -> %s)" % (
            state["page"], state["bin_path"], state["page_sha"], now)
    return None


def follow_page(state):
    bin_path, _ = src.active_action_bin()
    data = src.read_bin(bin_path)
    errors, _ = src.check_page(data, state["page"])
    if errors:
        raise SystemExit("page %d sekarang tidak bisa dilatih: %s" % (state["page"], "; ".join(errors)))
    state["page_history"].append({"from": state["page_sha"], "at": walk.now_iso(), "bin_path": state["bin_path"]})
    state.update(page_state(data, state["page"]))
    state["bin_path"] = bin_rel(bin_path)
    print("sesi mengikuti isi page %d yang baru (%s); kenop berlaku ke isi baru itu" % (state["page"], state["page_sha"]))


# ---- ROS ----------------------------------------------------------------------

class NotLying(Exception):
    """Profil bangun tapi robot tidak sedang berbaring."""


def empty_page(data):
    """Page kosong persis seperti buatan Action Editor (diambil dari .bin itu sendiri)."""
    for index in range(src.MAX_PAGE - 1, 0, -1):
        page = src.decode_page(data, index)
        if not page["name"] and page["stepnum"] == 0 and page["checksum_ok"]:
            return page["raw"]
    raise SystemExit("tidak ada page kosong di .bin sebagai contoh")


class Scratch:
    """Page kerja 254: diisi kandidat sebelum diputar, dikembalikan persis saat selesai."""

    def __init__(self, bin_path):
        self.path = bin_path
        self.original = src.read_bin(bin_path)[src.TRAINING_SCRATCH_PAGE * src.PAGE_SIZE:
                                               (src.TRAINING_SCRATCH_PAGE + 1) * src.PAGE_SIZE]
        name = self.original[:14].split(b"\0")[0].decode("ascii", "replace").strip()
        if name == SCRATCH_NAME:
            # sisa training yang terputus (listrik/kill -9): kembalikan ke page kosong sungguhan
            self.original = empty_page(src.read_bin(bin_path))
        elif self.original[20]:
            raise SystemExit("page %d berisi gerakan %r -- bukan page kosong, training tidak mau menimpanya"
                             % (src.TRAINING_SCRATCH_PAGE, name))

    def load(self, page_bytes):
        src.write_page(self.path, src.TRAINING_SCRATCH_PAGE, page_bytes)

    def restore(self):
        src.write_page(self.path, src.TRAINING_SCRATCH_PAGE, self.original)


def wait_page(robot, marks, timeout, watch_fall):
    """Tunggu action_module selesai memutar (termasuk page lanjutan). Jatuh/timeout = brake."""
    started = False
    while True:
        robot.spin_for(0.05)
        now = time.monotonic()
        if watch_fall and robot.fell_at is not None:
            robot.page_pub.publish(robot.Int32(data=-2))
            break
        running = robot.action_running()
        started = started or running
        if not running and (started or now - marks["start"] > 1.0):
            break
        if now - marks["start"] > timeout:
            robot.page_pub.publish(robot.Int32(data=-2))
            marks["timed_out"] = True
            break
    if not started and robot.fell_at is None:
        raise RuntimeError("action_module tidak memutar page %d (lihat /robotis/status: 'Failed to start page'?)"
                           % src.TRAINING_SCRATCH_PAGE)


def run_trial(robot, state, scratch, knobs):
    lying = core.PROFILES[state["profile"]]["lying"]
    robot.clear_speed_limit()
    if lying:
        robot.take_action_module()
    else:
        robot.play_page2()
    scratch.load(candidate_page(state, knobs))
    robot.begin_recording()
    marks = {"stand0": time.monotonic(), "planned_s": state["planned_s"]}
    robot.spin_for(1.0)
    if robot.tilt.roll is None:
        raise RuntimeError("IMU tidak terbit (/robotis/open_cr/imu)")
    tilt = max(abs(robot.tilt.roll), abs(robot.tilt.pitch))
    if lying and tilt < core.LYING_DEG:
        raise NotLying("robot belum berbaring (miring %.0f deg < %.0f deg)" % (tilt, core.LYING_DEG))
    robot.fall = None if lying else wcore.FallWatch(seed=(robot.tilt.t, robot.tilt.roll, robot.tilt.pitch))
    marks["start"] = time.monotonic()
    robot.page_pub.publish(robot.Int32(data=src.TRAINING_SCRATCH_PAGE))
    wait_page(robot, marks, 1.5 * state["planned_s"] + 5.0, not lying)
    marks["done"] = time.monotonic()
    marks["fell_at"] = robot.fell_at
    robot.spin_for(core.SETTLE_S)
    marks["end"] = time.monotonic()
    robot.recording = False
    return robot.imu, marks


LABEL_PROMPT = {
    "getup": "  Menurut kamu? Enter = %s (deteksi), n = berhasil berdiri, j = gagal berdiri, "
             "h = berdiri tapi dibantu/dipegang, ",
    None: "  Menurut kamu? Enter = %s (deteksi), n = normal, j = jatuh/tertahan tali, "
          "h = hampir jatuh, dipegang/diangkat, ",
}


def evaluate(robot, session, scratch, kind, knobs, args, generation=None, candidate=None, step=None):
    state = session.state
    n = len(state["trials"]) + 1
    print("\n=== percobaan %d (%s%s) ===\n  %s" % (n, kind, "" if generation is None else " gen %d" % generation,
                                                 ", ".join("%s=%g" % kv for kv in iteration.recorded(state, knobs).items())))
    # Ulangan memutar isi page SESI + kenop (bukan page di .bin), jadi tetap hasil yang sama;
    # baseline/kandidat baru tidak boleh dibandingkan dengan page yang sudah lain.
    reason = page_changed(state) if kind != "rerun" else None
    if reason:
        raise SystemExit("%s.\nBerhenti supaya kandidat tidak dibandingkan dengan page yang lain. Lanjutkan dengan "
                         "--session %s --follow-page" % (reason, session.name))
    if robot.voltage is not None and robot.voltage < args.min_voltage:
        walk.ask("  !! baterai %.1f V < %.1f V. Ganti baterai, lalu Enter. " % (robot.voltage, args.min_voltage))
    where = ("Robot BERBARING di matras" if core.PROFILES[state["profile"]]["lying"]
             else "Robot digantung, siap dari page 2")
    answer = walk.ask("  %s? Enter = mulai, s = lewati, q = selesai: " % where, {"", "s", "q"})
    if answer == "q":
        raise KeyboardInterrupt
    if answer == "s":
        return None
    try:
        imu, marks = run_trial(robot, state, scratch, knobs)
    except NotLying as error:
        robot.recording = False
        print("  %s -- baringkan dulu, percobaan diulang." % error)
        return None
    except RuntimeError as error:
        robot.recording = False
        robot.page_pub.publish(robot.Int32(data=-2))
        print("  GAGAL SISTEM (bukan salah kandidat, tidak dihitung): %s" % error)
        return None
    metrics = core.compute_metrics(imu, marks, state["profile"])
    if not metrics.get("valid"):
        print("  rekaman tidak sah (IMU terlalu sedikit) -- tidak dihitung.")
        return None
    getup = state["profile"] == "getup"
    detected_fail = core.failed(metrics, None, state["profile"])
    detected = ("gagal" if getup else "jatuh") if detected_fail else ("berhasil" if getup else "normal")
    answer = walk.ask_label(session, n, candidate, LABEL_PROMPT["getup" if getup else None] % detected,
                            {"kind": kind, "params": knobs, "metrics": metrics, "step": step})
    label = walk.LABEL_KEYS.get(answer, "jatuh" if detected_fail else "normal")
    session.save_raw(n, imu, [], marks)
    played = src.page_sha({"raw": candidate_page(state, knobs)})
    trial = {"n": n, "kind": kind, "generation": generation, "time": walk.now_iso(), "voltage": robot.voltage,
             "params": iteration.recorded(state, knobs), "metrics": metrics, "label": label,
             "page_sha": state["page_sha"], "played_sha": played}
    if kind == "feedback":
        trial["loaded"] = ("page kerja %d ditulis lalu DIBACA BALIK sama dari %s (isi %s) sebelum diputar "
                           "action_module" % (src.TRAINING_SCRATCH_PAGE, state["bin_path"], played))
    return trial


def print_result(state, trial):
    m = trial["metrics"]
    if trial.get("label") == "hampir":
        print("  HAMPIR JATUH / DIBANTU (label operator) -> skor %.2f" % trial["cost"])
    elif not usable_for(state)(trial):
        what = "GAGAL BERDIRI" if state["profile"] == "getup" else "JATUH"
        print("  %s -> skor %.2f" % (what, trial["cost"]))
    else:
        print("  skor %.3f | %s" % (trial["cost"], " | ".join(
            "%s %.2f" % (k, m[k]) for k in weights(state))))
    if m.get("timed_out"):
        print("  (page tidak selesai dalam batas waktu -- dihentikan)")


def tune(robot, session, scratch, args):
    state = session.state

    def run(kind, knobs, generation=None):
        trial = evaluate(robot, session, scratch, kind, knobs, args, generation)
        if trial is not None and kind != "baseline":
            trial["cost"] = score(state, trial)
            print_result(state, trial)
        return trial

    steps = feedback_runner(robot, session, scratch, args)
    walk.baseline_loop(session, lambda: run("baseline", state["start_params"]), recost_all, usable_for(state),
                       lambda t: print_result(state, t),
                       "page asli sendiri gagal/jatuh -- tidak dipakai sebagai baseline. Periksa gantungan, "
                       "baterai, dan page-nya dulu -- atau tulis 'c' (mis. 'kaki kiri mentok saat menendang') "
                       "supaya percobaan berikutnya memakai kenop yang diperbaiki.", steps)
    space = wcore.Space(state["params"], state["start_params"], {}, bounds=core.BOUNDS)
    space.lo, space.hi = wcore.np.array(state["lo"]), wcore.np.array(state["hi"])
    walk.cma_loop(robot, session, args, space, lambda params, gen: run("cma", params, gen), best_trial, steps)
    report(state)


def feedback_runner(robot, session, scratch, args):
    """feedback_steps() untuk loop pelatih page."""
    state = session.state

    def run(knobs, step_id):
        trial = evaluate(robot, session, scratch, "feedback", knobs, args, step=step_id)
        if trial is not None:
            trial["cost"] = score(state, trial)
            print_result(state, trial)
        return trial
    return lambda: walk.run_feedback_steps(session, run, recost_all)


def rerun_best(robot, session, scratch, args):
    state = session.state
    best = walk.rerun_target(state, args.trial, best_trial)
    print("mengulang kandidat percobaan %d (%s) %d kali" % (best["n"], best["kind"], args.rerun_best))
    steps, waiting = feedback_runner(robot, session, scratch, args), iteration.pending(state)
    for _ in range(args.rerun_best):
        trial = evaluate(robot, session, scratch, "rerun", dict(state["start_params"], **best["params"]), args,
                         candidate=best["n"])
        if trial is None:
            continue
        trial["cost"] = score(state, trial)
        trial["rerun_of"] = best["n"]
        state["trials"].append(trial)
        session.save()
        print_result(state, trial)
        if iteration.pending(state) not in (None, waiting):
            steps()            # catatan "c" di ulangan ini: perubahannya langsung dicoba


# ---- laporan / rescore --------------------------------------------------------

def report(state):
    names, keys = state["params"] + iteration.extra_keys(state), list(weights(state))
    check = usable_for(state)
    print("sesi mulai %s, %s, profil %s (%s), isi page %s, %d percobaan"
          % (state["createdAt"], state["start_name"], state["profile"], core.PROFILES[state["profile"]]["label"],
             state["page_sha"], len(state["trials"])))
    print("kriteria: %s" % core.criteria_text(state["profile"], state.get("weights")))
    for line in walk.plan_lines(state, core.METRIC_TEXT):
        print(line)
    print("%4s %-8s %4s %7s %6s %s | %s" % ("n", "jenis", "gen", "skor", "gagal", "  ".join(keys), "  ".join(names)))
    for t in state["trials"]:
        m = t.get("metrics") or {}
        gen = "" if t.get("generation") is None else t["generation"]
        flag = "" if check(t) else ("hampir" if t.get("label") == "hampir" else "YA")
        print("%4d %-8s %4s %7.3f %6s %s | %s" % (
            t["n"], t["kind"], gen, t["cost"], flag,
            "  ".join("-" if m.get(k) is None else "%.2f" % m[k] for k in keys),
            "  ".join("%g" % iteration.full_params(state, t).get(k, iteration._neutral(k)) for k in names)))
    best = best_trial(state)
    if best:
        mean, reruns = walk.candidate_cost(state, best)
        base = [t["cost"] for t in walk.baseline_trials(state, check)]
        print("\nterbaik: percobaan %d, skor %.3f%s | baseline page asli: %s" % (
            best["n"], mean, " (rata-rata dengan %d ulangan)" % reruns if reruns else " (belum diulang)",
            " ".join("%.3f" % c for c in base) or "-"))
        full = iteration.full_params(state, best)
        for k in names:
            print("  %-18s %8g  (asli %g)" % (k, full.get(k, iteration._neutral(k)), iteration._neutral(k)))
    else:
        print("\nbelum ada percobaan yang sah")


def rescore(session):
    state = session.state
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    with open(os.path.join(session.dir, "state.sebelum-rescore-%s.json" % stamp), "w", encoding="utf-8") as handle:
        json.dump(state, handle, indent=1)
    for trial in state["trials"]:
        path = os.path.join(session.dir, "raw", "trial_%03d.json.gz" % trial["n"])
        if trial.get("metrics") is None or not os.path.exists(path):
            continue
        with gzip.open(path, "rt", encoding="utf-8") as handle:
            raw = json.load(handle)
        trial["metrics"] = core.compute_metrics(raw["imu"], raw["marks"], state["profile"])
    state["metricsVersion"] = core.METRICS_VERSION
    recost_all(state)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", required=True)
    ap.add_argument("--page", type=int, help="page yang dilatih (sesi baru)")
    ap.add_argument("--profile", choices=sorted(core.PROFILES), help="jenis gerakan (bawaan: ditebak dari nama)")
    ap.add_argument("--params", help="kenop dipisah koma (bawaan: menurut profil): %s" % ",".join(core.BOUNDS))
    ap.add_argument("--trials", type=int, help="anggaran TOTAL percobaan CMA (bawaan %d)" % DEFAULT_BUDGET)
    ap.add_argument("--popsize", type=int)
    ap.add_argument("--sigma", type=float, default=0.2, help="langkah awal, pecahan rentang tiap kenop")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--baseline-runs", type=int, default=2)
    ap.add_argument("--cooldown-sec", type=float, default=15.0)
    ap.add_argument("--min-voltage", type=float, default=11.1)
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--rerun-best", type=int, default=0, metavar="N")
    ap.add_argument("--trial", type=int, metavar="N", help="dengan --rerun-best: ulangi percobaan N, bukan terbaik")
    ap.add_argument("--label", metavar="N=n|j|h,...")
    ap.add_argument("--rescore", action="store_true")
    ap.add_argument("--follow-page", action="store_true")
    ap.add_argument("--warm-start", metavar="SESI[:N]", help="sesi BARU mulai dari kenop sesi lain (isi page sama)")
    ap.add_argument("--weights", metavar="k=v,...", help="sesi BARU: bobot skor (metrik profil)")
    ap.add_argument("--feedback-run", action="store_true",
                    help="jalankan iterasi feedback yang menunggu (training.py feedback-apply), lalu berhenti")
    args = ap.parse_args()
    if args.trial is not None and not args.rerun_best:
        ap.error("--trial hanya bersama --rerun-best N")

    session = ActionSession(args.session)
    if session.exists() and (args.weights or args.warm_start):
        ap.error("--weights/--warm-start hanya untuk sesi BARU (sesi %r sudah ada)" % args.session)
    if session.exists():
        state = session.load()
        if args.page is not None and args.page != state["page"]:
            raise SystemExit("sesi %r melatih page %d, bukan %d -- pakai nama sesi lain" % (args.session, state["page"], args.page))
        if args.label or args.rescore:
            if args.rescore:
                rescore(session)
            if args.label:
                walk.apply_labels(state, args.label, recost_all)
            session.save()
            report(state)
            return 0
        if args.report:
            report(state)
            return 0
        if state.get("metricsVersion") != core.METRICS_VERSION:
            raise SystemExit("sesi %r memakai rumus skor versi %s -- jalankan dulu --session %s --rescore"
                             % (args.session, state.get("metricsVersion"), args.session))
        reason = page_changed(state)
        if reason and args.follow_page:
            follow_page(state)
            session.save()
        elif reason and args.rerun_best:
            print("catatan: %s.\n  Yang diulang tetap isi page saat sesi dibuat (%s) + kenop kandidatnya, lewat page "
                  "kerja %d -- persis hasil sesi ini." % (reason, state["page_sha"], src.TRAINING_SCRATCH_PAGE))
        elif reason:
            raise SystemExit("%s. Mulai sesi BARU, atau lanjutkan dengan --follow-page" % reason)
        print("melanjutkan sesi %r: %d percobaan tercatat" % (args.session, len(state["trials"])))
        if args.trials is not None and args.trials != state["budget"]:
            print("anggaran percobaan CMA %d -> %d" % (state["budget"], args.trials))
            state["budget"] = args.trials
            session.save()
    elif args.report or args.label or args.rescore or args.rerun_best or args.feedback_run:
        raise SystemExit("sesi %r belum ada" % args.session)
    elif args.page is None:
        raise SystemExit("sesi baru butuh --page N")
    else:
        if walk.Session(args.session).exists() or os.path.isdir(session.dir):
            raise SystemExit("nama sesi %r sudah dipakai -- pilih nama lain" % args.session)
        session.state = new_state(args)
        session.save()
        print("sesi baru %r: %s, profil %s, kenop %s" % (args.session, session.state["start_name"],
                                                         session.state["profile"], ", ".join(session.state["params"])))

    state = session.state
    if args.feedback_run and iteration.pending(state) is None:
        raise SystemExit("sesi %r tidak punya iterasi feedback yang menunggu (scripts/training.py feedback-apply)"
                         % args.session)
    done = sum(1 for t in state["trials"] if t["kind"] == "cma")
    if not (args.rerun_best or args.feedback_run) and done >= state["budget"]:
        raise SystemExit("anggaran sesi %r sudah habis (%d/%d percobaan CMA). Lanjutkan dengan anggaran lebih besar, "
                         "mis. --session %s --trials %d" % (args.session, done, state["budget"], args.session,
                                                            done + 2 * state["popsize"]))
    print(__doc__.split("KESELAMATAN -- WAJIB")[1].split("PAKAI")[0])
    print("kriteria %s: %s\n" % (state["profile"], core.criteria_text(state["profile"], state.get("weights"))))
    # Iterasi feedback dari menu: perubahan baru disetujui di menu, tiap percobaan tetap menunggu Enter.
    if not args.feedback_run and walk.ask("Ketik YA kalau robot sudah siap sesuai di atas dan D60/Studio tidak "
                                          "dipakai: ").upper() != "YA":
        return 1
    bin_path = os.path.join(walk.HERE, state["bin_path"]) if not os.path.isabs(state["bin_path"]) else state["bin_path"]
    scratch = Scratch(bin_path)
    session.lock()
    robot = None
    try:
        robot = walk.Robot("action_autotune")
        robot.spin_for(1.5)  # penemuan DDS
        if args.feedback_run:
            feedback_runner(robot, session, scratch, args)()
        elif args.rerun_best:
            rerun_best(robot, session, scratch, args)
        else:
            tune(robot, session, scratch, args)
    except KeyboardInterrupt:
        if robot is not None:
            robot.page_pub.publish(robot.Int32(data=-2))
            robot.spin_for(0.2)
        print("\ndihentikan -- page dihentikan, sesi tersimpan. Lanjutkan dengan --session %s." % args.session)
    finally:
        session.save()
        session.unlock()
        scratch.restore()
        print("page kerja %d dikembalikan seperti semula" % src.TRAINING_SCRATCH_PAGE)
        if robot is not None:
            robot.node.destroy_node()
            robot.rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Perintah di balik menu Training (./script.sh -> Training). Bisa juga dipakai
langsung; semuanya membaca data TERBARU setiap kali dipanggil.

Sumber & sesi:
  studio-list                      preset Bascorro Studio (urutan dropdown Studio)
  studio-info ID|NAMA              rincian preset + yang akan dilatih   (kode 3 = tidak bisa dilatih)
  action-list                      page .bin yang berisi gerakan (Motion_Index dulu)
  action-info PAGE                 rincian + validasi page            (kode 3 = tidak bisa dilatih)
  action-params PAGE | --session S [--trial N]
                                   APA yang dioptimasi training page: kenop, satuan, batas, sendi & raw
                                   per step, fase page, setelan algoritma, bobot penilaian
  sessions studio|action TARGET    sesi training milik preset/page itu
  session-flags SESI               KEY=VALUE untuk menu (jenis, progres, terbaik, perlu rescore/follow)
  free-page                        page kosong pertama yang disarankan
  suggest-name studio|action TARGET  nama sesi baru yang belum dipakai

Riwayat hasil (satu hasil = satu kandidat sesi, N = nomor percobaan atau "best"):
  results / results-list           semua sesi: sumber, target, waktu, status, terbaik + alasannya
  result-info SESI                 rincian satu sesi + hasil tersimpan + jumlah feedback
  trial-list SESI                  kandidat yang bisa dipilih (terbaik dulu, lalu yang tersimpan/ber-feedback)
  trial-info SESI N                semua tentang satu hasil: metrik, terbaik?, berkas, pengiriman, feedback
  feedback-add SESI N              tulis feedback (teks bebas, beberapa baris) -- interaktif
  feedback-list SESI [N]           baca feedback utuh
  feedback-plan SESI N             feedback -> pemahaman -> perubahan (sebelum/sesudah, alasan, batas).
                                   Kode 0 = ada perubahan gerak, 8 = hanya bobot, 7 = belum bisa diterapkan
  feedback-apply SESI N [--decline]  simpan perubahan itu di sesi: gerak -> MENUNGGU dijalankan pelatih
                                   (--feedback-run) sebagai percobaan berikutnya; kode sama dengan feedback-plan
  feedback-steps SESI [--last]     riwayat iterasi feedback: catatan -> pemahaman -> perubahan -> percobaan
                                   -> hasil; --last = hasil iterasi terakhir saja
  save SESI N                      "Simpan hasil terlebih dahulu" (berkas + daftar, robot tidak disentuh)
  deploy SESI N --to studio-new|studio-preset|page --target X [--name P] [--check] [--overwrite]
                                   "Simpan langsung ke robot". --check = hanya periksa: kode 0 siap,
                                   4 tidak bisa, 5 target berisi (perlu --overwrite), 6 sudah sama
  undo SESI N                      kembalikan target penerapan terakhir ke isi sebelumnya
  saved-list                       semua hasil tersimpan (semua sesi)
  bundle SESI N [--dir D]          salinan mandiri (berkas + feedback.md) ke folder lain
  plan SESI N                      = feedback-plan (nama lama)

Daftar untuk menu dicetak "KUNCI<TAB>LABEL" per baris.
"""

import argparse
import datetime
import os
import re
import shlex
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_action_core as acore  # noqa: E402
import autotune_walking as walk  # noqa: E402
import autotune_walking_core as wcore  # noqa: E402
import training_deploy as deploy  # noqa: E402
import training_feedback as feedback  # noqa: E402
import training_iteration as iteration  # noqa: E402
import training_results as results  # noqa: E402
import training_sources as src  # noqa: E402

NOT_TRAINABLE = 3
NOTHING_TO_APPLY, WEIGHTS_ONLY = 7, 8


rel = results.rel


def row(key, label):
    print("%s\t%s" % (key, label.replace("\t", " ")))


def field(name, value):
    print("  %-15s: %s" % (name, value))


def session_line(session):
    top = results.best(session)
    best = ("terbaik #%d skor %.3f" % (top["trial"]["n"], top["mean"])) if top else "belum ada hasil sah"
    return "%s -- %s, %s" % (session.name, results.status(session), best)


def related_sessions(kind, key):
    return [s for s in results.sessions() if results.matches_target(s, kind, key)]


# ---- Bascorro Studio ------------------------------------------------------------

def cmd_studio_list(_):
    store = src.load_studio_store()
    for v in src.studio_versions(store):
        motion = wcore.walking_motion(v["params"])[1].split(" (")[0]
        row(v["id"] or v["name"], "%s · %s · %s%s" % (v["name"], src.local_time(v["updated_at"]), motion,
                                                      " · aktif di Studio" if v["active"] else ""))


def cmd_studio_info(args):
    store = src.load_studio_store()
    version = src.find_studio_version(store, args.key)
    if version is None:
        print("Preset %r tidak ada lagi di %s (dihapus/diganti nama?) -- perbarui daftar." % (args.key, rel(src.STUDIO_VERSIONS)))
        return NOT_TRAINABLE
    p = version["params"]
    motion, motion_text = wcore.walking_motion(p)
    tuned = wcore.motion_params(motion)
    locked = ["x_move_amplitude", "y_move_amplitude"] + ([] if "angle_move_amplitude" in tuned else ["angle_move_amplitude"])
    reference_ok = store.get("poseReferenceId") == src.current_reference_id()
    print("Preset Bascorro Studio: %s" % version["name"])
    field("id", version["id"])
    field("diperbarui", "%s%s" % (src.local_time(version["updated_at"]), "  (versi aktif di Studio)" if version["active"] else ""))
    field("berkas", "%s (revisi %s)" % (rel(src.STUDIO_VERSIONS), store.get("revision", "-")))
    field("acuan page 2", "%s (%s)" % (store.get("poseReferenceId"), "cocok" if reference_ok else
                                       "BEDA dengan acuan sekarang %s" % src.current_reference_id()))
    field("jenis gerak", motion_text)
    field("perintah", "x %+.4f m, y %+.4f m, angle %+.4f rad, period %.3f s" % (
        p["x_move_amplitude"], p["y_move_amplitude"], p["angle_move_amplitude"], p["period_time"]))
    field("dilatih", ", ".join(tuned))
    field("dikunci", ", ".join(locked) + " (perintah gerak)")
    field("lama jalan", "%.0f s per percobaan" % walk.WALK_SEC_BY_MOTION[motion])
    field("kriteria", wcore.criteria_text(motion))
    field("persiapan", "robot DIGANTUNG tali kendur, lintasan kosong >= 1 m, D60/Studio walking tidak dipakai")
    found = related_sessions("studio", version["id"]) + [
        s for s in related_sessions("studio", version["name"]) if not s.state.get("start_id")]
    field("sesi training", "; ".join(session_line(s) for s in found) or "belum ada")
    if not reference_ok:
        print("\nTIDAK BISA DILATIH: versi Studio belum dimigrasi ke page 2 terbaru.\n"
              "  Jalankan dulu (Studio ditutup): python3 scripts/walking_ready_fk.py --write")
        return NOT_TRAINABLE
    return 0


# ---- Action Editor -----------------------------------------------------------------

def action_data():
    path, how = src.active_action_bin()
    return path, how, src.read_bin(path), src.motion_index()


def cmd_action_list(_):
    _, _, data, names = action_data()
    for s in src.list_pages(data, names):
        chain = " -> %s" % s["chain"][1] if len(s["chain"]) > 1 else ""
        label = "%3d · %s%s · %d step · ~%.1f s%s · %s" % (
            s["index"], s["name"] or "(tanpa nama)", " · %s" % s["motion_name"] if s["motion_name"] else "",
            s["stepnum"], s["seconds"], chain, acore.PROFILES[s["profile"]]["label"])
        row(s["index"], label)


def cmd_action_info(args):
    path, how, data, names = action_data()
    try:
        index = int(args.page)
    except ValueError:
        print("Nomor page %r tidak sah (harus angka 1..255)." % args.page)
        return NOT_TRAINABLE
    errors, warnings = src.check_page(data, index)
    page = src.decode_page(data, index) if 0 <= index < src.MAX_PAGE else None
    label = " (%s)" % names[index] if index in names else ""
    print("Page Action Editor: %d %s%s" % (index, page["name"] if page else "", label))
    field("berkas", "%s (%s)" % (rel(path), how))
    if page and page["stepnum"]:
        summary = src.page_summary(data, index, names)
        field("isi", "%d step, repeat %d, speed %d, accel %d, jadwal %s, sha %s" % (
            page["stepnum"], page["repeat"], page["speed"], page["accel"],
            "waktu" if page["schedule"] == src.TIME_BASE_SCHEDULE else "speed", summary["sha"]))
        field("step", " | ".join("%d: time %d pause %d (%.2f s)" % (n + 1, s["time"], s["pause"], src.step_seconds(page, s))
                                 for n, s in enumerate(page["steps"])))
        chain = summary["chain"]
        follow = ""
        if len(chain) > 1 and not summary["loops"]:
            nxt = src.decode_page(data, chain[1])
            follow = ", lalu page %d %s -> total ~%.1f s (page lanjutan tidak ikut dilatih)" % (
                chain[1], nxt["name"], summary["chain_seconds"])
        field("durasi", "~%.1f s%s" % (summary["seconds"], follow))
        if not errors:
            spec = acore.PROFILES[summary["profile"]]
            field("profil", "%s (%s) -- ditebak dari %s" % (summary["profile"], spec["label"], summary["profile_reason"]))
            field("dilatih", ", ".join("%s %g..%g" % (k, acore.BOUNDS[k][1], acore.BOUNDS[k][2]) for k in spec["params"]))
            field("kriteria", acore.criteria_text(summary["profile"]))
            field("persiapan", "robot BERBARING di matras (tengkurap/telentang sesuai page), tali kendur"
                  if spec["lying"] else "robot DIGANTUNG tali kendur, mulai dari page 2")
            field("page kerja", "kandidat diputar lewat page %d; page %d sendiri tidak ditulis selama training"
                  % (src.TRAINING_SCRATCH_PAGE, index))
    for warning in warnings:
        field("catatan", warning)
    if index == 2:
        field("PERHATIAN", "page 2 = acuan Init Pose walking (Studio + D60). Kalau hasilnya DIPAKAI, jalankan "
              "walking_ready_fk.py --write dan sync_ini_pose_from_page2.py --write")
    found = related_sessions("action", index)
    field("sesi training", "; ".join(session_line(s) for s in found) or "belum ada")
    if errors:
        print("\nTIDAK BISA DILATIH:\n  - " + "\n  - ".join(errors))
        return NOT_TRAINABLE
    return 0


def cmd_suggest_name(args):
    if args.kind == "studio":
        version = src.find_studio_version(src.load_studio_store(), args.target)
        stem = version["name"] if version else "studio"
    else:
        stem = "page%s" % args.target
    stem = re.sub(r"[^a-z0-9]+", "-", stem.lower()).strip("-")[:24] or "sesi"
    stem = "%s-%s" % (stem, datetime.date.today().strftime("%m%d"))
    name, n = stem, 2
    while os.path.exists(os.path.join(walk.SESSIONS_DIR, name)):
        name, n = "%s-%d" % (stem, n), n + 1
    print(name)


def cmd_free_page(_):
    _, _, data, _ = action_data()
    for index in list(range(200, src.MAX_PAGE)) + list(range(1, 200)):
        if index in (src.STUDIO_SCRATCH_PAGE, src.TRAINING_SCRATCH_PAGE):
            continue
        if not src.page_used(src.decode_page(data, index)):
            print(index)
            return 0
    return 1


# ---- sesi & hasil --------------------------------------------------------------------

def cmd_sessions(args):
    for session in related_sessions(args.kind, args.target):
        row(session.name, session_line(session))


def cmd_session_flags(args):
    session = results.open_session(args.session)
    stale = results.stale_reason(session)
    top = results.best(session)
    flags = {"KIND": session.kind, "DONE": results.cma_done(session.state), "BUDGET": session.state["budget"],
             "POPSIZE": session.state["popsize"], "STALE": stale[0] if stale else "",
             "RUNNING": session.running_pid() or 0, "HAS_BEST": 1 if top else 0,
             "BEST": top["trial"]["n"] if top else 0, "PAGE": session.state.get("page", ""),
             "START": session.state["start_name"], "START_ID": session.state.get("start_id") or session.state["start_name"],
             "PENDING": (iteration.pending(session.state) or {}).get("id", ""),
             "LAST_FB_TRIAL": next((s["trial"] for s in reversed(iteration.steps(session.state)) if s.get("trial")), 0)}
    for key, value in flags.items():
        print("%s=%s" % (key, shlex.quote(str(value))))


def describe_best(session, top, indent="  "):
    if not top:
        rejected = results.rejected_by_feedback(session)
        print("%sterbaik         : belum ada hasil yang sah (belum ada percobaan, semua jatuh/gagal%s)" % (
            indent, ", atau sisanya disimpulkan GAGAL lewat feedback: %s" % ", ".join("#%d" % n for n in rejected)
            if rejected else ""))
        return
    trial = top["trial"]
    print("%sterbaik         : percobaan %d (%s), skor %.3f" % (indent, trial["n"], trial["kind"], top["mean"]))
    print("%salasan          : %s" % (indent, top["reason"]))
    print("%sperubahan       : %s" % (indent, results.changes_text(session, trial)))
    print("%smetrik          : %s" % (indent, results.metrics_text(session, trial)))


def cmd_results(_):
    found = results.sessions()
    if not found:
        print("Belum ada hasil training di %s." % rel(walk.SESSIONS_DIR))
        return 0
    for session in found:
        state = session.state
        print("\n%s  [%s]" % (session.name, results.SOURCE_TEXT[session.kind]))
        print("  target          : %s" % results.target_text(session))
        print("  waktu           : mulai %s, terakhir %s" % (src.local_time(state.get("createdAt")),
                                                            src.local_time(results.last_activity(session))))
        print("  status          : %s" % results.status(session))
        stale = results.stale_reason(session)
        if stale:
            print("  perlu           : %s" % stale[1])
        files = results.session_files(session)
        saved = deploy.load_registry(session)["results"]
        print("  lokasi          : %s/ (%d hasil tersimpan, %d feedback)" % (files["dir"], len(saved), files["feedback"]))
        describe_best(session, results.best(session))
    return 0


def cmd_results_list(_):
    for session in results.sessions():
        top = results.best(session)
        saved = len(deploy.load_registry(session)["results"])
        notes = len(feedback.notes(session.dir))
        row(session.name, "%s · %s · %s · %s · %s%s%s" % (
            session.name, results.SOURCE_TEXT[session.kind], results.target_text(session),
            results.status(session), "terbaik #%d %.3f" % (top["trial"]["n"], top["mean"]) if top else "belum ada hasil",
            " · 💾 %d" % saved if saved else "", " · ✍ %d" % notes if notes else ""))


def cmd_result_info(args):
    session = results.open_session(args.session)
    state = session.state
    files = results.session_files(session)
    print("Sesi training: %s" % session.name)
    field("sumber", results.SOURCE_TEXT[session.kind])
    field("target", results.target_text(session))
    if session.kind == "studio":
        field("jenis gerak", state.get("motion_text", state.get("motion", "maju")))
        field("acuan page 2", state["reference_id"])
    else:
        field("profil", "%s (%s)" % (state["profile"], acore.PROFILES[state["profile"]]["label"]))
        field("isi page", "%s di %s" % (state["page_sha"], state["bin_path"]))
    field("dilatih", ", ".join(state["params"]))
    field("mulai", src.local_time(state.get("createdAt")))
    field("terakhir", src.local_time(results.last_activity(session)))
    field("status", results.status(session))
    stale = results.stale_reason(session)
    if stale:
        field("perlu", stale[1])
    field("lokasi", "%s/ (state.json, trials.csv, raw/ %d rekaman, feedback.jsonl %d catatan, hasil.json)" % (
        files["dir"], files["raw"], files["feedback"]))
    field("kriteria", results.criteria(session))
    if state.get("feedback_plan"):
        source = state["feedback_plan"].get("source", {})
        field("asal", "sesi dari rencana feedback lama: sesi %s percobaan %s" % (source.get("session"), source.get("trial")))
    done = iteration.steps(state)
    if done:
        waiting = iteration.pending(state)
        field("iterasi feedback", "%d langkah, %d diterapkan%s (riwayat: '🧾 Riwayat iterasi feedback')" % (
            len(done), sum(1 for s in done if s["status"] in ("dipakai", "diuji")),
            "; %s MENUNGGU dijalankan" % waiting["id"] if waiting else ""))
    if state.get("fixed"):
        field("ditetapkan", ", ".join("%s=%g" % kv for kv in state["fixed"].items()) + " (dari feedback, semua "
              "percobaan berikutnya)")
    describe_best(session, results.best(session))
    ranked = results.ranked(session)[:5]
    if ranked:
        print("  5 kandidat sah teratas:")
        for mean, reruns, trial in ranked:
            print("    #%-3d %-8s skor %.3f%s  %s" % (trial["n"], trial["kind"], mean,
                                                     " (%d ulangan)" % reruns if reruns else "",
                                                     results.changes_text(session, trial)))
    saved = deploy.load_registry(session)["results"]
    for record in saved:
        print("  tersimpan       : #%d %s -- %s" % (record["trial"], record["data_file"], deploy.delivery_text(record)))
    return 0


# ---- satu hasil -----------------------------------------------------------------------

def open_trial(args):
    session = results.open_session(args.session)
    which = str(args.trial)
    if which in ("best", ""):
        top = results.best(session)
        if not top:
            raise SystemExit("sesi %r belum punya hasil yang sah" % session.name)
        return session, top["trial"]
    trial = next((t for t in session.state["trials"] if str(t["n"]) == which), None)
    if trial is None:
        raise SystemExit("percobaan %s tidak ada di sesi %r (%d percobaan)" % (which, session.name,
                                                                             len(session.state["trials"])))
    if trial.get("rerun_of"):
        print("(percobaan %s adalah ulangan kandidat #%d -- yang dibuka kandidatnya)" % (which, trial["rerun_of"]))
        trial = next(t for t in session.state["trials"] if t["n"] == trial["rerun_of"])
    return session, trial


def trial_line(session, trial, mean, reruns, top, saved, notes, rejected):
    record = saved.get(trial["n"])
    parts = ["%s#%d %s" % ("🏆 " if top and top["trial"]["n"] == trial["n"] else "", trial["n"], trial["kind"]),
             "skor %.3f%s" % (mean, " (%d ulangan)" % reruns if reruns else "")]
    if record:
        done = deploy.last_success(record)
        parts.append("🤖 %s" % done["target_text"] if done else "💾 tersimpan")
    if notes.get(trial["n"]):
        parts.append("✍ %d" % notes[trial["n"]])
    if trial["n"] in rejected:
        parts.append("⚠ feedback: GAGAL")
    if not results.is_usable(session)(trial):
        parts.append("(jatuh/gagal/hampir)")
    changes = results.changes_text(session, trial)
    parts.append(changes if len(changes) <= 80 else changes[:77] + "...")
    return " · ".join(parts)


def cmd_trial_list(args):
    session = results.open_session(args.session)
    top = results.best(session)
    saved = {r["trial"]: r for r in deploy.load_registry(session)["results"]}
    notes = {}
    for note in feedback.notes(session.dir):
        candidate = int(note.get("candidate", note.get("trial")))
        notes[candidate] = notes.get(candidate, 0) + 1
    rejected = results.rejected_by_feedback(session)
    shown = []
    ranked = results.ranked(session)
    if top:
        ranked.sort(key=lambda item: item[2]["n"] != top["trial"]["n"])
    for mean, reruns, trial in ranked[:10]:
        shown.append(trial["n"])
        row(trial["n"], trial_line(session, trial, mean, reruns, top, saved, notes, rejected))
    for trial in session.state["trials"]:
        if trial["n"] not in shown and (trial["n"] in saved or trial["n"] in notes) and not trial.get("rerun_of"):
            mean, reruns = walk.candidate_cost(session.state, trial)
            row(trial["n"], trial_line(session, trial, mean, reruns, top, saved, notes, rejected))


def cmd_trial_info(args):
    session, trial = open_trial(args)
    state = session.state
    record = deploy.saved_record(session, trial["n"])
    top = results.best(session)
    mean, reruns = walk.candidate_cost(state, trial)
    print("Hasil training %s%s" % (deploy.result_id(session, trial), "" if record else " (belum disimpan)"))
    field("sesi", "%s (%s)" % (session.name, results.SOURCE_TEXT[session.kind]))
    field("sumber", results.target_text(session))
    field("percobaan", "#%d %s%s, dijalankan %s, label operator %s" % (
        trial["n"], trial["kind"], " gen %d" % trial["generation"] if trial.get("generation") is not None else "",
        src.local_time(trial.get("time")), trial.get("label") or "-"))
    reruns_runs = [t for t in state["trials"] if t.get("rerun_of") == trial["n"]]
    field("skor", "%.3f%s" % (mean, " (rata-rata %d run: %s)" % (len(reruns_runs) + 1, " ".join(
        "%.3f" % c for c in [trial["cost"]] + [t["cost"] for t in reruns_runs])) if reruns_runs else " (belum diulang)"))
    usable = results.is_usable(session)(trial)
    if top and top["trial"]["n"] == trial["n"]:
        field("status", "TERBAIK sesi ini -- %s" % top["reason"])
    else:
        rank = [t["n"] for _, _, t in results.ranked(session)]
        field("status", "peringkat %d dari %d kandidat sah%s" % (rank.index(trial["n"]) + 1, len(rank), (
            "; terbaik sekarang #%d" % top["trial"]["n"]) if top else "") if trial["n"] in rank
              else "TIDAK sah (jatuh/gagal/hampir) -- feedback boleh, disimpan/diterapkan tidak")
    field("perubahan", results.changes_text(session, trial))
    field("metrik", results.metrics_text(session, trial) or "-")
    field("kriteria", results.criteria(session))
    produced = iteration.step_of_trial(state, trial["n"])
    if produced:
        print("\n" + "\n".join("  " + line for line in iteration.result_text(state, produced).splitlines()))
        field("iterasi", "%s %s -- dari catatan %s pada percobaan #%s" % (
            produced["id"], iteration.STATUS_TEXT.get(produced["status"], produced["status"]),
            ", ".join(produced["notes"]), produced["source"]["trial"]))
    for step in iteration.steps(state):
        if step["source"]["trial"] == trial["n"] or trial["n"] in step["source"].get("configs", []):
            field("feedback ->", "%s %s%s" % (step["id"], iteration.STATUS_TEXT.get(step["status"], step["status"]),
                                              " -> percobaan #%s (%s)" % (step["trial"], (step.get("result") or {}).get(
                                                  "verdict", "-")) if step.get("trial") else ""))
    if record:
        field("disimpan", "%s -> %s" % (src.local_time(record["saved_at"]), ", ".join(record["files"])))
        field("kiriman", deploy.delivery_text(record))
        for entry in record["deployments"]:
            print("    %s · %s · %s · %s%s" % (
                src.local_time(entry["at"]), "kembalikan" if entry["action"] == "undo" else "kirim",
                entry["status"].upper(), entry["target_text"], " · cadangan %s" % entry["backup"]
                if entry.get("backup") else ""))
            print("        %s" % entry.get("message", ""))
    else:
        field("disimpan", "belum -- pilih 'Simpan hasil terlebih dahulu'" if usable else "-")
    notes = feedback.notes(session.dir, trial["n"])
    field("feedback", "%d catatan%s" % (len(notes), ":" if notes else " -- pilih 'Tambah feedback manual'"))
    for note in notes:
        print(feedback.note_text(note))
    return 0


def cmd_feedback_add(args):
    session, trial = open_trial(args)
    record = deploy.saved_record(session, trial["n"])
    done = deploy.last_success(record) if record else None
    mean, _ = walk.candidate_cost(session.state, trial)
    print("Feedback untuk hasil %s -- %s, %s, percobaan #%d (%s), skor %.3f" % (
        deploy.result_id(session, trial), results.SOURCE_TEXT[session.kind], results.target_text(session),
        trial["n"], trial["kind"], mean))
    tested_on = args.tested_on
    if args.text_file is None and tested_on is None:
        default = done["target_text"] if done else "uji ulang training (page kerja/walking dari menu Training)"
        try:
            answer = input("  Diuji lewat apa? Enter = '%s', atau ketik sendiri (mis. 'D60 di lapangan'): "
                           % default).strip()
        except (EOFError, KeyboardInterrupt):
            print("\n  Dibatalkan -- tidak ada feedback yang disimpan.")
            return 2
        tested_on = answer or default
    if args.text_file is not None:
        with open(args.text_file, encoding="utf-8") as handle:
            text = handle.read()
        note = feedback.add_note(session.dir, session.name, trial["n"], text, args.verdict, args.where,
                                 tested_on=tested_on, result_id=record["id"] if record else None)
        print("✔ feedback tersimpan (%s, %d karakter) di %s" % (note["id"], len(note["text"]),
                                                               rel(feedback.feedback_path(session.dir))))
        print("  Perubahan yang bisa diterapkan: training.py feedback-plan %s %d" % (session.name, trial["n"]))
        return 0
    note = feedback.prompt_note(session.dir, session.name, trial["n"], args.where, tested_on=tested_on,
                                result_id=record["id"] if record else None)
    return 0 if note else 2


def cmd_feedback_list(args):
    session = results.open_session(args.session)
    notes = feedback.notes(session.dir, args.trial)
    print("Feedback sesi %s%s: %d catatan (%s)" % (session.name, " kandidat #%s" % args.trial if args.trial else "",
                                                   len(notes), rel(feedback.feedback_path(session.dir))))
    for note in notes:
        print("\n  kandidat #%s" % note.get("candidate", note.get("trial")))
        print(feedback.note_text(note))
    return 0


def usable_trial(args):
    """(sesi, kandidat) yang boleh disimpan/diterapkan: kandidat sah."""
    session, trial = open_trial(args)
    return session, results.find_trial(session, trial["n"])


def cmd_save(args):
    session, trial = usable_trial(args)
    record, created = deploy.save_result(session, trial)
    print("%s hasil %s (%s, percobaan #%d, skor %.3f)" % ("✔ Tersimpan:" if created else "Sudah tersimpan sejak %s:"
                                                          % src.local_time(record["saved_at"]), record["id"],
                                                          results.SOURCE_TEXT[session.kind], trial["n"], record["score"]))
    for path in record["files"]:
        print("  berkas : %s" % path)
    print("  daftar : %s" % rel(deploy.registry_path(session)))
    if session.kind == "studio":
        print("  isi    : 29 parameter walking lengkap + metrik + kriteria + acuan page 2 %s" % session.state["reference_id"])
    else:
        print("  isi    : page hasil (YAML format action_yaml.py + JSON berisi page utuh, kenop, perubahan raw per sendi)")
    print("  Robot BELUM disentuh. Pilih lagi lewat Training -> Riwayat hasil (sesi %s, #%d) atau Hasil tersimpan,\n"
          "  lalu 'Simpan langsung ke robot' kapan saja. Feedback boleh ditambahkan sebelum atau sesudahnya."
          % (session.name, trial["n"]))
    return 0


def cmd_deploy(args):
    session, trial = usable_trial(args)
    if args.check:
        plan = deploy.check(session, trial, args.to, args.target, args.name)
        print(deploy.plan_text(plan))
        if not plan["ok"]:
            return 4
        if plan["unchanged"]:
            return 6
        return 5 if plan["overwrite"] else 0
    plan, entry = deploy.deploy(session, trial, args.to, args.target, args.name, args.overwrite)
    print(deploy.entry_text(entry))
    return 0 if entry["status"] in ("berhasil", "sama") else 1


def cmd_undo(args):
    session, trial = open_trial(args)
    entry = deploy.undo(session, trial)
    print(deploy.entry_text(entry))
    return 0 if entry["status"] == "berhasil" else 1


def cmd_saved_list(_):
    for session, record in deploy.all_saved():
        notes = len(feedback.notes(session.dir, record["trial"]))
        row("%s:%d" % (session.name, record["trial"]), "%s · %s · %s · disimpan %s · %s%s" % (
            record["id"], results.SOURCE_TEXT[session.kind], record["source_target"],
            src.local_time(record["saved_at"]), deploy.delivery_text(record), " · ✍ %d" % notes if notes else ""))


def cmd_bundle(args):
    session, trial = usable_trial(args)
    folder, written = deploy.bundle(session, trial, args.dir)
    print("✔ Salinan hasil %s di %s/" % (deploy.result_id(session, trial), rel(folder)))
    for path in written:
        print("  %s" % os.path.basename(path))
    return 0


# ---- feedback -> iterasi berikutnya -------------------------------------------------------

def proposal(args):
    session, trial = open_trial(args)
    where = getattr(args, "where", None) or "riwayat hasil"
    return session, trial, iteration.propose(session.state, session.dir, trial, where)


def proposal_code(step):
    if step["changes"]:
        return 0
    return WEIGHTS_ONLY if step["status"] == "usulan" else NOTHING_TO_APPLY


def cmd_feedback_plan(args):
    session, trial, step = proposal(args)
    if not step["notes"]:
        print("Belum ada feedback baru untuk hasil #%d sesi %s (yang sudah dipakai iterasi sebelumnya tidak dihitung "
              "lagi). Tulis dulu: Tambah feedback manual." % (trial["n"], session.name))
        return NOTHING_TO_APPLY
    print(iteration.step_text(session.state, step, session.name))
    print("\n  Cara alat ini memahami teks: pola kata bahasa Indonesia (training_feedback.py BEHAVIORS), bukan model "
          "bahasa;\n  arah & besar perubahan dari pedoman tetap (training_iteration.py), lalu dibuktikan di robot.")
    return proposal_code(step)


def cmd_feedback_apply(args):
    session, trial, step = proposal(args)
    pid = session.running_pid()
    if pid:
        raise SystemExit("sesi %s sedang berjalan (pid %d) -- tulis catatannya di terminal training itu dengan 'c'"
                         % (session.name, pid))
    if not step["notes"]:
        print("Belum ada feedback baru untuk hasil #%d." % trial["n"])
        return NOTHING_TO_APPLY
    _, recost = iteration._trainer(session.state)
    if args.decline or step["status"] == "tidak_berlaku":
        iteration.record(session.state, step, "ditolak" if args.decline else "tidak_berlaku")
        session.save()
        print("%s dicatat sebagai %s -- tidak ada yang berubah di sesi." % (
            step["id"], iteration.STATUS_TEXT[step["status"]]))
        return NOTHING_TO_APPLY
    iteration.apply(session.state, step, recost)
    session.save()
    if step["status"] == "menunggu":
        print("✔ %s disimpan di %s: MENUNGGU -- percobaan berikutnya sesi %s (#%d) memakai:" % (
            step["id"], rel(session.path), session.name, step["next_trial"]))
        for change in step["changes"]:
            print("    " + iteration._change_line(change))
        return 0
    print("✔ %s: %s" % (step["id"], step["used"]))
    return WEIGHTS_ONLY


def cmd_feedback_steps(args):
    session = results.open_session(args.session)
    state = session.state
    if args.last:
        done = [s for s in iteration.steps(state) if s.get("trial")]
        if not done:
            print("Belum ada iterasi feedback yang dijalankan di sesi %s." % session.name)
            return 1
        print(iteration.result_text(state, done[-1]))
        print("  status: %s%s" % (iteration.STATUS_TEXT.get(done[-1]["status"], done[-1]["status"]),
                                 " -- %s" % done[-1]["used"] if done[-1].get("used") else ""))
        return 0
    print("Riwayat iterasi feedback sesi %s (%s):\n" % (session.name, results.target_text(session)))
    print(iteration.history_text(state, session.name))
    for step in iteration.steps(state):
        if step["status"] in ("menunggu", "usulan") or args.full:
            print("\n" + iteration.step_text(state, step, session.name))
    return 0


def cmd_start_next(_):
    raise SystemExit("Menu ini versi lama: 'Training lanjutan' sekarang = feedback diterapkan di sesi yang SAMA dan "
                     "langsung dijalankan.\nKeluar dari menu Training lalu buka lagi.")


def cmd_action_params(args):
    """Apa yang sebenarnya dioptimasi training page Action Editor, untuk page/sesi tertentu."""
    if args.session:
        session = results.open_session(args.session)
        if session.kind != "action":
            raise SystemExit("sesi %s bukan training Action Editor" % session.name)
        state = session.state
        page, index, profile, params = iteration.snapshot(state), state["page"], state["profile"], state["params"]
        name_text = "%s %s" % (state.get("page_name", ""), state.get("motion_name", ""))
        trial = results.find_trial(session, args.trial or "best") if (args.trial or results.best(session)) else None
        setup = {k: state.get(k) for k in ("sigma0", "popsize", "budget", "baseline_runs")}
        weights = iteration.current_weights(state)
        title = "sesi %s, %s, isi page %s" % (session.name, state["start_name"], state["page_sha"])
    else:
        _, _, data, names = action_data()
        index = int(args.page)
        page = src.decode_page(data, index)
        profile = src.guess_profile(index, page, names)[0]
        params, trial, state = acore.PROFILES[profile]["params"], None, None
        name_text = "%s %s" % (page["name"], names.get(index, ""))
        setup = {"sigma0": 0.2, "popsize": "4 + 3 ln(n)", "budget": walk_action_budget(), "baseline_runs": 2}
        weights = acore.PROFILES[profile]["weights"]
        title = "page %d %s%s" % (index, page["name"], " (%s)" % names[index] if index in names else "")
    spec = acore.PROFILES[profile]
    print("APA YANG DIOPTIMASI TRAINING -- %s, profil %s (%s)\n" % (title, profile, spec["label"]))
    print("1. VARIABEL PEMBENTUK GERAKAN (yang dicari CMA-ES; disebut \"kenop\")")
    print("   BUKAN nilai tiap servo per step satu per satu (31 sendi x %d step), melainkan kenop yang dikenakan ke"
          "\n   SEMUA step page (dari feedback juga ke sebagian step, kunci \"kenop@a-b\"):" % page["stepnum"])
    print("   %-16s %-7s %-12s %-10s %s" % ("nama", "satuan", "batas", "dicari?", "sendi yang berubah (raw per 1 satuan) "
                                                                         "dan arah badannya"))
    for knob, (_, lo, hi) in acore.BOUNDS.items():
        if knob == "time_scale":
            effect = "time & pause tiap step x nilai (1 time = 8 ms x speed/32); 1,0 = asli"
        else:
            effect = ", ".join("%s %+.1f" % (joint, direction * acore.RAW_PER_DEG)
                               for joint, direction in acore.KNOB_JOINTS[knob])
            body = {"roll_ankle_deg": "; NEGATIF = badan condong ke KANAN di atas engkel",
                    "pitch_ankle_deg": "; NEGATIF = badan condong ke DEPAN di atas engkel",
                    "pitch_hip_deg": "; NEGATIF = badan atas menekuk ke DEPAN",
                    "roll_hip_deg": "; geser pinggul ke samping (dua kaki searah)"}[knob]
            effect += body
        print("   %-16s %-7s %-12s %-10s %s" % (knob, "x" if knob == "time_scale" else "derajat", "%g..%g" % (lo, hi),
                                               "YA" if knob in params else "dikunci", effect))
    print("   rumus per step & sendi: raw_baru = raw_asli + round(arah_sendi x kenop x 4096/360), dijepit 0..4095;"
          "\n   arah_sendi = DIR_R/DIR_L IK walking_module (walking_ready_fk.py). Sendi INVALID/TORQUE_OFF dibiarkan.")
    print("   tempat: scripts/autotune_action_core.py (KNOB_JOINTS, BOUNDS, NEUTRAL, PROFILES, apply_knobs);"
          "\n           per sesi: state.json \"params\" (yang dicari), \"lo\"/\"hi\" (batas), \"cma\" (titik tengah),"
          "\n           tiap percobaan \"params\" (nilai kenopnya), \"fixed\" (kenop yang ditetapkan feedback).")
    print("   TIDAK dioptimasi: posisi servo per step secara bebas, jumlah step, speed/accel page, p-gain,"
          "\n           offset sendi (offset.yaml), page lanjutan (next).")
    ph = acore.page_phases(page, name_text)
    print("\n2. BAGIAN GERAKAN PAGE INI (dari data servo, dipakai untuk menerjemahkan feedback)")
    if ph is None:
        print("   satu step: tidak ada fase")
    else:
        for row in ph["rows"]:
            step = page["steps"][row["step"] - 1]
            phase = next((acore.PHASE_NAME[k] for k, (a, b) in ph["phases"].items() if a <= row["step"] <= b), "-")
            print("   step %d: time %3d pause %3d (%.2f s) | kanan-kiri tidak simetris %5.1f deg | %s" % (
                row["step"], step["time"], step["pause"], src.step_seconds(page, step), row["asym_deg"], phase))
        if ph["kick"]:
            print("   kaki yang bergerak: %s (%s)%s" % (acore.SIDE_TEXT[ph["kick"]], ph["evidence"], (
                "; cocok dengan nama" if ph["name_side"] == ph["kick"] else "; nama page menyebut kaki %s -- BEDA"
                % acore.SIDE_TEXT[ph["name_side"]]) if ph["name_side"] else ""))
        else:
            print("   tidak ada fase satu kaki (kiri-kanan bergerak simetris) -- feedback memakai kenop seluruh page")
    print("\n3. CONTOH NILAI SERVO")
    if trial is not None:
        knobs = results.trial_params(session, trial)
        print("   percobaan #%d: %s" % (trial["n"], results.changes_text(session, trial)))
        for line in iteration.servo_table(state, dict(acore.NEUTRAL), knobs, limit=24):
            print("     " + line)
    else:
        example = dict(acore.NEUTRAL, roll_ankle_deg=-1.0)
        tuned = acore.apply_knobs(page, example)
        print("   roll_ankle_deg = -1,0 (badan 1 deg ke kanan) mengubah: %s" % ", ".join(
            "%s %s raw" % (j, "/".join("%+d" % d for d in v)) for j, v in acore.changed_joints(page, tuned).items()))
        first = page["steps"][0]["positions"]
        print("   mis. step 1 r_ank_roll %d -> %d, l_ank_roll %d -> %d" % (
            first[src.JOINT_IDS["r_ank_roll"]], tuned["steps"][0]["positions"][src.JOINT_IDS["r_ank_roll"]],
            first[src.JOINT_IDS["l_ank_roll"]], tuned["steps"][0]["positions"][src.JOINT_IDS["l_ank_roll"]]))
    print("\n4. SETELAN ALGORITMA (bukan gerakan; feedback TIDAK mengubahnya, kecuali titik tengah CMA)")
    print("   sigma0 %s (langkah awal, pecahan rentang kenop), popsize %s, anggaran %s percobaan CMA, baseline %s,"
          "\n   jeda pendinginan, batas tegangan -- argumen scripts/autotune_action.py" % (
              setup["sigma0"], setup["popsize"], setup["budget"], setup["baseline_runs"]))
    print("\n5. BOBOT PENILAIAN (sasaran evaluasi; feedback BISA mengubahnya)")
    for key, weight in weights.items():
        print("   %5.1f%%  %-18s %s" % (100 * weight, key, acore.METRIC_TEXT[key]))
    print("\n6. DARI HASIL KE PAGE")
    print("   isi page saat sesi dibuat (state.json page_hex) + kenop percobaan -> apply_knobs -> page 512 byte"
          "\n   (checksum dihitung ulang) -> disimpan YAML format action_yaml.py + JSON page_hex -> dikirim ke page"
          "\n   tujuan dengan cadangan .bin dan dibaca balik (menu hasil -> Simpan/Kirim ke robot).")
    print("\n7. YANG BERUBAH SAAT KAMU MEMBERI FEEDBACK")
    print("   - kenop: kenop yang dicari -> titik tengah CMA pindah; kenop fase (\"kenop@a-b\") / kenop yang"
          "\n     dikunci -> ditetapkan (\"fixed\") untuk semua percobaan berikutnya")
    print("   - bobot penilaian (bagian 5), dijaga 0,5..2x bobot bawaan")
    print("   - TIDAK: setelan algoritma (bagian 4), isi page asli di .bin, page lain")
    return 0


def walk_action_budget():
    import autotune_action as act
    return act.DEFAULT_BUDGET


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("studio-list").set_defaults(func=cmd_studio_list)
    p = sub.add_parser("studio-info")
    p.add_argument("key")
    p.set_defaults(func=cmd_studio_info)
    sub.add_parser("action-list").set_defaults(func=cmd_action_list)
    p = sub.add_parser("action-info")
    p.add_argument("page")
    p.set_defaults(func=cmd_action_info)
    sub.add_parser("free-page").set_defaults(func=cmd_free_page)
    p = sub.add_parser("suggest-name")
    p.add_argument("kind", choices=["studio", "action"])
    p.add_argument("target")
    p.set_defaults(func=cmd_suggest_name)
    p = sub.add_parser("sessions")
    p.add_argument("kind", choices=["studio", "action"])
    p.add_argument("target")
    p.set_defaults(func=cmd_sessions)
    for name, func in (("session-flags", cmd_session_flags), ("result-info", cmd_result_info),
                       ("trial-list", cmd_trial_list)):
        p = sub.add_parser(name)
        p.add_argument("session")
        p.set_defaults(func=func)
    sub.add_parser("results").set_defaults(func=cmd_results)
    sub.add_parser("results-list").set_defaults(func=cmd_results_list)
    sub.add_parser("saved-list").set_defaults(func=cmd_saved_list)
    for name, func in (("trial-info", cmd_trial_info), ("save", cmd_save), ("undo", cmd_undo),
                       ("plan", cmd_feedback_plan), ("feedback-plan", cmd_feedback_plan)):
        p = sub.add_parser(name)
        p.add_argument("session")
        p.add_argument("trial", help="nomor percobaan, atau best")
        p.set_defaults(func=func)
    p = sub.add_parser("feedback-apply")
    p.add_argument("session")
    p.add_argument("trial")
    p.add_argument("--decline", action="store_true", help="catat usulan sebagai TIDAK diterapkan (catatan saja)")
    p.set_defaults(func=cmd_feedback_apply)
    p = sub.add_parser("feedback-steps")
    p.add_argument("session")
    p.add_argument("--last", action="store_true", help="hasil iterasi terakhir saja")
    p.add_argument("--full", action="store_true", help="tampilkan juga bukti lengkap tiap langkah")
    p.set_defaults(func=cmd_feedback_steps)
    p = sub.add_parser("action-params")
    p.add_argument("page", nargs="?")
    p.add_argument("--session")
    p.add_argument("--trial")
    p.set_defaults(func=cmd_action_params)
    p = sub.add_parser("feedback-add")
    p.add_argument("session")
    p.add_argument("trial")
    p.add_argument("--where", default="riwayat hasil", help="konteks catatan (mis. 'sesudah training')")
    p.add_argument("--tested-on", help="diuji lewat apa (bawaan: ditanyakan)")
    p.add_argument("--text-file", help="isi feedback dari berkas (tanpa tanya-jawab)")
    p.add_argument("--verdict", choices=sorted(feedback.VERDICT_TEXT), help="dengan --text-file")
    p.set_defaults(func=cmd_feedback_add)
    p = sub.add_parser("feedback-list")
    p.add_argument("session")
    p.add_argument("trial", nargs="?")
    p.set_defaults(func=cmd_feedback_list)
    p = sub.add_parser("deploy")
    p.add_argument("session")
    p.add_argument("trial")
    p.add_argument("--to", required=True, choices=["studio-new", "studio-preset", "page"])
    p.add_argument("--target", required=True, help="nama preset baru / id-atau-nama preset / nomor page")
    p.add_argument("--name", help="page: nama page (maks 13 karakter); bawaan nama page asli")
    p.add_argument("--check", action="store_true", help="hanya periksa dan tampilkan rencana")
    p.add_argument("--overwrite", action="store_true", help="boleh menimpa target yang berisi")
    p.set_defaults(func=cmd_deploy)
    p = sub.add_parser("bundle")
    p.add_argument("session")
    p.add_argument("trial")
    p.add_argument("--dir", help="folder tujuan (bawaan %s)" % rel(deploy.BUNDLE_DIR))
    p.set_defaults(func=cmd_bundle)
    p = sub.add_parser("start-next")        # menu versi lama yang masih terbuka
    p.add_argument("session")
    p.add_argument("trial")
    p.add_argument("--name")
    p.add_argument("--trials", type=int)
    p.set_defaults(func=cmd_start_next)
    args = ap.parse_args()
    return args.func(args) or 0


if __name__ == "__main__":
    sys.exit(main())

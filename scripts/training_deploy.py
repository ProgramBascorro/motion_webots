"""Simpan hasil training dan terapkan ke robot -- untuk kedua sumber (MURNI, tanpa ROS).

  save_result(session, trial)   "Simpan hasil terlebih dahulu": berkas di
        <sesi>/export/<id>.json (+ .yaml untuk page) dalam format sumbernya, lalu
        didaftarkan di <sesi>/hasil.json. Robot BELUM disentuh; feedback boleh
        menyusul kapan saja.
  check(...) / deploy(...)      "Simpan langsung ke robot": hasil dibaca LAGI dari
        berkas yang disimpan (yang diterapkan = yang tersimpan), koneksi/format/
        target diperiksa, isi lama dicadangkan, ditulis, dibaca balik, lalu dicatat
        di hasil.json -- berhasil maupun gagal. Gagal tidak menghapus salinan hasil.
  undo(session, trial)          target dikembalikan ke isi sebelum penerapan terakhir.

Target:
  Bascorro Studio -> walking_versions.json lewat SERVER Studio (plugin Vite, satu-
        satunya penulis berkas itu: tiap tulis dicadangkan ke data/backup/ dan
        dibaca ulang di sana). Preset BARU, atau menimpa preset yang ada (id & nama
        tetap). Tidak ada yang dikirim ke walking_module: memakainya = Studio Load
        -> Apply & Start, atau mode D60 (membaca ulang preset tiap sesi D60).
  Action Editor   -> satu page di .bin yang dipakai action_module (env op3_manager).
        action_module membuka ulang .bin setiap page diputar: langsung aktif.

Menerapkan TIDAK PERNAH menggerakkan robot. Menguji gerakan = tindakan terpisah
(pelatih --rerun-best 1 --trial N; menu "Uji hasil ini di robot").
"""

import datetime
import json
import math
import os
import re
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_action as act  # noqa: E402
import autotune_action_core as acore  # noqa: E402
import autotune_walking as walk  # noqa: E402
import autotune_walking_core as wcore  # noqa: E402
import training_feedback as feedback  # noqa: E402
import training_results as results  # noqa: E402
import training_sources as src  # noqa: E402

REGISTRY_FILE = "hasil.json"
REGISTRY_FORMAT = "bascorro-studio/training-results"
STUDIO_EXPORT = "bascorro-studio/walking-autotune-export"
ACTION_EXPORT = "bascorro-studio/action-autotune-export"
# `ros2 run` menolak skrip ini kalau bit eksekusinya hilang (100644 di git), jadi lewat python3.
ACTION_YAML = "src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/action_yaml.py"
BUNDLE_DIR = os.path.join(walk.HERE, "training_export")
STUDIO_BACKUP_DIR = os.path.join(os.path.dirname(src.STUDIO_VERSIONS), "backup")
FLOAT32_MAX = 3.4028234663852886e38


def import_command(bin_path, yaml_path):
    return "python3 %s --action-file %s import --in %s" % (ACTION_YAML, bin_path, yaml_path)


def _stamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def _backup(bin_path):
    """Salinan utuh .bin sebelum ditulis: <bin>.bak.<waktu>[-n] (pola action_yaml.py). Dua tulis
    dalam detik yang sama tidak boleh berbagi nama -- cadangan pertama akan hilang."""
    base = "%s.bak.%s" % (bin_path, _stamp())
    path, n = base, 2
    while os.path.exists(path):
        path, n = "%s-%d" % (base, n), n + 1
    shutil.copyfile(bin_path, path)
    return path


def _abs(path):
    return path if os.path.isabs(path) else os.path.join(walk.HERE, path)


def result_id(session, trial):
    return "%s-n%03d" % (session.name, trial["n"])


# ---- daftar hasil tersimpan (<sesi>/hasil.json, hanya ditulis menu, tidak oleh pelatih) ----

def registry_path(session):
    return os.path.join(session.dir, REGISTRY_FILE)


def load_registry(session):
    try:
        with open(registry_path(session), encoding="utf-8") as handle:
            registry = json.load(handle)
    except FileNotFoundError:
        return {"format": REGISTRY_FORMAT, "formatVersion": 1, "results": []}
    if registry.get("format") != REGISTRY_FORMAT:
        raise SystemExit("%s bukan daftar hasil training" % registry_path(session))
    return registry


def _save_registry(session, registry):
    path = registry_path(session)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(registry, handle, indent=1, ensure_ascii=False)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def saved_record(session, n):
    return next((r for r in load_registry(session)["results"] if r["trial"] == int(n)), None)


def all_saved():
    """[(sesi, catatan hasil)] semua sesi, kejadian terbaru dulu."""
    found = [(session, record) for session in results.sessions() for record in load_registry(session)["results"]]
    return sorted(found, key=lambda item: last_event(item[1]), reverse=True)


def last_event(record):
    return max([record.get("saved_at") or ""] + [d.get("at") or "" for d in record.get("deployments", [])])


def last_success(record):
    """Penerapan berhasil terakhir yang belum dibatalkan, atau None."""
    for entry in reversed(record.get("deployments", [])):
        if entry.get("action") == "deploy" and entry.get("status") == "berhasil" and not entry.get("undone_at"):
            return entry
    return None


def delivery_text(record):
    """Status pengiriman ke robot dalam satu baris."""
    if record is None:
        return "belum disimpan"
    entries = record.get("deployments", [])
    if not entries:
        return "tersimpan, BELUM dikirim ke robot"
    done = last_success(record)
    last = entries[-1]
    if last.get("action") == "undo" and last.get("status") == "berhasil":
        return "dikirim lalu DIKEMBALIKAN %s (%s)" % (src.local_time(last["at"]), last["target_text"])
    text = ("di robot: %s sejak %s" % (done["target_text"], src.local_time(done["at"])) if done
            else "belum pernah berhasil dikirim")
    if last.get("status") == "gagal":
        text += "; percobaan kirim terakhir GAGAL %s" % src.local_time(last["at"])
    return text


# ---- "Simpan hasil terlebih dahulu" ----------------------------------------------------

def _write_json(path, data):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=1, ensure_ascii=False)
        handle.flush()
        os.fsync(handle.fileno())


def _write_files(session, trial, rid):
    """Berkas hasil dalam format sumbernya: (daftar berkas, berkas data JSON)."""
    out = os.path.join(session.dir, "export")
    os.makedirs(out, exist_ok=True)
    base = os.path.join(out, rid)
    state = session.state
    mean, reruns = walk.candidate_cost(state, trial)
    meta = {"session": session.name, "source": results.SOURCE_TEXT[session.kind], "trial": trial["n"],
            "trial_kind": trial["kind"], "trial_time": trial.get("time"), "score": round(mean, 4), "reruns": reruns,
            "metrics": trial.get("metrics"), "label": trial.get("label"), "criteria": results.criteria(session),
            "weights": state.get("weights"), "changes": results.changes_text(session, trial),
            "exported_at": feedback.now_iso()}
    if session.kind == "studio":
        meta.update({"format": STUDIO_EXPORT, "formatVersion": 2, "start_name": state["start_name"],
                     "start_id": state.get("start_id"), "reference_id": state["reference_id"],
                     "motion": state.get("motion", "maju"), "params": results.trial_params(session, trial)})
        _write_json(base + ".json", meta)
        return [results.rel(base + ".json")], results.rel(base + ".json")
    try:
        import yaml
    except ImportError:
        raise SystemExit("PyYAML tidak ada (jalankan di dalam kontainer op3)")
    page, page_bytes = results.tuned_page(session, trial)
    original = act.snapshot_page(state)
    entry = src.page_to_yaml_entry(page)
    with open(base + ".yaml", "w", encoding="utf-8") as handle:
        handle.write("# Hasil training sesi %s percobaan %d (skor %.3f) -- %s\n" % (
            session.name, trial["n"], mean, state["start_name"]))
        handle.write("# Impor (dari akar workspace): %s\n#   (menimpa page %d; cadangan .bin dibuat otomatis)\n"
                     % (import_command(state["bin_path"], results.rel(base + ".yaml")), entry["index"]))
        yaml.safe_dump({"meta": {"action_file": state["bin_path"], "generated_at": meta["exported_at"],
                                 "source_page": state["page"], "source_page_sha": state["page_sha"]},
                        "pages": [entry]}, handle, sort_keys=False)
    meta.update({"format": ACTION_EXPORT, "formatVersion": 2, "page": state["page"], "page_name": state["page_name"],
                 "motion_name": state.get("motion_name", ""), "profile": state["profile"],
                 "bin_path": state["bin_path"], "page_sha": state["page_sha"],
                 "result_sha": src.page_sha({"raw": page_bytes}), "page_hex": page_bytes.hex(),
                 "knobs": results.trial_params(session, trial),
                 "joint_raw_changes": acore.changed_joints(original, page)})
    _write_json(base + ".json", meta)
    return [results.rel(base + ".yaml"), results.rel(base + ".json")], results.rel(base + ".json")


def load_data(record):
    """Isi hasil tersimpan, DIBACA ULANG dari berkasnya dan diperiksa -- dasar setiap penerapan."""
    path = _abs(record["data_file"])
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, ValueError) as error:
        raise SystemExit("berkas hasil %s tidak terbaca (%s) -- simpan ulang dari riwayat sesinya" % (path, error))
    if record["source"] == "studio":
        if data.get("format") != STUDIO_EXPORT or not isinstance(data.get("params"), dict):
            raise SystemExit("%s bukan hasil training Bascorro Studio" % path)
        return data
    if data.get("format") != ACTION_EXPORT or "page_hex" not in data:
        raise SystemExit("%s bukan hasil training page (tidak ada page_hex)" % path)
    raw = bytes.fromhex(data["page_hex"])
    page = src.decode_page(raw, 0)
    if len(raw) != src.PAGE_SIZE or not page["checksum_ok"] or src.page_sha(page) != data["result_sha"]:
        raise SystemExit("%s rusak: page di dalamnya tidak cocok dengan sha/checksum-nya" % path)
    return data


def save_result(session, trial):
    """(catatan hasil, baru?) -- hasil yang sudah tersimpan dipakai lagi (berkasnya diperiksa)."""
    rid = result_id(session, trial)
    registry = load_registry(session)
    record = next((r for r in registry["results"] if r["id"] == rid), None)
    if record is not None:
        load_data(record)
        return record, False
    files, data_file = _write_files(session, trial, rid)
    mean, reruns = walk.candidate_cost(session.state, trial)
    top = results.best(session)
    is_best = bool(top and top["trial"]["n"] == trial["n"])
    record = {"id": rid, "session": session.name, "trial": trial["n"], "source": session.kind,
              "trial_kind": trial["kind"], "generation": trial.get("generation"), "trial_time": trial.get("time"),
              "saved_at": feedback.now_iso(), "score": round(mean, 4), "reruns": reruns,
              "best_at_save": is_best, "reason_at_save": top["reason"] if is_best else None,
              "label": trial.get("label"), "metrics": trial.get("metrics"),
              "source_target": results.target_text(session), "changes": results.changes_text(session, trial),
              "files": files, "data_file": data_file, "deployments": []}
    registry = load_registry(session)
    registry["results"].append(record)
    _save_registry(session, registry)
    load_data(record)
    return record, True


def _append_entry(session, rid, entry, mark_undone=None):
    registry = load_registry(session)
    record = next(r for r in registry["results"] if r["id"] == rid)
    if mark_undone is not None:
        for previous in record["deployments"]:
            if previous.get("at") == mark_undone and previous.get("action") == "deploy":
                previous["undone_at"] = entry["at"]
    record["deployments"].append(entry)
    _save_registry(session, registry)
    return record


# ---- pemeriksaan sebelum menulis ---------------------------------------------------

def _plan(session, trial):
    record, created = save_result(session, trial)
    data = load_data(record)
    mean, reruns = walk.candidate_cost(session.state, trial)
    top = results.best(session)
    plan = {"session": session.name, "kind": session.kind, "trial": trial["n"], "record": record,
            "created": created, "data": data, "checks": [], "lines": [], "notes": [], "usage": [],
            "overwrite": False, "unchanged": False}
    plan["lines"] += [
        ("hasil", "%s -- sesi %s, percobaan %d (%s), skor %.3f%s" % (
            record["id"], session.name, trial["n"], trial["kind"], mean,
            ", TERBAIK" if top and top["trial"]["n"] == trial["n"] else "")),
        ("sumber", "%s, %s" % (results.SOURCE_TEXT[session.kind], results.target_text(session))),
        ("perubahan", results.changes_text(session, trial)),
        ("salinan hasil", "%s (%s)" % (record["data_file"], "baru disimpan" if created else "sudah tersimpan")),
        ("robot", src.robot_name()),
    ]
    if not reruns and trial["kind"] != "baseline":
        plan["notes"].append("kandidat ini BELUM diulang -- skor bagusnya bisa kebetulan (menu: Uji hasil ini di robot)")
    verdict = feedback.latest_verdicts(session.dir).get(trial["n"])
    if verdict and verdict[0] == "gagal":
        plan["notes"].append("feedback %s menyimpulkan kandidat ini GAGAL" % verdict[1]["id"])
    return plan


def _check(plan, ok, text):
    plan["checks"].append((bool(ok), text))
    return ok


def _check_studio(plan, to, target):
    data = plan["data"]
    params = data["params"]
    plan["lines"].append(("penyimpanan", "%s lewat server Studio %s" % (results.rel(src.STUDIO_VERSIONS),
                                                                        walk.STUDIO_URL)))
    try:
        store = walk.studio_request("GET")
    except (OSError, ValueError, walk.StudioRefused) as error:
        _check(plan, False, "koneksi: server Studio tidak menjawab (%s) -- nyalakan Bascorro Studio dulu. Salinan "
               "hasil tetap tersimpan, bisa dikirim lagi nanti" % error)
        return
    if not isinstance(store.get("versions"), list):
        _check(plan, False, "koneksi: %s bukan server penyimpanan versi walking" % walk.STUDIO_URL)
        return
    _check(plan, True, "koneksi: server Studio menjawab (walking_versions.json revisi %s, %d versi)"
           % (store.get("revision"), len(store["versions"])))
    current = src.current_reference_id()
    _check(plan, data["reference_id"] == current == store.get("poseReferenceId"),
           "acuan page 2: hasil %s, versi Studio %s, sekarang %s%s" % (
               data["reference_id"], store.get("poseReferenceId"), current,
               "" if data["reference_id"] == current == store.get("poseReferenceId") else
               " -- angka init pose hasil ini berarti acuan lain; latih ulang dari preset yang sudah dimigrasi"))
    known = set().union(*(set(v.get("params", {})) for v in store["versions"])) if store["versions"] else set(params)
    bad = sorted(set(params) ^ known)
    for key, value in params.items():
        if key in walk.BOOL_FIELDS:
            ok = isinstance(value, bool)
        elif key in walk.INT_FIELDS:
            ok = isinstance(value, int) and not isinstance(value, bool)
        else:
            ok = (isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                  and abs(value) < FLOAT32_MAX)
        if not ok:
            bad.append("%s=%r" % (key, value))
    _check(plan, not bad, "format: %d parameter %s" % (
        len(params), "nama & tipe sama dengan versi Studio" if not bad else "TIDAK cocok: %s" % ", ".join(bad)))
    _check(plan, wcore.leg_reachable(params), "jangkauan kaki (IK): %s" % (
        "init pose & langkah masih terjangkau" if wcore.leg_reachable(params) else "DI LUAR jangkauan kaki"))
    now_ms = int(time.time() * 1000)
    if to == "studio-new":
        name = (target or "").strip()
        clash = [v for v in store["versions"] if v.get("name") == name]
        _check(plan, name and len(name) <= 200 and not clash, "nama preset %r %s" % (
            name, "belum dipakai" if name and not clash else "SUDAH dipakai versi lain (atau kosong)"))
        ids = {v.get("id") for v in store["versions"]}
        base_id = "training-%s-n%d" % (re.sub(r"[^A-Za-z0-9_.:-]", "-", plan["session"]), plan["trial"])
        version_id, n = base_id, 2
        while version_id in ids:
            version_id, n = "%s-%d" % (base_id, n), n + 1
        plan["version"] = {"id": version_id, "name": name, "createdAt": now_ms, "updatedAt": now_ms, "params": params}
        plan["previous"] = None
        plan["target_text"] = "preset BARU %r di Bascorro Studio" % name
        plan["lines"].append(("tujuan", "%s (id %s)" % (plan["target_text"], version_id)))
    else:
        found = [v for v in store["versions"] if target in (v.get("id"), v.get("name"))]
        if not _check(plan, found, "preset tujuan %r %s" % (target, "ada" if found else "TIDAK ada lagi di Studio")):
            return
        old = found[0]
        changed = [(k, old["params"].get(k), params[k]) for k in sorted(params) if old["params"].get(k) != params[k]]
        plan["overwrite"] = True
        plan["unchanged"] = not changed
        plan["previous"] = json.loads(json.dumps(old))
        plan["version"] = {"id": old["id"], "name": old["name"], "createdAt": old.get("createdAt", now_ms),
                           "updatedAt": now_ms, "params": params}
        plan["target_text"] = "preset %r di Bascorro Studio (ditimpa)" % old["name"]
        plan["lines"].append(("tujuan", "TIMPA preset %r (id %s, terakhir disimpan %s)" % (
            old["name"], old["id"], src.local_time(old.get("updatedAt")))))
        plan["lines"].append(("yang berubah", "; ".join("%s %s -> %s" % c for c in changed) if changed
                              else "tidak ada -- preset itu sudah berisi hasil ini"))
    plan["active_before"] = store.get("activeVersionId")
    plan["revision_before"] = store.get("revision")
    name = plan["version"]["name"]
    plan["usage"].append("BELUM aktif di walking_module (penerapan tidak menjalankan robot). Memakainya: Studio -> "
                         "muat ulang halaman (F5) -> pilih %r -> Load -> Apply & Start." % name)
    roles, approach_file = src.d60_presets()
    for role in roles.get(name, []):
        if role == "approach" and approach_file:
            plan["usage"].append("D60 peran 'approach' memakai berkas beku %s, BUKAN preset Studio ini -- D60 tidak "
                                 "berubah." % os.path.basename(approach_file))
        else:
            plan["usage"].append("D60 memakai preset ini sebagai '%s': berlaku saat D60 dinyalakan berikutnya "
                                 "(preset dibaca ulang tiap sesi D60, tanpa build)." % role)
    plan["notes"].append("Studio yang SEDANG terbuka masih memegang angka lama: muat ulang (F5) sebelum menekan Save "
                         "di sana, kalau tidak hasil ini tertimpa lagi.")


def _check_page(plan, target, name):
    data = plan["data"]
    bin_path, how = src.active_action_bin()
    plan["lines"].append(("penyimpanan", "%s (%s)" % (results.rel(bin_path), how)))
    try:
        content = src.read_bin(bin_path)
    except (OSError, SystemExit) as error:
        _check(plan, False, "berkas .bin tidak terbaca: %s" % error)
        return
    _check(plan, os.access(bin_path, os.W_OK), "berkas .bin terbaca (%d page) %s" % (
        src.MAX_PAGE, "dan bisa ditulis" if os.access(bin_path, os.W_OK) else "tapi TIDAK bisa ditulis (izin berkas)"))
    pid = src.manager_pid()
    manager_bin = src._local_path((src._manager_env("OP3_ACTION_FILE") or "").strip())
    same_bin = bool(manager_bin) and os.path.realpath(manager_bin) == os.path.realpath(bin_path)
    plan["lines"].append(("action_module", ("op3_manager pid %d, memakai %s" % (pid, results.rel(manager_bin))
                                            if pid and manager_bin else
                                            "op3_manager pid %d, berkas yang dipakainya tidak terbaca" % pid
                                            if pid else "op3_manager TIDAK hidup")))
    if results.rel(bin_path) != data["bin_path"] and _abs(data["bin_path"]) != bin_path:
        plan["notes"].append("hasil dilatih dari %s, sekarang ditulis ke %s" % (data["bin_path"], results.rel(bin_path)))
    try:
        index = int(target)
    except (TypeError, ValueError):
        _check(plan, False, "nomor page tujuan %r bukan angka" % (target,))
        return
    if not _check(plan, 1 <= index < src.MAX_PAGE and index not in (src.STUDIO_SCRATCH_PAGE, src.TRAINING_SCRATCH_PAGE),
                  "page tujuan %d %s" % (index, "sah" if 1 <= index < src.MAX_PAGE and index not in (
                      src.STUDIO_SCRATCH_PAGE, src.TRAINING_SCRATCH_PAGE) else "tidak boleh (1..255, bukan page kerja "
                      "%d/%d)" % (src.STUDIO_SCRATCH_PAGE, src.TRAINING_SCRATCH_PAGE))):
        return
    if name is not None and not _check(plan, re.fullmatch(r"[ -~]{1,13}", name), "nama page %r %s" % (
            name, "sah" if re.fullmatch(r"[ -~]{1,13}", name) else "tidak sah (1..13 karakter ASCII)")):
        return
    page = src.decode_page(bytes.fromhex(data["page_hex"]), 0)
    page["index"] = index
    if page["next"] == data["page"]:      # page yang mengulang dirinya tetap mengulang dirinya
        page["next"] = index
    page_bytes = src.encode_page(page, name=name)
    modified = bytearray(content)
    modified[index * src.PAGE_SIZE:(index + 1) * src.PAGE_SIZE] = page_bytes
    errors, warnings = src.check_page(bytes(modified), index)
    written = src.decode_page(bytes(modified), index)
    _check(plan, not errors, "format page: %d step, repeat %d, checksum %s, time tiap step %s%s" % (
        written["stepnum"], written["repeat"], "benar" if written["checksum_ok"] else "SALAH",
        "/".join(str(s["time"]) for s in written["steps"]), "" if not errors else " -- " + "; ".join(errors)))
    plan["notes"] += ["page tujuan: %s" % w for w in warnings]
    existing = src.decode_page(content, index)
    names = src.motion_index()
    label = " (%s)" % names[index] if index in names else ""
    if existing["raw"] == page_bytes:
        plan["unchanged"] = True
        state_text = "sudah berisi hasil ini persis"
    elif src.page_used(existing):
        plan["overwrite"] = True
        state_text = "berisi %r, %d step, sha %s" % (existing["name"], existing["stepnum"], src.page_sha(existing))
        if index == data["page"]:
            state_text += (" = isi saat training" if src.page_sha(existing) == data["page_sha"] else
                           " -- BEDA dari isi saat training (%s): perubahan sesudahnya ikut tertimpa" % data["page_sha"])
    else:
        state_text = "kosong"
    plan["target_text"] = "page %d%s di %s" % (index, label, os.path.basename(bin_path))
    plan["lines"].append(("tujuan", plan["target_text"]))
    plan["lines"].append(("isi sekarang", state_text))
    plan["lines"].append(("nama page", written["name"] or "(tanpa nama)"))
    plan["lines"].append(("durasi", "~%.1f s" % src.page_seconds(written)))
    for other in results.sessions():
        running = other.running_pid()
        if running and other.kind == "action":
            if other.state["page"] == index:
                _check(plan, False, "sesi %s (pid %d) sedang MELATIH page %d -- tunggu sampai selesai"
                       % (other.name, running, index))
            else:
                plan["notes"].append("training sesi %s sedang berjalan (pid %d) lewat page kerja %d -- tidak "
                                     "bertabrakan, tapi jangan putar page ini sampai training selesai"
                                     % (other.name, running, src.TRAINING_SCRATCH_PAGE))
    plan.update({"bin_path": bin_path, "index": index, "page_bytes": page_bytes, "previous_hex": existing["raw"].hex()})
    if pid and same_bin:
        plan["usage"].append("AKTIF LANGSUNG: action_module (op3_manager pid %d) membuka ulang %s setiap page diputar "
                             "-- tanpa restart. Penerapan sendiri TIDAK memutar page." % (pid, os.path.basename(bin_path)))
    elif pid and manager_bin:
        plan["usage"].append("TERSIMPAN, belum aktif: op3_manager sekarang memakai %s, bukan berkas ini."
                             % results.rel(manager_bin))
    elif pid:
        plan["usage"].append("TERSIMPAN. op3_manager hidup, tapi env OP3_ACTION_FILE-nya tidak terbaca dari sini: "
                             "kalau ia memakai berkas ini, page langsung dipakai saat diputar berikutnya.")
    else:
        plan["usage"].append("TERSIMPAN di berkas: op3_manager tidak hidup, page ini dipakai saat stack robot dinyalakan.")
    if index in names:
        plan["usage"].append("Demo memakai page %d sebagai %s (op_demo.h Motion_Index)." % (index, names[index]))
    plan["usage"].append("Action Editor terminal yang terbuka: buka ulang page-nya; tab Action Studio: tekan Reload.")
    if index == 2:
        plan["usage"].append("Page 2 = acuan Init Pose walking: jalankan walking_ready_fk.py --write (Studio ditutup) "
                             "dan sync_ini_pose_from_page2.py --write.")
    if index == data["page"] and not plan["unchanged"] and data["trial_kind"] != "baseline":
        plan["notes"].append("page asal sesi ini akan berisi hasil #%d: JANGAN lanjutkan training sesi %s dengan "
                             "'isi page yang baru' (--follow-page), kenop akan terkena dua kali -- mulai sesi baru "
                             "dari page ini" % (plan["trial"], plan["session"]))


def check(session, trial, to, target, name=None):
    """Rencana penerapan + hasil pemeriksaan (tidak menulis ke robot; salinan hasil disimpan)."""
    if (session.kind == "studio") != to.startswith("studio"):
        raise SystemExit("hasil %s tidak bisa diterapkan ke %s" % (results.SOURCE_TEXT[session.kind], to))
    plan = _plan(session, trial)
    plan["to"] = to
    if session.kind == "studio":
        _check_studio(plan, to, target)
    else:
        _check_page(plan, target, name)
    plan["ok"] = all(ok for ok, _ in plan["checks"])
    plan.setdefault("target_text", "%s %s" % ("preset" if session.kind == "studio" else "page", target))
    return plan


# ---- menulis --------------------------------------------------------------------------

def _studio_backups():
    try:
        return set(os.listdir(STUDIO_BACKUP_DIR))
    except OSError:
        return set()


def _write_studio(plan):
    version = plan["version"]
    before = _studio_backups()
    walk.studio_request("POST", {"op": "upsert", "version": version})
    if plan["active_before"] != version["id"]:
        # upsert menjadikan versi itu aktif; pilihan operator di Studio dikembalikan seperti semula
        walk.studio_request("POST", {"op": "activate", "id": plan["active_before"]})
    store = walk.studio_request("GET")
    stored = next((v for v in store["versions"] if v.get("id") == version["id"]), None)
    verified = stored is not None and stored.get("name") == version["name"] and stored.get("params") == version["params"]
    created = sorted(_studio_backups() - before)
    return {"verified": verified, "revision": store.get("revision"),
            "backup": results.rel(os.path.join(STUDIO_BACKUP_DIR, created[0])) if created else None,
            "verify_text": ("dibaca balik dari server Studio: preset %r berisi %d parameter yang sama persis (revisi %s)"
                            % (version["name"], len(version["params"]), store.get("revision")) if verified else
                            "dibaca balik dari server Studio: isinya TIDAK sama dengan yang dikirim")}


def _write_page(plan):
    bin_path, index, page_bytes = plan["bin_path"], plan["index"], plan["page_bytes"]
    backup = _backup(bin_path)
    src.write_page(bin_path, index, page_bytes)
    after = src.decode_page(src.read_bin(bin_path), index)
    verified = after["raw"] == page_bytes and after["checksum_ok"]
    return {"verified": verified, "backup": results.rel(backup), "sha": src.page_sha(after),
            "verify_text": ("dibaca balik dari %s: page %d sama persis byte demi byte, checksum benar (sha %s)"
                            % (os.path.basename(bin_path), index, src.page_sha(after)) if verified else
                            "dibaca balik: page %d TIDAK sama dengan yang ditulis" % index)}


def _target_record(plan):
    if plan["kind"] == "studio":
        return {"kind": "studio", "id": plan["version"]["id"], "name": plan["version"]["name"],
                "new": plan["to"] == "studio-new"}
    return {"kind": "page", "bin": results.rel(plan["bin_path"]), "page": plan["index"],
            "name": src.decode_page(plan["page_bytes"], 0)["name"]}


def deploy(session, trial, to, target, name=None, overwrite=False):
    """Periksa -> cadangan -> tulis -> baca balik -> catat. Mengembalikan (rencana, catatan penerapan)."""
    plan = check(session, trial, to, target, name)
    if plan["ok"] and plan["overwrite"] and not plan["unchanged"] and not overwrite:
        raise SystemExit("%s sudah berisi -- menimpanya butuh konfirmasi (menu: ketik YA; CLI: --overwrite). "
                         "Tidak ada yang ditulis." % plan["target_text"])
    entry = {"at": feedback.now_iso(), "action": "deploy", "robot": src.robot_name(), "to": to,
             "target_text": plan["target_text"], "status": "gagal", "verified": False}
    failed = [text for ok, text in plan["checks"] if not ok]
    if failed:
        entry["message"] = "pemeriksaan gagal, tidak ada yang ditulis: " + "; ".join(failed)
    elif plan["unchanged"]:
        entry.update(status="sama", target=_target_record(plan), usage=plan["usage"],
                     message="target sudah berisi hasil ini persis -- tidak ada yang ditulis")
    else:
        entry["target"] = _target_record(plan)
        try:
            outcome = _write_studio(plan) if plan["kind"] == "studio" else _write_page(plan)
        except (OSError, ValueError, walk.StudioRefused) as error:
            entry["message"] = "penulisan gagal: %s -- salinan hasil tetap tersimpan, bisa dicoba lagi" % error
        else:
            entry.update(status="berhasil" if outcome["verified"] else "gagal", verified=outcome["verified"],
                         backup=outcome.get("backup"), message=outcome["verify_text"], usage=plan["usage"])
            if plan["kind"] == "studio":
                entry.update(revision=outcome["revision"], previous=plan["previous"], written=plan["version"])
            else:
                entry.update(previous_hex=plan["previous_hex"], written_sha=outcome["sha"])
    plan["record"] = _append_entry(session, plan["record"]["id"], entry)
    return plan, entry


def undo(session, trial):
    """Kembalikan target penerapan berhasil terakhir ke isi sebelumnya -- hanya kalau target
    masih berisi persis hasil ini (perubahan orang lain sesudahnya tidak ditimpa)."""
    record = saved_record(session, trial["n"])
    last = last_success(record) if record else None
    if last is None:
        raise SystemExit("hasil ini belum pernah berhasil diterapkan -- tidak ada yang dikembalikan")
    target = last["target"]
    entry = {"at": feedback.now_iso(), "action": "undo", "robot": src.robot_name(), "to": last["to"],
             "target_text": last["target_text"], "target": target, "status": "gagal", "verified": False}
    try:
        if target["kind"] == "page":
            bin_path = _abs(target["bin"])
            now = src.decode_page(src.read_bin(bin_path), target["page"])
            if src.page_sha(now) != last["written_sha"]:
                raise SystemExit("page %d sudah diubah lagi sesudah penerapan (%s, bukan %s) -- tidak dikembalikan "
                                 "otomatis. Cadangan .bin sebelum penerapan: %s"
                                 % (target["page"], src.page_sha(now), last["written_sha"], last.get("backup")))
            previous = bytes.fromhex(last["previous_hex"])
            backup = _backup(bin_path)
            src.write_page(bin_path, target["page"], previous)
            entry.update(status="berhasil", verified=True, backup=results.rel(backup),
                         message="page %d dikembalikan ke isi sebelum penerapan (dibaca balik sama persis)"
                                 % target["page"])
        else:
            store = walk.studio_request("GET")
            stored = next((v for v in store["versions"] if v.get("id") == target["id"]), None)
            if stored is not None and stored.get("params") != last["written"]["params"]:
                raise SystemExit("preset %r sudah disimpan ulang sesudah penerapan -- tidak dikembalikan otomatis"
                                 % target["name"])
            active = store.get("activeVersionId")
            if target["new"]:
                if stored is not None:
                    walk.studio_request("POST", {"op": "delete", "id": target["id"]})
                check_store = walk.studio_request("GET")
                gone = all(v.get("id") != target["id"] for v in check_store["versions"])
                entry.update(status="berhasil" if gone else "gagal", verified=gone,
                             message="preset %r dihapus lagi dari Studio" % target["name"] if gone
                             else "preset %r masih ada sesudah dihapus" % target["name"])
            else:
                previous = dict(last["previous"], updatedAt=int(time.time() * 1000))
                walk.studio_request("POST", {"op": "upsert", "version": previous})
                if active != previous["id"]:
                    walk.studio_request("POST", {"op": "activate", "id": active})
                check_store = walk.studio_request("GET")
                back = next((v for v in check_store["versions"] if v.get("id") == target["id"]), None)
                ok = back is not None and back.get("params") == last["previous"]["params"]
                entry.update(status="berhasil" if ok else "gagal", verified=ok,
                             message="preset %r dikembalikan ke isi sebelum penerapan%s" % (
                                 target["name"], " (dibaca balik sama persis)" if ok else " -- TIDAK terverifikasi"))
    except (OSError, ValueError, walk.StudioRefused) as error:
        entry["message"] = "gagal mengembalikan: %s" % error
    _append_entry(session, record["id"], entry, mark_undone=last["at"] if entry["status"] == "berhasil" else None)
    return entry


# ---- ekspor salinan --------------------------------------------------------------------

def bundle(session, trial, out_dir=None):
    """Salinan mandiri satu hasil (berkas data, catatan, feedback) ke folder lain -- untuk
    diarsipkan, dibandingkan, atau dibawa ke robot lain. Mengembalikan (folder, berkas)."""
    record, _ = save_result(session, trial)
    record = saved_record(session, trial["n"])
    folder = os.path.join(out_dir or BUNDLE_DIR, record["id"])
    os.makedirs(folder, exist_ok=True)
    written = []
    for path in record["files"]:
        shutil.copy2(_abs(path), folder)
        written.append(os.path.join(folder, os.path.basename(path)))
    notes = feedback.notes(session.dir, trial["n"])
    _write_json(os.path.join(folder, "hasil.json"), dict(record, feedback=notes))
    with open(os.path.join(folder, "feedback.md"), "w", encoding="utf-8") as handle:
        handle.write("# Feedback %s\n\n%s, %s\n\n" % (record["id"], results.SOURCE_TEXT[session.kind],
                                                       record["source_target"]))
        if not notes:
            handle.write("(belum ada feedback)\n")
        for note in notes:
            handle.write("## %s · %s%s%s\n\n%s\n\n" % (
                src.local_time(note.get("time")), note.get("where") or "-",
                " · kesimpulan: %s" % feedback.VERDICT_TEXT.get(note["verdict"], note["verdict"])
                if note.get("verdict") else "",
                " · diuji di: %s" % note["tested_on"] if note.get("tested_on") else "", note["text"]))
    with open(os.path.join(folder, "BACA.txt"), "w", encoding="utf-8") as handle:
        if session.kind == "studio":
            handle.write("Hasil training preset Bascorro Studio. Parameter lengkap ada di %s (\"params\").\n"
                         "Di robot asal: ./script.sh -> Training -> Riwayat -> hasil ini -> Simpan langsung ke robot.\n"
                         "Di robot lain: salin angka \"params\" ke tab Walking Studio lalu Save sebagai versi baru;\n"
                         "angka init pose hanya berlaku untuk acuan page 2 %s.\n"
                         % (os.path.basename(record["data_file"]), record.get("reference_id") or
                            load_data(record)["reference_id"]))
        else:
            yaml_name = next(os.path.basename(p) for p in record["files"] if p.endswith(".yaml"))
            handle.write("Hasil training page Action Editor (format action_yaml.py).\nImpor dari akar workspace "
                         "(cadangan .bin otomatis):\n  %s\nAtau di robot asal: ./script.sh -> Training -> Riwayat -> "
                         "hasil ini -> Simpan langsung ke robot.\n" % import_command("<berkas .bin>", yaml_name))
    written += [os.path.join(folder, n) for n in ("hasil.json", "feedback.md", "BACA.txt")]
    return folder, written


# ---- teks untuk terminal ----------------------------------------------------------------

def plan_text(plan):
    out = ["Rencana penerapan ke robot (belum ada yang ditulis):"]
    out += ["  %-14s: %s" % (k, v) for k, v in plan["lines"]]
    out.append("  pemeriksaan:")
    out += ["    %s %s" % ("✔" if ok else "✘", text) for ok, text in plan["checks"]]
    if plan["ok"]:
        out.append("  cadangan     : %s" % ("salinan .bin utuh <berkas>.bak.<waktu> sebelum menulis; isi lama page "
                                         "dicatat untuk dikembalikan" if plan["kind"] == "action" else
                                         "server Studio menyalin walking_versions.json ke data/backup/ tiap tulis; "
                                         "isi lama preset dicatat untuk dikembalikan"))
        out += ["  sesudahnya   : %s" % u for u in plan["usage"]]
    out += ["  catatan      : %s" % n for n in plan["notes"]]
    if plan["unchanged"]:
        out.append("  => target sudah berisi hasil ini persis; tidak ada yang perlu ditulis.")
    elif not plan["ok"]:
        out.append("  => TIDAK bisa diterapkan; perbaiki yang bertanda ✘. Salinan hasil tetap tersimpan.")
    elif plan["overwrite"]:
        out.append("  => target SUDAH BERISI: menimpanya butuh konfirmasi YA.")
    return "\n".join(out)


def entry_text(entry):
    head = {"berhasil": "✔ BERHASIL", "sama": "= TIDAK ADA PERUBAHAN", "gagal": "✘ GAGAL"}[entry["status"]]
    what = "Pengembalian" if entry["action"] == "undo" else "Penerapan"
    out = ["%s %s -> %s" % (what, head, entry["target_text"]), "  %s" % entry.get("message", "")]
    if entry.get("backup"):
        out.append("  cadangan: %s" % entry["backup"])
    if entry["status"] == "berhasil" and entry["action"] == "deploy":
        out.append("  status: TERSIMPAN di robot dan terverifikasi.")
        out += ["  %s" % u for u in entry.get("usage", [])]
    elif entry["status"] == "gagal":
        out.append("  Salinan hasil tetap tersimpan; bisa dikirim lagi dari Riwayat hasil.")
    return "\n".join(out)

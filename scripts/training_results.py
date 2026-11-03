"""Hasil training: daftar sesi (Studio & Action Editor), status, dan hasil terbaik
beserta alasannya. MURNI (tanpa ROS) -- dipakai scripts/training.py. Menyimpan dan
menerapkan hasil ke robot: training_deploy.py; feedback: training_feedback.py.

Sesi kedua sumber tinggal di folder yang sama (src/bascorro_studio/data/autotune/);
jenisnya dikenali dari "format" di state.json.

Hasil terbaik = kandidat SAH (tidak jatuh/gagal/hampir menurut detektor DAN
label operator) dengan skor rata-rata terendah; skor kandidat yang diulang
(--rerun-best) adalah rata-rata semua run-nya. Skor ~1,0 = sama dengan preset/
page asli, jadi "lebih baik" berarti di bawah rata-rata baseline. Kandidat yang
oleh feedback operator disimpulkan "gagal" dilewati (alasannya disebut).
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autotune_action as act  # noqa: E402
import autotune_action_core as acore  # noqa: E402
import autotune_walking as walk  # noqa: E402
import autotune_walking_core as wcore  # noqa: E402
import training_feedback as feedback  # noqa: E402
import training_iteration as iteration  # noqa: E402
import training_sources as src  # noqa: E402

SOURCE_TEXT = {"studio": "Bascorro Studio", "action": "Action Editor"}
_SESSION_CLASS = {walk.STATE_FORMAT: ("studio", walk.Session), act.STATE_FORMAT: ("action", act.ActionSession)}


# ---- sesi -------------------------------------------------------------------------

def sessions():
    """Semua sesi training, terbaru dulu."""
    found = []
    root = walk.SESSIONS_DIR
    for name in sorted(os.listdir(root)) if os.path.isdir(root) else []:
        path = os.path.join(root, name, "state.json")
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8") as handle:
                state = json.load(handle)
        except (OSError, ValueError):
            continue
        if state.get("format") in _SESSION_CLASS:
            found.append(_wrap(name, state))
    return sorted(found, key=lambda s: last_activity(s), reverse=True)


def _wrap(name, state):
    kind, cls = _SESSION_CLASS[state["format"]]
    session = cls(name)
    session.state = state
    session.kind = kind
    return session


def open_session(name):
    session = walk.Session(name)
    if not session.exists():
        raise SystemExit("sesi %r tidak ada di %s" % (name, walk.SESSIONS_DIR))
    with open(session.path, encoding="utf-8") as handle:
        state = json.load(handle)
    if state.get("format") not in _SESSION_CLASS:
        raise SystemExit("%s bukan sesi training" % session.path)
    return _wrap(name, state)


def last_activity(session):
    times = [t.get("time") for t in session.state["trials"] if t.get("time")]
    return max(times) if times else session.state.get("createdAt", "")


def target_text(session):
    state = session.state
    if session.kind == "studio":
        return "preset %r" % state["start_name"]
    return state["start_name"]


def matches_target(session, kind, key):
    """Sesi milik preset Studio (id/nama) atau page Action Editor (nomor) ini?"""
    if session.kind != kind:
        return False
    state = session.state
    if kind == "studio":
        return key in (state.get("start_id"), state["start_name"])
    return str(state["page"]) == str(key)


def cma_done(state):
    return sum(1 for t in state["trials"] if t["kind"] == "cma")


def status(session):
    state = session.state
    pid = session.running_pid()
    done, budget = cma_done(state), state["budget"]
    if pid:
        return "BERJALAN (pid %d, %d/%d)" % (pid, done, budget)
    if not state["trials"]:
        return "belum ada percobaan"
    if done >= budget:
        return "selesai (%d/%d percobaan CMA)" % (done, budget)
    return "terhenti di %d/%d percobaan CMA -- bisa dilanjutkan" % (done, budget)


def stale_reason(session):
    """Kenapa sesi tidak bisa langsung dilanjutkan (None = bisa)."""
    state = session.state
    if session.kind == "studio":
        if state.get("metricsVersion") != wcore.METRICS_VERSION:
            return "rescore", "rumus skor sudah berubah -- hitung ulang dulu (rescore)"
        if src.current_reference_id() != state["reference_id"]:
            return "follow", "page 2 sudah di-teach ulang sejak sesi ini dimulai"
        return None
    if state.get("metricsVersion") != acore.METRICS_VERSION:
        return "rescore", "rumus skor sudah berubah -- hitung ulang dulu (rescore)"
    reason = act.page_changed(state)
    return ("follow", reason) if reason else None


def criteria(session):
    weights = session.state.get("weights")
    if session.kind == "studio":
        return wcore.criteria_text(session.state.get("motion", "maju"), weights)
    return acore.criteria_text(session.state["profile"], weights)


def metric_text(session):
    return wcore.METRIC_TEXT if session.kind == "studio" else acore.METRIC_TEXT


def is_usable(session):
    return walk.usable if session.kind == "studio" else act.usable_for(session.state)


def ranked(session):
    """Kandidat sah (baseline + CMA + iterasi feedback), terbaik dulu: (skor rata-rata, ulangan, trial)."""
    check = is_usable(session)
    out = []
    for trial in session.state["trials"]:
        if trial["kind"] in walk.CANDIDATE_KINDS and check(trial):
            mean, reruns = walk.candidate_cost(session.state, trial)
            out.append((mean, reruns, trial))
    return sorted(out, key=lambda item: item[0])


def rejected_by_feedback(session):
    """{kandidat: catatan} yang kesimpulan feedback TERAKHIR-nya "gagal"."""
    return {n: note for n, (verdict, note) in feedback.latest_verdicts(session.dir).items() if verdict == "gagal"}


def best(session):
    """Hasil terbaik + alasan pemilihannya, atau None kalau belum ada yang sah."""
    state = session.state
    candidates = ranked(session)
    rejected = rejected_by_feedback(session)
    skipped = [t["n"] for _, _, t in candidates if t["n"] in rejected]
    candidates = [c for c in candidates if c[2]["n"] not in rejected]
    if not candidates:
        return None
    mean, reruns, trial = candidates[0]
    base = [t["cost"] for t in walk.baseline_trials(state, is_usable(session))]
    base_mean = sum(base) / len(base) if base else None
    total = len(state["trials"])
    reason = ["skor terendah (%.3f) di antara %d kandidat sah dari %d percobaan" % (mean, len(candidates), total)]
    if skipped:
        reason.append("dilewati karena feedback operator menyimpulkan GAGAL: %s"
                      % ", ".join("#%d (%s)" % (n, rejected[n]["id"]) for n in skipped))
    if trial["kind"] == "baseline":
        reason.append("ITU preset/page ASLI: belum ada kandidat yang lebih baik dari aslinya")
    elif base_mean:
        reason.append("%.0f%% lebih baik dari rata-rata baseline %.3f" % (100 * (1 - mean / base_mean), base_mean))
    reason.append("sudah diulang %d kali (skor = rata-rata semua run)" % reruns if reruns
                  else "BELUM diulang -- jalankan 'ulangi terbaik' sebelum dipakai di demo")
    return {"trial": trial, "mean": mean, "reruns": reruns, "baseline_mean": base_mean,
            "is_baseline": trial["kind"] == "baseline", "reason": "; ".join(reason)}


def trial_params(session, trial):
    """Nilai lengkap kandidat: 29 parameter walking, atau kenop page (+ kenop fase dari feedback)."""
    return iteration.full_params(session.state, trial)


def changes_text(session, trial):
    state = session.state
    full = trial_params(session, trial)
    parts = []
    for key in state["params"] + [k for k in full if k not in state["params"]]:
        before = state["start_params"].get(key, iteration._neutral(key))
        if abs(full[key] - before) > 1e-9:
            parts.append("%s %g -> %g" % (key, before, full[key]))
    return ", ".join(parts) or "(sama dengan aslinya)"


def metrics_text(session, trial):
    m = trial.get("metrics") or {}
    keys = wcore.WEIGHTS if session.kind == "studio" else acore.PROFILES[session.state["profile"]]["weights"]
    text = ", ".join("%s %.2f" % (k, m[k]) for k in keys if m.get(k) is not None)
    if session.kind == "studio" and m.get("yaw_drift_deg") is not None:
        text += " (arah %s)" % wcore.turn_text(m["yaw_drift_deg"])
    return text


def rel(path):
    """Path untuk ditampilkan/dicatat: relatif ke akar workspace kalau di dalamnya."""
    path = os.path.abspath(path)
    return os.path.relpath(path, walk.HERE) if path.startswith(walk.HERE + os.sep) else path


def session_files(session):
    raw = os.path.join(session.dir, "raw")
    export = os.path.join(session.dir, "export")
    return {"dir": rel(session.dir),
            "raw": len(os.listdir(raw)) if os.path.isdir(raw) else 0,
            "export": sorted(os.listdir(export)) if os.path.isdir(export) else [],
            "feedback": len(feedback.notes(session.dir))}


# ---- kandidat ----------------------------------------------------------------------

def find_trial(session, which):
    """'best' / '' = terbaik, selain itu nomor percobaan (harus kandidat sah) -- untuk disimpan/diterapkan."""
    if which in (None, "", "best"):
        top = best(session)
        if not top:
            raise SystemExit("sesi %r belum punya hasil yang sah" % session.name)
        return top["trial"]
    trial = next((t for t in session.state["trials"] if str(t["n"]) == str(which)), None)
    if trial is None:
        raise SystemExit("percobaan %s tidak ada di sesi %r" % (which, session.name))
    if not is_usable(session)(trial):
        raise SystemExit("percobaan %s jatuh/gagal/tidak lengkap -- tidak bisa disimpan/diterapkan" % which)
    return trial


def tuned_page(session, trial, target=None, name=None):
    """(page dict hasil, 512 byte) -- page snapshot sesi + kenop kandidat."""
    state = session.state
    original = act.snapshot_page(state)
    page = acore.apply_knobs(original, trial_params(session, trial))
    target = state["page"] if target is None else target
    page["index"] = target
    if page["next"] == state["page"]:     # page yang mengulang dirinya tetap mengulang dirinya
        page["next"] = target
    return page, src.encode_page(page, name=name)

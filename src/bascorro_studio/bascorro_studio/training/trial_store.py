"""Penyimpanan percobaan yang sudah dinilai operator -- MURNI.

Satu berkas JSON {"trials": [...]} per tab:
  rl_runs/studio/trials.json  tab RL Training     (percobaan wajib berisi "params")
  rl_runs/action/trials.json  tab Action Training (percobaan wajib berisi "values")

Disimpan di ROBOT, bukan localStorage (localStorage dipisah per origin: localhost
dan IP robot melihat isi berbeda). Nomor percobaan dihitung per sesi DI SINI, jadi
dua peramban yang terbuka bersamaan tidak memberi nomor kembar. Tulis = berkas
sementara lalu os.replace, supaya berkas tidak pernah setengah jadi.
"""

import json
import os
import time


class TrialStore:
    def __init__(self, path, required_field):
        self.path = path
        self.required_field = required_field

    def load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return []
        trials = data.get("trials") if isinstance(data, dict) else None
        return trials if isinstance(trials, list) else []

    def save(self, trials):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump({"trials": trials}, handle, indent=2)
        os.replace(tmp, self.path)

    def add(self, trial):
        """Simpan satu percobaan; nomor = 1 + nomor terbesar di sesinya. -> (percobaan, semua)."""
        if not isinstance(trial, dict) or not isinstance(trial.get(self.required_field), dict):
            raise ValueError("trial harus berisi %s" % self.required_field)
        trials = self.load()
        session = str(trial.get("session") or "default")
        trial["session"] = session
        trial["no"] = 1 + max((int(t.get("no", 0)) for t in trials if t.get("session") == session), default=0)
        trial.setdefault("at", int(time.time() * 1000))
        trials.append(trial)
        self.save(trials)
        return trial, trials

    def delete(self, trial_id):
        """Hapus menurut id -> semua yang tersisa. LookupError kalau id tidak ada."""
        trials = self.load()
        kept = [t for t in trials if t.get("id") != trial_id]
        if len(kept) == len(trials):
            raise LookupError("percobaan tidak ditemukan")
        self.save(kept)
        return kept

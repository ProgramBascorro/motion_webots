"""Feedback manual operator untuk hasil training (Bascorro Studio & Action Editor).

MURNI (tanpa ROS) -- dipakai pelatih (autotune_walking.py / autotune_action.py,
tombol "c" saat memberi label) dan menu (scripts/training.py).

PENYIMPANAN
  <sesi>/feedback.jsonl, satu catatan JSON per baris, HANYA DITAMBAH (tidak
  pernah ditulis ulang). Pelatih memegang state.json selama training dan
  menulisnya ulang tiap percobaan; catatan di berkas tersendiri tidak bisa
  tertimpa olehnya, dan dua penulis (pelatih + menu) cukup menambah baris.
  Tiap catatan menyebut KANDIDAT yang dinilai ("candidate" = nomor percobaan
  kandidat; ulangan --rerun-best ikut ke kandidat aslinya) dan run yang
  benar-benar dilihat operator ("trial"), plus di mana diuji (tested_on).

INPUT
  Teks bebas, boleh beberapa baris/paragraf dan boleh ditempel (paste):
  Enter = baris baru, baris berisi titik saja "." = selesai, "!batal" atau
  Ctrl+C = batal. Sesudahnya isi ditampilkan lagi dan dikonfirmasi.

PEMANFAATAN (apa adanya -- alat ini TIDAK memahami bahasa):
  1. Dokumentasi: semua catatan tampil di riwayat hasil, ikut tersimpan di
     ekspor, dan tampil lagi sebelum hasil diterapkan ke robot.
  2. Kesimpulan "gagal" pada satu kandidat = kandidat itu tidak lagi dipilih
     sebagai TERBAIK (alasannya disebut). Skor dan label n/j/h tidak diubah.
  3. Rencana iterasi berikutnya (plan): kalimat dicocokkan dengan tabel RULES
     di bawah (kata kunci/regex). Tiap aturan yang cocok menaikkan bobot metrik
     tertentu dan/atau menambah parameter yang dicari; frasa pemicu, aturan, dan
     angka sebelum-sesudah dicatat di sesi baru (state.json "feedback_plan").
     Kalimat yang tidak cocok dengan aturan apa pun hanya jadi dokumentasi --
     daftar itu juga ditampilkan, supaya operator tahu mana yang TIDAK dipakai.
"""

import datetime
import json
import os
import re

FEEDBACK_FILE = "feedback.jsonl"
DONE_LINE = "."
CANCEL_LINE = "!batal"
VERDICTS = {"b": "baik", "p": "perbaikan", "g": "gagal"}
VERDICT_TEXT = {"baik": "baik, sesuai harapan", "perbaikan": "jalan, tapi perlu perbaikan",
                "gagal": "GAGAL / jangan dipakai"}
PLAN_FORMAT = "bascorro-studio/training-plan"
MAX_FACTOR = 2.0       # bobot satu metrik paling banyak dinaikkan 2x oleh seluruh aturan


def now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


# ---- simpan / baca ----------------------------------------------------------------

def feedback_path(session_dir):
    return os.path.join(session_dir, FEEDBACK_FILE)


def notes(session_dir, candidate=None):
    """Semua catatan sesi (urutan ditulis). Baris rusak dilewati, tidak membatalkan yang lain."""
    found = []
    try:
        with open(feedback_path(session_dir), encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    note = json.loads(line)
                except ValueError:
                    continue
                if isinstance(note, dict) and isinstance(note.get("text"), str):
                    found.append(note)
    except OSError:
        return []
    if candidate is not None:
        found = [n for n in found if str(n.get("candidate", n.get("trial"))) == str(candidate)]
    return found


def add_note(session_dir, session_name, candidate, text, verdict=None, where="riwayat hasil", run=None,
             tested_on=None, result_id=None):
    """Tambah satu catatan, lalu BACA BALIK -- 'tersimpan' hanya kalau isinya utuh di disk."""
    text = text.encode("utf-8", "replace").decode("utf-8").strip("\n")
    if not text.strip():
        raise ValueError("feedback kosong")
    if verdict not in (None, *VERDICTS.values()):
        raise ValueError("kesimpulan %r tidak dikenal" % verdict)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    note = {"id": "fb-%s-%s" % (stamp, os.urandom(2).hex()), "time": now_iso(), "session": session_name,
            "candidate": int(candidate), "trial": int(run if run is not None else candidate),
            "verdict": verdict, "where": where, "tested_on": tested_on, "result_id": result_id, "text": text}
    data = (json.dumps(note, ensure_ascii=False) + "\n").encode("utf-8")
    os.makedirs(session_dir, exist_ok=True)
    fd = os.open(feedback_path(session_dir), os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    try:
        view = memoryview(data)
        while view:
            view = view[os.write(fd, view):]
        os.fsync(fd)
    finally:
        os.close(fd)
    if not any(n.get("id") == note["id"] and n["text"] == text for n in notes(session_dir)):
        raise OSError("feedback tidak terbaca balik utuh dari %s" % feedback_path(session_dir))
    return note


def latest_verdicts(session_dir):
    """{kandidat: (kesimpulan terakhir, catatan)} -- hanya catatan yang memberi kesimpulan."""
    out = {}
    for note in notes(session_dir):
        if note.get("verdict"):
            out[int(note.get("candidate", note.get("trial")))] = (note["verdict"], note)
    return out


def note_text(note, indent="    "):
    """Satu catatan untuk dibaca di terminal: kepala + seluruh teks (tidak dipotong)."""
    from training_sources import local_time
    head = "%s · %s · %s" % (local_time(note.get("time")), note.get("where") or "-", note.get("id"))
    if note.get("trial") not in (None, note.get("candidate")):
        head += " · run #%s" % note["trial"]
    if note.get("verdict"):
        head += " · kesimpulan: %s" % VERDICT_TEXT.get(note["verdict"], note["verdict"])
    if note.get("tested_on"):
        head += " · diuji di: %s" % note["tested_on"]
    body = "\n".join(indent + "  " + line for line in note["text"].splitlines())
    return "%s%s\n%s" % (indent, head, body)


# ---- input terminal --------------------------------------------------------------

HINTS = ("Yang membantu ditulis: kondisi & perilaku robot, bagian yang sudah berhasil, "
         "masalah yang muncul dan KAPAN, perbaikan yang diharapkan.")


def read_text(read=input, write=print):
    """Teks bebas beberapa baris. None = dibatalkan. Ctrl+D = selesai seperti '.'."""
    try:
        import readline  # noqa: F401  (panah kiri/kanan untuk menyunting baris yang sedang diketik)
    except ImportError:
        pass
    write("  Ketik feedback. Boleh beberapa baris/paragraf, boleh ditempel (paste).")
    write("    Enter            = baris baru")
    write("    baris berisi  %s  = SELESAI menulis (lalu dikonfirmasi)" % DONE_LINE)
    write("    %s  atau Ctrl+C = BATAL, tidak ada yang disimpan" % CANCEL_LINE)
    write("  " + HINTS)
    lines = []
    while True:
        try:
            line = read("  > ")
        except EOFError:
            break
        except KeyboardInterrupt:
            write("")
            return None
        if line.strip() == DONE_LINE:
            break
        if line.strip().lower() == CANCEL_LINE:
            return None
        lines.append(line.rstrip("\r"))
    text = "\n".join(lines).strip("\n")
    return text if text.strip() else None


def _ask(prompt, allowed, read):
    while True:
        try:
            answer = read(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            return None
        if answer in allowed:
            return answer


def prompt_note(session_dir, session_name, candidate, where, run=None, tested_on=None, result_id=None,
                ask_verdict=True, read=input, write=print):
    """Tulis -> (kesimpulan) -> tampilkan ulang -> simpan/ulangi/batal. Mengembalikan catatan atau None."""
    while True:
        text = read_text(read, write)
        if text is None:
            write("  Dibatalkan -- tidak ada feedback yang disimpan.")
            return None
        verdict = None
        if ask_verdict:
            key = _ask("  Kesimpulan (boleh kosong): b = baik, p = perlu perbaikan, g = GAGAL/jangan dipakai, "
                       "Enter = tanpa kesimpulan: ", {"", "b", "p", "g"}, read)
            if key is None:
                write("  Dibatalkan -- tidak ada feedback yang disimpan.")
                return None
            verdict = VERDICTS.get(key)
        write("\n  ---- feedback untuk kandidat #%s (%d baris, %d karakter)%s ----"
              % (candidate, len(text.splitlines()), len(text),
                 ", kesimpulan: %s" % VERDICT_TEXT[verdict] if verdict else ""))
        for line in text.splitlines():
            write("  | " + line)
        write("  ----")
        answer = _ask("  Enter = SIMPAN, u = tulis ulang dari awal, b = BATAL: ", {"", "u", "b"}, read)
        if answer == "u":
            continue
        if answer != "":
            write("  Dibatalkan -- tidak ada feedback yang disimpan.")
            return None
        note = add_note(session_dir, session_name, candidate, text, verdict, where, run, tested_on, result_id)
        write("  ✔ feedback tersimpan (%s) di %s" % (note["id"], feedback_path(session_dir)))
        return note


# ---- memahami teks feedback ------------------------------------------------------------
#
# Teks bebas -> PEMAHAMAN terstruktur: per kalimat, perilaku yang dikeluhkan (dengan
# frasa pemicunya, sisi kiri/kanan, arah yang diinginkan), fase gerakan, bagian
# yang sudah baik, dan potongan yang tidak dipahami. Ini pencocokan pola bahasa
# Indonesia sehari-hari (termasuk salah ketik yang sudah pernah muncul di catatan
# operator: "kebalakang", "kedepan", "mendangak"), BUKAN model bahasa: kalimat
# di luar tabel di bawah dilaporkan sebagai "tidak dipahami", tidak ditebak.
# Yang menerjemahkan pemahaman ke variabel gerakan: training_iteration.py (butuh
# konteks preset/page).

NEGATIONS = {"tidak", "tak", "nggak", "gak", "ga", "enggak", "bukan", "tanpa", "jangan"}
_TO = r"(?:ke\s*)?(?:arah\s+)?"
_BACK = r"b[ae]l[ae]kang"           # belakang + salah ketik "balakang"/"belekang"
_LEAN = (r"(?:miring|condong|doyong|jatuh|terjatuh|rebah|terjengkang|terdorong|terjungkal|tumbang|ambruk|roboh|"
         r"oleng|berat)")
_BODY = r"(?:\s+(?:badan|tubuh|robot)(?:\s*nya)?)?"
_NOT = r"(?:tidak|tak|ga|gak|nggak|enggak|ngga|belum)"

# mode: "keluhan" = yang terjadi (arah yang diinginkan = sebaliknya), "ingin" = arah
# yang diminta. Keluhan yang didahului kata ingkar ("sudah TIDAK goyah") dicatat
# sebagai hal yang sudah baik. generic = dibuang kalau kalimatnya sudah punya perilaku berarah
# ("jatuh" + "jatuh ke belakang" = satu keluhan berarah). Urutan penting: pola
# yang lebih khusus lebih dulu; pola sesudahnya yang tumpang tindih dibuang.
BEHAVIORS = [
    {"id": "condong_samping", "meaning": "badan condong / berat ke samping",
     "patterns": [(r"\bkurang\s+(?:miring|condong|geser|bergeser|pindah|berpindah|ke)?\s*" + _TO
                   + r"(?P<side>kiri|kanan)\b", "ingin"),
                  (r"\b(?:harus|perlu|lebih|agar|supaya|mestinya)\s+(?:\w+\s+)?(?:miring|condong|geser|bergeser|ke)\s+"
                   + _TO + r"(?P<side>kiri|kanan)\b", "ingin"),
                  (_LEAN + _BODY + r"\s+" + _TO + r"(?P<side>kiri|kanan)\b", "keluhan")]},
    {"id": "condong_belakang", "meaning": "badan condong / berat / jatuh ke belakang", "want": "depan",
     "patterns": [(r"\b" + _LEAN + _BODY + r"\s+" + _TO + _BACK + r"\b", "keluhan"),
                  (r"\b(?:men)?d[ao]ng[ae]?k\b|\bndangak\b|\bmenengadah\b|\bterjengkang\b", "keluhan"),
                  (r"\bkurang\s+(?:condong\s+|maju\s+)?(?:ke\s*)?depan\b", "ingin")]},
    {"id": "condong_depan", "meaning": "badan condong / berat / jatuh ke depan", "want": "belakang",
     "patterns": [(r"\b" + _LEAN + _BODY + r"\s+" + _TO + r"depan\b", "keluhan"),
                  (r"\bnyungsep\b|\btersungkur\b|\b(?:me)?nunduk\b|\bnungging\b", "keluhan"),
                  (r"\bkurang\s+(?:condong\s+)?(?:ke\s*)?" + _BACK + r"\b", "ingin")]},
    {"id": "kaki_mentok", "meaning": "kaki mentok / menyeret / kurang terangkat",
     "patterns": [(r"\bmentok\b|\bmenyentuh\s+(?:tanah|lantai)\b|\bkena\s+(?:tanah|lantai)\b", "keluhan"),
                  (r"\b(?:menyeret|terseret|nyeret|tersandung|kesandung|menggesek|tergesek|tersangkut|nyangkut)\b",
                   "keluhan"),
                  (r"\b(?:kurang|" + _NOT[3:-1] + r")\s+(?:ter)?angkat\b", "keluhan"),
                  (r"\bangkat(?:an)?\s+kaki(?:\s*nya)?\s+(?:\w+\s+)?(?:kurang|rendah|pendek)\b", "keluhan")]},
    {"id": "tidak_menapak", "meaning": "telapak tidak menapak rata",
     "patterns": [(r"\b(?:" + _NOT[3:-1] + r"|kurang)\s+(?:\w+\s+)?menapak\b", "keluhan"),
                  (r"\bjinjit\b|\bbertumpu\s+(?:di\s+|pada\s+)?(?:tumit|ujung)\b", "keluhan")]},
    {"id": "goyah", "meaning": "badan goyah / kurang stabil",
     "patterns": [(r"\bgoyah\b|\b(?:ber)?goyang\b|\bgemetar\b|\bbergetar\b|\bterhuyung\b|\blimbung\b|"
                   r"\bsempoyongan\b|\bkehilangan\s+keseimbangan\b", "keluhan"),
                  (r"\b(?:" + _NOT[3:-1] + r"|kurang)\s+(?:stabil|seimbang)\b", "keluhan"),
                  (r"\b(?:lebih|makin|agar|supaya|perbaiki|tingkatkan)\s+(?:\w+\s+)?(?:stabil|kestabilan|keseimbangan|"
                   r"seimbang)\b", "ingin")]},
    {"id": "kasar", "meaning": "terlalu kuat / menyentak / kurang halus",
     "patterns": [(r"\b(?:terlalu|kelewat|kebanyakan)\s+(?:kuat|keras|kencang|mendadak|agresif)\b", "keluhan"),
                  (r"\b(?:menyentak|menghentak|hentakan|tersentak|sentakan|mendadak|terbanting|membanting|kasar)\b",
                   "keluhan"),
                  (r"\b(?:kurang|" + _NOT[3:-1] + r")\s+(?:halus|smooth|mulus|lembut)\b", "keluhan"),
                  (r"\b(?:lebih|agar|supaya)\s+(?:\w+\s+)?(?:halus|smooth|mulus|lembut)\b", "ingin")]},
    {"id": "belok", "meaning": "arah melenceng / berbelok",
     "patterns": [(r"\b(?:berbelok|belok|melenceng|menyimpang|serong|nyerong|berputar|memutar)\s+(?:\w+\s+)?"
                   + _TO + r"(?P<side>kiri|kanan)\b", "keluhan"),
                  (r"\b(?:berbelok|belok|melenceng|menyimpang|serong|nyerong)\b", "keluhan"),
                  (r"\b(?:" + _NOT[3:-1] + r"|kurang)\s+lurus\b|\b(?:berputar|memutar)\s+sendiri\b", "keluhan")]},
    {"id": "sulit_kembali", "meaning": "sulit kembali ke posisi siap / tegak",
     "patterns": [(r"\b(?:sulit|susah|kesulitan|lambat|lama|tidak bisa|tak bisa|gagal|" + _NOT[3:-1] + r"|kurang)\s+"
                   r"(?:\w+\s+){0,2}kembali\b", "keluhan"),
                  (r"\b(?:" + _NOT[3:-1] + r"|kurang)\s+tegak\b", "keluhan")]},
    {"id": "tendangan_lemah", "meaning": "tendangan lemah / kurang kuat",
     "patterns": [(r"\b(?:tendangan|tendang|nendang)(?:\s*nya)?\s+(?:\w+\s+){0,2}(?:lemah|pelan|kurang\s+"
                   r"(?:kuat|keras|jauh|kencang))\b", "keluhan")]},
    {"id": "tempo_cepat", "meaning": "tempo terlalu cepat",
     "patterns": [(r"\b(?:langkah|jalan|gerak|gerakan|tempo|ayunan|bangun|tendangan|tendang|nendang)(?:\s*nya)?\s+"
                   r"(?:\w+\s+)?(?:terlalu\s+)?(?:cepat|buru-buru|terburu-buru)\b", "keluhan")]},
    {"id": "tempo_lambat", "meaning": "tempo terlalu lambat",
     "patterns": [(r"\b(?:langkah|jalan|gerak|gerakan|tempo|ayunan|bangun|berdiri|tendangan|tendang|nendang)"
                   r"(?:\s*nya)?\s+(?:\w+\s+)?(?:terlalu\s+)?(?:lambat|pelan|lama)\b", "keluhan")]},
    {"id": "berat_badan", "meaning": "perpindahan berat badan kiri-kanan",
     "patterns": [(r"\b(?:perpindahan|pindah|pemindahan)\s+berat\b|\bayun(?:an)?\s+(?:badan|pinggul)\b", "keluhan")]},
    {"id": "jatuh", "meaning": "jatuh / roboh (arah tidak disebut)", "generic": True,
     "patterns": [(r"\b(?:jatuh|terjatuh|roboh|terguling|ambruk|tumbang)\b", "keluhan")]},
]
BEHAVIOR = {b["id"]: b for b in BEHAVIORS}
DIRECTIONAL = {"condong_samping", "condong_belakang", "condong_depan"}

# Fase dicari per potongan kalimat (koma, "tapi", "sehingga", ...): "awalan bagus,
# tapi agak jauh jatuh" = awal BAGUS, tengah JATUH. "mulai miring" bukan fase awal.
PHASES = [
    ("kembali", "kembali ke posisi siap", [
        r"\b(?:setelah|sesudah|habis)\s+(?:menendang|nendang|tendang)\b",
        r"\bkembali\s+(?:\w+\s+){0,2}(?:siap|semula|awal|berdiri|tegak)\b", r"\bbalik\s+(?:ke\s+)?(?:posisi|siap|semula)\b"]),
    ("tendang", "saat menendang / satu kaki terangkat", [
        r"\b(?:saat|ketika|waktu|pas|sewaktu)\s+(?:\w+\s+)?(?:menendang|nendang|tendang|mengayun|mengangkat|angkat)\b",
        r"\bposisi\s+(?:menendang|nendang|tendang)\b"]),
    ("berhenti", "saat berhenti", [r"\bberhenti\b", r"\bstop\b"]),
    ("tengah", "setelah beberapa langkah / makin jauh", [
        r"\b(?:setelah|sesudah)\s+(?:beberapa|bbrp|\w+)\s+langkah\b", r"\bagak\s+jauh\b", r"\b(?:makin|semakin)\s+jauh\b",
        r"\blama[- ]?(?:ke)?lamaan\b", r"\bdi\s+tengah\b", r"\blangkah\s+(?:ke-?\s*)?\d+\b"]),
    ("awal", "awal gerakan / mulai melangkah", [
        r"\bawal\w*\b", r"\bpermulaan\b", r"\bstart\b", r"\blangkah\s+pertama\b",
        r"\b(?:saat|ketika|waktu|pas|sewaktu|pertama|baru)\s+(?:\w+\s+)?mulai\b",
        r"\bmulai\s+(?:jalan|berjalan|melangkah|bergerak)\b", r"\bpertama\b"]),
]
PHASE_TEXT = {key: text for key, text, _ in PHASES}

_CLAUSE = r",|;|:|\s+-+\s+|—|\b(?:tapi|tetapi|namun|sedangkan|dan|sehingga|karena|soalnya|lalu|terus|jadi)\b"
_POSITIVE = (r"\b(?:bagus|baik|mantap|lancar|berhasil|sukses|mengenai|kena\s+sasaran|oke|rapi|stabil|halus|smooth|"
             r"lurus|tegak)\b")
_NOT_POSITIVE = {"kurang", "lebih", "agar", "supaya", "perbaiki", "tolong", "makin", "biar"}
_FILLER = {"sih", "yah", "ya", "itu", "ini", "dia", "nya", "robot", "juga", "masih", "tolong", "sudah", "udah",
           "yang", "si", "deh", "kok", "lah", "kayak", "kaya", "menurut", "ku", "aku", "saya", "sepertinya",
           "mungkin", "kalau", "kalo", "agak", "cukup", "sekali", "banget", "nih", "tuh", "ke", "di", "dari",
           "untuk", "buat", "dengan", "pada", "saat", "ketika", "waktu", "pas", "tadi", "lagi", "ada", "jadi",
           "terus", "lalu", "dong", "pun", "malah", "hanya", "cuma", "seperti", "sebelumnya", "sama"}
_STILL = r"\b(?:masih|tetap|belum\s+juga|sama\s+seperti|masih\s+sama)\b"
_SAME = r"\b(?:masih|tetap)\s+sama\b|\bsama\s+(?:seperti|kayak|dengan)\b|\bseperti\s+(?:yang\s+)?sebelumnya\b"
_CONTEXT = r"\b(?:saat|ketika|waktu|sewaktu)\s+(?:berjalan|jalan|melangkah|menendang|nendang|bangun|berdiri)\b"
_SOFTER = r"\b(?:sedikit|dikit|agak)\s+(?:\w+\s+)?$"


def _negated(text, start):
    before = re.findall(r"[a-z]+", text[max(0, start - 40):start])
    return bool(NEGATIONS.intersection(before[-2:]))


def _side_of(text):
    found = re.findall(r"\b(kiri|kanan)\b", text)
    return found[-1] if found else None


def _foot(sentence, start):
    """Kaki yang disebut paling dekat dengan frasa: kiri/kanan/tumpuan/tendang, atau None."""
    best = None
    for m in re.finditer(r"\bkaki\s+(kiri|kanan|tumpuan|penumpu|tendang|yang\s+menendang)(?:\s*nya)?\b", sentence):
        word = {"penumpu": "tumpuan", "yang menendang": "tendang"}.get(re.sub(r"\s+", " ", m.group(1)), m.group(1))
        distance = abs(m.start() - start)
        if best is None or distance < best[0]:
            best = (distance, word)
    return best[1] if best else None


def understand(text):
    """Pemahaman satu teks feedback. Mengembalikan {"sentences": [...]} dengan, per kalimat:
    text, behaviors [{id, meaning, phrases, mode, side, want, foot, phases, still, scale}],
    phases (seluruh kalimat), positives, unclear."""
    lowered = text.lower()
    same_length = len(text) == len(lowered)
    out = []
    for sm in re.finditer(r"[^.!?\n]+", lowered):
        sentence = sm.group(0)
        if not sentence.strip():
            continue
        clauses, start = [], 0
        for m in list(re.finditer(_CLAUSE, sentence)) + [None]:
            end = m.start() if m else len(sentence)
            if sentence[start:end].strip(" ,;:-—"):
                clauses.append((start, end))
            start = m.end() if m else end
        clause_of = lambda pos: next((i for i, (a, b) in enumerate(clauses) if a <= pos < b),
                                     min(range(len(clauses)), key=lambda i: abs(clauses[i][0] - pos)))
        piece = lambda i: sentence[clauses[i][0]:clauses[i][1]].strip(" ,;:-—")
        clause_phases = [[] for _ in clauses]
        taken = []
        for key, _, patterns in PHASES:
            for pattern in patterns:
                for m in re.finditer(pattern, sentence):
                    if any(m.start() < e and s < m.end() for s, e in taken):
                        continue
                    taken.append((m.start(), m.end()))
                    if key not in clause_phases[clause_of(m.start())]:
                        clause_phases[clause_of(m.start())].append(key)
        hits, spans, positives, explained = [], [], [], set()
        for behavior in BEHAVIORS:
            for pattern, mode in behavior["patterns"]:
                for m in re.finditer(pattern, sentence):
                    if any(m.start() < e and s < m.end() for s, e in spans):
                        continue
                    spans.append((m.start(), m.end()))
                    clause = clause_of(m.start())
                    if _negated(sentence, m.start()):
                        # "sudah tidak goyah" = hal yang sudah baik, bukan keluhan
                        positives.append(piece(clause))
                        explained.add(clause)
                        continue
                    phrase = text[sm.start() + m.start():sm.start() + m.end()] if same_length else m.group(0)
                    hits.append({"id": behavior["id"], "mode": mode, "side": m.groupdict().get("side"),
                                 "start": m.start(), "phrase": phrase.strip(), "clause": clause,
                                 "softer": bool(re.search(_SOFTER, sentence[max(0, m.start() - 20):m.start()]))})
        explained |= {h["clause"] for h in hits}
        if any(h["id"] in DIRECTIONAL for h in hits):
            hits = [h for h in hits if not BEHAVIOR[h["id"]].get("generic")]
        for i, (a, b) in enumerate(clauses):
            if i in explained:
                continue
            for m in re.finditer(_POSITIVE, sentence[a:b]):
                before = re.findall(r"[a-z]+", sentence[a:a + m.start()])[-2:]
                if not _NOT_POSITIVE.intersection(before) and not NEGATIONS.intersection(before):
                    positives.append(piece(i))
                    explained.add(i)
                    break
        still = bool(re.search(_STILL, sentence))
        unclear = []
        for i, (a, b) in enumerate(clauses):
            if i in explained or clause_phases[i] or re.search(_SAME + "|" + _CONTEXT, sentence[a:b]):
                continue
            if len([w for w in re.findall(r"[a-z]+", sentence[a:b]) if w not in _FILLER]) >= 2:
                unclear.append(piece(i))
        merged = {}
        for hit in hits:
            behavior = BEHAVIOR[hit["id"]]
            side, want = hit["side"], behavior.get("want")
            if side and (hit["id"] == "belok" or hit["mode"] == "keluhan"):
                want = {"kiri": "kanan", "kanan": "kiri"}[side]
            elif side:
                want = side
            # fase: dari potongan kalimat frasa itu sendiri, atau potongan tepat di sebelahnya
            # yang HANYA berisi fase ("Saat menendang, kaki kiri mentok")
            phases = list(clause_phases[hit["clause"]])
            for near in (hit["clause"] - 1, hit["clause"] + 1):
                if not phases and 0 <= near < len(clauses) and clause_phases[near] and near not in explained:
                    phases = list(clause_phases[near])
            a, b = clauses[hit["clause"]]
            entry = merged.setdefault((hit["id"], want), {
                "id": hit["id"], "meaning": behavior["meaning"], "phrases": [], "mode": hit["mode"], "side": side,
                "want": want, "foot": _foot(sentence[a:b], hit["start"] - a), "phases": [], "still": still,
                "scale": 0.5 if hit["softer"] else 1.0})
            entry["phrases"].append(hit["phrase"])
            entry["phases"] += [p for p in phases if p not in entry["phases"]]
            entry["foot"] = entry["foot"] or _foot(sentence[a:b], hit["start"] - a)
        all_phases = [key for key, _, _ in PHASES if any(key in c for c in clause_phases)]
        out.append({"text": text[sm.start():sm.end()].strip() if same_length else sentence.strip(),
                    "behaviors": list(merged.values()), "phases": all_phases, "positives": positives,
                    "unclear": unclear, "still": still})
    return {"sentences": out}


def understanding_text(result, indent="      "):
    """Pemahaman satu catatan untuk dibaca operator, per kalimat."""
    lines = []
    for s in result["sentences"]:
        if not (s["behaviors"] or s["positives"] or s["unclear"]):
            continue
        lines.append('%s"%s"' % (indent, s["text"]))
        for b in s["behaviors"]:
            extra = []
            if b["foot"]:
                extra.append("kaki %s" % b["foot"])
            if b["want"]:
                extra.append("yang diinginkan: %s" % ("arah jalan dikoreksi ke %s" % b["want"].upper()
                                                      if b["id"] == "belok" else _WANT_TEXT[b["want"]]))
            extra.append("fase: %s" % (", ".join(PHASE_TEXT[p] for p in b["phases"]) or "tidak disebut"))
            if b["still"]:
                extra.append("MASIH terjadi")
            lines.append('%s  - %s ("%s"; %s)' % (indent, b["meaning"], '", "'.join(b["phrases"]), "; ".join(extra)))
        for p in s["positives"]:
            lines.append('%s  + sudah baik, dipertahankan: "%s"' % (indent, p))
        for u in s["unclear"]:
            lines.append('%s  ? tidak dipahami: "%s"' % (indent, u))
    return "\n".join(lines) or "%s(tidak ada kalimat yang dikenali)" % indent


_WANT_TEXT = {"depan": "badan lebih condong ke DEPAN", "belakang": "badan lebih tegak / ke BELAKANG",
              "kiri": "badan lebih ke KIRI", "kanan": "badan lebih ke KANAN"}

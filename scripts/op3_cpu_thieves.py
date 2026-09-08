#!/usr/bin/env python3
"""Siapa yang mencuri CPU dari detektor bola.

Kenapa alat ini ada: i5-10210U di robot ini punya 4 inti FISIK tapi 8 utas,
dengan pasangan Hyper-Threading `0<->4  1<->5  2<->6  3<->7`. Dua utas dalam
satu pasangan BERBAGI unit eksekusi -- mereka bukan dua mesin, melainkan satu
mesin yang dipakai bergantian.

Detektor bola dipatok ke inti 0-2. Karena itu setiap proses yang duduk di inti
**4, 5, atau 6** mencuri tenaganya dari dalam, walau `top` memperlihatkan inti
0-2 "milik" detektor. Terukur di mesin ini:

    inferensi, inti sibling sepi ....  30,1 ms
    inferensi, inti sibling sibuk ...  47,7 ms     <- 1,6x lebih lambat

Dan `taskset -c 0-2` pada detektor TIDAK mencadangkan inti 0-2 untuk dia --
ia cuma melarang detektor keluar dari sana. Proses lain tetap bebas masuk.

Alat ini menjawab satu pertanyaan yang tidak bisa dijawab `top` maupun `htop`
tanpa dipelototi: **selama demo berjalan, siapa yang duduk di inti sibling
milik detektor, dan berapa CPU yang mereka pakai di sana?** Jawabannya
menentukan apakah perlu mematok proses tertentu -- keputusan yang sebaiknya
tidak diambil berdasarkan dugaan, karena mematok `op3_manager` menyentuh loop
bus servo 125 Hz.

Cara pakai: jalankan demo seperti biasa, lalu di terminal KEDUA:

    scripts/op3_cpu_thieves.py              # cuplik 30 detik, lalu lapor
    scripts/op3_cpu_thieves.py --seconds 60
    scripts/op3_cpu_thieves.py --detector-cpus 0-2   # kalau afinitas diubah

Berjalan di host maupun di dalam container: proses container tetap terlihat
dari host (`/proc` dibagi), jadi menjalankannya di host memberi gambaran paling
lengkap -- termasuk desktop, yang justru sering jadi pencuri terbesar.
"""

import argparse
import os
import sys
import time
from collections import defaultdict

BOLD, RED, GREEN, YELLOW, DIM, RST = (
    "\033[1m", "\033[31m", "\033[32m", "\033[33m", "\033[2m", "\033[0m",
) if sys.stdout.isatty() else ("",) * 6

CLK = os.sysconf("SC_CLK_TCK")


def parse_cpu_list(spec):
    """"0-2" / "3,7" / "0-2,4" -> set inti."""
    out = set()
    for part in str(spec).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.update(range(int(a), int(b) + 1))
        else:
            out.add(int(part))
    return out


def sibling_map():
    """{inti: inti pasangannya} dari topologi kernel, bukan tebakan."""
    sib = {}
    base = "/sys/devices/system/cpu"
    for name in os.listdir(base):
        if not name.startswith("cpu") or not name[3:].isdigit():
            continue
        cpu = int(name[3:])
        path = os.path.join(base, name, "topology/thread_siblings_list")
        try:
            with open(path) as fh:
                sib[cpu] = parse_cpu_list(fh.read().strip())
        except OSError:
            pass
    return sib


def read_stat(pid):
    """(utime+stime dalam tik, inti yang sedang dipakai, comm). None kalau hilang."""
    try:
        with open("/proc/%d/stat" % pid) as fh:
            data = fh.read()
    except (OSError, ProcessLookupError):
        return None
    # comm bisa mengandung spasi dan tanda kurung; potong di ')' TERAKHIR.
    lo, hi = data.find("("), data.rfind(")")
    if lo < 0 or hi < 0:
        return None
    comm = data[lo + 1:hi]
    rest = data[hi + 2:].split()
    try:
        # field setelah comm: state=0, ... utime=11, stime=12, processor=36
        return int(rest[11]) + int(rest[12]), int(rest[36]), comm
    except (IndexError, ValueError):
        return None


def affinity(pid):
    try:
        with open("/proc/%d/status" % pid) as fh:
            for line in fh:
                if line.startswith("Cpus_allowed_list:"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "?"


def cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            return fh.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def pids():
    return [int(n) for n in os.listdir("/proc") if n.isdigit()]


def sample(seconds, interval, watch):
    """Cuplik berkala. Kembalikan {pid: dict} dengan CPU total dan CPU di inti watch."""
    acc = defaultdict(lambda: {
        "ticks": 0, "on_watch": 0, "samples": 0, "comm": "", "cores": defaultdict(int),
    })
    prev = {}
    n = max(1, int(seconds / interval))
    for i in range(n + 1):
        for pid in pids():
            st = read_stat(pid)
            if st is None:
                continue
            ticks, cpu, comm = st
            if pid in prev:
                delta = ticks - prev[pid]
                if delta > 0:
                    e = acc[pid]
                    e["comm"] = comm
                    e["ticks"] += delta
                    e["cores"][cpu] += delta
                    if cpu in watch:
                        # `processor` adalah inti saat DICUPLIK, bukan rata-rata.
                        # Cukup untuk memberi peringkat, bukan angka pasti.
                        e["on_watch"] += delta
                    e["samples"] += 1
            prev[pid] = ticks
        if i < n:
            time.sleep(interval)
            sys.stderr.write("\r%s  mencuplik... %d/%d detik%s" %
                             (DIM, int((i + 1) * interval), int(seconds), RST))
            sys.stderr.flush()
    sys.stderr.write("\r" + " " * 40 + "\r")
    return acc


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="lama pencuplikan (bawaan 30)")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="jarak antar cuplikan, detik (bawaan 0,5)")
    ap.add_argument("--detector-cpus", default="0-2",
                    help="inti tempat detektor dipatok (bawaan 0-2, sama dengan "
                         "cpu_affinity di yolo_ball_detector_openvino.launch.py)")
    ap.add_argument("--top", type=int, default=12, help="berapa baris ditampilkan")
    args = ap.parse_args()

    det = parse_cpu_list(args.detector_cpus)
    sib = sibling_map()
    # Inti sibling = pasangan dari inti detektor, DIKURANGI inti detektor sendiri.
    watch = set()
    for c in det:
        watch |= sib.get(c, set())
    watch -= det

    print("%sInti detektor%s  : %s" % (BOLD, RST, sorted(det)))
    print("%sInti sibling%s   : %s   <- proses di sini mencuri tenaga detektor"
          % (BOLD, RST, sorted(watch)))
    if not watch:
        print("%sTidak ada sibling terdeteksi (Hyper-Threading mati?). "
              "Alat ini tidak berguna di mesin ini.%s" % (YELLOW, RST))
        return 0
    print("Mencuplik %.0f detik tiap %.1f detik...\n" % (args.seconds, args.interval))

    acc = sample(args.seconds, args.interval, watch)
    if not acc:
        print("%sTidak ada proses aktif tercuplik.%s" % (YELLOW, RST))
        return 0

    total_cpu_ticks = args.seconds * CLK  # setara satu inti terpakai penuh
    rows = []
    for pid, e in acc.items():
        if e["ticks"] <= 0:
            continue
        rows.append((
            e["on_watch"] / total_cpu_ticks * 100.0,   # % satu-inti di sibling
            e["ticks"] / total_cpu_ticks * 100.0,      # % satu-inti total
            pid, e["comm"], affinity(pid), e["cores"],
        ))
    rows.sort(reverse=True)

    print("%s%-7s %-18s %8s %8s  %-12s %s%s" %
          (BOLD, "PID", "PROSES", "CPU%", "di-SIB%", "AFINITAS", "CATATAN", RST))
    shown = 0
    thief_total = 0.0
    for on_watch, total, pid, comm, aff, cores in rows:
        thief_total += on_watch
        if shown >= args.top and on_watch < 1.0:
            continue
        shown += 1
        allowed = parse_cpu_list(aff) if aff != "?" else set()
        note = ""
        colour = ""
        if on_watch >= 5.0:
            colour, note = RED, "PENCURI: banyak duduk di inti sibling"
        elif on_watch >= 1.0:
            colour, note = YELLOW, "sesekali di inti sibling"
        if allowed and allowed & watch:
            if not (allowed - watch - det):
                note = (note + "; " if note else "") + "dipatok TEPAT ke sibling"
            elif len(allowed) >= 8:
                note = (note + "; " if note else "") + "TIDAK dipatok"
        elif allowed and not (allowed & watch):
            colour = colour or GREEN
            note = note or "aman (tidak boleh masuk sibling)"
        print("%s%-7d %-18s %7.1f%% %7.1f%%  %-12s %s%s"
              % (colour, pid, comm[:18], total, on_watch, aff, note, RST))

    print()
    print("%sTotal CPU yang dipakai di inti sibling: %.1f%% dari satu inti%s"
          % (BOLD, thief_total, RST))
    if thief_total >= 30.0:
        print("%s-> BESAR. Ini sebab nyata inferensi melambat. Kurung pelakunya:%s"
              % (RED, RST))
        print("   desktop host        : ./script.sh --match-mode on")
        print("   alat debug container: jalankan dengan `taskset -c 3,7 rqt`, atau tutup")
        print("   node ROS            : tambah launch-prefix `taskset -c 3,7`")
    elif thief_total >= 10.0:
        print("%s-> SEDANG. Layak dikurung, tapi bukan satu-satunya sebab.%s"
              % (YELLOW, RST))
    else:
        print("%s-> KECIL. Inti sibling bukan masalah utama di run ini;%s"
              % (GREEN, RST))
        print("   cari di tempat lain (panas, atau pesaing di inti 0-2 itu sendiri).")
        print("   Cek panas: cat /sys/devices/system/cpu/cpu0/thermal_throttle/"
              "package_throttle_total_time_ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())

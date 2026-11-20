// Panel Kalibrasi Batas Geser (operator 2026-11-12).
//
// Area kiper 1,8 m: kiper D58 tak boleh keluar. Panel ini adalah tampilan untuk
// `scripts/calib_batas_geser.py layan` di robot. Perekam itu mendengar perintah walking
// (Start/Stop dari tab ini juga), menghitung WAKTU EFEKTIF geser (soft start naik linear
// ikut dihitung), dan menulis src/ROBOTIS-OP3-Demo/op3_demo/data/kalibrasi_batas_geser.yaml
// yang dibaca demo. Panel ini TIDAK menggerakkan robot -- geser tetap lewat Apply & Start / Stop.
//
//   status   : /kalibrasi_batas_geser/status   (std_msgs/String JSON, ~2 Hz)
//   perintah : /kalibrasi_batas_geser/perintah (std_msgs/String JSON: mulai/simpan/batal/jarak/hapus)
import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { ChevronDown, ChevronRight, Play, Ruler, Save, Trash2, X } from "lucide-react";

const STATUS_TOPIC = "/kalibrasi_batas_geser/status";
const PERINTAH_TOPIC = "/kalibrasi_batas_geser/perintah";
const OPEN_KEY = "bascorro.kalibrasi_batas_geser.open";
const BTN =
  "inline-flex items-center gap-1.5 rounded-lg px-3 py-1.5 text-xs font-bold transition-colors disabled:cursor-not-allowed disabled:opacity-40";

const fmtCm = (v) => (v == null || !Number.isFinite(v) ? "—" : `${v.toFixed(1)} cm`);
const fmtS = (v) => (v == null || !Number.isFinite(v) ? "—" : `${v.toFixed(2)} dtk`);
const ARAH_LABEL = { kanan: "Geser kanan", kiri: "Geser kiri" };
// Titik awal rekaman (2026-11-12):
//   tengah : tengah gawang -> selotip tepi; waktunya langsung jadi batas.
//   ujung  : satu ujung area -> ke arah ujung seberang; jarak WAJIB diukur; lintasan lebih
//            panjang = kecepatan lebih teliti; batas = setengah lebar area / kecepatan.
const ASAL_LABEL = { tengah: "Tengah gawang", ujung: "Ujung gawang", semua: "Semua rekaman" };
const ARAH_OPSI = {
  tengah: { kanan: "Tengah → kanan (geser kanan)", kiri: "Tengah → kiri (geser kiri)" },
  ujung: { kanan: "Ujung KIRI → kanan (geser kanan)", kiri: "Ujung KANAN → kiri (geser kiri)" },
};

function parseCm(text) {
  const v = Number(String(text).replace(",", "."));
  return Number.isFinite(v) && v > 0 && v < 300 ? v : null;
}

function readOpen() {
  try {
    return localStorage.getItem(OPEN_KEY) !== "0";
  } catch {
    return true;
  }
}

export default function KalibrasiBatasGeser({ ros, rosState }) {
  const [status, setStatus] = useState(null);
  const [lastSeen, setLastSeen] = useState(0);
  const [now, setNow] = useState(() => Date.now());
  const [arah, setArah] = useState("kanan");
  const [mode, setMode] = useState("terus");
  const [asal, setAsal] = useState("tengah");
  const [jarakCm, setJarakCm] = useState("");
  const [open, setOpen] = useState(readOpen);
  const pubRef = useRef(null);
  const connected = rosState === "connected" && !!ros;

  useEffect(() => {
    if (!connected) return undefined;
    const sub = new ROSLIB.Topic({ ros, name: STATUS_TOPIC, messageType: "std_msgs/String" });
    pubRef.current = new ROSLIB.Topic({ ros, name: PERINTAH_TOPIC, messageType: "std_msgs/String" });
    sub.subscribe((msg) => {
      try {
        setStatus(JSON.parse(msg.data));
        setLastSeen(Date.now());
      } catch {
        /* pesan rusak -- abaikan */
      }
    });
    return () => {
      sub.unsubscribe();
      pubRef.current = null;
    };
  }, [ros, connected]);

  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);

  const toggleOpen = () => {
    setOpen((v) => {
      try {
        localStorage.setItem(OPEN_KEY, v ? "0" : "1");
      } catch {
        /* penyimpanan browser tak tersedia */
      }
      return !v;
    });
  };

  const alive = connected && now - lastSeen < 3000;
  const kirim = (obj) => pubRef.current?.publish(new ROSLIB.Message({ data: JSON.stringify(obj) }));
  const fase = status?.fase || null;
  const kal = status?.kalibrasi || null;
  const cmValid = parseCm(jarakCm);
  // Dari ujung: jarak wajib (tanpa itu rekaman tak berarti). Dari tengah: opsional.
  const jarakWajib = fase?.asal === "ujung";
  const jarakSalah = (jarakCm.trim() !== "" && cmValid == null) || (jarakWajib && cmValid == null);

  const simpan = () => {
    if (jarakSalah) return;
    kirim({ aksi: "simpan", jarak_cm: cmValid });
    setJarakCm("");
  };

  // Baris "CHRONUS bergeser ..." -- segmen yang sedang berjalan, atau yang terakhir.
  const live = (() => {
    if (!status) return null;
    if (status.berjalan) {
      const b = status.berjalan;
      return { judul: "Sedang geser", arah: b.arah, efektif: b.efektif, cm: b.cm, jalan: true };
    }
    const s = status.segmen_terakhir;
    if (s) return { judul: `Geser terakhir (segmen ${s.no})`, arah: s.arah, efektif: s.efektif, cm: s.cm, jalan: false };
    return null;
  })();

  return (
    <div className="bg-white rounded-2xl border border-gray-200 shadow-sm p-4">
      <button type="button" onClick={toggleOpen} className="flex w-full items-center gap-2 text-left">
        {open ? <ChevronDown size={16} className="text-gray-400" /> : <ChevronRight size={16} className="text-gray-400" />}
        <Ruler size={16} className="text-undip-blue" />
        <h3 className="text-sm font-bold text-gray-800">Kalibrasi Batas Geser (area kiper 1,8 m)</h3>
        <span
          className={`ml-auto rounded-full px-2 py-0.5 text-[10px] font-bold ${
            alive ? "bg-green-50 text-green-700 border border-green-100" : "bg-gray-100 text-gray-500 border border-gray-200"
          }`}
        >
          {alive ? "perekam aktif" : "perekam mati"}
        </span>
        {kal && (
          <span
            className={`rounded-full px-2 py-0.5 text-[10px] font-bold ${
              kal.valid ? "bg-blue-50 text-undip-blue border border-blue-100" : "bg-yellow-50 text-yellow-700 border border-yellow-100"
            }`}
          >
            {kal.valid ? "kalibrasi valid" : "belum valid"}
          </span>
        )}
      </button>

      {open && (
        <div className="mt-3 space-y-3">
          {!alive && (
            <div className="rounded-xl border border-gray-200 bg-gray-50 p-3 text-xs text-gray-600">
              Perekam belum jalan di robot. Di container:{" "}
              <code className="font-mono text-[11px]">python3 scripts/calib_batas_geser.py layan</code>
              {" "}(ROS_DOMAIN_ID=1, ROS_LOCALHOST_ONLY=1). Panel ini tidak menggerakkan robot.
            </div>
          )}

          {alive && (
            <>
              {/* Hasil geser langsung */}
              <div className={`rounded-xl border p-3 ${live?.jalan ? "border-green-200 bg-green-50" : "border-gray-200 bg-gray-50"}`}>
                {live ? (
                  <div className="flex flex-wrap items-baseline gap-x-4 gap-y-1">
                    <span className="text-[10px] font-bold uppercase tracking-wide text-gray-500">{live.judul}</span>
                    <span className="text-sm font-bold text-gray-800">{ARAH_LABEL[live.arah] || "—"}</span>
                    <span className="text-2xl font-black text-undip-blue">
                      {live.cm != null ? `≈ ${fmtCm(live.cm)}` : fmtS(live.efektif)}
                    </span>
                    <span className="font-mono text-xs text-gray-500">
                      {fmtS(live.efektif)} efektif
                      {live.cm == null && " — jarak cm muncul sesudah jarak terukur arah ini diisi"}
                    </span>
                  </div>
                ) : (
                  <span className="text-xs text-gray-500">Belum ada geser terekam sejak perekam jalan.</span>
                )}
                <div className="mt-1 font-mono text-[10px] text-gray-400">
                  soft start {fmtS(status.soft)} · y_move {status.y?.toFixed?.(4)} · period {status.period?.toFixed?.(3)}
                </div>
              </div>

              {/* Rekam fase */}
              {!fase ? (
                <div className="flex flex-wrap items-center gap-2">
                  <span className="w-20 shrink-0 text-[10px] font-bold uppercase tracking-wide text-gray-400">Rekam</span>
                  <div className="inline-flex overflow-hidden rounded-lg border border-gray-200 text-xs font-bold" role="group" aria-label="Mulai dari">
                    {["tengah", "ujung"].map((a) => (
                      <button
                        key={a}
                        type="button"
                        onClick={() => setAsal(a)}
                        className={`px-3 py-1.5 ${asal === a ? "bg-undip-blue text-white" : "bg-white text-gray-600 hover:bg-gray-50"}`}
                      >
                        Dari {ASAL_LABEL[a].toLowerCase()}
                      </button>
                    ))}
                  </div>
                  <select value={arah} onChange={(e) => setArah(e.target.value)} className="rounded-lg border border-gray-200 px-2 py-1.5 text-xs">
                    <option value="kanan">{ARAH_OPSI[asal].kanan}</option>
                    <option value="kiri">{ARAH_OPSI[asal].kiri}</option>
                  </select>
                  <select value={mode} onChange={(e) => setMode(e.target.value)} className="rounded-lg border border-gray-200 px-2 py-1.5 text-xs">
                    <option value="terus">Terus (sekali jalan)</option>
                    <option value="putus">Putus-putus (banyak start/stop)</option>
                  </select>
                  <button
                    type="button"
                    className={`${BTN} bg-undip-blue text-white hover:bg-opacity-90`}
                    disabled={status.jalan}
                    title={status.jalan ? "Stop dulu, letakkan robot di titik awal" : "Robot sudah di titik awal dan diam"}
                    onClick={() => kirim({ aksi: "mulai", arah, mode, asal })}
                  >
                    <Play size={14} /> Mulai Rekam
                  </button>
                  <span className="text-[11px] text-gray-400">
                    {asal === "tengah"
                      ? mode === "terus"
                        ? "Robot di tengah gawang. Geser sekali sampai selotip tepi, Stop, ukur jarak."
                        : "Dari tengah ke selotip yang SAMA dengan banyak start/stop pendek (~1 dtk) — koreksi ekor langkah."
                      : mode === "terus"
                        ? `Robot di ujung ${arah === "kanan" ? "KIRI" : "KANAN"} gawang. Geser sekali ke arah seberang, Stop, ukur jarak yang ditempuh (wajib).`
                        : "Dari ujung yang sama, banyak start/stop pendek (~1 dtk), Stop, ukur jarak (wajib) — koreksi ekor langkah."}
                  </span>
                </div>
              ) : (
                <div className="rounded-xl border border-undip-blue/20 bg-blue-50/40 p-3">
                  <div className="flex flex-wrap items-baseline gap-x-4 gap-y-1">
                    <span className="text-[10px] font-bold uppercase tracking-wide text-undip-blue">Merekam</span>
                    <span className="text-sm font-bold text-gray-800">
                      {ARAH_LABEL[fase.arah]} · {fase.mode === "terus" ? "terus" : "putus-putus"} · dari{" "}
                      {fase.asal === "ujung" ? `ujung ${fase.arah === "kanan" ? "kiri" : "kanan"}` : "tengah"}
                    </span>
                    <span className="font-mono text-xs text-gray-600">
                      total {fmtS(fase.total_efektif)} efektif · {fase.total_segmen} segmen
                      {fase.perkiraan_cm != null && ` · ≈ ${fmtCm(fase.perkiraan_cm)}`}
                    </span>
                    {fase.efektif_arah_lain > 0.2 && (
                      <span className="text-xs font-bold text-red-600">tercampur arah lain {fmtS(fase.efektif_arah_lain)} — Batal & ulangi</span>
                    )}
                  </div>
                  {fase.segmen.length > 0 && (
                    <div className="mt-2 flex flex-wrap gap-1.5">
                      {fase.segmen.map((s) => (
                        <span key={s.no} className="rounded-md border border-gray-200 bg-white px-2 py-0.5 font-mono text-[11px] text-gray-600">
                          #{s.no} {s.arah} {s.efektif.toFixed(2)}s{s.cm != null ? ` ≈${s.cm.toFixed(0)}cm` : ""}
                        </span>
                      ))}
                    </div>
                  )}
                  <div className="mt-3 flex flex-wrap items-center gap-2">
                    <label className="text-xs font-bold text-gray-600" htmlFor="kalib-jarak">
                      {jarakWajib ? "Jarak yang ditempuh (wajib)" : "Jarak terukur tengah → selotip"}
                    </label>
                    <input
                      id="kalib-jarak"
                      value={jarakCm}
                      onChange={(e) => setJarakCm(e.target.value)}
                      placeholder={jarakWajib ? "cm (wajib)" : fase.mode === "terus" ? "cm (disarankan)" : "cm (opsional)"}
                      inputMode="decimal"
                      className={`w-32 rounded-lg border px-2 py-1.5 text-xs ${
                        jarakCm.trim() !== "" && cmValid == null ? "border-red-300 bg-red-50" : "border-gray-200"
                      }`}
                    />
                    <span className="text-xs text-gray-400">cm</span>
                    <button
                      type="button"
                      className={`${BTN} bg-green-600 text-white hover:bg-green-700`}
                      disabled={status.jalan || fase.total_segmen === 0 || jarakSalah}
                      title={
                        status.jalan
                          ? "Stop dulu"
                          : jarakWajib && cmValid == null
                            ? "Isi jarak yang ditempuh dulu (dari ujung wajib)"
                            : "Simpan rekaman ini dan hitung ulang batas"
                      }
                      onClick={simpan}
                    >
                      <Save size={14} /> Simpan
                    </button>
                    <button type="button" className={`${BTN} bg-gray-100 text-gray-700 hover:bg-gray-200`} onClick={() => kirim({ aksi: "batal" })}>
                      <X size={14} /> Batal
                    </button>
                  </div>
                </div>
              )}

              {status.pesan && (
                <div className={`text-xs ${status.galat ? "font-bold text-red-600" : "text-gray-600"}`}>{status.pesan}</div>
              )}

              {/* Hasil kalibrasi */}
              {kal && (
                <div className="overflow-x-auto">
                  <table className="w-full text-xs">
                    <thead>
                      <tr className="text-left text-[10px] uppercase tracking-wide text-gray-400">
                        <th className="py-1 pr-3">Arah</th>
                        <th className="py-1 pr-3">Dari</th>
                        <th className="py-1 pr-3">Tepi (efektif)</th>
                        <th className="py-1 pr-3">Jarak tepi</th>
                        <th className="py-1 pr-3">Kecepatan</th>
                        <th className="py-1 pr-3">Ekor / segmen</th>
                        <th className="py-1 pr-3" title="Tepi luar kaki + galat 2 sigma; nilai yang dipakai demo ada di demo.launch.xml">
                          Toleransi saran
                        </th>
                      </tr>
                    </thead>
                    <tbody>
                      {["kanan", "kiri"].map((a) => {
                        const h = kal[a];
                        return (
                          <tr key={a} className="border-t border-gray-100 font-mono">
                            <td className="py-1 pr-3 font-sans font-bold text-gray-700">{ARAH_LABEL[a]}</td>
                            <td className="py-1 pr-3 font-sans">{h ? ASAL_LABEL[h.asal] || h.asal : "—"}</td>
                            <td className="py-1 pr-3" title={h?.rekaman_dicocokkan ? `dicocokkan dari ${h.rekaman_dicocokkan} rekaman` : undefined}>
                              {h ? fmtS(h.tepi_efektif_sec) : "belum"}
                            </td>
                            <td className="py-1 pr-3">{h?.jarak_tepi_m ? fmtCm(h.jarak_tepi_m * 100) : "—"}</td>
                            <td className="py-1 pr-3">{h?.cm_per_dtk ? `${h.cm_per_dtk.toFixed(2)} cm/dtk` : "—"}</td>
                            <td className="py-1 pr-3">
                              {h ? `${h.ekor_sec.toFixed(3)} dtk` : "—"}
                              {h && !h.punya_putus && <span className="font-sans text-[10px] text-yellow-700"> (belum ada putus-putus)</span>}
                            </td>
                            <td className="py-1 pr-3">{h?.toleransi_saran_m ? fmtCm(h.toleransi_saran_m * 100) : "—"}</td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                </div>
              )}

              {/* Daftar rekaman */}
              {status.rekaman?.length > 0 && (
                <details className="text-xs">
                  <summary className="cursor-pointer font-bold text-gray-500">Rekaman tersimpan ({status.rekaman.length})</summary>
                  <div className="mt-2 space-y-1">
                    {status.rekaman.map((r) => (
                      <div key={`${r.nomor}-${r.waktu}`} className="flex flex-wrap items-center gap-2 font-mono text-[11px] text-gray-600">
                        <span className="w-6 text-gray-400">#{r.nomor}</span>
                        <span className="w-12">{r.arah}</span>
                        <span className="w-12">{r.mode}</span>
                        <span className="w-14">{r.asal || "tengah"}</span>
                        <span className="w-20">{fmtS(r.efektif_sec)}</span>
                        <span className="w-16">{r.segmen} seg</span>
                        <span className="w-20">{r.jarak_m ? fmtCm(r.jarak_m * 100) : "—"}</span>
                        {r.campur && <span className="font-sans font-bold text-red-600">tercampur</span>}
                        <span className="text-gray-400">{r.waktu}</span>
                        <button
                          type="button"
                          className="ml-auto text-gray-400 hover:text-red-600"
                          title="Hapus rekaman ini"
                          onClick={() => {
                            if (window.confirm(`Hapus rekaman #${r.nomor} (${r.arah} ${r.mode})?`)) kirim({ aksi: "hapus", nomor: r.nomor });
                          }}
                        >
                          <Trash2 size={13} />
                        </button>
                      </div>
                    ))}
                  </div>
                </details>
              )}
            </>
          )}
        </div>
      )}
    </div>
  );
}

import React from "react";

// Menangkap galat render supaya kegagalan satu halaman TIDAK berakhir jadi
// layar putih kosong.
//
// Kenapa perlu: 2026-09-08 satu tombol di halaman Tendangan memakai variabel
// yang ternyata const lokal di dalam fungsi render halaman LAIN. Itu
// ReferenceError saat render, React membongkar seluruh pohon komponen, dan yang
// terlihat operator cuma halaman putih tanpa satu pun keterangan -- tidak ada
// petunjuk halaman mana, baris mana, atau bahkan bahwa itu galat program.
// `vite build` tidak menangkapnya: ini kesalahan runtime, bukan sintaks.
//
// Dengan ini, kegagalan yang sama menampilkan nama galat, pesannya, dan jejak
// tumpukannya, plus tombol untuk mencoba merender ulang tanpa memuat ulang
// halaman (koneksi ROS dan draft yang belum dikirim ikut hilang kalau di-reload).
export default class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { error: null, info: null };
  }

  static getDerivedStateFromError(error) {
    return { error };
  }

  componentDidCatch(error, info) {
    this.setState({ info });
    // Tetap dicetak ke console: jejak tumpukan lengkapnya lebih berguna di sana.
    console.error("Bascorro Studio: galat render", error, info);
  }

  render() {
    const { error, info } = this.state;
    if (!error) return this.props.children;

    return (
      <div className="min-h-screen bg-gray-50 p-6 md:p-10">
        <div className="max-w-3xl mx-auto bg-white rounded-2xl border border-red-200 shadow-sm overflow-hidden">
          <div className="px-6 py-5 border-b border-red-100 bg-red-50">
            <h1 className="text-lg font-bold text-red-700">Halaman ini gagal dirender</h1>
            <p className="mt-1 text-sm text-red-600">
              Ini galat program di Bascorro Studio, bukan masalah di robot. Robot dan
              parameternya tidak tersentuh.
            </p>
          </div>

          <div className="px-6 py-5 space-y-4">
            <div>
              <div className="text-[10px] font-bold uppercase text-gray-400">Galat</div>
              <div className="font-mono text-sm text-gray-900 break-words">
                {error.name}: {error.message}
              </div>
            </div>

            {info?.componentStack && (
              <div>
                <div className="text-[10px] font-bold uppercase text-gray-400">Komponen</div>
                <pre className="mt-1 max-h-56 overflow-auto rounded-lg bg-gray-50 border border-gray-200 p-3 text-[11px] leading-relaxed text-gray-700">
                  {info.componentStack.trim()}
                </pre>
              </div>
            )}

            {error.stack && (
              <details>
                <summary className="cursor-pointer text-xs font-bold text-gray-500">
                  Jejak tumpukan lengkap
                </summary>
                <pre className="mt-2 max-h-56 overflow-auto rounded-lg bg-gray-50 border border-gray-200 p-3 text-[11px] leading-relaxed text-gray-700">
                  {error.stack}
                </pre>
              </details>
            )}

            <div className="flex flex-wrap gap-3 pt-1">
              <button
                className="inline-flex items-center gap-2 px-4 py-2 rounded-lg bg-undip-blue text-white text-sm font-bold hover:bg-opacity-90"
                onClick={() => this.setState({ error: null, info: null })}
              >
                Coba render ulang
              </button>
              <button
                className="inline-flex items-center gap-2 px-4 py-2 rounded-lg bg-gray-100 text-gray-700 text-sm font-bold hover:bg-gray-200"
                onClick={() => window.location.reload()}
              >
                Muat ulang halaman
              </button>
            </div>
          </div>
        </div>
      </div>
    );
  }
}

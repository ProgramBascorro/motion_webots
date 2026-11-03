// Kandidat percobaan berikutnya untuk tab RL Training (parameter walking) dan
// Action Training (nilai servo page), dipandu penilaian operator.
//
// Mode "belajar" (cross-entropy sederhana):
//  1. elit = percobaan isGood, diambil separuh teratas menurut skor (maks 5);
//  2. pusat = rata-rata elit berbobot 2^(skor - skor_tertinggi): elit yang
//     skornya 2 lebih tinggi menarik 4x lebih kuat;
//  3. sebaran mulai 1/4 rentang dan MENYEMPIT tiap kali ada percobaan isSolid
//     (memperhalus di sekitar yang sudah benar), tidak lebih kecil dari 1/10
//     rentang -- elit yang kembar membuat sebarannya nol, dan tanpa lantai ini
//     robot cuma mengulang angka yang sama tanpa peluang membaik;
//     kalau elit >= 3, ikut dicampur dengan sebaran elit itu sendiri;
//  4. tiga percobaan terakhir isBad semua -> sebaran x1,5 supaya tidak terjebak
//     di sekitar titik yang ternyata buruk.
// Belum ada satu pun isGood -> eksplorasi acak penuh dalam rentang.

export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

// Box-Muller: sampel normal baku.
export const randn = (random = Math.random) => {
  let u = 0;
  while (u === 0) u = random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * random());
};

// params  : [{ key, lo, hi, step }] -- rentang yang boleh diacak, sudah dijepit;
// history : percobaan yang sebanding, TERBARU DULU;
// valueOf : (percobaan, key) -> nilai key di percobaan itu.
export function proposeCandidate({
  params, history, mode, scoreOf, isGood, isBad, isSolid, valueOf,
  exploreBasis = "eksplorasi: belum ada percobaan yang baik", random = Math.random,
}) {
  const varied = {};
  const good = history.filter(isGood).sort((a, b) => scoreOf(b) - scoreOf(a) || (b.no || 0) - (a.no || 0));
  const learn = mode === "learn" && good.length > 0;
  let elite = [];
  let weights = [];
  let shrink = 1;
  let widen = 1;
  if (learn) {
    elite = good.slice(0, Math.min(5, Math.max(1, Math.ceil(good.length / 2))));
    const top = scoreOf(elite[0]);
    weights = elite.map((t) => 2 ** (scoreOf(t) - top));
    const solid = history.filter(isSolid).length;
    shrink = Math.max(0.2, 1 / (1 + 0.5 * solid));
    const last3 = history.slice(0, 3);
    if (last3.length === 3 && last3.every(isBad)) widen = 1.5;
  }
  const wSum = weights.reduce((s, x) => s + x, 0);
  params.forEach((p) => {
    const span = p.hi - p.lo;
    let v;
    if (learn) {
      const vals = elite.map((t) => valueOf(t, p.key));
      const center = vals.reduce((s, x, i) => s + weights[i] * x, 0) / wSum;
      let sigma = (span / 4) * shrink;
      if (elite.length >= 3) {
        const sd = Math.sqrt(vals.reduce((s, x, i) => s + weights[i] * (x - center) ** 2, 0) / wSum);
        sigma = 0.5 * sigma + 0.5 * sd;
      }
      sigma = clamp(sigma * widen, span / 10, span / 2);
      v = center + randn(random) * sigma;
    } else {
      v = p.lo + random() * span;
    }
    v = clamp(v, p.lo, p.hi);
    varied[p.key] = Number((Math.round(v / p.step) * p.step).toFixed(4));
  });
  let basis = "acak penuh dalam rentang";
  if (learn) {
    basis = `pusat dari ${elite.map((t) => `#${t.no}`).join(", ")} · sebaran ${Math.round(shrink * widen * 100)}%`
      + (widen > 1 ? " (dilebarkan: 3 terakhir gagal)" : "");
  } else if (mode === "learn") {
    basis = exploreBasis;
  }
  return { varied, learn, basis };
}

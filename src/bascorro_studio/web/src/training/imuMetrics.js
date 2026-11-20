// Ringkasan IMU satu percobaan tab RL Training -- MURNI (tanpa ROS), diuji di
// web/test/imuMetrics.test.mjs. Perekamnya ada di imuRecorder.js.
//
// Sampel = [t, gx, gy, gz (rad/s), qx, qy, qz, qw] dari /robotis/open_cr/imu;
// t = cap waktu header (detik). Di CHRONUS quaternion itu = roll/pitch
// AKSELEROMETER + yaw kompas (OpenCRModule::publishIMU), jadi putaran badan di
// sini diambil dari gyro z, bukan dari quaternion: kompas hanya andal saat diam.

const RAD = 180 / Math.PI;
const round = (v, digits) => Number(v.toFixed(digits));

// Roll/pitch (derajat) dari quaternion sampel.
export function tiltDeg(sample) {
  const [, , , , x, y, z, w] = sample;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.min(1, Math.max(-1, 2 * (w * y - z * x))));
  return { roll: roll * RAD, pitch: pitch * RAD };
}

// Putaran badan selama berjalan: integral gyro z dikurangi bias saat diam
// (1 s sebelum start), sehingga ayunan badan tiap langkah saling meniadakan.
// REP-103: + = berlawanan jarum jam = KIRI (sama dengan tanda kalibrasi yaw CHRONUS).
//   turn_dps    : rata-rata kecepatan putar dari start sampai stop (°/s)
//   turn_deg    : total putaran dari start sampai rekaman selesai (termasuk berhenti)
//   wobble_dps  : goyang roll/pitch selama berjalan (RMS gyro, °/s)
//   tilt_max_deg: miring terbesar dari posisi diam sebelum start
// null kalau rekaman jalannya terlalu pendek.
export function turnSummary(samples, start, stop, end) {
  const still = samples.filter((s) => s[0] >= start - 1 && s[0] < start);
  const walking = samples.filter((s) => s[0] >= start && s[0] <= stop);
  if (walking.length < 10 || !(stop > start)) return null;
  const mean = (list, f) => list.reduce((sum, s) => sum + f(s), 0) / list.length;
  const bias = still.length >= 10 ? mean(still, (s) => s[3]) : 0;
  const integrate = (t0, t1) => {
    let sum = 0;
    for (let i = 1; i < samples.length; i += 1) {
      const [ta, , , za] = samples[i - 1];
      const [tb, , , zb] = samples[i];
      if (ta >= t0 && tb <= t1) sum += ((za - bias) + (zb - bias)) / 2 * (tb - ta);
    }
    return sum * RAD;
  };
  const wobble = Math.sqrt(mean(walking, (s) => s[1] ** 2 + s[2] ** 2)) * RAD;
  let tiltMax = null;
  if (still.length) {
    const ref = still.map(tiltDeg);
    const roll0 = ref.reduce((sum, r) => sum + r.roll, 0) / ref.length;
    const pitch0 = ref.reduce((sum, r) => sum + r.pitch, 0) / ref.length;
    tiltMax = Math.max(...samples.filter((s) => s[0] >= start && s[0] <= end)
      .map((s) => { const r = tiltDeg(s); return Math.hypot(r.roll - roll0, r.pitch - pitch0); }));
  }
  return {
    turn_dps: round(integrate(start, stop) / (stop - start), 1),
    turn_deg: round(integrate(start, end), 1),
    wobble_dps: round(wobble, 1),
    tilt_max_deg: tiltMax === null ? null : round(tiltMax, 1),
    bias_still: still.length >= 10,
    samples: walking.length,
  };
}

// "kanan 23.4°/s" -- kata arah untuk tampilan.
export function turnText(dps) {
  if (dps === null || dps === undefined) return "-";
  const side = Math.abs(dps) < 1 ? "lurus" : dps > 0 ? "kiri" : "kanan";
  return `${side} ${Math.abs(dps).toFixed(1)}°/s`;
}

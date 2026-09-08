// Mengukur letak bola dari satu bingkai kamera, memakai geometri yang SAMA
// dengan yang dipakai robot untuk memutuskan menendang.
//
// Ini cerminan `ball_geometry()` di scripts/observe_kick_geometry.py -- fungsi
// yang dipakai kalibrasi 2026-09-07 dan punya 12 uji sendiri di
// op3_demo/test/test_kick_geometry_observer.py. Padanan JS-nya diuji terhadap
// keluaran fungsi Python itu (test/kickMeasure.test.mjs), jadi kalau salah
// satunya bergeser, ujinya gagal.
//
// KENAPA DIHITUNG DI HALAMAN, bukan di studio_agent: operator memakai fitur ini
// untuk MEMILIH angka. Ia harus melihat akibat draft yang sedang diketik, bukan
// akibat parameter yang sudah tersimpan di robot.

import { parseWalkingNumber } from "./walkingParams.js";

// Setengah bukaan kamera yang dipakai BallTracker untuk mengubah piksel jadi
// SUDUT. Sengaja berbeda dari kick_fov_width_deg (28,94) yang dipakai model
// jarak-dari-radius: yang ini soal ARAH, dan jendela kaki kick_pan_* sudah
// ditera memakai angka ini. Jangan disamakan tanpa menera ulang jendelanya.
export const TRACKER_FOV_WIDTH_DEG = 35.2;
export const TRACKER_FOV_HEIGHT_DEG = 21.6;

const rad = (deg) => (deg * Math.PI) / 180;
const deg = (r) => (r * 180) / Math.PI;

/**
 * @param circle  {x, y, radiusPx} - x/y ternormalisasi [-1..1] dari detektor
 * @param head    {pan, tilt} radian, dari /robotis/present_joint_states
 * @param hipPitch  hip_pitch_offset walking_module, radian
 * @param params  parameter kick_* (bentuk KICK_DEFAULT_PARAMS)
 */
export function measureBall(circle, head, hipPitch, params) {
  const num = (key, fallback = NaN) => parseWalkingNumber(params?.[key], fallback);
  const x = Number(circle?.x);
  const y = Number(circle?.y);
  const radiusPx = Number(circle?.radiusPx);
  const pan = Number(head?.pan);
  const tilt = Number(head?.tilt);
  const hip = Number(hipPitch);

  const finite = [x, y, radiusPx, pan, tilt, hip].every(Number.isFinite);
  if (!finite || radiusPx <= 0) return null;

  const cameraPitchOffset = num("kick_camera_pitch_offset_deg", 0);
  if (!Number.isFinite(cameraPitchOffset) || Math.abs(cameraPitchOffset) > 60) return null;

  const imageLeft = -x * Math.tan(rad(TRACKER_FOV_WIDTH_DEG));
  const imageUp = -y * Math.tan(rad(TRACKER_FOV_HEIGHT_DEG));

  const tiltSign = num("kick_head_tilt_sign", 1);
  const cameraDepression = hip - tiltSign * tilt + rad(cameraPitchOffset);

  const pitchedForward = Math.cos(cameraDepression) + imageUp * Math.sin(cameraDepression);
  const pitchedUp = -Math.sin(cameraDepression) + imageUp * Math.cos(cameraDepression);
  const bodyForward = pitchedForward * Math.cos(pan) - imageLeft * Math.sin(pan);
  const bodyLeft = pitchedForward * Math.sin(pan) + imageLeft * Math.cos(pan);

  const bearing = Math.atan2(bodyLeft, bodyForward);
  const depression = -Math.atan2(pitchedUp, Math.hypot(bodyForward, bodyLeft));

  const height = num("kick_camera_height");
  const ballRadius = num("kick_ball_real_radius_m");
  const minDepressionDeg = num("kick_min_depression_deg");
  // Sahih hanya di antara lantai sudut dan 89 derajat: di bawahnya kepala
  // hampir datar dan jaraknya meledak, di atasnya tan berbalik tanda.
  const valid =
    Number.isFinite(height) && Number.isFinite(ballRadius) &&
    height > ballRadius && ballRadius > 0 &&
    minDepressionDeg <= deg(depression) && deg(depression) < 89;
  const distanceM = valid ? (height - ballRadius) / Math.tan(depression) : null;

  const panForWindow = num("kick_pan_window_from_bearing", 1) > 0.5 ? bearing : pan;
  const servoWindowDeg = num("head_pan_servo_center_deg", 180) +
    num("head_pan_servo_dir", 1) * deg(panForWindow);

  let footWindow = "outside";
  for (const side of ["right", "left"]) {
    const lo = num(`kick_pan_${side}_min_deg`, NaN);
    const hi = num(`kick_pan_${side}_max_deg`, NaN);
    if (Number.isFinite(lo) && Number.isFinite(hi) && servoWindowDeg >= lo && servoWindowDeg <= hi) {
      footWindow = side;
      break;
    }
  }

  return {
    distanceM,
    depressionDeg: deg(depression),
    bearingDeg: deg(bearing),
    servoWindowDeg,
    footWindow,
    radiusPx,
  };
}

// Jendela kaki dan pusat servo TIDAK ada di halaman Tendangan (mereka milik
// launch), jadi nilainya dipasok di sini supaya measureBall() tetap satu
// fungsi. Angkanya harus sama dengan demo.launch.xml.
export const MEASURE_WINDOW_DEFAULTS = {
  head_pan_servo_center_deg: 180.0,
  head_pan_servo_dir: 1.0,
  kick_pan_window_from_bearing: 1.0,
  kick_pan_right_min_deg: 152.0,
  kick_pan_right_max_deg: 178.0,
  kick_pan_left_min_deg: 193.0,
  kick_pan_left_max_deg: 208.0,
};

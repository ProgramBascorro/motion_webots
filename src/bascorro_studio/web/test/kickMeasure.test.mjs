import assert from "node:assert/strict";
import test from "node:test";
import { measureBall } from "../src/kickMeasure.js";

// Angka acuan DIHASILKAN oleh ball_geometry() di scripts/observe_kick_geometry.py
// -- implementasi Python yang dipakai kalibrasi 2026-09-07 dan punya ujinya
// sendiri di op3_demo/test/test_kick_geometry_observer.py. Kalau salah satu
// implementasi bergeser, uji ini yang menangkapnya.
//
// Bikin ulang fixture-nya:
//   python3 -c "import importlib.util; spec=importlib.util.spec_from_file_location(
//     'o','scripts/observe_kick_geometry.py'); m=importlib.util.module_from_spec(spec);
//     spec.loader.exec_module(m); print(m.ball_geometry(circle, head, hip, params))"
const PARAMS = {
  "kick_camera_height": 0.52,
  "kick_ball_real_radius_m": 0.067,
  "kick_camera_pitch_offset_deg": 30.84,
  "kick_head_tilt_sign": 1.0,
  "kick_min_depression_deg": 3.0,
  "head_pan_servo_center_deg": 180.0,
  "head_pan_servo_dir": 1.0,
  "kick_pan_window_from_bearing": 1.0,
  "kick_pan_right_min_deg": 152.0,
  "kick_pan_right_max_deg": 178.0,
  "kick_pan_left_min_deg": 193.0,
  "kick_pan_left_max_deg": 208.0
};

const KASUS = [
  {
    "nama": "acuan kontak kanan",
    "circle": [
      0.13595962524414062,
      0.18956180148654522,
      81.41329956054688
    ],
    "head": {
      "pan": -0.01227184630308513,
      "tilt": -0.521553467881118
    },
    "hip_pitch": 0.1395999938249588,
    "harap": {
      "distance_m": 0.14560602086806826,
      "depression_deg": 72.18123970690523,
      "bearing_deg": -18.829989784104313,
      "servo_window_deg": 161.17001021589567,
      "foot_window": "right"
    }
  },
  {
    "nama": "bola di tengah, kepala datar",
    "circle": [
      0.0,
      0.0,
      40.0
    ],
    "head": {
      "pan": 0.0,
      "tilt": 0.0
    },
    "hip_pitch": 0.1396,
    "harap": {
      "distance_m": 0.5626439368332884,
      "depression_deg": 38.838490820026294,
      "bearing_deg": 0.0,
      "servo_window_deg": 180.0,
      "foot_window": "outside"
    }
  },
  {
    "nama": "bola di kiri",
    "circle": [
      -0.5,
      0.3,
      55.0
    ],
    "head": {
      "pan": 0.2,
      "tilt": -0.4
    },
    "hip_pitch": 0.1396,
    "harap": {
      "distance_m": 0.24659608502198466,
      "depression_deg": 61.4377973355616,
      "bearing_deg": 55.19900656254072,
      "servo_window_deg": 235.19900656254072,
      "foot_window": "outside"
    }
  },
  {
    "nama": "di atas cakrawala (tak sahih)",
    "circle": [
      0.0,
      -0.9,
      20.0
    ],
    "head": {
      "pan": 0.0,
      "tilt": 0.5
    },
    "hip_pitch": 0.1396,
    "harap": {
      "distance_m": null,
      "depression_deg": -9.422172324517593,
      "bearing_deg": 0.0,
      "servo_window_deg": 180.0,
      "foot_window": "outside"
    }
  }
];

test("JS ball measurement matches the calibrated Python geometry", () => {
  for (const { nama, circle, head, hip_pitch: hipPitch, harap } of KASUS) {
    const hasil = measureBall(
      { x: circle[0], y: circle[1], radiusPx: circle[2] },
      { pan: head.pan, tilt: head.tilt }, hipPitch, PARAMS);
    assert.ok(hasil, `${nama}: seharusnya menghasilkan pengukuran`);
    if (harap.distance_m === null) {
      assert.equal(hasil.distanceM, null, `${nama}: jarak harus ditolak`);
    } else {
      assert.ok(Math.abs(hasil.distanceM - harap.distance_m) < 1e-9,
        `${nama}: jarak ${hasil.distanceM} != ${harap.distance_m}`);
    }
    for (const [js, py] of [["depressionDeg","depression_deg"],
                            ["bearingDeg","bearing_deg"],
                            ["servoWindowDeg","servo_window_deg"]]) {
      assert.ok(Math.abs(hasil[js] - harap[py]) < 1e-9, `${nama}: ${js} meleset`);
    }
    assert.equal(hasil.footWindow, harap.foot_window, `${nama}: jendela kaki`);
  }
});

test("measurement refuses input it cannot trust", () => {
  const head = { pan: 0, tilt: -0.5 };
  const c = { x: 0, y: 0, radiusPx: 40 };
  assert.equal(measureBall({ ...c, radiusPx: 0 }, head, 0.1396, PARAMS), null,
    "radius nol bukan deteksi");
  assert.equal(measureBall(c, { pan: NaN, tilt: 0 }, 0.1396, PARAMS), null,
    "sudut kepala tidak berhingga");
  assert.equal(measureBall(c, head, 0.1396, { ...PARAMS, kick_camera_pitch_offset_deg: 90 }), null,
    "koreksi kamera di luar batas");
  const terlalu_tinggi = measureBall(c, head, 0.1396,
    { ...PARAMS, kick_ball_real_radius_m: 0.9 });
  assert.equal(terlalu_tinggi.distanceM, null,
    "bola lebih tinggi dari kamera: tidak ada jarak, tapi arah tetap ada");
});

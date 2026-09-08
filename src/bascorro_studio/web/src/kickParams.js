import { parseWalkingNumber } from "./walkingParams.js";

export const KICK_DEFAULT_PARAMS = {
  kick_distance: 0.15,
  kick_pan_max_distance: 0.2,
  kick_camera_height: 0.52,
  kick_camera_pitch_offset_deg: 30.84,
  kick_ball_real_radius_m: 0.067,
  kick_fov_width_deg: 28.94,
  kick_image_width_px: 640,
  kick_head_tilt_sign: 1,
  kick_distance_from_radius: 0,
  kick_min_depression_deg: 3,
  kick_distance_filter_n: 5,
  kick_ready_count: 4,
  kick_ball_radius_px: 0,
  kick_ball_radius_margin: 0.1,
  kick_radius_ready_enable: 1,
};

export const KICK_PARAM_KEYS = Object.keys(KICK_DEFAULT_PARAMS);

export function validateKickParams(params) {
  const payload = {};
  for (const key of KICK_PARAM_KEYS) {
    const value = parseWalkingNumber(params?.[key], NaN);
    if (!Number.isFinite(value)) throw new Error(`${key}: isi angka yang lengkap dan berhingga`);
    payload[key] = value;
  }
  for (const key of ["kick_distance", "kick_pan_max_distance", "kick_ball_real_radius_m", "kick_image_width_px"]) {
    if (payload[key] <= 0) throw new Error(`${key}: harus lebih besar dari 0`);
  }
  if (payload.kick_camera_height <= payload.kick_ball_real_radius_m) throw new Error("Tinggi kamera harus lebih besar dari jari-jari bola");
  if (Math.abs(payload.kick_camera_pitch_offset_deg) > 60) throw new Error("Koreksi sudut kamera harus dalam -60..60 derajat");
  if (payload.kick_fov_width_deg <= 0 || payload.kick_fov_width_deg >= 90) throw new Error("Setengah bukaan kamera harus antara 0 dan 90 derajat (eksklusif)");
  if (![-1, 1].includes(payload.kick_head_tilt_sign)) throw new Error("kick_head_tilt_sign: hanya +1 atau -1");
  if (payload.kick_min_depression_deg <= 0 || payload.kick_min_depression_deg >= 89) throw new Error("Sudut tunduk minimum harus lebih dari 0 dan kurang dari 89 derajat");
  for (const key of ["kick_distance_filter_n", "kick_ready_count", "kick_image_width_px"]) {
    const minimum = key === "kick_ready_count" ? 0 : 1;
    if (!Number.isInteger(payload[key]) || payload[key] < minimum || payload[key] > 2147483647) throw new Error(`${key}: harus bilangan bulat ≥ ${minimum} dalam rentang int32`);
  }
  if (payload.kick_ball_radius_px < 0) throw new Error("Ambang radius manual tidak boleh negatif");
  if (payload.kick_ball_radius_margin < 0 || payload.kick_ball_radius_margin > 0.9) throw new Error("Potongan ambang radius harus antara 0 dan 0,9");
  for (const key of ["kick_distance_from_radius", "kick_radius_ready_enable"]) {
    if (![0, 1].includes(payload[key])) throw new Error(`${key}: hanya 0 atau 1`);
  }
  return payload;
}

export function decodeKickParameters(response) {
  const params = {};
  const types = {};
  for (const [index, key] of KICK_PARAM_KEYS.entries()) {
    const value = response?.values?.[index];
    if (![2, 3].includes(value?.type)) throw new Error(`${key}: tidak tersedia sebagai parameter numerik di /demo_node`);
    params[key] = value.type === 3 ? value.double_value : value.integer_value;
    types[key] = value.type;
  }
  return { params: validateKickParams(params), types };
}

export function makeKickParameters(params, types) {
  const payload = validateKickParams(params);
  return KICK_PARAM_KEYS.map((name) => {
    const type = types[name];
    if (type === 2 && !Number.isInteger(payload[name])) throw new Error(`${name}: node masih bertipe integer; perbaiki tipe float pada launch sebelum memakai pecahan`);
    if (![2, 3].includes(type)) throw new Error(`${name}: tipe parameter belum dibaca`);
    return { name, value: type === 2 ? { type, integer_value: payload[name] } : { type, double_value: payload[name] } };
  });
}

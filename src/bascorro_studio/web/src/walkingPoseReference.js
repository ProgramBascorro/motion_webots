import { parseWalkingNumber, snapFloat32 } from "./walkingParams.js";

export const WALKING_POSE_REFERENCE = Object.freeze({
  id: "alphonse-page2-display-v1",
  label: "Page 2 · Walking Ready",
  source: "op3_walking_module/config/param.yaml · acuan ALPHONSE 11 September 2026",
  offsets: Object.freeze({
    init_x_offset: -0.02,
    init_y_offset: 0,
    init_z_offset: 0.0408,
    init_roll_offset: 0,
    init_pitch_offset: -0.07,
    init_yaw_offset: 0,
    hip_pitch_offset: 0.1396,
  }),
});

export function walkingReferenceOffset(key, relative = true) {
  return relative && Object.hasOwn(WALKING_POSE_REFERENCE.offsets, key)
    ? WALKING_POSE_REFERENCE.offsets[key] : null;
}

export function walkingOffsetToDisplay(value, referenceOffset = null) {
  if (referenceOffset === null || referenceOffset === 0) return value;
  const native = parseWalkingNumber(value, NaN);
  if (!Number.isFinite(native) || !Number.isFinite(Math.fround(native))) return value;
  if (Math.fround(native) === Math.fround(referenceOffset)) return 0;
  const difference = native - referenceOffset;
  for (let digits = 1; digits <= 17; digits += 1) {
    const candidate = Number(difference.toPrecision(digits));
    if (Math.fround(candidate + referenceOffset) === Math.fround(native)) return candidate;
  }
  return difference;
}

export function walkingOffsetFromDisplay(value, referenceOffset = null) {
  if (referenceOffset === null || referenceOffset === 0) return value;
  const relative = parseWalkingNumber(value, NaN);
  if (!Number.isFinite(relative)) return value;
  return snapFloat32(relative + referenceOffset);
}

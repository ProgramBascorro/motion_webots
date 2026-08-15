// Shared by the two places that edit op3_walking_module's WalkingParam: the Walking
// tab in App.jsx and the Walking Tuner card in TuningPage.jsx.
//
// Both used to convert their input fields with a bare Number(). That has two failure
// modes on a form the user is actively typing in:
//   Number("")    === 0    -- a field cleared to be retyped is applied as 0
//   Number("0,5") === NaN  -- the Indonesian decimal comma is not a number
// The first one is the damaging one. On the amplitudes it silently undoes an edit; on
// period_time it is worse, because updateTimeParam() divides by period_time and every
// gait angle downstream turns into NaN.
//
// `fallback` is what a blank or unparseable field falls back to -- the caller decides
// whether that is the default gait or the value last read back from the robot.
export function parseWalkingNumber(raw, fallback) {
  if (typeof raw === "number") return Number.isFinite(raw) ? raw : fallback;
  if (raw === null || raw === undefined) return fallback;
  const text = String(raw).trim().replace(",", ".");
  if (text === "" || text === "." || text === "-" || text === "-.") return fallback;
  const numeric = Number(text);
  return Number.isFinite(numeric) ? numeric : fallback;
}

#ifndef OP3_DEMO_FALL_DETECTOR_H_
#define OP3_DEMO_FALL_DETECTOR_H_

namespace robotis_op
{

// Deteksi jatuh demo sepak bola, dipakai BERSAMA oleh SoccerDemo dan mode
// otomatis D60 supaya keduanya pasti memakai rumus dan ambang yang sama --
// bukan dua salinan yang lama-lama berbeda.
//
// Isinya dipindah apa adanya dari SoccerDemo::imuDataCallback():
//   - pitch dalam derajat, REP-103: condong ke DEPAN = positif (tanda ini
//     diverifikasi di robot, lihat open_cr_module::publishIMU());
//   - ditapis EMA dengan alpha 0,4;
//   - di atas +60 = jatuh telungkup (bangun depan), di bawah -60 = jatuh
//     telentang (bangun belakang).
// Roll TIDAK dipakai: kode aslinya memang tidak punya pemeriksaan jatuh ke
// samping, dan tidak ada data untuk menetapkan ambangnya.
constexpr double kFallPitchAlpha = 0.4;
constexpr double kFallForwardLimitDeg = 60.0;
constexpr double kFallBackLimitDeg = -60.0;

// Nilainya sama dengan SoccerDemo::Stand_Status (Stand 0, Fallen_Forward 1,
// Fallen_Behind 2), jadi aman di-cast ke sana.
enum class PitchFallState
{
  Stand = 0,
  FallenForward = 1,
  FallenBehind = 2,
};

// filtered_pitch_deg dipegang pemanggil. Nilai PERSIS 0 dianggap "belum pernah
// diisi" dan langsung diganti sampel pertama -- keanehan ini sengaja ikut
// dipertahankan dari kode aslinya supaya perilaku D56 tidak berubah sedikit pun.
inline PitchFallState updatePitchFallState(double &filtered_pitch_deg, double pitch_deg,
                                           double forward_limit_deg = kFallForwardLimitDeg,
                                           double back_limit_deg = kFallBackLimitDeg,
                                           double alpha = kFallPitchAlpha)
{
  if (filtered_pitch_deg == 0)
    filtered_pitch_deg = pitch_deg;
  else
    filtered_pitch_deg = filtered_pitch_deg * (1 - alpha) + pitch_deg * alpha;

  if (filtered_pitch_deg > forward_limit_deg)
    return PitchFallState::FallenForward;
  if (filtered_pitch_deg < back_limit_deg)
    return PitchFallState::FallenBehind;
  return PitchFallState::Stand;
}

}  // namespace robotis_op

#endif  // OP3_DEMO_FALL_DETECTOR_H_

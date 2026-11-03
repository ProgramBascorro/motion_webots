#ifndef OP3_DEMO_STRIKER_STRATEGY_H_
#define OP3_DEMO_STRIKER_STRATEGY_H_

// =============================================================================
//  Strategi striker KSTI 2026 -- logika murni, tanpa ROS (bisa di-gtest)
// =============================================================================
//
// Terjemahan panduan "ksti2026_STRIKER_adaptasi_minimal-v2" ke KONVENSI ROBOT
// ini (panduan yang menyesuaikan robot, bukan sebaliknya):
//
//   panduan                          robot ALPHONSE (konvensi D60)
//   cx/W piksel (0 kiri..1 kanan)    servo 19 persen (0 % kiri .. 100 % kanan)
//   cy/H piksel (besar = dekat)      servo 20 derajat (> 200 = dekat)
//   multi-bola + hysteresis lock     detektor robot SUDAH mengunci bola
//                                    terbesar (= terdekat) dan menahannya;
//                                    di sini hanya gerbang terima/tolak
//   search_direction -1 / +1         -1 = putar KANAN, +1 = putar KIRI
//   error_arah piksel                galat YAW derajat (yang dipakai orbit D60)
//   gyro_yaw_deg                     yaw hasil integrasi gyro z (TANPA
//                                    magnetometer, regulasi KSTI 2026)
//
// Konvensi sudut: yaw dan arah (bearing) POSITIF = KIRI (CCW, ROS); sama
// dengan head_pan (+ = menoleh kiri).

#include <array>
#include <limits>
#include <string>
#include <vector>

namespace robotis_op
{
namespace striker
{

constexpr int kJumlahBola = 4;   // urutan optimal: 0 = B, 1 = A, 2 = C, 3 = D

// Nilai arah putar sama dengan panduan (ARAH_PUTAR_KANAN = -1, KIRI = +1).
enum class ArahPutar
{
  Kanan = -1,
  Kiri = +1,
};

enum class PutusanBola
{
  Terima,
  TolakCooldown,   // bola yang baru ditendang (jauh, lurus ke depan, < 1,5 s)
  TolakSektor,     // servo 19 terlalu jauh dari sektor bola target
};

enum class StatusGerbang
{
  BelumDinilai,
  Diterima,
  Ditolak,
};

enum class ModeTarget
{
  OffsetKiper,   // MODE_UTAMA_OFFSET_KIPER
  Gawang,        // MODE_CADANGAN_GAWANG
  GyroDR,        // MODE_CADANGAN_GYRO_DR
};

const char *namaBola(int indeks);           // "B", "A", "C", "D"
const char *toString(ArahPutar arah);
const char *toString(PutusanBola putusan);
const char *toString(StatusGerbang status);
const char *toString(ModeTarget mode);      // nama mode persis seperti panduan

struct StrategyConfig
{
  int start_index = 0;

  // ---- Titik 1: pengarah search ------------------------------------------
  // Arah putar per bola: -1 kanan, +1 kiri, 0 = ikut tanda yaw (aturan B di
  // panduan 01: yaw > 0 -> kanan, selain itu kiri).
  std::array<int, kJumlahBola> turn_direction_per_step{{0, -1, 1, 1}};
  // Bias sektor panduan (0,50 / 0,70 / 0,30 / 0,25) dalam persen servo 19.
  std::array<double, kJumlahBola> sector_pct_per_step{{50.0, 70.0, 30.0, 25.0}};
  // Bola terkunci ditolak kalau |servo 19 - sektor| > ini. >= 100 = mati.
  double sector_gate_pct = 40.0;
  // Cooldown pasca-tendang (panduan: 1,5 s, |yaw| < 25, bola di bagian atas
  // layar = jauh). "Jauh" di robot = servo 20 <= ambang jarak D60.
  double post_kick_ignore_sec = 1.5;
  double post_kick_ignore_yaw_deg = 25.0;
  double far_tilt_servo_deg = 200.0;

  // ---- Titik 2: setpoint orbit ---------------------------------------------
  double keeper_min_conf = 0.50;
  double goalpost_min_conf = 0.45;
  double ema_alpha = 0.45;
  double keeper_width_offset_multiplier = 1.35;
  double min_offset_frame_ratio = 0.08;
  double max_offset_frame_ratio = 0.22;
  double image_width_px = 640.0;
  // Setengah FOV horizontal kamera = fov_width_deg head_tracking.yaml.
  double half_hfov_deg = 35.2;
  // Sisi celah saat gawang TIDAK terlihat: -1 kiri kiper, +1 kanan, 0 = ikut
  // tanda yaw (aturan B panduan 02: yaw < 0 -> kiri, selain itu kanan).
  std::array<int, kJumlahBola> gap_side_per_step{{0, -1, 1, 1}};
  // Cadangan terakhir: target yaw per bola. Tanda DIBALIK dari panduan 02
  // (keputusan operator) supaya konsisten dengan yaw + = kiri.
  std::array<double, kJumlahBola> gyro_target_yaw_per_step{{0.0, 22.0, -18.0, -35.0}};
};

// ---- Titik 1 ------------------------------------------------------------------

class PengarahCariBola
{
public:
  void configure(const StrategyConfig &cfg) { cfg_ = cfg; }
  void reset(int start_index);
  int indeks() const { return indeks_; }
  // Panggil SEKALI tepat sesudah halaman tendang selesai.
  void laporSelesaiMenendang(double now_sec);
  // Di robot, sesudah tendang masih ada INIT (pose JALAN MENUJU MEKKAH) sebelum
  // kepala menyala lagi; 1,5 s panduan dihitung dari saat robot MULAI MELIHAT
  // lagi, bukan dari akhir halaman (kalau tidak, jendelanya habis saat INIT).
  void mulaiCooldown(double now_sec);
  double waktuSelesaiTendang() const { return waktu_selesai_tendang_; }

  ArahPutar arahPutar(double yaw_deg) const;
  double sektorPct() const;
  // pan_pct / tilt_servo_deg = posisi bola dalam satuan servo 19 / 20.
  // boleh_tolak_sektor = false sesudah robot berkomitmen pada bola itu
  // (approach selesai), supaya bola yang akan ditendang tidak ditolak.
  PutusanBola nilaiBola(double pan_pct, double tilt_servo_deg, double yaw_deg, double now_sec,
                        bool boleh_tolak_sektor) const;

private:
  StrategyConfig cfg_;
  int indeks_ = 0;
  double waktu_selesai_tendang_ = -std::numeric_limits<double>::infinity();
};

// Kunci terima/tolak per bola yang dikunci detektor. Status dibuang begitu
// bola tidak terlihat lebih lama dari release_sec (= target_lost_sec detektor:
// sesudah itu detektor boleh mengunci bola lain).
class GerbangBola
{
public:
  void reset();
  // ada_bola: circle_set berisi bola. bisa_dinilai: pelacak TRACKING dan
  // posisi servo segar. Mengembalikan true kalau bola boleh dipakai demo
  // (hanya status Diterima).
  bool update(bool ada_bola, bool bisa_dinilai, double pan_pct, double tilt_servo_deg, double yaw_deg,
              double now_sec, bool boleh_tolak_sektor, double release_sec, const PengarahCariBola &pengarah);
  StatusGerbang status() const { return status_; }
  PutusanBola putusanTerakhir() const { return putusan_; }
  bool berubah() const { return berubah_; }   // status berganti pada update terakhir

private:
  StatusGerbang status_ = StatusGerbang::BelumDinilai;
  PutusanBola putusan_ = PutusanBola::Terima;
  double terakhir_terlihat_ = -std::numeric_limits<double>::infinity();
  bool berubah_ = false;
};

// ---- Titik 2 ------------------------------------------------------------------

struct Deteksi
{
  double cx = 0.0;     // piksel gambar asli
  double cy = 0.0;
  double w = 0.0;
  double h = 0.0;
  double conf = 0.0;
};

struct PengamatanTarget
{
  bool kiper_valid = false;
  double kiper_cx = 0.0;
  double kiper_w = 0.0;
  bool gawang_valid = false;
  double gawang_cx = 0.0;
  int frame_kiper = 0;    // frame yang menyumbang (untuk log)
  int frame_gawang = 0;
};

// Mengumpulkan deteksi selama lirik, dihaluskan EMA (ema_smoothing_alpha).
class PengumpulTarget
{
public:
  void reset() { obs_ = PengamatanTarget(); }
  // Kiper = kotak "robot" ber-conf tertinggi di atas ambang.
  void tambahKiper(const std::vector<Deteksi> &kotak, const StrategyConfig &cfg);
  // Gawang = tengah tiang paling kiri dan paling kanan; TORSO-21 melabeli
  // tiap tiang sendiri, jadi butuh >= 2 tiang.
  void tambahTiang(const std::vector<Deteksi> &kotak, const StrategyConfig &cfg);
  const PengamatanTarget &hasil() const { return obs_; }

private:
  PengamatanTarget obs_;
};

struct HasilTarget
{
  ModeTarget mode = ModeTarget::GyroDR;
  double target_yaw_deg = 0.0;
  double target_x_px = std::numeric_limits<double>::quiet_NaN();
  double offset_px = 0.0;
  int sisi_celah = 0;
};

// Arah sebuah kolom piksel relatif badan, derajat, + = kiri.
double arahPikselDeg(double x_px, double pan_deg, const StrategyConfig &cfg);

// pan_deg / yaw_deg = posisi kepala dan yaw gyro SAAT deteksi dikumpulkan.
HasilTarget hitungTargetOrbit(const PengamatanTarget &obs, double pan_deg, double yaw_deg, int indeks,
                              const StrategyConfig &cfg);

// ---- Yaw gyro (pengganti magnetometer) -------------------------------------------

class GyroYaw
{
public:
  // Yaw kembali 0 (arah sekarang = menghadap gawang); bias dipertahankan.
  void reset();
  // gz_rad_s = angular_velocity.z apa adanya; sign menyesuaikan arah sumbu.
  // diam = badan tidak bergerak: bias diperbarui (EMA, konstanta waktu tau).
  // Bias disimpan dalam satuan MENTAH (sebelum sign), jadi tanda boleh diganti
  // kapan saja tanpa merusak bias yang sudah terkumpul.
  void update(double gz_rad_s, double stamp_sec, bool diam, double sign, double bias_tau_sec, double max_dt_sec);
  double yawDeg() const { return yaw_deg_; }
  double biasDegS() const { return bias_deg_s_; }   // mentah, sebelum sign
  bool biasReady() const { return bias_time_sec_ >= kBiasReadySec; }

  static constexpr double kBiasReadySec = 1.0;

private:
  double yaw_deg_ = 0.0;
  double bias_deg_s_ = 0.0;
  double bias_time_sec_ = 0.0;
  double last_stamp_ = std::numeric_limits<double>::quiet_NaN();
};

double normalizeDeg(double deg);

}  // namespace striker
}  // namespace robotis_op

#endif  // OP3_DEMO_STRIKER_STRATEGY_H_

#include "op3_demo/striker_strategy.h"

#include <algorithm>
#include <cmath>

namespace robotis_op
{
namespace striker
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
double deg2rad(double deg) { return deg * kPi / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / kPi; }

int clampIndeks(int indeks) { return std::max(0, std::min(kJumlahBola - 1, indeks)); }

double ema(bool ada, double lama, double baru, double alpha)
{
  return ada ? alpha * baru + (1.0 - alpha) * lama : baru;
}

}  // namespace

const char *namaBola(int indeks)
{
  static const char *kNama[kJumlahBola] = {"B", "A", "C", "D"};
  return kNama[clampIndeks(indeks)];
}

const char *toString(ArahPutar arah)
{
  return arah == ArahPutar::Kanan ? "KANAN" : "KIRI";
}

const char *toString(PutusanBola putusan)
{
  switch (putusan)
  {
  case PutusanBola::Terima: return "TERIMA";
  case PutusanBola::TolakCooldown: return "TOLAK-COOLDOWN";
  case PutusanBola::TolakSektor: return "TOLAK-SEKTOR";
  }
  return "?";
}

const char *toString(StatusGerbang status)
{
  switch (status)
  {
  case StatusGerbang::BelumDinilai: return "belum-dinilai";
  case StatusGerbang::Diterima: return "diterima";
  case StatusGerbang::Ditolak: return "ditolak";
  }
  return "?";
}

const char *toString(ModeTarget mode)
{
  switch (mode)
  {
  case ModeTarget::OffsetKiper: return "MODE_UTAMA_OFFSET_KIPER";
  case ModeTarget::Gawang: return "MODE_CADANGAN_GAWANG";
  case ModeTarget::GyroDR: return "MODE_CADANGAN_GYRO_DR";
  }
  return "?";
}

double normalizeDeg(double deg)
{
  if (!std::isfinite(deg))
    return deg;
  deg = std::fmod(deg, 360.0);
  if (deg > 180.0)
    deg -= 360.0;
  else if (deg <= -180.0)
    deg += 360.0;
  return deg;
}

// ---- Titik 1 ----------------------------------------------------------------------

void PengarahCariBola::reset(int start_index)
{
  indeks_ = clampIndeks(start_index);
  waktu_selesai_tendang_ = -std::numeric_limits<double>::infinity();
}

void PengarahCariBola::laporSelesaiMenendang(double now_sec)
{
  indeks_ = std::min(kJumlahBola - 1, indeks_ + 1);
  waktu_selesai_tendang_ = now_sec;
}

void PengarahCariBola::mulaiCooldown(double now_sec)
{
  waktu_selesai_tendang_ = now_sec;
}

ArahPutar PengarahCariBola::arahPutar(double yaw_deg) const
{
  const int arah = cfg_.turn_direction_per_step[static_cast<size_t>(indeks_)];
  if (arah < 0)
    return ArahPutar::Kanan;
  if (arah > 0)
    return ArahPutar::Kiri;
  // Bola B (panduan 01): badan miring ke kiri (yaw > 0) -> putar kanan.
  return (std::isfinite(yaw_deg) && yaw_deg > 0.0) ? ArahPutar::Kanan : ArahPutar::Kiri;
}

double PengarahCariBola::sektorPct() const
{
  return cfg_.sector_pct_per_step[static_cast<size_t>(indeks_)];
}

PutusanBola PengarahCariBola::nilaiBola(double pan_pct, double tilt_servo_deg, double yaw_deg, double now_sec,
                                        bool boleh_tolak_sektor) const
{
  // Cooldown: bola yang masih menggelinding ke gawang = jauh dan lurus ke depan.
  // Yaw yang belum bisa dibaca dianggap lurus (lebih aman menolak).
  const bool dalam_cooldown = now_sec - waktu_selesai_tendang_ < cfg_.post_kick_ignore_sec;
  const bool lurus = !std::isfinite(yaw_deg) || std::fabs(yaw_deg) < cfg_.post_kick_ignore_yaw_deg;
  const bool jauh = std::isfinite(tilt_servo_deg) && tilt_servo_deg <= cfg_.far_tilt_servo_deg;
  if (dalam_cooldown && lurus && jauh)
    return PutusanBola::TolakCooldown;

  if (boleh_tolak_sektor && cfg_.sector_gate_pct < 100.0 && std::isfinite(pan_pct) &&
      std::fabs(pan_pct - sektorPct()) > cfg_.sector_gate_pct)
    return PutusanBola::TolakSektor;
  return PutusanBola::Terima;
}

void GerbangBola::reset()
{
  status_ = StatusGerbang::BelumDinilai;
  putusan_ = PutusanBola::Terima;
  terakhir_terlihat_ = -std::numeric_limits<double>::infinity();
  berubah_ = false;
}

bool GerbangBola::update(bool ada_bola, bool bisa_dinilai, double pan_pct, double tilt_servo_deg, double yaw_deg,
                         double now_sec, bool boleh_tolak_sektor, double release_sec,
                         const PengarahCariBola &pengarah)
{
  const StatusGerbang sebelum = status_;
  // Bola terkunci hilang lebih lama dari release_sec: detektor sudah boleh
  // mengunci bola lain, jadi penilaian lama tidak berlaku lagi.
  if (status_ != StatusGerbang::BelumDinilai && now_sec - terakhir_terlihat_ > release_sec)
    status_ = StatusGerbang::BelumDinilai;
  if (ada_bola)
  {
    terakhir_terlihat_ = now_sec;
    if (status_ == StatusGerbang::BelumDinilai && bisa_dinilai)
    {
      putusan_ = pengarah.nilaiBola(pan_pct, tilt_servo_deg, yaw_deg, now_sec, boleh_tolak_sektor);
      status_ = (putusan_ == PutusanBola::Terima) ? StatusGerbang::Diterima : StatusGerbang::Ditolak;
    }
  }
  berubah_ = (status_ != sebelum);
  return ada_bola && status_ == StatusGerbang::Diterima;
}

// ---- Titik 2 ----------------------------------------------------------------------

void PengumpulTarget::tambahKiper(const std::vector<Deteksi> &kotak, const StrategyConfig &cfg)
{
  const Deteksi *terbaik = nullptr;
  for (const Deteksi &d : kotak)
    if (d.conf >= cfg.keeper_min_conf && (terbaik == nullptr || d.conf > terbaik->conf))
      terbaik = &d;
  if (terbaik == nullptr)
    return;
  obs_.kiper_cx = ema(obs_.kiper_valid, obs_.kiper_cx, terbaik->cx, cfg.ema_alpha);
  obs_.kiper_w = ema(obs_.kiper_valid, obs_.kiper_w, terbaik->w, cfg.ema_alpha);
  obs_.kiper_valid = true;
  ++obs_.frame_kiper;
}

void PengumpulTarget::tambahTiang(const std::vector<Deteksi> &kotak, const StrategyConfig &cfg)
{
  int jumlah = 0;
  double kiri = 0.0;
  double kanan = 0.0;
  for (const Deteksi &d : kotak)
  {
    if (d.conf < cfg.goalpost_min_conf)
      continue;
    kiri = (jumlah == 0) ? d.cx : std::min(kiri, d.cx);
    kanan = (jumlah == 0) ? d.cx : std::max(kanan, d.cx);
    ++jumlah;
  }
  if (jumlah < 2)
    return;
  obs_.gawang_cx = ema(obs_.gawang_valid, obs_.gawang_cx, 0.5 * (kiri + kanan), cfg.ema_alpha);
  obs_.gawang_valid = true;
  ++obs_.frame_gawang;
}

double arahPikselDeg(double x_px, double pan_deg, const StrategyConfig &cfg)
{
  const double x_norm = 2.0 * x_px / cfg.image_width_px - 1.0;
  return pan_deg - rad2deg(std::atan(x_norm * std::tan(deg2rad(cfg.half_hfov_deg))));
}

HasilTarget hitungTargetOrbit(const PengamatanTarget &obs, double pan_deg, double yaw_deg, int indeks,
                              const StrategyConfig &cfg)
{
  HasilTarget hasil;
  const size_t i = static_cast<size_t>(clampIndeks(indeks));

  // PRIORITAS 1: celah di samping kiper lawan.
  if (obs.kiper_valid)
  {
    const double lebar = cfg.image_width_px;
    const double offset = std::max(cfg.min_offset_frame_ratio * lebar,
                                   std::min(cfg.max_offset_frame_ratio * lebar,
                                            cfg.keeper_width_offset_multiplier * obs.kiper_w));
    int sisi = 0;
    if (obs.gawang_valid)
    {
      sisi = (obs.kiper_cx > obs.gawang_cx) ? -1 : +1;   // kiper di kanan tengah gawang -> celah kiri
    }
    else
    {
      const int s = cfg.gap_side_per_step[i];
      sisi = (s < 0) ? -1 : (s > 0) ? +1 : ((std::isfinite(yaw_deg) && yaw_deg < 0.0) ? -1 : +1);
    }
    hasil.mode = ModeTarget::OffsetKiper;
    hasil.offset_px = offset;
    hasil.sisi_celah = sisi;
    hasil.target_x_px = obs.kiper_cx + sisi * offset;
    hasil.target_yaw_deg = normalizeDeg(yaw_deg + arahPikselDeg(hasil.target_x_px, pan_deg, cfg));
    return hasil;
  }

  // PRIORITAS 2: tengah gawang (TORSO-21 goalpost).
  if (obs.gawang_valid)
  {
    hasil.mode = ModeTarget::Gawang;
    hasil.target_x_px = obs.gawang_cx;
    hasil.target_yaw_deg = normalizeDeg(yaw_deg + arahPikselDeg(hasil.target_x_px, pan_deg, cfg));
    return hasil;
  }

  // PRIORITAS 3: dead reckoning gyro, target yaw per bola.
  hasil.mode = ModeTarget::GyroDR;
  hasil.target_yaw_deg = normalizeDeg(cfg.gyro_target_yaw_per_step[i]);
  return hasil;
}

// ---- Yaw gyro ---------------------------------------------------------------------

void GyroYaw::reset()
{
  yaw_deg_ = 0.0;
}

void GyroYaw::update(double gz_rad_s, double stamp_sec, bool diam, double sign, double bias_tau_sec,
                     double max_dt_sec)
{
  if (!std::isfinite(gz_rad_s) || !std::isfinite(stamp_sec))
    return;
  if (!std::isfinite(last_stamp_))
  {
    last_stamp_ = stamp_sec;
    return;
  }
  double dt = stamp_sec - last_stamp_;
  last_stamp_ = stamp_sec;
  if (dt <= 0.0)
    return;   // pesan ganda / urutan terbalik
  dt = std::min(dt, max_dt_sec);
  const double laju_mentah = rad2deg(gz_rad_s);
  if (diam)
  {
    // Rata-rata berjalan sampai satu tau terkumpul, sesudah itu EMA.
    const double jendela = std::max(1e-3, std::min(bias_time_sec_, std::max(1e-3, bias_tau_sec)));
    bias_deg_s_ += dt / (jendela + dt) * (laju_mentah - bias_deg_s_);
    bias_time_sec_ += dt;
  }
  yaw_deg_ = normalizeDeg(yaw_deg_ + sign * (laju_mentah - bias_deg_s_) * dt);
}

}  // namespace striker
}  // namespace robotis_op

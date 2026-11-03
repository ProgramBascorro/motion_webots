// Logika panduan striker KSTI 2026 dalam satuan robot (striker_strategy.h).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "op3_demo/striker_strategy.h"

using namespace robotis_op::striker;

namespace
{
const double kNaN = std::numeric_limits<double>::quiet_NaN();

PengarahCariBola pengarahBaru(int start = 0)
{
  PengarahCariBola p;
  p.configure(StrategyConfig());
  p.reset(start);
  return p;
}
}  // namespace

// ---- Titik 1: urutan dan arah putar -------------------------------------------------------

TEST(PengarahCariBola, UrutanBACDdanArahPutar)
{
  PengarahCariBola p = pengarahBaru();
  EXPECT_STREQ(namaBola(p.indeks()), "B");
  // B: badan miring ke kiri (yaw > 0) -> putar kanan, selain itu kiri.
  EXPECT_EQ(p.arahPutar(5.0), ArahPutar::Kanan);
  EXPECT_EQ(p.arahPutar(-5.0), ArahPutar::Kiri);
  EXPECT_EQ(p.arahPutar(kNaN), ArahPutar::Kiri);

  p.laporSelesaiMenendang(10.0);
  EXPECT_STREQ(namaBola(p.indeks()), "A");
  EXPECT_EQ(p.arahPutar(-30.0), ArahPutar::Kanan);
  EXPECT_DOUBLE_EQ(p.sektorPct(), 70.0);

  p.laporSelesaiMenendang(20.0);
  EXPECT_STREQ(namaBola(p.indeks()), "C");
  EXPECT_EQ(p.arahPutar(30.0), ArahPutar::Kiri);

  p.laporSelesaiMenendang(30.0);
  EXPECT_STREQ(namaBola(p.indeks()), "D");
  p.laporSelesaiMenendang(40.0);   // tidak lewat D
  EXPECT_STREQ(namaBola(p.indeks()), "D");
  EXPECT_EQ(p.arahPutar(0.0), ArahPutar::Kiri);
}

TEST(PengarahCariBola, StartIndexDijepit)
{
  EXPECT_EQ(pengarahBaru(2).indeks(), 2);
  EXPECT_EQ(pengarahBaru(9).indeks(), 3);
  EXPECT_EQ(pengarahBaru(-1).indeks(), 0);
}

TEST(PengarahCariBola, CooldownPascaTendang)
{
  PengarahCariBola p = pengarahBaru();
  p.laporSelesaiMenendang(10.0);   // target A, sektor 70 %
  p.mulaiCooldown(11.0);           // kepala menyala lagi sesudah INIT
  // Bola jauh (servo 20 <= 200) lurus ke depan dalam 1,5 s: bola yang baru ditendang.
  EXPECT_EQ(p.nilaiBola(60.0, 190.0, 3.0, 11.5, true), PutusanBola::TolakCooldown);
  EXPECT_EQ(p.nilaiBola(60.0, 190.0, kNaN, 11.5, true), PutusanBola::TolakCooldown);
  // Bola dekat, atau badan sudah berbelok jauh, atau cooldown habis: tidak ditolak cooldown.
  EXPECT_EQ(p.nilaiBola(60.0, 205.0, 3.0, 11.5, true), PutusanBola::Terima);
  EXPECT_EQ(p.nilaiBola(60.0, 190.0, 40.0, 11.5, true), PutusanBola::Terima);
  EXPECT_EQ(p.nilaiBola(60.0, 190.0, 3.0, 12.6, true), PutusanBola::Terima);
}

TEST(PengarahCariBola, GerbangSektor)
{
  PengarahCariBola p = pengarahBaru();
  p.laporSelesaiMenendang(0.0);   // A: 70 % +- 40
  EXPECT_EQ(p.nilaiBola(20.0, 205.0, 0.0, 100.0, true), PutusanBola::TolakSektor);
  EXPECT_EQ(p.nilaiBola(35.0, 205.0, 0.0, 100.0, true), PutusanBola::Terima);
  // Sesudah approach selesai, bola yang sama tidak boleh ditolak sektor.
  EXPECT_EQ(p.nilaiBola(20.0, 205.0, 0.0, 100.0, false), PutusanBola::Terima);

  StrategyConfig mati;
  mati.sector_gate_pct = 100.0;
  p.configure(mati);
  EXPECT_EQ(p.nilaiBola(0.0, 205.0, 0.0, 100.0, true), PutusanBola::Terima);
}

TEST(GerbangBola, KunciSampaiBolaHilang)
{
  PengarahCariBola p = pengarahBaru();
  p.laporSelesaiMenendang(0.0);   // A, sektor 70 %
  GerbangBola g;
  g.reset();

  // Belum TRACKING: belum dinilai, belum dipakai.
  EXPECT_FALSE(g.update(true, false, 10.0, 205.0, 0.0, 10.0, true, 1.5, p));
  EXPECT_EQ(g.status(), StatusGerbang::BelumDinilai);
  // TRACKING di 10 %: ditolak sektor, dan tetap ditolak walau persennya berubah.
  EXPECT_FALSE(g.update(true, true, 10.0, 205.0, 0.0, 10.1, true, 1.5, p));
  EXPECT_EQ(g.status(), StatusGerbang::Ditolak);
  EXPECT_TRUE(g.berubah());
  EXPECT_FALSE(g.update(true, true, 65.0, 205.0, 0.0, 10.2, true, 1.5, p));
  EXPECT_FALSE(g.berubah());
  // Frame kosong singkat: status bertahan.
  EXPECT_FALSE(g.update(false, false, kNaN, kNaN, 0.0, 11.0, true, 1.5, p));
  EXPECT_EQ(g.status(), StatusGerbang::Ditolak);
  // Hilang > 1,5 s: detektor boleh mengunci bola lain -> dinilai ulang.
  EXPECT_FALSE(g.update(false, false, kNaN, kNaN, 0.0, 11.8, true, 1.5, p));
  EXPECT_EQ(g.status(), StatusGerbang::BelumDinilai);
  EXPECT_TRUE(g.update(true, true, 72.0, 205.0, 0.0, 12.0, true, 1.5, p));
  EXPECT_EQ(g.status(), StatusGerbang::Diterima);
  EXPECT_TRUE(g.update(true, false, 72.0, 205.0, 0.0, 12.1, true, 1.5, p));
}

// ---- Titik 2: target orbit ---------------------------------------------------------------

TEST(TargetOrbit, ArahPiksel)
{
  StrategyConfig cfg;
  EXPECT_NEAR(arahPikselDeg(320.0, 0.0, cfg), 0.0, 1e-9);
  EXPECT_NEAR(arahPikselDeg(640.0, 0.0, cfg), -35.2, 1e-9);   // tepi kanan = -setengah FOV
  EXPECT_NEAR(arahPikselDeg(0.0, 10.0, cfg), 45.2, 1e-9);      // tepi kiri + pan kepala
}

TEST(TargetOrbit, OffsetKiperDijepitDanSisiCelah)
{
  StrategyConfig cfg;
  PengamatanTarget obs;
  obs.kiper_valid = true;
  obs.kiper_cx = 400.0;
  obs.kiper_w = 20.0;            // 1,35 x 20 = 27 < 0,08 x 640 = 51,2
  obs.gawang_valid = true;
  obs.gawang_cx = 320.0;         // kiper di KANAN tengah gawang -> celah KIRI kiper
  HasilTarget h = hitungTargetOrbit(obs, 0.0, 10.0, 1, cfg);
  EXPECT_EQ(h.mode, ModeTarget::OffsetKiper);
  EXPECT_EQ(h.sisi_celah, -1);
  EXPECT_NEAR(h.offset_px, 51.2, 1e-9);
  EXPECT_NEAR(h.target_x_px, 400.0 - 51.2, 1e-9);
  EXPECT_NEAR(h.target_yaw_deg, 10.0 + arahPikselDeg(400.0 - 51.2, 0.0, cfg), 1e-9);
  EXPECT_LT(h.target_yaw_deg, 10.0);   // target di kanan tengah gambar -> yaw lebih kecil

  obs.kiper_w = 200.0;           // 270 > 0,22 x 640 = 140,8
  obs.kiper_cx = 250.0;          // kiper di kiri tengah gawang -> celah KANAN
  h = hitungTargetOrbit(obs, 0.0, 0.0, 1, cfg);
  EXPECT_EQ(h.sisi_celah, +1);
  EXPECT_NEAR(h.offset_px, 140.8, 1e-9);
  EXPECT_NEAR(h.target_x_px, 250.0 + 140.8, 1e-9);
}

TEST(TargetOrbit, SisiCelahTanpaGawang)
{
  StrategyConfig cfg;
  PengamatanTarget obs;
  obs.kiper_valid = true;
  obs.kiper_cx = 320.0;
  obs.kiper_w = 60.0;
  EXPECT_EQ(hitungTargetOrbit(obs, 0.0, 0.0, 1, cfg).sisi_celah, -1);    // A
  EXPECT_EQ(hitungTargetOrbit(obs, 0.0, 0.0, 2, cfg).sisi_celah, +1);    // C
  EXPECT_EQ(hitungTargetOrbit(obs, 0.0, 0.0, 3, cfg).sisi_celah, +1);    // D
  EXPECT_EQ(hitungTargetOrbit(obs, 0.0, -2.0, 0, cfg).sisi_celah, -1);   // B, yaw < 0
  EXPECT_EQ(hitungTargetOrbit(obs, 0.0, 2.0, 0, cfg).sisi_celah, +1);    // B, yaw >= 0
}

TEST(TargetOrbit, GawangButuhDuaTiangLaluTabelGyro)
{
  StrategyConfig cfg;
  PengumpulTarget kumpul;
  kumpul.tambahTiang({{200.0, 100.0, 10.0, 80.0, 0.9}}, cfg);   // satu tiang saja
  EXPECT_FALSE(kumpul.hasil().gawang_valid);
  HasilTarget h = hitungTargetOrbit(kumpul.hasil(), 0.0, 5.0, 1, cfg);
  EXPECT_EQ(h.mode, ModeTarget::GyroDR);
  EXPECT_DOUBLE_EQ(h.target_yaw_deg, 22.0);   // tabel dibalik: A +22
  EXPECT_DOUBLE_EQ(hitungTargetOrbit(kumpul.hasil(), 0.0, 5.0, 2, cfg).target_yaw_deg, -18.0);
  EXPECT_DOUBLE_EQ(hitungTargetOrbit(kumpul.hasil(), 0.0, 5.0, 3, cfg).target_yaw_deg, -35.0);

  // Dua tiang (plus satu ber-conf rendah yang diabaikan): tengah = 320 -> lurus.
  kumpul.tambahTiang({{200.0, 100.0, 10.0, 80.0, 0.9}, {440.0, 100.0, 10.0, 80.0, 0.8},
                      {600.0, 100.0, 10.0, 80.0, 0.2}}, cfg);
  ASSERT_TRUE(kumpul.hasil().gawang_valid);
  h = hitungTargetOrbit(kumpul.hasil(), 0.0, 5.0, 1, cfg);
  EXPECT_EQ(h.mode, ModeTarget::Gawang);
  EXPECT_NEAR(h.target_yaw_deg, 5.0, 1e-9);
}

TEST(TargetOrbit, KiperConfTertinggiDanEma)
{
  StrategyConfig cfg;
  PengumpulTarget kumpul;
  kumpul.tambahKiper({{100.0, 0, 40.0, 0, 0.45}}, cfg);   // di bawah ambang 0,50
  EXPECT_FALSE(kumpul.hasil().kiper_valid);
  kumpul.tambahKiper({{100.0, 0, 40.0, 0, 0.55}, {300.0, 0, 60.0, 0, 0.9}}, cfg);
  ASSERT_TRUE(kumpul.hasil().kiper_valid);
  EXPECT_DOUBLE_EQ(kumpul.hasil().kiper_cx, 300.0);
  kumpul.tambahKiper({{400.0, 0, 60.0, 0, 0.9}}, cfg);
  EXPECT_NEAR(kumpul.hasil().kiper_cx, 0.45 * 400.0 + 0.55 * 300.0, 1e-9);
  EXPECT_EQ(kumpul.hasil().frame_kiper, 2);
}

// ---- yaw gyro ------------------------------------------------------------------------------

TEST(GyroYaw, BiasDibuangLaluPutaranTerhitung)
{
  GyroYaw g;
  const double dt = 0.008;
  const double bias = 0.01;   // rad/s
  double t = 100.0;
  g.update(bias, t, true, 1.0, 2.0, 0.1);
  for (int i = 0; i < 250; ++i)   // 2 s diam
    g.update(bias, t += dt, true, 1.0, 2.0, 0.1);
  ASSERT_TRUE(g.biasReady());
  EXPECT_NEAR(g.biasDegS(), bias * 180.0 / M_PI, 1e-3);
  g.reset();
  for (int i = 0; i < 125; ++i)   // 1 s memutar ke kiri 0,5 rad/s
    g.update(0.5 + bias, t += dt, false, 1.0, 2.0, 0.1);
  EXPECT_NEAR(g.yawDeg(), 0.5 * 180.0 / M_PI, 0.05);

  // Tanda sumbu bisa dibalik tanpa merusak bias.
  g.reset();
  for (int i = 0; i < 125; ++i)
    g.update(0.5 + bias, t += dt, false, -1.0, 2.0, 0.1);
  EXPECT_NEAR(g.yawDeg(), -0.5 * 180.0 / M_PI, 0.05);
}

TEST(GyroYaw, CelahWaktuDijepitDanPesanGandaDiabaikan)
{
  GyroYaw g;
  g.update(0.0, 0.0, true, 1.0, 2.0, 0.1);
  g.update(1.0, 2.0, false, 1.0, 2.0, 0.1);   // celah 2 s dihitung 0,1 s saja
  EXPECT_NEAR(g.yawDeg(), 0.1 * 180.0 / M_PI, 1e-9);
  g.update(1.0, 2.0, false, 1.0, 2.0, 0.1);   // stempel sama: diabaikan
  EXPECT_NEAR(g.yawDeg(), 0.1 * 180.0 / M_PI, 1e-9);
  EXPECT_FALSE(g.biasReady());
}

TEST(Normalisasi, Derajat)
{
  EXPECT_DOUBLE_EQ(normalizeDeg(190.0), -170.0);
  EXPECT_DOUBLE_EQ(normalizeDeg(-190.0), 170.0);
  EXPECT_DOUBLE_EQ(normalizeDeg(180.0), 180.0);
}

// Menera offset Init Pose walking module ke pose action page 2 (WALKING_READY).
//
// Memakai op3_kinematics_dynamics ASLI -- fungsi hitungnya menyalin persis
// WalkingModule::computeNeutralLegAngle(), jadi hasilnya adalah stance yang
// benar-benar akan dipakai robot saat walking module aktif dan diam.
//
// Cari 7 bilangan (x, y, z, roll, pitch, yaw, hip_pitch) yang membuat stance itu
// sedekat mungkin dengan 12 sudut sendi kaki page 2. Pencariannya Nelder-Mead
// sederhana dengan beberapa titik awal, karena IK-nya tidak bisa dibalik analitik.
#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

using robotis_op::OP3KinematicsDynamics;

static OP3KinematicsDynamics *kd;
static const char *NAMES[12] = {
  "r_hip_yaw","r_hip_roll","r_hip_pitch","r_knee","r_ank_pitch","r_ank_roll",
  "l_hip_yaw","l_hip_roll","l_hip_pitch","l_knee","l_ank_pitch","l_ank_roll"};

// Sudut sendi kaki page 2 (radian), hasil dekode motion_4095_CHRONUS.bin.
static double TARGET[12];

// Salinan persis WalkingModule::computeNeutralLegAngle().
static bool neutralLegAngle(const double *p, double *neutral)
{
  const double x=p[0], y=p[1], z=p[2], r=p[3], pi_=p[4], a=p[5], hip=p[6];
  double leg_length = kd->thigh_length_m_ + kd->calf_length_m_ + kd->ankle_length_m_;
  double ep[12];
  ep[0]=x;      ep[1]=-y/2;  ep[2]=z-leg_length;
  ep[3]=-r/2;   ep[4]=pi_;   ep[5]=-a/2;
  ep[6]=x;      ep[7]= y/2;  ep[8]=z-leg_length;
  ep[9]= r/2;   ep[10]=pi_;  ep[11]= a/2;
  if (!kd->calcInverseKinematicsForRightLeg(&neutral[0], ep[0],ep[1],ep[2],ep[3],ep[4],ep[5])) return false;
  if (!kd->calcInverseKinematicsForLeftLeg (&neutral[6], ep[6],ep[7],ep[8],ep[9],ep[10],ep[11])) return false;
  neutral[2] -= kd->getJointDirection("r_hip_pitch") * hip;
  neutral[8] -= kd->getJointDirection("l_hip_pitch") * hip;
  return true;
}

static double cost(const std::vector<double> &p)
{
  double n[12];
  if (!neutralLegAngle(p.data(), n)) return 1e9;
  double s = 0.0;
  for (int i = 0; i < 12; i++) { double d = n[i] - TARGET[i]; s += d * d; }
  return s;
}

static double costBounded(const std::vector<double> &p);

// Nelder-Mead
static std::vector<double> minimize(std::vector<double> x0, double step)
{
  const int n = x0.size();
  std::vector<std::vector<double>> S; std::vector<double> F;
  S.push_back(x0);
  for (int i = 0; i < n; i++) { auto v = x0; v[i] += step; S.push_back(v); }
  for (auto &v : S) F.push_back(costBounded(v));
  for (int it = 0; it < 20000; it++) {
    std::vector<int> idx(n + 1); for (int i = 0; i <= n; i++) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](int a, int b){ return F[a] < F[b]; });
    std::vector<std::vector<double>> S2; std::vector<double> F2;
    for (int i : idx) { S2.push_back(S[i]); F2.push_back(F[i]); }
    S = S2; F = F2;
    if (std::fabs(F[n] - F[0]) < 1e-18) break;
    std::vector<double> c(n, 0.0);
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) c[j] += S[i][j] / n;
    auto lin = [&](double t){ std::vector<double> v(n);
      for (int j = 0; j < n; j++) v[j] = c[j] + t * (c[j] - S[n][j]); return v; };
    auto xr = lin(1.0); double fr = costBounded(xr);
    if (fr < F[0]) { auto xe = lin(2.0); double fe = costBounded(xe);
      if (fe < fr) { S[n] = xe; F[n] = fe; } else { S[n] = xr; F[n] = fr; } }
    else if (fr < F[n - 1]) { S[n] = xr; F[n] = fr; }
    else { auto xc = lin(-0.5); double fc = costBounded(xc);
      if (fc < F[n]) { S[n] = xc; F[n] = fc; }
      else { for (int i = 1; i <= n; i++) { for (int j = 0; j < n; j++)
               S[i][j] = S[0][j] + 0.5 * (S[i][j] - S[0][j]); F[i] = costBounded(S[i]); } } }
  }
  return S[0];
}

// Batas wajar tiap offset. Di luar batas diberi denda besar supaya pencarian
// tidak kabur ke jawaban yang cocok secara angka tapi mustahil secara mekanik
// (percobaan pertama tanpa batas menghasilkan x_offset -67 mm).
static const double LO[7] = {-0.05, 0.000, -0.005, -0.10, -0.30, -0.10, 0.00};
static const double HI[7] = { 0.05, 0.080,  0.120,  0.10,  0.30,  0.10, 0.35};

static double costBounded(const std::vector<double> &p)
{
  double pen = 0.0;
  for (int i = 0; i < 7; i++) {
    if (p[i] < LO[i]) pen += 1e3 * (LO[i] - p[i]) * (LO[i] - p[i]);
    if (p[i] > HI[i]) pen += 1e3 * (p[i] - HI[i]) * (p[i] - HI[i]);
  }
  return cost(p) + pen;
}

static void report(const char *judul, const std::vector<double> &p)
{
  const char *PN[7] = {"init_x_offset","init_y_offset","init_z_offset",
                       "init_roll_offset","init_pitch_offset","init_yaw_offset","hip_pitch_offset"};
  double n[12];
  bool ok = neutralLegAngle(p.data(), n);
  printf("\n================ %s ================\n", judul);
  if (!ok) { printf("  IK GAGAL -- pose tidak terjangkau\n"); return; }
  for (int i = 0; i < 7; i++)
    printf("  %-18s %+10.6f rad  = %+8.3f deg\n", PN[i], p[i], p[i] * 180.0 / M_PI);
  double s2 = 0; for (int i = 0; i < 12; i++) { double d = n[i]-TARGET[i]; s2 += d*d; }
  printf("  RMS selisih ke page 2: %.3f deg\n", sqrt(s2/12.0)*180.0/M_PI);
  printf("\n  %-14s %9s %10s %9s\n", "sendi", "page2", "stance", "selisih");
  for (int i = 0; i < 12; i++)
    printf("  %-14s %+9.3f %+10.3f %+9.3f  deg\n", NAMES[i],
           TARGET[i]*180/M_PI, n[i]*180/M_PI, (n[i]-TARGET[i])*180/M_PI);
}

int main(int argc, char **argv)
{
  if (argc < 13) { fprintf(stderr, "pakai: fit <12 sudut target radian>\n"); return 2; }
  for (int i = 0; i < 12; i++) TARGET[i] = atof(argv[i + 1]);
  kd = new OP3KinematicsDynamics(robotis_op::WholeBody);
  printf("panjang tungkai: thigh=%.4f calf=%.4f ankle=%.4f total=%.4f m\n",
         kd->thigh_length_m_, kd->calf_length_m_, kd->ankle_length_m_,
         kd->thigh_length_m_ + kd->calf_length_m_ + kd->ankle_length_m_);

  // param.yaml yang dipakai robot sekarang, sebagai pembanding.
  const double D2R = M_PI / 180.0;
  std::vector<double> now = {-0.015, 0.015, 0.075, 0.0, 4.0*D2R, 0.0, 8.0*D2R};
  report("param.yaml SEKARANG (x-0.015 y0.015 z0.075 pitch4 hip8)", now);

  // Tahap 1: sapuan kasar pada empat penentu bentuk stance (x, y, z, hip_pitch),
  // orientasi telapak dikunci nol dulu supaya tidak saling menukar peran.
  std::vector<double> best4; double best4f = 1e18;
  for (double x = -0.04; x <= 0.04001; x += 0.005)
    for (double y = 0.0; y <= 0.06001; y += 0.005)
      for (double z = -0.004; z <= 0.10001; z += 0.002)
        for (double h = 0.0; h <= 0.30001; h += 0.02) {
          std::vector<double> p = {x, y, z, 0, 0, 0, h};
          double f = costBounded(p);
          if (f < best4f) { best4f = f; best4 = p; }
        }
  // Tahap 2: haluskan ketujuhnya dari titik terbaik tahap 1, beberapa kali
  // dengan langkah mengecil supaya tidak berhenti di tepi simpleks.
  std::vector<double> best = best4;
  for (double st : {0.02, 0.008, 0.003, 0.001, 0.0003}) {
    auto r = minimize(best, st);
    if (costBounded(r) < costBounded(best)) best = r;
  }
  report("HASIL TERA ke page 2 (WALKING_READY)", best);

  // Peta z_offset -> sudut lutut, dan sampai mana IK masih terjangkau saat gait
  // menaik-turunkan panggul. swing_top_down (z_swap) menambah/mengurangi panjang
  // kaki tiap langkah, jadi z_offset harus menyisakan ruang sebesar itu.
  const double Z_SWAP = 0.006;   // param.yaml swing_top_down
  printf("\n================ peta z_offset -> lutut (x,y,pitch,hip ikut hasil tera) ================\n");
  printf("  %10s %12s %12s %14s\n", "z_offset", "r_knee deg", "r_hip deg", "IK saat z_swap");
  for (double z = -0.002; z <= 0.0901; z += 0.004) {
    std::vector<double> p = best; p[2] = z;
    double n[12];
    if (!neutralLegAngle(p.data(), n)) { printf("  %10.4f   IK GAGAL\n", z); continue; }
    // kaki paling panjang terjadi saat panggul turun sejauh z_swap
    std::vector<double> q = p; q[2] = z - Z_SWAP;
    double m[12];
    const char *ok = neutralLegAngle(q.data(), m) ? "aman" : "GAGAL (mentok)";
    printf("  %10.4f %12.2f %12.2f   %14s\n", z, n[3]*180/M_PI, n[2]*180/M_PI, ok);
  }
  // Kalau yang digeser justru page 2 (bukan parameternya), inilah nilai raw yang
  // harus ditulis ke halaman itu supaya serah-terima ke walking module mulus.
  // x/y/pitch/hip dipertahankan dari param.yaml; hanya z_offset yang disapu.
  printf("\n================ nilai RAW page 2 kalau stance walking dipakai apa adanya ================\n");
  printf("(x=%.4f y=%.4f pitch=%.1fdeg hip=%.1fdeg dari param.yaml; hanya z_offset berubah)\n",
         now[0], now[1], now[4]*180/M_PI, now[6]*180/M_PI);
  printf("  %-9s", "z_offset");
  for (int i = 0; i < 12; i++) printf(" %10s", NAMES[i]);
  printf("\n");
  for (double z : {0.015, 0.020, 0.030, 0.040, 0.050, 0.075}) {
    std::vector<double> p = now; p[2] = z;
    double n[12];
    if (!neutralLegAngle(p.data(), n)) { printf("  %-9.3f  IK GAGAL\n", z); continue; }
    printf("  %-9.3f", z);
    for (int i = 0; i < 12; i++) printf(" %10d", (int)lround(2048.0 + n[i] * 2048.0 / M_PI));
    printf("\n");
  }
  printf("  %-9s", "page2 kini");
  for (int i = 0; i < 12; i++) printf(" %10d", (int)lround(2048.0 + TARGET[i] * 2048.0 / M_PI));
  printf("\n");
  return 0;
}

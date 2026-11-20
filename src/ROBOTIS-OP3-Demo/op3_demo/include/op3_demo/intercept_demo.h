/*
 * DEMO NORMAL + INTERCEPT  (tombol D58)
 *
 * Demo ini TIDAK membuat gerakan sendiri. Ia hanya mengatur KAPAN gerakan yang
 * sudah tersimpan dipanggil:
 *
 *   - stance/geser/mutar  -> preset parameter walking dari walking_versions.json
 *                            ("Jalan Ke surga", "geser kiri/kanan", "mutar kiri/kanan")
 *   - tahan bola & bangun -> action page 14 ("menahan bola") dan 10/12 dari motion_*.bin
 *
 * D58 TIDAK menendang (operator 2026-10-30). Bola mendekat sampai ~1 m ->
 * page 14; bola menjauh lagi -> kembali WALKING_READY (page 2); bola sampai di
 * kaki (jarak tendang) -> tahan 3 dtk, lalu kembali dan menunggu bola berikutnya.
 *
 * Sumber datanya juga yang sudah ada: BallTracker (deteksi "Bola-baru" lewat
 * /ball_detector_node/circle_set), /robotis/goal_joint_states untuk servo 19
 * (head_pan), dan /robotis/open_cr/imu untuk pitch/roll/yaw.
 *
 * Ambang jatuh disalin apa adanya dari SoccerDemo/BallFollower
 * (demo D56) -- lihat komentar di masing-masing konstanta.
 */

#ifndef INTERCEPT_DEMO_H_
#define INTERCEPT_DEMO_H_

#include <deque>
#include <map>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <eigen3/Eigen/Eigen>

#include "op3_action_module_msgs/srv/is_running.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_controller_msgs/srv/set_joint_module.hpp"

#include "op3_demo/op_demo.h"
#include "op3_demo/ball_tracker.h"
#include "robotis_math/robotis_linear_algebra.h"

namespace robotis_op
{

class InterceptDemo : public OPDemo
{
 public:
  // Satu state gerakan aktif pada satu waktu. Urutan enum tidak berarti apa-apa;
  // prioritas ditentukan di process(), bukan di sini.
  enum State
  {
    IDLE = 0,
    INIT,            // masuk stance dari versi "geser kiri" (amplitudo nol)
    DEMO_NORMAL,     // tracking bola: geser kiri / stance / geser kanan
    INTERCEPT_LEFT,  // koreksi yaw memakai save "mutar kiri"
    INTERCEPT_RIGHT, // koreksi yaw memakai save "mutar kanan"
    BLOCK,           // page 14 "menahan bola", tahan sampai bola menjauh / 3 dtk di kaki
    FALL_FRONT,      // page 10
    FALL_BACK,       // page 12
    RECOVERY_INIT,   // kembalikan modul + stance setelah action selesai
  };

  // Sama persis dengan SoccerDemo::Stand_Status supaya pembacaan pitch bisa
  // dibandingkan langsung dengan demo D56.
  enum StandStatus
  {
    Stand = 0,
    Fallen_Forward = 1,
    Fallen_Behind = 2,
  };

  InterceptDemo();
  ~InterceptDemo() override;

  void setNode(rclcpp::Node::SharedPtr node);
  // Siapa pemilik kepala -- disamakan dengan SoccerDemo oleh demo_node.
  //   false : head_tracking_node yang tracking + searching (demo.launch.xml)
  //   true  : BallTracker di node ini yang tracking + searching (yolo_demo*)
  void setUsingHeadControl(bool use_head_control);

  void setDemoEnable() override;
  void setDemoDisable() override;

  // FIXED INIT POSE di luar siklus demo: dipakai demo_node sesudah launch supaya
  // pose awal robot berasal dari template preset_init_, bukan page mentah.
  void applyFixedInitPose();
  // true setelah SATU action page selesai sukses (/robotis/movement_done "action")
  // sejak node ini hidup -- di awal launch itu page init WALKING_READY milik
  // op3_manager. demo_node menunggu ini sebelum applyFixedInitPose().
  bool bootInitPageDone() const { return boot_page_done_; }

  void process();

  rclcpp::Node::SharedPtr node_;

 protected:
  //== ambang warisan D56 (JANGAN diubah tanpa mengubah D56 juga) =============
  // soccer_demo.cpp: FALL_FORWARD_LIMIT(60), FALL_BACK_LIMIT(-60), alpha 0.4.
  const double FALL_FORWARD_LIMIT;
  const double FALL_BACK_LIMIT;
  const double IMU_FILTER_ALPHA;
  // ball_follower.cpp: NOT_FOUND_THRESHOLD(50) -- ambang "bola hilang".
  const int NOT_FOUND_THRESHOLD;
  const int SPIN_RATE;

  // callback
  void imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void movementDoneCallback(const std_msgs::msg::String::SharedPtr msg);

  // parameter & data tersimpan
  void declareParameters();
  bool loadWalkingVersions();
  bool hasPreset(const std::string &name) const;

  // gerakan tersimpan: walking
  void startPreset(const std::string &name);
  void stopWalking();
  bool isGeserPreset(const std::string &name) const;
  bool geserSoftStartDone() const;
  void applyPreset(const std::string &name, bool moving);
  // Soft start milik save (parameter ROS /op3_walking_module soft_start_time,
  // bukan field WalkingParam) dipasang SEBELUM "start". Menunggu jawaban.
  bool applySoftStartTime(double seconds);
  // Setel satu parameter node /op3_walking_module dan tunggu jawabannya.
  bool setWalkingNodeParam(const rclcpp::Parameter &param);
  // reuse_walking_bias di walking module; hanya memanggil service bila berubah.
  bool setWalkingBiasReuse(bool on);

  // gerakan tersimpan: action page
  void playPage(int page_num);
  void stopAction();
  bool isActionFinished();
  bool isActionRunning();

  // modul
  void setBodyModuleToDemo(const std::string &body_module);
  void setModuleToDemo(const std::string &module_name);
  void callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule &modules);
  void parseJointNameFromYaml(const std::string &path);

  // logika
  double ballPercent();
  bool ballIsFresh() const;
  // Posisi aktual servo 20 (head_tilt) dalam derajat servo; -1 kalau belum terbaca.
  double servo20Deg() const;
  void enterState(State next);
  const char *stateName(State s) const;
  void setRGBLED(int blue, int green, int red);

  BallTracker ball_tracker_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr movement_done_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr walking_command_pub_;
  rclcpp::Publisher<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr walking_param_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr action_page_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr head_tracker_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reset_yaw_pub_;
  void resetYaw(const std::string &reason);
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr rgb_led_pub_;

  std::map<std::string, op3_walking_module_msgs::msg::WalkingParam> presets_;
  // soft_start_time per save (kunci nama huruf kecil). Save tanpa kunci itu tidak
  // tercatat di sini -> nilai global walking module dibiarkan.
  std::map<std::string, double> preset_soft_start_;
  // Nilai soft_start_time yang terakhir BERHASIL dipasang D58; -1 = belum tahu
  // (Studio bisa mengubahnya kapan saja, jadi direset tiap D58 ON).
  double applied_soft_start_;
  bool   boot_page_done_;
  // Geser ditahan (tidak di-start) sampai soft start-nya terpasang; percobaan
  // ulang dijarangkan supaya service call tidak menahan loop tiap siklus.
  std::chrono::steady_clock::time_point soft_start_retry_at_;
  std::map<int, std::string> id_joint_table_;

  // parameter yang boleh disetel operator (lihat declareParameters())
  double yaw_limit_deg_;   // pemicu: yaw > +batas -> mutar kanan, yaw < -batas -> mutar kiri
  double yaw_release_deg_; // lepas: mutar kanan berhenti di yaw <= +lepas, mutar kiri di >= -lepas
  // Diagnosa overshoot: yaw saat stop dikirim vs yaw setelah gait benar-benar
  // tuntas. Selisihnya = sisa putaran yang terjadi SESUDAH perintah stop.
  bool   overshoot_pending_;
  bool   overshoot_was_right_;
  double overshoot_stop_yaw_;
  double intercept_max_sec_;
  // Histeresis geser DEMO_NORMAL (0% = bola paling kiri, 100% = paling kanan):
  //   geser kiri : mulai saat bola <  left_trigger,  berhenti saat bola >= left_release
  //   geser kanan: mulai saat bola >  right_trigger, berhenti saat bola <= right_release
  double left_trigger_pct_;
  double left_release_pct_;
  double right_trigger_pct_;
  double right_release_pct_;

  // Sub-state geser di dalam DEMO_NORMAL. Diingat supaya geser yang sudah aktif
  // tidak berhenti hanya karena bola melewati batas pemicunya.
  enum Lateral { LATERAL_NONE = 0, LATERAL_LEFT, LATERAL_RIGHT };
  Lateral lateral_;
  double pan_range_deg_;

  //== TAHAN BOLA (operator 2026-10-30, menggantikan tendang) =================
  // Jarak bola dibaca dari posisi AKTUAL servo ID 20 (head_tilt) dalam DERAJAT
  // SERVO (skala 0..360 = raw x 360/4096), vision dan tracking kepala yang sama
  // dengan pemicu tendang dulu. Makin menunduk = makin besar = bola makin dekat.
  //   > block_servo20_deg_         : bola ~1 m -> page 14
  //   < block_release_servo20_deg_ : bola menjauh -> kembali WALKING_READY
  //   > feet_servo20_deg_          : bola di kaki (ambang tendang lama, 233)
  //                                  -> tahan block_hold_sec_ lalu kembali
  double block_servo20_deg_;
  double block_release_servo20_deg_;
  double feet_servo20_deg_;
  double block_hold_sec_;
  int    page_block_;
  // Sesudah menahan bola, bola yang sama (masih di kaki) tidak memicu page 14
  // lagi. Pemicu dipasang ulang begitu bola TERLIHAT lebih jauh dari titik lepas.
  bool   block_armed_;
  bool   block_page_sent_;
  bool   block_hold_started_;
  std::chrono::steady_clock::time_point block_hold_until_;
  int    min_state_ticks_;
  double action_timeout_sec_;
  int    init_settle_ticks_;
  // Page action acuan FIXED INIT POSE (WALKING_READY). Dimainkan sebelum kaki
  // diserahkan ke walking_module, sebab walking_module memakai pose yang sedang
  // dipegang saat diaktifkan sebagai dasar gait -- tanpa page acuan ini, pose
  // akhir tendangan/bangun/gerakan terakhir yang jadi "init pose". 0 = mati.
  int    init_anchor_page_;
  // arm_swing_gain yang dikirim setiap INIT POSE (diam: bola di tengah, bola
  // hilang, D58 ON, sesudah page/soft stop). Operator 2026-09-19: 0.4.
  double init_arm_swing_gain_;
  // Ayunan lengan SAAT DIAM (walking module idle_arm_swing_*): amplitudo =
  // init_arm_swing_gain_ x IDLE_ARM_SWING_REF_DEG (0,4 x 25 = 10 deg, pilihan
  // operator 2026-09-19), satu ayunan penuh idle_arm_swing_period_ detik.
  double idle_arm_swing_period_;
  static constexpr double IDLE_ARM_SWING_REF_DEG = 25.0;
  void setIdleArmSwing(bool on);
  // Goyang pan kepala (servo 19) saat diam, pengganti ayunan lengan (operator
  // 2026-10-30). Dijalankan head_tracking_node lewat /ball_tracker/command
  // "sway_on"/"sway_off"; dikirim hanya saat berubah.
  void setHeadSway(bool on);
  bool head_sway_on_;
  // Page sesudah mana INIT langsung ke init pose preset_init_ TANPA page acuan
  // (walking module memakai ulang dasar stance lama). Page yang pose akhirnya jauh
  // dari page acuan -- mis. 12 BANGBEL, siku 150 derajat -- tetap lewat page acuan,
  // karena walking module tidak memegang siku/bahu roll untuk meluruskannya.
  std::vector<int64_t> direct_init_pages_;
  int    transition_hold_ms_;        // INTERCEPT -> DEMO_NORMAL
  int    intercept_enter_hold_ms_;   // DEMO_NORMAL -> INTERCEPT
  std::string preset_init_, preset_left_, preset_right_, preset_turn_left_, preset_turn_right_;
  // page_getup_front_ <= 0 = bangun depan MATI (operator 2026-10-30, sementara):
  // jatuh ke depan -> gerakan dihentikan, robot diam sampai diberdirikan lagi.
  int page_getup_front_, page_getup_back_;
  int upright_ticks_;   // siklus berturut-turut tegak selama FALL_* tanpa page

  // keadaan
  State state_;
  int   state_ticks_;
  // FIXED INIT POSE sudah dikirim di state INIT ini (sekali, setelah page acuan
  // tuntas), dan tick saat ia dikirim -- init_settle_ticks_ dihitung dari situ.
  bool  init_stance_sent_;
  bool  init_direct_;          // INIT ini langsung ke init pose (sesudah page)
  bool  walking_bias_reuse_on_;
  int   last_page_played_;
  int   init_stance_tick_;
  int   stand_state_;
  double present_pitch_;
  double present_roll_;
  double present_yaw_;
  bool  imu_seen_;
  double head_pan_;
  double head_tilt_;
  bool  head_joint_seen_;
  int   tracking_status_;
  int   ball_lost_count_;
  // Posisi AKTUAL servo 20 (head_tilt) dari /robotis/present_joint_states.
  double present_tilt_;
  bool  present_tilt_seen_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr present_joint_sub_;
  void presentJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // Sudut sendi (rad) -> derajat servo, memakai konstanta device XM430-W350
  // (raw 2048 = 0 rad, raw 4095 = +pi): servo_deg = 180 + rad * 2047*360/(4096*pi).
  static double jointRadToServoDeg(double rad);
  std::string current_preset_;
  bool  walking_running_;
  bool  action_running_;
  int   pending_page_;
  std::chrono::steady_clock::time_point action_deadline_;
  // Jeda sekali-pakai saat berpindah DEMO_NORMAL <-> INTERCEPT_*, supaya gait
  // lama benar-benar berhenti sebelum gait baru dimulai.
  std::chrono::steady_clock::time_point transition_hold_until_;
  // Gait tidak berhenti seketika saat "stop" dikirim: walking_module baru
  // menutup di batas siklus berikutnya, jadi butuh sampai satu period_time
  // penuh. Selama itu kaki MASIH bergerak, dan tidak boleh ada preset baru
  // yang menimpanya.
  std::chrono::steady_clock::time_point walk_settle_until_;
  double current_period_time_;

  // GESER KIRI/KANAN (operator 2026-09-18):
  //  - geser yang sudah mulai TIDAK dihentikan oleh logika geser sendiri (bola
  //    lewat titik lepas / di tengah / hilang) sebelum soft start-nya selesai;
  //  - sesudah geser berhenti dan gaitnya tuntas (walk_settle_until_), geser
  //    berikutnya -- termasuk arah yang sama -- menunggu geser_gap_sec_ lagi.
  // Intercept yaw dan tahan bola tidak ikut ditunda.
  double geser_gap_sec_;                                   // intercept_geser_gap_sec

  //== BATAS WAKTU GESER BERSAMA (operator 2026-09-19) ======================
  // lateral_net_sec_ = waktu geser kiri - waktu geser kanan sejak D58 ON (detik,
  // kiri positif). Geser kiri boleh selama < +geser_max_sec_, geser kanan selama
  // > -geser_max_sec_. Jadi sesudah kiri 3 dtk, kanan boleh sampai 13 dtk, dan
  // sebaliknya. Mencapai batas -> SOFT STOP (bukan stop biasa).
  double geser_max_sec_;                                   // intercept_geser_max_sec
  double lateral_net_sec_;
  std::chrono::steady_clock::time_point lateral_tick_;
  // Sesudah soft stop tuntas, stance dikembalikan ke init pose preset_init_.
  bool soft_stop_pending_init_;
  void softStopWalking(const char *why);
  bool geserBudgetLeft(const std::string &name) const;
  double geser_soft_start_sec_;                            // soft start geser yang berjalan
  std::chrono::steady_clock::time_point geser_started_at_;
  std::chrono::steady_clock::time_point geser_next_allowed_;
  State return_state_;

  //== BATAS GESER TERKALIBRASI, MODE KIPER (operator 2026-11-11) ===========
  // Area kiper 1,8 m. scripts/calib_batas_geser.py mengukur WAKTU EFEKTIF (integral
  // skala soft start yang naik linear) dari tengah ke tepi tiap arah + ekor per
  // segmen (langkah yang masih diselesaikan sesudah stop), lewat geser di Studio.
  // lateral_pos_: -1 = tepi kanan, 0 = tengah (posisi saat D58 ON), +1 = tepi kiri.
  // Batas = 1 - toleransi/setengah lebar. Tanpa YAML valid -> batas waktu lama.
  bool   gk_edge_ok_;
  double gk_edge_left_sec_, gk_edge_right_sec_;            // waktu efektif tengah->tepi
  double gk_edge_tail_left_sec_, gk_edge_tail_right_sec_;  // ekor per segmen (dtk efektif)
  double gk_edge_half_width_m_;
  // Jarak tengah->titik tepi kalibrasi yang DIUKUR operator (Studio, 2026-11-12);
  // tanpa ukuran = gk_edge_half_width_m_. Posisi 1,0 = titik itu.
  double gk_edge_dist_left_m_, gk_edge_dist_right_m_;
  double gk_edge_tolerance_m_;                             // intercept_gk_edge_tolerance_m (bawaan)
  // Per arah (2026-11-12, dari data kalibrasi: kaki + galat 2 sigma), m sebelum garis area.
  double gk_edge_tol_left_m_, gk_edge_tol_right_m_;
  double lateral_pos_;
  bool loadEdgeCalibration(const std::string &path);
  bool gkEdgeActive() const { return gk_mode_ == true && gk_edge_ok_ == true; }
  // Batas pecahan (0..1) arah itu sesudah toleransi.
  double gkEdgeLimit(bool left) const;
  // lateral_pos_ dalam meter dari tengah (+ = kiri).
  double gkEdgePosM() const;
  // Pecahan posisi yang masih akan ditempuh bila geser `left` dihentikan sekarang
  // (running=true: ramp soft stop + ekor) atau dimulai lalu dihentikan secepatnya
  // (running=false: soft start penuh + ramp turun + ekor).
  double gkEdgeReserve(bool left, bool running, double soft_sec) const;
  // Tambahkan gerak sesudah stop (ekor, + ramp turun untuk soft stop) ke lateral_pos_.
  void gkEdgeAddStopMotion(bool soft_stop);

  //== MODE KIPER (KSTI 2026, operator 2026-11-07) ==========================
  // Panduan ksti2026_GOALKEEPER_adaptasi_minimal. Gerakan TIDAK berubah; yang
  // berubah hanya SASARAN dan PEMICU:
  //   - sasaran geser + kepala = striker lawan ("robot") dan bola yang sedang
  //     didekatinya, dari /ball_detector_node/detections -- bukan bola terbesar;
  //   - striker di bola lalu menendang -> page 14 seketika, maks gk_block_max_sec_;
  //     bola tertahan di kaki -> tahan gk_block_hold_sec_ -> page 2 -> ulangi;
  //   - batas geser bersama gk_max_sidestep_sec_ (bukan geser_max_sec_);
  //   - tanpa magnetometer: resetYaw() tidak dipanggil (yaw D58 memang 0).
  // intercept_gk_mode=false -> semua cabang ini mati, D58 persis seperti dulu.
  struct GkBox
  {
    bool   valid = false;
    double cx = 0.0, cy = 0.0, w = 0.0, h = 0.0;
  };
  struct GkTarget
  {
    double x = 0.0, y = 0.0;   // titik halang (px)
    GkBox  ball;               // bola aktif (valid=false kalau tak ada)
    bool   lawan_siap = false; // striker rapat di bola (siap menendang)
    int    kasus = 4;          // 1..4 sesuai panduan
  };
  // Port 1:1 cari_target_penghalangan_x() dari panduan (4 kasus).
  GkTarget cariTargetPenghalanganX(const GkBox &lawan, const std::vector<GkBox> &bola,
                                   double target_x_sebelumnya, double target_y_sebelumnya) const;
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  bool gkTargetFresh() const;
  // Skala yang sama dengan ballPercent(): 0% = paling kiri, 100% = paling kanan.
  double gkTargetPercent() const;
  bool gkKickPending() const;
  // Batas geser bersama yang berlaku (gk_max_sidestep_sec_ di mode kiper).
  double geserMaxSec() const;
  void sendHeadTrackerCmd(const std::string &cmd);

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gk_target_pub_;

  bool   gk_mode_;
  double gk_min_conf_ball_;
  double gk_min_conf_robot_;
  double gk_ema_alpha_;
  double gk_ball_x_weight_;
  double gk_striker_x_weight_;
  double gk_striker_close_ratio_;
  double gk_max_sidestep_sec_;
  double gk_kick_velocity_px_;     // px per frame (Δcy turun / Δw membesar)
  double gk_same_ball_px_;         // lompatan > ini antar-frame = bola lain, bukan tendangan
  double gk_kick_ready_window_sec_;// tendangan sah kalau striker siap <= selama ini sebelumnya
  double gk_kick_valid_sec_;       // tendangan yang belum terpakai kedaluwarsa sesudah ini
  double gk_block_max_sec_;
  double gk_block_hold_sec_;
  double gk_frame_width_;
  double gk_frame_height_;
  double gk_camera_fps_;
  double gk_lost_sec_;             // target tanpa deteksi > ini = hilang

  bool   gk_has_target_;
  double gk_target_x_, gk_target_y_;          // titik halang (px, sesudah EMA)
  // Toleransi geser mode kiper (operator 2026-11-09): simpangan samping target
  // dalam METER = jarak bola aktif (kalibrasi servo 20) x tan(arah target).
  // Jarak tak terukur (bola tertutup striker) -> cadangan dalam DERAJAT.
  double gk_target_dist_m_;                    // jarak bola aktif, -1 = tak terukur
  double gk_lateral_trigger_m_, gk_lateral_release_m_;
  double gk_trigger_deg_, gk_release_deg_;
  // Simpangan bertanda (+ = target di KANAN kiper). in_meters=false -> derajat.
  double gkLateralError(bool &in_meters) const;
  int    gk_kasus_;
  std::chrono::steady_clock::time_point gk_target_time_;
  std::chrono::steady_clock::time_point gk_lawan_siap_time_;
  bool   gk_lawan_siap_seen_;
  // Bola aktif frame sebelumnya, untuk deteksi tendangan.
  GkBox  gk_prev_ball_;
  rclcpp::Time gk_prev_stamp_;
  bool   gk_prev_stamp_valid_;
  // Tendangan terdeteksi -> kunci sasaran ke bola itu (ikuti kontinuitasnya,
  // abaikan striker) sampai BLOCK selesai, supaya kepala ikut bola yang datang.
  bool   gk_kick_;
  std::chrono::steady_clock::time_point gk_kick_time_;
  bool   gk_lock_;
  GkBox  gk_lock_ball_;
  std::chrono::steady_clock::time_point block_started_at_;

  //== JARAK BOLA dari kalibrasi servo 20 (operator 2026-11-08) ==============
  // scripts/calib_jarak_kamera.py -> data/kalibrasi_jarak_kamera.yaml:
  //   s20_setara = servo20_aktual + atan((cy - cy0) / fy) [deg]
  //   jarak      = H / tan(s20_setara - 180 + theta0) - c   [m, dari ujung kaki]
  // Berlaku untuk bola MANA PUN di gambar, juga saat kepala mengikuti striker.
  bool   gk_calib_ok_;
  double gk_calib_fy_, gk_calib_cy0_, gk_calib_H_, gk_calib_theta0_deg_, gk_calib_c_;
  bool   loadDistanceCalibration(const std::string &path);
  // < 0 = tak bisa dihitung (di atas cakrawala / kalibrasi tak ada).
  double ballDistance(const GkBox &b) const;
  // Sudut depresi (deg) baris gambar y dari kalibrasi, memakai sudut kepala saat
  // gambar itu DITANGKAP (gk_cap_tilt_), bukan sudut sekarang. Dipakai bola & robot.
  double gkRowDepressionDeg(double y) const;
  // Arah ke kanan (rad, + = kanan kiper) kolom gambar x, dari pan saat tangkap.
  double gkColumnRightRad(double x) const;
  // Bola besar yang terpotong tepi bawah gambar = sudah menempel di kaki
  // (kalibrasi: bola <= 0,1 m tak pernah masuk utuh ke gambar).
  bool   ballCutAtFeet(const GkBox &b) const;

  // Pelacak bola sederhana antar-frame, supaya kecepatan mendekat bisa diukur
  // dan bola yang diam di dekat kaki tidak memicu page 14 berulang.
  struct GkTrack
  {
    GkBox  box;
    std::vector<std::pair<double, double>> hist;   // (stempel gambar dtk, jarak m)
    std::vector<double> hist_w;                    // lebar bbox (px), sejajar dengan hist
    double last_seen = 0.0;
    double vx = 0.0, vy = 0.0;      // kecepatan piksel (px/dtk) untuk prediksi pencocokan
    bool   fired = false;
    // Sejak bola mulai bergerak (operator 2026-11-10): riwayat <= 1 dtk untuk
    // mencocokkan d(t) = d0 - v t + a t^2 / 2 dan memprediksi titik berhentinya.
    std::vector<std::pair<double, double>> move_hist;
    double pred_stop_m = -1.0;      // prediksi titik berhenti terakhir (m dari ujung kaki)
    double pred_decel = 0.0;        // perlambatan yang dipakai prediksi terakhir (m/s^2)
    bool   pred_fitted = false;     // perlambatan dari data (true) atau nilai bawaan
    int    pred_ok_count = 0;       // frame BERTURUT-TURUT dengan prediksi "sampai kaki"
    // Lebar bbox & jarak saat bola mulai bergerak: bola yang SUNGGUH mendekat makin
    // besar di gambar, dan lebar tidak terpengaruh sudut kepala (2026-11-10).
    double move_start_w = 0.0;
    double move_start_d = 0.0;
    int    cepat_count = 0;         // frame BERTURUT-TURUT memenuhi syarat CEPAT
  };
  // Pembanding lebar: true bila pertumbuhan lebar bbox mendukung jarak yang turun.
  bool gkWidthConfirms(const GkTrack &tr, const GkBox &b, double d) const;
  // Jejak bola ini (frame sekarang) punya bukti gerak yang sah.
  bool gkBallHasMotion(const GkBox &b) const;

  //== PENGAMAN BIAS JARAK (operator 2026-11-10) ============================
  // Jarak dari servo 20 hanya sah bila kepala DIAM (gambar basi vs sudut kepala
  // sekarang) dan badan setegak saat kalibrasi (pitch IMU ~+11,1 deg).
  double gk_head_still_dps_;        // laju kepala > ini = bergerak
  double gk_calib_pitch_deg_;       // pitch IMU saat kalibrasi jarak
  double gk_pitch_tol_deg_;         // |pitch - kalibrasi| > ini = jarak tak sah
  double gk_max_ball_mps_;          // laju bola lebih dari ini = tukar bola / lompatan
  double gk_predict_max_m_;         // PREDIKSI hanya bila bola sedekat ini
  double present_pan_;
  //== WAKTU TANGKAP GAMBAR (operator 2026-11-12) ===========================
  // Stempel usb_cam 0.8.1 bergeser konstan 0-1 dtk (bug get_epoch_time_shift_us,
  // tv_usec / 1000.0) -- itulah "latensi ~900 ms" lama. Jadi waktu tangkap =
  // waktu TERIMA deteksi - gk_cam_latency_sec_ (diukur: calib_jarak_kamera.py
  // latensi), dan jarak dihitung dari sudut kepala SAAT ITU (riwayat 1 dtk).
  // 0 = sudut kepala sekarang (perilaku sebelum 2026-11-12).
  double gk_cam_latency_sec_;
  struct HeadSample
  {
    std::chrono::steady_clock::time_point t;
    double tilt, pan;
  };
  std::deque<HeadSample> head_hist_;
  double gk_cap_tilt_, gk_cap_pan_;   // sudut kepala saat gambar deteksi ini ditangkap
  void setCaptureHeadAngles(std::chrono::steady_clock::time_point t_recv);
  bool   head_ref_valid_;
  double head_ref_tilt_, head_ref_pan_;
  std::chrono::steady_clock::time_point head_ref_time_, head_moving_until_;
  bool   headStill() const;
  // Penghitung sampel jarak yang ditolak (dicatat berkala; direset tiap D58 ON).
  enum GkTolak { TOLAK_KEPALA = 0, TOLAK_PITCH, TOLAK_LOMPAT, TOLAK_LAJU, TOLAK_LEBAR, TOLAK_AMBIGU,
                 TOLAK_KUNCI, TOLAK_N };
  int gk_tolak_[TOLAK_N];
  std::vector<GkTrack> gk_tracks_;
  void updateBallTracks(const std::vector<GkBox> &bola, double stamp);
  // Pemicu saving: bola DIAM tidak pernah memicu.
  //   CEPAT    (2026-11-12): bola bergerak yang tiba di zona kaki dalam <= react_sec
  //            (page 14 butuh 0,56 dtk) -> page 14; 2 frame berturut (1 bila bola itu
  //            terkunci TENDANGAN piksel)
  //   PREDIKSI (2026-11-10): bola mulai bergerak dan titik
  //            berhentinya (fit d(t) kuadrat) <= reach_m -> page 14 sedini mungkin
  //   DEKAT    : jarak <= save_distance (0,6), mendekat >= slow_mps (cadangan)
  double gk_react_sec_;             // waktu tiba di zona kaki <= ini -> CEPAT
  double gk_reach_m_;               // titik berhenti sejauh ini dari ujung kaki = sampai ke kiper
  double gk_decel_prior_;           // perlambatan bola bawaan (m/s^2) bila fit belum layak
  double gk_decel_min_, gk_decel_max_;
  bool   gk_save_dry_run_;          // true = pemicu hanya dicatat, tanpa page 14 (mode tera)
  double gk_save_distance_m_;       // bola pelan mendekat di dalam jarak ini -> page 14
  double gk_slow_mps_;              // laju mendekat minimum untuk DEKAT (m/s)
  double gk_approach_mps_;          // laju mendekat minimum untuk CEPAT (m/s)
  double gk_ttc_sec_;               // waktu tiba maksimum untuk CEPAT (dtk)
  double gk_prealert_ttc_sec_;      // waktu tiba <= ini -> berhenti melangkah (bukan jongkok)
  double gk_min_drop_m_;            // penurunan jarak bersih minimum dalam jendela riwayat
  // Sama, tapi untuk bola di zona DEKAT (<= save_distance): derau jarak di sana jauh
  // lebih kecil, jadi bola pelan (~0,25 m/s) sudah boleh dianggap bergerak (2026-11-11).
  double gk_near_min_drop_m_;
  double gk_track_max_distance_m_;  // sampel jarak di luar ini (luar kalibrasi) dibuang
  int    gk_max_saves_;             // jatah saving (aturan KSTI: 6, sisanya foul)
  int    gk_saves_used_;            // reset saat D58 dinyalakan / dimatikan
  bool   gk_threat_;                // ada bola mendekat -> berhenti melangkah
  std::chrono::steady_clock::time_point gk_threat_time_;
  bool   gkSavesLeft();             // false = jatah habis (log WARN, pemicu ditelan)
  double gk_feet_distance_m_;       // bola tertahan di kaki
  double gk_feet_min_w_px_;         // lebar minimum bola terpotong tepi bawah = di kaki
  bool   gk_close_trigger_;         // pemicu "bola dekat" menunggu dipakai process()
  std::string gk_close_reason_;
  double gk_nearest_m_;             // jarak bola terdekat frame terakhir (-1 = tak ada)
  double gk_nearest_speed_;         // laju mendekat bola terdekat (m/s, + = mendekat)
  bool   gk_ball_at_feet_;          // frame terakhir: ada bola di kaki
  std::chrono::steady_clock::time_point gk_ball_at_feet_time_;

  //== FILTER ROBOT (operator 2026-11-12) ===================================
  // Model kadang menandai benda di LUAR lapangan sebagai "robot". Striker hanya
  // dari kotak yang lolos geometri tanah (kaki = tepi bawah kotak, kalibrasi
  // servo 20), keyakinan berhisteresis, jejak yang sudah PASTI (>= 3 frame), dan
  // dekat bola. Kasus 2 (robot tanpa bola) hanya untuk jejak yang barusan di bola.
  double gk_min_conf_robot_new_;    // jejak striker BARU butuh keyakinan ini
  double gk_robot_max_distance_m_;  // kaki lebih jauh dari ini (maju) = luar lapangan
  double gk_robot_max_lateral_m_;   // kaki lebih ke samping dari ini = luar lapangan
  double gk_robot_min_h_m_, gk_robot_max_h_m_;   // tinggi benda dari kotak
  double gk_robot_min_aspect_;      // tinggi/lebar kotak minimum (humanoid berdiri)
  double gk_striker_ball_max_m_;    // bola terdekat ke kaki striker paling jauh ini
  struct GkRobotGeo
  {
    bool   terukur = false;         // false = kotak terpotong tepi bawah / tanpa kalibrasi
    double maju = -1.0, samping = 0.0, tinggi = -1.0;   // m (maju dari ujung kaki kiper)
  };
  // -1 = lolos; selain itu indeks alasan GkRobotTolak.
  int robotGeometryCheck(const GkBox &b, GkRobotGeo &g) const;
  // Titik tanah bola (m, kerangka kamera mendatar): false bila tak terukur.
  bool ballGroundPoint(const GkBox &b, double &x, double &y) const;
  struct GkStriker
  {
    bool   aktif = false;
    bool   pasti = false;
    GkBox  box;
    GkRobotGeo geo;
    double vx = 0.0, vy = 0.0;      // px/dtk, untuk prediksi pencocokan
    double last_seen = 0.0;         // stempel gambar (dtk)
    double dekat_bola_at = -1e9;    // stempel terakhir bola <= striker_ball_max dari kakinya
    std::vector<double> hits;       // stempel frame yang mengenai jejak ini (<= 0,25 dtk)
  };
  GkStriker gk_striker_;
  enum GkRobotTolak { RTOLAK_JAUH = 0, RTOLAK_SAMPING, RTOLAK_TINGGI, RTOLAK_BENTUK, RTOLAK_CONF,
                      RTOLAK_BELUM, RTOLAK_BOLA, RTOLAK_N };
  int gk_rtolak_[RTOLAK_N];
  // Pilih striker frame ini dari semua kotak "robot" (valid=false = tak ada striker).
  GkBox selectStriker(const std::vector<std::pair<GkBox, double>> &robot,
                      const std::vector<GkBox> &bola, double stamp);
};

}  // namespace robotis_op

#endif  // INTERCEPT_DEMO_H_

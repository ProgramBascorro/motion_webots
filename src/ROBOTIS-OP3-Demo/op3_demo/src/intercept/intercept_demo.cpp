/*
 * DEMO NORMAL + INTERCEPT (tombol D58). Lihat intercept_demo.h untuk ringkasan.
 *
 * Aturan yang dipegang berkas ini:
 *   1. Tidak ada satu pun sudut sendi yang dihitung di sini. Gerakan hanya
 *      dipanggil: preset walking dari walking_versions.json, atau action page.
 *   2. Hanya satu gerakan aktif pada satu waktu. Setiap perpindahan state lewat
 *      enterState(), yang selalu menghentikan gerakan state sebelumnya dulu.
 *   3. Ambang yang berasal dari demo D56 disalin apa adanya dan ditandai.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "op3_demo/intercept_demo.h"

namespace robotis_op
{

namespace
{
// Nama versi di walking_versions.json ditulis manusia lewat BASCORRO Studio,
// jadi pencocokannya tidak boleh peka huruf besar/kecil ("Jalan Ke surga").
std::string toLower(const std::string &in)
{
  std::string out = in;
  for (auto &c : out)
    c = static_cast<char>(::tolower(static_cast<unsigned char>(c)));
  return out;
}

// Setengah FOV horizontal kamera, sama dengan BallTracker::FOV_WIDTH
// (ball_tracker.cpp) -- piksel -> sudut memakai rumus yang sama dengan
// BallTracker::processTracking(): sudut = -atan(norm_x * tan(FOV)).
constexpr double GK_HALF_FOV_WIDTH_RAD = 35.2 * M_PI / 180.0;
}  // namespace

InterceptDemo::InterceptDemo()
  : FALL_FORWARD_LIMIT(60),      // soccer_demo.cpp:28 -- ambang D56, jangan diubah
    FALL_BACK_LIMIT(-60),        // soccer_demo.cpp:29 -- ambang D56, jangan diubah
    IMU_FILTER_ALPHA(0.4),       // soccer_demo.cpp imuDataCallback -- filter D56
    NOT_FOUND_THRESHOLD(50),     // ball_follower.cpp:33
    SPIN_RATE(30),
    yaw_limit_deg_(10.0),
    yaw_release_deg_(6.0),
    overshoot_pending_(false),
    overshoot_was_right_(false),
    overshoot_stop_yaw_(0.0),
    intercept_max_sec_(20.0),
    left_trigger_pct_(46.0),
    left_release_pct_(49.0),
    right_trigger_pct_(54.0),
    right_release_pct_(51.0),
    lateral_(LATERAL_NONE),
    pan_range_deg_(85.0),
    // Taksiran awal, TERA DI LAPANGAN (log "TAHAN BOLA" mencetak servo 20).
    // Kamera ~0,45 m di atas lantai: bola 1 m -> menunduk ~20 deg sendi = servo
    // ~200; titik lepas 195 (~1,2 m). Ambang kaki = ambang tendang lama (233).
    block_servo20_deg_(200.0),
    block_release_servo20_deg_(195.0),
    feet_servo20_deg_(233.0),
    block_hold_sec_(3.0),
    page_block_(14),
    block_armed_(true),
    block_page_sent_(false),
    block_hold_started_(false),
    min_state_ticks_(9),
    action_timeout_sec_(12.0),
    init_settle_ticks_(0),
    init_anchor_page_(2),
    init_arm_swing_gain_(0.4),
    idle_arm_swing_period_(1.0),
    transition_hold_ms_(500),
    intercept_enter_hold_ms_(200),
    // Field non-offset INIT POSE (period, gain, ...) masih dari versi "mutar
    // kanan". Offset Init Pose-nya TIDAK lagi dipakai sejak 2026-10-30: posisi
    // diam = page 2 WALKING_READY, lihat applyPreset().
    preset_init_("mutar kanan"),
    preset_left_("geser kiri"),
    preset_right_("geser kanan"),
    preset_turn_left_("mutar kiri"),
    preset_turn_right_("mutar kanan"),
    // 0 = bangun depan MATI untuk sementara (operator 2026-10-30); page aslinya 10.
    page_getup_front_(0),
    page_getup_back_(12),
    upright_ticks_(0),
    head_sway_on_(false),
    state_(IDLE),
    state_ticks_(0),
    init_stance_sent_(false),
    init_direct_(false),
    walking_bias_reuse_on_(false),
    last_page_played_(0),
    init_stance_tick_(0),
    stand_state_(Stand),
    present_pitch_(0.0),
    present_roll_(0.0),
    present_yaw_(0.0),
    imu_seen_(false),
    head_pan_(0.0),
    head_tilt_(0.0),
    head_joint_seen_(false),
    tracking_status_(BallTracker::Waiting),
    ball_lost_count_(0),
    present_tilt_(0.0),
    present_tilt_seen_(false),
    walking_running_(false),
    action_running_(false),
    pending_page_(0),
    return_state_(DEMO_NORMAL),
    transition_hold_until_(std::chrono::steady_clock::now()),
    walk_settle_until_(std::chrono::steady_clock::now()),
    current_period_time_(0.7),
    applied_soft_start_(-1.0),
    boot_page_done_(false),
    soft_start_retry_at_(std::chrono::steady_clock::now())
{
  block_hold_until_ = std::chrono::steady_clock::now();
  enable_ = false;
  geser_gap_sec_ = 0.6;
  geser_max_sec_ = 15.0;   // operator 2026-09-19: 15 dtk (semula 10)
  lateral_net_sec_ = 0.0;
  lateral_tick_ = std::chrono::steady_clock::now();
  soft_stop_pending_init_ = false;
  geser_soft_start_sec_ = 0.0;
  geser_started_at_ = std::chrono::steady_clock::now();
  geser_next_allowed_ = std::chrono::steady_clock::now();
  // Batas geser terkalibrasi (2026-11-11) -- diisi loadEdgeCalibration().
  gk_edge_ok_ = false;
  gk_edge_left_sec_ = gk_edge_right_sec_ = 0.0;
  gk_edge_tail_left_sec_ = gk_edge_tail_right_sec_ = 0.0;
  gk_edge_half_width_m_ = 0.9;
  gk_edge_dist_left_m_ = gk_edge_dist_right_m_ = 0.9;
  gk_edge_tolerance_m_ = 0.15;
  gk_edge_tol_left_m_ = gk_edge_tol_right_m_ = 0.15;
  lateral_pos_ = 0.0;

  // MODE KIPER -- nilai awal = config/goalkeeper_adaptation_params.yaml panduan
  // KSTI 2026, plus keputusan operator 2026-11-07 (page 14 maks 5 dtk, bola
  // tertahan -> tahan 2 dtk). Semua bisa ditimpa param intercept_gk_*.
  gk_mode_ = true;
  gk_min_conf_ball_ = 0.42;
  gk_min_conf_robot_ = 0.48;
  gk_ema_alpha_ = 0.6;   // 2026-11-09: 0,45 -> 0,6, lebih cepat bereaksi
  gk_ball_x_weight_ = 0.60;
  gk_striker_x_weight_ = 0.40;
  gk_striker_close_ratio_ = 0.18;
  // 2026-11-09: 3,5 -> 6,0 dtk SEMENTARA (operator) -- 3,5 dtk memblokir geser
  // 34-50 % waktu saat target di luar tengah. Ganti dengan hasil ukur: setengah
  // lebar area merah / kecepatan geser.
  gk_max_sidestep_sec_ = 6.0;
  gk_target_dist_m_ = -1.0;
  gk_lateral_trigger_m_ = 0.10;
  gk_lateral_release_m_ = 0.04;
  gk_trigger_deg_ = 3.5;
  gk_release_deg_ = 1.0;
  gk_kick_velocity_px_ = 14.0;
  gk_same_ball_px_ = 90.0;
  gk_kick_ready_window_sec_ = 0.5;
  gk_kick_valid_sec_ = 1.0;
  gk_block_max_sec_ = 5.0;
  gk_block_hold_sec_ = 2.0;
  gk_frame_width_ = 640.0;    // config/camera_param.yaml op3_ball_detector
  gk_frame_height_ = 360.0;
  gk_camera_fps_ = 30.0;
  gk_lost_sec_ = 0.8;         // ~ ballIsFresh() (25 siklus / 30 Hz)
  gk_has_target_ = false;
  gk_target_x_ = gk_frame_width_ / 2.0;
  gk_target_y_ = gk_frame_height_ / 2.0;
  gk_kasus_ = 4;
  gk_target_time_ = std::chrono::steady_clock::now();
  gk_lawan_siap_time_ = std::chrono::steady_clock::now();
  gk_lawan_siap_seen_ = false;
  gk_prev_stamp_valid_ = false;
  gk_kick_ = false;
  gk_kick_time_ = std::chrono::steady_clock::now();
  gk_lock_ = false;
  block_started_at_ = std::chrono::steady_clock::now();

  // Jarak bola (kalibrasi servo 20). Dipakai hanya bila berkas kalibrasi termuat.
  gk_calib_ok_ = false;
  gk_calib_fy_ = 490.0;
  gk_calib_cy0_ = 180.0;
  gk_calib_H_ = 0.48;
  gk_calib_theta0_deg_ = 12.5;
  gk_calib_c_ = 0.07;
  // 2026-11-10: 1,0 -> 0,6 -> 0,4 (saveball palsu dari bola diam / jauh)
  gk_save_distance_m_ = 0.4;
  gk_head_still_dps_ = 10.0;
  gk_calib_pitch_deg_ = 11.1;   // median pitch IMU 350 sampel kalibrasi 2026-11-08
  gk_pitch_tol_deg_ = 8.0;
  gk_max_ball_mps_ = 3.5;
  gk_predict_max_m_ = 1.0;
  present_pan_ = 0.0;
  head_ref_valid_ = false;
  head_ref_tilt_ = 0.0;
  head_ref_pan_ = 0.0;
  head_ref_time_ = std::chrono::steady_clock::now();
  head_moving_until_ = std::chrono::steady_clock::now();
  for (int &n : gk_tolak_) n = 0;
  // 2026-11-12: waktu tangkap gambar, pemicu CEPAT, filter robot (lihat header).
  gk_cam_latency_sec_ = 0.0;   // 0 = sudut kepala sekarang, sampai latensi terukur
  gk_cap_tilt_ = 0.0;
  gk_cap_pan_ = 0.0;
  gk_react_sec_ = 0.65;        // page 14 0,56 dtk (time 70 x 8 ms) + ganti modul ~0,07 dtk
  gk_min_conf_robot_new_ = 0.60;
  gk_robot_max_distance_m_ = 3.0;
  gk_robot_max_lateral_m_ = 2.0;
  gk_robot_min_h_m_ = 0.25;
  gk_robot_max_h_m_ = 1.2;
  gk_robot_min_aspect_ = 0.8;
  gk_striker_ball_max_m_ = 1.2;
  for (int &n : gk_rtolak_) n = 0;
  gk_reach_m_ = 0.25;
  gk_decel_prior_ = 1.5;       // DITERA dengan dry_run di lapangan
  gk_decel_min_ = 0.3;
  gk_decel_max_ = 4.0;
  gk_save_dry_run_ = false;
  gk_slow_mps_ = 0.25;
  gk_approach_mps_ = 0.5;
  gk_ttc_sec_ = 1.0;
  gk_prealert_ttc_sec_ = 2.0;
  gk_min_drop_m_ = 0.15;
  gk_near_min_drop_m_ = 0.10;   // = slow_mps 0,25 x jendela 0,4 dtk
  gk_track_max_distance_m_ = 2.5;
  gk_max_saves_ = 6;
  gk_saves_used_ = 0;   // juga di-reset saat D58 dinyalakan / dimatikan (operator 2026-11-09)
  gk_threat_ = false;
  gk_threat_time_ = std::chrono::steady_clock::now();
  gk_feet_distance_m_ = 0.25;
  gk_feet_min_w_px_ = 120.0;
  gk_close_trigger_ = false;
  gk_nearest_m_ = -1.0;
  gk_nearest_speed_ = 0.0;
  gk_ball_at_feet_ = false;
  gk_ball_at_feet_time_ = std::chrono::steady_clock::now();

  // Tabel id->nama sendi dibaca dari berkas yang sama dengan SoccerDemo.
  std::string path =
      ament_index_cpp::get_package_share_directory("op3_gui_demo") + "/config/gui_config.yaml";
  parseJointNameFromYaml(path);
}

InterceptDemo::~InterceptDemo()
{
}

void InterceptDemo::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "Node is not set");
    return;
  }

  declareParameters();

  // BallTracker dipakai sebagai SUMBER POSISI BOLA saja. Head-nya dikuasai
  // head_tracking_node (demo.launch.xml menyetel use_head_control=false), jadi
  // tracker ini tidak menggerakkan kepala -- sama seperti pada demo D56.
  ball_tracker_.setNode(node_);
  ball_tracker_.setUsingHeadControl(false);

  imu_data_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/robotis/open_cr/imu", 10,
      std::bind(&InterceptDemo::imuDataCallback, this, std::placeholders::_1));
  present_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/present_joint_states", 10,
      std::bind(&InterceptDemo::presentJointCallback, this, std::placeholders::_1));
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/goal_joint_states", 10,
      std::bind(&InterceptDemo::jointStatesCallback, this, std::placeholders::_1));
  movement_done_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/robotis/movement_done", 10,
      std::bind(&InterceptDemo::movementDoneCallback, this, std::placeholders::_1));

  walking_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
  walking_param_pub_ =
      node_->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  action_page_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  head_tracker_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/ball_tracker/command", 10);
  reset_yaw_pub_ = node_->create_publisher<std_msgs::msg::String>("/robotis/open_cr/reset_yaw", 10);
  rgb_led_pub_ =
      node_->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);

  // MODE KIPER: semua kelas YOLO (ball/gawang/robot, px) dari ball_detector.py,
  // dan titik halang untuk head_tracking_node (format sama dengan ball_center:
  // x,y ternormalisasi 0..1). Dipakai hanya bila intercept_gk_mode=true.
  detections_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/ball_detector_node/detections", 10,
      std::bind(&InterceptDemo::detectionsCallback, this, std::placeholders::_1));
  gk_target_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/vision/gk/target_center", 10);

  loadWalkingVersions();
}

void InterceptDemo::setUsingHeadControl(bool use_head_control)
{
  ball_tracker_.setUsingHeadControl(use_head_control);
}

void InterceptDemo::declareParameters()
{
  auto get_double = [&](const std::string &name, double def) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<double>(name, def);
    return node_->get_parameter(name).as_double();
  };
  auto get_int = [&](const std::string &name, int def) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<int>(name, def);
    return static_cast<int>(node_->get_parameter(name).as_int());
  };
  auto get_bool = [&](const std::string &name, bool def) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<bool>(name, def);
    return node_->get_parameter(name).as_bool();
  };
  auto get_str = [&](const std::string &name, const std::string &def) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<std::string>(name, def);
    return node_->get_parameter(name).as_string();
  };

  yaw_limit_deg_   = get_double("intercept_yaw_limit_deg", yaw_limit_deg_);
  yaw_release_deg_ = get_double("intercept_yaw_release_deg", yaw_release_deg_);
  // INTERCEPT YAW, pemetaan FINAL dari operator (2026-09-10), ditulis langsung
  // tanpa bendera pembalik supaya tidak bisa tertukar lagi:
  //
  //   yaw > +10  -> INTERCEPT_RIGHT -> save "mutar kanan" -> berhenti saat yaw <= +6
  //   yaw < -10  -> INTERCEPT_LEFT  -> save "mutar kiri"  -> berhenti saat yaw >= -6
  //
  // Histeresis 10 -> 6: jarak 4 derajat antara pemicu dan titik lepas mencegah
  // mutar berhidup-mati akibat noise di sekitar batas. yaw yang dipakai = yaw
  // terpublish /robotis/open_cr/imu apa adanya (kompas terkalibrasi lima titik).
  // Nol-nya adalah arah robot saat demo DINYALAKAN: setDemoEnable() mengirim
  // /robotis/open_cr/reset_yaw (permintaan operator 2026-09-18; sebelumnya,
  // sejak 2026-09-10, tidak ada reset). Dimatikan = titik nol dibiarkan.
  //
  // Catatan riwayat: pemetaan ini sama dengan yang dulu dicapai lewat bendera
  // intercept_yaw_invert=true (hasil uji di robot 2026-09-09). Bendera itu dibuang
  // karena pembalikan ganda membuat arahnya sulit dibaca dan dua kali tertukar.
  // Pagar pengaman: kalau yaw tidak pernah menembus batas berhenti (mis.
  // mag_status bukan OK, yang membuat open_cr_module mengirim yaw = 0 terus),
  // rotasi akan berlangsung selamanya. Ini memotongnya dan kembali ke DEMO.
  intercept_max_sec_ = get_double("intercept_max_sec", intercept_max_sec_);
  // Histeresis geser (FINAL dari operator 2026-09-10). Titik lepas SENGAJA di
  // antara pemicu dan 50%: geser kiri 46 -> 49, geser kanan 54 -> 51. Jangan
  // ditukar (kiri 51 / kanan 49 salah) -- itu membuat robot berhenti SEBELUM
  // bola melewati tengah.
  left_trigger_pct_   = get_double("intercept_left_trigger_pct",   left_trigger_pct_);
  left_release_pct_   = get_double("intercept_left_release_pct",   left_release_pct_);
  right_trigger_pct_  = get_double("intercept_right_trigger_pct",  right_trigger_pct_);
  right_release_pct_  = get_double("intercept_right_release_pct",  right_release_pct_);
  // Jeda antar-geser (detik), dihitung sesudah geser sebelumnya berhenti dan
  // gaitnya tuntas. Permintaan operator 2026-09-18: 0,6 dtk (semula 0,3), berlaku
  // untuk geser kiri <-> geser kanan maupun arah yang sama diulang.
  geser_gap_sec_      = get_double("intercept_geser_gap_sec",      geser_gap_sec_);
  // Batas waktu geser bersama kiri/kanan (lihat lateral_net_sec_ di header).
  geser_max_sec_      = get_double("intercept_geser_max_sec",      geser_max_sec_);
  // Batas lunak head_pan di head_control_module (+-85 derajat) dipakai sebagai
  // ujung skala 0%..100%. head_pan positif = menoleh KIRI (lihat BallTracker).
  pan_range_deg_  = get_double("intercept_pan_range_deg", pan_range_deg_);

  // TAHAN BOLA (operator 2026-10-30) -- menggantikan TENDANG. Syarat bola sama
  // dengan tendang dulu: bola terdeteksi (ballIsFresh()) + posisi servo ID 20.
  block_servo20_deg_         = get_double("intercept_block_servo20_deg", block_servo20_deg_);
  block_release_servo20_deg_ = get_double("intercept_block_release_servo20_deg", block_release_servo20_deg_);
  feet_servo20_deg_          = get_double("intercept_feet_servo20_deg", feet_servo20_deg_);
  block_hold_sec_            = get_double("intercept_block_hold_sec", block_hold_sec_);
  page_block_                = get_int("intercept_page_block", page_block_);

  min_state_ticks_    = get_int("intercept_min_state_ticks", min_state_ticks_);
  action_timeout_sec_ = get_double("intercept_action_timeout_sec", action_timeout_sec_);
  // 0 = tanpa jeda: begitu D58 ditekan, stance init dikirim dan DEMO_NORMAL
  // langsung jalan di siklus berikutnya (diminta operator 2026-09-11). Dulu 45
  // siklus (1,5 dtk) untuk menunggu ramp stance walking selesai lebih dulu.
  init_settle_ticks_  = get_int("intercept_init_settle_ticks", init_settle_ticks_);
  // Page acuan FIXED INIT POSE. 2 = WALKING_READY, page yang sama yang dipakai
  // op3_manager saat boot dan demo_node saat kembali ke Ready.
  init_anchor_page_   = get_int("intercept_init_anchor_page", init_anchor_page_);
  init_arm_swing_gain_ = get_double("intercept_init_arm_swing_gain", init_arm_swing_gain_);
  idle_arm_swing_period_ = get_double("intercept_idle_arm_swing_period", idle_arm_swing_period_);
  // Sesudah page ini INIT langsung ke init pose "mutar kanan", tanpa page acuan
  // (permintaan operator 2026-09-13). 12 BANGBEL sengaja tidak masuk: pose akhirnya
  // jauh dari page 2 (siku ~150 derajat) dan siku tidak dipegang walking module.
  if (!node_->has_parameter("intercept_direct_init_pages"))
    node_->declare_parameter<std::vector<int64_t>>("intercept_direct_init_pages", std::vector<int64_t>{7, 8, 10});
  direct_init_pages_ = node_->get_parameter("intercept_direct_init_pages").as_integer_array();
  // Jeda perpindahan DEMO_NORMAL <-> INTERCEPT_*, diminta operator 500 ms
  // (semula 200 ms).
  transition_hold_ms_ = get_int("intercept_transition_hold_ms", transition_hold_ms_);
  // Jeda MASUK ke INTERCEPT, terpisah dari jeda keluarnya (operator 2026-09-11:
  // "action sebelumnya selesai -> 200 ms -> intercept"). Dihitung setelah gait
  // sebelumnya tuntas, dan hanya sekali per perpindahan -- bukan per langkah.
  intercept_enter_hold_ms_ = get_int("intercept_enter_hold_ms", intercept_enter_hold_ms_);

  preset_init_       = get_str("intercept_preset_init", preset_init_);
  preset_left_       = get_str("intercept_preset_left", preset_left_);
  preset_right_      = get_str("intercept_preset_right", preset_right_);
  preset_turn_left_  = get_str("intercept_preset_turn_left", preset_turn_left_);
  preset_turn_right_ = get_str("intercept_preset_turn_right", preset_turn_right_);

  page_getup_front_ = get_int("intercept_page_getup_front", page_getup_front_);
  page_getup_back_  = get_int("intercept_page_getup_back", page_getup_back_);

  // MODE KIPER (lihat header). false = D58 persis seperti sebelum 2026-11-07.
  gk_mode_                  = get_bool("intercept_gk_mode", gk_mode_);
  gk_min_conf_ball_         = get_double("intercept_gk_min_conf_ball", gk_min_conf_ball_);
  gk_min_conf_robot_        = get_double("intercept_gk_min_conf_robot", gk_min_conf_robot_);
  gk_ema_alpha_             = get_double("intercept_gk_ema_alpha", gk_ema_alpha_);
  // Toleransi geser mode kiper (operator 2026-11-09): +-10 cm mulai, +-4 cm berhenti.
  gk_lateral_trigger_m_     = get_double("intercept_gk_lateral_trigger_m", gk_lateral_trigger_m_);
  gk_lateral_release_m_     = get_double("intercept_gk_lateral_release_m", gk_lateral_release_m_);
  gk_trigger_deg_           = get_double("intercept_gk_trigger_deg", gk_trigger_deg_);
  gk_release_deg_           = get_double("intercept_gk_release_deg", gk_release_deg_);
  gk_ball_x_weight_         = get_double("intercept_gk_ball_x_weight", gk_ball_x_weight_);
  gk_striker_x_weight_      = get_double("intercept_gk_striker_x_weight", gk_striker_x_weight_);
  gk_striker_close_ratio_   = get_double("intercept_gk_striker_close_ratio", gk_striker_close_ratio_);
  gk_max_sidestep_sec_      = get_double("intercept_gk_max_sidestep_sec", gk_max_sidestep_sec_);
  gk_kick_velocity_px_      = get_double("intercept_gk_kick_velocity_px", gk_kick_velocity_px_);
  gk_same_ball_px_          = get_double("intercept_gk_same_ball_px", gk_same_ball_px_);
  gk_kick_ready_window_sec_ = get_double("intercept_gk_kick_ready_window_sec", gk_kick_ready_window_sec_);
  gk_kick_valid_sec_        = get_double("intercept_gk_kick_valid_sec", gk_kick_valid_sec_);
  gk_block_max_sec_         = get_double("intercept_gk_block_max_sec", gk_block_max_sec_);
  gk_block_hold_sec_        = get_double("intercept_gk_block_hold_sec", gk_block_hold_sec_);
  gk_frame_width_           = get_double("intercept_gk_frame_width", gk_frame_width_);
  gk_frame_height_          = get_double("intercept_gk_frame_height", gk_frame_height_);
  gk_camera_fps_            = get_double("intercept_gk_camera_fps", gk_camera_fps_);
  gk_lost_sec_              = get_double("intercept_gk_lost_sec", gk_lost_sec_);
  // Bola dekat dari kalibrasi jarak servo 20 (operator 2026-11-08).
  // Pemicu saving + jatah (operator 2026-11-09).
  gk_save_distance_m_       = get_double("intercept_gk_save_distance_m", gk_save_distance_m_);
  gk_slow_mps_              = get_double("intercept_gk_slow_mps", gk_slow_mps_);
  gk_approach_mps_          = get_double("intercept_gk_approach_mps", gk_approach_mps_);
  gk_ttc_sec_               = get_double("intercept_gk_ttc_sec", gk_ttc_sec_);
  // Prediksi titik berhenti bola (operator 2026-11-10).
  gk_reach_m_               = get_double("intercept_gk_reach_m", gk_reach_m_);
  gk_decel_prior_           = get_double("intercept_gk_decel_prior", gk_decel_prior_);
  gk_decel_min_             = get_double("intercept_gk_decel_min", gk_decel_min_);
  gk_decel_max_             = get_double("intercept_gk_decel_max", gk_decel_max_);
  gk_save_dry_run_          = get_bool("intercept_gk_save_dry_run", gk_save_dry_run_);
  // Pengaman bias jarak (operator 2026-11-10).
  gk_head_still_dps_        = get_double("intercept_gk_head_still_dps", gk_head_still_dps_);
  gk_calib_pitch_deg_       = get_double("intercept_gk_calib_pitch_deg", gk_calib_pitch_deg_);
  gk_pitch_tol_deg_         = get_double("intercept_gk_pitch_tol_deg", gk_pitch_tol_deg_);
  gk_max_ball_mps_          = get_double("intercept_gk_max_ball_mps", gk_max_ball_mps_);
  gk_predict_max_m_         = get_double("intercept_gk_predict_max_m", gk_predict_max_m_);
  gk_prealert_ttc_sec_      = get_double("intercept_gk_prealert_ttc_sec", gk_prealert_ttc_sec_);
  gk_min_drop_m_            = get_double("intercept_gk_min_drop_m", gk_min_drop_m_);
  gk_near_min_drop_m_       = get_double("intercept_gk_near_min_drop_m", gk_near_min_drop_m_);
  gk_track_max_distance_m_  = get_double("intercept_gk_track_max_distance_m", gk_track_max_distance_m_);
  gk_max_saves_             = get_int("intercept_gk_max_saves", gk_max_saves_);
  // 2026-11-12: waktu tangkap gambar, pemicu CEPAT, filter robot luar lapangan.
  gk_cam_latency_sec_       = get_double("intercept_gk_cam_latency_sec", gk_cam_latency_sec_);
  gk_react_sec_             = get_double("intercept_gk_react_sec", gk_react_sec_);
  gk_min_conf_robot_new_    = get_double("intercept_gk_min_conf_robot_new", gk_min_conf_robot_new_);
  gk_robot_max_distance_m_  = get_double("intercept_gk_robot_max_distance_m", gk_robot_max_distance_m_);
  gk_robot_max_lateral_m_   = get_double("intercept_gk_robot_max_lateral_m", gk_robot_max_lateral_m_);
  gk_robot_min_h_m_         = get_double("intercept_gk_robot_min_h_m", gk_robot_min_h_m_);
  gk_robot_max_h_m_         = get_double("intercept_gk_robot_max_h_m", gk_robot_max_h_m_);
  gk_robot_min_aspect_      = get_double("intercept_gk_robot_min_aspect", gk_robot_min_aspect_);
  gk_striker_ball_max_m_    = get_double("intercept_gk_striker_ball_max_m", gk_striker_ball_max_m_);
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
              "KIPER: CEPAT bila tiba di zona kaki <= %.2f dtk (>= %.2f m/s); latensi kamera %.3f dtk%s; "
              "striker: kaki <= %.1f m maju / %.1f m samping, tinggi %.2f-%.2f m, tinggi/lebar >= %.2f, "
              "keyakinan baru >= %.2f, bola <= %.1f m dari kaki",
              gk_react_sec_, gk_approach_mps_, gk_cam_latency_sec_,
              gk_cam_latency_sec_ > 0.0 ? "" : " (sudut kepala sekarang)",
              gk_robot_max_distance_m_, gk_robot_max_lateral_m_, gk_robot_min_h_m_, gk_robot_max_h_m_,
              gk_robot_min_aspect_, gk_min_conf_robot_new_, gk_striker_ball_max_m_);
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
              "KIPER: jatah saving %d; PREDIKSI berhenti <= %.2f m (perlambatan bawaan %.2f, %.1f-%.1f m/s^2); "
              "DEKAT <= %.2f m @ >= %.2f m/s; lacak <= %.1f m; berhenti melangkah bila tiba <= %.1f dtk; "
              "turun bersih >= %.2f m (zona DEKAT >= %.2f m)%s",
              gk_max_saves_, gk_reach_m_, gk_decel_prior_, gk_decel_min_, gk_decel_max_,
              gk_save_distance_m_, gk_slow_mps_, gk_track_max_distance_m_, gk_prealert_ttc_sec_,
              gk_min_drop_m_, std::min(gk_min_drop_m_, gk_near_min_drop_m_),
              gk_save_dry_run_ ? " -- MODE TERA (dry run): TANPA page 14" : "");
  gk_feet_distance_m_      = get_double("intercept_gk_feet_distance_m", gk_feet_distance_m_);
  gk_feet_min_w_px_         = get_double("intercept_gk_feet_min_w_px", gk_feet_min_w_px_);
  std::string calib_default;
  try
  {
    calib_default = ament_index_cpp::get_package_share_directory("op3_demo") + "/data/kalibrasi_jarak_kamera.yaml";
  }
  catch (const std::exception &)
  {
  }
  const std::string calib_file = get_str("intercept_gk_calib_file", calib_default);
  gk_calib_ok_ = loadDistanceCalibration(calib_file);

  // Batas geser terkalibrasi (scripts/calib_batas_geser.py, operator 2026-11-11).
  std::string edge_default;
  try
  {
    edge_default = ament_index_cpp::get_package_share_directory("op3_demo") + "/data/kalibrasi_batas_geser.yaml";
  }
  catch (const std::exception &)
  {
  }
  gk_edge_tolerance_m_ = get_double("intercept_gk_edge_tolerance_m", gk_edge_tolerance_m_);
  gk_edge_tol_right_m_ = get_double("intercept_gk_edge_tolerance_kanan_m", gk_edge_tolerance_m_);
  gk_edge_tol_left_m_ = get_double("intercept_gk_edge_tolerance_kiri_m", gk_edge_tolerance_m_);
  gk_edge_ok_ = loadEdgeCalibration(get_str("intercept_gk_edge_calib_file", edge_default));

  // Aturan KSTI 2026 Bab III.H.10: lebih dari 10 dtk dalam posisi save = foul.
  // Terburuk = bola tertahan tepat di ujung batas page 14 lalu ditahan lagi,
  // jadi JUMLAH keduanya yang harus di bawah 10 dtk (sisakan 1 dtk untuk page 2).
  if (gk_block_max_sec_ + gk_block_hold_sec_ > 9.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                "KIPER: block_max %.1f + block_hold %.1f > 9 dtk (batas foul 10 dtk) -- "
                "dipotong ke 5,0 + 2,0", gk_block_max_sec_, gk_block_hold_sec_);
    gk_block_max_sec_ = 5.0;
    gk_block_hold_sec_ = 2.0;
  }
}

//== data gerakan tersimpan ==================================================
//
// walking_versions.json ditulis BASCORRO Studio lewat apply_node dan tinggal di
// direktori data op3_action_module (bind-mount, jadi hidup melewati rebuild).
// JSON adalah bagian dari YAML, jadi yaml-cpp bisa membacanya langsung -- tidak
// perlu pustaka baru.
bool InterceptDemo::loadWalkingVersions()
{
  std::string path;
  try
  {
    path = ament_index_cpp::get_package_share_directory("op3_action_module") + "/data/walking_versions.json";
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "op3_action_module share not found: %s", e.what());
    return false;
  }

  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "Gagal membaca %s : %s", path.c_str(), e.what());
    return false;
  }

  presets_.clear();
  preset_soft_start_.clear();
  // Nama versi boleh kembar di walking_versions.json (BASCORRO Studio memberi id,
  // bukan nama, jadi "Save sebagai baru" dengan nama sama menghasilkan dua entri).
  // Tanpa penjagaan ini yang menang adalah entri TERAKHIR di berkas, yang bisa
  // jauh lebih tua -- gejalanya "parameter sudah saya perbarui tapi gerakannya
  // masih yang lama". Jadi dibandingkan updatedAt: yang paling baru yang dipakai.
  std::map<std::string, double> preset_updated;
  for (std::size_t i = 0; i < doc.size(); i++)
  {
    const YAML::Node &entry = doc[i];
    if (!entry["name"] || !entry["params"])
      continue;

    const YAML::Node &p = entry["params"];
    op3_walking_module_msgs::msg::WalkingParam wp;

    auto num = [&](const char *key, double def) {
      return p[key] ? p[key].as<double>() : def;
    };
    auto flag = [&](const char *key, bool def) {
      return p[key] ? p[key].as<bool>() : def;
    };

    wp.init_x_offset     = num("init_x_offset", 0.0);
    wp.init_y_offset     = num("init_y_offset", 0.0);
    wp.init_z_offset     = num("init_z_offset", 0.0);
    wp.init_roll_offset  = num("init_roll_offset", 0.0);
    wp.init_pitch_offset = num("init_pitch_offset", 0.0);
    wp.init_yaw_offset   = num("init_yaw_offset", 0.0);
    wp.period_time       = num("period_time", 0.6);
    wp.dsp_ratio         = num("dsp_ratio", 0.4);
    wp.step_fb_ratio     = num("step_fb_ratio", 0.28);
    wp.x_move_amplitude  = num("x_move_amplitude", 0.0);
    wp.y_move_amplitude  = num("y_move_amplitude", 0.0);
    wp.z_move_amplitude  = num("z_move_amplitude", 0.0);
    wp.angle_move_amplitude = num("angle_move_amplitude", 0.0);
    wp.move_aim_on       = flag("move_aim_on", false);
    wp.balance_enable    = flag("balance_enable", true);
    wp.balance_hip_roll_gain     = num("balance_hip_roll_gain", 0.0);
    wp.balance_knee_gain         = num("balance_knee_gain", 0.0);
    wp.balance_ankle_roll_gain   = num("balance_ankle_roll_gain", 0.0);
    wp.balance_ankle_pitch_gain  = num("balance_ankle_pitch_gain", 0.0);
    wp.y_swap_amplitude  = num("y_swap_amplitude", 0.0);
    wp.z_swap_amplitude  = num("z_swap_amplitude", 0.0);
    wp.arm_swing_gain    = num("arm_swing_gain", 0.0);
    wp.pelvis_offset     = num("pelvis_offset", 0.0);
    wp.hip_pitch_offset  = num("hip_pitch_offset", 0.0);
    wp.p_gain            = static_cast<int>(num("p_gain", 32));
    wp.i_gain            = static_cast<int>(num("i_gain", 0));
    wp.d_gain            = static_cast<int>(num("d_gain", 0));

    const std::string key = toLower(entry["name"].as<std::string>());
    const double updated = entry["updatedAt"] ? entry["updatedAt"].as<double>() : 0.0;
    auto seen = preset_updated.find(key);
    if (seen != preset_updated.end())
    {
      RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                  "Versi walking \"%s\" ada lebih dari satu; dipakai yang updatedAt terbaru",
                  key.c_str());
      if (updated <= seen->second)
        continue;                      // entri ini lebih tua -- jangan menimpa
    }
    preset_updated[key] = updated;
    presets_[key] = wp;
    // Soft start disimpan Studio di dalam params save, tapi BUKAN field
    // WalkingParam, jadi dulu tidak pernah terbawa ke D58: gait memakai nilai
    // global terakhir (mis. 0 dari Apply "mutar kiri") dan geser jalan tanpa soft
    // start. Diambil dari entri yang sama (updatedAt terbaru) dengan parameternya.
    if (p["soft_start_time"])
      preset_soft_start_[key] = p["soft_start_time"].as<double>();
    else
      preset_soft_start_.erase(key);
  }

  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "Memuat %zu versi walking dari %s",
              presets_.size(), path.c_str());

  // Laporkan yang hilang SEKARANG, bukan saat robot sudah berdiri di lapangan.
  const std::string wanted[] = {preset_init_, preset_left_, preset_right_,
                                preset_turn_left_, preset_turn_right_};
  for (const auto &name : wanted)
  {
    if (!hasPreset(name))
      RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"),
                   "Versi walking \"%s\" TIDAK ADA di walking_versions.json", name.c_str());
  }
  return true;
}

bool InterceptDemo::hasPreset(const std::string &name) const
{
  return presets_.find(toLower(name)) != presets_.end();
}

void InterceptDemo::applyPreset(const std::string &name, bool moving)
{
  auto it = presets_.find(toLower(name));
  if (it == presets_.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "Versi walking \"%s\" tidak ada", name.c_str());
    return;
  }

  op3_walking_module_msgs::msg::WalkingParam wp = it->second;
  if (moving == false)
  {
    // POSISI DIAM = PAGE 2 WALKING_READY (operator 2026-10-30). walking_module
    // menangkap page 2 sebagai dasar stance (walking_bias_) lewat offset rujukan
    // REF_* di op3_walking_module.h; diam dengan offset TEPAT sama dengan REF_*
    // (dan y 0, karena REF tidak punya y) = berdiri persis di page 2. Angka di
    // bawah salinan REF_* -- ubah bersama kalau REF_* ditera ulang.
    wp.init_x_offset     = -0.0427;
    wp.init_y_offset     = 0.0;
    wp.init_z_offset     = 0.002;
    wp.init_roll_offset  = 1.140186 * M_PI / 180.0;
    wp.init_pitch_offset = -0.360963 * M_PI / 180.0;
    wp.init_yaw_offset   = -1.443854 * M_PI / 180.0;
    wp.hip_pitch_offset  = 6.543178 * M_PI / 180.0;
    // Amplitudo dinolkan supaya walking_module berhenti di stance itu, bukan berjalan.
    wp.x_move_amplitude = 0.0;
    wp.y_move_amplitude = 0.0;
    wp.angle_move_amplitude = 0.0;
    // Khusus INIT POSE: arm swing 0.4 (operator 2026-09-19). Gerakan (moving
    // == true) tetap memakai arm_swing_gain save-nya sendiri.
    wp.arm_swing_gain = init_arm_swing_gain_;
  }

  if (walking_param_pub_)
    walking_param_pub_->publish(wp);
}

bool InterceptDemo::setWalkingNodeParam(const rclcpp::Parameter &param)
{
  // Pola yang sama dengan isActionRunning(): node sementara + tunggu jawaban,
  // karena node demo tidak di-spin di thread ini. Hanya dipanggil saat nilainya
  // berubah, bukan tiap siklus.
  auto temp_node = rclcpp::Node::make_shared("intercept_walking_param");
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(temp_node, "/op3_walking_module");
  if (!client->wait_for_service(std::chrono::milliseconds(500)))
    return false;

  auto future = client->set_parameters({param});
  if (rclcpp::spin_until_future_complete(temp_node, future, std::chrono::milliseconds(500)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    return false;

  const auto results = future.get();
  return results.empty() == false && results[0].successful == true;
}

bool InterceptDemo::applySoftStartTime(double seconds)
{
  return setWalkingNodeParam(rclcpp::Parameter("soft_start_time", seconds));
}

void InterceptDemo::setHeadSway(bool on)
{
  if (head_tracker_cmd_pub_ == nullptr)
    return;
  std_msgs::msg::String cmd;
  cmd.data = on ? "sway_on" : "sway_off";
  head_tracker_cmd_pub_->publish(cmd);
  head_sway_on_ = on;
}

void InterceptDemo::setIdleArmSwing(bool on)
{
  const double deg = on ? init_arm_swing_gain_ * IDLE_ARM_SWING_REF_DEG : 0.0;
  const bool ok_period = (on == false) ||
                         setWalkingNodeParam(rclcpp::Parameter("idle_arm_swing_period_sec", idle_arm_swing_period_));
  const bool ok_deg = setWalkingNodeParam(rclcpp::Parameter("idle_arm_swing_deg", deg));
  if (ok_period && ok_deg)
    RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "ayunan lengan saat diam: %.1f deg, %.2f dtk",
                deg, idle_arm_swing_period_);
  else
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "ayunan lengan saat diam GAGAL dipasang (%.1f deg)", deg);
}

bool InterceptDemo::setWalkingBiasReuse(bool on)
{
  if (walking_bias_reuse_on_ == on)
    return true;
  if (setWalkingNodeParam(rclcpp::Parameter("reuse_walking_bias", on)) == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "reuse_walking_bias=%s GAGAL dipasang",
                 on ? "true" : "false");
    return false;
  }
  walking_bias_reuse_on_ = on;
  return true;
}

void InterceptDemo::startPreset(const std::string &name)
{
  if (walking_running_ == true && current_preset_ == name)
    return;  // sudah berjalan dengan preset ini -- jangan kirim ulang tiap tick

  // Gait sebelumnya belum tuntas menutup siklusnya. Menimpanya sekarang membuat
  // langkah terakhir terpotong di tengah ayunan -- persis yang harus dihindari.
  if (std::chrono::steady_clock::now() < walk_settle_until_)
    return;

  // Jeda antar-geser (termasuk geser yang sama diulang) -- lihat geser_* di header.
  if (isGeserPreset(name) == true && std::chrono::steady_clock::now() < geser_next_allowed_)
    return;

  // Batas waktu geser bersama: arah yang sudah habis tidak di-start lagi.
  if (isGeserPreset(name) == true && geserBudgetLeft(name) == false)
  {
    if (gkEdgeActive() == true)
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 2000,
                           "\"%s\" DITAHAN: tepi area kiper (posisi %+.2f m, + = kiri; batas kiri %.2f / "
                           "kanan %.2f m)", name.c_str(), gkEdgePosM(),
                           gkEdgeLimit(true) * gk_edge_dist_left_m_, gkEdgeLimit(false) * gk_edge_dist_right_m_);
    else
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 2000,
                           "\"%s\" DITAHAN: batas waktu geser tercapai (kiri-kanan bersih %+.1f dtk, "
                           "batas +-%.1f)", name.c_str(), lateral_net_sec_, geserMaxSec());
    return;
  }

  auto it = presets_.find(toLower(name));
  if (it != presets_.end() && it->second.period_time > 0.0)
    current_period_time_ = it->second.period_time;

  // Soft start WAJIB dipasang sebelum "start": walking module membaca
  // soft_start_time tepat di siklus pertama sesudah start dari diam.
  //
  // GESER KIRI/KANAN TIDAK BOLEH JALAN TANPA SOFT START (operator 2026-09-13):
  // save tanpa soft start (> 0), atau soft start yang gagal dipasang, menahan
  // geser -- "start" tidak dikirim, dicoba lagi paling cepat 1 dtk kemudian.
  const bool is_geser = (toLower(name) == toLower(preset_left_) || toLower(name) == toLower(preset_right_));
  auto ss = preset_soft_start_.find(toLower(name));
  if (is_geser == true && (ss == preset_soft_start_.end() || ss->second <= 0.0))
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 2000,
                          "\"%s\" DITAHAN: save-nya tidak punya soft start > 0 dtk -- setel Soft start "
                          "di save itu lewat BASCORRO Studio", name.c_str());
    return;
  }
  if (ss != preset_soft_start_.end() && ss->second != applied_soft_start_)
  {
    if (is_geser == true && std::chrono::steady_clock::now() < soft_start_retry_at_)
      return;
    if (applySoftStartTime(ss->second) == true)
    {
      applied_soft_start_ = ss->second;
      RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "soft start %.2f dtk dipasang untuk \"%s\"",
                  ss->second, name.c_str());
    }
    else if (is_geser == true)
    {
      soft_start_retry_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"),
                   "soft start %.2f dtk untuk \"%s\" GAGAL dipasang -- geser DITAHAN, dicoba lagi",
                   ss->second, name.c_str());
      return;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"),
                   "soft start %.2f dtk untuk \"%s\" GAGAL dipasang -- gait memakai nilai sebelumnya",
                   ss->second, name.c_str());
    }
  }

  applyPreset(name, true);

  if (walking_command_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "start";
    walking_command_pub_->publish(cmd);
  }

  current_preset_ = name;
  walking_running_ = true;
  if (is_geser == true)
  {
    // Soft start mulai dihitung walking_module di siklus pertama sesudah "start".
    geser_started_at_ = std::chrono::steady_clock::now();
    geser_soft_start_sec_ = (ss != preset_soft_start_.end()) ? ss->second : 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "Jalankan save walking : %s", name.c_str());
}

bool InterceptDemo::isGeserPreset(const std::string &name) const
{
  return toLower(name) == toLower(preset_left_) || toLower(name) == toLower(preset_right_);
}

// true kalau tidak ada geser yang sedang berjalan, atau soft start geser yang
// berjalan sudah selesai.
bool InterceptDemo::geserSoftStartDone() const
{
  if (walking_running_ == false || isGeserPreset(current_preset_) == false)
    return true;
  return std::chrono::steady_clock::now() >=
         geser_started_at_ + std::chrono::milliseconds(static_cast<int>(geser_soft_start_sec_ * 1000.0));
}

bool InterceptDemo::geserBudgetLeft(const std::string &name) const
{
  // Mode kiper + kalibrasi batas (2026-11-11): posisi terhadap tepi area kiper,
  // termasuk gerak yang masih akan terjadi sesudah dihentikan (ramp + ekor).
  if (gkEdgeActive() == true && isGeserPreset(name) == true)
  {
    const bool left = (toLower(name) == toLower(preset_left_));
    const bool running = (walking_running_ == true && toLower(current_preset_) == toLower(name));
    double soft = geser_soft_start_sec_;
    if (running == false)
    {
      auto ss = preset_soft_start_.find(toLower(name));
      soft = (ss != preset_soft_start_.end()) ? ss->second : 0.0;
    }
    const double reserve = gkEdgeReserve(left, running, soft);
    return left ? (lateral_pos_ + reserve < gkEdgeLimit(true)) : (lateral_pos_ - reserve > -gkEdgeLimit(false));
  }
  if (toLower(name) == toLower(preset_left_))
    return lateral_net_sec_ < geserMaxSec();
  if (toLower(name) == toLower(preset_right_))
    return lateral_net_sec_ > -geserMaxSec();
  return true;
}

// Mode kiper: batas panduan (max_sidestep_accumulated_sec) supaya kaki tidak
// keluar garis merah gawang. Rumus penghitungnya tetap lateral_net_sec_.
double InterceptDemo::geserMaxSec() const
{
  return gk_mode_ ? gk_max_sidestep_sec_ : geser_max_sec_;
}

// SOFT STOP untuk geser yang mencapai batas waktu. BEDA dengan stopWalking():
// amplitudo TIDAK dinolkan dulu -- walking_module sendiri melandaikannya ke nol
// selama soft start save itu ("soft_stop"), baru menutup langkah terakhir.
// Init pose preset_init_ dipasang lagi sesudah gait tuntas (soft_stop_pending_init_).
void InterceptDemo::softStopWalking(const char *why)
{
  gkEdgeAddStopMotion(true);   // ramp turun + ekor -- sebelum current_preset_ dikosongkan
  if (walking_command_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "soft_stop";
    walking_command_pub_->publish(cmd);
  }

  const double ramp = (geser_soft_start_sec_ > 0.0) ? geser_soft_start_sec_ : 0.0;
  walk_settle_until_ = std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(static_cast<int>((ramp + current_period_time_) * 1000.0));
  if (isGeserPreset(current_preset_) == true)
    geser_next_allowed_ = walk_settle_until_ +
                          std::chrono::milliseconds(static_cast<int>(geser_gap_sec_ * 1000.0));
  soft_stop_pending_init_ = true;

  if (gkEdgeActive() == true)
    RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                "%s: SOFT STOP (%.2f dtk) -- tepi area kiper, posisi akhir %+.2f m (+ = kiri; batas kiri "
                "%.2f / kanan %.2f m)", why, ramp, gkEdgePosM(),
                gkEdgeLimit(true) * gk_edge_dist_left_m_, gkEdgeLimit(false) * gk_edge_dist_right_m_);
  else
    RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                "%s: SOFT STOP (%.2f dtk) -- batas waktu geser, kiri-kanan bersih %+.1f dtk (batas +-%.1f)",
                why, ramp, lateral_net_sec_, geserMaxSec());

  current_preset_.clear();
  walking_running_ = false;
}

void InterceptDemo::stopWalking()
{
  soft_stop_pending_init_ = false;   // stop biasa sudah memasang init pose sendiri
  // Urutannya penting dan disalin dari BallFollower::stopFollowing(): amplitudo
  // dinolkan DULU supaya siklus walking_module berikutnya melambat di tempat,
  // baru perintah "stop" dikirim. Kalau dibalik, gaitnya menyelesaikan langkah.
  applyPreset(preset_init_, false);

  if (walking_command_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "stop";
    walking_command_pub_->publish(cmd);
  }

  // walking_module menutup gait di batas siklus berikutnya (processPhase(),
  // cabang time_ == 0), jadi kaki masih menyelesaikan langkahnya sampai satu
  // period_time penuh setelah "stop" dikirim. period_time diambil dari preset
  // yang barusan berjalan -- bukan angka karangan.
  if (walking_running_ == true)
  {
    gkEdgeAddStopMotion(false);   // ekor langkah sesudah stop (mode kiper terkalibrasi)
    walk_settle_until_ = std::chrono::steady_clock::now() +
                         std::chrono::milliseconds(static_cast<int>(current_period_time_ * 1000.0));
    // Geser berikutnya menunggu geser_gap_sec_ SESUDAH gait ini tuntas.
    if (isGeserPreset(current_preset_) == true)
      geser_next_allowed_ = walk_settle_until_ +
                            std::chrono::milliseconds(static_cast<int>(geser_gap_sec_ * 1000.0));
  }

  current_preset_.clear();
  walking_running_ = false;
}

//== action page =============================================================

void InterceptDemo::playPage(int page_num)
{
  if (action_page_pub_ == nullptr)
    return;

  std_msgs::msg::Int32 msg;
  msg.data = page_num;
  action_page_pub_->publish(msg);

  action_running_ = true;
  pending_page_ = page_num;
  last_page_played_ = page_num;
  action_deadline_ = std::chrono::steady_clock::now() +
                     std::chrono::milliseconds(static_cast<int>(action_timeout_sec_ * 1000));
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "Mainkan action page %d", page_num);
}

void InterceptDemo::stopAction()
{
  if (action_running_ == false || action_page_pub_ == nullptr)
    return;

  // -1 = perintah stop resmi action_module (action_module.cpp:183). Ini satu-
  // satunya cara benar memotong action yang sedang berjalan.
  std_msgs::msg::Int32 msg;
  msg.data = -1;
  action_page_pub_->publish(msg);

  action_running_ = false;
  pending_page_ = 0;
}

void InterceptDemo::movementDoneCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "action" || msg->data == "action_failed")
    action_running_ = false;
  // Hanya selesai SUKSES yang dihitung: page yang gagal tidak menaruh robot di
  // pose acuan, jadi pose sekarang tidak boleh ditangkap walking_module.
  if (msg->data == "action")
    boot_page_done_ = true;
}

bool InterceptDemo::isActionFinished()
{
  if (action_running_ == false)
    return true;

  // Jaring pengaman kalau /robotis/movement_done tidak sampai (misal action
  // gagal start): setelah tenggat, tanyakan langsung ke service milik D56.
  if (std::chrono::steady_clock::now() > action_deadline_)
  {
    if (isActionRunning() == false)
    {
      action_running_ = false;
      return true;
    }
    // masih benar-benar jalan: perpanjang tenggat supaya tidak menanya tiap tick
    action_deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  }
  return false;
}

// Disalin dari SoccerDemo::isActionRunning().
bool InterceptDemo::isActionRunning()
{
  auto temp_node = rclcpp::Node::make_shared("intercept_is_running");
  auto client = temp_node->create_client<op3_action_module_msgs::srv::IsRunning>("/robotis/action/is_running");
  auto request = std::make_shared<op3_action_module_msgs::srv::IsRunning::Request>();

  if (!client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "action/is_running tidak tersedia");
    return false;
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
    return future.get()->is_running;

  return false;
}

//== modul ===================================================================
// Dua fungsi di bawah disalin dari soccer_demo.cpp supaya perilaku peralihan
// modulnya identik dengan D56. SoccerDemo sendiri tidak disentuh.

void InterceptDemo::setBodyModuleToDemo(const std::string &body_module)
{
  robotis_controller_msgs::msg::JointCtrlModule control_msg;
  const std::string head_module = "head_control_module";

  for (auto joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->second);
    if (joint_iter->second.find("head") != std::string::npos)
      control_msg.module_name.push_back(head_module);
    else
      control_msg.module_name.push_back(body_module);
  }

  if (control_msg.joint_name.empty())
    return;

  callServiceSettingModule(control_msg);
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "modul badan : %s", body_module.c_str());
}

void InterceptDemo::setModuleToDemo(const std::string &module_name)
{
  robotis_controller_msgs::msg::JointCtrlModule control_msg;

  for (auto joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->second);
    control_msg.module_name.push_back(module_name);
  }

  if (control_msg.joint_name.empty())
    return;

  callServiceSettingModule(control_msg);
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "modul semua sendi : %s", module_name.c_str());
}

void InterceptDemo::callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule &modules)
{
  auto temp_node = rclcpp::Node::make_shared("intercept_call_service");
  auto client = temp_node->create_client<robotis_controller_msgs::srv::SetJointModule>(
      "/robotis/set_present_joint_ctrl_modules");
  auto request = std::make_shared<robotis_controller_msgs::srv::SetJointModule::Request>();
  request->joint_name = modules.joint_name;
  request->module_name = modules.module_name;

  if (!client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "set_present_joint_ctrl_modules tidak tersedia");
    return;
  }

  auto future = client->async_send_request(request);
  rclcpp::spin_until_future_complete(temp_node, future);
}

void InterceptDemo::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"), "Gagal memuat tabel id_joint.");
    return;
  }

  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator it = id_sub_node.begin(); it != id_sub_node.end(); ++it)
    id_joint_table_[it->first.as<int>()] = it->second.as<std::string>();
}

//== sensor ==================================================================

// Pembacaan pitch/roll/yaw dan filternya mengikuti SoccerDemo::imuDataCallback.
// Bedanya hanya: yaw ikut diambil (D56 tidak memakainya) dan roll disimpan untuk
// keperluan diagnosa.
void InterceptDemo::imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (enable_ == false)
    return;

  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy = robotis_framework::convertQuaternionToRPY(orientation);
  rpy *= (180 / M_PI);

  const double roll = rpy.coeff(0, 0);
  const double pitch = rpy.coeff(1, 0);
  // YAW IMU TIDAK DIPAKAI di D58 (operator 2026-09-19): yaw selalu 0, jadi
  // INTERCEPT_LEFT/RIGHT (mutar kiri/kanan karena yaw) tidak pernah terpicu.
  // Roll & pitch TETAP dari IMU -- pitch dibutuhkan deteksi jatuh (bangun
  // depan/belakang). Nilai asli: rpy.coeff(2, 0).
  const double yaw = 0.0;


  if (imu_seen_ == false)
  {
    present_roll_ = roll;
    present_pitch_ = pitch;
    present_yaw_ = yaw;
    imu_seen_ = true;
  }

  else
  {
    const double a = IMU_FILTER_ALPHA;
    present_roll_ = present_roll_ * (1 - a) + roll * a;
    present_pitch_ = present_pitch_ * (1 - a) + pitch * a;
    // Yaw melintasi +-180. Filter selisihnya, bukan angkanya, supaya lompatan
    // 179 -> -179 tidak terbaca sebagai putaran 358 derajat.
    const double diff = std::fmod(yaw - present_yaw_ + 540.0, 360.0) - 180.0;
    present_yaw_ = std::fmod(present_yaw_ + diff * a + 540.0, 360.0) - 180.0;
  }

  if (present_pitch_ > FALL_FORWARD_LIMIT)
    stand_state_ = Fallen_Forward;
  else if (present_pitch_ < FALL_BACK_LIMIT)
    stand_state_ = Fallen_Behind;
  else
    stand_state_ = Stand;
}

// Servo 19 (head_pan) dan 20 (head_tilt) dibaca dari topik yang sama dengan
// BallFollower::currentJointStatesCallback.
void InterceptDemo::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  int got = 0;
  for (std::size_t ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      head_pan_ = msg->position[ix];
      got++;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      head_tilt_ = msg->position[ix];
      got++;
    }
    if (got == 2)
      break;
  }
  if (got == 2)
    head_joint_seen_ = true;
}

//== logika ==================================================================

// 0% = bola di paling kiri, 100% = paling kanan.
//
// Arah bola diambil seperti di BallFollower: sudut servo 19 DITAMBAH sisa error
// pan dari tracker (bola belum tentu tepat di tengah gambar). head_pan positif
// berarti kepala menoleh ke kiri, jadi skalanya dibalik.
double InterceptDemo::ballPercent()
{
  const double range_rad = pan_range_deg_ * M_PI / 180.0;
  double bearing = head_pan_ + ball_tracker_.getPanOfBall();
  if (bearing > range_rad)
    bearing = range_rad;
  if (bearing < -range_rad)
    bearing = -range_rad;

  return (1.0 - (bearing + range_rad) / (2.0 * range_rad)) * 100.0;
}

bool InterceptDemo::ballIsFresh() const
{
  // Ambang "bola hilang" mengikuti BallFollower::waitFollowing(), yaitu setengah
  // NOT_FOUND_THRESHOLD. Selama masih di bawah itu, posisi terakhir dipakai --
  // supaya satu-dua frame YOLO yang meleset tidak menghentikan gerakan.
  return ball_lost_count_ <= (NOT_FOUND_THRESHOLD / 2);
}

//== MODE KIPER ===============================================================

// Port 1:1 cari_target_penghalangan_x() dari panduan KSTI 2026
// (01_ADAPTASI_GESER_KANAN_KIRI_DAN_JONGKOK.md bagian 1). Tambahannya hanya y
// titik halang, dengan bobot yang sama, supaya kepala bisa mengikutinya.
InterceptDemo::GkTarget InterceptDemo::cariTargetPenghalanganX(
    const GkBox &lawan, const std::vector<GkBox> &bola,
    double target_x_sebelumnya, double target_y_sebelumnya) const
{
  GkTarget t;

  // KASUS 1: striker lawan DAN minimal 1 bola terlihat.
  if (lawan.valid == true && bola.empty() == false)
  {
    const double lawan_x = lawan.cx;
    const double lawan_kaki_y = lawan.cy + 0.5 * lawan.h;

    // Bola (dari 4 bola) yang paling dekat dengan kaki striker.
    double jarak_min = 1e18;
    for (const auto &b : bola)
    {
      const double dx = b.cx - lawan_x;
      const double dy = b.cy - lawan_kaki_y;
      const double jarak = dx * dx + 1.5 * dy * dy;
      if (jarak < jarak_min)
      {
        jarak_min = jarak;
        t.ball = b;
      }
    }

    // Striker sudah berdiri tepat di belakang bola.
    const double ambang = gk_striker_close_ratio_ * gk_frame_width_;
    t.lawan_siap = (jarak_min <= ambang * ambang);

    // Titik halang memotong garis tendang antara kaki striker dan bola.
    t.x = gk_ball_x_weight_ * t.ball.cx + gk_striker_x_weight_ * lawan_x;
    t.y = gk_ball_x_weight_ * t.ball.cy + gk_striker_x_weight_ * lawan.cy;
    t.kasus = 1;
    return t;
  }

  // KASUS 2: bola tertutup kaki striker (sering sesaat sebelum ditendang).
  if (lawan.valid == true)
  {
    t.x = lawan.cx;
    t.y = lawan.cy;
    t.lawan_siap = true;
    t.kasus = 2;
    return t;
  }

  // KASUS 3: striker tak terlihat, ada bola -> bola terdekat ke target lama.
  if (bola.empty() == false)
  {
    double best = 1e18;
    for (const auto &b : bola)
    {
      const double d = std::fabs(b.cx - target_x_sebelumnya);
      if (d < best)
      {
        best = d;
        t.ball = b;
      }
    }
    t.x = t.ball.cx;
    t.y = t.ball.cy;
    t.kasus = 3;
    return t;
  }

  // KASUS 4: tidak ada deteksi -> tahan posisi terakhir.
  t.x = target_x_sebelumnya;
  t.y = target_y_sebelumnya;
  t.kasus = 4;
  return t;
}

void InterceptDemo::detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (enable_ == false || gk_mode_ == false)
    return;

  const auto now = std::chrono::steady_clock::now();
  // Sudut kepala saat gambar ini DITANGKAP (waktu terima - latensi kamera terukur).
  setCaptureHeadAngles(now);

  // Kelas dicocokkan lewat NAMA (ball_detector.py mengisi id = nama kelas),
  // bukan id 0/1 seperti di panduan -- model robot_torso: 0 ball, 3 robot.
  std::vector<std::pair<GkBox, double>> robot;
  std::vector<GkBox> bola;
  for (const auto &d : msg->detections)
  {
    const double score = d.results.empty() ? 1.0 : d.results[0].hypothesis.score;
    GkBox b;
    b.valid = true;
    b.cx = d.bbox.center.position.x;
    b.cy = d.bbox.center.position.y;
    b.w = d.bbox.size_x;
    b.h = d.bbox.size_y;
    if (d.id == "ball" && score >= gk_min_conf_ball_)
      bola.push_back(b);
    else if (d.id == "robot" && score >= gk_min_conf_robot_)
      robot.emplace_back(b, score);
  }

  // Jarak + laju mendekat SEMUA bola (kalibrasi servo 20) -- pemicu bola dekat.
  const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
  updateBallTracks(bola, stamp_sec);
  // Striker (2026-11-12): dulu kotak "robot" paling yakin per frame, apa pun
  // letaknya -- benda di luar lapangan ikut terpilih. Kini lewat selectStriker().
  const GkBox lawan = selectStriker(robot, bola, stamp_sec);
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 10000,
                       "KIPER robot ditolak (sejak D58 ON): jauh %d, samping %d, tinggi %d, bentuk %d, "
                       "keyakinan %d, belum pasti %d, jauh dari bola %d",
                       gk_rtolak_[RTOLAK_JAUH], gk_rtolak_[RTOLAK_SAMPING], gk_rtolak_[RTOLAK_TINGGI],
                       gk_rtolak_[RTOLAK_BENTUK], gk_rtolak_[RTOLAK_CONF], gk_rtolak_[RTOLAK_BELUM],
                       gk_rtolak_[RTOLAK_BOLA]);
  // Pengaman bias (2026-11-10): berapa sampel/pemicu yang ditolak sejak D58 ON.
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 10000,
                       "KIPER tolak (sejak D58 ON): kepala bergerak %d, pitch %d, lompatan %d, laju mustahil %d, "
                       "lebar tak tumbuh %d, ambigu %d, kunci piksel %d",
                       gk_tolak_[TOLAK_KEPALA], gk_tolak_[TOLAK_PITCH], gk_tolak_[TOLAK_LOMPAT],
                       gk_tolak_[TOLAK_LAJU], gk_tolak_[TOLAK_LEBAR], gk_tolak_[TOLAK_AMBIGU],
                       gk_tolak_[TOLAK_KUNCI]);

  GkTarget t;
  if (gk_lock_ == true)
  {
    // Bola sudah ditendang: ikuti bola ITU (terdekat ke posisi terakhirnya),
    // striker diabaikan sampai BLOCK selesai.
    t.x = gk_target_x_;
    t.y = gk_target_y_;
    t.kasus = 4;
    double best = 1e18;
    for (const auto &b : bola)
    {
      const double d = std::hypot(b.cx - gk_lock_ball_.cx, b.cy - gk_lock_ball_.cy);
      if (d < best)
      {
        best = d;
        t.ball = b;
      }
    }
    if (t.ball.valid == true)
    {
      gk_lock_ball_ = t.ball;
      t.x = t.ball.cx;
      t.y = t.ball.cy;
      t.kasus = 3;
    }
  }
  else
  {
    t = cariTargetPenghalanganX(lawan, bola, gk_target_x_, gk_target_y_);
  }

  // Deteksi tendangan: bola aktif bergerak turun (Δcy) atau membesar (Δw) lebih
  // dari kick_velocity_px per frame, DAN striker rapat di bola sesaat
  // sebelumnya. Status "siap" frame ini sengaja dibaca SESUDAH pemeriksaan:
  // di frame tendangan bola sudah menjauh dari kaki striker.
  const rclcpp::Time stamp(msg->header.stamp);
  if (gk_lock_ == false && t.ball.valid == true && gk_prev_ball_.valid == true && gk_prev_stamp_valid_ == true)
  {
    const double dt = (stamp - gk_prev_stamp_).seconds();
    if (dt > 0.0 && dt <= 0.2)
    {
      const double frames = std::max(1.0, dt * gk_camera_fps_);
      const double jump = std::hypot(t.ball.cx - gk_prev_ball_.cx, t.ball.cy - gk_prev_ball_.cy);
      const double dcy = (t.ball.cy - gk_prev_ball_.cy) / frames;
      const double dw = (t.ball.w - gk_prev_ball_.w) / frames;
      const bool same_ball = (jump <= gk_same_ball_px_ * frames);
      const bool ready_recent =
          gk_lawan_siap_seen_ == true &&
          std::chrono::duration<double>(now - gk_lawan_siap_time_).count() <= gk_kick_ready_window_sec_;
      const bool gerak_piksel = (same_ball == true && ready_recent == true &&
                                 (dcy > gk_kick_velocity_px_ || dw > gk_kick_velocity_px_));
      // 2026-11-10 (operator): kunci kepala HANYA bila gerak piksel itu juga didukung
      // jarak bola yang benar-benar turun (lolos pengaman bias). Dulu ~tiap 2 dtk
      // terkunci ke bola diam karena gerak kepala sendiri menggeser cy.
      const bool jarak_turun = gkBallHasMotion(t.ball);
      if (gerak_piksel == true && jarak_turun == false)
      {
        gk_tolak_[TOLAK_KUNCI]++;
        RCLCPP_DEBUG(rclcpp::get_logger("InterceptDemo"),
                     "KIPER: gerak piksel (dcy %.1f, dw %.1f) tanpa jarak turun -- tidak dikunci", dcy, dw);
      }
      if (gerak_piksel == true && jarak_turun == true)
      {
        gk_kick_ = true;
        gk_kick_time_ = now;
        gk_lock_ = true;
        gk_lock_ball_ = t.ball;
        RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                    "KIPER: TENDANGAN terdeteksi (dcy %.1f, dw %.1f px/frame > %.1f) -- bola x %.0f px "
                    "dikunci", dcy, dw, gk_kick_velocity_px_, t.ball.cx);
      }
    }
  }
  if (t.lawan_siap == true)
  {
    gk_lawan_siap_seen_ = true;
    gk_lawan_siap_time_ = now;
  }
  gk_prev_ball_ = t.ball;
  gk_prev_stamp_ = stamp;
  gk_prev_stamp_valid_ = true;

  if (t.kasus == 4)
    return;   // tak ada deteksi: target lama dipertahankan, kepala dibiarkan scan

  // EMA panduan (ema_smoothing_alpha). Bola yang terkunci TIDAK dihaluskan --
  // bola yang ditendang bergerak cepat dan kepala harus mengikutinya tanpa lag.
  const double a = (gk_lock_ == true || gk_has_target_ == false) ? 1.0 : gk_ema_alpha_;
  gk_target_x_ = a * t.x + (1.0 - a) * gk_target_x_;
  gk_target_y_ = a * t.y + (1.0 - a) * gk_target_y_;
  gk_has_target_ = true;
  gk_target_time_ = now;
  gk_kasus_ = t.kasus;
  // Jarak bola aktif untuk toleransi geser dalam meter. Kasus 2 (bola tertutup
  // kaki striker) tak punya bola -> -1, gkLateralError() memakai derajat.
  gk_target_dist_m_ = t.ball.valid ? ballDistance(t.ball) : -1.0;

  if (gk_target_pub_)
  {
    geometry_msgs::msg::PointStamped p;
    p.header = msg->header;
    p.point.x = std::min(1.0, std::max(0.0, gk_target_x_ / gk_frame_width_));
    p.point.y = std::min(1.0, std::max(0.0, gk_target_y_ / gk_frame_height_));
    p.point.z = 1.0;
    gk_target_pub_->publish(p);
  }

  char striker_txt[64] = "-";
  if (lawan.valid == true && gk_striker_.geo.terukur == true)
    std::snprintf(striker_txt, sizeof(striker_txt), "terlihat (%.2f m, tinggi %.2f m)",
                  gk_striker_.geo.maju, gk_striker_.geo.tinggi);
  else if (lawan.valid == true)
    std::snprintf(striker_txt, sizeof(striker_txt), "terlihat");
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 1000,
                       "KIPER: kasus %d%s, target x %.0f px (%.1f%%), striker %s, bola %zu, lawan_siap %d, "
                       "bola terdekat %.2f m (-1 = tak terukur), mendekat %+.2f m/s%s, saving %d/%d, "
                       "simpangan %+.2f %s, %s %+.2f/%.2f %s",
                       gk_kasus_, gk_lock_ ? " [TERKUNCI]" : "", gk_target_x_, gkTargetPercent(),
                       striker_txt, bola.size(), t.lawan_siap ? 1 : 0,
                       gk_nearest_m_, gk_nearest_speed_, gk_ball_at_feet_ ? " [DI KAKI]" : "",
                       gk_saves_used_, gk_max_saves_,
                       [this]() { bool m; return gkLateralError(m); }(),
                       [this]() { bool m; gkLateralError(m); return m ? "m" : "deg"; }(),
                       gkEdgeActive() ? "posisi area kiper (+ = kiri)" : "geser bersih",
                       gkEdgeActive() ? gkEdgePosM() : lateral_net_sec_,
                       gkEdgeActive() ? (gkEdgePosM() >= 0.0 ? gkEdgeLimit(true) * gk_edge_dist_left_m_
                                                             : gkEdgeLimit(false) * gk_edge_dist_right_m_)
                                      : geserMaxSec(),
                       gkEdgeActive() ? "m" : "dtk");
}

// Integral skala soft start min(1, u/soft) du dari u0 ke u1 (u = dtk sejak "start").
// Rumus yang SAMA dengan integral_skala() di scripts/calib_batas_geser.py.
static double integralSkalaSoftStart(double u0, double u1, double soft)
{
  if (u1 <= u0)
    return 0.0;
  if (soft <= 0.0)
    return u1 - u0;
  auto F = [soft](double u) {
    u = std::max(0.0, u);
    return (u <= soft) ? u * u / (2.0 * soft) : soft / 2.0 + (u - soft);
  };
  return F(u1) - F(u0);
}

bool InterceptDemo::loadEdgeCalibration(const std::string &path)
{
  try
  {
    const YAML::Node doc = YAML::LoadFile(path);
    if (!doc["valid"] || doc["valid"].as<bool>() == false)
      throw std::runtime_error("valid: false (belum dikalibrasi kanan DAN kiri)");
    const YAML::Node ka = doc["kanan"];
    const YAML::Node ki = doc["kiri"];
    if (!ka || !ki || !ka.IsMap() || !ki.IsMap() || !ka["tepi_efektif_sec"] || !ki["tepi_efektif_sec"])
      throw std::runtime_error("kanan/kiri.tepi_efektif_sec tidak ada");
    gk_edge_right_sec_ = ka["tepi_efektif_sec"].as<double>();
    gk_edge_left_sec_ = ki["tepi_efektif_sec"].as<double>();
    // Ekor boleh sedikit negatif (2026-11-12: kiri -0,038 dtk dari 4 rekaman) -- dipaksa 0
    // membuat posisi hanyut tiap bolak-balik. Dibatasi -0,25 dtk (= setengah soft start).
    gk_edge_tail_right_sec_ = ka["ekor_sec"] ? std::max(-0.25, ka["ekor_sec"].as<double>()) : 0.0;
    gk_edge_tail_left_sec_ = ki["ekor_sec"] ? std::max(-0.25, ki["ekor_sec"].as<double>()) : 0.0;
    gk_edge_half_width_m_ = doc["setengah_lebar_area_m"] ? doc["setengah_lebar_area_m"].as<double>() : 0.9;
    // Jarak terukur (null di YAML = belum diukur -> setengah lebar area).
    auto jarak = [this](const YAML::Node &n) {
      const YAML::Node j = n["jarak_tepi_m"];
      return (j && j.IsScalar() && j.Scalar() != "null" && j.as<double>() > 0.0) ? j.as<double>()
                                                                                 : gk_edge_half_width_m_;
    };
    gk_edge_dist_right_m_ = jarak(ka);
    gk_edge_dist_left_m_ = jarak(ki);
    if (gk_edge_right_sec_ < 1.0 || gk_edge_left_sec_ < 1.0 || gk_edge_half_width_m_ <= 0.0)
      throw std::runtime_error("tepi < 1 dtk efektif / setengah lebar <= 0 -- tidak masuk akal");
    RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                "KIPER: batas geser terkalibrasi (%s): kanan %.2f dtk efektif = %.2f m (ekor %+.3f), kiri "
                "%.2f dtk = %.2f m (ekor %+.3f); toleransi kanan %.2f / kiri %.2f m -> titik tengah berhenti "
                "di kanan %.2f m / kiri %.2f m",
                path.c_str(), gk_edge_right_sec_, gk_edge_dist_right_m_, gk_edge_tail_right_sec_,
                gk_edge_left_sec_, gk_edge_dist_left_m_, gk_edge_tail_left_sec_, gk_edge_tol_right_m_,
                gk_edge_tol_left_m_, gkEdgeLimit(false) * gk_edge_dist_right_m_,
                gkEdgeLimit(true) * gk_edge_dist_left_m_);
    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                "KIPER: batas geser TIDAK terkalibrasi (%s: %s) -- memakai batas waktu +-%.1f dtk",
                path.c_str(), e.what(), gk_max_sidestep_sec_);
    return false;
  }
}

double InterceptDemo::gkEdgeLimit(bool left) const
{
  const double dist = left ? gk_edge_dist_left_m_ : gk_edge_dist_right_m_;
  const double tol = left ? gk_edge_tol_left_m_ : gk_edge_tol_right_m_;
  return std::clamp(1.0 - tol / dist, 0.1, 1.0);
}

double InterceptDemo::gkEdgePosM() const
{
  return lateral_pos_ * (lateral_pos_ >= 0.0 ? gk_edge_dist_left_m_ : gk_edge_dist_right_m_);
}

double InterceptDemo::gkEdgeReserve(bool left, bool running, double soft_sec) const
{
  const double edge = left ? gk_edge_left_sec_ : gk_edge_right_sec_;
  const double tail = left ? gk_edge_tail_left_sec_ : gk_edge_tail_right_sec_;
  const double soft = std::max(0.0, soft_sec);
  double eff = tail;
  if (running == true)
  {
    // Batas -> soft stop: skala turun linear dari nilainya sekarang ke 0 selama soft start.
    const double u = std::chrono::duration<double>(std::chrono::steady_clock::now() - geser_started_at_).count();
    const double scale = (soft <= 0.0) ? 1.0 : std::min(1.0, std::max(0.0, u / soft));
    eff += scale * soft / 2.0;
  }
  else
  {
    // Geser tak dihentikan sebelum soft start-nya selesai: minimal ramp naik penuh
    // (soft/2) + ramp turun soft stop (soft/2).
    eff += soft;
  }
  return eff / edge;
}

void InterceptDemo::gkEdgeAddStopMotion(bool soft_stop)
{
  if (gkEdgeActive() == false || walking_running_ == false || isGeserPreset(current_preset_) == false)
    return;
  const bool left = (toLower(current_preset_) == toLower(preset_left_));
  // stopWalking() menolkan amplitudo dulu -> hanya ekor; soft stop -> + ramp turun.
  const double frac = soft_stop ? gkEdgeReserve(left, true, geser_soft_start_sec_)
                                : (left ? gk_edge_tail_left_sec_ / gk_edge_left_sec_
                                        : gk_edge_tail_right_sec_ / gk_edge_right_sec_);
  lateral_pos_ += left ? frac : -frac;
}

bool InterceptDemo::loadDistanceCalibration(const std::string &path)
{
  try
  {
    const YAML::Node doc = YAML::LoadFile(path);
    const YAML::Node g = doc["geometri"];
    if (!doc["fy_px"] || !g || !g["H_m"] || !g["theta0_deg"] || !g["c_m"])
      throw std::runtime_error("kunci fy_px / geometri.{H_m,theta0_deg,c_m} tidak ada");
    gk_calib_fy_ = doc["fy_px"].as<double>();
    gk_calib_cy0_ = (doc["gambar"] && doc["gambar"]["cy0"]) ? doc["gambar"]["cy0"].as<double>() : 180.0;
    gk_calib_H_ = g["H_m"].as<double>();
    gk_calib_theta0_deg_ = g["theta0_deg"].as<double>();
    gk_calib_c_ = g["c_m"].as<double>();
    if (doc["model_terpilih"] && doc["model_terpilih"].as<std::string>() != "geometri")
      RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                  "KIPER: model_terpilih bukan geometri -- tetap memakai parameter geometri");
    RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                "KIPER: kalibrasi jarak dimuat (%s): fy %.1f, H %.3f m, theta0 %.2f deg, c %.3f m",
                path.c_str(), gk_calib_fy_, gk_calib_H_, gk_calib_theta0_deg_, gk_calib_c_);
    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                "KIPER: kalibrasi jarak TIDAK dimuat (%s: %s) -- pemicu bola dekat memakai servo20 lama",
                path.c_str(), e.what());
    return false;
  }
}

double InterceptDemo::ballDistance(const GkBox &b) const
{
  if (gk_calib_ok_ == false || present_tilt_seen_ == false || b.valid == false)
    return -1.0;
  // Kalibrasi dibuat dengan badan di pitch IMU ~+11,1 deg. Badan miring jauh dari itu
  // (uji 2026-11-10: page 14 palsu di pitch -13 dan -34 deg) -> kamera ikut miring,
  // jarak tak sah.
  if (imu_seen_ == true && std::fabs(present_pitch_ - gk_calib_pitch_deg_) > gk_pitch_tol_deg_)
    return -1.0;
  const double ang = gkRowDepressionDeg(b.cy) * M_PI / 180.0;
  if (ang < 1.0 * M_PI / 180.0)
    return -1.0;   // di atas / di cakrawala: terlalu jauh untuk diukur
  return std::max(0.0, gk_calib_H_ / std::tan(ang) - gk_calib_c_);
}

// s20_setara = servo20 + atan((y - cy0) / fy); sudut depresi = s20_setara - 180 + theta0.
// Servo 20 diambil SAAT GAMBAR DITANGKAP (setCaptureHeadAngles), bukan sekarang:
// gambar tiba belakangan, jadi saat kepala bergerak sudut sekarang memalsukan jarak.
double InterceptDemo::gkRowDepressionDeg(double y) const
{
  const double s20_setara = jointRadToServoDeg(gk_cap_tilt_) +
                            std::atan((y - gk_calib_cy0_) / gk_calib_fy_) * 180.0 / M_PI;
  return s20_setara - 180.0 + gk_calib_theta0_deg_;
}

double InterceptDemo::gkColumnRightRad(double x) const
{
  return std::atan((x - gk_frame_width_ / 2.0) / gk_calib_fy_) - gk_cap_pan_;   // pan + = menoleh KIRI
}

void InterceptDemo::setCaptureHeadAngles(std::chrono::steady_clock::time_point t_recv)
{
  gk_cap_tilt_ = present_tilt_;
  gk_cap_pan_ = present_pan_;
  if (gk_cam_latency_sec_ <= 0.0 || head_hist_.empty())
    return;
  const auto t = t_recv - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(gk_cam_latency_sec_));
  if (t >= head_hist_.back().t)
    return;
  if (t <= head_hist_.front().t)
  {
    gk_cap_tilt_ = head_hist_.front().tilt;
    gk_cap_pan_ = head_hist_.front().pan;
    return;
  }
  for (std::size_t i = head_hist_.size() - 1; i > 0; i--)
  {
    const HeadSample &a = head_hist_[i - 1];
    const HeadSample &b = head_hist_[i];
    if (a.t <= t)
    {
      const double span = std::chrono::duration<double>(b.t - a.t).count();
      const double u = (span > 1e-6) ? std::chrono::duration<double>(t - a.t).count() / span : 1.0;
      gk_cap_tilt_ = a.tilt + u * (b.tilt - a.tilt);
      gk_cap_pan_ = a.pan + u * (b.pan - a.pan);
      return;
    }
  }
}

bool InterceptDemo::ballGroundPoint(const GkBox &b, double &x, double &y) const
{
  const double d = ballDistance(b);
  if (d < 0.0 || d > gk_track_max_distance_m_)
    return false;
  const double r = d + gk_calib_c_;
  const double kanan = gkColumnRightRad(b.cx);
  x = r * std::cos(kanan);
  y = r * std::sin(kanan);
  return true;
}

// Geometri tanah kotak "robot" (label dataset = SELURUH badan: tepi bawah = kaki).
// Benda di luar lapangan: kaki jauh / terlalu ke samping / di atas cakrawala, atau
// tinggi & bentuknya bukan humanoid kid-size. Tinggi dihitung dari baris atas dan
// bawah kotak: h = H (1 - tan a_atas / tan a_bawah).
int InterceptDemo::robotGeometryCheck(const GkBox &b, GkRobotGeo &g) const
{
  g = GkRobotGeo();
  const bool berdiri = (state_ != BLOCK && state_ != FALL_FRONT && state_ != FALL_BACK);
  if (gk_calib_ok_ == false || present_tilt_seen_ == false || berdiri == false)
    return -1;   // tak bisa dinilai: tinggal keyakinan + jejak
  if (imu_seen_ == true && std::fabs(present_pitch_ - gk_calib_pitch_deg_) > gk_pitch_tol_deg_)
    return -1;
  const double y_bawah = b.cy + 0.5 * b.h;
  const double y_atas = b.cy - 0.5 * b.h;
  const bool potong_bawah = (y_bawah >= gk_frame_height_ - 3.0);
  const bool potong_atas = (y_atas <= 3.0);
  if (potong_bawah == false && potong_atas == false && b.w > 1.0 && b.h / b.w < gk_robot_min_aspect_)
    return RTOLAK_BENTUK;
  if (potong_bawah == true)
    return -1;   // kaki di luar gambar (robot sangat dekat): tak terukur
  const double a_bawah = gkRowDepressionDeg(y_bawah) * M_PI / 180.0;
  if (a_bawah < 1.0 * M_PI / 180.0)
    return RTOLAK_JAUH;   // kaki di atas / di cakrawala
  const double r = gk_calib_H_ / std::tan(a_bawah);
  const double kanan = gkColumnRightRad(b.cx);
  const double a_atas = gkRowDepressionDeg(y_atas) * M_PI / 180.0;
  g.terukur = true;
  g.maju = r * std::cos(kanan) - gk_calib_c_;
  g.samping = r * std::sin(kanan);
  g.tinggi = gk_calib_H_ * (1.0 - std::tan(a_atas) / std::tan(a_bawah));
  if (g.maju > gk_robot_max_distance_m_)
    return RTOLAK_JAUH;
  if (std::fabs(g.samping) > gk_robot_max_lateral_m_)
    return RTOLAK_SAMPING;
  // Kotak terpotong tepi atas: tinggi sebenarnya >= hitungan -> hanya batas atas.
  if (g.tinggi > gk_robot_max_h_m_ || (potong_atas == false && g.tinggi < gk_robot_min_h_m_))
    return RTOLAK_TINGGI;
  return -1;
}

// Satu jejak striker. Kotak baru jadi striker sesudah PASTI (>= 3 frame dalam
// 0,25 dtk), jejak baru butuh keyakinan lebih tinggi, dan selama jejak lama masih
// hidup (< 0,5 dtk tak terlihat) kandidat lain diabaikan -- kecuali jejak lama
// tak pernah dekat bola dalam 2 dtk dan kandidat itu dekat bola.
InterceptDemo::GkBox InterceptDemo::selectStriker(const std::vector<std::pair<GkBox, double>> &robot,
                                                  const std::vector<GkBox> &bola, double stamp)
{
  GkStriker &s = gk_striker_;
  if (s.aktif == true && (stamp - s.last_seen > 0.5 || stamp < s.last_seen - 1e-3))
    s = GkStriker();   // hilang 0,5 dtk, atau stempel mundur (usb_cam mulai ulang)

  std::vector<std::pair<double, double>> bola_xy;
  for (const auto &b : bola)
  {
    double x = 0.0, y = 0.0;
    if (ballGroundPoint(b, x, y))
      bola_xy.emplace_back(x, y);
  }

  struct Kand
  {
    GkBox b;
    double score;
    GkRobotGeo g;
    double ke_bola;   // m dari kaki ke bola terdekat, -1 = tak terukur
  };
  std::vector<Kand> lolos;
  for (const auto &r : robot)
  {
    Kand k{r.first, r.second, GkRobotGeo(), -1.0};
    const int alasan = robotGeometryCheck(k.b, k.g);
    if (alasan >= 0)
    {
      gk_rtolak_[alasan]++;
      RCLCPP_DEBUG(rclcpp::get_logger("InterceptDemo"),
                   "KIPER: robot ditolak (%d) x %.0f y %.0f w %.0f h %.0f conf %.2f, maju %.2f samping %.2f tinggi %.2f",
                   alasan, k.b.cx, k.b.cy, k.b.w, k.b.h, k.score, k.g.maju, k.g.samping, k.g.tinggi);
      continue;
    }
    if (k.g.terukur == true)
      for (const auto &p : bola_xy)
      {
        const double dd = std::hypot(k.g.maju + gk_calib_c_ - p.first, k.g.samping - p.second);
        if (k.ke_bola < 0.0 || dd < k.ke_bola)
          k.ke_bola = dd;
      }
    lolos.push_back(k);
  }
  const bool dinilai = (gk_calib_ok_ == true && state_ != BLOCK && state_ != FALL_FRONT && state_ != FALL_BACK);
  auto dekat_bola = [this](const Kand &k) {
    return k.ke_bola >= 0.0 && k.ke_bola <= gk_striker_ball_max_m_;
  };

  // Lanjutan jejak: kandidat terdekat dari posisi prediksi.
  int lanjut = -1;
  if (s.aktif == true)
  {
    const double gap = std::max(0.0, stamp - s.last_seen);
    const double frames = std::max(1.0, gap * gk_camera_fps_);
    const double px = s.box.cx + s.vx * gap;
    const double py = s.box.cy + s.vy * gap;
    double best = 1e18;
    for (std::size_t i = 0; i < lolos.size(); i++)
    {
      const Kand &k = lolos[i];
      if (dinilai == true && k.g.terukur == false && s.pasti == false)
        continue;
      const double dpx = std::hypot(k.b.cx - px, k.b.cy - py);
      const double gate = std::max(60.0, std::max(k.b.w, s.box.w)) * frames;
      const double hr = (s.box.h > 1.0) ? k.b.h / s.box.h : 1.0;
      if (dpx < gate && hr > 0.6 && hr < 1.67 && dpx < best)
      {
        best = dpx;
        lanjut = static_cast<int>(i);
      }
    }
  }

  // Jejak baru: keyakinan >= conf_new, kaki terukur (bila geometri dinilai);
  // utamakan yang dekat bola, lalu yang paling yakin.
  int baru = -1;
  // Jejak lama yang tak dekat bola 2 dtk (dan frame ini pun tidak) boleh diganti.
  const bool lama_jauh_bola = (s.aktif == true && dinilai == true && stamp - s.dekat_bola_at > 2.0 &&
                               (lanjut < 0 || dekat_bola(lolos[lanjut]) == false));
  if (s.aktif == false || (lama_jauh_bola == true))
  {
    for (std::size_t i = 0; i < lolos.size(); i++)
    {
      const Kand &k = lolos[i];
      if (static_cast<int>(i) == lanjut)
        continue;
      if (k.score < gk_min_conf_robot_new_)
      {
        if (s.aktif == false)
          gk_rtolak_[RTOLAK_CONF]++;
        continue;
      }
      if (dinilai == true && k.g.terukur == false)
        continue;
      if (s.aktif == true && dekat_bola(k) == false)
        continue;   // pengganti jejak lama harus dekat bola
      if (baru < 0)
      {
        baru = static_cast<int>(i);
        continue;
      }
      const Kand &kb = lolos[baru];
      const bool lebih_baik = (dekat_bola(k) != dekat_bola(kb))
                                  ? dekat_bola(k)
                                  : ((dekat_bola(k) && k.ke_bola < kb.ke_bola) ||
                                     (dekat_bola(k) == false && k.score > kb.score));
      if (lebih_baik)
        baru = static_cast<int>(i);
    }
  }

  int pakai = lanjut;
  if (baru >= 0 && (s.aktif == false || lama_jauh_bola == true))
  {
    if (s.aktif == true)
      RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                  "KIPER: striker diganti -- jejak lama tak dekat bola 2 dtk, kandidat baru %.2f m dari bola",
                  lolos[baru].ke_bola);
    s = GkStriker();
    s.aktif = true;
    pakai = baru;
  }
  if (pakai < 0)
    return GkBox();   // jejak lama tak terlihat frame ini (hidup s.d. 0,5 dtk) / tak ada kandidat

  const Kand &k = lolos[pakai];
  const double gap = stamp - s.last_seen;
  if (s.box.valid == true && gap > 1e-3 && gap <= 0.3)
  {
    s.vx = 0.5 * s.vx + 0.5 * (k.b.cx - s.box.cx) / gap;
    s.vy = 0.5 * s.vy + 0.5 * (k.b.cy - s.box.cy) / gap;
  }
  else
  {
    s.vx = 0.0;
    s.vy = 0.0;
  }
  s.box = k.b;
  s.geo = k.g;
  s.last_seen = stamp;
  s.hits.push_back(stamp);
  while (s.hits.empty() == false && stamp - s.hits.front() > 0.25)
    s.hits.erase(s.hits.begin());
  if (s.hits.size() >= 3)
    s.pasti = true;
  if (dekat_bola(k))
    s.dekat_bola_at = stamp;

  if (s.pasti == false)
  {
    gk_rtolak_[RTOLAK_BELUM]++;
    return GkBox();
  }
  if (dinilai == true)
  {
    // Kasus 1: bola terdekatnya harus dekat kaki (striker berdiri di bola).
    if (bola.empty() == false && k.ke_bola >= 0.0 && k.ke_bola > gk_striker_ball_max_m_)
    {
      gk_rtolak_[RTOLAK_BOLA]++;
      return GkBox();
    }
    // Kasus 2 (bola tertutup kaki striker): hanya jejak yang barusan di bola.
    if (bola.empty() == true && stamp - s.dekat_bola_at > 2.0)
    {
      gk_rtolak_[RTOLAK_BOLA]++;
      return GkBox();
    }
  }
  return s.box;
}

// Bola yang sungguh mendekat makin besar di gambar, dan lebar bbox (tidak seperti
// jarak dari servo 20) tak terpengaruh sudut kepala. Syarat: lebar tumbuh paling
// sedikit SEPARUH pertumbuhan ideal dari jarak miring kamera saat mulai bergerak ke
// sekarang. Kotak yang terpotong tepi gambar tak dinilai (lebarnya menyusut palsu).
bool InterceptDemo::gkWidthConfirms(const GkTrack &tr, const GkBox &b, double d) const
{
  if (tr.move_start_w <= 1.0 || tr.move_start_d <= 0.0 || d < 0.0)
    return false;
  const bool tepi = (b.cx - 0.5 * b.w <= 3.0 || b.cx + 0.5 * b.w >= gk_frame_width_ - 3.0 ||
                     b.cy - 0.5 * b.h <= 3.0 || b.cy + 0.5 * b.h >= gk_frame_height_ - 3.0);
  if (tepi)
    return true;
  const double miring_awal = std::hypot(tr.move_start_d + gk_calib_c_, gk_calib_H_);
  const double miring_kini = std::hypot(d + gk_calib_c_, gk_calib_H_);
  const double ideal = miring_awal / miring_kini;   // > 1 bila mendekat
  return b.w / tr.move_start_w >= 1.0 + 0.5 * (ideal - 1.0);
}

bool InterceptDemo::gkBallHasMotion(const GkBox &b) const
{
  for (const auto &tr : gk_tracks_)
    if (tr.box.cx == b.cx && tr.box.cy == b.cy && tr.box.w == b.w)
      return tr.move_hist.empty() == false &&
             tr.move_hist.front().second - tr.move_hist.back().second >= gk_min_drop_m_;
  return false;
}

bool InterceptDemo::ballCutAtFeet(const GkBox &b) const
{
  return b.valid == true && b.w >= gk_feet_min_w_px_ && b.cy + 0.5 * b.h >= gk_frame_height_ - 3.0;
}

// Pelacak sederhana: tiap bola frame ini dicocokkan ke jejak terdekat dari posisi
// PREDIKSI-nya (posisi terakhir + kecepatan piksel), dengan gerbang yang melebar
// sesuai ukuran bola dan lama tak terlihat -- bola menggelinding cepat tetap di
// jejak yang sama. Bola yang meloncat jauh (kepala menyapu) jadi jejak BARU tanpa
// riwayat, jadi tak pernah dianggap "mendekat".
//
// Pemicu saving (operator 2026-11-09): bola DIAM tidak pernah memicu. Laju
// regresi saja tidak cukup (derau jarak di 1,5 m ~0,1 m/derajat tilt) -- jarak
// juga harus TURUN BERSIH >= min_drop dalam jendela riwayat.
void InterceptDemo::updateBallTracks(const std::vector<GkBox> &bola, double stamp)
{
  std::vector<bool> dipakai(gk_tracks_.size(), false);
  gk_nearest_m_ = -1.0;
  gk_nearest_speed_ = 0.0;
  bool at_feet = false;

  for (const auto &b : bola)
  {
    const double d = ballDistance(b);
    if (ballCutAtFeet(b) || (d >= 0.0 && d < gk_feet_distance_m_))
      at_feet = true;

    int best = -1;
    double best_px = 1e9;
    for (std::size_t i = 0; i < gk_tracks_.size(); i++)
    {
      if (dipakai[i])
        continue;
      const auto &t = gk_tracks_[i];
      const double gap = std::max(0.0, stamp - t.last_seen);
      const double frames = std::max(1.0, gap * gk_camera_fps_);
      const double px_pred = t.box.cx + t.vx * gap;
      const double py_pred = t.box.cy + t.vy * gap;
      const double px = std::hypot(b.cx - px_pred, b.cy - py_pred);
      const double gate = std::max(80.0, 2.5 * std::max(b.w, t.box.w)) * frames;
      const double wr = (t.box.w > 1.0) ? b.w / t.box.w : 1.0;
      if (px < gate && px < best_px && wr > 0.6 && wr < 1.67)
      {
        best_px = px;
        best = static_cast<int>(i);
      }
    }
    // Tiga bola di depan kiper: bila bola LAIN juga masuk gerbang jejak yang sama,
    // pencocokan frame ini ambigu (bisa tertukar) -> jangan jadikan bukti gerak.
    // 2026-11-11: hanya bila bola lain itu SAINGAN NYATA -- sama dekatnya ke posisi
    // prediksi (< 1,5x jarak bola ini, min 20 px). Dulu cukup masuk gerbang (>= 80 px):
    // 263 sampel bola terpilih terbuang per ~6 mnt saat beberapa bola di depan kiper.
    bool ambigu = false;
    if (best >= 0)
    {
      const auto &t = gk_tracks_[best];
      const double gap = std::max(0.0, stamp - t.last_seen);
      const double frames = std::max(1.0, gap * gk_camera_fps_);
      const double px_pred = t.box.cx + t.vx * gap;
      const double py_pred = t.box.cy + t.vy * gap;
      const double gate = std::max(80.0, 2.5 * std::max(b.w, t.box.w)) * frames;
      const double saingan = std::min(gate, std::max(1.5 * best_px, 20.0));
      for (const auto &ob : bola)
        if (&ob != &b && std::hypot(ob.cx - px_pred, ob.cy - py_pred) < saingan)
          ambigu = true;
    }
    if (best < 0)
    {
      gk_tracks_.push_back(GkTrack());
      dipakai.push_back(true);
      best = static_cast<int>(gk_tracks_.size()) - 1;
    }
    else
      dipakai[best] = true;

    GkTrack &tr = gk_tracks_[best];
    {
      const double gap = stamp - tr.last_seen;
      if (tr.box.valid == true && gap > 1e-3 && gap <= 0.2)
      {
        // Kecepatan piksel diperhalus, hanya untuk prediksi pencocokan.
        tr.vx = 0.5 * tr.vx + 0.5 * (b.cx - tr.box.cx) / gap;
        tr.vy = 0.5 * tr.vy + 0.5 * (b.cy - tr.box.cy) / gap;
      }
      else
      {
        tr.vx = 0.0;
        tr.vy = 0.0;
      }
    }
    tr.box = b;
    tr.last_seen = stamp;
    // Di luar rentang kalibrasi (cakrawala / lonjakan 5-26 m di log) = tak terukur.
    if (d < 0.0 || d > gk_track_max_distance_m_)
    {
      if (d < 0.0 && imu_seen_ == true &&
          std::fabs(present_pitch_ - gk_calib_pitch_deg_) > gk_pitch_tol_deg_)
        gk_tolak_[TOLAK_PITCH]++;
      continue;
    }
    // PENGAMAN BIAS (2026-11-10): sampel dari frame ambigu atau saat kepala bergerak
    // tidak menambah riwayat -- jejak tetap dicocokkan, hanya tak jadi bukti gerak.
    if (ambigu == true)
    {
      gk_tolak_[TOLAK_AMBIGU]++;
      continue;
    }
    if (headStill() == false)
    {
      gk_tolak_[TOLAK_KEPALA]++;
      continue;
    }
    // Lonjakan jarak yang mustahil secara fisik (tukar bola) -> riwayat tak dipercaya.
    if (tr.hist.empty() == false)
    {
      const double dt = stamp - tr.hist.back().first;
      if (std::fabs(d - tr.hist.back().second) > gk_max_ball_mps_ * std::max(0.0, dt) + 0.08)
      {
        gk_tolak_[TOLAK_LOMPAT]++;
        tr.hist.clear();
        tr.hist_w.clear();
        tr.move_hist.clear();
        tr.pred_ok_count = 0;
      }
    }
    tr.hist.emplace_back(stamp, d);
    tr.hist_w.push_back(b.w);
    while (tr.hist.size() > 1 && stamp - tr.hist.front().first > 0.4)
    {
      tr.hist.erase(tr.hist.begin());
      tr.hist_w.erase(tr.hist_w.begin());
    }

    // Laju mendekat = kemiringan regresi jarak terhadap waktu (>= 3 sampel, >= 0,08 dtk).
    double speed = 0.0;
    double drop = 0.0;
    if (tr.hist.size() >= 3 && tr.hist.back().first - tr.hist.front().first >= 0.08)
    {
      double mt = 0.0, md = 0.0;
      for (const auto &h : tr.hist) { mt += h.first; md += h.second; }
      mt /= tr.hist.size();
      md /= tr.hist.size();
      double num = 0.0, den = 0.0;
      for (const auto &h : tr.hist)
      {
        num += (h.first - mt) * (h.second - md);
        den += (h.first - mt) * (h.first - mt);
      }
      if (den > 1e-9)
        speed = -num / den;
      drop = tr.hist.front().second - tr.hist.back().second;
    }
    // Laju yang mustahil (uji 2026-11-10: "1,89 m, 5,06 m/s") = jarak melompat karena
    // tukar bola / sudut kepala -> riwayat dibuang, tidak memicu apa pun.
    if (speed > gk_max_ball_mps_)
    {
      gk_tolak_[TOLAK_LAJU]++;
      tr.hist.clear();
      tr.hist_w.clear();
      tr.move_hist.clear();
      tr.pred_ok_count = 0;
      tr.cepat_count = 0;
      continue;
    }

    if (gk_nearest_m_ < 0.0 || d < gk_nearest_m_)
    {
      gk_nearest_m_ = d;
      gk_nearest_speed_ = speed;
    }

    // Zona DEKAT (2026-11-11): derau jarak di <= save_distance kecil, jadi ambang
    // turun bersih diperkecil -- bola pelan ~0,25 m/s tetap terbaca bergerak. Di luar
    // zona itu tetap min_drop (derau ~0,1 m/derajat tilt di ~1,5 m).
    const double min_drop = (d <= gk_save_distance_m_) ? std::min(gk_min_drop_m_, gk_near_min_drop_m_)
                                                       : gk_min_drop_m_;
    const bool bergerak = (drop >= min_drop && speed > 0.0);
    // Kiper berdiri (page 2 / stance walking): hanya di sini jarak sah, karena
    // kalibrasi servo 20 dibuat berdiri; saat jongkok page 14 kamera lebih rendah.
    const bool berdiri = (state_ != BLOCK && state_ != FALL_FRONT && state_ != FALL_BACK);
    if (bergerak == false)
    {
      tr.cepat_count = 0;
      // PERSENJATAI ULANG (2026-11-11): jejak yang sudah memicu saving dulu tak pernah
      // di-reset -- bola yang sama (tak pernah hilang dari kamera) digelindingkan lagi
      // sampai ke kaki tanpa saving (log t~523). Begitu kiper berdiri lagi dan bola ini
      // diam atau menjauh, ia boleh memicu lagi. Bola diam tetap tak memicu (continue).
      // hist >= 3: laju 0 bawaan (riwayat baru dibuang) bukan bukti bola diam.
      if (tr.fired == true && berdiri == true && tr.hist.size() >= 3 &&
          (std::fabs(speed) < 0.1 || speed <= -gk_slow_mps_))
      {
        tr.fired = false;
        RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                    "KIPER: bola siap disaving lagi (%.2f m, %+.2f m/s)", d, speed);
      }
      // Bola yang tadi bergerak kini berhenti: catat titik berhentinya vs prediksi,
      // bahan tera gk_decel_prior / gk_reach_m (lihat intercept_gk_save_dry_run).
      if (tr.move_hist.empty() == false && std::fabs(speed) < 0.1 &&
          stamp - tr.move_hist.back().first >= 0.3)
      {
        if (berdiri == true)
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "KIPER: BOLA BERHENTI di %.2f m (prediksi terakhir %.2f m, perlambatan %.2f m/s^2 %s, "
                      "mulai bergerak di %.2f m)",
                      d, tr.pred_stop_m, tr.pred_decel, tr.pred_fitted ? "terukur" : "bawaan",
                      tr.move_hist.front().second);
        tr.move_hist.clear();
        tr.pred_stop_m = -1.0;
        tr.pred_ok_count = 0;
      }
      continue;   // bola diam / menjauh: tidak pernah memicu apa pun
    }
    const double ttc = d / speed;

    // Riwayat sejak bola mulai bergerak (<= 1 dtk), diawali riwayat 0,4 dtk yang ada.
    if (tr.move_hist.empty())
    {
      tr.move_hist = tr.hist;
      // Titik awal pembanding lebar: sampel tertua di riwayat (sebelum bola dinilai bergerak).
      tr.move_start_d = tr.hist.front().second;
      tr.move_start_w = tr.hist_w.empty() ? b.w : tr.hist_w.front();
    }
    else if (tr.move_hist.back().first < stamp)
      tr.move_hist.emplace_back(stamp, d);
    while (tr.move_hist.size() > 1 && stamp - tr.move_hist.front().first > 1.0)
      tr.move_hist.erase(tr.move_hist.begin());

    // Fit d(tau) = c0 + c1 tau + c2 tau^2, tau = t - sekarang -> laju sekarang v = -c1,
    // perlambatan a = 2 c2 (> 0 bila bola makin pelan). Titik berhenti = d - v^2 / 2a.
    double v_now = speed;
    double decel = gk_decel_prior_;
    bool fitted = false;
    // >= 6 sampel dan >= 0,25 dtk data: uji sintetis 200 gulir (derau jarak +-3 cm)
    // -- syarat 5 sampel / 0,15 dtk + 1 frame memicu PALSU 35 %, ini + 3 frame
    // berturut-turut 7 %, page 14 tetap lebih dini dari DEKAT (median 0,75 m).
    const std::size_t n = tr.move_hist.size();
    if (n >= 6 && tr.move_hist.back().first - tr.move_hist.front().first >= 0.25)
    {
      double S[5] = {0, 0, 0, 0, 0}, T[3] = {0, 0, 0};   // sum tau^k, sum d tau^k
      for (const auto &h : tr.move_hist)
      {
        const double tau = h.first - stamp;
        double p = 1.0;
        for (int k = 0; k < 5; k++) { S[k] += p; if (k < 3) T[k] += h.second * p; p *= tau; }
      }
      // Normal equations 3x3 (Cramer).
      const double A[3][3] = {{S[0], S[1], S[2]}, {S[1], S[2], S[3]}, {S[2], S[3], S[4]}};
      auto det3 = [](const double M[3][3]) {
        return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
               M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
      };
      const double D = det3(A);
      if (std::fabs(D) > 1e-15)
      {
        double A1[3][3], A2[3][3];
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 3; c++)
          {
            A1[r][c] = (c == 1) ? T[r] : A[r][c];
            A2[r][c] = (c == 2) ? T[r] : A[r][c];
          }
        const double c1 = det3(A1) / D, c2 = det3(A2) / D;
        if (-c1 > 0.0)
          v_now = -c1;
        const double a_fit = 2.0 * c2;
        if (a_fit >= gk_decel_min_)
        {
          decel = std::min(a_fit, gk_decel_max_);
          fitted = true;
        }
      }
    }
    decel = std::max(decel, gk_decel_min_);
    const double d_stop = d - v_now * v_now / (2.0 * decel);
    tr.pred_stop_m = d_stop;
    tr.pred_decel = decel;
    tr.pred_fitted = fitted;
    // Prediksi "sampai kaki" hanya dari fit yang layak, dan harus bertahan 3 frame.
    tr.pred_ok_count = (fitted && v_now >= gk_slow_mps_ && d_stop <= gk_reach_m_) ? tr.pred_ok_count + 1 : 0;

    // CEPAT (operator 2026-11-12): page 14 butuh 0,56 dtk sampai jongkok (time 70 x 8 ms),
    // jadi bola yang TIBA di zona kaki dalam <= react_sec dipicu sekarang, tidak menunggu
    // 0,6 m (bola 2,6 m/s di DEKAT 0,56 m tiba 0,2 dtk kemudian = lolos, log 11-11).
    // Bola yang akan berhenti sendiri sebelum zona kaki (titik berhenti > reach) tidak.
    // Laju SESAAT = regresi 0,15 dtk terakhir: riwayat 0,4 dtk masih berisi sampel saat bola
    // diam, jadi laju regresinya tertinggal tepat sesudah tendangan (uji sintetis: tendangan
    // 1,5 m @ 2 m/s dipicu di 0,90 m -> 1,16 m dengan laju sesaat, bola diam tetap 0 palsu).
    double v_cepat = v_now;
    {
      double mt = 0.0, md = 0.0;
      std::size_t n_baru = 0;
      for (const auto &h : tr.hist)
        if (stamp - h.first <= 0.15 + 1e-9) { mt += h.first; md += h.second; n_baru++; }
      if (n_baru >= 3)
      {
        mt /= n_baru;
        md /= n_baru;
        double num = 0.0, den = 0.0, t_awal = stamp;
        for (const auto &h : tr.hist)
          if (stamp - h.first <= 0.15 + 1e-9)
          {
            num += (h.first - mt) * (h.second - md);
            den += (h.first - mt) * (h.first - mt);
            t_awal = std::min(t_awal, h.first);
          }
        if (den > 1e-9 && stamp - t_awal >= 0.06)
          v_cepat = std::max(v_cepat, -num / den);
      }
    }
    v_cepat = std::min(v_cepat, gk_max_ball_mps_);
    const double tiba = (v_cepat > 1e-3) ? std::max(0.0, d - gk_reach_m_) / v_cepat : 1e9;
    const double d_stop_cepat = d - v_cepat * v_cepat / (2.0 * decel);
    tr.cepat_count = (v_cepat >= gk_approach_mps_ && tiba <= gk_react_sec_ && d_stop_cepat <= gk_reach_m_ &&
                      d <= gk_predict_max_m_) ? tr.cepat_count + 1 : 0;
    // Bola yang terkunci TENDANGAN piksel (gerak px/frame = bukti independen) cukup 1 frame.
    const bool terkunci = (gk_lock_ == true && gk_lock_ball_.valid == true &&
                           std::hypot(b.cx - gk_lock_ball_.cx, b.cy - gk_lock_ball_.cy) <= std::max(40.0, b.w));

    // Pra-siaga: berhenti melangkah supaya kaki sudah menapak saat page 14 datang.
    if ((ttc <= gk_prealert_ttc_sec_ && speed >= gk_approach_mps_) ||
        (d <= 1.5 && speed >= gk_slow_mps_))
    {
      gk_threat_ = true;
      gk_threat_time_ = std::chrono::steady_clock::now();
    }

    // Hanya saat kiper berdiri: saat jongkok jarak tak sah, dan page 14 sudah jalan.
    if (tr.fired == false && berdiri == true)
    {
      // PREDIKSI (pengganti CEPAT, operator 2026-11-10): page 14 begitu bola yang
      // bergerak diperkirakan berhenti di zona kaki -- sedini mungkin, tapi bola
      // yang akan berhenti sendiri di depan kiper tidak menghabiskan jatah saving.
      // 2026-11-10: PREDIKSI hanya bila bola sudah <= predict_max (1,0 m) -- di jarak
      // jauh salah 1 deg sudut kepala = puluhan cm, sumber pemicu palsu terbesar.
      const bool prediksi = (tr.pred_ok_count >= 3 && d <= gk_predict_max_m_);
      const bool cepat = (tr.cepat_count >= (terkunci ? 1 : 2));
      // DEKAT: cadangan bila prediksi meleset -- bola sudah <= 0,4 m dan mendekat.
      const bool dekat = (d <= gk_save_distance_m_ && speed >= gk_slow_mps_);
      // Keduanya harus didukung pertumbuhan lebar bbox (tak terpengaruh sudut kepala).
      const bool lebar_ok = gkWidthConfirms(tr, b, d);
      if ((cepat || prediksi || dekat) && lebar_ok == false)
        gk_tolak_[TOLAK_LEBAR]++;
      if ((cepat || prediksi || dekat) && lebar_ok == true)
      {
        tr.fired = true;
        gk_close_trigger_ = true;
        char buf[260];
        std::snprintf(buf, sizeof(buf),
                      "%s%s: bola %.2f m, %.2f m/s, tiba %.2f dtk, perlambatan %.2f m/s^2 (%s), berhenti ~%.2f m, "
                      "turun %.2f m",
                      cepat ? "CEPAT" : (prediksi ? "PREDIKSI" : "DEKAT"), (cepat && terkunci) ? " [TENDANGAN]" : "",
                      d, cepat ? v_cepat : v_now, tiba, decel, fitted ? "terukur" : "bawaan",
                      cepat ? d_stop_cepat : d_stop, drop);
        gk_close_reason_ = buf;
      }
    }
  }

  // Jejak yang tak terlihat 0,4 dtk dibuang.
  gk_tracks_.erase(std::remove_if(gk_tracks_.begin(), gk_tracks_.end(),
                                  [stamp](const GkTrack &t) { return stamp - t.last_seen > 0.4; }),
                   gk_tracks_.end());

  gk_ball_at_feet_ = at_feet;
  if (at_feet)
    gk_ball_at_feet_time_ = std::chrono::steady_clock::now();
}

bool InterceptDemo::gkTargetFresh() const
{
  return gk_has_target_ == true &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() - gk_target_time_).count() <= gk_lost_sec_;
}

// Sama dengan ballPercent(), tapi arah diambil dari titik halang: sudut servo 19
// ditambah sudut titik itu di gambar (rumus BallTracker).
// Simpangan samping titik halang dari arah depan kiper, + = KANAN.
// Arah = sudut servo 19 + sudut titik di gambar (fy hasil kalibrasi bila ada,
// kalau tidak FOV BallTracker). Dalam METER bila jarak bola aktif terukur
// (kalibrasi servo 20): sudut tetap berarti simpangan makin besar makin jauh,
// jadi toleransi sudut lama (+-6,8 deg = +-30 cm di 2,5 m) terlalu longgar.
double InterceptDemo::gkLateralError(bool &in_meters) const
{
  const double dx = gk_target_x_ - gk_frame_width_ / 2.0;
  const double img = gk_calib_ok_ ? std::atan(dx / gk_calib_fy_)
                                  : std::atan((dx / (gk_frame_width_ / 2.0)) * std::tan(GK_HALF_FOV_WIDTH_RAD));
  const double kanan_rad = img - head_pan_;   // head_pan + = menoleh KIRI
  in_meters = (gk_target_dist_m_ > 0.0);
  if (in_meters)
    return gk_target_dist_m_ * std::tan(kanan_rad);
  return kanan_rad * 180.0 / M_PI;
}

double InterceptDemo::gkTargetPercent() const
{
  const double range_rad = pan_range_deg_ * M_PI / 180.0;
  const double norm_x = (gk_target_x_ / gk_frame_width_) * 2.0 - 1.0;
  double bearing = head_pan_ - std::atan(norm_x * std::tan(GK_HALF_FOV_WIDTH_RAD));
  if (bearing > range_rad)
    bearing = range_rad;
  if (bearing < -range_rad)
    bearing = -range_rad;

  return (1.0 - (bearing + range_rad) / (2.0 * range_rad)) * 100.0;
}

bool InterceptDemo::gkKickPending() const
{
  return gk_kick_ == true &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() - gk_kick_time_).count() <= gk_kick_valid_sec_;
}

// Jatah saving KSTI (maks gk_max_saves_, sisanya foul). Hanya untuk mode kiper.
bool InterceptDemo::gkSavesLeft()
{
  if (gk_mode_ == false || gk_saves_used_ < gk_max_saves_)
    return true;
  RCLCPP_WARN_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 2000,
                       "KIPER: jatah saving HABIS (%d/%d) -- page %d tidak dimainkan, tetap geser "
                       "(reset = matikan/nyalakan D58)", gk_saves_used_, gk_max_saves_, page_block_);
  return false;
}

void InterceptDemo::sendHeadTrackerCmd(const std::string &cmd)
{
  if (head_tracker_cmd_pub_ == nullptr)
    return;
  std_msgs::msg::String msg;
  msg.data = cmd;
  head_tracker_cmd_pub_->publish(msg);
}

// Sudut sendi (rad) -> derajat servo. Diturunkan dari konstanta device
// XM430-W350 di robotis_device (value_of_0_radian_position 2048,
// value_of_max_radian_position 4095 = +pi), jadi bukan pemetaan karangan:
//   raw       = 2048 + rad * 2047/pi
//   servo_deg = raw * 360/4096
// Uji cepat: rad 30 deg -> 210,0 ; rad 50 deg -> 230,0 ; 240 deg -> rad 60,03.
double InterceptDemo::jointRadToServoDeg(double rad)
{
  return 180.0 + rad * (2047.0 * 360.0) / (4096.0 * M_PI);
}

void InterceptDemo::presentJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  bool got_tilt = false;
  for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++)
  {
    if (msg->name[i] == "head_tilt")   // servo ID 20
    {
      present_tilt_ = msg->position[i];
      present_tilt_seen_ = true;
      got_tilt = true;
    }
    else if (msg->name[i] == "head_pan")   // servo ID 19 (posisi AKTUAL)
      present_pan_ = msg->position[i];
  }
  if (got_tilt == false)
    return;

  // Riwayat 1 dtk sudut kepala aktual: jarak dihitung dari sudut SAAT gambar
  // ditangkap (setCaptureHeadAngles), 2026-11-12.
  const auto t_now = std::chrono::steady_clock::now();
  head_hist_.push_back(HeadSample{t_now, present_tilt_, present_pan_});
  while (head_hist_.size() > 1 && t_now - head_hist_.front().t > std::chrono::seconds(1))
    head_hist_.pop_front();

  // Laju kepala aktual, dinilai per >= 50 ms (125 Hz terlalu berderau per sampel).
  // Kepala bergerak -> gambar (yang tiba belakangan) tak cocok dengan sudut servo 20
  // sekarang, jadi jarak bola diam ikut "berubah": sampel jarak ditolak sampai
  // kepala diam 0,25 dtk (operator 2026-11-10, 68 gerak semu dalam ~5 mnt).
  const auto now = std::chrono::steady_clock::now();
  if (head_ref_valid_ == false)
  {
    head_ref_valid_ = true;
    head_ref_time_ = now;
    head_ref_tilt_ = present_tilt_;
    head_ref_pan_ = present_pan_;
    return;
  }
  const double dt = std::chrono::duration<double>(now - head_ref_time_).count();
  if (dt < 0.05)
    return;
  const double dps = std::max(std::fabs(present_tilt_ - head_ref_tilt_),
                              std::fabs(present_pan_ - head_ref_pan_)) * 180.0 / M_PI / dt;
  if (dps > gk_head_still_dps_)
    head_moving_until_ = now + std::chrono::milliseconds(250);
  head_ref_time_ = now;
  head_ref_tilt_ = present_tilt_;
  head_ref_pan_ = present_pan_;
}

bool InterceptDemo::headStill() const
{
  return std::chrono::steady_clock::now() >= head_moving_until_;
}

// present_tilt_seen_: tanpa itu nilainya masih 0 rad (= 180 derajat servo),
// angka yang tidak pernah benar-benar dibaca dari robot.
double InterceptDemo::servo20Deg() const
{
  if (present_tilt_seen_ == false)
    return -1.0;
  return jointRadToServoDeg(present_tilt_);
}

const char *InterceptDemo::stateName(State s) const
{
  switch (s)
  {
    case IDLE: return "IDLE";
    case INIT: return "INIT";
    case DEMO_NORMAL: return "DEMO_NORMAL";
    case INTERCEPT_LEFT: return "INTERCEPT_LEFT";
    case INTERCEPT_RIGHT: return "INTERCEPT_RIGHT";
    case BLOCK: return "BLOCK";
    case FALL_FRONT: return "FALL_FRONT";
    case FALL_BACK: return "FALL_BACK";
    case RECOVERY_INIT: return "RECOVERY_INIT";
  }
  return "?";
}

// SATU-SATUNYA pintu perpindahan state. Gerakan state lama selalu dihentikan di
// sini, jadi tidak mungkin ada dua gerakan (mis. "geser kanan" + "mutar kiri")
// aktif bersamaan.
void InterceptDemo::enterState(State next)
{
  if (state_ == next)
    return;

  const bool leaving_walk = (state_ == DEMO_NORMAL || state_ == INTERCEPT_LEFT ||
                             state_ == INTERCEPT_RIGHT || state_ == INIT);
  const bool entering_action = (next == BLOCK || next == FALL_FRONT || next == FALL_BACK);
  const bool leaving_action = (state_ == BLOCK || state_ == FALL_FRONT || state_ == FALL_BACK);

  if (leaving_action && action_running_ == true)
    stopAction();          // hanya terjadi saat jatuh memotong page tahan bola
  if (leaving_walk && entering_action)
    stopWalking();

  // Perpindahan DEMO_NORMAL <-> INTERCEPT_* diberi jeda sekali-pakai: gait lama
  // dihentikan, lalu state baru menahan diri selama transition_hold_ms_ sebelum
  // mengirim preset barunya. Jedanya TIDAK memblokir loop -- deteksi jatuh dan
  // pembacaan bola tetap jalan, jadi FALL_* tetap bisa memotong kapan saja.
  const bool to_intercept   = (next == INTERCEPT_LEFT || next == INTERCEPT_RIGHT);
  const bool from_intercept = (state_ == INTERCEPT_LEFT || state_ == INTERCEPT_RIGHT);
  if ((state_ == DEMO_NORMAL && to_intercept) || (from_intercept && next == DEMO_NORMAL))
  {
    if (walking_running_ == true)
      stopWalking();
    // Jeda transisi dihitung SETELAH gait benar-benar tuntas, bukan sejak
    // perintah stop dikirim. Jadi urutannya: langkah terakhir selesai ->
    // diam transition_hold_ms_ -> gerakan berikutnya.
    const auto base = std::max(std::chrono::steady_clock::now(), walk_settle_until_);
    const int hold_ms = to_intercept ? intercept_enter_hold_ms_ : transition_hold_ms_;
    transition_hold_until_ = base + std::chrono::milliseconds(hold_ms);
  }

  // Keluar dari DEMO_NORMAL selalu menghentikan gait, jadi geser yang tadi aktif
  // tidak boleh "dilanjutkan" diam-diam saat kembali.
  if (state_ == DEMO_NORMAL)
    lateral_ = LATERAL_NONE;
  if (next == BLOCK)
  {
    block_page_sent_ = false;
    block_hold_started_ = false;
  }
  // Mode kiper: kunci ke bola yang ditendang hanya berlaku selama BLOCK. Keluar
  // lewat jalur apa pun (selesai, jatuh) -> kembali memilih striker + bola.
  if (state_ == BLOCK)
  {
    gk_lock_ = false;
    gk_kick_ = false;
  }

  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "state %s -> %s (yaw %.1f, pitch %.1f)",
              stateName(state_), stateName(next), present_yaw_, present_pitch_);

  state_ = next;
  state_ticks_ = 0;
}

//== siklus ==================================================================

// Titik nol yaw IMU = arah robot saat demo DINYALAKAN (tombol D56/D58 atau
// perintah). Saat dimatikan titik nol dibiarkan; dinyalakan lagi = diambil ulang
// (permintaan operator 2026-09-18). open_cr_module mengambil offset pada sampel
// kompas sah berikutnya.
void InterceptDemo::resetYaw(const std::string &reason)
{
  if (reset_yaw_pub_ == nullptr)
    return;
  std_msgs::msg::String msg;
  msg.data = reason;
  reset_yaw_pub_->publish(msg);
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "Yaw IMU direset ke 0 (%s)", reason.c_str());
}

void InterceptDemo::setDemoEnable()
{
  enable_ = true;
  // Mode kiper: TANPA magnetometer (KSTI 2026). reset_yaw meminta open_cr_module
  // mengambil titik nol dari kompas, jadi tidak dikirim sama sekali. Yaw memang
  // tidak dipakai D58 (imuDataCallback memaksanya 0).
  if (gk_mode_ == false)
    resetYaw("D58 menyalakan demo intercept");
  else
    RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "KIPER: reset yaw dilewati (tanpa magnetometer)");
  gk_has_target_ = false;
  gk_target_x_ = gk_frame_width_ / 2.0;
  gk_target_y_ = gk_frame_height_ / 2.0;
  gk_kasus_ = 4;
  gk_lawan_siap_seen_ = false;
  gk_prev_ball_ = GkBox();
  gk_prev_stamp_valid_ = false;
  gk_kick_ = false;
  gk_lock_ = false;
  gk_tracks_.clear();
  for (int &n : gk_tolak_) n = 0;
  gk_striker_ = GkStriker();
  for (int &n : gk_rtolak_) n = 0;
  gk_close_trigger_ = false;
  gk_threat_ = false;
  gk_ball_at_feet_ = false;
  // Jatah saving kembali penuh setiap D58 dinyalakan (operator 2026-11-09).
  if (gk_mode_ == true && gk_saves_used_ != 0)
    RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "KIPER: jatah saving di-reset (%d -> 0 dari %d)",
                gk_saves_used_, gk_max_saves_);
  gk_saves_used_ = 0;

  state_ = IDLE;
  state_ticks_ = 0;
  stand_state_ = Stand;
  imu_seen_ = false;
  lateral_ = LATERAL_NONE;
  head_joint_seen_ = false;
  ball_lost_count_ = 0;
  action_running_ = false;
  walking_running_ = false;
  current_preset_.clear();
  init_stance_sent_ = false;
  init_stance_tick_ = 0;
  init_direct_ = false;          // D58 ON selalu lewat page acuan
  walking_bias_reuse_on_ = true; // paksa kirim false di INIT (keadaan node tak diketahui)
  applied_soft_start_ = -1.0;   // bisa sudah diubah Studio sejak D58 terakhir
  lateral_net_sec_ = 0.0;        // batas waktu geser dihitung ulang tiap D58 ON
  lateral_pos_ = 0.0;            // posisi tengah area kiper = posisi saat D58 ON
  lateral_tick_ = std::chrono::steady_clock::now();
  soft_stop_pending_init_ = false;
  transition_hold_until_ = std::chrono::steady_clock::now();
  walk_settle_until_ = std::chrono::steady_clock::now();
  geser_next_allowed_ = std::chrono::steady_clock::now();
  geser_soft_start_sec_ = 0.0;
  block_armed_ = true;
  block_page_sent_ = false;
  block_hold_started_ = false;

  // SELALU baca ulang walking_versions.json, bukan hanya saat daftarnya kosong.
  // Operator menyetel ulang save-nya lewat BASCORRO Studio selagi stack jalan,
  // dan tanpa ini demo tetap memakai parameter yang dimuat saat node start --
  // gejalanya "sudah saya update tapi gerakannya tidak berubah".
  loadWalkingVersions();

  // Ayunan lengan saat diam DIHAPUS (operator 2026-10-30): dipasang 0 secara
  // eksplisit, karena nilai node walking module bisa tertinggal dari sebelumnya.
  // Penggantinya goyang pan kepala, dinyalakan process() saat robot diam.
  setIdleArmSwing(false);
  setHeadSway(false);

  // Nyalakan head_tracking_node (scan + PID kepala) dengan pola sapuan 2D
  // khusus D58. D56 tetap mengirim "start" dan memakai pola lamanya.
  if (head_tracker_cmd_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "start_2d";
    head_tracker_cmd_pub_->publish(cmd);
  }
  // Mode kiper: kepala mengikuti titik halang (striker + bolanya) dari
  // /vision/gk/target_center, bukan bola terbesar. Dikirim SESUDAH start_2d.
  sendHeadTrackerCmd(gk_mode_ ? "target_gk" : "target_ball");
  ball_tracker_.startTracking();

  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "Start DEMO NORMAL + INTERCEPT%s",
              gk_mode_ ? " -- MODE KIPER" : "");
  enterState(INIT);
}

void InterceptDemo::setDemoDisable()
{
  if (head_tracker_cmd_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "stop";
    head_tracker_cmd_pub_->publish(cmd);
  }
  // Kembalikan sumber kepala ke bola (D56 dan demo lain).
  sendHeadTrackerCmd("target_ball");
  gk_lock_ = false;
  gk_kick_ = false;
  // Jatah saving juga kembali penuh saat D58 dimatikan (operator 2026-11-09).
  gk_saves_used_ = 0;

  setHeadSway(false);
  stopAction();
  stopWalking();
  ball_tracker_.stopTracking();
  // Di luar D58 (Studio, D56, Init Pose) Walking Enable harus kembali menangkap
  // pose sekarang seperti biasa.
  setWalkingBiasReuse(false);
  setIdleArmSwing(false);

  enable_ = false;
  state_ = IDLE;
  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "Stop DEMO NORMAL + INTERCEPT");
}

// FIXED INIT POSE di luar siklus demo. Dipanggil demo_node sesudah launch, saat
// robot masih diam di page WALKING_READY yang dimainkan op3_manager -- jadi pose
// yang ditangkap walking_module memang page acuan itu, bukan pose gerakan apa pun.
// applyPreset(..., false) memasang offset rujukan page 2 dan menolkan amplitudo,
// dan "stop" menjaga gait tetap mati.
void InterceptDemo::applyFixedInitPose()
{
  if (presets_.empty())
    loadWalkingVersions();

  if (hasPreset(preset_init_) == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger("InterceptDemo"),
                 "FIXED INIT POSE dilewati: versi \"%s\" tidak ada", preset_init_.c_str());
    return;
  }

  setBodyModuleToDemo("walking_module");
  applyPreset(preset_init_, false);

  if (walking_command_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "stop";
    walking_command_pub_->publish(cmd);
  }
  walking_running_ = false;
  current_preset_.clear();

  RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
              "FIXED INIT POSE (launch) dari versi \"%s\"", preset_init_.c_str());
}

void InterceptDemo::process()
{
  if (enable_ == false)
    return;

  state_ticks_++;

  //-- Penghitung & batas waktu geser bersama ---------------------------------
  {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - lateral_tick_).count();
    lateral_tick_ = now;
    if (walking_running_ == true && isGeserPreset(current_preset_) == true)
    {
      const bool left = (toLower(current_preset_) == toLower(preset_left_));
      lateral_net_sec_ += left ? dt : -dt;
      if (gkEdgeActive() == true)
      {
        // Waktu efektif tick ini (skala soft start), dalam pecahan jarak tengah->tepi.
        const double u1 = std::chrono::duration<double>(now - geser_started_at_).count();
        const double eff = integralSkalaSoftStart(u1 - dt, u1, geser_soft_start_sec_);
        lateral_pos_ += left ? eff / gk_edge_left_sec_ : -eff / gk_edge_right_sec_;
      }
      // Batas tercapai -> soft stop. Geser yang soft start-nya belum selesai
      // tidak dihentikan dulu (aturan geser yang sudah ada).
      if (geserBudgetLeft(current_preset_) == false && geserSoftStartDone() == true)
        softStopWalking(left ? "GESER KIRI" : "GESER KANAN");
    }
    // Soft stop sudah tuntas: kembalikan stance ke init pose. Hanya bila tidak
    // ada gait lain yang sudah dimulai sementara itu.
    if (soft_stop_pending_init_ == true && walking_running_ == false && now >= walk_settle_until_)
    {
      soft_stop_pending_init_ = false;
      applyPreset(preset_init_, false);
    }
  }

  // Mode kiper: tendangan yang tidak terpakai dalam gk_kick_valid_sec_ (mis.
  // robot sedang INIT/bangun) dibuang, beserta kuncinya -- kalau tidak, kiper
  // jongkok terlambat untuk bola yang sudah lewat.
  // Pemicu bola dekat yang muncul saat sedang menahan bola / jatuh sudah basi.
  if (state_ == BLOCK || state_ == FALL_FRONT || state_ == FALL_BACK)
    gk_close_trigger_ = false;

  if (gk_kick_ == true && gkKickPending() == false && state_ != BLOCK)
  {
    // Dengan kalibrasi jarak, tendangan piksel memang tak pernah memicu page 14
    // sendiri -- kedaluwarsa = kunci target dilepas, bukan kejadian luar biasa.
    if (gk_calib_ok_ == true)
      RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                  "KIPER: kunci tendangan dilepas (bola tak mendekat, state %s)", stateName(state_));
    else
      RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                  "KIPER: tendangan kedaluwarsa tanpa page %d (state %s)", page_block_, stateName(state_));
    gk_kick_ = false;
    gk_lock_ = false;
  }

  // Status bola diperbarui tiap siklus. BallTracker di sini tidak menggerakkan
  // kepala (use_head_control=false), jadi ini murni pembacaan posisi.
  tracking_status_ = ball_tracker_.processTracking();
  if (tracking_status_ == BallTracker::Found)
    ball_lost_count_ = 0;
  else
    ball_lost_count_++;

  // Laporan diagnosa: yaw setelah gait BENAR-BENAR tuntas vs saat stop dikirim.
  // "lewat" = sisa putaran searah mutar sesudah perintah stop; kalau besar,
  // itulah sebab robot melewati titik lepas.
  if (overshoot_pending_ == true && std::chrono::steady_clock::now() >= walk_settle_until_)
  {
    const double release = overshoot_was_right_ ? yaw_release_deg_ : -yaw_release_deg_;
    const double past = overshoot_was_right_ ? (release - present_yaw_) : (present_yaw_ - release);
    RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                "%s selesai: stop di yaw %.1f, setelah gait tuntas yaw %.1f "
                "(sisa putar %.1f derajat, lewat titik lepas %+.1f)",
                overshoot_was_right_ ? "MUTAR KANAN" : "MUTAR KIRI",
                overshoot_stop_yaw_, present_yaw_,
                std::fabs(present_yaw_ - overshoot_stop_yaw_), past);
    overshoot_pending_ = false;
  }

  //-- PRIORITAS 1 : JATUH ---------------------------------------------------
  // Boleh memotong apa pun, termasuk page tahan bola yang sedang berjalan. Satu-
  // satunya yang tidak dipotong adalah gerakan bangun itu sendiri.
  if (stand_state_ != Stand && state_ != FALL_FRONT && state_ != FALL_BACK && state_ != RECOVERY_INIT)
  {
    enterState(stand_state_ == Fallen_Forward ? FALL_FRONT : FALL_BACK);
  }

  switch (state_)
  {
    case IDLE:
      break;

    //-- FIXED INIT POSE: page acuan -> offset versi preset_init_, amplitudo nol
    // Titik awal TIDAK BOLEH warisan pose gerakan terakhir. walking_module
    // menangkap pose yang sedang dipegang pada saat ia diaktifkan dan memakainya
    // sebagai dasar mentah gait (onModuleEnable -> capture_init_pose_), jadi
    // kalau kaki masih berdiri di pose akhir tendangan/bangun/geser, pose itulah
    // yang jadi "init pose" -- persis yang dilarang operator. Karena itu page
    // acuan (WALKING_READY) dimainkan DULU lewat action_module, baru kaki
    // diserahkan ke walking_module dan offset versi preset_init_ dipasang.
    // Hasilnya sama setiap kali: D58 ON, D58 OFF->ON, maupun sesudah action page.
    case INIT:
    {
      if (state_ticks_ == 1)
      {
        init_stance_sent_ = false;
        init_stance_tick_ = 0;
        setRGBLED(0x1F, 0x1F, 0x1F);

        // Sesudah page di intercept_direct_init_pages: langsung ke init pose,
        // walking module memakai ulang dasar stance dari page acuan terakhir.
        // Kalau parameternya gagal dipasang, jatuh ke jalur page acuan -- lebih
        // lambat, tapi dasar stance tetap benar.
        if (init_direct_ == true && setWalkingBiasReuse(true) == false)
          init_direct_ = false;

        if (init_direct_ == false)
        {
          // Jalur page acuan: dasar stance HARUS ditangkap baru dari page itu.
          setWalkingBiasReuse(false);
          if (init_anchor_page_ > 0)
          {
            setModuleToDemo("action_module");
            playPage(init_anchor_page_);
          }
          break;
        }
      }

      // Page acuan harus tuntas sebelum pose-nya ditangkap walking_module.
      if (init_direct_ == false && init_anchor_page_ > 0 && isActionFinished() == false)
        break;

      if (init_stance_sent_ == false)
      {
        setBodyModuleToDemo("walking_module");
        applyPreset(preset_init_, false);
        std_msgs::msg::String cmd;
        cmd.data = "stop";
        if (walking_command_pub_)
          walking_command_pub_->publish(cmd);
        walking_running_ = false;
        current_preset_.clear();
        init_stance_sent_ = true;
        init_stance_tick_ = state_ticks_;
        if (init_direct_ == true)
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "FIXED INIT POSE dari versi \"%s\" LANGSUNG sesudah page %d (tanpa page acuan)",
                      preset_init_.c_str(), last_page_played_);
        else
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "FIXED INIT POSE dari versi \"%s\" (page acuan %d)",
                      preset_init_.c_str(), init_anchor_page_);
      }

      // init_settle_ticks_ = 0 (default): begitu stance init dikirim, DEMO_NORMAL
      // langsung jalan tanpa menunggu ramp walking selesai.
      if (state_ticks_ - init_stance_tick_ >= init_settle_ticks_)
        enterState(DEMO_NORMAL);
      break;
    }

    //-- DEMO NORMAL --------------------------------------------------------
    case DEMO_NORMAL:
    {
      // Jeda perpindahan dari INTERCEPT (lihat enterState()). Sekali pakai:
      // begitu tenggatnya lewat, cabang ini tidak pernah menahan lagi.
      if (std::chrono::steady_clock::now() < transition_hold_until_)
        break;

      //-- PRIORITAS 2 : INTERCEPT YAW --------------------------------------
      // Tidak ada ambang tambahan: masuk saat |yaw| > limit, keluar saat sudah
      // kembali ke dalam limit. Jarak 12 derajat antara pemicu kiri dan kanan
      // membuat pergantian kiri<->kanan mustahil; noise IMU sudah ditapis
      // filter alpha 0.4 yang sama dengan D56, ditambah min_state_ticks_ agar
      // satu koreksi tidak dibatalkan sekejap setelah dimulai.
      // Mode kiper: cabang ini mati total (tanpa magnetometer, KSTI 2026) --
      // tidak bergantung pada yaw yang kebetulan dipaksa 0 di imuDataCallback.
      if (gk_mode_ == false && imu_seen_ == true)
      {
        if (present_yaw_ > yaw_limit_deg_)
        {
          enterState(INTERCEPT_RIGHT);   // yaw positif -> mutar kanan
          break;
        }
        if (present_yaw_ < -yaw_limit_deg_)
        {
          enterState(INTERCEPT_LEFT);    // yaw negatif -> mutar kiri
          break;
        }
      }

      //-- PRIORITAS 3 : TAHAN BOLA (menggantikan tendang, 2026-10-30) ------
      // Bola terdeteksi dan sudah ~1 m (servo 20 menunduk melewati ambang) ->
      // page 14. Tidak ditunda soft start geser, sama seperti tendang dulu:
      // kiper harus sempat menutup sebelum bola sampai.
      //
      // Mode kiper: striker di bola lalu menendang -> page 14 SEKETIKA. Pemicu
      // servo20 di bawah tetap ada sebagai cadangan, tapi hanya bila kepala
      // memang mengikuti BOLA (kasus 3 / bola terkunci): di kasus 1-2 kepala
      // mengikuti titik halang striker+bola, jadi servo20 bukan jarak bola.
      //
      // 2026-11-09: dengan kalibrasi jarak, tendangan piksel (Δcy/Δw) TIDAK lagi
      // memicu page 14 -- 57 dari 61 jongkok di uji terakhir berasal dari sini
      // karena kamera bergoyang, bola sebenarnya diam. Ia cuma mengunci target;
      // page 14 diputuskan pemicu jarak di bawah. Tanpa kalibrasi: perilaku lama.
      if (gk_mode_ == true && gk_calib_ok_ == false && gkKickPending() == true)
      {
        gk_kick_ = false;
        if (gkSavesLeft() == true)
        {
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "KIPER: striker menendang -> page %d (maks %.1f dtk)", page_block_, gk_block_max_sec_);
          enterState(BLOCK);
          break;
        }
      }
      // Mode kiper + kalibrasi jarak (operator 2026-11-09): bola yang BENAR-BENAR
      // mendekat -- DEKAT (<= save_distance) atau CEPAT (waktu tiba <= ttc_sec)
      // -> page 14. Bola diam tidak pernah memicu (lihat updateBallTracks()).
      if (gk_mode_ == true && gk_calib_ok_ == true && gk_close_trigger_ == true)
      {
        gk_close_trigger_ = false;
        if (gk_save_dry_run_ == true)
        {
          // Mode tera: catat saja, tanpa page 14 dan tanpa memakai jatah saving.
          RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"), "KIPER: [UJI] akan page %d -- %s",
                      page_block_, gk_close_reason_.c_str());
        }
        else if (gkSavesLeft() == true)
        {
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "KIPER: %s -> page %d",
                      gk_close_reason_.c_str(), page_block_);
          enterState(BLOCK);
          break;
        }
      }
      {
        const double servo20 = servo20Deg();
        const bool target_seen  = gk_mode_ ? gkTargetFresh() : (tracking_status_ == BallTracker::Found);
        const bool target_fresh = gk_mode_ ? gkTargetFresh() : ballIsFresh();
        // Cadangan servo20 lama: di mode kiper hanya bila kalibrasi jarak TIDAK ada.
        const bool servo20_is_ball = (gk_mode_ == false) ||
                                     (gk_calib_ok_ == false && (gk_lock_ == true || gk_kasus_ == 3));
        if (block_armed_ == false && target_seen == true && servo20_is_ball == true &&
            servo20 >= 0.0 && servo20 < block_release_servo20_deg_)
        {
          // Bola TERLIHAT lebih jauh dari titik lepas -> bola baru boleh ditahan.
          block_armed_ = true;
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "TAHAN BOLA siap lagi: bola terlihat jauh (servo20 %.1f deg)", servo20);
        }
        if (head_joint_seen_ == true && target_fresh == true && servo20_is_ball == true &&
            servo20 > block_servo20_deg_)
        {
          if (block_armed_ == true && (gk_mode_ == false || gkSavesLeft() == true))
          {
            RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                        "TAHAN BOLA: bola ~1 m (servo20 %.1f > %.1f deg, bola %.1f%%) -> page %d",
                        servo20, block_servo20_deg_, gk_mode_ ? gkTargetPercent() : ballPercent(),
                        page_block_);
            enterState(BLOCK);
            break;
          }
          RCLCPP_INFO_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 2000,
                               "TAHAN BOLA ditahan: bola yang barusan ditahan masih dekat "
                               "(servo20 %.1f deg) -- menunggu bola terlihat < %.1f",
                               servo20, block_release_servo20_deg_);
        }
      }

      //-- PRA-SIAGA KIPER (operator 2026-11-09) ------------------------------
      // Ada bola yang benar-benar mendekat -> berhenti melangkah SEKARANG (bukan
      // jongkok), supaya kaki sudah menapak saat pemicu page 14 datang dan BLOCK
      // tidak menunggu langkah geser tuntas (dulu 0,55-0,75 dtk).
      if (gk_mode_ == true && gk_threat_ == true)
      {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - gk_threat_time_).count() <= 0.5)
        {
          if (walking_running_ == true)
          {
            RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                        "KIPER: bola mendekat -> berhenti melangkah (bola %.2f m, %.2f m/s)",
                        gk_nearest_m_, gk_nearest_speed_);
            stopWalking();
          }
          lateral_ = LATERAL_NONE;
          break;
        }
        gk_threat_ = false;
      }

      //-- PRIORITAS 4 : geser kiri / tengah / geser kanan ------------------
      // Mode kiper: sasaran = titik halang striker+bola (gkTargetPercent()),
      // histeresis & pemanggil geser di bawah tidak berubah sama sekali.
      const bool target_fresh = gk_mode_ ? gkTargetFresh() : ballIsFresh();
      const char *target_name = gk_mode_ ? "target kiper" : "bola";
      if (head_joint_seen_ == false || target_fresh == false)
      {
        // Bola hilang: hentikan gerakan lateral dan tahan INIT POSE. Ini
        // perilaku yang sama dengan BallFollower::waitFollowing() pada D56 --
        // robot menunggu, kepala tetap dipindai head_tracking_node.
        // Geser yang soft start-nya belum selesai dibiarkan dulu (lihat geser_*).
        if (walking_running_ == true)
        {
          if (geserSoftStartDone() == false)
            break;
          stopWalking();
        }
        lateral_ = LATERAL_NONE;
        break;
      }

      // Histeresis geser: pemicu dan titik lepas berbeda, dan geser yang sudah
      // aktif diingat di lateral_ -- jadi bola yang melewati batas pemicu saja
      // tidak menghentikannya.
      const double pct = gk_mode_ ? gkTargetPercent() : ballPercent();
      // Mode kiper (operator 2026-11-09): histeresis dari simpangan samping --
      // +-10 cm mulai / +-4 cm berhenti (cadangan +-3,5 / +-1 deg tanpa jarak).
      // Dinyatakan lewat pct palsu supaya cabang di bawah tetap SATU jalur:
      // simpangan > pemicu -> 100, < -pemicu -> 0, di dalam titik lepas -> 50.
      double pct_geser = pct;
      if (gk_mode_ == true)
      {
        bool meter = false;
        const double err = gkLateralError(meter);
        const double trig = meter ? gk_lateral_trigger_m_ : gk_trigger_deg_;
        const double rel = meter ? gk_lateral_release_m_ : gk_release_deg_;
        // Petakan ke skala pemicu/lepas lama tanpa mengubah angka-angkanya.
        if (err > trig)
          pct_geser = right_trigger_pct_ + 1.0;
        else if (err < -trig)
          pct_geser = left_trigger_pct_ - 1.0;
        else if (std::fabs(err) < rel)
          pct_geser = 50.0;
        else
          pct_geser = (err > 0.0) ? (right_release_pct_ + right_trigger_pct_) / 2.0
                                  : (left_release_pct_ + left_trigger_pct_) / 2.0;
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 1000,
                             "KIPER geser: simpangan %+.3f %s (mulai > %.2f, lepas < %.2f), jarak bola %.2f m",
                             err, meter ? "m" : "deg", trig, rel, gk_target_dist_m_);
      }
      switch (lateral_)
      {
        case LATERAL_NONE:
          if (pct_geser > right_trigger_pct_)
          {
            lateral_ = LATERAL_RIGHT;
            RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "GESER KANAN mulai (%s %.1f%%)", target_name, pct);
          }
          else if (pct_geser < left_trigger_pct_)
          {
            lateral_ = LATERAL_LEFT;
            RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "GESER KIRI mulai (%s %.1f%%)", target_name, pct);
          }
          else if (walking_running_ == true && geserSoftStartDone() == true)
          {
            stopWalking();   // bola di tengah -> bertahan di INIT POSE
          }
          break;

        case LATERAL_RIGHT:
          if (pct_geser <= right_release_pct_ && geserSoftStartDone() == true)
          {
            stopWalking();   // -> INIT POSE
            lateral_ = LATERAL_NONE;
            RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "GESER KANAN stop (%s %.1f%%)", target_name, pct);
          }
          break;

        case LATERAL_LEFT:
          if (pct_geser >= left_release_pct_ && geserSoftStartDone() == true)
          {
            stopWalking();   // -> INIT POSE
            lateral_ = LATERAL_NONE;
            RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "GESER KIRI stop (%s %.1f%%)", target_name, pct);
          }
          break;
      }

      // Selama sub-state aktif, pastikan save-nya berjalan. startPreset() tidak
      // mengirim ulang kalau sudah berjalan, dan menunggu gait sebelumnya tuntas
      // kalau baru berpindah arah.
      if (lateral_ == LATERAL_RIGHT)
        startPreset(preset_right_);
      else if (lateral_ == LATERAL_LEFT)
        startPreset(preset_left_);
      break;
    }

    //-- INTERCEPT ----------------------------------------------------------
    case INTERCEPT_LEFT:
    case INTERCEPT_RIGHT:
    {
      // Jeda perpindahan dari DEMO_NORMAL (lihat enterState()).
      if (std::chrono::steady_clock::now() < transition_hold_until_)
        break;

      // Tetap mutar sampai yaw mencapai titik lepas:
      //   INTERCEPT_RIGHT (mutar kanan): berhenti saat yaw <= +yaw_release_deg_
      //   INTERCEPT_LEFT  (mutar kiri) : berhenti saat yaw >= -yaw_release_deg_
      const bool crossed = imu_seen_ &&
                           ((state_ == INTERCEPT_RIGHT) ? (present_yaw_ <= yaw_release_deg_)
                                                        : (present_yaw_ >= -yaw_release_deg_));

      // Diperiksa SEBELUM memutar: kalau selama jeda awal yaw sudah kembali ke
      // titik lepas (pemicunya cuma lonjakan noise), jangan mulai memutar sama
      // sekali -- langsung kembali ke DEMO_NORMAL.
      if (crossed == true && walking_running_ == false)
      {
        RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                    "INTERCEPT batal sebelum memutar: yaw %.1f sudah di titik lepas", present_yaw_);
        enterState(DEMO_NORMAL);
        break;
      }

      startPreset(state_ == INTERCEPT_LEFT ? preset_turn_left_ : preset_turn_right_);

      if (imu_seen_ == false || state_ticks_ < min_state_ticks_)
        break;
      const bool timed_out = (state_ticks_ > static_cast<int>(SPIN_RATE * intercept_max_sec_));

      if (timed_out == true && crossed == false)
      {
        RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                    "INTERCEPT dipotong setelah %.0f detik: yaw %.1f belum kembali ke +-%.1f "
                    "(cek mag_status OpenCR)", intercept_max_sec_, present_yaw_, yaw_release_deg_);
      }

      if (crossed == true || timed_out == true)
      {
        overshoot_pending_ = true;
        overshoot_was_right_ = (state_ == INTERCEPT_RIGHT);
        overshoot_stop_yaw_ = present_yaw_;
        RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"), "%s: STOP dikirim di yaw %.1f",
                    stateName(state_), present_yaw_);
        stopWalking();
        enterState(DEMO_NORMAL);
      }
      break;
    }

    //-- TAHAN BOLA ---------------------------------------------------------
    // Hanya BADAN yang diserahkan ke action_module: kepala tetap di
    // head_control_module, jadi head_tracking_node terus mengikuti bola dan
    // servo 20 tetap bisa dipakai membaca jaraknya selama pose page 14.
    case BLOCK:
    {
      const auto now = std::chrono::steady_clock::now();
      if (block_page_sent_ == false)
      {
        // Langkah terakhir gait harus tuntas dulu, kalau tidak page dimulai dari kaki
        // yang masih terangkat. 2026-11-11: TIDAK lagi menunggu walk_settle_until_
        // (satu period_time penuh, 0,56-0,69 dtk pemicu -> page 14 di log): service
        // set_present_joint_ctrl_modules sinkron dan controller sendiri menunggu
        // WalkingModule::isRunning() false sebelum menyerahkan sendi -- gait sungguh
        // berhenti di tengah double-support berikutnya (0,25-0,75 period). Loop ini
        // terblok selama itu; BLOCK memang sudah diputuskan.
        setBodyModuleToDemo("action_module");
        playPage(page_block_);
        block_page_sent_ = true;
        if (gk_mode_ == true)
        {
          gk_saves_used_++;
          RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"), "KIPER: SAVING ke-%d dari %d (page %d)",
                      gk_saves_used_, gk_max_saves_, page_block_);
        }
        // Diukur SESUDAH ganti modul (yang kini bisa memblok s.d. ~0,45 dtk), supaya
        // batas page 14 / foul 10 dtk dihitung dari page benar-benar mulai.
        block_started_at_ = std::chrono::steady_clock::now();
        setRGBLED(0x1F, 0, 0x1F);
        break;
      }

      //-- MODE KIPER (operator 2026-11-07) --------------------------------
      // Page 14 dipegang maks gk_block_max_sec_; bola tertahan di kaki ->
      // tahan gk_block_hold_sec_ -> page 2. Pembatalan cepat "bola menjauh"
      // milik D58 lama sengaja TIDAK dipakai. Terburuk 5 + 2 dtk < 10 dtk foul.
      if (gk_mode_ == true)
      {
        const double servo20 = servo20Deg();
        const bool seen = (head_joint_seen_ == true && gkTargetFresh() == true && servo20 >= 0.0);
        const double elapsed = std::chrono::duration<double>(now - block_started_at_).count();
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 1000,
                             "KIPER TAHAN BOLA %.1f dtk: bola %s, servo20 %.1f deg (kaki > %.1f)",
                             elapsed, seen ? "terlihat" : "HILANG", servo20, feet_servo20_deg_);

        // Bola tertahan: dari kalibrasi jarak (jarak < feet_distance atau bola besar
        // terpotong tepi bawah) bila ada, kalau tidak servo20 lama (> 233).
        const bool di_kaki = gk_calib_ok_
            ? (std::chrono::duration<double>(now - gk_ball_at_feet_time_).count() <= 0.3 && gk_ball_at_feet_ == true)
            : (seen == true && servo20 > feet_servo20_deg_);
        if (block_hold_started_ == false && di_kaki == true)
        {
          block_hold_started_ = true;
          block_hold_until_ = now + std::chrono::milliseconds(static_cast<int>(gk_block_hold_sec_ * 1000.0));
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "KIPER: bola tertahan di kaki (servo20 %.1f deg, %.1f dtk) -> tahan %.1f dtk",
                      servo20, elapsed, gk_block_hold_sec_);
        }

        if (isActionFinished() == false)
          break;

        const bool hold_done = (block_hold_started_ == true && now >= block_hold_until_);
        const bool max_done = (block_hold_started_ == false && elapsed >= gk_block_max_sec_);
        if (hold_done == true || max_done == true)
        {
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "KIPER TAHAN BOLA selesai sesudah %.1f dtk (%s) -> WALKING_READY, ikuti striker lagi",
                      elapsed, hold_done ? "bola tertahan" : "batas waktu");
          if (hold_done == true)
            block_armed_ = false;   // bola yang sama di kaki tak memicu cadangan servo20
          enterState(RECOVERY_INIT);
        }
        break;
      }

      const double servo20 = servo20Deg();
      const bool seen = (head_joint_seen_ == true && ballIsFresh() == true && servo20 >= 0.0);
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("InterceptDemo"), *node_->get_clock(), 1000,
                           "TAHAN BOLA: bola %s, servo20 %.1f deg (lepas < %.1f, kaki > %.1f)",
                           seen ? "terlihat" : "HILANG", servo20,
                           block_release_servo20_deg_, feet_servo20_deg_);

      // Bola sampai di kaki (jarak tendang): tahan block_hold_sec_ apa pun yang
      // terjadi sesudahnya, supaya bola benar-benar tertahan.
      if (block_hold_started_ == false && seen == true && servo20 > feet_servo20_deg_)
      {
        block_hold_started_ = true;
        block_hold_until_ = now + std::chrono::milliseconds(static_cast<int>(block_hold_sec_ * 1000.0));
        RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                    "TAHAN BOLA: bola di kaki (servo20 %.1f deg) -> tahan %.1f dtk",
                    servo20, block_hold_sec_);
      }

      if (isActionFinished() == false)
        break;

      if (block_hold_started_ == true)
      {
        if (now >= block_hold_until_)
        {
          RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                      "TAHAN BOLA selesai -> WALKING_READY, cari bola berikutnya");
          block_armed_ = false;
          enterState(RECOVERY_INIT);
        }
        break;
      }

      // Bola menjauh (atau hilang dari pandangan) sebelum sampai di kaki.
      if (seen == false || servo20 < block_release_servo20_deg_)
      {
        RCLCPP_INFO(rclcpp::get_logger("InterceptDemo"),
                    "TAHAN BOLA batal: bola %s -> WALKING_READY",
                    seen ? "menjauh" : "hilang");
        enterState(RECOVERY_INIT);
      }
      break;
    }

    //-- BANGUN -------------------------------------------------------------
    case FALL_FRONT:
    case FALL_BACK:
    {
      const int getup_page = (state_ == FALL_FRONT) ? page_getup_front_ : page_getup_back_;
      if (getup_page <= 0)
      {
        // Bangun MATI: gerakan sudah dihentikan enterState(). Robot diam sampai
        // diberdirikan operator dan tegak terus 1 dtk, baru kembali lewat page 2.
        if (state_ticks_ == 1)
        {
          upright_ticks_ = 0;
          setRGBLED(0, 0, 0x1F);
          RCLCPP_WARN(rclcpp::get_logger("InterceptDemo"),
                      "%s: bangun MATI -- robot diam, berdirikan manual", stateName(state_));
        }
        upright_ticks_ = (stand_state_ == Stand) ? upright_ticks_ + 1 : 0;
        if (upright_ticks_ >= SPIN_RATE)
          enterState(RECOVERY_INIT);
        break;
      }

      if (state_ticks_ == 1)
      {
        setModuleToDemo("action_module");
        playPage(getup_page);
        setRGBLED(0, 0, 0x1F);
      }
      else if (isActionFinished())
      {
        // Jeda 650 ms setelah action selesai, sama seperti SoccerDemo::handleFallen().
        if (state_ticks_ > static_cast<int>(SPIN_RATE * 0.65))
        {
            enterState(RECOVERY_INIT);
        }
      }
      break;
    }

    //-- kembali ke INIT POSE lalu DEMO NORMAL -------------------------------
    case RECOVERY_INIT:
    {
      if (state_ticks_ == 1)
      {
        // Modul SENGAJA dibiarkan di action_module: INIT yang menyerahkan kaki ke
        // walking_module (langsung, atau sesudah page acuan). Menyerahkannya di
        // sini berarti dua ganti-modul bolak-balik dalam sekejap.
        init_direct_ = std::find(direct_init_pages_.begin(), direct_init_pages_.end(),
                                 static_cast<int64_t>(last_page_played_)) != direct_init_pages_.end();
        setRGBLED(0x1F, 0x1F, 0x1F);
      }
      else if (state_ticks_ >= 3)
      {
        enterState(INIT);
      }
      break;
    }
  }

  // Goyang pan kepala hanya saat DIAM: DEMO_NORMAL, gait mati dan sudah tuntas.
  // Geser, page 14, INIT, jatuh -> mati (kepala murni tracking).
  // Mode kiper: tanpa goyang -- kepala harus tetap menghadap striker.
  const bool idle = (gk_mode_ == false && state_ == DEMO_NORMAL && walking_running_ == false &&
                     std::chrono::steady_clock::now() >= walk_settle_until_);
  if (idle != head_sway_on_)
    setHeadSway(idle);
}

// Disalin dari SoccerDemo::setRGBLED().
void InterceptDemo::setRGBLED(int blue, int green, int red)
{
  if (rgb_led_pub_ == nullptr)
    return;

  const int led_full_unit = 0x1F;
  const int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);

  robotis_controller_msgs::msg::SyncWriteItem msg;
  msg.item_name = "LED_RGB";
  msg.joint_name.push_back("open-cr");
  msg.value.push_back(led_value);
  rgb_led_pub_->publish(msg);
}

}  // namespace robotis_op

/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#include "op3_demo/ball_follower.h"
#include <algorithm>
#include <chrono>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace robotis_op
{

namespace
{

// Selisih sudut ke dalam (-180, 180] derajat. Wajib: yaw magnetometer melompat
// dari +179 ke -179, jadi "target - yaw" yang mentah bisa memberi 358 derajat
// dan menyuruh robot memutar hampir satu putaran penuh ke arah yang salah.
double normalizeAngleDeg(double angle_deg)
{
  while (angle_deg > 180.0)
    angle_deg -= 360.0;
  while (angle_deg <= -180.0)
    angle_deg += 360.0;

  return angle_deg;
}

std::chrono::steady_clock::duration secondsToSteady(double seconds)
{
  if (seconds < 0.0)
    seconds = 0.0;

  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
}

}  // namespace

BallFollower::BallFollower()
  : // Node("ball_follower"),
    FOV_WIDTH(35.2 * M_PI / 180),
    FOV_HEIGHT(21.6 * M_PI / 180),
    count_not_found_(0),
    count_to_kick_(0),
    on_tracking_(false),
    approach_ball_position_(NotFound),
    kick_motion_index_(40),
    camera_height_(0.56),
    camera_pitch_offset_deg_(0.0),
    // Ambang ini bukan cuma pemicu tendangan -- ia juga JARAK BERHENTI.
    // calcFootstep() diberi (jarak_bola - kick_distance), jadi robot berhenti
    // mendekat begitu jarak bola sama dengan angka ini. Terlalu besar = robot
    // berhenti sebelum bola sampai di depan kaki dan tidak pernah menendang.
    //
    // 0,54 m yang diukur operator itu jarak LURUS kamera-ke-bola; angka di sini
    // jarak TANAH. Konversinya harus memakai tinggi kamera saat DIUKUR (robot
    // berdiri, 0,56 m) dan jari-jari bola (0,11 m, ukuran 5), karena yang
    // dilihat kamera adalah PUSAT bola, bukan titik di lantai:
    //     beda tinggi = 0,56 - 0,11 = 0,45 m
    //     jarak tanah = sqrt(0,54^2 - 0,45^2) = 0,298 m
    kick_distance_(0.30),
    head_tilt_sign_(+1.0),   // head_tilt NEGATIF = MENUNDUK (diukur ulang, lihat catatan di bawah)
    log_clock_(rclcpp::Clock::make_shared()),
    // Pusatnya tetap angka yang diukur operator (157 dan 183), tapi jendelanya
    // dilebarkan +-4 derajat. Alasannya terukur di robot 2026-08-26: kepala
    // bergerak dalam LOMPATAN (tiap deteksi = satu perintah offset), bukan
    // menyapu mulus, jadi pita selebar 2 derajat nyaris tidak pernah dipijak.
    // Dari 93 sampel satu run: NOL yang masuk jendela, padahal kepala sempat
    // singgah di 154,6 (jendela kanan 156-158) dan 187,2 (jendela kiri
    // 182-184) -- meleset 1-3 derajat, dua-duanya.
    kick_pan_right_min_deg_(153.0),
    kick_pan_right_max_deg_(161.0),
    kick_pan_left_min_deg_(179.0),
    kick_pan_left_max_deg_(187.0),
    head_pan_servo_center_deg_(180.0),
    head_pan_servo_dir_(1.0),
    kick_pan_max_distance_(0.40),
    kick_ball_radius_px_(0.0),          // 0 = hitung sendiri, lihat setNode()
    kick_ball_real_radius_m_(0.11),     // bola ukuran 5
    kick_fov_width_deg_(35.2),          // sama dengan BallTracker::FOV_WIDTH
    kick_image_width_px_(640.0),        // sama dengan camera_param.yaml
    kick_ball_radius_margin_(0.10),     // ambang dipotong 10%, lihat setNode()
    radius_ready_enable_(false),        // MATI atas permintaan operator, lihat header
    distance_from_radius_(false),       // MATI atas permintaan operator, lihat header
    min_depression_deg_(3.0),
    distance_filter_n_(5),
    ball_img_pan_rad_(0.0),
    ball_img_tilt_rad_(0.0),
    pan_window_from_bearing_(true),
    preset_verified_(false),
    z_move_max_(0.037),                 // foot_height param.yaml; ditimpa getWalkingParam()
    z_move_max_captured_(false),
    z_move_min_ratio_(-1.0),   // negatif = hitung sendiri, lihat setNode()
    z_lead_enable_(true),
    z_lead_progress_(0.0),
    z_lead_step_mm_(2.5),      // ~2 detik dari angkat minimum ke angkat penuh
    kick_ready_count_(10),
    NOT_FOUND_THRESHOLD(50),
    // BATAS ATAS kecepatan jalan robot saat demo. calcFootstep() memotong langkah
    // maju di sini (fmin terhadap max_fb_step_), lalu hasilnya dikirim sebagai
    // x_move_amplitude -- jadi apa pun jarak bolanya, robot tidak akan melangkah
    // lebih panjang dari ini.
    //
    // Angka di sini cuma cadangan (18,1 mm = preset lama "dari ketua"). Nilai
    // yang benar-benar dipakai diambil setNode() dari x_move_amplitude preset
    // JALAN, supaya kecepatan jalan robot mengikuti setelan Bascorro Studio.
    max_fb_step_(18.1 * 0.001),
    MAX_RL_TURN(15.0 * M_PI / 180),
    IN_PLACE_FB_STEP(-3.0 * 0.001),
    MIN_FB_STEP(5.0 * 0.001),
    MIN_RL_TURN(5.0 * M_PI / 180),
    UNIT_FB_STEP(0.3 * 0.001),
    unit_fb_step_(0.8 * 0.001),
    UNIT_RL_TURN(0.5 * M_PI / 180),
    SPOT_FB_OFFSET(0.0 * 0.001),
    SPOT_RL_OFFSET(0.0 * 0.001),
    SPOT_ANGLE_OFFSET(0.0),
    // Radian, sama seperti nilai yang nanti datang dari walking_module. Dulu
    // ditulis 7.0 polos di sini padahal dipakai apa adanya di dalam tan() pada
    // calcFootstepDistance -- 7 radian, bukan 7 derajat. Nilainya memang selalu
    // ditimpa startFollowing() sebelum dipakai, tapi tidak ada gunanya menyimpan
    // angka yang salah 400x sebagai nilai awal.
    hip_pitch_offset_(7.9984904662224929 * M_PI / 180),
    current_pan_(-10),
    current_tilt_(-10),
    current_x_move_(0.003),
    current_r_angle_(0),
    curr_period_time_(0.8),
    accum_period_time_(0.0),
    DEBUG_PRINT(false),
    // ---- Penyelarasan yaw. Nilai di sini cuma keadaan awal; angka yang
    // ---- dipakai operator datang dari parameter kick_yaw_* di setNode().
    yaw_align_state_(YawAlignOff),
    body_yaw_deg_(0.0),
    body_yaw_seen_(false),
    mag_status_(-1),
    yaw_in_window_count_(0),
    yaw_turn_sign_(1.0),
    yaw_align_ref_error_(0.0),
    yaw_sign_settled_(true),
    yaw_align_enable_(true),
    yaw_target_deg_(0.0),
    yaw_tolerance_deg_(5.0),
    // 15 derajat = MAX_RL_TURN, batas belok yang sudah dipakai saat mengejar
    // bola. Tidak ada alasan memutar di tempat lebih ganas dari itu.
    yaw_max_turn_deg_(15.0),
    // 5 derajat = MIN_RL_TURN. Di bawah ini gait praktis tidak memutar badan
    // sama sekali, cuma menggoyang kaki -- jadi bukan angka yang boleh
    // dikecilkan tanpa mengukur ulang.
    yaw_min_turn_deg_(5.0),
    yaw_turn_gain_(0.35),
    yaw_turn_floor_deg_(5.0),
    yaw_stall_ref_error_(0.0),
    yaw_best_error_(0.0),
    // 0 = TANPA BATAS WAKTU: robot memutar sampai yaw benar-benar masuk
    // toleransi, tidak pernah menendang sebelum itu. Bawaan lama 12 detik
    // membuat robot menendang dengan galat 57-69 derajat masih tersisa
    // (terukur di robot 2026-08-28) -- persis yang tidak diinginkan.
    // Isi angka positif kalau ingin ada jalan keluar paksa.
    yaw_timeout_sec_(0.0),
    yaw_settle_sec_(0.7),
    yaw_ready_count_(3),
    yaw_brake_sec_(0.8),
    yaw_brake_max_sec_(2.5),
    yaw_brake_still_deg_(1.5),
    yaw_brake_last_yaw_(0.0),
    yaw_brake_moved_(0.0),
    last_settle_sec_(0.0),
    body_gyro_z_(0.0),
    gyro_seen_(false),
    gyro_integrating_(false),
    turned_deg_(0.0),
    turn_target_deg_(0.0),
    gyro_sign_(1.0),
    gyro_scale_(1.0),
    mag_before_deg_(0.0),
    gyro_check_sec_(1.2),
    gyro_dir_settled_(false),
    gyro_autoscale_(true),
    gyro_scale_min_(0.4),
    gyro_scale_max_(2.5),
    yaw_filter_sec_(-1.0),     // negatif = ikut periode gait; 0 = penapis mati
    yaw_strafe_gain_(0.30 * 0.001),   // 0,30 mm per derajat simpangan
    yaw_strafe_max_(12.0 * 0.001),
    yaw_runaway_deg_(45.0),
    yaw_runaway_ref_error_(0.0),
    yaw_recheck_max_(3),
    yaw_recheck_left_(0),
    yaw_turn_sign_param_(1.0),
    yaw_auto_sign_(true),
    yaw_sign_check_sec_(3.0),
    yaw_sign_margin_deg_(6.0),
    yaw_sign_min_error_deg_(20.0),
    yaw_max_age_sec_(1.0),
    yaw_stall_sec_(4.0),
    yaw_stall_margin_deg_(3.0),
    yaw_turn_step_deg_(2.5),
    refresh_foot_after_align_(true),
    // ---- Mengitari bola. Angka di sini cuma keadaan awal; presetnya dimuat
    // ---- dari config/preset_walking.yaml dan knob-nya dari parameter kick_*.
    base_walking_param_saved_(false),
    reposition_pass_count_(0),
    reposition_max_passes_(1),
    pose_reference_saved_(false),
    pose_reference_enable_(true),
    orbit_presets_loaded_(false),
    orbit_enable_(true),
    walk_preset_enable_(true),
    // ---- MENCARI CELAH: gerakan saat bola tidak terdeteksi ----------------
    // Hidup kalau bagian 'mencari_celah' ada di preset_walking.yaml; kalau
    // tidak ada, bola hilang kembali berarti berjalan di tempat seperti dulu.
    search_enable_(true),
    // 0,8 detik = titik yang sama persis dengan perilaku lama
    // (NOT_FOUND_THRESHOLD * 0,5 = 25 tick pada 30 Hz). Bola yang cuma
    // berkedip satu-dua frame karena itu tidak memutar badan.
    search_lost_sec_(0.8),
    // "Jeda sedikit" sesudah bola ketemu lagi. Disamakan 0,5 s dengan jeda
    // transformasi gerakan yang lain (transition_delay_sec_) atas permintaan
    // operator 2026-09-04; sebelumnya 1,0 s.
    search_resume_delay_sec_(0.5),
    search_state_(SearchOff),
    // +1 = aturan operator apa adanya: yaw POSITIF pakai PUTAR KIRI, yaw
    // NEGATIF pakai PUTAR KANAN. Hanya operator yang boleh membaliknya, lewat
    // kick_orbit_left_yaw_sign:=-1.0. Gyro cuma melapor, tidak menukar.
    orbit_left_yaw_sign_(1.0),
    orbit_using_left_(true),
    orbit_announced_(-1),
    reposition_enable_(true),
    reposition_use_preset_(true),
    shift_announced_(-1),
    // 0,30 mm per derajat -- sama dengan yaw_strafe_gain_ yang sudah terpakai
    // untuk menjaga bola di tengah, jadi lajunya sudah pernah terlihat wajar
    // di robot ini.
    // 1,5 mm per derajat dengan batas 25 mm.
    //
    // Angka lama (0,30 mm/deg, batas 12 mm) dipinjam dari geseran kecil yang
    // menjaga bola tetap di tengah SAMBIL berputar, dan di sini terlalu kecil
    // untuk apa pun: terukur di robot 2026-08-31, simpangan 10,2 deg cuma
    // menghasilkan 3,1 mm per langkah. Dengan satu langkah 0,7 detik itu
    // 4 mm/detik, sedangkan menutup simpangan 10 deg pada jarak setengah meter
    // butuh sekitar 9 cm. Fase itu MUSTAHIL selesai sebelum batas waktunya
    // habis, dan memang tidak pernah selesai sekali pun.
    //
    // 25 mm masih di bawah y_move_amplitude preset PUTAR yang dipakai operator
    // sendiri (30 dan 35 mm), jadi robot memang sudah terbukti bisa berjalan
    // menyamping sebesar itu.
    reposition_y_gain_(1.50 * 0.001),
    reposition_y_max_(25.0 * 0.001),
    // Setengah max_fb_step_. Di fase ini bola sudah di dekat kaki; langkah maju
    // penuh berarti menendangnya duluan dengan telapak.
    reposition_x_max_(9.0 * 0.001),
    reposition_x_ramp_(0.0),
    reposition_timeout_sec_(12.0),
    reposition_ready_count_(0),
    reposition_recheck_max_(0),
    reposition_recheck_left_(0),
    reposition_target_foot_(OutOfRange),
    reposition_foot_hysteresis_deg_(10.0),
    reposition_foot_switched_(false),
    foot_from_reposition_(false),
    reposition_y_sign_(1.0),
    reposition_auto_sign_(true),
    reposition_sign_settled_(false),
    reposition_sign_check_sec_(2.0),
    reposition_sign_margin_deg_(2.0),
    reposition_sign_min_error_deg_(3.0),
    reposition_sign_ref_deg_(-1.0),
    // 0 = setengah kick_yaw_tolerance_deg, dihitung di setNode().
    yaw_turn_stop_deg_(0.0),
    reposition_deadband_deg_(4.0),
    reposition_blind_sec_(3.0),
    reposition_blind_(false),
    reposition_stale_sec_(1.5),
    reposition_pan_last_deg_(0.0),
    // ---- Fase MENYESUAIKAN dibagi dua: MAJU dulu, baru MENGGESER ----------
    reposition_stage_(RepoShift),
    reposition_approach_enable_(true),
    approach_ready_count_(0),
    // 0,5 detik, atas permintaan operator 2026-09-04: tiap perpindahan gerakan
    // diberi jeda supaya peralihannya tidak mendadak.
    transition_delay_sec_(0.5),
    param_reload_pending_(false)
{
  // current_joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //     "/robotis/goal_joint_states", 10, std::bind(&BallFollower::currentJointStatesCallback, this, std::placeholders::_1));

  // set_walking_command_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
  // set_walking_param_pub_ = this->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  // get_walking_param_client_ = this->create_client<op3_walking_module_msgs::srv::GetWalkingParam>("/robotis/walking/get_params");

  prev_time_ = rclcpp::Clock().now();
}

BallFollower::~BallFollower()
{

}

// Semua parameter yang bisa ditera dari luar, dikumpulkan di satu tempat.
//
// Dulu badan fungsi ini berada langsung di dalam setNode(), dan itu berarti
// setiap angka di sini dibaca TEPAT SEKALI seumur hidup node. Mengubahnya
// lewat /demo_node/set_parameters (yang dipakai halaman Tendangan di
// Bascorro Studio) tersimpan rapi di penyimpanan parameter tapi tidak pernah
// dibaca lagi, jadi dari luar kelihatan seperti perintahnya hilang.
//
// Sekarang isinya bisa dipanggil ulang. Aman dipanggil berkali-kali:
// read_param() cuma mendeklarasikan parameter yang belum ada, sisanya sekadar
// membaca. Nilai yang DIHITUNG SENDIRI (ambang radius bola, pecahan terkecil
// tinggi angkat kaki, ambang berhenti memutar) ikut dihitung ulang dari angka
// yang baru -- itulah yang membuat mengubah kick_distance saja sudah cukup.
void BallFollower::loadTunableParameters()
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                 "loadTunableParameters() dipanggil tanpa node; dilewati.");
    return;
  }

  // Geometri pemicu tendangan, bisa ditera di robot tanpa compile ulang.
  // Dijaga has_parameter(): declare_parameter yang kedua kali pada nama yang
  // sama melempar exception, dan node ini dipakai bersama demo lain.
  // Tahan banting terhadap tipe yang salah. declare_parameter MELEMPAR kalau
  // nilai yang dioper dari launch bertipe lain dari nilai bawaannya -- dan
  // "193" (tanpa titik desimal) memang dibaca ROS sebagai integer, bukan
  // double. Exception itu tidak tertangkap siapa pun dan mematikan seluruh
  // op_demo_node seketika (exit -6), padahal yang terjadi cuma salah ketik
  // satu argumen. Sekarang: integer diterima apa adanya, tipe lain memakai
  // nilai bawaan sambil mencetak alasannya.
  auto read_param = [this](const std::string &name, double fallback) -> double
  {
    try
    {
      if (node_->has_parameter(name) == false)
        node_->declare_parameter(name, fallback);
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException &)
    {
      // Nilainya ada tapi bertipe lain; coba baca sebagai integer.
      try
      {
        node_->undeclare_parameter(name);
      }
      catch (const std::exception &) { }

      try
      {
        rclcpp::ParameterValue v = node_->declare_parameter(
            name, rclcpp::ParameterValue(static_cast<int64_t>(fallback)));
        const double as_double = static_cast<double>(v.get<int64_t>());
        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "parameter '%s' dioper sebagai bilangan bulat (%.0f); "
                    "dipakai sebagai %.3f. Tulis dengan titik desimal untuk menghindari ini.",
                    name.c_str(), as_double, as_double);
        return as_double;
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "parameter '%s' tidak terbaca (%s); memakai bawaan %.3f",
                     name.c_str(), e.what(), fallback);
        return fallback;
      }
    }

    // Pembacaan biasa, TAPI tahan terhadap parameter yang sudah terlanjur
    // bertipe integer.
    //
    // Perlu sejak fungsi ini boleh dipanggil ULANG. Pada panggilan pertama,
    // parameter yang dioper sebagai bilangan bulat ditangani cabang di atas dan
    // langsung dikembalikan -- baris ini tidak pernah tersentuh. Pada panggilan
    // berikutnya has_parameter() sudah true, jadi jalannya lewat sini, dan
    // get_parameter() bertipe double pada parameter integer MELEMPAR. Tanpa
    // penjagaan ini satu argumen tanpa titik desimal di launch akan mematikan
    // op_demo_node pada tekanan tombol "Kirim ke robot" yang pertama.
    try
    {
      double value = fallback;
      node_->get_parameter(name, value);
      return value;
    }
    catch (const std::exception &)
    {
      try
      {
        int64_t as_int = 0;
        node_->get_parameter(name, as_int);
        return static_cast<double>(as_int);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "parameter '%s' tidak terbaca ulang (%s); memakai %.3f",
                     name.c_str(), e.what(), fallback);
        return fallback;
      }
    }
  };
  const double previous_camera_height = camera_height_;
  const double previous_camera_pitch = camera_pitch_offset_deg_;
  const double previous_ball_radius = kick_ball_real_radius_m_;
  const double previous_tilt_sign = head_tilt_sign_;
  camera_height_  = read_param("kick_camera_height", camera_height_);
  camera_pitch_offset_deg_ = read_param("kick_camera_pitch_offset_deg", camera_pitch_offset_deg_);
  kick_distance_  = read_param("kick_distance", kick_distance_);
  head_tilt_sign_ = read_param("kick_head_tilt_sign", head_tilt_sign_);

  kick_pan_right_min_deg_ = read_param("kick_pan_right_min_deg", kick_pan_right_min_deg_);
  kick_pan_right_max_deg_ = read_param("kick_pan_right_max_deg", kick_pan_right_max_deg_);
  kick_pan_left_min_deg_  = read_param("kick_pan_left_min_deg",  kick_pan_left_min_deg_);
  kick_pan_left_max_deg_  = read_param("kick_pan_left_max_deg",  kick_pan_left_max_deg_);
  head_pan_servo_center_deg_ = read_param("head_pan_servo_center_deg", head_pan_servo_center_deg_);
  head_pan_servo_dir_        = read_param("head_pan_servo_dir", head_pan_servo_dir_);
  pan_window_from_bearing_   = read_param("kick_pan_window_from_bearing",
                                          pan_window_from_bearing_ ? 1.0 : 0.0) > 0.5;
  kick_pan_max_distance_     = read_param("kick_pan_max_distance", kick_pan_max_distance_);
  kick_ball_radius_px_       = read_param("kick_ball_radius_px", kick_ball_radius_px_);
  kick_ball_real_radius_m_   = read_param("kick_ball_real_radius_m", kick_ball_real_radius_m_);
  if (camera_height_ != previous_camera_height || camera_pitch_offset_deg_ != previous_camera_pitch ||
      kick_ball_real_radius_m_ != previous_ball_radius || head_tilt_sign_ != previous_tilt_sign)
    resetDistanceFilter();
  kick_fov_width_deg_        = read_param("kick_fov_width_deg", kick_fov_width_deg_);
  kick_image_width_px_       = read_param("kick_image_width_px", kick_image_width_px_);
  kick_ball_radius_margin_   = read_param("kick_ball_radius_margin", kick_ball_radius_margin_);
  radius_ready_enable_       = read_param("kick_radius_ready_enable",
                                          radius_ready_enable_ ? 1.0 : 0.0) > 0.5;
  distance_from_radius_      = read_param("kick_distance_from_radius",
                                          distance_from_radius_ ? 1.0 : 0.0) > 0.5;
  min_depression_deg_        = read_param("kick_min_depression_deg", min_depression_deg_);
  distance_filter_n_         = (int) read_param("kick_distance_filter_n",
                                                (double) distance_filter_n_);
  if (distance_filter_n_ < 1)
    distance_filter_n_ = 1;
  z_move_min_ratio_          = read_param("walk_z_move_min_ratio", z_move_min_ratio_);
  z_lead_enable_             = read_param("walk_z_lead_enable", z_lead_enable_ ? 1.0 : 0.0) > 0.5;
  z_lead_step_mm_            = read_param("walk_z_lead_ramp_step_mm", z_lead_step_mm_);

  // Ambang radius dihitung sendiri kalau tidak dipasang manual.
  //
  // Alasannya konkret: ambang radius TIDAK sama untuk robot berdiri dan robot
  // digantung, karena yang menentukan besar bola di gambar adalah jarak LURUS
  // kamera-ke-bola, dan itu ikut berubah saat kamera naik. Terukur di robot:
  // bola di lantai tepat di posisi kaki memberi 64,7 px saat digantung 29,5 cm,
  // tapi 101,3 px saat berdiri. Angka tetap 85 px terlalu menuntut saat
  // digantung -- bolanya harus diangkat dulu supaya cukup besar, dan itu
  // persis yang terjadi waktu diuji.
  //
  // Dengan dihitung dari kick_distance + kick_camera_height, cukup mengubah
  // kick_camera_height saat robot turun ke lantai dan ambang radiusnya ikut
  // benar sendiri.
  if (kick_ball_radius_px_ <= 0.0)
  {
    const double focal_px = (kick_image_width_px_ * 0.5)
                          / tan(kick_fov_width_deg_ * M_PI / 180.0);
    // Kamera melihat PUSAT bola, jadi beda tingginya dikurangi jari-jari bola.
    const double drop = camera_height_ - kick_ball_real_radius_m_;
    const double slant = sqrt(kick_distance_ * kick_distance_ + drop * drop);
    const double ideal_px = (slant > 0.0) ? (focal_px * kick_ball_real_radius_m_ / slant) : 0.0;

    // Ambang ideal dipotong margin, karena radius TERUKUR sistematis lebih
    // kecil daripada hitungan geometri justru saat bola sudah dekat kaki:
    // di posisi itu badan robot sendiri menutupi sebagian bola, kotak
    // deteksi YOLO ikut mengecil, dan radiusnya membaca rendah.
    //
    // Terukur 2026-08-28 (261 sampel, robot berdiri, kamera 0,56 m, operator
    // melaporkan bola berkali-kali sudah tepat di depan kaki):
    //   radius >= 92,3 px (ambang ideal 0,30 m) : 2 sampel  (1 %)
    //   radius >= 82   px                       : 7 sampel  (3 %)
    //   radius >= 75   px                       : 28 sampel (11 %)
    // Dengan ambang ideal robot TIDAK PERNAH menendang sama sekali.
    double margin = kick_ball_radius_margin_;
    if (margin < 0.0)  margin = 0.0;
    if (margin > 0.9)  margin = 0.9;

    kick_ball_radius_px_ = ideal_px * (1.0 - margin);

    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Kick radius bola dihitung sendiri: %.1f px "
                "(ideal %.1f px dipotong margin %.0f%%; dari kick_distance %.2f m "
                "+ tinggi kamera %.3f m, fokus %.0f px). "
                "Pasang kick_ball_radius_px untuk menimpanya.",
                kick_ball_radius_px_, ideal_px, margin * 100.0,
                kick_distance_, camera_height_, focal_px);
  }
  kick_ready_count_ = (int) read_param("kick_ready_count", (double) kick_ready_count_);

  // ---- Penyelarasan yaw sebelum menendang -------------------------------
  // Sengaja lewat read_param() yang sama: parameter bertipe integer dari
  // baris perintah (mis. kick_yaw_tolerance_deg:=5, tanpa titik desimal)
  // sudah pernah mematikan seluruh op_demo_node lewat exception
  // declare_parameter. Semua knob baru ikut jalur yang tahan itu.
  yaw_align_enable_    = read_param("kick_yaw_align_enable", yaw_align_enable_ ? 1.0 : 0.0) > 0.5;
  yaw_target_deg_      = read_param("kick_yaw_target_deg", yaw_target_deg_);
  yaw_tolerance_deg_   = read_param("kick_yaw_tolerance_deg", yaw_tolerance_deg_);
  yaw_max_turn_deg_    = read_param("kick_yaw_max_turn_deg", yaw_max_turn_deg_);
  yaw_min_turn_deg_    = read_param("kick_yaw_min_turn_deg", yaw_min_turn_deg_);
  yaw_turn_gain_       = read_param("kick_yaw_turn_gain", yaw_turn_gain_);
  yaw_timeout_sec_     = read_param("kick_yaw_timeout_sec", yaw_timeout_sec_);
  yaw_settle_sec_      = read_param("kick_yaw_settle_sec", yaw_settle_sec_);
  yaw_ready_count_     = (int) read_param("kick_yaw_ready_count", (double) yaw_ready_count_);
  yaw_recheck_max_     = (int) read_param("kick_yaw_recheck_max", (double) yaw_recheck_max_);
  yaw_runaway_deg_     = read_param("kick_yaw_runaway_deg", yaw_runaway_deg_);
  unit_fb_step_        = read_param("walk_x_ramp_step_mm", unit_fb_step_ * 1000.0) * 0.001;
  yaw_strafe_gain_     = read_param("kick_yaw_strafe_gain_mm_per_deg", yaw_strafe_gain_ * 1000.0) * 0.001;
  yaw_strafe_max_      = read_param("kick_yaw_strafe_max_mm", yaw_strafe_max_ * 1000.0) * 0.001;
  gyro_sign_           = read_param("kick_yaw_gyro_sign", gyro_sign_);
  gyro_scale_          = read_param("kick_yaw_gyro_scale", gyro_scale_);
  gyro_autoscale_      = read_param("kick_yaw_gyro_autoscale", gyro_autoscale_ ? 1.0 : 0.0) > 0.5;
  yaw_filter_sec_      = read_param("kick_yaw_filter_sec", yaw_filter_sec_);
  gyro_check_sec_      = read_param("kick_yaw_gyro_check_sec", gyro_check_sec_);
  yaw_brake_sec_       = read_param("kick_yaw_brake_sec", yaw_brake_sec_);
  yaw_brake_max_sec_   = read_param("kick_yaw_brake_max_sec", yaw_brake_max_sec_);
  yaw_brake_still_deg_ = read_param("kick_yaw_brake_still_deg", yaw_brake_still_deg_);
  yaw_turn_sign_param_ = read_param("kick_yaw_turn_sign", yaw_turn_sign_param_);
  yaw_auto_sign_       = read_param("kick_yaw_auto_sign", yaw_auto_sign_ ? 1.0 : 0.0) > 0.5;
  yaw_sign_check_sec_  = read_param("kick_yaw_sign_check_sec", yaw_sign_check_sec_);
  yaw_sign_margin_deg_ = read_param("kick_yaw_sign_margin_deg", yaw_sign_margin_deg_);
  yaw_sign_min_error_deg_ = read_param("kick_yaw_sign_min_error_deg", yaw_sign_min_error_deg_);
  yaw_max_age_sec_     = read_param("kick_yaw_max_age_sec", yaw_max_age_sec_);
  yaw_stall_sec_       = read_param("kick_yaw_stall_sec", yaw_stall_sec_);
  yaw_stall_margin_deg_ = read_param("kick_yaw_stall_margin_deg", yaw_stall_margin_deg_);
  yaw_turn_step_deg_   = read_param("kick_yaw_turn_step_deg", yaw_turn_step_deg_);
  refresh_foot_after_align_ =
      read_param("kick_refresh_foot_after_align", refresh_foot_after_align_ ? 1.0 : 0.0) > 0.5;

  // ---- Preset walking + mengitari bola + menggeser sesudahnya -----------
  walk_preset_enable_ = read_param("walk_preset_enable",
                                   walk_preset_enable_ ? 1.0 : 0.0) > 0.5;
  pose_reference_enable_ = read_param("walk_pose_reference_enable",
                                      pose_reference_enable_ ? 1.0 : 0.0) > 0.5;
  reposition_max_passes_ = (int) read_param("kick_reposition_max_passes",
                                            (double) reposition_max_passes_);
  search_enable_ = read_param("walk_search_enable",
                              search_enable_ ? 1.0 : 0.0) > 0.5;
  search_lost_sec_ = read_param("walk_search_lost_sec", search_lost_sec_);
  search_resume_delay_sec_ =
      read_param("walk_search_resume_delay_sec", search_resume_delay_sec_);
  orbit_enable_ = read_param("kick_orbit_enable", orbit_enable_ ? 1.0 : 0.0) > 0.5;
  orbit_left_yaw_sign_ = read_param("kick_orbit_left_yaw_sign", orbit_left_yaw_sign_);
  reposition_enable_ = read_param("kick_reposition_enable", reposition_enable_ ? 1.0 : 0.0) > 0.5;
  reposition_use_preset_ = read_param("kick_reposition_use_preset",
                                      reposition_use_preset_ ? 1.0 : 0.0) > 0.5;
  reposition_y_gain_ =
      read_param("kick_reposition_y_gain_mm_per_deg", reposition_y_gain_ * 1000.0) * 0.001;
  reposition_y_max_ = read_param("kick_reposition_y_max_mm", reposition_y_max_ * 1000.0) * 0.001;
  reposition_x_max_ = read_param("kick_reposition_x_max_mm", reposition_x_max_ * 1000.0) * 0.001;
  reposition_timeout_sec_ = read_param("kick_reposition_timeout_sec", reposition_timeout_sec_);
  reposition_recheck_max_ =
      (int) read_param("kick_reposition_recheck_max", (double) reposition_recheck_max_);
  reposition_foot_hysteresis_deg_ =
      read_param("kick_reposition_foot_hysteresis_deg", reposition_foot_hysteresis_deg_);
  reposition_y_sign_ = read_param("kick_reposition_y_sign", reposition_y_sign_);
  reposition_auto_sign_ =
      read_param("kick_reposition_auto_sign", reposition_auto_sign_ ? 1.0 : 0.0) > 0.5;
  reposition_sign_check_sec_ =
      read_param("kick_reposition_sign_check_sec", reposition_sign_check_sec_);
  reposition_sign_margin_deg_ =
      read_param("kick_reposition_sign_margin_deg", reposition_sign_margin_deg_);
  reposition_sign_min_error_deg_ =
      read_param("kick_reposition_sign_min_error_deg", reposition_sign_min_error_deg_);
  yaw_turn_stop_deg_ = read_param("kick_yaw_turn_stop_deg", yaw_turn_stop_deg_);
  reposition_deadband_deg_ =
      read_param("kick_reposition_deadband_deg", reposition_deadband_deg_);
  reposition_blind_sec_ = read_param("kick_reposition_blind_sec", reposition_blind_sec_);
  reposition_stale_sec_ = read_param("kick_reposition_stale_sec", reposition_stale_sec_);
  reposition_approach_enable_ =
      read_param("kick_reposition_approach_enable",
                 reposition_approach_enable_ ? 1.0 : 0.0) > 0.5;
  transition_delay_sec_ = read_param("kick_transition_delay_sec", transition_delay_sec_);

  // Sekali terukur, dipakai untuk sisa run -- sifat robot, bukan sifat satu
  // tendangan. Kalau koreksi otomatis dimatikan, tanda dari parameter
  // langsung dianggap final.
  reposition_sign_settled_ = (reposition_auto_sign_ == false);

  // Penilaian arah geser TIDAK BOLEH mulai di dalam pita mati: di sana robot
  // memang sengaja diam, jadi simpangannya tidak akan berubah dan tandanya
  // tidak akan pernah bisa dinilai -- yang terbaca cuma "belum bisa dinilai"
  // berulang-ulang sampai batas waktu habis.
  if (reposition_sign_min_error_deg_ < reposition_deadband_deg_ + 1.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "kick_reposition_sign_min_error_deg (%.1f) berada di dalam pita mati "
                "%.1f deg; dinaikkan ke %.1f supaya arah geser bisa dinilai.",
                reposition_sign_min_error_deg_, reposition_deadband_deg_,
                reposition_deadband_deg_ + 1.0);
    reposition_sign_min_error_deg_ = reposition_deadband_deg_ + 1.0;
  }

  // Letak berkas preset. Bertipe string, jadi tidak bisa lewat read_param()
  // yang khusus angka; jalur tahan-exception-nya tetap sama.
  std::string default_preset_path;
  try
  {
    default_preset_path =
        ament_index_cpp::get_package_share_directory("op3_demo") + "/config/preset_walking.yaml";
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                 "Tidak menemukan share directory op3_demo (%s); preset walking "
                 "hanya bisa dipakai kalau kick_preset_config diisi manual.", e.what());
  }

  preset_config_path_ = default_preset_path;
  try
  {
    if (node_->has_parameter("kick_preset_config") == false)
      node_->declare_parameter("kick_preset_config", default_preset_path);
    node_->get_parameter("kick_preset_config", preset_config_path_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                 "parameter 'kick_preset_config' tidak terbaca (%s); memakai %s",
                 e.what(), preset_config_path_.c_str());
  }

  // Presetnya dimuat SELALU, bahkan kalau mengitari bola dimatikan: bagian
  // jalan dan geser tidak ada hubungannya dengan fitur itu. Yang dimatikan
  // kick_orbit_enable cuma pemakaian pasangan putar.
  if (preset_config_path_.empty() == false)
  {
    const bool orbit_ready = loadWalkPresets(preset_config_path_);
    orbit_presets_loaded_ = (orbit_enable_ == true && orbit_ready == true);
  }

  // ---- Angka dari preset JALAN yang dipakai di LUAR gait itu sendiri -----
  //
  // Dua batas atas demo diambil dari presetnya, bukan dari angka yang
  // tertulis di kode:
  //
  //   x_move_amplitude -> batas atas langkah maju (max_fb_step_)
  //   z_move_amplitude -> batas atas tinggi angkat kaki (z_move_max_)
  //
  // Keduanya dipakai calcFootstep()/setWalkingParam() sebagai puncak ramp.
  // Diambil DI SINI, sekali, supaya z_move_min_ratio_ di bawah dihitung dari
  // angka yang benar dan supaya keduanya ikut tercetak di ringkasan.
  if (walk_preset_enable_ == true && preset_walk_.loaded == true)
  {
    const double preset_x = presetField(preset_walk_, "x_move_amplitude", 0.0);
    const double preset_z = presetField(preset_walk_, "z_move_amplitude", 0.0);

    if (preset_x > 0.001)
      max_fb_step_ = preset_x;

    if (preset_z > 0.001)
    {
      z_move_max_ = preset_z;
      // Dikunci di sini supaya getWalkingParam() tidak menangkapnya lagi dari
      // walking_module. Isi walking_module bukan lagi foot_height asli begitu
      // robot pernah berjalan pelan (setWalkingParam menerbitkan z yang sudah
      // diperkecil), dan menangkapnya ulang membuat batas atasnya menyusut
      // sendiri tiap kali demo dimulai lagi sesudah menendang.
      z_move_max_captured_ = true;
    }

    // Periode gait ikut dipasang di sini, bukan menunggu START. Ia menentukan
    // panjang blok penapis yaw dan lamanya satu tick ramp, dan ringkasan di
    // bawah mencetak keduanya -- kalau masih memakai nilai bawaan konstruktor
    // (0,8 s), angka yang tercetak bukan angka yang nanti dipakai robot.
    // startFollowing() memasangnya lagi dari parameter yang benar-benar
    // terbit, jadi keduanya tidak bisa berbeda isi.
    const double preset_period = presetField(preset_walk_, "period_time", 0.0);
    if (preset_period > 0.1)
      curr_period_time_ = preset_period;

    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Batas atas dari preset JALAN: langkah maju %.1f mm, tinggi angkat "
                "kaki %.1f mm, periode gait %.2f s.",
                max_fb_step_ * 1000.0, z_move_max_ * 1000.0, curr_period_time_);
  }

  // Pecahan terkecil tinggi angkat kaki DIHITUNG SENDIRI supaya benar-benar
  // sejajar dengan x, bukan diisi angka bulat yang kelihatan masuk akal.
  //
  // x tidak pernah mulai dari nol: langkah terkecilnya MIN_FB_STEP, jadi pada
  // langkah pertama x sudah berada di MIN_FB_STEP/max_fb_step_ dari
  // maksimumnya. Kalau pecahan z diisi 0,5 seperti sebelumnya, z mulai di
  // 50 % sementara x baru sepertiganya: z MENDAHULUI, dan di lapangan
  // terlihat seperti kaki sudah terangkat penuh padahal robot belum melangkah.
  //
  // Dengan pecahan yang sama persis, keduanya berangkat dari persentase yang
  // sama dan sampai di puncak bersama. Dihitung SESUDAH preset JALAN dibaca,
  // karena max_fb_step_ baru final di situ.
  if (z_move_min_ratio_ < 0.0)
  {
    z_move_min_ratio_ = (max_fb_step_ > 0.0) ? (MIN_FB_STEP / max_fb_step_) : 0.3;
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Pecahan terkecil tinggi angkat kaki dihitung sendiri: %.3f "
                "(= MIN_FB_STEP %.1f mm / max_fb_step_ %.1f mm), supaya z dan x "
                "berangkat dari persentase yang sama.",
                z_move_min_ratio_, MIN_FB_STEP * 1000.0, max_fb_step_ * 1000.0);
  }

  // Batas bawah tidak boleh melampaui batas atas: dengan min > max, fmin lalu
  // fmax menghasilkan min, artinya robot memutar sekencang batas bawah
  // sampai timeout dan tidak pernah menutup galat kecil.
  if (yaw_min_turn_deg_ > yaw_max_turn_deg_)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "kick_yaw_min_turn_deg (%.1f) > kick_yaw_max_turn_deg (%.1f); "
                "batas bawah diturunkan ke batas atas.",
                yaw_min_turn_deg_, yaw_max_turn_deg_);
    yaw_min_turn_deg_ = yaw_max_turn_deg_;
  }
  if (yaw_ready_count_ < 1)
    yaw_ready_count_ = 1;

  // Ambang berhenti memutar. Bawaan 0 berarti SETENGAH toleransi; alasannya
  // ada di catatan panjang di processYawAlign(). Tidak boleh lebih longgar
  // daripada toleransi verifikasinya, karena di situ putaran berhenti tepat
  // di ambang yang nanti dinyatakan gagal.
  if (yaw_turn_stop_deg_ <= 0.0)
    yaw_turn_stop_deg_ = 0.5 * yaw_tolerance_deg_;
  if (yaw_turn_stop_deg_ > yaw_tolerance_deg_)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "kick_yaw_turn_stop_deg (%.1f) lebih longgar daripada toleransi "
                "(%.1f); diturunkan ke setengah toleransi.",
                yaw_turn_stop_deg_, yaw_tolerance_deg_);
    yaw_turn_stop_deg_ = 0.5 * yaw_tolerance_deg_;
  }

  // Tanda putaran dipasang SEKALI di sini, bukan tiap kali menyelaraskan:
  // hasil koreksi otomatis (kalau sampai terjadi) berlaku untuk sisa run,
  // jadi tendangan kedua dan seterusnya sudah berputar ke arah yang benar
  // sejak detik pertama tanpa perlu meluncurkan ulang demo.
  yaw_turn_sign_ = yaw_turn_sign_param_;
  yaw_sign_settled_ = (yaw_auto_sign_ == false);

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Kick geometry: camera_height=%.3f m, camera_pitch_offset=%.2f deg, kick_distance=%.3f m, "
              "head_tilt_sign=%+.0f (%s = MENUNDUK) | JARAK BOLA diukur dari %s",
              camera_height_, camera_pitch_offset_deg_, kick_distance_, head_tilt_sign_,
              (head_tilt_sign_ > 0.0) ? "head_tilt negatif" : "head_tilt positif",
              distance_from_radius_ ? "JARI-JARI BOLA (sudut kepala jadi cadangan)"
                                    : "SUDUT SENDI KEPALA + geometri, jari-jari tidak dipakai");
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Mutu jarak: tinggi efektif %.3f m (tinggi kamera %.3f dikurangi jari-jari "
              "bola %.3f, karena kepala mengunci PUSAT bola) | sudut tunduk minimum %.1f deg, "
              "di bawah itu bacaan dibuang dan pemicu tendangan ditahan | penapis median "
              "%d sampel%s",
              fmax(camera_height_ - kick_ball_real_radius_m_, 0.05), camera_height_,
              kick_ball_real_radius_m_, min_depression_deg_, distance_filter_n_,
              (distance_filter_n_ <= 1) ? " (MATI)" : "");
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Kick pan window: kanan %.1f..%.1f deg, kiri %.1f..%.1f deg "
              "(servo head_pan, pusat %.1f, arah %+.0f), maks %.2f m, butuh %d siklus "
              "| dinilai dari %s",
              kick_pan_right_min_deg_, kick_pan_right_max_deg_,
              kick_pan_left_min_deg_, kick_pan_left_max_deg_,
              head_pan_servo_center_deg_, head_pan_servo_dir_,
              kick_pan_max_distance_, kick_ready_count_,
              pan_window_from_bearing_
                  ? "ARAH BOLA (geometri 3D: sudut leher + tunduk kamera + letak bola "
                    "di gambar). Diukur 2026-09-04 kepala menunduk 55-57 deg, dan di "
                    "situ 1 deg di gambar = 2 deg arah badan; itulah yang dulu membuat "
                    "kaki tertukar"
                  : "SUDUT LEHER apa adanya (perilaku lama; "
                    "kick_pan_window_from_bearing:=1.0 untuk memakai arah bola)");
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Kick radius bola: >= %.1f px -- %s. Ambangnya tetap dihitung dan "
              "dicetak di telemetri; nyalakan lagi dengan "
              "kick_radius_ready_enable:=1.0 kalau radius detektor sudah stabil.",
              kick_ball_radius_px_,
              (radius_ready_enable_ && kick_ball_radius_px_ > 0.0)
                  ? "IKUT memutuskan tendangan dan selesainya geseran"
                  : "MATI, tidak ikut memutuskan apa pun");
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Penyelarasan yaw sebelum menendang: %s | target %+.1f +-%.1f deg "
              "| putar %.1f..%.1f deg (gain %.2f, tanda %+.0f, koreksi otomatis %s) "
              "| batas waktu %s, reda %.1f s, butuh %d siklus "
              "| berhenti memutar di %.1f deg (gyro), divonis %.1f deg (magnetometer) "
              "| batas bawah naik %.1f deg tiap %.1f s kalau macet "
              "| kaki dihitung ulang sesudah memutar: %s (tidak berlaku kalau "
              "fase menggeser sudah menetapkannya)",
              yaw_align_enable_ ? "AKTIF" : "MATI",
              yaw_target_deg_, yaw_tolerance_deg_,
              yaw_min_turn_deg_, yaw_max_turn_deg_, yaw_turn_gain_,
              yaw_turn_sign_param_, yaw_auto_sign_ ? "ya" : "tidak",
              (yaw_timeout_sec_ > 0.0) ? "ada" : "TANPA BATAS (tendang hanya kalau sudah lurus)",
              settleWaitSec(yaw_settle_sec_), yaw_ready_count_,
              yaw_turn_stop_deg_, yaw_tolerance_deg_,
              yaw_turn_step_deg_, yaw_stall_sec_,
              refresh_foot_after_align_ ? "ya" : "tidak");
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Yaw dibaca lewat penapis %s: dua blok %.2f s berturut dirata-rata "
              "lalu diekstrapolasi setengah blok, supaya goyangan badan seirama "
              "langkah tidak ikut memutuskan kapan berhenti memutar | pengali gyro "
              "%.2f, diukur ulang sendiri tiap putaran: %s (batas %.1f..%.1f)",
              (yaw_filter_sec_ != 0.0) ? "AKTIF" : "MATI",
              yawFilterBlockSec(), gyro_scale_,
              gyro_autoscale_ ? "ya" : "tidak",
              gyro_scale_min_, gyro_scale_max_);

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "MENGITARI bola sebelum menendang: %s (preset %s) "
              "| yaw POSITIF -> %s, yaw NEGATIF -> %s (tetap; gyro cuma melapor) "
              "| MENYESUAIKAN kaki sesudahnya: %s (%s), DILEWATI kalau bola sudah "
              "di depan kaki; samping maks %.1f mm (%.2f mm/deg, jalur lama saja), "
              "pita mati %.1f deg, kaki dikunci dgn histeresis %.1f deg, batas waktu "
              "%.1f s (menyerah sesudah %.1f s tanpa umpan balik), jatah putar ulang %d",
              orbitActive() ? "AKTIF" : (orbit_enable_ ? "MATI (preset gagal dimuat)" : "MATI"),
              preset_config_path_.empty() ? "(tidak diketahui)" : preset_config_path_.c_str(),
              (orbit_left_yaw_sign_ > 0.0) ? "PUTAR KIRI" : "PUTAR KANAN",
              (orbit_left_yaw_sign_ > 0.0) ? "PUTAR KANAN" : "PUTAR KIRI",
              reposition_enable_ ? "aktif" : "mati",
              shiftPresetActive()
                  ? "preset GESER KIRI/KANAN"
                  : (reposition_use_preset_ ? "preset geser TIDAK ADA, pakai jalur lama"
                                            : "jalur lama, geseran sebanding simpangan"),
              reposition_y_max_ * 1000.0, reposition_y_gain_ * 1000.0,
              reposition_deadband_deg_,
              reposition_foot_hysteresis_deg_,
              reposition_timeout_sec_, reposition_blind_sec_,
              reposition_recheck_max_);
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Tanda menyesuaikan kaki %+.0f (koreksi otomatis %s): +1 = bola di KIRI "
              "jendela dijawab GESER KIRI. Dinilai dari sudut leher sesudah %.1f s "
              "menggeser, dibalik kalau simpangan bola justru membesar lebih dari "
              "%.1f deg.",
              reposition_y_sign_, reposition_auto_sign_ ? "ya" : "tidak",
              reposition_sign_check_sec_, reposition_sign_margin_deg_);
  if (z_lead_enable_ == true)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "URUTAN RAMP JALAN: z DULU, baru x. Tinggi angkat kaki naik %.2f mm "
                "tiap tick ramp sampai batas atas foot_height; selama itu langkah maju "
                "ditahan di %.1f mm. Sesudah z penuh, x merambat %.2f mm tiap tick "
                "sampai %.1f mm. Satu tick = seperempat periode gait. Setel "
                "walk_z_lead_enable:=0.0 untuk menaikkan keduanya berbarengan seperti "
                "sebelumnya.",
                z_lead_step_mm_, MIN_FB_STEP * 1000.0, unit_fb_step_ * 1000.0,
                max_fb_step_ * 1000.0);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "URUTAN RAMP JALAN: z dan x naik BERBARENGAN dengan persentase yang "
                "sama. Langkah maju merambat %.2f mm tiap tick dari %.1f mm ke %.1f mm, "
                "dan tinggi angkat kaki mengikuti persentasenya. Setel "
                "walk_z_lead_enable:=1.0 supaya z penuh dulu, baru x maju.",
                unit_fb_step_ * 1000.0, MIN_FB_STEP * 1000.0, max_fb_step_ * 1000.0);
  }
}

void BallFollower::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ != nullptr)
  {
    // Dibaca di sini, dan boleh dibaca ulang kapan saja sesudahnya -- lihat
    // pendaftaran di bawah.
    loadTunableParameters();

    // ---- Menera dari Bascorro Studio tanpa meluncurkan ulang demo ---------
    //
    // Halaman Tendangan di Studio menulis lewat /demo_node/set_parameters.
    // Callback ini SENGAJA tidak membaca ulang sendiri: di Humble
    // add_on_set_parameters_callback berjalan SEBELUM nilai barunya disimpan,
    // jadi get_parameter() di dalamnya masih mengembalikan angka yang lama dan
    // pembacaan ulangnya akan sia-sia. Yang dikerjakannya cuma menaikkan
    // bendera; pencacah di bawahnya yang membaca, sesudah nilainya benar-benar
    // tersimpan.
    //
    // demo_node memutar rclcpp::spin_some() terus-menerus, jadi pencacah ini
    // tetap jalan walaupun demo sedang berhenti -- operator bisa menera dulu,
    // baru menekan START.
    param_reload_pending_ = false;
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const rclcpp::Parameter &parameter : parameters)
          {
            if (parameter.get_name() != "kick_camera_pitch_offset_deg")
              continue;
            const auto type = parameter.get_type();
            const double offset = type == rclcpp::ParameterType::PARAMETER_DOUBLE ? parameter.as_double() :
                type == rclcpp::ParameterType::PARAMETER_INTEGER ? parameter.as_int() :
                std::numeric_limits<double>::quiet_NaN();
            if (!std::isfinite(offset) || std::abs(offset) > 60.0)
            {
              result.successful = false;
              result.reason = "kick_camera_pitch_offset_deg harus numerik berhingga dalam -60..60 derajat";
              return result;
            }
          }
          for (const rclcpp::Parameter &parameter : parameters)
          {
            const std::string &name = parameter.get_name();
            if (name.rfind("kick_", 0) == 0 || name.rfind("walk_", 0) == 0)
            {
              param_reload_pending_ = true;
              break;
            }
          }
          return result;
        });

    param_reload_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(PARAM_RELOAD_POLL_MS),
        [this]()
        {
          if (param_reload_pending_ == false)
            return;

          param_reload_pending_ = false;
          RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                      "Parameter tendangan diubah dari luar (Bascorro Studio atau "
                      "ros2 param set); semuanya dibaca ulang sekarang.");
          // Kenyamanan menera TIDAK BOLEH bisa mematikan demo yang sedang
          // berjalan: exception yang lolos dari callback pencacah akan
          // menghentikan seluruh op_demo_node, dan gejalanya di lapangan
          // menyesatkan (robot berjalan terus, tombol berhenti tidak berfungsi).
          try
          {
            loadTunableParameters();
          }
          catch (const std::exception &e)
          {
            RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                         "Gagal membaca ulang parameter tendangan (%s); robot tetap "
                         "memakai angka yang lama.", e.what());
          }
        });

    current_joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/robotis/goal_joint_states", 10, std::bind(&BallFollower::currentJointStatesCallback, this, std::placeholders::_1));

    // Persistent publishers to walking_module — created ONCE here so DDS
    // discovery is done well before any button press and no walking "start"/
    // "stop" is ever silently dropped (see the header note).
    set_walking_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
    set_walking_param_pub_ = node_->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set");
  }
}

void BallFollower::startFollowing()
{
  on_tracking_ = true;
  RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Start Ball following");

  // Ramp langkah dikembalikan ke titik AWAL tiap kali mulai mengejar bola.
  //
  // current_x_move_ adalah keadaan ramp yang dipakai calcFootstep(); ia diisi
  // setWalkingParam() tiap tick dan sebelumnya TIDAK PERNAH dikembalikan.
  // Sesudah menendang, demo mulai lagi sambil membawa nilai terakhirnya, jadi
  // langkah pertama langsung meloncat ke tengah ramp alih-alih merambat dari
  // MIN_FB_STEP. Karena tinggi angkat kaki kini mengikuti kemajuan ramp yang
  // sama, z ikut meloncat -- persis keluhan "rapi cuma di awal, sesudah
  // menendang tidak rapi lagi".
  //
  // 0.003 m = nilai awal yang sama dengan di konstruktor.
  current_x_move_ = 0.003;
  current_r_angle_ = 0.0;
  accum_period_time_ = 0.0;

  // Ramp z ikut dinolkan, dengan alasan yang sama persis: sesudah menendang
  // demo mulai lagi, dan tanpa ini fase "z dulu" cuma pernah terjadi satu kali
  // seumur run.
  z_lead_progress_ = 0.0;
  foot_from_reposition_ = false;
  resetDistanceFilter();

  // Keadaan MENCARI CELAH ikut dilupakan, dan penghitung bola-hilang dinolkan.
  // Tanpa penolan itu, demo yang dimulai lagi sesudah menendang mewarisi
  // count_not_found_ dari siklus sebelumnya dan robot bisa langsung berputar
  // di tick pertama, sebelum pelacak sempat sekali pun melihat bola.
  count_not_found_ = 0;
  resetSearchWalk();

  // ---- Preset JALAN (JALAN MENUJU MEKKAH) -------------------------------------
  //
  // Sejak tombol demo ditekan, robot berjalan dengan gerakan yang disetel
  // operator di Bascorro Studio, bukan dengan isi param.yaml.
  //
  // URUTANNYA PENTING, dan sejak 2026-09-04 dibalik. Dulu "start" dikirim
  // DULUAN, dan setWalkingCommand("start") itu sendiri memanggil
  // getWalkingParam() lalu setWalkingParam(IN_PLACE_FB_STEP, 0, 0) -- jadi
  // pesan set_params yang PERTAMA kali diterima walking_module berisi angka
  // param.yaml, bukan angka preset, dan gait sudah mulai berjalan dengan
  // angka itu. Preset baru menyusul satu putaran gait kemudian, dan sebagian
  // (amplitudo) hanya diterapkan di batas fase. Itulah kenapa jalannya masih
  // terasa bukan "JALAN MENUJU MEKKAH" walaupun isi berkas presetnya benar.
  //
  // Sekarang: preset diterbitkan UTUH lebih dulu -- 29 field apa adanya,
  // termasuk x/y/z_move_amplitude milik preset sendiri -- baru "start". Pada
  // detik tombol ditekan, isi walking_module sama persis dengan yang tersimpan
  // di Bascorro Studio. Ramp demo baru mengambil alih amplitudo sesudah itu.
  if (walk_preset_enable_ == true && preset_walk_.loaded == true)
  {
    // Dibaca sekali di sini supaya field yang TIDAK disebut preset mewarisi
    // angka yang benar-benar hidup di walking_module, bukan tebakan.
    getWalkingParam();

    // ACUAN POSE, direkam SEKALI seumur node dan hanya di sini: ini satu
    // satunya titik di mana isi walking_module dijamin masih param.yaml, yaitu
    // sebelum satu pun preset diterbitkan. Semua pemanggilan berikutnya
    // membaca preset, jadi merekam ulang justru mengunci robot ke preset.
    if (pose_reference_saved_ == false)
    {
      pose_reference_param_ = current_walking_param_;
      pose_reference_saved_ = true;
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Acuan pose direkam dari walking_module (param.yaml): "
                  "init x %+.4f y %+.4f z %+.4f roll %+.3f pitch %+.3f yaw %+.3f rad, "
                  "hip_pitch %+.4f rad, period %.3f s. Dipakai ulang tiap kali "
                  "walking_module dinyalakan lagi (mis. sesudah bangun dari jatuh) "
                  "supaya pose jalan selalu sama dengan demo pertama.",
                  pose_reference_param_.init_x_offset, pose_reference_param_.init_y_offset,
                  pose_reference_param_.init_z_offset, pose_reference_param_.init_roll_offset,
                  pose_reference_param_.init_pitch_offset, pose_reference_param_.init_yaw_offset,
                  pose_reference_param_.hip_pitch_offset, pose_reference_param_.period_time);
    }

    applyPresetOverlay(current_walking_param_, preset_walk_.fields);

    // Dasar yang ditimpa preset PUTAR dan GESER nanti, dan tempat robot
    // kembali sesudah keduanya selesai.
    base_walking_param_ = current_walking_param_;
    base_walking_param_saved_ = true;

    // Terbitkan preset APA ADANYA. publishPreset() dilewati dengan sengaja:
    // yang mau dikirim di sini justru amplitudo asli preset, tanpa ramp.
    set_walking_param_pub_->publish(current_walking_param_);

    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Preset JALAN diterbitkan UTUH sebelum start: period %.3f s, dsp %.2f, "
                "step_fb %.2f, soft start %.3f s | init pose x %+.4f y %+.4f z %+.4f "
                "roll %+.3f pitch %+.3f yaw %+.3f rad | x %.1f mm y %.1f mm z %.1f mm "
                "sudut %+.3f turn_in %+.3f | y_swap %+.4f z_swap %+.4f arm %.2f "
                "pelvis %+.3f hip_pitch %+.3f | balance %s (%.2f/%.2f/%.2f/%.2f) "
                "| aim %s | gain p%d i%d d%d",
                current_walking_param_.period_time, current_walking_param_.dsp_ratio,
                current_walking_param_.step_fb_ratio, current_walking_param_.soft_start_time,
                current_walking_param_.init_x_offset, current_walking_param_.init_y_offset,
                current_walking_param_.init_z_offset, current_walking_param_.init_roll_offset,
                current_walking_param_.init_pitch_offset, current_walking_param_.init_yaw_offset,
                current_walking_param_.x_move_amplitude * 1000.0,
                current_walking_param_.y_move_amplitude * 1000.0,
                current_walking_param_.z_move_amplitude * 1000.0,
                current_walking_param_.angle_move_amplitude,
                current_walking_param_.angle_move_amplitude_turn_in,
                current_walking_param_.y_swap_amplitude, current_walking_param_.z_swap_amplitude,
                current_walking_param_.arm_swing_gain, current_walking_param_.pelvis_offset,
                current_walking_param_.hip_pitch_offset,
                current_walking_param_.balance_enable ? "HIDUP" : "mati",
                current_walking_param_.balance_hip_roll_gain,
                current_walking_param_.balance_knee_gain,
                current_walking_param_.balance_ankle_roll_gain,
                current_walking_param_.balance_ankle_pitch_gain,
                current_walking_param_.move_aim_on ? "hidup" : "mati",
                current_walking_param_.p_gain, current_walking_param_.i_gain,
                current_walking_param_.d_gain);

    // publishWalkingCommand() bukan setWalkingCommand(): versi yang kedua
    // memanggil getWalkingParam() + setWalkingParam() lagi, dan itulah yang
    // dulu menimpa preset dengan angka param.yaml tepat sebelum gait mulai.
    publishWalkingCommand("start");

    // x/y/sudut preset itu BATAS ATAS, bukan langkah pertama: ramp demo yang
    // menaikkannya dari nol. Tanpa baris ini robot melangkah 16 mm penuh di
    // langkah pertama -- persis yang dicegah urutan ramp "z dulu, baru x".
    setWalkingParam(IN_PLACE_FB_STEP, 0.0, 0.0);

    // Periksa ulang ke robot: apa yang BENAR-BENAR tersimpan di
    // walking_module sesudah preset diterbitkan? Field yang tidak cocok
    // dicetak satu per satu, jadi "jalannya belum sama dengan JALAN MENUJU MEKKAH"
    // bisa langsung dibaca sebagai daftar angka, bukan ditebak dari gerakan.
    // Amplitudo gerak (x/y/z/sudut) sengaja dilewati: ramp demo memang
    // mengubahnya, dan itu bukan ketidakcocokan.
    verifyPresetApplied();
  }
  else
  {
    setWalkingCommand("start");
  }

  if (current_walking_param_.period_time > 0.1)
  {
    hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
    curr_period_time_ = current_walking_param_.period_time;
  }
  else
  {
    // Service unavailable or returned garbage — fall back to known defaults.
    hip_pitch_offset_ = 7.9984904662224929 * M_PI / 180;   // param.yaml hip_pitch_offset (deg)
    curr_period_time_ = 0.8;                               // param.yaml period_time: 800 (ms)
  }
}

void BallFollower::stopFollowing()
{
  on_tracking_ = false;
  count_to_kick_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Stop Ball following");

  // Gait ikut dihentikan di bawah, jadi keadaan MENCARI CELAH tidak boleh
  // tertinggal hidup: kalau tidak, demo berikutnya mengira robot masih
  // berputar dan menghabiskan satu jeda percuma sebelum berjalan.
  resetSearchWalk();

  // Zero the gait amplitudes BEFORE sending "stop" so walking_module's
  // next cycle decelerates from amplitude → 0 instead of completing the
  // last commanded forward/turn step. Visible effect: feet stop swinging
  // forward almost immediately while body finishes settling — vs the
  // old path where walking_module finished its full step (~750 ms)
  // before stopping, making the STOP button feel laggy.
  setWalkingParam(0.0, 0.0, 0.0);
  setWalkingCommand("stop");
}

void BallFollower::currentJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  double pan = 0.0, tilt = 0.0;
  int get_count = 0;

  // position boleh lebih pendek dari name (mis. pesan yang cuma membawa nama).
  // Versi lama mengindeks position[] dengan panjang name[] -- baca di luar
  // batas vector, dan itulah kelas bug yang berakhir SIGSEGV.
  const size_t count = std::min(msg->name.size(), msg->position.size());

  for (size_t ix = 0; ix < count; ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      pan = msg->position[ix];
      get_count += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      tilt = msg->position[ix];
      get_count += 1;
    }

    if (get_count == 2)
      break;
  }

  // Hanya pakai kalau KEDUANYA benar-benar ada di pesan ini. Versi lama
  // menyalin pan/tilt yang BELUM DIINISIALISASI kalau sendi kepala tidak ada
  // di pesan -- current_tilt_ jadi sampah, jarak bola ikut sampah (bisa NaN),
  // dan pembandingan "jarak < ambang" tidak pernah benar lagi. Dari luar itu
  // terlihat sebagai "robot tidak pernah menendang".
  if (get_count != 2 || !std::isfinite(pan) || !std::isfinite(tilt))
  {
    head_state_stamp_ = {};
    return;
  }

  current_pan_ = pan;
  current_tilt_ = tilt;
  head_state_stamp_ = std::chrono::steady_clock::now();
}

void BallFollower::updateBallObservation(double x_angle, double y_angle, double radius_px)
{
  if (!std::isfinite(x_angle) || !std::isfinite(y_angle) ||
      !std::isfinite(radius_px) || radius_px <= 0.0)
  {
    ball_observation_stamp_ = {};
    resetDistanceFilter();
    return;
  }
  ball_img_pan_rad_ = x_angle;
  ball_img_tilt_rad_ = y_angle;
  observed_ball_radius_px_ = radius_px;
  ball_observation_stamp_ = std::chrono::steady_clock::now();
  double azimuth = 0.0, depression = 0.0;
  ballDirection(x_angle, y_angle, azimuth, depression);
  const double distance = kickGroundDistance(camera_height_, kick_ball_real_radius_m_,
                                              depression, min_depression_deg_ * M_PI / 180.0);
  if (!hasFreshBallObservation() || !std::isfinite(distance))
  {
    resetDistanceFilter();
    return;
  }
  distance_buf_.push_back(distance);
  while (distance_buf_.size() > static_cast<size_t>(std::max(1, distance_filter_n_)))
    distance_buf_.pop_front();
}

bool BallFollower::hasFreshBallObservation() const
{
  const auto now = std::chrono::steady_clock::now();
  return now - ball_observation_stamp_ <= secondsToSteady(0.5) &&
         now - head_state_stamp_ <= secondsToSteady(0.5);
}

KickPositionDecision BallFollower::kickPositionDecision(bool ball_visible) const
{
  const bool radius_ok = !radius_ready_enable_ ||
      (std::isfinite(observed_ball_radius_px_) && observed_ball_radius_px_ >= kick_ball_radius_px_);
  const auto decision = evaluateKickPosition(ball_visible && hasFreshBallObservation(),
      ballDistanceFromObservation(), kick_distance_,
      kickFootFromHeadPan() != OutOfRange);
  return (decision == KickPositionDecision::Ready && !radius_ok)
      ? KickPositionDecision::Wait : decision;
}

bool BallFollower::canKickNow() const
{
  const double error = normalizeAngleDeg(yaw_target_deg_ - filteredBodyYawDeg());
  return kickPositionDecision(true) == KickPositionDecision::Ready &&
      evaluateKickPosition(hasFreshBallObservation(), ballDistanceFromObservation(false),
                           kick_distance_, true) == KickPositionDecision::Ready &&
      kickFootFromHeadPan() == approach_ball_position_ &&
      kickHeadingReady(yaw_align_enable_, isBodyYawUsable(), error, yaw_tolerance_deg_);
}

void BallFollower::calcFootstep(double target_distance, double target_angle, double delta_time,
                                double& fb_move, double& rl_angle)
{
  // clac fb
  double next_movement = current_x_move_;
  if (target_distance < 0)
    target_distance = 0.0;

  double fb_goal = fmin(target_distance * 0.1, max_fb_step_);
  accum_period_time_ += delta_time;
  if (accum_period_time_ > (curr_period_time_  / 4))
  {
    accum_period_time_ = 0.0;

    // TINGGI ANGKAT KAKI NAIK LEBIH DULU, langkah maju menyusul.
    //
    // z dan x dulu dinaikkan berbarengan dengan persentase yang sama, jadi
    // langkah-langkah pertama diayun dengan kaki yang baru terangkat 10 mm dan
    // telapaknya menyeret lantai -- itu yang terlihat sebagai jalan yang aneh.
    // Sekarang z merambat sendiri sampai batas atas dulu (robot melangkah di
    // langkah terkecilnya sambil kakinya makin terangkat), baru sesudah penuh
    // x mulai memanjang.
    //
    // Kenaikannya dinyatakan dalam MILIMETER z per tick supaya angkanya bisa
    // dibaca operator langsung, lalu diubah ke pecahan kemajuan di sini dengan
    // membaginya rentang z yang sebenarnya -- kalau tidak, mengubah foot_height
    // di param.yaml diam-diam mengubah lamanya fase ini juga.
    if (z_lead_enable_ == true && z_lead_progress_ < 1.0)
    {
      const double span_mm = z_move_max_ * (1.0 - z_move_min_ratio_) * 1000.0;
      const double step = (span_mm > 0.01) ? (z_lead_step_mm_ / span_mm) : 1.0;
      z_lead_progress_ = fmin(1.0, z_lead_progress_ + fmax(step, 0.0));
    }

    // MENGECIL selalu boleh: robot harus tetap bisa memperlambat langkah untuk
    // berhenti di depan bola, apa pun keadaan ramp z.
    if ((target_distance * 0.1 / 2) < current_x_move_)
      next_movement -= unit_fb_step_;
    else if (z_lead_enable_ == false || z_lead_progress_ >= 1.0)
      next_movement += unit_fb_step_;
  }
  fb_goal = fmin(next_movement, fb_goal);

  // BERHENTI kalau jarak berhentinya sudah tercapai.
  //
  // Inilah sebab robot menabrak bola, dan tidak bisa diperbaiki dengan
  // menyetel kick_distance. Baris lama fb_move = fmax(fb_goal, MIN_FB_STEP)
  // memaksa langkah maju MINIMUM 5 mm tiap langkah -- selamanya. target_distance
  // sudah dijepit ke nol di atas, jadi begitu robot sampai di kick_distance
  // fb_goal memang jadi 0, tapi fmax mengembalikannya ke +5 mm dan robot terus
  // merayap maju 5 mm per langkah sampai kakinya menyentuh bola.
  //
  // MIN_FB_STEP tetap dipakai selama masih MENDEKAT: di bawah itu langkahnya
  // terlalu kecil untuk benar-benar memindahkan badan. Yang berubah hanya
  // keadaan "sudah sampai".
  //
  // IN_PLACE_FB_STEP (-3 mm) dipakai, bukan 0, karena itu nilai yang sama
  // dengan yang dipakai setWalkingCommand("start") untuk jalan di tempat:
  // gait tetap hidup (jadi robot siap memutar untuk penyelarasan yaw) tanpa
  // maju.
  if (target_distance <= 0.0)
    fb_move = IN_PLACE_FB_STEP;
  else
    fb_move = fmax(fb_goal, MIN_FB_STEP);
  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", target_distance, fb_move, delta_time);
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "==============================================");
  }

  // calc rl angle
  double rl_goal = 0.0;
  if (fabs(target_angle) * 180 / M_PI > 5.0)
  {
    double rl_offset = fabs(target_angle) * 0.2;
    rl_goal = fmin(rl_offset, MAX_RL_TURN);
    rl_goal = fmax(rl_goal, MIN_RL_TURN);
    rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

    if (target_angle < 0)
      rl_angle *= (-1);
  }
}

// =========================================================================
//  Arah bola terhadap badan -- geometri 3 dimensi, bukan penjumlahan sudut
// =========================================================================
//
// Versi lama memakai dua penjumlahan datar:
//     arah bola = head_pan + sudut bola di gambar
//     sudut tunduk = hip_pitch - (head_tilt + y_angle)
// Keduanya hanya sahih kalau kamera MENDATAR. Begitu kepala menunduk sebesar
// phi, satu derajat pergeseran mendatar DI DALAM GAMBAR tidak lagi berarti
// satu derajat perubahan arah terhadap badan; perbandingannya 1/cos(phi).
//
// DIUKUR DI ROBOT 2026-09-04 (scripts/calib_pan_scale.py, bola diam di lantai,
// kepala diperintah pan -55..+55 deg). Sudut bola di dalam gambar dihitung
// dari FOV C920 yang memang 70,4 x 43,3 deg -- spesifikasi kamera, bukan
// tebakan -- dan hasilnya cuma SETENGAH perintahnya:
//
//     perintah  15    30    45    55   deg
//     terukur    7,5  13,6  20,5  24,3 deg
//
// Faktor 0,50 itu cos(60 deg); kecocokan terbaiknya kamera menunduk 57,2 deg.
// Jadi penjumlahan datar meleset dua kali lipat TEPAT di sudut yang menentukan
// kaki mana yang dipakai menendang.
//
// Yang dipakai sekarang: sinar ke bola dibangun di kerangka KAMERA, lalu
// diputar dengan tunduk kamera dan putaran leher ke kerangka BADAN. Azimut dan
// elevasi keluar sendiri dari sana dan benar di semua sudut tunduk; pada
// kamera mendatar hasilnya kembali sama persis dengan rumus lama.
//
// DIUJI DENGAN DATA YANG SAMA. Bolanya diam, jadi jawaban yang benar HARUS
// tetap sepanjang sapuan. Arah bola menurut kedua rumus:
//     lama  -29,9 .. +31,0 deg   (sebar 18,02 deg)
//     baru   -4,4 ..  +2,8 deg   (sebar  1,87 deg)
// Rumus lama membuat arah bola menyeberang dari -30 ke +31 derajat cuma karena
// KEPALANYA bergerak -- kaki tendang ikut bertukar kanan/kiri padahal bolanya
// tidak beranjak. Itu persis keluhan "penentuan tendang belum sesuai".
void BallFollower::ballDirection(double x_angle, double y_angle,
                                 double &azimuth_rad, double &depression_rad) const
{
  // Kerangka kamera: X ke depan, Y ke KIRI, Z ke ATAS.
  // x_angle positif = bola di KIRI gambar, y_angle positif = bola di ATAS
  // (konvensi ball_tracker, lihat x_error/y_error di sana).
  const double dx = 1.0;
  const double dy = tan(x_angle);
  const double dz = tan(y_angle);

  // Tunduk kamera terhadap badan. head_tilt_sign_ menerjemahkan tanda sendi
  // jadi "positif = menunduk"; hip_pitch_offset_ adalah condong badan yang
  // ikut menundukkan kamera.
  if (!std::isfinite(camera_pitch_offset_deg_) || std::abs(camera_pitch_offset_deg_) > 60.0)
  {
    azimuth_rad = std::numeric_limits<double>::quiet_NaN();
    depression_rad = std::numeric_limits<double>::quiet_NaN();
    return;
  }
  const double phi = hip_pitch_offset_ - head_tilt_sign_ * current_tilt_ +
      camera_pitch_offset_deg_ * M_PI / 180.0;
  const double cp = cos(phi), sp = sin(phi);

  // Pitch: putar sinar turun sebesar phi.
  const double px = dx * cp + dz * sp;
  const double py = dy;
  const double pz = -dx * sp + dz * cp;

  // Yaw: putar sebesar sudut leher (positif = kepala menoleh ke kiri).
  const double cy = cos(current_pan_), sy = sin(current_pan_);
  const double bx = px * cy - py * sy;
  const double by = px * sy + py * cy;

  azimuth_rad = atan2(by, bx);
  depression_rad = -atan2(pz, sqrt(bx * bx + by * by));
}

// x_angle : ball position (pan), y_angle : ball position (tilt), ball_size : angle of ball radius
double BallFollower::headPanServoDeg() const
{
  double pan_rad = current_pan_;

  if (pan_window_from_bearing_ == true)
  {
    // Arah BOLA, bukan arah leher. Selama kepala benar-benar mengunci bola di
    // tengah gambar keduanya sama, tapi justru saat bola sudah dekat kaki
    // kepala tidak pernah sempat (atau tidak mampu) mengunci: bola berhenti di
    // tepi gambar dan lehernya diam. Di situ sudut leher menyebut bola ada di
    // depan padahal bola sudah bergeser ke samping, dan kaki yang dipilih
    // salah. Diukur 2026-09-04: dengan kepala menunduk 57 derajat, selisih
    // keduanya sampai dua kali lipat.
    double azimuth = 0.0, depression = 0.0;
    ballDirection(ball_img_pan_rad_, ball_img_tilt_rad_, azimuth, depression);
    pan_rad = azimuth;
  }

  return head_pan_servo_center_deg_ + head_pan_servo_dir_ * pan_rad * 180.0 / M_PI;
}

int BallFollower::kickFootFromHeadPan() const
{
  const double servo_deg = headPanServoDeg();

  if (servo_deg >= kick_pan_right_min_deg_ && servo_deg <= kick_pan_right_max_deg_)
    return OnRight;
  if (servo_deg >= kick_pan_left_min_deg_ && servo_deg <= kick_pan_left_max_deg_)
    return OnLeft;

  return OutOfRange;
}

bool BallFollower::processFollowing(double x_angle, double y_angle, double ball_size,
                                   double ball_radius_px)
{
  rclcpp::Time curr_time = rclcpp::Clock().now();
  rclcpp::Duration dur = curr_time - prev_time_;
  double delta_time = dur.seconds();
  prev_time_ = curr_time;

  count_not_found_ = 0;
//  int ball_position_sum = 0;

  // Disimpan untuk headPanServoDeg(), yang dipanggil dari banyak tempat lain
  // tanpa membawa sudut bola.
  ball_img_pan_rad_ = x_angle;
  ball_img_tilt_rad_ = y_angle;

  if (!hasFreshBallObservation())
  {
    clearBallPosition();
    count_to_kick_ = 0;
    setWalkingParam(0.0, 0.0, 0.0);
    return false;
  }

  // check of getting head joints angle
  if (current_tilt_ == -10 && current_pan_ == -10)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Failed to get current angle of head joints.");
    setWalkingCommand("stop");

    on_tracking_ = false;
    approach_ball_position_ = NotFound;
    return false;
  }

  // ---- Kembali dari MENCARI CELAH ----------------------------------------
  //
  // Bola terlihat lagi, tapi robot TIDAK langsung berjalan: putarannya
  // dihentikan dulu, lalu ditahan selama search_resume_delay_sec_ -- "jeda
  // sedikit" yang diminta operator -- baru preset JALAN diterbitkan ulang.
  //
  // Selama jeda fungsi ini keluar LEBIH AWAL, jadi tidak ada satu pun perintah
  // gait lain yang terkirim: tidak ada ramp langkah maju, tidak ada pemicu
  // tendangan. Yang tetap jalan cuma pelacak kepala, yang memang node lain --
  // itulah gunanya jeda ini, memberi kepala waktu mengunci bola sebelum
  // badannya ikut bergerak.
  if (search_state_ != SearchOff)
  {
    if (search_state_ == SearchTurning)
      enterSearchResume();

    const double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - search_resume_from_).count();

    if (waited < search_resume_delay_sec_)
      return false;

    finishSearchResume();
  }

  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "   ============== Head | Ball ==============   ");
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Pan : %f | Ball X : %f", (current_pan_ * 180 / M_PI), (x_angle * 180 / M_PI));
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Tilt : %f | Ball Y : %f", (current_tilt_ * 180 / M_PI), (y_angle * 180 / M_PI));
  }

  approach_ball_position_ = OutOfRange;

  // JARAK BOLA dihitung dari SUDUT SENDI KEPALA saja -- tidak ada jari-jari
  // bola di sini sama sekali.
  //
  //     jarak_tanah = tinggi_kamera / tan(sudut_tunduk + hip_pitch_offset)
  //
  // yang ditulis sebagai H*tan(90 + t - hip) dengan t = sudut kepala dalam
  // konvensi rumus (NEGATIF saat menunduk).
  //
  // TANDA head_tilt: NEGATIF = MENUNDUK, jadi head_tilt_sign_ = +1 dan rumus
  // ROBOTIS asli dipakai apa adanya.
  //
  // Sebelumnya di sini tertulis -1 ("positif = menunduk"), dan ITU SALAH.
  // Dibuktikan dari 77 sampel telemetri run 2026-09-02
  // (op_demo_node_6476_1788373257253.log), TANPA memakai jari-jari bola:
  //
  //   - bola ada di LANTAI, jadi arah pandang ke bola HARUS di bawah horizon.
  //   - dengan "negatif = menunduk": 63 dari 77 sampel melihat ke bawah.
  //     dengan "positif = menunduk": cuma 14, dan sampel paling ekstrem
  //     (head_tilt -30,0 + bola -20,1) menempatkan bola 50 derajat DI ATAS
  //     horizon, yaitu di langit-langit. Mustahil.
  //   - pemeriksaan silang: bola yang jaraknya ~1,9 m menuntut sudut tunduk
  //     atan(0,56/1,9) = 16,4 derajat, dan head_tilt terbaca -16,2 derajat.
  //
  // Akibat tanda yang salah: jarak terbaca 1,3..31,4 m untuk bola yang
  // sebenarnya 1,1..2,5 m -- galat median 339 %. Dengan tanda yang benar
  // galat median turun jadi 33 %.
  //
  // Pengukuran LAMA yang menyimpulkan "positif = menunduk" (dua sudut
  // diperintahkan + dua foto) menilai sudut yang DIPERINTAHKAN ke pengendali
  // kepala, sedangkan angka di sini datang dari /robotis/goal_joint_states.
  // Bukti di atas hanya berbicara tentang yang kedua, dan itulah yang dipakai
  // rumus ini. Setel kick_head_tilt_sign:=-1.0 untuk mengembalikannya.
  //
  // + y_angle: sudut bola DI DALAM gambar, bukan cuma arah kepala. Rumus asli
  // hanya memakai head_tilt, jadi ia mengukur "seberapa jauh titik yang
  // DIPANDANG kepala", bukan "seberapa jauh BOLA". Selama kepala mengunci bola
  // di tengah frame keduanya sama, tapi justru saat bola mendekat kaki keduanya
  // berbeda jauh: bola turun ke tepi bawah gambar sementara kepala masih
  // menyusul (atau sudah mentok di batas sendi). Setengah-FOV vertikal 21,6 deg,
  // jadi selisih itu bisa 20 derajat lebih -- persis rentang yang menentukan
  // menendang atau tidak. y_angle sudah dalam konvensi rumus (negatif = bola di
  // bawah sumbu kamera), makanya ditambahkan apa adanya.
  //
  // KETELITIANNYA TIDAK MERATA, dan untungnya ke arah yang benar. Turunan
  // H/tan(d) terhadap d membesar saat d mengecil: pada bola jauh (kepala
  // hampir datar, d ~ 3 deg) satu derajat galat menggeser jarak lebih dari
  // 30 %, sedangkan pada ambang berhenti 0,30 m (d = 62 deg) cuma 3 %. Jadi
  // pengukuran ini paling teliti TEPAT di tempat ia dipakai memutuskan, dan
  // paling kasar di tempat yang tidak berpengaruh: di atas ~0,46 m panjang
  // langkah sudah mentok di batas atasnya, jadi bola 2 m dan bola 30 m
  // menghasilkan langkah yang sama persis.
  //
  // SUDUT TUNDUK ke pusat bola, dan AZIMUT bola, keduanya dari ballDirection().
  // Itu penulisan ulang 3 dimensi dari rumus lama H*tan(90 + t - hip):
  // tan(90 + x) = -cot(x), jadi H*tan(90 + t - hip) = H/tan(hip - t). Pada
  // kamera mendatar hasilnya sama persis; bedanya baru terasa saat kepala
  // menunduk, dan di robot ini kepala SELALU menunduk saat bolanya dekat.
  //
  // ball_size (SUDUT jari-jari bola) sengaja TIDAK ikut. Rumus asli memakainya
  // untuk menggeser bidikan dari pusat bola ke titik tumpunya di lantai, dan
  // pekerjaan itu sekarang dikerjakan tinggi efektif di bawah, tanpa perlu
  // angka apa pun dari detektor. Memakai keduanya berarti mengoreksi DUA KALI.
  // soccer_demo memang sudah mengopernya sebagai 0,0 sejak lama.
  (void) ball_size;

  double ball_azimuth_rad = 0.0, depression_rad = 0.0;
  ballDirection(x_angle, y_angle, ball_azimuth_rad, depression_rad);
  const double depression_deg = depression_rad * 180.0 / M_PI;

  // TINGGI EFEKTIF, bukan tinggi kamera. Kepala mengunci PUSAT bola, dan pusat
  // itu ada satu jari-jari bola di atas lantai -- persis potongan yang sudah
  // dipakai jalur radius (drop = camera_height - kick_ball_real_radius_m).
  // Memakai tinggi kamera penuh melebihkan jarak 0,56/0,45 = 1,244x, tetap,
  // di semua jarak.
  // Sahih hanya di antara lantai sudut dan 89 derajat. Di bawah lantai jarak
  // meledak (kepala hampir datar); di atas 89 tan berbalik tanda.
  const double observed_distance = ballDistanceFromObservation();
  const bool depression_ok = std::isfinite(observed_distance);
  const double distance_to_ball = depression_ok ? observed_distance : DISTANCE_FAR_M;

  // Sudut tunduk ke bola dipakai apa adanya lewat depression_deg; tidak ada
  // lagi salinan bertanda terbalik ("ball_y_angle") seperti di rumus lama.
  double ball_x_angle = ball_azimuth_rad * 180 / M_PI;

  // Jarak dari JARI-JARI BOLA. DIMATIKAN atas permintaan operator 2026-09-03
  // (kick_distance_from_radius bawaannya 0): pengukur jaraknya sekarang cuma
  // sudut sendi kepala + hitungan geometri di atas.
  //
  // Jalur ini dulu ada karena jarak sudut kepala terbaca kacau (1,3..31,4 m
  // untuk bola 1,1..2,5 m). Penyebabnya BUKAN rumusnya melainkan TANDA
  // head_tilt yang terbalik; sesudah tandanya dibetulkan (lihat catatan
  // panjang di atas) galat mediannya turun 339 % -> 33 % dan jalur ini tidak
  // dibutuhkan lagi.
  //
  // Kodenya sengaja dibiarkan utuh: kick_distance_from_radius:=1.0
  // mengembalikannya dalam satu argumen kalau ternyata masih diperlukan.
  //     miring = fokus * jari_jari_bola / radius_piksel
  //     tanah  = akar(miring^2 - (tinggi_kamera - jari_jari_bola)^2)
  double distance_for_walk = distance_to_ball;
  bool walk_distance_from_radius = false;

  if (distance_from_radius_ == true && ball_radius_px > 1.0 && kick_fov_width_deg_ > 0.0)
  {
    const double focal_px = (kick_image_width_px_ * 0.5)
                          / tan(kick_fov_width_deg_ * M_PI / 180.0);
    const double slant = focal_px * kick_ball_real_radius_m_ / ball_radius_px;
    const double drop = camera_height_ - kick_ball_real_radius_m_;
    const double ground_sq = slant * slant - drop * drop;

    // Akar negatif berarti bola terbaca lebih besar daripada yang mungkin ada
    // di lantai -- bola terangkat, atau kotak deteksinya melar. Di situ angka
    // ini tidak sahih dan jarak sudut kepala tetap dipakai.
    if (ground_sq > 0.0)
    {
      distance_for_walk = sqrt(ground_sq);
      walk_distance_from_radius = true;
    }
    else
    {
      distance_for_walk = 0.0;
      walk_distance_from_radius = true;
    }
  }

  double distance_to_kick = kick_distance_;

  // DUA pemicu, berdiri sendiri-sendiri:
  //
  //  (1) Jendela servo head_pan. Kepala sedang mengunci bola (fungsi ini hanya
  //      dipanggil saat tracking_status_ == Found), dan sudut lehernya sendiri
  //      yang memberi tahu bola ada di depan kaki mana. Sisi bola dicek ulang
  //      lewat ball_x_angle supaya jendela tidak kebetulan cocok saat kepala
  //      lewat begitu saja. Batas +-25 derajat SENGAJA tidak dipakai di jalur
  //      ini: kepala yang menoleh 23 derajat sudah hampir menyentuhnya, dan itu
  //      justru posisi tendang yang dimaksud.
  //
  //  (2) Jarak. Perilaku lama, tetap ada.
  const double head_pan_servo_deg = headPanServoDeg();
  const int foot_from_pan = kickFootFromHeadPan();
  const bool pan_side_ok = (foot_from_pan == OnRight && ball_x_angle < 0.0)
                        || (foot_from_pan == OnLeft && ball_x_angle > 0.0);
  // KEDUA pagar jarak ini memakai distance_for_walk, yang sejak 2026-09-03
  // datang dari sudut sendi kepala.
  //
  // Angka kacau yang dulu tercatat di sini (2,3 / 3,5 / 11,5 / 4,9 m untuk
  // bola yang jaraknya turun mulus dari 1,43 ke 0,89 m) memang nyata, tapi
  // sebabnya tanda head_tilt yang terbalik, bukan cara pengukurannya.
  //
  // Kedua pagar ini juga bekerja di jarak DEKAT (0,40 dan 0,30 m), yaitu
  // justru tempat pengukuran sudut kepala paling teliti: di situ kepala
  // menunduk 55-62 derajat dan satu derajat galat cuma bergeser 3 % jarak.
  //
  // KEDUANYA menuntut bacaan jaraknya SAHIH. Kalau sudut tunduk di bawah
  // lantainya, distance_for_walk sengaja diisi DISTANCE_FAR_M dan kedua pagar
  // ini otomatis tertutup -- tapi ditulis eksplisit supaya tetap tertutup
  // kalau suatu saat jalur radius dihidupkan lagi.
  const bool distance_ok = (distance_from_radius_ == true) ? true : depression_ok;
  const bool pan_ready = pan_side_ok && distance_ok
                      && (distance_for_walk < kick_pan_max_distance_);
  const bool near_enough = distance_ok && (distance_for_walk < distance_to_kick)
                        && (fabs(ball_x_angle) < 25.0);

  //  (3) Jari-jari bola di gambar. Pengukur jarak yang berdiri sendiri, dan
  //      paling sedikit asumsinya: tidak peduli tinggi kamera, hip_pitch,
  //      tanda head_tilt, atau apakah kepala menunjuk ke bola. Bola dekat
  //      SELALU besar. Gerbang +-25 derajat tetap dipakai di sini (beda dengan
  //      jalur jendela pan) supaya bola yang dekat tapi jauh di samping tidak
  //      ikut memicu.
  //
  //      DIMATIKAN atas permintaan operator 2026-09-01 (kick_radius_ready_enable
  //      bawaannya 0). Radius dari detektor melompat 12..87 px untuk bola yang
  //      sama, dan jalur ini pernah memicu tendangan dari 1,357 m padahal
  //      kick_distance 0,30 m. Ambangnya tetap dihitung dan tetap dicetak di
  //      telemetri supaya bisa ditera sebelum dinyalakan lagi.
  const bool radius_ready = (radius_ready_enable_ == true)
                         && (kick_ball_radius_px_ > 0.0)
                         && (ball_radius_px >= kick_ball_radius_px_)
                         && (fabs(ball_x_angle) < 25.0);

  // Selalu dicetak (1x/detik) supaya ambangnya bisa ditera langsung di robot:
  // dekatkan bola sampai jaraknya turun di bawah ambang, atau sampai servo
  // head_pan masuk salah satu jendela.
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"),
                       *log_clock_, 1000,
                       "jarak bola %.3f m (tunduk %.1f deg%s, median %d) | JALAN pakai %.3f m "
                       "(%s, ambang %.3f) | servo head_pan %.1f deg "
                       "(kanan %.0f-%.0f, kiri %.0f-%.0f) | head_tilt %+.1f deg "
                       "| bola dlm gambar %+.1f deg | radius %.1f px (ambang %.0f) "
                       "| bola x %+.1f deg | LANGKAH x %.1f mm (%.0f%%) z %.1f mm (%.0f%%)%s | siap: %s",
                       distance_to_ball, depression_deg,
                       depression_ok ? "" : " TAK SAHIH, kepala terlalu datar",
                       (int) distance_buf_.size(),
                       distance_for_walk,
                       walk_distance_from_radius ? "dari radius bola" : "dari sudut kepala",
                       distance_to_kick, head_pan_servo_deg,
                       kick_pan_right_min_deg_, kick_pan_right_max_deg_,
                       kick_pan_left_min_deg_, kick_pan_left_max_deg_,
                       current_tilt_ * 180 / M_PI, y_angle * 180 / M_PI,
                       ball_radius_px, kick_ball_radius_px_, ball_x_angle,
                       current_walking_param_.x_move_amplitude * 1000.0,
                       (max_fb_step_ > 0.0)
                           ? (current_walking_param_.x_move_amplitude / max_fb_step_ * 100.0) : 0.0,
                       current_walking_param_.z_move_amplitude * 1000.0,
                       (z_move_max_ > 0.0)
                           ? (current_walking_param_.z_move_amplitude / z_move_max_ * 100.0) : 0.0,
                       (z_lead_enable_ == true && z_lead_progress_ < 1.0)
                           ? " [z naik dulu, x ditahan]" : "",
                       radius_ready ? "radius"
                                    : (pan_ready ? "jendela pan"
                                                 : (near_enough ? "jarak"
                                                                : (pan_side_ok ? "jendela pan cocok tapi bola masih jauh"
                                                                               : "belum"))));

  // check whether ball is correct position.
  if (pan_ready || near_enough || radius_ready)
  {
    count_to_kick_ += 1;

    if (DEBUG_PRINT)
    {
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "head pan : %f | ball pan : %f", (current_pan_ * 180 / M_PI), (x_angle * 180 / M_PI));
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "head tilt : %f | ball tilt : %f", (current_tilt_ * 180 / M_PI), (y_angle * 180 / M_PI));
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "foot to kick : %f", ball_x_angle);
    }

    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "In range [%d | %f]", count_to_kick_, ball_x_angle);

    // ball queue
//    if(ball_position_queue_.size() >= 5)
//      ball_position_queue_.erase(ball_position_queue_.begin());

//    ball_position_queue_.push_back((ball_x_angle > 0) ? 1 : -1);


    if (count_to_kick_ > kick_ready_count_)
    {
      setWalkingCommand("stop");
      on_tracking_ = false;

      // Jendela servo yang menentukan kakinya kalau dialah yang memicu: itu
      // pengukuran langsung "bola ada di depan kaki mana", bukan tebakan dari
      // tanda sudut. Kalau yang memicu jaraknya, pakai aturan lama.
      if (pan_ready)
        approach_ball_position_ = foot_from_pan;
      else if (ball_x_angle > 0.02)
        approach_ball_position_ = OnLeft;
      else
        approach_ball_position_ = OnRight;

      RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                  "TENDANG kaki %s -- pemicu: %s | servo head_pan %.1f deg "
                  "| jarak %.3f m | radius %.1f px",
                  (approach_ball_position_ == OnLeft) ? "KIRI" : "KANAN",
                  radius_ready ? "radius bola" : (pan_ready ? "jendela pan" : "jarak"),
                  head_pan_servo_deg, distance_to_ball, ball_radius_px);

      return true;
    }
    else if (count_to_kick_ > kick_ready_count_ / 2)
    {
      //      if (ball_x_angle > 0)
      //        accum_ball_position_ += 1;
      //      else
      //        accum_ball_position_ -= 1;

      // send message
      setWalkingParam(IN_PLACE_FB_STEP, 0, 0);
      return false;
    }
  }
  else
  {
    count_to_kick_ = 0;
//    accum_ball_position_ = NotFound;
  }

  double fb_move = 0.0, rl_angle = 0.0;
  // Jarak dari jari-jari yang dipakai berjalan, BUKAN jarak sudut kepala.
  // Ini yang membuat robot benar-benar berhenti di kick_distance_ alih-alih
  // terus maju sampai menabrak bola.
  double distance_to_walk = distance_for_walk - distance_to_kick;

  calcFootstep(distance_to_walk, current_pan_, delta_time, fb_move, rl_angle);

  // send message
  setWalkingParam(fb_move, 0, rl_angle);

  // for debug
  //RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", distance_to_ball, fb_move, delta_time);

  return false;
}

void BallFollower::decideBallPositin(double x_angle, double y_angle)
{
  // check of getting head joints angle
  if (current_tilt_ == -10 && current_pan_ == -10)
  {
    approach_ball_position_ = NotFound;
    return;
  }

  ball_img_pan_rad_ = x_angle;
  ball_img_tilt_rad_ = y_angle;

  // Jendela servo head_pan lebih dipercaya daripada tanda sudut: ambangnya
  // 0,05 rad (2,9 derajat) dan jendela KIRI operator (servo 182-184 = +2..+4
  // derajat) duduk tepat di atasnya -- sedikit saja meleset, kakinya tertukar.
  const int foot_from_pan = kickFootFromHeadPan();
  if (foot_from_pan == OnRight || foot_from_pan == OnLeft)
  {
    approach_ball_position_ = foot_from_pan;
    return;
  }

  // Azimut penuh 3 dimensi, sama seperti yang dipakai jendela tendang.
  double ball_x_angle = 0.0, ball_depression = 0.0;
  ballDirection(x_angle, y_angle, ball_x_angle, ball_depression);

  if (ball_x_angle > 0.05)
    approach_ball_position_ = OnLeft;
  else
    approach_ball_position_ = OnRight;
}

void BallFollower::resetDistanceFilter()
{
  distance_buf_.clear();
}

void BallFollower::waitFollowing()
{
  count_not_found_++;

  // Bola hilang: isi penapis menunjuk bola yang sudah tidak terlihat lagi.
  resetDistanceFilter();

  // Sudut bola di gambar ikut basi. Dinolkan supaya headPanServoDeg() kembali
  // menyatakan arah leher apa adanya alih-alih memakai letak bola yang sudah
  // tidak ada -- itu yang dulu membuat jendela tendang tetap "cocok" beberapa
  // tick sesudah bolanya hilang.
  ball_img_pan_rad_ = 0.0;
  ball_img_tilt_rad_ = 0.0;

  // Tick PERTAMA sesudah bola hilang: dari sinilah search_lost_sec_ dihitung.
  if (count_not_found_ == 1)
    search_lost_from_ = std::chrono::steady_clock::now();

  // Bola sempat terlihat lalu hilang lagi SEBELUM jedanya habis. Jedanya
  // dibatalkan dan robot kembali berputar -- tanpa ini ia berdiri diam
  // menghitung jeda untuk bola yang sudah tidak ada.
  if (search_state_ == SearchResuming)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "MENCARI CELAH: bola hilang lagi sebelum jeda %.2f s habis; "
                "kembali berputar.", search_resume_delay_sec_);
    startSearchWalk();
    return;
  }

  // Sudah berputar: presetnya SUDAH hidup di walking_module. Menerbitkannya
  // lagi tiap tick cuma menimpa parameter dengan angka yang sama sambil
  // menambah lalu lintas -- dan pada preset yang sama itu tidak pernah
  // mempercepat apa pun.
  if (search_state_ == SearchTurning)
    return;

  const double lost_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - search_lost_from_).count();

  if (lost_sec < search_lost_sec_)
    return;

  if (searchWalkActive() == true)
  {
    startSearchWalk();
    return;
  }

  // Jalur lama: tanpa preset MENCARI CELAH robot berjalan di tempat.
  setWalkingParam(0.0, 0.0, 0.0);
}

// =========================================================================
//  MENCARI CELAH: berputar di tempat selama bola tidak terdeteksi
// =========================================================================

void BallFollower::startSearchWalk()
{
  // publishPreset() menimpakan lapisan MENCARI CELAH di atas base_walking_param_
  // (hasil preset JALAN), lalu ikut memindahkan periode gait -- yang menentukan
  // panjang blok penapis yaw dan lamanya satu tick ramp -- ke periode preset
  // ini (0,57 s, bukan 0,77 s milik JALAN).
  //
  // Gait-nya sendiri TIDAK di-start ulang: "start" sudah dikirim startFollowing()
  // dan tidak pernah dihentikan selama bola dicari, jadi robot langsung
  // berpindah dari melangkah di tempat ke berputar tanpa jeda berhenti.
  publishPreset(preset_search_);
  search_state_ = SearchTurning;

  // Hitungan "sudah N tick berturut-turut dalam posisi tendang" ikut dinolkan.
  // Bola sudah tidak terlihat selama search_lost_sec_, jadi hitungan itu
  // menggambarkan keadaan yang sudah lewat; membiarkannya berarti tendangan
  // bisa terpicu satu tick lebih awal sesudah bola ketemu lagi.
  count_to_kick_ = 0;

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "MENCARI CELAH: bola tidak terdeteksi %.2f s -> berputar di tempat "
              "(sudut %+.3f rad, turn_in %+.3f, periode %.2f s). Bola ketemu = "
              "jeda %.2f s, lalu kembali ke JALAN MENUJU MEKKAH.",
              search_lost_sec_,
              current_walking_param_.angle_move_amplitude,
              current_walking_param_.angle_move_amplitude_turn_in,
              current_walking_param_.period_time,
              search_resume_delay_sec_);
}

void BallFollower::enterSearchResume()
{
  search_state_ = SearchResuming;
  search_resume_from_ = std::chrono::steady_clock::now();

  // Putaran dihentikan DULU, gait dibiarkan hidup. Preset MENCARI CELAH tetap
  // yang tertulis di walking_module selama jeda; yang dinolkan cuma
  // amplitudonya, jadi robot melangkah di tempat sementara kepala mengunci
  // bola. Berjalan langsung dari tengah putaran berarti arah bola dibaca
  // sambil badan masih berayun.
  setWalkingParam(0.0, 0.0, 0.0);

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "MENCARI CELAH: bola terdeteksi -> berhenti berputar, jeda %.2f s "
              "sebelum kembali ke JALAN MENUJU MEKKAH.", search_resume_delay_sec_);
}

void BallFollower::finishSearchResume()
{
  search_state_ = SearchOff;

  // Kembali ke JALAN MENUJU MEKKAH lewat jalur yang sama dengan preset lain, bukan
  // dengan menerbitkan sendiri. base_walking_param_ SUDAH hasil preset JALAN
  // (disimpan startFollowing() sesudah menimpakannya ke atas param.yaml), jadi
  // menimpakan lapisan JALAN sekali lagi tidak mengubah apa pun -- yang dipakai
  // di sini justru urusan pembukuannya: jaring pengaman kalau salinan dasarnya
  // belum pernah dibuat, dan periode gait ikut dikembalikan ke 0,77 s.
  //
  // Kalau preset JALAN tidak ada (walk_preset_enable:=0.0), lapisannya kosong
  // dan yang terbit adalah isi param.yaml -- persis perilaku lama.
  publishPreset(preset_walk_);

  // Ramp langkah dimulai dari awal, alasannya sama persis dengan di
  // startFollowing(): robot baru saja berputar di tempat, jadi langkah
  // pertamanya tidak boleh langsung sepanjang batas atas preset. Fase
  // "z dulu, baru x" ikut diulang.
  current_x_move_ = 0.003;
  current_r_angle_ = 0.0;
  accum_period_time_ = 0.0;
  z_lead_progress_ = 0.0;
  setWalkingParam(IN_PLACE_FB_STEP, 0.0, 0.0);

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "MENCARI CELAH selesai: kembali ke JALAN MENUJU MEKKAH "
              "(periode %.2f s, batas langkah %.1f mm).",
              current_walking_param_.period_time, max_fb_step_ * 1000.0);
}

void BallFollower::publishWalkingCommand(const std::string &command)
{
  if (node_ == nullptr || set_walking_command_pub_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set, cannot set walking command");
    return;
  }

  std_msgs::msg::String command_msg;
  command_msg.data = command;
  set_walking_command_pub_->publish(command_msg);

  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Send Walking command : %s", command.c_str());
}

void BallFollower::setWalkingCommand(const std::string &command)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set, cannot set walking command");
    return;
  }

  // get param
  if (command == "start")
  {
    getWalkingParam();
    setWalkingParam(IN_PLACE_FB_STEP, 0, 0);
  }

  publishWalkingCommand(command);
}

void BallFollower::setWalkingParam(double x_move, double y_move, double rotation_angle)
{
  // Hanya tiga field ini yang boleh ditimpa demo. Sisanya -- init pose, timing,
  // bentuk gait, gain, dan balance_enable -- tetap seperti yang dibaca
  // getWalkingParam() dari walking_module, yaitu isi param.yaml.
  current_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET;
  current_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET;
  current_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

  // Tinggi angkat kaki naik BERTAHAP bersama besar gerakan, dengan foot_height
  // di param.yaml sebagai BATAS ATAS -- bukan lagi nilai tetap yang dipakai
  // sejak langkah pertama.
  //
  // Serasi dengan x_move_amplitude: x sendiri sudah dinaikkan bertahap oleh
  // calcFootstep() dalam kelipatan UNIT_FB_STEP sampai max_fb_step_, jadi
  // langkah kecil sekarang juga berarti angkat kaki rendah, dan kaki baru
  // terangkat penuh saat langkahnya memang penuh.
  //
  // Sudut putar ikut diperhitungkan, BUKAN cuma x. Kalau tidak, fase memutar
  // di tempat (x = 0, hanya angle) akan turun ke tinggi angkat minimum dan
  // kaki tidak cukup terangkat untuk memutar -- penyelarasan yaw akan macet
  // justru karena perbaikan ini.
  // Yang dipakai KEMAJUAN RAMP-nya, bukan nilai mutlaknya.
  //
  // x tidak pernah mulai dari nol: calcFootstep() menjepitnya di MIN_FB_STEP
  // (5,0 mm) dan menaikkannya bertahap sampai max_fb_step_ (18,1 mm). Kalau z
  // dipetakan dari x = 0, maka saat x baru mulai bergerak (kemajuan 0 %) z
  // sudah berada di 28 % perjalanannya -- keduanya naik, tapi tidak
  // berbarengan.
  //
  // Dengan memetakan rentang yang SAMA (MIN..MAX untuk x, ratio..1 untuk z),
  // keduanya berangkat bersama dan sampai di puncak bersama.
  auto ramp_progress = [](double value, double lo, double hi) -> double
  {
    if (hi <= lo)
      return 0.0;
    const double p = (fabs(value) - lo) / (hi - lo);
    if (p < 0.0) return 0.0;
    if (p > 1.0) return 1.0;
    return p;
  };

  // Dengan URUTAN RAMP "z dulu", sumbangan x bukan lagi kemajuan ramp x
  // melainkan kemajuan ramp z sendiri: selama fase itu x memang sengaja
  // ditahan di langkah terkecil, jadi memetakan z dari x akan mengunci z di
  // nol persen dan fasenya tidak pernah jalan.
  //
  // Syaratnya robot memang sedang MELANGKAH MAJU. Kalau tidak, ramp z yang
  // sudah penuh dari perjalanan tadi akan ikut terbawa ke fase-fase yang
  // amplitudonya nol -- mengerem, memutar, menggeser -- dan robot mengangkat
  // kaki 37 mm di tempat justru saat ia harus setenang mungkin supaya
  // magnetometernya bisa dibaca. Di luar itu perhitungannya kembali seperti
  // semula dan suku putaran serta geseran di bawah yang menentukan.
  double motion = (z_lead_enable_ == true && fabs(x_move) >= MIN_FB_STEP)
                      ? z_lead_progress_
                      : ramp_progress(x_move, MIN_FB_STEP, max_fb_step_);

  // Putaran dihitung dengan rentangnya sendiri lalu diambil yang terbesar.
  // Tanpa ini, fase memutar di tempat (x = 0, hanya angle) jatuh ke kemajuan
  // nol dan kaki tidak cukup terangkat untuk memutar.
  motion = fmax(motion, ramp_progress(rotation_angle, MIN_RL_TURN, MAX_RL_TURN));

  // Geseran menyamping ikut, dengan alasan yang persis sama seperti putaran:
  // fase MENGGESER sesudah mengitari bola bergerak dengan y saja (x = 0,
  // sudut = 0), dan tanpa baris ini kaki hanya terangkat setinggi minimum --
  // terlalu rendah untuk benar-benar melangkah ke samping. Rentangnya sama
  // dengan rentang x karena satuannya sama: meter per langkah.
  motion = fmax(motion, ramp_progress(y_move, MIN_FB_STEP, max_fb_step_));

  double ratio = z_move_min_ratio_;
  if (ratio < 0.0)  ratio = 0.0;
  if (ratio > 1.0)  ratio = 1.0;

  current_walking_param_.z_move_amplitude =
      z_move_max_ * (ratio + (1.0 - ratio) * motion);

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set, cannot set walking parameters");
    return;
  }
  set_walking_param_pub_->publish(current_walking_param_);

  current_x_move_ = x_move;
  current_r_angle_ = rotation_angle;
}

// =========================================================================
//  Preset walking demo: JALAN / MENCARI CELAH / PUTAR KIRI-KANAN /
//                       GESER KIRI-KANAN
// =========================================================================
//
// Preset disimpan sebagai lapisan tipis "nama field -> angka" dan ditimpakan
// ke atas parameter DASAR. Field yang tidak disebut berkas yaml karena itu
// mewarisi nilai aslinya, bukan nol.
//
// Dasarnya berlapis: preset JALAN ditimpakan ke atas isi param.yaml saat demo
// mulai, dan keempat preset gerakan ditimpakan ke atas hasilnya. Jadi field
// yang cuma ditulis di bagian jalan (mis. pelvis_offset) tetap ikut terbawa
// waktu robot mengitari bola atau menggeser.

namespace
{

typedef op3_walking_module_msgs::msg::WalkingParam WalkingParamMsg;

// Satu-satunya daftar nama field yang ada. Dipakai untuk MENIMPA sekaligus
// untuk MEMERIKSA salah ketik, supaya keduanya tidak bisa berbeda isi.
bool setWalkingParamField(WalkingParamMsg &param, const std::string &name, double value)
{
  static const std::pair<const char *, float WalkingParamMsg::*> float_fields[] = {
    { "init_x_offset",                &WalkingParamMsg::init_x_offset },
    { "init_y_offset",                &WalkingParamMsg::init_y_offset },
    { "init_z_offset",                &WalkingParamMsg::init_z_offset },
    { "init_roll_offset",             &WalkingParamMsg::init_roll_offset },
    { "init_pitch_offset",            &WalkingParamMsg::init_pitch_offset },
    { "init_yaw_offset",              &WalkingParamMsg::init_yaw_offset },
    { "period_time",                  &WalkingParamMsg::period_time },
    { "dsp_ratio",                    &WalkingParamMsg::dsp_ratio },
    { "step_fb_ratio",                &WalkingParamMsg::step_fb_ratio },
    { "soft_start_time",              &WalkingParamMsg::soft_start_time },
    { "x_move_amplitude",             &WalkingParamMsg::x_move_amplitude },
    { "y_move_amplitude",             &WalkingParamMsg::y_move_amplitude },
    { "z_move_amplitude",             &WalkingParamMsg::z_move_amplitude },
    { "angle_move_amplitude",         &WalkingParamMsg::angle_move_amplitude },
    { "angle_move_amplitude_turn_in", &WalkingParamMsg::angle_move_amplitude_turn_in },
    { "balance_hip_roll_gain",        &WalkingParamMsg::balance_hip_roll_gain },
    { "balance_knee_gain",            &WalkingParamMsg::balance_knee_gain },
    { "balance_ankle_roll_gain",      &WalkingParamMsg::balance_ankle_roll_gain },
    { "balance_ankle_pitch_gain",     &WalkingParamMsg::balance_ankle_pitch_gain },
    { "y_swap_amplitude",             &WalkingParamMsg::y_swap_amplitude },
    { "z_swap_amplitude",             &WalkingParamMsg::z_swap_amplitude },
    { "arm_swing_gain",               &WalkingParamMsg::arm_swing_gain },
    { "pelvis_offset",                &WalkingParamMsg::pelvis_offset },
    { "hip_pitch_offset",             &WalkingParamMsg::hip_pitch_offset },
  };
  static const std::pair<const char *, bool WalkingParamMsg::*> bool_fields[] = {
    { "move_aim_on",    &WalkingParamMsg::move_aim_on },
    { "balance_enable", &WalkingParamMsg::balance_enable },
  };
  static const std::pair<const char *, int32_t WalkingParamMsg::*> int_fields[] = {
    { "p_gain", &WalkingParamMsg::p_gain },
    { "i_gain", &WalkingParamMsg::i_gain },
    { "d_gain", &WalkingParamMsg::d_gain },
  };

  for (const auto &field : float_fields)
  {
    if (name == field.first)
    {
      param.*(field.second) = (float) value;
      return true;
    }
  }
  for (const auto &field : bool_fields)
  {
    if (name == field.first)
    {
      param.*(field.second) = (value > 0.5);
      return true;
    }
  }
  for (const auto &field : int_fields)
  {
    if (name == field.first)
    {
      param.*(field.second) = (int32_t) llround(value);
      return true;
    }
  }

  return false;
}

bool isWalkingParamField(const std::string &name)
{
  WalkingParamMsg scratch;
  return setWalkingParamField(scratch, name, 0.0);
}

// Kebalikan setWalkingParamField(): baca satu field berdasarkan namanya.
// Dipakai verifyPresetApplied() untuk membandingkan angka yang DIMINTA preset
// dengan angka yang benar-benar tersimpan di walking_module.
//
// Daftar namanya sengaja ditulis ulang di sini, bukan diturunkan dari setter:
// pointer-to-member C++ tidak bisa dibaca balik dari nilai. Kalau suatu saat
// WalkingParam bertambah field, TAMBAHKAN DI KEDUA TEMPAT -- nama yang cuma
// ada di setter akan lolos pemeriksaan ini tanpa suara.
bool getWalkingParamField(const WalkingParamMsg &param, const std::string &name, double &out)
{
  static const std::pair<const char *, float WalkingParamMsg::*> float_fields[] = {
    { "init_x_offset",                &WalkingParamMsg::init_x_offset },
    { "init_y_offset",                &WalkingParamMsg::init_y_offset },
    { "init_z_offset",                &WalkingParamMsg::init_z_offset },
    { "init_roll_offset",             &WalkingParamMsg::init_roll_offset },
    { "init_pitch_offset",            &WalkingParamMsg::init_pitch_offset },
    { "init_yaw_offset",              &WalkingParamMsg::init_yaw_offset },
    { "period_time",                  &WalkingParamMsg::period_time },
    { "dsp_ratio",                    &WalkingParamMsg::dsp_ratio },
    { "step_fb_ratio",                &WalkingParamMsg::step_fb_ratio },
    { "soft_start_time",              &WalkingParamMsg::soft_start_time },
    { "x_move_amplitude",             &WalkingParamMsg::x_move_amplitude },
    { "y_move_amplitude",             &WalkingParamMsg::y_move_amplitude },
    { "z_move_amplitude",             &WalkingParamMsg::z_move_amplitude },
    { "angle_move_amplitude",         &WalkingParamMsg::angle_move_amplitude },
    { "angle_move_amplitude_turn_in", &WalkingParamMsg::angle_move_amplitude_turn_in },
    { "balance_hip_roll_gain",        &WalkingParamMsg::balance_hip_roll_gain },
    { "balance_knee_gain",            &WalkingParamMsg::balance_knee_gain },
    { "balance_ankle_roll_gain",      &WalkingParamMsg::balance_ankle_roll_gain },
    { "balance_ankle_pitch_gain",     &WalkingParamMsg::balance_ankle_pitch_gain },
    { "y_swap_amplitude",             &WalkingParamMsg::y_swap_amplitude },
    { "z_swap_amplitude",             &WalkingParamMsg::z_swap_amplitude },
    { "arm_swing_gain",               &WalkingParamMsg::arm_swing_gain },
    { "pelvis_offset",                &WalkingParamMsg::pelvis_offset },
    { "hip_pitch_offset",             &WalkingParamMsg::hip_pitch_offset },
  };
  for (const auto &field : float_fields)
  {
    if (name == field.first)
    {
      out = param.*(field.second);
      return true;
    }
  }
  if (name == "move_aim_on")     { out = param.move_aim_on ? 1.0 : 0.0; return true; }
  if (name == "balance_enable")  { out = param.balance_enable ? 1.0 : 0.0; return true; }
  if (name == "p_gain")          { out = param.p_gain; return true; }
  if (name == "i_gain")          { out = param.i_gain; return true; }
  if (name == "d_gain")          { out = param.d_gain; return true; }
  return false;
}

}  // namespace

void BallFollower::applyPresetOverlay(op3_walking_module_msgs::msg::WalkingParam &param,
                                      const std::map<std::string, double> &fields) const
{
  for (const auto &entry : fields)
    setWalkingParamField(param, entry.first, entry.second);
}

double BallFollower::presetField(const WalkPreset &preset, const char *name, double fallback)
{
  const auto found = preset.fields.find(name);
  return (found == preset.fields.end()) ? fallback : found->second;
}

// Baca balik parameter dari walking_module dan bandingkan field demi field
// dengan preset JALAN. Yang tidak cocok dicetak beserta kedua angkanya.
//
// Ini menjawab pertanyaan "apakah jalannya sudah sama dengan JALAN MENUJU MEKKAH?"
// dengan bukti, bukan dengan perasaan: kalau ada yang meleset, namanya muncul
// di log; kalau tidak ada, tercetak satu baris bahwa semuanya cocok.
//
// Amplitudo gerak dilewati dengan sengaja -- x/y/z/sudut memang dikemudikan
// ramp demo dan pasti berbeda dari preset. Sisanya (init pose, timing, bentuk
// gait, balance, gain) harus sama persis.
void BallFollower::verifyPresetApplied()
{
  if (preset_walk_.loaded == false)
    return;

  // Sekali saja per demo. getWalkingParam() itu panggilan service yang
  // MEMBLOKIR loop demo (termasuk callback IMU yang dipakai penyelarasan yaw),
  // dan startFollowing() dijalankan lagi tiap kali robot kembali mengejar
  // bola. Yang mau diketahui cuma "apakah preset benar-benar masuk", dan itu
  // sudah terjawab di kali pertama.
  if (preset_verified_ == true)
    return;
  preset_verified_ = true;

  static const char *skip[] = { "x_move_amplitude", "y_move_amplitude",
                                "z_move_amplitude", "angle_move_amplitude",
                                "angle_move_amplitude_turn_in" };

  std::string mismatch;
  int checked = 0;

  // Terbitnya preset itu asinkron: walking_module baru menyimpannya saat
  // callback langganannya jalan. Kalau service dibaca tepat sebelum itu, yang
  // pulang masih angka lama dan laporannya jadi alarm palsu. Karena itu dicoba
  // sampai tiga kali; sekali cocok langsung berhenti.
  for (int attempt = 0; attempt < 3; attempt++)
  {
    if (attempt > 0)
      rclcpp::sleep_for(std::chrono::milliseconds(30));

    // Bacaan segar dari walking_module, bukan salinan lokal.
    getWalkingParam();

    mismatch.clear();
    checked = 0;

    for (const auto &entry : preset_walk_.fields)
    {
      bool skip_this = false;
      for (const char *name : skip)
        if (entry.first == name)
          skip_this = true;
      if (skip_this == true)
        continue;

      double live = 0.0;
      if (getWalkingParamField(current_walking_param_, entry.first, live) == false)
        continue;

      checked++;

      // Toleransi longgar: WalkingParam membawa float 32 bit, jadi 0,77 pulang
      // sebagai 0,7699999809. Yang dicari ketidakcocokan NYATA, bukan pembulatan.
      if (fabs(live - entry.second) <= 1e-4)
        continue;

      char buf[160];
      snprintf(buf, sizeof(buf), "%s%s: preset %.4f, robot %.4f",
               mismatch.empty() ? "" : "; ", entry.first.c_str(), entry.second, live);
      mismatch += buf;
    }

    if (mismatch.empty() == true)
      break;
  }

  if (mismatch.empty() == true)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Preset JALAN diperiksa ulang ke walking_module: %d field COCOK semua.",
                checked);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                 "Preset JALAN TIDAK COCOK di robot (%d field diperiksa) -- %s",
                 checked, mismatch.c_str());
  }
}

bool BallFollower::loadWalkPresets(const std::string &path)
{
  // Membaca SATU bagian ke dalam sebuah preset. required = bagian yang tanpanya
  // fitur mengitari bola tidak boleh jalan; bagian opsional yang tidak ada
  // cukup dilaporkan sebagai INFO, dan pemakainya jatuh ke jalur lama.
  auto read_section = [&](const YAML::Node &doc, WalkPreset &preset, bool required) -> bool
  {
    preset.loaded = false;
    preset.fields.clear();

    const YAML::Node node = doc[preset.section];
    if (node.IsDefined() == false || node.IsMap() == false)
    {
      if (required == true)
      {
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "Preset walking: bagian '%s' tidak ada di %s (atau bukan daftar "
                     "field). Fitur mengitari bola MATI; robot memutar di tempat "
                     "seperti dulu.",
                     preset.section.c_str(), path.c_str());
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                    "Preset walking: bagian opsional '%s' tidak ada di %s; "
                    "%s memakai jalur lama.",
                    preset.section.c_str(), path.c_str(), preset.label.c_str());
      }
      return false;
    }

    for (auto it = node.begin(); it != node.end(); ++it)
    {
      const std::string key = it->first.as<std::string>();

      if (isWalkingParamField(key) == false)
      {
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "Preset '%s': field '%s' TIDAK DIKENAL (salah ketik?). Dilewati -- "
                     "namanya harus sama persis dengan field WalkingParam.msg.",
                     preset.section.c_str(), key.c_str());
        continue;
      }

      double value = 0.0;
      try
      {
        value = it->second.as<double>();
      }
      catch (const std::exception &)
      {
        // true/false juga sah: move_aim_on dan balance_enable memang bool.
        try
        {
          value = it->second.as<bool>() ? 1.0 : 0.0;
        }
        catch (const std::exception &)
        {
          RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                       "Preset '%s': nilai field '%s' bukan angka maupun true/false. Dilewati.",
                       preset.section.c_str(), key.c_str());
          continue;
        }
      }

      preset.fields[key] = value;
    }

    if (preset.fields.empty() == true)
    {
      RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                   "Preset '%s' di %s kosong.", preset.section.c_str(), path.c_str());
      return false;
    }

    preset.loaded = true;
    return true;
  };

  preset_walk_.label         = "JALAN";        preset_walk_.section        = "jalan";
  preset_search_.label       = "MENCARI CELAH"; preset_search_.section     = "mencari_celah";
  preset_orbit_left_.label   = "PUTAR KIRI";   preset_orbit_left_.section  = "putar_kiri";
  preset_orbit_right_.label  = "PUTAR KANAN";  preset_orbit_right_.section = "putar_kanan";
  preset_shift_left_.label   = "GESER KIRI";   preset_shift_left_.section  = "geser_kiri";
  preset_shift_right_.label  = "GESER KANAN";  preset_shift_right_.section = "geser_kanan";

  bool orbit_ok = false;

  try
  {
    const YAML::Node doc = YAML::LoadFile(path);

    // Bagian jalan dan geser OPSIONAL: tanpa keduanya demo tetap jalan persis
    // seperti sebelum preset ini ada. Cuma pasangan putar yang wajib, karena
    // itulah satu-satunya cara robot mengitari bola.
    read_section(doc, preset_walk_, false);
    // Juga opsional: tanpa bagian ini, bola hilang berarti berjalan di tempat
    // persis seperti sebelum MENCARI CELAH ada.
    read_section(doc, preset_search_, false);
    const bool left_ok  = read_section(doc, preset_orbit_left_, true);
    const bool right_ok = read_section(doc, preset_orbit_right_, true);
    orbit_ok = (left_ok == true && right_ok == true);

    // Dua-duanya harus ada. Satu preset geser saja berarti robot hanya bisa
    // menyesuaikan ke satu sisi, dan itu lebih buruk daripada jalur lama.
    const bool shift_left_ok  = read_section(doc, preset_shift_left_, false);
    const bool shift_right_ok = read_section(doc, preset_shift_right_, false);
    if (shift_left_ok != shift_right_ok)
    {
      RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                   "Preset menggeser cuma ada SATU sisi (geser_kiri %s, geser_kanan %s) "
                   "di %s. Keduanya dimatikan; fase menggeser memakai jalur lama.",
                   shift_left_ok ? "ada" : "tidak ada",
                   shift_right_ok ? "ada" : "tidak ada", path.c_str());
      preset_shift_left_.loaded = false;
      preset_shift_right_.loaded = false;
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                 "Preset walking gagal dibaca dari %s (%s). Fitur mengitari bola MATI; "
                 "robot memutar di tempat seperti sebelum fitur ini ada.",
                 path.c_str(), e.what());
    return false;
  }

  auto describe = [](const WalkPreset &preset) -> std::string
  {
    if (preset.loaded == false)
      return preset.label + " TIDAK ADA";

    // period_time boleh tidak ditulis (mewarisi param.yaml), dan mencetaknya
    // sebagai "0,00 s" cuma bikin operator mengira presetnya rusak.
    const double period = presetField(preset, "period_time", -1.0);
    char period_txt[32];
    if (period > 0.0)
      snprintf(period_txt, sizeof(period_txt), "%.2f s", period);
    else
      snprintf(period_txt, sizeof(period_txt), "warisan param.yaml");

    char buf[224];
    snprintf(buf, sizeof(buf),
             "%s %zu field (turn_in %+.3f, sudut %+.3f, x %+.1f mm, y %+.1f mm, "
             "period %s)",
             preset.label.c_str(), preset.fields.size(),
             presetField(preset, "angle_move_amplitude_turn_in", 0.0),
             presetField(preset, "angle_move_amplitude", 0.0),
             presetField(preset, "x_move_amplitude", 0.0) * 1000.0,
             presetField(preset, "y_move_amplitude", 0.0) * 1000.0,
             period_txt);
    return std::string(buf);
  };

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Preset walking dimuat dari %s:\n  %s\n  %s\n  %s\n  %s\n  %s\n  %s\n"
              "  Field yang tidak disebut mewarisi preset JALAN; yang tidak disebut "
              "JALAN mewarisi param.yaml.",
              path.c_str(),
              describe(preset_walk_).c_str(),
              describe(preset_search_).c_str(),
              describe(preset_orbit_left_).c_str(),
              describe(preset_orbit_right_).c_str(),
              describe(preset_shift_left_).c_str(),
              describe(preset_shift_right_).c_str());

  return orbit_ok;
}

bool BallFollower::chooseOrbitLeft(double yaw_deviation_deg) const
{
  // Aturan dari operator, dipakai apa adanya:
  //
  //     yaw POSITIF  -> preset PUTAR KIRI
  //     yaw NEGATIF  -> preset PUTAR KANAN
  //
  // yaw_deviation_deg = yaw sekarang - yaw target. Dengan target 0 (bawaan)
  // angkanya persis bacaan yaw robot, jadi aturannya terbaca harfiah.
  //
  // Pemetaan ini TIDAK pernah ditukar sendiri di tengah demo. Sebelumnya gyro
  // boleh membaliknya pada putaran pertama, dan justru itu yang membuat preset
  // yang terpakai tidak bisa ditebak dari bacaan yaw. Kalau di robot ternyata
  // terbalik, yang membalik operator: kick_orbit_left_yaw_sign:=-1.0.
  return ((yaw_deviation_deg * orbit_left_yaw_sign_) > 0.0);
}

void BallFollower::publishPreset(const WalkPreset &preset)
{
  if (node_ == nullptr || set_walking_param_pub_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                 "Node is not set, cannot set walking parameters");
    return;
  }

  // Jaring pengaman: kalau entah bagaimana preset terbit sebelum salinan dasar
  // dibuat, salinannya dibuat SEKARANG -- sebelum current_walking_param_
  // ditimpa. Tanpa ini isi param.yaml hilang untuk sisa run.
  if (base_walking_param_saved_ == false)
  {
    base_walking_param_ = current_walking_param_;
    base_walking_param_saved_ = true;
  }

  op3_walking_module_msgs::msg::WalkingParam param = base_walking_param_;
  applyPresetOverlay(param, preset.fields);

  current_walking_param_ = param;
  set_walking_param_pub_->publish(param);

  current_x_move_ = param.x_move_amplitude;
  current_r_angle_ = param.angle_move_amplitude;

  // Periode gait IKUT presetnya. Dua hal bergantung padanya dan dua-duanya
  // salah kalau ia tertinggal di angka param.yaml:
  //
  //   1. panjang blok penapis yaw. Blok harus SAMA PANJANG dengan satu periode
  //      gait supaya goyangan badan seirama langkah benar-benar ternolkan;
  //      blok yang kepanjangan cuma menambah waktu tunggu.
  //   2. lamanya satu tick ramp di calcFootstep() (seperempat periode).
  //
  // Preset putar berjalan 0,57 s dan geser 0,55 s, sedangkan param.yaml 0,8 s:
  // memakai 0,8 di situ berarti tiap fase diam menunggu 1,8 s padahal 1,3 s
  // sudah cukup -- tiga kali per siklus tendangan.
  if (param.period_time > 0.1)
    curr_period_time_ = param.period_time;
}

void BallFollower::publishOrbitParam(bool use_left)
{
  publishPreset(use_left ? preset_orbit_left_ : preset_orbit_right_);
  orbit_using_left_ = use_left;
}

bool BallFollower::publishPoseReference()
{
  if (pose_reference_enable_ == false || pose_reference_saved_ == false)
    return false;
  if (node_ == nullptr || set_walking_param_pub_ == nullptr)
    return false;

  // Diterbitkan UTUH, bukan cuma ketujuh offset pose. Field lain (periode,
  // dsp, gain) juga bagian dari keadaan param.yaml, dan preset JALAN yang
  // terbit sesudah ini akan menimpanya lagi berdasarkan angka yang sama
  // seperti pada demo pertama. Mengirim sebagian saja membuat field yang tidak
  // disebut preset mewarisi sisa preset LAMA.
  set_walking_param_pub_->publish(pose_reference_param_);

  // current_walking_param_ ikut disamakan supaya getWalkingParam() berikutnya
  // tidak berlomba dengan pesan yang baru saja dikirim.
  current_walking_param_ = pose_reference_param_;
  if (pose_reference_param_.period_time > 0.1)
    curr_period_time_ = pose_reference_param_.period_time;

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Acuan pose dikembalikan ke walking_module sebelum modulnya "
              "dinyalakan lagi (init x %+.4f pitch %+.3f hip_pitch %+.4f). Tanpa ini "
              "bias pose akan direkam terhadap preset terakhir, dan kalau preset itu "
              "JALAN selisihnya nol sehingga robot terkunci di pose page 2.",
              pose_reference_param_.init_x_offset, pose_reference_param_.init_pitch_offset,
              pose_reference_param_.hip_pitch_offset);
  return true;
}

void BallFollower::restoreBaseWalkingParam()
{
  if (base_walking_param_saved_ == false)
    return;

  current_walking_param_ = base_walking_param_;

  // Periode gait ikut kembali, dengan alasan yang sama seperti di
  // publishPreset(): ia menentukan panjang blok penapis yaw dan lamanya satu
  // tick ramp, jadi ia harus selalu menggambarkan gait yang SEDANG tertulis di
  // walking_module.
  if (base_walking_param_.period_time > 0.1)
    curr_period_time_ = base_walking_param_.period_time;
}

int BallFollower::nearestKickFoot() const
{
  const double servo_deg = headPanServoDeg();
  const double right_center = 0.5 * (kick_pan_right_min_deg_ + kick_pan_right_max_deg_);
  const double left_center = 0.5 * (kick_pan_left_min_deg_ + kick_pan_left_max_deg_);

  return (fabs(servo_deg - right_center) <= fabs(servo_deg - left_center)) ? OnRight : OnLeft;
}

double BallFollower::kickWindowCenterDeg(int foot) const
{
  if (foot == OnRight)
    return 0.5 * (kick_pan_right_min_deg_ + kick_pan_right_max_deg_);

  return 0.5 * (kick_pan_left_min_deg_ + kick_pan_left_max_deg_);
}

// =========================================================================
//  Penyelarasan yaw sebelum menendang
// =========================================================================
//
// Dipanggil dari SoccerDemo::process(), satu langkah per tick (~30 Hz).
// Rangkaiannya:
//
//   startYawAlign()   sekali, saat bola baru masuk jangkauan tendang
//        v            -> true : mulai memutar
//   processYawAlign() tiap tick
//        |            -> false: masih memutar atau menunggu gait reda
//        v            -> true : selesai, pemanggil boleh menendang
//      (tendang)
//
// cancelYawAlign() membatalkan dari luar: tombol STOP, robot jatuh, atau demo
// dimatikan.

void BallFollower::setBodyYaw(double yaw_deg)
{
  if (!std::isfinite(yaw_deg))
    return;
  // Tidak ada kunci di sini dan memang tidak perlu: callback IMU dan process()
  // sama-sama dijalankan utas utama demo_node (process() lalu spin_some()).
  body_yaw_deg_ = yaw_deg;
  body_yaw_stamp_ = std::chrono::steady_clock::now();
  body_yaw_seen_ = true;

  // Sampel disimpan DI SINI, tiap pesan IMU, bukan tiap tick demo: tick demo
  // 30 Hz dan tidak selaras dengan irama langkah, jadi menyimpannya di sana
  // justru menambah aliasing yang mau dibuang penapis ini.
  //
  // turned_deg_ ikut disimpan pada stempel waktu yang sama supaya kedua deret
  // bisa digariskan atas jendela yang persis sama. Keduanya diperbarui dari
  // satu callback IMU yang sama (setBodyGyroZ lalu setBodyYaw), jadi selisih
  // urutannya paling banyak satu sampel.
  YawSample sample;
  sample.stamp = body_yaw_stamp_;
  sample.yaw_deg = yaw_deg;
  sample.turned_deg = turned_deg_;
  yaw_filter_buf_.push_back(sample);

  // Dibuang dari depan. Disimpan TIGA blok, bukan dua: taksirannya butuh dua
  // blok penuh, dan satu blok cadangan menjaga blok yang lebih tua tetap utuh
  // pada tick tempat batasnya baru saja bergeser.
  const double keep_sec = fmax(3.0 * yawFilterBlockSec(), 1.0);
  while (yaw_filter_buf_.size() > 1 &&
         std::chrono::duration<double>(body_yaw_stamp_ -
                                       yaw_filter_buf_.front().stamp).count() > keep_sec)
  {
    yaw_filter_buf_.pop_front();
  }
}

void BallFollower::resetYawFilter()
{
  // Sampel lama tidak dibuang, cuma tidak diikutkan lagi: dengan begitu
  // pemanggil boleh menolkan jendela sesering apa pun tanpa risiko penapisnya
  // kosong pada tick yang sama.
  yaw_filter_from_ = std::chrono::steady_clock::now();
}

double BallFollower::yawFilterBlockSec() const
{
  if (yaw_filter_sec_ > 0.0)
    return yaw_filter_sec_;

  // Negatif = ikut periode gait yang SEDANG hidup. Itu yang benar: preset
  // PUTAR KIRI berjalan 0,71 detik dan PUTAR KANAN 0,70 detik, dan blok yang
  // panjangnya bukan periode meninggalkan sisa ayunan.
  return (curr_period_time_ > 0.1) ? curr_period_time_ : 0.8;
}

double BallFollower::settleWaitSec(double requested_sec) const
{
  if (yaw_filter_sec_ == 0.0)
    return requested_sec;

  // Dua blok, plus sedikit kelonggaran supaya blok yang lebih tua benar-benar
  // terisi dan bukan pas-pasan di ambang jumlah sampel minimum.
  return fmax(requested_sec, 2.0 * yawFilterBlockSec() + 0.2);
}

bool BallFollower::estimateYawWindow(bool want_turned, double &value_at_now) const
{
  if (yaw_filter_sec_ == 0.0)
    return false;

  const double block = yawFilterBlockSec();
  const auto now = std::chrono::steady_clock::now();
  const auto edge_new = now - secondsToSteady(block);
  const auto edge_old = now - secondsToSteady(2.0 * block);

  // Deret yaw dipulihkan terhadap sampel TERBARU supaya lompatan +179 ->
  // -179 tidak masuk hitungan sebagai putaran 358 derajat. normalizeAngleDeg
  // mengembalikan tiap sampel ke dalam +-180 dari acuan itu.
  double reference = 0.0;
  bool reference_set = false;
  for (auto it = yaw_filter_buf_.rbegin(); it != yaw_filter_buf_.rend(); ++it)
  {
    if (it->stamp >= edge_new && it->stamp >= yaw_filter_from_)
    {
      reference = want_turned ? it->turned_deg : it->yaw_deg;
      reference_set = true;
      break;
    }
  }
  if (reference_set == false)
    return false;

  int n_new = 0, n_old = 0;
  double sum_new = 0.0, sum_old = 0.0;
  double oldest_age = 0.0;

  for (const YawSample &sample : yaw_filter_buf_)
  {
    if (sample.stamp < edge_old || sample.stamp < yaw_filter_from_)
      continue;

    oldest_age = fmax(oldest_age,
                      std::chrono::duration<double>(now - sample.stamp).count());

    double y = want_turned ? sample.turned_deg : sample.yaw_deg;
    if (want_turned == false)
      y = reference + normalizeAngleDeg(y - reference);

    if (sample.stamp >= edge_new)
    {
      n_new += 1;
      sum_new += y;
    }
    else
    {
      n_old += 1;
      sum_old += y;
    }
  }

  // Kedua blok harus benar-benar terisi, dalam JUMLAH sampel maupun dalam
  // RENTANG WAKTU. Blok yang cuma terisi sepotong menyisakan ayunan, dan
  // ayunan itu langsung dikalikan 1,5 oleh ekstrapolasi di bawah -- lebih
  // buruk daripada memakai nilai sesaat. Jumlah sampel saja tidak cukup jadi
  // ukuran: empat sampel IMU bisa muat dalam 0,04 detik.
  if (n_new < 4 || n_old < 4 || oldest_age < 1.8 * block)
    return false;

  const double mean_new = sum_new / (double) n_new;
  const double mean_old = sum_old / (double) n_old;

  // Rata-rata blok terbaru berlaku untuk TENGAH blok itu, yaitu setengah blok
  // yang lalu. Selisih kedua rata-rata memberi lajunya (juga sudah bersih dari
  // ayunan), jadi menambahkan setengah selisih memindahkan taksirannya ke
  // SEKARANG tanpa membawa kembali ayunan yang barusan dibuang.
  const double value = mean_new + 0.5 * (mean_new - mean_old);

  value_at_now = want_turned ? value : normalizeAngleDeg(value);
  return true;
}

double BallFollower::filteredBodyYawDeg() const
{
  double value = 0.0;
  return estimateYawWindow(false, value) ? value : body_yaw_deg_;
}

double BallFollower::filteredTurnedDeg() const
{
  double value = 0.0;
  return estimateYawWindow(true, value) ? value : turned_deg_;
}

void BallFollower::setBodyGyroZ(double gyro_z_rad_per_sec)
{
  if (!std::isfinite(gyro_z_rad_per_sec))
    return;
  const auto now = std::chrono::steady_clock::now();

  // Integrasi dilakukan DI SINI, bukan di tick demo, supaya tiap sampel gyro
  // terpakai. Tick demo jalan 30 Hz dan tidak selaras dengan laju IMU; kalau
  // integrasinya di sana, sebagian putaran hilang tak terhitung.
  if (gyro_integrating_ == true && gyro_seen_ == true)
  {
    const double dt = std::chrono::duration<double>(now - gyro_stamp_).count();

    // Lompatan waktu yang tidak masuk akal (mis. pesan tertahan) dibuang
    // daripada menyuntikkan putaran palsu ke hasil integrasi.
    if (dt > 0.0 && dt < 0.5)
      turned_deg_ += gyro_scale_ * gyro_sign_ * gyro_z_rad_per_sec * 180.0 / M_PI * dt;
  }

  body_gyro_z_ = gyro_z_rad_per_sec;
  gyro_stamp_ = now;
  gyro_seen_ = true;
}

void BallFollower::setMagStatus(int mag_status)
{
  mag_status_ = mag_status;
}

bool BallFollower::isBodyYawUsable() const
{
  if (body_yaw_seen_ == false)
    return false;

  // mag_status 2 = OK, 3 = sedang kalibrasi hard-iron -- pada keduanya
  // open_cr_module memang menerbitkan hasil pengukuran. Pada 0 (papan tanpa
  // magnetometer) dan 1 (belum settle) ia MEMAKSA yaw = 0, jadi "sudah lurus"
  // yang terbaca cuma nilai bawaan dan robot akan menendang ke arah mana pun
  // sambil mengaku sudah lurus. -1 = topiknya belum pernah terdengar; itu
  // dibiarkan lewat supaya fitur ini tetap jalan pada firmware yang tidak
  // menerbitkan mag_status sama sekali.
  if (mag_status_ >= 0 && mag_status_ < 2)
    return false;

  const auto age = std::chrono::steady_clock::now() - body_yaw_stamp_;
  return age <= secondsToSteady(yaw_max_age_sec_);
}

void BallFollower::pauseFollowingForTurn()
{
  // SENGAJA tidak memanggil setWalkingCommand("stop"). Dulu soccer_demo
  // memanggil stopFollowing() (yang mengirim "stop") lalu 39 mikrodetik
  // kemudian startInPlaceTurn() mengirim "start". Dua pesan itu tiba di
  // walking_module dalam satu siklus kontrol 8 ms yang sama, dan siapa yang
  // menang jadi undian.
  //
  // Terukur di robot 2026-08-28, dua penyelarasan di run yang sama:
  //   yaw -26,5 -> -14,9  : yaw berayun belasan derajat, gait JALAN
  //   yaw -28,9 -> -28,0  : yaw beku di pita +-1,1 deg selama 12 detik penuh
  // Yang kedua bukan kurang torsi -- gait-nya memang tidak jalan sama sekali.
  //
  // Sekarang gait dibiarkan terus berjalan dari fase mengejar bola; yang
  // dihentikan hanya pelacakannya.
  on_tracking_ = false;
  count_to_kick_ = 0;
}

void BallFollower::brakeBeforeTurn()
{
  // Hentikan gait DULU, lalu tunggu. Robot yang masih melangkah maju saat
  // mulai memutar akan menabrak bola yang tepat ada di depannya -- terlihat
  // langsung di lapangan 2026-08-28.
  //
  // "stop" dikirim di sini dan "start" baru dikirim yaw_brake_sec_ kemudian
  // oleh startInPlaceTurn(). Jarak WAKTU itu yang penting: sebelumnya
  // keduanya dikirim berselang 39 mikrodetik, tiba di walking_module dalam
  // satu siklus kontrol 8 ms yang sama, dan siapa yang menang jadi undian --
  // itulah yang dulu membuat gait mati diam-diam saat memutar.
  setWalkingParam(0.0, 0.0, 0.0);
  setWalkingCommand("stop");

  // Jendela penapis dinolkan: mulai detik ini yang boleh ikut dihitung hanya
  // sampel dari robot yang SEDANG BERHENTI. Tanpa ini bacaan "sudah berhenti di
  // depan bola" masih membawa goyangan dari langkah-langkah terakhir.
  resetYawFilter();
}

double BallFollower::enterBrakePhase(double yaw_now_deg)
{
  const auto now = std::chrono::steady_clock::now();
  const double brake_sec = settleWaitSec(yaw_brake_sec_);

  yaw_align_state_ = YawAlignBraking;
  yaw_brake_until_ = now + secondsToSteady(brake_sec);
  yaw_brake_deadline_ = now + secondsToSteady(fmax(yaw_brake_max_sec_, brake_sec + 1.0));
  // Sampel pertama diambil 0,25 detik lagi, BUKAN di ujung lama minimum:
  // lihat catatan di cabang YawAlignBraking processYawAlign().
  yaw_brake_sample_at_ = now + secondsToSteady(0.25);
  yaw_brake_last_yaw_ = yaw_now_deg;
  // Belum ada sampel yang sahih. Dipasang di atas ambang supaya fase ini tidak
  // bisa lulus membawa sisa bacaan dari fase mengerem sebelumnya.
  yaw_brake_moved_ = yaw_brake_still_deg_ + 1.0;
  yaw_in_window_count_ = 0;

  brakeBeforeTurn();

  return brake_sec;
}

void BallFollower::startInPlaceTurn()
{
  // Menyalakan gait untuk fase memutar. Kalau preset mengitari bola aktif,
  // yang terbit di sini cuma parameter DASAR beramplitudo nol; presetnya
  // sendiri terbit pada tick processYawAlign() berikutnya. Jaraknya satu tick
  // (~33 ms) dan walking_module memang menaikkan amplitudo bertahap, jadi
  // tidak ada hentakan.
  //
  // x dan y NOL. IN_PLACE_FB_STEP (-3 mm) yang dipakai
  // setWalkingCommand("start") sengaja tidak dipakai di sini: itu membuat robot
  // mundur pelan selama memutar, padahal yang sedang dijaga justru posisinya
  // terhadap bola yang sudah tepat di depan kaki.
  setWalkingParam(0.0, 0.0, 0.0);

  // publishWalkingCommand(), BUKAN setWalkingCommand("start"): yang terakhir
  // memanggil getWalkingParam(), sebuah panggilan service yang memblokir
  // utas -- utas yang sama yang menjalankan callback IMU pemasok yaw.
  // current_walking_param_ sendiri sudah terisi sejak startFollowing().
  publishWalkingCommand("start");
}

bool BallFollower::startYawAlign()
{
  yaw_align_state_ = YawAlignOff;
  yaw_in_window_count_ = 0;

  // Dinolkan di PALING ATAS, sebelum semua jalan keluar dini: kalau tidak, run
  // yang penyelarasannya dilewati akan membawa keputusan kaki milik tendangan
  // SEBELUMNYA.
  foot_from_reposition_ = false;
  last_settle_sec_ = 0.0;

  const double yaw_now = filteredBodyYawDeg();
  const double error = normalizeAngleDeg(yaw_target_deg_ - yaw_now);

  // Masuk fase MENGEREM dulu, bukan langsung memutar. Semua penghitung waktu
  // putaran (tenggat, macet, cek tanda) sengaja BELUM disetel di sini; mereka
  // baru mulai saat putaran benar-benar dimulai, supaya waktu mengerem tidak
  // ikut termakan tenggat.
  yaw_recheck_left_ = yaw_recheck_max_;
  reposition_recheck_left_ = reposition_recheck_max_;
  // Jatah fase MENGGESER dipasang ulang per percobaan tendangan, bukan per
  // masuknya fase itu -- kalau tidak, satu percobaan bisa menggeser berkali
  // kali dengan jatah yang selalu penuh.
  reposition_pass_count_ = 0;
  orbit_announced_ = -1;
  yaw_runaway_ref_error_ = fabs(error);
  yaw_align_ref_error_ = fabs(error);

  // Salinan parameter walking SEBELUM mengitari. Preset PUTAR menimpa seluruh
  // struct, jadi salinan ini satu-satunya jalan kembali ke parameter jalan yang
  // sedang dipakai. Diambil ulang tiap penyelarasan, supaya Apply dari Bascorro
  // Studio di tengah sesi ikut terbawa.
  base_walking_param_ = current_walking_param_;
  base_walking_param_saved_ = true;
  // Batas bawah putaran mulai dari nilai parameter tiap kali menyelaraskan,
  // BUKAN dari hasil eskalasi tendangan sebelumnya: torsi yang dibutuhkan
  // tergantung keadaan saat itu (digantung, lantai licin, baterai lemah),
  // jadi memulai lagi dari yang paling halus adalah pilihan yang lebih aman.
  yaw_turn_floor_deg_ = yaw_min_turn_deg_;
  // yaw_turn_sign_ dan yaw_sign_settled_ SENGAJA tidak disetel ulang di sini:
  // arah putaran itu sifat robot, bukan sifat satu tendangan. Sekali terukur,
  // dipakai seterusnya.

  pauseFollowingForTurn();
  const double brake_sec = enterBrakePhase(yaw_now);

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "BERHENTI dulu (%.1f s) lalu MEMUTAR sebelum menendang: "
              "yaw %+.1f -> target %+.1f deg (galat %+.1f, toleransi %.1f). %s",
              brake_sec, yaw_now, yaw_target_deg_, error,
              yaw_tolerance_deg_,
              (yaw_timeout_sec_ > 0.0)
                  ? "Batas waktu tidak membolehkan tendangan tanpa verifikasi."
                  : "TIDAK menendang sampai yaw benar-benar masuk toleransi.");

  return true;
}

bool BallFollower::finishYawTurn(const char *reason)
{
  gyro_integrating_ = false;

  // Lama fase reda dihitung SEBELUM parameter dasar dikembalikan.
  //
  // Panjang blok penapis mengikuti periode gait yang sedang tertulis, dan
  // sampel yang nanti dirata-rata di fase reda itu sisa goyangan gait PUTAR
  // (0,57 s), bukan gait jalan (0,8 s). Kalau dihitung sesudah restore, robot
  // menunggu 1,8 s untuk meredakan goyangan yang cuma butuh 1,3 s.
  const double settle_sec = settleWaitSec(yaw_settle_sec_);

  // Parameter dasar dikembalikan DULU. Kalau tidak, yang tertulis di
  // walking_module tetap preset PUTAR (period, init pose, balance-nya sendiri)
  // cuma dengan amplitudo nol -- dan langkah berikutnya berjalan dengan gait
  // mengitari, bukan gait mengejar bola.
  restoreBaseWalkingParam();

  // Amplitudo dinolkan SEBELUM "stop", urutan yang sama dengan stopFollowing():
  // walking_module mengecil dari amplitudo ke nol pada siklus berikutnya alih-
  // alih menyelesaikan langkah putar terakhirnya.
  setWalkingParam(0.0, 0.0, 0.0);
  publishWalkingCommand("stop");

  yaw_align_state_ = YawAlignSettling;
  yaw_settle_until_ = std::chrono::steady_clock::now() + secondsToSteady(settle_sec);
  last_settle_sec_ = settle_sec;
  yaw_in_window_count_ = 0;

  // Vonis magnetometer sesudah ini harus dihitung dari sampel robot yang sudah
  // BERHENTI saja, bukan dari sisa goyangan putaran barusan.
  resetYawFilter();

  RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
              "Putaran dihentikan (%s); menunggu %.1f s, lalu periksa yaw, jarak, dan kaki.",
              reason, settle_sec);

  // Belum selesai: fase reda diselesaikan tick berikutnya.
  return false;
}

bool BallFollower::processYawAlign(bool ball_visible, double ball_radius_px)
{
  if (yaw_align_state_ == YawAlignOff)
    return false;

  const auto now = std::chrono::steady_clock::now();

  if (yaw_align_enable_ && !isBodyYawUsable())
    return abortKickAlignment("yaw belum valid atau sudah basi");

  if (yaw_align_state_ == YawAlignBraking)
  {
    // Tahan amplitudo di nol sampai robot benar-benar berhenti. Tanpa fase
    // ini robot masih melangkah maju saat mulai memutar dan kakinya menabrak
    // bola yang tepat ada di depannya.
    setWalkingParam(0.0, 0.0, 0.0);

    // Uji "sudah diam" BERJALAN SEJAK DETIK PERTAMA mengerem, bukan baru
    // sesudah lama minimumnya lewat.
    //
    // Robot belum tentu benar-benar diam saat lama minimum habis. Terukur
    // 2026-08-28: selama mengerem 0,8 detik yaw masih bergeser -20,6 -> -16,2
    // (4,4 derajat). Kalau putaran dimulai dari angka yang masih bergerak,
    // galat awalnya salah sejak detik pertama. Jadi robot menunggu sampai dua
    // sampel berturut bedanya di bawah yaw_brake_still_deg_.
    //
    // Yang berubah: sampelnya diambil selama menunggu, bukan sesudahnya.
    // Versi lama mengambil sampel pertama tepat di ujung lama minimum, lalu
    // membandingkannya dengan yaw yang direkam saat robot masih BERJALAN --
    // bedanya praktis selalu di atas ambang, jadi robot selalu menunggu satu
    // putaran sampel tambahan (0,25-0,5 detik) hanya untuk membuang bacaan
    // yang memang tidak pernah bisa lulus. Dengan sampel yang sudah mengalir,
    // begitu lama minimum habis jawabannya sudah siap.
    //
    // Dibandingkan yang sudah DITAPIS. Dengan sampel sesaat, uji ini praktis
    // tidak berarti: terukur 2026-08-31 ia melaporkan yaw bergeser 45,3 derajat
    // dalam 0,25 detik pada robot yang sedang berhenti -- 180 derajat per
    // detik, mustahil. Yang terbaca itu goyangan badan dan derau magnetometer,
    // bukan putaran.
    if (now >= yaw_brake_sample_at_)
    {
      const double yaw_now = filteredBodyYawDeg();
      yaw_brake_moved_ = fabs(normalizeAngleDeg(yaw_now - yaw_brake_last_yaw_));
      yaw_brake_last_yaw_ = yaw_now;
      yaw_brake_sample_at_ = now + secondsToSteady(0.25);
    }

    if (now < yaw_brake_until_)
      return false;

    // Ada batas atas waktu supaya robot tidak menunggu selamanya kalau
    // lantainya bergetar.
    if (now < yaw_brake_deadline_ && yaw_brake_moved_ > yaw_brake_still_deg_)
    {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"), *log_clock_, 500,
                           "mengerem: yaw masih bergeser %.1f deg (batas diam %.1f), "
                           "menunggu robot benar-benar berhenti",
                           yaw_brake_moved_, yaw_brake_still_deg_);
      return false;
    }

    // Baru DI SINI putaran dimulai, dan baru di sini semua penghitung waktu
    // putaran dinyalakan -- supaya waktu mengerem tidak ikut termakan tenggat.
    // Magnetometer dibaca DI SINI, satu kali, saat robot sudah benar-benar
    // diam. Inilah satu-satunya tempat ia bisa dipercaya. Angkanya jadi
    // "berapa derajat harus diputar", dan sesudah ini magnetometer TIDAK
    // dilihat lagi sampai putaran selesai; yang mengendalikan gyro.
    const double yaw_at_rest = filteredBodyYawDeg();
    const double start_error = normalizeAngleDeg(yaw_target_deg_ - yaw_at_rest);

    turn_target_deg_ = start_error;
    turned_deg_ = 0.0;
    mag_before_deg_ = yaw_at_rest;

    if (kickHeadingReady(yaw_align_enable_, isBodyYawUsable(), start_error, yaw_tolerance_deg_))
      return finishYawTurn("yaw sudah sesuai; periksa ulang jarak dan posisi kaki");

    // turned_deg_ baru saja dinolkan, jadi sampel lama di jendela penapis
    // sekarang menunjuk putaran yang SUDAH SELESAI. Kalau tidak dinolkan juga,
    // garis lurus yang dicocokkan menariknya ke nilai lama dan putaran ini
    // dianggap sudah berjalan jauh sejak tick pertama.
    resetYawFilter();
    gyro_integrating_ = true;
    gyro_dir_settled_ = false;
    gyro_check_at_ = now + secondsToSteady(gyro_check_sec_);

    yaw_align_state_ = YawAlignTurning;
    yaw_in_window_count_ = 0;
    yaw_align_started_at_ = now;
    yaw_align_deadline_ = now + secondsToSteady(yaw_timeout_sec_);
    yaw_sign_check_at_ = now + secondsToSteady(yaw_sign_check_sec_);
    yaw_align_ref_error_ = fabs(start_error);
    yaw_runaway_ref_error_ = fabs(start_error);
    yaw_stall_ref_error_ = fabs(start_error);
    yaw_best_error_ = fabs(start_error);
    yaw_turn_floor_deg_ = yaw_min_turn_deg_;
    yaw_stall_check_at_ = now + secondsToSteady(yaw_stall_sec_);

    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Sudah berhenti di depan bola (yaw %+.1f deg, galat %+.1f). "
                "Mulai memutar.",
                yaw_at_rest, start_error);

    startInPlaceTurn();
    return false;
  }

  if (yaw_align_state_ == YawAlignSettling)
  {
    if (now < yaw_settle_until_)
      return false;

    // Periksa ULANG sesudah gait benar-benar reda.
    //
    // Yaw yang dibaca SAMBIL berjalan tidak bisa dipercaya: terukur di robot
    // 2026-08-28, dalam satu penyelarasan yang sama nilainya melompat
    // -26,5 / -30,3 / -31,5 / -27,5 / -33,4 / -44,3 / -28,2 / -18,1 dalam
    // selang setengah detik. Badan memang bergoyang saat gait jalan, dan
    // magnetometer ikut terbawa. Dengan toleransi +-5 deg, "tiga tick berturut
    // di dalam jendela" bisa terpenuhi oleh derau semata -- robot mengaku lurus
    // padahal tidak.
    //
    if (yaw_align_enable_ && isBodyYawUsable())
    {
      const double yaw_at_rest = filteredBodyYawDeg();
      const double settled_error = normalizeAngleDeg(yaw_target_deg_ - yaw_at_rest);

      // Pengali gyro TERUKUR, dari dua bacaan magnetometer yang sama-sama
      // diambil saat robot DIAM (satu sebelum memutar, satu sekarang). Itu
      // satu-satunya pembanding yang sahih di robot ini.
      //
      // Sekarang LANGSUNG DIPAKAI untuk putaran berikutnya, tidak cuma
      // dicetak. Ini yang menghapus overshoot sistematis: terukur 2026-08-31,
      // putaran yang menurut gyro 128,8 derajat ternyata 165 derajat menurut
      // magnetometer, jadi robot melewati target 37 derajat dan harus mengulang
      // empat kali sampai batas waktunya habis.
      //
      // Tetap TIDAK dipasang lewat launch: angkanya berbeda antar hari (1,74
      // pada 28 Agu, 0,66 pada 31 Agu), jadi ia harus diukur ulang tiap run --
      // dan itulah yang dikerjakan di sini.
      const double mag_turned = normalizeAngleDeg(yaw_at_rest - mag_before_deg_);
      if (fabs(turned_deg_) > 3.0)
      {
        const double measured = mag_turned / turned_deg_;

        // Syaratnya ketat, karena satu pengukuran buruk merusak semua putaran
        // sesudahnya: putarannya harus cukup besar (di bawah ini derau
        // magnetometer +-1,5 derajat mendominasi rasionya), searah, dan
        // rasionya masuk akal.
        const bool usable = (gyro_autoscale_ == true) &&
                            (fabs(turned_deg_) > 10.0) &&
                            (measured > 0.0) &&
                            (measured >= gyro_scale_min_) &&
                            (measured <= gyro_scale_max_);

        if (usable == true)
        {
          const double before = gyro_scale_;

          // Setengah-setengah, bukan langsung: satu pengukuran tetap membawa
          // derau, dan lingkar cek-ulang di bawah ini sudah menutup sisanya.
          gyro_scale_ = 0.5 * (gyro_scale_ + gyro_scale_ * measured);
          gyro_scale_ = fmin(fmax(gyro_scale_, gyro_scale_min_), gyro_scale_max_);

          RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                      "Pengali gyro DIPERBARUI: magnetometer %+.1f deg vs gyro "
                      "%+.1f deg -> rasio %.2f, pengali %.2f -> %.2f. Dipakai "
                      "untuk putaran berikutnya di run ini. Matikan dengan "
                      "kick_yaw_gyro_autoscale:=0.0.",
                      mag_turned, turned_deg_, measured, before, gyro_scale_);
        }
        else
        {
          RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                      "Pengali gyro terukur: magnetometer %+.1f deg vs gyro %+.1f deg "
                      "-> %.2f. TIDAK dipakai (%s).",
                      mag_turned, turned_deg_, measured,
                      (gyro_autoscale_ == false)
                          ? "kick_yaw_gyro_autoscale mati"
                          : ((fabs(turned_deg_) <= 10.0)
                                 ? "putarannya terlalu kecil untuk diukur"
                                 : "rasionya di luar batas yang masuk akal"));
        }
      }

      if (fabs(settled_error) > yaw_tolerance_deg_ && yaw_recheck_left_ > 0)
      {
        yaw_recheck_left_ -= 1;

        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Sesudah gait reda yaw ternyata masih %+.1f deg (galat %+.1f, "
                    "toleransi %.1f). Yang tadi terbaca 'lurus' itu derau saat "
                    "berjalan. Memutar lagi, sisa jatah %d.",
                    yaw_at_rest, settled_error, yaw_tolerance_deg_, yaw_recheck_left_);

        // Lewat fase MENGEREM, jalur yang sama persis dengan cek ulang sesudah
        // menggeser. Itu SATU-SATUNYA tempat turn_target_deg_ dan turned_deg_
        // dipasang ulang dari bacaan magnetometer yang baru, dan tanpanya
        // putaran ulang tidak pernah benar-benar terjadi.
        //
        // Versi lama melompat langsung ke YawAlignTurning sambil membawa
        // turn_target_deg_/turned_deg_ milik putaran SEBELUMNYA. Sisa putaran
        // yang tersimpan di situ sudah di dalam toleransi, jadi putarannya
        // "selesai" dua tick kemudian tanpa robot bergerak sedikit pun.
        //
        // Terukur di robot 2026-08-31, satu penyelarasan:
        //   68,031 yaw masih +6,5 -> memutar lagi, sisa jatah 2
        //   68,064 MENGITARI: sudah berputar -7,4 dari -12,2 (sisa -4,8)
        //   68,131 Putaran SELESAI: sudah berputar -7,4 dari -12,2 (sisa -4,8)
        // Angkanya tidak bergeser sama sekali dan jaraknya cuma 67 milidetik.
        // Ketiga jatah habis dalam 1,7 detik, lalu robot menendang dengan yaw
        // masih +6,7 deg. Dari luar persis terlihat sebagai "robot tidak
        // memutar sama sekali".
        enterBrakePhase(yaw_at_rest);
        return false;
      }

      if (fabs(settled_error) > yaw_tolerance_deg_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "Jatah putar ulang habis, yaw masih %+.1f deg (galat %+.1f). "
                     "Tendangan dibatalkan; toleransi tidak dilonggarkan otomatis.",
                     yaw_at_rest, settled_error);
        return abortKickAlignment("yaw masih di luar toleransi setelah putar ulang");
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Yaw TERVERIFIKASI sesudah gait reda: %+.1f deg (galat %+.1f).",
                    yaw_at_rest, settled_error);
      }
    }

    // Mengitari bola memindahkan SELURUH badan, jadi kaki belum tentu masih di
    // sebelah bola walaupun yaw-nya sudah benar. Geser dulu sampai bola tepat
    // di depan kaki terdekat, baru menendang.
    //
    // TAPI: kalau bolanya memang SUDAH di depan salah satu kaki, fase itu
    // dilewati sama sekali dan robot langsung menendang.
    //
    // Bukan sekadar menghemat waktu. Fase menggeser selalu menyalakan gait,
    // menunggu beberapa tick, lalu mematikannya lagi -- jadi robot melangkah
    // satu-dua langkah kecil yang tidak menghasilkan apa-apa, lalu menjalani
    // fase reda 1,8 detik lagi sesudahnya. Operator melihatnya sebagai
    // "gerakan kecil yang tidak berguna" sesudah menyesuaikan bola, dan
    // memang begitu adanya. Yaw baru saja diperiksa saat badan diam di blok
    // ini juga, jadi tidak ada yang perlu diperiksa ulang.
    const int foot_now = kickFootFromHeadPan();
    if (kickPositionDecision(ball_visible) == KickPositionDecision::Ready)
    {
      approach_ball_position_ = foot_now;
      foot_from_reposition_ = true;
      yaw_align_state_ = YawAlignOff;

      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Bola SUDAH di depan kaki %s (servo head_pan %.1f deg, jendela "
                  "kanan %.0f-%.0f kiri %.0f-%.0f). Fase menggeser DILEWATI, "
                  "langsung menendang.",
                  (foot_now == OnLeft) ? "KIRI" : "KANAN", headPanServoDeg(),
                  kick_pan_right_min_deg_, kick_pan_right_max_deg_,
                  kick_pan_left_min_deg_, kick_pan_left_max_deg_);
      return true;
    }

    if (reposition_enable_ == true)
    {
      startReposition();
      return false;
    }

    return abortKickAlignment("posisi bola belum sesuai dan reposisi dinonaktifkan");
  }

  // ---- MENYESUAIKAN: MAJU dulu kalau masih jauh, baru MENGGESER ---------
  // Dua tahap di dalam satu keadaan; processReposition() yang membaginya.
  if (yaw_align_state_ == YawAlignRepositioning)
    return processReposition(ball_visible, ball_radius_px);

  // ---- Diam sesudah menggeser, lalu yaw diperiksa sekali lagi ------------
  if (yaw_align_state_ == YawAlignPostSettle)
  {
    if (now < yaw_settle_until_)
      return false;

    if (yaw_align_enable_ && isBodyYawUsable())
    {
      const double yaw_at_rest = filteredBodyYawDeg();
      const double settled_error = normalizeAngleDeg(yaw_target_deg_ - yaw_at_rest);

      if (fabs(settled_error) > yaw_tolerance_deg_ && reposition_recheck_left_ > 0)
      {
        reposition_recheck_left_ -= 1;

        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Menggeser badan ternyata merusak yaw: sekarang %+.1f deg "
                    "(galat %+.1f, toleransi %.1f). Mengitari lagi, sisa jatah %d. "
                    "Kalau ini sering terjadi, kecilkan kick_reposition_y_max_mm.",
                    yaw_at_rest, settled_error, yaw_tolerance_deg_, reposition_recheck_left_);

        // Lewat fase MENGEREM, bukan langsung memutar: magnetometer harus
        // dibaca ulang saat robot benar-benar diam, dan itu yang dikerjakan
        // fase mengerem.
        yaw_recheck_left_ = yaw_recheck_max_;
        enterBrakePhase(yaw_at_rest);
        return false;
      }

      if (fabs(settled_error) > yaw_tolerance_deg_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "Jatah ulang sesudah menggeser habis, yaw masih %+.1f deg "
                     "(galat %+.1f). Tidak menendang.",
                     yaw_at_rest, settled_error);
        return abortKickAlignment("yaw berubah setelah reposisi");
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Yaw masih lurus sesudah menggeser: %+.1f deg (galat %+.1f). Periksa bola.",
                    yaw_at_rest, settled_error);
      }
    }

    if (kickPositionDecision(ball_visible) != KickPositionDecision::Ready)
    {
      // Menggeser LAGI hanya kalau jatahnya masih ada. Bawaannya satu kali,
      // jadi cabang ini biasanya langsung membatalkan: robot berhenti, mencari
      // dan mendekati bola dari awal, lalu memutar sekali lagi dengan bacaan
      // yang segar. Itu lebih cepat -- dan jauh lebih mudah dibaca operator --
      // daripada menggeser berulang ulang di tempat.
      if (reposition_enable_ && reposition_pass_count_ < reposition_max_passes_)
      {
        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Bola belum tepat sesudah menggeser; mengulang fase MENGGESER "
                    "(sudah %d dari %d kali).",
                    reposition_pass_count_, reposition_max_passes_);
        startReposition();
        return false;
      }
      if (reposition_enable_)
        return abortKickAlignment("bola belum tepat sesudah menggeser sekali");
      return abortKickAlignment("bola berpindah selama menunggu gait reda");
    }
    approach_ball_position_ = kickFootFromHeadPan();
    foot_from_reposition_ = true;
    yaw_align_state_ = YawAlignOff;
    return true;
  }

  // ---- YawAlignTurning ---------------------------------------------------
  // SISA PUTARAN dihitung dari gyro, bukan dari magnetometer.
  //
  // turn_target_deg_ dikunci saat robot diam (magnetometer sahih di situ),
  // turned_deg_ adalah integrasi gyro sejak saat itu. Selisihnya = sisa yang
  // belum diputar. Magnetometer sengaja TIDAK dilihat di sini: selama gait
  // berjalan ia berayun 40 derajat per detik dan akan merusak kendalinya.
  //
  // Hasil integrasi gyro DITAPIS dulu. Badan robot berputar maju-mundur di
  // dalam tiap langkah, dan ayunan itu ikut masuk ke integralnya: terukur
  // 2026-08-31, satu putaran satu arah tercatat -29 -16 +2 -1 -27 -42 -39
  // derajat berturut-turut, padahal integral putaran satu arah tidak mungkin
  // mengecil. Keputusan berhenti yang diambil dari satu sampel sesaat karena
  // itu jatuh pada fase ayunan yang acak, dan robot berhenti di mana saja.
  const double turned_now = filteredTurnedDeg();
  const double error = turn_target_deg_ - turned_now;

  // Dipakai hanya untuk MENCATAT: selama gait berjalan yang mengendalikan
  // putaran tetap gyro. Tapi angka yang dicatat pun harus yang sudah ditapis,
  // supaya baris log bisa dipakai menelusuri keputusan yang benar-benar
  // diambil dan bukan sampel lain lagi.
  const double yaw_logged = filteredBodyYawDeg();

  // Putaran berhenti pada ambang yang LEBIH KETAT daripada toleransi yang
  // dipakai memverifikasi hasilnya dengan magnetometer. Ini bukan kerewelan,
  // ini yang menentukan lingkarnya menutup atau tidak.
  //
  // Yang menghentikan putaran gyro, yang memvonis magnetometer, dan keduanya
  // TIDAK sepakat: pengukuran di robot ini malah berlawanan arah dari waktu ke
  // waktu -- 2026-08-28 magnetometer 32,4 deg vs gyro 18,6 deg (gyro kurang
  // baca, rasio 1,74), 2026-08-31 magnetometer 5,7 deg vs gyro 8,6 deg (gyro
  // lebih baca, rasio 0,66). Jadi rasionya bukan konstanta yang bisa dipasang.
  //
  // Dengan ambang berhenti SAMA DENGAN toleransi, titik diam lingkarnya jatuh
  // tepat DI toleransi: tiap putaran ulang berhenti persis di ambang lulus dan
  // hasilnya undian derau magnetometer. Dengan rasio r, galat sesudah satu
  // putaran adalah e' = e - r(e - ambang), yang menuju ambang/1 = ambang. Pakai
  // ambang setengah toleransi, titik diamnya jadi setengah toleransi juga --
  // aman di dalam, untuk r = 0,66 maupun r = 1,74.
  if (fabs(error) <= yaw_turn_stop_deg_)
  {
    yaw_in_window_count_ += 1;
    if (yaw_in_window_count_ >= yaw_ready_count_)
    {
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Putaran SELESAI menurut gyro: sudah berputar %+.1f dari %+.1f deg "
                  "(sisa %+.1f, ambang berhenti %.1f, toleransi %.1f). "
                  "Menunggu diam lalu dicek magnetometer.",
                  turned_now, turn_target_deg_, error, yaw_turn_stop_deg_,
                  yaw_tolerance_deg_);
      return finishYawTurn("yaw sudah di dalam toleransi");
    }
  }
  else
  {
    yaw_in_window_count_ = 0;
  }

  // Pengaman putaran LIAR, diperiksa SEBELUM apa pun yang lain.
  //
  // Terukur 2026-08-28: robot berputar dari galat 18 deg sampai 130 deg dan
  // terus menjauh, baik saat tanda putaran +1 MAUPUN sesudah dibalik jadi -1.
  // Artinya penyebabnya bukan sekadar tanda yang keliru, dan koreksi tanda
  // otomatis TIDAK cukup jadi pengaman. Apa pun sebabnya, robot yang berputar
  // makin jauh dari target harus dihentikan, bukan dibiarkan menghabiskan
  // batas waktu sambil terus berputar.
  if (yaw_runaway_deg_ > 0.0 &&
      fabs(error) > yaw_runaway_ref_error_ + yaw_runaway_deg_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                 "PUTARAN LIAR: galat membesar %.1f -> %.1f deg (lewat batas %.1f). "
                 "Putaran DIHENTIKAN supaya robot tidak berputar terus. "
                 "Periksa kick_yaw_turn_sign; setel kick_yaw_align_enable:=0.0 "
                 "untuk mematikan penyelarasan yaw sementara.",
                 yaw_runaway_ref_error_, fabs(error), yaw_runaway_deg_);
    yaw_recheck_left_ = 0;
    return finishYawTurn("putaran liar, dihentikan paksa");
  }

  // Batas waktu hanya berlaku kalau operator memang memasangnya. Bawaannya 0 =
  // tanpa batas: robot TIDAK BOLEH menendang sebelum yaw benar-benar lurus.
  if (yaw_timeout_sec_ > 0.0 && now >= yaw_align_deadline_)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Batas waktu penyelarasan yaw habis (%.1f s), sisa galat %+.1f deg. "
                "Berhenti dan periksa ulang; tendangan tetap memerlukan yaw, "
                "jarak, dan posisi kaki yang sesuai.",
                yaw_timeout_sec_, error);
    return finishYawTurn("batas waktu habis");
  }

  // Koreksi arah putaran DARI GYRO. Cepat dan tegas, karena gyro langsung
  // memberi tahu robot sedang berputar ke mana.
  //
  // Ini yang dulu tidak mungkin: dengan magnetometer, menilai arah butuh
  // jendela 3-5 detik dan hasilnya tetap salah dua kali, karena deraunya lebih
  // besar daripada sinyalnya. Gyro tidak punya masalah itu; sesudah ~1 detik
  // tanda putarannya sudah jelas.
  //
  // Syaratnya sudah benar-benar berputar (di atas 2 derajat), supaya getaran
  // kecil saat gait mulai tidak salah dibaca sebagai arah.
  if (gyro_dir_settled_ == false && now >= gyro_check_at_)
  {
    if (fabs(turned_now) < 2.0)
    {
      // Belum ada putaran terukur sama sekali. Bisa jadi soft start belum
      // selesai, bisa juga gyro memang tidak menjawab. Tunda penilaian dan
      // beri tahu supaya bedanya kelihatan di log.
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BallFollower"), *log_clock_, 2000,
                           "Gyro belum mencatat putaran (%.1f deg sesudah %.1f s). "
                           "Kalau angkanya tetap nol, periksa gyro_z di /robotis/open_cr/imu.",
                           turned_now, gyro_check_sec_);
      gyro_check_at_ = now + secondsToSteady(gyro_check_sec_);
    }
    else
    {
      gyro_dir_settled_ = true;

      // Berputar ke arah yang BERLAWANAN dengan yang diminta: balik tandanya
      // sekali, lalu nolkan hitungan supaya putaran yang salah tadi tidak ikut
      // terhitung sebagai kemajuan.
      if (turned_now * turn_target_deg_ < 0.0)
      {
        if (orbitActive() == true)
        {
          // LAPORAN saja: preset TIDAK ditukar diam-diam.
          //
          // Pemetaan "yaw positif -> PUTAR KIRI" itu perintah operator, dan
          // preset yang terpakai harus bisa ditebak dari bacaan yaw. Menukarnya
          // di tengah putaran membuat robot memakai preset yang berlawanan
          // dengan yang diminta, persis keluhan yang mau diperbaiki.
          //
          // Robot tidak dibiarkan berputar liar: pengaman PUTARAN LIAR di atas
          // menghentikannya sesudah galat membesar melewati kick_yaw_runaway_deg.
          RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                       "ARAH ORBIT TERBALIK menurut gyro: diminta %+.1f deg tapi "
                       "berputar %+.1f deg dalam %.1f s memakai preset %s. "
                       "Preset TIDAK ditukar otomatis (pemetaan yaw->preset dikunci). "
                       "Kalau ini terulang, jalankan dengan "
                       "kick_orbit_left_yaw_sign:=%.1f atau tukar isi kedua bagian "
                       "di preset_walking.yaml.",
                       turn_target_deg_, turned_now, gyro_check_sec_,
                       orbit_using_left_ ? "PUTAR KIRI" : "PUTAR KANAN",
                       -orbit_left_yaw_sign_);
        }
        else
        {
          // Jalur lama (memutar di poros sendiri) tetap mengoreksi diri: di
          // sana yang dibalik cuma tanda sudut, bukan pilihan preset operator.
          yaw_turn_sign_ *= -1.0;
          RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                       "ARAH PUTARAN TERBALIK menurut gyro: diminta %+.1f deg tapi "
                       "berputar %+.1f deg dalam %.1f s. Tanda dibalik jadi %+.0f. "
                       "Pasang permanen dengan kick_yaw_turn_sign:=%.1f",
                       turn_target_deg_, turned_now, gyro_check_sec_,
                       yaw_turn_sign_, yaw_turn_sign_);
          turn_target_deg_ -= turned_now;
          turned_deg_ = 0.0;

          // Hitungan dinolkan, jadi jendela penapis juga -- kalau tidak, garis
          // lurusnya masih memuat putaran ke arah yang salah tadi.
          resetYawFilter();
        }
      }
      else if (orbitActive() == true)
      {
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                    "Arah orbit benar menurut gyro (preset %s, tanda %+.0f): "
                    "diminta %+.1f, sudah berputar %+.1f deg.",
                    orbit_using_left_ ? "PUTAR KIRI" : "PUTAR KANAN",
                    orbit_left_yaw_sign_, turn_target_deg_, turned_now);
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                    "Arah putaran benar menurut gyro (%+.0f): diminta %+.1f, "
                    "sudah berputar %+.1f deg.",
                    yaw_turn_sign_, turn_target_deg_, turned_now);
      }
    }
  }

  // Eskalasi saat MACET.
  //
  // Terukur di robot 2026-08-28, dua penyelarasan dalam run yang sama:
  //   galat 39 deg -> perintah 13,7 deg/langkah -> yaw -39,1 ... -4,1 (3,7 s)
  //   galat 12 deg -> perintah  5,0 deg/langkah -> yaw -12 ... -17, TIDAK gerak
  // Jadi ada ambang torsi minimum, dan justru galat KECIL yang menghasilkan
  // perintah di bawah ambang itu (gain * galat jatuh ke batas bawah). Tanpa
  // eskalasi, "memutar sampai benar-benar lurus" tidak akan pernah selesai.
  //
  // Yang dinaikkan batas BAWAH-nya, bukan gain: yang kurang memang cuma langkah
  // terkecil. Batas atas tidak pernah dilewati, jadi putarannya tetap tidak
  // lebih ganas daripada berbelok saat mengejar bola.
  if (fabs(error) < yaw_best_error_)
    yaw_best_error_ = fabs(error);

  if (now >= yaw_stall_check_at_)
  {
    const bool stalled = (yaw_best_error_ > yaw_stall_ref_error_ - yaw_stall_margin_deg_);

    if (stalled == true && orbitActive() == true)
    {
      // Amplitudo preset itu tetap -- tidak ada batas bawah yang bisa dinaikkan
      // di sini. Yang bisa menolong cuma memperbesar salah satu sumbu putar di
      // presetnya sendiri. Sejak 2026-09-04 KEDUA sumbu dipakai (PUTAR KIRI
      // angle -0,07 + turn_in +0,015; PUTAR KANAN angle +0,015 + turn_in
      // -0,09), jadi angka yang sedang hidup ikut dicetak -- operator tidak
      // perlu menebak sumbu mana yang dominan di sisi ini.
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "MENGITARI MACET (%.1f s, galat terbaik %.1f deg dari %.1f) memakai "
                  "preset %s (sudut %+.3f rad, turn_in %+.3f rad). Amplitudo preset "
                  "tetap, jadi robot tidak bisa memperbesar putarannya sendiri: "
                  "besarkan angle_move_amplitude dan/atau angle_move_amplitude_turn_in "
                  "di %s, atau turunkan robot kalau ia sedang DIGANTUNG (kaki tidak "
                  "menapak = yaw memang tidak akan berubah).",
                  yaw_stall_sec_, yaw_best_error_, yaw_stall_ref_error_,
                  orbit_using_left_ ? "PUTAR KIRI" : "PUTAR KANAN",
                  current_walking_param_.angle_move_amplitude,
                  current_walking_param_.angle_move_amplitude_turn_in,
                  preset_config_path_.c_str());
    }
    else if (stalled == true && yaw_turn_floor_deg_ < yaw_max_turn_deg_)
    {
      yaw_turn_floor_deg_ = fmin(yaw_turn_floor_deg_ + yaw_turn_step_deg_, yaw_max_turn_deg_);
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Putaran MACET (%.1f s, galat terbaik %.1f deg dari %.1f). "
                  "Batas bawah putaran dinaikkan ke %.1f deg/langkah.",
                  yaw_stall_sec_, yaw_best_error_, yaw_stall_ref_error_, yaw_turn_floor_deg_);
    }
    else if (stalled == true)
    {
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Putaran MACET di batas atas %.1f deg/langkah, galat %+.1f deg tidak turun. "
                  "Kalau robot DIGANTUNG, kakinya tidak menapak dan yaw memang tidak akan "
                  "berubah -- turunkan robot atau tekan STOP.",
                  yaw_max_turn_deg_, error);
    }

    yaw_stall_ref_error_ = fabs(error);
    yaw_best_error_ = fabs(error);
    yaw_stall_check_at_ = now + secondsToSteady(yaw_stall_sec_);
  }

  // ---- MENGITARI BOLA memakai preset PUTAR KIRI / PUTAR KANAN ------------
  //
  // Preset dikirim APA ADANYA, tidak diskalakan galat. Operator sudah
  // menyetelnya langsung di robot lewat Bascorro Studio; mengalikan amplitudo
  // hasil setelan itu dengan galat sama saja dengan membuang setelannya. Yang
  // menghentikan putaran tetap gyro -- pemeriksaan toleransi di atas berlaku
  // untuk kedua jalur.
  //
  // Jalur lama (memutar di poros sendiri, di bawah) tetap ada dan dipakai kalau
  // presetnya gagal dimuat atau kick_orbit_enable dimatikan.
  if (orbitActive() == true)
  {
    // error = SISA putaran (target - yang sudah diputar), jadi -error adalah
    // simpangan yaw yang masih tersisa: positif berarti robot masih menghadap
    // terlalu ke arah yaw positif. Itulah angka yang dipakai aturan operator.
    //
    // Dinilai tiap tick, bukan sekali di awal: kalau putaran sampai MELEWATI
    // target, robot langsung pindah ke preset satunya untuk kembali.
    //
    // Kecuali sesudah galat masuk toleransi -- di situ pilihannya DIBEKUKAN.
    // Menyelesaikan putaran masih memakan yaw_ready_count_ siklus, dan dalam
    // sisa siklus itu galat bisa menyeberang nol satu-dua derajat; tanpa
    // pembekuan presetnya tertukar bolak-balik di detik terakhir tanpa guna.
    const bool use_left = (fabs(error) <= yaw_turn_stop_deg_ && orbit_announced_ >= 0)
                              ? orbit_using_left_
                              : chooseOrbitLeft(-error);
    publishOrbitParam(use_left);

    // Satu baris keras tiap kali presetnya berganti (termasuk yang pertama).
    // Tanpa ini satu-satunya jejak pilihan preset adalah INFO yang dibatasi
    // laju, dan pertanyaan "preset mana yang tadi dipakai?" jadi tidak
    // terjawab sesudah demo selesai.
    if (orbit_announced_ != (use_left ? 1 : 0))
    {
      orbit_announced_ = use_left ? 1 : 0;
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Preset MENGITARI dipilih: %s (yaw %+.1f deg %s nol, sisa putaran "
                  "%+.1f deg). sudut %+.3f, turn_in %+.3f, y %+.1f mm, period %.2f s.",
                  use_left ? "PUTAR KIRI" : "PUTAR KANAN",
                  yaw_logged, (yaw_logged >= 0.0) ? "di atas" : "di bawah",
                  error,
                  current_walking_param_.angle_move_amplitude,
                  current_walking_param_.angle_move_amplitude_turn_in,
                  current_walking_param_.y_move_amplitude * 1000.0,
                  current_walking_param_.period_time);
    }

    const double orbit_elapsed =
        std::chrono::duration<double>(now - yaw_align_started_at_).count();

    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"),
                         *log_clock_, 500,
                         "MENGITARI bola pakai preset %s: yaw %+.1f deg, target %+.1f "
                         "| sudah berputar %+.1f dari %+.1f deg (sisa %+.1f, berhenti di %.1f) "
                         "| sudut %+.3f, turn_in %+.3f, y %+.1f mm, z %.1f mm | sudah %.1f s",
                         use_left ? "PUTAR KIRI" : "PUTAR KANAN",
                         yaw_logged, yaw_target_deg_,
                         turned_now, turn_target_deg_, error, yaw_turn_stop_deg_,
                         current_walking_param_.angle_move_amplitude,
                         current_walking_param_.angle_move_amplitude_turn_in,
                         current_walking_param_.y_move_amplitude * 1000.0,
                         current_walking_param_.z_move_amplitude * 1000.0,
                         orbit_elapsed);

    return false;
  }

  // Besar putaran sebanding galat, dijepit antara batas bawah (yang bisa naik
  // sendiri di atas) dan batas atas -- pola yang sama dengan calcFootstep()
  // saat mengejar bola.
  double turn_deg = fabs(error) * yaw_turn_gain_;
  turn_deg = fmin(turn_deg, yaw_max_turn_deg_);

  // Batas bawah TIDAK BOLEH melebihi sisa galat. Kalau sisa galatnya 2 derajat
  // tapi batas bawahnya 5, robot dipaksa memutar 5 derajat dan pasti
  // melewati target -- overshoot yang dijamin oleh rumusnya sendiri.
  //
  // Terlihat di run lantai 2026-08-28 sebagai ayunan pada cek-ulang:
  //   LURUS -3,2 -> cek ulang -7,1 -> LURUS -3,4 -> cek ulang -5,8
  //   LURUS -3,7 -> cek ulang +14,3 -> LURUS -2,5 -> TERVERIFIKASI +2,6
  // Robot bolak-balik melewati nol, bukan mendekatinya.
  //
  // Batas bawah tetap ada gunanya (ada ambang torsi minimum di bawahnya kaki
  // tidak bergerak sama sekali), tapi dipotong oleh sisa galat.
  const double floor_deg = fmin(yaw_turn_floor_deg_, fabs(error));
  turn_deg = fmax(turn_deg, floor_deg);

  const double turn_rad =
      yaw_turn_sign_ * ((error >= 0.0) ? turn_deg : -turn_deg) * M_PI / 180.0;

  // MENGITARI bola, bukan berputar di poros sendiri.
  //
  // Robot yang berputar di tempat pasti melempar bola keluar dari tengah:
  // bola diam, badan berputar, jadi arah bola relatif badan bergeser sebesar
  // sudut putarnya. Supaya bola tetap di tengah, robot harus bergeser
  // MENYAMPING sambil berputar, yaitu mengitari bola.
  //
  // Umpan baliknya sudut leher. Kepala dipegang head_tracking_node dan terus
  // mengunci bola, jadi current_pan_ itu arah bola relatif badan: positif
  // berarti bola di kiri. Untuk membawanya kembali ke nol, robot bergeser ke
  // sisi yang sama dengan bolanya.
  //
  // Tandanya TIDAK ditebak: ia mengikuti konvensi yang sama dengan sumbu putar
  // yang sudah diukur di robot ini (angle_move_amplitude positif memutar ke
  // kiri dan menaikkan yaw), yaitu y_move_amplitude positif bergerak ke kiri.
  // Kalaupun keliru, ini lingkar tertutup dengan batas atas kecil: efeknya
  // geseran menyamping yang salah arah beberapa milimeter, bukan putaran liar.
  double y_move = 0.0;
  const double ball_bearing_deg = current_pan_ * 180.0 / M_PI;

  if (ball_visible == true && yaw_strafe_gain_ > 0.0)
  {
    y_move = ball_bearing_deg * yaw_strafe_gain_;
    y_move = fmin(y_move, yaw_strafe_max_);
    y_move = fmax(y_move, -yaw_strafe_max_);
  }

  setWalkingParam(0.0, y_move, turn_rad);

  const double elapsed =
      std::chrono::duration<double>(now - yaw_align_started_at_).count();

  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"),
                       *log_clock_, 500,
                       "menyelaraskan yaw: sekarang %+.1f deg, target %+.1f, galat %+.1f "
                       "(toleransi %.1f) -> putar %+.1f deg/langkah "
                       "(batas bawah %.1f) | bola %+.1f deg -> geser %+.1f mm (%s) | sudah %.1f s",
                       yaw_logged, yaw_target_deg_, error, yaw_tolerance_deg_,
                       turn_rad * 180.0 / M_PI, yaw_turn_floor_deg_,
                       ball_bearing_deg, y_move * 1000.0,
                       ball_visible ? "bola terlihat" : "bola TIDAK terlihat, tidak menggeser",
                       elapsed);

  return false;
}

// =========================================================================
//  MENGGESER: bawa bola ke depan kaki terdekat sesudah mengitari
// =========================================================================
//
// Mengitari bola memindahkan SELURUH badan, bukan cuma memutarnya. Jadi begitu
// yaw benar, kaki belum tentu masih di sebelah bola. Fase ini menutup selisih
// itu dengan bergeser menyamping (dan maju kalau bolanya memang menjauh),
// dengan sudut putar DIKUNCI NOL supaya yaw yang baru diluruskan tidak rusak.
//
// Syarat selesainya sama persis dengan pemicu tendangan biasa: servo head_pan
// masuk salah satu jendela tendang DAN jari-jari bola sudah melewati ambang.
// Memakai ukuran yang sama berarti "sudah di depan kaki" tidak bisa berarti dua
// hal berbeda di dua tempat.

double BallFollower::ballDistanceFromObservation(bool filtered) const
{
  if (!hasFreshBallObservation())
    return std::numeric_limits<double>::quiet_NaN();
  double azimuth = 0.0, depression = 0.0;
  ballDirection(ball_img_pan_rad_, ball_img_tilt_rad_, azimuth, depression);
  const double distance = kickGroundDistance(camera_height_, kick_ball_real_radius_m_,
                                              depression, min_depression_deg_ * M_PI / 180.0);
  if (!filtered || !std::isfinite(distance) || distance_buf_.empty())
    return distance;
  std::vector<double> sorted(distance_buf_.begin(), distance_buf_.end());
  std::sort(sorted.begin(), sorted.end());
  return sorted[sorted.size() / 2];
}

bool BallFollower::abortKickAlignment(const char *reason)
{
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Tendangan DIBATALKAN: %s. Berhenti lalu cari/dekati bola kembali.", reason);
  cancelYawAlign();
  foot_from_reposition_ = false;
  return false;
}

void BallFollower::enterApproachStage(const char *reason)
{
  restoreBaseWalkingParam();
  reposition_stage_ = RepoApproach;
  current_x_move_ = 0.003;
  current_r_angle_ = 0.0;
  accum_period_time_ = 0.0;
  z_lead_progress_ = 0.0;
  approach_ready_count_ = 0;
  reposition_ready_count_ = 0;
  shift_announced_ = -1;
  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "MAJU memakai preset JALAN MENUJU MEKKAH (%s), jarak %.3f -> %.3f m.",
              reason, ballDistanceFromObservation(), kick_distance_);
  beginTransitionDelay("PUTAR/GESER", "MAJU");
}

void BallFollower::beginTransitionDelay(const char *from, const char *to)
{
  if (transition_delay_sec_ <= 0.0)
    return;

  transition_until_ = std::chrono::steady_clock::now()
                    + secondsToSteady(transition_delay_sec_);
  resetDistanceFilter();

  // Amplitudo dinolkan SEKARANG, gait dibiarkan hidup. Robot melangkah di
  // tempat selama jeda: badan berhenti berayun dan kepala sempat mengunci bola
  // sebelum gerakan berikutnya mengambil alih.
  setWalkingParam(0.0, 0.0, 0.0);

  // Batas waktu fase menyesuaikan diperpanjang sebanyak jedanya. Jeda ini
  // bukan waktu kerja, jadi ia tidak boleh memakan jatah yang disediakan untuk
  // benar-benar menggeser.
  reposition_deadline_ += secondsToSteady(transition_delay_sec_);

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Jeda transformasi %.2f s: %s -> %s.", transition_delay_sec_, from, to);
}

bool BallFollower::transitionDelayActive()
{
  return std::chrono::steady_clock::now() < transition_until_;
}

void BallFollower::startReposition()
{
  const auto now = std::chrono::steady_clock::now();

  reposition_pass_count_ += 1;

  restoreBaseWalkingParam();

  reposition_x_ramp_ = 0.0;
  reposition_ready_count_ = 0;
  approach_ready_count_ = 0;
  reposition_prev_tick_ = now;
  reposition_deadline_ = now + secondsToSteady(reposition_timeout_sec_);
  reposition_blind_ = false;
  reposition_blind_since_ = now;
  reposition_pan_last_deg_ = headPanServoDeg();
  reposition_pan_moved_at_ = now;
  // Jendela penilaian tanda dilepas; ia dipasang lagi begitu simpangan bola
  // cukup besar untuk dinilai. reposition_sign_settled_ SENGAJA tidak disetel
  // ulang: arah geser itu sifat robot, sekali terukur dipakai seterusnya.
  reposition_sign_ref_deg_ = -1.0;

  // KAKI PENENDANG DIKUNCI DI SINI, sekali, dan sasaran geseran mengikutinya
  // sampai fase ini selesai. Lihat catatan panjang di header: menghitung ulang
  // tiap tick membuat sasaran melompat antar-jendela dan robot menggeser
  // bolak-balik.
  reposition_target_foot_ = nearestKickFoot();
  reposition_foot_switched_ = false;
  shift_announced_ = -1;

  // Gyro dinolkan dan dibiarkan berjalan, tapi hanya untuk MELAPORKAN berapa
  // derajat geseran ini menggeser yaw. Keputusannya tetap di magnetometer
  // sesudah badan diam (YawAlignPostSettle): selama gait berjalan magnetometer
  // berayun puluhan derajat dan tidak bisa dipercaya.
  turned_deg_ = 0.0;
  gyro_integrating_ = true;

  yaw_align_state_ = YawAlignRepositioning;

  // Amplitudo nol dulu, baru "start". Gait dihentikan finishYawTurn() dan sudah
  // menjalani fase reda, jadi jarak waktu ke "stop" sebelumnya sudah aman --
  // bukan dua perintah yang tiba dalam satu siklus kontrol 8 ms yang sama.
  setWalkingParam(0.0, 0.0, 0.0);
  publishWalkingCommand("start");

  // ---- MAJU dulu atau langsung MENGGESER? --------------------------------
  //
  // Mengitari bola memindahkan seluruh badan mengelilinginya, jadi jarak ke
  // bola sesudah yaw lurus sering lebih jauh daripada sebelum memutar. Kalau
  // masih jauh, robot maju dulu dengan preset JALAN biasa; menggeser
  // menyamping dari jarak segitu cuma menghabiskan batas waktu tanpa pernah
  // membawa bola ke depan kaki.
  if (reposition_approach_enable_ &&
      kickPositionDecision(true) == KickPositionDecision::Approach)
  {
    enterApproachStage("bola menjauh setelah memutar");
    return;
  }

  reposition_stage_ = RepoShift;
  beginTransitionDelay("MEMUTAR", "MENGGESER");

  if (shiftPresetActive() == true)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Yaw sudah lurus. MENYESUAIKAN posisi kaki dengan bola memakai preset "
                "GESER KIRI/KANAN: kaki %s (dikunci, sasaran servo %.1f deg), pita mati "
                "%.1f deg, batas waktu %.1f s.",
                (reposition_target_foot_ == OnLeft) ? "KIRI" : "KANAN",
                kickWindowCenterDeg(reposition_target_foot_),
                reposition_deadband_deg_, reposition_timeout_sec_);
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Yaw sudah lurus. MENGGESER badan supaya bola tepat di depan kaki %s "
                "(dikunci, sasaran servo %.1f deg; samping maks %.1f mm, maju maks %.1f mm, "
                "batas waktu %.1f s). Sudut putar dikunci nol.",
                (reposition_target_foot_ == OnLeft) ? "KIRI" : "KANAN",
                kickWindowCenterDeg(reposition_target_foot_),
                reposition_y_max_ * 1000.0, reposition_x_max_ * 1000.0,
                reposition_timeout_sec_);
  }
}

void BallFollower::enterShiftStage(const char *reason)
{
  reposition_stage_ = RepoShift;
  approach_ready_count_ = 0;

  // Yang tertulis di walking_module sekarang preset JALAN dengan amplitudo
  // hasil ramp. Dikembalikan ke parameter dasar dulu supaya tahap MENGGESER
  // berangkat dari keadaan yang sama seperti kalau ia dimasuki langsung.
  restoreBaseWalkingParam();
  shift_announced_ = -1;
  reposition_x_ramp_ = 0.0;
  reposition_ready_count_ = 0;

  // KAKI PENENDANG DIKUNCI ULANG di sini, sekali.
  //
  // startReposition() menguncinya SEBELUM tahap MAJU, dan MAJU memindahkan
  // badan beberapa puluh sentimeter ke depan -- bola bisa berpindah ke jendela
  // kaki yang lain karena perpindahan itu. Menguncinya sekali lagi di sini
  // memakai bacaan yang baru, lalu jatah pindah sekali (histeresis 10 deg) di
  // tahap MENGGESER dipasang lagi sebagai jaring pengaman.
  //
  // Yang TIDAK boleh dilakukan: menghitung ulang tiap tick. Terukur 2026-09-01
  // itu membuat sasaran melompat antar-jendela 111 kali dalam 200 tick.
  reposition_target_foot_ = nearestKickFoot();
  reposition_foot_switched_ = false;

  // Sudut leher dianggap BARU bergerak mulai sekarang. Pemeriksaan "umpan
  // balik basi" di tahap MENGGESER mengukur sejak kapan head_pan terakhir
  // berubah; tanpa penolan ini ia ikut menghitung detik-detik yang dihabiskan
  // MAJU dan bisa langsung memvonis basi di tick pertama.
  reposition_pan_last_deg_ = headPanServoDeg();
  reposition_pan_moved_at_ = std::chrono::steady_clock::now();

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "MAJU selesai (%s). Kaki penendang dikunci ulang: %s "
              "(sasaran servo %.1f deg).",
              reason, (reposition_target_foot_ == OnLeft) ? "KIRI" : "KANAN",
              kickWindowCenterDeg(reposition_target_foot_));
  beginTransitionDelay("MAJU", "MENGGESER");
}

bool BallFollower::processRepositionApproach(
    bool ball_visible, const std::chrono::steady_clock::time_point &now)
{
  const double delta_time =
      std::chrono::duration<double>(now - reposition_prev_tick_).count();
  reposition_prev_tick_ = now;

  if (now >= reposition_deadline_)
    return abortKickAlignment("batas waktu maju habis sebelum jarak tercapai");

  if (ball_visible == false)
  {
    setWalkingParam(0.0, 0.0, 0.0);
    approach_ready_count_ = 0;
    return false;
  }

  const double distance = ballDistanceFromObservation();
  if (!std::isfinite(distance))
  {
    setWalkingParam(0.0, 0.0, 0.0);
    approach_ready_count_ = 0;
    return false;
  }

  if (distance <= kick_distance_)
  {
    setWalkingParam(0.0, 0.0, 0.0);
    approach_ready_count_ += 1;
    if (approach_ready_count_ >= std::max(1, kick_ready_count_))
    {
      enterShiftStage("jarak sudah cukup");
      return false;
    }
    return false;
  }
  else
  {
    approach_ready_count_ = 0;
  }

  // Maju LURUS. Sudut putar dikunci nol: yaw baru saja diluruskan dan
  // diperiksa saat badan diam, dan tahap ini tidak boleh merusaknya lagi.
  // Karena itu target_angle 0.0 -- calcFootstep() menyisihkan suku putarannya
  // sendiri di bawah 5 derajat, jadi rl_angle memang tinggal nol.
  double fb_move = 0.0, rl_angle = 0.0;
  calcFootstep(distance - kick_distance_, 0.0, delta_time, fb_move, rl_angle);
  setWalkingParam(fb_move, 0.0, 0.0);

  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"), *log_clock_, 500,
                       "MAJU menyesuaikan (preset JALAN): jarak %.3f m -> ambang %.3f m "
                       "| langkah %.1f mm, angkat %.1f mm | siap %d/%d | sisa %.1f s",
                       distance, kick_distance_, fb_move * 1000.0,
                       current_walking_param_.z_move_amplitude * 1000.0,
                       approach_ready_count_, kick_ready_count_,
                       std::chrono::duration<double>(reposition_deadline_ - now).count());

  return false;
}

bool BallFollower::processReposition(bool ball_visible, double ball_radius_px)
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= reposition_deadline_)
    return abortKickAlignment("batas waktu reposisi habis");

  const auto decision = kickPositionDecision(ball_visible);
  if (decision == KickPositionDecision::Wait)
  {
    if (!reposition_blind_)
    {
      reposition_blind_ = true;
      reposition_blind_since_ = now;
    }
    restoreBaseWalkingParam();
    setWalkingParam(0.0, 0.0, 0.0);
    approach_ready_count_ = 0;
    reposition_ready_count_ = 0;
    if (now - reposition_blind_since_ >= secondsToSteady(reposition_blind_sec_))
      return abortKickAlignment("posisi atau jarak bola tidak dapat diverifikasi");
    return false;
  }

  // Jeda transformasi gerakan: robot melangkah di tempat dan TIDAK ada
  // keputusan yang diambil. Amplitudo nolnya sudah diterbitkan sekali oleh
  // beginTransitionDelay(); menerbitkannya lagi tiap tick tidak menambah apa
  // pun. Jam tick tetap dimajukan supaya delta_time tahap MAJU tidak melompat
  // sepanjang jeda begitu jedanya habis.
  if (transitionDelayActive() == true)
  {
    reposition_prev_tick_ = now;
    return false;
  }

  // ---- Tahap MAJU: mendekat dulu dengan preset JALAN ---------------------
  if (reposition_stage_ == RepoApproach)
  {
    reposition_blind_ = false;
    if (decision == KickPositionDecision::Shift)
    {
      enterShiftStage("arah bola keluar jendela kaki selama maju");
      return false;
    }
    return processRepositionApproach(ball_visible, now);
  }

  if (decision == KickPositionDecision::Approach)
  {
    if (!reposition_approach_enable_)
      return abortKickAlignment("bola masih jauh dan maju ulang dinonaktifkan");
    reposition_blind_ = false;
    enterApproachStage("bola menjauh selama menggeser");
    return false;
  }

  const int foot_now = kickFootFromHeadPan();
  // Ambang radius yang SAMA dengan pemicu tendangan, dan ikut MATI bersamanya.
  //
  // Justru di sini akibat buruknya paling besar: selama radius belum mencapai
  // ambang, syarat "bola sudah di depan kaki" tidak pernah terpenuhi. Terukur
  // 2026-08-31, radius mentok di 78,7 px terhadap ambang 83, jadi "siap" tidak
  // pernah naik dari 0/4 dan fase menggeser selalu habis di batas waktu 12
  // detik walaupun bolanya sudah tepat di depan kaki sejak detik pertama.
  const bool radius_ok = (radius_ready_enable_ == false)
                      || (kick_ball_radius_px_ <= 0.0)
                      || (ball_radius_px >= kick_ball_radius_px_);

  if (decision == KickPositionDecision::Ready && foot_now != OutOfRange && radius_ok)
  {
    restoreBaseWalkingParam();
    setWalkingParam(0.0, 0.0, 0.0);
    shift_announced_ = -1;
    reposition_ready_count_ += 1;

    if (reposition_ready_count_ >= std::max(1, kick_ready_count_))
    {
      approach_ball_position_ = foot_now;
      return finishReposition("bola sudah di depan kaki");
    }
    return false;
  }
  else
  {
    reposition_ready_count_ = 0;
  }

  // Sasaran memakai kaki yang DIKUNCI startReposition(), bukan yang terdekat
  // saat ini. Perpindahan diizinkan PALING BANYAK SEKALI, dan hanya kalau
  // jendela seberang benar-benar lebih dekat sejauh histeresisnya -- di bawah
  // itu perbedaannya cuma derau sudut leher, dan mengikutinya berarti robot
  // menggeser ke satu arah lalu ke arah sebaliknya tanpa pernah selesai.
  if (reposition_foot_switched_ == false)
  {
    const int other_foot = (reposition_target_foot_ == OnRight) ? OnLeft : OnRight;
    const double servo_deg = headPanServoDeg();
    const double dist_now =
        fabs(servo_deg - kickWindowCenterDeg(reposition_target_foot_));
    const double dist_other = fabs(servo_deg - kickWindowCenterDeg(other_foot));

    if (dist_other + reposition_foot_hysteresis_deg_ < dist_now)
    {
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Sasaran geseran PINDAH ke kaki %s (sekali saja): servo head_pan "
                  "%.1f deg, jarak ke jendela seberang %.1f deg lawan %.1f deg, "
                  "lewat histeresis %.1f deg.",
                  (other_foot == OnLeft) ? "KIRI" : "KANAN", servo_deg,
                  dist_other, dist_now, reposition_foot_hysteresis_deg_);
      reposition_target_foot_ = other_foot;
      reposition_foot_switched_ = true;
    }
  }

  const int target_foot = reposition_target_foot_;
  const double target_deg = kickWindowCenterDeg(target_foot);
  // Simpangan bola dari tengah jendela tendang, dengan konvensi "positif = bola
  // di KIRI". head_pan_servo_dir_ yang menerjemahkan dari derajat servo ke
  // konvensi itu, jadi servo yang terpasang terbalik tidak membalik arah geser.
  const double pan_now_deg = headPanServoDeg();
  const double bearing_deg = head_pan_servo_dir_ * (pan_now_deg - target_deg);

  // Sejak kapan sudut leher terakhir BERGERAK. Dicatat tiap tick, juga saat
  // robot sedang menunggu, supaya angkanya tidak ikut membeku.
  if (fabs(pan_now_deg - reposition_pan_last_deg_) > 0.05)
  {
    reposition_pan_last_deg_ = pan_now_deg;
    reposition_pan_moved_at_ = now;
  }

  // ---- Umpan balik harus SAHIH sebelum dipakai menggeser -----------------
  //
  // Dua cara ia jadi tidak sahih:
  //
  //  1. Bola tidak terlihat. Sudut leher cuma menunjuk ke bola kalau pelacak
  //     kepala memang sedang mengejarnya; kalau tidak, menggeser mengikuti
  //     angka itu berarti berjalan ke arah acak.
  //
  //  2. Bola "terlihat" tapi kepalanya TIDAK BERGERAK sama sekali. Ini yang
  //     benar-benar terjadi 2026-08-31: selama 12 detik penuh servo head_pan
  //     terbaca 175,2 deg persis di setiap baris log, simpangannya tetap
  //     +10,2 deg, dan robot menggeser 3,1 mm ke arah yang sama berulang-ulang
  //     tanpa satu pun bacaan baru. Lingkar tertutup yang umpan baliknya beku
  //     itu bukan lingkar tertutup; itu berjalan buta.
  //
  // Pemeriksaan basi hanya berlaku kalau simpangannya memang di luar pita
  // mati. Bola yang sudah tepat di depan kaki memang tidak menggerakkan kepala,
  // dan itu bukan kerusakan.
  const bool bearing_stale =
      (fabs(bearing_deg) > reposition_deadband_deg_) &&
      (std::chrono::duration<double>(now - reposition_pan_moved_at_).count() >
       reposition_stale_sec_);

  if (ball_visible == false || bearing_stale == true)
  {
    if (reposition_blind_ == false)
    {
      reposition_blind_ = true;
      reposition_blind_since_ = now;
    }

    const double blind_sec = std::chrono::duration<double>(now - reposition_blind_since_).count();

    // Robot berdiri diam, bukan berjalan berdasarkan bacaan lama. Parameter
    // dasar dikembalikan dulu supaya yang tertulis di walking_module bukan
    // preset GESER beramplitudo nol -- init pose dan period-nya masih milik
    // gerakan menyamping, dan itu yang akan dipakai langkah berikutnya.
    restoreBaseWalkingParam();
    setWalkingParam(0.0, 0.0, 0.0);
    shift_announced_ = -1;

    if (blind_sec >= reposition_blind_sec_)
    {
      return abortKickAlignment((ball_visible == false)
                                  ? "bola tidak terlihat lagi"
                                  : "sudut leher tidak bergerak, umpan balik basi");
    }

    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BallFollower"), *log_clock_, 1000,
                         "menggeser DITAHAN: %s. Diam di tempat %.1f dari %.1f s "
                         "sebelum membatalkan tendangan.",
                         (ball_visible == false) ? "bola tidak terlihat"
                                                 : "sudut leher beku, simpangan tidak diperbarui",
                         blind_sec, reposition_blind_sec_);
    return false;
  }

  reposition_blind_ = false;

  // Koreksi TANDA geseran, dinilai dari sudut leher itu sendiri.
  //
  // Bola di kiri jendela -> robot bergeser ke KIRI: bergeser ke kiri membuat
  // bola terlihat lebih ke kanan, jadi simpangannya mengecil. Itu benar KALAU
  // y_move_amplitude positif memang membawa badan ke kiri -- konvensi yang
  // dipakai di seluruh berkas ini tapi tidak pernah diukur langsung. Kalau
  // ternyata terbalik, simpangan akan MEMBESAR, dan di sinilah ia ketahuan.
  //
  // Jendela penilaian baru dipasang saat simpangannya memang cukup besar; di
  // bawah itu geserannya kecil sekali dan yang terbaca cuma derau.
  if (reposition_sign_settled_ == false)
  {
    if (reposition_sign_ref_deg_ < 0.0)
    {
      if (fabs(bearing_deg) >= reposition_sign_min_error_deg_)
      {
        reposition_sign_ref_deg_ = fabs(bearing_deg);
        reposition_sign_check_at_ = now + secondsToSteady(reposition_sign_check_sec_);
      }
    }
    else if (now >= reposition_sign_check_at_)
    {
      if (fabs(bearing_deg) > reposition_sign_ref_deg_ + reposition_sign_margin_deg_)
      {
        reposition_y_sign_ *= -1.0;
        reposition_sign_settled_ = true;
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "ARAH GESER TERBALIK: sesudah %.1f s menggeser, simpangan bola justru "
                     "membesar %.1f -> %.1f deg. Tanda dibalik jadi %+.0f. "
                     "Pasang permanen dengan kick_reposition_y_sign:=%.1f",
                     reposition_sign_check_sec_, reposition_sign_ref_deg_, fabs(bearing_deg),
                     reposition_y_sign_, reposition_y_sign_);
      }
      else if (fabs(bearing_deg) < reposition_sign_ref_deg_ - reposition_sign_margin_deg_)
      {
        reposition_sign_settled_ = true;
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                    "Arah geser benar (%+.0f): simpangan bola turun %.1f -> %.1f deg "
                    "dalam %.1f s.",
                    reposition_y_sign_, reposition_sign_ref_deg_, fabs(bearing_deg),
                    reposition_sign_check_sec_);
      }
      else
      {
        // Belum jelas -- bisa jadi robot memang belum sempat bergerak. Jendela
        // diulur sekali lagi alih-alih memutuskan dari perubahan sebesar derau.
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"), *log_clock_, 2000,
                             "Arah geser belum bisa dinilai: simpangan %.1f -> %.1f deg "
                             "(butuh beda di atas %.1f). Jendela diulur.",
                             reposition_sign_ref_deg_, fabs(bearing_deg),
                             reposition_sign_margin_deg_);
      }

      reposition_sign_ref_deg_ = fabs(bearing_deg);
      reposition_sign_check_at_ = now + secondsToSteady(reposition_sign_check_sec_);
    }
  }

  // PITA MATI. Bola yang sudah cukup tepat di depan kaki tidak digeser lagi:
  // tanpa ini robot menggeser bolak-balik mengejar derau beberapa derajat, dan
  // setiap geseran ikut merusak yaw yang baru saja diluruskan.
  const bool need_shift = (fabs(bearing_deg) > reposition_deadband_deg_);

  double y_move = 0.0;
  double x_move = 0.0;
  const char *how = "";

  if (shiftPresetActive() == true)
  {
    // ---- Menyesuaikan kaki memakai preset GESER KIRI / GESER KANAN --------
    //
    // Presetnya dikirim APA ADANYA, tidak diskalakan simpangan -- alasan yang
    // sama persis dengan preset PUTAR: operator sudah menyetel gerakannya
    // langsung di robot lewat Bascorro Studio, dan mengalikan amplitudo hasil
    // setelan itu dengan simpangan sama saja dengan membuang setelannya.
    //
    // Yang menghentikan geseran tetap pita mati di atas dan jendela tendang di
    // awal fungsi ini.
    if (need_shift == true)
    {
      // Bola di KIRI jendela tendang -> robot bergeser ke KIRI, karena itu yang
      // membuat bola terlihat bergerak ke kanan menuju tengah jendela.
      // reposition_y_sign_ menukar keduanya kalau di robot ini ternyata
      // terbalik; penilaiannya ada di blok koreksi tanda di atas.
      const bool use_left = ((bearing_deg * reposition_y_sign_) > 0.0);

      publishPreset(use_left ? preset_shift_left_ : preset_shift_right_);

      if (shift_announced_ != (use_left ? 1 : 0))
      {
        shift_announced_ = use_left ? 1 : 0;
        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Preset MENYESUAIKAN dipilih: %s (bola %+.1f deg dari tengah "
                    "jendela kaki %s). y %+.1f mm, x %+.1f mm, sudut %+.3f rad, "
                    "period %.2f s.",
                    use_left ? "GESER KIRI" : "GESER KANAN",
                    bearing_deg, (target_foot == OnLeft) ? "KIRI" : "KANAN",
                    current_walking_param_.y_move_amplitude * 1000.0,
                    current_walking_param_.x_move_amplitude * 1000.0,
                    current_walking_param_.angle_move_amplitude,
                    current_walking_param_.period_time);
      }

      how = (use_left == true) ? "preset GESER KIRI" : "preset GESER KANAN";
    }
    else
    {
      // Sudah di dalam pita mati: berdiri diam dengan gait jalan lagi, bukan
      // dengan gait geser beramplitudo nol. Kalau presetnya dibiarkan tertulis,
      // langkah berikutnya (dan tendangannya) berjalan dengan init pose geser.
      restoreBaseWalkingParam();
      setWalkingParam(0.0, 0.0, 0.0);
      shift_announced_ = -1;
      how = "sudah di dalam pita mati, diam";
    }

    y_move = current_walking_param_.y_move_amplitude;
    x_move = current_walking_param_.x_move_amplitude;
    reposition_x_ramp_ = 0.0;
  }
  else
  {
    // ---- Jalur lama: geseran sebanding simpangan --------------------------
    // Dipakai kalau preset geser tidak ada di yaml atau dimatikan lewat
    // kick_reposition_use_preset:=0.0.
    if (need_shift == true)
    {
      y_move = bearing_deg * reposition_y_gain_ * reposition_y_sign_;
      y_move = fmin(y_move, reposition_y_max_);
      y_move = fmax(y_move, -reposition_y_max_);
    }

    // Maju HANYA kalau bolanya memang masih jauh, dan merambat naik dari nol
    // dengan laju ramp yang sama seperti saat mengejar bola -- supaya telapak
    // tidak menyodok bola yang sudah ada di dekat kaki.
    if (radius_ok == false)
    {
      reposition_x_ramp_ = fmin(reposition_x_ramp_ + unit_fb_step_, reposition_x_max_);
      x_move = reposition_x_ramp_;
    }
    else
    {
      reposition_x_ramp_ = 0.0;
    }

    // Sudut putar DIKUNCI NOL. Yaw baru saja diluruskan dan diverifikasi saat
    // badan diam, dan jalur ini tidak boleh merusaknya lagi.
    setWalkingParam(x_move, y_move, 0.0);
    how = need_shift ? "geseran sebanding simpangan" : "sudah di dalam pita mati, diam";
  }

  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"),
                       *log_clock_, 500,
                       "menggeser ke kaki %s (%s): servo head_pan %.1f -> target %.1f deg "
                       "(simpang %+.1f, pita mati %.1f) | geser %+.1f mm, maju %+.1f mm "
                       "| radius %.1f px (ambang %.0f, %s) | yaw bergeser %+.1f deg "
                       "| siap %d/%d | sisa %.1f s",
                       (target_foot == OnLeft) ? "KIRI" : "KANAN", how,
                       pan_now_deg, target_deg, bearing_deg, reposition_deadband_deg_,
                       y_move * 1000.0, x_move * 1000.0,
                       ball_radius_px, kick_ball_radius_px_,
                       radius_ok ? "cukup dekat" : "masih jauh",
                       turned_deg_, reposition_ready_count_, kick_ready_count_,
                       std::chrono::duration<double>(reposition_deadline_ - now).count());

  return false;
}

bool BallFollower::finishReposition(const char *reason)
{
  gyro_integrating_ = false;

  // Dihitung sebelum restore, alasannya sama persis dengan di finishYawTurn():
  // yang harus mereda itu goyangan gait GESER (0,55 s), bukan gait jalan.
  const double settle_sec = settleWaitSec(yaw_settle_sec_);

  // Sesudah ini kaki penendang TIDAK boleh dihitung ulang lagi. Fase ini
  // bekerja membawa bola ke depan kaki tertentu; menghitung ulang dari sudut
  // leher beberapa detik kemudian (waktu kepala sudah dipegang action_module)
  // hanya bisa membatalkan hasil kerjanya sendiri.
  foot_from_reposition_ = (approach_ball_position_ == OnLeft ||
                           approach_ball_position_ == OnRight);

  restoreBaseWalkingParam();
  setWalkingParam(0.0, 0.0, 0.0);
  publishWalkingCommand("stop");

  yaw_align_state_ = YawAlignPostSettle;
  yaw_settle_until_ = std::chrono::steady_clock::now() + secondsToSteady(settle_sec);
  last_settle_sec_ = settle_sec;

  // Pemeriksaan yaw sesudah ini harus memakai sampel dari badan yang sudah
  // berhenti menggeser, bukan campuran keduanya.
  resetYawFilter();

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "Geseran SELESAI (%s); kaki penendang %s. Yaw bergeser %+.1f deg selama "
              "menggeser. Menunggu %.1f s sampai badan diam, lalu yaw diperiksa lagi.",
              reason,
              (approach_ball_position_ == OnLeft)
                  ? "KIRI"
                  : ((approach_ball_position_ == OnRight) ? "KANAN" : "belum diputuskan"),
              turned_deg_, settle_sec);

  return false;
}

void BallFollower::cancelYawAlign()
{
  if (yaw_align_state_ == YawAlignOff)
    return;

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"), "Penyelarasan yaw DIBATALKAN");

  yaw_align_state_ = YawAlignOff;
  yaw_in_window_count_ = 0;

  // Keputusan kaki penendang ikut dibuang. Membatalkan penyelarasan berarti
  // membatalkan tendangan ITU, dan approach_ball_position_ masih menyimpan
  // OnLeft/OnRight dari saat pemicu menyala. Kalau dibiarkan, tick berikutnya
  // membaca isBallInRange() == true lagi dan SoccerDemo langsung memulai
  // penyelarasan yang baru saja dibatalkan -- persis yang terjadi kalau
  // operator menekan STOP di tengah putaran.
  clearBallPosition();

  gyro_integrating_ = false;

  // Parameter jalan dikembalikan seperti sebelum mengitari. Tanpa ini preset
  // PUTAR tetap tertulis di walking_module dan robot mengejar bola berikutnya
  // dengan gait mengitari -- termasuk turn_in-nya.
  restoreBaseWalkingParam();

  // Amplitudo putar WAJIB dinolkan di sini. Tanpa ini, angle_move_amplitude
  // terakhir tetap tersimpan di walking_module dan robot langsung berputar lagi
  // begitu "start" berikutnya dikirim -- termasuk saat operator menekan START
  // untuk mulai mengejar bola lagi.
  setWalkingParam(0.0, 0.0, 0.0);
  publishWalkingCommand("stop");
}

bool BallFollower::getWalkingParam()
{
  auto temp_node = rclcpp::Node::make_shared("ballfollower_get_walking_param");
  auto get_walking_param_client_ = temp_node->create_client<op3_walking_module_msgs::srv::GetWalkingParam>("/robotis/walking/get_params");
  auto request = std::make_shared<op3_walking_module_msgs::srv::GetWalkingParam::Request>();

  // 200 ms timeout (was 1 s). walking_module's service is already advertised
  // by the time op3_manager is up; 1 s was just dead latency added to every
  // START press. If service legitimately isn't available, the fallback path
  // in startFollowing() kicks in with sensible defaults.
  if (!get_walking_param_client_->wait_for_service(std::chrono::milliseconds(200)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "BallFollower::getWalkingParam - Service not available");
    return false;
  }

  auto future = get_walking_param_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future, std::chrono::milliseconds(1500)) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    if (result)
    {
      current_walking_param_ = result->parameters;

      // BATAS ATAS tinggi angkat kaki diambil dari sini, yaitu foot_height di
      // param.yaml -- TAPI HANYA SEKALI, dan ini wajib.
      //
      // setWalkingParam() MENERBITKAN z hasil penskalaan ke walking_module.
      // Jadi begitu robot pernah berjalan pelan, isi walking_module bukan lagi
      // foot_height asli melainkan angka yang sudah dikecilkan. Kalau batas
      // atas ditangkap ulang tiap getWalkingParam(), ia menyusut sendiri lewat
      // umpan balik: 37 -> 18,5 -> 9,25 mm, satu tingkat tiap kali demo
      // dimulai lagi sesudah menendang.
      //
      // Persis itu yang dilaporkan operator: ramp x dan z rapi di awal, lalu
      // kacau sesudah tendangan pertama.
      //
      // Konsekuensinya foot_height yang diubah lewat Bascorro Studio di tengah
      // sesi tidak ikut terbaca; jalankan ulang demo supaya terbaca lagi.
      if (z_move_max_captured_ == false && current_walking_param_.z_move_amplitude > 0.0)
      {
        z_move_max_ = current_walking_param_.z_move_amplitude;
        z_move_max_captured_ = true;
        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Batas atas tinggi angkat kaki dikunci di %.1f mm (foot_height param.yaml).",
                    z_move_max_ * 1000.0);
      }

      if (DEBUG_PRINT)
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Get walking parameters");
      return true;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Fail to get walking parameters.");
    }
  }

  return false;
}

}

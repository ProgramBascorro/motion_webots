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
    head_tilt_sign_(-1.0),   // head_tilt POSITIF = MENUNDUK (diukur, lihat catatan di bawah)
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
    z_move_max_(0.037),                 // foot_height param.yaml; ditimpa getWalkingParam()
    z_move_min_ratio_(0.5),
    kick_ready_count_(10),
    NOT_FOUND_THRESHOLD(50),
    MAX_FB_STEP(12.0 * 0.001),
    MAX_RL_TURN(15.0 * M_PI / 180),
    IN_PLACE_FB_STEP(-3.0 * 0.001),
    MIN_FB_STEP(5.0 * 0.001),
    MIN_RL_TURN(5.0 * M_PI / 180),
    UNIT_FB_STEP(0.3 * 0.001),
    UNIT_RL_TURN(0.5 * M_PI / 180),
    SPOT_FB_OFFSET(0.0 * 0.001),
    SPOT_RL_OFFSET(0.0 * 0.001),
    SPOT_ANGLE_OFFSET(0.0),
    hip_pitch_offset_(7.0),
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
    refresh_foot_after_align_(true)
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

void BallFollower::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ != nullptr)
  {
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

      double value = fallback;
      node_->get_parameter(name, value);
      return value;
    };
    camera_height_  = read_param("kick_camera_height", camera_height_);
    kick_distance_  = read_param("kick_distance", kick_distance_);
    head_tilt_sign_ = read_param("kick_head_tilt_sign", head_tilt_sign_);

    kick_pan_right_min_deg_ = read_param("kick_pan_right_min_deg", kick_pan_right_min_deg_);
    kick_pan_right_max_deg_ = read_param("kick_pan_right_max_deg", kick_pan_right_max_deg_);
    kick_pan_left_min_deg_  = read_param("kick_pan_left_min_deg",  kick_pan_left_min_deg_);
    kick_pan_left_max_deg_  = read_param("kick_pan_left_max_deg",  kick_pan_left_max_deg_);
    head_pan_servo_center_deg_ = read_param("head_pan_servo_center_deg", head_pan_servo_center_deg_);
    head_pan_servo_dir_        = read_param("head_pan_servo_dir", head_pan_servo_dir_);
    kick_pan_max_distance_     = read_param("kick_pan_max_distance", kick_pan_max_distance_);
    kick_ball_radius_px_       = read_param("kick_ball_radius_px", kick_ball_radius_px_);
    kick_ball_real_radius_m_   = read_param("kick_ball_real_radius_m", kick_ball_real_radius_m_);
    kick_fov_width_deg_        = read_param("kick_fov_width_deg", kick_fov_width_deg_);
    kick_image_width_px_       = read_param("kick_image_width_px", kick_image_width_px_);
    kick_ball_radius_margin_   = read_param("kick_ball_radius_margin", kick_ball_radius_margin_);
    z_move_min_ratio_          = read_param("walk_z_move_min_ratio", z_move_min_ratio_);

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

    // Tanda putaran dipasang SEKALI di sini, bukan tiap kali menyelaraskan:
    // hasil koreksi otomatis (kalau sampai terjadi) berlaku untuk sisa run,
    // jadi tendangan kedua dan seterusnya sudah berputar ke arah yang benar
    // sejak detik pertama tanpa perlu meluncurkan ulang demo.
    yaw_turn_sign_ = yaw_turn_sign_param_;
    yaw_sign_settled_ = (yaw_auto_sign_ == false);

    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Kick geometry: camera_height=%.3f m, kick_distance=%.3f m, head_tilt_sign=%+.0f",
                camera_height_, kick_distance_, head_tilt_sign_);
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Kick pan window: kanan %.1f..%.1f deg, kiri %.1f..%.1f deg "
                "(servo head_pan, pusat %.1f, arah %+.0f), maks %.2f m, butuh %d siklus",
                kick_pan_right_min_deg_, kick_pan_right_max_deg_,
                kick_pan_left_min_deg_, kick_pan_left_max_deg_,
                head_pan_servo_center_deg_, head_pan_servo_dir_,
                kick_pan_max_distance_, kick_ready_count_);
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Kick radius bola: >= %.1f px (%s)", kick_ball_radius_px_,
                (kick_ball_radius_px_ > 0.0) ? "aktif" : "MATI");
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Penyelarasan yaw sebelum menendang: %s | target %+.1f +-%.1f deg "
                "| putar %.1f..%.1f deg (gain %.2f, tanda %+.0f, koreksi otomatis %s) "
                "| batas waktu %s, reda %.1f s, butuh %d siklus "
                "| batas bawah naik %.1f deg tiap %.1f s kalau macet "
                "| kaki dihitung ulang sesudah memutar: %s",
                yaw_align_enable_ ? "AKTIF" : "MATI",
                yaw_target_deg_, yaw_tolerance_deg_,
                yaw_min_turn_deg_, yaw_max_turn_deg_, yaw_turn_gain_,
                yaw_turn_sign_param_, yaw_auto_sign_ ? "ya" : "tidak",
                (yaw_timeout_sec_ > 0.0) ? "ada" : "TANPA BATAS (tendang hanya kalau sudah lurus)",
                yaw_settle_sec_, yaw_ready_count_,
                yaw_turn_step_deg_, yaw_stall_sec_,
                refresh_foot_after_align_ ? "ya" : "tidak");

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

  // setWalkingCommand("start") already calls getWalkingParam() internally and
  // populates current_walking_param_ before publishing the "start" command.
  // We READ THE CACHED VALUES instead of calling getWalkingParam() a second
  // time — the original code did a back-to-back service call that added
  // ~service-roundtrip + up-to-1s wait_for_service to the START button
  // latency. period_time and hip_pitch_offset aren't touched by
  // setWalkingParam(), so the cached values are valid.
  setWalkingCommand("start");

  if (current_walking_param_.period_time > 0.1)
  {
    hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
    curr_period_time_ = current_walking_param_.period_time;
  }
  else
  {
    // Service unavailable or returned garbage — fall back to known defaults.
    hip_pitch_offset_ = 7.0 * M_PI / 180;
    curr_period_time_ = 0.6;
  }
}

void BallFollower::stopFollowing()
{
  on_tracking_ = false;
  count_to_kick_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Stop Ball following");

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
  if (get_count != 2)
    return;

  current_pan_ = pan;
  current_tilt_ = tilt;
}

void BallFollower::calcFootstep(double target_distance, double target_angle, double delta_time,
                                double& fb_move, double& rl_angle)
{
  // clac fb
  double next_movement = current_x_move_;
  if (target_distance < 0)
    target_distance = 0.0;

  double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
  accum_period_time_ += delta_time;
  if (accum_period_time_ > (curr_period_time_  / 4))
  {
    accum_period_time_ = 0.0;
    if ((target_distance * 0.1 / 2) < current_x_move_)
      next_movement -= UNIT_FB_STEP;
    else
      next_movement += UNIT_FB_STEP;
  }
  fb_goal = fmin(next_movement, fb_goal);
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

// x_angle : ball position (pan), y_angle : ball position (tilt), ball_size : angle of ball radius
double BallFollower::headPanServoDeg() const
{
  return head_pan_servo_center_deg_ + head_pan_servo_dir_ * current_pan_ * 180.0 / M_PI;
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

  // check of getting head joints angle
  if (current_tilt_ == -10 && current_pan_ == -10)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Failed to get current angle of head joints.");
    setWalkingCommand("stop");

    on_tracking_ = false;
    approach_ball_position_ = NotFound;
    return false;
  }

  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "   ============== Head | Ball ==============   ");
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Pan : %f | Ball X : %f", (current_pan_ * 180 / M_PI), (x_angle * 180 / M_PI));
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Tilt : %f | Ball Y : %f", (current_tilt_ * 180 / M_PI), (y_angle * 180 / M_PI));
  }

  approach_ball_position_ = OutOfRange;

  // Rumus aslinya menganggap head_tilt NEGATIF saat menunduk (konvensi OP3
  // asli): dengan hip_pitch_offset 0 ia menjadi H*cot(sudut tunduk) = jarak
  // bola yang sebenarnya, persis.
  //
  // Robot ini TERBALIK dari konvensi OP3 asli: head_tilt POSITIF = MENUNDUK.
  //
  // Ini DIUKUR, bukan disimpulkan. Cara mengukurnya (30 detik, aman, cuma
  // menggerakkan kepala) -- hentikan pelacak lalu perintahkan dua sudut mutlak
  // berturut-turut DALAM RUN YANG SAMA, ambil satu frame kamera di tiap sudut:
  //     head_tilt -25,8 deg -> kamera MENDONGAK (langit-langit/ruangan)
  //     head_tilt +25,8 deg -> kamera MENUNDUK  (lantai, bola, badan sendiri)
  //
  // Harus dalam run yang sama dan berselang detik. Membandingkan foto dari dua
  // run berbeda pernah menyesatkan sampai tanda ini dibalik keliru: bolanya
  // sudah dipindah di antara kedua run, jadi "sama-sama kelihatan lantai"
  // tidak membuktikan apa pun. Jangan pula percaya komentar di
  // head_tracking.yaml atau kesan arah gerak kepala -- keduanya pernah salah.
  //
  // head_tilt_sign_ = -1 mengembalikan sudutnya ke konvensi yang diasumsikan
  // rumus. Setel kick_head_tilt_sign := 1.0 untuk robot konvensi asli.
  //
  // head_tilt_sign_ = -1 mengembalikan sudutnya ke konvensi yang diasumsikan
  // rumus. Setel kick_head_tilt_sign := 1.0 untuk robot yang masih konvensi
  // asli.
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
  const double tilt_for_geometry = head_tilt_sign_ * current_tilt_ + y_angle;
  double distance_to_ball = camera_height_ * tan(M_PI * 0.5 + tilt_for_geometry - hip_pitch_offset_ - ball_size);

  double ball_y_angle = (current_tilt_ + y_angle) * 180 / M_PI;
  double ball_x_angle = (current_pan_ + x_angle) * 180 / M_PI;

  if (distance_to_ball < 0)
    distance_to_ball *= (-1);

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
  const bool pan_ready = pan_side_ok && (distance_to_ball < kick_pan_max_distance_);
  const bool near_enough = (distance_to_ball < distance_to_kick) && (fabs(ball_x_angle) < 25.0);

  //  (3) Jari-jari bola di gambar. Pengukur jarak yang berdiri sendiri, dan
  //      paling sedikit asumsinya: tidak peduli tinggi kamera, hip_pitch,
  //      tanda head_tilt, atau apakah kepala menunjuk ke bola. Bola dekat
  //      SELALU besar. Gerbang +-25 derajat tetap dipakai di sini (beda dengan
  //      jalur jendela pan) supaya bola yang dekat tapi jauh di samping tidak
  //      ikut memicu.
  const bool radius_ready = (kick_ball_radius_px_ > 0.0)
                         && (ball_radius_px >= kick_ball_radius_px_)
                         && (fabs(ball_x_angle) < 25.0);

  // Selalu dicetak (1x/detik) supaya ambangnya bisa ditera langsung di robot:
  // dekatkan bola sampai jaraknya turun di bawah ambang, atau sampai servo
  // head_pan masuk salah satu jendela.
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"),
                       *log_clock_, 1000,
                       "jarak bola %.3f m (ambang %.3f) | servo head_pan %.1f deg "
                       "(kanan %.0f-%.0f, kiri %.0f-%.0f) | head_tilt %+.1f deg "
                       "| bola dlm gambar %+.1f deg | radius %.1f px (ambang %.0f) "
                       "| bola x %+.1f deg | siap: %s",
                       distance_to_ball, distance_to_kick, head_pan_servo_deg,
                       kick_pan_right_min_deg_, kick_pan_right_max_deg_,
                       kick_pan_left_min_deg_, kick_pan_left_max_deg_,
                       current_tilt_ * 180 / M_PI, y_angle * 180 / M_PI,
                       ball_radius_px, kick_ball_radius_px_, ball_x_angle,
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
  double distance_to_walk = distance_to_ball - distance_to_kick;

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

  // Jendela servo head_pan lebih dipercaya daripada tanda sudut: ambangnya
  // 0,05 rad (2,9 derajat) dan jendela KIRI operator (servo 182-184 = +2..+4
  // derajat) duduk tepat di atasnya -- sedikit saja meleset, kakinya tertukar.
  const int foot_from_pan = kickFootFromHeadPan();
  if (foot_from_pan == OnRight || foot_from_pan == OnLeft)
  {
    approach_ball_position_ = foot_from_pan;
    return;
  }

  double ball_x_angle = current_pan_ + x_angle;

  if (ball_x_angle > 0.05)
    approach_ball_position_ = OnLeft;
  else
    approach_ball_position_ = OnRight;
}

void BallFollower::waitFollowing()
{
  count_not_found_++;

  if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
    setWalkingParam(0.0, 0.0, 0.0);
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
    setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
  }

  publishWalkingCommand(command);
}

void BallFollower::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
  current_walking_param_.balance_enable = balance;
  current_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET;
  current_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET;
  current_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

  // Tinggi angkat kaki naik BERTAHAP bersama besar gerakan, dengan foot_height
  // di param.yaml sebagai BATAS ATAS -- bukan lagi nilai tetap yang dipakai
  // sejak langkah pertama.
  //
  // Serasi dengan x_move_amplitude: x sendiri sudah dinaikkan bertahap oleh
  // calcFootstep() dalam kelipatan UNIT_FB_STEP sampai MAX_FB_STEP, jadi
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
  // (5,0 mm) dan menaikkannya bertahap sampai MAX_FB_STEP (18,1 mm). Kalau z
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

  double motion = ramp_progress(x_move, MIN_FB_STEP, MAX_FB_STEP);

  // Putaran dihitung dengan rentangnya sendiri lalu diambil yang terbesar.
  // Tanpa ini, fase memutar di tempat (x = 0, hanya angle) jatuh ke kemajuan
  // nol dan kaki tidak cukup terangkat untuk memutar.
  motion = fmax(motion, ramp_progress(rotation_angle, MIN_RL_TURN, MAX_RL_TURN));

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
//  Penyelarasan yaw sebelum menendang
// =========================================================================
//
// Dipanggil dari SoccerDemo::process(), satu langkah per tick (~30 Hz).
// Rangkaiannya:
//
//   startYawAlign()   sekali, saat bola baru masuk jangkauan tendang
//        |            -> false: tidak perlu / tidak bisa, tendang seperti biasa
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
  // Tidak ada kunci di sini dan memang tidak perlu: callback IMU dan process()
  // sama-sama dijalankan utas utama demo_node (process() lalu spin_some()).
  body_yaw_deg_ = yaw_deg;
  body_yaw_stamp_ = std::chrono::steady_clock::now();
  body_yaw_seen_ = true;
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
}

void BallFollower::startInPlaceTurn()
{
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

  if (yaw_align_enable_ == false)
    return false;

  if (isBodyYawUsable() == false)
  {
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Penyelarasan yaw DILEWATI: yaw tidak bisa dipakai "
                "(mag_status %d, yaw %s). Langsung menendang seperti sebelumnya.",
                mag_status_,
                body_yaw_seen_ ? "pernah terbaca tapi sudah basi" : "belum pernah terbaca");
    return false;
  }

  const double error = normalizeAngleDeg(yaw_target_deg_ - body_yaw_deg_);

  if (fabs(error) <= yaw_tolerance_deg_)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                "Yaw sudah lurus: %+.1f deg (target %+.1f +-%.1f). Langsung menendang.",
                body_yaw_deg_, yaw_target_deg_, yaw_tolerance_deg_);
    return false;
  }

  const auto now = std::chrono::steady_clock::now();

  // Masuk fase MENGEREM dulu, bukan langsung memutar. Semua penghitung waktu
  // putaran (tenggat, macet, cek tanda) sengaja BELUM disetel di sini; mereka
  // baru mulai saat putaran benar-benar dimulai, supaya waktu mengerem tidak
  // ikut termakan tenggat.
  yaw_align_state_ = YawAlignBraking;
  yaw_brake_until_ = now + secondsToSteady(yaw_brake_sec_);
  yaw_brake_deadline_ = now + secondsToSteady(fmax(yaw_brake_max_sec_, yaw_brake_sec_));
  yaw_brake_sample_at_ = yaw_brake_until_;
  yaw_brake_last_yaw_ = body_yaw_deg_;
  yaw_recheck_left_ = yaw_recheck_max_;
  yaw_runaway_ref_error_ = fabs(error);
  yaw_align_ref_error_ = fabs(error);
  // Batas bawah putaran mulai dari nilai parameter tiap kali menyelaraskan,
  // BUKAN dari hasil eskalasi tendangan sebelumnya: torsi yang dibutuhkan
  // tergantung keadaan saat itu (digantung, lantai licin, baterai lemah),
  // jadi memulai lagi dari yang paling halus adalah pilihan yang lebih aman.
  yaw_turn_floor_deg_ = yaw_min_turn_deg_;
  // yaw_turn_sign_ dan yaw_sign_settled_ SENGAJA tidak disetel ulang di sini:
  // arah putaran itu sifat robot, bukan sifat satu tendangan. Sekali terukur,
  // dipakai seterusnya.

  pauseFollowingForTurn();
  brakeBeforeTurn();

  RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
              "BERHENTI dulu (%.1f s) lalu MEMUTAR sebelum menendang: "
              "yaw %+.1f -> target %+.1f deg (galat %+.1f, toleransi %.1f). %s",
              yaw_brake_sec_, body_yaw_deg_, yaw_target_deg_, error, yaw_tolerance_deg_,
              (yaw_timeout_sec_ > 0.0)
                  ? "Ada batas waktu; sesudahnya tetap menendang."
                  : "TIDAK menendang sampai yaw benar-benar masuk toleransi.");

  return true;
}

bool BallFollower::finishYawTurn(const char *reason)
{
  // Amplitudo dinolkan SEBELUM "stop", urutan yang sama dengan stopFollowing():
  // walking_module mengecil dari amplitudo ke nol pada siklus berikutnya alih-
  // alih menyelesaikan langkah putar terakhirnya.
  setWalkingParam(0.0, 0.0, 0.0);
  publishWalkingCommand("stop");

  yaw_align_state_ = YawAlignSettling;
  yaw_settle_until_ = std::chrono::steady_clock::now() + secondsToSteady(yaw_settle_sec_);
  yaw_in_window_count_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
              "Putaran dihentikan (%s); menunggu %.1f s sampai gait reda, lalu menendang.",
              reason, yaw_settle_sec_);

  // Belum selesai: fase reda diselesaikan tick berikutnya.
  return false;
}

bool BallFollower::processYawAlign()
{
  if (yaw_align_state_ == YawAlignOff)
    return true;

  const auto now = std::chrono::steady_clock::now();

  if (yaw_align_state_ == YawAlignBraking)
  {
    // Tahan amplitudo di nol sampai robot benar-benar berhenti. Tanpa fase
    // ini robot masih melangkah maju saat mulai memutar dan kakinya menabrak
    // bola yang tepat ada di depannya.
    setWalkingParam(0.0, 0.0, 0.0);

    if (now < yaw_brake_until_)
      return false;

    // Sesudah lama minimum terlewat, robot belum tentu benar-benar diam.
    // Terukur 2026-08-28: selama mengerem 0,8 detik yaw masih bergeser
    // -20,6 -> -16,2 (4,4 derajat). Kalau putaran dimulai dari angka yang
    // masih bergerak, galat awalnya salah sejak detik pertama.
    //
    // Jadi tunggu sampai yaw benar-benar tenang: dua sampel berturut yang
    // bedanya di bawah yaw_brake_still_deg_. Ada batas atas waktu supaya
    // robot tidak menunggu selamanya kalau lantainya bergetar.
    if (now < yaw_brake_deadline_)
    {
      if (now < yaw_brake_sample_at_)
        return false;

      const double moved = fabs(normalizeAngleDeg(body_yaw_deg_ - yaw_brake_last_yaw_));
      yaw_brake_last_yaw_ = body_yaw_deg_;
      yaw_brake_sample_at_ = now + secondsToSteady(0.25);

      if (moved > yaw_brake_still_deg_)
      {
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"), *log_clock_, 500,
                             "mengerem: yaw masih bergeser %.1f deg (batas diam %.1f), "
                             "menunggu robot benar-benar berhenti",
                             moved, yaw_brake_still_deg_);
        return false;
      }
    }

    // Baru DI SINI putaran dimulai, dan baru di sini semua penghitung waktu
    // putaran dinyalakan -- supaya waktu mengerem tidak ikut termakan tenggat.
    const double start_error = normalizeAngleDeg(yaw_target_deg_ - body_yaw_deg_);

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
                body_yaw_deg_, start_error);

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
    // Di sini robot sudah berhenti, jadi angkanya baru berarti. Kalau ternyata
    // masih meleset, putar lagi. Kalau jatah ulangnya habis, tetap menendang
    // supaya demo tidak macet selamanya -- tapi dicatat dengan keras.
    if (isBodyYawUsable() == true)
    {
      const double settled_error = normalizeAngleDeg(yaw_target_deg_ - body_yaw_deg_);

      if (fabs(settled_error) > yaw_tolerance_deg_ && yaw_recheck_left_ > 0)
      {
        yaw_recheck_left_ -= 1;

        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Sesudah gait reda yaw ternyata masih %+.1f deg (galat %+.1f, "
                    "toleransi %.1f). Yang tadi terbaca 'lurus' itu derau saat "
                    "berjalan. Memutar lagi, sisa jatah %d.",
                    body_yaw_deg_, settled_error, yaw_tolerance_deg_, yaw_recheck_left_);

        yaw_align_state_ = YawAlignTurning;
        yaw_in_window_count_ = 0;
        yaw_align_ref_error_ = fabs(settled_error);
        yaw_stall_ref_error_ = fabs(settled_error);
        yaw_best_error_ = fabs(settled_error);
        yaw_turn_floor_deg_ = yaw_min_turn_deg_;
        yaw_stall_check_at_ = now + secondsToSteady(yaw_stall_sec_);
        // Tenggat dan titik acuan runaway WAJIB ikut disetel ulang. Tanpa ini,
        // tenggat 12 detik dari putaran pertama sudah lewat, jadi tick
        // berikutnya langsung menyerah lagi: terukur 2026-08-28, ketiga jatah
        // putar ulang habis dalam 2 detik total (498,280 -> 499,047 ->
        // 499,813 -> 500,580) tanpa robot sempat memutar sama sekali.
        yaw_align_started_at_ = now;
        yaw_align_deadline_ = now + secondsToSteady(yaw_timeout_sec_);
        yaw_runaway_ref_error_ = fabs(settled_error);
        startInPlaceTurn();
        return false;
      }

      if (fabs(settled_error) > yaw_tolerance_deg_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                     "Jatah putar ulang habis, yaw masih %+.1f deg (galat %+.1f). "
                     "Tetap menendang. Kalau ini sering terjadi, lebarkan "
                     "kick_yaw_tolerance_deg atau naikkan kick_yaw_recheck_max.",
                     body_yaw_deg_, settled_error);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                    "Yaw TERVERIFIKASI sesudah gait reda: %+.1f deg (galat %+.1f). Menendang.",
                    body_yaw_deg_, settled_error);
      }
    }

    yaw_align_state_ = YawAlignOff;
    return true;
  }

  // ---- YawAlignTurning ---------------------------------------------------
  if (isBodyYawUsable() == false)
  {
    // Yaw hilang. Memutar terus berarti memutar BUTA, dan menendang di sini
    // berarti menendang tanpa tahu arah -- dua-duanya salah. Jadi robot diam
    // di tempat (gait tetap jalan, amplitudo nol) dan menunggu yaw kembali.
    // Kalau tidak pernah kembali, robot memang berhenti di situ: itu keadaan
    // yang aman dan terlihat jelas, bukan tendangan ke arah acak.
    setWalkingParam(0.0, 0.0, 0.0);
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BallFollower"), *log_clock_, 2000,
                         "Yaw hilang di tengah penyelarasan (mag_status %d). "
                         "Menunggu di tempat; tidak menendang sampai yaw kembali. "
                         "Tekan STOP kalau ingin membatalkan.",
                         mag_status_);
    return false;
  }

  const double error = normalizeAngleDeg(yaw_target_deg_ - body_yaw_deg_);

  if (fabs(error) <= yaw_tolerance_deg_)
  {
    yaw_in_window_count_ += 1;
    if (yaw_in_window_count_ >= yaw_ready_count_)
    {
      RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                  "Yaw LURUS: %+.1f deg (target %+.1f +-%.1f, galat %+.1f).",
                  body_yaw_deg_, yaw_target_deg_, yaw_tolerance_deg_, error);
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
                "Tetap menendang karena kick_yaw_timeout_sec dipasang. Setel 0.0 "
                "supaya robot memutar sampai benar-benar lurus.",
                yaw_timeout_sec_, error);
    return finishYawTurn("batas waktu habis");
  }

  // Koreksi tanda otomatis, SEKALI per run (bukan per tendangan).
  //
  // Arah putaran positif walking_module belum pernah diukur terhadap tanda yaw
  // magnetometer di robot ini, dan menebak tanda sudah dua kali memakan waktu
  // berjam-jam di berkas ini (lihat catatan head_tilt di processFollowing).
  // Daripada menebak: kalau sesudah jendela pengamatan galatnya justru MEMBESAR
  // melewati margin, tandanya dibalik satu kali lalu dicatat dengan keras --
  // angka yang tercetak itulah yang dipasang permanen lewat kick_yaw_turn_sign.
  // Jendelanya harus lebih lama dari soft_start_time walking_module (bawaan
  // 2 detik), karena selama ramp itu amplitudo putar belum penuh dan galat
  // memang belum banyak bergerak.
  //
  // yaw_sign_settled_ TIDAK dinolkan lagi oleh startYawAlign(), jadi begitu
  // arahnya terukur sekali ia dipakai untuk seluruh tendangan sesudahnya.
  if (yaw_sign_settled_ == false && now >= yaw_sign_check_at_)
  {
    // Galat awal harus cukup BESAR sebelum penilaian ini sah.
    //
    // Terukur 2026-08-28: bacaan yaw sambil gait jalan berbeda 5-10 derajat
    // dari bacaan sesudah robot diam. Kalau galat awalnya cuma 5,8 derajat,
    // derau sebesar itu sendirian sudah bisa membuatnya terlihat "membesar
    // 12,9 derajat" -- dan tandanya dibalik padahal arah putarannya benar.
    // Itu persis yang terjadi: tanda -1 yang sudah terukur benar dibalik lagi
    // jadi +1 di tengah rangkaian cek-ulang bergalat kecil.
    //
    // Di bawah ambang ini penilaian DITUNDA, bukan diputuskan: yaw_sign_settled_
    // dibiarkan false supaya kesempatan menilai tetap ada pada penyelarasan
    // berikutnya yang galatnya cukup besar.
    if (yaw_align_ref_error_ < yaw_sign_min_error_deg_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                  "Penilaian arah putaran DITUNDA: galat awal %.1f deg masih di "
                  "bawah %.1f deg, deraunya lebih besar daripada sinyalnya.",
                  yaw_align_ref_error_, yaw_sign_min_error_deg_);
      yaw_sign_check_at_ = now + secondsToSteady(yaw_sign_check_sec_);
    }
    else if (fabs(error) > yaw_align_ref_error_ + yaw_sign_margin_deg_)
    {
      yaw_sign_settled_ = true;
      yaw_turn_sign_ *= -1.0;
      RCLCPP_ERROR(rclcpp::get_logger("BallFollower"),
                   "ARAH PUTARAN TERBALIK: galat naik %.1f -> %.1f deg dalam %.1f s. "
                   "Tanda dibalik jadi %+.0f. Pasang permanen dengan "
                   "kick_yaw_turn_sign:=%.1f",
                   yaw_align_ref_error_, fabs(error), yaw_sign_check_sec_,
                   yaw_turn_sign_, yaw_turn_sign_);
    }
    else
    {
      // Galat awalnya cukup besar DAN tidak membesar: arah putaran terbukti
      // benar. Dikunci supaya tidak dinilai ulang di penyelarasan bergalat
      // kecil, tempat derau bisa membalikkannya secara keliru.
      yaw_sign_settled_ = true;
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                  "Arah putaran TERBUKTI benar (%+.0f): galat %.1f -> %.1f deg "
                  "dalam %.1f s. Tidak akan dinilai ulang di run ini.",
                  yaw_turn_sign_, yaw_align_ref_error_, fabs(error), yaw_sign_check_sec_);
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

    if (stalled == true && yaw_turn_floor_deg_ < yaw_max_turn_deg_)
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

  setWalkingParam(0.0, 0.0, turn_rad);

  const double elapsed =
      std::chrono::duration<double>(now - yaw_align_started_at_).count();

  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"),
                       *log_clock_, 500,
                       "menyelaraskan yaw: sekarang %+.1f deg, target %+.1f, galat %+.1f "
                       "(toleransi %.1f) -> putar %+.1f deg/langkah "
                       "(batas bawah %.1f), sudah %.1f s",
                       body_yaw_deg_, yaw_target_deg_, error, yaw_tolerance_deg_,
                       turn_rad * 180.0 / M_PI, yaw_turn_floor_deg_, elapsed);

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
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    if (result)
    {
      current_walking_param_ = result->parameters;

      // BATAS ATAS tinggi angkat kaki diambil dari sini, yaitu foot_height di
      // param.yaml. Harus disimpan terpisah: setWalkingParam() menulisi
      // current_walking_param_.z_move_amplitude tiap tick dengan nilai hasil
      // penskalaan, jadi membacanya lagi dari situ akan memberi nilai yang
      // sudah dikecilkan, bukan batas atasnya.
      if (current_walking_param_.z_move_amplitude > 0.0)
        z_move_max_ = current_walking_param_.z_move_amplitude;

      if (DEBUG_PRINT)
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Get walking parameters");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Fail to get walking parameters.");
    }
  }

  return true;
}

}

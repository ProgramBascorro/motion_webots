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

/* Author: Kayman */

#include <filesystem>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "op3_walking_module/op3_walking_module.h"

namespace robotis_op
{

// Dasar stance dari penangkapan SAHIH terakhir (walking_bias_ + pose tertangkap).
// Dipakai ulang saat parameter reuse_walking_bias = true: demo D58 menyalakannya
// sesudah tendang/bangun supaya kaki langsung melandai ke stance init pose tanpa
// memainkan page 2 dulu. Tanpa ini, mengaktifkan modul dari pose akhir page akan
// menangkap pose itu sebagai dasar -> "gerakan terakhir jadi init pose".
// Sengaja static di .cpp, bukan anggota kelas: ukuran WalkingModule tidak berubah,
// jadi op3_manager (yang meng-getInstance() kelas ini) tidak perlu dikompilasi
// ulang. Hanya disentuh dari thread process(); flag-nya atomic.
static Eigen::MatrixXd g_last_walking_bias;
static Eigen::MatrixXd g_last_captured_pose;
static bool g_last_walking_bias_valid = false;
static std::atomic<bool> g_reuse_walking_bias(false);

// SOFT STOP (perintah "soft_stop", operator 2026-09-19): kebalikan soft start.
// Skala amplitudo x/y/z/angle turun MENERUS dari nilainya sekarang ke 0 selama
// soft_start_time, baru sesudah itu stop() biasa. "stop" lama TIDAK berubah.
// Static di .cpp (bukan anggota kelas) supaya ukuran WalkingModule tetap -> tidak
// perlu mengompilasi ulang op3_manager. Request dari thread callback (atomic),
// sisanya hanya disentuh thread process().
static std::atomic<bool> g_soft_stop_request(false);
static bool   g_soft_stop_active = false;
static double g_soft_stop_elapsed = 0.0;
static double g_soft_stop_duration = 0.0;
static double g_soft_stop_from = 1.0;

// AYUNAN LENGAN SAAT DIAM (operator 2026-09-19, dipakai demo D58): selagi walking
// idle, kedua bahu pitch berayun berlawanan seperti saat berjalan -- amplitudo
// idle_arm_swing_deg (derajat), satu ayunan penuh idle_arm_swing_period_sec.
// 0 = mati (bawaan), jadi Studio/D56 tidak berubah. Masuk/keluar dipudarkan 0,5 s
// supaya lengan tidak menyentak saat gait mulai/berhenti. Ditambahkan SESUDAH
// pembatas laju stance 25 deg/s (ayunan 10 deg / 1 s berpuncak ~63 deg/s dan akan
// terpotong kalau ikut dibatasi). Static di .cpp: ukuran kelas tetap.
static std::atomic<double> g_idle_arm_swing_deg(0.0);
static std::atomic<double> g_idle_arm_swing_period(1.0);
static double g_idle_swing_time = 0.0;
static double g_idle_swing_scale = 0.0;   // 0..1, dipudarkan
static double g_idle_swing_amp_deg = 0.0; // amplitudo terakhir yang > 0 (untuk memudar)
static double g_idle_swing_prev[2] = {0.0, 0.0};   // offset yang sudah terpasang (rad)
static const double IDLE_SWING_FADE_SEC = 0.5;

static const int BalancingPhase0 = 0; // DSP : START
static const int BalancingPhase1 = 1; // DSP : R--O->L
static const int BalancingPhase2 = 2; // SSP : L_BALANCING1
static const int BalancingPhase3 = 3; // SSP : L_BALANCING2
static const int BalancingPhase4 = 4; // DSP : R--O<-L
static const int BalancingPhase5 = 5; // DSP : R<-O--L
static const int BalancingPhase6 = 6; // SSP : R_BALANCING1
static const int BalancingPhase7 = 7; // SSP : R_BALANCING2
static const int BalancingPhase8 = 8; // DSP : R->O--L
static const int BalancingPhase9 = 9; // DSP : END


WalkingModule::WalkingModule()
  : Node("op3_walking_module"),
    control_cycle_msec_(8),
    DEBUG(false)
{
  enable_ = false;
  module_name_ = "walking_module";
  control_mode_ = robotis_framework::PositionControl;

  init_pose_count_ = 0;
  walking_state_ = WalkingReady;
  previous_x_move_amplitude_ = 0.0;

  op3_kd_ = new OP3KinematicsDynamics(WholeBody);

  // result
  result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["r_hip_roll"] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll"] = new robotis_framework::DynamixelState();

  result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["l_hip_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll"] = new robotis_framework::DynamixelState();

  result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  result_["l_sho_pitch"] = new robotis_framework::DynamixelState();

  // joint table
  joint_table_["r_hip_yaw"] = 0;
  joint_table_["r_hip_roll"] = 1;
  joint_table_["r_hip_pitch"] = 2;
  joint_table_["r_knee"] = 3;
  joint_table_["r_ank_pitch"] = 4;
  joint_table_["r_ank_roll"] = 5;

  joint_table_["l_hip_yaw"] = 6;
  joint_table_["l_hip_roll"] = 7;
  joint_table_["l_hip_pitch"] = 8;
  joint_table_["l_knee"] = 9;
  joint_table_["l_ank_pitch"] = 10;
  joint_table_["l_ank_roll"] = 11;

  joint_table_["r_sho_pitch"] = 12;
  joint_table_["l_sho_pitch"] = 13;

  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  init_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  captured_init_pose_ = Eigen::MatrixXd::Zero(1, result_.size());
  walking_bias_ = Eigen::MatrixXd::Zero(1, result_.size());
  capture_init_pose_ = false;
  walking_bias_valid_ = false;
  joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size());

  soft_start_time_ = 2.0;          // ditimpa param.yaml kalau kuncinya ada
  soft_start_request_ = false;
  soft_start_active_ = false;
  soft_start_elapsed_ = 0.0;
  soft_start_scale_ = 1.0;

  balancing_idx_ = BalancingPhase0;
}

WalkingModule::~WalkingModule()
{
  queue_thread_.join();
}

void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = std::thread(&WalkingModule::queueThread, this);
  control_cycle_msec_ = control_cycle_msec;

  // m, s, rad
  //
  // FALLBACK DARURAT. loadWalkingParam(param_path_) di akhir initialize()
  // menimpa SEMUA nilai di bawah dari config/param.yaml, jadi blok ini hanya
  // terpakai kalau yaml itu hilang atau rusak (loadWalkingParam return lebih
  // awal saat gagal parse).
  //
  // Nilainya sengaja disamakan dengan param.yaml robot INI (CHRONUS), supaya
  // saat yaml gagal dimuat robot tetap dapat gait yang wajar, bukan gait asing.
  // Nilai stock ROBOTIS dulu jauh berbeda (period 600 ms, y_swap 0.020,
  // arm_swing 1.5) dan diam-diam bisa membuat robot berjalan dengan parameter
  // yang tidak pernah dilihat operator.
  //
  // Ini salinan KETIGA dari angka yang sama -- dua lainnya: config/param.yaml
  // dan WALKING_DEFAULT_PARAMS di bascorro_studio/web/src/walkingParams.js.
  // Ubah satu, ubah ketiganya. Ingat konversi: yaml period_time dalam ms,
  // yaml *_offset dalam derajat.
  //
  // init pose (yaml: x_offset, y_offset, z_offset, roll/pitch/yaw_offset)
  // Disamakan dengan ALPHONSE 2026-09-07 atas permintaan operator, menggantikan
  // hasil tera ke page 2 (WALKING_READY) 2026-08-28. Harus sama dengan
  // config/param.yaml -- di sana ada tabel ukurnya, konsekuensinya (z_offset
  // 0.0408 membuat robot jongkok, r_knee sekitar -67 deg, bukan berdiri seperti
  // page 2), dan nilai tera lama kalau perlu dikembalikan. Ini cuma cadangan
  // kalau yaml gagal dimuat.
  walking_param_.init_x_offset = -0.0427;
  walking_param_.init_y_offset = 0.0247;
  walking_param_.init_z_offset = 0.002;
  walking_param_.init_roll_offset = 1.140186 * DEGREE2RADIAN;
  walking_param_.init_pitch_offset = -0.360963 * DEGREE2RADIAN;
  walking_param_.init_yaw_offset = -1.443854 * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset = 6.543178 * DEGREE2RADIAN;
  // time (yaml: period_time ms, dsp_ratio, step_forward_back_ratio)
  // Bentuk gait disamakan dengan ALPHONSE, 2026-09-07 -- lihat param.yaml.
  // Kecuali dua ini: diperlambat menyusul karena gait-nya terlalu cepat.
  // Keduanya harus turun bersama -- jendela ayun kaki = period * (1 - dsp),
  // jadi menaikkan period sambil dsp tinggi cuma memperpanjang fase dua kaki.
  walking_param_.period_time = 1667 * 0.001;
  walking_param_.dsp_ratio = 0.25;
  walking_param_.step_fb_ratio = 0.25;
  // walking -- x/y/angle amplitude bukan dari yaml, itu perintah runtime
  walking_param_.x_move_amplitude = 0.0;
  walking_param_.y_move_amplitude = 0.0;
  // Tuas utama kecepatan per langkah di robot ini, bukan x_move_amplitude --
  // tabel ukurnya ada di param.yaml.
  walking_param_.z_move_amplitude = 0.025;    // foot height
  walking_param_.angle_move_amplitude = 0.0;
  // balance -- balance_enable sengaja tidak dibaca loadWalkingParam(),
  // jadi nilainya selalu berasal dari sini
  walking_param_.balance_enable = true;
  walking_param_.balance_hip_roll_gain = 0.35;
  walking_param_.balance_knee_gain = 0.40;
  walking_param_.balance_ankle_roll_gain = 0.70;
  walking_param_.balance_ankle_pitch_gain = 0.90;
  walking_param_.y_swap_amplitude = 0.020;    // yaml swing_right_left
  walking_param_.z_swap_amplitude = 0.006;    // yaml swing_top_down
  walking_param_.pelvis_offset = 3.0 * DEGREE2RADIAN;
  walking_param_.arm_swing_gain = 1.5;

  // member variable
  body_swing_y = 0;
  body_swing_z = 0;

  x_swap_phase_shift_ = M_PI;
  x_swap_amplitude_shift_ = 0;
  x_move_phase_shift_ = M_PI / 2;
  x_move_amplitude_shift_ = 0;
  y_swap_phase_shift_ = 0;
  y_swap_amplitude_shift_ = 0;
  y_move_phase_shift_ = M_PI / 2;
  z_swap_phase_shift_ = M_PI * 3 / 2;
  z_move_phase_shift_ = M_PI / 2;
  a_move_phase_shift_ = M_PI / 2;

  ctrl_running_ = false;
  real_running_ = false;
  time_ = 0;

  //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,
  //                     L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
  //                     R_ARM_SWING, L_ARM_SWING
  joint_axis_direction_ <<      -1,         -1,          -1,     -1,             1,            1,
                -1,         -1,           1,      1,            -1,            1,
                 1,         -1;
  init_position_        <<     0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
                 0.0,        0.0,         0.0,    0.0,           0.0,          0.0,
                 0.0,        0.0;
  init_position_ *= DEGREE2RADIAN;

  std::string default_param_path = ament_index_cpp::get_package_share_directory("op3_walking_module") + "/config/param.yaml";
  this->declare_parameter<std::string>("walking_param_path", default_param_path);
  this->get_parameter("walking_param_path", param_path_);

  loadWalkingParam(param_path_);

  // Parameter ROS durasi soft start. Nilai awalnya dari param.yaml (atau bawaan
  // 2 dtk), lalu bisa diubah saat jalan -- mis. dari Bascorro Studio.
  this->declare_parameter<double>("soft_start_time", soft_start_time_.load());
  // false (bawaan): tiap Walking Enable menangkap pose sekarang sebagai dasar,
  // perilaku lama. true: pakai ulang dasar penangkapan sahih terakhir (lihat
  // g_last_walking_bias). Tidak disimpan ke param.yaml -- selalu mulai false.
  this->declare_parameter<bool>("reuse_walking_bias", false);
  this->declare_parameter<double>("idle_arm_swing_deg", 0.0);
  this->declare_parameter<double>("idle_arm_swing_period_sec", 1.0);
  soft_start_param_cb_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const rclcpp::Parameter &param : params)
      {
        if (param.get_name() == "idle_arm_swing_deg" || param.get_name() == "idle_arm_swing_period_sec")
        {
          double value = 0.0;
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            value = param.as_double();
          else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            value = static_cast<double>(param.as_int());
          else
          {
            result.successful = false;
            result.reason = param.get_name() + " harus angka";
            return result;
          }
          const bool is_deg = (param.get_name() == "idle_arm_swing_deg");
          if (!std::isfinite(value) || (is_deg ? (value < 0.0 || value > 45.0) : (value < 0.2 || value > 10.0)))
          {
            result.successful = false;
            result.reason = is_deg ? "idle_arm_swing_deg harus 0..45" : "idle_arm_swing_period_sec harus 0.2..10";
            return result;
          }
          if (is_deg)
            g_idle_arm_swing_deg = value;
          else
            g_idle_arm_swing_period = value;
          RCLCPP_INFO(this->get_logger(), "%s = %.2f", param.get_name().c_str(), value);
          continue;
        }
        if (param.get_name() == "reuse_walking_bias")
        {
          if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
          {
            result.successful = false;
            result.reason = "reuse_walking_bias harus bool";
            return result;
          }
          g_reuse_walking_bias = param.as_bool();
          RCLCPP_INFO(this->get_logger(), "reuse_walking_bias = %s", param.as_bool() ? "true" : "false");
          continue;
        }
        if (param.get_name() != "soft_start_time")
          continue;
        double value = 0.0;
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
          value = param.as_double();
        else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          value = static_cast<double>(param.as_int());
        else
        {
          result.successful = false;
          result.reason = "soft_start_time harus angka (detik)";
          return result;
        }
        if (!std::isfinite(value) || value < 0.0 || value > SOFT_START_MAX_SEC)
        {
          result.successful = false;
          result.reason = "soft_start_time harus 0.." + std::to_string(SOFT_START_MAX_SEC) + " detik";
          return result;
        }
        const bool changed = (value != soft_start_time_.load());
        soft_start_time_ = value;
        RCLCPP_INFO(this->get_logger(), "soft_start_time = %.2f dtk", value);
        if (changed)
          persistSoftStartTime(value);
      }
      return result;
    });

  updateTimeParam();
  updateMovementParam();
}

void WalkingModule::queueThread()
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(this->get_node_base_interface());

  /* publish topics */
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("robotis/status", 1);

  /* ROS Service Callback Functions */
  auto get_walking_param_server = this->create_service<op3_walking_module_msgs::srv::GetWalkingParam>(
    "/robotis/walking/get_params", std::bind(&WalkingModule::getWalkingParameterCallback, this, std::placeholders::_1, std::placeholders::_2));

  /* sensor topic subscribe */
  auto walking_command_sub = this->create_subscription<std_msgs::msg::String>(
    "/robotis/walking/command", 10, std::bind(&WalkingModule::walkingCommandCallback, this, std::placeholders::_1));
  auto walking_param_sub = this->create_subscription<op3_walking_module_msgs::msg::WalkingParam>(
    "/robotis/walking/set_params", 10, std::bind(&WalkingModule::walkingParameterCallback, this, std::placeholders::_1));

  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok())
  {
    executor->spin_some();
    rate.sleep();
  }
}

void WalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  auto status_msg = robotis_controller_msgs::msg::StatusMsg();
  status_msg.header.stamp = rclcpp::Clock().now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

void WalkingModule::walkingCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if(enable_ == false)
  {
    RCLCPP_WARN(this->get_logger(), "walking module is not ready.");
    return;
  }

  if (msg->data == "start")
    startWalking();
  else if (msg->data == "stop")
    stop();
  else if (msg->data == "soft_stop")
  {
    // Tanpa durasi soft start, atau gait memang tidak berjalan: sama dengan stop.
    if (soft_start_time_.load() > 0.0 && real_running_ == true && ctrl_running_ == true)
      g_soft_stop_request = true;
    else
      stop();
  }
  else if (msg->data == "balance on")
    walking_param_.balance_enable = true;
  else if (msg->data == "balance off")
    walking_param_.balance_enable = false;
  else if (msg->data == "save")
    saveWalkingParam(param_path_);
}

void WalkingModule::walkingParameterCallback(const op3_walking_module_msgs::msg::WalkingParam::SharedPtr msg)
{
  walking_param_ = *msg;  
}

bool WalkingModule::getWalkingParameterCallback(const std::shared_ptr<op3_walking_module_msgs::srv::GetWalkingParam::Request> req,
                         std::shared_ptr<op3_walking_module_msgs::srv::GetWalkingParam::Response> res)
{
  res->parameters = walking_param_;

  return true;
}

double WalkingModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

// m, rad
// for default op3: it was used from previos version(OP2) but it's not using now.
bool WalkingModule::computeIK(double *out, double pos_x, double pos_y, double pos_z, double ori_roll, double ori_pitch,
                double ori_yaw)
{
  double thigh_length = 93.0 * 0.001;  //m
  double calf_length = 93.0 * 0.001;  //m
  double ankle_length = 33.5 * 0.001;  //m
  double leg_length = 219.5 * 0.001;  //m (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

  Eigen::MatrixXd transformation_ad, transformation_da, transformation_cd, transformation_dc, transformation_ac;
  Eigen::Vector3d vector;
  double r_ac, acos_value, atan_value, value_k, value_l, value_m, value_n, value_s, value_c, theta;

  // make transform matrix
  transformation_ad = robotis_framework::getTransformationXYZRPY(pos_x, pos_y, pos_z, ori_roll, ori_pitch, ori_yaw);

  vector << pos_x + transformation_ad.coeff(0, 2) * ankle_length, pos_y + transformation_ad.coeff(1, 2) * ankle_length, (pos_z
    - leg_length) + transformation_ad.coeff(2, 2) * ankle_length;

  // Get Knee
  r_ac = vector.norm();
  acos_value = acos(
    (r_ac * r_ac - thigh_length * thigh_length - calf_length * calf_length) / (2 * thigh_length * calf_length));
  if (std::isnan(acos_value) == 1)
    return false;
  *(out + 3) = acos_value;

  // Get Ankle Roll
  transformation_da = robotis_framework::getInverseTransformation(transformation_ad);
  double tda_y = transformation_da.coeff(1, 3);
  double tda_z = transformation_da.coeff(2, 3);
  value_k = sqrt(tda_y * tda_y + tda_z * tda_z);
  value_l = sqrt(tda_y * tda_y + (tda_z - ankle_length) * (tda_z - ankle_length));
  value_m = (value_k * value_k - value_l * value_l - ankle_length * ankle_length) / (2 * value_l * ankle_length);
  if (value_m > 1.0)
    value_m = 1.0;
  else if (value_m < -1.0)
    value_m = -1.0;
  acos_value = acos(value_m);
  if (std::isnan(acos_value) == 1)
    return false;
  if (tda_y < 0.0)
    *(out + 5) = -acos_value;
  else
    *(out + 5) = acos_value;

  // Get Hip Yaw
  transformation_cd = robotis_framework::getTransformationXYZRPY(0.0, 0.0, -ankle_length, *(out + 5), 0.0, 0.0);
  transformation_dc = robotis_framework::getInverseTransformation(transformation_cd);
  transformation_ac = transformation_ad * transformation_dc;
  atan_value = atan2(-transformation_ac.coeff(0, 1), transformation_ac.coeff(1, 1));
  if (std::isinf(atan_value) == 1)
    return false;
  *(out) = atan_value;

  // Get Hip Roll
  atan_value = atan2(transformation_ac.coeff(2, 1),
           -transformation_ac.coeff(0, 1) * sin(*(out)) + transformation_ac.coeff(1, 1) * cos(*(out)));
  if (std::isinf(atan_value) == 1)
    return false;
  *(out + 1) = atan_value;

  // Get Hip Pitch and Ankle Pitch
  atan_value = atan2(transformation_ac.coeff(0, 2) * cos(*(out)) + transformation_ac.coeff(1, 2) * sin(*(out)),
           transformation_ac.coeff(0, 0) * cos(*(out)) + transformation_ac.coeff(1, 0) * sin(*(out)));
  if (std::isinf(atan_value) == 1)
    return false;
  theta = atan_value;
  value_k = sin(*(out + 3)) * calf_length;
  value_l = -thigh_length - cos(*(out + 3)) * calf_length;
  value_m = cos(*(out)) * vector.x() + sin(*(out)) * vector.y();
  value_n = cos(*(out + 1)) * vector.z() + sin(*(out)) * sin(*(out + 1)) * vector.x()
    - cos(*(out)) * sin(*(out + 1)) * vector.y();
  value_s = (value_k * value_n + value_l * value_m) / (value_k * value_k + value_l * value_l);
  value_c = (value_n - value_k * value_s) / value_l;
  atan_value = atan2(value_s, value_c);
  if (std::isinf(atan_value) == 1)
    return false;
  *(out + 2) = atan_value;
  *(out + 4) = theta - *(out + 3) - *(out + 2);

  return true;
}

void WalkingModule::updateTimeParam(double scale)
{
  period_time_ = scale*walking_param_.period_time;  // * 1000;   // s -> ms
  dsp_ratio_ = walking_param_.dsp_ratio;
  ssp_ratio_ = 1 - dsp_ratio_;

  x_swap_period_time_ = period_time_ / 2;
  x_move_period_time_ = period_time_ * ssp_ratio_;
  y_swap_period_time_ = period_time_;
  y_move_period_time_ = period_time_ * ssp_ratio_;
  z_swap_period_time_ = period_time_ / 2;
  z_move_period_time_ = period_time_ * ssp_ratio_ / 2;
  a_move_period_time_ = period_time_ * ssp_ratio_;

  ssp_time_ = period_time_ * ssp_ratio_;
  l_ssp_start_time_ = (1 - ssp_ratio_) * period_time_ / 4;
  l_ssp_end_time_ = (1 + ssp_ratio_) * period_time_ / 4;
  r_ssp_start_time_ = (3 - ssp_ratio_) * period_time_ / 4;
  r_ssp_end_time_ = (3 + ssp_ratio_) * period_time_ / 4;

  phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2;
  phase2_time_ = (l_ssp_end_time_ + r_ssp_start_time_) / 2;
  phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_) / 2;

  pelvis_offset_ = walking_param_.pelvis_offset;
  pelvis_swing_ = pelvis_offset_ * 0.35;
  arm_swing_gain_ = walking_param_.arm_swing_gain;
}

void WalkingModule::updateMovementParam()
{
  // Soft start: skala 0..1 untuk x/y/z/angle (lihat soft_start_* di header). Di
  // luar masa soft start nilainya 1, jadi perilaku aslinya tidak berubah.
  const double ss = soft_start_scale_;

  // Forward/Back
  x_move_amplitude_ = walking_param_.x_move_amplitude * ss;
  x_swap_amplitude_ = walking_param_.x_move_amplitude * ss * walking_param_.step_fb_ratio;

  if (previous_x_move_amplitude_ == 0)
  {
    x_move_amplitude_ *= 0.5;
    x_swap_amplitude_ *= 0.5;
  }

  // Right/Left
  y_move_amplitude_ = walking_param_.y_move_amplitude * ss / 2;
  if (y_move_amplitude_ > 0)
    y_move_amplitude_shift_ = y_move_amplitude_;
  else
    y_move_amplitude_shift_ = -y_move_amplitude_;
  y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;

  z_move_amplitude_ = walking_param_.z_move_amplitude * ss / 2;
  z_move_amplitude_shift_ = z_move_amplitude_ / 2;
  z_swap_amplitude_ = walking_param_.z_swap_amplitude;
  z_swap_amplitude_shift_ = z_swap_amplitude_;

  // Direction
  if (walking_param_.move_aim_on == false)
  {
    a_move_amplitude_ = walking_param_.angle_move_amplitude * ss / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = a_move_amplitude_;
    else
      a_move_amplitude_shift_ = -a_move_amplitude_;
  }
  else
  {
    a_move_amplitude_ = -walking_param_.angle_move_amplitude * ss / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = -a_move_amplitude_;
    else
      a_move_amplitude_shift_ = a_move_amplitude_;
  }
}

void WalkingModule::updatePoseParam()
{
  x_offset_ = walking_param_.init_x_offset;
  y_offset_ = walking_param_.init_y_offset;
  z_offset_ = walking_param_.init_z_offset;
  r_offset_ = walking_param_.init_roll_offset;
  p_offset_ = walking_param_.init_pitch_offset;
  a_offset_ = walking_param_.init_yaw_offset;
  hit_pitch_offset_ = walking_param_.hip_pitch_offset;
}

void WalkingModule::startWalking()
{
  // Soft start hanya saat gait BENAR-BENAR mulai dari diam. "start" yang dikirim
  // ulang saat robot sudah berjalan tidak boleh menjatuhkan amplitudo ke nol --
  // itu justru sentakan. Diserahkan ke process() lewat flag supaya variabel ramp
  // hanya disentuh satu thread.
  //
  // "Dari diam" juga mencakup gait yang SUDAH diperintah berhenti (ctrl_running_
  // false) tapi masih menutup langkah terakhirnya (real_running_ masih true).
  // Dulu start yang tiba di jendela itu -- mis. D58 berpindah dari satu geser ke
  // geser berikutnya -- dianggap "masih berjalan", sehingga geser baru jalan TANPA
  // soft start. Amplitudo gait yang sedang berhenti memang sedang menuju nol, jadi
  // melandaikannya lagi dari nol tidak menimbulkan sentakan.
  if (real_running_ == false || ctrl_running_ == false)
    soft_start_request_ = true;
  // "start" membatalkan soft stop yang belum selesai: gait mulai lagi dengan
  // soft start, bukan diteruskan menuju berhenti.
  if (g_soft_stop_request.exchange(false) == true)
    soft_start_request_ = true;

  ctrl_running_ = true;
  real_running_ = true;

  // soft start: stretch the first cycle 1.5x so the first foot-lift isn't a slam
  updateTimeParam(1.5);
  time_ = control_cycle_msec_ * 0.001;

  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Start walking");
}

void WalkingModule::stop()
{
  ctrl_running_ = false;
  //period_time_ = walking_param_.period_time *1.5; // hs
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Stop walking");
}

bool WalkingModule::isRunning()
{
  return real_running_ || (walking_state_ == WalkingInitPose);
}

// default [angle : radian, length : m]
void WalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                            std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  const double time_unit = control_cycle_msec_ * 0.001;  // ms -> s
  int joint_size = result_.size();
  double angle[joint_size];
  double balance_angle[joint_size];

  for(int _idx = 0; _idx < joint_size; _idx++)
  {
    angle[_idx]         = 0.0f;
    balance_angle[_idx] = 0.0f;
  }

  if (walking_state_ == WalkingInitPose)
  {
    int total_count = calc_joint_tra_.rows();
    // Indeks sendi 0..13 (joint_table_), dan target_position_ berukuran 1x14.
    // Dulu tertulis "id = 1; id <= result_.size()", yaitu 1..14, dan itu SALAH
    // di dua arah sekaligus:
    //   * id 0 -- r_hip_yaw -- tidak pernah ikut dilandaikan. Selama ramp ia
    //     menahan pose lama, lalu begitu state kembali ke WalkingReady, cabang
    //     idle memerintahkannya ke stance netral yang baru dalam SATU siklus
    //     8 ms. Itulah sentakan yang muncul "kadang-kadang": besarnya sentakan
    //     sama dengan selisih init_yaw_offset lama vs baru, jadi ia tak terlihat
    //     selama offset yaw tidak berubah, dan menyentak keras saat berubah.
    //   * id 14 di luar batas matriks 1x14. Eigen tanpa NDEBUG-assert menulis
    //     begitu saja ke memori sesudah buffer -- perilaku tak terdefinisi, dan
    //     kandidat kuat untuk "stack smashing" yang muncul sesekali.
    for (int id = 0; id < static_cast<int>(result_.size()); id++)
      target_position_.coeffRef(0, id) = calc_joint_tra_(init_pose_count_, id);

    init_pose_count_ += 1;
    if (init_pose_count_ >= total_count)
    {
      walking_state_ = WalkingReady;
      if (DEBUG)
        std::cout << "End moving to Init : " << init_pose_count_ << std::endl;
    }

  }
  else if (walking_state_ == WalkingReady || walking_state_ == WalkingEnable)
  {
    // present angle
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
        state_iter != result_.end(); state_iter++)
    {
      std::string _joint_name = state_iter->first;
      int joint_index = joint_table_[_joint_name];

      robotis_framework::Dynamixel *dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(_joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;
      else
        continue;

      goal_position_.coeffRef(0, joint_index) = dxl->dxl_state_->goal_position_;
    }

    // Kunci stance ke pose yang ada saat modul diaktifkan -- di robot ini itu
    // page 2 (WALKING_READY), sebab op3_manager memainkannya saat boot dan tombol
    // Init Pose di Bascorro Studio memainkannya lagi.
    //
    //   walking_bias_ = pose tertangkap - stance netral IK dari offset sekarang
    //   diam     -> goal = walking_bias_ + neutral(offset sekarang)
    //   berjalan -> goal = walking_bias_ + sudut IK + balance
    //
    // Karena bias + neutral == pose tertangkap tepat sesudah penangkapan,
    // mengaktifkan modul tidak menimbulkan lompatan. Sesudah itu offset Init Pose
    // MENGGESER stance relatif terhadap page 2, bukan menetapkannya secara mutlak.
    //
    // Untuk lengan bias menyimpan pose penuh: baris di bawah hanya menimpa 12
    // sendi kaki, jadi kolom 12..13 tetap berisi goal_position_ apa adanya.
    if (capture_init_pose_ == true)
    {
      // Modul baru diaktifkan: goal berasal dari modul lain, tanpa ayunan.
      g_idle_swing_scale = 0.0;
      g_idle_swing_prev[0] = g_idle_swing_prev[1] = 0.0;
    }
    if (capture_init_pose_ == true && g_reuse_walking_bias.load() == true &&
        g_last_walking_bias_valid == true && g_last_walking_bias.cols() == goal_position_.cols())
    {
      // Dasar lama dipakai apa adanya; pose sekarang (akhir page) TIDAK ditangkap.
      // Cabang diam di bawah lalu melandaikan kaki dari pose sekarang ke
      // bias + neutral(offset versi) dengan pembatas laju yang sama.
      walking_bias_ = g_last_walking_bias;
      captured_init_pose_ = g_last_captured_pose;
      walking_bias_valid_ = true;
      capture_init_pose_ = false;
      RCLCPP_INFO(this->get_logger(), "Walking Enable: dasar stance sebelumnya dipakai ulang (reuse_walking_bias)");
    }
    if (capture_init_pose_ == true)
    {
      captured_init_pose_ = goal_position_;
      walking_bias_ = goal_position_;
      double neutral_leg[12];
      // Acuan bias = offset RUJUKAN tetap, bukan offset yang sedang dimuat (lihat
      // REF_* di header). Dengan begitu offset hasil tuning -- mis. pitch -0,07 --
      // tetap menggeser stance setelah penangkapan ulang, alih-alih ikut terserap
      // ke bias lalu lenyap. init_y_offset tetap ditinggalkan dari acuan: y_offset
      // adalah tambahan lebar pijakan di atas page 2.
      walking_bias_valid_ = computeReferenceLegAngle(neutral_leg);
      if (walking_bias_valid_ == true)
      {
        for (int i = 0; i < 12; i++)
          walking_bias_.coeffRef(0, i) = goal_position_.coeff(0, i) - neutral_leg[i];
        g_last_walking_bias = walking_bias_;
        g_last_captured_pose = captured_init_pose_;
        g_last_walking_bias_valid = true;
      }
      capture_init_pose_ = false;
    }

    bool get_angle = false;
    const bool walking_idle = (ctrl_running_ == false && real_running_ == false);

    if (walking_idle == false)
    {
      // Soft start: naikkan skala x/y/z/angle menerus tiap siklus, lalu hitung ulang
      // amplitudo SEKARANG juga (bukan menunggu batas fase) -- itulah yang
      // membuat lintasan kaki mulus. Lihat soft_start_* di header.
      if (g_soft_stop_request.exchange(false) == true && ctrl_running_ == true)
      {
        // Mulai dari skala SEKARANG (bisa < 1 bila soft start belum selesai),
        // supaya tidak ada lompatan amplitudo ke atas sebelum turun.
        soft_start_active_ = false;
        g_soft_stop_active = true;
        g_soft_stop_elapsed = 0.0;
        g_soft_stop_duration = soft_start_time_.load();
        g_soft_stop_from = soft_start_scale_;
        RCLCPP_INFO(this->get_logger(), "soft stop %.2f dtk", g_soft_stop_duration);
      }

      if (g_soft_stop_active == true && soft_start_request_.load() == true)
      {
        g_soft_stop_active = false;   // dibatalkan "start" -- soft start mengambil alih
      }
      else if (g_soft_stop_active == true)
      {
        g_soft_stop_elapsed += time_unit;
        if (g_soft_stop_duration <= 0.0 || g_soft_stop_elapsed >= g_soft_stop_duration)
        {
          g_soft_stop_active = false;
          soft_start_scale_ = 0.0;
          updateMovementParam();
          stop();                     // amplitudo sudah nol -> gait menutup langkahnya
        }
        else
        {
          soft_start_scale_ = g_soft_stop_from * (1.0 - g_soft_stop_elapsed / g_soft_stop_duration);
          updateMovementParam();
        }
      }
      else if (soft_start_request_.exchange(false) == true)
      {
        const double duration = soft_start_time_.load();
        soft_start_elapsed_ = 0.0;
        soft_start_active_ = (duration > 0.0);
        soft_start_scale_ = soft_start_active_ ? 0.0 : 1.0;
        updateMovementParam();
      }
      else if (soft_start_active_ == true)
      {
        const double duration = soft_start_time_.load();
        soft_start_elapsed_ += time_unit;
        if (duration <= 0.0 || soft_start_elapsed_ >= duration)
        {
          soft_start_active_ = false;
          soft_start_scale_ = 1.0;
        }
        else
        {
          soft_start_scale_ = soft_start_elapsed_ / duration;
        }
        updateMovementParam();
      }

      processPhase(time_unit);

      get_angle = computeLegAngle(&angle[0]);

      computeArmAngle(&angle[12]);

      double rl_gyro_err = 0.0 - sensors["gyro_x"];
      double fb_gyro_err = 0.0 - sensors["gyro_y"];

      sensoryFeedback(rl_gyro_err, fb_gyro_err, balance_angle);
    }

    // Stance saat diam, dihitung ulang tiap siklus supaya suntingan offset langsung
    // terlihat. Kalau IK tidak punya solusi untuk offset yang diketik operator,
    // tahan pose sekarang alih-alih memerintahkan stance yang tak terhitung.
    //
    // true: berbeda dengan penangkapan di atas, stance diam MEMBAWA init_y_offset,
    // supaya telapak berdiri selebar yang nanti dipakai gait. Tanpa itu kaki hanya
    // melebar saat "start" ditekan lalu merapat lagi saat "stop".
    double neutral_leg[12];
    bool neutral_ok = false;
    if (walking_bias_valid_ == true)
      neutral_ok = computeNeutralLegAngle(neutral_leg, true);

    // SERVO 17 & 18 (r/l_ank_roll) DIKUNCI KE PAGE 2. Offset Init Pose dari Studio
    // menggeser stance relatif ke page 2 sebesar neutral(offset sekarang) -
    // neutral(REF_*); di ank_roll geseran itu yang paling kelihatan (init_y_offset
    // 0,030 -> ~3,9 deg). Untuk dua sendi ini geseran tersebut dikurangkan lagi:
    // diam = nilai page 2 persis, berjalan = page 2 + ayunan gait + balance. Sendi
    // lain tetap mengikuti offset.
    //
    // Dulu (operator 2026-10-26) hanya servo 18 yang dikunci. Akibatnya 17 ikut
    // offset sedangkan 18 tidak, jadi pada versi yang offsetnya jauh dari REF_*
    // (mis. "geser kanan": y 0,015, roll 0,03) kedua pergelangan miring tak sama.
    // Kini keduanya dikunci (operator 2026-10-27). Konsekuensinya: bila
    // init_y_offset != 0, hip roll tetap melebar sementara ank_roll tidak
    // mengimbangi, jadi kedua telapak miring simetris sebesar geseran itu.
    double ank_roll_shift[12] = {0.0};
    if (neutral_ok == true)
    {
      double ref_leg[12];
      if (computeReferenceLegAngle(ref_leg) == true)
      {
        for (const char *name : {"r_ank_roll", "l_ank_roll"})
        {
          const int i = joint_table_[name];
          ank_roll_shift[i] = neutral_leg[i] - ref_leg[i];
        }
      }
    }

    double err_total = 0.0, err_max = 0.0;
    // set goal position
    // Ayunan lengan saat diam: pudar masuk saat idle & aktif, pudar keluar lainnya.
    double idle_swing[2] = {0.0, 0.0};
    {
      const double amp_param = g_idle_arm_swing_deg.load();
      if (amp_param > 0.0)
        g_idle_swing_amp_deg = amp_param;
      const double want = (walking_idle == true && amp_param > 0.0) ? 1.0 : 0.0;
      const double fade_step = time_unit / IDLE_SWING_FADE_SEC;
      if (g_idle_swing_scale < want)
        g_idle_swing_scale = std::min(want, g_idle_swing_scale + fade_step);
      else if (g_idle_swing_scale > want)
        g_idle_swing_scale = std::max(want, g_idle_swing_scale - fade_step);

      if (g_idle_swing_scale > 0.0)
      {
        const double period = std::max(0.2, g_idle_arm_swing_period.load());
        g_idle_swing_time = std::fmod(g_idle_swing_time + time_unit, period);
        const double s = std::sin(2.0 * M_PI * g_idle_swing_time / period);
        const double amp = g_idle_swing_amp_deg * g_idle_swing_scale * DEGREE2RADIAN;
        // Arah sama dengan computeArmAngle(): kanan -, kiri +, dikali arah sendi.
        idle_swing[0] = -amp * s * op3_kd_->getJointDirection("r_sho_pitch");
        idle_swing[1] =  amp * s * op3_kd_->getJointDirection("l_sho_pitch");
      }
      else
      {
        g_idle_swing_time = 0.0;
      }
    }

    for (int idx = 0; idx < 14; idx++)
    {
      double goal_position = 0.0;
      if (walking_idle == true)
      {
        const double target = (idx < 12 && neutral_ok)
                            ? walking_bias_.coeff(0, idx) + neutral_leg[idx]
                                - ank_roll_shift[idx]
                            : (idx < 12 ? goal_position_.coeff(0, idx)
                                        : captured_init_pose_.coeff(0, idx));
        // PEMBATAS LAJU. Tanpa ini, perubahan stance yang KECIL dihentakkan dalam
        // satu siklus 8 ms. Sebabnya gerbang ramp di bawah memakai err_total --
        // JUMLAH selisih 14 sendi dalam derajat -- dibandingkan dengan 5.0. Jadi
        // pergeseran yang totalnya di bawah 5 derajat sama sekali tidak dilandaikan:
        // ankle yang bergeser 4 derajat sendirian lolos gerbang itu dan diperintah
        // seketika, yaitu sekitar 500 deg/s. Itulah "tiba-tiba tumitnya naik dan
        // badan mundur" yang muncul saat Apply, walau yang diubah operator adalah
        // field yang tidak ada hubungannya: Apply mengirim SELURUH struct, jadi
        // offset Init Pose di tabel Studio ikut terkirim dan stance bergeser sedikit.
        //
        // Dengan pembatas ini setiap perpindahan stance saat diam melandai pada
        // 25 deg/s, berapa pun besarnya, dan gerbang err_total tidak lagi menentukan
        // halus atau tidaknya. Hanya berlaku saat DIAM -- gait yang sedang berjalan
        // tidak pernah lewat cabang ini. Pengaman yang sama sudah lama ada di
        // ALPHONSE; ia ikut terbuang di sini waktu walking_bias_ dihapus.
        const double max_step = 25.0 * DEGREE2RADIAN * time_unit;
        // Lengan: batasi laju POSE DASAR (tanpa ayunan), ayunan ditambah sesudahnya.
        const double present = goal_position_.coeff(0, idx) - (idx >= 12 ? g_idle_swing_prev[idx - 12] : 0.0);
        const double step = target - present;
        if (step > max_step)
          goal_position = present + max_step;
        else if (step < -max_step)
          goal_position = present - max_step;
        else
          goal_position = target;
      }
      else if (get_angle == false && idx < 12)
        goal_position = goal_position_.coeff(0, idx);
      else
        // Kaki DAN lengan sama bentuknya sekarang: ayunan yang berpusat di nol
        // DITAMBAHKAN ke pose tertangkap. Untuk kaki walking_bias_ berisi
        // (pose tertangkap - neutral), jadi hasilnya = page 2 + simpangan gait.
        // Untuk lengan ia berisi pose penuh, sebab computeArmAngle() juga
        // berpusat di nol -- tanpa penjumlahan ini bahu diperintah ke ~0 rad
        // begitu gait mulai, padahal page 2 menahannya di +56,5 / -51,4 derajat,
        // jadi lengan melompat ~56 derajat ke depan tepat saat start.
        // balance_angle[12..13] selalu 0 (sensoryFeedback() hanya menulis 8 sendi
        // kaki) tapi tetap dijumlahkan agar bentuknya seragam.
        goal_position = walking_bias_.coeff(0, idx) + angle[idx] + balance_angle[idx]
                        - (idx < 12 ? ank_roll_shift[idx] : 0.0);

      // Ayunan lengan saat diam (idle) atau sisa pudarnya (sesudah gait mulai).
      if (idx >= 12)
      {
        goal_position += idle_swing[idx - 12];
        g_idle_swing_prev[idx - 12] = idle_swing[idx - 12];
      }

      target_position_.coeffRef(0, idx) = goal_position;

      double err = fabs(target_position_.coeff(0, idx) - goal_position_.coeff(0, idx)) * RADIAN2DEGREE;
      if (err > err_max)
        err_max = err;
      err_total += err;
    }

    // Ease into the target instead of stepping to it. This runs when the module is
    // first enabled, and now also whenever an Init Pose offset moves the standing
    // stance -- without it, editing an offset would snap the legs in a single 8 ms
    // cycle. Guarded by walking_idle so a running gait is never interrupted.
    const bool first_enable = (walking_state_ == WalkingEnable);
    if ((first_enable || walking_idle == true) && err_total > 5.0)
    {
      if (DEBUG)
        std::cout << "Check Err : " << err_max << std::endl;

      // make trajectory for init pose (faster transition: 30 deg/s, min 1.5 s).
      // This is the lag between enabling/acquiring walking_module and the first
      // step; the robot is already near the ready pose (capture/bias) so the
      // motion is small and a quicker ramp is still smooth.
      double mov_time = err_max / 30.0;
      iniPoseTraGene(mov_time < 1.5 ? 1.5 : mov_time);

      // set target to goal
      target_position_ = goal_position_;

      walking_state_ = WalkingInitPose;

      if (first_enable == false)
      {
        RCLCPP_INFO(this->get_logger(), "Init Pose offsets changed - easing to the new stance");
      }
      else
      {
      RCLCPP_WARN_STREAM(this->get_logger(), "x_offset: " << walking_param_.init_x_offset);
      RCLCPP_WARN_STREAM(this->get_logger(), "y_offset: " << walking_param_.init_y_offset);
      RCLCPP_WARN_STREAM(this->get_logger(), "z_offset: " << walking_param_.init_z_offset);
      RCLCPP_WARN_STREAM(this->get_logger(), "roll_offset: " << walking_param_.init_roll_offset * RADIAN2DEGREE);
      RCLCPP_WARN_STREAM(this->get_logger(), "pitch_offset: " << walking_param_.init_pitch_offset * RADIAN2DEGREE);
      RCLCPP_WARN_STREAM(this->get_logger(), "yaw_offset: " << walking_param_.init_yaw_offset * RADIAN2DEGREE);
      RCLCPP_WARN_STREAM(this->get_logger(), "hip_pitch_offset: " << walking_param_.hip_pitch_offset * RADIAN2DEGREE);
      RCLCPP_WARN_STREAM(this->get_logger(), "period_time: " << walking_param_.period_time * 1000);
      RCLCPP_WARN_STREAM(this->get_logger(), "dsp_ratio: " << walking_param_.dsp_ratio);
      RCLCPP_WARN_STREAM(this->get_logger(), "step_forward_back_ratio: " << walking_param_.step_fb_ratio);
      RCLCPP_WARN_STREAM(this->get_logger(), "foot_height: " << walking_param_.z_move_amplitude);
      RCLCPP_WARN_STREAM(this->get_logger(), "swing_right_left: " << walking_param_.y_swap_amplitude);
      RCLCPP_WARN_STREAM(this->get_logger(), "swing_top_down: " << walking_param_.z_swap_amplitude);
      RCLCPP_WARN_STREAM(this->get_logger(), "pelvis_offset: " << walking_param_.pelvis_offset * RADIAN2DEGREE);
      RCLCPP_WARN_STREAM(this->get_logger(), "arm_swing_gain: " << walking_param_.arm_swing_gain);
      RCLCPP_WARN_STREAM(this->get_logger(), "balance_hip_roll_gain: " << walking_param_.balance_hip_roll_gain);
      RCLCPP_WARN_STREAM(this->get_logger(), "balance_knee_gain: " << walking_param_.balance_knee_gain);
      RCLCPP_WARN_STREAM(this->get_logger(), "balance_ankle_roll_gain: " << walking_param_.balance_ankle_roll_gain);
      RCLCPP_WARN_STREAM(this->get_logger(), "balance_ankle_pitch_gain: " << walking_param_.balance_ankle_pitch_gain);
      RCLCPP_WARN_STREAM(this->get_logger(), "balance : " << (walking_param_.balance_enable ? "TRUE" : "FALSE"));
      }
    }
    else
    {
      walking_state_ = WalkingReady;
    }
  }

  // set result
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
      state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int joint_index = joint_table_[joint_name];

    result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);

    // Todo : setting PID gain to the leg joints
    // result_[joint_name]->position_p_gain_ = walking_param_.p_gain;
    // result_[joint_name]->position_i_gain_ = walking_param_.i_gain;
    // result_[joint_name]->position_d_gain_ = walking_param_.d_gain;
  }

  //setJointGains(balancing_idx_);

  // time
  if (real_running_ == true)
  {
    time_ += time_unit;
    if (time_ >= period_time_)
    {
      time_ = 0;
      previous_x_move_amplitude_ = walking_param_.x_move_amplitude * 0.5;
    }
  }
}

void WalkingModule::setJointGains(int balancing_idx)
{

  if (balancing_idx == 2)
  {
    // L_BALANCING (L Support) first half
    result_["r_hip_pitch" ]->position_p_gain_ = 1600;
    result_["l_hip_pitch" ]->position_p_gain_ = 4000;
    
    result_["r_hip_roll" ]->position_p_gain_ = 1600;
    result_["l_hip_roll" ]->position_p_gain_ = 4000;

    result_["r_knee" ]->position_p_gain_ = 4000;
    result_["l_knee" ]->position_p_gain_ = 4000;

    result_["r_ank_pitch" ]->position_p_gain_ = 800;
    result_["l_ank_pitch" ]->position_p_gain_ = 1600;

    result_["r_ank_roll" ]->position_p_gain_ = 1600;
    result_["l_ank_roll" ]->position_p_gain_ = 1600;
  }
  else if (balancing_idx == 3)
  {
    // L_BALANCING (L Support) second half
    result_["r_hip_pitch" ]->position_p_gain_ = 1600;
    result_["l_hip_pitch" ]->position_p_gain_ = 4000;
    
    result_["r_hip_roll" ]->position_p_gain_ = 1600;
    result_["l_hip_roll" ]->position_p_gain_ = 4000;

    result_["r_knee" ]->position_p_gain_ = 4000;
    result_["l_knee" ]->position_p_gain_ = 4000;

    result_["r_ank_pitch" ]->position_p_gain_ = 800;
    result_["l_ank_pitch" ]->position_p_gain_ = 1600;

    result_["r_ank_roll" ]->position_p_gain_ = 1600;
    result_["l_ank_roll" ]->position_p_gain_ = 1600;
  }
  else if (balancing_idx == 6)
  {
    // R_BALANCING (R Support) first half
    result_["r_hip_pitch" ]->position_p_gain_ = 4000;
    result_["l_hip_pitch" ]->position_p_gain_ = 1600;
    
    result_["r_hip_roll" ]->position_p_gain_ = 4000;
    result_["l_hip_roll" ]->position_p_gain_ = 1600;

    result_["r_knee" ]->position_p_gain_ = 4000;
    result_["l_knee" ]->position_p_gain_ = 4000;

    result_["r_ank_pitch" ]->position_p_gain_ = 1600;
    result_["l_ank_pitch" ]->position_p_gain_ = 1600;

    result_["r_ank_roll" ]->position_p_gain_ = 1600;
    result_["l_ank_roll" ]->position_p_gain_ = 1600;
  }
  else if (balancing_idx == 7)
  {
    // R_BALANCING (R Support) second half
    result_["r_hip_pitch" ]->position_p_gain_ = 4000;
    result_["l_hip_pitch" ]->position_p_gain_ = 1600;

    result_["r_hip_roll" ]->position_p_gain_ = 4000;
    result_["l_hip_roll" ]->position_p_gain_ = 1600;
    
    result_["r_knee" ]->position_p_gain_ = 4000;
    result_["l_knee" ]->position_p_gain_ = 4000;

    result_["r_ank_pitch" ]->position_p_gain_ = 1600;
    result_["l_ank_pitch" ]->position_p_gain_ = 1600;

    result_["r_ank_roll" ]->position_p_gain_ = 1600;
    result_["l_ank_roll" ]->position_p_gain_ = 1600;
  }
  else
  {
    // DS
    result_["r_hip_pitch" ]->position_p_gain_ = 1600;
    result_["l_hip_pitch" ]->position_p_gain_ = 1600;
    
    result_["r_hip_roll" ]->position_p_gain_ = 1600;
    result_["l_hip_roll" ]->position_p_gain_ = 1600;
    
    result_["r_knee" ]->position_p_gain_ = 4000;
    result_["l_knee" ]->position_p_gain_ = 4000;

    result_["r_ank_pitch" ]->position_p_gain_ = 800;
    result_["l_ank_pitch" ]->position_p_gain_ = 800;
    
    result_["r_ank_roll" ]->position_p_gain_ = 1600;
    result_["l_ank_roll" ]->position_p_gain_ = 1600;
  }

  if (ctrl_running_ == false) // for stop
  {
    if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
    {
      if (balancing_idx == 0 || balancing_idx == 1 || balancing_idx == 5)
      {
        result_["r_hip_pitch"]->position_p_gain_ = 800;
        result_["l_hip_pitch"]->position_p_gain_ = 800;

        result_["r_hip_roll"]->position_p_gain_ = 800;
        result_["l_hip_roll"]->position_p_gain_ = 800;

        result_["r_knee"]->position_p_gain_ = 800;
        result_["l_knee"]->position_p_gain_ = 800;

        result_["r_ank_pitch"]->position_p_gain_ = 800;
        result_["l_ank_pitch"]->position_p_gain_ = 800;

        result_["r_ank_roll"]->position_p_gain_ = 800;
        result_["l_ank_roll"]->position_p_gain_ = 800;
      }
    }
  }
}



void WalkingModule::processPhase(const double &time_unit)
{
  // Update walk parameters
  if (time_ == 0)
  {
    updateTimeParam();
    phase_ = PHASE0;
    if (ctrl_running_ == false)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = 0;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;

        previous_x_move_amplitude_ = 0;
      }
    }
  }
  else if (time_ >= (phase1_time_ - time_unit / 2) && time_ < (phase1_time_ + time_unit / 2))  // the position of left foot is the highest.
  {
    updateMovementParam();

    updateTimeParam(); // hs
    time_ = phase1_time_; // hs

    phase_ = PHASE1;
  }
  else if (time_ >= (phase2_time_ - time_unit / 2) && time_ < (phase2_time_ + time_unit / 2))  // middle of double support state
  {
    updateTimeParam();

    time_ = phase2_time_;
    phase_ = PHASE2;
    if (ctrl_running_ == false)
    {
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // set walking param to init
        walking_param_.x_move_amplitude = previous_x_move_amplitude_;
        walking_param_.y_move_amplitude = 0;
        walking_param_.angle_move_amplitude = 0;
      }
    }
  }
  else if (time_ >= (phase3_time_ - time_unit / 2) && time_ < (phase3_time_ + time_unit / 2))  // the position of right foot is the highest.
  {
    updateMovementParam();
    updateTimeParam(); // hs
    time_ = phase3_time_; // hs
    phase_ = PHASE3;
  }
}

bool WalkingModule::computeLegAngle(double *leg_angle)
{
  Pose3D swap, right_leg_move, left_leg_move;
  double pelvis_offset_r, pelvis_offset_l;
  double ep[12];

  updatePoseParam();

  // Compute endpoints
  swap.x = wSin(time_, x_swap_period_time_, x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_);
  swap.y = wSin(time_, y_swap_period_time_, y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_);
  swap.z = wSin(time_, z_swap_period_time_, z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_);
  swap.roll = 0.0;
  swap.pitch = 0.0;
  swap.yaw = 0.0;

  if (time_ <= l_ssp_start_time_) // r support
  {
    balancing_idx_ = BalancingPhase5;
    left_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                           x_move_amplitude_shift_);
    left_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                           y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_start_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= l_ssp_end_time_)
  {
    balancing_idx_ = BalancingPhase6;
    left_leg_move.x = wSin(time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                           x_move_amplitude_shift_);
    left_leg_move.y = wSin(time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                           y_move_amplitude_shift_);
    left_leg_move.z = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, pelvis_swing_ / 2,
                           pelvis_swing_ / 2);
    pelvis_offset_r = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
                           -pelvis_offset_ / 2, -pelvis_offset_ / 2);
  }
  else if (time_ <= r_ssp_start_time_)
  {
    balancing_idx_ = BalancingPhase1;
    left_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                           x_move_amplitude_shift_);
    left_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                           y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (time_ <= r_ssp_end_time_)
  {
    balancing_idx_ = BalancingPhase2;
    left_leg_move.x = wSin(time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                           x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y = wSin(time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, pelvis_offset_ / 2,
                           pelvis_offset_ / 2);
    pelvis_offset_r = wSin(time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, -pelvis_swing_ / 2,
                           -pelvis_swing_ / 2);
  }
  else
  {
    balancing_idx_ = BalancingPhase0;
    left_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                           x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                           x_move_amplitude_, x_move_amplitude_shift_);
    left_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                           y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                           y_move_amplitude_, y_move_amplitude_shift_);
    left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                           z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                           z_move_amplitude_shift_);
    left_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                             a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                             a_move_amplitude_, a_move_amplitude_shift_);
    right_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                            x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -x_move_amplitude_, -x_move_amplitude_shift_);
    right_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                            y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                            -y_move_amplitude_, -y_move_amplitude_shift_);
    right_leg_move.z = wSin(r_ssp_end_time_, z_move_period_time_,
                            z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                            z_move_amplitude_shift_);
    right_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                              a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -a_move_amplitude_, -a_move_amplitude_shift_);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }

  left_leg_move.roll = 0;
  left_leg_move.pitch = 0;
  right_leg_move.roll = 0;
  right_leg_move.pitch = 0;

  double leg_length = op3_kd_->thigh_length_m_ + op3_kd_->calf_length_m_ + op3_kd_->ankle_length_m_;

  // mm, rad
  ep[0] = swap.x + right_leg_move.x + x_offset_;
  ep[1] = swap.y + right_leg_move.y - y_offset_ / 2;
  ep[2] = swap.z + right_leg_move.z + z_offset_ - leg_length;
  ep[3] = swap.roll + right_leg_move.roll - r_offset_ / 2;
  ep[4] = swap.pitch + right_leg_move.pitch + p_offset_;
  ep[5] = swap.yaw + right_leg_move.yaw - a_offset_ / 2;
  ep[6] = swap.x + left_leg_move.x + x_offset_;
  ep[7] = swap.y + left_leg_move.y + y_offset_ / 2;
  ep[8] = swap.z + left_leg_move.z + z_offset_ - leg_length;
  ep[9] = swap.roll + left_leg_move.roll + r_offset_ / 2;
  ep[10] = swap.pitch + left_leg_move.pitch + p_offset_;
  ep[11] = swap.yaw + left_leg_move.yaw + a_offset_ / 2;

  //std::cout << ep[0] << " " << ep[1] << " " << ep[2] << " " << ep[3] << " " << ep[4] << " " << ep[5] << " "
  //          << ep[6] << " " << ep[7] << " " << ep[8] << " " << ep[9] << " " << ep[10] << " " << ep[11] << " " << std::endl;

  // Compute body swing
  if (time_ <= l_ssp_end_time_)
  {
    body_swing_y = -ep[7];
    body_swing_z = ep[8];
  }
  else
  {
    body_swing_y = -ep[1];
    body_swing_z = ep[2];
  }
  body_swing_z -= leg_length;

  // right leg
  if (op3_kd_->calcInverseKinematicsForRightLeg(&leg_angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
  {
    RCLCPP_ERROR(this->get_logger(), "IK not Solved EPR : %f %f %f %f %f %f", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
    return false;
  }

  if (op3_kd_->calcInverseKinematicsForLeftLeg(&leg_angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
  {
    RCLCPP_ERROR(this->get_logger(), "IK not Solved EPL : %f %f %f %f %f %f", ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]);
    return false;
  }

  // Safety net: the analytical IK can return a non-finite angle for an
  // unreachable foot pose. If that ever happens, fail loudly (throttled) and
  // hold the legs instead of pushing NaN goals that the controller silently
  // drops -- that is what made the legs "freeze" while the arms kept swinging.
  for (int i = 0; i < 12; i++)
  {
    if (std::isfinite(leg_angle[i]) == false)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Leg IK produced a non-finite angle (joint %d) - foot pose out of reach. "
                            "Reduce x/y move amplitude or z_offset.", i);
      return false;
    }
  }

  // std::cout << leg_angle[0] << " " << leg_angle[1] << " " << leg_angle[2] << " " << leg_angle[3] << " " << leg_angle[4] << " " << leg_angle[5] << " "
  //           << leg_angle[6] << " " << leg_angle[7] << " " << leg_angle[8] << " " << leg_angle[9] << " " << leg_angle[10] << " " << leg_angle[11] << " " << std::endl;

  // Compute dxls angle
  for (int i = 0; i < 12; i++)
  {
    // offset : rad
    double offset = 0;

    if (i == joint_table_["r_hip_roll"])  // R_HIP_ROLL
      offset += op3_kd_->getJointDirection("r_hip_roll") * pelvis_offset_r;
    else if (i == joint_table_["l_hip_roll"])  // L_HIP_ROLL
      offset += op3_kd_->getJointDirection("l_hip_roll") * pelvis_offset_l;
    else if (i == joint_table_["r_hip_pitch"])
      offset -= op3_kd_->getJointDirection("r_hip_pitch") * hit_pitch_offset_;
    else if (i == joint_table_["l_hip_pitch"])  // R_HIP_PITCH or L_HIP_PITCH
      offset -= op3_kd_->getJointDirection("l_hip_pitch") * hit_pitch_offset_;

    leg_angle[i] += offset;
  }

  return true;
}

void WalkingModule::computeArmAngle(double *arm_angle)
{
  // Compute arm swing
  if (x_move_amplitude_ == 0)
  {
    arm_angle[0] = 0;  // Right
    arm_angle[1] = 0;  // Left
  }
  else
  {
    arm_angle[0] = wSin(time_, period_time_, M_PI * 1.5, -x_move_amplitude_ * arm_swing_gain_ * 1000,
                        0) * op3_kd_->getJointDirection("r_sho_pitch") * DEGREE2RADIAN;
    arm_angle[1] = wSin(time_, period_time_, M_PI * 1.5, x_move_amplitude_ * arm_swing_gain_ * 1000,
                        0) * op3_kd_->getJointDirection("l_sho_pitch") * DEGREE2RADIAN;
  }
}

void WalkingModule::sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle)
{
  // adjust balance offset
  if (walking_param_.balance_enable == false)
    return;

  double internal_gain = 0.05;

  balance_angle[joint_table_["r_hip_roll"]] =  op3_kd_->getJointDirection("r_hip_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_hip_roll_gain;  // R_HIP_ROLL
  balance_angle[joint_table_["l_hip_roll"]] =  op3_kd_->getJointDirection("l_hip_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_hip_roll_gain;  // L_HIP_ROLL

  balance_angle[joint_table_["r_knee"]] = - op3_kd_->getJointDirection("r_knee") * internal_gain
      * fbGyroErr * walking_param_.balance_knee_gain;  // R_KNEE
  balance_angle[joint_table_["l_knee"]] = - op3_kd_->getJointDirection("l_knee") * internal_gain
      * fbGyroErr * walking_param_.balance_knee_gain;  // L_KNEE

  balance_angle[joint_table_["r_ank_pitch"]] = - op3_kd_->getJointDirection("r_ank_pitch")
      * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // R_ANKLE_PITCH
  balance_angle[joint_table_["l_ank_pitch"]] = - op3_kd_->getJointDirection("l_ank_pitch")
      * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain;  // L_ANKLE_PITCH

  balance_angle[joint_table_["r_ank_roll"]] = - op3_kd_->getJointDirection("r_ank_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // R_ANKLE_ROLL
  balance_angle[joint_table_["l_ank_roll"]] = - op3_kd_->getJointDirection("l_ank_roll") * internal_gain
      * rlGyroErr * walking_param_.balance_ankle_roll_gain;  // L_ANKLE_ROLL

}

void WalkingModule::loadWalkingParam(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Fail to load yaml file.");
    return;
  }

  // parse movement time
  walking_param_.init_x_offset = doc["x_offset"].as<double>();
  walking_param_.init_y_offset = doc["y_offset"].as<double>();
  walking_param_.init_z_offset = doc["z_offset"].as<double>();
  walking_param_.init_roll_offset = doc["roll_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_pitch_offset = doc["pitch_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.init_yaw_offset = doc["yaw_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset = doc["hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
  // time
  walking_param_.period_time = doc["period_time"].as<double>() * 0.001;    // ms -> s
  walking_param_.dsp_ratio = doc["dsp_ratio"].as<double>();
  walking_param_.step_fb_ratio = doc["step_forward_back_ratio"].as<double>();
  // walking
  // walking_param_.x_move_amplitude
  // walking_param_.y_move_amplitude
  walking_param_.z_move_amplitude = doc["foot_height"].as<double>();
  // walking_param_.angle_move_amplitude
  // walking_param_.move_aim_on

  // balance
  // walking_param_.balance_enable
  walking_param_.balance_hip_roll_gain = doc["balance_hip_roll_gain"].as<double>();
  walking_param_.balance_knee_gain = doc["balance_knee_gain"].as<double>();
  walking_param_.balance_ankle_roll_gain = doc["balance_ankle_roll_gain"].as<double>();
  walking_param_.balance_ankle_pitch_gain = doc["balance_ankle_pitch_gain"].as<double>();
  walking_param_.y_swap_amplitude = doc["swing_right_left"].as<double>();
  walking_param_.z_swap_amplitude = doc["swing_top_down"].as<double>();
  walking_param_.pelvis_offset = doc["pelvis_offset"].as<double>() * DEGREE2RADIAN;
  walking_param_.arm_swing_gain = doc["arm_swing_gain"].as<double>();

  // gain
  walking_param_.p_gain = doc["p_gain"].as<int>();
  walking_param_.i_gain = doc["i_gain"].as<int>();
  walking_param_.d_gain = doc["d_gain"].as<int>();

  // Opsional: berkas lama tanpa kunci ini tetap terbaca, soft start bawaan 2 dtk.
  if (doc["soft_start_time"])
  {
    const double value = doc["soft_start_time"].as<double>();
    if (std::isfinite(value) && value >= 0.0 && value <= SOFT_START_MAX_SEC)
      soft_start_time_ = value;
  }
}

void WalkingModule::persistSoftStartTime(double seconds)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  // param_path_ ada di install/share dan merupakan SYMLINK ke src/.../param.yaml
  // (--symlink-install). Tulis ke berkas aslinya: mengganti nama tmp ke jalur
  // symlink akan menimpa symlink-nya dengan berkas biasa di install/, dan nilai
  // tak lagi sampai ke src.
  const fs::path real = fs::canonical(param_path_, ec);
  if (ec)
  {
    RCLCPP_WARN(this->get_logger(), "soft_start_time tidak tersimpan: %s tidak ditemukan (%s)",
                param_path_.c_str(), ec.message().c_str());
    return;
  }

  std::ifstream in(real);
  if (!in)
  {
    RCLCPP_WARN(this->get_logger(), "soft_start_time tidak tersimpan: gagal membaca %s", real.c_str());
    return;
  }
  std::string line, out;
  bool replaced = false;
  std::ostringstream wanted;
  wanted << "soft_start_time: " << seconds;
  while (std::getline(in, line))
  {
    if (replaced == false && line.rfind("soft_start_time:", 0) == 0)
    {
      out += wanted.str() + "\n";
      replaced = true;
    }
    else
    {
      out += line + "\n";   // tiap baris diakhiri newline -- tak ada lagi "0# ..."
    }
  }
  in.close();
  if (replaced == false)
    out += wanted.str() + "\n";

  // Tulis ke berkas sementara di direktori yang sama lalu ganti nama: kalau
  // proses mati di tengah jalan, param.yaml lama tetap utuh, bukan terpotong
  // (param.yaml yang gagal diurai membuat walking module memakai fallback C++).
  const fs::path tmp = real.string() + ".tmp";
  {
    std::ofstream o(tmp);
    o << out;
    if (!o)
    {
      RCLCPP_WARN(this->get_logger(), "soft_start_time tidak tersimpan: gagal menulis %s", tmp.c_str());
      return;
    }
  }
  // Warisi pemilik & izin berkas asli. op3_manager berjalan sebagai root di
  // kontainer, jadi berkas baru hasil rename akan jadi milik root 644 -- dan
  // operator di host tak bisa lagi menyunting param.yaml (terjadi pada uji
  // pertama fitur ini: bascorro 664 -> root 644).
  struct stat original {};
  if (::stat(real.c_str(), &original) == 0)
  {
    if (::chown(tmp.c_str(), original.st_uid, original.st_gid) != 0)
      RCLCPP_WARN(this->get_logger(), "soft_start_time: gagal mewarisi pemilik param.yaml");
    ::chmod(tmp.c_str(), original.st_mode & 07777);
  }
  fs::rename(tmp, real, ec);
  if (ec)
  {
    RCLCPP_WARN(this->get_logger(), "soft_start_time tidak tersimpan: %s", ec.message().c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "soft_start_time %.2f dtk disimpan ke %s", seconds, real.c_str());
}

void WalkingModule::saveWalkingParam(std::string &path)
{
  YAML::Emitter out_emitter;

  out_emitter << YAML::BeginMap;
  out_emitter << YAML::Key << "x_offset" << YAML::Value << walking_param_.init_x_offset;
  out_emitter << YAML::Key << "y_offset" << YAML::Value << walking_param_.init_y_offset;
  out_emitter << YAML::Key << "z_offset" << YAML::Value << walking_param_.init_z_offset;
  out_emitter << YAML::Key << "roll_offset" << YAML::Value << walking_param_.init_roll_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "pitch_offset" << YAML::Value << walking_param_.init_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "yaw_offset" << YAML::Value << walking_param_.init_yaw_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "hip_pitch_offset" << YAML::Value << walking_param_.hip_pitch_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "period_time" << YAML::Value << walking_param_.period_time * 1000;
  out_emitter << YAML::Key << "dsp_ratio" << YAML::Value << walking_param_.dsp_ratio;
  out_emitter << YAML::Key << "step_forward_back_ratio" << YAML::Value << walking_param_.step_fb_ratio;
  out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.z_move_amplitude;
  out_emitter << YAML::Key << "swing_right_left" << YAML::Value << walking_param_.y_swap_amplitude;
  out_emitter << YAML::Key << "swing_top_down" << YAML::Value << walking_param_.z_swap_amplitude;
  out_emitter << YAML::Key << "pelvis_offset" << YAML::Value << walking_param_.pelvis_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "arm_swing_gain" << YAML::Value << walking_param_.arm_swing_gain;
  out_emitter << YAML::Key << "balance_hip_roll_gain" << YAML::Value << walking_param_.balance_hip_roll_gain;
  out_emitter << YAML::Key << "balance_knee_gain" << YAML::Value << walking_param_.balance_knee_gain;
  out_emitter << YAML::Key << "balance_ankle_roll_gain" << YAML::Value << walking_param_.balance_ankle_roll_gain;
  out_emitter << YAML::Key << "balance_ankle_pitch_gain" << YAML::Value << walking_param_.balance_ankle_pitch_gain;

  out_emitter << YAML::Key << "p_gain" << YAML::Value << walking_param_.p_gain;
  out_emitter << YAML::Key << "i_gain" << YAML::Value << walking_param_.i_gain;
  out_emitter << YAML::Key << "d_gain" << YAML::Value << walking_param_.d_gain;
  out_emitter << YAML::Key << "soft_start_time" << YAML::Value << soft_start_time_.load();
  out_emitter << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out_emitter.c_str();
}

void WalkingModule::onModuleEnable()
{
  walking_state_ = WalkingEnable;
  // Tangkap goal dxl saat ini sebagai stance WALKING_READY pada tick process()
  // berikutnya -- itulah dasar mentah yang dipakai gait.
  capture_init_pose_ = true;
  RCLCPP_INFO(this->get_logger(), "Walking Enable");
}

void WalkingModule::onModuleDisable()
{
  RCLCPP_INFO(this->get_logger(), "Walking Disable");
  walking_state_ = WalkingDisable;
  // Modul lain memegang sendi sekarang; bias menggambarkan pose yang sudah tidak
  // berlaku. Mengaktifkan ulang akan menangkap dari awal, jadi jangan sekali-kali
  // melacak offset terhadap bias yang basi.
  walking_bias_valid_ = false;
}

// Sudut sendi kaki untuk stance diam (swap / leg_move nol) -- neutral yang
// dikelilingi ayunan gait. Titik ujungnya sama dengan computeLegAngle() saat
// swap = leg_move = 0, jadi neutral[0..5] kaki kanan, neutral[6..11] kaki kiri
// (terindeks seperti joint_table_). Mengembalikan false kalau IK tak bersolusi.
bool WalkingModule::computeNeutralLegAngle(double *neutral, bool with_y_offset)
{
  updatePoseParam();
  double leg_length = op3_kd_->thigh_length_m_ + op3_kd_->calf_length_m_ + op3_kd_->ankle_length_m_;

  // Offset lateral adalah SATU-SATUNYA offset yang diperlakukan berbeda oleh dua
  // pemanggilnya, karena ia satu-satunya tuas lebar pijakan.
  //
  // Semua yang dimasukkan ke acuan BIAS akan dikurangkan lagi lewat walking_bias_,
  // jadi offset yang hadir di kedua tempat sama sekali tidak berefek pada robot --
  // itu persis yang dulu membuat y_offset mati suri. Maka penangkapan
  // (with_y_offset == false) meninggalkannya; yang tersisa adalah tambahan lebar
  // pijakan nyata di atas stance page 2.
  //
  // Pelacakan saat diam mengirim true. Di sana tidak ada peniadaan -- ia menambah
  // pose ini ke bias yang sudah tetap -- jadi menyertakan y_offset membuat lebar
  // yang sama muncul saat robot cuma berdiri. Tanpa itu telapak baru melebar saat
  // "start" ditekan dan merapat lagi saat "stop".
  //
  // Offset lain ikut di kedua pemanggil: mereka hanya menetapkan titik kerja yang
  // dilinearisasi oleh ayunan gait.
  const double y_ep = (with_y_offset == true) ? y_offset_ / 2 : 0.0;

  double ep[12];
  ep[0] = x_offset_;        ep[1]  = -y_ep;           ep[2]  = z_offset_ - leg_length;
  ep[3] = -r_offset_ / 2;   ep[4]  = p_offset_;       ep[5]  = -a_offset_ / 2;
  ep[6] = x_offset_;        ep[7]  =  y_ep;           ep[8]  = z_offset_ - leg_length;
  ep[9] =  r_offset_ / 2;   ep[10] = p_offset_;       ep[11] =  a_offset_ / 2;

  if (op3_kd_->calcInverseKinematicsForRightLeg(&neutral[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
    return false;
  if (op3_kd_->calcInverseKinematicsForLeftLeg(&neutral[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
    return false;

  // hip pitch offset (pelvis offset is 0 at the neutral stance), matching computeLegAngle()
  neutral[joint_table_["r_hip_pitch"]] -= op3_kd_->getJointDirection("r_hip_pitch") * hit_pitch_offset_;
  neutral[joint_table_["l_hip_pitch"]] -= op3_kd_->getJointDirection("l_hip_pitch") * hit_pitch_offset_;

  // IK analitik bisa mengembalikan angka tak-hingga untuk pose telapak yang tak
  // terjangkau. Tolak di sini, jangan sampai NaN merambat ke goal_position_.
  for (int i = 0; i < 12; i++)
  {
    if (std::isfinite(neutral[i]) == false)
      return false;
  }

  return true;
}

bool WalkingModule::computeReferenceLegAngle(double *neutral)
{
  // Rumus titik ujung SAMA dengan computeNeutralLegAngle(..., false), hanya
  // offsetnya dari REF_* -- supaya bias + neutral(offset rujukan) tepat sama
  // dengan pose tertangkap.
  const double leg_length = op3_kd_->thigh_length_m_ + op3_kd_->calf_length_m_ + op3_kd_->ankle_length_m_;
  const double x = REF_X_OFFSET;
  const double z = REF_Z_OFFSET;
  const double r = REF_ROLL_OFFSET_DEG * DEGREE2RADIAN;
  const double p = REF_PITCH_OFFSET_DEG * DEGREE2RADIAN;
  const double a = REF_YAW_OFFSET_DEG * DEGREE2RADIAN;
  const double hip = REF_HIP_PITCH_OFFSET_DEG * DEGREE2RADIAN;

  if (op3_kd_->calcInverseKinematicsForRightLeg(&neutral[0], x, 0.0, z - leg_length, -r / 2, p, -a / 2) == false)
    return false;
  if (op3_kd_->calcInverseKinematicsForLeftLeg(&neutral[6], x, 0.0, z - leg_length, r / 2, p, a / 2) == false)
    return false;

  neutral[joint_table_["r_hip_pitch"]] -= op3_kd_->getJointDirection("r_hip_pitch") * hip;
  neutral[joint_table_["l_hip_pitch"]] -= op3_kd_->getJointDirection("l_hip_pitch") * hip;

  for (int i = 0; i < 12; i++)
  {
    if (std::isfinite(neutral[i]) == false)
      return false;
  }
  return true;
}

void WalkingModule::iniPoseTraGene(double mov_time)
{
  double smp_time = control_cycle_msec_ * 0.001;
  // Must match the row count produced by calcMinimumJerkTra() below, which uses
  // round(mov_time/smp_time + 1). Using int()+1 here yields one fewer row when
  // mov_time/smp_time has a fractional part >= 0.5 (e.g. 2.5s @ 8ms -> 312.5),
  // so the block assignment of `tra` triggered an Eigen resize assertion and
  // aborted op3_manager (SIGABRT) the moment the walking module was enabled.
  int all_time_steps = round(mov_time / smp_time + 1);
  // Satu kolom per sendi, bukan result_.size()+1. Kolom tambahan itu dulu ikut
  // diisi oleh loop "id <= result_.size()" di bawah, yang membaca
  // goal_position_.coeff(0, 14) dan target_position_.coeff(0, 14) -- keduanya
  // di luar batas matriks 1x14.
  calc_joint_tra_.resize(all_time_steps, result_.size());

  for (int id = 0; id < static_cast<int>(result_.size()); id++)
  {
    double ini_value = goal_position_.coeff(0, id);
    double tar_value = target_position_.coeff(0, id);

    Eigen::MatrixXd tra;

    tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, smp_time, mov_time);

    calc_joint_tra_.block(0, id, all_time_steps, 1) = tra;
  }

  if(DEBUG)
    std::cout << "Generate Trajecotry : " << mov_time << "s [" << all_time_steps << "]" << std::endl;

  init_pose_count_ = 0;
}
}

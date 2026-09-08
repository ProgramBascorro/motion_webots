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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "op3_walking_module/op3_walking_module.h"

namespace robotis_op
{

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

// DUA sumbu putar yang terpisah, keduanya bermuara ke leg_move.yaw lalu lewat IK
// menjadi r_hip_yaw (servo ID 7) dan l_hip_yaw (servo ID 8).
//
//  1. angle_move_amplitude  -- PERSIS asli ROBOTIS, tidak diubah sedikit pun:
//     kaki kiri memakai +a, kaki kanan -a (kedua hip yaw memutar berlawanan),
//     dan sapuannya terjadi saat kaki yang bersangkutan sedang di UDARA.
//
//  2. angle_move_amplitude_turn_in ("putar dalam") -- sumbu tambahan yang
//     DIJUMLAHKAN di atas yang pertama:
//       - kedua kaki memakai tanda yang SAMA (+t dan +t), jadi servo 7 dan 8
//         memutar ke arah yang sama -- inilah "putar dalam";
//       - satu kaki tidak bergerak yaw-nya selama telapaknya di udara, seluruh
//         sapuannya dipindah ke jendela DSP tepat SESUDAH telapak itu menapak;
//       - kaki mana yang mendapat perlakuan itu ditentukan TANDA nilainya:
//           t >  0 -> kaki KIRI (servo 8) yang berputar saat menapak, robot
//                     berputar searah jarum jam;
//           t <  0 -> CERMINANNYA: kaki KANAN (servo 7) yang berputar saat
//                     menapak, robot berputar berlawanan arah jarum jam,
//                     dengan sifat "ke dalam" yang sama.
//         Sisi minus dibentuk sebagai pencerminan sejati (tukar peran kiri/
//         kanan + balik tanda), bukan dengan memasukkan angka negatif ke rumus
//         yang sama -- lihat penjelasan panjang di computeLegAngle().
//     t = 0 membuat sumbu ini mati total dan gait kembali murni ROBOTIS.


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
  x_move_ramped_ = 0.0;
  y_move_ramped_ = 0.0;
  z_move_ramped_ = 0.0;
  a_move_ramped_ = 0.0;
  ai_move_ramped_ = 0.0;
  move_ramp_ = 0.0;

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
  // 25 deg/s: fast enough that a slider feels immediate, slow enough that jumping
  // z_offset by several centimetres in one keystroke ramps over about a second
  // instead of snapping. control_cycle_msec_ is re-read in initialize(), so this is
  // recomputed there too.
  idle_track_max_step_ = 25.0 * DEGREE2RADIAN * (control_cycle_msec_ * 0.001);
  joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size());

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
  // Depends on the cycle the controller actually runs at, not the constructor's guess.
  idle_track_max_step_ = 25.0 * DEGREE2RADIAN * (control_cycle_msec_ * 0.001);

  // m, s, rad
  // init pose — derived from WALKING_READY (action bin page 2) via forward
  // kinematics on the calibrated joint values, so walking_module's IK-generated
  // "walking ready" pose matches the user's calibrated standing pose. Only a
  // fallback for a missing/broken param.yaml — loadWalkingParam() overwrites all
  // of these — but a fallback that disagrees with page 2 means the robot stands
  // somewhere else entirely, so it is kept in step with param.yaml.
  // y is not FK-derived: see computeNeutralLegAngle(), which leaves the lateral offset
  // out of the bias reference so it stays a live foot-separation knob.
  walking_param_.init_x_offset = -0.02;
  walking_param_.init_y_offset = 0.028;
  walking_param_.init_z_offset = 0.0408;
  walking_param_.init_roll_offset = 0.0;
  walking_param_.init_pitch_offset = -0.07;   // rad, param.yaml pitch_offset: -4.0107 (deg)
  walking_param_.init_yaw_offset = 0.0 * DEGREE2RADIAN;
  walking_param_.hip_pitch_offset = 8.0 * DEGREE2RADIAN;
  // time
  walking_param_.period_time = 800 * 0.001;   // ms -> s, matches param.yaml
  walking_param_.dsp_ratio = 0.58;
  walking_param_.step_fb_ratio = 0.2;
  walking_param_.soft_start_time = 2.0;       // s, param.yaml soft_start_time: 2000 (ms)
  // walking
  walking_param_.x_move_amplitude = 0.0;
  walking_param_.y_move_amplitude = 0.0;
  walking_param_.z_move_amplitude = 0.037;    // foot height
  walking_param_.angle_move_amplitude = 0.0;
  walking_param_.angle_move_amplitude_turn_in = 0.0;
  // balance
  walking_param_.balance_enable = false;
  walking_param_.balance_hip_roll_gain = 0.35;
  walking_param_.balance_knee_gain = 0.4;
  walking_param_.balance_ankle_roll_gain = 0.7;
  walking_param_.balance_ankle_pitch_gain = 0.9;
  walking_param_.y_swap_amplitude = -0.002;
  walking_param_.z_swap_amplitude = 0.006;
  walking_param_.pelvis_offset = 3.0 * DEGREE2RADIAN;
  walking_param_.arm_swing_gain = 0.0;

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
                 5.0,       -5.0;
  init_position_ *= DEGREE2RADIAN;

  std::string default_param_path = ament_index_cpp::get_package_share_directory("op3_walking_module") + "/config/param.yaml";
  this->declare_parameter<std::string>("walking_param_path", default_param_path);
  this->get_parameter("walking_param_path", param_path_);

  loadWalkingParam(param_path_);

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
  else if (msg->data == "balance on")
    walking_param_.balance_enable = true;
  else if (msg->data == "balance off")
    walking_param_.balance_enable = false;
  else if (msg->data == "save")
    saveWalkingParam(param_path_);
}

void WalkingModule::walkingParameterCallback(const op3_walking_module_msgs::msg::WalkingParam::SharedPtr msg)
{
  // Amplitudo yang dipakai robot adalah keadaan tersendiri
  // (x/y/z/a_move_ramped_) yang mengejar nilai di sini dengan laju terbatas,
  // jadi Apply di tengah jalan otomatis dikejar bertahap dan "stop" tetap
  // berhenti seketika (mengecil tidak pernah ditahan).
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

double WalkingModule::rampToward(double current, double target, double dt)
{
  const double soft = walking_param_.soft_start_time;
  if (soft <= 0.0)
    return target;    // knob dimatikan: langsung, persis perilaku lama

  // Mengecil ke arah nol -- termasuk "stop" yang menolkan target -- tidak pernah
  // ditahan: itu arah yang mengurangi beban, dan menahannya justru membuat robot
  // telat berhenti. Ganti tanda (mis. maju lalu mundur) TIDAK terhitung mengecil,
  // jadi tetap dilewatkan pembatas.
  if (fabs(target) <= fabs(current) && current * target >= 0.0)
    return target;

  // Laju dipatok dari besar target, jadi perjalanan 0 -> target selalu memakan
  // tepat soft_start_time, berapa pun nilainya.
  const double step = fabs(target) * dt / soft;
  if (target > current)
    return (current + step > target) ? target : current + step;
  return (current - step < target) ? target : current - step;
}

double WalkingModule::timeRampScale()
{
  // ramp 0 -> mulai SOFT_START_SLOW_ kali lebih lambat; ramp 1 -> tempo penuh.
  return 1.0 + (SOFT_START_SLOW_ - 1.0) * (1.0 - move_ramp_);
}

void WalkingModule::updateTimeParam(double scale)
{
  period_time_ = scale * timeRampScale() * walking_param_.period_time;  // * 1000;   // s -> ms
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
  // Soft start: yang dipakai bukan nilai dari Studio langsung, melainkan nilai
  // yang sudah dibatasi lajunya di process(). Hanya amplitudo grup Movement --
  // y_swap dan z_swap milik grup Gait shape, sengaja tidak ikut. Lihat
  // op3_walking_module.h.

  // Forward/Back
  x_move_amplitude_ = x_move_ramped_;
  x_swap_amplitude_ = x_move_ramped_ * walking_param_.step_fb_ratio;

  if (previous_x_move_amplitude_ == 0)
  {
    x_move_amplitude_ *= 0.5;
    x_swap_amplitude_ *= 0.5;
  }

  // Right/Left
  y_move_amplitude_ = y_move_ramped_ / 2;
  if (y_move_amplitude_ > 0)
    y_move_amplitude_shift_ = y_move_amplitude_;
  else
    y_move_amplitude_shift_ = -y_move_amplitude_;
  y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;

  z_move_amplitude_ = z_move_ramped_ / 2;
  z_move_amplitude_shift_ = z_move_amplitude_ / 2;
  z_swap_amplitude_ = walking_param_.z_swap_amplitude;
  z_swap_amplitude_shift_ = z_swap_amplitude_;

  // Direction
  if (walking_param_.move_aim_on == false)
  {
    a_move_amplitude_ = a_move_ramped_ / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = a_move_amplitude_;
    else
      a_move_amplitude_shift_ = -a_move_amplitude_;
  }
  else
  {
    a_move_amplitude_ = -a_move_ramped_ / 2;
    if (a_move_amplitude_ > 0)
      a_move_amplitude_shift_ = -a_move_amplitude_;
    else
      a_move_amplitude_shift_ = a_move_amplitude_;
  }

  // "Putar dalam". Hanya BESAR dan TANDA yang diambil dari sini -- tidak ada
  // pasangan "shift" seperti a_move_ di atas. Konvensi ROBOTIS shift =
  // |amplitudo| memang sengaja tidak dipakai di sumbu ini: itulah yang dulu
  // membuat nilai minus cuma menggeser fasa, bukan membalik arah putaran.
  // computeLegAngle() yang membentuk sisi minus, sebagai cerminan sisi plus.
  // Tetap ikut move_aim_on supaya kedua sumbu putar tidak pernah tidak sinkron.
  ai_move_amplitude_ = (walking_param_.move_aim_on == false)
                           ? (ai_move_ramped_ / 2)
                           : (-ai_move_ramped_ / 2);
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
  ctrl_running_ = true;
  real_running_ = true;

  // Reset the gait clock to the start of a cycle. A stop leaves time_ parked on
  // whichever phase boundary cleared real_running_ (time_ only advances while
  // running), so without this reset a restart resumes mid-cycle and the move
  // amplitudes — applied only at PHASE1/PHASE3 via updateMovementParam() — may not
  // take effect until the NEXT full cycle; the robot looked like it "didn't move
  // again" after pressing START a second time.
  //
  // Soft start mulai dari nol di sini (lama: walking_param_.soft_start_time).
  // Pelambatan tempo 1.5x yang dulu ditulis sebagai updateTimeParam(1.5) sekarang
  // datang dari timeRampScale(): nilai awalnya sama persis, bedanya luruh mulus
  // alih-alih balik mendadak ke tempo penuh setelah satu siklus. soft_start_time
  // 0 membuat ramp langsung penuh, jadi perilakunya kembali seperti dulu.
  x_move_ramped_ = 0.0;
  y_move_ramped_ = 0.0;
  z_move_ramped_ = 0.0;
  a_move_ramped_ = 0.0;
  ai_move_ramped_ = 0.0;
  move_ramp_ = (walking_param_.soft_start_time > 0.0) ? 0.0 : 1.0;
  updateTimeParam();
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
    for (int id = 1; id <= result_.size(); id++)
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

    if (capture_init_pose_ == true)
    {
      captured_init_pose_ = goal_position_;
      walking_bias_ = goal_position_;
      double neutral_leg[12];
      // false: keep init_y_offset out of the bias so it does not cancel itself out.
      //
      // bias_reference_param_ (isi param.yaml), BUKAN walking_param_ yang hidup.
      // Kalau memakai yang hidup, offset preset yang baru saja di-Apply dikurangkan
      // terhadap dirinya sendiri: hasilnya nol, robot berdiri persis di page 2, dan
      // init_x/init_pitch operator tidak pernah terasa. Terukur 2026-09-09: Apply
      // SEBELUM enable menggeser stance 0,2-0,5 deg (nol), Apply SESUDAH enable
      // menggeser 64,8 deg -- nilai yang sama, dua hasil berbeda.
      walking_bias_valid_ = computeNeutralLegAngle(neutral_leg, false, bias_reference_param_);
      if (walking_bias_valid_ == true)
      {
        for (int i = 0; i < 12; i++)
          walking_bias_.coeffRef(0, i) = goal_position_.coeff(0, i) - neutral_leg[i];
      }
      capture_init_pose_ = false;
    }

    bool get_angle = false;
    // Hold the WALKING_READY stance when walking is idle (no gait running), so stance
    // never snaps back to init_position_ zero between Enable and Start. While
    // running, every joint (including arms) oscillates around the captured
    // pose via walking_bias_ so the gait centers on WALKING_READY.
    // ctrl_running_ has to be part of the test: "stop" only clears ctrl_running_, and
    // it is processPhase() itself that clears real_running_ at the next phase
    // boundary, so the gait pipeline below must keep running all the way through a
    // stop. Testing real_running_ alone would be equivalent today, but it hides that
    // dependency the moment anything else touches ctrl_running_.
    const bool walking_idle = (ctrl_running_ == false && real_running_ == false);

    // Standing still, the legs track the neutral stance of the *current* Init Pose
    // offsets instead of a frozen copy of the captured pose. Right after the capture
    // the two are identical by construction (walking_bias_ = captured - neutral, so
    // walking_bias_ + neutral == captured), so enabling the module still causes no
    // jump. What changes is that new offsets published on /robotis/walking/set_params
    // now show on the robot immediately: the stance shifts by exactly
    // (neutral_new - neutral_old) without having to start the gait first. That is the
    // only way to see x/z_offset at all while standing — computeLegAngle(), the only
    // other place they are read, runs solely while walking.
    // Falls back to the frozen pose whenever the bias is not trustworthy (IK failure
    // at capture time) or the IK has no solution for the offsets just entered.
    // true: unlike the capture above, the idle pose *does* carry init_y_offset, so the
    // feet stand as far apart as the gait will hold them. Standing and walking then
    // have the same stance width and "start"/"stop" no longer pull the legs sideways.
    double idle_leg[12];
    bool idle_tracks_offsets = false;
    if (walking_idle == true && walking_bias_valid_ == true)
      idle_tracks_offsets = computeNeutralLegAngle(idle_leg, true, walking_param_);

    // The gait pipeline only runs while a gait is actually running. It used to be
    // called every cycle, which cost a full leg IK plus a gyro feedback pass per 8 ms
    // for a result the idle branch below throws away, let computeLegAngle() log
    // "non-finite IK" while the robot was merely standing, and kept processPhase()
    // re-entering its phase branches (each one calls updateTimeParam()) forever at
    // whatever phase the last stop froze time_ at.
    //
    // Catatan lama di sini menyebut processPhase() ikut menolkan
    // x/y/angle_move_amplitude di dalam walking_param_ saat ctrl_running_ false.
    // Itu SUDAH TIDAK BERLAKU: sejak 2026-09-08 penghentian dikerjakan sasaran
    // ramp di bawah, dan walking_param_ tidak pernah lagi ditimpa. Menjauhkan
    // processPhase() dari jalur diam tetap berguna untuk alasan aslinya, yaitu
    // ongkos IK dan phase branch yang berputar percuma saat robot cuma berdiri.
    if (walking_idle == false)
    {
      processPhase(time_unit);

      get_angle = computeLegAngle(&angle[0]);

      computeArmAngle(&angle[12]);

      const double rl_gyro_err = 0.0 - sensors["gyro_x"];
      const double fb_gyro_err = 0.0 - sensors["gyro_y"];

      sensoryFeedback(rl_gyro_err, fb_gyro_err, balance_angle);
    }

    double err_total = 0.0, err_max = 0.0;
    // set goal position
    for (int idx = 0; idx < 14; idx++)
    {
      double goal_position = 0.0;
      if (walking_idle == true)
      {
        if (idle_tracks_offsets == true && idx < 12)
        {
          // Rate limit. A slider dragged in Bascorro Studio moves in small steps and
          // follows smoothly, but a value typed straight in is several degrees per
          // joint at once, and without a cap the servos would slam there in one 8 ms
          // cycle. Legs only; the arms keep the captured pose.
          // It also keeps the per-cycle error under the 5.0 deg threshold below, so
          // tracking never trips the WalkingInitPose trajectory path.
          const double target = walking_bias_.coeff(0, idx) + idle_leg[idx];
          const double present = goal_position_.coeff(0, idx);
          const double step = target - present;
          if (step > idle_track_max_step_)
            goal_position = present + idle_track_max_step_;
          else if (step < -idle_track_max_step_)
            goal_position = present - idle_track_max_step_;
          else
            goal_position = target;
        }
        else
          goal_position = captured_init_pose_.coeff(0, idx);
      }
      else if (get_angle == false && idx < 12)
        goal_position = goal_position_.coeff(0, idx);
      else
        goal_position = walking_bias_.coeff(0, idx) + angle[idx] + balance_angle[idx];

      target_position_.coeffRef(0, idx) = goal_position;

      double err = fabs(target_position_.coeff(0, idx) - goal_position_.coeff(0, idx)) * RADIAN2DEGREE;
      if (err > err_max)
        err_max = err;
      err_total += err;
    }

    // Check Enable
    if (walking_state_ == WalkingEnable && err_total > 5.0)
    {
      if (DEBUG)
        std::cout << "Check Err : " << err_max << std::endl;

      // Trajectory to the init pose, at roughly 30 deg/s with a 1.5 s floor. The
      // duration used to be an int, so err_max / 30 truncated: anything under 60 deg
      // collapsed to the same 1 s ramp and the robot lurched instead of easing in.
      // Doubles make it proportional again, and the higher floor keeps a small
      // correction gentle. (Matched from ORION_NEW.)
      double mov_time = err_max / 30.0;
      iniPoseTraGene(mov_time < 1.5 ? 1.5 : mov_time);

      // set target to goal
      target_position_ = goal_position_;

      walking_state_ = WalkingInitPose;

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
    // Soft start maju per siklus kontrol, bukan per batas fase, supaya lamanya
    // tetap soft_start_time berapa pun period_time-nya. Nilainya baru dipakai
    // saat updateMovementParam()/updateTimeParam() dipanggil di batas fase.
    //
    // BERHENTI = sasaran ramp jadi NOL, bukan setelannya yang dinolkan.
    //
    // Versi ROBOTIS asli menghentikan gait dengan menimpa walking_param_ jadi
    // nol di batas fase (lihat processPhase()). Itu berhasil menghentikan
    // robot, tapi ikut MEMBUANG setelan operator: nilai yang sama juga yang
    // dikembalikan /robotis/walking/get_params, jadi sesudah Apply lalu Start
    // lalu Stop, Bascorro Studio membaca 0 dan setelannya terlihat hilang
    // sendiri. Paling kentara pada gerakan MENCARI CELAH, karena di sana
    // angle_move_amplitude (-0,2) adalah SATU SATUNYA gerakan yang dipunyai
    // preset itu: sesudah Stop, presetnya jadi sama dengan berdiri diam.
    //
    // z SENGAJA tidak ikut dinolkan: tinggi angkat kaki bukan arah gerak, dan
    // versi lama pun tidak pernah menyentuhnya.
    const double move_target_x = (ctrl_running_ == true) ? walking_param_.x_move_amplitude : 0.0;
    const double move_target_y = (ctrl_running_ == true) ? walking_param_.y_move_amplitude : 0.0;
    const double move_target_a = (ctrl_running_ == true) ? walking_param_.angle_move_amplitude : 0.0;
    const double move_target_ai = (ctrl_running_ == true) ? walking_param_.angle_move_amplitude_turn_in : 0.0;

    x_move_ramped_ = rampToward(x_move_ramped_, move_target_x, time_unit);
    y_move_ramped_ = rampToward(y_move_ramped_, move_target_y, time_unit);
    z_move_ramped_ = rampToward(z_move_ramped_, walking_param_.z_move_amplitude, time_unit);
    a_move_ramped_ = rampToward(a_move_ramped_, move_target_a, time_unit);
    ai_move_ramped_ = rampToward(ai_move_ramped_, move_target_ai, time_unit);

    if (move_ramp_ < 1.0)
    {
      if (walking_param_.soft_start_time > 0.0)
        move_ramp_ += time_unit / walking_param_.soft_start_time;
      else
        move_ramp_ = 1.0;   // knob dimatikan di tengah jalan

      if (move_ramp_ > 1.0)
        move_ramp_ = 1.0;
    }

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
    if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0 && ai_move_amplitude_ == 0)
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
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0 && ai_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      else
      {
        // walking_param_ SENGAJA TIDAK disentuh di sini. Penghentiannya
        // dikerjakan sasaran ramp di process(); menolkan nilai tersimpan hanya
        // membuang setelan operator. Lihat catatan panjang di sana.
        //
        // Yang tetap dinolkan cuma pembukuan langkah: gait berikutnya berangkat
        // dari diam lagi, jadi potongan setengah langkah pertama di
        // updateMovementParam() harus dipasang ulang.
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
      if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0 && ai_move_amplitude_ == 0)
      {
        real_running_ = false;
      }
      // Tidak ada lagi cabang "else" yang menimpa walking_param_. Versi lama
      // menulis ulang x dengan previous_x_move_amplitude_, dan karena PHASE0 di
      // atas sudah menolkan penghitung itu, nilainya juga selalu 0 -- jadi
      // tidak ada perilaku yang hilang, cuma setelan operator yang selamat.
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

  // ===== Sumbu tambahan: "putar dalam" (angle_move_amplitude_turn_in) =====
  //
  // Dijumlahkan DI ATAS hasil gait asli di atas. Kalau nilainya 0 blok ini tidak
  // mengubah apa pun, jadi gait kembali persis ROBOTIS.
  //
  // Bedanya dengan sumbu asli ada dua:
  //   (a) kedua kaki memakai amplitudo bertanda SAMA -- gait asli memakai +a
  //       untuk kiri dan -a untuk kanan, jadi di sana kedua hip yaw memutar
  //       berlawanan; di sini keduanya searah, itulah "putar dalam";
  //   (b) SATU kaki yaw-nya dibekukan selama telapaknya di udara, dan seluruh
  //       sapuannya dipindah ke jendela DSP tepat sesudah telapak itu menapak.
  //
  // Sisi MINUS dibuat sebagai CERMINAN sejati sisi plus, bukan sekadar
  // memasukkan angka negatif ke rumus yang sama. Ini pernah salah: konvensi
  // ROBOTIS "shift = |amplitudo|" membuat sinyalnya tidak pernah bertanda
  // negatif, jadi memberi amplitudo minus hanya MENGGESER fasa setengah siklus
  // -- putarannya tetap searah jarum jam, cuma sifatnya berubah dari "ke dalam"
  // jadi "ke luar". Yang benar: tukar peran kiri/kanan (geser setengah siklus,
  // sebab r_ssp = l_ssp + period/2) DAN balik tandanya. Itulah pencerminan
  // terhadap bidang sagital, dan itu yang membuat putarannya jadi berlawanan
  // arah jarum jam dengan sifat "ke dalam" yang sama.
  if (ai_move_amplitude_ != 0.0)
  {
    const double ai_phase_l = a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_;
    const double ai_phase_r = a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI;

    // Bentuk dasarnya SELALU dihitung dengan magnitudo positif. Tanda dipakai
    // hanya untuk memilih cabang di bawah, tidak pernah masuk ke rumus -- itu
    // yang menjamin kedua sisi benar-benar bercermin.
    const double ai_mag = fabs(ai_move_amplitude_);

    // Lebar jendela DSP tepat sesudah telapak KIRI mendarat (kiri mendarat ->
    // kanan mulai diangkat). Dipakai sebagai SETENGAH periode sinus, jadi busur
    // yang terpakai pas -pi/2 -> +pi/2: berangkat dari nilai yang sedang
    // dipegang dan berhenti tepat di nilai yang dipakai fase berikutnya, jadi
    // tidak ada lompatan sudut di batas fase mana pun.
    const double ai_l_span = r_ssp_start_time_ - l_ssp_end_time_;

    // Pola asli ROBOTIS untuk satu kaki, struktur waktunya persis rantai
    // if/else di atas. Ditulis sebagai fungsi dari t supaya bisa dipanggil pada
    // waktu tergeser saat membuat cerminannya.
    auto ai_base = [&](double t) -> double
    {
      if (t <= l_ssp_start_time_)
        return wSin(l_ssp_start_time_, a_move_period_time_, ai_phase_l, ai_mag, ai_mag);
      if (t <= l_ssp_end_time_)
        return wSin(t, a_move_period_time_, ai_phase_l, ai_mag, ai_mag);
      if (t <= r_ssp_start_time_)
        return wSin(l_ssp_end_time_, a_move_period_time_, ai_phase_l, ai_mag, ai_mag);
      if (t <= r_ssp_end_time_)
        return wSin(t, a_move_period_time_, ai_phase_r, ai_mag, ai_mag);
      return wSin(r_ssp_end_time_, a_move_period_time_, ai_phase_r, ai_mag, ai_mag);
    };

    // Pola "berputar hanya saat menapak", ditulis untuk KAKI KIRI. Versi kaki
    // kanan tidak perlu ditulis sendiri: ia lahir dari pencerminan di bawah.
    auto ai_planted_left = [&](double t) -> double
    {
      if (ai_l_span <= 1e-6)
        return ai_base(t);                      // dsp_ratio 0: tidak ada jendela menapak
      if (t > l_ssp_start_time_ && t <= l_ssp_end_time_)
        return wSin(l_ssp_start_time_, a_move_period_time_, ai_phase_l, ai_mag, ai_mag);  // beku, telapak di udara
      if (t > l_ssp_end_time_ && t <= r_ssp_start_time_)
        return wSin(t - l_ssp_end_time_, 2.0 * ai_l_span, M_PI / 2, ai_mag, ai_mag);      // sapuan, telapak menapak
      return ai_base(t);
    };

    double ai_left;
    double ai_right;

    // Pemilihan cabang memakai tanda ai_move_amplitude_, bukan tanda parameter
    // mentahnya: dengan move_aim_on mati (normal) keduanya sama persis, dan
    // kalau move_aim_on dinyalakan arah yaw dan kaki yang menyapu tetap ikut
    // berbalik bersama-sama.
    if (ai_move_amplitude_ > 0.0)
    {
      // KIRI (servo 8) yang berputar saat menapak.
      ai_left = ai_planted_left(time_);
      ai_right = ai_base(time_);
    }
    else
    {
      // Cerminannya: KANAN (servo 7) yang berputar saat menapak, arah berbalik.
      const double ai_t_mirror = fmod(time_ + period_time_ / 2.0, period_time_);
      ai_left = -ai_base(ai_t_mirror);
      ai_right = -ai_planted_left(ai_t_mirror);
    }

    left_leg_move.yaw += ai_left;
    right_leg_move.yaw += ai_right;
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
  // DIBATASI 1 pesan/detik. Kalau IK gagal, ia gagal TIAP siklus kontrol 8 ms,
  // dan pesan tanpa pembatas membanjiri log 125 kali sedetik sampai penyebabnya
  // justru tidak terbaca.
  //
  // Angkanya sengaja lengkap: sejak penjaga NaN dikembalikan di
  // op3_kinematics_dynamics, kegagalan ini muncul di SINI dengan posisi yang
  // diminta, bukan lagi sebagai "non-finite leg_angle[2]" yang tidak bisa
  // ditelusuri.
  const double leg_reach_mm = (op3_kd_->thigh_length_m_ + op3_kd_->calf_length_m_) * 1000.0;

  if (op3_kd_->calcInverseKinematicsForRightLeg(&leg_angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "IK KAKI KANAN gagal: kaki diminta ke x %.1f y %.1f z %.1f mm "
                          "(roll %.3f pitch %.3f yaw %.3f rad). Jangkauan tungkai cuma %.1f mm. "
                          "init_z_offset yang sedang hidup %.1f mm; makin kecil, makin lurus "
                          "kakinya dan makin sedikit sisa jangkauan untuk melangkah. "
                          "Siklus gait ini dilewati, kaki menahan goal terakhir.",
                          ep[0] * 1000.0, ep[1] * 1000.0, ep[2] * 1000.0, ep[3], ep[4], ep[5],
                          leg_reach_mm, z_offset_ * 1000.0);
    return false;
  }

  if (op3_kd_->calcInverseKinematicsForLeftLeg(&leg_angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "IK KAKI KIRI gagal: kaki diminta ke x %.1f y %.1f z %.1f mm "
                          "(roll %.3f pitch %.3f yaw %.3f rad). Jangkauan tungkai cuma %.1f mm. "
                          "init_z_offset yang sedang hidup %.1f mm; makin kecil, makin lurus "
                          "kakinya dan makin sedikit sisa jangkauan untuk melangkah. "
                          "Siklus gait ini dilewati, kaki menahan goal terakhir.",
                          ep[6] * 1000.0, ep[7] * 1000.0, ep[8] * 1000.0, ep[9], ep[10], ep[11],
                          leg_reach_mm, z_offset_ * 1000.0);
    return false;
  }

  // Reject non-finite IK results: a NaN would propagate to goal_position_ and
  // the controller silently discards the cycle, leaving legs frozen mid-gait.
  for (int i = 0; i < 12; i++)
  {
    if (!std::isfinite(leg_angle[i]))
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "IK produced non-finite leg_angle[%d]", i);
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
    // Acuan bias tetap harus terisi: nilai bawaan initialize() dipakai apa adanya,
    // supaya walking_bias_ tidak pernah dihitung terhadap struct kosong.
    bias_reference_param_ = walking_param_;
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
  // Kunci baru (2026-08-18). Dijaga karena param.yaml lama -- termasuk hasil
  // "save" sebelum tanggal itu, dan salinan di robot lain -- tidak punya kunci
  // ini; tanpa penjaga satu kunci hilang membuat SELURUH loadWalkingParam()
  // gagal dan semua parameter diam-diam balik ke fallback initialize().
  if (doc["soft_start_time"])
    walking_param_.soft_start_time = doc["soft_start_time"].as<double>() * 0.001;   // ms -> s
  // walking
  // walking_param_.x_move_amplitude
  // walking_param_.y_move_amplitude
  walking_param_.z_move_amplitude = doc["foot_height"].as<double>();
  // walking_param_.angle_move_amplitude
  // walking_param_.move_aim_on

  // balance
  // Dijaga karena kunci ini baru (2026-08-22): param.yaml lama -- termasuk hasil
  // "save" sebelum tanggal itu dan salinan di robot lain -- tidak punya kunci ini,
  // dan satu kunci hilang membuat SELURUH pemuatan gagal lalu semua parameter
  // diam-diam balik ke fallback initialize(). Sebelum ini balance_enable memang
  // tidak pernah dibaca dari yaml, jadi mematikannya dari Studio selalu kembali
  // menyala begitu ada Apply berikutnya yang menimpa seluruh struct.
  if (doc["balance_enable"])
    walking_param_.balance_enable = doc["balance_enable"].as<bool>();
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

  // Acuan tetap untuk walking_bias_, dikunci di sini SEKALI saja. Yang dipakai
  // hanya offset init_* dan hip_pitch_offset; sisanya ikut supaya strukturnya
  // utuh dan mudah dicetak saat menelusuri.
  //
  // Sengaja TIDAK ikut diperbarui oleh Apply maupun perintah "save": acuan yang
  // ikut bergeser persis itulah yang membuat nilai sama memberi hasil berbeda.
  // Sesudah param.yaml diubah, jalankan ulang op3_manager.
  bias_reference_param_ = walking_param_;
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
  out_emitter << YAML::Key << "soft_start_time" << YAML::Value << walking_param_.soft_start_time * 1000;
  out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.z_move_amplitude;
  out_emitter << YAML::Key << "swing_right_left" << YAML::Value << walking_param_.y_swap_amplitude;
  out_emitter << YAML::Key << "swing_top_down" << YAML::Value << walking_param_.z_swap_amplitude;
  out_emitter << YAML::Key << "pelvis_offset" << YAML::Value << walking_param_.pelvis_offset * RADIAN2DEGREE;
  out_emitter << YAML::Key << "arm_swing_gain" << YAML::Value << walking_param_.arm_swing_gain;
  out_emitter << YAML::Key << "balance_enable" << YAML::Value << walking_param_.balance_enable;
  out_emitter << YAML::Key << "balance_hip_roll_gain" << YAML::Value << walking_param_.balance_hip_roll_gain;
  out_emitter << YAML::Key << "balance_knee_gain" << YAML::Value << walking_param_.balance_knee_gain;
  out_emitter << YAML::Key << "balance_ankle_roll_gain" << YAML::Value << walking_param_.balance_ankle_roll_gain;
  out_emitter << YAML::Key << "balance_ankle_pitch_gain" << YAML::Value << walking_param_.balance_ankle_pitch_gain;

  out_emitter << YAML::Key << "p_gain" << YAML::Value << walking_param_.p_gain;
  out_emitter << YAML::Key << "i_gain" << YAML::Value << walking_param_.i_gain;
  out_emitter << YAML::Key << "d_gain" << YAML::Value << walking_param_.d_gain;
  out_emitter << YAML::EndMap;

  // output to file
  std::ofstream fout(path.c_str());
  fout << out_emitter.c_str();
}

void WalkingModule::onModuleEnable()
{
  walking_state_ = WalkingEnable;
  // Capture the current dxl goal as WALKING_READY stance on the next process() tick.
  capture_init_pose_ = true;
  RCLCPP_INFO(this->get_logger(), "Walking Enable");
}

void WalkingModule::onModuleDisable()
{
  RCLCPP_INFO(this->get_logger(), "Walking Disable");
  walking_state_ = WalkingDisable;
  // Another module owns the joints now; the bias describes a pose that no longer
  // holds. Re-enabling captures afresh, so never track offsets against a stale bias.
  walking_bias_valid_ = false;
}

bool WalkingModule::computeNeutralLegAngle(double *neutral, bool with_y_offset,
                                           const op3_walking_module_msgs::msg::WalkingParam &src)
{
  // IK of the standing pose with zero swap/move amplitudes — used to derive
  // walking_bias_ so the gait oscillates around the captured WALKING_READY instead
  // of init_position_ zeros.
  //
  // Offset dibaca LOKAL dari src, bukan lewat updatePoseParam(). Dua alasan:
  // pemanggilnya memang memakai set parameter yang berbeda (penangkapan bias
  // memakai acuan param.yaml, penjejakan diam memakai parameter hidup), dan
  // fungsi ini jadi tidak lagi punya efek samping ke x_offset_ dkk yang dipakai
  // computeLegAngle().
  const double x_off = src.init_x_offset;
  const double y_off = src.init_y_offset;
  const double z_off = src.init_z_offset;
  const double r_off = src.init_roll_offset;
  const double p_off = src.init_pitch_offset;
  const double a_off = src.init_yaw_offset;
  const double hip_pitch_off = src.hip_pitch_offset;

  double leg_length = op3_kd_->thigh_length_m_ + op3_kd_->calf_length_m_ + op3_kd_->ankle_length_m_;
  // The lateral offset is the one offset that is treated differently by the two
  // callers, because it is the one knob for foot separation.
  //
  // Everything put into the *bias* reference is subtracted from the gait again
  // through walking_bias_, so an offset present in both places has no effect on the
  // robot at all — that is exactly what used to make y_offset inert. So the capture
  // (with_y_offset == false) leaves it out: what survives is a real "extra foot
  // separation" on top of the captured page 2 stance.
  //
  // The idle tracking passes true instead. It has no such cancellation — it adds this
  // pose to an already-fixed bias — so including y_offset there puts the same
  // separation on the robot while it is merely standing. Without that the feet only
  // spread at the moment "start" is pressed and pull back together on stop, which
  // reads as the legs closing toward the centreline as soon as the gait ends. Now the
  // stance width is identical standing and walking, and Apply shows it immediately
  // (rate limited, like x/z/pitch) instead of hiding it until the first step.
  //
  // The other offsets stay in for both callers: they only set the operating point the
  // gait oscillation is linearised around.
  const double y_ep = (with_y_offset == true) ? y_off / 2 : 0.0;
  double ep[12];
  // right leg
  ep[0] = x_off;
  ep[1] = -y_ep;
  ep[2] = z_off - leg_length;
  ep[3] = -r_off / 2;
  ep[4] = p_off;
  ep[5] = -a_off / 2;
  // left leg
  ep[6] = x_off;
  ep[7] = y_ep;
  ep[8] = z_off - leg_length;
  ep[9] = r_off / 2;
  ep[10] = p_off;
  ep[11] = a_off / 2;

  if (op3_kd_->calcInverseKinematicsForRightLeg(&neutral[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
    return false;
  if (op3_kd_->calcInverseKinematicsForLeftLeg(&neutral[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
    return false;

  // Match the hip_pitch offset that computeLegAngle applies on top of the IK result.
  neutral[joint_table_["r_hip_pitch"]] -= op3_kd_->getJointDirection("r_hip_pitch") * hip_pitch_off;
  neutral[joint_table_["l_hip_pitch"]] -= op3_kd_->getJointDirection("l_hip_pitch") * hip_pitch_off;

  for (int i = 0; i < 12; i++)
  {
    if (!std::isfinite(neutral[i]))
      return false;
  }
  return true;
}

void WalkingModule::iniPoseTraGene(double mov_time)
{
  double smp_time = control_cycle_msec_ * 0.001;
  // Use round() so the row count matches what calcMinimumJerkTra() produces;
  // an off-by-one here triggers an Eigen assertion and kills op3_manager.
  int all_time_steps = round(mov_time / smp_time + 1);
  calc_joint_tra_.resize(all_time_steps, result_.size() + 1);

  for (int id = 0; id <= result_.size(); id++)
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

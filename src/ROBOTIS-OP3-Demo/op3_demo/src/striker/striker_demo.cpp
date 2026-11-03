#include "op3_demo/striker_demo.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>

#include "robotis_math/robotis_linear_algebra.h"

namespace robotis_op
{

namespace striker_demo
{

const char *toString(Mode mode)
{
  switch (mode)
  {
  case Mode::Stopped: return "STOPPED";
  case Mode::Running: return "RUNNING";
  case Mode::Stopping: return "STOPPING";
  }
  return "?";
}

const char *toString(State state)
{
  switch (state)
  {
  case State::Idle: return "AUTO_IDLE";
  case State::Init: return "INIT";
  case State::StandSearch: return "STAND_SEARCH";
  case State::SearchBall: return "SEARCH_BALL";
  case State::SearchAlign: return "SEARCH_ALIGN";
  case State::Approach: return "APPROACH_BALL";
  case State::AimLook: return "AIM_LOOK";
  case State::YawCorrection: return "YAW_CORRECTION";
  case State::DistanceRecheck: return "DISTANCE_RECHECK";
  case State::BallAlignment: return "BALL_ALIGNMENT";
  case State::Shift: return "SHIFT";
  case State::PreKickNudge: return "PRE_KICK_NUDGE";
  case State::Kick: return "KICK";
  case State::GettingUp: return "GETTING_UP";
  case State::Park: return "PARK";
  }
  return "?";
}

const char *toString(HeadMode head)
{
  switch (head)
  {
  case HeadMode::Inactive: return "INACTIVE";
  case HeadMode::Search: return "HEAD_MODE_SEARCH";
  case HeadMode::Track: return "HEAD_MODE_TRACK";
  }
  return "?";
}

const char *toString(Foot foot)
{
  switch (foot)
  {
  case Foot::Left: return "KIRI";
  case Foot::Right: return "KANAN";
  default: return "-";
  }
}

const char *toString(AlignDir dir)
{
  switch (dir)
  {
  case AlignDir::FromLeft: return "DARI-KIRI";
  case AlignDir::FromRight: return "DARI-KANAN";
  default: return "-";
  }
}

const char *toString(PresetRole role)
{
  switch (role)
  {
  case PresetRole::PoseReference: return "acuan-pose";
  case PresetRole::Init: return "init";
  case PresetRole::Search: return "mencari";
  case PresetRole::SearchInverted: return "mencari-yaw-dibalik";
  case PresetRole::Approach: return "mendekat";
  case PresetRole::TurnLeft: return "putar-kiri";
  case PresetRole::TurnRight: return "putar-kanan";
  case PresetRole::ShiftLeft: return "geser-kiri";
  case PresetRole::ShiftRight: return "geser-kanan";
  default: return "?";
  }
}

}  // namespace striker_demo

namespace
{

using striker_demo::kNaN;
using striker_demo::kNever;
using striker_demo::PresetRole;

constexpr double kPi = 3.14159265358979323846;
const char *kLogger = "StrikerDemo";

const std::vector<std::string> kLegJoints = {
  "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
  "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
};
// Persis sendi milik walking_module (op3_walking_module.cpp result_).
const std::vector<std::string> kWalkingJoints = {
  "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll",
  "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll",
  "r_sho_pitch", "l_sho_pitch",
};
const std::vector<std::string> kAllJoints = {
  "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
  "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll", "r_hip_pitch", "l_hip_pitch",
  "r_knee", "l_knee", "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
  "head_pan", "head_tilt",
};
// Enam offset yang dipatok Bascorro Studio sebelum walking_module menyala
// (WALKING_BIAS_FIELDS di web/src/walkingParams.js) = acuan page 2. Angkanya
// TIDAK ditulis di sini: dibaca dari walking_ready_reference.json, berkas yang
// sama dengan Studio (lihat loadStudioPresets).
const std::array<const char *, 6> kPoseReferenceFields = {
  "init_x_offset", "init_z_offset", "init_roll_offset",
  "init_pitch_offset", "init_yaw_offset", "hip_pitch_offset",
};

struct RoleParam
{
  PresetRole role;
  const char *parameter;
  const char *default_name;
};
// Nama PERSIS seperti tersimpan di walking_versions.json Bascorro Studio.
const std::vector<RoleParam> kRoleParams = {
  {PresetRole::Init, "auto_demo_preset_init", "JALAN MENUJU MEKKAH"},
  {PresetRole::Search, "auto_demo_preset_search", "MENCARI CELAH"},
  {PresetRole::Approach, "auto_demo_preset_approach", "JALAN BARU"},
  {PresetRole::TurnLeft, "auto_demo_preset_turn_left", "Putar Kiri New"},
  {PresetRole::TurnRight, "auto_demo_preset_turn_right", "Putar Kanan New"},
  {PresetRole::ShiftLeft, "auto_demo_preset_shift_left", "KIRI MIN NEW"},
  {PresetRole::ShiftRight, "auto_demo_preset_shift_right", "KANAN MIN NEW"},
};

double deg2rad(double deg) { return deg * kPi / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / kPi; }
bool happened(double t) { return std::isfinite(t); }

std::string format(const char *fmt, ...)
{
  char buffer[768];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return std::string(buffer);
}

double normalizeAngleDeg(double angle_deg)
{
  if (!std::isfinite(angle_deg))
    return angle_deg;
  angle_deg = std::fmod(angle_deg, 360.0);
  if (angle_deg > 180.0)
    angle_deg -= 360.0;
  else if (angle_deg <= -180.0)
    angle_deg += 360.0;
  return angle_deg;
}

// Konvensi derajat servo demo D56/D58: pusat 180 = 0 rad, arah +1.
double jointToServoDeg(double joint_rad, double center_deg, double direction)
{
  return center_deg + direction * rad2deg(joint_rad);
}

// 0 = menoleh paling kiri, 100 = paling kanan (dari batas sendi head_pan).
double panPercentOf(double servo19_deg, double left_limit_servo_deg, double right_limit_servo_deg)
{
  const double span = left_limit_servo_deg - right_limit_servo_deg;
  if (!std::isfinite(servo19_deg) || !std::isfinite(span) || std::fabs(span) < 1e-9)
    return kNaN;
  return (left_limit_servo_deg - servo19_deg) / span * 100.0;
}

bool setField(op3_walking_module_msgs::msg::WalkingParam &m, const std::string &name, double v)
{
  if (name == "init_x_offset") m.init_x_offset = v;
  else if (name == "init_y_offset") m.init_y_offset = v;
  else if (name == "init_z_offset") m.init_z_offset = v;
  else if (name == "init_roll_offset") m.init_roll_offset = v;
  else if (name == "init_pitch_offset") m.init_pitch_offset = v;
  else if (name == "init_yaw_offset") m.init_yaw_offset = v;
  else if (name == "period_time") m.period_time = v;
  else if (name == "dsp_ratio") m.dsp_ratio = v;
  else if (name == "step_fb_ratio") m.step_fb_ratio = v;
  else if (name == "soft_start_time") m.soft_start_time = v;
  else if (name == "x_move_amplitude") m.x_move_amplitude = v;
  else if (name == "y_move_amplitude") m.y_move_amplitude = v;
  else if (name == "z_move_amplitude") m.z_move_amplitude = v;
  else if (name == "angle_move_amplitude") m.angle_move_amplitude = v;
  else if (name == "angle_move_amplitude_turn_in") m.angle_move_amplitude_turn_in = v;
  else if (name == "balance_hip_roll_gain") m.balance_hip_roll_gain = v;
  else if (name == "balance_knee_gain") m.balance_knee_gain = v;
  else if (name == "balance_ankle_roll_gain") m.balance_ankle_roll_gain = v;
  else if (name == "balance_ankle_pitch_gain") m.balance_ankle_pitch_gain = v;
  else if (name == "y_swap_amplitude") m.y_swap_amplitude = v;
  else if (name == "z_swap_amplitude") m.z_swap_amplitude = v;
  else if (name == "arm_swing_gain") m.arm_swing_gain = v;
  else if (name == "pelvis_offset") m.pelvis_offset = v;
  else if (name == "hip_pitch_offset") m.hip_pitch_offset = v;
  else if (name == "p_gain") m.p_gain = static_cast<int32_t>(std::lround(v));
  else if (name == "i_gain") m.i_gain = static_cast<int32_t>(std::lround(v));
  else if (name == "d_gain") m.d_gain = static_cast<int32_t>(std::lround(v));
  else return false;
  return true;
}

op3_walking_module_msgs::msg::WalkingParam toWalkingParam(const StudioWalkingVersion &version)
{
  op3_walking_module_msgs::msg::WalkingParam msg;
  for (const auto &item : version.numbers)
    setField(msg, item.first, item.second);
  msg.move_aim_on = version.flags.at("move_aim_on");
  msg.balance_enable = version.flags.at("balance_enable");
  return msg;
}

bool sameFloat(float a, float b)
{
  return a == b || std::fabs(a - b) <= 1e-6f * std::max(1.0f, std::max(std::fabs(a), std::fabs(b)));
}

bool sameWalkingParam(const op3_walking_module_msgs::msg::WalkingParam &a,
                      const op3_walking_module_msgs::msg::WalkingParam &b, std::string &diff)
{
  const auto check = [&diff](const char *name, float x, float y) {
    if (!sameFloat(x, y))
      diff += std::string(diff.empty() ? "" : ", ") + name + " " + std::to_string(x) + "!=" + std::to_string(y);
  };
  check("init_x_offset", a.init_x_offset, b.init_x_offset);
  check("init_y_offset", a.init_y_offset, b.init_y_offset);
  check("init_z_offset", a.init_z_offset, b.init_z_offset);
  check("init_roll_offset", a.init_roll_offset, b.init_roll_offset);
  check("init_pitch_offset", a.init_pitch_offset, b.init_pitch_offset);
  check("init_yaw_offset", a.init_yaw_offset, b.init_yaw_offset);
  check("period_time", a.period_time, b.period_time);
  check("dsp_ratio", a.dsp_ratio, b.dsp_ratio);
  check("step_fb_ratio", a.step_fb_ratio, b.step_fb_ratio);
  check("soft_start_time", a.soft_start_time, b.soft_start_time);
  check("x_move_amplitude", a.x_move_amplitude, b.x_move_amplitude);
  check("y_move_amplitude", a.y_move_amplitude, b.y_move_amplitude);
  check("z_move_amplitude", a.z_move_amplitude, b.z_move_amplitude);
  check("angle_move_amplitude", a.angle_move_amplitude, b.angle_move_amplitude);
  check("angle_move_amplitude_turn_in", a.angle_move_amplitude_turn_in, b.angle_move_amplitude_turn_in);
  check("balance_hip_roll_gain", a.balance_hip_roll_gain, b.balance_hip_roll_gain);
  check("balance_knee_gain", a.balance_knee_gain, b.balance_knee_gain);
  check("balance_ankle_roll_gain", a.balance_ankle_roll_gain, b.balance_ankle_roll_gain);
  check("balance_ankle_pitch_gain", a.balance_ankle_pitch_gain, b.balance_ankle_pitch_gain);
  check("y_swap_amplitude", a.y_swap_amplitude, b.y_swap_amplitude);
  check("z_swap_amplitude", a.z_swap_amplitude, b.z_swap_amplitude);
  check("arm_swing_gain", a.arm_swing_gain, b.arm_swing_gain);
  check("pelvis_offset", a.pelvis_offset, b.pelvis_offset);
  check("hip_pitch_offset", a.hip_pitch_offset, b.hip_pitch_offset);
  if (a.move_aim_on != b.move_aim_on) diff += std::string(diff.empty() ? "" : ", ") + "move_aim_on";
  if (a.balance_enable != b.balance_enable) diff += std::string(diff.empty() ? "" : ", ") + "balance_enable";
  if (a.p_gain != b.p_gain || a.i_gain != b.i_gain || a.d_gain != b.d_gain)
    diff += std::string(diff.empty() ? "" : ", ") + "p/i/d_gain";
  return diff.empty();
}

std::string realPath(const std::string &path)
{
  char buffer[PATH_MAX];
  return (::realpath(path.c_str(), buffer) != nullptr) ? std::string(buffer) : path;
}

double jointPosition(const sensor_msgs::msg::JointState &msg, const std::string &joint)
{
  const size_t count = std::min(msg.name.size(), msg.position.size());
  for (size_t i = 0; i < count; ++i)
    if (msg.name[i] == joint)
      return msg.position[i];
  return kNaN;
}

// STRIKER: parameter bool (striker_aim_look_enable, striker_search_original_is_right).
bool paramBoolOf(const rclcpp::Node::SharedPtr &node, const std::string &name, bool fallback)
{
  if (node == nullptr || !node->has_parameter(name))
    return fallback;
  const rclcpp::Parameter parameter = node->get_parameter(name);
  return parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ? parameter.as_bool() : fallback;
}

std::vector<striker::Deteksi> toDeteksi(const vision_msgs::msg::Detection2DArray &msg)
{
  // Format yolo_gawang_detector.py: piksel gambar asli, score di results[0].
  std::vector<striker::Deteksi> out;
  for (const auto &det : msg.detections)
  {
    striker::Deteksi d;
    d.cx = det.bbox.center.position.x;
    d.cy = det.bbox.center.position.y;
    d.w = det.bbox.size_x;
    d.h = det.bbox.size_y;
    d.conf = det.results.empty() ? 0.0 : det.results[0].hypothesis.score;
    out.push_back(d);
  }
  return out;
}

const char *fallName(PitchFallState fall)
{
  switch (fall)
  {
  case PitchFallState::FallenForward: return "DEPAN (telungkup)";
  case PitchFallState::FallenBehind: return "BELAKANG (telentang)";
  default: return "BERDIRI";
  }
}

}  // namespace

// ============================================================================
//  Konstruksi & parameter
// ============================================================================

StrikerDemo::StrikerDemo()
{
  log_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  preset_info_.assign(static_cast<size_t>(PresetRole::Count), striker_demo::PresetInfo());
}

StrikerDemo::~StrikerDemo() = default;

double StrikerDemo::steadyNow() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void StrikerDemo::declareParameters()
{
  if (parameters_declared_ || node_ == nullptr)
    return;
  parameters_declared_ = true;

  // dynamic_typing: "1" dan "1.0" dari baris perintah sama-sama diterima
  // (exception tipe parameter pernah membunuh op_demo_node, lihat soccer_demo).
  rcl_interfaces::msg::ParameterDescriptor dynamic;
  dynamic.dynamic_typing = true;
  const auto declare = [this, &dynamic](const std::string &name, const rclcpp::ParameterValue &value) {
    if (!node_->has_parameter(name))
      node_->declare_parameter(name, value, dynamic);
  };

  declare("auto_demo_studio_versions_path", rclcpp::ParameterValue(std::string("")));
  // 19 Sep 2026: berkas versi KHUSUS preset mendekat D60 (kosong = berkas Studio).
  declare("auto_demo_preset_approach_file", rclcpp::ParameterValue(std::string("")));
  for (const RoleParam &item : kRoleParams)
    declare(item.parameter, rclcpp::ParameterValue(std::string(item.default_name)));

  const std::vector<std::pair<const char *, double>> numbers = {
    {"auto_demo_transition_sec", 0.4},
    {"auto_demo_initial_search_sec", 3.0},
    {"auto_demo_start_search_sec", 7.0},
    {"auto_demo_start_forward_sec", 20.0},
    {"auto_demo_search_to_walk_delay_sec", 0.0},
    {"auto_demo_prekick_nudge_sec", 0.3},
    {"auto_demo_post_action_search_sec", 1.5},
    {"auto_demo_lost_search_sec", 3.0},
    {"auto_demo_ball_valid_gap_sec", 5.0 / 30.0},
    {"auto_demo_ball_lost_sec", 1.0},
    {"auto_demo_servo_center_deg", 180.0},
    {"auto_demo_servo_dir", 1.0},
    {"auto_demo_head_pan_left_limit_deg", 85.0},
    {"auto_demo_head_pan_right_limit_deg", -85.0},
    {"auto_demo_pan_kick_left", 48.0},
    {"auto_demo_pan_center", 50.0},
    {"auto_demo_pan_kick_right", 52.0},
    {"auto_demo_pan_outer_left", 45.0},
    {"auto_demo_pan_outer_left_stop", 46.0},
    {"auto_demo_pan_outer_right", 55.0},
    {"auto_demo_pan_outer_right_stop", 54.0},
    {"auto_demo_tilt_limit_deg", 200.0},
    {"auto_demo_steer_gain", 0.2},
    {"auto_demo_steer_min_deg", 5.0},
    {"auto_demo_steer_max_deg", 15.0},
    {"auto_demo_steer_ramp_deg", 0.5},
    {"auto_demo_steer_deadband_deg", 5.0},
    {"auto_demo_steer_sign", 1.0},
    {"auto_demo_steer_publish_period_sec", 0.1},
    {"auto_demo_search_align_left_max", 30.0},
    {"auto_demo_search_align_right_min", 70.0},
    {"auto_demo_search_align_stop_left", 40.0},
    {"auto_demo_search_align_stop_right", 60.0},
    {"auto_demo_head_park_wait_sec", 1.5},
    {"auto_demo_head_park_tol_deg", 3.0},
    {"auto_demo_head_park_pan_deg", 0.0},
    {"auto_demo_head_park_tilt_servo_deg", 200.0},
    {"auto_demo_stop_head_tilt_servo_deg", 135.0},
    {"auto_demo_yaw_target_deg", 0.0},
    {"auto_demo_yaw_tolerance_deg", 6.0},
    {"auto_demo_yaw_stop_deg", 4.0},
    {"auto_demo_yaw_tolerance_neg_deg", 0.5},
    {"auto_demo_yaw_stop_neg_deg", 0.0},
    {"auto_demo_yaw_timeout_sec", 20.0},
    {"auto_demo_approach_timeout_sec", 30.0},
    {"auto_demo_shift_timeout_sec", 8.0},
    {"auto_demo_search_timeout_sec", 60.0},
    {"auto_demo_page_kick_left", 104.0},
    {"auto_demo_page_kick_right", 105.0},
    {"auto_demo_page_getup_front", 85.0},
    {"auto_demo_page_getup_back", 86.0},
  };
  for (const auto &item : numbers)
    declare(item.first, rclcpp::ParameterValue(item.second));
  // Kosong = walking_ready_reference.json di samping walking_versions.json.
  declare("auto_demo_pose_reference_path", rclcpp::ParameterValue(std::string("")));

  // ---- STRIKER (config/striker_params.yaml) ----------------------------------------
  const std::vector<std::pair<const char *, double>> striker_numbers = {
    {"striker_start_index", 0.0},
    {"striker_sector_gate_pct", 40.0},
    {"striker_post_kick_ignore_sec", 1.5},
    {"striker_post_kick_ignore_yaw_deg", 25.0},
    {"striker_gate_release_sec", 1.5},
    {"striker_keeper_min_conf", 0.50},
    {"striker_goalpost_min_conf", 0.45},
    {"striker_ema_alpha", 0.45},
    {"striker_keeper_width_offset_multiplier", 1.35},
    {"striker_min_offset_frame_ratio", 0.08},
    {"striker_max_offset_frame_ratio", 0.22},
    {"striker_image_width_px", 640.0},
    {"striker_half_hfov_deg", 35.2},
    {"striker_half_vfov_deg", 21.6},
    {"striker_aim_look_tilt_servo_deg", 180.0},
    {"striker_aim_look_wait_sec", 1.5},
    {"striker_aim_look_tol_deg", 3.0},
    {"striker_aim_collect_sec", 0.6},
    {"striker_aim_reacquire_sec", 4.0},
    {"striker_gyro_z_sign", 1.0},
    {"striker_gyro_bias_tau_sec", 2.0},
    {"striker_gyro_max_dt_sec", 0.1},
  };
  for (const auto &item : striker_numbers)
    declare(item.first, rclcpp::ParameterValue(item.second));
  declare("striker_search_original_is_right", rclcpp::ParameterValue(true));
  declare("striker_aim_look_enable", rclcpp::ParameterValue(true));
  declare("striker_search_turn_direction_per_step", rclcpp::ParameterValue(std::vector<double>{0.0, -1.0, 1.0, 1.0}));
  declare("striker_sector_pct_per_step", rclcpp::ParameterValue(std::vector<double>{50.0, 70.0, 30.0, 25.0}));
  declare("striker_gap_side_per_step", rclcpp::ParameterValue(std::vector<double>{0.0, -1.0, 1.0, 1.0}));
  declare("striker_gyro_target_yaw_per_step", rclcpp::ParameterValue(std::vector<double>{0.0, 22.0, -18.0, -35.0}));
}

double StrikerDemo::param(const std::string &name, double fallback) const
{
  if (node_ == nullptr || !node_->has_parameter(name))
    return fallback;
  const rclcpp::Parameter parameter = node_->get_parameter(name);
  switch (parameter.get_type())
  {
  case rclcpp::ParameterType::PARAMETER_DOUBLE: return parameter.as_double();
  case rclcpp::ParameterType::PARAMETER_INTEGER: return static_cast<double>(parameter.as_int());
  default: return fallback;
  }
}

std::string StrikerDemo::paramString(const std::string &name, const std::string &fallback) const
{
  if (node_ == nullptr || !node_->has_parameter(name))
    return fallback;
  const rclcpp::Parameter parameter = node_->get_parameter(name);
  return parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING ? parameter.as_string() : fallback;
}

bool StrikerDemo::paramArray4(const std::string &name, std::array<double, striker::kJumlahBola> &out) const
{
  if (node_ == nullptr || !node_->has_parameter(name))
    return false;
  const rclcpp::Parameter parameter = node_->get_parameter(name);
  std::vector<double> values;
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
    values = parameter.as_double_array();
  else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
    for (const int64_t v : parameter.as_integer_array())
      values.push_back(static_cast<double>(v));
  if (values.size() != out.size())
    return false;
  for (size_t i = 0; i < out.size(); ++i)
    out[i] = values[i];
  return true;
}

void StrikerDemo::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ == nullptr)
    return;
  declareParameters();
  // STRIKER: integrator gyro sudah jalan sebelum sesi pertama (bias dikumpulkan
  // selama robot berdiri diam), jadi parameternya dibaca sekarang juga.
  cfg_.striker.gyro_z_sign = param("striker_gyro_z_sign", 1.0) < 0.0 ? -1.0 : 1.0;
  cfg_.striker.gyro_bias_tau_sec = param("striker_gyro_bias_tau_sec", 2.0);
  cfg_.striker.gyro_max_dt_sec = param("striker_gyro_max_dt_sec", 0.1);

  // Publisher ANGGOTA, dibuat sekali: publisher sekali-pakai kehilangan pesan
  // pertamanya karena kenalan DDS belum selesai.
  walking_param_pub_ = node_->create_publisher<WalkingParam>("/robotis/walking/set_params", 10);
  walking_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
  joint_module_pub_ = node_->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>(
      "/robotis/set_joint_ctrl_modules", 10);
  page_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  tracker_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/ball_tracker/command", 10);
  // STRIKER: tanpa reset_yaw (itu titik nol yaw MAGNETOMETER). Detektor kiper /
  // gawang = yolo_gawang_detector.py apa adanya, dinyalakan hanya saat AIM_LOOK.
  keeper_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/kiper_detector_node/enable", 10);
  goalpost_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/gawang_detector_node/enable", 10);
  head_abs_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states", 10);

  get_walking_param_client_ = node_->create_client<op3_walking_module_msgs::srv::GetWalkingParam>(
      "/robotis/walking/get_params");
  is_running_client_ = node_->create_client<op3_action_module_msgs::srv::IsRunning>("/robotis/action/is_running");

  using std::placeholders::_1;
  subscriptions_.push_back(node_->create_subscription<op3_ball_detector_msgs::msg::CircleSetStamped>(
      "/ball_detector_node/circle_set", 10, std::bind(&StrikerDemo::circleSetCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/ball_tracker/status", 10, std::bind(&StrikerDemo::trackerStatusCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/present_joint_states", 10, std::bind(&StrikerDemo::presentJointsCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/goal_joint_states", 10, std::bind(&StrikerDemo::goalJointsCallback, this, _1)));
  // STRIKER: antrean 50 -- yaw gyro diintegrasi per pesan, loop 30 Hz menerima
  // ~4 pesan IMU (125 Hz) tiap putaran; pesan yang terbuang = putaran yang hilang.
  subscriptions_.push_back(node_->create_subscription<sensor_msgs::msg::Imu>(
      "/robotis/open_cr/imu", 50, std::bind(&StrikerDemo::imuCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/kiper_detector_node/detections", 10, std::bind(&StrikerDemo::keeperDetectionsCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/gawang_detector_node/detections", 10, std::bind(&StrikerDemo::goalpostDetectionsCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<robotis_controller_msgs::msg::JointCtrlModule>(
      "/robotis/present_joint_ctrl_modules", 10, std::bind(&StrikerDemo::jointModulesCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<std_msgs::msg::String>(
      "/robotis/movement_done", 10, std::bind(&StrikerDemo::movementDoneCallback, this, _1)));
  subscriptions_.push_back(node_->create_subscription<robotis_controller_msgs::msg::StatusMsg>(
      "/robotis/status", 50, std::bind(&StrikerDemo::statusCallback, this, _1)));
}

bool StrikerDemo::prepareConfig(std::string &problem)
{
  striker_demo::Config cfg;
  cfg.transition_sec = param("auto_demo_transition_sec", 0.4);
  cfg.initial_search_sec = param("auto_demo_initial_search_sec", 3.0);
  cfg.start_search_sec = param("auto_demo_start_search_sec", 7.0);
  cfg.start_forward_sec = param("auto_demo_start_forward_sec", 20.0);
  cfg.search_to_walk_delay_sec = std::max(0.0, param("auto_demo_search_to_walk_delay_sec", 0.0));
  cfg.prekick_nudge_sec = param("auto_demo_prekick_nudge_sec", 0.3);
  cfg.post_action_search_sec = param("auto_demo_post_action_search_sec", 1.5);
  cfg.lost_search_sec = param("auto_demo_lost_search_sec", 3.0);
  cfg.ball_valid_gap_sec = param("auto_demo_ball_valid_gap_sec", 5.0 / 30.0);
  cfg.ball_lost_sec = param("auto_demo_ball_lost_sec", 1.0);
  cfg.servo_center_deg = param("auto_demo_servo_center_deg", 180.0);
  cfg.servo_dir = param("auto_demo_servo_dir", 1.0);
  cfg.pan_left_limit_servo_deg = jointToServoDeg(deg2rad(param("auto_demo_head_pan_left_limit_deg", 85.0)),
                                                 cfg.servo_center_deg, cfg.servo_dir);
  cfg.pan_right_limit_servo_deg = jointToServoDeg(deg2rad(param("auto_demo_head_pan_right_limit_deg", -85.0)),
                                                  cfg.servo_center_deg, cfg.servo_dir);
  cfg.pan_kick_left = param("auto_demo_pan_kick_left", 48.0);
  cfg.pan_center = param("auto_demo_pan_center", 50.0);
  cfg.pan_kick_right = param("auto_demo_pan_kick_right", 52.0);
  cfg.pan_outer_left = param("auto_demo_pan_outer_left", 45.0);
  cfg.pan_outer_left_stop = param("auto_demo_pan_outer_left_stop", 46.0);
  cfg.pan_outer_right = param("auto_demo_pan_outer_right", 55.0);
  cfg.pan_outer_right_stop = param("auto_demo_pan_outer_right_stop", 54.0);
  cfg.tilt_limit_deg = param("auto_demo_tilt_limit_deg", 200.0);
  cfg.steer_gain = param("auto_demo_steer_gain", 0.2);
  cfg.steer_min_deg = param("auto_demo_steer_min_deg", 5.0);
  cfg.steer_max_deg = param("auto_demo_steer_max_deg", 15.0);
  cfg.steer_ramp_deg = param("auto_demo_steer_ramp_deg", 0.5);
  cfg.steer_deadband_deg = param("auto_demo_steer_deadband_deg", 5.0);
  cfg.steer_sign = param("auto_demo_steer_sign", 1.0) < 0.0 ? -1.0 : 1.0;
  cfg.steer_publish_period_sec = param("auto_demo_steer_publish_period_sec", 0.1);
  cfg.search_align_left_max = param("auto_demo_search_align_left_max", 30.0);
  cfg.search_align_right_min = param("auto_demo_search_align_right_min", 70.0);
  cfg.search_align_stop_left = param("auto_demo_search_align_stop_left", 40.0);
  cfg.search_align_stop_right = param("auto_demo_search_align_stop_right", 60.0);
  cfg.head_park_wait_sec = param("auto_demo_head_park_wait_sec", 1.5);
  cfg.head_park_tol_deg = param("auto_demo_head_park_tol_deg", 3.0);
  cfg.head_park_pan_deg = param("auto_demo_head_park_pan_deg", 0.0);
  cfg.head_park_tilt_servo_deg = param("auto_demo_head_park_tilt_servo_deg", 200.0);
  cfg.stop_head_tilt_servo_deg = param("auto_demo_stop_head_tilt_servo_deg", 135.0);
  cfg.yaw_target_deg = param("auto_demo_yaw_target_deg", 0.0);
  cfg.yaw_tolerance_deg = param("auto_demo_yaw_tolerance_deg", 6.0);
  cfg.yaw_stop_deg = param("auto_demo_yaw_stop_deg", 4.0);
  cfg.yaw_tolerance_neg_deg = param("auto_demo_yaw_tolerance_neg_deg", 0.5);
  cfg.yaw_stop_neg_deg = param("auto_demo_yaw_stop_neg_deg", 0.0);
  cfg.yaw_timeout_sec = param("auto_demo_yaw_timeout_sec", 20.0);
  cfg.approach_timeout_sec = param("auto_demo_approach_timeout_sec", 30.0);
  cfg.shift_timeout_sec = param("auto_demo_shift_timeout_sec", 8.0);
  cfg.search_timeout_sec = param("auto_demo_search_timeout_sec", 60.0);
  cfg.page_kick_left = static_cast<int>(std::lround(param("auto_demo_page_kick_left", 104.0)));
  cfg.page_kick_right = static_cast<int>(std::lround(param("auto_demo_page_kick_right", 105.0)));
  cfg.page_getup_front = static_cast<int>(std::lround(param("auto_demo_page_getup_front", 85.0)));
  cfg.page_getup_back = static_cast<int>(std::lround(param("auto_demo_page_getup_back", 86.0)));
  cfg.yaw_max_age_sec = param("auto_demo_yaw_max_age_sec", 1.0);

  const auto finite = [](double v) { return std::isfinite(v); };
  if (!finite(cfg.transition_sec) || cfg.transition_sec <= 0.0)
    problem = "auto_demo_transition_sec harus > 0";
  else if (!(cfg.pan_kick_left >= 0.0 && cfg.pan_kick_left < cfg.pan_center && cfg.pan_center < cfg.pan_kick_right &&
             cfg.pan_kick_right <= 100.0))
    problem = "harus 0 <= auto_demo_pan_kick_left < auto_demo_pan_center < auto_demo_pan_kick_right <= 100";
  else if (!(cfg.pan_outer_left >= 0.0 && cfg.pan_outer_left < cfg.pan_outer_left_stop &&
             cfg.pan_outer_left_stop <= cfg.pan_kick_left && cfg.pan_kick_right <= cfg.pan_outer_right_stop &&
             cfg.pan_outer_right_stop < cfg.pan_outer_right && cfg.pan_outer_right <= 100.0))
    problem = "harus outer_left < outer_left_stop <= kick_left < center < kick_right <= outer_right_stop < outer_right";
  else if (!finite(cfg.tilt_limit_deg) || cfg.tilt_limit_deg <= 0.0 || cfg.tilt_limit_deg >= 360.0)
    problem = "auto_demo_tilt_limit_deg di luar 0..360";
  else if (!(cfg.yaw_stop_deg > 0.0 && cfg.yaw_stop_deg <= cfg.yaw_tolerance_deg))
    problem = "harus 0 < auto_demo_yaw_stop_deg <= auto_demo_yaw_tolerance_deg";
  else if (!(cfg.yaw_stop_neg_deg >= 0.0 && cfg.yaw_stop_neg_deg <= cfg.yaw_tolerance_neg_deg))
    problem = "harus 0 <= auto_demo_yaw_stop_neg_deg <= auto_demo_yaw_tolerance_neg_deg";
  else if (std::fabs(cfg.servo_dir) < 1e-9)
    problem = "auto_demo_servo_dir nol";
  else if (!(cfg.search_align_left_max >= 0.0 && cfg.search_align_left_max < cfg.search_align_stop_left &&
             cfg.search_align_stop_left < cfg.search_align_stop_right &&
             cfg.search_align_stop_right < cfg.search_align_right_min && cfg.search_align_right_min <= 100.0))
    problem = "harus 0 <= search_align_left_max < search_align_stop_left < search_align_stop_right < "
              "search_align_right_min <= 100";
  else if (!(cfg.steer_gain >= 0.0 && cfg.steer_min_deg >= 0.0 && cfg.steer_min_deg <= cfg.steer_max_deg &&
             cfg.steer_max_deg <= 45.0 && cfg.steer_ramp_deg > 0.0 && cfg.steer_publish_period_sec > 0.0))
    problem = "auto_demo_steer_*: gain >= 0, 0 <= min <= max <= 45 deg, ramp > 0, publish_period > 0";
  else if (cfg.page_kick_left <= 0 || cfg.page_kick_right <= 0 || cfg.page_getup_front <= 0 ||
           cfg.page_getup_back <= 0)
    problem = "nomor halaman action harus positif";

  // ---- STRIKER ---------------------------------------------------------------------
  {
    striker_demo::StrikerConfig &sc = cfg.striker;
    striker::StrategyConfig &st = sc.strategy;
    st.start_index = static_cast<int>(std::lround(param("striker_start_index", 0.0)));
    st.sector_gate_pct = param("striker_sector_gate_pct", 40.0);
    st.post_kick_ignore_sec = param("striker_post_kick_ignore_sec", 1.5);
    st.post_kick_ignore_yaw_deg = param("striker_post_kick_ignore_yaw_deg", 25.0);
    // "Jauh" = ambang jarak servo 20 milik robot (auto_demo_tilt_limit_deg).
    st.far_tilt_servo_deg = cfg.tilt_limit_deg;
    st.keeper_min_conf = param("striker_keeper_min_conf", 0.50);
    st.goalpost_min_conf = param("striker_goalpost_min_conf", 0.45);
    st.ema_alpha = param("striker_ema_alpha", 0.45);
    st.keeper_width_offset_multiplier = param("striker_keeper_width_offset_multiplier", 1.35);
    st.min_offset_frame_ratio = param("striker_min_offset_frame_ratio", 0.08);
    st.max_offset_frame_ratio = param("striker_max_offset_frame_ratio", 0.22);
    st.image_width_px = param("striker_image_width_px", 640.0);
    st.half_hfov_deg = param("striker_half_hfov_deg", 35.2);
    sc.half_vfov_deg = param("striker_half_vfov_deg", 21.6);
    sc.gate_release_sec = param("striker_gate_release_sec", 1.5);
    sc.search_original_is_right = paramBoolOf(node_, "striker_search_original_is_right", true);
    sc.aim_look_enable = paramBoolOf(node_, "striker_aim_look_enable", true);
    sc.aim_look_tilt_servo_deg = param("striker_aim_look_tilt_servo_deg", 180.0);
    sc.aim_look_wait_sec = param("striker_aim_look_wait_sec", 1.5);
    sc.aim_look_tol_deg = param("striker_aim_look_tol_deg", 3.0);
    sc.aim_collect_sec = param("striker_aim_collect_sec", 0.6);
    sc.aim_reacquire_sec = param("striker_aim_reacquire_sec", 4.0);
    sc.gyro_z_sign = param("striker_gyro_z_sign", 1.0) < 0.0 ? -1.0 : 1.0;
    sc.gyro_bias_tau_sec = param("striker_gyro_bias_tau_sec", 2.0);
    sc.gyro_max_dt_sec = param("striker_gyro_max_dt_sec", 0.1);

    std::array<double, striker::kJumlahBola> turn{}, sector{}, gap{}, yaw_table{};
    const auto sign_of = [](double v) { return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0); };
    if (!problem.empty())
    {
      // masalah konfigurasi D60 di atas dilaporkan lebih dulu
    }
    else if (!paramArray4("striker_search_turn_direction_per_step", turn) ||
             !paramArray4("striker_sector_pct_per_step", sector) || !paramArray4("striker_gap_side_per_step", gap) ||
             !paramArray4("striker_gyro_target_yaw_per_step", yaw_table))
      problem = "striker_*_per_step harus larik 4 angka (urutan B, A, C, D)";
    else if (st.start_index < 0 || st.start_index >= striker::kJumlahBola)
      problem = "striker_start_index harus 0..3 (0 = B, 1 = A, 2 = C, 3 = D)";
    else if (!(st.image_width_px > 0.0 && st.half_hfov_deg > 0.0 && st.half_hfov_deg < 90.0 &&
               sc.half_vfov_deg > 0.0 && sc.half_vfov_deg < 90.0))
      problem = "striker_image_width_px > 0 dan striker_half_hfov_deg / striker_half_vfov_deg di 0..90";
    else if (!(st.min_offset_frame_ratio >= 0.0 && st.min_offset_frame_ratio <= st.max_offset_frame_ratio &&
               st.ema_alpha > 0.0 && st.ema_alpha <= 1.0))
      problem = "harus 0 <= striker_min_offset_frame_ratio <= striker_max_offset_frame_ratio, 0 < striker_ema_alpha <= 1";
    else if (!(sc.aim_look_wait_sec > 0.0 && sc.aim_collect_sec > 0.0 && sc.aim_reacquire_sec > 0.0 &&
               sc.gate_release_sec > 0.0 && sc.gyro_bias_tau_sec > 0.0 && sc.gyro_max_dt_sec > 0.0))
      problem = "waktu striker_* (aim_look_wait, aim_collect, aim_reacquire, gate_release, gyro_*) harus > 0";
    if (problem.empty())
    {
      for (size_t i = 0; i < turn.size(); ++i)
      {
        st.turn_direction_per_step[i] = sign_of(turn[i]);
        st.sector_pct_per_step[i] = sector[i];
        st.gap_side_per_step[i] = sign_of(gap[i]);
        st.gyro_target_yaw_per_step[i] = yaw_table[i];
      }
    }
  }
  if (!problem.empty())
    return false;

  cfg_ = cfg;
  pengarah_.configure(cfg_.striker.strategy);
  problem = loadStudioPresets();
  if (!problem.empty())
    return false;

  const auto logger = rclcpp::get_logger(kLogger);
  RCLCPP_WARN(logger,
              "servo 20 = JARAK: <= %.1f deg bola masih JAUH, > %.1f deg bola sudah DEKAT | "
              "servo 19 = HORIZONTAL: 0%% = %.1f deg (kiri), 100%% = %.1f deg (kanan); < %.0f%% tarik KANAN MIN NEW "
              "sampai >= %.0f%%, > %.0f%% tarik KIRI MIN NEW sampai <= %.0f%%; tendang kiri < %.0f%%, "
              "geser kiri %.0f..%.0f%%, geser kanan %.0f..%.0f%%, tendang kanan > %.0f%% | "
              "yaw > %+.1f putar KIRI, yaw < %+.1f putar KANAN, berhenti di %+.1f..%+.1f | transisi %.2f s | "
              "cari awal %.1f s, sesudah aksi %.1f s, bola hilang %.1f s",
              cfg_.tilt_limit_deg, cfg_.tilt_limit_deg, cfg_.pan_left_limit_servo_deg, cfg_.pan_right_limit_servo_deg,
              cfg_.pan_outer_left, cfg_.pan_outer_left_stop, cfg_.pan_outer_right, cfg_.pan_outer_right_stop,
              cfg_.pan_kick_left, cfg_.pan_kick_left, cfg_.pan_center, cfg_.pan_center, cfg_.pan_kick_right,
              cfg_.pan_kick_right, cfg_.yaw_tolerance_deg, -cfg_.yaw_tolerance_neg_deg, -cfg_.yaw_stop_neg_deg, cfg_.yaw_stop_deg,
              cfg_.transition_sec, cfg_.initial_search_sec, cfg_.post_action_search_sec, cfg_.lost_search_sec);
  RCLCPP_WARN(logger, "halaman: tendang kiri %d, kanan %d, bangun depan %d, belakang %d",
              cfg_.page_kick_left, cfg_.page_kick_right, cfg_.page_getup_front, cfg_.page_getup_back);
  RCLCPP_WARN(logger, "setir saat mendekat: angle = %+.0f x clamp(|pan| x %.2f, %.0f..%.0f deg), naik %.1f deg/tick, "
                      "deadband %.0f deg (gain 0 = jalan lurus seperti sebelumnya)",
              cfg_.steer_sign, cfg_.steer_gain, cfg_.steer_min_deg, cfg_.steer_max_deg, cfg_.steer_ramp_deg,
              cfg_.steer_deadband_deg);
  {
    const striker::StrategyConfig &st = cfg_.striker.strategy;
    RCLCPP_WARN(logger,
                "STRIKER urutan B A C D, mulai %s | arah putar search %+d %+d %+d %+d (-1 kanan, +1 kiri, 0 ikut yaw) "
                "| sektor servo 19 %.0f/%.0f/%.0f/%.0f%% +-%.0f | cooldown %.1f s (|yaw|<%.0f, servo 20 <= %.0f) "
                "| target yaw cadangan %+.0f/%+.0f/%+.0f/%+.0f | lirik %s servo 20 %.0f, kumpul %.1f s | "
                "yaw = GYRO (tanda %+.0f), magnetometer TIDAK dipakai",
                striker::namaBola(st.start_index), st.turn_direction_per_step[0], st.turn_direction_per_step[1],
                st.turn_direction_per_step[2], st.turn_direction_per_step[3], st.sector_pct_per_step[0],
                st.sector_pct_per_step[1], st.sector_pct_per_step[2], st.sector_pct_per_step[3], st.sector_gate_pct,
                st.post_kick_ignore_sec, st.post_kick_ignore_yaw_deg, st.far_tilt_servo_deg,
                st.gyro_target_yaw_per_step[0], st.gyro_target_yaw_per_step[1], st.gyro_target_yaw_per_step[2],
                st.gyro_target_yaw_per_step[3], cfg_.striker.aim_look_enable ? "AKTIF" : "MATI",
                cfg_.striker.aim_look_tilt_servo_deg, cfg_.striker.aim_collect_sec, cfg_.striker.gyro_z_sign);
  }
  return true;
}

std::string StrikerDemo::loadStudioPresets()
{
  const auto logger = rclcpp::get_logger(kLogger);
  std::string path = paramString("auto_demo_studio_versions_path", "");
  if (path.empty())
  {
    try
    {
      // install/op3_demo/share/op3_demo -> akar workspace -> sumber Studio.
      path = ament_index_cpp::get_package_share_directory("op3_demo") +
             "/../../../../src/bascorro_studio/data/walking_versions.json";
    }
    catch (const std::exception &)
    {
      path = "src/bascorro_studio/data/walking_versions.json";
    }
  }
  path = realPath(path);
  const StudioWalkingFile file = loadStudioWalkingFile(path);

  // ---- acuan page 2 --------------------------------------------------------
  // Angka init pose tersimpan = nilai asli ROS TERHADAP acuan ini. Acuan yang
  // beda dengan yang dipakai saat angka itu disetel membuat robot berdiri
  // belasan derajat dari page 2 (16 deg pada pergantian 23 Okt 2026), jadi
  // ketidakcocokan apa pun di sini MENOLAK sesi, bukan diteruskan.
  std::string reference_path = paramString("auto_demo_pose_reference_path", "");
  if (reference_path.empty())
    reference_path = path.substr(0, path.find_last_of('/') + 1) + "walking_ready_reference.json";
  reference_path = realPath(reference_path);
  const std::vector<std::string> reference_fields(kPoseReferenceFields.begin(), kPoseReferenceFields.end());
  const WalkingPoseReference pose_reference = loadWalkingPoseReference(reference_path, reference_fields);
  if (!pose_reference.ok)
    return "acuan page 2 " + reference_path + ": " + pose_reference.error +
           " -- jalankan scripts/walking_ready_fk.py --write";
  for (size_t i = 0; i < kPoseReferenceFields.size(); ++i)
    pose_reference_[i] = pose_reference.offsets.at(kPoseReferenceFields[i]);
  RCLCPP_WARN(logger, "acuan page 2 %s dari %s (dibuat %s): x %+.4f z %+.4f roll %+.4f pitch %+.4f yaw %+.4f "
              "hip_pitch %+.4f", pose_reference.id.c_str(), reference_path.c_str(), pose_reference.generated_at.c_str(),
              pose_reference_[0], pose_reference_[1], pose_reference_[2], pose_reference_[3], pose_reference_[4],
              pose_reference_[5]);
  if (file.ok && file.pose_reference_id != pose_reference.id)
    return "versi Studio " + path + " bertanda acuan \"" + file.pose_reference_id + "\", padahal acuan sekarang \"" +
           pose_reference.id + "\": angka init pose-nya berarti acuan lain. Jalankan scripts/walking_ready_fk.py --write";

  // Page 2 di-teach ulang tanpa menurunkan acuan lagi: semua tetap konsisten
  // satu sama lain (Studio dan mode ini memakai acuan yang sama), hanya titik
  // kerja IK-nya tidak lagi tepat di page 2. Diingatkan, tidak ditolak.
  if (!pose_reference.action_file.empty() && pose_reference.page > 0)
  {
    const std::string data_dir = reference_path.substr(0, reference_path.find_last_of('/'));
    const std::string bin_path = realPath(data_dir + "/../../../" + pose_reference.action_file);
    std::map<std::string, int> now_raw;
    std::string bin_error;
    if (!readActionPageLegRaw(bin_path, pose_reference.page, now_raw, bin_error))
      RCLCPP_WARN(logger, "acuan page 2: %s tidak bisa diperiksa (%s)", bin_path.c_str(), bin_error.c_str());
    else if (now_raw != pose_reference.leg_raw)
      RCLCPP_ERROR(logger, "acuan page 2 BASI: page %d di %s sudah di-teach ulang sejak acuan %s dibuat. "
                   "Tutup Studio, jalankan scripts/walking_ready_fk.py --write",
                   pose_reference.page, bin_path.c_str(), pose_reference.id.c_str());
  }

  preset_msgs_.clear();
  preset_info_.assign(static_cast<size_t>(PresetRole::Count), striker_demo::PresetInfo());

  if (file.ok)
    RCLCPP_WARN(logger, "versi walking Bascorro Studio dibaca dari %s (revisi %d, disimpan %s, %zu versi)",
                path.c_str(), file.revision, file.saved_at.c_str(), file.versions.size());
  else
    RCLCPP_ERROR(logger, "berkas versi walking Bascorro Studio %s: %s", path.c_str(), file.error.c_str());

  // Preset mendekat D60 boleh diambil dari berkas versi tersendiri (salinan
  // JALAN BARU revisi 95), berkas Studio dan preset lain tidak disentuh.
  std::string approach_path = paramString("auto_demo_preset_approach_file", "");
  StudioWalkingFile approach_file;
  if (!approach_path.empty())
  {
    approach_path = realPath(approach_path);
    approach_file = loadStudioWalkingFile(approach_path);
    if (approach_file.ok)
      RCLCPP_WARN(logger, "preset mendekat D60 dibaca dari %s (revisi %d, disimpan %s), BUKAN dari berkas Studio",
                  approach_path.c_str(), approach_file.revision, approach_file.saved_at.c_str());
    else
      RCLCPP_ERROR(logger, "berkas preset mendekat D60 %s: %s", approach_path.c_str(), approach_file.error.c_str());
    // Salinan versi harus membawa tanda acuan berkas asalnya.
    if (approach_file.ok && approach_file.pose_reference_id != pose_reference.id)
      return "preset mendekat " + approach_path + " bertanda acuan \"" + approach_file.pose_reference_id +
             "\", padahal acuan sekarang \"" + pose_reference.id + "\". Jalankan scripts/walking_ready_fk.py --write";
  }

  bool any_missing = false;
  for (const RoleParam &item : kRoleParams)
  {
    striker_demo::PresetInfo &info = preset_info_[static_cast<size_t>(item.role)];
    info.name = paramString(item.parameter, item.default_name);
    const bool own_file = (item.role == PresetRole::Approach) && !approach_path.empty();
    const StudioWalkingFile &source = own_file ? approach_file : file;
    if (!source.ok)
    {
      info.problem = std::string(own_file ? "berkas preset mendekat D60" : "berkas Studio") + " tidak terbaca: " +
                     source.error;
      any_missing = true;
      continue;
    }
    std::vector<std::string> similar;
    const StudioWalkingVersion *version = findStudioWalkingVersion(source, info.name, &similar);
    if (version == nullptr)
    {
      info.problem = "nama PERSIS \"" + info.name + "\" tidak ada di Bascorro Studio";
      for (const std::string &name : similar)
        info.problem += " (ada \"" + name + "\" yang beda kapital; TIDAK dipakai otomatis)";
      any_missing = true;
      RCLCPP_ERROR(logger, "preset %s: %s", striker_demo::toString(item.role), info.problem.c_str());
      continue;
    }
    std::vector<std::string> missing;
    if (!studioWalkingVersionComplete(*version, &missing))
    {
      info.problem = "versi \"" + info.name + "\" tidak lengkap, field kurang:";
      for (const std::string &field : missing)
        info.problem += " " + field;
      RCLCPP_ERROR(logger, "preset %s: %s", striker_demo::toString(item.role), info.problem.c_str());
      continue;
    }
    const WalkingParam msg = toWalkingParam(*version);
    preset_msgs_[item.role] = msg;
    info.available = true;
    info.period_time = msg.period_time;
    info.x_move_amplitude = msg.x_move_amplitude;
    RCLCPP_WARN(logger,
                "preset %-11s = \"%s\": periode %.3f s | x %.4f y %.4f z %.4f m | sudut %+.3f | "
                "init pose x %+.4f z %+.4f pitch %+.3f hip_pitch %+.3f",
                striker_demo::toString(item.role), info.name.c_str(), msg.period_time, msg.x_move_amplitude,
                msg.y_move_amplitude, msg.z_move_amplitude, msg.angle_move_amplitude, msg.init_x_offset,
                msg.init_z_offset, msg.init_pitch_offset, msg.hip_pitch_offset);
  }
  if (any_missing && file.ok)
  {
    std::string names;
    for (const StudioWalkingVersion &version : file.versions)
      names += (names.empty() ? "\"" : ", \"") + version.name + "\"";
    RCLCPP_ERROR(logger, "versi yang ADA di Bascorro Studio: %s", names.c_str());
  }

  // Acuan pose = preset INIT dengan enam offset bias dipatok ke page 2, persis
  // publishWalkingPoseReference() di Bascorro Studio = "INIT POSE PAGE 2 versi
  // JALAN MENUJU MEKKAH".
  const striker_demo::PresetInfo &init = preset_info_[static_cast<size_t>(PresetRole::Init)];
  striker_demo::PresetInfo &reference = preset_info_[static_cast<size_t>(PresetRole::PoseReference)];
  reference.name = init.name + " + acuan page 2";
  if (init.available)
  {
    WalkingParam msg = preset_msgs_[PresetRole::Init];
    for (size_t i = 0; i < kPoseReferenceFields.size(); ++i)
      setField(msg, kPoseReferenceFields[i], pose_reference_[i]);
    preset_msgs_[PresetRole::PoseReference] = msg;
    reference.available = true;
    reference.period_time = msg.period_time;
    reference.x_move_amplitude = msg.x_move_amplitude;
  }
  else
  {
    reference.problem = init.problem;
  }

  // MENCARI CELAH dengan yaw DIBALIK, runtime saja: salinan UTUH preset Search
  // (init pose, soft start, periode, z, hip_pitch, gain, ... identik), hanya
  // angle_move_amplitude yang dinegasikan. Tidak ada yang ditulis ke Studio.
  const striker_demo::PresetInfo &search = preset_info_[static_cast<size_t>(PresetRole::Search)];
  striker_demo::PresetInfo &inverted = preset_info_[static_cast<size_t>(PresetRole::SearchInverted)];
  inverted.name = search.name + " (yaw dibalik, runtime)";
  if (search.available)
  {
    WalkingParam msg = preset_msgs_[PresetRole::Search];
    msg.angle_move_amplitude = -msg.angle_move_amplitude;
    preset_msgs_[PresetRole::SearchInverted] = msg;
    inverted.available = true;
    inverted.period_time = msg.period_time;
    inverted.x_move_amplitude = msg.x_move_amplitude;
    RCLCPP_WARN(logger, "preset %-11s = \"%s\": angle_move_amplitude %+.3f -> %+.3f rad (magnitude sama), "
                        "field lain identik", striker_demo::toString(PresetRole::SearchInverted), search.name.c_str(),
                preset_msgs_[PresetRole::Search].angle_move_amplitude, msg.angle_move_amplitude);
  }
  else
  {
    inverted.problem = search.problem;
  }
  return "";
}

const striker_demo::PresetInfo &StrikerDemo::preset(PresetRole role) const
{
  return preset_info_[static_cast<size_t>(role)];
}

// ============================================================================
//  Mulai / berhenti (tombol D60 = toggle)
// ============================================================================

void StrikerDemo::toggle(const std::string &source)
{
  const double now = steadyNow();
  const auto logger = rclcpp::get_logger(kLogger);
  switch (mode_)
  {
  case Mode::Stopped:
    RCLCPP_WARN(logger, "%s: toggle -> AUTONOMOUS ON, mulai dari AWAL", source.c_str());
    start_queued_ = false;
    queued_start_due_ = false;
    startSession(now);
    break;
  case Mode::Running:
    RCLCPP_WARN(logger, "%s: toggle -> AUTONOMOUS OFF", source.c_str());
    beginStop(now, source + ": toggle -> STOP");
    break;
  case Mode::Stopping:
    // Tekanan TERAKHIR yang berlaku: tekan sekali = mulai lagi dari awal
    // sesudah berhenti, tekan lagi = batal.
    start_queued_ = !start_queued_;
    RCLCPP_WARN(logger, "%s saat STOPPING: sesudah berhenti %s", source.c_str(),
                start_queued_ ? "MULAI dari awal" : "tetap OFF");
    break;
  }
}

void StrikerDemo::requestStop(const std::string &reason)
{
  const double now = steadyNow();
  if (mode_ == Mode::Running)
    beginStop(now, reason);
  else if (start_queued_ || queued_start_due_)
  {
    start_queued_ = false;
    queued_start_due_ = false;
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "%s: mulai-ulang yang sedang antre DIBATALKAN", reason.c_str());
  }
}

void StrikerDemo::process()
{
  if (!isBusy() && !hasQueuedStart())
    return;
  const double now = steadyNow();
  pollActionRunning(now);
  tick(now);
  if (queued_start_due_ && mode_ == Mode::Stopped)
  {
    queued_start_due_ = false;
    startSession(now);
  }
}

void StrikerDemo::startSession(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(logger, "node belum dipasang; mode D60 tidak bisa mulai");
    return;
  }
  if (mode_ != Mode::Stopped)
  {
    RCLCPP_WARN(logger, "sesi baru ditolak: mode masih %s", striker_demo::toString(mode_));
    return;
  }
  // Konfigurasi dan preset dibaca ULANG tiap sesi: tidak ada yang dibawa dari
  // sesi sebelumnya.
  std::string problem;
  if (!prepareConfig(problem))
  {
    RCLCPP_ERROR(logger, "konfigurasi D60 tidak sah, mode TIDAK mulai: %s", problem.c_str());
    return;
  }

  mode_ = Mode::Running;
  ++session_;
  start_queued_ = false;
  queued_start_due_ = false;

  // STRIKER MULAI: arah robot saat ini (menghadap gawang dari busur start) =
  // yaw GYRO 0. Bias gyro yang sudah terkumpul saat berdiri diam dipertahankan.
  // Tidak ada reset_yaw ke open_cr_module: itu yaw magnetometer.
  gyro_.reset();
  pengarah_.reset(cfg_.striker.strategy.start_index);
  gerbang_.reset();
  kumpul_.reset();
  aim_collecting_ = false;
  setDetectorsEnabled(false);
  post_kick_look_pending_ = false;
  setAimTargetFromTable("sesi baru");
  if (!gyro_.biasReady())
    RCLCPP_ERROR(logger, "bias gyro belum terkumpul (robot belum diam >= %.1f s): koreksi yaw akan DILEWATI sampai siap",
                 striker::GyroYaw::kBiasReadySec);
  RCLCPP_WARN(logger, "STRIKER: bola target %s (indeks %d), arah putar search %s, bias gyro %+.3f deg/s",
              striker::namaBola(pengarah_.indeks()), pengarah_.indeks(),
              striker::toString(pengarah_.arahPutar(gyro_.yawDeg())), gyro_.biasDegS());

  // §6: RESET semua state autonomous sebelumnya. Tidak ada yang dilanjutkan.
  flags_ = striker_demo::AttemptFlags();
  ball_tracking_valid_ = false;
  attempt_ = 0;
  walking_commanded_ = false;
  walking_stop_at_ = kNever;
  walking_start_sent_at_ = kNever;
  walking_start_reported_ = false;
  running_period_ = kNaN;
  running_role_ = PresetRole::Count;
  motion_end_ = kNever;
  switch_sent_at_ = kNever;
  owners_claimed_ = false;
  tracker_started_ = false;
  tracker_start_at_ = kNever;
  our_action_pending_ = false;
  our_action_started_ = false;
  our_action_failed_ = false;
  stop_waits_action_ = false;
  filtered_pitch_ = 0.0;
  fall_ = PitchFallState::Stand;
  stand_search_deadline_ = kNever;
  search_align_dir_ = striker_demo::AlignDir::None;
  soft_start_end_ = kNever;
  telemetry_logged_at_ = kNever;
  clearPresetRequest();
  state_ = State::Idle;
  RCLCPP_WARN(logger, "MODE D60 MULAI: sesi #%llu, dari INIT \"%s\" lalu cari bola %.1f s",
              static_cast<unsigned long long>(session_), preset(PresetRole::Init).name.c_str(),
              cfg_.start_search_sec);
  start_hold_ = true;
  start_forward_pending_ = false;
  start_forward_active_ = false;
  enterInit(now, State::StandSearch, cfg_.start_search_sec, true, "sesi baru");
}

void StrikerDemo::resetAttemptFlags(double now, const std::string &why)
{
  flags_ = striker_demo::AttemptFlags();
  ++attempt_;
  RCLCPP_WARN(rclcpp::get_logger(kLogger),
              "PERCOBAAN BARU #%llu (%s): penanda yaw, jarak, geser, tendang direset | %s",
              static_cast<unsigned long long>(attempt_), why.c_str(), snapshot(now).c_str());
}

void StrikerDemo::beginStop(double now, const std::string &reason)
{
  RCLCPP_WARN(rclcpp::get_logger(kLogger), "MODE D60 STOPPING dari %s (sesi #%llu): %s | %s",
              striker_demo::toString(state_), static_cast<unsigned long long>(session_), reason.c_str(),
              snapshot(now).c_str());
  mode_ = Mode::Stopping;
  stop_started_at_ = now;
  stop_reason_ = reason;
  start_queued_ = false;
  queued_start_due_ = false;
  ++session_;   // jawaban async milik sesi lama tidak diterima lagi

  // §7: semua keputusan dimatikan, penanda dan timer direset.
  flags_ = striker_demo::AttemptFlags();
  ball_tracking_valid_ = false;
  clearPresetRequest();
  motion_end_ = kNever;
  walking_start_sent_at_ = kNever;
  stand_search_deadline_ = kNever;
  search_align_dir_ = striker_demo::AlignDir::None;

  // Penghentian lewat API yang sudah ada, yang menjaga postur:
  //  - walking "stop": walking_module menyelesaikan langkah lalu berdiri
  //    (menghentikan MENCARI CELAH / jalan / putar / geser apa pun);
  //  - pelacak "stop": head_tracking_node berhenti dan memarkir kepala;
  //  - page_num -1 kalau halaman action sedang main: halaman DISELESAIKAN
  //    lalu berhenti, tidak dipotong di tengah tendangan/bangun.
  publishWalkingCommand("stop");
  walking_commanded_ = false;
  walking_stop_at_ = now;
  // D60 OFF: pelacak berhenti dan kepala diparkir ke servo 20 135 (bukan 200).
  // Saat D60 nyala lagi sapuan mulai dari pra-awal lalu perlahan ke titik awal.
  {
    char cmd[32];
    std::snprintf(cmd, sizeof(cmd), "stop:%.1f", cfg_.stop_head_tilt_servo_deg);
    publishTracker(cmd);
    tracker_started_ = false;
    ball_tracking_valid_ = false;
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "/ball_tracker/command <- %s", cmd);
  }
  publishTracker("scan_legacy");   // sapuan lama (demo soccer) sesudah D60 mati
  aim_collecting_ = false;
  setDetectorsEnabled(false);      // STRIKER: detektor kiper/gawang mati
  stop_waits_action_ = actionRunning(now);
  if (stop_waits_action_)
  {
    std_msgs::msg::Int32 msg;
    msg.data = -1;
    page_pub_->publish(msg);
    RCLCPP_WARN(rclcpp::get_logger(kLogger),
                "halaman action %d sedang main: dibiarkan selesai (page_num -1), lalu kembali ke INIT",
                our_action_page_);
  }
  state_ = State::Idle;
  setStep(now, 0);
}

void StrikerDemo::tickStopping(double now)
{
  if (state_ == State::Park)
  {
    tickPark(now);
    return;
  }
  const bool legs_fresh = now - leg_motion_stamp_ <= cfg_.leg_motion_fresh_sec;
  const bool controller_never_ran = !happened(leg_motion_stamp_);
  const bool walking_done = controller_never_ran ||
                            (now - walking_stop_at_ >= cfg_.gait_stop_factor * currentGaitPeriod() &&
                             legs_fresh && !(now - leg_fast_last_ < cfg_.leg_quiet_window_sec));
  const bool tracker_done = tracker_text_ == "INACTIVE" && tracker_stamp_ >= stop_started_at_;
  const bool action_done = !stop_waits_action_ ||
      (!our_action_pending_ && action_poll_request_time_ > stop_started_at_ && !action_poll_running_);

  if (walking_done && tracker_done && action_done)
  {
    // §7/§9: kondisi diam = INIT POSE PAGE 2 versi "JALAN MENUJU MEKKAH".
    enterState(now, State::Park, "semua gerakan berhenti; kembali ke pose INIT");
    return;
  }
  if (now - stop_started_at_ >= cfg_.stop_confirm_timeout_sec)
  {
    std::string missing;
    const auto add = [&missing](const char *item) { missing += (missing.empty() ? "" : ", ") + std::string(item); };
    if (!walking_done)
      add(legs_fresh ? "walking masih bergerak" : "goal_joint_states kaki tidak diterima");
    if (!tracker_done)
      add("head_tracking_node tidak melapor INACTIVE");
    if (!action_done)
      add("action_module tidak melapor selesai");
    finishStop(now, missing);
  }
}

void StrikerDemo::finishStop(double now, const std::string &unconfirmed)
{
  mode_ = Mode::Stopped;
  state_ = State::Idle;
  setStep(now, 0);
  const auto logger = rclcpp::get_logger(kLogger);
  if (unconfirmed.empty())
    RCLCPP_WARN(logger, "MODE D60 OFF (STOPPED) %.2f s sesudah perintah stop (%s); robot diam di pose INIT",
                now - stop_started_at_, stop_reason_.c_str());
  else
    RCLCPP_ERROR(logger, "MODE D60 OFF sesudah batas %.1f s TANPA konfirmasi: %s (%s)",
                 cfg_.stop_confirm_timeout_sec, unconfirmed.c_str(), stop_reason_.c_str());
  if (start_queued_)
  {
    start_queued_ = false;
    queued_start_due_ = true;
    RCLCPP_WARN(logger, "tekanan terakhir saat STOPPING meminta MULAI: sesi baru dimulai dari awal");
  }
}

// ============================================================================
//  Pengamatan turunan
// ============================================================================

bool StrikerDemo::ballValid(double now) const
{
  return ball_seen_ && now - ball_stamp_ <= cfg_.ball_valid_gap_sec;
}

bool StrikerDemo::trackerFresh(double now) const
{
  return happened(tracker_stamp_) && now - tracker_stamp_ <= cfg_.tracker_status_timeout_sec;
}

bool StrikerDemo::presentFresh(double now) const
{
  return happened(present_stamp_) && now - present_stamp_ <= cfg_.head_joint_fresh_sec;
}

// §12: SEARCH = kepala menyapu (pelacak melapor SCANNING/HOLD/tanpa bola),
// TRACK = pelacak melapor TRACKING DAN deteksi bola segar.
striker_demo::HeadMode StrikerDemo::headMode(double now) const
{
  if (!trackerFresh(now) || tracker_text_ == "INACTIVE" || tracker_text_.empty())
    return HeadMode::Inactive;
  if (tracker_text_ == "TRACKING" && ballValid(now))
    return HeadMode::Track;
  return HeadMode::Search;
}

bool StrikerDemo::trackValid(double now) const
{
  return ballValid(now) && ball_tracking_valid_ && headMode(now) == HeadMode::Track && presentFresh(now);
}

double StrikerDemo::servo19() const
{
  return jointToServoDeg(pan_present_, cfg_.servo_center_deg, cfg_.servo_dir);
}

double StrikerDemo::servo20() const
{
  return jointToServoDeg(tilt_present_, cfg_.servo_center_deg, cfg_.servo_dir);
}

double StrikerDemo::panPercent() const
{
  return panPercentOf(servo19(), cfg_.pan_left_limit_servo_deg, cfg_.pan_right_limit_servo_deg);
}

// STRIKER (Titik 2): galat yang masuk ke if/else orbit (tickYaw) diukur terhadap
// target AIM_LOOK -- celah samping kiper / tengah gawang / tabel gyro -- bukan
// auto_demo_yaw_target_deg. Tanda sama dengan error_arah panduan: + = target di
// KANAN -> "Putar Kiri New" (yang di robot ini memutar badan ke kanan).
double StrikerDemo::yawErrorDeg() const
{
  return normalizeAngleDeg(yaw_deg_ - aim_target_yaw_deg_);
}

bool StrikerDemo::yawInTolerance(double err) const
{
  return err <= cfg_.yaw_tolerance_deg && err >= -cfg_.yaw_tolerance_neg_deg;
}

bool StrikerDemo::yawInStopBand(double err) const
{
  return err <= cfg_.yaw_stop_deg && err >= -cfg_.yaw_stop_neg_deg;
}

bool StrikerDemo::yawUsable(double now) const
{
  // STRIKER: yaw gyro sah begitu bias sudah terkumpul (tanpa magnetometer).
  return happened(imu_stamp_) && now - imu_stamp_ <= cfg_.yaw_max_age_sec && std::isfinite(yaw_deg_) &&
         std::isfinite(aim_target_yaw_deg_) && gyro_.biasReady();
}

double StrikerDemo::currentGaitPeriod() const
{
  return (std::isfinite(running_period_) && running_period_ > 0.1) ? running_period_
                                                                    : cfg_.fallback_gait_period_sec;
}

bool StrikerDemo::bodyStopped(double now) const
{
  if (actionRunning(now) || walking_commanded_)
    return false;
  // walking_module baru diam di batas fase berikutnya sesudah "stop".
  if (happened(walking_stop_at_) && now - walking_stop_at_ < cfg_.gait_stop_factor * currentGaitPeriod())
    return false;
  if (now - leg_motion_stamp_ > cfg_.leg_motion_fresh_sec)
    return false;   // tanpa data sendi kaki tidak ada bukti badan diam
  return !(now - leg_fast_last_ < cfg_.leg_quiet_window_sec);
}

bool StrikerDemo::transitionElapsed(double now) const
{
  if (!happened(motion_end_))
    return true;
  // Soft start gait terakhir dibereskan dulu, baru jeda antar-gerakan dihitung.
  const double settled = happened(soft_start_end_) ? std::max(motion_end_, soft_start_end_) : motion_end_;
  return now >= settled + cfg_.transition_sec;
}

bool StrikerDemo::canStartMotion(double now) const
{
  return bodyStopped(now) && transitionElapsed(now);
}

bool StrikerDemo::actionRunning(double now) const
{
  if (our_action_pending_)
    return true;
  return happened(action_poll_stamp_) && now - action_poll_stamp_ <= 0.6 && action_poll_running_;
}

bool StrikerDemo::legsOwnedBy(const std::string &module) const
{
  return legs_owner_ == module;
}

bool StrikerDemo::allOwnedBy(const std::string &module) const
{
  return legs_owner_ == module && head_owner_ == module;
}

std::string StrikerDemo::snapshot(double now) const
{
  const bool tv = trackValid(now);
  return format("[%s/%s langkah %d | kepala %s bola %s valid %d | s19 %.1f (%.1f%%) s20 %.1f | yaw %+.1f err %+.1f "
                "bias %+.2f | pitch %+.1f jatuh %s | kaki '%s' kepala '%s' | walking %d action %d | percobaan #%llu "
                "flags a%d y%d(%d) d%d(%d) s%d k%d kaki %s]",
                striker_demo::toString(state_), striker_demo::toString(mode_), step_, striker_demo::toString(headMode(now)),
                ballValid(now) ? "ADA" : "tidak", tv ? 1 : 0, servo19(), panPercent(), servo20(), yaw_deg_,
                yawErrorDeg(), gyro_.biasDegS(), filtered_pitch_, fallName(fall_), legs_owner_.c_str(),
                head_owner_.c_str(), walking_commanded_ ? 1 : 0, actionRunning(now) ? 1 : 0,
                static_cast<unsigned long long>(attempt_), flags_.approach_completed ? 1 : 0,
                flags_.yaw_stage_completed ? 1 : 0, flags_.yaw_correction_attempted ? 1 : 0,
                flags_.distance_stage_completed ? 1 : 0, flags_.distance_correction_attempted ? 1 : 0,
                flags_.shift_attempted ? 1 : 0, flags_.kick_executed ? 1 : 0, striker_demo::toString(flags_.foot)) +
         format(" striker[bola %s target %+.1f %s lirik %d gerbang %s]", striker::namaBola(pengarah_.indeks()),
                aim_target_yaw_deg_, striker::toString(aim_mode_), flags_.aim_done ? 1 : 0,
                striker::toString(gerbang_.status()));
}

// §13: FALSE KICK PROTECTION. Semua syarat dinilai ulang tiap tick.
bool StrikerDemo::headParked(double now) const
{
  if (!presentFresh(now))
    return false;
  const double pan_deg = rad2deg(pan_present_);
  const double tilt_servo = servo20();
  return std::fabs(pan_deg - cfg_.head_park_pan_deg) <= cfg_.head_park_tol_deg &&
         std::fabs(tilt_servo - cfg_.head_park_tilt_servo_deg) <= cfg_.head_park_tol_deg;
}

std::string StrikerDemo::kickDenialReason(double now, bool after_park) const
{
  if (mode_ != Mode::Running) return "autoDemoEnabled false";
  if (!ballValid(now)) return "ballDetected false";
  if (!after_park)
  {
    if (!ball_tracking_valid_) return "ballTrackingValid false";
    if (headMode(now) != HeadMode::Track) return std::string("headMode ") + striker_demo::toString(headMode(now)) + " (bukan TRACK)";
  }
  if (!presentFresh(now)) return "posisi servo kepala basi";
  if (!flags_.approach_completed) return "approachCompleted false";
  if (!flags_.yaw_stage_completed) return "yawStageCompleted false";
  if (!flags_.distance_stage_completed) return "distanceStageCompleted false";
  if (fall_ != PitchFallState::Stand || now - imu_stamp_ > cfg_.yaw_max_age_sec) return "robotNotFallen false / IMU basi";
  if (actionRunning(now)) return "actionPlayerNotBusy false";
  if (flags_.kick_executed) return "tendangan percobaan ini sudah dikirim";
  if (walking_commanded_) return "walking masih diperintah";
  const double p = panPercent();
  if (!std::isfinite(p)) return "persen servo 19 tidak diketahui";
  // Sesudah geser zona tengah yang batasnya sudah TERSENTUH, atau sesudah
  // kepala diparkir (servo 19 bukan lagi arah bola), persen tidak dinilai lagi.
  if (!flags_.shift_target_touched && !after_park)
  {
    if (flags_.foot == Foot::Left && !(p < cfg_.pan_kick_left))
      return format("servo 19 %.1f%% tidak < %.0f%% untuk tendang kiri", p, cfg_.pan_kick_left);
    if (flags_.foot == Foot::Right && !(p > cfg_.pan_kick_right))
      return format("servo 19 %.1f%% tidak > %.0f%% untuk tendang kanan", p, cfg_.pan_kick_right);
  }
  if (flags_.foot == Foot::None) return "kaki penendang belum dipilih";
  return "";
}

// ============================================================================
//  Perintah
// ============================================================================

void StrikerDemo::clearPresetRequest()
{
  pending_preset_role_ = PresetRole::Count;
  verify_token_ = 0;
  verify_answered_ = false;
  verify_ok_ = false;
  verify_attempts_ = 0;
}

// Preset diterbitkan lalu DIBACA BALIK dari walking_module sebelum dipakai,
// seperti Bascorro Studio: "start" bisa tiba lebih dulu daripada set_params.
// 1 = terbukti hidup, 0 = masih menunggu, -1 = gagal.
int StrikerDemo::presetLanded(double now, PresetRole role)
{
  if (pending_preset_role_ == role && verify_token_ != 0)
  {
    if (verify_answered_ && verify_ok_)
      return 1;
    if (!verify_answered_ && now - verify_sent_at_ <= cfg_.preset_verify_timeout_sec)
      return 0;
  }
  else
  {
    clearPresetRequest();
    pending_preset_role_ = role;
  }
  if (verify_attempts_ >= cfg_.preset_verify_attempts)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                 "preset '%s' (%s) tidak terbukti hidup di walking_module sesudah %d percobaan baca balik",
                 preset(role).name.c_str(), striker_demo::toString(role), cfg_.preset_verify_attempts);
    clearPresetRequest();
    return -1;
  }
  ++verify_attempts_;
  verify_token_ = next_token_++;
  verify_answered_ = false;
  verify_ok_ = false;
  verify_sent_at_ = now;
  publishPreset(role);
  verifyPreset(role, verify_token_);
  return 0;
}

// Mulai gait preset: preset (termasuk init pose miliknya) harus hidup dulu.
// true = "start" sudah dikirim.
bool StrikerDemo::beginWalking(double now, PresetRole role)
{
  if (!preset(role).available)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLogger), *log_clock_, 3000, "preset %s tidak tersedia: %s",
                          striker_demo::toString(role), preset(role).problem.c_str());
    return false;
  }
  if (!legsOwnedBy("walking_module"))
  {
    if (!happened(switch_sent_at_) || now - switch_sent_at_ > 1.0)
    {
      publishJointModules(kWalkingJoints, "walking_module");
      switch_sent_at_ = now;
    }
    return false;
  }
  const int landed = presetLanded(now, role);
  if (landed <= 0)
    return false;
  clearPresetRequest();
  publishWalkingCommand("start");
  walking_commanded_ = true;
  running_period_ = preset(role).period_time;
  running_role_ = role;
  flags_.last_motion_turn_right = (role == PresetRole::TurnRight);
  {
    const auto msg_it = preset_msgs_.find(role);
    const double soft = (msg_it != preset_msgs_.end()) ? std::max(0.0, static_cast<double>(msg_it->second.soft_start_time))
                                                       : 0.0;
    soft_start_end_ = now + soft;
  }
  walking_start_sent_at_ = now;
  walking_start_reported_ = false;
  steer_angle_rad_ = 0.0;
  steer_published_at_ = kNever;
  RCLCPP_WARN(rclcpp::get_logger(kLogger),
              "walking START preset '%s' (%s), periode %.3f s, init pose milik preset itu sendiri | %s",
              preset(role).name.c_str(), striker_demo::toString(role), preset(role).period_time,
              snapshot(now).c_str());
  return true;
}

bool StrikerDemo::walkingStartAccepted(double now, const char *label)
{
  if (!walking_commanded_)
    return true;
  const bool confirmed = walking_status_started_at_ >= walking_start_sent_at_ ||
                         leg_fast_last_ > walking_start_sent_at_;
  if (confirmed)
  {
    if (!walking_start_reported_)
    {
      walking_start_reported_ = true;
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "%s: walking_module MENERIMA start", label);
    }
    return true;
  }
  if (now - walking_start_sent_at_ < cfg_.walking_start_confirm_sec)
    return true;
  RCLCPP_ERROR(rclcpp::get_logger(kLogger),
               "%s: walking_module TIDAK menerima start preset '%s' dalam %.1f s (pemilik kaki '%s')", label,
               preset(running_role_).name.c_str(), cfg_.walking_start_confirm_sec, legs_owner_.c_str());
  return false;
}

void StrikerDemo::stopWalking(double now, const std::string &why)
{
  if (!walking_commanded_)
    return;
  publishWalkingCommand("stop");
  walking_commanded_ = false;
  walking_stop_at_ = now;
  RCLCPP_WARN(rclcpp::get_logger(kLogger), "walking STOP (%s) | %s", why.c_str(), snapshot(now).c_str());
}

// Rumus BallFollower::calcFootstep() D56: kalau |arah bola| > deadband,
// sudut putar = clamp(|arah| x gain, min..max), dinaikkan bertahap ramp per tick
// supaya gait tidak tersentak; tanda mengikuti arah. Arah bola = sudut pan
// kepala (pelacak sudah menaruh bola di tengah gambar). Preset mendekat
// diterbitkan UTUH dengan hanya angle_move_amplitude yang diubah, jadi init
// pose, periode, dan bentuk gait tetap milik "JALAN MENUJU MEKKAH".
void StrikerDemo::steerTowardBall(double now)
{
  if (!walking_commanded_ || running_role_ != PresetRole::Approach || cfg_.steer_gain <= 0.0)
    return;
  const auto it = preset_msgs_.find(PresetRole::Approach);
  if (it == preset_msgs_.end())
    return;

  double target = 0.0;
  if (trackValid(now))
  {
    const double bearing = pan_present_;   // rad, + = kiri (konvensi head_pan)
    if (std::fabs(bearing) > deg2rad(cfg_.steer_deadband_deg))
    {
      double goal = std::fabs(bearing) * cfg_.steer_gain;
      goal = std::min(goal, deg2rad(cfg_.steer_max_deg));
      goal = std::max(goal, deg2rad(cfg_.steer_min_deg));
      target = std::copysign(goal, bearing) * cfg_.steer_sign;
    }
  }
  // Ramp: mendekati sasaran paling banyak steer_ramp_deg per tick, dua arah.
  const double step = deg2rad(cfg_.steer_ramp_deg);
  if (target > steer_angle_rad_ + step)
    steer_angle_rad_ += step;
  else if (target < steer_angle_rad_ - step)
    steer_angle_rad_ -= step;
  else
    steer_angle_rad_ = target;

  if (happened(steer_published_at_) && now - steer_published_at_ < cfg_.steer_publish_period_sec)
    return;
  steer_published_at_ = now;
  WalkingParam msg = it->second;
  msg.angle_move_amplitude = static_cast<float>(it->second.angle_move_amplitude + steer_angle_rad_);
  walking_param_pub_->publish(msg);
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger(kLogger), *log_clock_, 1000,
                       "setir: pan %+.1f deg -> angle %+.1f deg (sasaran %+.1f)", rad2deg(pan_present_),
                       rad2deg(steer_angle_rad_), rad2deg(target));
}

void StrikerDemo::sendPage(double now, int page)
{
  std_msgs::msg::Int32 msg;
  msg.data = page;
  page_pub_->publish(msg);
  our_action_pending_ = true;
  our_action_page_ = page;
  our_action_sent_at_ = now;
  our_action_started_ = false;
  our_action_started_at_ = kNever;
  our_action_failed_ = false;
  RCLCPP_WARN(rclcpp::get_logger(kLogger), "/robotis/action/page_num <- %d (penerima: %zu)", page,
              page_pub_->get_subscription_count());
}

void StrikerDemo::publishPreset(PresetRole role)
{
  const auto it = preset_msgs_.find(role);
  if (it == preset_msgs_.end())
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "preset %s tidak dimuat; tidak diterbitkan", striker_demo::toString(role));
    return;
  }
  walking_param_pub_->publish(it->second);
}

void StrikerDemo::verifyPreset(PresetRole role, uint64_t token)
{
  const auto it = preset_msgs_.find(role);
  if (it == preset_msgs_.end() || !get_walking_param_client_->service_is_ready())
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(kLogger), *log_clock_, 2000,
                         "/robotis/walking/get_params belum siap; verifikasi preset ditunda");
    return;
  }
  const WalkingParam expected = it->second;
  const uint64_t generation = session_;
  auto request = std::make_shared<op3_walking_module_msgs::srv::GetWalkingParam::Request>();
  get_walking_param_client_->async_send_request(
      request,
      [this, token, expected, generation, role](
          rclcpp::Client<op3_walking_module_msgs::srv::GetWalkingParam>::SharedFuture future) {
        if (generation != session_ || token != verify_token_)
          return;
        std::string diff;
        const bool ok = sameWalkingParam(future.get()->parameters, expected, diff);
        if (!ok)
          RCLCPP_WARN(rclcpp::get_logger(kLogger), "preset %s belum sama di walking_module: %s",
                      striker_demo::toString(role), diff.c_str());
        verify_answered_ = true;
        verify_ok_ = ok;
      });
}

void StrikerDemo::publishJointModules(const std::vector<std::string> &joints, const std::string &module)
{
  robotis_controller_msgs::msg::JointCtrlModule msg;
  msg.joint_name = joints;
  msg.module_name.assign(joints.size(), module);
  joint_module_pub_->publish(msg);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "%zu sendi -> %s", joints.size(), module.c_str());
}

void StrikerDemo::publishWalkingCommand(const char *text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  walking_command_pub_->publish(msg);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "/robotis/walking/command <- %s", text);
}

void StrikerDemo::publishTracker(const char *text)
{
  std_msgs::msg::String msg;
  msg.data = text;
  tracker_command_pub_->publish(msg);
}

void StrikerDemo::startTracker(double now)
{
  if (tracker_started_)
    return;
  // Sapuan 7 tahap khusus D60 (head_tracking_node scan_mode "stages"); demo
  // lain tetap sapuan lama -- dikembalikan ke "legacy" saat D60 berhenti.
  // Diam pertama sesudah D60 ditekan: kepala DITAHAN di (50 %, 135), tidak menyapu.
  publishTracker(start_hold_ ? "scan_hold" : "scan_stages");
  publishTracker("start");
  tracker_started_ = true;
  tracker_start_at_ = now;
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "/ball_tracker/command <- start (kepala: sapu / lacak oleh head_tracking_node)");
}

void StrikerDemo::stopTracker()
{
  publishTracker("stop");
  tracker_started_ = false;
  ball_tracking_valid_ = false;
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "/ball_tracker/command <- stop");
}

void StrikerDemo::pollActionRunning(double now)
{
  if (action_poll_in_flight_ || now - last_action_poll_ < 0.25 || !is_running_client_->service_is_ready())
    return;
  action_poll_in_flight_ = true;
  last_action_poll_ = now;
  const double request_time = now;
  auto request = std::make_shared<op3_action_module_msgs::srv::IsRunning::Request>();
  is_running_client_->async_send_request(
      request, [this, request_time](rclcpp::Client<op3_action_module_msgs::srv::IsRunning>::SharedFuture future) {
        action_poll_in_flight_ = false;
        const bool running = future.get()->is_running;
        const double t = steadyNow();
        action_poll_stamp_ = t;
        action_poll_request_time_ = request_time;
        action_poll_running_ = running;
        if (!our_action_pending_)
          return;
        if (running && request_time >= our_action_sent_at_ && !our_action_started_)
        {
          our_action_started_ = true;
          our_action_started_at_ = t;
        }
        else if (!running && our_action_started_ && request_time > our_action_started_at_)
        {
          our_action_pending_ = false;
        }
      });
}

void StrikerDemo::logTelemetry(double now)
{
  if (happened(telemetry_logged_at_) && now - telemetry_logged_at_ < cfg_.telemetry_period_sec)
    return;
  telemetry_logged_at_ = now;
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "%s", snapshot(now).c_str());
}

// ============================================================================
//  Mesin keadaan
// ============================================================================

void StrikerDemo::setStep(double now, int step)
{
  step_ = step;
  step_since_ = now;
}

void StrikerDemo::enterState(double now, State next, const std::string &why)
{
  RCLCPP_WARN(rclcpp::get_logger(kLogger), "STATE %s -> %s: %s | %s", striker_demo::toString(state_),
              striker_demo::toString(next), why.c_str(), snapshot(now).c_str());
  // Keluar dari SearchAlign lewat jalur mana pun (jatuh, bola hilang, stop):
  // correction dianggap selesai, boleh dipicu lagi nanti.
  if (state_ == State::SearchAlign && next != State::SearchAlign)
    search_align_dir_ = striker_demo::AlignDir::None;
  // STRIKER: keluar AIM_LOOK lewat jalur mana pun (jatuh, stop) -> detektor
  // kiper/gawang dimatikan lagi.
  if (state_ == State::AimLook && next != State::AimLook && aim_collecting_)
  {
    aim_collecting_ = false;
    setDetectorsEnabled(false);
  }
  // Sapuan kepala khusus SEARCH_BALL (bola tidak terdeteksi, badan MENCARI CELAH):
  // servo 20 tetap 169, servo 19 bolak-balik 0 <-> 100 %. Keluar dari
  // SEARCH_BALL: kembali ke sapuan 7 tahap D60.
  // Diam pertama (kepala ditahan 135) selesai: bola ketemu -> sapuan 7 tahap
  // seperti biasa; tidak ketemu -> SEARCH_BALL di bawah memasang scan_walk.
  const bool leaving_start_hold = start_hold_ && state_ == State::StandSearch && next != State::StandSearch;
  if (leaving_start_hold)
  {
    start_hold_ = false;
    // Diam pertama habis tanpa bola -> SEARCH_BALL mulai dengan JALAN BARU.
    start_forward_pending_ = (next == State::SearchBall);
  }
  if (state_ == State::SearchBall && next != State::SearchBall)
  {
    start_forward_pending_ = false;
    start_forward_active_ = false;
  }
  if (leaving_start_hold && next != State::SearchBall && mode_ == Mode::Running)
    publishTracker("scan_stages");
  if (next == State::SearchBall && state_ != State::SearchBall && mode_ == Mode::Running)
    publishTracker("scan_walk");
  else if (state_ == State::SearchBall && next != State::SearchBall && mode_ == Mode::Running)
    publishTracker("scan_stages");
  state_ = next;
  state_since_ = now;
  setStep(now, 0);
  clearPresetRequest();
  yaw_tolerance_seen_ = false;
  yaw_stop_reached_ = false;
}

// INIT "JALAN MENUJU MEKKAH" (pose page 2 versi save-an itu), lalu diam mencari
// selama search_sec, lalu StandSearch memutuskan.
void StrikerDemo::enterInit(double now, State after, double search_sec, bool reset_flags, const std::string &why)
{
  after_init_ = after;
  after_init_search_sec_ = search_sec;
  flags_.last_motion_turn_right = false;
  after_init_reset_flags_ = reset_flags;
  stopWalking(now, "masuk INIT");
  enterState(now, State::Init, why);
}

// §20/§26/§42: bola hilang saat badan bergerak -> data lama DIBUANG, motion
// dihentikan, INIT, diam mencari 3 s. Penanda percobaan TIDAK direset.
void StrikerDemo::enterLostRecovery(double now, const std::string &why)
{
  ball_tracking_valid_ = false;
  RCLCPP_WARN(rclcpp::get_logger(kLogger), "BOLA HILANG saat %s (%s): data lama tidak dipakai; INIT lalu cari %.1f s",
              striker_demo::toString(state_), why.c_str(), cfg_.lost_search_sec);
  enterInit(now, State::StandSearch, cfg_.lost_search_sec, false, "BALL_LOST_RECOVERY");
}

// §31-§33: jatuh punya prioritas di atas semua tahap.
void StrikerDemo::enterGetUp(double now, PitchFallState fall)
{
  RCLCPP_ERROR(rclcpp::get_logger(kLogger),
               "JATUH %s terdeteksi (pitch EMA %.1f deg, ambang D56 %+.0f/%+.0f) saat %s | %s", fallName(fall),
               filtered_pitch_, kFallForwardLimitDeg, kFallBackLimitDeg, striker_demo::toString(state_),
               snapshot(now).c_str());
  ball_tracking_valid_ = false;
  clearPresetRequest();
  stopWalking(now, "jatuh");
  stopTracker();
  if (actionRunning(now))
  {
    // Robot SUDAH jatuh: melanjutkan halaman yang sedang main (mis. tendangan)
    // di lantai tidak ada gunanya. Rem seketika, lalu bangun.
    std_msgs::msg::Int32 msg;
    msg.data = -2;
    page_pub_->publish(msg);
    our_action_pending_ = false;
  }
  // Init kepala sebelum halaman bangun: pelacak sudah di-"stop" (parkir);
  // sendi diserahkan ke action_module di tickGetUp sesudah kepala sampai
  // atau batas waktu.
  head_park_sent_at_ = now;
  switch_sent_at_ = kNever;
  enterState(now, State::GettingUp, "prioritas bangun");
}

// Sesudah bola terlihat lagi: lanjut tahap yang belum selesai, sesuai penanda.
void StrikerDemo::routeAfterTrack(double now)
{
  ball_tracking_valid_ = true;
  if (!flags_.approach_completed)
    enterState(now, State::Approach, "tracking sah; mendekati bola dengan \"" + preset(PresetRole::Approach).name + "\"");
  else if (!flags_.yaw_stage_completed)
    enterOrbitStage(now, "tracking sah; lanjut koreksi yaw");
  else if (!flags_.distance_stage_completed)
    enterState(now, State::DistanceRecheck, "tracking sah; periksa jarak lagi");
  else
    enterState(now, State::BallAlignment, "tracking sah; lanjut penyelarasan horizontal");
}

void StrikerDemo::tick(double now)
{
  if (mode_ == Mode::Stopped)
    return;
  if (mode_ == Mode::Stopping)
  {
    tickStopping(now);
    return;
  }

  // Sendi DIREBUT modul lain sesudah D60 memasang modulnya (user_long ->
  // base_module, Torque OFF Studio -> "none"): mode berhenti, tidak melawan.
  {
    const auto foreign = [](const std::string &module) {
      return module == "base_module" || module == "none" || module.empty();
    };
    if (owners_claimed_ && foreign(legs_owner_))
    {
      beginStop(now, format("sendi kaki diambil modul lain ('%s')", legs_owner_.c_str()));
      return;
    }
  }

  // §35 prioritas 2-3: FALL DETECTION di atas semua tahap kecuali saat
  // sedang bangun (jangan memanggil get-up berulang).
  if (state_ != State::GettingUp && fall_ != PitchFallState::Stand && now - imu_stamp_ <= cfg_.yaw_max_age_sec)
  {
    enterGetUp(now, fall_);
    return;
  }

  // §27: begitu bola hilang, semua data lama tidak sah; begitu pelacak
  // mengunci lagi (TRACKING + deteksi segar), data BARU sah kembali.
  if (!ballValid(now))
    ball_tracking_valid_ = false;
  else if (headMode(now) == HeadMode::Track)
    ball_tracking_valid_ = true;

  switch (state_)
  {
  case State::Init: tickInit(now); break;
  case State::StandSearch: tickStandSearch(now); break;
  case State::SearchBall: tickSearchBall(now); break;
  case State::SearchAlign: tickSearchAlign(now); break;
  case State::Approach: tickApproach(now); break;
  case State::AimLook: tickAimLook(now); break;
  case State::YawCorrection: tickYaw(now); break;
  case State::DistanceRecheck: tickDistance(now); break;
  case State::BallAlignment: tickAlignment(now); break;
  case State::Shift: tickShift(now); break;
  case State::PreKickNudge: tickPreKickNudge(now); break;
  case State::Kick: tickKick(now); break;
  case State::GettingUp: tickGetUp(now); break;
  case State::Park: tickPark(now); break;
  case State::Idle: break;
  }
  logTelemetry(now);
}

// ---- INIT -------------------------------------------------------------------
//
// Pose berdiri "JALAN MENUJU MEKKAH" = preset INIT dengan acuan page 2, TANPA
// start: walking_module yang diam menggeser kaki ke pose netral offset init_*
// (dibatasi 25 deg/s), jadi robot mengambil posenya tanpa melangkah.

void StrikerDemo::tickInit(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  switch (step_)
  {
  case 0:
    // Gait apa pun yang mungkin masih jalan dihentikan dulu.
    publishWalkingCommand("stop");
    walking_commanded_ = false;
    walking_stop_at_ = now;
    if (!std::isfinite(running_period_))
      running_period_ = cfg_.fallback_gait_period_sec;
    setStep(now, 1);
    return;
  case 1:
    if (now - leg_motion_stamp_ > cfg_.leg_motion_fresh_sec || !presentFresh(now))
    {
      RCLCPP_WARN_THROTTLE(logger, *log_clock_, 2000,
                           "INIT menunggu controller siap (goal/present_joint_states belum segar)");
      return;
    }
    if (actionRunning(now))
      return;   // halaman action (bangun/tendang) diselesaikan dulu
    if (!bodyStopped(now))
      return;
    setStep(now, 2);
    return;
  case 2:
  {
    if (legsOwnedBy("walking_module"))
    {
      setStep(now, 4);
      return;
    }
    if (!preset(PresetRole::PoseReference).available)
    {
      RCLCPP_ERROR_THROTTLE(logger, *log_clock_, 3000, "preset INIT '%s' tidak tersedia: %s -- D60 tidak bisa lanjut",
                            preset(PresetRole::Init).name.c_str(), preset(PresetRole::PoseReference).problem.c_str());
      return;
    }
    // Acuan pose HARUS hidup sebelum walking_module menyala: bias posenya
    // direkam pada tick pertama sesudah modul aktif (sama dengan Studio/D56).
    const int landed = presetLanded(now, PresetRole::PoseReference);
    if (landed < 0)
    {
      setStep(now, 2);   // coba lagi
      return;
    }
    if (landed == 0)
      return;
    clearPresetRequest();
    publishJointModules(kWalkingJoints, "walking_module");
    switch_sent_at_ = now;
    setStep(now, 3);
    return;
  }
  case 3:
    if (legsOwnedBy("walking_module"))
    {
      owners_claimed_ = true;
      setStep(now, 4);
      return;
    }
    if (now - switch_sent_at_ > cfg_.module_switch_timeout_sec)
    {
      RCLCPP_ERROR(logger, "walking_module tidak memegang kaki dalam %.1f s (pemilik: '%s'); dicoba lagi",
                   cfg_.module_switch_timeout_sec, legs_owner_.c_str());
      setStep(now, 2);
    }
    return;
  case 4:
  {
    const int landed = presetLanded(now, PresetRole::Init);
    if (landed < 0)
    {
      setStep(now, 4);
      return;
    }
    if (landed == 0)
      return;
    clearPresetRequest();
    owners_claimed_ = true;
    RCLCPP_WARN(logger, "pose INIT \"%s\" (acuan page 2) diterapkan TANPA start walking",
                preset(PresetRole::Init).name.c_str());
    setStep(now, 5);
    return;
  }
  case 5:
    if (now - step_since_ < cfg_.leg_quiet_window_sec || now - leg_motion_stamp_ > cfg_.leg_motion_fresh_sec)
      return;
    if (now - leg_moving_last_ < cfg_.leg_quiet_window_sec)
      return;
    motion_end_ = now;
    if (after_init_ == State::Idle)
    {
      enterState(now, State::Idle, "INIT selesai; diam");
      return;
    }
    // Kepala mencari SAMBIL robot diam: pelacak dinyalakan sesudah tidak ada
    // action yang berjalan (start merebut sendi kepala).
    startTracker(now);
    // STRIKER: cooldown pasca-tendang dihitung dari saat kepala menyala lagi.
    if (post_kick_look_pending_)
    {
      post_kick_look_pending_ = false;
      pengarah_.mulaiCooldown(now);
      RCLCPP_WARN(logger, "STRIKER: cooldown %.1f s mulai -- bola jauh lurus ke depan (bola yang baru ditendang) "
                  "diabaikan; target sekarang bola %s", cfg_.striker.strategy.post_kick_ignore_sec,
                  striker::namaBola(pengarah_.indeks()));
    }
    if (after_init_reset_flags_)
      resetAttemptFlags(now, "mulai mencari sesudah INIT");
    stand_search_deadline_ = now + after_init_search_sec_;
    enterState(now, after_init_, format("diam mencari bola maksimal %.1f s", after_init_search_sec_));
    return;
  default:
    return;
  }
}

// ---- STAND SEARCH -------------------------------------------------------------

void StrikerDemo::tickStandSearch(double now)
{
  if (!transitionElapsed(now))
    return;
  if (trackValid(now) || (ballValid(now) && headMode(now) == HeadMode::Track && presentFresh(now)))
  {
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "bola DITEMUKAN saat diam (%.1f s sebelum batas): HEAD_MODE_TRACK",
                stand_search_deadline_ - now);
    if (maybeEnterSearchAlign(now))
      return;
    routeAfterTrack(now);
    return;
  }
  if (now >= stand_search_deadline_)
  {
    enterState(now, State::SearchBall, format("bola tidak ditemukan dalam %.1f s: \"%s\" + sapuan kepala",
                                              after_init_search_sec_, preset(PresetRole::Search).name.c_str()));
    return;
  }
  if (headMode(now) == HeadMode::Inactive && now - tracker_start_at_ > 3.0)
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLogger), *log_clock_, 3000,
                          "head_tracking_node tidak melaporkan status di /ball_tracker/status");
}

// ---- SEARCH BALL ("MENCARI CELAH" + sapuan kepala) ------------------------------

void StrikerDemo::tickSearchBall(double now)
{
  switch (step_)
  {
  case 0:
  {
    startTracker(now);
    if (!canStartMotion(now))
      return;
    // Sesi baru (D60 ditekan), diam pertama habis tanpa bola: jalan dulu dengan
    // preset mendekat (JALAN BARU) selama start_forward_sec sambil kepala menyapu.
    const bool forward = start_forward_pending_ && cfg_.start_forward_sec > 0.0;
    // STRIKER (Titik 1): arah putar MENCARI CELAH mengikuti bola target B/A/C/D.
    const PresetRole search_role = rolePutarCari();
    if (!beginWalking(now, forward ? PresetRole::Approach : search_role))
      return;
    if (!forward)
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "STRIKER: cari bola %s -> putar %s (\"%s\")",
                  striker::namaBola(pengarah_.indeks()),
                  striker::toString(pengarah_.arahPutar(yawUsable(now) ? yaw_deg_ : kNaN)),
                  preset(search_role).name.c_str());
    if (forward)
    {
      start_forward_pending_ = false;
      start_forward_active_ = true;
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "awal sesi: \"%s\" %.1f s sambil kepala menyapu; bola terlihat = kejar, "
                  "tidak = lanjut \"%s\"", preset(PresetRole::Approach).name.c_str(), cfg_.start_forward_sec,
                  preset(PresetRole::Search).name.c_str());
    }
    setStep(now, 1);
    return;
  }
  case 1:
    if (!walkingStartAccepted(now, "SEARCH_BALL"))
    {
      stopWalking(now, "start ditolak");
      setStep(now, 0);
      return;
    }
    if (ballValid(now) && headMode(now) == HeadMode::Track)
    {
      // Bola di zona luar: lanjut MENCARI CELAH sebagai correction (gait tidak
      // dihentikan; yaw diganti live kalau perlu) sampai lewat 50 %.
      if (presentFresh(now) && maybeEnterSearchAlign(now))
        return;
      // §15: hentikan "MENCARI CELAH" secara aman, kepala TRACK, transisi 0,3 s.
      start_forward_active_ = false;
      stopWalking(now, "bola ditemukan saat mencari");
      setStep(now, 2);
      return;
    }
    if (start_forward_active_ && now - step_since_ > cfg_.start_forward_sec)
    {
      start_forward_active_ = false;
      stopWalking(now, format("\"%s\" %.1f s tanpa bola; lanjut \"%s\"", preset(PresetRole::Approach).name.c_str(),
                              cfg_.start_forward_sec, preset(PresetRole::Search).name.c_str()));
      setStep(now, 0);
      return;
    }
    if (now - step_since_ > cfg_.search_timeout_sec)
    {
      stopWalking(now, format("batas mencari %.0f s; INIT lalu cari lagi", cfg_.search_timeout_sec));
      enterInit(now, State::StandSearch, cfg_.initial_search_sec, false, "mencari terlalu lama");
    }
    return;
  case 2:
    if (!bodyStopped(now))
      return;
    if (!happened(motion_end_) || motion_end_ < walking_stop_at_)
      motion_end_ = now;
    if (!transitionElapsed(now))
      return;
    if (trackValid(now) || (ballValid(now) && headMode(now) == HeadMode::Track && presentFresh(now)))
    {
      if (maybeEnterSearchAlign(now))
        return;
      // MENCARI CELAH selesai -> jeda tambahan sebelum jalan (motion_end_ digeser).
      motion_end_ = now + cfg_.search_to_walk_delay_sec - cfg_.transition_sec;
      routeAfterTrack(now);
      return;
    }
    // Bola hilang lagi selama gait reda: lanjut mencari.
    if (now - step_since_ > cfg_.ball_lost_sec + cfg_.transition_sec)
      setStep(now, 0);
    return;
  default:
    return;
  }
}

// ---- SEARCH ALIGN ("MENCARI CELAH" sebagai correction saat mencari) -----------------
//
// Bola pertama terlihat saat mencari dengan servo 19 di 0-30 % (FromLeft) atau
// 70-100 % (FromRight): badan memakai "MENCARI CELAH" UTUH (init pose, soft
// start, periode, z, ... milik save-an itu) -- FromLeft dengan
// angle_move_amplitude dibalik tandanya (runtime), FromRight asli -- sampai
// persen servo 19 LIVE melewati target (> 40 dari kiri, < 60 dari kanan). Kepala
// tetap dipegang head_tracking_node (tidak ada perintah ke pelacak). Selama
// state ini tidak ada keputusan badan lain; sesudah gait berhenti barulah
// routeAfterTrack() memutuskan tahap berikutnya.

bool StrikerDemo::maybeEnterSearchAlign(double now)
{
  if (search_align_dir_ != striker_demo::AlignDir::None)
    return false;   // sudah aktif: tidak dipicu ulang
  if (!(ballValid(now) && headMode(now) == HeadMode::Track && presentFresh(now)))
    return false;
  const double p = panPercent();
  if (!std::isfinite(p))
    return false;
  striker_demo::AlignDir dir = striker_demo::AlignDir::None;
  if (p <= cfg_.search_align_left_max)
    dir = striker_demo::AlignDir::FromLeft;
  else if (p >= cfg_.search_align_right_min)
    dir = striker_demo::AlignDir::FromRight;
  if (dir == striker_demo::AlignDir::None)
    return false;   // di antara left_max dan right_min: logika D60 lama
  const PresetRole role = (dir == striker_demo::AlignDir::FromLeft) ? PresetRole::SearchInverted : PresetRole::Search;
  if (!preset(role).available)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(kLogger), *log_clock_, 3000,
                          "SearchAlign dilewati: preset %s tidak tersedia (%s)", striker_demo::toString(role),
                          preset(role).problem.c_str());
    return false;
  }
  search_align_dir_ = dir;
  ball_tracking_valid_ = true;
  enterState(now, State::SearchAlign,
             format("bola ketemu di servo 19 %.1f%% (%s): \"%s\" %s sampai persen %s %.0f%%, kepala tetap search/track", p,
                    striker_demo::toString(dir), preset(PresetRole::Search).name.c_str(),
                    dir == striker_demo::AlignDir::FromLeft ? "dengan yaw DIBALIK" : "ASLI",
                    dir == striker_demo::AlignDir::FromLeft ? ">" : "<",
                    dir == striker_demo::AlignDir::FromLeft ? cfg_.search_align_stop_left : cfg_.search_align_stop_right));
  return true;
}

void StrikerDemo::tickSearchAlign(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  const striker_demo::AlignDir dir = search_align_dir_;
  const PresetRole role = (dir == striker_demo::AlignDir::FromLeft) ? PresetRole::SearchInverted : PresetRole::Search;
  if (dir == striker_demo::AlignDir::None)
  {
    enterState(now, State::SearchBall, "SearchAlign tanpa arah; kembali mencari");
    return;
  }
  switch (step_)
  {
  case 0:
    if (walking_commanded_ && (running_role_ == PresetRole::Search || running_role_ == PresetRole::SearchInverted))
    {
      // Gait MENCARI CELAH sudah jalan (dari SEARCH_BALL): jangan stop/start
      // lagi -- cukup ganti parameternya live kalau yaw harus dibalik.
      if (running_role_ != role)
      {
        publishPreset(role);
        running_role_ = role;
        RCLCPP_WARN(logger, "MENCARI CELAH tetap jalan, parameter diganti live -> %s", striker_demo::toString(role));
      }
      setStep(now, 1);
      return;
    }
    if (!canStartMotion(now))
      return;
    if (!beginWalking(now, role))
      return;
    setStep(now, 1);
    return;
  case 1:
  {
    if (!walkingStartAccepted(now, "SEARCH_ALIGN"))
    {
      stopWalking(now, "start ditolak");
      setStep(now, 2);
      return;
    }
    if (!ballValid(now))
    {
      if (now - ball_stamp_ > cfg_.ball_lost_sec)
      {
        stopWalking(now, "bola hilang saat SearchAlign");
        search_align_dir_ = striker_demo::AlignDir::None;
        enterLostRecovery(now, "SearchAlign");
      }
      return;   // putus sesaat: motion terus, kepala tetap search/track
    }
    // Persen servo 19 LIVE (tracking sah saat ini), bukan nilai sebelum mulai.
    const double p = trackValid(now) ? panPercent() : kNaN;
    const bool crossed = std::isfinite(p) &&
        ((dir == striker_demo::AlignDir::FromLeft && p > cfg_.search_align_stop_left) ||
         (dir == striker_demo::AlignDir::FromRight && p < cfg_.search_align_stop_right));
    if (crossed)
    {
      stopWalking(now, format("servo 19 %.1f%% sudah %s %.0f%% (%s)", p,
                              dir == striker_demo::AlignDir::FromLeft ? ">" : "<",
                              dir == striker_demo::AlignDir::FromLeft ? cfg_.search_align_stop_left
                                                                   : cfg_.search_align_stop_right,
                              striker_demo::toString(dir)));
      setStep(now, 2);
      return;
    }
    if (now - step_since_ > cfg_.search_timeout_sec)
    {
      stopWalking(now, format("batas SearchAlign %.0f s (servo 19 %.1f%%)", cfg_.search_timeout_sec, p));
      setStep(now, 2);
    }
    return;
  }
  case 2:
    // Berhenti AMAN dulu (gait reda + transisi), baru keputusan D60 berikutnya.
    if (!bodyStopped(now))
      return;
    if (!happened(motion_end_) || motion_end_ < walking_stop_at_)
      motion_end_ = now;
    if (!transitionElapsed(now))
      return;
    search_align_dir_ = striker_demo::AlignDir::None;
    if (trackValid(now) || (ballValid(now) && headMode(now) == HeadMode::Track && presentFresh(now)))
    {
      // MENCARI CELAH selesai -> jeda tambahan sebelum jalan (motion_end_ digeser).
      motion_end_ = now + cfg_.search_to_walk_delay_sec - cfg_.transition_sec;
      routeAfterTrack(now);
      return;
    }
    if (now - step_since_ > cfg_.ball_lost_sec + cfg_.transition_sec)
      enterState(now, State::SearchBall, "bola hilang sesudah SearchAlign; mencari lagi");
    return;
  default:
    return;
  }
}

// ---- APPROACH ("JALAN MENUJU MEKKAH", servo 20 = jarak) ---------------------------

void StrikerDemo::tickApproach(double now)
{
  switch (step_)
  {
  case 0:
    if (!canStartMotion(now))
      return;
    if (!trackValid(now))
    {
      if (now - state_since_ > cfg_.ball_lost_sec)
        enterLostRecovery(now, "sebelum mulai mendekat");
      return;
    }
    // Sudah dekat sejak awal: tidak perlu jalan.
    if (servo20() > cfg_.tilt_limit_deg)
    {
      flags_.approach_completed = true;
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "servo 20 %.1f > %.1f: bola sudah DEKAT, tidak perlu mendekat",
                  servo20(), cfg_.tilt_limit_deg);
      enterOrbitStage(now, "bola sudah dekat");
      return;
    }
    if (!beginWalking(now, PresetRole::Approach))
      return;
    setStep(now, 1);
    return;
  case 1:
  {
    if (!walkingStartAccepted(now, "APPROACH"))
    {
      stopWalking(now, "start ditolak");
      setStep(now, 0);
      return;
    }
    steerTowardBall(now);   // badan mengikuti arah kepala selama berjalan
    if (!trackValid(now))
    {
      if (now - ball_stamp_ > cfg_.ball_lost_sec)
      {
        stopWalking(now, "bola hilang saat mendekat");
        enterLostRecovery(now, "mendekat");
      }
      return;   // bola putus sesaat: terus jalan
    }
    const double s20 = servo20();
    if (s20 > cfg_.tilt_limit_deg)
    {
      stopWalking(now, format("servo 20 %.1f > %.1f: bola sudah DEKAT", s20, cfg_.tilt_limit_deg));
      setStep(now, 2);
      return;
    }
    if (now - step_since_ > cfg_.approach_timeout_sec)
    {
      stopWalking(now, format("batas mendekat %.0f s", cfg_.approach_timeout_sec));
      enterLostRecovery(now, "mendekat terlalu lama");
    }
    return;
  }
  case 2:
    if (!bodyStopped(now))
      return;
    if (!happened(motion_end_) || motion_end_ < walking_stop_at_)
      motion_end_ = now;
    if (!transitionElapsed(now))
      return;
    flags_.approach_completed = true;
    enterOrbitStage(now, "APPROACH selesai; transisi 0,3 s lewat");
    return;
  default:
    return;
  }
}

// ---- STRIKER: AIM_LOOK (Titik 2, target orbit) --------------------------------------------
//
// Saat orbit kepala menunduk ke bola di kaki (servo 20 > 200), jadi kiper dan
// gawang tidak masuk frame. Sekali per percobaan, sebelum YAW_CORRECTION:
// pelacak "stop:<servo20>" (perintah parkir yang SUDAH ada) -> kepala pan 0 /
// servo 20 lirik -> yolo_gawang_detector kiper & gawang dinyalakan sebentar ->
// target yaw dihitung (hitungTargetOrbit) -> pelacak "start" -> tunggu TRACK.

void StrikerDemo::enterOrbitStage(double now, const std::string &why)
{
  if (!flags_.aim_done)
    enterState(now, State::AimLook, why + "; lirik kiper/gawang dulu");
  else
    enterState(now, State::YawCorrection, why);
}

void StrikerDemo::setAimTargetFromTable(const char *why)
{
  aim_mode_ = striker::ModeTarget::GyroDR;
  aim_target_yaw_deg_ = cfg_.striker.strategy.gyro_target_yaw_per_step[static_cast<size_t>(pengarah_.indeks())];
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "STRIKER: target orbit bola %s = tabel gyro %+.1f deg (%s)",
              striker::namaBola(pengarah_.indeks()), aim_target_yaw_deg_, why);
}

void StrikerDemo::setDetectorsEnabled(bool enabled)
{
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  keeper_enable_pub_->publish(msg);
  goalpost_enable_pub_->publish(msg);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "detektor kiper/gawang %s", enabled ? "DINYALAKAN" : "dimatikan");
}

striker_demo::PresetRole StrikerDemo::rolePutarCari() const
{
  const double now = steadyNow();
  const bool kanan = pengarah_.arahPutar(yawUsable(now) ? yaw_deg_ : kNaN) == striker::ArahPutar::Kanan;
  return (kanan == cfg_.striker.search_original_is_right) ? PresetRole::Search : PresetRole::SearchInverted;
}

void StrikerDemo::tickAimLook(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  const striker_demo::StrikerConfig &sc = cfg_.striker;
  switch (step_)
  {
  case 0:
  {
    if (!canStartMotion(now))
      return;
    if (!sc.aim_look_enable)
    {
      setAimTargetFromTable("lirik dimatikan (striker_aim_look_enable false)");
      flags_.aim_done = true;
      enterState(now, State::YawCorrection, "target orbit dari tabel gyro");
      return;
    }
    char cmd[32];
    std::snprintf(cmd, sizeof(cmd), "stop:%.1f", sc.aim_look_tilt_servo_deg);
    publishTracker(cmd);
    tracker_started_ = false;
    ball_tracking_valid_ = false;
    aim_sent_at_ = now;
    kumpul_.reset();
    RCLCPP_WARN(logger, "AIM_LOOK bola %s: /ball_tracker/command <- %s (kepala pan %+.0f, servo 20 %.0f) untuk "
                "membaca kiper/gawang", striker::namaBola(pengarah_.indeks()), cmd, cfg_.head_park_pan_deg,
                sc.aim_look_tilt_servo_deg);
    setStep(now, 1);
    return;
  }
  case 1:
  {
    const bool sampai = presentFresh(now) &&
                        std::fabs(rad2deg(pan_present_) - cfg_.head_park_pan_deg) <= sc.aim_look_tol_deg &&
                        std::fabs(servo20() - sc.aim_look_tilt_servo_deg) <= sc.aim_look_tol_deg;
    if (!sampai && now - aim_sent_at_ < sc.aim_look_wait_sec)
      return;
    kumpul_.reset();
    aim_collect_ros_start_ = node_->now();
    aim_collecting_ = true;
    aim_pan_deg_ = rad2deg(pan_present_);
    setDetectorsEnabled(true);
    RCLCPP_INFO(logger, "AIM_LOOK: kepala %s (pan %+.1f, servo 20 %.1f) sesudah %.2f s; kumpul %.1f s",
                sampai ? "sampai" : "BELUM sampai, batas waktu", aim_pan_deg_, servo20(), now - aim_sent_at_,
                sc.aim_collect_sec);
    setStep(now, 2);
    return;
  }
  case 2:
  {
    if (now - step_since_ < sc.aim_collect_sec)
      return;
    aim_collecting_ = false;
    setDetectorsEnabled(false);
    const double pan_deg = presentFresh(now) ? rad2deg(pan_present_) : aim_pan_deg_;
    const double yaw = yawUsable(now) ? yaw_deg_ : kNaN;
    const striker::PengamatanTarget &obs = kumpul_.hasil();
    const striker::HasilTarget hasil = striker::hitungTargetOrbit(obs, pan_deg, yaw, pengarah_.indeks(), sc.strategy);
    aim_mode_ = hasil.mode;
    aim_target_yaw_deg_ = hasil.target_yaw_deg;
    flags_.aim_done = true;
    RCLCPP_WARN(logger,
                "AIM_LOOK bola %s -> %s: target yaw %+.1f (yaw sekarang %+.1f, pan %+.1f) | kiper %s cx %.0f w %.0f "
                "(%d frame) | gawang %s cx %.0f (%d frame) | target_x %.0f px, offset %.0f px, celah %s kiper",
                striker::namaBola(pengarah_.indeks()), striker::toString(hasil.mode), hasil.target_yaw_deg, yaw,
                pan_deg, obs.kiper_valid ? "ADA" : "tidak", obs.kiper_cx, obs.kiper_w, obs.frame_kiper,
                obs.gawang_valid ? "ADA" : "tidak", obs.gawang_cx, obs.frame_gawang, hasil.target_x_px,
                hasil.offset_px, hasil.sisi_celah < 0 ? "KIRI" : (hasil.sisi_celah > 0 ? "KANAN" : "-"));
    startTracker(now);
    setStep(now, 3);
    return;
  }
  case 3:
    if (trackValid(now))
    {
      enterState(now, State::YawCorrection,
                 format("bola terkunci lagi sesudah lirik; orbit ke target %+.1f (%s)", aim_target_yaw_deg_,
                        striker::toString(aim_mode_)));
      return;
    }
    if (now - step_since_ > sc.aim_reacquire_sec)
      enterLostRecovery(now, "bola tidak terkunci lagi sesudah lirik");
    return;
  default:
    return;
  }
}

// ---- YAW CORRECTION -----------------------------------------------------------------
//
// yaw > +toleransi -> "PUTAR KIRI NEW", yaw < -toleransi -> "PUTAR KANAN NEW",
// SEKALI per percobaan. Kepala tetap TRACK selama badan memutar.

void StrikerDemo::tickYaw(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  switch (step_)
  {
  case 0:
  {
    if (!canStartMotion(now))
      return;
    if (!trackValid(now))
    {
      if (now - state_since_ > cfg_.ball_lost_sec)
        enterLostRecovery(now, "sebelum koreksi yaw");
      return;
    }
    if (!yawUsable(now))
    {
      if (now - state_since_ > 2.0)
      {
        RCLCPP_ERROR(logger, "yaw GYRO tidak bisa dipakai (bias %s, target %+.1f, umur %.1f s): koreksi yaw DILEWATI",
                     gyro_.biasReady() ? "siap" : "BELUM terkumpul", aim_target_yaw_deg_, now - imu_stamp_);
        flags_.yaw_stage_completed = true;
        enterState(now, State::DistanceRecheck, "yaw tidak tersedia");
      }
      return;
    }
    const double err = yawErrorDeg();
    // Cukup PERNAH terlihat di dalam toleransi (jepitan dari IMU 125 Hz),
    // walau cuma sesaat: tidak perlu memutar.
    if (yawInTolerance(err) || yaw_tolerance_seen_)
    {
      flags_.yaw_stage_completed = true;
      RCLCPP_WARN(logger, "yaw %+.1f (galat %+.1f) %s di dalam %+.1f..%+.1f: tidak perlu memutar", yaw_deg_, err,
                  yawInTolerance(err) ? "sudah" : "sempat terlihat", -cfg_.yaw_tolerance_neg_deg,
                  cfg_.yaw_tolerance_deg);
      enterState(now, State::DistanceRecheck, "yaw sudah sesuai");
      return;
    }
    if (flags_.yaw_correction_attempted)
    {
      flags_.yaw_stage_completed = true;
      RCLCPP_WARN(logger, "yaw galat %+.1f masih di luar %+.1f..%+.1f, tapi koreksi yaw sudah dipakai percobaan ini: "
                          "lanjut tanpa memutar lagi", err, -cfg_.yaw_tolerance_neg_deg, cfg_.yaw_tolerance_deg);
      enterState(now, State::DistanceRecheck, "koreksi yaw sudah sekali");
      return;
    }
    const PresetRole role = (err > 0.0) ? PresetRole::TurnLeft : PresetRole::TurnRight;
    if (!beginWalking(now, role))
      return;
    flags_.yaw_correction_attempted = true;
    RCLCPP_WARN(logger, "YAW %+.1f (galat %+.1f) -> \"%s\" SEKALI; berhenti di galat %+.1f..%+.1f; kepala tetap TRACK",
                yaw_deg_, err, preset(role).name.c_str(), -cfg_.yaw_stop_neg_deg, cfg_.yaw_stop_deg);
    setStep(now, 1);
    return;
  }
  case 1:
  {
    if (!walkingStartAccepted(now, "YAW"))
    {
      stopWalking(now, "start ditolak");
      flags_.yaw_stage_completed = true;
      enterState(now, State::DistanceRecheck, "putaran tidak diterima walking_module");
      return;
    }
    // §20: bola hilang saat memutar -> recovery, data lama tidak dipakai.
    if (!ballValid(now) && now - ball_stamp_ > cfg_.ball_lost_sec)
    {
      stopWalking(now, "bola hilang saat memutar");
      enterLostRecovery(now, "memutar");
      return;
    }
    const double err = yawUsable(now) ? yawErrorDeg() : kNaN;
    // Berhenti begitu |galat| <= stop PERNAH tercapai, walau cuma sepersekian
    // detik (jepitan dari IMU). Kalau sesudah itu badan kelebihan putar, tidak
    // dinilai ulang dan tidak diputar balik: tahap yaw dianggap selesai.
    if (yaw_stop_reached_ || (std::isfinite(err) && yawInStopBand(err)))
    {
      stopWalking(now, format("yaw galat %+.1f: pita stop %+.1f..%+.1f sudah tercapai (sekarang %+.1f)", err,
                              -cfg_.yaw_stop_neg_deg, cfg_.yaw_stop_deg, err));
      setStep(now, 2);
      return;
    }
    if (now - step_since_ > cfg_.yaw_timeout_sec)
    {
      stopWalking(now, format("batas memutar %.0f s (galat %+.1f)", cfg_.yaw_timeout_sec, err));
      setStep(now, 2);
    }
    return;
  }
  case 2:
    if (!bodyStopped(now))
      return;
    if (!happened(motion_end_) || motion_end_ < walking_stop_at_)
      motion_end_ = now;
    if (!transitionElapsed(now))
      return;
    flags_.yaw_stage_completed = true;
    RCLCPP_WARN(logger, "putaran selesai: yaw %+.1f (galat %+.1f)%s", yaw_deg_, yawErrorDeg(),
                !yawInTolerance(yawErrorDeg()) ? " -- kelebihan/diluar toleransi, tetap lanjut" : "");
    enterState(now, State::DistanceRecheck, "koreksi yaw selesai; transisi 0,3 s lewat");
    return;
  default:
    return;
  }
}

// ---- DISTANCE RECHECK (servo 20, maju SEKALI kalau masih jauh) --------------------------

void StrikerDemo::tickDistance(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  switch (step_)
  {
  case 0:
  {
    if (!canStartMotion(now))
      return;
    if (!trackValid(now))
    {
      if (now - state_since_ > cfg_.ball_lost_sec)
        enterLostRecovery(now, "sebelum periksa jarak");
      return;
    }
    const double s20 = servo20();
    if (s20 > cfg_.tilt_limit_deg)
    {
      flags_.distance_stage_completed = true;
      RCLCPP_WARN(logger, "servo 20 %.1f > %.1f: bola DEKAT, langsung penyelarasan", s20, cfg_.tilt_limit_deg);
      enterState(now, State::BallAlignment, "jarak sudah cukup");
      return;
    }
    if (flags_.distance_correction_attempted)
    {
      RCLCPP_WARN(logger, "servo 20 %.1f <= %.1f masih JAUH, tapi koreksi jarak sudah dipakai percobaan ini: "
                          "percobaan diulang dari INIT", s20, cfg_.tilt_limit_deg);
      enterInit(now, State::StandSearch, cfg_.post_action_search_sec, true, "koreksi jarak habis");
      return;
    }
    if (!beginWalking(now, PresetRole::Approach))
      return;
    flags_.distance_correction_attempted = true;
    RCLCPP_WARN(logger, "servo 20 %.1f <= %.1f: masih JAUH, maju lagi SEKALI dengan \"%s\"", s20,
                cfg_.tilt_limit_deg, preset(PresetRole::Approach).name.c_str());
    setStep(now, 1);
    return;
  }
  case 1:
  {
    if (!walkingStartAccepted(now, "DISTANCE"))
    {
      stopWalking(now, "start ditolak");
      setStep(now, 2);
      return;
    }
    steerTowardBall(now);
    if (!trackValid(now))
    {
      if (now - ball_stamp_ > cfg_.ball_lost_sec)
      {
        stopWalking(now, "bola hilang saat maju lagi");
        enterLostRecovery(now, "maju lagi");
      }
      return;
    }
    if (servo20() > cfg_.tilt_limit_deg)
    {
      stopWalking(now, format("servo 20 %.1f > %.1f: DEKAT", servo20(), cfg_.tilt_limit_deg));
      setStep(now, 2);
      return;
    }
    if (now - step_since_ > cfg_.approach_timeout_sec)
    {
      stopWalking(now, format("batas maju lagi %.0f s", cfg_.approach_timeout_sec));
      setStep(now, 2);
    }
    return;
  }
  case 2:
    if (!bodyStopped(now))
      return;
    if (!happened(motion_end_) || motion_end_ < walking_stop_at_)
      motion_end_ = now;
    if (!transitionElapsed(now))
      return;
    flags_.distance_stage_completed = true;
    enterState(now, State::BallAlignment, "koreksi jarak selesai; transisi 0,3 s lewat");
    return;
  default:
    return;
  }
}

// ---- BALL ALIGNMENT (persen servo 19, HANYA dari TRACK) -----------------------------------

void StrikerDemo::tickAlignment(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  if (!canStartMotion(now))
    return;
  if (!trackValid(now))
  {
    if (now - state_since_ > cfg_.ball_lost_sec)
      enterLostRecovery(now, "penyelarasan horizontal");
    return;
  }
  const double p = panPercent();
  if (!std::isfinite(p))
    return;

  // 1) Zona LUAR: bola ditarik dulu dengan geser berlawanan, SEKALI per
  //    percobaan (outer_shift_attempted), lalu tabel di bawah dipakai lagi.
  if (!flags_.outer_shift_attempted && (p < cfg_.pan_outer_left || p > cfg_.pan_outer_right))
  {
    // Koreksi operator 17 Sep (arah preset ternyata terbalik): < outer_left ->
    // KIRI MIN NEW, > outer_right -> KANAN MIN NEW.
    flags_.shift_outer = true;
    flags_.shift_role = (p < cfg_.pan_outer_left) ? PresetRole::ShiftLeft : PresetRole::ShiftRight;
    flags_.foot = (p < cfg_.pan_center) ? Foot::Left : Foot::Right;
    enterState(now, State::Shift,
               (p < cfg_.pan_outer_left)
                   ? format("servo 19 %.1f%% < %.0f%%: tarik dengan \"%s\" SEKALI sampai >= %.0f%%, kepala tetap TRACK",
                            p, cfg_.pan_outer_left, preset(PresetRole::ShiftLeft).name.c_str(), cfg_.pan_outer_left_stop)
                   : format("servo 19 %.1f%% > %.0f%%: tarik dengan \"%s\" SEKALI sampai <= %.0f%%, kepala tetap TRACK",
                            p, cfg_.pan_outer_right, preset(PresetRole::ShiftRight).name.c_str(), cfg_.pan_outer_right_stop));
    return;
  }

  // 2) Tabel utama.
  // Sesudah PUTAR KANAN NEW lalu LANGSUNG tendang (servo 19 dan 20 sudah
  // memenuhi): KIRI MIN NEW sebentar dulu. Kalau syarat belum terpenuhi,
  // tabel di bawah berjalan seperti biasa.
  if (cfg_.prekick_nudge_sec > 0.0 && flags_.last_motion_turn_right && !flags_.prekick_nudge_done &&
      (p < cfg_.pan_kick_left || p > cfg_.pan_kick_right))
  {
    const Foot foot = (p < cfg_.pan_kick_left) ? Foot::Left : Foot::Right;
    if (kickZoneSatisfied(now, foot))
    {
      flags_.foot = foot;
      flags_.prekick_nudge_done = true;
      enterState(now, State::PreKickNudge,
                 format("sesudah PUTAR KANAN NEW langsung tendang %s (servo 19 %.1f%%, servo 20 %.1f): \"%s\" %.2f s "
                        "dulu", striker_demo::toString(foot), p, servo20(), preset(PresetRole::ShiftLeft).name.c_str(),
                        cfg_.prekick_nudge_sec));
      return;
    }
  }
  if (p < cfg_.pan_kick_left)
  {
    flags_.foot = Foot::Left;
    enterState(now, State::Kick, format("servo 19 %.1f%% < %.0f%%: TENDANG KIRI (halaman %d)", p, cfg_.pan_kick_left,
                                        cfg_.page_kick_left));
    return;
  }
  if (p > cfg_.pan_kick_right)
  {
    flags_.foot = Foot::Right;
    enterState(now, State::Kick, format("servo 19 %.1f%% > %.0f%%: TENDANG KANAN (halaman %d)", p,
                                        cfg_.pan_kick_right, cfg_.page_kick_right));
    return;
  }
  if (flags_.shift_attempted)
  {
    RCLCPP_WARN(logger, "servo 19 %.1f%% masih di zona %.0f..%.0f%% sesudah geser SEKALI: percobaan diulang dari INIT",
                p, cfg_.pan_kick_left, cfg_.pan_kick_right);
    enterInit(now, State::StandSearch, cfg_.post_action_search_sec, true, "geser habis");
    return;
  }
  // Koreksi operator 17 Sep: kiri <= p < tengah -> KANAN MIN NEW (tendang kiri
  // sesudah p < kiri), tengah <= p <= kanan -> KIRI MIN NEW (tendang kanan
  // sesudah p > kanan). Kaki penendang dan syarat berhenti TIDAK berubah.
  flags_.shift_outer = false;
  flags_.foot = (p < cfg_.pan_center) ? Foot::Left : Foot::Right;
  flags_.shift_role = (flags_.foot == Foot::Left) ? PresetRole::ShiftRight : PresetRole::ShiftLeft;
  enterState(now, State::Shift, format("servo 19 %.1f%% di zona %s: \"%s\" SEKALI, kepala tetap TRACK", p,
                                       flags_.foot == Foot::Left ? "kiri" : "kanan",
                                       preset(flags_.shift_role).name.c_str()));
}

// ---- SHIFT ("KIRI MIN NEW" / "KANAN MIN NEW", sekali) --------------------------------------

void StrikerDemo::tickShift(double now)
{
  const PresetRole role = flags_.shift_role;
  switch (step_)
  {
  case 0:
    if (!canStartMotion(now))
      return;
    if (!trackValid(now))
    {
      if (now - state_since_ > cfg_.ball_lost_sec)
        enterLostRecovery(now, "sebelum geser");
      return;
    }
    if (!beginWalking(now, role))
      return;
    if (flags_.shift_outer)
      flags_.outer_shift_attempted = true;
    else
      flags_.shift_attempted = true;
    setStep(now, 1);
    return;
  case 1:
  {
    if (!walkingStartAccepted(now, "SHIFT"))
    {
      stopWalking(now, "start ditolak");
      setStep(now, 2);
      return;
    }
    // §26: bola hilang saat geser -> recovery, data lama tidak dipakai.
    if (!ballValid(now) && now - ball_stamp_ > cfg_.ball_lost_sec)
    {
      stopWalking(now, "bola hilang saat geser");
      enterLostRecovery(now, "geser");
      return;
    }
    const double p = trackValid(now) ? panPercent() : kNaN;
    // Geser tarik: selesai saat persen mencapai *_stop. Geser zona tengah:
    // selesai saat persen lewat batas zona. Kalau tidak, sampai batas waktu.
    // Zona tengah: batas yang PERNAH tersentuh (jepitan dari callback servo
    // 125 Hz, presentJointsCallback) sudah cukup -- tidak perlu bertahan.
    const bool reached = flags_.shift_target_touched || (std::isfinite(p) &&
        (flags_.shift_outer
             ? ((flags_.shift_role == PresetRole::ShiftLeft && p >= cfg_.pan_outer_left_stop) ||
                (flags_.shift_role == PresetRole::ShiftRight && p <= cfg_.pan_outer_right_stop))
             : ((flags_.foot == Foot::Left && p < cfg_.pan_kick_left) ||
                (flags_.foot == Foot::Right && p > cfg_.pan_kick_right))));
    if (reached && !flags_.shift_outer)
      flags_.shift_target_touched = true;
    if (reached)
    {
      stopWalking(now, flags_.shift_outer ? format("servo 19 %.1f%% sudah mencapai batas tarik", p)
                                          : format("servo 19 %.1f%% sudah di zona tendang %s", p,
                                                   striker_demo::toString(flags_.foot)));
      setStep(now, 2);
      return;
    }
    if (now - step_since_ > cfg_.shift_timeout_sec)
    {
      stopWalking(now, format("batas geser %.0f s (servo 19 %.1f%%)", cfg_.shift_timeout_sec, p));
      setStep(now, 2);
    }
    return;
  }
  case 2:
    if (!bodyStopped(now))
      return;
    if (!happened(motion_end_) || motion_end_ < walking_stop_at_)
      motion_end_ = now;
    if (!transitionElapsed(now))
      return;
    if (flags_.shift_target_touched)
    {
      // Batas sudah tersentuh saat geser: LANGSUNG tahap tendang, persen tidak
      // dinilai ulang (kalau dinilai ulang, gait yang reda bisa memantulkannya
      // kembali ke zona tengah dan percobaan diulang sia-sia).
      enterState(now, State::Kick, format("geser %s selesai, batas %.0f%% sudah tersentuh: TENDANG %s (halaman %d)",
                                          preset(flags_.shift_role).name.c_str(),
                                          flags_.foot == Foot::Left ? cfg_.pan_kick_left : cfg_.pan_kick_right,
                                          striker_demo::toString(flags_.foot),
                                          flags_.foot == Foot::Left ? cfg_.page_kick_left : cfg_.page_kick_right));
      return;
    }
    // Geser tarik / batas waktu: mapping servo 19 TERBARU dibaca lagi di BallAlignment.
    enterState(now, State::BallAlignment, "geser selesai; baca servo 19 terbaru");
    return;
  default:
    return;
  }
}

// ---- PRE KICK NUDGE ("KIRI MIN NEW" sebentar sesudah PUTAR KANAN NEW) ---------------------

bool StrikerDemo::kickZoneSatisfied(double now, Foot foot) const
{
  if (!trackValid(now))
    return false;
  const double p = panPercent();
  const bool pan_ok = std::isfinite(p) && ((foot == Foot::Left && p < cfg_.pan_kick_left) ||
                                           (foot == Foot::Right && p > cfg_.pan_kick_right));
  return pan_ok && servo20() > cfg_.tilt_limit_deg;
}

void StrikerDemo::tickPreKickNudge(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  switch (step_)
  {
  case 0:
    if (!canStartMotion(now))
      return;
    if (!trackValid(now))
    {
      if (now - state_since_ > cfg_.ball_lost_sec)
        enterLostRecovery(now, "sebelum geser singkat pra-tendang");
      return;
    }
    if (!beginWalking(now, PresetRole::ShiftLeft))
      return;
    setStep(now, 1);
    return;
  case 1:
    if (!walkingStartAccepted(now, "PRE_KICK_NUDGE"))
    {
      stopWalking(now, "start ditolak");
      setStep(now, 2);
      return;
    }
    if (now - walking_start_sent_at_ >= cfg_.prekick_nudge_sec)
    {
      stopWalking(now, format("geser singkat pra-tendang %.2f s selesai", cfg_.prekick_nudge_sec));
      setStep(now, 2);
    }
    return;
  case 2:
    if (!bodyStopped(now))
      return;
    if (!happened(motion_end_) || motion_end_ < walking_stop_at_)
      motion_end_ = now;
    if (!transitionElapsed(now))
      return;
    if (kickZoneSatisfied(now, flags_.foot))
    {
      enterState(now, State::Kick, format("geser singkat selesai, servo 19 %.1f%% / servo 20 %.1f masih memenuhi: "
                                          "TENDANG %s", panPercent(), servo20(), striker_demo::toString(flags_.foot)));
      return;
    }
    // Syarat servo 19/20 tidak lagi terpenuhi: logika semestinya.
    RCLCPP_WARN(logger, "sesudah geser singkat syarat tendang %s tidak terpenuhi (servo 19 %.1f%%, servo 20 %.1f): "
                        "kembali ke logika penyelarasan", striker_demo::toString(flags_.foot), panPercent(), servo20());
    if (!ballValid(now))
    {
      enterLostRecovery(now, "sesudah geser singkat pra-tendang");
      return;
    }
    if (servo20() <= cfg_.tilt_limit_deg)
      enterState(now, State::DistanceRecheck, "servo 20 belum dekat sesudah geser singkat");
    else
      enterState(now, State::BallAlignment, "servo 19 berubah sesudah geser singkat");
    return;
  default:
    return;
  }
}

// ---- KICK (halaman 104 / 105 lewat action_module) --------------------------------------------

void StrikerDemo::tickKick(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  switch (step_)
  {
  case 0:
  {
    if (!canStartMotion(now))
      return;
    const std::string denial = kickDenialReason(now);
    if (!denial.empty())
    {
      if (now - state_since_ > cfg_.ball_lost_sec)
      {
        RCLCPP_WARN(logger, "tendangan DITOLAK: %s", denial.c_str());
        if (!ballValid(now))
          enterLostRecovery(now, "sebelum menendang");
        else
          enterState(now, State::BallAlignment, "syarat tendang belum lengkap: " + denial);
      }
      return;
    }
    // INIT KEPALA sebelum halaman: pelacak "stop" -> head_tracking_node memarkir
    // kepala ke pan 0 deg / servo 20 200 lewat head_control_module (yang masih
    // memegang kepala). Sendi baru diserahkan ke action_module sesudah kepala
    // sampai, supaya perintah halaman, sapuan, dan tracking tidak saling timpa.
    stopTracker();
    head_park_sent_at_ = now;
    RCLCPP_WARN(logger, "izin tendang LULUS (kaki %s): kepala diparkir dulu ke pan %+.0f / servo 20 %.0f",
                striker_demo::toString(flags_.foot), cfg_.head_park_pan_deg, cfg_.head_park_tilt_servo_deg);
    setStep(now, 1);
    return;
  }
  case 1:
  {
    const std::string denial = kickDenialReason(now, true);
    if (!denial.empty())
    {
      RCLCPP_WARN(logger, "izin tendang DIBATALKAN saat kepala diparkir: %s", denial.c_str());
      enterInit(now, State::StandSearch, cfg_.post_action_search_sec, false, "tendangan batal");
      return;
    }
    if (!headParked(now) && now - head_park_sent_at_ < cfg_.head_park_wait_sec)
      return;
    RCLCPP_INFO(logger, "kepala %s (pan %+.1f, servo 20 %.1f) %.2f s sesudah parkir; sendi -> action_module",
                headParked(now) ? "sudah di parkir" : "BELUM sampai parkir, batas waktu", rad2deg(pan_present_),
                servo20(), now - head_park_sent_at_);
    publishJointModules(kAllJoints, "action_module");
    switch_sent_at_ = now;
    setStep(now, 2);
    return;
  }
  case 2:
  {
    const std::string denial = kickDenialReason(now, true);
    if (!denial.empty())
    {
      RCLCPP_WARN(logger, "izin tendang DIBATALKAN sebelum halaman dikirim: %s", denial.c_str());
      enterInit(now, State::StandSearch, cfg_.post_action_search_sec, false, "tendangan batal");
      return;
    }
    if (!allOwnedBy("action_module"))
    {
      if (now - switch_sent_at_ > cfg_.module_switch_timeout_sec)
      {
        RCLCPP_ERROR(logger, "action_module tidak memegang sendi dalam %.1f s (kaki '%s', kepala '%s'); tendangan batal",
                     cfg_.module_switch_timeout_sec, legs_owner_.c_str(), head_owner_.c_str());
        stopTracker();
        enterInit(now, State::StandSearch, cfg_.post_action_search_sec, false, "action_module tidak siap");
      }
      return;
    }
    const int page = (flags_.foot == Foot::Left) ? cfg_.page_kick_left : cfg_.page_kick_right;
    sendPage(now, page);
    flags_.kick_executed = true;
    RCLCPP_WARN(logger, "TENDANG %s: halaman %d dikirim SEKALI (timing asli Action Editor) | %s",
                striker_demo::toString(flags_.foot), page, snapshot(now).c_str());
    setStep(now, 3);
    return;
  }
  case 3:
    if (our_action_failed_)
    {
      RCLCPP_ERROR(logger, "halaman tendang %d DITOLAK action_module", our_action_page_);
      our_action_pending_ = false;
      enterInit(now, State::StandSearch, cfg_.post_action_search_sec, true, "halaman tendang ditolak");
      return;
    }
    if (our_action_started_)
    {
      setStep(now, 4);
      return;
    }
    if (now - our_action_sent_at_ > cfg_.action_start_timeout_sec)
    {
      RCLCPP_ERROR(logger, "halaman tendang %d tidak terkonfirmasi mulai dalam %.1f s", our_action_page_,
                   cfg_.action_start_timeout_sec);
      our_action_pending_ = false;
      enterInit(now, State::StandSearch, cfg_.post_action_search_sec, true, "halaman tendang tidak mulai");
    }
    return;
  case 4:
    if (our_action_pending_)
      return;   // menunggu Action Page selesai, durasi mengikuti .bin
    motion_end_ = now;
    RCLCPP_WARN(logger, "halaman tendang %d SELESAI -> INIT \"%s\", cari %.1f s", our_action_page_,
                preset(PresetRole::Init).name.c_str(), cfg_.post_action_search_sec);
    // STRIKER (Titik 1): bola target berikutnya (B -> A -> C -> D). Gerbang
    // dikosongkan supaya bola yang baru ditendang dinilai ulang (cooldown).
    {
      const int ditendang = pengarah_.indeks();
      pengarah_.laporSelesaiMenendang(now);
      gerbang_.reset();
      post_kick_look_pending_ = true;
      setAimTargetFromTable("bola berikutnya");
      RCLCPP_WARN(logger, "STRIKER: bola %s selesai ditendang -> target berikutnya bola %s, arah putar search %s",
                  striker::namaBola(ditendang), striker::namaBola(pengarah_.indeks()),
                  striker::toString(pengarah_.arahPutar(yawUsable(now) ? yaw_deg_ : kNaN)));
    }
    // §29: kembali ke INIT, diam mencari 1,5 s; bola ketemu -> percobaan baru.
    enterInit(now, State::StandSearch, cfg_.post_action_search_sec, true, "sesudah tendangan");
    return;
  default:
    return;
  }
}

// ---- GETTING UP (halaman 85 / 86) --------------------------------------------------------------

void StrikerDemo::tickGetUp(double now)
{
  const auto logger = rclcpp::get_logger(kLogger);
  switch (step_)
  {
  case 0:
  {
    if (!happened(switch_sent_at_))
    {
      if (!headParked(now) && now - head_park_sent_at_ < cfg_.head_park_wait_sec)
        return;
      publishJointModules(kAllJoints, "action_module");
      switch_sent_at_ = now;
      return;
    }
    if (!allOwnedBy("action_module"))
    {
      if (now - switch_sent_at_ > cfg_.module_switch_timeout_sec)
      {
        RCLCPP_ERROR(logger, "action_module tidak memegang sendi untuk bangun; dikirim ulang");
        publishJointModules(kAllJoints, "action_module");
        switch_sent_at_ = now;
      }
      return;
    }
    if (actionRunning(now) || now - imu_stamp_ > cfg_.yaw_max_age_sec)
      return;
    if (fall_ == PitchFallState::Stand)
    {
      RCLCPP_WARN(logger, "pitch kembali normal sebelum halaman bangun dikirim");
      motion_end_ = now;
      setStep(now, 3);
      return;
    }
    const int page = (fall_ == PitchFallState::FallenForward) ? cfg_.page_getup_front : cfg_.page_getup_back;
    sendPage(now, page);
    RCLCPP_WARN(logger, "BANGUN %s: halaman %d | %s", fallName(fall_), page, snapshot(now).c_str());
    setStep(now, 1);
    return;
  }
  case 1:
    if (our_action_failed_)
    {
      RCLCPP_ERROR(logger, "halaman bangun %d DITOLAK action_module; dicoba lagi", our_action_page_);
      our_action_pending_ = false;
      setStep(now, 0);
      return;
    }
    if (our_action_started_)
    {
      setStep(now, 2);
      return;
    }
    if (now - our_action_sent_at_ > cfg_.action_start_timeout_sec)
    {
      RCLCPP_ERROR(logger, "halaman bangun %d tidak terkonfirmasi mulai; dicoba lagi", our_action_page_);
      our_action_pending_ = false;
      setStep(now, 0);
    }
    return;
  case 2:
    if (our_action_pending_)
      return;
    motion_end_ = now;
    RCLCPP_WARN(logger, "halaman bangun %d SELESAI", our_action_page_);
    setStep(now, 3);
    return;
  case 3:
    if (!transitionElapsed(now) || now - imu_stamp_ > cfg_.yaw_max_age_sec)
      return;
    if (fall_ != PitchFallState::Stand)
    {
      RCLCPP_WARN(logger, "sesudah bangun robot MASIH %s (pitch EMA %.1f); bangun lagi", fallName(fall_), filtered_pitch_);
      setStep(now, 0);
      return;
    }
    // §34: INIT, diam mencari 1,5 s, penanda percobaan direset.
    enterInit(now, State::StandSearch, cfg_.post_action_search_sec, true, "robot berdiri kembali");
    return;
  default:
    return;
  }
}

// ---- PARK (kembali ke pose INIT lalu STOPPED) ----------------------------------------------------

void StrikerDemo::tickPark(double now)
{
  switch (step_)
  {
  case 0:
    if (actionRunning(now) || now - leg_motion_stamp_ > cfg_.leg_motion_fresh_sec)
    {
      if (now - state_since_ > cfg_.stop_confirm_timeout_sec)
        finishStop(now, "pose INIT tidak bisa diterapkan (action masih jalan / controller diam)");
      return;
    }
    if (legsOwnedBy("walking_module"))
    {
      setStep(now, 2);
      return;
    }
    if (!preset(PresetRole::PoseReference).available)
    {
      finishStop(now, "preset INIT tidak tersedia, pose tidak diterapkan");
      return;
    }
    {
      const int landed = presetLanded(now, PresetRole::PoseReference);
      if (landed < 0)
      {
        finishStop(now, "acuan pose tidak terbukti hidup");
        return;
      }
      if (landed == 0)
        return;
    }
    clearPresetRequest();
    publishJointModules(kWalkingJoints, "walking_module");
    switch_sent_at_ = now;
    setStep(now, 1);
    return;
  case 1:
    if (legsOwnedBy("walking_module"))
    {
      setStep(now, 2);
      return;
    }
    if (now - switch_sent_at_ > cfg_.module_switch_timeout_sec)
      finishStop(now, "walking_module tidak memegang kaki untuk pose INIT");
    return;
  case 2:
  {
    const int landed = presetLanded(now, PresetRole::Init);
    if (landed < 0)
    {
      finishStop(now, "preset INIT tidak terbukti hidup");
      return;
    }
    if (landed == 0)
      return;
    clearPresetRequest();
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "pose INIT \"%s\" (acuan page 2) diterapkan",
                preset(PresetRole::Init).name.c_str());
    // Kepala: kalau masih dipegang modul lain (mis. action_module sesudah page
    // selesai), serahkan ke head_control_module lalu parkir ke servo 20 135.
    if (head_owner_ != "head_control_module")
    {
      publishJointModules({"head_pan", "head_tilt"}, "head_control_module");
      switch_sent_at_ = now;
    }
    setStep(now, 3);
    return;
  }
  case 3:
    // Tunggu kepala dipegang head_control_module (dan goal-nya terisi satu siklus).
    if (head_owner_ != "head_control_module")
    {
      if (now - switch_sent_at_ > cfg_.module_switch_timeout_sec)
        finishStop(now, format("kepala tidak berpindah ke head_control_module (pemilik '%s'); tidak diparkir %.0f",
                               head_owner_.c_str(), cfg_.stop_head_tilt_servo_deg));
      return;
    }
    if (now - step_since_ < 0.15)
      return;
    {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = node_->now();
      msg.name = {"head_pan", "head_tilt"};
      msg.position = {deg2rad(cfg_.head_park_pan_deg), deg2rad(cfg_.stop_head_tilt_servo_deg - cfg_.servo_center_deg)};
      head_abs_pub_->publish(msg);
    }
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "kepala diparkir ke pan %+.0f, servo 20 %.0f; D60 idle",
                cfg_.head_park_pan_deg, cfg_.stop_head_tilt_servo_deg);
    finishStop(now, "");
    return;
  default:
    return;
  }
}

// ============================================================================
//  Callback sensor
// ============================================================================

void StrikerDemo::circleSetCallback(const op3_ball_detector_msgs::msg::CircleSetStamped::SharedPtr msg)
{
  // Sama dengan BallTracker D56: lingkaran ber-z (jari-jari) terbesar.
  int best = -1;
  for (size_t i = 0; i < msg->circles.size(); ++i)
    if (best < 0 || msg->circles[i].z > msg->circles[static_cast<size_t>(best)].z)
      best = static_cast<int>(i);
  const bool ada = !(best < 0 || msg->circles[static_cast<size_t>(best)].z <= 0.0);

  // STRIKER (Titik 1): gerbang terima/tolak bola yang dikunci detektor, dalam
  // satuan servo robot. Posisi bola = sudut kepala + letaknya di gambar (FOV
  // head_tracking.yaml), lalu dikonversi persis seperti D60 (servo 19 %, servo 20).
  const double now = steadyNow();
  double pan_pct = kNaN;
  double tilt_servo = kNaN;
  if (ada && std::isfinite(pan_present_) && std::isfinite(tilt_present_))
  {
    const auto &c = msg->circles[static_cast<size_t>(best)];
    const double pan_rad = pan_present_ - std::atan(c.x * std::tan(deg2rad(cfg_.striker.strategy.half_hfov_deg)));
    const double tilt_rad = tilt_present_ + std::atan(c.y * std::tan(deg2rad(cfg_.striker.half_vfov_deg)));
    pan_pct = panPercentOf(jointToServoDeg(pan_rad, cfg_.servo_center_deg, cfg_.servo_dir),
                           cfg_.pan_left_limit_servo_deg, cfg_.pan_right_limit_servo_deg);
    tilt_servo = jointToServoDeg(tilt_rad, cfg_.servo_center_deg, cfg_.servo_dir);
  }
  const bool bisa_dinilai = mode_ == Mode::Running && tracker_text_ == "TRACKING" && trackerFresh(now) &&
                            presentFresh(now);
  const double yaw_for_gate = yawUsable(now) ? yaw_deg_ : kNaN;
  const bool dipakai = gerbang_.update(ada, bisa_dinilai, pan_pct, tilt_servo, yaw_for_gate, now,
                                       !flags_.approach_completed, cfg_.striker.gate_release_sec, pengarah_);
  if (gerbang_.berubah() && mode_ == Mode::Running && gerbang_.status() != striker::StatusGerbang::BelumDinilai)
    RCLCPP_WARN(rclcpp::get_logger(kLogger),
                "STRIKER gerbang: bola terkunci %s (%s) -- target bola %s sektor %.0f%%, servo 19 %.1f%%, servo 20 %.1f, "
                "yaw %+.1f%s", striker::toString(gerbang_.status()), striker::toString(gerbang_.putusanTerakhir()),
                striker::namaBola(pengarah_.indeks()), pengarah_.sektorPct(), pan_pct, tilt_servo, yaw_for_gate,
                gerbang_.status() == striker::StatusGerbang::Ditolak ? "; dianggap TIDAK terlihat, lanjut mencari" : "");
  if (!dipakai)
  {
    ball_seen_ = false;
    return;
  }
  const auto &circle = msg->circles[static_cast<size_t>(best)];
  ball_seen_ = true;
  ball_stamp_ = now;
  ball_x_ = circle.x;
  ball_y_ = circle.y;
  ball_radius_ = circle.z;
}

void StrikerDemo::trackerStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  tracker_text_ = msg->data;
  tracker_stamp_ = steadyNow();
}

void StrikerDemo::presentJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // POSISI AKTUAL servo 19/20 (bulk read), bukan goal.
  const double pan = jointPosition(*msg, "head_pan");
  const double tilt = jointPosition(*msg, "head_tilt");
  if (!std::isfinite(pan) || !std::isfinite(tilt))
    return;
  pan_present_ = pan;
  tilt_present_ = tilt;
  present_stamp_ = steadyNow();
  // Jepitan batas geser zona tengah pada laju servo penuh: satu bacaan yang
  // menyentuh batas saat kepala TRACK sudah cukup (lihat tickShift).
  if (mode_ == Mode::Running && state_ == State::Shift && step_ == 1 && !flags_.shift_outer &&
      !flags_.shift_target_touched && ballValid(present_stamp_) && headMode(present_stamp_) == HeadMode::Track)
  {
    const double p = panPercent();
    if ((flags_.foot == Foot::Left && p < cfg_.pan_kick_left) ||
        (flags_.foot == Foot::Right && p > cfg_.pan_kick_right))
      flags_.shift_target_touched = true;
  }
}

void StrikerDemo::goalJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const double now = steadyNow();
  LegSample sample;
  sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
  if (sample.stamp <= 0.0)
    sample.stamp = now;
  for (size_t i = 0; i < kLegJoints.size(); ++i)
  {
    sample.position[i] = jointPosition(*msg, kLegJoints[i]);
    if (!std::isfinite(sample.position[i]))
      return;
  }
  leg_samples_.push_back(sample);
  while (leg_samples_.size() > 2 && sample.stamp - leg_samples_.front().stamp > 0.2)
    leg_samples_.pop_front();

  const LegSample *reference = nullptr;
  for (auto it = leg_samples_.rbegin(); it != leg_samples_.rend(); ++it)
  {
    if (sample.stamp - it->stamp >= 0.04)
    {
      reference = &(*it);
      break;
    }
  }
  if (reference == nullptr)
    return;
  const double dt = sample.stamp - reference->stamp;
  double speed = 0.0;
  for (size_t i = 0; i < kLegJoints.size(); ++i)
    speed = std::max(speed, std::fabs(rad2deg(sample.position[i] - reference->position[i])) / dt);
  leg_motion_stamp_ = now;
  if (speed > cfg_.idle_leg_speed_deg_s)
    leg_fast_last_ = now;
  if (speed > cfg_.leg_stationary_speed_deg_s)
    leg_moving_last_ = now;
}

void StrikerDemo::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Konversi yang sama dengan SoccerDemo::imuDataCallback().
  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy = robotis_framework::convertQuaternionToRPY(orientation);
  rpy *= (180.0 / M_PI);
  roll_deg_ = rpy.coeff(0, 0);
  pitch_deg_ = rpy.coeff(1, 0);
  // STRIKER: yaw = integrasi gyro z, BUKAN yaw quaternion (itu magnetometer,
  // dilarang regulasi KSTI 2026). Bias dikumpulkan saat badan diam.
  const double now = steadyNow();
  double stamp = rclcpp::Time(msg->header.stamp).seconds();
  if (stamp <= 0.0)
    stamp = now;
  const bool diam = bodyStopped(now) && now - leg_moving_last_ > cfg_.leg_quiet_window_sec &&
                    fall_ == PitchFallState::Stand;
  gyro_.update(msg->angular_velocity.z, stamp, diam, cfg_.striker.gyro_z_sign, cfg_.striker.gyro_bias_tau_sec,
               cfg_.striker.gyro_max_dt_sec);
  yaw_deg_ = gyro_.yawDeg();
  imu_stamp_ = now;
  // Rumus dan ambang jatuh D56 (fall_detector.h).
  if (mode_ == Mode::Running)
    fall_ = updatePitchFallState(filtered_pitch_, pitch_deg_);
  // Jepitan ambang yaw pada laju IMU penuh (lihat tickYaw).
  if (mode_ == Mode::Running && state_ == State::YawCorrection && yawUsable(imu_stamp_))
  {
    const double err = yawErrorDeg();
    if (step_ == 0 && yawInTolerance(err))
      yaw_tolerance_seen_ = true;
    if (step_ == 1 && yawInStopBand(err))
      yaw_stop_reached_ = true;
  }
}

// STRIKER: deteksi kiper / tiang gawang hanya dipakai selama jendela kumpul
// AIM_LOOK, dan hanya dari frame yang ditangkap SESUDAH kepala sampai di pose
// lirik (header.stamp kamera >= saat detektor dinyalakan).
void StrikerDemo::keeperDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (!aim_collecting_ || rclcpp::Time(msg->header.stamp, RCL_ROS_TIME) < aim_collect_ros_start_)
    return;
  kumpul_.tambahKiper(toDeteksi(*msg), cfg_.striker.strategy);
}

void StrikerDemo::goalpostDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  if (!aim_collecting_ || rclcpp::Time(msg->header.stamp, RCL_ROS_TIME) < aim_collect_ros_start_)
    return;
  kumpul_.tambahTiang(toDeteksi(*msg), cfg_.striker.strategy);
}

void StrikerDemo::jointModulesCallback(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg)
{
  const size_t count = std::min(msg->joint_name.size(), msg->module_name.size());
  const auto common = [&](const std::vector<std::string> &wanted) -> std::string {
    std::string module;
    for (const std::string &joint : wanted)
    {
      const auto it = std::find(msg->joint_name.begin(), msg->joint_name.begin() + static_cast<long>(count), joint);
      if (it == msg->joint_name.begin() + static_cast<long>(count))
        return "tidak-diketahui";
      const std::string &owner = msg->module_name[static_cast<size_t>(it - msg->joint_name.begin())];
      if (module.empty())
        module = owner;
      else if (module != owner)
        return "campuran";
    }
    return module;
  };
  legs_owner_ = common(kLegJoints);
  head_owner_ = common({"head_pan", "head_tilt"});
}

void StrikerDemo::movementDoneCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "action")
  {
    if (our_action_pending_ && our_action_started_)
      our_action_pending_ = false;
  }
  else if (msg->data == "action_failed")
  {
    if (our_action_pending_)
      our_action_failed_ = true;
  }
}

void StrikerDemo::statusCallback(const robotis_controller_msgs::msg::StatusMsg::SharedPtr msg)
{
  if (msg->module_name == "Walking")
  {
    if (msg->status_msg == "Start walking")
      walking_status_started_at_ = steadyNow();
    if (isBusy())
      RCLCPP_INFO(rclcpp::get_logger(kLogger), "walking_module menjawab: %s", msg->status_msg.c_str());
    return;
  }
  if (msg->module_name != "Action")
    return;
  int page = 0;
  if (std::sscanf(msg->status_msg.c_str(), "Succeed to start page %d", &page) == 1)
  {
    if (our_action_pending_ && page == our_action_page_)
    {
      our_action_started_ = true;
      our_action_started_at_ = steadyNow();
    }
  }
  else if (std::sscanf(msg->status_msg.c_str(), "Failed to start page %d", &page) == 1)
  {
    if (our_action_pending_ && page == our_action_page_)
      our_action_failed_ = true;
  }
  else if (msg->status_msg == "Action Module is not enabled")
  {
    if (our_action_pending_)
      our_action_failed_ = true;
  }
}

}  // namespace robotis_op

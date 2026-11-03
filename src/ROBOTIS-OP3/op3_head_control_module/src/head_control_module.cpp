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

#include <stdio.h>
#include "op3_head_control_module/head_control_module.h"

namespace robotis_op
{

HeadControlModule::HeadControlModule()
  : Node("op3_head_control_module"),
    control_cycle_msec_(0),
    stop_process_(false),
    is_moving_(false),
    is_direct_control_(true),
    tra_count_(0),
    tra_size_(0),
    moving_time_(3.0),
    scan_state_(NoScan),
    has_goal_position_(false),
    angle_unit_(35),
    tilt_envelope_enable_(true),
    tilt_envelope_center_rad_(27.0 * DEGREE2RADIAN),
    tilt_envelope_edge_rad_(0.0),
    tilt_envelope_flat_percent_(30.0),
    DEBUG(false)
{
  enable_ = false;
  module_name_ = "head_control_module";
  control_mode_ = robotis_framework::PositionControl;

  result_["head_pan"] = new robotis_framework::DynamixelState();
  result_["head_tilt"] = new robotis_framework::DynamixelState();

  using_joint_name_["head_pan"] = 0;
  using_joint_name_["head_tilt"] = 1;

  max_angle_[using_joint_name_["head_pan"]] = 85 * DEGREE2RADIAN;
  min_angle_[using_joint_name_["head_pan"]] = -85 * DEGREE2RADIAN;
  // Nilai awal saja; initialize() menimpanya dari parameter ROS.
  //
  // Angka bawaan ROBOTIS (max +30 / min -75) dibuat untuk konvensi di mana
  // head_tilt NEGATIF berarti menunduk, jadi di sana 75 derajat adalah jatah
  // MENUNDUK. Robot ini TERBALIK -- positif yang menunduk (diukur dengan dua
  // foto kamera pada dua sudut perintah, lihat catatan di ball_follower.cpp).
  // Karena itu jatahnya ditukar supaya yang 75 derajat tetap jatah MENUNDUK:
  // max +75 (menunduk), min -30 (mendongak).
  max_angle_[using_joint_name_["head_tilt"]] = 75 * DEGREE2RADIAN;
  min_angle_[using_joint_name_["head_tilt"]] = -30 * DEGREE2RADIAN;

  target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  current_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  last_msg_time_ = rclcpp::Clock().now();
}

HeadControlModule::~HeadControlModule()
{
  queue_thread_.join();
}

void HeadControlModule::joinQueueThread()
{
  if (queue_thread_.joinable())
    queue_thread_.join();
}

void HeadControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  this->declare_parameter("angle_unit", 35.0);
  angle_unit_ = this->get_parameter("angle_unit").as_double();

  // Batas sendi kepala, derajat. Bisa disetel supaya jangkauan menunduk bisa
  // dinaikkan bertahap sambil diamati, tanpa compile ulang. Lihat catatan
  // panjang di konstruktor soal kenapa tandanya ditukar untuk robot ini.
  this->declare_parameter("head_tilt_max_deg", 75.0);
  this->declare_parameter("head_tilt_min_deg", -30.0);
  const double tilt_max_deg = this->get_parameter("head_tilt_max_deg").as_double();
  const double tilt_min_deg = this->get_parameter("head_tilt_min_deg").as_double();
  max_angle_[using_joint_name_["head_tilt"]] = tilt_max_deg * DEGREE2RADIAN;
  min_angle_[using_joint_name_["head_tilt"]] = tilt_min_deg * DEGREE2RADIAN;

  RCLCPP_WARN(this->get_logger(),
              "Head control - angle unit : %f, head_tilt limit : %+.1f .. %+.1f deg "
              "(POSITIF = menunduk di robot ini)",
              angle_unit_, tilt_min_deg, tilt_max_deg);

  // AMPLOP MENUNDUK MENGIKUTI PAN (permintaan operator 17 Sep 2026): di bagian
  // tengah (flat_percent .. 100 - flat_percent) kepala boleh menunduk sampai
  // center (bawaan +27 deg = servo 20 207), lalu batasnya turun linier sampai
  // edge (bawaan 0 = servo 180) di ujung kiri/kanan (0 % / 100 % = head_pan
  // +-85). Berlaku untuk SEMUA sumber perintah -- sapuan dan tracking dari
  // head_tracking_node, BallTracker D56 -- jadi ketiga demo pasti sama.
  // Catatan: zona tendang D58 hasil kalibrasi 16 Sep butuh servo 20 sampai
  // 212,98; kalau D58 masih dipakai naikkan center_deg ke >= 33.
  this->declare_parameter("head_tilt_envelope_enable", true);
  this->declare_parameter("head_tilt_envelope_center_deg", 27.0);
  this->declare_parameter("head_tilt_envelope_edge_deg", 0.0);
  this->declare_parameter("head_tilt_envelope_flat_percent", 30.0);
  tilt_envelope_enable_ = this->get_parameter("head_tilt_envelope_enable").as_bool();
  tilt_envelope_center_rad_ = this->get_parameter("head_tilt_envelope_center_deg").as_double() * DEGREE2RADIAN;
  tilt_envelope_edge_rad_ = this->get_parameter("head_tilt_envelope_edge_deg").as_double() * DEGREE2RADIAN;
  tilt_envelope_flat_percent_ =
      std::max(0.0, std::min(50.0, this->get_parameter("head_tilt_envelope_flat_percent").as_double()));
  RCLCPP_WARN(this->get_logger(),
              "Amplop menunduk mengikuti pan: %s | tengah (%.0f..%.0f %%) maks %+.1f deg (servo 20 %.1f) | "
              "ujung 0/100 %% maks %+.1f deg (servo 20 %.1f) | 0 %% = head_pan %+.0f, 100 %% = %+.0f",
              tilt_envelope_enable_ ? "AKTIF" : "mati", tilt_envelope_flat_percent_,
              100.0 - tilt_envelope_flat_percent_, tilt_envelope_center_rad_ * RADIAN2DEGREE,
              180.0 + tilt_envelope_center_rad_ * RADIAN2DEGREE, tilt_envelope_edge_rad_ * RADIAN2DEGREE,
              180.0 + tilt_envelope_edge_rad_ * RADIAN2DEGREE,
              max_angle_[using_joint_name_["head_pan"]] * RADIAN2DEGREE,
              min_angle_[using_joint_name_["head_pan"]] * RADIAN2DEGREE);

  queue_thread_ = std::thread(&HeadControlModule::queueThread, this);

  control_cycle_msec_ = control_cycle_msec;

  /* publish topics */
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 10);
}

void HeadControlModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  /* subscribe topics */
  auto set_head_joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/head_control/set_joint_states", 10, std::bind(&HeadControlModule::setHeadJointCallback, this, std::placeholders::_1));
  auto set_head_joint_offset_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/head_control/set_joint_states_offset", 10, std::bind(&HeadControlModule::setHeadJointOffsetCallback, this, std::placeholders::_1));
  auto set_head_scan_sub = this->create_subscription<std_msgs::msg::String>(
      "/robotis/head_control/scan_command", 10, std::bind(&HeadControlModule::setHeadScanCallback, this, std::placeholders::_1));

  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void HeadControlModule::setHeadJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  setHeadJoint(msg, false);
}

void HeadControlModule::setHeadJointOffsetCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  setHeadJoint(msg, true);
}

void HeadControlModule::setHeadJoint(const sensor_msgs::msg::JointState::SharedPtr msg, bool is_offset)
{
  if (enable_ == false)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Head module is not enable.");
    publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_ERROR, "Not Enable");
    return;
  }

  // Drop the command if process() hasn't yet populated goal_position_ from
  // the controller's joint feedback. Blocking-spin here was flooding the
  // terminal for ~6 s during module-enable handshake. The caller
  // (head_tracking_node) re-issues its scan target every scan_period_sec,
  // so dropping the first one costs one period, not the session.
  if (has_goal_position_ == false)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "head_control_module: goal_position not initialized yet, dropping command");
    return;
  }

  // moving time
  moving_time_ = is_offset ? 0.1 : 1.0;               // default : 1 sec

  // set target joint angle
  target_position_ = goal_position_;        // default

  for (size_t ix = 0; ix < msg->name.size(); ix++)
  {
    std::string joint_name = msg->name[ix];
    auto joint_it = using_joint_name_.find(joint_name);

    if (joint_it != using_joint_name_.end())
    {
      double target_position = 0.0;
      int joint_index = joint_it->second;

      // set target position
      if (is_offset == true)
        target_position = goal_position_.coeff(0, joint_index) + msg->position[ix];
      else
        target_position = msg->position[ix];

      // check angle limit
      bool is_checked = checkAngleLimit(joint_index, target_position);
      if(is_checked == false)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to find limit angle \n    id : %d, value : %f", joint_index, (target_position_ * 180 / M_PI).value());
      }

      // apply target position
      target_position_.coeffRef(0, joint_index) = target_position;

      // set time
      //double angle_unit = 35 * M_PI / 180;
      double angle_unit = is_offset ? (angle_unit_ * M_PI / 180 * 1.5) : (angle_unit_ * M_PI / 180);
      double calc_moving_time = fabs(goal_position_.coeff(0, joint_index) - target_position_.coeff(0, joint_index)) / angle_unit;
      if (calc_moving_time > moving_time_)
        moving_time_ = calc_moving_time;

      if (DEBUG)
        std::cout << " - joint : " << joint_name << ", Index : " << joint_index << "\n     Target Angle : " << target_position_.coeffRef(0, joint_index) << ", Curr Goal : " << goal_position_.coeff(0, joint_index)
                  << ", Time : " << moving_time_ << ", msg : " << msg->position[ix] << std::endl;
    }
  }

  // Amplop menunduk mengikuti pan: dijepit pada SASARAN, sebelum trajektori
  // dibuat, untuk perintah absolut maupun offset. Amplopnya cekung (datar di
  // tengah, turun linier ke ujung), jadi garis lurus goal -> sasaran yang
  // kedua ujungnya di dalam amplop tetap di dalam sepanjang jalan.
  if (tilt_envelope_enable_)
  {
    const int pan_idx = using_joint_name_["head_pan"];
    const int tilt_idx = using_joint_name_["head_tilt"];
    const double tilt_max = tiltEnvelopeMax(target_position_.coeff(0, pan_idx));
    if (target_position_.coeff(0, tilt_idx) > tilt_max)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "head_tilt %+.1f dijepit ke %+.1f deg (servo 20 %.1f) pada pan %+.1f deg (%.0f %%)%s",
                           target_position_.coeff(0, tilt_idx) * RADIAN2DEGREE, tilt_max * RADIAN2DEGREE,
                           180.0 + tilt_max * RADIAN2DEGREE, target_position_.coeff(0, pan_idx) * RADIAN2DEGREE,
                           panPercent(target_position_.coeff(0, pan_idx)), is_offset ? " [tracking]" : " [absolut]");
      target_position_.coeffRef(0, tilt_idx) = tilt_max;
      const double calc_moving_time =
          fabs(goal_position_.coeff(0, tilt_idx) - tilt_max) / (angle_unit_ * M_PI / 180 * (is_offset ? 1.5 : 1.0));
      if (calc_moving_time > moving_time_)
        moving_time_ = calc_moving_time;
    }
  }

  // set mode
  is_direct_control_ = true;
  scan_state_ = NoScan;

  // generate trajectory
  // Objek sementara, langsung di-detach: tidak ada yang perlu disimpan atau
  // dibebaskan. Versi lama menyimpannya di anggota kelas dan menimpanya tiap
  // perintah tanpa delete -- bocor satu std::thread per perintah.
  std::thread(&HeadControlModule::jointTraGeneThread, this).detach();
}

void HeadControlModule::setHeadScanCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (enable_ == false)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Head control module is not enabled, scan command is canceled.");
    return;
  }
  else
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Scan command is accepted. [%d]", scan_state_);

  if (msg->data == "scan" && scan_state_ == NoScan)
  {
    std::srand(std::time(NULL)); // use current time as seed for random generator
    scan_state_ = std::rand() % 4 + 1;

    is_direct_control_ = false;

    generateScanTra(scan_state_);
  }
  else if (msg->data == "stop")
  {
    scan_state_ = NoScan;
  }
}

double HeadControlModule::panPercent(double pan_rad) const
{
  const double left = max_angle_.at(using_joint_name_.at("head_pan"));    // +85 = 0 %
  const double right = min_angle_.at(using_joint_name_.at("head_pan"));   // -85 = 100 %
  const double span = left - right;
  if (fabs(span) < 1e-9)
    return 50.0;
  return std::max(0.0, std::min(100.0, (left - pan_rad) / span * 100.0));
}

double HeadControlModule::tiltEnvelopeMax(double pan_rad) const
{
  const double p = panPercent(pan_rad);
  const double flat = tilt_envelope_flat_percent_;
  if (flat <= 0.0)
    return tilt_envelope_center_rad_;
  double t = 1.0;   // 1 = penuh (tengah), 0 = ujung
  if (p < flat)
    t = p / flat;
  else if (p > 100.0 - flat)
    t = (100.0 - p) / flat;
  return tilt_envelope_edge_rad_ + t * (tilt_envelope_center_rad_ - tilt_envelope_edge_rad_);
}

bool HeadControlModule::checkAngleLimit(const int joint_index, double &goal_position)
{
  std::map<int, double>::iterator angle_it = min_angle_.find(joint_index);
  if (angle_it == min_angle_.end())
    return false;
  double min_angle = angle_it->second;

  angle_it = max_angle_.find(joint_index);
  if (angle_it == max_angle_.end())
    return false;
  double max_angle = angle_it->second;

  if (goal_position < min_angle)
    goal_position = min_angle;
  if (goal_position > max_angle)
    goal_position = max_angle;

  return true;
}

void HeadControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  tra_lock_.lock();

  // get joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    robotis_framework::Dynamixel *_dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      _dxl = dxl_it->second;
    else
      continue;

    current_position_.coeffRef(0, index) = _dxl->dxl_state_->present_position_;
    goal_position_.coeffRef(0, index) = _dxl->dxl_state_->goal_position_;
  }

  has_goal_position_= true;

  // check to stop
  if (stop_process_ == true)
  {
    stopMoving();
  }
  else
  {
    // process
    if (tra_size_ != 0)
    {
      // start of steps
      if (tra_count_ == 0)
      {
        startMoving();
      }

      // end of steps
      if (tra_count_ >= tra_size_)
      {
        finishMoving();
      }
      else
      {
        // update goal position
        goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result_.size());
        goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
        goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());

        tra_count_ += 1;
      }
    }
  }
  tra_lock_.unlock();

  // set joint data
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];
    double goal_position = goal_position_.coeff(0, index);
    checkAngleLimit(index, goal_position);

    result_[joint_name]->goal_position_ = goal_position;
  }
}

void HeadControlModule::stop()
{
  tra_lock_.lock();

  if (is_moving_ == true)
    stop_process_ = true;

  tra_lock_.unlock();

  return;
}

bool HeadControlModule::isRunning()
{
  return is_moving_;
}

void HeadControlModule::onModuleEnable()
{
  scan_state_ = NoScan;
}

void HeadControlModule::onModuleDisable()
{
  // Dipanggil dari utas setCtrlModule controller, BUKAN utas process(). Tanpa
  // lock, penulisan di bawah ini bisa masuk di antara pemeriksaan
  // `tra_count_ < tra_size_` dan `calc_joint_tra_.block(tra_count_, ...)` di
  // process(): matriksnya sudah jadi 1 baris, blok dibaca dari baris
  // tra_count_ > 0 -> assertion Eigen Block.h:146 -> op3_manager mati SIGABRT.
  // Terjadi di robot 16 Sep saat kepala menyapu dan modul lain merebut kepala
  // (uji D60 ON/OFF berulang, siklus ke-8).
  std::lock_guard<std::mutex> guard(tra_lock_);

  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_direct_control_ = true;
  is_moving_ = false;
  has_goal_position_ = false;

  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  scan_state_ = NoScan;

  // Was a bare std::cout without a newline, so it printed mid-terminal and stuck
  // to whatever came next. It is normal bookkeeping -- the module is disabled
  // every time another module (an action page, for one) claims the head joints --
  // so it belongs at debug level, not in the operator's face.
  RCLCPP_DEBUG(this->get_logger(), "head_control_module : disable");
}

void HeadControlModule::startMoving()
{
  is_moving_ = true;

  // start procedure
}

void HeadControlModule::finishMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_direct_control_ = true;
  is_moving_ = false;

  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  // log
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Head movement is finished.");

  if (DEBUG)
    std::cout << "Trajectory End" << std::endl;

  // head scan
  switch (scan_state_)
  {
  case TopLeft:
    scan_state_ = BottomRight;
    break;

  case BottomRight:
    scan_state_ = BottomLeft;
    break;

  case BottomLeft:
    scan_state_ = TopRight;
    break;

  case TopRight:
    scan_state_ = TopLeft;
    break;

    // NoScan
  default:
    return;
  }

  generateScanTra(scan_state_);
}

void HeadControlModule::stopMoving()
{
  // init value
  calc_joint_tra_ = goal_position_;
  tra_size_ = 0;
  tra_count_ = 0;
  is_moving_ = false;
  is_direct_control_ = true;
  stop_process_ = false;
  scan_state_ = NoScan;

  goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
  goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

  // log
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_WARN, "Stop Module.");
}

void HeadControlModule::generateScanTra(const int head_direction)
{
  switch (head_direction)
  {
  case TopLeft:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) =  min_angle_[using_joint_name_["head_pan"]] * 0.6;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.25;
    break;
  }

  case TopRight:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) =  max_angle_[using_joint_name_["head_pan"]] * 0.6;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.25;
    break;
  }

  case BottomLeft:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.45;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
    break;
  }

  case BottomRight:
  {
    target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.45;
    target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
    break;
  }

  default:
    return;
  }

  // set moving time
  moving_time_ = 0.5;               // default : 0.5 sec

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    // set time
    double calc_moving_time = fabs(goal_position_.coeff(0, index) - target_position_.coeff(0, index)) / (60.0 * M_PI / 180.0);
    if (calc_moving_time > moving_time_)
      moving_time_ = calc_moving_time;
  }

  // generate trajectory
  // Objek sementara, langsung di-detach: tidak ada yang perlu disimpan atau
  // dibebaskan. Versi lama menyimpannya di anggota kelas dan menimpanya tiap
  // perintah tanpa delete -- bocor satu std::thread per perintah.
  std::thread(&HeadControlModule::jointTraGeneThread, this).detach();
}

/*
 simple minimum jerk trajectory

 pos_start : position at initial state
 vel_start : velocity at initial state
 accel_start : acceleration at initial state

 pos_end : position at final state
 vel_end : velocity at final state
 accel_end : acceleration at final state

 smp_time : sampling time

 mov_time : movement time
 */
Eigen::MatrixXd HeadControlModule::calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start,
                                                         double pos_end, double vel_end, double accel_end,
                                                         double smp_time, double mov_time)
{
  Eigen::MatrixXd poly_matrix(3, 3);
  Eigen::MatrixXd poly_vector(3, 1);

  poly_matrix << robotis_framework::powDI(mov_time, 3), robotis_framework::powDI(mov_time, 4), robotis_framework::powDI(
        mov_time, 5), 3 * robotis_framework::powDI(mov_time, 2), 4 * robotis_framework::powDI(mov_time, 3), 5
      * robotis_framework::powDI(mov_time, 4), 6 * mov_time, 12 * robotis_framework::powDI(mov_time, 2), 20
      * robotis_framework::powDI(mov_time, 3);

  poly_vector << pos_end - pos_start - vel_start * mov_time - accel_start * pow(mov_time, 2) / 2, vel_end - vel_start
      - accel_start * mov_time, accel_end - accel_start;

  Eigen::MatrixXd poly_coeff = poly_matrix.inverse() * poly_vector;

  int all_time_steps = round(mov_time / smp_time + 1);

  Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps, 1);
  Eigen::MatrixXd minimum_jer_tra = Eigen::MatrixXd::Zero(all_time_steps, 3);

  for (int step = 0; step < all_time_steps; step++)
    time.coeffRef(step, 0) = step * smp_time;

  for (int step = 0; step < all_time_steps; step++)
  {
    // position
    minimum_jer_tra.coeffRef(step, 0) = pos_start + vel_start * time.coeff(step, 0)
        + 0.5 * accel_start * robotis_framework::powDI(time.coeff(step, 0), 2)
        + poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 4)
        + poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 5);
    // velocity
    minimum_jer_tra.coeffRef(step, 1) = vel_start + accel_start * time.coeff(step, 0)
        + 3 * poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 4 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 3)
        + 5 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 4);
    // accel
    minimum_jer_tra.coeffRef(step, 2) = accel_start + 6 * poly_coeff.coeff(0, 0) * time.coeff(step, 0)
        + 12 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 2)
        + 20 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 3);
  }

  return minimum_jer_tra;
}

void HeadControlModule::jointTraGeneThread()
{
  tra_lock_.lock();

  double smp_time = control_cycle_msec_ * 0.001;		// ms -> s
  int all_time_steps = int(moving_time_ / smp_time) + 1;

  // for debug
  try
  {
    calc_joint_tra_.resize(all_time_steps, result_.size());
    calc_joint_vel_tra_.resize(all_time_steps, result_.size());
    calc_joint_accel_tra_.resize(all_time_steps, result_.size());
  }
  catch(std::exception &e)
  {
    std::cout << "All step tile : " << all_time_steps << std::endl;
    std::cout << e.what() << std::endl;
    throw;
  }

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
       state_it != result_.end(); state_it++)
  {
    std::string joint_name = state_it->first;
    int index = using_joint_name_[joint_name];

    double ini_value = goal_position_.coeff(0, index);
    double ini_vel = goal_velocity_.coeff(0, index);
    double ini_accel = goal_acceleration_.coeff(0, index);

    double tar_value = target_position_.coeff(0, index);

    Eigen::MatrixXd tra = calcMinimumJerkTraPVA(ini_value, ini_vel, ini_accel, tar_value, 0.0, 0.0, smp_time,
                                                moving_time_);

    calc_joint_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 0, all_time_steps, 1);
    calc_joint_vel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 1, all_time_steps, 1);
    calc_joint_accel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 2, all_time_steps, 1);
  }

  tra_size_ = calc_joint_tra_.rows();
  tra_count_ = 0;

  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[ready] make trajectory : %d, %d", tra_size_, tra_count_);

  tra_lock_.unlock();
}

void HeadControlModule::publishStatusMsg(unsigned int type, std::string msg)
{
  rclcpp::Time now = rclcpp::Clock().now();

  if(msg.compare(last_msg_) == 0)
  {
    rclcpp::Duration dur = now - last_msg_time_;
    if(dur.seconds() < 1)
      return;
  }

  auto status_msg = robotis_controller_msgs::msg::StatusMsg();
  status_msg.header.stamp = now;
  status_msg.type = type;
  status_msg.module_name = "Head Control";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);

  last_msg_ = msg;
  last_msg_time_ = now;
}
}

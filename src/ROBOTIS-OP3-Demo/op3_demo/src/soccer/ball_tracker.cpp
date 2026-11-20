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

#include "op3_demo/ball_tracker.h"

#include <algorithm>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace robotis_op
{

BallTracker::BallTracker()
  : // Node("ball_tracker"),
    FOV_WIDTH(35.2 * M_PI / 180),
    FOV_HEIGHT(21.6 * M_PI / 180),
    NOT_FOUND_THRESHOLD(50),
    WAITING_THRESHOLD(5),
    use_head_scan_(true),
    use_head_control_(true),
    count_not_found_(0),
    on_tracking_(false),
    current_ball_pan_(0),
    current_ball_tilt_(0),
    x_error_sum_(0),
    y_error_sum_(0),
    current_ball_bottom_(0),
    tracking_status_(NotFound),
    DEBUG_PRINT(false),
    scan_idx_(0),
    scan_session_(false),
    last_seen_pan_(0.0),
    present_pan_(0.0),
    present_tilt_(0.0),
    present_seen_(false),
    head_pan_limit_(85.0 * M_PI / 180.0),
    head_tilt_min_(-75.0 * M_PI / 180.0),
    head_tilt_max_(58.03 * M_PI / 180.0),
    scan_arrive_tol_(3.0 * M_PI / 180.0),
    scan_arrive_timeout_(10.0)
{
  // this->declare_parameter("p_gain", 0.4);
  // this->declare_parameter("i_gain", 0.0);
  // this->declare_parameter("d_gain", 0.0);

  // this->get_parameter("p_gain", p_gain_);
  // this->get_parameter("i_gain", i_gain_);
  // this->get_parameter("d_gain", d_gain_);
  p_gain_ = 0.45;
  i_gain_ = 0.0;
  d_gain_ = 0.045;

  RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "Ball tracking Gain : %f, %f, %f", p_gain_, i_gain_, d_gain_);

  loadScanPattern();

  // head_joint_offset_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states_offset", 10);
  // head_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states", 10);
  // head_scan_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/head_control/scan_command", 10);
  //  error_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ball_tracker/errors", 10);

  // ball_position_sub_ = this->create_subscription<op3_ball_detector_msgs::msg::CircleSetStamped>("/ball_detector_node/circle_set", 10, std::bind(&BallTracker::ballPositionCallback, this, std::placeholders::_1));
  // ball_tracking_command_sub_ = this->create_subscription<std_msgs::msg::String>("/ball_tracker/command", 10, std::bind(&BallTracker::ballTrackerCommandCallback, this, std::placeholders::_1));
}

BallTracker::~BallTracker()
{

}

void BallTracker::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ != nullptr)
  {
    ball_position_sub_ = node_->create_subscription<op3_ball_detector_msgs::msg::CircleSetStamped>("/ball_detector_node/circle_set", 10, std::bind(&BallTracker::ballPositionCallback, this, std::placeholders::_1));
    present_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/robotis/present_joint_states", 10, std::bind(&BallTracker::presentJointCallback, this, std::placeholders::_1));
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallTracker"), "Node is not set");
  }
}

void BallTracker::ballPositionCallback(const op3_ball_detector_msgs::msg::CircleSetStamped::SharedPtr msg)
{
  for (int idx = 0; idx < msg->circles.size(); idx++)
  {
    if (ball_position_.z >= msg->circles[idx].z)
      continue;

    ball_position_ = msg->circles[idx];
  }
}

// void BallTracker::ballTrackerCommandCallback(const std_msgs::msg::String::SharedPtr msg)
// {
//   if (msg->data == "start")
//   {
//     startTracking();
//   }
//   else if (msg->data == "stop")
//   {
//     stopTracking();
//   }
//   else if (msg->data == "toggle_start")
//   {
//     if (on_tracking_ == false)
//       startTracking();
//     else
//       stopTracking();
//   }
// }

void BallTracker::startTracking()
{
  on_tracking_ = true;
  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "Start Ball tracking");
}

void BallTracker::stopTracking()
{
  goInit();
  scan_session_ = false;

  on_tracking_ = false;
  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "Stop Ball tracking");

  current_ball_pan_ = 0;
  current_ball_tilt_ = 0;
  x_error_sum_ = 0;
  y_error_sum_ = 0;
}

void BallTracker::setUsingHeadScan(bool use_scan)
{
  use_head_scan_ = use_scan;
}

void BallTracker::setUsingHeadControl(bool use_head_control)
{
  use_head_control_ = use_head_control;
}

int BallTracker::processTracking()
{
  int tracking_status = Found;

  if (on_tracking_ == false)
  {
    ball_position_.z = 0;
    count_not_found_ = 0;
    return NotFound;
  }

  // check ball position
  if (ball_position_.z <= 0)
  {
    count_not_found_++;

    if (count_not_found_ < WAITING_THRESHOLD)
    {
      if(tracking_status_ == Found || tracking_status_ == Waiting)
        tracking_status = Waiting;
      else
        tracking_status = NotFound;
    }
    else if (count_not_found_ > NOT_FOUND_THRESHOLD)
    {
      scanBall();
      count_not_found_ = 0;
      tracking_status = NotFound;
    }
    else
    {
      tracking_status = NotFound;
    }
  }
  else
  {
    count_not_found_ = 0;
    scan_session_ = false;
  }

  // if ball is found
  // convert ball position to desired angle(rad) of head
  // ball_position : top-left is (-1, -1), bottom-right is (+1, +1)
  // offset_rad : top-left(+, +), bottom-right(-, -)
  double x_error = 0.0, y_error = 0.0, ball_size = 0.0;

  switch (tracking_status)
  {
  case NotFound:
    tracking_status_ = tracking_status;
    current_ball_pan_ = 0;
    current_ball_tilt_ = 0;
    x_error_sum_ = 0;
    y_error_sum_ = 0;
    return tracking_status;

  case Waiting:
    tracking_status_ = tracking_status;
    return tracking_status;

  case Found:
    x_error = -atan(ball_position_.x * tan(FOV_WIDTH));
    y_error = -atan(ball_position_.y * tan(FOV_HEIGHT));
    ball_size = ball_position_.z;
    last_seen_pan_ = x_error;   // + = bola di kiri gambar, - = di kanan
    break;

  default:
    break;
  }

  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "--------------------------------------------------------------");
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "Ball position : %f | %f", ball_position_.x, ball_position_.y);
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "Target angle : %f | %f", (x_error * 180 / M_PI), (y_error * 180 / M_PI));
  }

  rclcpp::Time curr_time = rclcpp::Clock().now();
  rclcpp::Duration dur = curr_time - prev_time_;
  double delta_time = dur.seconds();
  prev_time_ = curr_time;

  double x_error_diff = (x_error - current_ball_pan_) / delta_time;
  double y_error_diff = (y_error - current_ball_tilt_) / delta_time;
  x_error_sum_ += x_error;
  y_error_sum_ += y_error;
  double x_error_target = x_error * p_gain_ + x_error_diff * d_gain_ + x_error_sum_ * i_gain_;
  double y_error_target = y_error * p_gain_ + y_error_diff * d_gain_ + y_error_sum_ * i_gain_;

  //  std_msgs::msg::Float64MultiArray x_error_msg;
  //  x_error_msg.data.push_back(x_error);
  //  x_error_msg.data.push_back(x_error_diff);
  //  x_error_msg.data.push_back(x_error_sum_);
  //  x_error_msg.data.push_back(x_error * p_gain_);
  //  x_error_msg.data.push_back(x_error_diff * d_gain_);
  //  x_error_msg.data.push_back(x_error_sum_ * i_gain_);
  //  x_error_msg.data.push_back(x_error_target);
  //  error_pub_.publish(x_error_msg);

  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "------------------------  %d  --------------------------------------", tracking_status);
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "error         : %f | %f", (x_error * 180 / M_PI), (y_error * 180 / M_PI));
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "error_diff    : %f | %f | %f", (x_error_diff * 180 / M_PI), (y_error_diff * 180 / M_PI), delta_time);
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "error_sum    : %f | %f", (x_error_sum_ * 180 / M_PI), (y_error_sum_ * 180 / M_PI));
    RCLCPP_INFO(rclcpp::get_logger("BallTracker"), "error_target  : %f | %f | P : %f | D : %f | time : %f", (x_error_target * 180 / M_PI), (y_error_target * 180 / M_PI), p_gain_, d_gain_, delta_time);
  }

  // move head joint
  publishHeadJoint(x_error_target, y_error_target);

  // args for following ball
  current_ball_pan_ = x_error;
  current_ball_tilt_ = y_error;
  current_ball_bottom_ = ball_size;

  ball_position_.z = 0;

  tracking_status_ = tracking_status;
  return tracking_status;
}

void BallTracker::publishHeadJoint(double pan, double tilt)
{
  if (use_head_control_ == false)
    return;

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallTracker"), "Node is not set, cannot publish head joint");
    return;
  }

  double min_angle = 1 * M_PI / 180;
  if (fabs(pan) < min_angle && fabs(tilt) < min_angle)
    return;

  auto head_joint_offset_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states_offset", 10);
  sensor_msgs::msg::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(pan);
  head_angle_msg.position.push_back(tilt);

  head_joint_offset_pub_->publish(head_angle_msg);
}

void BallTracker::goInit()
{
  if (use_head_control_ == false)
    return;

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallTracker"), "Node is not set, cannot go init");
    return;
  }
  auto head_joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states", 10);
  sensor_msgs::msg::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");

  head_angle_msg.position.push_back(0.0);
  head_angle_msg.position.push_back(0.0);

  head_joint_pub_->publish(head_angle_msg);
}

void BallTracker::scanBall()
{
  if (use_head_control_ == false)
    return;

  if (use_head_scan_ == false)
    return;

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallTracker"), "Node is not set, cannot scan ball");
    return;
  }

  // Dulu: kirim "scan" ke head_control_module (sapuan diagonal bawaan ROBOTIS,
  // pojok awal acak, dan di robot ini menengadah). Sekarang: pola 2D yang sama
  // persis dengan head_tracking_node. processTracking() memanggil fungsi ini
  // tiap NOT_FOUND_THRESHOLD siklus selama bola hilang.
  if (scan_pattern_.empty())
    return;

  if (scan_session_ == false)
  {
    // Masuk (atau kembali) ke searching: LANJUTKAN urutan dari langkah terakhir,
    // jangan dimulai ulang, supaya perpindahannya tetap KANAN -> BAWAH -> KIRI ->
    // ATAS walau searching sempat terhenti karena bola terdeteksi.
    scan_session_ = true;
    publishScanTarget();
    return;
  }

  // Langkah berikutnya HANYA kalau kepala benar-benar sudah sampai di target
  // sekarang. Menyeberang kiri<->kanan penuh butuh 4,0 detik (170 derajat pada
  // 42 deg/s), lebih lama dari jeda antar panggilan -- tanpa penantian ini
  // perjalanan ke kiri dipotong dan kepala terjebak di sisi kanan.
  const double elapsed = (node_->now() - scan_target_time_).seconds();
  if (headArrived() == false && elapsed < scan_arrive_timeout_)
    return;

  if (headArrived() == false)
    RCLCPP_WARN(rclcpp::get_logger("BallTracker"),
                "Kepala tidak sampai ke target dalam %.0f dtk -- lanjut ke langkah berikutnya",
                scan_arrive_timeout_);

  publishScanTarget();
}

void BallTracker::publishScanTarget()
{
  scan_target_ = scan_pattern_[scan_idx_];
  scan_idx_ = (scan_idx_ + 1) % scan_pattern_.size();
  scan_target_time_ = node_->now();

  auto head_joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/robotis/head_control/set_joint_states", 10);
  sensor_msgs::msg::JointState head_angle_msg;
  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.name.push_back("head_tilt");
  head_angle_msg.position.push_back(scan_target_.first);
  head_angle_msg.position.push_back(scan_target_.second);
  head_joint_pub_->publish(head_angle_msg);
}

void BallTracker::presentJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++)
  {
    if (msg->name[i] == "head_pan")
      present_pan_ = msg->position[i];
    else if (msg->name[i] == "head_tilt")
      present_tilt_ = msg->position[i];
  }
  present_seen_ = true;
}

// Dibandingkan dengan target yang SUDAH dipotong batas keras head_control_module,
// karena target di luar batas tidak akan pernah dicapai persis.
bool BallTracker::headArrived() const
{
  if (present_seen_ == false)
    return false;
  const double pan  = std::max(-head_pan_limit_, std::min(head_pan_limit_, scan_target_.first));
  const double tilt = std::max(head_tilt_min_, std::min(head_tilt_max_, scan_target_.second));
  return std::fabs(present_pan_ - pan) <= scan_arrive_tol_ &&
         std::fabs(present_tilt_ - tilt) <= scan_arrive_tol_;
}

void BallTracker::loadScanPattern()
{
  // Angka cadangan = isi head_tracking.yaml per 2026-09-10. Hanya dipakai kalau
  // berkasnya tidak terbaca, dan itu dilaporkan sebagai ERROR.
  double p = 1.5708, fw = 0.5238, dn = 1.0128;   // servo 210 / 238 derajat

  try
  {
    const std::string path =
        ament_index_cpp::get_package_share_directory("op3_ball_localization") + "/config/head_tracking.yaml";
    YAML::Node doc = YAML::LoadFile(path);
    YAML::Node prm = doc["head_tracking_node"]["ros__parameters"];
    if (prm["scan_pan_rad"])          p  = prm["scan_pan_rad"].as<double>();
    if (prm["scan_tilt_forward_rad"]) fw = prm["scan_tilt_forward_rad"].as<double>();
    if (prm["scan_tilt_down_rad"])    dn = prm["scan_tilt_down_rad"].as<double>();
    if (prm["head_pan_limit_rad"])      head_pan_limit_      = std::fabs(prm["head_pan_limit_rad"].as<double>());
    if (prm["head_tilt_min_rad"])       head_tilt_min_       = prm["head_tilt_min_rad"].as<double>();
    if (prm["head_tilt_max_rad"])       head_tilt_max_       = prm["head_tilt_max_rad"].as<double>();
    if (prm["scan_arrive_tol_rad"])     scan_arrive_tol_     = prm["scan_arrive_tol_rad"].as<double>();
    if (prm["scan_arrive_timeout_sec"]) scan_arrive_timeout_ = prm["scan_arrive_timeout_sec"].as<double>();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallTracker"),
                 "head_tracking.yaml tidak terbaca (%s) -- pola searching memakai angka cadangan", e.what());
  }

  // Urutan SAMA PERSIS dengan head_tracking_node._scan_positions_2d:
  //   KANAN -> BAWAH -> KIRI -> ATAS -> (ulang)
  // head_pan positif = kiri; fw = servo 210 (atas), dn = servo 238 (bawah).
  scan_pattern_ = {{-p, fw}, {-p, dn}, {+p, dn}, {+p, fw}};
}

}


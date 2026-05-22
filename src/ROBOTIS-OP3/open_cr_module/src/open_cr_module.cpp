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

#include <cmath>
#include <sstream>

#include "open_cr_module/open_cr_module.h"

namespace robotis_op
{

OpenCRModule::OpenCRModule()
: Node("open_cr_module"),
  control_cycle_msec_(8),
  DEBUG_PRINT(false),
  previous_volt_(0.0),
  present_volt_(0.0)
{
  module_name_ = "open_cr_module";

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button_mode"] = 0.0;
  result_["button_start"] = 0.0;
  result_["button_user"] = 0.0;

  result_["present_voltage"] = 0.0;

  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["button_user"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;
  buttons_["published_user"] = false;

  previous_result_["gyro_x"] = 0.0;
  previous_result_["gyro_y"] = 0.0;
  previous_result_["gyro_z"] = 0.0;
  previous_result_["gyro_x_prev"] = 0.0;
  previous_result_["gyro_y_prev"] = 0.0;
  previous_result_["gyro_z_prev"] = 0.0;

  previous_result_["acc_x"] = 0.0;
  previous_result_["acc_y"] = 0.0;
  previous_result_["acc_z"] = 0.0;

  last_msg_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
}

OpenCRModule::~OpenCRModule()
{
  if (queue_thread_.joinable()) {
    queue_thread_.join();
  }
}

void OpenCRModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  (void) robot;
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = std::thread(&OpenCRModule::queueThread, this);
}

void OpenCRModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/robotis/open_cr/imu", 1);
  button_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/open_cr/button", 1);
  dxl_power_msg_pub_ =
    this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 1);

  rclcpp::WallRate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok()) {
    executor.spin_some();
    rate.sleep();
  }
}

void OpenCRModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, robotis_framework::Sensor *> sensors)
{
  (void) dxls;
  if (sensors["open-cr"] == nullptr) {
    return;
  }

  int16_t gyro_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_x"];
  int16_t gyro_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_y"];
  int16_t gyro_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_z"];

  int16_t acc_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_x"];
  int16_t acc_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_y"];
  int16_t acc_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_z"];

  uint16_t present_volt = sensors["open-cr"]->sensor_state_->bulk_read_table_["present_voltage"];

  result_["gyro_x"] = lowPassFilter(0.4, -getGyroValue(gyro_x), previous_result_["gyro_x"]);
  result_["gyro_y"] = lowPassFilter(0.4, -getGyroValue(gyro_y), previous_result_["gyro_y"]);
  result_["gyro_z"] = lowPassFilter(0.4, getGyroValue(gyro_z), previous_result_["gyro_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Gyro ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", gyro_x, gyro_y, gyro_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f",
                         result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  result_["acc_x"] = lowPassFilter(0.4, -getAccValue(acc_x), previous_result_["acc_x"]);
  result_["acc_y"] = lowPassFilter(0.4, -getAccValue(acc_y), previous_result_["acc_y"]);
  result_["acc_z"] = lowPassFilter(0.4, getAccValue(acc_z), previous_result_["acc_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Acc ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", acc_x, acc_y, acc_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f",
                         result_["acc_x"], result_["acc_y"], result_["acc_z"]);

  const auto& update_stamp = sensors["open-cr"]->sensor_state_->update_time_stamp_;
  rclcpp::Time update_time(
    update_stamp.sec_,
    update_stamp.nsec_,
    this->get_clock()->get_clock_type());
  rclcpp::Duration update_duration = this->get_clock()->now() - update_time;
  if (update_duration.nanoseconds() > 100000000) {
    publishDXLPowerMsg(1);
  }

  uint8_t button_flag = sensors["open-cr"]->sensor_state_->bulk_read_table_["button"];
  result_["button_mode"] = button_flag & 0x01;
  result_["button_start"] = (button_flag & 0x02) >> 1;
  result_["button_user"] = (button_flag & 0x04) >> 2;

  handleButton("mode");
  handleButton("start");
  handleButton("user");

  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  publishIMU();

  previous_result_["gyro_x_prev"] = result_["gyro_x"];
  previous_result_["gyro_y_prev"] = result_["gyro_y"];
  previous_result_["gyro_z_prev"] = result_["gyro_z"];
}

double OpenCRModule::getGyroValue(int raw_value)
{
  return static_cast<double>(raw_value) * GYRO_FACTOR * DEGREE2RADIAN;
}

double OpenCRModule::getAccValue(int raw_value)
{
  return static_cast<double>(raw_value) * ACCEL_FACTOR;
}

void OpenCRModule::publishIMU()
{
  imu_msg_.header.stamp = this->get_clock()->now();
  imu_msg_.header.frame_id = "body_link";

  imu_msg_.angular_velocity.x = result_["gyro_x"];
  imu_msg_.angular_velocity.y = result_["gyro_y"];
  imu_msg_.angular_velocity.z = result_["gyro_z"];

  imu_msg_.linear_acceleration.x = result_["acc_x"] * G_ACC;
  imu_msg_.linear_acceleration.y = result_["acc_y"] * G_ACC;
  imu_msg_.linear_acceleration.z = result_["acc_z"] * G_ACC;

  double mui = 0.01;
  double sign = std::copysign(1.0, result_["acc_z"]);
  double roll = std::atan2(
    -result_["acc_x"],
    sign * std::sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_y"] * result_["acc_y"]));
  double pitch = std::atan2(
    result_["acc_y"],
    std::sqrt(result_["acc_x"] * result_["acc_x"] + result_["acc_z"] * result_["acc_z"]));
  double yaw = 0.0;

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_->publish(imu_msg_);
}

void OpenCRModule::handleButton(const std::string &button_name)
{
  std::string button_key = "button_" + button_name;
  std::string button_published = "published_" + button_name;

  bool pushed = (result_[button_key] == 1.0);
  if (buttons_[button_key] == pushed) {
    if (pushed && !buttons_[button_published]) {
      auto press_time_it = buttons_press_time_.find(button_name);
      if (press_time_it == buttons_press_time_.end()) {
        buttons_press_time_.emplace(
          button_name,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));
        return;
      }
      rclcpp::Duration button_duration = this->get_clock()->now() - buttons_press_time_[button_name];
      if (button_duration.seconds() > 2.0) {
        publishButtonMsg(button_name + "_long");
        buttons_[button_published] = true;
      }
    }
  } else {
    buttons_[button_key] = pushed;

    if (pushed) {
      buttons_press_time_[button_name] = this->get_clock()->now();
      buttons_[button_published] = false;
    } else {
      auto press_time_it = buttons_press_time_.find(button_name);
      if (press_time_it == buttons_press_time_.end()) {
        buttons_press_time_.emplace(
          button_name,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));
        return;
      }
      rclcpp::Duration button_duration = this->get_clock()->now() - buttons_press_time_[button_name];
      if (button_duration.seconds() < 2.0) {
        publishButtonMsg(button_name);
      }
    }
  }
}

void OpenCRModule::publishButtonMsg(const std::string &button_name)
{
  std_msgs::msg::String button_msg;
  button_msg.data = button_name;

  button_pub_->publish(button_msg);
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void OpenCRModule::handleVoltage(double present_volt)
{
  double voltage_ratio = 0.4;
  previous_volt_ =
    (previous_volt_ != 0.0) ? previous_volt_ * (1.0 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  if (std::fabs(present_volt_ - previous_volt_) >= 0.1) {
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Duration dur = now - last_msg_time_;
    if (dur.seconds() < 1.0) {
      return;
    }

    last_msg_time_ = now;
    present_volt_ = previous_volt_;

    std::stringstream log_stream;
    log_stream << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg(
      (present_volt_ < 11.0 ?
        robotis_controller_msgs::msg::StatusMsg::STATUS_WARN :
        robotis_controller_msgs::msg::StatusMsg::STATUS_INFO),
      log_stream.str());

    RCLCPP_INFO_EXPRESSION(
      this->get_logger(), DEBUG_PRINT, "Present Volt : %fV, Read Volt : %fV",
      previous_volt_, result_["present_voltage"]);
  }
}

void OpenCRModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::msg::StatusMsg status_msg;
  status_msg.header.stamp = this->get_clock()->now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

void OpenCRModule::publishDXLPowerMsg(unsigned int value)
{
  robotis_controller_msgs::msg::SyncWriteItem sync_write_msg;
  sync_write_msg.item_name = "dynamixel_power";
  sync_write_msg.joint_name.push_back("open-cr");
  sync_write_msg.value.push_back(value);

  dxl_power_msg_pub_->publish(sync_write_msg);
}

double OpenCRModule::lowPassFilter(double alpha, double x_new, double &x_old)
{
  double filtered_value = alpha * x_new + (1.0 - alpha) * x_old;
  x_old = filtered_value;
  return filtered_value;
}

}  // namespace robotis_op

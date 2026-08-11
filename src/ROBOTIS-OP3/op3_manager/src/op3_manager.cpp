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

/* ROS2 API Header */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

/* Standard Library Header */
#include <chrono>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
#include "op3_head_control_module/head_control_module.h"
#include "op3_action_module/action_module.h"
#include "op3_walking_module/op3_walking_module.h"
#include "op3_direct_control_module/direct_control_module.h"
#include "op3_online_walking_module/online_walking_module.h"
#include "op3_tuning_module/tuning_module.h"

using namespace robotis_framework;
using namespace dynamixel;
using namespace robotis_op;

const int BAUD_RATE = 2000000;
const double PROTOCOL_VERSION = 2.0;
const int SUB_CONTROLLER_ID = 200;
const int DXL_BROADCAST_ID = 254;
const int DEFAULT_DXL_ID = 1;
const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
const int POWER_CTRL_TABLE = 24;
const int RGB_LED_CTRL_TABLE = 26;
const int TORQUE_ON_CTRL_TABLE = 64;

bool g_is_simulation = false;
int g_baudrate;
std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;
std::string g_device_name;
// On a stock OP3 the OpenCR (ID 200) sits on the same TTL bus as the servos, so
// one device_name served both. This robot's OpenCR is deaf on that bus and only
// answers over its own USB CDC port, so the ID 200 writes (DXL power-on, RGB
// LED) need a port of their own -- pointing them at the U2D2 is what produced
// the repeated "Torque on DXLs! [RxPacketError]" and "Fail to control LED".
// Empty means "same as device_name", which keeps stock OP3 behaviour.
std::string g_sub_controller_device;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_demo_command_pub;
// Boot pose goes out over the same two topics the studio's "Init Pose" button
// uses, because that path is the one proven to work on hardware (and is what
// the CHRONUS robot boots with). Calling setCtrlModule() + ActionModule::start()
// directly from main() looked equivalent and logged success, but the robot never
// actually moved -- see the 2026-08-11 logs.
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_enable_ctrl_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr g_action_page_pub;
std::mutex g_health_mutex;

std::string escapeJsonString(const std::string& value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (char ch : value)
  {
    switch (ch)
    {
    case '\\':
    case '"':
      escaped.push_back('\\');
      escaped.push_back(ch);
      break;
    case '\n':
      escaped += "\\n";
      break;
    case '\r':
      escaped += "\\r";
      break;
    case '\t':
      escaped += "\\t";
      break;
    default:
      escaped.push_back(ch);
      break;
    }
  }
  return escaped;
}

std::string buildJsonArray(const std::vector<std::string>& items)
{
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < items.size(); ++i)
  {
    if (i > 0)
      stream << ",";
    stream << "\"" << escapeJsonString(items[i]) << "\"";
  }
  stream << "]";
  return stream.str();
}

std::string buildJsonObject(const std::map<std::string, std::string>& items)
{
  std::ostringstream stream;
  stream << "{";
  size_t index = 0;
  for (const auto& entry : items)
  {
    if (index > 0)
      stream << ",";
    stream << "\"" << escapeJsonString(entry.first) << "\":\"" << escapeJsonString(entry.second) << "\"";
    ++index;
  }
  stream << "}";
  return stream.str();
}

// Decodes the servo's own hardware_error_status latch (X/MX-2.0 control table,
// address 70). A servo that trips one of these shuts its torque off by itself
// and refuses to come back until the latch is cleared -- which from ROS looks
// exactly like "the torque button did nothing".
std::string describeDxlError(uint8_t bits)
{
  std::string text;
  if (bits & 0x01) text += "input_voltage,";
  if (bits & 0x04) text += "overheating,";
  if (bits & 0x08) text += "motor_encoder,";
  if (bits & 0x10) text += "electrical_shock,";
  if (bits & 0x20) text += "overload,";
  if (text.empty())
    return "unknown(" + std::to_string((int) bits) + ")";
  text.erase(text.size() - 1);
  return text;
}

void buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "user_long")
  {
    RobotisController *controller = RobotisController::getInstance();

    controller->setCtrlModule("none");

    controller->stopTimer();

    if (g_is_simulation == false)
    {
      // power and torque on
      PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
      bool set_port_result = port_handler->setBaudRate(g_baudrate);
      if (set_port_result == false)
      {
        RCLCPP_ERROR(controller->get_logger(), "Error Set port");
        return;
      }
      PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

      // check dxls torque.
      uint8_t torque = 0;
      packet_handler->read1ByteTxRx(port_handler, DEFAULT_DXL_ID, TORQUE_ON_CTRL_TABLE, &torque);

      if (torque != 1)
      {
        controller->initializeDevice(g_init_file);
      }
      else
      {
        RCLCPP_INFO(controller->get_logger(), "Torque is already on!!");
      }
    }

    controller->startTimer();

    usleep(200 * 1000);

    // go to init pose
    std_msgs::msg::String init_msg;
    init_msg.data = "ini_pose";

    g_init_pose_pub->publish(init_msg);
    RCLCPP_INFO(controller->get_logger(), "Go to init pose");
  }
}

void dxlTorqueCheckCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (g_is_simulation == true)
    return;

  // check dxl torque
  uint8_t torque_result = 0;
  bool torque_on = true;
  RobotisController *controller = RobotisController::getInstance();
  //controller->robot_->port_default_device_

  for (std::map<std::string, std::string>::iterator map_it = controller->robot_->port_default_device_.begin();
       map_it != controller->robot_->port_default_device_.end(); map_it++)
  {
    std::string default_device_name = map_it->second;
    controller->read1Byte(default_device_name, TORQUE_ON_CTRL_TABLE, &torque_result);

    // if not, torque on
    if (torque_result != 1)
      torque_on = false;
  }

  if(torque_on == false)
  {
    controller->stopTimer();

    controller->initializeDevice(g_init_file);

    controller->startTimer();
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("op3_manager");

  RCLCPP_INFO(node->get_logger(), "manager->init");
  RobotisController *controller = RobotisController::getInstance();

  /* Load ROS Parameter */

  node->declare_parameter<std::string>("offset_file_path", "");
  node->declare_parameter<std::string>("robot_file_path", "");
  node->declare_parameter<std::string>("init_file_path", "");
  node->declare_parameter<std::string>("device_name", SUB_CONTROLLER_DEVICE);
  node->declare_parameter<std::string>("sub_controller_device", "");
  node->declare_parameter<int>("baud_rate", BAUD_RATE);

  node->get_parameter("offset_file_path", g_offset_file);
  node->get_parameter("robot_file_path", g_robot_file);
  node->get_parameter("init_file_path", g_init_file);
  node->get_parameter("device_name", g_device_name);
  node->get_parameter("sub_controller_device", g_sub_controller_device);
  node->get_parameter("baud_rate", g_baudrate);

  if (g_sub_controller_device.empty())
    g_sub_controller_device = g_device_name;

  auto button_sub = node->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 1, buttonHandlerCallback);
  auto dxl_torque_sub = node->create_subscription<std_msgs::msg::String>("/robotis/dxl_torque", 1, dxlTorqueCheckCallback);
  g_init_pose_pub = node->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose", 0);
  g_demo_command_pub = node->create_publisher<std_msgs::msg::String>("/ball_tracker/command", 10);
  g_enable_ctrl_pub = node->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 10);
  g_action_page_pub = node->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);

  node->declare_parameter<bool>("simulation", false);
  node->get_parameter("simulation", controller->gazebo_mode_);
  g_is_simulation = controller->gazebo_mode_;

  /* real robot */
  if (g_is_simulation == false)
  {
    // Sub controller port: everything in this block talks to ID 200 (OpenCR),
    // not to the servos, so it uses g_sub_controller_device.
    PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_sub_controller_device.c_str());
    bool set_port_result = port_handler->setBaudRate(g_baudrate);
    if (set_port_result == false)
      RCLCPP_ERROR(node->get_logger(), "Error Set port (sub controller: %s)", g_sub_controller_device.c_str());

    PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // power on dxls
    int torque_on_count = 0;

    while (torque_on_count < 5)
    {
      int _return = packet_handler->write1ByteTxRx(port_handler, SUB_CONTROLLER_ID, POWER_CTRL_TABLE, 1);

      if(_return != 0)
        RCLCPP_ERROR(node->get_logger(), "Torque on DXLs! [%s]", packet_handler->getRxPacketError(_return));
      else
        RCLCPP_INFO(node->get_logger(), "Torque on DXLs!");

      if (_return == 0)
        break;
      else
        torque_on_count++;
    }

    usleep(100 * 1000);

    // set RGB-LED to GREEN
    int led_full_unit = 0x1F;
    int led_range = 5;
    int led_value = led_full_unit << led_range;
    int _return = packet_handler->write2ByteTxRx(port_handler, SUB_CONTROLLER_ID, RGB_LED_CTRL_TABLE, led_value);

    if(_return != 0)
      RCLCPP_ERROR(node->get_logger(), "Fail to control LED [%s]", packet_handler->getRxPacketError(_return));

    port_handler->closePort();

    // The write above switches on the rail that feeds the servos, so they are
    // cold-starting right here and need time to answer. The blind 100 ms was
    // harmless while that write was still failing -- the rail was already on and
    // never toggled -- but now that it succeeds, initializeDevice() below can
    // run before the servos are listening. It writes torque_enable exactly once
    // per joint, so any servo still booting silently stays limp: the robot then
    // ignores the boot pose, and individual joints (the head is the usual
    // victim) end up in a different torque state than the rest.
    // Wait for a servo to actually answer instead of guessing a delay.
    PortHandler *dxl_port = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
    if (dxl_port->setBaudRate(g_baudrate) == false)
    {
      RCLCPP_ERROR(node->get_logger(), "Error Set port (dxl bus: %s)", g_device_name.c_str());
    }
    else
    {
      const int dxl_ready_timeout_ms = 3000;
      int dxl_waited_ms = 0;
      uint16_t model_number = 0;
      uint8_t dxl_error = 0;

      while (packet_handler->ping(dxl_port, DEFAULT_DXL_ID, &model_number, &dxl_error) != COMM_SUCCESS
             && dxl_waited_ms < dxl_ready_timeout_ms)
      {
        usleep(50 * 1000);
        dxl_waited_ms += 50;
      }

      if (dxl_waited_ms >= dxl_ready_timeout_ms)
        RCLCPP_ERROR(node->get_logger(),
                     "DXL bus silent %d ms after power on -- joints may come up without torque",
                     dxl_waited_ms);
      else
        RCLCPP_INFO(node->get_logger(), "DXLs answered %d ms after power on", dxl_waited_ms);

      dxl_port->closePort();
    }
  }
  /* simulation mode */
  else
  {
    RCLCPP_WARN(node->get_logger(), "SET TO SIMULATION MODE!");
    std::string robot_name;
    node->declare_parameter<std::string>("simulation_robot_name", "");
    node->get_parameter("simulation_robot_name", robot_name);
    if (robot_name != "")
      controller->gazebo_robot_name_ = robot_name;
  }

  if (g_robot_file == "")
  {
    RCLCPP_ERROR(node->get_logger(), "NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    RCLCPP_ERROR(node->get_logger(), "ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  // load offset
  if (g_offset_file != "")
    controller->loadOffset(g_offset_file);

  usleep(300 * 1000);

  /* Add Sensor Module */
  controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

  /* Add Motion Module */
  controller->addMotionModule((MotionModule*) ActionModule::getInstance());
  controller->addMotionModule((MotionModule*) BaseModule::getInstance());
  controller->addMotionModule((MotionModule*) HeadControlModule::getInstance());
  controller->addMotionModule((MotionModule*) WalkingModule::getInstance());
  controller->addMotionModule((MotionModule*) DirectControlModule::getInstance());
  controller->addMotionModule((MotionModule*) OnlineWalkingModule::getInstance());
  controller->addMotionModule((MotionModule*) TuningModule::getInstance());

  // start timer
  controller->startTimer();

  usleep(100 * 1000);

  // Boot-time INIT pose: play action page 2 (INIT_BARU) straight from
  // motion_4095_ros1_lama.bin, the same page the action editor edits. This
  // used to publish "ini_pose" and let base_module replay
  // op3_base_module/data/ini_pose.yaml, a hand-retuned mirror of page 2 that
  // drifts out of sync every time the page is re-taught.
  //
  // Both calls below go through singletons already constructed above, so this
  // allocates nothing new — adding publishers to this function is what smashed
  // the stack last time. Set boot_init_page:=0 to go back to the ini_pose path.
  int boot_init_page = 2;
  node->declare_parameter<int>("boot_init_page", boot_init_page);
  node->get_parameter("boot_init_page", boot_init_page);

  if (boot_init_page > 0)
  {
    // Ported from the CHRONUS robot, whose boot lands the pose reliably: drive
    // the action module over its topics instead of calling into the singleton.
    // The direct call (setCtrlModule + ActionModule::start) reported success and
    // even logged a full 1650 ms page play while the robot stood still, so the
    // publish path is not merely a stylistic difference -- it is the one that
    // works. Publishing also means the page request is handled by the module's
    // own callback, on the executor, after the module switch has really landed.
    std_msgs::msg::String mode_msg;
    mode_msg.data = "action_module";
    g_enable_ctrl_pub->publish(mode_msg);

    // CHRONUS waits 500 ms here. Keep that: the module switch runs on a worker
    // thread and the action module needs one enabled control cycle to seed the
    // pose it interpolates from (action_module.cpp:263-279).
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    std_msgs::msg::Int32 page_msg;
    page_msg.data = boot_init_page;
    g_action_page_pub->publish(page_msg);

    RCLCPP_INFO(node->get_logger(), "Go to init pose (action page %d)", boot_init_page);
  }
  else
  {
    // boot_init_page:=0 keeps the old base_module path.
    std_msgs::msg::String init_msg;
    init_msg.data = "ini_pose";
    g_init_pose_pub->publish(init_msg);
    RCLCPP_INFO(node->get_logger(), "Go to init pose (ini_pose → base_module)");
  }

  auto health_service = node->create_service<std_srvs::srv::Trigger>(
      "/robotis/health_check",
      [](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void) request;
        std::lock_guard<std::mutex> lock(g_health_mutex);

        if (g_is_simulation)
        {
          response->success = true;
          response->message = "{\"ok\":[],\"failed\":[],\"errors\":{},\"total\":0,\"duration_ms\":0,"
                              "\"skipped\":true,\"reason\":\"simulation\"}";
          return;
        }

        RobotisController *controller = RobotisController::getInstance();
        if (controller == nullptr || controller->robot_ == nullptr)
        {
          response->success = false;
          response->message = "RobotisController not initialized";
          return;
        }

        bool was_running = controller->isTimerRunning();
        if (was_running)
          controller->stopTimer();

        auto start_time = std::chrono::steady_clock::now();
        std::vector<std::string> ok;
        std::vector<std::string> failed;
        std::map<std::string, std::string> errors;
        std::map<std::string, std::string> torque;      // per joint: "on" / "off"
        std::map<std::string, std::string> hw_errors;   // only joints with a latch set
        std::map<std::string, std::string> temps;       // degrees C, straight from the servo

        for (const auto& it : controller->robot_->dxls_)
        {
          const std::string& joint_name = it.first;
          Dynamixel *dxl = it.second;
          int result = controller->ping(joint_name);
          if (result != COMM_SUCCESS)
          {
            failed.push_back(joint_name);
            dynamixel::PacketHandler *handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
            errors[joint_name] = handler->getTxRxResult(result);
            continue;
          }

          ok.push_back(joint_name);

          // Neither of these is in the bulk read -- OP3.robot only reads
          // position and the three gains -- so a joint that has switched its
          // own torque off is indistinguishable from a healthy one over ROS.
          // Reading them here is what lets "Run Check" answer why a joint is
          // dead instead of just confirming it answers a ping.
          uint8_t torque_state = 0;
          if (dxl->torque_enable_item_ != NULL
              && controller->read1Byte(joint_name, dxl->torque_enable_item_->address_, &torque_state) == COMM_SUCCESS)
          {
            torque[joint_name] = (torque_state == 1) ? "on" : "off";
          }

          // Temperature turns "it overheated" from a guess into a number, and
          // shows which joints are on their way there before the latch trips.
          auto temp_item = dxl->ctrl_table_.find("present_temperature");
          uint8_t temp_c = 0;
          if (temp_item != dxl->ctrl_table_.end()
              && controller->read1Byte(joint_name, temp_item->second->address_, &temp_c) == COMM_SUCCESS)
          {
            temps[joint_name] = std::to_string((int) temp_c);
          }

          auto error_item = dxl->ctrl_table_.find("hardware_error_status");
          uint8_t error_bits = 0;
          if (error_item != dxl->ctrl_table_.end()
              && controller->read1Byte(joint_name, error_item->second->address_, &error_bits) == COMM_SUCCESS
              && error_bits != 0)
          {
            hw_errors[joint_name] = describeDxlError(error_bits);
          }
        }

        if (was_running)
          controller->startTimer();

        auto end_time = std::chrono::steady_clock::now();
        double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        std::ostringstream payload;
        payload << "{\"ok\":" << buildJsonArray(ok)
                << ",\"failed\":" << buildJsonArray(failed)
                << ",\"errors\":" << buildJsonObject(errors)
                << ",\"torque\":" << buildJsonObject(torque)
                << ",\"hw_errors\":" << buildJsonObject(hw_errors)
                << ",\"temps\":" << buildJsonObject(temps)
                << ",\"total\":" << (ok.size() + failed.size())
                << ",\"duration_ms\":" << std::fixed << std::setprecision(1) << duration_ms
                << "}";

        response->success = true;
        response->message = payload.str();
      });
  (void) health_service;

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
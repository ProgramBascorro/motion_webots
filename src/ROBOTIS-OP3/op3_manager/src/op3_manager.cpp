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
#include <cstdio>
#include <cstdlib>
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

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_enable_ctrl_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr g_action_page_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_demo_command_pub;
std::mutex g_health_mutex;

/* Give up on a startup error without running the static destructors: the RobotisController
 * singleton is destroyed after rclcpp has shut down, so a plain return from main turns the
 * error into a SIGSEGV/SIGBUS (launch reports "exit code -11") that hides the message. */
[[noreturn]] void exitOnStartupError()
{
  fflush(NULL);
  std::_Exit(1);
}

void goToInitActionPage()
{
  const int kInitPosePageNum = 2;

  std_msgs::msg::String mode_msg;
  mode_msg.data = "action_module";
  g_enable_ctrl_pub->publish(mode_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(500));

  std_msgs::msg::Int32 page_msg;
  page_msg.data = kInitPosePageNum;
  g_action_page_pub->publish(page_msg);
}

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

    // go to init pose : action_module page 2 (WALKING_READY)
    goToInitActionPage();
    RCLCPP_INFO(controller->get_logger(), "Go to init pose (action page 2: WALKING_READY)");
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
  node->declare_parameter<int>("baud_rate", BAUD_RATE);

  node->get_parameter("offset_file_path", g_offset_file);
  node->get_parameter("robot_file_path", g_robot_file);
  node->get_parameter("init_file_path", g_init_file);
  node->get_parameter("device_name", g_device_name);
  node->get_parameter("baud_rate", g_baudrate);

  auto button_sub = node->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 1, buttonHandlerCallback);
  auto dxl_torque_sub = node->create_subscription<std_msgs::msg::String>("/robotis/dxl_torque", 1, dxlTorqueCheckCallback);
  g_init_pose_pub = node->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose", 0);
  g_enable_ctrl_pub = node->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 10);
  g_action_page_pub = node->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  g_demo_command_pub = node->create_publisher<std_msgs::msg::String>("/ball_tracker/command", 10);

  node->declare_parameter<bool>("simulation", false);
  node->get_parameter("simulation", controller->gazebo_mode_);
  g_is_simulation = controller->gazebo_mode_;

  /* real robot */
  if (g_is_simulation == false)
  {
    // The Dynamixel bus takes one master only; sharing it corrupts both sides' packets
    // and only shows up later as "first bulk read fail!!". Wait for the other node to go
    // away rather than dying instantly - a forgotten op3_action_editor is the usual
    // cause, and repeating the warning keeps it readable while the rest of the launch
    // floods the terminal with its own start-up output.
    const int kPortWaitSeconds = 30;
    const int kPortPollSeconds = 2;
    for (int waited = 0;; waited += kPortPollSeconds)
    {
      std::string port_user = RobotisController::findPortUser(g_device_name);
      if (port_user.empty() == true)
        break;

      if (waited >= kPortWaitSeconds)
      {
        RCLCPP_ERROR(node->get_logger(), "%s is STILL in use by %s - giving up.",
                     g_device_name.c_str(), port_user.c_str());
        exitOnStartupError();
      }

      RCLCPP_WARN(node->get_logger(), "%s is in use by %s - stop it to let op3_manager start (%ds left).",
                  g_device_name.c_str(), port_user.c_str(), kPortWaitSeconds - waited);
      rclcpp::sleep_for(std::chrono::seconds(kPortPollSeconds));
    }

    // open port
    PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler(g_device_name.c_str());
    bool set_port_result = port_handler->setBaudRate(g_baudrate);
    if (set_port_result == false)
      RCLCPP_ERROR(node->get_logger(), "Error Set port");

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
    exitOnStartupError();
  }

  // initialize robot
  if (controller->initialize(g_robot_file, g_init_file) == false)
  {
    RCLCPP_ERROR(node->get_logger(), "ROBOTIS Controller Initialize Fail!");
    exitOnStartupError();
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

  // go to init pose : action_module page 2 (WALKING_READY)
  goToInitActionPage();
  RCLCPP_INFO(node->get_logger(), "Go to init pose (action page 2: WALKING_READY)");

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

        for (const auto& it : controller->robot_->dxls_)
        {
          const std::string& joint_name = it.first;
          Dynamixel *dxl = it.second;
          int result = controller->ping(joint_name);
          if (result == COMM_SUCCESS)
          {
            ok.push_back(joint_name);
          }
          else
          {
            failed.push_back(joint_name);
            dynamixel::PacketHandler *handler = dynamixel::PacketHandler::getPacketHandler(dxl->protocol_version_);
            errors[joint_name] = handler->getTxRxResult(result);
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
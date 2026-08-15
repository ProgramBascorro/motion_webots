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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "op3_demo/soccer_demo.h"

namespace robotis_op
{

SoccerDemo::SoccerDemo()
  : // Node("soccer_demo"),
    is_start_soccer_running_(false),
    FALL_FORWARD_LIMIT(60),
    FALL_BACK_LIMIT(-60),
    SPIN_RATE(30),
    DEBUG_PRINT(false),
    wait_count_(0),
    on_following_ball_(false),
    on_tracking_ball_(false),
    restart_soccer_(false),
    start_following_(false),
    stop_following_(false),
    stop_fallen_check_(false),
    robot_status_(Waited),
    stand_state_(Stand),
    tracking_status_(BallTracker::Waiting),
    present_pitch_(0)
{
  //init ros
  enable_ = false;

  std::string default_path = ament_index_cpp::get_package_share_directory("op3_gui_demo") + "/config/gui_config.yaml";
  std::string path = default_path;
  // this->declare_parameter<std::string>("demo_config", default_path);
  // this->get_parameter<std::string>("demo_config", path);
  parseJointNameFromYaml(path);

  // subscriber & publisher
  // module_control_pub_ = this->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 10);
  // motion_index_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  // rgb_led_pub_ = this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);

  // button_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/open_cr/button", 10, std::bind(&SoccerDemo::buttonHandlerCallback, this, std::placeholders::_1));
  // demo_command_sub_ = this->create_subscription<std_msgs::msg::String>("/robotis/demo_command", 10, std::bind(&SoccerDemo::demoCommandCallback, this, std::placeholders::_1));
  // imu_data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/robotis/open_cr/imu", 10, std::bind(&SoccerDemo::imuDataCallback, this, std::placeholders::_1));

  // is_running_client_ = this->create_client<op3_action_module_msgs::srv::IsRunning>("/robotis/action/is_running");
  // set_joint_module_client_ = this->create_client<robotis_controller_msgs::srv::SetJointModule>("/robotis/set_present_joint_ctrl_modules");

  // test_pub_ = this->create_publisher<std_msgs::msg::String>("/debug_text", 10);

  // process_thread_ = std::thread(&SoccerDemo::processThread, this);
  // tracking_thread_ = std::thread(&SoccerDemo::trackingThread, this);

  // this->declare_parameter<bool>("grass_demo", false);
  // this->get_parameter<bool>("grass_demo", is_grass_);
  is_grass_ = false;
}

SoccerDemo::~SoccerDemo()
{
  // if (spin_thread_ && spin_thread_->joinable())
  //   spin_thread_->join();
  // if (process_thread_.joinable())
  //   process_thread_.join();
  // if (tracking_thread_.joinable())
  //   tracking_thread_.join();
}

void SoccerDemo::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ != nullptr)
  {
    imu_data_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("/robotis/open_cr/imu", 10,
                                                                      std::bind(&SoccerDemo::imuDataCallback, this, std::placeholders::_1));
    ball_tracker_.setNode(node);
    ball_follower_.setNode(node);
    head_tracker_cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/ball_tracker/command", 10);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Node is not set");
  }
}

void SoccerDemo::setUsingHeadControl(bool use_head_control)
{
  ball_tracker_.setUsingHeadControl(use_head_control);
}

void SoccerDemo::setDemoEnable()
{
  enable_ = true;
  // Head start publish is owned by startSoccerMode() now (called below) so
  // the start/stop toggle via the button works symmetrically. Previously
  // head was started here but on START button re-toggle (stop→start)
  // startSoccerMode() didn't republish head start, leaving the head stuck
  // in "stopped" state while the legs walked.
  startSoccerMode();
}

void SoccerDemo::setDemoDisable()
{
  if (head_tracker_cmd_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "stop";
    head_tracker_cmd_pub_->publish(cmd);
  }
  // handle disable procedure
  ball_tracker_.stopTracking();
  ball_follower_.stopFollowing();

  enable_ = false;
  wait_count_ = 0;
  on_following_ball_ = false;
  on_tracking_ball_ = false;
  restart_soccer_ = false;
  start_following_ = false;
  stop_following_ = false;
  stop_fallen_check_ = false;

  tracking_status_ = BallTracker::Waiting;
  // if (spin_thread_ && spin_thread_->joinable())
  //   spin_thread_->join();
  if (imu_data_sub_)
    imu_data_sub_.reset();
}

void SoccerDemo::process()
{
  if(enable_ == false)
    return;

  // check to start
  if (start_following_ == true)
  {
    ball_tracker_.startTracking();
    ball_follower_.startFollowing();
    start_following_ = false;

    // Short settle after Start before the robot begins tracking/following, so
    // it reacts quickly to the Start button (was 1 * SPIN_RATE = ~1s,
    // now ~0.3s). Matched from CHRONUS_NEW.
    wait_count_ = SPIN_RATE * 3 / 10;
  }

  // check to stop
  if (stop_following_ == true)
  {
    ball_tracker_.stopTracking();
    ball_follower_.stopFollowing();
    stop_following_ = false;

    wait_count_ = 0;
  }

  if (wait_count_ <= 0)
  {
    // ball following
    if (on_following_ball_ == true)
    {
      switch(tracking_status_)
      {
      case BallTracker::Found:
        ball_follower_.processFollowing(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall(), 0.0);
        break;

      case BallTracker::NotFound:
        ball_follower_.waitFollowing();
        break;

      default:
        break;
      }
    }

    if (on_tracking_ball_ == true)
    {
      // ball tracking
      int tracking_status;

      tracking_status = ball_tracker_.processTracking();

      // set led
      switch(tracking_status)
      {
      case BallTracker::Found:
        if(tracking_status_ != tracking_status)
          setRGBLED(0x1F, 0x1F, 0x1F);
        break;

      case BallTracker::NotFound:
        if(tracking_status_ != tracking_status)
          setRGBLED(0, 0, 0);
        break;

      default:
        break;
      }

      if(tracking_status != tracking_status_)
        tracking_status_ = tracking_status;
    }

    // check fallen states
    switch (stand_state_)
    {
    case Stand:
    {
      // check restart soccer
      if (restart_soccer_ == true)
      {
        restart_soccer_ = false;
        startSoccerMode();
        break;
      }

      // check states for kick
//      int ball_position = ball_follower_.getBallPosition();
      bool in_range = ball_follower_.isBallInRange();

      if(in_range == true)
      {
        ball_follower_.stopFollowing();
        handleKick();
      }
      break;
    }
      // fallen state : Fallen_Forward, Fallen_Behind
    default:
    {
      ball_follower_.stopFollowing();
      handleFallen(stand_state_);
      break;
    }
    }
  }
  else
  {
    wait_count_ -= 1;
  }
}

// void SoccerDemo::processThread()
// {
//   bool result = false;

//   //set node loop rate
//   rclcpp::Rate loop_rate(SPIN_RATE);

//   ball_tracker_.startTracking();

//   //node loop
//   while (rclcpp::ok())
//   {
//     if (enable_ == true)
//       process();

//     //relax to fit output rate
//     loop_rate.sleep();
//   }
// }

// void SoccerDemo::callbackThread()
// {

//   while (rclcpp::ok())
//   {
//     rclcpp::spin_some(this->get_node_base_interface());

//     std::this_thread::sleep_for(std::chrono::milliseconds(1));
//   }
// }

// void SoccerDemo::trackingThread()
// {

//   //set node loop rate
//   rclcpp::Rate loop_rate(SPIN_RATE);

//   ball_tracker_.startTracking();

//   //node loop
//   while (rclcpp::ok())
//   {

//     if(enable_ == true && on_tracking_ball_ == true)
//     {
//       // ball tracking
//       int tracking_status;

//       tracking_status = ball_tracker_.processTracking();

//       // set led
//       switch(tracking_status)
//       {
//       case BallTracker::Found:
//         if(tracking_status_ != tracking_status)
//           setRGBLED(0x1F, 0x1F, 0x1F);
//         break;

//       case BallTracker::NotFound:
//         if(tracking_status_ != tracking_status)
//           setRGBLED(0, 0, 0);
//         break;

//       default:
//         break;
//       }

//       if(tracking_status != tracking_status_)
//         tracking_status_ = tracking_status;
//     }
//     //relax to fit output rate
//     loop_rate.sleep();
//   }
// }

void SoccerDemo::setBodyModuleToDemo(const std::string &body_module, bool with_head_control)
{
  if(node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Node is not set, cannot set body module");
    return;
  }

  robotis_controller_msgs::msg::JointCtrlModule control_msg;

  std::string head_module = "head_control_module";
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    // check whether joint name contains "head"
    if (joint_iter->second.find("head") != std::string::npos)
    {
      if (with_head_control == true)
      {
        control_msg.joint_name.push_back(joint_iter->second);
        control_msg.module_name.push_back(head_module);
      }
      else
        continue;
    }
    else
    {
      control_msg.joint_name.push_back(joint_iter->second);
      control_msg.module_name.push_back(body_module);
    }
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  callServiceSettingModule(control_msg);
  RCLCPP_INFO(rclcpp::get_logger("SoccerDemo"), "enable module of body : %s", body_module.c_str());
}

void SoccerDemo::setModuleToDemo(const std::string &module_name)
{
  if(enable_ == false || node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Node is not set or demo is disabled, cannot set module");
    return;
  }

  robotis_controller_msgs::msg::JointCtrlModule control_msg;
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->second);
    control_msg.module_name.push_back(module_name);
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  callServiceSettingModule(control_msg);
  std::cout << "SoccerDemo::setModuleToDemo - enable module : " << module_name << std::endl;
}

void SoccerDemo::callServiceSettingModule(const robotis_controller_msgs::msg::JointCtrlModule &modules)
{
  auto temp_node = rclcpp::Node::make_shared("soccer_call_service");
  auto set_joint_module_client_ = temp_node->create_client<robotis_controller_msgs::srv::SetJointModule>("/robotis/set_present_joint_ctrl_modules");
  auto request = std::make_shared<robotis_controller_msgs::srv::SetJointModule::Request>();
  request->joint_name = modules.joint_name;
  request->module_name = modules.module_name;

  // 200 ms timeout (was 1 s). robotis_controller's
  // /robotis/set_present_joint_ctrl_modules service is advertised on
  // op3_manager boot — by the time the user can press the START button it's
  // been up for seconds. The old 1 s timeout sat in the START button's
  // critical path before walking_module could even receive its first command.
  if (!set_joint_module_client_->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "[SoccerDemo::callServiceSettingModule] Service not available");
    return;
  }

  auto future = set_joint_module_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("SoccerDemo"), "SoccerDemo::callServiceSettingModule(%s) : result : %d", modules.module_name[0].c_str(), future.get()->result);
  }
}

void SoccerDemo::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node _id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = _id_sub_node.begin(); _it != _id_sub_node.end(); ++_it)
  {
    int _id;
    std::string _joint_name;

    _id = _it->first.as<int>();
    _joint_name = _it->second.as<std::string>();

    id_joint_table_[_id] = _joint_name;
    joint_id_table_[_joint_name] = _id;
  }
}

// joint id -> joint name
bool SoccerDemo::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table_.find(id);
  if (_iter == id_joint_table_.end())
    return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool SoccerDemo::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table_.find(joint_name);
  if (_iter == joint_id_table_.end())
    return false;

  id = _iter->second;
  return true;
}

int SoccerDemo::getJointCount()
{
  return joint_id_table_.size();
}

bool SoccerDemo::isHeadJoint(const int &id)
{
  std::map<std::string, int>::iterator _iter;

  for (_iter = joint_id_table_.begin(); _iter != joint_id_table_.end(); ++_iter)
  {
    if (_iter->first.find("head") != std::string::npos)
      return true;
  }

  return false;
}

void SoccerDemo::buttonHandlerCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    if (on_following_ball_ == true)
    {
      stopSoccerMode();
    }
    else
    {
      // Reject restart during the post-stop debounce window — walking
      // is still finishing its current step and a button press here
      // would otherwise immediately re-engage walking.
      if (std::chrono::steady_clock::now() < stop_debounce_until_)
      {
        RCLCPP_WARN(rclcpp::get_logger("SoccerDemo"),
                    "Ignoring START press: still stopping (debounce)");
        return;
      }
      startSoccerMode();
    }
  }
}

void SoccerDemo::demoCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    if (on_following_ball_ == true)
    {
      stopSoccerMode();
    }
    else
    {
      if (std::chrono::steady_clock::now() < stop_debounce_until_)
      {
        RCLCPP_WARN(rclcpp::get_logger("SoccerDemo"),
                    "Ignoring start cmd: still stopping (debounce)");
        return;
      }
      startSoccerMode();
    }
  }
  else if (msg->data == "stop")
  {
    stopSoccerMode();
  }
}

// check fallen states
void SoccerDemo::imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (enable_ == false || node_ == nullptr)
    return;

  if (stop_fallen_check_ == true)
    return;

  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / M_PI);

  if(DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("SoccerDemo"), "Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

  double pitch = rpy_orientation.coeff(1, 0);

  double alpha = 0.4;
  if (present_pitch_ == 0)
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT)
    stand_state_ = Fallen_Forward;
  else if (present_pitch_ < FALL_BACK_LIMIT)
    stand_state_ = Fallen_Behind;
  else
    stand_state_ = Stand;
}

void SoccerDemo::startSoccerMode()
{
  if (is_start_soccer_running_ == true)
    return;

  is_start_soccer_running_ = true;

  // Switch the controller to walking_module FIRST. This is the slow path
  // (service call, ~100-300 ms) so we get it underway before we tell the
  // head tracker to scan — the head tracker is fire-and-forget and
  // starts SCAN-ing within ~30 ms, so order matters for the user-visible
  // "both started together" feel.
  setBodyModuleToDemo("walking_module");

  if (head_tracker_cmd_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "start";
    head_tracker_cmd_pub_->publish(cmd);
  }

  RCLCPP_INFO(rclcpp::get_logger("SoccerDemo"), "Start Soccer Demo");
  on_following_ball_ = true;
  on_tracking_ball_ = true;
  start_following_ = true;

  is_start_soccer_running_ = false;
}

void SoccerDemo::stopSoccerMode()
{
  RCLCPP_INFO(rclcpp::get_logger("SoccerDemo"), "Stop Soccer Demo");

  // Sequence the stop so it FEELS immediate (~50 ms perceived) instead of
  // waiting for walking_module to finish its current step (~750 ms).
  //
  // 1. Force walking amplitudes to zero FIRST. walking_module reads these
  //    next gait cycle and decelerates from amplitude → 0 instead of
  //    completing a full forward step. Visible effect: feet stop swinging
  //    almost immediately while body finishes settling.
  // 2. Publish walking "stop" command. Tells walking_module to wind down.
  // 3. Tell head_tracker to stop + park head at neutral (head_tracking_node
  //    now sends a final pan=0 / tilt=forward command on "stop" so the
  //    last in-flight SCAN command doesn't keep the head moving).
  // 4. Reset internal flags.

  ball_tracker_.stopTracking();
  ball_follower_.stopFollowing();  // publishes walking "stop" + sets amplitudes via setWalkingParam(0,0,0) next tick

  if (head_tracker_cmd_pub_)
  {
    std_msgs::msg::String cmd;
    cmd.data = "stop";
    head_tracker_cmd_pub_->publish(cmd);
  }

  on_following_ball_ = false;
  on_tracking_ball_ = false;
  // Set stop_following_ = true so the process() loop re-asserts the stop on the
  // next tick (~33 ms later). This double-publish protects against the
  // throwaway-publisher race in ball_follower (auto pub = create_publisher;
  // pub->publish; pub goes out of scope before DDS discovery completes →
  // message can be silently dropped on the first try). Matched from CHRONUS_NEW.
  stop_following_ = true;

  // Debounce window: 800 ms, comfortably over one gait cycle (param.yaml
  // period_time is 500 ms) so it still covers a slower gait that was tuned up.
  // Long enough to let walking_module fully wind down its last step,
  // short enough that a deliberate restart press feels responsive.
  // Bigger window (1200 ms) caused the press to feel "ignored".
  stop_debounce_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
}

void SoccerDemo::handleKick(int ball_position)
{
  rclcpp::sleep_for(std::chrono::milliseconds(1500));

  // change to motion module
  setModuleToDemo("action_module");

  if (handleFallen(stand_state_) == true || enable_ == false)
    return;

  // kick motion
  switch (ball_position)
  {
  case robotis_op::BallFollower::OnRight:
    std::cout << "Kick Motion [R]: " << ball_position << std::endl;
    playMotion(is_grass_ ? RightKick + ForGrass : RightKick);
    break;

  case robotis_op::BallFollower::OnLeft:
    std::cout << "Kick Motion [L]: " << ball_position << std::endl;
    playMotion(is_grass_ ? LeftKick + ForGrass : LeftKick);
    break;

  default:
    break;
  }

  on_following_ball_ = false;
  restart_soccer_ = true;
  tracking_status_ = BallTracker::NotFound;
  ball_follower_.clearBallPosition();

  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  if (handleFallen(stand_state_) == true)
    return;

  // ceremony
  //playMotion(Ceremony);
}

void SoccerDemo::handleKick()
{
  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  // change to motion module
  setModuleToDemo("action_module");

  if (handleFallen(stand_state_) == true || enable_ == false)
    return;

  // kick motion
  ball_follower_.decideBallPositin(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall());
  int ball_position = ball_follower_.getBallPosition();
  if(ball_position == BallFollower::NotFound || ball_position == BallFollower::OutOfRange)
  {
    on_following_ball_ = false;
    restart_soccer_ = true;
    tracking_status_ = BallTracker::NotFound;
    ball_follower_.clearBallPosition();
    return;
  }

  switch (ball_position)
  {
  case robotis_op::BallFollower::OnRight:
    std::cout << "Kick Motion [R]: " << ball_position << std::endl;
    sendDebugTopic("Kick the ball using Right foot");
    playMotion(is_grass_ ? RightKick + ForGrass : RightKick);
    break;

  case robotis_op::BallFollower::OnLeft:
    std::cout << "Kick Motion [L]: " << ball_position << std::endl;
    sendDebugTopic("Kick the ball using Left foot");
    playMotion(is_grass_ ? LeftKick + ForGrass : LeftKick);
    break;

  default:
    break;
  }

  on_following_ball_ = false;
  restart_soccer_ = true;
  tracking_status_ = BallTracker::NotFound;
  ball_follower_.clearBallPosition();

  rclcpp::sleep_for(std::chrono::milliseconds(2000));

  if (handleFallen(stand_state_) == true)
    return;

  // ceremony
  //playMotion(Ceremony);
}

bool SoccerDemo::handleFallen(int fallen_status)
{
  if (fallen_status == Stand)
  {
    return false;
  }

  // change to motion module
  setModuleToDemo("action_module");

  // getup motion
  switch (fallen_status)
  {
  case Fallen_Forward:
    std::cout << "Getup Motion [F]: " << std::endl;
    playMotion(is_grass_ ? GetUpFront + ForGrass : GetUpFront);
    break;

  case Fallen_Behind:
    std::cout << "Getup Motion [B]: " << std::endl;
    playMotion(is_grass_ ? GetUpBack + ForGrass : GetUpBack);
    break;

  default:
    break;
  }

  while(isActionRunning() == true)
    rclcpp::sleep_for(std::chrono::milliseconds(100));

  rclcpp::sleep_for(std::chrono::milliseconds(650));

  if (on_following_ball_ == true)
    restart_soccer_ = true;

  // reset state
  on_following_ball_ = false;

  return true;
}

void SoccerDemo::playMotion(int motion_index)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Node is not set, cannot play motion");
    return;
  }

  auto motion_index_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  std_msgs::msg::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_->publish(motion_msg);
}

void SoccerDemo::setRGBLED(int blue, int green, int red)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Node is not set, cannot set RGB LED");
    return;
  }

  auto rgb_led_pub_ = node_->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::msg::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_->publish(syncwrite_msg);
}

// check running of action
bool SoccerDemo::isActionRunning()
{
  auto temp_node = rclcpp::Node::make_shared("soccer_is_running");
  auto is_running_client_ = temp_node->create_client<op3_action_module_msgs::srv::IsRunning>("/robotis/action/is_running");
  auto request = std::make_shared<op3_action_module_msgs::srv::IsRunning::Request>();
  bool request_result = true;

  if (!is_running_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Failed to get action status: Service not available");
    return request_result;
  }

  auto future = is_running_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    RCLCPP_INFO(rclcpp::get_logger("SoccerDemo"), "SoccerDemo::isActionRunning - is running : %d", result->is_running);
    return result->is_running;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Failed to get action status: Service call failed (no result)");
    return true;
  }

  return request_result;
}

void SoccerDemo::sendDebugTopic(const std::string &msgs)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoccerDemo"), "Node is not set, cannot send debug topic");
    return;
  }

  auto test_pub_ = node_->create_publisher<std_msgs::msg::String>("/debug_text", 10);
  std_msgs::msg::String debug_msg;
  debug_msg.data = msgs;

  test_pub_->publish(debug_msg);
}

}

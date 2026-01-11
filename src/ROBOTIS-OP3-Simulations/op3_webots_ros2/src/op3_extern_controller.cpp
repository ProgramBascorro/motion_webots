#include "op3_webots_ros2/op3_extern_controller.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Speaker.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

namespace robotis_op
{

struct gains
{
  double p_gain;
  double i_gain;
  double d_gain;
  bool initialized;
};

std::string op3_joint_names[20] = {
    "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
    "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
    "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee",
    "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
    "head_pan", "head_tilt"};

std::string webots_joint_names[20] = {
    "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */, "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, 
    "PelvYR" /*ID7 */, "PelvYL" /*ID8 */, "PelvR" /*ID9 */, "PelvL" /*ID10*/,
    "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/, "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, 
    "AnkleR" /*ID15*/, "AnkleL" /*ID16*/, "FootR" /*ID17*/, "FootL" /*ID18*/, 
    "Neck" /*ID19*/, "Head" /*ID20*/
};

gains joint_gains[20];

}

using namespace std;
using namespace robotis_op;

OP3ExternController::OP3ExternController() : Node("op3_webots_extern_controller")
{
  time_step_ms_  = 8;
  time_step_sec_ = 0.008;

  current_time_sec_ = 0; // control time

  // for motor angle
  for (int i = 0; i < N_MOTORS; i++)
  {
    desired_joint_angle_rad_[i] = 0;
    current_joint_angle_rad_[i] = 0;
    current_joint_torque_Nm_[i] = 0;
  }  

  // center of mass
  for (int i = 0; i < 3; i++)
  {
    current_com_m_[i]       = 0;
    previous_com_m_[i]      = 0;
    current_com_vel_mps_[i] = 0;
  }

  // odometry
  odom_initialized_ = false;
  initial_pose_x_ = 0.0;
  initial_pose_y_ = 0.0;
  initial_yaw_ = 0.0;
  odom_msg_.pose.covariance.fill(0.0);
  odom_msg_.twist.covariance.fill(0.0);

}

OP3ExternController::~OP3ExternController()
{
  queue_thread_.join();
}

void OP3ExternController::initialize(std::string gain_file_path)
{
  parsePIDGainYAML(gain_file_path);

  // get basic time step 
  time_step_ms_ = getBasicTimeStep();
  time_step_sec_ = time_step_ms_ * 0.001;
  
  // initialize webots' devices
  head_led_ = getLED("HeadLed");
  body_led_ = getLED("BodyLed");
  camera_ = getCamera("Camera");
  
  gyro_ = getGyro("Gyro");
  acc_  = getAccelerometer("Accelerometer");
  iu_ = getInertialUnit("inertial unit");

  speaker_ = getSpeaker("Speaker");
  //key_board_ = getKeyboard();
  
  gyro_->enable(time_step_ms_);
  acc_->enable(time_step_ms_);
  iu_->enable(time_step_ms_);
  camera_->enable(time_step_ms_);
  //key_board_->enable(time_step_ms_);

  // initialize image_data_ and camera_info
  image_data_.header.stamp = this->get_clock()->now();
  image_data_.header.frame_id = "cam_link";
  image_data_.width  = camera_->getWidth();
  image_data_.height = camera_->getHeight();
  image_data_.is_bigendian = false;
  image_data_.step = sizeof(uint8_t) * 4 * camera_->getWidth();
  image_data_.data.resize(4*camera_->getWidth()*camera_->getHeight());
  image_data_.encoding = sensor_msgs::image_encodings::BGRA8;

  camera_info_msg_.header.stamp = this->get_clock()->now();
  camera_info_msg_.header.frame_id = "cam_link";
  camera_info_msg_.width = camera_->getWidth();
  camera_info_msg_.height = camera_->getHeight();
  camera_info_msg_.distortion_model = "plumb_bob"; // need to check what plumb_bob is

  double forcal_length = camera_->getWidth() / (2 * tan(camera_->getFov() * 0.5));
  camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  camera_info_msg_.r = {1.0, 0.0, 0.0, 
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
  camera_info_msg_.k = {forcal_length, 0.0,           camera_->getWidth() * 0.5,
                        0.0,           forcal_length, camera_->getHeight() * 0.5,
                        0.0,           0.0,           1.0};
  camera_info_msg_.p = {forcal_length,
                        0.0,
                        camera_->getWidth() * 0.5,
                        0.0,
                        0.0,
                        forcal_length,
                        camera_->getHeight() * 0.5,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                        0.0};

  // initialize motors
  for (int i = 0; i < N_MOTORS; i++) {
    // get motors
    motors_[i] = getMotor(webots_joint_names[i]);

    // enable torque feedback
    motors_[i]->enableTorqueFeedback(time_step_ms_);

    if (joint_gains[i].initialized == true)
      motors_[i]->setControlPID(joint_gains[i].p_gain, joint_gains[i].i_gain, joint_gains[i].d_gain);

    // initialize encoders
    std::string sensorName = webots_joint_names[i];
    sensorName.push_back('S');
    encoders_[i] = getPositionSensor(sensorName);
    encoders_[i]->enable(time_step_ms_);
  }

  // set publishers and subscribers and spin
  queue_thread_ = std::thread(&OP3ExternController::queueThread, this);
}

void OP3ExternController::process()
{
  setDesiredJointAngles();

  getPresentJointAngles();
  getPresentJointTorques();
  getCurrentRobotCOM();
  getIMUOutput();

  publishPresentJointStates();
  publishIMUOutput();
  publishCOMData();
  publishOdometry();
  //publishCameraData();

  stepWebots();
}

void OP3ExternController::setDesiredJointAngles()
{
  for (int joint_idx = 0; joint_idx < N_MOTORS; joint_idx++)
  {
    motors_[joint_idx]->setPosition(desired_joint_angle_rad_[joint_idx]);
  }
}

void OP3ExternController::getPresentJointAngles()
{
  for (int joint_idx = 0; joint_idx < N_MOTORS; joint_idx++)
  {
    current_joint_angle_rad_[joint_idx] = encoders_[joint_idx]->getValue();
  }
}

void OP3ExternController::getPresentJointTorques()
{
  for (int joint_idx = 0; joint_idx < N_MOTORS; joint_idx++)
  {
    current_joint_torque_Nm_[joint_idx] = motors_[joint_idx]->getTorqueFeedback();
  }
}

void OP3ExternController::getCurrentRobotCOM()
{
  const double* com = this->getSelf()->getCenterOfMass();

  previous_com_m_[0] = current_com_m_[0];
  previous_com_m_[1] = current_com_m_[1];
  previous_com_m_[2] = current_com_m_[2];
  
  current_com_m_[0] = com[0];
  current_com_m_[1] = com[1];
  current_com_m_[2] = com[2];

  current_com_vel_mps_[0] = (current_com_m_[0] - previous_com_m_[0]) / time_step_sec_;
  current_com_vel_mps_[1] = (current_com_m_[1] - previous_com_m_[1]) / time_step_sec_;
  current_com_vel_mps_[2] = (current_com_m_[2] - previous_com_m_[2]) / time_step_sec_;

  com_m_.x = current_com_m_[0];
  com_m_.y = current_com_m_[1];
  com_m_.z = current_com_m_[2];
}

void OP3ExternController::getIMUOutput()
{
  const double* gyro_rps = gyro_->getValues();
  const double* acc_mps2 = acc_->getValues();
  const double* quat = iu_->getQuaternion();

  imu_data_.angular_velocity.x = gyro_rps[0];
  imu_data_.angular_velocity.y = gyro_rps[1];
  imu_data_.angular_velocity.z = gyro_rps[2];

  imu_data_.linear_acceleration.x = acc_mps2[0];
  imu_data_.linear_acceleration.y = acc_mps2[1];
  imu_data_.linear_acceleration.z = acc_mps2[2];

  imu_data_.orientation.x = quat[0];
  imu_data_.orientation.y = quat[1];
  imu_data_.orientation.z = quat[2];
  imu_data_.orientation.w = quat[3];
}

void OP3ExternController::publishPresentJointStates()
{
  joint_state_msg_.name.clear();
  joint_state_msg_.position.clear();
  joint_state_msg_.velocity.clear();
  joint_state_msg_.effort.clear();

  joint_state_msg_.header.stamp = rclcpp::Clock().now();
    
  for(int i = 0; i < 20; i++)
  {
    joint_state_msg_.name.push_back(op3_joint_names[i]);
    joint_state_msg_.position.push_back(current_joint_angle_rad_[i]);
    joint_state_msg_.velocity.push_back(0);
    joint_state_msg_.effort.push_back(current_joint_torque_Nm_[i]);
  }

  present_joint_state_publisher_->publish(joint_state_msg_);
}

void OP3ExternController::publishIMUOutput()
{
  imu_data_publisher_->publish(imu_data_);
}

void OP3ExternController::publishCOMData()
{
  com_data_publisher_->publish(com_m_);
}

void OP3ExternController::publishCameraData()
{
  image_data_.header.stamp = this->get_clock()->now();
  camera_info_msg_.header.stamp = image_data_.header.stamp;
  
  if (camera_info_publisher_->get_subscription_count() > 0)
    camera_info_publisher_->publish(camera_info_msg_);

  auto image = camera_->getImage();
    
  if (image)
  {
    memcpy(image_data_.data.data(), image, image_data_.data.size());
    camera_image_publisher_->publish(image_data_);
  }
}

void OP3ExternController::publishOdometry()
{
  // Get ground-truth pose from Supervisor API
  const double* pos = this->getSelf()->getPosition();      // [x, y, z]
  const double* rot = this->getSelf()->getOrientation();   // 3x3 row-major

  // Extract yaw from rotation matrix (Z-up world)
  // Matrix layout row-major: [R00, R01, R02, R10, R11, R12, R20, R21, R22]
  // For Z-up, yaw = atan2(R10, R00) = atan2(rot[3], rot[0])
  double yaw = atan2(rot[3], rot[0]);

  // Initialize on first call (zero odometry at start)
  if (!odom_initialized_) {
    initial_pose_x_ = pos[0];
    initial_pose_y_ = pos[1];
    initial_yaw_ = yaw;
    odom_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Odom initialized at x=%.3f y=%.3f yaw=%.3f",
                initial_pose_x_, initial_pose_y_, initial_yaw_);
  }

  // Compute relative pose in world frame
  double dx = pos[0] - initial_pose_x_;
  double dy = pos[1] - initial_pose_y_;
  double dyaw = yaw - initial_yaw_;

  // Wrap dyaw to [-pi, pi]
  while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
  while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

  // Rotate displacement into initial frame (odom frame)
  double c = cos(-initial_yaw_);
  double s = sin(-initial_yaw_);
  double odom_x = c * dx - s * dy;
  double odom_y = s * dx + c * dy;

  // Create quaternion from yaw
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, dyaw);

  auto now = this->get_clock()->now();

  // Publish nav_msgs/Odometry
  odom_msg_.header.stamp = now;
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base";
  odom_msg_.pose.pose.position.x = odom_x;
  odom_msg_.pose.pose.position.y = odom_y;
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation.x = q.x();
  odom_msg_.pose.pose.orientation.y = q.y();
  odom_msg_.pose.pose.orientation.z = q.z();
  odom_msg_.pose.pose.orientation.w = q.w();
  // Small covariance for ground truth
  odom_msg_.pose.covariance[0] = 0.001;   // x
  odom_msg_.pose.covariance[7] = 0.001;   // y
  odom_msg_.pose.covariance[35] = 0.001;  // yaw
  odom_publisher_->publish(odom_msg_);

  // Broadcast TF: odom -> base
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "odom";
  t.child_frame_id = "base";
  t.transform.translation.x = odom_x;
  t.transform.translation.y = odom_y;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(t);
}

void OP3ExternController::queueThread()
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(this->get_node_base_interface());


  /* Publishers, Subsribers, and Service Clients */
  present_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis_op3/joint_states", 1);
  imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/robotis_op3/imu", 1);
  com_data_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/robotis_op3/com", 1);

  camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/robotis_op3/camera/camera_info", 1);
  camera_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/robotis_op3/camera/image_raw", rclcpp::SensorDataQoS().reliable());

  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr goal_pos_subs[N_MOTORS];

  for (int i = 0; i < N_MOTORS; i++)
  {
    // make subscribers for the joint position topic from robotis framework
    std::string goal_pos_topic_name = "/robotis_op3/" + op3_joint_names[i] + "_position/command";

    std::function<void(const std_msgs::msg::Float64::SharedPtr)> callback = 
         std::bind(&OP3ExternController::posCommandCallback, this, std::placeholders::_1, i);
    goal_pos_subs[i] = this->create_subscription<std_msgs::msg::Float64>(goal_pos_topic_name, 1, callback);
  }

  rclcpp::Rate rate(1000.0 / 8);
  while (rclcpp::ok())
  {
    executor->spin_some();
    this->publishCameraData();
    rate.sleep();
  }
}

void OP3ExternController::posCommandCallback(const std_msgs::msg::Float64::SharedPtr msg, const int &joint_idx)
{
  desired_joint_angle_rad_[joint_idx] = msg->data;
  desired_joint_angle_rcv_flag_[joint_idx] = true;
}

void OP3ExternController::stepWebots() {
  if (step(time_step_ms_) == -1)
    exit(EXIT_SUCCESS);
}

bool OP3ExternController::parsePIDGainYAML(std::string gain_file_path)
{
  if (gain_file_path == "")
    return false;

  YAML::Node doc;

  try
  {
    doc = YAML::LoadFile(gain_file_path.c_str());
    for (int joint_idx = 0; joint_idx < N_MOTORS; joint_idx++)
    {
      joint_gains[joint_idx].initialized = false;
      YAML::Node gains;
      if (gains = doc[webots_joint_names[joint_idx]])
      {
        joint_gains[joint_idx].p_gain = gains["p_gain"].as<double>();
        joint_gains[joint_idx].i_gain = gains["i_gain"].as<double>();
        joint_gains[joint_idx].d_gain = gains["d_gain"].as<double>();
        joint_gains[joint_idx].initialized = true;
      }
      else
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "there is not pre-defined gains for " << webots_joint_names[joint_idx]);
      }
    }

    return true;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "gain file not found: " << gain_file_path);
    return false;
  }
}
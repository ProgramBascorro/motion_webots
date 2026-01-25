#include "op3_webots_ros2/op3_extern_controller.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>
#include <webots/Speaker.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>

#include <math.h>
#include <cstring>

// =========================
// Namespace & Global Data
// =========================
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
  "head_pan", "head_tilt"
};

std::string webots_joint_names[20] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
  "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL",
  "AnkleR", "AnkleL", "FootR", "FootL",
  "Neck", "Head"
};

gains joint_gains[20];

} 

using namespace robotis_op;

// =========================
// Constructor / Destructor
// =========================
OP3ExternController::OP3ExternController()
: Node("op3_webots_extern_controller")
{
  time_step_ms_ = 8;
  time_step_sec_ = 0.008;
  current_time_sec_ = 0.0;

  for (int i = 0; i < N_MOTORS; i++) {
    desired_joint_angle_rad_[i] = 0.0;
    current_joint_angle_rad_[i] = 0.0;
    current_joint_torque_Nm_[i] = 0.0;
    desired_joint_angle_rcv_flag_[i] = false;
  }

  for (int i = 0; i < 3; i++) {
    current_com_m_[i] = 0.0;
    previous_com_m_[i] = 0.0;
    current_com_vel_mps_[i] = 0.0;
  }
}

OP3ExternController::~OP3ExternController()
{
  if (queue_thread_.joinable())
    queue_thread_.join();
}

// =========================
// Dummy PID Parser 
// =========================
bool OP3ExternController::parsePIDGainYAML(std::string gain_file_path)
{
  for (int i = 0; i < N_MOTORS; i++) {
    joint_gains[i].p_gain = 0.0;
    joint_gains[i].i_gain = 0.0;
    joint_gains[i].d_gain = 0.0;
    joint_gains[i].initialized = false;
  }

  RCLCPP_WARN(this->get_logger(),
              "parsePIDGainYAML() not implemented, using default PID gains.");

  return true;
}

// =========================
// Initialization
// =========================
void OP3ExternController::initialize(std::string gain_file_path)
{
  parsePIDGainYAML(gain_file_path);

  time_step_ms_  = getBasicTimeStep();
  time_step_sec_ = time_step_ms_ * 0.001;

  head_led_ = getLED("HeadLed");
  body_led_ = getLED("BodyLed");
  camera_   = getCamera("Camera");

  gyro_ = getGyro("Gyro");
  acc_  = getAccelerometer("Accelerometer");
  iu_   = getInertialUnit("inertial unit");

  gyro_->enable(time_step_ms_);
  acc_->enable(time_step_ms_);
  iu_->enable(time_step_ms_);
  camera_->enable(time_step_ms_);

  image_data_.header.frame_id = "cam_link";
  image_data_.width  = camera_->getWidth();
  image_data_.height = camera_->getHeight();
  image_data_.encoding = sensor_msgs::image_encodings::BGRA8;
  image_data_.is_bigendian = false;
  image_data_.step = 4 * camera_->getWidth();
  image_data_.data.resize(4 * camera_->getWidth() * camera_->getHeight());

  camera_info_msg_.header.frame_id = "cam_link";
  camera_info_msg_.width  = camera_->getWidth();
  camera_info_msg_.height = camera_->getHeight();
  camera_info_msg_.distortion_model = "plumb_bob";

  double focal_length =
    camera_->getWidth() / (2.0 * tan(camera_->getFov() * 0.5));

  camera_info_msg_.k = {
    focal_length, 0.0, camera_->getWidth() * 0.5,
    0.0, focal_length, camera_->getHeight() * 0.5,
    0.0, 0.0, 1.0
  };

  for (int i = 0; i < N_MOTORS; i++) {
    motors_[i] = getMotor(webots_joint_names[i]);
    motors_[i]->enableTorqueFeedback(time_step_ms_);

    std::string sensorName = webots_joint_names[i] + "S";
    encoders_[i] = getPositionSensor(sensorName);
    encoders_[i]->enable(time_step_ms_);
  }

  queue_thread_ = std::thread(&OP3ExternController::queueThread, this);
}

// =========================
// Main Process
// =========================
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
  publishCameraData();
  publishGroundTruth();

  stepWebots();
}

// =========================
// Ground Truth
// =========================
void OP3ExternController::publishGroundTruth()
{
  if (!gt_odom_pub_ || !tf_broadcaster_)
    return;

  const double* pos = this->getSelf()->getPosition();
  const double* rot = this->getSelf()->getOrientation();

  double yaw = atan2(rot[3], rot[0]);
  double qz = sin(yaw * 0.5);
  double qw = cos(yaw * 0.5);

  auto now = this->get_clock()->now();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = pos[0];
  odom.pose.pose.position.y = pos[1];
  odom.pose.pose.position.z = pos[2];
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  gt_odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";

  t.transform.translation.x = pos[0];
  t.transform.translation.y = pos[1];
  t.transform.translation.z = pos[2];
  t.transform.rotation.z = qz;
  t.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(t);
}

// =========================
// ROS Thread
// =========================
void OP3ExternController::queueThread()
{
  auto executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor->add_node(this->get_node_base_interface());

  present_joint_state_publisher_ =
    this->create_publisher<sensor_msgs::msg::JointState>(
      "/robotis_op3/joint_states", 1);

  imu_data_publisher_ =
    this->create_publisher<sensor_msgs::msg::Imu>(
      "/robotis_op3/imu", 1);

  com_data_publisher_ =
    this->create_publisher<geometry_msgs::msg::Vector3>(
      "/robotis_op3/com", 1);

  camera_info_publisher_ =
    this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/robotis_op3/camera/camera_info", 1);

  camera_image_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>(
      "/robotis_op3/camera/image_raw",
      rclcpp::SensorDataQoS().reliable());

  gt_odom_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(
      "/ground_truth/odom", 10);

  tf_broadcaster_ =
    std::make_shared<tf2_ros::TransformBroadcaster>(this);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr goal_pos_subs[N_MOTORS];

  for (int i = 0; i < N_MOTORS; i++) {
    std::string topic =
      "/robotis_op3/" + op3_joint_names[i] + "_position/command";

    int joint_idx = i;

    goal_pos_subs[i] =
      this->create_subscription<std_msgs::msg::Float64>(
        topic,
        1,
        [this, joint_idx](const std_msgs::msg::Float64::SharedPtr msg)
        {
          this->posCommandCallback(msg, joint_idx);
        });
  }

  rclcpp::Rate rate(1000.0 / time_step_ms_);
  while (rclcpp::ok()) {
    executor->spin_some();
    rate.sleep();
  }
}

// =========================
// Robot Functions
// =========================
void OP3ExternController::setDesiredJointAngles()
{
  for (int i = 0; i < N_MOTORS; i++)
    motors_[i]->setPosition(desired_joint_angle_rad_[i]);
}

void OP3ExternController::getPresentJointAngles()
{
  for (int i = 0; i < N_MOTORS; i++)
    current_joint_angle_rad_[i] = encoders_[i]->getValue();
}

void OP3ExternController::getPresentJointTorques()
{
  for (int i = 0; i < N_MOTORS; i++)
    current_joint_torque_Nm_[i] = motors_[i]->getTorqueFeedback();
}

void OP3ExternController::getCurrentRobotCOM()
{
  const double* com = this->getSelf()->getCenterOfMass();

  for (int i = 0; i < 3; i++)
    previous_com_m_[i] = current_com_m_[i];

  current_com_m_[0] = com[0];
  current_com_m_[1] = com[1];
  current_com_m_[2] = com[2];

  current_com_vel_mps_[0] =
    (current_com_m_[0] - previous_com_m_[0]) / time_step_sec_;
  current_com_vel_mps_[1] =
    (current_com_m_[1] - previous_com_m_[1]) / time_step_sec_;
  current_com_vel_mps_[2] =
    (current_com_m_[2] - previous_com_m_[2]) / time_step_sec_;

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

// =========================
// Publishers
// =========================
void OP3ExternController::publishPresentJointStates()
{
  joint_state_msg_.header.stamp = this->get_clock()->now();
  joint_state_msg_.name.clear();
  joint_state_msg_.position.clear();
  joint_state_msg_.velocity.clear();
  joint_state_msg_.effort.clear();

  for (int i = 0; i < N_MOTORS; i++) {
    joint_state_msg_.name.push_back(op3_joint_names[i]);
    joint_state_msg_.position.push_back(current_joint_angle_rad_[i]);
    joint_state_msg_.velocity.push_back(0.0);
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

  const unsigned char* image = camera_->getImage();
  if (image) {
    std::memcpy(image_data_.data.data(), image, image_data_.data.size());
    camera_image_publisher_->publish(image_data_);
  }
}

// =========================
// Callback
// =========================
void OP3ExternController::posCommandCallback(
  const std_msgs::msg::Float64::SharedPtr msg,
  int joint_idx)
{
  desired_joint_angle_rad_[joint_idx] = msg->data;
  desired_joint_angle_rcv_flag_[joint_idx] = true;
}

// =========================
// Webots Step
// =========================
void OP3ExternController::stepWebots()
{
  if (step(time_step_ms_) == -1)
    exit(EXIT_SUCCESS);
}

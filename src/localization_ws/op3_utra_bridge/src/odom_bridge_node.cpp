/**
 * @file odom_bridge_node.cpp
 * @brief Improved Odometry Bridge using OP3 Forward Kinematics
 * 
 * Uses op3_kinematics_dynamics to compute accurate foot positions
 * and derives odometry from support foot tracking
 */

#include <memory>
#include <string>
#include <cmath>
#include <map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// OP3 Kinematics
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

class OdometryBridge : public rclcpp::Node
{
public:
  OdometryBridge()
  : Node("odom_bridge")
  {
    // Initialize OP3 Kinematics
    try {
      op3_kinematics_ = std::make_shared<robotis_op::OP3KinematicsDynamics>(
        robotis_op::WholeBody);
      RCLCPP_INFO(this->get_logger(), "OP3 Kinematics initialized successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize kinematics: %s", e.what());
      throw;
    }

    // Determine if using simulation or real robot
    this->declare_parameter("use_sim", false);
    use_sim_ = this->get_parameter("use_sim").as_bool();
    
    // Select appropriate joint states topic
    std::string joint_states_topic = use_sim_ ? 
      "/robotis_op3/joint_states" : "/robotis/present_joint_states";
    
    RCLCPP_INFO(this->get_logger(), "Using joint states from: %s", 
                joint_states_topic.c_str());
    
    // Publisher for odometry in UTRA format
    odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "odom_combined", 10);
    
    // Subscriber to OP3 joint states
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, 10,
      std::bind(&OdometryBridge::jointStatesCallback, this, std::placeholders::_1));
    
    // Initialize odometry
    current_pose_.header.frame_id = "odom";
    current_pose_.pose.pose.position.x = 0.0;
    current_pose_.pose.pose.position.y = 0.0;
    current_pose_.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    current_pose_.pose.pose.orientation = tf2::toMsg(q);
    
    // Set covariance (estimates based on foot tracking accuracy)
    current_pose_.pose.covariance[0] = 0.005;  // x variance
    current_pose_.pose.covariance[7] = 0.005;  // y variance
    current_pose_.pose.covariance[14] = 0.001; // z variance
    current_pose_.pose.covariance[21] = 0.002; // roll variance
    current_pose_.pose.covariance[28] = 0.002; // pitch variance
    current_pose_.pose.covariance[35] = 0.01;  // yaw variance
    
    // Initialize support foot tracking
    support_foot_ = SupportFoot::RIGHT; // Assume starts on right foot
    last_update_time_ = this->now();
    initialized_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Odometry Bridge with FK initialized");
  }

private:
  enum class SupportFoot {
    RIGHT,
    LEFT,
    DOUBLE,
    UNKNOWN
  };

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!initialized_) {
      initializeOdometry(msg);
      return;
    }
    
    // Calculate time delta
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    
    // Validate dt to prevent NaN
    if (dt <= 0.0 || dt > 1.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Invalid dt: %.6f seconds, skipping update", dt);
      return;
    }
    
    // Update kinematics with current joint angles
    if (!updateKinematics(msg)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Failed to update kinematics, skipping");
      return;
    }
    
    // Compute foot positions using forward kinematics
    Eigen::Vector3d right_foot_pos = getRightFootPosition();
    Eigen::Vector3d left_foot_pos = getLeftFootPosition();
    
    // Validate foot positions
    if (!isValidVector(right_foot_pos) || !isValidVector(left_foot_pos)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Invalid foot positions detected, resetting odometry");
      resetOdometry();
      return;
    }
    
    // Determine support foot (foot on ground)
    SupportFoot current_support = determineSupportFoot(right_foot_pos, left_foot_pos);
    
    // Calculate odometry based on support foot movement
    Eigen::Vector3d support_foot_delta;
    
    if (current_support == SupportFoot::RIGHT) {
      support_foot_delta = right_foot_pos - prev_right_foot_pos_;
    } else if (current_support == SupportFoot::LEFT) {
      support_foot_delta = left_foot_pos - prev_left_foot_pos_;
    } else {
      // Double support or unknown - use average
      Eigen::Vector3d right_delta = right_foot_pos - prev_right_foot_pos_;
      Eigen::Vector3d left_delta = left_foot_pos - prev_left_foot_pos_;
      support_foot_delta = (right_delta + left_delta) / 2.0;
    }
    
    // Validate support foot delta
    if (!isValidVector(support_foot_delta)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Invalid support foot delta, skipping update");
      return;
    }
    
    // Limit maximum delta to prevent jumps (in meters)
    const double MAX_DELTA = 0.5; // 50cm per update
    support_foot_delta.x() = std::clamp(support_foot_delta.x(), -MAX_DELTA, MAX_DELTA);
    support_foot_delta.y() = std::clamp(support_foot_delta.y(), -MAX_DELTA, MAX_DELTA);
    
    // The support foot shouldn't move relative to ground
    // So robot moved opposite to support foot delta
    double delta_x = -support_foot_delta.x();
    double delta_y = -support_foot_delta.y();
    
    // Calculate orientation change from hip yaw joints
    double delta_theta = calculateYawChange(msg);
    
    // Validate delta_theta
    if (!std::isfinite(delta_theta)) {
      delta_theta = 0.0;
    }
    delta_theta = std::clamp(delta_theta, -M_PI, M_PI);
    
    // Update pose in odom frame
    // Transform delta from robot frame to odom frame
    double cos_theta = std::cos(current_yaw_);
    double sin_theta = std::sin(current_yaw_);
    
    double delta_x_odom = delta_x * cos_theta - delta_y * sin_theta;
    double delta_y_odom = delta_x * sin_theta + delta_y * cos_theta;
    
    current_pose_.pose.pose.position.x += delta_x_odom;
    current_pose_.pose.pose.position.y += delta_y_odom;
    current_yaw_ += delta_theta;
    
    // Normalize yaw to [-pi, pi]
    current_yaw_ = std::atan2(std::sin(current_yaw_), std::cos(current_yaw_));
    
    // Final validation before publishing
    if (!isValidPose()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid pose detected! Resetting odometry.");
      resetOdometry();
      return;
    }
    
    // Update orientation quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw_);
    current_pose_.pose.pose.orientation = tf2::toMsg(q);
    
    // Update timestamp and publish
    current_pose_.header.stamp = current_time;
    odom_pub_->publish(current_pose_);
    
    // Debug output
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Odom: x=%.3f, y=%.3f, yaw=%.3f | Delta: x=%.4f, y=%.4f, yaw=%.4f",
      current_pose_.pose.pose.position.x, current_pose_.pose.pose.position.y, current_yaw_,
      delta_x_odom, delta_y_odom, delta_theta);
    
    // Store for next iteration
    prev_right_foot_pos_ = right_foot_pos;
    prev_left_foot_pos_ = left_foot_pos;
    last_joint_state_ = *msg;
    last_update_time_ = current_time;
    support_foot_ = current_support;
  }
  
  void initializeOdometry(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = *msg;
    
    if (!updateKinematics(msg)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize kinematics");
      return;
    }
    
    prev_right_foot_pos_ = getRightFootPosition();
    prev_left_foot_pos_ = getLeftFootPosition();
    
    if (!isValidVector(prev_right_foot_pos_) || !isValidVector(prev_left_foot_pos_)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid initial foot positions");
      prev_right_foot_pos_ = Eigen::Vector3d::Zero();
      prev_left_foot_pos_ = Eigen::Vector3d::Zero();
    }
    
    initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Odometry initialized");
    RCLCPP_INFO(this->get_logger(), "Initial right foot: [%.3f, %.3f, %.3f]", 
                prev_right_foot_pos_.x(), prev_right_foot_pos_.y(), prev_right_foot_pos_.z());
    RCLCPP_INFO(this->get_logger(), "Initial left foot:  [%.3f, %.3f, %.3f]", 
                prev_left_foot_pos_.x(), prev_left_foot_pos_.y(), prev_left_foot_pos_.z());
  }
  
  bool updateKinematics(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    try {
      // Map joint names to IDs and set angles
      for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];
        double joint_angle = msg->position[i];
        
        // Validate joint angle
        if (!std::isfinite(joint_angle)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Invalid joint angle for %s", joint_name.c_str());
          continue;
        }
        
        // Set joint angle in kinematics
        auto link_data = op3_kinematics_->getLinkData(joint_name);
        if (link_data != nullptr) {
          link_data->joint_angle_ = joint_angle;
        }
      }
      
      // Compute forward kinematics
      op3_kinematics_->calcForwardKinematics(0);
      return true;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Exception in updateKinematics: %s", e.what());
      return false;
    }
  }
  
  Eigen::Vector3d getRightFootPosition()
  {
    // Try multiple possible link names for right foot
    std::vector<std::string> possible_names = {
      "r_ank_roll_link", "r_leg_end", "r_ank_pitch_link", "right_foot"
    };
    
    for (const auto& name : possible_names) {
      auto link = op3_kinematics_->getLinkData(name);
      if (link != nullptr) {
        return link->position_;
      }
    }
    
    RCLCPP_WARN_ONCE(this->get_logger(), 
      "Could not find right foot link, returning zero");
    return Eigen::Vector3d::Zero();
  }
  
  Eigen::Vector3d getLeftFootPosition()
  {
    // Try multiple possible link names for left foot
    std::vector<std::string> possible_names = {
      "l_ank_roll_link", "l_leg_end", "l_ank_pitch_link", "left_foot"
    };
    
    for (const auto& name : possible_names) {
      auto link = op3_kinematics_->getLinkData(name);
      if (link != nullptr) {
        return link->position_;
      }
    }
    
    RCLCPP_WARN_ONCE(this->get_logger(), 
      "Could not find left foot link, returning zero");
    return Eigen::Vector3d::Zero();
  }
  
  SupportFoot determineSupportFoot(const Eigen::Vector3d& right_pos, 
                                   const Eigen::Vector3d& left_pos)
  {
    // Simple heuristic: lower foot is support foot
    const double height_threshold = 0.01; // 1cm threshold
    
    double height_diff = right_pos.z() - left_pos.z();
    
    if (height_diff < -height_threshold) {
      // Right foot lower
      return SupportFoot::RIGHT;
    } else if (height_diff > height_threshold) {
      // Left foot lower
      return SupportFoot::LEFT;
    } else {
      // Both feet at similar height - double support
      return SupportFoot::DOUBLE;
    }
  }
  
  double calculateYawChange(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Calculate yaw change from hip yaw joints
    double right_hip_yaw_change = 0.0;
    double left_hip_yaw_change = 0.0;
    bool found_right = false;
    bool found_left = false;
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& joint_name = msg->name[i];
      
      // Find matching joint in last state
      auto it = std::find(last_joint_state_.name.begin(), 
                         last_joint_state_.name.end(), joint_name);
      if (it == last_joint_state_.name.end()) continue;
      
      size_t last_idx = std::distance(last_joint_state_.name.begin(), it);
      double pos_change = msg->position[i] - last_joint_state_.position[last_idx];
      
      // Validate position change
      if (!std::isfinite(pos_change)) continue;
      
      if (joint_name == "r_hip_yaw") {
        right_hip_yaw_change = pos_change;
        found_right = true;
      } else if (joint_name == "l_hip_yaw") {
        left_hip_yaw_change = pos_change;
        found_left = true;
      }
    }
    
    // Average hip yaw changes for body rotation estimate
    if (found_right && found_left) {
      return (right_hip_yaw_change + left_hip_yaw_change) / 2.0;
    } else if (found_right) {
      return right_hip_yaw_change;
    } else if (found_left) {
      return left_hip_yaw_change;
    }
    
    return 0.0;
  }
  
  bool isValidVector(const Eigen::Vector3d& vec)
  {
    return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z());
  }
  
  bool isValidPose()
  {
    return std::isfinite(current_pose_.pose.pose.position.x) &&
           std::isfinite(current_pose_.pose.pose.position.y) &&
           std::isfinite(current_pose_.pose.pose.position.z) &&
           std::isfinite(current_yaw_);
  }
  
  void resetOdometry()
  {
    current_pose_.pose.pose.position.x = 0.0;
    current_pose_.pose.pose.position.y = 0.0;
    current_pose_.pose.pose.position.z = 0.0;
    current_yaw_ = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    current_pose_.pose.pose.orientation = tf2::toMsg(q);
    
    RCLCPP_WARN(this->get_logger(), "Odometry reset to origin");
  }
  
  // ROS Communication
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  
  // OP3 Kinematics
  std::shared_ptr<robotis_op::OP3KinematicsDynamics> op3_kinematics_;
  
  // State variables
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
  sensor_msgs::msg::JointState last_joint_state_;
  rclcpp::Time last_update_time_;
  double current_yaw_ = 0.0;
  bool initialized_;
  bool use_sim_;
  
  // Foot tracking
  Eigen::Vector3d prev_right_foot_pos_;
  Eigen::Vector3d prev_left_foot_pos_;
  SupportFoot support_foot_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
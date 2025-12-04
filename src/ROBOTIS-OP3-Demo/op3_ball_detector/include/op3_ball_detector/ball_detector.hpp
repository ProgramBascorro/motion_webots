#ifndef OP3_BALL_DETECTOR__BALL_DETECTOR_HPP_
#define OP3_BALL_DETECTOR__BALL_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>

#include "op3_ball_detector/yolo_detector.hpp"

namespace robotis_op
{

class BallDetectorCpp : public rclcpp::Node
{
public:
  BallDetectorCpp();
  virtual ~BallDetectorCpp();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void declareParameters();
  void loadParameters();
  void detectBall(const cv::Mat& image);
  void publishBallPosition(const cv::Point2d& position);
  void publishBallStatus(const std::string& status);
  void publishDebugImage(const cv::Mat& image);
  void drawFPS(cv::Mat& image);
  void drawDetection(cv::Mat& image, const Detection& det);
  
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ball_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  
  std::unique_ptr<YoloDetector> yolo_detector_;
  
  cv::Mat current_image_;
  std::mutex image_mutex_;
  
  std::string camera_topic_;
  std::string model_path_;
  bool publish_debug_image_;
  double confidence_threshold_;
  double nms_threshold_;
  int ball_class_id_;
  int input_width_;
  int input_height_;
  bool use_gpu_;
  private:
  int process_every_n_frames_;
  int frame_counter_;

  // Track previous detection state
  bool prev_ball_detected_ = false;
  bool prev_goal_detected_ = false;
  bool prev_robot_detected_ = false;
};

}  // namespace robotis_op

#endif  // OP3_BALL_DETECTOR__BALL_DETECTOR_HPP_
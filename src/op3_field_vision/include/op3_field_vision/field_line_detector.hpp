#ifndef OP3_FIELD_VISION_FIELD_LINE_DETECTOR_HPP
#define OP3_FIELD_VISION_FIELD_LINE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <algorithm>

namespace op3_field_vision
{

struct Config {
  int hsv_s_max = 60;
  int hsv_v_min = 180;
  int canny_low = 50;
  int canny_high = 150;
  int hough_threshold = 50;
  int min_line_length = 40;
  int max_line_gap = 10;
  int morph_kernel = 3;
  int morph_iters = 1;
  double roi_y_min_ratio = 0.0;
};

class FieldLineDetector : public rclcpp::Node
{
public:
  FieldLineDetector();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectLines(const cv::Mat& bgr_image);
  
  rcl_interfaces::msg::SetParametersResult onParameterChange(
      const std::vector<rclcpp::Parameter>& params);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;

  // Camera intrinsics (stored for later use)
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  bool camera_info_received_;

  // Startup-only parameters
  std::string image_topic_;
  std::string camera_info_topic_;

  // Thread-safe config
  Config config_;
  std::mutex config_mutex_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace op3_field_vision

#endif  // OP3_FIELD_VISION_FIELD_LINE_DETECTOR_HPP

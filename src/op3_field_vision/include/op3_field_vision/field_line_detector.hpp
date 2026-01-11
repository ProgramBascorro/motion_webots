#ifndef OP3_FIELD_VISION_FIELD_LINE_DETECTOR_HPP
#define OP3_FIELD_VISION_FIELD_LINE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <algorithm>

namespace op3_field_vision
{

struct Config {
  // Image processing params
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
  
  // Ground projection params
  double ground_z = 0.0;
  double max_range_m = 5.0;
  int samples_per_segment = 5;
};

class FieldLineDetector : public rclcpp::Node
{
public:
  FieldLineDetector();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void detectLines(const cv::Mat& bgr_image, const builtin_interfaces::msg::Time& stamp);
  
  void projectToGround(const std::vector<cv::Vec4i>& lines,
                       const builtin_interfaces::msg::Time& stamp,
                       int roi_y_offset);
  void publishPointCloud(const std::vector<Eigen::Vector3d>& points,
                         const builtin_interfaces::msg::Time& stamp);
  void publishMarkers(const std::vector<Eigen::Vector3d>& points,
                      const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& segments,
                      const builtin_interfaces::msg::Time& stamp);
  
  rcl_interfaces::msg::SetParametersResult onParameterChange(
      const std::vector<rclcpp::Parameter>& params);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Camera intrinsics
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  bool camera_info_received_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Startup-only parameters
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_frame_;
  std::string camera_frame_;

  // Thread-safe config
  Config config_;
  std::mutex config_mutex_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace op3_field_vision

#endif  // OP3_FIELD_VISION_FIELD_LINE_DETECTOR_HPP

// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OP3_YOLO_VISION__YOLO_DETECTOR_HPP
#define OP3_YOLO_VISION__YOLO_DETECTOR_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

namespace op3_yolo_vision
{

class YoloDetector : public rclcpp::Node
{
public:
  YoloDetector();

private:
  struct Detection
  {
    cv::Rect box;
    int class_id;
    float score;
  };

  struct ClassInfo
  {
    std::string name;
    cv::Scalar color;
    float threshold;
  };

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void greenMaskCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  bool loadModel();
  bool computeRay(const cv::Point2f& pixel, tf2::Vector3& ray_cam) const;
  bool projectToGround(const tf2::Vector3& ray_cam,
                       const tf2::Transform& cam_to_out,
                       tf2::Vector3& out_point) const;

  void updateHorizon(const cv::Mat& mask, const rclcpp::Time& stamp);
  bool horizonAllows(int x, int y, const rclcpp::Time& stamp) const;

  std::vector<Detection> decodeDetections(const cv::Mat& output,
                                          float scale,
                                          int image_width,
                                          int image_height) const;

  sensor_msgs::msg::PointCloud2 buildCloud(const std::vector<tf2::Vector3>& points,
                                           const rclcpp::Time& stamp) const;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string green_mask_topic_;
  std::string output_frame_;
  std::string camera_frame_;
  std::string model_path_;

  int input_size_;
  float nms_threshold_;
  float nms_score_threshold_;
  bool publish_debug_;
  bool use_green_horizon_;
  double ground_z_;
  double max_range_m_;
  double horizon_max_age_sec_;
  int horizon_offset_px_;
  int horizon_stride_;
  int horizon_smooth_window_;
  double min_green_columns_ratio_;

  bool camera_info_received_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  int camera_width_;
  int camera_height_;

  cv::dnn::Net net_;
  bool model_loaded_;
  std::vector<ClassInfo> class_info_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr green_mask_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr balls_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr goals_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr robots_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr intersections_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex horizon_mutex_;
  std::vector<int> horizon_y_;
  bool horizon_valid_;
  rclcpp::Time horizon_stamp_;
};

}  // namespace op3_yolo_vision

#endif  // OP3_YOLO_VISION__YOLO_DETECTOR_HPP

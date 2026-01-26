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
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <soccer_msgs/msg/bounding_boxes.hpp>

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

namespace op3_yolo_vision
{

class YoloDetector : public rclcpp::Node
{
public:
  explicit YoloDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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

  bool loadModel();

  std::vector<Detection> decodeDetections(const cv::Mat& output,
                                          float scale,
                                          int image_width,
                                          int image_height) const;

  std::string image_topic_;
  std::string ball_center_topic_;
  std::string model_path_;
  std::string dnn_backend_;
  std::string dnn_target_;

  int input_size_;
  float nms_threshold_;
  float nms_score_threshold_;
  bool publish_debug_;
  bool use_gpu_;
  bool use_fp16_;
  int frame_skip_;

  cv::dnn::Net net_;
  bool model_loaded_;
  std::vector<ClassInfo> class_info_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<soccer_msgs::msg::BoundingBoxes>::SharedPtr detections_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ball_center_pub_;

  rclcpp::Time last_log_time_;
  int frame_count_;
  int frame_counter_;
  double total_inference_ms_;
  double total_full_ms_;
};

}  // namespace op3_yolo_vision

#endif  // OP3_YOLO_VISION__YOLO_DETECTOR_HPP

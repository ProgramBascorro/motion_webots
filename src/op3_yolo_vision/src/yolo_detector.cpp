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

#include "op3_yolo_vision/yolo_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <limits>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace op3_yolo_vision
{

namespace
{
constexpr int kNumClasses = 6;

std::string resolveModelPath(const std::string& model_path)
{
  if (model_path.empty()) {
    return model_path;
  }
  if (std::filesystem::path(model_path).is_absolute()) {
    return model_path;
  }
  const auto share = ament_index_cpp::get_package_share_directory("op3_yolo_vision");
  return (std::filesystem::path(share) / model_path).string();
}

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

int resolveDnnBackend(const std::string& name)
{
  const std::string key = toLower(name);
  if (key == "cuda") {
    return cv::dnn::DNN_BACKEND_CUDA;
  }
  if (key == "opencv" || key == "default" || key.empty()) {
    return cv::dnn::DNN_BACKEND_OPENCV;
  }
  return cv::dnn::DNN_BACKEND_OPENCV;
}

int resolveDnnTarget(const std::string& name)
{
  const std::string key = toLower(name);
  if (key == "cuda") {
    return cv::dnn::DNN_TARGET_CUDA;
  }
  if (key == "cuda_fp16" || key == "fp16") {
    return cv::dnn::DNN_TARGET_CUDA_FP16;
  }
  if (key == "cpu" || key == "default" || key.empty()) {
    return cv::dnn::DNN_TARGET_CPU;
  }
  return cv::dnn::DNN_TARGET_CPU;
}

sensor_msgs::msg::Image toImageMsg(const cv::Mat& image,
                                   const std_msgs::msg::Header& header,
                                   const std::string& encoding)
{
  sensor_msgs::msg::Image msg;
  msg.header = header;
  msg.height = static_cast<uint32_t>(image.rows);
  msg.width = static_cast<uint32_t>(image.cols);
  msg.encoding = encoding;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);

  const cv::Mat continuous = image.isContinuous() ? image : image.clone();
  const size_t size = continuous.total() * continuous.elemSize();
  msg.data.resize(size);
  std::memcpy(msg.data.data(), continuous.data, size);
  return msg;
}

cv::Mat toBgr(const sensor_msgs::msg::Image& msg)
{
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  if (width == 0 || height == 0 || msg.data.empty()) {
    return cv::Mat();
  }

  const std::string& enc = msg.encoding;
  const uint8_t* data = msg.data.data();

  if (enc == sensor_msgs::image_encodings::BGR8) {
    return cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(data), msg.step);
  }
  if (enc == sensor_msgs::image_encodings::RGB8) {
    cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t*>(data), msg.step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  if (enc == sensor_msgs::image_encodings::RGBA8) {
    cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(data), msg.step);
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    return bgr;
  }
  if (enc == sensor_msgs::image_encodings::BGRA8) {
    cv::Mat bgra(height, width, CV_8UC4, const_cast<uint8_t*>(data), msg.step);
    cv::Mat bgr;
    cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
    return bgr;
  }
  if (enc == sensor_msgs::image_encodings::MONO8) {
    cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t*>(data), msg.step);
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  if (enc == sensor_msgs::image_encodings::MONO16) {
    cv::Mat gray16(height, width, CV_16UC1, const_cast<uint8_t*>(data), msg.step);
    cv::Mat gray8;
    gray16.convertTo(gray8, CV_8U, 1.0 / 256.0);
    cv::Mat bgr;
    cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }

  throw std::runtime_error("Unsupported image encoding: " + enc);
}

cv::Mat toMono8(const sensor_msgs::msg::Image& msg)
{
  const int width = static_cast<int>(msg.width);
  const int height = static_cast<int>(msg.height);
  if (width == 0 || height == 0 || msg.data.empty()) {
    return cv::Mat();
  }

  const std::string& enc = msg.encoding;
  const uint8_t* data = msg.data.data();

  if (enc == sensor_msgs::image_encodings::MONO8) {
    return cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(data), msg.step);
  }
  if (enc == sensor_msgs::image_encodings::MONO16) {
    cv::Mat gray16(height, width, CV_16UC1, const_cast<uint8_t*>(data), msg.step);
    cv::Mat gray8;
    gray16.convertTo(gray8, CV_8U, 1.0 / 256.0);
    return gray8;
  }
  if (enc == sensor_msgs::image_encodings::BGR8) {
    cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8_t*>(data), msg.step);
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }
  if (enc == sensor_msgs::image_encodings::RGB8) {
    cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t*>(data), msg.step);
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
  }
  if (enc == sensor_msgs::image_encodings::RGBA8) {
    cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(data), msg.step);
    cv::Mat gray;
    cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
    return gray;
  }
  if (enc == sensor_msgs::image_encodings::BGRA8) {
    cv::Mat bgra(height, width, CV_8UC4, const_cast<uint8_t*>(data), msg.step);
    cv::Mat gray;
    cv::cvtColor(bgra, gray, cv::COLOR_BGRA2GRAY);
    return gray;
  }

  throw std::runtime_error("Unsupported image encoding: " + enc);
}

}  // namespace

YoloDetector::YoloDetector()
: Node("yolo_detector"),
  input_size_(640),
  nms_threshold_(0.5f),
  nms_score_threshold_(0.1f),
  publish_debug_(true),
  use_green_horizon_(true),
  use_gpu_(false),
  use_fp16_(false),
  ground_z_(0.0),
  max_range_m_(10.0),
  horizon_max_age_sec_(0.25),
  horizon_offset_px_(2),
  horizon_stride_(4),
  horizon_smooth_window_(9),
  min_green_columns_ratio_(0.1),
  camera_info_received_(false),
  camera_width_(0),
  camera_height_(0),
  model_loaded_(false),
  horizon_valid_(false)
{
  this->declare_parameter("image_topic", "/robotis_op3/camera/image_raw");
  this->declare_parameter("camera_info_topic", "/robotis_op3/camera/camera_info");
  this->declare_parameter("green_mask_topic", "/vision/green/mask");
  this->declare_parameter("ball_center_topic", "/vision/yolo/ball_center");
  this->declare_parameter("output_frame", "odom");
  this->declare_parameter("camera_frame", "cam_link");
  this->declare_parameter("model_path", "models/yolo.onnx");
  this->declare_parameter("use_gpu", use_gpu_);
  this->declare_parameter("use_fp16", use_fp16_);
  this->declare_parameter("dnn_backend", "opencv");
  this->declare_parameter("dnn_target", "cpu");
  this->declare_parameter("input_size", input_size_);
  this->declare_parameter("ball_confidence_threshold", 0.2);
  this->declare_parameter("goalpost_confidence_threshold", 0.5);
  this->declare_parameter("robot_confidence_threshold", 0.2);
  this->declare_parameter("intersection_confidence_threshold", 0.4);
  this->declare_parameter("nms_threshold", nms_threshold_);
  this->declare_parameter("nms_score_threshold", nms_score_threshold_);
  this->declare_parameter("publish_debug", publish_debug_);
  this->declare_parameter("use_green_horizon", use_green_horizon_);
  this->declare_parameter("ground_z", ground_z_);
  this->declare_parameter("max_range_m", max_range_m_);
  this->declare_parameter("horizon_max_age_sec", horizon_max_age_sec_);
  this->declare_parameter("horizon_offset_px", horizon_offset_px_);
  this->declare_parameter("horizon_stride", horizon_stride_);
  this->declare_parameter("horizon_smooth_window", horizon_smooth_window_);
  this->declare_parameter("min_green_columns_ratio", min_green_columns_ratio_);

  image_topic_ = this->get_parameter("image_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  green_mask_topic_ = this->get_parameter("green_mask_topic").as_string();
  ball_center_topic_ = this->get_parameter("ball_center_topic").as_string();
  output_frame_ = this->get_parameter("output_frame").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  model_path_ = resolveModelPath(this->get_parameter("model_path").as_string());
  use_gpu_ = this->get_parameter("use_gpu").as_bool();
  use_fp16_ = this->get_parameter("use_fp16").as_bool();
  dnn_backend_ = this->get_parameter("dnn_backend").as_string();
  dnn_target_ = this->get_parameter("dnn_target").as_string();
  if (use_gpu_) {
    const std::string backend_key = toLower(dnn_backend_);
    const std::string target_key = toLower(dnn_target_);
    if (backend_key.empty() || backend_key == "opencv" || backend_key == "default") {
      dnn_backend_ = "cuda";
    }
    if (target_key.empty() || target_key == "cpu" || target_key == "default") {
      dnn_target_ = use_fp16_ ? "cuda_fp16" : "cuda";
    }
  }
  input_size_ = this->get_parameter("input_size").as_int();
  nms_threshold_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
  nms_score_threshold_ = static_cast<float>(this->get_parameter("nms_score_threshold").as_double());
  publish_debug_ = this->get_parameter("publish_debug").as_bool();
  use_green_horizon_ = this->get_parameter("use_green_horizon").as_bool();
  ground_z_ = this->get_parameter("ground_z").as_double();
  max_range_m_ = this->get_parameter("max_range_m").as_double();
  horizon_max_age_sec_ = this->get_parameter("horizon_max_age_sec").as_double();
  horizon_offset_px_ = this->get_parameter("horizon_offset_px").as_int();
  horizon_stride_ = this->get_parameter("horizon_stride").as_int();
  horizon_smooth_window_ = this->get_parameter("horizon_smooth_window").as_int();
  min_green_columns_ratio_ = this->get_parameter("min_green_columns_ratio").as_double();

  class_info_.resize(kNumClasses);
  class_info_[0] = {"ball", cv::Scalar(0, 255, 255),
    static_cast<float>(this->get_parameter("ball_confidence_threshold").as_double())};
  class_info_[1] = {"goal post", cv::Scalar(0, 255, 0),
    static_cast<float>(this->get_parameter("goalpost_confidence_threshold").as_double())};
  class_info_[2] = {"robot", cv::Scalar(255, 0, 0),
    static_cast<float>(this->get_parameter("robot_confidence_threshold").as_double())};
  class_info_[3] = {"L-intersection", cv::Scalar(255, 255, 0),
    static_cast<float>(this->get_parameter("intersection_confidence_threshold").as_double())};
  class_info_[4] = {"T-intersection", cv::Scalar(255, 0, 255),
    static_cast<float>(this->get_parameter("intersection_confidence_threshold").as_double())};
  class_info_[5] = {"X-intersection", cv::Scalar(0, 255, 255),
    static_cast<float>(this->get_parameter("intersection_confidence_threshold").as_double())};

  model_loaded_ = loadModel();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&YoloDetector::imageCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, rclcpp::SensorDataQoS(),
    std::bind(&YoloDetector::cameraInfoCallback, this, std::placeholders::_1));

  green_mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    green_mask_topic_, rclcpp::SensorDataQoS(),
    std::bind(&YoloDetector::greenMaskCallback, this, std::placeholders::_1));

  debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/yolo/debug", 10);
  balls_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vision/yolo/balls", 10);
  goals_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vision/yolo/goals", 10);
  robots_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vision/yolo/robots", 10);
  intersections_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/vision/yolo/intersections", 10);
  if (!ball_center_topic_.empty()) {
    ball_center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      ball_center_topic_, 10);
  }

  RCLCPP_INFO(this->get_logger(), "YOLO detector initialized");
  RCLCPP_INFO(this->get_logger(), "  Image topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Model: %s", model_path_.c_str());
}

bool YoloDetector::loadModel()
{
  if (model_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "model_path is empty");
    return false;
  }

  if (!std::filesystem::exists(model_path_)) {
    RCLCPP_ERROR(this->get_logger(), "YOLO model not found: %s", model_path_.c_str());
    return false;
  }

  try {
    net_ = cv::dnn::readNetFromONNX(model_path_);
    const std::string backend_key = toLower(dnn_backend_);
    const std::string target_key = toLower(dnn_target_);
    const bool backend_known = (backend_key == "opencv" || backend_key == "cuda" || backend_key.empty() ||
      backend_key == "default");
    const bool target_known = (target_key == "cpu" || target_key == "cuda" || target_key == "cuda_fp16" ||
      target_key == "fp16" || target_key.empty() || target_key == "default");
    if (!backend_known) {
      RCLCPP_WARN(this->get_logger(), "Unknown dnn_backend '%s', falling back to OpenCV",
                  dnn_backend_.c_str());
    }
    if (!target_known) {
      RCLCPP_WARN(this->get_logger(), "Unknown dnn_target '%s', falling back to CPU",
                  dnn_target_.c_str());
    }
    const int backend = resolveDnnBackend(dnn_backend_);
    const int target = resolveDnnTarget(dnn_target_);
    try {
      net_.setPreferableBackend(backend);
      net_.setPreferableTarget(target);
      RCLCPP_INFO(this->get_logger(), "  DNN backend: %s", dnn_backend_.c_str());
      RCLCPP_INFO(this->get_logger(), "  DNN target: %s", dnn_target_.c_str());
    } catch (const cv::Exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to set DNN backend/target: %s", e.what());
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      dnn_backend_ = "opencv";
      dnn_target_ = "cpu";
      RCLCPP_WARN(this->get_logger(), "Falling back to CPU (OpenCV)");
    }
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO model: %s", e.what());
    RCLCPP_ERROR(this->get_logger(),
                 "OpenCV DNN is picky about ONNX. Try models/old.onnx or upgrade OpenCV.");
    return false;
  }

  return true;
}

void YoloDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  camera_width_ = static_cast<int>(msg->width);
  camera_height_ = static_cast<int>(msg->height);

  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
    msg->k[0], msg->k[1], msg->k[2],
    msg->k[3], msg->k[4], msg->k[5],
    msg->k[6], msg->k[7], msg->k[8]);

  if (!msg->d.empty()) {
    dist_coeffs_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }
  } else {
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
  }

  if (camera_frame_.empty()) {
    camera_frame_ = msg->header.frame_id;
  }

  camera_info_received_ = true;
}

void YoloDetector::greenMaskCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!use_green_horizon_) {
    return;
  }

  cv::Mat mask;
  try {
    mask = toMono8(*msg);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Green mask conversion failed: %s", e.what());
    return;
  }

  updateHorizon(mask, msg->header.stamp);
}

void YoloDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!model_loaded_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "YOLO model not loaded yet");
    return;
  }

  if (!camera_info_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "Waiting for camera info on %s", camera_info_topic_.c_str());
    return;
  }

  cv::Mat bgr;
  try {
    bgr = toBgr(*msg);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Image conversion failed: %s", e.what());
    return;
  }

  const int width = bgr.cols;
  const int height = bgr.rows;
  const int max_dim = std::max(width, height);
  cv::Mat letterbox(max_dim, max_dim, CV_8UC3, cv::Scalar(0, 0, 0));
  bgr.copyTo(letterbox(cv::Rect(0, 0, width, height)));

  cv::Mat blob = cv::dnn::blobFromImage(
    letterbox, 1.0 / 255.0, cv::Size(input_size_, input_size_), cv::Scalar(), true, false);

  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  if (outputs.empty()) {
    RCLCPP_WARN(this->get_logger(), "YOLO inference returned no outputs");
    return;
  }

  cv::Mat output = outputs[0];
  float scale = static_cast<float>(max_dim) / static_cast<float>(input_size_);

  std::vector<Detection> detections = decodeDetections(output, scale, width, height);
  if (detections.empty()) {
    if (publish_debug_) {
      debug_pub_->publish(toImageMsg(bgr, msg->header, "bgr8"));
    }
    return;
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  boxes.reserve(detections.size());
  scores.reserve(detections.size());
  for (const auto& det : detections) {
    boxes.push_back(det.box);
    scores.push_back(det.score);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, nms_score_threshold_, nms_threshold_, indices);

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(output_frame_, camera_frame_, msg->header.stamp,
                                         rclcpp::Duration::from_seconds(0.2));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed (%s -> %s): %s",
                output_frame_.c_str(), camera_frame_.c_str(), ex.what());
    return;
  }

  tf2::Transform cam_to_out;
  tf2::fromMsg(tf_msg.transform, cam_to_out);

  std::vector<tf2::Vector3> ball_points;
  std::vector<tf2::Vector3> goal_points;
  std::vector<tf2::Vector3> robot_points;
  std::vector<tf2::Vector3> intersection_points;
  bool have_ball_center = false;
  cv::Point2f best_ball_pixel(0.0f, 0.0f);
  double best_ball_dist = std::numeric_limits<double>::infinity();
  float best_ball_score = -1.0f;

  cv::Mat debug_image = bgr.clone();

  for (int idx : indices) {
    if (idx < 0 || idx >= static_cast<int>(detections.size())) {
      continue;
    }
    const auto& det = detections[idx];
    const auto& info = class_info_[det.class_id];

    int cx = det.box.x + det.box.width / 2;
    int cy = det.box.y + det.box.height / 2;
    int bottom_y = det.box.y + det.box.height;

    if (!horizonAllows(cx, bottom_y, msg->header.stamp)) {
      continue;
    }

    tf2::Vector3 ray_cam;
    cv::Point2f pixel_center(static_cast<float>(cx), static_cast<float>(cy));
    cv::Point2f pixel_bottom(static_cast<float>(cx), static_cast<float>(bottom_y));

    const bool use_bottom = (info.name != "L-intersection" &&
                             info.name != "T-intersection" &&
                             info.name != "X-intersection");

    if (!computeRay(use_bottom ? pixel_bottom : pixel_center, ray_cam)) {
      continue;
    }

    tf2::Vector3 point_out;
    if (!projectToGround(ray_cam, cam_to_out, point_out)) {
      continue;
    }

    double planar_dist = std::hypot(point_out.x(), point_out.y());
    if (planar_dist > max_range_m_) {
      continue;
    }

    if (info.name == "ball") {
      ball_points.push_back(point_out);
      if (ball_center_pub_) {
        if (!have_ball_center || planar_dist < best_ball_dist ||
            (std::abs(planar_dist - best_ball_dist) < 1e-3 && det.score > best_ball_score)) {
          have_ball_center = true;
          best_ball_dist = planar_dist;
          best_ball_score = det.score;
          best_ball_pixel = pixel_center;
        }
      }
    } else if (info.name == "goal post") {
      goal_points.push_back(point_out);
    } else if (info.name == "robot") {
      robot_points.push_back(point_out);
    } else {
      intersection_points.push_back(point_out);
    }

    if (publish_debug_) {
      cv::rectangle(debug_image, det.box, info.color, 2);
      const std::string label = info.name + " " +
        cv::format("%.2f", static_cast<double>(det.score));
      cv::putText(debug_image, label, cv::Point(det.box.x, std::max(0, det.box.y - 5)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, info.color, 1);
    }
  }

  balls_pub_->publish(buildCloud(ball_points, msg->header.stamp));
  goals_pub_->publish(buildCloud(goal_points, msg->header.stamp));
  robots_pub_->publish(buildCloud(robot_points, msg->header.stamp));
  intersections_pub_->publish(buildCloud(intersection_points, msg->header.stamp));
  if (ball_center_pub_ && have_ball_center && width > 0 && height > 0) {
    geometry_msgs::msg::PointStamped center_msg;
    center_msg.header = msg->header;
    const double norm_x = static_cast<double>(best_ball_pixel.x) / static_cast<double>(width);
    const double norm_y = static_cast<double>(best_ball_pixel.y) / static_cast<double>(height);
    center_msg.point.x = std::max(0.0, std::min(1.0, norm_x));
    center_msg.point.y = std::max(0.0, std::min(1.0, norm_y));
    center_msg.point.z = static_cast<double>(best_ball_score);
    ball_center_pub_->publish(center_msg);
  }

  if (publish_debug_) {
    debug_pub_->publish(toImageMsg(debug_image, msg->header, "bgr8"));
  }
}

std::vector<YoloDetector::Detection> YoloDetector::decodeDetections(
  const cv::Mat& output,
  float scale,
  int image_width,
  int image_height) const
{
  std::vector<Detection> detections;

  cv::Mat data;
  const int cols_no_obj = 4 + kNumClasses;
  const int cols_with_obj = 5 + kNumClasses;

  if (output.dims == 3) {
    const int dim1 = output.size[1];
    const int dim2 = output.size[2];
    if (dim1 == cols_no_obj || dim1 == cols_with_obj) {
      cv::Mat reshaped(dim1, dim2, CV_32F, const_cast<float*>(output.ptr<float>()));
      cv::transpose(reshaped, data);
    } else if (dim2 == cols_no_obj || dim2 == cols_with_obj) {
      data = cv::Mat(dim1, dim2, CV_32F, const_cast<float*>(output.ptr<float>())).clone();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unexpected YOLO output shape [%d, %d, %d]",
                  output.size[0], output.size[1], output.size[2]);
      return detections;
    }
  } else if (output.dims == 2) {
    data = output;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unexpected YOLO output dims: %d", output.dims);
    return detections;
  }

  if (data.cols < cols_no_obj) {
    RCLCPP_WARN(this->get_logger(), "YOLO output columns too small: %d", data.cols);
    return detections;
  }

  detections.reserve(data.rows);
  for (int i = 0; i < data.rows; ++i) {
    const float cx = data.at<float>(i, 0);
    const float cy = data.at<float>(i, 1);
    const float w = data.at<float>(i, 2);
    const float h = data.at<float>(i, 3);

    const bool has_objectness = data.cols >= cols_with_obj;
    const int class_offset = has_objectness ? 5 : 4;
    const float objectness = has_objectness ? data.at<float>(i, 4) : 1.0f;

    float best_score = -1.0f;
    int best_class = -1;
    for (int c = 0; c < kNumClasses; ++c) {
      const float score = data.at<float>(i, class_offset + c) * objectness;
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }

    if (best_class < 0 || best_class >= kNumClasses) {
      continue;
    }

    if (best_score < class_info_[best_class].threshold) {
      continue;
    }

    int left = static_cast<int>((cx - 0.5f * w) * scale);
    int top = static_cast<int>((cy - 0.5f * h) * scale);
    int width = static_cast<int>(w * scale);
    int height = static_cast<int>(h * scale);

    left = std::clamp(left, 0, image_width - 1);
    top = std::clamp(top, 0, image_height - 1);
    width = std::clamp(width, 1, image_width - left);
    height = std::clamp(height, 1, image_height - top);

    detections.push_back({cv::Rect(left, top, width, height), best_class, best_score});
  }

  return detections;
}

bool YoloDetector::computeRay(const cv::Point2f& pixel, tf2::Vector3& ray_cam) const
{
  if (camera_matrix_.empty()) {
    return false;
  }

  std::vector<cv::Point2f> pts = {pixel};
  std::vector<cv::Point2f> undist;
  cv::undistortPoints(pts, undist, camera_matrix_, dist_coeffs_);
  if (undist.empty()) {
    return false;
  }

  ray_cam = tf2::Vector3(undist[0].x, undist[0].y, 1.0);
  if (ray_cam.length() < 1e-6) {
    return false;
  }
  ray_cam.normalize();
  return true;
}

bool YoloDetector::projectToGround(const tf2::Vector3& ray_cam,
                                  const tf2::Transform& cam_to_out,
                                  tf2::Vector3& out_point) const
{
  const tf2::Vector3 origin = cam_to_out.getOrigin();
  const tf2::Vector3 direction = cam_to_out.getBasis() * ray_cam;
  if (std::abs(direction.z()) < 1e-6) {
    return false;
  }

  const double t = (ground_z_ - origin.z()) / direction.z();
  if (t <= 0.0) {
    return false;
  }

  out_point = origin + direction * t;
  return true;
}

void YoloDetector::updateHorizon(const cv::Mat& mask, const rclcpp::Time& stamp)
{
  if (mask.empty()) {
    return;
  }

  const int width = mask.cols;
  const int height = mask.rows;
  const int stride = std::max(1, horizon_stride_);

  std::vector<int> sample_x;
  std::vector<int> sample_y;
  sample_x.reserve((width + stride - 1) / stride);
  sample_y.reserve((width + stride - 1) / stride);

  int green_columns = 0;
  for (int x = 0; x < width; x += stride) {
    int y = 0;
    for (; y < height; ++y) {
      if (mask.at<uint8_t>(y, x) > 0) {
        break;
      }
    }
    if (y < height) {
      green_columns++;
    }
    sample_x.push_back(x);
    sample_y.push_back(y);
  }

  const double ratio = sample_x.empty() ? 0.0 :
    static_cast<double>(green_columns) / static_cast<double>(sample_x.size());
  if (ratio < min_green_columns_ratio_) {
    std::lock_guard<std::mutex> lock(horizon_mutex_);
    horizon_valid_ = false;
    return;
  }

  std::vector<int> horizon(width, height);
  for (size_t i = 0; i < sample_x.size(); ++i) {
    const int x0 = sample_x[i];
    const int y0 = sample_y[i];
    const int x1 = (i + 1 < sample_x.size()) ? sample_x[i + 1] : width - 1;
    const int y1 = (i + 1 < sample_y.size()) ? sample_y[i + 1] : y0;

    const int span = std::max(1, x1 - x0);
    for (int x = x0; x <= x1; ++x) {
      const double t = static_cast<double>(x - x0) / static_cast<double>(span);
      const int y = static_cast<int>(std::round((1.0 - t) * y0 + t * y1));
      horizon[x] = std::clamp(y - horizon_offset_px_, 0, height - 1);
    }
  }

  const int window = std::max(1, horizon_smooth_window_);
  if (window > 1) {
    std::vector<int> smooth(horizon.size(), 0);
    const int half = window / 2;
    for (size_t x = 0; x < horizon.size(); ++x) {
      int sum = 0;
      int count = 0;
      const int start = static_cast<int>(x) - half;
      const int end = static_cast<int>(x) + half;
      for (int xi = start; xi <= end; ++xi) {
        if (xi < 0 || xi >= static_cast<int>(horizon.size())) {
          continue;
        }
        sum += horizon[xi];
        count++;
      }
      smooth[x] = count > 0 ? sum / count : horizon[x];
    }
    horizon.swap(smooth);
  }

  std::lock_guard<std::mutex> lock(horizon_mutex_);
  horizon_y_ = std::move(horizon);
  horizon_valid_ = true;
  horizon_stamp_ = stamp;
}

bool YoloDetector::horizonAllows(int x, int y, const rclcpp::Time& stamp) const
{
  if (!use_green_horizon_) {
    return true;
  }

  std::lock_guard<std::mutex> lock(horizon_mutex_);
  if (!horizon_valid_ || horizon_y_.empty()) {
    return true;
  }

  if ((stamp - horizon_stamp_).seconds() > horizon_max_age_sec_) {
    return true;
  }

  const int clamped_x = std::clamp(x, 0, static_cast<int>(horizon_y_.size()) - 1);
  const int horizon_y = horizon_y_[clamped_x];
  return y >= horizon_y;
}

sensor_msgs::msg::PointCloud2 YoloDetector::buildCloud(
  const std::vector<tf2::Vector3>& points,
  const rclcpp::Time& stamp) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = output_frame_;
  cloud.header.stamp = stamp;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto& p : points) {
    *iter_x = static_cast<float>(p.x());
    *iter_y = static_cast<float>(p.y());
    *iter_z = static_cast<float>(p.z());
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

}  // namespace op3_yolo_vision

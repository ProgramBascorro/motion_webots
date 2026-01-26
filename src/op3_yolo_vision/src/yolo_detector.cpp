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
#include <cctype>
#include <cstring>
#include <filesystem>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <soccer_msgs/msg/bounding_box.hpp>

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

}  // namespace

YoloDetector::YoloDetector(const rclcpp::NodeOptions& options)
: Node("yolo_detector", options),
  input_size_(640),
  nms_threshold_(0.5f),
  nms_score_threshold_(0.1f),
  publish_debug_(true),
  use_gpu_(false),
  use_fp16_(false),
  frame_skip_(1),
  model_loaded_(false),
  frame_count_(0),
  frame_counter_(0),
  total_inference_ms_(0.0),
  total_full_ms_(0.0)
{
  this->declare_parameter("image_topic", "/robotis_op3/camera/image_raw");
  this->declare_parameter("ball_center_topic", "/vision/yolo/ball_center");
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
  this->declare_parameter("frame_skip", frame_skip_);

  image_topic_ = this->get_parameter("image_topic").as_string();
  ball_center_topic_ = this->get_parameter("ball_center_topic").as_string();
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
  frame_skip_ = this->get_parameter("frame_skip").as_int();
  if (frame_skip_ < 1) frame_skip_ = 1;

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

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&YoloDetector::imageCallback, this, std::placeholders::_1));

  debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/yolo/debug", 10);
  detections_pub_ = this->create_publisher<soccer_msgs::msg::BoundingBoxes>(
    "/vision/yolo/detections", 10);
  if (!ball_center_topic_.empty()) {
    ball_center_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      ball_center_topic_, 10);
  }

  last_log_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "YOLO detector initialized (SIMPLIFIED - 2D only)");
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

void YoloDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!model_loaded_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "YOLO model not loaded yet");
    return;
  }

  // Frame skipping
  frame_counter_++;
  if (frame_counter_ % frame_skip_ != 0) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  cv::Mat bgr;
  try {
    bgr = toBgr(*msg);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Image conversion failed: %s", e.what());
    return;
  }

  const int width = bgr.cols;
  const int height = bgr.rows;

  // Direct resize instead of letterboxing (faster, less memory)
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(input_size_, input_size_));

  // Calculate scale for bbox coordinates
  float scale_x = static_cast<float>(width) / static_cast<float>(input_size_);
  float scale_y = static_cast<float>(height) / static_cast<float>(input_size_);

  cv::Mat blob = cv::dnn::blobFromImage(
    resized, 1.0 / 255.0, cv::Size(input_size_, input_size_), cv::Scalar(), true, false);

  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());

  auto inference_time = std::chrono::high_resolution_clock::now();
  double inference_ms = std::chrono::duration<double, std::milli>(
    inference_time - start_time).count();

  if (outputs.empty()) {
    RCLCPP_WARN(this->get_logger(), "YOLO inference returned no outputs");
    return;
  }

  cv::Mat output = outputs[0];

  std::vector<Detection> detections = decodeDetections(output, 1.0f, input_size_, input_size_);

  // Scale detections back to original image size
  for (auto& det : detections) {
    det.box.x = static_cast<int>(det.box.x * scale_x);
    det.box.y = static_cast<int>(det.box.y * scale_y);
    det.box.width = static_cast<int>(det.box.width * scale_x);
    det.box.height = static_cast<int>(det.box.height * scale_y);
  }

  if (detections.empty()) {
    // Publish empty detections
    soccer_msgs::msg::BoundingBoxes empty_boxes;
    empty_boxes.header = msg->header;
    detections_pub_->publish(empty_boxes);
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

  soccer_msgs::msg::BoundingBoxes bboxes_msg;
  bboxes_msg.header = msg->header;
  bboxes_msg.bounding_boxes.reserve(indices.size());

  bool have_ball_center = false;
  cv::Point2f best_ball_pixel(0.0f, 0.0f);
  float best_ball_score = -1.0f;
  double best_ball_area = 0.0;

  // Only clone for debug if needed
  cv::Mat debug_image;
  if (publish_debug_) {
    debug_image = bgr.clone();
  }

  for (int idx : indices) {
    if (idx < 0 || idx >= static_cast<int>(detections.size())) {
      continue;
    }
    const auto& det = detections[idx];
    const auto& info = class_info_[det.class_id];

    soccer_msgs::msg::BoundingBox bbox;
    bbox.probability = det.score;
    bbox.xmin = det.box.x;
    bbox.ymin = det.box.y;
    bbox.xmax = det.box.x + det.box.width;
    bbox.ymax = det.box.y + det.box.height;
    bbox.class_id = info.name;
    bbox.id = det.class_id;

    int cx = det.box.x + det.box.width / 2;
    int cy = det.box.y + det.box.height / 2;
    bbox.xbase = cx;
    bbox.ybase = det.box.y + det.box.height;
    bbox.obstacle_detected = false;

    bboxes_msg.bounding_boxes.push_back(bbox);

    if (info.name == "ball") {
      double area = det.box.width * det.box.height;
      if (!have_ball_center || area > best_ball_area ||
          (std::abs(area - best_ball_area) < 1e-3 && det.score > best_ball_score)) {
        have_ball_center = true;
        best_ball_area = area;
        best_ball_score = det.score;
        best_ball_pixel = cv::Point2f(static_cast<float>(cx), static_cast<float>(cy));
      }
    }

    if (publish_debug_) {
      cv::rectangle(debug_image, det.box, info.color, 2);
      const std::string label = info.name + " " +
        cv::format("%.2f", static_cast<double>(det.score));
      cv::putText(debug_image, label, cv::Point(det.box.x, std::max(0, det.box.y - 5)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, info.color, 1);
    }
  }

  detections_pub_->publish(bboxes_msg);

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

  auto end_time = std::chrono::high_resolution_clock::now();
  double full_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  // Performance logging
  frame_count_++;
  total_inference_ms_ += inference_ms;
  total_full_ms_ += full_ms;
  auto now = this->now();
  if ((now - last_log_time_).seconds() >= 5.0) {
    double avg_inference_ms = total_inference_ms_ / frame_count_;
    double avg_full_ms = total_full_ms_ / frame_count_;
    double effective_fps = 1000.0 / avg_full_ms;
    RCLCPP_INFO(this->get_logger(),
                "Performance: inference=%.1fms, total=%.1fms, %.1f FPS (skip=%d, backend=%s/%s)",
                avg_inference_ms, avg_full_ms, effective_fps, frame_skip_,
                dnn_backend_.c_str(), dnn_target_.c_str());
    frame_count_ = 0;
    total_inference_ms_ = 0.0;
    total_full_ms_ = 0.0;
    last_log_time_ = now;
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

}  // namespace op3_yolo_vision

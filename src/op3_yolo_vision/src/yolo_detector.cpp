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
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <map>
#include <sstream>

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

bool isDefaultBackendValue(const std::string& value)
{
  const std::string key = toLower(value);
  return key.empty() || key == "opencv" || key == "default";
}

bool isDefaultTargetValue(const std::string& value)
{
  const std::string key = toLower(value);
  return key.empty() || key == "cpu" || key == "default";
}

std::string shapeToString(const ov::Shape& shape)
{
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < shape.size(); ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << shape[i];
  }
  stream << "]";
  return stream.str();
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
  device_("CPU"),
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
  this->declare_parameter("device", device_);
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
  device_ = this->get_parameter("device").as_string();
  if (device_.empty()) {
    device_ = "CPU";
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

  logDeprecatedBackendSettings();
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
  RCLCPP_INFO(this->get_logger(), "  Device: %s", device_.c_str());
}

void YoloDetector::logDeprecatedBackendSettings() const
{
  if (use_gpu_ || use_fp16_ || !isDefaultBackendValue(dnn_backend_) || !isDefaultTargetValue(dnn_target_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameters use_gpu/use_fp16/dnn_backend/dnn_target are deprecated and ignored. "
      "OpenVINO uses device='%s'.",
      device_.c_str());
  }
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
    model_ = core_.read_model(model_path_);
    if (!model_) {
      RCLCPP_ERROR(this->get_logger(), "OpenVINO returned a null model handle");
      return false;
    }

    if (model_->inputs().empty()) {
      RCLCPP_ERROR(this->get_logger(), "Model has no inputs");
      return false;
    }

    if (model_->outputs().empty()) {
      RCLCPP_ERROR(this->get_logger(), "Model has no outputs");
      return false;
    }

    try {
      std::map<std::string, ov::PartialShape> reshape_map;
      reshape_map.emplace(
        model_->input().get_any_name(),
        ov::PartialShape{1, 3, input_size_, input_size_});
      model_->reshape(reshape_map);
    } catch (const ov::Exception& e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to reshape model input to %dx%d: %s. Continuing with the model's native input shape.",
        input_size_, input_size_, e.what());
    }

    compiled_model_ = core_.compile_model(model_, device_);
    infer_request_ = compiled_model_.create_infer_request();

    const ov::Shape input_shape = compiled_model_.input().get_shape();
    if (input_shape.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), "Unexpected model input shape: %s",
                   shapeToString(input_shape).c_str());
      return false;
    }

    if (input_shape[0] != 1 || input_shape[1] != 3) {
      RCLCPP_WARN(this->get_logger(), "Expected NCHW input [1, 3, H, W], got %s",
                  shapeToString(input_shape).c_str());
    }

    if (input_shape[2] != input_shape[3]) {
      RCLCPP_WARN(this->get_logger(), "Model input is not square: %s",
                  shapeToString(input_shape).c_str());
    }

    if (static_cast<int>(input_shape[2]) != input_size_ || static_cast<int>(input_shape[3]) != input_size_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Requested input_size=%d but model uses %s. Using the compiled model input height.",
        input_size_, shapeToString(input_shape).c_str());
      input_size_ = static_cast<int>(input_shape[2]);
    }

    RCLCPP_INFO(this->get_logger(), "  OpenVINO compiled model on %s", device_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Model input shape: %s",
                shapeToString(compiled_model_.input().get_shape()).c_str());
    RCLCPP_INFO(this->get_logger(), "  Model output shape: %s",
                shapeToString(compiled_model_.output().get_shape()).c_str());
  } catch (const ov::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO model with OpenVINO: %s", e.what());
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
  auto publish_debug_frame = [&]() {
    if (publish_debug_) {
      debug_pub_->publish(toImageMsg(bgr, msg->header, "bgr8"));
    }
  };
  auto publish_empty_detections = [&]() {
    soccer_msgs::msg::BoundingBoxes empty_boxes;
    empty_boxes.header = msg->header;
    detections_pub_->publish(empty_boxes);
  };

  // Use square aspect ratio preserving resize (faster than letterbox)
  // Find the longer dimension to determine scale
  const int max_dim = std::max(width, height);

  // Create a square canvas (but don't use black padding, just resize)
  cv::Mat processed;
  if (width == height) {
    // Already square, just resize
    cv::resize(bgr, processed, cv::Size(input_size_, input_size_));
  } else {
    // Create square letterbox (more efficient than before)
    cv::Mat square(max_dim, max_dim, CV_8UC3, cv::Scalar(114, 114, 114));
    bgr.copyTo(square(cv::Rect(0, 0, width, height)));
    cv::resize(square, processed, cv::Size(input_size_, input_size_));
  }

  // Calculate scale for bbox coordinates
  float scale = static_cast<float>(max_dim) / static_cast<float>(input_size_);

  cv::Mat rgb;
  cv::cvtColor(processed, rgb, cv::COLOR_BGR2RGB);

  cv::Mat normalized;
  rgb.convertTo(normalized, CV_32FC3, 1.0 / 255.0);

  const size_t plane_size = static_cast<size_t>(input_size_) * static_cast<size_t>(input_size_);
  std::vector<float> input_buffer(plane_size * 3);
  std::vector<cv::Mat> channels;
  cv::split(normalized, channels);
  if (channels.size() != 3) {
    RCLCPP_WARN(this->get_logger(), "Expected 3 channels after preprocessing, got %zu", channels.size());
    publish_empty_detections();
    publish_debug_frame();
    return;
  }
  for (size_t channel = 0; channel < channels.size(); ++channel) {
    std::memcpy(input_buffer.data() + (channel * plane_size),
                channels[channel].ptr<float>(),
                plane_size * sizeof(float));
  }

  cv::Mat debug_image;
  if (publish_debug_) {
    debug_image = bgr.clone();
  }

  std::vector<Detection> detections;
  try {
    const ov::Shape input_shape = compiled_model_.input().get_shape();
    size_t expected_values = 1;
    for (const auto dimension : input_shape) {
      expected_values *= dimension;
    }
    if (expected_values != input_buffer.size()) {
      RCLCPP_WARN(this->get_logger(), "Input tensor size mismatch: expected %zu values, have %zu",
                  expected_values, input_buffer.size());
      publish_empty_detections();
      if (publish_debug_) {
        debug_pub_->publish(toImageMsg(debug_image, msg->header, "bgr8"));
      }
      return;
    }

    ov::Tensor input_tensor(ov::element::f32, input_shape, input_buffer.data());
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.infer();

    if (compiled_model_.outputs().empty()) {
      RCLCPP_WARN(this->get_logger(), "YOLO inference returned no outputs");
      publish_empty_detections();
      if (publish_debug_) {
        debug_pub_->publish(toImageMsg(debug_image, msg->header, "bgr8"));
      }
      return;
    }

    detections = decodeDetections(infer_request_.get_output_tensor(0), scale, width, height);
  } catch (const ov::Exception& e) {
    RCLCPP_WARN(this->get_logger(), "OpenVINO inference failed: %s", e.what());
    publish_empty_detections();
    if (publish_debug_) {
      debug_pub_->publish(toImageMsg(debug_image, msg->header, "bgr8"));
    }
    return;
  }

  auto inference_time = std::chrono::high_resolution_clock::now();
  double inference_ms = std::chrono::duration<double, std::milli>(
    inference_time - start_time).count();

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

  if (!detections.empty()) {
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
                "Performance: inference=%.1fms, total=%.1fms, %.1f FPS (skip=%d, device=%s)",
                avg_inference_ms, avg_full_ms, effective_fps, frame_skip_,
                device_.c_str());
    frame_count_ = 0;
    total_inference_ms_ = 0.0;
    total_full_ms_ = 0.0;
    last_log_time_ = now;
  }
}

std::vector<YoloDetector::Detection> YoloDetector::decodeDetections(
  const ov::Tensor& output,
  float scale,
  int image_width,
  int image_height) const
{
  std::vector<Detection> detections;
  const ov::Shape shape = output.get_shape();

  std::vector<float> converted_output;
  const float* output_data = nullptr;
  if (output.get_element_type() == ov::element::f32) {
    output_data = static_cast<const float*>(output.data());
  } else if (output.get_element_type() == ov::element::f16) {
    const auto* float16_data = static_cast<const ov::float16*>(output.data());
    converted_output.resize(output.get_size());
    for (size_t i = 0; i < output.get_size(); ++i) {
      converted_output[i] = static_cast<float>(float16_data[i]);
    }
    output_data = converted_output.data();
  } else {
    RCLCPP_WARN(this->get_logger(), "Unsupported YOLO output tensor type: %s",
                output.get_element_type().to_string().c_str());
    return detections;
  }

  cv::Mat data;
  const int cols_no_obj = 4 + kNumClasses;
  const int cols_with_obj = 5 + kNumClasses;

  if (shape.size() == 3) {
    const int dim1 = static_cast<int>(shape[1]);
    const int dim2 = static_cast<int>(shape[2]);
    if (dim1 == cols_no_obj || dim1 == cols_with_obj) {
      cv::Mat reshaped(dim1, dim2, CV_32F, const_cast<float*>(output_data));
      cv::transpose(reshaped, data);
    } else if (dim2 == cols_no_obj || dim2 == cols_with_obj) {
      data = cv::Mat(dim1, dim2, CV_32F, const_cast<float*>(output_data)).clone();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unexpected YOLO output shape %s",
                  shapeToString(shape).c_str());
      return detections;
    }
  } else if (shape.size() == 2) {
    data = cv::Mat(static_cast<int>(shape[0]), static_cast<int>(shape[1]), CV_32F,
                   const_cast<float*>(output_data)).clone();
  } else {
    RCLCPP_WARN(this->get_logger(), "Unexpected YOLO output rank: %zu", shape.size());
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

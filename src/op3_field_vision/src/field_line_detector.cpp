#include "op3_field_vision/field_line_detector.hpp"

namespace op3_field_vision
{

FieldLineDetector::FieldLineDetector()
: Node("field_line_detector"),
  camera_info_received_(false)
{
  // Declare startup-only parameters (topics)
  this->declare_parameter("image_topic", "/robotis_op3/camera/image_raw");
  this->declare_parameter("camera_info_topic", "/robotis_op3/camera/camera_info");
  
  // Declare tunable parameters
  this->declare_parameter("hsv_s_max", 60);
  this->declare_parameter("hsv_v_min", 180);
  this->declare_parameter("canny_low", 50);
  this->declare_parameter("canny_high", 150);
  this->declare_parameter("hough_threshold", 50);
  this->declare_parameter("min_line_length", 40);
  this->declare_parameter("max_line_gap", 10);
  this->declare_parameter("morph_kernel", 3);
  this->declare_parameter("morph_iters", 1);
  this->declare_parameter("roi_y_min_ratio", 0.0);

  // Get startup-only parameters
  image_topic_ = this->get_parameter("image_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  
  // Initialize config with declared parameters
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_.hsv_s_max = this->get_parameter("hsv_s_max").as_int();
    config_.hsv_v_min = this->get_parameter("hsv_v_min").as_int();
    config_.canny_low = this->get_parameter("canny_low").as_int();
    config_.canny_high = this->get_parameter("canny_high").as_int();
    config_.hough_threshold = this->get_parameter("hough_threshold").as_int();
    config_.min_line_length = this->get_parameter("min_line_length").as_int();
    config_.max_line_gap = this->get_parameter("max_line_gap").as_int();
    config_.morph_kernel = this->get_parameter("morph_kernel").as_int();
    config_.morph_iters = this->get_parameter("morph_iters").as_int();
    config_.roi_y_min_ratio = this->get_parameter("roi_y_min_ratio").as_double();
  }

  // Register parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FieldLineDetector::onParameterChange, this, std::placeholders::_1));

  // Create subscribers with SensorDataQoS for camera topics
  auto sensor_qos = rclcpp::SensorDataQoS();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, sensor_qos,
    std::bind(&FieldLineDetector::imageCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, sensor_qos,
    std::bind(&FieldLineDetector::cameraInfoCallback, this, std::placeholders::_1));

  // Create publishers
  debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/lines/debug", 10);
  mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/lines/mask", 10);

  RCLCPP_INFO(this->get_logger(), "Field line detector initialized");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Publishing debug to: /vision/lines/debug");
  RCLCPP_INFO(this->get_logger(), "  Publishing mask to: /vision/lines/mask");
}

rcl_interfaces::msg::SetParametersResult FieldLineDetector::onParameterChange(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Work on temporary copy
  Config new_config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    new_config = config_;
  }

  for (const auto& param : params) {
    const std::string& name = param.get_name();
    
    // Reject topic changes (startup-only)
    if (name == "image_topic" || name == "camera_info_topic") {
      result.successful = false;
      result.reason = name + " cannot be changed at runtime (restart required)";
      return result;
    }
    
    // Type checking and clamping
    if (name == "hsv_s_max") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "hsv_s_max must be an integer";
        return result;
      }
      new_config.hsv_s_max = std::clamp(static_cast<int>(param.as_int()), 0, 255);
    } 
    else if (name == "hsv_v_min") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "hsv_v_min must be an integer";
        return result;
      }
      new_config.hsv_v_min = std::clamp(static_cast<int>(param.as_int()), 0, 255);
    } 
    else if (name == "canny_low") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "canny_low must be an integer";
        return result;
      }
      new_config.canny_low = std::max(0, static_cast<int>(param.as_int()));
    } 
    else if (name == "canny_high") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "canny_high must be an integer";
        return result;
      }
      new_config.canny_high = std::max(0, static_cast<int>(param.as_int()));
    } 
    else if (name == "hough_threshold") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "hough_threshold must be an integer";
        return result;
      }
      new_config.hough_threshold = std::max(1, static_cast<int>(param.as_int()));
    } 
    else if (name == "min_line_length") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "min_line_length must be an integer";
        return result;
      }
      new_config.min_line_length = std::max(0, static_cast<int>(param.as_int()));
    } 
    else if (name == "max_line_gap") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "max_line_gap must be an integer";
        return result;
      }
      new_config.max_line_gap = std::max(0, static_cast<int>(param.as_int()));
    } 
    else if (name == "morph_kernel") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "morph_kernel must be an integer";
        return result;
      }
      int k = std::max(1, static_cast<int>(param.as_int()));
      new_config.morph_kernel = (k % 2 == 0) ? k + 1 : k;  // Force odd
    } 
    else if (name == "morph_iters") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "morph_iters must be an integer";
        return result;
      }
      new_config.morph_iters = std::max(0, static_cast<int>(param.as_int()));
    } 
    else if (name == "roi_y_min_ratio") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "roi_y_min_ratio must be a double";
        return result;
      }
      new_config.roi_y_min_ratio = std::clamp(param.as_double(), 0.0, 0.95);
    }
  }

  // Validate relational constraints
  if (new_config.canny_high < new_config.canny_low) {
    result.successful = false;
    result.reason = "canny_high must be >= canny_low";
    return result;
  }

  // Commit atomically
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = new_config;
  }

  RCLCPP_INFO(this->get_logger(), "Parameters updated: hsv_s_max=%d hsv_v_min=%d canny=%d/%d hough=%d morph=%dx%d roi=%.2f",
              new_config.hsv_s_max, new_config.hsv_v_min, 
              new_config.canny_low, new_config.canny_high,
              new_config.hough_threshold, 
              new_config.morph_kernel, new_config.morph_iters,
              new_config.roi_y_min_ratio);
  
  return result;
}

void FieldLineDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_info_received_) {
    camera_info_ = msg;
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d", msg->width, msg->height);
  }
}

void FieldLineDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Convert to BGR regardless of input encoding
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  detectLines(cv_ptr->image);
}

void FieldLineDetector::detectLines(const cv::Mat& bgr_image)
{
  // Copy config once per frame (thread-safe)
  Config cfg;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    cfg = config_;
  }

  // Apply ROI first (crop top portion of image)
  int y0 = static_cast<int>(bgr_image.rows * cfg.roi_y_min_ratio);
  y0 = std::clamp(y0, 0, bgr_image.rows - 1);
  cv::Mat roi = bgr_image(cv::Range(y0, bgr_image.rows), cv::Range::all());

  // 1. Convert BGR -> HSV
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

  // 2. White threshold: low saturation + high value
  cv::Mat mask;
  cv::inRange(hsv, 
              cv::Scalar(0, 0, cfg.hsv_v_min),           // H_min, S_min, V_min
              cv::Scalar(180, cfg.hsv_s_max, 255),       // H_max, S_max, V_max
              mask);

  // 3. Morphology: erode then dilate (opening) to clean noise
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                                              cv::Size(cfg.morph_kernel, cfg.morph_kernel));
  cv::erode(mask, mask, kernel, cv::Point(-1, -1), cfg.morph_iters);
  cv::dilate(mask, mask, kernel, cv::Point(-1, -1), cfg.morph_iters);

  // 4. Canny edge detection
  cv::Mat edges;
  cv::Canny(mask, edges, cfg.canny_low, cfg.canny_high);

  // 5. HoughLinesP to get line segments
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edges, lines, 1, CV_PI / 180, cfg.hough_threshold, 
                  cfg.min_line_length, cfg.max_line_gap);

  // 6. Draw segments on debug image (full image with ROI offset)
  cv::Mat debug_image = bgr_image.clone();
  
  // Draw ROI boundary
  cv::line(debug_image, cv::Point(0, y0), cv::Point(debug_image.cols, y0),
           cv::Scalar(255, 255, 0), 1);  // Cyan line for ROI boundary
  
  for (const auto& line : lines) {
    // Offset y coordinates by ROI start
    cv::line(debug_image, 
             cv::Point(line[0], line[1] + y0), 
             cv::Point(line[2], line[3] + y0),
             cv::Scalar(0, 255, 0), 2);  // Green lines
  }

  // Add info text
  std::string text = "Lines: " + std::to_string(lines.size());
  cv::putText(debug_image, text, cv::Point(10, 30), 
              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
  
  std::string roi_text = "ROI: " + std::to_string(static_cast<int>(cfg.roi_y_min_ratio * 100)) + "%";
  cv::putText(debug_image, roi_text, cv::Point(10, 60), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

  // Publish debug image
  auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg();
  debug_msg->header.stamp = this->now();
  debug_msg->header.frame_id = "cam_link";
  debug_pub_->publish(*debug_msg);

  // Publish mask (create full-size mask with ROI region)
  cv::Mat full_mask = cv::Mat::zeros(bgr_image.rows, bgr_image.cols, CV_8UC1);
  mask.copyTo(full_mask(cv::Range(y0, bgr_image.rows), cv::Range::all()));
  
  auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", full_mask).toImageMsg();
  mask_msg->header.stamp = this->now();
  mask_msg->header.frame_id = "cam_link";
  mask_pub_->publish(*mask_msg);
}

}  // namespace op3_field_vision

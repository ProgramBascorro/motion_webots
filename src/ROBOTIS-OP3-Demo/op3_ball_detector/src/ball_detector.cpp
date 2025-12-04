#include "op3_ball_detector/ball_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace robotis_op
{

BallDetectorCpp::BallDetectorCpp()
: Node("ball_detector_node")
{
  declareParameters();
  loadParameters();
  
  // Initialize YOLO
  if (!model_path_.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Loading YOLO model: %s", model_path_.c_str());
    yolo_detector_ = std::make_unique<YoloDetector>(
      model_path_,
      confidence_threshold_,
      nms_threshold_,
      input_width_,
      input_height_,
      use_gpu_
    );
    
    if (!yolo_detector_->initialize())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO");
      throw std::runtime_error("YOLO initialization failed");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "YOLO initialized successfully");
    }
  }
  
  // Publishers
  ball_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
    "ball_position", 10);
  
  ball_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "ball_status", 10);
  
  if (publish_debug_image_)
  {
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_out", 10);
    RCLCPP_INFO(this->get_logger(), "Debug image publisher created");
  }
  
  // Subscriber
  RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", camera_topic_.c_str());
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_topic_,
    10,
    std::bind(&BallDetectorCpp::imageCallback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(this->get_logger(), "C++ Ball Detector initialized");
  RCLCPP_INFO(this->get_logger(), "  Camera topic: %s", camera_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Using GPU: %s", use_gpu_ ? "true" : "false");
}

BallDetectorCpp::~BallDetectorCpp()
{
  RCLCPP_INFO(this->get_logger(), "C++ Ball Detector destroyed");
}

void BallDetectorCpp::declareParameters()
{
  this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
  this->declare_parameter<std::string>("model_path", "");
  this->declare_parameter<bool>("publish_debug_image", true);
  this->declare_parameter<double>("confidence_threshold", 0.6);
  this->declare_parameter<double>("nms_threshold", 0.4);
  this->declare_parameter<int>("ball_class_id", 0);
  this->declare_parameter<int>("input_width", 320);
  this->declare_parameter<int>("input_height", 320);
  this->declare_parameter<bool>("use_gpu", false);
  this->declare_parameter<int>("process_every_n_frames", 1);
}

void BallDetectorCpp::loadParameters()
{
  camera_topic_ = this->get_parameter("camera_topic").as_string();
  model_path_ = this->get_parameter("model_path").as_string();
  publish_debug_image_ = this->get_parameter("publish_debug_image").as_bool();
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  nms_threshold_ = this->get_parameter("nms_threshold").as_double();
  ball_class_id_ = this->get_parameter("ball_class_id").as_int();
  input_width_ = this->get_parameter("input_width").as_int();
  input_height_ = this->get_parameter("input_height").as_int();
  use_gpu_ = this->get_parameter("use_gpu").as_bool();
  process_every_n_frames_ = this->get_parameter("process_every_n_frames").as_int();
  frame_counter_ = 0;
}

void BallDetectorCpp::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{  
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      current_image_ = cv_ptr->image.clone();
    }
    
    detectBall(current_image_);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void BallDetectorCpp::detectBall(const cv::Mat& image)
{
  if (!yolo_detector_ || image.empty())
    return;
  
  // Run detection
  std::vector<Detection> detections = yolo_detector_->detect(image);
  
  // Create debug image
  cv::Mat debug_image = image.clone();
  
  // Draw FPS
  drawFPS(debug_image);

  // Track current detection state
  bool ball_detected = false;
  bool goal_detected = false;
  bool robot_detected = false;
  Detection best_ball_detection = {};
  float max_confidence = 0.0f;
  
  // Process detections
  for (const auto& det : detections)
  {
    // Draw all detections
    drawDetection(debug_image, det);
    
    // Track by class
    switch (det.class_id)
    {
      case 0:  // Ball
        ball_detected = true;
        if (det.confidence > max_confidence)
        {
          best_ball_detection = det;
          max_confidence = det.confidence;
        }
        break;
        
      case 1:  // Goal
        goal_detected = true;
        break;
        
      case 2:  // Robot
        robot_detected = true;
        break;
    }
  }
  
  // Log only on state change - Ball
  if (ball_detected && !prev_ball_detected_)
  {
    RCLCPP_INFO(this->get_logger(), 
                "⚽ BOLA TERDETEKSI (conf: %.1f%%)",
                max_confidence * 100);
  }
  
  // Log only on state change - Goal
  if (goal_detected && !prev_goal_detected_)
  {
    RCLCPP_INFO(this->get_logger(), "🥅 GAWANG TERDETEKSI");
  }
  
  // Log only on state change - Robot
  if (robot_detected && !prev_robot_detected_)
  {
    RCLCPP_INFO(this->get_logger(), "🤖 ROBOT TERDETEKSI");
  }
  
  // Log "Tidak ada objek terdeteksi" hanya saat transisi dari ada -> tidak ada
  bool any_detected = ball_detected || goal_detected || robot_detected;
  bool prev_any_detected = prev_ball_detected_ || prev_goal_detected_ || prev_robot_detected_;
  
  if (!any_detected && prev_any_detected)
  {
    RCLCPP_INFO(this->get_logger(), "❌ Tidak ada objek terdeteksi");
  }
  
  // Update previous state
  prev_ball_detected_ = ball_detected;
  prev_goal_detected_ = goal_detected;
  prev_robot_detected_ = robot_detected;
  
  // Publish ball status
  if (ball_detected)
  {
    std::ostringstream oss;
    oss << "Ball detected | x:" << best_ball_detection.center.x 
        << " y:" << best_ball_detection.center.y 
        << " conf:" << static_cast<int>(best_ball_detection.confidence * 100) << "%";
    
    publishBallStatus(oss.str()); 
  }
  else
  {
    publishBallStatus("Ball not found");
  }
  
  // Publish debug image
  publishDebugImage(debug_image);
}

void BallDetectorCpp::drawFPS(cv::Mat& image)
{
  double fps = yolo_detector_->getFPS();
  double inference_time = yolo_detector_->getInferenceTime();
  
  std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
  std::string time_text = "Inference: " + std::to_string(static_cast<int>(inference_time)) + "ms";
  
  int font = cv::FONT_HERSHEY_SIMPLEX;
  double font_scale = 0.7;
  int thickness = 2;
  
  // FPS text
  auto text_size = cv::getTextSize(fps_text, font, font_scale, thickness, nullptr);
  cv::rectangle(image, cv::Point(10, 10), 
                cv::Point(20 + text_size.width, 20 + text_size.height),
                cv::Scalar(0, 0, 0), -1);
  cv::putText(image, fps_text, cv::Point(15, 15 + text_size.height),
              font, font_scale, cv::Scalar(0, 255, 0), thickness);
  
  // Inference time text
  text_size = cv::getTextSize(time_text, font, font_scale, thickness, nullptr);
  cv::rectangle(image, cv::Point(10, 40), 
                cv::Point(20 + text_size.width, 50 + text_size.height),
                cv::Scalar(0, 0, 0), -1);
  cv::putText(image, time_text, cv::Point(15, 45 + text_size.height),
              font, font_scale, cv::Scalar(0, 255, 255), thickness);
}

void BallDetectorCpp::drawDetection(cv::Mat& image, const Detection& det)
{
  // Define colors for each class
  cv::Scalar box_color;
  cv::Scalar text_color;
  std::string label;
  
  switch (det.class_id)
  {
    case 0:  // Ball
      label = "Ball";
      box_color = cv::Scalar(0, 255, 0);   // Green
      text_color = cv::Scalar(0, 255, 0);
      break;
      
    case 1:  // Goal
      label = "Goal";
      box_color = cv::Scalar(0, 255, 255); // Yellow
      text_color = cv::Scalar(0, 255, 255);
      break;
      
    case 2:  // Robot
      label = "Robot";
      box_color = cv::Scalar(255, 0, 255); // Magenta
      text_color = cv::Scalar(255, 0, 255);
      break;
      
    default:
      label = "Unknown";
      box_color = cv::Scalar(128, 128, 128); // Gray
      text_color = cv::Scalar(128, 128, 128);
      break;
  }
  
  // Add confidence to label
  label += " " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";
  
  // Draw bounding box with class-specific color
  cv::rectangle(image, det.box, box_color, 2);
  
  // Draw center point
  cv::circle(image, det.center, 5, cv::Scalar(0, 0, 255), -1);
  
  // Draw crosshair
  int crosshair_size = 10;
  cv::line(image, 
           cv::Point(det.center.x - crosshair_size, det.center.y),
           cv::Point(det.center.x + crosshair_size, det.center.y),
           cv::Scalar(0, 0, 255), 2);
  cv::line(image,
           cv::Point(det.center.x, det.center.y - crosshair_size),
           cv::Point(det.center.x, det.center.y + crosshair_size),
           cv::Scalar(0, 0, 255), 2);
  
  // Draw label with background
  int baseline = 0;
  cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
  
  // Draw label background
  cv::rectangle(image,
                cv::Point(det.box.x, det.box.y - text_size.height - 10),
                cv::Point(det.box.x + text_size.width, det.box.y),
                box_color, -1);
  
  // Draw label text
  cv::putText(image, label, 
              cv::Point(det.box.x, det.box.y - 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, 
              cv::Scalar(0, 0, 0), 2);
}

void BallDetectorCpp::publishBallPosition(const cv::Point2d& position)
{
  geometry_msgs::msg::Point msg;
  msg.x = position.x;
  msg.y = position.y;
  msg.z = 0.0;
  ball_position_pub_->publish(msg);
}

void BallDetectorCpp::publishBallStatus(const std::string& status)
{ 
  std_msgs::msg::String msg;
  msg.data = status;
  ball_status_pub_->publish(msg);
}

void BallDetectorCpp::publishDebugImage(const cv::Mat& image)
{
  if (!debug_image_pub_)
    return;
  
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "camera";
  
  cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, image);
  debug_image_pub_->publish(*cv_image.toImageMsg());
}

}  // namespace robotis_op
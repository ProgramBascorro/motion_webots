#include "op3_field_vision/field_line_detector.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace op3_field_vision
{

FieldLineDetector::FieldLineDetector()
: Node("field_line_detector"),
  camera_info_received_(false)
{
  // Declare startup-only parameters (topics/frames)
  this->declare_parameter("image_topic", "/robotis_op3/camera/image_raw");
  this->declare_parameter("camera_info_topic", "/robotis_op3/camera/camera_info");
  this->declare_parameter("output_frame", "odom");
  this->declare_parameter("camera_frame", "cam_link");
  
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
  this->declare_parameter("ground_z", 0.0);
  this->declare_parameter("max_range_m", 5.0);
  this->declare_parameter("samples_per_segment", 5);

  // Get startup-only parameters
  image_topic_ = this->get_parameter("image_topic").as_string();
  camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
  output_frame_ = this->get_parameter("output_frame").as_string();
  camera_frame_ = this->get_parameter("camera_frame").as_string();
  
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
    config_.ground_z = this->get_parameter("ground_z").as_double();
    config_.max_range_m = this->get_parameter("max_range_m").as_double();
    config_.samples_per_segment = this->get_parameter("samples_per_segment").as_int();
  }

  // Register parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&FieldLineDetector::onParameterChange, this, std::placeholders::_1));

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
  ground_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/vision/lines/ground_points", 10);
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vision/lines/markers", 10);

  RCLCPP_INFO(this->get_logger(), "Field line detector initialized");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output frame: %s, Camera frame: %s", output_frame_.c_str(), camera_frame_.c_str());
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
    
    // Reject startup-only param changes
    if (name == "image_topic" || name == "camera_info_topic" ||
        name == "output_frame" || name == "camera_frame") {
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
      new_config.morph_kernel = (k % 2 == 0) ? k + 1 : k;
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
    else if (name == "ground_z") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "ground_z must be a double";
        return result;
      }
      new_config.ground_z = param.as_double();
    }
    else if (name == "max_range_m") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "max_range_m must be a double";
        return result;
      }
      new_config.max_range_m = std::max(0.1, param.as_double());
    }
    else if (name == "samples_per_segment") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = "samples_per_segment must be an integer";
        return result;
      }
      new_config.samples_per_segment = std::max(2, static_cast<int>(param.as_int()));
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

  RCLCPP_INFO(this->get_logger(), "Parameters updated");
  return result;
}

void FieldLineDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!camera_info_received_) {
    camera_info_ = msg;
    
    // Build camera matrix K (3x3)
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                      msg->k[0], msg->k[1], msg->k[2],
                      msg->k[3], msg->k[4], msg->k[5],
                      msg->k[6], msg->k[7], msg->k[8]);
    
    // Build distortion coefficients (handle variable length)
    if (msg->d.size() > 0) {
      dist_coeffs_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
      for (size_t i = 0; i < msg->d.size(); ++i) {
        dist_coeffs_.at<double>(0, i) = msg->d[i];
      }
    } else {
      dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    }
    
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d, D size=%zu", 
                msg->width, msg->height, msg->d.size());
  }
}

void FieldLineDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  detectLines(cv_ptr->image, msg->header.stamp);
}

void FieldLineDetector::detectLines(const cv::Mat& bgr_image, const builtin_interfaces::msg::Time& stamp)
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
              cv::Scalar(0, 0, cfg.hsv_v_min),
              cv::Scalar(180, cfg.hsv_s_max, 255),
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

  // 6. Project lines to ground plane
  projectToGround(lines, stamp, y0);

  // 7. Draw segments on debug image
  cv::Mat debug_image = bgr_image.clone();
  cv::line(debug_image, cv::Point(0, y0), cv::Point(debug_image.cols, y0),
           cv::Scalar(255, 255, 0), 1);
  
  for (const auto& line : lines) {
    cv::line(debug_image, 
             cv::Point(line[0], line[1] + y0), 
             cv::Point(line[2], line[3] + y0),
             cv::Scalar(0, 255, 0), 2);
  }

  std::string text = "Lines: " + std::to_string(lines.size());
  cv::putText(debug_image, text, cv::Point(10, 30), 
              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

  // Publish debug image
  auto debug_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg();
  debug_msg->header.stamp = stamp;
  debug_msg->header.frame_id = camera_frame_;
  debug_pub_->publish(*debug_msg);

  // Publish mask
  cv::Mat full_mask = cv::Mat::zeros(bgr_image.rows, bgr_image.cols, CV_8UC1);
  mask.copyTo(full_mask(cv::Range(y0, bgr_image.rows), cv::Range::all()));
  
  auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", full_mask).toImageMsg();
  mask_msg->header.stamp = stamp;
  mask_msg->header.frame_id = camera_frame_;
  mask_pub_->publish(*mask_msg);
}

void FieldLineDetector::projectToGround(const std::vector<cv::Vec4i>& lines,
                                         const builtin_interfaces::msg::Time& stamp,
                                         int roi_y_offset)
{
  if (!camera_info_received_) return;

  // Get config
  Config cfg;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    cfg = config_;
  }

  // TF lookup: camera_frame -> output_frame
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(output_frame_, camera_frame_, 
                                      rclcpp::Time(0),
                                      rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", e.what());
    return;
  }

  // Extract camera position and rotation in output frame
  Eigen::Vector3d cam_origin(tf.transform.translation.x,
                              tf.transform.translation.y,
                              tf.transform.translation.z);
  Eigen::Quaterniond cam_rot(tf.transform.rotation.w,
                              tf.transform.rotation.x,
                              tf.transform.rotation.y,
                              tf.transform.rotation.z);

  std::vector<Eigen::Vector3d> ground_points;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ground_segments;

  for (const auto& line : lines) {
    Eigen::Vector3d seg_start, seg_end;
    bool start_valid = false, end_valid = false;

    // Sample points along segment
    for (int s = 0; s < cfg.samples_per_segment; ++s) {
      double t = (cfg.samples_per_segment == 1) ? 0.5 : 
                 static_cast<double>(s) / (cfg.samples_per_segment - 1);
      
      // Interpolate pixel (apply ROI offset!)
      double px = line[0] + t * (line[2] - line[0]);
      double py = line[1] + t * (line[3] - line[1]) + roi_y_offset;

      // Undistort to normalized coords
      std::vector<cv::Point2d> pts_in = {{px, py}};
      std::vector<cv::Point2d> pts_out;
      cv::undistortPoints(pts_in, pts_out, camera_matrix_, dist_coeffs_);

      // Ray in camera frame (optical: Z forward)
      Eigen::Vector3d ray_cam(pts_out[0].x, pts_out[0].y, 1.0);
      ray_cam.normalize();

      // Transform ray to output frame
      Eigen::Vector3d ray_out = cam_rot * ray_cam;

      // Ray-ground intersection: z = ground_z
      if (ray_out.z() >= -1e-6) continue;  // pointing up, skip
      double t_hit = (cfg.ground_z - cam_origin.z()) / ray_out.z();
      if (t_hit <= 0) continue;

      Eigen::Vector3d pt = cam_origin + t_hit * ray_out;

      // Range filter (planar distance from camera projection)
      double dist = std::hypot(pt.x() - cam_origin.x(), pt.y() - cam_origin.y());
      if (dist > cfg.max_range_m) continue;

      ground_points.push_back(pt);

      // Track segment endpoints
      if (s == 0) {
        seg_start = pt;
        start_valid = true;
      }
      if (s == cfg.samples_per_segment - 1) {
        seg_end = pt;
        end_valid = true;
      }
    }

    // Store valid segment
    if (start_valid && end_valid) {
      ground_segments.push_back({seg_start, seg_end});
    }
  }

  // Publish
  publishPointCloud(ground_points, stamp);
  publishMarkers(ground_points, ground_segments, stamp);
}

void FieldLineDetector::publishPointCloud(const std::vector<Eigen::Vector3d>& points,
                                           const builtin_interfaces::msg::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = output_frame_;
  cloud.height = 1;
  cloud.width = points.size();
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto& pt : points) {
    *iter_x = static_cast<float>(pt.x());
    *iter_y = static_cast<float>(pt.y());
    *iter_z = static_cast<float>(pt.z());
    ++iter_x; ++iter_y; ++iter_z;
  }

  ground_points_pub_->publish(cloud);
}

void FieldLineDetector::publishMarkers(const std::vector<Eigen::Vector3d>& points,
                                        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& segments,
                                        const builtin_interfaces::msg::Time& stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Marker 1: Points
  visualization_msgs::msg::Marker points_marker;
  points_marker.header.stamp = stamp;
  points_marker.header.frame_id = output_frame_;
  points_marker.ns = "ground_points";
  points_marker.id = 0;
  points_marker.type = visualization_msgs::msg::Marker::POINTS;
  points_marker.action = visualization_msgs::msg::Marker::ADD;
  points_marker.scale.x = 0.02;
  points_marker.scale.y = 0.02;
  points_marker.color.r = 0.0;
  points_marker.color.g = 1.0;
  points_marker.color.b = 0.0;
  points_marker.color.a = 1.0;
  points_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  for (const auto& pt : points) {
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    points_marker.points.push_back(p);
  }
  marker_array.markers.push_back(points_marker);

  // Marker 2: Line segments (LINE_LIST)
  visualization_msgs::msg::Marker lines_marker;
  lines_marker.header.stamp = stamp;
  lines_marker.header.frame_id = output_frame_;
  lines_marker.ns = "ground_segments";
  lines_marker.id = 1;
  lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines_marker.action = visualization_msgs::msg::Marker::ADD;
  lines_marker.scale.x = 0.01;
  lines_marker.color.r = 1.0;
  lines_marker.color.g = 1.0;
  lines_marker.color.b = 0.0;
  lines_marker.color.a = 1.0;
  lines_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  for (const auto& seg : segments) {
    geometry_msgs::msg::Point p1, p2;
    p1.x = seg.first.x();
    p1.y = seg.first.y();
    p1.z = seg.first.z();
    p2.x = seg.second.x();
    p2.y = seg.second.y();
    p2.z = seg.second.z();
    lines_marker.points.push_back(p1);
    lines_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(lines_marker);

  markers_pub_->publish(marker_array);
}

}  // namespace op3_field_vision

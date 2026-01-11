#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class FieldMapPublisher : public rclcpp::Node
{
public:
  FieldMapPublisher()
  : Node("field_map_publisher")
  {
    // Declare parameters
    this->declare_parameter("frame_id", "odom");
    this->declare_parameter("publish_rate_hz", 1.0);
    this->declare_parameter("line_width", 0.05);
    this->declare_parameter("z_offset", 0.001);
    this->declare_parameter("line_color.r", 1.0);
    this->declare_parameter("line_color.g", 1.0);
    this->declare_parameter("line_color.b", 1.0);
    this->declare_parameter("line_color.a", 1.0);
    this->declare_parameter("segments_flat", std::vector<double>{});

    // Get parameters
    frame_id_ = this->get_parameter("frame_id").as_string();
    double rate = this->get_parameter("publish_rate_hz").as_double();
    line_width_ = this->get_parameter("line_width").as_double();
    z_offset_ = this->get_parameter("z_offset").as_double();
    
    line_color_.r = static_cast<float>(this->get_parameter("line_color.r").as_double());
    line_color_.g = static_cast<float>(this->get_parameter("line_color.g").as_double());
    line_color_.b = static_cast<float>(this->get_parameter("line_color.b").as_double());
    line_color_.a = static_cast<float>(this->get_parameter("line_color.a").as_double());

    segments_flat_ = this->get_parameter("segments_flat").as_double_array();

    // Validate segments (must be multiple of 4)
    if (segments_flat_.size() % 4 != 0) {
      RCLCPP_ERROR(this->get_logger(), 
                   "segments_flat must have length multiple of 4, got %zu", 
                   segments_flat_.size());
    }

    // Create publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/field_map/markers", 10);

    // Create timer
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FieldMapPublisher::publishFieldMarkers, this));

    RCLCPP_INFO(this->get_logger(), "Field map odomm publisher started");
    RCLCPP_INFO(this->get_logger(), "  Frame: %s, %zu segments", 
                frame_id_.c_str(), segments_flat_.size() / 4);
  }

private:
  void publishFieldMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker line_marker;

    // Header
    line_marker.header.stamp = this->now();
    line_marker.header.frame_id = frame_id_;

    // Marker properties
    line_marker.ns = "field_lines";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Scale (line width)
    line_marker.scale.x = line_width_;
    
    // Color
    line_marker.color = line_color_;
    
    // Pose (identity)
    line_marker.pose.orientation.w = 1.0;
    
    // Lifetime (0 = forever until replaced)
    line_marker.lifetime = rclcpp::Duration(0, 0);

    // Build line points from flat segment list
    for (size_t i = 0; i + 3 < segments_flat_.size(); i += 4) {
      geometry_msgs::msg::Point p1, p2;
      p1.x = segments_flat_[i];
      p1.y = segments_flat_[i + 1];
      p1.z = z_offset_;
      p2.x = segments_flat_[i + 2];
      p2.y = segments_flat_[i + 3];
      p2.z = z_offset_;
      
      line_marker.points.push_back(p1);
      line_marker.points.push_back(p2);
    }

    marker_array.markers.push_back(line_marker);
    marker_pub_->publish(marker_array);
  }

  // Parameters
  std::string frame_id_;
  double line_width_;
  double z_offset_;
  std_msgs::msg::ColorRGBA line_color_;
  std::vector<double> segments_flat_;

  // Publisher and timer
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FieldMapPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

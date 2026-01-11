#include <rclcpp/rclcpp.hpp>
#include "op3_field_vision/field_line_detector.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<op3_field_vision::FieldLineDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

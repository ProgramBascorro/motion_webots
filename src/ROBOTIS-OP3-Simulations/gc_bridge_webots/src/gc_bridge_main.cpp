#include <rclcpp/rclcpp.hpp>

#include "gc_bridge_webots/gc_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotis_op::GCBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

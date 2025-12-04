#include "op3_ball_detector/ball_detector.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try
  {
    auto node = std::make_shared<robotis_op::BallDetectorCpp>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
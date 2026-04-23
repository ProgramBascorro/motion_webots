#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class OpenCRReader : public rclcpp::Node
{
public:
  OpenCRReader() : Node("open_cr_reader")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/robotis/open_cr/imu", 10,
      std::bind(&OpenCRReader::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "OpenCR Reader node started - IMU mode");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "ACC: %.3f %.3f %.3f | GYRO: %.3f %.3f %.3f",
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z,
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenCRReader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

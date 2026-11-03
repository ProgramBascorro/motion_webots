// Tombol D58 di SUMBERNYA: open_cr_module::handleButton() ASLI diberi bit tombol
// palsu (tanpa OpenCR, domain ROS 80). Membuktikan satu tekanan fisik = satu
// pesan, menahan tombol tidak mengulang toggle, dan tekanan baru sesudah
// dilepas diterima lagi.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "open_cr_module/open_cr_module.h"
#include "robotis_device/sensor.h"

namespace
{

constexpr uint32_t kUserBit = 0x04;   // bit 2 = PIN_BUTTON_S3 = D58

TEST(OpenCRButtonEdge, OnePhysicalPressIsOneMessageAndHoldingDoesNotRepeat)
{
  rclcpp::init(0, nullptr);
  auto listener = rclcpp::Node::make_shared("opencr_button_edge_listener");
  std::vector<std::string> received;
  auto subscription = listener->create_subscription<std_msgs::msg::String>(
      "/robotis/open_cr/button", 50, [&received](std_msgs::msg::String::SharedPtr msg) {
        received.push_back(msg->data);
      });

  auto module = std::make_unique<robotis_op::OpenCRModule>();
  module->initialize(8, nullptr);   // publisher dibuat di utas antrean modul
  for (int i = 0; i < 200 && listener->count_publishers("/robotis/open_cr/button") == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_GT(listener->count_publishers("/robotis/open_cr/button"), 0u);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));   // discovery

  robotis_framework::Sensor sensor(200, "OPEN-CR", 2.0);
  sensor.sensor_state_->bulk_read_table_["mag_status"] = 2;
  std::map<std::string, robotis_framework::Sensor *> sensors = {{"open-cr", &sensor}};
  std::map<std::string, robotis_framework::Dynamixel *> dxls;

  // Siklus kendali 8 ms seperti robotis_controller.
  const auto hold = [&](bool pressed, double seconds) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < until)
    {
      sensor.sensor_state_->bulk_read_table_["button"] = pressed ? kUserBit : 0u;
      const rclcpp::Time now = module->get_clock()->now();
      sensor.sensor_state_->update_time_stamp_ =
          robotis_framework::TimeStamp(static_cast<long>(now.nanoseconds() / 1000000000LL),
                                       static_cast<long>(now.nanoseconds() % 1000000000LL));
      module->process(dxls, sensors);
      rclcpp::spin_some(listener);
      std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
  };
  const auto count = [&received](const std::string &name) {
    int n = 0;
    for (const std::string &item : received)
      n += (item == name) ? 1 : 0;
    return n;
  };

  hold(false, 0.3);

  // 1. Tekanan pendek: tepat satu "user", terbit saat DILEPAS.
  hold(true, 0.15);
  EXPECT_EQ(count("user"), 0) << "terbit sebelum dilepas";
  hold(false, 0.6);
  EXPECT_EQ(count("user"), 1);

  // 2. Ditahan 1,5 s (masih < 2 s): tidak ada pesan selama ditahan, satu saat dilepas.
  hold(true, 1.5);
  EXPECT_EQ(count("user"), 1) << "menahan tombol menerbitkan toggle berulang";
  hold(false, 0.6);
  EXPECT_EQ(count("user"), 2);

  // 3. Ditahan > 2 s: SATU "user_long" (milik op3_manager), TANPA "user" saat dilepas.
  hold(true, 2.6);
  hold(false, 0.6);
  EXPECT_EQ(count("user_long"), 1);
  EXPECT_EQ(count("user"), 2) << "tekanan panjang juga dihitung sebagai toggle";

  // 4. Pantulan kontak: dua tepi lepas berjarak ~60 ms dari SATU tekanan -> satu pesan.
  hold(true, 0.05);
  hold(false, 0.03);
  hold(true, 0.05);
  hold(false, 0.6);
  EXPECT_EQ(count("user"), 3) << "pantulan kontak terbaca sebagai dua toggle";

  // 5. Tekanan baru sesudah dilepas: diterima lagi.
  hold(true, 0.15);
  hold(false, 0.6);
  EXPECT_EQ(count("user"), 4);

  rclcpp::shutdown();   // utas antrean modul keluar dari while (rclcpp::ok())
  module.reset();
}

}  // namespace

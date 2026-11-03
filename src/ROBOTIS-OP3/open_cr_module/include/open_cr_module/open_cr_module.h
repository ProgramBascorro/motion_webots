/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman */

#ifndef OP3_OPEN_CR_MODULE_H_
#define OP3_OPEN_CR_MODULE_H_

#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Eigen>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_math/robotis_math_base.h"

namespace robotis_op
{

class OpenCRModule : public robotis_framework::SensorModule,
                     public robotis_framework::Singleton<OpenCRModule>,
                     public rclcpp::Node
{
public:
  OpenCRModule();
  virtual ~OpenCRModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  // Menunggu thread antrean modul selesai. Loop-nya berputar selama rclcpp::ok(),
  // jadi panggil SESUDAH rclcpp::shutdown(); dipakai op3_manager saat berhenti.
  void joinQueueThread();
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  const double G_ACC = 9.80665;
  const double GYRO_FACTOR = 2000.0 / 32800.0;
  const double ACCEL_FACTOR = 2.0 / 32768.0;
  const bool DEBUG_PRINT;

  void queueThread();

  double getGyroValue(int raw_value);
  double getAccValue(int raw_value);
  void publishIMU();

  void handleButton(const std::string &button_name);
  bool publishAllowed(const std::string &button_name);
  void publishButtonMsg(const std::string &button_name);
  void handleVoltage(double present_volt);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDXLPowerMsg(unsigned int value);
  double lowPassFilter(double alpha, double x_new, double &x_old);
  // Varian sirkular untuk sudut yang wrap di +-pi (yaw magnetometer).
  double lowPassFilterAngle(double alpha, double x_new, double &x_old);

  int control_cycle_msec_;
  std::thread queue_thread_;
  // TITIK NOL YAW (17 Sep 2026): pesan apa pun di /robotis/open_cr/reset_yaw
  // menjadikan arah robot SAAT ITU yaw 0. Diminta mode D60 saat mulai (tombol
  // D56/D58 dulu juga, dinonaktifkan 19 Sep 2026). Hanya yaw TERBITAN (/robotis/open_cr/imu)
  // yang digeser; state filter tetap di bingkai magnetometer.
  std::atomic<bool> yaw_zero_request_{false};
  double yaw_zero_offset_ = 0.0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_yaw_sub_;

  // KALIBRASI YAW N-TITIK (17 Sep 2026). Magnetometer ALPHONSE sangat tidak
  // linier (90 deg nyata terbaca 36..236 deg). Kalau aktif, yaw dihitung dari
  // mag_heading ABSOLUT lewat tabel interpolasi linier melingkar: heading
  // terukur -> arah sebenarnya, lalu relatif titik nol (awal nyala / reset_yaw).
  bool yaw_calib_enable_ = false;
  std::vector<double> yaw_calib_measured_deg_;   // mag_heading terukur tiap titik, urut arah nyata
  std::vector<double> yaw_calib_true_deg_;       // arah nyata tiap titik (naik, dalam 360)
  bool yaw_zero_initialized_ = false;
  // Status magnetometer sah (2/3) terakhir. 19 Sep 2026: sampel mag_status 0/1
  // SESUDAH pernah sah tidak dipakai -- yaw dan status ditahan di nilai sah
  // terakhir (tidak jadi 0). 0 = belum pernah sah (perilaku lama).
  uint8_t last_valid_mag_status_ = 0;
  double correctHeadingDeg(double heading_deg) const;
  // Heading dihitung DI PC dari komponen magnetometer mentah (register mag_x/y/z,
  // kerangka badan robot), sesudah offset hard-iron dikurangkan dan dikompensasi
  // kemiringan dengan roll/pitch dari akselerometer. Rumusnya sama dengan
  // firmware; bedanya offsetnya milik mini PC ini (yaw_calibration.yaml), bukan
  // EEPROM OpenCR -- operator minta kalibrasi tertanam di PC, board tidak ditulis.
  // Kembalian: derajat 0..360, atau NaN kalau medannya terlalu lemah.
  double headingFromRawMag(double mx, double my, double mz);

  bool mag_hard_iron_enable_ = false;
  double mag_hard_iron_offset_[3] = {0.0, 0.0, 0.0};
  // Penghalus sin/cos supaya lompatan 359 -> 0 tidak merusak rata-rata (sama
  // dengan firmware). Dipakai hanya di jalur heading versi PC.
  bool mag_flt_init_ = false;
  double mag_flt_sin_ = 0.0;
  double mag_flt_cos_ = 0.0;
  std::map<std::string, bool> buttons_;
  std::map<std::string, rclcpp::Time> buttons_press_time_;
  // Kapan terakhir kali nama tombol ini benar-benar diterbitkan. Dipakai
  // sebagai debounce: lihat penjelasan panjang di handleButton().
  std::map<std::string, rclcpp::Time> buttons_publish_time_;
  double button_debounce_sec_;
  // Tombol D60 (mode otomatis): mask bit di register Button, 0 = mati. Lihat
  // konstruktor (parameter d60_button_bit).
  uint8_t d60_button_mask_;
  uint8_t unmapped_button_bits_;
  rclcpp::Time button_press_time_;
  rclcpp::Time last_msg_time_;
  std::map<std::string, double> previous_result_;
  double previous_volt_;
  double present_volt_;

  sensor_msgs::msg::Imu imu_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_pub_;
  // Status magnetometer (0=tak ada, 1=settling, 2=OK, 3=kalibrasi) -- dipakai Bascorro
  // Studio untuk menandai apakah yaw layak dipercaya.
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mag_status_pub_;
  // mag_heading MENTAH firmware (deg, 0..360, absolut), TANPA tabel kalibrasi
  // dan tanpa titik nol. Hanya diterbitkan saat mag_status 2/3. Sumber data
  // alat kalibrasi yaw 5 titik (scripts/calib_imu_yaw.py): kalibrasi harus
  // dibaca dari sensor apa adanya, bukan dari yaw yang sudah dikoreksi.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mag_heading_pub_;
  // Komponen magnetometer MENTAH (kerangka badan, sebelum offset PC dikurangkan).
  // Ini bahan kalibrasi hard-iron versi PC: perekamnya butuh x/y/z, bukan heading.
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mag_raw_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr dxl_power_msg_pub_;
};

}  // namespace robotis_op

#endif  // OP3_OPEN_CR_MODULE_H_

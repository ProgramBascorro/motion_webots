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

#include <array>
#include <atomic>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

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
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);

private:
  const double G_ACC = 9.80665;
  const double GYRO_FACTOR = 2000.0 / 32800.0;
  const double ACCEL_FACTOR = 2.0 / 32768.0;

  // Register roll/pitch/yaw firmware berisi int16 satuan 0.1 derajat
  // (dxl_hw_op3_get_rpy: (IMU.rpy - offset) * 10).
  const double RPY_FACTOR = 0.1;

  // AK8963 di dalam MPU9250, mode 16 bit: 0.15 uT per LSB.
  const double MAG_FACTOR = 0.15e-6;    // LSB -> Tesla

  // Nilai register mag_status, dari dxl_hw_op3.h.
  enum MagStatus
  {
    MAG_STATUS_NO_SENSOR   = 0,
    MAG_STATUS_SETTLING    = 1,
    MAG_STATUS_OK          = 2,
    MAG_STATUS_CALIBRATING = 3,
  };

  // Bit perintah register imu_control, dari dxl_node_op3.h.
  enum ImuControlBit
  {
    IMU_CTRL_CALI_GYRO = (1 << 3),
    IMU_CTRL_CALI_MAG  = (1 << 4),
    IMU_CTRL_ZERO_YAW  = (1 << 5),
  };

  const bool DEBUG_PRINT;

  void queueThread();

  double getGyroValue(int raw_value);
  double getAccValue(int raw_value);
  void publishIMU();
  void publishMag();
  void publishMagStatus();
  void imuCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  void resetYawCallback(const std_msgs::msg::String::SharedPtr msg);
  void writeImuControl(uint8_t bits);

  void handleButton(const std::string &button_name);
  void publishButtonMsg(const std::string &button_name);
  void handleVoltage(double present_volt);
  void publishStatusMsg(unsigned int type, std::string msg);
  void publishDXLPowerMsg(unsigned int value);
  double lowPassFilter(double alpha, double x_new, double &x_old);
  // Sama seperti lowPassFilter, tapi untuk sudut (rad): selisihnya dirapikan ke
  // -pi..pi dulu, jadi lompatan 359 -> 0 derajat tidak membuat filter berputar
  // balik satu lingkaran penuh.
  double lowPassFilterAngle(double alpha, double x_new, double &x_old);

  int control_cycle_msec_;
  std::thread queue_thread_;
  std::map<std::string, bool> buttons_;
  std::map<std::string, rclcpp::Time> buttons_press_time_;
  rclcpp::Time button_press_time_;
  rclcpp::Time last_msg_time_;
  std::map<std::string, double> previous_result_;
  double previous_volt_;
  double present_volt_;

  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField mag_msg_;

  // Nilai magnetometer terakhir yang dibaca dari sub controller. mag_present_ =
  // mag_status 2 (OK) atau 3 (kalibrasi hard-iron); di luar itu yaw tidak sah.
  bool mag_present_;
  uint8_t mag_status_;
  double mag_yaw_rad_;       // yaw yang diterbitkan (sudah dikurangi titik nol)
  double mag_heading_deg_;   // mag_heading mentah, 0..360
  double yaw_gyro_deg_;

  // Yaw GYRO relatif terhadap reference "zero_yaw" (mag_status[7]). Tidak
  // disentuh kalibrasi yaw; atomic karena ditulis callback imu_command.
  std::atomic<double> last_raw_yaw_gyro_deg_;
  std::atomic<double> yaw_gyro_offset_deg_;

  //== KALIBRASI YAW 5 TITIK (magnetometer) ================================
  //
  // Tabel dari scripts/calib_imu_yaw.py, dimuat lewat parameter node dari
  // op3_manager/config/yaw_calibration.yaml (op3_manager.launch.py). Tabel
  // memetakan mag_heading MENTAH ke arah nyata (0/90/180/270 = putar kiri),
  // interpolasi linear per potongan -- hanya meluruskan SKALA sudut.
  //
  // Titik nol: sampel sah PERTAMA sesudah manager start, dan setiap pesan
  // /robotis/open_cr/reset_yaw (juga imu_command "zero_yaw"). Offset diambil di
  // process() (thread kontrol, satu thread dengan result_), bukan di callback.
  //
  // Parameter tidak sah (ukuran/bentang salah) -> kalibrasi MATI, yaw firmware
  // mentah yang dipakai.
  bool yaw_calib_enabled_;
  std::vector<double> yaw_calib_measured_deg_;
  std::vector<double> yaw_calib_true_deg_;
  double correctHeadingDeg(double heading_deg) const;
  void loadYawCalibParams();

  std::atomic<bool> yaw_zero_request_;
  double yaw_zero_offset_;         // rad, hanya thread process()
  bool yaw_zero_initialized_;      // hanya thread process()
  bool yaw_filter_valid_;          // hanya thread process()
  double yaw_filtered_;            // keadaan lowPassFilterAngle (rad)

  //== KOMPENSASI HANYUTAN GYRO (kode lama 2026-09-11/13, dipasang lagi) ====
  // Diukur 2026-09-11, robot diam 60 detik: yaw gyro naik +4,3 deg/menit --
  // BIAS sensor, besarnya berubah antar-nyala (0,01..0,75 deg/s), jadi
  // diestimasi terus-menerus:
  //   diam     (rentang gyro_z 1 dtk kecil) -> yaw gyro DITAHAN persis, bias
  //            dipelajari = total perubahan yaw mentah / total waktu diam;
  //   bergerak -> bias terakhir dikurangkan tiap siklus.
  // Hasilnya yaw_gyro_corr_deg_ (mag_status[9]).
  //
  // Dipakai bersama tabel 5 titik (hanya saat tabel aktif): yaw = gyro terkoreksi
  // yang ditarik ke kompas terkalibrasi HANYA saat diam (kompas terganggu arus
  // servo saat bergerak). Tabel mati -> yaw firmware mentah seperti spesifikasi.
  //
  // Semua anggota ini hanya disentuh thread process(), jadi bukan atomik.
  static constexpr size_t GZ_WINDOW = 125;           // 1 detik pada siklus 8 ms
  // DIAM DINILAI DARI KETERATURAN GYRO (2026-09-19, revisi kedua).
  // Patokan lama |rerata gyro - bias| < 0,5 macet saat bias besar (+2,47 deg/s:
  // robot diam tak pernah dianggap diam). Patokan kompas yang menggantikannya
  // macet saat medan terganggu: robot diam, gyro_z 0, tapi heading berkeliaran
  // 8 deg dalam 10 dtk -> diam hanya 15% waktu. Yang tahan terhadap keduanya:
  // RENTANG gyro_z dalam 1 dtk. Bias konstan sebesar apa pun tidak mengubah
  // rentangnya; gerakan nyata selalu mengubahnya. Diukur: diam maks 0,19 deg/s,
  // berjalan p5 2,02 deg/s (median 156) -> 0,5 memisahkan dengan margin lebar.
  static constexpr double STILL_GZ_RANGE_DPS = 0.5;
  // Bias gyro sebesar ini tidak bisa dikompensasi dengan mulus: minta firmware
  // mengkalibrasi ulang gyro (imu_control bit 3) saat robot terbukti diam.
  static constexpr double AUTO_GYRO_CAL_DPS = 1.0;
  static constexpr double AUTO_GYRO_CAL_STILL_SEC = 3.0;
  static constexpr double AUTO_GYRO_CAL_RETRY_SEC = 10.0;
  static constexpr int AUTO_GYRO_CAL_MAX = 3;
  static constexpr double GYRO_BIAS_TAU_SEC = 20.0;
  static constexpr double GYRO_BIAS_MIN_SEC = 5.0;
  std::array<double, GZ_WINDOW> gz_window_;
  size_t gz_head_;
  size_t gz_count_;
  double gyro_bias_dps_;
  double gyro_still_sum_deg_;   // jumlah perubahan yaw mentah saat diam (terlupa)
  double gyro_still_time_s_;    // jumlah waktu diam (terlupa sama rata)
  bool gyro_bias_valid_;
  double gyro_drift_corr_deg_;
  double prev_raw_gyro_deg_;
  bool prev_raw_gyro_valid_;
  bool gyro_still_;
  double yaw_gyro_corr_deg_;
  double still_sec_;              // sudah berapa lama diam menerus
  int auto_gyro_cal_count_;       // berapa kali kalibrasi gyro otomatis dikirim
  double auto_gyro_cal_wait_;     // sisa tunggu sebelum boleh mencoba lagi
  // Bias gyro tersimpan di NUC (op3_manager/config/imu_drift.yaml, ditulis
  // scripts/calib_imu_drift.py) -- dipakai sejak siklus pertama, sebelum ada
  // waktu diam untuk mempelajarinya sendiri.
  double gyro_bias_param_dps_;
  void updateGyroDrift();

  // Penggabungan gyro terkoreksi + kompas terkalibrasi (deg, 0..360 bebas).
  // Tarikan ke kompas saat diam. Semula 1 dtk; 2026-09-19 kompas terukur
  // berkeliaran 8 deg dalam 10 dtk walau robot diam (medan terganggu), dan
  // tarikan 1 dtk membuat yaw ikut berkeliaran. Gyro sesudah kalibrasi drift
  // hampir tak menghanyut, jadi kompas cukup meluruskan jangka panjang.
  static constexpr double FUSE_TAU_SEC = 30.0;
  static constexpr double FUSE_SNAP_DEG = 30.0;  // selisih sebesar ini selama ...
  static constexpr double FUSE_SNAP_SEC = 3.0;   // ... sekian detik diam -> lompat ke kompas
  bool fused_valid_;
  double fused_yaw_deg_;
  double prev_gyro_corr_deg_;
  double fuse_far_sec_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr dxl_power_msg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mag_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mag_heading_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_yaw_sub_;
};

}  // namespace robotis_op

#endif  // OP3_OPEN_CR_MODULE_H_

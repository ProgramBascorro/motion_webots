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

#include <cmath>
#include <limits>
#include <sstream>

#include "open_cr_module/open_cr_module.h"

namespace robotis_op
{

// Penanda "heading tidak bisa dihitung" -- dipakai jalur hard-iron versi PC.
static const double kNoHeading = std::numeric_limits<double>::quiet_NaN();

OpenCRModule::OpenCRModule()
: Node("open_cr_module"),
  control_cycle_msec_(8),
  DEBUG_PRINT(false),
  previous_volt_(0.0),
  present_volt_(0.0)
{
  module_name_ = "open_cr_module";

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button_mode"] = 0.0;
  result_["button_start"] = 0.0;
  result_["button_user"] = 0.0;

  result_["present_voltage"] = 0.0;

  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["button_user"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;
  buttons_["published_user"] = false;

  // Tombol D60 = mode otomatis (autonomous demo) di op_demo_node. Firmware
  // OpenCR bawaan hanya mengemas S1..S4 (D56..D59) di register Button; firmware
  // yang sudah "di-embed" D60 menaruhnya di bit berikutnya. Bit-nya dibuka
  // sebagai parameter supaya bisa dipindah tanpa build ulang kalau firmware
  // memakai bit lain: bawaan 4 (mask 0x10). Bit 3 (D59) SENGAJA bukan bawaan:
  // firmware mematikan power DXL kalau D59 ditahan > 100 ms.
  result_["button_d60"] = 0.0;
  buttons_["button_d60"] = false;
  buttons_["published_d60"] = false;
  {
    int bit = 4;
    try {
      bit = this->declare_parameter<int>("d60_button_bit", 4);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "d60_button_bit tidak terbaca (%s); memakai bit 4", e.what());
    }
    if (bit < 0 || bit > 7) {
      RCLCPP_WARN(this->get_logger(), "d60_button_bit %d di luar 0..7; tombol D60 dimatikan", bit);
      d60_button_mask_ = 0;
    } else {
      d60_button_mask_ = static_cast<uint8_t>(1u << bit);
    }
  }
  unmapped_button_bits_ = 0;

  // Kalibrasi yaw N-titik. Bawaan MATI (perilaku lama: yaw firmware).
  try {
    yaw_calib_enable_ = this->declare_parameter<bool>("yaw_calib_enable", false);
    yaw_calib_measured_deg_ =
      this->declare_parameter<std::vector<double>>("yaw_calib_measured_heading_deg", std::vector<double>{});
    yaw_calib_true_deg_ = this->declare_parameter<std::vector<double>>("yaw_calib_true_deg", std::vector<double>{});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "parameter yaw_calib_* tidak terbaca (%s); kalibrasi yaw MATI", e.what());
    yaw_calib_enable_ = false;
  }
  // Kalibrasi HARD-IRON versi MINI PC. Offsetnya dikurangkan dari komponen
  // magnetometer mentah di sini, bukan ditulis ke EEPROM OpenCR: board ini tidak
  // menanggapi perintah tulis kalibrasi (terbukti 19 Sep: imu_control bit 4
  // dikirim, mag_status tidak pernah jadi 3), dan operator memang ingin
  // kalibrasinya tertanam di PC.
  try {
    mag_hard_iron_enable_ = this->declare_parameter<bool>("mag_hard_iron_enable", false);
    const std::vector<double> offset =
      this->declare_parameter<std::vector<double>>("mag_hard_iron_offset", std::vector<double>{});
    if (offset.size() == 3) {
      for (size_t i = 0; i < 3; ++i)
        mag_hard_iron_offset_[i] = offset[i];
    } else if (mag_hard_iron_enable_) {
      RCLCPP_ERROR(this->get_logger(),
                   "mag_hard_iron_offset harus 3 angka (x, y, z); kalibrasi hard-iron PC DIMATIKAN");
      mag_hard_iron_enable_ = false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "parameter mag_hard_iron_* tidak terbaca (%s); kalibrasi PC MATI", e.what());
    mag_hard_iron_enable_ = false;
  }
  if (mag_hard_iron_enable_) {
    RCLCPP_WARN(this->get_logger(),
                "KALIBRASI HARD-IRON PC AKTIF: heading dihitung di sini dari mag_x/y/z dikurangi "
                "offset (%.1f, %.1f, %.1f), BUKAN dari mag_heading firmware",
                mag_hard_iron_offset_[0], mag_hard_iron_offset_[1], mag_hard_iron_offset_[2]);
  }

  if (yaw_calib_enable_) {
    bool ok = yaw_calib_measured_deg_.size() >= 3 && yaw_calib_measured_deg_.size() == yaw_calib_true_deg_.size();
    double span_total = 0.0;
    for (size_t i = 0; ok && i < yaw_calib_true_deg_.size(); ++i) {
      const size_t j = (i + 1) % yaw_calib_true_deg_.size();
      const double true_span = std::fmod(yaw_calib_true_deg_[j] - yaw_calib_true_deg_[i] + 720.0, 360.0);
      const double meas_span = std::fmod(yaw_calib_measured_deg_[j] - yaw_calib_measured_deg_[i] + 720.0, 360.0);
      if (true_span <= 0.0 || meas_span <= 0.0)
        ok = false;
      span_total += meas_span;
    }
    if (!ok || std::fabs(span_total - 360.0) > 1.0) {
      RCLCPP_ERROR(this->get_logger(),
                   "tabel kalibrasi yaw tidak sah (butuh >=3 titik, arah maju, total 360; total %.1f): kalibrasi MATI",
                   span_total);
      yaw_calib_enable_ = false;
    } else {
      std::string text;
      for (size_t i = 0; i < yaw_calib_true_deg_.size(); ++i)
        text += (i ? ", " : "") + std::to_string(yaw_calib_measured_deg_[i]).substr(0, 6) + "->" +
                std::to_string(yaw_calib_true_deg_[i]).substr(0, 5);
      RCLCPP_WARN(this->get_logger(), "KALIBRASI YAW AKTIF (%zu titik, mag_heading -> arah nyata): %s",
                  yaw_calib_true_deg_.size(), text.c_str());
    }
  }

  // Jendela debounce tombol. Lihat handleButton() untuk alasannya.
  button_debounce_sec_ = 0.40;

  previous_result_["gyro_x"] = 0.0;
  previous_result_["gyro_y"] = 0.0;
  previous_result_["gyro_z"] = 0.0;
  previous_result_["gyro_x_prev"] = 0.0;
  previous_result_["gyro_y_prev"] = 0.0;
  previous_result_["gyro_z_prev"] = 0.0;

  previous_result_["acc_x"] = 0.0;
  previous_result_["acc_y"] = 0.0;
  previous_result_["acc_z"] = 0.0;

  last_msg_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
}

OpenCRModule::~OpenCRModule()
{
  if (queue_thread_.joinable()) {
    queue_thread_.join();
  }
}

void OpenCRModule::joinQueueThread()
{
  if (queue_thread_.joinable())
    queue_thread_.join();
}

void OpenCRModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  (void) robot;
  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = std::thread(&OpenCRModule::queueThread, this);
}

void OpenCRModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/robotis/open_cr/imu", 1);
  button_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/open_cr/button", 1);
  mag_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/robotis/open_cr/mag_status", 1);
  mag_heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robotis/open_cr/mag_heading", 10);
  mag_raw_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/robotis/open_cr/mag_raw", 10);
  reset_yaw_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robotis/open_cr/reset_yaw", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      yaw_zero_request_.store(true);
      RCLCPP_INFO(this->get_logger(), "reset yaw diminta (%s)", msg->data.c_str());
    });
  dxl_power_msg_pub_ =
    this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 1);

  rclcpp::WallRate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok()) {
    executor.spin_some();
    rate.sleep();
  }
}

void OpenCRModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, robotis_framework::Sensor *> sensors)
{
  (void) dxls;
  if (sensors["open-cr"] == nullptr) {
    return;
  }

  int16_t gyro_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_x"];
  int16_t gyro_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_y"];
  int16_t gyro_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["gyro_z"];

  int16_t acc_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_x"];
  int16_t acc_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_y"];
  int16_t acc_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["acc_z"];

  uint16_t present_volt = sensors["open-cr"]->sensor_state_->bulk_read_table_["present_voltage"];

  // Yaw magnetometer, firmware OpenCR versi 3 ke atas.
  // bulk_read_table_ bertipe uint32_t dan framework tidak melakukan sign-extension,
  // jadi cast ke int16_t wajib -- sama seperti gyro/acc di atas.
  int16_t yaw_raw = sensors["open-cr"]->sensor_state_->bulk_read_table_["yaw"];
  uint8_t mag_status = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_status"];

  result_["gyro_x"] = lowPassFilter(0.4, -getGyroValue(gyro_x), previous_result_["gyro_x"]);
  result_["gyro_y"] = lowPassFilter(0.4, -getGyroValue(gyro_y), previous_result_["gyro_y"]);
  result_["gyro_z"] = lowPassFilter(0.4, getGyroValue(gyro_z), previous_result_["gyro_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Gyro ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", gyro_x, gyro_y, gyro_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f",
                         result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  result_["acc_x"] = lowPassFilter(0.4, -getAccValue(acc_x), previous_result_["acc_x"]);
  result_["acc_y"] = lowPassFilter(0.4, -getAccValue(acc_y), previous_result_["acc_y"]);
  result_["acc_z"] = lowPassFilter(0.4, getAccValue(acc_z), previous_result_["acc_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Acc ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", acc_x, acc_y, acc_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f",
                         result_["acc_x"], result_["acc_y"], result_["acc_z"]);

  // mag_status : 0 = tidak ada magnetometer, 1 = settling, 2 = OK, 3 = kalibrasi hard-iron.
  // Yaw hanya dipakai kalau sensornya memang melapor siap; kalau tidak, tetap 0 seperti
  // perilaku lama supaya orientasi tidak diisi angka karangan.
  // Komponen magnetometer mentah (kerangka badan robot, int16 -- framework tidak
  // melakukan sign-extension, jadi cast wajib seperti gyro/acc).
  const double mag_x = static_cast<int16_t>(sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_x"]);
  const double mag_y = static_cast<int16_t>(sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_y"]);
  const double mag_z = static_cast<int16_t>(sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_z"]);
  result_["mag_x"] = mag_x;
  result_["mag_y"] = mag_y;
  result_["mag_z"] = mag_z;

  // Sampel "tidak ada magnetometer"/settling SESUDAH pernah sah: jangan diambil.
  // Yaw tertapis dan status ditahan di nilai sah terakhir (tidak dipaksa 0).
  const bool mag_valid = (mag_status == 2 || mag_status == 3);
  const bool mag_hold = !mag_valid && last_valid_mag_status_ != 0;
  if (mag_valid)
    last_valid_mag_status_ = mag_status;
  if (mag_hold)
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "mag_status %d dari OpenCR diabaikan: yaw ditahan di nilai terakhir %.1f deg",
                         mag_status, result_["yaw"] * RADIAN2DEGREE);
  const uint8_t effective_mag_status = mag_hold ? last_valid_mag_status_ : mag_status;

  result_["mag_status"] = effective_mag_status;
  if (mag_hold) {
    // result_["yaw"] dan previous_result_["yaw"] sengaja tidak disentuh.
  } else if (mag_valid) {
    double yaw_rad = (static_cast<double>(yaw_raw) / 10.0) * DEGREE2RADIAN;
    if (yaw_calib_enable_ || mag_hard_iron_enable_) {
      // Heading absolut: versi PC (mag mentah - offset hard-iron) kalau kalibrasi
      // PC aktif, kalau tidak pakai mag_heading firmware seperti sebelumnya.
      double heading_deg = kNoHeading;
      if (mag_hard_iron_enable_)
        heading_deg = headingFromRawMag(mag_x, mag_y, mag_z);
      if (!std::isfinite(heading_deg)) {
        uint16_t heading_raw = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_heading"];
        heading_deg = static_cast<double>(heading_raw) / 10.0;
      }
      result_["mag_heading"] = heading_deg;
      yaw_rad = (yaw_calib_enable_ ? correctHeadingDeg(heading_deg) : heading_deg) * DEGREE2RADIAN;
      if (!yaw_zero_initialized_) {
        previous_result_["yaw"] = yaw_rad;
        yaw_zero_offset_ = yaw_rad;
        yaw_zero_initialized_ = true;
      }
    }
    result_["yaw"] = lowPassFilterAngle(0.4, yaw_rad, previous_result_["yaw"]);
    // Titik nol diambil di loop kontrol (satu utas dengan result_), dari yaw
    // tertapis saat ini.
    if (yaw_zero_request_.exchange(false)) {
      yaw_zero_offset_ = result_["yaw"];
      RCLCPP_WARN(this->get_logger(), "YAW DIRESET: arah sekarang = 0 deg (offset magnetometer %.1f deg)",
                  yaw_zero_offset_ * RADIAN2DEGREE);
    }
  } else {
    // Sensor belum siap: kembalikan ke perilaku lama sekaligus reset state filter,
    // supaya saat sensor siap lagi filter tidak merayap dari nilai basi.
    result_["yaw"] = 0.0;
    previous_result_["yaw"] = 0.0;
  }

  if (mag_status == 0 && !mag_hold) {
    static bool no_mag_reported = false;
    if (!no_mag_reported) {
      no_mag_reported = true;
      publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_WARN,
                       "OpenCR mag_status=0 : board tidak punya magnetometer, yaw dipaksa 0");
    }
  }

  std_msgs::msg::UInt8 mag_status_msg;
  mag_status_msg.data = effective_mag_status;
  mag_status_pub_->publish(mag_status_msg);

  if ((mag_status == 2 || mag_status == 3) && mag_heading_pub_) {
    // Yang diterbitkan = heading yang BENAR-BENAR dipakai jalur yaw: versi PC
    // kalau kalibrasi hard-iron aktif, kalau tidak versi firmware. Kalau yang
    // diterbitkan beda dengan yang dipakai, tabel yaw 5 titik akan dibuat dari
    // angka yang salah.
    double published = result_.count("mag_heading") ? result_["mag_heading"] : kNoHeading;
    if (!std::isfinite(published))
      published = static_cast<double>(sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_heading"]) / 10.0;
    std_msgs::msg::Float32 heading_msg;
    heading_msg.data = static_cast<float>(published);
    mag_heading_pub_->publish(heading_msg);
  }

  if (mag_raw_pub_) {
    geometry_msgs::msg::Vector3 raw_msg;
    raw_msg.x = mag_x;
    raw_msg.y = mag_y;
    raw_msg.z = mag_z;
    mag_raw_pub_->publish(raw_msg);
  }

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Yaw ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d (%.1f deg), mag_status : %d",
                         yaw_raw, yaw_raw / 10.0, mag_status);

  const auto& update_stamp = sensors["open-cr"]->sensor_state_->update_time_stamp_;
  rclcpp::Time update_time(
    update_stamp.sec_,
    update_stamp.nsec_,
    this->get_clock()->get_clock_type());
  rclcpp::Duration update_duration = this->get_clock()->now() - update_time;
  if (update_duration.nanoseconds() > 100000000) {
    publishDXLPowerMsg(1);
  }

  // Peta tombol fisik -> nama yang diterbitkan ke /robotis/open_cr/button.
  //
  // Firmware OpenCR (dxl_node_op3.cpp:551-554) mengemas keempat tombol jadi satu
  // byte: S1 di bit 0, S2 bit 1, S3 bit 2, S4 bit 3. Keempatnya ada di header
  // GPIO OpenCR, bukan tombol bawaan papan:
  //   S1 = pin 56 = BDPIN_GPIO_7    S3 = pin 58 = BDPIN_GPIO_9
  //   S2 = pin 57 = BDPIN_GPIO_8    S4 = pin 59 = BDPIN_GPIO_10
  //
  // 2026-08-22: START dipindah ke D56 (bit 0). Bawaannya "mode" yang di situ,
  // jadi keduanya ditukar -- bukan dipindah sepihak, supaya tidak ada nama yang
  // kehilangan tombol. "user" tetap di tempat aslinya (bit 2 / D58), karena
  // "user_long" adalah pemulihan torque-on + init pose di op3_manager
  // (buttonHandlerCallback), satu-satunya jalan balik kalau torque terlanjur mati.
  //
  // Tidak ada nama "stop": "start" memang toggle -- tekan untuk jalan, tekan lagi
  // untuk berhenti (SoccerDemo::buttonHandlerCallback). Sengaja dibiarkan begitu.
  //
  // Bit 3 (D59) sengaja tidak diterbitkan: firmware memakainya sendiri untuk
  // mematikan power DXL (tahan >100 ms), jadi jangan dipetakan ke nama apa pun.
  uint8_t button_flag = sensors["open-cr"]->sensor_state_->bulk_read_table_["button"];
  result_["button_start"] = button_flag & 0x01;        // bit 0 = PIN_BUTTON_S1 = D56
  result_["button_mode"] = (button_flag & 0x02) >> 1;  // bit 1 = PIN_BUTTON_S2 = D57
  result_["button_user"] = (button_flag & 0x04) >> 2;  // bit 2 = PIN_BUTTON_S3 = D58
  // D60 (mode otomatis): bit dari parameter d60_button_bit, bawaan bit 4.
  result_["button_d60"] = (d60_button_mask_ != 0 && (button_flag & d60_button_mask_)) ? 1.0 : 0.0;

  // Bit yang menyala tapi tidak dipetakan ke nama mana pun dicatat pada
  // TEPI-nya (sekali per tekan/lepas), supaya letak tombol baru di firmware
  // langsung terbaca di log op3_manager tanpa menebak.
  {
    const uint8_t mapped = static_cast<uint8_t>(0x07 | d60_button_mask_);
    const uint8_t unmapped = static_cast<uint8_t>(button_flag & static_cast<uint8_t>(~mapped));
    if (unmapped != unmapped_button_bits_) {
      if (unmapped != 0)
        RCLCPP_WARN(this->get_logger(),
                    "register Button OpenCR = 0x%02X: bit yang TIDAK dipetakan menyala (0x%02X). Kalau ini tombol "
                    "D60, set parameter d60_button_bit ke nomor bit itu (sekarang mask 0x%02X).",
                    button_flag, unmapped, d60_button_mask_);
      unmapped_button_bits_ = unmapped;
    }
  }

  handleButton("mode");
  handleButton("start");
  handleButton("user");
  if (d60_button_mask_ != 0)
    handleButton("d60");

  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  publishIMU();

  previous_result_["gyro_x_prev"] = result_["gyro_x"];
  previous_result_["gyro_y_prev"] = result_["gyro_y"];
  previous_result_["gyro_z_prev"] = result_["gyro_z"];
}

// Heading versi MINI PC: koreksi hard-iron + kompensasi kemiringan.
//
// Rumusnya sama dengan firmware (dxl_hw_op3_mag_update): komponen sudah dalam
// kerangka badan robot, jadi yang tersisa hanya mengurangi offset, memutar balik
// kemiringan memakai roll/pitch dari akselerometer, lalu atan2. Penghalusnya
// juga sama: rata-rata sin/cos, supaya lompatan 359 -> 0 tidak merusak filter.
double OpenCRModule::headingFromRawMag(double mx, double my, double mz)
{
  mx -= mag_hard_iron_offset_[0];
  my -= mag_hard_iron_offset_[1];
  mz -= mag_hard_iron_offset_[2];

  const double norm = std::sqrt(mx * mx + my * my + mz * mz);
  if (!std::isfinite(norm) || norm < 1.0)
    return kNoHeading;   // medan terlalu lemah / sensor diam di nol
  mx /= norm;
  my /= norm;
  mz /= norm;

  // roll & pitch dari akselerometer, rumus yang sama dengan publishIMU().
  const double mui = 0.01;
  const double sign = std::copysign(1.0, result_["acc_z"]);
  const double roll = std::atan2(
    -result_["acc_x"],
    sign * std::sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_y"] * result_["acc_y"]));
  const double pitch = std::atan2(
    -result_["acc_y"],
    std::sqrt(result_["acc_x"] * result_["acc_x"] + result_["acc_z"] * result_["acc_z"]));

  const double sr = std::sin(roll), cr = std::cos(roll);
  const double sp = std::sin(pitch), cp = std::cos(pitch);
  const double xh = mx * cp + my * sr * sp + mz * cr * sp;
  const double yh = my * cr - mz * sr;

  const double head_rad = std::atan2(-yh, xh);
  if (!mag_flt_init_) {
    mag_flt_sin_ = std::sin(head_rad);
    mag_flt_cos_ = std::cos(head_rad);
    mag_flt_init_ = true;
  } else {
    const double alpha = 0.1;
    mag_flt_sin_ += alpha * (std::sin(head_rad) - mag_flt_sin_);
    mag_flt_cos_ += alpha * (std::cos(head_rad) - mag_flt_cos_);
  }
  double deg = std::atan2(mag_flt_sin_, mag_flt_cos_) * RADIAN2DEGREE;
  return std::fmod(deg + 360.0, 360.0);
}

// Interpolasi linier melingkar: heading terukur -> arah nyata (deg, 0..360).
double OpenCRModule::correctHeadingDeg(double heading_deg) const
{
  const size_t n = yaw_calib_true_deg_.size();
  for (size_t i = 0; i < n; ++i) {
    const size_t j = (i + 1) % n;
    const double meas_span = std::fmod(yaw_calib_measured_deg_[j] - yaw_calib_measured_deg_[i] + 720.0, 360.0);
    const double off = std::fmod(heading_deg - yaw_calib_measured_deg_[i] + 720.0, 360.0);
    if (off <= meas_span) {
      const double true_span = std::fmod(yaw_calib_true_deg_[j] - yaw_calib_true_deg_[i] + 720.0, 360.0);
      return std::fmod(yaw_calib_true_deg_[i] + true_span * off / meas_span + 720.0, 360.0);
    }
  }
  return heading_deg;
}

double OpenCRModule::getGyroValue(int raw_value)
{
  return static_cast<double>(raw_value) * GYRO_FACTOR * DEGREE2RADIAN;
}

double OpenCRModule::getAccValue(int raw_value)
{
  return static_cast<double>(raw_value) * ACCEL_FACTOR;
}

void OpenCRModule::publishIMU()
{
  imu_msg_.header.stamp = this->get_clock()->now();
  imu_msg_.header.frame_id = "body_link";

  imu_msg_.angular_velocity.x = result_["gyro_x"];
  imu_msg_.angular_velocity.y = result_["gyro_y"];
  imu_msg_.angular_velocity.z = result_["gyro_z"];

  imu_msg_.linear_acceleration.x = result_["acc_x"] * G_ACC;
  imu_msg_.linear_acceleration.y = result_["acc_y"] * G_ACC;
  imu_msg_.linear_acceleration.z = result_["acc_z"] * G_ACC;

  double mui = 0.01;
  double sign = std::copysign(1.0, result_["acc_z"]);
  double roll = std::atan2(
    -result_["acc_x"],
    sign * std::sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_y"] * result_["acc_y"]));
  // Papan OpenCR di ALPHONSE terpasang diputar 90 derajat terhadap sumbu z, jadi
  // acc_x menunjuk ke kanan dan acc_y ke depan -- bukan seperti OP3 asli. Roll di atas
  // sudah menyesuaikan itu, tapi komponen depan masih perlu dinegasikan supaya tanda
  // pitch mengikuti REP-103: condong ke depan = positif. Diverifikasi di robot dengan
  // memiringkan badan ~30 derajat ke depan; tanpa minus ini hasilnya negatif, dan
  // soccer_demo membaca jatuh telungkup sebagai Fallen_Behind (motion bangkit terbalik).
  double pitch = std::atan2(
    -result_["acc_y"],
    std::sqrt(result_["acc_x"] * result_["acc_x"] + result_["acc_z"] * result_["acc_z"]));
  // Yaw sudah difilter di process() dengan lowPassFilterAngle (varian sirkular).
  // Roll & pitch di atas dihitung dari akselerometer, bukan dari register roll/pitch
  // OpenCR -- jadi yaw ini memang jalur baru, bukan salinan pola lama.
  // Yaw relatif titik nol terakhir (reset_yaw), dinormalisasi ke (-pi, pi].
  // Magnetometer belum siap: tetap 0 seperti perilaku lama.
  double yaw = result_["yaw"];
  if (result_["mag_status"] == 2 || result_["mag_status"] == 3)
    yaw = std::atan2(std::sin(yaw - yaw_zero_offset_), std::cos(yaw - yaw_zero_offset_));

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_->publish(imu_msg_);
}

void OpenCRModule::handleButton(const std::string &button_name)
{
  std::string button_key = "button_" + button_name;
  std::string button_published = "published_" + button_name;

  bool pushed = (result_[button_key] == 1.0);
  if (buttons_[button_key] == pushed) {
    if (pushed && !buttons_[button_published]) {
      auto press_time_it = buttons_press_time_.find(button_name);
      if (press_time_it == buttons_press_time_.end()) {
        // Waktu tekannya belum pernah tercatat. Pakai SEKARANG, bukan epoch:
        // dengan epoch, selisihnya jadi puluhan tahun sehingga "> 2 detik"
        // langsung benar dan tombol menerbitkan "_long" palsu pada siklus
        // pertama, tanpa ada yang menekan apa pun.
        buttons_press_time_.emplace(button_name, this->get_clock()->now());
        return;
      }
      rclcpp::Duration button_duration = this->get_clock()->now() - buttons_press_time_[button_name];
      if (button_duration.seconds() > 2.0) {
        publishButtonMsg(button_name + "_long");
        buttons_[button_published] = true;
      }
    }
  } else {
    buttons_[button_key] = pushed;

    if (pushed) {
      buttons_press_time_[button_name] = this->get_clock()->now();
      buttons_[button_published] = false;
    } else {
      auto press_time_it = buttons_press_time_.find(button_name);
      if (press_time_it == buttons_press_time_.end()) {
        buttons_press_time_.emplace(
          button_name,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));
        return;
      }
      rclcpp::Duration button_duration = this->get_clock()->now() - buttons_press_time_[button_name];
      if (button_duration.seconds() < 2.0) {
        // DEBOUNCE. Tombolnya mekanis dan dibaca tiap siklus kendali (8 ms).
        // Kontak yang memantul terbaca sebagai beberapa tekan-lepas berturut-
        // turut dalam beberapa milidetik, dan tiap tepi-lepas menerbitkan satu
        // "start" lagi. "start" itu TOGGLE di SoccerDemo, jadi dua pesan dari
        // SATU tekanan = nyala lalu mati lagi -- persis gejala "dipencet untuk
        // mematikan tapi robot tetap jalan".
        //
        // Tekanan manusia yang sungguhan tidak pernah lebih rapat dari ~400 ms,
        // jadi jendela ini tidak pernah membuang tekanan yang disengaja.
        if (publishAllowed(button_name)) {
          publishButtonMsg(button_name);
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "tombol '%s' dibuang: pantulan kontak (%.0f ms sejak yang terakhir)",
                      button_name.c_str(),
                      (this->get_clock()->now() - buttons_publish_time_[button_name]).seconds() * 1000.0);
        }
      }
    }
  }
}

// true kalau nama tombol ini boleh diterbitkan sekarang; sekaligus mencatat
// waktunya. Jendelanya dihitung dari penerbitan TERAKHIR, bukan dari tekanan,
// supaya rentetan pantulan tidak bisa lolos satu per satu.
bool OpenCRModule::publishAllowed(const std::string &button_name)
{
  const rclcpp::Time now = this->get_clock()->now();

  auto it = buttons_publish_time_.find(button_name);
  if (it != buttons_publish_time_.end()) {
    if ((now - it->second).seconds() < button_debounce_sec_) {
      return false;
    }
    it->second = now;
    return true;
  }

  buttons_publish_time_.emplace(button_name, now);
  return true;
}

void OpenCRModule::publishButtonMsg(const std::string &button_name)
{
  std_msgs::msg::String button_msg;
  button_msg.data = button_name;

  button_pub_->publish(button_msg);
  publishStatusMsg(robotis_controller_msgs::msg::StatusMsg::STATUS_INFO, "Button : " + button_name);
}

void OpenCRModule::handleVoltage(double present_volt)
{
  double voltage_ratio = 0.4;
  previous_volt_ =
    (previous_volt_ != 0.0) ? previous_volt_ * (1.0 - voltage_ratio) + present_volt * voltage_ratio : present_volt;

  if (std::fabs(present_volt_ - previous_volt_) >= 0.1) {
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Duration dur = now - last_msg_time_;
    if (dur.seconds() < 1.0) {
      return;
    }

    last_msg_time_ = now;
    present_volt_ = previous_volt_;

    std::stringstream log_stream;
    log_stream << "Present Volt : " << present_volt_ << "V";
    publishStatusMsg(
      (present_volt_ < 11.0 ?
        robotis_controller_msgs::msg::StatusMsg::STATUS_WARN :
        robotis_controller_msgs::msg::StatusMsg::STATUS_INFO),
      log_stream.str());

    RCLCPP_INFO_EXPRESSION(
      this->get_logger(), DEBUG_PRINT, "Present Volt : %fV, Read Volt : %fV",
      previous_volt_, result_["present_voltage"]);
  }
}

void OpenCRModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::msg::StatusMsg status_msg;
  status_msg.header.stamp = this->get_clock()->now();
  status_msg.type = type;
  status_msg.module_name = "SENSOR";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}

void OpenCRModule::publishDXLPowerMsg(unsigned int value)
{
  robotis_controller_msgs::msg::SyncWriteItem sync_write_msg;
  sync_write_msg.item_name = "dynamixel_power";
  sync_write_msg.joint_name.push_back("open-cr");
  sync_write_msg.value.push_back(value);

  dxl_power_msg_pub_->publish(sync_write_msg);
}

double OpenCRModule::lowPassFilter(double alpha, double x_new, double &x_old)
{
  double filtered_value = alpha * x_new + (1.0 - alpha) * x_old;
  x_old = filtered_value;
  return filtered_value;
}

// Low-pass untuk sudut yang wrap di +-pi. Rata-rata biasa tidak boleh dipakai di sini:
// begitu yaw melompat dari +179 ke -179, alpha*x_new + (1-alpha)*x_old akan menyeret
// hasilnya melewati seluruh lingkaran (mendekati 0) padahal robot cuma bergerak 2 derajat.
// Jadi yang difilter adalah SELISIH terpendeknya, lalu hasilnya di-wrap ulang.
double OpenCRModule::lowPassFilterAngle(double alpha, double x_new, double &x_old)
{
  // selisih terpendek x_new - x_old, dinormalkan ke [-pi, pi)
  double diff = std::fmod(x_new - x_old + M_PI, 2.0 * M_PI);
  if (diff < 0.0) {
    diff += 2.0 * M_PI;
  }
  diff -= M_PI;

  double filtered_value = std::fmod(x_old + alpha * diff + M_PI, 2.0 * M_PI);
  if (filtered_value < 0.0) {
    filtered_value += 2.0 * M_PI;
  }
  filtered_value -= M_PI;

  x_old = filtered_value;
  return filtered_value;
}

}  // namespace robotis_op

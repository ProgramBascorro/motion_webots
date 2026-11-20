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

#include <algorithm>
#include <cmath>
#include <sstream>

#include "open_cr_module/open_cr_module.h"

namespace robotis_op
{

// Ke (-180, 180].
static double wrap180Deg(double a)
{
  double r = std::fmod(a + 180.0, 360.0);
  if (r < 0.0)
    r += 360.0;
  return r - 180.0;
}

OpenCRModule::OpenCRModule()
: Node("open_cr_module"),
  control_cycle_msec_(8),
  DEBUG_PRINT(false),
  previous_volt_(0.0),
  present_volt_(0.0),
  mag_present_(false),
  mag_status_(MAG_STATUS_NO_SENSOR),
  mag_yaw_rad_(0.0),
  mag_heading_deg_(0.0),
  yaw_gyro_deg_(0.0),
  last_raw_yaw_gyro_deg_(0.0),
  yaw_gyro_offset_deg_(0.0),
  yaw_calib_enabled_(false),
  yaw_zero_request_(false),
  yaw_zero_offset_(0.0),
  yaw_zero_initialized_(false),
  yaw_filter_valid_(false),
  yaw_filtered_(0.0),
  gz_head_(0),
  gz_count_(0),
  gyro_bias_dps_(0.0),
  gyro_still_sum_deg_(0.0),
  gyro_still_time_s_(0.0),
  gyro_bias_valid_(false),
  gyro_drift_corr_deg_(0.0),
  prev_raw_gyro_deg_(0.0),
  prev_raw_gyro_valid_(false),
  gyro_still_(false),
  yaw_gyro_corr_deg_(0.0),
  still_sec_(0.0),
  auto_gyro_cal_count_(0),
  auto_gyro_cal_wait_(0.0),
  gyro_bias_param_dps_(0.0),
  fused_valid_(false),
  fused_yaw_deg_(0.0),
  prev_gyro_corr_deg_(0.0),
  fuse_far_sec_(0.0)
{
  // Tabel kalibrasi yaw 5 titik (op3_manager/config/yaw_calibration.yaml).
  loadYawCalibParams();

  // Bias gyro hasil kalibrasi drift (op3_manager/config/imu_drift.yaml). Dipakai
  // sebagai nilai awal supaya yaw tidak hanyut selama menit-menit pertama,
  // sebelum modul sempat mempelajari biasnya sendiri dari waktu diam.
  try
  {
    gyro_bias_param_dps_ = this->declare_parameter<double>("gyro_bias_dps", 0.0);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter gyro_bias_dps tidak terbaca (%s) -- dianggap 0", e.what());
    gyro_bias_param_dps_ = 0.0;
  }
  if (std::isfinite(gyro_bias_param_dps_) && gyro_bias_param_dps_ != 0.0)
  {
    gyro_bias_dps_ = gyro_bias_param_dps_;
    gyro_bias_valid_ = true;
    gyro_still_sum_deg_ = gyro_bias_param_dps_ * GYRO_BIAS_MIN_SEC;
    gyro_still_time_s_ = GYRO_BIAS_MIN_SEC;
    RCLCPP_WARN(this->get_logger(), "Kalibrasi drift gyro dipakai: bias awal %+.4f deg/s", gyro_bias_param_dps_);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Kalibrasi drift gyro tidak ada (gyro_bias_dps 0) -- bias dipelajari sendiri");
  }

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

  // Dibaca modul lain lewat sensor result (mis. balance/localization) dan ikut
  // terbawa ke topik /robotis/present_joint_states-nya sensor.
  result_["roll"] = 0.0;
  result_["pitch"] = 0.0;
  result_["yaw"] = 0.0;          // yaw kompas terfilter (rad), BELUM dikurangi titik nol
  result_["yaw_gyro"] = 0.0;     // yaw lama hasil fusi gyro, sebagai pembanding
  result_["mag_x"] = 0.0;
  result_["mag_y"] = 0.0;
  result_["mag_z"] = 0.0;
  result_["mag_heading"] = 0.0;  // arah kompas absolut, derajat 0..360
  result_["mag_status"] = 0.0;

  buttons_["button_mode"] = false;
  buttons_["button_start"] = false;
  buttons_["button_user"] = false;
  buttons_["published_mode"] = false;
  buttons_["published_start"] = false;
  buttons_["published_user"] = false;

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

void OpenCRModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  (void) robot;
  control_cycle_msec_ = control_cycle_msec;

  // Publisher dibuat di sini, BUKAN di queueThread: process() (thread kontrol)
  // sudah bisa memanggil publishStatusMsg() di siklus pertama -- handleVoltage()
  // selalu menerbitkan pada pembacaan pertama -- sebelum queueThread sempat
  // membuatnya, dan publisher null = SIGSEGV op3_manager.
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/robotis/open_cr/imu", 1);
  button_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/open_cr/button", 1);
  dxl_power_msg_pub_ =
    this->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 1);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/robotis/open_cr/mag", 1);
  mag_status_pub_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/robotis/open_cr/mag_status", 1);
  // mag_heading MENTAH (derajat, tanpa tabel) untuk alat kalibrasi
  // scripts/calib_imu_yaw.py; hanya terbit saat mag_status 2/3.
  mag_heading_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robotis/open_cr/mag_heading", 10);

  // Perintah kalibrasi dari luar (Bascorro Studio, atau `ros2 topic pub` manual).
  // Dibuat topik String, bukan service, supaya sejalan dengan gaya perintah lain
  // di stack ini (/robotis/enable_ctrl_module, /robotis/walking/command).
  imu_command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robotis/open_cr/imu_command", 1,
    std::bind(&OpenCRModule::imuCommandCallback, this, std::placeholders::_1));
  // Isi pesan bebas (alasan reset, untuk log).
  reset_yaw_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robotis/open_cr/reset_yaw", 10,
    std::bind(&OpenCRModule::resetYawCallback, this, std::placeholders::_1));

  queue_thread_ = std::thread(&OpenCRModule::queueThread, this);
}

void OpenCRModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

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

  // Firmware fw 0x03 menaruh roll/pitch/yaw sebagai int16 satuan 0.1 derajat.
  // Cast ke int16_t dulu: bulk_read_table_ menyimpan uint32_t, jadi nilai negatif
  // datang sebagai bilangan besar (0xFFFF...) kalau langsung dipakai.
  int16_t raw_roll = sensors["open-cr"]->sensor_state_->bulk_read_table_["roll"];
  int16_t raw_pitch = sensors["open-cr"]->sensor_state_->bulk_read_table_["pitch"];
  int16_t raw_yaw = sensors["open-cr"]->sensor_state_->bulk_read_table_["yaw"];

  int16_t mag_x = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_x"];
  int16_t mag_y = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_y"];
  int16_t mag_z = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_z"];
  uint16_t mag_heading = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_heading"];
  int16_t raw_yaw_gyro = sensors["open-cr"]->sensor_state_->bulk_read_table_["yaw_gyro"];
  uint8_t mag_status = sensors["open-cr"]->sensor_state_->bulk_read_table_["mag_status"];

  //== PEMETAAN SUMBU IMU -> KERANGKA BADAN =================================
  //
  // OpenCR di robot ini terpasang TERPUTAR 90 derajat terhadap badan. Diukur
  // pada robot (2026-08-19) dengan memiringkannya dan membaca accel + integral
  // gyro saat onset:
  //
  //   condong DEPAN -> sumbu sensor y turun kuat  => sensor +y = DEPAN badan
  //   miring  KIRI  -> sumbu sensor x naik kuat   => sensor +x = KANAN badan
  //
  // ("sumbu sensor" di sini = nilai SESUDAH pembalikan bawaan modul ini, yaitu
  //  -acc_x, -acc_y, +acc_z.)
  //
  // Dalam kerangka REP-103 (x depan, y kiri, z atas) berarti:
  //
  //   X_badan (depan) =  sensor_y
  //   Y_badan (kiri)  = -sensor_x
  //   Z_badan (atas)  =  sensor_z
  //
  // Ini BUKAN sekadar soal angka yang tampil terbalik di layar. sensors["gyro_x"]
  // dan sensors["gyro_y"] dibaca langsung oleh balance walking module
  // (op3_walking_module.cpp:546), jadi selama pemetaannya salah, koreksi
  // depan-belakang dikirim ke sendi ROLL dan koreksi kiri-kanan ke sendi PITCH --
  // robot dikoreksi ke sumbu yang sama sekali keliru.
  //
  // Dibuktikan: saat robot dimiringkan ke kiri, integral gyro_y = -18.9 derajat
  // (dominan) sementara gyro_x hanya +3.4. Guling kiri-kanan memang terbaca di
  // gyro_y, bukan gyro_x.
  //
  // Setelah pemetaan ini benar, rumus roll/pitch di publishIMU() TIDAK perlu
  // diubah sama sekali -- keduanya langsung menghasilkan konvensi REP-103:
  //   roll  negatif = miring ke KIRI
  //   pitch negatif = condong ke BELAKANG
  //
  // Kalau board suatu saat dipasang ulang menghadap standar, cukup kembalikan
  // enam baris di bawah ini ke bentuk semula (x<-x, y<-y) tanpa menyentuh
  // yang lain.
  //
  result_["gyro_x"] = lowPassFilter(0.4, -getGyroValue(gyro_y), previous_result_["gyro_x"]);
  result_["gyro_y"] = lowPassFilter(0.4, getGyroValue(gyro_x), previous_result_["gyro_y"]);
  result_["gyro_z"] = lowPassFilter(0.4, getGyroValue(gyro_z), previous_result_["gyro_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Gyro ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", gyro_x, gyro_y, gyro_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f",
                         result_["gyro_x"], result_["gyro_y"], result_["gyro_z"]);

  // Pemetaan sumbu yang sama seperti pada gyro di atas -- lihat penjelasannya.
  result_["acc_x"] = lowPassFilter(0.4, -getAccValue(acc_y), previous_result_["acc_x"]);
  result_["acc_y"] = lowPassFilter(0.4, getAccValue(acc_x), previous_result_["acc_y"]);
  result_["acc_z"] = lowPassFilter(0.4, getAccValue(acc_z), previous_result_["acc_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Acc ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", acc_x, acc_y, acc_z);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f",
                         result_["acc_x"], result_["acc_y"], result_["acc_z"]);

  const auto& update_stamp = sensors["open-cr"]->sensor_state_->update_time_stamp_;
  rclcpp::Time update_time(
    update_stamp.sec_,
    update_stamp.nsec_,
    this->get_clock()->get_clock_type());
  rclcpp::Duration update_duration = this->get_clock()->now() - update_time;
  if (update_duration.nanoseconds() > 100000000) {
    publishDXLPowerMsg(1);
  }

  uint8_t button_flag = sensors["open-cr"]->sensor_state_->bulk_read_table_["button"];

  // CHRONUS: bit "mode" dan "start" SENGAJA DITUKAR terhadap pemetaan bawaan.
  //
  // Register Button (OPEN-CR alamat 30) memetakan pin secara tetap di firmware
  // (dxl_node_op3.cpp:551-554): D56/S1 -> bit0, D57/S2 -> bit1, D58/S3 -> bit2.
  // Bawaan ROBOTIS menganggap bit0 = "mode" (pemilih demo) dan bit1 = "start"
  // (penjalan demo), karena di OP3 asli memang begitu tombolnya dipasang.
  //
  // Di robot ini tombol penjalan demo terpasang di D56, bukan D57. Tukar di sini
  // -- bukan di firmware -- supaya arti register tetap sesuai dokumentasinya
  // (bit0 SELALU berarti S1) dan perubahan ini tidak butuh flashing OpenCR.
  // Kalau nanti tombolnya dipindah ke D57 sesuai bawaan, cukup kembalikan dua
  // baris di bawah ini ke urutan semula.
  // "start" diterima dari D56 ATAU D59 -- tombol penjalan demo di robot ini boleh
  // dicolok ke salah satu dari kedua pin itu tanpa ubah kode lagi.
  //
  // D58 DIKELUARKAN dari gabungan itu (dulu ikut memicu "start") dan dikembalikan
  // ke jalur "user" sendiri, karena D58 sekarang punya tugas: menjalankan demo
  // "DEMO NORMAL + INTERCEPT" di demo_node. Yang dipakai adalah tekan SINGKAT
  // ("user"); "user_long" (tahan 2 dtk = torque on + init pose) tetap milik
  // op3_manager dan tidak berubah.
  bool start_pushed = (button_flag & (0x01 | 0x08)) != 0;          // D56, D59
  result_["button_start"] = start_pushed ? 1.0 : 0.0;
  result_["button_mode"] = (button_flag & 0x02) >> 1;              // D57 / S2 -> pilih demo
  result_["button_user"] = (button_flag & 0x04) >> 2;              // D58 / S3 -> demo intercept

  handleButton("mode");
  handleButton("start");
  handleButton("user");

  result_["present_voltage"] = present_volt * 0.1;
  handleVoltage(result_["present_voltage"]);

  //-- Magnetometer / yaw -------------------------------------------------
  //
  // Yaw hanya sah saat mag_status 2 (OK) atau 3 (kalibrasi hard-iron). Selama
  // SETTLING firmware masih mengunci referensi dan mengirim yaw 0; di luar 2/3
  // yaw = 0 seperti perilaku modul aslinya.
  mag_status_ = mag_status;
  mag_present_ = (mag_status == MAG_STATUS_OK || mag_status == MAG_STATUS_CALIBRATING);

  mag_heading_deg_ = mag_heading * RPY_FACTOR;
  yaw_gyro_deg_ = raw_yaw_gyro * RPY_FACTOR;
  last_raw_yaw_gyro_deg_ = yaw_gyro_deg_;

  // Gyro terkoreksi hanyutan (selalu dihitung; gyro sah apa pun status kompas).
  updateGyroDrift();
  const double d_gyro = wrap180Deg(yaw_gyro_corr_deg_ - prev_gyro_corr_deg_);
  prev_gyro_corr_deg_ = yaw_gyro_corr_deg_;

  if (mag_present_)
  {
    double yaw_rad = raw_yaw * RPY_FACTOR * DEGREE2RADIAN;
    if (yaw_calib_enabled_)
    {
      // Kompas terkalibrasi tabel = acuan mutlak; gyro terkoreksi hanyutan
      // meneruskan di antaranya. Kompas hanya menarik saat DIAM.
      const double compass_deg = correctHeadingDeg(mag_heading_deg_);
      if (fused_valid_ == false)
      {
        fused_yaw_deg_ = compass_deg;
        fused_valid_ = true;
        fuse_far_sec_ = 0.0;
      }
      else
      {
        const double dt = control_cycle_msec_ * 0.001;
        fused_yaw_deg_ += d_gyro;
        if (gyro_still_)
        {
          const double innov = wrap180Deg(compass_deg - fused_yaw_deg_);
          fuse_far_sec_ = (std::fabs(innov) > FUSE_SNAP_DEG) ? fuse_far_sec_ + dt : 0.0;
          if (fuse_far_sec_ >= FUSE_SNAP_SEC)
          {
            fused_yaw_deg_ = compass_deg;      // gyro jelas melenceng (mis. robot jatuh)
            fuse_far_sec_ = 0.0;
          }
          else
          {
            fused_yaw_deg_ += std::min(1.0, dt / FUSE_TAU_SEC) * innov;
          }
        }
        fused_yaw_deg_ = std::fmod(fused_yaw_deg_ + 720.0, 360.0);
      }
      yaw_rad = fused_yaw_deg_ * DEGREE2RADIAN;
    }

    if (yaw_filter_valid_ == false)
    {
      // Sampel sah pertama: isi keadaan filter dengan nilai ini supaya filter
      // tidak merayap dari 0, dan (sekali sesudah manager start) jadikan titik 0.
      yaw_filtered_ = yaw_rad;
      yaw_filter_valid_ = true;
      if (yaw_zero_initialized_ == false)
      {
        yaw_zero_offset_ = yaw_rad;
        yaw_zero_initialized_ = true;
      }
    }
    result_["yaw"] = lowPassFilterAngle(0.4, yaw_rad, yaw_filtered_);

    if (yaw_zero_request_.exchange(false) == true)
    {
      yaw_zero_offset_ = result_["yaw"];
      yaw_zero_initialized_ = true;
      RCLCPP_WARN(this->get_logger(), "YAW DIRESET: arah sekarang = 0 (yaw %s %.2f deg)",
                  yaw_calib_enabled_ ? "terkalibrasi" : "firmware",
                  result_["yaw"] * RADIAN2DEGREE);
    }

    const double rel = result_["yaw"] - yaw_zero_offset_;
    mag_yaw_rad_ = std::atan2(std::sin(rel), std::cos(rel));

    std_msgs::msg::Float32 heading_msg;
    heading_msg.data = static_cast<float>(mag_heading_deg_);
    if (mag_heading_pub_)
      mag_heading_pub_->publish(heading_msg);
  }
  else
  {
    result_["yaw"] = 0.0;
    fused_valid_ = false;
    yaw_filter_valid_ = false;
    yaw_filtered_ = 0.0;
    mag_yaw_rad_ = 0.0;
  }

  // Register 44/46 dinamai "Roll"/"Pitch" oleh firmware, tapi nama itu mengacu ke
  // sumbu SENSOR, bukan sumbu badan -- dan board terpasang terputar 90 derajat.
  // Diukur pada robot: saat register 44 = +5.5 dan register 46 = +1.4, sikap badan
  // yang sebenarnya adalah pitch +5.3 (condong depan) dan roll -1.5 (miring kiri).
  // Jadi keduanya tertukar, dan register 46 juga berlawanan tanda:
  //
  //   roll  badan = -register46 ("Pitch")
  //   pitch badan = +register44 ("Roll")
  //
  // Nilainya diambil dari fusi firmware (lebih tenang daripada accel murni),
  // sedangkan quaternion di imu_msg_ tetap dari accel; keduanya kini sepakat
  // tanda dan sumbunya.
  result_["roll"] = -raw_pitch * RPY_FACTOR * DEGREE2RADIAN;
  result_["pitch"] = raw_roll * RPY_FACTOR * DEGREE2RADIAN;
  result_["yaw_gyro"] = yaw_gyro_deg_ * DEGREE2RADIAN;
  result_["mag_x"] = mag_x;
  result_["mag_y"] = mag_y;
  result_["mag_z"] = mag_z;
  result_["mag_heading"] = mag_heading_deg_;
  result_["mag_status"] = mag_status;

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Mag ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d, status %d",
                         mag_x, mag_y, mag_z, mag_status);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Heading : %.1f deg, Yaw mag : %.1f deg, Yaw gyro : %.1f deg",
                         mag_heading_deg_, raw_yaw * RPY_FACTOR, yaw_gyro_deg_);

  publishIMU();
  publishMag();
  publishMagStatus();

  previous_result_["gyro_x_prev"] = result_["gyro_x"];
  previous_result_["gyro_y_prev"] = result_["gyro_y"];
  previous_result_["gyro_z_prev"] = result_["gyro_z"];
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
    result_["acc_y"],
    sign * std::sqrt(result_["acc_z"] * result_["acc_z"] + mui * result_["acc_x"] * result_["acc_x"]));
  double pitch = std::atan2(
    -result_["acc_x"],
    std::sqrt(result_["acc_y"] * result_["acc_y"] + result_["acc_z"] * result_["acc_z"]));
  // Roll & pitch tetap dihitung dari accelerometer seperti aslinya -- jangan
  // diganti ke register roll/pitch firmware, karena balance control sudah
  // ditala terhadap angka yang ini.
  //
  // Yaw kompas (mag_status 2/3) relatif titik nol: atan2(sin(yaw - offset),
  // cos(yaw - offset)), dihitung di process(). Di luar 2/3 tetap 0.
  double yaw = mag_present_ ? mag_yaw_rad_ : 0.0;

  Eigen::Quaterniond orientation = robotis_framework::convertRPYToQuaternion(roll, pitch, yaw);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  imu_pub_->publish(imu_msg_);
}

void OpenCRModule::publishMag()
{
  mag_msg_.header.stamp = this->get_clock()->now();
  mag_msg_.header.frame_id = "body_link";

  // Firmware sudah memetakan sumbu sensor ke kerangka badan robot dan mengurangi
  // offset hard-iron (MAG_BODY_* di dxl_hw_op3.cpp), jadi di sini tinggal ubah
  // satuan cacahan mentah menjadi Tesla.
  mag_msg_.magnetic_field.x = result_["mag_x"] * MAG_FACTOR;
  mag_msg_.magnetic_field.y = result_["mag_y"] * MAG_FACTOR;
  mag_msg_.magnetic_field.z = result_["mag_z"] * MAG_FACTOR;

  // Kovarians tidak diketahui. Konvensi sensor_msgs: elemen [0] = -1.
  mag_msg_.magnetic_field_covariance[0] = -1.0;

  mag_pub_->publish(mag_msg_);
}

void OpenCRModule::publishMagStatus()
{
  // Ringkasan angka untuk tuning, dalam satuan yang enak dibaca manusia
  // (derajat, bukan radian). Urutan tetap; label ada di layout supaya sisi
  // penerima tidak perlu menghafal indeks.
  std_msgs::msg::Float64MultiArray status_msg;

  status_msg.layout.dim.resize(1);
  // Urutan indeks 0..7 dipertahankan untuk pembaca lama (Studio, mag_calib.py).
  // [8] = kalibrasi yaw 5 titik aktif; [9..11] = kompensasi hanyutan gyro.
  status_msg.layout.dim[0].label =
    "status,heading_deg,yaw_mag_deg,yaw_gyro_deg,mag_x,mag_y,mag_z,yaw_gyro_rel_deg,yaw_cal_active,"
    "yaw_gyro_corr_deg,gyro_bias_dps,gyro_still";
  status_msg.layout.dim[0].size = 12;
  status_msg.layout.dim[0].stride = 12;

  status_msg.data.push_back(static_cast<double>(mag_status_));
  status_msg.data.push_back(mag_heading_deg_);
  status_msg.data.push_back(mag_yaw_rad_ * RADIAN2DEGREE);
  status_msg.data.push_back(yaw_gyro_deg_);
  status_msg.data.push_back(result_["mag_x"]);
  status_msg.data.push_back(result_["mag_y"]);
  status_msg.data.push_back(result_["mag_z"]);
  // Yaw gyro relatif terhadap reference "zero_yaw", dirapikan ke -180..180.
  status_msg.data.push_back(
    std::fmod(yaw_gyro_deg_ + yaw_gyro_offset_deg_.load() + 540.0, 360.0) - 180.0);
  status_msg.data.push_back(yaw_calib_enabled_ ? 1.0 : 0.0);
  status_msg.data.push_back(yaw_gyro_corr_deg_);
  status_msg.data.push_back(gyro_bias_valid_ ? gyro_bias_dps_ : 0.0);
  status_msg.data.push_back(gyro_still_ ? 1.0 : 0.0);

  mag_status_pub_->publish(status_msg);
}

void OpenCRModule::imuCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string &command = msg->data;

  if (command == "calibrate_mag") {
    // Robot harus diputar pelan satu putaran penuh selama 20 detik ke depan;
    // firmware mencatat min/maks tiap sumbu lalu menyimpan titik tengahnya ke
    // EEPROM sebagai offset hard-iron.
    writeImuControl(IMU_CTRL_CALI_MAG);
    publishStatusMsg(
      robotis_controller_msgs::msg::StatusMsg::STATUS_INFO,
      "Magnetometer calibration started : rotate the robot slowly for 20 seconds");
  } else if (command == "zero_yaw") {
    // Arah sekarang = 0: sama dengan /robotis/open_cr/reset_yaw (offset diambil
    // di process()). Reference gyro untuk mag_status[7] tetap diambil di sini.
    const double gyro_reference = last_raw_yaw_gyro_deg_.load();
    yaw_gyro_offset_deg_ = -gyro_reference;
    yaw_zero_request_ = true;
    publishStatusMsg(
      robotis_controller_msgs::msg::StatusMsg::STATUS_INFO,
      "Yaw di-reset: arah sekarang = 0 derajat");
    RCLCPP_INFO(this->get_logger(), "zero_yaw: reset yaw diminta, reference gyro = %+.2f deg",
                gyro_reference);
  } else if (command == "reload_yaw_calibration" || command == "clear_yaw_calibration" ||
             command == "reload_mag_calibration" || command == "clear_mag_calibration") {
    // Perintah jalur kalibrasi lama (dicabut 2026-09-18). Tabel sekarang dari
    // op3_manager/config/yaw_calibration.yaml, berlaku sesudah manager start ulang.
    publishStatusMsg(
      robotis_controller_msgs::msg::StatusMsg::STATUS_WARN,
      "Perintah " + command + " tidak dipakai lagi: kalibrasi yaw lewat "
      "scripts/calib_imu_yaw.py, berlaku sesudah op3_manager start ulang");
    RCLCPP_WARN(this->get_logger(), "imu_command %s diabaikan (jalur kalibrasi lama dicabut)",
                command.c_str());
  } else if (command == "calibrate_gyro") {
    writeImuControl(IMU_CTRL_CALI_GYRO);
    publishStatusMsg(
      robotis_controller_msgs::msg::StatusMsg::STATUS_INFO,
      "Gyro calibration started : keep the robot still");
  } else {
    publishStatusMsg(
      robotis_controller_msgs::msg::StatusMsg::STATUS_WARN,
      "Unknown IMU command : " + command);
  }
}

// Kompensasi hanyutan gyro -- kode lama (2026-09-11/13) apa adanya, tanpa
// penyimpanan keadaan antar-restart. Lihat penjelasan di header.
void OpenCRModule::updateGyroDrift()
{
  const double dt = control_cycle_msec_ * 0.001;
  const double gz_dps = result_["gyro_z"] * RADIAN2DEGREE;
  gz_window_[gz_head_] = gz_dps;
  gz_head_ = (gz_head_ + 1) % GZ_WINDOW;
  if (gz_count_ < GZ_WINDOW)
    ++gz_count_;

  // Diam = RENTANG gyro_z dalam jendela 1 detik kecil (lihat STILL_GZ_RANGE_DPS).
  // Bukan reratanya -- itulah biasnya -- dan bukan kompas, yang bisa terganggu
  // walau robot diam. Jendela penuh dulu sebelum menilai. Rerata tetap dihitung:
  // dipakai tebakan bias pertama dan kalibrasi gyro otomatis.
  bool still = false;
  double mean = 0.0;
  if (gz_count_ == GZ_WINDOW)
  {
    double gz_min = gz_window_[0], gz_max = gz_window_[0];
    for (double v : gz_window_)
    {
      mean += v;
      gz_min = std::min(gz_min, v);
      gz_max = std::max(gz_max, v);
    }
    mean /= static_cast<double>(GZ_WINDOW);
    still = (gz_max - gz_min) < STILL_GZ_RANGE_DPS;
  }

  const double d_raw = (prev_raw_gyro_valid_ == true)
                       ? wrap180Deg(yaw_gyro_deg_ - prev_raw_gyro_deg_) : 0.0;

  if (still)
  {
    if (gyro_bias_valid_ == false)
    {
      // Tebakan pertama dari gyro_z, supaya bias langsung terpakai.
      gyro_bias_dps_ = mean;
      gyro_bias_valid_ = true;
    }
    if (prev_raw_gyro_valid_ == true)
    {
      // Bias = total perubahan YAW MENTAH / total waktu diam -- bukan dari
      // gyro_z, karena yang harus dibatalkan saat bergerak adalah yaw hasil
      // integrasi firmware, dan terukur keduanya beda ~11% (gyro_z 0,173 vs
      // yaw mentah 0,155 deg/s pada boot yang sama). Rasio dua jumlah yang
      // dilupakan sama rata = rerata tertimbang waktu; kuantisasi 0,1 deg
      // hanya masuk sekali terhadap seluruh waktu diam, bukan per sampel.
      const double keep = 1.0 - dt / GYRO_BIAS_TAU_SEC;
      gyro_still_sum_deg_ = gyro_still_sum_deg_ * keep + d_raw;
      gyro_still_time_s_ = gyro_still_time_s_ * keep + dt;
      if (gyro_still_time_s_ >= GYRO_BIAS_MIN_SEC)
        gyro_bias_dps_ = gyro_still_sum_deg_ / gyro_still_time_s_;
    }
  }

  if (prev_raw_gyro_valid_ == true)
  {
    if (still)
      gyro_drift_corr_deg_ -= d_raw;                     // diam: tahan persis
    else if (gyro_bias_valid_ == true)
      gyro_drift_corr_deg_ -= gyro_bias_dps_ * dt;       // bergerak: buang bias
  }
  prev_raw_gyro_deg_ = yaw_gyro_deg_;
  prev_raw_gyro_valid_ = true;
  gyro_drift_corr_deg_ = std::fmod(gyro_drift_corr_deg_, 360.0);
  gyro_still_ = still;
  yaw_gyro_corr_deg_ = wrap180Deg(yaw_gyro_deg_ + gyro_drift_corr_deg_);

  //-- Kalibrasi gyro otomatis -------------------------------------------
  // Bias sebesar AUTO_GYRO_CAL_DPS tidak wajar dan tidak nyaman dikompensasi
  // perangkat lunak: firmware mengkalibrasi gyro-nya sendiri saat OpenCR
  // menyala, dan kalau robot saat itu sedang dipegang, hasilnya salah seumur
  // nyala (terukur +2,47 deg/s, 2026-09-19). Begitu robot terbukti diam,
  // minta firmware mengkalibrasi ulang -- inilah "kalibrasi drift" yang
  // sesungguhnya; sisanya baru dikompensasi perangkat lunak.
  still_sec_ = still ? (still_sec_ + dt) : 0.0;
  if (auto_gyro_cal_wait_ > 0.0)
    auto_gyro_cal_wait_ -= dt;
  if (still == true && still_sec_ >= AUTO_GYRO_CAL_STILL_SEC &&
      auto_gyro_cal_wait_ <= 0.0 && auto_gyro_cal_count_ < AUTO_GYRO_CAL_MAX &&
      std::fabs(mean) > AUTO_GYRO_CAL_DPS)
  {
    ++auto_gyro_cal_count_;
    auto_gyro_cal_wait_ = AUTO_GYRO_CAL_RETRY_SEC;
    writeImuControl(IMU_CTRL_CALI_GYRO);
    RCLCPP_WARN(this->get_logger(),
                "Bias gyro %+.2f deg/s saat robot diam -> minta firmware kalibrasi gyro (percobaan %d/%d)",
                mean, auto_gyro_cal_count_, AUTO_GYRO_CAL_MAX);
  }
}

void OpenCRModule::resetYawCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Hanya menandai; offset diambil di process() pada sampel sah berikutnya.
  yaw_zero_request_ = true;
  RCLCPP_INFO(this->get_logger(), "reset_yaw diminta (%s)", msg->data.c_str());
}

void OpenCRModule::loadYawCalibParams()
{
  yaw_calib_enabled_ = false;
  bool enable = false;
  std::vector<double> measured, truth;
  try
  {
    enable = this->declare_parameter<bool>("yaw_calib_enable", false);
    measured = this->declare_parameter<std::vector<double>>(
      "yaw_calib_measured_heading_deg", std::vector<double>());
    truth = this->declare_parameter<std::vector<double>>(
      "yaw_calib_true_deg", std::vector<double>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameter kalibrasi yaw tidak bisa dibaca (%s) -- kalibrasi MATI, yaw firmware mentah",
                 e.what());
    return;
  }

  if (enable == false)
  {
    RCLCPP_INFO(this->get_logger(), "Kalibrasi yaw mati (yaw_calib_enable: false) -- yaw firmware mentah");
    return;
  }

  const size_t n = measured.size();
  std::string err;
  if (n < 3 || truth.size() != n)
  {
    err = "butuh >= 3 titik dan ukuran measured == true (measured " + std::to_string(n) +
          ", true " + std::to_string(truth.size()) + ")";
  }
  else
  {
    double meas_total = 0.0, true_total = 0.0;
    for (size_t i = 0; i < n && err.empty(); ++i)
    {
      const size_t j = (i + 1) % n;
      if (!std::isfinite(measured[i]) || !std::isfinite(truth[i]))
      {
        err = "angka tak-hingga pada titik " + std::to_string(i + 1);
        break;
      }
      const double meas_span = std::fmod(measured[j] - measured[i] + 720.0, 360.0);
      const double true_span = std::fmod(truth[j] - truth[i] + 720.0, 360.0);
      if (!(meas_span > 0.0) || !(true_span > 0.0))
        err = "bentang maju titik " + std::to_string(i + 1) + " -> " + std::to_string(j + 1) +
              " tidak positif";
      meas_total += meas_span;
      true_total += true_span;
    }
    if (err.empty() && std::fabs(meas_total - 360.0) > 1.0)
      err = "total bentang measured " + std::to_string(meas_total) + " bukan 360 +-1";
    if (err.empty() && std::fabs(true_total - 360.0) > 1.0)
      err = "total bentang true " + std::to_string(true_total) + " bukan 360 +-1";
  }
  if (!err.empty())
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Tabel kalibrasi yaw TIDAK SAH: %s -- kalibrasi MATI, yaw firmware mentah",
                 err.c_str());
    return;
  }

  yaw_calib_measured_deg_ = measured;
  yaw_calib_true_deg_ = truth;
  yaw_calib_enabled_ = true;

  std::ostringstream table;
  table.setf(std::ios::fixed);
  table.precision(2);
  for (size_t i = 0; i < n; ++i)
    table << (i ? ", " : "") << measured[i] << "->" << truth[i];
  RCLCPP_WARN(this->get_logger(), "KALIBRASI YAW AKTIF (%zu titik, mag_heading -> arah nyata): %s",
              n, table.str().c_str());
}

double OpenCRModule::correctHeadingDeg(double h) const
{
  const size_t n = yaw_calib_measured_deg_.size();
  const std::vector<double> &m = yaw_calib_measured_deg_;
  const std::vector<double> &t = yaw_calib_true_deg_;
  for (size_t i = 0; i < n; ++i)
  {
    const size_t j = (i + 1) % n;
    const double meas_span = std::fmod(m[j] - m[i] + 720.0, 360.0);
    const double off = std::fmod(h - m[i] + 720.0, 360.0);
    if (off <= meas_span)
      return std::fmod(t[i] + std::fmod(t[j] - t[i] + 720.0, 360.0) * off / meas_span + 720.0, 360.0);
  }
  return h;
}

void OpenCRModule::writeImuControl(uint8_t bits)
{
  // Firmware membersihkan sendiri bit perintah setelah selesai
  // (dxl_node_op3_loop), jadi cukup satu tulisan -- tidak perlu menulis nol
  // sesudahnya, dan menulis nol lebih awal justru membatalkan kalibrasi.
  robotis_controller_msgs::msg::SyncWriteItem sync_write_msg;
  sync_write_msg.item_name = "imu_control";
  sync_write_msg.joint_name.push_back("open-cr");
  sync_write_msg.value.push_back(bits);

  dxl_power_msg_pub_->publish(sync_write_msg);
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
        buttons_press_time_.emplace(
          button_name,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));
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
        publishButtonMsg(button_name);
      }
    }
  }
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

double OpenCRModule::lowPassFilterAngle(double alpha, double x_new, double &x_old)
{
  const double diff = std::atan2(std::sin(x_new - x_old), std::cos(x_new - x_old));
  const double filtered = x_old + alpha * diff;
  x_old = std::atan2(std::sin(filtered), std::cos(filtered));
  return x_old;
}

}  // namespace robotis_op

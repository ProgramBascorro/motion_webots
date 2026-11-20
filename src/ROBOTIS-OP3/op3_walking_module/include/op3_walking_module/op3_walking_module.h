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

#ifndef OP3_WALKING_MODULE_H_
#define OP3_WALKING_MODULE_H_

#include "op3_walking_parameter.h"

#include <stdio.h>
#include <math.h>
#include <fstream>
#include <atomic>
#include <thread>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "op3_walking_module_msgs/srv/get_walking_param.hpp"
#include "op3_walking_module_msgs/srv/set_walking_param.hpp"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_trajectory_calculator.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

namespace robotis_op
{

typedef struct
{
  double x, y, z;
} Position3D;

typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

class WalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingModule>, public rclcpp::Node
{

 public:
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

  WalkingModule();
  virtual ~WalkingModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();
  void onModuleEnable();
  void onModuleDisable();

  int getCurrentPhase()
  {
    return phase_;
  }
  double getBodySwingY()
  {
    return body_swing_y;
  }
  double getBodySwingZ()
  {
    return body_swing_z;
  }

 private:
  enum
  {
    WalkingDisable = 0,
    WalkingEnable = 1,
    WalkingInitPose = 2,
    WalkingReady = 3
  };

  const bool DEBUG;

  void queueThread();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  void walkingParameterCallback(const op3_walking_module_msgs::msg::WalkingParam::SharedPtr msg);
  bool getWalkingParameterCallback(const std::shared_ptr<op3_walking_module_msgs::srv::GetWalkingParam::Request> req,
                                   std::shared_ptr<op3_walking_module_msgs::srv::GetWalkingParam::Response> res);

  /* ROS Service Callback Functions */
  void processPhase(const double &time_unit);
  bool computeLegAngle(double *leg_angle);
  void computeArmAngle(double *arm_angle);
  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  void publishStatusMsg(unsigned int type, std::string msg);
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
  void updateTimeParam(double scale = 1.0);
  void updateMovementParam();
  void updatePoseParam();
  void startWalking();
  void loadWalkingParam(const std::string &path);
  void saveWalkingParam(std::string &path);
  void iniPoseTraGene(double mov_time);
  // Sudut sendi kaki untuk stance diam (swap/leg_move nol) -- neutral yang
  // digelilingi ayunan gait. with_y_offset memilih siapa yang bertanya: penurunan
  // bias mengirim false (supaya init_y_offset tidak saling meniadakan), pelacakan
  // saat diam mengirim true (supaya lebar pijakan saat berdiri sama dengan saat
  // berjalan). Alasan lengkapnya di computeNeutralLegAngle().
  bool computeNeutralLegAngle(double *neutral, bool with_y_offset);
  // Stance netral dari OFFSET RUJUKAN tetap (REF_* di bawah), bukan dari offset
  // yang sedang dimuat. Dipakai HANYA untuk menurunkan walking_bias_.
  bool computeReferenceLegAngle(double *neutral);

  // Offset rujukan penurunan bias: hasil tera ke page 2 (tools_fit, RMS 5,26 deg,
  // 2026-09-09) -- dengan offset persis sebesar ini robot berdiri tepat di page 2.
  //
  // KENAPA TETAP, bukan offset yang sedang dimuat: dulu bias diturunkan dari
  // offset yang berlaku SAAT PENANGKAPAN, sehingga offset hanya berefek sebagai
  // selisih terhadap dirinya sendiri. Operator mengubah pitch ke -0,07, badan
  // maju, disimpan -- lalu setiap penangkapan ulang (restart manager, tombol Init
  // Pose, Apply setelah Studio dibuka lagi) menangkap page 2 DENGAN -0,07 sudah
  // di dalamnya, selisihnya nol, dan badan kembali ke page 2. Nilai tersimpan,
  // efeknya lenyap. Dengan rujukan tetap, stance = page 2 + (efek offset sekarang
  // - efek offset rujukan), sama berapa kali pun modul diaktifkan ulang.
  //
  // Sengaja konstanta, BUKAN kunci di param.yaml: perintah "save" menulis ulang
  // param.yaml hanya dengan kunci yang dikenalnya, jadi kunci tambahan akan hilang,
  // dan menyimpan offset hasil tuning ke sana tidak boleh ikut menggeser rujukan.
  // Nilai y tidak ada: lebar pijakan sengaja ditinggalkan dari acuan bias (lihat
  // computeNeutralLegAngle), jadi y_offset tetap tambahan lebar mutlak.
  static constexpr double REF_X_OFFSET = -0.0427;
  static constexpr double REF_Z_OFFSET = 0.002;
  static constexpr double REF_ROLL_OFFSET_DEG = 1.140186;
  static constexpr double REF_PITCH_OFFSET_DEG = -0.360963;
  static constexpr double REF_YAW_OFFSET_DEG = -1.443854;
  static constexpr double REF_HIP_PITCH_OFFSET_DEG = 6.543178;

  void setJointGains(int balancing_idx);

  OP3KinematicsDynamics* op3_kd_;
  int control_cycle_msec_;
  std::string param_path_;
  std::thread queue_thread_;
  std::mutex publish_mutex_;

  /* ROS Topic Publish Functions */
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr robot_pose_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;

  Eigen::MatrixXd calc_joint_tra_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd init_position_;
  // BERJALAN DI ATAS WALKING_READY. Pose yang tertangkap saat walking module
  // diaktifkan (stance page 2) jadi DASAR MENTAH semua gerakan:
  //   diam    -> goal = walking_bias_ + neutral(offset sekarang)
  //   berjalan-> goal = walking_bias_ + sudut IK + balance
  // walking_bias_ = pose tertangkap - stance netral IK, jadi tepat sesudah
  // penangkapan keduanya sama persis (bias + neutral == tertangkap) dan
  // mengaktifkan modul tidak menimbulkan lompatan sama sekali.
  //
  // Konsekuensinya offset Init Pose berubah arti: ia tidak lagi menentukan tinggi
  // berdiri secara mutlak, melainkan MENGGESER stance relatif terhadap page 2.
  // Itu yang diminta operator 2026-09-09 -- "init pose page 2 jadi init pose raw
  // untuk semua gerakan di simpanan walking, tanpa mengubah parameternya".
  // Membalik keputusan 2026-08-27 yang membuang bias ini.
  //
  // Untuk lengan walking_bias_ menyimpan pose PENUH (tidak dikurangi apa pun),
  // sebab lengan tidak punya offset Init Pose sendiri.
  Eigen::MatrixXd captured_init_pose_;
  Eigen::MatrixXd walking_bias_;
  bool capture_init_pose_;
  // False sampai sebuah penangkapan menghasilkan bias yang sahih (IK bisa gagal).
  // Selama belum sahih, kaki menahan pose tertangkap apa adanya.
  bool walking_bias_valid_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;
  int walking_state_;
  int init_pose_count_;
  op3_walking_module_msgs::msg::WalkingParam walking_param_;
  double previous_x_move_amplitude_;

  //-- Soft start amplitudo x/y/z/angle -----------------------------------
  // Saat gait mulai, x_move (maju/mundur), y_move (geser kiri/kanan), z_move
  // (tinggi angkat kaki) dan angle_move (putar kiri/kanan) naik dari nol ke nilai
  // penuh selama soft_start_time_ detik. angle_move ikut sejak 2026-09-18
  // (permintaan operator: putar kiri/kanan juga harus mulai pelan).
  //
  // Skala dinaikkan KONTINU tiap siklus, bukan per batas fase seperti ROBOTIS
  // mengganti amplitudo: fase 1 adalah saat kaki kiri di titik TERTINGGI, jadi
  // menaikkan tinggi angkat kaki bertahap di situ membuat kaki melonjak tiap
  // tahap. Dengan skala linear yang menerus, lintasan kaki = skala(t) x lintasan
  // asli -- perkalian dua fungsi mulus, tanpa loncatan.
  //
  // Durasinya parameter ROS "soft_start_time" milik node ini (bisa diubah dari
  // Bascorro Studio lewat /op3_walking_module/set_parameters) dan tersimpan di
  // param.yaml, BUKAN field WalkingParam: menambah field pesan mengharuskan
  // rosbridge di-restart lebih dulu, kalau tidak rosbridge menolak SELURUH
  // WalkingParam dan Apply mati total (pernah terjadi dengan y_optional).
  static constexpr double SOFT_START_MAX_SEC = 30.0;
  std::atomic<double> soft_start_time_;    // detik; 0 = soft start mati
  std::atomic<bool> soft_start_request_;   // dipasang "start", dikonsumsi process()
  bool soft_start_active_;
  double soft_start_elapsed_;
  double soft_start_scale_;                // 0..1, dipakai updateMovementParam()
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr soft_start_param_cb_;
  // Tulis HANYA baris soft_start_time di param.yaml (komentar dan kunci lain
  // utuh), langsung saat parameternya diterima. Tanpa ini nilai yang di-Apply
  // dari Studio cuma hidup di memori: satu-satunya jalan ke param.yaml adalah
  // perintah "save", yang diabaikan diam-diam kalau walking_module belum aktif,
  // jadi setelah restart soft start kembali ke nilai lama.
  void persistSoftStartTime(double seconds);

  //
  int balancing_idx_;

  // variable for walking
  double period_time_;
  double dsp_ratio_;
  double ssp_ratio_;
  double x_swap_period_time_;
  double x_move_period_time_;
  double y_swap_period_time_;
  double y_move_period_time_;
  double z_swap_period_time_;
  double z_move_period_time_;
  double a_move_period_time_;
  double ssp_time_;
  double l_ssp_start_time_;
  double l_ssp_end_time_;
  double r_ssp_start_time_;
  double r_ssp_end_time_;
  double phase1_time_;
  double phase2_time_;
  double phase3_time_;

  double x_offset_;
  double y_offset_;
  double z_offset_;
  double r_offset_;
  double p_offset_;
  double a_offset_;

  double x_swap_phase_shift_;
  double x_swap_amplitude_;
  double x_swap_amplitude_shift_;
  double x_move_phase_shift_;
  double x_move_amplitude_;
  double x_move_amplitude_shift_;
  double y_swap_phase_shift_;
  double y_swap_amplitude_;
  double y_swap_amplitude_shift_;
  double y_move_phase_shift_;
  double y_move_amplitude_;
  double y_move_amplitude_shift_;
  double z_swap_phase_shift_;
  double z_swap_amplitude_;
  double z_swap_amplitude_shift_;
  double z_move_phase_shift_;
  double z_move_amplitude_;
  double z_move_amplitude_shift_;
  double a_move_phase_shift_;
  double a_move_amplitude_;
  double a_move_amplitude_shift_;

  double pelvis_offset_;
  double pelvis_swing_;
  double hit_pitch_offset_;
  double arm_swing_gain_;

  bool ctrl_running_;
  bool real_running_;
  double time_;

  int phase_;
  double body_swing_y;
  double body_swing_z;
};

}

#endif /* OP3_WALKING_MODULE_H_ */

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
  // Leg joint angles for the standstill (zero-movement) neutral stance the gait
  // oscillates around — used to derive walking_bias_ so the stance locks to the
  // captured WALKING_READY pose. Returns false if the IK has no solution.
  // with_y_offset selects which of the two callers is asking: the bias derivation
  // passes false (so init_y_offset survives the subtraction and stays a live knob),
  // the idle tracking passes true (so standing already shows the same foot
  // separation the gait will use). See computeNeutralLegAngle() for why.
  // src memilih SET PARAMETER mana yang dipakai menghitung stance netral itu, dan
  // itu menentukan segalanya. Sebelum 2026-09-09 fungsi ini selalu memakai
  // walking_param_ yang sedang hidup, sehingga penangkapan bias memakai preset
  // yang baru saja di-Apply -- offset init preset itu dikurangkan terhadap
  // dirinya sendiri dan hasilnya NOL. Robot berdiri persis di page 2 dan
  // init_x/init_pitch operator tidak pernah terasa. Sekarang penangkapan memakai
  // bias_reference_param_ (isi param.yaml, tetap sepanjang sesi).
  bool computeNeutralLegAngle(double *neutral, bool with_y_offset,
                              const op3_walking_module_msgs::msg::WalkingParam &src);
  void computeArmAngle(double *arm_angle);
  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  void publishStatusMsg(unsigned int type, std::string msg);
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
  void updateTimeParam(double scale = 1.0);
  // Pengali period_time dari soft start: SOFT_START_SLOW_ saat ramp = 0, lalu
  // turun mulus ke 1.0 saat ramp penuh.
  double timeRampScale();
  // Menggeser satu amplitudo menuju targetnya dengan laju "0 -> target tepat
  // dalam soft_start_time detik". Mengecil ke arah nol tidak pernah ditahan.
  double rampToward(double current, double target, double dt);
  void updateMovementParam();
  void updatePoseParam();
  void startWalking();
  void loadWalkingParam(const std::string &path);
  void saveWalkingParam(std::string &path);
  void iniPoseTraGene(double mov_time);

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
  // Walk-in-WALKING_READY: pose captured when walking is enabled (the WALKING_READY
  // standing stance). Idle -> legs follow the neutral stance of the offsets currently
  // in force (identical to the captured pose right after the grab, see process());
  // walking -> walking_bias_ (= captured - neutral IK) + gait oscillation, so
  // the stance stays locked to WALKING_READY. capture_init_pose_ triggers the grab
  // on the first process() cycle after onModuleEnable().
  Eigen::MatrixXd captured_init_pose_;
  Eigen::MatrixXd walking_bias_;
  bool capture_init_pose_;
  // False until a capture has produced a usable bias (the IK can fail). Idle offset
  // tracking is only meaningful once the bias is trustworthy; otherwise the legs stay
  // on the frozen captured pose.
  bool walking_bias_valid_;
  // Cap on how far a leg joint may move per control cycle while standing still, so a
  // typed-in offset ramps instead of slamming. Radians per cycle.
  double idle_track_max_step_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;
  int walking_state_;
  int init_pose_count_;
  op3_walking_module_msgs::msg::WalkingParam walking_param_;
  // ACUAN TETAP untuk walking_bias_: isi param.yaml apa adanya, diambil sekali
  // saat boot dan tidak pernah ikut berubah waktu operator menekan Apply.
  //
  // Dengan ini pose berdiri jadi fungsi yang pasti dari parameter yang hidup:
  //
  //     pose = page 2 + [ neutral(parameter hidup) - neutral(param.yaml) ]
  //
  // dan hasilnya SAMA saja Apply dikirim sebelum atau sesudah walking_module
  // diaktifkan. Dulu keduanya memberi hasil berbeda dengan nilai yang sama
  // persis, karena acuannya ikut bergeser mengikuti Apply terakhir.
  op3_walking_module_msgs::msg::WalkingParam bias_reference_param_;
  double previous_x_move_amplitude_;

  // Soft start (walking_param_.soft_start_time detik, knob "Soft start" di grup
  // Timing Bascorro Studio). Nilai di Studio adalah BATAS ATAS: sesudah "start"
  // robot merangkak naik ke sana, bukan memakainya utuh di langkah pertama.
  //
  // Yang terpengaruh HANYA grup Movement dan Timing:
  //   Movement -> x/y/z/angle_move_amplitude, masing-masing dibatasi lajunya.
  //   Timing   -> period_time saja, lewat timeRampScale(). dsp_ratio dan
  //               step_fb_ratio TIDAK ikut: keduanya rasio pembagi fase, dan
  //               dsp_ratio yang dinaikkan dari nol berarti fase dua kaki
  //               menapak hilang justru di langkah pertama -- kebalikan dari
  //               yang dicari.
  // Grup lain (Init pose, Gait shape, Balance, Gain) tidak disentuh sama sekali.
  //
  // Pembatasnya PER SUMBU, bukan satu pengali bersama. Satu pengali bersama
  // terlihat lebih ringkas tapi salah perilaku: menaikkan x di tengah jalan ikut
  // menyeret turun tinggi angkat kaki, dan menambah belokan dari nol menyeret
  // seluruh gait ke nol. Per sumbu, tiap knob berdiri sendiri.
  //
  // Nilai ini maju per siklus kontrol supaya lamanya tetap soft_start_time
  // berapa pun period_time-nya, tapi hanya DISALIN ke amplitudo kerja di batas
  // fase (updateMovementParam) -- amplitudo tidak boleh berubah di tengah ayunan.
  double x_move_ramped_;
  double y_move_ramped_;
  double z_move_ramped_;
  double a_move_ramped_;
  // Sumbu kelima: angle_move_amplitude_turn_in ("putar dalam").
  double ai_move_ramped_;
  // 0 -> 1 sepanjang soft_start_time, khusus untuk tempo. Tempo tidak punya
  // "sumbu", jadi satu kemajuan skalar sudah cukup.
  double move_ramp_;
  // Pengali period_time saat move_ramp_ masih 0. 1.5 diambil dari startWalking()
  // yang sebelumnya memanggil updateTimeParam(1.5): nilai awalnya sama persis,
  // bedanya sekarang luruh mulus alih-alih balik mendadak ke tempo penuh setelah
  // satu siklus.
  static constexpr double SOFT_START_SLOW_ = 1.5;

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
  // Amplitudo kerja "putar dalam". Fase/periodenya menumpang punya a_move_,
  // hanya besar dan tandanya yang berdiri sendiri. Sengaja TIDAK punya
  // pasangan _shift_: lihat updateMovementParam().
  double ai_move_amplitude_;

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

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

/* Author: Kayman Jung */

#ifndef BALL_FOLLOWER_H_
#define BALL_FOLLOWER_H_

#include <math.h>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "op3_ball_detector_msgs/msg/circle_set_stamped.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "op3_walking_module_msgs/srv/get_walking_param.hpp"

namespace robotis_op
{

// following the ball using walking
class BallFollower // : public rclcpp::Node
{
 public:
  enum
  {
    NotFound = 0,
    OutOfRange = 1,
    OnRight = 2,
    OnLeft = 3,
  };

  BallFollower();
  ~BallFollower();

  // ball_size : sudut jari-jari bola dalam RADIAN (dipakai di dalam tan(), oper
  //             0.0 kalau tidak dipakai -- JANGAN oper piksel ke sini).
  // ball_radius_px : jari-jari bola dalam PIKSEL apa adanya dari detektor,
  //             dipakai sebagai pengukur jarak yang berdiri sendiri.
  bool processFollowing(double x_angle, double y_angle, double ball_size,
                        double ball_radius_px = 0.0);
  void decideBallPositin(double x_angle, double y_angle);
  void waitFollowing();
  void startFollowing();
  void stopFollowing();
  void clearBallPosition()
  {
    approach_ball_position_ = NotFound;
  }

  int getBallPosition()
  {
    return approach_ball_position_;
  }

  bool isBallInRange()
  {
    return (approach_ball_position_ == OnRight || approach_ball_position_ == OnLeft);
  }

  rclcpp::Node::SharedPtr node_;
  void setNode(rclcpp::Node::SharedPtr node);

 protected:
  const bool DEBUG_PRINT;
  // Geometri pemicu tendangan. Dulu CAMERA_HEIGHT konstanta 0,56 m; sekarang
  // bisa disetel lewat parameter ROS (kick_camera_height, kick_distance,
  // kick_head_tilt_sign) supaya bisa ditera langsung di robot tanpa compile
  // ulang. Urutan deklarasi sengaja menempati posisi CAMERA_HEIGHT yang lama
  // supaya urutan init-list di konstruktor tetap cocok.
  double camera_height_;
  double kick_distance_;
  // Tanda konvensi head_tilt:
  //   +1  head_tilt NEGATIF berarti menunduk  (konvensi OP3 asli)
  //   -1  head_tilt POSITIF berarti menunduk  (ALPHONSE)
  // Lihat penjelasan panjang di processFollowing() pada .cpp.
  double head_tilt_sign_;
  // Klok untuk RCLCPP_*_THROTTLE. WAJIB hidup lebih lama dari pemanggilannya:
  // makro THROTTLE menyimpan REFERENSI ke klok lalu memakainya lagi di
  // pernyataan berikutnya di dalam blok makro. Menuliskan
  // `*rclcpp::Clock::make_shared()` langsung di tempat pemanggilan membuat
  // shared_ptr sementara yang mati di akhir pernyataan PERTAMA, sehingga
  // pemakaian berikutnya membaca objek yang sudah dibebaskan -- SIGSEGV di
  // rclcpp::Clock::now(). Itu yang membunuh op_demo_node ~0,5 detik sesudah
  // demo soccer mulai.
  rclcpp::Clock::SharedPtr log_clock_;
  // Pemicu tendangan berdasarkan sudut SERVO head_pan -- diukur langsung di
  // robot oleh operator, dalam satuan yang dibaca Dynamixel/Studio (0..360,
  // tengah 180). Bola di KANAN + kepala menoleh kanan sampai servo ~156-158
  // -> tendang kaki kanan; bola di KIRI + servo ~182-184 -> kaki kiri.
  //
  // Konversi: XM430/MX-28 memetakan 0 rad ke nilai 2048 = 180 derajat servo,
  // dan 4096 langkah menutup 360 derajat, jadi
  //     servo_deg = pusat + arah * (head_pan dalam derajat).
  // head_pan positif = menoleh KIRI, dan angka operator memang > 180 untuk
  // kiri, jadi arah = +1. Keduanya tetap parameter kalau servo dipasang
  // terbalik atau offset-nya diubah.
  double kick_pan_right_min_deg_;
  double kick_pan_right_max_deg_;
  double kick_pan_left_min_deg_;
  double kick_pan_left_max_deg_;
  double head_pan_servo_center_deg_;
  double head_pan_servo_dir_;
  // Bola harus sedekat ini juga sebelum jendela pan boleh menendang.
  // Jendela KIRI (servo 182-184 = +2..+4 derajat) duduk hampir di tengah,
  // yaitu sudut leher yang WAJAR saat robot masih berjalan menuju bola yang
  // jauh -- tanpa pagar ini robot bisa menendang angin dari 2 meter. Sedikit
  // lebih longgar dari kick_distance supaya jendela pan boleh menyala sesaat
  // sebelum robot benar-benar berhenti. Besarkan kalau ingin jendela pan
  // berdiri sepenuhnya sendiri.
  double kick_pan_max_distance_;
  // Ambang jari-jari bola (piksel) untuk menendang. Pengukur jarak yang jauh
  // lebih sedikit asumsinya daripada sudut kepala: tidak peduli tinggi kamera,
  // hip_pitch_offset, tanda head_tilt, ataupun apakah kepala benar-benar
  // menunjuk ke bola -- semuanya sudah pernah salah dan memakan waktu berjam-jam.
  // Bola yang dekat SELALU besar di gambar.
  //
  // Kalibrasinya satu angka: taruh bola di tempat yang diinginkan, baca
  // "radius=NN.Npx" di log detektor, pasang angka itu.
  //
  // Terukur di robot: radius 13,2 px = 3,75 m ... 92 px = 0,30 m (berdiri).
  // Bawaan 85 px (~0,38 m saat berdiri) SENGAJA sedikit lebih jauh dari titik
  // kontak ideal: badan robot sendiri menutupi bola di bawah ~0,32 m, jadi
  // memicu di titik kontak berarti memicu saat bola sudah tidak terlihat.
  // Ayunan kaki yang menutup sisanya. Setel 0 untuk mematikan jalur ini.
  double kick_ball_radius_px_;
  // Berapa siklus berturut-turut syaratnya harus benar sebelum menendang.
  // Jendela servo cuma selebar 2 derajat, jadi kalau kepala masih bergoyang
  // angka besar bikin pemicunya tidak pernah penuh -- turunkan kalau begitu.
  int kick_ready_count_;
  const int NOT_FOUND_THRESHOLD;
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  const double MAX_FB_STEP;
  const double MAX_RL_TURN;
  const double IN_PLACE_FB_STEP;
  const double MIN_FB_STEP;
  const double MIN_RL_TURN;
  const double UNIT_FB_STEP;
  const double UNIT_RL_TURN;

  const double SPOT_FB_OFFSET;
  const double SPOT_RL_OFFSET;
  const double SPOT_ANGLE_OFFSET;

  // Sudut servo head_pan (0..360, tengah 180) dari sudut sendi saat ini.
  double headPanServoDeg() const;
  // OnRight / OnLeft kalau servo head_pan sedang berada di salah satu jendela
  // tendang, OutOfRange kalau tidak.
  int kickFootFromHeadPan() const;

  void currentJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void setWalkingCommand(const std::string &command);
  void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);
  bool getWalkingParam();
  void calcFootstep(double target_distance, double target_angle, double delta_time,
                    double& fb_move, double& rl_angle);

  //image publisher/subscriber
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_control_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_joint_pub_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr head_scan_pub_;
  // Persistent member publishers (NOT throwaway locals). A publisher created
  // inside setWalkingCommand()/setWalkingParam() and destroyed on function
  // return can drop its message when DDS discovery/matching with walking_module
  // hasn't finished yet — this intermittently lost the walking "stop" command,
  // so the legs kept walking after the STOP button ("kadang jalan terus"), and
  // likewise dropped "start". Created once in setNode(), they are matched with
  // walking_module long before the first button press, so every command lands.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_walking_command_pub_;
  rclcpp::Publisher<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr set_walking_param_pub_;

  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;
  // rclcpp::Client<op3_walking_module_msgs::srv::GetWalkingParam>::SharedPtr get_walking_param_client_;

  // rclcpp::Subscription<op3_ball_detector_msgs::msg::CircleSetStamped>::SharedPtr ball_position_sub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ball_tracking_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_states_sub_;

  // (x, y) is the center position of the ball in image coordinates
  // z is the ball radius
  geometry_msgs::msg::Point ball_position_;
  op3_walking_module_msgs::msg::WalkingParam current_walking_param_;

  int count_not_found_;
  int count_to_kick_;
  bool on_tracking_;
  int approach_ball_position_;
  double current_pan_, current_tilt_;
  double current_x_move_, current_r_angle_;
  int kick_motion_index_;
  double hip_pitch_offset_;
  rclcpp::Time prev_time_;

  double curr_period_time_;
  double accum_period_time_;

};
}

#endif /* BALL_FOLLOWER_H_ */

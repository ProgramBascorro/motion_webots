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
#include <chrono>
#include <deque>
#include <map>
#include <string>
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
#include "op3_demo/kick_alignment_policy.h"

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

  // ===== Penyelarasan yaw sebelum menendang =================================
  //
  // Urutan yang diminta: deteksi bola -> dekati bola -> begitu bola masuk
  // jangkauan tendang, JANGAN langsung menendang. Putar badan di tempat dulu
  // sampai yaw kembali ke arah acuan (bawaan 0 derajat, toleransi +-5), baru
  // menendang.
  //
  // "Yaw 0" itu arah yang DIHADAPI ROBOT SAAT OPENCR MENYALA: papan itu
  // menolkan acuan headingnya sendiri waktu start, jadi nol-nya ikut ke mana
  // pun robot diarahkan saat dinyalakan. Konsekuensi praktisnya: hadapkan robot
  // ke gawang waktu menyalakannya, dan penyelarasan ini berarti "hadap gawang
  // dulu, baru tendang".
  //
  // Mekanismenya duduk di kelas ini, bukan di SoccerDemo, karena SELURUH
  // perintah ke walking_module (dan seluruh parameter kick_*) memang milik
  // kelas ini. SoccerDemo hanya menyuapkan yaw dari callback IMU yang sudah
  // dimilikinya, lalu memanggil processYawAlign() sekali per tick process().
  //
  // Semua langkahnya TIDAK MEMBLOKIR, satu langkah per tick. Itu bukan selera
  // gaya: demo_node berjalan satu utas -- process() lalu spin_some() -- jadi
  // loop tunggu di dalam process() ikut membekukan callback IMU yang justru
  // menjadi syarat berhentinya loop itu sendiri.

  // Yaw badan dalam DERAJAT, dari /robotis/open_cr/imu.
  void setBodyYaw(double yaw_deg);
  void updateBallObservation(double x_angle, double y_angle, double radius_px);
  bool canKickNow() const;
  // mag_status OpenCR: 0 = papan tanpa magnetometer, 1 = belum settle, 2 = OK,
  // 3 = sedang kalibrasi hard-iron. Pada 0 dan 1, open_cr_module MEMAKSA
  // yaw = 0, jadi angkanya bukan pengukuran dan tidak boleh dipercaya.
  void setMagStatus(int mag_status);
  // Laju putar sumbu tegak, rad/detik, dari gyro OpenCR. Dipanggil tiap pesan
  // IMU. Selama fase memutar, nilainya langsung DIINTEGRASIKAN di sini supaya
  // tiap sampel terpakai, bukan cuma yang kebetulan terbaca saat tick demo.
  void setBodyGyroZ(double gyro_z_rad_per_sec);

  // Menghentikan pelacakan bola TANPA menghentikan gait. Dipakai sebelum
  // memutar: kalau gait ikut dimatikan, perintah putar tidak punya efek.
  void pauseFollowingForTurn();
  bool startYawAlign();
  // Satu langkah penyelarasan. true = selesai, boleh menendang.
  // ball_visible: pelacak sedang benar-benar melihat bola. Hanya saat itu
  // sudut leher boleh dipercaya sebagai arah bola untuk menjaga bola di tengah.
  // ball_radius_px: jari-jari bola dalam piksel dari detektor. Dipakai fase
  // MENGGESER sebagai pengukur jarak -- ambangnya sama persis dengan pemicu
  // tendangan (kick_ball_radius_px), jadi "sudah cukup dekat" berarti hal yang
  // sama di kedua tempat.
  bool processYawAlign(bool ball_visible = false, double ball_radius_px = 0.0);
  void cancelYawAlign();
  // Berapa lama fase reda sesudah putaran berhenti. Dibaca soccer_demo supaya
  // jeda pra-tendang tidak menghitung ulang waktu diam yang SUDAH dijalani.
  // Lama fase reda yang BENAR-BENAR dijalani, bukan angka yang diminta
  // operator. handleKick() menguranginya dari kick_pre_delay_sec supaya robot
  // tidak menunggu hal yang sama dua kali; kalau yang dilaporkan angka mentah
  // (0,7 s) sementara yang dijalani 1,8 s, selisihnya jadi 1,1 detik menunggu
  // percuma tepat sebelum menendang.
  //
  // Yang dikembalikan angka yang BENAR-BENAR dipasang fase reda terakhir
  // (last_settle_sec_), bukan hasil hitung ulang. Sejak periode gait mengikuti
  // preset, settleWaitSec() memberi jawaban berbeda tergantung preset mana yang
  // sedang tertulis: fase reda dipasang saat preset PUTAR/GESER masih hidup
  // (blok 0,55-0,57 s), sedangkan di sini parameter dasar sudah dikembalikan.
  // Menghitung ulang berarti melaporkan 1,8 s untuk fase yang cuma berjalan
  // 1,3 s, lalu memotong jeda pra-tendang yang justru menjaga robot tidak
  // roboh.
  double yawSettleSec() const
  {
    return (last_settle_sec_ > 0.0) ? last_settle_sec_ : settleWaitSec(yaw_settle_sec_);
  }
  // Kaki penendang sudah ditetapkan fase menggeser; jangan dihitung ulang.
  bool footDecidedByReposition() const
  {
    return foot_from_reposition_;
  }
  bool isYawAligning() const
  {
    return yaw_align_state_ != YawAlignOff;
  }
  bool refreshFootAfterAlign() const
  {
    return refresh_foot_after_align_;
  }

  // ---- ACUAN POSE: dipanggil SEBELUM walking_module diaktifkan lagi ------
  //
  // walking_module merekam walking_bias_ = pose_page2 - neutral(offset yang
  // SEDANG hidup) tepat saat modulnya dinyalakan, dan sesudah itu pose robot =
  // page2 + [neutral(preset) - neutral(offset saat perekaman)]. Jadi yang
  // sampai ke robot cuma SELISIHNYA terhadap apa pun yang kebetulan hidup
  // waktu modul dinyalakan.
  //
  // Pada demo PERTAMA sesudah op3_manager hidup, yang hidup adalah param.yaml
  // dan selisihnya benar. Tapi sesudah robot JATUH, urutannya jadi:
  //
  //   demo jalan  -> preset JALAN/MENCARI CELAH tertulis di walking_module
  //   robot jatuh -> action_module mengambil alih untuk gerakan bangun
  //   bangkit     -> walking_module dinyalakan LAGI, dan bias direkam terhadap
  //                  preset tadi, BUKAN param.yaml
  //   demo lanjut -> preset JALAN diterbitkan lagi
  //
  // Kalau preset yang tertulis kebetulan JALAN (kasus paling sering: robot
  // jatuh sambil mengejar bola), selisihnya jadi NOL dan robot terkunci di
  // pose page 2 -- seluruh init pose hasil setelan operator mati total.
  // Kalau yang tertulis MENCARI CELAH atau PUTAR, posenya bergeser sebesar
  // selisih preset-ke-preset, bukan preset-ke-param.yaml.
  //
  // publishPoseReference() menerbitkan ulang salinan parameter yang dibaca
  // SEKALI pada demo pertama, sehingga bias selalu direkam terhadap acuan yang
  // sama persis seperti sesudah boot. Aman dipanggil selagi action_module
  // memegang kaki: walkingParameterCallback() hanya menyimpan struct dan tidak
  // menggerakkan sendi apa pun.
  //
  // Mengembalikan true kalau acuannya benar-benar dikirim.
  bool publishPoseReference();

  rclcpp::Node::SharedPtr node_;
  void setNode(rclcpp::Node::SharedPtr node);

 protected:
  enum YawAlignState
  {
    YawAlignOff = 0,       // tidak sedang menyelaraskan
    YawAlignBraking = 3,   // MENGEREM: berhenti di depan bola sebelum memutar
    YawAlignTurning = 1,   // sedang MENGITARI bola (atau memutar di tempat)
    YawAlignSettling = 2,  // sudah lurus, menunggu gait benar-benar berhenti
    // Sesudah mengitari bola, badan sudah menghadap yaw target tapi kakinya
    // belum tentu masih di sebelah bola -- mengitari memindahkan seluruh badan.
    // Di fase ini robot menggeser (dan maju kalau perlu) TANPA memutar sedikit
    // pun, sampai bola benar-benar ada di depan kaki terdekat.
    YawAlignRepositioning = 4,
    // Diam sejenak sesudah menggeser, lalu yaw diperiksa sekali lagi dengan
    // magnetometer -- satu-satunya saat ia bisa dipercaya. Kalau geseran tadi
    // merusak yaw, robot mengitari lagi (jatahnya terbatas).
    YawAlignPostSettle = 5,
  };

  const bool DEBUG_PRINT;
  // Geometri pemicu tendangan. Dulu CAMERA_HEIGHT konstanta 0,56 m; sekarang
  // bisa disetel lewat parameter ROS (kick_camera_height, kick_distance,
  // kick_head_tilt_sign) supaya bisa ditera langsung di robot tanpa compile
  // ulang. Urutan deklarasi sengaja menempati posisi CAMERA_HEIGHT yang lama
  // supaya urutan init-list di konstruktor tetap cocok.
  double camera_height_;
  double camera_pitch_offset_deg_;
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
  // Bahan untuk menghitung kick_ball_radius_px_ sendiri kalau tidak dipasang
  // manual. Semuanya besaran fisik yang bisa diukur, bukan angka ajaib.
  double kick_ball_real_radius_m_;   // jari-jari bola sungguhan (ukuran 5 = 0,11 m)
  double kick_fov_width_deg_;        // setengah-FOV mendatar kamera
  double kick_image_width_px_;       // lebar gambar
  // Potongan pada ambang radius hasil hitungan, pecahan 0..1. Ada karena
  // radius TERUKUR sistematis lebih kecil daripada hitungan ideal saat bola
  // dekat kaki: di situ badan robot sendiri menutupi sebagian bola, kotak
  // deteksi jadi mengecil. Lihat setNode().
  double kick_ball_radius_margin_;
  // Jalur pemicu TENDANG dari jari-jari bola: MATI atas permintaan operator
  // (2026-09-01). Ambangnya tetap dihitung dan tetap dicetak di telemetri
  // supaya bisa ditera, tapi ia tidak lagi ikut memutuskan kapan menendang
  // maupun kapan fase menggeser boleh selesai.
  //
  // Alasannya terukur: radius dari detektor melompat 12..87 px untuk bola yang
  // sama, dan robot pernah memutuskan "bola sudah dekat" dari 1,357 m padahal
  // kick_distance 0,30 m. Di fase menggeser akibatnya justru terbalik: radius
  // tidak pernah mencapai ambang, jadi "siap" tidak pernah naik dari 0/4 dan
  // geseran selalu habis di batas waktu 12 detik.
  bool radius_ready_enable_;
  // JARI-JARI bola sebagai PENGUKUR JARAK. Mati atas permintaan operator
  // 2026-09-03: jaraknya sekarang murni dari sudut sendi kepala + geometri.
  // Jalur radius dibiarkan utuh di kode supaya bisa dihidupkan lagi dengan
  // kick_distance_from_radius:=1.0 tanpa mengubah apa pun.
  bool distance_from_radius_;

  // ---- Mutu pengukuran jarak dari sudut kepala ---------------------------
  //
  // Tiga hal yang membuat angkanya bisa dipercaya, semuanya diukur dari 77
  // sampel telemetri run op_demo_node_6476:
  //
  //  1. TINGGI EFEKTIF. Kamera mengunci PUSAT bola, yang ada satu jari-jari
  //     bola di atas lantai. Jadi pembilang rumusnya bukan tinggi kamera
  //     melainkan (tinggi kamera - jari-jari bola). Memakai tinggi kamera
  //     penuh melebihkan jarak 1,244x secara sistematis.
  //
  //  2. LANTAI SUDUT TUNDUK. Jarak = H/tan(tunduk), jadi saat kepala hampir
  //     datar angkanya meledak: pada tunduk 3 derajat satu derajat galat
  //     menggeser jarak lebih dari 30 %, sedangkan pada ambang berhenti
  //     0,30 m (tunduk 62 derajat) cuma 3 %. Di bawah lantai ini bacaannya
  //     TIDAK dipakai sama sekali: robot dianggap masih jauh dan pemicu
  //     tendangan ditahan. 9 dari 77 sampel ada di bawah 5 derajat.
  //
  //  3. PENAPIS MEDIAN. Sudut leher yang segar dijumlahkan dengan posisi bola
  //     dari frame yang bisa berumur ratusan milidetik, jadi tiap kali kepala
  //     bergerak ada satu-dua sampel yang meleset jauh. Median beberapa sampel
  //     terakhir membuangnya tanpa menambah keterlambatan berarti; median
  //     bukan rata-rata justru supaya satu sampel liar tidak ikut menggeser
  //     hasilnya sedikit pun.
  double min_depression_deg_;   // lantai sudut tunduk, derajat
  int distance_filter_n_;       // panjang jendela median; <= 1 = penapis mati

  // Letak bola DI DALAM gambar pada pembacaan terakhir (radian, konvensi
  // ball_tracker: x positif = bola di kiri gambar, y positif = bola di atas).
  // Disimpan supaya headPanServoDeg() -- yang dipanggil dari banyak tempat
  // tanpa membawa argumen -- bisa ikut memperhitungkannya.
  double ball_img_pan_rad_;
  double ball_img_tilt_rad_;
  double observed_ball_radius_px_ = 0.0;
  std::chrono::steady_clock::time_point ball_observation_stamp_;
  std::chrono::steady_clock::time_point head_state_stamp_;
  // false = jendela tendang dinilai dari sudut leher saja (perilaku sebelum
  // 2026-09-04).
  bool pan_window_from_bearing_;
  // Pemeriksaan preset ke walking_module cuma dijalankan sekali per demo.
  bool preset_verified_;
  std::deque<double> distance_buf_;
  // Jarak yang dilaporkan saat bacaannya TIDAK sahih. Sengaja besar: di atas
  // 0,46 m panjang langkah sudah mentok di batas atasnya, jadi angka ini tidak
  // mengubah cara robot berjalan -- ia cuma menahan pemicu tendangan.
  static constexpr double DISTANCE_FAR_M = 10.0;
  void resetDistanceFilter();
  // Tinggi angkat kaki (z_move_amplitude) dibuat naik bertahap bersama
  // x_move_amplitude. z_move_max_ = nilai foot_height dari param.yaml, dipakai
  // sebagai BATAS ATAS; z_move_min_ratio_ = pecahan batas atas saat robot nyaris
  // diam. Lihat setWalkingParam().
  double z_move_max_;
  bool z_move_max_captured_;   // batas atas dikunci sekali, lihat getWalkingParam()
  double z_move_min_ratio_;
  // URUTAN RAMP: tinggi angkat kaki (z) naik DULU sampai penuh, langkah maju
  // (x) baru merambat naik sesudahnya. Sebelumnya keduanya naik berbarengan,
  // sehingga langkah pertama diayun dengan kaki yang baru terangkat 10 mm dan
  // telapaknya menyeret lantai. Lihat calcFootstep() dan setWalkingParam().
  bool z_lead_enable_;
  double z_lead_progress_;     // 0..1; 1 = z sudah di batas atas
  double z_lead_step_mm_;      // kenaikan z per tick ramp (tiap seperempat periode gait)
  // Berapa siklus berturut-turut syaratnya harus benar sebelum menendang.
  // Jendela servo cuma selebar 2 derajat, jadi kalau kepala masih bergoyang
  // angka besar bikin pemicunya tidak pernah penuh -- turunkan kalau begitu.
  int kick_ready_count_;
  const int NOT_FOUND_THRESHOLD;
  const double FOV_WIDTH;
  const double FOV_HEIGHT;
  // BATAS ATAS langkah maju. Bukan const lagi: nilainya diambil dari
  // x_move_amplitude preset "jalan" (JALAN MENUJU MEKKAH) kalau presetnya ada, jadi
  // kecepatan jalan robot benar-benar mengikuti setelan Bascorro Studio alih-
  // alih angka yang tertulis di sini. 18,1 mm = nilai preset lama "dari ketua",
  // dipakai kalau presetnya tidak dimuat.
  double max_fb_step_;
  const double MAX_RL_TURN;
  const double IN_PLACE_FB_STEP;
  const double MIN_FB_STEP;
  const double MIN_RL_TURN;
  const double UNIT_FB_STEP;
  // Langkah ramp maju yang BENAR-BENAR dipakai. Menimpa UNIT_FB_STEP supaya
  // laju naiknya bisa disetel tanpa compile ulang; UNIT_FB_STEP tetap ada
  // sebagai nilai asli ROBOTIS.
  double unit_fb_step_;
  const double UNIT_RL_TURN;

  const double SPOT_FB_OFFSET;
  const double SPOT_RL_OFFSET;
  const double SPOT_ANGLE_OFFSET;

  // Arah bola terhadap BADAN, dihitung penuh 3 dimensi dari sudut leher, sudut
  // tunduk kamera, dan letak bola di dalam gambar. Menggantikan penjumlahan
  // sudut yang lama (head_pan + sudut gambar), yang cuma sahih kalau kamera
  // mendatar -- lihat catatan panjang di ballDirection().
  //   azimuth_rad    : positif = bola di KIRI badan
  //   depression_rad : positif = arah pandang ke bola ada DI BAWAH horizon
  void ballDirection(double x_angle, double y_angle,
                     double &azimuth_rad, double &depression_rad) const;

  // Sudut servo head_pan (0..360, tengah 180). Sejak 2026-09-04 ini menyatakan
  // arah BOLA, bukan arah leher: keduanya sama hanya selama kepala benar-benar
  // mengunci bola di tengah gambar. Setel kick_pan_window_from_bearing:=0.0
  // untuk kembali memakai sudut leher apa adanya.
  double headPanServoDeg() const;
  // OnRight / OnLeft kalau servo head_pan sedang berada di salah satu jendela
  // tendang, OutOfRange kalau tidak.
  int kickFootFromHeadPan() const;

  void currentJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // Menerbitkan perintah apa adanya, tanpa efek samping. Dipisah dari
  // setWalkingCommand() karena versi itu memanggil getWalkingParam() untuk
  // "start", dan panggilan service itu MEMBLOKIR loop demo -- termasuk callback
  // IMU yang dibutuhkan penyelarasan yaw. Lihat startInPlaceTurn().
  void publishWalkingCommand(const std::string &command);
  void setWalkingCommand(const std::string &command);
  // Tidak lagi menerima argumen "balance". Dulu setiap publish memaksa
  // balance_enable = true, sehingga Balance yang dimatikan lewat param.yaml atau
  // Bascorro Studio selalu menyala lagi begitu demo mengirim kecepatan berikutnya
  // -- dan itu tiap siklus. Sekarang nilainya dibiarkan apa adanya seperti yang
  // dibaca getWalkingParam() dari walking_module, jadi konfigurasi yang menentukan.
  void setWalkingParam(double x_move, double y_move, double rotation_angle);
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

  // ---- Penyelarasan yaw --------------------------------------------------
  // Mulai memutar di tempat (walking "start" dengan amplitudo maju/samping nol).
  void brakeBeforeTurn();
  // Masuk fase MENGEREM dan memasang semua penghitung waktunya. Satu tempat
  // untuk ketiga jalan masuk (mulai menyelaraskan, cek ulang sesudah memutar,
  // cek ulang sesudah menggeser) supaya ketiganya tidak bisa berbeda isi.
  // Mengembalikan lama MINIMUM mengerem, detik.
  double enterBrakePhase(double yaw_now_deg);
  void startInPlaceTurn();
  // Nolkan amplitudo, kirim "stop", lalu masuk fase menunggu gait reda.
  // Selalu mengembalikan false: selesainya diputuskan tick berikutnya.
  bool finishYawTurn(const char *reason);
  // Yaw boleh dipakai kalau pernah terbaca, magnetometernya melapor siap, dan
  // pesan terakhirnya belum basi.
  bool isBodyYawUsable() const;

  int yaw_align_state_;
  // Yaw terakhir dari IMU + kapan ia tiba. steady_clock, BUKAN jam ROS:
  // membandingkan dua rclcpp::Time dengan clock type berbeda melempar
  // exception, dan di berkas ini ada dua sumber waktu ROS yang berbeda
  // (rclcpp::Clock() sistem pada prev_time_ vs jam node).
  double body_yaw_deg_;
  bool body_yaw_seen_;
  std::chrono::steady_clock::time_point body_yaw_stamp_;
  int mag_status_;                 // -1 = belum pernah terdengar

  std::chrono::steady_clock::time_point yaw_align_started_at_;
  std::chrono::steady_clock::time_point yaw_align_deadline_;
  std::chrono::steady_clock::time_point yaw_settle_until_;
  std::chrono::steady_clock::time_point yaw_sign_check_at_;
  int yaw_in_window_count_;
  double yaw_turn_sign_;           // salinan kerja, bisa dibalik sendiri
  double yaw_align_ref_error_;     // |galat| saat mulai, bahan cek tanda
  bool yaw_sign_settled_;          // cek tanda otomatis sudah selesai

  // Batas bawah putaran yang MENINGKAT sendiri saat robot tidak kunjung
  // berputar. Terukur di robot 2026-08-28: perintah 13,7 deg/langkah membuat
  // yaw bergerak -39 -> -4 dalam 3,7 detik, tapi perintah 5,0 deg/langkah
  // (batas bawah) sama sekali tidak menggerakkannya -- yaw cuma melayang
  // -12 ... -17. Jadi ada ambang torsi minimum, dan galat kecil justru
  // menghasilkan perintah di bawah ambang itu. Tanpa eskalasi, "berputar sampai
  // benar-benar lurus" berubah jadi "berputar selamanya".
  double yaw_turn_floor_deg_;
  std::chrono::steady_clock::time_point yaw_stall_check_at_;
  double yaw_stall_ref_error_;     // |galat| di awal jendela pengamatan
  double yaw_best_error_;          // |galat| terkecil di dalam jendela ini

  bool yaw_align_enable_;
  double yaw_target_deg_;
  double yaw_tolerance_deg_;
  double yaw_max_turn_deg_;
  double yaw_min_turn_deg_;
  double yaw_turn_gain_;
  double yaw_timeout_sec_;
  double yaw_settle_sec_;
  int yaw_ready_count_;
  // Berapa kali putaran boleh diulang kalau cek sesudah gait reda ternyata
  // masih di luar toleransi. Yaw saat gait jalan berayun belasan derajat, jadi
  // "lurus" versi sambil berjalan tidak bisa dipercaya.
  // Pengaman putaran LIAR. Kalau galat membesar lebih dari yaw_runaway_deg_
  // dari titik acuan, putaran dihentikan paksa: apa pun sebabnya, robot yang
  // berputar makin jauh dari target tidak boleh dibiarkan terus.
  // Lama mengerem sebelum memutar. Robot harus benar-benar BERHENTI di depan
  // bola dulu; kalau langsung memutar sambil masih melangkah maju, kakinya
  // menabrak bola.
  double yaw_brake_sec_;        // lama MINIMUM mengerem
  double yaw_brake_max_sec_;    // batas atas; sesudah ini menyerah dan lanjut
  double yaw_brake_still_deg_;  // yaw dianggap diam kalau bergesernya di bawah ini
  std::chrono::steady_clock::time_point yaw_brake_until_;
  std::chrono::steady_clock::time_point yaw_brake_deadline_;
  std::chrono::steady_clock::time_point yaw_brake_sample_at_;
  double yaw_brake_last_yaw_;
  // Simpangan yaw antara DUA sampel terakhir fase mengerem. Diperbarui sejak
  // detik pertama mengerem, bukan baru sesudah lama minimumnya lewat: kalau
  // sampel pertama baru diambil di ujung, ia dibandingkan dengan yaw milik
  // robot yang masih BERJALAN, hasilnya selalu di atas ambang, dan robot
  // menunggu satu putaran sampel lagi (0,25-0,5 s) tanpa guna.
  double yaw_brake_moved_;
  // Lama fase reda yang terakhir DIPASANG, detik. Lihat yawSettleSec().
  double last_settle_sec_;
  // Menjaga bola tetap di TENGAH selama memutar. Robot yang berputar di
  // porosnya sendiri pasti melempar bola keluar dari tengah pandangan; supaya
  // tetap di tengah ia harus MENGITARI bola, yaitu bergeser menyamping sambil
  // berputar. Lihat processYawAlign().
  // ---- Putaran dikendalikan GYRO, arah gawang dibaca magnetometer saat DIAM.
  //
  // Magnetometer tidak bisa dipakai selama gait berjalan (terukur berayun 40
  // derajat per detik), tapi TENANG saat robot berhenti (1,6 derajat). Gyro
  // sebaliknya: ia mengukur laju putar, bukan arah mata angin, jadi tidak
  // terganggu medan magnet motor, tapi tidak tahu di mana gawangnya.
  //
  // Jadi masing-masing dipakai di tempat ia bisa dipercaya: magnetometer
  // menetapkan BERAPA DERAJAT harus diputar (sekali, saat diam), gyro
  // mengukur SUDAH BERPUTAR BERAPA (terus-menerus, saat berjalan).
  double body_gyro_z_;              // rad/detik, sampel terakhir
  bool gyro_seen_;
  bool gyro_integrating_;           // sedang menghitung putaran
  std::chrono::steady_clock::time_point gyro_stamp_;
  double turned_deg_;               // hasil integrasi gyro sejak putaran mulai
  double turn_target_deg_;          // dari magnetometer, dibaca sekali saat diam
  double gyro_sign_;
  // Pengali hasil integrasi gyro. Terukur di robot ini gyro UNDER-BACA: satu
  // putaran yang menurut magnetometer (dibaca saat diam, dua kali) 32,4 derajat
  // hanya tercatat 18,6 derajat oleh gyro. Lihat setNode().
  double gyro_scale_;
  double mag_before_deg_;      // yaw saat putaran mulai, untuk mengukur pengali
  double gyro_check_sec_;           // kapan arah putaran dinilai dari gyro
  std::chrono::steady_clock::time_point gyro_check_at_;
  bool gyro_dir_settled_;
  // Pengali gyro DIUKUR SENDIRI dan langsung dipakai untuk putaran berikutnya
  // dalam penyelarasan yang sama. Dua bacaan magnetometer yang sama-sama
  // diambil saat robot DIAM mengapit satu putaran, jadi rasionya sahih tanpa
  // menebak apa pun. Ini yang menghapus overshoot sistematis: tanpa koreksi,
  // gyro yang kurang baca 40 persen membuat tiap putaran melewati target dan
  // robot harus mengulang tiga sampai empat kali.
  //
  // TIDAK dipasang lewat launch: angkanya berbeda antar hari (terukur 1,74 dan
  // 0,66 pada dua hari berbeda), jadi ia harus diukur ulang tiap run.
  bool gyro_autoscale_;
  double gyro_scale_min_;
  double gyro_scale_max_;
  // ---- Penapis yaw: satu periode gait di-rata-rata sebelum dipakai memutuskan
  //
  // Badan robot berputar MAJU-MUNDUR di dalam tiap langkah (turn_in menyuruh
  // telapak berputar saat menapak), jadi baik magnetometer maupun hasil
  // integrasi gyro berayun belasan derajat dengan irama langkah. Keputusan
  // "berhenti memutar" yang diambil dari satu sampel sesaat karena itu jatuh
  // pada fase ayunan yang acak.
  //
  // Terukur di robot 2026-08-31, satu putaran, bacaan yaw tiap setengah detik:
  //   +71 +74 +75 +77 +67 +66 +78 +73 +77 +81 +73 +60 +52 +54 +67 +86 +94 +61
  // dan hasil integrasi gyro pada rentang yang sama ikut naik-turun
  //   -29 -16  +2  -1 -27 -42 -39 -38 -41 -53 -48
  // padahal integral putaran satu arah tidak mungkin mengecil.
  //
  // Caranya: DUA BLOK berturut-turut, masing-masing sepanjang satu periode
  // gait, dirata-rata sendiri-sendiri lalu diekstrapolasi setengah blok ke
  // depan (m1 + (m1 - m2)/2). Blok sepanjang tepat satu periode membuat
  // ayunannya nol dengan sendirinya -- baik yang berirama satu kali per
  // periode maupun dua kali per langkah -- jadi kedua rata-ratanya sudah
  // bersih sebelum kemiringannya dihitung, dan ekstrapolasinya menghapus
  // keterlambatan setengah blok yang selalu dibawa rata-rata bergerak.
  //
  // Digarisluruskan kuadrat terkecil atas SATU jendela sudah dicoba dan
  // dibuang: nilai di ujung jendela masih memuat sisa ayunan yang besarnya
  // bergantung pada perbandingan panjang jendela dengan periode gait, dan
  // pada jendela = periode ia justru paling buruk. Diuji 400 putaran acak:
  //   garis lurus, jendela = periode     0 % berhenti di dalam toleransi
  //   garis lurus, jendela 0,80 s       20 %
  //   dua blok,    blok = periode      100 % (sebaran 0,4 deg)
  //
  // NEGATIF = ikut periode gait yang sedang hidup (bawaan, dan itu yang benar
  // karena periodenya berubah mengikuti preset PUTAR). Positif = panjang
  // tetap. Nol = penapis mati.
  double yaw_filter_sec_;
  struct YawSample
  {
    std::chrono::steady_clock::time_point stamp;
    double yaw_deg;
    double turned_deg;
  };
  std::deque<YawSample> yaw_filter_buf_;
  std::chrono::steady_clock::time_point yaw_filter_from_;
  double yaw_strafe_gain_;      // meter per derajat simpangan arah bola
  double yaw_strafe_max_;       // batas atas geseran menyamping, meter
  double yaw_runaway_deg_;
  double yaw_runaway_ref_error_;
  int yaw_recheck_max_;
  int yaw_recheck_left_;
  double yaw_turn_sign_param_;
  bool yaw_auto_sign_;
  double yaw_sign_check_sec_;
  double yaw_sign_margin_deg_;
  // Galat awal MINIMUM sebelum koreksi tanda otomatis boleh menilai. Di bawah
  // ini derau yaw lebih besar daripada sinyalnya dan tandanya bisa terbalik
  // karena goyangan semata.
  double yaw_sign_min_error_deg_;
  double yaw_max_age_sec_;
  double yaw_stall_sec_;
  double yaw_stall_margin_deg_;
  double yaw_turn_step_deg_;
  bool refresh_foot_after_align_;

  // ===== PRESET WALKING DEMO (config/preset_walking.yaml) ==================
  //
  // Enam gerakan yang seluruhnya disetel operator di Bascorro Studio, bukan di
  // dalam kode ini:
  //
  //   jalan          JALAN MENUJU MEKKAH   selama bola TERLIHAT
  //   mencari_celah  MENCARI CELAH    berputar saat bola TIDAK terlihat
  //   putar_kiri     Putar Kiri New   mengitari bola saat yaw POSITIF
  //   putar_kanan    Putar Kanan New  mengitari bola saat yaw NEGATIF
  //   geser_kiri     KIRI MIN NEW     menyesuaikan kaki ke KIRI
  //   geser_kanan    KANAN MIN NEW    menyesuaikan kaki ke KANAN
  //
  // Presetnya disimpan sebagai LAPISAN TIPIS (nama field -> angka), bukan
  // sebagai WalkingParam utuh. Alasannya: berkas yaml itu boleh menyebut
  // sebagian field saja, dan yang tidak disebut WAJIB mewarisi nilai yang
  // sedang hidup di walking_module. Menyimpannya sebagai struct utuh berarti
  // field yang tidak ditulis operator terisi nol -- init pose nol, period nol,
  // dan robot roboh.
  //
  // Keempat preset gerakan ditimpakan DI ATAS hasil preset jalan (lewat
  // base_walking_param_), jadi field yang mereka diamkan mewarisi angka jalan,
  // bukan angka param.yaml.
  struct WalkPreset
  {
    WalkPreset() : loaded(false) { }
    std::string label;                     // untuk log: "PUTAR KIRI", ...
    std::string section;                   // nama bagian di berkas yaml
    std::map<std::string, double> fields;
    bool loaded;
  };

  bool loadWalkPresets(const std::string &path);
  // Menimpakan lapisan ke atas param dasar.
  void applyPresetOverlay(op3_walking_module_msgs::msg::WalkingParam &param,
                          const std::map<std::string, double> &fields) const;
  // Satu nilai dari lapisan, atau fallback kalau presetnya tidak menyebutnya.
  static double presetField(const WalkPreset &preset, const char *name, double fallback);
  // Baca balik parameter dari walking_module dan laporkan field preset JALAN
  // yang tidak cocok. Dipanggil sekali tiap kali demo mulai mengejar bola.
  void verifyPresetApplied();
  // Menerbitkan preset: lapisan ditimpakan ke base_walking_param_, hasilnya
  // jadi current_walking_param_ dan dikirim ke walking_module. Periode gait
  // yang dipakai penapis yaw ikut mengikuti presetnya.
  void publishPreset(const WalkPreset &preset);
  bool orbitActive() const
  {
    return (orbit_enable_ == true && orbit_presets_loaded_ == true);
  }
  // Fase MENGGESER memakai preset GESER KIRI/KANAN, bukan geseran yang
  // dihitung sebanding simpangan. Mati sendiri kalau salah satu presetnya
  // tidak ada di yaml -- jalur lama tetap utuh sebagai cadangan.
  bool shiftPresetActive() const
  {
    return (reposition_use_preset_ == true &&
            preset_shift_left_.loaded == true && preset_shift_right_.loaded == true);
  }
  // Preset mana yang dipakai, diputuskan dari TANDA YAW:
  //   yaw positif -> PUTAR KIRI, yaw negatif -> PUTAR KANAN.
  // yaw_deviation_deg = yaw sekarang - yaw target, jadi dengan target 0
  // (bawaan) angkanya persis bacaan yaw robot.
  bool chooseOrbitLeft(double yaw_deviation_deg) const;
  void publishOrbitParam(bool use_left);
  // Mengembalikan current_walking_param_ ke keadaan sebelum mengitari. TIDAK
  // menerbitkan apa pun -- pemanggil yang memutuskan kapan mengirimnya, supaya
  // urutan "nolkan amplitudo lalu stop" tetap di tangan pemanggil.
  void restoreBaseWalkingParam();

  // ---- Penapis yaw -------------------------------------------------------
  // Membuang goyangan yaw seirama langkah sebelum angkanya dipakai memutuskan.
  //
  // Isinya garis lurus kuadrat terkecil atas semua sampel di dalam jendela
  // yaw_filter_sec_ yang lebih baru daripada yaw_filter_from_, dievaluasi di
  // SEKARANG -- jadi ayunan periodiknya hilang tanpa menambah keterlambatan.
  //
  // filteredBodyYawDeg()  : arah mata angin, sudah ditangani lompatan +-180.
  // filteredTurnedDeg()   : hasil integrasi gyro sejak putaran mulai.
  //
  // Keduanya JATUH KEMBALI ke nilai sesaat kalau sampelnya belum cukup (mis.
  // tepat sesudah jendelanya dinolkan), sehingga tidak pernah ada tick tanpa
  // angka. Penapis dimatikan dengan kick_yaw_filter_sec:=0.0.
  double filteredBodyYawDeg() const;
  double filteredTurnedDeg() const;
  // Menolkan jendela: sampel yang lebih tua dari SEKARANG tidak ikut dihitung.
  // Dipanggil tiap kali artinya berubah -- mulai mengerem, mulai memutar, dan
  // tiap kali turned_deg_ dinolkan.
  void resetYawFilter();
  // Panjang satu blok, detik. Mengikuti periode gait yang sedang hidup kalau
  // yaw_filter_sec_ negatif.
  double yawFilterBlockSec() const;
  // Lama menunggu yang BENAR-BENAR dipakai untuk fase diam (mengerem, reda,
  // reda sesudah menggeser). Penapis butuh DUA blok penuh sebelum ia menjawab;
  // menunggu lebih pendek dari itu berarti angka yang menentukan segalanya --
  // vonis magnetometer saat robot diam -- justru diambil dari sampel sesaat,
  // yaitu satu-satunya tempat penapis paling dibutuhkan.
  double settleWaitSec(double requested_sec) const;
  // Inti kedua penapis. want_turned memilih deret mana yang ditaksir.
  bool estimateYawWindow(bool want_turned, double &value_at_now) const;

  // ---- fase MENGGESER ----------------------------------------------------
  int nearestKickFoot() const;
  double kickWindowCenterDeg(int foot) const;
  void startReposition();
  bool processReposition(bool ball_visible, double ball_radius_px);
  // Selalu mengembalikan false: selesainya diputuskan sesudah badan diam.
  bool finishReposition(const char *reason);
  bool abortKickAlignment(const char *reason);
  bool hasFreshBallObservation() const;
  KickPositionDecision kickPositionDecision(bool ball_visible) const;
  void enterApproachStage(const char *reason);

  // Parameter walking sebelum mengitari. Preset PUTAR menimpa SELURUH struct,
  // jadi tanpa salinan ini nilai asli param.yaml hilang untuk sisa run.
  op3_walking_module_msgs::msg::WalkingParam base_walking_param_;
  bool base_walking_param_saved_;

  // Berapa kali fase MENGGESER boleh dijalankan dalam SATU percobaan
  // tendangan. Bawaannya 1: menyesuaikan posisi kaki cukup sekali, sesudah itu
  // robot menendang atau membatalkan -- tidak menggeser berulang ulang.
  //
  // Tanpa batas ini, cabang di YawAlignPostSettle memanggil startReposition()
  // lagi setiap kali bola belum tepat "Ready" sesudah menggeser, DAN
  // startReposition() memasang ulang tenggat waktunya sendiri. Jadi tidak ada
  // yang pernah menghentikannya: robot bisa menggeser, diam, menggeser lagi
  // tanpa henti selama bolanya terlihat tapi tidak pas.
  int reposition_pass_count_;
  int reposition_max_passes_;

  // Acuan pose: isi walking_module apa adanya saat demo PERTAMA dimulai,
  // yaitu param.yaml. Direkam sekali seumur node dan tidak pernah ditimpa
  // preset. Lihat penjelasan panjang di publishPoseReference().
  op3_walking_module_msgs::msg::WalkingParam pose_reference_param_;
  bool pose_reference_saved_;
  bool pose_reference_enable_;

  WalkPreset preset_walk_;          // JALAN MENUJU MEKKAH
  WalkPreset preset_search_;        // MENCARI CELAH
  WalkPreset preset_orbit_left_;    // PUTAR KIRI
  WalkPreset preset_orbit_right_;   // PUTAR KANAN
  WalkPreset preset_shift_left_;    // GESER KIRI
  WalkPreset preset_shift_right_;   // GESER KANAN
  // true kalau KEDUA preset putar berhasil dimuat; itu syarat mengitari bola.
  bool orbit_presets_loaded_;
  bool orbit_enable_;
  // Preset jalan boleh dimatikan supaya robot memakai isi param.yaml apa
  // adanya, seperti sebelum preset ini ada.
  bool walk_preset_enable_;
  std::string preset_config_path_;

  // ---- MENCARI CELAH: gerakan saat bola TIDAK terdeteksi -----------------
  //
  // Dulu bola hilang berarti setWalkingParam(0, 0, 0): gait tetap jalan tapi
  // amplitudonya nol, jadi robot cuma berdiri melangkah di tempat sampai
  // pelacak menemukan bola lagi. Sekarang ia memakai preset "MENCARI CELAH"
  // dari Bascorro Studio -- berputar di tempat -- supaya robot ikut mengubah
  // arah pandang badannya, bukan cuma kepalanya.
  //
  // Tiga keadaan, dan hanya SATU yang boleh menerbitkan parameter tiap tick:
  //
  //   SearchOff       bola terlihat; processFollowing() yang mengemudi.
  //   SearchTurning   preset MENCARI CELAH sudah diterbitkan SEKALI dan
  //                   dibiarkan hidup. waitFollowing() tidak menerbitkan apa
  //                   pun lagi, jadi putarannya tidak terpotong tiap tick.
  //   SearchResuming  bola ketemu lagi. Amplitudo dinolkan dan ditahan selama
  //                   search_resume_delay_sec_ ("jeda sedikit"), baru preset
  //                   JALAN diterbitkan ulang dan keadaan kembali SearchOff.
  //
  // Fase mengitari/menggeser TIDAK terpengaruh: soccer_demo hanya memanggil
  // waitFollowing() saat isYawAligning() == false.
  enum SearchWalkState
  {
    SearchOff = 0,
    SearchTurning = 1,
    SearchResuming = 2,
  };

  // Aktif hanya kalau operator menghidupkannya DAN bagian 'mencari_celah'
  // benar-benar ada di preset_walking.yaml. Tanpa presetnya, bola hilang
  // kembali berarti berjalan di tempat seperti sebelum fitur ini ada.
  bool searchWalkActive() const
  {
    return (search_enable_ == true && preset_search_.loaded == true);
  }
  // Menerbitkan preset MENCARI CELAH dan masuk SearchTurning.
  void startSearchWalk();
  // Bola terlihat lagi: hentikan putaran, mulai hitung jeda.
  void enterSearchResume();
  // Jeda habis: terbitkan ulang preset JALAN, kembali ke SearchOff.
  void finishSearchResume();
  // Dipanggil startFollowing()/stopFollowing(): lupakan keadaan mencari tanpa
  // menerbitkan apa pun. Penghitung "sudah berapa lama bola hilang" ikut
  // dinolkan, jadi sesudah menendang robot selalu mendapat jatah
  // search_lost_sec_ penuh sebelum mulai berputar lagi.
  void resetSearchWalk()
  {
    search_state_ = SearchOff;
    search_lost_from_ = std::chrono::steady_clock::now();
  }

  bool search_enable_;
  // Berapa lama bola harus hilang sebelum robot mulai berputar, detik.
  // Kedipan deteksi satu-dua frame tidak boleh langsung memutar badan.
  double search_lost_sec_;
  // "Jeda sedikit" sesudah bola ketemu lagi, detik.
  double search_resume_delay_sec_;
  int search_state_;
  // steady_clock, sama alasannya dengan penghitung waktu yaw di atas:
  // mencampur clock type rclcpp::Time melempar exception.
  //
  // search_lost_from_  : kapan bola terakhir terlihat (dipasang pada tick
  //                      pertama bola hilang). Ambangnya waktu, bukan jumlah
  //                      tick, supaya tidak diam-diam berubah kalau laju loop
  //                      demo diubah.
  // search_resume_from_: kapan bola terlihat lagi, awal "jeda sedikit".
  std::chrono::steady_clock::time_point search_lost_from_;
  std::chrono::steady_clock::time_point search_resume_from_;
  // Pembalik pemetaan preset, +1 = aturan operator apa adanya:
  //   yaw positif -> PUTAR KIRI, yaw negatif -> PUTAR KANAN.
  // -1 menukar keduanya. Nilainya TIDAK pernah berubah sendiri saat demo
  // berjalan: pemetaan ini perintah operator, jadi ia harus sama persis di
  // setiap tendangan. Gyro tetap mengukur arah putaran, tapi cuma untuk
  // MELAPORKAN kalau ternyata terbalik -- yang membalik operator, lewat
  // kick_orbit_left_yaw_sign:=-1.0.
  double orbit_left_yaw_sign_;
  bool orbit_using_left_;
  // Preset yang TERAKHIR diumumkan: -1 belum ada, 0 PUTAR KANAN, 1 PUTAR KIRI.
  // Dipakai supaya pergantian preset tercetak sebagai satu baris WARN yang
  // jelas, bukan tenggelam di antara INFO yang dibatasi laju.
  int orbit_announced_;

  bool reposition_enable_;
  // Fase menggeser memakai preset GESER KIRI/KANAN (bawaan) atau geseran yang
  // dihitung sebanding simpangan (jalur lama). Dimatikan dengan
  // kick_reposition_use_preset:=0.0.
  bool reposition_use_preset_;
  // Preset yang TERAKHIR diumumkan di fase menggeser: -1 belum ada,
  // 0 GESER KANAN, 1 GESER KIRI. Sama gunanya dengan orbit_announced_.
  int shift_announced_;
  double reposition_y_gain_;       // meter geser per derajat simpangan sudut leher
  double reposition_y_max_;        // batas atas geseran menyamping, meter
  double reposition_x_max_;        // batas atas langkah maju saat menggeser, meter
  double reposition_x_ramp_;       // keadaan ramp maju, meter
  double reposition_timeout_sec_;
  int reposition_ready_count_;     // siklus berturut syarat tendang sudah benar
  int reposition_recheck_max_;
  int reposition_recheck_left_;
  // Kaki penendang DIKUNCI sekali di awal fase menggeser, bukan dihitung ulang
  // tiap tick.
  //
  // nearestKickFoot() memilih jendela yang pusatnya paling dekat, jadi bola
  // yang kebetulan ada di antara kedua jendela membuat pilihannya berbalik
  // hanya karena kepala bergeser sepersekian derajat -- dan sasaran geseran
  // ikut melompat 35 derajat, jadi robot menggeser ke kiri lalu ke kanan lalu
  // ke kiri lagi. Itu yang terlihat sebagai "menyesuaikan kaki berkali-kali".
  //
  // Sasaran boleh berpindah PALING BANYAK SEKALI, dan hanya kalau jendela
  // seberang benar-benar lebih dekat sejauh reposition_foot_hysteresis_deg_ --
  // itu histeresisnya.
  int reposition_target_foot_;
  double reposition_foot_hysteresis_deg_;
  bool reposition_foot_switched_;
  // true = kaki penendang sudah ditetapkan oleh fase menggeser (atau oleh
  // pemeriksaan "sudah di depan kaki" yang melewatinya). Di situ ia TIDAK boleh
  // dihitung ulang lagi sesudahnya: fase menggeser justru bekerja membawa bola
  // ke depan kaki itu, dan menghitung ulang dari sudut leher beberapa detik
  // kemudian bisa membatalkan hasil kerjanya sendiri.
  bool foot_from_reposition_;
  std::chrono::steady_clock::time_point reposition_deadline_;
  // Tanda geseran menyamping: +1 = bola di KIRI jendela dijawab dengan bergeser
  // ke kiri (preset GESER KIRI di jalur preset, y_move_amplitude positif di
  // jalur lama). Itu konvensi yang dipakai di seluruh berkas ini, tapi TIDAK
  // PERNAH diukur langsung -- dan tanda yang ditebak sudah dua kali salah di
  // robot ini. Jadi ia dinilai sendiri dari sudut leher: kalau sesudah beberapa
  // detik menggeser simpangan bola justru MEMBESAR, tandanya dibalik sekali
  // lalu dipakai untuk sisa run. Di jalur preset yang tertukar adalah PILIHAN
  // PRESETNYA, bukan tanda sebuah amplitudo.
  double reposition_y_sign_;
  bool reposition_auto_sign_;
  bool reposition_sign_settled_;
  double reposition_sign_check_sec_;
  double reposition_sign_margin_deg_;
  double reposition_sign_min_error_deg_;
  // < 0 berarti jendela penilaian belum dipasang (simpangan masih terlalu
  // kecil untuk dinilai).
  double reposition_sign_ref_deg_;
  std::chrono::steady_clock::time_point reposition_sign_check_at_;

  // Simpangan di bawah angka ini dianggap SUDAH di depan kaki. Tanpa pita mati
  // robot menggeser terus mengejar derau beberapa derajat.
  // Ambang BERHENTI memutar, dibaca dari gyro. Sengaja lebih ketat daripada
  // yaw_tolerance_deg_ yang dipakai memverifikasi hasil dengan magnetometer;
  // lihat catatan panjang di processYawAlign().
  double yaw_turn_stop_deg_;

  double reposition_deadband_deg_;
  // Berapa lama boleh kehilangan umpan balik (bola tidak terlihat, atau sudut
  // leher tidak bergerak sama sekali) sebelum fase ini menyerah dan menendang.
  // Tanpa batas ini robot berdiri bergoyang sampai batas waktu penuh habis.
  double reposition_blind_sec_;
  bool reposition_blind_;
  std::chrono::steady_clock::time_point reposition_blind_since_;
  // Deteksi umpan balik BASI. Sudut leher hanya berarti "arah bola" kalau
  // pelacak kepala memang sedang mengejar bola; kalau kepalanya diam, angkanya
  // beku dan menggeser mengikutinya berarti berjalan berdasarkan bacaan lama.
  double reposition_stale_sec_;
  double reposition_pan_last_deg_;
  std::chrono::steady_clock::time_point reposition_pan_moved_at_;

  // ===== Fase MENYESUAIKAN dibagi dua: MAJU dulu, baru MENGGESER ==========
  //
  // Mengitari bola memindahkan SELURUH badan mengelilinginya, jadi sesudah yaw
  // lurus robot sering berdiri lebih jauh dari bola daripada sebelum memutar.
  // Dulu fase ini langsung menggeser menyamping dari jarak berapa pun, dan
  // preset GESER memang tidak punya langkah maju yang berarti (x -2,6 dan
  // 0,0 mm) -- jadi robot menggeser menyamping di depan bola yang masih jauh,
  // sampai batas waktunya habis.
  //
  // Sekarang:
  //   MAJU     -- preset JALAN biasa, lurus ke bola, sampai jaraknya <=
  //               kick_distance_ (ambang berhenti yang sama dengan fase
  //               mengejar). Sudut putar DIKUNCI NOL supaya yaw yang baru
  //               diluruskan tidak rusak lagi.
  //   MENGGESER -- persis seperti sebelumnya, preset GESER KIRI/KANAN.
  //
  // Di antara keduanya ada jeda transformasi (lihat transition_delay_sec_).
  enum RepositionStage
  {
    RepoApproach = 0,   // MAJU
    RepoShift = 1,      // MENGGESER
  };
  int reposition_stage_;
  bool reposition_approach_enable_;
  // Berapa tick berturut-turut jaraknya harus sudah cukup dekat sebelum pindah
  // ke MENGGESER. Dipisah dari reposition_ready_count_, yang menghitung syarat
  // "bola sudah di depan kaki" -- dua pertanyaan yang berbeda.
  int approach_ready_count_;
  // delta_time untuk calcFootstep() di fase MAJU. processReposition() tidak
  // punya jam sendiri seperti processFollowing(), jadi dicatat di sini.
  std::chrono::steady_clock::time_point reposition_prev_tick_;

  // Jarak bola dihitung dari SUDUT SENDI KEPALA saja, dengan bola dianggap
  // tepat di tengah gambar. Itu sahih di fase ini karena pelacak kepala memang
  // sedang mengunci bola; kalau bolanya tidak terlihat, fase MAJU tidak
  // dijalankan sama sekali. Rumusnya sama persis dengan processFollowing()
  // (ballDirection() + tinggi efektif), jadi ambang kick_distance_ berarti hal
  // yang sama di kedua tempat.
  double ballDistanceFromObservation(bool filtered = true) const;
  // Satu tick fase MAJU. Selalu mengembalikan false: fase menyesuaikan baru
  // selesai di tahap MENGGESER.
  bool processRepositionApproach(bool ball_visible,
                                 const std::chrono::steady_clock::time_point &now);
  // Pindah MAJU -> MENGGESER: kembalikan parameter dasar, nolkan keadaan ramp,
  // dan pasang jeda transformasinya.
  void enterShiftStage(const char *reason);

  // ---- Jeda transformasi gerakan ----------------------------------------
  //
  // Jeda pendek tiap kali robot berpindah dari satu gerakan ke gerakan lain.
  // Selama jeda gait tetap HIDUP tapi amplitudonya nol, jadi robot melangkah
  // di tempat: badan sempat berhenti berayun dan kepala sempat mengunci bola
  // sebelum gerakan berikutnya mengambil alih.
  //
  // Dipakai di peralihan yang SEBELUMNYA tidak punya jeda sama sekali:
  //   MEMUTAR   -> MAJU
  //   MAJU      -> MENGGESER
  //
  // Peralihan lain sudah punya fase diamnya sendiri dan TIDAK ditambah lagi:
  //   MENCARI CELAH -> JALAN   search_resume_delay_sec_ (0,5 s)
  //   JALAN -> MEMUTAR         fase MENGEREM, yaw_brake_sec_ (0,8 s) -- tidak
  //                            boleh dipendekkan ke 0,5: penapis yaw butuh DUA
  //                            blok gait penuh, lihat settleWaitSec()
  //   MENGGESER -> TENDANG     yaw_settle_sec_ lalu kick_pre_delay_sec (2,0 s)
  double transition_delay_sec_;
  std::chrono::steady_clock::time_point transition_until_;
  // Memasang jeda dan mencetak satu baris "dari -> ke".
  void beginTransitionDelay(const char *from, const char *to);
  // true selama jeda masih berjalan. Menerbitkan amplitudo nol sekali di tick
  // pertama saja; sesudah itu walking_module sudah memegangnya.
  bool transitionDelayActive();

  // ---- Menera parameter tendangan selagi robot hidup ---------------------
  //
  // Membaca ULANG seluruh parameter kick_* / walk_* dari penyimpanan parameter
  // node, lalu menghitung ulang semua angka turunannya. Dipanggil sekali dari
  // setNode(), dan sesudah itu tiap kali ada yang menulis lewat
  // /demo_node/set_parameters -- jalur yang dipakai halaman Tendangan di
  // Bascorro Studio. Tanpa ini, menera jarak tendangan selalu berarti
  // meluncurkan ulang seluruh demo.
  //
  // Aman dipanggil berkali-kali; yang sengaja ikut disetel ulang cuma angka
  // yang memang sifat SETELAN, bukan hasil pengukuran satu tendangan.
  void loadTunableParameters();

  // Bendera dari add_on_set_parameters_callback, dibaca pencacah di bawah.
  // Perlu ditunda karena callback itu berjalan SEBELUM nilai barunya disimpan:
  // membaca ulang di dalamnya cuma mengembalikan angka yang lama.
  bool param_reload_pending_;
  // 200 ms: cukup cepat supaya tombol "Kirim ke robot" di Studio terasa
  // langsung, cukup jarang supaya tidak menambah beban loop demo.
  static constexpr int PARAM_RELOAD_POLL_MS = 200;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::TimerBase::SharedPtr param_reload_timer_;
};
}

#endif /* BALL_FOLLOWER_H_ */

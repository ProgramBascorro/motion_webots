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

#include "op3_demo/ball_follower.h"

namespace robotis_op
{

BallFollower::BallFollower()
  : // Node("ball_follower"),
    FOV_WIDTH(35.2 * M_PI / 180),
    FOV_HEIGHT(21.6 * M_PI / 180),
    count_not_found_(0),
    count_to_kick_(0),
    on_tracking_(false),
    approach_ball_position_(NotFound),
    kick_motion_index_(40),
    camera_height_(0.56),
    // Ambang ini bukan cuma pemicu tendangan -- ia juga JARAK BERHENTI.
    // calcFootstep() diberi (jarak_bola - kick_distance), jadi robot berhenti
    // mendekat begitu jarak bola sama dengan angka ini. Dipasang 0,54 m robot
    // berhenti satu setengah langkah dari bola: bola tidak pernah sampai di
    // depan kaki, jendela servo head_pan tidak pernah tercapai, dan tendangan
    // tidak pernah terjadi.
    //
    // 0,54 m yang diukur operator itu jarak LURUS kamera-ke-bola. Angka yang
    // dipakai di sini jarak TANAH. Dengan kamera ~0,50 m di atas lantai,
    // sqrt(0,54^2 - 0,50^2) = 0,20 m -- itu padanannya.
    kick_distance_(0.20),
    head_tilt_sign_(-1.0),   // ALPHONSE: head_tilt POSITIF = menunduk
    kick_pan_right_min_deg_(156.0),
    kick_pan_right_max_deg_(158.0),
    kick_pan_left_min_deg_(182.0),
    kick_pan_left_max_deg_(184.0),
    head_pan_servo_center_deg_(180.0),
    head_pan_servo_dir_(1.0),
    kick_pan_max_distance_(0.30),
    kick_ready_count_(10),
    NOT_FOUND_THRESHOLD(50),
    MAX_FB_STEP(12.0 * 0.001),
    MAX_RL_TURN(15.0 * M_PI / 180),
    IN_PLACE_FB_STEP(-3.0 * 0.001),
    MIN_FB_STEP(5.0 * 0.001),
    MIN_RL_TURN(5.0 * M_PI / 180),
    UNIT_FB_STEP(0.3 * 0.001),
    UNIT_RL_TURN(0.5 * M_PI / 180),
    SPOT_FB_OFFSET(0.0 * 0.001),
    SPOT_RL_OFFSET(0.0 * 0.001),
    SPOT_ANGLE_OFFSET(0.0),
    hip_pitch_offset_(7.0),
    current_pan_(-10),
    current_tilt_(-10),
    current_x_move_(0.003),
    current_r_angle_(0),
    curr_period_time_(0.8),
    accum_period_time_(0.0),
    DEBUG_PRINT(false)
{
  // current_joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //     "/robotis/goal_joint_states", 10, std::bind(&BallFollower::currentJointStatesCallback, this, std::placeholders::_1));

  // set_walking_command_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
  // set_walking_param_pub_ = this->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  // get_walking_param_client_ = this->create_client<op3_walking_module_msgs::srv::GetWalkingParam>("/robotis/walking/get_params");

  prev_time_ = rclcpp::Clock().now();
}

BallFollower::~BallFollower()
{

}

void BallFollower::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  if (node_ != nullptr)
  {
    // Geometri pemicu tendangan, bisa ditera di robot tanpa compile ulang.
    // Dijaga has_parameter(): declare_parameter yang kedua kali pada nama yang
    // sama melempar exception, dan node ini dipakai bersama demo lain.
    auto read_param = [this](const std::string &name, double fallback) -> double
    {
      if (node_->has_parameter(name) == false)
        return node_->declare_parameter(name, fallback);
      double value = fallback;
      node_->get_parameter(name, value);
      return value;
    };
    camera_height_  = read_param("kick_camera_height", camera_height_);
    kick_distance_  = read_param("kick_distance", kick_distance_);
    head_tilt_sign_ = read_param("kick_head_tilt_sign", head_tilt_sign_);

    kick_pan_right_min_deg_ = read_param("kick_pan_right_min_deg", kick_pan_right_min_deg_);
    kick_pan_right_max_deg_ = read_param("kick_pan_right_max_deg", kick_pan_right_max_deg_);
    kick_pan_left_min_deg_  = read_param("kick_pan_left_min_deg",  kick_pan_left_min_deg_);
    kick_pan_left_max_deg_  = read_param("kick_pan_left_max_deg",  kick_pan_left_max_deg_);
    head_pan_servo_center_deg_ = read_param("head_pan_servo_center_deg", head_pan_servo_center_deg_);
    head_pan_servo_dir_        = read_param("head_pan_servo_dir", head_pan_servo_dir_);
    kick_pan_max_distance_     = read_param("kick_pan_max_distance", kick_pan_max_distance_);
    kick_ready_count_ = (int) read_param("kick_ready_count", (double) kick_ready_count_);

    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Kick geometry: camera_height=%.3f m, kick_distance=%.3f m, head_tilt_sign=%+.0f",
                camera_height_, kick_distance_, head_tilt_sign_);
    RCLCPP_WARN(rclcpp::get_logger("BallFollower"),
                "Kick pan window: kanan %.1f..%.1f deg, kiri %.1f..%.1f deg "
                "(servo head_pan, pusat %.1f, arah %+.0f), maks %.2f m, butuh %d siklus",
                kick_pan_right_min_deg_, kick_pan_right_max_deg_,
                kick_pan_left_min_deg_, kick_pan_left_max_deg_,
                head_pan_servo_center_deg_, head_pan_servo_dir_,
                kick_pan_max_distance_, kick_ready_count_);

    current_joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/robotis/goal_joint_states", 10, std::bind(&BallFollower::currentJointStatesCallback, this, std::placeholders::_1));

    // Persistent publishers to walking_module — created ONCE here so DDS
    // discovery is done well before any button press and no walking "start"/
    // "stop" is ever silently dropped (see the header note).
    set_walking_command_pub_ = node_->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
    set_walking_param_pub_ = node_->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set");
  }
}

void BallFollower::startFollowing()
{
  on_tracking_ = true;
  RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Start Ball following");

  // setWalkingCommand("start") already calls getWalkingParam() internally and
  // populates current_walking_param_ before publishing the "start" command.
  // We READ THE CACHED VALUES instead of calling getWalkingParam() a second
  // time — the original code did a back-to-back service call that added
  // ~service-roundtrip + up-to-1s wait_for_service to the START button
  // latency. period_time and hip_pitch_offset aren't touched by
  // setWalkingParam(), so the cached values are valid.
  setWalkingCommand("start");

  if (current_walking_param_.period_time > 0.1)
  {
    hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
    curr_period_time_ = current_walking_param_.period_time;
  }
  else
  {
    // Service unavailable or returned garbage — fall back to known defaults.
    hip_pitch_offset_ = 7.0 * M_PI / 180;
    curr_period_time_ = 0.6;
  }
}

void BallFollower::stopFollowing()
{
  on_tracking_ = false;
  count_to_kick_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Stop Ball following");

  // Zero the gait amplitudes BEFORE sending "stop" so walking_module's
  // next cycle decelerates from amplitude → 0 instead of completing the
  // last commanded forward/turn step. Visible effect: feet stop swinging
  // forward almost immediately while body finishes settling — vs the
  // old path where walking_module finished its full step (~750 ms)
  // before stopping, making the STOP button feel laggy.
  setWalkingParam(0.0, 0.0, 0.0);
  setWalkingCommand("stop");
}

void BallFollower::currentJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  double pan, tilt;
  int get_count = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_pan")
    {
      pan = msg->position[ix];
      get_count += 1;
    }
    else if (msg->name[ix] == "head_tilt")
    {
      tilt = msg->position[ix];
      get_count += 1;
    }

    if (get_count == 2)
      break;
  }

  // check variation
  current_pan_ = pan;
  current_tilt_ = tilt;
}

void BallFollower::calcFootstep(double target_distance, double target_angle, double delta_time,
                                double& fb_move, double& rl_angle)
{
  // clac fb
  double next_movement = current_x_move_;
  if (target_distance < 0)
    target_distance = 0.0;

  double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
  accum_period_time_ += delta_time;
  if (accum_period_time_ > (curr_period_time_  / 4))
  {
    accum_period_time_ = 0.0;
    if ((target_distance * 0.1 / 2) < current_x_move_)
      next_movement -= UNIT_FB_STEP;
    else
      next_movement += UNIT_FB_STEP;
  }
  fb_goal = fmin(next_movement, fb_goal);
  fb_move = fmax(fb_goal, MIN_FB_STEP);
  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", target_distance, fb_move, delta_time);
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "==============================================");
  }

  // calc rl angle
  double rl_goal = 0.0;
  if (fabs(target_angle) * 180 / M_PI > 5.0)
  {
    double rl_offset = fabs(target_angle) * 0.2;
    rl_goal = fmin(rl_offset, MAX_RL_TURN);
    rl_goal = fmax(rl_goal, MIN_RL_TURN);
    rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

    if (target_angle < 0)
      rl_angle *= (-1);
  }
}

// x_angle : ball position (pan), y_angle : ball position (tilt), ball_size : angle of ball radius
double BallFollower::headPanServoDeg() const
{
  return head_pan_servo_center_deg_ + head_pan_servo_dir_ * current_pan_ * 180.0 / M_PI;
}

int BallFollower::kickFootFromHeadPan() const
{
  const double servo_deg = headPanServoDeg();

  if (servo_deg >= kick_pan_right_min_deg_ && servo_deg <= kick_pan_right_max_deg_)
    return OnRight;
  if (servo_deg >= kick_pan_left_min_deg_ && servo_deg <= kick_pan_left_max_deg_)
    return OnLeft;

  return OutOfRange;
}

bool BallFollower::processFollowing(double x_angle, double y_angle, double ball_size)
{
  rclcpp::Time curr_time = rclcpp::Clock().now();
  rclcpp::Duration dur = curr_time - prev_time_;
  double delta_time = dur.seconds();
  prev_time_ = curr_time;

  count_not_found_ = 0;
//  int ball_position_sum = 0;

  // check of getting head joints angle
  if (current_tilt_ == -10 && current_pan_ == -10)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Failed to get current angle of head joints.");
    setWalkingCommand("stop");

    on_tracking_ = false;
    approach_ball_position_ = NotFound;
    return false;
  }

  if (DEBUG_PRINT)
  {
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "   ============== Head | Ball ==============   ");
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Pan : %f | Ball X : %f", (current_pan_ * 180 / M_PI), (x_angle * 180 / M_PI));
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "== Head Tilt : %f | Ball Y : %f", (current_tilt_ * 180 / M_PI), (y_angle * 180 / M_PI));
  }

  approach_ball_position_ = OutOfRange;

  // Rumus aslinya menganggap head_tilt NEGATIF saat menunduk (konvensi OP3
  // asli): dengan hip_pitch_offset 0 ia menjadi H*cot(sudut tunduk) = jarak
  // bola yang sebenarnya, persis.
  //
  // ALPHONSE TERBALIK -- head_tilt POSITIF berarti menunduk. Tercatat di
  // op3_ball_localization/config/head_tracking.yaml ("POSITIVE = look DOWN")
  // dan dibuktikan lagi 2026-08-25: nilai scan_tilt_* diturunkan tiga kali
  // (0,25 -> 0,12 -> 0,05 -> -0,12) dan kepala memang makin MENDONGAK.
  //
  // Tanpa koreksi tanda, syaratnya menjadi |head_tilt - 8,0| > 67,7 deg, yaitu
  // head_tilt > +75,7 deg ATAU < -59,7 deg. Sapuan robot ini cuma bergerak di
  // -6,9 .. +5,2 deg, jadi in_range TIDAK PERNAH benar dan handleKick() tidak
  // pernah dipanggil -- persis gejala "didekatkan ke kaki tapi tidak menendang".
  // Rumusnya juga melebih-lebihkan jarak: pada head_tilt +59,7 deg ia melaporkan
  // 0,443 m padahal geometrinya 0,230 m -- hampir dua kali lipat.
  //
  // head_tilt_sign_ = -1 mengembalikan sudutnya ke konvensi yang diasumsikan
  // rumus. Setel kick_head_tilt_sign := 1.0 untuk robot yang masih konvensi
  // asli.
  //
  // + y_angle: sudut bola DI DALAM gambar, bukan cuma arah kepala. Rumus asli
  // hanya memakai head_tilt, jadi ia mengukur "seberapa jauh titik yang
  // DIPANDANG kepala", bukan "seberapa jauh BOLA". Selama kepala mengunci bola
  // di tengah frame keduanya sama, tapi justru saat bola mendekat kaki keduanya
  // berbeda jauh: bola turun ke tepi bawah gambar sementara kepala masih
  // menyusul (atau sudah mentok di batas sendi). Setengah-FOV vertikal 21,6 deg,
  // jadi selisih itu bisa 20 derajat lebih -- persis rentang yang menentukan
  // menendang atau tidak. y_angle sudah dalam konvensi rumus (negatif = bola di
  // bawah sumbu kamera), makanya ditambahkan apa adanya.
  const double tilt_for_geometry = head_tilt_sign_ * current_tilt_ + y_angle;
  double distance_to_ball = camera_height_ * tan(M_PI * 0.5 + tilt_for_geometry - hip_pitch_offset_ - ball_size);

  double ball_y_angle = (current_tilt_ + y_angle) * 180 / M_PI;
  double ball_x_angle = (current_pan_ + x_angle) * 180 / M_PI;

  if (distance_to_ball < 0)
    distance_to_ball *= (-1);

  double distance_to_kick = kick_distance_;

  // DUA pemicu, berdiri sendiri-sendiri:
  //
  //  (1) Jendela servo head_pan. Kepala sedang mengunci bola (fungsi ini hanya
  //      dipanggil saat tracking_status_ == Found), dan sudut lehernya sendiri
  //      yang memberi tahu bola ada di depan kaki mana. Sisi bola dicek ulang
  //      lewat ball_x_angle supaya jendela tidak kebetulan cocok saat kepala
  //      lewat begitu saja. Batas +-25 derajat SENGAJA tidak dipakai di jalur
  //      ini: kepala yang menoleh 23 derajat sudah hampir menyentuhnya, dan itu
  //      justru posisi tendang yang dimaksud.
  //
  //  (2) Jarak. Perilaku lama, tetap ada.
  const double head_pan_servo_deg = headPanServoDeg();
  const int foot_from_pan = kickFootFromHeadPan();
  const bool pan_side_ok = (foot_from_pan == OnRight && ball_x_angle < 0.0)
                        || (foot_from_pan == OnLeft && ball_x_angle > 0.0);
  const bool pan_ready = pan_side_ok && (distance_to_ball < kick_pan_max_distance_);
  const bool near_enough = (distance_to_ball < distance_to_kick) && (fabs(ball_x_angle) < 25.0);

  // Selalu dicetak (1x/detik) supaya ambangnya bisa ditera langsung di robot:
  // dekatkan bola sampai jaraknya turun di bawah ambang, atau sampai servo
  // head_pan masuk salah satu jendela.
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("BallFollower"),
                       *rclcpp::Clock::make_shared(), 1000,
                       "jarak bola %.3f m (ambang %.3f) | servo head_pan %.1f deg "
                       "(kanan %.0f-%.0f, kiri %.0f-%.0f) | head_tilt %+.1f deg "
                       "| bola dlm gambar %+.1f deg | tunduk total %+.1f deg | bola x %+.1f deg | siap: %s",
                       distance_to_ball, distance_to_kick, head_pan_servo_deg,
                       kick_pan_right_min_deg_, kick_pan_right_max_deg_,
                       kick_pan_left_min_deg_, kick_pan_left_max_deg_,
                       current_tilt_ * 180 / M_PI, y_angle * 180 / M_PI,
                       -tilt_for_geometry * 180 / M_PI, ball_x_angle,
                       pan_ready ? "jendela pan"
                                 : (near_enough ? "jarak"
                                                : (pan_side_ok ? "jendela pan cocok tapi bola masih jauh"
                                                               : "belum")));

  // check whether ball is correct position.
  if (pan_ready || near_enough)
  {
    count_to_kick_ += 1;

    if (DEBUG_PRINT)
    {
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "head pan : %f | ball pan : %f", (current_pan_ * 180 / M_PI), (x_angle * 180 / M_PI));
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "head tilt : %f | ball tilt : %f", (current_tilt_ * 180 / M_PI), (y_angle * 180 / M_PI));
      RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "foot to kick : %f", ball_x_angle);
    }

    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "In range [%d | %f]", count_to_kick_, ball_x_angle);

    // ball queue
//    if(ball_position_queue_.size() >= 5)
//      ball_position_queue_.erase(ball_position_queue_.begin());

//    ball_position_queue_.push_back((ball_x_angle > 0) ? 1 : -1);


    if (count_to_kick_ > kick_ready_count_)
    {
      setWalkingCommand("stop");
      on_tracking_ = false;

      // Jendela servo yang menentukan kakinya kalau dialah yang memicu: itu
      // pengukuran langsung "bola ada di depan kaki mana", bukan tebakan dari
      // tanda sudut. Kalau yang memicu jaraknya, pakai aturan lama.
      if (pan_ready)
        approach_ball_position_ = foot_from_pan;
      else if (ball_x_angle > 0.02)
        approach_ball_position_ = OnLeft;
      else
        approach_ball_position_ = OnRight;

      RCLCPP_INFO(rclcpp::get_logger("BallFollower"),
                  "TENDANG kaki %s -- pemicu: %s | servo head_pan %.1f deg | jarak %.3f m",
                  (approach_ball_position_ == OnLeft) ? "KIRI" : "KANAN",
                  pan_ready ? "jendela pan" : "jarak",
                  head_pan_servo_deg, distance_to_ball);

      return true;
    }
    else if (count_to_kick_ > kick_ready_count_ / 2)
    {
      //      if (ball_x_angle > 0)
      //        accum_ball_position_ += 1;
      //      else
      //        accum_ball_position_ -= 1;

      // send message
      setWalkingParam(IN_PLACE_FB_STEP, 0, 0);
      return false;
    }
  }
  else
  {
    count_to_kick_ = 0;
//    accum_ball_position_ = NotFound;
  }

  double fb_move = 0.0, rl_angle = 0.0;
  double distance_to_walk = distance_to_ball - distance_to_kick;

  calcFootstep(distance_to_walk, current_pan_, delta_time, fb_move, rl_angle);

  // send message
  setWalkingParam(fb_move, 0, rl_angle);

  // for debug
  //RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", distance_to_ball, fb_move, delta_time);

  return false;
}

void BallFollower::decideBallPositin(double x_angle, double y_angle)
{
  // check of getting head joints angle
  if (current_tilt_ == -10 && current_pan_ == -10)
  {
    approach_ball_position_ = NotFound;
    return;
  }

  // Jendela servo head_pan lebih dipercaya daripada tanda sudut: ambangnya
  // 0,05 rad (2,9 derajat) dan jendela KIRI operator (servo 182-184 = +2..+4
  // derajat) duduk tepat di atasnya -- sedikit saja meleset, kakinya tertukar.
  const int foot_from_pan = kickFootFromHeadPan();
  if (foot_from_pan == OnRight || foot_from_pan == OnLeft)
  {
    approach_ball_position_ = foot_from_pan;
    return;
  }

  double ball_x_angle = current_pan_ + x_angle;

  if (ball_x_angle > 0.05)
    approach_ball_position_ = OnLeft;
  else
    approach_ball_position_ = OnRight;
}

void BallFollower::waitFollowing()
{
  count_not_found_++;

  if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
    setWalkingParam(0.0, 0.0, 0.0);
}

void BallFollower::setWalkingCommand(const std::string &command)
{
  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set, cannot set walking command");
    return;
  }

  // get param
  if (command == "start")
  {
    getWalkingParam();
    setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
  }

  std_msgs::msg::String _command_msg;
  _command_msg.data = command;
  set_walking_command_pub_->publish(_command_msg);

  if (DEBUG_PRINT)
    RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Send Walking command : %s", command.c_str());
}

void BallFollower::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
  current_walking_param_.balance_enable = balance;
  current_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET;
  current_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET;
  current_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

  if (node_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Node is not set, cannot set walking parameters");
    return;
  }
  set_walking_param_pub_->publish(current_walking_param_);

  current_x_move_ = x_move;
  current_r_angle_ = rotation_angle;
}

bool BallFollower::getWalkingParam()
{
  auto temp_node = rclcpp::Node::make_shared("ballfollower_get_walking_param");
  auto get_walking_param_client_ = temp_node->create_client<op3_walking_module_msgs::srv::GetWalkingParam>("/robotis/walking/get_params");
  auto request = std::make_shared<op3_walking_module_msgs::srv::GetWalkingParam::Request>();

  // 200 ms timeout (was 1 s). walking_module's service is already advertised
  // by the time op3_manager is up; 1 s was just dead latency added to every
  // START press. If service legitimately isn't available, the fallback path
  // in startFollowing() kicks in with sensible defaults.
  if (!get_walking_param_client_->wait_for_service(std::chrono::milliseconds(200)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "BallFollower::getWalkingParam - Service not available");
    return false;
  }

  auto future = get_walking_param_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(temp_node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    if (result)
    {
      current_walking_param_ = result->parameters;

      if (DEBUG_PRINT)
        RCLCPP_INFO(rclcpp::get_logger("BallFollower"), "Get walking parameters");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("BallFollower"), "Fail to get walking parameters.");
    }
  }

  return true;
}

}

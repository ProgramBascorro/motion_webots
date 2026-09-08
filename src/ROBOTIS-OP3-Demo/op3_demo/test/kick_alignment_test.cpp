#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>

#include "op3_demo/ball_follower.h"

namespace robotis_op
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

class FollowerHarness : public BallFollower
{
public:
  using BallFollower::kickPositionDecision;

  explicit FollowerHarness(const rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    set_walking_command_pub_ = node->create_publisher<std_msgs::msg::String>(
        "/op3_demo_alignment_test/walking/command", 10);
    set_walking_param_pub_ = node->create_publisher<op3_walking_module_msgs::msg::WalkingParam>(
        "/op3_demo_alignment_test/walking/set_params", 10);
    loadWalkPresets(OP3_DEMO_PRESET_PATH);
    applyPresetOverlay(current_walking_param_, preset_walk_.fields);
    base_walking_param_ = current_walking_param_;
    base_walking_param_saved_ = true;
    hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
    curr_period_time_ = current_walking_param_.period_time;
    max_fb_step_ = current_walking_param_.x_move_amplitude;
    kick_pan_right_min_deg_ = 152.0;
    kick_pan_right_max_deg_ = 178.0;
    kick_pan_left_min_deg_ = 193.0;
    kick_pan_left_max_deg_ = 208.0;
    yaw_tolerance_deg_ = 15.0;
    yaw_filter_sec_ = 0.0;
    kick_ready_count_ = 4;
    setMagStatus(2);
    setBodyYaw(0.0);
    observe(0.25, 165.0);
  }

  void observe(double distance, double servo_deg)
  {
    auto joints = std::make_shared<sensor_msgs::msg::JointState>();
    joints->name = {"head_pan", "head_tilt"};
    joints->position = {(servo_deg - 180.0) * kPi / 180.0,
        hip_pitch_offset_ + camera_pitch_offset_deg_ * kPi / 180.0 -
        std::atan2(camera_height_ - kick_ball_real_radius_m_, distance)};
    currentJointStatesCallback(joints);
    resetDistanceFilter();
    updateBallObservation(0.0, 0.0, 80.0);
  }

  void calibrateCamera(double pitch_offset = 30.84)
  {
    camera_height_ = 0.52;
    kick_ball_real_radius_m_ = 0.067;
    camera_pitch_offset_deg_ = pitch_offset;
    kick_distance_ = 0.15;
    kick_pan_max_distance_ = 0.20;
    resetDistanceFilter();
  }

  void observeRecordedRightBall()
  {
    hip_pitch_offset_ = 0.1395999938249588;
    auto joints = std::make_shared<sensor_msgs::msg::JointState>();
    joints->name = {"head_pan", "head_tilt"};
    joints->position = {-0.01227184630308513, -0.521553467881118};
    currentJointStatesCallback(joints);
    updateBallObservation(-std::atan(0.1358221054077149 * std::tan(FOV_WIDTH)),
                          -std::atan(0.190411885579427 * std::tan(FOV_HEIGHT)),
                          81.51219177246094);
  }

  double servoWindow() const { return headPanServoDeg(); }

  void expireBrake()
  {
    yaw_brake_until_ = {};
    yaw_brake_deadline_ = {};
  }

  void expireSettle() { yaw_settle_until_ = {}; }
  void expireTransition() { transition_until_ = {}; }
  void expireReposition() { reposition_deadline_ = {}; }
  void expireBall() { ball_observation_stamp_ = {}; }
  void expireHead() { head_state_stamp_ = {}; }
  void expireYaw() { body_yaw_stamp_ = {}; }
  void expireBlind() { reposition_blind_since_ = {}; }
  void disableYaw() { yaw_align_enable_ = false; }
  void disableReposition() { reposition_enable_ = false; }
  void disableApproach() { reposition_approach_enable_ = false; }
  void beginReposition() { startReposition(); }
  bool approaching() const { return reposition_stage_ == RepoApproach; }
  bool shifting() const { return reposition_stage_ == RepoShift; }
  bool postSettling() const { return yaw_align_state_ == YawAlignPostSettle; }
  bool settling() const { return yaw_align_state_ == YawAlignSettling; }
  double distance() const { return ballDistanceFromObservation(); }
  bool orbitLeft(double yaw) const { return chooseOrbitLeft(yaw); }
  const op3_walking_module_msgs::msg::WalkingParam &walking() const
  {
    return current_walking_param_;
  }

  // Merekam acuan pose seperti startFollowing() pada demo PERTAMA: isi
  // param.yaml, sebelum satu pun preset diterbitkan.
  void recordPoseReference(const op3_walking_module_msgs::msg::WalkingParam &param)
  {
    pose_reference_param_ = param;
    pose_reference_saved_ = true;
  }
  void publishOrbitLeft() { publishOrbitParam(true); }
  void setMaxPasses(int passes) { reposition_max_passes_ = passes; }

  void residualTurn()
  {
    turned_deg_ = 100.0;
    mag_before_deg_ = -100.0;
  }
  double turned() const { return turned_deg_; }
};

class KickAlignmentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("kick_alignment_test");
    follower_ = std::make_unique<FollowerHarness>(node_);
  }
  void finishInitialYawCheck()
  {
    ASSERT_TRUE(follower_->startYawAlign());
    follower_->expireBrake();
    ASSERT_FALSE(follower_->processYawAlign(true, 80.0));
    ASSERT_TRUE(follower_->settling());
    follower_->expireSettle();
  }
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FollowerHarness> follower_;
};

TEST(KickAlignmentPolicy, DistanceMustBeValidBeforeFootWindowCanPass)
{
  EXPECT_EQ(KickPositionDecision::Approach, evaluateKickPosition(true, 0.5, 0.3, true));
  EXPECT_EQ(KickPositionDecision::Shift, evaluateKickPosition(true, 0.5, 0.3, false));
  EXPECT_EQ(KickPositionDecision::Shift, evaluateKickPosition(true, 0.25, 0.3, false));
  EXPECT_EQ(KickPositionDecision::Ready, evaluateKickPosition(true, 0.3, 0.3, true));
  EXPECT_EQ(KickPositionDecision::Wait, evaluateKickPosition(false, 0.25, 0.3, true));
  for (double invalid : {0.0, -0.1, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_EQ(KickPositionDecision::Wait, evaluateKickPosition(true, invalid, 0.3, true));
    EXPECT_EQ(KickPositionDecision::Wait, evaluateKickPosition(true, 0.25, invalid, true));
  }
}

TEST(KickAlignmentPolicy, HeadingRequiresFreshValidYawWithinFifteenDegrees)
{
  EXPECT_TRUE(kickHeadingReady(true, true, 15.0, 15.0));
  EXPECT_TRUE(kickHeadingReady(true, true, -15.0, 15.0));
  EXPECT_FALSE(kickHeadingReady(true, true, 15.01, 15.0));
  EXPECT_FALSE(kickHeadingReady(true, true, -15.01, 15.0));
  EXPECT_FALSE(kickHeadingReady(true, false, 0.0, 15.0));
  EXPECT_FALSE(kickHeadingReady(true, true, std::numeric_limits<double>::quiet_NaN(), 15.0));
  EXPECT_TRUE(kickHeadingReady(false, false, 0.0, 15.0));
}

TEST(KickAlignmentPolicy, GroundDistanceUsesBallCenterHeightAndRejectsInvalidRays)
{
  EXPECT_NEAR(0.45, kickGroundDistance(0.56, 0.11, kPi / 4.0, 3.0 * kPi / 180.0), 1e-9);
  EXPECT_TRUE(std::isnan(kickGroundDistance(0.56, 0.11, 0.0, 0.05)));
  EXPECT_TRUE(std::isnan(kickGroundDistance(0.56, 0.11, -0.5, 0.05)));
  EXPECT_TRUE(std::isnan(kickGroundDistance(0.56, 0.11, kPi / 2.0, 0.05)));
  EXPECT_TRUE(std::isnan(kickGroundDistance(0.1, 0.11, 0.5, 0.05)));
}

TEST_F(KickAlignmentTest, CameraCalibrationMatchesRecordedRightContactWithoutChangingWalking)
{
  const auto walking_before = follower_->walking();
  follower_->calibrateCamera();
  follower_->observeRecordedRightBall();
  EXPECT_NEAR(0.1456, follower_->distance(), 0.001);
  EXPECT_NEAR(161.19, follower_->servoWindow(), 0.1);
  EXPECT_EQ(KickPositionDecision::Ready, follower_->kickPositionDecision(true));
  EXPECT_FALSE(follower_->canKickNow());
  EXPECT_EQ(walking_before, follower_->walking());
  finishInitialYawCheck();
  ASSERT_TRUE(follower_->processYawAlign(true, 81.51219177246094));
  EXPECT_EQ(BallFollower::OnRight, follower_->getBallPosition());
  EXPECT_TRUE(follower_->canKickNow());
  follower_->setBodyYaw(45.42);
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, HeightAndRadiusAloneDoNotCorrectRecordedDistance)
{
  follower_->calibrateCamera(0.0);
  follower_->observeRecordedRightBall();
  EXPECT_NEAR(0.5042, follower_->distance(), 0.001);
  EXPECT_EQ(KickPositionDecision::Approach, follower_->kickPositionDecision(true));
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, CalibratedGeometryStillApproachesBallThatMovedAway)
{
  follower_->calibrateCamera();
  follower_->observe(0.30, 161.2);
  EXPECT_NEAR(0.30, follower_->distance(), 1e-6);
  EXPECT_EQ(KickPositionDecision::Approach, follower_->kickPositionDecision(true));
  follower_->beginReposition();
  EXPECT_TRUE(follower_->approaching());
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, InvalidCameraPitchCannotAuthorizeKick)
{
  for (double invalid : {-60.01, 60.01, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()})
  {
    follower_->calibrateCamera(invalid);
    follower_->observeRecordedRightBall();
    EXPECT_TRUE(std::isnan(follower_->distance()));
    EXPECT_EQ(KickPositionDecision::Wait, follower_->kickPositionDecision(true));
    EXPECT_FALSE(follower_->canKickNow());
  }
}

TEST_F(KickAlignmentTest, WalkingPresetMatchesAllScreenshotFields)
{
  const auto presets = YAML::LoadFile(OP3_DEMO_PRESET_PATH);
  const auto walking = presets["jalan"];
  const std::map<std::string, double> expected = {
    {"init_x_offset", -0.03}, {"init_y_offset", 0.028}, {"init_z_offset", 0.0408},
    {"init_roll_offset", 0.0}, {"init_pitch_offset", -0.12}, {"init_yaw_offset", 0.0},
    {"period_time", 0.77}, {"dsp_ratio", 0.4}, {"step_fb_ratio", 0.25},
    {"soft_start_time", 0.05}, {"x_move_amplitude", 0.016}, {"y_move_amplitude", 0.0},
    {"z_move_amplitude", 0.037}, {"angle_move_amplitude", 0.0},
    {"angle_move_amplitude_turn_in", 0.0}, {"balance_hip_roll_gain", 0.35},
    {"balance_knee_gain", 0.4}, {"balance_ankle_roll_gain", 0.7},
    {"balance_ankle_pitch_gain", 0.9}, {"y_swap_amplitude", 0.0021},
    {"z_swap_amplitude", 0.0}, {"arm_swing_gain", 0.0}, {"pelvis_offset", 0.05},
    {"hip_pitch_offset", 0.14}, {"p_gain", 36.0}, {"i_gain", 0.0}, {"d_gain", 0.0}
  };
  ASSERT_EQ(29u, walking.size());
  for (const auto &field : expected)
    EXPECT_DOUBLE_EQ(field.second, walking[field.first].as<double>()) << field.first;
  EXPECT_FALSE(walking["move_aim_on"].as<bool>());
  EXPECT_FALSE(walking["balance_enable"].as<bool>());
  EXPECT_EQ(36, follower_->walking().p_gain);
}

TEST_F(KickAlignmentTest, YawSignKeepsOperatorPresetMapping)
{
  EXPECT_TRUE(follower_->orbitLeft(30.0));
  EXPECT_FALSE(follower_->orbitLeft(-30.0));
}

TEST_F(KickAlignmentTest, AlreadyAlignedYawStillChecksFarBallInsideFootWindow)
{
  follower_->observe(0.5, 165.0);
  finishInitialYawCheck();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->approaching());
  EXPECT_FALSE(follower_->canKickNow());
  EXPECT_EQ(36, follower_->walking().p_gain);
  EXPECT_NEAR(0.77, follower_->walking().period_time, 1e-6);
}

TEST_F(KickAlignmentTest, AlignedYawDoesNotReusePreviousGyroIntegral)
{
  follower_->residualTurn();
  finishInitialYawCheck();
  EXPECT_DOUBLE_EQ(0.0, follower_->turned());
}

TEST_F(KickAlignmentTest, MissingYawAbortsRatherThanSkippingAlignment)
{
  follower_->expireYaw();
  ASSERT_TRUE(follower_->startYawAlign());
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FALSE(follower_->isYawAligning());
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, DisablingYawDoesNotDisableDistanceAndFootChecks)
{
  follower_->disableYaw();
  follower_->expireYaw();
  follower_->observe(0.5, 165.0);
  finishInitialYawCheck();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->approaching());
}

TEST_F(KickAlignmentTest, ValidNearbyBallChoosesRightAndLeftFoot)
{
  for (const auto &sample : {std::make_pair(165.0, BallFollower::OnRight),
                             std::make_pair(200.0, BallFollower::OnLeft)})
  {
    follower_->observe(0.25, sample.first);
    finishInitialYawCheck();
    EXPECT_TRUE(follower_->processYawAlign(true, 80.0));
    EXPECT_EQ(sample.second, follower_->getBallPosition());
    EXPECT_TRUE(follower_->canKickNow());
  }
}

TEST_F(KickAlignmentTest, NearBallOutsideFootWindowsMustReposition)
{
  follower_->observe(0.25, 185.0);
  finishInitialYawCheck();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->shifting());
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, BallMovingAwayDuringShiftReturnsToForwardPreset)
{
  follower_->observe(0.25, 185.0);
  follower_->beginReposition();
  follower_->expireTransition();
  follower_->observe(0.5, 165.0);
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->approaching());
  EXPECT_EQ(36, follower_->walking().p_gain);
  EXPECT_FLOAT_EQ(0.0, follower_->walking().angle_move_amplitude_turn_in);
}

TEST_F(KickAlignmentTest, ForwardStopsAtDistanceThenRechecksLateralPosition)
{
  follower_->observe(0.5, 165.0);
  follower_->beginReposition();
  follower_->expireTransition();
  for (int sample = 0; sample < 4; ++sample)
  {
    follower_->observe(0.25, 165.0);
    EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
    EXPECT_FLOAT_EQ(0.0, follower_->walking().x_move_amplitude);
  }
  EXPECT_TRUE(follower_->shifting());
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, FarLateralBallMustShiftBeforeAdvancing)
{
  follower_->observe(0.5, 220.0);
  follower_->beginReposition();
  EXPECT_TRUE(follower_->shifting());
  follower_->observe(0.5, 200.0);
  follower_->expireTransition();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->approaching());
  follower_->observe(0.4, 215.0);
  follower_->expireTransition();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->shifting());
  EXPECT_FLOAT_EQ(0.0, follower_->walking().x_move_amplitude);
}

TEST_F(KickAlignmentTest, ReadyWindowStopsShiftImmediatelyBeforeSettling)
{
  follower_->observe(0.25, 165.0);
  follower_->beginReposition();
  follower_->expireTransition();
  for (int sample = 0; sample < 4; ++sample)
  {
    EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
    EXPECT_FLOAT_EQ(0.0, follower_->walking().y_move_amplitude);
    EXPECT_FLOAT_EQ(0.0, follower_->walking().angle_move_amplitude);
  }
  EXPECT_TRUE(follower_->postSettling());
  follower_->expireSettle();
  EXPECT_TRUE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->canKickNow());
}

// Jatah lebih dari satu kali SENGAJA dipasang di sini: yang diuji adalah
// jalur pemulihannya, yaitu robot mendekat lagi alih alih menendang dari jauh.
// Dengan jatah bawaan (satu kali) robot membatalkan tendangan, dan itu diuji
// terpisah di RepositionRunsOnlyOncePerKickAttempt.
TEST_F(KickAlignmentTest, BallMovingDuringFinalSettleMustApproachAgain)
{
  follower_->setMaxPasses(2);
  follower_->beginReposition();
  follower_->expireTransition();
  for (int sample = 0; sample < 4; ++sample)
    follower_->processYawAlign(true, 80.0);
  ASSERT_TRUE(follower_->postSettling());
  follower_->observe(0.5, 165.0);
  follower_->expireSettle();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_TRUE(follower_->approaching());
}

// Operator meminta penyesuaian posisi kaki cukup SEKALI. Kalau sesudah satu
// kali menggeser bolanya masih belum tepat, robot membatalkan tendangan dan
// mendekati bola dari awal, bukan menggeser lagi di tempat. Tanpa batas ini
// startReposition() memasang ulang tenggatnya sendiri, jadi pengulangannya
// tidak pernah berhenti.
TEST_F(KickAlignmentTest, RepositionRunsOnlyOncePerKickAttempt)
{
  follower_->beginReposition();
  follower_->expireTransition();
  for (int sample = 0; sample < 4; ++sample)
    follower_->processYawAlign(true, 80.0);
  ASSERT_TRUE(follower_->postSettling());

  follower_->observe(0.5, 165.0);
  follower_->expireSettle();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FALSE(follower_->isYawAligning());
  EXPECT_FALSE(follower_->approaching());
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, TimeoutNeverAuthorizesKick)
{
  follower_->observe(0.5, 185.0);
  follower_->beginReposition();
  follower_->expireReposition();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FALSE(follower_->isYawAligning());
  EXPECT_FALSE(follower_->isBallInRange());
}

TEST_F(KickAlignmentTest, YawOutsideToleranceAfterShiftNeverAuthorizesKick)
{
  follower_->beginReposition();
  follower_->expireTransition();
  for (int sample = 0; sample < 4; ++sample)
    follower_->processYawAlign(true, 80.0);
  ASSERT_TRUE(follower_->postSettling());
  follower_->setBodyYaw(20.0);
  follower_->expireSettle();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FALSE(follower_->isYawAligning());
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, MissingOrStaleBallStopsAndEventuallyAborts)
{
  follower_->beginReposition();
  follower_->expireBall();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FLOAT_EQ(0.0, follower_->walking().x_move_amplitude);
  EXPECT_FLOAT_EQ(0.0, follower_->walking().y_move_amplitude);
  follower_->expireBlind();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FALSE(follower_->isYawAligning());
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, CameraOffsetsUpdateDistanceWithoutNormalFollowing)
{
  follower_->observe(0.5, 165.0);
  const double centered_distance = follower_->distance();
  for (int sample = 0; sample < 5; ++sample)
    follower_->updateBallObservation(0.0, -10.0 * kPi / 180.0, 80.0);
  EXPECT_LT(follower_->distance(), centered_distance);
  EXPECT_GT(follower_->distance(), 0.0);
}

TEST_F(KickAlignmentTest, FinalGateRejectsChangedFootYawAndStaleHead)
{
  finishInitialYawCheck();
  ASSERT_TRUE(follower_->processYawAlign(true, 80.0));
  ASSERT_TRUE(follower_->canKickNow());
  follower_->observe(0.25, 200.0);
  EXPECT_FALSE(follower_->canKickNow());
  follower_->observe(0.25, 165.0);
  follower_->setBodyYaw(20.0);
  EXPECT_FALSE(follower_->canKickNow());
  follower_->setBodyYaw(0.0);
  follower_->expireHead();
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, InvalidFrameCannotReusePreviousReadyObservation)
{
  finishInitialYawCheck();
  ASSERT_TRUE(follower_->processYawAlign(true, 80.0));
  follower_->updateBallObservation(std::numeric_limits<double>::quiet_NaN(), 0.0, 80.0);
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, FinalGateDoesNotLetMedianHideBallMovingAway)
{
  finishInitialYawCheck();
  ASSERT_TRUE(follower_->processYawAlign(true, 80.0));
  for (int sample = 0; sample < 5; ++sample)
    follower_->updateBallObservation(0.0, 0.0, 80.0);
  ASSERT_TRUE(follower_->canKickNow());
  follower_->updateBallObservation(0.0, 15.0 * kPi / 180.0, 80.0);
  EXPECT_LT(follower_->distance(), 0.3);
  EXPECT_FALSE(follower_->canKickNow());
}

TEST_F(KickAlignmentTest, DisabledApproachAbortsInsteadOfKickingFromFarAway)
{
  follower_->disableApproach();
  follower_->observe(0.5, 165.0);
  follower_->beginReposition();
  follower_->expireTransition();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FALSE(follower_->isYawAligning());
  EXPECT_FALSE(follower_->canKickNow());
}

// Tahap MAJU dimatikan 2026-09-08: sesudah memutar, robot hanya boleh
// menyesuaikan posisi kaki dengan preset GESER. Bola yang jauh pun tidak boleh
// membuatnya masuk tahap MAJU; yang benar adalah membatalkan tendangan
// (diuji terpisah di DisabledApproachAbortsInsteadOfKickingFromFarAway).
TEST_F(KickAlignmentTest, DisabledApproachNeverEntersForwardStage)
{
  follower_->disableApproach();
  follower_->observe(0.5, 165.0);
  follower_->beginReposition();
  EXPECT_FALSE(follower_->approaching());
  EXPECT_TRUE(follower_->shifting());
}

// walking_module merekam walking_bias_ terhadap offset yang sedang hidup saat
// modulnya dinyalakan. Sesudah robot bangun dari jatuh, yang hidup adalah
// preset dari siklus sebelumnya; kalau itu preset JALAN selisihnya NOL dan
// robot terkunci di pose page 2. Acuannya karena itu harus selamat dari preset
// mana pun dan bisa diterbitkan ulang.
TEST_F(KickAlignmentTest, PoseReferenceSurvivesPresetsThatOverwriteEveryField)
{
  EXPECT_FALSE(follower_->publishPoseReference());

  op3_walking_module_msgs::msg::WalkingParam reference;
  reference.init_x_offset = -0.02;      // angka param.yaml, BUKAN preset JALAN
  reference.init_pitch_offset = -0.07;
  reference.hip_pitch_offset = 0.1396;
  reference.period_time = 0.8;
  follower_->recordPoseReference(reference);

  // Preset PUTAR menimpa SELURUH struct, ketujuh offset pose ikut.
  follower_->publishOrbitLeft();
  EXPECT_NE(follower_->walking().init_x_offset, reference.init_x_offset);

  // FLOAT_EQ, bukan DOUBLE_EQ: field WalkingParam bertipe float32, jadi
  // membandingkannya dengan literal double selalu meleset di digit terakhir.
  EXPECT_TRUE(follower_->publishPoseReference());
  EXPECT_FLOAT_EQ(follower_->walking().init_x_offset, -0.02f);
  EXPECT_FLOAT_EQ(follower_->walking().init_pitch_offset, -0.07f);
  EXPECT_FLOAT_EQ(follower_->walking().hip_pitch_offset, 0.1396f);
  EXPECT_FLOAT_EQ(follower_->walking().period_time, 0.8f);
}

TEST_F(KickAlignmentTest, DisabledRepositionDoesNotAuthorizeFarKick)
{
  follower_->disableReposition();
  follower_->observe(0.5, 165.0);
  finishInitialYawCheck();
  EXPECT_FALSE(follower_->processYawAlign(true, 80.0));
  EXPECT_FALSE(follower_->isYawAligning());
  EXPECT_FALSE(follower_->canKickNow());
}

}
}

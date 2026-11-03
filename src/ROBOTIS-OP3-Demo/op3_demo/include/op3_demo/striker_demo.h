#ifndef OP3_DEMO_STRIKER_DEMO_H_
#define OP3_DEMO_STRIKER_DEMO_H_

// =============================================================================
//  demo_striker -- striker KSTI 2026 di atas mesin keadaan D60
// =============================================================================
//
// SALINAN auto_demo.h (mode otomatis D60) dengan penyesuaian minimal dari
// panduan "ksti2026_STRIKER_adaptasi_minimal-v2". Panduan yang menyesuaikan
// robot: semua gerakan, detektor, pelacak kepala, preset, dan halaman tetap
// milik robot. Bandingkan dengan `diff auto_demo.h striker_demo.h`.
//
// Yang berbeda dari D60:
//   * Titik 1 -- pengarah search B -> A -> C -> D (striker_strategy.h):
//     arah putar MENCARI CELAH per bola target, gerbang terima/tolak bola yang
//     dikunci detektor (cooldown pasca-tendang + sektor servo 19), indeks bola
//     naik sesudah halaman tendang selesai.
//   * Titik 2 -- setpoint orbit: sebelum YAW_CORRECTION, state AIM_LOOK melirik
//     ke depan (perintah tracker "stop:<servo20>" yang sudah ada), menyalakan
//     yolo_gawang_detector (kelas robot / goalpost) sebentar, lalu galat yaw
//     orbit diukur terhadap celah di samping kiper (cadangan: tengah gawang,
//     lalu tabel yaw gyro).
//   * yaw = integrasi gyro z /robotis/open_cr/imu. Magnetometer TIDAK dipakai
//     (regulasi KSTI 2026): tidak ada mag_status, tidak ada reset_yaw.
//   * Tidak dipasang ke tombol: dinyalakan op_striker_node lewat
//     /robotis/demo_command "striker_start" / "striker_stop".
//
// Sisanya identik dengan D60 (lihat auto_demo.h): mesin keadaan yang menumpang
// SELURUH perangkat yang sudah ada -- tidak ada detektor, pelacak, mesin jalan,
// mesin gerakan, atau lintasan servo baru:
//
//   * jalan/geser/putar  = versi tersimpan Bascorro Studio (walking_module),
//                          diterbitkan UTUH termasuk offset init_* miliknya;
//   * tendang & bangun   = halaman action editor lewat action_module
//                          (page 104/105/85/86), timing apa adanya dari .bin;
//   * kepala             = head_tracking_node ("start"/"stop" di
//                          /ball_tracker/command);
//   * deteksi bola       = /ball_detector_node/circle_set (kunci 1 bola milik
//                          detektor = hysteresis lock panduan);
//   * deteksi jatuh      = fall_detector.h, ambang pitch yang sama dengan D56.
//
// Tidak ada panggilan yang memblokir: op_striker_node satu utas (process()
// lalu spin_some()), jadi service dipanggil async dan hasilnya disuapkan balik.

#include <array>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "op3_action_module_msgs/srv/is_running.hpp"
#include "op3_ball_detector_msgs/msg/circle_set_stamped.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"
#include "op3_walking_module_msgs/srv/get_walking_param.hpp"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"

#include "op3_demo/fall_detector.h"
#include "op3_demo/op_demo.h"
#include "op3_demo/striker_strategy.h"
#include "op3_demo/studio_walking_versions.h"

namespace robotis_op
{

namespace striker_demo
{

constexpr double kNever = -std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Status MODE, terpisah dari tahap perilaku.
enum class Mode
{
  Stopped,    // tidak ada perilaku otomatis dan tidak ada perintah yang keluar
  Running,
  Stopping,   // perintah stop sudah terkirim; menunggu walking/kepala/action reda
};

enum class State
{
  Idle,
  Init,             // pose berdiri "JALAN MENUJU MEKKAH" (acuan page 2), tanpa jalan
  StandSearch,      // diam sambil kepala menyapu, berbatas waktu (3 s / 1,5 s)
  SearchBall,       // "MENCARI CELAH" + sapuan kepala
  SearchAlign,      // bola ketemu di 0-30 % / 70-100 %: "MENCARI CELAH" (yaw dibalik / asli) sampai lewat 50 %
  Approach,         // "JALAN MENUJU MEKKAH" menuju bola
  AimLook,          // STRIKER: lirik ke depan, baca kiper/gawang -> target yaw orbit
  YawCorrection,    // "PUTAR KIRI NEW" / "PUTAR KANAN NEW", sekali per percobaan
  DistanceRecheck,  // servo 20 diperiksa lagi sesudah yaw
  BallAlignment,    // persen servo 19 menentukan geser / tendang
  Shift,            // "KIRI MIN NEW" / "KANAN MIN NEW", sekali per percobaan
  PreKickNudge,     // sesudah PUTAR KANAN NEW langsung tendang: KIRI MIN NEW sebentar dulu
  Kick,             // halaman 104 / 105
  GettingUp,        // halaman 85 / 86
  Park,             // kembali ke pose INIT lalu berhenti
};

// Status kepala yang boleh dipakai keputusan. SEARCH dan TRACK tidak pernah
// tertukar: TRACK menuntut head_tracking_node melapor TRACKING dan deteksi bola
// masih segar; nilai servo saat kepala MENYAPU tidak pernah sah.
enum class HeadMode
{
  Inactive,
  Search,
  Track,
};

enum class Foot
{
  None,
  Left,
  Right,
};

// Arah pemicu penyelarasan saat mencari (SearchAlign): dari mana bola pertama
// kali terlihat. Menentukan yaw yang dipakai DAN syarat berhentinya.
enum class AlignDir
{
  None,
  FromLeft,    // servo 19 0-30 %: MENCARI CELAH yaw DIBALIK, berhenti saat > 40 %
  FromRight,   // servo 19 70-100 %: MENCARI CELAH ASLI, berhenti saat < 60 %
};

// Peran preset = versi tersimpan Bascorro Studio. Init dan Approach memakai
// versi yang sama ("JALAN MENUJU MEKKAH"): Init diterbitkan TANPA start (pose
// berdiri saja), Approach diterbitkan lalu start.
enum class PresetRole
{
  PoseReference,   // versi INIT dengan 6 offset bias = acuan page 2 (seperti Studio)
  Init,
  Search,
  SearchInverted,  // MENCARI CELAH dengan angle_move_amplitude DIBALIK tandanya (runtime saja, tidak disimpan)
  Approach,
  TurnLeft,
  TurnRight,
  ShiftLeft,
  ShiftRight,
  Count,
};

const char *toString(Mode mode);
const char *toString(State state);
const char *toString(HeadMode head);
const char *toString(Foot foot);
const char *toString(AlignDir dir);
const char *toString(PresetRole role);

// Penanda SATU PERCOBAAN (§45 permintaan operator: yaw, jarak, dan geser
// masing-masing hanya boleh sekali). Direset saat D60 dinyalakan, sesudah
// menendang, dan sesudah bangun dari jatuh -- TIDAK direset saat bola hilang
// lalu ketemu lagi.
struct AttemptFlags
{
  bool yaw_correction_attempted = false;
  bool distance_correction_attempted = false;
  bool shift_attempted = false;
  bool kick_executed = false;

  bool approach_completed = false;
  bool yaw_stage_completed = false;
  bool distance_stage_completed = false;

  Foot foot = Foot::None;
  // Geser TARIK dari zona luar (p < outer_left -> KANAN MIN NEW sampai p >=
  // outer_left_stop; p > outer_right -> KIRI MIN NEW sampai p <= outer_right_stop),
  // sekali per percobaan, terpisah dari geser zona tengah (shift_attempted).
  bool outer_shift_attempted = false;
  PresetRole shift_role = PresetRole::Count;   // preset geser yang sedang/akan dipakai
  bool shift_outer = false;                    // geser yang berjalan = geser tarik
  // Geser zona tengah: batasnya (kiri < kick_left, kanan > kick_right) PERNAH
  // tersentuh -- dijepit di callback servo (~125 Hz), walau cuma sesaat. Begitu
  // tersentuh: stop, lalu LANGSUNG tendang tanpa menilai ulang persen.
  bool shift_target_touched = false;
  // Gerakan badan TERAKHIR yang dijalankan adalah "PUTAR KANAN NEW" (tanpa
  // jalan/geser sesudahnya). Dipakai PreKickNudge.
  bool last_motion_turn_right = false;
  bool prekick_nudge_done = false;
  // STRIKER: target orbit sudah ditentukan (lirik AIM_LOOK) untuk percobaan ini.
  bool aim_done = false;
};

// STRIKER: parameter striker_* (config/striker_params.yaml).
struct StrikerConfig
{
  striker::StrategyConfig strategy;
  // MENCARI CELAH asli (angle_move_amplitude -0,2) = putar KANAN; dibalik = kiri.
  bool search_original_is_right = true;
  // Gerbang bola: status dibuang sesudah bola hilang selama ini (= target_lost_sec detektor).
  double gate_release_sec = 1.5;
  // Setengah FOV vertikal kamera = fov_height_deg head_tracking.yaml.
  double half_vfov_deg = 21.6;
  // Lirik sebelum orbit.
  bool aim_look_enable = true;
  double aim_look_tilt_servo_deg = 180.0;
  double aim_look_wait_sec = 1.5;     // batas tunggu kepala sampai pose lirik
  double aim_look_tol_deg = 3.0;
  double aim_collect_sec = 0.6;       // detektor kiper/gawang dinyalakan selama ini
  double aim_reacquire_sec = 4.0;     // batas pelacak mengunci bola lagi sesudah lirik
  // Yaw gyro.
  double gyro_z_sign = 1.0;
  double gyro_bias_tau_sec = 2.0;
  double gyro_max_dt_sec = 0.1;
};

struct PresetInfo
{
  bool available = false;
  std::string name;
  double period_time = kNaN;
  double x_move_amplitude = kNaN;
  std::string problem;
};

struct Config
{
  // ---- waktu -------------------------------------------------------------
  double transition_sec = 0.4;          // satu-satunya jeda antar-gerakan
  // Jeda TAMBAHAN sesudah "MENCARI CELAH" (SEARCH_BALL / SEARCH_ALIGN) benar-benar
  // selesai dan diputuskan lanjut, sebelum gerakan badan berikutnya dimulai.
  double search_to_walk_delay_sec = 0.0;
  // Sesudah PUTAR KANAN NEW lalu langsung tendang: KIRI MIN NEW selama ini dulu.
  // <= 0 = mati.
  double prekick_nudge_sec = 0.3;
  double initial_search_sec = 3.0;      // diam mencari sesudah INIT
  double start_forward_sec = 20.0;     // sesudah diam pertama tanpa bola: jalan preset mendekat (JALAN BARU) + sapuan kepala
  double start_search_sec = 7.0;        // idem, KHUSUS sesi baru (tepat sesudah D60 ditekan)
  double post_action_search_sec = 1.5;  // sesudah tendang / bangun
  double lost_search_sec = 3.0;         // sesudah bola hilang
  double ball_valid_gap_sec = 5.0 / 30.0;   // aturan NotFound BallTracker D56
  double ball_lost_sec = 1.0;           // hilang selama ini saat badan bergerak = recovery
  double tracker_status_timeout_sec = 0.5;
  double head_joint_fresh_sec = 0.5;

  // ---- servo 19 / 20 -----------------------------------------------------
  double servo_center_deg = 180.0;
  double servo_dir = 1.0;
  double pan_left_limit_servo_deg = 265.0;   // head_pan +85 deg = 0 %
  double pan_right_limit_servo_deg = 95.0;   // head_pan -85 deg = 100 %
  // Zona keputusan persen servo 19 (0 % kiri, 100 % kanan).
  double pan_kick_left = 48.0;
  double pan_center = 50.0;
  double pan_kick_right = 52.0;
  // Zona luar: di bawah/atas ini bola ditarik dulu dengan geser berlawanan
  // (sekali) sampai persen mencapai *_stop, baru tabel di atas dipakai.
  double pan_outer_left = 45.0;
  double pan_outer_left_stop = 46.0;
  double pan_outer_right = 55.0;
  double pan_outer_right_stop = 54.0;
  // SearchAlign: bola pertama terlihat saat mencari di <= search_align_left_max
  // -> FromLeft; >= search_align_right_min -> FromRight; di antaranya = logika lama.
  double search_align_left_max = 30.0;
  double search_align_right_min = 70.0;
  // Target berhenti SearchAlign: FromLeft (yaw dibalik) berhenti saat > stop_left,
  // FromRight (asli) berhenti saat < stop_right.
  double search_align_stop_left = 40.0;
  double search_align_stop_right = 60.0;
  // Servo 20 = JARAK. <= batas: bola masih jauh. > batas: bola sudah dekat.
  double tilt_limit_deg = 200.0;

  // ---- yaw ---------------------------------------------------------------
  double yaw_target_deg = 0.0;
  double yaw_tolerance_deg = 6.0;    // di dalam ini tidak perlu memutar
  double yaw_stop_deg = 4.0;          // putaran dihentikan di sini
  // Sisi NEGATIF (19 Sep 2026): putar kanan sudah mulai di galat < -0,5 dan
  // berhenti begitu galat >= 0; sisi positif tetap +6 / +4.
  double yaw_tolerance_neg_deg = 0.5;
  double yaw_stop_neg_deg = 0.0;
  double yaw_max_age_sec = 1.0;
  double yaw_timeout_sec = 20.0;

  // ---- setir saat mendekat (rumus BallFollower D56, angka lebih agresif) ----
  // angle_move_amplitude = tanda x clamp(|pan| x gain, min..max), naik bertahap
  // ramp tiap tick; di bawah deadband tidak menyetir (jalan lurus).
  double steer_gain = 0.2;             // D56: 0.2
  double steer_min_deg = 5.0;          // D56 MIN_RL_TURN
  double steer_max_deg = 15.0;         // D56 MAX_RL_TURN
  double steer_ramp_deg = 0.5;         // D56 UNIT_RL_TURN 0.5
  double steer_deadband_deg = 5.0;     // D56: 5
  double steer_sign = 1.0;             // +1 = pan kiri (+) -> angle +
  double steer_publish_period_sec = 0.1;

  // ---- batas waktu gerakan ------------------------------------------------
  double approach_timeout_sec = 30.0;
  double shift_timeout_sec = 8.0;
  double search_timeout_sec = 60.0;

  // ---- badan --------------------------------------------------------------
  double idle_leg_speed_deg_s = 30.0;
  double leg_stationary_speed_deg_s = 0.5;
  double leg_quiet_window_sec = 0.1;
  double leg_motion_fresh_sec = 0.5;
  double gait_stop_factor = 1.125;    // 0,75 periode x soft start 1,5
  double fallback_gait_period_sec = 0.8;
  double walking_start_confirm_sec = 1.0;

  // ---- init kepala sebelum halaman action ------------------------------------
  // Sebelum sendi diserahkan ke action_module, pelacak di-"stop" -> kepala
  // diparkir (head_tracking_node: pan 0 deg, servo 20 200) dan ditunggu sampai
  // posisi nyata dekat parkir atau batas waktu (head_control_module: absolut
  // butuh >= 1,0 s).
  double head_park_wait_sec = 1.5;
  double head_park_tol_deg = 3.0;
  double head_park_pan_deg = 0.0;
  double head_park_tilt_servo_deg = 200.0;
  // Servo 20 saat D60 dimatikan (kepala diparkir mendongak, bukan menunduk).
  double stop_head_tilt_servo_deg = 135.0;

  // ---- action -------------------------------------------------------------
  int page_kick_left = 104;
  int page_kick_right = 105;
  int page_getup_front = 85;
  int page_getup_back = 86;
  double action_start_timeout_sec = 1.0;
  double module_switch_timeout_sec = 5.0;
  double preset_verify_timeout_sec = 0.5;
  int preset_verify_attempts = 4;

  // ---- berhenti -----------------------------------------------------------
  double stop_confirm_timeout_sec = 10.0;
  double telemetry_period_sec = 1.0;

  // ---- STRIKER --------------------------------------------------------------
  StrikerConfig striker;
};

}  // namespace striker_demo

// ============================================================================

class StrikerDemo
{
public:
  StrikerDemo();
  ~StrikerDemo();

  void setNode(rclcpp::Node::SharedPtr node);

  // TOGGLE (op_striker_node: "striker_start" saat STOPPED). STOPPED -> sesi baru
  // dari awal (semua penanda direset), RUNNING -> berhenti, STOPPING -> tekanan
  // terakhir yang berlaku. Tidak dipasang ke tombol mana pun.
  void toggle(const std::string &source);
  // Berhenti dari sebab lain: mode_long / demo berpindah, perintah stop.
  void requestStop(const std::string &reason);
  // WAJIB dipanggil tiap loop selama isBusy().
  void process();

  striker_demo::Mode modeStatus() const { return mode_; }
  bool isRunning() const { return mode_ == striker_demo::Mode::Running; }
  bool isBusy() const { return mode_ != striker_demo::Mode::Stopped; }
  bool hasQueuedStart() const { return start_queued_ || queued_start_due_; }

private:
  using WalkingParam = op3_walking_module_msgs::msg::WalkingParam;
  using PresetRole = striker_demo::PresetRole;
  using State = striker_demo::State;
  using Mode = striker_demo::Mode;
  using Foot = striker_demo::Foot;
  using HeadMode = striker_demo::HeadMode;

  // ---- waktu & parameter ---------------------------------------------------
  double steadyNow() const;
  void declareParameters();
  double param(const std::string &name, double fallback) const;
  std::string paramString(const std::string &name, const std::string &fallback) const;
  // STRIKER: larik 4 angka (yaml boleh bulat atau desimal).
  bool paramArray4(const std::string &name, std::array<double, striker::kJumlahBola> &out) const;
  bool prepareConfig(std::string &problem);
  // Kosong = siap; selain itu alasan sesi DITOLAK (acuan page 2 tidak cocok).
  std::string loadStudioPresets();
  const striker_demo::PresetInfo &preset(PresetRole role) const;

  // ---- sesi ---------------------------------------------------------------
  void startSession(double now);
  void beginStop(double now, const std::string &reason);
  void tickStopping(double now);
  void finishStop(double now, const std::string &unconfirmed);
  void resetAttemptFlags(double now, const std::string &why);

  // ---- mesin keadaan -------------------------------------------------------
  void tick(double now);
  void enterState(double now, State next, const std::string &why);
  void setStep(double now, int step);
  void enterInit(double now, State after, double search_sec, bool reset_flags, const std::string &why);
  void enterLostRecovery(double now, const std::string &why);
  void enterGetUp(double now, PitchFallState fall);
  void routeAfterTrack(double now);

  void tickInit(double now);
  void tickStandSearch(double now);
  void tickSearchBall(double now);
  void tickSearchAlign(double now);
  // true = bola (tracking sah) di zona luar saat mencari -> masuk SearchAlign.
  bool maybeEnterSearchAlign(double now);
  void tickApproach(double now);
  // STRIKER: lirik ke depan sebelum orbit, hasilnya aim_target_yaw_deg_.
  void tickAimLook(double now);
  // STRIKER: approach selesai -> AIM_LOOK (sekali per percobaan) lalu YAW_CORRECTION.
  void enterOrbitStage(double now, const std::string &why);
  void setAimTargetFromTable(const char *why);
  // STRIKER: preset MENCARI CELAH sesuai arah putar bola target (Titik 1).
  PresetRole rolePutarCari() const;
  void setDetectorsEnabled(bool enabled);
  void tickYaw(double now);
  void tickDistance(double now);
  void tickAlignment(double now);
  void tickShift(double now);
  void tickPreKickNudge(double now);
  // true = servo 19 di zona tendang kaki itu DAN servo 20 dekat (TRACK sah).
  bool kickZoneSatisfied(double now, Foot foot) const;
  void tickKick(double now);
  void tickGetUp(double now);
  void tickPark(double now);

  // ---- pengamatan turunan ---------------------------------------------------
  bool ballValid(double now) const;
  bool trackerFresh(double now) const;
  bool presentFresh(double now) const;
  HeadMode headMode(double now) const;
  bool trackValid(double now) const;       // ballDetected && ballTrackingValid && TRACK
  double servo19() const;
  double servo20() const;
  double panPercent() const;
  double yawErrorDeg() const;
  bool yawInTolerance(double err) const;  // -tol_neg <= err <= tol
  bool yawInStopBand(double err) const;   // -stop_neg <= err <= stop
  bool yawUsable(double now) const;
  bool bodyStopped(double now) const;
  bool transitionElapsed(double now) const;
  bool canStartMotion(double now) const;
  bool actionRunning(double now) const;
  bool legsOwnedBy(const std::string &module) const;
  bool allOwnedBy(const std::string &module) const;
  double currentGaitPeriod() const;
  std::string snapshot(double now) const;
  // kosong = boleh menendang. after_park = kepala sudah diparkir (pelacak mati):
  // syarat TRACK / persen servo 19 tidak dinilai lagi, sisanya tetap.
  std::string kickDenialReason(double now, bool after_park = false) const;
  bool headParked(double now) const;   // posisi nyata kepala dekat pose parkir
  double head_park_sent_at_ = striker_demo::kNever;

  // ---- perintah -------------------------------------------------------------
  int presetLanded(double now, PresetRole role);    // 1 = sudah terbukti hidup
  void clearPresetRequest();
  bool beginWalking(double now, PresetRole role);
  bool walkingStartAccepted(double now, const char *label);
  void stopWalking(double now, const std::string &why);
  // Menyetir gait mendekat mengikuti arah kepala (servo 19) selama berjalan.
  void steerTowardBall(double now);
  void sendPage(double now, int page);
  void publishPreset(PresetRole role);
  void verifyPreset(PresetRole role, uint64_t token);
  void publishJointModules(const std::vector<std::string> &joints, const std::string &module);
  void publishWalkingCommand(const char *text);
  void publishTracker(const char *text);
  void startTracker(double now);
  void stopTracker();
  void pollActionRunning(double now);
  void logTelemetry(double now);

  // ---- callback sensor ------------------------------------------------------
  void circleSetCallback(const op3_ball_detector_msgs::msg::CircleSetStamped::SharedPtr msg);
  void trackerStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void presentJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void goalJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // STRIKER: yolo_gawang_detector (kelas robot / goalpost) selama AIM_LOOK.
  void keeperDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void goalpostDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void jointModulesCallback(const robotis_controller_msgs::msg::JointCtrlModule::SharedPtr msg);
  void movementDoneCallback(const std_msgs::msg::String::SharedPtr msg);
  void statusCallback(const robotis_controller_msgs::msg::StatusMsg::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  bool parameters_declared_ = false;
  striker_demo::Config cfg_;

  // ---- preset ---------------------------------------------------------------
  std::map<PresetRole, WalkingParam> preset_msgs_;
  std::vector<striker_demo::PresetInfo> preset_info_;
  std::array<double, 6> pose_reference_{};

  // ---- publisher / client / langganan ---------------------------------------
  rclcpp::Publisher<WalkingParam>::SharedPtr walking_param_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr walking_command_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr joint_module_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr page_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_command_pub_;
  // STRIKER: enable/disable yolo_gawang_detector (kiper / gawang).
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr keeper_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalpost_enable_pub_;
  // Kepala absolut ke head_control_module: parkir servo 20 135 saat D60 OFF
  // kalau kepala masih dipegang action_module (D60 dimatikan saat page jalan).
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_abs_pub_;   // titik nol yaw saat D60 mulai
  rclcpp::Client<op3_walking_module_msgs::srv::GetWalkingParam>::SharedPtr get_walking_param_client_;
  rclcpp::Client<op3_action_module_msgs::srv::IsRunning>::SharedPtr is_running_client_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Clock::SharedPtr log_clock_;

  // ---- mode / tahap ---------------------------------------------------------
  Mode mode_ = Mode::Stopped;
  State state_ = State::Idle;
  int step_ = 0;
  double state_since_ = striker_demo::kNever;
  double step_since_ = striker_demo::kNever;
  uint64_t session_ = 0;
  uint64_t attempt_ = 0;
  bool start_queued_ = false;
  bool queued_start_due_ = false;
  double stop_started_at_ = striker_demo::kNever;
  std::string stop_reason_;
  bool stop_waits_action_ = false;
  double telemetry_logged_at_ = striker_demo::kNever;

  striker_demo::AttemptFlags flags_;
  // Sasaran sesudah INIT selesai.
  State after_init_ = State::StandSearch;
  double after_init_search_sec_ = 3.0;
  bool after_init_reset_flags_ = false;
  double stand_search_deadline_ = striker_demo::kNever;
  striker_demo::AlignDir search_align_dir_ = striker_demo::AlignDir::None;   // != None = correction aktif

  // ---- bola / kepala ---------------------------------------------------------
  bool ball_seen_ = false;
  double ball_stamp_ = striker_demo::kNever;
  double ball_x_ = striker_demo::kNaN;
  double ball_y_ = striker_demo::kNaN;
  double ball_radius_ = striker_demo::kNaN;
  // §27: begitu bola hilang, SEMUA angka lama (persen servo 19, servo 20, arah
  // tendang) dianggap tidak sah sampai pelacak mengunci lagi.
  bool ball_tracking_valid_ = false;
  std::string tracker_text_;
  double tracker_stamp_ = striker_demo::kNever;
  bool tracker_started_ = false;
  // 19 Sep 2026: diam mencari PERTAMA sesudah D60 ditekan -> kepala ditahan di
  // (50 %, servo 20 135) lewat sapuan "scan_hold" (tidak menyapu). Dilepas saat
  // StandSearch pertama itu ditinggalkan.
  bool start_hold_ = false;
  // 19 Sep 2026: diam pertama habis TANPA bola -> SEARCH_BALL jalan dulu dengan
  // preset mendekat (JALAN BARU) selama start_forward_sec, baru MENCARI CELAH.
  bool start_forward_pending_ = false;
  bool start_forward_active_ = false;
  double tracker_start_at_ = striker_demo::kNever;
  double pan_present_ = striker_demo::kNaN;
  double tilt_present_ = striker_demo::kNaN;
  double present_stamp_ = striker_demo::kNever;

  // ---- badan ------------------------------------------------------------------
  double leg_motion_stamp_ = striker_demo::kNever;
  double leg_fast_last_ = striker_demo::kNever;
  double leg_moving_last_ = striker_demo::kNever;
  bool walking_commanded_ = false;
  double walking_stop_at_ = striker_demo::kNever;
  double walking_start_sent_at_ = striker_demo::kNever;
  double walking_status_started_at_ = striker_demo::kNever;
  bool walking_start_reported_ = false;
  double running_period_ = striker_demo::kNaN;
  PresetRole running_role_ = PresetRole::Count;
  double motion_end_ = striker_demo::kNever;
  // Saat soft start gait terakhir SELESAI (start + soft_start_time preset itu).
  // Jeda antar-gerakan baru dihitung sesudah titik ini DAN sesudah badan diam.
  double soft_start_end_ = striker_demo::kNever;
  double steer_angle_rad_ = 0.0;               // sudut setir yang sedang dipakai (ramp)
  double steer_published_at_ = striker_demo::kNever;
  double switch_sent_at_ = striker_demo::kNever;
  bool owners_claimed_ = false;

  // ---- IMU --------------------------------------------------------------------
  double roll_deg_ = striker_demo::kNaN;
  double pitch_deg_ = striker_demo::kNaN;
  double yaw_deg_ = striker_demo::kNaN;
  double imu_stamp_ = striker_demo::kNever;
  double filtered_pitch_ = 0.0;
  PitchFallState fall_ = PitchFallState::Stand;
  // Jepitan (latch) ambang yaw, diisi di imuCallback (~125 Hz), BUKAN di tick
  // 30 Hz: persilangan ambang yang cuma sepersekian detik tetap terhitung.
  bool yaw_tolerance_seen_ = false;   // galat di dalam toleransi pernah terlihat saat menilai
  bool yaw_stop_reached_ = false;     // galat di dalam pita stop pernah terlihat saat memutar

  // ---- kepemilikan sendi --------------------------------------------------------
  std::string legs_owner_ = "tidak-diketahui";
  std::string head_owner_ = "tidak-diketahui";

  // ---- action --------------------------------------------------------------------
  bool our_action_pending_ = false;
  int our_action_page_ = 0;
  double our_action_sent_at_ = striker_demo::kNever;
  bool our_action_started_ = false;
  double our_action_started_at_ = striker_demo::kNever;
  bool our_action_failed_ = false;
  bool action_poll_in_flight_ = false;
  double last_action_poll_ = striker_demo::kNever;
  double action_poll_stamp_ = striker_demo::kNever;
  double action_poll_request_time_ = striker_demo::kNever;
  bool action_poll_running_ = false;

  // ---- verifikasi preset -----------------------------------------------------------
  PresetRole pending_preset_role_ = PresetRole::Count;
  uint64_t verify_token_ = 0;
  uint64_t next_token_ = 1;
  double verify_sent_at_ = striker_demo::kNever;
  bool verify_answered_ = false;
  bool verify_ok_ = false;
  int verify_attempts_ = 0;

  // ---- laju sendi kaki --------------------------------------------------------------
  struct LegSample
  {
    double stamp;
    std::array<double, 12> position;
  };
  std::deque<LegSample> leg_samples_;

  // ---- STRIKER ---------------------------------------------------------------------
  striker::PengarahCariBola pengarah_;
  striker::GerbangBola gerbang_;
  striker::GyroYaw gyro_;
  striker::PengumpulTarget kumpul_;
  double aim_target_yaw_deg_ = 0.0;
  striker::ModeTarget aim_mode_ = striker::ModeTarget::GyroDR;
  bool aim_collecting_ = false;
  double aim_sent_at_ = striker_demo::kNever;
  double aim_pan_deg_ = striker_demo::kNaN;
  // Sesudah tendang: cooldown dimulai saat kepala menyala lagi (akhir INIT).
  bool post_kick_look_pending_ = false;
  // Waktu ROS saat detektor kiper/gawang dinyalakan: deteksi dari frame yang
  // ditangkap sebelum kepala sampai di pose lirik (latensi kamera) dibuang.
  rclcpp::Time aim_collect_ros_start_{0, 0, RCL_ROS_TIME};
};

}  // namespace robotis_op

#endif  // OP3_DEMO_STRIKER_DEMO_H_

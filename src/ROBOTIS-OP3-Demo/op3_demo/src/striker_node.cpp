// =============================================================================
//  op_striker_node -- program demo_striker (KSTI 2026)
// =============================================================================
//
// Program TERPISAH dari op_demo_node (demo lama + D60 tidak disentuh). Isinya
// hanya:
//   1. tunggu op3_manager siap lalu page 2 (WALKING_READY) -- urutan boot yang
//      sama persis dengan demo_node.cpp (blok boot + goInitPose disalin);
//   2. StrikerDemo (striker_demo.h) yang dinyalakan LEWAT PROGRAM:
//        ros2 topic pub -1 /robotis/demo_command std_msgs/String "{data: striker_start}"
//        ros2 topic pub -1 /robotis/demo_command std_msgs/String "{data: striker_stop}"
//      Node ini TIDAK berlangganan tombol OpenCR sama sekali (D60, mode, start
//      tidak berefek apa pun pada striker).
//
// Jangan jalankan bersama demo.launch.xml: dua pengendali badan.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "op3_demo/op_demo.h"
#include "op3_demo/striker_demo.h"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"

#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <string>
#include <vector>

namespace
{

// Jejak tumpukan saat node mati mendadak (sama dengan demo_node.cpp).
extern "C" void crashHandler(int sig)
{
  static const char msg[] = "\n=== op_striker_node MATI karena sinyal ";
  ssize_t ignored = write(STDERR_FILENO, msg, sizeof(msg) - 1);
  char digits[2] = {static_cast<char>('0' + (sig / 10) % 10), static_cast<char>('0' + sig % 10)};
  ignored = write(STDERR_FILENO, digits, 2);
  ignored = write(STDERR_FILENO, " ===\n", 6);
  (void) ignored;

  void *frames[32];
  const int n = backtrace(frames, 32);
  backtrace_symbols_fd(frames, n, STDERR_FILENO);

  const int fd = open("/tmp/op_striker_crash.log", O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd >= 0)
  {
    ignored = write(fd, msg, sizeof(msg) - 1);
    ignored = write(fd, digits, 2);
    ignored = write(fd, " ===\n", 6);
    backtrace_symbols_fd(frames, n, fd);
    close(fd);
  }
  signal(sig, SIG_DFL);
  raise(sig);
}

const int SPIN_RATE = 30;
const int WALKING_READY_PAGE_NUM = robotis_op::OPDemo::WalkingReady;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr sync_write_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr enable_ctrl_module_pub;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr action_page_pub;
std::shared_ptr<robotis_op::StrikerDemo> striker_demo;

bool checkManagerRunning(const std::string &manager_name)
{
  for (const auto &node_name : node->get_node_graph_interface()->get_node_names())
    if (node_name == manager_name)
      return true;
  return false;
}

// Disalin dari demo_node.cpp goInitPose(): action_module lalu page 2.
void goInitPose()
{
  std_msgs::msg::String enable_msg;
  enable_msg.data = "action_module";
  enable_ctrl_module_pub->publish(enable_msg);
  rclcpp::sleep_for(std::chrono::milliseconds(300));
  for (int i = 0; i < 30; ++i)
  {
    if (action_page_pub->get_subscription_count() > 0)
      break;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  std_msgs::msg::Int32 page_msg;
  page_msg.data = WALKING_READY_PAGE_NUM;
  action_page_pub->publish(page_msg);
}

// Disalin dari demo_node.cpp clearServoSpeedLimit(): batas profile_velocity
// yang tertinggal (Studio Torque ON / op3_action_editor) melumpuhkan gait tanpa
// pesan galat, dan D60 melepasnya sebelum mulai -- striker juga.
void clearServoSpeedLimit()
{
  static const std::vector<std::string> kJoints = {
    "r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll", "r_el", "l_el",
    "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
    "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee",
    "r_ank_pitch", "l_ank_pitch", "r_ank_roll", "l_ank_roll",
    "head_pan", "head_tilt",
  };
  for (const char *item : {"profile_velocity", "profile_acceleration"})
  {
    robotis_controller_msgs::msg::SyncWriteItem msg;
    msg.item_name = item;
    for (const std::string &joint : kJoints)
    {
      msg.joint_name.push_back(joint);
      msg.value.push_back(0);
    }
    sync_write_pub->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(60));
  }
}

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr demo_command_sub;

// Semua entitas ROS dilepas BERURUTAN sebelum rclcpp::shutdown(). Kalau
// dibiarkan sampai destruktor statis (sesudah konteks DDS mati), Fast-DDS
// gagal menghapus datareader lalu node mati SIGSEGV saat Ctrl-C.
int cleanExit()
{
  striker_demo.reset();
  demo_command_sub.reset();
  action_page_pub.reset();
  enable_ctrl_module_pub.reset();
  sync_write_pub.reset();
  node.reset();
  if (rclcpp::ok())
    rclcpp::shutdown();
  return 0;
}

void demoCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "striker_start")
  {
    if (striker_demo->isBusy())
    {
      RCLCPP_WARN(node->get_logger(), "striker_start diabaikan: striker sedang %s (striker_stop dulu)",
                  striker_demo->isRunning() ? "RUNNING" : "STOPPING");
      return;
    }
    clearServoSpeedLimit();
    striker_demo->toggle("/robotis/demo_command striker_start");
  }
  else if (msg->data == "striker_stop" || msg->data == "stop")
  {
    striker_demo->requestStop("/robotis/demo_command " + msg->data);
  }
}

}  // namespace

int main(int argc, char **argv)
{
  signal(SIGSEGV, crashHandler);
  signal(SIGABRT, crashHandler);
  signal(SIGBUS, crashHandler);
  signal(SIGFPE, crashHandler);

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("striker_node");

  sync_write_pub = node->create_publisher<robotis_controller_msgs::msg::SyncWriteItem>("/robotis/sync_write_item", 10);
  enable_ctrl_module_pub = node->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 10);
  action_page_pub = node->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 10);
  demo_command_sub =
      node->create_subscription<std_msgs::msg::String>("/robotis/demo_command", 10, demoCommandCallback);

  striker_demo = std::make_shared<robotis_op::StrikerDemo>();
  striker_demo->setNode(node);
  RCLCPP_WARN(node->get_logger(), "op_striker_node: TIDAK memakai tombol. Mulai/berhenti lewat "
                                  "/robotis/demo_command \"striker_start\" / \"striker_stop\"");

  // ---- boot: sama dengan demo_node.cpp -------------------------------------------
  const std::string manager_name = "/op3_manager";
  while (rclcpp::ok() && !checkManagerRunning(manager_name))
  {
    RCLCPP_WARN(node->get_logger(), "Waiting for op3 manager");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  // Ctrl-C saat masih menunggu: keluar rapi (membuat subscription sesudah
  // shutdown melempar RCLError dan membuat node abort).
  if (!rclcpp::ok())
    return cleanExit();
  {
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::seconds(180);
    auto next_report = started + std::chrono::seconds(5);
    bool modules_constructed = false;
    bool controller_running = false;
    auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/robotis/present_joint_states", 1,
        [&controller_running](const sensor_msgs::msg::JointState::SharedPtr) { controller_running = true; });
    while (rclcpp::ok() && !(modules_constructed && controller_running) &&
           std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(node);
      if (!modules_constructed && action_page_pub->get_subscription_count() > 0)
        modules_constructed = true;
      if (std::chrono::steady_clock::now() >= next_report)
      {
        next_report += std::chrono::seconds(5);
        RCLCPP_WARN(node->get_logger(), "menunggu op3_manager siap %.0f s: action_module %s, present_joint_states %s",
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
                    modules_constructed ? "sudah hidup" : "BELUM hidup",
                    controller_running ? "sudah terbit" : "BELUM terbit");
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    js_sub.reset();
    if (modules_constructed && controller_running)
    {
      RCLCPP_WARN(node->get_logger(), "controller running -- playing WALKING_READY (action page %d)",
                  WALKING_READY_PAGE_NUM);
      goInitPose();
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "boot-init DILEWATI sesudah 180 s: action_module %s, present_joint_states %s",
                   modules_constructed ? "hidup" : "TIDAK hidup", controller_running ? "terbit" : "TIDAK terbit");
    }
  }

  rclcpp::Rate loop_rate(SPIN_RATE);
  RCLCPP_WARN(node->get_logger(), "op_striker_node siap (page 2, diam). Kirim \"striker_start\" untuk mulai.");
  while (rclcpp::ok())
  {
    if (striker_demo->isBusy() || striker_demo->hasQueuedStart())
      striker_demo->process();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return cleanExit();
}

#include "gc_bridge_webots/gc_bridge.hpp"
#include "gc_bridge_webots/game_controller/new_RoboCupGameControlData.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <utility>

namespace robotis_op
{

namespace
{
static_assert(sizeof(HlRobotInfo) == 6, "Unexpected HlRobotInfo size");
static_assert(sizeof(HlRoboCupGameControlReturnData) == 8, "Unexpected HL return packet size");

constexpr const char * kHeader = GAMECONTROLLER_STRUCT_HEADER;
constexpr uint16_t kHlVersion = HL_GAMECONTROLLER_STRUCT_VERSION;

constexpr size_t kHlMinPacketSize = sizeof(HlRoboCupGameControlData);

const char * stateName(uint8_t state)
{
  switch (state) {
    case 0: return "INITIAL";
    case 1: return "READY";
    case 2: return "SET";
    case 3: return "PLAYING";
    case 4: return "FINISHED";
    default: return "UNKNOWN";
  }
}

const char * secondaryName(uint8_t secondary)
{
  switch (secondary) {
    case 0: return "NORMAL";
    case 1: return "PENALTYSHOOT";
    case 2: return "OVERTIME";
    case 3: return "TIMEOUT";
    case 4: return "DIRECT_FREEKICK";
    case 5: return "INDIRECT_FREEKICK";
    case 6: return "PENALTYKICK";
    case 7: return "CORNER_KICK";
    case 8: return "GOAL_KICK";
    case 9: return "THROW_IN";
    default: return "UNKNOWN";
  }
}
}  // namespace

GCBridge::GCBridge()
: Node("op3_gc_bridge"),
  control_socket_fd_(-1),
  status_socket_fd_(-1),
  fallen_(false),
  hl_message_(2),  // GAMECONTROLLER_RETURN_MSG_ALIVE
  gc_host_("127.0.0.1"),
  control_port_(GAMECONTROLLER_DATA_PORT),
  status_port_(GAMECONTROLLER_RETURN_PORT),
  team_number_(1),
  player_number_(1),
  gc_ip_({127, 0, 0, 1})
{
  this->declare_parameter<std::string>("gc_host", gc_host_);
  this->declare_parameter<int>("control_port", control_port_);
  this->declare_parameter<int>("status_port", status_port_);
  this->declare_parameter<int>("team_number", team_number_);
  this->declare_parameter<int>("player_number", player_number_);
  this->declare_parameter<int>("rx_period_ms", 20);
  this->declare_parameter<int>("tx_period_ms", 500);

  gc_host_ = this->get_parameter("gc_host").as_string();
  control_port_ = static_cast<uint16_t>(this->get_parameter("control_port").as_int());
  status_port_ = static_cast<uint16_t>(this->get_parameter("status_port").as_int());
  team_number_ = static_cast<uint8_t>(this->get_parameter("team_number").as_int());
  player_number_ = static_cast<uint8_t>(this->get_parameter("player_number").as_int());

  const int rx_period_ms = this->get_parameter("rx_period_ms").as_int();
  const int tx_period_ms = this->get_parameter("tx_period_ms").as_int();

  raw_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/game_controller/raw", 10);
  state_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/game_controller/state", 10);
  state_json_pub_ = this->create_publisher<std_msgs::msg::String>("/game_controller/state_json", 10);

  fallen_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/game_controller/status/fallen",
    10,
    std::bind(&GCBridge::fallenCallback, this, std::placeholders::_1));
  hl_message_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "/game_controller/status/hl_message",
    10,
    std::bind(&GCBridge::hlMessageCallback, this, std::placeholders::_1));

  setupSockets();

  rx_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(std::max(1, rx_period_ms)),
    std::bind(&GCBridge::receiveControlPackets, this));
  tx_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(std::max(50, tx_period_ms)),
    std::bind(&GCBridge::sendStatusPacket, this));

  RCLCPP_INFO(
    this->get_logger(),
    "GC bridge started (humanoid-only). control_port=%u status_port=%u team=%u player=%u",
    control_port_,
    status_port_,
    team_number_,
    player_number_);
}

GCBridge::~GCBridge()
{
  closeSockets();
}

void GCBridge::setupSockets()
{
  control_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  status_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

  if (control_socket_fd_ < 0 || status_socket_fd_ < 0) {
    throw std::runtime_error("failed to create UDP sockets");
  }

  int reuse = 1;
  if (setsockopt(control_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
    RCLCPP_WARN(this->get_logger(), "Failed setting SO_REUSEADDR on control socket");
  }

  const int flags = fcntl(control_socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(control_socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  sockaddr_in bind_addr {};
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_addr.sin_port = htons(control_port_);
  if (bind(control_socket_fd_, reinterpret_cast<sockaddr *>(&bind_addr), sizeof(bind_addr)) != 0) {
    throw std::runtime_error("failed binding control socket to 0.0.0.0:3838");
  }

  in_addr out_addr {};
  if (inet_aton(gc_host_.c_str(), &out_addr) == 0) {
    throw std::runtime_error("invalid gc_host IPv4 address");
  }
  std::memcpy(gc_ip_.data(), &out_addr.s_addr, 4);
}

void GCBridge::closeSockets()
{
  if (control_socket_fd_ >= 0) {
    close(control_socket_fd_);
    control_socket_fd_ = -1;
  }
  if (status_socket_fd_ >= 0) {
    close(status_socket_fd_);
    status_socket_fd_ = -1;
  }
}

void GCBridge::receiveControlPackets()
{
  std::array<uint8_t, 2048> buffer {};

  while (rclcpp::ok()) {
    sockaddr_in src_addr {};
    socklen_t addr_len = sizeof(src_addr);
    const ssize_t n = recvfrom(
      control_socket_fd_,
      buffer.data(),
      buffer.size(),
      0,
      reinterpret_cast<sockaddr *>(&src_addr),
      &addr_len);

    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "recvfrom() failed: %s", std::strerror(errno));
      break;
    }

    if (n == 0) {
      break;
    }

    std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + n);
    ParsedState state {};
    const bool parsed = parseHlControl(packet, state);

    if (parsed) {
      std::memcpy(gc_ip_.data(), &src_addr.sin_addr.s_addr, 4);
      publishState(state, packet);
    }
  }
}

void GCBridge::sendStatusPacket()
{
  if (status_socket_fd_ < 0) {
    return;
  }

  sockaddr_in dst {};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(status_port_);
  std::memcpy(&dst.sin_addr.s_addr, gc_ip_.data(), 4);

  HlRoboCupGameControlReturnData return_data {};
  return_data.team = team_number_;
  return_data.player = player_number_;
  return_data.message = hl_message_;

  const ssize_t sent = sendto(
    status_socket_fd_,
    &return_data,
    sizeof(return_data),
    0,
    reinterpret_cast<sockaddr *>(&dst),
    sizeof(dst));
  if (sent < 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "sendto(HL) failed: %s", std::strerror(errno));
  }
  if (fallen_) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Fallen flag is set but ignored in humanoid return packet format.");
  }
}

void GCBridge::publishState(const ParsedState & state, const std::vector<uint8_t> & raw_packet)
{
  std_msgs::msg::UInt8MultiArray raw_msg;
  raw_msg.data = raw_packet;
  raw_pub_->publish(raw_msg);

  std_msgs::msg::Int32MultiArray parsed_msg;
  // Data order:
  // [state, secondary, kicking_team, secs_remaining, secondary_time,
  //  team0_number, team0_score, team1_number, team1_score, packet_number, first_half]
  parsed_msg.data = {
    static_cast<int32_t>(state.state),
    state.secondary,
    static_cast<int32_t>(state.kicking_team),
    state.secs_remaining,
    state.secondary_time,
    static_cast<int32_t>(state.teams[0].team_number),
    static_cast<int32_t>(state.teams[0].score),
    static_cast<int32_t>(state.teams[1].team_number),
    static_cast<int32_t>(state.teams[1].score),
    static_cast<int32_t>(state.packet_number),
    static_cast<int32_t>(state.first_half)
  };
  state_pub_->publish(parsed_msg);

  std_msgs::msg::String json_msg;
  std::ostringstream s;
  s << "{"
    << "\"state\":\"" << stateName(state.state) << "\","
    << "\"secondary_state\":\"" << secondaryName(static_cast<uint8_t>(state.secondary)) << "\","
    << "\"packet_number\":" << static_cast<int>(state.packet_number) << ","
    << "\"players_per_team\":" << static_cast<int>(state.players_per_team) << ","
    << "\"game_type\":" << static_cast<int>(state.game_type) << ","
    << "\"first_half\":" << static_cast<int>(state.first_half) << ","
    << "\"kicking_team\":" << static_cast<int>(state.kicking_team) << ","
    << "\"drop_in_team\":" << static_cast<int>(state.drop_in_team) << ","
    << "\"drop_in_time\":" << state.drop_in_time << ","
    << "\"secs_remaining\":" << state.secs_remaining << ","
    << "\"secondary_time\":" << state.secondary_time << ","
    << "\"secondary_state_info\":[" << static_cast<int>(state.secondary_info[0]) << ","
    << static_cast<int>(state.secondary_info[1]) << ","
    << static_cast<int>(state.secondary_info[2]) << ","
    << static_cast<int>(state.secondary_info[3]) << "],"
    << "\"raw_packet_size\":" << raw_packet.size() << ","
    << "\"raw_packet_hex\":\"";
  for (size_t i = 0; i < raw_packet.size(); ++i) {
    if (i > 0) {
      s << " ";
    }
    s << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(raw_packet[i]);
  }
  s << std::dec << "\","
    << "\"teams\":[";
  for (size_t t = 0; t < state.teams.size(); ++t) {
    const auto & team = state.teams[t];
    if (t > 0) {
      s << ",";
    }
    s << "{"
      << "\"team_number\":" << static_cast<int>(team.team_number) << ","
      << "\"field_player_colour\":" << static_cast<int>(team.field_player_colour) << ","
      << "\"score\":" << static_cast<int>(team.score) << ","
      << "\"penalty_shot\":" << static_cast<int>(team.penalty_shot) << ","
      << "\"single_shots\":" << static_cast<int>(team.single_shots) << ","
      << "\"coach_sequence\":" << static_cast<int>(team.coach_sequence) << ","
      << "\"coach\":{\"penalty\":" << static_cast<int>(team.coach.penalty)
      << ",\"secs_till_unpenalised\":" << static_cast<int>(team.coach.secs_till_unpenalised)
      << ",\"warnings\":" << static_cast<int>(team.coach.warnings)
      << ",\"yellow_cards\":" << static_cast<int>(team.coach.yellow_cards)
      << ",\"red_cards\":" << static_cast<int>(team.coach.red_cards)
      << ",\"goalkeeper\":" << static_cast<int>(team.coach.goalkeeper) << "},"
      << "\"coach_message_hex\":\"";
    for (size_t i = 0; i < team.coach_message.size(); ++i) {
      if (i > 0) {
        s << " ";
      }
      s << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(team.coach_message[i]);
    }
    s << std::dec << "\","
      << "\"players\":[";
    for (size_t p = 0; p < team.players.size(); ++p) {
      const auto & player = team.players[p];
      if (p > 0) {
        s << ",";
      }
      s << "{"
        << "\"player_index\":" << (p + 1) << ","
        << "\"penalty\":" << static_cast<int>(player.penalty) << ","
        << "\"secs_till_unpenalised\":" << static_cast<int>(player.secs_till_unpenalised) << ","
        << "\"warnings\":" << static_cast<int>(player.warnings) << ","
        << "\"yellow_cards\":" << static_cast<int>(player.yellow_cards) << ","
        << "\"red_cards\":" << static_cast<int>(player.red_cards) << ","
        << "\"goalkeeper\":" << static_cast<int>(player.goalkeeper)
        << "}";
    }
    s << "]"
      << "}";
  }
  s << "]"
    << "}";
  json_msg.data = s.str();
  state_json_pub_->publish(json_msg);
}

bool GCBridge::parseHlControl(const std::vector<uint8_t> & packet, ParsedState & state) const
{
  if (packet.size() != kHlMinPacketSize) {
    return false;
  }

  HlRoboCupGameControlData data {};
  std::memcpy(&data, packet.data(), sizeof(data));

  if (std::memcmp(data.header, kHeader, 4) != 0) {
    return false;
  }
  if (data.version != kHlVersion) {
    return false;
  }

  state.packet_number = data.packetNumber;
  state.players_per_team = data.playersPerTeam;
  state.game_type = data.gameType;
  state.state = data.state;
  state.secondary = data.secondaryState;
  state.secondary_info[0] = static_cast<uint8_t>(data.secondaryStateInfo[0]);
  state.secondary_info[1] = static_cast<uint8_t>(data.secondaryStateInfo[1]);
  state.secondary_info[2] = static_cast<uint8_t>(data.secondaryStateInfo[2]);
  state.secondary_info[3] = static_cast<uint8_t>(data.secondaryStateInfo[3]);
  state.first_half = data.firstHalf;
  state.kicking_team = data.kickOffTeam;
  state.drop_in_team = data.dropInTeam;
  state.drop_in_time = (data.dropInTime == 0xFFFFu) ? -1 : static_cast<int32_t>(data.dropInTime);
  state.secs_remaining = static_cast<int32_t>(data.secsRemaining);
  state.secondary_time = static_cast<int32_t>(data.secondaryTime);

  for (size_t team_idx = 0; team_idx < state.teams.size(); ++team_idx) {
    const auto & src_team = data.teams[team_idx];
    auto & dst_team = state.teams[team_idx];

    dst_team.team_number = src_team.teamNumber;
    dst_team.field_player_colour = src_team.fieldPlayerColour;
    dst_team.score = src_team.score;
    dst_team.penalty_shot = src_team.penaltyShot;
    dst_team.single_shots = src_team.singleShots;
    dst_team.coach_sequence = src_team.coachSequence;
    std::copy(
      std::begin(src_team.coachMessage),
      std::end(src_team.coachMessage),
      dst_team.coach_message.begin());

    dst_team.coach.penalty = src_team.coach.penalty;
    dst_team.coach.secs_till_unpenalised = src_team.coach.secsTillUnpenalised;
    dst_team.coach.warnings = src_team.coach.numberOfWarnings;
    dst_team.coach.yellow_cards = src_team.coach.yellowCardCount;
    dst_team.coach.red_cards = src_team.coach.redCardCount;
    dst_team.coach.goalkeeper = src_team.coach.goalKeeper;

    for (size_t player_idx = 0; player_idx < dst_team.players.size(); ++player_idx) {
      const auto & src_player = src_team.players[player_idx];
      auto & dst_player = dst_team.players[player_idx];
      dst_player.penalty = src_player.penalty;
      dst_player.secs_till_unpenalised = src_player.secsTillUnpenalised;
      dst_player.warnings = src_player.numberOfWarnings;
      dst_player.yellow_cards = src_player.yellowCardCount;
      dst_player.red_cards = src_player.redCardCount;
      dst_player.goalkeeper = src_player.goalKeeper;
    }
  }

  return true;
}

void GCBridge::fallenCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  fallen_ = msg->data;
}

void GCBridge::hlMessageCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  hl_message_ = msg->data;
}

}  // namespace robotis_op

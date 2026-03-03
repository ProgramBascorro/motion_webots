#ifndef OP3_WEBOTS_ROS2_GC_BRIDGE_HPP
#define OP3_WEBOTS_ROS2_GC_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace robotis_op
{

class GCBridge : public rclcpp::Node
{
public:
  GCBridge();
  ~GCBridge() override;

private:
  static constexpr size_t HL_MAX_NUM_PLAYERS = 11;

  struct ParsedRobotInfo
  {
    uint8_t penalty = 0;
    uint8_t secs_till_unpenalised = 0;
    uint8_t warnings = 0;
    uint8_t yellow_cards = 0;
    uint8_t red_cards = 0;
    uint8_t goalkeeper = 0;
  };

  struct ParsedTeamInfo
  {
    uint8_t team_number = 0;
    uint8_t field_player_colour = 0;
    uint8_t score = 0;
    uint8_t penalty_shot = 0;
    uint16_t single_shots = 0;
    uint8_t coach_sequence = 0;
    std::array<uint8_t, 253> coach_message {};
    ParsedRobotInfo coach;
    std::array<ParsedRobotInfo, HL_MAX_NUM_PLAYERS> players;
  };

  struct ParsedState
  {
    uint8_t packet_number = 0;
    uint8_t players_per_team = 0;
    uint8_t game_type = 0;
    uint8_t state = 0;
    int32_t secondary = 0;
    std::array<uint8_t, 4> secondary_info {{0, 0, 0, 0}};
    uint8_t drop_in_team = 0;
    int32_t drop_in_time = 0;
    uint8_t kicking_team = 0;
    int32_t secs_remaining = 0;
    int32_t secondary_time = 0;
    uint8_t first_half = 0;
    std::array<ParsedTeamInfo, 2> teams;
  };

  void setupSockets();
  void closeSockets();
  void receiveControlPackets();
  void sendStatusPacket();
  void publishState(const ParsedState & state, const std::vector<uint8_t> & raw_packet);

  bool parseHlControl(const std::vector<uint8_t> & packet, ParsedState & state) const;

  void fallenCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void hlMessageCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  int control_socket_fd_;
  int status_socket_fd_;
  bool fallen_;
  uint8_t hl_message_;

  std::string gc_host_;
  uint16_t control_port_;
  uint16_t status_port_;
  uint8_t team_number_;
  uint8_t player_number_;

  std::array<uint8_t, 4> gc_ip_;

  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr tx_timer_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_json_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fallen_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr hl_message_sub_;
};

}  // namespace robotis_op

#endif  // OP3_WEBOTS_ROS2_GC_BRIDGE_HPP

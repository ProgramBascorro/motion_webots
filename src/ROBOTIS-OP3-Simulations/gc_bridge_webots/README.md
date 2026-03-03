# Webots GameController Bridge (Humanoid League)

## Purpose and Context

`gc_bridge_webots` is a ROS 2 bridge that receives RoboCup GameController UDP packets and republishes them as ROS topics for behavior nodes.

This bridge is **Humanoid League focused** and uses the official packet layout from:

- `include/gc_bridge_webots/game_controller/new_RoboCupGameControlData.h`

This module does **not** implement soccer strategy/decision logic. It only handles:

1. UDP receive from GameController
2. Struct-based packet parsing
3. ROS topic publication
4. UDP return/status packet send

## Module Layout

- `include/gc_bridge_webots/game_controller/new_RoboCupGameControlData.h`
  - Official Humanoid League packet structs/constants.
- `include/gc_bridge_webots/game_controller/SPLCoachMessage.h`
  - Coach message structure definitions.
- `include/gc_bridge_webots/gc_bridge.hpp`
  - `GCBridge` class interface, parsed-state model, publishers/subscribers.
- `src/gc_bridge.cpp`
  - UDP socket setup, RX/TX loop, struct parsing, topic publishing.
- `src/gc_bridge_main.cpp`
  - ROS 2 node entry point (`op3_gc_bridge`).

## Data Flow

1. Receive control packet via UDP `control_port` (default `3838`).
2. Parse bytes using `HlRoboCupGameControlData` (struct-based parsing).
3. Publish ROS topics:
   - `/game_controller/raw`
   - `/game_controller/state`
   - `/game_controller/state_json`
4. Send return packet to GameController via UDP `status_port` (default `3939`) using `HlRoboCupGameControlReturnData`.

```text
GameController (UDP 3838)
        |
        v
  op3_gc_bridge (parse HL struct)
        |
        +--> /game_controller/raw
        +--> /game_controller/state
        +--> /game_controller/state_json
        |
        v
Return packet (UDP 3939) --> GameController
```

## Protocol Compliance (HL-focused)

Current protocol mode:

- **Humanoid League only**

Key packet checks:

- Header must match `GAMECONTROLLER_STRUCT_HEADER`.
- Version must match `HL_GAMECONTROLLER_STRUCT_VERSION`.
- Packet size must match `sizeof(HlRoboCupGameControlData)`.

Key fields consumed:

- Primary state (`INITIAL`, `READY`, `SET`, `PLAYING`, `FINISHED`)
- Secondary state and secondary state info
- Kick-off team, remaining times
- Team score data
- Player penalty/card data (from HL team/player entries)

Return packet behavior:

- Sends `team`, `player`, and `message` in HL return format.
- `fallen` flag topic is accepted but not encoded in HL return packet format.

Out of scope:

- SPL-specific parsing/behavior in this bridge configuration.

## ROS Interface Contract

### Published by bridge

1. `/game_controller/raw`
- Type: `std_msgs/msg/UInt8MultiArray`
- Meaning: exact raw UDP bytes from GameController.
- Update behavior: only when valid HL packet is received.
- Use case: strict algorithm/parser input, debugging packet-level issues.

2. `/game_controller/state`
- Type: `std_msgs/msg/Int32MultiArray`
- Meaning: compact numeric state array for fast consumption.
- Current data order:
  - `[state, secondary, kicking_team, secs_remaining, secondary_time, team0_number, team0_score, team1_number, team1_score, packet_number, first_half]`
- Update behavior: on each valid HL packet.
- Use case: behavior state machine gating.

3. `/game_controller/state_json`
- Type: `std_msgs/msg/String`
- Meaning: readable JSON-like text containing parsed game/team/player values and raw hex payload.
- Update behavior: on each valid HL packet.
- Use case: observability/logging/debugging.

### Subscribed by bridge

4. `/game_controller/status/fallen`
- Type: `std_msgs/msg/Bool`
- Meaning: robot fallen status input to bridge.
- Note: stored internally; in HL mode it is not encoded into return packet.

5. `/game_controller/status/hl_message`
- Type: `std_msgs/msg/UInt8`
- Meaning: HL return message code sent back to GameController.
- Typical default in code: `2` (alive).

## Launch and Runtime Wiring

`op3_gc_bridge` is launched from:

- `src/ROBOTIS-OP3-Simulations/op3_webots_ros2/launch/robot_launch.py`

Default bridge parameters in launch:

- `gc_host: 127.0.0.1`
- `control_port: 3838`
- `status_port: 3939`
- `team_number: 1`
- `player_number: 1`
- `rx_period_ms: 20`
- `tx_period_ms: 500`

`robot_launch.py` also supports:

- `enable_robot_state_publisher:=true|false`

## End-to-End Runbook (WSL/Webots/Linux)

### Terminal 1 - Build and source workspace

```bash
cd ~/motion_webots
source /opt/ros/humble/setup.bash
colcon build --packages-select op3_webots_ros2
source ~/motion_webots/install/setup.bash
```

### Terminal 2 - Run Webots + extern controller + GC bridge

```bash
source /opt/ros/humble/setup.bash
source ~/motion_webots/install/setup.bash
ros2 launch op3_webots_ros2 robot_launch.py
```

Optional (disable TF publisher if needed):

```bash
ros2 launch op3_webots_ros2 robot_launch.py enable_robot_state_publisher:=false
```

### Terminal 3 - Run GameController binary

```bash
cd ~/motion_webots/game_controller-2025.1-x86_64-unknown-linux-gnu
./game_controller
```

### Terminal 4 - Verify bridge topics

```bash
source /opt/ros/humble/setup.bash
source ~/motion_webots/install/setup.bash
ros2 topic list | grep '^/game_controller'
```

Expected topics:

- `/game_controller/raw`
- `/game_controller/state`
- `/game_controller/state_json`
- `/game_controller/status/fallen`
- `/game_controller/status/hl_message`

### Terminal 4 - Inspect payload

```bash
ros2 topic echo /game_controller/state
ros2 topic echo /game_controller/state_json
ros2 topic hz /game_controller/state
```

### Terminal 4 - Send test status into bridge

```bash
ros2 topic pub /game_controller/status/hl_message std_msgs/msg/UInt8 "{data: 2}" -1
ros2 topic pub /game_controller/status/fallen std_msgs/msg/Bool "{data: false}" -1
```

## Validation Checklist

Pass if all points below are true:

1. `op3_gc_bridge` starts without crash.
2. `/game_controller/*` topics are visible.
3. Topic payload changes when GameController state changes (`INITIAL/READY/SET/PLAYING`).
4. No repeated UDP bind/send errors in bridge logs.
5. Behavior node can consume `/game_controller/state` or `/game_controller/raw` as input.

## Failure Modes and Troubleshooting

1. **Package not found after source**
- Symptom: `Package '...' not found`.
- Check:
  ```bash
  printenv | grep -E 'AMENT_PREFIX_PATH|COLCON_PREFIX_PATH'
  ```
- Fix: source only the intended workspace chain:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/motion_webots/install/setup.bash
  ```

2. **DDS/socket permission issues**
- Symptom: errors like `Operation not permitted` for UDP/SHM.
- Check where command is running (restricted sandbox/container vs normal shell).
- Fix: run from normal WSL shell with network/socket permissions.

3. **UDP port conflict (`3838`/`3939`)**
- Check:
  ```bash
  ss -lunp | grep -E ':3838|:3939'
  ```
- Fix: stop conflicting process or change bridge launch params.

4. **Wrong host/interface**
- Symptom: bridge runs but no packet received.
- Check `gc_host` and network route to GameController host.
- For same machine, keep `gc_host:=127.0.0.1`.

5. **Webots launched but bridge topics empty**
- Check bridge log line:
  - `GC bridge started (humanoid-only)...`
- Check GameController is running and actually sending game state.
- Check packet compatibility/version in use.

## Integration Notes for Behavior Algorithms

Recommended consumption strategy:

1. Use `/game_controller/state` for fast state machine transitions.
2. Use `/game_controller/raw` when strict packet-level parsing/control is required.
3. Use `/game_controller/state_json` for logging and operator debugging.

Guard behavior recommendation:

- If state topic is stale/invalid, fall back to safe robot mode (stop movement / hold posture).
- Treat unknown packet/state values as non-play condition until validated.

## Known Limitations / Non-Goals

- Humanoid League-only parsing path in this bridge mode.
- No built-in strategy layer (attack/defense/role logic).
- `fallen` input is currently not encoded into HL return packet payload.
- JSON-like topic is intended for readability/debugging, not as lowest-latency control channel.

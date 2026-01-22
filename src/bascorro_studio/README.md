# bascorro_studio

Studio UI and telemetry bridge for OP3 workflows (health, vision, tuning, and action pages).

## Quick start (dev)

1) Build and source the workspace:

```
colcon build --packages-select bascorro_studio
source install/setup.bash
```

2) Install rosbridge (required for browser ROS connection):

```
sudo apt install ros-humble-rosbridge-server
```

3) Start the stack (tmux):

```
./script.sh --studio
```

This launches:
- rosbridge websocket
- studio apply node (action pages)
- studio asset server (URDF + meshes)
- studio agent (metrics, events, bags, snapshots)
- bridge_webots for live preview
- Vite dev server

Open the URL printed by Vite (default: http://localhost:5173).

## Env overrides

- `OP3_STUDIO_PORT` (default: 5173)
- `OP3_STUDIO_ASSETS_PORT` (default: 8001)
- `OP3_STUDIO_ASSETS_DIR` (default: /tmp/bascorro_studio_assets)
- `OP3_STUDIO_WEB_DIR` (default: <ws>/src/bascorro_studio/web)
- `OP3_STUDIO_LOG_DIR` (default: /tmp/bascorro_studio_logs)
- `OP3_STUDIO_SKIP_ROSBRIDGE` (set to 1 to skip rosbridge)
- `OP3_STUDIO_SKIP_BRIDGE` (set to 1 to skip bridge_webots)
- `OP3_ROSBRIDGE_URL` (default: ws://localhost:9090)

Legacy `OP3_ACTION_WEB_*` vars are still supported.

## Topics

- `/bascorro_studio/metrics` (std_msgs/String JSON)
- `/bascorro_studio/events` (std_msgs/String JSON)
- `/bascorro_studio/request` (std_msgs/String JSON)
- `/bascorro_studio/result` (std_msgs/String JSON)

## Services

- `/bascorro_studio/snapshot` (std_srvs/Trigger)
- `/bascorro_studio/bag_start` (std_srvs/Trigger)
- `/bascorro_studio/bag_stop` (std_srvs/Trigger)

## Action requests

Example apply payload:

```
{"action": "apply", "yaml": "<yaml text>"}
```

Export:

```
{"action": "export", "pages": "used"}
```

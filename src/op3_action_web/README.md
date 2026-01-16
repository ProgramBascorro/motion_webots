# op3_action_web

Web UI for editing OP3 action pages with a live 3D preview.

## Quick start (dev)

1) Build and source the workspace:

```
colcon build --packages-select op3_action_web
source install/setup.bash
```

2) Install rosbridge (required for browser ROS connection):

```
sudo apt install ros-humble-rosbridge-server
```

3) Start the stack (tmux):

```
./script.sh --action-web
```

This launches:
- rosbridge websocket
- action web apply node
- action web asset server
- bridge_webots for live preview
- Vite dev server

Open the URL printed by Vite (default: http://localhost:5173).

## Env overrides

- `OP3_ACTION_WEB_PORT` (default: 5173)
- `OP3_ACTION_WEB_ASSETS_PORT` (default: 8001)
- `OP3_ACTION_WEB_ASSETS_DIR` (default: /tmp/op3_action_web_assets)
- `OP3_ACTION_WEB_DIR` (default: <ws>/src/op3_action_web/web)
- `OP3_ACTION_WEB_LOG_DIR` (default: /tmp/op3_action_web_logs)
- `OP3_ROSBRIDGE_URL` (default: ws://localhost:9090)
- `OP3_ACTION_WEB_SKIP_ROSBRIDGE` (set to 1 to skip rosbridge)
- `OP3_ACTION_WEB_SKIP_BRIDGE` (set to 1 to skip bridge_webots)

## Messages

- Publish apply requests to `/op3_action_web/request` (std_msgs/String).
- Results arrive on `/op3_action_web/result` (std_msgs/String JSON).

Example payload:

```
{"action": "apply", "yaml": "<yaml text>"}
```

Export:

```
{"action": "export", "pages": "used"}
```

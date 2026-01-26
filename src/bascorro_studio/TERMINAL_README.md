# Bascorro Studio - Web Terminal Integration

The Bascorro Studio now includes a **web-based terminal** that provides full shell access through your browser.

## Features

- ✨ **Browser-Based**: Access a full Linux terminal directly from the web UI
- 🔒 **Optional Authentication**: Password protection with `user:password` format
- 🚀 **Auto-Launch**: Starts automatically with `bascorro_studio`
- 🐳 **Docker Support**: Works in both Docker and native environments
- 🌐 **Remote Access**: Access your robot's terminal from anywhere on the network

## Quick Start

### 1. Install ttyd (Native Setup Only)

If running outside Docker, install `ttyd`:

```bash
# Automatic installation
./scripts/install_ttyd.sh

# Or manual installation (Ubuntu/Debian)
sudo apt-get install -y ttyd
```

For Docker, `ttyd` is already included in the image.

### 2. Launch Bascorro Studio

```bash
# Standard launch
./scripts/bascorro_studio.sh

# Or via script.sh
./script.sh
# Then select "Launch picker" → "bascorro_studio"
```

### 3. Access the Terminal

Open your browser and navigate to:
- **Web UI**: http://localhost:5173
- Click on the **"Terminal"** tab in the sidebar
- Or directly access: http://localhost:7681

## Configuration

### Environment Variables

Configure the terminal server with these environment variables:

```bash
# Terminal server port (default: 7681)
export OP3_STUDIO_TERMINAL_PORT=7681

# Authentication (format: "username:password")
export OP3_STUDIO_TERMINAL_CREDENTIAL="admin:secret"

# Terminal URL (for custom setups)
export OP3_STUDIO_TERMINAL_URL="http://localhost:7681"
```

### Enable Password Protection

For security, especially on robots accessible over the network:

```bash
# Set credential before launching
export OP3_STUDIO_TERMINAL_CREDENTIAL="op3:robot2024"
./scripts/bascorro_studio.sh
```

Then access the terminal with:
- Username: `op3`
- Password: `robot2024`

### Disable Terminal Server

To run bascorro_studio without the terminal:

```bash
# Temporarily disable
export OP3_STUDIO_SKIP_TERMINAL=1
./scripts/bascorro_studio.sh
```

Or comment out the `terminal_server` line in `scripts/bascorro_studio.sh`.

## ROS2 Integration

The terminal server runs as a ROS2 node: `bascorro_studio_terminal`

### Launch Parameters

```bash
# Via ros2 launch
ros2 launch bascorro_studio bascorro_studio.launch.py \
  terminal_port:=7681 \
  terminal_credential:="admin:password"

# Via ros2 run (standalone)
ros2 run bascorro_studio terminal_server \
  --ros-args \
  -p port:=7681 \
  -p credential:="admin:password"
```

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | int | 7681 | Web terminal server port |
| `credential` | string | "" | Authentication (user:pass or empty) |
| `max_clients` | int | 10 | Maximum concurrent connections |
| `writable` | bool | true | Allow terminal input |
| `check_origin` | bool | false | Enable CORS origin checking |

## Usage Examples

### Remote Robot Access

```bash
# On robot (OP3 NUC)
export OP3_STUDIO_TERMINAL_CREDENTIAL="op3:secure123"
./scripts/bascorro_studio.sh

# On your laptop
# Open browser to: http://192.168.1.100:5173
# Click "Terminal" tab
# Enter credentials when prompted
```

### Docker Environment

```bash
# Build with terminal support (already included)
./script.sh
# Select: "Docker build"

# Run container
./script.sh
# Select: "Docker run"

# Access terminal in container
# Browser: http://localhost:5173 → Terminal tab
```

### Multiple Terminal Sessions

The terminal server supports multiple concurrent users (default: 10 clients).

## Security Considerations

⚠️ **Important Security Notes**:

1. **Default = No Auth**: By default, the terminal is **NOT** password-protected
2. **Network Exposure**: If accessible over network, **always** set a credential
3. **HTTPS**: For production/remote use, consider running behind an HTTPS reverse proxy
4. **Firewall**: Restrict port 7681 to trusted networks

### Recommended Setup for Remote Access

```bash
# Strong password
export OP3_STUDIO_TERMINAL_CREDENTIAL="op3:$(openssl rand -base64 16)"

# Or use a fixed strong password
export OP3_STUDIO_TERMINAL_CREDENTIAL="op3:YourStrongPassword123!"

./scripts/bascorro_studio.sh
```

## Troubleshooting

### Terminal Not Loading

1. **Check if ttyd is installed**:
   ```bash
   which ttyd
   ttyd --version
   ```

2. **Check terminal server logs**:
   ```bash
   tail -f /tmp/bascorro_studio_logs/terminal_server.log
   ```

3. **Verify port is not in use**:
   ```bash
   sudo lsof -i :7681
   ```

### Permission Denied

If you see permission errors in the terminal:
- The terminal runs as the same user that launched `bascorro_studio`
- For Docker: runs as `root` by default
- For native: runs as your current user

### Connection Refused

1. **Check if the node is running**:
   ```bash
   ros2 node list | grep terminal
   ```

2. **Test direct access**:
   ```bash
   curl http://localhost:7681
   # Should return HTML content
   ```

3. **Firewall blocking**:
   ```bash
   # Allow port 7681
   sudo ufw allow 7681/tcp
   ```

## Advanced Usage

### Custom Shell

Edit `terminal_server.py` to change the default shell:

```python
# Use zsh instead of bash
cmd.append("zsh")

# Or use a custom command
cmd.extend(["bash", "-c", "cd /ros2_ws && bash"])
```

### Integration with tmux

For persistent sessions, use `ttyd` with `tmux`:

```bash
# Start terminal with tmux
ttyd -p 7681 tmux new -A -s robot
```

## Files Modified/Created

- `src/bascorro_studio/bascorro_studio/terminal_server.py` - ROS2 node
- `src/bascorro_studio/setup.py` - Added entry point
- `src/bascorro_studio/launch/bascorro_studio.launch.py` - Added node
- `src/bascorro_studio/web/src/App.jsx` - Added Terminal tab
- `scripts/bascorro_studio.sh` - Launch terminal server
- `scripts/install_ttyd.sh` - Installation script
- `Dockerfile` - Install ttyd in image

## References

- **ttyd**: https://github.com/tsl0922/ttyd
- **Bascorro Studio**: Main dashboard for OP3 robot telemetry and control

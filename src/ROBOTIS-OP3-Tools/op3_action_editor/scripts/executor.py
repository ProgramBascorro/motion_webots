#!/usr/bin/env python3

import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import atexit
import rclpy
import time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

class ActionEditorExecutor(Node):
    def __init__(self):
        super().__init__('op3_action_editor_executor')


def _can_open_serial(dev: str) -> bool:
    """True if the serial device can be opened right now (exists and not busy)."""
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(fd)
        return True
    except OSError:
        return False


def detect_dynamixel_device(preferred: str = '/dev/ttyUSB0') -> str:
    """Find the OpenCR/Dynamixel serial port regardless of its ttyUSB number.

    The FTDI/OpenCR adapter enumerates as /dev/ttyUSB0../dev/ttyUSB9 depending on
    plug order and what else is connected, and the udev symlink /dev/ttyOP3 is not
    recreated inside Docker. So scan the ttyUSB0-9 range and pick a usable port.
    Set OP3_DEVICE to force a specific port."""
    override = os.environ.get('OP3_DEVICE', '').strip()
    if override:
        return override

    candidates = [f'/dev/ttyUSB{i}' for i in range(10) if os.path.exists(f'/dev/ttyUSB{i}')]

    # Prefer a port we can actually open (skips ones held by e.g. DynamixelWizard).
    for dev in candidates:
        if _can_open_serial(dev):
            return dev

    # A stable udev symlink, if present, is the next best hint.
    if os.path.exists('/dev/ttyOP3'):
        return '/dev/ttyOP3'

    # Nothing free/open: fall back to first existing, else the preferred default so
    # the node still emits a clear "Error Set port" instead of silently guessing.
    return candidates[0] if candidates else preferred


def robot_file_with_device(src_robot_path: str, device: str) -> str:
    """Return a robot-config path whose port matches `device`.

    RobotisController opens the ports listed in OP3.robot, so those must point at the
    detected device too. Rather than mutate the version-controlled file, write a
    patched copy to a temp path and return it (falls back to the original on error)."""
    try:
        with open(src_robot_path, 'r') as f:
            content = f.read()
        patched = re.sub(r'/dev/tty[A-Za-z0-9_]+', device, content)
        if patched == content:
            return src_robot_path
        fd, tmp_path = tempfile.mkstemp(prefix='OP3_', suffix='.robot')
        with os.fdopen(fd, 'w') as f:
            f.write(patched)
        atexit.register(lambda: os.path.exists(tmp_path) and os.remove(tmp_path))
        return tmp_path
    except OSError as e:
        print(f"Could not patch robot file port ({e}); using original.", file=sys.stderr)
        return src_robot_path


def has_ros_package(package_name: str) -> bool:
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', package_name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def ensure_action_file(action_file_path: str, default_path: str) -> None:
    if os.path.isfile(action_file_path):
        return
    if not os.path.isfile(default_path):
        print(f"Default action file not found: {default_path}", file=sys.stderr)
        return
    os.makedirs(os.path.dirname(action_file_path), exist_ok=True)
    shutil.copyfile(default_path, action_file_path)

def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _descendant_pids(pid: int) -> list:
    """All descendants of `pid`, deepest first. Pure /proc, no psutil dependency."""
    children = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open('/proc/%s/stat' % entry) as handle:
                stat = handle.read()
        except OSError:
            continue
        # comm may contain spaces and parentheses; ppid is the second field after ')'.
        try:
            ppid = int(stat[stat.rindex(')') + 1:].split()[1])
        except (ValueError, IndexError):
            continue
        children.setdefault(ppid, []).append(int(entry))

    ordered = []

    def walk(parent: int) -> None:
        for child in children.get(parent, []):
            walk(child)
            ordered.append(child)

    walk(pid)
    return ordered


def terminate_tree(proc, grace: float = 2.0) -> None:
    """Stop `proc` and everything below it, escalating to SIGKILL.

    Two reasons this cannot be `proc.kill()`. First, `proc` is `ros2 run`, a thin
    python wrapper: killing it alone leaves the C++ op3_action_editor running, and
    that process keeps /dev/ttyUSBx open -- the next op3_manager then waits 30 s and
    gives up with "is in use by op3_action_edit(pid ...)". Second, op3_action_editor
    installs its own SIGINT/SIGTERM handler (main.cpp:103-106) that calls
    tcsetattr() and system("clear"); once the terminal is gone that handler hangs,
    so SIGTERM alone never finishes the job either.
    """
    if proc is None or proc.poll() is not None:
        return

    pids = _descendant_pids(proc.pid) + [proc.pid]
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass

    deadline = time.time() + grace
    while time.time() < deadline:
        if not any(_pid_alive(pid) for pid in pids):
            break
        time.sleep(0.1)

    for pid in pids:
        if _pid_alive(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass

    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass


def install_exit_signals() -> None:
    """Turn SIGHUP/SIGTERM into a normal exception so cleanup still runs.

    By default both kill python *immediately* -- the `finally` block that stops the
    editor never executes, and the editor is left alive holding /dev/ttyUSBx. That
    is the common path, not the rare one: the terminal or tmux pane going away
    sends SIGHUP, and only an explicit Ctrl-C sends SIGINT. SIGKILL still cannot be
    caught; `./script.sh --free-port` is the backstop for that.
    """
    def _raise(_signum, _frame):
        raise KeyboardInterrupt

    for sig in (signal.SIGHUP, signal.SIGTERM):
        try:
            signal.signal(sig, _raise)
        except (OSError, ValueError):
            pass


def resolve_action_file_default() -> str:
    """Default action file = the version-controlled workspace src copy that the op3
    stack and Bascorro Studio web read (via OP3_ACTION_FILE). Editing the same file
    keeps the editor and the running stack from drifting apart. Falls back to the
    installed share copy if the src tree is not available."""
    share_copy = get_package_share_directory('op3_action_module') + '/data/motion_4095_ORION.bin'
    try:
        ws_root = os.path.dirname(os.path.dirname(get_package_prefix('op3_action_module')))
        src_copy = os.path.join(ws_root, 'src/ROBOTIS-OP3/op3_action_module/data/motion_4095_ORION.bin')
        if os.path.isfile(src_copy):
            return src_copy
    except Exception:
        pass
    return share_copy


def main(args=None):
    rclpy.init(args=args)
    node = ActionEditorExecutor()

    # Define your package and executable
    package = "op3_action_editor"
    executable = "op3_action_editor"

    gazebo_default = False
    gazebo_robot_name_default = 'robotis_op3'

    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = resolve_action_file_default()
    action_file_path = os.environ.get('OP3_ACTION_FILE', '').strip() or action_file_path_default

    # Auto-detect the OpenCR/Dynamixel port (ttyUSB0-9) so it works whatever number
    # the adapter enumerated as, then make the robot config use that same port.
    device_name_default = detect_dynamixel_device()
    print(f"[executor] using Dynamixel device: {device_name_default}", file=sys.stderr)
    robot_file_path_default = robot_file_with_device(robot_file_path_default, device_name_default)

    if action_file_path != action_file_path_default:
        ensure_action_file(action_file_path, action_file_path_default)
    if not os.path.isfile(action_file_path):
        print(f"Action file not found: {action_file_path}", file=sys.stderr)
        return 1

    # Define any parameters or arguments
    params = [
        '--ros-args',
        '-p', f'gazebo:={gazebo_default}',
        '-p', f'gazebo_robot_name:={gazebo_robot_name_default}',
        '-p', f'offset_file_path:={offset_file_path_default}',
        '-p', f'robot_file_path:={robot_file_path_default}',
        '-p', f'init_file_path:={init_file_path_default}',
        '-p', f'action_file_path:={action_file_path}',
        '-p', f'device_name:={device_name_default}'
    ]

    proc_player = None
    if has_ros_package('ros_mpg321_player'):
        try:
            proc_player = subprocess.Popen(
                ['ros2', 'run', 'ros_mpg321_player', 'ros_mpg321_player'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            print(f"Failed to run ros_mpg321_player: {e}", file=sys.stderr)
    else:
        print("ros_mpg321_player is not installed; starting without sound support.", file=sys.stderr)

    try:
        # Run the node in the same terminal, and in the same process group, so the
        # editor keeps reading the keyboard and Ctrl-C still reaches it directly.
        proc_editor = subprocess.Popen(['ros2', 'run', package, executable] + params)
    except (OSError, subprocess.SubprocessError) as e:
        print(f"Error while running op3_action_editor: {e}")
        terminate_tree(proc_player)
        rclpy.shutdown()
        return 1

    # Whatever ends this wait -- the editor quitting on its own, Ctrl-C, or the
    # terminal going away -- the editor must not outlive us. An orphaned one holds
    # the Dynamixel bus (single master) and silently blocks every later
    # op3_manager and action editor, with no sign of it left in tmux.
    install_exit_signals()
    try:
        while proc_editor.poll() is None:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        terminate_tree(proc_player)
        terminate_tree(proc_editor)
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

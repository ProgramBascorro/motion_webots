#!/usr/bin/env python3

import os
import shutil
import stat
import subprocess
import sys
import tempfile
import termios
import rclpy
import time
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

class ActionEditorExecutor(Node):
    def __init__(self):
        super().__init__('op3_action_editor_executor')


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

def robot_file_without_sensors(source_path: str) -> str:
    """Salinan OP3.robot tanpa baris sensor (sub controller ID 200).

    Kenapa: robotis_controller melakukan bulk read ke SETIAP port pada tiap
    siklus kontrol 8 ms. Pada robot ini, papan OpenCR masih duduk di jalur TTL
    servo, jadi tiap kali ia menjawab lewat USB CDC ia ikut menahan jalur itu.
    Pada 8 ms ia menahannya nyaris tanpa jeda dan bus servo mati: 0 byte balasan
    dari 20 servo, dan paket sync-write goal_position tidak pernah diterapkan
    walau paketnya sah di kabel (CRC benar, terbaca di write() ke port).

    Gejalanya menyamar sebagai "play tidak sesuai data page": yang mendarat cuma
    sebagian acak dari 20 sendi -- kepala hampir tidak pernah -- sehingga sendi
    yang kebetulan sudah berada di pose page terlihat benar dan sisanya terlihat
    macet. Diukur: tanpa polling ID 200 => 20/20 sendi mendarat; dengan polling
    8 ms => 0 balasan dan hanya ~9/20 sendi mendarat, berpindah-pindah tiap coba.

    Editor tidak memerlukan IMU/tombol sama sekali. ID 200 tetap dipakai lewat
    parameter device_name untuk menyalakan daya DXL sekali di awal (main.cpp),
    dan itu satu transaksi, bukan polling -- jadi tidak merusak bus.

    Berkas dihasilkan ulang tiap kali jalan supaya tidak pernah basi terhadap
    OP3.robot. Kalau gagal, kembalikan berkas aslinya: lebih baik jalan dengan
    bus terganggu daripada tidak jalan sama sekali.
    """
    try:
        with open(source_path, 'r') as f:
            lines = f.read().splitlines()

        kept = [ln for ln in lines
                if ln.lstrip().startswith('#') or not ln.lstrip().startswith('sensor')]

        # Buang juga baris [ port info ] yang sudah tidak diacu perangkat mana pun,
        # supaya controller tidak membuka port yang tidak dipakai.
        used_ports = set()
        for ln in kept:
            if ln.lstrip().startswith('#'):
                continue
            fields = [x.strip() for x in ln.split('|')]
            if len(fields) >= 2 and fields[0] == 'dynamixel':
                used_ports.add(fields[1])

        result = []
        for ln in kept:
            stripped = ln.lstrip()
            if not stripped.startswith('#') and stripped.startswith('/dev/'):
                port = stripped.split('|')[0].strip()
                if port not in used_ports:
                    continue
            result.append(ln)

        out_path = os.path.join(tempfile.gettempdir(), 'OP3_action_editor.robot')
        with open(out_path, 'w') as f:
            f.write('\n'.join(result) + '\n')
        return out_path
    except Exception as e:
        print(f"Failed to strip sensors from {source_path}: {e}", file=sys.stderr)
        return source_path


SUB_CONTROLLER_ID = 200
FIRST_SERVO_ID = 1


def dxl_ping(dev: str, dxl_id: int, baud_flag: int = termios.B2000000) -> bool:
    """True if ``dxl_id`` answers a protocol 2.0 ping on ``dev``.

    Pure stdlib on purpose: this runs before the workspace is fully up and the
    container has no pyserial. Baud is nominal on a CDC port (the OpenCR bridge
    ignores it) and mandatory on an FTDI one, so 2 Mbps works for both.
    """
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id, 3, 0, 0x01])
    crc = 0
    for byte in body:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    packet = body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = baud_flag
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        os.write(fd, packet)
        reply = b''
        deadline = time.time() + 0.3
        while time.time() < deadline and len(reply) < 14:
            try:
                reply += os.read(fd, 64)
            except BlockingIOError:
                time.sleep(0.002)
        return len(reply) >= 14
    except (OSError, termios.error):
        return False
    finally:
        os.close(fd)


def _point_link(link_path: str, target: str) -> None:
    """Repoint ``link_path`` at ``target``, quietly and idempotently."""
    try:
        if link_path == target or os.path.realpath(link_path) == target:
            return
        if os.path.exists(link_path) and not os.path.islink(link_path):
            print(f'ensure_opencr_port: {link_path} is a real device node, not a '
                  f'symlink; leaving it alone (wanted {target})', file=sys.stderr)
            return
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.remove(link_path)
        os.symlink(target, link_path)
        print(f'ensure_opencr_port: {link_path} -> {target}', file=sys.stderr)
    except OSError as e:
        print(f'ensure_opencr_port: could not set {link_path}: {e}', file=sys.stderr)


def _usb_serial_ports() -> list:
    """Every USB serial tty the kernel actually has, adapters (ttyUSB*) first.

    Enumerated from sysfs rather than guessed: /sys/class/tty lists a name only
    once its driver is bound, and carries the real major:minor. Inside a container
    started with --device the node can be missing from /dev although the kernel has
    the tty, so recreate exactly that one -- we are root in a privileged container
    and device numbers are global. Nothing is ever created for a tty the kernel
    does not have, so no phantom nodes are left behind in the shared /dev.
    """
    ports = []
    for name in os.listdir('/sys/class/tty'):
        if not name.startswith(('ttyUSB', 'ttyACM')):
            continue
        try:
            with open(f'/sys/class/tty/{name}/dev') as handle:
                major, minor = (int(part) for part in handle.read().strip().split(':'))
        except (OSError, ValueError):
            continue
        dev = f'/dev/{name}'
        if not os.path.exists(dev):
            try:
                os.mknod(dev, 0o666 | stat.S_IFCHR, os.makedev(major, minor))
            except OSError:
                continue
        ports.append(dev)
    # Adapters before CDC ports, then in number order (length first, so ttyUSB2
    # stays ahead of ttyUSB10).
    ports.sort(key=lambda dev: (not dev.startswith('/dev/ttyUSB'), len(dev), dev))
    return ports


def _openable(dev: str) -> bool:
    """True if ``dev`` can be opened -- a node with no driver bound cannot."""
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    os.close(fd)
    return True


def ensure_opencr_port(servo_link: str = '/dev/ttyOP3',
                       sub_link: str = '/dev/ttyOPENCR') -> None:
    """Point the servo port name at whatever is actually plugged in now.

    OP3.robot references stable names, but a board that re-enumerates (unplug,
    re-flash, swapped adapter) lands on a different minor and the old node/symlink
    goes stale -> "Error opening serial port". So probe who answers and repoint the
    link. Idempotent; safe on every launch; degrades quietly off-hardware.

    Wiring on CHRONUS is split: servos on the U2D2, sub controller ID 200 (IMU,
    magnetometer, button, buzzer, DXL power) on the OpenCR micro-USB, because the
    firmware moves DXL_PORT from Serial3 to Serial. Two real ports, so both links
    are rewritten.

    Pick by probing, not by liveness -- who answers decides, so the same code keeps
    working on a board flashed with stock opencr_op3, which puts ID 200 back on the
    servo bus. That case is reported rather than guessed at: OP3.robot would have to
    give the OPEN-CR sensor the servo port again, and pointing both links at one tty
    would put two PortHandlers on a single fd.

    A port nobody answered on is never adopted. Pointing the link at "the first
    live tty" only moves the failure downstream, where it turns into twenty
    "JOINT[...] does NOT respond!!" lines instead of one plain sentence about the
    adapter being unplugged.
    """
    live = [dev for dev in _usb_serial_ports() if _openable(dev)]

    if not live:
        print(f'ensure_opencr_port: no live serial port found; leaving {servo_link} '
              f'and {sub_link} as-is', file=sys.stderr)
        return

    servo_port = next((dev for dev in live if dxl_ping(dev, FIRST_SERVO_ID)), None)
    sub_port = next((dev for dev in live if dxl_ping(dev, SUB_CONTROLLER_ID)), None)

    if servo_port is None:
        print(f'ensure_opencr_port: no servo answered on {", ".join(live)} -- the DXL '
              f'bus adapter (U2D2/FTDI, /dev/ttyUSB*) is not reachable, so {servo_link} '
              f'is left as it is. Check the U2D2 cable and that the robot is powered; '
              f'the OpenCR micro-USB alone cannot drive the servos.', file=sys.stderr)
    else:
        _point_link(servo_link, servo_port)

    if sub_port is None:
        print(f'ensure_opencr_port: OpenCR not answering (ID {SUB_CONTROLLER_ID} '
              f'silent on {", ".join(live)}) -- no IMU, no button. Check it is '
              f'powered and that its micro-USB is plugged in -- that is the only '
              f'route to ID 200 now.',
              file=sys.stderr)
    elif sub_port == servo_port:
        print(f'ensure_opencr_port: ID {SUB_CONTROLLER_ID} answers on the servo port '
              f'{sub_port} -- this board runs stock opencr_op3, which keeps ID 200 on '
              f'the TTL bus. OP3.robot in this workspace expects it on {sub_link}, so '
              f'the OPEN-CR sensor stays unreachable until the board is reflashed or '
              f'OP3.robot is pointed back at the servo port.', file=sys.stderr)
    elif sub_link == servo_link:
        print(f'ensure_opencr_port: ID {SUB_CONTROLLER_ID} answers on {sub_port}, not on '
              f'the servo port {servo_port}, but both links are configured as the same '
              f'name -- pointing them at one tty would put two PortHandlers on a single '
              f'fd. Give the OPEN-CR sensor line in OP3.robot its own port.',
              file=sys.stderr)
    else:
        # Decided on its own, never gated on the servo bus: two separate ports now,
        # so an unpowered servo bus is no reason to leave the OpenCR link unset --
        # that is exactly the bench state used to tune the magnetometer.
        if servo_port is None:
            print(f'ensure_opencr_port: ID {SUB_CONTROLLER_ID} answers on {sub_port} -- '
                  f'the OpenCR is up, the servo bus is not.', file=sys.stderr)
        print(f'ensure_opencr_port: OpenCR found on {sub_port} '
              f'(ID {SUB_CONTROLLER_ID}) -- IMU available', file=sys.stderr)
        _point_link(sub_link, sub_port)


def resolve_action_file_default() -> str:
    """Default action file = the version-controlled workspace src copy that the op3
    stack and Bascorro Studio web read (via OP3_ACTION_FILE). Editing the same file
    keeps the editor and the running stack from drifting apart. Falls back to the
    installed share copy if the src tree is not available."""
    share_copy = get_package_share_directory('op3_action_module') + '/data/motion_4095_CHRONUS.bin'
    try:
        ws_root = os.path.dirname(os.path.dirname(get_package_prefix('op3_action_module')))
        src_copy = os.path.join(ws_root, 'src/ROBOTIS-OP3/op3_action_module/data/motion_4095_CHRONUS.bin')
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
    robot_file_path_default = robot_file_without_sensors(
        get_package_share_directory('op3_manager') + '/config/OP3.robot')
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = resolve_action_file_default()
    action_file_path = os.environ.get('OP3_ACTION_FILE', '').strip() or action_file_path_default
    # device_name is only the port used to power on sub controller ID 200; the servos
    # come from OP3.robot. The buses are split on this board -- the firmware serves ID
    # 200 over the OpenCR micro-USB CDC, not the TTL bus -- so this has to be the
    # OpenCR port. Pointing it at the servo port makes the power-on talk to a bus where
    # ID 200 is silent, and the editor reports the OpenCR as missing even though it is
    # plugged in and answering. Same default as SUB_CONTROLLER_DEVICE in main.cpp;
    # ensure_opencr_port() just below repoints the link at whoever answers now.
    device_name_default = '/dev/ttyOPENCR'
    if not gazebo_default:
        ensure_opencr_port()

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
        # Run the node in the same terminal
        proc_editor = subprocess.Popen(['ros2', 'run', package, executable] + params)
    except subprocess.CalledProcessError as e:
        print(f"Error while running op3_action_editor: {e}")
        if proc_player is not None and proc_player.poll() is None:
            proc_player.kill()
        return 1

    while True:
        if proc_editor.poll() is not None:
            break

        time.sleep(1)

    if proc_player is not None and proc_player.poll() is None:
        proc_player.kill()

    if proc_editor.poll() is None:
        proc_editor.kill()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

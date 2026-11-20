import os
import stat
import struct
import sys
import termios
import time

import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

SUB_CONTROLLER_ID = 200
FIRST_SERVO_ID = 1


def _dxl_answers(dev, dxl_id):
    """True if ``dxl_id`` replies to a protocol 2.0 ping on ``dev``."""
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id]) + struct.pack('<H', 3) + bytes([0x01])
    crc = 0
    for byte in body:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    packet = body + struct.pack('<H', crc)
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[4] = attrs[5] = termios.B2000000
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


def _usb_serial_ports():
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
            with open('/sys/class/tty/%s/dev' % name) as handle:
                major, minor = (int(part) for part in handle.read().strip().split(':'))
        except (OSError, ValueError):
            continue
        dev = '/dev/%s' % name
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


def _openable(dev):
    """True if ``dev`` can be opened -- a node with no driver bound cannot."""
    try:
        fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    os.close(fd)
    return True


def ensure_ports(servo_link='/dev/ttyOP3', sub_link='/dev/ttyOPENCR'):
    """Resolve the port names by probing who actually answers.

    Mirrors ensure_opencr_port() in op3_action_editor's executor.py -- the manager
    needs the same treatment because it is usually the first thing started. A board
    that re-enumerates lands on a different minor, so the stable name has to follow
    the port that answers. Idempotent; quiet off-hardware.

    Wiring here is split: the servos answer on the U2D2 and sub controller ID 200
    answers on the OpenCR micro-USB, because the firmware moves DXL_PORT from
    Serial3 to Serial. So the two links are genuinely two ports and both get
    repointed. Probing rather than trusting the names matters more now, not less:
    a CDC port re-enumerates on every board reset, so /dev/ttyACM0 becomes ACM1
    behind an unchanged udev symlink more often than the U2D2 ever moves.

    A port nobody answered on is never adopted -- adopting one only turns a missing
    adapter into twenty "JOINT[...] does NOT respond!!" lines further downstream.
    """
    live = [dev for dev in _usb_serial_ports() if _openable(dev)]

    if not live:
        print('op3_manager.launch: no live serial port found', file=sys.stderr)
        return

    servo_port = next((d for d in live if _dxl_answers(d, FIRST_SERVO_ID)), None)
    sub_port = next((d for d in live if _dxl_answers(d, SUB_CONTROLLER_ID)), None)

    def point(link, target):
        try:
            if link == target or os.path.realpath(link) == target:
                return
            if os.path.exists(link) and not os.path.islink(link):
                print('op3_manager.launch: %s is a real device node, not a symlink; '
                      'leaving it alone (wanted %s)' % (link, target), file=sys.stderr)
                return
            if os.path.islink(link) or os.path.exists(link):
                os.remove(link)
            os.symlink(target, link)
            print('op3_manager.launch: %s -> %s' % (link, target), file=sys.stderr)
        except OSError as e:
            print('op3_manager.launch: could not set %s: %s' % (link, e), file=sys.stderr)

    if servo_port is None:
        print('op3_manager.launch: no servo answered on %s -- the DXL bus adapter '
              '(U2D2/FTDI, /dev/ttyUSB*) is not reachable, so %s is left as it is. '
              'Check the U2D2 cable and that the robot is powered; the OpenCR '
              'micro-USB alone cannot drive the servos.'
              % (', '.join(live), servo_link), file=sys.stderr)
    else:
        point(servo_link, servo_port)

    if sub_port is None:
        print('op3_manager.launch: ID %d silent on %s -- no IMU, no button'
              % (SUB_CONTROLLER_ID, ', '.join(live)), file=sys.stderr)
    elif sub_port == servo_port:
        # One port answering as both servo and ID 200 means the board is running
        # stock firmware (ID 200 back on the TTL bus). Say so plainly: OP3.robot
        # here expects two ports, and the IMU would silently never appear.
        print('op3_manager.launch: ID %d answers on the servo port %s -- this board '
              'runs stock opencr_op3, which keeps ID 200 on the TTL bus. OP3.robot '
              'in this workspace expects it on %s instead, so the OPEN-CR sensor '
              'will not be reachable until the board is reflashed or OP3.robot is '
              'pointed back at the servo port.'
              % (SUB_CONTROLLER_ID, servo_port, sub_link), file=sys.stderr)
    elif sub_link == servo_link:
        print('op3_manager.launch: ID %d answers on %s, not on the servo port %s, but '
              'both links are configured as the same name -- pointing them at one tty '
              'would put two PortHandlers on a single fd. The OPEN-CR sensor line in '
              'OP3.robot needs its own port.'
              % (SUB_CONTROLLER_ID, sub_port, servo_port), file=sys.stderr)
    else:
        # Decided on its own, never gated on the servo bus: two separate ports now,
        # so an unpowered servo bus is no reason to leave the OpenCR link unset --
        # that is exactly the bench state used to tune the magnetometer.
        if servo_port is None:
            print('op3_manager.launch: ID %d answers on %s -- the OpenCR is up, the '
                  'servo bus is not. IMU and magnetometer will work; joints will not.'
                  % (SUB_CONTROLLER_ID, sub_port), file=sys.stderr)
        point(sub_link, sub_port)



def _running_managers():
    """PID dan baris perintah tiap op3_manager yang sudah jalan, selain proses ini.

    Dicocokkan pada argv[0] SAJA, dan itu disengaja. Dua cara yang lebih naluriah
    dua-duanya salah di sini:

      * `pgrep -x op3_manager` tidak pernah cocok -- nama proses di /proc/<pid>/stat
        dipotong 15 karakter, dan "op3_manager" pas 11 tapi peluncurnya bukan.
      * mencari "op3_manager" di SELURUH baris perintah lebih buruk lagi: action
        editor membawa argumen /install/op3_manager/share/op3_manager/config/...,
        jadi pola longgar akan menemukan editor operator dan melaporkannya sebagai
        manager. (Persis jebakan itu pernah membunuh dua editor yang sedang dipakai.)

    argv[0] milik manager selalu berakhir "/op3_manager"; peluncur ros2 launch
    argv[0]-nya python3, dan editor argv[0]-nya op3_action_editor. Bersih.
    """
    me = os.getpid()
    found = []
    try:
        entries = os.listdir('/proc')
    except OSError:
        return found                      # bukan Linux / tanpa procfs: lewati saja
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open('/proc/%s/cmdline' % entry, 'rb') as handle:
                raw = handle.read()
        except OSError:
            continue                      # proses keburu keluar; bukan masalah
        if not raw:
            continue                      # kernel thread
        argv0 = raw.split(b'\0', 1)[0].decode('utf-8', 'replace')
        if os.path.basename(argv0) == 'op3_manager':
            cmd = raw.replace(b'\0', b' ').decode('utf-8', 'replace').strip()
            found.append((pid, cmd))
    return found


def refuse_if_manager_running():
    """Berhenti kalau sudah ada op3_manager lain -- JANGAN biarkan yang kedua jalan.

    Dua manager membuka /dev/ttyOP3 dan /dev/ttyOPENCR yang sama. Linux tidak
    mengunci tty secara eksklusif, jadi keduanya berhasil membuka dan tidak ada
    error apa pun; yang kedua lalu membaca potongan balasan yang ditujukan untuk
    yang pertama, panjangnya tidak sesuai dugaan, dan prosesnya mati dengan
    "*** stack smashing detected ***" plus exit -6 -- tanpa satu pun petunjuk ke
    penyebab sebenarnya. Terjadi dua kali pada 2026-08-28.

    Dipanggil SEBELUM ensure_ports(), bukan sesudah: ensure_ports() menembakkan
    paket ping ke bus, dan melakukan itu di atas bus yang sedang dipakai manager
    lain justru merusak lalu lintasnya.
    """
    others = _running_managers()
    if not others:
        return
    print('', file=sys.stderr)
    print('=' * 72, file=sys.stderr)
    print('op3_manager.launch: SUDAH ADA op3_manager LAIN YANG JALAN -- dibatalkan.',
          file=sys.stderr)
    print('', file=sys.stderr)
    for pid, cmd in others:
        print('    PID %-7d %s' % (pid, cmd[:100]), file=sys.stderr)
    print('', file=sys.stderr)
    print('Dua manager memakai /dev/ttyOP3 dan /dev/ttyOPENCR yang sama. Yang kedua',
          file=sys.stderr)
    print('akan mati dengan "*** stack smashing detected ***" (exit -6), dan pesan',
          file=sys.stderr)
    print('itu sama sekali tidak menyebut penyebabnya. Hentikan yang lama dulu:',
          file=sys.stderr)
    print('', file=sys.stderr)
    print('    kill %s' % ' '.join(str(pid) for pid, _ in others), file=sys.stderr)
    print('    ps -eo pid,cmd | grep op3_manager      # pastikan sudah kosong',
          file=sys.stderr)
    print('', file=sys.stderr)
    print('Kalau memang sengaja mau menjalankan dua (misal simulasi di domain lain),',
          file=sys.stderr)
    print('setel OP3_ALLOW_MULTIPLE_MANAGERS=1.', file=sys.stderr)
    print('=' * 72, file=sys.stderr)
    print('', file=sys.stderr)
    sys.exit(1)


def generate_launch_description():
    if os.environ.get('OP3_ALLOW_MULTIPLE_MANAGERS') != '1':
        refuse_if_manager_running()
    ensure_ports()

    gazebo_default = False
    gazebo_robot_name_default = 'robotis_op3'
    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    # Same names OP3.robot uses. Two parameters, two ports: device_name is the
    # servo port (the startup torque check reads joint ID 1 through it), and the
    # sub controller name is the OpenCR micro-USB, where ID 200 now answers -- the
    # DXL power-on and RGB LED writes follow it there.
    device_name_default = '/dev/ttyOP3'
    sub_controller_device_name_default = '/dev/ttyOPENCR'

    return LaunchDescription([
        Node(
            package='op3_manager',
            executable='op3_manager',
            # name='op3_manager',
            output='screen',
            parameters=[{
                # Kecepatan rata-rata kepala (deg/s) untuk head_control_module.
                # Berkas parameter launch memakai wildcard /**, jadi nilai ini
                # sampai ke node op3_head_control_module walau dipasang di sini.
                # 30 diminta operator 2026-09-13 (sebelumnya 42 sejak 2026-09-11,
                # awalnya 30).
                'angle_unit': 30.0,
                # Kecepatan kepala ke POSISI TETAP (scan/searching bola), terpisah
                # dari angle_unit di atas yang tetap dipakai tracking. Operator
                # 2026-09-19: scan terlalu lama -> 60 deg/s (sapuan kiri-kanan
                # penuh 170 derajat: ~2,8 dtk, dulu ~5,7 dtk pada 30).
                'angle_unit_scan': 60.0,
                'gazebo': gazebo_default,
                'gazebo_robot_name': gazebo_robot_name_default,
                'offset_file_path': offset_file_path_default,
                'robot_file_path': robot_file_path_default,
                'init_file_path': init_file_path_default,
                'device_name': device_name_default,
                'sub_controller_device_name': sub_controller_device_name_default
            },
                # Tabel kalibrasi yaw IMU 5 titik (scripts/calib_imu_yaw.py).
                # Sesudah dict di atas; berkunci /** supaya sampai ke node
                # open_cr_module. Lewat share (symlink ke src pada
                # --symlink-install), jadi hasil kalibrasi baru berlaku tanpa build.
                os.path.join(get_package_share_directory('op3_manager'), 'config',
                             'yaw_calibration.yaml'),
                # Kalibrasi drift (bias) gyro IMU -- scripts/calib_imu_drift.py.
                os.path.join(get_package_share_directory('op3_manager'), 'config',
                             'imu_drift.yaml')]
        )
        # Node(
        #     package='op3_localization',
        #     executable='op3_localization',
        #     name='op3_localization',
        #     output='screen'
        # )
    ])

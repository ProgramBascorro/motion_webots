#!/usr/bin/env python3

import argparse
import os
import shutil
import struct
import sys
import time
from typing import Dict, List, Tuple

try:
    import yaml
except ImportError:
    yaml = None

from ament_index_python.packages import get_package_share_directory

MAXNUM_PAGE = 256
MAXNUM_STEP = 7
MAXNUM_JOINTS = 31
PAGE_SIZE = 512
HEADER_SIZE = 64
STEP_SIZE = 64

INVALID_BIT_MASK = 0x4000
TORQUE_OFF_BIT_MASK = 0x2000

SCHEDULE_TO_NAME = {
    0x00: "speed",
    0x0A: "time",
}
NAME_TO_SCHEDULE = {
    "speed": 0x00,
    "time": 0x0A,
}


def require_yaml() -> None:
    if yaml is None:
        print("PyYAML is required. Install with: sudo apt install python3-yaml", file=sys.stderr)
        raise SystemExit(1)


def get_default_action_file() -> str:
    env_path = os.environ.get("OP3_ACTION_FILE", "").strip()
    if env_path:
        return env_path
    return get_package_share_directory("op3_action_module") + "/data/motion_4095_ORION.bin"


def get_default_robot_file() -> str:
    return get_package_share_directory("op3_manager") + "/config/OP3.robot"


def load_joint_map(robot_file_path: str) -> Tuple[Dict[int, str], Dict[str, int]]:
    id_to_name: Dict[int, str] = {}
    name_to_id: Dict[str, int] = {}

    if not os.path.isfile(robot_file_path):
        return id_to_name, name_to_id

    with open(robot_file_path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#") or "|" not in line:
                continue
            parts = [p.strip() for p in line.split("|")]
            if len(parts) < 6:
                continue
            if parts[0] != "dynamixel":
                continue
            try:
                joint_id = int(parts[2])
            except ValueError:
                continue
            joint_name = parts[5]
            id_to_name[joint_id] = joint_name
            name_to_id[joint_name] = joint_id
    return id_to_name, name_to_id


def load_action_bytes(path: str) -> bytearray:
    with open(path, "rb") as handle:
        data = bytearray(handle.read())
    expected = PAGE_SIZE * MAXNUM_PAGE
    if len(data) != expected:
        raise ValueError(f"Invalid action file size: {len(data)} (expected {expected})")
    return data


def write_action_bytes(path: str, data: bytearray, backup: bool) -> None:
    if backup and os.path.isfile(path):
        ts = time.strftime("%Y%m%d_%H%M%S")
        backup_path = f"{path}.bak.{ts}"
        shutil.copyfile(path, backup_path)
        print(f"Backup created: {backup_path}")
    with open(path, "wb") as handle:
        handle.write(data)


def decode_schedule(value: int):
    return SCHEDULE_TO_NAME.get(value, value)


def encode_schedule(value) -> int:
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        key = value.strip().lower()
        if key in NAME_TO_SCHEDULE:
            return NAME_TO_SCHEDULE[key]
        if key.startswith("0x"):
            return int(key, 16)
    raise ValueError(f"Invalid schedule value: {value}")


def decode_position(value: int):
    if value & INVALID_BIT_MASK:
        return None
    if value & TORQUE_OFF_BIT_MASK:
        return "torque_off"
    return int(value)


def encode_position(value):
    if value is None:
        return INVALID_BIT_MASK
    if isinstance(value, str):
        key = value.strip().lower()
        if key in ("invalid", "none", "null", "~"):
            return INVALID_BIT_MASK
        if key in ("torque_off", "off", "torque"):
            return TORQUE_OFF_BIT_MASK
        if key.startswith("0x"):
            return int(key, 16)
    if isinstance(value, (int, float)):
        ivalue = int(value)
        if 0 <= ivalue <= 0xFFFF:
            return ivalue
    raise ValueError(f"Invalid position value: {value}")


def decode_pgain(value: int) -> int:
    return int(value)


def encode_pgain(value) -> int:
    if not isinstance(value, (int, float)):
        raise ValueError(f"Invalid pgain value: {value}")
    ivalue = int(value)
    if 0 <= ivalue <= 255:
        return ivalue
    raise ValueError(f"Invalid pgain value: {value}")


def page_offset(page_index: int) -> int:
    return page_index * PAGE_SIZE


def step_offset(step_index: int) -> int:
    return HEADER_SIZE + (step_index * STEP_SIZE)


def read_header(page_bytes: bytearray) -> Dict[str, object]:
    name_raw = bytes(page_bytes[0:14])
    name = name_raw.split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    header = {
        "name": name,
        "repeat": page_bytes[15],
        "schedule": decode_schedule(page_bytes[16]),
        "stepnum": page_bytes[20],
        "speed": page_bytes[22],
        "accel": page_bytes[24],
        "next": page_bytes[25],
        "exit": page_bytes[26],
        "pgain_raw": list(page_bytes[32:63]),
    }
    return header


def write_header(page_bytes: bytearray, header: Dict[str, object], id_to_name: Dict[int, str]) -> None:
    if "name" in header:
        name = str(header["name"])
        encoded = name.encode("ascii", errors="ignore")[:13]
        page_bytes[0:14] = b"\x00" * 14
        page_bytes[0:len(encoded)] = encoded
    if "repeat" in header:
        page_bytes[15] = int(header["repeat"]) & 0xFF
    if "schedule" in header:
        page_bytes[16] = encode_schedule(header["schedule"]) & 0xFF
    if "stepnum" in header:
        page_bytes[20] = int(header["stepnum"]) & 0xFF
    if "speed" in header:
        page_bytes[22] = int(header["speed"]) & 0xFF
    if "accel" in header:
        page_bytes[24] = int(header["accel"]) & 0xFF
    if "next" in header:
        page_bytes[25] = int(header["next"]) & 0xFF
    if "exit" in header:
        page_bytes[26] = int(header["exit"]) & 0xFF
    if "pgain" in header:
        pgain_values = pgain_to_list(header["pgain"], id_to_name)
        for idx, value in enumerate(pgain_values):
            page_bytes[32 + idx] = value & 0xFF


def pgain_to_list(pgain, id_to_name: Dict[int, str]) -> List[int]:
    values = [0] * MAXNUM_JOINTS
    if isinstance(pgain, list):
        for idx in range(min(len(pgain), MAXNUM_JOINTS)):
            values[idx] = encode_pgain(pgain[idx])
        return values
    if isinstance(pgain, dict):
        for key, value in pgain.items():
            idx = resolve_joint_id(key, id_to_name)
            if idx is None or idx >= MAXNUM_JOINTS:
                continue
            values[idx] = encode_pgain(value)
        return values
    raise ValueError("pgain must be a list or map")


def resolve_joint_id(key, id_to_name: Dict[int, str]) -> int:
    if isinstance(key, int):
        return key
    if not isinstance(key, str):
        raise ValueError(f"Invalid joint key: {key}")
    key = key.strip()
    if key.isdigit():
        return int(key)
    if key.startswith("id_") and key[3:].isdigit():
        return int(key[3:])
    if key in id_to_name.values():
        for joint_id, name in id_to_name.items():
            if name == key:
                return joint_id
    raise ValueError(f"Unknown joint key: {key}")


def read_step(page_bytes: bytearray, step_index: int) -> Tuple[List[int], int, int]:
    offset = step_offset(step_index)
    values = struct.unpack_from("<31HBB", page_bytes, offset)
    positions = list(values[:MAXNUM_JOINTS])
    pause = values[MAXNUM_JOINTS]
    time_value = values[MAXNUM_JOINTS + 1]
    return positions, pause, time_value


def write_step(page_bytes: bytearray, step_index: int, positions: List[int], pause: int, time_value: int) -> None:
    offset = step_offset(step_index)
    packed = struct.pack(
        "<31HBB",
        *positions,
        pause & 0xFF,
        time_value & 0xFF,
    )
    page_bytes[offset : offset + STEP_SIZE] = packed


def positions_to_dict(positions: List[int], id_to_name: Dict[int, str]) -> Dict[str, object]:
    data: Dict[str, object] = {}
    for joint_id in range(1, MAXNUM_JOINTS):
        key = id_to_name.get(joint_id, f"id_{joint_id}")
        data[key] = decode_position(positions[joint_id])
    return data


def pgain_to_dict(values: List[int], id_to_name: Dict[int, str]) -> Dict[str, int]:
    data: Dict[str, int] = {}
    for joint_id in range(1, MAXNUM_JOINTS):
        key = id_to_name.get(joint_id, f"id_{joint_id}")
        data[key] = decode_pgain(values[joint_id])
    return data


def update_positions(
    positions: List[int],
    updates,
    id_to_name: Dict[int, str],
) -> List[int]:
    updated = list(positions)
    if isinstance(updates, list):
        for idx in range(min(len(updates), MAXNUM_JOINTS)):
            updated[idx] = encode_position(updates[idx])
        return updated
    if isinstance(updates, dict):
        for key, value in updates.items():
            joint_id = resolve_joint_id(key, id_to_name)
            if joint_id is None or joint_id >= MAXNUM_JOINTS:
                continue
            updated[joint_id] = encode_position(value)
        return updated
    raise ValueError("positions must be a list or map")


def update_checksum(page_bytes: bytearray) -> None:
    page_bytes[31] = 0
    checksum = (0xFF - (sum(page_bytes) & 0xFF)) & 0xFF
    page_bytes[31] = checksum


def export_pages(args) -> int:
    require_yaml()
    action_file = args.action_file or get_default_action_file()
    robot_file = args.robot_file or get_default_robot_file()

    id_to_name, _ = load_joint_map(robot_file)
    data = load_action_bytes(action_file)

    pages = []

    for page_index in range(MAXNUM_PAGE):
        offset = page_offset(page_index)
        page_bytes = data[offset : offset + PAGE_SIZE]
        header = read_header(page_bytes)
        used = bool(header["name"]) or int(header["stepnum"]) > 0
        if args.pages == "used" and not used:
            continue
        if args.pages == "all":
            pass
        elif args.pages not in ("all", "used"):
            if page_index not in args.page_list:
                continue

        page_entry = {
            "index": page_index,
            "name": header["name"],
            "header": {
                "repeat": header["repeat"],
                "schedule": header["schedule"],
                "stepnum": header["stepnum"],
                "speed": header["speed"],
                "accel": header["accel"],
                "next": header["next"],
                "exit": header["exit"],
                "pgain": pgain_to_dict(header["pgain_raw"], id_to_name),
            },
            "steps": [],
        }

        stepnum = int(header["stepnum"])
        for step_index in range(min(stepnum, MAXNUM_STEP)):
            positions, pause, time_value = read_step(page_bytes, step_index)
            page_entry["steps"].append(
                {
                    "index": step_index,
                    "pause": pause,
                    "time": time_value,
                    "positions": positions_to_dict(positions, id_to_name),
                }
            )

        pages.append(page_entry)

    meta = {
        "action_file": action_file,
        "robot_file": robot_file,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "joint_order": [
            {"id": joint_id, "name": id_to_name[joint_id]}
            for joint_id in sorted(id_to_name.keys())
            if 0 < joint_id < MAXNUM_JOINTS
        ],
    }

    payload = {"meta": meta, "pages": pages}
    with open(args.out, "w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)
    print(f"Wrote {len(pages)} page(s) to {args.out}")
    return 0


def import_pages(args) -> int:
    require_yaml()
    action_file = args.action_file or get_default_action_file()
    robot_file = args.robot_file or get_default_robot_file()

    id_to_name, _ = load_joint_map(robot_file)
    data = load_action_bytes(action_file)

    with open(args.input, "r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}

    pages = payload.get("pages", [])
    if not pages:
        print("No pages found in YAML.", file=sys.stderr)
        return 1

    for page in pages:
        if "index" not in page:
            raise ValueError("Each page must include an index.")
        page_index = int(page["index"])
        if page_index < 0 or page_index >= MAXNUM_PAGE:
            raise ValueError(f"Invalid page index: {page_index}")

        offset = page_offset(page_index)
        page_bytes = data[offset : offset + PAGE_SIZE]

        header_updates = page.get("header", {})
        if "name" in page:
            header_updates = dict(header_updates)
            header_updates["name"] = page["name"]

        if header_updates:
            write_header(page_bytes, header_updates, id_to_name)

        max_step_index = -1
        steps = page.get("steps", [])
        for default_index, step in enumerate(steps):
            step_index = int(step.get("index", default_index))
            if step_index < 0 or step_index >= MAXNUM_STEP:
                raise ValueError(f"Invalid step index: {step_index}")
            max_step_index = max(max_step_index, step_index)

            positions, pause, time_value = read_step(page_bytes, step_index)
            if "pause" in step:
                pause = int(step["pause"])
            if "time" in step:
                time_value = int(step["time"])
            if "positions" in step:
                positions = update_positions(positions, step["positions"], id_to_name)
            write_step(page_bytes, step_index, positions, pause, time_value)

        if "stepnum" not in header_updates and steps:
            page_bytes[20] = max_step_index + 1

        update_checksum(page_bytes)
        data[offset : offset + PAGE_SIZE] = page_bytes

    if args.dry_run:
        print("Dry run: no file written.")
        return 0

    write_action_bytes(action_file, data, backup=not args.no_backup)
    print(f"Updated action file: {action_file}")
    return 0


def parse_pages_arg(pages: str) -> Tuple[str, List[int]]:
    if pages in ("all", "used"):
        return pages, []
    result = set()
    for part in pages.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start = int(start_s)
            end = int(end_s)
            for idx in range(start, end + 1):
                result.add(idx)
        else:
            result.add(int(part))
    return "list", sorted(result)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export/import OP3 action pages as YAML.")
    parser.add_argument("--action-file", help="Path to OP3 action bin (defaults to OP3_ACTION_FILE or motion_4095_ORION.bin)")
    parser.add_argument("--robot-file", help="Path to OP3.robot for joint mapping")

    subparsers = parser.add_subparsers(dest="command", required=True)

    export_parser = subparsers.add_parser("export", help="Export pages to YAML")
    export_parser.add_argument("--out", required=True, help="Output YAML file")
    export_parser.add_argument(
        "--pages",
        default="used",
        help="Pages to export: used | all | comma list (e.g. 120,121 or 120-125)",
    )

    import_parser = subparsers.add_parser("import", help="Import pages from YAML")
    import_parser.add_argument("--in", dest="input", required=True, help="Input YAML file")
    import_parser.add_argument("--no-backup", action="store_true", help="Skip action file backup")
    import_parser.add_argument("--dry-run", action="store_true", help="Validate without writing")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "export":
        mode, page_list = parse_pages_arg(args.pages)
        args.pages = "used" if mode == "used" else "all" if mode == "all" else "list"
        args.page_list = page_list
        return export_pages(args)
    if args.command == "import":
        return import_pages(args)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

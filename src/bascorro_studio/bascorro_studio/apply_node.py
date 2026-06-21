#!/usr/bin/env python3

import datetime
import json
import os
import shutil
import subprocess
import tempfile
from typing import Dict, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


def resolve_action_file() -> str:
    env_path = os.environ.get("OP3_ACTION_FILE", "").strip()
    if env_path:
        return env_path
    return get_package_share_directory("op3_action_module") + "/data/motion_4095_CHRONUS.bin"


def resolve_seed_file() -> str:
    env_path = os.environ.get("OP3_ACTION_FILE_SEED", "").strip()
    if env_path:
        return env_path
    return get_package_share_directory("op3_action_module") + "/data/motion_4095_CHRONUS.bin"


def resolve_robot_file() -> str:
    return get_package_share_directory("op3_manager") + "/config/OP3.robot"


def ensure_action_file(action_file: str, seed_file: str, logger) -> None:
    if os.path.isfile(action_file):
        return
    if not os.path.isfile(seed_file):
        logger.warning(f"Action seed file not found: {seed_file}")
        return
    os.makedirs(os.path.dirname(action_file), exist_ok=True)
    shutil.copyfile(seed_file, action_file)
    logger.info(f"Initialized action file: {action_file}")


def run_action_yaml(action_file: str, robot_file: str, args: list) -> Tuple[int, str, str]:
    cmd = [
        "ros2",
        "run",
        "op3_action_editor",
        "action_yaml.py",
        "--action-file",
        action_file,
        "--robot-file",
        robot_file,
    ] + args
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout, proc.stderr


class ActionWebNode(Node):
    def __init__(self) -> None:
        super().__init__("bascorro_studio_apply")

        self.action_file = resolve_action_file()
        self.seed_file = resolve_seed_file()
        self.robot_file = resolve_robot_file()
        raw_backup = self.declare_parameter("create_backup", False).value
        if isinstance(raw_backup, bool):
            self.create_backup = raw_backup
        else:
            self.create_backup = str(raw_backup).strip().lower() in ("1", "true", "yes")

        ensure_action_file(self.action_file, self.seed_file, self.get_logger())

        self.result_pub = self.create_publisher(
            String, "/bascorro_studio/result", 10
        )
        self.legacy_result_pub = self.create_publisher(
            String, "/op3_action_web/result", 10
        )
        self.apply_sub = self.create_subscription(
            String,
            "/bascorro_studio/apply_yaml",
            self.handle_apply_yaml,
            10,
        )
        self.request_sub = self.create_subscription(
            String,
            "/bascorro_studio/request",
            self.handle_request,
            10,
        )
        self.create_subscription(
            String,
            "/op3_action_web/apply_yaml",
            self.handle_apply_yaml,
            10,
        )
        self.create_subscription(
            String,
            "/op3_action_web/request",
            self.handle_request,
            10,
        )

        self._busy = False
        self.get_logger().info("Studio action node ready")
        self.get_logger().info(f"Action file: {self.action_file}")

    def handle_apply_yaml(self, msg: String) -> None:
        payload = {"action": "apply", "yaml": msg.data}
        self.handle_payload(payload)

    def handle_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"action": "apply", "yaml": msg.data}
        if not isinstance(payload, dict):
            payload = {"action": "apply", "yaml": msg.data}
        elif "action" not in payload:
            payload["action"] = "apply"
        self.handle_payload(payload)

    def handle_payload(self, payload: Dict[str, object]) -> None:
        if self._busy:
            self.publish_result(
                {
                    "ok": False,
                    "action": payload.get("action", "apply"),
                    "message": "Busy; try again shortly",
                }
            )
            return

        self._busy = True
        try:
            action = str(payload.get("action", "apply"))
            request_id = payload.get("request_id")
            if action == "apply":
                self.apply_yaml(str(payload.get("yaml", "")), request_id)
            elif action == "export":
                pages = str(payload.get("pages", "used"))
                self.export_yaml(pages, request_id)
            elif action == "stat":
                # cheap query: which bin will be written and when it last changed,
                # so the UI can warn before overwriting it with a stale draft.
                self.publish_result(
                    {"ok": True, "action": "stat", "request_id": request_id, **self.file_info()}
                )
            else:
                self.publish_result(
                    {
                        "ok": False,
                        "action": action,
                        "message": f"Unknown action: {action}",
                        "request_id": request_id,
                    }
                )
        finally:
            self._busy = False

    def apply_yaml(self, yaml_text: str, request_id=None) -> None:
        if not yaml_text.strip():
            self.publish_result(
                {
                    "ok": False,
                    "action": "apply",
                    "message": "YAML is empty",
                    "request_id": request_id,
                }
            )
            return

        tmp_path = ""
        try:
            with tempfile.NamedTemporaryFile("w", delete=False, suffix=".yaml") as handle:
                handle.write(yaml_text)
                tmp_path = handle.name

            args = ["import", "--in", tmp_path]
            if not self.create_backup:
                args.append("--no-backup")

            code, stdout, stderr = run_action_yaml(self.action_file, self.robot_file, args)
            ok = code == 0
            if not ok:
                err_text = stderr.strip() or stdout.strip()
                if err_text:
                    self.get_logger().error(f"action_yaml import failed: {err_text}")
            self.publish_result(
                {
                    "ok": ok,
                    "action": "apply",
                    "message": "Applied YAML" if ok else "Failed to apply YAML",
                    "stdout": stdout.strip(),
                    "stderr": stderr.strip(),
                    "request_id": request_id,
                }
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.publish_result(
                {
                    "ok": False,
                    "action": "apply",
                    "message": f"Apply error: {exc}",
                    "request_id": request_id,
                }
            )
        finally:
            if tmp_path and os.path.isfile(tmp_path):
                os.unlink(tmp_path)

    def export_yaml(self, pages: str, request_id=None) -> None:
        tmp_path = ""
        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".yaml") as handle:
                tmp_path = handle.name

            args = ["export", "--out", tmp_path, "--pages", pages]
            code, stdout, stderr = run_action_yaml(self.action_file, self.robot_file, args)
            ok = code == 0
            if not ok:
                err_text = stderr.strip() or stdout.strip()
                if err_text:
                    self.get_logger().error(f"action_yaml export failed: {err_text}")
            yaml_text = ""
            if ok and os.path.isfile(tmp_path):
                with open(tmp_path, "r", encoding="utf-8") as handle:
                    yaml_text = handle.read()

            self.publish_result(
                {
                    "ok": ok,
                    "action": "export",
                    "message": "Exported YAML" if ok else "Failed to export YAML",
                    "stdout": stdout.strip(),
                    "stderr": stderr.strip(),
                    "yaml": yaml_text,
                    "request_id": request_id,
                }
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.publish_result(
                {
                    "ok": False,
                    "action": "export",
                    "message": f"Export error: {exc}",
                    "request_id": request_id,
                }
            )
        finally:
            if tmp_path and os.path.isfile(tmp_path):
                os.unlink(tmp_path)

    def file_info(self) -> Dict[str, object]:
        """Target bin path + when it was last modified on disk, so the UI can show
        'this bin was last updated at X' and warn if a draft is older than that."""
        path = self.action_file
        info: Dict[str, object] = {
            "action_file": path,
            "action_file_name": os.path.basename(path),
        }
        try:
            mtime = os.path.getmtime(path)
            info["mtime"] = mtime  # epoch seconds
            info["mtime_iso"] = datetime.datetime.fromtimestamp(mtime).isoformat(timespec="seconds")
        except OSError:
            info["mtime"] = None
            info["mtime_iso"] = None
        return info

    def publish_result(self, payload: Dict[str, object]) -> None:
        # always attach current bin file info so the UI knows the on-disk state
        if "action_file" not in payload:
            payload = {**payload, **self.file_info()}
        msg = String()
        msg.data = json.dumps(payload)
        self.result_pub.publish(msg)
        self.legacy_result_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

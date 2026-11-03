#!/usr/bin/env python3

import datetime
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Dict, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


def resolve_action_file() -> str:
    env_path = os.environ.get("OP3_ACTION_FILE", "").strip()
    if env_path:
        return env_path
    return get_package_share_directory("op3_action_module") + "/data/ALPHONSE.bin"


def resolve_seed_file() -> str:
    env_path = os.environ.get("OP3_ACTION_FILE_SEED", "").strip()
    if env_path:
        return env_path
    return get_package_share_directory("op3_action_module") + "/data/ALPHONSE.bin"


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
        # Directory that holds the action .bin files. The web "Bin File" panel can
        # list/select/delete bins here; self.action_file is the launch-active one
        # (the bin the running op3_action_module actually loaded) and is protected.
        self.data_dir = os.path.dirname(self.action_file)
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
        self._action_train = None
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
            target = payload.get("target")
            if action == "apply":
                self.apply_yaml(str(payload.get("yaml", "")), request_id, target)
            elif action == "export":
                pages = str(payload.get("pages", "used"))
                self.export_yaml(pages, request_id, target)
            elif action == "stat":
                # cheap query: which bin will be written and when it last changed,
                # so the UI can warn before overwriting it with a stale draft.
                path, err = self.resolve_target(target, must_exist=True)
                if err:
                    self.publish_result({"ok": False, "action": "stat", "message": err, "request_id": request_id})
                else:
                    self.publish_result(
                        {"ok": True, "action": "stat", "request_id": request_id, **self.file_info(path)}
                    )
            elif action == "list":
                result = self.list_bins()
                result["action"] = "list"
                result["request_id"] = request_id
                self.publish_result(result)
            elif action == "delete":
                self.delete_bin(target, request_id)
            elif action == "rl_get":
                self.rl_get(request_id)
            elif action == "rl_add_trial":
                self.rl_add_trial(payload.get("trial"), request_id)
            elif action == "rl_delete_trial":
                self.rl_delete_trial(payload.get("id"), request_id)
            elif action == "act_get":
                self.rl_get(request_id, store="action", action=action)
            elif action == "act_add_trial":
                self.rl_add_trial(payload.get("trial"), request_id, store="action", action=action)
            elif action == "act_delete_trial":
                self.rl_delete_trial(payload.get("id"), request_id, store="action", action=action)
            elif action.startswith("act_"):
                self.action_train(action, payload, request_id)
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

    def apply_yaml(self, yaml_text: str, request_id=None, target=None) -> None:
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

        action_file, err = self.resolve_target(target, must_exist=True)
        if err:
            self.publish_result(
                {"ok": False, "action": "apply", "message": err, "request_id": request_id}
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

            code, stdout, stderr = run_action_yaml(action_file, self.robot_file, args)
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
                    **self.file_info(action_file),
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

    def export_yaml(self, pages: str, request_id=None, target=None) -> None:
        action_file, err = self.resolve_target(target, must_exist=True)
        if err:
            self.publish_result(
                {"ok": False, "action": "export", "message": err, "request_id": request_id}
            )
            return

        tmp_path = ""
        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".yaml") as handle:
                tmp_path = handle.name

            args = ["export", "--out", tmp_path, "--pages", pages]
            code, stdout, stderr = run_action_yaml(action_file, self.robot_file, args)
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
                    **self.file_info(action_file),
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

    def resolve_target(self, target, must_exist=False) -> Tuple[str, str]:
        """Resolve a user-supplied bin name to a full path inside the data dir.

        Only a bare ``*.bin`` filename is allowed (no path separators, no ``..``),
        so the web cannot read/write/delete outside the action data directory.
        Returns ``(path, error)`` -- error is "" on success. When target is empty
        the launch-active action file is used.
        """
        if not target:
            return self.action_file, ""
        name = str(target)
        base = os.path.basename(name)
        if base != name or base in ("", ".", ".."):
            return "", "Invalid target name"
        if not base.endswith(".bin"):
            return "", "Target must be a .bin file"
        path = os.path.join(self.data_dir, base)
        if must_exist and not os.path.isfile(path):
            return "", f"Bin not found: {base}"
        return path, ""

    def list_bins(self) -> Dict[str, object]:
        """List the *.bin files in the action data dir for the Bin File picker.
        Marks the launch-active bin so the UI can protect it from deletion."""
        active = os.path.basename(self.action_file)
        bins = []
        try:
            for name in sorted(os.listdir(self.data_dir)):
                if not name.endswith(".bin"):
                    continue
                path = os.path.join(self.data_dir, name)
                if not os.path.isfile(path):
                    continue
                try:
                    mtime = os.path.getmtime(path)
                    mtime_iso = datetime.datetime.fromtimestamp(mtime).isoformat(timespec="seconds")
                    size = os.path.getsize(path)
                except OSError:
                    mtime, mtime_iso, size = None, None, None
                bins.append(
                    {
                        "name": name,
                        "mtime": mtime,
                        "mtime_iso": mtime_iso,
                        "size": size,
                        "active": name == active,
                    }
                )
            return {"ok": True, "bins": bins, "active": active}
        except OSError as exc:
            return {"ok": False, "bins": [], "active": active, "message": f"List error: {exc}"}

    #-- RL Training (tab "RL Training" di Studio) ------------------------------
    # Tiap percobaan yang sudah dinilai operator disimpan di robot, bukan di
    # localStorage (localStorage dipisah per origin -- lihat walking_versions).
    # Satu berkas untuk semua sesi; nomor percobaan dihitung per sesi DI SINI,
    # jadi dua peramban yang terbuka bersamaan tidak memberi nomor kembar.
    # Percobaan lama dari menu Training masuk lewat scripts/rl_impor_training.py.
    # Percobaan yang sudah dinilai: tab RL Training di rl_runs/studio, tab Action
    # Training di rl_runs/action (aksi act_get/act_add_trial/act_delete_trial).
    def rl_path(self, store="studio") -> str:
        ws = os.environ.get("OP3_WS", "/ros2_ws")
        return os.path.join(ws, "rl_runs", store, "trials.json")

    def rl_load(self, store="studio") -> list:
        try:
            with open(self.rl_path(store), "r", encoding="utf-8") as handle:
                data = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return []
        trials = data.get("trials") if isinstance(data, dict) else None
        return trials if isinstance(trials, list) else []

    def rl_save(self, trials: list, store="studio") -> None:
        path = self.rl_path(store)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump({"trials": trials}, handle, indent=2)
        os.replace(tmp, path)

    def rl_get(self, request_id=None, store="studio", action="rl_get") -> None:
        self.publish_result(
            {"ok": True, "action": action, "trials": self.rl_load(store), "path": self.rl_path(store),
             "request_id": request_id}
        )

    def rl_add_trial(self, trial, request_id=None, store="studio", action="rl_add_trial") -> None:
        field = "values" if store == "action" else "params"
        if not isinstance(trial, dict) or not isinstance(trial.get(field), dict):
            self.publish_result({"ok": False, "action": action,
                                 "message": f"trial harus berisi {field}", "request_id": request_id})
            return
        trials = self.rl_load(store)
        session = str(trial.get("session") or "default")
        trial["session"] = session
        trial["no"] = 1 + max((int(t.get("no", 0)) for t in trials if t.get("session") == session),
                              default=0)
        trial.setdefault("at", int(time.time() * 1000))
        trials.append(trial)
        try:
            self.rl_save(trials, store)
        except OSError as exc:
            self.publish_result({"ok": False, "action": action,
                                 "message": f"Write error: {exc}", "request_id": request_id})
            return
        self.publish_result({"ok": True, "action": action, "trial": trial,
                             "trials": trials, "request_id": request_id})

    def rl_delete_trial(self, trial_id, request_id=None, store="studio", action="rl_delete_trial") -> None:
        trials = self.rl_load(store)
        kept = [t for t in trials if t.get("id") != trial_id]
        if len(kept) == len(trials):
            self.publish_result({"ok": False, "action": action,
                                 "message": "percobaan tidak ditemukan", "request_id": request_id})
            return
        try:
            self.rl_save(kept, store)
        except OSError as exc:
            self.publish_result({"ok": False, "action": action,
                                 "message": f"Write error: {exc}", "request_id": request_id})
            return
        self.publish_result({"ok": True, "action": action, "trials": kept,
                             "request_id": request_id})

    # Tab Action Training: semua yang menyentuh .bin ada di scripts/training_action_web.py
    # (satu rumus dengan menu Training). Robotnya dijalankan dari browser.
    def action_train_module(self):
        if self._action_train is None:
            scripts = os.path.join(os.environ.get("OP3_WS", "/ros2_ws"), "scripts")
            if scripts not in sys.path:
                sys.path.insert(0, scripts)
            import training_action_web
            self._action_train = training_action_web
        return self._action_train

    def action_train(self, action, payload, request_id=None) -> None:
        try:
            web = self.action_train_module()
            if action == "act_pages":
                result = web.page_list()
            elif action == "act_page":
                result = web.page_detail(payload.get("page"))
            elif action == "act_load":
                result = web.load(payload.get("page"), payload.get("sha"), payload.get("values") or {})
            elif action == "act_restore":
                result = web.restore()
            elif action == "act_metrics":
                result = web.trial_metrics(payload.get("samples") or [], payload.get("marks") or {},
                                           str(payload.get("profile") or "kick"))
            elif action == "act_deploy":
                result = web.deploy_result(payload.get("page"), payload.get("sha"), payload.get("values") or {},
                                           payload.get("target"), payload.get("name") or None,
                                           bool(payload.get("overwrite")))
            else:
                raise ValueError(f"Unknown action: {action}")
        # SystemExit juga: alat terminal di scripts/ melapor galat dengannya, dan node ini
        # tidak boleh ikut mati karena satu permintaan.
        except (Exception, SystemExit) as exc:
            self.get_logger().error(f"{action}: {exc}")
            self.publish_result({"ok": False, "action": action, "message": str(exc), "request_id": request_id})
            return
        self.publish_result({"ok": True, "action": action, "request_id": request_id, **result})

    def delete_bin(self, target, request_id=None) -> None:
        path, err = self.resolve_target(target, must_exist=True)
        if err:
            self.publish_result({"ok": False, "action": "delete", "message": err, "request_id": request_id})
            return
        if os.path.abspath(path) == os.path.abspath(self.action_file):
            self.publish_result(
                {
                    "ok": False,
                    "action": "delete",
                    "message": "Refusing to delete the active action file",
                    "request_id": request_id,
                }
            )
            return
        name = os.path.basename(path)
        try:
            os.remove(path)
            ok, message = True, f"Deleted {name}"
        except OSError as exc:
            ok, message = False, f"Delete error: {exc}"
        result = {"ok": ok, "action": "delete", "message": message, "request_id": request_id}
        # include a refreshed bin list so the dropdown updates immediately
        listing = self.list_bins()
        result["bins"] = listing.get("bins", [])
        result["active"] = listing.get("active")
        self.publish_result(result)

    def file_info(self, path=None) -> Dict[str, object]:
        """Target bin path + when it was last modified on disk, so the UI can show
        'this bin was last updated at X' and warn if a draft is older than that."""
        if not path:
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

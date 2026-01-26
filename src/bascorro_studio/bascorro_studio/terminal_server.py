#!/usr/bin/env python3

import argparse
import os
import signal
import subprocess
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node


class TerminalServer(Node):
    def __init__(self) -> None:
        super().__init__("bascorro_studio_terminal")

        self.declare_parameter("port", 7681)
        self.declare_parameter("credential", "")
        self.declare_parameter("max_clients", 10)
        self.declare_parameter("writable", True)
        self.declare_parameter("check_origin", False)

        self._process: Optional[subprocess.Popen] = None
        self._start_server()

    def _start_server(self) -> None:
        port = int(self.get_parameter("port").value)
        credential = str(self.get_parameter("credential").value).strip()
        max_clients = int(self.get_parameter("max_clients").value)
        writable = bool(self.get_parameter("writable").value)
        check_origin = bool(self.get_parameter("check_origin").value)

        # Check if ttyd is installed
        if not self._check_ttyd():
            self.get_logger().error("ttyd is not installed. Cannot start terminal server.")
            self.get_logger().info("Install ttyd with: sudo apt-get install -y ttyd")
            return

        # Build ttyd command
        cmd = [
            "ttyd",
            "--port", str(port),
            "--max-clients", str(max_clients),
            "--interface", "0.0.0.0",  # Bind to all interfaces for network access
        ]

        if credential:
            cmd.extend(["--credential", credential])

        if writable:
            cmd.append("--writable")

        if not check_origin:
            cmd.append("--check-origin")

        # Use bash as the shell
        cmd.append("bash")

        self.get_logger().info(f"Starting terminal server on port {port}")
        self.get_logger().info("Terminal accessible on all network interfaces (0.0.0.0)")
        if credential:
            self.get_logger().info("Authentication enabled (user:pass format)")
        else:
            self.get_logger().warn("Authentication DISABLED - terminal is publicly accessible on network!")
            self.get_logger().warn("Set credential parameter for security: credential:='user:password'")

        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            time.sleep(1)

            if self._process.poll() is None:
                self.get_logger().info(f"Terminal server running at http://0.0.0.0:{port}")
                self.get_logger().info("Access from local network: http://<your-ip>:{port}")
                self.get_logger().info("Access the terminal from your browser or the Bascorro Studio web UI")
            else:
                stderr = self._process.stderr.read().decode() if self._process.stderr else ""
                self.get_logger().error(f"Terminal server failed to start: {stderr}")
                self._process = None

        except Exception as exc:
            self.get_logger().error(f"Failed to start terminal server: {exc}")
            self._process = None

    def _check_ttyd(self) -> bool:
        try:
            result = subprocess.run(
                ["which", "ttyd"],
                capture_output=True,
                check=False,
            )
            return result.returncode == 0
        except Exception:
            return False

    def shutdown(self) -> None:
        if self._process and self._process.poll() is None:
            self.get_logger().info("Stopping terminal server")
            self._process.send_signal(signal.SIGTERM)
            try:
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerminalServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

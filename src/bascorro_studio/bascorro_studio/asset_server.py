#!/usr/bin/env python3

import argparse
import functools
import http.server
import os
import socketserver
import subprocess
import sys
from typing import Optional

from ament_index_python.packages import get_package_share_directory


class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        super().end_headers()

    def do_OPTIONS(self) -> None:
        self.send_response(200)
        self.end_headers()

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return


def generate_assets(assets_dir: str) -> Optional[str]:
    op3_description = get_package_share_directory("op3_description")
    xacro_path = os.path.join(op3_description, "urdf", "robotis_op3.urdf.xacro")
    mesh_dir = os.path.join(op3_description, "meshes")

    if not os.path.isfile(xacro_path):
        print(f"URDF xacro not found: {xacro_path}", file=sys.stderr)
        return None
    if not os.path.isdir(mesh_dir):
        print(f"Mesh directory not found: {mesh_dir}", file=sys.stderr)
        return None

    os.makedirs(assets_dir, exist_ok=True)

    try:
        result = subprocess.run(
            ["xacro", xacro_path],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        print("xacro failed:", exc.stderr, file=sys.stderr)
        return None

    urdf_text = result.stdout
    mesh_uri = f"file://{mesh_dir}"
    urdf_text = urdf_text.replace(mesh_uri + "/", "/meshes/")
    urdf_text = urdf_text.replace(mesh_uri, "/meshes")

    meshes_link = os.path.join(assets_dir, "meshes")
    if os.path.islink(meshes_link) or os.path.isfile(meshes_link):
        os.unlink(meshes_link)
    if os.path.isdir(meshes_link) and not os.path.islink(meshes_link):
        print(f"Meshes directory already exists: {meshes_link}")
        print("Using existing mesh directory without changes.")
    else:
        os.symlink(mesh_dir, meshes_link)

    urdf_path = os.path.join(assets_dir, "robotis_op3.urdf")
    with open(urdf_path, "w", encoding="utf-8") as handle:
        handle.write(urdf_text)

    return urdf_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve OP3 URDF + meshes for the action web UI.")
    parser.add_argument(
        "--port",
        type=int,
        default=int(
            os.environ.get(
                "OP3_STUDIO_ASSETS_PORT",
                os.environ.get("OP3_ACTION_WEB_ASSETS_PORT", "8001"),
            )
        ),
        help="Port to serve assets (default: 8001)",
    )
    parser.add_argument(
        "--dir",
        default=os.environ.get(
            "OP3_STUDIO_ASSETS_DIR",
            os.environ.get("OP3_ACTION_WEB_ASSETS_DIR", "/tmp/bascorro_studio_assets"),
        ),
        help="Directory to place generated assets",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    urdf_path = generate_assets(args.dir)
    if urdf_path is None:
        sys.exit(1)

    socketserver.TCPServer.allow_reuse_address = True
    handler = functools.partial(CORSRequestHandler, directory=args.dir)
    server = http.server.ThreadingHTTPServer(("", args.port), handler)

    with server as httpd:
        print(f"Assets ready: {urdf_path}")
        print(f"Serving: http://localhost:{args.port}/robotis_op3.urdf")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

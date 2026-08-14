from __future__ import annotations

import math
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse

from scratch_hci_bridge.command_state import (
    CommandState,
    JOINT_SPEED_COMMANDS,
    RELATIVE_JOINT_COMMANDS,
    SUPPORTED_COMMANDS,
    is_safe_relative_swing_command,
    is_safe_pid_speed_limit,
)


CommandCallback = Callable[[str, float], None]


class ScratchHTTPRequestHandler(BaseHTTPRequestHandler):
    state: CommandState
    command_callback: CommandCallback | None = None

    def do_GET(self) -> None:
        parsed_path = urlparse(self.path)
        if parsed_path.path != "/":
            self._send_text(404, "not found")
            return

        query = parse_qs(parsed_path.query, keep_blank_values=True)
        if "type" not in query:
            self._send_text(400, "type=command is required")
            return
        if "cmd" not in query or "val" not in query:
            self._send_text(400, "cmd and val are required")
            return
        if set(query) != {"type", "cmd", "val"}:
            self._send_text(400, "only type, cmd and val are allowed")
            return
        if query["type"] != ["command"]:
            self._send_text(400, "type=command is required")
            return
        if len(query["cmd"]) != 1 or len(query["val"]) != 1:
            self._send_text(400, "cmd and val must appear once")
            return

        command = query["cmd"][0]
        raw_value = query["val"][0]
        try:
            value = float(raw_value)
        except ValueError:
            self._send_text(400, "val must be a finite number")
            return
        if not math.isfinite(value):
            self._send_text(400, "val must be a finite number")
            return
        if command not in SUPPORTED_COMMANDS:
            self._send_text(400, f"unknown cmd: {command}")
            return
        if (
            command in RELATIVE_JOINT_COMMANDS
            and not is_safe_relative_swing_command(command, value)
        ):
            self._send_text(
                400, "relative swing angle must be less than 180 degrees"
            )
            return
        if command in JOINT_SPEED_COMMANDS and value < 0.0:
            self._send_text(400, "joint speed must be non-negative")
            return
        if (
            command in JOINT_SPEED_COMMANDS
            and value > 0.0
            and not is_safe_pid_speed_limit(value)
        ):
            self._send_text(400, "joint speed would exceed ROS Duration range")
            return
        if not self.state.apply_command(command, value):
            self._send_text(400, "command would produce a non-finite value")
            return

        callback = type(self).command_callback
        if callback is not None:
            callback(command, value)
        self._send_text(200, "OK")

    def log_message(self, format: str, *args: object) -> None:
        return

    def _send_text(self, status: int, message: str) -> None:
        body = message.encode("utf-8")
        self.send_response(status)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


class ScratchHTTPServer(HTTPServer):
    allow_reuse_address = True

    def get_request(self):
        request, client_address = super().get_request()
        # 未完了のHTTP接続が終了を妨げないようreadを制限する。
        request.settimeout(1.0)
        return request, client_address


def create_http_server(
    host: str,
    port: int,
    state: CommandState,
    command_callback: CommandCallback | None = None,
) -> ScratchHTTPServer:
    class BoundScratchHTTPRequestHandler(ScratchHTTPRequestHandler):
        pass

    BoundScratchHTTPRequestHandler.state = state
    BoundScratchHTTPRequestHandler.command_callback = staticmethod(command_callback)
    # Process one request at a time so commands are handed to ROS in arrival order.
    return ScratchHTTPServer((host, port), BoundScratchHTTPRequestHandler)

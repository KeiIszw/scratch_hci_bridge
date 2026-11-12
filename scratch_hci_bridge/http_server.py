"""ScratchのHTTP要求を受付順に適用し、停止と状態照会は即座に処理する。"""
from __future__ import annotations

import math
import threading
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
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
    # 動作状態の読み取り専用窓口。操作状態を書き換えるcommand callbackとは分ける。
    status_callback: Callable[[], int] | None = None

    request_callback = None
    error_callback = None
    joint_queue_callback = None

    def do_GET(self) -> None:
        request_callback = type(self).request_callback
        if request_callback is not None:
            request_callback(False)
        self.request_order, self.request_epoch = self.server.request_orders[self.request]
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
        # Scratchから動作完了を確認する要求。ここでは新しい動作指令を登録しない。
        if query["type"] == ["reporter"]:
            if query["cmd"] == ["joint_queue"]:
                if query["val"] not in (["0"], ["1"], ["2"], ["3"]) or type(self).joint_queue_callback is None:
                    self._send_text(400, "joint_queue val must be 0..3")
                else:
                    self._send_text(200, str(type(self).joint_queue_callback(int(query["val"][0]))))
                return
            if query["val"] != ["0"]:
                self._send_text(400, "reporter val must be 0")
                return
            command = query["cmd"]
            if command == ["heartbeat"]:
                if request_callback is not None:
                    request_callback(True)
                self._send_text(200, "ok")
            elif command == ["motion_error"] and type(self).error_callback is not None:
                self._send_text(200, type(self).error_callback())
            elif command == ["motion_status"] and type(self).status_callback is not None:
                self._send_text(200, {0: "idle", 1: "busy", -1: "failed", 2: "stopped"}.get(type(self).status_callback(), "failed"))
            else:
                self._send_text(400, "unknown reporter")
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
        # 通常指令は接続受付順に適用する。停止はこの待ち列を通さず即座に適用する。
        if command in ("stop", "reset"):
            self._apply(command, value)
        else:
            with self.server.command_condition:
                self.server.command_condition.wait_for(
                    lambda: not any(n < self.request_order for n in self.server.pending_orders))
            self._apply(command, value)

    def _apply(self, command: str, value: float) -> None:
        # 状態とschedulerへの登録を同じ短い区間で更新し、停止との隙間をなくす。
        accepted = True
        with self.server.state_lock:
            # 世代更新と停止処理の間に後続指令を受け付けると、その指令まで破棄されるため一体で更新する。
            if command in ("stop", "reset"):
                self.server.command_epoch += 1
            expired = command not in ("stop", "reset") and self.request_epoch != self.server.command_epoch
            if not expired:
                accepted = self.state.apply_command(command, value)
                callback = type(self).command_callback
                if accepted and callback is not None:
                    accepted = callback(command, value) is not False
        if accepted:
            self._send_text(200, "OK")
        else:
            self._send_text(400, "command would produce a non-finite value")

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


class ScratchHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, *args):
        self.command_condition = threading.Condition()
        self.state_lock = threading.RLock()
        self.pending_orders = set()
        self.request_orders = {}
        self.next_order = 0
        self.command_epoch = 0
        super().__init__(*args)

    def process_request(self, request, client_address):
        # 停止の世代更新と受付を直列化し、停止後の接続へ古い世代を割り当てない。
        with self.state_lock:
            with self.command_condition:
                self.next_order += 1
                self.request_orders[request] = (self.next_order, self.command_epoch)
                self.pending_orders.add(self.next_order)
        super().process_request(request, client_address)

    def finish_request(self, request, client_address):
        try:
            super().finish_request(request, client_address)
        finally:
            with self.command_condition:
                self.pending_orders.discard(self.request_orders.pop(request)[0])
                self.command_condition.notify_all()

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
    status_callback: Callable[[], int] | None = None,
    error_callback: Callable[[], str] | None = None,
    request_callback: Callable[[bool], None] | None = None,
    state_lock=None,
    joint_queue_callback: Callable[[int], int] | None = None,
) -> ScratchHTTPServer:
    class BoundScratchHTTPRequestHandler(ScratchHTTPRequestHandler):
        pass

    BoundScratchHTTPRequestHandler.joint_queue_callback = staticmethod(joint_queue_callback)
    BoundScratchHTTPRequestHandler.error_callback = staticmethod(error_callback)
    BoundScratchHTTPRequestHandler.request_callback = staticmethod(request_callback)
    BoundScratchHTTPRequestHandler.state = state
    BoundScratchHTTPRequestHandler.command_callback = staticmethod(command_callback)
    # 要求を1件ずつ処理し、到着順に指令をROSへ渡す。
    BoundScratchHTTPRequestHandler.status_callback = staticmethod(status_callback)
    server = ScratchHTTPServer((host, port), BoundScratchHTTPRequestHandler)
    if state_lock is not None:
        server.state_lock = state_lock
    return server

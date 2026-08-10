from __future__ import annotations

import math
import signal
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from scratch_hci_bridge.command_state import CommandState, JOINT_NAMES
from scratch_hci_bridge.http_server import create_http_server


class ScratchHciBridge(Node):
    def __init__(self) -> None:
        super().__init__("scratch_hci_bridge")

        self.declare_parameter("excavator", "tb20e")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8899)
        self.declare_parameter("drive_linear_scale", 1.0)
        self.declare_parameter("drive_angular_scale", 1.0)

        excavator = str(self.get_parameter("excavator").value).strip("/")
        publish_hz = float(self.get_parameter("publish_hz").value)
        http_host = str(self.get_parameter("http_host").value)
        http_port = int(self.get_parameter("http_port").value)
        drive_linear_scale = float(self.get_parameter("drive_linear_scale").value)
        drive_angular_scale = float(self.get_parameter("drive_angular_scale").value)

        if not excavator:
            raise ValueError("excavator must not be empty")
        if not math.isfinite(publish_hz) or publish_hz <= 0.0:
            raise ValueError("publish_hz must be finite and greater than zero")
        if not 1 <= http_port <= 65535:
            raise ValueError("http_port must be between 1 and 65535")
        if not all(
            math.isfinite(value)
            for value in (drive_linear_scale, drive_angular_scale)
        ):
            raise ValueError("drive scales must be finite")

        self._state = CommandState(
            drive_linear_scale=drive_linear_scale,
            drive_angular_scale=drive_angular_scale,
        )
        self._drive_publisher = self.create_publisher(
            Twist, f"/{excavator}/tracks/cmd_vel", 10
        )
        self._joint_publishers = {
            joint: self.create_publisher(Float64, f"/{excavator}/{joint}/cmd", 10)
            for joint in JOINT_NAMES
        }
        self._timer = self.create_timer(1.0 / publish_hz, self._publish_commands)

        self._http_server = create_http_server(
            http_host,
            http_port,
            self._state,
            self._log_command,
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="scratch-hci-http",
            daemon=True,
        )
        self._http_thread.start()

        self.get_logger().info(
            f"Scratch HCI HTTP server listening on {http_host or '0.0.0.0'}:{http_port}"
        )
        self.get_logger().info(
            f"Publishing OperaSim commands under /{excavator} at {publish_hz:g} Hz"
        )

    def _log_command(self, command: str, value: float) -> None:
        self.get_logger().info(f"HTTP command: {command}={value:g}")

    def _publish_commands(self) -> None:
        frame = self._state.next_frame()

        drive_message = Twist()
        drive_message.linear.x = frame.linear_velocity
        drive_message.angular.z = frame.angular_velocity
        self._drive_publisher.publish(drive_message)

        for joint, degrees in frame.joints_deg.items():
            message = Float64()
            message.data = math.radians(degrees)
            self._joint_publishers[joint].publish(message)

    def destroy_node(self) -> None:
        self._timer.cancel()
        self._http_server.shutdown()
        self._http_server.server_close()
        self._http_thread.join(timeout=2.0)

        # Scratchが先に終了しても、ノード停止時に走行指令を残さない。
        self._state.stop_drive()
        if rclpy.ok():
            self._publish_commands()
            if not self._drive_publisher.wait_for_all_acked(Duration(seconds=0.5)):
                self.get_logger().warning(
                    "Timed out waiting for the stop command acknowledgment"
                )
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: ScratchHciBridge | None = None
    stop_requested = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_requested.set()

    previous_handlers = {
        signal_number: signal.signal(signal_number, request_stop)
        for signal_number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        node = ScratchHciBridge()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signal_number, previous_handler in previous_handlers.items():
            signal.signal(signal_number, previous_handler)


if __name__ == "__main__":
    main()

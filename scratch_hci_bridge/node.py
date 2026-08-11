from __future__ import annotations

import math
import signal
import threading
from functools import partial
from typing import Any

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint

from scratch_hci_bridge.command_state import CommandState, JOINT_NAMES
from scratch_hci_bridge.http_server import create_http_server
from scratch_hci_bridge.trajectory_targets import (
    PID_JOINTS,
    TrajectoryTargetSnapshot,
    TrajectoryTargets,
)


PID_HTTP_JOINT_NAMES = frozenset(http_name for http_name, _ in PID_JOINTS)
DIRECT_JOINT_NAMES = tuple(
    joint for joint in JOINT_NAMES if joint not in PID_HTTP_JOINT_NAMES
)


class ScratchHciBridge(Node):
    def __init__(self) -> None:
        super().__init__("scratch_hci_bridge")

        self.declare_parameter("excavator", "tb20e")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8899)
        self.declare_parameter("drive_linear_scale", 1.0)
        self.declare_parameter("drive_angular_scale", 1.0)
        self.declare_parameter(
            "trajectory_action", "/tb20e_controller/follow_joint_trajectory"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("trajectory_duration_sec", 3.0)
        self.declare_parameter("trajectory_debounce_sec", 0.05)

        excavator = str(self.get_parameter("excavator").value).strip("/")
        publish_hz = float(self.get_parameter("publish_hz").value)
        http_host = str(self.get_parameter("http_host").value)
        http_port = int(self.get_parameter("http_port").value)
        drive_linear_scale = float(self.get_parameter("drive_linear_scale").value)
        drive_angular_scale = float(self.get_parameter("drive_angular_scale").value)
        trajectory_action = str(self.get_parameter("trajectory_action").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._trajectory_duration_sec = float(
            self.get_parameter("trajectory_duration_sec").value
        )
        self._trajectory_debounce_sec = float(
            self.get_parameter("trajectory_debounce_sec").value
        )

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
        if not trajectory_action:
            raise ValueError("trajectory_action must not be empty")
        if not self._joint_state_topic:
            raise ValueError("joint_state_topic must not be empty")
        if (
            not math.isfinite(self._trajectory_duration_sec)
            or self._trajectory_duration_sec <= 0.0
        ):
            raise ValueError(
                "trajectory_duration_sec must be finite and greater than zero"
            )
        if (
            not math.isfinite(self._trajectory_debounce_sec)
            or self._trajectory_debounce_sec < 0.0
        ):
            raise ValueError(
                "trajectory_debounce_sec must be finite and non-negative"
            )

        self._state = CommandState(
            drive_linear_scale=drive_linear_scale,
            drive_angular_scale=drive_angular_scale,
        )
        self._drive_publisher = self.create_publisher(
            Twist, f"/{excavator}/tracks/cmd_vel", 10
        )
        self._direct_joint_publishers = {
            joint: self.create_publisher(Float64, f"/{excavator}/{joint}/cmd", 10)
            for joint in DIRECT_JOINT_NAMES
        }
        self._trajectory_targets = TrajectoryTargets()
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, trajectory_action
        )
        self._joint_state_subscription = self.create_subscription(
            JointState, self._joint_state_topic, self._joint_state_callback, 10
        )
        self._action_server_unavailable_reported = False
        self._waiting_for_joint_state_reported = False
        self._timer = self.create_timer(1.0 / publish_hz, self._on_timer)

        self._http_server = create_http_server(
            http_host,
            http_port,
            self._state,
            self._command_callback,
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
            f"Publishing drive/direct commands under /{excavator} at {publish_hz:g} Hz"
        )
        self.get_logger().info(
            f"Sending four-axis PID targets to {trajectory_action}"
        )

    def _command_callback(self, command: str, value: float) -> None:
        self.get_logger().info(f"HTTP command: {command}={value:g}")
        if command not in PID_HTTP_JOINT_NAMES:
            return

        clamped_value = self._state.snapshot().joints_deg[command]
        self._trajectory_targets.update_http_target(command, clamped_value)

    def _joint_state_callback(self, message: JointState) -> None:
        self._trajectory_targets.update_from_joint_state(
            message.name, message.position
        )

    def _on_timer(self) -> None:
        self._publish_drive_and_direct_commands()
        self._send_pending_trajectory()

    def _publish_drive_and_direct_commands(self) -> None:
        frame = self._state.next_frame()

        drive_message = Twist()
        drive_message.linear.x = frame.linear_velocity
        drive_message.angular.z = frame.angular_velocity
        self._drive_publisher.publish(drive_message)

        for joint, publisher in self._direct_joint_publishers.items():
            message = Float64()
            message.data = math.radians(frame.joints_deg[joint])
            publisher.publish(message)

    def _send_pending_trajectory(self) -> None:
        command_snapshot = self._state.snapshot()
        target = self._trajectory_targets.pending_snapshot(
            self._trajectory_debounce_sec,
            base_duration_sec=self._trajectory_duration_sec,
            speed_limits_deg_s=command_snapshot.joint_speed_limits_deg_s,
        )
        if target is None:
            missing = self._trajectory_targets.missing_feedback_joint_names()
            if missing:
                if not self._waiting_for_joint_state_reported:
                    self.get_logger().warning(
                        f"Waiting for {self._joint_state_topic} to hold "
                        "unspecified PID joints: "
                        + ", ".join(missing)
                    )
                    self._waiting_for_joint_state_reported = True
            return

        self._waiting_for_joint_state_reported = False
        if not self._trajectory_client.server_is_ready():
            if not self._action_server_unavailable_reported:
                self.get_logger().warning(
                    "Trajectory action server is not ready; retaining the latest target"
                )
                self._action_server_unavailable_reported = True
            return

        self._action_server_unavailable_reported = False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(target.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(target.positions_rad)
        point.time_from_start = Duration(
            seconds=target.duration_sec
        ).to_msg()
        goal.trajectory.points = [point]

        updated_names = ",".join(
            pending.http_name for pending in target.pending_http_targets
        )
        target_summary = ", ".join(
            f"{http_name}={math.degrees(position):.2f}deg"
            for (http_name, _), position in zip(PID_JOINTS, target.positions_rad)
        )
        self.get_logger().info(
            f"Sending trajectory goal (updated={updated_names}, "
            f"duration={target.duration_sec:.3f}s): {target_summary}"
        )

        try:
            future = self._trajectory_client.send_goal_async(goal)
        except Exception as exception:
            self.get_logger().error(f"Failed to send trajectory goal: {exception}")
            return
        self._trajectory_targets.mark_submitted(target)
        future.add_done_callback(
            partial(self._goal_response_callback, target=target)
        )

    def _goal_response_callback(
        self, future: Any, target: TrajectoryTargetSnapshot
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exception:
            self._trajectory_targets.restore_after_send_failure(target)
            self.get_logger().error(f"Failed to send trajectory goal: {exception}")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return

        self.get_logger().info("Trajectory goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future: Any) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exception:
            self.get_logger().error(f"Failed to receive trajectory result: {exception}")
            return

        if wrapped_result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                "Trajectory failed: "
                f"code={wrapped_result.result.error_code} "
                f"message={wrapped_result.result.error_string}"
            )

    def destroy_node(self) -> None:
        self._timer.cancel()
        self._http_server.shutdown()
        self._http_server.server_close()
        self._http_thread.join(timeout=2.0)

        # Scratchが先に終了しても、ノード停止時に走行指令を残さない。
        self._state.stop_drive()
        if rclpy.ok():
            self._publish_drive_and_direct_commands()
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

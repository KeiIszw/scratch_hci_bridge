from __future__ import annotations

import math
import signal
import threading
import time
from functools import partial
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint

from scratch_hci_bridge.command_state import (
    EXECUTION_MODE_COMMANDS,
    RELATIVE_JOINT_COMMANDS,
    CommandState,
    JOINT_NAMES,
    is_ros_duration_safe,
)
from scratch_hci_bridge.http_server import create_http_server
from scratch_hci_bridge.trajectory_scheduler import (
    ScheduledTrajectory,
    TrajectoryScheduler,
)
from scratch_hci_bridge.trajectory_targets import PID_JOINTS


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
        self.declare_parameter("trajectory_debounce_sec", 0.5)
        self.declare_parameter("initial_mode_command_wait_sec", 0.5)
        self.declare_parameter("joint_state_timeout_sec", 0.5)
        self.declare_parameter("trajectory_goal_tolerance_deg", 0.75)
        self.declare_parameter("trajectory_stopped_velocity_deg_s", 0.5)
        self.declare_parameter("trajectory_goal_response_timeout_sec", 5.0)
        self.declare_parameter("trajectory_result_timeout_margin_sec", 35.0)
        self.declare_parameter("trajectory_shutdown_cancel_timeout_sec", 2.0)
        self.declare_parameter("unity_position_output_enabled", False)
        self.declare_parameter("unity_position_command_prefix", "/TB20e")
        self.declare_parameter("direct_joint_command_prefix", "/TB20e")
        self.declare_parameter("swing_unity_position_sign", -1.0)
        self.declare_parameter("boom_unity_position_sign", 1.0)
        self.declare_parameter("arm_unity_position_sign", 1.0)
        self.declare_parameter("bucket_unity_position_sign", 1.0)

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
        self._initial_mode_command_wait_sec = float(
            self.get_parameter("initial_mode_command_wait_sec").value
        )
        self._joint_state_timeout_sec = float(
            self.get_parameter("joint_state_timeout_sec").value
        )
        self._trajectory_goal_tolerance_deg = float(
            self.get_parameter("trajectory_goal_tolerance_deg").value
        )
        self._trajectory_goal_tolerance_rad = math.radians(
            self._trajectory_goal_tolerance_deg
        )
        self._trajectory_stopped_velocity_rad_s = math.radians(
            float(
                self.get_parameter("trajectory_stopped_velocity_deg_s").value
            )
        )
        self._trajectory_goal_response_timeout_sec = float(
            self.get_parameter("trajectory_goal_response_timeout_sec").value
        )
        self._trajectory_result_timeout_margin_sec = float(
            self.get_parameter("trajectory_result_timeout_margin_sec").value
        )
        self._trajectory_shutdown_cancel_timeout_sec = float(
            self.get_parameter("trajectory_shutdown_cancel_timeout_sec").value
        )
        self._unity_position_output_enabled = bool(
            self.get_parameter("unity_position_output_enabled").value
        )
        unity_position_command_prefix_raw = str(
            self.get_parameter("unity_position_command_prefix").value
        )
        unity_position_command_prefix = (
            "/" + unity_position_command_prefix_raw.strip("/")
        )
        direct_joint_command_prefix_raw = str(
            self.get_parameter("direct_joint_command_prefix").value
        )
        direct_joint_command_prefix = (
            "/" + direct_joint_command_prefix_raw.strip("/")
        )
        self._unity_position_signs = {
            name: float(self.get_parameter(f"{name}_unity_position_sign").value)
            for name, _ in PID_JOINTS
        }

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
            not is_ros_duration_safe(self._trajectory_duration_sec)
            or self._trajectory_duration_sec <= 0.0
        ):
            raise ValueError(
                "trajectory_duration_sec must be greater than zero and fit "
                "in ROS Duration"
            )
        if (
            not math.isfinite(self._trajectory_debounce_sec)
            or self._trajectory_debounce_sec < 0.0
        ):
            raise ValueError(
                "trajectory_debounce_sec must be finite and non-negative"
            )
        if (
            not math.isfinite(self._initial_mode_command_wait_sec)
            or self._initial_mode_command_wait_sec < 0.0
        ):
            raise ValueError(
                "initial_mode_command_wait_sec must be finite and non-negative"
            )
        if (
            not math.isfinite(self._joint_state_timeout_sec)
            or self._joint_state_timeout_sec < 0.0
        ):
            raise ValueError(
                "joint_state_timeout_sec must be finite and non-negative"
            )
        if (
            not math.isfinite(self._trajectory_goal_tolerance_rad)
            or self._trajectory_goal_tolerance_rad <= 0.0
        ):
            raise ValueError(
                "trajectory_goal_tolerance_deg must be finite and greater than zero"
            )
        if (
            not math.isfinite(self._trajectory_stopped_velocity_rad_s)
            or self._trajectory_stopped_velocity_rad_s <= 0.0
        ):
            raise ValueError(
                "trajectory_stopped_velocity_deg_s must be finite and greater than zero"
            )
        if (
            not math.isfinite(self._trajectory_goal_response_timeout_sec)
            or self._trajectory_goal_response_timeout_sec <= 0.0
        ):
            raise ValueError(
                "trajectory_goal_response_timeout_sec must be finite and "
                "greater than zero"
            )
        if (
            not math.isfinite(self._trajectory_result_timeout_margin_sec)
            or self._trajectory_result_timeout_margin_sec < 0.0
        ):
            raise ValueError(
                "trajectory_result_timeout_margin_sec must be finite and "
                "non-negative"
            )
        if (
            not math.isfinite(self._trajectory_shutdown_cancel_timeout_sec)
            or self._trajectory_shutdown_cancel_timeout_sec < 0.0
        ):
            raise ValueError(
                "trajectory_shutdown_cancel_timeout_sec must be finite and "
                "non-negative"
            )
        if (
            self._unity_position_output_enabled
            and not unity_position_command_prefix_raw.strip("/")
        ):
            raise ValueError(
                "unity_position_command_prefix must not be empty when output is enabled"
            )
        if not direct_joint_command_prefix_raw.strip("/"):
            raise ValueError("direct_joint_command_prefix must not be empty")
        if any(
            not math.isfinite(sign) or abs(sign) != 1.0
            for sign in self._unity_position_signs.values()
        ):
            raise ValueError("Unity position signs must be either -1 or 1")

        self._state = CommandState(
            drive_linear_scale=drive_linear_scale,
            drive_angular_scale=drive_angular_scale,
        )
        self._drive_publisher = self.create_publisher(
            Twist, f"/{excavator}/tracks/cmd_vel", 10
        )
        self._direct_joint_publishers = {
            joint: self.create_publisher(
                Float64, f"{direct_joint_command_prefix}/{joint}/cmd", 10
            )
            for joint in DIRECT_JOINT_NAMES
        }
        self._unity_position_publishers = {
            http_name: self.create_publisher(
                Float64,
                f"{unity_position_command_prefix}/{http_name}/cmd",
                10,
            )
            for http_name, _ in PID_JOINTS
            if self._unity_position_output_enabled
        }
        self._trajectory_scheduler = TrajectoryScheduler(
            batch_wait_sec=self._trajectory_debounce_sec,
            initial_mode_command_wait_sec=self._initial_mode_command_wait_sec,
            completion_tolerance_deg=self._trajectory_goal_tolerance_deg,
            feedback_timeout_sec=self._joint_state_timeout_sec,
        )
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, trajectory_action
        )
        self._joint_state_subscription = self.create_subscription(
            JointState, self._joint_state_topic, self._joint_state_callback, 10
        )
        self._action_server_unavailable_reported = False
        self._waiting_for_joint_state_reported = False
        self._goal_response_future: Any | None = None
        self._goal_response_scheduled: ScheduledTrajectory | None = None
        self._goal_submission_started_at: float | None = None
        self._goal_response_timeout_reported = False
        self._active_goal_handle: Any | None = None
        self._active_goal_scheduled: ScheduledTrajectory | None = None
        self._goal_result_future: Any | None = None
        self._active_goal_deadline: float | None = None
        self._active_cancel_future: Any | None = None
        self._active_cancel_requested = False
        self._active_cancel_retry_at: float | None = None
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
            f"Publishing drive commands under /{excavator} and direct joint "
            f"commands under {direct_joint_command_prefix} at {publish_hz:g} Hz"
        )
        self.get_logger().info(
            f"Sending four-axis PID targets to {trajectory_action} "
            "(default execution mode: serial)"
        )
        if self._unity_position_output_enabled:
            self.get_logger().info(
                "Publishing accepted four-axis targets under "
                f"{unity_position_command_prefix}/<axis>/cmd"
            )

    def _command_callback(self, command: str, value: float) -> None:
        self.get_logger().info(f"HTTP command: {command}={value:g}")
        execution_mode = EXECUTION_MODE_COMMANDS.get(command)
        if execution_mode is not None:
            self._trajectory_scheduler.set_execution_mode(execution_mode)
            self.get_logger().info(
                f"Selected trajectory execution mode: {execution_mode.value}"
            )
            return
        relative_command = RELATIVE_JOINT_COMMANDS.get(command)
        if command in PID_HTTP_JOINT_NAMES:
            joint_name = command
        elif (
            relative_command is not None
            and relative_command.joint in PID_HTTP_JOINT_NAMES
        ):
            joint_name = relative_command.joint
        else:
            return

        command_snapshot = self._state.snapshot()
        self._trajectory_scheduler.update_http_target(
            command,
            value,
            speed_limit_deg_s=(
                command_snapshot.joint_speed_limits_deg_s[joint_name]
            ),
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self._trajectory_scheduler.update_from_joint_state(
            message.name, message.position
        )

    def _on_timer(self) -> None:
        self._publish_drive_and_direct_commands()
        self._check_action_timeouts()
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
        scheduled = self._trajectory_scheduler.pending_snapshot(
            base_duration_sec=self._trajectory_duration_sec,
        )
        if scheduled is None:
            missing = self._trajectory_scheduler.missing_feedback_joint_names()
            if missing:
                if not self._waiting_for_joint_state_reported:
                    self.get_logger().warning(
                        f"Waiting for {self._joint_state_topic} to hold "
                        "unspecified PID joints: "
                        + ", ".join(missing)
                    )
                    self._waiting_for_joint_state_reported = True
            return

        target = scheduled.target
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
        goal.goal_tolerance = [
            JointTolerance(
                name=joint_name,
                position=self._trajectory_goal_tolerance_rad,
                velocity=self._trajectory_stopped_velocity_rad_s,
            )
            for joint_name in target.joint_names
        ]

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
            f"Sending {scheduled.execution_mode.value} trajectory goal "
            f"(updated={updated_names}, "
            f"duration={target.duration_sec:.3f}s): {target_summary}"
        )

        if not self._trajectory_scheduler.mark_submitted(scheduled):
            self.get_logger().error(
                "Trajectory queue changed before the goal could be tracked"
            )
            return
        try:
            future = self._trajectory_client.send_goal_async(goal)
        except Exception as exception:
            self._trajectory_scheduler.mark_send_failed(scheduled)
            self.get_logger().error(f"Failed to send trajectory goal: {exception}")
            return
        self._goal_response_future = future
        self._goal_response_scheduled = scheduled
        self._goal_submission_started_at = time.monotonic()
        self._goal_response_timeout_reported = False
        future.add_done_callback(
            partial(self._goal_response_callback, scheduled=scheduled)
        )

    def _publish_unity_position_target(self, scheduled: ScheduledTrajectory) -> None:
        if not self._unity_position_publishers:
            return
        positions_by_joint = dict(
            zip(
                scheduled.target.joint_names,
                scheduled.target.positions_rad,
            )
        )
        for http_name, ros_name in PID_JOINTS:
            position_rad = positions_by_joint.get(ros_name)
            if position_rad is None or not math.isfinite(position_rad):
                self.get_logger().error(
                    f"Unity position target is missing or invalid for {ros_name}"
                )
                return
        for http_name, ros_name in PID_JOINTS:
            message = Float64()
            message.data = (
                self._unity_position_signs[http_name]
                * positions_by_joint[ros_name]
            )
            self._unity_position_publishers[http_name].publish(message)

    def _goal_response_callback(
        self, future: Any, scheduled: ScheduledTrajectory
    ) -> None:
        if (
            future is not self._goal_response_future
            or scheduled is not self._goal_response_scheduled
        ):
            self.get_logger().warning(
                "Ignored a stale trajectory goal response callback"
            )
            return

        submission_started_at = self._goal_submission_started_at
        self._goal_response_future = None
        self._goal_response_scheduled = None
        self._goal_submission_started_at = None
        self._goal_response_timeout_reported = False
        try:
            goal_handle = future.result()
        except Exception as exception:
            self._trajectory_scheduler.mark_send_failed(scheduled)
            self.get_logger().error(f"Failed to send trajectory goal: {exception}")
            return

        if not goal_handle.accepted:
            self._trajectory_scheduler.mark_completed(
                scheduled, successful=False
            )
            self.get_logger().error(
                "Trajectory goal was rejected; cleared queued trajectory commands"
            )
            return

        self._active_goal_handle = goal_handle
        self._active_goal_scheduled = scheduled
        self._active_cancel_future = None
        self._active_cancel_requested = False
        self._active_cancel_retry_at = None
        self._publish_unity_position_target(scheduled)
        if submission_started_at is None:
            submission_started_at = time.monotonic()
        self._active_goal_deadline = (
            submission_started_at
            + scheduled.target.duration_sec
            + self._trajectory_result_timeout_margin_sec
        )
        self.get_logger().info("Trajectory goal accepted")
        try:
            result_future = goal_handle.get_result_async()
        except Exception as exception:
            self.get_logger().error(
                "Failed to request trajectory result; retaining the serial "
                f"gate and cancelling the goal: {exception}"
            )
            self._request_active_goal_cancel("result request failed")
            return
        self._goal_result_future = result_future
        result_future.add_done_callback(
            partial(self._goal_result_callback, scheduled=scheduled)
        )

    def _goal_result_callback(
        self, future: Any, scheduled: ScheduledTrajectory
    ) -> None:
        if (
            future is not self._goal_result_future
            or scheduled is not self._active_goal_scheduled
        ):
            self.get_logger().warning(
                "Ignored a stale trajectory result callback"
            )
            return

        try:
            wrapped_result = future.result()
        except Exception as exception:
            self._goal_result_future = None
            self.get_logger().error(
                "Failed to receive trajectory result; retaining the serial "
                f"gate and cancelling the goal: {exception}"
            )
            self._request_active_goal_cancel("result response failed")
            return

        successful = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped_result.result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )
        self._clear_active_goal_tracking()
        self._trajectory_scheduler.mark_completed(
            scheduled, successful=successful
        )
        if successful:
            self.get_logger().info(
                f"Completed {scheduled.execution_mode.value} trajectory goal"
            )
            return

        self.get_logger().error(
            "Trajectory failed; cleared queued trajectory commands: "
            f"status={wrapped_result.status} "
            f"code={wrapped_result.result.error_code} "
            f"message={wrapped_result.result.error_string}"
        )

    def _check_action_timeouts(self, now: float | None = None) -> None:
        current_time = time.monotonic() if now is None else now
        if (
            self._goal_response_future is not None
            and self._goal_submission_started_at is not None
            and not self._goal_response_timeout_reported
            and current_time - self._goal_submission_started_at
            >= self._trajectory_goal_response_timeout_sec
        ):
            self._goal_response_timeout_reported = True
            self.get_logger().error(
                "Timed out waiting for the trajectory goal response; the "
                "server-side goal state is unknown, so the serial gate remains "
                "closed"
            )

        cancel_retry_due = (
            self._active_cancel_retry_at is not None
            and current_time >= self._active_cancel_retry_at
        )
        result_timeout_due = (
            self._active_goal_deadline is not None
            and current_time >= self._active_goal_deadline
        )
        if (
            self._active_goal_handle is not None
            and not self._active_cancel_requested
            and (cancel_retry_due or result_timeout_due)
        ):
            if cancel_retry_due:
                self.get_logger().warning(
                    "Retrying cancellation of the active trajectory goal"
                )
                reason = "cancellation retry"
            else:
                self.get_logger().error(
                    "Timed out waiting for the trajectory result; cancelling the "
                    "goal and retaining the serial gate until a terminal result"
                )
                reason = "trajectory result timed out"
            self._request_active_goal_cancel(reason)

    def _request_active_goal_cancel(self, reason: str) -> Any | None:
        if self._active_goal_handle is None:
            return None
        if self._active_cancel_requested:
            return self._active_cancel_future

        self._active_cancel_requested = True
        try:
            cancel_future = self._active_goal_handle.cancel_goal_async()
        except Exception as exception:
            self._active_cancel_requested = False
            self._active_cancel_retry_at = time.monotonic() + 1.0
            self.get_logger().error(
                f"Failed to cancel the active trajectory goal ({reason}): "
                f"{exception}; the serial gate remains closed and cancellation "
                "will be retried"
            )
            return None

        self._active_cancel_future = cancel_future
        self._active_cancel_retry_at = None
        cancel_future.add_done_callback(self._cancel_response_callback)
        self.get_logger().warning(
            f"Requested cancellation of the active trajectory goal ({reason})"
        )
        return cancel_future

    def _cancel_response_callback(self, future: Any) -> None:
        if future is not self._active_cancel_future:
            return
        try:
            response = future.result()
        except Exception as exception:
            self._active_cancel_future = None
            self._active_cancel_requested = False
            self._active_cancel_retry_at = time.monotonic() + 1.0
            self.get_logger().error(
                "Failed to receive the trajectory cancellation response: "
                f"{exception}; the serial gate remains closed and cancellation "
                "will be retried"
            )
            return

        if getattr(response, "goals_canceling", ()):
            self.get_logger().info(
                "Trajectory cancellation was accepted; waiting for the "
                "terminal result"
            )
            return
        self.get_logger().error(
            "Trajectory cancellation was not accepted; the serial gate "
            "remains closed until a terminal result"
        )

    def _clear_active_goal_tracking(self) -> None:
        self._active_goal_handle = None
        self._active_goal_scheduled = None
        self._goal_result_future = None
        self._active_goal_deadline = None
        self._active_cancel_future = None
        self._active_cancel_requested = False
        self._active_cancel_retry_at = None

    def _wait_for_future_until(
        self, future: Any | None, deadline: float, description: str
    ) -> bool:
        if future is None:
            return False
        try:
            if future.done():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not rclpy.ok():
                return False
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=remaining,
            )
            return bool(future.done())
        except Exception as exception:
            self.get_logger().warning(
                f"Failed while waiting for {description}: {exception}"
            )
            return False

    def _resolve_goal_response_during_shutdown(self, deadline: float) -> bool:
        response_future = self._goal_response_future
        response_scheduled = self._goal_response_scheduled
        if response_future is None:
            return True
        response_completed = self._wait_for_future_until(
            response_future,
            deadline,
            "the trajectory goal response during shutdown",
        )
        if (
            response_completed
            and response_future is self._goal_response_future
            and response_scheduled is not None
        ):
            # rclpy marks a Future done before its queued done-callback is
            # necessarily executed. Resolve it here so an accepted goal can
            # still be cancelled during the bounded shutdown window.
            self._goal_response_callback(
                response_future,
                response_scheduled,
            )
        return response_completed

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

        cancel_deadline = (
            time.monotonic() + self._trajectory_shutdown_cancel_timeout_sec
        )
        if self._goal_response_future is not None:
            if not self._resolve_goal_response_during_shutdown(cancel_deadline):
                self.get_logger().warning(
                    "Shutdown timed out before the trajectory goal response; "
                    "the server-side goal could not be cancelled"
                )

        if self._active_goal_handle is not None:
            cancel_future = self._request_active_goal_cancel("node shutdown")
            if cancel_future is not None and not self._wait_for_future_until(
                cancel_future,
                cancel_deadline,
                "trajectory cancellation during shutdown",
            ):
                self.get_logger().warning(
                    "Shutdown timed out waiting for trajectory cancellation"
                )
            result_future = self._goal_result_future
            result_scheduled = self._active_goal_scheduled
            if result_future is not None:
                result_completed = self._wait_for_future_until(
                    result_future,
                    cancel_deadline,
                    "the terminal trajectory result during shutdown",
                )
                if (
                    result_completed
                    and result_future is self._goal_result_future
                    and result_scheduled is not None
                ):
                    self._goal_result_callback(
                        result_future,
                        result_scheduled,
                    )
                elif not result_completed:
                    self.get_logger().warning(
                        "Shutdown timed out before the active trajectory became "
                        "terminal"
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

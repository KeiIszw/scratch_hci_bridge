"""HTTP状態とROS Actionを接続し、停止・置換・軸許可を一つのロックで同期する。"""
from __future__ import annotations

import math
import signal
import threading
import time
from functools import partial, wraps
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, UInt8, Int8
from trajectory_msgs.msg import JointTrajectoryPoint

from scratch_hci_bridge.command_state import (
    ABSOLUTE_JOINT_COMMANDS,
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


def _motion_serialized(method):
    """HTTPとROSの状態遷移を原子的にし、送信と停止の競合を防ぐ。"""
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        lock = getattr(self, "_motion_lock", None)
        if lock is None:
            return method(self, *args, **kwargs)
        with lock:
            return method(self, *args, **kwargs)
    return wrapped


class ScratchHciBridge(Node):
    def __init__(self) -> None:
        super().__init__("scratch_hci_bridge")

        self._motion_lock = threading.RLock()
        self._stop_pending = False
        self._lease_enabled = False
        self._last_http_at = time.monotonic()
        self._mask_commanded_axes: set[str] = set()
        self._mask_catch_up_targets: dict[str, float] = {}
        self._last_active_axis_mask = 0
        for name, default in (("client_lease_sec", 2.0), ("max_dispatch_delay_sec", 1.0)):
            self.declare_parameter(name, default)
        self._client_lease_sec = float(self.get_parameter("client_lease_sec").value)
        if not math.isfinite(self._client_lease_sec) or self._client_lease_sec <= 0:
            raise ValueError("client_lease_sec must be finite and positive")
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
        for joint in ("swing", "boom", "arm", "bucket"):
            self.declare_parameter(f"{joint}_default_speed_deg_s", 0.0)
        self.declare_parameter("trajectory_duration_sec", 3.0)
        self.declare_parameter("joint_state_timeout_sec", 0.5)
        self.declare_parameter("trajectory_goal_tolerance_deg", 0.75)
        self.declare_parameter("trajectory_stopped_velocity_deg_s", 0.5)
        self.declare_parameter("trajectory_goal_response_timeout_sec", 5.0)
        self.declare_parameter("trajectory_result_timeout_margin_sec", 35.0)
        self.declare_parameter("trajectory_shutdown_cancel_timeout_sec", 2.0)
        self.declare_parameter("unity_position_output_enabled", False)
        self.declare_parameter("unity_position_command_prefix", "/TB20e")
        self.declare_parameter("direct_joint_command_prefix", "/TB20e")
        self.declare_parameter("active_axis_mask_topic", "")
        self.declare_parameter("catch_up_axes_in_mask", True)
        self._catch_up_axes_in_mask = self.get_parameter("catch_up_axes_in_mask").value
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
        active_axis_mask_topic = str(
            self.get_parameter("active_axis_mask_topic").value
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
            default_joint_speeds_deg_s={
                joint: float(self.get_parameter(f"{joint}_default_speed_deg_s").value)
                for joint in ("swing", "boom", "arm", "bucket")
            },
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
        self._active_axis_mask_publisher = (
            self.create_publisher(
                UInt8,
                active_axis_mask_topic,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            if active_axis_mask_topic
            else None
        )
        self._publish_active_axis_mask()
        self._unity_position_publishers = {
            http_name: self.create_publisher(
                Float64,
                f"{unity_position_command_prefix}/{http_name}/cmd",
                10,
            )
            for http_name, _ in PID_JOINTS
            if self._unity_position_output_enabled
        }
        ground_timeout = self.declare_parameter('ground_timeout_sec', 0.5).value
        ground_prefix = self.declare_parameter('ground_topic_prefix', '/TB20e_0/ground').value.rstrip('/')
        self._ground_clearance, self._ground_clearance_at = float('nan'), float('-inf')
        self._ground_timeout_sec = ground_timeout
        self._trajectory_scheduler = TrajectoryScheduler(
            ground_timeout_sec=ground_timeout,
            feedback_timeout_sec=self._joint_state_timeout_sec,
            **{name: float(self.get_parameter(name).value) for name in
               ("max_dispatch_delay_sec",)},
        )
        self._ground_contact_subscription = self.create_subscription(
            Int8, ground_prefix+'/contact', self._ground_contact_callback, 10)
        self._ground_clearance_subscription = self.create_subscription(
            Float64, ground_prefix+'/clearance', self._ground_clearance_callback, 10)
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
        self._active_goal_planned_end: float | None = None
        self._replacement_previous: ScheduledTrajectory | None = None
        self._replacement_deferred_result: tuple[Any, ScheduledTrajectory] | None = None
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
            # HTTPの完了確認は、同じschedulerが管理するキューとAction状態を参照する。
            self._trajectory_scheduler.motion_status,
            self._trajectory_scheduler.motion_error,
            self._http_activity,
            self._motion_lock,
            joint_queue_callback=self._trajectory_scheduler.joint_queue,
            ground_reporter_callback=self._ground_reporter,
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
            "(joint timeline)"
        )
        if self._unity_position_output_enabled:
            self.get_logger().info(
                "Publishing accepted four-axis targets under "
                f"{unity_position_command_prefix}/<axis>/cmd"
            )

    @_motion_serialized
    def _command_callback(self, command: str, value: float) -> bool | None:
        previous_error = self._trajectory_scheduler.motion_error()
        try:
            self.get_logger().info(f"HTTP command: {command}={value:g}")
            if command in ("stop", "reset"):
                self._stop_motion(reset=command == "reset")
                return
            if command == "barrier":
                self._trajectory_scheduler.barrier()
                return
            if command in ("left_crawler_speed", "right_crawler_speed"):
                self._trajectory_scheduler.accept_motion()
            if command == 'boom_down_until_contact':
                snapshot = self._state.snapshot()
                accepted = self._trajectory_scheduler.start_ground_guard(value,
                    snapshot.joint_speed_limits_deg_s['boom'], self._trajectory_duration_sec)
                if not accepted:
                    # HTTP拡張が400を無視しても、guard_statusとmotion_statusから異常を検出する。
                    self._stop_motion('ground_guard_rejected')
                return accepted
            relative_command = RELATIVE_JOINT_COMMANDS.get(command)
            absolute_joint = ABSOLUTE_JOINT_COMMANDS.get(command)
            if absolute_joint in DIRECT_JOINT_NAMES:
                self._trajectory_scheduler.accept_motion()
            if absolute_joint in PID_HTTP_JOINT_NAMES:
                joint_name = absolute_joint
            elif (
                relative_command is not None
                and relative_command.joint in PID_HTTP_JOINT_NAMES
            ):
                joint_name = relative_command.joint
            else:
                return

            command_snapshot = self._state.snapshot()
            return self._trajectory_scheduler.update_http_target(
                command,
                value,
                speed_limit_deg_s=(
                    command_snapshot.joint_speed_limits_deg_s[joint_name]
                ),
                base_duration_sec=self._trajectory_duration_sec,
            )
        finally:
            # HTTP受付中に検知した失敗も最初の原因をログへ残す。
            self._log_new_motion_failure(previous_error)

    @_motion_serialized
    def _ground_contact_callback(self, message):
        self._trajectory_scheduler.update_ground_contact(message.data)
        reason = self._trajectory_scheduler.check_ground_guard()
        if reason:
            self._stop_motion(reason)
        else:
            self._refresh_active_axis_mask()

    @_motion_serialized
    def _ground_clearance_callback(self, message):
        self._ground_clearance, self._ground_clearance_at = message.data, time.monotonic()

    @_motion_serialized
    def _ground_reporter(self, command):
        if command == 'guard_status':
            return self._trajectory_scheduler.guard_status()
        if command == 'ground_contact':
            return {-1: 'unknown', 0: '0', 1: '1'}[self._trajectory_scheduler.ground_contact()]
        now = time.monotonic()
        value = self._ground_clearance
        return f'{value:.3f}' if math.isfinite(value) and 0 <= now-self._ground_clearance_at <= self._ground_timeout_sec else 'unknown'

    def _joint_state_callback(self, message: JointState) -> None:
        self._trajectory_scheduler.update_from_joint_state(
            message.name, message.position
        )

    @_motion_serialized
    def _http_activity(self, heartbeat: bool) -> None:
        self._last_http_at = time.monotonic()
        if heartbeat:
            self._lease_enabled = True

    def _log_new_motion_failure(self, previous_error: str) -> None:
        error = self._trajectory_scheduler.motion_error()
        if error not in ("", "stopped") and error != previous_error:
            self._publish_active_axis_mask()
            self.get_logger().error(
                f"Motion failed: reason={error} {self._trajectory_scheduler.motion_failure_details()}"
            )

    def _stop_motion(self, reason: str = "stopped", *, reset: bool = False) -> None:
        """HTTP応答は終端結果を待たず、cancelの要求だけを行う。"""
        previous_error = self._trajectory_scheduler.motion_error()
        self._trajectory_scheduler.stop(reason, reset=reset)
        self._log_new_motion_failure(previous_error)
        self._state.stop_drive()
        self._stop_pending = bool(self._goal_response_future or self._active_goal_handle)
        self._publish_active_axis_mask()
        self._request_active_goal_cancel(reason)

    @_motion_serialized
    def _on_timer(self) -> None:
        now = time.monotonic()
        if (self._lease_enabled and now - self._last_http_at > self._client_lease_sec
                and (self._trajectory_scheduler.has_work() or self._mask_catch_up_targets)):
            self._stop_motion("lease_expired")
        if self._mask_catch_up_targets and self._trajectory_scheduler.stale_feedback(now):
            self._stop_motion("stale")
        previous_error = self._trajectory_scheduler.motion_error()
        self._trajectory_scheduler.check_deadlines(now)
        reason = self._trajectory_scheduler.check_ground_guard(now)
        if reason:
            self._stop_motion(reason)
        self._log_new_motion_failure(previous_error)
        if self._trajectory_scheduler.needs_cancel() and not self._stop_pending:
            self._stop_motion(self._trajectory_scheduler.motion_error())
        self._publish_drive_and_direct_commands()
        self._check_action_timeouts()
        self._send_pending_trajectory()
        self._refresh_active_axis_mask()

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
        if self._goal_response_future is not None or self._stop_pending:
            return
        replacing = None
        if self._active_goal_handle is not None:
            if self._active_goal_scheduled is None or self._active_cancel_requested:
                return
            replacing = self._active_goal_scheduled
        scheduled = self._trajectory_scheduler.pending_snapshot(
            replacing=replacing,
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
        commanded_ros_joints = {
            ros_name
            for http_name, ros_name in PID_JOINTS
            if any(
                command.joint_name == http_name
                for command in scheduled.commands
            )
        }
        goal.goal_tolerance = [
            JointTolerance(
                name=joint_name,
                position=(
                    self._trajectory_goal_tolerance_rad
                    if joint_name in commanded_ros_joints
                    else -1.0
                ),
                velocity=(
                    self._trajectory_stopped_velocity_rad_s
                    if joint_name in commanded_ros_joints
                    else -1.0
                ),
            )
            for joint_name in target.joint_names
        ]

        # 全関節の境界を含む計画を送り、置換時にも実測へ巻き戻さない。
        goal.trajectory.points = []
        for planned in scheduled.track_points:
            point = JointTrajectoryPoint()
            point.positions = list(planned.positions)
            point.velocities = list(planned.velocities)
            point.time_from_start = Duration(seconds=planned.time).to_msg()
            goal.trajectory.points.append(point)

        updated_names = ",".join(command.http_name for command in scheduled.commands)
        target_summary = ", ".join(
            f"{http_name}={math.degrees(position):.2f}deg"
            for (http_name, _), position in zip(PID_JOINTS, target.positions_rad)
        )
        self.get_logger().info(
            f"Sending joint timeline goal "
            f"(updated={updated_names}, "
            f"duration={target.duration_sec:.3f}s): {target_summary}"
        )

        if replacing is None and not self._trajectory_scheduler.mark_submitted(scheduled):
            self.get_logger().error(
                "Trajectory queue changed before the goal could be tracked"
            )
            return
        previous_mask_axes = self._mask_commanded_axes.copy()
        previous_catch_up_targets = self._mask_catch_up_targets.copy()
        self._publish_active_axis_mask(scheduled)
        try:
            future = self._trajectory_client.send_goal_async(goal)
        except Exception as exception:
            if replacing is None:
                self._trajectory_scheduler.mark_send_failed(scheduled)
            self._stop_pending = False
            # 送信自体が失敗した置換では旧goalが動いているため、その追従許可を戻す。
            self._mask_commanded_axes = previous_mask_axes
            self._mask_catch_up_targets = previous_catch_up_targets
            self._refresh_active_axis_mask()
            self.get_logger().error(f"Failed to send trajectory goal: {exception}")
            return
        self._replacement_previous = replacing
        self._replacement_deferred_result = None
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

    def _publish_active_axis_mask(
        self, scheduled: ScheduledTrajectory | None = None
    ) -> None:
        # マスクは保持補正で無関係な軸を動かさないための実機側の出力制限である。
        # 区間置換で未到達軸まで止めないよう、指令軸に保持目標への追従軸を加える。
        publisher = getattr(self, "_active_axis_mask_publisher", None)
        if publisher is None:
            return
        self._mask_commanded_axes = (
            {command.joint_name for command in scheduled.commands}
            if scheduled is not None
            else set()
        )
        if scheduled is not None and scheduled.track_points:
            self._mask_commanded_axes = self._trajectory_scheduler.track_active_axes()
        self._mask_catch_up_targets = (
            self._trajectory_scheduler.targets_outside_tolerance(
                self._trajectory_scheduler.tracking_targets() if scheduled.track_points else dict(zip(
                    (axis for axis, _ in PID_JOINTS), scheduled.target.positions_rad)),
                self._trajectory_goal_tolerance_rad)
            if scheduled is not None and self._catch_up_axes_in_mask else {}
        )
        self._write_active_axis_mask()

    def _refresh_active_axis_mask(self) -> None:
        if getattr(self, "_active_axis_mask_publisher", None) is None:
            return
        if self._active_goal_scheduled is not None and self._active_goal_scheduled.track_points and not self._trajectory_scheduler.needs_cancel():
            self._mask_commanded_axes = self._trajectory_scheduler.track_active_axes()
        # 一度到達した軸は次の指令まで再許可せず、無関係な保持補正を再開しない。
        if self._mask_catch_up_targets:
            self._mask_catch_up_targets = self._trajectory_scheduler.targets_outside_tolerance(
                self._mask_catch_up_targets, self._trajectory_goal_tolerance_rad,
            )
        self._write_active_axis_mask(only_if_changed=True)

    def _write_active_axis_mask(self, *, only_if_changed: bool = False) -> None:
        active_joints = self._mask_commanded_axes | self._mask_catch_up_targets.keys()
        scheduler = getattr(self, '_trajectory_scheduler', None)
        blocks_old_boom = getattr(scheduler, 'guard_blocks_old_boom', None)
        if blocks_old_boom and blocks_old_boom(getattr(self, '_active_goal_scheduled', None)):
            # 接地後にAction応答が遅れても、旧目標への追従出力で下降を続けない。
            active_joints.discard('boom')
        mask = 0
        for bit, (http_name, _) in enumerate(PID_JOINTS):
            if http_name in active_joints:
                mask |= 1 << bit
        # idle時の再publishで、単独replayが所有するマスクを上書きしない。
        if only_if_changed and mask == self._last_active_axis_mask:
            return
        self._last_active_axis_mask = mask
        message = UInt8()
        message.data = mask
        self._active_axis_mask_publisher.publish(message)

    @_motion_serialized
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
        replacing = self._replacement_previous
        self._replacement_previous = None
        deferred_result = self._replacement_deferred_result
        self._replacement_deferred_result = None
        self._goal_response_future = None
        self._goal_response_scheduled = None
        self._goal_submission_started_at = None
        self._goal_response_timeout_reported = False
        try:
            goal_handle = future.result()
        except Exception as exception:
            if replacing is not None:
                # 新ゴールの受理が不明なので、旧ゴールを止めても再送のゲートは開けない。
                self._goal_response_future = future
                self._goal_response_scheduled = scheduled
                self._goal_submission_started_at = submission_started_at
                self._goal_response_timeout_reported = True
                self._stop_motion("result_timeout")
            else:
                self._trajectory_scheduler.mark_send_failed(scheduled)
                self._stop_pending = False
                # 再送可能な通信失敗では、既に送った目標への追従許可を失わせない。
                self._mask_commanded_axes.clear()
                self._refresh_active_axis_mask()
            self.get_logger().error(f"Failed to send trajectory goal: {exception}")
            return

        if not goal_handle.accepted:
            if replacing is not None:
                # 拒否だけでは旧ゴールは止まらないため、cancelと終端待ちを維持する。
                self._stop_motion("goal_rejected")
                if deferred_result is not None:
                    self._goal_result_callback(*deferred_result)
            else:
                self._stop_pending = False
                self._trajectory_scheduler.mark_completed(
                    scheduled, successful=False, reason="goal_rejected"
                )
            self._publish_active_axis_mask()
            self.get_logger().error(
                "Trajectory goal was rejected; reason=goal_rejected "
                + self._trajectory_scheduler.goal_diagnostics(scheduled)
            )
            return

        if replacing is not None:
            self._trajectory_scheduler.mark_submitted(scheduled, replacing=replacing)
        self._active_goal_handle = goal_handle
        self._active_goal_scheduled = scheduled
        self._active_cancel_future = None
        self._active_cancel_requested = False
        self._active_cancel_retry_at = None
        if replacing is not None and self._trajectory_scheduler.needs_cancel():
            self._stop_motion(self._trajectory_scheduler.motion_error())
        if not getattr(self, "_stop_pending", False):
            is_guard_hold = getattr(self._trajectory_scheduler, 'is_guard_hold', None)
            if is_guard_hold and is_guard_hold(scheduled):
                # 受理前のドリフトも保持位置へ戻せるよう、新goalの実測差で許可を再評価する。
                self._publish_active_axis_mask(scheduled)
            self._publish_unity_position_target(scheduled)
        if submission_started_at is None:
            submission_started_at = time.monotonic()
        self._active_goal_planned_end = submission_started_at + scheduled.target.duration_sec
        self._active_goal_deadline = (
            self._active_goal_planned_end + self._trajectory_result_timeout_margin_sec
        )
        self.get_logger().info("Trajectory goal accepted")
        try:
            result_future = goal_handle.get_result_async()
        except Exception as exception:
            self.get_logger().error(
                "Failed to request trajectory result; retaining the goal "
                f"gate and cancelling the goal: {exception}"
            )
            self._request_active_goal_cancel("result request failed")
            return
        self._goal_result_future = result_future
        result_future.add_done_callback(
            partial(self._goal_result_callback, scheduled=scheduled)
        )
        if getattr(self, "_stop_pending", False):
            self._request_active_goal_cancel("stopped before acceptance")

    @_motion_serialized
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

        if self._replacement_previous is scheduled:
            # JTCの旧resultは新goalの応答より先にも届くため、受理が確定するまで保留する。
            self._replacement_deferred_result = (future, scheduled)
            return

        try:
            wrapped_result = future.result()
        except Exception as exception:
            self._goal_result_future = None
            self.get_logger().error(
                "Failed to receive trajectory result; retaining the goal "
                f"gate and cancelling the goal: {exception}"
            )
            self._request_active_goal_cancel("result response failed")
            return

        successful = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped_result.result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )
        reason = ("tolerance_violated" if wrapped_result.result.error_code in (
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
        ) else "goal_aborted")
        diagnostics = self._trajectory_scheduler.goal_diagnostics(scheduled)
        self._clear_active_goal_tracking()
        self._stop_pending = False
        # 途中目標の未到達分を次区間で取り戻せるよう、指令基準を保って先へ進む。
        self._trajectory_scheduler.mark_completed(
            scheduled, successful=successful, reason=reason
        )
        if successful:
            self._mask_commanded_axes.clear()
            self._refresh_active_axis_mask()
        else:
            self._publish_active_axis_mask()
        if successful:
            self.get_logger().info(
                f"Completed joint timeline goal"
            )
            return
        message = (
            f"reason={reason} {diagnostics} status={wrapped_result.status} "
            f"code={wrapped_result.result.error_code} "
            "message=" + " ".join(wrapped_result.result.error_string.splitlines())
        )
        self.get_logger().error("Trajectory failed; cleared queued trajectory commands: " + message)

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
            self._stop_motion("result_timeout")
            self.get_logger().error(
                "Timed out waiting for the trajectory goal response; the "
                "server-side goal state is unknown, so the goal gate remains "
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
                    "goal and retaining the goal gate until a terminal result"
                )
                reason = "trajectory result timed out"
                self._stop_motion("result_timeout")
                return
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
                f"{exception}; the goal gate remains closed and cancellation "
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

    @_motion_serialized
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
                f"{exception}; the goal gate remains closed and cancellation "
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
            "Trajectory cancellation was not accepted; the goal gate "
            "remains closed until a terminal result"
        )

    def _clear_active_goal_tracking(self) -> None:
        self._active_goal_handle = None
        self._active_goal_scheduled = None
        self._goal_result_future = None
        self._active_goal_deadline = None
        self._active_goal_planned_end = None
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
            # rclpyはキュー内の完了コールバックが実行される前にFutureを完了状態にする。
            # ここで結果を解決し、受理済みのゴールを
            # 終了処理の制限時間内にキャンセルできるようにする。
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
            self._publish_active_axis_mask()
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

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from enum import Enum


JOINT_NAMES = ("blade", "body", "swing", "boom", "arm", "bucket", "thumb")
PID_JOINT_NAMES = ("swing", "boom", "arm", "bucket")
JOINT_SPEED_COMMANDS = {
    f"{joint}_speed": joint for joint in PID_JOINT_NAMES
}
# builtin_interfaces/Duration stores whole seconds in a signed 32-bit field.
# Keep generated durations at or below this exactly representable boundary.
MAX_ROS_DURATION_SEC = float((1 << 31) - 1)
_MAX_PID_JOINT_TRAVEL_DEG = 360.0
MAX_RELATIVE_SWING_DEG = 180.0


class ExecutionMode(str, Enum):
    SERIAL = "serial"
    PARALLEL = "parallel"


EXECUTION_MODE_COMMANDS = {
    mode.value: mode for mode in ExecutionMode
}


@dataclass(frozen=True)
class RelativeJointCommand:
    joint: str
    direction: float


# Use the physical direction names from OCS20250118, mapped onto OperaSim's
# joint coordinates. The HTTP value is a magnitude; the name sets direction.
RELATIVE_JOINT_COMMANDS = {
    # swing_joint rotates the vehicle upper body about ROS +Z. Positive yaw is
    # physically left and negative yaw is physically right in OperaSim.
    "swing_left": RelativeJointCommand("swing", 1.0),
    "swing_right": RelativeJointCommand("swing", -1.0),
    "boom_up": RelativeJointCommand("boom", -1.0),
    "boom_down": RelativeJointCommand("boom", 1.0),
    "arm_extend": RelativeJointCommand("arm", -1.0),
    "arm_retract": RelativeJointCommand("arm", 1.0),
    "bucket_dump": RelativeJointCommand("bucket", -1.0),
    "bucket_scoop": RelativeJointCommand("bucket", 1.0),
}

INITIAL_JOINT_DEG = {
    "blade": 0.0,
    "body": 0.0,
    "swing": 0.0,
    "boom": -83.0,
    "arm": 155.0,
    "bucket": 89.0,
    "thumb": 0.0,
}

JOINT_LIMITS_DEG = {
    "blade": (-30.0, 15.0),
    "swing": (-180.0, 180.0),
    "boom": (-83.0, 48.0),
    "arm": (32.0, 155.0),
    "bucket": (-31.0, 159.0),
    "thumb": (0.0, 140.0),
}

SUPPORTED_COMMANDS = (
    frozenset(JOINT_NAMES)
    | frozenset(JOINT_SPEED_COMMANDS)
    | frozenset(EXECUTION_MODE_COMMANDS)
    | frozenset(RELATIVE_JOINT_COMMANDS)
    | frozenset(("left_crawler_speed", "right_crawler_speed"))
)


@dataclass(frozen=True)
class CommandSnapshot:
    joints_deg: dict[str, float]
    joint_speed_limits_deg_s: dict[str, float | None]
    execution_mode: ExecutionMode
    left_crawler_speed: float
    right_crawler_speed: float


@dataclass(frozen=True)
class PublishFrame:
    linear_velocity: float
    angular_velocity: float
    joints_deg: dict[str, float]


class CommandState:
    """Thread-safe command state shared by HTTP and ROS callback threads."""

    def __init__(
        self,
        drive_linear_scale: float = 1.0,
        drive_angular_scale: float = 1.0,
    ) -> None:
        if not all(
            math.isfinite(value)
            for value in (drive_linear_scale, drive_angular_scale)
        ):
            raise ValueError("drive scales must be finite")

        self._lock = threading.Lock()
        self._joints_deg = dict(INITIAL_JOINT_DEG)
        self._joint_speed_limits_deg_s = {
            joint: None for joint in PID_JOINT_NAMES
        }
        self._execution_mode = ExecutionMode.SERIAL
        self._left_crawler_speed = 0.0
        self._right_crawler_speed = 0.0
        self._drive_linear_scale = drive_linear_scale
        self._drive_angular_scale = drive_angular_scale

    def apply_command(self, command: str, value: float) -> bool:
        if not math.isfinite(value):
            return False
        with self._lock:
            if command in self._joints_deg:
                self._joints_deg[command] = _clamp_joint(command, value)
                return True
            if command in RELATIVE_JOINT_COMMANDS:
                # Relative PID targets are resolved from fresh joint feedback by
                # TrajectoryScheduler when their batch is dispatched.
                return is_safe_relative_swing_command(command, value)
            speed_joint = JOINT_SPEED_COMMANDS.get(command)
            if speed_joint is not None:
                if value < 0.0 or (
                    value > 0.0 and not is_safe_pid_speed_limit(value)
                ):
                    return False
                self._joint_speed_limits_deg_s[speed_joint] = (
                    value if value > 0.0 else None
                )
                return True
            execution_mode = EXECUTION_MODE_COMMANDS.get(command)
            if execution_mode is not None:
                # The HCI block always supplies val, but mode commands use it only
                # to satisfy the common HTTP command shape.
                self._execution_mode = execution_mode
                return True
            if command == "left_crawler_speed":
                if not self._drive_is_finite(value, self._right_crawler_speed):
                    return False
                self._left_crawler_speed = value
                return True
            if command == "right_crawler_speed":
                if not self._drive_is_finite(self._left_crawler_speed, value):
                    return False
                self._right_crawler_speed = value
                return True
        return False

    def next_frame(self) -> PublishFrame:
        with self._lock:
            linear_velocity, angular_velocity = self._drive_velocities(
                self._left_crawler_speed,
                self._right_crawler_speed,
            )

            return PublishFrame(
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
                joints_deg=dict(self._joints_deg),
            )

    def stop_drive(self) -> None:
        with self._lock:
            self._left_crawler_speed = 0.0
            self._right_crawler_speed = 0.0

    def snapshot(self) -> CommandSnapshot:
        with self._lock:
            return CommandSnapshot(
                joints_deg=dict(self._joints_deg),
                joint_speed_limits_deg_s=dict(self._joint_speed_limits_deg_s),
                execution_mode=self._execution_mode,
                left_crawler_speed=self._left_crawler_speed,
                right_crawler_speed=self._right_crawler_speed,
            )

    def _drive_is_finite(self, left: float, right: float) -> bool:
        return all(math.isfinite(value) for value in self._drive_velocities(left, right))

    def _drive_velocities(self, left: float, right: float) -> tuple[float, float]:
        # 半分にしてから加減算し、中間計算のoverflowを避ける。
        linear = (right / 2.0 + left / 2.0) * self._drive_linear_scale
        angular = (right / 2.0 - left / 2.0) * self._drive_angular_scale
        return linear, angular


def _clamp_joint(joint: str, value: float) -> float:
    limit = JOINT_LIMITS_DEG.get(joint)
    if limit is None:
        return value
    return max(limit[0], min(limit[1], value))


def is_ros_duration_safe(duration_sec: float) -> bool:
    """Return whether seconds fit safely in builtin_interfaces/Duration."""
    return (
        math.isfinite(duration_sec)
        and 0.0 <= duration_sec <= MAX_ROS_DURATION_SEC
    )


def is_safe_pid_speed_limit(speed_limit_deg_s: float) -> bool:
    """Validate a positive speed against the worst-case PID joint travel."""
    if not math.isfinite(speed_limit_deg_s) or speed_limit_deg_s <= 0.0:
        return False
    duration_sec = _MAX_PID_JOINT_TRAVEL_DEG / speed_limit_deg_s
    return is_ros_duration_safe(duration_sec)


def is_safe_relative_swing_command(command: str, degrees: float) -> bool:
    """Reject relative swing travel that a continuous joint could reverse."""
    relative_command = RELATIVE_JOINT_COMMANDS.get(command)
    return (
        math.isfinite(degrees)
        and (
            relative_command is None
            or relative_command.joint != "swing"
            or abs(degrees) < MAX_RELATIVE_SWING_DEG
        )
    )

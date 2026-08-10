from __future__ import annotations

import math
import threading
from dataclasses import dataclass


JOINT_NAMES = ("blade", "body", "swing", "boom", "arm", "bucket", "thumb")

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
    "swing": (-49.0, 78.0),
    "boom": (-83.0, 48.0),
    "arm": (32.0, 155.0),
    "bucket": (-31.0, 159.0),
    "thumb": (0.0, 140.0),
}

SUPPORTED_COMMANDS = frozenset(JOINT_NAMES) | frozenset(
    ("left_crawler_speed", "right_crawler_speed")
)


@dataclass(frozen=True)
class CommandSnapshot:
    joints_deg: dict[str, float]
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

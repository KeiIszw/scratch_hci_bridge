from __future__ import annotations

import math
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass


from scratch_hci_bridge.command_state import ABSOLUTE_JOINT_COMMANDS


PID_JOINTS = (
    ("swing", "swing_joint"),
    ("boom", "boom_joint"),
    ("arm", "arm_joint"),
    ("bucket", "bucket_joint"),
)
PID_HTTP_TO_ROS = dict(PID_JOINTS)
PID_ROS_TO_HTTP = {ros_name: http_name for http_name, ros_name in PID_JOINTS}


@dataclass(frozen=True)
class PendingHttpTarget:
    http_name: str
    degrees: float
    revision: int


@dataclass(frozen=True)
class TrajectoryTargetSnapshot:
    revision: int
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    duration_sec: float
    pending_http_targets: tuple[PendingHttpTarget, ...]


class TrajectoryTargets:
    """Build each goal from this HTTP batch and the latest joint feedback."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._feedback_positions_rad: dict[str, float] = {}
        self._pending_targets: dict[str, PendingHttpTarget] = {}
        self._revision = 0
        self._last_http_update = 0.0

    def update_http_target(
        self, command: str, degrees: float, now: float | None = None
    ) -> bool:
        joint = ABSOLUTE_JOINT_COMMANDS.get(command)
        if joint not in PID_HTTP_TO_ROS or not math.isfinite(degrees):
            return False

        with self._lock:
            self._revision += 1
            self._pending_targets[joint] = PendingHttpTarget(
                http_name=joint,
                degrees=degrees,
                revision=self._revision,
            )
            self._last_http_update = time.monotonic() if now is None else now
        return True

    def update_from_joint_state(
        self, names: Sequence[str], positions_rad: Sequence[float]
    ) -> None:
        with self._lock:
            for name, position_rad in zip(names, positions_rad):
                http_name = PID_ROS_TO_HTTP.get(name)
                if http_name is None or not math.isfinite(position_rad):
                    continue
                self._feedback_positions_rad[http_name] = position_rad

    def pending_snapshot(
        self,
        debounce_sec: float,
        base_duration_sec: float = 3.0,
        speed_limits_deg_s: dict[str, float | None] | None = None,
        now: float | None = None,
    ) -> TrajectoryTargetSnapshot | None:
        if not math.isfinite(base_duration_sec) or base_duration_sec <= 0.0:
            raise ValueError("base_duration_sec must be finite and positive")

        current_time = time.monotonic() if now is None else now
        speed_limits = speed_limits_deg_s or {}
        with self._lock:
            if not self._pending_targets:
                return None
            if current_time - self._last_http_update < debounce_sec:
                return None
            if any(
                http_name not in self._feedback_positions_rad
                for http_name, _ in PID_JOINTS
            ):
                return None

            positions_rad: list[float] = []
            duration_sec = 0.0
            for http_name, _ in PID_JOINTS:
                pending = self._pending_targets.get(http_name)
                if pending is not None:
                    target_position = math.radians(pending.degrees)
                    current_position = self._feedback_positions_rad[http_name]
                    if http_name == "swing":
                        target_position = current_position + _shortest_angular_delta(
                            target_position, current_position
                        )
                    positions_rad.append(target_position)

                    speed_limit = speed_limits.get(http_name)
                    if speed_limit is None:
                        required_duration = base_duration_sec
                    else:
                        if not math.isfinite(speed_limit) or speed_limit <= 0.0:
                            raise ValueError(
                                f"{http_name} speed limit must be finite and positive"
                            )
                        position_delta = target_position - current_position
                        required_duration = math.degrees(
                            abs(position_delta)
                        ) / speed_limit
                    duration_sec = max(duration_sec, required_duration)
                    continue

                positions_rad.append(self._feedback_positions_rad[http_name])

            pending_targets = tuple(
                sorted(
                    self._pending_targets.values(),
                    key=lambda target: target.revision,
                )
            )
            return TrajectoryTargetSnapshot(
                revision=self._revision,
                joint_names=tuple(ros_name for _, ros_name in PID_JOINTS),
                positions_rad=tuple(positions_rad),
                duration_sec=duration_sec,
                pending_http_targets=pending_targets,
            )

    def mark_submitted(self, snapshot: TrajectoryTargetSnapshot) -> None:
        with self._lock:
            for submitted in snapshot.pending_http_targets:
                current = self._pending_targets.get(submitted.http_name)
                if current is not None and current.revision <= snapshot.revision:
                    self._pending_targets.pop(submitted.http_name)

    def restore_after_send_failure(
        self, snapshot: TrajectoryTargetSnapshot, now: float | None = None
    ) -> None:
        with self._lock:
            for submitted in snapshot.pending_http_targets:
                current = self._pending_targets.get(submitted.http_name)
                if current is None or current.revision < submitted.revision:
                    self._pending_targets[submitted.http_name] = submitted
            self._last_http_update = time.monotonic() if now is None else now

    def missing_feedback_joint_names(self) -> tuple[str, ...]:
        with self._lock:
            if not self._pending_targets:
                return ()
            return tuple(
                http_name
                for http_name, _ in PID_JOINTS
                if http_name not in self._feedback_positions_rad
            )


def _shortest_angular_delta(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, replace
from decimal import Decimal

from scratch_hci_bridge.command_state import (
    ExecutionMode,
    JOINT_LIMITS_DEG,
    ABSOLUTE_JOINT_COMMANDS,
    RELATIVE_JOINT_COMMANDS,
    is_safe_relative_swing_command,
    is_ros_duration_safe,
    is_safe_pid_speed_limit,
)
from scratch_hci_bridge.trajectory_targets import (
    PID_HTTP_TO_ROS,
    PID_JOINTS,
    PID_ROS_TO_HTTP,
    PendingHttpTarget,
    TrajectoryTargetSnapshot,
)


_MIN_TRAJECTORY_DURATION_SEC = 0.001


@dataclass(frozen=True)
class QueuedMotionCommand:
    http_name: str
    joint_name: str
    degrees: float
    revision: int
    relative: bool = False
    speed_limit_deg_s: float | None = None


@dataclass
class _MotionBatch:
    batch_id: int
    mode: ExecutionMode
    commands: list[QueuedMotionCommand]
    ready_at: float | None = None
    wait_stopped: bool = True


@dataclass(frozen=True)
class ScheduledTrajectory:
    dispatch_id: int
    batch_id: int
    execution_mode: ExecutionMode
    commands: tuple[QueuedMotionCommand, ...]
    target: TrajectoryTargetSnapshot
    feedback_revisions: tuple[int, ...]
    wait_stopped: bool = True
    start_positions_rad: tuple[float, ...] = ()
    start_velocities_rad_s: tuple[float, ...] = ()
    end_velocities_rad_s: tuple[float, ...] = ()


class TrajectoryScheduler:
    """Queue PID-joint HTTP commands as serial or parallel action goals."""

    def __init__(
        self,
        batch_wait_sec: float = 0.5,
        initial_mode_command_wait_sec: float = 0.5,
        completion_tolerance_deg: float = 0.75,
        feedback_timeout_sec: float = 0.5,
        max_dispatch_delay_sec: float = 1.0,
        batch_open_timeout_sec: float = 5.0,
    ) -> None:
        if not math.isfinite(batch_wait_sec) or batch_wait_sec < 0.0:
            raise ValueError("batch_wait_sec must be finite and non-negative")
        if (
            not math.isfinite(initial_mode_command_wait_sec)
            or initial_mode_command_wait_sec < 0.0
        ):
            raise ValueError(
                "initial_mode_command_wait_sec must be finite and non-negative"
            )
        if (
            not math.isfinite(completion_tolerance_deg)
            or completion_tolerance_deg <= 0.0
        ):
            raise ValueError(
                "completion_tolerance_deg must be finite and positive"
            )
        if (
            not math.isfinite(feedback_timeout_sec)
            or feedback_timeout_sec < 0.0
        ):
            raise ValueError(
                "feedback_timeout_sec must be finite and non-negative"
            )

        for value in (max_dispatch_delay_sec, batch_open_timeout_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("safety parameters must be finite and positive")
        self._max_dispatch_delay_sec = max_dispatch_delay_sec
        self._batch_open_timeout_sec = batch_open_timeout_sec
        self._explicit_last_command_at = None
        self._last_error = ""
        self._failure_details = ""
        self._stopped = False
        self._interrupting = False
        self._lock = threading.RLock()
        self._batch_wait_sec = batch_wait_sec
        self._initial_mode_command_wait_sec = initial_mode_command_wait_sec
        self._feedback_timeout_sec = feedback_timeout_sec
        self._feedback_positions_rad: dict[str, float] = {}
        self._feedback_revisions: dict[str, int] = {}
        self._feedback_updated_at: dict[str, float] = {}
        self._commanded_targets_rad: dict[str, float] = {}
        self._feedback_revision = 0
        self._selected_mode = ExecutionMode.SERIAL
        self._startup_mode_window_open = True
        self._building_batch: _MotionBatch | None = None
        self._building_deadline: float | None = None
        self._ready_batches: deque[_MotionBatch] = deque()
        self._in_flight: ScheduledTrajectory | None = None
        self._completed_continuation: ScheduledTrajectory | None = None
        self._retry_trajectory: ScheduledTrajectory | None = None
        self._revision = 0
        self._next_batch_id = 0
        self._next_dispatch_id = 0
        # 再生側が失敗を見逃さないよう、resetまでAction失敗を保持する。
        self._motion_failed = False

    @property
    def selected_execution_mode(self) -> ExecutionMode:
        with self._lock:
            return self._selected_mode

    def set_execution_mode(
        self, mode: ExecutionMode | str, now: float | None = None
    ) -> bool:
        try:
            parsed_mode = ExecutionMode(mode)
        except ValueError:
            return False

        current_time = time.monotonic() if now is None else now
        with self._lock:
            self._seal_expired_building_batch(current_time)
            if self._explicit_last_command_at is not None:
                self._explicit_last_command_at = current_time
                return True
            # 旧HCI版ではfetchの完了を待っていなかった。OCSの実装と同様に、
            # 要求が逆順に届いた場合、最初の明示的なモードで未確定のバッチを引き継ぐ。
            # 引き継いだバッチを未確定のまま保持し、モード指定の後に届く
            # 動作要求も同じバッチに追加できるようにする。
            if self._startup_mode_window_open and self._building_batch is not None:
                self._building_batch.mode = parsed_mode
                partitions = self._partition_parallel_swing_batch(
                    self._building_batch
                )
                if len(partitions) > 1:
                    if not self._ready_batches and self._in_flight is None:
                        partitions[0].ready_at = current_time
                    self._ready_batches.extend(partitions[:-1])
                    self._building_batch = partitions[-1]
                self._selected_mode = parsed_mode
                self._startup_mode_window_open = False
                self._building_deadline = current_time + self._batch_wait_sec
                return True

            self._seal_building_batch(current_time)
            self._selected_mode = parsed_mode
            self._startup_mode_window_open = False
        return True

    def update_http_target(
        self,
        command: str,
        degrees: float,
        speed_limit_deg_s: float | None = None,
        now: float | None = None,
    ) -> bool:
        relative_command = RELATIVE_JOINT_COMMANDS.get(command)
        absolute_joint = ABSOLUTE_JOINT_COMMANDS.get(command)
        if absolute_joint in PID_HTTP_TO_ROS:
            joint_name = absolute_joint
            queued_degrees = degrees
            relative = False
        elif (
            relative_command is not None
            and relative_command.joint in PID_HTTP_TO_ROS
        ):
            joint_name = relative_command.joint
            queued_degrees = relative_command.direction * abs(degrees)
            relative = True
        else:
            return False
        if not math.isfinite(degrees):
            return False
        if not is_safe_relative_swing_command(command, degrees):
            return False
        if speed_limit_deg_s is not None and not is_safe_pid_speed_limit(
            speed_limit_deg_s
        ):
            return False

        current_time = time.monotonic() if now is None else now
        with self._lock:
            self._seal_expired_building_batch(current_time)
            if self._explicit_last_command_at is not None:
                self._explicit_last_command_at = current_time
            self._stopped = False
            if self._building_batch is None:
                self._next_batch_id += 1
                self._building_batch = _MotionBatch(
                    batch_id=self._next_batch_id,
                    mode=self._selected_mode,
                    commands=[],
                )

            if (
                self._explicit_last_command_at is None
                and joint_name == "swing"
                and self._building_batch.mode == ExecutionMode.PARALLEL
                and any(
                    command.joint_name == "swing"
                    for command in self._building_batch.commands
                )
            ):
                # 連続回転関節では、半回転の位置で最短方向が一意に決まらない。
                # 各旋回指令を個別のゴールに分けることで、並列指令の合算や
                # 絶対・相対指令の混在で半回転を越えないようにする。
                self._seal_building_batch(current_time)
                self._next_batch_id += 1
                self._building_batch = _MotionBatch(
                    batch_id=self._next_batch_id,
                    mode=self._selected_mode,
                    commands=[],
                )

            self._revision += 1
            self._building_batch.commands.append(
                QueuedMotionCommand(
                    http_name=command,
                    joint_name=joint_name,
                    degrees=queued_degrees,
                    revision=self._revision,
                    relative=relative,
                    speed_limit_deg_s=speed_limit_deg_s,
                )
            )
            wait_sec = self._batch_wait_sec
            if self._startup_mode_window_open:
                wait_sec = max(wait_sec, self._initial_mode_command_wait_sec)
            self._building_deadline = current_time + wait_sec
        return True

    def update_from_joint_state(
        self,
        names: Sequence[str],
        positions_rad: Sequence[float],
        now: float | None = None,
    ) -> None:
        current_time = time.monotonic() if now is None else now
        with self._lock:
            updates: list[tuple[str, float]] = []
            for name, position_rad in zip(names, positions_rad):
                http_name = PID_ROS_TO_HTTP.get(name)
                if http_name is None or not math.isfinite(position_rad):
                    continue
                updates.append((http_name, position_rad))
            if not updates:
                return
            self._feedback_revision += 1
            for http_name, position_rad in updates:
                self._feedback_positions_rad[http_name] = position_rad
                self._feedback_revisions[http_name] = self._feedback_revision
                self._feedback_updated_at[http_name] = current_time

    def pending_snapshot(
        self,
        base_duration_sec: float = 3.0,
        now: float | None = None,
        replacing: ScheduledTrajectory | None = None,
    ) -> ScheduledTrajectory | None:
        if not is_ros_duration_safe(base_duration_sec) or base_duration_sec <= 0.0:
            raise ValueError("base_duration_sec must be positive and fit in ROS Duration")

        current_time = time.monotonic() if now is None else now
        with self._lock:
            self._seal_expired_building_batch(current_time)
            if self._interrupting or not self._ready_batches:
                return None
            index = 0
            if replacing is not None:
                # commit 0の確定済み後続だけを先読みし、受理までは旧ゴールを保持する。
                if self._in_flight != replacing or replacing.wait_stopped:
                    return None
                index = 1
                if len(self._ready_batches) <= index:
                    return None
            elif self._in_flight is not None:
                return None
            batch = self._ready_batches[index]
            if batch.ready_at is not None and current_time - batch.ready_at > self._max_dispatch_delay_sec:
                self.stop("stale")
                return None
            if any(not self._feedback_is_fresh(axis, current_time) for axis, _ in PID_JOINTS):
                return None
            if replacing is None and self._retry_trajectory is not None:
                retry = self._retry_trajectory
                if self._matches_ready_batch(retry):
                    return self._build_retry_trajectory(retry, base_duration_sec)
                self._retry_trajectory = None

            basis = tuple(
                self._relative_basis(axis, self._feedback_positions_rad[axis])
                for axis, _ in PID_JOINTS
            )
            scheduled = self._plan_batch(batch, basis, base_duration_sec)
            previous = replacing or self._completed_continuation
            if previous is not None or not scheduled.wait_stopped:
                following = None
                if not scheduled.wait_stopped and len(self._ready_batches) > index + 1:
                    following = self._plan_batch(
                        self._ready_batches[index + 1],
                        scheduled.target.positions_rad, base_duration_sec,
                    )
                if previous is not None or following is not None:
                    slope = _segment_velocity(basis, scheduled.target)
                    # 先読み済みの境界速度を引き継ぎ、未確定だった境界は新区間の速度で始める。
                    start_velocity = (
                        previous.end_velocities_rad_s or slope
                    ) if previous else (0.0,) * len(PID_JOINTS)
                    start_velocity = _boundary_velocity(start_velocity, slope)
                    end_velocity = (
                        _boundary_velocity(slope, _segment_velocity(
                            scheduled.target.positions_rad, following.target,
                        )) if following else (0.0,) * len(PID_JOINTS)
                    )
                    scheduled = replace(
                        scheduled, start_positions_rad=basis,
                        start_velocities_rad_s=start_velocity,
                        end_velocities_rad_s=end_velocity,
                    )
            self._next_dispatch_id += 1
            return replace(scheduled, dispatch_id=self._next_dispatch_id)

    def _plan_batch(
        self, batch: _MotionBatch, basis: tuple[float, ...], base_duration_sec: float,
    ) -> ScheduledTrajectory:
        commands = tuple(batch.commands[:1] if batch.mode == ExecutionMode.SERIAL else batch.commands)
        positions = []
        pending = []
        duration = 0.0
        for index, (axis, _) in enumerate(PID_JOINTS):
            selected = [command for command in commands if command.joint_name == axis]
            position = _resolve_target_position_rad(axis, basis[index], selected) if selected else basis[index]
            positions.append(position)
            if selected:
                # 実測の追従遅れで再生時間まで延びないよう、指令目標間の距離で計画する。
                duration = max(duration, _required_duration_sec(
                    axis, selected, position, basis[index], base_duration_sec,
                ))
                pending.append(PendingHttpTarget(
                    selected[-1].http_name, math.degrees(position), selected[-1].revision,
                ))
        target = TrajectoryTargetSnapshot(
            revision=max(command.revision for command in commands),
            joint_names=tuple(name for _, name in PID_JOINTS), positions_rad=tuple(positions),
            duration_sec=duration,
            pending_http_targets=tuple(sorted(pending, key=lambda item: item.revision)),
        )
        return ScheduledTrajectory(
            dispatch_id=0, batch_id=batch.batch_id, execution_mode=batch.mode,
            commands=commands, target=target, wait_stopped=batch.wait_stopped,
            feedback_revisions=tuple(self._feedback_revisions[axis] for axis, _ in PID_JOINTS),
        )

    def mark_submitted(
        self, scheduled: ScheduledTrajectory, replacing: ScheduledTrajectory | None = None,
    ) -> bool:
        with self._lock:
            if replacing is not None:
                if self._in_flight != replacing or replacing.wait_stopped:
                    return False
                self._seal_expired_building_batch(time.monotonic())
                if self._interrupting:
                    # stop中に受理された置換ゴールもcancelし、終端までゲートを閉じる。
                    self._in_flight = scheduled
                    return True
                if not self._matches_ready_batch(scheduled, index=1):
                    return False
                self.mark_completed(replacing, successful=True)
            if self._in_flight is not None or not self._matches_ready_batch(scheduled):
                return False
            self._in_flight = scheduled
            self._completed_continuation = None
            # 到達判定と独立に送信した目標を保持し、未到達分を次区間へ持ち越す。
            self._commanded_targets_rad = dict(zip(
                (axis for axis, _ in PID_JOINTS), scheduled.target.positions_rad,
            ))
        return True

    def mark_send_failed(self, scheduled: ScheduledTrajectory) -> bool:
        """Release a goal that failed before an action response was possible."""
        with self._lock:
            if (
                self._in_flight is None
                or self._in_flight.dispatch_id != scheduled.dispatch_id
            ):
                return False
            # 指令は正常に完了するまで実行待ちキューの先頭に保持する。
            # 解決済みの絶対目標を保持し、結果が不明な通信障害によって
            # 相対オフセットが二重に適用されないようにする。
            if self._interrupting:
                self._interrupting = False
                self._retry_trajectory = None
                if self._ready_batches:
                    self._ready_batches[0].ready_at = time.monotonic()
            else:
                self._retry_trajectory = scheduled
            self._in_flight = None
        return True

    def mark_completed(
        self, scheduled: ScheduledTrajectory, *, successful: bool,
        preserve_queued: bool = False,
        now: float | None = None, reason: str = "goal_aborted",
    ) -> bool:
        with self._lock:
            if (
                self._in_flight is None
                or self._in_flight.dispatch_id != scheduled.dispatch_id
            ):
                return False

            # タイマーが動かなかった間に確定した待機バッチにも、終端時刻を起点として与える。
            self._seal_expired_building_batch(time.monotonic() if now is None else now)
            if self._interrupting:
                self._in_flight = None
                self._interrupting = False
                if self._ready_batches:
                    self._ready_batches[0].ready_at = time.monotonic() if now is None else now
                return True
            if not successful:
                self._record_failure(reason, scheduled)
                self._commanded_targets_rad.clear()
                self._completed_continuation = None
                # 待機キューを破棄しても、HTTP reporterでは失敗が分かるようにする。
                self._motion_failed = True

            if not successful and not preserve_queued:
                # 先行指令が失敗した場合、後続の直列指令は実行しない。
                # 待機中の処理を破棄して選択中のモードは保持し、後続のHTTP指令で
                # ノードを再起動せずに新しいバッチを開始できるようにする。
                self._ready_batches.clear()
                self._building_batch = None
                self._building_deadline = None
                self._explicit_last_command_at = None
                self._in_flight = None
                self._retry_trajectory = None
                return True

            if not self._matches_ready_batch(scheduled):
                self._ready_batches.clear()
                self._building_batch = None
                self._building_deadline = None
                self._explicit_last_command_at = None
                self._in_flight = None
                self._retry_trajectory = None
                return False

            batch = self._ready_batches[0]
            if batch.mode == ExecutionMode.SERIAL:
                batch.commands.pop(0)
            else:
                batch.commands.clear()
            if not batch.commands:
                self._ready_batches.popleft()
            # 結果が境界タイマーより先に届いても、確定済み後続へ同じ速度を引き継ぐ。
            self._completed_continuation = (
                scheduled if successful and not scheduled.wait_stopped and self._ready_batches else None
            )
            self._in_flight = None
            self._retry_trajectory = None
            # 送信可能になった時刻から、後続の送信待ち期限を数える。
            if self._ready_batches:
                self._ready_batches[0].ready_at = time.monotonic() if now is None else now
        return True

    def targets_outside_tolerance(
        self, targets_rad: dict[str, float], tolerance_rad: float,
    ) -> dict[str, float]:
        """JointStateと同じロックで比較し、追従許可の解除に一貫した実測を使う。"""
        with self._lock:
            remaining = {}
            for axis, target in targets_rad.items():
                measured = self._feedback_positions_rad[axis]
                delta = target - measured
                if axis == "swing":
                    # 連続関節の同値角を未到達と誤判定して出力を許可し続けない。
                    delta = (delta + math.pi) % (2 * math.pi) - math.pi
                if abs(delta) > tolerance_rad:
                    remaining[axis] = target
            return remaining

    def missing_feedback_joint_names(
        self, now: float | None = None
    ) -> tuple[str, ...]:
        current_time = time.monotonic() if now is None else now
        with self._lock:
            if self._in_flight is not None or not self._ready_batches:
                return ()
            return tuple(
                http_name
                for http_name, _ in PID_JOINTS
                if not self._feedback_is_fresh(http_name, current_time)
            )

    def motion_queue(self) -> int:
        """実行中の指令を除き、構築中と未送信のバッチを数える。"""
        with self._lock:
            count = len(self._ready_batches) + int(self._building_batch is not None)
            if self._in_flight is not None and self._ready_batches:
                batch = self._ready_batches[0]
                # serialバッチの未送信の残りがある場合は、そのバッチを残す。
                if batch.batch_id == self._in_flight.batch_id and len(batch.commands) == len(self._in_flight.commands):
                    count -= 1
            return count

    def motion_status(self) -> int:
        """動作状態を返す。0=待機、1=処理中、-1=失敗、2=停止済み。"""
        with self._lock:
            if self._motion_failed:
                return -1
            if self._interrupting and self._in_flight is not None:
                return 1
            if self._stopped:
                return 2
            # 送信前のバッチ集約・待機キューも処理中に含める。
            # 受付直後、Action送信前の短い時間を「完了」と誤判定させない。
            return int(bool(self._building_batch or self._ready_batches or self._in_flight))

    def _relative_basis(self, joint: str, measured: float) -> float:
        """未到達分を失わないよう、停止・致命的失敗までは指令目標を使う。"""
        return self._commanded_targets_rad.get(joint, measured)

    def needs_cancel(self) -> bool:
        with self._lock:
            return self._interrupting

    def has_work(self) -> bool:
        with self._lock:
            return bool(self._building_batch or self._ready_batches or self._in_flight)

    def accept_motion(self) -> None:
        with self._lock:
            self._stopped = False

    def motion_error(self) -> str:
        with self._lock:
            return self._last_error

    def stop(self, reason: str = "stopped", *, reset: bool = False) -> None:
        """送信待ちを破棄し、送信済みゴールの終端まではゲートを保持する。"""
        with self._lock:
            if not reset and reason != "stopped":
                self._record_failure(reason)
            self._ready_batches.clear()
            self._building_batch = None
            self._building_deadline = None
            self._explicit_last_command_at = None
            self._retry_trajectory = None
            self._commanded_targets_rad.clear()
            self._completed_continuation = None
            self._interrupting = self._in_flight is not None
            if reset:
                self._last_error = ""
                self._failure_details = ""
            elif reason == "stopped":
                if not self._last_error:
                    self._last_error = reason
            self._motion_failed = self._last_error not in ("", "stopped")
            self._stopped = not reset and reason == "stopped"

    def touch_explicit_batch(self, now: float | None = None) -> None:
        """速度設定もバッチ構築に必要なので、reporterとは分けて通信期限を更新する。"""
        with self._lock:
            current = time.monotonic() if now is None else now
            self._seal_expired_building_batch(current)
            if self._explicit_last_command_at is not None:
                self._explicit_last_command_at = current

    def _record_failure(self, reason: str, scheduled: ScheduledTrajectory | None = None) -> None:
        # Scratchが失敗検知後にstopを送っても、resetまでは最初の原因を残す。
        if self._last_error in ("", "stopped"):
            self._last_error = reason
            self._failure_details = self.goal_diagnostics(scheduled)

    def motion_failure_details(self) -> str:
        with self._lock:
            return self._failure_details

    def goal_diagnostics(self, scheduled: ScheduledTrajectory | None = None) -> str:
        """実測を目標で上書きせず、失敗時の指令軸ごとの角度差を残す。"""
        with self._lock:
            scheduled = scheduled or self._in_flight
            batch = self._building_batch or (self._ready_batches[0] if self._ready_batches else None)
            commands = scheduled.commands if scheduled else (batch.commands if batch else ())
            details = []
            for i, (axis, _) in enumerate(PID_JOINTS):
                selected = [c for c in commands if c.joint_name == axis]
                if not selected:
                    continue
                measured = self._feedback_positions_rad.get(axis)
                target = scheduled.target.positions_rad[i] if scheduled else (
                    _resolve_target_position_rad(axis, self._relative_basis(axis, measured), selected)
                    if measured is not None else None
                )
                def degrees(value):
                    return "unavailable" if value is None else f"{math.degrees(value):.3f}"
                delta = None if measured is None or target is None else (
                    _shortest_angular_delta(target, measured) if axis == "swing" else target - measured
                )
                details.append(f"axis={axis} target_deg={degrees(target)} measured_deg={degrees(measured)} delta_deg={degrees(delta)}")
            return "; ".join(details) or "axes=none"

    def begin(self, now: float | None = None) -> None:
        """明示バッチにはdebounceを適用しない。"""
        with self._lock:
            current = time.monotonic() if now is None else now
            self._seal_expired_building_batch(current)
            if self._explicit_last_command_at is None:
                self._seal_building_batch(current)
            self._next_batch_id += 1
            self._building_batch = _MotionBatch(self._next_batch_id, ExecutionMode.PARALLEL, [])
            self._explicit_last_command_at = current
            self._stopped = False

    def commit(self, wait_stopped: bool, now: float | None = None) -> None:
        with self._lock:
            current = time.monotonic() if now is None else now
            self._seal_expired_building_batch(current)
            if self._explicit_last_command_at is None:
                return
            self._building_batch.wait_stopped = wait_stopped
            self._explicit_last_command_at = None
            self._seal_building_batch(current)

    def check_deadlines(self, now: float | None = None) -> None:
        with self._lock:
            current = time.monotonic() if now is None else now
            self._seal_expired_building_batch(current)
            if self._in_flight is None and self._ready_batches:
                ready = self._ready_batches[0].ready_at
                if ready is not None and current - ready > self._max_dispatch_delay_sec:
                    self.stop("stale")

    def queued_command_count(self) -> int:
        with self._lock:
            ready_count = sum(
                len(batch.commands) for batch in self._ready_batches
            )
            building_count = (
                len(self._building_batch.commands)
                if self._building_batch is not None
                else 0
            )
            return ready_count + building_count

    def _build_retry_trajectory(
        self,
        retry: ScheduledTrajectory,
        base_duration_sec: float,
    ) -> ScheduledTrajectory:
        commands_by_joint: dict[str, list[QueuedMotionCommand]] = {
            http_name: [] for http_name, _ in PID_JOINTS
        }
        for command in retry.commands:
            commands_by_joint[command.joint_name].append(command)

        positions_rad: list[float] = []
        duration_sec = 0.0
        for index, (http_name, _) in enumerate(PID_JOINTS):
            joint_commands = commands_by_joint[http_name]
            current_position = self._feedback_positions_rad[http_name]
            if joint_commands:
                target_position = retry.target.positions_rad[index]
                duration_sec = max(
                    duration_sec,
                    _required_duration_sec(
                        http_name,
                        joint_commands,
                        target_position,
                        current_position,
                        base_duration_sec,
                    ),
                )
            else:
                target_position = self._relative_basis(http_name, current_position)
            positions_rad.append(target_position)

        target = TrajectoryTargetSnapshot(
            revision=retry.target.revision,
            joint_names=retry.target.joint_names,
            positions_rad=tuple(positions_rad),
            duration_sec=duration_sec,
            pending_http_targets=retry.target.pending_http_targets,
        )
        self._next_dispatch_id += 1
        return ScheduledTrajectory(
            dispatch_id=self._next_dispatch_id,
            batch_id=retry.batch_id,
            execution_mode=retry.execution_mode,
            commands=retry.commands,
            wait_stopped=retry.wait_stopped,
            target=target,
            feedback_revisions=tuple(
                self._feedback_revisions[http_name]
                for http_name, _ in PID_JOINTS
            ),
        )

    def _matches_ready_batch(self, scheduled: ScheduledTrajectory, index: int = 0) -> bool:
        if len(self._ready_batches) <= index:
            return False
        batch = self._ready_batches[index]
        if (
            batch.batch_id != scheduled.batch_id
            or batch.mode != scheduled.execution_mode
        ):
            return False
        expected = (
            tuple(batch.commands[:1])
            if batch.mode == ExecutionMode.SERIAL
            else tuple(batch.commands)
        )
        return expected == scheduled.commands

    def _feedback_is_fresh(self, http_name: str, now: float) -> bool:
        if http_name not in self._feedback_positions_rad:
            return False
        if self._feedback_timeout_sec == 0.0:
            return True
        updated_at = self._feedback_updated_at[http_name]
        # 経過時間が負になるのは、擬似時計を使うテストの場合だけである。
        return now <= updated_at or now - updated_at <= self._feedback_timeout_sec

    def _seal_expired_building_batch(self, now: float) -> None:
        if self._explicit_last_command_at is not None:
            if now - self._explicit_last_command_at > self._batch_open_timeout_sec:
                self.stop("stale")
            return
        if (
            self._building_batch is not None
            and self._building_deadline is not None
            and now >= self._building_deadline
        ):
            self._seal_building_batch(self._building_deadline)

    def _seal_building_batch(self, now: float | None = None) -> None:
        if self._building_batch is not None and self._building_batch.commands:
            batches = self._partition_parallel_swing_batch(self._building_batch)
            for batch in batches:
                batch.ready_at = None
                batch.wait_stopped = self._building_batch.wait_stopped
            if not self._ready_batches and self._in_flight is None:
                batches[0].ready_at = time.monotonic() if now is None else now
            self._ready_batches.extend(batches)
            self._startup_mode_window_open = False
        self._building_batch = None
        self._building_deadline = None

    def _partition_parallel_swing_batch(
        self, batch: _MotionBatch
    ) -> list[_MotionBatch]:
        """Return ordered parallel batches containing at most one swing command."""
        if batch.mode != ExecutionMode.PARALLEL:
            return [batch]

        command_groups: list[list[QueuedMotionCommand]] = [[]]
        group_has_swing = False
        for command in batch.commands:
            if command.joint_name == "swing" and group_has_swing:
                command_groups.append([])
                group_has_swing = False
            command_groups[-1].append(command)
            if command.joint_name == "swing":
                group_has_swing = True
        if len(command_groups) == 1:
            return [batch]

        partitions = [
            _MotionBatch(
                batch_id=batch.batch_id,
                mode=batch.mode,
                commands=command_groups[0],
            )
        ]
        for commands in command_groups[1:]:
            self._next_batch_id += 1
            partitions.append(
                _MotionBatch(
                    batch_id=self._next_batch_id,
                    mode=batch.mode,
                    commands=commands,
                )
            )
        return partitions


def _shortest_angular_delta(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def _resolve_target_position_rad(
    http_name: str,
    current_position_rad: float,
    commands: Sequence[QueuedMotionCommand],
) -> float:
    absolute_command = next(
        (command for command in reversed(commands) if not command.relative),
        None,
    )
    if http_name == "swing":
        relative_offset_degrees = sum(
            (
                Decimal.from_float(command.degrees)
                for command in commands
                if command.relative
            ),
            Decimal(0),
        )
        if absolute_command is None:
            return current_position_rad + math.radians(
                float(relative_offset_degrees)
            )
        lower_deg, upper_deg = JOINT_LIMITS_DEG[http_name]
        absolute_degrees = max(
            lower_deg, min(upper_deg, absolute_command.degrees)
        )
        absolute_position = math.radians(absolute_degrees)
        shortest_absolute_position = (
            current_position_rad
            + _shortest_angular_delta(absolute_position, current_position_rad)
        )
        return shortest_absolute_position + math.radians(
            float(relative_offset_degrees)
        )

    origin_degrees = (
        Decimal.from_float(absolute_command.degrees)
        if absolute_command is not None
        else Decimal.from_float(math.degrees(current_position_rad))
    )
    relative_offset_degrees = sum(
        (
            Decimal.from_float(command.degrees)
            for command in commands
            if command.relative
        ),
        Decimal(0),
    )
    target_degrees = origin_degrees + relative_offset_degrees
    lower_deg, upper_deg = JOINT_LIMITS_DEG[http_name]
    clamped_degrees = max(
        Decimal.from_float(lower_deg),
        min(Decimal.from_float(upper_deg), target_degrees),
    )
    return math.radians(float(clamped_degrees))


def _required_duration_sec(
    http_name: str,
    commands: Sequence[QueuedMotionCommand],
    target_position_rad: float,
    current_position_rad: float,
    base_duration_sec: float,
) -> float:
    relative_commands = [command for command in commands if command.relative]
    absolute_command = next(
        (command for command in reversed(commands) if not command.relative),
        None,
    )
    contributing_commands = list(relative_commands)
    if absolute_command is not None:
        contributing_commands.append(absolute_command)

    explicit_speed_limits = [
        command.speed_limit_deg_s
        for command in contributing_commands
        if command.speed_limit_deg_s is not None
    ]
    required_duration = (
        base_duration_sec
        if any(
            command.speed_limit_deg_s is None
            for command in contributing_commands
        )
        else 0.0
    )
    if explicit_speed_limits:
        speed_limit = min(explicit_speed_limits)
        if not is_safe_pid_speed_limit(speed_limit):
            raise ValueError(
                f"{http_name} speed limit cannot produce a ROS Duration"
            )
        distance_rad = _position_error_rad(
            http_name, target_position_rad, current_position_rad
        )
        speed_duration = math.degrees(distance_rad) / speed_limit
        if not is_ros_duration_safe(speed_duration):
            raise ValueError(
                f"{http_name} trajectory duration does not fit in ROS Duration"
            )
        required_duration = max(required_duration, speed_duration)
    if not is_ros_duration_safe(required_duration):
        raise ValueError(
            f"{http_name} trajectory duration does not fit in ROS Duration"
        )
    return max(required_duration, _MIN_TRAJECTORY_DURATION_SEC)


def _position_error_rad(
    http_name: str, target_position_rad: float, current_position_rad: float
) -> float:
    if http_name == "swing":
        return abs(_shortest_angular_delta(target_position_rad, current_position_rad))
    return abs(target_position_rad - current_position_rad)


def _segment_velocity(start: tuple[float, ...], target: TrajectoryTargetSnapshot) -> tuple[float, ...]:
    return tuple((end - begin) / target.duration_sec for begin, end in zip(start, target.positions_rad))


def _boundary_velocity(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
    # 反転・保持では停止し、同方向は両区間の小さい速度に制限して3次補間の行き過ぎを防ぐ。
    return tuple(math.copysign(min(abs(a), abs(b)), a) if a * b > 0.0 else 0.0
                 for a, b in zip(left, right))

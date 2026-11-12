"""PID関節のタイムラインを管理し、goal置換と安全停止を同期する。"""
from __future__ import annotations

import math
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass, replace
from decimal import Decimal

from .command_state import (JOINT_LIMITS_DEG, ABSOLUTE_JOINT_COMMANDS,
    RELATIVE_JOINT_COMMANDS, is_safe_relative_swing_command, is_ros_duration_safe,
    is_safe_pid_speed_limit)
from .track_timeline import TrackTimeline, TrackPoint, TrackSegment
from .trajectory_targets import PID_JOINTS, PID_HTTP_TO_ROS, PID_ROS_TO_HTTP, TrajectoryTargetSnapshot

_MIN_TRAJECTORY_DURATION_SEC = 0.001


@dataclass(frozen=True)
class QueuedMotionCommand:
    http_name: str
    joint_name: str
    degrees: float
    revision: int
    relative: bool = False
    speed_limit_deg_s: float | None = None


@dataclass(frozen=True)
class ScheduledTrajectory:
    dispatch_id: int
    generation: int
    commands: tuple[QueuedMotionCommand, ...]
    target: TrajectoryTargetSnapshot
    track_points: tuple[TrackPoint, ...]
    track_key: tuple


class TrajectoryScheduler:
    """受付時に時刻を固定し、Action結果の遅れを後続区間へ積み重ねない。"""

    def __init__(self, feedback_timeout_sec=0.5, max_dispatch_delay_sec=1.0, ground_timeout_sec=0.5):
        if not math.isfinite(feedback_timeout_sec) or feedback_timeout_sec < 0:
            raise ValueError('feedback_timeout_sec must be finite and non-negative')
        if not math.isfinite(max_dispatch_delay_sec) or max_dispatch_delay_sec <= 0:
            raise ValueError('max_dispatch_delay_sec must be finite and positive')
        if not math.isfinite(ground_timeout_sec) or ground_timeout_sec <= 0:
            raise ValueError('ground_timeout_sec must be positive and finite')
        self._ground_timeout_sec = ground_timeout_sec
        self._ground_value, self._ground_at = -1, float('-inf')
        self._guard_status = 'reached'
        self._guard_segment = None
        self._guard_hold_revision = None
        self._lock = threading.RLock()
        self._feedback_timeout_sec = feedback_timeout_sec
        self._max_dispatch_delay_sec = max_dispatch_delay_sec
        self._feedback_positions_rad = {}
        self._feedback_updated_at = {}
        self._commanded_targets_rad = {}
        self._timeline = None
        # feedback待ちの間もbarrierと指令の受付順序・速度を保持する。
        self._pending = []
        self._commands = []
        self._ready_at = None
        self._generation = 0
        self._revision = 0
        self._next_dispatch_id = 0
        self._in_flight = None
        self._interrupting = False
        self._stopped = False
        self._motion_failed = False
        self._last_error = ''
        self._failure_details = ''

    def _ensure_timeline(self, now):
        """feedback待ちの計画も一括で確定し、不正なDurationを部分適用しない。"""
        if self._interrupting or self._timeline is not None or not all(self._feedback_is_fresh(a, now) for a, _ in PID_JOINTS):
            return True
        timeline = TrackTimeline(tuple(self._relative_basis(a, self._feedback_positions_rad[a]) for a, _ in PID_JOINTS))
        try:
            for entry in self._pending:
                if entry is None:
                    timeline.barrier()
                else:
                    command, duration = entry
                    self._append(timeline, command, duration, now)
        except ValueError:
            self.stop('invalid_trajectory')
            return False
        self._timeline = timeline
        self._pending.clear()
        return True

    def update_http_target(self, command, degrees, speed_limit_deg_s=None, base_duration_sec=3.0, now=None):
        """絶対・相対指令を同じFIFOへ追加し、速度を受付時に固定する。"""
        relative = RELATIVE_JOINT_COMMANDS.get(command)
        axis = relative.joint if relative else ABSOLUTE_JOINT_COMMANDS.get(command)
        if (axis not in PID_HTTP_TO_ROS or not math.isfinite(degrees)
                or not is_safe_relative_swing_command(command, degrees)
                or not is_ros_duration_safe(base_duration_sec) or base_duration_sec <= 0
                or (speed_limit_deg_s is not None and not is_safe_pid_speed_limit(speed_limit_deg_s))):
            return False
        current = time.monotonic() if now is None else now
        with self._lock:
            # ガードの後にboom区間を予約すると停止後に再下降するため受付を制限する。
            if axis == 'boom' and self._guard_status == 'running':
                return False
            index = [a for a, _ in PID_JOINTS].index(axis)
            if self.joint_queue(index, current):
                return False
            cmd = QueuedMotionCommand(command, axis, relative.direction * abs(degrees) if relative else degrees,
                                      self._revision + 1, relative is not None, speed_limit_deg_s)
            if not self._ensure_timeline(current):
                return False
            if self._timeline is None:
                self._pending.append((cmd, base_duration_sec))
            else:
                try:
                    self._append(self._timeline, cmd, base_duration_sec, current)
                except ValueError:
                    return False
            self._revision += 1
            self._commands.append(cmd)
            if self._ready_at is None and self._in_flight is None:
                self._ready_at = current
            self._stopped = False
            return True

    def update_ground_contact(self, value, now=None):
        """受信時刻も保持し、推定ノードの停止で古い非接地を使い続けない。"""
        with self._lock:
            self._ground_value = value if value in (-1, 0, 1) else -1
            self._ground_at = time.monotonic() if now is None else now

    def ground_contact(self, now=None):
        current = time.monotonic() if now is None else now
        with self._lock:
            return self._ground_value if 0 <= current-self._ground_at <= self._ground_timeout_sec else -1

    def guard_status(self):
        with self._lock:
            return self._guard_status

    def start_ground_guard(self, degrees, speed_limit_deg_s=None, base_duration_sec=3., now=None):
        """unknown開始は拒否する。未確定の実測やcancel待ちからの下降も許可しない。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._guard_status == 'running':
                return False
            if (self.ground_contact(current) == -1 or self.stale_feedback(current)
                    or self._interrupting or self._motion_failed):
                self._guard_status = 'failed'
                return False
            if not self.update_http_target('boom_down', degrees, speed_limit_deg_s, base_duration_sec, current):
                self._guard_status = 'failed'
                return False
            self._guard_segment = self._timeline.axes['boom'][-1]
            self._guard_hold_revision = None
            self._guard_status = 'running'
            return True

    def check_ground_guard(self, now=None):
        """接地時はboomだけを実測位置へ置換する。異常時は呼出し側の既存安全停止を使う。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._guard_status != 'running':
                return None
            contact = self.ground_contact(current)
            if contact == -1 or self._motion_failed or self.stale_feedback(current):
                self._guard_status = 'failed'
                return 'ground_unknown'
            # barrier待ちの他軸を巻き込まない。既に接地なら区間開始時に保持する。
            if current < self._guard_segment.start or contact == 0:
                return None
            measured = self._feedback_positions_rad['boom']
            self._timeline.axes['boom'] = [TrackSegment(current, current+.001, measured, measured)]
            self._timeline.revision += 1
            self._timeline.barrier_time = max([current] + [s.end for ss in self._timeline.axes.values() for s in ss])
            self._revision += 1
            self._commands = [c for c in self._commands if c.joint_name != 'boom']
            self._commands.append(QueuedMotionCommand('boom_angle', 'boom', math.degrees(measured), self._revision))
            self._commanded_targets_rad['boom'] = measured
            self._guard_hold_revision = self._revision
            self._guard_status = 'contact'
            self._guard_segment = None
            return None

    def is_guard_hold(self, scheduled):
        with self._lock:
            return self._guard_hold_revision is not None and scheduled.target.revision == self._guard_hold_revision

    def guard_blocks_old_boom(self, scheduled):
        """保持goalの受理待ち中、旧goalへの追従許可で再下降しないようにする。"""
        with self._lock:
            return (self._guard_hold_revision is not None and
                    (scheduled is None or scheduled.target.revision < self._guard_hold_revision))

    def _append(self, timeline, command, duration, now):
        axis = command.joint_name
        segments = timeline.axes[axis]
        basis = segments[-1].target if segments else timeline.basis[[a for a, _ in PID_JOINTS].index(axis)]
        target = _resolve_target_position_rad(axis, basis, command)
        duration = _required_duration_sec(axis, command, target, basis, duration)
        timeline.add(axis, target, duration, now)

    def barrier(self, now=None):
        """全関節の計画終点で区切る。実測の停止やreporter周期を待たない。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if not self._commands or not self._ensure_timeline(current):
                return None
            if self._timeline is not None:
                return self._timeline.barrier()
            if self._pending:
                self._pending.append(None)
            return None

    def joint_queue(self, index, now=None):
        if index not in range(4):
            raise ValueError('joint_queue index must be 0..3')
        current = time.monotonic() if now is None else now
        axis = PID_JOINTS[index][0]
        with self._lock:
            return (self._timeline.queue(axis, current) if self._timeline else
                    sum(e is not None and e[0].joint_name == axis for e in self._pending))

    def pending_snapshot(self, now=None, replacing=None):
        """計画の追加・区間開始時だけ置換し、既存区間の時計は動かさない。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._interrupting or not self._commands:
                return None
            if self._in_flight is not None and self._in_flight != replacing:
                return None
            if any(not self._feedback_is_fresh(a, current) for a, _ in PID_JOINTS):
                return None
            if not self._ensure_timeline(current):
                return None
            timeline = self._timeline
            key = timeline.key(current)
            if replacing is not None and replacing.track_key == key:
                return None
            points = timeline.points(current)
            if len(points) == 1:
                points += (replace(points[0], time=_MIN_TRAJECTORY_DURATION_SEC),)
            self._next_dispatch_id += 1
            target = TrajectoryTargetSnapshot(self._revision, tuple(n for _, n in PID_JOINTS),
                                              points[-1].positions, points[-1].time)
            return ScheduledTrajectory(self._next_dispatch_id, self._generation, tuple(self._commands), target,
                points, key)

    def mark_submitted(self, scheduled, replacing=None):
        """置換の受理前は旧goalを保持し、停止後に遅着した受理もcancel対象にする。"""
        with self._lock:
            if replacing is not None and self._in_flight != replacing:
                return False
            if self._interrupting and replacing is not None:
                self._in_flight = scheduled
                return True
            if self._interrupting or scheduled.generation != self._generation or not self._commands:
                return False
            if replacing is None and self._in_flight is not None:
                return False
            self._in_flight = scheduled
            self._ready_at = None
            self._commanded_targets_rad = dict(zip((a for a, _ in PID_JOINTS), scheduled.target.positions_rad))
            return True

    def mark_send_failed(self, scheduled):
        """解決済みの区間を保ち、相対量を二重適用せず送信をやり直す。"""
        with self._lock:
            if self._in_flight != scheduled:
                return False
            self._in_flight = None
            self._interrupting = False
            self._ready_at = time.monotonic() if self._commands else None
            return True

    def mark_completed(self, scheduled, *, successful, now=None, reason='goal_aborted'):
        """最新revisionの成功だけで空にし、旧goalの結果で後続を捨てない。"""
        with self._lock:
            if self._in_flight != scheduled:
                return False
            self._in_flight = None
            if self._interrupting:
                self._interrupting = False
                self._ready_at = (time.monotonic() if now is None else now) if self._commands else None
                return True
            if not successful:
                self.stop(reason)
            elif scheduled.target.revision == self._revision:
                if self._guard_status == 'running':
                    self._guard_status = 'reached'
                    self._guard_segment = None
                self._timeline = None
                self._commands.clear()
                self._ready_at = None
                self._generation += 1
            elif self._commands:
                self._ready_at = time.monotonic() if now is None else now
            # 結果待ちの間に追加された区間は、旧goalの成功で消さない。
            return True

    def track_active_axes(self, now=None):
        with self._lock:
            return (self._timeline.active_axes(time.monotonic() if now is None else now)
                    if self._timeline is not None and not self._interrupting else set())

    def tracking_targets(self, now=None):
        """未来の目標で待機軸を駆動しないよう、開始済み区間の終点だけを許可する。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._timeline is None:
                return {}
            return {axis: started[-1].target for axis, ss in self._timeline.axes.items()
                    if (started := [seg for seg in ss if seg.start <= current])}

    def missing_feedback_joint_names(self, now=None):
        current = time.monotonic() if now is None else now
        with self._lock:
            return tuple(a for a, _ in PID_JOINTS if not self._feedback_is_fresh(a, current)) if self._commands else ()

    def motion_status(self):
        with self._lock:
            if self._motion_failed:
                return -1
            if self._interrupting and self._in_flight is not None:
                return 1
            if self._stopped:
                return 2
            return int(bool(self._commands or self._in_flight))

    def has_work(self):
        with self._lock:
            return bool(self._commands or self._in_flight)

    def needs_cancel(self):
        with self._lock:
            return self._interrupting

    def accept_motion(self):
        with self._lock:
            self._stopped = False

    def motion_error(self):
        with self._lock:
            return self._last_error

    def _relative_basis(self, joint, measured):
        return self._commanded_targets_rad.get(joint, measured)

    def stop(self, reason='stopped', *, reset=False):
        """計画を捨てて実測基準へ戻し、送信済みgoalの終端まではゲートを保持する。"""
        with self._lock:
            if not reset and reason != 'stopped' and self._last_error in ('', 'stopped'):
                self._failure_details = self.goal_diagnostics()
                self._last_error = reason
            if self._guard_status == 'running' or (reason != 'stopped' and not reset):
                self._guard_status = 'failed'
            if reset:
                self._guard_status = 'reached'
            self._guard_segment = None
            self._guard_hold_revision = None
            self._timeline = None
            self._pending.clear()
            self._commands.clear()
            self._commanded_targets_rad.clear()
            self._ready_at = None
            self._generation += 1
            self._interrupting = self._in_flight is not None
            if reset:
                self._last_error = self._failure_details = ''
            elif reason == 'stopped' and not self._last_error:
                self._last_error = reason
            self._motion_failed = self._last_error not in ('', 'stopped')
            self._stopped = not reset and reason == 'stopped'

    def stale_feedback(self, now=None):
        current = time.monotonic() if now is None else now
        with self._lock:
            return any(not self._feedback_is_fresh(a, current) for a, _ in PID_JOINTS)

    def check_deadlines(self, now=None):
        current = time.monotonic() if now is None else now
        with self._lock:
            if ((self._in_flight is not None and not self._interrupting and self.stale_feedback(current))
                    or (self._in_flight is None and self._ready_at is not None and current - self._ready_at > self._max_dispatch_delay_sec)):
                self.stop('stale')

    def motion_failure_details(self):
        with self._lock:
            return self._failure_details

    def goal_diagnostics(self, scheduled=None):
        """実測を目標で上書きせず、失敗時の指令軸との差を残す。"""
        with self._lock:
            scheduled = scheduled or self._in_flight
            details = []
            for i, (axis, _) in enumerate(PID_JOINTS):
                if not any(c.joint_name == axis for c in (scheduled.commands if scheduled else self._commands)):
                    continue
                measured = self._feedback_positions_rad.get(axis)
                target = scheduled.target.positions_rad[i] if scheduled else (self._timeline.axes[axis][-1].target if self._timeline and self._timeline.axes[axis] else None)
                def degrees(v):
                    return 'unavailable' if v is None else f'{math.degrees(v):.3f}'
                delta = None if measured is None or target is None else (_shortest_angular_delta(target, measured) if axis == 'swing' else target-measured)
                details.append(f'axis={axis} target_deg={degrees(target)} measured_deg={degrees(measured)} delta_deg={degrees(delta)}')
            return '; '.join(details) or 'axes=none'

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
            for http_name, position_rad in updates:
                self._feedback_positions_rad[http_name] = position_rad
                self._feedback_updated_at[http_name] = current_time

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

    def _feedback_is_fresh(self, http_name: str, now: float) -> bool:
        if http_name not in self._feedback_positions_rad:
            return False
        if self._feedback_timeout_sec == 0.0:
            return True
        updated_at = self._feedback_updated_at[http_name]
        # 経過時間が負になるのは、擬似時計を使うテストの場合だけである。
        return now <= updated_at or now - updated_at <= self._feedback_timeout_sec


def _shortest_angular_delta(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def _resolve_target_position_rad(axis, basis, command):
    """相対量は直前目標へ加え、絶対旋回だけ最短方向へ変換する。"""
    if command.relative and axis == 'swing':
        return basis + math.radians(command.degrees)
    degrees = (Decimal.from_float(math.degrees(basis)) + Decimal.from_float(command.degrees)
               if command.relative else Decimal.from_float(command.degrees))
    lo, hi = JOINT_LIMITS_DEG[axis]
    target = math.radians(float(max(Decimal(lo), min(Decimal(hi), degrees))))
    return basis + _shortest_angular_delta(target, basis) if axis == 'swing' else target


def _required_duration_sec(axis, command, target, basis, base_duration_sec):
    """クランプ済みの移動量と受付時の速度から一度だけ時間を決める。"""
    speed = command.speed_limit_deg_s
    distance = abs(_shortest_angular_delta(target, basis)) if axis == 'swing' else abs(target-basis)
    duration = base_duration_sec if speed is None else math.degrees(distance)/speed
    if not is_ros_duration_safe(duration):
        raise ValueError(f'{axis} trajectory duration does not fit in ROS Duration')
    return max(duration, _MIN_TRAJECTORY_DURATION_SEC)


def _boundary_velocity(left: tuple[float, ...], right: tuple[float, ...]) -> tuple[float, ...]:
    # 反転・保持では停止し、同方向は両区間の小さい速度に制限して3次補間の行き過ぎを防ぐ。
    return tuple(math.copysign(min(abs(a), abs(b)), a) if a * b > 0.0 else 0.0
                 for a, b in zip(left, right))

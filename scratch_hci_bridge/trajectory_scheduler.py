"""PID関節のタイムラインを管理し、goal置換と安全停止を同期する。"""
from __future__ import annotations

import math
from collections import deque
from statistics import median
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass, replace
from decimal import Decimal

from .command_state import (JOINT_LIMITS_DEG, ABSOLUTE_JOINT_COMMANDS,
    RELATIVE_JOINT_COMMANDS, GUARD_MOTIONS, is_safe_relative_swing_command, is_ros_duration_safe,
    is_safe_pid_speed_limit)
from .track_timeline import TrackTimeline, TrackPoint, TrackSegment
from .guard_progress import ProgressWindow
from .trajectory_targets import PID_JOINTS, PID_HTTP_TO_ROS, PID_ROS_TO_HTTP, TrajectoryTargetSnapshot

_MIN_TRAJECTORY_DURATION_SEC = 0.001
# 到着ゲートで保留した接地動作を、通常の角度指令と区別する印。
_GUARD_HTTP_NAME = 'ground_guard'


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
    planned_at: float = 0.0


class TrajectoryScheduler:
    """関節別計画と実測到着ゲートを管理し、後続の先行動作を防ぐ。"""

    def __init__(self, feedback_timeout_sec=0.5, max_dispatch_delay_sec=1.0, ground_timeout_sec=0.5,
                 ground_slow_distance_m=0.2, ground_slow_speed_deg_s=1.0,
                 arm_ground_slow_speed_deg_s=2.0, bucket_ground_slow_speed_deg_s=4.0,
                 ground_stall_sec=0.5, ground_stall_epsilon_deg=0.1,
                 swing_serial_arrival_tolerance_deg=1.5, boom_serial_arrival_tolerance_deg=2.5,
                 arm_serial_arrival_tolerance_deg=5.5, bucket_serial_arrival_tolerance_deg=5.5,
                 serial_arrival_speed_deg_s=2.0, serial_arrival_timeout_sec=10.0,
                 swing_load_stall_lead_deg=2.5, boom_load_stall_lead_deg=2.5,
                 arm_load_stall_lead_deg=8.5, bucket_load_stall_lead_deg=8.5,
                 swing_load_stall_window_sec=1.5, boom_load_stall_window_sec=1.5,
                 arm_load_stall_window_sec=1.5, bucket_load_stall_window_sec=1.5,
                 stall_motion_deg=0.5, motion_timeout_sec=240.0):
        if not math.isfinite(feedback_timeout_sec) or feedback_timeout_sec < 0:
            raise ValueError('feedback_timeout_sec must be finite and non-negative')
        if not math.isfinite(max_dispatch_delay_sec) or max_dispatch_delay_sec <= 0:
            raise ValueError('max_dispatch_delay_sec must be finite and positive')
        if not math.isfinite(ground_timeout_sec) or ground_timeout_sec <= 0:
            raise ValueError('ground_timeout_sec must be positive and finite')
        if not all(math.isfinite(v) and v > 0 for v in (ground_slow_distance_m, ground_slow_speed_deg_s,
                arm_ground_slow_speed_deg_s, bucket_ground_slow_speed_deg_s,
                ground_stall_sec, ground_stall_epsilon_deg)):
            raise ValueError('ground slow/stall parameters must be positive and finite')
        tolerances = dict(zip((a for a, _ in PID_JOINTS), (
            swing_serial_arrival_tolerance_deg, boom_serial_arrival_tolerance_deg,
            arm_serial_arrival_tolerance_deg, bucket_serial_arrival_tolerance_deg)))
        if not all(math.isfinite(v) and v > 0 for v in
                   (*tolerances.values(), serial_arrival_speed_deg_s, serial_arrival_timeout_sec)):
            raise ValueError('serial arrival parameters must be positive and finite')
        leads = dict(zip((a for a, _ in PID_JOINTS), (
            swing_load_stall_lead_deg, boom_load_stall_lead_deg,
            arm_load_stall_lead_deg, bucket_load_stall_lead_deg)))
        windows = dict(zip((a for a, _ in PID_JOINTS), (
            swing_load_stall_window_sec, boom_load_stall_window_sec,
            arm_load_stall_window_sec, bucket_load_stall_window_sec)))
        if not all(math.isfinite(v) and v > 0 for v in
                   (*leads.values(), *windows.values(), stall_motion_deg, motion_timeout_sec)):
            raise ValueError('load stall / motion timeout parameters must be positive and finite')
        self._load_stall_leads = {a: math.radians(v) for a, v in leads.items()}
        self._load_stall_windows = windows
        self._stall_motion = math.radians(stall_motion_deg)
        self._motion_timeout = motion_timeout_sec
        self._motion_started_at = None
        self._stall_samples = {a: deque() for a, _ in PID_JOINTS}
        self._resistance_holds = {}
        self._stall_positions = {}
        self._stall_basis_axes = set()
        self._tracking_trajectory = None
        self._motion_outcome = 'arrived'
        self._serial_arrival_tolerances = {a: math.radians(v) for a, v in tolerances.items()}
        self._serial_arrival_speed = math.radians(serial_arrival_speed_deg_s)
        self._feedback_travel = {a: deque(maxlen=128) for a, _ in PID_JOINTS}
        # 通常到着は理想基準を残し、抵抗停止だけ未達基準も捨てる。保持出力は実測へ合わせる。
        self._serial_holds = {}
        self._serial_arrival_timeout = serial_arrival_timeout_sec
        self._arrival_gate = False
        self._arrival_received_at = float('-inf')
        self._arrival_pending = []
        self._completed_track_key = None
        self._ground_stall_sec = ground_stall_sec
        self._ground_stall_epsilon = math.radians(ground_stall_epsilon_deg)
        self._guard_progress = ProgressWindow(ground_stall_sec, self._ground_stall_epsilon)
        self._guard_hold_position = None
        self._guard_following = False
        self._guard_finishing = False
        self._ground_slow_distance_m = ground_slow_distance_m
        # 接地前の減速速度。ground_slow_speed_deg_s はブーム用。回転半径が小さいアーム・バケットは、
        # バケットが地面へ近づく速さがブームと同程度（約5cm/s）になるよう角速度を大きくする。
        # バケットを1°/sにすると0.2mを下りるのに約20秒かかり、バケットの陰の地面が遮蔽保持の10秒を
        # 過ぎて不明になり停止した（2026-11-26 Unity、bucket_dump）。
        self._ground_slow_speeds_rad_s = {
            'boom': math.radians(ground_slow_speed_deg_s), 'arm': math.radians(arm_ground_slow_speed_deg_s),
            'bucket': math.radians(bucket_ground_slow_speed_deg_s)}
        self._ground_clearance, self._ground_clearance_at = float('nan'), float('-inf')
        self._ground_distances = deque(maxlen=5)
        self._guard_slowed = False
        self._ground_timeout_sec = ground_timeout_sec
        self._ground_value, self._ground_at = -1, float('-inf')
        self._guard_status = 'reached'
        # 接地で止める関節と、地面へ近づく向き（関節角が増える向きを+1）。
        self._guard_axis, self._guard_direction = 'boom', 1.0
        # 距離指定の接地動作の目標距離[m]。接地で止める動作ではNone。
        self._guard_clearance_target = None
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
                self._apply_track_entry(timeline, entry, now)
        except ValueError:
            self.stop('invalid_trajectory')
            return False
        self._timeline = timeline
        self._completed_track_key = None
        self._pending.clear()
        return True

    def _deeper(self, a, b):
        """接地動作の向きに進んだ方の角度を返す（ブーム下げならmax）。"""
        return a if self._guard_direction*(a-b) >= 0 else b

    def guard_axis(self):
        with self._lock:
            return self._guard_axis

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
            if self._motion_failed:
                return False
            # ガード中に同じ関節の区間を予約すると、停止後に再び動くため受付を制限する。
            if axis == self._guard_axis and (self._guard_status == 'running' or self._guard_settling(current)):
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
                    self._apply_track_entry(self._timeline, (cmd, base_duration_sec), current)
                except ValueError:
                    return False
            if axis == self._guard_axis:
                self._guard_hold_position = self._guard_hold_revision = None
                self._guard_finishing = False
            if self._motion_started_at is None:
                self._motion_started_at = current
                self._motion_outcome = 'running'
            self._revision += 1
            self._commands.append(cmd)
            if self._ready_at is None and self._in_flight is None:
                self._ready_at = current
            self.accept_motion()
            return True

    def update_ground_contact(self, value, now=None):
        """受信時刻も保持し、推定ノードの停止で古い非接地を使い続けない。"""
        with self._lock:
            self._ground_value = value if value in (-1, 0, 1) else -1
            self._ground_at = time.monotonic() if now is None else now

    def update_ground_clearance(self, value, now=None):
        """減速の距離は接地と独立に鮮度を確認し、一度減速した後は再加速しない。"""
        with self._lock:
            self._ground_clearance = value
            self._ground_clearance_at = time.monotonic() if now is None else now
            if math.isfinite(value):
                self._ground_distances.append((self._ground_clearance_at, value))

    def ground_contact(self, now=None):
        current = time.monotonic() if now is None else now
        with self._lock:
            return self._ground_value if 0 <= current-self._ground_at <= self._ground_timeout_sec else -1

    def guard_status(self):
        with self._lock:
            return self._guard_status

    def start_ground_guard(self, motion='boom_down', speed_limit_deg_s=None, base_duration_sec=3., now=None,
                           clearance_m=None, travel_deg=None):
        """motionの向きへ可動域の端まで動かし、接地（距離指定なら目標距離以下）で止める。

        HTTP指令は上限角を取らない（travel_deg=None）。地面が見つからなければ可動域の端に着いて
        reached になる。travel_deg は区間の終点を近くに置いて終端処理を短時間で試すための引数で、
        単体テストだけが使う。
        unknown開始は拒否する。未確定の実測やcancel待ちからの開始も許可しない。
        """
        current = time.monotonic() if now is None else now
        if motion not in GUARD_MOTIONS:
            return False
        relative = RELATIVE_JOINT_COMMANDS[motion]
        if clearance_m is not None and not (math.isfinite(clearance_m) and clearance_m >= 0):
            return False
        with self._lock:
            if self._guard_status == 'running' or self._guard_settling(current):
                return False
            if (self.ground_contact(current) == -1 or self.stale_feedback(current)
                    or self._interrupting or self._motion_failed):
                self._guard_status = 'failed'
                return False
            axis = relative.joint
            previous = (self._guard_axis, self._guard_direction, self._guard_hold_position,
                        self._guard_hold_revision, self._guard_finishing, self._guard_following)
            if axis != self._guard_axis:
                # 前の関節の保持は通常の保持目標として残し、行き過ぎ追従だけを新しい関節へ移す。
                self._guard_hold_position = self._guard_hold_revision = None
                self._guard_finishing = self._guard_following = False
            self._guard_axis, self._guard_direction = axis, relative.direction
            lo, hi = JOINT_LIMITS_DEG[axis]
            command, value = ((motion, travel_deg) if travel_deg is not None
                              else (f'{axis}_angle', hi if relative.direction > 0 else lo))
            if not self.update_http_target(command, value, speed_limit_deg_s, base_duration_sec, current):
                # 受付できなかった時は、前の関節の保持状態を失わない。
                (self._guard_axis, self._guard_direction, self._guard_hold_position,
                 self._guard_hold_revision, self._guard_finishing, self._guard_following) = previous
                self._guard_status = 'failed'
                return False
            if self._arrival_gate:
                # 保留中の動作を既存区間と取り違えて接地停止させない。
                cmd, duration = self._arrival_pending[-1]
                self._arrival_pending[-1] = (replace(cmd, http_name=_GUARD_HTTP_NAME), duration)
                self._guard_segment = None
            else:
                self._guard_segment = self._timeline.axes[axis][-1]
            self._guard_clearance_target = clearance_m
            self._guard_hold_revision = None
            self._guard_status = 'running'
            self._guard_slowed = False
            self._guard_progress.samples.clear()
            return True

    def check_ground_guard(self, now=None):
        """接地時は接地動作の関節だけを実測位置へ置換する。異常時は呼出し側の既存安全停止を使う。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._guard_status != 'running':
                if self._guard_finishing and (
                        self.ground_contact(current) == -1 or self.stale_feedback(current)):
                    self._guard_status = 'failed'
                    return 'ground_unknown'
                return None
            contact = self.ground_contact(current)
            if contact == -1 or self._motion_failed or self.stale_feedback(current):
                self._guard_status = 'failed'
                return 'ground_unknown'
            # barrier待ちの他軸を巻き込まない。既に接地なら区間開始時に保持する。
            if self._guard_segment is None or current < self._guard_segment.start:
                return None
            if contact == 0:
                axis = self._guard_axis
                measured = self._feedback_positions_rad[axis]
                clearance_fresh = (math.isfinite(self._ground_clearance)
                                   and 0 <= current-self._ground_clearance_at <= self._ground_timeout_sec)
                target = self._guard_clearance_target
                if target is not None and clearance_fresh and self._ground_clearance <= target:
                    # 距離指定は単発値で止める。誤って早く止まる側に倒れるだけで、押し込みは起きない。
                    self._finish_guard('distance', current)
                    return None
                if (current >= self._guard_segment.end and
                        abs(measured-self._guard_segment.target) <= self._ground_stall_epsilon):
                    self._finish_guard('reached', current)
                    return None
                # 始動時の補間速度ゼロを障害物と誤認しないよう、計画にも十分な進みを要求する。
                planned_progress = self._guard_planned_progress(current)
                if self._guard_progress.stalled() and planned_progress > 2*self._ground_stall_epsilon:
                    if not (math.isfinite(self._ground_clearance) and
                            0 <= current-self._ground_clearance_at <= self._ground_timeout_sec):
                        self._guard_status = 'failed'
                        return 'ground_unknown'
                    # 近い/遠いの分類も単発の距離急落に引かれないよう直近5観測の中央値を使う。
                    distances = [d for t, d in self._ground_distances if 0 <= current-t <= self._ground_timeout_sec]
                    self._finish_guard('stalled' if median(distances) < self._ground_slow_distance_m
                                       else 'blocked', current)
                    return None
                # 距離指定では目標距離の手前0.2mから減速し、目標を行き過ぎにくくする。
                slow_distance = self._ground_slow_distance_m + (target or 0.)
                if (not self._guard_slowed and clearance_fresh
                        and self._ground_clearance < slow_distance
                        and current < self._guard_segment.end):
                    try:
                        replacement = self._timeline.slow_remaining(axis, self._guard_segment,
                            current, self._ground_slow_speeds_rad_s[axis], measured=measured)
                    except ValueError:
                        self._guard_status = 'failed'
                        return 'invalid_trajectory'
                    if replacement is not self._guard_segment:
                        # 旧goalの遅着成功で減速後の計画を消さないためscheduler側も更新する。
                        self._revision += 1
                        self._guard_segment = replacement
                        # 減速前の観測を新しい補間の始動待ちへ持ち込まない。
                        self._guard_progress.samples.clear()
                    self._guard_slowed = True
                return None
            self._finish_guard('contact', current)
            return None

    def _guard_planned_progress(self, now):
        """停止端の3次補間の始動待ちを停滞と誤認しないため、窓内の予定変位を見る。"""
        segment = self._guard_segment
        if now >= segment.end:
            # 計画終了後は予定速度がゼロでも未到達の実測停滞を打ち切る必要がある。
            return abs(segment.target-self._feedback_positions_rad[self._guard_axis]) + 2*self._ground_stall_epsilon
        return abs(self._guard_command_position(now)-self._guard_command_position(now-self._ground_stall_sec))

    def _guard_command_position(self, now):
        """JTCの停止端Hermite補間で、実測に対して地面へ近づく要求があるかを評価する。"""
        segment = self._guard_segment
        duration = segment.end-segment.start
        velocity = self._timeline.state(self._guard_axis, segment.start)[1]
        u = max(0., min(1., (now-segment.start)/duration))
        return (segment.origin + (3*u*u-2*u*u*u)*(segment.target-segment.origin)
                + (u*u*u-2*u*u+u)*duration*velocity)

    def _guard_settling(self, now):
        """接地後も惰性で動いている間は、同じ関節の次の指令と完了を待たせる。"""
        return self._guard_finishing and not self._guard_progress.stalled()

    def _finish_guard(self, status, now):
        """動作の終点を捨て、検知角へ戻さず実測の停止位置で保持する。"""
        self._guard_status = status
        self._guard_segment = None
        self._guard_hold_position = self._feedback_positions_rad[self._guard_axis]
        self._guard_following = True
        self._guard_finishing = True
        self._guard_progress.samples.clear()
        self._guard_progress.update(now, self._guard_hold_position)
        self._replace_guard_hold(now)

    def _replace_guard_hold(self, now):
        """地面へ近づく向きに行き過ぎた分だけ追従し、離れる側へ目標を戻さない。"""
        axis = self._guard_axis
        measured = self._feedback_positions_rad[axis]
        self._guard_hold_position = self._deeper(self._guard_hold_position, measured)
        # 外力で戻された場合も離れる向きの目標は作らず、従来の最深位置を保つ。
        position = self._guard_hold_position
        self._timeline.axes[axis] = [TrackSegment(now, now+.001, position, position)]
        self._timeline.revision += 1
        self._timeline.barrier_time = max([now] + [s.end for ss in self._timeline.axes.values() for s in ss])
        self._revision += 1
        self._commands = [c for c in self._commands if c.joint_name != axis]
        self._commands.append(QueuedMotionCommand(axis+'_angle', axis, math.degrees(position), self._revision))
        self._commanded_targets_rad[axis] = position
        self._guard_hold_revision = self._revision

    def is_guard_hold(self, scheduled):
        with self._lock:
            return self._guard_hold_revision is not None and scheduled.target.revision == self._guard_hold_revision

    def guard_blocks_old_target(self, scheduled):
        """検知直後とドリフト中の古い保持goalには、接地動作の関節の出力を許可しない。"""
        with self._lock:
            index = [a for a, _ in PID_JOINTS].index(self._guard_axis)
            measured = self._feedback_positions_rad[self._guard_axis] if self._guard_hold_revision is not None else 0.
            return (self._guard_hold_revision is not None and
                    (scheduled is None or scheduled.target.revision < self._guard_hold_revision
                     or self._guard_direction*(scheduled.target.positions_rad[index]-measured) < 0))

    def _append(self, timeline, command, duration, now):
        axis = command.joint_name
        segments = timeline.axes[axis]
        basis = segments[-1].target if segments else timeline.basis[[a for a, _ in PID_JOINTS].index(axis)]
        if axis == self._guard_axis and self._guard_hold_position is not None:
            basis = self._deeper(basis, self._feedback_positions_rad[axis])
        if axis in self._stall_basis_axes:
            # 抵抗で未達の計画終点は捨て、次の明示指令は固定した停止位置から再開する。
            basis = self._resistance_holds[axis]
        target = _resolve_target_position_rad(axis, basis, command)
        duration = _required_duration_sec(axis, command, target, basis, duration)
        if axis == self._guard_axis and self._guard_hold_position is not None and segments:
            # 保持後の次の明示指令も、行き過ぎ前の古い角を相対基準にしない。
            segments[-1] = replace(segments[-1], origin=basis, target=basis)
        hold = self._serial_holds.get(axis)
        if hold is not None and math.isinf(hold[1]):
            duration = _required_duration_sec(axis, command, target, hold[0], duration)
        timeline.add(axis, target, duration, now)
        if axis in self._stall_basis_axes:
            # 予約が成功する前に保持を解除すると、不正な指令の拒否だけで軸が自由になる。
            self._resistance_holds.pop(axis)
            self._stall_basis_axes.discard(axis)
            self._stall_samples[axis].clear()
            self._commanded_targets_rad[axis] = basis
        if hold is not None and math.isinf(hold[1]):
            segment = timeline.axes[axis][-1]
            timeline.axes[axis][-1] = replace(segment, origin=hold[0])
            self._serial_holds[axis] = (hold[0], segment.start)

    def _apply_track_entry(self, timeline, entry, now):
        """到着待ちの後続はActionへ含めず、受付順と速度だけを保存する。"""
        if self._arrival_gate:
            self._arrival_pending.append(entry)
        elif isinstance(entry, bool):
            timeline.barrier()
            self._arrival_gate = entry
            self._arrival_received_at = now
        else:
            command, duration = entry
            self._append(timeline, command, duration, now)
            if command.http_name == _GUARD_HTTP_NAME:
                self._guard_segment = timeline.axes[command.joint_name][-1]

    def barrier(self, now=None, *, arrival=False):
        """val=1だけ実測到着も待ち、val=0は従来の計画時刻で区切る。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if not self._commands or not self._ensure_timeline(current):
                return None
            if self._timeline is not None:
                self._apply_track_entry(self._timeline, bool(arrival), current)
                return self._timeline.barrier_time
            self._pending.append(bool(arrival))
            return None

    def _check_arrival_gate(self, now):
        """計画完了後の新鮮な実測で解除し、遅れを後続の開始へ反映する。"""
        while self._arrival_gate and self._timeline is not None:
            timeline = self._timeline
            # 接地減速と正常打切りで変わった計画終点・保持目標を毎回使う。
            end = max(timeline.barrier_time, self._arrival_received_at)
            if now < end:
                return
            if now - end > self._serial_arrival_timeout:
                self.stop('serial_arrival_timeout')
                return
            targets = {axis: ss[-1].target for axis, ss in timeline.axes.items() if ss}
            if ((self._guard_status == 'running' and self._guard_segment is not None) or self.stale_feedback(now)
                    or any(self.targets_outside_tolerance({a: target}, self._serial_arrival_tolerances[a])
                           or self.feedback_speed(a) >= self._serial_arrival_speed for a, target in targets.items())):
                return
            completed_prefix = self._in_flight is None and timeline.key(now) == self._completed_track_key
            self._serial_holds.update({a: (self._completion_hold_position(a, target), float('inf'))
                                       for a, target in targets.items()})
            self._arrival_gate = False
            pending, self._arrival_pending = self._arrival_pending, []
            try:
                for entry in pending:
                    self._apply_track_entry(timeline, entry, now)
            except ValueError:
                self.stop('invalid_trajectory')
                return
            # 保留中のgoal成功後も、新区間の送信待ち期限は解除時刻から数える。
            if completed_prefix:
                self._ready_at = now

    def joint_queue(self, index, now=None):
        if index not in range(4):
            raise ValueError('joint_queue index must be 0..3')
        current = time.monotonic() if now is None else now
        axis = PID_JOINTS[index][0]
        with self._lock:
            return (self._timeline.queue(axis, current) if self._timeline else
                    sum(not isinstance(e, bool) and e[0].joint_name == axis for e in self._pending)) + sum(
                        not isinstance(e, bool) and e[0].joint_name == axis for e in self._arrival_pending)

    def pending_snapshot(self, now=None, replacing=None):
        """計画の追加・区間開始時だけ置換し、既存区間の時計は動かさない。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            self._check_bridge_completion(current)
            if self._interrupting or not self._commands:
                return None
            if self._in_flight is not None and self._in_flight != replacing:
                return None
            if any(not self._feedback_is_fresh(a, current) for a, _ in PID_JOINTS):
                return None
            if not self._ensure_timeline(current):
                return None
            self._check_arrival_gate(current)
            if self._timeline is None:
                return None
            timeline = self._timeline
            key = timeline.key(current)
            if replacing is None and key == self._completed_track_key:
                return None
            if replacing is not None and replacing.track_key == key:
                return None
            points = timeline.points(current)
            points = tuple(replace(p,
                positions=tuple(self._serial_holds[a][0] if a in self._serial_holds
                                and current+p.time < self._serial_holds[a][1] else p.positions[i]
                                for i, (a, _) in enumerate(PID_JOINTS)),
                velocities=tuple(0. if a in self._serial_holds and current+p.time < self._serial_holds[a][1]
                                 else p.velocities[i] for i, (a, _) in enumerate(PID_JOINTS))) for p in points)
            if self._guard_hold_position is not None:
                # 静止確定後の微小な行き過ぎも、他軸追加によるgoal再送で地面から離れる向きに戻さない。
                index = [a for a, _ in PID_JOINTS].index(self._guard_axis)
                position = self._deeper(self._guard_hold_position, self._feedback_positions_rad[self._guard_axis])
                points = tuple(replace(p, positions=p.positions[:index]+(position,)+p.positions[index+1:],
                                       velocities=p.velocities[:index]+(0.,)+p.velocities[index+1:]) for p in points)
            if len(points) == 1:
                points += (replace(points[0], time=_MIN_TRAJECTORY_DURATION_SEC),)
            self._next_dispatch_id += 1
            target = TrajectoryTargetSnapshot(self._revision, tuple(n for _, n in PID_JOINTS),
                                              points[-1].positions, points[-1].time)
            # idleを挟んでも保持軸をActionの対象から外さず、終点許容も有効にする。
            commands = tuple(self._commands) + tuple(
                QueuedMotionCommand(a+'_angle', a, math.degrees(pos), self._revision)
                for a, pos in self._resistance_holds.items()
                if not any(c.joint_name == a for c in self._commands))
            return ScheduledTrajectory(self._next_dispatch_id, self._generation, commands, target,
                points, key, current)

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
            self._tracking_trajectory = scheduled
            self._ready_at = None
            self._commanded_targets_rad = dict(zip((a for a, _ in PID_JOINTS), scheduled.target.positions_rad))
            # 保持出力の実測化で相対指令の誤差を累積させない。
            if self._timeline is not None:
                self._commanded_targets_rad.update({a: ss[-1].target for a, ss in self._timeline.axes.items() if ss})
            if self._guard_hold_position is not None:
                self._commanded_targets_rad[self._guard_axis] = self._deeper(
                    self._guard_hold_position, self._feedback_positions_rad[self._guard_axis])
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
            current = time.monotonic() if now is None else now
            if (successful and scheduled.target.revision == self._revision and self._guard_status == 'running'
                    and self._guard_segment is not None
                    and scheduled.track_key[0] == self._timeline.revision):
                self._finish_guard('reached', current)
                self._ready_at = current
                return True
            if not successful and reason == 'tolerance_violated' and self._guard_status == 'running':
                # JTC終端がタイマーより先でも、実測停滞と他軸の到達を確認できる場合だけ回復する。
                others = {c.joint_name: scheduled.target.positions_rad[[a for a, _ in PID_JOINTS].index(c.joint_name)]
                          for c in scheduled.commands if c.joint_name != self._guard_axis}
                if not self.stale_feedback(current) and not self.targets_outside_tolerance(others, math.radians(.75)):
                    self.check_ground_guard(current)
            others = {c.joint_name: scheduled.target.positions_rad[[a for a, _ in PID_JOINTS].index(c.joint_name)]
                      for c in scheduled.commands if c.joint_name != self._guard_axis}
            recoverable = (not successful and reason == 'tolerance_violated'
                           and self._guard_hold_revision is not None
                           and self._guard_status in ('contact', 'stalled', 'reached', 'blocked')
                           and scheduled.target.revision < self._guard_hold_revision
                           and self.ground_contact(current) != -1 and not self.stale_feedback(current)
                           and not self.targets_outside_tolerance(others, math.radians(.75)))
            if not successful and not recoverable:
                self.stop(reason)
            elif recoverable:
                self._ready_at = current
            elif self._arrival_gate:
                # 送信済みの前半だけが成功しても、未送信の後半とゲートを捨てない。
                if scheduled.track_key[0] == self._timeline.revision:
                    # 送信後に開始時刻を迎えた区間も、そのgoalの成功で完了している。
                    self._completed_track_key = self._timeline.key(current)
                    self._ready_at = None
                else:
                    self._completed_track_key = scheduled.track_key
                    self._ready_at = current
            elif self._guard_settling(current):
                # 保持goalの早い成功でも、惰性の下降が終わるまで追従計画を捨てない。
                self._ready_at = current
            elif (scheduled.target.revision == self._revision
                  and scheduled.track_key[0] == self._timeline.revision):
                # Action成功だけで完了にせず、ブリッジの軸別位置・実測速度も確認する。
                self._completed_track_key = self._timeline.key(current)
                self._ready_at = None
                self._check_bridge_completion(current)
            elif self._commands:
                self._ready_at = time.monotonic() if now is None else now
            # 結果待ちの間に追加された区間は、旧goalの成功で消さない。
            return True

    def track_active_axes(self, now=None):
        with self._lock:
            return (self._timeline.active_axes(time.monotonic() if now is None else now)
                    if self._timeline is not None and not self._interrupting else set())

    def serial_held_axes(self, now=None):
        """ゲートで受け入れた停止軸は、次の明示区間まで出力許可を再取得させない。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            return {a for a, (_, until) in self._serial_holds.items()
                    if current < until and a not in self._resistance_holds}

    def resistance_held_axes(self):
        """停止位置を能動保持する軸を返す。完了や他軸指令では解除しない。"""
        with self._lock:
            return set(self._resistance_holds)

    def resistance_blocks_old_goal(self, scheduled):
        """保持goalの受理までは、古い未達目標で再び土を押さない。"""
        with self._lock:
            if scheduled is None:
                return set(self._resistance_holds) if self.has_work() else set()
            return {a for a, pos in self._resistance_holds.items()
                    if any(abs(p.positions[[n for n, _ in PID_JOINTS].index(a)]-pos) > 1e-9
                           for p in scheduled.track_points)}

    def tracking_targets(self, now=None):
        """未来の目標で待機軸を駆動しないよう、開始済み区間の終点だけを許可する。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._timeline is None:
                return {}
            return {axis: (self._serial_holds[axis][0] if axis in self._serial_holds
                           and current < self._serial_holds[axis][1] else started[-1].target)
                    for axis, ss in self._timeline.axes.items()
                    if (started := [seg for seg in ss if seg.start <= current])}

    def missing_feedback_joint_names(self, now=None):
        current = time.monotonic() if now is None else now
        with self._lock:
            return tuple(a for a, _ in PID_JOINTS if not self._feedback_is_fresh(a, current)) if self._commands else ()

    def motion_status(self):
        with self._lock:
            if self._motion_failed:
                return -1
            if self._stopped:
                return 2
            if self._interrupting and self._in_flight is not None:
                return 1
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
            if self._last_error == 'stopped':
                self._last_error = ''

    def motion_error(self):
        with self._lock:
            return self._last_error

    def _relative_basis(self, joint, measured):
        """抵抗は固定停止角、接地は行き過ぎ後の実測、通常は理想目標を相対基準にする。"""
        if joint in self._stall_basis_axes:
            return self._resistance_holds[joint]
        if joint == self._guard_axis and self._guard_hold_position is not None:
            return self._deeper(measured, self._commanded_targets_rad.get(joint, measured))
        return self._commanded_targets_rad.get(joint, measured)

    def stop_axis(self, axis, now=None):
        """1関節だけを今の実測位置で止めて保つ。他の関節の計画はそのまま続ける。

        接地動作中の関節なら終わり方を halted にし、惰性の行き過ぎも接地時と同じく追従して保持する。
        実測が古い・動作異常中は受け付けない（止める位置が決まらないため）。
        """
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._motion_failed or not self._feedback_is_fresh(axis, current):
                return False
            def same_axis(entry):
                return not isinstance(entry, bool) and entry[0].joint_name == axis
            if axis == self._guard_axis and self._guard_status == 'running':
                if self._guard_segment is None:
                    # 到着ゲートで保留中の接地動作はまだ始まっていないので、保留から外すだけでよい。
                    self._arrival_pending = [e for e in self._arrival_pending if not same_axis(e)]
                    self._pending = [e for e in self._pending if not same_axis(e)]
                    self._commands = [c for c in self._commands if c.joint_name != axis]
                    self._guard_status = 'halted'
                    return True
                self._finish_guard('halted', current)
                return True
            self._arrival_pending = [e for e in self._arrival_pending if not same_axis(e)]
            self._pending = [e for e in self._pending if not same_axis(e)]
            if self._timeline is None or not self._timeline.axes[axis]:
                return True
            measured = self._feedback_positions_rad[axis]
            self._timeline.axes[axis] = [TrackSegment(current, current+.001, measured, measured)]
            self._timeline.revision += 1
            self._revision += 1
            self._commands = [c for c in self._commands if c.joint_name != axis]
            self._commands.append(QueuedMotionCommand(axis+'_angle', axis, math.degrees(measured), self._revision))
            self._commanded_targets_rad[axis] = measured
            self._ready_at = current if self._in_flight is None else self._ready_at
            return True

    def axis_moving(self, index, now=None):
        """関節が止まったかを返す。'1'=動いている、'0'=止まった、'unknown'=実測が古い。

        計画の残り、接地後の惰性、実測速度のどれかがあれば動いているとみなす。
        """
        current = time.monotonic() if now is None else now
        axis = PID_JOINTS[index][0]
        with self._lock:
            if not self._feedback_is_fresh(axis, current):
                return 'unknown'
            planned = self._timeline is not None and any(current < s.end for s in self._timeline.axes[axis])
            queued = any(not isinstance(e, bool) and e[0].joint_name == axis
                         for e in (*self._pending, *self._arrival_pending))
            settling = axis == self._guard_axis and (self._guard_status == 'running' or self._guard_settling(current))
            moving = planned or queued or settling or self.feedback_speed(axis) >= self._serial_arrival_speed
            return '1' if moving else '0'

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
            self._guard_hold_position = None
            self._guard_finishing = False
            self._guard_progress.samples.clear()
            self._timeline = None
            self._pending.clear()
            self._arrival_pending.clear()
            self._arrival_gate = False
            self._completed_track_key = None
            self._commands.clear()
            self._commanded_targets_rad.clear()
            self._serial_holds.clear()
            for samples in self._stall_samples.values():
                samples.clear()
            self._resistance_holds.clear()
            self._stall_basis_axes.clear()
            self._tracking_trajectory = None
            self._motion_started_at = None
            self._motion_outcome = 'arrived' if reset else ('stopped' if reason == 'stopped'
                                                            and self._last_error in ('', 'stopped') else 'abnormal')
            if reset:
                self._stall_positions.clear()
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
            if (self.has_work() and self._motion_started_at is not None
                    and current-self._motion_started_at > self._motion_timeout):
                self.stop('motion_timeout')
                return
            self._check_arrival_gate(current)
            self._check_bridge_completion(current)
            if (((self._in_flight is not None or self._arrival_gate or self._completed_track_key is not None)
                    and not self._interrupting and self.stale_feedback(current))
                    or (self._in_flight is None and self._ready_at is not None and current - self._ready_at > self._max_dispatch_delay_sec
                        and not (self._timeline and self._timeline.key(current) == self._completed_track_key))):
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
                if http_name is None:
                    continue
                if not math.isfinite(position_rad):
                    # NaNを直前の正常値に置き換えると異常を隠すため、その軸を直ちに未知へ戻す。
                    self._feedback_positions_rad.pop(http_name, None)
                    self._feedback_updated_at.pop(http_name, None)
                    self._feedback_travel[http_name].clear()
                    self._stall_samples[http_name].clear()
                    if self.has_work() or self._resistance_holds:
                        self.stop('nonfinite_feedback')
                    continue
                updates.append((http_name, position_rad))
            if not updates:
                return
            for http_name, position_rad in updates:
                history = self._feedback_travel[http_name]
                previous_time = self._feedback_updated_at.get(http_name, float('-inf'))
                if current_time <= previous_time or current_time-previous_time > .2:
                    history.clear()
                    self._stall_samples[http_name].clear()
                travel = history[-1][1] if history else 0.
                if history:
                    delta = position_rad-self._feedback_positions_rad[http_name]
                    if http_name == 'swing':
                        delta = _shortest_angular_delta(position_rad, self._feedback_positions_rad[http_name])
                    travel += abs(delta)
                history.append((current_time, travel))
                # 符号反転で速度が相殺されないよう、0.1秒窓の総移動量を平滑化に使う。
                while len(history) > 2 and history[1][0] <= current_time-.1:
                    history.popleft()
                self._feedback_positions_rad[http_name] = position_rad
                self._feedback_updated_at[http_name] = current_time
                self._observe_load_stall(http_name, current_time)
                if http_name == self._guard_axis and (self._guard_status == 'running' or self._guard_hold_position is not None):
                    if ((self._guard_segment is not None and current_time >= self._guard_segment.start)
                            or self._guard_hold_position is not None):
                        # 計画が実測に追いつく待ち時間は、障害物による停滞ではない。
                        if (self._guard_segment is not None and self._guard_direction*(
                                self._guard_command_position(current_time)-position_rad) <= self._ground_stall_epsilon):
                            self._guard_progress.samples.clear()
                        else:
                            self._guard_progress.update(current_time, position_rad)
                    if (self._guard_hold_position is not None and self._guard_following
                            and self._guard_direction*(position_rad-self._guard_hold_position) > 0):
                        if self._timeline is not None:
                            self._replace_guard_hold(current_time)
                        else:
                            # 正常終端後の微小ドリフトも次回の相対指令の古い基準にしない。
                            self._guard_hold_position = position_rad
                            self._commanded_targets_rad[http_name] = position_rad
                    if self._guard_hold_position is not None and self._guard_progress.stalled():
                        self._guard_following = False

    def arrival_tolerance(self, axis):
        """JTC終点とブリッジの完了・追従マスクに同一の軸別許容を使う。"""
        return self._serial_arrival_tolerances[axis]

    def joint_angle(self, index, now=None):
        """角度ブロックと同じROS基準の実測度数を返し、欠測はunknownにする。"""
        current = time.monotonic() if now is None else now
        with self._lock:
            axis = PID_JOINTS[index][0]
            return (f'{math.degrees(self._feedback_positions_rad[axis]):.3f}'
                    if self._feedback_is_fresh(axis, current) else 'unknown')

    def stall_joints(self):
        """resetまで関節ごとの最新抵抗停止を保存し、後続動作から参照できるようにする。"""
        with self._lock:
            return ','.join(a for a, _ in PID_JOINTS if a in self._stall_positions) or 'none'

    def stall_position(self, index):
        with self._lock:
            value = self._stall_positions.get(PID_JOINTS[index][0])
            return 'unknown' if value is None else f'{math.degrees(value):.3f}'

    def motion_outcome(self):
        with self._lock:
            return self._motion_outcome

    def _planned_position(self, axis, now):
        """実際に送った軌道のHermite位置を使い、置換時の補間や境界速度も反映する。"""
        scheduled = self._tracking_trajectory
        if scheduled is None:
            return None
        index = [a for a, _ in PID_JOINTS].index(axis)
        elapsed = now-scheduled.planned_at
        points = scheduled.track_points
        if elapsed < 0 or not points:
            return None
        for left, right in zip(points, points[1:]):
            if elapsed <= right.time:
                duration = right.time-left.time
                u = max(0., (elapsed-left.time)/duration)
                return ((2*u**3-3*u*u+1)*left.positions[index]
                        +(u**3-2*u*u+u)*duration*left.velocities[index]
                        +(-2*u**3+3*u*u)*right.positions[index]
                        +(u**3-u*u)*duration*right.velocities[index])
        return points[-1].positions[index]

    def _observe_load_stall(self, axis, now):
        """先行差がある期間の角度幅で判定し、微振動で観測時間を切らない。"""
        if (self._timeline is None or self._interrupting or self._motion_failed
                or axis in self.serial_held_axes(now)
                or (axis == self._guard_axis and (self._guard_status == 'running' or self._guard_finishing))):
            self._stall_samples[axis].clear()
            return
        segments = [s for s in self._timeline.axes[axis] if s.start <= now]
        planned = self._planned_position(axis, now)
        if not segments or planned is None:
            self._stall_samples[axis].clear()
            return
        segment = segments[-1]
        direction = (segment.target > segment.origin)-(segment.target < segment.origin)
        measured = self._feedback_positions_rad[axis]
        # swingも連続計画の方向を使う。wrapした実測だけを計画付近の同値角へ揃える。
        lead = _shortest_angular_delta(planned, measured) if axis == 'swing' else planned-measured
        if direction*lead < self._load_stall_leads[axis]:
            self._stall_samples[axis].clear()
            return
        samples = self._stall_samples[axis]
        # wrapで角度幅が360度にならないよう、直前観測から連続角へ展開する。
        value = (samples[-1][1] + _shortest_angular_delta(measured, samples[-1][1])
                 if axis == 'swing' and samples else measured)
        samples.append((now, value))
        window = self._load_stall_windows[axis]
        # 窓の左端をまたぐ1観測は残す。補間で揺れを小さく見積もらないため。
        while len(samples) > 2 and samples[1][0] <= now-window:
            samples.popleft()
        if (now-samples[0][0] < window-1e-9
                or max(v for _, v in samples)-min(v for _, v in samples) >= self._stall_motion):
            return
        self._stall_positions[axis] = measured
        if axis == 'swing':
            # 旋回の拘束は障害物衝突の可能性があるため、他軸を続行させない。
            self.stop('swing_load_stall')
            return
        self._timeline.axes[axis] = [TrackSegment(now, now+.001, measured, measured)]
        self._timeline.revision += 1
        self._revision += 1
        self._commands = [c for c in self._commands if c.joint_name != axis]
        self._commands.append(QueuedMotionCommand(axis+'_angle', axis, math.degrees(measured), self._revision))
        self._commanded_targets_rad[axis] = measured
        self._serial_holds[axis] = (measured, float('inf'))
        self._stall_basis_axes.add(axis)
        self._resistance_holds[axis] = measured
        self._stall_samples[axis].clear()
        self._motion_outcome = 'resistance'
        self._ready_at = now if self._in_flight is None else self._ready_at
        # 残りの同軸区間は捨てるが、既存barrierは早めず他軸の計画を保つ。

    def _completion_hold_position(self, axis, target):
        """抵抗保持は検出角を固定し、通常到着だけ実測へ揃える。"""
        if axis in self._resistance_holds:
            return self._resistance_holds[axis]
        measured = self._feedback_positions_rad[axis]
        return target + _shortest_angular_delta(measured, target) if axis == 'swing' else measured

    def _check_bridge_completion(self, now):
        """最終Action成功後も新鮮な到着観測を待ち、全体期限は保持し続ける。"""
        if (self._timeline is None or self._in_flight is not None or self._arrival_gate
                or self._completed_track_key != self._timeline.key(now)
                or self._guard_settling(now) or self.stale_feedback(now)):
            return
        targets = {a: ss[-1].target for a, ss in self._timeline.axes.items() if ss}
        if any(now < s.end-1e-9 for ss in self._timeline.axes.values() for s in ss):
            return
        if any(self.targets_outside_tolerance({a: t}, self.arrival_tolerance(a))
               or self.feedback_speed(a) >= self._serial_arrival_speed for a, t in targets.items()):
            return
        # 連続旋回の実測wrapをそのまま保持すると、次の相対区間が一回転余分に動く。
        self._serial_holds.update({a: (self._completion_hold_position(a, target), float('inf'))
                                   for a, target in targets.items()})
        self._guard_finishing = False
        self._timeline = None
        self._completed_track_key = None
        self._commands.clear()
        self._ready_at = None
        self._motion_started_at = None
        self._tracking_trajectory = None
        self._generation += 1
        if self._motion_outcome != 'resistance':
            self._motion_outcome = 'arrived'

    def targets_outside_tolerance(
        self, targets_rad: dict[str, float], tolerance_rad: float,
    ) -> dict[str, float]:
        """JointStateと同じロックで比較し、追従許可の解除に一貫した実測を使う。"""
        with self._lock:
            remaining = {}
            for axis, target in targets_rad.items():
                measured = self._feedback_positions_rad.get(axis)
                if measured is None:
                    remaining[axis] = target
                    continue
                delta = target - measured
                if axis == "swing":
                    # 連続関節の同値角を未到達と誤判定して出力を許可し続けない。
                    delta = (delta + math.pi) % (2 * math.pi) - math.pi
                if abs(delta) > (tolerance_rad[axis] if isinstance(tolerance_rad, dict) else tolerance_rad):
                    remaining[axis] = target
            return remaining

    def feedback_speed(self, axis):
        """連続した0.1秒以上の角度差分が揃うまでは、静止とみなさない。"""
        with self._lock:
            history = self._feedback_travel[axis]
            if len(history) < 2 or history[-1][0]-history[0][0] < .1-1e-9:
                return float('inf')
            return (history[-1][1]-history[0][1])/(history[-1][0]-history[0][0])

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

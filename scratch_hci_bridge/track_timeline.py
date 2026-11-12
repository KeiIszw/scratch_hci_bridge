"""関節別FIFOを共通時計上へ配置し、ROSに依存せず置換軌道を計画する。"""
from dataclasses import dataclass

from .trajectory_targets import PID_JOINTS
from .command_state import is_ros_duration_safe


@dataclass(frozen=True)
class TrackSegment:
    start: float
    end: float
    origin: float
    target: float

    @property
    def slope(self):
        return (self.target - self.origin) / (self.end - self.start)


@dataclass(frozen=True)
class TrackPoint:
    time: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


class TrackTimeline:
    """実測を再参照せず、他軸の追加で既存の位置・終了時刻を変えない計画。"""
    def __init__(self, basis):
        self.basis = tuple(basis)
        self.axes = {axis: [] for axis, _ in PID_JOINTS}
        self.revision = 0
        self.barrier_time = float("-inf")

    def barrier(self):
        """受付済み終点の最大値を固定し、後続追加で過去の区切りを動かさない。"""
        self.barrier_time = max([self.barrier_time] + [s.end for ss in self.axes.values() for s in ss])
        return self.barrier_time

    def queue(self, axis, now):
        return sum(s.start > now for s in self.axes[axis])

    def add(self, axis, target, duration, now):
        """始まっていない区間は一件に制限し、同軸は直前の終点へ接続する。"""
        if self.queue(axis, now):
            raise ValueError('joint queue is full')
        segments = self.axes[axis]
        origin = segments[-1].target if segments else self.basis[[a for a, _ in PID_JOINTS].index(axis)]
        start = max(now, self.barrier_time, segments[-1].end if segments else now)
        # 同軸の二区間を一本のgoalへ含めるため、合計のDurationも表現可能にする。
        if duration <= 0 or not is_ros_duration_safe(start + duration - now):
            raise ValueError('track trajectory duration does not fit in ROS Duration')
        segments.append(TrackSegment(start, start + duration, origin, target))
        self.revision += 1

    def active_axes(self, now):
        return {axis for axis, segments in self.axes.items()
                if any(s.start <= now < s.end for s in segments)}

    def key(self, now):
        # 追加と区間開始だけを置換契機にし、最後はActionの停止判定を待つ。
        return self.revision, tuple(sum(s.start <= now for s in ss) for ss in self.axes.values())

    def state(self, axis, now):
        """位置は区分線形、境界速度は従来の同方向最小・反転ゼロ規則を使う。"""
        from .trajectory_scheduler import _boundary_velocity
        segments = self.axes[axis]
        position = self.basis[[a for a, _ in PID_JOINTS].index(axis)]
        for i, segment in enumerate(segments):
            if now < segment.start:
                return position, 0.0
            if now < segment.end:
                velocity = segment.slope
                if now == segment.start:
                    previous = segments[i-1] if i else None
                    velocity = _boundary_velocity((previous.slope,), (velocity,))[0] if previous and previous.end == now else 0.0
                return segment.origin + segment.slope * (now-segment.start), velocity
            position = segment.target
        return position, 0.0

    def points(self, now):
        """全軸の区切りの和集合を作り、置換始点にも計画位置を使う。"""
        times = {0: now}
        for segments in self.axes.values():
            for segment in segments:
                for t in (segment.start, segment.end):
                    nanoseconds = round((t-now) * 1_000_000_000)
                    # 時刻0との重複点はJTCが拒否するため、1ns未満の境界を重ねない。
                    if nanoseconds > 0:
                        times[nanoseconds] = t
        return tuple(TrackPoint(ns / 1_000_000_000,
                               tuple(self.state(a, t)[0] for a, _ in PID_JOINTS),
                               tuple(self.state(a, t)[1] for a, _ in PID_JOINTS))
                     for ns, t in sorted(times.items()))

"""実機・DDSを使わず、計画境界での置換と指令基準の連続性を検証する。"""
import math
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory

from scratch_hci_bridge.command_state import JOINT_LIMITS_DEG
from test_goal_time_tolerance import _sending_bridge
from test_node_command_callback import _Future, _GoalHandle
from test_sync_safety import NAMES, scheduler


def queue(s, delta=10, wait=False, command='boom_down', speed=None, now=0):
    s.begin(now=now)
    assert s.update_http_target(command, delta, speed_limit_deg_s=speed, now=now)
    s.commit(wait, now=now)


class Harness:
    def __init__(self):
        self.bridge = _sending_bridge()
        self.scheduler = self.bridge._trajectory_scheduler
        self.sent = []
        self.bridge._trajectory_client.send_goal_async.side_effect = self.send

    def send(self, goal):
        response = _Future()
        handle = _GoalHandle(result_future=_Future())
        self.sent.append((goal, response, handle))
        return response

    def tick(self, now):
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=now):
            self.bridge._on_timer()

    def accept(self, index, now=0, accepted=True):
        _, response, handle = self.sent[index]
        handle.accepted = accepted
        response._result = handle
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=now):
            for callback in response.callbacks:
                callback(response)
        return handle

    def result(self, index, status=GoalStatus.STATUS_SUCCEEDED, code=0, now=1):
        future = self.sent[index][2]._result_future
        future._result = SimpleNamespace(status=status,
            result=SimpleNamespace(error_code=code, error_string='dummy'))
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=now):
            for callback in future.callbacks:
                callback(future)


@pytest.mark.parametrize('period, phase', [(0.02, 0.0), (0.02, 0.013), (0.1, 0.023)])
def test_replacement_is_sent_on_first_timer_at_or_after_planned_end(period, phase):
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.tick(phase)
    old = h.accept(0, now=phase)
    end = phase + 1
    ticks = [phase + i * period for i in range(1, math.ceil(1 / period) + 2)]
    for now in ticks:
        h.tick(now)
        if now < end:
            assert len(h.sent) == 1
        else:
            assert len(h.sent) == 2
            assert 0 <= now - end < period + 1e-12
            break
    assert old.cancel_calls == 0
    assert not old._result_future._result
    # 応答待ちの間は同じ後続を再送しない。
    h.tick(end + .2)
    assert len(h.sent) == 2


@pytest.mark.parametrize('before_accept', [False, True])
def test_preempt_canceled_invalid_goal_does_not_fail_or_drop_queue(before_accept):
    h = Harness()
    for _ in range(3):
        queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    h.tick(1)
    if before_accept:
        h.result(0, GoalStatus.STATUS_CANCELED, FollowJointTrajectory.Result.INVALID_GOAL)
    h.accept(1, now=1)
    if not before_accept:
        h.result(0, GoalStatus.STATUS_CANCELED, FollowJointTrajectory.Result.INVALID_GOAL)
    assert h.scheduler.motion_error() == ''
    assert h.scheduler.motion_status() == 1
    assert h.scheduler.motion_queue() == 1
    assert h.scheduler._in_flight == h.bridge._active_goal_scheduled
    h.tick(2)
    assert len(h.sent) == 3
    h.accept(2, now=2)
    h.result(2, now=3)
    assert h.scheduler.motion_status() == 0


def test_velocity_points_use_commanded_position_and_match_known_boundaries():
    h = Harness()
    for delta, wait in [(10, False), (5, False), (10, True)]:
        queue(h.scheduler, delta, wait)
    h.tick(0)
    first = h.sent[0][0].trajectory.points
    assert len(first) == 2
    assert list(first[0].velocities) == [0.0] * 4
    assert first[-1].velocities[1] == pytest.approx(math.radians(5))
    h.accept(0)
    # 追従しなくても目標位置・時間は目標間で決める。
    h.scheduler.update_from_joint_state(NAMES, [0.0, math.radians(-70), 2.0, 1.0], now=1)
    h.tick(1)
    second = h.sent[1][0].trajectory.points
    assert second[0].positions == first[-1].positions
    assert second[0].velocities == first[-1].velocities
    assert second[-1].velocities[1] == pytest.approx(math.radians(5))
    assert second[-1].positions[1] == pytest.approx(math.radians(-35))
    assert second[-1].time_from_start.sec == 1
    h.accept(1, now=1)
    h.tick(2)
    third = h.sent[2][0].trajectory.points
    assert third[0].positions == second[-1].positions
    assert third[0].velocities == second[-1].velocities
    assert list(third[-1].velocities) == [0.0] * 4
    h.accept(2, now=2)
    h.tick(3)
    assert len(h.sent) == 3
    assert h.scheduler.motion_status() == 1
    h.result(2, now=3.2)
    assert h.scheduler.motion_status() == 0


def test_late_known_successor_starts_with_new_segment_velocity_and_ends_at_zero():
    h = Harness()
    queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    assert len(h.sent[0][0].trajectory.points) == 1
    queue(h.scheduler, delta=5, now=.5)
    h.tick(1)
    points = h.sent[1][0].trajectory.points
    assert points[0].positions[1] == pytest.approx(math.radians(-40))
    assert points[0].velocities[1] == pytest.approx(math.radians(5))
    assert list(points[-1].velocities) == [0.0] * 4
    assert list(points[0].accelerations) == list(points[-1].accelerations) == []


@pytest.mark.parametrize('next_command,next_delta', [('boom_up', 10), ('arm_extend', 10), ('boom_down', 500)])
def test_boundary_velocities_do_not_push_past_reversal_hold_or_joint_limit(next_command, next_delta):
    h = Harness()
    queue(h.scheduler, delta=500 if next_delta == 500 else 10)
    queue(h.scheduler, next_delta, command=next_command)
    h.tick(0)
    h.accept(0)
    assert h.sent[0][0].trajectory.points[-1].velocities[1] == 0
    h.tick(1)
    points = h.sent[1][0].trajectory.points
    assert points[0].velocities[1] == points[-1].velocities[1] == 0
    assert math.radians(JOINT_LIMITS_DEG['boom'][0]) <= points[-1].positions[1] <= math.radians(JOINT_LIMITS_DEG['boom'][1])


@pytest.mark.parametrize('wait,has_next', [(True, True), (False, False)])
def test_commit_one_and_no_successor_wait_for_result(wait, has_next):
    h = Harness()
    queue(h.scheduler, wait=wait)
    if has_next:
        queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    h.tick(1)
    h.tick(1.6)
    assert len(h.sent) == 1
    assert h.scheduler.motion_status() == 1
    h.result(0, now=1.6)
    h.tick(1.62)
    assert len(h.sent) == (2 if has_next else 1)


def test_uncommitted_successor_is_not_replaced():
    h = Harness()
    queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    h.scheduler.begin(now=.5)
    h.scheduler.update_http_target('boom_down', 10, now=.5)
    h.tick(1)
    assert len(h.sent) == 1


@pytest.mark.parametrize('stop', ['stop', 'reset', 'lease_expired', 'stale'])
def test_stop_has_priority_over_replacement_at_boundary(stop):
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.tick(0)
    handle = h.accept(0)
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=1):
        if stop in ('stop', 'reset'):
            h.bridge._command_callback(stop, 0)
        elif stop == 'lease_expired':
            h.bridge._lease_enabled = True
            h.bridge._client_lease_sec = .5
        else:
            h.scheduler._batch_open_timeout_sec = .5
            h.scheduler.begin(now=.2)
    h.tick(1)
    assert len(h.sent) == 1
    assert handle.cancel_calls == 1
    assert h.scheduler.motion_queue() == 0
    assert h.scheduler.motion_error() == ('' if stop == 'reset' else 'stopped' if stop == 'stop' else stop)


@pytest.mark.parametrize('stop', ['stop', 'reset', 'lease_expired', 'stale'])
def test_stop_during_replacement_response_cancels_new_goal_and_keeps_gate(stop):
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.tick(0)
    old = h.accept(0)
    h.tick(1)
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=1.01):
        h.bridge._stop_motion('stopped' if stop in ('stop', 'reset') else stop, reset=stop == 'reset')
    assert old.cancel_calls == 1
    new = h.accept(1, now=1.02)
    assert new.cancel_calls == 1
    queue(h.scheduler, now=1.03)
    h.result(0, GoalStatus.STATUS_CANCELED, -1, now=1.03)
    h.tick(1.04)
    assert len(h.sent) == 2
    h.result(1, GoalStatus.STATUS_CANCELED, 0, now=1.05)
    h.tick(1.06)
    assert len(h.sent) == 3


@pytest.mark.parametrize('old_terminal', [False, True])
def test_rejected_replacement_is_fatal_and_waits_for_old_terminal(old_terminal):
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.tick(0)
    old = h.accept(0)
    h.tick(1)
    if old_terminal:
        h.result(0, now=1)
    h.accept(1, now=1.01, accepted=False)
    assert h.scheduler.motion_error() == 'goal_rejected'
    assert old.cancel_calls == 1
    assert (h.scheduler._in_flight is None) == old_terminal
    if not old_terminal:
        h.result(0, GoalStatus.STATUS_CANCELED, now=1.02)
    assert h.scheduler._in_flight is None


def test_send_exception_keeps_old_goal_and_can_retry_replacement():
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    previous = h.scheduler._in_flight
    h.bridge._trajectory_client.send_goal_async.side_effect = RuntimeError('send failed')
    h.tick(1)
    assert h.scheduler._in_flight == previous
    assert h.scheduler.motion_error() == ''
    h.bridge._trajectory_client.send_goal_async.side_effect = h.send
    h.tick(1.02)
    assert len(h.sent) == 2
    h.accept(1, now=1.02)
    assert h.scheduler._in_flight.target.positions_rad[1] == pytest.approx(math.radians(-30))


def test_unknown_replacement_response_stops_and_does_not_reopen_gate():
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.tick(0)
    old = h.accept(0)
    h.tick(1)
    h.sent[1][1]._exception = RuntimeError('lost response')
    h.accept(1, now=1.01)
    h.result(0, GoalStatus.STATUS_CANCELED, now=1.02)
    h.tick(1.04)
    assert old.cancel_calls == 1
    assert h.scheduler.motion_error() == 'result_timeout'
    assert h.bridge._goal_response_future is not None
    assert len(h.sent) == 2


def test_relative_targets_accumulate_after_tolerance_then_clamp():
    h = Harness()
    for index, delta in enumerate((20, 25, 500)):
        queue(h.scheduler, delta, now=index)
        h.tick(index)
        index = len(h.sent) - 1
        h.accept(index, now=index)
        h.result(index, GoalStatus.STATUS_ABORTED, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, now=index + 1)
    targets = [math.degrees(goal.trajectory.points[-1].positions[1]) for goal, _, _ in h.sent]
    assert targets == pytest.approx([-30, -5, JOINT_LIMITS_DEG['boom'][1]])
    assert h.scheduler.motion_error() == ''
    assert h.scheduler.motion_status() == 0


@pytest.mark.parametrize('end', ['stop', 'reset', 'fatal'])
def test_relative_and_hold_basis_return_to_measured_after_stop_reset_or_fatal(end):
    s = scheduler()
    queue(s, 20)
    goal = s.pending_snapshot(now=0)
    s.mark_submitted(goal)
    s.mark_completed(goal, successful=end != 'fatal', now=1)
    if end != 'fatal':
        s.stop(reset=end == 'reset')
    queue(s, 1, command='arm_extend', now=1)
    following = s.pending_snapshot(now=1)
    assert following.target.positions_rad[1] == pytest.approx(math.radians(-50))
    assert following.target.positions_rad[2] == pytest.approx(2 - math.radians(1))


def test_speed_duration_uses_commanded_distance_despite_large_tracking_error():
    s = scheduler()
    queue(s, 10, speed=10)
    first = s.pending_snapshot(now=0)
    s.mark_submitted(first)
    queue(s, 10, speed=10, now=.1)
    s.update_from_joint_state(NAMES, [0., math.radians(-70), 2., 1.], now=1)
    following = s.pending_snapshot(now=1, replacing=first)
    assert following.target.duration_sec == pytest.approx(1)
    assert following.target.positions_rad[1] == pytest.approx(math.radians(-30))


def test_result_before_boundary_timer_still_preserves_velocity_for_final_goal():
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler, wait=True)
    h.tick(0)
    h.accept(0)
    previous = h.sent[0][0].trajectory.points[-1]
    h.result(0, now=1)
    h.tick(1.01)
    points = h.sent[1][0].trajectory.points
    assert points[0].positions == previous.positions
    assert points[0].velocities == previous.velocities
    assert list(points[-1].velocities) == [0.0] * 4


@pytest.mark.parametrize('deltas', [(10, 5, 10), (500, 5, -10), (10, -10, 5)])
def test_cubic_interpolation_stays_between_clamped_commanded_endpoints(deltas):
    s = scheduler()
    for delta in deltas:
        queue(s, abs(delta), command='boom_down' if delta >= 0 else 'boom_up')
    previous = None
    for _ in deltas:
        goal = s.pending_snapshot(now=0, replacing=previous)
        points = zip(goal.start_positions_rad, goal.target.positions_rad,
                     goal.start_velocities_rad_s, goal.end_velocities_rad_s)
        for start, end, v0, v1 in points:
            duration = goal.target.duration_sec
            for step in range(101):
                u = step / 100
                position = ((2*u**3 - 3*u**2 + 1)*start + (u**3 - 2*u**2 + u)*duration*v0
                            + (-2*u**3 + 3*u**2)*end + (u**3 - u**2)*duration*v1)
                assert min(start, end) - 1e-12 <= position <= max(start, end) + 1e-12
        with patch('scratch_hci_bridge.trajectory_scheduler.time.monotonic', return_value=0):
            assert s.mark_submitted(goal, replacing=previous)
        previous = goal


@pytest.mark.parametrize('direction,expected', [('swing_left', 210), ('swing_right', 170)])
def test_replacement_preserves_unwrapped_swing_across_180_degrees(direction, expected):
    s = scheduler()
    s.update_from_joint_state(NAMES, [math.radians(170), math.radians(-50), 2., 1.], now=0)
    queue(s, 20, command='swing_left')
    first = s.pending_snapshot(now=0)
    s.mark_submitted(first)
    queue(s, 20, command=direction, now=.1)
    following = s.pending_snapshot(now=1, replacing=first)
    assert math.degrees(following.start_positions_rad[0]) == pytest.approx(190)
    assert math.degrees(following.target.positions_rad[0]) == pytest.approx(expected)
    assert abs(following.target.positions_rad[0] - following.start_positions_rad[0]) < math.pi


def test_replacement_does_not_dispatch_with_stale_feedback():
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.scheduler._feedback_timeout_sec = .5
    h.tick(0)
    h.accept(0)
    h.tick(1)
    assert len(h.sent) == 1
    h.scheduler.update_from_joint_state(NAMES, [0., math.radians(-50), 2., 1.], now=1.01)
    h.tick(1.02)
    assert len(h.sent) == 2


def test_batch_expiring_in_replacement_acceptance_stops_new_goal():
    h = Harness()
    queue(h.scheduler)
    queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    h.tick(1)
    h.scheduler._batch_open_timeout_sec = .1
    h.scheduler.begin(now=1)
    with patch.object(h.bridge, '_publish_unity_position_target') as publish:
        new = h.accept(1, now=1.2)
    publish.assert_not_called()
    assert h.scheduler.motion_error() == 'stale'
    assert new.cancel_calls == 1
    assert h.scheduler._in_flight is h.bridge._active_goal_scheduled
    h.result(1, GoalStatus.STATUS_CANCELED, now=1.21)
    assert h.scheduler._in_flight is None

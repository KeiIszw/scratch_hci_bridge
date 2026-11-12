"""関節別の計画・HTTP受付・Action置換を実機なしで検証する。"""
import math
import threading
from unittest.mock import patch

import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory

from scratch_hci_bridge.command_state import CommandState
from scratch_hci_bridge.http_server import create_http_server
from scratch_hci_bridge.track_timeline import TrackTimeline
from test_http_server import _get
from test_sync_safety import scheduler
from bridge_test_helpers import Harness, queue
from test_catch_up_axis_mask import harness, mask


def add(s, command='boom_down', delta=10, speed=10, now=0, duration=3):
    return s.update_http_target(command, delta, speed_limit_deg_s=speed, base_duration_sec=duration, now=now)


def test_independent_positions_times_and_union():
    t = TrackTimeline((0, -1, 2, 1))
    t.add('boom', 0, 4, 0)
    before = t.state('boom', 1)
    t.add('bucket', 2, 1, 1)
    assert t.state('boom', 1) == before
    assert t.axes['boom'][0].end == 4
    assert [p.time for p in t.points(1)] == [0, 1, 3]
    assert t.points(1)[0].positions[1] == -.75
    assert t.points(1)[1].positions[1] == -.5
    assert t.active_axes(1) == {'boom', 'bucket'}
    assert t.active_axes(2) == {'boom'}
    assert t.points(4)[0].velocities == (0, 0, 0, 0)


@pytest.mark.parametrize('target,expected', [(3, 1), (1, 0), (2, 0)])
def test_fifo_origin_and_boundary_velocity(target, expected):
    t = TrackTimeline((0, 0, 0, 0))
    t.add('boom', 2, 1, 0)
    t.add('boom', target, 1, .1)
    assert t.queue('boom', .2) == 1
    assert t.axes['boom'][1].origin == 2
    assert t.axes['boom'][1].start == 1
    assert t.state('boom', 1) == (2, expected)
    assert t.state('boom', 2) == (target, 0)
    with pytest.raises(ValueError, match='full'):
        t.add('boom', 4, 1, .2)
    assert t.queue('boom', 1) == 0
    t.add('boom', 4, 1, 1)


@pytest.mark.parametrize('speed,duration', [(None, 3), (5, 2), (20, .5)])
def test_speed_duration_and_signed_magnitude(speed, duration):
    s = scheduler()
    assert add(s, delta=-10, speed=speed)
    p = s.pending_snapshot(now=0)
    assert p.target.duration_sec == pytest.approx(duration)
    assert math.degrees(p.target.positions_rad[1]) == pytest.approx(-40)


def test_clamp_duration_and_swing_limit():
    s = scheduler()
    assert add(s, delta=1000, speed=10)
    p = s.pending_snapshot(now=0)
    assert math.degrees(p.target.positions_rad[1]) == pytest.approx(48)
    assert p.target.duration_sec == pytest.approx(9.8)
    assert not add(s, 'swing_left', 180)
    assert add(s, 'swing_right', -30)


def test_queue_and_busy_through_final_result():
    s = scheduler()
    assert add(s)
    assert s.joint_queue(1, now=0) == 0
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    assert add(s, delta=5, now=.1)
    assert s.joint_queue(1, now=.1) == 1
    assert not add(s, delta=3, now=.2)
    q = s.pending_snapshot(now=.2, replacing=p)
    assert math.degrees(q.target.positions_rad[1]) == pytest.approx(-35)
    s.mark_submitted(q, replacing=p)
    assert s.pending_snapshot(now=.3, replacing=q) is None
    boundary = s._timeline.axes['boom'][1].start
    assert s.joint_queue(1, now=boundary) == 0
    r = s.pending_snapshot(now=boundary, replacing=q)
    assert r.track_points[0].velocities[1] == pytest.approx(math.radians(10))
    s.mark_submitted(r, replacing=q)
    assert s.motion_status() == 1
    assert s.pending_snapshot(now=2, replacing=r) is None
    s.mark_completed(r, successful=True, now=2)
    assert s.motion_status() == 0


@pytest.mark.parametrize('reason,reset', [('stopped', False), ('stopped', True), ('lease_expired', False), ('stale', False)])
def test_stop_clears_all_tracks_and_rebases(reason, reset):
    s = scheduler()
    add(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    add(s, now=.1)
    s.stop(reason, reset=reset)
    assert all(s.joint_queue(i, now=.1) == 0 for i in range(4))
    assert s.track_active_axes(now=.1) == set()
    s.mark_completed(p, successful=False, now=.2)
    assert add(s, now=.2)
    q = s.pending_snapshot(now=.2)
    assert math.degrees(q.target.positions_rad[1]) == pytest.approx(-40)


def test_http_track_reporter_rejection_and_speed_snapshot():
    h = Harness()
    s = h.scheduler
    server = create_http_server('127.0.0.1', 0, h.bridge._state, h.bridge._command_callback,
                                joint_queue_callback=s.joint_queue)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}/?'
    def get(typ, cmd, val):
        return _get(base+f'type={typ}&cmd={cmd}&val={val}')
    try:
        # HTTP受付時の単調時計を0へ揃え、ROSタイマーはダミーのままにする。
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=0):
            assert get('command', 'boom_speed', 5)[0] == 200
            assert get('command', 'boom_down', 10)[0] == 200
            assert get('command', 'boom_speed', 20)[0] == 200
            assert get('command', 'boom_down', 10)[0] == 200
            assert get('command', 'boom_down', 10)[0] == 400
            assert get('reporter', 'joint_queue', 1)[1] == '1'
            assert get('reporter', 'joint_queue', 0)[1] == '0'
            assert get('reporter', 'joint_queue', 4)[0] == 400
            assert get('command', 'boom_time', 2)[0] == 400
            p = s.pending_snapshot(now=0)
            assert p.target.duration_sec == pytest.approx(2.5)
            assert [pt.time for pt in p.track_points] == pytest.approx([0, 2, 2.5])
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


@pytest.mark.parametrize('before_accept', [True, False])
def test_node_replacement_ignores_preempted_results_and_keeps_plan(before_accept):
    h = Harness()
    add(h.scheduler, delta=20, speed=5)
    h.tick(0)
    h.accept(0)
    add(h.scheduler, command='bucket_dump', delta=10, speed=10, now=1)
    h.tick(1)
    points = h.sent[1][0].trajectory.points
    assert math.degrees(points[0].positions[1]) == pytest.approx(-45)
    assert points[-1].time_from_start.sec == 3
    if before_accept:
        h.result(0, GoalStatus.STATUS_CANCELED, -1, now=1)
    h.accept(1, now=1)
    if not before_accept:
        h.result(0, GoalStatus.STATUS_CANCELED, -1, now=1)
    assert h.scheduler.motion_status() == 1
    h.tick(4)
    assert len(h.sent) == 2
    h.result(1, now=4)
    assert h.scheduler.motion_status() == 0


@pytest.mark.parametrize('code', [FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                  FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED])
def test_final_track_goal_has_stopping_tolerance_and_fatal_failure(code):
    h = Harness()
    add(h.scheduler)
    h.tick(0)
    h.accept(0)
    tol = h.sent[0][0].goal_tolerance[1]
    assert tol.position > 0 and tol.velocity > 0
    h.result(0, GoalStatus.STATUS_ABORTED, code, now=1)
    assert h.scheduler.motion_status() == -1
    assert h.scheduler.joint_queue(1, now=1) == 0


@pytest.mark.parametrize('catchup', [True, False])
def test_track_axis_mask_removes_completed_axes_and_stop_clears(catchup):
    h = harness(catchup)
    add(h.scheduler, delta=10, speed=10)
    h.tick(0)
    h.accept(0)
    add(h.scheduler, command='bucket_dump', delta=20, speed=5, now=.2)
    h.tick(.2)
    h.accept(1, now=.2)
    assert mask(h) == 0b1010
    h.tick(1.1)
    assert mask(h) == (0b1010 if catchup else 0b1000)
    h.bridge._command_callback('stop', 0)
    assert mask(h) == 0
    assert h.scheduler.joint_queue(1, now=1.1) == 0


def test_track_before_feedback_waits_and_still_captures_received_speed():
    from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler
    from test_sync_safety import NAMES
    s = TrajectoryScheduler(feedback_timeout_sec=0)
    assert add(s, speed=5)
    assert s.joint_queue(1, now=0) == 1
    assert s.motion_status() == 1
    assert s.pending_snapshot(now=0) is None
    s.update_from_joint_state(NAMES, [0, math.radians(-60), 2, 1], now=.1)
    p = s.pending_snapshot(now=.1)
    assert p.target.duration_sec == 2
    assert math.degrees(p.target.positions_rad[1]) == pytest.approx(-50)


@pytest.mark.parametrize('reason', ['stop', 'reset', 'lease', 'reject'])
def test_track_stop_during_replacement_response_and_rejection(reason):
    h = harness()
    add(h.scheduler)
    h.tick(0)
    old = h.accept(0)
    add(h.scheduler, command='bucket_dump', now=.2)
    h.tick(.2)
    if reason in ('stop', 'reset'):
        h.bridge._command_callback(reason, 0)
    elif reason == 'lease':
        h.bridge._lease_enabled = True
        h.bridge._client_lease_sec = .1
        h.tick(.3)
    else:
        h.accept(1, now=.3, accepted=False)
        assert old.cancel_calls == 1
        assert mask(h) == 0
        assert h.scheduler.motion_status() == -1
        assert all(h.scheduler.joint_queue(i, now=.3) == 0 for i in range(4))
        return
    new = h.accept(1, now=.3)
    assert old.cancel_calls == new.cancel_calls == 1
    assert mask(h) == 0
    h.result(1, GoalStatus.STATUS_CANCELED, now=.4)
    assert h.scheduler.motion_status() == (-1 if reason == 'lease' else 0 if reason == 'reset' else 2)


def test_subnanosecond_boundaries_do_not_duplicate_time_zero():
    t = TrackTimeline((0, 0, 0, 0))
    t.add('boom', 1, 1+.0000000001, 0)
    t.add('boom', 2, 1, .2)
    points = t.points(1)
    assert [p.time for p in points] == [0, 1]


def test_two_long_segments_exceeding_ros_duration_reject_only_new_segment():
    from scratch_hci_bridge.command_state import MAX_ROS_DURATION_SEC
    s = scheduler()
    assert add(s, speed=None, duration=MAX_ROS_DURATION_SEC*.75)
    before = s.pending_snapshot(now=0)
    assert not add(s, speed=None, duration=MAX_ROS_DURATION_SEC*.75, now=.1)
    after = s.pending_snapshot(now=0)
    assert before.track_points == after.track_points
    assert len(after.commands) == 1


def test_barrier_holds_short_axis_and_preserves_existing_plan_on_prefetch():
    s = scheduler()
    assert add(s, delta=40, speed=10)  # boomは4秒、bucketは1秒。
    assert add(s, command='bucket_dump', delta=10, speed=10)
    p = s.pending_snapshot(now=0)
    assert s.mark_submitted(p)
    before = tuple(s._timeline.axes['boom'])
    assert s.barrier(now=.1) == pytest.approx(4)
    assert add(s, command='bucket_dump', delta=5, speed=5, now=.1)
    assert s.joint_queue(3, now=.1) == 1
    assert not add(s, command='bucket_dump', now=.2)
    assert s._timeline.axes['bucket'][1].start == 4
    assert s._timeline.state('bucket', 2) == s._timeline.state('bucket', 3)
    assert s._timeline.state('bucket', 2)[1] == 0
    assert tuple(s._timeline.axes['boom']) == before
    assert s._timeline.barrier_time == 4
    replacement = s.pending_snapshot(now=.2, replacing=p)
    assert replacement.track_points[0].positions[1] == pytest.approx(math.radians(-48))
    assert s.mark_submitted(replacement, replacing=p)
    assert s.barrier(now=.3) == pytest.approx(5)
    assert tuple(s._timeline.axes['boom']) == before
    assert s._timeline.axes['bucket'][1].start == 4


def test_barrier_allows_same_direction_at_equal_end_without_stop():
    s = scheduler()
    assert add(s, delta=10, speed=10)
    assert s.barrier(now=.1) == pytest.approx(1)
    assert add(s, delta=5, speed=5, now=.1)
    t = s._timeline
    boundary = t.axes['boom'][1].start
    assert boundary == pytest.approx(1)
    assert t.state('boom', boundary)[1] == pytest.approx(math.radians(5))
    # 追加・区切りを進めても、既存の区間位置と時刻はそのまま。
    old = tuple(t.axes['boom'])
    s.barrier(now=.2)
    assert add(s, command='arm_extend', delta=5, now=.2)
    assert tuple(t.axes['boom']) == old
    assert t.axes['arm'][0].start == pytest.approx(2)


@pytest.mark.parametrize('cmd,target', [('boom_angle', -30), ('arm_angle', 100), ('bucket_angle', -31), ('swing_angle', -170)])
def test_absolute_targets_append_to_same_timeline(cmd, target):
    s = scheduler()
    assert s.update_http_target(cmd, target, speed_limit_deg_s=10, now=0)
    axis = cmd.split('_')[0]
    first = s._timeline.axes[axis][0]
    assert math.degrees(first.target) == pytest.approx(target)
    assert s.update_http_target(cmd, target+5, speed_limit_deg_s=5, now=.01)
    second = s._timeline.axes[axis][1]
    assert second.start == first.end
    assert second.origin == first.target
    assert second.end-second.start == pytest.approx(1)


def test_barrier_before_feedback_preserves_order_and_speed():
    from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler
    from test_sync_safety import NAMES
    s = TrajectoryScheduler(feedback_timeout_sec=0)
    assert add(s, delta=10, speed=5)
    s.barrier(now=.1)
    assert add(s, command='arm_extend', delta=10, speed=10, now=.2)
    assert s.joint_queue(1, now=.2) == 1
    s.update_from_joint_state(NAMES, [0, math.radians(-50), 2, 1], now=.3)
    p = s.pending_snapshot(now=.3)
    assert p is not None
    assert s._timeline.axes['boom'][0].start == .3
    assert s._timeline.axes['arm'][0].start == pytest.approx(2.3)
    assert s._timeline.barrier_time == pytest.approx(2.3)


def test_success_of_previous_revision_keeps_new_segments_and_send_retry_basis():
    s = scheduler()
    assert add(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    s.barrier(now=.1)
    assert add(s, command='arm_extend', now=.1)
    assert s.mark_completed(p, successful=True, now=1)
    assert s.motion_status() == 1
    q = s.pending_snapshot(now=1)
    assert q.track_points[0].positions[1] == pytest.approx(math.radians(-40))
    s.mark_submitted(q)
    before = tuple(s._timeline.axes['arm'])
    s.mark_send_failed(q)
    r = s.pending_snapshot(now=1.1)
    assert tuple(s._timeline.axes['arm']) == before
    assert r.target.positions_rad == q.target.positions_rad


def test_stale_feedback_during_motion_clears_timeline_and_waits_for_cancel():
    s = scheduler()
    s._feedback_timeout_sec = .5
    assert add(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    s.check_deadlines(now=.51)
    assert s.motion_status() == -1
    assert s.motion_error() == 'stale'
    assert s.needs_cancel()
    assert s._timeline is None
    assert s.mark_completed(p, successful=False, now=.6)
    assert not s.has_work()


def test_empty_barrier_does_not_freeze_measured_basis():
    s = scheduler()
    s.barrier(now=0)
    s.update_from_joint_state(['boom_joint'], [math.radians(-60)], now=.2)
    assert add(s, now=.2)
    assert math.degrees(s._timeline.axes['boom'][0].origin) == pytest.approx(-60)


def test_pending_feedback_overflow_fails_without_partial_trajectory():
    from scratch_hci_bridge.command_state import MAX_ROS_DURATION_SEC
    from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler
    from test_sync_safety import NAMES
    s = TrajectoryScheduler(feedback_timeout_sec=0)
    assert add(s, speed=None, duration=MAX_ROS_DURATION_SEC*.75)
    s.barrier(now=.1)
    assert add(s, command='arm_extend', speed=None, duration=MAX_ROS_DURATION_SEC*.75, now=.1)
    s.update_from_joint_state(NAMES, [0,-1,2,1], now=.2)
    assert s.pending_snapshot(now=.2) is None
    assert s.motion_status() == -1
    assert s.motion_error() == 'invalid_trajectory'
    assert s._timeline is None


def test_new_commands_during_reset_wait_for_cancel_and_use_latest_feedback():
    from test_sync_safety import NAMES
    s = scheduler()
    assert add(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    s.stop(reset=True)
    assert add(s, now=.1)
    s.barrier(now=.1)
    assert add(s, command='arm_extend', now=.1)
    assert s._timeline is None
    assert s.joint_queue(1, now=.1) == 1
    s.update_from_joint_state(NAMES, [0,math.radians(-60),2,1],now=5)
    s.mark_completed(p,successful=False,now=5)
    s.check_deadlines(now=5.1)
    q = s.pending_snapshot(now=5.1)
    assert q is not None
    assert math.degrees(q.track_points[0].positions[1]) == pytest.approx(-60)
    assert math.degrees(q.target.positions_rad[1]) == pytest.approx(-50)
    assert s._timeline.axes['boom'][0].start == 5.1

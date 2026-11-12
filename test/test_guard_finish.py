"""接地ガードの正常な打切り・惰性下降・遅着結果と後続指令をROSなしで検証する。"""
import math
from unittest.mock import Mock, patch

import pytest
from action_msgs.msg import GoalStatus
from test_sync_safety import scheduler, NAMES
from test_ground_guard import start
from bridge_test_helpers import Harness


def feedback(s, t, boom=-50, distance=.1):
    """実際の受信だけを進め、timerの再読込で停滞を作らない。"""
    s.update_from_joint_state(NAMES, [0, math.radians(boom), 2, 1], now=t)
    s.update_ground_contact(0, now=t)
    s.update_ground_clearance(distance, now=t)


def finish_hold(s, now):
    """検知後の静止確認と最終保持goalの終端まで進める。"""
    for dt in (.1, .3, .6):
        feedback(s, now+dt, math.degrees(s._guard_hold_position))
    goal = s.pending_snapshot(now=now+.6)
    assert s.mark_submitted(goal)
    assert s.mark_completed(goal, successful=True, now=now+.6)
    assert s.motion_status() == 0 and s.motion_error() == ''


@pytest.mark.parametrize('distance,status', [(.1, 'stalled'), (.4, 'blocked')])
def test_stall_finishes_without_failure_and_next_command_works(distance, status):
    s = scheduler(); start(s)
    for t in (.1, .3, .61):
        feedback(s, t, distance=distance)
    assert s.check_ground_guard(.61) is None
    assert s.guard_status() == status and s.motion_status() == 1
    finish_hold(s, .61)
    assert s.update_http_target('boom_up', 2, now=1.3)
    assert s.motion_status() == 1


def test_timer_without_new_feedback_does_not_count_as_stall():
    s = scheduler(); start(s)
    feedback(s, .1)
    for t in (.2, .4, .59):
        s.update_ground_contact(0, now=t)
        s.check_ground_guard(t)
    assert s.guard_status() == 'running'


def test_long_cubic_start_does_not_false_stall_before_commanded_progress():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard(40, speed_limit_deg_s=.2, now=0)
    for t in (.1, .3, .61):
        feedback(s, t)
        s.check_ground_guard(t)
    assert s.guard_status() == 'running'


def test_far_stall_classification_ignores_two_low_distance_outliers():
    s = scheduler(); start(s)
    for t in (.1, .2, .3, .4, .5):
        feedback(s, t, distance=.4)
    for t in (.55, .61):
        feedback(s, t, distance=-.18)
    s.check_ground_guard(.61)
    assert s.guard_status() == 'blocked'


def test_post_hold_drift_and_other_axis_goal_never_emit_upward_boom_target():
    s = scheduler(); start(s)
    s.update_ground_contact(1, now=.1); s.check_ground_guard(.1)
    finish_hold(s, .1)
    feedback(s, .8, -49.98)
    assert s.update_http_target('arm_extend', 2, now=.8)
    goal = s.pending_snapshot(now=.8)
    assert all(p.positions[1] >= math.radians(-49.98) for p in goal.track_points)


def test_finished_guard_does_not_require_ground_sensor_for_later_ordinary_motion():
    s = scheduler(); start(s)
    s.update_ground_contact(1, now=.1); s.check_ground_guard(.1)
    finish_hold(s, .1)
    s.update_ground_contact(-1, now=.8)
    assert s.update_http_target('arm_extend', 2, now=.8)
    assert s.check_ground_guard(.8) is None
    assert s.motion_status() == 1


def test_reached_holds_actual_and_next_command_uses_that_basis():
    s = scheduler(); start(s)
    target = s._guard_segment.target
    feedback(s, 2.1, math.degrees(target))
    s.check_ground_guard(2.1)
    assert s.guard_status() == 'reached'
    finish_hold(s, 2.1)
    assert s.update_http_target('boom_up', 1, now=2.8)
    assert s._timeline.axes['boom'][-1].target == pytest.approx(target-math.radians(1))


def test_overshoot_follows_measured_and_stale_hold_never_enables_upward_output():
    from test_catch_up_axis_mask import harness, mask
    h = harness(); s = h.scheduler; start(s)
    h.tick(0); h.accept(0)
    s.update_ground_contact(1, now=.1); h.tick(.1)
    old_hold = h.sent[1][0].trajectory.points[-1].positions[1]
    for t, angle in ((.2, -49), (.3, -48)):
        feedback(s, t, angle)
        assert s._guard_hold_position >= math.radians(angle)
        assert mask(h) == 0
    h.accept(1, now=.3)
    assert mask(h) == 0
    assert old_hold < math.radians(-48)
    h.tick(.31)
    points = h.sent[2][0].trajectory.points
    assert all(p.positions[1] >= math.radians(-48) for p in points)
    assert all(p.velocities[1] == 0 for p in points)
    h.accept(2, now=.31)
    # 直後の成功で静止確認を飛ばさず、その後に固定された位置で完了する。
    h.result(2, now=.32)
    assert s.motion_status() == 1
    feedback(s, .9, -48)
    h.tick(.9); h.accept(3, now=.9); h.result(3, now=.91)
    assert s.motion_status() == 0
    assert s.update_http_target('boom_up', 1, now=1)
    assert s._timeline.axes['boom'][-1].origin == pytest.approx(math.radians(-48))


def test_tolerance_result_on_stalled_descent_recovers_without_latch():
    h = Harness(); s = h.scheduler; start(s)
    h.tick(0); h.accept(0)
    for t in (.1, .3, .61):
        feedback(s, t)
    h.result(0, status=GoalStatus.STATUS_ABORTED, code=-5, now=.61)
    assert s.guard_status() == 'stalled' and s.motion_error() == ''
    assert s.motion_status() == 1
    h.tick(.62)
    assert len(h.sent) == 2


def test_delayed_hold_acceptance_does_not_publish_old_unity_boom_position():
    h = Harness(); s = h.scheduler
    h.bridge._unity_position_publishers = {axis: Mock() for axis in ('swing', 'boom', 'arm', 'bucket')}
    h.bridge._unity_position_signs = dict.fromkeys(h.bridge._unity_position_publishers, 1.)
    start(s); h.tick(0); h.accept(0)
    s.update_ground_contact(1, now=.1); h.tick(.1)
    feedback(s, .2, -48)
    boom = h.bridge._unity_position_publishers['boom']
    count = boom.publish.call_count
    h.accept(1, now=.2)
    assert boom.publish.call_count == count
    h.tick(.21); h.accept(2, now=.21)
    assert boom.publish.call_args.args[0].data == pytest.approx(math.radians(-48))


def test_tolerance_failure_of_other_axis_is_not_hidden_by_boom_stall():
    s = scheduler(); start(s)
    s.update_http_target('arm_extend', 10, now=0)
    goal = s.pending_snapshot(now=0); s.mark_submitted(goal)
    for t in (.1, .3, .61): feedback(s, t)
    s.mark_completed(goal, successful=False, reason='tolerance_violated', now=.61)
    assert s.motion_status() == -1


@pytest.mark.parametrize('reason', ['stopped', 'ground_unknown', 'tolerance_violated'])
def test_stop_failure_reset_and_late_acceptance_cannot_restore_old_goal(reason):
    h = Harness(); s = h.scheduler; start(s); h.tick(0)
    s.update_ground_contact(1, now=.1); h.tick(.1)
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=.2):
        h.bridge._stop_motion(reason)
        h.bridge._stop_motion(reset=True)
    feedback(s, .3, -45)
    h.accept(0, now=.3)
    h.result(0, status=GoalStatus.STATUS_CANCELED, now=.4)
    h.tick(.4)
    assert len(h.sent) == 1 and s.pending_snapshot(now=.4) is None
    assert s.update_http_target('boom_up', 1, now=.4)
    assert s._timeline.axes['boom'][-1].origin == pytest.approx(math.radians(-45))

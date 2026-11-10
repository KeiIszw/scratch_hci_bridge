"""ダミーActionと時計で、未到達軸の出力許可と停止時の解除を検証する。"""
import math
from unittest.mock import Mock, patch

import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory

from scratch_hci_bridge.node import ScratchHciBridge
from test_goal_time_tolerance import _BeforeRosInterfaces, _initialize_without_ros
from test_node_command_callback import _Publisher
from test_sync_safety import NAMES, scheduler
from test_trajectory_chaining import Harness, queue


def harness(enabled=True):
    h = Harness()
    h.bridge._catch_up_axes_in_mask = enabled
    h.bridge._active_axis_mask_publisher = _Publisher()
    return h


def mask(h):
    return h.bridge._active_axis_mask_publisher.values[-1]


def feedback(h, boom=-50, now=0, swing=0, bucket=1):
    h.scheduler.update_from_joint_state(
        NAMES, [math.radians(swing), math.radians(boom), 2, bucket], now=now)


def replace_with_bucket(h):
    queue(h.scheduler)
    queue(h.scheduler, command='bucket_scoop')
    h.tick(0)
    h.accept(0)
    assert mask(h) == 0b0010
    h.tick(1)
    h.accept(1, now=1)


@pytest.mark.parametrize('enabled, expected', [(True, 0b1010), (False, 0b1000)])
def test_replacement_keeps_unreached_boom_only_when_enabled(enabled, expected):
    h = harness(enabled)
    replace_with_bucket(h)
    assert mask(h) == expected
    assert h.sent[1][0].trajectory.points[-1].positions[1] == pytest.approx(math.radians(-40))
    h.tick(1.1)
    assert mask(h) == expected
    feedback(h, boom=-40.5, now=1.12)
    h.tick(1.12)
    assert mask(h) == 0b1000
    # 到達して外れた軸は、その後の実測変化だけでは再許可しない。
    feedback(h, boom=-50, now=1.14)
    h.tick(1.14)
    assert mask(h) == 0b1000


def test_multiple_catch_up_axes_survive_further_replacement_and_expire_independently():
    h = harness()
    queue(h.scheduler)
    queue(h.scheduler, command='swing_left')
    queue(h.scheduler, command='bucket_scoop')
    h.tick(0)
    h.accept(0)
    h.tick(1)
    h.accept(1, now=1)
    assert mask(h) == 0b0011
    h.tick(2)
    # 受理応答待ちでも追従中の軸は残す。
    assert mask(h) == 0b1011
    feedback(h, boom=-40.5, now=2.02)
    h.tick(2.02)
    assert mask(h) == 0b1001
    h.accept(2, now=2.02)
    feedback(h, boom=-40.5, swing=10, now=2.04)
    h.tick(2.04)
    assert mask(h) == 0b1000


@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('intermediate', [True, False])
def test_completion_preserves_only_catch_up_axes_until_measured_arrival(enabled, intermediate):
    h = harness(enabled)
    replace_with_bucket(h)
    # bucketは到達済みだが、boomは前goalの保持目標へまだ追従している。
    target = h.bridge._active_goal_scheduled.target.positions_rad
    feedback(h, now=1.1, bucket=target[3])
    if intermediate:
        h.result(1, GoalStatus.STATUS_ABORTED,
                 FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, now=1.1)
    else:
        h.result(1, now=1.1)
    assert not h.scheduler.has_work()
    assert mask(h) == (0b0010 if enabled else 0)
    feedback(h, boom=-40.5, now=1.12, bucket=target[3])
    h.tick(1.12)
    assert mask(h) == 0


@pytest.mark.parametrize('stop', ['stop', 'reset', 'lease', 'fatal', 'stale', 'result_timeout'])
def test_stop_paths_clear_catch_up_and_late_result_does_not_restore_mask(stop):
    h = harness()
    replace_with_bucket(h)
    assert mask(h) == 0b1010
    if stop in ('stop', 'reset'):
        h.bridge._command_callback(stop, 0)
    elif stop == 'lease':
        h.bridge._lease_enabled = True
        h.bridge._last_http_at = 0
        h.tick(2.01)
    elif stop == 'fatal':
        h.result(1, GoalStatus.STATUS_ABORTED,
                 FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, now=1.1)
    elif stop == 'stale':
        h.scheduler.begin(now=1)
        h.tick(6.01)
    else:
        h.bridge._active_goal_deadline = 1.1
        h.tick(1.1)
    assert mask(h) == 0
    assert not h.bridge._mask_catch_up_targets
    if stop != 'fatal':
        h.result(1, GoalStatus.STATUS_CANCELED, now=6.1)
    h.tick(6.12)
    assert mask(h) == 0


def test_lease_also_stops_catch_up_after_last_action_completed():
    h = harness()
    queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    h.result(0, GoalStatus.STATUS_ABORTED,
             FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, now=1)
    assert not h.scheduler.has_work()
    assert mask(h) == 0b0010
    h.bridge._lease_enabled = True
    h.bridge._last_http_at = 0
    h.tick(2.01)
    assert mask(h) == 0
    assert h.scheduler.motion_error() == 'lease_expired'


@pytest.mark.parametrize('failure', ['rejected', 'response_timeout'])
def test_failure_while_replacement_acceptance_is_pending_clears_mask(failure):
    h = harness()
    queue(h.scheduler)
    queue(h.scheduler, command='bucket_scoop')
    h.tick(0)
    h.accept(0)
    h.tick(1)
    assert mask(h) == 0b1010
    if failure == 'rejected':
        h.accept(1, now=1.02, accepted=False)
        assert h.scheduler.motion_error() == 'goal_rejected'
    else:
        h.tick(6.01)
        assert h.scheduler.motion_error() == 'result_timeout'
        # timeout後の遅い受理も追従マスクを復活させない。
        h.accept(1, now=6.02)
    assert mask(h) == 0
    assert not h.bridge._mask_catch_up_targets


def test_shutdown_clears_remaining_catch_up_without_starting_ros():
    h = harness()
    queue(h.scheduler)
    h.tick(0)
    h.accept(0)
    h.result(0, now=1)
    assert mask(h) == 0b0010
    b = h.bridge
    b._timer, b._http_server, b._http_thread, b._drive_publisher = Mock(), Mock(), Mock(), Mock()
    b._trajectory_shutdown_cancel_timeout_sec = 0
    with patch('scratch_hci_bridge.node.rclpy.ok', return_value=True), \
            patch('scratch_hci_bridge.node.Node.destroy_node') as destroy:
        b.destroy_node()
    destroy.assert_called_once()
    assert mask(h) == 0
    assert not b._mask_catch_up_targets


def test_idle_timer_does_not_overwrite_standalone_replay_mask():
    h = harness()
    h.bridge._publish_active_axis_mask()
    published = h.bridge._active_axis_mask_publisher.values
    assert published == [0]
    for now in (0, .02, .04):
        h.tick(now)
    assert published == [0]


def test_send_exception_during_replacement_keeps_old_goal_permission():
    h = harness()
    queue(h.scheduler)
    queue(h.scheduler, command='bucket_scoop')
    h.tick(0)
    h.accept(0)
    h.bridge._trajectory_client.send_goal_async.side_effect = RuntimeError('dummy send failure')
    h.tick(1)
    assert mask(h) == 0b0010
    assert h.bridge._mask_catch_up_targets == {'boom': pytest.approx(math.radians(-40))}


def test_retryable_response_failure_keeps_catch_up_permission_until_stop():
    h = harness()
    queue(h.scheduler)
    h.tick(0)
    response = h.sent[0][1]
    response._exception = RuntimeError('dummy response failure')
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=.02):
        for callback in response.callbacks:
            callback(response)
    assert mask(h) == 0b0010
    assert h.scheduler.motion_error() == ''
    h.bridge._command_callback('stop', 0)
    assert mask(h) == 0


@pytest.mark.parametrize('overrides, expected', [({}, True), ({'catch_up_axes_in_mask': False}, False)])
def test_node_parameter_defaults_and_override(overrides, expected):
    b = ScratchHciBridge.__new__(ScratchHciBridge)
    with pytest.raises(_BeforeRosInterfaces):
        _initialize_without_ros(b, overrides)
    assert b._catch_up_axes_in_mask is expected


def test_swing_same_angle_and_tolerance_boundary_do_not_leave_unneeded_axes():
    s = scheduler()
    s.update_from_joint_state(NAMES, [math.radians(-179), 0, 0, 0], now=0)
    targets = {'swing': math.radians(181), 'boom': .125, 'arm': -.126, 'bucket': .126}
    assert s.targets_outside_tolerance(targets, .125) == {'arm': -.126, 'bucket': .126}
    assert s.targets_outside_tolerance({}, .125) == {}

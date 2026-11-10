"""実機やDDSへ接続せず、途中区間だけの猶予とパラメータ検証を確認する。"""
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

import test_node_command_callback as callbacks
import test_sync_safety as safety
from scratch_hci_bridge.command_state import MAX_ROS_DURATION_SEC
from scratch_hci_bridge.node import ScratchHciBridge


def _sending_bridge():
    bridge = safety.NodeSafetyTest().bridge()
    del bridge._send_pending_trajectory
    bridge._trajectory_scheduler = safety.TrajectoryScheduler(
        batch_wait_sec=0.25, initial_mode_command_wait_sec=0, feedback_timeout_sec=0
    )
    bridge._trajectory_scheduler.update_from_joint_state(
        safety.NAMES, [0.0, math.radians(-50), 2.0, 1.0], now=0
    )
    bridge._trajectory_duration_sec = 1.0
    bridge._trajectory_goal_tolerance_rad = math.radians(0.75)
    bridge._trajectory_stopped_velocity_rad_s = math.radians(0.5)
    bridge._trajectory_client = Mock()
    bridge._trajectory_client.server_is_ready.return_value = True
    bridge._trajectory_client.send_goal_async.side_effect = lambda goal: callbacks._Future()
    return bridge


def _send_all(bridge, now=0.5):
    scheduler = bridge._trajectory_scheduler
    goals = []
    with patch('scratch_hci_bridge.trajectory_scheduler.time.monotonic', return_value=now):
        while scheduler.has_work():
            bridge._send_pending_trajectory()
            scheduled = scheduler._in_flight
            assert scheduled is not None
            goals.append(bridge._trajectory_client.send_goal_async.call_args.args[0])
            scheduler.mark_completed(scheduled, successful=True, now=now)
            # このテストはgoal生成だけを検証するため、ダミーの未応答状態も解放する。
            bridge._goal_response_future = None
    return goals


@pytest.mark.parametrize('mode', ['parallel', 'serial'])
@pytest.mark.parametrize('fatal', [False, True])
@pytest.mark.parametrize('seconds', [0.0, 0.5, 1.25])
def test_commit_zero_goal_time_tolerance(mode, fatal, seconds):
    bridge = _sending_bridge()
    bridge._intermediate_tolerance_failure_fatal = fatal
    bridge._intermediate_goal_time_tolerance_sec = seconds
    scheduler = bridge._trajectory_scheduler
    scheduler.set_execution_mode(mode, now=0)
    scheduler.begin(now=0)
    scheduler.update_http_target('boom_down', 1, now=0)
    scheduler.update_http_target('arm_extend', 1, now=0)
    scheduler.commit(False, now=0)

    goals = _send_all(bridge)

    # 明示バッチは選択中のモードによらずparallelで確定するため、常に1ゴールになる。
    assert len(goals) == 1
    for goal in goals:
        duration = goal.goal_time_tolerance
        assert (duration.sec, duration.nanosec) == (
            int(seconds), int((seconds % 1) * 1_000_000_000)
        )

    # 途中区間の設定を後続の暗黙バッチへ持ち越すと、停止確認の猶予も短くなる。
    scheduler.update_http_target('boom_angle', -45, now=0.5)
    for goal in _send_all(bridge, now=1.0):
        assert (goal.goal_time_tolerance.sec, goal.goal_time_tolerance.nanosec) == (0, 0)


@pytest.mark.parametrize('mode', ['parallel', 'serial'])
@pytest.mark.parametrize('origin', ['commit_one', 'debounce', 'mode_change', 'initial_pose'])
def test_other_goals_keep_controller_default(mode, origin):
    bridge = _sending_bridge()
    bridge._intermediate_goal_time_tolerance_sec = 1.25
    scheduler = bridge._trajectory_scheduler
    scheduler.set_execution_mode(mode, now=0)
    if origin == 'commit_one':
        scheduler.begin(now=0)
    if origin == 'initial_pose':
        for command, value in [('swing_angle', 0), ('boom_angle', -50),
                               ('arm_angle', 100), ('bucket_angle', 60)]:
            scheduler.update_http_target(command, value, now=0)
    else:
        scheduler.update_http_target('boom_down', 1, now=0)
        scheduler.update_http_target('arm_extend', 1, now=0)
    if origin == 'commit_one':
        scheduler.commit(True, now=0)
    elif origin == 'mode_change':
        scheduler.set_execution_mode('serial' if mode == 'parallel' else 'parallel', now=0)

    goals = _send_all(bridge)

    assert len(goals) == (1 if origin == 'commit_one' else
                         4 if origin == 'initial_pose' and mode == 'serial' else
                         2 if mode == 'serial' else 1)
    for goal in goals:
        assert (goal.goal_time_tolerance.sec, goal.goal_time_tolerance.nanosec) == (0, 0)


class _BeforeRosInterfaces(Exception):
    pass


def _initialize_without_ros(bridge, overrides):
    parameters = {}

    def declare(name, default):
        parameters[name] = overrides.get(name, default)

    # 検証より後のCommandStateで打ち切り、publisherやHTTPサーバーを作らない。
    with patch('scratch_hci_bridge.node.Node.__init__', return_value=None), \
         patch.object(ScratchHciBridge, 'declare_parameter', side_effect=declare), \
         patch.object(ScratchHciBridge, 'get_parameter',
                      side_effect=lambda name: SimpleNamespace(value=parameters[name])), \
         patch('scratch_hci_bridge.node.CommandState', side_effect=_BeforeRosInterfaces):
        bridge.__init__()


@pytest.mark.parametrize('seconds', [-0.1, float('nan'), float('inf'), float('-inf'),
                                    MAX_ROS_DURATION_SEC + 1])
def test_invalid_goal_time_tolerance_is_rejected(seconds):
    bridge = ScratchHciBridge.__new__(ScratchHciBridge)
    with pytest.raises(ValueError, match='intermediate_goal_time_tolerance_sec'):
        _initialize_without_ros(bridge, {'intermediate_goal_time_tolerance_sec': seconds})


@pytest.mark.parametrize('overrides, expected', [({}, 0.5),
    ({'intermediate_goal_time_tolerance_sec': 0.0}, 0.0),
    ({'intermediate_goal_time_tolerance_sec': 1.25}, 1.25),
    ({'intermediate_goal_time_tolerance_sec': MAX_ROS_DURATION_SEC}, MAX_ROS_DURATION_SEC)])
def test_valid_goal_time_tolerance_and_node_default(overrides, expected):
    bridge = ScratchHciBridge.__new__(ScratchHciBridge)
    with pytest.raises(_BeforeRosInterfaces):
        _initialize_without_ros(bridge, overrides)
    assert bridge._intermediate_goal_time_tolerance_sec == expected

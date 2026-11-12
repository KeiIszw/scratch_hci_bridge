"""接地ガードの保持・unknown拒否・Action競合・停止とHTTP reporterを検証する。"""
import math
import threading
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest
from action_msgs.msg import GoalStatus
from scratch_hci_bridge.command_state import CommandState
from scratch_hci_bridge.http_server import create_http_server
from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler
from test_http_server import _get
from test_sync_safety import scheduler, NAMES
from bridge_test_helpers import Harness


def start(s, now=0):
    s.update_ground_contact(0, now=now)
    assert s.start_ground_guard(20, speed_limit_deg_s=10, now=now)


def test_unknown_start_and_feedback_rejected_without_plan():
    s = scheduler()
    assert not s.start_ground_guard(10, now=0)
    assert s.guard_status() == 'failed' and not s.has_work()
    s.update_ground_contact(0, now=0)
    assert not s.start_ground_guard(10, now=1)
    s = TrajectoryScheduler()
    s.update_ground_contact(0, now=0)
    assert not s.start_ground_guard(10, now=0)


def test_hold_uses_measured_position_discards_remaining_boom_only():
    s = scheduler()
    start(s)
    assert s.update_http_target('arm_extend', 10, base_duration_sec=3, now=0)
    other = s._timeline.axes['arm'][:]
    first = s.pending_snapshot(now=0)
    s.mark_submitted(first)
    s.update_from_joint_state(NAMES, [0, math.radians(-47), 2, 1], now=.2)
    assert not s.update_http_target('boom_down', 5, now=.2)
    s.update_ground_contact(1, now=.2)
    assert s.check_ground_guard(.2) is None
    assert s.guard_status() == 'contact'
    assert s._timeline.axes['arm'] == other
    replacement = s.pending_snapshot(now=.2, replacing=first)
    assert replacement is not None
    assert all(p.positions[1] == pytest.approx(math.radians(-47)) for p in replacement.track_points)
    assert all(p.velocities[1] == 0 for p in replacement.track_points)
    # 置換前の成功結果は保持目標と後続計画を消せない。
    s.mark_completed(first, successful=True, now=.2)
    assert s.has_work() and s.guard_status() == 'contact'
    assert not any(c.http_name == 'boom_down' for c in replacement.commands)


def test_barrier_queued_guard_starts_at_interval_boundary():
    s = scheduler()
    assert s.update_http_target('arm_extend', 10, base_duration_sec=1, now=0)
    s.barrier(now=0)
    start(s, now=.1)
    segment = s._guard_segment
    assert segment.start == 1
    s.update_ground_contact(1, now=.2)
    assert s.check_ground_guard(.2) is None and s.guard_status() == 'running'
    s.update_ground_contact(1, now=1)
    s.check_ground_guard(1)
    assert s.guard_status() == 'contact'


def test_reached_only_on_latest_successful_terminal_result():
    s = scheduler()
    start(s)
    goal = s.pending_snapshot(now=0)
    s.mark_submitted(goal)
    s.update_ground_contact(0, now=2.1)
    s.check_ground_guard(2.1)
    assert s.guard_status() == 'running'
    s.mark_completed(goal, successful=True, now=2.1)
    assert s.guard_status() == 'reached' and s.motion_status() == 0


@pytest.mark.parametrize('value,now', [(-1, .1), (99, .1), (0, .501)])
def test_unknown_or_stale_requests_existing_safety_stop(value, now):
    s = scheduler()
    start(s)
    s.update_ground_contact(value, now=0)
    assert s.check_ground_guard(now) == 'ground_unknown'
    assert s.guard_status() == 'failed'
    s.stop('ground_unknown')
    assert s.motion_status() == -1 and not s.has_work()


@pytest.mark.parametrize('reason', ['stopped', 'lease_expired', 'stale', 'goal_rejected', 'tolerance_violated', 'result_timeout'])
def test_existing_stops_fail_running_guard_and_reset_releases(reason):
    s = scheduler()
    start(s)
    s.stop(reason)
    assert s.guard_status() == 'failed' and not s.has_work()
    s.stop(reset=True)
    assert s.guard_status() == 'reached'


def test_node_contact_during_acceptance_and_replacement():
    h = Harness()
    s = h.scheduler
    start(s)
    h.tick(0)
    # 受理応答待ちに接地しても、保持のために応答ゲートを開けてしまわない。
    s.update_from_joint_state(NAMES, [0, math.radians(-49), 2, 1], now=.1)
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=.1):
        h.bridge._ground_contact_callback(NS(data=1))
    h.tick(.1)
    assert len(h.sent) == 1
    h.accept(0, now=.1)
    h.tick(.12)
    assert len(h.sent) == 2
    h.accept(1, now=.12)
    assert h.sent[1][0].trajectory.points[-1].positions[1] == pytest.approx(math.radians(-49))
    h.result(1, now=.2)
    assert s.guard_status() == 'contact' and s.motion_status() == 0


def test_node_unknown_cancels_and_keeps_gate_until_result():
    h = Harness()
    start(h.scheduler)
    h.tick(0); handle = h.accept(0)
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=.1):
        h.bridge._ground_contact_callback(NS(data=-1))
    assert h.scheduler.guard_status() == 'failed'
    assert h.scheduler.needs_cancel()
    assert handle.cancel_calls == 1
    h.tick(.2)
    assert len(h.sent) == 1
    h.result(0, status=GoalStatus.STATUS_CANCELED, now=.2)
    assert h.scheduler.motion_status() == -1


def test_http_reporters_freshness_format_and_guard_command():
    h = Harness()
    bridge = h.bridge
    bridge._ground_timeout_sec = .5
    bridge._ground_clearance, bridge._ground_clearance_at = .12349, 0
    s = h.scheduler
    s.update_ground_contact(0, now=0)
    server = create_http_server('127.0.0.1', 0, CommandState(), bridge._command_callback,
        s.motion_status, s.motion_error, ground_reporter_callback=bridge._ground_reporter)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    port = server.server_address[1]
    try:
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=0):
            assert _get(f'http://127.0.0.1:{port}/?type=reporter&cmd=ground_clearance&val=0')[:2] == (200, '0.123')
            assert _get(f'http://127.0.0.1:{port}/?type=reporter&cmd=ground_contact&val=0')[:2] == (200, '0')
            assert _get(f'http://127.0.0.1:{port}/?type=command&cmd=boom_down_until_contact&val=10')[:2] == (200, 'OK')
            assert _get(f'http://127.0.0.1:{port}/?type=reporter&cmd=guard_status&val=0')[:2] == (200, 'running')
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=1):
            assert _get(f'http://127.0.0.1:{port}/?type=reporter&cmd=ground_clearance&val=0')[:2] == (200, 'unknown')
            assert _get(f'http://127.0.0.1:{port}/?type=reporter&cmd=ground_contact&val=0')[:2] == (200, 'unknown')
    finally:
        server.shutdown(); server.server_close(); thread.join()


def test_contact_masks_old_boom_during_hold_goal_response_delay():
    from test_catch_up_axis_mask import harness as masked_harness, mask
    h = masked_harness()
    start(h.scheduler)
    h.tick(0); h.accept(0)
    assert mask(h) == 2
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=.1):
        h.bridge._ground_contact_callback(NS(data=1))
    assert mask(h) == 0
    h.tick(.1)
    assert len(h.sent) == 2 and mask(h) == 0
    # 新しい保持目標を受理した後だけ、その保持位置への追従を許可できる。
    h.scheduler.update_from_joint_state(NAMES, [0, math.radians(-48), 2, 1], now=.2)
    h.accept(1, now=.2)
    assert mask(h) == 2
    assert h.sent[1][0].trajectory.points[-1].positions[1] == pytest.approx(math.radians(-50))

"""抵抗停止を実測保持へ変換し、異常停止と相対基準をダミーActionで検証する。"""
import math
import threading
from unittest.mock import patch

import pytest

from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler
from scratch_hci_bridge.http_server import create_http_server
from scratch_hci_bridge.command_state import CommandState
from bridge_test_helpers import Harness
from test_sync_safety import NAMES, scheduler
from test_track_timeline import add
from test_http_server import _get


def observe(s, start, end, positions=None):
    """50Hzの新規実測で連続性を保ち、timerの再読込による停滞を作らない。"""
    positions = positions or [0, math.radians(-50), 2, 1]
    for i in range(round((end-start)/.02)+1):
        s.update_from_joint_state(NAMES, positions, now=round(start+i*.02, 8))


def submitted(s, command='bucket_dump', delta=30, speed=15):
    assert add(s, command=command, delta=delta, speed=speed)
    goal = s.pending_snapshot(now=0)
    assert s.mark_submitted(goal)
    return goal


@pytest.mark.parametrize('axis,cmd', [('boom','boom_down'),('arm','arm_extend'),('bucket','bucket_dump')])
def test_resistance_holds_drops_future_and_next_relative_uses_stopped_position(axis, cmd):
    s = scheduler()
    goal = submitted(s, cmd)
    assert add(s, command=cmd, delta=5, now=.1)
    held = s._feedback_positions_rad[axis]
    observe(s, .02, 3.5)
    assert s.stall_joints() == axis
    assert s.motion_outcome() == 'resistance'
    assert s.motion_status() == 1 and s.motion_error() == ''
    assert s.joint_queue([a[:-6] for a in NAMES].index(axis), now=3.5) == 0
    assert all(seg.origin == seg.target == held for seg in s._timeline.axes[axis])
    replacement = s.pending_snapshot(now=3.5, replacing=goal)
    assert s.mark_submitted(replacement, replacing=goal)
    assert s.mark_completed(replacement, successful=True, now=3.51)
    assert s.motion_status() == 0
    assert add(s, command=cmd, delta=3, now=3.6)
    seg = s._timeline.axes[axis][-1]
    direction = -1 if cmd in ('arm_extend','bucket_dump') else 1
    assert seg.origin == pytest.approx(held)
    assert seg.target == pytest.approx(held+direction*math.radians(3))
    assert s.motion_status() == 1


def test_gate_treats_resistance_as_arrived_and_preserves_other_axis():
    s = scheduler()
    goal = submitted(s)
    s.barrier(now=.1, arrival=True)
    assert add(s, command='boom_down', now=.1)
    observe(s, .02, 3.5)
    s.check_deadlines(now=3.5)
    assert s.motion_status() != -1 and not s._arrival_gate
    assert s._timeline.axes['boom'][-1].start == 3.5
    replacement = s.pending_snapshot(now=3.5, replacing=goal)
    assert all(p.positions[3] == 1 for p in replacement.track_points)


@pytest.mark.parametrize('delta,speed', [(7.9, .5), (8.49, 30)])
def test_start_deadband_and_slow_start_wait_do_not_count_as_stall(delta, speed):
    s = scheduler()
    submitted(s, delta=delta, speed=speed)
    observe(s, .02, 10)
    assert s.stall_joints() == 'none'
    assert not any(s._stall_samples.values())
    assert s.motion_status() == 1


def test_moving_wrong_direction_and_sample_gap_reset_stall_window():
    s = scheduler()
    goal = submitted(s)
    observe(s, .02, 1.8)
    assert s.stall_joints() == 'none'
    # 0.2秒超の欠測を挟んだ静止観測は継続1.5秒にならない。
    observe(s, 2.5, 3.6)
    assert s.stall_joints() == 'none'
    observe(s, 3.62, 4.3)
    assert s.stall_joints() == 'bucket'
    s = scheduler()
    submitted(s)
    for i in range(1, 201):
        s.update_from_joint_state(NAMES, [0, math.radians(-50), 2, 1-math.radians(i*.02)], now=i*.02)
    assert s.stall_joints() == 'none'  # 1度/秒で動いている。
    s = scheduler()
    submitted(s)
    observe(s, .02, 4, [0, math.radians(-50), 2, 0])
    assert s.stall_joints() == 'none'  # 指令より実測が先へ進んだ誤差は抵抗ではない。


def test_swing_stall_stops_all_motion_as_obstacle_and_retains_position():
    s = scheduler()
    submitted(s, 'swing_left')
    observe(s, .02, 3.5)
    assert s.motion_status() == -1 and s.motion_error() == 'swing_load_stall'
    assert s.stall_joints() == 'swing' and s.stall_position(0) == '0.000'
    assert s.motion_outcome() == 'abnormal'
    assert s.needs_cancel()


def test_action_success_alone_waits_for_bridge_arrival_and_does_not_resend():
    s = scheduler()
    goal = submitted(s, delta=10)
    s.mark_completed(goal, successful=True, now=1)
    assert s.motion_status() == 1
    assert s.pending_snapshot(now=1.1) is None
    observe(s, 1.12, 1.5, [0, math.radians(-50), 2, 1-math.radians(6.71)])
    s.check_deadlines(now=1.5)
    assert s.motion_status() == 0 and s.motion_outcome() == 'arrived'
    assert s.stall_joints() == 'none'  # 3.29度残差はbucketの到着許容内。


@pytest.mark.parametrize('state', ['running','stalled','action_success'])
def test_overall_timeout_survives_stall_and_action_success(state):
    s = scheduler(); s._motion_timeout = 3.6
    goal = submitted(s)
    if state == 'stalled':
        observe(s, .02, 3.5)
        assert s.stall_joints() == 'bucket'
    elif state == 'action_success':
        s.mark_completed(goal, successful=True, now=2)
    s.check_deadlines(now=3.61)
    assert s.motion_status() == -1 and s.motion_error() == 'motion_timeout'


@pytest.mark.parametrize('reason', ['goal_rejected','lease_expired','result_timeout','tolerance_violated'])
def test_abnormal_failures_are_not_recovered_as_resistance(reason):
    s = scheduler(); goal = submitted(s)
    observe(s, .02, 3.5)
    replacement = s.pending_snapshot(now=3.5, replacing=goal)
    s.mark_submitted(replacement, replacing=goal)
    s.mark_completed(replacement, successful=False, reason=reason, now=3.51)
    assert s.motion_status() == -1 and s.motion_error() == reason
    assert s.motion_outcome() == 'abnormal'


def test_nonfinite_immediately_invalidates_angle_and_stale_after_success_fails():
    s = scheduler(); submitted(s)
    s.update_from_joint_state(['bucket_joint'], [float('nan')], now=.1)
    assert s.joint_angle(3, now=.1) == 'unknown'
    assert s.motion_status() == -1 and s.motion_error() == 'nonfinite_feedback'
    s = scheduler(); s._feedback_timeout_sec = .5
    goal = submitted(s)
    s.mark_completed(goal, successful=True, now=.1)
    s.check_deadlines(now=.51)
    assert s.motion_status() == -1 and s.motion_error() == 'stale'


def test_sensor_http_reports_real_angles_stall_positions_and_unknown():
    s = scheduler(); s._feedback_timeout_sec = .5
    submitted(s)
    observe(s, .02, 3.5)
    def sensor(cmd, index):
        return {'joint_angle': lambda: s.joint_angle(index, now=3.5),
                'stall_joints': s.stall_joints, 'stall_position': lambda: s.stall_position(index),
                'motion_outcome': s.motion_outcome}[cmd]()
    server = create_http_server('127.0.0.1', 0, CommandState(), sensor_reporter_callback=sensor)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    base = f'http://127.0.0.1:{server.server_address[1]}/?type=reporter&cmd='
    try:
        for i, expected in enumerate(('0.000','-50.000','114.592','57.296')):
            assert _get(base+f'joint_angle&val={i}')[:2] == (200, expected)
        assert _get(base+'stall_joints&val=0')[:2] == (200, 'bucket')
        assert _get(base+'stall_position&val=3')[:2] == (200, '57.296')
        assert _get(base+'stall_position&val=1')[:2] == (200, 'unknown')
        assert _get(base+'motion_outcome&val=0')[:2] == (200, 'resistance')
        for cmd in ('joint_angle&val=4','joint_angle&val=-1','joint_angle&val=nan','stall_joints&val=1'):
            assert _get(base+cmd)[0] == 400
    finally:
        server.shutdown();server.server_close();thread.join()
    assert s.joint_angle(3, now=4.01) == 'unknown'
    s.stop(reset=True)
    assert s.stall_joints() == 'none' and s.stall_position(3) == 'unknown'


@pytest.mark.parametrize('name', ['stall_motion_deg','motion_timeout_sec']+
                         [a+'_load_stall_window_sec' for a in ('swing','boom','arm','bucket')]+
                         [a+'_load_stall_lead_deg' for a in ('swing','boom','arm','bucket')])
@pytest.mark.parametrize('bad', [0,-1,float('nan'),float('inf')])
def test_invalid_load_stall_parameters_rejected(name, bad):
    with pytest.raises(ValueError): TrajectoryScheduler(**{name:bad})


def test_node_holds_stalled_axis_after_hold_acceptance_and_dispatches_next():
    """実ノードのcallback・マスク・置換・Action成功・後続送信を一続きで確認する。"""
    from types import SimpleNamespace
    from test_catch_up_axis_mask import harness, mask
    h = harness(); s = h.scheduler
    assert add(s, command='bucket_dump', delta=30, speed=15)
    h.tick(0); h.accept(0)
    assert mask(h) == 8
    for i in range(1, 176):
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=i*.02):
            h.bridge._joint_state_callback(SimpleNamespace(name=NAMES, position=[0.,math.radians(-50),2.,1.]))
    assert s.stall_joints() == 'bucket' and mask(h) == 0
    h.tick(3.5)
    assert len(h.sent) == 2
    assert all(p.positions[3] == 1 and p.velocities[3] == 0 for p in h.sent[1][0].trajectory.points)
    assert mask(h) == 0
    h.accept(1, now=3.5)
    assert mask(h) == 8
    h.result(1, now=3.51)
    assert mask(h) == 8
    assert s.motion_status() == 0 and s.motion_error() == ''
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=3.6):
        assert h.bridge._command_callback('bucket_dump', 3)
    h.tick(3.6)
    assert h.sent[2][0].trajectory.points[-1].positions[3] == pytest.approx(1-math.radians(3))


def test_node_nonfinite_and_overall_deadline_stop_drive_and_cancel():
    from types import SimpleNamespace
    for cause in ('nonfinite', 'timeout'):
        h = Harness(); s = h.scheduler
        assert add(s)
        h.tick(0); handle = h.accept(0)
        h.bridge._state.apply_command('left_crawler_speed', 1)
        if cause == 'nonfinite':
            with patch('scratch_hci_bridge.node.time.monotonic', return_value=.1):
                h.bridge._joint_state_callback(SimpleNamespace(name=['boom_joint'], position=[float('inf')]))
        else:
            s._motion_timeout = .1
            h.tick(.11)
        assert s.motion_status() == -1
        assert h.bridge._state.next_frame().linear_velocity == 0
        assert handle.cancel_calls == 1
        assert not s.update_http_target('boom_down', 1, now=.2)


def test_http_action_tolerances_match_arrival_and_keep_large_finite_path_limit():
    h = Harness()
    for cmd in ('swing_left','boom_down','arm_extend','bucket_dump'):
        assert add(h.scheduler, command=cmd)
    h.tick(0)
    goal = h.sent[0][0]
    assert [math.degrees(t.position) for t in goal.goal_tolerance] == pytest.approx([1.5,2.5,5.5,5.5])
    assert all(math.degrees(t.position) == pytest.approx(120) for t in goal.path_tolerance)
    assert goal.goal_time_tolerance.sec == 30


def test_failed_http_rejects_state_mutations_until_reset():
    """failed中は走行・直結角度もCommandStateを変更せず、reset後だけ通す。"""
    s = scheduler(); state = CommandState()
    s.stop('goal_rejected')
    def command(cmd, val):
        if cmd == 'reset': s.stop(reset=True)
    server = create_http_server('127.0.0.1', 0, state, command_callback=command, status_callback=s.motion_status)
    thread = threading.Thread(target=server.serve_forever, daemon=True);thread.start()
    base = f'http://127.0.0.1:{server.server_address[1]}/?type=command&cmd='
    try:
        before = state.snapshot()
        for command in ('left_crawler_speed','blade_angle','bucket_dump'):
            assert _get(base+command+'&val=1')[0] == 400
            assert state.snapshot() == before
        assert _get(base+'reset&val=0')[:2] == (200, 'OK')
        assert _get(base+'left_crawler_speed&val=1')[:2] == (200, 'OK')
        assert state.snapshot().left_crawler_speed == 1
    finally:
        server.shutdown();server.server_close();thread.join()


def test_normal_completion_without_gate_preserves_equivalent_swing_hold():
    """ゲートなしの通常完了でも、実測370度から次の10度を一回転逆行させない。"""
    s = scheduler()
    goal = submitted(s, 'swing_left', delta=10, speed=10)
    observe(s, 1.02, 1.2, [math.radians(370),math.radians(-50),2.,1.])
    assert s.mark_completed(goal, successful=True, now=1.2)
    assert s.motion_status() == 0
    assert add(s, command='swing_left', delta=10, speed=10, now=1.3)
    seg = s._timeline.axes['swing'][-1]
    assert math.degrees(seg.origin) == pytest.approx(10)
    assert math.degrees(seg.target) == pytest.approx(20)


@pytest.mark.parametrize('sink_deg_s', [0., .2])
def test_window_detects_vibration_and_slow_sink_without_speed_timer_resets(sink_deg_s):
    """0.16度の揺れや緩い沈み込みでも、幅0.5度未満なら窓で打ち切る。"""
    s = scheduler()
    submitted(s, 'boom_down', delta=30)
    detected = None
    for i in range(1, 201):
        t = i*.02
        boom = -50+sink_deg_s*t+.08*math.sin(i*1.7)
        s.update_from_joint_state(NAMES, [0, math.radians(boom), 2, 1], now=t)
        if s.stall_joints() == 'boom':
            detected = t
            break
    assert detected is not None and 1.5 < detected < 2.1
    assert s._resistance_holds['boom'] == s._feedback_positions_rad['boom']


def test_window_does_not_stop_sustained_motion_above_range_threshold():
    """0.4度/秒は1.5秒で0.6度進むため、低速でも抵抗停止にしない。"""
    s = scheduler()
    submitted(s, 'boom_down')
    for i in range(1, 251):
        s.update_from_joint_state(NAMES, [0, math.radians(-50+.4*i*.02), 2, 1], now=i*.02)
    assert s.stall_joints() == 'none'


@pytest.mark.parametrize('arrival_gate', [False, True])
def test_fixed_hold_survives_drift_completion_other_axis_and_explicit_release(arrival_gate):
    """ゲートや完了で停止角を更新せず、他軸goalにも固定保持・許容を引き継ぐ。"""
    from test_catch_up_axis_mask import harness, mask
    h = harness(); s = h.scheduler
    assert add(s, command='boom_down', delta=30, speed=15)
    h.tick(0); h.accept(0)
    if arrival_gate:
        s.barrier(now=.1, arrival=True)
    observe(s, .02, 3.5)
    held = s._resistance_holds['boom']
    from test_node_command_callback import _Publisher
    h.bridge._unity_position_publishers = {a: _Publisher() for a in ('swing','boom','arm','bucket')}
    h.bridge._unity_position_signs = {a: 1 for a in ('swing','boom','arm','bucket')}
    with patch('scratch_hci_bridge.node.time.monotonic', return_value=3.5):
        h.bridge._publish_unity_position_target(h.bridge._active_goal_scheduled)
    assert not h.bridge._unity_position_publishers['boom'].values
    # 検出後の外力による変位を、新しい保持角に採用しない。
    observe(s, 3.52, 3.7, [0, held-math.radians(1.6), 2, 1])
    h.tick(3.7); h.accept(1, now=3.7); h.result(1, now=3.71)
    assert s.motion_status() == 0 and mask(h) == 2
    assert h.bridge._unity_position_publishers['boom'].values == [held]
    assert add(s, command='arm_extend', delta=10, now=3.8)
    h.tick(3.8)
    goal = h.sent[2][0]
    assert all(p.positions[1] == held and p.velocities[1] == 0 for p in goal.trajectory.points)
    assert goal.goal_tolerance[1].position > 0
    h.accept(2, now=3.8)
    assert mask(h) & 2
    # 他軸の置換でも固定値を維持する。
    assert add(s, command='bucket_dump', delta=10, now=3.9)
    h.tick(3.9)
    assert all(p.positions[1] == held for p in h.sent[3][0].trajectory.points)
    h.accept(3, now=3.9)
    assert add(s, command='boom_up', delta=3, now=4.)
    assert 'boom' not in s.resistance_held_axes()
    segment = s._timeline.axes['boom'][-1]
    assert segment.origin == held
    assert segment.target == pytest.approx(held-math.radians(3))


@pytest.mark.parametrize('state', ['guard', 'resistance'])
def test_stop_discards_guard_or_active_hold_and_accepts_next_command_without_reset(state):
    """通常stopは保持・計画を破棄し、履歴は残してresetなしで再開する。"""
    s = scheduler()
    if state == 'guard':
        s.update_ground_contact(0, now=0)
        assert s.start_ground_guard(30, 15, now=0)
        goal = s.pending_snapshot(now=0); s.mark_submitted(goal)
    else:
        goal = submitted(s, 'boom_down')
        observe(s, .02, 3.5)
        assert s.resistance_held_axes() == {'boom'}
    s.stop()
    assert s.motion_status() == 2 and s.motion_outcome() == 'stopped'
    assert not s.resistance_held_axes() and s._guard_hold_position is None
    assert s.guard_status() == ('failed' if state == 'guard' else 'reached')
    assert s.stall_joints() == ('boom' if state == 'resistance' else 'none')
    assert add(s, command='boom_up', delta=3, now=3.6)
    assert s.motion_error() == '' and s.motion_status() == 1
    assert s.pending_snapshot(now=3.6) is None  # cancel終端までは送らない。
    assert s.mark_completed(goal, successful=False, now=3.7)
    assert s.pending_snapshot(now=3.7) is not None


@pytest.mark.parametrize('reason', ['lease', 'stale', 'nonfinite'])
def test_idle_active_hold_still_obeys_lease_and_feedback_deadlines(reason):
    """idle表示後も能動出力を続けるため、リースと角度鮮度の監視を残す。"""
    from test_catch_up_axis_mask import harness, mask
    h = harness();s = h.scheduler
    assert add(s, command='boom_down', delta=30)
    h.tick(0); h.accept(0)
    observe(s, .02, 3.5)
    h.tick(3.5);h.accept(1, now=3.5);h.result(1, now=3.51)
    assert s.motion_status() == 0 and mask(h) == 2
    if reason == 'lease':
        h.bridge._lease_enabled = True
        h.bridge._last_http_at = 0
    elif reason == 'stale':
        s._feedback_timeout_sec = .5
    else:
        from types import SimpleNamespace
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=3.6):
            h.bridge._joint_state_callback(SimpleNamespace(name=['boom_joint'], position=[float('nan')]))
        assert s.motion_error() == 'nonfinite_feedback'
    h.tick(4.1)
    assert s.motion_status() == -1 and mask(h) == 0

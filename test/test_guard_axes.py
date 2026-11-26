"""ブーム以外の関節の接地動作、距離指定の接地動作、1関節だけの停止と停止確認を検証する。

上限角を取らない接地動作（HTTP指令）が可動域の端を終点にすること、関節ごとの地面へ近づく向きで
保持・行き過ぎ追従が働くこと、関節を止めても他の関節の計画は続くことを確かめる。
"""
import math
import threading
from unittest.mock import patch

import pytest

from scratch_hci_bridge.command_state import CommandState, JOINT_LIMITS_DEG
from scratch_hci_bridge.http_server import create_http_server
from test_http_server import _get
from test_sync_safety import scheduler, NAMES
from bridge_test_helpers import Harness


def joints(boom=-50, arm=2.0, bucket=1.0):
    """scheduler()の初期姿勢（ブームは度、アーム・バケットはrad）から一部だけ変えた実測。"""
    return [0.0, math.radians(boom), arm, bucket]


def test_contact_guard_without_limit_targets_joint_limit_in_motion_direction():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard('arm_extend', speed_limit_deg_s=10, now=0)
    assert s.guard_axis() == 'arm' and s.guard_status() == 'running'
    # アームを伸ばす向きは角度が減る向き。終点は可動域の下端。
    assert s._guard_segment.target == pytest.approx(math.radians(JOINT_LIMITS_DEG['arm'][0]))
    s.update_ground_contact(0, now=0)
    s2 = scheduler()
    s2.update_ground_contact(0, now=0)
    assert s2.start_ground_guard('boom_down', speed_limit_deg_s=10, now=0)
    assert s2._guard_segment.target == pytest.approx(math.radians(JOINT_LIMITS_DEG['boom'][1]))


@pytest.mark.parametrize('motion', ['boom_up', 'swing_left', 'unknown'])
def test_guard_rejects_motions_that_do_not_approach_ground(motion):
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert not s.start_ground_guard(motion, now=0)
    assert not s.has_work()


def test_arm_contact_holds_measured_and_follows_overshoot_only_toward_ground():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard('arm_extend', speed_limit_deg_s=10, now=0)
    first = s.pending_snapshot(now=0)
    s.mark_submitted(first)
    s.update_from_joint_state(NAMES, joints(arm=1.9), now=.2)
    s.update_ground_contact(1, now=.2)
    assert s.check_ground_guard(.2) is None
    assert s.guard_status() == 'contact'
    assert s._guard_hold_position == pytest.approx(1.9)
    # 地面へ近づく向き（角度が減る向き）の惰性だけを保持目標へ追従させる。
    s.update_from_joint_state(NAMES, joints(arm=1.85), now=.25)
    assert s._guard_hold_position == pytest.approx(1.85)
    s.update_from_joint_state(NAMES, joints(arm=1.95), now=.3)
    assert s._guard_hold_position == pytest.approx(1.85)
    replacement = s.pending_snapshot(now=.3, replacing=first)
    # 押し戻された実測(1.95)ではなく、最も地面側の保持角(1.85)を送り続ける。
    assert all(p.positions[2] == pytest.approx(1.85) for p in replacement.track_points)
    # ブームは接地動作の関節ではないので、ブームの指令は受け付ける。
    assert s.update_http_target('boom_down', 5, now=.3)


def test_old_goal_masking_uses_guard_axis_direction():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard('bucket_dump', speed_limit_deg_s=10, now=0)
    first = s.pending_snapshot(now=0)
    s.mark_submitted(first)
    s.update_from_joint_state(NAMES, joints(bucket=0.9), now=.2)
    s.update_ground_contact(1, now=.2)
    s.check_ground_guard(.2)
    # 古いgoalは保持revisionより前なので、バケットの出力を許可しない。
    assert s.guard_blocks_old_target(first)
    hold = s.pending_snapshot(now=.2, replacing=first)
    assert not s.guard_blocks_old_target(hold)


def test_clearance_guard_slows_early_and_finishes_with_distance_status():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard('boom_down', speed_limit_deg_s=10, now=0, clearance_m=.1)
    # 目標距離0.1m＋減速距離0.2mの手前で減速する。
    s.update_ground_clearance(.29, now=.1)
    s.update_ground_contact(0, now=.1)
    s.check_ground_guard(.1)
    assert s._guard_slowed and s.guard_status() == 'running'
    s.update_ground_clearance(.1, now=.2)
    s.update_ground_contact(0, now=.2)
    s.check_ground_guard(.2)
    assert s.guard_status() == 'distance'
    assert s._guard_hold_position == s._feedback_positions_rad['boom']


@pytest.mark.parametrize('value', [-.1, float('nan'), float('inf')])
def test_clearance_guard_rejects_invalid_distance(value):
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert not s.start_ground_guard('boom_down', now=0, clearance_m=value)


def test_contact_before_target_distance_finishes_as_contact():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard('boom_down', speed_limit_deg_s=10, now=0, clearance_m=.05)
    s.update_ground_contact(1, now=.1)
    s.check_ground_guard(.1)
    assert s.guard_status() == 'contact'


def test_failed_guard_start_keeps_previous_axis_hold():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard('boom_down', speed_limit_deg_s=10, now=0, travel_deg=5)
    s.update_ground_contact(1, now=.1)
    s.check_ground_guard(.1)
    hold = s._guard_hold_position
    # アームの送信枠が埋まっていると接地動作は受け付けられない。前のブームの保持は失わない。
    assert s.update_http_target('arm_extend', 5, now=.1)
    s.update_ground_contact(0, now=.1)
    assert not s.start_ground_guard('arm_extend', now=.1)
    assert s.guard_axis() == 'boom' and s._guard_hold_position == hold


def test_stop_axis_holds_only_that_axis_and_keeps_others():
    s = scheduler()
    assert s.update_http_target('boom_down', 20, speed_limit_deg_s=10, now=0)
    assert s.update_http_target('arm_extend', 20, speed_limit_deg_s=10, now=0)
    arm = s._timeline.axes['arm'][:]
    s.update_from_joint_state(NAMES, joints(boom=-45), now=.5)
    assert s.stop_axis('boom', now=.5)
    segment, = s._timeline.axes['boom']
    assert segment.origin == segment.target == pytest.approx(math.radians(-45))
    assert s._timeline.axes['arm'] == arm
    assert s.motion_status() == 1 and s.guard_status() == 'reached'
    # 止めた位置を次の相対指令の基準にする。
    assert s.update_http_target('boom_down', 5, speed_limit_deg_s=10, now=.6)
    assert s._timeline.axes['boom'][-1].target == pytest.approx(math.radians(-40))


def test_stop_axis_during_guard_finishes_as_halted():
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard('boom_down', speed_limit_deg_s=10, now=0)
    s.update_from_joint_state(NAMES, joints(boom=-40), now=1)
    assert s.stop_axis('boom', now=1)
    assert s.guard_status() == 'halted'
    assert s._guard_hold_position == pytest.approx(math.radians(-40))


def test_stop_axis_rejects_stale_feedback():
    s = scheduler()
    s._feedback_timeout_sec = .5
    assert s.update_http_target('boom_down', 20, now=0)
    assert not s.stop_axis('boom', now=1)


def test_axis_moving_reports_plan_and_feedback_speed():
    s = scheduler()
    assert s.axis_moving(1, now=0) == '1'  # 実測速度の窓が揃うまでは静止とみなさない
    for i in range(5):
        s.update_from_joint_state(NAMES, joints(), now=i*.05)
    assert s.axis_moving(1, now=.2) == '0'
    assert s.update_http_target('boom_down', 5, speed_limit_deg_s=10, now=.2)
    assert s.axis_moving(1, now=.3) == '1'
    assert s.axis_moving(2, now=.3) == '0'
    s._feedback_timeout_sec = .1
    assert s.axis_moving(1, now=1) == 'unknown'


def test_http_accepts_new_guard_and_stop_commands():
    h = Harness()
    bridge, s = h.bridge, h.scheduler
    s.update_ground_contact(0, now=0)
    server = create_http_server('127.0.0.1', 0, CommandState(), bridge._command_callback,
        s.motion_status, s.motion_error, ground_reporter_callback=bridge._ground_reporter,
        sensor_reporter_callback=bridge._sensor_reporter)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    port = server.server_address[1]
    base = f'http://127.0.0.1:{port}/?type='
    try:
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=0), \
                patch('scratch_hci_bridge.trajectory_scheduler.time.monotonic', return_value=0):
            assert _get(base+'command&cmd=boom_down_until_clearance&val=9')[0] == 400
            assert _get(base+'command&cmd=bucket_scoop_until_contact&val=0')[:2] == (200, 'OK')
            assert s.guard_axis() == 'bucket'
            assert _get(base+'reporter&cmd=guard_status&val=0')[:2] == (200, 'running')
            assert _get(base+'command&cmd=bucket_stop&val=0')[:2] == (200, 'OK')
            assert _get(base+'reporter&cmd=guard_status&val=0')[:2] == (200, 'halted')
            assert _get(base+'reporter&cmd=axis_moving&val=3')[0] == 200
            assert _get(base+'reporter&cmd=axis_moving&val=4')[0] == 400
    finally:
        server.shutdown(); server.server_close(); thread.join()


@pytest.mark.parametrize('motion,speed', [('boom_down', 1.0), ('arm_extend', 2.0), ('bucket_dump', 4.0)])
def test_slow_speed_depends_on_guard_axis(motion, speed):
    """回転半径の小さい関節ほど速い角速度で減速する（地面へ近づく速さを揃えるため）。"""
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard(motion, speed_limit_deg_s=10, now=0)
    axis = s.guard_axis()
    s.update_ground_clearance(.1, now=.1)
    s.update_ground_contact(0, now=.1)
    s.check_ground_guard(.1)
    segment = s._guard_segment
    remaining = math.degrees(abs(segment.target-segment.origin))
    # 3次補間の最大速度が平均の1.5倍になる分を見込んだ所要時間（README「減速」）。
    assert segment.end-segment.start == pytest.approx(1.5*remaining/speed, rel=1e-6)

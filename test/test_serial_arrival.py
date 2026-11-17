"""到着ゲートの保留・解除・安全停止を、擬似時計とダミーActionで検証する。"""
import math
import threading
from unittest.mock import patch

import pytest
from action_msgs.msg import GoalStatus

from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler
from scratch_hci_bridge.http_server import create_http_server
from bridge_test_helpers import Harness
from test_sync_safety import scheduler, NAMES
from test_track_timeline import add
from test_http_server import _get


def gated(s):
    """一秒のboom動作の後ろへ、到着待ちのarmを予約する。"""
    assert add(s)
    s.barrier(now=.1, arrival=True)
    assert add(s, command='arm_extend', now=.1)


def settle(s, axis, value, now):
    """静止した0.12秒の観測を与え、単一位置サンプルと停止を区別する。"""
    for t in (now-.12, now-.06, now):
        s.update_from_joint_state([axis+'_joint'], [value], now=t)


def arrive(s, now=1, boom=-40):
    settle(s, 'boom', math.radians(boom), now)


def test_gate_holds_past_planned_end_and_releases_when_position_and_speed_pass():
    """実測未到着では保留し、位置と停止速度を満たした判定で始める。"""
    s = scheduler()
    gated(s)
    p = s.pending_snapshot(now=0)
    assert p.target.duration_sec == 1
    assert p.target.positions_rad[2] == 2  # 後続目標はActionへ漏らさない。
    assert s.mark_submitted(p)
    for now in (1.01, 1.2):
        assert s.pending_snapshot(now=now, replacing=p) is None
        assert s.joint_queue(2, now=now) == 1
        assert s.track_active_axes(now=now) == set()
        assert 'arm' not in s.tracking_targets(now=now)
    arrive(s, now=1.3, boom=-41.4)
    q = s.pending_snapshot(now=1.3, replacing=p)
    assert q is not None
    assert s._timeline.axes['arm'][0].start == 1.3
    assert q.target.duration_sec == pytest.approx(1)
    assert s.joint_queue(2, now=1.3) == 0
    assert s.track_active_axes(now=1.3) == {'arm'}


def test_close_feedback_does_not_release_before_plan_end():
    """先に許容内へ入っても計画終了までは後続を動かさない。"""
    s = scheduler()
    gated(s)
    arrive(s, now=.2)
    s.check_deadlines(now=.2)
    assert s.joint_queue(2, now=.2) == 1
    s.mark_submitted(s.pending_snapshot(now=.2))
    s.check_deadlines(now=1.01)
    assert s._timeline.axes['arm'][0].start == 1.01


def test_all_preceding_axes_and_multiple_gates_preserve_order_and_speed():
    """全軸の到着確認と複数ゲート越しの受付順・速度保持を確認する。"""
    s = scheduler()
    assert add(s)
    assert add(s, command='bucket_dump')
    s.barrier(now=.1, arrival=True)
    assert add(s, command='arm_extend', speed=5, now=.1)
    s.barrier(now=.2, arrival=True)
    assert add(s, command='swing_left', now=.2)
    s.mark_submitted(s.pending_snapshot(now=.2))
    arrive(s, now=1.2)
    s.check_deadlines(now=1.2)
    assert not s._timeline.axes['arm']  # bucketも未到着。
    settle(s, 'bucket', 1-math.radians(10), 1.3)
    s.check_deadlines(now=1.3)
    arm = s._timeline.axes['arm'][0]
    assert (arm.start, arm.end) == pytest.approx((1.3, 3.3))
    assert s.joint_queue(0, now=4) == 1
    settle(s, 'arm', arm.target, 4)
    s.check_deadlines(now=4)
    assert s._timeline.axes['swing'][0].start == 4


def test_completed_prefix_does_not_clear_or_resend_held_tail():
    """前半だけの成功で保留を消したり同じ保持goalを再送したりしない。"""
    s = scheduler()
    gated(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    s.mark_completed(p, successful=True, now=1)
    assert s.motion_status() == 1
    assert s.pending_snapshot(now=1.1) is None
    s.check_deadlines(now=2)  # 待機中にmax_dispatch_delayを誤適用しない。
    assert s.motion_error() == ''
    arrive(s, now=2)
    q = s.pending_snapshot(now=2)
    assert q is not None and s.joint_queue(2, now=2) == 0


def test_late_prefix_success_after_gate_release_keeps_new_tail():
    """解除直後に前半の成功が届いても、新しい区間を残す。"""
    s = scheduler()
    gated(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    arrive(s, now=1.1)
    s.check_deadlines(now=1.1)
    s.mark_completed(p, successful=True, now=1.1)
    assert s.motion_status() == 1
    assert s.pending_snapshot(now=1.1).target.duration_sec == pytest.approx(1)


@pytest.mark.parametrize('finish_prefix', [False, True])
def test_timeout_fails_flushes_and_uses_node_safety_path(finish_prefix):
    """Action実行中と成功済みの双方で期限切れが走行停止へ届くことを確認する。"""
    h = Harness()
    gated(h.scheduler)
    h.tick(0)
    handle = h.accept(0)
    if finish_prefix:
        h.result(0, now=1)
    h.bridge._state.apply_command('left_crawler_speed', 1)
    h.tick(11.01)
    assert h.scheduler.motion_status() == -1
    assert h.scheduler.motion_error() == 'serial_arrival_timeout'
    assert all(h.scheduler.joint_queue(i, now=12) == 0 for i in range(4))
    assert h.bridge._state.next_frame().linear_velocity == 0
    assert h.scheduler.needs_cancel() == (not finish_prefix)
    if not finish_prefix:
        assert handle.cancel_calls == 1
    assert h.scheduler.pending_snapshot(now=12) is None


@pytest.mark.parametrize('reset', [False, True])
def test_stop_reset_discard_gates_and_rebase(reset):
    """停止・リセット後は保留を捨てて新しい実測を起点にする。"""
    s = scheduler()
    gated(s)
    s.stop(reset=reset)
    assert s.joint_queue(2, now=2) == 0
    assert not s._arrival_pending and not s._arrival_gate
    arrive(s, now=2, boom=-60)
    assert add(s, now=2)
    assert math.degrees(s._timeline.axes['boom'][0].origin) == pytest.approx(-60)


def test_failure_and_feedback_loss_clear_gate():
    """Action失敗とfeedback途絶のどちらでも保留が残らない。"""
    s = scheduler()
    gated(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    s.mark_completed(p, successful=False, now=1, reason='goal_rejected')
    assert s.motion_error() == 'goal_rejected' and s.joint_queue(2, now=1) == 0
    s = scheduler()
    gated(s)
    s._feedback_timeout_sec = .5
    s.check_deadlines(now=.6)
    assert s.motion_error() == 'stale' and not s._arrival_pending


def test_before_feedback_preserves_arrival_gate_order():
    """feedback前に受けた到着ゲートも通常の指令順で解決する。"""
    s = TrajectoryScheduler(feedback_timeout_sec=0)
    assert add(s)
    s.barrier(now=.1, arrival=True)
    assert add(s, command='arm_extend', now=.1)
    s.update_from_joint_state(NAMES, [0, math.radians(-50), 2, 1], now=.2)
    p = s.pending_snapshot(now=.2)
    assert p.target.duration_sec == pytest.approx(1)
    assert s.joint_queue(2, now=3) == 1
    s.mark_submitted(p)
    arrive(s, now=3)
    s.check_deadlines(now=3)
    assert s._timeline.axes['arm'][0].start == 3


def test_legacy_barrier_starts_despite_outside_feedback():
    """val=0は実測の遅れを条件にせず、従来の計画時計で開始する。"""
    s = scheduler()
    assert add(s)
    s.barrier(now=.1)
    assert add(s, command='arm_extend', now=.1)
    assert s._timeline.axes['arm'][0].start == pytest.approx(1)
    assert s.joint_queue(2, now=1.01) == 0


@pytest.mark.parametrize('name', [a+'_serial_arrival_tolerance_deg' for a in ('swing', 'boom', 'arm', 'bucket')]+['serial_arrival_speed_deg_s', 'serial_arrival_timeout_sec'])
@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_parameters(name, value):
    """無効化を招くゼロ・負値・非有限の到着設定を拒否する。"""
    with pytest.raises(ValueError): TrajectoryScheduler(**{name: value})


def test_guard_waits_behind_gate_then_contact_updates_arrival_target():
    """保留中guardを始動させず、接地後の目標は打切り位置へ切り替える。"""
    s = scheduler()
    assert add(s, command='arm_extend')
    s.barrier(now=.1, arrival=True)
    s.update_ground_contact(0, now=.1)
    assert s.start_ground_guard(10, 10, now=.1)
    assert s._guard_segment is None
    s.mark_submitted(s.pending_snapshot(now=.1))
    s.update_ground_contact(1, now=.5)
    assert s.check_ground_guard(now=.5) is None
    assert s.guard_status() == 'running'
    arm = s._timeline.axes['arm'][0]
    settle(s, 'arm', arm.target, 1.1)
    # 保留中のguardが、直前のゲート解除そのものを妨げてはいけない。
    s.check_deadlines(now=1.1)
    assert s._guard_segment.start == 1.1
    s.update_ground_contact(1, now=1.1)
    s.check_ground_guard(now=1.1)
    assert s.guard_status() == 'contact'
    s.barrier(now=1.1, arrival=True)
    assert add(s, command='bucket_dump', now=1.1)
    settle(s, 'boom', s._feedback_positions_rad['boom'], 1.22)
    s.check_deadlines(now=1.22)
    assert s._timeline.axes['bucket'][0].start == 1.22


def test_http_barrier_value_and_omission():
    """HTTPで省略・0・1を受付し、未定義の値を拒否する。"""
    h = Harness()
    server = create_http_server('127.0.0.1', 0, h.bridge._state, h.bridge._command_callback)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}/?type=command&cmd=barrier'
    try:
        assert _get(base)[0] == 200
        assert _get(base+'&val=0')[0] == 200
        assert _get(base+'&val=2')[0] == 400
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=.1):
            assert add(h.scheduler)
            assert _get(base+'&val=1')[0] == 200
        assert h.scheduler._arrival_gate
    finally:
        server.shutdown(); server.server_close(); thread.join()


def test_gate_does_not_disable_initial_dispatch_deadline():
    """到着待ちがあっても初回送信できない異常を期限内に検出する。"""
    s = scheduler()
    gated(s)
    s.check_deadlines(now=1.01)
    assert s.motion_error() == 'stale'


def test_timeout_uses_later_of_barrier_receipt_and_plan_end():
    """古い計画の後でゲートを受けても、待ち時間を遡って即失敗させない。"""
    s = scheduler()
    assert add(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    s.barrier(now=20, arrival=True)
    assert add(s, command='arm_extend', now=20)
    s.check_deadlines(now=29.9)
    assert s.motion_status() == 1
    s.check_deadlines(now=30.01)
    assert s.motion_error() == 'serial_arrival_timeout'


def test_swing_gate_accepts_equivalent_continuous_angle():
    """連続関節の同値角を未到着と誤認しない。"""
    s = scheduler()
    assert add(s, command='swing_left', delta=10)
    s.barrier(now=.1, arrival=True)
    assert add(s, command='arm_extend', now=.1)
    s.mark_submitted(s.pending_snapshot(now=0))
    settle(s, 'swing', math.radians(370), 1.1)
    s.check_deadlines(now=1.1)
    assert s.joint_queue(2, now=1.1) == 0


def test_guard_slowdown_moves_arrival_deadline_and_target():
    """接地減速で延びた計画時刻より前にゲートを期限切れにしない。"""
    s = scheduler()
    s.update_ground_contact(0, now=0)
    assert s.start_ground_guard(10, 10, now=0)
    s.barrier(now=.1, arrival=True)
    assert add(s, command='arm_extend', now=.1)
    s.mark_submitted(s.pending_snapshot(now=0))
    s.update_ground_clearance(.1, now=.2)
    s.update_ground_contact(0, now=.2)
    s.check_ground_guard(now=.2)
    assert s._guard_segment.end > 10
    s.check_deadlines(now=11.1)
    assert s.motion_error() == '' and s.joint_queue(2, now=11.1) == 1


def test_new_motion_after_completed_gated_sequence_is_dispatched():
    """完了済みの計画キーが次の動作の送信を妨げない。"""
    s = scheduler()
    gated(s)
    p = s.pending_snapshot(now=0)
    s.mark_submitted(p)
    s.mark_completed(p, successful=True, now=1)
    arrive(s, now=1.1)
    q = s.pending_snapshot(now=1.1)
    s.mark_submitted(q)
    settle(s, 'arm', q.target.positions_rad[2], 2.2)
    s.mark_completed(q, successful=True, now=2.2)
    assert s.motion_status() == 0
    assert add(s, now=3)
    s.barrier(now=3.1, arrival=True)
    assert add(s, command='arm_extend', now=3.1)
    assert s.pending_snapshot(now=3.1) is not None


def test_later_legacy_barrier_does_not_bypass_arrival_gate():
    """後続のval=0を先行ゲートの解除として扱わない。"""
    s = scheduler()
    gated(s)
    s.barrier(now=.2, arrival=False)
    assert add(s, command='bucket_dump', now=.2)
    s.mark_submitted(s.pending_snapshot(now=0))
    s.check_deadlines(now=1.1)
    assert s.joint_queue(2, now=1.1) == s.joint_queue(3, now=1.1) == 1
    arrive(s, now=1.2)
    s.check_deadlines(now=1.2)
    arm, bucket = s._timeline.axes['arm'][0], s._timeline.axes['bucket'][0]
    assert arm.start == 1.2 and bucket.start == arm.end


def test_outside_tolerance_by_small_margin_still_waits():
    """閾値の直外と直内を区別して到着判定する。"""
    s = scheduler()
    gated(s)
    s.mark_submitted(s.pending_snapshot(now=0))
    arrive(s, now=1.1, boom=-42.51)
    s.check_deadlines(now=1.1)
    assert s.joint_queue(2, now=1.1) == 1
    arrive(s, now=1.2, boom=-42.49)
    s.check_deadlines(now=1.2)
    assert s.joint_queue(2, now=1.2) == 0


def test_success_including_future_prefix_segments_does_not_resend_hold():
    """同一goal内で後から始まった区間も成功済みとして保持再送を防ぐ。"""
    s = scheduler()
    assert add(s)
    assert add(s, now=.1)
    s.barrier(now=.2, arrival=True)
    assert add(s, command='arm_extend', now=.2)
    p = s.pending_snapshot(now=.2)
    s.mark_submitted(p)
    s.mark_completed(p, successful=True, now=2.1)
    assert s.pending_snapshot(now=2.2) is None
    s.check_deadlines(now=3.5)
    assert s.motion_status() == 1 and s.joint_queue(2, now=3.5) == 1


@pytest.mark.parametrize('axis,tolerance', [('swing',1.5),('boom',2.5),('arm',5.5),('bucket',5.5)])
def test_axis_tolerance_and_stationary_outside_timeout(axis, tolerance):
    """軸別境界を確認し、静止していても許容外なら期限で停止する。"""
    s = scheduler()
    command = dict(swing='swing_left', boom='boom_down', arm='arm_extend', bucket='bucket_dump')[axis]
    assert add(s, command=command)
    s.barrier(now=.1, arrival=True)
    assert add(s, command='swing_left' if axis != 'swing' else 'arm_extend', now=.1)
    s.mark_submitted(s.pending_snapshot(now=0))
    target = s._timeline.axes[axis][-1].target
    settle(s, axis, target-math.radians(tolerance+.01), 1.1)
    s.check_deadlines(now=1.1)
    assert s._arrival_gate
    assert s.feedback_speed(axis) < math.radians(2)
    settle(s, axis, target-math.radians(tolerance-.01), 1.3)
    s.check_deadlines(now=1.3)
    assert not s._arrival_gate
    # 別の試行では静止許容外を期限まで維持する。
    s = scheduler(); gated(s)
    s.mark_submitted(s.pending_snapshot(now=0))
    arrive(s, 11.01, -43)
    s.check_deadlines(now=11.01)
    assert s.motion_error() == 'serial_arrival_timeout'


def test_fast_pass_inside_position_waits_for_smoothed_speed():
    """位置内を通過しても速度が高ければ保留し、停止観測後だけ解除する。"""
    s = scheduler(); gated(s)
    s.mark_submitted(s.pending_snapshot(now=0))
    for t in (1.,1.02,1.04,1.06,1.08,1.10):
        s.update_from_joint_state(['boom_joint'], [math.radians(-41+(t-1)*10)], now=t)
    s.check_deadlines(now=1.1)
    assert s._arrival_gate and math.degrees(s.feedback_speed('boom')) == pytest.approx(10)
    arrive(s, now=1.24)
    s.check_deadlines(now=1.24)
    assert not s._arrival_gate


def test_speed_requires_continuous_history_and_handles_wrap_reversal_and_noise():
    """欠測を静止とせず、旋回wrap・反転相殺・小さな角度ノイズを扱う。"""
    s = scheduler()
    s.update_from_joint_state(['boom_joint'], [0.], now=1)
    assert math.isinf(s.feedback_speed('boom'))
    for i in range(1,7):
        s.update_from_joint_state(['boom_joint'], [math.radians(.001*(-1)**i)], now=1+i*.02)
    assert math.degrees(s.feedback_speed('boom')) < .11
    for i, angle in enumerate((359.9,359.95,0.,.05,.1,.15)):
        s.update_from_joint_state(['swing_joint'], [math.radians(angle)], now=2+i*.02)
    assert math.degrees(s.feedback_speed('swing')) == pytest.approx(2.5)
    for i, angle in enumerate((0.,.2,.4,.4,.2,0.)):
        s.update_from_joint_state(['arm_joint'], [math.radians(angle)], now=2+i*.02)
    assert math.degrees(s.feedback_speed('arm')) == pytest.approx(8.)
    s.update_from_joint_state(['swing_joint'], [math.radians(.15)], now=3)
    assert math.isinf(s.feedback_speed('swing'))


def test_hold_rebase_changes_output_and_mask_but_preserves_relative_target():
    """他軸のgoal再送で停止済みbucketを戻さず、往復の理想終点を累積誤差から守る。"""
    s = scheduler()
    assert add(s, command='bucket_dump', delta=20)
    original = s._timeline.axes['bucket'][-1].origin
    ideal = s._timeline.axes['bucket'][-1].target
    s.barrier(now=.1, arrival=True)
    assert add(s, command='boom_down', now=.1)
    p = s.pending_snapshot(now=0); s.mark_submitted(p)
    measured = ideal+math.radians(3.)
    settle(s, 'bucket', measured, 2.2)
    q = s.pending_snapshot(now=2.2, replacing=p)
    assert q is not None and all(point.positions[3] == measured for point in q.track_points)
    assert s.tracking_targets(now=2.2)['bucket'] == measured
    assert 'bucket' in s.serial_held_axes(now=2.2)
    assert s.mark_submitted(q, replacing=p)
    assert s._commanded_targets_rad['bucket'] == ideal
    # 次の相対復帰は実測+20でなく、元の理想終点まで戻る。
    assert add(s, command='bucket_scoop', delta=20, now=2.3)
    segment = s._timeline.axes['bucket'][-1]
    assert segment.target == pytest.approx(original)
    assert segment.origin == measured
    assert 'bucket' not in s.serial_held_axes(now=segment.start)
    s.stop(reset=True)
    assert not s.serial_held_axes(now=3)


def test_node_mask_suppresses_reaccepted_old_target_during_goal_response_delay():
    """新goalの受理が遅れても、古いcatch-up目標による補正出力を復活させない。"""
    from unittest.mock import Mock
    h = Harness(); s = h.scheduler
    s._serial_holds['bucket'] = (1., float('inf'))
    h.bridge._mask_commanded_axes = {'boom'}
    h.bridge._mask_catch_up_targets = {'bucket': 2.}
    h.bridge._active_axis_mask_publisher = Mock()
    h.bridge._last_active_axis_mask = 0
    with patch('scratch_hci_bridge.trajectory_scheduler.time.monotonic', return_value=2.):
        h.bridge._write_active_axis_mask()
    assert h.bridge._active_axis_mask_publisher.publish.call_args.args[0].data == 2


def test_relative_basis_survives_completed_action_with_measured_holds():
    """Action完了でタイムラインが消えた後も、保持実測を相対量へ二重に加えない。"""
    s=scheduler()
    original=s._feedback_positions_rad['bucket']
    assert add(s,command='bucket_dump',delta=20)
    s.barrier(now=.1,arrival=True)
    assert add(s,command='boom_down',now=.1)
    p=s.pending_snapshot(now=0);s.mark_submitted(p)
    ideal=s._timeline.axes['bucket'][-1].target
    settle(s,'bucket',ideal+math.radians(3),2.2)
    q=s.pending_snapshot(now=2.2,replacing=p)
    assert s.mark_submitted(q,replacing=p)
    settle(s,'boom',q.target.positions_rad[1],3.3)
    assert s.mark_completed(q,successful=True,now=3.3)
    assert s._timeline is None
    assert add(s,command='bucket_scoop',delta=20,now=4)
    assert s._timeline.axes['bucket'][-1].target == pytest.approx(original)
    assert s._timeline.axes['bucket'][-1].origin == pytest.approx(ideal+math.radians(3))


def test_equivalent_swing_hold_does_not_add_full_turn_to_next_relative_motion():
    """wrapした実測を保持しても次の相対軌道へ360度のジャンプを入れない。"""
    s=scheduler()
    assert add(s,command='swing_left',delta=10)
    s.barrier(now=.1,arrival=True)
    assert add(s,command='arm_extend',now=.1)
    s.mark_submitted(s.pending_snapshot(now=0))
    settle(s,'swing',math.radians(370),1.2)
    s.check_deadlines(now=1.2)
    assert add(s,command='swing_left',delta=10,now=1.3)
    segment=s._timeline.axes['swing'][-1]
    assert math.degrees(segment.origin) == pytest.approx(10)
    assert math.degrees(segment.target) == pytest.approx(20)

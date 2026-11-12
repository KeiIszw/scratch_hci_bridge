"""接地前減速の位置連続・barrier順序・Action受理ゲート・停止経路を検証する。"""
import math
from types import SimpleNamespace as NS
from unittest.mock import patch
import pytest
from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler
from test_sync_safety import scheduler
from test_ground_guard import start
from bridge_test_helpers import Harness


def test_slow_joins_planned_position_once_and_preserves_barrier_order():
    s=scheduler(); start(s)
    old=s._guard_segment
    s.barrier(now=0)
    assert s.update_http_target('arm_extend',10,base_duration_sec=1,now=0)
    arm=s._timeline.axes['arm'][-1]
    expected=s._timeline.state('boom',.4)[0]
    first=s.pending_snapshot(now=0); s.mark_submitted(first)
    s.update_ground_clearance(.199,now=.4)
    s.check_ground_guard(.4)
    new=s._guard_segment
    assert new.start == .4 and new.origin == pytest.approx(expected)
    assert new.target == old.target and new.end > old.end
    assert s._timeline.state('boom',.4-1e-9)[0] == pytest.approx(expected,abs=1e-8)
    assert s._timeline.axes['arm'][-1].start == pytest.approx(new.end)
    assert s._timeline.axes['arm'][-1].target == arm.target
    assert s._timeline.barrier_time == pytest.approx(new.end)
    replacement=s.pending_snapshot(now=.4,replacing=first)
    assert replacement is not None
    # 減速前の成功結果が遅着しても、残区間は消えない。
    s.mark_completed(first,successful=True,now=.4)
    assert s.has_work() and s.guard_status()=='running'
    revision=s._revision
    s.update_ground_clearance(.3,now=.5);s.update_ground_contact(0,now=.5);s.check_ground_guard(.5)
    assert s._revision==revision and s._guard_segment is new


def test_slow_never_accelerates_already_slow_motion_and_stale_distance_is_ignored():
    s=scheduler();s.update_ground_contact(0,now=0)
    assert s.start_ground_guard(2,speed_limit_deg_s=.5,now=0)
    old=s._guard_segment
    s.update_ground_clearance(.1,now=-1);s.check_ground_guard(.1)
    assert s._guard_segment is old and not s._guard_slowed
    s.update_ground_clearance(.1,now=.1);s.check_ground_guard(.1)
    assert s._guard_segment is old and s._guard_slowed


def test_waiting_guard_does_not_modify_preceding_motion():
    s=scheduler()
    s.update_http_target('arm_extend',5,base_duration_sec=1,now=0);s.barrier(now=0)
    start(s,now=.1)
    old=s._guard_segment; arm=s._timeline.axes['arm'][:]
    s.update_ground_clearance(.1,now=.2);s.check_ground_guard(.2)
    assert s._guard_segment is old and not s._guard_slowed
    s.update_ground_contact(0,now=1);s.update_ground_clearance(.1,now=1);s.check_ground_guard(1)
    assert s._guard_segment.start==1 and s._guard_segment.end>old.end
    assert s._timeline.axes['arm']==arm


def test_replacement_cubic_speed_is_bounded_at_one_degree_per_second():
    s=scheduler();start(s)
    s.update_ground_clearance(.1,now=.1);s.check_ground_guard(.1)
    points=s.pending_snapshot(now=.1).track_points
    # JTCと同じHermite補間の導関数を区間内で調べ、平均速度だけの検証にしない。
    for left,right in zip(points,points[1:]):
        dt=right.time-left.time
        for u in [i/100 for i in range(101)]:
            v=((6*u*u-6*u)*left.positions[1]+(-6*u*u+6*u)*right.positions[1])/dt
            v+=(3*u*u-4*u+1)*left.velocities[1]+(3*u*u-2*u)*right.velocities[1]
            assert abs(math.degrees(v))<=1.000001


def test_node_slow_during_pending_goal_waits_for_acceptance_then_contact_holds():
    h=Harness();s=h.scheduler;start(s)
    h.tick(0)
    with patch('scratch_hci_bridge.node.time.monotonic',return_value=.1):
        h.bridge._ground_clearance_callback(NS(data=.1))
    h.tick(.1)
    assert len(h.sent)==1 and s._guard_slowed
    h.accept(0,now=.1);h.tick(.12)
    assert len(h.sent)==2
    h.accept(1,now=.12)
    with patch('scratch_hci_bridge.node.time.monotonic',return_value=.2):
        h.bridge._ground_contact_callback(NS(data=1))
    h.tick(.2)
    assert s.guard_status()=='contact' and len(h.sent)==3
    assert all(p.velocities[1]==0 for p in h.sent[2][0].trajectory.points)
    s.stop();assert s.guard_status()=='contact'
    s.stop(reset=True);assert s.guard_status()=='reached'


@pytest.mark.parametrize('parameter',['ground_slow_distance_m','ground_slow_speed_deg_s',
                                      'ground_stall_sec','ground_stall_epsilon_deg'])
@pytest.mark.parametrize('value',[0,-1,float('nan'),float('inf')])
def test_invalid_slow_settings(parameter,value):
    with pytest.raises(ValueError):TrajectoryScheduler(**{parameter:value})


@pytest.mark.parametrize('distance',[.2,.3,float('nan'),float('inf')])
def test_slow_requires_finite_distance_strictly_below_threshold(distance):
    s=scheduler();start(s);old=s._guard_segment
    s.update_ground_clearance(distance,now=.1);s.check_ground_guard(.1)
    assert s._guard_segment is old and not s._guard_slowed


@pytest.mark.parametrize('reason',['stopped','lease_expired','stale','goal_rejected'])
def test_stop_after_slow_keeps_existing_failure_and_reset_rules(reason):
    s=scheduler();start(s)
    s.update_ground_clearance(.1,now=.1);s.check_ground_guard(.1)
    assert s._guard_slowed
    s.stop(reason)
    assert not s.has_work() and s.guard_status()=='failed'
    s.stop(reset=True)
    assert s.guard_status()=='reached' and s.motion_status()==0


@pytest.mark.parametrize('advance_deg', [3., 4., 25.])
def test_measured_ahead_slow_never_commands_upward_even_beyond_endpoint(advance_deg):
    from test_sync_safety import NAMES
    s = scheduler(); start(s)
    measured = s._timeline.state('boom', .4)[0] + math.radians(advance_deg)
    s.update_from_joint_state(NAMES, [0, measured, 2, 1], now=.4)
    s.update_ground_clearance(.15, now=.4)
    s.check_ground_guard(.4)
    points = s.pending_snapshot(now=.4).track_points
    assert points[0].positions[1] == pytest.approx(measured)
    for left, right in zip(points, points[1:]):
        dt = right.time-left.time
        for i in range(101):
            u = i/100
            value = ((2*u**3-3*u*u+1)*left.positions[1]+(-2*u**3+3*u*u)*right.positions[1]
                     + dt*((u**3-2*u*u+u)*left.velocities[1]+(u**3-u*u)*right.velocities[1]))
            velocity = ((6*u*u-6*u)*left.positions[1]+(-6*u*u+6*u)*right.positions[1])/dt
            velocity += (3*u*u-4*u+1)*left.velocities[1]+(3*u*u-2*u)*right.velocities[1]
            assert value >= measured-1e-12
            assert -1e-12 <= velocity <= math.radians(1)+1e-12


def test_real_wait_for_plan_is_not_stall_and_window_starts_only_after_demand():
    from test_guard_finish import feedback
    s = scheduler(); start(s)
    # 旧減速後の実測先行を模す。窓内の計画変位が大きくても下降要求がなければ数えない。
    for t in (.1, .3, .61):
        feedback(s, t, boom=-40, distance=.4)
        s.check_ground_guard(t)
    assert s.guard_status() == 'running'
    assert not s._guard_progress.samples
    # 計画が追いついた最初の観測だけで、過去の待ち時間を停滞へ加算しない。
    for t in (1.1, 1.3):
        feedback(s, t, boom=-40, distance=.4)
        s.check_ground_guard(t)
        assert s.guard_status() == 'running'
    feedback(s, 1.61, boom=-40, distance=.4)
    s.check_ground_guard(1.61)
    assert s.guard_status() == 'blocked'


def test_slow_cubic_start_wait_does_not_count_as_stall_but_obstacle_eventually_does():
    from test_guard_finish import feedback
    s = scheduler(); start(s)
    feedback(s, .1); s.check_ground_guard(.1)
    for i in range(2, 100):
        t = i/10
        feedback(s, t)
        s.check_ground_guard(t)
        if t < .7:
            assert s.guard_status() == 'running'
        if s.guard_status() != 'running':
            break
    assert s.guard_status() == 'stalled'


def test_waiting_observation_breaks_preceding_stall_window():
    from test_guard_finish import feedback
    s = scheduler(); start(s)
    for t in (.1, .4):
        feedback(s, t, distance=.4)
    feedback(s, .5, boom=-30, distance=.4)
    assert not s._guard_progress.samples
    for t in (.6, .9):
        feedback(s, t, distance=.4)
        s.check_ground_guard(t)
        assert s.guard_status() == 'running'
    feedback(s, 1.11, distance=.4)
    s.check_ground_guard(1.11)
    assert s.guard_status() == 'blocked'

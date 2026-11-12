"""実機やROS通信を使わず、停止と期限の状態遷移・HTTP並行性を検証する。"""
from contextlib import nullcontext
import math
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from scratch_hci_bridge.command_state import CommandState
from scratch_hci_bridge.http_server import ScratchHTTPRequestHandler, ScratchHTTPServer
from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler


NAMES = ['swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint']


def scheduler():
    result = TrajectoryScheduler(feedback_timeout_sec=0)
    result.update_from_joint_state(NAMES, [0.0, math.radians(-50), 2.0, 1.0], now=0)
    return result


class SafetyTest(unittest.TestCase):
    def test_stop_discards_queue_but_waits_for_terminal_result(self):
        s = scheduler()
        s.update_http_target('boom_down', 10, now=0)
        goal = s.pending_snapshot(now=0)
        s.mark_submitted(goal)
        s.update_http_target('arm_extend', 10, now=.1)
        s.stop()
        self.assertEqual(s.motion_status(), 1)
        self.assertTrue(all(s.joint_queue(i, now=0) == 0 for i in range(4)))
        self.assertIsNone(s.pending_snapshot(now=10))
        s.mark_completed(goal, successful=False)
        self.assertEqual(s.motion_status(), 2)
        self.assertEqual(s.motion_error(), 'stopped')
        s.update_http_target('boom_up', 1, now=11)
        self.assertEqual(s.motion_status(), 1)

    def test_reset_clears_failure_and_pending_work(self):
        s = scheduler()
        s.update_http_target('boom_down', 1, now=0)
        s.stop('lease_expired')
        self.assertEqual(s.motion_status(), -1)
        self.assertEqual(s.motion_error(), 'lease_expired')
        s.stop(reset=True)
        self.assertEqual(s.motion_status(), 0)
        self.assertEqual(s.motion_error(), '')
        self.assertFalse(s.has_work())


    def test_first_failure_survives_stop_and_later_failures_until_reset(self):
        for reason in ('stale', 'tolerance_violated'):
            with self.subTest(reason=reason):
                s = scheduler()
                s.stop(reason)
                s.stop()
                s.stop('lease_expired')
                self.assertEqual(s.motion_error(), reason)
                self.assertEqual(s.motion_status(), -1)
                s.stop(reset=True)
                self.assertEqual(s.motion_error(), '')
                self.assertEqual(s.motion_status(), 0)
                s.stop()
                self.assertEqual(s.motion_error(), 'stopped')
                self.assertEqual(s.motion_status(), 2)


    def test_relative_basis_keeps_target_regardless_of_error_and_measured_after_reset(self):
        for measured, reset, expected in ((-40.5, False, -39), (-46, False, -39), (-40.5, True, -39.5)):
            with self.subTest(measured=measured, reset=reset):
                s = scheduler()
                s.update_http_target('boom_down', 10, now=0)
                first = s.pending_snapshot(now=0)
                s.mark_submitted(first)
                s.mark_completed(first, successful=True, now=1)
                s.update_from_joint_state(NAMES, [0, math.radians(measured), 2, 1], now=1)
                if reset:
                    s.stop(reset=True)
                s.update_http_target('boom_down', 1, now=1)
                second = s.pending_snapshot(now=1)
                self.assertAlmostEqual(math.degrees(second.target.positions_rad[1]), expected)


class HttpMemoryTest(unittest.TestCase):
    def setUp(self):
        self.server = SimpleNamespace(command_condition=threading.Condition(), command_lock=threading.Lock(),
                                      state_lock=nullcontext(), pending_orders=set(),
                                      request_orders={}, command_epoch=0)
        self.state = CommandState()
        self.callback = Mock()
        self.activity = Mock()

    def request(self, command, value=0, kind='command', order=1):
        handler_type = type('BoundHandler', (ScratchHTTPRequestHandler,), {
            'state': self.state, 'command_callback': staticmethod(self.callback),
            'request_callback': staticmethod(self.activity), 'status_callback': staticmethod(lambda: 2),
            'joint_queue_callback': staticmethod(lambda i: i),
            'error_callback': staticmethod(lambda: 'stopped')})
        h = handler_type.__new__(handler_type)
        h.server = self.server
        h.request = object()
        self.server.request_orders[h.request] = (order, self.server.command_epoch)
        h.path = f'/?type={kind}&cmd={command}&val={value}'
        result = []
        h._send_text = lambda code, text: result.append((code, text))
        h.do_GET()
        return result[0]

    def test_reporter_and_validation(self):
        self.assertTrue(ScratchHTTPServer.daemon_threads)
        self.assertEqual(self.request('heartbeat', kind='reporter'), (200, 'ok'))
        self.activity.assert_any_call(True)
        self.assertEqual(self.request('motion_status', kind='reporter'), (200, 'stopped'))
        self.assertEqual(self.request('motion_error', kind='reporter'), (200, 'stopped'))
        self.assertEqual(self.request('joint_queue', value=3, kind='reporter'), (200, '3'))
        self.assertEqual(self.request('joint_queue', value=4, kind='reporter')[0], 400)
        self.assertEqual(self.request('unknown', kind='reporter')[0], 400)
        self.assertEqual(self.request('commit', 2)[0], 400)
        self.assertEqual(self.request('stop', 99)[0], 200)

    def test_stop_and_reporter_bypass_blocked_command(self):
        entered, release = threading.Event(), threading.Event()
        def callback(command, value):
            if command == 'boom_angle':
                entered.set()
                release.wait(2)
        self.callback = callback
        worker = threading.Thread(target=lambda: self.request('boom_angle', -50))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.request('stop', order=2), (200, 'OK'))
            self.assertEqual(self.request('motion_status', kind='reporter', order=3), (200, 'stopped'))
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_waiting_command_before_stop_cannot_apply_later(self):
        self.server.pending_orders.add(1)
        entered = threading.Event()
        original_activity = self.activity
        def activity(heartbeat):
            original_activity(heartbeat)
            entered.set()
        self.activity = activity
        result = []
        worker = threading.Thread(target=lambda: result.append(self.request('boom_angle', -50, order=2)))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.request('stop', order=3), (200, 'OK'))
        finally:
            with self.server.command_condition:
                self.server.pending_orders.clear()
                self.server.command_condition.notify_all()
            worker.join(2)
        self.assertEqual(result, [(200, 'OK')])
        self.callback.assert_called_once_with('stop', 0)

    def test_stop_and_reset_serialize_epoch_with_new_connection(self):
        for command in ('stop', 'reset'):
            with self.subTest(command=command):
                self.setUp()
                server = ScratchHTTPServer.__new__(ScratchHTTPServer)
                server.__dict__.update(self.server.__dict__)
                self.server = server
                self.server.state_lock = threading.RLock()
                self.server.next_order = 1
                entered, release, registering = (threading.Event() for _ in range(3))
                applied = []
                motion = scheduler()
                motion.update_http_target('boom_down', 1, now=0)

                def callback(cmd, value):
                    applied.append(cmd)
                    if cmd == command:
                        entered.set()
                        release.wait(2)
                        motion.stop(reset=cmd == 'reset')
                    else:
                        motion.update_http_target(cmd, value, now=1)

                self.callback = callback
                responses = []
                stopping = threading.Thread(target=lambda: responses.append(self.request(command)))
                connection = object()

                def register():
                    registering.set()
                    ScratchHTTPServer.process_request(self.server, connection, ('127.0.0.1', 1))

                with patch('http.server.ThreadingHTTPServer.process_request'):
                    stopping.start()
                    try:
                        self.assertTrue(entered.wait(1))
                        accepting = threading.Thread(target=register)
                        accepting.start()
                        self.assertTrue(registering.wait(1))
                        self.assertNotIn(connection, self.server.request_orders)
                    finally:
                        release.set()
                        stopping.join(2)
                        accepting.join(2)
                self.assertFalse(stopping.is_alive())
                self.assertFalse(accepting.is_alive())
                self.assertEqual(responses, [(200, 'OK')])
                order, epoch = self.server.request_orders[connection]
                self.assertEqual(epoch, self.server.command_epoch)
                self.assertEqual(self.request('arm_angle', 90, order=order), (200, 'OK'))
                self.assertEqual(applied, [command, 'arm_angle'])
                goal = motion.pending_snapshot(now=1)
                self.assertIsNotNone(goal)
                self.assertEqual([c.joint_name for c in goal.commands], ['arm'])


class NodeSafetyTest(unittest.TestCase):
    def bridge(self):
        # 既存のダミーAction境界を使い、Node初期化やDDS接続は行わない。
        from test_node_command_callback import NodeCommandCallbackTest
        bridge, _, _ = NodeCommandCallbackTest()._bridge()
        bridge._action_server_unavailable_reported = False
        bridge._trajectory_scheduler = scheduler()
        bridge._state = CommandState()
        bridge._motion_lock = threading.RLock()
        bridge._stop_pending = False
        bridge._lease_enabled = False
        bridge._last_http_at = 0
        bridge._client_lease_sec = 2
        bridge._publish_drive_and_direct_commands = Mock()
        bridge._send_pending_trajectory = Mock()
        return bridge

    def test_lease_is_opt_in_and_all_http_activity_renews_it(self):
        b = self.bridge()
        s = b._trajectory_scheduler
        s.update_http_target('boom_down', 10, now=0)
        s.mark_submitted(s.pending_snapshot(now=0))
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=3):
            b._on_timer()
        self.assertNotEqual(s.motion_error(), 'lease_expired')
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=3):
            b._http_activity(True)
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=4.9):
            b._http_activity(False)
            b._on_timer()
        self.assertEqual(s.motion_error(), '')
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=7):
            b._on_timer()
        self.assertEqual(s.motion_error(), 'lease_expired')
        self.assertEqual(s.motion_status(), -1)
        self.assertTrue(s.needs_cancel())

    def test_stop_before_acceptance_cancels_late_goal_and_waits_for_result(self):
        from test_node_command_callback import _Future, _GoalHandle, _Publisher
        from action_msgs.msg import GoalStatus
        from control_msgs.action import FollowJointTrajectory
        b = self.bridge()
        s = b._trajectory_scheduler
        s.update_http_target('boom_down', 1, now=0)
        goal = s.pending_snapshot(now=0)
        s.mark_submitted(goal)
        result = _Future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                         result=SimpleNamespace(error_code=0, error_string='')))
        handle = _GoalHandle(result_future=result)
        response = _Future(handle)
        b._goal_response_future = response
        b._goal_response_scheduled = goal
        b._goal_submission_started_at = 0
        publisher = _Publisher()
        b._active_axis_mask_publisher = publisher
        b._state.apply_command('left_crawler_speed', 1)
        b._command_callback('stop', 123)
        self.assertEqual(s.motion_status(), 1)
        self.assertEqual(publisher.values[-1], 0)
        self.assertEqual(b._state.snapshot().left_crawler_speed, 0)
        b._goal_response_callback(response, goal)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(s.motion_status(), 1)
        b._goal_result_callback(result, goal)
        self.assertEqual(s.motion_status(), 2)
        self.assertEqual(s.motion_error(), 'stopped')
        b._command_callback('blade_angle', 2)
        self.assertEqual(s.motion_status(), 0)


    def test_failure_reasons_and_reset_during_active_goal(self):
        from test_node_command_callback import _Future, _GoalHandle
        from action_msgs.msg import GoalStatus
        from control_msgs.action import FollowJointTrajectory
        for status, code, reason in ((GoalStatus.STATUS_ABORTED, -1, 'goal_aborted'),
                                     (GoalStatus.STATUS_ABORTED, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, 'tolerance_violated'),
                                     (GoalStatus.STATUS_ABORTED, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, 'tolerance_violated')):
            b = self.bridge()
            s = b._trajectory_scheduler
            s.update_http_target('boom_down', 1, now=0)
            goal = s.pending_snapshot(now=0)
            s.mark_submitted(goal)
            result = _Future(SimpleNamespace(status=status, result=SimpleNamespace(error_code=code, error_string='dummy')))
            b._active_goal_scheduled = goal
            b._goal_result_future = result
            b._goal_result_callback(result, goal)
            self.assertEqual(s.motion_error(), reason)
            b._command_callback('reset', 0)
            self.assertEqual(s.motion_status(), 0)

    def test_rejection_and_result_timeout_reasons(self):
        from test_node_command_callback import _Future, _GoalHandle
        for reject in (False, True):
            b = self.bridge()
            s = b._trajectory_scheduler
            s.update_http_target('boom_down', 1, now=0)
            goal = s.pending_snapshot(now=0)
            s.mark_submitted(goal)
            handle = _GoalHandle(result_future=_Future())
            handle.accepted = not reject
            response = _Future(handle)
            b._goal_response_future = response
            b._goal_response_scheduled = goal
            b._goal_submission_started_at = 0
            b._goal_response_callback(response, goal)
            if reject:
                self.assertEqual(s.motion_error(), 'goal_rejected')
            else:
                b._check_action_timeouts(now=100)
                self.assertEqual(s.motion_error(), 'result_timeout')
                self.assertEqual(handle.cancel_calls, 1)
                b._command_callback('reset', 0)
                self.assertEqual(s.motion_error(), '')
                self.assertEqual(s.motion_status(), 1)


    def test_stale_joint_feedback_is_never_dispatched_after_deadline(self):
        s = TrajectoryScheduler()
        s.update_from_joint_state(NAMES, [0.0, -1.0, 2.0, 1.0], now=0)
        s.update_http_target('boom_down', 1, now=1)
        self.assertIsNone(s.pending_snapshot(now=1))
        s.check_deadlines(now=2.01)
        s.update_from_joint_state(NAMES, [0.0, -1.0, 2.0, 1.0], now=2.02)
        self.assertIsNone(s.pending_snapshot(now=2.02))
        self.assertEqual(s.motion_error(), 'stale')

class PrefetchTest(unittest.TestCase):


    def test_uncommanded_axes_hold_target_until_stop_reset_or_failure(self):
        for state in ('success', 'large_error', 'stop', 'reset', 'failure', 'retry'):
            with self.subTest(state=state):
                s = scheduler()
                s.update_http_target('arm_angle', 125.89, now=0)
                s.update_http_target('swing_angle', 10, now=0)
                first = s.pending_snapshot(now=0)
                s.mark_submitted(first)
                s.mark_completed(first, successful=state != 'failure', now=1)
                if state in ('stop', 'reset'):
                    s.stop(reset=state == 'reset')
                arm = 140 if state == 'large_error' else 128.03
                s.update_from_joint_state(NAMES, [math.radians(9.4), math.radians(-50), math.radians(arm), 1], now=1)
                s.update_http_target('boom_down', 1, now=1)
                goal = s.pending_snapshot(now=1)
                if state == 'retry':
                    s.mark_submitted(goal)
                    s.mark_send_failed(goal)
                    goal = s.pending_snapshot(now=1)
                expected_arm = 125.89 if state in ('success', 'retry', 'large_error') else arm
                expected_swing = 10 if state in ('success', 'retry', 'large_error') else 9.4
                self.assertAlmostEqual(math.degrees(goal.target.positions_rad[2]), expected_arm)
                self.assertAlmostEqual(math.degrees(goal.target.positions_rad[0]), expected_swing)

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
    result = TrajectoryScheduler(batch_wait_sec=0, initial_mode_command_wait_sec=0,
                                 feedback_timeout_sec=0)
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
        self.assertEqual(s.queued_command_count(), 0)
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

    def test_explicit_batch_ignores_debounce_and_mode_switch(self):
        s = scheduler()
        s.begin(now=0)
        s.update_http_target('boom_down', 1, now=.1)
        s.set_execution_mode('serial', now=1)
        s.update_http_target('arm_extend', 1, now=2)
        self.assertIsNone(s.pending_snapshot(now=2))
        s.commit(False, now=2)
        goal = s.pending_snapshot(now=2)
        self.assertEqual(len(goal.commands), 2)
        self.assertFalse(goal.wait_stopped)
        s.mark_submitted(goal)
        s.mark_completed(goal, successful=True, now=3)
        s.begin(now=3)
        s.commit(True, now=3)
        self.assertEqual(s.motion_status(), 0)

    def test_open_and_dispatch_deadlines(self):
        s = scheduler()
        s.begin(now=0)
        s.check_deadlines(now=5.01)
        self.assertEqual(s.motion_error(), 'stale')
        s.stop(reset=True)
        s.begin(now=6)
        s.update_http_target('boom_down', 1, now=6)
        s.commit(True, now=6)
        s.check_deadlines(now=7.01)
        self.assertIsNone(s.pending_snapshot(now=7.01))
        self.assertEqual(s.motion_error(), 'stale')
        self.assertFalse(s.has_work())

    def test_one_hz_commands_keep_explicit_batch_alive(self):
        s = scheduler()
        s.begin(now=0)
        for second in range(1, 13):
            s.check_deadlines(now=second)
            s.update_http_target('boom_down', 1, now=second)
            self.assertEqual(s.motion_error(), '')
        s.commit(False, now=13)
        goal = s.pending_snapshot(now=13)
        self.assertEqual(len(goal.commands), 12)
        self.assertFalse(goal.wait_stopped)

    def test_explicit_timeout_is_strictly_after_last_command(self):
        s = scheduler()
        s.begin(now=0)
        s.update_http_target('boom_down', 2, now=4)
        s.check_deadlines(now=9)
        self.assertEqual(s.motion_error(), '')
        s.check_deadlines(now=9.001)
        self.assertEqual(s.motion_error(), 'stale')
        self.assertFalse(s.has_work())
        self.assertIn('axis=boom', s.motion_failure_details())
        self.assertIn('target_deg=-48.000', s.motion_failure_details())

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

    def test_serial_success_starts_next_dispatch_deadline(self):
        s = scheduler()
        s._batch_wait_sec = .1
        s.set_execution_mode('serial', now=0)
        s.update_http_target('boom_down', 1, now=0)
        s.update_http_target('arm_extend', 1, now=0)
        first = s.pending_snapshot(now=.1)
        s.mark_submitted(first)
        s.check_deadlines(now=100)
        s.mark_completed(first, successful=True, now=100)
        self.assertIsNotNone(s.pending_snapshot(now=100.9))
        s.check_deadlines(now=101.01)
        self.assertEqual(s.motion_error(), 'stale')

    def test_queued_batches_start_deadline_after_previous_goal(self):
        for mode in ('parallel', 'explicit', 'serial'):
            with self.subTest(mode=mode):
                s = scheduler()
                s.update_http_target('boom_down', 1, now=0)
                first = s.pending_snapshot(now=0)
                s.mark_submitted(first)
                if mode == 'explicit':
                    s.begin(now=.1)
                else:
                    s.set_execution_mode(mode, now=.1)
                s.update_http_target('arm_extend', 1, now=.1)
                if mode == 'explicit':
                    s.commit(True, now=.1)
                s.check_deadlines(now=3)
                self.assertEqual(s.motion_error(), '')
                self.assertIsNone(s.pending_snapshot(now=3))
                s.mark_completed(first, successful=True, now=3)
                second = s.pending_snapshot(now=3.9)
                self.assertIsNotNone(second)
                self.assertEqual(second.commands[0].joint_name, 'arm')
                s.check_deadlines(now=4.01)
                self.assertEqual(s.motion_error(), 'stale')

    def test_debounce_during_goal_without_timer_uses_terminal_time(self):
        s = scheduler()
        s.update_http_target('boom_down', 1, now=0)
        first = s.pending_snapshot(now=0)
        s.mark_submitted(first)
        s._batch_wait_sec = .1
        s.update_http_target('arm_extend', 1, now=.1)
        s.mark_completed(first, successful=True, now=3)
        self.assertIsNotNone(s.pending_snapshot(now=3.9))
        self.assertEqual(s.motion_error(), '')

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
            'queue_callback': staticmethod(lambda: 3),
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
        self.assertEqual(self.request('motion_queue', kind='reporter'), (200, '3'))
        self.assertEqual(self.request('motion_queue', value=1, kind='reporter')[0], 400)
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
        s.begin(now=0)
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
        self.assertFalse(s.has_work())

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

    def test_prefetched_batch_is_sent_on_first_timer_after_result(self):
        self.check_prefetched_batch_after_result(0)

    def test_prefetched_batch_is_sent_after_intermediate_tolerance_warning(self):
        self.check_prefetched_batch_after_result(-5)

    def check_prefetched_batch_after_result(self, code):
        from test_node_command_callback import _Future, _GoalHandle
        from action_msgs.msg import GoalStatus
        b = self.bridge()
        del b._send_pending_trajectory
        s = b._trajectory_scheduler
        b._trajectory_duration_sec = 1
        b._trajectory_goal_tolerance_rad = math.radians(.75)
        b._trajectory_stopped_velocity_rad_s = math.radians(.5)
        b._trajectory_client = Mock()
        b._trajectory_client.server_is_ready.return_value = True
        b._trajectory_client.send_goal_async.return_value = _Future()
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=0):
            s.begin(now=0)
            s.update_http_target('boom_down', 1, now=0)
            s.commit(False, now=0)
            b._on_timer()
        first = s._in_flight
        result = _Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED if code == 0 else GoalStatus.STATUS_ABORTED,
                         result=SimpleNamespace(error_code=code, error_string='dummy')))

        response = b._goal_response_future
        response._result = _GoalHandle(result_future=result)
        b._goal_response_callback(response, first)
        s.begin(now=.1)
        s.update_http_target('arm_extend', 1, now=.1)
        s.commit(False, now=.1)
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=.2):
            b._on_timer()
        self.assertEqual(b._trajectory_client.send_goal_async.call_count, 1)
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=1):
            b._goal_result_callback(result, first)
            b._on_timer()
        self.assertEqual(b._trajectory_client.send_goal_async.call_count, 2)
        self.assertEqual(s._in_flight.commands[0].joint_name, 'arm')
        self.assertEqual(s.motion_error(), '')
        self.assertEqual(s.motion_status(), 1)

    def test_commit_velocity_tolerance_and_stale_server(self):
        from test_node_command_callback import _Future
        for wait_stopped in (False, True):
            with self.subTest(wait_stopped=wait_stopped):
                b = self.bridge()
                # このテストでは実際の送信メソッドを呼び、ダミーclientでGoalだけ検査する。
                del b._send_pending_trajectory
                s = b._trajectory_scheduler
                s.begin(now=0)
                s.update_http_target('boom_down', 1, now=0)
                s.commit(wait_stopped, now=0)
                b._trajectory_duration_sec = 1
                b._trajectory_goal_tolerance_rad = math.radians(.75)
                b._trajectory_stopped_velocity_rad_s = math.radians(.5)
                b._trajectory_client = Mock()
                b._trajectory_client.server_is_ready.return_value = False
                with patch('scratch_hci_bridge.trajectory_scheduler.time.monotonic', return_value=.5):
                    b._send_pending_trajectory()
                b._trajectory_client.send_goal_async.assert_not_called()
                b._trajectory_client.server_is_ready.return_value = True
                b._trajectory_client.send_goal_async.return_value = _Future()
                with patch('scratch_hci_bridge.trajectory_scheduler.time.monotonic', return_value=.6):
                    b._send_pending_trajectory()
                sent = b._trajectory_client.send_goal_async.call_args.args[0]
                for tolerance in sent.goal_tolerance:
                    self.assertEqual(tolerance.velocity, math.radians(.5) if tolerance.name == 'boom_joint' and wait_stopped else -1)

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

    def test_intermediate_tolerance_policy_and_next_relative_basis(self):
        from test_node_command_callback import _Future
        from action_msgs.msg import GoalStatus
        from control_msgs.action import FollowJointTrajectory
        cases = [(False, False, -5, -40.5, False, -39),
                 (False, False, -5, -46, False, -39),
                 (True, False, -5, -40.5, True, None),
                 (False, True, -5, -40.5, True, None),
                 (False, False, -4, -40.5, True, None),
                 (False, False, -1, -40.5, True, None)]
        for wait, fatal, code, measured, failed, expected in cases:
            with self.subTest(wait=wait, fatal=fatal, code=code, measured=measured):
                b = self.bridge()
                b._intermediate_tolerance_failure_fatal = fatal
                s = b._trajectory_scheduler
                s.begin(now=0)
                s.update_http_target('boom_down', 10, now=0)
                s.commit(wait, now=0)
                goal = s.pending_snapshot(now=0)
                s.mark_submitted(goal)
                s.begin(now=.1)
                s.update_http_target('boom_down', 1, now=.1)
                s.commit(False, now=.1)
                s.update_from_joint_state(NAMES, [0, math.radians(measured), 2, 1], now=1)
                result = _Future(SimpleNamespace(status=GoalStatus.STATUS_ABORTED,
                    result=SimpleNamespace(error_code=code, error_string='dummy')))
                b._active_goal_scheduled = goal
                b._goal_result_future = result
                with patch('scratch_hci_bridge.node.time.monotonic', return_value=1):
                    b._goal_result_callback(result, goal)
                self.assertEqual(s.motion_status() == -1, failed)
                if failed:
                    self.assertEqual(s.motion_queue(), 0)
                    self.assertIn('axis=boom target_deg=-40.000', b.get_logger().error_messages[-1])
                    self.assertIn(f'measured_deg={measured:.3f}', b.get_logger().error_messages[-1])
                    original = s.motion_error()
                    b._command_callback('stop', 0)
                    self.assertEqual(s.motion_error(), original)
                else:
                    self.assertEqual(s.motion_error(), '')
                    self.assertEqual(s.motion_queue(), 1)
                    following = s.pending_snapshot(now=1)
                    self.assertAlmostEqual(math.degrees(following.target.positions_rad[1]), expected)
                    self.assertIn('reason=tolerance_violated', b.get_logger().warning_messages[-1])

    def test_late_commit_logs_stale_target_and_preserves_reason_after_stop(self):
        b = self.bridge()
        s = b._trajectory_scheduler
        s.begin(now=0)
        s.update_http_target('boom_down', 2, now=1)
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=6.01):
            b._command_callback('commit', 0)
        message = b.get_logger().error_messages[-1]
        self.assertIn('reason=stale', message)
        self.assertIn('axis=boom target_deg=-48.000 measured_deg=-50.000 delta_deg=2.000', message)
        b._command_callback('stop', 0)
        self.assertEqual(s.motion_error(), 'stale')
        self.assertEqual(s.motion_status(), -1)
        b._command_callback('reset', 0)
        self.assertEqual(s.motion_error(), '')

    def test_intermediate_rejected_goal_is_fatal(self):
        from test_node_command_callback import _Future, _GoalHandle
        b = self.bridge()
        s = b._trajectory_scheduler
        s.begin(now=0)
        s.update_http_target('boom_down', 2, now=0)
        s.commit(False, now=0)
        goal = s.pending_snapshot(now=0)
        s.mark_submitted(goal)
        handle = _GoalHandle()
        handle.accepted = False
        response = _Future(handle)
        b._goal_response_future = response
        b._goal_response_scheduled = goal
        b._goal_response_callback(response, goal)
        self.assertEqual(s.motion_error(), 'goal_rejected')
        self.assertEqual(s.motion_status(), -1)
        self.assertEqual(s.motion_queue(), 0)
        self.assertIn('axis=boom', b.get_logger().error_messages[-1])

    def test_speed_commands_renew_batch_but_reporters_do_not(self):
        b = self.bridge()
        s = b._trajectory_scheduler
        s.begin(now=0)
        for second in range(1, 9):
            with patch('scratch_hci_bridge.node.time.monotonic', return_value=second):
                b._command_callback('boom_speed', 5)
                b._on_timer()
        self.assertEqual(s.motion_error(), '')
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=13.01):
            b._http_activity(False)
            b._on_timer()
        self.assertEqual(s.motion_error(), 'stale')
        self.assertIn('reason=stale', b.get_logger().error_messages[-1])

    def test_stale_joint_feedback_is_never_dispatched_after_deadline(self):
        s = TrajectoryScheduler(batch_wait_sec=0, initial_mode_command_wait_sec=0)
        s.update_from_joint_state(NAMES, [0.0, -1.0, 2.0, 1.0], now=0)
        s.update_http_target('boom_down', 1, now=1)
        self.assertIsNone(s.pending_snapshot(now=1))
        s.check_deadlines(now=2.01)
        s.update_from_joint_state(NAMES, [0.0, -1.0, 2.0, 1.0], now=2.02)
        self.assertIsNone(s.pending_snapshot(now=2.02))
        self.assertEqual(s.motion_error(), 'stale')

class PrefetchTest(unittest.TestCase):
    def test_queue_counts_and_commit_zero_has_no_dispatch_wait(self):
        s = scheduler()
        s._batch_wait_sec = .5
        self.assertEqual(s.motion_queue(), 0)
        s.begin(now=0)
        self.assertEqual(s.motion_queue(), 1)
        s.update_http_target('boom_down', 1, now=0)
        s.commit(False, now=0)
        self.assertEqual(s.motion_queue(), 1)
        first = s.pending_snapshot(now=0)
        s.mark_submitted(first)
        self.assertEqual(s.motion_queue(), 0)
        for t in (0.1, 0.2):
            s.begin(now=t)
            s.update_http_target('boom_down', 1, now=t)
            s.commit(False, now=t)
        self.assertEqual(s.motion_queue(), 2)
        self.assertIsNone(s.pending_snapshot(now=.9))
        for t, count in ((1, 1), (2, 0)):
            s.mark_completed(first, successful=True, now=t)
            first = s.pending_snapshot(now=t)
            self.assertIsNotNone(first)
            self.assertFalse(first.wait_stopped)
            s.mark_submitted(first)
            self.assertEqual(s.motion_queue(), count)
        s.mark_completed(first, successful=True, now=3)
        self.assertEqual(s.motion_queue(), 0)

    def test_queue_keeps_serial_remainder_and_new_batch_during_cancel(self):
        s = scheduler()
        s.update_http_target('boom_down', 1, now=0)
        s.update_http_target('arm_extend', 1, now=0)
        first = s.pending_snapshot(now=0)
        s.mark_submitted(first)
        self.assertEqual(s.motion_queue(), 1)
        s.stop(reset=True)
        s.begin(now=.1)
        self.assertEqual(s.motion_queue(), 1)
        s.update_http_target('boom_down', 1, now=.1)
        s.commit(False, now=.1)
        self.assertEqual(s.motion_queue(), 1)
        self.assertIsNone(s.pending_snapshot(now=.1))
        s.mark_completed(first, successful=False, now=.2)
        self.assertEqual(s.motion_queue(), 1)
        self.assertIsNotNone(s.pending_snapshot(now=.2))

    def test_uncommanded_axes_hold_target_until_stop_reset_or_failure(self):
        for state in ('success', 'large_error', 'stop', 'reset', 'failure', 'retry'):
            with self.subTest(state=state):
                s = scheduler()
                s.begin(now=0)
                s.update_http_target('arm_angle', 125.89, now=0)
                s.update_http_target('swing_angle', 10, now=0)
                s.commit(False, now=0)
                first = s.pending_snapshot(now=0)
                s.mark_submitted(first)
                s.mark_completed(first, successful=state != 'failure', now=1)
                if state in ('stop', 'reset'):
                    s.stop(reset=state == 'reset')
                arm = 140 if state == 'large_error' else 128.03
                s.update_from_joint_state(NAMES, [math.radians(9.4), math.radians(-50), math.radians(arm), 1], now=1)
                s.begin(now=1)
                s.update_http_target('boom_down', 1, now=1)
                s.commit(False, now=1)
                goal = s.pending_snapshot(now=1)
                if state == 'retry':
                    s.mark_submitted(goal)
                    s.mark_send_failed(goal)
                    goal = s.pending_snapshot(now=1)
                expected_arm = 125.89 if state in ('success', 'retry', 'large_error') else arm
                expected_swing = 10 if state in ('success', 'retry', 'large_error') else 9.4
                self.assertAlmostEqual(math.degrees(goal.target.positions_rad[2]), expected_arm)
                self.assertAlmostEqual(math.degrees(goal.target.positions_rad[0]), expected_swing)

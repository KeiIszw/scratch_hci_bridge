import importlib.util
import unittest
from types import SimpleNamespace
from typing import Any

from scratch_hci_bridge.command_state import CommandState


class _Logger:
    def __init__(self) -> None:
        self.info_messages: list[str] = []
        self.warning_messages: list[str] = []
        self.error_messages: list[str] = []

    def info(self, _message: str) -> None:
        self.info_messages.append(_message)

    def warning(self, _message: str) -> None:
        self.warning_messages.append(_message)

    def error(self, _message: str) -> None:
        self.error_messages.append(_message)


class _Scheduler:
    def __init__(self) -> None:
        self.targets: list[tuple[str, float, float | None]] = []
        self.send_failures: list[Any] = []
        self.completions: list[tuple[Any, bool]] = []

    def update_http_target(
        self,
        command: str,
        degrees: float,
        speed_limit_deg_s: float | None = None,
    ) -> bool:
        self.targets.append((command, degrees, speed_limit_deg_s))
        return True

    def mark_send_failed(self, scheduled: Any) -> bool:
        self.send_failures.append(scheduled)
        return True

    def mark_completed(
        self, scheduled: Any, *, successful: bool, preserve_queued: bool = False
    ) -> bool:
        self.preserve_queued = preserve_queued
        self.completions.append((scheduled, successful))
        return True


class _Future:
    def __init__(self, result: Any = None, exception: Exception | None = None) -> None:
        self._result = result
        self._exception = exception
        self.callbacks: list[Any] = []

    def result(self) -> Any:
        if self._exception is not None:
            raise self._exception
        return self._result

    def add_done_callback(self, callback: Any) -> None:
        self.callbacks.append(callback)

    def done(self) -> bool:
        return True


class _Publisher:
    def __init__(self) -> None:
        self.values: list[float] = []

    def publish(self, message: Any) -> None:
        self.values.append(message.data)


class _GoalHandle:
    accepted = True

    def __init__(
        self,
        *,
        result_future: _Future | None = None,
        result_exception: Exception | None = None,
        cancel_exceptions: list[Exception] | None = None,
    ) -> None:
        self._result_future = result_future
        self._result_exception = result_exception
        self._cancel_exceptions = list(cancel_exceptions or [])
        self.cancel_calls = 0
        self.cancel_future = _Future(
            SimpleNamespace(goals_canceling=[SimpleNamespace()])
        )

    def get_result_async(self) -> _Future:
        if self._result_exception is not None:
            raise self._result_exception
        assert self._result_future is not None
        return self._result_future

    def cancel_goal_async(self) -> _Future:
        self.cancel_calls += 1
        if self._cancel_exceptions:
            raise self._cancel_exceptions.pop(0)
        return self.cancel_future


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "ROS 2 is not installed")
class NodeCommandCallbackTest(unittest.TestCase):
    def _bridge(self) -> tuple[Any, _Scheduler, _Logger]:
        from scratch_hci_bridge.node import ScratchHciBridge

        bridge = ScratchHciBridge.__new__(ScratchHciBridge)
        scheduler = _Scheduler()
        logger = _Logger()
        bridge._trajectory_scheduler = scheduler
        bridge._trajectory_goal_response_timeout_sec = 5.0
        bridge._trajectory_result_timeout_margin_sec = 35.0
        bridge._goal_response_future = None
        bridge._goal_response_scheduled = None
        bridge._goal_submission_started_at = None
        bridge._goal_response_timeout_reported = False
        bridge._active_goal_handle = None
        bridge._active_goal_scheduled = None
        bridge._goal_result_future = None
        bridge._active_goal_deadline = None
        bridge._active_cancel_future = None
        bridge._active_cancel_requested = False
        bridge._active_cancel_retry_at = None
        bridge._unity_position_publishers = {}
        bridge._unity_position_signs = {
            "swing": -1.0,
            "boom": 1.0,
            "arm": 1.0,
            "bucket": 1.0,
        }
        bridge.get_logger = lambda: logger
        return bridge, scheduler, logger

    def test_accepted_target_is_published_to_unity_topics(self) -> None:
        from scratch_hci_bridge.node import ScratchHciBridge

        bridge = ScratchHciBridge.__new__(ScratchHciBridge)
        publishers = {
            name: _Publisher() for name in ("swing", "boom", "arm", "bucket")
        }
        bridge._unity_position_publishers = publishers
        bridge._unity_position_signs = {
            "swing": -1.0,
            "boom": 1.0,
            "arm": 1.0,
            "bucket": 1.0,
        }
        bridge.get_logger = lambda: _Logger()
        scheduled = SimpleNamespace(
            target=SimpleNamespace(
                joint_names=(
                    "swing_joint",
                    "boom_joint",
                    "arm_joint",
                    "bucket_joint",
                ),
                positions_rad=(0.1, 0.2, 0.3, 0.4),
            )
        )

        bridge._publish_unity_position_target(scheduled)

        self.assertEqual(publishers["swing"].values, [-0.1])
        self.assertEqual(publishers["boom"].values, [0.2])
        self.assertEqual(publishers["arm"].values, [0.3])
        self.assertEqual(publishers["bucket"].values, [0.4])

    def test_active_axis_mask_contains_only_scheduled_joints(self) -> None:
        from scratch_hci_bridge.node import ScratchHciBridge

        bridge = ScratchHciBridge.__new__(ScratchHciBridge)
        publisher = _Publisher()
        bridge._active_axis_mask_publisher = publisher
        scheduled = SimpleNamespace(
            commands=(
                SimpleNamespace(joint_name="boom"),
                SimpleNamespace(joint_name="bucket"),
                SimpleNamespace(joint_name="boom"),
            )
        )

        bridge._publish_active_axis_mask(scheduled)
        bridge._publish_active_axis_mask()

        self.assertEqual(publisher.values, [0b1010, 0])

    def test_pid_callback_queues_raw_value_and_captured_speed(self) -> None:
        from scratch_hci_bridge.node import ScratchHciBridge

        state = CommandState()
        self.assertTrue(state.apply_command("boom_speed", 5.0))
        self.assertTrue(state.apply_command("boom_angle", 100.0))
        scheduler = _Scheduler()
        bridge = ScratchHciBridge.__new__(ScratchHciBridge)
        bridge._state = state
        bridge._trajectory_scheduler = scheduler
        bridge.get_logger = lambda: _Logger()

        bridge._command_callback("boom_angle", 100.0)
        self.assertTrue(state.apply_command("boom_up", -10.0))
        bridge._command_callback("boom_up", -10.0)
        self.assertTrue(state.apply_command("swing_speed", 4.0))
        self.assertTrue(state.apply_command("swing_left", -30.0))
        bridge._command_callback("swing_left", -30.0)

        # CommandState keeps its clamped absolute state for direct publishing,
        # but the scheduler needs raw 100 so parallel absolute+relative mixing
        # is clamped only after the final target has been composed.
        self.assertEqual(state.snapshot().joints_deg["boom"], 48.0)
        self.assertEqual(
            scheduler.targets,
            [
                ("boom_angle", 100.0, 5.0),
                ("boom_up", -10.0, 5.0),
                ("swing_left", -30.0, 4.0),
            ],
        )

    def test_result_request_exception_cancels_without_releasing_gate(self) -> None:
        bridge, scheduler, _logger = self._bridge()
        scheduled = SimpleNamespace(
            target=SimpleNamespace(duration_sec=3.0),
            execution_mode=SimpleNamespace(value="serial"),
        )
        goal_handle = _GoalHandle(result_exception=RuntimeError("transport"))
        response_future = _Future(goal_handle)
        bridge._goal_response_future = response_future
        bridge._goal_response_scheduled = scheduled
        bridge._goal_submission_started_at = 10.0

        bridge._goal_response_callback(response_future, scheduled)

        self.assertIs(bridge._active_goal_handle, goal_handle)
        self.assertIs(bridge._active_goal_scheduled, scheduled)
        self.assertEqual(goal_handle.cancel_calls, 1)
        self.assertEqual(scheduler.completions, [])
        self.assertEqual(scheduler.send_failures, [])

    def test_goal_tolerance_failure_preserves_queue_and_releases_gate(self) -> None:
        bridge, scheduler, logger = self._bridge()
        scheduled = SimpleNamespace(execution_mode=SimpleNamespace(value="serial"))
        future = _Future(SimpleNamespace(
            status=6,
            result=SimpleNamespace(error_code=-5, error_string="goal_time_tolerance"),
        ))
        bridge._active_goal_scheduled = scheduled
        bridge._goal_result_future = future

        bridge._goal_result_callback(future, scheduled)

        self.assertEqual(scheduler.completions, [(scheduled, False)])
        self.assertTrue(scheduler.preserve_queued)
        self.assertIsNone(bridge._active_goal_scheduled)
        self.assertIn("continuing queued", logger.error_messages[-1])

    def test_result_response_exception_cancels_without_releasing_gate(self) -> None:
        bridge, scheduler, _logger = self._bridge()
        scheduled = SimpleNamespace(
            target=SimpleNamespace(duration_sec=3.0),
            execution_mode=SimpleNamespace(value="serial"),
        )
        goal_handle = _GoalHandle(result_future=_Future())
        result_future = _Future(exception=RuntimeError("transport"))
        bridge._active_goal_handle = goal_handle
        bridge._active_goal_scheduled = scheduled
        bridge._goal_result_future = result_future
        bridge._active_goal_deadline = 100.0

        bridge._goal_result_callback(result_future, scheduled)

        self.assertIs(bridge._active_goal_handle, goal_handle)
        self.assertIsNone(bridge._goal_result_future)
        self.assertEqual(goal_handle.cancel_calls, 1)
        self.assertEqual(scheduler.completions, [])

    def test_result_timeout_requests_cancel_only_once(self) -> None:
        bridge, scheduler, _logger = self._bridge()
        scheduled = SimpleNamespace(
            target=SimpleNamespace(duration_sec=3.0),
            execution_mode=SimpleNamespace(value="serial"),
        )
        goal_handle = _GoalHandle(result_future=_Future())
        bridge._active_goal_handle = goal_handle
        bridge._active_goal_scheduled = scheduled
        bridge._active_goal_deadline = 10.0

        bridge._check_action_timeouts(now=10.0)
        bridge._check_action_timeouts(now=20.0)

        self.assertEqual(goal_handle.cancel_calls, 1)
        self.assertIs(bridge._active_goal_handle, goal_handle)
        self.assertEqual(scheduler.completions, [])

    def test_goal_response_timeout_keeps_unknown_goal_blocked(self) -> None:
        bridge, scheduler, logger = self._bridge()
        scheduled = SimpleNamespace(
            target=SimpleNamespace(duration_sec=3.0),
            execution_mode=SimpleNamespace(value="serial"),
        )
        response_future = _Future()
        bridge._goal_response_future = response_future
        bridge._goal_response_scheduled = scheduled
        bridge._goal_submission_started_at = 1.0

        bridge._check_action_timeouts(now=6.0)
        bridge._check_action_timeouts(now=20.0)

        self.assertIs(bridge._goal_response_future, response_future)
        self.assertEqual(len(logger.error_messages), 1)
        self.assertEqual(scheduler.completions, [])
        self.assertEqual(scheduler.send_failures, [])

    def test_cancel_transport_error_is_retried_without_releasing_gate(self) -> None:
        bridge, scheduler, _logger = self._bridge()
        scheduled = SimpleNamespace(
            target=SimpleNamespace(duration_sec=3.0),
            execution_mode=SimpleNamespace(value="serial"),
        )
        goal_handle = _GoalHandle(
            result_future=_Future(),
            cancel_exceptions=[RuntimeError("transport")],
        )
        bridge._active_goal_handle = goal_handle
        bridge._active_goal_scheduled = scheduled
        bridge._active_goal_deadline = 0.0

        bridge._check_action_timeouts(now=1.0)
        self.assertFalse(bridge._active_cancel_requested)
        assert bridge._active_cancel_retry_at is not None
        bridge._check_action_timeouts(now=bridge._active_cancel_retry_at)

        self.assertEqual(goal_handle.cancel_calls, 2)
        self.assertTrue(bridge._active_cancel_requested)
        self.assertIs(bridge._active_goal_handle, goal_handle)
        self.assertEqual(scheduler.completions, [])

    def test_shutdown_resolves_done_response_before_cancelling(self) -> None:
        bridge, scheduler, _logger = self._bridge()
        scheduled = SimpleNamespace(
            target=SimpleNamespace(duration_sec=3.0),
            execution_mode=SimpleNamespace(value="serial"),
        )
        result_future = _Future()
        goal_handle = _GoalHandle(result_future=result_future)
        response_future = _Future(goal_handle)
        bridge._goal_response_future = response_future
        bridge._goal_response_scheduled = scheduled
        bridge._goal_submission_started_at = 10.0

        self.assertTrue(bridge._resolve_goal_response_during_shutdown(0.0))

        self.assertIs(bridge._active_goal_handle, goal_handle)
        self.assertIs(bridge._goal_result_future, result_future)
        self.assertEqual(scheduler.completions, [])


if __name__ == "__main__":
    unittest.main()

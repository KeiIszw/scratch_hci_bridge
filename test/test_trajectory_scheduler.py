import math
import unittest

from scratch_hci_bridge.command_state import (
    ExecutionMode,
    MAX_ROS_DURATION_SEC,
)
from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler


JOINT_NAMES = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]


def _feedback(
    scheduler: TrajectoryScheduler,
    *,
    swing: float = 0.0,
    boom: float = -80.0,
    arm: float = 140.0,
    bucket: float = 90.0,
    now: float | None = None,
) -> None:
    scheduler.update_from_joint_state(
        JOINT_NAMES,
        [
            math.radians(swing),
            math.radians(boom),
            math.radians(arm),
            math.radians(bucket),
        ],
        now=now,
    )


class TrajectorySchedulerTest(unittest.TestCase):
    def test_default_serial_waits_for_success_before_next_joint(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            initial_mode_command_wait_sec=0.5,
        )
        _feedback(scheduler)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        scheduler.update_http_target("arm_angle", 32.0, now=1.01)

        self.assertEqual(
            scheduler.selected_execution_mode, ExecutionMode.SERIAL
        )
        self.assertIsNone(scheduler.pending_snapshot(now=1.50))
        boom_goal = scheduler.pending_snapshot(now=1.51)

        self.assertIsNotNone(boom_goal)
        assert boom_goal is not None
        self.assertEqual(boom_goal.execution_mode, ExecutionMode.SERIAL)
        self.assertEqual(
            tuple(command.http_name for command in boom_goal.commands), ("boom_angle",)
        )
        self.assertTrue(
            math.isclose(
                boom_goal.target.positions_rad[1], math.radians(-50.0)
            )
        )
        self.assertTrue(
            math.isclose(
                boom_goal.target.positions_rad[2], math.radians(140.0)
            )
        )

        self.assertTrue(scheduler.mark_submitted(boom_goal))
        self.assertIsNone(scheduler.pending_snapshot(now=2.0))

        self.assertTrue(scheduler.mark_completed(boom_goal, successful=True))
        arm_goal = scheduler.pending_snapshot(now=2.0)

        self.assertIsNotNone(arm_goal)
        assert arm_goal is not None
        self.assertEqual(
            tuple(command.http_name for command in arm_goal.commands), ("arm_angle",)
        )
        self.assertTrue(
            math.isclose(
                arm_goal.target.positions_rad[1], math.radians(-80.0)
            )
        )
        self.assertTrue(
            math.isclose(arm_goal.target.positions_rad[2], math.radians(32.0))
        )

    def test_parallel_combines_joints_into_one_goal(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler)
        self.assertTrue(scheduler.set_execution_mode(ExecutionMode.PARALLEL, now=0.0))
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        scheduler.update_http_target("arm_angle", 32.0, now=1.01)

        self.assertIsNone(scheduler.pending_snapshot(now=1.059))
        goal = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(goal.execution_mode, ExecutionMode.PARALLEL)
        self.assertEqual(
            tuple(command.http_name for command in goal.commands),
            ("boom_angle", "arm_angle"),
        )
        self.assertEqual(
            tuple(target.http_name for target in goal.target.pending_http_targets),
            ("boom_angle", "arm_angle"),
        )
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(-50.0))
        )
        self.assertTrue(
            math.isclose(goal.target.positions_rad[2], math.radians(32.0))
        )

    def test_mode_command_is_an_ordered_batch_boundary(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=10.0)
        _feedback(scheduler)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        scheduler.set_execution_mode("serial", now=1.01)
        scheduler.update_http_target("arm_angle", 32.0, now=1.02)
        scheduler.set_execution_mode("parallel", now=1.03)

        first = scheduler.pending_snapshot(now=1.03)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertEqual(first.execution_mode, ExecutionMode.PARALLEL)
        self.assertEqual(first.commands[0].http_name, "boom_angle")
        self.assertTrue(scheduler.mark_submitted(first))
        _feedback(scheduler, boom=-50.0)
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        second = scheduler.pending_snapshot(now=1.03)
        self.assertIsNotNone(second)
        assert second is not None
        self.assertEqual(second.execution_mode, ExecutionMode.SERIAL)
        self.assertEqual(second.commands[0].http_name, "arm_angle")
        self.assertEqual(
            scheduler.selected_execution_mode, ExecutionMode.PARALLEL
        )

    def test_first_late_mode_command_adopts_open_batch(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            initial_mode_command_wait_sec=0.5,
        )
        _feedback(scheduler)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        scheduler.update_http_target("arm_angle", 32.0, now=1.01)

        scheduler.set_execution_mode("parallel", now=1.02)
        self.assertIsNone(scheduler.pending_snapshot(now=1.069))
        goal = scheduler.pending_snapshot(now=1.071)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(goal.execution_mode, ExecutionMode.PARALLEL)
        self.assertEqual(len(goal.commands), 2)

    def test_first_late_parallel_mode_keeps_batch_open_for_later_motion(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            initial_mode_command_wait_sec=0.5,
        )
        _feedback(scheduler)
        # 非同期処理の完了を待たない旧HCI版を再現する。最初のモード要求が
        # 2つの動作要求の間にサーバーへ届く。
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        scheduler.set_execution_mode("parallel", now=1.01)
        scheduler.update_http_target("arm_angle", 32.0, now=1.02)

        goal = scheduler.pending_snapshot(now=1.071)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(goal.execution_mode, ExecutionMode.PARALLEL)
        self.assertEqual(
            tuple(command.http_name for command in goal.commands),
            ("boom_angle", "arm_angle"),
        )

    def test_late_parallel_mode_partitions_accumulated_swing_commands(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            initial_mode_command_wait_sec=0.5,
        )
        _feedback(scheduler, swing=0.0)
        scheduler.update_http_target("swing_left", 100.0, now=1.0)
        scheduler.update_http_target("swing_left", 100.0, now=1.01)

        scheduler.set_execution_mode("parallel", now=1.02)
        first = scheduler.pending_snapshot(now=1.02)

        self.assertIsNotNone(first)
        assert first is not None
        self.assertEqual(first.execution_mode, ExecutionMode.PARALLEL)
        self.assertEqual(len(first.commands), 1)
        self.assertTrue(
            math.isclose(first.target.positions_rad[0], math.radians(100.0))
        )
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        second = scheduler.pending_snapshot(now=1.071)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertEqual(second.execution_mode, ExecutionMode.PARALLEL)
        self.assertEqual(len(second.commands), 1)
        self.assertTrue(
            math.isclose(second.target.positions_rad[0], math.radians(100.0))
        )

    def test_startup_mode_adoption_closes_after_first_default_batch(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            initial_mode_command_wait_sec=0.5,
        )
        _feedback(scheduler)
        scheduler.update_http_target("boom_angle", -60.0, now=1.0)
        first = scheduler.pending_snapshot(now=1.501)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertEqual(first.execution_mode, ExecutionMode.SERIAL)
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        scheduler.update_http_target("boom_angle", -50.0, now=2.0)
        scheduler.set_execution_mode("parallel", now=2.01)
        second = scheduler.pending_snapshot(now=2.01)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertEqual(second.execution_mode, ExecutionMode.SERIAL)
        self.assertEqual(
            scheduler.selected_execution_mode, ExecutionMode.PARALLEL
        )

    def test_expired_startup_batch_is_not_retroactively_changed(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            initial_mode_command_wait_sec=0.5,
        )
        _feedback(scheduler)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)

        scheduler.set_execution_mode("parallel", now=1.501)
        goal = scheduler.pending_snapshot(now=1.501)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(goal.execution_mode, ExecutionMode.SERIAL)
        self.assertEqual(
            scheduler.selected_execution_mode, ExecutionMode.PARALLEL
        )

    def test_serial_preserves_repeated_commands_for_the_same_joint(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_angle", -60.0, now=1.0)
        scheduler.update_http_target("boom_angle", -40.0, now=1.01)

        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertEqual(first.commands[0].degrees, -60.0)
        self.assertTrue(scheduler.mark_submitted(first))
        _feedback(scheduler, boom=-60.0)
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        second = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(second)
        assert second is not None
        self.assertEqual(second.commands[0].degrees, -40.0)
        self.assertTrue(
            math.isclose(second.target.positions_rad[1], math.radians(-40.0))
        )

    def test_parallel_uses_last_absolute_command_for_the_same_joint(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target("boom_angle", -60.0, now=1.0)
        scheduler.update_http_target("boom_angle", -40.0, now=1.01)

        goal = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(len(goal.commands), 2)
        self.assertEqual(len(goal.target.pending_http_targets), 1)
        self.assertEqual(goal.target.pending_http_targets[0].degrees, -40.0)
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(-40.0))
        )

    def test_serial_relative_commands_use_each_dispatch_position(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_up", -10.0, now=1.0)
        scheduler.update_http_target("boom_down", -5.0, now=1.01)

        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(first.commands[0].relative)
        self.assertEqual(first.commands[0].joint_name, "boom")
        self.assertEqual(first.commands[0].degrees, -10.0)
        self.assertTrue(
            math.isclose(first.target.positions_rad[1], math.radians(-60.0))
        )
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        second = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(second)
        assert second is not None
        self.assertEqual(second.commands[0].degrees, 5.0)
        self.assertTrue(
            math.isclose(second.target.positions_rad[1], math.radians(-45.0))
        )

    def test_relative_command_uses_latest_feedback_at_dispatch(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-80.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        _feedback(scheduler, boom=-70.0)

        goal = scheduler.pending_snapshot(now=1.051)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(-80.0))
        )

    def test_non_swing_relative_commands_follow_operasim_directions(self) -> None:
        cases = (
            ("boom_up", "boom", 1, -50.0, -60.0),
            ("boom_down", "boom", 1, -50.0, -40.0),
            ("arm_extend", "arm", 2, 100.0, 90.0),
            ("arm_retract", "arm", 2, 100.0, 110.0),
            ("bucket_dump", "bucket", 3, 50.0, 40.0),
            ("bucket_scoop", "bucket", 3, 50.0, 60.0),
        )

        for command, joint, index, current, expected in cases:
            with self.subTest(command=command):
                scheduler = TrajectoryScheduler(batch_wait_sec=0.0)
                _feedback(scheduler, **{joint: current})
                scheduler.set_execution_mode("serial", now=0.0)
                scheduler.update_http_target(command, 10.0, now=1.0)

                goal = scheduler.pending_snapshot(now=1.0)

                self.assertIsNotNone(goal)
                assert goal is not None
                self.assertTrue(
                    math.isclose(
                        goal.target.positions_rad[index], math.radians(expected)
                    )
                )

    def test_serial_relative_success_uses_target_with_small_error(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("boom_down", 5.0, now=1.01)
        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(scheduler.mark_submitted(first))

        _feedback(scheduler, boom=-59.5)
        self.assertTrue(scheduler.mark_completed(first, successful=True))
        second = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertTrue(
            math.isclose(second.target.positions_rad[1], math.radians(-55.0))
        )

    def test_serial_success_ignores_newer_feedback_outside_goal_tolerance(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            completion_tolerance_deg=0.75,
        )
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("boom_down", 5.0, now=1.01)
        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(scheduler.mark_submitted(first))

        # 実測との差が基準角の許容5度以内なら、成功目標を相対基準にする。
        _feedback(scheduler, boom=-55.0)
        self.assertTrue(scheduler.mark_completed(first, successful=True))
        second = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertTrue(
            math.isclose(second.target.positions_rad[1], math.radians(-55.0))
        )

    def test_success_quarantines_late_intermediate_feedback(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            completion_tolerance_deg=0.75,
        )
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("boom_down", 5.0, now=1.01)
        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        # 遅延した途中位置も実測として保存し、成功目標との差で基準を選ぶ。
        _feedback(scheduler, boom=-55.0, now=1.062)
        second = scheduler.pending_snapshot(now=1.062)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertTrue(
            math.isclose(second.target.positions_rad[1], math.radians(-55.0))
        )

    def test_fresh_in_tolerance_feedback_keeps_successful_target(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.05,
            completion_tolerance_deg=0.75,
        )
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("boom_down", 5.0, now=1.01)
        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        _feedback(scheduler, boom=-59.5, now=1.062)
        second = scheduler.pending_snapshot(now=1.062)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertTrue(
            math.isclose(second.target.positions_rad[1], math.radians(-55.0))
        )

    def test_parallel_relative_commands_share_origin_and_sum_offsets(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("boom_down", -3.0, now=1.01)

        goal = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(len(goal.commands), 2)
        self.assertEqual(len(goal.target.pending_http_targets), 1)
        self.assertEqual(
            goal.target.pending_http_targets[0].http_name, "boom_down"
        )
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(-57.0))
        )
        self.assertTrue(
            math.isclose(goal.target.pending_http_targets[0].degrees, -57.0)
        )

    def test_parallel_relative_commands_move_different_axes_together(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0, arm=100.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("arm_retract", 15.0, now=1.01)

        goal = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(
            tuple(command.http_name for command in goal.commands),
            ("boom_up", "arm_retract"),
        )
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(-60.0))
        )
        self.assertTrue(
            math.isclose(goal.target.positions_rad[2], math.radians(115.0))
        )

    def test_serial_relative_command_waits_then_uses_successful_hold_target(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0, arm=100.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("arm_retract", 15.0, now=1.01)
        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertEqual(first.commands[0].http_name, "boom_up")
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertIsNone(scheduler.pending_snapshot(now=2.0))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        _feedback(scheduler, boom=-60.0, arm=102.0)
        second = scheduler.pending_snapshot(now=2.0)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertEqual(second.commands[0].http_name, "arm_retract")
        self.assertTrue(
            math.isclose(second.target.positions_rad[2], math.radians(115.0))
        )

    def test_parallel_mixes_last_absolute_target_with_relative_sum(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-80.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        scheduler.update_http_target("boom_angle", -50.0, now=1.01)
        scheduler.update_http_target("boom_down", 3.0, now=1.02)

        goal = scheduler.pending_snapshot(now=1.071)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(-57.0))
        )

    def test_parallel_clamps_after_mixing_raw_absolute_and_relative(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-80.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target("boom_angle", 100.0, now=1.0)
        scheduler.update_http_target("boom_down", 20.0, now=1.01)

        goal = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(48.0))
        )

    def test_absolute_swing_keeps_existing_signed_input_clamp(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, swing=0.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("swing_angle", 270.0, now=1.0)

        goal = scheduler.pending_snapshot(now=1.051)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(
            math.isclose(goal.target.positions_rad[0], math.radians(-180.0))
        )

    def test_relative_swing_uses_unwrapped_physical_direction(self) -> None:
        left = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(left, swing=170.0)
        left.set_execution_mode("serial", now=0.0)
        self.assertTrue(
            left.update_http_target(
                "swing_left", -30.0, speed_limit_deg_s=10.0, now=1.0
            )
        )

        left_goal = left.pending_snapshot(now=1.051)

        self.assertIsNotNone(left_goal)
        assert left_goal is not None
        self.assertEqual(left_goal.commands[0].degrees, 30.0)
        self.assertTrue(
            math.isclose(
                left_goal.target.positions_rad[0], math.radians(200.0)
            )
        )
        self.assertTrue(math.isclose(left_goal.target.duration_sec, 3.0))

        right = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(right, swing=-170.0)
        right.set_execution_mode("serial", now=0.0)
        self.assertTrue(
            right.update_http_target(
                "swing_right", 30.0, speed_limit_deg_s=10.0, now=1.0
            )
        )

        right_goal = right.pending_snapshot(now=1.051)

        self.assertIsNotNone(right_goal)
        assert right_goal is not None
        self.assertEqual(right_goal.commands[0].degrees, -30.0)
        self.assertTrue(
            math.isclose(
                right_goal.target.positions_rad[0], math.radians(-200.0)
            )
        )
        self.assertTrue(math.isclose(right_goal.target.duration_sec, 3.0))

    def test_parallel_relative_swing_commands_are_split_into_safe_goals(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, swing=170.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        self.assertTrue(
            scheduler.update_http_target("swing_left", 100.0, now=1.0)
        )
        self.assertTrue(
            scheduler.update_http_target("swing_left", 100.0, now=1.01)
        )

        first = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(first)
        assert first is not None
        self.assertEqual(len(first.commands), 1)
        self.assertTrue(
            math.isclose(first.target.positions_rad[0], math.radians(270.0))
        )
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        second = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertEqual(len(second.commands), 1)
        self.assertTrue(
            math.isclose(second.target.positions_rad[0], math.radians(270.0))
        )

    def test_parallel_absolute_and_relative_swing_are_split(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, swing=170.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        self.assertTrue(
            scheduler.update_http_target("swing_angle", -170.0, now=1.0)
        )
        self.assertTrue(
            scheduler.update_http_target("swing_left", 30.0, now=1.01)
        )

        first = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(
            math.isclose(first.target.positions_rad[0], math.radians(190.0))
        )
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_completed(first, successful=True))

        second = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(second)
        assert second is not None
        self.assertTrue(
            math.isclose(second.target.positions_rad[0], math.radians(200.0))
        )

    def test_relative_swing_retry_preserves_unwrapped_target(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, swing=170.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target(
            "swing_left", 30.0, speed_limit_deg_s=10.0, now=1.0
        )
        scheduler.update_http_target(
            "swing_right", 10.0, speed_limit_deg_s=10.0, now=1.01
        )
        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_send_failed(first))

        _feedback(scheduler, swing=-175.0)
        retry = scheduler.pending_snapshot(now=1.07)

        self.assertIsNotNone(retry)
        assert retry is not None
        self.assertTrue(
            math.isclose(retry.target.positions_rad[0], math.radians(200.0))
        )
        self.assertTrue(math.isclose(retry.target.duration_sec, 1.5))
        self.assertTrue(scheduler.mark_submitted(retry))
        self.assertTrue(scheduler.mark_completed(retry, successful=True))

        successor = scheduler.pending_snapshot(now=1.07)
        self.assertIsNotNone(successor)
        assert successor is not None
        self.assertTrue(
            math.isclose(successor.target.positions_rad[0], math.radians(-185.0))
        )

    def test_relative_target_is_clamped_before_duration_is_calculated(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-80.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target(
            "boom_down", 200.0, speed_limit_deg_s=10.0, now=1.0
        )

        goal = scheduler.pending_snapshot(now=1.051)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(48.0))
        )
        self.assertTrue(math.isclose(goal.target.duration_sec, 12.8))

    def test_extreme_relative_sum_is_finite_and_clamped(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-80.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target("boom_down", 1e308, now=1.0)
        scheduler.update_http_target("boom_down", 1e308, now=1.01)

        goal = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(math.isfinite(goal.target.positions_rad[1]))
        self.assertTrue(math.isfinite(goal.target.duration_sec))
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(48.0))
        )

    def test_relative_command_at_joint_limit_has_positive_duration(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=48.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target(
            "boom_down", 10.0, speed_limit_deg_s=5.0, now=1.0
        )

        goal = scheduler.pending_snapshot(now=1.051)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(48.0))
        )
        self.assertGreater(goal.target.duration_sec, 0.0)

    def test_parallel_same_joint_uses_slowest_contributing_speed(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target(
            "boom_up", 20.0, speed_limit_deg_s=10.0, now=1.0
        )
        scheduler.update_http_target(
            "boom_down", 5.0, speed_limit_deg_s=5.0, now=1.01
        )

        goal = scheduler.pending_snapshot(now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(
            math.isclose(goal.target.positions_rad[1], math.radians(-65.0))
        )
        self.assertTrue(math.isclose(goal.target.duration_sec, 3.0))

    def test_parallel_relative_without_speed_keeps_base_duration(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target(
            "boom_up", 20.0, speed_limit_deg_s=10.0, now=1.0
        )
        scheduler.update_http_target("boom_down", 5.0, now=1.01)

        goal = scheduler.pending_snapshot(base_duration_sec=4.0, now=1.061)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(math.isclose(goal.target.duration_sec, 4.0))

    def test_bucket_relative_command_uses_bucket_feedback(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, bucket=90.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("bucket_scoop", 20.0, now=1.0)

        goal = scheduler.pending_snapshot(now=1.051)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(goal.commands[0].joint_name, "bucket")
        self.assertTrue(
            math.isclose(goal.target.positions_rad[3], math.radians(110.0))
        )

    def test_relative_retry_reuses_resolved_absolute_target(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-50.0, arm=140.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target(
            "boom_up", 10.0, speed_limit_deg_s=5.0, now=1.0
        )
        scheduler.update_http_target(
            "boom_down", 5.0, speed_limit_deg_s=5.0, now=1.01
        )
        first = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(math.isclose(first.target.duration_sec, 2.0))
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_send_failed(first))

        _feedback(scheduler, boom=-47.0, arm=100.0)
        retry = scheduler.pending_snapshot(now=1.07)

        self.assertIsNotNone(retry)
        assert retry is not None
        self.assertNotEqual(first.dispatch_id, retry.dispatch_id)
        self.assertTrue(
            math.isclose(retry.target.positions_rad[1], math.radians(-60.0))
        )
        self.assertTrue(
            math.isclose(retry.target.positions_rad[2], math.radians(100.0))
        )
        self.assertTrue(math.isclose(retry.target.duration_sec, 2.6))
        self.assertTrue(scheduler.mark_submitted(retry))
        self.assertTrue(scheduler.mark_completed(retry, successful=True))

        successor = scheduler.pending_snapshot(now=1.07)
        self.assertIsNotNone(successor)
        assert successor is not None
        self.assertTrue(
            math.isclose(successor.target.positions_rad[1], math.radians(-42.0))
        )
        self.assertTrue(
            math.isclose(successor.target.positions_rad[2], math.radians(100.0))
        )

    def test_tolerance_failure_keeps_successors_and_measured_position(self) -> None:
        for mode in ("serial", "parallel"):
            with self.subTest(mode=mode):
                scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
                _feedback(scheduler)
                scheduler.set_execution_mode(mode, now=0.0)
                scheduler.update_http_target("boom_angle", -50.0, now=1.0)
                first = scheduler.pending_snapshot(now=1.06)
                self.assertIsNotNone(first)
                self.assertTrue(scheduler.mark_submitted(first))
                scheduler.update_http_target("boom_up", 10.0, now=1.1)
                _feedback(scheduler, boom=-70.0)
                self.assertTrue(scheduler.mark_completed(
                    first, successful=False, preserve_queued=True,
                ))
                self.assertEqual(scheduler.queued_command_count(), 1)
                successor = scheduler.pending_snapshot(now=1.16)
                self.assertIsNotNone(successor)
                self.assertAlmostEqual(
                    successor.target.positions_rad[1], math.radians(-80.0),
                )

    def test_failed_goal_discards_successors_but_keeps_selected_mode(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        scheduler.update_http_target("arm_angle", 32.0, now=1.01)
        goal = scheduler.pending_snapshot(now=1.061)
        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(scheduler.mark_submitted(goal))

        self.assertTrue(scheduler.mark_completed(goal, successful=False))

        self.assertEqual(scheduler.queued_command_count(), 0)
        self.assertEqual(
            scheduler.selected_execution_mode, ExecutionMode.SERIAL
        )
        self.assertIsNone(scheduler.pending_snapshot(now=2.0))

    def test_send_failure_releases_gate_for_retry(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        first = scheduler.pending_snapshot(now=1.051)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(scheduler.mark_submitted(first))

        self.assertTrue(scheduler.mark_send_failed(first))
        retry = scheduler.pending_snapshot(now=1.06)

        self.assertIsNotNone(retry)
        assert retry is not None
        self.assertNotEqual(first.dispatch_id, retry.dispatch_id)
        self.assertEqual(first.commands, retry.commands)

    def test_goal_waits_for_all_joint_feedback(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)

        self.assertIsNone(scheduler.pending_snapshot(now=1.0))
        self.assertEqual(
            scheduler.missing_feedback_joint_names(),
            ("swing", "boom", "arm", "bucket"),
        )

        _feedback(scheduler)
        self.assertIsNotNone(scheduler.pending_snapshot(now=1.0))

    def test_goal_waits_for_fresh_joint_feedback(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.0,
            feedback_timeout_sec=0.5,
        )
        _feedback(scheduler, boom=-50.0, now=1.0)
        scheduler.set_execution_mode("serial", now=1.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.1)

        self.assertIsNone(scheduler.pending_snapshot(now=1.6))
        self.assertEqual(
            scheduler.missing_feedback_joint_names(now=1.6),
            ("swing", "boom", "arm", "bucket"),
        )

        _feedback(scheduler, now=1.61)
        self.assertIsNotNone(scheduler.pending_snapshot(now=1.61))

    def test_retry_waits_for_fresh_joint_feedback(self) -> None:
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.0,
            feedback_timeout_sec=0.5,
        )
        _feedback(scheduler, boom=-50.0, now=1.0)
        scheduler.set_execution_mode("serial", now=1.0)
        scheduler.update_http_target("boom_up", 10.0, now=1.0)
        first = scheduler.pending_snapshot(now=1.01)
        self.assertIsNotNone(first)
        assert first is not None
        self.assertTrue(scheduler.mark_submitted(first))
        self.assertTrue(scheduler.mark_send_failed(first))

        self.assertIsNone(scheduler.pending_snapshot(now=1.6))
        _feedback(scheduler, boom=-49.0, now=1.61)
        retry = scheduler.pending_snapshot(now=1.61)

        self.assertIsNotNone(retry)
        assert retry is not None
        self.assertTrue(
            math.isclose(retry.target.positions_rad[1], math.radians(-60.0))
        )

    def test_parallel_duration_uses_slowest_updated_joint(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-80.0, arm=140.0)
        scheduler.set_execution_mode("parallel", now=0.0)
        scheduler.update_http_target(
            "boom_angle", -70.0, speed_limit_deg_s=10.0, now=1.0
        )
        scheduler.update_http_target(
            "arm_angle", 100.0, speed_limit_deg_s=5.0, now=1.01
        )

        goal = scheduler.pending_snapshot(
            base_duration_sec=3.0,
            now=1.061,
        )

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertTrue(math.isclose(goal.target.duration_sec, 8.0))

    def test_speed_limit_is_captured_when_motion_command_arrives(self) -> None:
        scheduler = TrajectoryScheduler(batch_wait_sec=0.05)
        _feedback(scheduler, boom=-80.0)
        scheduler.set_execution_mode("serial", now=0.0)
        scheduler.update_http_target(
            "boom_angle", -50.0, speed_limit_deg_s=5.0, now=1.0
        )

        goal = scheduler.pending_snapshot(now=1.051)

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(goal.commands[0].speed_limit_deg_s, 5.0)
        self.assertTrue(math.isclose(goal.target.duration_sec, 6.0))

    def test_invalid_configuration_mode_and_target_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TrajectoryScheduler(batch_wait_sec=-1.0)
        with self.assertRaises(ValueError):
            TrajectoryScheduler(initial_mode_command_wait_sec=float("inf"))
        with self.assertRaises(ValueError):
            TrajectoryScheduler(completion_tolerance_deg=0.0)
        with self.assertRaises(ValueError):
            TrajectoryScheduler(feedback_timeout_sec=-1.0)

        scheduler = TrajectoryScheduler()
        self.assertFalse(scheduler.set_execution_mode("invalid"))
        self.assertFalse(scheduler.update_http_target("blade_angle", 1.0))
        self.assertFalse(scheduler.update_http_target("swing_left", 180.0))
        self.assertFalse(scheduler.update_http_target("end_dump", 1.0))
        self.assertFalse(scheduler.update_http_target("boom_angle", float("nan")))
        self.assertFalse(
            scheduler.update_http_target(
                "boom_angle", -50.0, speed_limit_deg_s=0.0
            )
        )
        self.assertFalse(
            scheduler.update_http_target(
                "boom_angle", -50.0, speed_limit_deg_s=5e-324
            )
        )
        self.assertFalse(
            scheduler.update_http_target(
                "boom_angle", -50.0, speed_limit_deg_s=1e-100
            )
        )
        _feedback(scheduler)
        scheduler.update_http_target("boom_angle", -50.0, now=1.0)
        with self.assertRaises(ValueError):
            scheduler.pending_snapshot(
                base_duration_sec=MAX_ROS_DURATION_SEC + 1.0,
                now=2.0,
            )


if __name__ == "__main__":
    unittest.main()

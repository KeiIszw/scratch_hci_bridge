import math
import unittest

from scratch_hci_bridge.trajectory_targets import TrajectoryTargets


JOINT_NAMES = ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"]


class TrajectoryTargetsTest(unittest.TestCase):
    def test_http_target_is_completed_from_latest_joint_feedback(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(JOINT_NAMES, [0.1, -0.2, 1.2, 0.7])
        self.assertTrue(targets.update_http_target("boom_angle", -30.0, now=10.0))

        snapshot = targets.pending_snapshot(debounce_sec=0.05, now=10.05)

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertEqual(
            snapshot.joint_names,
            ("swing_joint", "boom_joint", "arm_joint", "bucket_joint"),
        )
        self.assertTrue(math.isclose(snapshot.positions_rad[0], 0.1))
        self.assertTrue(math.isclose(snapshot.positions_rad[1], math.radians(-30.0)))
        self.assertTrue(math.isclose(snapshot.positions_rad[2], 1.2))
        self.assertTrue(math.isclose(snapshot.positions_rad[3], 0.7))

    def test_previous_http_target_is_not_reused_by_the_next_goal(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(JOINT_NAMES, [0.0, -1.4, 2.35, 1.55])
        targets.update_http_target("arm_angle", 50.0, now=1.0)
        arm_snapshot = targets.pending_snapshot(debounce_sec=0.0, now=1.0)
        self.assertIsNotNone(arm_snapshot)
        assert arm_snapshot is not None
        targets.mark_submitted(arm_snapshot)

        targets.update_from_joint_state(JOINT_NAMES, [0.01, -1.39, 2.36, 1.54])
        targets.update_http_target("boom_angle", -30.0, now=2.0)
        boom_snapshot = targets.pending_snapshot(debounce_sec=0.0, now=2.0)

        self.assertIsNotNone(boom_snapshot)
        assert boom_snapshot is not None
        self.assertTrue(
            math.isclose(boom_snapshot.positions_rad[1], math.radians(-30.0))
        )
        self.assertTrue(math.isclose(boom_snapshot.positions_rad[2], 2.36))

    def test_multiple_http_targets_in_one_batch_override_feedback(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(JOINT_NAMES, [0.1, -1.4, 2.3, 1.5])
        targets.update_http_target("boom_angle", -35.0, now=3.0)
        targets.update_http_target("arm_angle", 100.0, now=3.01)

        snapshot = targets.pending_snapshot(debounce_sec=0.05, now=3.061)

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertTrue(math.isclose(snapshot.positions_rad[0], 0.1))
        self.assertTrue(math.isclose(snapshot.positions_rad[1], math.radians(-35.0)))
        self.assertTrue(math.isclose(snapshot.positions_rad[2], math.radians(100.0)))
        self.assertTrue(math.isclose(snapshot.positions_rad[3], 1.5))

    def test_goal_waits_for_debounce_and_missing_feedback(self) -> None:
        targets = TrajectoryTargets()
        targets.update_http_target("boom_angle", -30.0, now=4.0)

        self.assertIsNone(targets.pending_snapshot(debounce_sec=0.05, now=4.04))
        self.assertEqual(
            targets.missing_feedback_joint_names(),
            ("swing", "boom", "arm", "bucket"),
        )

        targets.update_from_joint_state(JOINT_NAMES, [0.0, -1.0, 1.0, 0.5])
        self.assertIsNotNone(
            targets.pending_snapshot(debounce_sec=0.05, now=4.051)
        )

    def test_speed_limit_extends_duration_beyond_fixed_duration(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(
            JOINT_NAMES, [0.0, math.radians(-80.0), 2.0, 1.0]
        )
        targets.update_http_target("boom_angle", -30.0, now=5.0)

        snapshot = targets.pending_snapshot(
            debounce_sec=0.0,
            base_duration_sec=3.0,
            speed_limits_deg_s={"boom": 5.0},
            now=5.0,
        )

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertTrue(math.isclose(snapshot.duration_sec, 10.0))

    def test_speed_limit_duration_has_no_fixed_minimum(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(
            JOINT_NAMES, [0.0, math.radians(-80.0), 2.0, 1.0]
        )
        targets.update_http_target("boom_angle", -30.0, now=5.0)

        snapshot = targets.pending_snapshot(
            debounce_sec=0.0,
            base_duration_sec=3.0,
            speed_limits_deg_s={"boom": 20.0},
            now=5.0,
        )

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertTrue(math.isclose(snapshot.duration_sec, 2.5))

    def test_joint_without_speed_limit_uses_fixed_duration(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(
            JOINT_NAMES, [0.0, math.radians(-80.0), math.radians(140.0), 1.0]
        )
        targets.update_http_target("boom_angle", -70.0, now=5.0)
        targets.update_http_target("arm_angle", 130.0, now=5.01)

        snapshot = targets.pending_snapshot(
            debounce_sec=0.0,
            base_duration_sec=3.0,
            speed_limits_deg_s={"boom": 100.0},
            now=5.01,
        )

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertTrue(math.isclose(snapshot.duration_sec, 3.0))

    def test_slowest_updated_joint_determines_shared_duration(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(
            JOINT_NAMES,
            [0.0, math.radians(-80.0), math.radians(140.0), 1.0],
        )
        targets.update_http_target("boom_angle", -70.0, now=5.0)
        targets.update_http_target("arm_angle", 100.0, now=5.01)

        snapshot = targets.pending_snapshot(
            debounce_sec=0.0,
            base_duration_sec=3.0,
            speed_limits_deg_s={"boom": 10.0, "arm": 5.0},
            now=5.01,
        )

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertTrue(math.isclose(snapshot.duration_sec, 8.0))

    def test_swing_speed_uses_shortest_angular_distance(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(
            JOINT_NAMES, [math.radians(170.0), -1.0, 2.0, 1.0]
        )
        targets.update_http_target("swing_angle", -170.0, now=5.0)

        snapshot = targets.pending_snapshot(
            debounce_sec=0.0,
            base_duration_sec=3.0,
            speed_limits_deg_s={"swing": 5.0},
            now=5.0,
        )

        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertTrue(
            math.isclose(snapshot.positions_rad[0], math.radians(190.0))
        )
        self.assertTrue(math.isclose(snapshot.duration_sec, 4.0))

    def test_submitted_goal_is_not_republished(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(JOINT_NAMES, [0.0, -1.0, 2.0, 1.0])
        targets.update_http_target("boom_angle", -30.0, now=5.0)

        snapshot = targets.pending_snapshot(debounce_sec=0.0, now=5.0)
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        targets.mark_submitted(snapshot)

        self.assertIsNone(targets.pending_snapshot(debounce_sec=0.0, now=6.0))

    def test_send_failure_restores_only_targets_without_newer_updates(self) -> None:
        targets = TrajectoryTargets()
        targets.update_from_joint_state(JOINT_NAMES, [0.0, -1.0, 2.0, 1.0])
        targets.update_http_target("boom_angle", -30.0, now=7.0)
        targets.update_http_target("arm_angle", 100.0, now=7.0)
        snapshot = targets.pending_snapshot(debounce_sec=0.0, now=7.0)
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        targets.mark_submitted(snapshot)

        targets.update_http_target("arm_angle", 90.0, now=8.0)
        targets.restore_after_send_failure(snapshot, now=8.0)
        restored = targets.pending_snapshot(debounce_sec=0.0, now=8.0)

        self.assertIsNotNone(restored)
        assert restored is not None
        self.assertTrue(math.isclose(restored.positions_rad[1], math.radians(-30.0)))
        self.assertTrue(math.isclose(restored.positions_rad[2], math.radians(90.0)))

    def test_unknown_and_non_finite_targets_are_rejected(self) -> None:
        targets = TrajectoryTargets()
        self.assertFalse(targets.update_http_target("blade_angle", 1.0))
        self.assertFalse(targets.update_http_target("boom_angle", float("nan")))


if __name__ == "__main__":
    unittest.main()

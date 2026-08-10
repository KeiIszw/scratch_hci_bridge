import math
import unittest

from scratch_hci_bridge.command_state import (
    CommandState,
    JOINT_NAMES,
    SUPPORTED_COMMANDS,
)


class CommandStateTest(unittest.TestCase):
    def test_joint_command_sets_absolute_angle_and_is_limited(self) -> None:
        state = CommandState()

        self.assertTrue(state.apply_command("boom", -30.0))
        self.assertEqual(state.next_frame().joints_deg["boom"], -30.0)

        self.assertTrue(state.apply_command("boom", 1000.0))
        self.assertEqual(state.next_frame().joints_deg["boom"], 48.0)

    def test_body_is_not_angle_limited(self) -> None:
        state = CommandState()

        self.assertTrue(state.apply_command("body", 720.0))
        self.assertEqual(state.next_frame().joints_deg["body"], 720.0)

    def test_crawler_speeds_are_converted_to_twist(self) -> None:
        state = CommandState(drive_linear_scale=2.0, drive_angular_scale=3.0)

        self.assertTrue(state.apply_command("left_crawler_speed", 0.2))
        self.assertTrue(state.apply_command("right_crawler_speed", 0.6))
        frame = state.next_frame()

        self.assertTrue(math.isclose(frame.linear_velocity, 0.8))
        self.assertTrue(math.isclose(frame.angular_velocity, 0.6))

        state.stop_drive()
        stopped = state.next_frame()
        self.assertEqual(stopped.linear_velocity, 0.0)
        self.assertEqual(stopped.angular_velocity, 0.0)

    def test_unknown_and_non_finite_commands_are_rejected(self) -> None:
        state = CommandState()

        self.assertFalse(state.apply_command("unknown", 1.0))
        self.assertFalse(state.apply_command("boom_speed", 1.0))
        self.assertFalse(state.apply_command("boom", float("nan")))
        self.assertFalse(state.apply_command("boom", float("inf")))

    def test_supported_commands_are_absolute_joints_and_crawlers(self) -> None:
        state = CommandState()
        expected_joints = (
            "blade",
            "body",
            "swing",
            "boom",
            "arm",
            "bucket",
            "thumb",
        )
        self.assertEqual(JOINT_NAMES, expected_joints)
        expected_commands = {
            *expected_joints,
            "left_crawler_speed",
            "right_crawler_speed",
        }
        self.assertEqual(SUPPORTED_COMMANDS, expected_commands)

        joint_commands = {
            "blade": 1.0,
            "body": 2.0,
            "swing": 3.0,
            "boom": 4.0,
            "arm": 40.0,
            "bucket": 6.0,
            "thumb": 7.0,
        }
        for joint, angle in joint_commands.items():
            self.assertTrue(state.apply_command(joint, angle))
        self.assertTrue(state.apply_command("left_crawler_speed", -0.5))
        self.assertTrue(state.apply_command("right_crawler_speed", 0.5))

        snapshot = state.snapshot()
        for joint, angle in joint_commands.items():
            self.assertEqual(snapshot.joints_deg[joint], angle)
        self.assertEqual(snapshot.left_crawler_speed, -0.5)
        self.assertEqual(snapshot.right_crawler_speed, 0.5)

    def test_invalid_configuration_and_drive_overflow_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            CommandState(drive_linear_scale=float("inf"))

        state = CommandState(
            drive_linear_scale=1e308,
            drive_angular_scale=1e308,
        )
        self.assertTrue(state.apply_command("left_crawler_speed", 2.0))
        self.assertFalse(state.apply_command("right_crawler_speed", 2.0))
        frame = state.next_frame()
        self.assertTrue(math.isfinite(frame.linear_velocity))
        self.assertTrue(math.isfinite(frame.angular_velocity))


if __name__ == "__main__":
    unittest.main()

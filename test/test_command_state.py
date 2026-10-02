import math
import unittest

from scratch_hci_bridge.command_state import (
    CommandState,
    ExecutionMode,
    JOINT_NAMES,
    MAX_ROS_DURATION_SEC,
    RELATIVE_JOINT_COMMANDS,
    SUPPORTED_COMMANDS,
    is_ros_duration_safe,
)


class CommandStateTest(unittest.TestCase):
    def test_all_absolute_commands_require_angle_suffix(self) -> None:
        state = CommandState()
        for joint in JOINT_NAMES:
            with self.subTest(joint=joint):
                self.assertFalse(state.apply_command(joint, 40.0))
                self.assertTrue(state.apply_command(f"{joint}_angle", 40.0))
                self.assertNotIn(joint, SUPPORTED_COMMANDS)
                self.assertIn(f"{joint}_angle", SUPPORTED_COMMANDS)

    def test_default_speeds_can_be_overridden_by_http_commands(self) -> None:
        state = CommandState(default_joint_speeds_deg_s={"boom": 5.0, "arm": 2.0})
        self.assertEqual(state.snapshot().joint_speed_limits_deg_s["boom"], 5.0)
        self.assertEqual(state.snapshot().joint_speed_limits_deg_s["arm"], 2.0)
        self.assertIsNone(state.snapshot().joint_speed_limits_deg_s["swing"])
        self.assertTrue(state.apply_command("boom_speed", 8.0))
        self.assertEqual(state.snapshot().joint_speed_limits_deg_s["boom"], 8.0)
        self.assertTrue(state.apply_command("boom_speed", 0.0))
        self.assertIsNone(state.snapshot().joint_speed_limits_deg_s["boom"])

    def test_invalid_default_speeds_are_rejected(self) -> None:
        for speed in (-1.0, math.nan, math.inf, 1e-300):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                CommandState(default_joint_speeds_deg_s={"boom": speed})

    def test_joint_command_sets_absolute_angle_and_is_limited(self) -> None:
        state = CommandState()

        self.assertTrue(state.apply_command("boom_angle", -30.0))
        self.assertEqual(state.next_frame().joints_deg["boom"], -30.0)

        self.assertTrue(state.apply_command("boom_angle", 1000.0))
        self.assertEqual(state.next_frame().joints_deg["boom"], 48.0)

    def test_body_is_not_angle_limited(self) -> None:
        state = CommandState()

        self.assertTrue(state.apply_command("body_angle", 720.0))
        self.assertEqual(state.next_frame().joints_deg["body"], 720.0)

    def test_swing_uses_tb20e_continuous_angle_range(self) -> None:
        state = CommandState()

        self.assertTrue(state.apply_command("swing_angle", 270.0))
        self.assertEqual(state.next_frame().joints_deg["swing"], 180.0)
        self.assertTrue(state.apply_command("swing_angle", -270.0))
        self.assertEqual(state.next_frame().joints_deg["swing"], -180.0)

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
        self.assertFalse(state.apply_command("boom_speed", -1.0))
        self.assertFalse(state.apply_command("boom_speed", 5e-324))
        self.assertFalse(state.apply_command("boom_speed", 1e-100))
        self.assertFalse(state.apply_command("boom_angle", float("nan")))
        self.assertFalse(state.apply_command("boom_angle", float("inf")))

    def test_pid_joint_speed_limit_can_be_set_and_disabled(self) -> None:
        state = CommandState()

        self.assertTrue(state.apply_command("boom_speed", 5.0))
        self.assertEqual(state.snapshot().joint_speed_limits_deg_s["boom"], 5.0)
        self.assertTrue(state.apply_command("boom_speed", 0.0))
        self.assertIsNone(state.snapshot().joint_speed_limits_deg_s["boom"])

    def test_ros_duration_boundary_is_validated(self) -> None:
        self.assertTrue(is_ros_duration_safe(MAX_ROS_DURATION_SEC))
        self.assertFalse(is_ros_duration_safe(MAX_ROS_DURATION_SEC + 1.0))

    def test_execution_mode_defaults_to_serial_and_ignores_dummy_value(self) -> None:
        state = CommandState()

        self.assertEqual(state.snapshot().execution_mode, ExecutionMode.SERIAL)
        self.assertTrue(state.apply_command("parallel", 123.0))
        self.assertEqual(state.snapshot().execution_mode, ExecutionMode.PARALLEL)
        self.assertTrue(state.apply_command("serial", -456.0))
        self.assertEqual(state.snapshot().execution_mode, ExecutionMode.SERIAL)

    def test_relative_joint_command_is_accepted_without_precomputing_target(self) -> None:
        state = CommandState()

        self.assertTrue(state.apply_command("boom_up", -10.0))
        self.assertEqual(state.snapshot().joints_deg["boom"], -83.0)
        actual_commands = {
            name: (spec.joint, spec.direction)
            for name, spec in RELATIVE_JOINT_COMMANDS.items()
        }
        self.assertEqual(
            actual_commands,
            {
                "swing_left": ("swing", 1.0),
                "swing_right": ("swing", -1.0),
                "boom_up": ("boom", -1.0),
                "boom_down": ("boom", 1.0),
                "arm_extend": ("arm", -1.0),
                "arm_retract": ("arm", 1.0),
                "bucket_dump": ("bucket", -1.0),
                "bucket_scoop": ("bucket", 1.0),
            },
        )

        self.assertTrue(state.apply_command("swing_left", -179.999))
        self.assertTrue(state.apply_command("swing_right", 179.999))
        self.assertFalse(state.apply_command("swing_left", 180.0))
        self.assertFalse(state.apply_command("swing_right", -181.0))
        self.assertFalse(state.apply_command("end_dump", 10.0))
        self.assertFalse(state.apply_command("end_scoop", 10.0))

    def test_supported_commands_include_absolute_relative_and_crawlers(self) -> None:
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
            *(f"{joint}_angle" for joint in expected_joints),
            "swing_speed",
            "boom_speed",
            "arm_speed",
            "bucket_speed",
            "serial",
            "parallel",
            "swing_left",
            "swing_right",
            "boom_up",
            "boom_down",
            "arm_extend",
            "arm_retract",
            "bucket_dump",
            "bucket_scoop",
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
            self.assertTrue(state.apply_command(f"{joint}_angle", angle))
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

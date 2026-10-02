from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("excavator", default_value="tb20e"),
            DeclareLaunchArgument("publish_hz", default_value="50.0"),
            DeclareLaunchArgument("http_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("http_port", default_value="8899"),
            DeclareLaunchArgument("drive_linear_scale", default_value="1.0"),
            DeclareLaunchArgument("drive_angular_scale", default_value="1.0"),
            DeclareLaunchArgument(
                "trajectory_action",
                default_value="/tb20e_controller/follow_joint_trajectory",
            ),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            *[
                DeclareLaunchArgument(f"{joint}_default_speed_deg_s", default_value="0.0")
                for joint in ("swing", "boom", "arm", "bucket")
            ],
            DeclareLaunchArgument("trajectory_duration_sec", default_value="3.0"),
            DeclareLaunchArgument("trajectory_debounce_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "initial_mode_command_wait_sec", default_value="0.5"
            ),
            DeclareLaunchArgument("joint_state_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "trajectory_goal_tolerance_deg", default_value="0.75"
            ),
            DeclareLaunchArgument(
                "trajectory_stopped_velocity_deg_s", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "trajectory_goal_response_timeout_sec", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "trajectory_result_timeout_margin_sec", default_value="35.0"
            ),
            DeclareLaunchArgument(
                "trajectory_shutdown_cancel_timeout_sec", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "unity_position_output_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "unity_position_command_prefix", default_value="/TB20e"
            ),
            DeclareLaunchArgument(
                "direct_joint_command_prefix", default_value="/TB20e"
            ),
            DeclareLaunchArgument("active_axis_mask_topic", default_value=""),
            DeclareLaunchArgument("swing_unity_position_sign", default_value="-1.0"),
            DeclareLaunchArgument("boom_unity_position_sign", default_value="1.0"),
            DeclareLaunchArgument("arm_unity_position_sign", default_value="1.0"),
            DeclareLaunchArgument("bucket_unity_position_sign", default_value="1.0"),
            Node(
                package="scratch_hci_bridge",
                executable="scratch_hci_bridge",
                name="scratch_hci_bridge",
                output="screen",
                parameters=[
                    {
                        "excavator": LaunchConfiguration("excavator"),
                        "publish_hz": ParameterValue(
                            LaunchConfiguration("publish_hz"), value_type=float
                        ),
                        "http_host": LaunchConfiguration("http_host"),
                        "http_port": ParameterValue(
                            LaunchConfiguration("http_port"), value_type=int
                        ),
                        "drive_linear_scale": ParameterValue(
                            LaunchConfiguration("drive_linear_scale"), value_type=float
                        ),
                        "drive_angular_scale": ParameterValue(
                            LaunchConfiguration("drive_angular_scale"), value_type=float
                        ),
                        "trajectory_action": LaunchConfiguration("trajectory_action"),
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        **{
                            f"{joint}_default_speed_deg_s": ParameterValue(
                                LaunchConfiguration(f"{joint}_default_speed_deg_s"),
                                value_type=float,
                            )
                            for joint in ("swing", "boom", "arm", "bucket")
                        },
                        "trajectory_duration_sec": ParameterValue(
                            LaunchConfiguration("trajectory_duration_sec"),
                            value_type=float,
                        ),
                        "trajectory_debounce_sec": ParameterValue(
                            LaunchConfiguration("trajectory_debounce_sec"),
                            value_type=float,
                        ),
                        "initial_mode_command_wait_sec": ParameterValue(
                            LaunchConfiguration("initial_mode_command_wait_sec"),
                            value_type=float,
                        ),
                        "joint_state_timeout_sec": ParameterValue(
                            LaunchConfiguration("joint_state_timeout_sec"),
                            value_type=float,
                        ),
                        "trajectory_goal_tolerance_deg": ParameterValue(
                            LaunchConfiguration("trajectory_goal_tolerance_deg"),
                            value_type=float,
                        ),
                        "trajectory_stopped_velocity_deg_s": ParameterValue(
                            LaunchConfiguration(
                                "trajectory_stopped_velocity_deg_s"
                            ),
                            value_type=float,
                        ),
                        "trajectory_goal_response_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "trajectory_goal_response_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "trajectory_result_timeout_margin_sec": ParameterValue(
                            LaunchConfiguration(
                                "trajectory_result_timeout_margin_sec"
                            ),
                            value_type=float,
                        ),
                        "trajectory_shutdown_cancel_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "trajectory_shutdown_cancel_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "unity_position_output_enabled": ParameterValue(
                            LaunchConfiguration("unity_position_output_enabled"),
                            value_type=bool,
                        ),
                        "unity_position_command_prefix": LaunchConfiguration(
                            "unity_position_command_prefix"
                        ),
                        "direct_joint_command_prefix": LaunchConfiguration(
                            "direct_joint_command_prefix"
                        ),
                        "active_axis_mask_topic": LaunchConfiguration(
                            "active_axis_mask_topic"
                        ),
                        "swing_unity_position_sign": ParameterValue(
                            LaunchConfiguration("swing_unity_position_sign"),
                            value_type=float,
                        ),
                        "boom_unity_position_sign": ParameterValue(
                            LaunchConfiguration("boom_unity_position_sign"),
                            value_type=float,
                        ),
                        "arm_unity_position_sign": ParameterValue(
                            LaunchConfiguration("arm_unity_position_sign"),
                            value_type=float,
                        ),
                        "bucket_unity_position_sign": ParameterValue(
                            LaunchConfiguration("bucket_unity_position_sign"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )

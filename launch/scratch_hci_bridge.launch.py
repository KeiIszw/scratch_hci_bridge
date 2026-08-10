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
                    }
                ],
            ),
        ]
    )

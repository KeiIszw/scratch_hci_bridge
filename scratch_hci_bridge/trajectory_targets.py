"""関節のHTTP名・ROS名と、タイムラインから送る最終目標を定義する。"""
from dataclasses import dataclass

PID_JOINTS = (
    ('swing', 'swing_joint'), ('boom', 'boom_joint'),
    ('arm', 'arm_joint'), ('bucket', 'bucket_joint'),
)
PID_HTTP_TO_ROS = dict(PID_JOINTS)
PID_ROS_TO_HTTP = {ros_name: http_name for http_name, ros_name in PID_JOINTS}


@dataclass(frozen=True)
class TrajectoryTargetSnapshot:
    revision: int
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    duration_sec: float

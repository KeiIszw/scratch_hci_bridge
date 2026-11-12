"""ROS通信を起動せず、受理・置換・結果の順序を操作するダミー境界を提供する。"""
import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
from scratch_hci_bridge.node import ScratchHciBridge
import test_node_command_callback as callbacks
from test_node_command_callback import _Future, _GoalHandle
import test_sync_safety as safety


def _sending_bridge():
    bridge = safety.NodeSafetyTest().bridge()
    del bridge._send_pending_trajectory
    bridge._trajectory_scheduler = safety.TrajectoryScheduler(
        feedback_timeout_sec=0
    )
    bridge._trajectory_scheduler.update_from_joint_state(
        safety.NAMES, [0.0, math.radians(-50), 2.0, 1.0], now=0
    )
    bridge._trajectory_duration_sec = 1.0
    bridge._trajectory_goal_tolerance_rad = math.radians(0.75)
    bridge._trajectory_stopped_velocity_rad_s = math.radians(0.5)
    bridge._trajectory_client = Mock()
    bridge._trajectory_client.server_is_ready.return_value = True
    bridge._trajectory_client.send_goal_async.side_effect = lambda goal: callbacks._Future()
    return bridge


class _BeforeRosInterfaces(Exception):
    pass


def _initialize_without_ros(bridge, overrides):
    parameters = {}

    def declare(name, default):
        parameters[name] = overrides.get(name, default)

    # 検証より後のCommandStateで打ち切り、publisherやHTTPサーバーを作らない。
    with patch('scratch_hci_bridge.node.Node.__init__', return_value=None), \
         patch.object(ScratchHciBridge, 'declare_parameter', side_effect=declare), \
         patch.object(ScratchHciBridge, 'get_parameter',
                      side_effect=lambda name: SimpleNamespace(value=parameters[name])), \
         patch('scratch_hci_bridge.node.CommandState', side_effect=_BeforeRosInterfaces):
        bridge.__init__()


def queue(s, delta=10, command='boom_down', speed=None, now=0):
    s.barrier(now=now)
    assert s.update_http_target(command, delta, speed_limit_deg_s=speed, base_duration_sec=1, now=now)


class Harness:
    def __init__(self):
        self.bridge = _sending_bridge()
        self.scheduler = self.bridge._trajectory_scheduler
        self.sent = []
        self.bridge._trajectory_client.send_goal_async.side_effect = self.send

    def send(self, goal):
        response = _Future()
        handle = _GoalHandle(result_future=_Future())
        self.sent.append((goal, response, handle))
        return response

    def tick(self, now):
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=now):
            self.bridge._on_timer()

    def accept(self, index, now=0, accepted=True):
        _, response, handle = self.sent[index]
        handle.accepted = accepted
        response._result = handle
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=now):
            for callback in response.callbacks:
                callback(response)
        return handle

    def result(self, index, status=GoalStatus.STATUS_SUCCEEDED, code=0, now=1):
        future = self.sent[index][2]._result_future
        future._result = SimpleNamespace(status=status,
            result=SimpleNamespace(error_code=code, error_string='dummy'))
        with patch('scratch_hci_bridge.node.time.monotonic', return_value=now):
            for callback in future.callbacks:
                callback(future)



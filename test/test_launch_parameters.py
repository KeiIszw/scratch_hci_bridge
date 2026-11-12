"""プロセスを起動せず、途中ゴールの失敗方針と猶予の型付き設定を確認する。"""
import ast
import importlib.util
import logging
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def test_obsolete_relative_basis_fallback_parameter_is_removed():
    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('bridge_launch', root/'launch/scratch_hci_bridge.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch('launch.logging.get_logger', return_value=logging.getLogger(__name__)):
        description = module.generate_launch_description()
    assert 'relative_basis_max_error_deg' not in {
        action.name for action in description.entities if isinstance(action, DeclareLaunchArgument)
    }
    # launch経由と直接初期化で、廃止した実測フォールバックを復活させない。
    assert 'relative_basis_max_error_deg' not in (root/'scratch_hci_bridge/node.py').read_text()


@pytest.mark.parametrize('override, expected', [({}, True), ({'catch_up_axes_in_mask': 'false'}, False)])
def test_catch_up_axes_launch_parameter(override, expected):
    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('bridge_launch', root/'launch/scratch_hci_bridge.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with patch('launch.logging.get_logger', return_value=logging.getLogger(__name__)):
        context = LaunchContext()
        context.launch_configurations.update(override)
        description = module.generate_launch_description()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        node = next(action for action in description.entities if isinstance(action, Node))
        parameters = evaluate_parameters(context, node._Node__parameters)[0]
    assert parameters['catch_up_axes_in_mask'] is expected


def test_ground_slow_launch_parameters_are_typed_and_overridable():
    """launchからの文字列が浮動小数の減速パラメータとしてノードへ届くことを確認する。"""
    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('bridge_launch', root/'launch/scratch_hci_bridge.launch.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    with patch('launch.logging.get_logger', return_value=logging.getLogger(__name__)):
        context = LaunchContext()
        context.launch_configurations.update(ground_slow_distance_m='0.15',ground_slow_speed_deg_s='1.5',
            ground_stall_sec='0.7',ground_stall_epsilon_deg='0.08')
        description = module.generate_launch_description()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument): action.execute(context)
        node = next(action for action in description.entities if isinstance(action, Node))
        parameters = evaluate_parameters(context,node._Node__parameters)[0]
    assert parameters['ground_slow_distance_m']==.15
    assert parameters['ground_slow_speed_deg_s']==1.5

    assert parameters['ground_stall_sec']==.7
    assert parameters['ground_stall_epsilon_deg']==.08

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


def test_intermediate_tolerance_parameter_defaults_and_override():
    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location('bridge_launch', root/'launch/scratch_hci_bridge.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for override, expected in (({}, False), ({'intermediate_tolerance_failure_fatal': 'true'}, True)):
        with patch('launch.logging.get_logger', return_value=logging.getLogger(__name__)):
            context = LaunchContext()
            context.launch_configurations.update(override)
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            node = next(action for action in description.entities if isinstance(action, Node))
            parameters = evaluate_parameters(context, node._Node__parameters)[0]
            assert parameters['intermediate_tolerance_failure_fatal'] is expected
            assert parameters['batch_open_timeout_sec'] == 5.0
    # Nodeを初期化せず、単体起動時の宣言もlaunchと同じ既定値であることを確認する。
    tree = ast.parse((root/'scratch_hci_bridge/node.py').read_text())
    declaration = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and isinstance(n.func, ast.Attribute) and n.func.attr == 'declare_parameter'
                       and n.args and isinstance(n.args[0], ast.Constant)
                       and n.args[0].value == 'intermediate_tolerance_failure_fatal')
    assert ast.literal_eval(declaration.args[1]) is False


@pytest.mark.parametrize('override, expected', [({}, 0.5),
    ({'intermediate_goal_time_tolerance_sec': '1.25'}, 1.25),
    ({'intermediate_goal_time_tolerance_sec': '0'}, 0.0)])
def test_intermediate_goal_time_tolerance_launch_parameter(override, expected):
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
    assert parameters['intermediate_goal_time_tolerance_sec'] == expected
    assert type(parameters['intermediate_goal_time_tolerance_sec']) is float


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

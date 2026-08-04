import json
import math
import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_nav2_control.command_policy import SportCommandPolicy  # noqa: E402


def test_move_uses_official_api_and_clamps_positive_axes():
    command = _policy().accept_velocity(
        vx=1.8,
        vy=-0.8,
        vyaw=1.2,
        now_sec=10.0,
    )

    assert command.api_id == 1008
    assert json.loads(command.parameter) == {
        'x': 1.0,
        'y': -0.4,
        'z': 1.0,
    }


def test_move_clamps_reverse_independently_from_forward_limit():
    command = _policy().accept_velocity(
        vx=-1.8,
        vy=0.8,
        vyaw=-1.2,
        now_sec=10.0,
    )

    assert json.loads(command.parameter) == {
        'x': -0.5,
        'y': 0.4,
        'z': -1.0,
    }


def _policy(enabled=True):
    return SportCommandPolicy(
        min_linear_x=-0.5,
        max_linear_x=1.0,
        max_linear_y=0.4,
        max_angular_z=1.0,
        cmd_vel_timeout=0.3,
        enabled=enabled,
    )


def test_disabled_policy_does_not_claim_the_robot_command_channel():
    policy = _policy(enabled=False)

    command = policy.accept_velocity(0.1, 0.0, 0.0, now_sec=10.0)

    assert command is None
    assert policy.check_timeout(now_sec=20.0) is None
    assert policy.shutdown_command() is None


def test_zero_velocity_is_encoded_as_stop_move():
    command = _policy().accept_velocity(0.0, -0.0, 0.0, now_sec=10.0)

    assert command.api_id == 1003
    assert command.parameter == ''


@pytest.mark.parametrize('invalid_value', [math.nan, math.inf, -math.inf])
def test_non_finite_velocity_fails_safe_with_stop_move(invalid_value):
    command = _policy().accept_velocity(
        invalid_value,
        0.0,
        0.0,
        now_sec=10.0,
    )

    assert command.api_id == 1003
    assert command.parameter == ''


def test_watchdog_emits_one_stop_after_active_command_times_out():
    policy = _policy()
    policy.accept_velocity(0.1, 0.0, 0.0, now_sec=10.0)

    assert policy.check_timeout(now_sec=10.29) is None
    stop = policy.check_timeout(now_sec=10.30)
    assert stop.api_id == 1003
    assert stop.parameter == ''
    assert policy.check_timeout(now_sec=10.60) is None


def test_shutdown_stops_only_after_bridge_has_commanded_motion():
    idle_policy = _policy()
    active_policy = _policy()
    active_policy.accept_velocity(0.1, 0.0, 0.0, now_sec=10.0)

    assert idle_policy.shutdown_command() is None
    assert active_policy.shutdown_command().api_id == 1003


@pytest.mark.parametrize(
    'parameter,value',
    [
        ('min_linear_x', 0.1),
        ('min_linear_x', -math.inf),
        ('max_linear_x', -0.1),
        ('max_linear_y', math.inf),
        ('max_angular_z', math.nan),
        ('cmd_vel_timeout', 0.0),
    ],
)
def test_invalid_safety_parameter_is_rejected(parameter, value):
    arguments = {
        'min_linear_x': -0.5,
        'max_linear_x': 1.0,
        'max_linear_y': 0.4,
        'max_angular_z': 1.0,
        'cmd_vel_timeout': 0.3,
        'enabled': True,
    }
    arguments[parameter] = value

    with pytest.raises(ValueError):
        SportCommandPolicy(**arguments)

import json
import sys
from pathlib import Path

from unitree_api.msg import Request


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_nav2_control.command_policy import SportCommand  # noqa: E402
from go2_nav2_control import sport_cmd_bridge  # noqa: E402
from go2_nav2_control.sport_cmd_bridge import build_request, main  # noqa: E402


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_move_command_becomes_real_unitree_request():
    command = SportCommand(
        api_id=1008,
        parameter='{"x":0.1,"y":0.0,"z":-0.2}',
    )

    request = build_request(command)

    assert isinstance(request, Request)
    assert request.header.identity.api_id == 1008
    assert json.loads(request.parameter) == {
        'x': 0.1,
        'y': 0.0,
        'z': -0.2,
    }
    assert list(request.binary) == []


def test_stop_command_has_official_id_and_empty_parameter():
    request = build_request(SportCommand(api_id=1003, parameter=''))

    assert request.header.identity.api_id == 1003
    assert request.parameter == ''
    assert callable(main)


def test_sport_bridge_defaults_match_mppi_motion_envelope():
    assert getattr(sport_cmd_bridge, 'DEFAULT_MIN_LINEAR_X', None) == -0.5
    assert getattr(sport_cmd_bridge, 'DEFAULT_MAX_LINEAR_X', None) == 1.0
    assert getattr(sport_cmd_bridge, 'DEFAULT_MAX_LINEAR_Y', None) == 0.4
    assert getattr(sport_cmd_bridge, 'DEFAULT_MAX_ANGULAR_Z', None) == 1.0


def test_package_exports_lowstate_joint_state_bridge():
    setup_text = (PACKAGE_ROOT / 'setup.py').read_text()
    package_text = (PACKAGE_ROOT / 'package.xml').read_text()

    assert 'lowstate_joint_state_bridge = ' in setup_text
    assert "tests_require=['pytest']" in setup_text
    assert '<exec_depend>sensor_msgs</exec_depend>' in package_text
    assert '<exec_depend>unitree_go</exec_depend>' in package_text

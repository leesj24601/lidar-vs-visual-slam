import sys
from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_nav2_control.lowstate_joint_state_bridge import (  # noqa: E402
    build_joint_state,
)


def test_build_joint_state_populates_live_position_velocity_and_effort():
    motors = [
        SimpleNamespace(
            q=float(index),
            dq=float(index + 20),
            tau_est=float(index + 40),
        )
        for index in range(20)
    ]
    stamp = Time(sec=123, nanosec=456)

    message = build_joint_state(motors, stamp)

    assert message.header.stamp == stamp
    assert message.name[:3] == [
        'FL_hip_joint',
        'FL_thigh_joint',
        'FL_calf_joint',
    ]
    assert list(message.position) == [
        3.0, 4.0, 5.0,
        0.0, 1.0, 2.0,
        9.0, 10.0, 11.0,
        6.0, 7.0, 8.0,
    ]
    assert list(message.velocity) == [
        23.0, 24.0, 25.0,
        20.0, 21.0, 22.0,
        29.0, 30.0, 31.0,
        26.0, 27.0, 28.0,
    ]
    assert list(message.effort) == [
        43.0, 44.0, 45.0,
        40.0, 41.0, 42.0,
        49.0, 50.0, 51.0,
        46.0, 47.0, 48.0,
    ]

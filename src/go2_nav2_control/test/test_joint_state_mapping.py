from types import SimpleNamespace
import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_nav2_control.joint_state_mapping import (  # noqa: E402
    JOINT_NAMES,
    extract_joint_values,
)


def test_extract_joint_values_reorders_unitree_motors_for_urdf():
    motors = [
        SimpleNamespace(
            q=float(index),
            dq=float(index + 20),
            tau_est=float(index + 40),
        )
        for index in range(20)
    ]

    position, velocity, effort = extract_joint_values(motors)

    assert JOINT_NAMES == (
        'FL_hip_joint',
        'FL_thigh_joint',
        'FL_calf_joint',
        'FR_hip_joint',
        'FR_thigh_joint',
        'FR_calf_joint',
        'RL_hip_joint',
        'RL_thigh_joint',
        'RL_calf_joint',
        'RR_hip_joint',
        'RR_thigh_joint',
        'RR_calf_joint',
    )
    assert position == [
        3.0, 4.0, 5.0,
        0.0, 1.0, 2.0,
        9.0, 10.0, 11.0,
        6.0, 7.0, 8.0,
    ]
    assert velocity == [
        23.0, 24.0, 25.0,
        20.0, 21.0, 22.0,
        29.0, 30.0, 31.0,
        26.0, 27.0, 28.0,
    ]
    assert effort == [
        43.0, 44.0, 45.0,
        40.0, 41.0, 42.0,
        49.0, 50.0, 51.0,
        46.0, 47.0, 48.0,
    ]


def test_extract_joint_values_rejects_short_motor_array():
    motors = [
        SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0)
        for _ in range(11)
    ]

    with pytest.raises(ValueError, match='at least 12'):
        extract_joint_values(motors)

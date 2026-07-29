import importlib
import math
import sys
from pathlib import Path

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _alignment_module():
    return importlib.import_module(
        'go2_rtabmap_bridge.odom_initial_alignment_tf'
    )


def _stamp_from_seconds(seconds):
    stamp = TimeMsg()
    stamp.sec = int(seconds)
    stamp.nanosec = round((seconds - stamp.sec) * 1_000_000_000)
    return stamp


def _odom(frame_id, stamp_seconds, x=0.0, y=0.0, yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.header.stamp = _stamp_from_seconds(stamp_seconds)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
    return msg


def _yaw(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def test_inverse_first_planar_pose_becomes_comparison_frame_transform():
    alignment = _alignment_module()
    first_pose = _odom(
        'go2_odom',
        stamp_seconds=10.0,
        x=1.0,
        y=2.0,
        yaw=math.pi / 2.0,
    )
    transform_stamp = _stamp_from_seconds(10.02)

    transform = alignment.inverse_planar_origin_transform(
        first_pose,
        parent_frame_id='odom_compare',
        transform_stamp=transform_stamp,
    )

    assert transform.header.stamp == transform_stamp
    assert transform.header.frame_id == 'odom_compare'
    assert transform.child_frame_id == 'go2_odom'
    assert math.isclose(
        transform.transform.translation.x,
        -2.0,
        abs_tol=1e-9,
    )
    assert math.isclose(
        transform.transform.translation.y,
        1.0,
        abs_tol=1e-9,
    )
    assert transform.transform.translation.z == 0.0
    assert math.isclose(
        _yaw(transform.transform.rotation),
        -math.pi / 2.0,
        abs_tol=1e-9,
    )


def test_nearest_valid_pair_is_selected_within_time_limit():
    alignment = _alignment_module()
    invalid_vo = _odom('', 10.001)
    go2_messages = [
        _odom('go2_odom', 10.000),
        _odom('go2_odom', 10.040),
    ]
    vo_messages = [
        invalid_vo,
        _odom('vo_odom', 10.043),
    ]

    pair = alignment.find_nearest_valid_pair(
        go2_messages,
        vo_messages,
        max_time_gap_ns=50_000_000,
    )

    assert pair == (go2_messages[1], vo_messages[1])


def test_pair_is_not_selected_when_time_gap_exceeds_limit():
    alignment = _alignment_module()

    pair = alignment.find_nearest_valid_pair(
        [_odom('go2_odom', 10.000)],
        [_odom('vo_odom', 10.051)],
        max_time_gap_ns=50_000_000,
    )

    assert pair is None

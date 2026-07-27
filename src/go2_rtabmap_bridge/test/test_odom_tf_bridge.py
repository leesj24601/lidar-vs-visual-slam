import sys
from pathlib import Path

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_rtabmap_bridge import odom_tf_bridge
from go2_rtabmap_bridge.odom_tf_bridge import (
    apply_planar_base_motion,
    apply_time_offset,
    split_planar_and_body_transforms,
    transform_from_odom,
)


def _stamp(sec, nanosec):
    stamp = TimeMsg()
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def test_apply_time_offset_preserves_sensor_delta():
    first_stamp = _stamp(10, 200_000_000)
    second_stamp = _stamp(12, 700_000_000)
    offset = Duration(seconds=123, nanoseconds=456_000_000)

    corrected_first = apply_time_offset(first_stamp, offset)
    corrected_second = apply_time_offset(second_stamp, offset)

    original_delta = Time.from_msg(second_stamp) - Time.from_msg(first_stamp)
    corrected_delta = Time.from_msg(corrected_second) - Time.from_msg(corrected_first)

    assert corrected_delta.nanoseconds == original_delta.nanoseconds


def test_corrected_odom_stamp_applies_negative_sensor_residual_offset():
    assert hasattr(odom_tf_bridge, 'corrected_odom_stamp')
    sensor_stamp = _stamp(10, 200_000_000)
    clock_epoch_offset = Duration(seconds=100)

    corrected = odom_tf_bridge.corrected_odom_stamp(
        sensor_stamp,
        clock_epoch_offset,
        sensor_time_offset_sec=-0.015,
    )

    assert Time.from_msg(corrected).nanoseconds == 110_185_000_000


def test_transform_from_odom_uses_corrected_stamp_and_configured_frames():
    corrected_stamp = _stamp(200, 42)
    msg = Odometry()
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.position.z = 0.75
    msg.pose.pose.orientation.x = 0.1
    msg.pose.pose.orientation.y = 0.2
    msg.pose.pose.orientation.z = 0.3
    msg.pose.pose.orientation.w = 0.9

    transform = transform_from_odom(
        msg,
        corrected_stamp,
        odom_frame_id='visual_odom',
        base_frame_id='go2_base',
    )

    assert transform.header.stamp == corrected_stamp
    assert transform.header.frame_id == 'visual_odom'
    assert transform.child_frame_id == 'go2_base'
    assert transform.transform.translation.x == msg.pose.pose.position.x
    assert transform.transform.translation.y == msg.pose.pose.position.y
    assert transform.transform.translation.z == msg.pose.pose.position.z
    assert transform.transform.rotation == msg.pose.pose.orientation


def test_split_planar_and_body_transforms_keep_camera_body_tilt_chain():
    corrected_stamp = _stamp(200, 42)
    msg = Odometry()
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.position.z = 0.75
    msg.pose.pose.orientation.x = 0.2
    msg.pose.pose.orientation.y = -0.1
    msg.pose.pose.orientation.z = 0.3
    msg.pose.pose.orientation.w = 0.9

    odom_to_footprint, footprint_to_base = split_planar_and_body_transforms(
        msg,
        corrected_stamp,
        odom_frame_id='odom',
        footprint_frame_id='base_footprint',
        base_frame_id='base_link',
    )

    assert odom_to_footprint.header.frame_id == 'odom'
    assert odom_to_footprint.child_frame_id == 'base_footprint'
    assert odom_to_footprint.transform.translation.x == msg.pose.pose.position.x
    assert odom_to_footprint.transform.translation.y == msg.pose.pose.position.y
    assert odom_to_footprint.transform.translation.z == 0.0
    assert odom_to_footprint.transform.rotation.x == 0.0
    assert odom_to_footprint.transform.rotation.y == 0.0

    assert footprint_to_base.header.frame_id == 'base_footprint'
    assert footprint_to_base.child_frame_id == 'base_link'
    assert footprint_to_base.transform.translation.z == msg.pose.pose.position.z
    assert footprint_to_base.transform.rotation.x != 0.0
    assert footprint_to_base.transform.rotation.y != 0.0


def test_apply_planar_base_motion_keeps_yaw_and_removes_z_roll_pitch():
    msg = Odometry()
    msg.child_frame_id = 'raw_base'
    msg.pose.pose.position.x = 1.25
    msg.pose.pose.position.y = -0.5
    msg.pose.pose.position.z = 0.75
    msg.pose.pose.orientation.x = 0.2
    msg.pose.pose.orientation.y = -0.1
    msg.pose.pose.orientation.z = 0.3
    msg.pose.pose.orientation.w = 0.9

    apply_planar_base_motion(msg, 'base_link')

    assert msg.child_frame_id == 'base_link'
    assert msg.pose.pose.position.x == 1.25
    assert msg.pose.pose.position.y == -0.5
    assert msg.pose.pose.position.z == 0.0
    assert msg.pose.pose.orientation.x == 0.0
    assert msg.pose.pose.orientation.y == 0.0
    assert msg.pose.pose.orientation.z != 0.0
    assert msg.pose.pose.orientation.w != 0.0

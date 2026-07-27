import struct
import sys
from pathlib import Path

from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_rtabmap_bridge.bridge_node import remove_zero_padding_points


def _field(name, offset):
    field = PointField()
    field.name = name
    field.offset = offset
    field.datatype = PointField.FLOAT32
    field.count = 1
    return field


def _cloud(points):
    msg = PointCloud2()
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        _field('x', 0),
        _field('y', 4),
        _field('z', 8),
        _field('intensity', 16),
    ]
    msg.point_step = 32
    msg.row_step = msg.point_step * msg.width
    data = bytearray(msg.row_step)
    for index, point in enumerate(points):
        offset = index * msg.point_step
        for field_offset, value in zip((0, 4, 8, 16), point):
            struct.pack_into('<f', data, offset + field_offset, value)
    msg.data = bytes(data)
    return msg


def _read_points(msg):
    points = []
    for index in range(msg.width):
        offset = index * msg.point_step
        points.append(
            tuple(
                struct.unpack_from('<f', msg.data, offset + field_offset)[0]
                for field_offset in (0, 4, 8, 16)
            )
        )
    return points


def test_remove_zero_padding_points_removes_only_zero_xyz_intensity_points():
    cloud = _cloud([
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 2.0, 3.0, 10.0),
        (0.0, 0.0, 0.0, 5.0),
        (-1.0, 0.5, 0.25, 20.0),
    ])

    filtered = remove_zero_padding_points(cloud)

    assert filtered.height == 1
    assert filtered.width == 3
    assert filtered.row_step == filtered.point_step * filtered.width
    assert _read_points(filtered) == [
        (1.0, 2.0, 3.0, 10.0),
        (0.0, 0.0, 0.0, 5.0),
        (-1.0, 0.5, 0.25, 20.0),
    ]

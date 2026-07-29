import csv
import json

from nav_msgs.msg import Odometry
import pytest
from rclpy.serialization import serialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialWriter,
    StorageOptions,
    TopicMetadata,
)

from go2_rtabmap_bridge.analyze_odom_bag import (
    analyze_bag,
    build_argument_parser,
)


def odometry(stamp_ns, x):
    message = Odometry()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = 'test_odom'
    message.child_frame_id = 'base_link'
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


def write_test_bag(bag_path, messages_by_topic):
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    for topic_name in messages_by_topic:
        writer.create_topic(TopicMetadata(
            name=topic_name,
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr',
            offered_qos_profiles='',
        ))

    recorded_messages = []
    for topic_name, messages in messages_by_topic.items():
        for message in messages:
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            recorded_messages.append((stamp_ns, topic_name, message))
    for stamp_ns, topic_name, message in sorted(recorded_messages):
        writer.write(topic_name, serialize_message(message), stamp_ns)
    del writer
    return bag_path


def test_analyze_bag_writes_aligned_csv_and_summary_from_real_rosbag(
    tmp_path,
):
    bag_path = write_test_bag(
        tmp_path / 'bag',
        {
            '/odom/go2': [
                odometry(1_000_000_000, x=10.0),
                odometry(1_100_000_000, x=11.0),
            ],
            '/odom/vo': [
                odometry(1_010_000_000, x=0.0),
                odometry(1_110_000_000, x=1.0),
            ],
        },
    )

    csv_path, json_path = analyze_bag(
        bag_directory=bag_path,
        go2_topic='/odom/go2',
        vo_topic='/odom/vo',
        max_time_gap_ms=50.0,
        long_vo_gap_ms=500.0,
        output_prefix=tmp_path / 'results' / 'comparison',
    )

    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    summary = json.loads(json_path.read_text(encoding='utf-8'))

    assert len(rows) == 2
    assert float(rows[1]['position_difference_m']) == pytest.approx(0.0)
    assert summary['matched_pairs'] == 2
    assert summary['time_gap_max_ms'] == pytest.approx(10.0)
    assert summary['final_position_difference_m'] == pytest.approx(0.0)


def test_analyze_bag_reports_missing_required_topic(tmp_path):
    bag_path = write_test_bag(
        tmp_path / 'bag',
        {
            '/odom/go2': [
                odometry(1_000_000_000, x=0.0),
            ],
        },
    )

    with pytest.raises(ValueError, match='/odom/vo'):
        analyze_bag(
            bag_directory=bag_path,
            go2_topic='/odom/go2',
            vo_topic='/odom/vo',
            max_time_gap_ms=50.0,
            long_vo_gap_ms=500.0,
            output_prefix=tmp_path / 'comparison',
        )


def test_cli_defaults_match_comparison_launch_topics():
    args = build_argument_parser().parse_args(['/tmp/example_bag'])

    assert args.go2_topic == '/odom/go2'
    assert args.vo_topic == '/odom/vo'
    assert args.max_time_gap_ms == 50.0
    assert args.long_vo_gap_ms == 500.0

import argparse
import csv
import json
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageFilter,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message

from .odom_comparison import (
    PoseSample,
    build_comparison_rows,
    match_nearest_samples,
    normalized_quaternion,
    summarize_comparison,
)


CSV_FIELDS = [
    'stamp_ns',
    'time_gap_ms',
    'go2_x_m',
    'go2_y_m',
    'go2_z_m',
    'go2_yaw_rad',
    'vo_x_m',
    'vo_y_m',
    'vo_z_m',
    'vo_yaw_rad',
    'position_difference_m',
    'yaw_difference_rad',
]


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description='Compare Go2 and RGB-D visual odometry in a ROS 2 bag.',
    )
    parser.add_argument(
        'bag_directory',
        type=Path,
        help='ROS 2 bag directory containing both odometry topics.',
    )
    parser.add_argument('--go2-topic', default='/odom/go2')
    parser.add_argument('--vo-topic', default='/odom/vo')
    parser.add_argument(
        '--max-time-gap-ms',
        type=float,
        default=50.0,
        help='Maximum timestamp difference for a matched pose pair.',
    )
    parser.add_argument(
        '--long-vo-gap-ms',
        type=float,
        default=500.0,
        help='VO output gap counted as a possible tracking interruption.',
    )
    parser.add_argument(
        '--output-prefix',
        type=Path,
        default=Path('odom_comparison'),
        help='Prefix used for the generated CSV and JSON files.',
    )
    return parser


def _pose_sample_from_odometry(message, recorded_stamp_ns):
    header_stamp_ns = (
        message.header.stamp.sec * 1_000_000_000
        + message.header.stamp.nanosec
    )
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return PoseSample(
        stamp_ns=header_stamp_ns or recorded_stamp_ns,
        position=(position.x, position.y, position.z),
        orientation=normalized_quaternion((
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )),
    )


def read_odometry_topics(bag_directory, requested_topics):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_directory), storage_id=''),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topic_types = {
        metadata.name: metadata.type
        for metadata in reader.get_all_topics_and_types()
    }
    missing_topics = [
        topic
        for topic in requested_topics
        if topic not in topic_types
    ]
    if missing_topics:
        raise ValueError(
            'Missing odometry topics in bag: '
            + ', '.join(missing_topics)
        )

    invalid_topics = [
        topic
        for topic in requested_topics
        if topic_types[topic] != 'nav_msgs/msg/Odometry'
    ]
    if invalid_topics:
        raise ValueError(
            'Expected nav_msgs/msg/Odometry for topics: '
            + ', '.join(invalid_topics)
        )

    reader.set_filter(StorageFilter(topics=list(requested_topics)))
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in requested_topics
    }
    samples = {topic: [] for topic in requested_topics}
    while reader.has_next():
        topic, serialized, recorded_stamp_ns = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        samples[topic].append(
            _pose_sample_from_odometry(message, recorded_stamp_ns)
        )
    for topic in samples:
        samples[topic].sort(key=lambda item: item.stamp_ns)
    return samples


def write_outputs(output_prefix, rows, summary):
    output_prefix = Path(output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = output_prefix.with_name(
        f'{output_prefix.name}_samples.csv'
    )
    json_path = output_prefix.with_name(
        f'{output_prefix.name}_summary.json'
    )

    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    return csv_path, json_path


def analyze_bag(
    bag_directory,
    go2_topic,
    vo_topic,
    max_time_gap_ms,
    long_vo_gap_ms,
    output_prefix,
):
    if max_time_gap_ms < 0.0:
        raise ValueError('max_time_gap_ms must be non-negative')
    if long_vo_gap_ms < 0.0:
        raise ValueError('long_vo_gap_ms must be non-negative')

    samples = read_odometry_topics(
        bag_directory,
        [go2_topic, vo_topic],
    )
    pairs = match_nearest_samples(
        vo_samples=samples[vo_topic],
        go2_samples=samples[go2_topic],
        max_gap_ns=round(max_time_gap_ms * 1_000_000.0),
    )
    rows = build_comparison_rows(pairs)
    summary = summarize_comparison(
        rows,
        vo_samples=samples[vo_topic],
        long_gap_sec=long_vo_gap_ms / 1000.0,
    )
    return write_outputs(output_prefix, rows, summary)


def main(args=None):
    parser = build_argument_parser()
    parsed = parser.parse_args(args)
    try:
        csv_path, json_path = analyze_bag(
            bag_directory=parsed.bag_directory,
            go2_topic=parsed.go2_topic,
            vo_topic=parsed.vo_topic,
            max_time_gap_ms=parsed.max_time_gap_ms,
            long_vo_gap_ms=parsed.long_vo_gap_ms,
            output_prefix=parsed.output_prefix,
        )
    except (RuntimeError, ValueError) as error:
        parser.error(str(error))
    print(f'Wrote sample comparison: {csv_path}')
    print(f'Wrote summary: {json_path}')


if __name__ == '__main__':
    main()

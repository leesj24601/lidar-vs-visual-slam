import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ImuSample:
    stamp_ns: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description='Measure and validate an integrated RealSense IMU topic.',
    )
    parser.add_argument('--topic', default='/camera/imu')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--output-json')
    return parser


def summarize_samples(samples):
    sample_count = len(samples)
    duration_s = 0.0
    rate_hz = 0.0
    if sample_count >= 2:
        duration_s = (
            samples[-1].stamp_ns - samples[0].stamp_ns
        ) / 1_000_000_000.0
        if duration_s > 0.0:
            rate_hz = (sample_count - 1) / duration_s

    accel_norms = [
        math.sqrt(sample.ax ** 2 + sample.ay ** 2 + sample.az ** 2)
        for sample in samples
    ]
    accel_norm_mean = (
        sum(accel_norms) / sample_count
        if sample_count
        else 0.0
    )
    gyro_abs_max = max(
        (
            abs(axis)
            for sample in samples
            for axis in (sample.gx, sample.gy, sample.gz)
        ),
        default=0.0,
    )

    return {
        'sample_count': sample_count,
        'duration_s': duration_s,
        'rate_hz': rate_hz,
        'accel_norm_mean': accel_norm_mean,
        'gyro_abs_max': gyro_abs_max,
    }


def validate_summary(
    summary,
    min_rate_hz=160.0,
    max_rate_hz=240.0,
    min_accel_norm=7.0,
    max_accel_norm=12.0,
    max_gyro_abs=1.0,
):
    errors = []
    if summary['sample_count'] < 2:
        errors.append('at least 2 IMU samples are required')
    elif summary['duration_s'] <= 0.0:
        errors.append('IMU timestamps must increase')
    elif not min_rate_hz <= summary['rate_hz'] <= max_rate_hz:
        errors.append(
            f"rate_hz {summary['rate_hz']:.3f} is outside "
            f'{min_rate_hz:.3f}..{max_rate_hz:.3f} Hz'
        )

    if not min_accel_norm <= summary['accel_norm_mean'] <= max_accel_norm:
        errors.append(
            f"accel_norm_mean {summary['accel_norm_mean']:.3f} m/s^2 "
            f'is outside {min_accel_norm:.3f}..{max_accel_norm:.3f} m/s^2'
        )

    if summary['gyro_abs_max'] >= max_gyro_abs:
        errors.append(
            f"gyro_abs_max {summary['gyro_abs_max']:.3f} rad/s "
            f'is not below {max_gyro_abs:.3f} rad/s'
        )

    return errors


def main(argv=None):
    args = build_argument_parser().parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class ImuProbeNode(Node):
        def __init__(self):
            super().__init__('realsense_imu_probe')
            self.samples = []
            self.frame_ids = set()
            self.subscription = self.create_subscription(
                Imu,
                args.topic,
                self.on_imu,
                qos_profile_sensor_data,
            )

        def on_imu(self, message):
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            self.samples.append(ImuSample(
                stamp_ns=stamp_ns,
                ax=message.linear_acceleration.x,
                ay=message.linear_acceleration.y,
                az=message.linear_acceleration.z,
                gx=message.angular_velocity.x,
                gy=message.angular_velocity.y,
                gz=message.angular_velocity.z,
            ))
            self.frame_ids.add(message.header.frame_id)

    rclpy.init(args=None)
    node = ImuProbeNode()
    started_at = time.monotonic()
    try:
        while rclpy.ok():
            elapsed = time.monotonic() - started_at
            if elapsed >= args.duration:
                break
            rclpy.spin_once(
                node,
                timeout_sec=min(0.1, args.duration - elapsed),
            )
    finally:
        wall_duration_s = time.monotonic() - started_at
        node.destroy_node()
        rclpy.shutdown()

    summary = summarize_samples(node.samples)
    frame_ids = sorted(node.frame_ids)
    summary.update({
        'topic': args.topic,
        'wall_duration_s': wall_duration_s,
        'frame_id': frame_ids[0] if len(frame_ids) == 1 else None,
        'frame_ids': frame_ids,
    })
    validation_errors = validate_summary(summary)
    if frame_ids != ['camera_imu_optical_frame']:
        validation_errors.append(
            'frame_id must be camera_imu_optical_frame'
        )
    summary['validation_errors'] = validation_errors

    output = json.dumps(summary, indent=2, sort_keys=True)
    print(output)
    if args.output_json:
        Path(args.output_json).write_text(
            output + '\n',
            encoding='utf-8',
        )

    return 1 if validation_errors else 0


if __name__ == '__main__':
    raise SystemExit(main())

import pytest

from go2_rtabmap_bridge.realsense_imu_probe import (
    ImuSample,
    build_argument_parser,
    summarize_samples,
    validate_summary,
)


def samples(gyro_x=0.01, count=201, step_ns=5_000_000):
    return [
        ImuSample(
            stamp_ns=index * step_ns,
            ax=0.0,
            ay=0.0,
            az=9.81,
            gx=gyro_x,
            gy=-0.01,
            gz=0.005,
        )
        for index in range(count)
    ]


def test_stationary_200_hz_imu_passes():
    summary = summarize_samples(samples())

    assert summary['rate_hz'] == pytest.approx(200.0)
    assert summary['accel_norm_mean'] == pytest.approx(9.81)
    assert validate_summary(summary) == []


def test_corrupted_stationary_gyro_fails():
    summary = summarize_samples(samples(gyro_x=19.7))

    assert validate_summary(summary) == [
        'gyro_abs_max 19.700 rad/s is not below 1.000 rad/s'
    ]


def test_too_few_samples_fails_cleanly():
    summary = summarize_samples(samples(count=1))

    assert 'at least 2 IMU samples are required' in validate_summary(summary)


def test_cli_defaults_target_integrated_realsense_imu():
    args = build_argument_parser().parse_args([])

    assert args.topic == '/camera/imu'
    assert args.duration == 30.0
    assert args.output_json is None

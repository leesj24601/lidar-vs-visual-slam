import math

import pytest

from go2_rtabmap_bridge.odom_comparison import (
    PoseSample,
    build_comparison_rows,
    match_nearest_samples,
    summarize_comparison,
    yaw_quaternion,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def sample(stamp_ms, x=0.0, y=0.0, yaw=0.0):
    return PoseSample(
        stamp_ns=stamp_ms * 1_000_000,
        position=(x, y, 0.0),
        orientation=yaw_quaternion(yaw),
    )


def test_nearest_matching_rejects_samples_outside_time_limit():
    go2 = [sample(0), sample(100), sample(200)]
    vo = [sample(47), sample(151), sample(280)]

    pairs = match_nearest_samples(
        vo_samples=vo,
        go2_samples=go2,
        max_gap_ns=50_000_000,
    )

    assert [pair.go2.stamp_ns for pair in pairs] == [
        0,
        200_000_000,
    ]
    assert [pair.time_gap_ns for pair in pairs] == [
        47_000_000,
        49_000_000,
    ]


def test_relative_poses_remove_different_origins_and_heading():
    go2 = [
        sample(0, x=10.0, y=-2.0, yaw=0.5),
        sample(
            100,
            x=10.0 + math.cos(0.5),
            y=-2.0 + math.sin(0.5),
            yaw=0.6,
        ),
    ]
    vo = [
        sample(0),
        sample(100, x=1.0, y=0.0, yaw=0.1),
    ]

    rows = build_comparison_rows(
        match_nearest_samples(
            vo_samples=vo,
            go2_samples=go2,
            max_gap_ns=50_000_000,
        )
    )

    assert rows[0]['position_difference_m'] == pytest.approx(0.0)
    assert rows[1]['go2_x_m'] == pytest.approx(1.0)
    assert rows[1]['go2_y_m'] == pytest.approx(0.0, abs=1e-12)
    assert rows[1]['position_difference_m'] == pytest.approx(
        0.0,
        abs=1e-12,
    )
    assert rows[1]['yaw_difference_rad'] == pytest.approx(
        0.0,
        abs=1e-12,
    )


def test_yaw_difference_wraps_across_pi_boundary():
    go2 = [sample(0), sample(100, yaw=math.radians(179.0))]
    vo = [sample(0), sample(100, yaw=math.radians(-179.0))]

    rows = build_comparison_rows(
        match_nearest_samples(
            vo_samples=vo,
            go2_samples=go2,
            max_gap_ns=50_000_000,
        )
    )

    assert math.degrees(rows[1]['yaw_difference_rad']) == pytest.approx(
        2.0
    )


def test_summary_reports_sync_divergence_and_vo_gaps():
    stamps_ms = (0, 100, 200, 1000)
    go2 = [
        sample(stamp, x=index * 0.1)
        for index, stamp in enumerate(stamps_ms)
    ]
    vo = [
        sample(stamp, x=index * 0.1)
        for index, stamp in enumerate(stamps_ms)
    ]
    pairs = match_nearest_samples(
        vo_samples=vo,
        go2_samples=go2,
        max_gap_ns=50_000_000,
    )
    rows = build_comparison_rows(pairs)

    summary = summarize_comparison(
        rows,
        vo_samples=vo,
        long_gap_sec=0.5,
    )

    assert summary['matched_pairs'] == 4
    assert summary['duration_sec'] == pytest.approx(1.0)
    assert summary['time_gap_p95_ms'] == pytest.approx(0.0)
    assert summary['go2_path_length_m'] == pytest.approx(0.3)
    assert summary['vo_path_length_m'] == pytest.approx(0.3)
    assert summary['position_difference_rmse_m'] == pytest.approx(0.0)
    assert summary['yaw_difference_p95_deg'] == pytest.approx(0.0)
    assert summary['vo_effective_rate_hz'] == pytest.approx(3.0)
    assert summary['vo_max_gap_sec'] == pytest.approx(0.8)
    assert summary['vo_long_gap_count'] == 1


def test_comparison_rejects_empty_matched_trajectory():
    with pytest.raises(ValueError, match='No odometry pairs matched'):
        build_comparison_rows([])

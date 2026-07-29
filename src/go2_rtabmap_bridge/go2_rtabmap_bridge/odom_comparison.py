from bisect import bisect_left
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    position: tuple
    orientation: tuple


@dataclass(frozen=True)
class MatchedPair:
    vo: PoseSample
    go2: PoseSample
    time_gap_ns: int


def normalized_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm == 0.0:
        raise ValueError('Quaternion norm must be non-zero')
    return tuple(value / norm for value in quaternion)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(quaternion):
    x, y, z, w = normalized_quaternion(quaternion)
    return -x, -y, -z, w


def rotate_vector(quaternion, vector):
    unit_quaternion = normalized_quaternion(quaternion)
    rotated = quaternion_multiply(
        quaternion_multiply(
            unit_quaternion,
            (vector[0], vector[1], vector[2], 0.0),
        ),
        quaternion_inverse(unit_quaternion),
    )
    return rotated[0], rotated[1], rotated[2]


def yaw_from_quaternion(quaternion):
    x, y, z, w = normalized_quaternion(quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_quaternion(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def match_nearest_samples(vo_samples, go2_samples, max_gap_ns):
    if max_gap_ns < 0:
        raise ValueError('max_gap_ns must be non-negative')

    ordered_go2 = sorted(go2_samples, key=lambda item: item.stamp_ns)
    go2_stamps = [item.stamp_ns for item in ordered_go2]
    pairs = []
    for vo_sample in sorted(vo_samples, key=lambda item: item.stamp_ns):
        insertion_index = bisect_left(go2_stamps, vo_sample.stamp_ns)
        candidate_indices = {
            insertion_index - 1,
            insertion_index,
        }
        candidates = [
            ordered_go2[index]
            for index in candidate_indices
            if 0 <= index < len(ordered_go2)
        ]
        if not candidates:
            continue

        go2_sample = min(
            candidates,
            key=lambda item: (
                abs(item.stamp_ns - vo_sample.stamp_ns),
                item.stamp_ns,
            ),
        )
        time_gap_ns = abs(go2_sample.stamp_ns - vo_sample.stamp_ns)
        if time_gap_ns <= max_gap_ns:
            pairs.append(MatchedPair(
                vo=vo_sample,
                go2=go2_sample,
                time_gap_ns=time_gap_ns,
            ))
    return pairs


def relative_pose(origin, sample):
    origin_inverse = quaternion_inverse(origin.orientation)
    position_delta = tuple(
        sample_value - origin_value
        for sample_value, origin_value in zip(
            sample.position,
            origin.position,
        )
    )
    return PoseSample(
        stamp_ns=sample.stamp_ns,
        position=rotate_vector(origin_inverse, position_delta),
        orientation=normalized_quaternion(
            quaternion_multiply(origin_inverse, sample.orientation)
        ),
    )


def build_comparison_rows(matched_pairs):
    if not matched_pairs:
        raise ValueError(
            'No odometry pairs matched within the configured time gap'
        )

    go2_origin = matched_pairs[0].go2
    vo_origin = matched_pairs[0].vo
    rows = []
    for pair in matched_pairs:
        go2_pose = relative_pose(go2_origin, pair.go2)
        vo_pose = relative_pose(vo_origin, pair.vo)
        position_delta = tuple(
            vo_value - go2_value
            for vo_value, go2_value in zip(
                vo_pose.position,
                go2_pose.position,
            )
        )
        go2_yaw = yaw_from_quaternion(go2_pose.orientation)
        vo_yaw = yaw_from_quaternion(vo_pose.orientation)
        rows.append({
            'stamp_ns': pair.vo.stamp_ns,
            'time_gap_ms': pair.time_gap_ns / 1_000_000.0,
            'go2_x_m': go2_pose.position[0],
            'go2_y_m': go2_pose.position[1],
            'go2_z_m': go2_pose.position[2],
            'go2_yaw_rad': go2_yaw,
            'vo_x_m': vo_pose.position[0],
            'vo_y_m': vo_pose.position[1],
            'vo_z_m': vo_pose.position[2],
            'vo_yaw_rad': vo_yaw,
            'position_difference_m': math.sqrt(
                sum(value * value for value in position_delta)
            ),
            'yaw_difference_rad': wrap_angle(vo_yaw - go2_yaw),
        })
    return rows


def _path_length(rows, prefix):
    total = 0.0
    previous = None
    for row in rows:
        current = (
            row[f'{prefix}_x_m'],
            row[f'{prefix}_y_m'],
            row[f'{prefix}_z_m'],
        )
        if previous is not None:
            total += math.sqrt(sum(
                (current_value - previous_value) ** 2
                for current_value, previous_value in zip(
                    current,
                    previous,
                )
            ))
        previous = current
    return total


def _rmse(values):
    array = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(array * array)))


def _percentile(values, percentile):
    return float(np.percentile(
        np.asarray(values, dtype=float),
        percentile,
    ))


def summarize_comparison(rows, vo_samples, long_gap_sec=0.5):
    if not rows:
        raise ValueError(
            'No odometry pairs matched within the configured time gap'
        )
    if long_gap_sec < 0.0:
        raise ValueError('long_gap_sec must be non-negative')

    ordered_vo = sorted(vo_samples, key=lambda item: item.stamp_ns)
    vo_gaps_sec = [
        (current.stamp_ns - previous.stamp_ns) / 1_000_000_000.0
        for previous, current in zip(ordered_vo, ordered_vo[1:])
    ]
    vo_duration_sec = (
        (
            ordered_vo[-1].stamp_ns
            - ordered_vo[0].stamp_ns
        ) / 1_000_000_000.0
        if len(ordered_vo) >= 2
        else 0.0
    )
    position_differences = [
        row['position_difference_m']
        for row in rows
    ]
    yaw_differences_deg = [
        math.degrees(abs(row['yaw_difference_rad']))
        for row in rows
    ]
    time_gaps_ms = [row['time_gap_ms'] for row in rows]

    return {
        'matched_pairs': len(rows),
        'duration_sec': (
            rows[-1]['stamp_ns'] - rows[0]['stamp_ns']
        ) / 1_000_000_000.0,
        'time_gap_median_ms': float(np.median(time_gaps_ms)),
        'time_gap_p95_ms': _percentile(time_gaps_ms, 95),
        'time_gap_max_ms': max(time_gaps_ms),
        'go2_path_length_m': _path_length(rows, 'go2'),
        'vo_path_length_m': _path_length(rows, 'vo'),
        'final_position_difference_m': position_differences[-1],
        'position_difference_rmse_m': _rmse(position_differences),
        'position_difference_p95_m': _percentile(
            position_differences,
            95,
        ),
        'final_yaw_difference_deg': math.degrees(
            rows[-1]['yaw_difference_rad']
        ),
        'yaw_difference_rmse_deg': _rmse(yaw_differences_deg),
        'yaw_difference_p95_deg': _percentile(
            yaw_differences_deg,
            95,
        ),
        'vo_effective_rate_hz': (
            (len(ordered_vo) - 1) / vo_duration_sec
            if vo_duration_sec > 0.0
            else 0.0
        ),
        'vo_max_gap_sec': max(vo_gaps_sec, default=0.0),
        'vo_long_gap_count': sum(
            gap_sec > long_gap_sec
            for gap_sec in vo_gaps_sec
        ),
    }

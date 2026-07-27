# VO-Go2 Odometry Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 기존 Visual SLAM 로직을 변경하지 않고 RealSense RGB-D VO와 Go2 내부 odometry를 동시에 생성·기록하여 두 궤적의 차이와 VO tracking 안정성을 정량 비교한다.

**Architecture:** 새 `vo_odom_comparison.launch.py`가 RTAB-Map SLAM 노드 없이 `rgbd_sync`, `rgbd_odometry`, `odom_tf_bridge`만 실행한다. 두 odometry는 `/odom/vo`와 `/odom/go2`로 분리하고 TF를 발행하지 않아 기존 `odom -> base_link`와 충돌하지 않게 한다. 비교는 rosbag을 오프라인으로 읽고 첫 동기 pose를 공통 원점으로 정렬한 뒤 위치·yaw 차이, 시간 동기 품질, VO 출력 중단 구간을 CSV와 JSON으로 저장한다.

**Tech Stack:** ROS 2 Humble, Python 3, `rtabmap_sync/rgbd_sync`, `rtabmap_odom/rgbd_odometry`, `nav_msgs/Odometry`, `rosbag2_py`, NumPy, pytest, ament/colcon

## Global Constraints

- 기존 `src/go2_rtabmap_launch/launch/visual_slam.launch.py`, `visual_localization.launch.py`, `rtabmap_visual_real.yaml`의 동작과 기본값을 변경하지 않는다.
- RTAB-Map SLAM, map 생성, loop closure, localization 노드는 비교 실행에 포함하지 않는다.
- 비교용 출력 토픽은 `/odom/go2`와 `/odom/vo`로 고정한다.
- 두 비교 노드는 모두 odometry TF를 발행하지 않는다. 동일한 `base_link`에 대한 복수 부모 TF를 만들지 않는다.
- Go2 odometry는 기존 bridge와 동일하게 clock epoch를 보정하고 `sensor_time_offset_sec=-0.015`를 적용한다.
- VO는 aligned RGB-D와 현재 `base_link -> camera_link` extrinsic 기본값 `x=0.34`, `y=0.0`, `z=0.095`, `roll=pitch=yaw=0.0`을 사용한다.
- 비교 시 최대 timestamp 차이는 기본 50 ms로 제한한다.
- 첫 번째로 유효하게 매칭된 pose 쌍을 각 odometry의 원점으로 삼아 상대 궤적을 비교한다.
- Go2 odometry는 ground truth가 아니다. 모든 결과 명칭은 `error` 대신 가능한 한 `difference` 또는 `divergence`를 사용한다.
- 현재 worktree에 존재하는 사용자 변경사항을 덮어쓰거나 정리하지 않는다.

## 성공 조건

1. 단일 launch 명령으로 `/odom/go2`와 `/odom/vo`가 발행된다.
2. 비교 launch를 실행해도 `/rtabmap/*` SLAM 노드와 `map -> odom` TF가 생성되지 않는다.
3. 비교 launch 내부의 Go2 bridge와 VO 노드는 동적 TF를 발행하지 않는다.
4. rosbag 분석기는 최대 50 ms 이내의 최근접 timestamp 쌍만 사용한다.
5. 분석 결과는 timestamp별 CSV와 전체 요약 JSON을 생성한다.
6. 요약에는 매칭 수, 시간차 통계, 주행거리, 최종/전체 위치 차이, yaw 차이, VO 출력률 및 장시간 gap이 포함된다.
7. 합성 궤적 단위 테스트, launch 정적 테스트, 패키지 빌드와 전체 테스트가 통과한다.

## 파일 구조

- Create: `src/go2_rtabmap_launch/launch/vo_odom_comparison.launch.py`
  - SLAM 없이 Go2 odom bridge, RGB-D sync, RGB-D VO, camera static TF를 실행한다.
- Create: `src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py`
  - 토픽, TF 비활성화, 기본 시간 보정값, SLAM 미실행을 정적으로 검증한다.
- Modify: `src/go2_rtabmap_launch/package.xml`
  - 런타임 의존성 `rtabmap_odom`을 선언한다.
- Create: `src/go2_rtabmap_bridge/go2_rtabmap_bridge/odom_comparison.py`
  - ROS I/O와 분리된 pose 정렬, timestamp matching, 통계 계산 로직을 제공한다.
- Create: `src/go2_rtabmap_bridge/go2_rtabmap_bridge/analyze_odom_bag.py`
  - rosbag에서 두 odometry를 읽고 CSV/JSON을 생성하는 CLI를 제공한다.
- Create: `src/go2_rtabmap_bridge/test/test_odom_comparison.py`
  - 합성 궤적으로 matching, 상대 pose, angle wrap, 통계를 검증한다.
- Create: `src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py`
  - CLI 인자와 결과 파일 schema를 검증한다.
- Modify: `src/go2_rtabmap_bridge/setup.py`
  - `analyze_odom_bag` console script를 등록한다.
- Modify: `src/go2_rtabmap_bridge/package.xml`
  - `rosbag2_py`, `rosidl_runtime_py` 실행 의존성을 선언한다.
- Create: `VO_ODOM_COMPARISON.md`
  - 빌드, 실행, rosbag 기록, 분석, 판정 방법을 루트에서 설명한다.

---

### Task 1: 비교 전용 VO/Go2 odometry launch

**Files:**

- Create: `src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py`
- Create: `src/go2_rtabmap_launch/launch/vo_odom_comparison.launch.py`
- Modify: `src/go2_rtabmap_launch/package.xml`

**Interfaces:**

- Consumes:
  - `/utlidar/robot_odom` (`nav_msgs/msg/Odometry`)
  - `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
  - `/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/msg/Image`)
  - `/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)
- Produces:
  - `/odom/go2` (`nav_msgs/msg/Odometry`, frame `go2_odom`, child `base_link`)
  - `/odom/vo` (`nav_msgs/msg/Odometry`, frame `vo_odom`, child `base_link`)
  - `/camera/vo_compare/rgbd_image` (`rtabmap_msgs/msg/RGBDImage`)
- TF ownership:
  - 비교용 Go2 bridge: `publish_tf=false`
  - RGB-D VO: `publish_tf=false`
  - camera static TF만 `base_link -> camera_link`로 발행

- [ ] **Step 1: 비교 launch의 계약을 나타내는 실패 테스트 작성**

```python
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'vo_odom_comparison.launch.py'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'


def test_comparison_launch_runs_only_go2_and_rgbd_odometry():
    text = LAUNCH_FILE.read_text()

    assert "executable='odom_tf_bridge'" in text
    assert "package='rtabmap_sync'" in text
    assert "executable='rgbd_sync'" in text
    assert "package='rtabmap_odom'" in text
    assert "executable='rgbd_odometry'" in text
    assert "package='rtabmap_slam'" not in text
    assert "executable='rtabmap'" not in text


def test_comparison_launch_separates_topics_and_disables_odom_tf():
    text = LAUNCH_FILE.read_text()

    assert "default_value='/odom/go2'" in text
    assert "default_value='/odom/vo'" in text
    assert "'odom_frame_id': 'go2_odom'" in text
    assert "'odom_frame_id': 'vo_odom'" in text
    assert text.count("'publish_tf': False") == 2
    assert "default_value='-0.015'" in text
    assert "'sensor_time_offset_sec': go2_time_offset_param" in text


def test_launch_package_declares_rgbd_odometry_dependency():
    assert '<exec_depend>rtabmap_odom</exec_depend>' in PACKAGE_XML.read_text()
```

- [ ] **Step 2: 테스트가 기능 누락으로 실패하는지 확인**

Run:

```bash
pytest -q src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py
```

Expected: `vo_odom_comparison.launch.py`가 아직 없어 `FileNotFoundError`로 FAIL한다.

- [ ] **Step 3: 비교 launch 최소 구현**

`vo_odom_comparison.launch.py`에 다음 launch argument를 선언한다.

```python
DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw')
DeclareLaunchArgument(
    'depth_topic',
    default_value='/camera/aligned_depth_to_color/image_raw',
)
DeclareLaunchArgument(
    'camera_info_topic',
    default_value='/camera/color/camera_info',
)
DeclareLaunchArgument(
    'rgbd_topic',
    default_value='/camera/vo_compare/rgbd_image',
)
DeclareLaunchArgument(
    'go2_input_odom_topic',
    default_value='/utlidar/robot_odom',
)
DeclareLaunchArgument('go2_odom_topic', default_value='/odom/go2')
DeclareLaunchArgument('vo_odom_topic', default_value='/odom/vo')
DeclareLaunchArgument('go2_sensor_time_offset_sec', default_value='-0.015')
DeclareLaunchArgument('frame_id', default_value='base_link')
DeclareLaunchArgument('camera_frame_id', default_value='camera_link')
DeclareLaunchArgument('camera_x', default_value='0.34')
DeclareLaunchArgument('camera_y', default_value='0.0')
DeclareLaunchArgument('camera_z', default_value='0.095')
DeclareLaunchArgument('camera_roll', default_value='0.0')
DeclareLaunchArgument('camera_pitch', default_value='0.0')
DeclareLaunchArgument('camera_yaw', default_value='0.0')
DeclareLaunchArgument('use_sim_time', default_value='false')
```

launch 파일 상단에서 다음 substitution을 만든다.

```python
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


rgb_topic = LaunchConfiguration('rgb_topic')
depth_topic = LaunchConfiguration('depth_topic')
camera_info_topic = LaunchConfiguration('camera_info_topic')
rgbd_topic = LaunchConfiguration('rgbd_topic')
go2_input_odom_topic = LaunchConfiguration('go2_input_odom_topic')
go2_odom_topic = LaunchConfiguration('go2_odom_topic')
vo_odom_topic = LaunchConfiguration('vo_odom_topic')
frame_id = LaunchConfiguration('frame_id')
use_sim_time = LaunchConfiguration('use_sim_time')
go2_time_offset_param = ParameterValue(
    LaunchConfiguration('go2_sensor_time_offset_sec'),
    value_type=float,
)
```

Go2 bridge는 기존 `odom_tf_bridge`를 재사용하되 출력과 TF 설정만 비교용으로 격리한다.

```python
Node(
    package='go2_rtabmap_bridge',
    executable='odom_tf_bridge',
    name='go2_comparison_odom_bridge',
    output='screen',
    parameters=[{
        'input_odom_topic': go2_input_odom_topic,
        'output_odom_topic': go2_odom_topic,
        'odom_frame_id': 'go2_odom',
        'footprint_frame_id': '',
        'base_frame_id': frame_id,
        'publish_tf': False,
        'planarize_base_frame': False,
        'sensor_time_offset_sec': go2_time_offset_param,
        'use_sim_time': use_sim_time,
    }],
)
```

`rgbd_sync`는 기존 Visual SLAM과 동일한 세 카메라 입력을 사용하되 비교 전용 RGB-D 토픽을 발행한다.

```python
Node(
    package='rtabmap_sync',
    executable='rgbd_sync',
    name='vo_comparison_rgbd_sync',
    output='screen',
    parameters=[{
        'approx_sync': True,
        'approx_sync_max_interval': 0.03,
        'queue_size': 20,
        'sync_queue_size': 20,
        'qos_image': 1,
        'qos_camera_info': 1,
        'use_sim_time': use_sim_time,
    }],
    remappings=[
        ('rgb/image', rgb_topic),
        ('depth/image', depth_topic),
        ('rgb/camera_info', camera_info_topic),
        ('rgbd_image', rgbd_topic),
    ],
)
```

VO는 합쳐진 RGB-D 메시지를 받아 `/odom/vo`만 발행한다. 첫 비교에서는 현재 visual registration과 동일하게 GFTT/ORB 및 최소 inlier 20을 사용하고, GO2 odometry를 initial guess로 넣지 않는다.

```python
Node(
    package='rtabmap_odom',
    executable='rgbd_odometry',
    name='rgbd_vo_comparison',
    output='screen',
    parameters=[{
        'frame_id': frame_id,
        'odom_frame_id': 'vo_odom',
        'publish_tf': False,
        'subscribe_rgbd': True,
        'wait_for_transform': 0.2,
        'qos': 1,
        'qos_camera_info': 1,
        'Vis/FeatureType': '8',
        'Vis/MinInliers': '20',
        'use_sim_time': use_sim_time,
    }],
    remappings=[
        ('rgbd_image', rgbd_topic),
        ('odom', vo_odom_topic),
    ],
)
```

다음 static transform만 추가한다. RealSense가 이미 발행하는 `camera_link -> optical frame` TF는 중복 생성하지 않는다.

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='vo_comparison_camera_static_tf',
    output='screen',
    arguments=[
        '--x', LaunchConfiguration('camera_x'),
        '--y', LaunchConfiguration('camera_y'),
        '--z', LaunchConfiguration('camera_z'),
        '--roll', LaunchConfiguration('camera_roll'),
        '--pitch', LaunchConfiguration('camera_pitch'),
        '--yaw', LaunchConfiguration('camera_yaw'),
        '--frame-id', frame_id,
        '--child-frame-id', LaunchConfiguration('camera_frame_id'),
    ],
)
```

`package.xml`에 다음 의존성을 추가한다.

```xml
<exec_depend>rtabmap_odom</exec_depend>
```

- [ ] **Step 4: launch 계약 테스트 통과 확인**

Run:

```bash
pytest -q src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py
```

Expected: 3 tests PASS.

- [ ] **Step 5: 기존 visual launch 회귀 테스트 확인**

Run:

```bash
pytest -q src/go2_rtabmap_launch/test/test_visual_launch_defaults.py
```

Expected: 기존 테스트 전체 PASS.

- [ ] **Step 6: 변경 범위를 분리해 커밋**

```bash
git add \
  src/go2_rtabmap_launch/launch/vo_odom_comparison.launch.py \
  src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py \
  src/go2_rtabmap_launch/package.xml
git commit -m "feat: add standalone VO odometry comparison launch"
```

---

### Task 2: 궤적 정렬과 timestamp matching 코어

**Files:**

- Create: `src/go2_rtabmap_bridge/test/test_odom_comparison.py`
- Create: `src/go2_rtabmap_bridge/go2_rtabmap_bridge/odom_comparison.py`

**Interfaces:**

- Produces:
  - `PoseSample(stamp_ns, position, orientation)`
  - `MatchedPair(vo, go2, time_gap_ns)`
  - `match_nearest_samples(vo_samples, go2_samples, max_gap_ns)`
  - `relative_pose(origin, sample)`
  - `build_comparison_rows(matched_pairs)`
  - `summarize_comparison(rows, vo_samples)`
- `PoseSample.position`: `tuple[float, float, float]`
- `PoseSample.orientation`: normalized quaternion `(x, y, z, w)`
- `build_comparison_rows()`의 각 row:
  - `stamp_ns`
  - `time_gap_ms`
  - `go2_x_m`, `go2_y_m`, `go2_z_m`, `go2_yaw_rad`
  - `vo_x_m`, `vo_y_m`, `vo_z_m`, `vo_yaw_rad`
  - `position_difference_m`
  - `yaw_difference_rad`

- [ ] **Step 1: 최근접 timestamp matching 실패 테스트 작성**

```python
from go2_rtabmap_bridge.odom_comparison import (
    PoseSample,
    match_nearest_samples,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def sample(stamp_ms, x=0.0, y=0.0, yaw_quaternion=IDENTITY):
    return PoseSample(
        stamp_ns=stamp_ms * 1_000_000,
        position=(x, y, 0.0),
        orientation=yaw_quaternion,
    )


def test_match_nearest_samples_rejects_pairs_over_max_gap():
    go2 = [sample(0), sample(100), sample(200)]
    vo = [sample(47), sample(151), sample(280)]

    pairs = match_nearest_samples(
        vo,
        go2,
        max_gap_ns=50_000_000,
    )

    assert [pair.go2.stamp_ns for pair in pairs] == [0, 200]
    assert [pair.time_gap_ns for pair in pairs] == [47_000_000, 49_000_000]
```

- [ ] **Step 2: matching 테스트 RED 확인**

Run:

```bash
pytest -q \
  src/go2_rtabmap_bridge/test/test_odom_comparison.py::test_match_nearest_samples_rejects_pairs_over_max_gap
```

Expected: `odom_comparison` 모듈 누락으로 FAIL.

- [ ] **Step 3: `PoseSample`, `MatchedPair`, 최근접 matching 최소 구현**

```python
from bisect import bisect_left
from dataclasses import dataclass


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


def match_nearest_samples(vo_samples, go2_samples, max_gap_ns):
    ordered_go2 = sorted(go2_samples, key=lambda item: item.stamp_ns)
    stamps = [item.stamp_ns for item in ordered_go2]
    pairs = []
    for vo in sorted(vo_samples, key=lambda item: item.stamp_ns):
        index = bisect_left(stamps, vo.stamp_ns)
        candidates = ordered_go2[max(0, index - 1):min(len(ordered_go2), index + 1)]
        if not candidates:
            continue
        go2 = min(candidates, key=lambda item: abs(item.stamp_ns - vo.stamp_ns))
        gap_ns = abs(go2.stamp_ns - vo.stamp_ns)
        if gap_ns <= max_gap_ns:
            pairs.append(MatchedPair(vo=vo, go2=go2, time_gap_ns=gap_ns))
    return pairs
```

- [ ] **Step 4: matching 테스트 GREEN 확인**

Run:

```bash
pytest -q \
  src/go2_rtabmap_bridge/test/test_odom_comparison.py::test_match_nearest_samples_rejects_pairs_over_max_gap
```

Expected: PASS.

- [ ] **Step 5: 공통 원점 상대 pose와 angle wrap 실패 테스트 추가**

```python
import math

from go2_rtabmap_bridge.odom_comparison import (
    build_comparison_rows,
    match_nearest_samples,
    yaw_quaternion,
)


def test_comparison_uses_each_first_pose_as_common_relative_origin():
    go2 = [
        sample(0, x=10.0, y=-2.0, yaw_quaternion=yaw_quaternion(0.5)),
        sample(100, x=10.0 + math.cos(0.5), y=-2.0 + math.sin(0.5),
               yaw_quaternion=yaw_quaternion(0.6)),
    ]
    vo = [
        sample(0, x=0.0, y=0.0, yaw_quaternion=yaw_quaternion(0.0)),
        sample(100, x=1.0, y=0.0, yaw_quaternion=yaw_quaternion(0.1)),
    ]

    rows = build_comparison_rows(
        match_nearest_samples(vo, go2, max_gap_ns=50_000_000)
    )

    assert rows[0]['position_difference_m'] == 0.0
    assert abs(rows[1]['position_difference_m']) < 1e-9
    assert abs(rows[1]['yaw_difference_rad']) < 1e-9


def test_yaw_difference_wraps_at_pi_boundary():
    go2 = [
        sample(0),
        sample(100, yaw_quaternion=yaw_quaternion(math.radians(179.0))),
    ]
    vo = [
        sample(0),
        sample(100, yaw_quaternion=yaw_quaternion(math.radians(-179.0))),
    ]

    rows = build_comparison_rows(
        match_nearest_samples(vo, go2, max_gap_ns=50_000_000)
    )

    assert math.isclose(
        abs(rows[1]['yaw_difference_rad']),
        math.radians(2.0),
        abs_tol=1e-9,
    )
```

- [ ] **Step 6: 상대 transform과 row 생성 테스트 RED 확인**

Run:

```bash
pytest -q src/go2_rtabmap_bridge/test/test_odom_comparison.py
```

Expected: `yaw_quaternion` 또는 `build_comparison_rows` 미구현으로 FAIL.

- [ ] **Step 7: 3D rigid transform 기반 상대 pose 구현**

다음 계산을 그대로 구현한다.

```python
import math


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
    rotated = quaternion_multiply(
        quaternion_multiply(
            normalized_quaternion(quaternion),
            (vector[0], vector[1], vector[2], 0.0),
        ),
        quaternion_inverse(quaternion),
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


def relative_pose(origin, sample):
    origin_inverse = quaternion_inverse(origin.orientation)
    delta = tuple(
        sample_value - origin_value
        for sample_value, origin_value in zip(sample.position, origin.position)
    )
    return PoseSample(
        stamp_ns=sample.stamp_ns,
        position=rotate_vector(origin_inverse, delta),
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
        go2 = relative_pose(go2_origin, pair.go2)
        vo = relative_pose(vo_origin, pair.vo)
        position_delta = tuple(
            vo_value - go2_value
            for vo_value, go2_value in zip(vo.position, go2.position)
        )
        go2_yaw = yaw_from_quaternion(go2.orientation)
        vo_yaw = yaw_from_quaternion(vo.orientation)
        rows.append({
            'stamp_ns': pair.vo.stamp_ns,
            'time_gap_ms': pair.time_gap_ns / 1_000_000.0,
            'go2_x_m': go2.position[0],
            'go2_y_m': go2.position[1],
            'go2_z_m': go2.position[2],
            'go2_yaw_rad': go2_yaw,
            'vo_x_m': vo.position[0],
            'vo_y_m': vo.position[1],
            'vo_z_m': vo.position[2],
            'vo_yaw_rad': vo_yaw,
            'position_difference_m': math.sqrt(
                sum(value * value for value in position_delta)
            ),
            'yaw_difference_rad': wrap_angle(vo_yaw - go2_yaw),
        })
    return rows
```

`relative_pose()`는 단순 위치 빼기가 아니라 다음 rigid transform을 사용한다.

```text
T_relative = inverse(T_origin) * T_sample
```

`build_comparison_rows()`는 첫 matched pair의 Go2 pose와 VO pose를 각각 원점으로 사용한다. 두 상대 position의 Euclidean norm 차이를 `position_difference_m`로, `wrap_angle(vo_yaw - go2_yaw)`를 `yaw_difference_rad`로 저장한다.

- [ ] **Step 8: 상대 pose 테스트 GREEN 확인**

Run:

```bash
pytest -q src/go2_rtabmap_bridge/test/test_odom_comparison.py
```

Expected: 전체 PASS.

- [ ] **Step 9: 통계와 VO gap 실패 테스트 추가**

```python
from go2_rtabmap_bridge.odom_comparison import summarize_comparison


def test_summary_reports_divergence_sync_and_vo_tracking_gaps():
    go2 = [sample(stamp) for stamp in (0, 100, 200, 1000)]
    vo = [
        sample(0),
        sample(100, x=0.1),
        sample(200, x=0.2),
        sample(1000, x=1.0),
    ]
    pairs = match_nearest_samples(vo, go2, max_gap_ns=50_000_000)
    rows = build_comparison_rows(pairs)

    summary = summarize_comparison(rows, vo_samples=vo, long_gap_sec=0.5)

    assert summary['matched_pairs'] == 4
    assert summary['vo_long_gap_count'] == 1
    assert summary['vo_max_gap_sec'] == 0.8
    assert 'position_difference_rmse_m' in summary
    assert 'yaw_difference_p95_deg' in summary
    assert 'time_gap_p95_ms' in summary
```

- [ ] **Step 10: 통계 테스트 RED 확인**

Run:

```bash
pytest -q \
  src/go2_rtabmap_bridge/test/test_odom_comparison.py::test_summary_reports_divergence_sync_and_vo_tracking_gaps
```

Expected: `summarize_comparison` 미구현으로 FAIL.

- [ ] **Step 11: 요약 통계 최소 구현**

`summarize_comparison()`은 다음 key를 항상 반환한다.

```python
{
    'matched_pairs': int,
    'duration_sec': float,
    'time_gap_median_ms': float,
    'time_gap_p95_ms': float,
    'time_gap_max_ms': float,
    'go2_path_length_m': float,
    'vo_path_length_m': float,
    'final_position_difference_m': float,
    'position_difference_rmse_m': float,
    'position_difference_p95_m': float,
    'final_yaw_difference_deg': float,
    'yaw_difference_rmse_deg': float,
    'yaw_difference_p95_deg': float,
    'vo_effective_rate_hz': float,
    'vo_max_gap_sec': float,
    'vo_long_gap_count': int,
}
```

다음 helper와 계산식을 사용한다.

```python
import numpy as np


def path_length(rows, prefix):
    total = 0.0
    previous = None
    for row in rows:
        current = (
            row[f'{prefix}_x_m'],
            row[f'{prefix}_y_m'],
            row[f'{prefix}_z_m'],
        )
        if previous is not None:
            total += math.sqrt(
                sum(
                    (current_value - previous_value) ** 2
                    for current_value, previous_value in zip(current, previous)
                )
            )
        previous = current
    return total


def rmse(values):
    array = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(array * array)))


def percentile(values, percentile_value):
    return float(np.percentile(np.asarray(values, dtype=float), percentile_value))


def summarize_comparison(rows, vo_samples, long_gap_sec=0.5):
    if not rows:
        raise ValueError(
            'No odometry pairs matched within the configured time gap'
        )

    ordered_vo = sorted(vo_samples, key=lambda item: item.stamp_ns)
    vo_gaps_sec = [
        (current.stamp_ns - previous.stamp_ns) / 1_000_000_000.0
        for previous, current in zip(ordered_vo, ordered_vo[1:])
    ]
    duration_sec = (
        (rows[-1]['stamp_ns'] - rows[0]['stamp_ns']) / 1_000_000_000.0
    )
    vo_duration_sec = (
        (ordered_vo[-1].stamp_ns - ordered_vo[0].stamp_ns) / 1_000_000_000.0
        if len(ordered_vo) >= 2
        else 0.0
    )
    position_difference = [
        row['position_difference_m']
        for row in rows
    ]
    yaw_difference_deg = [
        math.degrees(abs(row['yaw_difference_rad']))
        for row in rows
    ]
    time_gap_ms = [row['time_gap_ms'] for row in rows]

    return {
        'matched_pairs': len(rows),
        'duration_sec': duration_sec,
        'time_gap_median_ms': float(np.median(time_gap_ms)),
        'time_gap_p95_ms': percentile(time_gap_ms, 95),
        'time_gap_max_ms': max(time_gap_ms),
        'go2_path_length_m': path_length(rows, 'go2'),
        'vo_path_length_m': path_length(rows, 'vo'),
        'final_position_difference_m': position_difference[-1],
        'position_difference_rmse_m': rmse(position_difference),
        'position_difference_p95_m': percentile(position_difference, 95),
        'final_yaw_difference_deg': math.degrees(
            rows[-1]['yaw_difference_rad']
        ),
        'yaw_difference_rmse_deg': rmse(yaw_difference_deg),
        'yaw_difference_p95_deg': percentile(yaw_difference_deg, 95),
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
```

빈 매칭 결과에는 다음 예외를 발생시킨다.

```python
raise ValueError('No odometry pairs matched within the configured time gap')
```

- [ ] **Step 12: 코어 전체 테스트 통과 확인**

Run:

```bash
pytest -q src/go2_rtabmap_bridge/test/test_odom_comparison.py
```

Expected: 전체 PASS.

- [ ] **Step 13: 비교 코어 커밋**

```bash
git add \
  src/go2_rtabmap_bridge/go2_rtabmap_bridge/odom_comparison.py \
  src/go2_rtabmap_bridge/test/test_odom_comparison.py
git commit -m "feat: add odometry trajectory comparison core"
```

---

### Task 3: rosbag 분석 CLI와 CSV/JSON 출력

**Files:**

- Create: `src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py`
- Create: `src/go2_rtabmap_bridge/go2_rtabmap_bridge/analyze_odom_bag.py`
- Modify: `src/go2_rtabmap_bridge/setup.py`
- Modify: `src/go2_rtabmap_bridge/package.xml`

**Interfaces:**

- Command:

```bash
ros2 run go2_rtabmap_bridge analyze_odom_bag \
  <bag_directory> \
  --go2-topic /odom/go2 \
  --vo-topic /odom/vo \
  --max-time-gap-ms 50 \
  --long-vo-gap-ms 500 \
  --output-prefix results/vo_go2
```

- Produces:
  - `results/vo_go2_samples.csv`
  - `results/vo_go2_summary.json`
- Exit codes:
  - `0`: 분석 완료
  - non-zero: bag 열기 실패, 토픽 누락, 타입 불일치, 유효 matching 없음

- [ ] **Step 1: CLI parser와 출력 schema 실패 테스트 작성**

```python
import json

from go2_rtabmap_bridge.analyze_odom_bag import (
    build_argument_parser,
    write_outputs,
)


def test_cli_defaults_use_isolated_odometry_topics():
    args = build_argument_parser().parse_args(['/tmp/example_bag'])

    assert args.go2_topic == '/odom/go2'
    assert args.vo_topic == '/odom/vo'
    assert args.max_time_gap_ms == 50.0
    assert args.long_vo_gap_ms == 500.0


def test_write_outputs_creates_csv_and_json(tmp_path):
    rows = [{
        'stamp_ns': 100,
        'time_gap_ms': 1.0,
        'go2_x_m': 0.0,
        'go2_y_m': 0.0,
        'go2_z_m': 0.0,
        'go2_yaw_rad': 0.0,
        'vo_x_m': 0.0,
        'vo_y_m': 0.0,
        'vo_z_m': 0.0,
        'vo_yaw_rad': 0.0,
        'position_difference_m': 0.0,
        'yaw_difference_rad': 0.0,
    }]
    summary = {'matched_pairs': 1}

    csv_path, json_path = write_outputs(
        tmp_path / 'comparison',
        rows,
        summary,
    )

    assert csv_path.name == 'comparison_samples.csv'
    assert json_path.name == 'comparison_summary.json'
    assert json.loads(json_path.read_text())['matched_pairs'] == 1
    assert 'position_difference_m' in csv_path.read_text().splitlines()[0]
```

- [ ] **Step 2: CLI 테스트 RED 확인**

Run:

```bash
pytest -q src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py
```

Expected: `analyze_odom_bag` 모듈 누락으로 FAIL.

- [ ] **Step 3: parser와 output writer 최소 구현**

`build_argument_parser()`는 다음 인자를 구현한다.

```text
bag_directory                 positional Path
--go2-topic                   default /odom/go2
--vo-topic                    default /odom/vo
--max-time-gap-ms             default 50.0
--long-vo-gap-ms              default 500.0
--output-prefix               default odom_comparison
```

`write_outputs()`는 부모 디렉터리를 만들고 고정된 column 순서로 UTF-8 CSV와 들여쓰기 2칸 JSON을 쓴다.

```python
import argparse
import csv
import json
from pathlib import Path


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
        description='Compare VO and Go2 odometry stored in a ROS 2 bag.'
    )
    parser.add_argument('bag_directory', type=Path)
    parser.add_argument('--go2-topic', default='/odom/go2')
    parser.add_argument('--vo-topic', default='/odom/vo')
    parser.add_argument('--max-time-gap-ms', type=float, default=50.0)
    parser.add_argument('--long-vo-gap-ms', type=float, default=500.0)
    parser.add_argument(
        '--output-prefix',
        type=Path,
        default=Path('odom_comparison'),
    )
    return parser


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
```

- [ ] **Step 4: parser/output 테스트 GREEN 확인**

Run:

```bash
pytest -q src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py
```

Expected: 2 tests PASS.

- [ ] **Step 5: rosbag topic reader 실패 테스트 추가**

rosbag I/O 자체를 mock으로 검증하지 않는다. 테스트에서 임시 sqlite3 bag을 생성하고 두 `Odometry` 메시지를 기록한 뒤 다시 읽는다.

```python
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialWriter,
    StorageOptions,
    TopicMetadata,
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
    for topic_name, messages in messages_by_topic.items():
        for message in messages:
            stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            writer.write(topic_name, serialize_message(message), stamp_ns)
    del writer
    return bag_path


def test_read_odometry_topics_from_real_rosbag(tmp_path):
    bag_path = write_test_bag(
        tmp_path / 'bag',
        {
            '/odom/go2': [odometry(stamp_ns=1_000_000_000, x=0.0)],
            '/odom/vo': [odometry(stamp_ns=1_001_000_000, x=0.0)],
        },
    )

    samples = read_odometry_topics(
        bag_path,
        ['/odom/go2', '/odom/vo'],
    )

    assert samples['/odom/go2'][0].stamp_ns == 1_000_000_000
    assert samples['/odom/vo'][0].stamp_ns == 1_001_000_000
```

- [ ] **Step 6: rosbag reader 테스트 RED 확인**

Run:

```bash
pytest -q \
  src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py::test_read_odometry_topics_from_real_rosbag
```

Expected: `read_odometry_topics` 미구현으로 FAIL.

- [ ] **Step 7: 실제 rosbag2 reader 구현**

다음 ROS 2 API를 사용한다.

```python
from rclpy.serialization import deserialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageFilter,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message
```

구현 규칙:

1. `SequentialReader.open()`으로 bag을 연다.
2. `get_all_topics_and_types()`에서 요청 토픽 존재 여부와 타입을 확인한다.
3. 두 토픽 타입이 모두 `nav_msgs/msg/Odometry`인지 확인한다.
4. `reader.set_filter(StorageFilter(topics=requested_topics))`를 적용한다.
5. 메시지의 `header.stamp`를 비교 timestamp로 사용한다.
6. `header.stamp`가 0이면 bag record timestamp를 fallback으로 사용한다.
7. 각 토픽 샘플을 timestamp 오름차순으로 반환한다.

```python
def pose_sample_from_odometry(message, recorded_stamp_ns):
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
        StorageOptions(uri=str(bag_directory), storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    topic_types = {
        metadata.name: metadata.type
        for metadata in reader.get_all_topics_and_types()
    }
    missing = [
        topic
        for topic in requested_topics
        if topic not in topic_types
    ]
    if missing:
        raise ValueError(
            f'Missing odometry topics in bag: {", ".join(missing)}'
        )
    invalid = [
        topic
        for topic in requested_topics
        if topic_types[topic] != 'nav_msgs/msg/Odometry'
    ]
    if invalid:
        raise ValueError(
            f'Expected nav_msgs/msg/Odometry: {", ".join(invalid)}'
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
            pose_sample_from_odometry(message, recorded_stamp_ns)
        )
    for topic in samples:
        samples[topic].sort(key=lambda item: item.stamp_ns)
    return samples
```

- [ ] **Step 8: rosbag reader 테스트 GREEN 확인**

Run:

```bash
pytest -q src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py
```

Expected: 전체 PASS.

- [ ] **Step 9: `main()` 연결과 entry point 등록**

`main()` 흐름을 다음 순서로 고정한다.

```python
args = build_argument_parser().parse_args()
samples = read_odometry_topics(
    args.bag_directory,
    [args.go2_topic, args.vo_topic],
)
pairs = match_nearest_samples(
    samples[args.vo_topic],
    samples[args.go2_topic],
    max_gap_ns=round(args.max_time_gap_ms * 1_000_000),
)
rows = build_comparison_rows(pairs)
summary = summarize_comparison(
    rows,
    vo_samples=samples[args.vo_topic],
    long_gap_sec=args.long_vo_gap_ms / 1000.0,
)
write_outputs(args.output_prefix, rows, summary)
```

`setup.py`에 다음 entry point를 추가한다.

```python
'analyze_odom_bag = go2_rtabmap_bridge.analyze_odom_bag:main',
```

`package.xml`에 다음 의존성을 추가한다.

```xml
<exec_depend>rosbag2_py</exec_depend>
<exec_depend>rosidl_runtime_py</exec_depend>
```

- [ ] **Step 10: bridge 패키지 테스트 전체 확인**

Run:

```bash
pytest -q src/go2_rtabmap_bridge/test
```

Expected: 기존 odom bridge 테스트와 새 비교 테스트 전체 PASS.

- [ ] **Step 11: 분석 CLI 커밋**

```bash
git add \
  src/go2_rtabmap_bridge/go2_rtabmap_bridge/analyze_odom_bag.py \
  src/go2_rtabmap_bridge/setup.py \
  src/go2_rtabmap_bridge/package.xml \
  src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py
git commit -m "feat: analyze VO and Go2 odometry bags"
```

---

### Task 4: 실행 문서와 실기 비교 절차

**Files:**

- Create: `VO_ODOM_COMPARISON.md`

**Interfaces:**

- Documents:
  - 빌드
  - 토픽 사전 확인
  - 비교 launch
  - 최소/재현용 rosbag 기록
  - 분석 명령
  - 결과 판독
  - 실패 조건과 주의사항

- [ ] **Step 1: 실행 문서 검증 테스트 추가**

`src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py`에 다음 테스트를 추가한다.

```python
RUNBOOK = PACKAGE_ROOT.parents[1] / 'VO_ODOM_COMPARISON.md'


def test_comparison_runbook_has_reproducible_commands_and_caveat():
    text = RUNBOOK.read_text()

    assert 'vo_odom_comparison.launch.py' in text
    assert '/odom/go2' in text
    assert '/odom/vo' in text
    assert 'ros2 bag record' in text
    assert 'analyze_odom_bag' in text
    assert 'ground truth가 아니다' in text
```

- [ ] **Step 2: 문서 테스트 RED 확인**

Run:

```bash
pytest -q \
  src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py::test_comparison_runbook_has_reproducible_commands_and_caveat
```

Expected: `VO_ODOM_COMPARISON.md` 누락으로 FAIL.

- [ ] **Step 3: 빌드와 실행 절차 작성**

문서에 다음 명령을 그대로 포함한다.

```bash
colcon build --symlink-install \
  --packages-select go2_rtabmap_bridge go2_rtabmap_launch
source install/setup.bash

ros2 launch go2_rtabmap_launch vo_odom_comparison.launch.py
```

실행 전 다음 입력을 확인한다.

```bash
ros2 topic hz /utlidar/robot_odom
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

출력을 확인한다.

```bash
ros2 topic hz /odom/go2
ros2 topic hz /odom/vo
ros2 topic echo /odom/vo --once
```

- [ ] **Step 4: rosbag 기록 절차 작성**

빠른 비교용 최소 기록:

```bash
ros2 bag record \
  -o bags/vo_go2_compare_minimal \
  /odom/go2 \
  /odom/vo
```

VO 설정을 바꿔 재처리할 수 있는 재현용 기록:

```bash
ros2 bag record \
  -o bags/vo_go2_compare_reproducible \
  /utlidar/robot_odom \
  /odom/go2 \
  /odom/vo \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info \
  /tf \
  /tf_static
```

- [ ] **Step 5: 분석과 판독 절차 작성**

```bash
mkdir -p results
ros2 run go2_rtabmap_bridge analyze_odom_bag \
  bags/vo_go2_compare_minimal \
  --output-prefix results/vo_go2
```

문서에서 다음 기준을 설명한다.

- `time_gap_p95_ms <= 33 ms`: 30 Hz 카메라 한 프레임 이내의 양호한 비교 동기.
- `vo_max_gap_sec > 0.5` 또는 `vo_long_gap_count > 0`: tracking loss 또는 VO 출력 중단 후보.
- `position_difference_*`, `yaw_difference_*`: 두 추정기의 발산량이며 정확도 오차가 아니다.
- 시작점으로 돌아오는 폐루프 주행에서는 각 odometry의 최종 상대 pose norm도 별도로 확인한다.
- 정지 구간에서는 VO jitter와 Go2 yaw drift를 구분해 본다.
- 저특징 직선, 빠른 회전, 정상 속도 전체 루프를 별도 bag으로 기록한다.

- [ ] **Step 6: 문서 테스트 GREEN 확인**

Run:

```bash
pytest -q \
  src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py
```

Expected: 전체 PASS.

- [ ] **Step 7: 실행 문서 커밋**

```bash
git add \
  VO_ODOM_COMPARISON.md \
  src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py
git commit -m "docs: add VO and Go2 odometry comparison runbook"
```

---

### Task 5: 통합 빌드와 실기 전 검증

**Files:**

- Verify only: Task 1-4에서 생성·수정한 파일

**Interfaces:**

- Produces:
  - 설치된 `vo_odom_comparison.launch.py`
  - 설치된 `analyze_odom_bag` executable
  - 통과한 패키지/단위 테스트 결과

- [ ] **Step 1: 두 패키지 선택 빌드**

Run:

```bash
colcon build --symlink-install \
  --packages-select go2_rtabmap_bridge go2_rtabmap_launch
```

Expected: 두 패키지 build 성공.

- [ ] **Step 2: 설치 공간을 source한 뒤 executable 확인**

Run:

```bash
source install/setup.bash
ros2 pkg executables go2_rtabmap_bridge
```

Expected output에 다음 두 줄이 모두 존재한다.

```text
go2_rtabmap_bridge odom_tf_bridge
go2_rtabmap_bridge analyze_odom_bag
```

- [ ] **Step 3: 전체 패키지 테스트**

Run:

```bash
colcon test \
  --packages-select go2_rtabmap_bridge go2_rtabmap_launch \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `0 tests failed`.

- [ ] **Step 4: launch 구성 출력 확인**

Run:

```bash
ros2 launch go2_rtabmap_launch \
  vo_odom_comparison.launch.py \
  --show-args
```

Expected: camera topic, `/odom/go2`, `/odom/vo`, `-0.015`, camera extrinsic 인자가 표시된다.

- [ ] **Step 5: ROS graph 수동 smoke test**

카메라와 Go2가 연결된 환경에서 launch 후 확인한다.

```bash
ros2 node list
ros2 topic list
ros2 topic hz /odom/go2
ros2 topic hz /odom/vo
```

Expected:

- `go2_comparison_odom_bridge`, `vo_comparison_rgbd_sync`, `rgbd_vo_comparison`이 존재한다.
- `/rtabmap/rtabmap` 노드가 존재하지 않는다.
- `/odom/go2`는 Go2 입력과 유사한 고주파로 발행된다.
- `/odom/vo`는 카메라 처리율 범위에서 지속적으로 발행된다.

- [ ] **Step 6: TF 중복 여부 확인**

Run:

```bash
ros2 run tf2_tools view_frames
```

Expected: 비교 노드가 `go2_odom -> base_link` 또는 `vo_odom -> base_link` TF를 발행하지 않는다. 기존 시스템이 별도로 실행 중이면 그 시스템이 소유한 `odom -> base_link`만 유지된다.

- [ ] **Step 7: 최종 diff 검토**

Run:

```bash
git status --short
git diff --check
git diff -- \
  src/go2_rtabmap_launch/launch/vo_odom_comparison.launch.py \
  src/go2_rtabmap_launch/test/test_vo_odom_comparison_launch.py \
  src/go2_rtabmap_launch/package.xml \
  src/go2_rtabmap_bridge/go2_rtabmap_bridge/odom_comparison.py \
  src/go2_rtabmap_bridge/go2_rtabmap_bridge/analyze_odom_bag.py \
  src/go2_rtabmap_bridge/test/test_odom_comparison.py \
  src/go2_rtabmap_bridge/test/test_analyze_odom_bag_cli.py \
  src/go2_rtabmap_bridge/setup.py \
  src/go2_rtabmap_bridge/package.xml \
  VO_ODOM_COMPARISON.md
```

Expected: whitespace error가 없고, 기존 Visual SLAM 파일에는 diff가 없다.

- [ ] **Step 8: 검증 결과만 커밋할 변경이 있는지 확인**

검증 과정에서 코드나 문서 수정이 필요했다면 해당 파일만 명시적으로 stage하고 다음 메시지로 커밋한다.

```bash
git commit -m "test: verify VO and Go2 odometry comparison workflow"
```

수정이 없으면 빈 커밋은 만들지 않는다.

## 실기 실험 순서

구현 완료 후 동일한 시작 위치에서 다음 세 bag을 각각 기록한다.

1. **정지 2분**
   - Go2 yaw drift와 VO 정지 jitter/출력 중단을 확인한다.
2. **저특징 복도 왕복**
   - VO feature 부족 시 tracking loss와 재초기화 여부를 확인한다.
3. **기존 Visual SLAM 검증 경로 전체 루프**
   - 주행거리 차이, 최종 위치 차이, yaw 발산, VO gap을 비교한다.

각 bag은 독립된 output prefix로 분석한다.

```bash
ros2 run go2_rtabmap_bridge analyze_odom_bag \
  bags/vo_go2_stationary \
  --output-prefix results/stationary

ros2 run go2_rtabmap_bridge analyze_odom_bag \
  bags/vo_go2_corridor \
  --output-prefix results/corridor

ros2 run go2_rtabmap_bridge analyze_odom_bag \
  bags/vo_go2_full_loop \
  --output-prefix results/full_loop
```

## 판정 원칙

- GO2와 VO 중 어느 쪽이 더 정확한지는 두 odometry만으로 확정하지 않는다.
- 먼저 VO가 전 구간에서 끊기지 않고 출력되는지 확인한다.
- tracking이 유지될 때 두 상대 궤적의 위치·yaw 발산이 어느 동작 구간에서 커지는지 확인한다.
- 폐루프 최종 pose와 정지 구간 drift는 비교 판단에 활용할 수 있지만 독립 ground truth를 대체하지 않는다.
- VO가 안정적이면 다음 단계에서만 GO2 odom 대체 또는 sensor fusion 실험 계획을 별도로 작성한다.

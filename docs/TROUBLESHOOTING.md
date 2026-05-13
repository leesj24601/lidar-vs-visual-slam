# Go2 RTAB-Map SLAM Troubleshooting

> 이 문서는 프로젝트 진행 중 실제로 마주친 문제와 해결 과정을 기록한다.
> 목표는 “RTAB-Map을 실행했다”가 아니라, Go2 실기체 데이터를 RTAB-Map 입력으로 연결하면서 생긴
> 시간축, TF, QoS, PointCloud2 layout 문제를 어떻게 진단하고 해결했는지 재사용 가능하게 남기는 것이다.

## 빠른 진단 순서

문제가 생기면 아래 순서로 좁힌다.

```bash
# 1. Go2 원천 토픽
ros2 topic list --no-daemon | grep utlidar
ros2 topic info /utlidar/robot_odom --verbose --no-daemon
ros2 topic info /utlidar/cloud_deskewed --verbose --no-daemon
ros2 topic hz /utlidar/robot_odom
ros2 topic hz /utlidar/cloud_deskewed

# 2. 브릿지 출력
ros2 topic hz /odom
ros2 topic hz /scan_cloud
ros2 topic echo /scan_cloud --once --field header
ros2 run tf2_ros tf2_echo odom base_link

# 3. RTAB-Map 출력
ros2 topic hz /rtabmap/mapData
ros2 topic echo /rtabmap/info --field loop_closure_id
ros2 node info /rtabmap/rtabmap
```

---

## Issue 1: `/utlidar/*` timestamp가 현재 시간보다 약 461일 과거

### Symptom

- `/utlidar/robot_odom`, `/utlidar/cloud_deskewed`의 `header.stamp`가 현재 ROS 시간과 크게 다름.
- TF lookup이나 RTAB-Map 동기화에서 시간축 불일치 문제가 생길 수 있음.

### Root Cause

- `unitree_lidar_server`가 자체 시간축으로 `/utlidar/*` 메시지를 발행한다.
- `.161` lidar server에는 SSH 접근이 안 되어 원천 수정이 불가능하다.

### Fix

- 첫 `/utlidar/robot_odom`에서 한 번만 offset 계산:

```text
offset = now() - sensor_stamp
```

- 이후 odom과 cloud 모두에 같은 offset 적용:

```text
corrected_stamp = original_stamp + offset
```

- 매 메시지마다 `now()`로 덮어쓰지 않는다.

### Why

`now()`로 단순 대체하면 odom과 cloud 사이의 상대 시간이 깨진다. 같은 offset을 적용하면 현재 ROS 시간축으로 옮기면서도 odom-cloud 상대 시간 관계를 보존할 수 있다.

### Verification

```bash
ros2 topic hz /odom
ros2 topic hz /scan_cloud
ros2 run tf2_ros tf2_echo odom base_link
```

2026-04-12 실측:

- `/odom`: 약 150~152Hz
- `/scan_cloud`: 약 14.6~14.8Hz
- `tf2_echo odom base_link`: 정상 출력

---

## Issue 2: RTAB-Map `odom_frame_id='odom'` 설정 시 `/odom` 토픽을 구독하지 않음

### Symptom

- 계획상 `/odom`과 `/scan_cloud`를 approximate sync로 넣으려 했지만, `odom_frame_id='odom'`을 설정하면 RTAB-Map이 `/odom` 토픽을 구독하지 않음.

### Root Cause

RTAB-Map의 `odom_frame_id`는 “`/odom` 토픽의 frame 이름”이 아니다.

```text
odom_frame_id 설정됨:
  /odom 토픽 대신 TF에서 odometry를 lookup

odom_frame_id 비움:
  /odom 토픽 구독
```

### Fix

- `odom_frame_id`는 생략/빈 값 유지.
- `/odom`, `/scan_cloud`를 절대 경로로 remap:

```python
remappings=[
    ('odom', '/odom'),
    ('scan_cloud', '/scan_cloud'),
]
```

### Verification

```bash
ros2 node info /rtabmap/rtabmap
```

기대값:

```text
Subscribers:
  /odom: nav_msgs/msg/Odometry
  /scan_cloud: sensor_msgs/msg/PointCloud2
```

---

## Issue 3: cloud 변환 TF lookup 방향 혼동

### Symptom

- `/utlidar/cloud_deskewed`는 `frame_id=odom`이다.
- RTAB-Map에는 `base_link` 기준 `/scan_cloud`로 넣고 싶다.
- `odom -> base_link` TF를 발행하다 보니 lookup 인자 순서를 반대로 넣기 쉽다.

### Root Cause

`tf2` lookup 인자 순서는 다음이다.

```python
lookup_transform(target_frame, source_frame, time)
```

TF 트리는 `odom -> base_link`가 맞지만, cloud 변환은 “odom에 있는 point를 base_link로 표현”하는 것이므로:

```python
lookup_transform('base_link', 'odom', corrected_time)
```

이어야 한다.

### Fix

브릿지 cloud callback에서:

```python
transform = tf_buffer.lookup_transform(
    'base_link',
    cloud_msg.header.frame_id,  # 'odom'
    corrected_time,
)
```

### Verification

```bash
ros2 topic echo /scan_cloud --once --field header
```

기대값:

```text
frame_id: base_link
```

---

## Issue 4: 같은 노드에서 TF 발행 후 lookup할 때 callback timing 문제가 생김

### Symptom

- 브릿지 노드는 odom callback에서 TF를 발행하고, cloud callback에서 그 TF를 lookup한다.
- `SingleThreadedExecutor`에서 cloud callback이 `lookup_transform(timeout=...)`으로 기다리면 새 odom callback이 실행되지 못할 수 있다.

### Root Cause

한 스레드에서 cloud callback이 대기 중이면, 같은 노드의 odom callback이 그동안 처리되지 못한다.
즉 “TF가 들어오길 기다리지만 TF를 만드는 callback이 실행되지 못하는” 상황이 생길 수 있다.

### Fix

odom callback에서 만든 `TransformStamped`를 `/tf`로 발행하면서 내부 `tf_buffer`에도 즉시 넣는다.

```python
self._tf_broadcaster.sendTransform(transform)
self._tf_buffer.set_transform(transform, self.get_name())
```

### Verification

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic hz /scan_cloud
```

---

## Issue 5: Go2 `PointCloud2` padding 때문에 `do_transform_cloud()`가 실패

### Symptom

실제 Go2 `/utlidar/cloud_deskewed`로 브릿지를 실행하면 다음 오류로 노드가 종료됨.

```text
AssertionError: PointFields and structured NumPy array dtype do not match
```

### Root Cause

실제 Go2 cloud layout:

```text
x         offset 0   FLOAT32
y         offset 4   FLOAT32
z         offset 8   FLOAT32
intensity offset 16  FLOAT32
point_step = 32
```

필드 값은 16바이트 분량이지만 점 하나의 stride는 32바이트다. 중간과 뒤쪽에 padding이 있다.
`tf2_sensor_msgs.do_transform_cloud()`는 변환 후 새 cloud를 만들면서 이 padding 있는 dtype/layout을 그대로 재생성하지 못했다.

### Fix

브릿지에서 raw `PointCloud2.data` layout을 보존하고 `x/y/z` float32 값만 numpy view로 직접 변환한다.

```text
원본 byte layout 복사
  x/y/z 위치만 변환된 값으로 덮어씀
  intensity 유지
  padding 유지
  point_step=32 유지
```

### Verification

```bash
ros2 topic echo /scan_cloud --once --field fields
ros2 topic echo /scan_cloud --once --field point_step
ros2 topic echo /scan_cloud --once --field header
```

2026-04-12 실측:

```text
/scan_cloud fields: x, y, z, intensity
/scan_cloud point_step: 32
/scan_cloud header.frame_id: base_link
```

---

## Issue 6: RTAB-Map과 함께 실행하면 `/scan_cloud` 처리율이 14.7Hz에서 8~9Hz로 떨어짐

### Symptom

브릿지 단독에서는 `/scan_cloud`가 약 14.7Hz였지만, RTAB-Map과 함께 실행하자 일시적으로 8~9Hz까지 떨어짐.

로그:

```text
Dropping cloud; transform to base_link unavailable:
Lookup would require extrapolation into the future.
Requested time ... but the latest data is at time ...
```

### Root Cause

cloud timestamp가 브릿지 내부 최신 odom TF보다 몇 ms~수십 ms 앞서는 경우가 있었다.

```text
cloud timestamp: 10.100
latest odom TF:  10.095
```

정확한 timestamp lookup만 허용하면 이런 cloud는 future extrapolation으로 drop된다.

### Fix

1. 정확한 timestamp로 TF lookup 시도
2. 실패하면 최신 TF lookup
3. cloud timestamp와 최신 TF timestamp 차이가 `tf_latest_fallback_tolerance_sec` 이내이면 최신 TF 사용
4. 기본 tolerance: 0.2초

### Why

작은 timing jitter 때문에 cloud를 버리는 것보다, 가까운 최신 TF로 변환해 입력 주기를 유지하는 것이 더 안정적이다.
0.2초는 최대 허용치이며, 주행 검증 후 0.05~0.1초로 줄일 수 있다.

### Verification

fallback 적용 후 2026-04-12 실측:

```text
/scan_cloud: 약 14.6~14.8Hz
/rtabmap/mapData: 약 1Hz
/rtabmap/cloud_map: frame_id=map 수신
/rtabmap/map: frame_id=map 수신
```

---

## Issue 7: `/rtabmap/loop_closure_id` 토픽이 없음

### Symptom

다음 토픽이 존재하지 않음.

```bash
/rtabmap/loop_closure_id
```

### Root Cause

RTAB-Map ROS2 Humble에서는 loop closure id가 별도 토픽이 아니라 `/rtabmap/info` 메시지 필드다.

```text
/rtabmap/info.loop_closure_id
```

### Fix

```bash
ros2 topic echo /rtabmap/info --field loop_closure_id
```

### Verification

정지 상태 실기체 검증에서는:

```text
loop_closure_id = 0
```

주행 중 루프 클로저가 발생하면 0이 아닌 값이 나오는지 확인한다.

---

## 현재 남은 검증

정지 상태 실기체 검증은 완료됐다. 남은 것은 실내 주행 품질 검증이다.

- RViz2에서 `/rtabmap/cloud_map` 누적 확인
- `/rtabmap/map` 2D occupancy grid 확인
- `/rtabmap/info.loop_closure_id` 0이 아닌 값 확인
- 필요 시 `approx_sync_max_interval`, `tf_latest_fallback_tolerance_sec`, ICP 파라미터 튜닝

## Tuning Note 3: Mapping과 localization 파라미터 분리

### Observation

- Mapping에서는 proximity/neighbor ICP correction이 false link를 만들 수 있어 꺼두는 편이 나았다.
- 하지만 localization에서 같은 설정을 쓰면 현재 scan과 저장 map을 적극적으로 매칭하지 못한다.
- `/rtabmap/localization_pose`는 발행되지만 실제 위치와 다르고, `/rtabmap/info.proximity_detection_id=0`, `loop_closure_id=0`이 지속될 수 있다.

### Fix

공통 YAML은 mapping baseline으로 두고, `localization.launch.py`에서만 localization 전용 override를 적용한다.

```python
'RGBD/ProximityBySpace': 'true',
'RGBD/ProximityOdomGuess': 'true',
'RGBD/ProximityPathMaxNeighbors': '5',
'RGBD/ProximityGlobalScanMap': 'true',
'RGBD/OptimizeMaxError': ParameterValue(
    LaunchConfiguration('optimize_max_error'),
    value_type=str,
),  # default 30.0
'RGBD/MaxOdomCacheSize': '0',
```

`RGBD/OptimizeMaxError` 기본값 3.0에서 localization 후보가 graph consistency 검증으로 reject되는 경우가 있었다.
`10.0`에서도 error ratio 10.6 수준 후보가 reject되었고, `20.0`에서도 yaw graph error ratio 21~24 수준 후보가 reject되어, initial pose 없이 global LiDAR relocalization을 실험하기 위해 `30.0`으로 완화했다.
이 값은 틀린 후보를 accept할 위험이 있으므로 RViz에서 `/scan_cloud`가 `/rtabmap/map` 위에 맞는지 반드시 확인해야 한다.
`RGBD/MaxOdomCacheSize=0`은 localization 후보가 odom cache/graph consistency 경로에서 계속 reject되는 상황을 줄이기 위한 실험값이다.
RTAB-Map 내부 파라미터(`RGBD/...`, `Icp/...` 등)는 문자열 타입으로 선언되므로 launch override도 `ParameterValue(..., value_type=str)`로 넘겨야 한다.

`30.0`에서도 error ratio 30~31 수준으로 reject되면 threshold를 계속 올리기보다 diagnostic으로 validation을 꺼서 후보 자체가 맞는지 확인한다.

```bash
ros2 launch go2_rtabmap_launch localization.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db \
  optimize_max_error:=0 \
  rviz:=true \
  rtabmap_viz:=true
```

`optimize_max_error:=0`은 최종 설정 후보가 아니라 진단용이다. 이 상태에서 RViz 위치가 맞으면 graph validation/variance 문제이고, 위치가 틀리면 global LiDAR 후보 자체가 틀린 것이다.

### Verification

```bash
ros2 launch go2_rtabmap_launch localization.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db \
  rviz:=true \
  rtabmap_viz:=true

ros2 topic echo /rtabmap/info --field proximity_detection_id
ros2 topic echo /rtabmap/info --field loop_closure_id
```

성공 기준은 pose 발행만이 아니다. RViz에서 `/scan_cloud`가 `/rtabmap/map` 위에 실제 위치와 맞게 겹쳐야 한다.

## Issue 8: `rtabmap_viz`는 괜찮은데 RViz `/rtabmap/cloud_map`이 너무 성김

### Symptom

- `rtabmap_viz`의 3D Map은 비교적 괜찮게 보인다.
- RViz에서 `/rtabmap/cloud_map` 또는 `/rtabmap/map`은 포인트가 적거나 지도처럼 보이지 않는다.

### Root Cause

`rtabmap_viz`와 RViz는 같은 데이터를 같은 방식으로 보여주는 도구가 아니다.

```text
rtabmap_viz:
  RTAB-Map 내부 MapData/local scans/graph를 RTAB-Map 방식으로 시각화

RViz /rtabmap/cloud_map:
  RTAB-Map ROS wrapper가 외부로 publish한 PointCloud2
  map publishing/filtering/voxelization 파라미터 영향을 받음
```

실측 중 `/scan_cloud`는 약 10,657 points인데 `/rtabmap/cloud_map`은 약 6,917 points로 더 성긴 출력이 확인됐다.

### Fix

RViz 확인용 출력 밀도를 높이기 위해 map publishing 설정을 조정한다.

```yaml
map_always_update: true
map_filter_radius: 0.0
map_filter_angle: 0.0
cloud_output_voxelized: false
```

### Verification

재시작 후 비교한다.

```bash
ros2 topic echo /scan_cloud --once --field width
ros2 topic echo /rtabmap/cloud_map --once --field width
ros2 topic echo /rtabmap/cloud_map --once --field header
```

RViz 목적별 권장:

```text
브릿지/odom 확인:
  Fixed Frame = odom
  Topic = /scan_cloud
  Decay Time = 20~60초

RTAB-Map export map 확인:
  Fixed Frame = map
  Topic = /rtabmap/cloud_map
  Decay Time = 0초

RTAB-Map 내부 graph/map 확인:
  rtabmap_viz 사용
```

## Tuning Note 1: `/scan_cloud`는 안정적인데 RTAB-Map map이 중복되는 경우

### Observation

- RViz2에서 `/scan_cloud` decay를 늘리면 벽이 거의 같은 위치에 유지됨.
- 하지만 `/rtabmap/cloud_map` 또는 `/rtabmap/map`에서는 같은 벽이 여러 위치에 중복되어 보임.
- `/rtabmap/info.loop_closure_id`, `/rtabmap/info.proximity_detection_id`가 계속 0.

### Interpretation

브릿지/Go2 odom/cloud 변환은 대체로 정상이고, RTAB-Map의 LiDAR-only graph correction 또는 proximity ICP가 충분히 작동하지 않는 상태로 본다.

### First Tuning Set Tried

처음에는 proximity/ICP 보정이 약해서 문제라고 보고 다음 튜닝을 적용했다.

```yaml
odom_sensor_sync: true
"Reg/Force3DoF": "true"
"RGBD/OptimizeFromGraphEnd": "true"
"RGBD/ProximityOdomGuess": "true"
"RGBD/ProximityPathMaxNeighbors": "5"
```

하지만 `/scan_cloud` 단순 누적은 안정적인데 RTAB-Map map만 중복되는 관찰이 반복되면,
proximity/neighbor ICP가 false link를 추가해 graph를 망가뜨리는 쪽을 먼저 의심한다.

### Odom-only Baseline

Go2 odom이 안정적인지 확인하기 위해 RTAB-Map의 scan 기반 graph correction을 끄고 비교한다.

```yaml
odom_sensor_sync: true
"Reg/Force3DoF": "true"
"RGBD/NeighborLinkRefining": "false"
"RGBD/OptimizeFromGraphEnd": "false"
"RGBD/ProximityBySpace": "false"
"RGBD/ProximityOdomGuess": "false"
"RGBD/ProximityPathMaxNeighbors": "0"
```

### Verification

반드시 새 DB로 비교한다.

```bash
ros2 launch go2_rtabmap_launch slam.launch.py \
  database_path:=/tmp/go2_odom_only_rtabmap_test/rtabmap.db \
  reset_db:=true \
  rviz:=true \
  rtabmap_viz:=true
```

확인 항목:

```bash
ros2 topic echo /rtabmap/info --field proximity_detection_id
ros2 topic echo /rtabmap/info --field loop_closure_id
```

RViz2에서 `/scan_cloud`와 `/rtabmap/cloud_map`을 비교해 벽 중복이 줄었는지 확인한다.
odom-only baseline에서 정상이면 문제 원인은 RTAB-Map proximity/neighbor ICP false correction 쪽이다.

## Tuning Note 2: Nav2용 `/rtabmap/map` Grid 튜닝 시도와 보류

### Observation

- `/rtabmap/cloud_map`은 볼만하지만 `/rtabmap/map`이 Nav2에서 쓰기엔 성기거나 난잡하다.
- `ros2 topic echo /rtabmap/map --once --field info`로 map 크기/resolution은 정상인데, RViz Map display에서 벽/free space가 불명확하다.

### Tuning Tried

Nav2용 2D projection을 우선하려고 다음 조합을 시도했다.

```yaml
"Grid/3D": "false"
"Grid/NormalsSegmentation": "false"
"Grid/CellSize": "0.05"
"Grid/RangeMin": "0.3"
"Grid/RangeMax": "8.0"
"Grid/MaxGroundHeight": "0.15"
"Grid/MaxObstacleHeight": "1.5"
"Grid/RayTracing": "true"
```

### Result

이 조합은 현재 Go2/RTAB-Map 구성에서는 `/rtabmap/cloud_map`이 빨간 2D 점처럼 보이게 만들고,
rtabmap_viz/RViz 3D cloud map 확인 품질을 악화시켰다. 따라서 현재 baseline에서는 보류한다.

### Current Decision

3D cloud map 확인과 RTAB-Map 내부 상태를 우선 유지한다.

```yaml
"Grid/Sensor": "0"
"Grid/CellSize": "0.05"
"Grid/RangeMin": "0.3"
"Grid/RangeMax": "10.0"
```

Nav2용 2D occupancy grid는 추후 별도 실험으로 분리한다. 후보는 RTAB-Map grid 파라미터를 다시 조정하거나,
RTAB-Map DB/cloud map에서 별도 2D map 생성 파이프라인을 두는 방식이다.

### Verification

Grid 파라미터를 바꿀 때는 새 DB로 실행하고, `/rtabmap/cloud_map`과 `/rtabmap/map`을 모두 확인한다.

```bash
ros2 launch go2_rtabmap_launch slam.launch.py \
  database_path:=/tmp/go2_nav2_grid_test/rtabmap.db \
  reset_db:=true \
  rviz:=true

ros2 topic echo /rtabmap/map --once --field info
```

RViz 확인:

```text
Fixed Frame = map
Display = PointCloud2 /rtabmap/cloud_map
Display = Map /rtabmap/map
```

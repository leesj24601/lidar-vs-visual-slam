# Go2 Visual SLAM 구현 계획

> **에이전트 실행자:** 이 계획을 실제 구현할 때는 `superpowers:subagent-driven-development` 또는 `superpowers:executing-plans` 방식으로 작업 단위별 검증을 수행한다. 각 단계는 체크박스로 추적한다.

**목표:** 현재 저장소의 LiDAR SLAM 구조를 유지하면서, Go2 RGB-D 카메라 입력을 사용하는 Visual/RGB-D RTAB-Map 매핑 및 로컬라이제이션 경로를 추가한다.

**아키텍처:** 기존 `go2_rtabmap_bridge`와 `go2_rtabmap_launch` 패키지를 그대로 사용하고, Visual 전용 브릿지/launch/config 파일만 옆에 추가한다. Go2의 `/utlidar/robot_odom`을 새 브릿지에서 `/odom`과 `odom -> base_link` TF로 정규화하고, 카메라 RGB/Depth는 `rtabmap_sync/rgbd_sync` 노드가 동기화해 RTAB-Map에 전달한다. 기존 LiDAR SLAM 파일의 동작은 유지한다.

**기술 스택:** ROS 2 Humble, Python launch, `rclpy`, `nav_msgs/Odometry`, `tf2_ros`, `rtabmap_slam`, `rtabmap_sync/rgbd_sync`, `rtabmap_viz`, pytest.

---

## 1. 이번 범위에서 확정한 결정

| 항목 | 결정 |
|------|------|
| 구현 범위 | Visual/RGB-D RTAB-Map 매핑 및 로컬라이제이션 1차 구현 |
| 제외 범위 | Nav2, 대시보드, semantic navigation, raw depth PC alignment, 새 ROS 패키지 분리 |
| 폴더 구조 | 현재 저장소 구조 유지. 기존 패키지 안에 Visual 전용 파일 추가 |
| odom 입력 | Go2가 발행하는 `/utlidar/robot_odom` 사용 |
| odom TF | 새 브릿지에서 `odom -> base_link` TF 발행 |
| `/odom` 발행 | 새 브릿지에서 보정된 stamp로 `/odom` 재발행 |
| 카메라 입력 | 기본값은 aligned depth 사용 |
| RGB 토픽 기본값 | `/camera/color/image_raw` |
| Depth 토픽 기본값 | `/camera/aligned_depth_to_color/image_raw` |
| CameraInfo 기본값 | `/camera/color/camera_info` |
| 카메라 동기화 | 커스텀 노드가 아니라 `rtabmap_sync/rgbd_sync`를 launch 파일에서 실행 |
| DB 정책 | 매핑과 로컬라이제이션 launch 분리. 매핑은 `reset_db:=false` 기본값 |
| 설정 방식 | RTAB-Map 파라미터는 YAML로 분리 |

---

## 2. 파일 구조

새로 만들거나 수정할 파일은 아래로 제한한다.

| 파일 | 작업 | 책임 |
|------|------|------|
| `src/go2_rtabmap_bridge/go2_rtabmap_bridge/odom_tf_bridge.py` | 생성 | `/utlidar/robot_odom`을 구독해 stamp 보정, `/odom` 재발행, `odom -> base_link` TF 발행 |
| `src/go2_rtabmap_bridge/setup.py` | 수정 | `odom_tf_bridge` 콘솔 스크립트 등록 |
| `src/go2_rtabmap_bridge/test/test_odom_tf_bridge.py` | 생성 | 브릿지 helper 함수와 TF 생성 로직 테스트 |
| `src/go2_rtabmap_launch/config/rtabmap_visual_real.yaml` | 생성 | Visual/RGB-D RTAB-Map 공통 파라미터 |
| `src/go2_rtabmap_launch/launch/visual_slam.launch.py` | 생성 | Visual 매핑 launch |
| `src/go2_rtabmap_launch/launch/visual_localization.launch.py` | 생성 | Visual 로컬라이제이션 launch |
| `src/go2_rtabmap_launch/package.xml` | 수정 | `rtabmap_sync` 실행 의존성 추가 |
| `src/go2_rtabmap_launch/test/test_visual_launch_defaults.py` | 생성 | launch 기본값과 YAML 핵심값 고정 테스트 |
| `README.ko.md` | 수정 | Visual SLAM 실행 명령과 범위 문서화 |
| `README.md` | 수정 | 영어 README에 Visual SLAM 실행 명령 반영 |
| `STATUS.md` | 수정 | Visual SLAM 트랙 추가 상태 기록 |

기존 LiDAR 핵심 파일은 동작 변경하지 않는다.

- `src/go2_rtabmap_bridge/go2_rtabmap_bridge/bridge_node.py`
- `src/go2_rtabmap_launch/launch/slam.launch.py`
- `src/go2_rtabmap_launch/launch/localization.launch.py`
- `src/go2_rtabmap_launch/config/rtabmap_lidar_indoor.yaml`

---

## 3. 전체 데이터 흐름

```text
Go2 /utlidar/robot_odom
  -> odom_tf_bridge.py
  -> /odom + odom -> base_link TF

RGB image + aligned depth + camera_info
  -> rtabmap_sync/rgbd_sync
  -> /camera/rgbd_image

/odom + /camera/rgbd_image + static base_link -> camera_link TF
  -> rtabmap_slam/rtabmap
  -> /map, /rtabmap/mapData, rtabmap.db
```

핵심은 Visual SLAM도 Go2 자체 odom을 외부 odometry로 사용한다는 점이다. 즉, 이번 계획은 “카메라만으로 odom을 새로 추정하는 Visual Odometry”가 아니라 “Go2 odom + RGB-D RTAB-Map” 구조다.

---

## 4. 작업 계획

### Task 1: `odom_tf_bridge` 테스트 추가

**대상 파일**

- 생성: `src/go2_rtabmap_bridge/test/test_odom_tf_bridge.py`

**구현 내용**

- `apply_time_offset(stamp, offset)` 함수가 원본 메시지 간 시간 차이를 유지하는지 테스트한다.
- `transform_from_odom(msg, corrected_stamp, odom_frame_id, base_frame_id)` 함수가 보정된 stamp와 지정 프레임 이름을 사용해 `TransformStamped`를 만드는지 테스트한다.
- live Go2 없이 실행 가능한 pytest로 작성한다.

**검증 명령**

```bash
python3 -m pytest src/go2_rtabmap_bridge/test/test_odom_tf_bridge.py -q
```

**초기 기대 결과**

- 구현 전에는 `go2_rtabmap_bridge.odom_tf_bridge` import 실패로 테스트가 실패한다.
- Task 2 완료 후에는 테스트가 통과한다.

---

### Task 2: 새 odom/TF 브릿지 구현

**대상 파일**

- 생성: `src/go2_rtabmap_bridge/go2_rtabmap_bridge/odom_tf_bridge.py`
- 수정: `src/go2_rtabmap_bridge/setup.py`
- 테스트: `src/go2_rtabmap_bridge/test/test_odom_tf_bridge.py`

**노드 이름**

- `go2_odom_tf_bridge`

**콘솔 스크립트 이름**

- `odom_tf_bridge`

**파라미터**

| 이름 | 기본값 | 의미 |
|------|--------|------|
| `input_odom_topic` | `/utlidar/robot_odom` | Go2 원본 odom |
| `output_odom_topic` | `/odom` | RTAB-Map에 넣을 odom |
| `odom_frame_id` | `odom` | world/odom 프레임 |
| `base_frame_id` | `base_link` | 로봇 base 프레임 |
| `odom_qos_depth` | `50` | odom 구독 QoS depth |
| `publish_tf` | `true` | `odom -> base_link` TF 발행 여부 |

**동작 규칙**

1. 첫 `/utlidar/robot_odom` 메시지에서 `offset = now() - sensor_stamp`를 한 번 계산한다.
2. 이후 모든 odom 메시지는 `corrected_stamp = original_stamp + offset`으로 보정한다.
3. `/odom` 메시지는 보정된 stamp, `frame_id='odom'`, `child_frame_id='base_link'`로 발행한다.
4. `publish_tf=true`이면 같은 pose로 `odom -> base_link` TF를 발행한다.
5. 기존 LiDAR용 `bridge_node.py`는 수정하지 않는다.

**`setup.py` entry point 추가**

```python
'odom_tf_bridge = go2_rtabmap_bridge.odom_tf_bridge:main',
```

**검증 명령**

```bash
python3 -m pytest src/go2_rtabmap_bridge/test/test_odom_tf_bridge.py -q
```

**기대 결과**

- `test_apply_time_offset_preserves_sensor_delta` 통과
- `test_transform_from_odom_uses_corrected_stamp_and_configured_frames` 통과

---

### Task 3: Visual RTAB-Map YAML 추가

**대상 파일**

- 생성: `src/go2_rtabmap_launch/config/rtabmap_visual_real.yaml`

**핵심 설정**

```yaml
frame_id: 'base_link'
subscribe_rgbd: true
subscribe_rgb: false
subscribe_depth: false
subscribe_scan_cloud: false
approx_sync: true
approx_sync_max_interval: 0.1
odom_sensor_sync: true
qos_image: 1
qos_camera_info: 1
qos_odom: 1
Reg/Strategy: '0'
Reg/Force3DoF: 'true'
Vis/EstimationType: '2'
Vis/MinInliers: '20'
Grid/FromDepth: 'true'
Grid/RangeMin: '0.3'
Grid/RangeMax: '4.0'
Grid/CellSize: '0.05'
RGBD/CreateOccupancyGrid: 'true'
RGBD/OptimizeFromGraphEnd: 'false'
RGBD/NeighborLinkRefining: 'false'
RGBD/ProximityBySpace: 'false'
RGBD/ProximityOdomGuess: 'false'
Rtabmap/DetectionRate: '1.0'
RGBD/LinearUpdate: '0.1'
RGBD/AngularUpdate: '0.1'
```

**설정 의도**

- `Reg/Strategy: 0`: Visual feature 기반 registration 사용
- `Reg/Force3DoF: true`: Go2 실내 평면 주행 기준으로 roll/pitch/z 자유도 억제
- `Grid/FromDepth: true`: RGB-D depth에서 occupancy grid 생성
- `DetectionRate: 1.0`: 첫 매핑은 안정성 우선
- `ProximityBySpace: false`: 1차 매핑에서는 false loop/link 위험을 줄임

**검증 명령**

```bash
python3 -m pytest src/go2_rtabmap_launch/test/test_visual_launch_defaults.py -q
```

Task 4에서 테스트 파일을 만든 뒤 이 YAML 값도 함께 검증한다.

---

### Task 4: Visual 매핑 launch 추가

**대상 파일**

- 생성: `src/go2_rtabmap_launch/launch/visual_slam.launch.py`
- 수정: `src/go2_rtabmap_launch/package.xml`
- 생성: `src/go2_rtabmap_launch/test/test_visual_launch_defaults.py`

**launch 인자 기본값**

| 인자 | 기본값 |
|------|--------|
| `rgb_topic` | `/camera/color/image_raw` |
| `depth_topic` | `/camera/aligned_depth_to_color/image_raw` |
| `camera_info_topic` | `/camera/color/camera_info` |
| `rgbd_topic` | `/camera/rgbd_image` |
| `odom_topic` | `/odom` |
| `frame_id` | `base_link` |
| `camera_frame_id` | `camera_link` |
| `database_path` | `maps/visual/active/rtabmap.db` |
| `reset_db` | `false` |
| `rtabmap_viz` | `true` |

**launch에 포함할 노드**

1. `go2_rtabmap_bridge/odom_tf_bridge`
2. `tf2_ros/static_transform_publisher` for `base_link -> camera_link`
3. `rtabmap_sync/rgbd_sync`
4. `rtabmap_slam/rtabmap`
5. `rtabmap_viz/rtabmap_viz` 조건부 실행

**정적 TF 기본값**

카메라 위치는 실제 장착값을 측정하기 전까지 보수적 초기값으로 둔다.

```text
base_link -> camera_link
x=0.33, y=0.0, z=0.09, roll=0.0, pitch=0.0, yaw=0.0
```

ROS 2 Humble launch에서는 아래 형태의 named argument를 사용한다.

```python
arguments=[
    '--x', '0.33',
    '--y', '0.0',
    '--z', '0.09',
    '--roll', '0.0',
    '--pitch', '0.0',
    '--yaw', '0.0',
    '--frame-id', 'base_link',
    '--child-frame-id', 'camera_link',
]
```

**`rtabmap_sync` 의존성**

`src/go2_rtabmap_launch/package.xml`에 아래 의존성을 추가한다.

```xml
<exec_depend>rtabmap_sync</exec_depend>
```

**테스트에서 고정할 항목**

- `visual_slam.launch.py`에 `rgbd_sync`가 포함되어야 한다.
- `visual_slam.launch.py`에 `odom_tf_bridge`가 포함되어야 한다.
- 기본 `reset_db` 값은 `false`여야 한다.
- 기본 depth topic은 `/camera/aligned_depth_to_color/image_raw`여야 한다.
- Visual YAML에 `Reg/Strategy: '0'`, `Grid/FromDepth: 'true'`, `Reg/Force3DoF: 'true'`가 있어야 한다.
- `package.xml`에 `rtabmap_sync` 의존성이 있어야 한다.

**검증 명령**

```bash
python3 -m pytest src/go2_rtabmap_launch/test/test_visual_launch_defaults.py -q
```

**수동 실행 명령**

```bash
ros2 launch go2_rtabmap_launch visual_slam.launch.py
```

**DB 초기화 실행 명령**

```bash
ros2 launch go2_rtabmap_launch visual_slam.launch.py reset_db:=true
```

---

### Task 5: Visual 로컬라이제이션 launch 추가

**대상 파일**

- 생성: `src/go2_rtabmap_launch/launch/visual_localization.launch.py`
- 수정: `src/go2_rtabmap_launch/test/test_visual_launch_defaults.py`

**launch 인자 기본값**

| 인자 | 기본값 |
|------|--------|
| `database_path` | 빈 문자열 |
| `localization_mode` | `true` |
| `delete_db_on_start` | `false` |
| `Mem/IncrementalMemory` | `false` |
| `Mem/InitWMWithAllNodes` | `true` |
| `Rtabmap/DetectionRate` | `2.0` |
| `RGBD/LinearUpdate` | `0.0` |
| `RGBD/AngularUpdate` | `0.0` |
| `RGBD/ProximityBySpace` | `true` |
| `RGBD/ProximityOdomGuess` | `true` |

**동작 규칙**

1. `database_path`가 비어 있으면 launch 시 명확한 에러를 낸다.
2. 로컬라이제이션에서는 DB를 삭제하지 않는다.
3. mapping launch와 같은 `odom_tf_bridge`, `static_transform_publisher`, `rgbd_sync`를 사용한다.
4. RTAB-Map은 localization mode로 실행한다.

**에러 메시지 예시**

```text
visual_localization.launch.py requires database_path. Example: database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/visual/active/rtabmap.db
```

**수동 실행 명령**

```bash
ros2 launch go2_rtabmap_launch visual_localization.launch.py database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/visual/active/rtabmap.db
```

**테스트에서 고정할 항목**

- `visual_localization.launch.py`는 `database_path` 빈 문자열을 기본값으로 가져야 한다.
- `delete_db_on_start`는 `false`여야 한다.
- localization override에 `Mem/IncrementalMemory=false`가 있어야 한다.
- localization override에 `Mem/InitWMWithAllNodes=true`가 있어야 한다.
- `Rtabmap/DetectionRate`는 mapping보다 높은 `2.0`으로 설정해야 한다.

---

### Task 6: 문서 업데이트

**대상 파일**

- 수정: `README.ko.md`
- 수정: `README.md`
- 수정: `STATUS.md`

**README.ko.md에 추가할 내용**

- Visual SLAM은 1차 범위에서 `Go2 odom + RGB-D RTAB-Map` 구조임을 명시한다.
- aligned depth topic을 기본으로 사용한다고 적는다.
- raw depth와 RGB를 PC에서 직접 align하는 방식은 이번 범위에 포함하지 않는다고 적는다.
- 실행 명령을 아래처럼 추가한다.

```bash
ros2 launch go2_rtabmap_launch visual_slam.launch.py
ros2 launch go2_rtabmap_launch visual_slam.launch.py reset_db:=true
ros2 launch go2_rtabmap_launch visual_localization.launch.py database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/visual/active/rtabmap.db
```

**STATUS.md에 추가할 내용**

- Visual SLAM 트랙 계획이 확정되었다고 기록한다.
- 기존 LiDAR SLAM 파일은 유지한다고 기록한다.
- 구현 후 남은 측정 항목으로 실제 `base_link -> camera_link` extrinsic 보정과 카메라 토픽 stamp 품질 확인을 적는다.

---

### Task 7: 전체 검증

**빌드**

```bash
colcon build --packages-select go2_rtabmap_bridge go2_rtabmap_launch --symlink-install
```

**테스트**

```bash
python3 -m pytest src/go2_rtabmap_bridge/test/test_odom_tf_bridge.py -q
python3 -m pytest src/go2_rtabmap_launch/test/test_visual_launch_defaults.py -q
```

**launch 파일 확인**

```bash
source install/setup.bash
ros2 launch go2_rtabmap_launch visual_slam.launch.py --show-args
ros2 launch go2_rtabmap_launch visual_localization.launch.py --show-args
```

**실기기 실행 전 토픽 확인**

```bash
ros2 topic list
ros2 topic hz /utlidar/robot_odom
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/color/camera_info
```

**실기기 실행 중 TF 확인**

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

**성공 기준**

- `/odom`이 현재 시간대 stamp로 발행된다.
- `odom -> base_link` TF가 연속적으로 발행된다.
- `/camera/rgbd_image`가 발행된다.
- RTAB-Map이 `/rtabmap/mapData`를 발행한다.
- 매핑 실행 후 `maps/visual/active/rtabmap.db`가 생성된다.
- 로컬라이제이션 launch가 기존 DB를 삭제하지 않고 시작된다.

---

## 5. 구현 순서 요약

1. `odom_tf_bridge` 테스트를 먼저 만든다.
2. `odom_tf_bridge.py`를 구현하고 `setup.py`에 콘솔 스크립트를 등록한다.
3. `rtabmap_visual_real.yaml`을 만든다.
4. `visual_slam.launch.py`와 `rtabmap_sync` 의존성을 추가한다.
5. `visual_localization.launch.py`를 추가한다.
6. README와 STATUS를 갱신한다.
7. pytest, colcon build, launch `--show-args`, 실기기 토픽/TF 확인 순서로 검증한다.

---

## 6. 남은 기술 판단 항목

이번 구현을 시작하기 전에 더 결정할 필요는 없다. 구현 후 실기기에서 아래 값은 측정 결과에 따라 조정한다.

| 항목 | 초기값 | 조정 기준 |
|------|--------|-----------|
| `base_link -> camera_link` x/y/z/rpy | `x=0.33, y=0.0, z=0.09, rpy=0/0/0` | 실제 카메라 장착 위치 |
| `approx_sync_max_interval` | `0.1` | RGB/depth/camera_info stamp 차이 |
| `Grid/RangeMax` | `4.0` | 실내 depth 품질과 노이즈 |
| `Vis/MinInliers` | `20` | 특징점 부족 또는 false match 빈도 |
| `DetectionRate` | mapping `1.0`, localization `2.0` | CPU 사용량과 위치추정 안정성 |

---

## 7. 리스크와 대응

| 리스크 | 증상 | 대응 |
|--------|------|------|
| 카메라 stamp가 현재 시간과 크게 다름 | `rgbd_sync` 출력 없음 또는 RTAB-Map sync 실패 | 카메라 restamp 브릿지를 별도 파일로 추가 |
| depth가 RGB에 aligned되지 않음 | occupancy grid가 실제 구조와 어긋남 | 카메라 드라이버에서 aligned depth 발행 설정 확인 |
| 카메라 extrinsic 초기값 부정확 | 맵 벽/바닥 위치가 비뚤어짐 | `base_link -> camera_link` static TF 실측값 반영 |
| 조명 또는 텍스처 부족 | loop closure 부족, graph drift 증가 | Visual feature 파라미터 조정 또는 LiDAR baseline 병행 사용 |
| `/utlidar/robot_odom` 미수신 | `/odom`과 TF 미발행 | Go2 LiDAR server 및 네트워크 상태 확인 |

---

## 8. 실행 선택지

계획 실행 방식은 두 가지다.

1. **Subagent-Driven 실행 권장**
   - 작업 단위별로 독립 실행자를 붙이고, 각 작업 후 리뷰한다.
   - 테스트와 구현을 분리하기 좋다.

2. **Inline 실행**
   - 현재 세션에서 순서대로 직접 구현한다.
   - 작은 수정에는 빠르지만, 작업량이 많아질수록 중간 리뷰 지점이 중요하다.

이번 작업은 파일 수가 여러 개이고 launch/test/doc가 같이 움직이므로 **Subagent-Driven 실행**이 더 적합하다.

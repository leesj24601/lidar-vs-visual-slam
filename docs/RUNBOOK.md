# Go2 LiDAR SLAM Runbook

> 이 문서는 구현 후 반복 실행할 빌드, 매핑, 로컬라이제이션, 검증 절차를 정리한다.
> 설계와 구현 단계는 `SLAM_PLAN.md`, Go2 실측 제약은 `docs/GO2_REFERENCE.md`를 기준으로 한다.
> 문제별 원인/해결 기록은 `docs/TROUBLESHOOTING.md`를 기준으로 한다.

## 전제 조건

- Ubuntu 22.04
- ROS2 Humble
- `rtabmap_ros` apt 패키지 설치
- Go2와 같은 DDS 네트워크에 연결
- 필요 시 `/home/cvr/Desktop/sj/go2_ws/install/setup.bash` source

```bash
source /opt/ros/humble/setup.bash
```

Go2 이동 제어까지 필요한 경우:

```bash
source /home/cvr/Desktop/sj/go2_ws/install/setup.bash
```

## Go2 연결 확인

Go2 bare DDS 토픽은 ROS daemon 캐시에 안 잡힐 수 있으므로 `--no-daemon`을 우선 사용한다.

```bash
ros2 topic list --no-daemon
ros2 topic info /utlidar/robot_odom --verbose --no-daemon
ros2 topic info /utlidar/cloud_deskewed --verbose --no-daemon
```

기대값:

- `/utlidar/robot_odom`: `nav_msgs/msg/Odometry`, 약 150Hz, RELIABLE
- `/utlidar/cloud_deskewed`: `sensor_msgs/msg/PointCloud2`, 약 14.7Hz, RELIABLE publisher 1개(2026-04-12 실측), 브릿지는 BEST_EFFORT 구독

## 빌드

```bash
cd /home/cvr/Desktop/sj/go2_lidar_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

패키지 확인:

```bash
ros2 pkg list | grep go2_rtabmap
ros2 launch go2_rtabmap_launch slam.launch.py --show-args
```

테스트 확인:

```bash
colcon test --event-handlers console_direct+
```

현재 `slam.launch.py` 주요 인자:

| 인자 | 기본값 | 용도 |
|------|--------|------|
| `database_path` | `/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db` | 매핑 DB 저장/로드 경로 |
| `reset_db` | `false` | `true`일 때 시작 전 기존 DB 삭제 |
| `use_sim_time` | `false` | `/clock` 사용 여부 |
| `rviz` | `false` | RViz2 실행 |
| `rtabmap_viz` | `false` | rtabmap_viz 실행 |

## 브릿지 단독 검증

매핑 전에 브릿지 출력이 정상인지 먼저 확인한다.

```bash
ros2 run go2_rtabmap_bridge bridge_node
```

다른 터미널에서:

```bash
source /opt/ros/humble/setup.bash
source /home/cvr/Desktop/sj/go2_lidar_slam/install/setup.bash

ros2 topic hz /odom
ros2 topic hz /scan_cloud
ros2 run tf2_ros tf2_echo odom base_link
```

기대값:

- `/odom`: 약 150Hz
- `/scan_cloud`: 약 14.7Hz
- `tf2_echo odom base_link`: transform 지속 출력

## 매핑 실행

```bash
cd /home/cvr/Desktop/sj/go2_lidar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch go2_rtabmap_launch slam.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db
```

기본 매핑은 기존 DB를 삭제하지 않는다. 새 매핑을 강제로 시작할 때만:

```bash
ros2 launch go2_rtabmap_launch slam.launch.py reset_db:=true
```

`reset_db:=true`는 `database_path`의 기존 DB와 SQLite sidecar 파일(`-wal`, `-shm`, `-journal`)을 삭제한다.
기존 맵을 이어서 확장하려면 `reset_db`를 사용하지 않는다.

GUI는 기본 실행하지 않는다. RViz2가 기본 시각화 도구이고, RTAB-Map 내부 상태가 필요할 때만 `rtabmap_viz`를 보조로 사용한다.

```bash
# TF, odom, scan cloud, cloud map, occupancy grid 확인
ros2 launch go2_rtabmap_launch slam.launch.py rviz:=true

# pose graph, loop closure, RTAB-Map statistics 보조 확인
ros2 launch go2_rtabmap_launch slam.launch.py rtabmap_viz:=true
```

둘 다 필요하면 같은 launch에서 함께 켠다.

```bash
ros2 launch go2_rtabmap_launch slam.launch.py rviz:=true rtabmap_viz:=true
```

검증 명령:

```bash
ros2 topic hz /rtabmap/mapData
ros2 topic hz /rtabmap/cloud_map
ros2 topic hz /rtabmap/map
ros2 topic echo /rtabmap/info --field loop_closure_id
```

`loop_closure_id`는 `/rtabmap/loop_closure_id` 별도 토픽이 아니라
`/rtabmap/info` (`rtabmap_msgs/msg/Info`) 메시지의 필드다.
값이 0이면 해당 update에서 루프 클로저가 없고, 0이 아닌 값이면 루프 클로저가 검출된 것이다.

RViz에서 확인할 항목:

- Fixed Frame: `map` 또는 `odom`
- TF: `odom -> base_link -> utlidar_lidar`
- Odometry: `/odom`
- PointCloud2: `/scan_cloud`
- PointCloud2: `/rtabmap/cloud_map`
- Map: `/rtabmap/map`

`rtabmap_viz`는 기본 검증 도구가 아니다. 루프 클로저, pose graph, RTAB-Map statistics를 더 자세히 볼 때 사용한다.

## 맵 형태

`slam.launch.py` 실행 결과는 하나의 파일만이 아니라 저장 DB와 ROS 토픽 출력으로 나뉜다.

| 형태 | 경로/토픽 | 용도 |
|------|-----------|------|
| RTAB-Map DB | `maps/active/rtabmap.db` | 저장 및 localization 재사용 |
| 2D occupancy grid | `/rtabmap/map` | 2D 지도 확인, 추후 Nav2 연계 후보 |
| 누적 PointCloud2 | `/rtabmap/cloud_map` | RViz2에서 실내 구조 3D 확인 |
| RTAB-Map map data | `/rtabmap/mapData` | RTAB-Map update 동작 확인 |
| Pose graph | `/rtabmap/mapGraph` | graph/loop closure 확인 |
| 상태 정보 | `/rtabmap/info` | loop closure id, statistics 확인 |

## 로컬라이제이션 실행

매핑으로 생성한 DB를 재사용한다.

```bash
cd /home/cvr/Desktop/sj/go2_lidar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch go2_rtabmap_launch localization.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db
```

`database_path` 파일이 없으면 localization은 실행 실패로 처리한다.

시작 위치가 매핑 시작점과 크게 다르면 초기 위치를 지정할 수 있다.

```bash
ros2 launch go2_rtabmap_launch localization.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db \
  initial_pose:="0 0 0 0 0 0"
```

로컬라이제이션도 GUI는 기본 실행하지 않는다. 필요할 때만:

```bash
ros2 launch go2_rtabmap_launch localization.launch.py rviz:=true
ros2 launch go2_rtabmap_launch localization.launch.py rtabmap_viz:=true
```

검증 명령:

```bash
ros2 topic hz /rtabmap/localization_pose
ros2 topic echo /rtabmap/localization_pose --once
ros2 node info /rtabmap/rtabmap
```

기대값:

- `/rtabmap/rtabmap`이 `/odom`, `/scan_cloud`를 구독
- `/rtabmap/localization_pose` publisher 존재 및 pose 발행
- 로그에 `Localization mode (Mem/IncrementalMemory=false)` 출력

## 기본 진단 순서

문제가 생기면 아래 순서로 확인한다.

1. Go2 원천 토픽 수신 여부

```bash
ros2 topic hz /utlidar/robot_odom --no-daemon
ros2 topic hz /utlidar/cloud_deskewed --no-daemon
```

2. 브릿지 출력 여부

```bash
ros2 topic hz /odom
ros2 topic hz /scan_cloud
ros2 topic echo /scan_cloud --once
```

3. TF 연결 여부

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link utlidar_lidar
```

4. rtabmap 입력 동기화 여부

```bash
ros2 topic hz /rtabmap/mapData
ros2 node info /rtabmap/rtabmap
```

`/rtabmap/mapData`가 발행되지 않으면 `approx_sync`, `frame_id`, QoS, TF lookup 실패 로그를 우선 확인한다.

## 자주 보는 증상

| 증상 | 우선 확인 |
|------|-----------|
| `/scan_cloud`가 안 나옴 | cloud QoS가 BEST_EFFORT인지, `_time_offset` 초기화 전 cloud가 드롭되는지 |
| `/scan_cloud` frame이 이상함 | `/scan_cloud.header.frame_id`가 `base_link`인지 확인 |
| PointCloud2 dtype/assert 오류 | Go2 cloud는 padding 있는 `point_step=32` 구조이므로 raw layout 보존 변환을 사용해야 함 |
| TF extrapolation 오류 | odom stamp 보정과 cloud stamp 보정에 같은 offset을 쓰는지 |
| `/rtabmap/mapData`가 0Hz | `approx_sync=true`, `approx_sync_max_interval` 초기값 0.2, `/odom`/`/scan_cloud` 주파수 |
| 맵이 많이 흔들림 | cloud frame 변환 결과, `Icp/VoxelSize`, `Icp/MaxCorrespondenceDistance` |
| `/scan_cloud`는 정상인데 `/rtabmap/cloud_map` 벽이 중복됨 | 새 DB로 재현 후 odom-only baseline 확인: `NeighborLinkRefining=false`, `ProximityBySpace=false`, `ProximityPathMaxNeighbors=0` |
| `rtabmap_viz`는 괜찮은데 RViz `/rtabmap/cloud_map`이 너무 성김 | RViz는 RTAB-Map이 export한 `cloud_map` 토픽을 보고, rtabmap_viz는 내부 MapData를 시각화한다. `cloud_output_voxelized=false`, `map_always_update=true` 확인 |
| `/rtabmap/map`이 Nav2용으로 너무 성기거나 난잡함 | 3D cloud map 품질을 먼저 유지한다. `Grid/3D=false`, `Grid/NormalsSegmentation=false`, `Grid/RayTracing=true` 조합은 3D cloud export를 2D처럼 망가뜨려 보류 |
| localization pose가 안 나옴 | DB 경로, `Mem/IncrementalMemory=false`, 초기 위치 차이 |

## 산출물

- 기본 매핑 DB: `maps/active/rtabmap.db`
- 추후 보관 구조: `maps/sessions/<timestamp>_<map_name>/rtabmap.db`
- 실행 검증 기준: `SLAM_PLAN.md`의 수용 기준
- Go2 환경 제약 기준: `docs/GO2_REFERENCE.md`

## 최근 실기체 검증 메모

2026-04-12 실기체 검증:

- 원천 `/utlidar/robot_odom`: 약 151Hz
- 원천 `/utlidar/cloud_deskewed`: 약 14.7Hz, `frame_id=odom`, `point_step=32`
- 브릿지 `/odom`: 약 150~152Hz
- 브릿지 `/scan_cloud`: 약 14.6~14.8Hz, `frame_id=base_link`
- `slam.launch.py`: `/rtabmap/mapData` 약 1Hz, `/rtabmap/cloud_map`, `/rtabmap/map`, DB 생성 확인
- `localization.launch.py`: `/rtabmap/localization_pose` 약 1Hz 확인
- 짧은 주행/이동 세션에서 `maps/active/rtabmap.db` 약 2.8MB 저장 및 localization 재검증 확인
- 루프 클로저는 짧은 주행/이동 세션에서 발생하지 않음(`loop_closure_id=0`); 명확한 폐루프 주행 검증 필요

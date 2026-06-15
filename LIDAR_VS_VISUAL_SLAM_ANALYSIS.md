# LiDAR SLAM vs Visual/RGB-D SLAM 코드 차이 분석

분석일: 2026-06-15

## 분석 대상

### 현재 저장소: `/home/cvr/Desktop/sj/go2_lidar_slam`

현재 저장소는 Unitree Go2 내장 LiDAR odometry와 deskewed point cloud를 RTAB-Map에 연결하는 전용 LiDAR SLAM 저장소다.

핵심 파일:

- `src/go2_rtabmap_bridge/go2_rtabmap_bridge/bridge_node.py`
- `src/go2_rtabmap_launch/launch/slam.launch.py`
- `src/go2_rtabmap_launch/launch/localization.launch.py`
- `src/go2_rtabmap_launch/config/rtabmap_lidar_indoor.yaml`
- `dashboard/server.py`
- `docs/GO2_REFERENCE.md`
- `docs/adr/001-slam-tool-selection.md`
- `STATUS.md`

### 비교 대상: `/home/cvr/Desktop/sj/go2_intelligence_framework`

비교 대상은 단순 Visual SLAM 저장소라기보다, Isaac Sim, RTAB-Map RGB-D SLAM, Nav2, GUI, semantic navigation, 실기체 배포를 묶은 전체 자율주행 프레임워크다.

핵심 파일:

- `launch/go2_rtabmap.launch.py`
- `launch/go2_rtabmap_real.launch.py`
- `launch/go2_navigation.launch.py`
- `launch/go2_navigation_real.launch.py`
- `src/go2_gui_controller/go2_gui_controller/odom_restamper.py`
- `src/go2_gui_controller/go2_gui_controller/rgbd_restamper.py`
- `src/go2_gui_controller/go2_gui_controller/rgbd_odom_sync.py`
- `src/go2_gui_controller/go2_gui_controller/web_launch_manager.py`
- `config/go2_nav2_params.yaml`
- `config/go2_nav2_params_real.yaml`
- `docs/plan/05_visual_odom_migration.md`
- `docs/reference/03_rtabmap_real_topics.md`
- `docs/reference/04_realsense_operating_basis.md`

## 핵심 요약

가장 중요한 차이는 센서가 LiDAR냐 카메라냐가 아니다. 현재 저장소는 **LiDAR SLAM을 안정적으로 한 가지 방식으로 재현하기 위한 좁고 깊은 파이프라인**이고, 비교 대상은 **RGB-D 기반 RTAB-Map을 Nav2, GUI, semantic navigation, 시뮬레이션/실기체 운용까지 연결한 넓은 자율성 프레임워크**다.

또 하나의 중요한 점은, 비교 대상의 "Visual SLAM"은 최종 코드 기준으로 순수 Visual Odometry 기반 SLAM이 아니다. 문서상 `rtabmap_odom` 기반 visual odom 경로는 폐기되었고, 최종 구조는 다음에 가깝다.

```text
시뮬레이션: Isaac Sim ground-truth /odom + RGB-D RTAB-Map
실기체: Go2 LiDAR+IMU /utlidar/robot_odom + RealSense RGB-D RTAB-Map
```

즉 비교 대상에서 visual/RGB-D는 주로 **지도 생성, 시각 정합, loop closure, RGB-D export/semantic map**에 쓰이고, 짧은 구간 odometry는 시뮬 ground truth 또는 Go2 LiDAR+IMU odom에 의존한다.

## 큰 구조 차이

| 항목 | 현재 LiDAR 저장소 | 비교 대상 Visual/RGB-D 프레임워크 | 의미 |
|---|---|---|---|
| 저장소 목적 | Go2 LiDAR RTAB-Map baseline | Go2 전체 자율주행 프레임워크 | 현재 저장소는 좁고 재현성 중심, 비교 대상은 통합 시스템 중심 |
| 실행 환경 | 실기체 Go2 중심 | Isaac Sim + 실기체 모두 지원 | 비교 대상은 sim-to-real 구조가 코드 전반에 반영됨 |
| SLAM 패키지 구성 | `go2_rtabmap_bridge`, `go2_rtabmap_launch`로 분리 | root `launch/`와 `go2_gui_controller` 보조 노드에 혼재 | 현재 저장소가 SLAM 모듈 경계가 더 명확함 |
| RTAB-Map 입력 | `/odom` + `/scan_cloud` | RGB-D 또는 RGBDImage + TF 기반 odom | RTAB-Map 입력 계약 자체가 다름 |
| Nav2 연동 | 핵심 범위 밖 | 정식 launch/config 포함 | 비교 대상은 SLAM 출력이 바로 자율주행 입력임 |
| GUI | SLAM 운용 대시보드 | 미션 컨트롤 GUI/Web GUI | GUI의 역할과 범위가 훨씬 큼 |
| semantic layer | 없음 | YOLO + RTAB-Map export 기반 semantic object navigation | RGB-D 데이터가 semantic 확장에 직접 사용됨 |

## RTAB-Map 입력 계약 차이

### 현재 LiDAR 저장소

현재 저장소의 RTAB-Map 입력은 매우 명확하다.

```text
/utlidar/robot_odom
  -> go2_rtabmap_bridge
  -> /odom
  -> odom -> base_link TF

/utlidar/cloud_deskewed
  -> go2_rtabmap_bridge
  -> /scan_cloud

/odom + /scan_cloud
  -> rtabmap_slam/rtabmap
```

`rtabmap_lidar_indoor.yaml`은 다음처럼 LiDAR-only 입력을 강하게 고정한다.

- `subscribe_scan_cloud: true`
- `subscribe_depth: false`
- `subscribe_rgb: false`
- `subscribe_rgbd: false`
- `frame_id: base_link`
- `Reg/Strategy: 1`
- `Icp/PointToPlane: true`

즉 RTAB-Map은 RGB 이미지나 depth image를 보지 않고, `base_link` 기준 point cloud와 odometry만 본다.

### 비교 대상 Visual/RGB-D 프레임워크

시뮬레이션 런치(`launch/go2_rtabmap.launch.py`)는 RGB/depth/camera_info를 직접 RTAB-Map 입력으로 연결한다.

```text
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/camera_info
/odom
/imu/data
  -> rtabmap_slam/rtabmap
```

실기체 런치(`launch/go2_rtabmap_real.launch.py`)는 `rtabmap_sync/rgbd_sync`를 먼저 실행해서 RGB, aligned depth, camera info를 `/camera/rgbd_image`로 묶고, RTAB-Map은 이 RGBDImage를 구독한다.

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
  -> rgbd_sync
  -> /camera/rgbd_image
  -> rtabmap_slam/rtabmap
```

실기체 코드에서는 RTAB-Map이 odom topic을 직접 remap해서 받기보다, `odom_frame_id: odom`과 TF tree를 통해 외부 odometry를 사용한다. 문서 `docs/reference/03_rtabmap_real_topics.md`도 이 점을 명확히 정리한다.

## Odometry 전략 차이

### 현재 LiDAR 저장소

현재 저장소는 `/utlidar/robot_odom`을 SLAM의 핵심 odometry로 직접 사용한다. 브리지 노드는 이 odom을 다음처럼 가공한다.

- Go2의 과거 시간축 stamp를 현재 ROS clock 기준으로 offset 보정
- `/odom`으로 재발행
- `odom -> base_link` TF 발행
- 같은 offset을 `/utlidar/cloud_deskewed`에도 적용
- cloud를 `base_link` 기준 `/scan_cloud`로 변환

여기서 중요한 설계는 **odom과 cloud의 상대 시간 관계를 보존하는 것**이다. `now()`로 단순 치환하지 않고, 첫 odom stamp에서 offset을 계산한 뒤 같은 offset을 계속 적용한다.

### 비교 대상 Visual/RGB-D 프레임워크

비교 대상은 원래 visual odometry를 검토했지만 폐기했다. `docs/plan/05_visual_odom_migration.md` 기준 최종 판단은 다음이다.

- 시뮬레이션 odom: Isaac Sim ground truth `/odom`
- 실기체 odom: Go2 LiDAR+IMU `/utlidar/robot_odom`
- visual odom(`rtabmap_odom`): 텍스처/조명 의존성과 drift 때문에 기본 경로에서 제외

따라서 이쪽의 visual/RGB-D SLAM은 "카메라만으로 odom까지 추정하는 구조"가 아니라, 외부 odom을 받아 RGB-D 정합과 loop closure를 수행하는 구조다.

`odom_restamper.py`는 `/utlidar/robot_odom`을 받아 `/utlidar/robot_odom_restamped`로 재발행하면서 다음 처리를 한다.

- timestamp 보정
- z 위치를 0으로 고정
- roll/pitch 제거, yaw만 유지
- `linear.z`, `angular.x`, `angular.y` 제거
- 선택적으로 `odom -> base_link` TF 발행

다만 현재 실기체 RTAB-Map 기본 경로에서는 RTAB-Map 본체가 `/utlidar/robot_odom_restamped`를 직접 구독하지 않고, upstream의 `odom -> base_link` TF를 쓰는 것으로 정리되어 있다. restamped odom은 주로 Nav2 소비 토픽으로 보는 것이 맞다.

## 시간 동기화 방식 차이

현재 LiDAR 저장소는 시간 보정을 매우 보수적으로 처리한다.

- 첫 odom에서 `_time_offset`을 계산
- odom과 cloud에 같은 offset 적용
- cloud는 보정된 시간의 TF를 lookup
- exact lookup 실패 시 0.2초 이내 최신 TF fallback
- PointCloud2 raw layout은 보존하고 x/y/z만 변환

비교 대상은 카메라/RGB-D 동기화와 consumer 호환성을 우선한다.

- `rgbd_sync`는 RGB/depth/camera_info를 묶어 `/camera/rgbd_image` 생성
- `rgbd_restamper.py`는 RGB/depth/camera_info stamp를 모두 `now()`로 맞춤
- `rgbd_odom_sync.py`는 RGB-D와 최신 odom을 모두 같은 `now()` stamp로 발행
- `odom_restamper.py`는 offset 보정값을 갖지만 timer publish 단계에서는 최신 odom을 현재 clock stamp로 다시 발행

즉 현재 저장소는 **원천 센서 간 상대 시간을 보존하는 쪽**이고, 비교 대상은 **여러 입력을 RTAB-Map/Nav2가 동기화하기 쉽게 재스탬프하는 쪽**이다.

## TF 구조 차이

### 현재 LiDAR 저장소

```text
map
└── odom              # RTAB-Map이 map -> odom 발행
    └── base_link     # go2_rtabmap_bridge가 odom -> base_link 발행
        └── utlidar_lidar  # static_transform_publisher
```

주요 특징:

- `base_link -> utlidar_lidar` 정적 TF를 launch에서 직접 발행
- `odom -> base_link` 동적 TF는 bridge가 `/utlidar/robot_odom`에서 생성
- `/utlidar/cloud_deskewed`는 원래 `odom` frame이므로 bridge가 `base_link`로 변환

### 비교 대상 Visual/RGB-D 프레임워크

시뮬레이션:

```text
map
└── odom
    └── base_link
        └── camera_link
            └── camera_optical_frame
```

실기체:

```text
map
└── odom
    └── base_link
        └── camera_link
            └── camera_color_optical_frame
```

주요 특징:

- 카메라 장착 위치를 `base_link -> camera_link` 정적 TF로 표현
- optical frame 회전을 별도 정적 TF로 표현
- 시뮬레이션에서는 `robot_state_publisher`와 optional `joint_state_publisher`도 포함
- 실기체에서는 upstream odom publisher가 `odom -> base_link`를 제공하는 것을 기본 전제로 둠

## RTAB-Map 파라미터 차이

현재 LiDAR 저장소는 ICP 중심이다.

```yaml
Reg/Strategy: 1
Reg/Force3DoF: true
Icp/PointToPlane: true
Icp/VoxelSize: 0.1
Icp/MaxCorrespondenceDistance: 0.3
```

그리고 매핑 기본값에서는 false proximity link를 피하려고 공간 proximity를 꺼 둔다.

```yaml
RGBD/NeighborLinkRefining: false
RGBD/ProximityBySpace: false
RGBD/ProximityOdomGuess: false
RGBD/ProximityPathMaxNeighbors: 0
```

반면 비교 대상은 visual registration 중심이다.

```python
"Reg/Strategy": "0"
"Vis/EstimationType": "2"
"Vis/MinInliers": "15" 또는 "20"
"Kp/MaxFeatures": "1000"
"Grid/FromDepth": "true"
```

여기서 `Vis/EstimationType=2`는 RGB 특징점과 depth를 이용해 3D-3D 시각 정합을 수행하는 설정이다. mapping/localization 모드에서는 loop threshold, detection rate, feature 수를 다르게 조정한다.

또한 비교 대상은 RTAB-Map 파라미터를 별도 YAML로 빼기보다 launch 파일 내부 dict에 많이 넣는다. 현재 저장소는 `rtabmap_lidar_indoor.yaml`에 공통 파라미터를 분리하고, launch에서 모드별 override만 얹는다.

## 지도 DB 운영 차이

### 현재 LiDAR 저장소

현재 저장소는 DB 운영을 비교적 엄격하게 다룬다.

- 기본 active DB: `maps/active/rtabmap.db`
- 세션 DB: `maps/sessions/<session>/rtabmap.db`
- 백업 DB: `maps/backups/...`
- `slam.launch.py`는 `reset_db:=true`일 때만 DB 및 SQLite sidecar를 삭제
- `localization.launch.py`는 DB가 없으면 즉시 오류
- dashboard는 이미 존재하는 세션 DB 위에 덮어쓰기를 거부

이 구조는 실기체 검증에서 얻은 "좋은 DB"를 보존하고, localization 재현성을 높이기 위한 운영 방식이다.

### 비교 대상 Visual/RGB-D 프레임워크

비교 대상은 모드와 환경마다 DB 전략이 다르다.

- 시뮬 SLAM 기본 DB: `maps/rtabmap.db`
- 시뮬 localization 기본 DB: `maps/rtabmap_office.db`
- 실기체 기본 DB: `maps/rtabmap_real.db`
- `go2_rtabmap.launch.py`의 SLAM 노드는 `arguments=["-d"]`로 기존 DB를 삭제하고 새로 시작
- README에서는 만족스러운 map을 `rtabmap_ground_truth.db` 또는 office DB로 승격하는 흐름을 설명
- `WebLaunchManager`는 map name을 선택하고 Nav2 실행 시 선택된 DB를 넘김

즉 현재 저장소는 DB lifecycle을 SLAM 저장소 안에서 관리하고, 비교 대상은 전체 runtime manager와 문서 흐름 안에서 관리한다.

## Nav2 연동 차이

현재 LiDAR 저장소는 Nav2를 핵심 범위로 포함하지 않는다. RTAB-Map mapping/localization과 대시보드 운용에 집중한다.

비교 대상은 Nav2가 1급 구성 요소다.

- `launch/go2_navigation.launch.py`
- `launch/go2_navigation_real.launch.py`
- `config/go2_nav2_params.yaml`
- `config/go2_nav2_params_real.yaml`

구조는 다음과 같다.

```text
RTAB-Map localization
  -> /map
  -> map -> odom TF

depthimage_to_laserscan
  -> /scan

Nav2
  -> /cmd_vel
```

AMCL은 쓰지 않는다. RTAB-Map이 `/map`과 `map -> odom`을 직접 제공하기 때문이다.

또한 비교 대상은 Go2 사족보행 특성에 맞춰 Nav2 controller로 MPPI를 선택하고, `motion_model: Omni`를 사용한다. 실기체 설정은 시뮬 대비 속도와 가속도를 보수적으로 낮춘 별도 YAML을 둔다.

중요한 점은 `/scan`의 역할이다. 비교 대상에서 `depthimage_to_laserscan`이 `/scan`을 만들지만, 실기체 RTAB-Map은 `subscribe_scan=false`라서 이 `/scan`으로 맵을 만들지 않는다. `/scan`은 주로 Nav2 costmap과 RViz/디버깅용이다.

## GUI와 런타임 관리 차이

현재 LiDAR 저장소의 `dashboard/`는 SLAM 운용에 집중한다.

주요 기능:

- mapping start/stop
- localization start/stop
- `/rtabmap/initialpose` 발행
- align mode와 lock tracking 모드 전환
- `/rtabmap/info`, `/rtabmap/localization_pose`, `map -> odom` 상태 표시
- dashboard가 띄운 ROS 프로세스 정리

비교 대상의 GUI/Web GUI는 미션 컨트롤에 가깝다.

주요 기능:

- simulation start/stop
- SLAM stack start/stop
- Navigation stack start/stop
- RViz start/stop
- map 선택과 저장
- waypoint goal
- manual control
- STT/text command
- semantic object navigation
- runtime stack 상호 배타 실행 관리

따라서 현재 저장소 dashboard는 "RTAB-Map localization 운용 콘솔"이고, 비교 대상 GUI는 "로봇 운용 콘솔"이다.

## Semantic navigation 차이

현재 LiDAR 저장소에는 semantic object layer가 없다.

비교 대상은 RTAB-Map RGB-D DB를 semantic map 생성의 원천 데이터로 사용한다.

```text
RTAB-Map DB / RGB-D export
  -> YOLO inference
  -> depth projection
  -> map frame object coordinate
  -> semantic_objects.yaml
  -> Web GUI text command
  -> Nav2 goal
```

이 차이는 센서 차이에서 파생되지만, 단순히 카메라가 있다는 수준을 넘어선다. 비교 대상은 RGB-D frame, camera pose, calibration을 DB에서 export해서 객체 좌표를 map frame에 투영한다. 현재 LiDAR-only 저장소의 point cloud map만으로는 같은 semantic pipeline을 바로 재사용하기 어렵다.

## 시뮬레이션 지원 차이

현재 LiDAR 저장소는 실기체 Go2의 `/utlidar/*` 토픽을 전제로 한다. Isaac Sim이나 RL policy, USD scene은 포함하지 않는다.

비교 대상은 시뮬레이션이 핵심 축이다.

- `scripts/go2_sim.py`
- `scripts/my_slam_env.py`
- Isaac Sim / Isaac Lab
- RL policy 기반 보행
- USD 환경 (`assets/*.usd`)
- OmniGraph ROS2 bridge
- 시뮬용 RTAB-Map/Nav2 launch

이 때문에 비교 대상에는 시뮬과 실기체의 차이를 흡수하기 위한 조건 분기, 별도 launch, 별도 config, 환경 변수, conda 실행 로직이 많다.

## 검증과 테스트 차이

현재 LiDAR 저장소는 검증 범위가 좁지만 실기체 SLAM 관련 관측이 구체적이다.

- `/utlidar/robot_odom`: 약 150Hz
- `/utlidar/cloud_deskewed`: 약 14.7Hz
- `/scan_cloud`: `frame_id=base_link`
- `/rtabmap/mapData`, `/rtabmap/cloud_map`, `/rtabmap/map` 발행 확인
- `maps/active/rtabmap.db` 저장 확인
- LiDAR-only localization 한계 기록

코드 테스트는 상대적으로 작다.

- `src/go2_rtabmap_launch/test/test_localization_defaults.py`
- `dashboard/test_server_profiles.py`

비교 대상은 테스트 대상이 훨씬 넓다.

- web launch manager
- semantic object registry
- semantic goal resolver
- text command parser
- state bridge
- semantic place builder/registry
- GUI/runtime 관련 테스트

다만 SLAM 자체의 실시간 성공 여부보다, 통합 프레임워크의 구성 요소들이 기대한 command/parameter/path를 만드는지 확인하는 테스트가 많다.

## 현재 한계와 리스크 차이

### 현재 LiDAR 저장소

문서와 STATUS 기준 핵심 한계는 localization이다.

- LiDAR-only RTAB-Map ICP/proximity는 initial pose 없이 전역 후보를 안정적으로 찾지 못함
- false proximity/local match가 accepted되면 `map -> odom`이 흔들릴 수 있음
- 현재는 kidnapped/global relocalization보다 known-start localization에 가까움
- 후속 방향은 `ALIGN -> LOCK -> TRACKING`과 Scan Context + ICP PoC

### 비교 대상 Visual/RGB-D 프레임워크

핵심 리스크는 RGB-D 입력 품질과 실기체 통합이다.

- visual odom은 텍스처/조명 의존성 때문에 기본 경로에서 제외됨
- RealSense aligned depth는 Go2 내부 및 PC 수신 기준 fps가 낮아짐
- RTAB-Map visual loop closure는 특징점, 조명, motion blur, depth 품질에 민감함
- 실기체 Nav2는 `/cmd_vel` 수신, 속도 제한, costmap 품질, TF 안정성까지 같이 봐야 함

## 코드 조직 관점 차이

현재 저장소는 SLAM 모듈로 보기 쉽다.

```text
go2_rtabmap_bridge
  -> Go2 LiDAR 토픽 정규화

go2_rtabmap_launch
  -> mapping/localization launch와 RTAB-Map YAML

dashboard
  -> SLAM 운용 보조
```

비교 대상은 역할이 넓어서 한 패키지 안에 여러 기능이 들어 있다.

```text
launch/
  -> SLAM, Navigation, sim/real 런치

src/go2_gui_controller/
  -> GUI, web app, launch manager, odom/rgbd restamper,
     semantic registry, command parser, navigator bridge

scripts/
  -> Isaac Sim, semantic map builder, scene deploy
```

즉 현재 저장소는 유지보수 시 "SLAM 입력/출력"만 추적하면 되고, 비교 대상은 SLAM을 건드려도 Nav2, GUI, semantic, simulation 쪽 영향까지 봐야 한다.

## 결론

현재 `go2_lidar_slam`은 LiDAR SLAM baseline을 실기체에서 재현하고, RTAB-Map LiDAR localization의 한계를 정밀하게 다루기 위한 저장소다. 핵심 가치는 Go2의 비표준 timestamp, TF 누락, PointCloud2 frame/layout 문제를 한 브리지에서 해결하고, RTAB-Map LiDAR 입력을 단순하게 유지하는 데 있다.

반면 `go2_intelligence_framework`의 visual/RGB-D SLAM 코드는 전체 자율주행 시스템의 한 계층이다. RGB-D RTAB-Map은 지도, localization, loop closure, semantic export를 제공하고, 그 결과가 Nav2와 GUI, semantic object navigation으로 이어진다. 하지만 최종 odometry 전략은 visual odom이 아니라 시뮬 ground truth 또는 Go2 LiDAR+IMU odom이다.

따라서 두 코드의 본질적 차이는 다음처럼 정리할 수 있다.

```text
현재 저장소:
  Go2 LiDAR 원천 토픽을 RTAB-Map LiDAR 입력으로 정확히 정규화하는 전용 SLAM 파이프라인

비교 대상:
  외부 odom + RGB-D RTAB-Map을 Nav2/GUI/semantic/sim-real 운용으로 확장한 통합 자율성 프레임워크
```

Visual과 LiDAR의 센서 차이 외에 가장 큰 차이는 **범위, 시간 동기화 철학, odom 사용 방식, DB 운영 방식, Nav2/semantic/GUI와의 결합도**다.

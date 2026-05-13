# Go2 RTAB-Map LiDAR SLAM 구현 계획

## 목표

Unitree Go2 내장 센서(odom + LiDAR)를 활용한 실내 3D LiDAR SLAM 구현

- **매핑 모드**: 실내 3D 지도 생성 및 저장 (`maps/active/rtabmap.db`)
- **로컬라이제이션 모드**: 기존 지도 로드 후 재위치추정

## 관련 문서

- `STATUS.md`: 현재 진행 상태, 다음 작업, 최근 검증 결과
- `docs/GO2_REFERENCE.md`: Go2 ROS2 토픽, TF, QoS, 타임스탬프 실측 레퍼런스
- `docs/RUNBOOK.md`: 빌드, 실행, 검증, 트러블슈팅 절차
- `docs/TROUBLESHOOTING.md`: 실제 통합 중 발생한 문제의 증상, 원인, 해결, 검증 기록
- `docs/adr/001-slam-tool-selection.md`: SLAM 도구 및 브릿지 아키텍처 선택 기록

---

## 환경 정보

| 항목 | 값 |
|------|-----|
| ROS 버전 | ROS2 Humble |
| rtabmap | 0.22.1 (apt) |
| odom 토픽 | `/utlidar/robot_odom` (nav_msgs/Odometry, 150Hz) |
| 포인트클라우드 | `/utlidar/cloud_deskewed` (PointCloud2, 14.7Hz, frame_id: odom, 모션보정) |
| 타임스탬프 오프셋 | 현재 시각보다 약 461일 과거 (unitree_lidar_server 내부 시간축, 원천 수정 불가) |
| base_link→utlidar_lidar TF | x=0.28945, z=-0.046825, pitch=2.8782rad (공식 URDF) |
| IMU | 제외 (odom이 IMU+발 센서 융합 결과) |
| 타겟 환경 | 실내 |

### 타임스탬프 특이사항

- `/utlidar/*` 토픽 전체가 `unitree_lidar_server` (192.168.123.161) 발행
- 모든 `header.stamp`가 현재 시각 대비 약 461일 과거 시간축 사용
- 원천 수정 불가 → 브릿지 노드에서 로컬 보정 처리

### QoS 프로파일 (실측)

| 토픽 | Reliability | 비고 |
|------|------------|------|
| `/utlidar/robot_odom` | `RELIABLE` | 단일 발행자 |
| `/utlidar/cloud_deskewed` | `RELIABLE` + `BEST_EFFORT` | 발행자 2개 |

브릿지 노드 구독 시:
- `robot_odom`: 기본 QoS (RELIABLE)
- `cloud_deskewed`: `BEST_EFFORT`로 설정 → 두 발행자 모두 수신 가능

### 기존 설치 패키지 (go2_ws)

- `go2_driver`: `/cmd_vel` → Unitree SDK(`/api/sport/request`) 변환 담당
  - TF/odom 발행 기능은 버그 있어 사용하지 않음 (odom 1회 발행, stamp `now()` 불일치)

---

## 아키텍처

```
[Go2 내장 발행]
/utlidar/robot_odom (150Hz, odom→base_link 위치 포함)
/utlidar/cloud_deskewed (14.7Hz, frame_id: odom)
        ↓
[go2_rtabmap_bridge 노드 — Python]
        ↓
/odom (보정된 stamp) + odom→base_link TF (150Hz)
/scan_cloud (base_link 프레임, 보정된 stamp)
        ↓
[rtabmap_ros — LiDAR SLAM 모드]
        ↓
/rtabmap/mapData, /map, maps/active/rtabmap.db
```

---

## 구현 상세

### go2_rtabmap_bridge 노드

#### 역할 1: odom 처리

```
/utlidar/robot_odom 구독
  → 첫 메시지에서 오프셋 1회 계산: offset = now() - sensor_stamp
  → 이후 모든 메시지: corrected_stamp = original_stamp + offset
  → odom→base_link TransformStamped 생성
  → odom→base_link TF 발행 (보정된 stamp, 150Hz)
  → 동일 TransformStamped를 내부 tf_buffer에도 즉시 저장
  → /odom 리퍼블리시 (rtabmap 입력)
```

#### 역할 2: 포인트클라우드 처리

```
/utlidar/cloud_deskewed 구독
  → stamp 보정 (동일 offset 적용)
  → 보정된 stamp로 TF 룩업
  → cloud.header.frame_id='odom' 데이터를 'base_link' 프레임으로 변환
  → /scan_cloud 발행 (rtabmap 입력)
```

구현 시 `tf2` lookup 인자 순서는 반드시 `target_frame, source_frame, time`이다.
`/utlidar/cloud_deskewed`는 `frame_id='odom'`이므로 `/scan_cloud`를 `base_link`로 발행하려면:

```python
transform = tf_buffer.lookup_transform(
    'base_link',              # target_frame
    cloud_msg.header.frame_id, # source_frame = 'odom'
    corrected_time,
    timeout=Duration(seconds=0.0),
)
cloud_base = transform_cloud_preserve_layout(cloud_msg, transform)
```

브릿지가 발행하는 TF 트리는 `odom -> base_link`가 맞지만, cloud 변환 호출은
`lookup_transform('base_link', 'odom', corrected_time)` 방향이어야 한다.
실제 Go2 `/utlidar/cloud_deskewed`는 `x/y/z/intensity` 필드 사이에 padding이 있는
`point_step=32` 구조이므로 `tf2_sensor_msgs.do_transform_cloud()`를 직접 쓰지 않는다.
브릿지는 raw `PointCloud2.data` layout을 유지하고 `x/y/z` float32 값만 변환한다.

#### 처리 순서 (중요)

1. odom 콜백 → 보정 → TF 발행 (항상 먼저)
2. cloud 콜백 → 보정 → **같은 방식으로 보정된 stamp**로 TF 룩업 → 변환 → 발행
3. odom 콜백에서 만든 TransformStamped는 `/tf` 발행과 동시에 내부 `tf_buffer`에도 저장
   → `self._tf_broadcaster.sendTransform(transform)`
   → `self._tf_buffer.set_transform(transform, 'go2_rtabmap_bridge')`
4. cloud 콜백은 내부 `tf_buffer`에서 `lookup_transform('base_link', 'odom', corrected_time)` 수행
5. odom(150Hz)이 cloud(14.7Hz)보다 통계적으로 먼저 TF를 채우지만 보장 아님
   → exact lookup 실패 시 최신 TF가 `tf_latest_fallback_tolerance_sec`(기본 0.2초) 안이면 최신 TF로 변환
   → lookup 실패(ExtrapolationException) 시 해당 cloud 프레임 드롭 후 계속

`set_transform()`을 함께 호출하는 이유:
`SingleThreadedExecutor`에서는 cloud 콜백이 `lookup_transform(timeout=...)`으로 기다리는 동안
새 odom 콜백이 실행되지 못할 수 있다. 브릿지 내부 buffer를 odom 콜백에서 즉시 채워두면
cloud 변환이 `/tf` 재수신 타이밍에 의존하지 않는다.
따라서 cloud lookup timeout 기본값은 0초로 두고, 작은 시간차는 최신 TF fallback으로 처리한다.

#### Executor 설정

`SingleThreadedExecutor` 사용 (기본값) — 콜백 직렬 실행으로 `_time_offset` 레이스 컨디션 원천 차단.
`MultiThreadedExecutor` 전환 시 `_time_offset` 쓰기에 `threading.Lock` 보호 필수.

#### 타임스탬프 보정 전략

- `now()` 단순 대체 금지 (odom-cloud 간 상대 시간 파괴됨)
- **오프셋은 odom 콜백에서만 1회 계산**: `offset = now() - sensor_stamp` (최초 odom 메시지)
- cloud 콜백은 `_time_offset`이 설정될 때까지 메시지 드롭
- 이후 모든 메시지: `corrected_stamp = original_stamp + offset`
- odom과 cloud에 동일한 오프셋을 적용하여 상대 시간 관계 보존
- 장시간 매핑(1시간+) 시 clock skew 누적 가능성 있으나, 통상 매핑 세션 내 무시 가능 수준

#### 정적 TF

launch 파일에서 `static_transform_publisher`로 발행:
```
base_link → utlidar_lidar
x=0.28945, y=0, z=-0.046825, roll=0, pitch=2.8782, yaw=0  (공식 URDF)
```

---

## rtabmap 파라미터 (`rtabmap_lidar_indoor.yaml`)

```yaml
# 핵심 모드 설정
Reg/Strategy: '1'              # ICP 사용 (기본값 '0'=visual → 반드시 설정)
subscribe_scan_cloud: true
subscribe_depth: false
subscribe_rgb: false

# 외부 odometry 사용
# slam.launch.py는 rtabmap_slam/rtabmap 노드만 직접 실행한다.
# rtabmap icp_odometry/visual_odometry 노드는 launch하지 않는다.

# [필수] 토픽 동기화 — odom(150Hz) vs scan_cloud(14.7Hz) 주파수 불일치
# approx_sync: false(기본값)이면 ExactTime 정책 적용 → 콜백 영원히 호출 안됨
# 이 계획은 /utlidar/robot_odom 값을 브릿지에서 보정해 /odom 토픽으로 넘기는 구조다.
# RTAB-Map의 odom_frame_id 파라미터는 "odom 토픽의 frame 이름"이 아니라
# "odom 토픽 대신 TF에서 odometry를 읽을 때의 frame 이름"이다.
# 따라서 /odom 토픽을 쓰려면 odom_frame_id를 비워 둬야 한다.
# odom_frame_id='odom'을 설정하면 /odom 토픽 구독이 꺼지고 TF lookup 모드가 된다.
approx_sync: true
approx_sync_max_interval: 0.2  # 초기값. 안정화 후 0.05~0.1까지 축소 가능

# [필수] 프레임 ID — rtabmap 기본값은 'camera_link'이므로 명시 필수
frame_id: 'base_link'
# odom_frame_id: ''              # 의도적으로 생략/빈 값 유지 (/utlidar/robot_odom -> /odom 토픽 사용)
odom_sensor_sync: true           # scan_cloud stamp에 맞춰 odom pose 보정

# QoS — 브릿지 노드 발행(RELIABLE)과 일치
qos_scan: 1                    # 1 = RELIABLE
qos_odom: 1                    # 1 = RELIABLE

# ICP 파라미터
Reg/Force3DoF: 'true'           # 실내 평면 주행 기준 x/y/yaw 중심 정합
Icp/PointToPlane: 'true'
Icp/VoxelSize: '0.1'
Icp/Iterations: '10'
Icp/MaxCorrespondenceDistance: '0.3'
Icp/MaxTranslation: '0.5'

# 실내 맵 설정
Grid/Sensor: '0'               # scan_cloud 기반 occupancy grid
Grid/CellSize: '0.05'          # 5cm 해상도
Grid/RangeMax: '10.0'          # 실내 최대 10m
Grid/RangeMin: '0.3'

# 루프 클로져
Rtabmap/DetectionRate: '1'
RGBD/NeighborLinkRefining: 'false'       # odom-only baseline: false link 방지
RGBD/OptimizeFromGraphEnd: 'false'
RGBD/ProximityBySpace: 'false'
RGBD/ProximityOdomGuess: 'false'
RGBD/ProximityMaxGraphDepth: '0'
RGBD/ProximityPathMaxNeighbors: '0'

# 맵 저장
Rtabmap/StartNewMapOnLoopClosure: 'false'
```

---

## 프로젝트 구조

```
go2_lidar_slam/
├── SLAM_PLAN.md
├── STATUS.md
├── docs/
│   ├── GO2_REFERENCE.md
│   ├── RUNBOOK.md
│   └── adr/
│       └── 001-slam-tool-selection.md
├── maps/                              ← 생성된 맵 저장
│   ├── active/
│   │   └── rtabmap.db
│   └── sessions/                      ← 추후 세션별 맵 보관
└── src/
    ├── go2_rtabmap_bridge/            ← 브릿지 노드 패키지
    │   ├── go2_rtabmap_bridge/
    │   │   ├── __init__.py
    │   │   └── bridge_node.py
    │   ├── package.xml
    │   └── setup.py
    └── go2_rtabmap_launch/            ← 런치/설정 패키지
        ├── launch/
        │   ├── slam.launch.py         ← 매핑 모드
        │   └── localization.launch.py ← 재위치추정 모드
        ├── config/
        │   └── rtabmap_lidar_indoor.yaml
        ├── package.xml
        └── setup.py
```

---

## 런치 파일 구성

### 맵 저장 정책

- 기본 DB 경로: `/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db`
- `slam.launch.py` 기본 동작은 기존 DB를 삭제하지 않는다.
- 새 매핑을 강제로 시작할 때만 `reset_db:=true`를 사용한다.
- `reset_db:=true`일 때만 시작 전 기존 `database_path` 파일을 삭제하거나 RTAB-Map 삭제 옵션을 적용한다.
- `localization.launch.py`는 지정한 `database_path`가 없으면 명확한 오류로 실패한다.
- 실험/운영 맵 보관은 추후 `maps/sessions/<timestamp>_<map_name>/rtabmap.db` 구조로 확장한다.

### slam.launch.py (매핑 모드)

```
1. static_transform_publisher (base_link → utlidar_lidar)
2. go2_rtabmap_bridge 노드
3. rtabmap_ros (Mem/IncrementalMemory=true)
   - namespace: rtabmap
   - database_path: maps/active/rtabmap.db (launch 인자로 변경 가능)
   - reset_db 기본값 false
   - rviz 기본값 false
   - rtabmap_viz 기본값 false
   - /odom, /scan_cloud 입력
   - 입력 remap은 namespace 영향을 피하도록 절대 경로 사용: `odom -> /odom`, `scan_cloud -> /scan_cloud`
   - odom_frame_id는 비워 두어 /odom 토픽을 approximate sync 대상으로 사용
4. RViz2 / rtabmap_viz (선택)
   - `rviz:=true`: TF, `/odom`, `/scan_cloud`, `/rtabmap/cloud_map`, `/rtabmap/map` 확인
   - `rtabmap_viz:=true`: RTAB-Map pose graph, loop closure, statistics 보조 확인
```

### localization.launch.py (재위치추정 모드)

```
1. static_transform_publisher (base_link → utlidar_lidar)
2. go2_rtabmap_bridge 노드
3. rtabmap_ros
   - namespace: rtabmap
   - Mem/IncrementalMemory=false  ← 새 노드 추가 안 함
   - Mem/InitWMWithAllNodes=true  ← 전체 맵 로드
   - database_path: maps/active/rtabmap.db (launch 인자로 변경 가능)
   - initial_pose 기본값 빈 값 (launch 인자로 변경 가능)
   - localization 전용 proximity override 활성화
   - rviz 기본값 false
   - rtabmap_viz 기본값 false
   - DB 파일이 없으면 실행 실패
   - 입력 remap은 namespace 영향을 피하도록 절대 경로 사용: `odom -> /odom`, `scan_cloud -> /scan_cloud`
```

맵 경로는 런치 인자로 지정 가능:
```python
DeclareLaunchArgument('database_path',
    default_value='/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db')
DeclareLaunchArgument('initial_pose',
    default_value='')
DeclareLaunchArgument('reset_db',
    default_value='false')
DeclareLaunchArgument('rviz',
    default_value='false')
DeclareLaunchArgument('rtabmap_viz',
    default_value='false')
```

---

## 수용 기준

| # | 기준 | 검증 방법 |
|---|------|-----------|
| 1 | slam.launch.py 실행 시 오류 없이 rtabmap 구동 | 터미널 오류 없음, `ros2 topic hz /rtabmap/mapData` > 0 확인 |
| 2 | RViz2에서 포인트클라우드 맵 누적 | `/rtabmap/cloud_map` 구독 후 30초 내 포인트 증가 확인 |
| 3 | 루프 클로저 1회 이상 발생 | `ros2 topic echo /rtabmap/info --field loop_closure_id` 실행 중 0이 아닌 값 수신 |
| 4 | localization 모드에서 재위치추정 동작 | 기존 맵 로드 후 `/rtabmap/localization_pose` 토픽 발행 확인 |
| 5 | bridge 노드 정상 동작 | `ros2 topic hz /scan_cloud` ≈ 14.7Hz, `/odom` ≈ 150Hz 확인 |

---

## 구현 단계

### Phase 1: 워크스페이스 스캐폴딩

**목표**: ROS2 빌드 가능한 기본 패키지 구조를 만든다.

- `src/go2_rtabmap_bridge` Python 패키지 생성
- `src/go2_rtabmap_launch` 런치/설정 패키지 생성
- `maps/active/` 및 `maps/sessions/` 디렉터리 생성
- `package.xml`, `setup.py`, 기본 엔트리포인트 정리

**완료 기준**

- `colcon build` 성공
- `ros2 pkg list`에서 두 패키지 확인

### Phase 2: Go2 RTAB-Map 브릿지 구현

**목표**: Go2 원천 토픽을 rtabmap이 사용할 수 있는 `/odom`, `/scan_cloud`, TF로 정규화한다.

- `/utlidar/robot_odom` 구독
- 최초 odom 기준 timestamp offset 계산
- `/odom` 재발행
- `odom -> base_link` 동적 TF 발행
- 발행한 `odom -> base_link` TransformStamped를 내부 `tf_buffer`에도 `set_transform()`으로 저장
- `/utlidar/cloud_deskewed` 구독
- 동일 offset으로 cloud timestamp 보정
- 보정된 stamp 기준 TF lookup: `lookup_transform('base_link', 'odom', corrected_time)`
- cloud를 `base_link` 프레임으로 변환 후 `/scan_cloud` 발행
- TF lookup 실패 시 해당 cloud만 드롭하고 노드는 계속 실행

**완료 기준**

- `/odom` 주파수 약 150Hz
- `/scan_cloud` 주파수 약 14.7Hz
- `ros2 run tf2_ros tf2_echo odom base_link` 정상 출력

### Phase 3: RTAB-Map 매핑 구성

**목표**: 외부 odom + LiDAR cloud 입력으로 rtabmap 매핑 모드를 실행한다.

- `config/rtabmap_lidar_indoor.yaml` 작성
- `Reg/Strategy=1`, `subscribe_scan_cloud=true` 설정
- `rtabmap_slam/rtabmap` 노드 직접 실행 (`icp_odometry`, `visual_odometry` 노드는 launch하지 않음)
- `approx_sync=true`, `approx_sync_max_interval=0.2` 설정
- `frame_id=base_link` 명시
- `odom_frame_id`는 생략 또는 빈 값 유지 (`/odom` 토픽 구독 모드)
- `slam.launch.py` 작성
- RTAB-Map 노드는 `namespace='rtabmap'`으로 실행해 출력 토픽을 `/rtabmap/...` 아래로 정리
- RTAB-Map 입력 remap은 절대 경로 사용: `odom -> /odom`, `scan_cloud -> /scan_cloud`
- GUI는 기본 실행하지 않음. 필요 시 `rviz:=true` 또는 `rtabmap_viz:=true`
- `base_link -> utlidar_lidar` 정적 TF 포함
- `database_path` 런치 인자 제공
- `reset_db` 런치 인자 제공, 기본값 `false`
- 기본 DB 경로는 `maps/active/rtabmap.db`

**완료 기준**

- `slam.launch.py` 실행 시 rtabmap 오류 없음
- `/rtabmap/mapData` 발행 확인
- RViz에서 cloud/map 누적 확인

### Phase 4: 로컬라이제이션 구성

**목표**: 저장된 `rtabmap.db`를 로드해 새 맵을 만들지 않고 재위치추정을 수행한다.

- `localization.launch.py` 작성
- `Mem/IncrementalMemory=false` 설정
- `Mem/InitWMWithAllNodes=true` 설정
- 기존 `database_path` 재사용
- `database_path` 파일이 없으면 명확한 오류로 실행 실패
- `initial_pose` 런치 인자 제공, 기본값 빈 값
- localization 전용 override:
  - `RGBD/ProximityBySpace=true`
  - `RGBD/ProximityOdomGuess=true`
  - `RGBD/ProximityPathMaxNeighbors=5`
  - `RGBD/ProximityGlobalScanMap=true`
  - `RGBD/OptimizeMaxError`는 `optimize_max_error` launch 인자로 조정, 기본값 `30.0`
  - `RGBD/MaxOdomCacheSize=0`
- GUI는 기본 실행하지 않음. 필요 시 `rviz:=true` 또는 `rtabmap_viz:=true`
- 초기 위치 차이가 큰 경우 `initial_pose` 적용 가능성 검토

**완료 기준**

- 기존 DB 로드 성공
- `/rtabmap/localization_pose` 발행 확인
- 새 맵 노드가 불필요하게 증가하지 않음

### Phase 5: 실기체 검증 및 튜닝

**목표**: Go2 실제 주행 환경에서 SLAM 품질과 재현성을 검증한다.

- Go2 연결 후 원천 토픽 수신 확인
- TF tree 확인
- `/odom`, `/scan_cloud`, `/rtabmap/mapData` 주파수 확인
- 실내 매핑 주행 테스트
- 루프 클로저 발생 여부 확인
- 저장 DB로 localization 재실행
- ICP 파라미터와 grid range 필요 시 조정

**완료 기준**

- 30초 이상 포인트클라우드 맵 누적
- 루프 클로저 1회 이상 확인
- `maps/active/rtabmap.db` 재사용 localization 성공
- 실행/검증 절차가 `docs/RUNBOOK.md`에 반영됨

---

## 확인 필요 사항

- [x] Python PointCloud2 변환 처리율이 실제 `/utlidar/cloud_deskewed` 14.7Hz를 유지하는지
- [x] Go2 bare DDS QoS 프로파일 확인 (`/utlidar/cloud_deskewed` RELIABLE publisher, 브릿지 BEST_EFFORT 구독 호환)
- [x] `approx_sync_max_interval: 0.2` 적용 후 rtabmap 콜백 호출 여부 (`ros2 topic hz /rtabmap/mapData`로 확인)
- [ ] `/rtabmap/mapData`가 안정적으로 발행되면 `approx_sync_max_interval`을 0.05~0.1 범위로 축소 가능 여부 확인
- [x] localization 모드에서 기존 DB 로드 및 `/rtabmap/localization_pose` 발행 확인
- [ ] odom-only baseline(`NeighborLinkRefining=false`, `ProximityBySpace=false`) 후 맵 품질 재검증
- [ ] Nav2용 2D map 튜닝은 3D cloud map 품질과 분리해 별도 실험으로 재검토
- [ ] localization 모드 시작 위치가 매핑 시작점과 크게 다를 경우 initial_pose 파라미터 설정 필요 여부

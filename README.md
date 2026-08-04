<div align="center">
  <h1>Go2 LiDAR & Visual SLAM</h1>
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=flat&logo=ros&logoColor=white" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?style=flat&logo=ubuntu&logoColor=white" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/RTAB--Map-SLAM-7C3AED?style=flat" alt="RTAB-Map">
  <img src="https://img.shields.io/badge/Unitree-Go2-111827?style=flat" alt="Unitree Go2">
  <p>Unitree Go2에서 LiDAR, Go2 odom + RGB-D, 순수 RGB-D를 비교·운용하는 ROS 2 SLAM 프로젝트</p>
  <p><em>ROS 2 SLAM workspace for LiDAR, Go2-odometry RGB-D, and pure RGB-D mapping on the Unitree Go2.</em></p>
</div>

## 프로젝트 개요

이 저장소는 Unitree Go2와 RealSense를 RTAB-Map에 연결해 세 가지 SLAM 경로를
실기에서 비교하고 운용한다. Go2 전용 DDS message를 단순히 launch하는 데 그치지 않고,
timestamp epoch 보정, QoS, TF 소유권, padding이 있는 PointCloud2 변환, RGB-D 동기화,
DB 기반 localization과 Visual Nav2까지 포함한다.

## 세 SLAM 모듈

| 모듈 | odometry | RTAB-Map 입력 | 기본 DB | 현재 지원 범위 |
|---|---|---|---|---|
| LiDAR SLAM | Go2 `/utlidar/robot_odom` | `/odom` + `/scan_cloud` | `maps/active/rtabmap.db` | mapping, known-start 중심 localization |
| Go2 odom 기반 Visual SLAM | Go2 `/utlidar/robot_odom` | `/odom` + RGB-D | `maps/visual/active/rtabmap.db` | mapping, localization, Nav2 |
| 순수 Visual SLAM | RTAB-Map RGB-D VO | `/odom/vo` + RGB-D + odom info | `maps/visual_vo/active/rtabmap.db` | mapping, Go2 odom 비교 |

TF의 지역 odometry edge는 모듈별로 한 노드만 소유한다.

```text
LiDAR / Go2 odom Visual: map -> odom -> base_link -> sensor
Pure Visual:             map -> vo_odom -> base_link -> camera
```

세 모듈의 데이터 흐름과 차이는 [시스템 아키텍처](docs/ARCHITECTURE.md)에 자세히
정리돼 있다.

## 문서

프로젝트를 다시 볼 때는 아래 여섯 문서만 먼저 보면 된다.

| 문서 | 내용 |
|---|---|
| [README](README.md) | 프로젝트 요약과 최소 실행 예시 |
| [ARCHITECTURE](docs/ARCHITECTURE.md) | 세 모듈의 데이터 흐름, TF, 패키지 책임과 설계 결정 |
| [OPERATIONS](docs/OPERATIONS.md) | 설치, 빌드, mapping/localization/Nav2, DB와 bag 운용 |
| [GO2_REFERENCE](docs/GO2_REFERENCE.md) | Go2 원천 topic, QoS, TF와 센서 특성 원본 및 구현 보충 |
| [VALIDATION](docs/VALIDATION.md) | 테스트, DB·bag 산출물, 실험 수치와 해석 범위 |
| [TROUBLESHOOTING](docs/TROUBLESHOOTING.md) | 증상별 원인, 조치와 확인 명령 |

`docs/superpowers/`는 당시 구현 계획과 설계 기록이므로 참고용으로 보존한다. 현재 동작은
위 핵심 문서와 실제 코드를 기준으로 판단한다. SLAM 도구 선정 배경은
[ADR 001](docs/adr/001-slam-tool-selection.md)에 남아 있다.

## 요구 환경

- Ubuntu 22.04
- ROS 2 Humble
- `rtabmap_ros`
- 같은 DDS 네트워크의 Unitree Go2
- Go2 odom Visual·순수 Visual용 RealSense aligned RGB-D
- Unitree message와 RobotModel용 `go2_ws`

Go2 raw topic은 ROS daemon cache에 나타나지 않을 수 있다.

```bash
ros2 topic list --no-daemon
ros2 topic info /utlidar/robot_odom --verbose --no-daemon
ros2 topic info /utlidar/cloud_deskewed --verbose --no-daemon
```

## 설치와 빌드

```bash
git clone https://github.com/leesj24601/lidar-vs-visual-slam.git go2_lidar_slam
cd go2_lidar_slam

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Unitree package가 필요한 build와 실행에서는 overlay를 아래 순서로 source한다.

```bash
source /opt/ros/humble/setup.bash
source /home/cvr/Desktop/sj/go2_ws/install/setup.bash
source /home/cvr/Desktop/sj/go2_lidar_slam/install/setup.bash
```

이 프로젝트는 `go2_ws`의 설치된 message와 description을 사용한다. 별도 통합
`go2_driver`/`go2_bringup` launch를 동시에 실행해 sensor 또는 TF publisher를
중복시키지 않는다.

## 빠른 실행

모든 명령은 저장소 루트에서 실행한다. 실제 운용 인자와 검증 명령은
[OPERATIONS](docs/OPERATIONS.md)를 따른다.

### 1. LiDAR SLAM

Mapping:

```bash
ros2 launch go2_rtabmap_launch slam.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db \
  rviz:=true
```

기존 DB localization:

```bash
ros2 launch go2_rtabmap_launch localization.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/active/rtabmap.db \
  rviz:=true
```

LiDAR-only localization은 비슷한 구조의 false ICP match에 취약하므로 가능한 한
`initial_pose`를 주고, pose topic뿐 아니라 현재 scan과 저장 map이 실제 위치에서
겹치는지 확인한다.

### 2. Go2 odom 기반 Visual SLAM

RealSense color, aligned depth와 color CameraInfo가 먼저 발행되고 있어야 한다.

Mapping 전용 상위 모드:

```bash
ros2 launch go2_nav2_bringup visual_mapping_mode.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/visual/active/rtabmap.db
```

기존 DB localization + Nav2 안전 점검:

```bash
ros2 launch go2_nav2_bringup visual_navigation_mode.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/visual/active/rtabmap.db \
  enable_motion:=false \
  rviz:=true \
  show_robot_model:=true \
  lowstate_topic:=/lowstate
```

이 모드는 RTAB-Map이 `/rtabmap/map`과 `map -> odom`을 제공하고, aligned depth를
`/scan`으로 바꿔 Nav2 obstacle source로 사용한다. AMCL과 `nav2_map_server`는 시작하지
않는다. `/lowstate`는 읽기 전용 joint bridge를 거쳐 `/joint_states`가 되고,
`go2_description.urdf`의 live RobotModel을 표시한다.

먼저 `enable_motion:=false`로 localization, TF, costmap, MPPI path와 RobotModel을 모두
검증한다. 주변을 비우고 비상 정지 수단과 운용자를 준비한 실기 시험에서만
`enable_motion:=true`를 명시한다. Sport bridge는 `/cmd_vel`을 제한해 Move API 1008로
보내며, zero·비정상·0.30초 stale command에는 StopMove API 1003을 보낸다.

### 3. 순수 Visual SLAM

```bash
ros2 launch go2_rtabmap_launch vo_visual_slam.launch.py \
  database_path:=/home/cvr/Desktop/sj/go2_lidar_slam/maps/visual_vo/active/rtabmap.db \
  rviz:=true
```

이 경로는 Go2 odom을 사용하지 않고 `rgbd_odometry`가 `/odom/vo`와
`vo_odom -> base_link`를 만든다. 현재는 mapping 전용이며 별도 localization·Nav2
launch는 없다.

Go2 odom과 VO를 같은 bag에서 비교하는 명령은
[OPERATIONS의 비교 절](docs/OPERATIONS.md#go2-odom과-visual-odometry-비교)을 사용한다. 두 odometry
사이 차이는 ground truth가 아니므로 절대 정확도로 해석하지 않는다.

## DB와 안전 원칙

- mapping의 `reset_db` 기본값은 `false`다.
- `reset_db:=true`는 선택한 DB와 같은 이름의 SQLite sidecar를 지우므로 새 실험에서만
  사용한다.
- 세 모듈의 DB를 서로 바꿔 쓰지 않는다.
- localization은 존재하는 DB의 절대 경로를 지정한다.
- 비교 실험은 기존 active DB가 아닌 새 경로에서 시작한다.
- 여러 SLAM 모듈을 동시에 실행해 같은 TF child frame을 중복 발행하지 않는다.

## 검증

현재 Python 테스트 기준:

```bash
python3 -m pytest -q \
  src/go2_rtabmap_bridge/test \
  src/go2_rtabmap_launch/test \
  src/go2_nav2_bringup/test \
  src/go2_nav2_control/test
```

2026-08-05 문서 정리 시점에는 76개 테스트가 통과했다. 이는 코드·launch·config 계약을
검증한 결과이며 실기 지도 정확도나 모든 환경의 navigation 성공을 보장하지 않는다.
보존 DB와 VO 비교 수치는 [VALIDATION](docs/VALIDATION.md)에 기록돼 있다.

## 저장소 구조

```text
go2_lidar_slam/
├── README.md
├── dashboard/                     # 정적 UI와 Python backend
├── docs/
│   ├── ARCHITECTURE.md
│   ├── OPERATIONS.md
│   ├── GO2_REFERENCE.md
│   ├── VALIDATION.md
│   ├── TROUBLESHOOTING.md
│   ├── adr/                       # architecture decision records
│   └── superpowers/               # 과거 계획·설계 기록
├── maps/                          # 모듈별 RTAB-Map DB
├── bags/                          # 비교 rosbag
├── results/                       # 분석 JSON·CSV·plot
└── src/
    ├── go2_rtabmap_bridge/        # Go2 timestamp, odom, TF, cloud bridge
    ├── go2_rtabmap_launch/        # 세 RTAB-Map 모듈 launch/config
    ├── go2_nav2_bringup/          # Visual mapping/navigation 상위 모드
    └── go2_nav2_control/          # Sport command와 LowState joint bridge
```

## Dashboard

`dashboard/`에는 mapping/localization 제어용 browser UI와 Python backend가 있다.

```bash
python3 dashboard/server.py --host 127.0.0.1 --port 8080
```

자세한 내용은 [dashboard README](dashboard/README.md)를 참고한다.

## License

ROS package manifest는 MIT license를 선언한다.

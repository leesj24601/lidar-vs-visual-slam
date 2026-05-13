# Unitree Go2 ROS2 개발 레퍼런스

> Go2 관련 ROS2 프로젝트 시작 시 매번 반복 조사 없이 참고할 수 있는 범용 레퍼런스.
> 마지막 실측: 2026-03-31 ~ 2026-04-10

---

## 네트워크 구조

| 호스트 | IP | 역할 |
|--------|-----|------|
| Go2 메인 보드 | `192.168.123.18` | SSH 접속 가능, ROS2/DDS 참여 |
| unitree_lidar_server | `192.168.123.161` | LiDAR/SLAM 통합 런타임, DDS 발행자 |
| PC | `192.168.123.x` | 개발 머신 |

- `.161`은 SSH 접속 불가 (22/tcp closed)
- `.18`에서도 `.161`로의 내부 접근 경로 없음
- ROS2 DDS 도메인은 공유됨 (같은 네트워크면 토픽 구독 가능)

```bash
# Go2 SSH 접속
ssh unitree  # alias: 192.168.123.18, user: unitree
```

---

## 토픽 목록

### unitree_lidar_server (192.168.123.161) 발행 토픽

모두 **bare DDS 앱** 발행 (`_CREATED_BY_BARE_DDS_APP_`), 모두 **과거 시간축** 사용.

| 토픽 | 타입 | 주파수 | frame_id | QoS | 비고 |
|------|------|--------|----------|-----|------|
| `/utlidar/robot_odom` | nav_msgs/Odometry | 150.6 Hz | odom | RELIABLE | child_frame_id: base_link, **SLAM 핵심** |
| `/utlidar/cloud_deskewed` | sensor_msgs/PointCloud2 | 14.7 Hz | odom | RELIABLE | 모션보정, 포인트 약 10,720개, **SLAM 핵심** |
| `/utlidar/cloud` | sensor_msgs/PointCloud2 | ~10 Hz | utlidar_lidar | - | 원본, 포인트 1,413개 |
| `/utlidar/cloud_base` | sensor_msgs/PointCloud2 | ~10 Hz | base_link | - | 필터링됨, 포인트 626개 |
| `/utlidar/robot_pose` | geometry_msgs/PoseStamped | ~10 Hz | odom | - | 위치만 있음, 속도 없음 |
| `/utlidar/imu` | sensor_msgs/Imu | - | utlidar_imu | - | 과거 시간축 |
| `/utlidar/lidar_state` | - | - | - | - | 자체 float64 stamp 필드 |
| `/utlidar/range_info` | - | - | base_link | - | 과거 시간축 |
| `/sportmodestate` | unitree_go/SportModeState | - | - | - | 스포츠 모드 상태 |

### 클라우드 토픽 선택 기준

| 토픽 | 포인트 수 | 모션보정 | 권장 용도 |
|------|-----------|----------|-----------|
| `/utlidar/cloud` | 1,413 | ❌ | 원본 데이터 필요 시 |
| `/utlidar/cloud_base` | 626 | ❌ | 사용 비권장 (필터링으로 포인트 적음) |
| `/utlidar/cloud_deskewed` | 약 10,720 | ✅ | **SLAM 권장** (고밀도) |

### ROS2 토픽 조회 방법

Go2 bare DDS 특성상 daemon 캐시에 안 잡힘 → `--no-daemon` 필수:

```bash
ros2 topic list --no-daemon
ros2 topic echo /utlidar/robot_odom --no-daemon
ros2 topic info /utlidar/robot_odom --verbose --no-daemon
```

---

## 타임스탬프 문제

### 핵심 사실

- **원인**: `unitree_lidar_server`가 자체 시간축(과거 epoch)으로 `header.stamp` 발행
- **크기**: 현재 시각 대비 약 **461일 과거** (2026-04-06 기준)
- **특성**: 멈춘 값이 아님, 정상 속도로 증가함
- **원천 수정**: 불가 (`.161` SSH 접근 불가)
- **영향 토픽**: `/utlidar/*` 전체 (cloud, odom, imu, pose 등 모두 동일 시간축)

### 측정값 (2026-03-31 기준)

| 항목 | 값 |
|------|-----|
| 전체 오프셋 (odom 기준) | 39,314,188.008 초 |
| 전체 오프셋 (cloud 기준) | 39,314,188.074 초 |
| Cloud-Odom 차이 | 0.066 초 (~66ms) |
| 오프셋 일수 | 약 461일 |

> ⚠️ 오프셋은 런타임마다 달라질 수 있음 (측정값은 참고용). 코드에서는 항상 **동적으로 계산**.

### 대응 전략: 브릿지 노드에서 보정

```python
# ✅ 올바른 방법: 첫 메시지에서 오프셋 1회 계산 후 동일 오프셋 유지
# odom과 cloud에 같은 오프셋 적용 → 상대 시간 관계 보존

self._time_offset = None

def _correct_stamp(self, sensor_stamp):
    now = self.get_clock().now()
    sensor_time = rclpy.time.Time.from_msg(sensor_stamp)
    if self._time_offset is None:
        self._time_offset = now - sensor_time
    return (sensor_time + self._time_offset).to_msg()

# ❌ 잘못된 방법: 매번 now()로 대체
# → odom-cloud 간 상대 시간 파괴 → SLAM 품질 저하
```

---

## TF 구조

### 정적 TF (항상 필요)

```bash
# base_link → utlidar_lidar (공식 URDF: go2_description/urdf/go2_description.urdf, radar_joint)
# URDF 프레임명은 "radar"이나 실제 토픽 frame_id는 "utlidar_lidar"로 맞춤
ros2 run tf2_ros static_transform_publisher \
  --x 0.28945 --y 0 --z -0.046825 \
  --roll 0 --pitch 2.8782 --yaw 0 \
  --frame-id base_link --child-frame-id utlidar_lidar
```

### 동적 TF (odom → base_link)

**Go2는 직접 발행하지 않음** → 브릿지 노드에서 구현 필요

```python
# /utlidar/robot_odom (Odometry)에서 추출하여 발행
# robot_odom.child_frame_id = 'base_link' 이미 올바른 기준점
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def publish_tf(self, odom_msg, corrected_stamp):
    t = TransformStamped()
    t.header.stamp = corrected_stamp
    t.header.frame_id = 'odom'
    t.child_frame_id = 'base_link'
    t.transform.translation.x = odom_msg.pose.pose.position.x
    t.transform.translation.y = odom_msg.pose.pose.position.y
    t.transform.translation.z = odom_msg.pose.pose.position.z
    t.transform.rotation = odom_msg.pose.pose.orientation
    self._tf_broadcaster.sendTransform(t)
```

> ⚠️ go2_ws의 `go2_driver`가 `/utlidar/robot_pose`로 odom→base_link TF를 발행하지만,
> stamp에 `now()` 사용 + odom 1회만 발행하는 버그가 있어 **SLAM 용도로 사용 불가**.

---

## QoS 설정 (실측)

| 토픽 | Reliability | Durability | 구독 시 권장 설정 |
|------|------------|------------|------------------|
| `/utlidar/robot_odom` | RELIABLE | VOLATILE | 기본 QoS |
| `/utlidar/cloud_deskewed` | RELIABLE (2026-04-12 실측 publisher 1개) | VOLATILE | BEST_EFFORT |

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# odom 구독 (RELIABLE)
qos_reliable = QoSProfile(depth=10)

# cloud 구독 (BEST_EFFORT — RELIABLE publisher와 호환)
qos_best_effort = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

> 2026-04-12 실측에서는 `/utlidar/cloud_deskewed` publisher가 RELIABLE 1개로 확인됐다.
> BEST_EFFORT subscriber는 RELIABLE publisher와 호환되어 브릿지 출력 `/scan_cloud` 약 14.7Hz를 확인했다.
> 실제 cloud field layout은 `x(0), y(4), z(8), intensity(16)`, `point_step=32`로 padding이 있다.

---

## go2_ws 패키지 (설치 위치: `/home/cvr/Desktop/sj/go2_ws`)

### 패키지 목록

| 패키지 | 용도 |
|--------|------|
| `go2_driver` | cmd_vel → Unitree SDK 변환, TF/odom 발행 (버그 있음) |
| `go2_bringup` | 전체 시스템 런치 파일 |
| `go2_description` | URDF/로봇 모델 |
| `go2_interfaces` | 커스텀 서비스 정의 |
| `go2_rviz` | RViz 설정 |
| `unitree_go` | Unitree 메시지 타입 |
| `unitree_api` | Unitree API 메시지 타입 |

### go2_driver 기능 및 주의사항

**사용 가능한 기능:**
- `/cmd_vel` (geometry_msgs/Twist) → `/api/sport/request` → Go2 이동
- 각종 서비스: body_height, continuous_gait, euler, foot_raise_height, mode, pose, speed_level, switch_gait

**버그 / 사용 불가 기능:**
- odom→base_link TF: `stamp = now()` 사용 → 시간 오프셋 충돌로 TF 룩업 실패
- `/odom` 토픽: 첫 메시지에서 딱 1회만 발행 (`odom_published_` 플래그)

**cmd_vel 연결 방법:**
```bash
# go2_ws 빌드 후
source /home/cvr/Desktop/sj/go2_ws/install/setup.bash
ros2 run go2_driver go2_driver  # 또는 launch 파일 사용
# 이후 /cmd_vel 발행하면 Go2가 움직임
```

---

## 새 프로젝트 시작 체크리스트

```
□ ros2 topic list --no-daemon  # Go2 연결 확인
□ 시간 오프셋 확인 (런타임마다 변동 가능)
  ros2 topic echo /utlidar/robot_odom --no-daemon --once
  → header.stamp.sec 확인, 현재 시각과 차이 계산
□ QoS 확인 (변동 가능성 있음)
  ros2 topic info /utlidar/cloud_deskewed --verbose --no-daemon
□ 브릿지 노드 필요 여부 판단
  - 타임스탬프 보정 필요: 항상 YES
  - odom→base_link TF 필요: YES (Go2 미발행)
  - cloud 좌표 변환 필요: cloud_deskewed 사용 시 YES (frame_id: odom → base_link)
□ go2_ws source 여부 결정 (cmd_vel로 로봇 제어 필요 시)
```

---

## 환경 정보

| 항목 | 값 |
|------|-----|
| OS | Ubuntu 22.04 (Jammy) |
| ROS | ROS2 Humble |
| rtabmap | 0.22.1 (apt, 2026-04-10 업데이트) |
| go2_ws 패키지 | 최신 (2026-04-10 확인) |
| tf2_sensor_msgs Python | 설치됨, Humble에서 정상 동작 확인 |

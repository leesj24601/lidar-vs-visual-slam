# Go2 RealSense D435i RGB-D·IMU 실행 가이드

## 구성 상태

Go2의 기존 RealSense 환경은 유지되고, 신규 환경이 별도 경로에 설치되어
있다.

| 항목 | 기존 환경 | 신규 환경 |
|---|---|---|
| ROS 2 | Foxy | Foxy |
| librealsense | 시스템 package 2.54.2 | 사용자 prefix 2.56.5 |
| realsense_ros | 4.54.1 | 4.56.4 + Foxy 호환 patch |
| workspace | `~/ros2_realsense_ws` | `~/ros2_realsense_456_ws` |
| SDK backend | 기존 설치값 | native V4L2/HID |
| DDS | 기존 CycloneDDS | 기존 CycloneDDS 설정을 그대로 사용 |
| 카메라 펌웨어 | 5.17.0.10 | 변경하지 않음 |

신규 SDK와 wrapper 경로:

```text
/home/unitree/librealsense-2.56.5-src
/home/unitree/librealsense-2.56.5-build
/home/unitree/librealsense-2.56.5-install
/home/unitree/ros2_realsense_456_ws
```

시스템 SDK, 커널, DKMS, udev rule, 펌웨어와 기존 workspace는 변경하지
않는다. 기존 RealSense workspace와 신규 RealSense workspace를 같은
shell에서 차례로 source하지 말고 항상 새 shell을 사용한다. DDS는 별도의
카메라 구성요소가 아니므로 기존 Go2 CycloneDDS 환경을 그대로 유지한다.

## 신규 환경 실행

Go2에 접속한 새 대화형 shell에서 기존 시작 메뉴의 `1`번(Foxy)을 선택한
뒤 다음 한 줄을 실행한다.

```bash
source /home/unitree/ros2_realsense_456_ws/activate.bash
```

Go2의 기존 `.bashrc`가 `1`번 선택 시 다음 DDS 환경을 이미 구성한다.

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
```

`activate.bash`는 이 두 값을 수정하거나 다시 설정하지 않는다. 이미 Foxy가
활성화되어 있으면 그 환경을 보존하고 다음 두 항목만 추가한다.

1. librealsense 2.56.5의 `bin`, CMake prefix, library path
2. `/home/unitree/ros2_realsense_456_ws/install/local_setup.bash`

ROS 환경이 전혀 없는 shell에서는 편의를 위해 `/opt/ros/foxy/setup.bash`도
먼저 source하지만, DDS 구현체는 임의로 고르지 않는다.

환경이 올바른지 확인하려면 다음을 실행한다.

```bash
echo "$RMW_IMPLEMENTATION"
echo "$CYCLONEDDS_URI"
rs-enumerate-devices --version
ros2 pkg xml realsense2_camera | grep -m1 '<version>'
```

예상 DDS는 `rmw_cyclonedds_cpp`, SDK와 wrapper 버전은 각각 `2.56.5.0`,
`4.56.4`다.

SSH 비대화형 명령이나 systemd처럼 `.bashrc` 시작 메뉴를 거치지 않는
실행에서는 CycloneDDS 환경을 먼저 명시적으로 구성해야 한다.

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/cyclonedds_ws/cyclonedds.xml
source /home/unitree/ros2_realsense_456_ws/activate.bash
```

## RGB-D와 IMU launch

realsense_ros 4.56.4는 기존 4.54.1과 profile 인수 이름이 다르다.
Foxy launch에서 빈 namespace 인수는 파싱되지 않으므로 `/camera/...` 토픽을
유지하려면 `camera_namespace:=/`를 사용한다. 카메라를 다시 실행할 때
V4L2/HID 장치 상태가 남아 스트림이 열리지 않는 경우가 있어
`initial_reset:=true`도 사용한다.

```bash
source /home/unitree/ros2_realsense_456_ws/activate.bash

ros2 launch realsense2_camera rs_launch.py \
  initial_reset:=true \
  camera_namespace:=/ \
  camera_name:=camera \
  depth_module.depth_profile:=848x480x30 \
  rgb_camera.color_profile:=848x480x30 \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_sync:=true \
  align_depth.enable:=true \
  pointcloud.enable:=false \
  enable_accel:=true \
  enable_gyro:=true \
  accel_fps:=100 \
  gyro_fps:=200 \
  unite_imu_method:=2
```

주요 토픽:

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_rect_raw
/camera/aligned_depth_to_color/image_raw
/camera/imu
```

`/camera/imu`는 accel을 gyro timestamp에 선형 보간한 약 200 Hz 통합 raw IMU
토픽이다. D435i는 orientation을 직접 계산하지 않으므로 raw 메시지에서
orientation 값이 0이고 `orientation_covariance[0] == -1`인 것은 정상이다.
자세와 중력 방향은 다음 단계에서 `imu_filter_madgwick` 같은 filter가
계산해야 한다.

## 외부 Humble PC에서 받기

Go2와 PC 모두 기존 CycloneDDS를 사용한다. PC의 `cyclone` alias는
`enx00e04c361d3a` 인터넷 인터페이스만 강제로 선택하므로 Go2 카메라를 받을
때는 실행하지 않는다. 기본 CycloneDDS interface 선택 상태에서 Go2 전용
유선 인터페이스 `eno1`을 통해 정상 수신되는 것을 확인했다.

```bash
cd /home/cvr/Desktop/sj/go2_lidar_slam
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# 이전 DDS 환경에서 시작한 daemon이 있으면 새 환경으로 다시 띄운다.
ros2 daemon stop || true
ros2 topic list | grep '^/camera/'
```

검증된 외부 수신 결과는 컬러와 정렬 깊이 각각 약 `29.75 Hz`, 통합 IMU
`199.57 Hz`다. 이때 PC에서는 `cyclone` alias를 실행하지 않았고 Go2에서는
기존 `cyclonedds.xml`을 사용했다.

이전에 보인 반복적인 `bad_alloc caught: std::bad_alloc`은 카메라 SDK의
메모리 부족이 아니라 임시 FastDDS 시험에서 Foxy의
`libfastrtps.so.2.1.4`가 출력한 메시지였다. 위 CycloneDDS 환경으로 정확히
실행한 시험에서는 나타나지 않았다. 같은 메시지가 다시 보이면 새 shell에서
`echo "$RMW_IMPLEMENTATION"`이 `rmw_cyclonedds_cpp`인지 먼저 확인한다.

## IMU 값 검증

카메라와 Go2를 정지시킨 뒤 다른 Go2 shell에서 실행한다.

```bash
source /home/unitree/ros2_realsense_456_ws/activate.bash

python3 \
  /home/unitree/ros2_realsense_456_ws/verification/realsense_imu_probe.py \
  --topic /camera/imu \
  --duration 30 \
  --output-json \
  /home/unitree/ros2_realsense_456_ws/verification/new_imu.json
```

probe의 1차 합격 기준:

- 통합 IMU rate 160–240 Hz
- accel norm 평균 7–12 m/s²
- 모든 gyro 축의 절댓값 최대치 1 rad/s 미만
- frame `camera_imu_optical_frame`

기존 SDK 2.54.2와 펌웨어 5.17.0.10 조합에서는 같은 시험에서 정지 gyro
최대치가 약 33.56 rad/s로 측정되어 실패했다.

## 기존 환경으로 복귀

신규 환경을 source했던 shell을 재사용하지 말고 새 SSH shell을 열어 시작
메뉴의 `1`번(Foxy)을 선택한 뒤 기존 workspace만 source한다.

```bash
source /home/unitree/ros2_realsense_ws/install/local_setup.bash
```

기존 launch는 기존 인수 이름을 그대로 사용한다.

```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=848x480x30 \
  rgb_camera.profile:=848x480x30 \
  enable_infra1:=false \
  enable_infra2:=false \
  enable_sync:=true \
  align_depth.enable:=true \
  pointcloud.enable:=false
```

## 빌드 재현 참고

- Go2에서 GitHub DNS가 동작하지 않아 PC에서 고정 tag를 받은 뒤 SHA-256을
  검증하여 전송했다.
- librealsense `v2.56.5`는 native V4L2/HID backend,
  `BUILD_TOOLS=ON`, 사용자 install prefix로 빌드했다.
- RSUSB backend도 먼저 검증했지만 Go2의 `usbfs_memory_mb=16` 환경에서
  IMU는 정상이어도 RGB-D 시작이 실패했다. 커널/sysfs를 바꾸지 않는 원칙을
  지키기 위해 native backend로 다시 빌드했고, `rs-record`와 ROS RGB-D
  30 Hz 수신을 모두 확인했다.
- realsense_ros `4.56.4`에는
  `patches/realsense-ros-4.56.4-foxy.patch`를 적용했다.
  이 patch는 Foxy에서 legacy ament link export를 사용하고 Foxy용 tf2
  `Quaternion.h`를 선택한다.
- Go2에 `xacro`가 없어 목표에 필요하지 않은 `realsense2_description`은
  빌드하지 않았다. `realsense2_camera_msgs`와 `realsense2_camera`만
  빌드했다.
- D435i는 시작 로그에서 저장된 IMU calibration을 찾지 못해 기본
  intrinsic/extrinsic을 사용한다고 경고한다. raw IMU의 주기와 물리값은
  정상화됐지만, 정밀한 중력 constraint를 연결하기 전에는 IMU-camera
  extrinsic과 정지 bias를 별도로 검증해야 한다.
- 현재 단계는 raw IMU 정상화까지다. Madgwick와 RTAB-Map gravity constraint
  연결은 포함하지 않는다.

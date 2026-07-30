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
| 외부 DDS | 기존 설정 | FastDDS + 전용 Ethernet profile |
| 카메라 펌웨어 | 5.17.0.10 | 변경하지 않음 |

신규 SDK와 wrapper 경로:

```text
/home/unitree/librealsense-2.56.5-src
/home/unitree/librealsense-2.56.5-build
/home/unitree/librealsense-2.56.5-install
/home/unitree/ros2_realsense_456_ws
```

시스템 SDK, 커널, DKMS, udev rule, 펌웨어와 기존 workspace는 변경하지
않는다. 기존 환경과 신규 환경을 같은 shell에서 차례로 source하지 말고 항상
새 shell을 사용한다.

## 신규 환경 실행

Go2에 접속한 새 shell에서 다음 한 줄을 실행한다.

```bash
source /home/unitree/ros2_realsense_456_ws/activate.bash
```

이 스크립트는 다음 순서로 환경을 구성한다.

1. `/opt/ros/foxy/setup.bash`
2. librealsense 2.56.5의 `bin`, CMake prefix, library path
3. `/home/unitree/ros2_realsense_456_ws/install/local_setup.bash`
4. `rmw_fastrtps_cpp`와 Go2 `eth0` 전용 FastDDS profile

환경이 올바른지 확인하려면 다음을 실행한다.

```bash
rs-enumerate-devices --version
ros2 pkg xml realsense2_camera | grep -m1 '<version>'
```

예상 버전은 각각 `2.56.5.0`, `4.56.4`다.

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

PC에는 NIC가 두 개 있으므로 기본 DDS 설정을 사용하면 인터넷 NIC와 Go2
전용 NIC 사이에서 discovery가 불안정하고 대용량 image가 전달되지 않았다.
Go2 카메라 launch에는 활성화 스크립트가
`config/fastdds_go2.xml`에 대응하는 원격 profile을 자동 적용한다. PC에서는
저장소 루트의 `config/fastdds_pc.xml`을 적용한다.

```bash
cd /home/cvr/Desktop/sj/go2_lidar_slam
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds_pc.xml"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# 이전 CycloneDDS daemon이 있으면 middleware 변경 전에 한 번 종료한다.
ros2 daemon stop || true
ros2 topic list | grep '^/camera/'
```

검증된 외부 수신 결과는 컬러 `29.978 Hz`, 정렬 깊이 `29.978 Hz`, 통합
IMU `199.558 Hz`다. PC나 Go2의 `192.168.123.*` 주소 또는 NIC 구성이
변경되면 두 XML의 whitelist 주소도 함께 갱신해야 한다.

Foxy의 `libfastrtps.so.2.1.4`는 다른 DDS participant가 남은 상태에서
discovery를 시작할 때 `bad_alloc caught: std::bad_alloc`을 일시적으로
출력할 수 있었다. 이 문자열의 출처는 카메라 SDK가 아니라 FastDDS였고,
검증 당시 메모리는 5.9 GiB available이었으며 위 수신률에는 영향이 없었다.
계속 반복되거나 토픽이 보이지 않으면 양쪽 ROS daemon을 종료하고 두
FastDDS profile을 적용한 새 shell에서 다시 시작한다.

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

신규 환경을 source했던 shell을 재사용하지 말고 새 shell을 연다.

```bash
source /opt/ros/foxy/setup.bash
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

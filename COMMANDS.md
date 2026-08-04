# 실행 명령어

## 공통 환경 설정

```bash
cd /home/cvr/Desktop/sj/go2_lidar_slam
source /opt/ros/humble/setup.bash
source /home/cvr/Desktop/sj/go2_ws/install/setup.bash
source install/setup.bash
```

## VO-Go2 Odometry 비교

비교 노드:

```bash
ros2 launch go2_rtabmap_launch vo_odom_comparison.launch.py
```

RViz2 비교 화면(새 터미널):

```bash
rviz2 -d src/go2_rtabmap_launch/config/vo_odom_comparison.rviz
```

## LiDAR SLAM

```bash
ros2 launch go2_rtabmap_launch slam.launch.py \
  reset_db:=true \
  rtabmap_viz:=true
```

## Go2 Odometry 기반 Visual SLAM

```bash
ros2 launch go2_rtabmap_launch visual_slam.launch.py \
  reset_db:=true \
  rtabmap_viz:=true
```

## Visual Mapping 모드

공식 Unitree 조종기로 Go2를 움직이면서 맵만 작성한다. 이 모드는 Nav2와
PC의 Sport 명령 브리지를 시작하지 않는다.

```bash
ros2 launch go2_nav2_bringup visual_mapping_mode.launch.py \
  database_path:=maps/visual/active/rtabmap.db \
  reset_db:=true
```

## Visual Localization + Nav2 모드

먼저 로봇을 움직이지 않는 기본 설정으로 저장 DB, localization, TF,
costmap과 경로 계획을 검증한다. RViz가 자동으로 열리고 `/lowstate`의 실제
관절값을 반영한 Go2 URDF가 `map` 위에 표시된다.

```bash
ros2 launch go2_nav2_bringup visual_navigation_mode.launch.py \
  database_path:=maps/visual/active/rtabmap.db \
  enable_motion:=false \
  rviz:=true \
  show_robot_model:=true \
  lowstate_topic:=/lowstate
```

다른 터미널에서 실시간 관절과 localization TF를 확인한다.

```bash
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo base_link FL_hip
ros2 run tf2_ros tf2_echo map base_link
```

안전 구역과 비상 정지 수단을 준비하고 실제 이동을 허용할 때만 다음처럼
명시한다.

```bash
ros2 launch go2_nav2_bringup visual_navigation_mode.launch.py \
  database_path:=maps/visual/active/rtabmap.db \
  enable_motion:=true
```

별도 RViz를 사용할 때는 `rviz:=false`, RobotModel 자체가 필요 없을 때는
`show_robot_model:=false`를 지정한다.

`go2_ws`에서는 `unitree_api`, `unitree_go` 메시지와 `go2_description`의
일반 URDF만 사용한다. `go2_driver` 또는 `go2_bringup`은 함께 실행하지 않는다.

## VO 기반 Visual SLAM

```bash
ros2 launch go2_rtabmap_launch vo_visual_slam.launch.py reset_db:=true
```

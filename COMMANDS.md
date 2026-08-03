# 실행 명령어

## 공통 환경 설정

```bash
cd /home/cvr/Desktop/sj/go2_lidar_slam
source /opt/ros/humble/setup.bash
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

## VO 기반 Visual SLAM

```bash
ros2 launch go2_rtabmap_launch vo_visual_slam.launch.py reset_db:=true
```

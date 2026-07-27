/utlidar/cloud 한 메시지는 한 순간에 찍힌 사진이 아니라 약 0.06~0.07초 동안 찍힌 점들의 묶음입니다.

  우리가 확인한 raw cloud에는 time 필드가 있었죠.

  /utlidar/cloud
    point time: 0.0 ~ 약 0.064초

  즉 한 cloud 안에서도:

  첫 번째 점: 로봇이 A 위치일 때 찍힘
  마지막 점: 로봇이 A에서 조금 움직인 뒤 찍힘

  일 수 있습니다.

  per-point deskew는 이걸 각 점마다 보정하는 겁니다.

  point 1은 t=0.00초 pose로 보정
  point 2는 t=0.01초 pose로 보정
  point 3은 t=0.02초 pose로 보정
  ...

  그래서 전체 cloud를 “한 순간에 찍은 것처럼” 펴는 작업입니다.

  그런데 지금 bridge가 하는 건 더 단순합니다.

  /utlidar/cloud 전체를 하나의 덩어리로 보고
  cloud header stamp 시각의 odom pose 하나만 사용해서
  utlidar_lidar -> base_link로 한 번에 변환

  즉 각 point의 time을 따로 읽어서 보정하지 않습니다.

  그래서 차이는 이겁니다.

  per-point deskew 있음:
    cloud 안의 각 점을 찍힌 시간별 pose로 각각 보정

  현재 bridge:
    cloud 전체를 같은 시간에 찍힌 것으로 가정하고 한 번에 변환
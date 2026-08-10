# 검증 결과와 실험 근거

이 문서는 현재 설정을 채택한 근거와 저장소에 남아 있는 검증 결과를 한곳에 모은다.
구조와 데이터 흐름은 [시스템 아키텍처](ARCHITECTURE.md)를 참고한다.

## 근거를 읽는 방법

이 문서에서는 서로 성격이 다른 근거를 다음처럼 구분한다.

| 등급 | 의미 | 예시 |
|---|---|---|
| A: 코드·자동 테스트 | 현재 저장소에서 다시 확인할 수 있는 동작과 기본값 | launch, YAML, Python test |
| B: 보존 산출물 | 저장된 DB, rosbag, JSON 분석 결과 | `maps/`, `bags/`, `results/` |
| C: 기존 관측 기록 | 당시 실기 실행에서 기록했지만 원본 전체를 다시 계산하지 않은 수치 | 이전 아키텍처·상태 문서의 실험 메모 |
| D: 미검증 | 코드에는 경로가 있지만 실기 완료 근거가 충분하지 않은 기능 | 순수 Visual localization |

대용량 DB와 rosbag 원본은 공개 저장소에 포함하지 않는다. 공개 저장소에서는
`results/`의 요약 결과와 이 문서에 기록한 통계를 근거로 제공한다.

등급 B와 C의 수치는 특정 주행 환경에서 얻은 결과다. 반복성이나 절대 정확도를 보장하는
벤치마크가 아니며, 서로 다른 실험의 수치를 한 실험처럼 직접 비교하면 안 된다.

## 현재 자동 검증 기준

2026-08-05에 다음 명령을 저장소 루트에서 실행했다.

```bash
python3 -m pytest -q \
  src/go2_rtabmap_bridge/test \
  src/go2_rtabmap_launch/test \
  src/go2_nav2_bringup/test \
  src/go2_nav2_control/test
```

결과는 **76 passed**였다. 이 테스트는 다음 항목을 확인한다.

- LiDAR bridge의 timestamp 보정, TF fallback 제한, point cloud 변환과 padding 제거
- Go2 odom bridge의 timestamp 보정, pose 유지·평면화 선택과 TF 발행
- 세 SLAM launch의 topic, frame, DB 경로와 주요 RTAB-Map 기본값
- visual mapping/localization의 RGB-D sync, PnP, proximity와 처리율 설정
- Visual Nav2 bringup의 DB 요구 조건, costmap·MPPI 구성과 motion 안전 기본값
- Go2 Move API command policy와 watchdog

자동 테스트는 launch와 노드 단위 계약을 검증한다. 카메라 calibration, 실제 센서 품질,
지도 정확도나 현장 주행 성공까지 대신 검증하지는 않는다.

## 보존 산출물 스냅샷

2026-08-05에 SQLite DB를 read-only로 조회한 스냅샷이다. `Node`, `Link`, `Data`는
RTAB-Map DB의 해당 테이블 행 수이며, 지도 품질 점수가 아니다.

| 모듈·용도 | 파일 | 크기 | Node | Link | Data |
|---|---|---:|---:|---:|---:|
| LiDAR 현재 DB | `maps/active/rtabmap.db` | 5,853,184 B | 452 | 262 | 452 |
| Go2 odom Visual 현재 DB | `maps/visual/active/rtabmap.db` | 286,052,352 B | 660 | 392 | 660 |
| Visual 8 Hz·15 ms 보존본 | `maps/visual/active/rtabmap_final_8hz_15ms.db` | 293,806,080 B | 643 | 221 | 643 |
| Visual PnP·proximity 보존본 | `maps/visual/active/rtabmap_pnp_proximity_8hz.db` | 179,576,832 B | 349 | 234 | 349 |
| 순수 Visual 현재 DB | `maps/visual_vo/active/rtabmap.db` | 192,155,648 B | 434 | 125 | 434 |

DB 파일이 존재하고 테이블을 읽을 수 있다는 사실은 저장 성공의 근거다. 다시 시작한
localization이 모든 위치에서 안정적이라는 뜻은 아니다.

## LiDAR SLAM 검증

### 코드와 테스트로 확인한 범위 — A

현재 `bridge_node`와 테스트에서 확인되는 계약은 다음과 같다.

- `/utlidar/robot_odom`의 첫 stamp에서 하나의 epoch offset을 계산한다.
- 같은 offset을 odom과 `/utlidar/cloud_deskewed`에 적용한다.
- 보정된 odom과 `odom -> base_link`를 발행한다.
- cloud는 보정된 시각의 TF를 우선 사용하고, 실패할 때만 최대 0.2초 차이의 최신 TF를
  fallback으로 허용한다.
- `x=y=z=intensity=0`인 padding record만 제거하고 나머지 PointCloud2 layout은 보존한다.
- cloud 전체에 하나의 rigid transform을 적용하며 point별 재-deskew는 하지 않는다.
- LiDAR RTAB-Map은 `Reg/Strategy=1`, 6DoF, point-to-plane ICP를 사용한다.
- mapping과 기존 DB를 사용하는 localization launch가 모두 존재한다.

### 실기 토픽 관측 기록 — C

2026-04-12 운용 기록에는 다음 값이 남아 있다.

| 토픽 | 당시 관측값 |
|---|---:|
| raw `/utlidar/robot_odom` | 약 151 Hz, RELIABLE |
| raw `/utlidar/cloud_deskewed` | 약 14.7 Hz, frame `odom`, point step 32 |
| bridge `/odom` | 약 150–152 Hz |
| bridge `/scan_cloud` | 약 14.6–14.8 Hz, frame `base_link` |
| RTAB-Map `mapData`·localization pose | 약 1 Hz |

이는 당시 장비와 실행 상태의 관측값이며 현재 실행의 합격 기준으로 고정된 수치는 아니다.
현재 저장된 LiDAR DB의 행 수는 위 산출물 스냅샷을 기준으로 한다.

### localization 관측과 한계 — C

기존 기록에서는 알려진 시작 자세를 준 localization은 성공했지만, 잘못 승인된 proximity
constraint가 `map -> odom`을 흔드는 현상이 있었다. 현재 localization 설정은 후보 수와
ICP acceptance를 제한하고 global scan map을 끈 구성이다. 따라서 이 경로는 완전한
global relocalization보다 **대략적인 초기 위치를 알고 시작하는 운용**에 우선 맞춰져 있다.

## Go2 odom 기반 Visual SLAM 검증

### 현재 채택 설정 — A

현재 코드와 YAML의 핵심 설정은 다음과 같다.

| 항목 | 현재값 |
|---|---:|
| RGB-D approximate sync 최대 간격 | 0.03 s |
| Go2 odom residual time offset | -0.015 s |
| `Kp/DetectorStrategy` | `8` |
| `Vis/FeatureType` | `8` |
| `Vis/EstimationType` | `1` — 3D-to-2D PnP |
| `RGBD/NeighborLinkRefining` | `true` |
| `RGBD/ProximityBySpace` | `true` |
| `RGBD/ProximityOdomGuess` | `false` |
| mapping `Rtabmap/DetectionRate` | 8.0 Hz |
| localization `Rtabmap/DetectionRate` | 2.0 Hz |

### 카메라와 odom 시간 오프셋 — C

2026-07-20의 기존 실험 기록은 47.03초 동안 color image 1,407개, raw Go2 odom
7,036개, bridge `/odom` 7,049개를 사용했다. 영상 optical-flow 회전 신호와 odom 회전
신호의 최대 상관계수는 **0.9926**이었다. odom angular velocity 기준 최적치는 약
-8~-11 ms, quaternion yaw 변화 기준은 약 -19 ms였고, 30 Hz 카메라의 시간 해상도를
고려해 중간값인 `sensor_time_offset_sec=-0.015`를 채택했다.

이 값은 첫 odom에서 계산하는 clock epoch offset과 별개다. 특정 카메라·Go2 조합에서
측정한 residual이므로 센서나 driver가 바뀌면 다시 측정해야 한다.

### PnP와 처리율을 채택한 근거 — B·C

초기 1 Hz DB에서는 인접 링크 49개 중 11개의 visual refinement가 실패했고, 프레임
사이 회전이 30° 이상인 세 구간은 모두 실패했다. 이에 visual transform을
2D-to-2D 방식에서 metric depth를 사용하는 3D-to-2D PnP로 바꾸고,
`RGBD/NeighborLinkRefining=true`를 적용했다. 처리율은 1 Hz → 5 Hz → 8 Hz 순서로
올려 검증했다.

기존 8 Hz 전체 루프 기록은 다음과 같다.

- 주행 거리 8.53 m, 시간 46.82초
- 입력 통계 349개, 최적화 지도 node 96개
- 설정 8.0 Hz, 실효 처리율 **7.43 Hz**
- 처리시간 중앙값 38.31 ms, p95 58.79 ms
- 125 ms를 넘은 처리 348개 중 1개
- neighbor refinement 95회 중 90회 성공
- 마지막 global loop closure: node 241 ↔ 59, match 314개, **194개 inlier**
- 최적화 전 neighbor chain과 closure 차이 26.4 cm / 7.85°
- 최적화 후 neighbor constraint 잔차 p95 0.82 mm / 0.20°

해당 주행에서는 과거의 큰 이중상이 육안으로 재현되지 않아 8 Hz를 현재 기본값으로
채택했다. 다만 local spatial proximity link는 생성되지 않고 global closure 하나만
생성됐으므로, 이 한 번의 주행이 proximity 효과까지 검증한 것은 아니다.

### 큰 겹침과 proximity 실험 — C

과거 큰 루프의 visual-refined neighbor chain은 검증한 loop constraint와
**61–83 cm / 7.9–11.9°** 불일치했다. 같은 구간의 raw Go2 odom loop mismatch는
**16–27 cm / 1.0–2.6°**였다. 이는 PnP neighbor refinement가 짧은 구간 정렬에는
필요하지만 긴 chain의 전역 정확도를 항상 보장하지는 않는다는 근거다.

동일한 graph 설정에서 spatial proximity만 추가한 기존 통제 실험에서는 visual
spatial detection 11개가 추가됐고, neighbor chain과 loop 사이 위치 차이 중앙값이
약 80.0 cm에서 26.0 cm로, 회전 차이 중앙값이 8.77°에서 3.47°로 줄었다. 이 때문에
`RGBD/ProximityBySpace=true`를 유지한다. 경로나 초기 drift에 따라 후보가 없으면
proximity link는 생성되지 않을 수 있다.

### 남은 얇은 경계 번짐 — C

큰 이중상이 줄어든 뒤에도 빠른 회전에서 얇은 경계 번짐이 남았다. 기존 slow/fast
관측에서는 천천히 움직일 때 영상 sharpness 중앙값이 43.1에서 61.1로 높아지고,
neighbor 잔차 p95가 4.24 cm / 1.45°에서 0.88 cm / 0.26°로 줄었다. 이 결과는 남은
번짐이 loop closure 하나의 문제가 아니라 motion blur, 프레임 간 이동량, depth 경계
노이즈와 camera extrinsic 오차의 영향을 함께 받는다는 진단 근거로 사용한다.

수동 exposure 166 → 100 실험은 feature inlier와 sharpness를 함께 개선하지 못해
기본값으로 채택하지 않았다. camera extrinsic은 현재 운용값일 뿐 정밀 calibration을
완료한 값은 아니다.

### Nav2 목표점 주행 공개 영상 — C

[Go2 Nav2 목표점 주행 실기 영상](https://youtu.be/n31tp01uUzw)은 Go2 odometry 기반
Visual 경로에서 Nav2 목표점 주행을 수행한 당시의 공개 관측 기록이다. 이 영상은 해당
구성의 실기 동작 여부를 확인하는 근거로 사용하며, 반복 성공률이나 지도·궤적의 정량
정확도를 입증하는 자료로 해석하지 않는다.

## 순수 Visual SLAM과 VO 비교

### 구현·산출물 범위 — A·B·D

순수 Visual 모드는 `rgbd_odometry`가 `/odom/vo`와 `vo_odom -> base_link`를 만들고,
별도 namespace `/rtabmap_vo`에서 mapping한다. 현재 DB에는 Node 434, Link 125,
Data 434가 저장돼 있다.

mapping launch와 비교 도구는 코드와 테스트로 확인했지만, 이 모드 전용 localization과
Nav2 launch는 없다. 따라서 현재 완료 범위는 **순수 RGB-D odometry를 사용한 mapping과
Go2 odom 비교**까지다.

### 일반 주행 비교 결과 — B

`results/vo_go2_summary.json`은 `bags/vo_go2_compare`를 분석한 결과다.

| 항목 | 결과 |
|---|---:|
| bag 길이 / 전체 message | 51.584 s / 9,218 |
| `/odom/go2` / `/odom/vo` message | 7,677 / 1,541 |
| matched pair | 1,540 |
| 비교 구간 | 51.538 s |
| timestamp gap 중앙값 / p95 / 최대 | 1.774 / 3.214 / 30.554 ms |
| Go2 / VO path length | 10.348 / 17.967 m |
| 최종 position 차이 | 0.265 m |
| position 차이 RMSE / p95 | 0.239 / 0.395 m |
| 최종 yaw 차이 | 1.107° |
| yaw 차이 RMSE / p95 | 2.218 / 3.984° |
| VO 실효율 / 최대 gap / long gap | 29.862 Hz / 0.0667 s / 0 |

### 느린 주행 비교 결과 — B

`results/vo_go2_depth_0p3_4p0_slow_summary.json`과 planar 분석 결과는
`bags/vo_go2_compare_depth_0p3_4p0_slow`에서 나왔다.

| 항목 | 결과 |
|---|---:|
| bag 길이 / 전체 message | 87.557 s / 18,250 |
| Go2 / VO / odom_info message | 13,046 / 2,602 / 2,602 |
| matched pair / 비교 구간 | 2,601 / 87.494 s |
| timestamp gap 중앙값 / p95 / 최대 | 1.673 / 3.204 / 25.144 ms |
| 3D Go2 / VO path length | 10.401 / 17.153 m |
| 최종 position 차이 | 0.364 m |
| position 차이 RMSE / p95 | 0.384 / 0.580 m |
| 최종 yaw 차이 | -11.698° |
| yaw 차이 RMSE / p95 | 6.442 / 10.542° |
| VO 실효율 / 최대 gap / long gap | 29.716 Hz / 0.1334 s / 0 |

SE(2) planar 재계산에서는 raw 2D path 비율이 VO/Go2 = 1.488, 1 Hz resampling
비율이 1.181이었다. 최종 XY 차이는 0.201 m, yaw 차이는 -11.721°였다. 첫 5초 정지
구간의 누적 이동은 Go2 0.0032 m, VO 0.1676 m였지만 net displacement는 각각
0.00065 m와 0.00075 m였다. 즉 VO가 제자리 주변에서 더 많이 흔들렸으나 멀리
이탈하지는 않은 형태였다.

### 해석 제한

두 결과의 position·yaw 차이는 **두 odometry의 불일치량**이다. 둘 중 어느 쪽도
ground truth가 아니므로 다음 결론은 내릴 수 없다.

- Go2 또는 VO 중 어느 쪽의 절대 궤적이 더 정확한가
- 실제 이동 거리가 두 path length 중 어느 값에 가까운가
- mapping graph optimization 후 전역 지도가 어느 쪽에서 더 정확한가

현재 근거로 말할 수 있는 것은 VO가 두 bag에서 약 30 Hz로 끊김 없이 출력됐고,
Go2와 비교했을 때 누적 path와 특히 느린 주행의 yaw가 유의하게 달랐다는 점이다.
정확도 판정에는 motion capture, survey point, AprilTag 기준 궤적처럼 독립된 ground
truth가 필요하다.

## 현재 결론

| 모듈 | 확인된 완료 범위 | 남은 핵심 검증 |
|---|---|---|
| LiDAR SLAM | bridge 계약, mapping DB, 알려진 시작 자세 localization 경로 | 반복 localization 성공률, 전역 초기화, 정량 지도 오차 |
| Go2 odom Visual | RGB-D mapping·localization, 8 Hz/PnP/proximity 설정, Nav2 경로 | camera extrinsic 재측정, 속도별 blur, 반복 loop·Nav2 주행 |
| 순수 Visual | RGB-D VO mapping, DB, Go2 odom 비교 bag·분석 | 독립 ground truth, 전용 localization, Nav2 통합 |

현 시점의 기본 운용 선택은 다음과 같다.

- 구조가 단순하고 카메라 조건과 무관한 기준 mapping에는 LiDAR SLAM을 사용한다.
- RGB-D loop constraint와 Nav2까지 필요한 주 경로에는 Go2 odom 기반 Visual SLAM을
  사용한다.
- 순수 Visual SLAM은 Go2 odom 의존성을 제거한 실험·비교 모듈로 유지한다.

## 다시 검증하는 방법

새 실험은 [README 빠른 시작](../README.ko.md#quick-start)의 launch 명령을 기준으로
실행하고 최소한 다음 정보를 함께 보존한다.

1. 사용한 commit과 launch 인자
2. 카메라 해상도·FPS·exposure와 장착 extrinsic
3. DB와 rosbag 원본
4. 자동 생성한 JSON/CSV 결과
5. 주행 경로, 속도, 조명과 출발 자세
6. ground truth 유무와 결론의 적용 범위

새 결과가 기존 결론과 다르면 기존 숫자를 덮어쓰기보다 날짜와 산출물 경로를 붙여 별도
실험으로 추가한다.

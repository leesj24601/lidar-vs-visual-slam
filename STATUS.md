# 프로젝트 상태

> 마지막 업데이트: 2026-05-13

## 현재 단계

Phase 5: 실기체 주행 매핑 및 localization 기본 검증 완료
Phase 6/7 계획 추가

브릿지 노드가 `/utlidar/robot_odom`을 보정해 `/odom`과 `odom -> base_link` TF로 재발행하고,
`/utlidar/cloud_deskewed`를 `base_link` 프레임 `/scan_cloud`로 변환하도록 구현했다.
`slam.launch.py`가 브릿지, 정적 TF, RTAB-Map 매핑 노드를 함께 실행하도록 구성했다.
`localization.launch.py`가 기존 RTAB-Map DB를 로드해 재위치추정 모드로 실행하도록 구성했다.
Go2 실기체에서 브릿지, 매핑, 로컬라이제이션 출력까지 확인했다.
최신 매핑 DB를 `maps/active/rtabmap.db`로 승격했다.
짧은 주행/이동 세션에서 DB 증가와 map/cloud 발행은 확인했지만, loop closure/global relocalization은 아직 안정화되지 않았다.

## 완료

- `SLAM_PLAN.md`에 SLAM 구현 계획 작성
- `docs/GO2_REFERENCE.md`에 Go2 ROS2 실측 레퍼런스 정리
- `docs/adr/001-slam-tool-selection.md`에 SLAM 도구 및 브릿지 아키텍처 선택 기록
- `docs/RUNBOOK.md`에 실행 및 트러블슈팅 절차 작성
- 구현 단계를 다음과 같이 정의:
  - Phase 1: 워크스페이스 스캐폴딩
  - Phase 2: Go2 RTAB-Map 브릿지 구현
  - Phase 3: RTAB-Map 매핑 구성
  - Phase 4: 로컬라이제이션 구성
  - Phase 5: 실기체 검증 및 튜닝
- `src/go2_rtabmap_bridge` Python 패키지 생성
- `src/go2_rtabmap_launch` launch/config 패키지 생성
- `maps/active/` 및 `maps/sessions/` 디렉터리 생성
- `bridge_node` 기본 엔트리포인트 추가
- `/utlidar/robot_odom` 구독 및 timestamp offset 보정 구현
- `/odom` 재발행 및 `odom -> base_link` TF 발행 구현
- 발행한 TransformStamped를 내부 `tf_buffer`에 `set_transform()`으로 저장
- `/utlidar/cloud_deskewed` BEST_EFFORT 구독 구현
- `lookup_transform('base_link', 'odom', corrected_time)` 기반 cloud 변환 구현
- `/scan_cloud` 발행 구현
- `rtabmap_lidar_indoor.yaml` 매핑 파라미터 작성
- `slam.launch.py` 매핑 launch 구현
  - `base_link -> utlidar_lidar` 정적 TF
  - `go2_rtabmap_bridge` 노드 실행
  - `rtabmap_slam/rtabmap` 노드 `namespace='rtabmap'` 실행
  - 입력 remap: `odom -> /odom`, `scan_cloud -> /scan_cloud`
  - `database_path`, `reset_db`, `use_sim_time` launch 인자 제공
  - `rviz`, `rtabmap_viz` launch 인자 제공, 기본값 `false`
- `reset_db:=true`일 때 기존 DB 및 SQLite sidecar 파일 삭제 처리 구현
- `localization.launch.py` 구현
  - 기존 `database_path` 파일 존재 검사
  - `base_link -> utlidar_lidar` 정적 TF
  - `go2_rtabmap_bridge` 노드 실행
  - `rtabmap_slam/rtabmap` 노드 `namespace='rtabmap'` 실행
  - 입력 remap: `odom -> /odom`, `scan_cloud -> /scan_cloud`
  - `Mem/IncrementalMemory=false`
  - `Mem/InitWMWithAllNodes=true`
  - `initial_pose`, `use_sim_time`, `rviz`, `rtabmap_viz` launch 인자 제공
- 실제 Go2 cloud padding layout(`point_step=32`)에 맞춰 raw `PointCloud2.data` layout 보존 변환 구현
- cloud TF exact lookup 실패 시 최신 TF fallback 허용(`tf_latest_fallback_tolerance_sec=0.2`)
- Phase 5 정지 상태 실기체 검증 완료
- Phase 5 짧은 주행/이동 매핑 검증 완료
  - `maps/active/rtabmap.db` 저장 확인
  - `/rtabmap/mapData`, `/rtabmap/cloud_map`, `/rtabmap/map` 발행 확인
  - 저장 DB로 localization 재실행 확인
- 최신 세션 DB를 active DB로 승격
  - source: `maps/sessions/20260513_130033_odom_mapping/rtabmap.db`
  - active: `maps/active/rtabmap.db`
  - 기존 active DB 백업: `maps/backups/active_20260513_142225/rtabmap.db`
- RTAB-Map LiDAR-only localization 한계 확인
  - 초기 pose를 매핑 시작점 근처로 주면 map/scan 정렬 성공
  - 후속 proximity/local match가 yaw가 다른 근접 노드에 붙으면 `map -> odom`이 다시 흔들릴 수 있음
  - 현재 구성은 완전한 kidnapped/global relocalization보다 known-start localization에 가까움
- `SLAM_PLAN.md`에 후속 구현 단계 추가
  - Phase 6: `ALIGN -> LOCK -> TRACKING` known-start localization 안정화
  - Phase 7: Scan Context 기반 global relocalization PoC

## 진행 중

- Phase 6 known-start localization 안정화 설계/구현 대기
- Phase 7 Scan Context 기반 global relocalization PoC 설계 대기

## 막힌 항목

- 짧은 주행/이동 세션에서는 `loop_closure_id`가 계속 0이었음
- RTAB-Map LiDAR-only ICP/proximity 구성은 initial pose 없이 전역 후보를 안정적으로 찾지 못함
- 잘못된 proximity/local match가 한 번 accepted되면 마지막 `map -> odom` 보정이 유지되어 자동 복구가 어려움

## 다음 작업

- Phase 6 구현
  - `localization.launch.py`에 align/tracking 운용 모드 또는 lock 옵션 추가
  - 매핑 시작점/도킹 위치 등 known start pose 목록 문서화
  - initial pose 주입 후 정렬 성공 시 `RGBD/ProximityBySpace=false`로 잠그는 절차 정리
  - lock 이후 1~2m 저속 주행에서 map/scan 정렬 유지 확인
- Phase 7 설계 및 PoC
  - Scan Context descriptor index 저장 형식 결정
  - `/scan_cloud` query로 top-K 후보 검색
  - 후보별 ICP/GICP 검증 후 `/rtabmap/initialpose` 주입
  - RTAB-Map proximity only, ALIGN-LOCK-TRACKING, Scan Context + ICP 세 모드 비교
- Go2 실내 폐루프 주행 검증은 Phase 6/7 평가 데이터로 계속 수행

## 최근 검증

- 문서 구조 확인
- `SLAM_PLAN.md`의 Phase 구조 확인
- `docs/RUNBOOK.md`의 명령 흐름 확인
- Phase 1 패키지 파일 생성 확인
- `colcon list`에서 `go2_rtabmap_bridge`, `go2_rtabmap_launch` 인식 확인
- `colcon build --symlink-install` 성공
- `ros2 pkg list`에서 두 패키지 확인
- `ros2 run go2_rtabmap_bridge bridge_node` smoke test 성공
- `ros2 launch go2_rtabmap_launch slam.launch.py` scaffold 로딩 확인
- Phase 1 당시 `ros2 launch go2_rtabmap_launch localization.launch.py` scaffold 로딩 확인
- `colcon test` 실행: 테스트 0개, 두 패키지 모두 OK
- 합성 `/utlidar/robot_odom` 및 `/utlidar/cloud_deskewed` 발행 smoke test 성공
  - `/odom` 출력 수신
  - `/scan_cloud` 출력 수신 및 `frame_id=base_link` 확인
  - `/tf`에서 `odom -> base_link` 확인
- `slam.launch.py` smoke test 성공
  - `/go2_rtabmap_bridge`, `/utlidar_static_tf`, `/rtabmap/rtabmap` 노드 기동 확인
  - RTAB-Map이 `/odom`, `/scan_cloud`를 approx sync로 구독하는 것 확인
  - `/rtabmap/mapData`, `/rtabmap/cloud_map`, `/rtabmap/info` publisher 확인
  - `reset_db:=true`가 기존 DB 및 `-wal` 파일을 삭제하는 것 확인
- 전체 launch 합성 데이터 smoke test 성공
  - 합성 `/utlidar/robot_odom`, `/utlidar/cloud_deskewed` 발행
  - `/rtabmap/mapData` 수신 확인
- `ros2 launch go2_rtabmap_launch slam.launch.py --show-args`에서 `rviz`, `rtabmap_viz` 인자 확인
- `slam.launch.py` 기본 실행에서 GUI 노드가 뜨지 않고 core 노드만 뜨는 것 확인
- `localization.launch.py` 컴파일 및 build 성공
- `localization.launch.py --show-args`에서 `database_path`, `initial_pose`, `rviz`, `rtabmap_viz` 인자 확인
- 존재하지 않는 DB로 localization 실행 시 명확한 오류로 실패 확인
- 기존 임시 DB(`/tmp/go2_phase3_full/rtabmap.db`)로 localization launch smoke test 성공
  - `/go2_rtabmap_bridge`, `/utlidar_static_tf`, `/rtabmap/rtabmap` 노드 기동 확인
  - `Mem/IncrementalMemory=false`, `Mem/InitWMWithAllNodes=true` 확인
  - RTAB-Map이 `/odom`, `/scan_cloud`를 approx sync로 구독하는 것 확인
  - `/rtabmap/localization_pose` publisher 확인
- 합성 데이터로 `/rtabmap/localization_pose` 수신 smoke test 성공
- Go2 원천 토픽 실측
  - `/utlidar/robot_odom`: 약 151Hz, RELIABLE publisher 1개
  - `/utlidar/cloud_deskewed`: 약 14.7Hz, RELIABLE publisher 1개, `frame_id=odom`, `point_step=32`
- 브릿지 실기체 검증
  - `/odom`: 약 150~152Hz
  - `/scan_cloud`: 약 14.6~14.8Hz, `frame_id=base_link`
  - `tf2_echo odom base_link` 정상 출력
- `slam.launch.py` 실기체 정지 상태 검증
  - `/rtabmap/mapData`: 약 1Hz
  - `/rtabmap/cloud_map`: `frame_id=map` 수신
  - `/rtabmap/map`: `frame_id=map` 수신
  - `/tmp/go2_phase5_slam2/rtabmap.db` 생성 확인
  - `loop_closure_id=0` 확인(정지 상태이므로 예상)
- `localization.launch.py` 실기체 정지 상태 검증
  - `/rtabmap/localization_pose`: 약 1Hz
  - pose `frame_id=map` 수신
  - `Mem/IncrementalMemory=false`, `Mem/InitWMWithAllNodes=true` 확인
- `slam.launch.py` 실기체 짧은 주행/이동 검증
  - `/odom`: 약 150Hz
  - `/scan_cloud`: 약 14.7Hz
  - `/rtabmap/mapData`: 약 0.97~0.98Hz
  - `/rtabmap/cloud_map`: 약 1Hz, `frame_id=map`
  - `/rtabmap/map`: `frame_id=map`
  - `maps/active/rtabmap.db`: 약 2.8MB 저장
  - `loop_closure_id`: 45초 추가 모니터링 동안 0 유지
- `maps/active/rtabmap.db`로 localization 재검증
  - `/rtabmap/localization_pose`: 약 0.98Hz
  - pose header `frame_id=map`
  - `Mem/IncrementalMemory=false`, `Mem/InitWMWithAllNodes=true`
- RTAB-Map odom-only baseline 반영
  - `odom_sensor_sync=true`
  - `Reg/Force3DoF=true`
  - `RGBD/NeighborLinkRefining=false`
  - `RGBD/OptimizeFromGraphEnd=false`
  - `RGBD/ProximityBySpace=false`
  - `RGBD/ProximityOdomGuess=false`
  - `RGBD/ProximityPathMaxNeighbors=0`
- RViz용 RTAB-Map map publishing 설정 반영
  - `map_always_update=true`
  - `map_filter_radius=0.0`
  - `map_filter_angle=0.0`
  - `cloud_output_voxelized=false`
- Nav2용 2D occupancy grid 튜닝 시도 후 보류
  - `Grid/3D=false`, `Grid/NormalsSegmentation=false`, `Grid/RayTracing=true` 조합은 `/rtabmap/cloud_map`을 2D처럼 악화시켜 되돌림
  - 현재는 3D cloud map 확인 품질 유지를 우선
- `localization.launch.py`에 localization 전용 proximity override 반영
  - `RGBD/ProximityBySpace=true`
  - `RGBD/ProximityOdomGuess=true`
  - `RGBD/ProximityPathMaxNeighbors=1`
  - `RGBD/ProximityMaxGraphDepth=10`
  - `RGBD/ProximityGlobalScanMap=false`
  - `RGBD/OptimizeMaxError`를 `optimize_max_error` launch 인자로 노출, 기본값 `3.0`
  - `RGBD/MaxOdomCacheSize=10`
  - `Icp/CorrespondenceRatio=0.3`
  - `Icp/MaxTranslation=0.25`
- 최신 active DB 확인
  - `maps/active/rtabmap.db`: Node 227, Link 158, Data 227
  - DB 시작 노드 pose: `x=-0.171703`, `y=0.161575`, `yaw=2.870674 rad`
- rtabmap_viz 기반 localization 실험
  - 시작점 근처에서 `initial_pose` 주입 시 map/scan 정렬 성공
  - 후속 proximity link가 저장 노드 59, 63, 134, 136 등 yaw가 다른 근접 노드에 붙으며 `map -> odom`을 흔드는 현상 확인

## 메모

- 이 파일에는 현재 프로젝트 상태만 기록한다.
- 설계 세부사항은 `SLAM_PLAN.md`에 둔다.
- 반복 실행 명령은 `docs/RUNBOOK.md`에 둔다.
- Go2 실측 환경 정보는 `docs/GO2_REFERENCE.md`에 둔다.

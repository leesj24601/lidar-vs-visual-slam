# ADR 001 — SLAM 도구 및 브릿지 아키텍처 선택

> 작성일: 2026-04-11

## Decision

Python 브릿지 노드 + rtabmap ICP SLAM (`approx_sync=true`)

## Drivers

1. **ROS2 Humble apt 패키지 제약**: 커스텀 빌드 없이 apt 패키지(rtabmap 0.22.1)만 사용
2. **Go2 타임스탬프 비표준**: 약 461일 과거 시간축 — 원천 수정 불가, 브릿지에서 보정 필수
3. **개발 속도 우선**: 프로토타입 단계에서 Python으로 빠른 검증 후 필요 시 C++ 전환

## Alternatives Considered

| 대안 | 기각 이유 |
|------|-----------|
| **KISS-ICP** | apt 설치 가능하고 단순하지만 루프 클로저 없음 — 넓은 공간 매핑 시 드리프트 누적. rtabmap이 루프 클로저 + 맵 관리 + 로컬라이제이션을 통합 제공 |
| **C++ 브릿지 노드** | 포인트클라우드 변환 성능 우수하나 개발 시간 3~5배 증가. tf2_sensor_msgs Python 구현이 이미 numpy einsum 벡터화를 사용하므로 성능 차이 미미 |
| **cloud를 odom 프레임 그대로 전달** | `frame_id='odom'`으로 변환 없이 전달 시 rtabmap ICP의 의미론적 기준점이 로봇 본체가 아닌 odom 원점이 되어 SLAM 품질 저하 |
| **`now()` 단순 대체** | odom-cloud 간 상대 시간 관계 파괴 → ICP 정합 시 모션보정 무효화 |

## Why Chosen

rtabmap은 루프 클로저, 맵 관리, 로컬라이제이션 모드를 단일 패키지로 제공하며 apt 설치가 가능하다. Python 브릿지는 타임스탬프 보정 로직을 투명하게 구현하고 디버깅이 용이하다. 실제 Go2 cloud는 padding 있는 `PointCloud2` layout이므로 브릿지는 raw data layout을 보존하고 numpy로 `x/y/z`만 변환한다.

## Consequences

- **긍정**: 빠른 프로토타이핑, 단일 언어(Python) 유지보수, ROS2 표준 TF 트리 활용
- **부정**: GC pause로 인한 타임스탬프 지터 이론적 가능성, C++ 대비 결정론적 타이밍 미보장
- **리스크 완화**: SingleThreadedExecutor로 `_time_offset` 레이스 컨디션 차단, odom 콜백에서 `tf_buffer.set_transform()`으로 내부 TF buffer를 즉시 채움, TF lookup timeout으로 가정 실패 처리

## Follow-ups

- Phase 5 주행 검증에서 SLAM 품질과 루프 클로저 확인
- Python 병목 확인 시(14Hz 미달) cloud 변환만 C++ composable node로 교체
- 장시간(1시간+) 매핑 세션에서 clock drift 영향 모니터링

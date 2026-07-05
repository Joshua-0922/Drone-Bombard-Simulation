# Issue #023 — Payload OdometryPublisher 2D mode (default) → z 좌표 항상 0

**Created**: 2026-06-07
**Status**: RESOLVED (SDF fix applied)
**Impact**: CRITICAL — v3/v4 의 모든 진단을 왜곡한 root cause

## 한 줄 요약

`payload_0~3/model.sdf` 의 `OdometryPublisher` 에 `<dimensions>` 미지정 → gz-sim8 default = 2 → z 와 vz 가 항상 0 으로 publish.

`drop_calculator` 가 `payload_z ≤ 0.04` 시 즉시 impact 처리 → drop_error = drone 의 분리 시점 horizontal 위치 (= d_xy_at_trigger).

→ 결과: drop_error 가 진짜 ground impact 가 아닌 d_xy 였음. v3 의 모든 통계 (gap 1.73m, correlation 0.955) 가 measurement artifact.

## 진단 경로

1. v3 (396 drops) 분석: drop_error mean 3.59m, **drop_error ≈ d_xy** (모든 speed bucket 에서 1.0-1.05배)
2. 가설 1 (잘못됨): velocity inheritance failure → 옵션 A, G-a, G-b 모두 무용한 처방
3. minimal_test 결과: multi-model DetachableJoint 가 velocity 를 정상 보존 (101.5%)
4. 학습 환경 직접 측정: drone alt 4.58m 인데 **payload odom z = 0**
5. WebFetch (Gazebo docs): `<dimensions>` default = 2, 2D 모드는 z/vz 미포함
6. SDF fix 후 검증: drone alt 4.06m → payload odom z = 4.053m ✓

## Fix

`gazebo_models/payload_{0,1,2,3}/model.sdf` 의 OdometryPublisher 에 추가:

```xml
<dimensions>3</dimensions>
<odom_publish_frequency>50</odom_publish_frequency>  <!-- 10 → 50 (정밀도) -->
```

## v3/v4 의 모든 진단 재해석

| 과거 결론 | 실제 |
|---|---|
| DetachableJoint 가 분리 시 velocity reset | 처음부터 정상 (100% 보존) |
| 옵션 A (ApplyLinkWrench) 가 16% gap 감소 효과 | 측정 artifact 안의 noise 변동 |
| correlation(gap, speed_xy) = 0.955 | speed 클수록 drop trigger 시 d_xy 큼 (당연한 statistical, mechanism 무관) |
| vfast (4+ m/s) 의 gap 6.99m | drone 의 그 시점 horizontal 위치, payload 와 무관 |

## 정리한 잔존 코드

- `drone_drop_env.py`: 옵션 A (wrench_pub, EntityWrench 임포트, publish_drop 의 wrench publish, bridge config wrench topic, DROP_SKIP_WRENCH toggle) 모두 제거
- `worlds/x_marker_world.sdf`: `ApplyLinkWrench` plugin 제거
- `drop_calculator_node.py`: `/tmp/w_verify.csv` 로깅 제거 (검증용 임시)

## 관련

- Issue #007 (CCIP prediction gap) — 이 root cause 가 issue #007 의 모든 가설을 무효화
- minimal_test mtest2 (multi-model DetachableJoint velocity preservation 검증) → PASS 101.5%

## 교훈

- "보이는 통계 패턴" 과 "진짜 mechanism" 의 차이를 늘 의심하라
- 옵션 A 의 부분적 성공 (gap 16% 감소) 이 잘못된 가설을 강화했음 — 작은 신호의 noise 일 수 있음
- minimal test PASS 인데 학습 데이터와 모순될 때, 학습 환경의 **측정 자체** 를 의심하라

# Issue #022 — 학습 fps 병목 (낮은 6 fps 정체)

생성일: 2026-06-07
상태: 진단 완료, 처방 후보 다수 / 격리 실험 필요

## 1. 문제

학습 fps 가 일관되게 낮음. 300k step 학습이 10-12 시간 소요.

| 라운드 | fps | 학습 시간 (300k) |
|---|---|---|
| Round 7 v3 (Phase 1 endpoint) | 8 | ~10시간 |
| Phase 1 redux v1 (89k 시점) | 2-8 | 측정 못 함 |
| Phase 1 redux v2 (수동 중단 143k) | 8 | 잔여 8.8h 예상 |
| Phase 1 redux v3 (수동 중단 254k) | 6 | 잔여 5h 예상 |
| 옵션 W 시험 | 4-6 | 너무 느림 |
| 옵션 A 시험 | 6 | 정상 |

목표 fps: 15-25 (학습 시간 3-5시간 단축).

## 2. Root Cause 분석

### Per-step 시간 분해 (이론)

```
obs_wait_timeout : 20 ms
state snapshot   : <1 ms
reward 계산      : <1 ms
policy forward   : 5-10 ms (GPU, 256x256 network)
gradient step    : 10-20 ms (SAC critic + actor + ent)
─────────────────────────
이론 최소        : ~40 ms → 25 fps
```

### Per-episode overhead (실측 추정)

```
_kill_episode    : 2-6 s (timeout 2s × 3 nodes)
_start_episode   : 1-2 s (3 ROS 노드 spawn + DDS discovery)
_wait_for_cruise : 1-10 s (takeoff + cruise altitude)
gz_reset_poses   : 0.5 s
_kill_infra      : 10-15 s (drop 시만, full restart)
─────────────────────────
fast path 평균   : ~2.5 s/episode
drop path 평균   : ~15 s/episode
```

### 실측 분해 (v3, fps 8 기준)

```
ep_len_mean = 112 step ≈ 5.6s 학습 시간
episode 총 시간 = 112/8 = 14s
→ reset overhead = 14 - 5.6 = 8.4 s/episode
```

대부분의 시간이 reset overhead 임을 확인.

### 역설적 발견 — fps 가 정책 품질과 반비례

```
학습 초기 (random):  episode 길게 (드론 헤맴) → fps 18
학습 후기 (학습됨):  episode 짧게 (drop 빨리)  → fps 8
```

**정책이 잘 학습할수록 학습이 느려지는 구조적 문제.**
짧은 episode → reset 비율 증가 → fps 감소.

## 3. 처방 후보

### 🟢 A. Persistent infra (Episode 노드 유지)

매 episode 마다 ROS 노드 죽이지 말고 "reset signal" 보냄.
- mission_manager / drone_controller / drop_calculator 의 reset handler 추가

| | |
|---|---|
| 효과 | _kill_episode + _start_episode + DDS discovery 모두 제거 (5-10s 절약) |
| 위험 | ROS 노드 상태 잔존, race condition |
| 작업량 | 큼 (1-2일) |
| **예상 fps** | **10-15** |
| 검증 상태 | 미검증 |

### 🟡 B. Takeoff 생략

```python
# _gz_reset_poses 에서 드론을 cruise altitude 에 직접 spawn
drone_position = (0, 0, 5.0)  # 5m
# PX4 OFFBOARD mode 즉시 진입
```

| | |
|---|---|
| 효과 | _wait_for_cruise 5-10s → 0-1s 제거 |
| 위험 | PX4 EKF 안정화 시간 필요, 초기 비행 불안정 |
| 작업량 | 중간 (PX4 mission 로직 수정) |
| **예상 fps** | **8-12** |
| 검증 상태 | 미검증 |

### 🟡 C. Drop case 의 _kill_infra 제거

현재 코드 주석:
```
# _gz_world_reset(model_only) caused ODE AABB integer overflow crash
# because residual payload velocity after drop destabilises physics.
```

해결안:
1. Drop 후 episode 끝나기 전 zero_velocity command (드론 감속)
2. 그 다음 model_only world reset (DetachableJoint 재연결)
3. _kill_infra 안 함

| | |
|---|---|
| 효과 | Drop episode 의 reset 30s → 5-8s |
| 위험 | 옛날 ODE 충돌 재발 가능 |
| 작업량 | 작음 |
| **예상 fps** | **8-11 (+30%)** |
| 검증 상태 | 미검증 |

### 🟢 D. obs_wait_timeout 줄임

```yaml
obs_wait_timeout: 0.02 → 0.01
```

| | |
|---|---|
| 효과 | 10ms × ep_len 112 = 1.1s/episode 절약 (~5% gain) |
| 위험 | stale obs 가능 |
| **예상 fps** | **+5-10% (6 → 7)** |
| 검증 상태 | 미검증 |

### 🟡 E. Multi-env num_envs=2

```yaml
num_envs: 1 → 2
```

| | |
|---|---|
| 효과 | 2× theoretical, 실제 1.5-1.8× |
| 위험 | PX4 multi-instance airframe sync 이슈 (메모리 있음) |
| 메모리 | 컨테이너 +3GB |
| 작업량 | 중간 (검증 필요) |
| **예상 fps** | **9-11** |
| 검증 상태 | 미검증 |

## 4. 검증 원칙

**한 번에 한 처방만 격리 실험** (Phase 2 plan 의 단일 변수 원칙):

1. Dry run (5k step): 학습 정상 진행 + crash 없음 확인
2. 짧은 run (50k step): fps 측정 + 학습 품질 비교
3. 베이스라인 (현재 v3) 과 정량 비교

다중 조합 사전 금지 — 효과 / 부작용 분리 불가능.

## 5. 권고 진행 순서 (저위험 → 고위험)

1. **D (obs_wait 0.01)**: 가장 단순, 작은 효과 검증
2. **C (drop reset 대안)**: 중간, 큰 효과
3. **B (Takeoff 생략)**: 중간, 큰 효과
4. **E (Multi-env)**: 복잡, 큰 효과
5. **A (Persistent infra)**: 가장 복잡, 가장 큰 효과

각 단계 후 학습 품질도 함께 평가.

## 6. 결정 이력

[2026-06-04] Phase 1 redux v1 fps 폭락 (drop 빈도 ↑ → kill_infra 누적)
[2026-06-04] _kill_infra timeout 5s → 2s (Phase 1 redux v2)
[2026-06-05] v3 fps 6 — scale 처방으로 episode 길어졌지만 reset overhead 더 큼
[2026-06-07] root cause 정리 + 처방 후보 5개 분류
[2026-06-07] CCIP gap 처방 (Issue #007) 우선 → fps 처방은 그 이후
[2026-06-07] 격리 실험 원칙 명시 (조합 금지)
[2026-06-07] CCIP gap = 측정 artifact (Issue #023) — SDF fix 후 fps 처방 진행
[2026-06-07] fps phase 별 측정 (12 reset events):
              - fast path 평균 7s, drop reset 38.6s (start_infra 21s + cruise 15s)
              - 진짜 병목 = drop reset 의 start_infra
              - fps 16 (fresh), drop_rate 50% 가정 시 7 (v3/v4 일치)
[2026-06-07] 옵션 C 적용 + ODE crash 안전장치 마련:
              - safe drop path: zero-vel → _gz_reset_poses → settle → attach topic
              - try/except fallback → _kill_infra (학습 손상 0)
              - 누적 카운터 50 회 후 강제 _kill_infra (Gazebo leak 차단)
              - bridge config 에 /payload/attach_cmd 추가
              - drone_drop_env.py 의 attach_pub publisher 추가

## 7b. 옵션 C 검증 후 followup (예정)

옵션 C 가 실사용에서 ODE crash 없이 fps 개선 입증되면 추가 안전장치 검토:

### A. drone vel 임계 기반 추가 wait
  - 현재: 고정 150ms zero-velocity command
  - 한계: PX4 EKF latency 로 완전 0 안 됨. 임계 (예: 2 m/s) 이상이면 추가 wait
  - 구현: zero-vel 후 vel_enu 측정 → 임계 초과 시 더 보냄
  - 비용: 작음 (drone 안정화 0.3-0.5s 추가 가능)

### C. payload attach 검증
  - 현재: attach topic publish 후 300ms wait (joint 형성 가정)
  - 한계: attach 실패 시 payload 가 미부착 상태로 다음 ep 진입 → drop 학습 불가
  - 구현: reset 후 payload odom z 확인 → drone 위치 ±0.2m 안이면 부착됨, 아니면 fallback
  - 비용: 작음 (확인 0.1s)

### 시점 결정
- 옵션 C 학습 완주 후 → safe drop path 의 실패 사례 (logger 의 warn) 확인
- 실패 사례 패턴 보고 A 또는 C 우선 결정
- 둘 다 적용 비용 작음 → 같이 적용 검토 가능


## 7c. v6 fps 처방 계획 (2026-06-07 토론)

v5 학습 중 fps 3 관측. 목표 fps 20 으로 단계적 처방 시도 plan.
v5 끝난 후 v6 시작 시 적용.

### 단계적 적용 순서 (안전 → 위험)

| Phase | 처방 | 작업량 | 예상 효과 (누적) | 위험 |
|---|---|---|---|---|
| 1 | **D**: obs_wait_timeout 0.02 → 0.01 | 1줄 | fps 3.5 | 작음 |
| 2 | **E**: num_envs 1 → 4 (multi-env) | 30분 | fps 10-13 (×4 이론, 3x 실측 예상) | SAC + multi-env stability |
| 3 (조건부) | **B**: Takeoff 생략 (drone 을 5m 에 spawn) | 2시간 | fps 18-25 | PX4 EKF 불안정 |
| 4 (보류) | **A**: Persistent infra | 1-2일 | fps 40+ | issue #014 와 비슷한 race condition |

### 검증 절차

```
v5 끝 → v6 base 결정 (fresh or v5 resume)
       → Phase 1 (D) 적용 + 5k dry-run
       → Phase 2 (E) 적용 + 5k dry-run
       → fps 측정
         IF fps ≥ 20: 본 학습 시작
         ELSE: Phase 3 (B) 적용 + 5k dry-run → 본 학습
```

각 phase 검증 통과 기준:
- D: obs 누락 없음, success_rate 진행
- E: 4 instance 동시 정상 학습, replay buffer race 없음, critic_loss 안정
- B: PX4 EKF 안정, 첫 step 정상 pos 보고, takeoff 없는 정책 학습 성공

### v6 의 다른 변경

- target_entropy: −15 → −12 (Plan 1, exploration 강화)
  - 이유: v5 의 ent_coef 0.001 + ent_damping 0.79 → exploration 부족 우려
  - Plan 2/3 (damping 약화) 는 v6 결과 보고 결정
- 호버 처방 (α+β+δ): 그대로
- SDF dimensions=3: 그대로
- Option C safe drop path: 그대로
- w_prediction=0: 그대로

### v6 base 결정 분기 (v5 결과)

```
IF v5 success_rate ≥ 25% AND 증가 중:
  → v5 best checkpoint resume

ELIF v5 success_rate ∈ [15-25%] AND plateau:
  → fresh start (안전), 또는 v5 resume (실험)

ELSE (success < 15% or 발산):
  → fresh start + 진단 (env reward 분석)
```

### 격리 실험 위배 수준

v6 의 변경 → 격리 위배 수준 낮음:
- fps 처방 (Phase 1, 2, 3): reward 신호 무관 → 학습 결과 분석에 영향 X
- target_entropy: entropy metric 추적으로 분리 분석 가능
- base 결정: fresh vs resume — 영향 큼 (분석 어려움)

### 멈출 시점 기준

- 목표 fps 20 도달 → 멈춤
- 도달 못 한 경우 Phase 4 (A) 시도 검토 (위험 큼)

## 7. 연관 문서

- `issue_011_drop_infra_restart_overhead.txt` — 원래 발견 (2026-05-25)
- `local/design/model_history.md` — fps trajectory
- `local/issues/issue_007_ccip_prediction_gap.txt` — CCIP gap (현재 우선)

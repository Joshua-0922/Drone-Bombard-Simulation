---
marp: true
theme: default
paginate: true
header: 'Drone Bombard Simulation — 학습 진척 정리'
footer: '2026-06-07'
---

# Drone Bombard Simulation
## 학습 진척 정리: Round 5 → Phase 1 redux v5

**범위**: 2026-05-31 ~ 2026-06-07 (8 일)

발표자: 한오 / 정리: 2026-06-07

---

## 0. 한 줄로 요약

> "드론이 비행하면서 정확한 위치에 물건을 떨어뜨리도록 강화학습으로 가르치는 프로젝트"

- 학습 방식: **SAC** (Soft Actor-Critic, RL 알고리즘)
- 시뮬레이션: **Gazebo** + **PX4** (실제 드론과 동일한 비행 제어)
- 통신: **ROS2**
- 목표: 떨어뜨린 위치와 target 의 거리 (`drop_error`) 를 0 에 가깝게

---

## 1. 시스템 구조 (한 그림)

```
┌──────────────┐    obs       ┌──────────────┐
│  RL 정책     │ ◀─────────── │  Gazebo 시뮬 │
│  (SAC)       │              │  + PX4 비행  │
│              │ ──action───▶ │  + 드론모델  │
└──────────────┘              └──────────────┘
       ▲                              │
       │                              │
       │       drop_error             │
       └──────────────────────────────┘
              (정책 학습 신호)
```

- 정책 = 신경망 (256x256). obs → action 결정
- 비행 = PX4 가 action 받아 실제 드론처럼 비행 시뮬
- 보상 = 떨어뜨린 위치의 정확도

---

## 2. 학습 사이클

매 학습 라운드의 흐름:

```
문제 발견 (이전 라운드)
   ↓
가설 + 처방 설계
   ↓
코드/설정 적용
   ↓
학습 (수십만 step, 보통 10-25시간)
   ↓
결과 분석 → 다음 라운드
```

학습 1 라운드 = 50k~700k step. 1 step ≈ 50ms. 라운드당 수 시간~수일.

---

## 3. Round 5 (2026-05-31): "발산하는 학습"

### 문제

- 학습 중 한 가지 hyperparameter (`ent_coef`) 가 **6.0+ 까지 폭주**
- ent_coef = 정책의 "탐색 의지" 강도
- 너무 크면 정책이 무작위처럼 행동 → 학습 멈춤

### 원인

- SAC 의 **자동 entropy 조정** 이 의도와 다르게 작동
- bounded action space (tanh squash) 에서 default 가 부적절

### 결과

- 학습 실패 (148k step 에서 발산)

---

## 4. Round 6 (2026-05-31~06-03): "Damping 도입"

### 처방

- `ent_damping` 추가: ent_coef 가 너무 높아지면 강제 억제
- 두 버전 시도:
  - v1: mean damping → 작동 안 함
  - v2: percentile damping → 작동

### 결과

- 학습 안정화 일부 성공
- 하지만 다른 문제: **critic_loss 가 14M 까지 폭주 → 메모리 부족 (OOM)**

---

## 5. Round 7 (2026-06-03~04): "Phase 1 완성"

### 처방 (총 4가지 동시)

1. `target_entropy = -15` — entropy 강제 deterministic
2. Huber loss + critic clip 500 — critic 폭주 방지
3. per-sample damping — entropy 미세 조정
4. Infrastructure 안정화 처방 (Issue #021)

### 결과

- **685k step 완주!** (자연 종료)
- best drop 1.32m, 16 success
- **Phase 1 의 endpoint** 로 백업 보존

---

## 6. 그러나 본질적 한계 발견

학습 환경 평가에서:

- Target 까지의 거리 14.87m 가 **너무 어려움**
- 정책이 안정적인 drop 못함
- 정밀도가 평균 14m

→ **task 자체를 쉽게 만들자**: Phase 1 **redux**

---

## 7. Phase 1 redux (2026-06-04): "더 가까운 target"

### 변경

- Target 거리 14m → **5m** (3배 가까움)
- Random drop 보조 제거 — 정책 스스로 drop 학습 강제

### redux v1, v2, v3 진화

| 버전 | 결과 | 학습 |
|---|---|---|
| **v1** | 89k step, best 0.81m, 799 success | 가까운 target 효과 입증 |
| **v2** | 143k step **실패** | curriculum (5m → 1m) 너무 가팔라 정책이 drop 행동 잃음 |
| **v3** | 254k step | scale 처방 + curriculum 완화 |

---

## 8. redux v3 분석 (2026-06-07): "이상한 발견"

### 문제 발견

v3 의 396 drops 분석:

- 평균 drop 위치 오차: **3.59m**
- CCIP (예측) 와 실제 차이 (gap): **1.73m**
- **gap 이 속도에 비례** (correlation 0.955)

### 가설 1 (잘못된)

> "DetachableJoint 가 분리 순간 payload 의 velocity 를 0 으로 reset 함"

비유: "비행기에서 폭탄 떨어뜨리면 폭탄이 멈춘 채로 그냥 떨어진다"

---

## 8b. 왜 가설 1 이 "압도적" 으로 그럴듯했나

### 논리 흐름

**정상 CCIP (velocity 정상 inherit)**:
```
impact_pos = drop_pos + vel × t_f
drop_error ≈ d_impact (예측 = 실제)
→ gap ≈ 0
```

**Velocity = 0 가설**:
```
impact_pos = drop_pos (그대로 떨어짐)
drop_error = d_xy (drone 의 horizontal 위치)
→ gap = d_xy - d_impact = vel × t_f
```

### 데이터와의 일치

정책은 `d_impact ≤ 2m` 에서 drop trigger:

| 속도 | 가설 예측 (vel × t_f) | 실측 gap |
|---|---|---|
| 0-1 m/s | ~0.5m | 0.75m |
| 1-2 m/s | ~1.5m | 1.50m |
| 2-4 m/s | ~2.5m | 2.42m |

→ **정확히 일치** + correlation 0.955. "이건 결정적 증거" 라고 결론.

---

## 8c. 그러나 — 같은 패턴을 만드는 다른 mechanism

### 수학적 동등성

**SDF dimensions=2 bug 의 경우**:
```
payload odom z = 0 (항상)
drop_calculator 가 분리 즉시 impact 처리
drop_error = drone 의 그 순간 horizontal 위치 = d_xy

→ gap = d_xy - d_impact = vel × t_f   ← Velocity 가설과 동일!
```

### 데이터만 보면 두 가설을 구분 못함

- correlation 0.955 ✓ (둘 다 만족)
- bucket 별 선형 패턴 ✓ (둘 다 만족)
- gap mean 1.73m ✓ (둘 다 만족)

### 당시 놓친 것

> "압도적 통계 증거" 도 **수학적으로 동등한 다른 가설** 이 있으면 단정할 수 없다.

세 가지 점검을 안 함:
1. **다른 mechanism 으로 같은 패턴 가능한가?**
2. **측정값 (drop_error) 자체가 정확한가?**
3. **minimal test 로 mechanism 직접 검증?**

이 점검들이 모두 v4 학습 실패 후에야 진행됨.

---

## 9. redux v4 (2026-06-07 새벽): "잘못된 처방 — 옵션 A"

### 처방 — ApplyLinkWrench

분리 직전 payload 에 **물리적 충격 (impulse force)** 인가:

```
F = mass × velocity / dt = 25 × v  (1 step 동안)
→ 강제로 payload 에 drone 의 속도 전달
```

### 사전 검증

mechanism 테스트: 112.8% 전달 확인 ✓

### 학습 결과 (188 drops)

| 지표 | v3 | v4 (옵션 A) |
|---|---|---|
| drop_error | 3.59m | **3.585m** (변화 없음!) |
| gap | 1.73m | 1.46m (16% 감소) |
| correlation | 0.955 | 0.849 |

→ 효과 거의 없음. **가설 자체가 잘못된 것이 아닐까?**

---

## 10. 가설 재검토 (2026-06-07): "충격적 발견"

### 추가 검증

별도 **minimal test** 진행 (multi-model DetachableJoint):

- detach 후 child velocity **101.5% 보존**
- 즉 DetachableJoint 자체는 정상 작동!

학습 환경에서 직접 측정:

- drone 이 4.58m 에 호버 중
- payload 의 odometry 의 z 값 확인 → **0** ?!
- payload 가 분명 4.5m 에 매달려 있는데 측정값이 0

---

## 11. 진짜 Root Cause: SDF 의 1줄 누락

### 발견

- gz-sim8 의 **OdometryPublisher** 의 `<dimensions>` 옵션 default = **2D mode**
- payload SDF 에 `<dimensions>3</dimensions>` 가 빠져 있었음
- → z 좌표가 항상 0 으로 publish 됨

### 영향

drop_calculator 가 `z ≤ 0.04m` 면 ground impact 로 판정:

- payload odom z = 0 (항상)
- → **분리 즉시 impact 로 잘못 판정**
- → drop_error = drone 의 그 시점 horizontal 위치 = `d_xy`

**과거 모든 분석 (gap 1.73, correlation 0.955) 이 측정 artifact!**

---

## 12. 깨달음

### 잘못된 진단의 흔적

| 무엇 | 진짜 의미 |
|---|---|
| "velocity inheritance 실패" 가설 | **틀림** — DetachableJoint 정상 |
| "옵션 A 16% 개선" | 측정 artifact 안의 noise |
| "v4 280k 학습" | 잘못된 보상 신호 위 학습 — 폐기 |

### 교훈

> **"보이는 통계 패턴" 과 "진짜 mechanism" 의 차이를 항상 의심하라**
>
> minimal test PASS 인데 학습 환경 결과 와 모순 → 학습 환경의 측정 자체를 의심

---

## 13. redux v5 (2026-06-07): "진짜 처방"

### 적용된 변경 (6개 카테고리)

1. **SDF fix**: payload_0~3 의 `<dimensions>3</dimensions>` 추가
2. **옵션 A 완전 폐기**: wrench publisher, ApplyLinkWrench plugin 등 제거
3. **Reward 정리**: `w_prediction = 0` (gap reward 무의미)
4. **호버 처방**: threshold 200→150, penalty -15→-30, 호버 강제 종료
5. **fps 처방 (옵션 C)**: drop 후 infra 재시작 우회 (38s → 5s)
6. **Instrumentation**: reset phase 별 측정

### 검증

dry-run 후 본 학습 시작 — **Fresh start** (이전 모델 안 씀)

---

## 14. v5 결과 (8시간 학습, 119k step)

### 정량 지표

```
total_episodes: 4,625
total_drops:      162
total_success:     26  (≤2m)
total_jackpot:      1  (≤0.3m)
wandb success:    16%
```

### 문제 발견!

drop_error 분포 자세히 보니:

- **50.6% drop 이 정확히 99.00m** (default value!)
- 즉 절반이 invalid drop
- 진짜 success_rate = 26/82 = **31.7%**

---

## 15. 새로운 진단: "호버 drop 의 invalid"

### 패턴

Invalid drops 의 공통점:
- `vel ≈ 0` (호버 상태)
- `d_impact ≈ 1.98` (auto_drop_threshold 직전)
- 정책이 **"호버 → drop"** 학습

### 학습 진행 따라 invalid 증가

```
step 0-5k    : invalid 0%
step 20-50k  : invalid 48%
step 50-80k  : invalid 60%
step 80-120k : invalid 53%
```

→ **정책이 학습될수록 호버 drop 패턴 늘어남 → 학습 신호 망가짐**

---

## 16. 가설 (현재 진단 중)

| 가설 | 의미 |
|---|---|
| **A. Collision** | 호버 시 payload 가 drone body 와 충돌 → 떨어지지 못함 |
| **B. Attach silent fail** | safe drop 의 attach 가 무작위로 실패 → 다음 drop 무동작 |
| **C. Drone downwash** | 호버 propeller 의 하강기류가 payload 를 잡아둠 |

### 다음 단계

- 학습 중단 ✓ (이미 완료)
- 진단 진행 (호버 drop 의 trajectory 직접 측정)
- v6 처방 결정

---

## 17. v6 계획 (가설 진단 후 시작)

### 핵심 목표

- 학습 시간 25h → **4h** (fps 3 → 20)
- exploration 강화 (target_entropy −15 → −12)
- 호버 drop invalid 문제 해결

### fps 처방 단계 적용

| Phase | 처방 | 효과 |
|---|---|---|
| 1 | obs_wait 0.02 → 0.01 | 작음 |
| 2 | **multi-env (4 instance 병렬)** | ×4 |
| 3 (조건부) | takeoff 생략 | +50% |
| 4 (보류) | persistent infra | 최대 |

---

## 18. 진행 흐름 (시간 순)

```
2026-05-31  Round 5-6: SAC 발산 문제
2026-06-03  Round 7: critic 안정 → Phase 1 완성
2026-06-04  Phase 1 redux v1-v3: 가까운 target
2026-06-07  redux v3 분석 → v4 옵션 A 시도 (실패)
2026-06-07  진짜 root cause 발견 (SDF 1줄)
2026-06-07  redux v5 시작 (Fresh, 6 처방 적용)
2026-06-07  v5 8h 학습 후 호버 drop 문제 발견 ← 현재
2026-06-08  v6 시작 예정 (가설 진단 + fps 처방)
```

---

## 19. 핵심 교훈

### 진단

> **숫자가 일관되어 보여도 측정 자체가 잘못될 수 있다.**
> v3 의 correlation 0.955 는 압도적이었지만 root cause 와 무관했음.

> **수학적으로 동등한 다른 가설이 같은 패턴을 만들 수 있다.**
> "Velocity inheritance 실패" 와 "측정 artifact" 가 동일한 gap = vel × t_f 를 만들어냄.
> 한 가설이 데이터와 일치한다고 그 가설이 옳은 것은 아님.

### 처방

> **잘못된 처방이 부분적으로 작동하는 것처럼 보일 수 있다.**
> 옵션 A 의 "16% 개선" 은 사실 noise 안의 변동이었음.

### 디버깅

> **minimal test 가 PASS 인데 학습이 망가지면 측정/통합 layer 를 의심하라.**

### 작업 흐름

> **격리 실험 원칙: 한 번에 한 가지만 변경.**
> v5 는 6 처방 동시 적용 — 분석 어려움을 감수한 trade-off.

---

## 20. 결론 + 질문

### 지금까지

- 학습 안정성 (Round 5-7) → 완료
- task 쉽게 (redux v1-v3) → 완료
- 측정 오류 발견 (v3 분석 → v5 fix) → 완료
- v5 학습 → 호버 drop 문제 발견

### 진행 중

- 호버 drop 의 진짜 원인 진단
- v6 의 처방 설계

### 질문 / 토론

준비해 주신 질문 환영합니다.

---

# 부록 — 더 자세히

다음 슬라이드부터:
- A. 학습 모델 list (전체)
- B. 처방 별 효과 정량
- C. 발견된 문제 list

---

## A. 학습 모델 전체 list

| 라운드 | 시기 | step | 결과 | 상태 |
|---|---|---|---|---|
| Round 5 v1/v2 | 05-31 | 65k/148k | 발산 | 폐기 |
| Round 6 v1/v2 | 05-31~06-03 | 162k/294k | 일부 안정 | 폐기 |
| Round 7 v3 | 06-04 | **685k 완주** | best 1.32m | **백업** |
| redux v1 | 06-04 | 89k | best 0.81m | preempt |
| redux v2 | 06-05 | 143k 실패 | — | 폐기 |
| redux v3 | 06-05~07 | 254k | best 1.51m | analytic baseline |
| redux v4 | 06-07 | 280k | best 1.92m | 폐기 (가설 오류) |
| **redux v5** | **06-07** | **119k 수동** | **best ?? / 26 success** | **kq4aldtv** |

---

## B. 처방의 정량 효과

| 처방 | 측정 효과 |
|---|---|
| **SDF dimensions=3** | drop_error 측정 정상화 (99m default 50%→0%) |
| **옵션 A wrench** | gap 16% 감소 (사실 measurement noise) |
| **옵션 C safe drop** | drop reset 38.6s → 5.6s (85% 감소) |
| **hover truncate δ** | hover_timeout 16% ep 발동 |
| **target_entropy=-15** | ent_coef 1.0 → 0.001 (deterministic) |

---

## C. 발견된 주요 문제 list

| Issue # | 문제 | 상태 |
|---|---|---|
| #007 | CCIP prediction gap | resolved (#023 의 부속) |
| #014 | DetachableJoint timing | resolved |
| #019 | SAC ent_coef 발산 | resolved (Round 7) |
| #021 | Infrastructure crashes | resolved |
| **#022** | **fps 낮음** | **v6 처방 진행 중** |
| **#023** | **payload odom 2D bug** | **resolved (v5)** |

---

## 끝

감사합니다.

**Run ID 참조** (Wandb):
- Round 7 v3: 436xl0bb
- redux v3: TBD
- redux v4: 4hz2y01h
- **redux v5: kq4aldtv** (가장 최근)

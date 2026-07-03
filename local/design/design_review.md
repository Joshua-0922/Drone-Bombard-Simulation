# Design Review — 최종 설계 요약

> 이 문서는 **현재 기준 최종 결정만** 담는다.
> 변경 이유, 이력, 대안 분석은 날짜별 design_review 파일 참조.
>
> 최종 갱신: 2026-05-26 (Round 2 최종: gradient 완만화 + max_steps 800 + 종료 조건 추가 + WandB 정리)
> 상세 이력: [design_review_2026-05-25.md](design_review_2026-05-25.md)

---

## 1. 보상 구조

```
Drop 보상 (Layer 4):
  reward = 30 * exp(-0.15 * d_xy)              (proximity bonus — Round 2)
         + 100 * exp(-0.2 * d_error)           (precision — Round 2)
         + 20 * exp(-0.1 * |d_impact - d_error|) (prediction bonus)
         + 50 (jackpot, d_error ≤ 0.1m)

proximity + precision 거리별 (Round 2):
  d_xy=15m: 3.2 + 5.0  =  8.2
  d_xy=10m: 6.7 + 13.5 = 20.2
  d_xy= 8m: 9.0 + 20.2 = 29.2
  d_xy= 5m: 14.2+ 36.8 = 51.0
  d_xy= 3m: 19.1+ 54.9 = 74.0
  d_xy= 0m: 30  + 100  = 130 (+prediction 20 +jackpot 50 = 200)
```

| 파라미터 | 값 |
|----------|-----|
| drop_attempt_bonus | 30 |
| k_drop_proximity | 0.15 (bonus *= exp(-0.15 * d_xy)) — Round 2: 0.3→0.15 |
| w_drop_base | 100 |
| k2_precision | 0.2 — Round 2: 0.5→0.2 |
| w_prediction | 20 |
| k_prediction | 0.1 |
| r_success_jackpot | 50 |
| success_threshold | 5.0m (total_success_count 기준) |
| jackpot_threshold | 0.1m (total_jackpot_count 기준) |
| penalty_instability | 50 |

```
Per-step 보상 (Layer 2+3):
  R = -0.05 (time)
    - 0.05 * ||omega||²
    - 0.05 * ||Δa||²
    + 1.0 * (d_prev - d_now)       ← Round 2: 0.5→1.0
    + 0.7 * cos(heading) * speed_gate
    + 0.4 * exp(-0.05 * d_impact) * speed_gate
```

---

## 2. Drop 메커니즘

**Auto + Random (manual drop 비활성)**

```
action[4] 무시 (manual drop 비활성 — "즉시 drop" exploit 방지)
random_drop = (step >= 600) and (random() < 0.005)  # 0.5%/step
auto_drop   = d_impact ≤ 3.0m

if (random_drop or auto_drop) and not dropped:
    drop 발동
```

| 파라미터 | 값 |
|----------|-----|
| auto_drop_threshold | 3.0m (타겟 근처에서만 발동) |
| random_drop_start_step | 600 — Round 2: 150→600 (600 step 자유 접근 확보) |
| random_drop_prob | 0.005 (0.5%/step, step 600~800 구간 63% drop 확률) |

agent는 비행만 학습. drop 타이밍은 환경이 제어.
auto_drop: 타겟 근처 정밀 drop. random_drop: 접근 후 다양한 거리에서 drop 경험.
wandb에서 `env/total_drops` / `env/total_auto_drops`로 구분 추적.

---

## 3. 에피소드 종료 조건

| 조건 | 기준 | 페널티 |
|------|------|--------|
| crash (ground) | altitude < 0.5m (무조건) | -50 |
| crash (low alt) | altitude < 3.0m (step > 1) | -50 |
| overspeed | speed > 20 m/s | -30 |
| ang_vel | \|\|omega\|\| > 2.0 rad/s | -30 |
| inverted | \|roll\| or \|pitch\| > 60° | -30 |
| out_of_range | d_xy > 100m | -30 |
| max_altitude | altitude > 25m | -30 — Round 2 신규 |
| stagnation | 200 step 내 d_xy 2m 미감소 (step≥200) | -15 — Round 2 신규 |
| timeout | step ≥ 800 (drop 미발동 시) | -15 — Round 2: 500→800 |

우선순위: crash > overspeed > ang_vel > inverted > out_of_range > max_altitude > stagnation > timeout

---

## 4. SAC 하이퍼파라미터

| 파라미터 | 값 |
|----------|-----|
| learning_rate | 3e-4 |
| buffer_size | 500,000 |
| batch_size | 256 |
| tau | 0.005 |
| gamma | 0.995 |
| learning_starts | 1,000 |
| gradient_steps | 1 |
| net_arch | [256, 256] |
| device | cuda |

---

## 5. 환경 설정

| 파라미터 | 값 |
|----------|-----|
| action_vx_scale | 8.0 m/s |
| action_vy_scale | 5.0 m/s |
| action_vz_scale | 3.0 m/s |
| action_yaw_scale | 1.0 rad/s |
| action_rate_limit | 0.2 (step당 |Δa| 제한) |
| max_steps | 800 — Round 2: 500→800 |
| min_altitude | 3.0m |
| obs_wait_timeout | 0.02s |
| target | (11.0, 10.0) ENU |

---

## 6. Observation / Action

```
Observation (17D):
  [0-2]   pos / 50m
  [3-5]   vel / 15m/s
  [6-8]   ang_vel / π
  [9-11]  vision (synthetic)
  [12]    payload attached
  [13-14] rel_target / 50m
  [15]    d_impact / 50m
  [16]    t_f / 10s

Action (5D):
  [0] vx    ×8 m/s
  [1] vy    ×5 m/s
  [2] vz    ×3 m/s
  [3] yaw   ×1 rad/s
  [4] drop  >0.5 → manual drop
```

---

## 7. Curriculum 원칙

1. **Phase 분리** — Phase 안에서는 환경 고정
2. **Phase 사이 buffer reset** — fresh buffer + model resume
3. **계단식** — 매 step 미세 변화 아닌 구간 고정
4. **5k dry-run** — Phase 전환 시 코드 검증 필수

---

## 8. 학습 실행

- Fresh start (resume 없음, buffer 새로)
- 150k steps
- WANDB_MODE=online
- run_name: round1_hybrid_drop
- Round 1 WandB run ID: ruozrv5x (150k 완주)
  - 결과: 432 drops, best 4.64m, avg 14.02m, success 1건
  - d_xy 최소 평균 11.3m — auto_drop 3m 도달 0건
- **Round 2 파라미터 조율** (학습 예정):
  - w_dist: 0.5→1.0, k2_precision: 0.5→0.2, k_drop_proximity: 0.3→0.15
  - max_steps: 500→800, random_drop_start_step: 150→600
  - 종료 조건 추가: max_altitude 25m, stagnation (200step/2m)
  - WandB metric 22개→8개 정리
  - 목적: gradient 확보 + 접근 시간 확보 + 이탈 조기 차단
- Issue #013+#014 해결: inline SDF + paused start + unpause
- best drop 시 모델 가중치 자동 저장 (auto drop 성공 시만)
- 검증 방법: deterministic evaluate + GUI (action replay 폐기)

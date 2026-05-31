# Design Review — 최종 설계 요약

> 이 문서는 **현재 기준 최종 결정만** 담는다.
> 변경 이유, 이력, 대안 분석은 날짜별 design_review 파일 참조.
>
> 최종 갱신: 2026-05-31 (Round 4 발산 분석 + Hover terminal penalty + Round 5 시작)
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
    + 1.0 * (d_prev - d_now)              ← Round 2: 0.5→1.0
    + 0.7 * cos(heading) * speed_gate     ← Round 5: 0.3→0.7 복원
    + 0.4 * exp(-0.05 * d_impact) * speed_gate
    (Round 4 w_distance_penalty 0.03 제거됨 — SAC 발산 원인)
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
| max_altitude | altitude > 50m | -15 — Round 3 부활 |
| timeout | step ≥ 800 (drop 미발동 시) | -15 — Round 2: 500→800 |
| hover_penalty | max_consecutive_still > 200 step | -15 — Round 5 신규 |

우선순위: crash > overspeed > ang_vel > inverted > out_of_range > max_altitude > timeout

**Hover Penalty (Round 5 신규)** — episode 종료 시 1회 적용:
- 조건: max_consecutive_still > 200 step (속도 < 1 m/s 연속)
- 페널티: -15
- 제외: drop 발생 시 (terminated) — 정밀 hover 정당
- **Per-step density 변화 없음** → SAC 안정성 유지 (Round 4 발산 회피)

**Drop 시점 고도 페널티 (Round 3 수정)** — Sigmoid bounded:
- `penalty = -alt_penalty_max * sigmoid(k * (alt - mid))`
- 파라미터: max=50, mid=30m, k=0.15
- 거리별: 15m→-4.7, 20m→-11.7, 30m→-25, 50m→-47.5, 100m+→-50 (포화)
- Round 2 지수 페널티가 -6.77e9 폭주하여 Sigmoid로 교체 (q13hli0y 학습 실패)

**Max altitude truncate (Round 3, 부활)**:
- alt > 50m → truncate + 페널티 -15
- 비행 중 극단 상승 차단 (sigmoid 페널티 외 추가 방어층)

**Reward Hard Cap (Round 3 신규)**:
- step 끝에 reward 범위 검사: [-200, +300]
- 범위 밖이면 `[WARN]` 출력 + np.clip
- 정상 학습엔 영향 없음. 방어적 안전망.

**제거됨**:
- ~~stagnation~~ (200step/2m progress) — speed_gate가 이미 hover 차단

---

## 4. SAC 하이퍼파라미터

| 파라미터 | 값 |
|----------|-----|
| learning_rate | **1e-4** — Round 3: 3e-4→1e-4 (critic overshoot 완화) |
| buffer_size | 500,000 |
| batch_size | 256 |
| tau | **0.002** — Round 3: 0.005→0.002 (target stability) |
| gamma | 0.995 |
| learning_starts | 1,000 |
| gradient_steps | 1 |
| net_arch | [256, 256] |
| device | cuda |
| **Replay Buffer** | **PER (alpha=0.6, eps=0.1, priority_max=30)** — Round 3 |

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
- Round 1 (ruozrv5x, 150k 완주):
  - 432 drops, best 4.64m, avg 14.02m, success 1건 (0.2%)
- Round 2 첫 시도 (dbi74uif, 150k 완주):
  - 접근 실패 — random_drop_start=150 너무 이름
- Round 2 최종 (z05fx7g9, 150k 완주):
  - 427 drops, best 2.53m, avg 19.09m, success 16건 (3.7% — 16배 증가)
  - Deterministic eval: drop 0건, 모두 crash 종료
  - 발견: Reset 버그 (fix 완료), Post-success regression
- Round 3 첫 시도 (q13hli0y, 30k 중단):
  - 지수 고도 페널티 폭주 → -6.77e+9 outlier → 학습 망가짐
- Round 3 수정 학습 (lidq3ydu, 157k 크래시):
  - 104 drops, best 4.32m, success 8건 (Round 2 대비 2배 속도)
  - 100~125k 최우수 (avg 13.9m, success 3건)
  - PX4 로그 20GB 누적 → Gazebo timeout 크래시
- Round 4 발산 (4j46qwpk, 146k 중단):
  - per-step density 변경 (w_heading↓, distance_penalty 신규) → SAC auto-entropy 발산
  - ent_coef 6.03 폭주, critic_loss 230k+
- **Round 5 학습 중** (sdjytkpv, 300k):
  - Round 4 처방 전면 복원 (w_heading 0.7, distance_penalty 0)
  - 신규 Hover Terminal Penalty: episode 종료 시 -15 (sustained hover만)
  - Per-step density 보존 → SAC 안정성 유지
  - PER + LR/Tau + 4가지 안전장치 + PX4 로깅 비활성 유지
- Issue #013+#014 해결: inline SDF + paused start + unpause
- Issue (Reset 버그) 해결: reset()에서 pos_enu/vel/ang/roll/pitch 명시 초기화
- best drop 시 모델 가중치 자동 저장 (auto drop 성공 시만)
- 검증 방법: deterministic evaluate + GUI (action replay 폐기)

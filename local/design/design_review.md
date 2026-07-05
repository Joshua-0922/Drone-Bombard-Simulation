# Design Review — 최종 설계 요약

> 이 문서는 **v8/v9a 기준 최종 결정** 만 담는다 (RAD 도입 이전, legacy reference).
> 변경 이유, 이력, 대안 분석은 날짜별 design_review 파일 참조.
>
> **🆕 새 framework RAD v1 도입 (2026-06-30)**: [rad_v1_design.md](rad_v1_design.md) — single source of truth. v8/v9a 와 완전 다른 framework (Round/redux 시리즈의 patch 가 아닌 새 framework 의 v1). 이 문서의 §1~§N 은 v8/v9a 의 reward / drop 메커니즘 만 — RAD 의 값은 [rad_v1_design.md](rad_v1_design.md) 참조.
>
> **🔥 RAD v1 Phase 1 v1~v6 종합 (2026-07-05)**: [design_review_2026-07-05.md](design_review_2026-07-05.md) — 6번 시행 이력 + 진짜 원인 확정 (Issue #029: curriculum stage 설계 결함) + stage 재설계 안 (v7 계획).
>
> 최종 갱신: 2026-07-05 (RAD v1 Phase 1 v1~v6 종합 완료)
> 이전 갱신: 2026-06-30 (RAD v1 도입 노트)
> 상세 이력:
>   - [design_review_2026-07-05.md](design_review_2026-07-05.md) — **RAD v1 Phase 1 v1~v6 종합 + 진짜 원인 확정 + stage 재설계 안 (현재 최신)**
>   - [rad_v1_design.md](rad_v1_design.md) — **RAD v1 의 모든 design 결정 (현재 활성)**
>   - [design_review_2026-06-30.md](design_review_2026-06-30.md) — RAD 도입 결정 narrative
>   - [design_review_2026-06-27.md](design_review_2026-06-27.md) — v9a 결과 + 다음 처방 후보
>   - [design_review_2026-05-25.md](design_review_2026-05-25.md) — Round 1 결정
>
> 현재 활성 처방 요약:
>   - **RAD v1 (design 완료, 코드 작업 대기, 2026-06-30)**: 2 정책 hierarchical, obs 14d 상대좌표, spawn yaw 랜덤, z Hann reward, 7 final state 조건 jackpot. v8/v9a 와 완전 다른 framework
>   - Phase 1 redux v8 (96bokgae): success 80.6% (학습 통계), 10 ep dgui 50%, mean 2.0m — **best baseline 확정 (v8/v9a 시리즈 한정)**
>   - v9a (zjexq20k 313k): w_dist 1.5, drop_angaccel 0.5/N=5 — fine-tune 17k → 10 ep dgui 33% (v8 보다 17% ↓)
>   - v9a resume (xzoz52cw 432k): CUDA error 종료, 5 ep dgui 67% — 정책 악화
>   - ang_vel callback fix (Issue #024): PX4 dds_topics uncomment, limit_ang_vel 10
>   - dgui 도구 (local/scripts/evaluate_gui.py)
>
> **사용자 결정 (2026-06-27)**: v8 = best baseline. v9a 처방으로는 사용자 의도 미달성.
> **사용자 결정 (2026-06-30)**: 새 framework RAD v1 design 완료. 코드 작업 후 학습 시작.
> **사용자 결정 (2026-07-05, 대기)**: RAD v1 Phase 1 v1~v6 종합 완료. 진짜 원인 (Issue #029) 확정. Stage 재설계 (v7) 진행 결정 대기.
>
> 다음 처방 후보 (RAD 가 대체 또는 우회): Issue #025 (fine-tune step 부족), #026 (toss 환경), #027 (payload tracking)

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

| 파라미터 | 값 (v9a 활성) | 이력 |
|----------|-----|-----|
| drop_attempt_bonus | 30 | Round 2 |
| k_drop_proximity | **0.4** | Phase 1 redux v3 (0.15→0.4) |
| w_drop_base | 100 | jekyun v2 base |
| k2_precision | 0.2 | Round 2 |
| w_prediction | **0.0** | **v5: 20→0 (SDF fix 후 CCIP gap 의미 없음)** |
| k_prediction | 0.1 | Round 1 |
| r_success_jackpot | 50 | Round 2 |
| success_threshold | **2.0m** | Phase 1 redux v3 (5→2) |
| jackpot_threshold | **0.3m** | Phase 1 redux v2 (0.1→0.3) |
| penalty_instability | 50 | Round 1 |
| invalid_drop_threshold | **95.0** | v8 (50→95) |
| invalid_drop_penalty | **0.0** | **v8 (50→0) — drop 회피 학습 차단** |
| **drop_angaccel_penalty_scale** | **0.5** | **v9a NEW** |
| **drop_angaccel_window_n** | **5** | **v9a NEW** |
| limit_ang_vel | **10.0** | **ang_vel fix (2.0→10.0)** |

```
Per-step 보상 (Layer 2+3):
  R = -0.05 (time)
    - 0.05 * ||omega||²
    - 0.05 * ||Δa||²
    + 1.5 * (d_prev - d_now)              ← v9a: 1.0 → 1.5 (payload distance reward 강화, 처방 2)
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
| auto_drop_threshold | **2.0m** — Phase 1 redux v3: v2 의 1.0 너무 빡빡 → 2.0 (curriculum gap 완화) |
| success_threshold | **2.0m** — Phase 1 redux v3: v2 의 1.0 → 2.0 (일관성) |
| jackpot_threshold | **0.3m** — Phase 1 redux v2: 0.1 → 0.3 (도달 가능 영역) |
| random_drop_start_step | 600 — Round 2: 150→600 (600 step 자유 접근 확보) |
| random_drop_prob | **0.0** — Phase 1 redux: 0.005 → 0 (정책 자체 drop 학습 강제) |
| target_enu (x, y) | **(4, 3)** — Phase 1 redux: (11, 10) → (4, 3), spawn 부터 5m |
| **pos_scale** | **5.0** — Phase 1 redux v3: 50 → 5 (5m task 에 맞춤, obs 범위 활용) |
| **action_vx_scale** | **3.0 m/s** — Phase 1 redux v3: 8 → 3 (5m 거리 정밀 제어) |
| **action_vy_scale** | **3.0 m/s** — Phase 1 redux v3: 5 → 3 |
| **max_distance** | **20.0m** — Phase 1 redux v3: 100 → 20 (4× 안전 마진) |
| **k_drop_proximity** | **0.4** — Phase 1 redux v3: 0.15 → 0.4 (5m 환경 sharp gradient) |

Phase 1 redux 이전: random_drop 이 학습 안전망. 14m task 설계 그대로.
Phase 1 redux v1: 96% (5m), best 0.809m. random_drop=0 효과 입증.
  그러나 pos_scale=50 등 거리 의존 파라미터 mis-scaled (14m task 설계 유지).
Phase 1 redux v2 (실패): 1m 임계 너무 가팔라서 정책 drop 행동 잃음.
Phase 1 redux v3: scale 처방 + 임계 2m (curriculum gap 완화). 5m task 전용 첫 일관 설계.

auto_drop: 타겟 근처 정밀 drop. (random_drop 비활성화됨)
wandb에서 `env/total_drops` / `env/total_auto_drops`로 추적.

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
| **알고리즘** | **DampedEntropySAC** (Round 6: SAC 서브클래스) |
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
| **ent_damping_threshold** | **5.0** — Round 6 (Soft damping) |
| **ent_coef_hard_cap** | **1.0** — Round 7: 2.0→1.0 (안전망 강화) |
| **target_entropy** | **-15.0** — Round 7: -5→-15 (근본 처방, bounded action) |
| **target_q_clip** | **500.0** — Round 7 v3: critic bootstrap inflation 차단 |
| **critic loss** | **Huber (smooth_l1)** — Round 7 v3: MSE→Huber (outlier gradient saturation) |

**DampedEntropySAC (Round 7 v3 — Phase 1 최종)**:
- **Per-sample damping** (Round 7 v3 갱신, 이전 q95 → element-wise):
  - `damping_per_sample[i] = 5.0 / (5.0 + max(0, log_prob[i] - 15))`
  - element-wise → ent_coef_loss 계산 시 sample 별 weighting
  - 이전 q95 scalar 는 batch 전체 동일 damping → outlier 잡지만 정상 sample 도 약화
  - per-sample 은 outlier 만 강하게 damped, 정상 sample 은 그대로
- **Huber loss + target Q clipping** (Round 7 v3 신규):
  - `target_q_clipped = target_q_values.clamp(-500, 500)`
  - `critic_loss = 0.5 * sum(F.smooth_l1_loss(current_q, target_q_clipped) for ...)`
  - MSE 의 outlier 폭주 (Round 6 v2 critic_loss 14M) 차단
- Hard cap: `log_ent_coef.clamp_(max=log(1.0))` → ent_coef ≤ 1.0
- 새 metric: `train/ent_damping` (모니터링)
- 검증: Round 7 v3 자연 종료까지 critic_loss 35-40 안정, ent_coef cap 회복 (1.0→0.055)

**Checkpoint Callback fix (Issue #020 — 동시 발견)**:
- 이전: 알파벳순 정렬 → '100000' < '75000' → 새 체크포인트 즉시 삭제
- 수정: step 번호 기반 정렬 → 시간순 보존
- 영향: Round 3 lidq3ydu, Round 6 v1 모두 95k 이후 손실

**SuccessReplay 시스템 (Round 6 v2 — best_drops 대체)**:
- 조건: `is_success` (drop_error ≤ 5m) AND `drop_trigger == 'auto'`
  - 이전 best_drops의 "best 갱신 시만" 조건 제거 → 모든 정밀 drop 저장
- 저장 위치: `/workspace/ros2_ws/success_replay/{wandb_run_id}/`
  - host에 bind-mounted (container 비대 X)
  - symlink: `local/success_replay → ../ros2_ws/success_replay`
- 파일명: `success_step{N}_err{X.XX}m.zip`
- run_id별 영구 보관 — 라운드 재시작 영향 X
- 용도: 나중에 GUI로 그 모델 deterministic evaluate 가능
- **다음 학습부터 적용** (현재 학습은 영향 X)

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
- Round 4 발산 (4j46qwpk, 146k): per-step 처방 → ent_coef 6.03
- Round 5 발산 (sdjytkpv→mnlr1zpe, 148k): terminal 처방 → ent_coef 6.16
- 결론: 처방 무관, SAC + PER + sparse reward 본질적 발산 모드 (Issue #019)
- Round 6 v1 (bfv4la9a, 162k 중단): mean damping 안 잡힘
- Round 6 v2 (6b8bslmz, 294k OOM):
  - percentile damping 작동, 95~195k 학습 성공 (160k에 best 4.36m)
  - 그러나 hard cap 2.0 갇혀 학습 망가짐 → critic 14M → OOM
- Round 7 1차 (iobwvcrm, 14.9k crash): target_entropy=-15 검증, #021 gz timeout 첫 노출
- Round 7 진단 (9qocfk9y, 39k 수동): SuccessReplay 첫 작동, 2 successes
- Round 7 resilient v1 (y6mxu5q2, 102k 중단): 1·2차 처방 적용 (#021)
- Round 7 v2 (dx5fmck6, 385k 중단): per-sample damping + critic 폭주 발견
- **Round 7 v3 (436xl0bb, 685k 자연 종료)** — Phase 1 최종:
  - Huber + target_q_clip=500 + per-sample damping 모두 적용
  - 6055 episodes, 162 drops, 87 auto, **16 successes**, best **1.32m**
  - critic_loss 안정 (35-40), ent_coef cap 회복 (1.0 → 0.055)
  - Forced restart 29회 정상, #021 0회
  - 모든 처방 효과 입증 → **Phase 1 마감**
  - 백업: `local/backups/phase1_final_round7_v3/` (609 MB)
- Phase 1 eval 진단 (1.32m + milestone_600000):
  - deterministic eval 5 EP: 0-1 drops, 정책 자체 drop 능력 약함
  - random_drop 보조 없이는 reliable auto_drop 못함
  - 14.87m 거리는 학습 난이도 높음
- **Phase 1 redux v1** (ayi27a56, 89k 수동 중단):
  - target_enu (11, 10) → (4, 3) — spawn 부터 5m
  - random_drop_prob 0.005 → 0 — 정책 자체 drop 학습 강제
  - `_kill_episode` timeout 5s → 2s
  - drop_calculator x_marker_x hardcoded 버그 fix
  - 결과: 1,205 episodes / 830 drops / **799 successes (96.3%)**
  - **best drop 0.809m** (Round 7 v3 의 1.32m 갱신)
  - 정책 매우 deterministic (ent_coef 0.001)
  - fps 18→2 (drop 빈번 → _kill_infra 5s timeout 누적)
- **Phase 1 redux v2 실패** (za9zxdh6, 143k 수동 중단):
  - auto_drop_threshold/success 3.0/5.0 → 1.0/1.0
  - curriculum learning gap 너무 컸음 (5m → 1m, 4× 정밀화)
  - step 123k 이후 17k+ step 동안 0 drops (정책 drop 행동 잃음)
  - pos_scale=50 등 거리 의존 파라미터 mis-scaled 발견
  - 보존: archive/phase1_redux_v2_failed_143k/ (case study)
- **Phase 1 redux v3 진행 중** (TBD, fresh start, scale 처방):
  - **Scale 처방**: pos_scale 50→5, action_vx 8→3, action_vy 5→3, max_distance 100→20, k_drop_proximity 0.15→0.4
  - **임계 완화**: auto/success 1.0 → 2.0 (curriculum 점진적)
  - jackpot 0.3 유지
  - WandB metric 추가: env/d_xy_outlier_ratio, env/success_rate, env/current_success_streak
  - 제거: env/total_truncate_*
  - 도구: RepresentativeBestCallback (실시간 manifest), drop_trigger column
  - 초기 결과: ep_len_mean 351 (v2 의 15배), ep_rew_mean +168 (양수)
  - 목표: fps 15-25, 2m success > 50%, 첫 jackpot, representative top 3 첫 의미 측정
- Issue #013+#014 해결: inline SDF + paused start + unpause
- Issue (Reset 버그) 해결: reset()에서 pos_enu/vel/ang/roll/pitch 명시 초기화
- best drop 시 모델 가중치 자동 저장 (auto drop 성공 시만)
- 검증 방법: deterministic evaluate + GUI (action replay 폐기)

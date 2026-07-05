# Model History — 학습 라운드 전체 정리

> 지금까지 진행한 모든 학습 라운드의 식별/도입동기/변경/결과/폐기이유 정리.
> "Round 6 이 뭐였더라?" 식의 혼란 방지 + 의사결정 흐름 추적용.
>
> **최신 라운드가 가장 위.** 스크롤 내릴수록 옛 라운드.
> 새 학습 추가 시 위에 prepend 할 것.
>
> 각 model 마다 **이전 한계 → 도입 목적 → 변경 → 결과 → 다음 model 로 교체 이유** 의 흐름으로 narrative.

작성일: 2026-06-05
최종 갱신: 2026-07-05 (RAD v1 Phase 1 v1~v6 시행 이력 추가)

---

## Representative Best Drop — 학습 평가 기준 (2026-06-05 도입)

기존 "best drop = min(drop_error)" 의 한계: 단일 lucky shot 가능성.

**새 정의** — 학습 종료 후 사후 산출:
```
peak_episode  = LAST episode E where rolling_100_success_rate(E) == max
peak_window   = episodes [E-99, E]
representative_top_3 = sorted(auto+success drops in window by drop_error)[:3]
```

**규칙**:
- "auto drop 만" 카운트 (random/manual 제외)
- drop 갯수 < 3 시 → "not_measurable" (skip)
- tie 시 LAST peak

**저장 위치**: `success_replay/{run_id}/REPRESENTATIVE_BEST.json`
**분석 도구**: `local/tools/representative_best_analysis.py` (사후) +
              `RepresentativeBestCallback` (실시간, train_sac.py 내장)

이 metric 이 representative 한 이유:
- success_rate peak 시점 = 정책이 가장 안정적으로 성공하는 상태
- 그 윈도우 내 best 는 우연이 아닌 정책 능력 반영
- auto 만 필터 → random_drop 노이즈 제거

---

## 한 눈에 보기 (최신순)

| 라운드 | run_id | 시기 | step | 결과 (best/success) | 상태 | 핵심 변경 |
|---|---|---|---|---|---|---|
| **RAD v1 Phase 1 v6** | TBD | 2026-07-04~05 | 62k+ (중단 결정 대기) | Stage 3 완전 실패 (0%), cycle 반복 | 🟡 **진짜 원인 확정: curriculum stage 설계 결함 (Issue #029)** | spawn_yaw ±45°, penalty_hover -30, initial pos log 신규 |
| RAD v1 Phase 1 v5 | TBD | 2026-07-04 | 100k+ 수동 | Stage 2 hover 65% 실패 (cycle 반복) | 폐기 | stage2_close 완화 (radius 5→5.5m 등) |
| RAD v1 Phase 1 v4 | TBD | 2026-07-03 | 250k 수동 | Stage 1↔2 cycle 반복 | 폐기 | reward magnitude 축소 (crash -20, hover -10, trunc -5, q_clip 200, fast_reset 500) |
| RAD v1 Phase 1 v3 | TBD | 2026-07-03 | 44k 수동 | Stage 1 정체 (75%) | 폐기 | 5-stage curriculum + regression 도입, advance 강화 |
| RAD v1 Phase 1 v2 | TBD | 2026-07-02 | 171k → NaN abort | NaN abort at 171k | 폐기 | Self-healing 도입 (A1 grad clip + B3 weight rollback + 상세 metric) |
| RAD v1 Phase 1 v1 | g8mvzniw | 2026-06-30~07-02 | 62k → NaN abort | NaN abort at 62k | 폐기 | Original RAD v1 design 그대로 |
| **RAD v1 (design)** | — | 2026-06-30 | 0 (design 완료) | TBD | 🔵 **신규 framework 설계 완료, 코드 작업 대기** | **Relative + Approach (Phase 1) + Drop (Phase 2). 2 정책 hierarchical, obs 14d 상대좌표 (yaw-only body frame), spawn yaw 랜덤 ±90°, cruise 1m/s 가속, target (4,3,0), switch sphere d²≤20.5, z Hann reward, 7 final state 조건 jackpot. v8/v9a 와 완전 다른 framework — Round/redux 시리즈의 patch 가 아닌 새 framework 의 v1** |
| **v9a 313k (10 ep 재평가)** | zjexq20k | 2026-06-27 | 313k (재평가) | **3/9 = 33%, mean 2.006m** | 10 ep stat | 5 ep 80% 는 표본 운 — 실제 33% (v8 보다 17% ↓) |
| **v9a resume 432k** | xzoz52cw | 2026-06-26~27 | 432k (+119k CUDA error) | 2/3 = 67% (5 ep), 2 hover_timeout | 폐기 | CUDA error 로 SIGKILL. 추가 학습이 정책 악화 |
| **Phase 1 redux v9a** | **zjexq20k** | **2026-06-26~27** | **313k (+17k) SIGTERM** | **1.65m / 80% (5 ep eval)** | **fine-tune** | **w_dist 1.5, drop_angaccel 0.5/N=5 (ang_vel fix 후)** |
| **v8 10 ep 재평가** | 96bokgae | 2026-06-27 | 217k (재평가) | **5/10 = 50%, mean 2.002m** | best baseline 확정 | 5 ep 100% 는 표본 운 — 실제 50% (mean 2.0m, threshold 경계) |
| ang_vel callback fix | — | 2026-06-22 | — | v8 모델 영향 미미 | milestone | PX4 dds_topics.yaml uncomment + rebuild + limit_ang_vel 10 (Issue #024) |
| **Phase 1 redux v8** | **96bokgae** | **2026-06-19~21** | **303k 완성** | **0.07m / 80.6% (13 jackpot)** | **🥇 Phase 1 redux 의 진짜 baseline** | **invalid_drop_penalty 50→0, threshold 50→95. toss 전략 발견** |
| Phase 1 redux v7 | TBD | 2026-06-15 | ~37k 실패 | drop 0 | 폐기 | safe attach (옵션 C). invalid_drop_penalty 50 이 drop 회피 학습 강제 |
| Phase 1 redux v6 (실제) | TBD | 2026-06-11 | 300k | 1.X / 11% | 결과 보존 | drop_wait_timeout 10→3 (#022), hover_drop_block 폐기 (0 비활성) |
| Phase 1 redux v6 (계획) | TBD | 2026-06-07 | v5 후 | 계획 | 🔘 plan (이전) | fps 처방 (D+E[+B]) + target_entropy −12 |
| **Phase 1 redux v5** | **TBD** | **2026-06-07** | **진행 중 (~32%)** | **TBD** | **🔄 진행 중** | **SDF dimensions=3 fix (root cause)** |
| Phase 1 redux v4 | 4hz2y01h | 2026-06-07 | 280k 수동 | 1.92m / 1건 | 폐기 | Option A — 잘못된 가설 (issue #023) |
| Phase 1 redux v3 | TBD | 2026-06-05 | 254k 수동 | 1.51m / 5건 | analytic baseline | scale 처방 + 임계 완화 + fresh |
| Phase 1 redux v2 | za9zxdh6 | 2026-06-05 | 143k 실패 | 0.93m / 3건 | 폐기 (case study) | curriculum gap 너무 컸음 |
| Phase 1 redux v1 | ayi27a56 | 2026-06-04 | 89k 수동 | 0.809m / 799건 | preempt 유지 | target (4,3) + random_drop=0 |
| **Round 7 v3** | **436xl0bb** | **2026-06-04** | **685k 완주** | **1.32m / 16건** | **백업 유지** | **Phase 1 endpoint** |
| Round 7 v2 | dx5fmck6 | 2026-06-03 | 385k 수동 | 1.85m / 12건 | 폐기 | critic 폭주 발견 |
| Round 7 res v1 | 3vj4ydx1 | 2026-06-03 | 102k 수동 | — | 폐기 | v2 로 이관 |
| Round 7 res v1 | y6mxu5q2 | 2026-06-03 | 시작 실패 | — | 폐기 | resume bug |
| Round 7 진단 | 9qocfk9y | 2026-06-03 | 39k 수동 | 3.63m / 2건 | 폐기 | 진단 instrumentation 추가 |
| Round 7 1차 | iobwvcrm | 2026-06-02 | 14.9k crash | — | 폐기 | #021 세번째 발생 |
| Round 6 v2 | 6b8bslmz | 2026-05-31 | 294k 중단 | 4.36m / 4건 | 폐기 | #021 두번째 발생 |
| Round 6 v1 | bfv4la9a | 2026-05-31 | 165k 중단 | — | 폐기 | mean damping 작동 안함 |
| Round 5 v2 | mnlr1zpe | 2026-05-31 | 156k | — | 폐기 | ent_coef 6.16 발산 |
| Round 5 v1 | sdjytkpv | 2026-05-31 | 65k | — | 폐기 | terminal 처방 실패 |
| Round 4 v2 | 4j46qwpk | 2026-05-31 | 146k | — | 폐기 | ent_coef 6.0+ 발산 |
| Round 4 v1 | vo1l9wl6 | 2026-05-31 | 14k | — | 폐기 | per-step 처방 실패 |
| Round 3 v2 | lidq3ydu | 2026-05-30 | 157k crash | 4.32m / 8건 | 폐기 | #021 첫 발생 (당시 미진단) |
| Round 3 v1 | q13hli0y | 2026-05-30 | 30k 폭주 | — | 폐기 | 지수 페널티 발산 |
| Round 2 | z05fx7g9 | 2026-05-30 | 150k | 2.53m / 16건 | 폐기 | reward shaping |
| Round 1 | ruozrv5x | 2026-05-26 | 150k | 4.64m / 1건 | 폐기 | 첫 베이스라인 |

---

## 라운드별 상세 (최신순)

### 🟡 RAD v1 Phase 1 v6 — spawn_yaw ±45° + hover -30 + initial pos log (2026-07-04~05, 진행 중)

**한 줄**: v4/v5 의 stage2 stuck 을 hover_penalty -30 강화로 돌파 → **stage3 진입 성공**. But stage3 완전 실패 (성공률 0%) 재확인. Initial pos log 로 **진짜 원인 확정** — curriculum stage 조건이 spawn 위치 (target 거리 5m) 반영 안 함 (Issue #029).

- **이전 (v5) 한계**: stage2 hover 65%, cycle 반복
- **도입 목적**:
  - Hover local optimum 회피 (hover_penalty -10→-30)
  - 학습 어려움 완화 (spawn_yaw ±90°→±45°)
  - **Fact 확인 도구** — initial pos log 신규 (근본 진단 목적)
- **주요 변경**:
  - `spawn_yaw_relative_range: [-π/2, π/2] → [-π/4, π/4]` (±45°)
  - `penalty_hover: -10 → -30` (crash -20 대비 강 penalty)
  - **신규**: env step() 첫 step 시점 위치/속도/거리 저장 → info → callback rolling stats → WandB (`env/initial_target_dist_{3d,xy}_{mean,min,max}` 등)
- **결과 (62k+ step)**:
  - Stage1 통과 (14k, hover 문제 해결)
  - **Stage2 통과 (28k)** — v4/v5 stuck 지점 돌파. Hover 65% → **1%** (65배 감소)
  - Stage3 완전 실패 (0%) → regression → 재도전 → 재실패 (cycle 반복)
- **결정적 발견 (initial pos log)**:
  - 정책 시작 시점 target xy 거리 = **5.10m** (min 5.00 ~ max 5.23)
  - Spawn (0,0,0) → Target (4,3,0) = 5.0m + cruise 0.15m
  - Stage1 (radius 6m), stage2 (radius 5.5m) 는 정책 시작 위치가 이미 sphere 안 → **trivial**
  - Stage3 (radius 4.53m) 만 진짜 학습 필요 → 정책 능력 부재 → 실패
- **진짜 원인**: Curriculum stage 조건이 정책 시작 위치 반영 안 함 (Issue #029)
- **교훈 확립**:
  - [[feedback_initial_pos_log_first]] — Fact 확인 없이 이론 계산만 진단 시 오추정 반복
  - [[feedback_curriculum_stage_spawn]] — Stage radius 는 spawn 거리 반영 필수
  - [[feedback_hover_penalty_vs_crash]] — Hover ≥ crash penalty (local optimum 회피)
- **상태**: 🟡 중단 결정 대기. v7 = stage 재설계 (radius 4.80→4.65→4.53m gradient) 진행 예정
- **참조**: [design_review_2026-07-05.md](design_review_2026-07-05.md), [../issues/issue_029_curriculum_stage_spawn_ignorance.md](../issues/issue_029_curriculum_stage_spawn_ignorance.md)

---

### ⛔ RAD v1 Phase 1 v5 — stage2_close 완화 (2026-07-04, 폐기, hover 65%)

**한 줄**: v4 의 stage2 stuck 을 stage 조건 완화로 해결 시도. But hover 65% 여전 → 문제는 stage 조건 아닌 정책의 hover local optimum 임을 확인.

- **이전 (v4) 한계**: stage1↔2 cycle 반복 (advance/regression 왕복만)
- **도입 목적**: stage2_close 완화로 정책 학습 부담 감소
- **변경**:
  - `stage2_close` config:
    - switch_d² 25 → 30.25 (radius 5.0 → 5.5m)
    - z_min 0.15 → 0.08
    - max_dist 40 → 50
    - limit_tilt 0.9 → 1.0
- **결과 (100k+ step 수동 중단)**:
  - 여전 stage2 hover_timeout 65% 로 stuck
  - Advance/Regression 반복 (2회씩 발생)
  - Cycle 지속 시간만 미미하게 증가 (13.9k → 19.1k)
- **폐기 이유**: 조건 완화가 문제 아님. 정책이 sphere entry 시도 자체 안 함 → hover 로 안전 실패 선호. hover_penalty magnitude 조정 필요 (v6 에서 -30 강화로 해결).
- **참조**: v4 관찰이 하도 반복돼서 stage 자체 문제로 가정한 오판. Fact 확인 없이 접근.

---

### ⛔ RAD v1 Phase 1 v4 — reward magnitude 축소 + q_clip 200 (2026-07-03, 폐기, cycle 반복)

**한 줄**: v3 의 stage1 정체 원인을 Q dynamics 발산 압력으로 진단하고 reward magnitude + Q clip 축소. Stage1 통과 성공 but stage2 에서 stuck (hover 65%).

- **이전 (v3) 한계**: Stage 1 정체 (75%), 발산 방향 (actor_loss +141, q_target_std 289)
- **도입 목적**: Q bootstrap 발산 압력 축소 → 정책 학습 안정화
- **변경**:
  - `penalty_crash: -50 → -20`
  - `penalty_hover: -30 → -10`
  - `truncation_penalty: -15 → -5`
  - `target_q_clip: 500 → 200`
  - `max_consecutive_fast_resets: 50 → 500` (Phase 1 은 drop 없음 → fast reset 정상)
- **결과 (250k 수동 중단)**:
  - Q_target_std 289 → 166 (43% 축소) ← 명백한 개선
  - Q_clip_ratio 58% → 41% (bootstrap 완화)
  - Stage1 통과 성공 (23,945 step)
  - Stage2 진입 후 hover 65% 로 실패
  - Advance-Regress 사이클 2회 (23k → 37k → 91k → 102k)
- **폐기 이유**: Stage2 통과 못 함. Reward magnitude 축소는 필요했지만 hover_penalty 축소가 오히려 정책 hover 선호 유도 (local optimum). hover_penalty 재강화 필요 (v6 -30 으로 확립).
- **파생 finding**:
  - Reward magnitude 축소는 학습 안정성에 효과적
  - fast_reset threshold 500 확대는 phase 1 특성상 무해
  - Hover penalty 는 crash 대비 강해야 함 (v6 에서 확립)

---

### ⛔ RAD v1 Phase 1 v3 — 5-stage curriculum + regression 도입 (2026-07-03, 폐기, stage1 정체)

**한 줄**: v2 의 NaN abort 대책이 self-healing 만으로 부족 → curriculum learning + 자동 재도전 매커니즘 도입. But Stage 1 자체에서 학습 정체 (75%).

- **이전 (v2) 한계**: NaN abort at 171k step (self-healing 발동 안 함), 발산 방향 진행
- **도입 목적**: 쉬운 조건부터 학습 + 실패 시 자동 되돌아가기 매커니즘으로 학습 실패 fault-tolerance 구현
- **주요 변경**:
  - `curriculum_enabled: true`
  - **5-stage** (intro/close/target/partial/full)
  - **Advance 강화**: window 5k → 10k, threshold 0.9 → 0.95, min_stage 2k → 10k
  - **Regression 신규** (v3 도입): window success < 0.3 시 이전 stage 복귀, cooldown 20k
- **결과 (44k 수동 중단)**:
  - Curriculum + regression 매커니즘 정상 작동 (설계대로 발동)
  - Stage 1 (radius 6m) 에서 정책 정체 (75% 성공률)
  - Actor_loss +141 (발산 방향), Q_target_std 299 (매우 큼)
- **폐기 이유**: Curriculum 도입 후에도 발산 방향 유지 → Q dynamics 자체 문제. Reward magnitude 축소 필요 (v4 로 이동).
- **인프라 성과 (재사용)**:
  - Curriculum 시스템 (5-stage, window, threshold, min_stage)
  - Regression 시스템 (threshold, min_steps, cooldown)
  - v4~v6 계속 사용

---

### ⛔ RAD v1 Phase 1 v2 — Self-healing SAC (A1 + B3) 도입 (2026-07-02, 폐기, NaN 171k)

**한 줄**: v1 의 NaN abort 재현 방지 위해 gradient clip + weight NaN rollback 도입. But 171k 에서 NaN 재현 — self-healing 은 학습 dynamics fix 못 함 확인.

- **이전 (v1) 한계**: NaN abort at 62k step, 원인 여러 가설 (H1/H3/H5) 확답 못 함
- **도입 목적**: NaN 이 어디서든 예방 or 복구되도록 안전장치 구축
- **주요 변경** (SAC 확장):
  - **A1 — Gradient Clipping**: `clip_grad_norm_(actor, 20.0)`, `clip_grad_norm_(critic, 200.0)` — optimizer.step() 직전 발동
  - **B3 — Weight NaN Rollback**: 매 gradient step 시작 시 state_dict clone → step 후 NaN 검사 → NaN 감지 시 snapshot 복원
  - **상세 metric 대량 추가**: `train/q_{target,current,pi}_{max,mean,min,std}`, `actor_loss_{q,ent}_term`, `log_prob_{mean,std}`, `reward_batch_{mean,max,min}`, actor/critic `weight_norm`/`grad_norm`, `nan_rollback_count`, `actor/critic_grad_clip_hit`
- **결과 (171k 후 NaN abort)**:
  - NaN abort at 171k step (v1 대비 3배 이상 도달 but 결국 실패)
  - nan_rollback_count = 0 (rollback 발동 안 함)
  - NaN 은 rollout action sampling 시 발생 → train() 안 검사 못 잡음
  - Actor gradient 발산 signal 관찰 (actor_loss +297, q_target_std 289 등)
- **폐기 이유**: Self-healing 은 NaN 방지 완전하지 않고, 학습 dynamics 자체가 발산 방향. 근본 원인 (Q bootstrap 발산 압력) 대응 필요 → v3 로 이동.
- **인프라 성과 (재사용)**:
  - Self-healing SAC (v3~v6 계속 사용)
  - 상세 metric (Q 값, weight/grad norm, log_prob, reward batch) — 근본 진단 도구

---

### ⛔ RAD v1 Phase 1 v1 — Original design 학습 (2026-06-30~07-02, 폐기, NaN 62k)

**한 줄**: RAD v1 설계 완료 후 첫 학습 시도. 62k step 에서 NaN abort — 신규 framework 의 첫 실패, self-healing 필요 확인.

- **이전 상태**: RAD v1 design 완료 (2026-06-30)
- **도입 목적**: RAD v1 설계 검증
- **주요 설정**: Original RAD v1 design 그대로 (relative obs, spawn yaw ±90°, cruise 1m/s, sphere entry + 7 conditions, z Hann reward 등)
- **결과 (62k / 552 ep abort, run g8mvzniw)**:
  - 62k step 부근 NaN actor abort 발생 (`reset() recursively > 2 times` 로 학습 crash)
  - 원인 진단 시도 (H1 task variance / H3 reward magnitude / H5 numerical overflow) — 확답 못 함
  - Q_target_min -900+, target_q_clip=500 초과 지속 관찰
- **폐기 이유**: NaN abort 로 학습 자체 crash. 안전장치 필요 → v2 로 이동 (self-healing 도입).
- **참조**: [../issues/issue_028_rad_ekf_stale_after_reset.md](../issues/issue_028_rad_ekf_stale_after_reset.md) (진단 방향 정정 이력)

---

### 🔵 RAD v1 — Relative Approach Drop (design 완료, 2026-06-30)

**한 줄**: 기존 Round/redux 시리즈 (single SAC, 절대좌표) 와 완전히 다른 framework. 2 정책 hierarchical (Phase 1 approach + Phase 2 drop), obs 14d 상대좌표 (yaw-only body frame), spawn yaw 랜덤화, z Hann reward, 7 final state 조건 jackpot. v9a 의 catastrophic forgetting 문제와 v8 의 toss 일관 행동 (다양성 부족) 두 가지 모두 해결 의도.

- **이전 framework 의 한계**:
  - v8: 80% success 달성 but 일관된 toss 전략만 학습 (정책 다양성 부족)
  - v9a fine-tune: catastrophic forgetting + 처방 흡수 — 17k step 부족
  - v9a resume 432k: CUDA error + 정책 악화 — fine-tune 옵션 3 위험 노출
  - 10 ep 통계 재평가: v8 50% / v9a 313k 33% — v8 이 best baseline 확정
- **이 framework 의 도입 목적**:
  - 정책 다양성 보존 (상대좌표 + spawn yaw 랜덤)
  - 단계별 학습 (approach 와 drop 분리, 두 정책 독립 학습)
  - catastrophic forgetting 차단 (warm start init only, Phase 1 freeze 별도)
- **핵심 변경 (v8/v9a 대비, 13 항목)**:
  - 정책 구조: single SAC → 2 SAC (Phase 1 + Phase 2)
  - obs: 17d 절대좌표 → 14d 상대좌표 (yaw-only body frame)
  - spawn yaw: 고정 → uniform ±90° relative to drone→target
  - cruise: 자동 이동 → head 방향 1m/s 가속만
  - target: (4,3,5) [임시 + 미사용 z] → (4,3,0) 지면 marker 통합
  - w_dist 거리: 2D → 3D 포함 (z−4 reference)
  - z reward: 없음 → Hann (z=4 max, [0.5, 7.5] 0, w_z=0.3)
  - Phase 1 종료: 단순 drop → sphere 진입 + 7 final state 조건 jackpot (+120)
  - drop trigger: d_impact ≤ 2.0 → ≤ 1.0 (정밀도 강화)
  - w_impact: 0.4 → 1.0 강화
  - Phase 2 sphere 벗어남 crash (신규, d²>22 → −30)
  - Phase 2 per-step penalty 2× 강화 (time/ang_vel/action_smooth)
  - Phase 2 action_rate_limit 0.2 → 0.15
- **신규 항목 (RAD only)**:
  - Phase 1 final state 7 조건 (C1 z∈[3,5], C2 ‖v_xy‖≤4, C3 ‖v_z‖≤2, C4 tilt≤0.26, C5 ‖ω‖≤2, C6 yaw_err≤60°, C7 ‖v_xy‖≥0.3)
  - Phase 1 terminal = floor 20 + Σ(50/7 × satisfied) + jackpot 50 × ∏ satisfied (모두 만족 시 +120)
  - phase2_max_steps = 200
  - success_threshold Phase 2 = 1.0m (d_impact trigger 와 정합)
- **인프라**: I-2 (Phase 1 정책 rollout 매 ep, sphere 도달까지 simulate. rollout step 은 Phase 2 buffer 제외)
- **코드 분리**: 옵션 A — `drone_drop_env_rad.py`, `train_sac_rad.py`, `hyperparams_rad.yaml`, `mission_manager_rad_node.py`, `infra_rad.launch.py`, `episode_rad.launch.py` 신규. v8 코드 그대로 (왔다갔다 학습 가능)
- **학습 목표**:
  - Phase 1: step ≥ 300k AND 50k window success ≥ 90% (7 조건 jackpot 비율)
  - Phase 2: step ≥ 150k AND 50k window success ≥ 90% (drop_error ≤ 1.0m)
- **참조 문서**: [rad_v1_design.md](rad_v1_design.md) — 모든 design 결정의 single source of truth
- **상태**: 🔵 design 완료, 코드 작성 대기. 학습 전 Reset 잔존 속도/EKF 검증 (#129) 필요

---

### 🔄 Phase 1 redux v9a — payload_dist + drop_angaccel fine-tune (2026-06-26~27)

**한 줄**: v8 의 toss 전략 완전 수렴 후, 사용자 의도 (지나치는 현상 해결) 위해 fine-tune. 처방 1 (drop_angaccel penalty) 효과 명확 but 처방 2 (w_dist 1.5) 미미 — 17k step 부족.

- **이전 model (v8) 의 한계**:
  - 80% success 달성 but **toss 전략** (drone 이 marker 지나친 후 pitch back + drop)
  - 사용자 의도 = hover-drop 또는 marker 위 도달 후 drop
  - 환경이 두 행동 (hover-drop vs toss) 구분 못 함 (Issue #026)
- **이 model 의 도입 목적**:
  - drop 시점 drone 의 각속도 변화 제한 (사용자 처방 1)
  - payload trajectory 의 monotonic 거리 감소 보상 (사용자 처방 2)
- **prerequisite**:
  - ang_vel callback fix (Issue #024) — PX4 dds_topics uncomment
- **핵심 변경**:
  - `reward.w_dist`: 1.0 → 1.5 (payload distance reward 강화)
  - `reward.drop_angaccel_penalty_scale`: NEW = 0.5
  - `reward.drop_angaccel_window_n`: NEW = 5 (drop 직전 5 step max ang_accel)
  - `reward.limit_ang_vel`: 2.0 → 10.0 (ang_vel fix 후 false crash 방지)
  - v8_peak warm start + fresh replay buffer
  - run_name: `phase1_redux_v9a_payload_dist_angaccel`
- **결과** (313k SIGTERM, +17k step / 의도 100k 의 17%):
  - 5 ep dgui 평가: success 4/5 = 80% (v8 동일), mean 1.89m, **max ang_vel 2.10 rad/s (v8 2.5 대비 -16% ↓)**
  - drop_angaccel penalty 효과 명확 (시각으로는 미세)
  - w_dist 1.5 효과 미미 (terminal +30 의 5%)
  - 정책 행동: toss 그대로 유지
- **결론** (Issue #025):
  - fine-tune 17k 으로는 본질 행동 변화 부족
  - v8 의 강한 toss prior 가 처방 흡수
  - 권장: 100k step 추가 또는 처방 강화 또는 환경 변경
- **현재 보존**:
  - `eval_models/v9a_preempt_step313k.zip` (3.2 MB)
  - `rl_checkpoints/sac_drop_preempt_replay.pkl` (86 MB)
  - 평가: `local/eval_logs/eval_2026-06-27T00-36-50_v9a_preempt_step313k.json`

---

### 🔧 ang_vel callback fix — PX4 dds_topics uncomment (2026-06-22, milestone)

**한 줄**: v8 학습 전체가 `obs[6:9]` (ang_vel) = 0 으로 진행됨에도 80% success 달성. PX4 의 `vehicle_angular_velocity` 토픽이 PX4 default 에서 disable. v9a 의 drop_angaccel penalty 의 prerequisite.

- **발견**:
  - v9 처방 검증 위해 v8 의 drop_episodes 500 ep sample 의 obs 분석 → 모든 ang_vel = 0
  - 토픽 publisher 0 (subscriber 1 정상)
- **Root cause**:
  - `/opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` 의 두 줄 주석 처리됨
  - PX4 maintainers 의 의도적 disable (quaternion derivative 라 redundant 판단)
- **Fix**:
  - dds_topics.yaml 의 두 줄 uncomment
  - PX4 rebuild (`make px4_sitl_default`)
  - `limit_ang_vel`: 2.0 → 10.0 (실측 max 4.89 rad/s, false crash 방지)
  - install/share sync
- **영향 (v8 모델)**: 5 ep 평가 100% success — 정책 weights 가 그 input 에 무지 (학습 시 항상 0 이었음). 변화 미미.
- **측정값 (toss dynamics)**:
  - spawn: 0.05-0.25 rad/s
  - toss pitch back peak: max 2.1-2.7 rad/s (평균), 단일 max 4.89 rad/s
  - drop 직전: < 0.5 rad/s (drone 자연 안정화)
- **Backup**:
  - container: `/tmp/ang_vel_fix_backup/dds_topics.yaml`
  - host: `local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml`

---

### 🥇 Phase 1 redux v8 — no_invalid_penalty — Phase 1 redux 의 진짜 baseline (2026-06-19~21)

**한 줄**: v7 의 drop 0 (invalid_drop_penalty=50 이 정책의 drop 회피 강제) 원인 제거. **success 80.6%, jackpot 13, mean 1.85m, best 0.07m**. toss 전략 발견 (drone 이 marker 지나친 후 pitch back + drop). Phase 1 redux 의 진짜 baseline.

- **이전 model (v7) 의 한계**: drop 0 (학습 완전 실패)
- **진단**: `invalid_drop_penalty: 50.0` 의 신호가 정책의 drop 회피 학습 강제. v5/v6 의 50% invalid drop 경험 → penalty 누적 → drop = 위험 → 정책이 drop 안 함이 최적.
- **핵심 변경**:
  - `reward.invalid_drop_penalty`: 50.0 → **0.0** (drop 회피 학습 차단)
  - `reward.invalid_drop_threshold`: 50.0 → **95.0** (99m default 만 잡음)
  - D1 처방 = 매 drop 후 `_kill_infra + _start_infra` (38s 매번 fresh PX4)
  - 옵션 C 비활성 (DetachableJoint silent fail 우회 가 D1 의 진짜 해결)
- **결과** (303,801 step, 52 시간 학습):
  - **8,736 episodes / 3,342 drops / 2,694 successes (80.6%)**
  - **13 jackpots (≤ 0.3m)**
  - mean drop_err 1.85m
  - **best drop 0.07m** (drop_0588_step157201)
  - **toss 전략 발견** (모든 ep 일관)
- **사용자 GUI 관찰**:
  > drone 이 marker 지나친 후 멈춤 + pitch back + drop → payload toss → marker 근처
- **사용자 평가**:
  > "이게 나쁜건 아닌데 우리가 막연하게 생각하는 그런 방향은 아니잖아? 이렇게라도 학습이 되었다는 점은 고무적이야"
- **현재 보존**: `local/backups/phase1_redux_v8_2026-06-21/` (8.6 GB)
  - models/ (6.2MB), success_replay/ (8.1GB), drop_episodes/ (28MB), wandb/ (421MB), code_snapshot/ (180KB)
- **eval_models 의 v8 모델 3 개**:
  - `v8_peak_step217040_err0.87m.zip` (peak window best)
  - `v8_best_step157201_err0.07m.zip` (global best drop)
  - `v8_final_step303801.zip` (학습 종료 시점)

---

### ⛔ Phase 1 redux v7 — safe attach (옵션 C) — drop 0 학습 실패 (2026-06-15, 폐기)

**한 줄**: v5/v6 의 invalid drop 50% 의 진짜 원인 (DetachableJoint plugin 의 reattach 후 detach silent fail) 우회 위해 옵션 C 시도. invalid_drop_penalty 50 이 정책 drop 회피 학습 강제 → drop 0.

- **이전 model (v6) 의 한계**: invalid drop 50% 여전. hover_drop_block 처방 효과 없음.
- **이 model 의 도입 목적**:
  - DetachableJoint plugin 의 silent fail 우회 (옵션 C safe drop reset path)
  - fps 회복
- **핵심 변경**:
  - 옵션 C 활성 (`drone_drop_env.py`)
  - `invalid_drop_penalty`: 50.0 유지 (drop_calculator timeout 회피 학습 의도)
- **결과 (재앙)**:
  - **drop 0건** (~37k step)
  - 정책이 drop 행동 자체를 학습 회피
- **원인 분석**:
  - invalid_drop_penalty 50 의 신호가 강력 → 정책이 "drop = 50% invalid (v5/v6 경험)" 학습
  - drop 아예 안 함이 정책 최적 결정
  - 옵션 C 의 reattach 도 silent fail (DetachableJoint plugin 의 본질 문제)
- **교훈**: invalid_drop_penalty 의 정책 영향 분리 → v8 = penalty 0 으로 비활성
- **현재 보존**: 폐기 (자료만)

---

### 🟢 Phase 1 redux v6 — fps 처방 + invalid 진단 (2026-06-11, 결과 보존)

**한 줄**: v5 의 invalid drop 50% 진단 + Issue #022 fps 회복. `drop_wait_timeout: 10 → 3` (정상 1초 자유낙하 충분). `hover_drop_block_threshold` 검토 후 폐기 (0 = 비활성). invalid drop 의 진짜 원인 (DetachableJoint plugin) 미해결.

- **이전 model (v5) 의 한계**: invalid drop 50% 발생, fps 정체
- **이 model 의 도입 목적**:
  - fps 회복 (Issue #022) — drop_wait_timeout 7초 줄임
  - hover drop 가설 검토
- **핵심 변경**:
  - `reward.drop_wait_timeout`: 10.0 → 3.0
  - `reward.hover_drop_block_threshold`: 검토 후 0 (비활성)
- **결과**: success 11% (v5 의 16% 와 variance 안), invalid drop 50% 여전
- **결론**: hover 가설 폐기. 진짜 원인 = DetachableJoint plugin 의 silent fail → v7 옵션 C 시도

---

### 🔘 Phase 1 redux v6 — fps 처방 + entropy plan (계획, 이전 entry)

**한 줄**: v5 의 fps 3 가 학습 시간 25h. v6 의 핵심 목표 = fps 20 (학습 시간 4h) + 약한 exploration 강화.

- **v5 의 한계 (관측 중)**:
  - fps 3 (목표 20 의 15%)
  - drop reset safe path 작동하지만 ep_len 짧아 reset overhead 비율 큼
  - ent_coef 0.001 + ent_damping 0.79 → SAC 가 exploration 늘리려 하지만 damping 이 막음
  - success_rate 추이는 v5 끝까지 봐야 확정
- **v6 의 도입 목적**:
  - 학습 시간 25h → 4h (fps 20)
  - exploration 약간 강화 (target_entropy −15 → −12)
  - 더 많은 실험 가능
- **fps 처방 단계 (issue #022 §7c)**:
  - **Phase 1 (D)**: obs_wait_timeout 0.02 → 0.01 (작음, 안전)
  - **Phase 2 (E)**: num_envs 1 → 4 (multi-env, 가장 큰 효과)
  - **Phase 3 (B, 조건부)**: takeoff 생략 — drone 을 cruise alt 에 직접 spawn
  - **Phase 4 (A, 보류)**: persistent infra (issue #014 risk 재발 가능)
  - 단계적 적용, 각 phase 5k dry-run 검증
- **Entropy 처방**:
  - `target_entropy: -15 → -12` (Plan 1, 안전)
  - exploration 강화 — ent_coef 예상 0.005-0.02 범위
- **유지**:
  - SDF `<dimensions>3</dimensions>` (v5 fix)
  - 옵션 C safe drop path + 안전장치 B (50회 카운터)
  - 호버 처방 (α+β+δ): threshold 150, penalty −30, hover_truncate
  - `w_prediction: 0.0`
- **v6 base 결정 분기** (v5 결과 보고):
  - v5 success ≥ 25% 증가 중 → v5 best resume
  - v5 success 15-25% plateau → fresh 또는 resume (실험)
  - v5 success < 15% 발산 → fresh + 진단
- **상태**: 🔘 **계획**. v5 학습 끝나면 시작.

---

### 🟡 Phase 1 redux v5 — SDF dimensions=3 fix (진짜 root cause)

**한 줄**: payload `OdometryPublisher` 의 `<dimensions>` 미지정 → default 2D → z=0 publish → drop_calculator 가 분리 즉시 impact 처리 → drop_error = d_xy. 이게 v3/v4 의 모든 통계 패턴을 만든 진짜 root cause. issue #023 참조.

- **이전 model (Phase 1 redux v4) 의 진단 정정**:
  - Option A (ApplyLinkWrench) 의 16% gap 감소는 measurement artifact 안의 noise
  - "velocity inheritance failure" 가설 자체가 잘못 (minimal_test mtest2 가 101.5% 보존 입증)
  - 진짜 원인: payload odom 의 z 가 항상 0 → drop_calculator 의 impact 판단 (z ≤ 0.04) 가 분리 즉시 trigger
- **이 model 의 도입 목적**:
  - drop_error 가 진짜 ground impact 측정값이 되도록 SDF 수정
  - CCIP d_impact 와 drop_error 가 진짜로 비교 가능한 신호로
  - 정책이 진짜 reward 신호로 학습 (이전엔 잘못된 measurement 위에서 학습)
- **핵심 변경**:
  - `payload_0~3/model.sdf` 의 `OdometryPublisher` 에 `<dimensions>3</dimensions>` 추가
  - frequency 10Hz → 50Hz (정밀도)
  - Option A 코드 전부 제거 (wrench publisher, EntityWrench imports, bridge config wrench topic)
  - ApplyLinkWrench plugin 제거 (world SDF)
  - `w_prediction` 0 (gap reward 비활성)
  - run_name → phase1_redux_v5_sdf_fix
- **시작점**: **Fresh start** (v3/v4 preempt 안 씀 — 잘못된 reward 위에 학습됐음)
- **사전 검증** (test_v1.py 직접 측정):
  - payload first sample z = 5.33m (이전: 0)
  - 자유낙하 → 1.0s 후 ground 도달
  - velocity transfer ratio 103.1% (거의 100% 보존)
- **목표**:
  - drop_error 가 진짜 ground impact 측정 (v3 의 d_xy artifact 아님)
  - CCIP gap (drop_error - d_impact) ≈ 0
  - 정책이 d_impact 최소화 학습 가능
- **상태**: 🟡 **시작 전** (documentation 정리 후 fresh 학습 시작)

---

### ⛔ Phase 1 redux v4 — Option A (잘못된 진단, 폐기)

**한 줄**: DetachableJoint 가 velocity 보존 못 한다고 가설 → ApplyLinkWrench 로 impulse force 인가. **잘못된 가설이었음** (issue #023 참조).

- **이전 model (Phase 1 redux v3) 의 한계**:
  - 396 drops 분석: CCIP gap mean 1.73m, correlation(speed_xy) = 0.955
  - Root cause: 분리 시 payload velocity = 0 으로 reset (DetachableJoint plugin 결함)
  - 81.6% of drops: drop_error ≈ d_xy_at_trigger (velocity 전달 안 됨)
  - 정책이 정밀화 시도해도 본질적 한계
- **이 model 의 도입 목적**:
  - CCIP 의 예측 = 실제 결과 일치시키기 (velocity 보정 작동)
  - Drop_error 분포 ≈ d_impact 분포로 이동
  - Success_rate 의미 있는 증가 가능성
- **핵심 변경**:
  - x_marker_world.sdf 에 ApplyLinkWrench plugin 추가
  - ros_gz_bridge config 에 /world/x_marker_world/wrench topic
  - env._RLBridgeNode 에 EntityWrench publisher
  - publish_drop 에서 F = m × v / dt = 25 × v 의 impulse force publish
- **시작점**: v3 preempt 254k (resume)
- **사전 검증** (test_v1.py):
  - mission_manager 우회 (fake target → TRACK state)
  - Drone velocity 측정 → wrench publish → payload velocity 측정
  - **Velocity transfer ratio = 112.8%** ← 사실 SDF bug 의 노이즈 안에서의 값
- **실제 결과 (4hz2y01h, 280k 수동)**:
  - 188 drops, drop_error mean **3.585m** (v3 의 3.59m 와 동일)
  - gap mean 1.46m, correlation 0.849 — 16% 만 감소 (= noise)
  - Mechanism 안 통함 — 효과 미미
- **폐기 이유**: 가설 자체가 잘못. 진짜 root cause = SDF 의 `<dimensions>` 누락 (issue #023). v5 가 진짜 fix.
- **상태**: ⛔ 폐기. v4 의 280k checkpoint 도 잘못된 reward 위 학습이라 v5 base 로 안 씀.

---

### Phase 1 redux v3 — Scale 처방 + 임계 완화 (254k 수동)

**한 줄**: v2 실패 원인 (scale 부조화 + 임계 점프 너무 가파름) 정면 대응. 5m task 전용 첫 일관 설계.

- **이전 model (Phase 1 redux v2) 의 한계**:
  - Curriculum gap (5m → 1m, 4× 정밀화) 너무 가팔라 정책 적응 못함
  - step 123k 이후 17k+ step 동안 0 drops — 정책이 drop 행동 잃음
  - pos_scale=50 등 거리 의존 파라미터가 14m task 설계 그대로 → 5m 환경에 mis-scaled
  - 결과 success_rate 0.9 가 misleading (ep_info_buffer 의 v1 잔존)
- **이 model 의 도입 목적**:
  - Scale 처방으로 5m task 와 일관된 학습 환경
  - 임계 완화 (1m → 2m) 로 정책 적응 가능한 단계
  - Fresh start 로 mis-scaled 정책 완전 무효화
  - 새 도구들 (RepresentativeBest, drop_trigger column, d_xy_outlier) 정상 작동 첫 검증
- **핵심 변경 — Scale 처방**:
  - `pos_scale: 50 → 5` (obs 정규화, [-1,1] 범위 활용)
  - `action_vx_scale: 8 → 3` m/s (정밀 제어, 8 m/s 면 0.6초 통과)
  - `action_vy_scale: 5 → 3` m/s
  - `max_distance: 100 → 20` m (4× 안전 마진)
  - `k_drop_proximity: 0.15 → 0.4` (sharp gradient)
- **핵심 변경 — 임계 완화**:
  - `auto_drop_threshold: 1.0 → 2.0` m
  - `success_threshold: 1.0 → 2.0` m
  - `jackpot_threshold: 0.3` m 유지
- **WandB metric 변경**:
  - 추가: `env/d_xy_outlier_ratio` (0-6m: 0, 6m+: 1 의 평균)
  - 추가: `env/success_rate` mirror
  - 작동: `env/current_success_streak` (v2 는 옛 코드라 안 됐던 것)
  - 제거: `env/total_truncate_*` 모두
- **시작점**: Fresh start (v2 의 mis-scaled 정책 폐기)
- **목표**:
  - fps 15-25 회복
  - 2m 기준 success_rate > 50%
  - 첫 jackpot 발화 (drop_error < 0.3m)
  - Representative top 3 첫 의미 측정
- **상태**: 🔄 **진행 중**
- **초기 결과** (step 1405):
  - ep_len_mean 351 (v2 의 23 대비 15배 ↑) ← scale 처방 즉각 효과
  - ep_rew_mean +168 (v2 의 -42 와 정반대)
  - 드론 안정 비행 (action_vx 3 m/s 효과)
  - critic_loss 1.53 안정
- **다음 model 로 교체 조건**: 결과 보고 결정

---

### Phase 1 redux v2 — Curriculum 실패 (폐기 case study, 143k 중단)

**한 줄**: v1 의 96% (5m) 정책 위에 1m 정밀화 시도, 그러나 curriculum gap 너무 컸음.

- **이전 model (Phase 1 redux v1) 의 한계**:
  - 96% success 인데 단지 5m 안 명중 — 정밀도 자체는 측정 못 됨
  - jackpot 0.1m 너무 빡빡해서 한 번도 발화 안 됨
  - drop 잦음 → _kill_infra 5s timeout 누적으로 fps 2 폭락
- **이 model 의 도입 목적**:
  - "정말로 정밀하게 떨어뜨리는" 능력 측정
  - Curriculum learning — 96% 정책 (접근 마스터) 위에 정밀화만 추가 학습
  - jackpot 0.3m 으로 첫 발화 가능성 도전
- **핵심 변경**:
  - `auto_drop_threshold 3.0 → 1.0m`
  - `success_threshold 5.0 → 1.0m`
  - `jackpot_threshold 0.1 → 0.3m`
  - `_kill_infra timeout 5s → 2s` (fps 회복)
- **결과** (143k 수동 중단):
  - 10 drops in 50k step (정상 학습의 1/100)
  - **step 123k 이후 17k+ step 동안 0 drops** — 정책이 drop 행동 잃음
  - success_rate metric 0.9 → misleading (ep_info_buffer carryover)
  - fps 8 (예상 잔여 8.8h 효율 낮음)
  - ep_rew_mean -15 ~ -42 (음수, 학습 신호 좋지 않음)
- **실패 분석 (case study 가치)**:
  - Curriculum gap (5m → 1m) 너무 가팔라 정책 적응 못함
  - 또한 pos_scale=50 등 거리 의존 파라미터 mis-scaled 발견 (v3 처방 동기)
  - **"Curriculum learning gap 이 너무 크면 정책이 학습 못함" 의 명확한 증거**
- **다음 model (Phase 1 redux v3) 로 교체 이유**:
  - 임계 완화 (1m → 2m) + scale 처방 함께 적용
  - fresh start 로 깨끗한 baseline
- **현재 보존**:
  - `rl_checkpoints/archive/phase1_redux_v2_failed_143k/`
  - sac_drop_preempt.zip + replay.pkl
  - REPRESENTATIVE_BEST.json (실시간 callback 결과 — not_measurable 예상)
  - NOTE.md (실패 원인 분석)

---

### Phase 1 redux v1 — Target 이동 + random_drop=0 (preempt 보존, 89k 수동)

**한 줄**: Round 7 v3 의 "정책이 random_drop 보조에 의존" 발견에 대응, task 쉽게 + drop 강제로 정책 자체 능력 입증.

- **이전 model (Round 7 v3) 의 한계**:
  - eval 결과 deterministic 모드에서 0-1 drops/5 episodes 만 발생
  - 정책이 random_drop 보조에 의존 (random_drop 발화로 우연한 success 가 학습 신호)
  - 14.87m 거리는 정책에 너무 어려운 task
  - median drop_error 15m (bimodal 분포, 11% 정밀, 89% 빗나감)
- **이 model 의 도입 목적**:
  - **Task 자체를 쉽게** 만들어서 정책이 정말로 학습 가능한지 검증
  - random_drop 보조 없이 **정책이 직접 drop trigger** 할 능력 강제 학습
  - Round 7 v3 의 한계 (random_drop 의존성) 정면 돌파
- **핵심 변경**:
  - **target_enu (11, 10) → (4, 3)** — spawn 부터 14.87m → **5m**
  - **random_drop_prob 0.005 → 0** — 안전망 제거, 정책만 의존
  - drop_calculator x_marker_x hardcoded 11.0 → cfg 사용 (버그 fix)
  - x_marker_world.sdf 의 x_marker_0 pose 도 (4, 3, 0) 변경
  - _kill_episode timeout 5s → 2s (Phase 2 speedup)
- **결과** (89k 수동 중단):
  - 1,205 episodes, **830 drops (모두 auto, random_drop=0 효과 확인)**
  - **799 successes (96.3% — 5m 기준)** ← random_drop 없이도 정책 학습 성공
  - **best drop 0.809m** (Round 7 v3 의 1.32m 갱신)
  - jackpot 0 (0.1m 임계 한계)
  - ent_coef 0.001 (매우 deterministic, 이전 0.055 대비 50배 작음)
  - fps 18 → 2 (drop 빈번 → _kill_infra 5s 누적 = 새 문제 발견)
- **다음 model (Phase 1 redux v2) 로 교체 이유**:
  - **96% success 빠르게 달성** → 다음 단계 (정밀화) 필요
  - jackpot 발화 불가 → threshold 늘림 필요
  - 5m success 임계 헐거움 → 1m 으로 정밀화
  - fps 폭락 원인 (kill_infra) 발견 → 처방 필요
  - 더 학습해도 marginal 이라 판단 → curriculum 으로 진입
- **현재 보존**:
  - `rl_checkpoints/archive/phase1_redux_v1_pause_89k/sac_drop_preempt.zip` (3 MB) — v2 가 여기서 resume
  - `rl_checkpoints/archive/phase1_redux_v1_pause_89k/sac_drop_preempt_replay.pkl` (83 MB)
  - `success_replay/ayi27a56_best5/` (16 MB) — best 5 정책 (0.81m, 2.03m, 2.25m, 2.45m, 3.13m)

---

### 🥇 Round 7 v3 — Phase 1 최종 (Phase 1 endpoint, 685k 자연 종료)

**한 줄**: Round 7 v2 의 critic 폭주 원인 발견 후, Huber + target_q_clip + per-sample damping 으로 SAC 완전 안정화. Phase 1 마감.

- **이전 model (Round 7 v2) 의 한계**:
  - 280k 에서 ent_coef 1.0 cap 다시 도달 (target_entropy=-15 처방에도 불구)
  - **분석 결과**: critic_loss 가 250x 점프 (68 → 17,100) 후 millions 로 폭주
  - critic 폭주 → Q 값 inflation → 정책 행동 비정상 → log_prob 거대화 → alpha 증가 → cap
  - 즉 critic 폭주가 entropy 발산의 **원인** (반대 아님)
- **이 model 의 도입 목적**:
  - critic 안정성 처방으로 root cause 차단
  - SAC entropy 발산을 완전히 해결 (Round 4~6 부터 이어진 문제)
  - Phase 1 마감 + baseline 확립
- **핵심 처방**:
  - **Per-sample damping**: q95 scalar → element-wise (이전 q95 는 batch 전체 동일 damping → outlier 만 damped 가 더 정확)
  - **Huber loss (smooth_l1)**: MSE 의 outlier gradient saturation (큰 TD error 에 대해 gradient 1.0 으로 saturate)
  - **target_q_clip = 500**: bootstrap inflation 차단 (reward 스케일 ±200 고려 ±500 충분)
- **결과** (685k 자연 종료):
  - 6,055 episodes, 162 drops, 87 auto, **16 successes**, best **1.32m** (Round 6 v2 의 4.36m 갱신)
  - critic_loss 35-40 안정 (Round 6 v2 의 14M 대비)
  - ent_coef 1.0 cap → 0.055 회복 (per-sample damping 효과 입증)
  - #021 crash 0회 (1·2차 처방도 검증)
  - Forced restart 29회 정상 작동
- **다음 model (Phase 1 redux) 로 교체 이유**:
  - **SAC + 인프라 처방 모두 완성** → Phase 1 목표 달성
  - 그러나 정책의 **정밀도 한계** 발견 (median 15m)
  - Eval 진단: 정책이 random_drop 보조에 의존, deterministic 모드 약함
  - 정밀도 향상 위해 task 자체 변경 필요 → Phase 1 redux
- **현재 보존**: ✅ **`local/backups/phase1_final_round7_v3/`** (609 MB) — **Phase 1 공식 endpoint**
  - sac_drop_final.zip, success_replay_436xl0bb/ (15 모델), drop_episodes/, wandb run, 코드/yaml snapshot
- **Representative Best 분석** (2026-06-05 추가 — `REPRESENTATIVE_BEST.json`):
  - **결과: `not_measurable`**
  - Peak success_rate = 0.13 at episode 8488 (step 661,673)
  - Peak window [episodes 8388~8488, steps 657591~661673] 안 **auto+success drop = 0건**
  - 저장된 15 success 모델 중 peak window 안 들어가는 것 없음 (가장 가까운 게 step 649,769)
  - **의미**: best 1.32m 은 **isolated lucky auto drop** 이었음을 입증.
    Round 7 v3 의 success_rate 자체는 random_drop 도움으로 올라간 것.
    이 결과가 Phase 1 redux (random_drop=0) 도입의 정당성 검증.
  - 분석 도구: `local/tools/representative_best_analysis.py`

---

### Round 7 v2 — Critic 폭주 발견 (폐기, 385k 중단)

**한 줄**: Round 7 res v1 의 1·2차 처방 위에 추가 학습. critic 폭주의 메커니즘 발견 → v3 처방 도출.

- **이전 model (Round 7 res v1 retry) 의 한계**:
  - 1·2차 처방으로 #021 해결됐지만, 학습 진행할수록 ent_coef 다시 상승 의심
  - 더 길게 학습해서 한계 측정 필요
- **이 model 의 도입 목적**:
  - 1·2차 처방 위에 long-run 안정성 검증
  - target_entropy=-15 가 충분한지 추가 확인
- **변경**: (없음 — 분석용 long run)
- **결과**:
  - 280k+ 에서 다시 **ent_coef 1.0 cap 도달**
  - critic_loss 패턴 분석: 68 → 17,100 (250x 점프) → 결국 millions 폭주
  - 처음으로 **critic 폭주가 ent_coef 발산의 원인**임을 발견
  - 12 successes, best 1.85m
- **다음 model (Round 7 v3) 로 교체 이유**:
  - critic 폭주 원인 식별 → Huber + target_q_clip 처방 필요
  - per-sample damping 도 동시 필요 (q95 만으로는 outlier 못 잡음)
  - 분석용 run 종료, 처방 적용 model 로 이관
- **현재 보존**: ❌

---

### Round 7 res v1 재시도 — 1·2차 처방 (폐기, 102k 수동)

**한 줄**: y6mxu5q2 의 resume path bug fix 후 재시도. 1·2차 처방 정상 작동 확인.

- **이전 model (y6mxu5q2) 의 한계**: resume path bug 로 시작 실패
- **이 model 의 도입 목적**: bug fix 후 1·2차 처방 정상 작동 검증
- **변경**: 코드 수정 (path 처리)
- **결과**: 102k 까지 학습. #021 처방 작동 확인.
- **다음 model (Round 7 v2) 로 교체 이유**: long-run 안정성 추가 검증 필요
- **현재 보존**: ❌

---

### Round 7 res v1 — 1·2차 처방 적용 (resume 실패)

**한 줄**: #021 의 1·2차 처방 첫 적용 시도 — resume bug 로 즉시 실패.

- **이전 model (Round 7 진단) 의 한계**:
  - 진단 instrumentation 까지 완료, 실제 처방 적용 단계 필요
- **이 model 의 도입 목적**:
  - 1차 처방: `_check_infra_healthy` 의 `subprocess.TimeoutExpired` catch
  - 2차 처방: `max_consecutive_fast_resets = 100` (강제 full restart)
- **결과**: resume path bug — `_pickle.UnpicklingError` 로 시작 실패
- **다음 model (3vj4ydx1) 로 교체 이유**: bug fix 후 재시도
- **현재 보존**: ❌

---

### Round 7 진단 — SuccessReplay 도입 (폐기, 39k 수동)

**한 줄**: #021 진단 instrumentation + SuccessReplay 시스템 첫 작동 확인.

- **이전 model (Round 7 1차) 의 한계**:
  - #021 crash 발생했지만 원인 진단 불가 (어떤 가설로 검증할지 모름)
  - 학습 중 success 모델 자동 보존 시스템 없음
- **이 model 의 도입 목적**:
  - InfraHealthMonitor 콜백 (200 step 마다 gz_ms, gz_rss, velocity 로깅)
  - SuccessReplay 시스템 (is_success + auto_drop 시 모델 자동 저장)
  - 다음 #021 발생 시 진단 데이터 자동 수집
- **변경**:
  - 코드 추가: `InfraHealthMonitorCallback`
  - 코드 추가: `DropEpisodeRecorderCallback` 의 success+auto 시 model.save()
- **결과** (39k 수동 중단):
  - **SuccessReplay 첫 작동 확인**: success 2건 저장 (3.63m, 3.82m)
  - 진단 데이터 정상 로깅
- **다음 model (Round 7 res v1) 로 교체 이유**:
  - 진단 + 보존 시스템 검증 완료 → 1·2차 처방 적용 단계
  - 짧은 검증 run 이라 학습 자체엔 의미 작음
- **현재 보존**: ❌

---

### Round 7 1차 — Target_entropy 처방 (폐기, 14.9k crash)

**한 줄**: Round 6 v2 의 cap 도달 분석 → bounded action 환경에서 target_entropy=-5 (default) 가 부적합. -15 로 변경.

- **이전 model (Round 6 v2) 의 한계**:
  - 95k~195k 잘 학습 후 hard cap 2.0 도달 → critic 14M+ 폭주
  - hard cap 자체가 문제가 아니라 default target_entropy 가 원인 의심
- **이 model 의 도입 목적**:
  - **분석 결과**: SAC 자동 entropy 튜닝 의 `d(loss)/d(log_alpha) = -(log_prob + target_entropy)`
  - bounded action (tanh squash) 환경에서 log_prob 에 Jacobian 보정 → 자연스럽게 큰 양수
  - target_entropy=-5 (default) → log_prob > 5 면 alpha ↑ 압력 → 거의 항상 발생
  - 해결: target_entropy = -15 → log_prob > 15 는 극단적 → 대부분 alpha ↓
- **변경**:
  - **target_entropy: -5 (default) → -15** (근본 처방)
  - ent_coef_hard_cap 2.0 → 1.0 (안전망 강화, 학습 가능 수준)
- **결과** (14.9k 에서 crash):
  - **SAC 처방 효과 검증됨**: ent_coef 0.29 안정 (이전 Round 6 v2 의 2.0 cap 갇힘 대비)
  - critic_loss 200-400 정상 범위
  - ep_rew_mean -33 → -25 학습 우상향
  - **종료**: #021 gz timeout (세번째 발생)
- **다음 model (Round 7 진단) 로 교체 이유**:
  - SAC 처방 효과 확인 but #021 인프라 버그 해결 필요
  - 진단 instrumentation 추가 후 처방 적용
- **현재 보존**: ❌

---

### Round 6 v2 — Percentile damping + #021 두번째 (폐기, 294k 중단)

**한 줄**: Round 6 v1 의 mean damping 실패 후 percentile 로 변경. 작동했으나 cap 갇혀서 발산.

- **이전 model (Round 6 v1) 의 한계**:
  - mean() 기반 damping 이 PER outlier 못 잡음
  - damping=1.0 유지하며 ent_coef 발산
- **이 model 의 도입 목적**:
  - mean → 95th percentile 로 변경
  - 상위 5% transition 의 집중도 감지
- **변경**:
  - damping concentration 계산: `mean(log_prob)` → `quantile(log_prob, 0.95)`
  - 95k 체크포인트 (Round 6 v1) 에서 resume
- **결과**:
  - 95k~195k 잘 학습, success 4건, **160k 에 best 4.36m**
  - Hard cap 2.0 갇혀 학습 망가짐 → critic_loss 14M+ 폭주
- **종료 원인 (당시 분석)**: "컨테이너 OOM exit 137" ← **틀린 진단**
- **종료 원인 (실제, 2026-06-03 재분석)**: **#021 gz timeout** (두번째 발생)
- **다음 model (Round 7 1차) 로 교체 이유**:
  - hard cap 만 처방으론 부족 — 근본 원인 (target_entropy) 파악 필요
  - bounded action space 에서 SAC auto-tuning 의 본질적 문제 분석
- **현재 보존**: **`success_replay/round6_v2_recovered/success_step160625_err4.36m.zip`** (3 MB) — 역사적 best 보존

---

### Round 6 v1 — DampedEntropySAC 도입 (폐기, 165k 중단)

**한 줄**: Round 5 의 "처방 형태 무관 발산" 결론에 대응, SAC 자체에 damping + cap 의 안전망 추가.

- **이전 model (Round 5) 의 한계**:
  - per-step 페널티도 terminal 페널티도 모두 ent_coef 6.0+ 발산
  - **SAC + PER + sparse reward 의 본질적 발산 모드** 확인
- **이 model 의 도입 목적**:
  - SAC 자체의 자동 entropy 튜닝에 안전망 도입
  - Soft damping (log_prob 집중도 따른 alpha 증가 둔화) + Hard cap
- **변경**:
  - **DampedEntropySAC 신규 클래스** (SAC 서브클래스)
  - Soft damping: `damping = 5.0 / (5.0 + concentration)`, concentration = mean(log_prob) - target
  - Hard cap: `ent_coef ≤ 2.0`
- **결과**: 162k 까지 학습, mean() 기반 damping 이 PER outlier 못 잡음 (damping=1.0 유지하며 발산)
- **다음 model (Round 6 v2) 로 교체 이유**:
  - mean() 은 PER outlier 의 극단 영향을 평균에 묻혀 못 감지
  - percentile (q95) 로 바꿔서 상위 5% 잡아야
- **현재 보존**: ❌

---

### Round 5 — Terminal 처방 (폐기, 156k 발산)

**한 줄**: Round 4 의 per-step 처방 발산 후, terminal-only 로 변경 시도. 같은 패턴 반복 → 본질적 문제 확정.

- **이전 model (Round 4) 의 한계**:
  - per-step hover 페널티가 ent_coef 6.0+ 발산 유발
  - 가설: per-step 페널티 매 step 마다 정책에 미세 영향 → noise 누적
- **이 model 의 도입 목적**:
  - per-step → terminal-only 로 페널티 적용 시점 변경
  - terminal 만 적용하면 정책에 noise 덜 가지 않을까 가설
- **변경**: hover_terminal_penalty -15 (per-step 페널티 제거)
- **결과**: 156k 까지 학습, **ent_coef 6.16 발산** (같은 패턴)
- **결정적 발견**: 처방 형태 (per-step / terminal) 무관 발산
  → **SAC + PER + sparse reward 의 본질적 발산 모드** (Issue #019 신규)
- **다음 model (Round 6 v1) 로 교체 이유**:
  - 처방 형태로 해결 불가능
  - SAC 자체에 damping + cap 안전망 도입 필요
- **현재 보존**: ❌

---

### Round 4 — Hover 차단 (폐기, 발산)

**한 줄**: Round 3 의 hover exploit (Issue #017) 첫 처방 시도. per-step 페널티 도입했으나 SAC 발산.

- **이전 model (Round 3) 의 한계**:
  - 정책이 타겟 위에 hover 만 하고 drop 안 함 (hover exploit)
- **이 model 의 도입 목적**:
  - hover 행동에 매 step 페널티 부과 → drop 강제
- **변경 (v1 vo1l9wl6)**: per-step hover 페널티 신규
- **결과 (v1)**: 14k 에서 코드 버그 발견 → 중단
- **변경 (v2 4j46qwpk)**: v1 의 버그 fix
- **결과 (v2)**: 146k 까지 학습, **ent_coef 6.03 발산**
- **다음 model (Round 5) 로 교체 이유**:
  - per-step 페널티가 매 step 정책에 영향 → 발산 의심
  - terminal-only 로 변경해서 noise 줄이는 시도
- **현재 보존**: ❌

---

### Round 3 — PER 도입 + #021 첫 발생 (폐기)

**한 줄**: Round 2 의 post-success regression 처방 + PER 도입. 첫 시도 폭주, 두번째는 안정 학습했으나 crash.

- **이전 model (Round 2) 의 한계**:
  - Success 직후 정책이 발산하는 패턴 (post-success regression)
  - 학습 효율도 개선 여지 — PER 도입 검토
- **이 model 의 도입 목적**:
  - **v1 (q13hli0y)**: post-success regression 처방 — 지수 고도 페널티
  - **v2 (lidq3ydu)**: v1 폭주 후 선형 페널티로 수정 + PER + LR + Tau 동시 적용
- **변경 (v1)**: 지수 고도 페널티 도입
- **결과 (v1)**: 30k 에서 reward -6.77e+9 폭주, 학습 망가짐 — 지수의 outlier 폭주
- **변경 (v2)**:
  - 지수 → 선형 시그모이드 페널티
  - **PER (alpha=0.6, eps=0.1, priority_max=30)** 첫 도입 (Round 7 까지 유지)
  - learning_rate 3e-4 → 1e-4 (critic overshoot 완화)
  - tau 0.005 → 0.002 (target stability)
- **결과 (v2)**: 157k 에서 crash, 104 drops, **best 4.32m**, success 8건
- **종료 원인 (당시 분석)**: "PX4 로그 20GB 누적 → Gazebo timeout" ← **틀린 진단**
- **종료 원인 (실제, 2026-06-03 재분석)**: **#021 gz model --list TimeoutExpired** (첫 발생, 당시 미진단)
- **다음 model (Round 4) 로 교체 이유**:
  - hover exploit (Issue #017) 발견 — 정책이 타겟 위 hover 함
  - hover 차단 처방 필요
- **현재 보존**: ❌

---

### Round 2 — Gradient 완만화 (폐기)

**한 줄**: Round 1 의 학습 신호 부족 해결, success bonus 와 random_drop 시점 조정.

- **이전 model (Round 1) 의 한계**:
  - success 단 1건 (0.2%) — 학습 신호 너무 sparse
  - best 4.64m 정확도 매우 낮음
  - Reward gradient 너무 가파름 (k2_precision 5.0) → 정책이 jump
- **이 model 의 도입 목적**:
  - Reward gradient 를 완만하게 만들어 학습 안정화
  - random_drop 시작 시점을 늦춰 정책이 자유롭게 접근하도록
- **변경**:
  - k2_precision 5.0 → 0.3 (success bonus 완만화)
  - w_drop_base 50 → 100
  - random_drop_start_step 150 → 600 (이전엔 너무 일찍 발화)
  - timeout reward -50 → -150
- **결과**: 150k 완주, 427 drops, **best 2.53m**, success 16건 (3.7%, **16배 증가**)
- **발견**:
  - Reset 버그 (n_steps=1 가짜 success 13건) → 해결됨
  - Post-success regression (success 직후 발산)
  - Deterministic eval: drop 0건 (정책 stochastic 의존)
- **다음 model (Round 3) 로 교체 이유**:
  - post-success regression 처방 필요
  - 학습 효율 추가 개선 위해 PER 도입 검토
- **현재 보존**: ❌

---

### Round 1 — 첫 베이스라인 (폐기)

**한 줄**: 학습 파이프라인 + 보상 시스템 검증용 첫 학습.

- **이전 model**: 없음 (첫 시도)
- **이 model 의 도입 목적**:
  - 보상 시스템 + 학습 파이프라인 동작 검증
  - 베이스라인 정량 측정
- **설정**: 기본 SAC, default 처방 없음 (이후 모든 처방의 시작점)
- **결과**: 150k 완주, 432 drops, **best 4.64m**, success 1건 (0.2%)
- **다음 model (Round 2) 로 교체 이유**:
  - success 1건 → 학습 신호 너무 sparse
  - Reward gradient 완만화 필요
- **현재 보존**: ❌ (이후 모든 라운드가 갱신)

---

## 핵심 발견의 시간 순 정리 (최신순)

| 시기 | 발견 | 영향 |
|---|---|---|
| Phase 1 redux v1 | drop 잦음 → _kill_infra timeout 5s 가 fps 폭락 원인 | v2 에서 2s 적용 |
| Phase 1 redux v1 | 96% success at 5m 빠르게 달성 | curriculum 가능성 확인 |
| Phase 1 eval | 정책이 random_drop 의존 | **Phase 1 redux 전환** |
| Phase 1 eval | Episode 간 의존성 (PX4 EKF carry-over) | A2 hybrid (gz_world_reset) |
| Phase 1 eval | mean action ≠ stochastic train | deterministic gap 발견 |
| Round 7 v3 | 1·2차 처방 작동 검증 | #021 처방 효과 입증 |
| Round 7 v3 | critic_loss 폭주 (250x) 가 ent 발산 원인 | Huber + target_q_clip 처방 |
| Round 7 1차 | gz timeout crash (3번째) | **Issue #021 신규** |
| Round 7 1차 | bounded action + target_entropy 분석 | **Issue #019 v2 — 근본 처방** |
| Round 6 v2 | Percentile 작동 but cap 도달 | target_entropy 의심 |
| Round 6 v1 | Mean damping 실패 | percentile 시도 |
| Round 5 | per-step + terminal 모두 발산 | **Issue #019 (SAC 본질적 발산)** |
| Round 4 | SAC entropy 발산 (per-step) | Round 5 다른 접근 |
| Round 2 | Post-success regression | Round 3 처방 후보 |
| Round 2 | Reset 버그 (가짜 success) | 해결 (pos_enu 명시 초기화) |

---

## 현재 보존 중인 모델 자산

```
local/backups/phase1_final_round7_v3/
├── sac_drop_final.zip                ← Round 7 v3 685k endpoint (1.32m)
├── success_replay_436xl0bb/          ← Round 7 v3 의 15 success 모델
├── drop_episodes/                    ← 679 npz trajectories
├── wandb_run/                        ← 전체 metric history
└── 코드/yaml snapshot

ros2_ws/rl_checkpoints/archive/
└── phase1_redux_v1_pause_89k/        ← Phase 1 redux v1 의 89k preempt
    ├── sac_drop_preempt.zip          ← (v2 가 여기서 resume 중)
    └── sac_drop_preempt_replay.pkl

ros2_ws/success_replay/
├── round6_v2_recovered/              ← Round 6 v2 의 160k best (4.36m, 역사적)
├── ayi27a56_best5/                   ← Phase 1 redux v1 best 5 (0.81m, ...)
└── za9zxdh6/                         ← Phase 1 redux v2 현재 진행
```

## 관련 문서

- `local/issues/master.txt` — 안건 전체 관리
- `local/issues/issue_019_sac_entropy_divergence.txt` — SAC 발산 처방 history
- `local/issues/issue_021_gz_timeout_recurrence.md` — #021 처방
- `local/parameter_log.md` — 파라미터 변경 시간 순 (Entry #1~#28)
- `local/design/design_review.md` — 현재 최종 설계
- `local/design/phase2_plan.md` — Phase 2 검토 (현재 보류)
- `local/backups/phase1_final_round7_v3/README.md` — Phase 1 백업 상세

---

## 새 라운드 추가 시 절차

1. 이 문서 맨 위의 "한 눈에 보기" 표에 새 행 추가 (가장 첫 줄)
2. "라운드별 상세" 의 가장 위에 새 섹션 추가 (`---` 위에)
3. 각 섹션은 **narrative 형식** 으로:
   - **한 줄 요약**
   - **이전 model 의 한계** (왜 새 model 이 필요했나)
   - **이 model 의 도입 목적**
   - **핵심 변경**
   - **결과**
   - **다음 model 로 교체 이유**
   - **현재 보존**
4. 옛 라운드들은 그대로 (시간 흐를수록 자연스레 아래로 밀림)
5. 보존/폐기 결정 기록
6. parameter_log.md 에도 Entry 추가 (동시)

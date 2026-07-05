# Design Review — 2026-07-05

> RAD v1 Phase 1 학습 v1~v6 종합 + 진짜 원인 확정 + curriculum stage 재설계 안.

작성: 2026-07-05
관련:
- [rad_v1_design.md](rad_v1_design.md) — RAD v1 single source of truth (self-healing + curriculum + regression 도구 반영)
- [design_review.md](design_review.md) — 총 요약 (RAD v1 현재 상태 갱신)
- [design_review_2026-06-30.md](design_review_2026-06-30.md) — RAD 도입 결정
- [../parameter_log.md](../parameter_log.md) — v1~v6 각 변경 entry
- [../issues/issue_028_rad_ekf_stale_after_reset.md](../issues/issue_028_rad_ekf_stale_after_reset.md) — 진단 방향 정정
- [../issues/issue_029_curriculum_stage_spawn_ignorance.md](../issues/issue_029_curriculum_stage_spawn_ignorance.md) — 진짜 원인
- [../meeting_notes/meeting_notes_2026-07-05.txt](../meeting_notes/meeting_notes_2026-07-05.txt)

---

## 1. 이번 주 목표

RAD v1 Phase 1 학습 300k step 완주 + sphere entry 성공률 90%+.

## 2. v1 ~ v6 시행 이력

| Version | 결과 | 주요 변경 |
|---|---|---|
| v1 (원본) | NaN abort at 62k step | Original RAD v1 그대로 |
| v2 (self-healing) | NaN abort at 171k step | + A1 gradient clip + B3 weight NaN rollback + 상세 metric |
| v3 (curriculum) | Stage1 정체 (75%) | + 5-stage curriculum + regression |
| v4 (reward 축소) | stage1↔2 cycle 반복 | + crash/hover/truncation penalty 축소 + target_q_clip 500→200 |
| v5 (stage 완화) | 여전 실패 (hover 65%) | + stage2_close 조건 완화 (radius 5→5.5m 등) |
| v6 (근본 접근) | stage3 진입 후 실패 | + spawn_yaw ±45° + hover -30 + initial pos log |

## 3. v6 진행 상세

Curriculum 이벤트:

| Step | 이벤트 |
|---|---|
| 14,011 | Advance 1: stage1 → stage2 |
| 27,813 | Advance 2: stage2 → stage3 (v4/v5 못 넘긴 지점 돌파) |
| 37,828 | Regress: stage3 실패 (성공률 0%) → stage2 |
| 52,138 | Advance 3: stage2 → stage3 (재도전) |
| 62,214 | Regress: stage3 재실패 (0%) → stage2 |

**Stage3 두 번 도전 모두 완전 실패**. Cycle 반복 pattern 확정.

## 4. 진짜 원인 (initial pos log 로 확정)

### 실측 fact

정책 시작 시점 실제 위치 (100 ep 통계):
- `initial_target_dist_xy_mean` = **5.10m** (min 5.00, max 5.23)
- `initial_pos_x_mean` = -0.157m
- `initial_pos_y_mean` = 0.049m
- `initial_speed_xy_mean` = 0.031 m/s

Spawn (0, 0, 0) → Target (4, 3, 0) 직선 거리 = **5.0m**. Cruise 로 이동 0.15m 미미.

### Stage 조건 vs 시작 거리 매칭

| Stage | Sphere radius | Switch d² | Spawn d²≈25 | 학습 유도? |
|---|---|---|---|---|
| stage1_intro (v6) | 6.0m | 36.0 | 안 (여유 11) | ✗ trivial |
| stage2_close (v6) | 5.5m | 30.25 | 안 (여유 5.25) | ✗ trivial |
| **stage3_target** | **4.53m** | **20.5** | **밖 (부족 4.5)** | **✓ 유일 학습 stage** |

### 결론

- **Stage1/2 는 정책이 학습 없이 통과** (spawn 이 이미 sphere 안)
- Stage3 에서 처음 이동 학습 필요 → 정책 능력 부재 → **완전 실패 (성공률 0%)**
- 실제 학습해야 할 이동거리 = **0.47m** (5m → 4.53m)
- Curriculum stage 설계가 spawn 위치를 반영 안 함 = 근본 결함

## 5. Stage 재설계 안 (다음 실험 v7)

### 5-stage 재설계 (spawn d²≈25 반영)

| Stage | switch_d² | Radius | active_conditions | z_min | max_dist | limit_tilt |
|---|---|---|---|---|---|---|
| stage1_intro | 23.04 | 4.80m | [] | 0.05 | 40 | 1.2 |
| stage2_close | 21.62 | 4.65m | [] | 0.15 | 30 | 0.9 |
| stage3_target | 20.5 | 4.53m | [] | 0.25 | 25 | 0.5 |
| stage4_partial | 20.5 | 4.53m | [C1, C4, C5] | 0.35 | 20 | 0.30 |
| stage5_full | 20.5 | 4.53m | [C1~C7] | 0.5 | 15 | 0.26 |

**변경 원리**:
- Stage1 부터 spawn 밖 (d²=23 < spawn d²=25) → 정책이 이동 학습 필요
- 매 stage radius 갭 0.15m (매우 완만)
- Stage3 까지는 sphere entry 자체만 목표 (조건 없음)
- Stage4, 5 에서 조건 추가

### 예상 학습 궤적

1. Stage1 (radius 4.80m): 정책이 target 방향 0.2m 이동 학습
2. Stage2 (radius 4.65m): 0.35m 이동
3. Stage3 (radius 4.53m): 0.47m 이동 (design 목표)
4. Stage4: 부가 조건 3개 학습
5. Stage5: 부가 조건 7개 학습

Regression 발동 예상 지점: stage2, stage3 (radius 축소 시 어려움 증가)

## 6. Self-healing 매커니즘 (재사용, v2 도입)

### A1: Gradient clipping
- Actor: `clip_grad_norm_(max_norm=20)`
- Critic: `clip_grad_norm_(max_norm=200)`
- 목적: gradient 크기 제한 → weight 폭발 방지

### B3: Weight NaN rollback
- 매 gradient step 시작 시 snapshot (state_dict copy)
- Optimizer step 후 weight NaN/Inf 검사
- NaN 감지 시 snapshot 복원 + zero_grad
- Log: `train/nan_rollback_count`

### 실전 관찰
- v2~v6 전 학습에서 nan_rollback_count = 0 (발동 안 함)
- NaN abort 은 A1 (gradient clip) 이 예방하는 게 지배적
- B3 는 marginal case 대비 안전망

## 7. Curriculum + Regression 매커니즘 (v3 도입)

### 구조
- Stage 순차 진행 (0 → 1 → 2 → 3 → 4)
- Window-based success rate 판정 (window=10k step, threshold=0.95)
- Regression: window success < 0.3 시 이전 stage 복귀 (cooldown 20k)
- Deque maxlen 200 (ep history)

### v6 실전 검증
- Advance 3회, Regression 2회 정상 발동
- 매커니즘 자체는 정상 작동
- **하지만 근본 문제 (stage 설계) 를 매커니즘으로 해결 못 함**
- Regression 은 "학습 실패 시 되돌아가기" 만 담당. Stage 조건 자체 재설계는 별개.

## 8. Reward magnitude 정책 (v4 도입)

| Parameter | v1~v3 | v4 이후 | 변경 이유 |
|---|---|---|---|
| penalty_crash | -50 | **-20** | Q negative 축적 완화 |
| penalty_hover | -30 | -10 (v4) → **-30** (v6) | v4 는 magnitude 축소, v6 는 hover local optimum 회피 |
| truncation_penalty | -15 | **-5** | Truncation 페널티 축소 |
| target_q_clip | 500 | **200** | Q bootstrap 발산 압력 축소 |
| max_consecutive_fast_resets | 50 | **500** | Phase 1 은 drop 없음 → fast reset 정상. Threshold 상승 |

### hover_penalty 재조정 원칙 (v6 도입 시 확립)
- Hover penalty ≥ crash penalty (v6: -30 vs crash -20)
- 이유: crash > hover 이면 정책이 hover 를 "안전 실패" 로 선택 → local optimum
- Hover ≥ crash 이면 정책이 hover 대신 능동적 시도 선택
- v5 hover 65% → v6 hover 1% (65배 감소) 로 검증

## 9. Initial position 실측 도구 (v6 도입)

### 구현
- Env step() 첫 step 시점에 위치/속도/거리 저장 (`self._initial_target_dist_3d` 등)
- Ep 종료 시 info 에 write
- Callback (TerminalTypeMonitorCallback) 이 rolling stats (window 100 ep) → WandB

### Metric
- `env/initial_target_dist_3d_{mean,min,max}`
- `env/initial_target_dist_xy_{mean,min,max}`
- `env/initial_pos_{x,y,z}_{mean,min,max}`
- `env/initial_speed_xy_{mean,min,max}`

### 근본 원인 파악의 결정적 도구
- 이전 진단 (v1~v5) 는 이론 계산 기반 → 여러 번 오추정
- v6 도입 직후 첫 dump 에서 실제 위치 = 5.10m 확인 → 원인 즉시 파악
- **모든 debug 시 실측 도구 우선 원칙** 확립

## 10. 다음 결정

**옵션 1 (권고)**: Stage 재설계 (5장 상세) — v7 로 진행
**옵션 2**: v6 cycle 계속 관찰 — 실용성 낮음
**옵션 3**: 다른 근본 접근 (spawn 위치 mission_manager 수정 등)

---

## 부록: 시간별 요약

| 날짜 | Version | 주요 변경 | 결과 |
|---|---|---|---|
| 6/30 | v1 | 원본 설계 | NaN abort at 62k |
| 7/2 | v2 | + Self-healing (A1+B3) | NaN abort at 171k |
| 7/3 | v3 | + Curriculum + regression | Stage 1 정체 |
| 7/3 | v4 | + Reward magnitude 축소 | Stage 1↔2 사이클 반복 |
| 7/4 | v5 | + Stage 2 완화 | 여전 실패 (hover 65%) |
| 7/4~5 | **v6** | **+ Yaw ±45° + Hover -30 + Initial log** | **Stage 3 진입, cycle 반복. 진짜 원인 확정** |

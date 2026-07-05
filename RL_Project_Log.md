# RL Training Pipeline — Project Log

> **Branch:** `main` | **VM 프리엠션 후 컨텍스트 복구용 로그**

---

# 1. Current State

**업데이트:** 2026-07-05

### 두 트랙 병행

**🥇 Track 1 — V8 baseline (검증 완료, 실사용 대상)**
- Phase 1 redux v8 (96bokgae, 303k): 80.6% success, best drop 0.07m (13 jackpot)
- 10 ep dgui 재평가: 50% success, mean 2.002m
- 백업: `local/backups/phase1_redux_v8_2026-06-21/` (8.6 GB 완성 자산)
- 평가 모델: `ros2_ws/eval_models/v8_{peak,best,final}_*.zip` (3종)
- 상태: 검증 완료. 사용자 결정 (2026-06-27) = v8 = best baseline 확정.
- 관련 문서: [local/design/design_review_2026-06-27.md](local/design/design_review_2026-06-27.md), [local/design/model_history.md](local/design/model_history.md)

**🟡 Track 2 — RAD v1 (Relative Approach Drop, 진행 중 실험)**
- v8/v9a 와 완전 다른 framework (Round/redux 시리즈 patch 아님, 새 framework 의 v1)
- Phase 1: Approach (sphere entry) → Phase 2: Drop (정밀 투하)
- Phase 1 학습 진행 중 (v6, 62k+ step)
- 관련 문서: [local/design/rad_v1_design.md](local/design/rad_v1_design.md), [local/design/design_review_2026-07-05.md](local/design/design_review_2026-07-05.md)

### RAD v6 진행 상태 (Track 2)
- 62k+ step (2026-07-05 시점, 중단 결정 대기)
- Stage3 진입 후 완전 실패 → 자동 재도전 cycle 반복
- **진짜 원인 확정 (Issue #029)**: curriculum stage 조건이 spawn 위치 (target 거리 5m) 반영 안 함
- 개입 결정: Stage 재설계 (v7 계획)
- **V8 baseline 은 별개 트랙으로 계속 유효** (실사용 시 v8 사용)

### 학습 환경 (RAD v1)
| 파라미터 | 값 |
|---|---|
| Algorithm | SAC (SB3) + custom DampedEntropySAC (self-healing 포함) |
| Network | `net_arch=[256,256]`, `device=cuda` |
| num_envs | 1 |
| target_timesteps | 300k (진행 중 62k+) |
| GPU | NVIDIA RTX 4060 Laptop (8GB VRAM) |
| Container | drone-bombard-harmonic |

### Self-healing (v2 부터 활성)
- **A1**: gradient clip (actor max_norm=20, critic max_norm=200)
- **B3**: weight NaN 감지 + snapshot rollback
- 실전: v2~v6 학습에서 nan_rollback_count = 0 (발동 안 함, A1 예방 지배적)

### Curriculum + Regression (v3 부터 활성)
- 5-stage (intro/close/target/partial/full)
- Advance: window 10k step, threshold 0.95, min_stage 10k
- Regression: window success < 0.3 시 이전 stage 복귀, cooldown 20k
- 실전: v6 에서 Advance 3회, Regression 2회 정상 발동

### Initial position 실측 도구 (v6 신규)
- Env step() 첫 step 위치/속도/거리 저장
- Callback rolling stats (100 ep) → WandB
- **근본 원인 파악의 결정적 도구**
- Metric: env/initial_target_dist_{3d,xy}_{mean,min,max}, initial_pos_{x,y,z}_*

---

# 2. Recent Progress

- **2026-07-05:** RAD v1 Phase 1 v6 진행 중. Initial pos log 로 진짜 원인 확정 — curriculum stage 조건이 spawn 위치 반영 안 함 (Issue #029). Stage 재설계 안 확정 (v7 대기). 문서 전면 업데이트.
- **2026-07-04:** v5 (stage2 완화) 실패 → v6 (spawn_yaw ±45°, hover -30, initial pos log) 시작. Stage 2 통과 (v4/v5 stuck 지점 돌파). Stage 3 완전 실패.
- **2026-07-03:** v3 (curriculum + regression) 도입, Stage 1 정체 → v4 (reward magnitude 축소) → Stage 1↔2 cycle 반복. Reward magnitude 축소 효과 (Q_target_std 289→166).
- **2026-07-02:** v1 NaN abort 후 v2 self-healing 도입 (A1 gradient clip + B3 weight rollback + 상세 metric). v2 도 171k 에서 NaN abort 재현.
- **2026-06-30:** RAD v1 (Relative Approach Drop) 새 framework design 완료 + v1 학습 시작. Phase 1 sphere entry + 7 조건 목표.
- **2026-06-27:** v9a fine-tune 미달성 → v8 = best baseline 확정. 새 design framework 방향 결정.
- **2026-06-22:** dgui (evaluate_gui.py) 도구 완성. ang_vel callback fix (Issue #024).
- **2026-06-21:** Phase 1 redux v8 완성 (96bokgae, 303k, 80.6% success, best 0.07m).

---

# 3. Remaining Tasks

### 즉시 (v7 stage 재설계)
- [ ] v6 학습 종료 결정 (사용자 대기)
- [ ] Curriculum yaml 재설계 반영:
  - Stage1: radius 4.80m, 조건 없음, crash 완화
  - Stage2: radius 4.65m, 조건 없음
  - Stage3: radius 4.53m, 조건 없음
  - Stage4: radius 4.53m + [C1, C4, C5]
  - Stage5: radius 4.53m + [C1~C7]
- [ ] v7 학습 시작 (300k)
- [ ] Stage 진입 후 정책이 이동 학습하는지 initial log 로 확인
- [ ] Stage 1 → 2 → 3 → 4 → 5 순차 진행 여부 관찰

### 이후 (Phase 2 준비)
- [ ] Phase 1 완료 시 policy checkpoint 저장
- [ ] Phase 2 (Drop) 학습 설계 확정
- [ ] Phase 2 curriculum 설계 (drop 정밀도 단계)
- [ ] Phase 1 policy 를 checkpoint 로 사용해 Phase 2 warm start

### 장기
- [ ] Multi-env parallel training (fps 개선)
- [ ] Curriculum 자동 조정 (진행 상태에 따라 dynamic threshold)
- [ ] Reward shaping 재검토 (dense signal 강화)

---

# 4. Training History (추가만 가능)

| Version | 시기 | 결과 | 주요 변경 |
|---|---|---|---|
| RAD v1 Phase 1 v1 (g8mvzniw) | 2026-06-30~07-02 | NaN abort at 62k | Original RAD v1 design 그대로 |
| RAD v1 Phase 1 v2 | 2026-07-02 | NaN abort at 171k | + Self-healing (A1 grad clip + B3 weight rollback + 상세 metric) |
| RAD v1 Phase 1 v3 | 2026-07-03 | Stage 1 정체 (75%) | + 5-stage curriculum + regression, advance 강화 |
| RAD v1 Phase 1 v4 | 2026-07-03 | Stage 1↔2 cycle 반복 | + Reward magnitude 축소 (crash -20, hover -10, trunc -5, q_clip 200, fast_reset 500) |
| RAD v1 Phase 1 v5 | 2026-07-04 | Stage 2 hover 65% 실패 | + Stage 2 완화 (radius 5→5.5m, z_min 0.15→0.08, tilt 0.9→1.0) |
| RAD v1 Phase 1 v6 | 2026-07-04~05 | Stage 3 완전 실패 (cycle 반복), 진짜 원인 확정 | + spawn_yaw ±45°, penalty_hover -30, initial pos log 신규 |
| **RAD v1 Phase 1 v7** (계획) | 2026-07-05~ | TBD | + Stage 재설계 (radius 4.80→4.65→4.53m gradient, spawn 5m 반영) |

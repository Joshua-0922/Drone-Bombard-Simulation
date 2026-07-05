# Design Review — Drone-Bombard-Simulation (2026-06-30)

> **배경**: v9a 처방 (drop_angaccel + w_dist 1.5) 의 한계 확인 + 사용자 의도 (toss 의 다양성 / 사용자 thoughts framework 반영) 위해 새 framework 설계 결정.

> **이전 design_review**:
> - [design_review.md](design_review.md) — 현재 활성 처방 요약 (v8/v9a)
> - [design_review_2026-06-27.md](design_review_2026-06-27.md) — v9a 결과 분석 + 다음 처방 후보
> - [two_stage_learning_plan.md](two_stage_learning_plan.md) — 사용자 thoughts 원안

작성: 2026-06-30
관련:
- [rad_v1_design.md](rad_v1_design.md) — RAD v1 의 모든 design 결정 통합 (single source of truth)
- [model_history.md](model_history.md) — RAD v1 entry
- [../parameter_log.md](../parameter_log.md) — entry #42 (RAD v1)

---

## 1. 결정 — 새 framework "RAD v1" 도입

**RAD** = **R**elative + **A**pproach (Phase 1) + **D**rop (Phase 2).

Round / Phase 1 redux 시리즈 (single SAC, 절대좌표 obs, drop reward 통합) 와 **완전히 다른 framework**. v9 의 patch 가 아닌 **새 framework 의 v1**.

### 핵심 결정 요약

| 영역 | 결정 |
|---|---|
| 정책 구조 | 2 SAC (Phase 1 approach + Phase 2 drop, warm start init only) |
| obs | 14d 상대좌표 (yaw-only body frame) |
| spawn yaw | uniform ±90° relative to drone→target |
| cruise | head 방향 1 m/s 가속 (자동 이동 폐기) |
| target | (4, 3, 0) 지면 marker (z 통합) |
| Phase 1 종료 | switch sphere d²≤20.5 진입 + 7 final state 조건 jackpot |
| Phase 2 drop trigger | d_impact ≤ 1.0m (v8: 2.0 → 1.0) |
| Phase 2 success | drop_error ≤ 1.0m (trigger 와 정합) |
| 인프라 | I-2 (Phase 1 정책 rollout 매 ep, sphere 도달까지) |
| 코드 분리 | 옵션 A (파일 분리, 같은 package) — `_rad` 접미사 |

→ 전체 세부 사항은 [rad_v1_design.md](rad_v1_design.md) 참조.

---

## 2. 결정 동기 (v9a 의 한계 + 사용자 의도)

### v9a 의 4 가지 한계 (2026-06-27 평가)

1. **처방 흡수**: w_dist 1.5 효과 미미 (terminal +30 의 5%) — 17k step fine-tune 부족
2. **Catastrophic forgetting 위험**: v9a resume 432k 에서 success rate ↓ (313k 33% → 432k 67% 5 ep 이지만 hover_timeout 2)
3. **CUDA error 위험**: rolling checkpoint 5k 마다 외에 안전망 없음
4. **사용자 의도 미달성**: toss 행동 그대로 유지, "지나치는 현상" 해결 안 됨

### RAD 가 해결하는 영역

| 한계 | RAD 의 해결 |
|---|---|
| 처방 흡수 | Phase 1/2 정책 분리 → 각 정책이 자기 reward 만 학습 |
| Catastrophic forgetting | warm start init only + Phase 1 freeze 별도 보관 |
| 사용자 의도 (다양성) | spawn yaw 랜덤 + 상대좌표 obs → 매 ep 다른 task perspective |
| toss 학습 보존 | k_drop_proximity 약화 + auto_drop 1m + w_impact 강화 |

---

## 3. RAD 의 3 가지 핵심 변경 (사용자 thoughts 매핑)

| 사용자 thoughts | RAD 구현 |
|---|---|
| "Phase 1 = 12m 안 도달까지 학습, drop 없음. 종단에서 다양한 속도/방향/높이" | switch sphere d²≤20.5, Phase 1 drop dead, spawn yaw 랜덤으로 다양한 시작 |
| "Phase 2 = Phase 1 의 종단 state 에서 drop 정밀도 학습. Phase 1 정책 건드리지 말 것" | Phase 1 freeze + Phase 2 warm start init only (D1-a) |
| "확률적 융합 (50:50 mixing) 대신 다른 접근" | I-2 (Phase 1 rollout 매 ep) — Phase 1 정책의 stochasticity 가 자동 융합 |

---

## 4. 결정에서 폐기된 후보들

### 4.1 정책 구조

| 후보 | 폐기 이유 |
|---|---|
| 상태머신축 (단일 SAC + mode bit) | reward hacking 가능성 ↑ — v9a 와 같은 구조 |
| 커리큘럼축 (단일 SAC + reward schedule) | hyper-param hell, 2 단계 분리 의미 약화 |
| 시간축 (Phase B→C→D→E ladder) | 사용자 thoughts 원안에 가장 가까우나, fusion 검증 부족 + 4 phase 관리 비용 |

→ 정책축 (2 SAC) 선택 — catastrophic forgetting 차단 가장 강력.

### 4.2 인프라

| 후보 | 폐기 이유 |
|---|---|
| I-1 (Gazebo state injection) | 6 가지 silent fail risk (set_entity_pose, EKF reset, GPS sync, etc.). 우리 인프라 경험상 도박 |
| I-3 (Hybrid) | I-2 만으로 다양성 충분 시 불필요. 추후 검토 |

→ I-2 (Phase 1 rollout) 선택 — 인프라 변경 최소.

### 4.3 obs

| 후보 | 폐기 이유 |
|---|---|
| Full body frame (yaw+pitch+roll 회전) | pitch/roll 자세 정보가 obs 에 별도 포함 — 회전 적용 안 함이 더 명확 |
| World frame (변경 없음) | spawn yaw 랜덤화 시 매번 다른 task 로 보임 → 정책 학습 어려움 |
| 17d 그대로 + mode flag 만 추가 | v10 의 다양성 보존 효과 작음 |

→ Yaw-only body frame 14d 선택.

### 4.4 z reward 형태

| 후보 | 폐기 이유 |
|---|---|
| Gaussian | 가장자리 (z<0.5 or z>7.5) 에서 0 으로 안 떨어짐 — 사용자 의도 미부합 |
| Quadratic / Triangle | 미분 불연속 또는 plateau 없음 — gradient 학습 친화성 ↓ |
| Quartic | 정상 plateau 가 너무 평평 — 정책 z=4 정확히 안 가도 비슷한 reward |

→ Hann (raised cosine) 선택 — 매끄러운 미분 + 가장자리 정확히 0.

### 4.5 Final state reward 메커니즘

| 후보 | 폐기 이유 |
|---|---|
| F1 Binary AND | 정책이 "5/7 만족" 영역에서 local optimum |
| F3 Multiplicative penalty | floor 보장 없음, 학습 신호 sparse |
| F4 Soft barrier | 모든 조건에 soft barrier 면 dense reward 폭증 — 학습 noise |

→ F2 Weighted sum + Jackpot (W3 균등 + jackpot 강화) 선택. 6→7 만족 시 +57 강한 incentive.

### 4.6 Drop trigger

| 후보 | 폐기 이유 |
|---|---|
| A1 v8 그대로 (d_impact ≤ 2m) | Phase 2 시작 즉시 trigger 발동 가능 → 학습 신호 부족 |
| A3 Manual only | drop ASAP exploit 위험 |
| A4 Hybrid + safety net | 사용자 결정: 강제 drop 안 함 (학습 속도 우선) |

→ A2 (auto d_impact ≤ 1m, 강제 drop 없음) 선택.

### 4.7 Success threshold

| 후보 | 폐기 이유 |
|---|---|
| 2.0 (v8) | trigger 1m 과 mismatch → 모든 drop 이 success → 학습 신호 약함 |
| 0.5 | trigger 후에도 fail 가능, 단 trigger 자동 발동되므로 정책이 학습 불가 |
| 0.3 (= jackpot) | 너무 빡빡 |

→ 1.0 (trigger 와 정합) 선택.

---

## 5. RAD v1 학습 전 체크리스트

- [ ] 코드 신규 파일 작성 (drone_drop_env_rad.py 등 6 개)
- [ ] hyperparams_rad.yaml 작성
- [ ] launch 파일 작성 (infra_rad / episode_rad)
- [ ] setup.py 업데이트 (entry point + data_files)
- [ ] colcon build --packages-select rl_navigation mission_manager
- [ ] **Reset 잔존 속도 dgui smoke test (#129)** — 학습 신뢰성 확보
- [ ] Phase 1 5k dry-run (구현 검증)
- [ ] Phase 1 학습 시작 (300k 목표)
- [ ] Phase 1 종료 후 final model 백업 (`rl_checkpoints_rad/sac_phase1_final.zip`)
- [ ] Phase 2 학습 시작 (150k 목표)
- [ ] Phase 2 종료 후 dgui 통합 eval (10 ep)

---

## 6. RAD v1 학습 시작 후 모니터링 priority

| Phase | 주요 metric | watch out |
|---|---|---|
| Phase 1 | success_rate (7 조건 jackpot 비율) | sphere 진입 rate 보다 success_rate 가 너무 낮으면 → 어떤 C 조건이 어려운지 (C_satisfied_rate 별 확인) |
| Phase 1 | init yaw 분포 | uniform 확인 (spawn 코드 검증) |
| Phase 1 | 종단 state 분포 (z, v_xy, v_z, tilt, ω) | 좁으면 (v8 같은 한 점 분포) → target_entropy 완화 검토 |
| Phase 2 | success_rate (drop_error ≤ 1m 비율) | trigger 빈도와 비교 — gap 크면 d_impact 와 d_error 의 prediction_gap 검토 |
| Phase 2 | sphere_escape_rate | > 5% 이면 margin (0.16m) 늘려야 할 수도 |
| Phase 2 | ep_length_p50 | 사용자 원칙 "짧고 빠른 drop" — < 100 step 목표 |
| Phase 2 | mean ang_vel at drop | v9a 처방 (drop_angaccel 0.5) 의 효과 확인 |

---

> **결론**: RAD v1 design 완료. 코드 작성 (옵션 A 파일 분리) + Reset 잔존 속도 검증 후 학습 시작.

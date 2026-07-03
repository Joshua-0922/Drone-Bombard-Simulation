---
date: 2026-07-03
tags: [research, isaac-lab, ppo, reward, tuning, vision]
status: active
type: research
---

# Isaac PPO 1차 학습 결론 — 무엇을 바꿔야 하는가 (exp_013 기반)

> **근거 실험:** [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] (wandb `wcjklw7a`,
> 2048 envs × 1000 iters, deterministic eval 200-ep).
> **결과 요약:** success 36% plateau, d_xy_min 1.4m 정체(게이트 0.8m 밖), 실패는
> max_altitude 33% + crash 27%. 원인 3종(1-3번)이 코드/수식 수준에서 규명됐고, **사후
> 검증에서 0번(리셋 속도킥)이 이 전부를 교란하는 선행 물리 버그로 확인됨** (exp_013 §4d).

---

## 우선순위 변경 목록

| # | 대상 | 현재 | 권고 | 근거 | fresh start |
|---|---|---|---|---|---|
| **0** | **★리셋 속도킥 수정** (`_apply_body_mass_override` → 스폰타임 USD 질량 설정) | 런타임 `set_masses()` → 매 리셋 ~7-9 m/s 수직킥 ([[isaac_mass_override_reset_bug]], **미해결**) | `CRAZYFLIE_CFG.replace(...)`의 spawn/mass_props 단계에서 질량·관성 설정(articulation 생성 전) — PhysX 내부 표현 전부에 일관 반영. 수정 후 `play.py --zero-actions` **PASS(<1m) 확인이 이후 모든 실험의 게이트** | exp_013 사후 검증 FAIL(드리프트 11.9m). 킥이 스폰 9-11m를 ~18-22m(천장 25m 직하)로 밀어올림 → **max_alt 33%·crash 27%의 1차 용의자.** 이거 안 고치면 1-3번 효과 측정 불가 | ✅ 필수 (plant 변경) |
| 1 | **analytic conf 거리 감쇠** (`DroneBombardVisionCfg` / `_update_vision`) | conf 0.73-0.95, 거리 무관 | slant range에 비례해 conf·탐지확률 감쇠 (예: `conf *= clamp(1.5 - slant/10, 0.1, 1)` 계열 — YOLO 캘리브레이션(`yolo_eval.py --calibrate`)으로 커브 확정이 정도) | 거리-무관 conf + 고도↑=centering 기하 유리 → **고도 상승 farming attractor**(기하적으로 실증; 단 exp_013의 max_alt 33% 귀속은 0번 킥과 분리 불가 — 0번 수정 후 재측정). 실제 YOLO는 apparent size ∝ 1/거리(Rule 13) — parity 회복이 목적이므로 buff가 아니라 버그 수정에 가깝다 | ✅ 필수 |
| 2 | **`reward_success`** (`DroneBombardRewardCfg`) | 100 | **300** | farmer(+225) > finisher(+121) 수지 역전용. 300이면 finisher ≈ +321로 지배 전략 복원. Gazebo v14 final-approach stagnation과 동일 병인 | ✅ 필수 |
| 3 | **`entropy_coef`** (`agents/rsl_rl_ppo_cfg.py`) | 0.005 | **0.0** (또는 0.001 + noise_std 모니터링) | noise_std 0.8→3.92 폭주. 스무딩 액션 파이프라인(clip→rate_limit→LPF)이 노이즈를 필터링해 entropy 압력을 견제할 task 손실이 없음 | PPO라 buffer 오염은 없으나 위 1·2와 묶어 fresh 권장 |
| 4 | `num_steps_per_env` | 32 | 64 (선택) | 종단 보상 300 도입 시 GAE horizon(3.2s)이 평균 에피소드(~6s)의 절반 — credit assignment 여유 확보. 2차 실험에서 A/B | — |
| 5 | `w_vision_center` | 1.5 | 유지 (1번 적용 후 재평가) | 1번이 들어가면 원거리/고고도 farming 자체가 사라짐. 1번 없이 이것만 낮추면 비전 서보잉 신호까지 죽는다 — 순서 중요 | — |
| 6 | **스폰 고도** `spawn_alt_range` (사용자 질문 "initial height") | (9, 11) m | **유지** | v15 parity 값. 고도 자체는 실패 원인이 아님(실패는 보상 지형). 단 스폰 가시성이 63%(6-7m 스폰은 17%)이므로, 1번 적용 후 acquisition이 문제 되면 `handoff_dist_range` (3,7)→(3,6) 축소가 올바른 레버(고도 아님 — Rule 13 동일 원리) | — |
| 7 | `success_radius` | 0.8 | 유지 → success ≥70% 안정 후 0.5 커리큘럼 (`--resume`) | 기존 계획대로. 지금 조이면 4b 불균형이 악화만 됨 | 커리큘럼은 resume |
| 8 | `max_altitude` / `min_altitude` | 25 / 3 m | 유지 | max_alt 33%는 천장이 낮아서가 아니라 상승이 이득이라서다. 1번이 원인 제거. crash 27%(다이브)도 1-3 적용 후 재측정 — 원인 미확정 항목으로 이월 | — |

**적용 순서/묶음:** **0을 단독 선행하고 `--zero-actions` PASS를 게이트로** — 그 다음
1+2+3을 한 번에 fresh로 (exp_014 제안: `exp014_massfix_visionrange_succ300_ent0`).
0만 고친 짧은 대조 run(200-300 iters)을 사이에 끼우면 "킥 제거만으로 얼마나 회복되는지"가
분리 측정되어 1-3의 효과 크기를 정직하게 잴 수 있다 — 시간 여유 있으면 권장.
4는 exp_014 결과 보고 A/B. 5-8은 관찰 대기.

---

## 판정 요지 (사용자 질문에 대한 답)

1. **수렴했는가?** — plateau 수렴(iter ~700 이후 flat, last200 Δreward −3.6)이지만
   **올바른 행동으로의 수렴은 아니다**: deterministic 36%로 rollout과 동일 → 노이즈
   문제가 아니라 정책 평균이 세 모드(완주 36 / 상승-천장 33 / 다이브 27)로 갈라진 채 안정화.
   그리고 이 수치 자체가 **리셋 속도킥이 활성인 오염 plant에서 측정된 것**(0번) —
   진짜 능력 상한이 아니다.
2. **mean d_xy는 잘 가는가?** — 초중반은 건강(4.1→1.18m), 종반 1.4m 정체. 게이트 0.8m
   밖에서 멈춘다. "접근은 배웠고 마무리는 안 배움" — 그리고 현 보상 지형에선 마무리를
   안 배우는 게 **합리적**(farmer 수지 우세)이라는 것이 구조적 발견(0번과 무관하게 유효).
3. **무엇을 바꿔야?** — 위 표. 순서가 중요: **물리 버그(0번) → 보상/비전모델 구조(1·2번)
   → 하이퍼파라미터(3번 entropy_coef)**. "hyperparameter tuning"으로 접근하면 안 되는
   상태 — plant와 보상 구조가 먼저다.

---

## 이 실험이 세운 진단 절차 (재사용용)

0. **학습 시작 전: 알려진 미해결 물리 이슈부터 재검증** (`play.py --zero-actions` PASS가
   게이트). exp_013은 "이전에 hover 통과"를 믿고 건너뛰었다가 43분 run 전체가 리셋 킥에
   오염된 채 진행됐다 — 그리고 그 버그는 이미 메모리에 기록되어 있었다.
1. 학습 로그에서 `noise_std` 추세 먼저 — 단조 상승이면 rollout 통계 신뢰 불가.
2. `Episode_Termination/*` 분포로 실패 모드 분해 (success/crash/max_alt/stagnation/timeout).
3. deterministic eval(수정된 `play.py --policy`)로 "정책 평균 vs 노이즈" 분리.
4. 지배 실패 모드에 대해 **보상 수지 계산**(finisher vs farmer) — 정책이 합리적으로
   그 행동을 선택한 것인지 확인. 착취 가능하면 하이퍼 튜닝 전에 보상 구조부터.

## 관련

- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] — 근거 데이터 전체
- [[errors/err_20260703_vision_env_origin_frame]] — 선행 버그(비전 사멸) 수정
- [[research/isaac_lab_reward_tuning]] — 각 필드의 위치/의미 레퍼런스
- [[research/isaac_lab_wandb_guide]] — 메트릭 해석 (noise_std 게이트 추가됨)
- [[research/rl_rules]] Rule 17·18 — 이 실험에서 규칙화된 발견

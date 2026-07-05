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
| **0** | **리셋 속도킥 위생 수정** (`_apply_body_mass_override` → 스폰타임 mass_props, 또는 init 직후 zero-wrench sim step 1회) | 런타임 `set_masses()` → **프로세스당 1회**, init 후 첫 물리 substep에 +8.4 m/s 킥 (07-04 계측 재정정: 매 리셋 아님 — [[isaac_mass_override_reset_bug]] 참조) | (원칙) `CRAZYFLIE_CFG.replace(...)` spawn cfg의 `mass_props`(UsdPhysics.MassAPI)로 질량 설정 — 단 **inertia는 native 유지**(x500 inertia까지 baking하면 회전 plant가 바뀌는 confound); (최소 diff) init 직후 zero-wrench step 1회로 질량 flush — `_diag_kick.py` t2first가 킥 전무를 실증 | 학습 오염은 env당 첫 에피소드뿐(~0.3%) → **max_alt 33%의 원인 아님**(그 rate는 iter ~200 창발 = 학습된 행동). 수정 가치: `--zero-actions` 게이트 복원 + 첫-에피소드 아티팩트 제거. **더 이상 1-3번의 선행 게이트가 아님** | plant 정상화라 fresh 권장 (1-3과 묶어서) |
| 1 | **analytic conf 거리 감쇠** (`DroneBombardVisionCfg` / `_update_vision`) | conf 0.73-0.95, 거리 무관 | slant range에 비례해 conf·탐지확률 감쇠 (예: `conf *= clamp(1.5 - slant/10, 0.1, 1)` 계열 — YOLO 캘리브레이션(`yolo_eval.py --calibrate`)으로 커브 확정이 정도) | 거리-무관 conf + 고도↑=centering 기하 유리 → **고도 상승 farming attractor. 07-04 재정정으로 max_alt 27-43%의 1차 가설로 복권**(킥 배제: 그 rate는 iter ~200 창발·지속 = 학습된 행동이고, 같은 구간 rew_vision 고유지). 실제 YOLO는 apparent size ∝ 1/거리(Rule 13) — parity 회복이 목적이므로 buff가 아니라 버그 수정에 가깝다 | ✅ 필수 |
| 2 | **`reward_success`** (`DroneBombardRewardCfg`) | 100 | **300** | farmer(+225) > finisher(+121) 수지 역전용. 300이면 finisher ≈ +321로 지배 전략 복원. Gazebo v14 final-approach stagnation과 동일 병인 | ✅ 필수 |
| 3 | **`entropy_coef`** (`agents/rsl_rl_ppo_cfg.py`) | 0.005 | **0.0** (또는 0.001 + noise_std 모니터링) | noise_std 0.8→3.92 폭주. 스무딩 액션 파이프라인(clip→rate_limit→LPF)이 노이즈를 필터링해 entropy 압력을 견제할 task 손실이 없음 | PPO라 buffer 오염은 없으나 위 1·2와 묶어 fresh 권장 |
| 4 | `num_steps_per_env` | 32 | 64 (선택) | 종단 보상 300 도입 시 GAE horizon(3.2s)이 평균 에피소드(~6s)의 절반 — credit assignment 여유 확보. 2차 실험에서 A/B | — |
| 5 | `w_vision_center` | 1.5 | 유지 (1번 적용 후 재평가) | 1번이 들어가면 원거리/고고도 farming 자체가 사라짐. 1번 없이 이것만 낮추면 비전 서보잉 신호까지 죽는다 — 순서 중요 | — |
| 6 | **스폰 고도** `spawn_alt_range` (사용자 질문 "initial height") | (9, 11) m | **유지** | v15 parity 값. 고도 자체는 실패 원인이 아님(실패는 보상 지형). 단 스폰 가시성이 63%(6-7m 스폰은 17%)이므로, 1번 적용 후 acquisition이 문제 되면 `handoff_dist_range` (3,7)→(3,6) 축소가 올바른 레버(고도 아님 — Rule 13 동일 원리) | — |
| 7 | `success_radius` | 0.8 | 유지 → success ≥70% 안정 후 0.5 커리큘럼 (`--resume`) | 기존 계획대로. 지금 조이면 4b 불균형이 악화만 됨 | 커리큘럼은 resume |
| 8 | `max_altitude` / `min_altitude` | 25 / 3 m | 유지 | max_alt 33%는 천장이 낮아서가 아니라 상승이 이득이라서다. 1번이 원인 제거. crash 27%(다이브)도 1-3 적용 후 재측정 — 원인 미확정 항목으로 이월 | — |

**적용 순서/묶음 (07-04 재정정 반영):** 0+1+2+3을 한 번에 fresh로
(exp_014 제안: `exp014_massfix_visionrange_succ300_ent0`) — 0은 더 이상 분리 측정이
필요한 후보가 아니라 위생 수정이므로(학습 오염 ~0.3% 실증) 묶어도 attribution이 안 깨진다.
원인별 분리 측정을 원하면 [[research/exp014_ablation_protocol]]의 arm 설계(1 단독 A/B)를 따를 것.
4는 exp_014 결과 보고 A/B. 5-8은 관찰 대기.

---

## 2026-07-05 실행 현황 (exp_014 세션)

- **#0 완료** (커밋 `cd0c617`): 스폰타임 `UsdPhysics.MassAPI` authoring + 로터스핀
  리셋 재주입 제거. `--zero-actions` **PASS** (0.18-0.22 m). ⚠️ **추가 대발견**: 구
  런타임 `set_inertias`는 사실 **전파되고 있었다** → exp_013은 rate loop ~1300×
  저토크 plant에서 학습됨. 수정판은 x500 inertia authoring + 컨트롤러 동일값 읽기로
  최초의 일관 plant (커밋 `7d0e9b6`, [[research/isaac_inertia_ctrl_mismatch]]).
  **구 체크포인트는 새 plant에서 무효** (Rule 19c).
- **#1 구현** (커밋 `5343e38`): slant-range conf 감쇠 `g=clamp(1-(slant-5)·0.1, 0.1, 1)`,
  `range_falloff_enabled` cfg 게이트. **실 YOLO 캘리브레이션은 컨테이너 annotator
  버그로 차단**(stock 카메라 예제도 동일 크래시) — 커브는 분석적 후보값이며
  `yolo_eval.py --calibrate` 하네스는 수리 완료(X-마커·카메라 spawn·스윕 하드닝),
  이미지 수정 후 재캘리브레이션 필수. 검증: [[experiments/exp_014_A2_visionrange]].
- **#2(reward_success)·#3(entropy_coef)는 사용자 지시로 이번 세션 불변** — 다음 페이즈.
- 신규 관측 도구: `Episode_Metric/action_sat_frac` (raw |a|>1 비율, 커밋 `7657df4`).

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

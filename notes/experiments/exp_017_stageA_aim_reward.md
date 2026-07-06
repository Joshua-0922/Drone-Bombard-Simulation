---
date: 2026-07-06
tags: [experiment, isaac-lab, ppo, reward-design, ccip, release]
status: done
type: experiment
wandb_run: 750gpldr (P1 base), 6z0gpnhy (Stage A v1), fv5qqmtz (Stage A v2)
---

# exp_017 — Stage A: 밀집 CCIP 조준오차 보상 (release_rate 개입 1차)

> [[research/ccip_aim_reward_stageA]]의 실행 기록. exp_016(Rule 21)이 확정한
> "근접 학습 정책 ≠ 릴리스 능력" 갭에 대한 **보상-변경-단독** 개입.
> 사용자 제약: 종단/성공 게이트·entropy_coef·비전 캘리브레이션 불변, DR/residual은
> Stage B로 이월(이번 런 전부 Phase 1: nominal 탄도, residual no-op).

## 1. 설정

- **코드**: `math_utils.aim_error_reward` + `RewardCfg.w_aim/aim_reward_scale`
  (기본 0.0=off) + `_aim_err_last` 배관 + train.py CLI. 유닛테스트 40/40,
  5-lens 적대 검증(상세: research 노트 §2).
- **warm-start 소스 확정**: exp_015는 스모크만(학습 체크포인트 없음), exp_014
  A2는 4-dim(HEAD 6-dim 로드 불가) → **P1 기준선을 HEAD에서 신규 학습**해
  warm-start 소스로 사용. obs 14-dim/action 6-dim 전 런 동일 → `runner.load()`
  무손실(iter 카운터 연속 실증: 399→799→1399).
- 공통: seed 42, num_envs 2048, L4, `isaac-verify` 컨테이너.
- 아티팩트(컨테이너, 각 run dir `model_final.pt` + 안정 경로 사본):
  - P1 기준선: `/workspace/logs/isaac_lab/drone_bombard/exp017_phase1_final.pt`
  - **Stage A v1 (차기 스테이지 warm-start 소스 권장 — 전 릴리스 지표 최우수):
    `.../exp017_stageA_final.pt`**
  - Stage A v2: `.../exp017_stageA2_final.pt` (v1 대비 회귀 — 기록용)
  - 호스트 백업: `/opt/drone-bombard/checkpoints/exp017/` (컨테이너 소실 대비)

| Run | wandb | w_aim | knee(m) | iters | warm-start |
|---|---|---|---|---|---|
| P1 base | 750gpldr | 0.0 | — | 0–399 | scratch |
| Stage A v1 | 6z0gpnhy | 1.0 | 0.5 | 399–799 | P1 final |
| Stage A v2 | fv5qqmtz | 2.0 | 1.0 | 799–1399 | v1 final |

## 2. 학습 내 추세 (50-iter 버킷, release_rate = 1차 지표)

**P1 base**: release_rate 0.122→0.093→0.061→0.048→0.044→0.046→0.037→0.038
(**단조 하락** — 근접 최적화가 릴리스 능력을 파괴), aim_err_min 0.80→1.19 m,
final_speed 1.87→2.87 m/s, success 0.72→0.99+.

**v1 (iters 400–799)**: release_rate ~0.032–0.034 평탄, aim_err_min ~1.21 평탄,
final_speed 3.0→3.35 상승, rew_aim 0.19→0.17/ep(미미). farm 시그니처 전무
(에피소드 길이 ~20 스텝 안정, timeout/stagnation 0). → 학습 내 지표는 σ~1.1
노이즈의 CCIP 증폭(×1.5 s)에 가려짐; deterministic eval이 판정 기준.

**v2 (iters 800–1399)**: release_rate 전 구간 ~0.027–0.036 평탄(개선 없음),
rew_aim ~1.39–1.42/ep(v1의 8× 소득이나 **행동 불변의 수동 소득**), final_speed
~3.38–3.43 고정, aim_err_min ~1.19–1.22 고정, **σ 1.18→1.55 폭등**. farm
시그니처 전무(에피소드 길이 19–20, timeout/stagnation 0, max_alt 0).

## 3. Deterministic 200-ep eval (전 지표)

| 지표 | P1 base | v1 (w=1, s=0.5) | v2 (w=2, s=1.0) |
|---|---|---|---|
| success (d_xy≤0.8) | 100.00% | 100.00% | 100.00% |
| **release_rate@0.2m** | **2.50%** (5/200) | **5.50%** (11/200) | **3.50%** (7/200) |
| drop_impact_error@release (mean) | 0.112 m | 0.138 m | 0.150 m |
| aim_err_min med (mean) | 1.146 (1.222) m | **0.889 (1.028) m** | 1.096 (1.176) m |
| final_speed_xy (mean) | 3.354 m/s | **2.721 m/s** | 3.446 m/s |
| d_xy_min (mean) | 0.650 m | 0.684 m | 0.659 m |

## 4. 판정

**(b) — 정체.** v1이 유일한 실질 이동(2.2× 자체 기준선, 단 n=200에서 release_rate
차이는 p≈0.13; aim_err/speed 분포 이동은 실재), v2는 회귀. 어떤 변형도 exp_016
6% 참조선을 유의하게 상회하지 못함. 릴리스 능력은 dense 사이드 보상이 아니라
**릴리스-종단 구조(Phase 2)**로 학습할 것 — Rule 22.

상세 분석·구조 원인·Stage B 권고: [[research/ccip_aim_reward_stageA]] §4–5.

상세 분석·구조 원인·Stage B 권고: [[research/ccip_aim_reward_stageA]] §4–5.

## 관련
- [[research/ccip_aim_reward_stageA]] ←→ 본 노트
- [[experiments/exp_016_ccip_release_reeval]] — 문제 확정(6% 기준선)
- [[experiments/exp_015_phased_curriculum]] — Phase 2 구조(다음 스테이지)
- [[experiments/training_history]]

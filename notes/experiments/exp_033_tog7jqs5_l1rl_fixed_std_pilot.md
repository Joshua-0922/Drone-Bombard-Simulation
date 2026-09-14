---
date: 2026-09-14
tags: [experiment, residual, l1, rl, ppo, pilot, fixed-std, running]
status: running
type: experiment
wandb_run: "tog7jqs5 (2026-09-14_00-55_pilot_fixed)"
---

# exp_033 — L1-RL 파일럿 (탐험 폭 고정): 탐험을 지키면 잔차가 드리프트를 찾는가

> **exp_032의 처방 하나만 바꾼다.** 0 초기화 잔차 PPO에서 탐험 std를 학습 대상에서 빼고 **0.05에 고정**
> (`--residual_fixed_std`, `policy.std.requires_grad_(False)`). 나머지는 exp_032와 동일
> (동결 L0 seed 1, 38채널, scale 2.0, EMA 0.3, nominal std 0.01, entropy 0, DR 1.5, 18–22 m).
> 사용자 결정: 1000 iter, 100 iter마다 보고, 안 되면 중단. 이것도 안 되면 잔차 RL은 제어기 쪽(행동 공간)으로.

관련: [[experiments/exp_032_icpj8p4r_l1rl_zero_pilot]] · [[research/residual_rl_exploration_collapse]] §5 ·
[[research/l1_rl_preflight]] · [[research/rl_rules]] (Rule 29·38·44) · [[experiments/training_history]]

## 1. 무엇이 다른가 — 한 줄

exp_032에서 학습 가능한 std는 80 iter 만에 0.05 → 0.01로 붙었고 그 뒤 평균 잔차가 움직일 신호가 없었다
(참 드리프트 대비 $R^2 \approx 0$, cos ≈ 0.1, 300 iter). 고정하면 매 롤아웃 게이트 비용(성공률 약 80%)을
지불하는 대신 평균의 그래디언트가 살아 있다. 대가는 **학습 중에만** 낸다 — 평가는 결정론적이다.

## 2. 판정 (사전 등록)

- **진행 중 판정** (100 iter마다): `_probe_rl_residual.py`로 최신 체크포인트의 잔차 출력을 참 드리프트와 대조 —
  $R^2$·cos가 0에서 떨어지기 시작하는가, |출력|이 0.1 m를 넘어 자라는가. 300 iter까지 둘 다 0 근처면 중단(Rule 29).
- **최종 판정**: model_final paired n=600 (seed 3000/4000/5000 × DR 1.5) + DR 2.5 + 사거리 26–30.
  기준 L1-SL gi CEP50 0.209 / succ@0.5 89.67% / CEP90 0.411 (Rule 40: 세 지표 함께).
- 중단 서명: NaN·Traceback, crash 급증, 롤아웃 성공률이 80%에서 계속 내려감(잔차가 게이트를 더 망가뜨림).

## 3. 결과

(진행 중, 00:56 착수 — iteration 0/1 롤아웃 succ@1.0 79.1 / 68.6%, std 0.02 평균(잔차 0.05 고정))

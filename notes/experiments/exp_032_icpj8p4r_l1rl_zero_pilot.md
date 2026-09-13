---
date: 2026-09-13
tags: [experiment, residual, l1, rl, ppo, pilot, running]
status: running
type: experiment
wandb_run: "icpj8p4r (2026-09-13_15-27-10_pilot_zero)"
---

# exp_032 — L1-RL 파일럿 (0 초기화): 종단 보상만으로 드리프트를 찾는가

> **질문 하나.** L1-SL gi(지도 잔차, CEP50 0.209 / −31.6%)와 **학습 신호만 다른** 팔을 PPO로 돌린다.
> 정보(38채널)·권한(2.0 m)·평활(EMA 0.3)·비행(동결 L0 seed 1)·평가 시드가 전부 같다.
> SL은 다시 하지 않는다 — 비교 대상은 [[experiments/exp_030_gain_invariant_residual]]의 기존 표다.

관련: [[research/l1_rl_preflight]] · [[research/residual_ceiling]] §7.3 ·
[[research/research_architecture]] §7.6.6·§9.1 · [[research/release_gate_jitter]] ·
[[research/rl_rules]] (Rule 29·38·40·43) · [[experiments/training_history]]

---

## 1. 설정

### 1.0 L1-SL gi가 적합된 조건과의 정합 (사용자 요청으로 재확인)

컨테이너 `/tmp/sl_collect.sh`·[[experiments/exp_028_l1_sl_pilot]] §2·[[experiments/exp_030_gain_invariant_residual]]:
`res_gi.pt`는 **동결 L0 seed 1**을 `--no_residual`(결정론적 추론)로 굴려 **DR 1.5 · 사거리 18–22 m**에서
모은 덤프(seed 1000·2000, 256 envs, 3,501 슬롯-에피소드)로만 적합했다. 아래 파일럿은 같은 정책·같은 DR·
같은 사거리·결정론적 비행 분포(`--nominal_std 0.01`)이므로 **조건이 일치**한다. 다른 것은 학습 신호(PPO on
종단 보상 vs MSE on 드리프트)와 데이터 양(PPO는 iteration당 약 2,000 에피소드)뿐이다.

> 15:23에 500 iter로 먼저 걸었던 run(`ac0mhdof`)은 6분 만에 중단하고 1000 iter로 재착수했다(사용자 결정).

| 항목 | 값 |
|---|---|
| 액터 | **동결 L0 seed 1** (`2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt`, 행 0:5) + **잔차 트렁크** 38→128→128→2 ELU (행 5:7), 출력층 0 초기화 |
| 관측 | 26 + tilt 누적 10 + gi 2 = **38** (`--accum_obs`, env 내부, `play.py` 누적과 비트 동일) |
| 잔차 권한 / 평활 | `residual.scale` **2.0 m** / env EMA **α=0.3** (L1-SL과 동일) |
| 롤아웃 std | nominal **0.01** (결정론적 비행 분포) · 잔차 **0.05** (0.1 m, EMA 후 ≈0.04 m) |
| PPO | rsl_rl 3.1.2, 2048 envs × 96 steps, lr 3e-4 adaptive(KL 0.01), **entropy_coef 0.0**, 크리틱 L0 warm-start(입력 0열 확장), 새 Adam |
| 보상 | 현행 task reward, **조준 포텐셜 nominal-only**(09-13 수정), `w_residual` 0 |
| DR | `model_err.scale` 1.5, 사거리 18–22 m |
| 반복 | **1000 iter**, seed 1, 약 17 s/iter → ≈4.8 h (L0 자체 학습과 같은 반복 수) |
| 명령 | `_l1rl_pilot.sh` (`ARMS=zero ITERS=1000`), 로그 컨테이너 `/tmp/l1rl/train_zero.log` |

**평가** (학습 종료 후 자동): `play.py --paired_eval --accum_obs --residual_scale 2.0 --residual_ema 0.3`,
200 ep × seed {3000,4000,5000} × {DR 1.5 · DR 2.5 · 사거리 26–30}. 산출물 `/tmp/l1rl/L1RL_zero_*.json`.
비교 표는 `_agg_table1.py`로 기존 `/tmp/sl_GI` · `/tmp/sl_R` JSON과 합친다(시드셋 assert).

## 2. 판정 기준 (사전 등록)

| 지표 | L0 (n=600) | **L1-SL gi** | 오라클 | L1-RL zero |
|---|---|---|---|---|
| CEP50 | 0.305 | **0.209** | 0.192 | ? |
| CEP90 | 0.593 | **0.411** | 0.392 | ? |
| succ@0.5 | 78.83% | **89.67%** | 90.83% | ? |
| succ@1.0 | 93.83% | 94.50% | 94.33% | ? |

- L1-SL과 동률이면 *"종단 보상은 드리프트 예측을 재발견한다"*, 못 미치면 *"잔차의 최적값은 드리프트 예측이고
  RL은 그것을 종단 보상만으로는 못 찾는다"* — 둘 다 발표 가능한 결과다.
- Rule 40: CEP50 하나로 판정하지 않는다. Rule 29: 롤아웃 성공률 평탄 + 잔차 std 상승 200 iter면 중단.
  잔차 std가 0.1(action)을 넘으면 게이트가 무너지므로 그 자체가 중단 신호.

## 3. 결과

(진행 중, 15:27 착수 — iteration 0/1 롤아웃 succ@1.0 79.1 / 80.5%, 잔차 std 0.05, ETA 약 20:20 평가 포함)

---
date: 2026-09-25
tags: [research, reference, code-map, wind, gru, rl]
status: active
type: reference
---

# 코드 지도 — 바람 모델 · 강화학습(비행) · 지도학습(잔차) · 평가가 어느 파일에 있나

> 파이프라인 설명은 [[research/l1_sl_pipeline]], 표·그림은 [[research/final_tables_v6]], 개요는 [[research/paper_outline_v6]].
> 이 문서는 "그 코드가 어디 있나"만 답한다. 줄 번호는 2026-09-25 기준.

## τ(타우)가 무엇인가

**바람의 상관 시간**(초). 바람이 지금 값을 얼마나 오래 유지하는지의 척도로, 상관이 $e^{-\Delta/\tau}$로 줄어든다.
τ = ∞(코드에서는 `wind_tau_s = 0`)이면 에피소드 내내 한 값(학습 조건), τ = 3 s면 약 3초 뒤 상관이 37%로 떨어진다.
낙하 시간(약 0.85 s)보다 짧으면 놓는 순간의 바람으로 착지를 맞힐 수 없다(오라클도 짐). 저고도 10 m·풍속 5 m/s의 실제 수평 돌풍은 τ ≈ 13 s.

## 1. 바람 모델 — `isaac_lab/drone_bombard/drone_bombard_env.py`

| 무엇 | 위치 | 모델 |
|---|---|---|
| 설정 필드 | `ModelErrCfg` L312~340: `wind_std`(1.5 m/s, 축당), `wind_max`(5 m/s 상한), `scale`(DR 노브, 학습 1.5), `wind_tau_s`, `wind_gust_frac` | |
| **정상 바람** (기본, 모든 표의 전제) | `_reset_idx` L2142 → `mdp/domain_rand.py:49 sample_wind_capped` | 에피소드마다 한 번 $N(0, \text{wind\_std}\cdot\text{scale})$ 2축, 크기 상한 `wind_max·scale`, 에피소드 내 고정. 수평·공간 균일 |
| **돌풍만 (스트레스 사다리)** `wind_tau_s > 0`, `wind_gust_frac = 0` | `_step_wind_ou` L1160, 매 물리 스텝(0.01 s) 호출 L1154 | 평균 0 OU: $w \leftarrow w\,e^{-dt/\tau} + \sigma\sqrt{1-e^{-2dt/\tau}}\,\varepsilon$, 정상분포는 정상 바람과 동일. 평균풍 없음 → 현실보다 가혹 |
| **현실 바람** `wind_gust_frac > 0` (exp_037) | 같은 함수, `_wind_mean` + `_wind_gust` | 에피소드 평균풍(정상 샘플러) + OU 돌풍(σ = frac × wind_std × scale, 상관 τ). Dryden 종방향 돌풍 구조 |
| 바람이 물리에 들어가는 곳 | 기체 항력 L1389 부근(`v_air = wind − v`), 페이로드 낙하 `_step_payload_physics` | |
| 평가 스위치 | `play.py --wind_tau T --wind_gust FRAC --dr_scale S` (L47, L53 부근) | |

## 2. 강화학습 — 비행 정책 L0

| 무엇 | 위치 |
|---|---|
| 학습 스크립트 | `isaac_lab/train.py` (`main` L457, rsl_rl `OnPolicyRunner` L583). 명령: `train.py --task_env --no_residual --dr_scale 1.5 --num_envs 2048 --max_iterations 1000` |
| PPO 하이퍼파라미터 | `isaac_lab/drone_bombard/agents/rsl_rl_ppo_cfg.py` L18 (`num_steps_per_env` 96, actor/critic [256,256] ELU, lr 3e-4 adaptive, entropy 0.005) |
| 환경(MDP) | `isaac_lab/drone_bombard/task_env.py`: 관측 `_get_observations` L832, 보상 `_get_rewards` L1089, 릴리즈 게이트 `_resolve_release` L794, 착탄점 예측 `_ccip` L695 |
| 보상 상수 | `task_env.py` `TaskRewardCfg` (L263~) — 설명은 [[research/reward_design]] |
| 잔차 RL(불채택, ablation 기록용) | `train.py --residual_net`, `drone_bombard/residual_actor.py`, `_l1rl_pilot.sh` — [[experiments/exp_032_icpj8p4r_l1rl_zero_pilot]] |
| 체크포인트 | 컨테이너 `/tmp/l0b/logs/drone_bombard_ppo/2026-08-30_03-50-27_task_dr1.5_nores/model_final.pt` (seed 1) |

## 3. 지도학습 — 잔차 GRU (본 방법 GRU-S)

| 단계 | 위치 |
|---|---|
| ① 데이터 수집 | `play.py --dump_sl` (`_sl_row`/`_sl_save`, L510 부근): 동결 L0 비행, 스텝마다 관측 26 + 정답 드리프트 2 + 원 상태 |
| ① 정답 정의 | `drone_bombard_env.py` `oracle_impact_residual(wind_only=True)` L1242 → `math_utils.integrate_payload_impact` L167 (낙하 ODE 적분) |
| ③ 학습 | `isaac_lab/_fit_sl_seq.py`: `Filter`(GRU 26→64→2) L160, `loss_fn`(MSE; `--nll`면 가우시안 NLL) L176, `realised_drift`(`--label realised`) L68, `Exported`(GRUCell 한 스텝) L245. **본 방법 명령:** `_fit_sl_seq.py v2_s1a.npz v2_s1b.npz --export res_gru_S.pt` (정상 데이터, MSE, 분산 없음) |
| ③ (구 방법, ablation) | `_fit_sl_residual.py`: 수제 누적 특징 + MLP 38→128→128→2 |
| ⑤ 주입 | `play.py` `_SLResidual` L431 (recurrent 모듈이면 env별 hidden 유지, EMA `--sl_ema 0.3`) → 행동 채널 5:7 → `task_env._ccip` L695 (예측 착탄점 = 공식 + 2 m × 채널) |
| 회귀기 파일 | 컨테이너 `/tmp/sl/res_gru_S.pt`(본 방법) · `res_gru_{R,I,U,Urel2,Urel4}.pt`(변형) · `res_gi.pt`(수제 MLP) |

## 4. 평가·표·그림

| 무엇 | 위치 |
|---|---|
| 팔 평가 (paired, 3 seed) | `play.py --paired_eval --episodes 200 --num_envs 200 --seed {3000,4000,5000}`; 스크립트 `_gru_eval.sh`(τ 사다리), `_gust_eval.sh`(현실 A·B), `_grus_final.sh`(DR 스윕·미지 사거리), `_gi_headline.sh`·`_t_reseed.sh`(기존 팔) |
| 규칙 팔 T0/T2 | `baseline_drop.py` (`--arm hover/argmin`, 투하 고도 3.5 m) |
| 표 | `_agg_table1.py "라벨=글롭"…` (시드셋 assert) → [[research/final_tables_v6]] |
| 그림 | `_fig_final.py --out /tmp/figs` → `notes/figures/fig{1,2,3}.png` |
| 결과 JSON | 컨테이너 `/tmp/sl_GI`(정상), `/tmp/gru`(τ 사다리), `/tmp/gust`(현실 바람), `/tmp/dr_axis`(DR 스윕), `/tmp/sl_R`(미지 사거리), `/tmp/ou`(L0·오라클 τ 사다리) |

> 실제 실행 사본은 컨테이너 `isaac-verify:/tmp/rebuild`이며, repo `isaac_lab/`을 수정하면 `docker cp isaac_lab/. isaac-verify:/tmp/rebuild/`로 동기화해야 한다.

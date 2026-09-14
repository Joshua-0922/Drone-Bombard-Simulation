---
date: 2026-09-14
tags: [experiment, residual, supervised, l1, gru, temporal-filter, ou-wind, running]
status: running
type: experiment
wandb_run: (없음 — PPO 미사용, 오프라인 회귀 + 주입 평가)
---

# exp_034 — 누적 특징을 학습된 시간 필터(GRU)로 대체: 정상 바람에서 같은가, 시변 바람에서 나은가

> **한 줄.** L1-SL의 손 설계 부분(tilt 누적 10 + gi 2)을 원 관측 26채널 위의 GRU(hidden 64)로 바꾼다.
> 물음은 둘: ① 정상 바람에서 수제 누적과 같은 성능이 나오는가(같아야 대체 가능), ② τ 혼합 데이터로 학습하면
> OU 바람(τ = 10·3·1 s)에서 수제 누적이 잃는 이득을 되찾는가. 참조 상한은 오라클+EMA(τ ≤ 3에서 그것도 진다 — Rule 42).

관련: [[research/l1_sl_pipeline]] · [[research/residual_wind_stationarity]] (Rule 42) · [[research/release_gate_jitter]] (Rule 38) ·
[[experiments/exp_031_ou_wind]] (OU 사다리 참조값) · [[experiments/exp_030_gain_invariant_residual]] · [[experiments/training_history]]

## 1. 설정

| 항목 | 값 |
|---|---|
| 모델 | `nn.GRU(26, 64)` + Linear(64→2), 약 1.8만 파라미터. 입력 = 원 관측 26(표준화), 누적 특징 없음 |
| 학습 | 에피소드 단위 시퀀스, 화물 부착 프레임 마스크 MSE, Adam 1e-3, wd 1e-4, 코사인, 150 epoch, 배치 256 시퀀스, grad clip 1 |
| 라벨 | 기존과 동일 — `oracle_impact_residual(wind_only=True)` (릴리즈 순간 바람 고정 적분, 미터) |
| 분할 | 에피소드 단위 2:1 (`_fit_sl_seq.py`) |
| 내보내기 | TorchScript, `GRUCell` 단일 스텝 `forward(x, h) -> (d, h)`, `recurrent=True`. `play.py::_SLResidual`이 env마다 상태 유지, 에피소드 시작 시 0 |
| 주입 | `--sl_residual res_gru_*.pt --sl_ema 0.3`, scale 2.0 — res_gi.pt와 동일 경로 |
| 데이터 | **S** = 정상 덤프(s1a·s1b, 3,411 ep) / **M** = S + OU 덤프 τ 10·3·1 (각 750 ep, 수집 시드 2000·1300·1100) |
| 팔 | GRU-S(정상만 학습) · GRU-M(혼합) · MLP38-M(수제 누적 + gi, 혼합 학습 — "모델인가 데이터인가" ablation) |
| 평가 | 동결 L0 seed 1, DR 1.5, 18–22 m, seed {3000,4000,5000} × τ {∞, 10, 3, 1, 0.3} paired 200 ep. 참조: `/tmp/sl_GI`, `/tmp/ou` |

## 2. 판정 (사전 등록)

- ① 정상: GRU-S CEP50이 L1-SL gi(0.209, n=600)와 ±0.01 이내면 "대체 가능". CEP90·succ@0.5도 함께(Rule 40).
- ② OU: GRU-M이 τ = 10·3에서 L0(0.272·0.282)보다 낫고 gi(0.263·0.292)보다 나으면 "시변 바람 회복". τ ≤ 1은 오라클도 지므로 기대하지 않는다.
- MLP38-M이 GRU-M과 같으면 이득은 **데이터(τ 혼합)** 때문이고, GRU-M만 나으면 **모델(학습된 창)** 때문이다.

## 3. 결과

### 3.1 오프라인 (GRU-S, 홀드아웃 1,136 ep)

| | $R^2$ 부착 프레임 | $R^2$ 릴리즈 직전 10 | 예측 스텝 변화 |
|---|---|---|---|
| GRU-S (원 관측 26) | 0.639 | 0.822 | 0.083 m/step |
| MLP38 gi (참조, 같은 프로토콜) | 0.668 | — | 0.07~0.13 |

### 3.2 주입, seed 3000 (첫 판정)

| τ | L0 | L1-SL gi | **GRU-S** | 오라클+EMA |
|---|---|---|---|---|
| ∞ | 0.296 / 0.534 | 0.225 / 0.386 | **0.217 / 0.388** | 0.207 / 0.419 |
| 3 s | 0.295 / 0.643 | 0.300 / 0.612 | **0.297 / 0.572** | 0.333 / 1.187 |

(CEP50 / CEP90) 정상 바람에서 수제 누적 없이 같은 성능. τ=3에서는 아직 이득 없음(OU 데이터 미학습), 손해도 없음.

(3 seed × τ 사다리, GRU-M, MLP38-M — 진행 중)

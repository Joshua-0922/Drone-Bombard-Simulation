---
date: 2026-09-24
tags: [experiment, residual, supervised, l1, gru, label, ou-wind, running]
status: running
type: experiment
wandb_run: (없음 — 오프라인 회귀 + 주입 평가)
---

# exp_035 — 실현 라벨: 정답을 "지금 바람"에서 "낙하 중 실제 바람 적분"으로

> **한 줄.** exp_034의 결론(시변 바람의 병목은 라벨, Rule 45)에 대한 처방. 입력(관측 26)·모델(GRU 64)·데이터(정상 + τ 10·3·1)는
> 그대로 두고 **라벨만** 바꾼다. 순간 라벨 $\delta^{\text{순간}}_t$ = 바람 $w_t$를 고정하고 적분한 드리프트 →
> 실현 라벨 $\delta^{\text{실현}}_t$ = 프레임 t에서 놓았을 때 **그 뒤 실제로 기록된 바람열**로 적분한 드리프트(반사실, 전 프레임).
> MSE 최적해가 "낙하 적분의 조건부 기대값"이 되므로, 상관이 있는 만큼만 예보하고 없는 만큼은 0으로 수축한다.

관련: [[experiments/exp_034_learned_temporal_filter]] · [[research/residual_wind_stationarity]] (Rule 42·45) ·
[[research/l1_sl_pipeline]] · [[experiments/training_history]]

## 1. 설정

| 항목 | 값 |
|---|---|
| 라벨 | `--label realised`: `drift_instant + (integrate(wind_seq) − integrate(w_t))` — 같은 적분기(`integrate_payload_impact`, `wind_seq` 경로 신설), 지연 0.22 s 오프셋, 10 Hz 바람 0차 홀드, 공칭 지연·탄도계수 (오라클 wind-only와 동일 정의) |
| 덤프 | `play.py --dump_sl`에 원 상태(pos_xy·vel_xy·alt·vz)와 상수(지연·마운트·지면·k/m·g·dt) 추가 → v2 재수집: 정상 s1a·s1b(각 1500 ep), τ 10·3·1(각 750 ep) |
| 모델·학습 | GRU 26→64→2, `_fit_sl_seq.py`, **1000 epoch**, 100 epoch마다 보고(홀드아웃 $R^2$ 부착/릴리즈, 파일별 $R^2$, 예측 스텝 변화) |
| 대조 | 같은 v2 혼합 데이터 + 순간 라벨, 같은 1000 epoch (GRU-I) — 라벨 효과만 분리 |
| 판정 | τ 사다리 × 3 seed 주입(EMA 0.3, scale 2.0). 사전 등록: 정상 바람 손실 없음(gi 0.209 ± 0.01), τ 3~10 회복(gi 대비), τ ≤ 1 무해(L0 ± 0.01) |

**예측 가능 비율** $f(\tau) = \frac{\tau}{T}(1 - e^{-T/\tau})$, $T \approx 0.85$ s: τ=10 → 0.96, 3 → 0.87, 1 → 0.67, 0.3 → 0.33.
실현 라벨의 홀드아웃 $R^2$는 τ별로 이 근처($f^2$ × 정상 바람 $R^2$)가 상한이다.

## 2. 경과

(진행 중)

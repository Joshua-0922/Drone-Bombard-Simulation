---
date: 2026-09-14
updated: 2026-09-25
tags: [research, residual, supervised, l1, pipeline, overview, gru, ema]
status: active
type: research
---

# L1-SL 파이프라인 — 본 방법(L0 + GRU-S + EMA)이 무엇을, 어떤 입력으로, 어디에 넣는가

> **한 줄.** 비행 정책(L0)은 그대로 두고, "공식이 예측한 착탄점과 실제 착탄점의 차이(드리프트)"를 **관측 이력만 보고** 맞히는
> 작은 GRU(26 → 64 → 2)를 오프라인 지도학습으로 따로 학습해, 그 예측값만큼 예측 착탄점을 옮긴다. 바람 센서는 없다.
> 알맹이는 **정답의 정의**(①), **이력 위의 시간 필터**(③), **게이트 앞 EMA**(⑤) 셋이다. 09-25 기준 최종 구성이며, 표·그림은 [[research/final_tables_v6]].

관련: [[research/residual_ceiling]] §7 (왜 지도학습인가) · [[research/residual_observability]] (관측에 바람 정보가 있는가) ·
[[research/release_gate_jitter]] (EMA, Rule 38) · [[research/residual_wind_stationarity]] (시변 바람의 한계) ·
[[research/residual_rl_exploration_collapse]] (RL이 안 되는 이유, Rule 44) · [[research/code_map]] (파일·줄 위치)

---

## 1. 전체 흐름 — 5단계

```
① 수집      동결 L0가 잔차 없이 비행 ─ 0.1 s마다 (관측 26, 정답 드리프트 2, 원 상태 6) 저장
② 입력      관측 26 그대로 (손으로 만든 특징 없음)
③ 학습      표준화 → GRU 26→64 → 선형 2 → MSE (부착 프레임), 에피소드 단위 분할
④ 내보내기  표준화 포함 TorchScript res_gru_S.pt  (GRUCell 한 스텝: (x, h) → (δ̂, h'))
⑤ 주입      매 스텝: 은닉 상태 갱신 → δ̂ → EMA α=0.3 → /2 m → ±1 → 행동 채널 5:7
            env: 예측 착탄점 = 공식 CCIP + 2 m × 채널 → (a) 게이트·첫 교차(언제 놓나) (b) L0의 조준 오차 관측(조향)
```

### ① 데이터 수집 — `play.py --dump_sl`

| 항목 | 값 |
|---|---|
| 비행 | 동결 L0 seed 1, `--no_residual`(결정론), DR 1.5, 사거리 18–22 m, 256 envs |
| 덤프 | `v2_s1a.npz`·`v2_s1b.npz`(정상 바람, 수집 시드 1000·2000, 3,411 에피소드, 약 17만 프레임) |
| 저장 항목 | 관측 26 · **정답 드리프트 2(미터)** · 원 상태 6(위치·속도·고도·vz) · 참 바람 · 부착/탐지 플래그 · 상수 |
| 정답 정의 | `oracle_impact_residual(wind_only=True)`: **이 순간 놓으면 실제 떨어질 자리**(참 바람으로 낙하 ODE 적분, 지연·탄도계수는 공칭) **− 공식 예측 자리** |

참 바람은 **정답을 만들 때만** 쓰이고 입력에는 들어가지 않는다(teacher–student). 실기에서는 "던진 뒤 예측 vs 실측"으로 같은 라벨이 사후에 생긴다.

### ② 입력 — 관측 26 그대로

표적 상대위치·CCIP 오차·착탄 오차 크기(5, 탐지 전 0) · 고도·속도·자세·각속도·낙하시간·남은 시간(16) · 직전 속도 명령(4) · 탐지 플래그(1).
바람은 기체의 기울기·속도 추종 편차에 흔적을 남기지만 순간값은 가속 성분과 섞여 있다. 예전 방법은 에피소드 평균(tilt 누적 10 + gi 2)을 손으로 만들어 넣었고,
GRU는 같은 정보를 은닉 상태로 스스로 요약한다(정상 바람에서 0.204 vs 0.209, [[experiments/exp_034_learned_temporal_filter]]).

### ③ 학습 — `_fit_sl_seq.py v2_s1a.npz v2_s1b.npz --export res_gru_S.pt`

| 항목 | 값 |
|---|---|
| 망 | GRU(26 → 64) + 선형(64 → 2), 약 1.8만 파라미터 |
| 입력 처리 | 학습 프레임의 평균·표준편차로 표준화(가중치와 함께 내보냄) |
| 손실 / 최적화 | MSE(화물 부착 프레임만) / Adam lr 1e-3, weight decay 1e-4, 코사인 감쇠, 150 epoch, 배치 256 시퀀스, grad clip 1 |
| 분할 | **에피소드 단위** 2:1 — 같은 에피소드 프레임은 같은 바람이라 섞으면 정답이 샌다 |
| 결과 | 홀드아웃 $R^2$ 부착 프레임 0.64 · 릴리즈 직전 0.82 |

### ④ 내보내기 — `Exported`: GRUCell 한 스텝 `(x, h) → (δ̂ [m], h')`, 표준화 내장.

### ⑤ 주입 — `play.py --sl_residual res_gru_S.pt --sl_ema 0.3`

매 0.1 s: 관측 → GRU 한 스텝(env별 은닉 상태, 에피소드 시작 시 0) → $\delta_t$ (m) →
**EMA** $\hat\delta_t = 0.3\,\delta_t + 0.7\,\hat\delta_{t-1}$ → $\hat\delta / 2\,\text{m}$ → clamp ±1 → 행동 채널 5:7.
환경(`task_env._ccip`): 예측 착탄점 = 공식 CCIP + 2 m × 채널. 이 점이
(a) **릴리즈 게이트·첫 교차 판정**에 들어가 "언제 놓을지"를 정하고,
(b) L0가 읽는 **조준 오차 채널 3개**를 채워 L0가 그 점을 표적에 맞추도록 조향한다. L0의 가중치는 바뀌지 않는다.

**EMA를 거는 이유와 α.** 첫 교차 게이트는 예측이 스텝마다 흔들리면 요동의 극단값을 잡아 계통적으로 일찍 놓는다(Rule 38). EMA는 크기를
줄이는 것이 아니라 스텝 간 요동을 누르는 시간 평균이다. α는 민감도 스윕([[experiments/exp_041_ema_alpha_sweep]])에서 평탄 구간 [0.1, 0.4]의
지연이 가장 짧은 값(시간 상수 0.28 s)으로 정했다. 필터를 빼면 CEP50 +17%, CEP90 +92%.

---

## 2. 성능 (정상 바람, DR 1.5, n=600 paired, seed 3000·4000·5000)

| 팔 | CEP50 | CEP90 | succ@0.5 |
|---|---|---|---|
| L0 (잔차 없음) | 0.305 | 0.596 | 78.8% |
| L0 + 수제 특징 MLP 38 (구 방법) | 0.209 | 0.411 | 89.7% |
| **L0 + GRU-S + EMA 0.3 (본 방법)** | **0.204** | **0.403** | **90.8%** |
| 오라클 (참 바람을 앎) | 0.188 | — | — |

현실 돌풍(평균풍 + 20~30% 돌풍, τ 10·3 s)에서도 L0 대비 −33% / −28%로 이득이 유지된다([[experiments/exp_037_relative_shrink_realistic_wind]]).
낙하 시간(0.85 s)보다 짧은 상관의 돌풍만 남긴 스트레스 조건에서는 오라클도 지며, 본 방법은 L0 수준으로 물러선다(Rule 46).

---

## 3. 시도했고 채택하지 않은 것 (논문에는 넣지 않음, 기록만)

| 시도 | 결과 | 노트 |
|---|---|---|
| 잔차를 PPO로 학습 (L1-RL) | 탐험 std 소멸 또는 반대 방향 성장 | [[experiments/exp_032_icpj8p4r_l1rl_zero_pilot]] · [[experiments/exp_033_tog7jqs5_l1rl_fixed_std_pilot]] |
| OU 바람 데이터를 학습에 섞기 | 순간 라벨이 병목, 더 나빠짐 | [[experiments/exp_034_learned_temporal_filter]] (Rule 45) |
| 실현 라벨(낙하 중 실제 바람 적분) | 스트레스 τ에서 개선, 현실 조건에서는 차이 없음 | [[experiments/exp_035_realised_label]] |
| 평균 + 분산 헤드, 불확실하면 수축 | 스트레스 손해 절반, 현실 조건 동률 | [[experiments/exp_036_uncertainty_head]] · [[experiments/exp_037_relative_shrink_realistic_wind]] |
| 시간 일관성 손실로 EMA 대체 | 12조건 전부 동률 → 단순한 EMA 유지 | [[experiments/exp_038_gate_aware_stage1]] · [[experiments/exp_040_consistency_final]] |
| 게이트를 손실에 직접 결합 | 악화 또는 유의차 없음 | [[experiments/exp_039_gate_aware_stage23]] |

---

## 4. 제어기 의존성 — sim2real 대비 ([[research/residual_observability]] §7)

시뮬레이터 속도 루프는 순수 P. PID(PX4)면 "명령 − 실제 속도" 흔적이 사라진다. 그 4채널을 빼고 재적합해도 $R^2$ 0.668 → 0.657(MLP 기준).
적합의 본체는 **자세**(힘 균형, 어떤 제어기든 존재)다. 다른 제어기에서는 회귀기를 그 비행 데이터로 **재적합**하면 된다(수 분).
비싼 것은 L0의 재학습이지 잔차가 아니다. EMA α도 배포 쪽 손잡이라 재학습 없이 현장에서 다시 맞출 수 있다.

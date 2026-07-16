---
date: 2026-04-14
tags: [research, RL, rules, debugging, wandb, checkpoint]
status: active
type: research
---

# RL 실험 & 디버깅 규칙

> CLAUDE.md에서 이전. 학습 실행 전 반드시 확인.

---

## Rule 1 — Fail-Fast 디버깅

코드 변경 후 장기 학습(>10K steps) **즉시 시작 금지**.

1. 짧은 dry-run 먼저 (2–3 에피소드, `num_envs=1`)
2. WandB 정상 로깅 확인
3. ODE 크래시/PX4 타임아웃 없음 확인
4. 이후 full training 진행

---

## Rule 2 — 시뮬레이터 안정성 & 병렬화

- `SubprocVecEnv` 시 인스턴스 간 `time.sleep(15)` 스태거 필수
- **ODE AABB 크래시 주의:** 페이로드 자유낙하 없도록 스폰 고도 확인 → [[errors/err_20260319_ode_aabb_crash]]
- `num_envs=4` 반복 크래시 → `num_envs=1` 폴백
- **현재 결론:** `num_envs=1` 고정 (단일 Gazebo가 PX4 lockstep 직렬화)

---

## Rule 3 — 평가 & 논문용 데이터

- `rollout/ep_rew_mean` **단독 사용 금지** (탐색 노이즈 포함)
- `EvalCallback` 필수 (10K steps마다, deterministic 환경)
- `best_model.zip` 자동 저장 (eval 환경 기준)
- WandB 필수 커스텀 메트릭:
  - `custom/success_rate`: 0.5 m 이내 투하 비율
  - `custom/final_distance`: 평균 착탄 오차 (m)
  - `custom/timeout_rate`: 500-step 타임아웃 비율

---

## Rule 4 — 보상 함수 설계

**거리 기반 보상 구현 전 반드시 확인:**

$$e^{-k_1 d_{max}} > 10^{-4}$$

- $k_1=1.0$, $d=45\,\text{m}$: $e^{-45} \approx 6.5 \times 10^{-20}$ → machine zero → **학습 신호 없음**
- 50 m 운용 범위: $k_1 < 0.18$ 필수
- **권장:** 선형 보상 $w_{dist}(d_{prev} - d_{xy})$ — 어떤 거리에서도 기울기 존재

**보상 공식 변경 후 → 반드시 fresh start**
- Replay buffer에 구 보상 저장됨 → critic이 혼합 보상으로 ~100K 스텝 학습 → Q-value 오염

**WandB 첫 롤아웃 후 확인 사항:**
- `env/mean_rew_dist ≠ 0`이면 진행
- `= 0`이면 즉시 중단 & 진단

---

## Rule 5 — 체크포인트 & Replay Buffer 무결성

| 체크포인트 | 설명 | 재개 안전성 |
|-----------|------|-----------|
| `sac_drop_preempt.zip` | SIGTERM 시 `_emergency_save()` 저장 | ⚠️ 폭발 후 오염 가능 |
| `sac_drop_{N}_steps.zip` | 5000 스텝마다 rolling 저장 (최근 5개) | ✅ 항상 안전 |

- **물리 폭발 후 preempt 재개 절대 금지** → 최신 rolling checkpoint 사용
- 오염된 버퍼: `.CORRUPTED_{YYYYMMDD}` 리네임 (삭제 금지)
- rolling checkpoint에 `_replay.pkl` 없음 → SB3가 자동으로 새 버퍼 시작

---

## Rule 6 — WandB 메트릭 무결성

**물리 글리치 값을 running mean에 포함 금지.**

패턴:
```python
# 글리치 스텝: 별도 키 사용
info['glitch_d_xy'] = d_xy   # WandB 누산 안 됨
# 정상 스텝만 누산
if 'd_xy' in info:
    d_xy_accum.append(info['d_xy'])
```

---

## WandB 메트릭 레퍼런스

| 메트릭 | 의미 | 정상 신호 |
|--------|------|---------|
| `env/mean_d_xy` | 롤아웃 평균 드론↔표적 거리 | 감소 추세 |
| `env/mean_rew_dist` | Layer 3 거리 성분 평균 | 양수 = 접근 중 |
| `env/mean_rew_orient` | 방향 정렬 보상 평균 | 초기 0, 점차 양수 |
| `env/mean_rew_ctrl` | Layer 2 안정성 페널티 평균 | 항상 음수, 소 크기 |
| `env/drop_error_actual_m` | 물리 착탄 오차 평균 | 감소; 0 = 완벽 |
| `env/success_rate` | 0.5 m 이내 투하 비율 | 증가; 목표 > 0.8 |
| `env/physics_glitch_count` | 롤아웃당 Gazebo 폭발 횟수 | 0 유지; >0 이면 조사 |

---

## Rule 7 — RTF(Real-Time Factor) 선택

> **상세 분석:** [[research/rtf_fps_analysis]]

**결론: RTF=2 고정.** RTF=4 이상은 Python RL 루프 병목으로 FPS가 오히려 감소한다.

| RTF | avg fps | 권장 |
|-----|---------|------|
| 1 | 40.5 | 디버깅·dry-run 전용 |
| **2** | **59.5** | **기본 학습 (최적)** |
| 4 | 51.5 | 금지 — fps 역전 |

**왜 역전되는가:**
- Gazebo+PX4는 RTF배로 가속되지만 Python `train_sac` 루프는 CPU 바운드
- `obs_wait_timeout=0.02s` 기준, RTF=4에서 obs 유실 및 강제 진행 발생
- 결과: step당 실질 대기 시간 증가 → FPS 하락

**RTF > 2로 이득 보려면 먼저 필요한 것:**
- PyTorch AMP (GPU SAC update 가속)
- 비동기 obs collector 분리

---

## Rule 8 — Per-step Reward Density 변경 금지 (SAC 발산)

> **상세 메커니즘:** [[research/sac_reward_density_junsang]] · **발견 run:** Round 4 `4j46qwpk` (2026-05-31)

**SAC auto-entropy는 per-step reward density에 매우 민감하다. density를 줄이면 발산한다.**

- Round 4에서 hover 차단을 위해 per-step 보상을 축소(`w_heading` 0.7→0.3) + per-step 페널티 신규(`w_distance_penalty` 0.03) → **146k에서 발산**
  - per-step : drop reward 비율 1:300 → **1:700** (sparsity 2배)
  - 결과: `ent_coef` 0.58 → **6.03 폭주**, `critic_loss` 100 → **230,000+**
- **발산 모드 메커니즘:** sparsity↑ → critic variance↑ → policy가 dominant 신호(drop)에 집중 → entropy↓ → SAC "탐색 부족" 판단 → `ent_coef`↑ → bounded action space[-1,1]라 entropy 못 올림 → 양성 피드백 → 발산

**규칙:**
1. hover/loitering exploit 차단은 **per-step density를 건드리지 말고 terminal signal로 해결** (Round 5 Hover Terminal Penalty: 종료 시 -15 1회)
2. per-step 보상 weight를 바꿔야 하면 반드시 짧은 run에서 `ent_coef` 추세 먼저 확인 (단조 감소 = 정상)
3. 정상 oscillation vs 발산 구별 (Known Failure Modes 참조)

---

## Rule 9 — Post-success Regression

> **발견:** Round 2 `z05fx7g9`, Round 3 `lidq3ydu` 공통

큰 success terminal reward(+200~+550)가 critic을 흔들어 직후 정책이 붕괴하는 패턴.

- Round 3: 100~125k 최우수(avg 13.9m, success 3건) → 125~150k avg 35.5m로 후퇴
- 완화책: PER priority cap(30), LR 1e-4, tau 0.002, Hard cap [-200,+300] — 발산은 막지만 oscillation 자체는 SAC + sparse reward의 본질적 특성 (완전 제거 불가)
- **판단:** 단기 fluctuation에 흔들리지 말고 장기 추세로 평가

---

> **Phase 1 전체 계획:** [[research/phase1_plan]] — CCIP 기반 자율 접근, 8주, 14개 실험

---

## Rule 10 — 움직이는 상태로 spawn 시 컨트롤러 setpoint seed (Isaac)

**드론을 정지가 아닌 속도로 spawn**하면(예: cruise 핸드오프 `cruise_speed`), reset에서
캐스케이드 속도 컨트롤러 상태(`_v_filt`, `_prev_action`)를 **그 속도로 seed**해야 한다.
0으로 두면 첫 스텝에 실제 속도 vs setpoint의 큰 추종오차 → 급격한 tilt →
`ang_vel > limit_ang_vel(2.0)` → `bad_attitude`로 **에피소드 즉사**(length 1, 학습 신호 0).
seed 후 정상 학습(v11 dry-run 100% success). → [[research/isaac_cruise_handoff_junsang]],
[[experiments/exp_006_v11_dryrun_junsang]]

---

## Known Failure Modes

| 증상 | 원인 | 해결 |
|------|------|------|
| `mean_rew_dist = 0` | 지수 포텐셜 포화 (k1 너무 큼) | 선형 보상 사용; $e^{-k_1 d_{max}} > 10^{-6}$ 확인 |
| `mean_d_xy` → 1e11 | Gazebo ODE 물리 폭발 | 3중 방어 레이어 → [[errors/err_20260320_physics_explosion]] |
| CRUISE 타임아웃 | PX4 arm race / 드론 뒤집힘 | `reset()` 1회 재시도; 10에피소드당 >1회면 조사 |
| `ep_rew_mean` 나선형 하락 | CRUISE 타임아웃 → 크래시 페널티 에피소드 버퍼 오염 | CRUISE 타임아웃 근본 원인 수정; fps 하락 확인 |
| fps 급감 | CRUISE 타임아웃 (65 s 대기) 또는 ODE 크래시 | 로그에서 "Timed out waiting for CRUISE" 확인 |
| `ent_coef` 폭주 (1.0+), `critic_loss` 1000+ 지속 | per-step reward density 과소 → reward sparsity 발산 (SAC auto-entropy 양성 피드백) | per-step density 복원; hover는 terminal penalty로 차단 → [[research/sac_reward_density_junsang]] (Rule 8) |
| critic 폭주 (17K), 큰 terminal과 충돌 | `gradient_steps=4` + 큰 terminal reward(+250~350) 동시 → critic 4배 빠르게 fit 실패 | `gradient_steps` 1로 복원 (junsang_v2 `zn7xrm7e` 발산) |
| 100~125k 최우수 후 급후퇴 | post-success regression (큰 success reward가 critic 교란) | PER cap + LR/tau↓로 완화, 장기 추세로 평가 (Rule 9) |
| hover 후 random_drop으로 종료 (drop_error ≈ spawn→target 거리) | hover exploit — heading 보상 수확이 success보다 안전 | Hover Terminal Penalty (종료 시 -15, sustained hover만) → [[experiments/exp_004_round5_hover_junsang]] |
| `gz model --list` 5s timeout → 학습 abort | PX4 `.ulg` 로그 누적(20GB) → 디스크 I/O 지연 | PX4 로깅 비활성화 (`SDLOG_MODE -1`), 누적 로그 삭제 |
| `bad_attitude 1.0`, episode length 1 (Isaac) | 움직이는 상태로 spawn했는데 컨트롤러 setpoint(`_v_filt`/`_prev_action`) 0으로 리셋 → 첫 스텝 큰 속도오차 → 급기동 | reset에서 컨트롤러를 cruise 속도로 seed (Rule 10) → [[research/isaac_cruise_handoff_junsang]] |

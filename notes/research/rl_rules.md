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

## Rule 8 — Arming 게이팅 & Stuck-Takeoff Early-Bail (Throughput)

> **상세 분석:** [[research/cruise_timeout_arming]] / [[errors/err_20260615_cruise-timeout-arming]]

CRUISE 타임아웃은 "느린 비행"이 아니라 **arming-rejection** 문제. teleport
(`gz_reset_poses`) 직후 stale EKF → `pre_flight_checks_pass=False` → PX4 arm 거부.

**필수 규칙:**
- **arm은 `vehicle_status.pre_flight_checks_pass=True`일 때만.** 고정 warmup 후 무조건 스팸 금지.
- arm 거부 사유는 `/fmu/out/vehicle_command_ack`로 로깅 (`ARM REJECTED by PX4: <reason>`).
- TAKEOFF 고착 시 cruise 타임아웃을 끝까지 기다리지 말 것. `arm_bail_timeout=10.0s`
  내 arm 안 되면 즉시 full infra restart (fresh restart는 다음 시도에서 항상 EKF 재수렴).
- **타임아웃 *값* 낮추기는 no-op** — 진짜 레버는 타임아웃 *횟수* 감소.
- ⚠️ `cruise_poll_timeout` 기본값(`cfg_env.get(..., 60.0)`)을 실제 설정값으로 오독 주의.

---

## Rule 9 — Episode 종료가 빠를 때 `ep_rew_mean` 해석

드론이 마커에 빨리 도달하면 `ep_len`이 붕괴(예: 151→36.5)하여 SB3
`rollout/ep_rew_mean`이 *하락*하는 **착시**가 생긴다 (v11: 70→48).

- **per-episode `env/ep_reward`를 진짜 신호로 사용** (v11: 20→54 상승).
- `ep_len`과 함께 해석. `ep_len` 급감 + per-episode 보상 상승 = **개선 중**.
- 보조 지표: `ep_best_d_xy`, `reached_close`(≤3m) 비율, success(d_xy≤0.5m) 수.

---

## Rule 10 — 핸드오프 거리와 종단 보상 트랩 (Overshoot Moat)

> **상세 분석:** [[research/terminal_overshoot_trap]]

RL 에피소드는 **CRUISE→TRACKING 핸드오프 위치**에서 시작한다 (`reset()` L584-587).
카메라 방향이 이 핸드오프 거리를 결정한다 — **정하방 카메라는 마커를 ~1m 머리 위에서만 탐지** →
핸드오프 d_xy≈1.0m → 정책은 "마지막 1m 닫기"만 학습.

**필수 규칙:**
- **종단 종료 가드는 핸드오프 거리에서 무장되면 안 된다.** `overshoot_close_threshold`는
  핸드오프 d_xy보다 *작게*, 그리고 `success_radius`보다 작거나 같게 둘 것.
  (v12: threshold 1.5 > 핸드오프 1.0 → step 1부터 무장 → 매 에피소드 -20.)
- **액션 스케일은 핸드오프 거리에 맞춰라.** ~1m에서 시작하면 8 m/s vx는 0.5m 성공원을 지나친다.
  종단 단계 vx/vy는 4/3 m/s 수준. (action_*_scale은 `step()` L605-609, RL 단계 전용.)
- **`success_radius`는 도달 가능해야 한다.** 신뢰성 있게 도달하는 거리(여기선 ~1m)보다 살짝 작게
  잡아 +100 신호가 실제로 발생하게 한 뒤, 커리큘럼으로 조인다 (0.8→0.5).
- **종단 위험/보상 비대칭 주의:** 거리보상은 망원 합산되어 핸드오프가 가까우면 총합이 작다
  (1m → 단 +2.0). 이진 성공 게이트 앞에 큰 음수 해자를 두면 정책이 "돌진=위험"을 학습해 호버한다.
- **진단 시그니처:** `ep_len` *감소* + `ep_rew` 음수 고정 + success 0 + d_xy는 잘 도달 = 보상 트랩
  (under-training 아님, Rule 9 착시와 구분).

---

## Rule 11 — 인프라 타임아웃은 "복구 곡선"을 계측한 뒤 정하라

> **상세 분석:** [[research/cruise_timeout_arming]] · [[experiments/exp_006_xgzum51v_armdiag_dryrun]]

Rule 8의 `arm_bail_timeout=10s`는 *추정값*이었고, 그게 v13의 지배적 throughput 싱크였다.
teleport 후 `pre_flight_checks_pass` 재수렴 시간을 계측하니 **bimodal**: **0.0s (warm, 7/12)** 또는
**13–16s (stuck-EKF cohort, 5/12 ≈ 42%)**. 10s 컷은 복구 직전(3–6s 전)에 멀쩡한 PX4를 버리고
~25–30s full restart를 강제했다. 25s 창에서는 **bail 0**, late cohort 전부 회복.

**필수 규칙:**
- **early-bail 타임아웃을 정하기 전에 "복구가 언제 일어나는가"를 먼저 계측하라.** "복구 불가"로
  단정하고 짧게 자르면, recoverable-with-time을 full-restart-only로 오판한다.
- 임계는 **관측 복구 최댓값 + 마진**으로. (여기선 max 15.7s → `arm_bail_timeout=20.0`.)
  진짜 죽은 인프라는 여전히 그 시간에 fast-fail.
- **로그 메시지가 메커니즘을 단정하지 않게 하라.** "arm-reject suspected"는 실제로는
  "arm-never-attempted (EKF 미수렴)"였다. 진단 계측(컨트롤러 `PREFLIGHT-PASS` dt 로깅)이
  추측을 데이터로 바꿨다.
- (장기) 타임아웃은 증상 완화. 근본 해결은 teleport reset이 EKF를 덜 흔들게 하는 것.

---

> **Phase 1 전체 계획:** [[research/phase1_plan]] — CCIP 기반 자율 접근, 8주, 14개 실험

---

## Known Failure Modes

| 증상 | 원인 | 해결 |
|------|------|------|
| `mean_rew_dist = 0` | 지수 포텐셜 포화 (k1 너무 큼) | 선형 보상 사용; $e^{-k_1 d_{max}} > 10^{-6}$ 확인 |
| `mean_d_xy` → 1e11 | Gazebo ODE 물리 폭발 | 3중 방어 레이어 → [[errors/err_20260320_physics_explosion]] |
| CRUISE 타임아웃 (~42% late-arm) | teleport 후 EKF 재수렴이 bimodal (0s 또는 13–16s). `pre_flight_checks_pass`가 늦게 True. **v12의 10s 컷이 복구 직전 단두대질** → full restart 강제 (진짜 throughput 싱크) | pre_flight_checks_pass 게이팅 + `arm_bail_timeout` **10s→20s** (복구 곡선 계측 후) → [[research/cruise_timeout_arming]] (Rule 8, **Rule 11**) |
| `ep_rew_mean` 나선형 하락 | (1) CRUISE 타임아웃 버퍼 오염 **또는** (2) ep_len 붕괴 착시 (도달 빨라짐) | 근본 원인 수정; per-episode `env/ep_reward`로 진짜 신호 판정 (Rule 9) |
| **ep_len 감소 + ep_rew 음수 고정 + success 0** (d_xy는 잘 도달) ⚠️ v12 | 정하방 카메라 → 핸드오프 ~1m → overshoot 가드(threshold 1.5)가 step 1부터 무장 + 8 m/s 액추에이터가 0.5m 성공원 지나침 → 매 에피소드 -20 | overshoot 무장 거리 < 핸드오프, success_radius 도달가능하게, 액션 스케일 ↓ → [[research/terminal_overshoot_trap]] (Rule 10), `hyperparams_v13.yaml` |
| **YOLO `target_lost_rate` ~29% bimodal (악화 중)** ⚠️ OPEN | per-step YOLO 트리거가 에피소드별 전부-탐지(rate=0, 70.7%) 또는 전무-탐지(rate=1, 29.3%)로 분리; partial 0%. 추세 0.24→0.35 | **미해결.** ~29% step에서 obs[9-11] zeroed + `-10` 페널티. 별도 처리 필요 → [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] |
| fps 급감 | CRUISE 타임아웃 (65 s 대기) 또는 ODE 크래시 | 로그에서 "Timed out waiting for CRUISE" 확인 |
| **드론이 마커 거울상으로 비행** (East 부호 반전) | East 타겟이 -11(거울)로 설정됨. PX4 East = +Gazebo_East (반전 없음)인데 반전 가정함 | `target_ned_y=+11`, `cruise_speed_y=-1`, `target_enu_x=+11`. ⚠️ d_xy 로그는 거울상 자기일치로 속임 → `gz model -p` ground-truth 검증 필수. 상세: [[coordinate-frames]] / [[research/ekf_east_reversal]] (06-12 진단 RETRACTED) |
| YOLO 탐지 무효 (silent) | ultralytics Boxes boolean 인덱싱 silent fail | `detections[:0]` 정수 슬라이스로 대체 |
| 이중 YOLO 노드 실행 | 수동 기동 + env 기동 중복 | env가 YOLO를 `_infra_procs`로 관리; 추가 수동 기동 금지 |

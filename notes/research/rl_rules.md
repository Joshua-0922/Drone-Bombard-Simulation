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

## Rule 12 — 평가는 시작 상태를 게이트하라 + harness 지표를 env와 정합시켜라

> **상세:** [[research/eval_terminal_env_metrics]] · [[experiments/exp_007_iyhfy5ps_v13_eval]]

v13 deterministic eval(20-ep 요청)에서 ep 1–3은 0.8m 성공(reward ~124)했으나 ep 4–13은 **전부 step 1에서
EKF 위치 발산(`d_xy≈11.9m`=home→target) → −15 truncation**. 연속 full-infra restart가 EKF를 ~21s 안에
수렴시키지 못해 drift→restart→drift **흡수 루프**(자체 회복 불가)에 빠짐. 정책이 아니라 **시작 상태 결함**.

**✅ 06-21 근본 원인 규명:** 발산은 fundamental EKF 버그가 아니라 **누적 sim degradation**이었다.
`_start_infra`가 fresh-start마다 YOLO `xmarker_detector`를 **죽이지 않고 새로 spawn** → 누적(3개) →
다중 detector 충돌 pixel_coords → spurious TRACKING(conf=0.00) + EKF↔camera 불일치. **clean teardown +
YOLO 누수 fix(fresh-kill에 `xmarker_detector` 추가) 후 dry-run 3/3 SUCCESS, gate 0회.** 원래 ep 1–3 성공도
누적이 ep 4에 임계 초과한 것으로 설명됨.

**필수 규칙:**
- **에피소드 시작 직전 상태를 게이트하라.** step 1 전에 EKF 위치 ↔ 카메라-마커 위치 **일치 검사**(또는
  fresh infra 후 settle-wait). 불일치면 통계에 −15로 세지 말고 **retry**. (학습 throughput에도 동일 이득.)
- **인프라를 재시작하면 그 인프라가 spawn한 모든 노드를 죽여라.** YOLO 누수처럼 "kill 안 하고 respawn"은
  조용히 누적되어 N번째 restart에서 임계를 넘긴다. 장기 run/연속 restart 후 평가 전 **clean teardown** 필수.
- **eval은 학습보다 reset 결함에 더 취약하다.** 학습의 산발적 truncation이 eval의 연속 restart에선 누적되어
  absorbing loop가 된다 — 짧은 eval에서 "정책 실패"로 오판하기 쉽다.
- **harness 지표를 env가 실제로 emit하는 것과 정합시켜라.** `evaluate.py`는 env가 안 만드는
  `info['drop_error_actual_m']`를 읽어 miss-distance/CEP/drop-speed를 전부 NaN으로 보고했다.
- **종료 조건이 곧 측정 가능한 지표를 정한다.** v13 env는 0.8m 성공원에서 종료(탄도 투하 미모델링) →
  "CEP/drop_error"는 실재하지 않음. 올바른 지표는 **success_rate + step-to-reach + deterministic ep_reward**.

---

## Rule 13 — "늦은 탐지/짧은 윈도우"는 탐지 게이트로 풀어라 (고도 아님)

> **상세:** [[research/detection_gate_vs_altitude]] · [[experiments/exp_008_dryrun_alt10_handoff_window]]

핸드오프(CRUISE→TRACKING)가 "거의 머리 위"(d_xy ~3.5 m)에서 일어나 RL 윈도우가 짧은 문제를
**순항 고도↑(5→10 m)로 풀려 했으나 실패.** 고도는 레버가 아니다:
- 마커 apparent size ∝ 1/고도 → 10 m에선 절반 크기 → YOLO가 *더 가까워야* lock → 핸드오프 여전히 2.7 m.
- `vision_callback`의 **200 px 공간 필터**가 고도 무관하게 핸드오프를 중심 근처로 클립.
- 넓어진 FoV가 순항 시작에서 X-like 지면 FP를 잡아 **spurious CRUISE→TRACKING(conf=0.00, d_xy≈11 m)** 유발.

**진짜 수정 = 탐지 게이트:**
- **confidence 게이트**(`min_detection_conf=0.5`): real 마커 conf 0.73–0.95 vs 지면 FP ≤0.45 → 간격에 임계 안착.
- **공간 필터 완화**(200→300 px): real off-center 탐지(264–293 px) 조기 accept → 핸드오프 2.7→**5.0 m**(윈도우 ~2배).

**필수 규칙:**
- **탐지 타이밍 문제는 탐지 파이프라인에서 풀어라.** 필터 반경·conf 임계·마커 가시성이 레버. 비행 고도/기하는 아니다.
- **공간 필터 완화는 반드시 confidence 게이트와 함께.** 반경만 키우면 off-center FP 표면이 커진다.
- **게이트 임계는 데이터로 정하라.** accept/reject를 로깅해 real vs FP conf 분포 사이 간격에 임계를 놓아라.
- **고도↑는 마커 가시성을 깎는다** — apparent size ∝ 1/고도. 윈도우 확장엔 역효과.
- **기하/탐지 변경은 보상 공식 변경이 아니다** → fresh start 필수는 아니나, 핸드오프 거리가 바뀌면 정책 초반 재적응 예상.

---

## Rule 14 — 리셋 처리량은 "EKF를 교란 안 하는 리셋"으로 풀어라 (param 아님)

> **상세:** [[research/reset_throughput_bottleneck]] · [[experiments/exp_009_softreset_throughput]]

에피소드 리셋의 지배적 비용(v14 fps≈2, ETA ~2.5일)은 `gz_reset_poses`(teleport)+disarm 후 PX4
**EKF 추정기 재수렴** 대기다. `ctrl_0.log`: `Delaying arm — pre_flight_checks_pass=False (EKF not yet
reconverged)`. fresh PX4 restart도 동일하게 timeout(cold-start EKF 수렴 bimodal 0s/13–16s, Rule 11과 동일 뿌리).

**param/timeout은 못 고친다(음성 확정):** `EKF2_GPS_CHECK 0` A/B = 차이 없음 — airframe가 이미
`COM_ARM_WO_GPS 1`이라 GPS는 게이트가 아니었음. 실제 게이트 = EKF validity/innovation 수렴이고 **이를 끄는
param은 설계상 없다**(안전). arm_bail·mag-check·fast-path-제거도 전부 증상 완화였다.

**통하는 것 = soft reset(teleport 회피):** 종료 시 flyable이면 disarm/teleport 없이 armed+airborne 유지 →
position setpoint로 출발점 복귀(controller 살려둬 20Hz offboard heartbeat 유지) → mission_manager FSM만
재시작 → 재핸드오프. EKF가 연속 비행 내내 교란 안 됨 → 재수렴 대기 0. **결과: reset 65s→11s, fps 2→9,
throughput ~3.9×, 32연속 soft reset에서 EKF d_xy 안정(4.5–5.8m, 발산 없음).**

**필수 규칙:**
- **teleport/disarm는 EKF 재수렴을 강제하고, 그 대기는 param으로 못 줄인다.** 연속 비행 복귀로 비용을 0으로.
- **항상 fallback.** 전복/저고도/EKF발산/비유한 종료는 soft reset 불가 → 기존 teleport+restart. 최악도 baseline(그 이하 아님).

**✅ Production 검증완료 (2026-06-23, byxyaf4d 0→196.5K):** 3096 soft resets에서 soft 성공 ~91%(fallback ~9%만 teleport), 학습 내내 EKF health gate(10m) 안쪽 유지(발산 루프 0). 학습된 정책에서도 fallback율·EKF drift 모두 bounded. → [[experiments/exp_010_byxyaf4d_v14_195k_eval]]
- **drift guard = self-correct.** soft reset로 EKF가 느리게 누적돼도 handoff d_xy > start_drift_max면 그 에피소드만
  teleport fallback → 누적 리셋. soft reset + drift guard = 빠르되 안전.
- **음성 결과도 규칙이다.** "param 만지면 빨라질 것"은 COM_ARM_WO_GPS 맥락에선 틀림 — 재시도 금지.

---

## Rule 15 — RL 인수 후 wobble = smoothness 문제, 보상+로직으로 교정

**진단 먼저:** eval이 `deterministic=True`면 wobble은 탐험 노이즈가 아니라 **학습된 bang-bang 정책**이다.
정량화: PX4 수신 속도명령(`/fmu/in/trajectory_setpoint.velocity`)의 **연속 차분 RMS(=command jerk)**로
측정. v14: raw jerk RMS **2.92 m/s/sample**(50 ms마다 ~m/s 진동) = wobble의 정체.

**원인 3종:** ① 과도한 액션 권한(vx=8/vy=5) ② 순수 진행 보상(w_dist=2.0)에 댐핑 거의 0
(w_ang_vel=w_action_smooth=0.05, 속도 페널티 부재) ③ 출력 평활 없음(RL setpoint 직행).

**교정(둘 다):**
- **로직 = 출력 LPF**(컨트롤러 EMA `velocity_lpf_alpha`, 0.4≈75 ms tau). A/B로 jerk **−45%**, 평균 속력 불변.
  값싸고 즉효지만 정책이 진동을 원하면 lag만 추가 → **원인 교정 병행 필수**. 학습 시에도 켜라(train==deploy plant).
- **보상 = 근접-게이팅 속도 댐핑**(B) `−w_vel·speed_xy·max(0,1−d_xy/R)`: **먼 구간 0(순항 자유), 근처만 감속.**
  "너무 느리게 하지 마라" 제약을 이렇게 만족(cruise-out 자유). + **smoothness 가중↑**(C).
- **레버 우선순위:** 보상 shaping → (부족 시) 액션 스케일↓ → (구조적) 이전 액션을 obs에 추가.

→ [[research/control_smoothness_wobble]] / [[experiments/exp_011_wobble_lpf_reward_damping]]

---

## Rule 16 — 시뮬레이터 이식 시 plant/reward parity는 상수가 아니라 "타이밍+메커니즘"까지 검증

Gazebo→Isaac Lab처럼 다른 시뮬레이터로 보상/제어 로직을 이식할 때, **값(상수)이 같다고 plant가
같다는 보장이 없다.** 세 가지를 반드시 소스에서 재확인:

1. **제어 주기 정합성:** 정책 스텝 Hz, 내부 필터(LPF 등) tick 주기, 이산 재귀식이 원본과
   정확히 일치해야 한다. `decimation×sim.dt`가 원래 정책 Hz와 다르면 정책은 **다른 plant를
   학습**하게 되고, 이후 성능 비교는 시뮬레이터 차이가 아니라 이 confound로 오염된다.
2. **가드/게이트의 "의도된 dormancy"를 버그로 착각하지 말 것:** 새 환경에서 값이 도달 불가능해
   보이는 종단 조건은 원본 소스에서 실제로 같은 이유로 dormant인지 먼저 확인(예: overshoot
   guard의 arm_radius < success_radius — Rule 10의 의도된 설계, curriculum 전환 시 활성화).
   확인 없이 "고쳐서" arm_radius를 낮추면 Rule 10이 막았던 바로 그 실패 모드(정상 접근 오탐)를
   재도입한다.
3. **미검정 컴포넌트는 명시적으로 라벨링:** 원본 시뮬레이터의 내부 루프(PX4 컨트롤러 등)를
   근사하는 새 컴포넌트(캐스케이드 컨트롤러 게인 등)는 실측 대조 전까지 "구조적으로 일치,
   미검정"으로 문서화 — 조용히 초기값을 최종값처럼 취급하면 이후 행동 비교가 이 미검정
   컴포넌트의 오차인지 실제 정책/보상 차이인지 구분 불가능해진다.

→ [[experiments/exp_012_isaac_migration_phase2]] / [[research/isaac_velocity_controller]]

---

## Rule 17 — 관측/보상용 센서 근사 모델은 "실패 특성"까지 이식하라 (성공 특성만 이식 금지)

> **상세:** [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] §4a · [[research/isaac_ppo_tuning_recommendations]]

analytic vision(핀홀 투영 + conf 0.73-0.95)은 YOLO의 **성공 특성**(탐지 시 픽셀/conf 분포)만
이식하고 **실패 특성**(apparent size ∝ 1/거리 → 원거리 conf 붕괴, Rule 13)을 누락했다.
그 결과 "고도를 올리면 centering이 기하적으로 쉬워지는데(`u_n ∝ x/z`) conf는 안 깎이는"
보상 지형이 생긴다 — 상승 farming은 실제 YOLO 환경에선 존재할 수 없는 정책이다.
(07-04 forensics로 귀속 확정 수순: 경합 가설이던 리셋 속도킥은 프로세스당 1회로 실증되어
기각 — max_alt 27-43%는 **iter ~200에서 창발**한 학습된 행동이고 같은 구간 rew_vision이
고유지, 이 attractor가 1차 가설. 인과 확증 실험 설계: [[research/exp014_ablation_protocol]].)

**필수 규칙:**
- 센서를 근사로 대체하면 그 센서가 **언제 못 보는지**(거리·각도·조명 감쇠)를 같이 모델링하라.
  누락된 실패 모드는 곧 착취 가능한 보상 지형이다.
- 근사 모델 기반 보상이 있으면 **그 보상을 극대화하는 퇴화 정책이 실물 센서에서도 가능한지**
  사고실험으로 검증하라 (여기선 "무한 상승 centering" — 실물 YOLO면 conf=0이라 불가능).
- 감쇠 커브는 추측 말고 **실측 캘리브레이션**(`yolo_eval.py --calibrate`)으로.
- 종단 실패 분포에서 특정 guard(max_altitude 등)가 지배하면 "천장이 낮다"가 아니라
  "**그쪽으로 가는 것이 이득인 보상 지형**"을 먼저 의심하라.

---

## Rule 18 — 종단 보상은 shaping 스트림을 지배해야 한다 + PPO noise_std는 감시 대상

> **상세:** [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] §4b/4c

**(a) Farmer-vs-finisher 수지 검산.** 보상 설계/변경 시 반드시 계산: "성공 반경 직전에서
per-step shaping(비전+근접)을 에피소드 끝까지 farming한 리턴" vs "즉시 완주 리턴".
exp_013: farmer +225 > finisher +121 → 정책이 마무리를 안 배우는 것이 **합리적**이었다.
Gazebo v14의 final-approach stagnation(0.5-0.8m 정체)도 같은 병인 — SAC는 증상으로,
PPO는 체계적 착취로 나타난다. 종단 보상은 farming 스트림 총합의 ≥1.5×로 설정
(`reward_success` 100→300).

**(b) PPO noise_std 폭주 감시.** 액션 파이프라인에 스무딩(clip→rate_limit→LPF)이 있으면
가우시안 노이즈가 plant에서 필터링되어 **entropy bonus를 견제할 task 손실이 없다** →
σ 단조 폭주(exp_013: 0.8→3.92). **07-04 계측(`_diag_noise.py`)으로 메커니즘 확정:**
- 노이즈는 액션 레벨에선 살아남는다(σ=3.9에서 executed Δaction 3.66× vs deterministic,
  rate-limiter 68% 포화, 액션 부호의 29%가 정책 평균과 반대) — "완전 흡수"가 아니다.
- 그러나 **실행 속도 궤적은 σ-불변**(velocity-Δ 비율: σ3.9/σ0.8=1.11×, σ3.9/det=1.01×) —
  LPF+accel clamp가 plant 레벨에서 균질화. σ를 키워도 task 리턴이 거의 안 변하니 entropy가
  공짜로 자란다. 폐해는 "rollout 오염"이 아니라(행동은 거의 동일) ① log-prob gradient 노이즈
  (부호 반전 29%) ② 목적함수 왜곡 ③ 탐험 이득 없는 σ 성장.
- **부수 발견:** 정책 평균 자체가 포화(σ=0에서 raw|a|=2.6, 성분 77%가 |a|>1) — rsl_rl
  가우시안 정책(무-squash)이 스무딩 파이프라인 아래서 클립-레일 위 bang-bang 평균을 학습.
  Rule 15의 wobble 병인과 동족 — 이식 시 재발 감시 대상.
- `Mean action noise std`가 중반까지 init값의 ~1.5×를 넘으면 개입 (entropy_coef ↓ 또는 0).
- rollout 지표가 나쁠 때 **deterministic eval로 정책 평균과 노이즈를 분리**한 뒤 판단하라
  — exp_013에선 둘 다 36%로 같았다(σ-불변 계측과 정합).

## Rule 19 — 런타임 물리 프로퍼티 오버라이드 금지: 컨트롤러가 읽는 값 == solver 값을 계측으로 증명하라

> **상세:** [[research/isaac_inertia_ctrl_mismatch]] (2026-07-05 `_diag_inertia.py` 계측)

**(a) 물리 프로퍼티(질량·관성·COM)는 스폰타임 USD authoring으로만 설정.** 런타임 뷰 API
(`set_masses`/`set_inertias`)는 두 종류의 불일치를 만든다: ① **1-substep 지연 소비** —
solver가 새 값을 첫 sim step까지 안 받아 stale 값에 wrench가 적분됨(+8.4 m/s 속도킥,
[[isaac_mass_override_reset_bug]]); ② **뷰 캐시 ≠ solver ≠ 컨트롤러 3자 분열** — exp_013은
`set_inertias`가 "전파 안 된다"는 잘못된 믿음 위에서 컨트롤러가 override 이전 값(1.66e-5)으로
토크를 사이징했지만, 계측 결과 solver는 x500 값(0.0217)을 실제로 받고 있었다 → **rate loop
~1300× 저토크**인 자세-마비 plant에서 학습 전체가 진행됨.

**(b) hover 게이트는 회전 plant 버그를 못 잡는다** (토크 ≈ 0 지점). plant 수정 후에는
**토크 응답 계측을 게이트에 추가**: 알려진 순수 토크 인가 → $I_{est}=\tau/\alpha$ vs
`get_inertias()` 일치 확인 (`isaac_lab/_diag_inertia.py` 재사용, ~3분).

**(c) plant 동역학을 바꾸는 수정 후 구 체크포인트 성능은 무효 — fresh start.** 정책은 보상만이
아니라 **plant의 버그에도 overfit**한다: exp_013 정책은 저토크 plant의 굼뜬 자세를 전제로
|ω| 한계(2.0) 직하에서 살았고, 일관 plant에선 중앙값 1.5 s 만에 68%가 bad_attitude로 죽는다
(프롭스핀 A/B로 로터 무관 실증). "교정된 plant에서 구 정책 성능 하락"은 회귀가 아니라
**정상** — 판단 기준은 fresh 학습 커브다.

## Rule 20 — 커리큘럼 warm-start는 아키텍처 고정으로, 페이즈 전환은 태스크 재정의로 취급하라

> **상세:** [[research/phased_curriculum]] / [[experiments/exp_015_phased_curriculum]] (2026-07-05)

**(a) 페이즈 간 weight 인계를 하려면 네트워크 아키텍처를 고정하라.** rsl_rl `OnPolicyRunner`는
env action/obs space에서 MLP를 만들므로, 페이즈마다 차원이 바뀌면 `runner.load()`가 shape
불일치로 실패한다. 커리큘럼(접근→CCIP+residual→이동타겟)에서 action을 4→6으로 늘려야 할 때는
**전 페이즈 6-dim 고정**하고, 초기 페이즈에선 env가 미사용 dim을 zero-out(무해). 부분 로딩
surgery보다 견고. obs 확장도 동일 — index 0-13 불변 + append만(exp_012 superset 규약).

**(b) 릴리스/터미널 보상 도입은 보상 공식 변경 = 태스크 재정의.** warm-start를 해도 2·3단계는
새 보상 지형이므로 **첫 롤아웃 후 재검증 필수**: `release_rate`(드론이 실제로 투하하는가) +
릴리스 실제 `drop_impact_error_m`(맞히는가)를 먼저 보라. proximity success가 아니라 릴리스가
종단을 지배하므로, 접근만 잘하고 안 던지면 timeout `no_release_penalty`로 신호가 뜬다.

**(c) MLP 정책은 단일 obs로 시간미분(타겟 속도)을 추론 못 한다.** 이동 타겟 lead를 "정책이
학습"하게 하려면 타겟 속도를 obs에 넣어야 한다(14→16). 넣기 전에는 env가 lead를 해석적으로
계산하고 residual만 모델오차를 보정하는 설계로 둘 것 — "정책이 리드를 배웠다"고 오독 금지.

**(d) 순차 학습은 서브프로세스로.** Isaac Sim은 프로세스당 1 sim만 안전 → 한 프로세스에서
env를 만들고 부수고 다시 만들지 말고, 페이즈마다 새 프로세스(`--resume`으로 체인).

**(e) 실학습 corroboration (2026-07-12, exp_015 baseline 완주, [[research/curriculum_phase_convergence]]).**
2048 envs·600/500/500 iters로 1→2→3 완주(ORCH_EXIT=0, ~65 min): **Phase 1만 완전 수렴**
(success 0.48→1.00, reward→107 — exp_014 100% 재현). 페이즈 경계에서 **reward 딥→빠른 회복**
(P2 −0.8→94.7 @~150 iter, P3 시작 57)이 (a)의 warm-start 무손실 + (b)의 보상 재정의를 그대로
실증. **그러나 P2·P3는 success ~0** — reward 우상향은 접근/proximity 스트림이 지배하고,
release-종단 명중(real_err ≤ 0.8 m)은 베이스라인 500 iter로 형성 안 됨(P2 drop 4.66→2.91 m로
반경 밖 정체, release_rate tail ~0.33 변동; P3 lead best 0.071 m이나 tail 평탄). **커리큘럼
baseline의 P2/P3는 reward가 아니라 `release_rate`+릴리스 `drop_impact_error_m`(+`lead_error_m`)로
판정하라** — Rule 21/22의 "근접≠릴리스"가 커리큘럼 스케일에서 재확인됐고, 뚫는 해법은 Rule 23의
릴리스=종단 구조(exp_018).

**(f) 이어학습 corroboration (2026-07-13, exp_015 §8 P2/P3 각 +2000 iters).**
§7 baseline 체크포인트에서 페이즈별 단독 연장(P2: 1098→3097, P3: 3097→5096, 2048 envs,
`release_terminal` 미적용). **0.8 m 돌파 없음(success ≈ 0 전 구간).** P2 drop tail
2.91→2.87 m(**정체**, best_min 0.008 m는 스파이크), release_rate 0.33→**0.01** 급락(근접 최적화가
릴리스 억제 — exp_017 단조 하락과 동형). P3 drop 3.20→**5.31 m 회귀**, reward 101.7→74.5.
**베이스라인 커리큘럼에서 iter 예산만 늘려서는 P2/P3 릴리스-종단 명중을 기대하지 말 것** —
구조 개입(exp_018) 없이는 P2 ~3 m plateau, P3 불안정/회귀. 상세: [[research/curriculum_phase_convergence]] §2(e).

---

## Rule 21 — 이벤트 조건부 지표는 그 이벤트를 명시적으로 시뮬레이트한 순간에 측정하라

> **상세:** [[research/ccip_release_decoupling]] / [[experiments/exp_016_ccip_release_reeval]] (2026-07-05)

**(a) "종단 스냅샷에서 계산한 임무 지표"는 종단 조건과 독립인지 먼저 검증하라.** exp_014의
`drop_impact_error_m` 4.59 m는 투하 오차가 아니라 **d_xy-성공 종단 순간의 잔여속도 탄도 캐리**
($\mathbf{v}\cdot(\sqrt{2H/g}+t_{delay})$ = 3.0 m/s × 1.53 s ≈ 4.6 m)였다. 릴리스 설계
의도(v15 `drop_calculator_node`)는 CCIP 예측착탄 오차 ≤ 0.2 m **트리거**인데, Phase-1
이식에서 트리거가 사라지고(`DropCfg.release_tolerance` 정의만 되고 미사용) 지표만 "잘못된
순간"에 측정됐다. 지표가 릴리스·발사·전환 같은 **이벤트**를 가정하면, 그 이벤트의 발화
조건을 스크립트로라도 시뮬레이트해서 **그 순간**에 측정할 것.

**(b) 근접 성공 ≠ 릴리스 능력.** 접근 정책의 CCIP 스윕 최근접은 기하학적으로 비행경로
cross-track 오차와 같은 차수다(재평가: aim_err_min med 0.755 m ≈ d_xy_min 0.665 m).
**d_xy ≤ 0.8 근접 보상으로 학습한 정책은 0.2 m 릴리스 윈도우를 통과할 이유가 없다**
(200-ep 재평가 release_rate 6-11.5%). success 100%가 임무 능력을 보증하지 않는다 —
릴리스 조건부 보상(Phase 2)이 만들어야 하는 능력이다.

**(c) 이산 트리거는 윈도우-스킵을 계측하라.** tol 0.2 m·10 Hz에서 착탄 예측점은
스텝당 ~v×0.1 m 스윕 — 윈도우를 샘플 사이로 건너뛸 수 있다. `aim_err_min`(에피소드별
CCIP 오차 최솟값)을 같이 로깅하면 "트리거가 못 발화한 이유"가 샘플링인지 cross-track인지
분리된다(재평가에선 100 Hz referee와 차이 ~0.04 m로 cross-track 지배 확인).

---

## Rule 22 — 이벤트 능력은 dense 사이드 보상이 아니라 종단 구조로 학습시켜라

> **상세:** [[research/ccip_aim_reward_stageA]] / [[experiments/exp_017_stageA_aim_reward]] (2026-07-06)

**(a) γ-할인 완주 보너스 + 조기 성공 종단 구조에서는 dense 사이드 보상이 진다.** 릴리스
능력을 겨냥한 밀집 CCIP 조준 보상($w(1-\tanh(e/s))$, w=1→2, knee 0.5→1.0 m, 총 1,000
warm-start iters)은 release_rate를 2.5→5.5%(n=200에서 p≈0.13)까지만 움직였고 더 강한
변형은 회귀했다. 구조 요인 3종: ①+100 성공의 γ=0.995 할인이 모든 감속을 ~0.5/0.1 s로
벌함(밴드 체류 소득은 tanh 폭이 캡) ②성공 종단이 조준을 다듬을 근접 구간 자체를 제거
③아래 (b). **이벤트(릴리스)가 에피소드의 종단·주보상이 되게 하라**(Phase 2 구조) —
사이드 지표로 두고 shaping으로 우회하지 말 것.

**(b) 예측 지평으로 증폭되는 노이즈는 dense 그래디언트를 평탄화한다.** CCIP 착탄점은
속도 노이즈를 $t_{fall}+t_d\approx1.5$ s 배율로 증폭 — σ~1.1-1.5의 탐험 노이즈면 착탄
지터가 m급이 되어 기대보상면에서 조준 항의 그래디언트가 사라진다(σ는 entropy가 유지).
knee를 넓히면 소득은 늘지만 **행동 불변의 수동 소득**이 될 뿐(v2: rew_aim 8× ↑, 행동
불변·회귀). 예측량 기반 보상을 설계할 땐 노이즈 증폭 배율부터 계산하라.

**(c) 학습 내(stochastic) 지표로 dense shaping 효과를 판정하지 말라.** v1의 실질 이동
(aim_err_min med 1.15→0.89 m, final_speed 3.35→2.72 m/s)은 학습 커브에선 전혀 안 보였다
(σ 지터가 가림) — deterministic eval이 판정 기준. 역으로 in-training 평탄이 "효과 없음"의
증거도 아니다.

**(d) shaping 소득 항 추가 시 farm 시그니처 모니터를 함께 켜라** (exp_013 교훈의 사전
적용): 에피소드 길이 creep + 지속 소득/step + timeout/stagnation/max_alt 상승. 이번엔
5-lens 적대 검증이 duty-cycle 펌프 가능성(PV 손익 계산)을 사전 경고 → w=1 헤지로 개시,
실측 farm 발생 0. 보상 추가 전 "이 항을 완주 없이 채굴하는 경로의 PV"를 계산할 것.

---

## Rule 23 — 임무 이벤트는 종단 이벤트로 만들라; 자동 발화 referee가 노이즈를 발견 메커니즘으로 바꾼다

> **상세:** [[research/release_terminal_stageB]] / [[experiments/exp_018_release_terminal]] (2026-07-06)

**(a) Rule 22a의 인과 확정.** 동일 보상·동일 warm-start에서 **종단만** 근접(d_xy≤0.8)에서
릴리스-발화로 교체 → det release_rate 5.5%→**100%**, 학습 내 추세 단조 하락(12→3.7%)이
단조 상승(23→99.6%)으로 반전. 조기 종단이 조준 구간을 잘라먹는 것이 지배 요인이었다.
이벤트 능력이 목표라면 그 이벤트가 에피소드를 끝내고 주보상을 지급하게 하라.

**(b) 자동 발화 트리거는 탐험 노이즈를 그래디언트 평탄화기에서 +100 샘플러로 바꾼다.**
릴리스가 정책 액션이 아니라 조건 충족 시 자동 발화이므로, 정책은 "임계 직상 유지"를 할 수
없다 — 노이즈로 0.2 m를 우연히 찍으면 즉시 종단 보상이 샘플링된다. Stage A에서 dense
그래디언트를 죽이던 CCIP 노이즈 증폭(×1.5 s)이 Stage B에선 발견 메커니즘이 됐고, 사전
경제 분석이 경고한 배회-farm 정지-hazard 균형(소득>1.84/step → never-fire)도 같은 이유로
물리적으로 미발현(ep_len 52→36 감소 수렴).

**(c) 이벤트가 종단이 되면 그 이벤트를 겨냥한 dense shaping은 거의 잉여.** w_aim
0/1.0/1.5 전부 100%·~0.13 m(mid-training 가속 정도의 차이). 쓸 거면 좁은 knee 유지 —
넓히면(0.5→0.75) 근-윈도 기울기가 희석돼 근소 열화(98.5%, 종단 속도 0.30 m/s).

**(d) done-flag를 reset이 in-place 변조하는 버퍼의 alias로 캐시하지 말라.** `success =
self._just_released`(alias) → `_reset_idx`의 in-place clear가 step() 반환 전에
`_done_flags`를 지워 **평가 하니스가 완벽한 정책을 success 0%로 보고**할 뻔함(wandb는
스냅샷 clone이라 면역). 종단 플래그 캐시는 `.clone()`으로. (적대 검증이 학습 전 발견.)

**(e) 지표 의미론:** 발화=종단 모드에선 release_rate ≡ success rate. 비종단 래치
시절(exp_016/017의 6%/5.5%)과 수치 직접 비교 금지.

---

## Rule 24 — per-env 동적 결합/분리는 조인트가 아니라 kinematic weld로; 물리↔해석 parity를 계측으로 증명 후 전환하라

> **상세:** [[research/physical_payload_attach]] / [[experiments/exp_019_physical_payload]] (2026-07-21)

**(a)** GPU-복제 PhysX는 per-env 조인트 생성/제거(토폴로지 변경) 불가 — "부착했다 분리"는
fixed joint가 아니라 **kinematic weld**(부착 env만 매 physics step pose+velocity write,
분리 = write 중단)로 구현하라. 완전 벡터화되고 복제 물리를 건드리지 않는다. 검증: 추적
오차 1.1 mm, 측정 착탄 vs 해석적 CCIP |Δ| ≤ 0.021 m (exp_019, 8/8).

**(b)** weld는 kinematic이라 부착 중 하중이 드론 solver에 전달되지 않는다 — 페이로드
질량은 드론 authored 질량에 포함 유지(Rule 19: 런타임 질량 변경 금지). 비용 = 분리 후
팬텀 질량(릴리스=종단 커리큘럼에선 미발현; 에피소드를 착탄까지 연장 시 per-env ctrl_mass
필요).

**(c)** 물리 경로(측정 착탄)로 해석적 경로(referee/보상)를 교체하기 전, 두 경로의 parity를
같은 조건에서 계측으로 증명하라. 특히 DR(drag/wind)이 해석식에만 적용되는 상태에서
교체하면 Phase 2가 조용히 다른 과제가 된다.

---

> **Phase 1 전체 계획:** [[research/phase1_plan]] — CCIP 기반 자율 접근, 8주, 14개 실험

---

## Rule 25 — smoothness/속도 댐핑 반경이 종단 실패 반경을 덮으면 회귀 유발

**상세:** [[daily/daily_2026-07-05_gazebo_v15_regression]] · [[experiments/exp_011_wobble_lpf_reward_damping]] · [[experiments/exp_010_byxyaf4d_v14_195k_eval]]

> Gazebo/SAC 트랙(jekyun/Isaac-JS 브랜치, isaac_jk 분기 이전 07-01~07-05 구간)에서 나온 규칙. Isaac Lab 트랙과는 별개 시뮬레이터/보상 코드다.

v14의 지배적 실패 모드는 **final-approach stagnation**(0.5–1.2m에서 정체, `success_radius` 직전).
Rule 15의 wobble 교정(B: 근접-게이팅 속도 댐핑)을 `vel_damp_radius=3.0m`로 도입했는데, 이는
`success_radius=0.8m`보다 훨씬 넓다 — 결과적으로 **정체가 이미 벌어지던 바로 그 구간 전체에
"가까울수록 감속" 유인이 추가로 걸림.** `w_ang_vel`/`w_action_smooth` 상향도 방향은 같아서, 종단
gap을 closing하는 데 필요한 결정적 보정 기동에 비용을 물릴 수 있다.

**교훈 — smoothness/속도 댐핑 보상을 도입할 때:**
- 댐핑 반경(`*_damp_radius`)은 반드시 **success/termination 반경보다 작게** 설정할 것. 크면
  "wobble 억제"가 "종단 접근 억제"로 새는 회귀를 유발할 수 있다.
- 기존에 알려진 실패 모드(여기선 stagnation)가 있는 상태에서 새 댐핑/스무딩 보상을 얹을 땐,
  그 실패 모드가 발생하던 정확한 거리 구간과 새 보상의 활성 구간이 겹치는지 **먼저 겹쳐 그려볼 것**.
- 완화(값 축소, 0.15→0.08처럼)는 리스크를 줄이지만 **제거하지는 않는다** — "완화했으니 됐다"로
  넘기지 말고 반드시 eval outcome breakdown으로 재발 여부를 확인.

**추가로:** crash-resume 서포바이저(`run_train_supervised.sh`)는 재개 시 정책 가중치만 복원하고
**replay buffer는 매번 초기화**된다. "recurring abort"가 있었다면 최종 스텝 카운트만큼의 연속 학습이
실제로는 이뤄지지 않았을 수 있다 — 반복 크래시가 있었던 run을 평가할 땐 재개 횟수를 로그에서
정량화하고, 가능하면 crash-resume 시 replay buffer도 함께 보존하도록 체크포인트 로직을 개선할 것.

**⚠️ 07-05 정정:** v15의 310K 정지는 애초 계획된 eval-stop이 아니라, **호스트 GPU 드라이버가 Isaac Lab
트랙용으로 580으로 업그레이드되면서 `drone-bombard-harmonic` 컨테이너(NVML/CUDA lib 535 빌드)가 깨져
강제 중단된 것**으로 확인됨. → Rule 26.

---

## Rule 26 — GPU 드라이버는 트랙 간 공유 자원이다: 한쪽 업그레이드가 다른 쪽 컨테이너를 깬다

**상세:** [[daily/daily_2026-07-05_gazebo_v15_regression]]

> Gazebo/SAC 트랙 쪽 발견. Isaac Lab 컨테이너(`isaac-verify`)와 Gazebo 컨테이너(`drone-bombard-harmonic`)가
> 같은 호스트 GPU를 공유하던 시절(07-03)의 인시던트 — isaac_jk 워크플로우가 `isaac-verify` 전용으로 굳어진
> 지금은 재발 가능성이 낮지만, 호스트 드라이버를 바꿀 일이 있으면 여전히 유효한 경고.

Isaac Lab 트랙 작업을 위해 호스트 GPU 드라이버를 535→580으로 업그레이드했는데, 이 호스트는
Gazebo/SAC 트랙(`drone-bombard-harmonic` 컨테이너, NVML/CUDA userspace lib 535 빌드)과 **GPU를 공유**한다.
드라이버 업그레이드 시점(07-03)에 v15 학습이 마침 진행 중이었고, 이후 조사 결과:

- 컨테이너 내부 `nvidia-smi` → `Failed to initialize NVML: Driver/library version mismatch`.
- `gz sim -s -r --headless-rendering ...` 단독 실행 시 **45초간 로그 무출력**, `gz topic -l`은
  `timeout 8` 래핑에도 **행(hang)** — CUDA(추론)만이 아니라 gz sim의 헤드리스 GPU 렌더링
  (Ogre2/EGL) 경로 자체가 깨졌을 가능성.
- PX4가 position data를 못 받아 **CRUISE timeout → full restart ×3 → forced reset**을 반복
  — v15 학습 말기에 관측된 reset-recursion과 동일 시그니처.

**교훈:**
- 한 트랙(Isaac)의 드라이버 업그레이드가 다른 트랙(Gazebo/SAC)의 컨테이너를 조용히 깰 수 있다.
  드라이버 변경 후에는 **양쪽 컨테이너 모두** `nvidia-smi`/`gz sim` 기동을 검증할 것.
- "학습이 특정 스텝에서 멈췄다"는 사실만으로 계획된 stop인지 강제 중단인지 판단하지 말 것 —
  체크포인트 타임스탬프와 그 시점의 다른 트랙 작업(드라이버·인프라 변경) 이력을 대조 확인.
- 인프라(드라이버/컨테이너) 문제는 보상/정책 진단보다 **우선순위가 높다** — 인프라가 죽어 있으면
  eval 실패가 정책 실패처럼 보일 수 있다(reset-recursion 시그니처가 두 경우 모두 동일하게 나타난다).

---

## Rule 27 — 고정 초기조건은 성능이 아니라 표현을 암기시킨다; 랜덤화 축은 "표현을 바꾸는 축"과 "강건성만 요구하는 축"으로 나눠라

**상세:** [[research/handoff_generalization_p0]] / [[experiments/exp_022_p0_handoff_dyn_dr]]

**근거(동일 ckpt, 동일 표본 200-ep, seed 42):** 고정 핸드오프 91.0% → +동역학/센싱 DR 91.5%
→ +속도/고도/오프셋/자세 77.1% → +**월드프레임 방위 ±180°** **7.5%**(out_of_range 54%,
d_xy_min med 1.06 → 16.78 m). 발화 시 착탄오차는 네 조건 모두 0.32–0.40 m로 **불변**.

**교훈:**
- **일반화를 주장하기 전에 초기조건 분포를 랜덤화하라.** 고정 핸드오프 수치는 그 초기조건에서의
  성능일 뿐, 암기와 구분되지 않는다.
- **축마다 난이도가 질적으로 다르다.** 플랜트 파라미터(질량 신념 ±5%, 게인 ±10%, 페이로드
  탄도계수 ±20%)와 두-시간척도 센서/액추에이터 노이즈는 **거의 공짜**(−0.5 pp 이내).
  반면 관측 프레임을 회전시키는 축(방위)은 표현을 무효화해 warm-start 자체를 무의미하게 만든다.
- **분포 전이 실패는 관측 프레임을 먼저 의심하라.** 능력을 분해해 "무엇이 살아남았는가"를 보라 —
  발화 시 착탄오차 불변 = 조준/투하는 멀쩡, 접근만 파괴 → 월드프레임 obs가 범인.
- **랜덤화 토글은 OFF일 때 RNG를 소비해서는 안 된다.** 소비하면 노브를 켜는 것만으로 이전 버전들의
  난수 스트림이 밀려 "하위 버전 무손상" 보장이 깨진다.
- **런타임 물리 오버라이드(Rule 19) 대신 등가 변환을 찾아라:** 기체 질량 랜덤화 → *컨트롤러 질량 신념*
  랜덤화($a=(1+\delta)a_{des}+\delta g$로 동형), 페이로드 질량 랜덤화 → *탄도계수* 랜덤화
  (자유낙하는 $k/m$에만 의존하므로 정확히 등가). 둘 다 PhysX 쓰기가 0이다.

---

## Rule 28 — 종단 이벤트 판정은 "정지 상태의 기하"를 포함해야 한다; 래치 실패는 타임아웃으로 위장한다

**상세:** [[errors/err_20260803_payload_landing_latch]] / [[experiments/exp_023_table1_baselines]]

**근거:** 물리 페이로드 착지 판정이 `z_local <= payload_ground_z(=0.0)`였는데 페이로드는 높이
0.06 m 실린더 → 지면에 **정지하면 중심 z = 0.03 m**, 솔버가 관통을 막으므로 조건이 영원히 거짓.
래치는 서브스텝 터널링이라는 **물리 아티팩트에 의존**하고 있었다. 강제 릴리스 32-env 프로브:
래치 **0/32**, 도달 최소 z가 정확히 0.0300 m. `payload_land_eps=0.10`(exp_019 parity 검증값) 적용 후 **32/32**.
v19 정책 @ v19 성공률 **91.0% → 100.00%** 정정.

**교훈:**
- **부피가 있는 물체에 평면 통과 테스트를 쓰지 말 것.** 임계값은 반드시 정지 시 기하(half-height,
  반경, 접촉 오프셋)보다 커야 한다. "지면 = 0"은 점질량에서만 옳다.
- **이벤트 래치 실패는 실패처럼 보이지 않는다 — 타임아웃으로 위장한다.** 이벤트율(`release_rate`)은
  정상인데 결과율(`success`)만 낮아지므로 **정책 문제로 오진**하기 쉽다.
  → **이벤트율과 결과율을 항상 분리 보고**하고 `released_not_delivered` 같은 **불일치 카운터**를 둘 것.
- **래치되는 순간 리셋이 같은 step 안에서 버퍼를 지운다.** step 직후 라이브 버퍼를 읽으면 100% 놓친다 —
  pre-reset 스냅샷에서 읽을 것(Rule 23d와 동일한 alias 함정).
- **무학습 베이스라인은 계측 결함 탐지기이기도 하다.** 정책과 무관한 기준점이 있어야
  "이건 정책이 아니라 계측이 틀린 것"을 구분할 수 있다.

---

## Rule 29 — 보상이 오르는데 과제 지표가 평탄하면 "페널티 회피 수렴"이다; σ 폭주 + 액션 포화가 그 동반 서명

**상세:** [[experiments/exp_024_v20_warmstart_failure]]

**근거(exp_024, v20 방위 랜덤 분포 warm-start 1000 iter):** reward −44.8 → **+10.8**로 확실히 상승했는데
`d_xy_min` 14.74 → 14.41 m · `aim_err_min` 10.63 → 10.46 m · `release_rate` 0.127 → 0.127로 **과제 지표 3종이
완전 평탄**. 상승분은 전부 `out_of_range`(−30) 회피(0.699 → 0.089)였다. 동시에 **σ 2.10 → 6.27 단조**,
**action_sat_frac 0.76 → 0.93**. deterministic eval: 학습 분포 7.5 → 13.0%(CI 겹침), **원래 분포 100.00 → 8.50%**
(catastrophic forgetting), 지배 실패가 OOR → **bad_attitude 84.5%**로 이동.

**교훈:**
- **보상 곡선만으로 학습 성공을 판정하지 말 것.** 큰 이산 페널티가 있으면 정책은 과제를 배우기 전에
  **페널티 회피라는 훨씬 쉬운 해**를 먼저 찾는다. 반드시 **과제 고유 지표**(접근 거리·조준 오차·이벤트율)를
  같이 보고, **그것들이 평탄하면 보상이 올라도 실패**다.
- **σ 단조 상승 + 액션 포화율 상승 = advantage에 방향 정보가 없다는 뜻.** 보상이 거의 균일하게 음수면
  entropy 항만 남아 σ를 밀어 올린다. 이때 **확률적 롤아웃은 그럭저럭인데 결정론적 평가는 붕괴**한다
  (평균 행동이 레일에 붙어 자세 발산) — 학습 지표와 eval의 괴리가 이 서명이다.
- **잘못된 사전지식으로 넓은 분포에 warm-start하지 말 것.** 사전지식이 분포의 상당 부분에서
  *적극적으로 틀리면* warm-start는 이득이 아니라 부채다. 완화된 분포에서 시작해
  **커리큘럼으로 넓히거나**(Rule 20), 표현 자체를 바꿔라.
- **원래 분포 성능을 반드시 같이 측정하라.** 새 분포 학습이 구 분포 능력을 파괴할 수 있다
  (100.00% → 8.50%). "새 분포에서 낮다"와 "둘 다 낮다"는 처방이 다르다.
- 조기 중단 기준으로 쓸 것: **과제 지표 평탄 + σ 단조 상승이 200~300 iter 지속되면 예산을 더 태우지 말고
  레시피를 바꿔라.** 재적응 딥(내려갔다 올라옴)과 달리 이건 회복되지 않는다.

---

## Rule 30 — 해석 모델의 "의도적 특수화"는 전제가 참인 동안만 유효하다; 전제를 깨는 커밋이 특수화를 버그로 바꾼다

**상세:** [[research/ccip_vz_omission]] / [[errors/err_20260823_ccip_vz_omission]] (2026-08-23)

**근거:** `ballistic_impact`가 낙하시간을 $t=\sqrt{2H/g}$로 계산했다. 이는 **정지 투하 특수화**이고,
docstring이 전제를 정확히 명시하고 있었다 — *"specialised to release-from-rest-vertically (vz~=0 at
release, matching the Gazebo referee which triggers at/near success)"*. Gazebo 시절엔 참이었다.
**v16(exp_019)이 물리 페이로드를 도입**하면서 페이로드가 드론의 실제 선속도를 상속받기 시작했고,
**v19가 `release_max_vz=3.0`을 허용**하며 $v_z\neq0$ 릴리즈가 정상 경로가 되었다. 예측기는 그대로였다.

결과: 릴리즈 엔벨로프에서 모델 오차의 **약 70%가 이 누락**($v_z$ 0.547 m vs 바람 0.197 m vs 항력 0.120 m,
p50). $v_z=-3$ m/s·$H=8$ m·수평 6 m/s에서 **1.62 m overshoot**. 잔차 RL이 이를 조용히 흡수하며 수렴했고,
**에러 하나 나지 않았다**. 심지어 exp_019 후속 #3과 [[research/ccip_release_decoupling]] §4에
**해야 할 일로 이미 기록되어 있었으나 이행되지 않았다.**

**교훈:**
- **"의도적 근사"와 "버그"의 차이는 전제의 진위뿐이다.** 전제를 만든 조건이 바뀌는 순간 근사는
  주석이 달린 채로 버그가 된다. 주석은 스스로를 방어하지 못한다.
- **특수화의 전제는 실행 시점에 검증 가능한 형태로 남겨라** — assert, 진단 로깅, 또는 최소한
  전제를 깨뜨릴 수 있는 **기능 플래그 쪽에 역참조 주석**을 걸어라(`payload_physics_enabled`,
  `release_max_vz`에서 예측기를 가리키도록). 예측기 쪽 주석만으로는 아무도 안 본다.
- **후속작업 노트는 실행을 보장하지 않는다.** 전제를 깨는 커밋의 리뷰 체크리스트에 들어가야 한다.
- **잔차/학습 항이 있는 시스템은 모델 오차를 은폐한다.** 학습이 흡수해버리므로 지표가 정상으로 보인다.
  → **모델 오차는 반드시 원인별로 분해해서 로깅하라.** 합계만 보면 원인을 못 찾는다.
- 시그니처 수정 시 **기본값을 주지 말 것.** `vel_z`를 필수 인자로 승격해 모든 호출부가 깨지게 만들면
  "조용히 옛 경로 유지"가 구조적으로 불가능해진다.
- **부호 규약을 테스트로 고정하라.** ENU UP-positive에서 하강은 $v_z<0$이고 낙하시간이 **짧아진다**.
  하향 양수로 착각하면 방향과 크기가 둘 다 틀린다(실제로 1차 분석에서 틀렸다).

---

## Rule 31 — 상한선을 근사식으로 정의하면 그것은 상한선이 아니다; 오라클은 플랜트와 같은 적분으로 정의하라

> ⚠️ **2026-08-27 후속 정정 — "상한선"이라는 표현은 과했다.**
> T3는 T2와 **같은 릴리즈 규칙**에 예측기의 공칭 파라미터만 참값으로 치환한 것으로,
> **모수 불확실성 하나만 격리하는 정보 ablation**이지 달성 가능 성능의 상한이 아니다.
> 실측이 이를 반증한다 — DR_SCALE 0/0.5에서 **T2가 T3를 이겼다**
> (74.2% vs 69.5%, 77.3% vs 71.9%). 진짜 상한선은 질 수 없다.
> 아래 본문의 "상한선"은 **"T2와 같은 규칙에 참값을 준 참조 조건"**으로 읽을 것.
> 확정된 문안: [[research/research_architecture]] §1.4.

**상세:** [[research/t3_oracle_entrainment]] / [[errors/err_20260827_payload_drag_body_frame]] (2026-08-27)

**근거:** T3 "wind-oracle" 베이스라인은 참 바람·참 탄도계수를 쥐여준 **이론적 상한선**이어야 했는데
`baseline_drop.py`가 보정량을 `ballistic_impact`의 해석식 바람항 $(v+w)\cdot t$로 계산했다.
이 식은 **"페이로드가 분리 즉시 바람 속도에 100% 실린다"**(즉시 완전 엔트레인먼트)를 가정한다.
실제 플랜트는 드론 속도로 출발해 항력으로 서서히 끌려가며, 시정수 $\tau=m/(k\lVert v_{air}\rVert)\approx2.5$ s에
낙하는 $\approx1.0$ s라 **바람의 1/4 정도만 엔트레인**된다.

결과: 오라클이 **3.7~4.7배 과보정**하고, 게이트 판정과 조종 목표점 **양쪽에** 그 값을 써서
상풍 1.5 m 지점으로 날아가 투하했다. 페이로드는 0.26 m만 흘러 **1.24 m 상풍에 착탄** —
아예 보정하지 않는 것보다 나빴다. **T3 47.5% < T0 hover 91.5%**, 즉 상한선이 하한선 아래.
"학습이 T2→T3 간격의 X%를 회복했다"는 논문 문장에서 **간격이 음수**였다.

수정 과정에서 더 큰 결손이 드러났다: **무풍에서도 실제 착탄이 예측보다 1.15 m 짧다.**
지배 성분은 바람이 아니라 **페이로드 자기 속도에 대한 항력**(6 m/s에서 −1.00 m)이었고,
이것은 드론이 자기 속도를 아는 이상 **원리상 계산 가능한 항**이다 — Rule 30의 $v_z$ 누락과
같은 종류의 결손이 하나 더 남아 있었던 것.

**교훈:**
- **오라클·상한·참조 구현은 플랜트와 같은 방정식·같은 파라미터·같은 적분기로 정의하라.**
  근사식으로 정의한 상한선은 "근사가 좋은 동안만" 상한선이고, 언제 나빠졌는지 알려주지 않는다.
  전방 적분은 느리지만 **구성상 정확**하므로 별도 검증이 필요 없다.
- ⚠️ **진단 신호: 특권 정보를 준 조건이 안 준 조건보다 나쁘면, 정보가 부족한 게 아니라
  정보를 소비하는 모델이 틀린 것이다.** 이 신호는 이미 Table 1에 찍혀 있었으나
  "오라클도 완벽하진 않다"로 해석되고 넘어갔다.
- **결정론적이고 계측 가능한 오차를 잔차/학습에 맡기지 마라.** 잔차가 조용히 흡수해버려
  지표가 정상으로 보이고, 진짜 학습 대상(관측 불가 외란)의 크기를 알 수 없게 된다.
  모델 오차는 **원인별로 분해해 로깅**해야 한다(Rule 30과 동일한 처방).
- **외력 API의 프레임 기본값을 확인하라.** `set_external_force_and_torque`/
  `permanent_wrench_composer.set_forces_and_torques`는 둘 다 `is_global=False`(링크 프레임)가
  기본값이다. 월드 프레임에서 조립한 힘을 그대로 넘기면 **에러 없이** 바디 자세만큼 회전된다.
  같은 파일 기체 쪽 코드는 올바르게 변환하고 있었다 — **규약을 아는 것으로는 부족하고 한 곳에 묶어야 한다.**

---

## Rule 32 — 잔차가 못 고치는 오차부터 걷어내라: 판정 격자가 잔차보다 큰 오차를 만들고 있었다

**증상.** exp_025가 랜덤화를 완전히 끈 상태(DR_SCALE 0)에서 CEP50 0.57 m를 측정했다.
표적은 참값, 탄도 모델 오차는 p50 0.010 m. **잔차가 원리상 손댈 수 없는 오차가
잔차가 배울 오차의 3배였다.**

**원인.** 릴리즈 판정이 정책 주파수(10 Hz)에서만 일어났다. 순항 4 m/s에서 판정
스텝당 0.4 m를 이동하므로 "언제 던질까"의 선택지가 0.4 m 간격 격자 위에만 있었다.
조준이 아무리 정확해도 격자 사이의 최적 시점은 고를 수 없다.

**처방 — 결정의 주파수와 실행의 주파수를 분리하라.** 정책은 10 Hz로 *커밋*하고
(pickle-and-hold), 발사 *시점*은 물리 주파수 솔버가 고른다: 예측 탄착점이 표적을
통과하는(= 조준 오차가 감소를 멈추는) 서브스텝에 발사. 실제 CCIP 폭격 시스템의
동작이다. 실측 CEP50 0.762 → 0.395 m ([[experiments/exp_026_release_rate_100hz]]).

**파생 규칙.**
- **오차 예산을 "잔차가 고칠 수 있는가"로 분류한 뒤, 못 고치는 항부터 제거하라.**
  못 고치는 항이 남아 있으면 스윕이 평탄해 보이고 학습 효과가 묻힌다 (Rule 31과 같은 처방).
- ⚠️ **판정 해상도를 올리면 그 해상도가 이점의 전부였던 베이스라인이 무너진다.**
  T1(단순 임계)과 T2(AeroThrow horizon-argmin)가 100 Hz에서 **완전히 동일**해졌다.
  argmin은 "통과 순간"을 10 Hz로 샘플링한 것이므로 당연한 결과다. 결함이 아니라
  결과이지만, **Table 1의 팔 구성을 바꾸므로 사전에 알고 있어야 한다.**
- **비교 대상 baseline을 개선하는 변경은 논문 기여를 깎을 수 있다.** 그래도 해야 한다 —
  안 하면 "우리 방법이 좋다"가 "우리 베이스라인이 나빴다"와 구분되지 않는다.
  물러설 길(`--release_10hz`)을 ablation 축으로 남길 것.

---

## Rule 33 — 사건 시점의 양은 사건에서 래치하라; 종단 스냅샷의 `final_*`은 "사건 시점"이 아니다

**증상.** L0의 "릴리즈 속도"를 1.39 m/s로 보고했다. 이 값으로
"정책이 감속해서 place로 퇴화했다", "명시적 투하속도 보상항이 필요하다"는
결론 두 개를 세웠다. **둘 다 틀렸다. 실제 투하 속도는 3.42 m/s다.**

**원인.** `eval_harness`가 쓰던 `final_speed_xy`는 종단 스냅샷의 `final_vel_xy`,
즉 **에피소드가 끝난 시점**의 드론 속도다. 이 태스크는 **페이로드 착지로 종료**되므로
그 시점은 투하보다 $(\tau + t_{fall}) \approx 1.0$ s 뒤이고, 드론은 그 사이에 감속한다.
이름이 `final_`이라 "최종 상태"인 것은 맞지만 **"릴리즈 상태"는 아니었다.**

**처방.**
- **사건 조건부 지표는 그 사건이 발생하는 코드에서 래치하라** (Rule 21의 확장).
  여기서는 `request_release()` — 학습 팔과 스크립트 팔이 **모두 통과하는 유일한
  깔때기** — 안에서 `_release_speed_xy` / `_release_vz` / `_release_alt`를 래치한다.
  깔때기가 하나이므로 모든 팔이 같은 정의로 측정된다.
- ⚠️ **종단이 사건과 다른 순간이면 `final_*`을 사건 지표로 쓰지 마라.**
  종단이 곧 사건이던 시절(release-terminal)에는 맞는 값이었고, **종단을 착지로 바꾼
  커밋이 이 지표를 조용히 무효화했다** — Rule 30과 같은 구조의 실패다.
- **분류(taxonomy) 판정은 스케일 자유 양으로 하라.** throw/drop/place는
  속도가 아니라 **전진거리** $v_{rel}(\tau + t_{fall})$로 판정한다. 속도만으로는
  기체 크기·투하 고도가 다른 연구와 비교할 수 없다.

---

## Rule 34 — 잔차를 넣기 전에 오차 예산에서 "계통분"이 얼마인지 먼저 재라

**증상.** 바람과 L0 착지오차의 상관이 $r = 0.495$로 뚜렷했다. "잔차가 고칠 것이 많다"로
읽고 L1로 넘어가려 했다. 그런데 같은 데이터에서 **바람을 완전히 제거해도
`success@r=1.0`은 92.0 → 93.5%(+1.5 pp)** 밖에 오르지 않는다.

**두 가지가 겹쳐 있었다.**

1. **지표 포화.** 판정 반경 1.0 m가 CEP50 0.319 m의 **3.1배**다. 바람 4 m/s 미만의
   모든 구간이 이미 100%다. 같은 반사실을 CEP50으로 재면 −33%로 크게 보인다.
   → **판정 반경이 CEP50의 2배를 넘으면 성공률은 정밀도 지표가 아니다.**
   정밀도는 CEP로, 성공률은 배달률/안전 지표로 쓴다.
2. **평균 0 잡음은 잔차로 안 줄어든다.** 잔차는 정보를 만들지 못한다.
   오차 예산을 **"계통(학습 가능) / 평균 0 잡음(불가) / 약관측(부분 가능)"** 으로
   분류하라. 이 프로젝트에서는 릴리즈 지연 편차(RMS 0.186 m)와 탄도계수가
   **원리적으로 잔차 밖**이다 — 에피소드마다 i.i.d.로 뽑히고 투하 후에야 드러난다.

**처방.** 잔차 도입 전 세 줄짜리 계산을 먼저 하라:
$E[e^2]$를 각 오차원의 제곱에 회귀 → 계통분 비율 → **그 비율이 지표의 측정 한계보다
큰지** 확인. 크지 않으면 **잔차가 아니라 지표를 먼저 고쳐야 한다.**

→ [[research/error_budget_l0]] · [[research/paper_metrics]] · [[research/residual_ceiling]]

---

## Rule 35 — 상한선은 **비교 대상과 같은 조건 위에서** 정의하라

**증상.** T3(스크립트 비행 + 참값 오라클)의 바람 민감도가 0.068, L0가 0.071이었다.
"L0가 이미 오라클 수준에 도달했으니 잔차의 여지가 없다"고 두 번 결론지었다.
**두 번 다 틀렸다.**

**원인.** T3는 **다른 비행 프로파일**(6.0 m 수평 투하, 1.4~2.8 m/s)이다.
그 팔의 바람 민감도는 *그 팔의* 바닥이지 L0의 바닥이 아니다.
같은 정책 위에 참값 잔차를 주입해 재니 진짜 바닥은 **0.0056** — L0는 그 12배였고
**바람 오차가 92% 남아 있었다.**

**처방.**
- **상한선 팔은 비교 대상과 비행·투하 조건이 같아야 한다.** 우리 경우
  `play.py --oracle_residual`(같은 정책 + 참값 잔차 주입)이 그것이고,
  스크립트 T3는 상한선이 아니라 **그냥 다른 팔**이다.
- ⚠️ **상한선이 알고 있는 정보가 학습으로 도달 가능한지 따져라.**
  전지적 오라클은 에피소드별 릴리즈 지연·탄도계수까지 아는데, 그것들은
  **투하 후에야 드러나므로 어떤 잔차도 알 수 없다.** 그것까지 포함한 상한선은
  도달 불가능한 목표다 (`--oracle_residual_wind_only`가 공정 상한).
- **반사실 산술로 상한선을 대체하지 마라.** 이 프로젝트에서 산술 추정은
  −33% → −12% → 실측 −31%로 두 번 다 틀렸다.

→ [[research/residual_ceiling]]

---

## Rule 36 — 진단 지표 하나로 팔을 고르지 마라; baseline에는 정책이 가진 자유도를 전부 줘라

**증상 A.** "바람 민감도"를 팔 비교의 지표로 쓰려 했다. 그런데 T2를 밴드 바닥에서
던지게 하니 **기울기가 0.0584로 L0(0.0711)보다 낮아졌는데 CEP50은 0.358로 더 나빠졌다.**
바람 노출은 줄지만 게이트가 아슬아슬해져 다른 오차가 늘어난다.
→ **기울기는 진단이고, 판정은 총 오차(CEP)와 배달 시간으로 한다.**

**증상 B.** L0는 투하 고도를 **스스로 고르는데**(3.53 m) 스크립트 팔은 기본값 6.0 m에
고정돼 있었다. 바람 표류가 $t_{fall}^2$에 비례하므로 **손해를 떠안긴 채 비교**하고 있었다.
고도만 열어줘도 T2@1.5가 CEP50 0.457 → **0.287**(−37%), 성공률 68.2 → **74.7%**.

**처방.**
- **baseline에게 정책이 가진 자유도를 전부 주고, 그 중 최적 구성과 비교하라.**
  우리 경우 (패스 속도, 투하 고도, 하강률). 선례: Zhai et al. 2026 Fig. 5도
  baseline의 튜닝 파라미터를 스윕해 Pareto front를 그린다.
- **못 주는 자유도가 무엇인지 명시하라.** T2는 그 값들을 *에피소드마다* 바꿀 수 없다.
  그게 학습이 사는 자리이고, 못 주는 것이 정당하다.
- ⭐ **부수 발견:** 정책의 동작점을 스크립트에 **이식하면 오히려 나빠진다**
  (T2 다이브 CEP50 0.287 → 0.358). 정책의 선택은 나머지 전부와 함께 최적화된 것이라
  한 조각만 떼어 고정 규칙에 넣을 수 없다 — 학습의 근거 중 하나.

→ [[experiments/exp_027_seen_unseen_3seed]] · [[research/error_budget_l0]] §3

---

## Rule 37 — 학습기를 돌리기 전에 "관측에 정보가 있는가"를 회귀로 먼저 재라

잔차가 안 오를 때 원인은 두 가지다: **관측에 정보가 없거나**, **학습 설정이 나쁘거나.**
PPO를 돌려서는 이 둘을 구분할 수 없다. 지도 회귀 하나면 몇 분 만에 갈린다.

**절차.** 동결 정책을 **추론만** 굴려 `(관측, 특권 라벨)` 쌍을 덤프 → MLP를 MSE로 적합
→ 반출 $R^2$. 라벨은 이미 오라클 함수로 존재한다 (`oracle_impact_residual(wind_only=True)`).

**우리 측정 (exp_028).** 상수 0.000 · linear(obs) 0.127 · **MLP(obs) 0.436** ·
MLP(obs+tilt 누적) 0.611 · MLP(obs+참바람) **0.998**.
→ 관측은 **장님이 아니다**. 그리고 정보는 **비선형으로** 들어 있다(선형은 13%뿐).

**주의 두 가지.**
- **분할은 에피소드 단위.** 한 에피소드의 프레임들은 같은 외란 추출을 공유하므로
  프레임 단위 무작위 분할은 라벨 누수다.
- **채점 프레임을 명시하라.** "들고 있는 전 구간" 0.436 vs "릴리즈 프레임" 0.309.
  결정에 쓰이는 것은 후자다.

→ [[research/residual_observability]] · [[experiments/exp_028_l1_sl_pilot]]

---

## Rule 38 — 첫 교차로 발사하는 게이트 앞에서, 예측의 흔들림은 계통 편향이 된다

**증상.** RMSE 0.24 m짜리 잔차를 오라클과 **같은 경로**로 넣었더니 CEP50이
0.296 → **0.814 m**로 악화했다. 배달률·추락·자세이탈은 **완전히 동일** — 비행은
안 바뀌고 **릴리즈 시점만** 바뀌었다. 배달 시간이 5.79 → 5.63 s로 **짧아졌다**.

**원인.** 게이트는 $\min\{t:\hat{x}_{impact}(t)\ \text{crosses target}\}$ 로 발사한다.
접근 중 예측이 표적을 단조롭게 쓸고 지나가므로, 교차 시점은 예측 오차가 접근 방향으로
**가장 크게 튄 스텝**에서 정해진다. 평균이 0이어도 **최소값 통계는 0이 아니고**,
편향의 부호는 항상 "일찍"이다.

**계측.** 참 드리프트의 스텝간 변화 **0.018 m/step**. 학습된 예측기는 **0.073~0.129**
(4~7배). EMA α=0.3이면 0.023~0.036까지 내려가고 **RMSE는 오히려 좋아진다**.

**처방.** ① 주입 시 EMA(`--sl_ema`), ② 적합 손실에 시간 평활 항(원인 처치),
③ 게이트 히스테리시스(후순위).

**RL에도 그대로 적용된다** — 탐험 노이즈가 얹히면 더 튄다. `task_env`의 smoothness 항은
잔차 채널을 **명시적으로 제외**하고 있다("기체가 slew할 대상이 아니다"). 그 주석은
**기체 관점에서는 옳지만 게이트 관점에서는 틀렸다.**

⭐ **따라서 오라클이 잰 상한선(−31%)은 "매끄러운 잔차"에 대한 상한선이다.** 학습된
잔차가 못 미칠 때 부족한 것이 **정보인지 매끄러움인지** 구분해서 보고하라.

→ [[research/release_gate_jitter]] · [[experiments/exp_028_l1_sl_pilot]]

## Rule 39 — 잔차 회귀기는 그것을 배포할 정책의 데이터로 적합하라

**증상.** seed-1 정책의 롤아웃으로 적합한 obs→drift 회귀기를 **seed-2 정책**에 주입했더니
CEP50은 −7% 좋아졌는데 **succ@1.0 96.0 → 89.3%**, **CEP90 0.542 → 0.830 m**, 평균 오차 +18%.
순손실이다. seed-2 자체 데이터로 재적합하면 CEP50 −22.0%, CEP90 0.562로 꼬리가 복구된다.

**원인.** tilt 누적이 재는 것은 바람이 아니라 **"이 컨트롤러가 이 바람에 얼마나 기울어지는가"**
이다. 정상상태 자세 편향 $\bar\theta \approx F_{wind}/K_{policy}$ — 회귀기는 $K$를 상수로
흡수해 학습한다. 다른 정책은 다른 $K$이므로 **계통적으로 잘못 보정된 이득**이 되고,
잡음이 아니라 편향이라 큰 바람에서 크게 틀린다.

**정보 문제가 아님의 증거.** ① 오라클은 같은 정책에서 −30.2%를 낸다(주입 경로 정상),
② seed-2의 회귀 $R^2$가 오히려 더 높다(obs+tilt 0.631 vs 0.611), ③ 재적합으로 복구된다.

**처방.** 정책을 바꾸면(재학습·미세조정·게인 변경) **잔차를 무효로 간주하고 다시 적합**하라.
수집은 추론 전용이라 몇 분이지만, 논문에 명시해야 할 제약이다. 이것은 **L1-RL의 실질적 장점**
(공동 적응이라 구조적으로 결합이 없음)이므로 비교 절에 그렇게 적을 것.

→ [[research/residual_policy_coupling]] · [[experiments/exp_029_l1_sl_generalization]]

---

## Rule 40 — 잔차 arm을 CEP50 하나로 판정하지 마라; 꼬리가 먼저 망가진다

**증상.** 잘못 보정된 잔차(전이 회귀기, 또는 라벨 100개짜리 회귀기)는 **CEP50이 개선된 채로**
succ@1.0과 CEP90을 망가뜨린다. 라벨 100개 arm: CEP50 −5.5%(개선)인데
CEP90 0.593 → **0.850**, succ@1.0 93.8 → **87.3%**.

**원인.** 오보정은 잡음이 아니라 **계통 이득 오차**다. 중앙값 근처(작은 바람)에서는 절대 오차가
작아 살아남고, 분포 꼬리(큰 바람)에서 크게 틀린다. 그래서 손상이 CEP90에 먼저 나타난다.

**처방.** 잔차 arm은 **CEP50 · CEP90 · succ@1.0을 함께** 보고하라. CEP50만 보면 순손실인
설정을 이득으로 오독한다. 라벨이 임계 미만이면 잔차를 **끄는 것**이 옳다
(이 환경 기준 임계 ≈ 1,000 에피소드; 500 미만은 순손실 위험).

→ [[research/residual_label_efficiency]] · [[research/residual_policy_coupling]]

---

---

## Known Failure Modes

| 증상 | 원인 | 해결 |
|------|------|------|
| **eval ep 연속 step1 `d_xy≈11.9m` −15 truncation** (정책 정상인데 못 빠져나옴) | **(06-21 규명) 누적 leaked YOLO `xmarker_detector`(restart마다 spawn, kill 안 함 → 3개) 충돌 탐지 → spurious TRACKING + EKF↔camera 불일치.** clean slate에선 정상(handoff 0.9m). | ① fresh-start kill에 `xmarker_detector` 추가, ② 에피소드 시작 health gate(불일치 retry), ③ 장기 run 후 clean teardown 후 평가 → [[research/eval_terminal_env_metrics]] (Rule 12) |
| **eval miss-distance/CEP 전부 NaN** | `evaluate.py`가 env 미emit 키 `info['drop_error_actual_m']` 의존; v13 env는 0.8m 종료(탄도 투하 없음) | success_rate/step-to-reach로 지표 교체 → [[research/eval_terminal_env_metrics]] (Rule 12) |
| `mean_rew_dist = 0` | 지수 포텐셜 포화 (k1 너무 큼) | 선형 보상 사용; $e^{-k_1 d_{max}} > 10^{-6}$ 확인 |
| `mean_d_xy` → 1e11 | Gazebo ODE 물리 폭발 | 3중 방어 레이어 → [[errors/err_20260320_physics_explosion]] |
| CRUISE 타임아웃 (~42% late-arm) | teleport 후 EKF 재수렴이 bimodal (0s 또는 13–16s). `pre_flight_checks_pass`가 늦게 True. **v12의 10s 컷이 복구 직전 단두대질** → full restart 강제 (진짜 throughput 싱크) | pre_flight_checks_pass 게이팅 + `arm_bail_timeout` **10s→20s** (복구 곡선 계측 후) → [[research/cruise_timeout_arming]] (Rule 8, **Rule 11**) |
| `ep_rew_mean` 나선형 하락 | (1) CRUISE 타임아웃 버퍼 오염 **또는** (2) ep_len 붕괴 착시 (도달 빨라짐) | 근본 원인 수정; per-episode `env/ep_reward`로 진짜 신호 판정 (Rule 9) |
| **ep_len 감소 + ep_rew 음수 고정 + success 0** (d_xy는 잘 도달) ⚠️ v12 | 정하방 카메라 → 핸드오프 ~1m → overshoot 가드(threshold 1.5)가 step 1부터 무장 + 8 m/s 액추에이터가 0.5m 성공원 지나침 → 매 에피소드 -20 | overshoot 무장 거리 < 핸드오프, success_radius 도달가능하게, 액션 스케일 ↓ → [[research/terminal_overshoot_trap]] (Rule 10), `hyperparams_v13.yaml` |
| **YOLO `target_lost_rate` ~29% bimodal (악화 중)** ⚠️ OPEN | per-step YOLO 트리거가 에피소드별 전부-탐지(rate=0, 70.7%) 또는 전무-탐지(rate=1, 29.3%)로 분리; partial 0%. 추세 0.24→0.35 | **미해결.** ~29% step에서 obs[9-11] zeroed + `-10` 페널티. 별도 처리 필요 → [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] |
| fps 급감 | CRUISE 타임아웃 (65 s 대기) 또는 ODE 크래시 | 로그에서 "Timed out waiting for CRUISE" 확인 |
| **드론이 마커 거울상으로 비행** (East 부호 반전) | East 타겟이 -11(거울)로 설정됨. PX4 East = +Gazebo_East (반전 없음)인데 반전 가정함 | `target_ned_y=+11`, `cruise_speed_y=-1`, `target_enu_x=+11`. ⚠️ d_xy 로그는 거울상 자기일치로 속임 → `gz model -p` ground-truth 검증 필수. 상세: [[coordinate-frames]] / [[research/ekf_east_reversal]] (06-12 진단 RETRACTED) |
| YOLO 탐지 무효 (silent) | ultralytics Boxes boolean 인덱싱 silent fail | `detections[:0]` 정수 슬라이스로 대체 |
| **eval success 0% (학습/wandb는 정상)** | done-flag가 `_just_released` 등 reset이 in-place 변조하는 버퍼의 **alias**로 캐시됨 — `_reset_idx`가 step() 반환 전에 지움 | 종단 플래그 캐시는 `.clone()` (Rule 23d) → [[research/release_terminal_stageB]] |
| 이중 YOLO 노드 실행 | 수동 기동 + env 기동 중복 | env가 YOLO를 `_infra_procs`로 관리; 추가 수동 기동 금지 |

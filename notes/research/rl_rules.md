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

> **Phase 1 전체 계획:** [[research/phase1_plan]] — CCIP 기반 자율 접근, 8주, 14개 실험

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
| 이중 YOLO 노드 실행 | 수동 기동 + env 기동 중복 | env가 YOLO를 `_infra_procs`로 관리; 추가 수동 기동 금지 |

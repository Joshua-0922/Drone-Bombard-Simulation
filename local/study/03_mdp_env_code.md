# 03. MDP & DroneDropEnv 해부

> 카테고리 3. `drone_drop_env.py` (1835 줄) 의 핵심 — observation, action, transition, reward, termination 을 줄별로.

---

## Session 3.1 — MDP 4 요소 한 줄 정의

| 요소 | 우리 환경에서 | 코드 위치 |
|---|---|---|
| **State (S)** | 드론 pos(ENU) + vel + ang_vel + 비전 + payload_attached + rel_target + CCIP impact/t_f | `_get_obs()` line 1222 |
| **Action (A)** | (vx, vy, vz, yaw_rate, drop_trigger) — drop_trigger 는 무시됨 | `step()` line 744-749 |
| **Transition (P)** | PX4 + Gazebo + 100 ms step (10 Hz) — deterministic 처럼 보이지만 EKF noise + step-internal Hz drift 로 stochastic | `step()` line 750-753 |
| **Reward (R)** | 4-Layer Hierarchical (Safety / Stability / Approach / Terminal) | `_compute_reward()` line 1094, step()의 drop branch line 836-905 |

핵심: **state 는 17 차원 정규화 vector, action 은 5 차원 [-1, 1] continuous box, 단 [4] (drop) 는 dead dimension**.

---

## Session 3.2 — Observation 17 차원 한 줄씩

```python
obs = [
    pos_n[0], pos_n[1], pos_n[2],     # 0-2:  드론 위치 (ENU) / 50  → [-1, 1]
    vel_n[0], vel_n[1], vel_n[2],     # 3-5:  드론 속도 (ENU) / 15  → [-1, 1]
    ang_n[0], ang_n[1], ang_n[2],     # 6-8:  각속도 (body) / π    → [-1, 1]
    u_norm, v_norm, conf,             # 9-11: 비전 pixel u, v, 신뢰도
    attached,                         # 12:   payload 부착 여부 (0.0 또는 1.0)
    rel_dx, rel_dy,                   # 13-14: 타겟 상대 위치 / 50
    obs_d_impact,                     # 15:   CCIP 예측 miss / 50  → [0, 1]
    obs_t_f,                          # 16:   CCIP 비행시간 / 10s  → [0, 1]
]
```

### 직관

- **[0-2]** 절대 위치: 학습 안정성을 위해 ±50 m 로 normalize. world 가 ~150 m 이므로 끝부분은 saturate.
- **[3-5]** 속도: drone control 의 핵심 신호. ±15 m/s 가 정상 범위 (overspeed limit 20 m/s).
- **[6-8]** 각속도: 자세 변화 속도. drop 직전 안정성 진단에 중요. π rad/s 가 빠른 회전 기준.
- **[9-11]** 비전: 우리 RL 모드는 `use_vision=False` → u=v=0, conf=1 (synthetic). vision 모드만 진짜 값.
- **[12]** attached: payload 분리 후 0 으로 떨어짐. **drop 이후 step 이 있다면 이 bit 이 transition 신호**.
- **[13-14]** 타겟 상대 위치: 절대 위치 [0-2] 와 중복처럼 보이지만 **target 좌표를 obs 에 직접 박지 않고 차이로 표현** — 정책 일반화 (다른 target 으로 평가 시 transferable).
- **[15]** d_impact: **CCIP 예측 — 지금 drop 하면 타겟에서 얼마나 빗나갈까**. 정책에 "drop 타이밍 정보" 제공. 코드 `_predict_impact_point()` 가 free-fall + 수평 등속 가정으로 계산.
- **[16]** t_f: **언제 drop 하면 정확할까** 의 시간 거리. 보통 0~10 s. 둘 (15, 16) 이 함께 있을 때 정책이 "drop timing window" 학습 가능.

### Vision dimension 처리의 미묘함

`use_vision=False` 인데도 `conf=1.0` synthetic. **이유: SB3 의 obs normalizer 가 conf 분포를 학습해 두는데 0 으로 두면 분산이 0 → normalizer 가 폭주.** 카메라 안 써도 "보인다" 라고 말해 줘야 obs space 가 항상 일관됨.

---

## Session 3.3 — Action 5 차원 (사실상 4 차원)

```python
vx       = action[0] * action_vx_scale     # ±15 m/s (default)
vy       = action[1] * action_vy_scale     # ±5  m/s
vz       = action[2] * action_vz_scale     # ±3  m/s
yaw_rate = action[3] * action_yaw_scale    # ±1  rad/s
drop     = action[4]                       # 무시됨 (drop-ASAP exploit 차단)
```

핵심 포인트:

### Action scale 의 비대칭

| 축 | scale | 의도 |
|---|---|---|
| vx | 15 | 전진 (target 방향) — 빠른 cruise 허용 |
| vy | 5  | 측면 보정 — 너무 빠르면 toss 궤적 망함 |
| vz | 3  | 수직 — drop 직전 미세 고도 조정용 |
| yaw_rate | 1 | 거의 직진 가정. 헤딩만 천천히 |

→ **이 scale 자체가 정책에 박는 prior**. "빠르게 전진하고 좌우/상하는 조심해라" 가 design choice.

### Drop trigger 가 dead 인 이유

코드 주석: `action[4] ignored (manual drop disabled — causes "drop ASAP" exploit)`.
- 정책이 drop 을 직접 결정 가능하면 학습 초반에 **"즉시 drop → terminal 보상 한 번 받기"** 가 local optimum.
- 그래서 drop 결정은 환경이 하드코딩: **CCIP 예측 d_impact ≤ 3 m 이면 자동 drop**.
- 정책은 trajectory 만 학습 → CCIP 가 자연스럽게 작아지는 방향으로 비행하면 drop 됨.

### Action rate limit (P2 — junsang_v4)

```python
action = np.clip(
    action,
    self.action_prev - 0.2,
    self.action_prev + 0.2,
)
```

매 step 당 |Δa| ≤ 0.2. velocity setpoint 의 가속도 hard cap. **이게 toss 학습의 양날의 검**:
- 좋은 면: 폭주 (action 0 → 1 → 0 → 1) 차단, 자세 안정.
- 나쁜 면: toss 의 "pitch back" 동작이 빠른 ang 변화 요구 → 학습 어려움. 우리 v9a 가 angaccel penalty 추가했지만 실패.

---

## Session 3.4 — Step transition 흐름

```
step(action):
  ┌────────────────────────────────────────────────────────────┐
  │ 1. action rate limit (clip to ±0.2 from prev)              │
  │ 2. publish_velocity(vx, vy, vz, yaw_rate)                  │
  │    → ROS2 topic /drone/cmd/velocity                        │
  │    → drone_controller → /fmu/in/trajectory_setpoint        │
  │    → PX4 cascade [2]~[5] → motor PWM                       │
  │    → Gazebo physics → 새 위치/속도                          │
  │ 3. obs_ready.wait(timeout=0.15)  ← PX4 → ROS2 callback     │
  │ 4. snapshot state (lock 보호)                              │
  │ 5. _ang_vel_history.append                                 │
  │ 6. d_xy, d_3d, d_impact, t_f 계산                          │
  │ 7. hover counter 갱신                                       │
  │ 8. physics glitch guard (NaN, > 500m)                       │
  │ 9. drop 판정 (auto + random + hover_block)                  │
  │    ├─ drop 발동: Layer 4 reward, terminated=True           │
  │    └─ no drop: Layer 1-3 reward (_compute_reward)          │
  │ 10. safety termination (crash, overspeed, ang_vel, ...)     │
  │ 11. step limit truncation                                   │
  │ 12. hover truncate                                          │
  │ 13. hover penalty                                           │
  │ 14. action_prev 갱신                                        │
  │ 15. reward hard cap [-200, 300]                             │
  │ 16. _get_obs() → 새 17-d obs                                │
  │ return (obs, reward, terminated, truncated, info)           │
  └────────────────────────────────────────────────────────────┘
```

### "10 Hz" 의 실체

- env step 의 wall-clock 주기 = `obs_ready.wait(timeout=0.15)` 가 사실상 결정.
- PX4 가 `vehicle_local_position` 을 100 Hz 정도로 publish → 평균 10 ms 마다 callback → `_obs_ready.set()`.
- 실제 step 시간은 ~100 ms (action 처리 + wait + 계산) → **약 10 Hz**.
- env step Hz 는 코드에 명시 안 됨 — wait timeout + callback rate 의 emergent property.

### Timeout 의 미묘함

`obs_wait_timeout=0.15` 가 의미: PX4 가 150 ms 안에 새 obs 안 주면 그냥 마지막 값으로 진행. → DDS 끊김 / PX4 freeze 시 silent stall 방지.

---

## Session 3.5 — Reward 4 레이어 해부

### Layer 1: Safety (per-step penalty, 종료 안 함)

| 조건 | 페널티 | 코드 |
|---|---|---|
| 고도 < ground_contact_alt (0.5 m) | penalty_crash (-50) | line 1118 |
| 고도 < min_altitude (2.0 m) after start | penalty_crash (-50) | line 1121 |
| speed > V_MAX_SAFETY (20 m/s) | penalty_overspeed (-30) | line 1125 |
| vision conf == 0 (use_vision=True 일 때만) | penalty_target_lost (-10) | line 1131 |

→ Layer 1 은 **penalty 만 적용, 종료는 step() 의 truncate 로직이 처리**.

### Layer 2: Stability (per-step, 효율)

```python
r2 = (
    -0.05                                     # time penalty (step 비용)
    - w_ang_vel * ||omega||²                  # 각속도 패널티 (default 0.05)
    - w_action_smooth * ||Δa||²               # action smoothness (default 0.05)
)
```

→ "시간 비용 + 자세 흔들면 페널티 + action 급변하면 페널티". 안정한 비행 권장.

### Layer 3: Approach (per-step, 타겟 접근)

```python
r3_dist     = w_dist * (d_prev - d_now)               # 거리 감소량 × 1.0
r3_orient   = w_heading * cos(heading, bearing) * speed_gate   # 헤딩 정렬 × 0.7
r3_impact   = w_impact * exp(-k * d_impact)           # CCIP shaping (default 0)
r3_distance_penalty = -w_distance_penalty * d_xy / 50  # 멀수록 hover 비용
```

핵심:

- **r3_dist (linear)**: 이전 step 대비 가까워진 거리. 매 step 의 incremental reward.
  - 과거엔 exp(-k1*d) 사용했으나 d>10m 에선 ~0 (saturate) → linear 로 교체.
  - "어떤 거리에서도 nonzero gradient" 가 목표.
- **r3_orient**: 진행 방향 cos. 단, **speed_gate** (속도 < 2 m/s 면 비례 감쇠) 로 "느리게 회전만 해서 reward 챙기기" (spiral milking) 차단.
- **r3_impact**: CCIP 예측 거리 기반 shaping. default 0 — v8 에서 안 씀.
- **r3_distance_penalty**: 거리 비례 페널티 (멀수록 매 step 비용). v4 시절 hover 차단용.

→ Layer 3 가 **궤적 학습의 메인**. 다른 가중치 (w_dist, w_heading) 가 v8 → v9a 튜닝의 주 타겟.

### Layer 4: Terminal Drop (drop 시 한 번에 큰 reward)

drop 발동 시 (auto: `d_impact ≤ 3m` 또는 random):

```python
# 1. Drop attempt bonus (proximity-scaled)
reward = drop_attempt_bonus * exp(-k_drop_proximity * d_xy)
       = 30 * exp(-0.15 * d_xy)
# d_xy=0 → 30, d_xy=10 → 6.7, d_xy=30 → 0.3

# 2. Base precision (drop_calculator 의 실제 physics 결과)
reward += w_drop_base * exp(-k2 * d_error)
       = 100 * exp(-0.2 * d_error)
# d_error=0 → 100, d_error=5 → 36.8, d_error=20 → 1.8

# 3. Prediction accuracy bonus (CCIP gap)
reward += w_prediction * exp(-k * |d_impact - d_error|)
       = 20 * exp(-0.1 * gap)

# 4. Jackpot
if d_error <= 0.1: reward += 50

# 5. Altitude penalty (sigmoid bounded)
reward -= alt_penalty_max * sigmoid(k * (alt - mid))
       = -50 * sigmoid(0.15 * (alt - 30))
# alt=30 → -25, alt=20 → -9, alt=40 → -41

# 6. Instability penalty (one-shot)
if ||omega|| > 2.0 or |roll/pitch| > 0.26 rad:
    reward -= 50

# 7. v9a: drop 직전 N step 의 max angular acceleration penalty
reward -= scale * max_ang_accel
# default scale=0 (v8); v9a 는 0.5 사용

# 8. Invalid drop penalty (drop_error > 50m)
if d_error > 50: reward -= 50
```

핵심 직관:

- **Layer 4 의 총합 이론 max ≈ 30 + 100 + 20 + 50 = 200**. min ≈ -100 (altitude + instability + invalid).
- 정책 입장에선 **drop 한 번이 한 episode 의 ~80%의 reward 결정**.
- 그래서 hard cap [-200, 300] 이 안전망.

---

## Session 3.6 — Termination & Truncation

Gymnasium 의 두 신호 구분:
- **terminated**: 환경의 "자연스러운 끝" (목표 달성 / 실패). bootstrap 안 함.
- **truncated**: 외부 제약 (시간 초과, 안전). value bootstrap 함.

### 우리 환경의 분류

| 신호 | 종류 | 조건 | 처리 |
|---|---|---|---|
| terminated | drop 발동 | auto_drop or random_drop | Layer 4 reward + ep 종료 |
| terminated | stage1 reached | stage1_only & d_3d < R | bonus + ep 종료 |
| terminated | physics glitch | d_xy NaN or > 500 | -100 + ep 종료 |
| truncated | crash | altitude < 2.0 m | penalty_crash 이미 적용 + ep 종료 |
| truncated | overspeed | speed > 20 m/s | penalty_overspeed 이미 + ep 종료 |
| truncated | ang_vel | ||ω|| > 2.0 rad/s | penalty_bad_attitude (-30) + ep 종료 |
| truncated | inverted | |roll/pitch| > 1.047 rad (60°) | penalty_bad_attitude + ep 종료 |
| truncated | out_of_range | d_xy > 100 m | penalty_out_of_range (-30) + ep 종료 |
| truncated | max_altitude | pos[2] > 50 m | penalty_max_altitude (-15) + ep 종료 |
| truncated | timeout | step ≥ 800 & not dropped | truncation_penalty (-15) |
| truncated | hover_timeout | consecutive_still > threshold | hover_penalty (-15, 따로 적용) |

→ **"성공" 의 정의는 terminated=True & d_error ≤ success_threshold (5m)** (info['is_success']).

### Drop 안 한 채 800 step 채우면?

→ truncated=True, penalty -15. 정책 입장에서 학습 초반에 흔한 케이스. drop attempt 자체에 +30 bonus (proximity-scaled) 가 붙는 이유.

---

## Session 3.7 — State machine (mission_manager 측)

env 코드 안에는 명시적 state machine 안 보이지만 `mission_manager_node` 에서 mission state 가 string 으로 흐름:

```
IDLE → ARMING → ARMED → TAKEOFF → CRUISE → MISSION_RUNNING → LANDED
```

env 의 `mission_state` (line 83) 가 이 string 을 받음 — env 는 CRUISE 도달까지 reset 안 끝남.

reset 시:
1. `_kill_episode_nodes()` — mission node 3 개 죽임
2. `_gz_world_reset()` — Gazebo world reset service 호출
3. (또는 fast-path) PX4 teleport
4. `_start_episode_nodes()` — mission node 재기동
5. `cruise_poll_timeout=60.0` 동안 `mission_state == CRUISE` 대기
6. 안 오면 cruise_timeout → 다음 시도

→ **reset 의 90% 시간이 CRUISE 도달 대기**. FPS 의 병목.

---

## Session 3.8 — 우리 env 의 hidden assumptions

코드에 박혀 있지만 학습이 "당연하게 따르는" 가정들:

1. **PX4_GZ_MODEL_NAME 으로 pre-spawn 모델에 연결**. PX4 가 자기 모델을 spawn 안 함. Issue #014 의 해결책.
2. **inline x500_bombard model in world SDF** (include merge="true" 가 DetachableJoint 망가뜨림).
3. **CCIP 가 free-fall + 수평 등속 가정**. 공기저항/wind 무시. 실제 d_error vs CCIP 의 gap 으로 보완.
4. **target ENU (11, 10)** — drop_calculator 와 env 양쪽이 같은 좌표 사용.
5. **action[4] (drop) 무시 + auto_drop 자동화**. 정책은 "trajectory 만" 학습.
6. **20 m/s 가 overspeed 한계**. v8 의 toss 가 vx ≈ 13 m/s 였음 — 여유 있음.
7. **drop_calculator 가 10 초 안에 결과 반환** 가정. 안 오면 default 99 m (invalid).

---

## 카테고리 3 정리 — 4 줄

1. obs 17 차원 = **물리 (위치/속도/자세) + 비전 + payload + 타겟 상대 + CCIP**.
2. action 5 차원 = **(vx, vy, vz, yaw_rate, drop)** 인데 **drop 은 dead** → 정책은 trajectory 만 학습.
3. reward 4 layer = **Safety / Stability / Approach / Terminal** — Layer 4 (drop) 가 한 episode 의 ~80% reward 결정.
4. termination 은 **drop 만이 자연 종료** (terminated). 나머지 (crash/overspeed/timeout/hover) 는 truncated.

→ 다음: **카테고리 4 — SAC + PER 알고리즘**. 우리가 쓰는 SB3 의 SAC 가 위 MDP 에서 정확히 무엇을 학습하는지.

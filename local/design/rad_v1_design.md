# RAD v1 Design — Relative Approach Drop

> **RAD** = **R**elative coordinate + **A**pproach (Phase 1) + **D**rop (Phase 2).
> 기존 Round / redux 시리즈 (single SAC, 절대좌표, 단일 정책) 와 **완전히 다른 framework**.
> Round / v8 / v9a 의 patch 가 아닌, **새 framework 의 v1**.

작성: 2026-06-30
관련:
- [model_history.md](model_history.md) — RAD v1 entry
- [../parameter_log.md](../parameter_log.md) — RAD parameter set
- [design_review.md](design_review.md) — 현재 활성 처방
- [two_stage_learning_plan.md](two_stage_learning_plan.md) — RAD 의 사용자 thoughts 원안

---

## 0. RAD 의 정의

RAD 는 다음 3 핵심으로 정의되는 새 framework:

| 핵심 | 의미 |
|---|---|
| **R**elative | obs 가 yaw-only body frame 기준 상대좌표 (절대 좌표 → 상대 좌표 전환) |
| **A**pproach (Phase 1) | sphere 진입 + 7 final state 조건 만족까지 학습. drop 없음 |
| **D**rop (Phase 2) | Phase 1 종단 distribution 에서 시작 + drop 정밀도 학습. Phase 1 weights 로 warm start (init only) |

→ **2 정책 hierarchical RL** + **상대좌표 obs** + **단계별 학습 ladder**.

---

## 1. v8 / v9a 대비 핵심 변경 (13 항목)

| # | 영역 | v8 / v9a | RAD v1 |
|---|---|---|---|
| 1 | 정책 구조 | single SAC | **2 SAC (Phase 1 + Phase 2, warm start init only)** |
| 2 | obs | 17d 절대좌표 | **14d 상대좌표 (yaw-only body frame)** |
| 3 | spawn yaw | 고정 (0°) | **Uniform(−90°, +90°) relative to drone→target** |
| 4 | cruise | 자동 이동 (vx=1, vy=−1) | **head 방향 1 m/s 가속만, 이동 폐기** |
| 5 | target 정의 | (4, 3, 5) 임시 z + drop_calculator (11, 10) default 불일치 | **(4, 3, 0) 지면 marker 통합** |
| 6 | w_dist 거리 | 2D (xy) | **3D 포함, √(x²+y²+(z−4)²) — (target_x, target_y, 4) 까지** |
| 7 | z reward | 없음 | **Hann raised cosine, z=4 max, [0.5, 7.5] 0, w_z=0.3** |
| 8 | Phase 1 종료 | 단순 drop reward | **sphere d²≤20.5 진입 + 7 final state 조건 jackpot (총 +120)** |
| 9 | drop trigger | d_impact ≤ 2.0m | **d_impact ≤ 1.0m (정밀도 강화)** |
| 10 | w_impact | 0.4, k=0.05 | **1.0, k=0.1 (강화)** |
| 11 | crash 신규 | — | **Phase 2 sphere 벗어남 d²>22 → −30** |
| 12 | per-step penalty | time −0.05, ang_vel/action 0.05 | **Phase 2: time −0.1, ang_vel 0.1, action_smooth 0.1 (2× 강화)** |
| 13 | action_rate_limit | 0.2 | **Phase 2: 0.15 (강화)** |

---

## 2. Phase 1 (Approach) 결정 누적

### 2.1 종료 조건

```
Phase 1 학습 종료:
  step ≥ 300k
  AND 최신 50k window 의 success_rate ≥ 90%
    (success = sphere 진입 AND 7 final state 조건 모두 만족)
```

| 항목 | 값 |
|---|---|
| step floor | 300k |
| window | 50k |
| threshold | 90% |
| success 정의 | sphere 진입 AND ∏(C1~C7 satisfied) = 1 |

### 2.2 좌표계

| 항목 | 값 |
|---|---|
| **Target** | (4.0, 3.0, 0.0) — 지면 marker |
| Spawn 시작 | (0, 0, 0.24) → climb 5m hover |
| Cruise 종료 시점 위치 | (0, 0, 5) hover + head 방향 1 m/s |
| 초기 거리 (3D) | √(16 + 9 + 25) = √50 ≈ 7.07m |
| **Switch sphere** | d²(drone → target) ≤ 20.5 |
| Sphere radius | √20.5 ≈ 4.53m |
| 이동 필요 거리 | ≈ 2.54m (적정 난이도) |

### 2.3 Spawn yaw 랜덤화

| 항목 | 값 |
|---|---|
| Distribution | Uniform(−π/2, +π/2) |
| Reference | drone → target vector (atan2(target_y − spawn_y, target_x − spawn_x) 기준) |
| 의미 | drone 이 target 을 정면 또는 좌/우 90° 까지 바라봄. 등 돌린 경우 제외 |
| 매 ep | 새 랜덤 yaw |

### 2.4 Cruise 메커니즘 (재정의)

| 단계 | 동작 | RL step 카운트 |
|---|---|---|
| 1. Spawn | (0, 0, 0.24), yaw = 랜덤 (위 2.3) | ❌ (mission_manager) |
| 2. Climb | target_altitude=5m 까지 ascent, altitude_hold_ticks ≥ 2 | ❌ (mission_manager) |
| 3. Cruise | head 방향 1 m/s 까지 가속. yaw 고정 | ❌ (mission_manager) |
| 4. Cruise 종료 | speed_xy ≥ 1.0 m/s 도달 시 | ❌ |
| **5. RL 시작** | **drone 의 obs (상대좌표) → 정책 → action** | ✅ **step 1 부터 카운트** |

→ **Cruise 1~4 단계는 mission_manager_rad 가 control. RL 정책 미관여 → step 카운트 안 됨**.
→ **`env.step()` 의 첫 호출 = RL 시작 시점 (cruise 완료 직후)**.
→ `max_steps=800` (Phase 1) 은 RL step 1 ~ 800 만 카운트.

기존 v8 의 cruise (vx=1.0, vy=−1.0 자동 이동) 폐기.

### 2.5 obs 14d (상대좌표, yaw-only body frame)

```
obs[0:3]  = (Δx_b, Δy_b, Δz_world) / POS_SCALE   # target - drone, body frame
obs[3:6]  = (vx_b, vy_b, vz_world) / VEL_SCALE   # drone vel, body frame
obs[6:9]  = (ωx, ωy, ωz) / ANG_VEL_SCALE         # body ang_vel (그대로)
obs[9:11] = (roll, pitch) / π                    # attitude
obs[11]   = payload_attached                     # Phase 1 dead = 1.0
obs[12]   = clip(d_impact / POS_SCALE, 0, 1)     # Phase 1 dead = 0, Phase 2 active
obs[13]   = clip(t_f / 10, 0, 1)                 # Phase 1 dead = 0, Phase 2 active (t_f clamped at 10s in predict)
```

| 항목 | 값 |
|---|---|
| obs dim | 14 |
| Body frame | Yaw-only (drone yaw 만 회전, pitch/roll 은 별도 obs) |
| 상대 위치 정의 | target − drone (target 방향 가리킴) |
| POS_SCALE | **10.0** (v8: 5 → 10, v10 좌표 √50 ≈ 7m 대응) |
| VEL_SCALE | 15 (v8 그대로) |
| ANG_VEL_SCALE | π (v8 그대로) |
| yaw 절대값 | obs 에서 제외 (정책이 yaw-invariant 학습) |

### Body frame 변환식

```
[Δx_b]   [ cos(yaw)  sin(yaw)] [Δx_w]
[Δy_b] = [-sin(yaw)  cos(yaw)] [Δy_w]
Δz_world (회전 적용 안 함)

velocity 도 동일 변환 (vx_w, vy_w → vx_b, vy_b)
vz_world (회전 적용 안 함)
```

### 2.6 Action space (변경 없음)

| idx | 의미 | scale |
|---|---|---|
| 0 | vx command | action_vx_scale = 3.0 |
| 1 | vy command | action_vy_scale = 3.0 |
| 2 | vz command | action_vz_scale = 3.0 |
| 3 | yaw_rate command | action_yaw_scale = 1.0 |
| 4 | drop trigger | dead (manual 비활성, auto 만) |

action_rate_limit: Phase 1 = 0.2 (v8 그대로), Phase 2 = 0.15 (강화).

### 2.7 Reward 함수 (per-step)

```
r_total = r_layer1 + r_layer2 + r_layer3 + r_z_hann
```

#### Layer 1 (safety, penalty only)

| 조건 | penalty |
|---|---|
| z < ground_contact_altitude (0.5m) | -50 |
| speed > V_MAX_SAFETY | -30 |
| target_lost (vision dead) | -10 |

#### Layer 2 (efficiency / stability) — Phase 1 값 (Phase 2 는 §3.4 참조)

```
r_layer2 = -0.05                  # time penalty (Phase 1: -0.05, Phase 2: -0.1)
         - 0.05 * ‖ω‖²            # w_ang_vel (Phase 1: 0.05, Phase 2: 0.1)
         - 0.05 * ‖Δa‖²           # w_action_smooth (Phase 1: 0.05, Phase 2: 0.1)
```

#### Layer 3 (approach shaping)

```
# d_reward = √((x_drone − target_x)² + (y_drone − target_y)² + (z_drone − 4)²)
#         = (target_x, target_y, 4) 까지의 3D 거리
r3_dist    = 1.0 * (d_reward_prev − d_reward_now)         # w_dist
r3_heading = 0.7 * cos(yaw_error) * speed_gate            # 수평만, speed_gate = min(v_xy/2, 1)
r3_impact  = 0.0                                          # w_impact = 0 (Phase 1 비활성, Phase 2 = 1.0 × exp(-0.1 × d_impact))
```

#### z Hann reward (신규)

```
r_z_hann = 0.3 * 0.5 * (1 + cos(π * (z − 4) / 3.5))  for z ∈ [0.5, 7.5]
         = 0                                           otherwise
```

| z 값 | r_z_hann |
|---|---|
| 4.0 (max) | +0.30 |
| 3.0 or 5.0 (mid) | +0.15 |
| 2.0 or 6.0 | +0.045 |
| 0.5 or 7.5 (가장자리) | 0 |

### 2.8 Final state 7 조건 (terminal — sphere 진입 시)

```
phase1_terminal_reward = floor
                      + Σ(w_i × satisfied_i)
                      + complete_bonus × ∏(satisfied_i)
```

| 항목 | 값 |
|---|---|
| floor | +20 |
| w_i (각 조건) | 50/7 ≈ 7.14 |
| complete_bonus | +50 |
| Total (0/7 만족) | +20 |
| Total (5/7) | +20 + 35.7 = +55.7 |
| Total (6/7) | +20 + 42.9 = +62.9 |
| **Total (7/7 jackpot)** | **+20 + 50 + 50 = +120** |
| Jackpot delta (6→7) | **+57.1** ← 마지막 조건 만족의 강력 incentive |

#### 7 조건 정의

| C# | 조건 | 임계값 | 의미 |
|---|---|---|---|
| C1 | z ∈ [3, 5] | drop 시점 고도 적정 |
| C2 | ‖v_xy‖ ≤ 4 m/s | drop ballistic forward 적정 |
| C3 | ‖v_z‖ ≤ 2 m/s | vertical momentum 안정 |
| C4 | tilt ≤ 0.26 rad | 자세 안정 (v8 limit_tilt) |
| C5 | ‖ω‖ ≤ 2 rad/s | toss pitch back 차단 (v9a 의도) |
| C6 | yaw_error ≤ 60° | target 방향 정렬 |
| C7 | ‖v_xy‖ ≥ 0.3 m/s | 안전장치 (full heading reward 가 사실상 강제) |

**Success 정의**: sphere 진입 AND ∏(C1~C7) = 1 (모든 조건 만족, jackpot 발동).

**C 조건 검사 시점**: **sphere 진입 step 에서 한 번만 검사** (매 step 검사 아님).
- 매 step 환경이 `d²(drone → target)` 계산
- 첫 번째 d² ≤ 20.5 를 만족하는 step 발견 시 → 그 step 의 (pos, vel, ang_vel, attitude) 로 C1~C7 검사
- 같은 step 에서 terminal reward 계산 후 ep 종료
- terminate=True, info['phase1_success']=bool(모든 C 만족), info['phase1_C_satisfied']=[C1, ..., C7] (각 bool)

### 2.9 환경 임계값 (Layer 1)

| 항목 | v8 | RAD Phase 1 | 변경 이유 |
|---|---|---|---|
| ground_contact_altitude | 0.5 | **0.5** (그대로) | z=4 강제는 Hann reward 로, crash 는 v8 그대로 |
| min_altitude (점진 페널티) | 3.0 | **0.5** | ground_contact_alt 와 동일값 → 사실상 **단일 crash 임계값** (이전 두 단계 z < 0.5 즉시 crash + z < 3 점진 → RAD: z < 0.5 만 발동, min_altitude path 는 ground_contact 와 동시 발동 → 코드상 어느 path 가 먼저 검사되든 같은 −50 페널티) |
| min_altitude_start_step | 1 | 1 (그대로) | |
| max_distance | 20.0 | **15.0** | v10 좌표 √50 ≈ 7m 에 적정 마진 |
| max_altitude | 50.0 | 50.0 (그대로) | |
| max_steps | 800 | 800 (그대로) | Phase 1 ep 단위 |
| hover_speed_threshold | 1.0 | 1.0 (그대로) | |
| hover_consecutive_threshold | 150 | 150 (그대로) | |
| hover_truncate_enabled | true | true (그대로) | |
| max_consecutive_fast_resets | 100 | **50** | Phase 1 drop 없어 fast reset 빈도 ↑ |

### 2.10 Phase 1 종료 시 페널티 (변경 없음)

| 항목 | 값 |
|---|---|
| penalty_crash (z<0.5) | −50 |
| penalty_overspeed | −30 |
| penalty_out_of_range (d>15m) | −30 |
| penalty_max_altitude | −15 |
| penalty_hover | −30 |
| truncation_penalty | −15 |
| penalty_bad_attitude (tilt>0.26 or inv>1.047) | −30 |
| penalty_target_lost | −10 (vision dead) |
| penalty_instability | 50 |

---

## 3. Phase 2 (Drop) 결정 누적

### 3.1 종료 조건

```
Phase 2 학습 종료:
  step ≥ 150k
  AND 최신 50k window 의 success_rate ≥ 90%
    (success = drop_error ≤ 1.0m, ← d_impact trigger 와 정합)
```

| 항목 | 값 |
|---|---|
| step floor | 150k (Phase 1 의 절반) |
| window | 50k |
| threshold | 90% |
| **success 정의** | **drop_error ≤ 1.0m** (v8 의 2.0 → 1.0, trigger 와 정합) |

### 3.2 시작 state (warm start init + I-2)

| 항목 | 값 |
|---|---|
| **Weights init** | Phase 1 종단 weights (warm start init only). 그 후 두 정책 독립 진화. Phase 1 freeze 별도 보관 |
| **시작 state distribution** | Phase 1 종단 distribution (sphere 안 + 7 조건 모두 만족) |
| **인프라 구현** | **I-2 — Phase 1 정책 rollout 매 ep** (sphere 도달까지 simulate, 그 시점부터 Phase 2 정책 control) |
| Phase 1 rollout max_steps | 300 (헤매도 너무 오래 끌지 않음) |
| **Phase 1 rollout step 은 Phase 2 buffer 에 안 들어감** | Phase 2 학습 신호 오염 방지 |
| **Phase 1 정책의 stochasticity 보존** | spawn yaw 랜덤 + target_entropy 로 다양성 확보 |

### 3.3 Drop trigger

| 항목 | 값 |
|---|---|
| `auto_drop_threshold` | **1.0 m** (v8: 2.0 → 1.0, 정밀도 강화) |
| Manual drop (action[4]) | 비활성 (drop ASAP exploit 차단) |
| `random_drop_prob` | 0.0 |
| `random_drop_start_step` | 600 (dead) |
| `hover_drop_block_threshold` | 0 (비활성) |
| 강제 drop / no-drop safety | **없음** (학습 속도 우선, 사용자 결정) |

→ 정책이 d_impact ≤ 1m 달성할 때까지 비행 학습. drop 안 일어나면 자연 crash 또는 max_steps 종료.

### 3.4 Per-step reward

| 항목 | Phase 1 값 | RAD Phase 2 값 | 변경 |
|---|---|---|---|
| time penalty (hardcoded) | −0.05 | **−0.1** (2× 강화) | 시간 압박 |
| `w_ang_vel` | 0.05 | **0.1** (2× 강화) | 안정성 |
| `w_action_smooth` | 0.05 | **0.1** (2× 강화) | 부드러움 |
| `w_dist` (z 포함) | 1.0 | 1.0 (그대로) | |
| `w_heading` (수평만, speed_gate) | 0.7 | 0.7 (그대로) | |
| `w_z` (Hann) | 0.3 | 0.3 (그대로) | |
| **`w_impact` (CCIP exp decay)** | 0 | **1.0** | drop 정밀도 강조 |
| **`k_impact`** | 0.05 | **0.1** | decay 강화 |
| `w_distance_penalty` | 0 | 0 (dead) | |

### 3.5 Drop event reward (drop 시점 1회)

```python
# 코드 위치: drone_drop_env_rad.py 의 drop event block

# 1. Drop attempt × proximity (약화)
reward  = drop_attempt_bonus × exp(−k_drop_proximity × d_xy)
       = 30 × exp(−0.1 × d_xy)         # RAD: k 0.4 → 0.1

# 2. Base precision (v8 그대로)
reward += w_drop_base × exp(−k2 × d_error)
       = 100 × exp(−0.2 × d_error)

# 3. Jackpot (v8 그대로)
if d_error ≤ 0.3: reward += 50

# 4. Altitude penalty (폐기)
reward −= 0                              # RAD: alt_penalty_max 0 (v10 z=4 강제로 의미 없음)

# 5. Instability (event)
if ω>10 or tilt>0.26: reward −= 50      # v8 그대로

# 6. v9a drop_angaccel (유지)
reward −= 0.5 × max(diff in 5-step window)

# 7. Invalid drop (dead)
reward −= 0                              # invalid_drop_penalty 0
```

| 항목 | v8/v9a | RAD Phase 2 | 변경 |
|---|---|---|---|
| `drop_attempt_bonus` | 30 | 30 (v8) | 유지 |
| `k_drop_proximity` | 0.4 (v8) | **0.1** (약화) | toss 보존 |
| `w_drop_base` | 100 | 100 (v8) | 유지 |
| `k2_precision` | 0.2 | 0.2 (v8) | 유지 |
| `success_threshold` | 2.0 | **1.0** | trigger 와 정합 |
| `jackpot_threshold` | 0.3 | 0.3 (v8) | 유지 |
| `r_success_jackpot` | 50 | 50 (v8) | 유지 |
| `alt_penalty_max/mid/k` | 50/30/0.15 | **0** | 폐기 |
| `drop_angaccel_scale` | 0.5 (v9a) | 0.5 (v9a) | 유지 |
| `drop_angaccel_window_n` | 5 (v9a) | 5 (v9a) | 유지 |
| `invalid_drop_penalty` | 0 | 0 | dead |
| `penalty_instability` | 50 (v8) | 50 (v8) | 유지 |
| `w_prediction` | 0 (v5+) | 0 | dead |

### 3.6 Drop precision 의 reward 분포 (sanity check)

| d_error | base × exp(−0.2×e) | jackpot (≤0.3) | total |
|---|---|---|---|
| 0 m | +100 | +50 | **+150** |
| 0.3 m | +94.2 | +50 | **+144** |
| 0.5 m | +90.5 | 0 | +90.5 |
| **1.0 m (success threshold)** | **+81.9** | 0 | **+81.9** |
| 2.0 m | +67.0 | 0 | +67.0 (success ❌ in RAD, ✅ in v8) |
| 5.0 m | +36.8 | 0 | +36.8 |
| 10 m | +13.5 | 0 | +13.5 |

### 3.7 경계 제약 강화 (action / attitude)

| 항목 | v8 | RAD Phase 2 |
|---|---|---|
| `action_rate_limit` | 0.2 | **0.15** (강화) |
| `action_v*_scale` | 3.0 | 3.0 (그대로 — sphere 안 toss 위해 충분 필요) |
| `limit_tilt` (event crash) | 0.26 | 0.26 (그대로, ang_vel fix 후 안정) |
| `limit_ang_vel` (event) | 10.0 | 10.0 (그대로) |
| `limit_inverted_tilt` | 1.047 | 1.047 (그대로) |

### 3.8 Crash 조건 (H1 + H2)

#### H1 — 기존 v8 crash 그대로

| 항목 | 발동 조건 | reward |
|---|---|---|
| penalty_crash | z < 0.5 | −50 |
| penalty_overspeed | speed > V_MAX_SAFETY | −30 |
| penalty_bad_attitude | tilt > 0.26 or inv > 1.047 | −30 (event, ep 계속) |
| penalty_out_of_range | d > 15m | −30 + ep terminate |
| penalty_max_altitude | z > 50m | −15 + ep terminate |
| penalty_hover | 150 step 정체 | −30 + ep terminate |
| truncation_penalty | phase2_max_steps 도달 | −15 |
| penalty_target_lost | vision dead | −10 (vision off) |
| penalty_instability | drop 시 ω/tilt 초과 | −50 (drop 시점만) |

#### H2.1 — Sphere 벗어남 crash (RAD 신규)

| 항목 | 값 |
|---|---|
| 발동 조건 | d²(drone → target) > 22 |
| **검사 주기** | **매 step 검사** (drop event 만 아님). Phase 2 정책 제어 시작 후 매 step `_check_sphere_escape()` 호출 |
| Sphere margin | radius √22 ≈ 4.690m (sphere 4.527m + 0.163m margin) |
| reward | −30 |
| ep terminate | true |
| 목적 | Phase 1 종단 distribution 보호 + 정책 발산 방지 |

### 3.9 Phase 2 max_steps (신규)

| 항목 | 값 |
|---|---|
| `phase2_max_steps` | **200** (≈ 10 초 @ 20Hz) |
| **카운트 시작 시점** | **Phase 2 정책 control 인수 시점 (= sphere 진입 + Phase 1 rollout 종료) 부터 step 1 카운트** |
| 의미 | Phase 2 의 sphere 진입 후 drop 까지 max step. 초과 시 truncation (−15) |
| 학습 시 ep 구조 | Phase 1 rollout (~25s, RL step 안 카운트, Phase 2 buffer 안 들어감) → Phase 2 정책 control (RL step 1 ~ 200 카운트) → drop 또는 truncation |
| 사용자 원칙 | "Phase 2 짧고 빠른 drop" |

---

## 4. 학습 절차 (Phase 1 → Phase 2 warm start)

### 4.1 Phase 1 학습

```bash
# rad_v1 launch
ros2 launch mission_manager infra_rad.launch.py    # Gazebo + PX4 + bridge + agent
ros2 launch mission_manager episode_rad.launch.py  # mission_manager_rad (cruise 1m/s 가속)
ros2 run rl_navigation train_sac_rad --phase 1     # Phase 1 학습
```

학습 종료 조건 (위 2.1):
- step ≥ 300k AND 50k window success_rate ≥ 90%

종료 시 자동 처리:
- 마지막 checkpoint = `rl_checkpoints_rad/sac_phase1_step{N}.zip`
- Frozen copy = `rl_checkpoints_rad/sac_phase1_final.zip` (deploy 용)
- wandb run 종료

### 4.2 Phase 2 학습

```bash
# 동일 launch (infra/mission_manager 재사용)
ros2 run rl_navigation train_sac_rad --phase 2 \
  --phase1-model rl_checkpoints_rad/sac_phase1_final.zip
```

train_sac_rad 의 Phase 2 logic:
1. Phase 1 모델 load (deploy 용으로 freeze 별도 보관, `predict(deterministic=False)` 호출 — stochastic)
2. Phase 2 정책 = Phase 1 weights 로 init (warm start init only)
3. Phase 2 replay buffer = fresh (empty)
4. 학습 loop:
   - 매 ep 시작 시 Phase 1 정책으로 rollout (`predict(deterministic=False)`, max 300 step)
   - Phase 1 rollout 종료 조건:
     - (a) sphere 진입 (d² ≤ 20.5) → 그 시점 state 로 Phase 2 정책 control 인수
     - (b) max 300 step 도달 → **ep 폐기** (Phase 1 rollout 실패), 다음 ep reset 후 재시도. Phase 2 buffer 에 아무것도 안 들어감. wandb 에 `phase1_rollout_failed_rate` 카운트
     - (c) crash (z<0.5 등) → ep 폐기, 동일 처리
   - sphere 진입 성공 시:
     - Phase 2 정책으로 control 인수 — step 1 부터 카운트 (`phase2_max_steps=200`)
     - 매 step transition 을 **Phase 2 buffer 에만** 저장 (Phase 1 rollout step 은 제외)
     - Phase 2 정책 update (gradient_steps=1)
     - drop trigger / sphere 벗어남 / crash / truncation 으로 종료
5. 종료 조건: step ≥ 150k AND 50k window success_rate ≥ 90% (Phase 2 정책 step 만 카운트)

### 4.3 Eval

```bash
# Phase 1 단독 eval (sphere 진입 + 7 조건 만족률)
dgui sac_phase1_final --phase 1 --episodes 10

# Phase 2 단독 eval (Phase 1 rollout + drop)
dgui sac_phase2_final --phase 2 --phase1 sac_phase1_final --episodes 10

# 통합 eval (Phase 1 → Phase 2)
dgui sac_phase2_final --integrated --episodes 10
```

---

## 5. 인프라 — I-2 (Phase 1 rollout 매 ep)

### 5.1 선택 이유

| 옵션 | 의미 | RAD 선택 |
|---|---|---|
| I-1 (Gazebo state injection) | set_entity_pose + EKF reset | ❌ 6 가지 silent fail risk |
| **I-2 (Phase 1 rollout)** | 매 ep Phase 1 정책으로 sphere 도달까지 simulate | ✅ 인프라 변경 최소 |
| I-3 (Hybrid) | I-2 + state perturb | 추후 검토 |

### 5.2 I-2 의 비용

| 측면 | 값 |
|---|---|
| ep 당 추가 시간 | ~25s (Phase 1 rollout, sphere 도달까지) |
| 학습 시간 증가 (150k step) | +50% (vs I-1) |
| Silent fail risk | ★ (기존 인프라 그대로) |

### 5.3 다양성 보존 메커니즘 + stochasticity 모드

I-2 가 D2-b (state pool sampling) 보다 stochasticity 낮은 문제 보완:
- Phase 1 정책의 stochasticity 가 곧 Phase 2 init distribution 의 다양성
- spawn yaw 랜덤 ±90° + target_entropy = −15.0 로 정책 stochasticity 유지
- 만약 Phase 1 정책이 너무 deterministic 화되면 Phase 2 init 좁아짐 → Phase 1 entropy 강화 필요

**Phase 1 정책의 `predict()` 모드** (시점별):

| 시점 | mode | 이유 |
|---|---|---|
| Phase 1 학습 중 | stochastic (default) | exploration |
| Phase 1 단독 eval (dgui) | configurable (default deterministic for 평가 reproducibility) | 평가 일관성 |
| **Phase 2 학습 중 rollout** | **stochastic (`deterministic=False`)** | **Phase 2 init distribution 다양성 확보** |
| Phase 2 학습 후 통합 eval (dgui) | configurable (default deterministic) | 평가 일관성 |
| Phase 2 단독 eval (dgui) | configurable | 평가 일관성 |

→ **Phase 2 학습 시 Phase 1 정책 rollout = stochastic** (deterministic 아님). 매 ep 마다 다른 trajectory → 다양한 종단 state.

---

## 6. 코드 분리 — 옵션 A (파일 분리, 같은 package)

### 6.1 신규 파일 list

```
ros2_ws/src/
├── rl_navigation/
│   ├── rl_navigation/
│   │   ├── drone_drop_env_rad.py        ← 신규 (Class: DroneDropEnvRAD)
│   │   ├── train_sac_rad.py             ← 신규 (Phase 1/2 분리 logic)
│   │   ├── drone_drop_env.py            ← 그대로 (v8 학습용)
│   │   ├── train_sac.py                 ← 그대로
│   │   └── (기타 그대로)
│   ├── config/
│   │   ├── hyperparams_rad.yaml         ← 신규
│   │   └── hyperparams.yaml             ← 그대로
│   └── setup.py                         ← entry_point + data_files 추가
│
└── mission_manager/
    ├── mission_manager/
    │   ├── mission_manager_rad_node.py  ← 신규 (cruise 1m/s 가속, yaw 랜덤)
    │   └── mission_manager_node.py      ← 그대로
    ├── launch/
    │   ├── infra_rad.launch.py          ← 신규
    │   ├── episode_rad.launch.py        ← 신규
    │   └── (기존 launch 그대로)
    └── setup.py                         ← entry_point 추가
```

### 6.2 hyperparams_rad.yaml 의 top-level 구조 (가이드)

```yaml
training:
  phase: "phase1"                    # "phase1" or "phase2", train_sac_rad 가 분기
  total_timesteps: 600000            # floor; success_rate gate 로 조기 종료 가능
  checkpoint_freq: 5000
  checkpoint_dir: "/workspace/ros2_ws/rl_checkpoints_rad"
  phase1_model_path: ""              # Phase 2 시 필수: "rl_checkpoints_rad/sac_phase1_final.zip"
  phase1_rollout_max_steps: 300      # Phase 2 시 Phase 1 rollout max
  phase1_success_window: 50000       # 50k window
  phase1_success_threshold: 0.9      # 90%
  phase1_step_floor: 300000          # 300k
  phase2_success_window: 50000
  phase2_success_threshold: 0.9
  phase2_step_floor: 150000

sac:
  # 전부 v8 그대로 — 변경 없음
  learning_rate: 1.0e-4
  buffer_size: 500000
  batch_size: 256
  tau: 0.002
  gamma: 0.995
  learning_starts: 1000
  gradient_steps: 1
  net_arch: [256, 256]
  device: "cuda"
  use_per: true
  per_alpha: 0.6
  per_eps: 0.1
  per_priority_max: 30.0
  ent_damping_threshold: 5.0
  ent_coef_hard_cap: 1.0
  target_entropy: -15.0
  target_q_clip: 500.0

environment:
  target_enu_x: 4.0
  target_enu_y: 3.0
  target_enu_z: 0.0                  # 지면 marker
  pos_scale: 10.0                    # v8: 5 → 10
  vel_scale: 15.0
  ang_vel_scale: 3.14159265
  action_vx_scale: 3.0
  action_vy_scale: 3.0
  action_vz_scale: 3.0
  action_yaw_scale: 1.0
  action_rate_limit_phase1: 0.2      # Phase 1
  action_rate_limit_phase2: 0.15     # Phase 2 (강화)
  max_steps: 800                     # Phase 1 의 max RL step
  phase2_max_steps: 200              # Phase 2 의 max RL step (sphere 진입 후)
  min_altitude: 0.5
  ground_contact_altitude: 0.5
  max_distance: 15.0
  max_altitude: 50.0
  obs_wait_timeout: 0.02
  cruise_poll_timeout: 60.0
  use_vision: false
  max_consecutive_fast_resets: 50

cruise:
  spawn_yaw_random_enabled: true
  spawn_yaw_relative_range: [-1.5708, 1.5708]   # -π/2 ~ +π/2
  target_speed: 1.0                  # head 방향 1 m/s 가속

phase1:
  switch_d_sq: 20.5
  terminal_floor: 20.0
  terminal_w_each: 7.14              # = 50/7
  terminal_complete_bonus: 50.0
  C1_z_min: 3.0
  C1_z_max: 5.0
  C2_v_xy_max: 4.0
  C3_v_z_max: 2.0
  C4_tilt_max: 0.26
  C5_omega_max: 2.0
  C6_yaw_err_max: 1.047              # 60°
  C7_v_xy_min: 0.3

phase2:
  sphere_escape_d_sq: 22.0
  sphere_escape_penalty: -30.0

reward:
  # --- 공통 (Phase 1 + Phase 2) ---
  k1_potential: 1.0
  w_dist: 1.0                        # Phase 1 + Phase 2 동일
  w_heading: 0.7
  w_z: 0.3                           # Hann z reward (Phase 1 + Phase 2)
  z_target: 4.0
  z_half_range: 3.5                  # Hann window half-range
  speed_gate_enabled: true

  # --- Phase 별 분기 (w_*_phase1 / w_*_phase2 또는 phase 별 별도 hyperparams) ---
  w_time_phase1: -0.05
  w_time_phase2: -0.1                # 강화
  w_ang_vel_phase1: 0.05
  w_ang_vel_phase2: 0.1
  w_action_smooth_phase1: 0.05
  w_action_smooth_phase2: 0.1
  w_impact_phase1: 0.0               # Phase 1 비활성
  w_impact_phase2: 1.0               # 강화
  k_impact_phase2: 0.1

  # --- Drop trigger (Phase 2 만) ---
  auto_drop_threshold: 1.0           # v8: 2 → 1
  random_drop_prob: 0.0
  random_drop_start_step: 600
  hover_drop_block_threshold: 0.0

  # --- Drop event (Phase 2 만) ---
  drop_attempt_bonus: 30.0
  k_drop_proximity: 0.1              # v8: 0.4 → 0.1
  w_drop_base: 100.0
  k2_precision: 0.2
  success_threshold: 1.0             # v8: 2 → 1
  jackpot_threshold: 0.3
  r_success_jackpot: 50.0
  penalty_instability: 50.0
  alt_penalty_max: 0.0               # 폐기
  alt_penalty_mid: 0.0
  alt_penalty_k: 0.0
  drop_angaccel_penalty_scale: 0.5
  drop_angaccel_window_n: 5
  invalid_drop_threshold: 95.0
  invalid_drop_penalty: 0.0
  drop_wait_timeout: 3.0
  w_prediction: 0.0
  k_prediction: 0.1

  # --- Layer 1 (variable 명 v8 그대로) ---
  limit_ang_vel: 10.0
  limit_tilt: 0.26
  limit_inverted_tilt: 1.047
  penalty_crash: -50.0
  penalty_overspeed: -30.0
  penalty_target_lost: -10.0
  penalty_out_of_range: -30.0
  penalty_max_altitude: -15.0
  penalty_bad_attitude: -30.0
  truncation_penalty: -15.0
  hover_speed_threshold: 1.0
  hover_consecutive_threshold: 150
  hover_truncate_enabled: true
  penalty_hover: -30.0

wandb:
  project: "drone-bombard-sac"
  entity: "nayoonho0922-seoul-national-university"
  run_name: "rad_phase1_v1"          # 또는 "rad_phase2_v1"
  tags: ["rad", "phase1", "v1"]      # phase 별 ["rad", "phase2", "v1"]
  save_model_artifact: true
  log_freq: 1000
```

→ 정확한 yaml 구조는 코드 작성 시 작성 (#135 task). 위는 참고 schema.

### 6.3 setup.py 변경

`rl_navigation/setup.py`:
```python
data_files=[
    ...
    ('share/' + package_name + '/config', [
        'config/hyperparams.yaml',
        'config/hyperparams_rad.yaml',     # 신규
    ]),
],
entry_points={
    'console_scripts': [
        ...
        'train_sac        = rl_navigation.train_sac:main',
        'train_sac_rad    = rl_navigation.train_sac_rad:main',   # 신규
    ],
},
```

`mission_manager/setup.py`:
```python
entry_points={
    'console_scripts': [
        'mission_manager_node     = mission_manager.mission_manager_node:main',
        'mission_manager_rad      = mission_manager.mission_manager_rad_node:main',  # 신규
    ],
},
```

### 6.4 충돌 방지

| 영역 | 처리 |
|---|---|
| ROS2 topic | 시간차 학습이라 충돌 없음 (동시 학습 안 함) |
| Shared file flag | 시간차이므로 OK |
| **Checkpoint 경로** | `rl_checkpoints/` (v8) vs `rl_checkpoints_rad/` (RAD) 분리 |
| **Wandb run_name** | `phase1_redux_v*` (v8) vs `rad_phase1_v1`, `rad_phase2_v1` (RAD) |
| **Wandb tags** | `["rad", "phase1", "v1"]` |
| **Wandb project** | `drone-bombard-sac` 그대로 (별도 project 필요 시 `drone-bombard-rad`) |

### 6.5 학습 전환 workflow (RAD ↔ v8 왔다갔다)

```bash
# RAD 학습 중인 process stop (SIGTERM)
pkill -SIGTERM -f train_sac_rad
# preempt save 완료 대기

# 인프라 정리
dgui_kill

# v8 학습으로 전환 (기존 launch)
ros2 launch mission_manager infra.launch.py &
ros2 launch mission_manager episode.launch.py &
ros2 run rl_navigation train_sac
```

---

## 7. 학습 전 검증 (Reset 잔존 속도 / EKF 누설)

### 7.1 검증 대상

| 항목 | 위험 |
|---|---|
| `resetWorld` 만으로 drone velocity = 0 reset 되는가 | Gazebo bug 사례 — set_link_state 명시 호출 필요할 수도 |
| PX4 EKF 가 resetWorld 직후 새 spawn 위치 잡는가 | EKF 가 이전 velocity fusion 시 drift |
| `consecutive_fast_resets cap` (RAD: 50) 동작 검증 | Phase 1 drop 없음 → fast reset 비율 100% |
| 이전 ep final attitude 가 다음 ep 시작에 누설되는가 | 사용자 우려 직접 대상 |

### 7.2 dgui smoke test 계획

```bash
# 100 ep resetWorld 만 돌리기 (drop 없음, fast path)
dgui sac_phase1_step_dummy --episodes 100 --no-drop --record-init-state

# 결과 분석
python local/tools/init_state_distribution_analysis.py \
  --eval-log eval_logs/smoke_test_*.json \
  --output init_state_distribution.png
```

확인 항목:
- ep0 ~ ep100 의 초기 z 분포 → 5.0 근방 모이는지 (cruise 종단)
- 초기 vx, vy 분포 → 1.0 m/s 근방 (head 방향, yaw 따라 회전)
- 초기 ang_vel 분포 → 0 근방 모이는지

→ 분포가 좁으면 OK. 만약 ep 마다 drift 누적되면 jekyun_v2 patch 추가 검증 필요.

---

## 8. 폐기 / 유지 / 변경 항목 (v8/v9a 대비)

### 8.1 폐기 항목 (RAD 에서 사용 안 함)

| 항목 | 폐기 이유 |
|---|---|
| `cruise_speed_x` (1.0), `cruise_speed_y` (−1.0) | 신규 cruise 메커니즘 (head 방향 1m/s 가속) 으로 대체 |
| `stage1_R` (2.0, v10a 임시) | 신규 `switch_d_sq = 20.5` |
| `stage1_only` (false, v10a 임시) | RAD 는 2 정책 구조 |
| `stage1_reach_bonus` (100) | `phase1_terminal` 의 floor + Σ + jackpot 통합 |
| `target_enu_z` (5.0, v10a 임시) | target = (4, 3, 0) 명시 |
| `alt_penalty_max/mid/k` (50/30/0.15) | z=4 강제 정책으로 dead → 0 |
| `random_drop_prob` (0.005 → 0 in v8) | 그대로 0 |
| `hover_drop_block_threshold` (0) | 그대로 0 |
| v9a 의 w_dist 1.5 | RAD 는 1.0 (v8 그대로) |

### 8.2 유지 항목 (v8 그대로)

| 항목 | 값 |
|---|---|
| 대부분의 Layer 1 페널티 (crash, overspeed, attitude, max_altitude, hover, truncation) | v8 그대로 |
| 대부분의 drop event reward (drop_attempt_bonus, w_drop_base, k2, jackpot, instability) | v8 그대로 |
| limit_tilt, limit_inverted_tilt, limit_ang_vel | v8 그대로 (ang_vel fix 후 안정) |
| SAC hyperparams (lr=1e-4, buffer=500k, batch=256, tau=0.002, gamma=0.995) | v8 그대로 |
| PER 설정 (alpha=0.6, eps=0.1, priority_max=30) | v8 그대로 |
| DampedEntropySAC (damping_threshold=5.0, hard_cap=1.0, target_q_clip=500) | v8 그대로 |
| target_entropy=-15.0 | v8 그대로 (Phase 1 의 다양성 위해 추후 조정 가능) |
| max_consecutive_fast_resets 100 (Round 7 처방) | RAD 는 50 (drop 없는 Phase 1 고려) |

### 8.3 변경 항목 (v8 → RAD)

| 항목 | v8 | RAD |
|---|---|---|
| obs dim | 17 | 14 (yaw-only body frame 상대좌표) |
| POS_SCALE | 5 | 10 |
| auto_drop_threshold | 2.0 | 1.0 |
| w_impact | 0.4 | 1.0 |
| k_impact | 0.05 | 0.1 |
| w_dist 거리 계산 | 2D (xy) | 3D ((x²+y²+(z−4)²)^0.5) |
| k_drop_proximity | 0.4 | 0.1 (Phase 2) |
| time penalty | −0.05 | −0.05 (Phase 1) / **−0.1 (Phase 2)** |
| w_ang_vel | 0.05 | 0.05 (Phase 1) / **0.1 (Phase 2)** |
| w_action_smooth | 0.05 | 0.05 (Phase 1) / **0.1 (Phase 2)** |
| action_rate_limit | 0.2 | 0.2 (Phase 1) / **0.15 (Phase 2)** |
| success_threshold | 2.0 | **1.0** (Phase 2, trigger 정합) |
| max_distance | 20 | **15** |
| max_consecutive_fast_resets | 100 | **50** (Phase 1) |

### 8.4 신규 항목 (RAD only)

| 항목 | 값 |
|---|---|
| `spawn_yaw_random_enabled` | true |
| `spawn_yaw_relative_range` | [−π/2, +π/2] relative to drone→target |
| `cruise_target_speed` | 1.0 m/s (head 방향) |
| `switch_d_sq` (Phase 1 → Phase 2) | 20.5 |
| `phase1_terminal_floor` | 20 |
| `phase1_terminal_w_each` | 50/7 ≈ 7.14 |
| `phase1_terminal_complete_bonus` | 50 |
| `phase1_final_state_C1_z_min` | 3 |
| `phase1_final_state_C1_z_max` | 5 |
| `phase1_final_state_C2_v_xy_max` | 4 |
| `phase1_final_state_C3_v_z_max` | 2 |
| `phase1_final_state_C4_tilt_max` | 0.26 |
| `phase1_final_state_C5_omega_max` | 2 |
| `phase1_final_state_C6_yaw_err_max` | 1.047 (60°) |
| `phase1_final_state_C7_v_xy_min` | 0.3 |
| `w_z` (Hann reward 가중치) | 0.3 |
| `z_target` (Hann mu) | 4.0 |
| `z_half_range` (Hann 폭, 가장자리까지) | 3.5 (즉 reward range [0.5, 7.5]). ⚠️ gaussian σ 아님 — Hann window 의 half-range. |
| `phase1_rollout_max_steps` | 300 (Phase 2 학습 시) |
| `phase2_max_steps` | 200 |
| `phase2_sphere_escape_d_sq` | 22 |
| `phase2_sphere_escape_penalty` | −30 |

---

## 9. 모니터링 (wandb)

### 9.1 Phase 1 wandb metric (주요)

```
phase1/success_rate              ← 7 조건 모두 만족 비율 (Phase 1 종료 조건 metric)
phase1/sphere_entry_rate          ← sphere 진입만 한 비율 (success rate 보다 ≥)
phase1/C1_z_satisfied_rate ~ C7   ← 조건별 만족률 (어느 조건이 어려운지)
phase1/ep_length_p50              ← median ep 길이
phase1/mean_terminal_reward       ← phase1 terminal 평균 reward (20~120)
phase1/jackpot_rate               ← 7/7 jackpot 발동 비율
phase1/init_yaw_distribution      ← spawn yaw 분포 (uniform 확인)
```

### 9.2 Phase 2 wandb metric

```
phase2/success_rate               ← drop_error ≤ 1.0m 비율 (Phase 2 종료 조건 metric)
phase2/mean_drop_error            ← 평균 drop_error
phase2/jackpot_rate               ← drop_error ≤ 0.3m 비율
phase2/ep_length_p50              ← Phase 2 ep 길이 (≤ 100 목표)
phase2/sphere_escape_rate         ← sphere 벗어남 crash 비율 (낮아야 함)
phase2/auto_drop_rate             ← d_impact ≤ 1m trigger 비율
phase2/mean_d_impact_at_drop      ← drop 시점 d_impact
phase2/mean_ang_vel_at_drop       ← drop 시점 ang_vel (v9a 처방 효과 확인)
```

---

## 10. 미해결 / 추후 검토

| 항목 | 비고 |
|---|---|
| target_entropy −15.0 가 Phase 1 의 다양성 보존에 충분한가? | spawn yaw 랜덤이 주된 다양성 source. 학습 후 종단 distribution 분포 분석으로 검증 |
| Phase 1 rollout 의 학습 시간 부담 | I-2 가 +50% 시간 → 150k step Phase 2 학습이 v8 의 22h × 1.5 ≈ 33h. 견딜 수준 |
| Phase 2 의 catastrophic forgetting 위험 | warm start init only + Phase 1 reward 와 다른 reward 사용 → forgetting 가능. 모니터링 필요 |
| Phase 2 의 sphere 벗어남 빈도 | margin 0.16m 적정한지 학습 중 모니터링 |
| 9.x metric 의 일부가 코드 미구현 | env 코드에서 명시적 logging 추가 필요 |

---

## 11. RAD v1 학습 전 체크리스트

- [ ] 코드 신규 파일 작성 (drone_drop_env_rad.py 등 6 개)
- [ ] hyperparams_rad.yaml 작성
- [ ] launch 파일 작성 (infra_rad / episode_rad)
- [ ] setup.py 업데이트 (entry point + data_files)
- [ ] colcon build --packages-select rl_navigation mission_manager
- [ ] Reset 잔존 속도 dgui smoke test (위 7.2)
- [ ] Phase 1 5k dry-run (구현 검증)
- [ ] Phase 1 학습 시작 (300k 목표)
- [ ] Phase 1 종료 후 final model 백업 (`rl_checkpoints_rad/sac_phase1_final.zip`)
- [ ] Phase 2 학습 시작 (150k 목표)
- [ ] Phase 2 종료 후 dgui 통합 eval (10 ep)

---

> **이 문서는 RAD v1 의 모든 design 결정의 single source of truth**.
> 코드 작성 / hyperparams 작성 / 학습 진행 시 이 문서 참조.
> 결정 변경 시 이 문서 update + parameter_log + model_history 동기화.

---

## 12. 학습 진행 이력 (2026-07-05 갱신)

### 12-1. v1 ~ v6 시행 요약

| Version | 시기 | 결과 | 주요 변경 |
|---|---|---|---|
| v1 (원본) | 2026-06-30~07-02 | NaN abort at 62k | Original RAD v1 design 그대로 (run g8mvzniw) |
| v2 (self-healing) | 2026-07-02 | NaN abort at 171k | + A1 gradient clip + B3 weight NaN rollback + 상세 metric |
| v3 (curriculum) | 2026-07-03 | Stage 1 정체 (75%) | + 5-stage curriculum + regression + advance 강화 |
| v4 (reward 축소) | 2026-07-03 | Stage 1↔2 cycle 반복 | + crash -20 / hover -10 / trunc -5 / q_clip 200 / fast_reset 500 |
| v5 (stage 완화) | 2026-07-04 | Stage 2 hover 65% 실패 | + stage2_close 조건 완화 (radius 5→5.5m, z_min 0.15→0.08 등) |
| **v6 (근본 접근)** | 2026-07-04~05 | Stage 3 진입 후 실패 (cycle 반복) | + spawn_yaw ±90°→±45°, hover -10→-30, **initial pos log 신규** |

**v6 진행 상세** (2026-07-05 시점, 62k+ step):
| Step | 이벤트 |
|---|---|
| 14,011 | Advance 1: stage1_intro → stage2_close |
| 27,813 | Advance 2: stage2_close → stage3_target |
| 37,828 | Regress: stage3_target 실패 (0%) → stage2_close |
| 52,138 | Advance 3: stage2_close → stage3_target |
| 62,214 | Regress: stage3_target 재실패 (0%) → stage2_close |

### 12-2. 이번 주 도입된 인프라 (재사용 가능)

#### A. Self-healing SAC (v2 도입)

`DampedEntropySAC` 확장 (train_sac.py 안 `class DampedEntropySAC(SAC)`).

**A1 — Gradient clipping**:
- Actor: `torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=20.0)`
- Critic: `torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=200.0)`
- Optimizer.step() 직전 발동. Overflow 방지.

**B3 — Weight NaN rollback**:
- 매 gradient step 시작 시 `actor.state_dict()` + `critic.state_dict()` clone
- Optimizer.step() 후 weight NaN/Inf 검사
- NaN 감지 시 snapshot 복원 + `optimizer.zero_grad()`
- Metric: `train/nan_rollback_count`

**실전 관찰**: v2~v6 학습에서 nan_rollback_count = 0 (rollback 발동 없음). A1 이 예방 지배적.

#### B. Curriculum + Regression 시스템 (v3 도입)

**Env config (hyperparams_rad.yaml phase1 안)**:
```yaml
curriculum_enabled: true
curriculum_window_size: 10000          # window step 수
curriculum_success_threshold: 0.95     # advance 조건
curriculum_min_stage_steps: 10000      # 최소 stage 유지 step

# Regression (v3 신규)
curriculum_regression_enabled: true
curriculum_regression_threshold: 0.3   # window success < 30% 시 regression
curriculum_regression_min_steps: 5000  # 진입 후 최소 시간
curriculum_regression_cooldown: 20000  # regression 반복 방지
```

**Env 코드 (drone_drop_env_rad.py)**:
- `_curriculum_check_progress()` — 매 ep 종료 시 호출
- Advance / Regress 판정 로직 (config 값 기반)
- Deque `_curriculum_ep_history` (maxlen=200) 로 ep history 관리
- Log: `[CURRICULUM_ADVANCE]` / `[CURRICULUM_REGRESS]`

**Callback**: `SuccessRateGateCallback` (train_sac_rad.py) — Phase 완료 조건 별도 관리.

#### C. Initial position 실측 도구 (v6 도입) ★

**Env 코드**:
```python
# drone_drop_env_rad.py step() 안
if self._step_count == 1:
    self._initial_target_dist_3d = ...
    self._initial_target_dist_xy = ...
    self._initial_pos_x/y/z = ...
    self._initial_speed_xy = ...
```

**Ep 종료 시 info 에 write**:
```python
info['initial_target_dist_3d'] = ...
info['initial_target_dist_xy'] = ...
info['initial_pos_x/y/z'] = ...
info['initial_speed_xy'] = ...
```

**Callback**: `TerminalTypeMonitorCallback` (train_sac_rad.py) — rolling window (100 ep) stats → WandB.

**Metric**:
- `env/initial_target_dist_3d_{mean,min,max}`
- `env/initial_target_dist_xy_{mean,min,max}`
- `env/initial_pos_{x,y,z}_{mean,min,max}`
- `env/initial_speed_xy_{mean,min,max}`

**★ 근본 원인 파악의 결정적 도구**. v1~v5 는 이론 계산만으로 진단 → 여러 번 오추정. v6 도입 후 첫 dump 에서 target 거리 5.10m 확인 → 진짜 원인 즉시 확정.

#### D. Terminal Type Monitor (v2 도입)

**Env 코드**: Ep 종료 시 `info['terminal_type']` 분류
- `entry_success` — sphere 진입 + active_conditions 만족
- `entry_partial_fail` — sphere 진입 but conditions 실패
- `crash` — z<z_min 등 crash 판정
- `bad_att_{tilt,inverted,ang_vel}` — 자세 이상
- `trunc_{timeout,out_of_range,overspeed,max_altitude,hover_timeout,sphere_escape}` — 다양한 truncation

**Callback**: `TerminalTypeMonitorCallback` — Rolling window rate → WandB
**Metric**: `env/{terminal_type}_rate_100`

### 12-3. Reward magnitude 정책 (v4 도입)

| Parameter | v1~v3 | v4 이후 | v6 이후 | 이유 |
|---|---|---|---|---|
| penalty_crash | -50 | **-20** | -20 | Q negative 축적 완화 |
| penalty_hover | -30 | -10 | **-30** | v4 축소 → v6 재강화 (crash 대비 강 penalty 로 local optimum 회피) |
| truncation_penalty | -15 | **-5** | -5 | Truncation 페널티 축소 |
| target_q_clip | 500 | **200** | 200 | Q bootstrap 발산 압력 축소 |
| max_consecutive_fast_resets | 50 | **500** | 500 | Phase 1 은 drop 없음 → fast reset 정상 |

**hover_penalty 재조정 원칙 (v6 확립)**:
- Hover penalty ≥ crash penalty (v6: -30 vs -20)
- 이유: crash > hover 이면 정책이 hover 를 "안전 실패" 로 선택 → local optimum
- 검증: v5 hover 65% → v6 hover 1% (65배 감소)

### 12-4. Spawn yaw random 범위 (v6 조정)

- Original: `spawn_yaw_relative_range: [-π/2, +π/2]` (±90°)
- v6: `[-π/4, +π/4]` (±45°)
- 이유: 정책 학습 어려움 완화. Random 범위 축소로 방향 전환 학습 부담 감소.

### 12-5. 진짜 원인 확정 — Curriculum stage 설계 결함 (Issue #029)

**v6 initial pos log 실측**:
- 정책 시작 시점 target xy 거리 = **5.10m** (min 5.00 ~ max 5.23)
- Spawn (0,0,0) → Target (4,3,0) 직선 거리 = 5.0m
- Cruise 로 이동 미미 (0.15m)

**Stage 조건 vs 시작 거리**:
| Stage | Radius | switch_d² | Spawn d²≈25 | 학습 유도? |
|---|---|---|---|---|
| stage1_intro (v6) | 6.0m | 36.0 | 안 (여유 11) | ✗ trivial |
| stage2_close (v6) | 5.5m | 30.25 | 안 (여유 5) | ✗ trivial |
| **stage3_target** | **4.53m** | **20.5** | **밖 (부족 4.5)** | **✓ 유일 학습 stage** |

**결론**:
- Stage1/2 는 정책이 학습 없이 통과
- Stage3 에서 처음 이동 학습 필요 → 능력 부재 → 완전 실패
- 실제 학습해야 할 이동거리 = 0.47m
- Curriculum stage 조건이 spawn 위치 반영 안 함 = 근본 결함

### 12-6. v7 계획 — Curriculum stage 재설계

| Stage | switch_d² | Radius | active_conditions | z_min | max_dist | limit_tilt |
|---|---|---|---|---|---|---|
| stage1_intro | 23.04 | 4.80m | [] | 0.05 | 40 | 1.2 |
| stage2_close | 21.62 | 4.65m | [] | 0.15 | 30 | 0.9 |
| stage3_target | 20.5 | 4.53m | [] | 0.25 | 25 | 0.5 |
| stage4_partial | 20.5 | 4.53m | [C1, C4, C5] | 0.35 | 20 | 0.30 |
| stage5_full | 20.5 | 4.53m | [C1~C7] | 0.5 | 15 | 0.26 |

**원리**:
- Stage1 부터 spawn 밖 (d²=23 < spawn d²=25) → 정책이 이동 학습 강제
- 매 stage radius 갭 0.15m (완만)
- Stage3 까지는 sphere entry 만 (조건 없음)
- Stage4/5 에서 조건 추가

### 12-7. 관련 문서

- [design_review_2026-07-05.md](design_review_2026-07-05.md) — 이번 주 종합 + 재설계 안
- [../issues/issue_028_rad_ekf_stale_after_reset.md](../issues/issue_028_rad_ekf_stale_after_reset.md) — 진단 방향 정정 (cruise 정상 확인)
- [../issues/issue_029_curriculum_stage_spawn_ignorance.md](../issues/issue_029_curriculum_stage_spawn_ignorance.md) — 진짜 원인
- [../meeting_notes/meeting_notes_2026-07-05.txt](../meeting_notes/meeting_notes_2026-07-05.txt)
- [../parameter_log.md](../parameter_log.md) — entry #43~48 (v1~v6 각 변경)

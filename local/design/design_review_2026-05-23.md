# Design Review — Drone-Bombard-Simulation (2026-05-23)

> **배경**: 사용자가 "근본부터 다시 설계 + 점검" 요청. 두 번의 200k 학습 실패 (junsang_v2: critic 폭주 / junsang_v3: 정책 학습 안 됨) + GUI 관찰 (드론이 튀어나가거나 땅에 부딪힘) 종합.
>
> **목적**: parameter tweak 가 아니라 **시스템 흐름 전체 점검** + 사용자 관찰을 진단/처방으로 매핑 + 처방 우선순위 매트릭스 결정.

작성: 2026-05-23
관련:
- [parameter_log.md](parameter_log.md)
- [meeting_notes/meeting_notes_2026-05-22.txt](meeting_notes/meeting_notes_2026-05-22.txt)
- [meeting_notes/meeting_notes_2026-05-23.txt](meeting_notes/meeting_notes_2026-05-23.txt)
- [../Drone-Bombard-Simulation/RL_analysis.md](../Drone-Bombard-Simulation/RL_analysis.md)

주요 코드 경로: `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`, `ros2_ws/src/rl_navigation/config/hyperparams.yaml`
(host 의 git repo 기준 — 절대경로는 `/home/juns/Drone-Bombard-Simulation/...`)

---

## 0. 목차

- § 1. 현재 시스템 흐름 (env.step)
- § 2. 발견된 문제점 → 고찰 → 해결 (Executive Summary) — **P1~P11**
- § 2A. RL 설계 원칙 — 큰 페널티 / Early termination 의 역효과 검토
- § 2B. 학습 진행 자동 판단 기준
  - 2B-1. "절대 안 좋은 행동" 정의 (사용자 명시)
  - 2B-2. 학습 추세 모니터링 (Run 단위)
  - 2B-3. 자동 모니터링 구현 옵션
- § 3. 컴포넌트별 진단 (상세) — Action / Obs / Reward / Episode / CCIP / SAC
- § 4. 사용자 관찰 → 진단 매핑 (테이블)
- § 5. 처방 매트릭스 — Tier 1 / Tier 2 / Tier 3
- § 6. 새 학습 사이클 plan — Run #12 (junsang_v4)
  - 6-1 사전 정리 / 6-2 yaml / 6-3 code / 6-4 미러 / 6-5 run_name+log / 6-6 검증
- § 7. 폐기/유지 결정
- § 8. 폐기된 시도들 (참고)
- § 9. 다음 행동

---

## 1. 현재 시스템 흐름 (env.step) — **Tier 1 적용 전 (현재)**

```
action [-1,1]^4
   ↓ × action_*_scale (vx 15, vy 5, vz 3, yaw 1)
velocity setpoint (m/s)
   ↓ PX4 OFFBOARD
drone 비행 (Gazebo physics)
   ↓
observation 17-D (pos/50, vel/15, ang_vel/π, vision, attached, rel_target/50, d_impact/50, t_f/10)
   ↓
4-Layer reward 계산
   ↓
terminate (drop) / truncate (max_steps=500) / 계속
```

**Tier 1 적용 후의 새 흐름** (§ 6 plan 의 구현 결과):

```
action [-1,1]^4
   ↓ NEW: rate-limit hard clip (|Δa| ≤ 0.2 per step)
   ↓ × action_*_scale (vx 8, vy 5, vz 3, yaw 1)  ← vx 축소
velocity setpoint → PX4 OFFBOARD → drone 비행
   ↓
observation 17-D (동일)
   ↓
NEW: 매 step safety 검사 (crash / overspeed / ang_vel / inverted)
   ↓ 위반 시 즉시 truncate + penalty (-50~-100)
4-Layer reward 계산
   ↓
terminate (drop @ threshold=10m) / truncate (max_steps=500 또는 safety 위반) / 계속
```

---

## 2. 발견된 문제점 → 고찰 → 해결 (Executive Summary)

> 이 섹션은 시스템 전체의 모든 문제점을 한 곳에서 본다. 각 문제마다 **고찰** (왜 문제, 데이터/관찰 근거) + **해결** (변경 + Tier) 흐름. 상세는 § 3 (컴포넌트별 진단) 이하.
>
> 또한 § 2A 에서 큰 페널티 / early termination 의 역효과 검토 (사용자 우려), § 2B 에서 학습 진행 자동 판단 기준 (사용자 명시 — "안 좋은 학습 판단 근거").

### P1. Action 명령 scale 큼

**고찰**:
- 데이터: `action_vx_scale = 15 m/s` → action[0]=1 시 즉시 15 m/s 명령
- 사용자 GUI 관찰: "시작하자마자 막 튀어나가는" 행동의 직접 원인
- 학습 초기 random action 의 vx=±15 + vz=-3 조합이 자주 나옴 → 즉시 비정상 비행 → buffer 오염

**해결**: `action_vx_scale: 15 → 8` (Tier 1)

### P2. 가속도 제한 없음

**고찰**:
- 코드: action 변화량 (Δa) 무제한. `w_action_smooth=0.05` (soft penalty) 만
- SAC actor 의 step 별 출력이 부드럽지 못함 (특히 학습 초기 ent_coef 1.0 random 시점)
- 가속도 폭증 → 드론 dynamics 한계 초과 → 비정상 비행

**해결**: `action_rate_limit: 0.2` (hard clip in step()) — step 당 |Δa| ≤ 0.2 (Tier 1)
- **효과 추정**: vx 측 max 변화 = 0.2 × `action_vx_scale` (8) = 1.6 m/s / step
- env step dt 추정 0.1~0.2s (PX4 OFFBOARD 명령 rate) → effective max acceleration ≈ 8~16 m/s²
- PX4 default `MPC_ACC_HOR_MAX` ≈ 5 m/s² — 즉 우리 rate_limit 0.2 는 PX4 자체 한계와 비슷한 수준. 더 부드러운 가속 원하면 0.1 로 (max accel ~4~8 m/s²)

### P3. 최대 속도 측정 안전망 약함

**고찰**:
- 데이터: `penalty_overspeed = -8` (speed > 20 m/s), terminate 안 함
- P1 으로 명령 scale 줄여도 PX4 측 누적 가속 또는 외란으로 overspeed 발생 가능
- 페널티 -8 은 정상 step reward (~0.5) 의 16배 — 약한 차단

**해결**: `penalty_overspeed: -8 → -50` + `truncated=True` (Tier 1)

### P4. 고도 최저선 약함 + 검사 늦음 (사용자 명시)

**고찰**:
- 데이터: `min_altitude = 2.0m`, `min_altitude_start_step = 20` (~1초 grace)
- 학습 초기 takeoff 실패 → grace 동안 자유낙하 → 바닥 충돌 → episode 계속
- 사용자 GUI: "땅에 부딪혀 도는" 행동의 일부 원인

**해결**: `min_altitude: 2 → 3m`, `min_altitude_start_step: 20 → 10` (Tier 1)

### P5. Crash penalty 약함 + episode terminate 안 함 (사용자 명시)

**고찰**:
- 데이터: `penalty_crash = -10`, terminate 안 함 (drone_drop_env.py L805-807)
- 사용자 GUI: 드론이 바닥에 부딪힌 채로 episode 계속 진행 → 망가진 dynamics 의 transition 이 buffer 50% 차지
- 후속 gradient update 가 "땅에서 기는" 행동도 학습 가능

**해결**: `penalty_crash: -10 → -100` + crash 시 `truncated=True` (drone_drop_env.py) (Tier 1)

### P6. CCIP threshold 너무 빡셈 (사용자 명시 — "10m 안으로 일단")

**고찰**:
- 데이터: `auto_drop_threshold = 4m`
- 학습 초기 random policy 가 4m 안 도달 거의 불가능 → drop_attempt_bonus +150 못 받음
- 핵심 학습 동기 (drop = 큰 보상) 부재 → 정책이 drop 학습 못함

**해결**: `auto_drop_threshold: 4 → 10m` (curriculum phase 1, 사용자 명시) (Tier 1)

### P7. Per-step reward 약함

**고찰**:
- 데이터: `w_dist=0.5, w_heading=0.7, w_impact=0.4` (jekyun v3)
- junsang_v3 timeline: `mean_rew_dist ≈ -0.002` (사실상 0), `mean_rew_orient` 양수/음수 진동
- d_xy 19m 부근에서 reward gradient 너무 작음 → 정책이 target 방향으로 가는 학습 동기 부족

**해결**: P5 + P6 + P9 적용 후 결과 보고 결정 (Tier 2). 우선 너무 키우면 orbit-milking 위험.

### P8. Reward scale 격차 + truncation 압도

**고찰**:
- 데이터: per-step (~0.5) vs terminal drop (+250~350) vs truncation (-80)
- 학습 초기 모든 episode 가 truncation → -80 압도 → 정책이 "빨리 끝내기" 선호 가능
- Reward 분포 dynamic range 가 critic variance ↑ (junsang_v2 의 17K 발산 가설 중 하나)

**해결**: `truncation_penalty: -80 → -30` (사용자 명시) (Tier 1)

### P9. Crash/overspeed 시 episode 안 끝남 — 안 좋은 학습 못 막음 (사용자 명시 — "빨리 쳐내야")

**고찰**:
- 현재: crash/overspeed/tilt 위반 시 reward 페널티만, episode 계속 진행
- 망가진 dynamics 의 transition 누적 → 후속 학습 신호 오염 (사용자 통찰: "이후 긴 학습에 악영향")
- 사용자 우려: 망가진 sample 이 buffer 에서 random sample 될 때마다 gradient update 가 잘못된 방향

**해결** (Tier 1):
- 별도 변경 불요 — **P3/P5/P11 의 `truncated=True` 처방이 P9 도 통합 해결**
- (Tier 2 검토) drop 시점이 아닌 step 도중에도 `tilt > limit_tilt` 시 terminate

### P10. 학습 진행 객관 판단 기준 부재 (사용자 명시)

**고찰**:
- 현재: 학습 끝나야 평가 (200k 완주 후 evaluate)
- junsang_v2 의 critic 발산 (63k 시점) 을 200k 완주 후에야 발견 — 자원 낭비 ~7시간
- 사용자 통찰: "안 좋은 학습은 시작부터 과감히 쳐내야"

**해결**: § 2B 의 자동 모니터링 기준 도입 — § 2B-3 옵션 A (`wandb_watch.py` 외부 스크립트) (Tier 2)

### P11. "절대 안 좋은 행동" 정의 + 검사 부재 (사용자 명시)

**고찰**:
- 사용자 명시: 급격한 가속, 매우 큰 흔들림 같은 행동은 **모든 학습 단계에서 universal bad** — 학습 신호로 의미 없음
- 데이터: 현재 `limit_ang_vel=2.0`, `limit_tilt=0.26` 은 **drop 시점에만** instability 검사 (drop 안 일어나면 검사 안 됨)
- 코드 (drone_drop_env.py): step 도중 각속도/tilt 폭증 시 어떤 처리도 없음
- 영향: 정책이 비정상 자세 (예: 90° 기울어진 채) 로 비행하며 학습 → buffer 의 transition 이 비현실적 dynamics

**해결** (Tier 1):
- 매 step 에서 다음 행동 발생 시 즉시 `truncated=True` + penalty:
  - **각속도 폭증**: `|ω| > limit_ang_vel` (=2.0 rad/s)
  - **Inverted 자세**: `|roll| 또는 |pitch| > limit_inverted_tilt` (=π/3 ≈ 60°)
  - **(이미) 바닥 충돌**: altitude < min_altitude (P5)
  - **(이미) 과속**: speed > 20 m/s (P3)
- **급격한 가속** 은 P2 의 `action_rate_limit` hard clip 으로 사전 차단 (별도 검사 불요)
- penalty: -50 (crash 보다 약하게 — 회복 가능 영역, cliff effect 회피)
- yaml: `limit_inverted_tilt: 1.047` 추가 (60°)
- drone_drop_env.py: step 도중 검사 코드 추가 (drop 시점 instability 와 별도)

---

## 2A. RL 설계 원칙 — 큰 페널티 / Early termination 의 역효과 검토 (사용자 우려)

사용자 우려: **"큰 - 를 어떻게 줄건지? 역효과 내진 않을까? 어떻게 설계?"** — 매우 중요한 메타 질문.

### 역효과 1: Cliff effect

- 큰 페널티 (-1000) + terminate → policy gradient 가 "그 상태 회피" 만 강하게 학습
- 정상적인 행동도 그 상태 근처면 회피 → 학습 영역 축소
- **극단 예**: penalty_crash=-1000 + terminate → 학습 끝나면 드론이 takeoff 시점에 매우 보수적 (예: 천천히 호버만, target 으로 안 감) — 즉 cliff 영역 주변을 회피하는 차선책 으로 수렴

### 역효과 2: Reward variance 폭증 → critic 발산

- per-step ±0.5 + crash -1000 + drop +250 = range 1250
- critic 이 fit 해야 할 Q 분산 폭증 → SAC variance ↑
- **참고 사례**: junsang_v2 의 critic 17K 발산. 단순 range 만의 영향은 아니고 **M2 (gradient_steps=4) 가 큰 terminal spike 를 4배 빠르게 fit 시도 + gamma=0.995 의 future 누적**이 더 핵심. 다만 reward dynamic range (per-step ±0.5 vs terminal +350) 가 그 fit 부담을 키운 요인 중 하나.

### 역효과 3: Death-seeking policy

- truncation 페널티가 너무 크면 → 정책이 "차라리 빨리 죽기" 학습 가능
- 또는 crash penalty < truncation penalty 이면 (-50 vs -80) 의도적 crash 선호 가능

### 권장 설계 원칙

| 행동 | reward | termination | 이유 |
|---|---|---|---|
| **정상 step** | -0.05 ~ +0.5 | no | per-step 학습 shaping |
| **나쁜 행동 (crash, overspeed)** | -50 ~ -100 | **yes** | 학습 데이터 보호 + 차단. 페널티는 적당 |
| **"절대 안 좋은 행동" (각속도 폭증, inverted)** | -50 | **yes** | universal bad. 학습 모든 단계에서 차단. cliff 회피 위해 페널티 적당 |
| **좋은 terminal (drop)** | +200 ~ +350 | yes | jackpot, 학습 동기 |
| **Truncation (timeout)** | -10 ~ -30 | yes | 시간 압박 약하게 — "안 좋음" 일 뿐 "재앙" 아님 |

### 핵심 원칙

1. **나쁜 행동의 페널티 ≤ 좋은 행동의 보상** (예: crash -100 vs drop +250) — death-seeking 방지
2. **Terminate 가 페널티의 역할** — 큰 -negative 는 위험. terminate 자체가 강한 차단 신호
3. **Truncation 페널티 작게** — "안 좋음" 일 뿐 "재앙" 아님
4. **점진적 강화** — 처음엔 약한 페널티 + terminate, 학습 안정화 후 페널티 강화

### Tier 1 처방 검증

- `penalty_crash = -100` + terminate: ✓ 적정 (drop jackpot +250 보다 작음, cliff 위험 낮음)
- `penalty_overspeed = -50` + terminate: ✓ 적정 (crash 보다 약하게 — overspeed 가 덜 치명적)
- `penalty_bad_attitude = -50` + terminate (P11 의 각속도/inverted): ✓ 적정 (crash 보다 약하게 — 회복 가능 영역)
- `truncation_penalty = -30`: ✓ 안전 (-80 보다 보수적)
- 모두 dynamic range 안에서 균형 잡힘. terminate 가 강한 차단 신호 역할 — 페널티는 학습 가능 영역 안에 유지

---

## 2B. 학습 진행 자동 판단 기준 (사용자 명시 — "판단 근거")

### 2B-1. "절대 안 좋은 행동" 정의 (사용자 명시)

매 step 에 검사 → 발견 시 episode 즉시 truncate + penalty:

| 행동 | 판정 임계 | 처리 |
|---|---|---|
| 바닥 충돌 (P5) | `altitude < min_altitude` (3.0m) after start_step | -100 + truncate |
| 과속 (P3) | `speed > 20 m/s` | -50 + truncate |
| 각속도 폭증 (P11) | `\|ω_xyz\| > limit_ang_vel` (2.0 rad/s) | -50 + truncate |
| Inverted 자세 (P11) | `\|roll\| 또는 \|pitch\| > limit_inverted_tilt` (π/3 ≈ 60°) | -50 + truncate |
| 급격한 가속 (P2) | action_rate_limit 으로 hard clip — 사전 차단 | (terminate 불요) |

"절대 안 좋은 행동" 의 universal 기준 — 학습 단계와 무관하게 적용. 사용자 명시 "안 좋은 학습은 시작부터 쳐내야".

### 2B-2. 학습 추세 모니터링 (Run 단위)

**정상 학습 신호** (이 중 다수 만족) — **[2026-05-23 갱신: rate → 누적 count, entry #13]**

| metric | 임계 | 의미 |
|---|---|---|
| `train/critic_loss` | < 500 | SAC critic 안정 |
| `train/ent_coef` | 단조 감소 (1.0 → 0.2) | exploration → exploitation 전환 |
| `env/total_drop_terminated_count` | 단조 ↑, 50k step 당 ≥ 25 | drop 학습 진행 |
| `env/total_safety_violation_count` | 증가율 감소 (학습 후반 거의 정체) | crash/overspeed 빈도 감소 |
| `env/mean_d_xy` | 단조 감소 추세 (rolling avg) | target 접근 |
| `rollout/ep_rew_mean` | 단조 또는 점진 개선 | 전체 보상 학습 |

**발산 / 비정상 신호** (이 중 하나라도 → alert/abort)

| metric | 임계 | 의미 |
|---|---|---|
| `train/critic_loss` | > 5000 | critic 발산 (junsang_v2 패턴) |
| `train/ent_coef` | > 1.0 또는 증가 추세 | SAC auto-temp 통제 불능 |
| `env/total_drop_terminated_count` | 50k step 이후도 < 25 | drop 학습 못함 |
| `env/total_safety_violation_count` | 학습 후반 가파른 ↑ | 정책이 자주 crash |
| `env/mean_d_xy` | 100k 이후도 > 50m 진동 | target 접근 못함 |

(이전 rate metric `env/truncate_*_rate`, `safety_violation_rate` 등은 entry #13 에서 제거됨. x축은 wandb 대시보드에서 `env/total_episodes` 로 설정 권장 — `wandb.define_metric` 으로 자동.)

### 2B-3. 자동 모니터링 구현 옵션

**옵션 A — 외부 watch 스크립트 (가벼움, Tier 1+ 권장)**:
- `wandb_watch.py` — 매 5분 wandb API 로 latest metric 읽음
- 위 임계 위반 시 console alert + log
- 학습 자체엔 영향 없음 — 사용자 판단으로 SIGINT 중단

**옵션 B — train_sac.py 자체 abort 콜백 (Tier 2)**:
- 매 10k step rollout 후 자체 검사
- critic_loss > 10000 또는 50k 후 drops < 5 → `sys.exit(1)`
- 자동 보호 but 잘못된 임계로 정상 학습 중단 위험

**옵션 C — 학습 도중 periodic evaluate (Tier 3, 비용 큼)**:
- 매 50k step 마다 별도 evaluate (deterministic 5 epi)
- sim cost 2배 — 비용 정당화 어려움

**권장**: 첫 cycle (Run #12) 은 옵션 A 만. 결과 보고 옵션 B 도입.

---

## 3. 컴포넌트별 진단

### 3-1. Action space — **너무 강함**

| 항목 | 값 | 진단 |
|---|---|---|
| `action_vx_scale` | 15 m/s | action[0]=1 이면 즉시 15 m/s 명령 → 사용자 관찰 "시작하자마자 튀어나가는" 직접 원인 |
| `action_vy_scale` | 5 m/s | OK |
| `action_vz_scale` | 3 m/s | action[2]=-1 이면 -3 m/s 하강 → 즉시 바닥 충돌 가능 |
| `action_yaw_scale` | 1 rad/s | ≈57°/s, 적당 |

**문제**: 학습 초기 random action 의 `vx=±15 m/s + vz=-3 m/s` 가 자주 나오면 → 드론이 시작 즉시 비정상 비행 → 학습 데이터 오염.

### 3-2. Observation — **normalisation 약점**

| 항목 | 값 | 진단 |
|---|---|---|
| `pos_scale` 50m | obs = pos/50 | target 50m 이상 거리에서 obs > 1 → Box(-1,1) clip 영역. 정책이 그 영역 학습 못함 |
| `vel_scale` 15 | obs = vel/15 | action_vx_scale 과 일치 |
| `d_impact / 50m` | obs[15] | CCIP 50m 이상에서 1.0 clip |
| L4 (Obs clip) | 미적용 | 이전 회의록 LOW priority 였으나 문제 발생 가능 |

**문제**: 학습 초기 d_xy 가 100~300m 자주 나옴 (timeline 분석). 그 동안 obs[rel_target] = clip(±1.0) — 정책이 "멀리 떨어진 상태" 를 구분 못함.

### 3-3. Reward 4-Layer — **scale 격차 + 약한 신호**

| Layer | 값 | 문제 |
|---|---|---|
| L1 안전 | penalty_crash **-10**, penalty_overspeed **-8** | 너무 약함 (사용자 명시) |
| L2 안정 | w_time 0.01, w_ang_vel 0.05, w_action_smooth 0.05 | 약함 (per-step ~±0.05) |
| L3 접근 | w_dist 0.5, w_heading 0.7, w_impact 0.4 | per-step ~0.1~0.5 — 약함, 학습 동기 부족 |
| L4 terminal | drop_attempt_bonus **+150**, w_drop_base 100, jackpot 100 | **+250 ~ +350 압도적**, truncation -30 (Tier1 적용 후) 또는 -80 (현재) |

**문제**:
- per-step (~0.5) vs terminal (+250~350) 격차 500배+
- crash penalty (-10) 가 per-step reward 보다 큰 negative 인데도 episode 안 끝남 → bad sample 누적

### 3-4. Episode 종료 — **early termination 부재**

| 조건 | 처리 |
|---|---|
| `dropped = True` (CCIP threshold 만족) | terminated, Layer 4 reward |
| `step_count >= 500` | truncated, -80 penalty |
| `altitude < min_altitude` (crash) | **종료 X** — reward -10 만, episode 계속 |
| `speed > 20 m/s` (overspeed) | **종료 X** — reward -8 만, episode 계속 |
| `tilt > limit_tilt` (15°) | 검사 X (drop 시점만) |

**문제 (사용자 명시한 진단)**:
- 드론이 바닥에 부딪힌 채로 episode 계속 → buffer 에 망가진 transition 누적
- "안 좋은 훈련 빨리 제거" 가 안 됨
- 이 망가진 sample 들이 후속 gradient update 에서 학습 신호로 사용

### 3-5. CCIP + Drop trigger

| 항목 | 값 | 문제 |
|---|---|---|
| `auto_drop_threshold` | 4m | 학습 초기 random policy 가 4m 안 거의 못 들어감 → drop 경험 sparse |
| `success_threshold` | 0.1m | jackpot 절대 못 받음 (현실적으로 0.5m 도 어려운데 0.1m) |
| `k2_precision` | 0.3 | jekyun v2 변경, 먼 거리도 의미있는 보상 ✓ |
| `drop_wait_timeout` | 10s | OK |

**문제**: threshold 4m 가 너무 빡셈. 사용자가 명시한 "10m 안으로 잡고 일단 성공" 의 정확한 대응 = threshold 4m → 10m.

### 3-6. SAC — **안정성 회복됨**

| 항목 | 값 | 진단 |
|---|---|---|
| `buffer_size` | 500k (M1) | OK |
| `gamma` | 0.995 (H3) | OK |
| `gradient_steps` | 1 (M2 REVERTED) | OK (#11 에서 critic 18.5 안정 확인) |
| `learning_rate` | 3e-4 | OK |
| `learning_starts` | 1000 | OK |

**결론**: SAC 측은 문제 없음. 문제는 **env/reward 측**.

---

## 4. 사용자 관찰 → 진단 매핑

| 사용자 관찰 | 진단 (§ 참조) | 관련 P | Tier |
|---|---|---|---|
| "시작하자마자 튀어나감" | § 3-1 (action_vx_scale 15 너무 큼) | P1 | T1 |
| "땅에 부딪혀 돈다" | § 3-3 (penalty_crash 약함) + § 3-4 (episode 안 끝남, instability 검사 없음) | P4, P5, P11 | T1 |
| "중간부터 이상하게 학습됨" | § 3-4 (bad transition buffer 누적) + § 3-3 (reward scale 격차) | P9, P8 | T1 |
| "drops 발생해도 학습 안 됨" | § 3-5 (drops 가 우연/threshold 효과) + § 3-3 (per-step 약함) | P6, P7 | T1+T2 |
| "d_xy 안 줄어듦, 20m 정체" | § 3-3 (per-step reward 약함) + § 3-2 (obs clip 영역) | P7 | T2 |
| "안 좋은 학습 빨리 제거" | § 3-4 (early termination 부재) | P9 | T1 |
| "10m 안으로 일단 성공" | § 3-5 (threshold curriculum phase 1) | P6 | T1 |
| "급격한 가속 / 큰 흔들림 차단" | § 3-1 (action scale) + P2 (rate limit 없음) + § 3-4 (instability 검사 부재) | P1, P2, P11 | T1 |
| "안 좋은 학습 판단 근거" | (현재 부재) | P10 | T2 |

---

## 5. 처방 매트릭스

### Tier 1 — 즉시 적용 (사용자 명시 + 가장 큰 영향)

| # | 변경 | 이전 → 새 | 위치 | 근거 (P#) |
|---|---|---|---|---|
| 1 | `environment.action_vx_scale` | 15.0 → **8.0** | yaml | P1 — 최대 속도 (수평) 제한 |
| 2 | `environment.action_rate_limit` (NEW key) | — → **0.2** | yaml + drone_drop_env.py step() | P2 — 가속도 제한 (hard clip) |
| 3 | `environment.min_altitude` | 2.0 → **3.0** | yaml | P4 — 고도 최저선 상향 |
| 4 | `environment.min_altitude_start_step` | 20 → **10** | yaml | P4 — 검사 시작 빠르게 |
| 5 | `reward.penalty_overspeed` | -8 → **-50** | yaml | P3 — overspeed 강력 차단 |
| 6 | `reward.penalty_crash` | -10 → **-100** | yaml | P5 — 바닥 충돌 차단 |
| 7 | `reward.auto_drop_threshold` | 4.0 → **10.0** | yaml | P6 — curriculum phase 1 |
| 8 | `reward.truncation_penalty` | -80 → **-30** | yaml | P8 — timeout 압도 완화 |
| 9 | `reward.limit_inverted_tilt` (NEW key) | — → **1.047** (60°) | yaml | P11 — inverted 자세 판정 임계 |
| 10 | `reward.penalty_bad_attitude` (NEW key) | — → **-50** | yaml | P11 — 각속도/inverted penalty |
| 11 | `wandb.run_name` | "junsang_v3" → **"junsang_v4"** | yaml | 새 run 식별 |
| 12 | **crash 시 `truncated=True`** | (none) → (new) | drone_drop_env.py | P5/P9 |
| 13 | **overspeed 시 `truncated=True`** | (none) → (new) | drone_drop_env.py | P3/P9 |
| 14 | **각속도 폭증 (\|ω\| > limit_ang_vel) 시 `truncated=True` + penalty** | (none) → (new) | drone_drop_env.py | P11 — "절대 안 좋은 행동" (yaml `limit_ang_vel=2.0` 키 재사용, 새 키 추가 X) |
| 15 | **Inverted (\|roll\|/\|pitch\| > limit_inverted_tilt) 시 `truncated=True` + penalty** | (none) → (new) | drone_drop_env.py | P11 — "절대 안 좋은 행동" |
| 16 | **`info['truncate_reason']` wandb 로깅 callback** (NEW) | (none) → (new) | train_sac.py `WandbMetricsCallback` | P11 모니터링 — crash/overspeed/ang_vel/inverted 비율 추적 |
| **17** | **WandB callback overhaul: rate → 누적 count + episode x축** ([entry #13](parameter_log.md)) | rate 8개 → 누적 count 9개 + `wandb.define_metric` | train_sac.py `WandbMetricsCallback` + `main()` | rollout 작을 때 rate 가 0/1 binary 진동 문제. 누적 count + x축 = `env/total_episodes` 로 깔끔한 추세 시각화. **src 만 수정 (Phase 1 학습 영향 0)**, Phase 2 진입 시 install 미러 |

### Tier 2 — Tier 1 결과 보고 결정

| 변경 | 이전 → 새 (제안) | 근거 |
|---|---|---|
| `environment.action_vz_scale` | 3.0 → 2.0 | vz 도 즉시 바닥 충돌 가능 |
| `reward.limit_tilt` (drop 시점 검사) | 0.26 (15°) → 0.20 (11°) | drop 시 instability 검사 강화 |
| Per-step reward 강화 | (0.5/0.7/0.4) → (1.0/1.0/0.5) | P7. orbit-milking 위험 — Tier 1 안전조건 후만 |
| `wandb_watch.py` (외부 모니터링 스크립트) | (none) → (new) | P10 — 자동 진행 판단 (§ 2B-3 옵션 A) |

### Tier 3 — 다음 라운드 (대규모)

- Observation 정규화 재검토 (P7 의 obs[rel_target] clip 영역 문제 — Box clip 처리 또는 scale 조정)
- `max_steps` 500 → 300 (정상 episode 도 짧아져 학습 throughput ↑. Tier 1 의 early terminate 와 보완)
- CCIP threshold curriculum (10 → 5 → 2) — phased (사용자 명시 "10m 일단 성공 → 좁혀가기")
- evaluate 평가 metric 자동화 (학습 도중 deterministic check — § 2B-3 옵션 C)

---

## 6. 새 학습 사이클 plan

### Run #12 (junsang_v4) — Tier 1 한 번에 적용

#### 6-1. 사전 정리

- **Container 안 살아있는 sim process 모두 종료** (pkill 의 -f 는 substring 매칭이라 OR 패턴 안 됨 — 개별 호출):
    ```bash
    docker exec drone-bombard-harmonic bash -c "\
      pkill -9 -f 'gz sim'; \
      pkill -9 -f 'bin/px4'; \
      pkill -9 -f 'MicroXRCEAgent'; \
      pkill -9 -f 'parameter_bridge'; \
      pkill -9 -f 'train_sac'"
    ```
- **임시 파일 정리**: `rm -f /dev/shm/fastrtps_* /tmp/px4_lock-* /tmp/px4-sock-*`
- **이전 run 체크포인트 archive 이동**: `archive/junsang_v3_w9flirvp_159k_failed_2026-05-23/`
- 새 학습은 자동으로 fresh buffer 시작 (Stable-Baselines3 의 SAC 가 매 run 마다 새 buffer 생성) — `--resume <path>` cli arg 안 쓰면 자동으로 fresh. 우리는 reward 공식 변경이라 [CLAUDE.md 의 RL 규칙](../Drone-Bombard-Simulation/CLAUDE.md) 의 "**보상 공식 변경 → 반드시 Fresh Start**" 도 만족

#### 6-2. Yaml 변경 (총 11 키)

```yaml
environment:
  action_vx_scale: 8.0          # P1: 15 → 8 (최대 속도 수평 제한)
  action_rate_limit: 0.2        # P2 NEW: 가속도 hard clip (step 당 |Δa| ≤ 0.2)
  min_altitude: 3.0             # P4: 2.0 → 3.0 (고도 최저선 상향)
  min_altitude_start_step: 10   # P4: 20 → 10 (검사 빨리)
reward:
  penalty_crash: -100           # P5: -10 → -100
  penalty_overspeed: -50        # P3: -8 → -50
  auto_drop_threshold: 10.0     # P6: 4 → 10 (curriculum phase 1)
  truncation_penalty: -30       # P8: -80 → -30
  limit_inverted_tilt: 1.047    # P11 NEW: π/3 (60°) — step 별 inverted 자세 임계 (drop 시점은 limit_tilt 별도)
  penalty_bad_attitude: -50     # P11 NEW: 각속도/inverted 위반 페널티
  # NOTE: 기존 키 유지 (변경 X):
  #   - limit_ang_vel=2.0 (drop 시점 + P11 step 별 검사 모두 사용)
  #   - limit_tilt=0.26 (drop 시점 instability 검사만, ~15°)
  #   - jekyun v3 의 모든 reward 키 유지 (w_drop_base=100, r_success_jackpot=100,
  #     drop_attempt_bonus=150, k2_precision=0.3, w_dist=0.5, w_heading=0.7,
  #     w_impact=0.4, k_impact=0.05, penalty_instability=50 등) — Tier 1 효과 확인 후 검토
wandb:
  run_name: "junsang_v4"        # 새 run name
```

#### 6-3. Code 변경 — `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`

**(0) Config 로드 — `__init__` 에 신규 키 3개 추가** (먼저 적용)

```python
# environment section
self._cfg_action_rate_limit = cfg_env.get('action_rate_limit', 0.2)
# reward section
self._cfg_limit_inverted_tilt = r.get('limit_inverted_tilt', 1.047)
self._cfg_penalty_bad_attitude = r.get('penalty_bad_attitude', -50.0)
```

**(1) step() flow** — action 받음 → **rate clip (P2)** → velocity publish → state 받음 → 기존 `_compute_reward` 호출 → **truncated 결정 (P11 + 기존 crash/overspeed)** → max_steps 처리 → obs → return

**실제 코드 signature 검증** (drone_drop_env.py grep 결과):
- `def _compute_reward(self, pos, vel, ang, pix, d_xy, d_impact, action)` — **3-tuple `(reward, False, info)` 반환** (L781, L895). **terminated 항상 False** — `_compute_reward` 는 non-terminal step 만 처리
- Drop terminate path 는 **step() 의 별도 분기** (L657-677) — `_compute_reward` 호출 안 함. layer 4 reward + `terminated=True` 직접 처리
- `info['crash']`, `info['overspeed']` 는 **`_compute_reward` 의 Layer 1 안에서 set** (L806, L810) — penalty 만 add, terminate 안 함
- `truncated` 는 **step() 안 max_steps 처리에서만 set** (L688)
- `_V_MAX_SAFETY = 20.0` 은 class attribute (L313, hardcoded, yaml 외)

따라서 **`_compute_reward` 와 drop terminate path 는 변경 없음**, step() 의 _compute_reward 호출 *뒤* 에 truncated + P11 검사 + truncate_reason 결정 추가.

**Step() 의 분기 구조 (변경 없음, 참고)**:
```
step()
  ├── 1. rate clip (NEW P2)
  ├── 2. velocity publish (기존)
  ├── 3. state snapshot (기존)
  ├── 4. d_xy, d_impact 계산 (기존)
  ├── 5. if d_impact ≤ auto_drop_threshold and not dropped:
  │     └── drop 발동 분기 (L657-677, 기존) — Layer 4 reward, terminated=True
  └── 6. else (non-drop step):
        ├── _compute_reward 호출 (L1+L2+L3, 기존)
        ├── NEW: truncated + truncate_reason 결정 (crash/overspeed/ang_vel/inverted)
        └── NEW: max_steps 처리 + truncate_reason='timeout'
return (obs, reward, terminated, truncated, info)
```

```python
def step(self, action):
    self._step_count += 1

    # P2: action rate limit (가속도 hard clip) — action_prev 는 reset() 에서 zeros 로 초기화됨
    action = np.clip(
        action,
        self.action_prev - self._cfg_action_rate_limit,
        self.action_prev + self._cfg_action_rate_limit,
    )

    # action → velocity setpoint (기존)
    vx = float(action[0]) * self._cfg_action_vx_scale
    vy = float(action[1]) * self._cfg_action_vy_scale
    vz = float(action[2]) * self._cfg_action_vz_scale
    yaw_rate = float(action[3]) * self._cfg_action_yaw_scale
    self._node.publish_velocity(vx, vy, vz, yaw_rate)

    # state snapshot (기존 코드 그대로)
    self._obs_ready.wait(timeout=self._cfg_obs_wait)
    with self._state_lock:
        pos = self._node.pos_enu.copy()
        vel = self._node.vel_enu.copy()
        ang = self._node.ang_vel.copy()
        roll = self._node.roll
        pitch = self._node.pitch
        pix = self._node.pixel_coords.copy()

    d_xy = self._compute_d_xy(pos)
    _, _, t_f, d_impact = self._predict_impact_point(pos, vel)

    # 기존 _compute_reward (signature/반환 유지)
    # info 안에 crash/overspeed boolean 이 이미 채워짐 (Tier 1 yaml 의 새 penalty 값으로 add 됨)
    reward, terminated, info = self._compute_reward(
        pos, vel, ang, pix, d_xy, d_impact, action)

    # NEW: truncated + P11 추가 검사 — drop terminated 면 skip
    # NOTE: terminated 는 drop terminate path 에서만 True. _compute_reward 는 항상 False.
    truncated = False
    truncate_reason = None
    if not terminated:
        # 우선순위: crash > overspeed > ang_vel > inverted
        # crash/overspeed 는 _compute_reward 의 Layer 1 안에서 이미 info['crash'/'overspeed']
        # 설정 + penalty add. 여기선 truncate_reason 만 결정.
        if info.get('crash'):
            truncated = True
            truncate_reason = 'crash'
        elif info.get('overspeed'):
            truncated = True
            truncate_reason = 'overspeed'
        elif np.linalg.norm(ang) > self._cfg_limit_ang_vel:
            info['bad_attitude'] = 'ang_vel'
            reward += self._cfg_penalty_bad_attitude
            truncated = True
            truncate_reason = 'ang_vel'
        elif (abs(roll) > self._cfg_limit_inverted_tilt
              or abs(pitch) > self._cfg_limit_inverted_tilt):
            info['bad_attitude'] = 'inverted'
            reward += self._cfg_penalty_bad_attitude
            truncated = True
            truncate_reason = 'inverted'

    # 기존 max_steps 처리 (truncation_penalty 는 Tier 1 의 -30 적용됨)
    if not terminated and not truncated and self._step_count >= self._cfg_max_steps:
        truncated = True
        reward += self._cfg_truncation_penalty
        truncate_reason = 'timeout'

    if truncate_reason:
        info['truncate_reason'] = truncate_reason

    # action_prev 갱신 (rate clip + 기존 action_smooth penalty 용)
    self.action_prev = action.copy()

    obs = self._get_obs()
    return obs, reward, terminated, truncated, info
```

**reset() 의 `action_prev` 초기화** (drone_drop_env.py 기존 L415, L465 에 이미 존재):
```python
self.action_prev = np.zeros(4, dtype=np.float32)   # P2 의 rate_clip 가 zeros 기준으로 작동
```

**(2) 코드 통합 요약**

- **`_compute_reward` 는 변경 없음** — 기존 signature `(pos, vel, ang, pix, d_xy, d_impact, action)` + 반환 `(reward, terminated, info)` 그대로. yaml 의 `penalty_crash=-100`, `penalty_overspeed=-50` 새 값을 자동으로 받음 (`self._cfg_penalty_crash` 등)
- **P11 의 ang_vel/inverted 검사는 step() 안** — `_compute_reward` 호출 후, `terminated` 가 False 일 때만
- **`truncated` 와 `truncate_reason` 은 step() 가 결정** — 가능한 값: `crash, overspeed, ang_vel, inverted, timeout` 5개

**기존 코드와의 통합 점**:
- 기존 L803-816 의 `_compute_reward` 안 Layer 1 검사 (crash/overspeed) — **그대로 유지** (penalty 만 add, terminate 안 함)
- 기존 L687-690 의 max_steps truncate — 위 코드로 통합
- 기존 L661-663 의 drop 시점 instability 검사 (Layer 4) — **그대로 유지** (drop 발동 시 `penalty_instability` add, terminated)

**가정** (drone_drop_env.py grep 으로 검증됨):
- `self._cfg_limit_ang_vel = r.get('limit_ang_vel', 2.0)` 이미 존재 (L381)
- `self._cfg_limit_tilt = r.get('limit_tilt', 0.26)` 이미 존재 (L382, drop 시점만 사용)
- `self._cfg_obs_wait = cfg_env.get('obs_wait_timeout', 0.15)` 이미 존재 (L360)
- step 별 검사는 두 임계 분리: tilt 측은 `limit_inverted_tilt=1.047` (60°, 신규 키), ang_vel 측은 `limit_ang_vel=2.0` 재사용

**우선순위 (동시 발생 시)**:
- drop terminate (Layer 4) > crash (_compute_reward) > overspeed > ang_vel (P11) > inverted (P11) > timeout (max_steps)
- 한 step 에 여러 위반 동시 발생해도 첫 위반만 truncate_reason 으로 기록, reward penalty 도 첫 위반만 add (단 crash/overspeed 는 `_compute_reward` 안에서 합산 가능 — 우선순위 절단 안 됨. 검사 후 step() 가 truncate_reason 만 우선순위로 선택)

**SB3 SAC 의 truncate 처리**:
- 우리 `truncated=True` 는 gym 0.26+ 표준 — Stable-Baselines3 의 ReplayBuffer 에 `done` 으로 저장 + next obs 도 저장 (TimeLimit-style)
- bootstrap 시 next_value 사용 가능 → bias 없음. 즉 early truncate 가 RL 학습에 정상적으로 통합됨

**(3) `train_sac.py` 의 callback — `info['truncate_reason']` wandb 로깅** (Tier 1 # 16)

> ⚠ **갱신 (Tier 1 #17, parameter_log entry #13)**: 이 코드 sketch 는 rate metric 기반. 이후 누적 count 로 변환됨 (rate 8개 제거, 누적 count 9개 추가, `env/total_episodes` 가 x축). 실제 적용 코드는 [parameter_log.md entry #13](parameter_log.md) 참조. 아래는 #16 시점 구현 (Phase 1 학습 영향 0 — install 미러 시 #17 의 누적 count 코드가 적용됨).

`WandbMetricsCallback` (`ros2_ws/src/rl_navigation/rl_navigation/train_sac.py`) 의 `__init__`, `_on_step`, `_on_rollout_end` 3곳 수정. 5개 reason (drop terminated 외): `crash, overspeed, ang_vel, inverted, timeout`.

```python
# __init__ — counter 추가
self._truncate_counts = {
    'crash': 0, 'overspeed': 0, 'ang_vel': 0, 'inverted': 0, 'timeout': 0
}
self._rollout_done_episodes = 0   # rollout 동안 done=True (terminated|truncated) 인 episode 수

# _on_step — info 의 truncate_reason 집계 (infos/dones 는 SB3 의 locals 에서 받음)
def _on_step(self) -> bool:
    infos = self.locals.get('infos', [])
    dones = self.locals.get('dones', [])
    for info, done in zip(infos, dones):
        # ... 기존의 d_xy, rew_* 집계 코드 그대로 ...

        if not done:
            continue
        self._rollout_done_episodes += 1
        reason = info.get('truncate_reason')
        if reason in self._truncate_counts:
            self._truncate_counts[reason] += 1
        # NOTE: reason 이 None 이면 drop terminated — truncate counter 에 안 들어감
    return True

# _on_rollout_end — wandb 로깅 + reset
if self._rollout_done_episodes > 0:
    for reason, count in self._truncate_counts.items():
        log_dict[f'env/truncate_{reason}_rate'] = count / self._rollout_done_episodes
    # drop 비율 = 1 - sum(위 5개) → drop rate 도 자동 계산 가능
    drop_episodes = self._rollout_done_episodes - sum(self._truncate_counts.values())
    log_dict['env/drop_terminated_rate'] = drop_episodes / self._rollout_done_episodes
    # reset counters
    self._truncate_counts = {k: 0 for k in self._truncate_counts}
    self._rollout_done_episodes = 0
```

→ wandb 에 등장하는 6 metric (rollout 단위 비율, 합 = 1.0):
- `env/truncate_crash_rate` — 학습 도중 단조 감소가 핵심
- `env/truncate_overspeed_rate`
- `env/truncate_ang_vel_rate`
- `env/truncate_inverted_rate`
- `env/truncate_timeout_rate`
- `env/drop_terminated_rate` — 학습 도중 단조 증가가 핵심 (drop 성공률)

#### 6-4. install/share 미러 (메모리 [feedback_ros2_install_cache])

src 변경 후 매번:
- yaml: `docker cp .../config/hyperparams.yaml drone-bombard-harmonic:/workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml`
- drone_drop_env.py: 동일 경로의 `lib/python3.10/site-packages/rl_navigation/drone_drop_env.py`

#### 6-5. Run name + parameter_log

- `wandb.run_name = "junsang_v4"`
- `parameter_log.md` entry #12 추가 (Tier 1 변경 11 + 코드 패치 + 검증 기준)

#### 6-6. 검증 단계

**Step 1 — 5k dry-run** (WANDB_MODE=offline 권장)

통과 기준 (정량):
- ✓ RuntimeError / Traceback 없음
- ✓ Training complete 메시지 (자연 완주)
- ✓ `env/total_drop_count` ≥ 1 (학습 동기 신호 작동 확인)
- ✓ `env/safety_violation_rate` < 1.0 (모든 step crash 가 아님)
- ✓ `env/mean_d_xy` 측정됨 (NaN 아님)
- ✓ `rollout/ep_len_mean` < 500 (episode 가 truncation 외에 다른 이유로도 끝남 — early terminate 작동 확인)
- ✓ **임계 적정성** — `env/total_truncate_ang_vel_count` / `env/total_episodes` < 0.5 (만약 ≥ 50% 이면 `limit_ang_vel` 또는 `action_rate_limit` 너무 빡셈 → 5k 더 보고 조정)
- ✓ **답답함 점검** — `env/mean_d_xy` 가 5k 동안 어느 정도 감소 (시작 ~50~100m → 끝 ~30~50m). 만약 5k 동안 거의 안 줄어들면 `action_rate_limit` 0.2→0.3 으로 완화 검토

**Step 2 — 200k 본학습** (WANDB_MODE=online)

학습 도중 모니터링 (10~30분 단위로 wandb 대시보드 — 200k = 약 ~2~7시간 예상이라 4~12 회 확인):
- `env/safety_violation_rate` 학습 도중 단조 감소 (학습 초기 ~0.5~0.8 → 50k 이후 < 0.3)
- `env/total_drop_count` stair-step 증가 (10k step 당 ≥ 5 drops, 50k 이후)
- `env/mean_d_xy` 단조 감소 추세 (rolling avg)
- `train/critic_loss` < 500 안정 — 폭주 (>5000) 시 abort 결정
- `train/ent_coef` 단조 감소 (1.0 → 0.5 → 0.2)
- `rollout/ep_len_mean` — 학습 초기엔 짧고 (early terminate 작동), 학습 진행되면 길어짐 (50k 이후 평균 ≥ 200)
- `env/total_truncate_<reason>_count` (5개: crash/overspeed/ang_vel/inverted/timeout) — 학습 후반 증가율 감소가 핵심, 특히 crash count 의 plateau 가 가장 중요 (entry #13 갱신 후)

**Step 3 — 200k 완주 후 성공 판정**

| 지표 | 성공 기준 | 진단 |
|---|---|---|
| `env/total_drop_count` | ≥ 30 (드물지만 학습 가능 영역) | drop 학습 진행 |
| `env/mean_d_xy` (final) | ≤ 10m | 사용자 "10m 안 성공" 달성 |
| `env/total_truncate_crash_count` 증가율 (마지막 50k step) | < total_episodes 증가율 × 0.1 | "안 좋은 학습" 차단 — Tier 1 의 P5 효과 확인 |
| `train/critic_loss` (final) | < 500 | SAC 안정 유지 |
| `train/ent_coef` (final) | < 0.5 | exploitation 전환 |
| Deterministic evaluate 5 epi | drops ≥ 2, 평균 reward > 0 | 정책 학습 검증 |

분기:
- **성공** (위 6개 중 5개 이상) → Run #13: threshold 10→5m (curriculum phase 2)
- **부분 성공** (drops 있지만 d_xy 정체) → Run #13: per-step reward 강화 (Tier 2)
- **실패** (drops < 5, d_xy > 30m) → Tier 1 의 수치 재조정 (예: action_rate_limit 0.2→0.1, penalty_crash -100→-50) 또는 Tier 3 (observation 정규화)

> **참고**: Run #13~ 의 상세 분기는 위 § 6-6 Step 3 의 분기 (성공/부분 성공/실패 트리거) 와 동일. 중복 제거를 위해 § 6-6 만 참조.

---

## 7. 폐기/유지 결정

| 항목 | 결정 | 이유 |
|---|---|---|
| `sac.gradient_steps = 1` (M2 REVERTED) | ✅ 유지 | #11 에서 critic 18.5 안정 확인 |
| `sac.buffer_size = 500000` (M1) | ✅ 유지 | catastrophic forgetting 방지 |
| `sac.gamma = 0.995` (H3) | ✅ 유지 | effective horizon 확장 |
| `eval_freq, eval_episodes` (L6) | ✅ 유지 | callback infra |
| `_step_rew_impact` callback | ✅ 유지 | 측정 인프라 |
| `_total_drop_count` counter | ✅ 유지 | stair-step 차트 |
| jekyun_v2 인프라 patch (spin/reset/infra kill) | ✅ 유지 | 14m offset 해결됨 |
| jekyun v3 reward (w_drop_base 100, jackpot 100, drop_attempt_bonus 150) | ✅ 유지 | Tier 1 효과 확인 후 검토 |
| `speed_gate_enabled: true` (jekyun_v2 그대로) | ✅ 유지 | r3_orient 에만 speed_gate 적용 (L870-871). **r3_impact 에는 미적용** — w_impact=0.4 작아서 loitering hack 위험 낮음 (junsang N1=B v2 의 speed_gate*r3_impact patch 는 폐기) |
| `wandb.mode = online` | ✅ 유지 | 사용자 선택 |

---

## 8. 폐기된 시도들 (참고)

| Run | WandB run_id | 결정 | 이유 |
|---|---|---|---|
| junsang N1=B v2 (w_impact=8) | `um8txjvk` | ❌ 폐기 (2026-05-22) | per-step 강화 실패 모드 (orbit milking + critic 발산) |
| junsang_v2 (jekyun base + H3/M1/M2) | `zn7xrm7e` | ❌ 폐기 (2026-05-22) | M2=4 + 큰 terminal → critic 17K 발산 (63k 시점) |
| junsang_v3 (M2 복원만) | `w9flirvp` | ❌ 폐기 (2026-05-23) | SAC 안정성은 회복했으나 정책 학습 안 됨 (159k 까지 mean_d_xy 정체 ~20m) — env/reward 측 근본 문제 |

---

## 9. 다음 행동 (사용자 결정 필요 X — 명시된 처방)

1. **사전 정리** — sim process 모두 종료, 임시 파일 정리, 이전 체크포인트 archive 이동 (§ 6-1)
2. ⚠ **Tier 1 적용** (총 16 항목 — § 5 Tier 1 표):
   - yaml 11 키 변경/추가 (action_vx_scale, action_rate_limit, min_altitude, min_altitude_start_step, penalty_overspeed, penalty_crash, auto_drop_threshold, truncation_penalty, limit_inverted_tilt, penalty_bad_attitude, wandb.run_name)
   - drone_drop_env.py 코드 변경: cfg 로드 3 키 + step() 의 action rate clip + _compute_reward 의 safety 검사 if-elif 4가지 (crash/overspeed/ang_vel/inverted) (§ 6-3 단일 패턴 참조)
   - train_sac.py callback 변경: rate → 누적 count overhaul (entry #13). `env/total_episodes` + `env/total_truncate_<reason>_count` 5개 + `env/total_drop_terminated_count` + `env/total_success_count` + `env/total_jackpot_count` + `env/total_safety_violation_count` + `env/total_physics_glitch_count` 누적 metric
3. **install/share 미러** (§ 6-4)
4. **parameter_log entry #12 추가** (§ 6-5)
5. **사용자가 5k dry-run 실행** (WANDB_MODE=offline) — § 6-6 Step 1 통과 기준 만족 확인 (임계 적정성 + 답답함 점검 포함)
6. **5k 통과 → 사용자가 200k 본학습 실행** (WANDB_MODE=online)
7. (선택 Tier 2) `wandb_watch.py` 외부 모니터링 스크립트 작성 — § 2B-3 옵션 A

---

끝.

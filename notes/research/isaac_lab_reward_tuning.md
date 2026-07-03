---
date: 2026-07-03
tags: [research, isaac-lab, reward, hyperparameters, tuning, migration, onboarding]
status: active
type: research
---

# Isaac Lab 보상·하이퍼파라미터 레퍼런스 (`feat/isaac-env-migration`)

> **대상 독자:** `feat/isaac-env-migration` 브랜치(Isaac Lab + PPO/rsl_rl)에서 보상 공식,
> 순항 고도, 속도 스케일 등을 바꿔가며 실험할 연구자. Gazebo/SAC 트랙(`jekyun`)의
> [[research/reward_design]] / [[research/rl_rules]] 와는 **별도 코드베이스**이므로
> 파라미터 위치·이름이 다르다 — 이 문서가 Isaac 쪽 매핑표.
>
> **전체 아키텍처:** [[research/isaac_lab_architecture]] · **컨트롤러 게인:**
> [[research/isaac_velocity_controller]] · **이식 시 준수 규칙:** [[research/rl_rules]] Rule 16

---

## 0. Gazebo 트랙과 가장 다른 점 — 설정 파일이 없다

Gazebo/SAC 트랙은 `ros2_ws/src/rl_navigation/config/hyperparams_v13.yaml`처럼 **외부 YAML**을
읽었다. Isaac 트랙은 그런 파일이 없다 — 모든 값이
`isaac_lab/drone_bombard/drone_bombard_env.py` 상단의 `@configclass` 데이터클래스
(`DroneBombardRewardCfg`, `DroneBombardTerminationCfg`, `DroneBombardActionCfg`,
`DroneBombardResetCfg`, `DroneBombardVisionCfg`, `DroneBombardAssetCfg`,
`DroneBombardDropCfg`, `DroneBombardControllerCfg`)에 **Python 기본값**으로 박혀 있다.

**바꾸는 방법 두 가지:**
1. `drone_bombard_env.py`의 기본값을 직접 수정 (가장 흔한 방법 — git diff로 변경 이력이
   남으므로 실험 재현에 유리).
2. `train.py`가 `DroneBombardEnvCfg()`를 인스턴스화한 직후(§`main()` 상단)에 필드를
   덮어쓰는 코드를 추가 (`env_cfg.reward.w_dist = 3.0` 등). 여러 실험을 스크립트로
   스윕할 때 편리하지만, 코드에 없는 값이므로 **실험 노트에 반드시 기록**할 것
   ([[experiments/training_history]] 참조 — CLAUDE.md MANDATORY 로깅 규칙).

값의 "근거"는 대부분 `drone_bombard_env.py` / `math_utils.py` 주석과
[[experiments/exp_012_isaac_migration_phase2]]의 parity 표에 있다 — v13/v15 Gazebo 상수를
그대로 이식한 값이 대부분이므로, 왜 이 숫자인지 궁금하면 그 표부터 볼 것.

---

## 1. 액션 / 속도 스케일 — `DroneBombardActionCfg`

정책이 내는 액션은 항상 `[-1, 1]` — 실제 물리 단위로 바꾸는 배율이 여기 있다.
"드론 속도를 얼마나 빠르게/느리게 허용할지"를 바꾸고 싶으면 이 표.

| 필드 | 기본값 | 의미 |
|---|---|---|
| `vx_scale` | 4.0 m/s | 전후(ENU X) 최대 속도 |
| `vy_scale` | 3.0 m/s | 좌우(ENU Y) 최대 속도 |
| `vz_scale` | 3.0 m/s | 수직(ENU Z) 최대 속도 |
| `yaw_scale` | 1.0 rad/s | 요 회전 속도 |
| `rate_limit` | 0.2 | 정규화 액션의 스텝당 최대 변화량(가속도 하드클립, 물리 스케일 적용 **전**) |
| `lpf_alpha` | 0.4 | 속도 명령 저역통과(EMA) 계수 — [[research/control_smoothness_wobble]] Rule 15와 동일 값 |
| `lpf_tick_period_steps` | 5 | LPF가 몇 물리스텝마다 갱신되는지 (100Hz/5=20Hz, PX4 로직 주기 재현) |

**튜닝 시 주의:**
- `vx_scale`/`vy_scale`을 올리면 Rule 10(overshoot moat)이 재도입될 수 있다 — 핸드오프
  거리(`reset.handoff_dist_range`, §3)에 비해 액션이 너무 크면 `success_radius`를 지나쳐
  버린다. 액션 스케일은 항상 스폰 거리와 짝지어 생각할 것.
- `rate_limit`/`lpf_alpha`는 **Layer 2 스무스니스 보상**(`w_ang_vel`, `w_action_smooth`, §2)과
  세트로 작동한다. 로직(LPF)만 조이고 보상 가중치를 그대로 두면 진동은 줄어도 정책이
  "왜 느린지" 학습 신호를 못 받는다 — Rule 15의 결론(로직+보상 **병행**)이 여기도 적용됨.
- `lpf_tick_period_steps`를 바꾸는 것은 **타이밍 parity를 깨는 것**이다 — Rule 16 위반
  위험. 정말 필요하면 `notes/research/isaac_lab_architecture.md`의 "데이터 흐름" 절 전체를
  재검토하고 문서화할 것.

---

## 2. 보상 함수 — `DroneBombardRewardCfg` (`math_utils.compute_reward`)

Gazebo 4-layer 보상([[research/reward_design]])과 동일 계열의 3-layer 구조
(Layer 1 종단 페널티는 `_get_rewards`에서 `_done_flags` 기반으로 별도 가산).

$$R = \underbrace{-w_{time} - w_{\omega}\|\omega\|^2 - w_{\Delta a}\|\Delta a\|^2}_{\text{Layer 2 (안정성)}}
+ \underbrace{w_{dist}(d_{prev}-d_{xy}) + w_{heading}\cos\theta \cdot g + w_{prox}\max(0,1-\tfrac{d_{xy}}{R_{prox}}) + w_{vis}(\ldots) - w_{vel}\|v_{xy}\|\max(0,1-\tfrac{d_{xy}}{R_{vel}})}_{\text{Layer 3 (접근)}}
+ \underbrace{\text{success/crash/...}}_{\text{Layer 1 (종단, 아래 §4)}}$$

| 필드 | 기본값 | 의미 | 코드 |
|---|---|---|---|
| `w_dist` | 2.0 | 스텝당 거리 개선 보상 (선형, Rule 4의 지수 포텐셜 함정 없음 — 이미 선형) | `r3_dist` |
| `w_heading` | **0.0 (비활성)** | 진행 방향-표적 정렬 보상. v13/v15에서 근접 시 기하 노이즈로 꺼둠 | `r3_orient` |
| `speed_gate_enabled` | True | `w_heading≠0`으로 켤 때만 의미 있음 — 저속(호버) 시 방향 보상 억제(anti-milking, [[research/reward_design]] 참조) | `speed_gate` |
| `w_proximity` | 0.6 | 근접 반경 내 밀집 보상 | `r3_proximity` |
| `proximity_radius` | 2.0 m | 위 보상이 적용되는 반경 | — |
| `w_vision_center` | 1.5 | 표적을 화면 중앙에 두는 보상(비전 서보잉 신호) | `r3_vision` |
| `w_time` | 0.05 | 스텝당 시간 페널티(긴급성) | `r2` |
| `w_ang_vel` | 0.15 | 각속도 페널티(스무스니스) — Rule 15 교정값(0.05→0.15) | `r2` |
| `w_action_smooth` | 0.20 | 액션 변화량 페널티(스무스니스) — Rule 15 교정값(0.05→0.20) | `r2` |
| `w_vel` | 0.08 | **근접-게이팅 속도 댐핑**(Rule 15의 "B" 교정) — 표적에서 멀면 0, 가까울수록 감속 유인 | `r3_vel` |
| `vel_damp_radius` | 3.0 m | 위 댐핑이 켜지는 반경 (Rule 15 원본은 `R=4`; Isaac 이식값은 3.0 — parity 표에서 재확인) | — |
| `success_radius` | 0.8 m | 성공 판정 반경. **주석에 커리큘럼 계획 명시**: 학습 안정화 후 0.5로 조여라 (Rule 10 참조) | `_get_dones` |

**튜닝 우선순위 (Rule 15 인용):** wobble/과속 계열 문제는 **보상 shaping 먼저**
(`w_vel`, `w_ang_vel`, `w_action_smooth`) → 그래도 부족하면 액션 스케일↓(§1) → 그래도
구조적이면 이전 액션을 obs에 추가(코드 변경 필요, 현재 없음).

### 종단(Layer 1) 페널티/보상 — 항상 `DroneBombardRewardCfg`에 같이 있음

| 필드 | 기본값 | 발동 조건 (§4 termination과 연결) |
|---|---|---|
| `reward_success` | +100.0 | `d_xy <= success_radius` |
| `penalty_crash` | −50.0 | 고도 < `ground_contact_altitude` 또는 스텝>1에서 < `min_altitude` |
| `penalty_overspeed` | −30.0 | 속력 > `v_max_safety` |
| `penalty_bad_attitude` | −30.0 | 각속도 초과 또는 전복(§4) |
| `penalty_out_of_range` | −30.0 | `d_xy > max_distance` |
| `penalty_max_altitude` | −30.0 | 고도 > `max_altitude` |
| `penalty_overshoot` | −10.0 | overshoot guard (§4, Rule 10) |
| `penalty_stagnation` | −15.0 | stagnation guard (§4) |
| `truncation_penalty` | −15.0 | 타임아웃(300 스텝) 이면서 다른 종단 아님 |
| `penalty_target_lost` | 0.0 | conf=0 스텝 — v13/v15에서도 0, parity용으로만 유지 |

⚠️ **보상 공식(가중치·구조) 변경 후에는 fresh start를 권장한다.** PPO는 replay buffer가
없어 Rule 4의 "Q-value 오염" 메커니즘 자체는 없지만, 진행 중인 run의 value function이
구 보상 스케일에 맞춰져 있어 advantage 추정이 한동안 왜곡된다 — 특히 가중치를 크게
바꿨다면(예: `w_dist` 2.0→5.0) 새 `run_name`으로 fresh 시작해 비교를 깨끗하게 유지할 것.
`success_radius`를 **커리큘럼으로 조이는 것**(0.8→0.5, 코드 주석의 의도된 사용법)은
예외 — 기존 체크포인트에서 `--resume`하는 것이 정상 워크플로우다.

---

## 3. 리셋 / 스폰 — `DroneBombardResetCfg` ("순항 고도"·"핸드오프 거리" 여기)

> **용어 주의:** Isaac 쪽엔 Gazebo의 스크립트 CRUISE 단계가 없다. "순항 고도"에 해당하는
> 개념은 **에피소드 시작 시 스폰 고도**다 — 아래 `spawn_alt_range`.

| 필드 | 기본값 | 의미 |
|---|---|---|
| `spawn_alt_range` | (9.0, 11.0) m | 에피소드 시작 고도 — Gazebo의 "순항 고도" 역할. env마다 균일분포 랜덤화 |
| `handoff_dist_range` | (3.0, 7.0) m | 스폰 시 표적까지 수평거리 — Gazebo의 "핸드오프 거리"(CRUISE→TRACKING) 역할. Rule 13("고도는 레버 아님, 탐지 게이트가 레버")의 교훈은 Isaac 비전이 analytic pinhole이라 다르게 적용됨(§ [[research/isaac_lab_architecture]] Vision 행 참조) |
| `yaw_range` | (−π, π) | 스폰 요 각도 |
| `init_vel_std` | 0.3 m/s | 스폰 초기 속도 노이즈(가우시안) |
| `target_xy_range` | 10.0 m | 표적 위치 랜덤화 범위(env 원점 기준 ±) — Gazebo는 고정 표적(11,10)이었음, Isaac은 매 에피소드 랜덤화가 **의도된 차이** |

**핸드오프 거리(`handoff_dist_range`)를 넓히기 전에 확인할 것:** §1의 액션 스케일과
§4의 `overshoot_close_threshold`가 이 범위 **안쪽**에서 무장되지 않는지 (Rule 10 — 무장
반경이 핸드오프 최소거리보다 크면 매 에피소드 종단 트랩에 걸린다). `handoff_dist_range[0]`
을 `overshoot_close_threshold`보다 작게 내리지 말 것.

**스폰 고도(`spawn_alt_range`)를 바꾸면:** 비전 프로젝션(`math_utils.project_target_pinhole`)
의 apparent-size 관계상 고도가 높을수록 같은 픽셀 노이즈가 상대적으로 더 큰 각도 오차가
된다 — Gazebo Rule 13의 "고도↑=가시성↓" 직관은 analytic vision에도 정성적으로 남아있다.
고도 범위를 크게 벌리기 전 `vision.pixel_noise_std`(§5)와 같이 검토할 것.

---

## 4. 종단 조건 (guard) — `DroneBombardTerminationCfg`

| 필드 | 기본값 | 의미 |
|---|---|---|
| `ground_contact_altitude` | 0.5 m | 이 아래면 즉시 crash |
| `min_altitude` | 3.0 m | 스텝 `min_altitude_start_step` 이후 이 아래면 crash |
| `min_altitude_start_step` | 1 | 위 가드가 무장되는 스텝(스폰 직후 여유) |
| `v_max_safety` | 20.0 m/s | 이 초과 시 overspeed |
| `limit_ang_vel` | 2.0 rad/s | 각속도 노름 초과 시 bad_attitude |
| `limit_inverted_tilt` | 1.047 rad (~60°) | roll/pitch 초과 시 bad_attitude(전복) |
| `max_distance` | 100.0 m | 표적과 이 이상 멀어지면 out_of_range |
| `max_altitude` | 25.0 m | 이 이상 고도는 max_altitude 종단 |
| `overshoot_close_threshold` | 0.6 m | **overshoot guard 무장 반경** — §3 핸드오프 최소거리보다 반드시 작아야 함(Rule 10) |
| `overshoot_margin` | 2.0 m | 최근접점 대비 이만큼 멀어지면 overshoot 발동 |
| `stagnation_window` | 150 스텝 | 이 구간 동안 |
| `stagnation_min_progress` | 1.0 m | 이만큼도 접근 못하면 stagnation 발동 |
| `stagnation_start_step` | 50 | stagnation 가드가 무장되는 최소 스텝 |
| `overshoot_flythrough_radius` | 1.2 m | **비종단** 진단용(§ WandB `Episode_Diag/overshoot_flythrough`) — 종료/보상에 영향 없음 |

`success_radius`(0.8, §2)를 커리큘럼으로 좁힐 때는 `overshoot_close_threshold`(0.6)와의
관계를 재확인할 것 — 0.6보다 작게 좁히면(예: 0.5) overshoot guard가 성공 반경보다 커져서
Rule 10의 트랩이 재발한다.

---

## 5. 비전 / 컨트롤러 / 애셋 — 자주 안 건드리지만 알아둘 것

- **`DroneBombardVisionCfg`**: `pixel_noise_std`(3.0px), `conf_lo/hi`(0.73/0.95), `dropout_prob`
  (0.05), `hold_frames`(10 @ 10Hz = 1s) — YOLO 캘리브레이션 값 이식. 실제 YOLO 대비
  재검정은 `yolo_eval.py --calibrate`로 (아직 L4 VM 미실행).
- **`DroneBombardControllerCfg`**: PX4 게인 근사값 — **미검정**([[research/isaac_velocity_controller]]
  참조). 보상/커리큘럼 튜닝과 이 게인을 **동시에** 바꾸지 말 것 — 어느 쪽이 행동 변화를
  일으켰는지 구분 불가능해진다.
- **`DroneBombardAssetCfg`**: 드론/페이로드 질량·관성(SDF 실측값) — 물리적 사실이므로
  튜닝 대상 아님. 바꾸려면 반드시 `notes/`에 실측 근거를 남길 것.

---

## 관련 링크

- [[research/isaac_lab_architecture]] — 전체 구조, 데이터 흐름, Gazebo 대비 구조 차이
- [[research/isaac_lab_wandb_guide]] — 위 파라미터들이 WandB의 어느 메트릭에 나타나는지
- [[research/isaac_lab_experiment_workflow]] — dry-run → fresh/resume → 실험 로깅 절차
- [[research/isaac_velocity_controller]] — 컨트롤러 게인 근거/검정 상태
- [[research/rl_rules]] — Gazebo 트랙 규칙(Rule 4/10/15/16이 여기서도 그대로 적용)
- [[experiments/exp_012_isaac_migration_phase2]] — 전체 parity 표(v13 상수 → 이 문서의 필드)

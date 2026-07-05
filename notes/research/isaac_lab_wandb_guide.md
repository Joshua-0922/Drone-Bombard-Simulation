---
date: 2026-07-03
tags: [research, isaac-lab, wandb, rsl_rl, ppo, metrics, onboarding]
status: active
type: research
---

# Isaac Lab WandB 메트릭 가이드 (`feat/isaac-env-migration`)

> **대상 독자:** Isaac Lab/PPO(rsl_rl) 트랙에서 학습 결과를 WandB로 판단할 연구자.
> Gazebo/SAC 트랙의 [[research/rl_rules]] Rule 3/6/9 (`env/mean_d_xy`, `env/success_rate` 등)
> 와는 **완전히 다른 메트릭 네임스페이스**다 — 두 브랜치 대시보드를 섞어 보지 말 것.
>
> ✅ **부분 검증됨 (2026-07-03, 커밋 `5a6a71b`):** 256-env·20-iteration dry-run이 이
> 박스에서 실제로 돌아 **WandB 프로젝트 `drone-bombard-isaac`에 로깅 확인**됐다 — reward
> −74.98→+28.65, ep_len 2.2→~100, `Episode_Metric/d_xy_min` 0.747(성공 반경 도달)까지
> 관찰됨. 아래 §2-4(`Episode_*`)는 이 dry-run으로 **키 이름·동작 모두 실증**됐다. §5(rsl_rl
> 표준 `Train/*`·`Loss/*`)는 여전히 관례 기준 추정 — 그 dry-run의 실제 wandb run(프로젝트
> `drone-bombard-isaac`, 커밋 시점 기준 가장 이른 run)을 열어 정확한 스펠링을 대조하고 이
> 문서를 갱신할 것. **2048-env 본 학습(3000 iteration) 규모의 곡선은 아직 없다** — 위 숫자는
> 20 iteration짜리 dry-run 기준이니 절대값을 본 학습 기준치로 쓰지 말 것(경향 확인용).

---

## 1. 두 개의 메트릭 그룹

| 그룹 | 소스 | 확실성 |
|---|---|---|
| `Episode_Termination/*`, `Episode_Reward/*`, `Episode_Metric/*`, `Episode_Diag/*` | `drone_bombard_env.py`의 `self.extras["log"][...]` (§§2-4) | ✅ 코드 리터럴 + 2026-07-03 256-env dry-run으로 실증 |
| `Train/*`, `Loss/*`, `Policy/*`, `Perf/*` | rsl_rl `OnPolicyRunner` 표준 로깅(§5) | ⚠️ rsl_rl 버전 관례 — 위 dry-run의 실제 wandb run에서 정확한 키 이름 대조 필요(아직 미대조) |

---

## 2. 종단 원인 분포 — `Episode_Termination/*`

`_log_reset_extras()`가 리셋되는 env들의 종단 플래그 평균을 매 리셋마다 기록한다
(0~1 사이 비율로 해석 — 예: `success=0.3`은 방금 리셋된 env 중 30%가 성공 종료).

| 키 | 의미 | 건강한 추세 |
|---|---|---|
| `Episode_Termination/success` | `d_xy <= success_radius(0.8)` | **상승**, 목표 > 0.7~0.8 |
| `Episode_Termination/crash` | 저고도/지면 접촉 | 낮게 유지, 초반엔 자연히 높을 수 있음 |
| `Episode_Termination/overspeed` | 속력 > 20 m/s | 0에 가까워야(액션 스케일이 4/3/3이라 이론상 도달 어려움 — 0이 아니면 컨트롤러/게인 버그 의심) |
| `Episode_Termination/bad_attitude` | 각속도 초과 또는 전복 | 학습 초반 탐험 노이즈로 존재 가능, 정책 수렴 후 감소해야 함. **컨트롤러 게인 미검정**([[research/isaac_velocity_controller]])이므로 초반에 비정상적으로 높으면 게인 문제부터 의심 |
| `Episode_Termination/out_of_range` | `d_xy > 100m` | 0에 가까워야 — 스폰 거리가 3-7m 랜덤이라 정상 정책이면 거의 발생 안 함 |
| `Episode_Termination/max_altitude` | 고도 > 25m | 0에 가까워야 |
| `Episode_Termination/overshoot` | Rule 10 트랩 발동 | **지속적으로 높으면(예: >0.3)** → [[research/isaac_lab_reward_tuning]] §4의 `overshoot_close_threshold` vs `handoff_dist_range` 관계부터 점검 |
| `Episode_Termination/stagnation` | 150스텝 동안 1m 미접근 | 학습 후반까지 높으면 접근 유인(`w_dist`, `w_proximity`) 부족 의심 |
| `Episode_Termination/timeout` | 300스텝(30s) 소진 | success/stagnation/overshoot로 흡수되지 않은 나머지 |
| `Episode_Termination/timeout_near_miss` | 타임아웃인데 `final_d_xy < 2m` | 높으면 "거의 다 왔는데 못 끝맺음" — `success_radius`가 너무 타이트하거나 종단 근처 속도 제어(§`w_vel`)가 과함 신호 |

Gazebo Rule 9(`ep_len` 붕괴로 인한 `ep_rew_mean` 착시)의 Isaac 대응: PPO는 스텝별
`reward`를 iteration 단위로 평균하는 rsl_rl 표준 `Train/mean_reward`(§5)를 보므로
같은 착시가 원리상 발생할 수 있다 — **에피소드가 빨리 끝나는 것(success 상승)과
`ep_len` 감소를 항상 같이 봐서** 개선인지 붕괴인지 구분할 것.

---

## 3. 보상 성분 분해 — `Episode_Reward/*`

`_episode_sums`에 스텝마다 누적된 값의 **에피소드 합**(평균 아님 — [[research/isaac_lab_reward_tuning]]
§2의 가중치가 클수록 이 합도 커진다)이 리셋마다 로깅된다.

| 키 | 대응 보상 항 | 정상 신호 |
|---|---|---|
| `Episode_Reward/rew_dist` | `w_dist*(d_prev-d_xy)` 누적 | 양수, 접근할수록 큼 |
| `Episode_Reward/rew_orient` | 방향 정렬(`w_heading=0`이라 **항상 0** — 값이 0이 아니면 cfg가 v13/v15 기본값에서 바뀐 것) | 0 (기본 설정 기준) |
| `Episode_Reward/rew_proximity` | 근접 보상 | 양수, 표적 근처에서 큼 |
| `Episode_Reward/rew_vision` | 비전 중앙 정렬 보상 | 양수, conf>0인 스텝에서만 기여 |
| `Episode_Reward/rew_vel` | 근접-게이팅 속도 댐핑(음수) | 음수, 표적 근처에서 커짐(절댓값) — Rule 15 |
| `Episode_Reward/rew_ctrl` | `-w_time-w_ang_vel*ω²-w_action_smooth*Δa²` | 항상 음수, 진동이 심할수록 더 음수 |

**`rew_ctrl`이 비정상적으로 음수로 크면** wobble/난폭한 정책 신호 — [[research/control_smoothness_wobble]]
(Rule 15)와 동일한 진단 절차(단, Isaac은 PX4 로그가 아니라 `Episode_Reward/rew_ctrl`
자체 + `Episode_Termination/bad_attitude`로 판단)를 적용할 것.

---

## 4. 물리/투하 지표 — `Episode_Metric/*`, `Episode_Diag/*`

| 키 | 의미 |
|---|---|
| `Episode_Metric/drop_impact_error_m` | 종료 시점 상태로 계산한 CCIP 탄도 예측 오차(analytic, `ballistic_impact`) — Gazebo Rule 12에서 지적된 "CEP 비실재" 문제가 Isaac에선 **해결됨**: 실제 조인트 투하가 아니라 스크립트 CCIP 메트릭이지만 항상 유효한 값을 emit한다 |
| `Episode_Metric/d_xy_min` | 에피소드 중 최근접 거리의 평균 — success_rate가 낮아도 이 값이 꾸준히 줄면 "잘 접근은 하는데 못 끝맺음"(정책 미성숙) 신호, Gazebo Rule 9 패턴과 유사 |
| `Episode_Diag/overshoot_flythrough` | **비종단** 진단(반경 1.2m 기준) — `Episode_Termination/overshoot`(반경 0.6m, 종단)보다 항상 크거나 같아야 정상. 이 값만 높고 종단 overshoot는 낮으면 "성공 반경 근처를 스치듯 지나가지만 아직 트랩엔 안 걸림" — 커리큘럼으로 `success_radius`를 조일 준비가 됐다는 신호일 수 있음 |
| `Episode_Metric/action_sat_frac` (07-05 추가) | **클립 전 raw 정책 액션** 성분 중 \|a\|>1 비율의 에피소드 평균 — 정책 평균의 rail-riding(포화)을 유일하게 직접 보는 지표 (exp_013 사후 계측: σ=0에서 77%). 지속 >0.5면 bang-bang 평균 — Rule 18(b) 부수발견 참조. 학습 중 추세 감시용 |

---

## 5. rsl_rl PPO 표준 학습 진단 (✅ 콘솔 로그 명칭은 exp_013에서 실증 — rsl_rl 3.1.2)

`OnPolicyRunner`가 매 iteration 콘솔에 찍는 블록 기준 명칭 (wandb 차트 키는 대응
네임스페이스로 매핑됨 — run `wcjklw7a` 대시보드 참조):

| 콘솔 명칭 (실증) | 의미 | 건강한 신호 |
|---|---|---|
| `Mean reward` | iteration당 평균 스텝 보상 | 상승 추세 (탐험 노이즈 포함 — Gazebo Rule 3의 `rollout/ep_rew_mean` 단독사용 금지와 동일 주의) |
| `Mean episode length` | 평균 에피소드 길이 | success 비율과 같이 해석(§2) |
| `Mean value_function loss` | critic loss | 발산하지 않고 감소/평탄화 (exp_013: 노이즈 폭주 구간에서 ~590 고착 — 노이즈 리턴 적합의 신호) |
| `Mean surrogate loss` | PPO 정책 loss | 0 근처 진동이 정상 |
| **`Mean action noise std`** | 정책 가우시안 σ | **⚠️ 최우선 감시 항목 (Rule 18b).** init 0.8에서 서서히 감소가 건강. **단조 상승(exp_013: 0.8→3.92)이면 entropy가 스무딩 파이프라인에 견제받지 않고 폭주 중** — 중반까지 init의 ~1.5× 넘으면 개입(entropy_coef ↓). 이 값이 크면 §2-3의 rollout 통계 전체가 포화-랜덤 액션 산물이므로 신뢰 금지, deterministic eval로 판정 |
| `Computation: N steps/s` | 처리량 | 2048 envs 기준 ~29K steps/s (L4 단독). GPU-vectorized라 Gazebo류 fps 병목(Rule 7/14) 없음 |

`schedule="adaptive"`, `desired_kl=0.01`(agents/rsl_rl_ppo_cfg.py)이므로 learning rate가
자동 조절된다 — KL이 지속적으로 목표치를 벗어나면(로그에 `Loss/learning_rate` 변동 폭이
크면) 보상 스케일이 급격히 바뀐 직후(§2 참조)일 가능성이 높다.

### 참고 — 실제로 관찰된 건강한 dry-run 곡선 (256 envs, 20 iters, [[experiments/exp_012_isaac_migration_phase2]] §6d)

절대값은 20-iteration짜리 소규모 dry-run 기준이라 본 학습(2048 envs, 3000 iterations)
기준치로 쓰면 안 되지만, **형태**(reward가 음수에서 시작해 부드럽게 상승, ep_len이 초반
붕괴 후 회복)는 정상 학습의 참고가 된다:

| iter | Mean reward | Mean ep_len |
|---|---|---|
| 0 | −74.98 | 2.23 (즉시 crash) |
| 3 | −24.36 | 45.66 |
| 8 | −2.95 | 109.66 |
| 13 | +18.63 | 108.59 |
| 19 | +28.65 | 96.92 |

19 iteration 시점 `Episode_Metric/d_xy_min=0.747`(< `success_radius=0.8`), 보상 분해는
`Episode_Reward/rew_dist≈+10.9`, `rew_proximity≈+4.6`, `rew_ctrl≈−19.2` — 부호가 §3 표와
일치하면 보상 구현이 의도대로 작동 중이라는 뜻.

---

## 6. 두 브랜치 메트릭 대조표 (헷갈림 방지용)

| 개념 | Gazebo/SAC (`jekyun`) | Isaac Lab/PPO (`feat/isaac-env-migration`) |
|---|---|---|
| 접근 거리 | `env/mean_d_xy` | `Episode_Metric/d_xy_min` (근접 최솟값 평균 — 정의가 다름, 직접 비교 금지) |
| 성공률 | `env/success_rate` (스칼라) | `Episode_Termination/success` (리셋 배치 평균 비율) |
| 착탄 오차 | `env/drop_error_actual_m` (Rule 12: v13은 NaN이었음) | `Episode_Metric/drop_impact_error_m` (analytic, 항상 유효) |
| 물리 글리치 | `env/physics_glitch_count` (Gazebo ODE 폭발) | 해당 없음 — Isaac은 PhysX, teleport 리셋이라 ODE 폭발 클래스의 문제 자체가 없음 |

---

## 관련 링크

- [[research/isaac_lab_reward_tuning]] — 여기 메트릭들이 어느 cfg 필드에서 나오는지
- [[research/isaac_lab_experiment_workflow]] — run 시작·비교 절차
- [[research/isaac_lab_architecture]] — 데이터 흐름 전체 그림
- [[research/rl_rules]] Rule 3/6/9 — Gazebo 트랙의 대응 규칙(원리는 재사용, 키 이름은 별도)
- [[experiments/exp_012_isaac_migration_phase2]] — 이식 배경, L4 VM 학습 미실행 현황

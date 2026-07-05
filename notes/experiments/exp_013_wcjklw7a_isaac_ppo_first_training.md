---
date: 2026-07-03
tags: [experiment, isaac-lab, ppo, rsl_rl, wandb, vision]
status: complete
type: experiment
wandb_run: wcjklw7a (exp013_v2_visionfix) / 이전 시도 exp013_v1_baseline (vision-dead, iter 106에서 중단)
---

# exp_013 — Isaac Lab 첫 프로덕션 PPO 학습 (2048 envs, 1000 iters)

> **목적:** Isaac Lab 트랙 첫 본 학습. 수렴 여부·d_xy 추세 판정 + 튜닝 대상 도출.
> **환경:** dev 박스 L4(driver 580), `isaac-verify` 컨테이너, GPU 단독 사용(Gazebo SAC 정지 상태).
> **설정:** 코드 기본값 그대로 (v13/v15 parity 상수, [[research/isaac_lab_reward_tuning]]).
> PPO: rsl_rl 3.1.2, `agents/rsl_rl_ppo_cfg.py` 기본값 (lr 3e-4 adaptive/KL 0.01, entropy 0.005,
> init_noise_std 0.8, [256,256] ELU). seed 42. 처리량 ~29K steps/s, 1000 iters ≈ 43분.

---

## 1. v1 시도 — 비전 사멸 버그 발견으로 중단 (iter 106)

첫 launch(exp013_v1_baseline) iter 20에서 `Episode_Reward/rew_vision` **정확히 0.0000** 발견
→ 코드 추적으로 `_update_vision`의 **env-origin 프레임 혼용 버그** 확정 (world `root_pos_w`
vs env-local `_target_xy` → 2048-env grid에서 타겟이 항상 프레임 밖 투영 → conf≡0).
SIGTERM 중단(preempt 저장) 후 수정·재기동. 수치 재현/수정 검증 포함 상세:
[[errors/err_20260703_vision_env_origin_frame]]. **`yolo_eval.py`의 동일 버그도 수정.**

부수 확인: 이전 256-env dry-run(exp_012 §6d)의 d_xy_min 0.747도 비전 죽은 채
privileged rel_dx/dy만으로 달성된 것이었음 — parity 메트릭 "로깅 확인"과 "채널 활성
확인"은 다르다.

## 2. v2 (vision fix) 학습 곡선 — wandb `wcjklw7a`

| iter | reward | ep_len | success | d_xy_min | crash | max_alt | noise_std |
|---|---|---|---|---|---|---|---|
| 100 | −54.5 | 126 | 0.12 | 3.74 | 0.53 | — | 1.08 |
| 300 | −31.8 | 87 | 0.23 | 2.48 | 0.23 | — | 1.47 |
| 500 | +2.9 | 59 | 0.36 | 1.69 | 0.32 | — | 1.81 |
| 700 | +13.9 | 58 | 0.43 | 1.18 | 0.37 | — | 2.39 |
| 994 | +7.3 | 59 | 0.31 | 1.45 | 0.39 | — | **3.92** |

- 50-iter 스무딩 success: iter 350까지 ~14% 정체 → 700에서 39% 도달 → **이후 평탄**(마지막
  300 iters 0.33-0.37 오실레이션). last200 vs prev200: reward −3.6, success −0.03 → **plateau 확정.**
- 마지막 100 iters 종단 분포: success ~36% / crash ~39% / **max_altitude ~27%** /
  bad_attitude 0.9% / 나머지 ~0. stagnation은 iter 300 이후 0.
- `rew_vision` 에피소드합 7-17로 학습 내내 활성(v1의 0과 대조) — 수정 유효.
- `rew_dist` 에피소드합 **−14**(음수): 롤아웃 평균으로는 시작점보다 먼 곳에서 종료.
- **`noise_std` 0.8→3.92 단조 폭주** — §4 참조. 학습 통계 전체가 포화-랜덤 액션 오염.

## 3. Deterministic eval (model_final.pt, 32 envs, 200 episodes)

```
success_rate = 36.00% (72/200)
종단 분포: success 72 | max_altitude 66 (33%) | crash 54 (27%) | overspeed 6 | bad_attitude 1 | out_of_range 1
mean d_xy_min = 1.433 m (게이트 0.8m 밖) | timeout/stagnation = 0
mean drop_impact_error = 24.0 m (실패 지배 구간이라 무의미)
```

**deterministic도 36%** — rollout 통계(36%)와 동일. plateau는 탐험 노이즈 착시가 아니라
**정책 평균 자체의 한계**다. 정책은 호버링하지 않는다(timeout/stagnation 0) — 세 attractor로
갈라진다: 완주(36%) / **25m 천장까지 상승 후 종료(33%)** / 지면 다이브 crash(27%).

> 참고: eval 하니스는 이 세션에서 수정된 `play.py::run_policy` 사용 — 기존 코드는
> post-reset `_current_d_xy()`를 "final d_xy"로 읽는 버그(리셋 후 새 스폰 거리를 보고)가
> 있어 종단 원인 분포+`extras["log"]` 스냅샷 기반으로 교체함.

## 4. 진단 — 세 겹의 문제 + 속도킥의 정확한 정체(§4d, 07-04 forensics로 재정정)

> **§4d 판정 이력:** 07-03 사후 검증(`--zero-actions` FAIL 11.9m)에서 "리셋 속도킥이 run
> 전체를 오염, max_alt 1차 용의자"로 판정했으나, **07-04 계측 forensics(`_diag_kick.py`
> 매트릭스)가 이를 재정정**: 킥은 매 리셋이 아니라 **프로세스당 1회**(첫 물리 substep)라
> 학습 오염이 사실상 없고, max_alt 27-43%는 iter ~200에서 창발한 **학습된 행동**이다.
> 4a(비전 farming attractor)가 1차 가설로 복권됨. 상세는 §4d.

### 4a. Analytic vision 보상의 고도-상승 attractor (max_alt 33%의 원인)

analytic conf(0.73-0.95)는 **거리에 따라 감쇠하지 않는다**(edge falloff만 존재). 그런데
centering 기하는 고도가 높을수록 유리(`u_n ∝ x_c/z_c` — 같은 수평 오프셋도 고도↑면
각도↓ → center_dist↓). 즉 **상승하면서 타겟을 밑에 두면 r3_vision(최대 ~1.5/step)이
더 쉽게 벌린다** → 25m 천장(−30)까지 올라가는 것이 국소 최적. 실제 YOLO는 apparent
size ∝ 1/거리로 conf가 깎이므로(Rule 13) 이 attractor는 **analytic 모델의 parity 누락**이
만든 인공물이다.

### 4b. Farmer-vs-finisher 보상 불균형 (완주 인센티브 부족)

현재 상수로 수지 계산: 5m→0.8m 완주(~21 steps) 리턴 ≈ **+121** vs 1m 근처에서 비전+근접
스트림 farming(~1.47/step, stagnation −15까지 170 steps) ≈ **+225**. γ=0.995 할인해도
farmer 우세. **완주가 지배 전략이 아니다.** Gazebo v14 195K eval의 "final-approach
stagnation(0.5-0.8m)" 실패 7건과 같은 서명 — SAC에서는 증상으로 나타났고 PPO는 이
불균형을 더 체계적으로 착취한다.

### 4c. noise_std 폭주 (0.8→3.92 단조)

entropy bonus(0.005)는 σ를 키우는 압력인데, 이 태스크의 액션 파이프라인(clip →
rate_limit ±0.2 → LPF 0.4)이 **의도적으로**(v15 smoothness 이식) 고주파 노이즈를
필터링하므로 σ가 커져도 task 손실이 작다 → entropy 이득이 이겨 σ 무한 성장.
**07-04 계측(`_diag_noise.py`, σ∈{0,0.8,2.0,3.9} × 300 policy steps)으로 정밀화:** 노이즈는
액션 레벨에선 살아남지만(executed Δaction 3.66× vs det, rate-limiter 68% 포화, 부호 반전
29%) **실행 속도 궤적은 σ-불변**(velocity-Δ σ3.9/det = 1.01×) — LPF+accel clamp가 plant
레벨에서 균질화. 따라서 폐해는 "rollout이 랜덤 액션으로 오염"이 아니라(행동은 거의 동일;
det eval==rollout 36%가 그 증거) **log-prob gradient 노이즈 + 탐험 이득 없는 목적함수
왜곡**이다. 부수 발견: 정책 평균 자체가 클립 포화(σ=0에서 raw|a|=2.6, 77% 성분 |a|>1) —
가우시안 무-squash 정책이 레일-라이딩 평균을 학습(Rule 15 동족). Gazebo SAC(ent_coef
자동조정)에는 없던 **PPO×스무딩-파이프라인 조합의 신규 실패 모드.** → Rule 18b 갱신.

### 4d. 속도킥의 정확한 정체 — 07-04 계측 forensics (`_diag_kick.py` 매트릭스)

07-03의 `--zero-actions` FAIL(11.9m)로 "매 리셋 킥 → run 전체 오염"으로 판정했었으나,
per-substep 계측 매트릭스(모드별 로그 `/workspace/logs/_diag_*.log`)가 정정했다:

**확정된 메커니즘:** PhysX solver는 `__init__`의 `root_physx_view.set_masses()`(0.028→2.173kg)를
**첫 sim step까지 소비하지 않는다.** 그 한 substep 동안 컨트롤러의 hover-사이즈 wrench
(21.319N, 2.173kg 기준 중력보상)가 **stale native 질량에 적분**된다:
관측 dvz=+8.417 m/s → `m_eff = F·dt/(dvz+g·dt) = 21.319×0.01/(8.417+0.0981) = 0.02504 kg`
= **native body link 질량 0.025와 정확 일치.** 다음 substep부터 질량 영구 동기화
(킥 직후 회복률 dvz=−0.040/substep = 2.173kg 기준 −4 m/s² accel clamp와 정확 일치).

**격리 실험 결과:**
| 모드 | 결과 |
|---|---|
| combo t1m/t1a (manual 리셋 ×2 + auto 리셋 ×2) | **킥은 프로세스 첫 substep 1회뿐** — 이후 모든 리셋 dvz≡0 (clean) |
| t2 (리셋 시 wrench 제로) | 무력 자유낙하 dvz=−0.0981=−g·dt 정확 (중력은 가속도 적용 = 질량 무관 → **우리 힘이 있어야만 킥**) |
| t2first (첫 substep을 zero-wrench로 통과) | **킥 전무** + ssr=10에 추력 재개해도 무점프 (zero-force step이 질량 flush → 최소 수정 경로 실증) |
| t4m (set_masses만) | 킥 동일 재현 (dvz=+8.417, m_eff=0.02504) — **set_masses 단독 충분** |
| t4i (set_inertias만) | 무점프 — inertia 콜은 킥과 무관 (전파 자체를 안 함, 기존 문서화대로) |
| t6 (리셋 직후 속도 재기록) | 무점프 |
| noreset | 스크립트 한계로 크래시(reset 없이 step → `_last_vision` 미설정) — 질문 자체는 combo+t2first가 대체 답변 |

**07-03 판정의 오류 원인:** 메모리의 "매 리셋" 관찰은 매 관측이 fresh 프로세스였던 탓
("첫 리셋"과 "프로세스 시작"이 항상 겹침). `--zero-actions`가 항상 FAIL한 것도 같은 이유
— 단일 킥(+8.4 m/s, −0.04/substep 감쇠, ~11-12m 상승)이 드리프트 11.9m를 정량 완전 설명.

**exp_013 재귀속:**
- **학습 오염 = 사실상 없음.** 킥은 2048 envs의 첫 에피소드(전체 ~60만 에피소드 중
  ~0.3%)에만 영향. **max_alt 27-43%는 킥일 수 없다** — 결정적으로, 학습 커브에서
  max_altitude 종단은 **iter 0-199에 ~0%였다가 iter ~200에서 창발**(0.33→0.43)해 이후
  지속된다. 프로세스 시작 1회성 transient는 창발-지속 패턴을 만들 수 없다 → **학습된
  행동** = 4a(비전 farming attractor) 1차 가설 복권. (같은 구간 `rew_vision`이 높게 유지되는
  것도 정합적.)
- eval(200ep)에서는 프로세스 첫 substep 킥이 32 envs의 첫 에피소드에만 영향(≤32/200) —
  max_alt 66 중 최소 34는 킥과 무관.
- 36%는 "오염 plant 수치"가 아니라 **유효한 수치**로 복권 (첫-에피소드 아티팩트 소폭 제외).

**교훈 (07-03 교훈의 교정판):** 알려진 이슈의 재검증은 여전히 필수지만, **증상 재현
(--zero-actions FAIL)과 원인 귀속(training 오염)은 별개다** — 귀속은 메커니즘 계측
(per-substep 로깅, m_eff 검산, 창발 타이밍)까지 가서야 확정된다. 07-03의 "run 전체 오염"
판정은 재현만 보고 귀속을 성급히 넘겨짚은 사례.

## 5. 결론 (07-04 재정정 반영)

- **수렴 판정: plateau 수렴(iter ~700), 목표 미달.** success 36%(deterministic 동일) —
  §4d 재정정으로 이 수치는 **유효**하다(킥 오염은 첫 에피소드 ~0.3%뿐). 실패 분해도 유효:
  max_alt ~33%는 학습된 attractor(4a 1차 가설), crash 27%는 원인 미확정 이월.
- **d_xy 추세: 초중반 건강, 종반 정체.** `Episode_Metric/d_xy_min` 4.1→1.18(iter 700)
  이후 1.4m 정체 — 게이트(0.8m) 밖. 접근 학습은 확실히 일어났다(비전 수정 유효 포함).
- **다음 실험(exp_014):** 4a-4c 교정(conf 거리감쇠 + reward_success 300 + entropy 0)이
  본체. 리셋 킥은 **위생 수정으로 동반**(스폰타임 mass_props 또는 init 직후 zero-wrench
  step 1회 — t2first 실증) — 더 이상 게이트가 아님. 상세: [[research/isaac_ppo_tuning_recommendations]].

## 관련

- [[errors/err_20260703_vision_env_origin_frame]] — v1 중단 사유
- [[research/isaac_ppo_tuning_recommendations]] — 튜닝 권고 (다음 실험 설계)
- [[research/isaac_lab_wandb_guide]] — 메트릭 해석 (이 run으로 §5 rsl_rl 키 실증)
- [[experiments/exp_012_isaac_migration_phase2]] — 선행 dry-run
- [[experiments/training_history]] — 실험 이력 허브

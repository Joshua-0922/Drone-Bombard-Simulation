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

## 4. 진단 — 세 겹의 문제

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
필터링하므로 σ가 커져도 task 손실이 작다 → entropy 이득이 이겨 σ 무한 성장. 결과:
학습 후반 rollout이 포화-랜덤 액션으로 수집되어 advantage 신호가 묻히고(§2의 rew_dist
−14, crash 39%가 그 산물), 학습이 plateau 위에서 진동만 한다. Gazebo SAC(ent_coef
자동조정)에는 없던 **PPO×스무딩-파이프라인 조합의 신규 실패 모드.**

## 5. 결론

- **수렴 판정: plateau 수렴(iter ~700), 목표 미달.** success 36%(deterministic 동일) —
  Gazebo v13 eval 80%(clean 3-ep) / v14 65% 대비 낮다. 단, 태스크가 다름(타겟·스폰 랜덤화).
- **d_xy 추세: 초중반 건강, 종반 정체.** `Episode_Metric/d_xy_min` 4.1→1.18(iter 700)
  이후 1.4m 부근 정체 — 게이트(0.8m) 밖. "접근은 배웠고 마무리를 안 배운" 상태이며,
  §4a/4b가 마무리를 배우지 않는 것이 합리적이도록 보상 지형을 만들고 있다.
- **변경 권고(보상·하이퍼파라미터·고도)는 [[research/isaac_ppo_tuning_recommendations]]**
  — 이 실험의 결론 문서. 요지: ① analytic conf에 거리 감쇠 추가(4a 제거, YOLO parity)
  ② reward_success 100→300(4b 역전) ③ entropy_coef 0.005→0.0(4c) ④ 스폰 고도/거리는
  현행 유지(v15 parity), success_radius 0.8 유지 후 커리큘럼.

## 관련

- [[errors/err_20260703_vision_env_origin_frame]] — v1 중단 사유
- [[research/isaac_ppo_tuning_recommendations]] — 튜닝 권고 (다음 실험 설계)
- [[research/isaac_lab_wandb_guide]] — 메트릭 해석 (이 run으로 §5 rsl_rl 키 실증)
- [[experiments/exp_012_isaac_migration_phase2]] — 선행 dry-run
- [[experiments/training_history]] — 실험 이력 허브

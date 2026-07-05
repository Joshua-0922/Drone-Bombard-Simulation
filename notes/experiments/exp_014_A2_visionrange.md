---
date: 2026-07-05
tags: [experiment, isaac-lab, ppo, vision, ablation, physics-fix]
status: done
type: experiment
wandb_run: v3qk07pg (A2), azoc1xp0 (A0')
---

# exp_014 — plant 수정(질량/inertia/로터) + A2 비전 거리감쇠

> [[research/exp014_ablation_protocol]]의 실행. 사용자 지시: 두 확정 버그의 기계적
> 수정 → `--zero-actions` 하드 게이트 → 3종 probe 교차검증 → 판정 게이트 →
> (수렴 시) A1 생략, A2 직행. **reward_success·entropy_coef는 이번 세션에서 불변.**

## 1. Plant 수정 (커밋 `cd0c617`, `7657df4`, `7d0e9b6`)

| 수정 | 내용 | 검증 |
|---|---|---|
| 속도킥 | 런타임 `set_masses()` → 스폰타임 `UsdPhysics.MassAPI` authoring (`_author_body_mass_props`) | `--zero-actions` **PASS** (0.18-0.22 m, 이전 11.9 m FAIL) |
| 로터 스핀 재주입 | `CRAZYFLIE_CFG` 기본 ±200 rad/s → `init_state` 오버라이드로 0 | 프롭스핀 A/B로 행동 영향 없음도 실증 |
| **(발견) inertia 불일치** | `set_inertias`는 **전파됐었음** → exp_013은 rate loop ~1300× 저토크. 수정: x500 I(0.0217) authoring + 컨트롤러 동일값 읽음 | `_diag_inertia.py` 계측, [[research/isaac_inertia_ctrl_mismatch]] |
| action_sat_frac 메트릭 | raw \|a\|>1 비율 per-episode 로깅 (`Episode_Metric/action_sat_frac`) | 다음 페이즈용 |

## 2. Probe 결과 (구 정책 exp013_v2 `model_final.pt`, deterministic 200 ep)

### Probe A — 수정 plant 재평가 (종단 분포)

| | 구 plant (exp_013) | 수정 plant (최종, x500 I) |
|---|---|---|
| success | 36% | 8.5% |
| bad_attitude | 0.5% | **69.0%** (중앙값 1.5 s, \|ω\| 한계) |
| max_altitude | 33% | **22.5% — 잔존** |
| crash | 27% | 0% |

- 킥 제거 후에도 climb 지속 → **물리 원인 직접 배제**.
- bad_attitude 폭증 = 구 정책의 plant-overfit ([[research/isaac_inertia_ctrl_mismatch]] §3).
  프롭스핀 A/B(65%≈68%)로 로터 수정 무관 실증 — inertia 일관성 변화 단독 기인.

### Probe B — max_alt 에피소드 z(t) 판독 (45 ep)

- **100% climb_from_spawn**, 중앙값: climb 지속 5.5 s, **vz_med = vz_max = 3.00 m/s
  (명령 가능 최대치 정확히 rail)** — 물리 스파이크가 아니라 **명령된 전략**.
  (loss-of-control 판정 13/45는 climb 중 \|ω\|>1.8 wobble 플래그 — vz 초과(>4.5)는 0건.)
- climb 중 타깃 탐지율 0.77, center_dist 중앙값 0.94 (프레임 가장자리 유지).

### Probe C — 보상 반사실 (거리감쇠 적용 재채점)

| 그룹 | 원 vision 수익/ep | 감쇠 적용 | 배율 |
|---|---|---|---|
| max_altitude (45 ep) | +9.6 | +1.5 | **×0.16** |
| — 그중 z>12 m 구간 | +7.9 | +0.9 | **×0.12** |
| success (17 ep) | +2.6 | +0.7 | ×0.27 |

### 교차검증 결론

세 probe **수렴**: (A) 킥 제거 후 climb 잔존 = 물리 배제, (B) climb은 정확한 명령
rail = 의도적 전략, (C) 거리감쇠가 climb 수익 84-88% 제거 = 유인 구조가 원인.
exp_013 학습 커브의 iter ~200 창발(0-199 구간 ~0%)과 정합.
**판정: climb-and-farm 지배 → A1 생략, A2 직행.**

주의: Probe A의 분포 변화 자체는 "kick-fix only" 설계가 아니라 inertia 일관성
변화가 지배 — A1 arm의 원래 목적(킥 단독 효과)은 `_diag_kick`/`_diag_inertia`
직접 계측으로 대체됨. 상세 confound 논의는 §5.

## 3. A2 — 비전 거리감쇠 캘리브레이션 (실 YOLO) → **컨테이너 버그로 차단, 분석 커브로 진행**

**도구 정비 완료** (커밋 `5343e38`, `yolo_eval.py`): X-마커 시각 부재 수정(1.5×1.5 m
텍스처 쿼드, 타깃 추적 배치 — `--marker-texture`는 데드 코드였음), DirectRLEnv에서
scene cfg 주입 카메라가 anchor 불가 → `_setup_scene` 내 TiledCamera 생성(공식
cartpole_camera 패턴), 카메라 spawn 누락 수정(PinholeCameraCfg, h_fov 정합),
teleport 스윕 중 종단 비활성화 + 매 스텝 pose 재고정, 슬랜트 3-27 m 확장(9×5 bins),
ultralytics 설치.

**차단**: TiledCamera annotator 초기화가 `TypeError: Unable to write from unknown
dtype, kind=f, size=0`로 크래시 — **stock `Isaac-Cartpole-RGB-Camera-Direct-v0`도
동일 크래시** → `isaac-lab-local:580` 이미지의 omni.syntheticdata/replicator 버전
불일치(플랫폼 레벨). 이미지 수정 후 `--calibrate` 재실행 필수 (하네스는 준비 완료).

**적용 커브 (분석적 후보, calibration-pending)**: $g(s) = \mathrm{clamp}(1 - (s-5)\cdot0.1,\ 0.1,\ 1)$
— 5 m 이내 full, 10 m에서 0.5, 14 m 이상 floor 0.1. Probe C 반사실이 이 커브로
climb 수익 ×0.12-0.16을 실증. `range_falloff_enabled` cfg 게이트.

## 4. A2 — 학습 결과 (400 iters, fresh, seed 42, wandb `v3qk07pg`)

| 지표 (iters 300-400 평균) | exp_013 (참고: 300-400 구간) | **exp_014 A2** |
|---|---|---|
| `Episode_Termination/max_altitude` (**R_alt**) | ~0.35-0.43 | **0.0000** |
| `Episode_Termination/success` | (rollout ~0.3) | **0.9985** |
| `Episode_Termination/crash` | ~0.27 (eval) | 0.0000 |
| `Episode_Termination/bad_attitude` | ~0.005 | 0.0015 |
| `Episode_Metric/d_xy_min` | 1.4 m (eval) | 0.68 m |
| `Mean action noise std` | 0.9→3.9 폭주 | **0.80 안정 (폭주 없음)** |
| `Episode_Metric/action_sat_frac` | (사후계측 0.77) | 0.46 |
| 에피소드 길이 | ~100+ steps | ~25 steps (2.5 s 완주) |

**판정: R_alt(300-400) = 0.0000 < 0.10 → attractor 붕괴 확증.**

**핵심 시그니처 — climb 모드의 탐색-후-기각**: 50-iter 버킷별 max_alt rate:
0-149 ≈ 0.001 → **150-199: 0.278** → 200-399: **0.0000**. exp_013과 거의 같은
학습 연령(iter ~150-200)에 climb 모드가 창발했지만, 감쇠 plant에선 수익이 없어
**50 iter 만에 완전 기각**됐다. exp_013(감쇠 없음)에선 같은 시점의 창발이
27-43%로 고착. attractor의 존재와 그 유인(거리무관 conf)의 인과를 학습 커브
안에서 직접 보여주는 결과.

**부수 확증 (Rule 18b 재해석)**: noise_std가 전 구간 0.75-0.80 안정 —
entropy_coef 0.005 그대로인데도 폭주 없음. 일관 plant에선 노이즈가 실제 리턴을
훼손해 task 손실이 entropy 압력을 견제한다. exp_013의 σ 폭주도 상당 부분
저토크 plant 아티팩트였다는 뜻 (σ-불변 계측과 정합 — 그 plant에선 노이즈가
자세에 거의 전달 안 됐음).

**주의**: success 99.85%는 "d_xy ≤ 0.8 도달"(고도 무관) 기준이다. 진짜 임무
지표(투하 정확도)는 `drop_impact_error_m` 재평가와 성공 반경 커리큘럼(0.8→0.5)
후 판단할 것. 또한 이 수치는 일관 plant의 효과가 겹쳐 있다 — 분리는 §5 A0′.

> **(07-05 후속 규명 — exp_016)**: eval의 `drop_impact_error_m` **4.59 m는 투하
> 오차가 아니라 지표 의미론 버그**였다. Phase 1엔 릴리스 트리거가 없고, 지표가
> d_xy-성공 종단 스냅샷(잔여속도 ~3.0 m/s 포함)의 탄도 예측을 보고 = 속도 캐리
> 3.0×1.53 s ≈ 4.6 m. 올바른 CCIP referee(≤0.2 m 트리거)로 재평가: 발화 시
> 0.137 m, 단 release_rate 6%(CCIP 스윕 최근접 med 0.755 m ≈ 경로 cross-track).
> → [[research/ccip_release_decoupling]] / [[experiments/exp_016_ccip_release_reeval]] (Rule 21)

## 5. A0′ 대조군 — 일관 plant + 감쇠 없음 (400 iters, seed 42, wandb `azoc1xp0`)

A2의 R_alt를 exp_013 기준선(~0.39)과 비교할 때 plant도 함께 바뀌었으므로(inertia
일관성·킥·로터), 붕괴의 귀속(감쇠 vs plant)을 분리하는 대조군.
`range_falloff_enabled=False` 일시 편집으로 실행(run dir의 git diff에 기록됨).

| 지표 (iters 300-400 평균) | **A2 (감쇠 ON)** | **A0′ (감쇠 OFF)** |
|---|---|---|
| `Episode_Termination/max_altitude` (R_alt) | **0.0000** | **0.0365** (버킷 300-349: 0.0583, 350-399: 0.0146) |
| `Episode_Termination/success` | 0.9985 | 0.9649 |
| `Episode_Reward/rew_vision` | 1.89 | 10.13 (×5.4) |
| `Mean action noise std` | 0.80 | 0.76 (둘 다 안정) |

**최종 귀속 판정 — 두 요인 모두 실재, 지배 요인은 plant:**

1. **exp_013 규모(33-43%)의 attractor 고착은 일관 plant에서 재현되지 않는다**
   (A0′ R_alt 0.037 ≪ 0.39). 일관 plant에선 완주가 2.5 s에 끝나 +100 회전율이
   farming 수익을 압도 — exp_013의 farming 경쟁력은 "저토크 plant에서 완주가
   어렵고 느렸기 때문"에 성립했던 것. farmer-vs-finisher 수지(Rule 18a)는
   보상 상수만이 아니라 **plant가 결정하는 완주 소요시간의 함수**다.
2. **감쇠의 인과 효과도 방향·크기 모두 실측된다**: 감쇠 없으면 climb 종단이
   후반 재창발(300-349 버킷 5.8%, 우상향 기미)하고 success가 3.4 pp 낮으며,
   감쇠가 있으면 창발 시도(150-199, 27.8%)조차 50 iter 내 완전 기각된다.
   400 iter 너머에서 A0′의 꼬리가 더 자랄 가능성도 배제 못 함.
3. **결론: 감쇠 유지** (`range_falloff_enabled=True` 기본값). 성능 효과와 별개로
   실 YOLO parity 수정(Rule 13·17)이므로 sim-to-real 관점에서 무조건 필요.
   단 커브 수치는 캘리브레이션-pending (§3).

## 관련

- [[research/isaac_inertia_ctrl_mismatch]] — plant 실체 (이 세션 최대 발견)
- [[research/exp014_ablation_protocol]] — 설계
- [[research/isaac_ppo_tuning_recommendations]] — 변경 우선순위 (1번 = 본 A2)
- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] — 기준 실험
- [[experiments/training_history]]

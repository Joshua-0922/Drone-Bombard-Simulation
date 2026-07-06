---
date: 2026-07-06
tags: [research, isaac-lab, reward-design, ccip, release, curriculum]
status: active
type: research
---

# Stage A — 밀집 CCIP 조준오차 보상으로 release_rate 올리기 (exp_017)

> [[research/ccip_release_decoupling]]의 후속. 사용자 지시: **보상 변경만으로**
> release_rate@0.2m을 6% 기준선에서 유의미하게 올릴 것. 종단/성공 게이트·
> entropy_coef·비전 캘리브레이션 불변, 탄도 모델 nominal 유지(DR은 Stage B),
> residual 브랜치 no-op 유지(Phase 1 `residual_enabled=False`).

관련: [[experiments/exp_017_stageA_aim_reward]] · [[research/ccip_release_decoupling]] · [[research/phased_curriculum]] · [[research/rl_rules]]

---

## 1. 보상 항 정식화

기존 Layer-3 항들과 같은 dense·smooth 매핑 (`1 - tanh`):

$$r_{\text{aim}} = w_{\text{aim}} \cdot \left(1 - \tanh\!\frac{e_{\text{aim}}}{s}\right), \qquad
e_{\text{aim}} = \left\lVert \underbrace{p_{xy} + v_{xy}\,(t_{\text{fall}}+t_d)}_{\text{predict\_impact\_nominal}} - \text{target} \right\rVert$$

- $e_{\text{aim}}$은 **스크립트 릴리스 referee가 매 policy step 평가하는 바로 그 양**
  (`_advance_phase_dynamics` → `_aim_err_last` 스태시 → `_get_rewards` 소비;
  dones-before-rewards 계약으로 항상 당 스텝 값).
- `DroneBombardRewardCfg.w_aim`(기본 **0.0 = off**, exp_014 보상 bit-parity 유지),
  `aim_reward_scale`(tanh knee, m). train.py `--w_aim/--aim_reward_scale`로 주입.
- `distance_to_goal`(w_dist)은 그대로 — 거친 "접근" 신호 위에 미세 "조준" 신호를 얹는 구조.
- 순수 함수 `math_utils.aim_error_reward` (+ 유닛테스트 2종, 40/40 PASS).

## 2. 사전 적대 검증 (5-lens 워크플로, 학습 전)

| Lens | 판정 | 핵심 |
|---|---|---|
| parity (w_aim=0) | ✅ 생존(high) | 보상 bit-identical·RNG 불변 → 기준선 런 유효 |
| timing | ✅ 생존(high) | `_aim_err_last` 항상 당-스텝·전-env 갱신, partial reset 누수 불가 |
| plumbing | ✅ 생존(high) | cfg 주입·rsl_rl warm-start(N iters **추가** 실행) 실증. ⚠️ `--final_out` 무조건 덮어씀 → run별 경로 분리 |
| physics | ✅ 생존(high) | v=d/T 감속 프로파일 실현가능(감속≤2.14 m/s² ≪ clamp 8), 0.2 m 윈도 **26–28 연속 스텝** 유지, 조준 수익이 직선 통과 대비 ~16× |
| pathology | ❌ **반박(medium)** | w_aim=2.0/knee 0.5에서 duty-cycle 방사 펌핑 farm PV ~137–180 vs 완주 ~149 (γ=0.995) — 배제 불가 |

**헤지 채택**: 1차 런은 `w_aim=1.0`(펌프 손익분기 duty ~100% = 물리적 불가) +
farm 시그니처 모니터링(에피소드 길이·timeout/stagnation·rew_aim/step).
검증자가 제안한 `~released` 게이팅은 **역효과**로 판단(완주자 수익을 farmer보다
더 깎아 경제를 반대로 뒤집음) — 미채택.

## 3. 결과

### 3a. HEAD 6-dim Phase-1 기준선 (750gpldr, 400 iters, w_aim=0)

**근접 학습이 릴리스 능력을 능동적으로 파괴한다**: 학습 내 release_rate
12.2%(0–49) → 3.7%(350–399) **단조 하락**, aim_err_min 0.80→1.19 m 악화,
final_speed 1.87→2.87 m/s 상승. 정책이 빠른 완주를 최적화할수록 CCIP 스윕은
타겟에서 멀어진다. Deterministic 200-ep: success 100%, **release_rate 2.5%**,
aim_err_min med 1.146 m. (exp_014 A2 4-dim의 6%보다도 낮음 — 6-dim 신규 학습의
개체차.)

### 3b. Stage A v1 (6z0gpnhy, w_aim=1.0, knee 0.5 m, 400 iters, P1 warm-start)

- 학습 내 지표는 **평평**(release_rate ~3.2%, rew_aim ~0.17/ep) — σ~1.1의 탐험
  노이즈가 T≈1.5 s CCIP 지평으로 증폭돼(±m급 착탄점 지터) 신호를 가림.
- **Deterministic에서는 방향이 정확히 맞음**: release_rate 2.5→**5.5%**(2.2×),
  aim_err_min med 1.146→**0.889 m**, final_speed 3.35→**2.72 m/s**(감속 방향).
- 결론: 그래디언트는 작동하나 **너무 약함** — 정책 작동점(aim_err ~0.9 m)에서
  knee 0.5 m의 기울기는 ~0.14/m뿐.

### 3c. Stage A v2 (fv5qqmtz, w_aim=2.0, knee 1.0 m, 600 iters, v1 warm-start)

작동점 기울기 ~6× 강화(0.84/m @1 m) 의도. farm 안전선 유지(hover@1 m 소득
0.48/step, stagnation guard 150-step 캡) — 실제로 farm 시그니처 전무.

**결과: v1의 이득을 되돌림(회귀)**. 학습 내 release_rate 전 구간 ~3.0–3.5%
평탄, rew_aim ~1.4/ep(**행동 변화 없이 걷는 수동 소득** — 넓은 knee가 현재
행동에 그냥 지불), σ 1.18→**1.55 폭등**. Deterministic 200-ep:
release_rate **3.5%**, aim_err_min med **1.096 m**, final_speed **3.45 m/s**
— 세 지표 모두 v1(5.5%/0.889/2.72)에서 기준선 방향으로 후퇴. 넓은 knee는
σ_CCIP ~2 m 지터 아래서 기대보상면을 평탄화해 확률적 그래디언트가 오히려
약해지고, 남는 압력은 γ-할인 완주 가속뿐이었던 것으로 해석.

## 4. 왜 보상만으로는 어려운가 (구조 분석)

1. **γ-할인 완주 보너스가 모든 감속을 벌한다**: +100 성공이 0.1 s 지연될 때마다
   PV −0.5. tanh 밴드(≤1.5 m) 안 체류 소득은 밴드 폭이 캡 — 근접 통과 속도
   ~3 m/s에서 밴드 통과는 ~1 s. 국소 섭동(약간 감속)은 순손실 → 완전한
   CCIP-hold 프로파일(순이득 +25~49)과의 사이에 **return valley**.
2. **탐험 노이즈의 CCIP 증폭**: 속도 노이즈 σ_v가 착탄점에 T≈1.5 s 배율로 전달
   → 학습 중 aim_err 지터가 밴드 폭을 압도, 기대 그래디언트 평탄화.
   entropy_coef 0.005 불변 제약(사용자)이라 σ 자체를 줄일 수 없음.
3. **성공 종단이 조준 구간을 잘라먹는다**: d_xy≤0.8 도달 즉시 에피소드 종료 —
   조준을 다듬을 근접 구간이 존재하지 않음. (릴리스가 종단 이벤트인 Phase 2
   구조가 이 문제의 정공법.)

## 5. 판정 & 다음 단계

**판정 = (b): 보상 변경 단독으로는 정체 — 사용 가능한 release_rate에 도달하지
못함.** 두 번의 개입(w=1/knee 0.5 → w=2/knee 1.0), 총 1,000 warm-start iters:
최고치는 v1의 deterministic 5.5%(자체 기준선 2.5% 대비 2.2×이나 n=200에서
p≈0.13 — 통계적으로 미확정; 방향성 자체는 aim_err_min med 1.146→0.889 m,
final_speed 3.35→2.72 m/s의 분포 이동으로 실재). exp_016의 6% 참조 기준선을
"충분히 상회"하는 수준에 전혀 못 미치고, 더 강한 v2는 회귀. §4의 세 구조
요인(γ-할인 완주 보너스·노이즈 CCIP 증폭·성공 조기 종단)이 결합해 dense
shaping이 이길 수 없는 지형 — **릴리스는 사이드 지표가 아니라 종단 이벤트로
학습되어야 한다** (Rule 22).

- **Stage B(별도 지시 대기)**: Phase 2 release-conditioned terminal reward
  (이미 구현됨, `release_enabled=True`)가 정공법 — 릴리스가 종단이 되면 위 1·3
  구조 문제가 소멸. DR(drag/wind)+residual도 그 단계에서.
- ⚠️ **w_aim을 Phase 2+로 그대로 가져가지 말 것**: `_evaluate_release`의
  aim_err는 residual 포함 — 정책이 residual 채널만으로 aim_err를 조작해
  비행 정확도 없이 소득 채굴 가능 (pathology lens 2차 플래그).
- farm 시그니처 모니터 항목(이번에 정립): 에피소드 길이 creep + 지속
  rew_aim/step > ~0.5 + timeout/stagnation 상승.

## 관련
- [[experiments/exp_017_stageA_aim_reward]] — 런 상세/원자료
- [[research/ccip_release_decoupling]] — 문제 정의 (Rule 21)
- [[research/phased_curriculum]] — Phase 2 릴리스 종단 설계
- [[experiments/training_history]]

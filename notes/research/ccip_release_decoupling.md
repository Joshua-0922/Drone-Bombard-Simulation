---
date: 2026-07-05
tags: [research, isaac-lab, ccip, release, metrics, ballistics]
status: active
type: research
---

# CCIP 릴리스 디커플링 — success 100% vs drop_impact_error 4.59 m의 실체

> exp_014 A2의 deterministic eval에서 success 100%(d_xy ≤ 0.8 m)와
> `drop_impact_error_m` 4.59 m이 동시에 나온 원인 규명. **버그 확정: 릴리스
> 판정이 존재하지 않았고, 지표가 "잘못된 순간"(성공 종단 스냅샷)의 탄도 예측을
> 보고하고 있었다.** 릴리스 설계 의도(v15 `drop_calculator_node`)는 CCIP
> 예측착탄 오차 ≤ 0.2 m 트리거다.

관련: [[experiments/exp_016_ccip_release_reeval]] · [[experiments/exp_014_A2_visionrange]] · [[research/rl_rules]] Rule 21

---

## 1. 크럭스 답변 — 릴리스 조건은 (a) 위치 기반이었다

Phase 1(exp_014 코드, 커밋 `6407f8d`)에는 **릴리스 트리거 자체가 없다**:

- 종단(=암묵적 "릴리스 순간"): `success = d_xy <= rc.success_radius` —
  `drone_bombard_env.py:701` (구), HEAD `:901`. **순수 위치 조건, 속도 무관.**
- 지표: `_log_reset_extras`가 그 성공-종단 스냅샷에서
  `ballistic_impact(final_pos_xy, final_vel_xy, final_alt, 0.1, g)`로 착탄점을
  예측 — **종단 순간의 잔여 속도가 그대로 탄도에 실린다**:

$$\text{impact} = \mathbf{p}_{xy} + \mathbf{v}_{xy}\cdot\left(\sqrt{2H/g} + t_{delay}\right)$$

- `DropCfg.release_tolerance = 0.2`("was drop_calculator_node drop_tolerance")는
  **정의만 되고 어디서도 읽히지 않았다.**
- `math_utils.py:114-117`의 vz≈0 특수화 주석("matching the Gazebo referee which
  triggers at/near success")이 이식 가정의 화석: Gazebo에선 referee가 릴리스
  시점을 정했지만, Isaac 이식에선 그 시점이 "성공 종단"으로 슬쩍 바뀌었다.

## 2. 산수 — 4.59 m는 속도 캐리 그 자체

계측(재평가 harness): 종단 시 평균 수평속도 **~3.0 m/s**, 고도 ~10 m
→ $t = \sqrt{2\cdot10/9.81}+0.1 = 1.53$ s → 캐리 $3.0 \times 1.53 \approx 4.6$ m.
**4.59 m는 투하 정확도가 아니라 "성공 순간에 눈감고 떨어뜨리면 어디 떨어지나"의
답이었다.** (d_xy ≤ 0.8 슬랙까지 포함하면 관측치와 정확히 정합.)

## 3. 수정 — 스크립티드 CCIP referee (지표 전용)

매 policy step, $|\text{predicted impact} - \text{target}| \le 0.2$ m ∧ 고도 > 1 m
최초 충족 시 `_released` 래치, 그 순간의 실착탄 오차를 `drop_impact_error_m`으로
보고. 구(종단 스냅샷) 지표는 `drop_impact_error_terminal_m`으로 개명 보존.
추가 진단: `release_rate`, `aim_err_min_m`(에피소드별 CCIP 오차 최솟값 — 10 Hz
샘플링 윈도우-스킵 진단), `final_speed_xy`.
**보상·종단 절대 불변**(`_just_released` 미설정, Phase-1 분기에서 릴리스 버퍼를
읽는 코드 없음 — 학습 동역학 bit-identical).

## 4. 재평가 결과 (A2 `model_final.pt`, deterministic 200 ep) — [[experiments/exp_016_ccip_release_reeval]]

| 지표 | 값 (200 ep, deterministic) |
|---|---|
| success (d_xy ≤ 0.8) | **100.0% (200/200)**, d_xy_min mean 0.660 m (exp_014 재현 ✓) |
| **release_rate @0.2 m, 10 Hz** | **6.0% (12/200)** |
| **release_rate @0.2 m, 100 Hz referee** | **11.5% (23/200)** |
| drop_impact_error@release | **10 Hz: mean 0.137 m** / 100 Hz: mean 0.172 m (≤ tol by construction) |
| aim_err_min (10 Hz / 100 Hz) | med 0.776 / **0.755 m** (p90 1.39/1.36, max 2.04) |
| drop_impact_error_terminal (구 지표 재현) | **mean 4.649 m** (med 4.54, p90 5.70) — 4.59 m 재현 ✓ |
| final_speed_xy @종단 | mean 2.989 m/s → 2.99×1.53 s = 4.57 m 캐리 (산수 봉합 ✓) |

**핵심 해석**: 접근 정책의 CCIP 스윕 최근접(100 Hz med **0.755 m**)이
**d_xy_min(0.665 m)과 사실상 동일**. 기하학적으로 당연: 착탄 예측점은 속도
레이를 따라 경로를 앞서갈 뿐이므로, 스윕의 cross-track 최근접 ≈ 비행경로의
cross-track 오차. **d_xy ≤ 0.8 근접 보상으로 학습한 정책은 0.2 m 릴리스
윈도우를 통과할 이유가 없다** — 릴리스가 6%에서만 발화한 이유.
100 Hz vs 10 Hz 차이는 aim_err_min ~0.02-0.04 m·release_rate 2배(6→11.5%)로
실재하나 부차적 — 지배 요인은 샘플링이 아니라 정책의 cross-track 정확도.
따라서 "새 drop_impact_error"의 정직한 보고: **릴리스 발화 시 0.14-0.17 m
(설계상 ≤0.2), 단 이 정책은 6-11.5%에서만 릴리스 가능; 실질 하한은 CCIP
최근접 ~0.75 m(중앙값)** — 진짜 투하 능력은 exp_015 Phase 2(릴리스 조건부
보상)가 만들어야 한다.

## 5. exp_013과의 비교 (질문 #4)

같은 지표 코드(동일 종단-스냅샷 의미론, exp_012에서 도입). exp_013의 24 m는
실패 지배(36% success, max_alt 33%가 >15 m 고도·고속 종단 → 거대한 탄도 캐리)로
**같은 메커니즘의 극단값**. 디커플링은 plant 수정이 만든 게 아니라 **지표 도입
시점부터 내재**했고, exp_014의 100% success가 이를 깨끗하게 노출했을 뿐.

## 6. 적용 규칙 / 후속

1. **Rule 21**: 종단-스냅샷에서 계산되는 "임무 지표"는 종단 조건과 독립인지
   검증하라. 지표가 특정 "이벤트"(릴리스)를 가정하면, 그 이벤트를 명시적으로
   시뮬레이트해서 그 순간에 측정할 것.
2. exp_015 Phase 2가 이미 올바른 CCIP 트리거(`_evaluate_release`, tol 0.5 m)를
   구현 — 본 수정으로 Phase 1 지표가 Phase 2와 같은 의미론으로 정렬되어
   페이즈 간 `drop_impact_error_m` 비교가 유효해짐.
3. Phase-2 학습 시 `aim_err_min` 추이가 "릴리스 가능 정책"으로의 수렴 신호.
   tol 0.5 m(10 Hz)는 스윕 속도 ~0.3-0.8 m/step 대비 윈도우 1.0 m로 대체로
   안전하나, 고속 접근에선 스킵 가능 — `aim_err_min`이 tol 바로 위에 몰리면
   샘플링 레이트/톨러런스 재조정.
4. vz 미반영(t=√(2H/g) 특수화)은 예측·"실측" 양쪽에 동일 적용되어 상호 일관
   — 실 페이로드 바디 도입 시 full-vz 공식(`t=(v_z+\sqrt{v_z^2+2gH})/g`)으로
   업그레이드 필요(문서화만, 이번 세션 범위 밖).

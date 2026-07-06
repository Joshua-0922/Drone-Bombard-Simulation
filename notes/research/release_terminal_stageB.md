---
date: 2026-07-06
tags: [research, isaac-lab, reward-design, ccip, release, termination]
status: active
type: research
---

# Stage B — 릴리스-종단 구조로 release_rate 5.5% → 100% (exp_018)

> [[research/ccip_aim_reward_stageA]](판정 b)의 후속. 사용자 지시: d_xy≤0.8 근접
> 종단을 **릴리스-종단 이벤트**로 교체(실패 게이트·타임아웃 불변), aim_err 보상은
> **nominal CCIP 전용**으로 고정(reward-hacking 차단), Stage-A 보상 탐색을 교정된
> 종단 아래서 소폭·단일-노브로 재실행. DR/residual은 여전히 별도 지시(Stage C).

관련: [[experiments/exp_018_release_terminal]] · [[research/ccip_aim_reward_stageA]] · [[research/ccip_release_decoupling]] · [[research/phased_curriculum]] · [[research/rl_rules]]

---

## 1. 구현

- **`DroneBombardEnvCfg.release_terminal`** (기본 False = 레거시 bit-identical;
  phase 파생 아님 — Phase 2+의 `release_enabled`와 독립). True면 Phase-1
  스크립트 CCIP referee(aim_err≤0.2 m ∧ alt>1 m)의 발화가 `_just_released`를
  올리고 `_get_dones`가 이를 **종단 success**로 사용(+100). 근접 성공 제거 —
  드론은 타겟 근처를 **배회하며 dense 보상을 계속 수령**, 릴리스/실패 게이트/
  타임아웃까지. crash/overspeed/bad_attitude/out_of_range/max_altitude/
  overshoot/stagnation·타임아웃 전부 불변(적대 검증으로 byte-미접촉 확인).
  stagnation guard(150-step 창, 1 m 진행)가 배회 상한 ~150 스텝 역할 겸임.
- **aim_err 보상 = nominal 전용 (DELIBERATE)**: 양 referee 경로 모두
  `_aim_err_last`에 `predict_impact_nominal` 기반 오차만 스태시. Phase 2+에서
  트리거는 설계대로 residual-포함 유지, **보상 양은 residual 미포함** — 정책이
  비행 없이 residual 출력만으로 조작 가능한 양에 보상을 걸지 않는다는 원칙을
  코드 주석으로 명문화 (Stage A pathology lens의 2차 플래그 해소).
- `train.py --release_terminal` / `play.py --release-terminal` (평가 의미론은
  학습과 일치해야 함 — 레거시 모드로 평가하면 근접 종단이 배회를 잘라 오측).
- **버그 수정(적대 검증 발견)**: 신규 branch의 `success = self._just_released`가
  텐서 **alias** — `_reset_idx`의 in-place clear가 step() 반환 전에
  `_done_flags`를 지워 **평가 하니스가 완벽한 정책을 success 0%로 보고**할
  뻔함(학습/wandb는 스냅샷 clone이라 무영향). `.clone()`으로 수정 + Phase-2
  branch의 잠재 alias도 동일 수정.

## 2. 결과 — B0: 종단 교체 단독 (xt0hrr1c, w_aim=1.0/knee 0.5, v1 warm-start, 400 it)

Stage A v1과 **보상·warm-start 완전 동일, 종단만 교체**:

**학습 내 release_rate가 단조 상승 23% → 99.6%** (25-iter 버킷):

| iters | release_rate | max_alt | aim_err_min | speed | ep_len |
|---|---|---|---|---|---|
| 800–824 | 23.4% | 78.3% | 0.47 m | 0.71 | 52 |
| 900–924 | 33.0% | 68.0% | 0.34 m | 0.54 | 51 |
| 950–974 | 71.5% | 30.2% | 0.18 m | 0.28 | 46 |
| 1025–1049 | 91.7% | 9.3% | 0.144 m | 0.23 | 41 |
| 1175–1199 | **99.6%** | 0.5% | 0.136 m | 0.21 | 36 |

- Stage A의 단조 **하락**(12→3.7%)이 정확히 반전. 초기 과도기: v1의 미학습
  post-approach 행동이 25 m 천장으로 흘러 max_alt 78%로 시작 → 250 iter 내
  릴리스로 전환. stagnation/timeout/overshoot 전 구간 0.
- **Deterministic 200-ep: release_rate 100.00% (200/200), drop_impact_error@release
  0.125 m (p90 0.183, max 0.198)**. 종단 속도 med 0.11 m/s, d_xy_min med 0.12 m —
  **호버-드롭 프로파일**로 수렴(속도 캐리 소멸 → 레거시 terminal 지표도 0.125 m로 일치).

## 3. 보상 탐색 (교정된 종단, 단일-노브·소폭)

| Run | w_aim | knee | det release_rate | drop err (mean/max) | ep_len(train 종단) | 비고 |
|---|---|---|---|---|---|---|
| B0 | 1.0 | 0.5 | **100.00%** | 0.125/0.198 m | 36 | 기준 |
| B1 | 1.5 | 0.5 | **100.00%** | 0.131/0.200 m | 36 | w 축 +50% — B0과 사실상 동일 (수렴 궤적도 일치) |
| B2 | 1.0 | 0.75 | 98.51% | 0.137/0.199 m | 43 | knee 축 +50% — 근소 열화: max_alt 4건, aim med 0.155, 종단 속도 0.30 (넓은 knee가 근-윈도 기울기 희석) |
| B3 | 0.0 | — | **100.00%** | 0.128/0.198 m | 37 | 하한 앵커 — B0과 사실상 동일(수렴 소폭 지연: 90% 도달 iter ~1025 vs ~1100) |

**탐색 판정**: 교정된 종단 아래서 성과는 aim 보상 노브에 **거의 불감** —
w_aim 0/1.0/1.5 모두 100%·~0.13 m(0.5 knee), 넓은 knee(0.75)만 근소 열화
(98.5%, aim med 0.155, 종단 속도 0.30 — 근-윈도 기울기 희석). **aim 항은
릴리스-종단 구조에서 사실상 잉여**: +100 종단 이벤트(referee 자동 발화)가
학습을 견인. 소폭의 mid-training 가속(B0 vs B3) 정도의 부가가치. ⚠️ 주의:
B3의 warm-start(v1)는 Stage A에서 aim 보상으로 학습된 가중치 — "aim 보상을
한 번도 안 쓴" 실험은 아님(from-scratch B3는 미실행, 필요 시 후속).
**Stage C warm-start 권장 = B0**(정식 보상 구성 + 최우수 수치).

## 4. Stage A의 구조 요인 3종 — 해소 vs 잔존

1. **성공 조기 종단(잘림)** — ✅ **해소, 이것이 지배 요인이었음.** 같은 보상·
   같은 warm-start에서 종단만 바꿔 5.5%→100%. 근접 종단이 조준을 다듬을 구간
   자체를 제거하고 있었다.
2. **노이즈의 CCIP 증폭** — ✅ **차단 요인 아님이 판명** (σ~1.35에서 학습 완료).
   구조가 반전시킴: referee가 **자동 발화**라 정책이 윈도 근처에서 노이즈로
   우연히 0.2 m를 찍으면 즉시 +100 이벤트가 샘플링됨 — Stage A에서 그래디언트를
   평탄화하던 노이즈가 Stage B에선 **발견 메커니즘**으로 작동. (경제 분석이
   경고한 "0.2 직상 유지 farm"도 같은 이유로 물리적으로 유지 불가.)
3. **γ-할인의 감속 억압** — ✅ **역전.** 감속이 더 이상 +100 기회를 버리지 않음.
   에피소드 길이가 52→36으로 **줄며** 수렴(배회-farm으로 늘지 않음) — 사전 경제
   분석의 "정지 hazard 균형" 우려(-배회 소득이 1.84/step 넘으면 never-fire)와
   달리, 자동 발화 + stagnation 상한 + overshoot 가드가 farm 경로를 막았고
   정책은 신속-호버-드롭을 택함.

## 5. 규칙화 (Rule 23)

임무 이벤트는 **종단 이벤트**로 학습시켜라 — 자동 발화 referee가 탐험 노이즈를
발견 메커니즘으로 바꾼다. (a) 동일 보상·동일 warm-start에서 종단만 바꿔
5.5%→100%: Rule 22a의 인과 확정. (b) 정책이 "임계 직상 유지"를 할 수 없는
구조(노이즈 dip → 자동 발화 → +100 샘플링)가 farm 균형을 물리적으로 배제.
(c) 이벤트가 종단이 되면 그 이벤트를 겨냥한 dense shaping은 거의 잉여(B3≈B0);
쓰려면 좁은 knee 유지(넓히면 열화, B2). (d) done-flag를 reset이 in-place
변조하는 버퍼의 alias로 캐시하지 말 것 — `.clone()` (평가 하니스가 post-step에
읽음; wandb는 스냅샷 clone이라 면역이었음). (e) 이 모드에선 release_rate ≡
success rate(발화=종단)이므로 비종단 래치 시절 수치와 직접 비교 금지.

## 6. 수렴 행동·Stage C 관찰

- **호버-드롭 프로파일**: 타겟 직상 저속 진입(med 0.07–0.11 m/s, d_xy_min med
  0.04–0.12 m) → aim_err→0 → 발화. v15 Gazebo의 "비행 중 CCIP 릴리스"와 다른
  해법이지만 referee 의미론상 동등하며 nominal 물리에선 최적.
- Stage C(DR: drag/wind)에서 이 프로파일의 강건성은 열린 질문 — 바람이 있으면
  호버 드롭도 캐리가 생겨 residual 보정이 필요해짐(정확히 Phase 2 설계 의도).
  릴리스 고도(스폰 9–11 m 유지) 로깅 추가 검토.
- 사전 경제 분석(정지-hazard 균형, 배회 소득>1.84/step → never-fire)은 자동
  발화 특성 때문에 미발현 — 단 Stage C에서 release_tolerance가 0.5로 넓어지면
  (Phase 2 기본) 발화가 더 쉬워져 같은 방향. DR 도입 시 재점검.

## 관련

## 관련
- [[experiments/exp_018_release_terminal]] — 런/커브/아티팩트
- [[research/ccip_aim_reward_stageA]] — Stage A 실패 분석 (Rule 22)
- [[research/phased_curriculum]] — Phase 2 본선(DR+residual)과의 관계
- [[experiments/training_history]]

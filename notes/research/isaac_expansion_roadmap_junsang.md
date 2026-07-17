---
date: 2026-07-15
tags: [research, roadmap, isaac, ccip-residual, domain-randomization, vision, expansion, planning]
type: research
status: active
owner: junsang
---

# Isaac 모델 확장 로드맵 — CCIP residual 다음단계 개요 + 남은 축 정리

> **현재까지:** v11(완화)·v12(랜덤 marker)·v13(부분관측) 전부 100% success.
> 이 문서 = ① **다음 단계(CCIP residual + DR) 상세 개요** + ② **남은 확장 축 전체 로드맵**.
> 관련: [[research/isaac_v11_v13_design_guide_junsang]] · [[experiments/exp_008_v13_partial_obs_junsang]]

---

## 0. 현재 위치 (완료)

| 버전 | 확장 | 결과 |
|------|------|------|
| v11 | 완화 baseline (완벽obs·고정 marker·정책 drop_signal) | ✅ 100%, 착탄 0.43m |
| v12 | 랜덤 marker (5m disk) | ✅ 100%, ~0.72m |
| v13 | 부분관측 (blind→7m reveal) | ✅ 100%, ~0.8m |
| v14 | DR + CCIP residual (Stage A, 바람 관측) | ✅ residual ON>OFF (착탄 0.69/0.82m) — [[experiments/exp_009_v14_ccip_residual_junsang]] |
| v15 | 바람이 기체에 작용 | ✅ wind-test 검증 / wind 2.0 튜닝(dry-run 미실행) — [[experiments/exp_010_v15_airframe_wind_junsang]] |
| v16 | **실제 물리 payload drop** | ✅ drop-test PASS, dry-run success 0.80·실제 착탄 sub-meter — [[experiments/exp_011_v16_physical_drop_junsang]] |

이동타겟/vision은 여전히 **inert hook**. **업데이트:** residual/DR은 v14/v15에서, 물리 drop은 v16에서
착수 완료. 남은 큰 축 = **vision, 이동 타겟, 시변 바람, 그리고 과제 2(시각/검증 도구)**.

---

## 1. ⭐ 다음 단계 — CCIP residual + Domain Randomization (개요)

### 1.1 목표 & 핵심 난제 (wind trap)
- **residual = nominal 탄도 예측과 실제 착탄의 gap을 정책이 학습으로 보정.** (base: `action[4:6]`, ±3m; v11계열은 [4]=drop_signal이라 **[5:7]로 확장** 필요)
- DR을 켜면 env마다 바람/드래그가 달라 nominal과 실제가 어긋남 → residual이 메울 대상 생김.
- **wind trap**: 정책이 바람을 **모르면**(obs에 없음) env마다 필요한 residual이 달라 **학습 불가**. → 관건은 **바람/편향을 관측 or 추정하게 하는 것**.

### 1.2 설계 — 2단계
**🟢 v14 (Stage A) — 메커니즘 검증 (cue를 obs로 제공):** v12 위에(부분관측 v13 아님, 어려운 문제 2개 동시 X)
- DR: 에피소드마다 wind_xy(±1.5)·drag(U[0,0.15]) 샘플 (이미 plumbed, `dr_enabled` on). 시작 약하게(wind 1.0)→증가.
- action **7D**: `[0:4]`속도·`[4]`drop_signal·**`[5:7]` residual(±3m)**.
- obs **27D**: v12 24D + wind_xy(2)+drag(1) → 정책이 편향→residual 매핑 학습.
- 적용: 보정착탄 = nominal + residual → **게이트/조준은 보정값**, **착탄 보상은 실제(drag/wind) 착탄**.
- **대조군**: DR on + residual **off** → residual이 실제로 오차 줄이는지 인과 증명.

**🟡 v15 (Stage B) — 현실화 (cue 제거→추정):** 바람 등을 obs에서 빼고 **운동 이력에서 추정**(frame-stack/RNN). v14 성공 후.

### 1.3 넣을 요인 — "체계적 편향" vs "비가역 잡음"
> residual은 **입력으로 예측 가능한 체계적 편향만** 보정. 관측 불가 랜덤은 못 고침(분산만↑).

**A. 체계적 편향 (residual 학습 대상, cue 필요):**
| 요인 | 효과 | 현재 | cue |
|---|---|---|---|
| payload 드래그(Cd) | 낙하 짧아짐 | drag_coef 있음 | 종류/추정 |
| **payload 질량 변동** | 탄도계수 변화 | 고정→랜덤화 추가 | 질량 obs |
| 바람(정상풍) | 드리프트 | wind_xy 있음 | 관측/추정 |
| release delay/지연 | 지연 중 이동 | release_delay 있음 | 지연값 |
| release 사출 임펄스 | 기구 kick·downwash | 없음→추가 | 기구특성 |
| **타겟 고도≠0(지형)** | 낙하거리 변화 | 평지 z=0→랜덤화 | 타겟 고도 |
| 공기밀도(고도의존) | 드래그 변화 | 없음 | 고도 |

**B. 비가역 잡음 (robustness·오차 하한):**
- **고도/상태추정 노이즈** — 낙하시간 $t_f\propto\sqrt{2h/g}$ 직결, **가장 치명적**. GPS/IMU/속도 노이즈.
- 돌풍/난류(시변 바람), payload 텀블링.

### 1.4 지침
1. **CCIP 오차 큰 순 우선**: 고도정확도 > 드래그(질량/Cd) > 바람 > release 속도/지연.
2. **각 편향의 cue를 obs→추정으로 단계 이행** (wind trap 일반화).
3. **점진 도입 + 대조군**: (드래그+질량)→(바람)→(지연+지형)→(노이즈), 각 단계 residual off/on 대조.
4. **지표**: residual ≈ 실제 drift 추종 / 착탄오차가 B 잡음 하한까지 / ablation으로 편향 크기.

---

## 2. 남은 확장 축 전체 (로드맵)

축별로 **현재 → 다음 → 최종**:

### 2.1 인지 (Perception)
`완벽 obs(v12)` → `거리게이트 reveal(v13 ✅)` → **`픽셀 양자화(v17 ✅)`**(위치를 셀 중심으로 대략, 셀∝거리) → **진짜 vision**(핀홀 카메라 u,v,conf, footprint≈고도×0.58, down-camera) → **YOLO 실물**(yolo_eval, 단 TiledCamera 버그로 캘리브 블록됨) → sim-to-real 비전.
- v17로 "대략 위치 + 가까워야 정밀" 근사 완료(success ~0.8) → [[experiments/exp_012_v17_pixel_vision_junsang]]. 다음은 실제 핀홀 카메라.
- 최난: "안 보이는 타겟 접근 + 인지 오차" 동시.

### 2.2 물리 / 탄도 (Ballistics)
`nominal(v11-13)` → **DR + residual(v14/v15, §1)** → 질량/지형/사출/공기밀도 확대 → 센서·고도 노이즈 하한.

### 2.3 타겟 동역학 (Target dynamics)
`정지(현재)` → **이동 타겟**(base Phase 3, `moving_target_enabled`) → 등속→기동(가속/회피) → 다중 타겟·타겟 선택.
- 정책이 이동 예측·리드(lead) 조준 학습.

### 2.4 시나리오 / 핸드오프 (Scenario)
`고정 +X·4m/s(v11-13)` → **랜덤 핸드오프**(초기 방향·속도·자세·고도 범위) → 순항속도↑ / 스폰 원뿔·거리 확대 → reveal 반경↓(v13 난이도↑).

### 2.5 드롭 / 페이로드 realism
`analytic payload(현재)` → **물리 payload**(rigidbody 실제 낙하·충돌) → 다중 payload·투하 순서 → 투하 후 재기동(연속 임무).

### 2.6 통합 / Sim-to-real
`단일 컴포넌트` → **커리큘럼/다단계 통합**(--phases) → 전체 통합 임무 → **sim-to-real**(도메인갭·지연·실기) → PX4/하드웨어.

---

## 3. 우선순위 제안 (의존성 기반)

1. **CCIP residual + DR (v14→v15)** ← 다음 (§1). 물리 realism의 관문.
2. **이동 타겟** — residual과 독립, 병렬 가능. 리드 조준.
3. **진짜 vision** — 가장 큰 인지 축. v13(거리게이트)이 징검다리였음.
4. **랜덤 핸드오프 / 난이도 스케일** — 각 축 성공 후 일반화 강화.
5. **물리 payload / 다중 payload** — 임무 realism.
6. **통합 + sim-to-real** — 마지막.

> 원칙 유지: **한 번에 하나씩 · 삭제 없이 토글 · 대조군으로 인과 확인 · 각 단계 smoke→dry-run 검증**.

---

## 관련 노트

- [[research/isaac_v11_v13_design_guide_junsang]] · [[research/isaac_cruise_handoff_junsang]] · [[research/rl_rules]]
- [[experiments/exp_006_v11_dryrun_junsang]] · [[experiments/exp_007_v12_random_marker_junsang]] · [[experiments/exp_008_v13_partial_obs_junsang]] · [[experiments/training_history]] · [[00_index]]

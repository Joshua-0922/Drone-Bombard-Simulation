---
date: 2026-07-21
tags: [isaac-lab, payload, physics, gpu-vectorization, design-pattern]
status: active
type: research
---

# 물리 페이로드 부착/분리 — kinematic weld 패턴

> **출처 실험:** [[experiments/exp_019_physical_payload]] (2026-07-21, 검증 4/4 PASS)

## 문제

GPU-복제(replicate_physics) PhysX에서 "드론에 페이로드를 달았다가 조건 충족 시 분리"를
per-env로 구현해야 함. 표준 접근(fixed joint 생성 → 릴리스 시 제거)은 **per-env 조인트
토폴로지 변경이 GPU 텐서 물리에서 불가능**이라 막힘 — 전 env 동시 분리만 가능해 RL 학습에
쓸 수 없다.

## 패턴 (검증됨)

1. **부착 = 매 physics step pose+velocity write** (kinematic weld). 페이로드는 일반 dynamic
   RigidObject지만 매 step 드론 pose(+마운트 오프셋)·velocity를 덮어써 사실상 용접.
   조인트 없음 → 복제 물리 무접촉, 완전 벡터화(부착 env 서브셋만 write).
2. **분리 = write 중단.** 마지막 write의 속도를 물려받고 물리가 자유낙하 적분. 기계 지연은
   physics-step 카운트다운으로 재현(`release_delay`/`sim.dt`).
3. **착탄 = z-임계 래치.** 분리 env의 페이로드 중심 z ≤ 임계(안착 높이 h/2 + 마진) 최초
   도달 시 xy를 측정 착탄점으로 래치. 100 Hz 체크면 종단 낙하속도(~14 m/s)에서도 스텝당
   0.14 m — 임계 0.10 m로 충분.

**계측 근거 (exp_019):** 부착 추적 오차 max 1.1 mm(1 s 호버) · 측정 착탄 vs 해석적 CCIP
예측 |Δ| mean 0.012 / max 0.021 m (8/8, Phase-1 drag-free).

## 트레이드오프 / 제약

- **부착 중 하중 미전달:** weld가 kinematic이라 드론 solver는 페이로드 무게를 못 느낌 →
  드론 authored 질량에 페이로드를 포함(`loaded_mass`)해 보정. 결과: **분리 후에만** ~0.1 kg
  팬텀. 현 커리큘럼(릴리스=종단)에선 미발현; 에피소드를 착탄까지 연장하면 per-env
  `_ctrl_mass` 또는 보상력 필요. 런타임 질량 변경은 Rule 19 금지.
- **world-frame 오프셋:** 마운트 오프셋을 world -Z로 적용(해석적 referee의 릴리스 상태와
  일치 우선). 틸트 클램프 35°에서 시각 오차 ≤ 0.08 m.
- **DR 갈라짐(Phase 2+ 주의):** drag/wind는 해석식(`ballistic_impact`)에만 있음. 물리
  페이로드 낙하에 동일 힘을 가하지 않으면 Phase 2에서 측정 착탄 ≠ 해석적 DR 착탄.
  referee를 물리 착탄으로 교체하기 전 반드시 정합할 것.
- **종단 모드에선 낙하 관측 불가:** release_enabled/release_terminal은 발화 스텝에 종단 →
  reset이 페이로드를 즉시 회수. 물리 착탄 측정은 에피소드가 지속되는 모드에서만
  (`payload_impact_rate` < `release_rate`는 버그가 아니라 이 구조의 서명).

## 학습 검증 (exp_020, 2026-07-23)

hover-drop parity(exp_019)에 이어 **학습 스케일 확증**: B0 warm-start + 보상 bit-match에서
`physical_payload=True`가 유일한 델타인 400-iter 학습 → det 200-ep **success/release
100.00%, drop err 0.169 m** — release_rate 첫 롤아웃부터 100% 고정(재학습 과도기 없음).
**kinematic weld는 학습에도 무비용**(설계 예측대로 드론 동역학 무접촉). 처리량 ~3.1 s/iter
(2048 envs, 100 Hz follow-write 포함). → [[experiments/exp_020_o5jn9xzk_payload_training]]

## 적용 규칙

→ [[research/rl_rules]] **Rule 24**: per-env 동적 결합/분리는 조인트가 아니라 kinematic
weld(write 중단 = 분리)로 구현하고, 물리 경로와 해석적 경로의 parity를 계측으로 증명한 후
전환하라. (exp_020 부기: parity가 증명된 kinematic weld는 학습 파이프라인에도 무비용으로
부착 가능 — 단, 종단 모드에선 `payload_impact_rate=0`이 정상 서명임을 지표 해석 시 유의.)

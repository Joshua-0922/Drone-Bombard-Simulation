---
date: 2026-07-21
tags: [isaac-lab, payload, release, physics, validation]
status: done
type: experiment
wandb_run: N/A (검증 전용 — 학습 없음)
---

# exp_019 — 물리 페이로드 attach/detach (kinematic weld) 구현·검증

> **목표:** 해석적(analytic) 페이로드를 실제 per-env RigidObject로 교체 — 드론에 물리적으로
> 부착(attach)하고 CCIP 발화 시 분리(detach)해 실 낙하·착탄을 시뮬레이션.
> **판정: ✅ 4/4 PASS** — 부착 추적 1.1 mm, 분리 정확, 8/8 착탄, **측정 착탄오차 = 해석적
> CCIP 예측과 cm-parity(mean 0.012 m / max 0.021 m)**.

## 1. 배경 — 코드 전수 검토에서 발견한 결함 (2026-07-21)

사용자 요청("페이로드를 실제로 달고 CCIP 근접 시 분리")으로 `isaac_lab/` 전수 검토:

| # | 결함 | 위치 |
|---|------|------|
| 1 | **페이로드가 물리적으로 존재하지 않음** — 질량은 스폰 시 드론 body에 융합(`loaded_mass` 2.17 kg), 투하는 순수 해석식 | `_author_body_mass_props` |
| 2 | **릴리스 로직이 분리를 안 함** — `_payload_attached`가 init/reset에서 True로만 설정되는 죽은 플래그(발화 시 False 전환 부재) | `_evaluate_release`/`_evaluate_scripted_release_metric` |
| 3 | 마커 attach 판정이 **env 0 전용**(`_payload_attached[0]`으로 전 env 게이팅) | `_update_markers` |
| 4 | `_ctrl_mass`/`_max_thrust`가 스칼라 고정(분리 후 질량 변화 미반영) | `__init__` |
| 5 | 릴리스=즉시 종단이라 물리 낙하 관측 불가(release_enabled/release_terminal 공통) | `_get_dones` |
| 6 | CCIP가 vz 항 생략(`t=sqrt(2H/g)` 특수화) — 물리 페이로드는 실제 vz 상속 | `time_to_fall` |

이번 구현 범위 = #1·#2·#3 (+ 측정 착탄 지표). #4·#5·#6은 §5 후속.

## 2. 구현 — kinematic weld 패턴 (커밋 참조)

**제약:** GPU-복제 PhysX는 per-env 조인트 생성/제거(토폴로지 변경) 불가 → 조인트 방식 불가능.

- **부착:** per-env `RigidObject` 실린더(r 0.05/h 0.06/0.1 kg, `payload_cylinder` SDF 동일)를
  매 physics step(100 Hz) pose+velocity write로 드론에 용접(world-frame 마운트 오프셋
  `payload_mount_offset_z=-0.14`). 조인트 없음 → 복제 물리 무접촉.
- **분리:** CCIP 발화(양쪽 referee 공통)가 `_detach_countdown`을 arm →
  `release_delay`(0.1 s = 10 physics steps, 해석식이 이미 캐리로 모델링하는 기계 지연) 후
  follow-write 중단 → 물리가 자유낙하 적분. `_payload_attached` 발화 시 False 전환(#2 수정).
- **착탄 측정:** 분리 후 z ≤ 0.10 m 최초 도달 시 `|착탄_xy − target_xy|` 래치 —
  `Episode_Metric/payload_impact_rate`·`payload_impact_err_measured_m` 신설(해석적
  `drop_impact_error_m`와 병행), `play.py --policy` 출력 추가.
- **질량 정책(의도적):** 드론 authored 질량은 `loaded_mass` 2.17 kg 유지 — 검증된 exp_014
  plant 불변(Rule 19: 런타임 물리 오버라이드 금지). weld는 kinematic이라 부착 중 드론이
  페이로드 하중을 조인트로 느끼지 않음. 비용 = **분리 후에만** ~0.1 kg 팬텀(현 커리큘럼은
  릴리스=종단이라 미발현).
- `physical_payload=False` cfg로 구 해석적 경로 보존(record_episode.py 호환).

## 3. 검증 — hover-drop 강제 릴리스 (`_test_payload_drop.py`, isaac-verify/L4)

8 envs, Phase-1 기본(metric-only — 에피소드가 발화 후 지속되는 유일 모드), zero-action 호버:

| 체크 | 결과 | 판정 |
|------|------|------|
| 1. ATTACH: 1 s 호버 중 추적 오차 | max **0.0011 m** | ✅ |
| 2. DETACH: 강제 arm 후 잔류 부착 | **0/8** (지연 1 policy step) | ✅ |
| 3. FALL/IMPACT: 착탄 래치 | **8/8** | ✅ |
| 4. PARITY: \|측정 − 해석적\| | **mean 0.012 / max 0.021 m** | ✅ |

Phase 1은 drag/wind-free → 물리 낙하가 해석식과 일치해야 하며, 실제로 cm 수준 일치.
per-env 절대 오차 3.3–6.4 m는 조준 없는 강제 호버-드롭이므로 무의미(파리티가 판정 지표).

## 4. 의미

- 해석적 referee(보상/종단의 진실 소스)와 물리 페이로드(시각·검증·향후 실착탄)가 **동일
  탄도를 재현함을 계측으로 증명** — 이후 Phase 2+에서 referee를 물리 착탄으로 교체할 때의
  기준선.
- 보상·종단·referee는 **bit-identical 유지**(순수 추가 변경) — 기존 체크포인트/실험과 비교
  가능성 보존.

## 5. 후속 (별도 지시 대기)

1. **에피소드를 착탄까지 연장**(#5): 발화 시 즉시 종단 대신 페이로드 착탄에서 종단·터미널
   보상 지급 → 측정 착탄오차를 보상 신호로 사용 가능. 종단 의미론 변경 = fresh 학습 필요.
2. **Phase-2 DR 정합**: drag/wind가 해석식에만 적용됨 — 물리 페이로드 낙하에 동일 drag/wind
   힘 적용 없으면 Phase 2에서 측정≠해석 갈라짐(현재는 Phase 1이라 미발현).
3. **CCIP vz 항 복원**(#6): `t=(vz+sqrt(vz²+2gH))/g` 풀 공식.
4. **분리 후 질량**(#4): per-env `_ctrl_mass` 텐서화 또는 팬텀 0.1 kg 보상력.

## 관련

- [[research/physical_payload_attach]] — 메커니즘·제약·규칙 (Rule 24)
- [[experiments/exp_018_release_terminal]] — 릴리스-종단 구조(Stage B), 본 구현의 전제
- [[research/ccip_release_decoupling]] — 근접≠릴리스, CCIP referee 의미론
- [[experiments/training_history]] / [[research/rl_rules]]

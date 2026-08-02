---
date: 2026-07-15
tags: [research, guide, isaac, v11, v12, v13, migration, design]
type: research
status: active
owner: junsang
---

# Isaac v11~v13 설계 가이드 — 기존 migration 모델과 무엇이 달라졌나

> **목적:** feat/isaac-env-migration의 기존 `DroneBombardEnv`(3-phase 커리큘럼, vision, scripted
> drop)와, 이번에 만든 **완화-후-단계확장** 모델(v11→v12→v13, `Issac_JS` 브랜치)의 **차이**를
> 한눈에 정리. "왜 뺐고, 어떻게 다시 켜는가"가 핵심.

관련: [[experiments/exp_006_v11_dryrun_junsang]] · [[experiments/exp_007_v12_random_marker_junsang]] · [[experiments/exp_008_v13_partial_obs_junsang]] · [[research/isaac_cruise_handoff_junsang]]

---

## 0. 한 줄 요약

기존 모델은 **"인지(vision) + 3단계 커리큘럼 + 자동(scripted) 드롭"** 을 한꺼번에 담았다.
신규 모델은 이를 **완화(v11)** 해서 제어·조준·투하만 격리 검증한 뒤, **삭제하지 않고 토글로
한 축씩 다시 켜며**(v12 랜덤 marker → v13 부분관측) 확장한다. **억제한 기능은 전부 inert
hook으로 보존** → 나중에 플래그 하나로 복원.

---

## 1. 핵심 변경 (기존 base vs v11 완화)

| 축 | 기존 base (`DroneBombardEnv`, migration) | v11 완화 (신규) |
|----|------------------------------------------|-----------------|
| **커리큘럼** | **3-phase** (P1 접근 → P2 CCIP residual+DR → P3 이동타겟), phase에서 플래그 파생 | **단일 통합 phase** (phase=1 고정, 모든 확장 플래그 off) |
| **드롭 트리거** | **scripted CCIP referee 자동 발사** (`_advance_phase_dynamics`, 예측 착탄이 맞으면 fire) | **정책 drop_signal** — `action[4]>0.5` (C=A). 언제 떨어뜨릴지를 정책이 학습 |
| **타겟 인지** | **vision** (핀홀 카메라 u,v,conf, footprint 제한) | **완벽 타겟 obs** (marker 상대위치·CCIP 직접 제공, B=vision 제거) |
| **obs 차원** | **14D** (vision 포함) | **24D** (vision 제거, marker/CCIP/kinematics) |
| **action[4:6]** | CCIP **residual** (Phase 2 보정) | `[4]`=drop_signal, `[5]`=reserved (residual hook 보존) |
| **타겟/스폰** | 랜덤 타겟 + 랜덤 핸드오프(반경/각도), **거의 정지** spawn | **고정 marker 정면 20m** + **cruise 핸드오프(실제 4 m/s로 순항 중 spawn)** (A=실제) |
| **DR / wind** | Phase 2+ 활성 | **off** (nominal, `dr_enabled=False` hook) |
| **이동 타겟** | Phase 3 활성 | **static** (`moving_target_enabled=False` hook) |
| **종료** | 근접/release + 실패조건 | **release-terminal + 착탄 결과** (투하 순간 종료) |

### 왜 이렇게 바꿨나
- **인지·커리큘럼·자동드롭이 섞이면** 무엇이 학습을 막는지 분리 불가 → 먼저 **제어+조준+투하**만 남겨 "Isaac에서 학습이 되긴 하는가"부터 확인.
- **C=A(정책 드롭)**: scripted referee는 "정책이 스스로 투하 타이밍을 배운다"는 목표와 안 맞음 → drop_signal로 전환.
- **삭제 없이 토글**: 확장할 때 코드 재작성 없이 플래그만.

---

## 2. 단계적 확장 (v11 → v12 → v13)

| 버전 | 무엇을 켰나 | 스폰/인지 | obs | 결과 |
|------|-----------|-----------|-----|------|
| **v11** | 완화 baseline | 고정 marker 20m · 완벽 obs | 24D | ✅ success 100%, 착탄 0.43m |
| **v12** | **랜덤 marker** (`marker_random`) | (20,0) 중심 **5m 원** 면적균일 · 완벽 obs | 24D | ✅ success 100%, 착탄 ~0.72m |
| **v13** | **부분관측 reveal** | 랜덤 marker · **blind → 수평 7m 진입 시 공개** | **25D** (+detected) | ✅ success 100%, 착탄 ~0.8m |

**v12 (일반화):** 드롭이 "한 지오메트리 암기"가 아닌지 검증. 드론은 여전히 +X 순항, marker가 옆(±5m)이면 **조향**해서 정렬.

**v13 (부분관측 = vision 징검다리):** 드론이 marker를 **모른 채 blind 순항** → 근접(7m)해야 위치가 열림. 설계 포인트 3가지:
1. **연속(비latch)** — 7m 벗어나면 다시 정보 끊김.
2. **미탐지 페널티** (`-0.2/step`) — 재진입·접촉 유지 강제.
3. **보상 게이팅** — marker 의존 보상(progress·CCIP·gate·drop)을 **탐지 시에만** 지급 → 보상 gradient가 숨은 위치를 **누출하지 않게** (진짜 blind 보장). reward가 v11/v12(~314)보다 낮은 ~290인 건 이 blind 페널티 때문.

---

## 3. 발견한 버그 (신규 스폰 방식이 노출)

**cruise 핸드오프 자세 폭주 (step-1 즉사):** 기존은 정지 근처 spawn이라 없던 문제. v11은 4 m/s로
순항하며 spawn하는데 `super()._reset_idx`가 속도 컨트롤러(`_v_filt`, `_prev_action`)를 0으로
리셋 → 첫 스텝 큰 속도오차 → 급격 tilt → `bad_attitude` 즉사(episode length 1).
**수정:** reset에서 컨트롤러를 cruise setpoint로 seed. → [[research/rl_rules]] Rule 10, [[research/isaac_cruise_handoff_junsang]]

---

## 4. 아직 "꺼둔" 것 (inert hook → 다음 확장)

| 기능 | 현재 | 켜는 법 | 주의 |
|------|------|---------|------|
| **진짜 vision** | 완벽 obs / v13는 거리게이트 | `_update_vision`(핀홀 카메라 u,v,conf) obs 연결 | down-camera라 footprint ≈ 고도×0.58 → "안 보이는 타겟 접근" 과제로 급증 (가장 큰 축) |
| **DR / wind** | off | `dr_enabled`(phase≥2 or 플래그) | **wind trap**: obs에 바람 없음 → residual 없이는 보정 불가 → residual과 함께 |
| **CCIP residual** | `action[5]` reserved | residual_enabled | DR와 짝 |
| **이동 타겟** | static | `moving_target_enabled` | Phase 3 |
| **커리큘럼** | 단일 | `--phases 1,2,3` | 통합이 되면 불필요할 수도 |

---

## 5. 코드 위치 (Issac_JS)

- `isaac_lab/drone_bombard/v11_env.py` — `DroneBombardV11Env/Cfg`(완화), `V12Cfg`(marker_random), `V13Env/Cfg`(부분관측)
- `isaac_lab/drone_bombard/__init__.py` — 태스크 `Isaac-DroneBombard-V11/V12/V13-Direct-v0`
- `isaac_lab/train.py` — `--v11_test` / `--v12` / `--v13`
- `isaac_lab/drone_bombard/drone_bombard_env.py` — **기존 base**(참조·미수정), vision/CCIP referee/커리큘럼 원본

---

## 관련 노트

- [[experiments/exp_006_v11_dryrun_junsang]] · [[experiments/exp_007_v12_random_marker_junsang]] · [[experiments/exp_008_v13_partial_obs_junsang]]
- [[research/isaac_cruise_handoff_junsang]] · [[research/rl_rules]] · [[experiments/training_history]] · [[00_index]]

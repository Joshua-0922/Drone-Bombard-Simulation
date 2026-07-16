---
date: 2026-07-15
tags: [experiment, isaac, v13, expansion, partial-observability, reveal, vision-proxy, drop-signal]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-15_*_v13_dryrun)"
owner: junsang
---

# Exp 008 — Isaac v13: 부분관측(blind cruise → 7m 탐지) 학습

> **목적:** vision의 "가까워야 보인다"를 **거리 게이트로 단순화**한 partial-observability 과제.
> 드론이 marker 위치를 **모른 채 순항하다가, 근접(7m)했을 때만 위치가 열리는** 상황에서
> 조준·투하를 학습할 수 있는지 (vision 복원 전 징검다리).
> **출처:** branch `Issac_JS`, `DroneBombardV13Env/Cfg`, [[experiments/exp_007_v12_random_marker_junsang]]

---

## 설계 (v12 + reveal)

| 항목 | 값 |
|------|-----|
| marker | v12와 동일 (20,0 중심 5m disk 랜덤) |
| 초기 상태 | **blind** — marker 상대위치·CCIP·d_impact obs 채널 **마스킹(0)** |
| 탐지 조건 | **수평 d_xy ≤ 7m** (반경 7m = disk 5m + 여유 2m) |
| latch? | **아니오 (연속)** — 벗어나면 다시 마스킹 |
| obs | **25D** (24 + `detected` 플래그) |
| 미탐지 페널티 | **-0.2 / step** (탐지 획득·유지 강제) |
| 보상 게이팅 | marker 의존 보상(progress·CCIP·gate·drop_signal) **전부 detected 시에만** → 보상이 숨은 위치 누출 안 함 |
| 나머지 | drop_signal·envelope·cruise 핸드오프·물리 = v11 동일 |

- `DroneBombardV13Env(DroneBombardV11Env)`: `_get_observations`(마스킹+플래그), `_get_rewards`(게이팅+페널티) override
- 태스크 `Isaac-DroneBombard-V13-Direct-v0`, `train.py --v13`
- 로컬: py_compile OK, pytest 45 passed, 마스킹/reveal 게이트/페널티 로직 sanity 통과

---

## 결과 (VM L4)

| 단계 | 설정 | 결과 |
|------|------|------|
| smoke | 16env / 3iter | ✅ 25D obs 구성·마스킹 동작·에피소드 ~20스텝·EXIT 0 (shape mismatch 없음) |
| **dry-run** | **512env / 300iter** | ✅ **~iter 48 수렴 시작** |

**수렴 지표 (iter 115):**
- `success` = **1.00 (100%)**
- `release_rate` = **1.00 (100%)**
- `drop_impact_error_terminal` = **~0.8 m**
- `Mean reward` = **~290** (iter48 208 → iter115 290 상승)

→ **부분관측 과제 학습 성공.** 드론이 marker를 모른 채 +X blind 순항 → 7m 진입 탐지 → 머무르며 하강·조준·투하 → 100% 성공. reward가 v11/v12(~314)보다 낮은 ~290은 **blind 순항 구간의 미탐지 페널티**(불가피) 때문. 208→290 상승 = 탐지를 더 빨리 획득하도록 학습.

---

## 관찰 / 다음

- 학습 곡선 노이즈가 v11/v12보다 큼(초기 0.67~1.0 진동) — 과제 난이도↑ 반영.
- 다음 확장 후보:
  - [ ] reveal 반경↓(5m 등)으로 난이도↑ / 미탐지 페널티 튜닝
  - [ ] **진짜 vision**(핀홀 카메라 u,v,conf)로 교체 — footprint가 고도의존·down camera (`_update_vision` hook) → [[research/isaac_cruise_handoff_junsang]] 인근
  - [ ] wind/DR (+ residual), 이동 타겟
- 관련: [[experiments/exp_007_v12_random_marker_junsang]], [[experiments/exp_006_v11_dryrun_junsang]], [[experiments/training_history]]

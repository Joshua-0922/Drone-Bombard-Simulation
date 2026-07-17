---
date: 2026-07-16
tags: [experiment, isaac, v17, pixel-vision, quantization, partial-observability, perception]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-16_*_v17_dryrun)"
owner: junsang
---

# Exp 012 — Isaac v17: 픽셀 양자화 vision (대략 위치 → 접근하며 정밀화)

> **목적:** v13(정확한 위치 reveal)을 더 어렵게 — 비전이 **위치를 픽셀 셀 단위로 "대략"** 알려주고,
> **멀면 애매·가까우면 정밀**하게 만든다. 드론은 애매한 셀 중심으로 대충 접근하다가, 가까워지며
> 인지가 선명해져 정밀 투하해야 함. (실제 카메라의 "가까워야 잘 보인다"를 셀 양자화로 근사)
> **출처:** branch `Issac_JS`, `DroneBombardV17Env/Cfg`, [[experiments/exp_008_v13_partial_obs_junsang]]

---

## 설계 (v13 기반 — 물리 drop v16과 별개 축)

| 요소 | 방식 |
|------|------|
| 탐지 게이트 | v13 재사용: 수평 d_xy ≤ 7m ON, 밖이면 마스킹+페널티(−0.2/step) |
| **픽셀 양자화** | 인지 위치 = 실제 타겟이 속한 **셀의 중심**으로 스냅 (`(floor(rel/cell)+0.5)·cell`) |
| **셀 크기 ∝ 거리** | `cell = 0.15 · slant`(드론→타겟 3D거리) → 멀면 큰 셀(애매), 가까우면 작은 셀(정밀) |
| 인지 범위 | 정책은 **양자화 타겟만** 인지: obs(rel·CCIP)·release 게이트 전부 양자화값 사용 |
| success | **실제 타겟으로 판정** (애매한 조준을 접근으로 다듬어야 함) |
| 토글 | `pixel_vision_enabled` (False = v13 정확 reveal) |

**구현:** `_ccip`를 양자화 타겟 기준으로 override(obs CCIP·게이트·보상 shaping 전파) + obs rel 채널(0,1)을 양자화 오프셋으로 덮어씀. v13 reveal/페널티/detected 플래그 상속.

### 양자화 곡선 (로컬 검증)
| 수평거리·고도 | 셀 크기 | 인지오차 |
|------|------|------|
| 7m·10m | 1.83m | 1.09m (애매, success 반경 1.0m 초과) |
| 5m·8m | 1.42m | 0.71m |
| 3m·5m | 0.87m | 0.44m |
| 1m·4m | 0.62m | 0.32m |
→ 다가갈수록 셀·오차 단조 감소. 7m 첫 탐지 땐 애매해 정확히 못 맞춤 → **접근해야 정밀 투하**.

---

## 결과 (VM L4)

| 단계 | 설정 | 결과 |
|------|------|------|
| smoke | 16env / 3iter | ✅ 25D obs 구성·EXIT 0 (에러 없음) |
| **dry-run** | 512env / 300iter | ✅ iter48 success 0.83 → iter114 **success 0.67~0.83·release 100%·착탄 ~0.75m·reward 250** |

→ **픽셀로 대략 위치만 알아도 학습 성공.** 드론이 애매한 셀 중심으로 접근 → 가까워지며 인지 선명 →
정밀 투하. 픽셀 애매성에도 v13 수준(~sub-meter)으로 수렴. 창별 노이즈(0.67~0.83)는 과제 난이도 반영.

---

## 다음 / 관찰
- 더 어렵게: `pixel_cell_k`↑(더 애매), 셀 경계 노이즈, 오탐지 등.
- v16(물리 drop)과 결합하면 "픽셀 인지 + 실제 낙하" 통합 가능.
- 관련: [[experiments/exp_008_v13_partial_obs_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[experiments/training_history]]

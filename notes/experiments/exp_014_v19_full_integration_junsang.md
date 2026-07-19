---
date: 2026-07-19
tags: [experiment, isaac, v19, integration, physical-drop, warm-start, curriculum]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-19_*_v19_warmstart)"
owner: junsang
---

# Exp 014 — Isaac v19: 전체 통합 (v18 + 실제 물리 drop)

> **목적:** 단계적 통합 3단계 — v18(인지+물리) 위에 **v16의 실제 물리 payload 낙하**를 합쳐
> "모든 능력 + 실제로 떨어지는 payload"를 한 모델로. (v18 라이브에서 payload가 안 날아간 이유 =
> analytic drop이었기 때문 → v19에서 진짜 낙하)
> **출처:** branch `Issac_JS`, `DroneBombardV19Env/Cfg`, [[experiments/exp_013_v18_integration_curriculum_junsang]] · [[experiments/exp_011_v16_physical_drop_junsang]]

---

## 설계

`DroneBombardV19Env(DroneBombardV18Env)`:
- **유지 (v18)**: `_ccip`(픽셀 양자화 인지 + residual 보정), obs 28D, action 7D
- **교체 (v16)**: analytic release-terminal → **물리 land-terminal**
  - drop 발화 → RigidObject payload 분리 → **바람 속 자유낙하**(궤적 휨) → 지면 착지
  - 투하 후 드론 호버, 에피소드는 **payload 착지 시 종료**
  - success/보상 = **실제 착지 지점** (수식 아님)
- 보상 = v13 인지(미탐지 페널티·detected 게이팅, 투하 후 shaping 동결) + v16 실제착탄 터미널
- **obs/action이 v18과 동일** → v18 체크포인트로 **warm-start 가능** (조준·투하 정책 전이)

---

## 결과 (v18 Phase2 모델 warm-start)

| iter | success | release | 실제 물리 착탄 |
|------|---------|---------|------|
| 331 (초기) | 0 | **1.00** | 0.67m |
| 375 | **1.00** | 1.00 | 0.56m |

- **warm-start 덕에 release 처음부터 100%** (조준/투하 정책 물려받음) → 데드락 없음
- **~iter75만에 수렴**: success 1.0, **실제 물리 착탄 0.56~0.67m**
- payload가 **실제 RigidObject로 바람 속을 낙하해 타겟 명중** — 통합 완성

⚠️ 창별 노이즈 큼(물리 drop이 analytic보다 변동↑, 착탄이 1m 경계 근처라 success 진동). 다중시드·추가학습으로 견고화 여지.

---

## 시각화 (라이브스트림)
v19를 `play.py --show --livestream 1`로 시청 → 상세 [[research/isaac_viz_tools_junsang]]. 요약: payload가 드론에서 분리·낙하하는 걸 보이게 하려고 카메라 근접·타겟 비콘·**큰 노란 하이라이트 구슬**(실물 10cm는 너무 작아 안 보임) 등 반복 조정.

## 다음 / 백업
- v19 체크포인트: 노트북 `~/v18_backup/v19_model.pt` + VM js-v11 로그. → [[research/isaac_v18_curriculum_continuation_junsang]]
- 다음: 다중시드 견고화 / 이동 타겟 / sim-to-real / 시각화 마무리.
- 관련: [[experiments/exp_013_v18_integration_curriculum_junsang]] · [[experiments/training_history]]

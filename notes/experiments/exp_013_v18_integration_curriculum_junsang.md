---
date: 2026-07-18
tags: [experiment, isaac, v18, integration, curriculum, warm-start, residual-deadlock]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-18_*_v18_eased / v18_phase2)"
owner: junsang
---

# Exp 013 — Isaac v18: 능력 통합 (인지+물리) + 커리큘럼으로 데드락 해결

> **목적:** 지금까지 격리 검증한 능력들을 **한 모델로 통합** (단계적 통합 1단계 = 인지+물리).
> 랜덤타겟·blind탐색·픽셀인지(v17) + DR·CCIP residual·기체바람(v14/v15)을 동시에.
> **출처:** branch `Issac_JS`, `DroneBombardV18Env/Cfg`, [[experiments/exp_012_v17_pixel_vision_junsang]] · [[experiments/exp_010_v15_airframe_wind_junsang]]

---

## 설계 (통합 env)

`DroneBombardV18Env(DroneBombardV14Env)` — 물리 파이프라인(v14 reset DR·residual·wind obs, v15 기체바람 base 훅) 상속 + 인지(v13 reveal·페널티·detected 플래그, v17 픽셀 양자화) 추가.
- `_ccip`: **residual 보정 예측 vs 픽셀 양자화된 인지 타겟** (두 축 조합)
- obs **28D** (24 + wind/drag 3 + detected 1), action **7D**, drop은 analytic

---

## 🐞 핵심 발견 — 통합 시 "투하 데드락"

**증상:** from-scratch로 다 켜니 300 iter까지 **release_rate 0·success 0** (투하 자체를 못 함).

**원인:** release 게이트는 `|nominal + residual − 인지타겟| ≤ 1m`일 때만 열림. **학습 초기 residual(action[5:7])은 랜덤 ±3m** → CCIP가 항상 1m 초과 → 게이트 안 열림 → **투하 못 함 → drop 보상 못 받음 → residual 학습 불가 → 영원히 랜덤** (닭-달걀 데드락). 애매한 픽셀 인지(~1m)가 악화.

**확정:** residual을 강제 OFF → release 0→**100%** 로 뛴 것으로 원인 확정.

→ 규칙화: [[research/rl_rules]] Rule 12 (통합엔 성공경험 부트스트랩이 관건).

---

## 해결 — 2단계 커리큘럼

### Phase 1 (완화, from scratch)
게이트/residual/바람/픽셀을 완화해 **투하를 부트스트랩**:
| 파라미터 | 완화 |
|------|------|
| release_radius | 1.0 → **1.5** (게이트 열림 확률 9%→44%, 핵심) |
| residual_scale | 3.0 → 2.0 |
| wind_std | 2.0 → 1.5 |
| pixel_cell_k | 0.15 → 0.12 |
→ dry-run: release 0.71(iter36) → **success 1.0·release 100%·착탄 0.53m** (iter222). 데드락 해소.

### Phase 2 (hard, warm-start)
Phase 1 모델(`model_200.pt`)을 `--resume` + **원래 hard 설정 복원**(`--v18_hard`: 게이트 1.0·residual 3.0·바람 2.0·픽셀 0.15):
- **시작부터 success 0.87** (기본기 물려받음, 데드락 없음)
- 최종 **success 1.0 · release 100% · 실제 착탄 ~0.65m** (hard 난이도)

---

## 결론
- **한 모델이 6능력 동시 학습**: 랜덤타겟 blind 탐색 → 애매한 픽셀 인지 → 바람과 싸우며 → residual로 드리프트 보정 → 명중.
- **커리큘럼(쉬움→어려움 warm-start)이 통합 데드락의 정석 해법.** Phase 2는 완화 불필요.
- 다음: 3단계(+물리 drop v16) / 재현 다중시드 / 게이트 다시 조이기.
- 관련: [[research/isaac_v18_curriculum_continuation_junsang]] · [[research/rl_rules]] · [[experiments/training_history]]

---
date: 2026-07-16
tags: [experiment, isaac, v14, ccip-residual, domain-randomization, wind, control-group]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-15_*_v14_dryrun / v14_nores_dryrun)"
owner: junsang
---

# Exp 009 — Isaac v14: DR + CCIP 잔차학습 (Stage A, 바람 관측)

> **목적:** DR(바람/드래그)로 nominal 탄도 예측과 실제 착탄이 어긋날 때, 정책이 **CCIP residual**로
> 그 gap을 보정 학습하는지 검증. wind trap을 **바람을 obs로 제공(Stage A)** 해서 격리.
> **출처:** branch `Issac_JS`, `DroneBombardV14Env/Cfg`, [[research/isaac_expansion_roadmap_junsang]]

---

## 설계 (v12 기반 — 인지문제와 분리)

| 항목 | 값 |
|------|-----|
| DR | 에피소드마다 wind N(0,**1.0**)·drag U[0,0.15] |
| action | **7D**: [0:4]속도 · [4]drop_signal · **[5:7] residual(±3m)** |
| obs | **27D** = v12 24D + wind_xy(2)+drag(1) → 정책이 바람→residual 매핑 학습 |
| residual 적용 | 보정착탄 = nominal + residual. **게이트/조준은 보정값**, **착탄 보상은 실제(drag/wind)** |
| 대조군 | `--v14_no_residual` (DR/obs/action 동일, residual만 미적용) |

로컬 사전검증: drift 최대 1.92m(wind 1.5) < residual 3m, residual=drift 시 보정착탄==실제착탄(오차 0).

---

## 결과 — residual ON vs OFF (마지막 50 iter 평균, 250~299)

| 지표 | residual ON | residual OFF | 차이 |
|------|-------------|--------------|------|
| **착탄오차** | **0.694 m** | 0.823 m | −0.13m (ON 우세) |
| success | **81.5%** | 72.2% | +9.3%p |
| release | 100% | 94.6% | +5.4%p |
| reward | 276.4 | 248.0 | +28.4 |

**residual이 전 지표에서 일관 우세.** 다만 격차가 예상(drift 1.9m)보다 작음.

### 핵심 해석 (중요)
격차가 작은 이유 = **v14에선 바람이 payload 탄도에만 작용하고 드론 기체엔 안 미침.** 그래서 대조군도
**obs로 바람을 보고 "비행 자체를 조정"**(바람 세면 더 낮게·느리게 투하 → 드리프트↓)해서 0.82m를 냄.
→ 결론: **"residual 없으면 불가"가 아니라 "정책 자체 적응 위에 추가 이득"**. 이 한계가 v15(바람을 기체에 작용)로 이어짐. → [[experiments/exp_010_v15_airframe_wind_junsang]]

### 한계
- **시드 1개씩** — success 9.3%p 차이는 일부 노이즈 가능. 엄밀하려면 다중 시드.
- 두 런 GPU 공유(동시 실행) — 최종 지표엔 무영향.

---

## 다음
- v15: 바람을 **드론 기체에 실제 작용**시켜 "비행 보정" 경로 차단 → residual 진짜 가치 측정.
- 관련: [[experiments/exp_007_v12_random_marker_junsang]] · [[research/isaac_expansion_roadmap_junsang]] · [[experiments/training_history]]

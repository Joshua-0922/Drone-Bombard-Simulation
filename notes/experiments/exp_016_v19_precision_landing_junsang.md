---
date: 2026-07-23
tags: [experiment, isaac, v19, reward, precision, landing, potential-shaping, warm-start]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-23_07-58-11_v19_precise)"
owner: junsang
---

# Exp 016 — Isaac v19 정밀도 향상: 연속 착지보상

> **목적:** exp_015(붕괴수정) best(iter599, 착탄 0.563m)에서 **정밀도를 더 높이기**. 착탄 정체의
> 원인=성공존 내부 평평보상 → 연속 착지보상으로 교체 + best warm-start 이어학습.
> **출처:** branch `Issac_JS` `09a53a2`. → [[experiments/exp_015_v19_abd_retrain_junsang]] · [[research/rl_rules]] (Rule 13)

---

## 처방 (패치 1 + 3)
**패치 1 — 연속 착지보상** (`cfg.precise_landing_reward=True`, 토글):
- 기존: `err<=success_radius(1.0)`이면 **무조건 reward_success(300)** → 0.1m·0.9m 동일 → 정밀 유인 없음(med_err 0.56m 정체).
- 신규: $300\,e^{-k_{land}\cdot err} + \mathbf{1}[err\le1.0]\cdot bonus$ — **0m까지 계속 당기는 연속 보상** + 성공 문턱 이산 보너스.
- `v19_k_landing=2.0`(가파른 sub-meter 기울기), `v19_success_bonus=100`. 기존 평평형은 토글 뒤 보존.

**패치 3 — 이어학습:** exp_015 best(`v19_best.pt`=iter599) warm-start → 새 보상에 이어학습(iter599→899, 1024 envs). dry-run(3iter) 선통과.

## 결과 — ✅ 정밀도 대폭 향상
`select_best_checkpoint.py` 전 체크포인트 평가 (best=iter875):

| | exp_015 (A+B+D) | **exp_016 (정밀 패치)** |
|---|---|---|
| success | 76.7% | **100%** |
| release | 96% | **100%** |
| **med_err** | **0.563m** | **0.356m** (37%↓) |

- 표 전체 med_err가 0.56~0.72 → **0.32~0.45대로 하락**. iter875에서 success·release 100%·착탄 0.356m.
- **패치 1이 정확히 먹힘**: 성공존 내부 평평보상 제거 → med_err 정체 해소, 덤으로 success도 100%.
- warm-start 직후 release 잠깐 dip(iter599~600 0)→즉시 회복(재적응, Rule 13과 동일 패턴).

## 백업 & 라이브
- best=`model_875.pt`→`model_best.pt`→`v19_precise_best.pt`. 노트북 `~/v19_backup/v19_precise_run/`(15 ckpt), VM `~/v19_precise_backup/run/`.
- 라이브스트림(payload 추적)으로 정밀 명중 시청 확인. → [[research/isaac_viz_tools_junsang]]

## 다음
- 더 정밀화 여지: release_radius 1.5→1.0 커리큘럼, k_landing↑, 다중시드 견고화.
- 관련: [[experiments/exp_015_v19_abd_retrain_junsang]] · [[experiments/training_history]] · [[00_index]]

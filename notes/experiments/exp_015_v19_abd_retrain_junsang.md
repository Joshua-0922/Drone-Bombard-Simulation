---
date: 2026-07-23
tags: [experiment, isaac, v19, reward, collapse-fix, potential-shaping, warm-start, best-checkpoint]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-23_02-31-51_v19_abd_full)"
owner: junsang
---

# Exp 015 — Isaac v19 재학습: no-drop 붕괴 수정 (A+B+D)

> **목적:** v19 체크포인트가 수렴 후 **투하를 아예 안 하도록 붕괴**(iter499 release 0%)한 것을
> 진단→수정→재학습으로 복구. 원인=상주 CCIP shaping 수확이 노이즈 큰 물리 투하보다 이득.
> **출처:** branch `Issac_JS` `2f2bf9b`. → [[research/isaac_v19_collapse_nodrop_junsang]] · [[research/rl_rules]] (Rule 13)

---

## 처방
- **A — 포텐셜(차분) shaping:** CCIP 조준 보상을 $w_c e^{-k d}$(상주) → $w_c(e^{-k d_t}-e^{-k d_{t-1}})$(차분).
  개선 시에만 보상, 유지 시 0 → "호버 수확" 인센티브 제거. `cfg.ccip_potential_shaping`.
- **B — 인엔벨로프 미투하 누진 페널티:** $-w_{loiter}\cdot(\text{gate 열린 채 안 던진 연속 스텝수})$.
  서성일수록 비용↑ → 탈출구는 투하뿐. `cfg.v19_w_loiter=0.02`, `_gate_steps` 카운터.
- **D — best-checkpoint 저장:** `save_interval 50→25`; `select_best_checkpoint.py`가 model_*.pt를
  훑어 실제착탄 success 기준 best를 `model_best.pt`로 복사(Isaac 1회 부팅, 가중치만 교체).

## 설정
v18_phase2_model300 warm-start → `--v19 --resume ... --num_envs 1024 --max_iterations 300`
(iter300→600), seed 42, tensorboard. dry-run(3 iter) 선통과: release 1.0, 학습 안 깨짐.

## 결과 — ✅ 붕괴 방지 + 정밀도 향상

| iter | success | release | med_err(m) |
|---|---|---|---|
| 300 (시작) | 8.7% | 98.7% | 1.04 |
| 350~450 | **0%** | 0~6% | 재적응 딥 |
| 500 | 34% | 96% | 0.94 |
| 525 | 41% | 96.8% | 0.93 |
| 550 | 61% | 99.3% | 0.72 |
| 575 | 70% | 98% | 0.64 |
| **599 (best)** | **76.7%** | **96%** | **0.563** |

- **붕괴 방지 확정:** iter499 넘어 **iter600까지 release 96~100% 유지**(옛 붕괴는 0). A가 핵심.
- **정밀도:** success 0%(붕괴)/8%(v18) → **76.7%**, 착탄 **0.563m**(유실된 옛 v19 best 0.56m 복원 + success↑).
- **재적응 딥(350~450):** 새 보상 적응 중 release 일시 0 → 회복. **딥에서 조기종료 금지**, D로 peak 포착 (D 필요성 실증).

## 백업 & 라이브
- best=`model_599.pt`→`model_best.pt`. 노트북 `~/v19_backup/v19_abd_run/`(15 ckpt), VM `~/v19_abd_backup/run/`.
- `play.py --policy v19_best.pt --show --livestream 1`(payload 추적)로 명중 시청 확인. → [[research/isaac_viz_tools_junsang]]

## 관련
[[experiments/exp_014_v19_full_integration_junsang]] · [[experiments/training_history]] · [[00_index]]

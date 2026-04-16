---
date: 2026-04-16
tags: [experiment, dry-run, rtf, fps, sim-speed]
status: completed
type: experiment
wandb_run: "mtx7ud6o / x8jq9fsy / u8w3xn0w"
---

# Exp 003 — RTF Dry-Run 비교 (RTF 1 / 2 / 4)

> **목적:** `PX4_SIM_SPEED_FACTOR` 설정에 따른 실제 학습 FPS 측정. Exp 002 Full Training에서 최적 RTF 결정.

## 설정

| 항목 | 값 |
|------|----|
| 총 timesteps | 5,500 (에피소드 ~10개) |
| `learning_starts` | 10,000 (SAC 업데이트 없음 — FPS 측정 전용) |
| `max_steps` | 500 |
| WandB project | drone-bombard-sac |

## 결과

| RTF | 2000 steps (fps) | 4000 steps (fps) | **평균 fps** | 4000 steps 소요시간 | WandB run |
|-----|-----------------|-----------------|------------|-------------------|-----------|
| 1 | 37 | 44 | **40.5** | 89s | `mtx7ud6o` — dryrun-RTF1-2026-04-16 |
| 2 | 55 | 64 | **59.5** | 61s | `x8jq9fsy` — dryrun-RTF2-2026-04-16 |
| 4 | 46 | 57 | **51.5** | 70s | `u8w3xn0w` — dryrun-RTF4-2026-04-16 |

## 핵심 발견

**RTF=2가 최적.** RTF=4로 올렸을 때 오히려 fps 역전 발생.

### RTF 향상 효과

- RTF=1 → RTF=2: +47% fps, 벽시계 32% 단축 ✅
- RTF=2 → RTF=4: fps 8% **감소**, 벽시계 오히려 증가 ❌

### RTF=4 역전 원인 분석

시뮬레이션 속도 4x 증가 → Gazebo/PX4 obs publish 빈도 4x 증가
→ Python RL 루프 (`train_sac`)가 obs 처리 병목
→ `obs_wait_timeout=0.02s` 초과 스텝 증가
→ 실질 fps 저하 및 벽시계 시간 역전

결론: **Python side가 bottleneck**. RTF를 무한정 올려도 FPS는 증가하지 않음.

## 결정

**Exp 002 Full Training: RTF=2로 진행.**

- 1M steps 기준 RTF=1 대비 ~30% 벽시계 시간 단축 예상
- 안정성 문제 없음 (RTF=4와 달리 obs 처리 여유 있음)

## 관련 노트

- [[experiments/training_history]]
- [[research/rl_rules]]
- [[experiments/exp_002_reward_shaping_patches]]

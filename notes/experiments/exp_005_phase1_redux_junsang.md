---
date: 2026-06-04
updated: 2026-06-05
tags: [experiment, phase1-redux, curriculum, target-move, random-drop, success-rate]
status: in-progress
type: experiment
wandb_run: "ayi27a56 (v1) / za9zxdh6 (v2)"
owner: junsang
---

# Exp 005 — Phase 1 redux: Task 재설계 + Curriculum 정밀화

> **목적:** Round 7 v3에서 발견한 "정책의 random_drop 의존 + 14.87m 난이도 과다"를 task 자체를 바꿔 정면 돌파.
> **출처:** `local/meeting_notes/meeting_notes_2026-06-05.txt`, `local/design/model_history.md`, `local/issues/master.txt`

---

## 배경 — 왜 redux인가

Phase 1(Round 7 v3) eval 진단:
- deterministic eval에서 0~1 drops/5ep, EP2·EP3 d_xy 정확 일치(episode 의존성)
- best 1.32m은 Representative Best 분석상 **isolated lucky drop** (`not_measurable`)
- 정책이 random_drop 보조에 의존, 14.87m는 학습 난이도 과다

→ Phase 2(단일 파라미터 튜닝)로는 부족. **task 자체를 쉽게 + drop 강제**. ([[research/phase1_summary_junsang]])

---

## redux v1 — Target 이동 + random_drop=0 (`ayi27a56`, fresh, 89k 수동)

### 설정

| 변경 | old → new | 의도 |
|------|-----------|------|
| `target_enu` | (11,10) → **(4,3)** | spawn에서 14.87m → 5m |
| `random_drop_prob` | 0.005 → **0** | 정책이 직접 drop trigger 강제 |
| drop_calculator `x_marker_x` | hardcoded 11.0 → cfg | 버그 fix (target 변경 시 부조화) |
| `x_marker_world.sdf` x_marker_0 | (11,10,0) → (4,3,0) | world와 cfg 일치 |
| `_kill_episode` timeout | 5s → 2s | Phase 2 speedup |

### 결과 (89k 수동 중단) — 대성공

| 지표 | 값 |
|------|-----|
| episodes | 1,205 |
| drops | **830 (모두 auto, random_drop=0 검증)** |
| success | **799 (96.3% at 5m)** |
| best drop | **0.809m** (Round 7 v3의 1.32m 갱신) |
| jackpot | 0 (0.1m 임계 한계) |
| ent_coef | 0.001 (매우 deterministic) |
| critic_loss | 3~7 안정 |

### 발견된 새 문제

- **fps 폭락**: drop이 잦아지자 `_kill_infra` 5s timeout 누적 → fps 18 → 2 (예상 잔여 30시간) → 수동 중단
- 5m success 임계 헐거움, jackpot 0.1m 발화 불가 → tightening 필요

---

## redux v2 — Tight thresholds + curriculum (`za9zxdh6`, resume from 89k, 진행 중)

### 설정 (v1의 89k preempt에서 resume = curriculum)

| 변경 | old → new | 의도 |
|------|-----------|------|
| `auto_drop_threshold` | 3.0 → **1.0m** | 정밀 d_impact만 trigger |
| `success_threshold` | 5.0 → **1.0m** | success 정의 정밀화 |
| `jackpot_threshold` | 0.1 → **0.3m** | 도달 가능 영역, 첫 jackpot 도전 |
| `_kill_infra` timeout | 5s → **2s** | fps 회복 (v1 폭락 처방) |
| (callback) | `current_success_streak` 추가 | 모니터링 (reward 영향 X) |

> **Resume 정당성:** 보상 공식 변경이 아니라 threshold·trigger 조정 → replay buffer stale 영향 작음(100k 후 자연 refresh). 96% 정책(접근 마스터) 위에 정밀화만 추가 학습.

### 검증 목표 (decision tree)

| 목표 | 판정 |
|------|------|
| 새 1m success_rate > 50% | 정밀화 성공 → 더 좁은 임계 시도 |
| 1m success < 20% | 정책 한계 → reward 재설계(Phase 3) |
| 첫 jackpot 발화(<0.3m) | 정밀도 새 단계 → Phase 2 jackpot 활용 |
| fps 5~10 회복 | kill_infra 처방 검증 |

### 주의 — rolling success_rate carryover

resume 직후 ~30분간 `rollout/success_rate`가 0.97로 표시됨. 원인: SB3 `ep_info_buffer` rolling-100이 이전 episodes 포함. ~100 episode 후 자연 refresh → 그 후 실제 1m 추세 봐야 함.

---

## 결과 (TBD — v2 진행 중)

- 누적 target ~390k step, 진행 중
- 보존: `success_replay/za9zxdh6/`, `wandb/run-20260605_152711-za9zxdh6/`

---

## 관련 노트

- [[research/phase1_summary_junsang]] — Phase 1 전체 맥락
- [[research/sac_bounded_action_target_entropy_junsang]] — SAC 안정화 (Round 7 v3에서 계승)
- [[errors/err_20260528_gz_timeout_recurrence_junsang]] — #021 (kill_infra 관련)
- [[experiments/training_history]]
- [[daily/daily_2026-06-05_junsang]]
- [[00_index_junsang]]
- local: `design/model_history`, `meeting_notes/meeting_notes_2026-06-05`
